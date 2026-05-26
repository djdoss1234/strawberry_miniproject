#!/usr/bin/env python3
"""
ArUco Marker Tracking → cuRobo Pipeline Node

RealSense depth 카메라로 ArUco 마커를 인식하고,
캘리브레이션 변환(T_cam_to_gripper) + FK로 base_link 좌표 계산 후
cuRobo planner에 목표 pose를 전송합니다.

[수정사항]
  - calib['T_cam_to_base'] → calib['T_cam_to_gripper'] (KeyError 수정)
  - R @ tc + t 단순 변환 → T_cam2gripper + FK(joint_states) 파이프라인으로 교체
  - /dsr01/joint_states 구독 추가
  - 화면에 조인트 상태 수신 여부 표시 추가

Usage:
    ros2 run e0509_gripper_description marker_tracking_node.py

    # 파라미터 지정
    ros2 run e0509_gripper_description marker_tracking_node.py \
        --ros-args \
        -p calibration_path:=~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand__1_.npz \
        -p marker_id:=0 \
        -p marker_size:=0.1 \
        -p safe_z_offset:=0.15

Keys:
    s: 현재 마커 위치 위로 이동 (safe_z_offset 적용)
    p: 마커 위치로 pick 명령 전송
    q: 종료
"""

import os
import numpy as np
import cv2
from cv2 import aruco
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor


# ── E0509 DH 파라미터 (FK용) ───────────────────────────────────────────────────
# [a, d, alpha, theta_offset]  단위: m, rad
E0509_DH = [
    [0.0,    0.1555, np.pi/2,  0.0],
    [0.409,  0.0,    0.0,      0.0],
    [0.0,    0.0,    np.pi/2,  np.pi/2],
    [0.0,    0.402, -np.pi/2,  0.0],
    [0.0,    0.0,    np.pi/2,  0.0],
    [0.0,    0.082,  0.0,      0.0],
]


def dh_matrix(a, d, alpha, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1],
    ])


def fk_gripper_to_base(joint_angles_rad):
    """
    joint_angles_rad: [j1..j6] (rad)
    반환: T_gripper_to_base (4x4)
    """
    T = np.eye(4)
    for i, (a, d, alpha, offset) in enumerate(E0509_DH):
        T = T @ dh_matrix(a, d, alpha, joint_angles_rad[i] + offset)
    return T


class MarkerTrackingNode(Node):

    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def __init__(self):
        super().__init__("marker_tracking_node")

        # ── 파라미터 ────────────────────────────────────────────────────────────
        self.declare_parameter(
            "calibration_path",
            os.path.expanduser(
                "~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand__1_.npz"
            ),
            ParameterDescriptor(description="Path to calibration .npz file"))
        self.declare_parameter("marker_id", 0,
            ParameterDescriptor(description="ArUco marker ID to track"))
        self.declare_parameter("marker_size", 0.1,
            ParameterDescriptor(description="Marker size in meters"))
        self.declare_parameter("safe_z_offset", 0.15,
            ParameterDescriptor(description="Safety height offset above marker (meters)"))
        self.declare_parameter("target_topic", "/dsr01/curobo/target_pose",
            ParameterDescriptor(description="Target pose topic for cuRobo planner"))
        self.declare_parameter("auto_send", False,
            ParameterDescriptor(description="Automatically send target on detection"))

        calib_path      = self.get_parameter("calibration_path").value
        self.marker_id  = self.get_parameter("marker_id").value
        self.marker_size= self.get_parameter("marker_size").value
        self.safe_z_offset = self.get_parameter("safe_z_offset").value
        target_topic    = self.get_parameter("target_topic").value
        self.auto_send  = self.get_parameter("auto_send").value

        # ── 캘리브레이션 로드 ────────────────────────────────────────────────────
        self.get_logger().info(f"Loading calibration: {calib_path}")
        try:
            calib = np.load(os.path.expanduser(calib_path))
            # ★ 수정: T_cam_to_base → T_cam_to_gripper
            self.T_cam2gripper = calib['T_cam_to_gripper']  # (4,4)
            self.get_logger().info(
                f"Calibration loaded. cam→gripper translation (mm): "
                f"{self.T_cam2gripper[:3,3]*1000}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise

        # ── 조인트 상태 ──────────────────────────────────────────────────────────
        self.current_joints = None  # [j1..j6] rad

        # ── RealSense ────────────────────────────────────────────────────────────
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense started (color + depth)")

        # ── ArUco 검출기 ─────────────────────────────────────────────────────────
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_6X6_50),
            aruco.DetectorParameters())

        # ── ROS2 인터페이스 ──────────────────────────────────────────────────────
        # ★ 추가: 조인트 상태 구독 (FK 계산에 필요)
        self.joint_sub = self.create_subscription(
            JointState, "/dsr01/joint_states", self.joint_cb, 10)

        self.pub = self.create_publisher(PoseStamped, target_topic, 10)
        self.pick_pub = self.create_publisher(PoseStamped, "/dsr01/curobo/pick_pose", 10)

        # EE orientation (그리퍼 아래 방향)
        self.ee_quat = [0.7071, 0.7071, 0.0, 0.0]  # xyzw

        # 상태
        self.last_pos = None  # base_link 기준 마커 위치 (m)

        self.timer = self.create_timer(1.0 / 30.0, self.camera_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("  Marker Tracking Node Ready")
        self.get_logger().info(f"  Marker ID  : {self.marker_id}")
        self.get_logger().info(f"  Marker size: {self.marker_size}m")
        self.get_logger().info(f"  Safe Z offset: {self.safe_z_offset}m")
        self.get_logger().info(f"  Target topic : {target_topic}")
        self.get_logger().info("  Keys: 's'=move  'p'=pick  'q'=quit")
        self.get_logger().info("=" * 50)

    # ── 조인트 콜백 ──────────────────────────────────────────────────────────────
    def joint_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            self.current_joints = [jmap[n] for n in self.JOINT_NAMES]
        except KeyError:
            pass

    # ── 카메라 좌표 → base_link 변환 ─────────────────────────────────────────────
    def cam_to_base(self, pt_cam_xyz):
        """
        pt_cam_xyz : [x, y, z] camera frame (m)
        반환       : np.array [x, y, z] base_link frame (m)
                     조인트 상태 미수신 시 None
        """
        if self.current_joints is None:
            return None

        pt_h = np.array([*pt_cam_xyz, 1.0])

        # Step 1: camera → gripper  (캘리브레이션)
        pt_gripper_h = self.T_cam2gripper @ pt_h

        # Step 2: gripper → base_link  (FK)
        T_gripper2base = fk_gripper_to_base(self.current_joints)
        pt_base_h = T_gripper2base @ pt_gripper_h

        return pt_base_h[:3]

    # ── 카메라 루프 ──────────────────────────────────────────────────────────────
    def camera_loop(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf:
            return

        img = np.asanyarray(cf.get_data())
        display = img.copy()

        # 조인트 상태 표시
        if self.current_joints is not None:
            cv2.putText(display, "Joint: OK", (470, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 1)
        else:
            cv2.putText(display, "Joint: WAITING", (440, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        corners, ids, _ = self.detector.detectMarkers(img)

        if ids is not None and self.marker_id in ids.flatten():
            idx = list(ids.flatten()).index(self.marker_id)
            aruco.drawDetectedMarkers(display, corners, ids)
            center = corners[idx][0].mean(axis=0)
            cx, cy = int(center[0]), int(center[1])

            # depth: 중심 주변 패치 median (노이즈 보정)
            depth_m = self._depth_patch(df, cx, cy)

            if depth_m > 0.05:
                intr = df.profile.as_video_stream_profile().intrinsics
                pt_cam = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth_m)

                # ★ 수정: cam_to_base() 로 변환 (T_cam2gripper + FK)
                pb = self.cam_to_base(pt_cam)

                if pb is not None:
                    self.last_pos = pb
                    cv2.putText(display,
                        f"BASE  X:{pb[0]*1000:.1f}  Y:{pb[1]*1000:.1f}  Z:{pb[2]*1000:.1f} mm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(display,
                        f"depth: {depth_m*100:.1f} cm",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    if self.auto_send:
                        self.send_target()
                else:
                    cv2.putText(display, "Waiting for joint state...",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                cv2.putText(display, "Marker found — no depth",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(display, "No marker detected",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, "'s'=move  'p'=pick  'q'=quit",
            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Marker Tracking", display)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('s') and self.last_pos is not None:
            self.send_target()
        elif k == ord('p') and self.last_pos is not None:
            self.send_pick()
        elif k == ord('q'):
            self.get_logger().info("Shutting down...")
            raise SystemExit

    # ── depth 패치 median ────────────────────────────────────────────────────────
    def _depth_patch(self, df, cx, cy, half=5):
        vals = []
        for du in range(-half, half+1):
            for dv in range(-half, half+1):
                u, v = cx+du, cy+dv
                if 0 <= u < 640 and 0 <= v < 480:
                    d = df.get_distance(u, v)
                    if 0.05 < d < 5.0:
                        vals.append(d)
        return float(np.median(vals)) if vals else 0.0

    # ── 목표 전송 (safe_z_offset 적용) ───────────────────────────────────────────
    def send_target(self):
        if self.last_pos is None:
            return
        target_z = max(self.last_pos[2] + self.safe_z_offset, 0.15)

        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.last_pos[0])
        msg.pose.position.y = float(self.last_pos[1])
        msg.pose.position.z = float(target_z)
        msg.pose.orientation.x = float(self.ee_quat[0])
        msg.pose.orientation.y = float(self.ee_quat[1])
        msg.pose.orientation.z = float(self.ee_quat[2])
        msg.pose.orientation.w = float(self.ee_quat[3])
        self.pub.publish(msg)

        self.get_logger().info(
            f"[MOVE] X={self.last_pos[0]*1000:.1f}  "
            f"Y={self.last_pos[1]*1000:.1f}  "
            f"Z={target_z*1000:.1f} mm")

    # ── pick 전송 ────────────────────────────────────────────────────────────────
    def send_pick(self):
        if self.last_pos is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.last_pos[0])
        msg.pose.position.y = float(self.last_pos[1])
        msg.pose.position.z = float(self.last_pos[2])
        msg.pose.orientation.x = float(self.ee_quat[0])
        msg.pose.orientation.y = float(self.ee_quat[1])
        msg.pose.orientation.z = float(self.ee_quat[2])
        msg.pose.orientation.w = float(self.ee_quat[3])
        self.pick_pub.publish(msg)

        self.get_logger().info(
            f"[PICK] X={self.last_pos[0]*1000:.1f}  "
            f"Y={self.last_pos[1]*1000:.1f}  "
            f"Z={self.last_pos[2]*1000:.1f} mm")

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()