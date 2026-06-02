#!/usr/bin/env python3
"""
Strawberry Seg+Pose Fusion Detection Node

Seg 모델(과실 마스크·익음도) + Pose 모델(줄기 3-키포인트)을 결합하여
ripe 딸기의 줄기 파지 위치를 계산하고 수확 후보를 퍼블리시합니다.

Fusion 규칙: Pose bbox 중심이 Seg 마스크 안에 있으면 같은 과실.
수확 후보: class=ripe + Pose 매칭 성공.

Published topics:
  /strawberry/detection/pick_pose      (PoseStamped)           — ripe 후보 1개씩
  /strawberry/detection/scene_positions (Float64MultiArray)    — 모든 ripe 중심 [x,y,z, ...]
"""

import os
import threading
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as ScipyR

# ── Joint names (Doosan E0509) ────────────────────────────────────────────────
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# ── Seg model class IDs ───────────────────────────────────────────────────────
RIPE_CLASS_ID = 0   # 0=ripe, 1=unripe, 2=sick

# ── Visualization ─────────────────────────────────────────────────────────────
COLOR_RIPE   = (0, 255, 0)
COLOR_UNRIPE = (0, 220, 220)
COLOR_SICK   = (0, 0, 220)
COLOR_MATCH  = (0, 255, 255)
COLOR_KP0    = (0, 140, 255)   # stem_base — orange
COLOR_KP1    = (0, 0, 255)     # stem_mid  — red
COLOR_KP2    = (0, 200, 0)     # stem_tip  — green
SEG_ALPHA    = 0.35


# ── E0509 FK (calibration-identical) ─────────────────────────────────────────
def _T(xyz, rpy, q=0.0):
    M = np.eye(4)
    M[:3, 3] = xyz
    R_fixed = ScipyR.from_euler('xyz', rpy).as_matrix()
    R_joint  = ScipyR.from_euler('z',   q  ).as_matrix()
    M[:3, :3] = R_fixed @ R_joint
    return M


def e0509_fk(q_rad):
    T = np.eye(4)
    T = T @ _T([0,      0,       0.2045], [0,         0,           0      ], q_rad[0])
    T = T @ _T([0,      0,       0     ], [0,        -np.pi/2,    -np.pi/2], q_rad[1])
    T = T @ _T([0.373,  0,       0     ], [0,         0,           np.pi/2], q_rad[2])
    T = T @ _T([0,     -0.373,   0     ], [np.pi/2,   0,           0      ], q_rad[3])
    T = T @ _T([0,      0,       0     ], [-np.pi/2,  0,           0      ], q_rad[4])
    T = T @ _T([0,     -0.1725,  0     ], [np.pi/2,   0,           0      ], q_rad[5])
    T = T @ _T([0,      0,       0     ], [np.pi,    -np.pi/2,     0      ])  # TCP fixed
    return T


def stem_vec_to_quat_xyzw(stem_vec_3d: np.ndarray) -> np.ndarray:
    """Quaternion [x,y,z,w] that aligns gripper Z-axis with stem direction.

    stem_vec_3d: KP2_3d - KP0_3d in base_link frame.
    Falls back to identity when vector is degenerate.
    NOTE: curobo_planner_node currently uses its own WALL_QUAT — this
    orientation is stored in pick_pose for future dynamic stem grasping.
    """
    v = stem_vec_3d.copy().astype(float)
    n = np.linalg.norm(v)
    if n < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    v /= n

    z_ref = np.array([0.0, 0.0, 1.0])
    axis  = np.cross(z_ref, v)
    axis_n = np.linalg.norm(axis)
    if axis_n < 1e-6:
        # Parallel → identity; Anti-parallel → 180° around Y
        return np.array([0.0, 0.0, 0.0, 1.0]) if np.dot(z_ref, v) > 0 \
               else np.array([0.0, 1.0, 0.0, 0.0])

    axis /= axis_n
    angle = np.arccos(np.clip(np.dot(z_ref, v), -1.0, 1.0))
    return ScipyR.from_rotvec(angle * axis).as_quat()  # [x,y,z,w]


# ─────────────────────────────────────────────────────────────────────────────
class StrawberryFusionNode(Node):

    def __init__(self):
        super().__init__("strawberry_fusion_node")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            "seg_model",
            "~/Downloads/share_yolo/share_yolo/strawberry_seg_best.pt")
        self.declare_parameter(
            "pose_model",
            "~/Downloads/share_yolo/share_yolo/strawberry_pose_best.pt")
        self.declare_parameter(
            "calib_npz",
            "~/doosan_ws/src/e0509_gripper_description/config/calibration_eye_in_hand_1.npz")
        self.declare_parameter("yolo_conf",    0.25)
        self.declare_parameter("kp_conf_min",  0.40)   # keypoint visibility threshold
        self.declare_parameter("infer_every",  3)       # run inference every N camera frames
        self.declare_parameter("stable_hits_required", 2)
        self.declare_parameter("track_match_distance_m", 0.050)
        self.declare_parameter("track_ttl_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.7)
        self.declare_parameter("show_display", True)

        seg_path   = os.path.expanduser(self.get_parameter("seg_model").value)
        pose_path  = os.path.expanduser(self.get_parameter("pose_model").value)
        calib_path = os.path.expanduser(self.get_parameter("calib_npz").value)
        self._conf    = self.get_parameter("yolo_conf").value
        self._kp_min  = self.get_parameter("kp_conf_min").value
        self._infer_n = max(1, self.get_parameter("infer_every").value)
        self._stable_hits = max(1, int(self.get_parameter("stable_hits_required").value))
        self._track_match_dist = float(self.get_parameter("track_match_distance_m").value)
        self._track_ttl_sec = float(self.get_parameter("track_ttl_sec").value)
        self._publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self._display = self.get_parameter("show_display").value

        # ── calibration ───────────────────────────────────────────────────────
        self.get_logger().info(f"Loading calibration: {calib_path}")
        calib = np.load(calib_path)
        self.T_cam2gripper = calib['T_cam_to_gripper']
        self.get_logger().info(
            f"T_cam2gripper translation(mm): {self.T_cam2gripper[:3,3]*1000}")

        # ── joint state ───────────────────────────────────────────────────────
        self.current_joints = None
        self._jlock = threading.Lock()

        # ── YOLO ──────────────────────────────────────────────────────────────
        self.get_logger().info(f"Loading seg model:  {seg_path}")
        self.seg_model = YOLO(seg_path)
        self.get_logger().info(f"Loading pose model: {pose_path}")
        self.pose_model = YOLO(pose_path)
        self.get_logger().info(
            f"Seg classes: {self.seg_model.names} | "
            f"Pose classes: {self.pose_model.names}")

        # ── RealSense ─────────────────────────────────────────────────────────
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense started.")

        # ── ROS2 I/O ──────────────────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/dsr01/joint_states", self._joint_cb, 10)
        self.pick_pub  = self.create_publisher(
            PoseStamped, "/strawberry/detection/pick_pose", 20)
        self.scene_pub = self.create_publisher(
            Float64MultiArray, "/strawberry/detection/scene_positions", 10)

        self._frame_n = 0
        self._tracks = {}
        self._next_track_id = 1
        self.timer = self.create_timer(1.0 / 30.0, self._loop)
        self.get_logger().info(
            "StrawberryFusionNode ready.  q=quit in display window")

    def _update_track(self, pos_base: np.ndarray, quat_xyzw: np.ndarray):
        """Simple 3-D nearest-neighbor tracker to suppress frame flicker."""
        now = time.monotonic()
        stale_ids = [
            tid for tid, track in self._tracks.items()
            if now - track["last_seen"] > self._track_ttl_sec
        ]
        for tid in stale_ids:
            del self._tracks[tid]

        best_id = None
        best_dist = float("inf")
        for tid, track in self._tracks.items():
            dist = float(np.linalg.norm(pos_base - track["pos"]))
            if dist < best_dist:
                best_id = tid
                best_dist = dist

        if best_id is None or best_dist > self._track_match_dist:
            best_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[best_id] = {
                "pos": pos_base.astype(float),
                "quat": quat_xyzw.astype(float),
                "hits": 0,
                "last_seen": now,
                "last_pub": 0.0,
            }

        track = self._tracks[best_id]
        alpha = 0.55
        track["pos"] = alpha * pos_base + (1.0 - alpha) * track["pos"]
        track["quat"] = quat_xyzw.astype(float)
        track["hits"] += 1
        track["last_seen"] = now
        return best_id, track

    def _should_publish_track(self, track) -> bool:
        now = time.monotonic()
        if track["hits"] < self._stable_hits:
            return False
        if now - track["last_pub"] < self._publish_period_sec:
            return False
        track["last_pub"] = now
        return True

    # ── joint callback ────────────────────────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            with self._jlock:
                self.current_joints = [jmap[n] for n in JOINT_NAMES]
        except KeyError:
            pass

    # ── coordinate transform ──────────────────────────────────────────────────
    def _cam_to_base(self, pt_cam_xyz):
        with self._jlock:
            joints = self.current_joints
        if joints is None:
            return None
        T_g2b   = e0509_fk(joints)
        T_total = T_g2b @ self.T_cam2gripper
        return (T_total @ np.array([*pt_cam_xyz, 1.0]))[:3]

    # ── depth helpers ─────────────────────────────────────────────────────────
    def _depth_at_px(self, depth_frame, u, v, radius=4) -> float | None:
        """Median depth in a small region around pixel (u, v)."""
        fw, fh = depth_frame.get_width(), depth_frame.get_height()
        samples = []
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                pu, pv = int(u) + du, int(v) + dv
                if 0 <= pu < fw and 0 <= pv < fh:
                    d = depth_frame.get_distance(pu, pv)
                    if 0.05 < d < 3.0:
                        samples.append(d)
        return float(np.median(samples)) if samples else None

    def _px_to_3d(self, depth_frame, intr, u, v, radius=4):
        """Return 3-D point in base_link, or None on failure."""
        d = self._depth_at_px(depth_frame, u, v, radius)
        if d is None:
            return None
        pt_cam = rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], d)
        return self._cam_to_base(pt_cam)

    def _polygon_centroid_3d(self, depth_frame, intr, polygon):
        """3-D centroid of a seg polygon (via moment centroid pixel)."""
        if len(polygon) < 3:
            return None
        M = cv2.moments(polygon.astype(np.float32))
        if M["m00"] < 1e-6:
            return None
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        return self._px_to_3d(depth_frame, intr, cx, cy)

    # ── main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        try:
            frames  = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                return

            self._frame_n += 1
            img  = np.asanyarray(color_f.get_data())
            intr = depth_f.profile.as_video_stream_profile().intrinsics
            vis  = img.copy()

            # Joint state guard
            with self._jlock:
                joint_ok = self.current_joints is not None
            if not joint_ok:
                cv2.putText(vis, "NO JOINT STATE — waiting",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self._show(vis)
                return

            # Throttle YOLO inference for performance
            if self._frame_n % self._infer_n != 0:
                self._show(vis)
                return

            # ── YOLO inference ────────────────────────────────────────────────
            seg_res  = self.seg_model(img,  conf=self._conf, verbose=False)[0]
            pose_res = self.pose_model(img, conf=self._conf, verbose=False)[0]

            # ── parse seg detections ──────────────────────────────────────────
            # seg_items: list of (cls_id, polygon_np)
            seg_items = []
            if seg_res.masks is not None and seg_res.boxes is not None:
                for i, polygon in enumerate(seg_res.masks.xy):
                    cls_id = int(seg_res.boxes.cls[i].item())
                    seg_items.append((cls_id, polygon))

            # ── parse pose detections ─────────────────────────────────────────
            # pose_items: list of (bbox_xyxy, kps_np(3,3))
            pose_items = []
            if pose_res.keypoints is not None and pose_res.boxes is not None:
                for i, kps in enumerate(pose_res.keypoints.data):
                    bbox   = pose_res.boxes.xyxy[i].cpu().numpy()
                    kps_np = kps.cpu().numpy()   # (3, 3): [[x,y,conf], ...]
                    pose_items.append((bbox, kps_np))

            # ── draw seg overlays ─────────────────────────────────────────────
            cls_color = {0: COLOR_RIPE, 1: COLOR_UNRIPE, 2: COLOR_SICK}
            overlay   = vis.copy()
            for cls_id, polygon in seg_items:
                if len(polygon) >= 3:
                    color = cls_color.get(cls_id, (200, 200, 200))
                    cv2.fillPoly(overlay, [polygon.astype(np.int32)], color)
            cv2.addWeighted(overlay, SEG_ALPHA, vis, 1 - SEG_ALPHA, 0, vis)
            for cls_id, polygon in seg_items:
                if len(polygon) >= 3:
                    cv2.polylines(vis, [polygon.astype(np.int32)],
                                  True, cls_color.get(cls_id, (200, 200, 200)), 1)

            # ── scene positions: all ripe fruit centroids (for obstacle avoidance) ──
            scene_flat = []
            for cls_id, polygon in seg_items:
                if cls_id == RIPE_CLASS_ID and len(polygon) >= 3:
                    pt3d = self._polygon_centroid_3d(depth_f, intr, polygon)
                    if pt3d is not None:
                        scene_flat.extend([float(pt3d[0]), float(pt3d[1]), float(pt3d[2])])
            if scene_flat:
                smsg = Float64MultiArray()
                smsg.data = scene_flat
                self.scene_pub.publish(smsg)

            # ── fusion: match each pose detection to a seg mask ───────────────
            kp_colors = [COLOR_KP0, COLOR_KP1, COLOR_KP2]
            kp_labels = ["KP0", "KP1", "KP2"]

            for pose_bbox, kps_np in pose_items:
                pose_cx = (pose_bbox[0] + pose_bbox[2]) / 2.0
                pose_cy = (pose_bbox[1] + pose_bbox[3]) / 2.0

                # Draw pose bounding box
                cv2.rectangle(vis,
                    (int(pose_bbox[0]), int(pose_bbox[1])),
                    (int(pose_bbox[2]), int(pose_bbox[3])),
                    (200, 200, 0), 1)

                # Draw visible keypoints
                for ki in range(min(3, len(kps_np))):
                    kx, ky, kconf = kps_np[ki]
                    if kconf >= self._kp_min:
                        cv2.circle(vis, (int(kx), int(ky)), 5, kp_colors[ki], -1)
                        cv2.putText(vis, kp_labels[ki],
                                    (int(kx) + 6, int(ky) - 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, kp_colors[ki], 1)

                # Find which seg mask contains this pose bbox center
                matched_cls = None
                for cls_id, polygon in seg_items:
                    if len(polygon) < 3:
                        continue
                    inside = cv2.pointPolygonTest(
                        polygon.astype(np.float32),
                        (float(pose_cx), float(pose_cy)), False)
                    if inside >= 0:
                        matched_cls = cls_id
                        break

                if matched_cls is None:
                    cv2.putText(vis, "no-seg",
                                (int(pose_cx), int(pose_cy) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 120, 120), 1)
                    continue

                cls_str = {0: "ripe", 1: "unripe", 2: "sick"}.get(matched_cls, str(matched_cls))
                cv2.putText(vis, f"[{cls_str}]",
                            (int(pose_bbox[0]), int(pose_bbox[1]) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            cls_color.get(matched_cls, (200, 200, 200)), 1)

                if matched_cls != RIPE_CLASS_ID:
                    continue  # only harvest ripe

                # ── compute 3D keypoint positions ─────────────────────────────
                # KP0 = stem_base (grasp target — nearest to fruit)
                # KP1 = stem_mid
                # KP2 = stem_tip (direction reference)
                kp3d = {}
                for ki in range(min(3, len(kps_np))):
                    kx, ky, kconf = kps_np[ki]
                    if kconf >= self._kp_min:
                        pt3d = self._px_to_3d(depth_f, intr, kx, ky)
                        if pt3d is not None:
                            kp3d[ki] = pt3d

                # Grasp position: prefer KP0 (stem_base), fallback KP1
                if 0 in kp3d:
                    grasp_pt = kp3d[0]
                elif 1 in kp3d:
                    grasp_pt = kp3d[1]
                else:
                    cv2.putText(vis, "no-depth",
                                (int(pose_cx), int(pose_cy) + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 220), 1)
                    continue

                # Stem direction: KP2 - KP0 (or KP2 - KP1 as fallback)
                if 2 in kp3d and 0 in kp3d:
                    stem_vec = kp3d[2] - kp3d[0]
                elif 2 in kp3d and 1 in kp3d:
                    stem_vec = kp3d[2] - kp3d[1]
                else:
                    stem_vec = None

                quat_xyzw = (stem_vec_to_quat_xyzw(stem_vec)
                             if stem_vec is not None
                             else np.array([0.0, 0.0, 0.0, 1.0]))

                track_id, track = self._update_track(grasp_pt, quat_xyzw)
                stable = track["hits"] >= self._stable_hits
                if self._should_publish_track(track):
                    # ── publish pick pose ─────────────────────────────────────
                    pmsg = PoseStamped()
                    pmsg.header.frame_id    = "base_link"
                    pmsg.header.stamp       = self.get_clock().now().to_msg()
                    pmsg.pose.position.x    = float(track["pos"][0])
                    pmsg.pose.position.y    = float(track["pos"][1])
                    pmsg.pose.position.z    = float(track["pos"][2])
                    pmsg.pose.orientation.x = float(track["quat"][0])
                    pmsg.pose.orientation.y = float(track["quat"][1])
                    pmsg.pose.orientation.z = float(track["quat"][2])
                    pmsg.pose.orientation.w = float(track["quat"][3])
                    self.pick_pub.publish(pmsg)

                gx, gy, gz = track["pos"]
                label = "PICK" if stable else "WAIT"
                cv2.putText(vis,
                    f"{label}#{track_id} h={track['hits']} ({gx:.3f},{gy:.3f},{gz:.3f})",
                    (int(pose_bbox[0]), int(pose_bbox[3]) + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_MATCH, 1)

                # Draw stem direction arrow (KP0 → KP2 in image)
                if stem_vec is not None and 0 in kp3d and 2 in kp3d:
                    kp0_px = (int(kps_np[0][0]), int(kps_np[0][1]))
                    kp2_px = (int(kps_np[2][0]), int(kps_np[2][1]))
                    cv2.arrowedLine(vis, kp0_px, kp2_px,
                                    (0, 200, 255), 2, tipLength=0.3)

            # ── HUD ──────────────────────────────────────────────────────────
            n_ripe = sum(1 for c, _ in seg_items if c == RIPE_CLASS_ID)
            cv2.putText(vis,
                f"seg_ripe={n_ripe}  pose_det={len(pose_items)}  frame={self._frame_n}",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(vis, "q: quit",
                        (10, 458), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1)

            self._show(vis)

        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")
            import traceback
            traceback.print_exc()

    def _show(self, vis):
        if not self._display:
            return
        cv2.imshow("Fusion Detection", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise SystemExit


def main():
    rclpy.init()
    node = StrawberryFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
