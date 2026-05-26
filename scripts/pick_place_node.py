#!/usr/bin/env python3
"""
Pick & Place Node — 벽면 딸기 수확
CuRobo 없이 Doosan 네이티브 MoveJ + MoveL 사용.

시퀀스:
  1. Open gripper
  2. (선택) MoveJ → ready pose
  3. MoveL → approach (벽 앞 approach_offset_m)
  4. MoveL → grasp   (벽 앞 grasp_offset_m, 저속)
  5. Close gripper
  6. MoveL → retreat (벽 앞 retreat_offset_m)

Subscribe:
  /dsr01/curobo/pick_pose  (PoseStamped, base_link, strawberry_yolo_node 호환)
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveJointx

# 벽면 접근 orientation (티치펜던트 실측값)
WALL_RX, WALL_RY, WALL_RZ = 148.23, 90.94, -90.0


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_place_node")

        # ── 파라미터 ──────────────────────────────────────────────────────────
        self.declare_parameter("approach_offset_m", 0.15)   # 접근: 딸기 앞 15cm
        self.declare_parameter("grasp_offset_m",   -0.02)   # 파지: 딸기 앞 3cm
        self.declare_parameter("retreat_offset_m",  0.20)   # 후퇴: 딸기 앞 20cm
        self.declare_parameter("min_z_m",           0.05)   # 최소 Z (바닥 보호)
        self.declare_parameter("vel_approach",      50.0)   # mm/s
        self.declare_parameter("vel_grasp",         20.0)   # mm/s (저속)
        self.declare_parameter("vel_retreat",       80.0)   # mm/s
        self.declare_parameter("vel_rot",           30.0)   # deg/s
        # ready pose: True로 켜면 approach 전에 MoveJ로 안전 자세 먼저 이동
        # ready_joints_deg는 실제 로봇 환경에 맞게 조정 필요
        self.declare_parameter("use_ready_pose",    False)
        self.declare_parameter("ready_joints_deg",  [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

        # ── 콜백 그룹 (서비스 호출 deadlock 방지) ────────────────────────────
        cb = rclpy.callback_groups.ReentrantCallbackGroup()

        # ── 구독 ──────────────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, "/dsr01/curobo/pick_pose",
            self.pick_cb, 10, callback_group=cb)

        # ── 서비스 클라이언트 ─────────────────────────────────────────────────
        self.cli_movej  = self.create_client(MoveJoint,  "/dsr01/motion/move_joint",  callback_group=cb)
        self.cli_movejx = self.create_client(MoveJointx, "/dsr01/motion/move_jointx", callback_group=cb)
        self.cli_movel  = self.create_client(MoveLine,   "/dsr01/motion/move_line",   callback_group=cb)
        self.cli_open  = self.create_client(Trigger,   "/dsr01/gripper/open",      callback_group=cb)
        self.cli_close = self.create_client(Trigger,   "/dsr01/gripper/close",     callback_group=cb)

        self._busy = False

        self.get_logger().info("Pick & Place Node ready (CuRobo-free)")
        self.get_logger().info("  Subscribe: /dsr01/curobo/pick_pose")

    # ── pick 콜백 ─────────────────────────────────────────────────────────────

    def pick_cb(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn("Pick 진행 중, 무시합니다.")
            return
        self._busy = True
        try:
            self._pick(msg.pose.position)
        finally:
            self._busy = False

    def _pick(self, pos):
        ao    = self.get_parameter("approach_offset_m").value
        go    = self.get_parameter("grasp_offset_m").value
        ro    = self.get_parameter("retreat_offset_m").value
        min_z = self.get_parameter("min_z_m").value
        v_ap  = self.get_parameter("vel_approach").value
        v_gr  = self.get_parameter("vel_grasp").value
        v_rt  = self.get_parameter("vel_retreat").value

        tx = pos.x
        ty = pos.y
        tz = max(pos.z, min_z)

        self.get_logger().info(
            f"=== PICK 시작: ({tx*1000:.1f}, {ty*1000:.1f}, {tz*1000:.1f}) mm ===")

        # Step 1: 그리퍼 열기
        self._step(1, "그리퍼 열기")
        self._trigger(self.cli_open)
        time.sleep(0.8)

        # Step 2: ready pose (선택)
        if self.get_parameter("use_ready_pose").value:
            joints = self.get_parameter("ready_joints_deg").value
            self._step(2, f"MoveJ ready {[f'{j:.1f}' for j in joints]} deg")
            if not self._movej(joints):
                self.get_logger().error("Ready pose 실패 — pick 중단")
                return
            time.sleep(0.3)

        # Step 3: approach — MoveJX (joint 보간, IK flip 없음)
        ax = tx + ao
        self._step(3, f"MoveJX approach ({ax*1000:.1f}, {ty*1000:.1f}, {tz*1000:.1f}) mm")
        if not self._movejx(ax, ty, tz, vel=v_ap):
            self.get_logger().error("Approach 실패 — pick 중단")
            return
        time.sleep(0.3)

        # Step 4: grasp advance — MoveJX (joint 보간, 긴 MoveL 대신)
        gx = tx + go
        self._step(4, f"MoveJX grasp  ({gx*1000:.1f}, {ty*1000:.1f}, {tz*1000:.1f}) mm")
        if not self._movejx(gx, ty, tz, vel=v_gr):
            self.get_logger().error("Grasp advance 실패 — pick 중단")
            return
        time.sleep(0.3)

        # Step 5: 그리퍼 닫기
        self._step(5, "그리퍼 닫기")
        self._trigger(self.cli_close)
        time.sleep(1.2)

        # Step 6: retreat
        rx = tx + ro
        self._step(6, f"MoveL retreat ({rx*1000:.1f}, {ty*1000:.1f}, {tz*1000:.1f}) mm")
        self._movel(rx, ty, tz, vel=v_rt)
        time.sleep(0.3)

        self.get_logger().info("=== PICK 완료 ===")

    # ── Doosan 서비스 래퍼 ────────────────────────────────────────────────────

    def _movejx(self, x_m, y_m, z_m, vel=50.0, timeout=20.0) -> bool:
        """MoveJointX: joint 보간으로 TCP 이동 (IK flip 없음, approach용)"""
        if not self.cli_movejx.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJointX 서비스 없음")
            return False
        req = MoveJointx.Request()
        req.pos        = [x_m * 1000, y_m * 1000, z_m * 1000, WALL_RX, WALL_RY, WALL_RZ]
        req.vel        = vel
        req.acc        = vel * 2.0
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0   # base_link
        req.mode       = 0   # absolute
        req.blend_type = 0
        req.sol        = 0   # auto solution space
        req.sync_type  = 0   # sync
        return self._call(self.cli_movejx, req, timeout)

    def _movel(self, x_m, y_m, z_m, vel=50.0, timeout=20.0) -> bool:
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine 서비스 없음")
            return False
        v_rot = self.get_parameter("vel_rot").value
        req = MoveLine.Request()
        req.pos        = [x_m * 1000, y_m * 1000, z_m * 1000, WALL_RX, WALL_RY, WALL_RZ]
        req.vel        = [vel, v_rot]
        req.acc        = [vel * 2.0, v_rot * 2.0]
        req.time       = 0.0
        req.ref        = 0   # base_link
        req.mode       = 0   # absolute
        req.blend_type = 0
        req.sync_type  = 0   # sync (모션 완료 후 리턴)
        return self._call(self.cli_movel, req, timeout)

    def _movej(self, joints_deg, vel=30.0, acc=60.0, timeout=20.0) -> bool:
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint 서비스 없음")
            return False
        req = MoveJoint.Request()
        req.pos        = list(joints_deg)
        req.vel        = vel
        req.acc        = acc
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0   # absolute
        req.blend_type = 0
        req.sync_type  = 1   # async
        return self._call(self.cli_movej, req, timeout)

    def _trigger(self, client, timeout=10.0) -> bool:
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Trigger 서비스 없음")
            return False
        return self._call(client, Trigger.Request(), timeout)

    def _call(self, client, req, timeout) -> bool:
        future = client.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error("서비스 타임아웃")
            return False
        result = future.result()
        if result is None:
            self.get_logger().error("서비스 응답 없음")
            return False
        success = getattr(result, "success", True)
        if not success:
            self.get_logger().error(f"서비스 실패: {getattr(result, 'message', '')}")
        return bool(success)

    def _step(self, n, msg):
        self.get_logger().info(f"  Step {n}: {msg}")


def main():
    rclpy.init()
    node = PickPlaceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
