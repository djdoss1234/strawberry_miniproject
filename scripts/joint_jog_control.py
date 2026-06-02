#!/usr/bin/env python3
"""
Keyboard joint jog controller for Doosan E0509.

Keys:
  1~6       select joint
  Up/Down   move selected joint +/- step degrees
  Left/Right step degrees down/up
  g         type 6 joint degrees and move
  p         print current joints
  h         help
  q         quit
"""

import sys
import time
import select
import termios
import tty
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetRobotMode, GetRobotState, MoveJoint


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
DEFAULT_VEL = 80.0
DEFAULT_ACC = 120.0
DEFAULT_STEP_DEG = 1.0
MIN_STEP_DEG = 0.1
MAX_STEP_DEG = 10.0
STEP_DEG_DELTA = 0.5
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout=0.05):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        ch = os.read(self.fd, 1).decode(errors="ignore")
        if ch == "\x1b":
            seq = ""
            end = time.time() + 0.08
            while time.time() < end:
                if select.select([sys.stdin], [], [], 0.005)[0]:
                    seq += os.read(self.fd, 1).decode(errors="ignore")
                elif seq:
                    break
            if seq.startswith("[") and len(seq) >= 2:
                return {"A": "up", "B": "down", "C": "right", "D": "left"}.get(seq[1], ch)
        return ch


class CookedInput:
    def __init__(self, raw_terminal):
        self.raw_terminal = raw_terminal

    def __enter__(self):
        termios.tcsetattr(self.raw_terminal.fd, termios.TCSADRAIN, self.raw_terminal.old)

    def __exit__(self, exc_type, exc, tb):
        tty.setcbreak(self.raw_terminal.fd)


class JointJogControl(Node):
    def __init__(self):
        super().__init__("joint_jog_control")
        self.current_joints_rad = None
        self.target_joints_deg = None
        self.selected_joint = 0
        self.vel = DEFAULT_VEL
        self.acc = DEFAULT_ACC
        self.step_deg = DEFAULT_STEP_DEG
        self.last_cmd_time = 0.0
        self.create_subscription(JointState, "/dsr01/joint_states", self.joint_cb, 10)
        self.cli_movej = self.create_client(MoveJoint, "/dsr01/motion/move_joint")
        self.cli_mode = self.create_client(GetRobotMode, "/dsr01/system/get_robot_mode")
        self.cli_state = self.create_client(GetRobotState, "/dsr01/system/get_robot_state")
        self.get_logger().info("Joint jog control ready")
        self.print_help()

    def joint_cb(self, msg):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in JOINT_NAMES]
        if None not in joints:
            self.current_joints_rad = joints
            if self.target_joints_deg is None:
                self.target_joints_deg = self.current_joints_deg()

    def current_joints_deg(self):
        if self.current_joints_rad is None:
            return None
        return [v * 180.0 / 3.141592653589793 for v in self.current_joints_rad]

    def print_help(self):
        print("")
        print("Joint Jog Control")
        print("  1~6       select joint")
        print("  Up/Down   move selected joint +/- step degrees")
        print("  Left/Right step degrees -/+")
        print("  w/s       move selected joint +/- step degrees (fallback)")
        print("  a/d       step degrees -/+ (fallback)")
        print("  g         input 6 joint degrees and move")
        print("  p         print current joints")
        print("  h         help")
        print("  q         quit")
        print("")

    def print_status(self):
        cur = self.current_joints_deg()
        if cur is None:
            print("No joint state yet")
            return
        print(
            f"J{self.selected_joint + 1} selected | step={self.step_deg:.1f}deg | "
            f"vel={self.vel:.1f} acc={self.acc:.1f} | "
            f"current={[round(v, 2) for v in cur]}"
        )

    def send_movej(self, joints_deg, sync_type=0, wait=True):
        if not self.cli_movej.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("/dsr01/motion/move_joint not available")
            return False
        req = MoveJoint.Request()
        req.pos = [float(v) for v in joints_deg]
        req.vel = float(self.vel)
        req.acc = float(self.acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = int(sync_type)
        future = self.cli_movej.call_async(req)
        self.last_cmd_time = time.time()
        if not wait:
            return True
        timeout_end = time.time() + 90.0
        while not future.done() and time.time() < timeout_end:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            self.get_logger().error("MoveJoint timeout")
            return False
        res = future.result()
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"MoveJoint failed: {res}")
            self.log_robot_status_hint()
        return ok

    def _call_simple(self, client, req, timeout=0.5):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(req)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result() if future.done() else None

    def log_robot_status_hint(self):
        mode = self._call_simple(self.cli_mode, GetRobotMode.Request())
        state = self._call_simple(self.cli_state, GetRobotState.Request())
        mode_text = "unknown"
        state_text = "unknown"
        if mode and mode.success:
            mode_names = {0: "MANUAL", 1: "AUTONOMOUS", 2: "MEASURE"}
            mode_text = f"{mode.robot_mode}({mode_names.get(mode.robot_mode, '?')})"
        if state and state.success:
            state_names = {
                0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
                4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP",
                7: "HOMING", 8: "RECOVERY", 9: "SAFE_STOP2",
                10: "SAFE_OFF2", 15: "NOT_READY",
            }
            state_text = f"{state.robot_state}({state_names.get(state.robot_state, '?')})"
        self.get_logger().warn(
            f"Robot status: mode={mode_text}, state={state_text}. "
            "MoveJoint usually needs AUTONOMOUS + STANDBY and no active motion/alarm."
        )

    def move_step(self, sign):
        cur = self.current_joints_deg()
        if cur is None:
            print("No joint state yet")
            return
        base = self.target_joints_deg if self.target_joints_deg is not None else cur
        target = list(base)
        target[self.selected_joint] += float(sign) * self.step_deg
        self.target_joints_deg = target
        direction = "+" if sign > 0 else "-"
        print(
            f"\rJ{self.selected_joint + 1} {direction}{self.step_deg:.1f}deg -> "
            f"{target[self.selected_joint]:.2f}deg",
            end="",
            flush=True,
        )
        if not self.send_movej(target, sync_type=0, wait=True):
            cur_after = self.current_joints_deg()
            if cur_after is not None:
                self.target_joints_deg = cur_after

    def change_step(self, delta):
        self.step_deg = max(MIN_STEP_DEG, min(MAX_STEP_DEG, self.step_deg + delta))
        print(f"step = {self.step_deg:.1f} deg")

    def goto_joints(self, text):
        parts = text.replace(",", " ").split()
        if len(parts) != 6:
            print("Need exactly 6 joint degrees")
            return
        try:
            joints = [float(v) for v in parts]
        except ValueError:
            print("Invalid number")
            return
        self.target_joints_deg = joints
        print(f"MoveJoint -> {[round(v, 2) for v in joints]}")
        self.send_movej(joints, sync_type=0, wait=True)

    def handle_key(self, key, raw_term):
        if key in ["1", "2", "3", "4", "5", "6"]:
            if self.current_joints_deg() is not None:
                self.target_joints_deg = self.current_joints_deg()
            self.selected_joint = int(key) - 1
            self.print_status()
        elif key in ("up", "w"):
            self.move_step(+1.0)
        elif key in ("down", "s"):
            self.move_step(-1.0)
        elif key in ("left", "a"):
            self.change_step(-STEP_DEG_DELTA)
        elif key in ("right", "d"):
            self.change_step(+STEP_DEG_DELTA)
        elif key == "p":
            self.print_status()
        elif key == "h":
            self.print_help()
        elif key == "g":
            with CookedInput(raw_term):
                text = input("\nEnter 6 joint degrees (comma/space separated): ")
            self.goto_joints(text)
        elif key == "q":
            return False
        return True


def main():
    rclpy.init()
    node = JointJogControl()
    running = True
    try:
        with RawTerminal() as term:
            while rclpy.ok() and running:
                rclpy.spin_once(node, timeout_sec=0.01)
                key = term.read_key(0.03)
                if key is not None:
                    running = node.handle_key(key, term)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
