#!/usr/bin/env python3
"""
Teach egg-tray place slots without DART.

Keys:
  p: print current joints + Doosan current posx
  a: save current pose as slot_i above
  r: save current pose as slot_i release
  n: next slot
  b: previous slot
  w: write config/place_slots.yaml
  q: quit
"""

import os
import sys
import time
import json
import select
import termios
import tty
from datetime import datetime

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetCurrentPosx


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
POSX_REF = 0
def resolve_config_path():
    """개발 중에는 install 복사본보다 src/config를 단일 진실로 사용한다."""
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/place_slots.yaml"),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "place_slots.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


CONFIG_PATH = resolve_config_path()


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout=0.1):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        return sys.stdin.read(1)


class PlaceSlotTeacher(Node):
    def __init__(self):
        super().__init__("teach_place_slots")
        self.current_joints = None
        self.slot_idx = 0
        self.slots = self._load_existing()
        self.create_subscription(JointState, "/dsr01/joint_states", self.joint_cb, 10)
        self.cli_posx = self.create_client(GetCurrentPosx, "/dsr01/aux_control/get_current_posx")
        self.get_logger().info(f"Place slot teacher ready. YAML: {CONFIG_PATH}")
        self.print_help()

    def joint_cb(self, msg):
        jmap = {n: p for n, p in zip(msg.name, msg.position)}
        joints = [jmap.get(n) for n in JOINT_NAMES]
        if None not in joints:
            self.current_joints = joints

    def _load_existing(self):
        if not os.path.exists(CONFIG_PATH):
            return []
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data.get("slots", []) or []
        except Exception as e:
            self.get_logger().warn(f"Failed to load existing slots: {e}")
            return []

    def print_help(self):
        print("")
        print("Keys: p=print  a=save above  r=save release  n=next  b=prev  w=write  q=quit")
        print("Move the robot to a pose, then press a/r to capture controller posx and joints.")
        print("")

    def ensure_slot(self):
        while len(self.slots) <= self.slot_idx:
            idx = len(self.slots)
            self.slots.append({"name": f"slot{idx}", "above": None, "release": None})
        return self.slots[self.slot_idx]

    def get_posx(self):
        if not self.cli_posx.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/dsr01/aux_control/get_current_posx not available")
            return None
        req = GetCurrentPosx.Request()
        req.ref = POSX_REF
        future = self.cli_posx.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("get_current_posx timeout/failure")
            return None
        res = future.result()
        if not res.success:
            self.get_logger().error("get_current_posx returned success=False")
            return None
        if not res.task_pos_info:
            self.get_logger().error("get_current_posx returned empty task_pos_info")
            return None
        return [float(v) for v in res.task_pos_info[0].data]

    def snapshot(self):
        if self.current_joints is None:
            self.get_logger().warn("No /dsr01/joint_states yet")
            return None
        posx = self.get_posx()
        if posx is None:
            return None
        joints_deg = [float(v * 180.0 / 3.141592653589793) for v in self.current_joints]
        return {
            "joints_deg": joints_deg,
            "posx": posx,
            "ref": POSX_REF,
            "captured_at": datetime.now().isoformat(timespec="seconds"),
        }

    def print_current(self):
        snap = self.snapshot()
        if snap is None:
            return
        print(f"\nslot{self.slot_idx} current")
        print("  joints_deg:", json.dumps([round(v, 3) for v in snap["joints_deg"]]))
        print("  posx:", json.dumps([round(v, 3) for v in snap["posx"]]))

    def save_pose(self, kind):
        snap = self.snapshot()
        if snap is None:
            return
        slot = self.ensure_slot()
        slot[kind] = snap
        print(f"\nSaved {slot['name']} {kind}")
        print("  joints_deg:", json.dumps([round(v, 3) for v in snap["joints_deg"]]))
        print("  posx:", json.dumps([round(v, 3) for v in snap["posx"]]))
        self.write_yaml()

    def write_yaml(self):
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        data = {
            "generated_by": "teach_place_slots.py",
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "posx_service": "/dsr01/aux_control/get_current_posx",
            "slots": self.slots,
        }
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        print(f"\nWrote {CONFIG_PATH}")

    def handle_key(self, key):
        if key == "p":
            self.print_current()
        elif key == "a":
            self.save_pose("above")
        elif key == "r":
            self.save_pose("release")
        elif key == "n":
            self.slot_idx += 1
            self.ensure_slot()
            print(f"\nslot -> {self.slot_idx}")
        elif key == "b":
            self.slot_idx = max(0, self.slot_idx - 1)
            print(f"\nslot -> {self.slot_idx}")
        elif key == "w":
            self.write_yaml()
        elif key == "h":
            self.print_help()
        elif key == "q":
            return False
        return True


def main():
    rclpy.init()
    node = PlaceSlotTeacher()
    running = True
    try:
        with RawTerminal() as term:
            while rclpy.ok() and running:
                rclpy.spin_once(node, timeout_sec=0.02)
                key = term.read_key(0.05)
                if key:
                    running = node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
