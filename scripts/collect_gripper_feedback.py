#!/usr/bin/env python3
"""Collect repeated gripper position/current samples with one ground-truth label."""

import argparse
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


DEFAULT_ROOT = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/logs/gripper_calibration")


class FeedbackCollector(Node):
    def __init__(self):
        super().__init__("gripper_feedback_collector")
        self.client = self.create_client(Trigger, "/dsr01/gripper/read_state")

    def read(self, timeout_sec):
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            return None, "read_state service unavailable"
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            return None, "read_state timeout"
        response = future.result()
        try:
            state = json.loads(response.message)
        except json.JSONDecodeError:
            return None, f"invalid response: {response.message!r}"
        state["service_success"] = bool(response.success)
        return state, ""


def main():
    parser = argparse.ArgumentParser(
        description="그리퍼 양방향 position/current 보정 표본을 자동 수집합니다.")
    parser.add_argument(
        "--condition", required=True,
        choices=("empty", "stem", "leaf_or_non_target"))
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval-sec", type=float, default=0.5)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--notes", default="")
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.samples < 1:
        raise SystemExit("--samples must be >= 1")
    day_dir = os.path.join(DEFAULT_ROOT, datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(day_dir, exist_ok=True)
    output = os.path.abspath(os.path.expanduser(
        args.output or os.path.join(day_dir, "gripper_feedback.jsonl")))

    rclpy.init()
    node = FeedbackCollector()
    valid = 0
    try:
        with open(output, "a", encoding="utf-8") as stream:
            for index in range(args.samples):
                state, error = node.read(args.timeout_sec)
                record = {
                    "schema_version": "strawberry_gripper_feedback.v1",
                    "timestamp": datetime.now().astimezone().isoformat(
                        timespec="milliseconds"),
                    "condition": args.condition,
                    "sample_index": index + 1,
                    "notes": args.notes,
                    "state": state,
                    "error": error or None,
                }
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
                stream.flush()
                if state and state.get("service_success"):
                    valid += 1
                    print(
                        f"{index + 1}/{args.samples}: "
                        f"position={state.get('position')} "
                        f"current_raw={state.get('current_raw')}")
                else:
                    print(f"{index + 1}/{args.samples}: FAIL {error or state}")
                if index + 1 < args.samples:
                    time.sleep(max(0.0, args.interval_sec))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"saved: {output}")
    print(f"valid: {valid}/{args.samples}")


if __name__ == "__main__":
    main()
