#!/usr/bin/env python3
"""Run one explicitly armed SafeGrasp trial and save action feedback/result."""

import argparse
import json
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dsr_gripper_tcp_interfaces.action import SafeGrasp


LOG_ROOT = Path(os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/logs/gripper_calibration"))


def _state_dict(state):
    return {
        "ready": state.ready,
        "torque_enabled": state.torque_enabled,
        "moving": state.moving,
        "in_position": state.in_position,
        "grasp_detected": state.grasp_detected,
        "object_lost": state.object_lost,
        "present_position": state.present_position,
        "goal_position": state.goal_position,
        "present_current": state.present_current,
        "current_limit": state.current_limit,
        "present_velocity": state.present_velocity,
        "present_temperature": state.present_temperature,
        "status_text": state.status_text,
    }


class SafeGraspTrial(Node):
    def __init__(self, action_name):
        super().__init__("safe_grasp_trial_logger")
        self.client = ActionClient(self, SafeGrasp, action_name)
        self.feedback = []

    def on_feedback(self, message):
        feedback = message.feedback
        sample = {
            "timestamp": datetime.now().astimezone().isoformat(
                timespec="milliseconds"),
            "present_position": feedback.present_position,
            "present_current": feedback.present_current,
            "current_delta": feedback.current_delta,
            "grasp_detected": feedback.grasp_detected,
            "object_lost": feedback.object_lost,
            "state": _state_dict(feedback.state),
        }
        self.feedback.append(sample)
        print(
            f"position={feedback.present_position} "
            f"current={feedback.present_current} "
            f"delta={feedback.current_delta} "
            f"detected={feedback.grasp_detected} lost={feedback.object_lost}")

    def run(self, goal, server_timeout_sec):
        if not self.client.wait_for_server(timeout_sec=server_timeout_sec):
            raise RuntimeError("SafeGrasp action server unavailable")
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("SafeGrasp goal rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def main():
    parser = argparse.ArgumentParser(
        description="SafeGrasp 단일 보정 시험을 실행하고 JSONL로 기록합니다.")
    parser.add_argument(
        "--condition", required=True,
        choices=("empty", "stem", "leaf_or_non_target"))
    parser.add_argument("--target-position", type=int, default=700)
    parser.add_argument("--max-current", type=int, default=400)
    parser.add_argument("--current-delta-threshold", type=int, default=120)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--server-timeout-sec", type=float, default=3.0)
    parser.add_argument("--action-name", default="/gripper_service/safe_grasp")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output")
    parser.add_argument(
        "--execute", action="store_true",
        help="실제 그리퍼 명령을 허용합니다. 없으면 명령을 보내지 않습니다.")
    args = parser.parse_args()

    if not args.execute:
        raise SystemExit(
            "DRY RUN: 실제 명령을 보내지 않았습니다. 기존 그리퍼 노드 종료와 "
            "빈 공간을 확인한 뒤 --execute를 추가하세요.")

    day_dir = LOG_ROOT / datetime.now().strftime("%Y-%m-%d")
    day_dir.mkdir(parents=True, exist_ok=True)
    output = Path(os.path.expanduser(
        args.output or str(day_dir / "safe_grasp_trials.jsonl"))).resolve()

    goal = SafeGrasp.Goal()
    goal.target_position = args.target_position
    goal.max_current = args.max_current
    goal.current_delta_threshold = args.current_delta_threshold
    goal.timeout_sec = args.timeout_sec

    rclpy.init()
    node = SafeGraspTrial(args.action_name)
    try:
        wrapped = node.run(goal, args.server_timeout_sec)
        result = wrapped.result
        record = {
            "schema_version": "strawberry_safe_grasp_trial.v1",
            "timestamp": datetime.now().astimezone().isoformat(
                timespec="milliseconds"),
            "condition": args.condition,
            "notes": args.notes,
            "goal": {
                "target_position": args.target_position,
                "max_current": args.max_current,
                "current_delta_threshold": args.current_delta_threshold,
                "timeout_sec": args.timeout_sec,
            },
            "action_status": wrapped.status,
            "result": {
                "success": result.success,
                "message": result.message,
                "final_position": result.final_position,
                "final_current": result.final_current,
                "grasp_detected": result.grasp_detected,
                "object_lost": result.object_lost,
                "state": _state_dict(result.state),
            },
            "feedback": node.feedback,
        }
        with output.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(f"saved: {output}")
        print(
            f"result success={result.success} detected={result.grasp_detected} "
            f"lost={result.object_lost} position={result.final_position} "
            f"current={result.final_current} message={result.message}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
