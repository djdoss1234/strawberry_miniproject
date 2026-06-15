#!/usr/bin/env python3
"""SafeGrasp action adapter for the verified ROS flange-serial gripper services."""

import json
import threading
import time

import rclpy
from dsr_gripper_tcp_interfaces.action import SafeGrasp
from dsr_gripper_tcp_interfaces.msg import GripperState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class SafeGraspRosAdapter(Node):
    """Expose SafeGrasp while delegating hardware access to the existing ROS node."""

    def __init__(self):
        super().__init__("safe_grasp_ros_adapter")
        self.declare_parameter("gripper_prefix", "/dsr01/gripper")
        self.declare_parameter("settle_sec", 2.0)
        self.declare_parameter("service_timeout_sec", 8.0)
        self.declare_parameter("position_tolerance", 8)
        prefix = self.get_parameter("gripper_prefix").value.rstrip("/")
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.position_tolerance = int(self.get_parameter("position_tolerance").value)
        self.group = ReentrantCallbackGroup()
        self.position_pub = self.create_publisher(
            Int32, f"{prefix}/position_cmd", 10)
        self.read_state = self.create_client(
            Trigger, f"{prefix}/read_state", callback_group=self.group)
        self.action = ActionServer(
            self, SafeGrasp, "/gripper_service/safe_grasp",
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
            callback_group=self.group)
        self.get_logger().warning(
            "ROS flange-serial SafeGrasp adapter ready. Initial calibration mode: "
            "one baseline read and one final read; no continuous object-lost monitoring.")

    def accept_goal(self, request):
        if request.target_position < 0 or request.target_position > 700:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def call_read_state(self):
        if not self.read_state.wait_for_service(timeout_sec=self.service_timeout):
            raise RuntimeError("existing /dsr01/gripper/read_state service unavailable")
        future = self.read_state.call_async(Trigger.Request())
        complete = threading.Event()
        future.add_done_callback(lambda _future: complete.set())
        if not complete.wait(timeout=self.service_timeout):
            raise RuntimeError("read_state timeout")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                f"read_state failed: {response.message if response else 'timeout'}")
        data = json.loads(response.message)
        position = int(data["position"])
        current = int(data["current_raw"])
        if position < 0 or current < 0:
            raise RuntimeError(f"invalid read_state: {response.message}")
        return position, current

    def state_message(
        self, position, current, goal_position, current_limit, detected, text):
        state = GripperState()
        state.stamp = self.get_clock().now().to_msg()
        state.ready = True
        state.torque_enabled = True
        state.moving = False
        state.in_position = abs(position - goal_position) <= self.position_tolerance
        state.grasp_detected = detected
        state.object_lost = False
        state.present_position = position
        state.goal_position = goal_position
        state.present_current = current
        state.current_limit = current_limit
        state.status_text = text
        return state

    def feedback(self, position, current, delta, goal, detected, text):
        feedback = SafeGrasp.Feedback()
        feedback.present_position = position
        feedback.present_current = current
        feedback.current_delta = delta
        feedback.grasp_detected = detected
        feedback.object_lost = False
        feedback.state = self.state_message(
            position, current, goal.target_position, goal.max_current, detected, text)
        return feedback

    def execute(self, goal_handle):
        goal = goal_handle.request
        result = SafeGrasp.Result()
        try:
            start_position, start_current = self.call_read_state()
            goal_handle.publish_feedback(self.feedback(
                start_position, start_current, 0, goal, False, "baseline"))
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "canceled before command"
                return result

            self.position_pub.publish(Int32(data=int(goal.target_position)))
            time.sleep(max(0.0, self.settle_sec))
            final_position, final_current = self.call_read_state()
            delta = abs(abs(final_current) - abs(start_current))
            detected = (
                abs(final_current) >= abs(int(goal.max_current))
                or (
                    goal.current_delta_threshold > 0
                    and delta >= abs(int(goal.current_delta_threshold))
                )
            )
            state = self.state_message(
                final_position, final_current, goal.target_position,
                goal.max_current, detected, "final")
            goal_handle.publish_feedback(self.feedback(
                final_position, final_current, delta, goal, detected, "final"))
            result.success = detected
            result.message = (
                "grasp detected" if detected else "target reached without grasp")
            result.final_position = final_position
            result.final_current = final_current
            result.grasp_detected = detected
            result.object_lost = False
            result.state = state
            if detected:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:  # noqa: BLE001
            result.success = False
            result.message = str(exc)
            result.state.status_text = str(exc)
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = SafeGraspRosAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
