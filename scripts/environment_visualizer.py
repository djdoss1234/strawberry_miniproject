#!/usr/bin/env python3
"""
Publish a shared environment model to RViz markers and MoveIt CollisionObject.

This is intentionally visualization-first. The same config/environment.yaml can
later be used by cuRobo WorldConfig so RViz and cuRobo reason about the same
wall/tray model.
"""

import os
import math

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def resolve_environment_yaml():
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/environment.yaml"),
        os.path.join(
            get_package_share_directory("e0509_gripper_description"),
            "config",
            "environment.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def resolve_place_slots_yaml():
    candidates = [
        os.path.expanduser("~/doosan_ws/src/e0509_gripper_description/config/place_slots.yaml"),
        os.path.join(
            get_package_share_directory("e0509_gripper_description"),
            "config",
            "place_slots.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def pose_from_wxyz(values):
    x, y, z, qw, qx, qy, qz = [float(v) for v in values]
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = qw
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    return pose


def color_from_rgba(values):
    color = ColorRGBA()
    color.r = float(values[0])
    color.g = float(values[1])
    color.b = float(values[2])
    color.a = float(values[3])
    return color


def normalized_axis(values, fallback):
    axis = [float(v) for v in values]
    norm = math.sqrt(sum(v * v for v in axis))
    if norm < 1e-9:
        return [float(v) for v in fallback]
    return [v / norm for v in axis]


def yaw_quat_from_axis_x(axis_x):
    yaw = math.atan2(float(axis_x[1]), float(axis_x[0]))
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


class EnvironmentVisualizer(Node):
    def __init__(self):
        super().__init__("environment_visualizer")
        self.config_path = resolve_environment_yaml()
        self.place_slots_path = resolve_place_slots_yaml()
        self.frame_id = "base_link"
        self.objects = []
        self.egg_tray = {}
        self.place_grid = {}
        self.load_config()

        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self.collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.timer = self.create_timer(1.0, self.publish_all)

        self.get_logger().info(f"Environment visualizer ready: {self.config_path}")
        self.get_logger().info(f"Place slot overlay ready: {self.place_slots_path}")
        self.get_logger().info(f"Enabled objects: {[obj['name'] for obj in self.objects]}")

    def load_config(self):
        with open(self.config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        self.frame_id = data.get("frame_id", "base_link")
        self.objects = [
            obj for obj in data.get("objects", [])
            if obj.get("enabled", True) and obj.get("type", "cuboid") == "cuboid"
        ]
        self.egg_tray = data.get("egg_tray", {}) or {}
        if os.path.exists(self.place_slots_path):
            with open(self.place_slots_path, "r", encoding="utf-8") as f:
                place_data = yaml.safe_load(f) or {}
            self.place_grid = place_data.get("grid_generation", {}) or {}
        else:
            self.place_grid = {}

    def make_cuboid_marker(self, obj, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "environment"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_from_wxyz(obj["pose"])
        marker.scale.x = float(obj["dims"][0])
        marker.scale.y = float(obj["dims"][1])
        marker.scale.z = float(obj["dims"][2])
        marker.color = color_from_rgba(obj.get("color", [0.6, 0.8, 1.0, 0.35]))
        return marker

    def make_collision_object(self, obj):
        collision = CollisionObject()
        collision.header.frame_id = self.frame_id
        collision.header.stamp = self.get_clock().now().to_msg()
        collision.id = obj["name"]
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in obj["dims"]]
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose_from_wxyz(obj["pose"]))
        collision.operation = CollisionObject.ADD
        return collision

    def egg_tray_grid_center(self):
        if not self.egg_tray.get("enabled", False):
            return None
        cols, rows = self.egg_tray.get("layout", [5, 3])
        pitch_x, pitch_y = self.place_grid.get(
            "pitch_m", self.egg_tray.get("slot_pitch", [0.055, 0.055]))
        axis_x = normalized_axis(self.place_grid.get("axis_x", [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0])
        axis_y = normalized_axis(self.place_grid.get("axis_y", [0.0, 1.0, 0.0]), [0.0, 1.0, 0.0])
        origin = self.egg_tray.get("slot0_center", [0.62, 0.11, 0.05])
        return [
            float(origin[i])
            + 0.5 * (int(cols) - 1) * float(pitch_x) * float(axis_x[i])
            + 0.5 * (int(rows) - 1) * float(pitch_y) * float(axis_y[i])
            for i in range(3)
        ]

    def aligned_egg_tray_body(self, obj):
        if obj.get("name") != "egg_tray_body":
            return obj
        center = self.egg_tray_grid_center()
        if center is None:
            return obj
        aligned = dict(obj)
        pose = list(obj["pose"])
        pose[0] = center[0]
        pose[1] = center[1]
        axis_x = normalized_axis(self.place_grid.get("axis_x", [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0])
        pose[3:7] = yaw_quat_from_axis_x(axis_x)
        aligned["pose"] = pose
        return aligned

    def add_egg_tray_slot_markers(self, markers, start_id):
        if not self.egg_tray.get("enabled", False):
            return start_id
        cols, rows = self.egg_tray.get("layout", [5, 3])
        pitch_x, pitch_y = self.place_grid.get(
            "pitch_m", self.egg_tray.get("slot_pitch", [0.055, 0.055]))
        axis_x = normalized_axis(self.place_grid.get("axis_x", [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0])
        axis_y = normalized_axis(self.place_grid.get("axis_y", [0.0, 1.0, 0.0]), [0.0, 1.0, 0.0])
        origin = self.egg_tray.get("slot0_center", [0.62, 0.11, 0.05])
        diameter = float(self.egg_tray.get("slot_diameter", 0.05))
        color = color_from_rgba(self.egg_tray.get("color", [1.0, 0.75, 0.1, 0.7]))

        marker_id = start_id
        for row in range(int(rows)):
            for col in range(int(cols)):
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "egg_tray_slots"
                marker.id = marker_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = float(origin[0]) + col * float(pitch_x) * float(axis_x[0]) + row * float(pitch_y) * float(axis_y[0])
                marker.pose.position.y = float(origin[1]) + col * float(pitch_x) * float(axis_x[1]) + row * float(pitch_y) * float(axis_y[1])
                marker.pose.position.z = float(origin[2]) + col * float(pitch_x) * float(axis_x[2]) + row * float(pitch_y) * float(axis_y[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = 0.01
                marker.color = color
                markers.markers.append(marker)
                marker_id += 1
        return marker_id

    def publish_all(self):
        markers = MarkerArray()
        marker_id = 0
        for obj in self.objects:
            obj = self.aligned_egg_tray_body(obj)
            marker = self.make_cuboid_marker(obj, marker_id)
            markers.markers.append(marker)
            marker_id += 1
            self.collision_pub.publish(self.make_collision_object(obj))

        self.add_egg_tray_slot_markers(markers, marker_id)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = EnvironmentVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
