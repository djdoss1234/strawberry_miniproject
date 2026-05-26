from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="e0509_gripper_description",
            executable="environment_visualizer.py",
            name="environment_visualizer",
            output="screen",
        ),
    ])
