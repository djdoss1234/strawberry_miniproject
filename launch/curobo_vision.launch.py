"""
cuRobo Vision-based Control Launch File

RealSense 카메라로 ArUco 마커를 인식하고 cuRobo로 경로 생성 후 로봇을 제어합니다.

사전 조건:
    - bringup.launch.py가 먼저 실행되어 있어야 합니다.
    - 캘리브레이션 파일이 준비되어 있어야 합니다.

Usage:
    # 기본 실행 (마커 인식 + cuRobo planner)
    ros2 launch e0509_gripper_description curobo_vision.launch.py

    # 캘리브레이션 경로 지정
    ros2 launch e0509_gripper_description curobo_vision.launch.py \
        calibration_path:=/path/to/calibration.npz

    # 자동 전송 모드 (마커 감지 시 자동으로 로봇 이동)
    ros2 launch e0509_gripper_description curobo_vision.launch.py auto_send:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ARGUMENTS = [
        DeclareLaunchArgument('calibration_path',
            default_value=os.path.expanduser('~/sim2real/sim2real/config/calibration_eye_to_hand.npz'),
            description='Path to calibration .npz file'),
        DeclareLaunchArgument('marker_id', default_value='0',
            description='ArUco marker ID'),
        DeclareLaunchArgument('marker_size', default_value='0.05',
            description='Marker size in meters'),
        DeclareLaunchArgument('safe_z_offset', default_value='0.15',
            description='Safety height offset above marker (meters)'),
        DeclareLaunchArgument('auto_send', default_value='false',
            description='Auto send target on marker detection'),
    ]

    # cuRobo Planner Node
    curobo_planner = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'export CUDA_HOME=/usr/local/cuda-12.8 && '
            'source /opt/ros/humble/setup.bash && '
            'source ~/doosan_ws/install/setup.bash && '
            'python3 ~/doosan_ws/src/e0509_gripper_description/scripts/curobo_planner_node.py'
        ],
        output='screen',
    )

    # Marker Tracking Node
    marker_tracking = Node(
        package='e0509_gripper_description',
        executable='marker_tracking_node.py',
        name='marker_tracking_node',
        parameters=[{
            'calibration_path': LaunchConfiguration('calibration_path'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'safe_z_offset': LaunchConfiguration('safe_z_offset'),
            'auto_send': LaunchConfiguration('auto_send'),
        }],
        output='screen',
    )

    return LaunchDescription(ARGUMENTS + [
        curobo_planner,
        marker_tracking,
    ])
