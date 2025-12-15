#!/usr/bin/env python3
"""
ROS2 Bridge for Digital Twin (별도 프로세스로 실행)

ROS2에서 joint_states를 받아서 공유 파일에 저장합니다.
Isaac Sim digital_twin.py가 이 파일을 읽어서 로봇을 동기화합니다.

사용법:
    # 터미널 2: ROS2 Bridge 실행 (ROS2 환경에서)
    source /opt/ros/humble/setup.bash
    source ~/doosan_ws/install/setup.bash
    python3 digital_twin_bridge.py
"""

import json
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 공유 파일 경로
SHARED_FILE = '/tmp/doosan_joint_states.json'


class JointStateBridge(Node):
    """Joint states를 파일로 저장하는 브릿지 노드"""

    def __init__(self, namespace='dsr01'):
        super().__init__('joint_state_bridge')
        self.namespace = namespace

        # Subscriber
        topic_name = f'/{namespace}/joint_states'
        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f'Joint State Bridge started')
        self.get_logger().info(f'Subscribed to: {topic_name}')
        self.get_logger().info(f'Writing to: {SHARED_FILE}')

        self.count = 0

    def joint_state_callback(self, msg):
        """Joint state를 파일에 저장"""
        data = {
            'timestamp': time.time(),
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
        }

        try:
            with open(SHARED_FILE, 'w') as f:
                json.dump(data, f)

            self.count += 1
            if self.count % 100 == 0:
                self.get_logger().info(f'Published {self.count} joint states')

        except Exception as e:
            self.get_logger().error(f'Failed to write: {e}')


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', '-n', default='dsr01')
    args = parser.parse_args()

    rclpy.init()
    node = JointStateBridge(namespace=args.namespace)

    print('=' * 60)
    print('  ROS2 Joint State Bridge')
    print('=' * 60)
    print(f'  Namespace: {args.namespace}')
    print(f'  Output: {SHARED_FILE}')
    print('=' * 60)
    print('  Press Ctrl+C to exit')
    print('=' * 60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # 파일 정리
        if os.path.exists(SHARED_FILE):
            os.remove(SHARED_FILE)


if __name__ == '__main__':
    main()
