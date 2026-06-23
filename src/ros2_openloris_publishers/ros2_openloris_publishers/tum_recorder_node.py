#!/usr/bin/env python3
"""Record a ROS pose stream in TUM trajectory format."""

from datetime import datetime
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class TumRecorderNode(Node):
    def __init__(self):
        super().__init__('openloris_tum_recorder')

        self.declare_parameter(
            'pose_topic', '/lio_sam/map_optimization/pose')
        self.declare_parameter('pose_msg_type', 'pose_with_covariance')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('slam_name', 'lio_sam_openloris')
        self.declare_parameter('append_datetime', True)

        pose_topic = str(self.get_parameter('pose_topic').value)
        pose_msg_type = str(self.get_parameter('pose_msg_type').value)
        output_dir = str(self.get_parameter('output_dir').value)
        slam_name = str(self.get_parameter('slam_name').value)
        append_datetime = bool(
            self.get_parameter('append_datetime').value)

        name = slam_name
        if append_datetime:
            name += datetime.now().strftime('_%Y-%m-%d_%H-%M-%S')
        self.output_file = os.path.join(output_dir, name + '.tum')
        self.output_dir = output_dir

        message_types = {
            'pose_with_covariance': (
                PoseWithCovarianceStamped, self._on_pose_with_covariance),
            'pose': (PoseStamped, self._on_pose),
            'odometry': (Odometry, self._on_odometry),
        }
        if pose_msg_type not in message_types:
            raise ValueError(
                f"Unsupported pose_msg_type '{pose_msg_type}'")

        message_type, callback = message_types[pose_msg_type]
        self.subscription = self.create_subscription(
            message_type, pose_topic, callback, 100)
        self.file = None
        self.count = 0
        self.get_logger().info(
            f'Recording {pose_topic} to {self.output_file}')

    def _on_pose_with_covariance(self, message):
        self._write(message.header, message.pose.pose)

    def _on_pose(self, message):
        self._write(message.header, message.pose)

    def _on_odometry(self, message):
        self._write(message.header, message.pose.pose)

    def _write(self, header, pose):
        if not self.output_dir:
            return
        if self.file is None:
            os.makedirs(self.output_dir, exist_ok=True)
            self.file = open(self.output_file, 'w', encoding='utf-8')

        timestamp = header.stamp.sec + header.stamp.nanosec * 1.0e-9
        position = pose.position
        orientation = pose.orientation
        self.file.write(
            f'{timestamp:.9f} '
            f'{position.x:.9f} {position.y:.9f} {position.z:.9f} '
            f'{orientation.x:.9f} {orientation.y:.9f} '
            f'{orientation.z:.9f} {orientation.w:.9f}\n'
        )
        self.file.flush()
        self.count += 1

    def destroy_node(self):
        if self.file is not None:
            self.file.close()
            self.get_logger().info(
                f'Saved {self.count} poses to {self.output_file}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TumRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
