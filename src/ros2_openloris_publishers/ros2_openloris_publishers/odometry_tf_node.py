#!/usr/bin/env python3
"""Normalize OpenLORIS wheel odometry and publish its missing TF."""

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .geometry import relative_pose


class OdometryTfNode(Node):
    """Move the arbitrary dataset odometry origin to identity."""

    def __init__(self):
        super().__init__('openloris_odometry_tf')

        self.declare_parameter('odometry_topic_in', '/odom')
        self.declare_parameter('odometry_topic_out', '/openloris/odom')
        self.declare_parameter('parent_frame', 'base_odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('normalize_to_first_pose', True)
        self.declare_parameter('linear_velocity_variance', 0.01)
        self.declare_parameter('angular_velocity_variance', 0.01)
        self.declare_parameter('position_variance', 0.01)
        self.declare_parameter('orientation_variance', 0.01)

        input_topic = str(self.get_parameter('odometry_topic_in').value)
        output_topic = str(self.get_parameter('odometry_topic_out').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.normalize = bool(
            self.get_parameter('normalize_to_first_pose').value)
        self.linear_velocity_variance = float(
            self.get_parameter('linear_velocity_variance').value)
        self.angular_velocity_variance = float(
            self.get_parameter('angular_velocity_variance').value)
        self.position_variance = float(
            self.get_parameter('position_variance').value)
        self.orientation_variance = float(
            self.get_parameter('orientation_variance').value)

        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Odometry, output_topic, output_qos)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.origin = None

        self.get_logger().info(
            f'Normalizing {input_topic} to {output_topic} and broadcasting '
            f'{self.parent_frame} -> {self.child_frame}')

    @staticmethod
    def _pose_tuple(pose):
        position = (
            pose.position.x, pose.position.y, pose.position.z)
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return position, orientation

    def _on_odometry(self, message):
        position, orientation = self._pose_tuple(message.pose.pose)
        if self.origin is None:
            self.origin = (position, orientation)
            self.get_logger().info('Captured the wheel-odometry origin')

        if self.normalize:
            position, orientation = relative_pose(
                self.origin[0], self.origin[1], position, orientation)

        output = copy.deepcopy(message)
        output.header.frame_id = self.parent_frame
        output.child_frame_id = self.child_frame
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation.x = orientation[0]
        output.pose.pose.orientation.y = orientation[1]
        output.pose.pose.orientation.z = orientation[2]
        output.pose.pose.orientation.w = orientation[3]
        if not any(output.pose.covariance):
            for index in (0, 7, 14):
                output.pose.covariance[index] = self.position_variance
            for index in (21, 28, 35):
                output.pose.covariance[index] = self.orientation_variance
        if not any(output.twist.covariance):
            for index in (0, 7, 14):
                output.twist.covariance[index] = self.linear_velocity_variance
            for index in (21, 28, 35):
                output.twist.covariance[index] = self.angular_velocity_variance
        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTfNode()
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
