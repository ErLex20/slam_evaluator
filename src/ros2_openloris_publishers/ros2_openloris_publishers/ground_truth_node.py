#!/usr/bin/env python3
"""Expose OpenLORIS /gt TF messages as normalized pose and path topics."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from .geometry import relative_pose


class GroundTruthNode(Node):
    """Normalize the arbitrary gt_map origin for an online SLAM comparison."""

    def __init__(self):
        super().__init__('openloris_ground_truth')

        self.declare_parameter('tf_topic_in', '/gt')
        self.declare_parameter('source_child_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link_gt')
        self.declare_parameter(
            'pose_topic_out', '/openloris/ground_truth/pose')
        self.declare_parameter(
            'path_topic_out', '/openloris/ground_truth/path')
        self.declare_parameter('normalize_to_first_pose', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('path_min_distance', 0.02)

        input_topic = str(self.get_parameter('tf_topic_in').value)
        self.source_child_frame = str(
            self.get_parameter('source_child_frame').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(
            self.get_parameter('child_frame_id').value)
        pose_topic = str(self.get_parameter('pose_topic_out').value)
        path_topic = str(self.get_parameter('path_topic_out').value)
        self.normalize = bool(
            self.get_parameter('normalize_to_first_pose').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.path_min_distance = float(
            self.get_parameter('path_min_distance').value)

        self.pose_publisher = self.create_publisher(
            PoseStamped, pose_topic, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 1)
        self.subscription = self.create_subscription(
            TFMessage, input_topic, self._on_tf, qos_profile_sensor_data)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None)

        self.origin = None
        self.last_path_position = None
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.get_logger().info(
            f'Publishing normalized ground truth from {input_topic} on '
            f'{path_topic}')

    @staticmethod
    def _transform_tuple(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _on_tf(self, message):
        source = next(
            (
                transform for transform in message.transforms
                if transform.child_frame_id == self.source_child_frame
            ),
            None,
        )
        if source is None:
            return

        position, orientation = self._transform_tuple(source)
        if self.origin is None:
            self.origin = (position, orientation)
            self.get_logger().info('Captured the ground-truth origin')
        if self.normalize:
            position, orientation = relative_pose(
                self.origin[0], self.origin[1], position, orientation)

        pose = PoseStamped()
        pose.header.stamp = source.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        self.pose_publisher.publish(pose)

        should_append = self.last_path_position is None
        if self.last_path_position is not None:
            should_append = (
                math.dist(position, self.last_path_position)
                >= self.path_min_distance
            )
        if should_append:
            self.path.poses.append(pose)
            self.last_path_position = position
        self.path.header.stamp = pose.header.stamp
        self.path_publisher.publish(self.path)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
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
