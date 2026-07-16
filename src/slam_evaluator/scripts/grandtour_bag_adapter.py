#!/usr/bin/env python3
# Copyright 2026 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Adapt raw GrandTour rosbag topics to the evaluator's TF convention."""

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATIC_TF_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
CLOCK_INPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CLOCK_OUTPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose(left, right):
    """Compose T(a,b) and T(b,c) into T(a,c)."""
    left_position, left_orientation = left
    right_position, right_orientation = right
    rotated = rotate_vector(left_orientation, right_position)
    return (
        tuple(left_position[index] + rotated[index] for index in range(3)),
        quaternion_multiply(left_orientation, right_orientation),
    )


def invert(transform):
    """Invert T(a,b) into T(b,a)."""
    position, orientation = transform
    inverse_orientation = quaternion_conjugate(orientation)
    inverse_position = rotate_vector(
        inverse_orientation,
        tuple(-value for value in position),
    )
    return inverse_position, inverse_orientation


def pose_transform(pose):
    position = pose.position
    orientation = pose.orientation
    return (
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


def stamped_transform(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class GrandTourBagAdapter(Node):
    """Normalize raw ANYmal odometry and reconstruct evaluator TF links."""

    def __init__(self):
        super().__init__('grandtour_bag_adapter')

        self.declare_parameter(
            'odometry_topic_in', '/anymal/state_estimator/odometry')
        self.declare_parameter(
            'odometry_topic_out', '/slam_evaluator/grandtour/odometry')
        self.declare_parameter(
            'ground_truth_topic',
            '/boxi/inertial_explorer/tc/odometry')
        self.declare_parameter(
            'clock_topic_in', '/slam_evaluator/grandtour/bag_clock')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base')
        self.declare_parameter('box_frame', 'box_base')
        self.declare_parameter('ground_truth_frame', 'enu_origin')
        self.declare_parameter('ground_truth_body_frame', 'cpt7_imu')
        self.declare_parameter('normalize_position', True)

        odometry_topic_in = str(
            self.get_parameter('odometry_topic_in').value)
        odometry_topic_out = str(
            self.get_parameter('odometry_topic_out').value)
        ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value)
        clock_topic_in = str(self.get_parameter('clock_topic_in').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.box_frame = str(self.get_parameter('box_frame').value)
        self.ground_truth_frame = str(
            self.get_parameter('ground_truth_frame').value)
        self.ground_truth_body_frame = str(
            self.get_parameter('ground_truth_body_frame').value)
        self.normalize_position = bool(
            self.get_parameter('normalize_position').value)

        self.publisher = self.create_publisher(
            Odometry, odometry_topic_out, RELIABLE_QOS)
        self.clock_publisher = self.create_publisher(
            Clock, '/clock', CLOCK_OUTPUT_QOS)
        self.odometry_subscription = self.create_subscription(
            Odometry,
            odometry_topic_in,
            self._on_odometry,
            RELIABLE_QOS,
        )
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            ground_truth_topic,
            self._on_ground_truth,
            RELIABLE_QOS,
        )
        self.static_tf_subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self._on_static_tf,
            STATIC_TF_QOS,
        )
        self.clock_subscription = self.create_subscription(
            Clock,
            clock_topic_in,
            self._on_clock,
            CLOCK_INPUT_QOS,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.position_origin = None
        self.alignment_odometry = None
        self.first_ground_truth = None
        self.static_transforms = {}
        self.alignment_published = False

        self.get_logger().info(
            f'Adapting {odometry_topic_in} to {odometry_topic_out}; '
            f'broadcasting {self.parent_frame} -> {self.child_frame}')

    def _on_odometry(self, message):
        if self.position_origin is None and self.normalize_position:
            position = message.pose.pose.position
            self.position_origin = (position.x, position.y, position.z)
            self.get_logger().info(
                'Captured the initial ANYmal odometry position origin')

        if self.position_origin is None:
            self.position_origin = (0.0, 0.0, 0.0)
        self._publish_odometry(message)

    def _on_clock(self, message):
        self.clock_publisher.publish(message)

    def _publish_odometry(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = self.parent_frame
        output.child_frame_id = self.child_frame

        position = output.pose.pose.position
        if self.normalize_position:
            position.x -= self.position_origin[0]
            position.y -= self.position_origin[1]
            position.z -= self.position_origin[2]

        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.alignment_odometry = output
        self._publish_alignment_if_ready()

    def _on_ground_truth(self, message):
        if self.first_ground_truth is None:
            self.first_ground_truth = message
            self._publish_alignment_if_ready()

    def _on_static_tf(self, message):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self.static_transforms[key] = stamped_transform(transform)
        self._publish_alignment_if_ready()

    def _publish_alignment_if_ready(self):
        if (
            self.alignment_published
            or self.alignment_odometry is None
            or self.first_ground_truth is None
        ):
            return

        if (
            stamp_nanoseconds(self.alignment_odometry.header.stamp)
            < stamp_nanoseconds(self.first_ground_truth.header.stamp)
        ):
            return

        base_to_box = self.static_transforms.get(
            (self.child_frame, self.box_frame))
        box_to_body = self.static_transforms.get(
            (self.box_frame, self.ground_truth_body_frame))
        if base_to_box is None or box_to_body is None:
            return

        odom_to_base = pose_transform(
            self.alignment_odometry.pose.pose)
        ground_truth_to_body = pose_transform(
            self.first_ground_truth.pose.pose)
        odom_to_ground_truth = compose(
            compose(compose(odom_to_base, base_to_box), box_to_body),
            invert(ground_truth_to_body),
        )

        position, orientation = odom_to_ground_truth
        transform = TransformStamped()
        transform.header.stamp = self.first_ground_truth.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.ground_truth_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        self.static_tf_broadcaster.sendTransform(transform)
        self.alignment_published = True
        self.get_logger().info(
            f'Broadcast one-shot {self.parent_frame} -> '
            f'{self.ground_truth_frame} alignment')


def main(args=None):
    rclpy.init(args=args)
    node = GrandTourBagAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
