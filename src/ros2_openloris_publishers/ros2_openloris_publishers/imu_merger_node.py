#!/usr/bin/env python3
"""Merge OpenLORIS D400 accelerometer and gyroscope samples."""

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
from sensor_msgs.msg import Imu


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ImuMergerNode(Node):
    """Publish gyro-rate IMU messages using the latest accelerometer sample."""

    def __init__(self):
        super().__init__('openloris_imu_merger')

        self.declare_parameter('accel_topic', '/d400/accel/sample')
        self.declare_parameter('gyro_topic', '/d400/gyro/sample')
        self.declare_parameter('imu_topic', '/openloris/imu')
        self.declare_parameter('max_accel_age', 0.05)
        self.declare_parameter('accel_variance', 0.01)
        self.declare_parameter('gyro_variance', 0.0001)

        accel_topic = str(self.get_parameter('accel_topic').value)
        gyro_topic = str(self.get_parameter('gyro_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.max_accel_age_ns = int(
            float(self.get_parameter('max_accel_age').value) * 1.0e9)
        accel_variance = float(
            self.get_parameter('accel_variance').value)
        gyro_variance = float(self.get_parameter('gyro_variance').value)

        self.accel_covariance = [
            accel_variance, 0.0, 0.0,
            0.0, accel_variance, 0.0,
            0.0, 0.0, accel_variance,
        ]
        self.gyro_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance,
        ]

        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Imu, imu_topic, output_qos)
        self.accel_sub = self.create_subscription(
            Imu, accel_topic, self._on_accel, qos_profile_sensor_data)
        self.gyro_sub = self.create_subscription(
            Imu, gyro_topic, self._on_gyro, qos_profile_sensor_data)

        self.latest_accel = None
        self.warned_waiting = False
        self.logged_first_imu = False
        self.get_logger().info(
            f'Merging {accel_topic} and {gyro_topic} into {imu_topic}')

    def _on_accel(self, message):
        self.latest_accel = message
        self.warned_waiting = False

    def _on_gyro(self, gyro):
        if self.latest_accel is None:
            if not self.warned_waiting:
                self.get_logger().warning(
                    'Waiting for the first accelerometer sample')
                self.warned_waiting = True
            return

        age = abs(
            stamp_ns(gyro.header.stamp)
            - stamp_ns(self.latest_accel.header.stamp)
        )
        if age > self.max_accel_age_ns:
            return

        output = Imu()
        output.header = gyro.header
        output.orientation.w = 1.0
        output.orientation_covariance[0] = -1.0
        output.angular_velocity = gyro.angular_velocity
        output.angular_velocity_covariance = self.gyro_covariance
        output.linear_acceleration = self.latest_accel.linear_acceleration
        output.linear_acceleration_covariance = self.accel_covariance
        self.publisher.publish(output)

        if not self.logged_first_imu:
            self.get_logger().info(
                'Published first merged IMU in frame '
                f"'{output.header.frame_id}'")
            self.logged_first_imu = True


def main(args=None):
    rclpy.init(args=args)
    node = ImuMergerNode()
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
