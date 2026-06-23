#!/usr/bin/env python3
"""Bridge rosbag2's best-effort clock to a reliable ROS clock."""

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
from rosgraph_msgs.msg import Clock


class ClockRelayNode(Node):
    def __init__(self):
        super().__init__('openloris_clock_relay')
        self.declare_parameter('input_topic', '/clock_raw')
        self.declare_parameter('output_topic', '/clock')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Clock, output_topic, reliable_qos)
        self.subscription = self.create_subscription(
            Clock,
            input_topic,
            self.publisher.publish,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Bridging {input_topic} to reliable {output_topic}')


def main(args=None):
    rclpy.init(args=args)
    node = ClockRelayNode()
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
