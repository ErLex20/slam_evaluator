#!/usr/bin/env python3

"""
Rescales a single PointCloud2 field by a constant factor and republishes.

Some point cloud sources record their per-point timestamp field as
nanoseconds since epoch (e.g. the Livox Mid-360 recording used in the
IILABS3D benchmark), while scan_deskewer (dotX-Automation/scan_deskewer)
reads that field directly as seconds. Left unconverted, the ~1e9 scale
mismatch turns every per-point time delta into a bogus multi-second dt during
deskewing, which explodes the integrated motion and corrupts every point.

dotX Automation s.r.l. <info@dotxautomation.com>
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudFieldRescale(Node):

    def __init__(self):
        super().__init__('pointcloud_field_rescale')

        self.declare_parameter('field', 'timestamp')
        self.declare_parameter('scale', 1e-9)
        self.field = self.get_parameter('field').value
        self.scale = self.get_parameter('scale').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.pub = self.create_publisher(PointCloud2, 'output', qos)
        self.sub = self.create_subscription(PointCloud2, 'input', self.callback, qos)

    def callback(self, msg):
        points = pc2.read_points(msg, field_names=None, skip_nans=False).copy()
        points[self.field] = points[self.field] * self.scale
        out = pc2.create_cloud(msg.header, msg.fields, points)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointCloudFieldRescale()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
