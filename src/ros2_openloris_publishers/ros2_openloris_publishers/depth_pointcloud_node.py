#!/usr/bin/env python3
"""Project OpenLORIS aligned D400 depth images into reliable XYZI clouds."""

import numpy as np

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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthPointCloudNode(Node):
    """Use the bag's static calibration for every aligned depth frame."""

    def __init__(self):
        super().__init__('openloris_depth_pointcloud')

        self.declare_parameter(
            'image_topic', '/d400/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/d400/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('pointcloud_topic', '/openloris/point_cloud')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 8.0)
        self.declare_parameter('pixel_stride', 2)
        self.declare_parameter('edge_discontinuity_rel', 0.03)

        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        pointcloud_topic = str(
            self.get_parameter('pointcloud_topic').value)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.pixel_stride = max(
            1, int(self.get_parameter('pixel_stride').value))
        self.disc_rel = float(self.get_parameter('edge_discontinuity_rel').value)

        reliable_scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, pointcloud_topic, reliable_scan_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)

        self.intrinsics = None
        self.grid_key = None
        self.grid_u = None
        self.grid_v = None
        self.warned_no_calibration = False
        self.warned_encoding = set()
        self.logged_first_cloud = False

        self.fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        self.get_logger().info(
            f'Projecting {image_topic} to {pointcloud_topic} '
            f'(pixel stride {self.pixel_stride})')

    def _on_camera_info(self, message):
        fx = float(message.k[0])
        fy = float(message.k[4])
        cx = float(message.k[2])
        cy = float(message.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error(
                'CameraInfo contains invalid focal lengths')
            return
        self.intrinsics = (fx, fy, cx, cy)
        self.warned_no_calibration = False

    def _depth_array(self, image):
        encoding = image.encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            dtype = np.dtype('>u2' if image.is_bigendian else '<u2')
            scale = self.depth_scale
        elif encoding == '32fc1':
            dtype = np.dtype('>f4' if image.is_bigendian else '<f4')
            scale = 1.0
        else:
            if encoding not in self.warned_encoding:
                self.get_logger().error(
                    f"Unsupported depth encoding '{image.encoding}'")
                self.warned_encoding.add(encoding)
            return None

        depth = np.ndarray(
            shape=(image.height, image.width),
            dtype=dtype,
            buffer=memoryview(image.data),
            strides=(image.step, dtype.itemsize),
        )
        return (
            depth[::self.pixel_stride, ::self.pixel_stride]
            .astype(np.float32, copy=False) * scale
        )

    def _pixel_grid(self, image, shape):
        key = (image.width, image.height, self.pixel_stride)
        if key != self.grid_key:
            rows = np.arange(
                0, image.height, self.pixel_stride, dtype=np.float32)
            cols = np.arange(
                0, image.width, self.pixel_stride, dtype=np.float32)
            self.grid_u, self.grid_v = np.meshgrid(cols, rows)
            self.grid_key = key
        if self.grid_u.shape != shape:
            raise ValueError('Depth image dimensions do not match pixel grid')
        return self.grid_u, self.grid_v

    def _discontinuity_mask(self, depth):
        d = np.where(np.isfinite(depth) & (depth > 0.0), depth, np.nan)
        jump = np.zeros(d.shape, dtype=np.float32)

        def fold(shifted):
            nonlocal jump
            diff = np.abs(d - shifted)
            jump = np.fmax(jump, np.nan_to_num(diff, nan=0.0))

        s = np.full_like(d, np.nan); s[1:, :]  = d[:-1, :]; fold(s)   # su
        s = np.full_like(d, np.nan); s[:-1, :] = d[1:, :];  fold(s)   # giu'
        s = np.full_like(d, np.nan); s[:, 1:]  = d[:, :-1]; fold(s)   # sx
        s = np.full_like(d, np.nan); s[:, :-1] = d[:, 1:];  fold(s)   # dx

        tau = self.disc_rel * np.nan_to_num(d, nan=0.0)
        return jump > tau

    def _on_image(self, image):
        if self.intrinsics is None:
            if not self.warned_no_calibration:
                self.get_logger().warning(
                    'Waiting for the static aligned-depth CameraInfo; '
                    'play the '
                    'bag with --delay 2 so its single message is discovered')
                self.warned_no_calibration = True
            return

        depth = self._depth_array(image)
        if depth is None:
            return

        u, v = self._pixel_grid(image, depth.shape)
        valid = (
            np.isfinite(depth)
            & (depth >= self.min_depth)
            & (depth <= self.max_depth)
        )
        if self.disc_rel > 0.0:
            valid &= ~self._discontinuity_mask(depth)

        if not np.any(valid):
            return

        z = depth[valid]
        fx, fy, cx, cy = self.intrinsics
        points = np.empty((z.size, 4), dtype=np.float32)
        points[:, 0] = (u[valid] - cx) * z / fx
        points[:, 1] = (v[valid] - cy) * z / fy
        points[:, 2] = z
        points[:, 3] = z

        cloud = PointCloud2()
        cloud.header = image.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points.tobytes()
        cloud.is_dense = True
        self.publisher.publish(cloud)

        if not self.logged_first_cloud:
            self.get_logger().info(
                f'Published first cloud with {cloud.width} points in '
                f"frame '{cloud.header.frame_id}'")
            self.logged_first_cloud = True


def main(args=None):
    rclpy.init(args=args)
    node = DepthPointCloudNode()
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
