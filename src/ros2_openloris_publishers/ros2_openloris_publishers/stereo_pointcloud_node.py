#!/usr/bin/env python3
"""Build an XYZI point cloud from the OpenLORIS T265 stereo pair."""

from collections import OrderedDict
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def quaternion_matrix_xyzw(quaternion):
    """Return the rotation matrix represented by an xyzw quaternion."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(np.float64).eps:
        raise ValueError('Stereo extrinsic quaternion has zero length')
    scale = 2.0 / norm
    return np.array([
        [
            1.0 - scale * (y * y + z * z),
            scale * (x * y - z * w),
            scale * (x * z + y * w),
        ],
        [
            scale * (x * y + z * w),
            1.0 - scale * (x * x + z * z),
            scale * (y * z - x * w),
        ],
        [
            scale * (x * z - y * w),
            scale * (y * z + x * w),
            1.0 - scale * (x * x + y * y),
        ],
    ])


def baseline_rectification(
        camera2_translation_camera1,
        camera2_rotation_camera1,
):
    """Return rotations that align both camera images with their baseline."""
    translation = np.asarray(
        camera2_translation_camera1, dtype=np.float64)
    baseline = float(np.linalg.norm(translation))
    if baseline <= np.finfo(np.float64).eps:
        raise ValueError('Stereo baseline has zero length')

    x_axis = translation / baseline
    optical_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    z_axis = optical_axis - x_axis * np.dot(optical_axis, x_axis)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm <= np.finfo(np.float64).eps:
        raise ValueError('Stereo baseline is parallel to the optical axis')
    z_axis /= z_norm
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)

    left_rotation = np.stack((x_axis, y_axis, z_axis))
    right_rotation = left_rotation @ camera2_rotation_camera1
    return left_rotation, right_rotation, baseline


class StereoPointCloudNode(Node):
    """Rectify T265 fisheye images, compute disparity, and publish XYZI."""

    def __init__(self):
        super().__init__('openloris_stereo_pointcloud')

        self.declare_parameter(
            'left_image_topic', '/t265/fisheye1/image_raw')
        self.declare_parameter(
            'right_image_topic', '/t265/fisheye2/image_raw')
        self.declare_parameter(
            'left_camera_info_topic', '/t265/fisheye1/camera_info')
        self.declare_parameter(
            'right_camera_info_topic', '/t265/fisheye2/camera_info')
        self.declare_parameter(
            'pointcloud_topic', '/openloris/stereo/point_cloud')
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('rectified_focal_length', 285.0)
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('num_disparities', 128)
        self.declare_parameter('block_size', 7)
        self.declare_parameter('uniqueness_ratio', 10)
        self.declare_parameter('speckle_window_size', 100)
        self.declare_parameter('speckle_range', 2)
        self.declare_parameter('disparity_edge_threshold', 4.0)
        self.declare_parameter('sync_tolerance_ms', 2.0)
        self.declare_parameter('opencv_threads', 4)
        self.declare_parameter(
            'camera2_translation_camera1',
            [0.0639765113592148, 0.000148267135955393,
             -0.000398468371713534],
        )
        self.declare_parameter(
            'camera2_rotation_camera1_xyzw',
            [0.00188497, 0.00347595, 0.00154952, 0.999991],
        )

        self.left_image_topic = str(
            self.get_parameter('left_image_topic').value)
        self.right_image_topic = str(
            self.get_parameter('right_image_topic').value)
        left_info_topic = str(
            self.get_parameter('left_camera_info_topic').value)
        right_info_topic = str(
            self.get_parameter('right_camera_info_topic').value)
        pointcloud_topic = str(
            self.get_parameter('pointcloud_topic').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.point_stride = max(
            1, int(self.get_parameter('point_stride').value))
        self.rectified_focal_length = float(
            self.get_parameter('rectified_focal_length').value)
        self.min_disparity = int(
            self.get_parameter('min_disparity').value)
        self.num_disparities = int(
            self.get_parameter('num_disparities').value)
        self.block_size = int(self.get_parameter('block_size').value)
        self.uniqueness_ratio = int(
            self.get_parameter('uniqueness_ratio').value)
        self.speckle_window_size = int(
            self.get_parameter('speckle_window_size').value)
        self.speckle_range = int(
            self.get_parameter('speckle_range').value)
        self.disparity_edge_threshold = float(
            self.get_parameter('disparity_edge_threshold').value)
        self.sync_tolerance_ns = int(
            float(self.get_parameter('sync_tolerance_ms').value) * 1e6)

        if self.num_disparities <= 0 or self.num_disparities % 16:
            raise ValueError('num_disparities must be a positive multiple of 16')
        if self.block_size < 3 or self.block_size % 2 == 0:
            raise ValueError('block_size must be odd and at least 3')
        if not 0.0 < self.min_depth < self.max_depth:
            raise ValueError('Require 0 < min_depth < max_depth')
        if self.rectified_focal_length <= 0.0:
            raise ValueError('rectified_focal_length must be positive')

        cv2.setNumThreads(max(
            1, int(self.get_parameter('opencv_threads').value)))

        translation = self.get_parameter(
            'camera2_translation_camera1').value
        quaternion = self.get_parameter(
            'camera2_rotation_camera1_xyzw').value
        camera2_rotation_camera1 = quaternion_matrix_xyzw(quaternion)
        (
            self.left_rectification,
            self.right_rectification,
            self.baseline,
        ) = baseline_rectification(
            translation, camera2_rotation_camera1)

        channels = 1
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * channels * self.block_size * self.block_size,
            P2=32 * channels * self.block_size * self.block_size,
            disp12MaxDiff=1,
            preFilterCap=31,
            uniquenessRatio=self.uniqueness_ratio,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        reliable_cloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, pointcloud_topic, reliable_cloud_qos)
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            left_info_topic,
            lambda message: self._on_camera_info('left', message),
            qos_profile_sensor_data,
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            right_info_topic,
            lambda message: self._on_camera_info('right', message),
            qos_profile_sensor_data,
        )
        self.left_image_sub = self.create_subscription(
            Image,
            self.left_image_topic,
            lambda message: self._on_image('left', message),
            qos_profile_sensor_data,
        )
        self.right_image_sub = self.create_subscription(
            Image,
            self.right_image_topic,
            lambda message: self._on_image('right', message),
            qos_profile_sensor_data,
        )

        self.camera_info = {}
        self.maps = None
        self.rectified_intrinsics = None
        self.left_cache = OrderedDict()
        self.right_cache = OrderedDict()
        self.cache_size = 6
        self.warned_no_calibration = False
        self.warned_encoding = set()
        self.cloud_count = 0
        self.processing_seconds = 0.0
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
            f'Computing T265 stereo clouds from {self.left_image_topic} and '
            f'{self.right_image_topic} on {pointcloud_topic}; '
            f'calibrated baseline {self.baseline:.6f} m')

    def _on_camera_info(self, side, message):
        self.camera_info[side] = message
        if len(self.camera_info) == 2:
            try:
                self._prepare_rectification()
            except (TypeError, ValueError, cv2.error) as error:
                self.get_logger().error(
                    f'Cannot initialize stereo rectification: {error}')

    def _prepare_rectification(self):
        left = self.camera_info['left']
        right = self.camera_info['right']
        if left.width != right.width or left.height != right.height:
            raise ValueError('Left and right image dimensions differ')
        if left.width == 0 or left.height == 0:
            raise ValueError('CameraInfo has empty image dimensions')
        for side, info in (('left', left), ('right', right)):
            model = info.distortion_model.lower().replace(' ', '_')
            if model not in ('kannala_brandt4', 'equidistant'):
                raise ValueError(
                    f"{side} distortion model '{info.distortion_model}' "
                    'is not a supported fisheye model')
            if len(info.d) < 4:
                raise ValueError(
                    f'{side} CameraInfo has fewer than four coefficients')

        size = (left.width, left.height)
        left_k = np.asarray(left.k, dtype=np.float64).reshape(3, 3)
        right_k = np.asarray(right.k, dtype=np.float64).reshape(3, 3)
        left_d = np.asarray(left.d[:4], dtype=np.float64)
        right_d = np.asarray(right.d[:4], dtype=np.float64)
        focal = self.rectified_focal_length
        rectified_k = np.array([
            [focal, 0.0, left.width / 2.0],
            [0.0, focal, left.height / 2.0],
            [0.0, 0.0, 1.0],
        ])
        left_maps = cv2.fisheye.initUndistortRectifyMap(
            left_k,
            left_d,
            self.left_rectification,
            rectified_k,
            size,
            cv2.CV_32FC1,
        )
        right_maps = cv2.fisheye.initUndistortRectifyMap(
            right_k,
            right_d,
            self.right_rectification,
            rectified_k,
            size,
            cv2.CV_32FC1,
        )
        self.maps = (left_maps, right_maps)
        self.rectified_intrinsics = (
            focal, focal, left.width / 2.0, left.height / 2.0)
        self.warned_no_calibration = False
        self.get_logger().info(
            f'Rectification ready for {left.width}x{left.height} '
            f'images at focal length {focal:.3f} px')

    @staticmethod
    def _stamp_ns(message):
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _on_image(self, side, message):
        if self.maps is None:
            if not self.warned_no_calibration:
                self.get_logger().warning(
                    'Waiting for both static T265 CameraInfo messages; '
                    'play the bag with --delay 2')
                self.warned_no_calibration = True
            return

        own_cache = self.left_cache if side == 'left' else self.right_cache
        other_cache = self.right_cache if side == 'left' else self.left_cache
        stamp = self._stamp_ns(message)
        own_cache[stamp] = message
        while len(own_cache) > self.cache_size:
            own_cache.popitem(last=False)

        match_stamp = None
        if stamp in other_cache:
            match_stamp = stamp
        elif other_cache:
            nearest = min(other_cache, key=lambda value: abs(value - stamp))
            if abs(nearest - stamp) <= self.sync_tolerance_ns:
                match_stamp = nearest
        if match_stamp is None:
            return

        own_cache.pop(stamp, None)
        other = other_cache.pop(match_stamp)
        if side == 'left':
            self._process_pair(message, other)
        else:
            self._process_pair(other, message)

    def _mono_array(self, image):
        encoding = image.encoding.lower()
        if encoding not in ('8uc1', 'mono8'):
            if encoding not in self.warned_encoding:
                self.get_logger().error(
                    f"Unsupported stereo encoding '{image.encoding}'")
                self.warned_encoding.add(encoding)
            return None
        return np.ndarray(
            shape=(image.height, image.width),
            dtype=np.uint8,
            buffer=memoryview(image.data),
            strides=(image.step, 1),
        )

    def _disparity_edge_mask(self, disparity):
        if self.disparity_edge_threshold <= 0.0:
            return np.zeros(disparity.shape, dtype=bool)
        maximum_jump = np.zeros(disparity.shape, dtype=np.float32)
        maximum_jump[1:, :] = np.maximum(
            maximum_jump[1:, :],
            np.abs(disparity[1:, :] - disparity[:-1, :]),
        )
        maximum_jump[:-1, :] = np.maximum(
            maximum_jump[:-1, :],
            np.abs(disparity[:-1, :] - disparity[1:, :]),
        )
        maximum_jump[:, 1:] = np.maximum(
            maximum_jump[:, 1:],
            np.abs(disparity[:, 1:] - disparity[:, :-1]),
        )
        maximum_jump[:, :-1] = np.maximum(
            maximum_jump[:, :-1],
            np.abs(disparity[:, :-1] - disparity[:, 1:]),
        )
        return maximum_jump > self.disparity_edge_threshold

    def _process_pair(self, left_message, right_message):
        started = time.perf_counter()
        left_image = self._mono_array(left_message)
        right_image = self._mono_array(right_message)
        if left_image is None or right_image is None:
            return
        if left_image.shape != right_image.shape:
            self.get_logger().error('Synchronized stereo image sizes differ')
            return

        (left_maps, right_maps) = self.maps
        left_rectified = cv2.remap(
            left_image, *left_maps, interpolation=cv2.INTER_LINEAR)
        right_rectified = cv2.remap(
            right_image, *right_maps, interpolation=cv2.INTER_LINEAR)
        disparity = (
            self.matcher.compute(left_rectified, right_rectified)
            .astype(np.float32) / 16.0
        )

        fx, fy, cx, cy = self.rectified_intrinsics
        valid_disparity = disparity > (self.min_disparity + 0.5)
        valid_disparity &= disparity < (
            self.min_disparity + self.num_disparities - 1.0)
        depth = np.zeros(disparity.shape, dtype=np.float32)
        depth[valid_disparity] = (
            fx * self.baseline / disparity[valid_disparity])
        valid = (
            valid_disparity
            & np.isfinite(depth)
            & (depth >= self.min_depth)
            & (depth <= self.max_depth)
            & (left_rectified > 2)
            & (right_rectified > 2)
        )
        valid &= ~self._disparity_edge_mask(disparity)

        stride = self.point_stride
        valid = valid[::stride, ::stride]
        if not np.any(valid):
            return
        sampled_depth = depth[::stride, ::stride]
        sampled_intensity = left_rectified[::stride, ::stride]
        rows = np.arange(
            0, left_message.height, stride, dtype=np.float32)
        columns = np.arange(
            0, left_message.width, stride, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(columns, rows)

        z = sampled_depth[valid]
        rectified_points = np.empty((z.size, 3), dtype=np.float32)
        rectified_points[:, 0] = (grid_u[valid] - cx) * z / fx
        rectified_points[:, 1] = (grid_v[valid] - cy) * z / fy
        rectified_points[:, 2] = z

        # R_left maps original-left vectors into the rectified camera. Undo
        # that rotation so the published header truthfully remains fisheye1.
        original_points = rectified_points @ self.left_rectification
        points = np.empty((z.size, 4), dtype=np.float32)
        points[:, :3] = original_points
        points[:, 3] = sampled_intensity[valid].astype(np.float32)

        cloud = PointCloud2()
        cloud.header = left_message.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points.tobytes()
        cloud.is_dense = True
        self.publisher.publish(cloud)

        self.cloud_count += 1
        self.processing_seconds += time.perf_counter() - started
        if self.cloud_count == 1 or self.cloud_count % 30 == 0:
            average_ms = (
                1000.0 * self.processing_seconds / self.cloud_count)
            self.get_logger().info(
                f'Published stereo cloud {self.cloud_count} with '
                f'{cloud.width} points in frame '
                f"'{cloud.header.frame_id}' (average {average_ms:.1f} ms)")


def main(args=None):
    rclpy.init(args=args)
    node = StereoPointCloudNode()
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
