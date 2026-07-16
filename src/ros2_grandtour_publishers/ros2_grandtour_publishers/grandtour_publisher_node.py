#!/usr/bin/env python3
"""Replay a downloaded ETH GrandTour mission (raw zarr store) as ROS 2 topics.

GrandTour ships each mission as independent per-stream zarr v2 groups (see
``tools/download_grandtour.py``) rather than a rosbag, so there is no
existing ROS graph to adapt: this node opens the requested streams directly,
merges their timestamps into one global playback order, and publishes the
matching ROS 2 message for each row. See this package's README for the data
model and the assumptions flagged as unverified (quaternion component order,
odometry covariance layout).
"""

import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import alignment, geometry
from .zarr_timeseries import load_static_tf_tree, resolve_stream

# Documented in anymal_state_state_estimator's zarr .zattrs description
# ("Joint Naming 0-11: [...]"); ANYbotics ANYmal C leg/joint order.
ANYMAL_JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',
    'RF_HAA', 'RF_HFE', 'RF_KFE',
    'LH_HAA', 'LH_HFE', 'LH_KFE',
    'RH_HAA', 'RH_HFE', 'RH_KFE',
]

RELIABLE_KEEP_LAST = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def stamp_from_epoch(epoch_seconds):
    stamp = Time()
    stamp.sec = int(epoch_seconds)
    stamp.nanosec = int(round((epoch_seconds - stamp.sec) * 1.0e9))
    return stamp


class GrandTourPublisherNode(Node):
    """Merge-sort enabled GrandTour streams and replay them on a timer thread."""

    def __init__(self):
        super().__init__('grandtour_publisher')

        self.declare_parameter('mission_dir', '')
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('use_wall_clock_pacing', True)
        self.declare_parameter('loop', False)
        self.declare_parameter('start_offset', 0.0)

        self.mission_dir = str(self.get_parameter('mission_dir').value)
        self.frame_prefix = str(self.get_parameter('frame_prefix').value)
        self.playback_rate = max(
            1.0e-6, float(self.get_parameter('playback_rate').value))
        self.publish_clock_enabled = bool(
            self.get_parameter('publish_clock').value)
        self.use_wall_clock_pacing = bool(
            self.get_parameter('use_wall_clock_pacing').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.start_offset = float(self.get_parameter('start_offset').value)

        if not self.mission_dir:
            self.get_logger().error(
                'mission_dir parameter not set. Pass: --ros-args -p '
                'mission_dir:=/home/neo/workspace/logs/grandtour/'
                'SPX-2')
            return

        clock_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub_clock = (
            self.create_publisher(Clock, '/clock', clock_qos)
            if self.publish_clock_enabled else None
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.streams = []
        self._static_frames = {}
        self._static_tf_broadcaster = None
        self._static_transforms = []
        self._declare_and_resolve_lidar()
        self._declare_and_resolve_imu()
        self._declare_and_resolve_odom(
            'anymal_odometry', ['anymal_state_odometry'],
            '/grandtour/anymal/odometry', 'base')
        self._declare_and_resolve_anymal_state()
        self._declare_and_resolve_odom(
            'ground_truth_odometry', ['cpt7_ie_tc_odometry'],
            '/grandtour/gt/odometry', 'cpt7_imu')
        self._declare_and_resolve_prism_position()
        self._declare_and_resolve_ground_truth_tf()
        self._broadcast_static_tf()
        self._broadcast_ground_truth_alignment()
        self._broadcast_prism_alignment()

        if not self.streams:
            self.get_logger().warning(
                'No stream resolved under this mission_dir; nothing to '
                'publish. Check candidates/enable parameters.')

        self._build_timeline()

        self._stop = False
        self._thread = threading.Thread(target=self._playback, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop = True
        super().destroy_node()

    # ------------------------------------------------------------------
    # Parameter declaration + per-kind stream resolution
    # ------------------------------------------------------------------

    def _frame(self, name):
        return f'{self.frame_prefix}{name}' if self.frame_prefix else name

    def _resolve(self, group, candidates, required_msg):
        reader = resolve_stream(self.mission_dir, candidates)
        if reader is None:
            self.get_logger().warning(
                f'[{group}] none of {candidates} found under '
                f'{self.mission_dir}/data - {required_msg}')
        else:
            self.get_logger().info(f'[{group}] using stream "{reader.name}"')
        return reader

    def _declare_and_resolve_lidar(self):
        self.declare_parameter('lidar.enable', True)
        self.declare_parameter(
            'lidar.candidates',
            ['hesai_points_undistorted', 'hesai_points_undistorted_filtered',
             'hesai_points'])
        self.declare_parameter('lidar.topic', '/grandtour/lidar/points')
        self.declare_parameter('lidar.frame_id_override', '')

        if not bool(self.get_parameter('lidar.enable').value):
            return
        candidates = list(self.get_parameter('lidar.candidates').value)
        reader = self._resolve('lidar', candidates, 'skipping lidar')
        if reader is None:
            return

        topic = str(self.get_parameter('lidar.topic').value)
        frame_override = str(
            self.get_parameter('lidar.frame_id_override').value)
        self.streams.append({
            'kind': 'lidar',
            'reader': reader,
            'pub': self.create_publisher(PointCloud2, topic, 10),
            'frame_id': self._frame(frame_override or reader.frame_id),
            'pc2_layout': None,
        })

    def _declare_and_resolve_imu(self):
        self.declare_parameter('imu.enable', True)
        self.declare_parameter(
            'imu.candidates', ['cpt7_imu', 'stim320_imu', 'anymal_imu'])
        self.declare_parameter('imu.topic', '/grandtour/imu')
        self.declare_parameter('imu.frame_id_override', '')

        if not bool(self.get_parameter('imu.enable').value):
            return
        candidates = list(self.get_parameter('imu.candidates').value)
        reader = self._resolve('imu', candidates, 'skipping imu')
        if reader is None:
            return

        topic = str(self.get_parameter('imu.topic').value)
        frame_override = str(self.get_parameter('imu.frame_id_override').value)
        self.streams.append({
            'kind': 'imu',
            'reader': reader,
            'pub': self.create_publisher(Imu, topic, RELIABLE_KEEP_LAST),
            'frame_id': self._frame(frame_override or reader.frame_id),
        })

    def _declare_and_resolve_odom(
            self, group, default_candidates, default_topic,
            default_child_frame):
        self.declare_parameter(f'{group}.enable', True)
        self.declare_parameter(f'{group}.candidates', default_candidates)
        self.declare_parameter(f'{group}.topic', default_topic)
        self.declare_parameter(f'{group}.child_frame_id', default_child_frame)
        self.declare_parameter(f'{group}.publish_tf', False)
        self.declare_parameter(f'{group}.normalize_position', False)

        if not bool(self.get_parameter(f'{group}.enable').value):
            return
        candidates = list(self.get_parameter(f'{group}.candidates').value)
        reader = self._resolve(group, candidates, f'skipping {group}')
        if reader is None:
            return

        topic = str(self.get_parameter(f'{group}.topic').value)
        child_frame_id = self._frame(
            str(self.get_parameter(f'{group}.child_frame_id').value))
        position_origin = np.zeros(3, dtype=np.float64)
        if bool(self.get_parameter(f'{group}.normalize_position').value):
            lidar_stream = next(
                (stream for stream in self.streams
                 if stream['kind'] == 'lidar'),
                None)
            ref_epoch = (
                lidar_stream['reader'].timestamps[0]
                if lidar_stream is not None else reader.timestamps[0])
            origin_idx = min(
                int(np.searchsorted(reader.timestamps, ref_epoch)),
                len(reader) - 1)
            position_origin = np.asarray(
                reader.row('pose_pos', origin_idx), dtype=np.float64)
            self.get_logger().info(
                f'[{group}] translating position origin by '
                f'{tuple(round(float(value), 4) for value in position_origin)} '
                f'at t={reader.timestamps[origin_idx]:.3f}')
        self.streams.append({
            'kind': 'odom',
            'group': group,
            'reader': reader,
            'pub': self.create_publisher(Odometry, topic, RELIABLE_KEEP_LAST),
            'frame_id': self._frame(reader.frame_id),
            'child_frame_id': child_frame_id,
            'publish_tf': bool(self.get_parameter(f'{group}.publish_tf').value),
            'position_origin': position_origin,
        })

    def _declare_and_resolve_anymal_state(self):
        group = 'anymal_state'
        self.declare_parameter(f'{group}.enable', True)
        self.declare_parameter(
            f'{group}.candidates', ['anymal_state_state_estimator'])
        self.declare_parameter(
            f'{group}.odometry_topic', '/grandtour/anymal/state_odometry')
        self.declare_parameter(
            f'{group}.joint_state_topic', '/grandtour/anymal/joint_states')
        self.declare_parameter(f'{group}.child_frame_id', 'base')
        self.declare_parameter(f'{group}.publish_tf', False)

        if not bool(self.get_parameter(f'{group}.enable').value):
            return
        candidates = list(self.get_parameter(f'{group}.candidates').value)
        reader = self._resolve(group, candidates, f'skipping {group}')
        if reader is None:
            return
        if not reader.has_field('joint_positions'):
            self.get_logger().warning(
                f'[{group}] stream "{reader.name}" has no joint_positions '
                'field; only odometry will be published')

        odom_topic = str(self.get_parameter(f'{group}.odometry_topic').value)
        joint_topic = str(
            self.get_parameter(f'{group}.joint_state_topic').value)
        child_frame_id = self._frame(
            str(self.get_parameter(f'{group}.child_frame_id').value))
        self.streams.append({
            'kind': 'anymal_state',
            'reader': reader,
            'pub': self.create_publisher(
                Odometry, odom_topic, RELIABLE_KEEP_LAST),
            'joint_pub': (
                self.create_publisher(
                    JointState, joint_topic, RELIABLE_KEEP_LAST)
                if reader.has_field('joint_positions') else None
            ),
            'frame_id': self._frame(reader.frame_id),
            'child_frame_id': child_frame_id,
            'publish_tf': bool(self.get_parameter(f'{group}.publish_tf').value),
        })

    def _declare_and_resolve_prism_position(self):
        group = 'prism_position'
        self.declare_parameter(f'{group}.enable', True)
        self.declare_parameter(f'{group}.candidates', ['prism_position'])
        self.declare_parameter(
            f'{group}.topic', '/grandtour/gt/prism_position')

        if not bool(self.get_parameter(f'{group}.enable').value):
            return
        candidates = list(self.get_parameter(f'{group}.candidates').value)
        reader = self._resolve(group, candidates, f'skipping {group}')
        if reader is None:
            return
        if not reader.has_field('point'):
            self.get_logger().warning(
                f'[{group}] stream "{reader.name}" has no point field; '
                f'skipping {group}')
            return

        topic = str(self.get_parameter(f'{group}.topic').value)
        self.streams.append({
            'kind': 'point',
            'group': group,
            'reader': reader,
            'pub': self.create_publisher(
                PointStamped, topic, RELIABLE_KEEP_LAST),
            'frame_id': self._frame(reader.frame_id),
        })

    def _declare_and_resolve_ground_truth_tf(self):
        group = 'ground_truth_tf'
        # Default OFF: cpt7_imu already has a static parent (box_base, via
        # metadata/tf.yaml). Also broadcasting enu_origin -> cpt7_imu here
        # would give cpt7_imu two TF parents (TF_MULTIPLE_PARENT). Consume
        # ground truth as the plain cpt7_ie_tc_odometry topic instead, or
        # enable this only if you also disable/rewire the static cpt7_imu
        # entry so the frame has a single parent.
        self.declare_parameter(f'{group}.enable', False)
        self.declare_parameter(f'{group}.candidates', ['cpt7_ie_tc_tf'])
        # The IE-TC filter's parent frame is not itself a GrandTour stream;
        # 'enu_origin' matches cpt7_ie_tc_odometry's frame_id. Verify against
        # the mission you are replaying if trajectories look offset.
        self.declare_parameter(f'{group}.parent_frame_id', 'enu_origin')

        if not bool(self.get_parameter(f'{group}.enable').value):
            return
        candidates = list(self.get_parameter(f'{group}.candidates').value)
        reader = self._resolve(group, candidates, f'skipping {group}')
        if reader is None:
            return

        parent_frame_id = self._frame(
            str(self.get_parameter(f'{group}.parent_frame_id').value))
        self.streams.append({
            'kind': 'tf_dynamic',
            'reader': reader,
            'parent_frame_id': parent_frame_id,
            'child_frame_id': self._frame(reader.frame_id),
        })

    def _broadcast_static_tf(self):
        self.declare_parameter('static_tf.enable', True)
        if not bool(self.get_parameter('static_tf.enable').value):
            return

        try:
            frames = load_static_tf_tree(self.mission_dir)
        except OSError as error:
            self.get_logger().warning(
                f'[static_tf] could not read metadata/tf.yaml: {error}')
            return
        self._static_frames = frames

        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child_frame_id, entry in frames.items():
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._frame(entry['base_frame_id'])
            transform.child_frame_id = self._frame(child_frame_id)
            tx, ty, tz = entry['translation']
            rx, ry, rz, rw = entry['rotation']
            transform.transform.translation.x = tx
            transform.transform.translation.y = ty
            transform.transform.translation.z = tz
            transform.transform.rotation.x = rx
            transform.transform.rotation.y = ry
            transform.transform.rotation.z = rz
            transform.transform.rotation.w = rw
            transforms.append(transform)

        self._send_static_transforms(transforms)
        self.get_logger().info(
            f'[static_tf] broadcast {len(transforms)} static transforms '
            'from metadata/tf.yaml')

    def _send_static_transforms(self, transforms):
        """Publish the complete static tree so late subscribers receive it."""
        self._static_transforms.extend(transforms)
        if self._static_tf_broadcaster is None:
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        # StaticTransformBroadcaster uses transient-local durability. Send the
        # cumulative set each time so its last retained sample contains every
        # transform, including alignments added after the sensor extrinsics.
        self._static_tf_broadcaster.sendTransform(self._static_transforms)

    def _broadcast_ground_truth_alignment(self):
        """Bridge the ground-truth (enu_origin) and SLAM (odom) trees.

        cpt7_ie_tc_odometry/tf lives in enu_origin, a georeferenced ENU
        frame unrelated to anymal_state_odometry's "odom" (an arbitrary
        frame fixed at wherever the leg-odometry filter was initialized).
        Without a link between them, ground truth can never be transformed
        into the SLAM tree's fixed frame (e.g. by an RViz Odometry display),
        which just silently drops every message instead of rendering it.

        This computes a single best-effort static transform odom -> enu_origin
        by matching each stream's pose at the closest available sample to
        their shared start time, then composing through the known static
        base -> box_base -> body chain (metadata/tf.yaml). It is a one-shot
        alignment for VISUALIZATION, not a calibrated registration - accuracy
        is bounded by the two streams' sample-rate mismatch at that instant.
        """
        self.declare_parameter('ground_truth_alignment.enable', True)
        self.declare_parameter(
            'ground_truth_alignment.body_frame_id', 'cpt7_imu')
        if not bool(self.get_parameter('ground_truth_alignment.enable').value):
            return

        anymal_stream = next(
            (s for s in self.streams if s.get('group') == 'anymal_odometry'),
            None)
        gt_stream = next(
            (s for s in self.streams
             if s.get('group') == 'ground_truth_odometry'),
            None)
        if anymal_stream is None or gt_stream is None:
            self.get_logger().warning(
                '[ground_truth_alignment] needs both anymal_odometry and '
                'ground_truth_odometry resolved; skipping. Ground truth '
                'will not be TF-transformable into the SLAM tree.')
            return

        body_frame_id = str(
            self.get_parameter('ground_truth_alignment.body_frame_id').value)
        box_base = self._static_frames.get('box_base')
        body = self._static_frames.get(body_frame_id)
        if (box_base is None or body is None
                or body['base_frame_id'] != 'box_base'):
            self.get_logger().warning(
                '[ground_truth_alignment] static_tf tree missing "box_base" '
                f'or "{body_frame_id}" as its child; skipping (enable '
                'static_tf, or check ground_truth_alignment.body_frame_id).')
            return

        anymal_reader = anymal_stream['reader']
        gt_reader = gt_stream['reader']
        ref_epoch = max(anymal_reader.timestamps[0], gt_reader.timestamps[0])
        anymal_idx = min(
            int(np.searchsorted(anymal_reader.timestamps, ref_epoch)),
            len(anymal_reader) - 1)
        gt_idx = min(
            int(np.searchsorted(gt_reader.timestamps, ref_epoch)),
            len(gt_reader) - 1)

        # T(odom, base) at the matched sample.
        anymal_position = self._position_for_stream(
            anymal_stream, anymal_idx)
        t_odom_base = (
            tuple(anymal_position.tolist()),
            tuple(anymal_reader.row('pose_orien', anymal_idx).tolist()),
        )
        # T(base, box_base) and T(box_base, body): static (metadata/tf.yaml).
        t_base_boxbase = (
            tuple(box_base['translation']), tuple(box_base['rotation']))
        t_boxbase_body = (tuple(body['translation']), tuple(body['rotation']))
        # T(enu_origin, body) at the matched sample, inverted to T(body, enu_origin).
        t_enu_body = (
            tuple(gt_reader.row('pose_pos', gt_idx).tolist()),
            tuple(gt_reader.row('pose_orien', gt_idx).tolist()),
        )
        t_body_enu = geometry.invert(t_enu_body)

        t_odom_boxbase = geometry.compose(t_odom_base, t_base_boxbase)
        t_odom_body = geometry.compose(t_odom_boxbase, t_boxbase_body)
        translation, rotation = geometry.compose(t_odom_body, t_body_enu)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = anymal_stream['frame_id']
        transform.child_frame_id = gt_stream['frame_id']
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self._send_static_transforms([transform])
        self.get_logger().info(
            f'[ground_truth_alignment] broadcast static '
            f'{transform.header.frame_id} -> {transform.child_frame_id} '
            f'(one-shot alignment at t~{ref_epoch:.3f}, translation='
            f'{tuple(round(v, 3) for v in translation)}) - a best-effort '
            'visualization aid, not a calibrated registration.')

    def _broadcast_prism_alignment(self):
        """Connect Leica prism coordinates to the NovAtel ENU frame.

        GrandTour does not provide a Leica-to-ENU transform. For live RViz
        visualization, fit one rigid transform over timestamp-matched Leica
        prism and NovAtel body positions. The prism-to-body lever arm is not
        published by the dataset, so this remains a visualization alignment,
        not a calibration or a replacement for raw-reference evaluation.
        """
        group = 'prism_alignment'
        self.declare_parameter(f'{group}.enable', True)
        self.declare_parameter(f'{group}.max_time_difference', 0.01)
        if not bool(self.get_parameter(f'{group}.enable').value):
            return

        prism_stream = next(
            (stream for stream in self.streams
             if stream.get('group') == 'prism_position'), None)
        gt_stream = next(
            (stream for stream in self.streams
             if stream.get('group') == 'ground_truth_odometry'), None)
        if prism_stream is None or gt_stream is None:
            self.get_logger().warning(
                '[prism_alignment] needs both prism_position and '
                'ground_truth_odometry; skipping Leica frame alignment.')
            return

        prism_reader = prism_stream['reader']
        gt_reader = gt_stream['reader']
        max_difference = float(
            self.get_parameter(f'{group}.max_time_difference').value)
        prism_indices, gt_indices, deltas = (
            alignment.nearest_timestamp_matches(
                prism_reader.timestamps, gt_reader.timestamps,
                max_difference))
        if prism_indices.size < 3:
            self.get_logger().warning(
                f'[prism_alignment] only {prism_indices.size} timestamp '
                f'matches within {max_difference:.3f} s; need at least 3.')
            return

        prism_points = np.asarray(
            prism_reader.field('point')[:], dtype=np.float64)[prism_indices]
        novatel_points = np.stack([
            gt_reader.row('pose_pos', int(index)) for index in gt_indices
        ]).astype(np.float64)
        finite = (
            np.isfinite(prism_points).all(axis=1)
            & np.isfinite(novatel_points).all(axis=1))
        if np.count_nonzero(finite) < 3:
            self.get_logger().warning(
                '[prism_alignment] fewer than 3 finite matched point pairs; '
                'skipping Leica frame alignment.')
            return

        translation, rotation, rmse = alignment.fit_rigid_transform(
            prism_points[finite], novatel_points[finite])
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = gt_stream['frame_id']
        transform.child_frame_id = prism_stream['frame_id']
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self._send_static_transforms([transform])
        self.get_logger().info(
            f'[prism_alignment] broadcast static '
            f'{transform.header.frame_id} -> {transform.child_frame_id} '
            f'from {np.count_nonzero(finite)} matched samples '
            f'(max dt={float(np.max(deltas)):.4f} s, fit RMSE={rmse:.3f} m) '
            '- visualization only; the unknown prism lever arm remains.')

    # ------------------------------------------------------------------
    # Timeline
    # ------------------------------------------------------------------

    def _build_timeline(self):
        all_timestamps, all_stream_idx, all_row_idx = [], [], []
        for stream_idx, stream in enumerate(self.streams):
            timestamps = stream['reader'].timestamps
            all_timestamps.append(timestamps)
            all_stream_idx.append(np.full(timestamps.shape[0], stream_idx))
            all_row_idx.append(np.arange(timestamps.shape[0]))

        if not all_timestamps:
            self.timeline_timestamps = np.empty(0, dtype=np.float64)
            self.timeline_stream_idx = np.empty(0, dtype=np.int64)
            self.timeline_row_idx = np.empty(0, dtype=np.int64)
            return

        timestamps = np.concatenate(all_timestamps)
        stream_idx = np.concatenate(all_stream_idx)
        row_idx = np.concatenate(all_row_idx)
        order = np.argsort(timestamps, kind='stable')

        self.timeline_timestamps = timestamps[order]
        self.timeline_stream_idx = stream_idx[order]
        self.timeline_row_idx = row_idx[order]

        if self.start_offset > 0.0 and self.timeline_timestamps.size:
            cutoff = self.timeline_timestamps[0] + self.start_offset
            start = int(
                np.searchsorted(self.timeline_timestamps, cutoff))
            self.timeline_timestamps = self.timeline_timestamps[start:]
            self.timeline_stream_idx = self.timeline_stream_idx[start:]
            self.timeline_row_idx = self.timeline_row_idx[start:]

        self.get_logger().info(
            f'Merged timeline: {self.timeline_timestamps.size} events across '
            f'{len(self.streams)} stream(s)')

    # ------------------------------------------------------------------
    # Playback loop
    # ------------------------------------------------------------------

    def _playback(self):
        if self.timeline_timestamps.size == 0:
            self.get_logger().warning('Empty timeline; nothing to play back.')
            return

        while not self._stop and rclpy.ok():
            start_epoch = None
            start_wall = None

            for i in range(self.timeline_timestamps.size):
                if self._stop or not rclpy.ok():
                    break

                epoch = float(self.timeline_timestamps[i])
                if start_epoch is None:
                    start_epoch = epoch
                    start_wall = time.monotonic()

                if self.use_wall_clock_pacing:
                    target = start_wall + (
                        epoch - start_epoch) / self.playback_rate
                    remaining = target - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)

                if self._stop or not rclpy.ok():
                    break

                self._publish_clock(epoch)
                stream = self.streams[int(self.timeline_stream_idx[i])]
                row = int(self.timeline_row_idx[i])
                try:
                    self._dispatch(stream, row, epoch)
                except Exception as error:  # noqa: BLE001 - keep playback alive
                    self.get_logger().error(
                        f"[{stream['kind']}/{stream['reader'].name}] row "
                        f'{row} failed: {error}')

            if not self.loop:
                break

        self.get_logger().info('Mission playback complete.')

    def _publish_clock(self, epoch):
        if self.pub_clock is None:
            return
        msg = Clock()
        msg.clock = stamp_from_epoch(epoch)
        self.pub_clock.publish(msg)

    def _dispatch(self, stream, row, epoch):
        kind = stream['kind']
        if kind == 'lidar':
            self._publish_lidar(stream, row, epoch)
        elif kind == 'imu':
            self._publish_imu(stream, row, epoch)
        elif kind == 'odom':
            self._publish_odom(stream, row, epoch)
        elif kind == 'anymal_state':
            self._publish_anymal_state(stream, row, epoch)
        elif kind == 'point':
            self._publish_point(stream, row, epoch)
        elif kind == 'tf_dynamic':
            self._publish_tf_dynamic(stream, row, epoch)

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _publish_lidar(self, stream, row, epoch):
        reader = stream['reader']
        points = reader.row('points', row)
        valid = int(reader.row('valid', row)[0])

        if stream['pc2_layout'] is None:
            stream['pc2_layout'] = self._build_pc2_layout(reader)
        fields, np_dtype, point_step = stream['pc2_layout']

        cloud_arr = np.zeros(valid, dtype=np_dtype)
        cloud_arr['x'] = points[:valid, 0]
        cloud_arr['y'] = points[:valid, 1]
        cloud_arr['z'] = points[:valid, 2]
        if reader.has_field('intensity'):
            cloud_arr['intensity'] = reader.row('intensity', row)[:valid]
        if reader.has_field('time'):
            per_point_time = reader.row('time', row)[:valid]
            # 't': scan-relative offset (float32; the usual lidar-driver
            # convention, e.g. Velodyne/Ouster) for generic PCL/rviz
            # consumers.
            cloud_arr['t'] = (per_point_time - epoch).astype(np.float32)
            # 'timestamp': absolute epoch seconds (float64) - what
            # scan_deskewer (dotX-Automation/scan_deskewer) expects, since
            # it fuses point times against an absolute-epoch IMU buffer.
            # float32 cannot hold epoch time (~1.7e9 s) with sub-ms
            # precision, hence the separate float64 field.
            cloud_arr['timestamp'] = per_point_time
        if reader.has_field('ring'):
            cloud_arr['ring'] = reader.row('ring', row)[:valid]

        cloud = PointCloud2()
        cloud.header.stamp = stamp_from_epoch(epoch)
        cloud.header.frame_id = stream['frame_id']
        cloud.height = 1
        cloud.width = valid
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * valid
        cloud.is_dense = True
        cloud.data = cloud_arr.tobytes()
        stream['pub'].publish(cloud)

    @staticmethod
    def _build_pc2_layout(reader):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity', offset=12, datatype=PointField.FLOAT32,
                count=1),
        ]
        dtype_fields = [
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')]
        offset = 16

        if reader.has_field('time'):
            # offset 16 is 8-byte aligned, so 'timestamp' (float64) is
            # placed before the narrower 't'/'ring' fields.
            fields.append(PointField(
                name='timestamp', offset=offset, datatype=PointField.FLOAT64,
                count=1))
            dtype_fields.append(('timestamp', '<f8'))
            offset += 8
            fields.append(PointField(
                name='t', offset=offset, datatype=PointField.FLOAT32, count=1))
            dtype_fields.append(('t', '<f4'))
            offset += 4
        if reader.has_field('ring'):
            fields.append(PointField(
                name='ring', offset=offset, datatype=PointField.UINT16,
                count=1))
            dtype_fields.append(('ring', '<u2'))
            offset += 2
            if offset % 4 != 0:
                pad = 4 - (offset % 4)
                dtype_fields.append(('_pad', f'V{pad}'))
                offset += pad

        return fields, np.dtype(dtype_fields), offset

    def _publish_imu(self, stream, row, epoch):
        reader = stream['reader']
        orientation = reader.row('orien', row)
        angular_velocity = reader.row('ang_vel', row)
        linear_acceleration = reader.row('lin_acc', row)
        orientation_cov = reader.row('orien_cov', row)
        angular_velocity_cov = reader.row('ang_vel_cov', row)
        linear_acceleration_cov = reader.row('lin_acc_cov', row)

        msg = Imu()
        msg.header.stamp = stamp_from_epoch(epoch)
        msg.header.frame_id = stream['frame_id']

        # REP-145: signal "orientation unknown" with covariance[0] = -1
        # rather than publishing a bogus all-zero quaternion.
        if not np.any(orientation):
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
        else:
            msg.orientation.x = float(orientation[0])
            msg.orientation.y = float(orientation[1])
            msg.orientation.z = float(orientation[2])
            msg.orientation.w = float(orientation[3])
            msg.orientation_covariance = (
                orientation_cov.flatten().astype(np.float64).tolist())

        msg.angular_velocity.x = float(angular_velocity[0])
        msg.angular_velocity.y = float(angular_velocity[1])
        msg.angular_velocity.z = float(angular_velocity[2])
        msg.angular_velocity_covariance = (
            angular_velocity_cov.flatten().astype(np.float64).tolist())

        msg.linear_acceleration.x = float(linear_acceleration[0])
        msg.linear_acceleration.y = float(linear_acceleration[1])
        msg.linear_acceleration.z = float(linear_acceleration[2])
        msg.linear_acceleration_covariance = (
            linear_acceleration_cov.flatten().astype(np.float64).tolist())

        stream['pub'].publish(msg)

    @staticmethod
    def _position_for_stream(stream, row):
        position = np.asarray(
            stream['reader'].row('pose_pos', row), dtype=np.float64)
        return position - stream.get('position_origin', 0.0)

    def _fill_odometry(self, msg, stream, row, epoch):
        reader = stream['reader']
        position = self._position_for_stream(stream, row)
        # Assumed (x, y, z, w) component order (not confirmed against the
        # upstream GrandTour schema docs) - see README "Unverified
        # assumptions".
        orientation = reader.row('pose_orien', row)
        linear = reader.row('twist_lin', row)
        angular = reader.row('twist_ang', row)

        msg.header.stamp = stamp_from_epoch(epoch)
        msg.header.frame_id = stream['frame_id']
        msg.child_frame_id = stream['child_frame_id']

        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.x = float(orientation[0])
        msg.pose.pose.orientation.y = float(orientation[1])
        msg.pose.pose.orientation.z = float(orientation[2])
        msg.pose.pose.orientation.w = float(orientation[3])
        # Some streams (e.g. anymal_state_state_estimator) don't carry a
        # covariance; leave the message's default all-zero covariance,
        # which this codebase already treats as "unknown" elsewhere (see
        # ros2_openloris_publishers/odometry_tf_node.py).
        if reader.has_field('pose_cov'):
            msg.pose.covariance = (
                reader.row('pose_cov', row)
                .flatten().astype(np.float64).tolist())

        msg.twist.twist.linear.x = float(linear[0])
        msg.twist.twist.linear.y = float(linear[1])
        msg.twist.twist.linear.z = float(linear[2])
        msg.twist.twist.angular.x = float(angular[0])
        msg.twist.twist.angular.y = float(angular[1])
        msg.twist.twist.angular.z = float(angular[2])
        if reader.has_field('twist_cov'):
            msg.twist.covariance = (
                reader.row('twist_cov', row)
                .flatten().astype(np.float64).tolist())
        return position, orientation

    def _broadcast_tf(self, epoch, frame_id, child_frame_id, position, orientation):
        transform = TransformStamped()
        transform.header.stamp = stamp_from_epoch(epoch)
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(orientation[0])
        transform.transform.rotation.y = float(orientation[1])
        transform.transform.rotation.z = float(orientation[2])
        transform.transform.rotation.w = float(orientation[3])
        self.tf_broadcaster.sendTransform(transform)

    def _publish_odom(self, stream, row, epoch):
        msg = Odometry()
        position, orientation = self._fill_odometry(
            msg, stream, row, epoch)
        stream['pub'].publish(msg)
        if stream['publish_tf']:
            self._broadcast_tf(
                epoch, stream['frame_id'], stream['child_frame_id'],
                position, orientation)

    def _publish_anymal_state(self, stream, row, epoch):
        reader = stream['reader']
        msg = Odometry()
        position, orientation = self._fill_odometry(
            msg, stream, row, epoch)
        stream['pub'].publish(msg)
        if stream['publish_tf']:
            self._broadcast_tf(
                epoch, stream['frame_id'], stream['child_frame_id'],
                position, orientation)

        if stream['joint_pub'] is not None:
            joint_msg = JointState()
            joint_msg.header.stamp = stamp_from_epoch(epoch)
            joint_msg.name = ANYMAL_JOINT_NAMES
            joint_msg.position = (
                reader.row('joint_positions', row)
                .astype(np.float64).tolist())
            joint_msg.velocity = (
                reader.row('joint_velocities', row)
                .astype(np.float64).tolist())
            joint_msg.effort = (
                reader.row('joint_efforts', row)
                .astype(np.float64).tolist())
            stream['joint_pub'].publish(joint_msg)

    def _publish_point(self, stream, row, epoch):
        point = stream['reader'].row('point', row)
        msg = PointStamped()
        msg.header.stamp = stamp_from_epoch(epoch)
        msg.header.frame_id = stream['frame_id']
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])
        stream['pub'].publish(msg)

    def _publish_tf_dynamic(self, stream, row, epoch):
        reader = stream['reader']
        translation = reader.row('translation', row)
        rotation = reader.row('rotation', row)
        self._broadcast_tf(
            epoch, stream['parent_frame_id'], stream['child_frame_id'],
            translation, rotation)


def main(args=None):
    rclpy.init(args=args)
    node = GrandTourPublisherNode()

    try:
        if hasattr(node, '_thread'):
            node._thread.join()
    except KeyboardInterrupt:
        node._stop = True
        if hasattr(node, '_thread'):
            node._thread.join(timeout=1.0)
    except ExternalShutdownException:
        pass
    finally:
        node._stop = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
