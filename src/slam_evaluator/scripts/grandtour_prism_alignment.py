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

"""Register GrandTour Leica prism points and correct their lever arm."""

from collections import deque

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


RELIABLE_QOS = QoSProfile(
    depth=200,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def normalized_quaternion(orientation):
    quaternion = np.asarray((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ), dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise RuntimeError('ground-truth odometry contains a zero quaternion')
    return quaternion / norm


def initial_rigid_transform(source, target):
    """Return a no-scale Kabsch initialization mapping source to target."""
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    u_matrix, _, vt_matrix = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def calibrate_prism(points, body_positions, body_quaternions):
    """Fit Leica registration and the rotating prism-to-body lever arm.

    The model is identical to the ETH-1 offline prism extraction model::

        p_body = R_global p_leica + t_global - R_body lever_arm

    Here the lever arm is expressed in the IE-TC odometry child frame.
    """
    initial_rotation, initial_translation = initial_rigid_transform(
        points, body_positions)
    body_rotations = Rotation.from_quat(body_quaternions).as_matrix()
    initial_parameters = np.concatenate((
        Rotation.from_matrix(initial_rotation).as_rotvec(),
        initial_translation,
        np.zeros(3, dtype=np.float64),
    ))

    def residuals(parameters):
        global_rotation = Rotation.from_rotvec(parameters[:3]).as_matrix()
        global_translation = parameters[3:6]
        lever_arm = parameters[6:9]
        prism_positions = points @ global_rotation.T + global_translation
        rotated_lever_arm = np.einsum(
            'nij,j->ni', body_rotations, lever_arm)
        return (
            prism_positions - rotated_lever_arm - body_positions
        ).ravel()

    solution = least_squares(
        residuals,
        initial_parameters,
        method='trf',
        max_nfev=1000,
        ftol=1.0e-12,
        xtol=1.0e-12,
        gtol=1.0e-12,
    )
    if not solution.success:
        raise RuntimeError(f'Leica calibration failed: {solution.message}')

    global_rotation = Rotation.from_rotvec(solution.x[:3]).as_matrix()
    global_translation = solution.x[3:6]
    lever_arm = solution.x[6:9]
    residual_vectors = residuals(solution.x).reshape((-1, 3))
    rmse = float(np.sqrt(np.mean(np.sum(
        residual_vectors * residual_vectors, axis=1))))
    return global_rotation, global_translation, lever_arm, rmse


class GrandTourPrismAlignment(Node):
    """Calibrate Leica samples and publish lever-arm-corrected RTK points."""

    def __init__(self):
        super().__init__('grandtour_prism_alignment')
        self.declare_parameter(
            'prism_topic_in', '/slam_evaluator/grandtour/prism_raw')
        self.declare_parameter(
            'prism_topic_out', '/boxi/ap20/prism_position')
        self.declare_parameter(
            'ground_truth_topic', '/boxi/inertial_explorer/tc/odometry')
        self.declare_parameter('max_time_difference', 0.01)
        self.declare_parameter('minimum_matches', 200)
        self.declare_parameter('minimum_baseline', 1.0)
        self.declare_parameter('minimum_rotation', 0.1)
        self.declare_parameter('maximum_output_residual', 0.5)

        prism_topic_in = str(self.get_parameter('prism_topic_in').value)
        prism_topic_out = str(self.get_parameter('prism_topic_out').value)
        ground_truth_topic = str(
            self.get_parameter('ground_truth_topic').value)
        self.max_time_difference_ns = int(
            float(self.get_parameter('max_time_difference').value) * 1.0e9)
        self.minimum_matches = int(
            self.get_parameter('minimum_matches').value)
        self.minimum_baseline = float(
            self.get_parameter('minimum_baseline').value)
        self.minimum_rotation = float(
            self.get_parameter('minimum_rotation').value)
        self.maximum_output_residual = float(
            self.get_parameter('maximum_output_residual').value)

        self.publisher = self.create_publisher(
            PointStamped, prism_topic_out, RELIABLE_QOS)
        self.prism_subscription = self.create_subscription(
            PointStamped, prism_topic_in, self._on_prism, RELIABLE_QOS)
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            ground_truth_topic,
            self._on_ground_truth,
            RELIABLE_QOS,
        )

        self.ground_truth = deque(maxlen=2000)
        self.unmatched_prisms = deque()
        self.queued_prisms = deque()
        self.live_pending_prisms = deque()
        self.matched_orientations = {}
        self.prism_points = []
        self.ground_truth_points = []
        self.ground_truth_orientations = []
        self.ground_truth_frame = None
        self.registration_rotation = None
        self.registration_translation = None
        self.lever_arm = None
        self.aligned = False

        self.get_logger().info(
            f'Holding {prism_topic_in} while calibrating Leica registration '
            f'and prism lever arm against {ground_truth_topic}; corrected '
            f'points will be published on {prism_topic_out}')

    def _on_ground_truth(self, message):
        timestamp = stamp_nanoseconds(message.header.stamp)
        position = message.pose.pose.position
        orientation = normalized_quaternion(message.pose.pose.orientation)
        self.ground_truth.append((
            timestamp,
            np.asarray((position.x, position.y, position.z)),
            orientation,
        ))
        self.ground_truth_frame = message.header.frame_id
        if self.aligned:
            self._publish_pending_live_prisms()
        else:
            self._match_pending_prisms()

    def _on_prism(self, message):
        if self.aligned:
            self.live_pending_prisms.append(message)
            self._publish_pending_live_prisms()
            return

        self.queued_prisms.append(message)
        self.unmatched_prisms.append(message)
        self._match_pending_prisms()

    def _nearest_ground_truth(self, timestamp):
        if not self.ground_truth:
            return None
        sample = min(
            self.ground_truth,
            key=lambda item: abs(item[0] - timestamp),
        )
        return sample, abs(sample[0] - timestamp)

    def _match_pending_prisms(self):
        if not self.ground_truth or not self.unmatched_prisms:
            return

        remaining = deque()
        newest_ground_truth = self.ground_truth[-1][0]
        for message in self.unmatched_prisms:
            timestamp = stamp_nanoseconds(message.header.stamp)
            nearest = self._nearest_ground_truth(timestamp)
            (nearest_timestamp, nearest_position,
             nearest_orientation), difference = nearest
            if difference <= self.max_time_difference_ns:
                if timestamp not in self.matched_orientations:
                    point = message.point
                    self.prism_points.append(np.asarray(
                        (point.x, point.y, point.z), dtype=np.float64))
                    self.ground_truth_points.append(nearest_position)
                    self.ground_truth_orientations.append(
                        nearest_orientation)
                    self.matched_orientations[timestamp] = (
                        nearest_position, nearest_orientation)
            elif timestamp + self.max_time_difference_ns >= newest_ground_truth:
                remaining.append(message)
        self.unmatched_prisms = remaining
        self._try_alignment()

    def _try_alignment(self):
        if self.aligned or len(self.prism_points) < self.minimum_matches:
            return

        source = np.asarray(self.prism_points, dtype=np.float64)
        target = np.asarray(self.ground_truth_points, dtype=np.float64)
        orientations = np.asarray(
            self.ground_truth_orientations, dtype=np.float64)
        source_baseline = float(np.linalg.norm(np.ptp(source, axis=0)))
        target_baseline = float(np.linalg.norm(np.ptp(target, axis=0)))
        if min(source_baseline, target_baseline) < self.minimum_baseline:
            return

        body_rotations = Rotation.from_quat(orientations)
        relative_rotations = body_rotations[0].inv() * body_rotations
        rotation_range = float(np.max(relative_rotations.magnitude()))
        if rotation_range < self.minimum_rotation:
            return

        result = calibrate_prism(source, target, orientations)
        (self.registration_rotation,
         self.registration_translation,
         self.lever_arm,
         rmse) = result
        self.aligned = True

        printable_lever_arm = tuple(
            round(float(value), 6) for value in self.lever_arm)
        self.get_logger().info(
            f'Calibrated Leica registration and rotating lever arm from '
            f'{source.shape[0]} matched samples (fit RMSE={rmse:.3f} m, '
            f'lever arm={printable_lever_arm} m); releasing corrected '
            f'points in frame {self.ground_truth_frame!r}')
        self._release_queued_prisms()

    def _correct_and_publish(
            self, message, reference_position, body_quaternion):
        point = np.asarray((
            message.point.x,
            message.point.y,
            message.point.z,
        ), dtype=np.float64)
        prism_position = (
            self.registration_rotation @ point
            + self.registration_translation
        )
        body_rotation = Rotation.from_quat(body_quaternion).as_matrix()
        body_position = prism_position - body_rotation @ self.lever_arm
        residual = float(np.linalg.norm(body_position - reference_position))
        if (
            self.maximum_output_residual > 0.0
            and residual > self.maximum_output_residual
        ):
            self.get_logger().warning(
                f'Dropped Leica sample with {residual:.3f} m residual '
                f'(limit={self.maximum_output_residual:.3f} m)')
            return

        corrected = PointStamped()
        corrected.header.stamp = message.header.stamp
        corrected.header.frame_id = self.ground_truth_frame
        corrected.point.x = float(body_position[0])
        corrected.point.y = float(body_position[1])
        corrected.point.z = float(body_position[2])
        self.publisher.publish(corrected)

    def _release_queued_prisms(self):
        dropped = 0
        while self.queued_prisms:
            message = self.queued_prisms.popleft()
            timestamp = stamp_nanoseconds(message.header.stamp)
            matched_sample = self.matched_orientations.get(timestamp)
            if matched_sample is None:
                dropped += 1
                continue
            reference_position, orientation = matched_sample
            self._correct_and_publish(
                message, reference_position, orientation)
        self.unmatched_prisms.clear()
        self.matched_orientations.clear()
        if dropped:
            self.get_logger().warning(
                f'Dropped {dropped} queued Leica samples without a '
                f'time-matched RTK orientation')

    def _publish_pending_live_prisms(self):
        if not self.ground_truth or not self.live_pending_prisms:
            return

        remaining = deque()
        newest_ground_truth = self.ground_truth[-1][0]
        for message in self.live_pending_prisms:
            timestamp = stamp_nanoseconds(message.header.stamp)
            nearest = self._nearest_ground_truth(timestamp)
            (_, reference_position, orientation), difference = nearest
            if difference <= self.max_time_difference_ns:
                self._correct_and_publish(
                    message, reference_position, orientation)
            elif timestamp + self.max_time_difference_ns >= newest_ground_truth:
                remaining.append(message)
        self.live_pending_prisms = remaining


def main(args=None):
    rclpy.init(args=args)
    node = GrandTourPrismAlignment()
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
