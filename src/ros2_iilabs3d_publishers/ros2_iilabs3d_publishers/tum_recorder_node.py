#!/usr/bin/env python3
"""
TUM trajectory recorder for the IILABS3D benchmark.

Subscribes to the SLAM pose estimate and appends each pose to a TUM file
(timestamp x y z qx qy qz qw, timestamps from the message headers, i.e. bag
epoch time), ready for:

  iilabs3d eval <sequence_dir>/ground_truth.tum <output_file>.tum

The SLAM pose is expected in the same reference frame as the ground truth
(base_link); LIO-SAM's /lio_sam/map_optimization/pose already is, so no
`iilabs3d correct-frame` step is needed.
"""
from datetime import datetime
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class IILABS3DTumRecorderNode(Node):

    def __init__(self):
        super().__init__('iilabs3d_tum_recorder')

        self.declare_parameter('pose_topic', '/lio_sam/map_optimization/pose')

        # Message type on pose_topic: 'pose_with_covariance', 'pose' or 'odometry'
        self.declare_parameter('pose_msg_type', 'pose_with_covariance')

        # Directory the .tum file is written to (created if missing)
        self.declare_parameter('output_dir', '')

        # Output file is <slam_name>.tum, or <slam_name>_<datetime>.tum when
        # append_datetime is set, so repeated runs do not overwrite each other.
        self.declare_parameter('slam_name', 'lio_sam')
        self.declare_parameter('append_datetime', True)

        pose_topic = str(self.get_parameter('pose_topic').value)
        pose_msg_type = str(self.get_parameter('pose_msg_type').value)
        output_dir = str(self.get_parameter('output_dir').value)
        slam_name = str(self.get_parameter('slam_name').value)
        append_datetime = bool(self.get_parameter('append_datetime').value)

        if not output_dir:
            self.get_logger().error(
                'output_dir parameter not set. Pass: '
                '--ros-args -p output_dir:=/path/to/results'
            )
            return

        name = slam_name
        if append_datetime:
            name += datetime.now().strftime('_%Y-%m-%d_%H-%M-%S')
        self.output_file = os.path.join(output_dir, name + '.tum')

        msg_types = {
            'pose_with_covariance': (PoseWithCovarianceStamped, self._on_pose_with_cov),
            'pose': (PoseStamped, self._on_pose),
            'odometry': (Odometry, self._on_odometry),
        }
        if pose_msg_type not in msg_types:
            self.get_logger().error(
                f"Invalid pose_msg_type '{pose_msg_type}', "
                f"expected one of {list(msg_types)}"
            )
            return

        self._file = None
        self._count = 0

        msg_class, callback = msg_types[pose_msg_type]
        self.sub = self.create_subscription(msg_class, pose_topic, callback, 100)

        self.get_logger().info(
            f'Recording {msg_class.__name__} poses from {pose_topic} '
            f'to {self.output_file}'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_pose_with_cov(self, msg: PoseWithCovarianceStamped):
        self._write(msg.header, msg.pose.pose)

    def _on_pose(self, msg: PoseStamped):
        self._write(msg.header, msg.pose)

    def _on_odometry(self, msg: Odometry):
        self._write(msg.header, msg.pose.pose)

    # ------------------------------------------------------------------
    # TUM file writing
    # ------------------------------------------------------------------

    def _write(self, header, pose):
        if self._file is None:
            # Open lazily on the first pose so aborted runs leave no file
            os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
            self._file = open(self.output_file, 'w')

        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        p = pose.position
        q = pose.orientation

        self._file.write(
            f'{t:.9f} '
            f'{p.x:.9f} {p.y:.9f} {p.z:.9f} '
            f'{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n'
        )
        self._file.flush()
        self._count += 1

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None
            self.get_logger().info(
                f'Saved {self._count} poses to {self.output_file}'
            )

    def destroy_node(self):
        self.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IILABS3DTumRecorderNode()

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
