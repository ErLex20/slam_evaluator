#!/usr/bin/env python3
"""
Ground truth publisher for the IILABS3D dataset.

Reads a TUM trajectory file (timestamp x y z qx qy qz qw, timestamps in bag
epoch time) and replays it against the ROS clock. The bag is played with
`ros2 bag play --clock`, so with use_sim_time this node publishes exactly the
ground truth poses whose timestamps have already passed, allowing an online
visual comparison with the SLAM estimate in RViz.

Publishes:
  - nav_msgs/Path        (growing, distance-decimated)  for RViz
  - geometry_msgs/PoseStamped (latest ground truth pose)
  - TF map -> eve/base_link_gt (optional)
"""
from bisect import bisect_right
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time

from tf2_ros import TransformBroadcaster


class IILABS3DGTPublisherNode(Node):

    def __init__(self):
        super().__init__('iilabs3d_gt_publisher')

        self.declare_parameter('gt_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'eve/base_link_gt')
        self.declare_parameter('pose_topic_out', '/iilabs3d/ground_truth/pose')
        self.declare_parameter('path_topic_out', '/iilabs3d/ground_truth/path')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 10.0)

        # Minimum translation between consecutive path poses; keeps the
        # RViz Path message small (the raw file is ~250 Hz).
        self.declare_parameter('path_min_distance', 0.05)

        gt_file = str(self.get_parameter('gt_file').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        pose_topic_out = str(self.get_parameter('pose_topic_out').value)
        path_topic_out = str(self.get_parameter('path_topic_out').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.path_min_distance = float(self.get_parameter('path_min_distance').value)

        if not gt_file:
            self.get_logger().error(
                'gt_file parameter not set. Pass: '
                '--ros-args -p gt_file:=/path/to/ground_truth.tum'
            )
            return

        self.stamps_ns, self.poses = self._load_tum(gt_file)
        if not self.stamps_ns:
            self.get_logger().error(f'No poses loaded from {gt_file}')
            return

        self.get_logger().info(
            f'Loaded {len(self.stamps_ns)} ground truth poses from {gt_file} '
            f'({(self.stamps_ns[-1] - self.stamps_ns[0]) / 1e9:.1f} s)'
        )

        self.pub_pose = self.create_publisher(PoseStamped, pose_topic_out, 10)
        self.pub_path = self.create_publisher(Path, path_topic_out, 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self._idx = 0           # first entry not yet consumed
        self._last_now_ns = 0
        self._last_path_xyz = None

        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    # ------------------------------------------------------------------
    # TUM file loading
    # ------------------------------------------------------------------

    @staticmethod
    def _load_tum(path: str):
        stamps_ns = []
        poses = []

        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                values = line.split()
                if len(values) != 8:
                    continue

                t, x, y, z, qx, qy, qz, qw = (float(v) for v in values)
                stamps_ns.append(int(t * 1e9))
                poses.append((x, y, z, qx, qy, qz, qw))

        return stamps_ns, poses

    # ------------------------------------------------------------------
    # Periodic publishing
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_from_ns(timestamp_ns: int) -> Time:
        t = Time()
        t.sec = int(timestamp_ns // 1_000_000_000)
        t.nanosec = int(timestamp_ns % 1_000_000_000)
        return t

    def _make_pose(self, index: int) -> PoseStamped:
        x, y, z, qx, qy, qz, qw = self.poses[index]

        msg = PoseStamped()
        msg.header.stamp = self._stamp_from_ns(self.stamps_ns[index])
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def _reset(self):
        self._idx = 0
        self._last_path_xyz = None
        self.path_msg.poses.clear()

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            # Sim time not received yet (bag not playing)
            return

        # Bag restarted / looped: replay the trajectory from the beginning
        if now_ns < self._last_now_ns - int(1e9):
            self.get_logger().info('Clock jumped backwards, resetting ground truth')
            self._reset()
        self._last_now_ns = now_ns

        new_idx = bisect_right(self.stamps_ns, now_ns)
        if new_idx == 0:
            return

        # Append the newly elapsed poses to the path, decimated by distance
        for i in range(self._idx, new_idx):
            x, y, z = self.poses[i][:3]
            if self._last_path_xyz is not None:
                lx, ly, lz = self._last_path_xyz
                if math.dist((x, y, z), (lx, ly, lz)) < self.path_min_distance:
                    continue
            self.path_msg.poses.append(self._make_pose(i))
            self._last_path_xyz = (x, y, z)
        self._idx = new_idx

        latest = self._make_pose(new_idx - 1)

        self.path_msg.header.stamp = latest.header.stamp
        self.pub_path.publish(self.path_msg)
        self.pub_pose.publish(latest)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = latest.header
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = latest.pose.position.x
            tf.transform.translation.y = latest.pose.position.y
            tf.transform.translation.z = latest.pose.position.z
            tf.transform.rotation = latest.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = IILABS3DGTPublisherNode()

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
