#!/usr/bin/env python3
from pathlib import Path
import struct
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time

from rosbags.rosbag1 import Reader, ReaderError
from rosbags.typesys import Stores, get_typestore


class HiltiPublisherNode(Node):

    LIDAR_TOPIC_IN_DEFAULT = '/rslidar_points'
    IMU_TOPIC_IN_DEFAULT = '/imu/data'
    ODOM_TOPIC_IN_DEFAULT = '/track_odometry'

    CAM_TOPICS_IN_DEFAULT = [
        '/oak_cam_front/left/image_raw',
        '/oak_cam_front/right/image_raw',
        '/oak_cam_rear/left/image_raw',
        '/oak_cam_rear/right/image_raw',
        '/oak_cam_left/left/image_raw',
        '/oak_cam_left/right/image_raw',
        '/oak_cam_right/left/image_raw',
        '/oak_cam_right/right/image_raw',
    ]

    def __init__(self):
        super().__init__('hilti_publisher_node')

        self.declare_parameter('bag_path', '')

        self.declare_parameter('lidar_topic_in', self.LIDAR_TOPIC_IN_DEFAULT)
        self.declare_parameter('imu_topic_in', self.IMU_TOPIC_IN_DEFAULT)
        self.declare_parameter('odom_topic_in', self.ODOM_TOPIC_IN_DEFAULT)

        self.declare_parameter('lidar_topic_out', '/hilti/point_cloud')
        self.declare_parameter('imu_topic_out', '/hilti/imu')
        self.declare_parameter('odom_topic_out', '/hilti/track_odometry')
        self.declare_parameter('twist_topic_out', '/hilti/twist')

        self.declare_parameter('publish_cameras', False)
        self.declare_parameter('publish_track_odometry', True)
        self.declare_parameter('publish_track_twist', True)
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('publish_clock', True)

        self.declare_parameter('lidar_frame_id', 'rslidar')
        self.declare_parameter('imu_frame_id', 'xsens_imu_link')

        # Usually the twist inside nav_msgs/Odometry is expressed in child_frame_id.
        # In your Hilti bag, child_frame_id is base_link.
        self.declare_parameter('twist_frame_id', '')

        bag_path = self.get_parameter('bag_path').value

        self.lidar_topic_in = self.get_parameter('lidar_topic_in').value
        self.imu_topic_in = self.get_parameter('imu_topic_in').value
        self.odom_topic_in = self.get_parameter('odom_topic_in').value

        lidar_topic_out = self.get_parameter('lidar_topic_out').value
        imu_topic_out = self.get_parameter('imu_topic_out').value
        odom_topic_out = self.get_parameter('odom_topic_out').value
        twist_topic_out = self.get_parameter('twist_topic_out').value

        self.publish_cameras = self.get_parameter('publish_cameras').value
        self.publish_track_odometry = self.get_parameter('publish_track_odometry').value
        self.publish_track_twist = self.get_parameter('publish_track_twist').value
        self.playback_rate = self.get_parameter('playback_rate').value
        self.publish_clock = self.get_parameter('publish_clock').value

        self.lidar_frame_id = str(self.get_parameter('lidar_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.twist_frame_id = str(self.get_parameter('twist_frame_id').value)

        self.pub_lidar = self.create_publisher(PointCloud2, lidar_topic_out, 10)
        self.pub_imu = self.create_publisher(Imu, imu_topic_out, 100)

        self.pub_odom = (
            self.create_publisher(Odometry, odom_topic_out, 100)
            if self.publish_track_odometry else None
        )

        self.pub_twist = (
            self.create_publisher(TwistStamped, twist_topic_out, 100)
            if self.publish_track_twist else None
        )

        clock_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub_clock = (
            self.create_publisher(Clock, '/clock', clock_qos)
            if self.publish_clock else None
        )

        self.pub_cams = []
        if self.publish_cameras:
            for i in range(5):
                self.pub_cams.append(
                    self.create_publisher(Image, f'/hilti/cam{i}/image_raw', 10)
                )

        if not bag_path:
            self.get_logger().error(
                'bag_path parameter not set. Pass: '
                '--ros-args -p bag_path:=/path/to/file.bag'
            )
            return

        self.get_logger().info(f'Opening bag: {bag_path}')
        self.typestore = get_typestore(Stores.ROS1_NOETIC)

        self._stop = False
        self._thread = threading.Thread(
            target=self._playback,
            args=(bag_path,),
            daemon=True,
        )
        self._thread.start()

    def destroy_node(self):
        self._stop = True
        super().destroy_node()

    # ------------------------------------------------------------------
    # Playback loop
    # ------------------------------------------------------------------

    def _playback(self, bag_path: str):
        topics_of_interest = {
            self.lidar_topic_in,
            self.imu_topic_in,
        }

        if self.publish_track_odometry or self.publish_track_twist:
            topics_of_interest.add(self.odom_topic_in)

        if self.publish_cameras:
            topics_of_interest.update(self.CAM_TOPICS_IN_DEFAULT)

        try:
            if self._bag_index_is_beyond_file_size(bag_path):
                self.get_logger().error(
                    f'Bag appears incomplete: {bag_path}. The ROS1 index points past '
                    'the current file size. Wait for the download to finish, then launch again.'
                )
                return

            with Reader(bag_path) as reader:
                connections = [
                    c for c in reader.connections
                    if c.topic in topics_of_interest
                ]

                if not connections:
                    self.get_logger().error(
                        f'None of the expected topics found in bag. '
                        f'Available: {[c.topic for c in reader.connections]}'
                    )
                    return

                self.get_logger().info(
                    f'Playing {len(connections)} topic(s): '
                    f'{[c.topic for c in connections]}'
                )

                pc2_fields: list | None = None

                start_bag_ns: int | None = None
                start_wall: float | None = None

                for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
                    if self._stop or not rclpy.ok():
                        break

                    try:
                        ros1_msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
                    except Exception as e:
                        self.get_logger().warn(
                            f'Deserialize error on {connection.topic}: {e}'
                        )
                        continue

                    if start_bag_ns is None:
                        start_bag_ns = timestamp_ns
                        start_wall = time.monotonic()

                    target = start_wall + (
                        timestamp_ns - start_bag_ns
                    ) / (1e9 * self.playback_rate)

                    remaining = target - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)

                    if self._stop or not rclpy.ok():
                        break

                    self._publish_clock(timestamp_ns)

                    if connection.topic == self.lidar_topic_in:
                        if pc2_fields is None:
                            pc2_fields = self._build_pc2_fields(ros1_msg.fields)

                        self.pub_lidar.publish(
                            self._convert_pointcloud2(
                                ros1_msg,
                                timestamp_ns,
                                pc2_fields,
                            )
                        )

                    elif connection.topic == self.imu_topic_in:
                        self.pub_imu.publish(
                            self._convert_imu(ros1_msg, timestamp_ns)
                        )

                    elif connection.topic == self.odom_topic_in:
                        odom_msg = self._convert_odometry(ros1_msg, timestamp_ns)

                        if self.pub_odom is not None:
                            self.pub_odom.publish(odom_msg)

                        if self.pub_twist is not None:
                            self.pub_twist.publish(
                                self._extract_twist_from_odometry(odom_msg)
                            )

                    elif self.publish_cameras:
                        idx = self.CAM_TOPICS_IN_DEFAULT.index(connection.topic)
                        if 0 <= idx < len(self.pub_cams):
                            self.pub_cams[idx].publish(
                                self._convert_image(ros1_msg, timestamp_ns)
                            )

        except ReaderError as e:
            self.get_logger().error(f'Could not read ROS1 bag {bag_path}: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Playback failed: {e}')
            raise

        self.get_logger().info('Bag playback complete.')

    # ------------------------------------------------------------------
    # Message converters: rosbags types -> ROS2 messages
    # ------------------------------------------------------------------

    @staticmethod
    def _ros2_stamp(timestamp_ns: int) -> Time:
        t = Time()
        t.sec = int(timestamp_ns // 1_000_000_000)
        t.nanosec = int(timestamp_ns % 1_000_000_000)
        return t

    def _publish_clock(self, timestamp_ns: int):
        if self.pub_clock is None:
            return

        msg = Clock()
        msg.clock = self._ros2_stamp(timestamp_ns)
        self.pub_clock.publish(msg)

    @staticmethod
    def _read_rosbag_header_fields(header: bytes) -> dict[str, bytes]:
        fields = {}
        pos = 0

        while pos + 4 <= len(header):
            field_len = struct.unpack_from('<I', header, pos)[0]
            pos += 4

            if pos + field_len > len(header):
                break

            field = header[pos:pos + field_len]
            pos += field_len

            key, _, value = field.partition(b'=')
            fields[key.decode(errors='replace')] = value

        return fields

    @classmethod
    def _bag_index_is_beyond_file_size(cls, bag_path: str) -> bool:
        path = Path(bag_path)

        try:
            file_size = path.stat().st_size

            with path.open('rb') as bag:
                if not bag.readline().startswith(b'#ROSBAG V2.0'):
                    return False

                raw_len = bag.read(4)
                if len(raw_len) != 4:
                    return True

                header_len = struct.unpack('<I', raw_len)[0]
                header = bag.read(header_len)
                if len(header) != header_len:
                    return True

            fields = cls._read_rosbag_header_fields(header)
            index_raw = fields.get('index_pos')

            if not index_raw or len(index_raw) != 8:
                return False

            index_pos = struct.unpack('<Q', index_raw)[0]
            return index_pos >= file_size

        except OSError:
            return False

    def _build_pc2_fields(self, src_fields) -> list:
        out = []

        for f in src_fields:
            ff = PointField()
            ff.name = str(f.name)
            ff.offset = int(f.offset)
            ff.datatype = int(f.datatype)
            ff.count = int(f.count)
            out.append(ff)

        return out

    def _convert_pointcloud2(
        self,
        src,
        timestamp_ns: int,
        cached_fields: list,
    ) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = self._ros2_stamp(timestamp_ns)
        msg.header.frame_id = self.lidar_frame_id or str(src.header.frame_id)

        msg.height = int(src.height)
        msg.width = int(src.width)
        msg.is_bigendian = bool(src.is_bigendian)
        msg.point_step = int(src.point_step)
        msg.row_step = int(src.row_step)
        msg.is_dense = bool(src.is_dense)
        msg.data = src.data.tobytes()
        msg.fields = cached_fields

        return msg

    def _convert_imu(self, src, timestamp_ns: int) -> Imu:
        msg = Imu()
        msg.header.stamp = self._ros2_stamp(timestamp_ns)
        msg.header.frame_id = self.imu_frame_id or str(src.header.frame_id)

        msg.orientation.x = float(src.orientation.x)
        msg.orientation.y = float(src.orientation.y)
        msg.orientation.z = float(src.orientation.z)
        msg.orientation.w = float(src.orientation.w)

        msg.angular_velocity.x = float(src.angular_velocity.x)
        msg.angular_velocity.y = float(src.angular_velocity.y)
        msg.angular_velocity.z = float(src.angular_velocity.z)

        msg.linear_acceleration.x = float(src.linear_acceleration.x)
        msg.linear_acceleration.y = float(src.linear_acceleration.y)
        msg.linear_acceleration.z = float(src.linear_acceleration.z)

        msg.orientation_covariance = [
            float(v) for v in src.orientation_covariance
        ]
        msg.angular_velocity_covariance = [
            float(v) for v in src.angular_velocity_covariance
        ]
        msg.linear_acceleration_covariance = [
            float(v) for v in src.linear_acceleration_covariance
        ]

        return msg

    def _convert_odometry(self, src, timestamp_ns: int) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self._ros2_stamp(timestamp_ns)
        msg.header.frame_id = str(src.header.frame_id)
        msg.child_frame_id = str(src.child_frame_id)

        msg.pose.pose.position.x = float(src.pose.pose.position.x)
        msg.pose.pose.position.y = float(src.pose.pose.position.y)
        msg.pose.pose.position.z = float(src.pose.pose.position.z)

        msg.pose.pose.orientation.x = float(src.pose.pose.orientation.x)
        msg.pose.pose.orientation.y = float(src.pose.pose.orientation.y)
        msg.pose.pose.orientation.z = float(src.pose.pose.orientation.z)
        msg.pose.pose.orientation.w = float(src.pose.pose.orientation.w)

        msg.twist.twist.linear.x = float(src.twist.twist.linear.x)
        msg.twist.twist.linear.y = float(src.twist.twist.linear.y)
        msg.twist.twist.linear.z = float(src.twist.twist.linear.z)

        msg.twist.twist.angular.x = float(src.twist.twist.angular.x)
        msg.twist.twist.angular.y = float(src.twist.twist.angular.y)
        msg.twist.twist.angular.z = float(src.twist.twist.angular.z)

        msg.pose.covariance = [float(v) for v in src.pose.covariance]
        msg.twist.covariance = [float(v) for v in src.twist.covariance]

        return msg

    def _extract_twist_from_odometry(self, odom: Odometry) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = odom.header.stamp

        # In nav_msgs/Odometry, twist is normally expressed in child_frame_id.
        # In your bag, child_frame_id == base_link.
        if self.twist_frame_id:
            msg.header.frame_id = self.twist_frame_id
        else:
            msg.header.frame_id = odom.child_frame_id or 'base_link'

        msg.twist.linear.x = odom.twist.twist.linear.x
        msg.twist.linear.y = odom.twist.twist.linear.y
        msg.twist.linear.z = odom.twist.twist.linear.z

        msg.twist.angular.x = odom.twist.twist.angular.x
        msg.twist.angular.y = odom.twist.twist.angular.y
        msg.twist.angular.z = odom.twist.twist.angular.z

        return msg

    def _convert_image(self, src, timestamp_ns: int) -> Image:
        msg = Image()
        msg.header.stamp = self._ros2_stamp(timestamp_ns)
        msg.header.frame_id = str(src.header.frame_id)

        msg.height = int(src.height)
        msg.width = int(src.width)
        msg.encoding = str(src.encoding)
        msg.is_bigendian = bool(src.is_bigendian)
        msg.step = int(src.step)
        msg.data = bytes(src.data)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = HiltiPublisherNode()

    try:
        if hasattr(node, '_thread'):
            node._thread.join()
    except KeyboardInterrupt:
        node._stop = True
        if hasattr(node, '_thread'):
            node._thread.join(timeout=1.0)
    finally:
        node._stop = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()