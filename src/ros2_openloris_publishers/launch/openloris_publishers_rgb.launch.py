"""OpenLORIS sensor adapters for the monocular-depth pipeline.

Same as openloris_publishers.launch.py, minus the depth-camera
`depth_pointcloud` node: monocular_depth publishes `/openloris/point_cloud`
from the RGB stream instead, so the depth-camera producer must stay off to
avoid two publishers racing on the same topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'ros2_openloris_publishers')
    config = os.path.join(
        package_share, 'config', 'openloris_publishers.yaml')

    bag_path = LaunchConfiguration('bag_path')
    play_bag = LaunchConfiguration('play_bag')

    nodes = [
        Node(
            package='ros2_openloris_publishers',
            executable='clock_relay',
            name='openloris_clock_relay',
            parameters=[{
                'input_topic': '/clock_raw',
                'output_topic': '/clock',
            }],
            output='screen',
        ),
        Node(
            package='ros2_openloris_publishers',
            executable='imu_merger',
            name='openloris_imu_merger',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='ros2_openloris_publishers',
            executable='odometry_tf',
            name='openloris_odometry_tf',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='ros2_openloris_publishers',
            executable='ground_truth',
            name='openloris_ground_truth',
            parameters=[config],
            output='screen',
        ),
        # rosbags-convert cannot preserve ROS 1 latched /tf_static QoS.
        # Publish the two transforms needed by the RGB-D and IMU pipelines
        # with a native ROS 2 transient-local static broadcaster instead.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_d400_color',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0.2264836849091656',
                '--y', '-0.05114194035652147',
                '--z', '0.916',
                '--qx', '-0.49676229968284147',
                '--qy', '0.4998795887129772',
                '--qz', '-0.49510681269354095',
                '--qw', '0.5081504289345848',
                '--frame-id', 'base_link',
                '--child-frame-id', 'd400_color',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_d400_imu',
            parameters=[{'use_sim_time': True}],
            arguments=[
                # Inverse of the calibrated base_link -> d400_imu pose.
                # dua_robot_localization currently reverses source and target
                # for IMU vector transforms, so publishing the inverse here
                # makes its resulting IMU -> base_link rotation correct.
                '--x', '-0.061159013133332246',
                '--y', '0.91930388167961319',
                '--z', '-0.22445689655448128',
                '--qx', '0.49538006860636924',
                '--qy', '-0.4995473514230948',
                '--qz', '0.49840674562280735',
                '--qw', '0.5065982108450464',
                '--frame-id', 'base_link',
                '--child-frame-id', 'd400_imu',
            ],
            output='screen',
        ),
    ]

    # The converted bag has one CameraInfo message. The delay gives DDS
    # discovery time before that singleton is published.
    bag_player = ExecuteProcess(
        condition=IfCondition(play_bag),
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '100.0',
            '--delay', '2.0',
            '--topics',
            '/d400/color/camera_info',
            '/d400/color/image_raw',
            '/d400/accel/sample',
            '/d400/gyro/sample',
            '/odom',
            '/gt',
            '--remap', '/clock:=/clock_raw',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Converted OpenLORIS ROS 2 bag directory',
        ),
        DeclareLaunchArgument(
            'play_bag',
            default_value='false',
            description='Play bag_path and publish /clock',
        ),
        *nodes,
        bag_player,
    ])
