"""
LIO-SAM evaluation on the OpenLORIS-Scene D400 RGB-D sequences.

ros2_openloris_publishers normalizes the bag topics:
  /openloris/point_cloud  XYZI PointCloud2 from aligned depth, frame d400_color
  /openloris/imu          merged D400 accel + gyro, frame d400_imu
  /openloris/odom         normalized wheel odometry (identity origin)

TF tree:
  map [static identity] -> base_odom -> base_link -> d400_color -> d400_imu

Launch arguments:
  bag_path          Converted OpenLORIS ROS 2 bag directory
  play_bag          Start ros2 bag play (default: true)
  rviz              Start RViz2 (default: true)
  record_trajectory Record SLAM output in TUM format (default: true)
  output_dir        Directory for TUM trajectory files

June 2026
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('slam_evaluator')
    publishers_share = get_package_share_directory(
        'ros2_openloris_publishers')
    config = os.path.join(
        slam_share, 'config', 'lio_sam_openloris.yaml')
    rviz_config = os.path.join(
        slam_share, 'rviz', 'lio_sam_openloris.rviz')

    namespace = LaunchConfiguration('namespace')
    bag_path = LaunchConfiguration('bag_path')
    play_bag = LaunchConfiguration('play_bag')
    use_rviz = LaunchConfiguration('rviz')
    record_trajectory = LaunchConfiguration('record_trajectory')
    output_dir = LaunchConfiguration('output_dir')

    publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                publishers_share,
                'launch',
                'openloris_publishers.launch.py',
            )
        ),
        launch_arguments={
            'bag_path': bag_path,
            'play_bag': play_bag,
        }.items(),
    )

    # The normalized wheel odometry supplies base_odom -> base_link.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base_odom',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'base_odom',
        ],
    )

    tf_server = Node(
        namespace=namespace,
        package='dua_tf_server',
        executable='dua_tf_server_app',
        name='dua_tf_server',
        parameters=[config],
        output='screen',
    )

    lio_sam = Node(
        namespace=namespace,
        package='lio_sam',
        executable='lio_sam_app',
        name='lio_sam',
        emulate_tty=True,
        shell=True,
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
            ('/point_cloud', '/openloris/point_cloud'),
            ('/odometry', '/ekf_global/odometry'),
        ],
        output='both',
    )

    ekf = Node(
        namespace=namespace,
        package='dua_robot_localization',
        executable='dua_robot_localization_app',
        name='ekf_global',
        emulate_tty=True,
        shell=True,
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
        ],
        output='both',
    )

    recorder = Node(
        condition=IfCondition(record_trajectory),
        package='ros2_openloris_publishers',
        executable='tum_recorder',
        name='openloris_tum_recorder',
        parameters=[{
            'pose_topic': '/lio_sam/map_optimization/pose',
            'pose_msg_type': 'pose_with_covariance',
            'output_dir': output_dir,
            'slam_name': 'lio_sam_openloris',
            'append_datetime': True,
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/neo/workspace/office1-1',
            description='Converted OpenLORIS ROS 2 bag directory',
        ),
        DeclareLaunchArgument(
            'play_bag',
            default_value='true',
            description='Start ros2 bag play with the required topics',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'record_trajectory',
            default_value='true',
            description='Record the LIO-SAM trajectory in TUM format',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/neo/workspace/logs/openloris',
            description='Directory for recorded TUM trajectories',
        ),
        publishers,
        static_map_to_odom,
        tf_server,
        lio_sam,
        ekf,
        recorder,
        rviz,
    ])
