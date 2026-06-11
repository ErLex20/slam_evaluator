"""
SPARK Fast-LIO launch file for the IILABS3D dataset.

The 'sensor' argument selects the sequence directory, the parameter file
(config/spark_fast_lio_iilabs3d_<sensor>.yaml) and the point cloud topic:
  - livox_mid_360            -> sensor:=livox_mid-360
  - ouster_os1_64            -> sensor:=ouster_os1-64
  - robosense_rs_helios_5515 -> sensor:=robosense_rs-helios-5515
  - velodyne_vlp_16          -> sensor:=velodyne_vlp-16

The base -> LiDAR extrinsics come from the bag's /tf_static
(eve/base_link -> LiDAR frame); the LiDAR -> IMU extrinsics are set in the
config file. A static identity map -> eve/odom completes the tree.

NOTE: spark_fast_lio unconditionally broadcasts map -> eve/base_link on /tf,
while the bag's wheel odometry chain (eve/odom -> eve/base_footprint ->
eve/base_link) also claims eve/base_link: its TF parent flips between the
two. Evaluation is unaffected since the recorder uses the odometry topic.

June 2026
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('slam_evaluator')

    config = PathJoinSubstitution([
        pkg_share,
        'config',
        ['spark_fast_lio_iilabs3d_', LaunchConfiguration('sensor'), '.yaml'],
    ])

    # The Ouster sequences keep the driver's original topic name; all the
    # other sensors are recorded as /eve/lidar3d.
    lidar_topic = PythonExpression([
        "'/eve/ouster/points' if '",
        LaunchConfiguration('sensor'),
        "' == 'ouster_os1-64' else '/eve/lidar3d'",
    ])

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'spark_fast_lio_iilabs3d.rviz'
    )

    ns = LaunchConfiguration('namespace')

    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    ld.add_action(ns_launch_arg)

    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value='/home/neo/workspace/logs/iilabs/iilabs3d_dataset',
        description='Root of the IILABS3D dataset',
    )
    ld.add_action(dataset_dir_arg)

    sensor_arg = DeclareLaunchArgument(
        'sensor',
        default_value='velodyne_vlp-16',
        description='LiDAR sensor of the sequence being played',
    )
    ld.add_action(sensor_arg)

    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='nav_a_diff',
        description='Sequence being played',
    )
    ld.add_action(sequence_arg)

    sequence_dir = PathJoinSubstitution([
        LaunchConfiguration('dataset_dir'),
        'benchmark',
        LaunchConfiguration('sensor'),
        LaunchConfiguration('sequence'),
    ])

    gt_file_arg = DeclareLaunchArgument(
        'gt_file',
        default_value=PathJoinSubstitution([sequence_dir, 'ground_truth.tum']),
        description='Absolute path to the TUM ground truth file',
    )
    ld.add_action(gt_file_arg)

    # Ground truth replay (eve/base_link trajectory in map frame), driven by
    # /clock from the bag for online comparison with the SLAM estimate.
    gt_publisher = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_gt_publisher',
        name='iilabs3d_gt_publisher',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('ros2_iilabs3d_publishers'),
                'config',
                'iilabs3d_gt_publisher.yaml'
            ),
            {
                'gt_file': LaunchConfiguration('gt_file'),
            },
        ],
    )
    ld.add_action(gt_publisher)

    # SLAM pose recorder: writes <sequence_dir>/results/spark_fast_lio_<datetime>.tum
    # for offline evaluation with `iilabs3d eval`.
    tum_recorder = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_tum_recorder',
        name='iilabs3d_tum_recorder',
        output='screen',
        parameters=[{
            'pose_topic': '/spark_fast_lio/odometry',
            'pose_msg_type': 'odometry',
            'output_dir': PathJoinSubstitution([sequence_dir, 'results']),
            'slam_name': 'spark_fast_lio',
            'append_datetime': True,
            'use_sim_time': True,
        }],
    )
    ld.add_action(tum_recorder)

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'eve/odom',
        ],
    )
    ld.add_action(static_tf_map_to_odom)

    spark_fast_lio = Node(
        namespace=ns,
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='spark_fast_lio',
        emulate_tty=True,
        shell=True,
        output='both',
        parameters=[config],
        remappings=[
            ('lidar', lidar_topic),
            ('imu', '/eve/imu/data'),
            ('odometry', '/spark_fast_lio/odometry'),
            ('path', '/spark_fast_lio/path'),
            ('cloud_registered', '/spark_fast_lio/cloud_registered'),
            ('cloud_registered_lidar', '/spark_fast_lio/cloud_registered_lidar'),
            ('cloud_registered_body', '/spark_fast_lio/cloud_registered_body'),
            ('cloud_registered_base', '/spark_fast_lio/cloud_registered_base'),
        ]
    )
    ld.add_action(spark_fast_lio)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # ld.add_action(rviz)

    return ld
