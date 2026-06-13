"""
LIO-SAM launch file for the IILABS3D dataset.

The 'sensor' argument selects the sequence directory and the point cloud
topic; all sensors share config/lio_sam_iilabs3d.yaml (see its header for the
Ouster-specific parameters):
  - livox_mid_360            -> sensor:=livox_mid-360
  - ouster_os1_64            -> sensor:=ouster_os1-64
  - robosense_rs_helios_5515 -> sensor:=robosense_rs-helios-5515
  - velodyne_vlp_16          -> sensor:=velodyne_vlp-16

All sensor extrinsics come from the bag's /tf and /tf_static
(eve/odom -> eve/base_footprint -> eve/base_link -> LiDAR frame, eve/imu_link);
only a static identity map -> eve/odom is added to complete the tree.

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

    config = os.path.join(
        pkg_share,
        'config',
        'lio_sam_iilabs3d.yaml'
    )

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
        'lio_sam_iilabs3d.rviz'
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

    # SLAM pose recorder: writes <sequence_dir>/results/lio_sam_<datetime>.tum
    # for offline evaluation with `iilabs3d eval`.
    tum_recorder = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_tum_recorder',
        name='iilabs3d_tum_recorder',
        output='screen',
        parameters=[{
            'pose_topic': '/lio_sam/map_optimization/pose',
            'pose_msg_type': 'pose_with_covariance',
            'output_dir': PathJoinSubstitution([sequence_dir, 'results']),
            'slam_name': 'lio_sam',
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

    dua_tf_server = Node(
        namespace=ns,
        package='dua_tf_server',
        executable='dua_tf_server_app',
        name='dua_tf_server',
        parameters=[config],
    )
    ld.add_action(dua_tf_server)

    lio_sam = Node(
        namespace=ns,
        package='lio_sam',
        executable='lio_sam_app',
        name='lio_sam',
        emulate_tty=True,
        shell=True,
        output='both',
        # prefix='gdbserver localhost:8081',
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
            ('/point_cloud', lidar_topic),
            ('/odometry', '/ekf_global/odometry'),
        ]
    )
    ld.add_action(lio_sam)

    ekf_global = Node(
        namespace=ns,
        package='dua_robot_localization',
        executable='dua_robot_localization_app',
        name='ekf_global',
        emulate_tty=True,
        shell=True,
        output='both',
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
        ]
    )
    ld.add_action(ekf_global)

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
