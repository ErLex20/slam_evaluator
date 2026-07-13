"""Faster-LIO launch file for all IILABS3D benchmark LiDARs.

Supported sensor values are livox_mid-360, ouster_os1-64,
robosense_rs-helios-5515, and velodyne_vlp-16.

The evaluator config supplies the dataset-specific LiDAR-to-IMU calibration
through Faster-LIO's original mapping.extrinsic_T parameter.

Faster-LIO publishes its raw IMU/body odometry. This launch remaps the
upstream absolute output topics into the /faster_lio namespace and records
/faster_lio/odometry for later TUM evaluation.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _faster_lio_node(context):
    pkg_share = get_package_share_directory('slam_evaluator')
    sensor = LaunchConfiguration('sensor').perform(context)
    config = os.path.join(pkg_share, 'config', f'faster_lio_iilabs3d_{sensor}.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(
            f'No Faster-LIO IILABS3D configuration for sensor "{sensor}": {config}')

    return [Node(
        namespace=LaunchConfiguration('namespace').perform(context),
        package='faster_lio',
        executable='run_mapping_online',
        name='faster_lio',
        emulate_tty=True,
        output='both',
        parameters=[config, {
            'timing.enable': ParameterValue(
                LaunchConfiguration('timing'), value_type=bool),
            'timing.csv_path': LaunchConfiguration('timing_csv'),
        }],
        remappings=[
            ('/Odometry', '/faster_lio/odometry'),
            ('/path', '/faster_lio/path'),
            ('/cloud_registered', '/faster_lio/cloud_registered'),
            ('/cloud_registered_body', '/faster_lio/cloud_registered_body'),
            ('/cloud_registered_effect_world', '/faster_lio/cloud_registered_effect_world'),
        ],
    )]


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('slam_evaluator')

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'faster_lio_iilabs3d.rviz'
    )

    ld.add_action(DeclareLaunchArgument(
        'namespace',
        default_value=''
    ))

    ld.add_action(DeclareLaunchArgument(
        'dataset_dir',
        default_value='/home/neo/workspace/logs/iilabs/iilabs3d_dataset',
        description='Root of the IILABS3D dataset',
    ))

    ld.add_action(DeclareLaunchArgument(
        'sensor',
        default_value='livox_mid-360',
        description=(
            'LiDAR sensor: livox_mid-360, ouster_os1-64, '
            'robosense_rs-helios-5515, or velodyne_vlp-16'),
    ))

    ld.add_action(DeclareLaunchArgument(
        'sequence',
        default_value='nav_a_diff',
        description='Sequence being played',
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2',
    ))

    sequence_dir = PathJoinSubstitution([
        LaunchConfiguration('dataset_dir'),
        'benchmark',
        LaunchConfiguration('sensor'),
        LaunchConfiguration('sequence'),
    ])

    timing_filename = (
        f'faster_lio_{datetime.now().strftime("%Y%m%d_%H%M%S")}_timing.csv')
    ld.add_action(DeclareLaunchArgument(
        'timing',
        default_value='true',
        description='Write normalized preprocess/mapping timing CSV',
    ))
    ld.add_action(DeclareLaunchArgument(
        'timing_csv',
        default_value=PathJoinSubstitution([
            sequence_dir, 'results', timing_filename]),
        description='Output timing CSV path',
    ))

    ld.add_action(DeclareLaunchArgument(
        'gt_file',
        default_value=PathJoinSubstitution([sequence_dir, 'ground_truth.tum']),
        description='Absolute path to the TUM ground truth file',
    ))

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

    tum_recorder = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_tum_recorder',
        name='iilabs3d_tum_recorder',
        output='screen',
        parameters=[{
            'pose_topic': '/faster_lio/odometry',
            'pose_msg_type': 'odometry',
            'output_dir': PathJoinSubstitution([sequence_dir, 'results']),
            'slam_name': 'faster_lio',
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

    static_tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_camera_init',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'camera_init',
        ],
    )
    ld.add_action(static_tf_map_to_camera_init)

    ld.add_action(OpaqueFunction(function=_faster_lio_node))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    ld.add_action(rviz)

    return ld
