"""Run standalone VoxelMap on any IILABS3D benchmark LiDAR."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _voxel_map_node(context):
    evaluator_share = get_package_share_directory('slam_evaluator')
    sensor = LaunchConfiguration('sensor').perform(context)
    config = os.path.join(
        evaluator_share, 'config', f'voxel_map_iilabs3d_{sensor}.yaml')
    if not os.path.isfile(config):
        raise RuntimeError(
            f'No VoxelMap IILABS3D configuration for sensor "{sensor}": {config}')

    return [Node(
        package='voxel_map',
        executable='voxel_mapping_odom',
        name='voxel_map',
        output='screen',
        parameters=[config, {
            'timing.enable': ParameterValue(
                LaunchConfiguration('timing'), value_type=bool),
            'timing.csv_path': LaunchConfiguration('timing_csv'),
        }],
        remappings=[
            ('/aft_mapped_to_init', '/voxel_map/odometry'),
            ('/path', '/voxel_map/path'),
            ('/cloud_registered', '/voxel_map/cloud_registered'),
            ('/cloud_effected', '/voxel_map/cloud_effected'),
            ('/planes', '/voxel_map/planes'),
        ],
    )]


def generate_launch_description():
    evaluator_share = get_package_share_directory('slam_evaluator')
    dataset_dir = LaunchConfiguration('dataset_dir')
    sensor = LaunchConfiguration('sensor')
    sequence = LaunchConfiguration('sequence')
    sequence_dir = PathJoinSubstitution([
        dataset_dir, 'benchmark', sensor, sequence])

    rviz_config = os.path.join(
        evaluator_share, 'rviz', 'voxel_map_iilabs3d.rviz')
    timing_filename = (
        f'voxel_map_{datetime.now().strftime("%Y%m%d_%H%M%S")}_timing.csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_dir',
            default_value='/home/neo/workspace/logs/iilabs/iilabs3d_dataset'),
        DeclareLaunchArgument(
            'sensor',
            default_value='livox_mid-360',
            description=(
                'LiDAR sensor: livox_mid-360, ouster_os1-64, '
                'robosense_rs-helios-5515, or velodyne_vlp-16')),
        DeclareLaunchArgument('sequence', default_value='nav_a_diff'),
        DeclareLaunchArgument(
            'gt_file',
            default_value=PathJoinSubstitution([sequence_dir, 'ground_truth.tum'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'timing',
            default_value='true',
            description='Write normalized preprocess/mapping timing CSV'),
        DeclareLaunchArgument(
            'timing_csv',
            default_value=PathJoinSubstitution([
                sequence_dir, 'results', timing_filename]),
            description='Output timing CSV path'),

        Node(
            package='ros2_iilabs3d_publishers',
            executable='iilabs3d_gt_publisher',
            name='iilabs3d_gt_publisher',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('ros2_iilabs3d_publishers'),
                    'config', 'iilabs3d_gt_publisher.yaml'),
                {'gt_file': LaunchConfiguration('gt_file')},
            ],
        ),
        Node(
            package='ros2_iilabs3d_publishers',
            executable='iilabs3d_tum_recorder',
            name='iilabs3d_tum_recorder',
            output='screen',
            parameters=[{
                'pose_topic': '/voxel_map/odometry',
                'pose_msg_type': 'odometry',
                'output_dir': PathJoinSubstitution([sequence_dir, 'results']),
                'slam_name': 'voxel_map',
                'append_datetime': True,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_camera_init',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                'map', 'camera_init'],
        ),
        OpaqueFunction(function=_voxel_map_node),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
