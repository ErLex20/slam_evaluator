"""Run GLIM on the IILABS3D Livox Mid-360 benchmark."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _glim_node(context):
    evaluator_share = get_package_share_directory('slam_evaluator')
    sensor = LaunchConfiguration('sensor').perform(context)
    config_path = os.path.join(
        evaluator_share, 'config', f'glim_iilabs3d_{sensor}')

    if not os.path.isfile(os.path.join(config_path, 'config.json')):
        raise RuntimeError(
            f'No GLIM IILABS3D configuration for sensor "{sensor}" at '
            f'{config_path}'
        )

    return [Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'use_sim_time': True,
            'timing.enable': ParameterValue(
                LaunchConfiguration('timing'), value_type=bool),
            'timing.csv_path': LaunchConfiguration('timing_csv'),
        }],
    )]


def generate_launch_description():
    evaluator_share = get_package_share_directory('slam_evaluator')
    dataset_dir = LaunchConfiguration('dataset_dir')
    sensor = LaunchConfiguration('sensor')
    sequence = LaunchConfiguration('sequence')
    sequence_dir = PathJoinSubstitution([
        dataset_dir, 'benchmark', sensor, sequence])

    rviz_config = os.path.join(
        evaluator_share, 'rviz', 'glim_iilabs3d.rviz')
    timing_filename = (
        f'glim_{datetime.now().strftime("%Y%m%d_%H%M%S")}_timing.csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_dir',
            default_value='/home/neo/workspace/logs/iilabs/iilabs3d_dataset',
            description='Root of the IILABS3D dataset'),
        DeclareLaunchArgument(
            'sensor',
            default_value='livox_mid-360',
            description='LiDAR sensor (currently configured: livox_mid-360)'),
        DeclareLaunchArgument(
            'sequence',
            default_value='nav_a_diff',
            description='Sequence being played'),
        DeclareLaunchArgument(
            'gt_file',
            default_value=PathJoinSubstitution([
                sequence_dir, 'ground_truth.tum']),
            description='Absolute path to the TUM ground-truth file'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2'),
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
                'pose_topic': '/glim_ros/pose',
                'pose_msg_type': 'pose',
                'output_dir': PathJoinSubstitution([
                    sequence_dir, 'results']),
                'slam_name': 'glim',
                'append_datetime': True,
                'use_sim_time': True,
            }],
        ),
        OpaqueFunction(function=_glim_node),
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
