"""Launch the GrandTour mission replay node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ros2_grandtour_publishers')
    config = os.path.join(
        package_share, 'config', 'grandtour_publishers.yaml')

    mission_dir = LaunchConfiguration('mission_dir')
    playback_rate = LaunchConfiguration('playback_rate')
    loop = LaunchConfiguration('loop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_dir',
            default_value=(
                '/home/neo/workspace/logs/grandtour/SPX-2'),
            description=(
                'Extracted GrandTour mission directory (contains data/ and '
                'metadata/).'),
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Playback speed multiplier.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Repeat the mission when the timeline is exhausted.',
        ),
        Node(
            package='ros2_grandtour_publishers',
            executable='grandtour_publisher',
            name='grandtour_publisher',
            parameters=[
                config,
                {
                    'mission_dir': mission_dir,
                    'playback_rate': playback_rate,
                    'loop': loop,
                },
            ],
            output='screen',
        ),
    ])
