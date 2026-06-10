import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros2_hilti_publishers')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/neo/workspace/site2_robot_2.bag',
        description='Absolute path to the Hilti ROS1 bag file',
    )

    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = real-time)',
    )

    publish_cameras_arg = DeclareLaunchArgument(
        'publish_cameras',
        default_value='false',
        description='Also publish Alphasense camera images',
    )

    publisher_node = Node(
        package='ros2_hilti_publishers',
        executable='hilti_publisher',
        name='hilti_publisher_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'hilti_publisher.yaml'),
            {
                'bag_path': LaunchConfiguration('bag_path'),
                'playback_rate': LaunchConfiguration('playback_rate'),
                'publish_cameras': LaunchConfiguration('publish_cameras'),
            },
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        playback_rate_arg,
        publish_cameras_arg,
        publisher_node,
    ])
