import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros2_iilabs3d_publishers')

    gt_file_arg = DeclareLaunchArgument(
        'gt_file',
        default_value='/home/neo/workspace/logs/iilabs/iilabs3d_dataset/'
                      'benchmark/livox_mid-360/nav_a_diff/ground_truth.tum',
        description='Absolute path to the TUM ground truth file',
    )

    publisher_node = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_gt_publisher',
        name='iilabs3d_gt_publisher',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'iilabs3d_gt_publisher.yaml'),
            {
                'gt_file': LaunchConfiguration('gt_file'),
            },
        ],
    )

    return LaunchDescription([
        gt_file_arg,
        publisher_node,
    ])
