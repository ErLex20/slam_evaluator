"""
LIO-SAM launch file for the Hilti SLAM Challenge dataset.

June 2026
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('slam_evaluator')

    config = os.path.join(
        pkg_share,
        'config',
        'lio_sam_hilti.yaml'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'lio_sam_hilti.rviz'
    )

    robot_description_path = os.path.join(
        pkg_share,
        'urdf',
        'hilti.urdf.xacro'
    )

    # If the file is a xacro, process it.
    # This also works if you switch the extension to .urdf.xacro above.
    if robot_description_path.endswith('.xacro'):
        robot_description = xacro.process_file(robot_description_path).toxml()
    else:
        with open(robot_description_path, 'r') as f:
            robot_description = f.read()

    ns = LaunchConfiguration('namespace')

    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    ld.add_action(ns_launch_arg)

    robot_state_publisher = Node(
        namespace=ns,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_description,
            }
        ],
    )
    ld.add_action(robot_state_publisher)

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'odom',
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
            ('/point_cloud', '/hilti/point_cloud'),
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