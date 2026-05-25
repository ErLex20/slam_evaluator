"""
LIO-SAM launch file.

June 26, 2025
"""

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('slam_evaluator'),
        'config',
        'lio_sam_kitti.yaml'
    )
    rviz_config = os.path.join(
        get_package_share_directory('slam_evaluator'),
        'rviz',
        'lio_sam_kitti.rviz'
    )

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='')
    ld.add_action(ns_launch_arg)

    # Create dua_tf_server node
    dua_tf_server = Node(
        namespace=ns,
        package='dua_tf_server',
        executable='dua_tf_server_app',
        name='dua_tf_server',
        parameters=[config],
    )
    ld.add_action(dua_tf_server)

    # Create LIO-SAM node
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
            ('/get_transform',               '/dua_tf_server/get_transform'),
            ('/point_cloud',                 '/kitti/point_cloud'),
            ('/odometry',                    '/ekf_global/odometry'),
        ]
    )
    ld.add_action(lio_sam)

    # EKF Global
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
            ("/get_transform",                  "/dua_tf_server/get_transform"),
        ]
    )
    ld.add_action(ekf_global)

    # Static transform publisher: base_link -> imu_link
    static_tf_imu_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_to_base',
        arguments=[
            '0.8086759', '-0.3195559', '0.7997231',
            '0.00741353', '-0.00101574', '0.00037587', '0.99997186',
            'base_link', 'imu_link'
        ]
    )
    ld.add_action(static_tf_imu_to_base)

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    # ld.add_action(rviz)

    return ld
