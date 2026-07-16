# Copyright 2026 dotX Automation s.r.l.
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

"""
LIO-SAM launch file for merged GrandTour ROS2 bags (Hesai XT-32).

The bag is expected at <mission_dir>/ros2 by default and is played with a
simulation clock. Only the standard-message inputs used by this evaluator are
replayed. In particular, the converted bag's raw /tf is intentionally omitted:
it contains inverse dataset transforms such as base -> odom. The
grandtour_bag_adapter instead normalizes /anymal/state_estimator/odometry,
publishes the required odom -> base transform, and creates a one-shot
odom -> enu_origin alignment for ground-truth visualization. The recorded
/tf_static supplies the sensor extrinsics.

By default LIO-SAM consumes /ekf_global/odometry. Set motion_prior:=raw_odom
to bypass the EKF and consume the adapted ANYmal odometry directly.

record_tum:=true (default) reuses ros2_iilabs3d_publishers' generic TUM
recorder to write <mission_dir>/results/lio_sam_<datetime>.tum from
/lio_sam/map_optimization/pose.

Select a supported mission and its matching parameter file with, e.g.:

  ros2 launch slam_evaluator lio_sam_grandtour.launch.py sequence:=ETH-1

July 2026
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('slam_evaluator')
    sequence = LaunchConfiguration('sequence')
    default_config = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression([
            "'lio_sam_grandtour_' + '", sequence,
            "'.lower().replace('-', '') + '.yaml'",
        ]),
    ])
    default_mission_dir = PythonExpression([
        "{'SPX-2': '/home/neo/workspace/logs/grandtour/SPX-2', "
        "'ETH-1': '/home/neo/workspace/logs/grandtour/ETH-1', "
        "'SNOW-2': '/home/neo/workspace/logs/grandtour/SNOW-2', "
        "'EIG-1': '/home/neo/workspace/logs/grandtour/EIG-1', "
        "'ARC-2': '/home/neo/workspace/logs/grandtour/ARC-2', "
        "'ARC-3': '/home/neo/workspace/logs/grandtour/ARC-3', "
        "'CON-4': '/home/neo/workspace/logs/grandtour/CON-4', "
        "'ARC-7': '/home/neo/workspace/logs/grandtour/ARC-7', "
        "'HEAP-1': '/home/neo/workspace/logs/grandtour/HEAP-1'}"
        "['", sequence, "']",
    ])
    config = LaunchConfiguration('config_file')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lio_sam_grandtour.rviz')

    bag_qos_config = os.path.join(
        pkg_share, 'config', 'grandtour_rosbag_qos.yaml')

    ns = LaunchConfiguration('namespace')
    motion_prior = LaunchConfiguration('motion_prior')
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))

    ld.add_action(DeclareLaunchArgument(
        'sequence',
        default_value='SPX-2',
        choices=[
            'SPX-2', 'ETH-1', 'SNOW-2', 'EIG-1', 'ARC-2', 'ARC-3', 'CON-4',
            'ARC-7', 'HEAP-1',
        ],
        description='GrandTour sequence; selects mission_dir and config_file',
    ))

    ld.add_action(DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='LIO-SAM parameter file (derived from sequence by default)',
    ))

    ld.add_action(DeclareLaunchArgument(
        'mission_dir',
        default_value=default_mission_dir,
        description=(
            'GrandTour mission directory '
            '(derived from sequence by default)'),
    ))

    ld.add_action(DeclareLaunchArgument(
        'bag_path',
        default_value=PathJoinSubstitution([
            LaunchConfiguration('mission_dir'), 'ros2']),
        description='Merged ROS2 bag directory (default: <mission_dir>/ros2)',
    ))

    ld.add_action(DeclareLaunchArgument(
        'lidar_topic',
        default_value='/boxi/hesai/points_undistorted',
        description='PointCloud2 topic from the merged bag',
    ))

    ld.add_action(DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Rosbag playback speed multiplier',
    ))

    ld.add_action(DeclareLaunchArgument(
        'motion_prior',
        default_value='ekf',
        choices=['ekf', 'raw_odom'],
        description=(
            'External motion prior for LIO-SAM: EKF fusion or the raw '
            'ANYmal odometry stream'),
    ))

    ld.add_action(DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Repeat the mission when the timeline is exhausted',
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2',
    ))

    ld.add_action(DeclareLaunchArgument(
        'record_tum',
        default_value='true',
        description=(
            'Record /lio_sam/map_optimization/pose to a TUM file under '
            '<mission_dir>/results'),
    ))

    results_dir = PathJoinSubstitution(
        [LaunchConfiguration('mission_dir'), 'results'])

    bag_topics = [
        '/anymal/state_estimator/odometry',
        '/boxi/ap20/prism_position',
        '/boxi/inertial_explorer/tc/odometry',
        '/boxi/cpt7/imu',
        '/boxi/stim320/imu',
        '/boxi/hesai/points',
        '/boxi/hesai/points_undistorted',
        '/tf_static',
    ]
    bag_play_command = [
        'ros2', 'bag', 'play', LaunchConfiguration('bag_path'),
        '--clock', '100',
        '--rate', LaunchConfiguration('playback_rate'),
        '--disable-keyboard-controls',
        '--remap',
        '/clock:=/slam_evaluator/grandtour/bag_clock',
        '/boxi/ap20/prism_position:=/slam_evaluator/grandtour/prism_raw',
        '--qos-profile-overrides-path', bag_qos_config,
    ]
    bag_player_once = ExecuteProcess(
        cmd=[*bag_play_command, '--topics', *bag_topics],
        name='grandtour_bag_player',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('loop')),
    )
    bag_player_loop = ExecuteProcess(
        cmd=[*bag_play_command, '--loop', '--topics', *bag_topics],
        name='grandtour_bag_player_loop',
        output='screen',
        condition=IfCondition(LaunchConfiguration('loop')),
    )
    ld.add_action(bag_player_once)
    ld.add_action(bag_player_loop)

    # Stop all consumers when a finite playback completes, and also shut down
    # cleanly if either player variant exits with an error.
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player_once,
            on_exit=[EmitEvent(event=Shutdown(
                reason='GrandTour rosbag playback completed'))],
        )))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player_loop,
            on_exit=[EmitEvent(event=Shutdown(
                reason='GrandTour rosbag player exited'))],
        )))

    bag_adapter = Node(
        namespace=ns,
        package='slam_evaluator',
        executable='grandtour_bag_adapter.py',
        name='grandtour_bag_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'normalize_position': True,
        }],
    )
    ld.add_action(bag_adapter)

    # Leica points live in a disconnected total-station frame. Calibrate their
    # global registration and rotating body-fixed lever arm against IE-TC,
    # then publish corrected positions directly in the RTK frame. Holding the
    # raw samples privately also prevents RViz message-filter queue overflows.
    prism_alignment = Node(
        package='slam_evaluator',
        executable='grandtour_prism_alignment.py',
        name='grandtour_prism_alignment',
        condition=IfCondition(PythonExpression([
            "'", sequence, "' in ['ETH-1', 'HEAP-1', 'ARC-3']",
        ])),
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'prism_topic_in': '/slam_evaluator/grandtour/prism_raw',
            'prism_topic_out': '/boxi/ap20/prism_position',
            'ground_truth_topic': '/boxi/inertial_explorer/tc/odometry',
        }],
    )
    ld.add_action(prism_alignment)

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

    # LIO-SAM hardcodes its body frame as tf_prefix + "base_link", while the
    # GrandTour bag uses "base". Bridge the two without rewriting bag frames.
    static_tf_base_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_base_link',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'base',
            'base_link',
        ],
    )
    ld.add_action(static_tf_base_to_base_link)

    dua_tf_server = Node(
        namespace=ns,
        package='dua_tf_server',
        executable='dua_tf_server_app',
        name='dua_tf_server',
        parameters=[config],
    )
    ld.add_action(dua_tf_server)

    # scan_deskewer = Node(
    #     namespace=ns,
    #     package='scan_deskewer',
    #     executable='scan_deskewer_app',
    #     name='scan_deskewer',
    #     emulate_tty=True,
    #     shell=True,
    #     output='both',
    #     parameters=[config],
    #     remappings=[
    #         ('/get_transform', '/dua_tf_server/get_transform'),
    #         ('/input', LaunchConfiguration('lidar_topic')),
    #         ('/imu', '/boxi/cpt7/imu'),
    #         ('/scan_deskewer/output', '/point_cloud/deskewed'),
    #     ],
    # )
    # ld.add_action(scan_deskewer)

    lio_sam = Node(
        namespace=ns,
        package='lio_sam',
        executable='lio_sam_app',
        name='lio_sam',
        emulate_tty=True,
        output='both',
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
            ('/point_cloud', LaunchConfiguration('lidar_topic')),
            ('/odometry', PythonExpression([
                "'/ekf_global/odometry' if '", motion_prior,
                "' == 'ekf' else '/slam_evaluator/grandtour/odometry'",
            ])),
        ],
    )
    ld.add_action(lio_sam)

    ekf_global = Node(
        namespace=ns,
        package='dua_robot_localization',
        executable='dua_robot_localization_app',
        name='ekf_global',
        emulate_tty=True,
        output='both',
        condition=IfCondition(PythonExpression([
            "'", motion_prior, "' == 'ekf'",
        ])),
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
        ]
    )
    ld.add_action(ekf_global)

    # Generic TUM recorder (dataset-agnostic despite the package name),
    # reused as-is from the IILABS3D launch files.
    tum_recorder = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_tum_recorder',
        name='lio_sam_tum_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_tum')),
        parameters=[{
            'pose_topic': '/lio_sam/map_optimization/pose',
            'pose_msg_type': 'pose_with_covariance',
            'output_dir': results_dir,
            'slam_name': 'lio_sam',
            'append_datetime': True,
            'use_sim_time': True,
        }],
    )
    ld.add_action(tum_recorder)

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
