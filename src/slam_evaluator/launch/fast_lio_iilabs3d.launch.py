"""
Original FAST-LIO2 launch file for the IILABS3D dataset.

The 'sensor' argument selects the sequence directory and the parameter file:
  - livox_mid_360            -> sensor:=livox_mid-360
  - ouster_os1_64            -> sensor:=ouster_os1-64
  - robosense_rs_helios_5515 -> sensor:=robosense_rs-helios-5515
  - velodyne_vlp_16          -> sensor:=velodyne_vlp-16

Unlike spark_fast_lio, original FAST-LIO does not consume the bag /tf_static
to compute base -> LiDAR extrinsics. It uses the LiDAR -> IMU extrinsics from
the YAML file and publishes its odometry in its own mapping frame, typically
camera_init/body in the upstream implementation.

For benchmark recording, this launch remaps /Odometry to /fast_lio/odometry
and records that topic.

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

    config = PathJoinSubstitution([
        pkg_share,
        'config',
        ['fast_lio_iilabs3d_', LaunchConfiguration('sensor'), '.yaml'],
    ])

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
        'fast_lio_iilabs3d.rviz'
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

    # Ground truth replay: eve/base_link trajectory in map frame.
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

    # SLAM pose recorder:
    # writes <sequence_dir>/results/fast_lio_<datetime>.tum
    tum_recorder = Node(
        package='ros2_iilabs3d_publishers',
        executable='iilabs3d_tum_recorder',
        name='iilabs3d_tum_recorder',
        output='screen',
        parameters=[{
            'pose_topic': '/fast_lio/odometry',
            'pose_msg_type': 'odometry',
            'output_dir': PathJoinSubstitution([sequence_dir, 'results']),
            'slam_name': 'fast_lio',
            'append_datetime': True,
            'use_sim_time': True,
        }],
    )
    ld.add_action(tum_recorder)

    # Keep the same auxiliary TF used by the Spark launch. Original FAST-LIO
    # usually publishes camera_init -> body, so this does not fix FAST-LIO's
    # internal frame convention; it only keeps the IILABS3D tree complete.
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

    fast_lio = Node(
        namespace=ns,
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        emulate_tty=True,
        shell=True,
        output='both',
        parameters=[config],
        remappings=[
            # Inputs. Original FAST-LIO reads topic names from YAML, but these
            # remaps make the setup robust if the code still uses default names.
            ('/livox/lidar', lidar_topic),
            ('/livox/imu', '/eve/imu/data'),
            ('/velodyne_points', lidar_topic),
            ('/os_cloud_node/points', lidar_topic),

            # Outputs. Upstream FAST-LIO commonly publishes these absolute names.
            ('/Odometry', '/fast_lio/odometry'),
            ('/path', '/fast_lio/path'),
            ('/cloud_registered', '/fast_lio/cloud_registered'),
            ('/cloud_registered_body', '/fast_lio/cloud_registered_body'),
            ('/cloud_effected', '/fast_lio/cloud_effected'),
            ('/Laser_map', '/fast_lio/Laser_map'),
        ],
    )
    ld.add_action(fast_lio)

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