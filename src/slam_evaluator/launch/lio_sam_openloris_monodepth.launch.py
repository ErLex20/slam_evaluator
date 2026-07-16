"""LIO-SAM evaluation on OpenLORIS-Scene using a monocular-depth point cloud.

Variant of lio_sam_openloris.launch.py where /openloris/point_cloud is
produced by the monocular_depth node (RGB image -> TensorRT depth model ->
XYZRGB PointCloud2) instead of the D400 depth camera. The depth-camera
`depth_pointcloud` node is therefore never launched (see
openloris_publishers_rgb.launch.py) to avoid two publishers racing on the
same topic.

monocular_depth's `subscriber.camera.topic_name` parameter must contain the
literal substring "image_rect_color" for the node's built-in camera_info
topic derivation (string substitution in subscribers.cpp) to resolve to the
real `.../camera_info` topic. Since the OpenLORIS bag publishes raw (already
rectified, distortion-free) images under `.../image_raw`, this launch file
declares the subscriber topic as a fake ".../image_rect_color" name and
remaps it to the real ".../image_raw" topic -- monocular_depth's C++ source
is not modified, only its launch-time topic resolution.

Launch arguments:
  bag_path          Converted OpenLORIS ROS 2 bag directory
  play_bag          Start ros2 bag play (default: true)
  rviz              Start RViz2 (default: false)
  record_trajectory Record SLAM output in TUM format (default: true)
  output_dir        Directory for TUM trajectory files
  monodepth_config  Path to the monocular_depth parameters YAML
  run_name          Identifier used for the recorded TUM file name

July 2026
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('slam_evaluator')
    publishers_share = get_package_share_directory(
        'ros2_openloris_publishers')
    config = os.path.join(
        slam_share, 'config', 'lio_sam_openloris.yaml')
    rviz_config = os.path.join(
        slam_share, 'rviz', 'lio_sam_openloris.rviz')
    default_monodepth_config = os.path.join(
        slam_share, 'config', 'monocular_depth_openloris_depth_anything-v2.yaml')

    namespace = LaunchConfiguration('namespace')
    bag_path = LaunchConfiguration('bag_path')
    play_bag = LaunchConfiguration('play_bag')
    use_rviz = LaunchConfiguration('rviz')
    record_trajectory = LaunchConfiguration('record_trajectory')
    output_dir = LaunchConfiguration('output_dir')
    monodepth_config = LaunchConfiguration('monodepth_config')
    run_name = LaunchConfiguration('run_name')

    publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                publishers_share,
                'launch',
                'openloris_publishers_rgb.launch.py',
            )
        ),
        launch_arguments={
            'bag_path': bag_path,
            'play_bag': play_bag,
        }.items(),
    )

    # The normalized wheel odometry supplies base_odom -> base_link.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base_odom',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'base_odom',
        ],
    )

    tf_server = Node(
        namespace=namespace,
        package='dua_tf_server',
        executable='dua_tf_server_app',
        name='dua_tf_server',
        parameters=[config],
        output='screen',
    )

    lio_sam = Node(
        namespace=namespace,
        package='lio_sam',
        executable='lio_sam_app',
        name='lio_sam',
        emulate_tty=True,
        shell=True,
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
            ('/point_cloud', '/openloris/point_cloud'),
            ('/odometry', '/ekf_global/odometry'),
        ],
        output='both',
    )

    ekf = Node(
        namespace=namespace,
        package='dua_robot_localization',
        executable='dua_robot_localization_app',
        name='ekf_global',
        emulate_tty=True,
        shell=True,
        parameters=[config],
        remappings=[
            ('/get_transform', '/dua_tf_server/get_transform'),
        ],
        output='both',
    )

    monocular_depth = Node(
        package='monocular_depth',
        executable='monocular_depth_app',
        name='monocular_depth',
        emulate_tty=True,
        parameters=[monodepth_config],
        remappings=[
            ('~/point_cloud', '/openloris/point_cloud'),
            (
                '/d400/color/image_rect_color',
                '/d400/color/image_raw',
            ),
        ],
        output='both',
    )

    recorder = Node(
        condition=IfCondition(record_trajectory),
        package='ros2_openloris_publishers',
        executable='tum_recorder',
        name='openloris_tum_recorder',
        parameters=[{
            'pose_topic': '/lio_sam/map_optimization/pose',
            'pose_msg_type': 'pose_with_covariance',
            'output_dir': output_dir,
            'slam_name': run_name,
            'append_datetime': True,
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/neo/workspace/logs/openloris/cafe1-1',
            description='Converted OpenLORIS ROS 2 bag directory',
        ),
        DeclareLaunchArgument(
            'play_bag',
            default_value='true',
            description='Start ros2 bag play with the required topics',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Start RViz2'),
        DeclareLaunchArgument(
            'record_trajectory',
            default_value='true',
            description='Record the LIO-SAM trajectory in TUM format',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/neo/workspace/logs/openloris',
            description='Directory for recorded TUM trajectories',
        ),
        DeclareLaunchArgument(
            'monodepth_config',
            default_value=default_monodepth_config,
            description='Path to the monocular_depth parameters YAML',
        ),
        DeclareLaunchArgument(
            'run_name',
            default_value='lio_sam_openloris_monodepth',
            description='Identifier used for the recorded TUM file name',
        ),
        publishers,
        static_map_to_odom,
        tf_server,
        lio_sam,
        ekf,
        monocular_depth,
        recorder,
        rviz,
    ])
