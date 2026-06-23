from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ros2_openloris_publishers'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ErLex20',
    maintainer_email='alessandro.cretu2000@gmail.com',
    description=(
        'ROS 2 sensor adapters and evaluation publishers for OpenLORIS'),
    license='MIT',
    entry_points={
        'console_scripts': [
            'clock_relay = ros2_openloris_publishers.clock_relay_node:main',
            'depth_pointcloud = '
            'ros2_openloris_publishers.depth_pointcloud_node:main',
            'imu_merger = ros2_openloris_publishers.imu_merger_node:main',
            'odometry_tf = ros2_openloris_publishers.odometry_tf_node:main',
            'ground_truth = '
            'ros2_openloris_publishers.ground_truth_node:main',
            'tum_recorder = '
            'ros2_openloris_publishers.tum_recorder_node:main',
        ],
    },
)
