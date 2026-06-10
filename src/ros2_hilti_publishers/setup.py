from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_hilti_publishers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ErLex20',
    maintainer_email='alessandro.cretu2000@gmail.com',
    description='ROS2 publisher node for the Hilti SLAM Challenge dataset',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hilti_publisher = ros2_hilti_publishers.hilti_publisher_node:main',
        ],
    },
)
