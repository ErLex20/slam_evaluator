from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ros2_grandtour_publishers'


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
        'ROS 2 publisher node for the ETH GrandTour dataset '
        '(raw zarr mission replay)'),
    license='MIT',
    entry_points={
        'console_scripts': [
            'grandtour_publisher = '
            'ros2_grandtour_publishers.grandtour_publisher_node:main',
        ],
    },
)
