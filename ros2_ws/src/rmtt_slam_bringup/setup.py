import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rmtt_slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='DJI RoboMaster TT 2D SLAM Prototype bringup, driver, and monocular metric depth to point cloud nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmtt_driver_node = rmtt_slam_bringup.rmtt_driver_node:main',
            'depth_to_cloud_node = rmtt_slam_bringup.depth_to_cloud_node:main',
            'rmtt_teleop_node = rmtt_slam_bringup.rmtt_teleop_node:main',
        ],
    },
)
