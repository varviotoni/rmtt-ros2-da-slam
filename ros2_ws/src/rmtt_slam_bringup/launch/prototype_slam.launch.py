"""Unified launch file for the DJI RoboMaster TT 2D SLAM Prototype."""

import os
import socket
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _send_emergency_stop_packet(event, context):
    """External failsafe: fires a burst of stop commands directly to the drone when the launch shuts down."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.2)
        for _ in range(5):
            for cmd in [b'streamoff', b'rc 0 0 0 0', b'motoroff', b'emergency', b'land']:
                sock.sendto(cmd, ('192.168.10.1', 8889))
        sock.close()
    except Exception:
        pass


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rmtt_slam_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    bench_cooling_mode = LaunchConfiguration('bench_cooling_mode')

    # 1. Static TF: base_stabilized -> camera_link (0.03 0 0.02 0 0 0)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=['0.03', '0', '0.02', '0', '0', '0', 'base_stabilized', 'camera_link'],
        output='screen',
    )

    # 2. RMTT Driver Node
    rmtt_driver_node = Node(
        package='rmtt_slam_bringup',
        executable='rmtt_driver_node',
        name='rmtt_driver_node',
        parameters=[
            {
                'bench_cooling_mode': ParameterValue(
                    bench_cooling_mode, value_type=bool
                )
            }
        ],
        output='screen',
    )

    # 3. Depth Anything V2 to PointCloud2 Node
    depth_to_cloud_node = Node(
        package='rmtt_slam_bringup',
        executable='depth_to_cloud_node',
        name='depth_to_cloud_node',
        output='screen',
    )

    # 4. PointCloud2 to LaserScan Node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/cloud_obstacles'),
            ('scan', '/scan'),
        ],
        parameters=[
            {
                'target_frame': 'base_stabilized',
                'transform_tolerance': 0.05,
                'min_height': -0.20,
                'max_height': 0.20,
                'range_min': 0.15,
                'range_max': 4.50,
                'use_inf': True,
            }
        ],
        output='screen',
    )

    # 5. Robot Localization EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    # 6. SLAM Toolbox Async Node
    async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    # 7. Launch-level Shutdown Handler (failsafe stop for motors/flight if launch terminal is killed)
    shutdown_stop_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=_send_emergency_stop_packet)
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            DeclareLaunchArgument(
                'bench_cooling_mode',
                default_value='false',
                description='Enable motor cooling while the RMTT is on the bench.',
            ),
            shutdown_stop_handler,
            static_tf_node,
            rmtt_driver_node,
            depth_to_cloud_node,
            pointcloud_to_laserscan_node,
            ekf_node,
            async_slam_toolbox_node,
        ]
    )
