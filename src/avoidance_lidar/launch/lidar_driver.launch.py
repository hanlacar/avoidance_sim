"""Start one real front RPLIDAR and publish /scan_front."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('avoidance_lidar')
    config = os.path.join(share, 'config', 'rplidar.yaml')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('lidar_frame')
    scan_mode = LaunchConfiguration('scan_mode')
    base_frame = LaunchConfiguration('base_frame')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('lidar_frame', default_value='front_laser'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_x', default_value='0.65'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.20'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        Node(
            package='rplidar_ros', executable='rplidar_node',
            name='front_rplidar_node', output='screen',
            parameters=[config, {
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'scan_mode': scan_mode,
                'topic_name': '/scan_front',
            }]),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='front_lidar_static_tf', output='screen',
            arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                       '--yaw', lidar_yaw, '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', base_frame, '--child-frame-id', frame_id]),
    ])
