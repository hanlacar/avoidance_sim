"""Start the complete real-vehicle front-LiDAR avoidance stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('avoidance_lidar'), 'launch')
    names_defaults = (
        ('serial_port', '/dev/ttyUSB0'),
        ('serial_baudrate', '256000'),
        ('lidar_frame', 'front_laser'),
        ('scan_mode', 'Sensitivity'),
        ('base_frame', 'base_link'),
        ('lidar_x', '0.65'), ('lidar_y', '0.0'),
        ('lidar_z', '0.20'), ('lidar_yaw', '0.0'),
        ('route_file', ''), ('auto_start', 'false'), ('use_rviz', 'false'))
    declared = [DeclareLaunchArgument(name, default_value=default)
                for name, default in names_defaults]
    driver_arguments = {
        name: LaunchConfiguration(name) for name, _default in names_defaults[:9]
    }
    core_arguments = {
        name: LaunchConfiguration(name) for name, _default in names_defaults[9:]
    }
    return LaunchDescription(declared + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'lidar_driver.launch.py')),
            launch_arguments=driver_arguments.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'avoidance_core.launch.py')),
            launch_arguments=core_arguments.items()),
    ])
