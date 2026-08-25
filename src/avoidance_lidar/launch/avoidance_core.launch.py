"""Start perception, planning, route following, and the safety command gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('avoidance_lidar')
    planner_share = get_package_share_directory('avoidance_planner')
    route_share = get_package_share_directory('avoidance_route')
    route_file = LaunchConfiguration('route_file')
    auto_start = LaunchConfiguration('auto_start')
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file', default_value='',
            description='Optional real-vehicle route CSV; leave empty for Path input.'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(
            package='avoidance_lidar', executable='front_lidar_detector',
            name='front_lidar_detector', output='screen',
            parameters=[os.path.join(lidar_share, 'config', 'front_lidar.yaml')]),
        Node(
            package='avoidance_planner', executable='avoidance_coordinator',
            name='avoidance_coordinator', output='screen',
            parameters=[os.path.join(
                planner_share, 'config', 'avoidance_planner.yaml')]),
        Node(
            package='avoidance_route', executable='route_follower',
            name='route_follower', output='screen',
            parameters=[
                os.path.join(route_share, 'config', 'route_follower.yaml'),
                {'route_file': route_file, 'auto_start': auto_start}]),
        Node(
            package='avoidance_lidar', executable='lidar_safety',
            name='lidar_safety', output='screen',
            parameters=[os.path.join(lidar_share, 'config', 'safety.yaml')]),
        Node(
            package='rviz2', executable='rviz2', name='avoidance_rviz',
            output='screen', condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(
                lidar_share, 'rviz', 'avoidance_lidar.rviz')]),
    ])
