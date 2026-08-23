import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('avoidance_gazebo')
    route_share = get_package_share_directory('avoidance_route')
    route_file = LaunchConfiguration('route_file')
    spawn_obstacles = LaunchConfiguration('spawn_obstacles')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start')
    actual_path_csv = LaunchConfiguration('actual_path_csv')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'straight_avoidance.launch.py')),
        launch_arguments={
            'spawn_obstacles': spawn_obstacles,
            'use_rviz': use_rviz,
            'record_route': 'false',
            'route_file': route_file,
        }.items())

    follower = Node(
        package='avoidance_route', executable='route_follower',
        parameters=[
            os.path.join(route_share, 'config', 'route_follower.yaml'),
            {'route_file': route_file, 'auto_start': auto_start,
             'obstacles_enabled': spawn_obstacles,
             'actual_path_csv': actual_path_csv, 'use_sim_time': True},
        ], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value=os.path.join(os.path.expanduser('~'), 'avoidance_sim_ws',
                                       'routes', 'straight_reference.csv')),
        DeclareLaunchArgument('spawn_obstacles', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument(
            'actual_path_csv',
            default_value=os.path.join(os.path.expanduser('~'), 'avoidance_sim_ws',
                                       'routes', 'replay_actual.csv')),
        LogInfo(
            condition=IfCondition(spawn_obstacles),
            msg='[route_replay] WARNING: OBSTACLES ENABLED; AUTOMATIC START INHIBITED'),
        simulation,
        follower,
    ])
