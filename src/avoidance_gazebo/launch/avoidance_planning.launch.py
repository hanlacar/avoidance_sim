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
    planner_share = get_package_share_directory('avoidance_planner')
    route_file = LaunchConfiguration('route_file')
    spawn_obstacles = LaunchConfiguration('spawn_obstacles')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start')
    planner_enabled = LaunchConfiguration('planner_enabled')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'straight_avoidance.launch.py')),
        launch_arguments={
            'spawn_obstacles': spawn_obstacles,
            'use_rviz': use_rviz,
            'record_route': 'false',
            'route_file': route_file,
            'random_seed': LaunchConfiguration('random_seed'),
        }.items())

    follower = Node(
        package='avoidance_route', executable='route_follower',
        parameters=[
            os.path.join(route_share, 'config', 'route_follower.yaml'),
            {'route_file': route_file, 'auto_start': auto_start,
             'obstacles_enabled': False, 'replan_stop_enabled': True,
             'actual_path_csv': LaunchConfiguration('actual_path_csv'),
             'use_sim_time': True},
        ], output='screen')

    planner = Node(
        package='avoidance_planner', executable='avoidance_coordinator',
        parameters=[
            os.path.join(planner_share, 'config', 'avoidance_planner.yaml'),
            {'use_sim_time': True},
        ], condition=IfCondition(planner_enabled), output='screen')

    default_route = os.path.join(
        os.path.expanduser('~'), 'avoidance_sim_ws', 'routes',
        'straight_reference.csv')
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=default_route),
        DeclareLaunchArgument('spawn_obstacles', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('planner_enabled', default_value='true'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument(
            'actual_path_csv',
            default_value=os.path.join(os.path.expanduser('~'),
                                       'avoidance_sim_ws', 'routes',
                                       'planning_stop_actual.csv')),
        LogInfo(msg='[avoidance_planning] Planning-only mode: selected path is never driven'),
        LogInfo(msg='[avoidance_planning] route_follower owns the only /cmd_vel controller'),
        simulation,
        follower,
        planner,
    ])

