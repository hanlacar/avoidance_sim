import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('avoidance_gazebo')
    route_share = get_package_share_directory('avoidance_route')
    planner_share = get_package_share_directory('avoidance_planner')
    route_file = LaunchConfiguration('route_file')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 's_curve_avoidance.launch.py')),
        launch_arguments={
            'spawn_obstacles': LaunchConfiguration('spawn_obstacles'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'record_route': 'false', 'route_file': route_file,
            'obstacle_seed': LaunchConfiguration('obstacle_seed')}.items())
    follower = Node(
        package='avoidance_route', executable='route_follower',
        parameters=[os.path.join(route_share, 'config', 'route_follower.yaml'), {
            'route_file': route_file,
            'auto_start': LaunchConfiguration('auto_start'),
            'obstacles_enabled': False, 'replan_stop_enabled': True,
            'actual_path_csv': LaunchConfiguration('actual_path_csv'),
            'max_steering_deg': LaunchConfiguration('max_steering_deg'),
            'auto_start_avoidance': LaunchConfiguration('auto_start_avoidance'),
            'avoidance_speed_mps': 0.60,
            'rejoin_approach_speed_mps': 0.40,
            'use_sim_time': True}], output='screen')
    planner = Node(
        package='avoidance_planner', executable='avoidance_coordinator',
        parameters=[os.path.join(planner_share, 'config', 'avoidance_planner.yaml'), {
            'use_sim_time': True,
            'replan_trigger_distance_m': LaunchConfiguration('replan_trigger_distance_m'),
            'max_steering_deg': LaunchConfiguration('max_steering_deg'),
            'rejoin_straight_extension_m': 2.5}],
        condition=IfCondition(LaunchConfiguration('planner_enabled')), output='screen')
    follower_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=follower,
        on_exit=[LogInfo(msg='[s_curve_planning] route_follower exited; shutting down'),
                 EmitEvent(event=Shutdown(reason='route_follower exited'))]))
    default_route = os.path.join(os.path.expanduser('~'), 'avoidance_sim_ws',
                                 'routes', 's_curve_reference.csv')
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=default_route),
        DeclareLaunchArgument('spawn_obstacles', default_value='true'),
        DeclareLaunchArgument('obstacle_seed', default_value='-1'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('planner_enabled', default_value='true'),
        DeclareLaunchArgument('auto_start_avoidance', default_value='true'),
        DeclareLaunchArgument('replan_trigger_distance_m', default_value='2.0'),
        DeclareLaunchArgument('max_steering_deg', default_value='25.0'),
        DeclareLaunchArgument('actual_path_csv', default_value=''),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '12'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        LogInfo(msg='[s_curve_planning] Common Frenet planner / single /cmd_vel authority'),
        simulation, follower_exit_shutdown, follower, planner])
