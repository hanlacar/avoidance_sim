import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from avoidance_gazebo.launch_guard import acquire_simulation_lock


def _facility_actions(context):
    lock_path = acquire_simulation_lock(os.environ.get('ROS_DOMAIN_ID', '12'))
    gazebo_share = get_package_share_directory('avoidance_gazebo')
    route_share = get_package_share_directory('avoidance_route')
    planner_share = get_package_share_directory('avoidance_planner')
    lidar_share = get_package_share_directory('avoidance_lidar')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(
        gazebo_share, 'worlds', 'facility_s_curve_avoidance.sdf')
    worlds = ET.parse(world_file).getroot().findall('world')
    if len(worlds) != 1 or not worlds[0].get('name'):
        raise RuntimeError('facility SDF must contain exactly one named world')
    world_name = worlds[0].get('name')
    xacro_file = os.path.join(gazebo_share, 'urdf', 'turtle_car.urdf.xacro')
    route_file = LaunchConfiguration('route_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_text = use_rviz.perform(context).strip().lower()
    headless = use_rviz_text not in ('true', '1', 'yes', 'on')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {"-s " if headless else ""}{world_file}',
        }.items())
    robot_description = Command(['xacro ', xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}], output='screen')
    spawn_vehicle = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-world', world_name,
             '-topic', 'robot_description', '-name', 'turtle_car',
             '-x', '3.25', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
        output='screen')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')
    lidar = Node(
        package='avoidance_lidar', executable='front_lidar_detector',
        parameters=[os.path.join(lidar_share, 'config', 'front_lidar.yaml'),
                    {'use_sim_time': True}], output='screen')
    route_visualizer = Node(
        package='avoidance_route', executable='route_visualizer',
        parameters=[{'csv_path': route_file, 'frame_id': 'odom',
                     'use_sim_time': True}], output='screen')
    follower = Node(
        package='avoidance_route', executable='route_follower',
        parameters=[os.path.join(route_share, 'config', 'route_follower.yaml'), {
            'route_file': route_file,
            'auto_start': LaunchConfiguration('auto_start'),
            'obstacles_enabled': False,
            'replan_stop_enabled': True,
            'actual_path_csv': LaunchConfiguration('actual_path_csv'),
            'max_steering_deg': LaunchConfiguration('max_steering_deg'),
            'auto_start_avoidance': LaunchConfiguration('auto_start_avoidance'),
            'avoidance_speed_mps': 0.60,
            'rejoin_approach_speed_mps': 0.40,
            'use_sim_time': True}], output='screen')
    planner = Node(
        package='avoidance_planner', executable='avoidance_coordinator',
        parameters=[os.path.join(planner_share, 'config',
                                 'avoidance_planner.yaml'), {
            'use_sim_time': True,
            'replan_trigger_distance_m':
                LaunchConfiguration('replan_trigger_distance_m'),
            'max_steering_deg': LaunchConfiguration('max_steering_deg'),
            'rejoin_straight_extension_m': 2.5,
            'return_transition_lengths_m': [4.0],
            'lateral_target_samples': 3,
            'fixed_environment_mode': True,
            # Projections of the two immutable SDF boxes onto this fixed CSV.
            'fixed_obstacle_s_m': [7.2121, 27.1544],
            'fixed_obstacle_d_m': [0.6099, -1.2098],
            # The facility CSV/SDF contract has a 1.50 m minimum curb inner
            # offset (wider sections remain conservatively bounded at 1.50).
            'left_curb_inner_y_m': 1.50,
            'right_curb_inner_y_m': -1.50}],
        condition=IfCondition(LaunchConfiguration('planner_enabled')),
        output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(
            lidar_share, 'rviz', 'avoidance_lidar.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz),
        output='screen')
    follower_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=follower,
        on_exit=[LogInfo(msg='[facility_s_curve] route_follower exited; shutting down'),
                 EmitEvent(event=Shutdown(reason='route_follower exited'))]))

    return [
        LogInfo(msg=f'[facility_s_curve] Single-simulator lock: {lock_path}'),
        LogInfo(msg=f'[facility_s_curve] Fixed world: {world_name} ({world_file})'),
        LogInfo(msg='[facility_s_curve] Static obstacles / common Frenet planner / '
                    'single /cmd_vel authority'),
        gazebo, robot_state_publisher, bridge, spawn_vehicle, lidar,
        route_visualizer, rviz, follower_shutdown, follower, planner]


def generate_launch_description():
    default_route = os.path.join(
        os.path.expanduser('~'), 'avoidance_sim_ws', 'routes',
        'facility_s_curve_reference.csv')
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=default_route),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('planner_enabled', default_value='true'),
        DeclareLaunchArgument('auto_start_avoidance', default_value='true'),
        DeclareLaunchArgument('replan_trigger_distance_m', default_value='2.0'),
        DeclareLaunchArgument('max_steering_deg', default_value='25.0'),
        DeclareLaunchArgument('actual_path_csv', default_value=''),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '12'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        OpaqueFunction(function=_facility_actions)])
