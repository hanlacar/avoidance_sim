import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from avoidance_gazebo.launch_guard import acquire_simulation_lock
from avoidance_gazebo.obstacle_layout import generate_s_layout
from avoidance_gazebo.s_curve_course import SPAWN_X, interpolate_s, offset_point, sampled_centerline
from avoidance_gazebo.s_curve_layout_validation import validate_s_layout


def _launch_course(context):
    lock_path = acquire_simulation_lock(os.environ.get('ROS_DOMAIN_ID', '12'))
    pkg = get_package_share_directory('avoidance_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    template_path = os.path.join(pkg, 'worlds', 's_curve_avoidance.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'turtle_car.urdf.xacro')
    spawn_text = LaunchConfiguration('spawn_obstacles').perform(context).strip().lower()
    if spawn_text not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError('spawn_obstacles must be true or false')
    spawn_obstacles = spawn_text in ('true', '1', 'yes', 'on')
    use_rviz = LaunchConfiguration('use_rviz').perform(context).strip().lower()
    headless = use_rviz not in ('true', '1', 'yes', 'on')
    requested_seed = int(LaunchConfiguration('obstacle_seed').perform(context).strip())
    route_path = LaunchConfiguration('route_file').perform(context)
    max_layout_attempts = int(
        LaunchConfiguration('max_layout_sampling_attempts').perform(context))
    validator = (lambda item: validate_s_layout(item, route_path))
    layout = generate_s_layout(
        requested_seed if spawn_obstacles else 0,
        validator=validator if spawn_obstacles else None,
        max_attempts=max_layout_attempts)
    centerline = sampled_centerline(0.025)
    spawn_s = min(centerline, key=lambda pose: abs(pose.x-SPAWN_X)).s
    obstacle_poses = []
    for route_s, lateral in layout.ordered:
        center = interpolate_s(centerline, spawn_s+route_s)
        x, y = offset_point(center, lateral)
        obstacle_poses.append((x, y, center.yaw, lateral))

    world_text = open(template_path, encoding='utf-8').read()
    if spawn_obstacles:
        for name, (x, y, yaw, _d) in zip(('obstacle_1', 'obstacle_2'), obstacle_poses):
            pattern = rf'(<model name="{name}"><static>true</static><pose>)[^<]+(</pose>)'
            replacement = rf'\g<1>{x:.6f} {y:.6f} 0.075 0 0 {yaw:.8f}\g<2>'
            world_text, count = re.subn(pattern, replacement, world_text, count=1)
            if count != 1:
                raise RuntimeError(f'could not set {name} pose in S-curve template')
    else:
        for name in ('obstacle_1', 'obstacle_2'):
            world_text, count = re.subn(
                rf'\s*<model name="{name}">.*?</model>', '', world_text,
                count=1, flags=re.DOTALL)
            if count != 1:
                raise RuntimeError(f'could not remove {name} from S-curve template')
    world_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='s_curve_avoidance_', suffix='.sdf',
        encoding='utf-8', delete=False)
    with world_file:
        world_file.write(world_text)
    generated_world = world_file.name

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {"-s " if headless else ""}{generated_world}'}.items())
    robot_description = Command(['xacro ', xacro_file])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')
    spawn_vehicle = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-world', 's_curve_avoidance',
             '-topic', 'robot_description', '-name', 'turtle_car',
             '-x', f'{SPAWN_X:.2f}', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
        output='screen')])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    lidar_share = get_package_share_directory('avoidance_lidar')
    route_share = get_package_share_directory('avoidance_route')
    route_file = LaunchConfiguration('route_file')
    lidar = Node(package='avoidance_lidar', executable='front_lidar_detector',
                 parameters=[os.path.join(lidar_share, 'config', 'front_lidar.yaml'),
                             {'use_sim_time': True}], output='screen')
    route_viewer = Node(package='avoidance_route', executable='route_visualizer',
                        parameters=[{'csv_path': route_file, 'frame_id': 'odom',
                                     'use_sim_time': True}], output='screen')
    route_recorder = Node(package='avoidance_route', executable='route_recorder',
                          parameters=[os.path.join(route_share, 'config', 'route_recorder.yaml'),
                                      {'out_csv': route_file, 'use_sim_time': True}],
                          condition=IfCondition(LaunchConfiguration('record_route')),
                          output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', os.path.join(lidar_share, 'rviz', 'avoidance_lidar.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen')
    logs = []
    if spawn_obstacles:
        for index, ((route_s, lateral), (x, y, yaw, _)) in enumerate(
                zip(layout.ordered, obstacle_poses), 1):
            logs.append(LogInfo(msg=(f'[avoidance_gazebo] S obstacle {index}: '
                                     f'{"LEFT" if lateral > 0 else "RIGHT"}, '
                                     f's={route_s:.3f}, world=({x:.3f},{y:.3f}), yaw={yaw:.6f}')))
        logs.append(LogInfo(msg=f'[avoidance_gazebo] Obstacle seed: {layout.seed} '
                                f'({"system-random" if requested_seed < 0 else "fixed"})'))
        logs.append(LogInfo(msg=(f'[avoidance_gazebo] Layout sampling attempts: '
                                 f'{layout.sampling_attempts}')))
    else:
        logs.append(LogInfo(msg='[avoidance_gazebo] S-CURVE OBSTACLES DISABLED'))
    return [LogInfo(msg=f'[avoidance_gazebo] Single-simulator lock: {lock_path}'),
            LogInfo(msg=f'[avoidance_gazebo] Generated S world: {generated_world}'),
            *logs, gz, rsp, bridge, spawn_vehicle, lidar, route_viewer,
            route_recorder, rviz]


def generate_launch_description():
    default_route = os.path.join(os.path.expanduser('~'), 'avoidance_sim_ws',
                                 'routes', 's_curve_reference.csv')
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_seed', default_value='-1'),
        DeclareLaunchArgument('spawn_obstacles', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('record_route', default_value='false'),
        DeclareLaunchArgument('route_file', default_value=default_route),
        DeclareLaunchArgument('max_layout_sampling_attempts', default_value='100'),
        OpaqueFunction(function=_launch_course)])
