import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from avoidance_gazebo.obstacle_layout import generate_layout


def _launch_course(context):
    pkg = get_package_share_directory('avoidance_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    template_path = os.path.join(pkg, 'worlds', 'straight_avoidance.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'turtle_car.urdf.xacro')

    seed_text = LaunchConfiguration('obstacle_seed').perform(context).strip()
    spawn_text = LaunchConfiguration('spawn_obstacles').perform(context).strip().lower()
    if spawn_text not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError('spawn_obstacles must be true or false')
    spawn_obstacles = spawn_text in ('true', '1', 'yes', 'on')
    seed = int(seed_text)
    layout = generate_layout(seed)
    (first_x, first_y), (second_x, second_y) = layout.ordered

    with open(template_path, 'r', encoding='utf-8') as source:
        world_text = source.read()
    if spawn_obstacles:
        for name, x, y in (
                ('obstacle_1', first_x, first_y),
                ('obstacle_2', second_x, second_y)):
            pattern = rf'(<model name="{name}">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)'
            replacement = rf'\g<1>{x:.6f} {y:.3f} 0.075 0 0 0\g<2>'
            world_text, count = re.subn(pattern, replacement, world_text, count=1)
            if count != 1:
                raise RuntimeError(f'could not set {name} pose in world template')
    else:
        for name in ('obstacle_1', 'obstacle_2'):
            pattern = rf'\s*<model name="{name}">.*?</model>'
            world_text, count = re.subn(pattern, '', world_text, count=1, flags=re.DOTALL)
            if count != 1:
                raise RuntimeError(f'could not remove {name} from world template')

    world_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='straight_avoidance_', suffix='.sdf',
        encoding='utf-8', delete=False)
    with world_file:
        world_file.write(world_text)
    generated_world = world_file.name

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {generated_world}'}.items(),
    )

    robot_description = Command(['xacro ', xacro_file])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # The 1.30 m vehicle's front end is x=3.90, 0.10 m behind the start line.
    # Wait for the Gazebo GUI scene manager so the dynamic model is rendered.
    spawn_vehicle = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'straight_avoidance',
             '-topic', 'robot_description', '-name', 'turtle_car',
             '-x', '3.25', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
        output='screen',
    )])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    lidar_share = get_package_share_directory('avoidance_lidar')
    route_share = get_package_share_directory('avoidance_route')
    lidar = Node(
        package='avoidance_lidar', executable='front_lidar_detector',
        parameters=[os.path.join(lidar_share, 'config', 'front_lidar.yaml'),
                    {'use_sim_time': True}], output='screen')
    route_file = LaunchConfiguration('route_file')
    route_viewer = Node(
        package='avoidance_route', executable='route_visualizer',
        parameters=[{'csv_path': route_file, 'frame_id': 'odom',
                     'use_sim_time': True}], output='screen')
    route_recorder = Node(
        package='avoidance_route', executable='route_recorder',
        parameters=[os.path.join(route_share, 'config', 'route_recorder.yaml'),
                    {'out_csv': route_file, 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('record_route')),
        output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(lidar_share, 'rviz', 'avoidance_lidar.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen')

    obstacle_logs = ([
        LogInfo(msg=(f'[avoidance_gazebo] Obstacle 1: '
                     f'{"LEFT" if first_y > 0 else "RIGHT"}, '
                     f'x={first_x:.2f}, y={first_y:+.3f}')),
        LogInfo(msg=(f'[avoidance_gazebo] Obstacle 2: '
                     f'{"LEFT" if second_y > 0 else "RIGHT"}, '
                     f'x={second_x:.2f}, y={second_y:+.3f}')),
        LogInfo(msg=f'[avoidance_gazebo] Center longitudinal gap: {second_x - first_x:.6f} m'),
        LogInfo(msg=f'[avoidance_gazebo] Obstacle seed: {layout.seed} '
                    f'({"system-random" if seed < 0 else "fixed"})'),
    ] if spawn_obstacles else [
        LogInfo(msg='[avoidance_gazebo] REFERENCE RECORDING MODE: OBSTACLES DISABLED')
    ])

    return [
        LogInfo(msg=f'[avoidance_gazebo] Generated world: {generated_world}'),
        LogInfo(msg='[avoidance_gazebo] Road: 30.00 m'),
        LogInfo(msg='[avoidance_gazebo] White-line inner width: 1.95 m'),
        LogInfo(msg='[avoidance_gazebo] Start: x=4.00 m'),
        LogInfo(msg='[avoidance_gazebo] Finish: x=26.00 m'),
        LogInfo(msg='[avoidance_gazebo] Start-to-finish distance: 22.00 m'),
        LogInfo(msg='[avoidance_gazebo] Vehicle spawn: x=3.25 m, y=0.00 m, z=0.00 m, yaw=0.00 rad'),
        LogInfo(msg='[avoidance_gazebo] Front LiDAR: x=+0.65 m'),
        LogInfo(msg='[avoidance_gazebo] Rear LiDAR: x=-0.65 m'),
        *obstacle_logs,
        gz,
        rsp,
        bridge,
        spawn_vehicle,
        lidar,
        route_viewer,
        route_recorder,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_seed', default_value='-1',
            description='-1 uses protected system randomness; >=0 is reproducible.'),
        DeclareLaunchArgument(
            'spawn_obstacles', default_value='false',
            description='Generate the two seeded obstacles (default: false).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 with the avoidance diagnostics config.'),
        DeclareLaunchArgument(
            'record_route', default_value='false',
            description='Record actual /odom to route_file during this launch.'),
        DeclareLaunchArgument(
            'route_file',
            default_value=os.path.join(os.path.expanduser('~'), 'avoidance_sim_ws',
                                       'routes', 'straight_reference.csv'),
            description='CSV used by the recorder and route visualizer.'),
        OpaqueFunction(function=_launch_course),
    ])
