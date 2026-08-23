import os
import random
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _obstacle_layout(seed):
    rng = random.Random(seed)
    first_x = rng.uniform(9.325, 15.675)
    second_x = first_x + 5.0
    first_left = bool(rng.getrandbits(1))
    first_y = 0.585 if first_left else -0.585
    second_y = -first_y
    return first_x, first_y, second_x, second_y


def _launch_course(context):
    pkg = get_package_share_directory('avoidance_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    template_path = os.path.join(pkg, 'worlds', 'straight_avoidance.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'turtle_car.urdf.xacro')

    seed_text = LaunchConfiguration('random_seed').perform(context).strip()
    seed = None if seed_text == '' else int(seed_text)
    first_x, first_y, second_x, second_y = _obstacle_layout(seed)

    with open(template_path, 'r', encoding='utf-8') as source:
        world_text = source.read()
    replacements = {
        '__OBS1_X__': f'{first_x:.6f}',
        '__OBS1_Y__': f'{first_y:.3f}',
        '__OBS2_X__': f'{second_x:.6f}',
        '__OBS2_Y__': f'{second_y:.3f}',
    }
    for placeholder, value in replacements.items():
        world_text = world_text.replace(placeholder, value)

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
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan_front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan_rear@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    return [
        LogInfo(msg=f'[avoidance_gazebo] Generated world: {generated_world}'),
        LogInfo(msg='[avoidance_gazebo] Road: 30.00 m'),
        LogInfo(msg='[avoidance_gazebo] White-line inner width: 1.56 m'),
        LogInfo(msg='[avoidance_gazebo] Start: x=4.00 m'),
        LogInfo(msg='[avoidance_gazebo] Finish: x=26.00 m'),
        LogInfo(msg='[avoidance_gazebo] Start-to-finish distance: 22.00 m'),
        LogInfo(msg='[avoidance_gazebo] Vehicle spawn: x=3.25 m, y=0.00 m, z=0.00 m, yaw=0.00 rad'),
        LogInfo(msg='[avoidance_gazebo] Front LiDAR: x=+0.65 m'),
        LogInfo(msg='[avoidance_gazebo] Rear LiDAR: x=-0.65 m'),
        LogInfo(msg=(f'[avoidance_gazebo] Obstacle 1: '
                     f'{"LEFT" if first_y > 0 else "RIGHT"}, '
                     f'x={first_x:.2f}, y={first_y:+.3f}')),
        LogInfo(msg=(f'[avoidance_gazebo] Obstacle 2: '
                     f'{"LEFT" if second_y > 0 else "RIGHT"}, '
                     f'x={second_x:.2f}, y={second_y:+.3f}')),
        LogInfo(msg='[avoidance_gazebo] Center longitudinal gap: 5.00 m'),
        LogInfo(msg=f'[avoidance_gazebo] Random seed: {seed_text or "system-random"}'),
        gz,
        rsp,
        bridge,
        spawn_vehicle,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'random_seed', default_value='',
            description='Optional integer seed for reproducible obstacle layout.'),
        OpaqueFunction(function=_launch_course),
    ])
