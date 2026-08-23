import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def _launch_course(context):
    pkg = get_package_share_directory('avoidance_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg, 'worlds', 'straight_avoidance.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'turtle_car.urdf.xacro')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_description = Command(['xacro ', xacro_file])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # The 1.30 m vehicle's front end is x=1.40, 0.10 m behind the start line.
    # Wait for the Gazebo GUI scene manager so the dynamic model is rendered.
    spawn_vehicle = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'straight_avoidance',
             '-topic', 'robot_description', '-name', 'turtle_car',
             '-x', '0.75', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
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
        LogInfo(msg=f'[avoidance_gazebo] World: {world_path}'),
        LogInfo(msg='[avoidance_gazebo] Obstacles: disabled'),
        LogInfo(msg='[avoidance_gazebo] Road: 18.50 m x 2.50 m, curb height: 0.05 m'),
        LogInfo(msg='[avoidance_gazebo] Start: x=1.50 m'),
        LogInfo(msg='[avoidance_gazebo] Finish: x=16.50 m'),
        LogInfo(msg='[avoidance_gazebo] Start-to-finish distance: 15.00 m'),
        LogInfo(msg='[avoidance_gazebo] Vehicle spawn: x=0.75 m, y=0.00 m, z=0.00 m, yaw=0.00 rad'),
        gz,
        rsp,
        bridge,
        spawn_vehicle,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_course),
    ])
