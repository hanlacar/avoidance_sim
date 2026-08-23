from pathlib import Path
import re

from avoidance_gazebo.obstacle_layout import generate_layout, same_positions


ROOT = Path(__file__).parents[1]


def test_launch_defaults_to_obstacle_free_mode():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    assert "'spawn_obstacles', default_value='false'" in launch
    assert 'REFERENCE RECORDING MODE: OBSTACLES DISABLED' in launch
    assert "for name in ('obstacle_1', 'obstacle_2')" in launch


def test_world_has_expected_obstacle_templates():
    world = (ROOT / 'worlds' / 'straight_avoidance.sdf').read_text()
    assert len(re.findall(r'<model name="obstacle_[12]">', world)) == 2
    assert '<size>0.65 0.39 0.15</size>' in world


def test_launch_preserves_vehicle_and_sensor_contracts():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    for expected in ("'-x', '3.25'", "'/scan_front@", "'/scan_rear@",
                     "'/odom@", "'/cmd_vel@"):
        assert expected in launch
    assert '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' in launch
    assert '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' in launch
    assert '/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' in launch
    assert '/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' in launch
    assert '@geometry_msgs/msg/Twist@gz.msgs.Twist' not in launch


def test_replay_launch_defaults_safe_and_starts_single_follower():
    launch = (ROOT / 'launch' / 'route_replay.launch.py').read_text()
    assert "DeclareLaunchArgument('spawn_obstacles', default_value='false')" in launch
    assert "DeclareLaunchArgument('auto_start', default_value='false')" in launch
    assert launch.count("executable='route_follower'") == 1
    assert 'AUTOMATIC START INHIBITED' in launch


def test_planning_launch_defaults_and_control_ownership():
    launch = (ROOT / 'launch' / 'avoidance_planning.launch.py').read_text()
    assert "DeclareLaunchArgument('spawn_obstacles', default_value='true')" in launch
    assert "DeclareLaunchArgument('auto_start', default_value='false')" in launch
    assert "DeclareLaunchArgument('planner_enabled', default_value='true')" in launch
    assert launch.count("executable='route_follower'") == 1
    assert "executable='avoidance_coordinator'" in launch
    assert "'replan_stop_enabled': True" in launch
    assert "DeclareLaunchArgument('obstacle_seed', default_value='-1')" in launch
    assert "DeclareLaunchArgument('replan_trigger_distance_m', default_value='2.0')" in launch
    assert "DeclareLaunchArgument('max_steering_deg', default_value='25.0')" in launch
    assert "DeclareLaunchArgument('auto_start_avoidance', default_value='true')" in launch
    assert "SetEnvironmentVariable('ROS_DOMAIN_ID', '12')" in launch
    assert "SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')" in launch
    coordinator = (ROOT.parent / 'avoidance_planner' / 'avoidance_planner' /
                   'coordinator_node.py').read_text()
    assert "create_publisher(Twist" not in coordinator


def test_planning_launch_propagates_max_steering_to_follower():
    launch = (ROOT / 'launch' / 'avoidance_planning.launch.py').read_text()
    follower_block = launch.split("follower = Node(")[1].split(")\n\n")[0]
    assert "'max_steering_deg': LaunchConfiguration('max_steering_deg')" in follower_block
    assert "'auto_start_avoidance': auto_start_avoidance" in follower_block


def test_clock_bridge_is_single_direction_gz_to_ros_and_unique():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    token = "'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'"
    assert launch.count(token) == 1
    assert '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock' not in launch
    assert '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock' not in launch


def test_all_major_nodes_use_sim_time_and_follower_exit_stops_launch():
    straight = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    planning = (ROOT / 'launch' / 'avoidance_planning.launch.py').read_text()
    assert straight.count("'use_sim_time': True") >= 5
    assert planning.count("'use_sim_time': True") >= 2
    assert 'OnProcessExit' in planning and "target_action=follower" in planning
    assert "Shutdown(reason='route_follower exited')" in planning


def test_launch_has_single_simulator_domain_lock():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    assert launch.count('acquire_simulation_lock(') == 1
    guard = (ROOT / 'avoidance_gazebo' / 'launch_guard.py').read_text()
    for process in ('parameter_bridge /cmd_vel', 'route_follower',
                    'avoidance_coordinator', 'gz sim '):
        assert process in guard


def test_routes_directory_has_exactly_one_reference_csv_and_yaml():
    routes = ROOT.parents[1] / 'routes'
    csv_files = sorted(p.name for p in routes.glob('*.csv'))
    yaml_files = sorted(p.name for p in routes.glob('*.yaml'))
    assert csv_files == ['straight_reference.csv']
    assert yaml_files == ['straight_reference.yaml']


def test_fixed_obstacle_seed_is_reproducible():
    first, second = generate_layout(42), generate_layout(42)
    assert same_positions(first, second)
    assert first.seed == second.seed == 42


def test_random_layout_never_repeats_previous(tmp_path):
    state = tmp_path / 'last.yaml'
    layouts = [generate_layout(-1, state) for _ in range(10)]
    assert all(not same_positions(first, second)
               for first, second in zip(layouts, layouts[1:]))
    for layout in layouts:
        ordered = layout.ordered
        assert {y for _, y in ordered} == {-0.780, 0.780}
        assert 10.0 <= ordered[1][0]-ordered[0][0] <= 11.35
