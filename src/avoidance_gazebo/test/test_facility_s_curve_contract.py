import csv
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from avoidance_route.route_following import load_route_csv


ROOT = Path(__file__).parents[1]
WORKSPACE = ROOT.parents[1]
SPAWN_X = 3.25


def test_facility_world_is_fixed_and_has_exact_obstacles():
    world_path = ROOT/'worlds/facility_s_curve_avoidance.sdf'
    root = ET.parse(world_path).getroot()
    worlds = root.findall('world')
    assert len(worlds) == 1
    assert worlds[0].get('name') == 's_curve_avoidance'
    text = world_path.read_text(encoding='utf-8')
    assert len(re.findall(r'<model name="obstacle_1">', text)) == 1
    assert len(re.findall(r'<model name="obstacle_2">', text)) == 1
    assert '<size>1.30 0.78 0.15</size>' in text


def test_facility_route_schema_values_and_yaml_pair():
    csv_path = WORKSPACE/'routes/facility_s_curve_reference.csv'
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == (
        'index', 'timestamp', 'latitude', 'longitude', 'x_m', 'y_m', 'yaw',
        'direction', 'mode', 'drive_level', 'curb_offset')
    assert len(rows) == 286
    assert [int(row['index']) for row in rows] == list(range(286))
    assert all(math.isfinite(float(row[key])) for row in rows
               for key in ('x_m', 'y_m', 'yaw', 'curb_offset'))
    assert max(abs(float(row['yaw'])) for row in rows) <= math.pi
    route, warnings = load_route_csv(csv_path)
    assert len(route) == 286
    assert warnings == ()
    yaml_text = csv_path.with_suffix('.yaml').read_text(encoding='utf-8')
    assert 'yaw_unit: radian' in yaml_text
    assert 'coordinate_source: gazebo_odometry' in yaml_text


def test_facility_route_matches_world_spawn_and_road_extent():
    csv_path = WORKSPACE/'routes/facility_s_curve_reference.csv'
    with csv_path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    start = rows[0]
    goal = rows[-1]
    assert float(start['x_m'])+SPAWN_X == 3.25
    assert float(start['y_m']) == 0.0
    assert float(goal['x_m'])+SPAWN_X == 25.241569
    assert float(goal['y_m']) == 24.113866
    assert all(1.5 <= float(row['curb_offset']) <= 2.1 for row in rows)


def test_facility_launch_is_independent_directional_and_complete():
    launch = (ROOT/'launch/facility_s_curve_planning.launch.py').read_text()
    assert 's_curve_avoidance.launch.py' not in launch
    assert 'tempfile' not in launch
    assert 'generate_s_layout' not in launch
    assert launch.count('acquire_simulation_lock(') == 1
    for token in (
            "'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'",
            "'/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'",
            "'/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'",
            "'/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'",
            "'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'",
            "'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'"):
        assert launch.count(token) == 1
    assert '@nav_msgs/msg/Odometry@gz.msgs.Odometry' not in launch
    assert '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan' not in launch
    assert launch.count("executable='route_follower'") == 1
    assert launch.count("executable='avoidance_coordinator'") == 1
    assert "executable='front_lidar_detector'" in launch
    assert "executable='route_visualizer'" in launch
    assert "'--world', world_name, '--name', 'turtle_car'" in launch
    assert "'--x', '3.25', '--y', '0.0', '--z', '0.30'" in launch
    assert "DeclareLaunchArgument('actual_path_csv', default_value='')" in launch
    assert "'return_transition_lengths_m': [4.0]" in launch
    assert "'lateral_target_samples': 3" in launch
    assert "'fixed_environment_mode': True" in launch
    assert "'fixed_obstacle_s_m': [7.2121, 27.1544]" in launch
    assert "'fixed_obstacle_d_m': [0.6099, -1.2098]" in launch


def test_setup_installs_facility_launch_and_world_patterns():
    setup = (ROOT/'setup.py').read_text()
    assert "glob('launch/*.launch.py')" in setup
    assert "glob('worlds/*.sdf')" in setup


def test_facility_spawn_is_readiness_checked_and_gates_driving_nodes():
    launch = (ROOT/'launch/facility_s_curve_planning.launch.py').read_text()
    verifier = (ROOT/'avoidance_gazebo/spawn_verified_vehicle.py').read_text()
    assert "executable='spawn_verified_vehicle'" in launch
    assert 'TimerAction' not in launch
    assert "target_action=spawn_vehicle" in launch
    assert 'if event.returncode != 0:' in launch
    assert "'gz_args': f'-r -s {world_file}'" in launch
    assert "cmd=['gz', 'sim', '-g', '--force-version', '8']" in launch
    assert 'verified_actions + [' in launch
    assert 'follower_shutdown, follower, planner' in launch
    assert "'/robot_description'" in verifier
    assert '_wait_for_world(args.world, args.timeout)' in verifier
    assert "f'/world/{world_name}/create'" in verifier
    assert "f'/world/{world_name}/scene/info'" in verifier
    assert "'-allow_renaming', 'false'" in verifier
    assert 'duplicate turtle_car MODEL already exists' in verifier
    assert 'Verify twice' in verifier


def test_turtle_car_urdf_has_visible_vehicle_parts():
    root = ET.parse(ROOT/'urdf/turtle_car.urdf.xacro').getroot()
    visuals = root.findall('.//visual')
    assert len(visuals) >= 5
    assert all(visual.find('geometry') is not None for visual in visuals)
    source = (ROOT/'urdf/turtle_car.urdf.xacro').read_text()
    assert '<xacro:wheel_link name="front_left_wheel_link"/>' in source
    assert '<xacro:wheel_link name="front_right_wheel_link"/>' in source
    assert '<xacro:wheel_link name="rear_left_wheel_link"/>' in source
    assert '<xacro:wheel_link name="rear_right_wheel_link"/>' in source
    for link in ('chassis_link', 'laser_link', 'rear_laser_link'):
        element = root.find(f"link[@name='{link}']")
        assert element is not None
        assert element.find('visual/geometry') is not None
        assert element.find('collision/geometry') is not None
        visual_geometry = element.find('visual/geometry')[0]
        collision_geometry = element.find('collision/geometry')[0]
        visual_geometry.tail = None
        collision_geometry.tail = None
        assert ET.tostring(visual_geometry) == ET.tostring(collision_geometry)
