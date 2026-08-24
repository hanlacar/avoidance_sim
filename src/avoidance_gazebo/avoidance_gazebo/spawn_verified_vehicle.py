"""Spawn turtle_car only after ROS and Gazebo are ready, then verify it."""

import argparse
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


EXPECTED_LINKS = (
    'base_footprint',
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_left_wheel_link',
    'rear_right_wheel_link',
)
EXPECTED_VISUALS = (
    'chassis_link_visual',
    'front_left_wheel_link_visual',
    'front_right_wheel_link_visual',
    'rear_left_wheel_link_visual',
    'rear_right_wheel_link_visual',
    'laser_link_visual',
    'rear_laser_link_visual',
)


def _run(command, timeout=10.0):
    return subprocess.run(
        command, capture_output=True, text=True, timeout=timeout, check=False)


def _scene_info(world_name):
    result = _run([
        'gz', 'service', '-s', f'/world/{world_name}/scene/info',
        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
        '--timeout', '3000', '--req', ''], timeout=5.0)
    if result.returncode != 0:
        return ''
    return result.stdout


def _model_in_scene(scene):
    marker = 'model {\n  name: "turtle_car"'
    start = scene.find(marker)
    if start < 0:
        return ''
    return scene[start:]


def _vehicle_pose(world_name):
    result = _run([
        'gz', 'topic', '-e', '-t', f'/world/{world_name}/pose/info',
        '-n', '1', '--json-output'], timeout=8.0)
    if result.returncode != 0:
        return None
    try:
        message = json.loads(result.stdout.strip())
    except json.JSONDecodeError:
        return None
    for pose in message.get('pose', []):
        if pose.get('name') == 'turtle_car':
            position = pose.get('position', {})
            return tuple(float(position.get(axis, 0.0)) for axis in ('x', 'y', 'z'))
    return None


class RobotDescriptionWaiter(Node):
    def __init__(self):
        super().__init__('facility_vehicle_spawn_verifier')
        self.description = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/robot_description', self._receive, qos)

    def _receive(self, message):
        if message.data.strip():
            self.description = message.data


def _wait_for_description(timeout):
    node = RobotDescriptionWaiter()
    deadline = time.monotonic() + timeout
    while rclpy.ok() and node.description is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    description = node.description
    node.destroy_node()
    return description


def _validate_description(description):
    if not description:
        raise RuntimeError('/robot_description was not published before spawn')
    root = ET.fromstring(description)
    if root.tag != 'robot' or root.get('name') != 'turtle_car':
        raise RuntimeError('robot_description is not the turtle_car URDF')
    if not root.findall('.//visual'):
        raise RuntimeError('turtle_car robot_description contains no visual geometry')


def _wait_for_world(world_name, timeout):
    required = {
        f'/world/{world_name}/create',
        f'/world/{world_name}/scene/info',
    }
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = _run(['gz', 'service', '-l'], timeout=4.0)
        if result.returncode == 0 and required.issubset(set(result.stdout.splitlines())):
            return
        time.sleep(0.2)
    raise RuntimeError(
        f'Gazebo world [{world_name}] did not expose create and scene services')


def _verify_vehicle(world_name, expected_x, expected_y, ground_z):
    scene = _scene_info(world_name)
    model = _model_in_scene(scene)
    if not model:
        raise RuntimeError('turtle_car is absent from Gazebo scene/info')
    missing_links = [name for name in EXPECTED_LINKS if f'name: "{name}"' not in model]
    if missing_links:
        raise RuntimeError(f'turtle_car MODEL is missing links: {missing_links}')
    missing_visuals = [name for name in EXPECTED_VISUALS if name not in model]
    if missing_visuals:
        raise RuntimeError(f'turtle_car MODEL is missing visuals: {missing_visuals}')

    pose = _vehicle_pose(world_name)
    if pose is None:
        raise RuntimeError('turtle_car pose is absent from Gazebo pose/info')
    x, y, z = pose
    if not all(math.isfinite(value) for value in pose):
        raise RuntimeError(f'turtle_car pose is not finite: {pose}')
    if abs(x-expected_x) > 0.20 or abs(y-expected_y) > 0.20:
        raise RuntimeError(f'turtle_car pose is outside spawn tolerance: {pose}')
    if z < ground_z-0.02 or z > ground_z+0.60:
        raise RuntimeError(
            f'turtle_car z={z:.6f} is not safely on/above ground z={ground_z:.6f}')
    return pose


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--name', default='turtle_car')
    parser.add_argument('--x', type=float, default=3.25)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.30)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--ground-z', type=float, default=-0.025)
    parser.add_argument('--timeout', type=float, default=30.0)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    try:
        _wait_for_world(args.world, args.timeout)
        description = _wait_for_description(args.timeout)
        _validate_description(description)
        if _model_in_scene(_scene_info(args.world)):
            raise RuntimeError('duplicate turtle_car MODEL already exists; refusing spawn')

        command = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', args.world, '-string', description,
            '-name', args.name, '-allow_renaming', 'false',
            '-x', str(args.x), '-y', str(args.y), '-z', str(args.z),
            '-Y', str(args.yaw)]
        result = _run(command, timeout=args.timeout)
        if result.stdout:
            print(result.stdout.rstrip(), flush=True)
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr, flush=True)
        if result.returncode != 0:
            raise RuntimeError(
                f'Gazebo create returned failure code {result.returncode}')

        # Verify twice so a transiently created, immediately removed entity fails.
        first_pose = _verify_vehicle(args.world, args.x, args.y, args.ground_z)
        time.sleep(1.0)
        final_pose = _verify_vehicle(args.world, args.x, args.y, args.ground_z)
        print(
            '[facility_spawn] VERIFIED '
            f'world={args.world} name={args.name} type=MODEL '
            f'pose=({final_pose[0]:.6f},{final_pose[1]:.6f},{final_pose[2]:.6f}) '
            f'first_pose_z={first_pose[2]:.6f} links={len(EXPECTED_LINKS)} '
            f'visuals={len(EXPECTED_VISUALS)}', flush=True)
        return 0
    except (RuntimeError, subprocess.SubprocessError, ET.ParseError) as error:
        print(f'[facility_spawn] FAILED: {error}', file=sys.stderr, flush=True)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
