"""Record actual Gazebo odometry in a mission_manager-compatible CSV."""

import csv
from datetime import datetime, timezone
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import yaml

from .route_io import CSV_HEADER, non_overwriting_path


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')
        defaults = {
            'out_csv': '', 'odom_topic': '/odom', 'cmd_topic': '/cmd_vel',
            'min_distance_m': 0.10, 'min_period_s': 0.10,
            'direction_deadband_mps': 0.02, 'record_direction': 'forward',
            'record_mode': 'NORMAL', 'record_drive_level': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        requested = str(self.get_parameter('out_csv').value).strip()
        if not requested:
            raise RuntimeError('out_csv is empty; provide an absolute CSV path')
        self.output_path = non_overwriting_path(requested)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.min_distance = max(0.0, float(self.get_parameter('min_distance_m').value))
        self.min_period = max(0.0, float(self.get_parameter('min_period_s').value))
        self.deadband = max(0.0, float(self.get_parameter('direction_deadband_mps').value))
        self.direction = 1 if str(self.get_parameter('record_direction').value).lower() == 'forward' else -1
        self.mode = str(self.get_parameter('record_mode').value).strip()
        self.drive_level = float(self.get_parameter('record_drive_level').value)
        if not self.mode or self.drive_level not in (1.0, 2.0, 3.0):
            raise ValueError('record_mode must be nonempty and drive_level must be 1, 2, or 3')

        self.stream = self.output_path.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(CSV_HEADER)
        self.stream.flush()
        self._write_metadata()
        self.count = 0
        self.last_xy = None
        self.last_record_time = None
        self.path = PathMsg()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.live_path_pub = self.create_publisher(
            PathMsg, '/avoidance/route/live_path', qos)
        self.saved_path_pub = self.create_publisher(
            PathMsg, '/avoidance/route/saved_path', qos)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._odom, 20)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_topic').value), self._cmd, 10)
        self.get_logger().info(
            f'Recording actual odometry to {self.output_path} '
            f'(distance>={self.min_distance:.3f} m, period>={self.min_period:.3f} s)')

    def _write_metadata(self):
        metadata = {
            'format_version': 1, 'origin_lat': 0.0, 'origin_lon': 0.0,
            'loop': False, 'created_at': datetime.now(timezone.utc).isoformat(),
            'coordinate_source': 'gazebo_odometry', 'yaw_unit': 'radian',
        }
        with self.output_path.with_suffix('.yaml').open('x', encoding='utf-8') as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)

    def _cmd(self, msg):
        if msg.linear.x > self.deadband:
            self.direction = 1
        elif msg.linear.x < -self.deadband:
            self.direction = -1

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom(self, msg):
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        norm = math.sqrt(sum(value * value for value in values[2:]))
        if (not msg.header.frame_id or not all(math.isfinite(value) for value in values)
                or norm < 1.0e-6):
            self.get_logger().warning(
                'Ignoring invalid odometry (frame/finite quaternion check failed)',
                throttle_duration_sec=2.0)
            return
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds / 1e9
        xy = (float(pose.position.x), float(pose.position.y))
        if self.last_xy is not None:
            if math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1]) < self.min_distance:
                return
            if timestamp - self.last_record_time < self.min_period:
                return
        yaw = self._yaw(pose.orientation)
        self.writer.writerow((
            self.count, f'{timestamp:.9f}', '0.0000000000', '0.0000000000',
            f'{xy[0]:.6f}', f'{xy[1]:.6f}', f'{yaw:.9f}', self.direction,
            self.mode, f'{self.drive_level:.2f}'))
        self.stream.flush()
        self.count += 1
        self.last_xy, self.last_record_time = xy, timestamp
        self.path.header = msg.header
        point = PoseStamped()
        point.header = msg.header
        point.pose = pose
        self.path.poses.append(point)
        self.live_path_pub.publish(self.path)
        self.saved_path_pub.publish(self.path)

    def destroy_node(self):
        if hasattr(self, 'stream') and not self.stream.closed:
            self.stream.flush()
            self.stream.close()
        if hasattr(self, 'count'):
            # launch may already have invalidated rosout while delivering
            # SIGINT, so use stdout for this final durability confirmation.
            print(f'[route_recorder] CSV saved: {self.count} points -> '
                  f'{self.output_path}', flush=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RouteRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'route_recorder: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
