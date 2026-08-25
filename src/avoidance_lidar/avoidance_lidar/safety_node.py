"""Sole publisher of the real vehicle drive and steering command topics."""

import json
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

from .safety import LidarSafetyGate, clamp_steering


class LidarSafetyNode(Node):
    def __init__(self):
        super().__init__('lidar_safety')
        defaults = {
            'scan_topic': '/scan_front',
            'requested_drive_topic': '/avoidance/command/drive_requested',
            'requested_wheel_topic': '/avoidance/command/wheel_requested',
            'drive_topic': '/lidar_drive', 'wheel_topic': '/lidar_wheel',
            'stop_distance_m': 0.50, 'resume_distance_m': 0.60,
            'emergency_stop_distance_m': 0.30,
            'front_sector_deg': 30.0, 'min_valid_range_m': 0.05,
            'stop_confirm_scans': 2, 'clear_confirm_scans': 3,
            'scan_timeout_sec': 0.50, 'command_timeout_sec': 0.30,
            'stop_on_scan_timeout': True, 'steering_limit_deg': 27,
            'publish_rate_hz': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not 0 < int(self.p['steering_limit_deg']) <= 27:
            raise ValueError('steering_limit_deg must be in [1, 27]')
        self.gate = LidarSafetyGate(
            self.p['stop_distance_m'], self.p['resume_distance_m'],
            self.p['front_sector_deg'], self.p['min_valid_range_m'],
            self.p['stop_confirm_scans'], self.p['clear_confirm_scans'],
            self.p['scan_timeout_sec'], self.p['stop_on_scan_timeout'],
            self.p['emergency_stop_distance_m'])
        self.requested_drive = 0.0
        self.requested_wheel = 0
        self.last_scan_time = None
        self.last_drive_time = None
        self.last_wheel_time = None
        self.drive_pub = self.create_publisher(
            Float32, str(self.p['drive_topic']), 10)
        self.wheel_pub = self.create_publisher(
            Int32, str(self.p['wheel_topic']), 10)
        self.stop_pub = self.create_publisher(
            Bool, '/avoidance/safety/stop_required', 10)
        self.status_pub = self.create_publisher(
            String, '/avoidance/safety/status', 10)
        self.create_subscription(
            LaserScan, str(self.p['scan_topic']), self._scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Float32, str(self.p['requested_drive_topic']), self._drive, 10)
        self.create_subscription(
            Int32, str(self.p['requested_wheel_topic']), self._wheel, 10)
        self.create_timer(
            1.0 / max(1.0, float(self.p['publish_rate_hz'])), self._tick)
        self.get_logger().info(
            'Safety gate owns /lidar_drive and /lidar_wheel; drive unit is '
            'the MCU discrete level and steering is saturated to +/-27 deg')

    def _scan(self, msg):
        self.last_scan_time = self.get_clock().now()
        previous, current = self.gate.update_scan(msg)
        if current != previous:
            self.get_logger().warning(f'safety state: {previous} -> {current}')

    def _drive(self, msg):
        self.requested_drive = float(msg.data) if math.isfinite(msg.data) else 0.0
        self.last_drive_time = self.get_clock().now()

    def _wheel(self, msg):
        self.requested_wheel = int(msg.data)
        self.last_wheel_time = self.get_clock().now()

    def _command_fresh(self, now):
        if self.last_drive_time is None or self.last_wheel_time is None:
            return False
        timeout = float(self.p['command_timeout_sec'])
        return ((now - self.last_drive_time).nanoseconds / 1e9 <= timeout and
                (now - self.last_wheel_time).nanoseconds / 1e9 <= timeout)

    def _publisher_conflicts(self):
        conflicts = []
        for topic in (str(self.p['drive_topic']), str(self.p['wheel_topic'])):
            for endpoint in self.get_publishers_info_by_topic(topic):
                if endpoint.node_name != self.get_name():
                    conflicts.append(f'{topic}:{endpoint.node_namespace}/{endpoint.node_name}')
        return sorted(set(conflicts))

    def _tick(self):
        now = self.get_clock().now()
        if self.last_scan_time is not None:
            self.gate.update_timeout((now - self.last_scan_time).nanoseconds / 1e9)
        conflicts = self._publisher_conflicts()
        command_fresh = self._command_fresh(now)
        drive, wheel = self.gate.filter_command(
            self.requested_drive, self.requested_wheel)
        stop_required = self.gate.should_stop or not command_fresh or bool(conflicts)
        if stop_required:
            drive = 0.0
        wheel = clamp_steering(wheel, self.p['steering_limit_deg'])
        self.drive_pub.publish(Float32(data=float(drive)))
        self.wheel_pub.publish(Int32(data=wheel))
        self.stop_pub.publish(Bool(data=stop_required))
        self.status_pub.publish(String(data=json.dumps({
            'state': self.gate.state,
            'front_min_distance_m': self.gate.front_min_distance,
            'command_fresh': command_fresh,
            'publisher_conflicts': conflicts,
            'drive_unit': 'mcu_discrete_level',
            'output_drive': drive,
            'output_wheel_deg': wheel,
        }, sort_keys=True)))

    def stop(self):
        self.drive_pub.publish(Float32(data=0.0))
        self.wheel_pub.publish(Int32(data=0))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = LidarSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
