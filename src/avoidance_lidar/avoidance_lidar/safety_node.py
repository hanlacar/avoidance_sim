"""Sole publisher of the real vehicle drive and steering command topics."""

import json
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
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
            'front_sector_deg': 90.0, 'min_valid_range_m': 0.05,
            'corridor_width_m': 1.20, 'wheelbase_m': 0.77,
            'roi_min_length_m': 1.50, 'roi_max_length_m': 5.0,
            'roi_time_horizon_sec': 1.50,
            'reaction_time_sec': 0.30, 'max_deceleration_mps2': 1.0,
            'ttc_enabled': True, 'ttc_stop_sec': 1.50,
            'ttc_confirm_scans': 2, 'closing_speed_alpha': 0.50,
            'max_closing_speed_mps': 10.0,
            'stop_confirm_scans': 2, 'clear_confirm_scans': 3,
            'scan_timeout_sec': 0.50, 'command_timeout_sec': 0.30,
            'stop_on_scan_timeout': True, 'steering_limit_deg': 27,
            'publish_rate_hz': 20.0, 'odom_topic': '/odom',
            'odom_timeout_sec': 0.50, 'steering_sign': -1.0,
            'active_topic': '/avoidance/active',
            'active_timeout_sec': 0.30,
            'mode_topic': '/mcu/current_mode',
            'allowed_avoidance_modes': ['5'],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not 0 < int(self.p['steering_limit_deg']) <= 27:
            raise ValueError('steering_limit_deg must be in [1, 27]')
        self.gate = LidarSafetyGate(**{
            name: self.p[name] for name in (
                'stop_distance_m', 'resume_distance_m',
                'front_sector_deg', 'min_valid_range_m',
                'stop_confirm_scans', 'clear_confirm_scans',
                'scan_timeout_sec', 'stop_on_scan_timeout',
                'emergency_stop_distance_m', 'corridor_width_m',
                'wheelbase_m', 'roi_min_length_m', 'roi_max_length_m',
                'roi_time_horizon_sec', 'reaction_time_sec',
                'max_deceleration_mps2', 'ttc_enabled', 'ttc_stop_sec',
                'ttc_confirm_scans', 'closing_speed_alpha',
                'max_closing_speed_mps')})
        self.requested_drive = 0.0
        self.requested_wheel = 0
        self.last_scan_time = None
        self.last_drive_time = None
        self.last_wheel_time = None
        self.last_odom_time = None
        self.ego_speed_mps = 0.0
        self.active = False
        self.last_active_time = None
        self.current_mode = None
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
        self.create_subscription(
            Odometry, str(self.p['odom_topic']), self._odom, 10)
        self.create_subscription(
            Bool, str(self.p['active_topic']), self._active, 10)
        self.create_subscription(
            String, str(self.p['mode_topic']), self._mode, 10)
        self.create_timer(
            1.0 / max(1.0, float(self.p['publish_rate_hz'])), self._tick)
        self.get_logger().info(
            'Safety gate owns /lidar_drive and /lidar_wheel; drive unit is '
            'the MCU discrete level and steering is saturated to +/-27 deg')

    def _scan(self, msg):
        self.last_scan_time = self.get_clock().now()
        speed = self.ego_speed_mps
        if (self.last_odom_time is None or
                (self.last_scan_time - self.last_odom_time).nanoseconds / 1e9 >
                float(self.p['odom_timeout_sec'])):
            speed = 0.0
        physical_steering = float(self.p['steering_sign']) * self.requested_wheel
        previous, current = self.gate.update_scan(
            msg, timestamp=self.last_scan_time.nanoseconds / 1e9,
            ego_speed_mps=speed, steering_deg=physical_steering)
        if current != previous:
            self.get_logger().warning(f'safety state: {previous} -> {current}')

    def _drive(self, msg):
        self.requested_drive = float(msg.data) if math.isfinite(msg.data) else 0.0
        self.last_drive_time = self.get_clock().now()

    def _wheel(self, msg):
        self.requested_wheel = int(msg.data)
        self.last_wheel_time = self.get_clock().now()

    def _odom(self, msg):
        speed = float(msg.twist.twist.linear.x)
        self.ego_speed_mps = speed if math.isfinite(speed) else 0.0
        self.last_odom_time = self.get_clock().now()

    def _active(self, msg):
        self.active = bool(msg.data)
        self.last_active_time = self.get_clock().now()

    def _mode(self, msg):
        self.current_mode = str(msg.data).strip()
        if not self._mode_allowed():
            self.active = False

    def _mode_allowed(self):
        return self.current_mode in {
            str(value).strip() for value in self.p['allowed_avoidance_modes']}

    def _active_authority_valid(self, now):
        if (not self._mode_allowed() or not self.active or
                self.last_active_time is None):
            return False
        age = (now-self.last_active_time).nanoseconds/1e9
        return 0.0 <= age <= float(self.p['active_timeout_sec'])

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
        authority_active = self._active_authority_valid(now)
        if not authority_active:
            active_heartbeat_stale = bool(
                self.active and self._mode_allowed() and
                self.last_active_time is not None)
            self.drive_pub.publish(Float32(data=0.0))
            self.wheel_pub.publish(Int32(data=0))
            self.stop_pub.publish(Bool(data=active_heartbeat_stale))
            self.status_pub.publish(String(data=json.dumps({
                'state': ('ACTIVE_HEARTBEAT_TIMEOUT'
                          if active_heartbeat_stale else 'INACTIVE'),
                'avoidance_active': self.active,
                'mode_allowed': self._mode_allowed(),
                'mcu_mode': self.current_mode,
                'output_drive': 0.0,
                'output_wheel_deg': 0,
            }, sort_keys=True)))
            return
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
            'effective_roi_length_m': self.gate.effective_roi_length_m,
            'effective_stop_distance_m': self.gate.effective_stop_distance_m,
            'closing_speed_mps': self.gate.closing_speed_mps,
            'ttc_sec': self.gate.ttc_sec,
            'risk_level': self.gate.risk_level,
            'ego_speed_mps': self.ego_speed_mps,
            'physical_steering_deg': (
                float(self.p['steering_sign']) * self.requested_wheel),
            'command_fresh': command_fresh,
            'publisher_conflicts': conflicts,
            'drive_unit': 'mcu_discrete_level',
            'output_drive': drive,
            'output_wheel_deg': wheel,
            'avoidance_active': True,
            'mode_allowed': True,
            'mcu_mode': self.current_mode,
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
