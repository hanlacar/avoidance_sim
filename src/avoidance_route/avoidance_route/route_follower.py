"""Safe actual-odometry CSV replay using forward-only Pure Pursuit."""

import csv
import json
import math
from pathlib import Path
import signal
import statistics
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

from .authority import CommandAuthority, CommandAuthorityError
from .route_following import (
    RouteError, Waypoint, avoidance_rejoin_ready, compute_control, curvature_at,
    load_route_csv, nearest_projection, normalize_angle, path_remaining,
    route_length, safety_reason, start_pose_matches, terminal_curvature)
from .route_io import non_overwriting_path


STATES = ('WAITING_FOR_ODOM', 'WAITING_FOR_ROUTE', 'START_POSE_CHECK',
          'READY', 'FOLLOWING', 'GOAL_REACHED', 'STOPPED',
          'STOPPED_FOR_REPLAN', 'WAITING_FOR_AVOIDANCE_START',
          'FOLLOWING_AVOIDANCE', 'REJOINING_CSV', 'REJOIN_ALIGNING',
          'REJOIN_FAILED', 'TIME_RESET_STOP', 'ERROR')


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')
        defaults = {
            'route_file': '', 'odom_topic': '/odom', 'cmd_topic': '/cmd_vel',
            'auto_start': False, 'obstacles_enabled': False,
            'lookahead_m': 1.20, 'wheelbase_m': 0.77,
            'cruise_speed_mps': 1.0, 'max_steering_deg': 25.0,
            'max_steering_change_deg_per_cycle': 2.0,
            'acceleration_limit_mps2': 0.8, 'goal_tolerance_m': 0.10,
            'start_position_tolerance_m': 0.30,
            'start_yaw_tolerance_deg': 10.0,
            'max_cross_track_error_m': 0.60, 'odom_timeout_s': 0.50,
            'stationary_speed_tolerance_mps': 0.03,
            'control_rate_hz': 20.0, 'actual_path_min_distance_m': 0.03,
            'actual_path_csv': '',
            'replan_stop_enabled': False,
            'replan_required_topic': '/avoidance/replan_required',
            'selected_path_topic': '/avoidance/selected_path',
            'selected_path_timeout_s': 300.0,
            'avoidance_speed_mps': 1.0,
            'rejoin_approach_speed_mps': 0.5,
            'avoidance_lookahead_m': 1.2,
            'avoidance_max_cross_track_error_m': 0.20,
            'avoidance_rejoin_remaining_m': 0.80,
            'avoidance_rejoin_max_overshoot_m': 0.50,
            'avoidance_rejoin_lateral_tolerance_m': 0.10,
            'avoidance_rejoin_yaw_tolerance_deg': 2.0,
            'avoidance_rejoin_steering_tolerance_deg': 2.0,
            'avoidance_rejoin_alignment_distance_m': 1.0,
            'csv_rejoin_lookahead_m': 2.0,
            'csv_rejoin_recovery_distance_m': 6.0,
            'gps_drive_level': 2.0,
            'scan_timeout_s': 0.30,
            'auto_start_avoidance': True,
            'path_ready_hold_sec': 0.30,
            'clock_forward_jump_threshold_s': 2.0,
            'clock_reset_threshold_s': 1.0,
            'tf_timeout_s': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, str(self.p['cmd_topic']), 10)
        self.status_pub = self.create_publisher(String, '/avoidance/route/status', 10)
        self.metrics_pub = self.create_publisher(String, '/avoidance/route/metrics', qos)
        self.nearest_index_pub = self.create_publisher(Int32, '/avoidance/route/nearest_index', 10)
        self.lookahead_index_pub = self.create_publisher(Int32, '/avoidance/route/lookahead_index', 10)
        self.cross_track_pub = self.create_publisher(Float32, '/avoidance/route/cross_track_error', 10)
        self.steering_pub = self.create_publisher(Float32, '/avoidance/route/steering_deg', 10)
        self.reference_pub = self.create_publisher(PathMsg, '/avoidance/route/reference_path', qos)
        self.actual_pub = self.create_publisher(PathMsg, '/avoidance/route/actual_path', qos)
        self.avoidance_actual_pub = self.create_publisher(
            PathMsg, '/avoidance/actual_avoidance_path', qos)
        self.gps_drive_pub = self.create_publisher(Float32, '/gps_drive', 10)
        self.gps_wheel_pub = self.create_publisher(Int32, '/gps_wheel', 10)
        self.lidar_drive_pub = self.create_publisher(Float32, '/lidar_drive', 10)
        self.lidar_wheel_pub = self.create_publisher(Int32, '/lidar_wheel', 10)
        self.control_source_pub = self.create_publisher(
            String, '/avoidance/control_source', qos)
        self.lookahead_marker_pub = self.create_publisher(Marker, '/avoidance/route/lookahead_marker', 10)
        self.nearest_marker_pub = self.create_publisher(Marker, '/avoidance/route/nearest_marker', 10)
        self.segment_marker_pub = self.create_publisher(Marker, '/avoidance/route/segment_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/avoidance/route/goal_marker', qos)
        self.status_marker_pub = self.create_publisher(Marker, '/avoidance/route/status_marker', 10)
        self.traveled_marker_pub = self.create_publisher(
            Marker, '/avoidance/route/traveled_marker', 10)
        self.remaining_marker_pub = self.create_publisher(
            Marker, '/avoidance/route/remaining_marker', 10)
        self.rejoin_marker_pub = self.create_publisher(
            Marker, '/avoidance/route/rejoin_marker', qos)
        self.local_frame_pub = self.create_publisher(
            MarkerArray, '/avoidance/route/local_frenet_frame', 10)
        self.start_service = self.create_service(Trigger, '/avoidance/route/start', self._start)
        self.stop_service = self.create_service(Trigger, '/avoidance/route/stop', self._stop_service)
        self.global_stop_service = self.create_service(
            Trigger, '/avoidance/stop', self._stop_service)
        self.avoidance_start_service = self.create_service(
            Trigger, '/avoidance/start_selected_path', self._start_selected_path)

        self.state = 'WAITING_FOR_ROUTE'
        self.reason = ''
        self.points = ()
        self.route_warnings = ()
        self.odom = None
        self.odom_receive_time = None
        self.start_requested = bool(self.p['auto_start'])
        self.segment = 0
        self.steering = 0.0
        self.speed = 0.0
        self.last_control_time = None
        self.last_control_dt = None
        self.last_clock_time = None
        self.time_reset_resume_state = None
        self.time_reset_is_large = False
        self.time_reset_logged = False
        self.start_time = None
        self.actual_path = PathMsg()
        self.last_actual_xy = None
        self.actual_length = 0.0
        self.cross_track_samples = []
        self.steering_samples = []
        self.visited_indices = set()
        self.actual_stream = None
        self.actual_writer = None
        self.final_metrics = None
        self.selected_points = ()
        self.selected_path_receive_time = None
        self.selected_path_signature = None
        self.avoidance_segment = 0
        self.rejoin_index = 0
        self.transition_stop_cycles = 0
        self.control_source = 'STOP'
        self.planner_state = ''
        self.selected_track_id = -1
        self.avoidance_track_id = -1
        self.avoidance_actual_path = PathMsg()
        self.avoidance_cte_samples = []
        self.avoidance_steering_samples = []
        self.last_scan_receive_time = None
        self.last_rear_scan_receive_time = None
        self.last_odom_stamp_ns = None
        self.last_front_scan_stamp_ns = None
        self.last_rear_scan_stamp_ns = None
        self.selected_path_stamp_ns = None
        self.replan_ignore_until_clear = False
        self.csv_rejoin_recovery = False
        self.csv_rejoin_start_xy = None
        self.avoidance_ready_since = None
        self.avoidance_auto_start_failure_logged = False
        self.authority = None
        try:
            self.authority = CommandAuthority('route_follower')
        except CommandAuthorityError as exc:
            self._error('COMMAND_AUTHORITY_CONFLICT', str(exc))

        route_file = str(self.p['route_file']).strip()
        if self.state != 'ERROR':
            if not route_file:
                self._error('CSV_LOAD_FAILED', 'route_file is empty')
            else:
                try:
                    self.points, self.route_warnings = load_route_csv(route_file)
                    for warning in self.route_warnings:
                        self.get_logger().warning(f'CSV row skipped: {warning}')
                    self.state = 'WAITING_FOR_ODOM'
                    self._publish_reference()
                except RouteError as exc:
                    self._error('CSV_LOAD_FAILED', str(exc))
        if bool(self.p['obstacles_enabled']):
            self.get_logger().warning('OBSTACLES ENABLED: route replay start is inhibited')
            self.start_requested = False

        self.create_subscription(Odometry, str(self.p['odom_topic']), self._odom, 20)
        self.create_subscription(
            Bool, str(self.p['replan_required_topic']), self._replan_required, 10)
        self.create_subscription(
            PathMsg, str(self.p['selected_path_topic']), self._selected_path, qos)
        self.create_subscription(
            String, '/avoidance/planner_status', self._planner_status, qos)
        self.create_subscription(
            Int32, '/avoidance/selected_track_id', self._selected_track, qos)
        self.create_subscription(
            LaserScan, '/scan_front', self._scan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_rear', self._rear_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        period = 1.0 / max(1.0, float(self.p['control_rate_hz']))
        self.create_timer(period, self._safe_control)
        self.get_logger().info(
            f'Route follower loaded {len(self.points)} points; wheelbase=0.77 m, '
            f'max steering=+/-{float(self.p["max_steering_deg"]):.1f} deg; '
            'Twist angular.z is yaw rate (rad/s)')

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0-2.0*(q.y*q.y + q.z*q.z))

    def _odom(self, msg):
        pose = msg.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if not msg.header.frame_id or not all(math.isfinite(v) for v in values):
            self._error('INVALID_ODOMETRY', 'non-finite odometry or empty frame')
            return
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if self._input_stamp_regressed('odom', stamp_ns, self.last_odom_stamp_ns):
            return
        self.last_odom_stamp_ns = stamp_ns
        self.odom = msg
        self.odom_receive_time = self.get_clock().now()
        if self.state == 'WAITING_FOR_ODOM':
            self.state = 'START_POSE_CHECK'
        if self.state in ('READY', 'FOLLOWING', 'GOAL_REACHED', 'STOPPED',
                          'STOPPED_FOR_REPLAN', 'WAITING_FOR_AVOIDANCE_START',
                          'FOLLOWING_AVOIDANCE', 'REJOINING_CSV',
                          'REJOIN_ALIGNING'):
            self._append_actual(msg)
        if self.state in ('FOLLOWING_AVOIDANCE', 'REJOINING_CSV',
                  'REJOIN_ALIGNING'):
            self._append_avoidance_actual(msg)

    def _scan(self, msg):
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if self._input_stamp_regressed(
                'scan_front', stamp_ns, self.last_front_scan_stamp_ns):
            return
        self.last_front_scan_stamp_ns = stamp_ns
        self.last_scan_receive_time = self.get_clock().now()

    def _rear_scan(self, msg):
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if self._input_stamp_regressed(
                'scan_rear', stamp_ns, self.last_rear_scan_stamp_ns):
            return
        self.last_rear_scan_stamp_ns = stamp_ns
        self.last_rear_scan_receive_time = self.get_clock().now()

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _input_stamp_regressed(self, source, stamp_ns, previous_ns):
        if previous_ns is not None and stamp_ns < previous_ns:
            self._handle_time_jump(
                self.get_clock().now(), True,
                f'{source} stamp regressed {previous_ns}->{stamp_ns}')
            return True
        return False

    def _replan_required(self, msg):
        if not bool(self.p['replan_stop_enabled']):
            return
        if self.state in ('ERROR', 'TIME_RESET_STOP'):
            self._publish_stop()
            return
        if not msg.data:
            self.replan_ignore_until_clear = False
            return
        if self.replan_ignore_until_clear:
            return
        if self.state in ('STOPPED_FOR_REPLAN', 'WAITING_FOR_AVOIDANCE_START',
                          'FOLLOWING_AVOIDANCE', 'REJOINING_CSV',
                          'REJOIN_ALIGNING'):
            if self.state in ('STOPPED_FOR_REPLAN', 'WAITING_FOR_AVOIDANCE_START'):
                self._publish_stop()
            return
        self.state, self.reason = 'STOPPED_FOR_REPLAN', 'REPLAN_REQUIRED'
        self.start_requested = False
        self._publish_stop()
        self._publish_status()
        self.get_logger().warning(
            'STOPPED_FOR_REPLAN: exact-zero /cmd_vel latched; automatic restart disabled')

    def _selected_track(self, msg):
        if self.state == 'FOLLOWING_AVOIDANCE' and self.avoidance_track_id >= 0:
            if msg.data != self.avoidance_track_id:
                self._error('SELECTED_TRACK_CHANGED',
                            f'{self.avoidance_track_id}->{msg.data}')
                return
        self.selected_track_id = msg.data

    def _planner_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.planner_state = str(payload.get('state', ''))
        if self.state == 'FOLLOWING_AVOIDANCE':
            if self.planner_state in ('DYNAMIC_OBSTACLE_STOP', 'ERROR'):
                self._error('PLANNER_SAFETY_STOP', self.planner_state)
            elif int(payload.get('runtime_tf_drop_count', 0)) > 0:
                self._error('RUNTIME_TF_FAILURE', payload.get('last_tf_error', ''))

    def _selected_path(self, msg):
        stamp_ns = self._stamp_ns(msg.header.stamp)
        if self._input_stamp_regressed(
                'selected_path', stamp_ns, self.selected_path_stamp_ns):
            return
        values = []
        for index, stamped in enumerate(msg.poses):
            pose = stamped.pose
            yaw = self._yaw(pose.orientation)
            if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, yaw)):
                return
            values.append((index, pose.position.x, pose.position.y, yaw))
        if len(values) < 2:
            return
        signature = (len(values), round(values[0][1], 4), round(values[0][2], 4),
                     round(values[-1][1], 4), round(values[-1][2], 4))
        if self.state == 'FOLLOWING_AVOIDANCE' and signature != self.selected_path_signature:
            self._error('SELECTED_PATH_CHANGED', str(signature))
            return
        self.selected_points = tuple(
            Waypoint(index, x, y, yaw, 1, float(self.p['avoidance_speed_mps']))
            for index, x, y, yaw in values)
        self.selected_path_signature = signature
        self.selected_path_stamp_ns = stamp_ns
        self.selected_path_receive_time = self.get_clock().now()
        if self.state == 'STOPPED_FOR_REPLAN':
            self.state, self.reason = 'WAITING_FOR_AVOIDANCE_START', 'PATH_READY'
            self.avoidance_ready_since = self.get_clock().now()
            self.avoidance_auto_start_failure_logged = False
            self._publish_stop()
            self.get_logger().info(
                'WAITING_FOR_AVOIDANCE_START: path_ready_hold_sec countdown started')

    def _pose(self):
        pose = self.odom.pose.pose
        return pose.position.x, pose.position.y, self._yaw(pose.orientation)

    def _check_start(self):
        x, y, yaw = self._pose()
        matched, position_error, yaw_error = start_pose_matches(
            x, y, yaw, self.points[0], float(self.p['start_position_tolerance_m']),
            math.radians(float(self.p['start_yaw_tolerance_deg'])))
        twist = self.odom.twist.twist
        stationary = (abs(twist.linear.x) <= float(self.p['stationary_speed_tolerance_mps'])
                      and abs(twist.angular.z) <= 0.05)
        if not matched:
            detail = {
                'current_x': x, 'current_y': y, 'current_yaw': yaw,
                'route_start_x': self.points[0].x,
                'route_start_y': self.points[0].y,
                'route_start_yaw': self.points[0].yaw,
                'position_error': position_error, 'yaw_error': yaw_error,
            }
            self.get_logger().error('START_POSE_MISMATCH ' + json.dumps(detail))
            self.state, self.reason = 'STOPPED', 'START_POSE_MISMATCH'
            self._publish_stop()
            return False
        if not stationary:
            self.state, self.reason = 'STOPPED', 'VEHICLE_NOT_STATIONARY'
            self._publish_stop()
            return False
        self.state, self.reason = 'READY', ''
        return True

    def _start(self, _request, response):
        if self.state in ('STOPPED_FOR_REPLAN', 'WAITING_FOR_AVOIDANCE_START'):
            response.success, response.message = False, 'replan stop is latched; reset launch required'
            self._publish_stop()
            return response
        if bool(self.p['obstacles_enabled']):
            response.success, response.message = False, 'obstacles enabled; replay inhibited'
            self._publish_stop()
            return response
        if self.state == 'FOLLOWING':
            response.success, response.message = False, 'already following'
            return response
        if self.state == 'ERROR' or not self.points:
            response.success, response.message = False, self.reason or 'route unavailable'
            self._publish_stop()
            return response
        conflicts = self._conflicting_publishers()
        if conflicts:
            self._error('COMMAND_AUTHORITY_CONFLICT', ', '.join(conflicts))
            response.success = False
            response.message = 'other /cmd_vel publisher: ' + ', '.join(conflicts)
            return response
        if self.odom is None:
            self.start_requested = True
            response.success, response.message = False, 'waiting for odometry'
            return response
        self.state = 'START_POSE_CHECK'
        if not self._check_start():
            response.success, response.message = False, self.reason
            return response
        self._begin_following()
        response.success, response.message = True, 'route replay started'
        return response

    def _attempt_avoidance_start(self):
        """Shared gate used by both the auto-start timer and the manual
        debug service. Returns (ok, message); never forces a start."""
        self._publish_stop()
        if self.state != 'WAITING_FOR_AVOIDANCE_START':
            return False, f'not ready: {self.state}'
        if self.planner_state != 'PATH_READY' or len(self.selected_points) < 2:
            return False, 'selected path is unavailable'
        age = (self.get_clock().now()-self.selected_path_receive_time).nanoseconds/1e9
        if age > float(self.p['selected_path_timeout_s']):
            return False, f'selected path stale: {age:.3f}s'
        if self.odom is None:
            return False, 'odometry unavailable'
        x, y, _ = self._pose()
        if math.hypot(x-self.selected_points[0].x, y-self.selected_points[0].y) > 0.35:
            return False, 'selected path start mismatch'
        self.avoidance_segment = 0
        self.steering = self.speed = 0.0
        self.avoidance_track_id = self.selected_track_id
        self.avoidance_actual_path = PathMsg()
        self.state, self.reason = 'FOLLOWING_AVOIDANCE', ''
        self.control_source = 'LIDAR'
        self.last_control_time = None
        self.get_logger().info('FOLLOWING_AVOIDANCE: LIDAR control source active')
        return True, 'selected avoidance path started'

    def _start_selected_path(self, _request, response):
        response.success, response.message = self._attempt_avoidance_start()
        return response

    def _try_auto_start_avoidance(self, now):
        if not bool(self.p['auto_start_avoidance']):
            return
        if self.state != 'WAITING_FOR_AVOIDANCE_START' or self.avoidance_ready_since is None:
            return
        held = (now-self.avoidance_ready_since).nanoseconds/1e9
        if held < float(self.p['path_ready_hold_sec']):
            return
        ok, message = self._attempt_avoidance_start()
        if ok:
            self.avoidance_ready_since = None
            self.avoidance_auto_start_failure_logged = False
        elif not self.avoidance_auto_start_failure_logged:
            self.avoidance_auto_start_failure_logged = True
            self.get_logger().warning(
                f'AUTO_START_AVOIDANCE_BLOCKED: {message}; remaining stopped')

    def _stop_service(self, _request, response):
        self.state, self.reason = 'STOPPED', 'USER_STOP'
        self.start_requested = False
        self._publish_stop()
        self._close_actual_log()
        response.success, response.message = True, 'route replay stopped'
        return response

    def _begin_following(self):
        if self.state != 'READY':
            return
        self.state = 'FOLLOWING'
        self.reason = ''
        self.start_time = self.get_clock().now()
        self.last_control_time = self.start_time
        self._open_actual_log()
        self.get_logger().info('FOLLOWING: route replay control authority active')

    def _safe_control(self):
        """Never let an unexpected timer exception leave a stale command active."""
        try:
            self._control()
        except Exception as exc:  # final safety boundary around the timer
            self._publish_stop()
            self.state, self.reason = 'ERROR', 'CONTROL_CALLBACK_EXCEPTION'
            self.get_logger().error(f'CONTROL_CALLBACK_EXCEPTION: {exc!r}')
            self._publish_status()

    def _handle_time_jump(self, now, large, detail):
        if self.state != 'TIME_RESET_STOP':
            self.time_reset_resume_state = self.state
        self.state, self.reason = 'TIME_RESET_STOP', detail
        self.time_reset_is_large = self.time_reset_is_large or bool(large)
        self.last_control_time = now
        self.last_control_dt = None
        self.last_clock_time = now
        # Freshness is tied to an epoch. Preserve route/path objects and indices,
        # but require new sensor samples before any command can be generated.
        self.odom_receive_time = None
        self.last_scan_receive_time = None
        self.last_rear_scan_receive_time = None
        self.selected_path_receive_time = None
        self.avoidance_ready_since = None
        self.transition_stop_cycles = 0
        self.csv_rejoin_start_xy = None
        self.start_time = None
        self._publish_stop()
        if not self.time_reset_logged:
            level = 'large simulation reset; manual restart required' if large else (
                'clock discontinuity; waiting for fresh odom/scans/exact TF')
            self.get_logger().error(f'TIME_RESET_STOP: {detail}; {level}')
            self.time_reset_logged = True

    def _tf_is_valid(self):
        if self.odom is None:
            return False
        try:
            stamp = Time.from_msg(self.odom.header.stamp)
            return self.tf_buffer.can_transform(
                'odom', 'base_footprint', stamp,
                timeout=Duration(seconds=float(self.p['tf_timeout_s'])))
        except (TransformException, TypeError, ValueError):
            return False

    def _time_reset_recovery_ready(self):
        if self.time_reset_is_large:
            return False
        required = (self.odom_receive_time, self.last_scan_receive_time,
                    self.last_rear_scan_receive_time)
        if any(value is None for value in required):
            return False
        if self.time_reset_resume_state in ('FOLLOWING_AVOIDANCE', 'REJOIN_ALIGNING'):
            if self.selected_path_receive_time is None:
                return False
        return self._tf_is_valid()

    def _control(self):
        now = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = now
            self.last_control_dt = None
            self.last_clock_time = now
            self._publish_stop()
            self._publish_status()
            return
        delta_ns = (now-self.last_control_time).nanoseconds
        if delta_ns <= 0:
            backwards_ns = max(0, -delta_ns)
            large = (backwards_ns >= int(
                float(self.p['clock_reset_threshold_s'])*1e9))
            self._handle_time_jump(
                now, large, f'non-monotonic ROS time delta_ns={delta_ns}')
            self._publish_status()
            return
        if delta_ns > int(float(self.p['clock_forward_jump_threshold_s'])*1e9):
            self._handle_time_jump(
                now, False, f'forward ROS time jump delta_ns={delta_ns}')
            self._publish_status()
            return
        self.last_clock_time = now
        dt = max(0.001, min(0.2, delta_ns/1e9))
        self.last_control_dt = dt
        self.last_control_time = now
        if self.state == 'TIME_RESET_STOP':
            self._publish_stop()
            if self._time_reset_recovery_ready():
                resume = self.time_reset_resume_state
                if resume in STATES and resume != 'TIME_RESET_STOP':
                    self.state, self.reason = resume, 'TIME_RECOVERED'
                    self.time_reset_resume_state = None
                    self.time_reset_logged = False
                    self.get_logger().warning(
                        'TIME_RECOVERED: fresh odom/scans and exact-stamp TF verified; '
                        'one final zero cycle published')
            self._publish_status()
            return
        if self.odom is None:
            self._publish_stop()
            self._publish_status()
            return
        if self.odom_receive_time is None:
            self._publish_stop()
            self._publish_status()
            return
        odom_age = (now-self.odom_receive_time).nanoseconds/1e9
        if odom_age < 0:
            self._handle_time_jump(now, True, f'odom freshness age={odom_age:.6f}s')
            self._publish_status()
            return
        if odom_age > float(self.p['odom_timeout_s']):
            if self.state == 'FOLLOWING':
                self._error('ODOMETRY_TIMEOUT', f'age={odom_age:.3f}s')
            else:
                self._publish_stop()
            self._publish_status()
            return
        if self.state == 'START_POSE_CHECK':
            if self._check_start() and self.start_requested:
                self._begin_following()
                self._publish_stop()
                self._publish_status()
                return
        elif self.state == 'READY' and self.start_requested:
            self._begin_following()
            self._publish_stop()
            self._publish_status()
            return
        if self.state == 'WAITING_FOR_AVOIDANCE_START':
            self._try_auto_start_avoidance(now)
            if self.state == 'FOLLOWING_AVOIDANCE':
                self._publish_stop()
                self._publish_status()
                return
        if self.state == 'REJOINING_CSV':
            self._publish_stop()
            if self.transition_stop_cycles > 0:
                self.transition_stop_cycles -= 1
                self._publish_status()
                return
            self.segment = max(self.segment, self.rejoin_index)
            self.replan_ignore_until_clear = True
            self.state, self.reason = 'FOLLOWING', 'CSV_REJOINED'
            self.control_source = 'GPS'
            self.csv_rejoin_recovery = True
            self.csv_rejoin_start_xy = self._pose()[:2]
            self.last_control_time = now
            self.get_logger().info(f'FOLLOWING_CSV: rejoined at index {self.segment}')
        if self.state not in ('FOLLOWING', 'FOLLOWING_AVOIDANCE',
                      'REJOIN_ALIGNING'):
            self._publish_stop()
            self._publish_status()
            return

        conflicts = self._conflicting_publishers()
        if conflicts:
            self._error('COMMAND_AUTHORITY_CONFLICT', ', '.join(conflicts))
            return

        avoiding = self.state in ('FOLLOWING_AVOIDANCE', 'REJOIN_ALIGNING')
        if avoiding:
            if self.last_scan_receive_time is None:
                self._error('SCAN_TIMEOUT', 'no front scan received')
                return
            if self.last_rear_scan_receive_time is None:
                self._error('SCAN_TIMEOUT', 'no rear scan received')
                return
            scan_age = (now-self.last_scan_receive_time).nanoseconds/1e9
            rear_scan_age = (now-self.last_rear_scan_receive_time).nanoseconds/1e9
            if scan_age > float(self.p['scan_timeout_s']):
                self._error('SCAN_TIMEOUT', f'age={scan_age:.3f}s')
                return
            if rear_scan_age > float(self.p['scan_timeout_s']):
                self._error('SCAN_TIMEOUT', f'rear age={rear_scan_age:.3f}s')
                return
            if scan_age < 0 or rear_scan_age < 0:
                self._handle_time_jump(
                    now, True, f'scan freshness front={scan_age:.6f} rear={rear_scan_age:.6f}')
                return
            if self.selected_path_receive_time is None:
                self._publish_stop()
                return
            path_age = (now-self.selected_path_receive_time).nanoseconds/1e9
            if path_age > float(self.p['selected_path_timeout_s']):
                self._error('SELECTED_PATH_STALE', f'age={path_age:.3f}s')
                return
        active_points = self.selected_points if avoiding else self.points
        active_segment = self.avoidance_segment if avoiding else self.segment
        x, y, yaw = self._pose()
        if self.csv_rejoin_recovery and self.csv_rejoin_start_xy is not None:
            recovery_distance = math.hypot(
                x-self.csv_rejoin_start_xy[0], y-self.csv_rejoin_start_xy[1])
            if recovery_distance >= float(self.p['csv_rejoin_recovery_distance_m']):
                self.csv_rejoin_recovery = False
                self.csv_rejoin_start_xy = None
                self.get_logger().info('CSV_REJOIN_RECOVERY_COMPLETE')
        goal = active_points[-1]
        goal_distance = math.hypot(goal.x-x, goal.y-y)
        try:
            csv_lookahead = (float(self.p['csv_rejoin_lookahead_m'])
                             if self.csv_rejoin_recovery else
                             float(self.p['lookahead_m']))
            result = compute_control(
                active_points, x, y, yaw, active_segment,
                float(self.p['avoidance_lookahead_m'] if avoiding else
                      csv_lookahead),
                float(self.p['wheelbase_m']), math.radians(float(self.p['max_steering_deg'])),
                self.steering,
                math.radians(float(self.p['max_steering_change_deg_per_cycle'])))
        except (RouteError, ValueError) as exc:
            self._error('TARGET_SEARCH_FAILED', str(exc))
            return
        if avoiding:
            self.avoidance_segment = max(self.avoidance_segment, result.nearest.segment)
        else:
            self.segment = max(self.segment, result.nearest.segment)
        self.visited_indices.add(result.nearest_index)
        remaining_path = lateral = yaw_error = None
        terminal = steering_ready = ready = False
        if avoiding:
            projection = nearest_projection(active_points, x, y,
                                            self.avoidance_segment)
            remaining_path = path_remaining(active_points, projection)
            ready, remaining, lateral, yaw_error = avoidance_rejoin_ready(
                x, y, yaw, goal, float(self.p['avoidance_rejoin_remaining_m']),
                float(self.p['avoidance_rejoin_lateral_tolerance_m']),
                math.radians(float(self.p['avoidance_rejoin_yaw_tolerance_deg'])),
                float(self.p['avoidance_rejoin_max_overshoot_m']))
            terminal = remaining_path <= float(
                self.p['avoidance_rejoin_alignment_distance_m'])
            expected_terminal_steering = math.atan(
                float(self.p['wheelbase_m'])*terminal_curvature(active_points))
            steering_ready = abs(normalize_angle(
                result.steering_rad-expected_terminal_steering)) <= math.radians(
                    float(self.p['avoidance_rejoin_steering_tolerance_deg']))
        # Only after passing the final point does nearest.distance include
        # harmless along-track overshoot.  On a curved route, points in the
        # terminal alignment zone can legitimately be far from the *final*
        # tangent even while they are exactly on the selected path.
        terminal_overshoot = ((x-goal.x)*math.cos(goal.yaw) +
                              (y-goal.y)*math.sin(goal.yaw))
        safety_cross_track = (lateral if avoiding and terminal_overshoot > 0.0
                              else result.nearest.distance)
        reason = safety_reason(
            odom_age, float(self.p['odom_timeout_s']), safety_cross_track,
            float(self.p['avoidance_max_cross_track_error_m'] if avoiding else
                  self.p['max_cross_track_error_m']),
            all(math.isfinite(value) for value in
                (result.steering_rad, result.angular_rate, goal_distance)))
        if reason:
            self._error(reason, f'cross_track={safety_cross_track:.3f}')
            return
        if avoiding:
            if terminal and ready and steering_ready:
                forward_indices = range(self.segment+1, len(self.points))
                self.rejoin_index = min(
                    forward_indices,
                    key=lambda i: math.hypot(self.points[i].x-x,
                                             self.points[i].y-y),
                    default=-1)
                if self.rejoin_index < 0:
                    self.state, self.reason = 'REJOIN_FAILED', 'CSV index unavailable'
                    self._publish_stop()
                    return
                self.state, self.reason = 'REJOINING_CSV', f'index={self.rejoin_index}'
                self.transition_stop_cycles = 1
                self._publish_stop()
                self._publish_rejoin_marker()
                self.get_logger().info(
                    f'REJOINING_CSV: remaining_path={remaining_path:.3f} m, '
                    f'lateral={lateral:.3f} m, '
                    f'yaw_error={math.degrees(yaw_error):.3f} deg, '
                    f'steering={math.degrees(result.steering_rad):.3f} deg, '
                    f'expected_steering={math.degrees(expected_terminal_steering):.3f} deg, '
                    f'index={self.rejoin_index}')
                return
            if terminal and self.state != 'REJOIN_ALIGNING':
                self.state, self.reason = 'REJOIN_ALIGNING', (
                    f'path_remaining={remaining_path:.3f} m, '
                    f'lateral={lateral:.3f} m, '
                    f'yaw_error={math.degrees(yaw_error):.3f} deg, '
                    f'steering={math.degrees(result.steering_rad):.3f} deg')
                self.get_logger().warning('REJOIN_ALIGNING: ' + self.reason)
        if not avoiding and goal_distance <= float(self.p['goal_tolerance_m']):
            self.state, self.reason = 'GOAL_REACHED', ''
            self.csv_rejoin_recovery = False
            self.csv_rejoin_start_xy = None
            self._publish_stop()
            self._finalize_metrics(goal_distance)
            self._close_actual_log()
            self.get_logger().info(f'GOAL_REACHED: final distance={goal_distance:.3f} m')
            return

        distance_from_start = math.hypot(x-self.points[0].x, y-self.points[0].y)
        ramp_speed = min(
            float(self.p['cruise_speed_mps']),
            math.sqrt(max(0.0, 2.0*float(self.p['acceleration_limit_mps2'])*distance_from_start + 0.01)),
            math.sqrt(max(0.0, 2.0*float(self.p['acceleration_limit_mps2'])*goal_distance)))
        desired_speed = (float(self.p['rejoin_approach_speed_mps'])
                 if self.state == 'REJOIN_ALIGNING' else
                 float(self.p['avoidance_speed_mps']) if avoiding else
                 min(ramp_speed, max(0.0, active_points[result.nearest_index].drive_level)))
        max_speed_delta = float(self.p['acceleration_limit_mps2'])*dt
        self.speed += max(-max_speed_delta, min(max_speed_delta, desired_speed-self.speed))
        self.steering = result.steering_rad
        angular = self.speed * result.angular_rate
        if not all(math.isfinite(value) for value in (self.speed, self.steering, angular)):
            self._error('NON_FINITE_CONTROL', 'computed command is not finite')
            return
        command = Twist()
        command.linear.x = self.speed
        command.angular.z = angular
        self.cmd_pub.publish(command)
        steering_deg = math.degrees(self.steering)
        self._publish_source_pair('LIDAR' if avoiding else 'GPS', steering_deg)
        self.cross_track_samples.append(result.nearest.distance)
        self.steering_samples.append(abs(steering_deg))
        self.nearest_index_pub.publish(Int32(data=result.nearest_index))
        self.lookahead_index_pub.publish(Int32(data=result.lookahead_index))
        self.cross_track_pub.publish(Float32(data=float(result.nearest.distance)))
        self.steering_pub.publish(Float32(data=float(steering_deg)))
        self._publish_markers(result, x, y)
        self._write_actual(now, x, y, yaw, result, steering_deg)
        self._publish_status(result, goal_distance)

    def _publish_stop(self):
        self.speed = 0.0
        self.steering = 0.0
        self.cmd_pub.publish(Twist())
        self.steering_pub.publish(Float32(data=0.0))
        if hasattr(self, 'gps_drive_pub'):
            self.gps_drive_pub.publish(Float32(data=0.0))
            self.gps_wheel_pub.publish(Int32(data=0))
            self.lidar_drive_pub.publish(Float32(data=0.0))
            self.lidar_wheel_pub.publish(Int32(data=0))
            self.control_source = 'STOP'
            self.control_source_pub.publish(String(data='STOP'))

    def _publish_source_pair(self, source, steering_deg):
        wheel = int(round(steering_deg))
        if source == 'GPS':
            self.gps_drive_pub.publish(Float32(data=float(self.p['gps_drive_level'])))
            self.gps_wheel_pub.publish(Int32(data=wheel))
            self.lidar_drive_pub.publish(Float32(data=0.0))
            self.lidar_wheel_pub.publish(Int32(data=0))
        elif source == 'LIDAR':
            self.gps_drive_pub.publish(Float32(data=0.0))
            self.gps_wheel_pub.publish(Int32(data=0))
            self.lidar_drive_pub.publish(Float32(data=1.0))
            self.lidar_wheel_pub.publish(Int32(data=wheel))
        else:
            self._publish_stop()
            return
        self.control_source = source
        self.control_source_pub.publish(String(data=source))

    def _conflicting_publishers(self):
        allowed = {self.get_name(), 'ros_gz_bridge'}
        conflicts = []
        for endpoint in self.get_publishers_info_by_topic(str(self.p['cmd_topic'])):
            if endpoint.node_name not in allowed:
                namespace = endpoint.node_namespace.rstrip('/')
                conflicts.append(f'{namespace}/{endpoint.node_name}')
        return sorted(set(conflicts))

    def _error(self, reason, detail):
        self.state, self.reason = 'ERROR', reason
        self._publish_stop()
        self._close_actual_log()
        self.get_logger().error(f'{reason}: {detail}')
        self._publish_status()

    def _publish_reference(self):
        path = PathMsg()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for point in self.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = point.x, point.y
            pose.pose.orientation.z = math.sin(point.yaw/2.0)
            pose.pose.orientation.w = math.cos(point.yaw/2.0)
            path.poses.append(pose)
        self.reference_pub.publish(path)
        self._point_marker(self.goal_marker_pub, 'goal', 0, self.points[-1].x,
                           self.points[-1].y, (1.0, 0.0, 0.0), 0.24)

    def _append_actual(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.last_actual_xy is not None:
            distance = math.hypot(x-self.last_actual_xy[0], y-self.last_actual_xy[1])
            if distance < float(self.p['actual_path_min_distance_m']):
                return
            self.actual_length += distance
        self.last_actual_xy = (x, y)
        self.actual_path.header = msg.header
        pose = PoseStamped()
        pose.header, pose.pose = msg.header, msg.pose.pose
        self.actual_path.poses.append(pose)
        self.actual_pub.publish(self.actual_path)

    def _append_avoidance_actual(self, msg):
        self.avoidance_actual_path.header = msg.header
        pose = PoseStamped()
        pose.header, pose.pose = msg.header, msg.pose.pose
        self.avoidance_actual_path.poses.append(pose)
        self.avoidance_actual_pub.publish(self.avoidance_actual_path)

    def _base_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = namespace, marker_id, marker_type, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    def _point_marker(self, publisher, namespace, marker_id, x, y, color, scale):
        marker = self._base_marker(namespace, marker_id, Marker.SPHERE)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.12
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = color
        publisher.publish(marker)

    def _publish_markers(self, result, x, y):
        self._point_marker(self.nearest_marker_pub, 'route_nearest', 0,
                           result.nearest.x, result.nearest.y, (1.0, 1.0, 0.0), 0.16)
        self._point_marker(self.lookahead_marker_pub, 'route_lookahead', 0,
                           result.target_x, result.target_y, (1.0, 0.0, 1.0), 0.20)
        marker = self._base_marker('target_segment', 0, Marker.LINE_LIST)
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b = 0.1, 0.8, 1.0
        marker.points = [Point(x=x, y=y), Point(x=result.target_x, y=result.target_y)]
        self.segment_marker_pub.publish(marker)
        self._publish_local_frame(result, x, y)
        self._publish_progress_markers()

    def _publish_local_frame(self, result, x, y):
        active = (self.selected_points if self.state in
                  ('FOLLOWING_AVOIDANCE', 'REJOIN_ALIGNING') else self.points)
        index = max(0, min(result.nearest_index, len(active)-1))
        yaw = active[index].yaw
        markers = MarkerArray()
        for marker_id, (name, dx, dy, color) in enumerate((
                ('csv_tangent', 0.8*math.cos(yaw), 0.8*math.sin(yaw), (0.0, 1.0, 0.0)),
                ('csv_normal', -0.6*math.sin(yaw), 0.6*math.cos(yaw), (0.0, 0.5, 1.0)))):
            marker = self._base_marker(name, marker_id, Marker.ARROW)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.04, 0.08, 0.10
            marker.color.r, marker.color.g, marker.color.b = color
            marker.points = [Point(x=result.nearest.x, y=result.nearest.y, z=0.10),
                             Point(x=result.nearest.x+dx, y=result.nearest.y+dy, z=0.10)]
            markers.markers.append(marker)
        text = self._base_marker('local_frenet_status', 2, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 1.55
        text.scale.z = 0.16
        text.color.r = text.color.g = text.color.b = 1.0
        text.text = (f's={index} source={self.control_source} state={self.state}\n'
                     f'CTE={result.nearest.distance:.3f}m '
                     f'heading_err={math.degrees(abs(normalize_angle(self._pose()[2]-yaw))):.2f}deg\n'
                     f'steer={math.degrees(self.steering):.2f}deg '
                     f'kappa={curvature_at(active, index):.4f} 1/m')
        markers.markers.append(text)
        footprint = self._base_marker('vehicle_footprint', 3, Marker.CUBE)
        footprint.pose.position.x, footprint.pose.position.y = x, y
        footprint.pose.position.z = 0.03
        vehicle_yaw = self._pose()[2]
        footprint.pose.orientation.z = math.sin(vehicle_yaw/2.0)
        footprint.pose.orientation.w = math.cos(vehicle_yaw/2.0)
        footprint.scale.x, footprint.scale.y, footprint.scale.z = 1.30, 0.78, 0.06
        footprint.color.g, footprint.color.b, footprint.color.a = 0.9, 0.8, 0.25
        markers.markers.append(footprint)
        self.local_frame_pub.publish(markers)

    def _publish_progress_markers(self):
        """Traveled (grey, 0..segment) vs remaining (blue, segment..end) of
        the single reference CSV, so both are visible simultaneously."""
        index = max(0, min(self.segment, len(self.points)-1))
        traveled = self._base_marker('route_traveled', 0, Marker.LINE_STRIP)
        traveled.scale.x = 0.05
        traveled.color.r = traveled.color.g = traveled.color.b = 0.55
        traveled.points = [Point(x=p.x, y=p.y, z=0.02) for p in self.points[:index+1]]
        self.traveled_marker_pub.publish(traveled)
        remaining = self._base_marker('route_remaining', 0, Marker.LINE_STRIP)
        remaining.scale.x = 0.05
        remaining.color.b = 1.0
        remaining.points = [Point(x=p.x, y=p.y, z=0.02) for p in self.points[index:]]
        self.remaining_marker_pub.publish(remaining)

    def _publish_rejoin_marker(self):
        point = self.points[self.rejoin_index]
        self._point_marker(self.rejoin_marker_pub, 'csv_rejoin_point', 0,
                           point.x, point.y, (0.6, 0.0, 1.0), 0.22)

    def _publish_status(self, result=None, goal_distance=None):
        payload = {'state': self.state, 'reason': self.reason,
                   'control_source': self.control_source,
                   'planner_state': self.planner_state,
                   'csv_index': self.segment,
                   'rejoin_index': self.rejoin_index,
                   'max_steering_deg': float(self.p['max_steering_deg']),
                   'max_observed_steering_deg': max(self.steering_samples, default=0.0),
                   'csv_rejoin_recovery': self.csv_rejoin_recovery,
                   'avoidance_max_steering_deg': max(
                       self.avoidance_steering_samples, default=0.0),
                   'avoidance_max_cte_m': max(
                       self.avoidance_cte_samples, default=0.0),
                   'avoidance_mean_cte_m': statistics.fmean(
                       self.avoidance_cte_samples)
                       if self.avoidance_cte_samples else 0.0}
        if result is not None:
            payload.update({'nearest_index': result.nearest_index,
                            'lookahead_index': result.lookahead_index,
                            'cross_track_error_m': result.nearest.distance,
                            'steering_deg': math.degrees(self.steering),
                            'speed_mps': self.speed,
                            'goal_distance_m': goal_distance})
            active = (self.selected_points if self.state in
                      ('FOLLOWING_AVOIDANCE', 'REJOIN_ALIGNING') else self.points)
            local_index = max(0, min(result.nearest_index, len(active)-1))
            payload.update({
                'heading_error_deg': math.degrees(abs(normalize_angle(
                    self._yaw(self.odom.pose.pose.orientation)-active[local_index].yaw))),
                'local_curvature': curvature_at(active, local_index)})
            if self.state in ('FOLLOWING_AVOIDANCE', 'REJOIN_ALIGNING'):
                self.avoidance_cte_samples.append(result.nearest.distance)
                self.avoidance_steering_samples.append(abs(math.degrees(self.steering)))
                final = self.selected_points[-1]
                before_final = self.selected_points[-2]
                payload.update({
                    'actual_x_m': self.odom.pose.pose.position.x,
                    'actual_y_m': self.odom.pose.pose.position.y,
                    'actual_yaw_rad': self._yaw(self.odom.pose.pose.orientation),
                    'csv_rejoin_yaw_rad': final.yaw,
                    'selected_path_terminal_tangent_rad': math.atan2(
                        final.y-before_final.y, final.x-before_final.x),
                    'selected_path_terminal_x_m': final.x,
                    'selected_path_terminal_y_m': final.y,
                    'rejoin_heading_error_deg': math.degrees(abs(normalize_angle(
                        self._yaw(self.odom.pose.pose.orientation)-final.yaw))),
                    'rejoin_lateral_error_m': abs(-(final.x-
                        self.odom.pose.pose.position.x)*math.sin(final.yaw) +
                        (final.y-self.odom.pose.pose.position.y)*math.cos(final.yaw)),
                    'selected_path_remaining_m': path_remaining(
                        self.selected_points,
                        nearest_projection(self.selected_points,
                                            self.odom.pose.pose.position.x,
                                            self.odom.pose.pose.position.y,
                                            self.avoidance_segment)),
                })
        text = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=text))
        marker = self._base_marker('route_status', 0, Marker.TEXT_VIEW_FACING)
        marker.pose.position.z = 1.2
        marker.scale.z = 0.22
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.text = text
        self.status_marker_pub.publish(marker)

    def _open_actual_log(self):
        requested = str(self.p['actual_path_csv']).strip()
        if not requested:
            return
        path = non_overwriting_path(requested)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.actual_stream = path.open('x', newline='', encoding='utf-8')
        self.actual_writer = csv.writer(self.actual_stream)
        self.actual_writer.writerow(('timestamp', 'x_m', 'y_m', 'yaw',
                                     'cross_track_error_m', 'steering_deg',
                                     'speed_mps', 'nearest_index',
                                     'lookahead_index', 'state'))
        self.actual_stream.flush()
        self.actual_log_path = path

    def _write_actual(self, now, x, y, yaw, result, steering_deg):
        if self.actual_writer is None:
            return
        self.actual_writer.writerow((
            f'{now.nanoseconds/1e9:.9f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.9f}',
            f'{result.nearest.distance:.6f}', f'{steering_deg:.6f}',
            f'{self.speed:.6f}', result.nearest_index, result.lookahead_index,
            self.state))
        self.actual_stream.flush()

    def _close_actual_log(self):
        if self.actual_stream is not None and not self.actual_stream.closed:
            self.actual_stream.flush()
            self.actual_stream.close()
            self.get_logger().info(f'Actual replay log saved: {self.actual_log_path}')
        self.actual_writer = None

    def _finalize_metrics(self, goal_distance):
        errors = sorted(self.cross_track_samples)
        p95 = errors[min(len(errors)-1, math.ceil(0.95*len(errors))-1)] if errors else 0.0
        elapsed = ((self.get_clock().now()-self.start_time).nanoseconds/1e9
                   if self.start_time else 0.0)
        self.final_metrics = {
            'reference_length_m': route_length(self.points),
            'actual_length_m': self.actual_length,
            'max_cross_track_error_m': max(errors, default=0.0),
            'mean_cross_track_error_m': statistics.fmean(errors) if errors else 0.0,
            'p95_cross_track_error_m': p95,
            'max_steering_deg': max(self.steering_samples, default=0.0),
            'mean_steering_deg': statistics.fmean(self.steering_samples) if self.steering_samples else 0.0,
            'goal_distance_m': goal_distance, 'goal_time_s': elapsed,
            'final_speed_command': 0.0, 'final_steering_command_deg': 0.0,
            'max_nearest_index': max(self.visited_indices, default=0),
            'route_last_index': len(self.points)-1,
            'actual_log': str(getattr(self, 'actual_log_path', '')),
        }
        text = json.dumps(self.final_metrics, sort_keys=True)
        self.metrics_pub.publish(String(data=text))
        print('[route_follower] METRICS ' + text, flush=True)

    def destroy_node(self):
        for _ in range(3):
            self._publish_stop()
            time.sleep(0.02)
        self._close_actual_log()
        if self.authority is not None:
            self.authority.close()
        super().destroy_node()


def main(args=None):
    # Keep the context valid until our finally block has published the last
    # exact-zero command; the default rclpy SIGINT handler shuts it down first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = RouteFollower()
        # A bounded wait lets a POSIX SIGINT raised by ros2 launch return to
        # Python promptly instead of remaining inside an unbounded wait-set.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
