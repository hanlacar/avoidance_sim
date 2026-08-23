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
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .authority import CommandAuthority, CommandAuthorityError
from .route_following import (
    RouteError, compute_control, load_route_csv, normalize_angle,
    route_length, safety_reason, start_pose_matches)
from .route_io import non_overwriting_path


STATES = ('WAITING_FOR_ODOM', 'WAITING_FOR_ROUTE', 'START_POSE_CHECK',
          'READY', 'FOLLOWING', 'GOAL_REACHED', 'STOPPED', 'ERROR')


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')
        defaults = {
            'route_file': '', 'odom_topic': '/odom', 'cmd_topic': '/cmd_vel',
            'auto_start': False, 'obstacles_enabled': False,
            'lookahead_m': 0.80, 'wheelbase_m': 0.77,
            'cruise_speed_mps': 1.0, 'max_steering_deg': 20.0,
            'max_steering_change_deg_per_cycle': 2.0,
            'acceleration_limit_mps2': 0.8, 'goal_tolerance_m': 0.10,
            'start_position_tolerance_m': 0.30,
            'start_yaw_tolerance_deg': 10.0,
            'max_cross_track_error_m': 0.60, 'odom_timeout_s': 0.50,
            'stationary_speed_tolerance_mps': 0.03,
            'control_rate_hz': 20.0, 'actual_path_min_distance_m': 0.03,
            'actual_path_csv': '',
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
        self.lookahead_marker_pub = self.create_publisher(Marker, '/avoidance/route/lookahead_marker', 10)
        self.nearest_marker_pub = self.create_publisher(Marker, '/avoidance/route/nearest_marker', 10)
        self.segment_marker_pub = self.create_publisher(Marker, '/avoidance/route/segment_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/avoidance/route/goal_marker', qos)
        self.status_marker_pub = self.create_publisher(Marker, '/avoidance/route/status_marker', 10)
        self.start_service = self.create_service(Trigger, '/avoidance/route/start', self._start)
        self.stop_service = self.create_service(Trigger, '/avoidance/route/stop', self._stop_service)

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
        period = 1.0 / max(1.0, float(self.p['control_rate_hz']))
        self.create_timer(period, self._control)
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
        self.odom = msg
        self.odom_receive_time = self.get_clock().now()
        if self.state == 'WAITING_FOR_ODOM':
            self.state = 'START_POSE_CHECK'
        if self.state in ('READY', 'FOLLOWING', 'GOAL_REACHED', 'STOPPED'):
            self._append_actual(msg)

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

    def _control(self):
        now = self.get_clock().now()
        if self.odom is None:
            self._publish_stop()
            self._publish_status()
            return
        odom_age = (now-self.odom_receive_time).nanoseconds/1e9
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
        elif self.state == 'READY' and self.start_requested:
            self._begin_following()
        if self.state != 'FOLLOWING':
            self._publish_stop()
            self._publish_status()
            return

        conflicts = self._conflicting_publishers()
        if conflicts:
            self._error('COMMAND_AUTHORITY_CONFLICT', ', '.join(conflicts))
            return

        x, y, yaw = self._pose()
        goal = self.points[-1]
        goal_distance = math.hypot(goal.x-x, goal.y-y)
        try:
            result = compute_control(
                self.points, x, y, yaw, self.segment, float(self.p['lookahead_m']),
                float(self.p['wheelbase_m']), math.radians(float(self.p['max_steering_deg'])),
                self.steering,
                math.radians(float(self.p['max_steering_change_deg_per_cycle'])))
        except (RouteError, ValueError) as exc:
            self._error('TARGET_SEARCH_FAILED', str(exc))
            return
        self.segment = max(self.segment, result.nearest.segment)
        self.visited_indices.add(result.nearest_index)
        reason = safety_reason(
            odom_age, float(self.p['odom_timeout_s']), result.nearest.distance,
            float(self.p['max_cross_track_error_m']),
            all(math.isfinite(value) for value in
                (result.steering_rad, result.angular_rate, goal_distance)))
        if reason:
            self._error(reason, f'cross_track={result.nearest.distance:.3f}')
            return
        if goal_distance <= float(self.p['goal_tolerance_m']):
            self.state, self.reason = 'GOAL_REACHED', ''
            self._publish_stop()
            self._finalize_metrics(goal_distance)
            self._close_actual_log()
            self.get_logger().info(f'GOAL_REACHED: final distance={goal_distance:.3f} m')
            return

        dt = max(0.001, min(0.2, (now-self.last_control_time).nanoseconds/1e9))
        self.last_control_time = now
        distance_from_start = math.hypot(x-self.points[0].x, y-self.points[0].y)
        ramp_speed = min(
            float(self.p['cruise_speed_mps']),
            math.sqrt(max(0.0, 2.0*float(self.p['acceleration_limit_mps2'])*distance_from_start + 0.01)),
            math.sqrt(max(0.0, 2.0*float(self.p['acceleration_limit_mps2'])*goal_distance)))
        desired_speed = min(ramp_speed, max(0.0, self.points[result.nearest_index].drive_level))
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

    def _publish_status(self, result=None, goal_distance=None):
        payload = {'state': self.state, 'reason': self.reason}
        if result is not None:
            payload.update({'nearest_index': result.nearest_index,
                            'lookahead_index': result.lookahead_index,
                            'cross_track_error_m': result.nearest.distance,
                            'steering_deg': math.degrees(self.steering),
                            'speed_mps': self.speed,
                            'goal_distance_m': goal_distance})
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
            route = Path(str(self.p['route_file'])).expanduser()
            requested = str(route.with_name(route.stem + '_replay_actual.csv'))
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
        self._publish_stop()
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
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
