"""ROS coordinator for perception, safe stop, and stopped local planning.

This node never publishes ``/cmd_vel`` and never drives a selected path.  It
publishes a latched stop/replan contract; the existing route_follower remains
the sole Gazebo command authority.
"""

from concurrent.futures import ThreadPoolExecutor
import json
import math
import resource
import signal
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .collision_evaluator import (
    ReplanDebounce, evaluate_track_collision, track_box)
from .geometry import Pose2
from .local_planner import plan_candidates
from .perception import (
    DYNAMIC_OBSTACLE, STATIC_OBSTACLE, TrackManager, ScanPoint,
    cluster_groups, split_walls_and_objects, preprocess_scan)


PLANNER_STATES = (
    'FOLLOWING_CSV', 'OBSTACLE_CANDIDATE', 'OBSTACLE_CONFIRMED',
    'REPLAN_REQUIRED', 'STOPPING', 'STOPPED_FOR_PLANNING',
    'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
    'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES', 'PATH_READY',
    'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP', 'ERROR')


class AvoidanceCoordinator(Node):
    def __init__(self):
        super().__init__('avoidance_coordinator')
        defaults = {
            'front_scan_topic': '/scan_front', 'rear_scan_topic': '/scan_rear',
            'odom_topic': '/odom',
            'reference_path_topic': '/avoidance/route/reference_path',
            'target_frame': 'odom', 'planning_roi_x_min_m': 0.10,
            'planning_roi_x_max_m': 7.0, 'planning_roi_half_width_m': 1.30,
            'self_x_min_m': -0.75, 'self_x_max_m': 0.75,
            'self_y_half_width_m': 0.47, 'min_cluster_points': 4,
            'min_cluster_width_m': 0.08, 'max_cluster_gap_near_m': 0.10,
            'max_cluster_gap_far_m': 0.20,
            'adaptive_gap_distance_m': 8.0, 'confirmation_frames': 3,
            'lost_frames': 5, 'scan_timeout_sec': 0.30,
            'tf_lookup_timeout_sec': 0.01,
            'association_distance_m': 0.45,
            'velocity_filter_alpha': 0.35,
            'dynamic_enter_speed_mps': 0.15,
            'dynamic_exit_speed_mps': 0.08,
            'dynamic_confirmation_frames': 3,
            'static_confirmation_frames': 5,
            'wall_min_length_m': 1.50, 'wall_max_residual_m': 0.05,
            'wall_parallel_tolerance_deg': 15.0,
            'vehicle_length_m': 1.30, 'vehicle_width_m': 0.78,
            'vehicle_center_x_offset_m': 0.0, 'wheelbase_m': 0.77,
            'max_steering_deg': 20.0,
            'obstacle_safety_lateral_m': 0.20,
            'obstacle_safety_longitudinal_m': 0.15,
            'minimum_obstacle_depth_m': 0.65,
            'curb_safety_m': 0.08, 'left_curb_inner_y_m': 1.095,
            'right_curb_inner_y_m': -1.095,
            'obstacle_monitor_distance_m': 5.0,
            'replan_trigger_distance_m': 3.0,
            'emergency_stop_distance_m': 0.5,
            'stopped_linear_speed_mps': 0.03,
            'stopped_angular_speed_rps': 0.03,
            'stopped_confirmation_sec': 0.30,
            'path_sample_interval_m': 0.05,
            'collision_check_interval_m': 0.02,
            'return_transition_lengths_m': [2.0, 2.5, 3.0],
            'corridor_candidate_fractions': [0.25, 0.50, 0.75],
            'marker_lifetime_sec': 2.0, 'planning_rate_hz': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self._validate_parameters()

        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.replan_pub = self.create_publisher(
            Bool, '/avoidance/replan_required', transient)
        self.status_pub = self.create_publisher(
            String, '/avoidance/planner_status', transient)
        self.selected_path_pub = self.create_publisher(
            Path, '/avoidance/selected_path', transient)
        self.candidate_pub = self.create_publisher(
            MarkerArray, '/avoidance/candidate_paths', transient)
        self.static_pub = self.create_publisher(
            MarkerArray, '/avoidance/static_obstacles', transient)
        self.dynamic_pub = self.create_publisher(
            MarkerArray, '/avoidance/dynamic_obstacles', transient)
        self.walls_pub = self.create_publisher(
            MarkerArray, '/avoidance/walls', transient)
        self.unknown_pub = self.create_publisher(
            MarkerArray, '/avoidance/unknown_objects', transient)
        self.corridor_pub = self.create_publisher(
            Marker, '/avoidance/corridor_marker', transient)
        self.planning_points_pub = self.create_publisher(
            MarkerArray, '/avoidance/planning_points', transient)
        self.status_marker_pub = self.create_publisher(
            Marker, '/avoidance/planner_status_marker', transient)
        self.lidar_drive_pub = self.create_publisher(Float32, '/lidar_drive', 10)
        self.lidar_wheel_pub = self.create_publisher(Int32, '/lidar_wheel', 10)
        self.nearest_distance_pub = self.create_publisher(
            Float32, '/avoidance/nearest_obstacle_distance', 10)
        self.selected_track_pub = self.create_publisher(
            Int32, '/avoidance/selected_track_id', transient)
        self.max_steering_pub = self.create_publisher(
            Float32, '/avoidance/max_required_steering_deg', transient)
        self.max_curvature_pub = self.create_publisher(
            Float32, '/avoidance/max_curvature', transient)
        self.obstacle_clearance_pub = self.create_publisher(
            Float32, '/avoidance/min_obstacle_clearance', transient)
        self.curb_clearance_pub = self.create_publisher(
            Float32, '/avoidance/min_curb_clearance', transient)
        self.planning_time_pub = self.create_publisher(
            Float32, '/avoidance/planning_time_ms', transient)
        self.candidate_count_pub = self.create_publisher(
            Int32, '/avoidance/candidate_count', transient)
        self.valid_candidate_count_pub = self.create_publisher(
            Int32, '/avoidance/valid_candidate_count', transient)
        self.failure_pub = self.create_publisher(
            String, '/avoidance/planning_failure_reason', transient)
        self.diagnostics_pub = self.create_publisher(
            String, '/avoidance/planner_diagnostics', transient)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracker = TrackManager(
            float(self.p['association_distance_m']),
            int(self.p['confirmation_frames']), int(self.p['lost_frames']),
            float(self.p['velocity_filter_alpha']),
            float(self.p['dynamic_enter_speed_mps']),
            float(self.p['dynamic_exit_speed_mps']),
            int(self.p['dynamic_confirmation_frames']),
            int(self.p['static_confirmation_frames']))
        self.debounce = ReplanDebounce(int(self.p['confirmation_frames']))
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='local_planner')
        self.future = None
        self.state = 'FOLLOWING_CSV'
        self.reason = ''
        self.state_history = [self.state]
        self.odom = None
        self.route = ()
        self.route_nearest_index = 0
        self.tracks = ()
        self.walls = ()
        self.boundary_source = 'configured_fallback'
        self.wall_hits = 0
        self.unknown = ()
        self.selected_track = None
        self.last_front_scan_time = None
        self.last_rear_scan_time = None
        self.stopped_since = None
        self.planning_started_wall = None
        self.tf_failures = 0
        self.scan_drops = 0
        self.scan_count = 0
        self.performance = {
            'scan_processing_ms': 0.0, 'clustering_ms': 0.0,
            'track_update_ms': 0.0, 'wall_detection_ms': 0.0,
            'candidate_generation_ms': 0.0, 'collision_check_ms': 0.0,
            'total_planning_ms': 0.0, 'cpu_percent': 0.0,
            'effective_scan_hz': 0.0}
        self.last_scan_wall = None
        self.last_cpu_wall = time.perf_counter()
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self.last_cpu_seconds = usage.ru_utime+usage.ru_stime

        self.create_subscription(
            LaserScan, str(self.p['front_scan_topic']), self._front_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.p['rear_scan_topic']), self._rear_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.p['odom_topic']), self._odom, 20)
        self.create_subscription(
            Path, str(self.p['reference_path_topic']), self._reference, transient)
        self.create_timer(1.0/max(1.0, float(self.p['planning_rate_hz'])), self._tick)
        self.replan_pub.publish(Bool(data=False))
        self._publish_status()
        self.get_logger().info(
            'Avoidance planner ready: perception/planning separated; '
            'no /cmd_vel publisher; selected path will not be driven')

    def _validate_parameters(self):
        if float(self.p['wheelbase_m']) <= 0.0:
            raise ValueError('wheelbase_m must be positive')
        if not 0.0 < float(self.p['max_steering_deg']) <= 20.0:
            raise ValueError('max_steering_deg must be in (0, 20]')
        if float(self.p['left_curb_inner_y_m']) <= float(self.p['right_curb_inner_y_m']):
            raise ValueError('curb boundaries are inverted')

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                          1.0-2.0*(q.y*q.y+q.z*q.z))

    def _odom(self, msg):
        self.odom = msg

    def _reference(self, msg):
        route = []
        for stamped in msg.poses:
            pose = stamped.pose
            route.append(Pose2(pose.position.x, pose.position.y,
                               self._yaw(pose.orientation)))
        if len(route) >= 2:
            self.route = tuple(route)

    def _front_scan(self, scan):
        self.last_front_scan_time = self.get_clock().now()
        self._process_scan(scan)

    def _rear_scan(self, scan):
        self.last_rear_scan_time = self.get_clock().now()
        # Validate transform availability and timestamp without spending the
        # front-planner budget on the rear 360-degree field.
        if not self._lookup_transform(scan):
            self.scan_drops += 1

    def _lookup_transform(self, scan):
        if not scan.header.frame_id:
            self.tf_failures += 1
            return None
        try:
            stamp = Time.from_msg(scan.header.stamp)
            return self.tf_buffer.lookup_transform(
                str(self.p['target_frame']), scan.header.frame_id, stamp,
                timeout=RclpyDuration(seconds=float(self.p['tf_lookup_timeout_sec'])))
        except TransformException:
            self.tf_failures += 1
            return None

    def _transform_points(self, points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw(rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        return tuple(ScanPoint(
            point.index,
            translation.x+c*point.x-s*point.y,
            translation.y+s*point.x+c*point.y,
            point.distance) for point in points)

    def _process_scan(self, scan):
        started = time.perf_counter()
        if self.last_scan_wall is not None:
            period = started-self.last_scan_wall
            if period > 1.0e-6:
                self.performance['effective_scan_hz'] = 1.0/period
        self.last_scan_wall = started
        transform = self._lookup_transform(scan)
        if transform is None:
            self.scan_drops += 1
            return
        points = preprocess_scan(
            scan.ranges, scan.angle_min, scan.angle_increment,
            max(scan.range_min, 0.12), min(scan.range_max, 12.0),
            float(self.p['planning_roi_x_min_m']),
            float(self.p['planning_roi_x_max_m']),
            float(self.p['planning_roi_half_width_m']),
            float(self.p['self_x_min_m']), float(self.p['self_x_max_m']),
            float(self.p['self_y_half_width_m']))
        points = self._transform_points(points, transform)
        cluster_started = time.perf_counter()
        groups = cluster_groups(
            points, float(self.p['max_cluster_gap_near_m']),
            float(self.p['max_cluster_gap_far_m']),
            float(self.p['adaptive_gap_distance_m']))
        groups = tuple(group for group in groups
                       if len(group) >= int(self.p['min_cluster_points']))
        self.performance['clustering_ms'] = (time.perf_counter()-cluster_started)*1000.0
        wall_started = time.perf_counter()
        route_heading = self._current_pose().yaw if self.odom else 0.0
        walls, detections, unknown = split_walls_and_objects(
            groups, route_heading, float(self.p['wall_min_length_m']),
            float(self.p['wall_max_residual_m']),
            math.radians(float(self.p['wall_parallel_tolerance_deg'])),
            float(self.p['min_cluster_width_m']))
        self.performance['wall_detection_ms'] = (time.perf_counter()-wall_started)*1000.0
        self.wall_hits = self.wall_hits+1 if walls else 0
        self.walls = tuple(walls) if self.wall_hits >= int(self.p['confirmation_frames']) else ()
        self.unknown = tuple(unknown)
        track_started = time.perf_counter()
        stamp = Time.from_msg(scan.header.stamp).nanoseconds/1e9
        self.tracks = self.tracker.update(detections, stamp)
        self.performance['track_update_ms'] = (time.perf_counter()-track_started)*1000.0
        self.performance['scan_processing_ms'] = (time.perf_counter()-started)*1000.0
        self.scan_count += 1
        self._publish_perception(scan.header.stamp)

    def _current_pose(self):
        pose = self.odom.pose.pose
        return Pose2(pose.position.x, pose.position.y, self._yaw(pose.orientation))

    def _tick(self):
        self._update_cpu_usage()
        self._publish_stop_contract()
        self._watchdog()
        if self.odom is None or len(self.route) < 2:
            self._publish_status()
            return
        if self.state in ('PATH_READY', 'PATH_INFEASIBLE',
                          'DYNAMIC_OBSTACLE_STOP', 'ERROR'):
            self._publish_status()
            return
        if self.state in ('STOPPING', 'STOPPED_FOR_PLANNING',
                          'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
                          'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES'):
            self._advance_planning()
            self._publish_status()
            return
        pose = self._current_pose()
        risks = []
        for track in self.tracks:
            if track.state not in (STATIC_OBSTACLE, DYNAMIC_OBSTACLE):
                continue
            risk = evaluate_track_collision(
                self.route, pose, track, float(self.p['vehicle_length_m']),
                float(self.p['vehicle_width_m']),
                float(self.p['vehicle_center_x_offset_m']),
                float(self.p['obstacle_safety_lateral_m']),
                float(self.p['obstacle_safety_longitudinal_m']),
                float(self.p['obstacle_monitor_distance_m']),
                float(self.p['emergency_stop_distance_m']),
                float(self.p['left_curb_inner_y_m']),
                float(self.p['right_curb_inner_y_m']),
                float(self.p['minimum_obstacle_depth_m']), self.route_nearest_index)
            self.route_nearest_index = max(self.route_nearest_index, risk.nearest_path_index)
            if risk.required:
                risks.append((risk.longitudinal_distance, track, risk))
        if risks:
            distance, track, risk = min(risks, key=lambda item: item[0])
            self.nearest_distance_pub.publish(Float32(data=float(distance)))
            if distance <= float(self.p['replan_trigger_distance_m']) or risk.emergency:
                self.selected_track = track
                if track.state == DYNAMIC_OBSTACLE:
                    self.debounce.latched = True
                    self._set_state('DYNAMIC_OBSTACLE_STOP', 'dynamic obstacle intersects CSV path')
                    self.replan_pub.publish(Bool(data=True))
                elif self.debounce.update(True, track.track_id):
                    self._set_state('OBSTACLE_CONFIRMED', f'track={track.track_id}')
                    self._set_state('REPLAN_REQUIRED', f'track={track.track_id}')
                    self.replan_pub.publish(Bool(data=True))
                    self._set_state('STOPPING', 'waiting for odom-confirmed stop')
                else:
                    self._set_state('OBSTACLE_CANDIDATE', f'track={track.track_id}')
        elif self.state == 'OBSTACLE_CANDIDATE':
            self.debounce.update(False)
            self._set_state('FOLLOWING_CSV', '')
        self._publish_status()

    def _advance_planning(self):
        if self.state == 'STOPPING':
            twist = self.odom.twist.twist
            stopped = (abs(twist.linear.x) <= float(self.p['stopped_linear_speed_mps']) and
                       abs(twist.angular.z) <= float(self.p['stopped_angular_speed_rps']))
            now = self.get_clock().now()
            if stopped:
                self.stopped_since = self.stopped_since or now
                elapsed = (now-self.stopped_since).nanoseconds/1e9
                if elapsed >= float(self.p['stopped_confirmation_sec']):
                    self._set_state('STOPPED_FOR_PLANNING', 'vehicle stationary')
            else:
                self.stopped_since = None
            return
        if self.state == 'STOPPED_FOR_PLANNING':
            self._set_state('DETECTING_BOUNDARIES',
                            f'{len(self.walls)} confirmed wall segments')
            return
        if self.state == 'DETECTING_BOUNDARIES':
            if self.selected_track is None:
                self._set_state('ERROR', 'selected obstacle disappeared')
                return
            self._set_state('BUILDING_CORRIDOR', '')
            return
        if self.state == 'BUILDING_CORRIDOR':
            self._set_state('GENERATING_CANDIDATES', '')
            self.planning_started_wall = time.perf_counter()
            obstacle = track_box(
                self.selected_track, float(self.p['minimum_obstacle_depth_m']))
            left_boundary, right_boundary = self._boundary_values()
            args = (
                self.route, self._current_pose(), obstacle,
                left_boundary, right_boundary)
            kwargs = {
                'vehicle_length': float(self.p['vehicle_length_m']),
                'vehicle_width': float(self.p['vehicle_width_m']),
                'center_offset': float(self.p['vehicle_center_x_offset_m']),
                'wheelbase': float(self.p['wheelbase_m']),
                'max_steering_rad': math.radians(float(self.p['max_steering_deg'])),
                'obstacle_safety_lateral': float(self.p['obstacle_safety_lateral_m']),
                'obstacle_safety_longitudinal': float(self.p['obstacle_safety_longitudinal_m']),
                'curb_safety': float(self.p['curb_safety_m']),
                'sample_interval': float(self.p['path_sample_interval_m']),
                'target_fractions': tuple(self.p['corridor_candidate_fractions']),
                'return_lengths': tuple(self.p['return_transition_lengths_m']),
                'minimum_index': self.route_nearest_index,
                'collision_check_interval': float(self.p['collision_check_interval_m']),
            }
            self.future = self.worker.submit(plan_candidates, *args, **kwargs)
            return
        if self.state == 'GENERATING_CANDIDATES':
            if self.future is not None and self.future.done():
                self._set_state('VALIDATING_CANDIDATES', '')
            return
        if self.state == 'VALIDATING_CANDIDATES':
            try:
                result = self.future.result()
            except Exception as exc:  # safety boundary around worker
                self._set_state('ERROR', f'planner exception: {exc}')
                return
            elapsed_ms = (time.perf_counter()-self.planning_started_wall)*1000.0
            self.performance['candidate_generation_ms'] = result.generation_time_ms
            self.performance['collision_check_ms'] = result.validation_time_ms
            self.performance['total_planning_ms'] = elapsed_ms
            self._publish_plan(result, elapsed_ms)
            if result.selected is None:
                self._set_state('PATH_INFEASIBLE', 'no collision-free <=20 deg candidate')
            else:
                self._set_state('PATH_READY', f'candidate={result.selected.candidate_id}')

    def _watchdog(self):
        if self.last_front_scan_time is None:
            return
        age = (self.get_clock().now()-self.last_front_scan_time).nanoseconds/1e9
        if age > float(self.p['scan_timeout_sec']) and self.state not in (
                'PATH_READY', 'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP'):
            self.scan_drops += 1
            if self.debounce.latched:
                self._set_state('ERROR', f'front scan timeout {age:.3f}s')

    def _update_cpu_usage(self):
        now = time.perf_counter()
        usage = resource.getrusage(resource.RUSAGE_SELF)
        cpu = usage.ru_utime+usage.ru_stime
        wall_delta = now-self.last_cpu_wall
        if wall_delta >= 0.5:
            self.performance['cpu_percent'] = max(
                0.0, 100.0*(cpu-self.last_cpu_seconds)/wall_delta)
            self.last_cpu_wall, self.last_cpu_seconds = now, cpu

    def _boundary_values(self):
        configured_left = float(self.p['left_curb_inner_y_m'])
        configured_right = float(self.p['right_curb_inner_y_m'])
        midpoints = [0.5*(wall.y1+wall.y2) for wall in self.walls]
        left = [value for value in midpoints if value > 0.0]
        right = [value for value in midpoints if value < 0.0]
        if left and right:
            self.boundary_source = 'lidar_confirmed_walls'
            return (min(left, key=lambda value: abs(value-configured_left)),
                    min(right, key=lambda value: abs(value-configured_right)))
        self.boundary_source = 'configured_fallback'
        return configured_left, configured_right

    def _set_state(self, state, reason):
        if state not in PLANNER_STATES:
            raise ValueError(f'unknown planner state {state}')
        changed = state != self.state or reason != self.reason
        self.state, self.reason = state, reason
        if changed:
            self.state_history.append(state)
            self.get_logger().info(f'{state}: {reason}')
            self._publish_status()

    def _publish_stop_contract(self):
        active = self.debounce.latched or self.state in (
            'STOPPING', 'STOPPED_FOR_PLANNING', 'DETECTING_BOUNDARIES',
            'BUILDING_CORRIDOR', 'GENERATING_CANDIDATES',
            'VALIDATING_CANDIDATES', 'PATH_READY', 'PATH_INFEASIBLE',
            'DYNAMIC_OBSTACLE_STOP', 'ERROR')
        if active:
            self.replan_pub.publish(Bool(data=True))
            self.lidar_drive_pub.publish(Float32(data=0.0))
            self.lidar_wheel_pub.publish(Int32(data=0))

    def _marker(self, namespace, marker_id, marker_type, stamp=None):
        marker = Marker()
        marker.header.frame_id = str(self.p['target_frame'])
        marker.header.stamp = stamp or self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = (
            namespace, marker_id, marker_type, Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        lifetime = float(self.p['marker_lifetime_sec'])
        marker.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1.0)*1e9))
        return marker

    @staticmethod
    def _clear_array():
        marker = Marker(); marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])

    def _publish_perception(self, stamp):
        static_markers, dynamic_markers = self._clear_array(), self._clear_array()
        for track in self.tracks:
            if track.state not in (STATIC_OBSTACLE, DYNAMIC_OBSTACLE):
                continue
            marker = self._marker('tracked_obstacles', track.track_id, Marker.CUBE, stamp)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = track.x, track.y, 0.10
            marker.scale.x = max(float(self.p['minimum_obstacle_depth_m']), track.max_x-track.min_x)
            marker.scale.y = max(0.08, track.max_y-track.min_y)
            marker.scale.z = 0.20
            if track.state == STATIC_OBSTACLE:
                marker.color.r, marker.color.a = 1.0, 0.90
                static_markers.markers.append(marker)
            else:
                marker.color.r, marker.color.g, marker.color.a = 1.0, 0.45, 0.90
                dynamic_markers.markers.append(marker)
                arrow = self._marker('dynamic_velocity', track.track_id, Marker.ARROW, stamp)
                arrow.points = [Point(x=track.x, y=track.y),
                                Point(x=track.x+track.vx, y=track.y+track.vy)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.04, 0.08, 0.10
                arrow.color.r, arrow.color.g, arrow.color.a = 1.0, 0.45, 1.0
                dynamic_markers.markers.append(arrow)
                trail = self._marker('dynamic_trail', track.track_id, Marker.LINE_STRIP, stamp)
                trail.points = [Point(x=x, y=y, z=0.03) for _, x, y in track.history]
                trail.scale.x = 0.035
                trail.color.r, trail.color.g, trail.color.a = 1.0, 0.45, 0.8
                dynamic_markers.markers.append(trail)
        self.static_pub.publish(static_markers)
        self.dynamic_pub.publish(dynamic_markers)
        wall_markers = self._clear_array()
        for marker_id, wall in enumerate(self.walls):
            marker = self._marker('walls', marker_id, Marker.LINE_STRIP, stamp)
            marker.points = [Point(x=wall.x1, y=wall.y1, z=0.05),
                             Point(x=wall.x2, y=wall.y2, z=0.05)]
            marker.scale.x = 0.06
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            wall_markers.markers.append(marker)
        self.walls_pub.publish(wall_markers)
        unknown_markers = self._clear_array()
        for marker_id, item in enumerate(self.unknown):
            marker = self._marker('unknown', marker_id, Marker.CUBE, stamp)
            marker.pose.position.x, marker.pose.position.y = item.x, item.y
            marker.scale.x = max(0.08, item.depth)
            marker.scale.y, marker.scale.z = max(0.08, item.width), 0.15
            marker.color.r, marker.color.b, marker.color.a = 0.65, 1.0, 0.8
            unknown_markers.markers.append(marker)
        self.unknown_pub.publish(unknown_markers)

    def _publish_plan(self, result, elapsed_ms):
        candidates = self._clear_array()
        valid_count = 0
        for candidate in result.candidates:
            marker = self._marker('candidate_paths', candidate.candidate_id, Marker.LINE_STRIP)
            marker.points = [Point(x=pose.x, y=pose.y, z=0.04) for pose in candidate.path]
            marker.scale.x = 0.035
            if candidate.valid:
                valid_count += 1
                marker.color.r, marker.color.g, marker.color.a = 1.0, 0.85, 0.55
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.75, 0.15, 0.15, 0.35
            candidates.markers.append(marker)
        self.candidate_pub.publish(candidates)
        self.candidate_count_pub.publish(Int32(data=len(result.candidates)))
        self.valid_candidate_count_pub.publish(Int32(data=valid_count))
        self.planning_time_pub.publish(Float32(data=float(elapsed_ms)))
        obstacle = track_box(self.selected_track, float(self.p['minimum_obstacle_depth_m']))
        corridor = self._marker('safe_corridor', 0, Marker.CUBE)
        corridor.pose.position.x = 0.5*(obstacle.min_x+obstacle.max_x)
        corridor.pose.position.y = 0.5*(result.corridor.lower_center_d+result.corridor.upper_center_d)
        corridor.pose.position.z = 0.025
        corridor.scale.x = obstacle.max_x-obstacle.min_x+1.6
        corridor.scale.y = max(0.001, result.corridor.width)
        corridor.scale.z = 0.05
        corridor.color.g, corridor.color.b, corridor.color.a = 0.9, 0.8, 0.35
        self.corridor_pub.publish(corridor)
        planning_points = self._clear_array()
        colors = ((1.0, 1.0, 0.0), (0.0, 1.0, 0.9), (0.7, 0.0, 1.0))
        for marker_id, (pose, color) in enumerate(zip(
                (result.start_pose, result.pass_pose, result.return_pose), colors)):
            if pose is None:
                continue
            marker = self._marker('planning_points', marker_id, Marker.SPHERE)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose.x, pose.y, 0.12
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color.r, marker.color.g, marker.color.b = color
            planning_points.markers.append(marker)
        footprint = self._marker('vehicle_footprint', 3, Marker.CUBE)
        footprint.pose.position.x = result.start_pose.x
        footprint.pose.position.y = result.start_pose.y
        footprint.pose.position.z = 0.03
        footprint.pose.orientation.z = math.sin(result.start_pose.yaw/2.0)
        footprint.pose.orientation.w = math.cos(result.start_pose.yaw/2.0)
        footprint.scale.x = float(self.p['vehicle_length_m'])
        footprint.scale.y = float(self.p['vehicle_width_m'])
        footprint.scale.z = 0.06
        footprint.color.g, footprint.color.b, footprint.color.a = 0.9, 0.8, 0.30
        planning_points.markers.append(footprint)
        inflated = obstacle.inflated(
            float(self.p['obstacle_safety_longitudinal_m']),
            float(self.p['obstacle_safety_lateral_m']))
        inflated_marker = self._marker('inflated_obstacle', 4, Marker.CUBE)
        inflated_marker.pose.position.x = 0.5*(inflated.min_x+inflated.max_x)
        inflated_marker.pose.position.y = 0.5*(inflated.min_y+inflated.max_y)
        inflated_marker.pose.position.z = 0.04
        inflated_marker.scale.x = inflated.max_x-inflated.min_x
        inflated_marker.scale.y = inflated.max_y-inflated.min_y
        inflated_marker.scale.z = 0.08
        inflated_marker.color.r, inflated_marker.color.a = 1.0, 0.25
        planning_points.markers.append(inflated_marker)
        self.planning_points_pub.publish(planning_points)
        selected_path = Path()
        selected_path.header.frame_id = str(self.p['target_frame'])
        selected_path.header.stamp = self.get_clock().now().to_msg()
        if result.selected:
            for pose in result.selected.path:
                stamped = PoseStamped(); stamped.header = selected_path.header
                stamped.pose.position.x, stamped.pose.position.y = pose.x, pose.y
                stamped.pose.orientation.z = math.sin(pose.yaw/2.0)
                stamped.pose.orientation.w = math.cos(pose.yaw/2.0)
                selected_path.poses.append(stamped)
            selected = result.selected
            self.max_steering_pub.publish(Float32(data=math.degrees(selected.max_steering_rad)))
            self.max_curvature_pub.publish(Float32(data=selected.max_curvature))
            self.obstacle_clearance_pub.publish(Float32(data=selected.obstacle_clearance))
            self.curb_clearance_pub.publish(Float32(data=selected.curb_clearance))
            self.failure_pub.publish(String(data=''))
        else:
            self.max_steering_pub.publish(Float32(data=math.nan))
            self.max_curvature_pub.publish(Float32(data=math.nan))
            self.failure_pub.publish(String(data='NO_VALID_CANDIDATE'))
        self.selected_path_pub.publish(selected_path)
        self.selected_track_pub.publish(Int32(data=self.selected_track.track_id))

    def _publish_status(self):
        payload = {
            'state': self.state, 'reason': self.reason,
            'selected_track_id': self.selected_track.track_id if self.selected_track else -1,
            'track_count': len(self.tracks), 'wall_count': len(self.walls),
            'scan_count': self.scan_count, 'scan_drops': self.scan_drops,
            'tf_failures': self.tf_failures, 'performance_ms': self.performance,
            'boundary_source': self.boundary_source,
            'state_history': self.state_history[-12:],
            'drives_selected_path': False,
        }
        text = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=text))
        self.diagnostics_pub.publish(String(data=text))
        marker = self._marker('planner_status', 0, Marker.TEXT_VIEW_FACING)
        if self.odom:
            pose = self.odom.pose.pose
            marker.pose.position.x, marker.pose.position.y = pose.position.x, pose.position.y
        marker.pose.position.z = 1.25
        marker.scale.z = 0.20
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'{self.state}\n{self.reason}'
        self.status_marker_pub.publish(marker)

    def destroy_node(self):
        self.replan_pub.publish(Bool(data=True if self.debounce.latched else False))
        self.lidar_drive_pub.publish(Float32(data=0.0))
        self.lidar_wheel_pub.publish(Int32(data=0))
        self.worker.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = AvoidanceCoordinator()
    try:
        while rclpy.ok():
            # Perception callbacks are bounded; heavy candidate generation is
            # already isolated in ``self.worker`` so a busy multi-threaded ROS
            # executor is unnecessary here.
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
