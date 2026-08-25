"""ROS coordinator for perception, safe stop, and stopped local planning.

This node never publishes vehicle commands and never drives a selected path.
It publishes a latched stop/replan contract; the route follower requests
commands through the independent LiDAR safety gate.
"""

from concurrent.futures import ThreadPoolExecutor
from collections import deque
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
from .local_planner import (interpolate_route, plan_candidates, project_route,
                            route_lengths)
from .perception import (
    DYNAMIC_OBSTACLE, STATIC_OBSTACLE, Detection, LineFeature, TrackManager,
    ScanPoint, cluster_groups, split_walls_and_objects, preprocess_scan)


PLANNER_STATES = (
    'FOLLOWING_CSV', 'OBSTACLE_CANDIDATE', 'OBSTACLE_CONFIRMED',
    'REPLAN_REQUIRED', 'STOPPING', 'STOPPED_FOR_PLANNING',
    'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
    'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES', 'PATH_READY',
    'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP', 'TIME_RESET_STOP', 'ERROR')


class AvoidanceCoordinator(Node):
    def __init__(self):
        super().__init__('avoidance_coordinator')
        defaults = {
            'front_scan_topic': '/scan_front', 'rear_scan_topic': '/scan_rear',
            'rear_lidar_required': False,
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
            # Candidate validation is deliberately synchronous and can take a
            # few seconds.  TF callbacks queued during that computation must
            # be allowed to catch up before exact-stamp scans are classified
            # as failed.
            'tf_lookup_timeout_sec': 5.0,
            'association_distance_m': 0.45,
            'velocity_filter_alpha': 0.35,
            'dynamic_enter_speed_mps': 0.15,
            'dynamic_exit_speed_mps': 0.08,
            'dynamic_confirmation_frames': 3,
            'dynamic_min_observations': 3,
            'static_confirmation_frames': 5,
            'fixed_environment_mode': False,
            'fixed_obstacle_s_m': [0.0], 'fixed_obstacle_d_m': [0.0],
            'fixed_obstacle_match_s_m': 1.50,
            'fixed_obstacle_match_d_m': 0.70,
            'fixed_obstacle_length_m': 1.30,
            'fixed_obstacle_width_m': 0.78,
            'wall_min_length_m': 1.50, 'wall_max_residual_m': 0.05,
            'wall_parallel_tolerance_deg': 15.0,
            'vehicle_length_m': 1.30, 'vehicle_width_m': 0.78,
            'vehicle_center_x_offset_m': 0.0, 'wheelbase_m': 0.77,
            'max_steering_deg': 25.0,
            'obstacle_safety_lateral_m': 0.20,
            'obstacle_safety_longitudinal_m': 0.15,
            'minimum_obstacle_depth_m': 0.65,
            'minimum_obstacle_width_m': 0.39,
            'expected_obstacle_lateral_center_m': 0.78,
            'curb_safety_m': 0.08, 'left_curb_inner_y_m': 1.095,
            'right_curb_inner_y_m': -1.095,
            'obstacle_monitor_distance_m': 5.0,
            'replan_trigger_distance_m': 2.0,
            'front_lidar_x_offset_m': 0.65,
            'emergency_stop_distance_m': 0.5,
            'stopped_linear_speed_mps': 0.03,
            'stopped_angular_speed_rps': 0.03,
            'stopped_confirmation_sec': 0.30,
            'path_sample_interval_m': 0.05,
            'collision_check_interval_m': 0.02,
            'return_transition_lengths_m': [2.0, 2.5, 3.0],
            'rejoin_straight_extension_m': 1.0,
            'corridor_candidate_fractions': [0.25, 0.50, 0.75, 0.80, 0.85, 0.90],
            'adaptive_candidate_sampling': True,
            'lateral_target_samples': 7,
            'marker_lifetime_sec': 2.0, 'planning_rate_hz': 20.0,
            'clock_forward_jump_threshold_s': 2.0,
            'clock_reset_threshold_s': 1.0,
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
        self.nearest_distance_pub = self.create_publisher(
            Float32, '/avoidance/nearest_obstacle_distance', transient)
        self.lidar_surface_distance_pub = self.create_publisher(
            Float32, '/avoidance/lidar_surface_distance', transient)
        self.vehicle_front_distance_pub = self.create_publisher(
            Float32, '/avoidance/vehicle_front_surface_distance', transient)
        self.obstacle_center_distance_pub = self.create_publisher(
            Float32, '/avoidance/obstacle_center_distance', transient)
        self.collision_point_distance_pub = self.create_publisher(
            Float32, '/avoidance/collision_point_distance', transient)
        self.replan_threshold_pub = self.create_publisher(
            Float32, '/avoidance/replan_threshold', transient)
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
        self.obstacle_status_pub = self.create_publisher(
            MarkerArray, '/avoidance/obstacle_status', transient)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracker = TrackManager(
            float(self.p['association_distance_m']),
            int(self.p['confirmation_frames']), int(self.p['lost_frames']),
            float(self.p['velocity_filter_alpha']),
            float(self.p['dynamic_enter_speed_mps']),
            float(self.p['dynamic_exit_speed_mps']),
            int(self.p['dynamic_confirmation_frames']),
            int(self.p['static_confirmation_frames']),
            int(self.p['dynamic_min_observations']))
        self.debounce = ReplanDebounce(
            1 if bool(self.p['fixed_environment_mode']) else
            int(self.p['confirmation_frames']))
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='local_planner')
        self.future = None
        self.state = 'FOLLOWING_CSV'
        self.reason = ''
        self.state_history = [self.state]
        self.odom = None
        self.odom_receive_time = None
        self.last_odom_stamp_ns = None
        self.route = ()
        self.route_nearest_index = 0
        self.tracks = ()
        self.fixed_matched_track_ids = set()
        self.walls = ()
        self.boundary_source = 'configured_fallback'
        self.wall_hits = 0
        self.unknown = ()
        self.selected_track = None
        self.avoidance_started = False
        self.passed_track_ids = set()
        self.path_relevant_track_ids = set()
        self.passed_obstacle_s = []
        self.last_front_scan_time = None
        self.last_rear_scan_time = None
        self.last_front_scan_stamp_ns = None
        self.last_rear_scan_stamp_ns = None
        self.pending_scans = deque()
        self.tf_ready_frames = set()
        self.front_tf_ready = False
        self.rear_tf_ready = False
        self.tf_ready = False
        self.startup_tf_drop_count = 0
        self.runtime_tf_drop_count = 0
        self.future_extrapolation_count = 0
        self.past_extrapolation_count = 0
        self.missing_frame_count = 0
        self.last_tf_error = ''
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
        self.last_plan_summary = {}
        self.last_track_decisions = []
        self.cluster_diagnostics = []
        self.track_decision_states = {}
        self.last_scan_wall = None
        self.last_scan_stamp = None
        self.last_tick_time = None
        self.time_reset_resume_state = None
        self.time_reset_is_large = False
        self.selected_path_msg = None
        self.last_cpu_wall = time.perf_counter()
        usage = resource.getrusage(resource.RUSAGE_SELF)
        self.last_cpu_seconds = usage.ru_utime+usage.ru_stime

        self.create_subscription(
            LaserScan, str(self.p['front_scan_topic']), self._front_scan,
            qos_profile_sensor_data)
        if bool(self.p['rear_lidar_required']):
            self.create_subscription(
                LaserScan, str(self.p['rear_scan_topic']), self._rear_scan,
                qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.p['odom_topic']), self._odom, 20)
        self.create_subscription(
            Path, str(self.p['reference_path_topic']), self._reference, transient)
        self.create_subscription(
            String, '/avoidance/control_source', self._control_source, transient)
        self.create_timer(1.0/max(1.0, float(self.p['planning_rate_hz'])), self._tick)
        self.replan_pub.publish(Bool(data=False))
        self.replan_threshold_pub.publish(
            Float32(data=float(self.p['replan_trigger_distance_m'])))
        self._publish_status()
        self.get_logger().info(
            'Avoidance planner waiting for exact-timestamp front LiDAR TF; '
            'perception/planning and vehicle safety are separated')

    def _validate_parameters(self):
        if float(self.p['wheelbase_m']) <= 0.0:
            raise ValueError('wheelbase_m must be positive')
        if not 0.0 < float(self.p['max_steering_deg']) <= 25.0:
            raise ValueError('max_steering_deg must be in (0, 25]')
        if float(self.p['left_curb_inner_y_m']) <= float(self.p['right_curb_inner_y_m']):
            raise ValueError('curb boundaries are inverted')

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                          1.0-2.0*(q.y*q.y+q.z*q.z))

    def _is_passed_obstacle_face(self, route_s):
        return any(abs(route_s-passed_s) <= 1.5
                   for passed_s in getattr(self, 'passed_obstacle_s', ()))

    def _is_beyond_route_goal(self, track):
        """Return true when an entire obstacle is beyond the drivable CSV."""
        if len(self.route) < 2:
            return False
        goal, previous = self.route[-1], self.route[-2]
        yaw = math.atan2(goal.y-previous.y, goal.x-previous.x)
        forward = ((track.x-goal.x)*math.cos(yaw) +
                   (track.y-goal.y)*math.sin(yaw))
        required = (float(self.p['vehicle_length_m'])/2.0 +
                    float(self.p['minimum_obstacle_depth_m'])/2.0 +
                    float(self.p['obstacle_safety_longitudinal_m']))
        return forward > required

    def _odom(self, msg):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self._stamp_regressed('odom', stamp_ns, self.last_odom_stamp_ns):
            return
        self.last_odom_stamp_ns = stamp_ns
        self.odom = msg
        self.odom_receive_time = self.get_clock().now()

    def _reference(self, msg):
        route = []
        for stamped in msg.poses:
            pose = stamped.pose
            route.append(Pose2(pose.position.x, pose.position.y,
                               self._yaw(pose.orientation)))
        if len(route) >= 2:
            self.route = tuple(route)

    def _control_source(self, msg):
        if msg.data == 'LIDAR':
            self.avoidance_started = True
        elif msg.data == 'GPS' and self.avoidance_started:
            completed_track = self.selected_track.track_id if self.selected_track else -1
            if completed_track >= 0:
                self.passed_track_ids.add(completed_track)
                route = getattr(self, 'route', ())
                if len(route) >= 2 and hasattr(self.selected_track, 'x'):
                    completed = project_route(
                        route, self.selected_track.x, self.selected_track.y, 0)
                    if not hasattr(self, 'passed_obstacle_s'):
                        self.passed_obstacle_s = []
                    self.passed_obstacle_s.append(completed.s)
                # A box may be observed as separate front/side face tracks.
                # Mark colocated faces together so one physical obstacle is
                # never planned twice after CSV rejoin.
                for track in getattr(self, 'tracks', ()):
                    if math.hypot(track.x-self.selected_track.x,
                                  track.y-self.selected_track.y) <= 1.0:
                        self.passed_track_ids.add(track.track_id)
                self.get_logger().info('PASSED ' + json.dumps({
                    'track_id': completed_track,
                    'passed_track_ids': sorted(self.passed_track_ids),
                    'passed_obstacle_s': self.passed_obstacle_s[-1]
                    if self.passed_obstacle_s else None,
                }, sort_keys=True))
            self.debounce = ReplanDebounce(
                1 if bool(self.p['fixed_environment_mode']) else
                int(self.p['confirmation_frames']))
            # Associations accumulated while the vehicle follows a large
            # avoidance arc mix different visible faces of segmented curbs.
            # They are not valid motion evidence after CSV rejoin.  Start a
            # fresh perception epoch so the next physical obstacle must be
            # confirmed from current stationary-world observations.
            self.tracker.reset_epoch()
            self.tracks = ()
            self.walls = ()
            self.unknown = ()
            self.wall_hits = 0
            self.last_track_decisions = []
            self.selected_track = None
            self.future = None
            self.avoidance_started = False
            if hasattr(self, '_clear_completed_plan'):
                self._clear_completed_plan()
            self.replan_pub.publish(Bool(data=False))
            self._set_state('FOLLOWING_CSV',
                            f'avoidance complete for track={completed_track}')

    def _clear_completed_plan(self):
        self.candidate_pub.publish(self._clear_array())
        self.planning_points_pub.publish(self._clear_array())
        empty_path = Path()
        empty_path.header.frame_id = str(self.p['target_frame'])
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.selected_path_pub.publish(empty_path)
        corridor = self._marker('safe_corridor', 0, Marker.CUBE)
        corridor.action = Marker.DELETE
        self.corridor_pub.publish(corridor)
        self.selected_path_msg = None

    def _front_scan(self, scan):
        stamp_ns = Time.from_msg(scan.header.stamp).nanoseconds
        if self._stamp_regressed(
                'scan_front', stamp_ns, self.last_front_scan_stamp_ns):
            return
        self.last_front_scan_stamp_ns = stamp_ns
        self.last_front_scan_time = self.get_clock().now()
        self._queue_scan(scan, True)

    def _rear_scan(self, scan):
        stamp_ns = Time.from_msg(scan.header.stamp).nanoseconds
        if self._stamp_regressed(
                'scan_rear', stamp_ns, self.last_rear_scan_stamp_ns):
            return
        self.last_rear_scan_stamp_ns = stamp_ns
        self.last_rear_scan_time = self.get_clock().now()
        self._queue_scan(scan, False)

    def _queue_scan(self, scan, is_front):
        if len(self.pending_scans) >= 100:
            self.pending_scans.popleft()
            self._record_tf_drop('pending scan queue overflow')
        self.pending_scans.append((scan, is_front, time.perf_counter()))

    def _lookup_transform(self, scan):
        if not scan.header.frame_id:
            return None, 'missing', 'scan frame_id is empty'
        try:
            stamp = Time.from_msg(scan.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                str(self.p['target_frame']), scan.header.frame_id, stamp,
                timeout=RclpyDuration(seconds=0.0))
            return transform, '', ''
        except TransformException as exc:
            detail = str(exc)
            lowered = detail.lower()
            if 'future' in lowered:
                category = 'future'
            elif 'past' in lowered:
                category = 'past'
            else:
                category = 'missing'
            return None, category, detail

    def _record_tf_drop(self, detail, category='missing'):
        self.scan_drops += 1
        self.tf_failures += 1
        self.last_tf_error = detail
        if self.tf_ready:
            self.runtime_tf_drop_count += 1
        else:
            self.startup_tf_drop_count += 1
        if category == 'future':
            self.future_extrapolation_count += 1
        elif category == 'past':
            self.past_extrapolation_count += 1
        else:
            self.missing_frame_count += 1

    def _drain_scan_queue(self):
        if not self.pending_scans:
            return
        remaining = deque()
        timeout = float(self.p['tf_lookup_timeout_sec'])
        now = time.perf_counter()
        while self.pending_scans:
            scan, is_front, received = self.pending_scans.popleft()
            transform, category, detail = self._lookup_transform(scan)
            if transform is not None:
                self.tf_ready_frames.add(scan.header.frame_id)
                if is_front:
                    self.front_tf_ready = True
                    self._process_scan(scan, transform)
                else:
                    self.rear_tf_ready = True
                self.tf_ready = (
                    self.front_tf_ready and
                    (not bool(self.p['rear_lidar_required']) or
                     self.rear_tf_ready))
                continue
            if now-received < timeout:
                remaining.append((scan, is_front, received))
            else:
                self._record_tf_drop(detail, category)
        self.pending_scans = remaining

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

    def _process_scan(self, scan, transform):
        started = time.perf_counter()
        stamp_seconds = Time.from_msg(scan.header.stamp).nanoseconds/1e9
        if self.last_scan_stamp is not None:
            period = stamp_seconds-self.last_scan_stamp
            if period > 1.0e-6:
                self.performance['effective_scan_hz'] = 1.0/period
        self.last_scan_wall = started
        self.last_scan_stamp = stamp_seconds
        transform_started = time.perf_counter()
        points = preprocess_scan(
            scan.ranges, scan.angle_min, scan.angle_increment,
            max(scan.range_min, 0.12), min(scan.range_max, 12.0),
            float(self.p['planning_roi_x_min_m']),
            float(self.p['planning_roi_x_max_m']),
            float(self.p['planning_roi_half_width_m']),
            float(self.p['self_x_min_m']), float(self.p['self_x_max_m']),
            float(self.p['self_y_half_width_m']))
        points = self._transform_points(points, transform)
        self.performance['tf_transform_ms'] = (
            time.perf_counter()-transform_started)*1000.0
        cluster_started = time.perf_counter()
        groups = cluster_groups(
            points, float(self.p['max_cluster_gap_near_m']),
            float(self.p['max_cluster_gap_far_m']),
            float(self.p['adaptive_gap_distance_m']))
        groups = tuple(group for group in groups
                       if len(group) >= int(self.p['min_cluster_points']))
        self.performance['clustering_ms'] = (time.perf_counter()-cluster_started)*1000.0
        wall_started = time.perf_counter()
        curved_walls = ()
        route_heading = self._current_pose().yaw if self.odom else 0.0
        walls, detections, unknown = split_walls_and_objects(
            groups, route_heading, float(self.p['wall_min_length_m']),
            float(self.p['wall_max_residual_m']),
            math.radians(float(self.p['wall_parallel_tolerance_deg'])),
            float(self.p['min_cluster_width_m']))
        walls = tuple(curved_walls)+tuple(walls)
        walls, detections = self._classify_curved_boundaries(walls, detections)
        if self.p['fixed_environment_mode']:
            lengths = route_lengths(self.route)
            mapped = []
            box_length = float(self.p['fixed_obstacle_length_m'])
            box_width = float(self.p['fixed_obstacle_width_m'])
            for fixed_s, fixed_d in zip(
                    self.p['fixed_obstacle_s_m'],
                    self.p['fixed_obstacle_d_m']):
                x, y, yaw = interpolate_route(self.route, lengths, fixed_s)
                x -= fixed_d*math.sin(yaw)
                y += fixed_d*math.cos(yaw)
                half_x = 0.5*(box_length*abs(math.cos(yaw))+
                              box_width*abs(math.sin(yaw)))
                half_y = 0.5*(box_length*abs(math.sin(yaw))+
                              box_width*abs(math.cos(yaw)))
                mapped.append(Detection(
                    x, y, x-half_x, x+half_x, y-half_y, y+half_y,
                    100, box_width, box_length))
            # The fixed SDF is the obstacle source of truth.  LiDAR returns
            # above still establish exact TF readiness and observe boundaries,
            # while transient visible-face clusters cannot replace a box.
            detections = tuple(mapped)
        self.performance['wall_detection_ms'] = (time.perf_counter()-wall_started)*1000.0
        self.wall_hits = self.wall_hits+1 if walls else 0
        self.walls = tuple(walls) if self.wall_hits >= int(self.p['confirmation_frames']) else ()
        self.unknown = tuple(unknown)
        track_started = time.perf_counter()
        updated_tracks = self.tracker.update(detections, stamp_seconds)
        # In a world whose collision geometry is contractually all static, a
        # DYNAMIC result can only be apparent motion: for example, association
        # walking along consecutive tessellated curb faces as the ego vehicle
        # rounds a bend.  Do not promote that surface alias into a collision
        # track.  The default remains unchanged for worlds with moving actors.
        if self.p['fixed_environment_mode']:
            fixed_tracks = []
            expected = tuple(zip(
                self.p['fixed_obstacle_s_m'], self.p['fixed_obstacle_d_m']))
            for track in updated_tracks:
                if track.state == DYNAMIC_OBSTACLE:
                    projection = project_route(
                        self.route, track.x, track.y, 0)
                    matched = any(
                        abs(projection.s-fs) <=
                        float(self.p['fixed_obstacle_match_s_m']) and
                        abs(projection.d-fd) <=
                        float(self.p['fixed_obstacle_match_d_m'])
                        for fs, fd in expected)
                    if matched:
                        self.fixed_matched_track_ids.add(track.track_id)
                    if (not matched and
                            track.track_id not in self.fixed_matched_track_ids):
                        continue
                    # The map contract proves this actor is fixed; the
                    # measured velocity is only visible-face association.
                    track.state = STATIC_OBSTACLE
                    track.vx = track.vy = track.speed = 0.0
                    track.dynamic_count = 0
                fixed_tracks.append(track)
            self.tracks = tuple(fixed_tracks)
        else:
            self.tracks = updated_tracks
        self.performance['track_update_ms'] = (time.perf_counter()-track_started)*1000.0
        self.performance['scan_processing_ms'] = (time.perf_counter()-started)*1000.0
        self.scan_count += 1
        self._publish_perception(scan.header.stamp)

    def _classify_curved_boundaries(self, walls, detections):
        """Keep curved curb returns out of obstacle tracking using route d."""
        if len(self.route) < 2:
            return tuple(walls), tuple(detections)
        kept = []
        curved_walls = list(walls)
        diagnostics = []
        lengths = route_lengths(self.route)
        for detection in detections:
            projection = project_route(
                self.route, detection.x, detection.y, self.route_nearest_index)
            raw_points = detection.points
            if len(raw_points) > 9:
                last = len(raw_points)-1
                raw_points = tuple(
                    raw_points[round(sample*last/8)] for sample in range(9))
            raw_projections = [
                # A tight facility bend can leave a rearward curb visible in
                # the front ROI after rejoin.  Forcing that static map point
                # onto a future-only segment moves its Frenet d into the lane.
                # Boundary classification is map-global; collision risk below
                # remains progressive from route_nearest_index.
                project_route(self.route, point.x, point.y, 0)
                for point in raw_points]
            curb_band = 0.16
            left_limit = float(self.p['left_curb_inner_y_m'])-curb_band
            right_limit = float(self.p['right_curb_inner_y_m'])+curb_band
            curb_points = sum(
                item.d >= left_limit or item.d <= right_limit
                for item in raw_projections)
            raw_curb = (not self.p['fixed_environment_mode'] and
                        detection.width < 0.65 and raw_projections and
                        curb_points >= math.ceil(0.80*len(raw_projections)))
            diagnostics.append({
                'center_x': detection.x, 'center_y': detection.y,
                'center_s': projection.s, 'center_d': projection.d,
                'width_m': detection.width,
                'raw_d_min': min((item.d for item in raw_projections),
                                 default=None),
                'raw_d_max': max((item.d for item in raw_projections),
                                 default=None),
                'raw_curb_fraction': (curb_points/len(raw_projections)
                                      if raw_projections else 0.0),
                'classified_as_curb': bool(
                    detection.width >= 1.50 or raw_curb),
            })
            if detection.width >= 1.50 or raw_curb:
                if raw_curb:
                    projection = sorted(
                        raw_projections, key=lambda item: item.d)[
                            len(raw_projections)//2]
                x, y, yaw = interpolate_route(self.route, lengths, projection.s)
                x -= projection.d*math.sin(yaw)
                y += projection.d*math.cos(yaw)
                half = detection.width/2.0
                curved_walls.append(LineFeature(
                    x-half*math.cos(yaw), y-half*math.sin(yaw),
                    x+half*math.cos(yaw), y+half*math.sin(yaw),
                    yaw, detection.width, 0.0))
            else:
                kept.append(detection)
        self.cluster_diagnostics = diagnostics[-12:]
        return tuple(curved_walls), tuple(kept)

    def _current_pose(self):
        pose = self.odom.pose.pose
        return Pose2(pose.position.x, pose.position.y, self._yaw(pose.orientation))

    def _stamp_regressed(self, source, stamp_ns, previous_ns):
        if previous_ns is not None and stamp_ns < previous_ns:
            self._handle_time_jump(
                self.get_clock().now(), True,
                f'{source} stamp regressed {previous_ns}->{stamp_ns}')
            return True
        return False

    def _handle_time_jump(self, now, large, detail):
        if self.state != 'TIME_RESET_STOP':
            self.time_reset_resume_state = self.state
        self.state, self.reason = 'TIME_RESET_STOP', detail
        self.time_reset_is_large = self.time_reset_is_large or bool(large)
        self.last_tick_time = now
        self.odom_receive_time = None
        self.last_front_scan_time = None
        self.last_rear_scan_time = None
        self.stopped_since = None
        self.pending_scans.clear()
        self.tf_ready_frames.clear()
        self.front_tf_ready = False
        self.rear_tf_ready = False
        self.tf_ready = False
        self.last_scan_stamp = None
        self.replan_pub.publish(Bool(data=True))
        self.get_logger().error(
            f'TIME_RESET_STOP: {detail}; '
            f'{"manual restart required" if large else "waiting for fresh exact-stamp inputs"}')

    def _time_reset_recovery_ready(self):
        rear_ready = (not bool(self.p['rear_lidar_required']) or
                      self.last_rear_scan_time is not None)
        return (not self.time_reset_is_large and
                self.odom_receive_time is not None and
                self.last_front_scan_time is not None and rear_ready and
                self.tf_ready)

    def _tick(self):
        now = self.get_clock().now()
        if self.last_tick_time is None:
            self.last_tick_time = now
        else:
            delta_ns = (now-self.last_tick_time).nanoseconds
            if delta_ns <= 0 or delta_ns > int(
                    float(self.p['clock_forward_jump_threshold_s'])*1e9):
                backwards_ns = max(0, -delta_ns)
                large = backwards_ns >= int(
                    float(self.p['clock_reset_threshold_s'])*1e9)
                self._handle_time_jump(now, large, f'ROS time delta_ns={delta_ns}')
                self._publish_status()
                return
            self.last_tick_time = now
        self._drain_scan_queue()
        self._update_cpu_usage()
        self._publish_stop_contract()
        self._watchdog()
        if self.state == 'TIME_RESET_STOP':
            if self._time_reset_recovery_ready():
                resume = self.time_reset_resume_state or 'FOLLOWING_CSV'
                self.state, self.reason = resume, 'TIME_RECOVERED'
                self.time_reset_resume_state = None
                self.replan_pub.publish(Bool(data=False))
                if resume == 'PATH_READY' and self.selected_path_msg is not None:
                    stamp = now.to_msg()
                    self.selected_path_msg.header.stamp = stamp
                    for pose in self.selected_path_msg.poses:
                        pose.header.stamp = stamp
                    self.selected_path_pub.publish(self.selected_path_msg)
                self.get_logger().warning(
                    'TIME_RECOVERED: fresh odom/front scan and exact TF verified')
            self._publish_status()
            return
        if not self.tf_ready or self.odom is None or len(self.route) < 2:
            self._publish_status()
            return
        if self.state == 'DYNAMIC_OBSTACLE_STOP' and self.selected_track is not None:
            active_ids = {track.track_id for track in self.tracks}
            if self.selected_track.track_id not in active_ids:
                nearby_static = [
                    track for track in self.tracks
                    if track.state == STATIC_OBSTACLE and math.hypot(
                        track.x-self.selected_track.x,
                        track.y-self.selected_track.y) <=
                    2.0*float(self.p['association_distance_m'])]
                if nearby_static:
                    previous_id = self.selected_track.track_id
                    self.selected_track = min(
                        nearby_static, key=lambda track: math.hypot(
                            track.x-self.selected_track.x,
                            track.y-self.selected_track.y))
                    self.get_logger().info(
                        f'STATIC_TRACK_REASSOCIATED: {previous_id}->'
                        f'{self.selected_track.track_id}')
        if (self.state == 'DYNAMIC_OBSTACLE_STOP' and self.selected_track is not None and
                self.selected_track.state == STATIC_OBSTACLE):
            self.debounce = ReplanDebounce(int(self.p['confirmation_frames']))
            # The follower already latched its stop on the dynamic hazard.  It
            # cannot drive closer to re-enter the distance trigger, so promote
            # the now-confirmed static track directly into stopped planning.
            self.debounce.latched = True
            detail = f'track={self.selected_track.track_id} stationary-confirmed'
            self._set_state('REPLAN_REQUIRED', detail)
            self.replan_pub.publish(Bool(data=True))
            self._set_state('STOPPING', 'waiting for odom-confirmed stop')
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
        track_decisions = []
        for track in self.tracks:
            if track.state not in (STATIC_OBSTACLE, DYNAMIC_OBSTACLE):
                track_decisions.append({
                    'track_id': track.track_id, 'decision': 'UNCONFIRMED',
                    'state': track.state, 'x': track.x, 'y': track.y})
                continue
            if track.track_id in self.passed_track_ids:
                track_decisions.append({
                    'track_id': track.track_id, 'decision': 'PASSED_ID',
                    'state': track.state, 'x': track.x, 'y': track.y})
                continue
            track_projection = project_route(
                self.route, track.x, track.y, self.route_nearest_index)
            if self._is_beyond_route_goal(track):
                track_decisions.append({
                    'track_id': track.track_id,
                    'decision': 'BEYOND_ROUTE_GOAL',
                    'state': track.state, 's': track_projection.s,
                    'd': track_projection.d, 'x': track.x, 'y': track.y})
                continue
            # A curved box is commonly split into new front/side LiDAR tracks
            # after rejoin.  Track IDs are transient, so suppress every face
            # in the already-passed physical obstacle's longitudinal band.
            if self._is_passed_obstacle_face(track_projection.s):
                self.passed_track_ids.add(track.track_id)
                track_decisions.append({
                    'track_id': track.track_id, 'decision': 'PASSED_S_BAND',
                    'state': track.state, 's': track_projection.s,
                    'd': track_projection.d, 'x': track.x, 'y': track.y})
                continue
            # A finite curved-curb chord projects slightly inside its true
            # Frenet offset.  Keep a classification band around |d|=1.095;
            # real course obstacles at |d|=0.78 remain well outside it.
            curb_band = 0.16
            if (track_projection.d >= float(self.p['left_curb_inner_y_m'])-curb_band or
                    track_projection.d <= float(self.p['right_curb_inner_y_m'])+curb_band):
                track_decisions.append({
                    'track_id': track.track_id, 'decision': 'CURB_BAND',
                    'state': track.state, 's': track_projection.s,
                    'd': track_projection.d, 'x': track.x, 'y': track.y})
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
                float(self.p['minimum_obstacle_depth_m']), self.route_nearest_index,
                float(self.p['front_lidar_x_offset_m']),
                float(self.p['minimum_obstacle_width_m']))
            self.route_nearest_index = max(self.route_nearest_index, risk.nearest_path_index)
            if risk.required:
                self.path_relevant_track_ids.add(track.track_id)
            remembered_risk = track.track_id in self.path_relevant_track_ids
            track_decisions.append({
                'track_id': track.track_id,
                'decision': ('COLLISION_RISK' if risk.required else
                             ('LATCHED_COLLISION_RISK' if remembered_risk else
                              'NO_COLLISION_RISK')),
                'state': track.state, 's': track_projection.s,
                'd': track_projection.d, 'x': track.x, 'y': track.y,
                'lidar_surface_distance_m': risk.lidar_surface_distance,
                'collision_path_index': risk.collision_path_index})
            if remembered_risk:
                risks.append((risk.lidar_surface_distance, track, risk))
        self.last_track_decisions = track_decisions
        for decision in track_decisions:
            track_id = decision['track_id']
            signature = (decision['state'], decision['decision'])
            if self.track_decision_states.get(track_id) != signature:
                self.track_decision_states[track_id] = signature
                self.get_logger().info(
                    'TRACK_DECISION ' + json.dumps(decision, sort_keys=True))
        if risks:
            distance, track, risk = min(risks, key=lambda item: item[0])
            self.nearest_distance_pub.publish(Float32(data=float(distance)))
            self.lidar_surface_distance_pub.publish(
                Float32(data=float(risk.lidar_surface_distance)))
            self.vehicle_front_distance_pub.publish(
                Float32(data=float(risk.vehicle_front_surface_distance)))
            self.obstacle_center_distance_pub.publish(
                Float32(data=float(risk.obstacle_center_distance)))
            self.collision_point_distance_pub.publish(
                Float32(data=float(risk.collision_point_distance)))
            if (risk.lidar_surface_distance <=
                    float(self.p['replan_trigger_distance_m']) or risk.emergency):
                self.selected_track = track
                if track.state == DYNAMIC_OBSTACLE:
                    self.debounce.latched = True
                    self._set_state('DYNAMIC_OBSTACLE_STOP', 'dynamic obstacle intersects CSV path')
                    self.replan_pub.publish(Bool(data=True))
                elif self.debounce.update(True, track.track_id):
                    current = self._current_pose()
                    twist = self.odom.twist.twist
                    self.get_logger().info('STOP_TRIGGER ' + json.dumps({
                        'track_id': track.track_id,
                        'lidar_surface_distance_m': risk.lidar_surface_distance,
                        'vehicle_x': current.x, 'vehicle_y': current.y,
                        'vehicle_yaw': current.yaw,
                        'obstacle_x': track.x, 'obstacle_y': track.y,
                        'csv_index': self.route_nearest_index,
                        'odom_linear_speed_mps': twist.linear.x,
                        'odom_angular_speed_rps': twist.angular.z,
                        'stamp_ns': self.get_clock().now().nanoseconds,
                    }, sort_keys=True))
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
                self.selected_track, float(self.p['minimum_obstacle_depth_m']),
                float(self.p['min_cluster_width_m']))
            selected_projection = project_route(
                self.route, self.selected_track.x, self.selected_track.y,
                self.route_nearest_index)
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
                'target_fractions': (() if self.p['adaptive_candidate_sampling'] else
                                     tuple(self.p['corridor_candidate_fractions'])),
                'return_lengths': tuple(self.p['return_transition_lengths_m']),
                'rejoin_straight_extension': float(
                    self.p['rejoin_straight_extension_m']),
                'minimum_index': self.route_nearest_index,
                'collision_check_interval': float(self.p['collision_check_interval_m']),
                # Fixed-environment detections are complete SDF boxes, not a
                # single LiDAR face. Preserve both their front face and actual
                # d extent instead of shifting the box forward by half a body.
                'obstacle_surface_observation': not bool(
                    self.p['fixed_environment_mode']),
                'obstacle_length': float(self.p['minimum_obstacle_depth_m']),
                'obstacle_width': float(self.p['minimum_obstacle_width_m']),
                'lateral_target_samples': int(self.p['lateral_target_samples']),
                # Fixed-map tracks contain the actual SDF box centre. Using
                # the old nominal |d|=0.78 moved facility obstacle_1 from its
                # measured d=0.61 outward and falsely approved a colliding
                # near-centreline path.
                'expected_obstacle_lateral_center': abs(selected_projection.d),
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
                self._set_state('PATH_INFEASIBLE', 'no collision-free <=25 deg candidate')
            else:
                self._set_state('PATH_READY', f'candidate={result.selected.candidate_id}')
                selected = result.selected
                self.get_logger().info('PATH_READY_METRICS ' + json.dumps({
                    'track_id': self.selected_track.track_id,
                    'candidate_id': selected.candidate_id,
                    'candidate_count': len(result.candidates),
                    'valid_candidate_count': sum(
                        candidate.valid for candidate in result.candidates),
                    'path_length_m': selected.length,
                    'max_curvature_1pm': selected.max_curvature,
                    'required_steering_deg': math.degrees(
                        selected.max_steering_rad),
                    'obstacle_clearance_m': selected.obstacle_clearance,
                    'curb_clearance_m': selected.curb_clearance,
                    'terminal_curvature_error_1pm':
                        selected.terminal_curvature_error,
                    'rejoin_index': selected.rejoin_index,
                }, sort_keys=True))

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
        midpoints = [(0.5*(wall.x1+wall.x2), 0.5*(wall.y1+wall.y2))
                     for wall in self.walls]
        lateral = [project_route(self.route, x, y, self.route_nearest_index).d
                   for x, y in midpoints] if len(self.route) >= 2 else []
        left = [value for value in lateral if value > 0.0]
        right = [value for value in lateral if value < 0.0]
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
            'DYNAMIC_OBSTACLE_STOP', 'TIME_RESET_STOP', 'ERROR')
        if active:
            self.replan_pub.publish(Bool(data=True))

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
        rejection_counts = {}
        for candidate in result.candidates:
            marker = self._marker('candidate_paths', candidate.candidate_id, Marker.LINE_STRIP)
            marker.points = [Point(x=pose.x, y=pose.y, z=0.04) for pose in candidate.path]
            marker.scale.x = 0.035
            if candidate.valid:
                valid_count += 1
                marker.color.r, marker.color.g, marker.color.a = 1.0, 0.85, 0.55
            else:
                rejection_counts[candidate.reason] = (
                    rejection_counts.get(candidate.reason, 0)+1)
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.75, 0.15, 0.15, 0.35
            candidates.markers.append(marker)
        self.candidate_pub.publish(candidates)
        self.candidate_count_pub.publish(Int32(data=len(result.candidates)))
        self.valid_candidate_count_pub.publish(Int32(data=valid_count))
        self.last_plan_summary = {
            'candidate_count': len(result.candidates),
            'valid_candidate_count': valid_count,
            'rejection_counts': rejection_counts,
            'corridor_side': result.corridor.side,
            'corridor_width_m': result.corridor.width,
            'max_candidate_steering_deg': max(
                (math.degrees(item.max_steering_rad)
                 for item in result.candidates), default=0.0),
            'candidates': [{
                'candidate_id': item.candidate_id,
                'entry_length_m': item.entry_length,
                'lateral_offset_m': item.target_d,
                'hold_length_m': item.hold_length,
                'return_length_m': item.return_length,
                'rejoin_index': item.rejoin_index,
                'max_curvature_1pm': item.max_curvature,
                'max_steering_deg': math.degrees(item.max_steering_rad),
                'obstacle_clearance_m': item.obstacle_clearance,
                'curb_clearance_m': item.curb_clearance,
                'collision_path_index': item.collision_path_index,
                'collision_pose': (None if item.collision_pose is None else {
                    'x': item.collision_pose.x, 'y': item.collision_pose.y,
                    'yaw': item.collision_pose.yaw}),
                'collision_target': item.collision_target or None,
                'rejection_reason': item.reason or None,
            } for item in result.candidates],
        }
        if result.selected is not None:
            self.last_plan_summary.update({
                'selected_candidate_id': result.selected.candidate_id,
                'selected_return_length_m': result.selected.return_length,
                'selected_max_steering_deg': math.degrees(
                    result.selected.max_steering_rad),
                'selected_max_curvature_rate': result.selected.max_curvature_rate,
                'rejoin_curvature_error': result.selected.terminal_curvature_error,
            })
        self.planning_time_pub.publish(Float32(data=float(elapsed_ms)))
        obstacle = track_box(
            self.selected_track, float(self.p['minimum_obstacle_depth_m']),
            float(self.p['minimum_obstacle_width_m']))
        corridor = self._marker('safe_corridor', 0, Marker.CUBE)
        obstacle_center_x = 0.5*(obstacle.min_x+obstacle.max_x)
        obstacle_center_y = 0.5*(obstacle.min_y+obstacle.max_y)
        obstacle_projection = project_route(
            self.route, obstacle_center_x, obstacle_center_y,
            self.route_nearest_index)
        route_lengths_value = route_lengths(self.route)
        route_x, route_y, route_yaw = interpolate_route(
            self.route, route_lengths_value, obstacle_projection.s)
        corridor_d = 0.5*(result.corridor.lower_center_d+
                         result.corridor.upper_center_d)
        corridor.pose.position.x = route_x-corridor_d*math.sin(route_yaw)
        corridor.pose.position.y = route_y+corridor_d*math.cos(route_yaw)
        corridor.pose.position.z = 0.025
        corridor.pose.orientation.z = math.sin(route_yaw/2.0)
        corridor.pose.orientation.w = math.cos(route_yaw/2.0)
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
        self.selected_path_msg = selected_path
        self.selected_track_pub.publish(Int32(data=self.selected_track.track_id))

    def _obstacle_statuses(self):
        """Classify tracked/passed obstacles as UNSEEN/TRACKED/PLANNING/
        AVOIDING/PASSED, ordered by x so the first two match obstacle_layout
        ordering (obstacle_1 = nearer to route start)."""
        known_ids = {track.track_id for track in self.tracks
                     if track.state in (STATIC_OBSTACLE, DYNAMIC_OBSTACLE)}
        known_ids |= self.passed_track_ids
        by_id = {track.track_id: track for track in self.tracks}
        entries = []
        for track_id in known_ids:
            track = by_id.get(track_id)
            if track_id in self.passed_track_ids:
                status = 'PASSED'
            elif self.selected_track is not None and track_id == self.selected_track.track_id:
                status = 'AVOIDING' if self.avoidance_started else 'PLANNING'
            else:
                status = 'TRACKED'
            x = track.x if track is not None else (
                self.selected_track.x if self.selected_track and
                self.selected_track.track_id == track_id else math.inf)
            y = getattr(track, 'y', 0.0) if track is not None else (
                getattr(self.selected_track, 'y', 0.0) if self.selected_track and
                self.selected_track.track_id == track_id else math.inf)
            route = getattr(self, 'route', ())
            route_s = (project_route(route, x, y, 0).s
                       if math.isfinite(x) and math.isfinite(y) and len(route) >= 2
                       else x)
            entries.append({'track_id': track_id, 'status': status, '_sort_s': route_s,
                            'x': x if math.isfinite(x) else None,
                            'y': y if math.isfinite(y) else None, 's': route_s})
        entries.sort(key=lambda item: item['_sort_s'])
        for order, entry in enumerate(entries, start=1):
            entry['label'] = f'obstacle_{order}'
            del entry['_sort_s']
        return entries

    def _publish_obstacle_status(self, statuses):
        markers = self._clear_array()
        for entry in statuses:
            marker = self._marker(
                'obstacle_status', entry['track_id'], Marker.TEXT_VIEW_FACING)
            marker.pose.position.x = entry['x'] if entry['x'] is not None else 0.0
            marker.pose.position.y = entry['y'] if entry['y'] is not None else 0.0
            marker.pose.position.z = 1.6
            marker.scale.z = 0.18
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            marker.text = f"{entry['label']}: {entry['status']} (track={entry['track_id']})"
            markers.markers.append(marker)
        self.obstacle_status_pub.publish(markers)

    def _publish_status(self):
        obstacle_statuses = self._obstacle_statuses()
        self._publish_obstacle_status(obstacle_statuses)
        payload = {
            'state': self.state, 'reason': self.reason,
            'selected_track_id': self.selected_track.track_id if self.selected_track else -1,
            'track_count': len(self.tracks), 'wall_count': len(self.walls),
            'scan_count': self.scan_count, 'scan_drops': self.scan_drops,
            'tf_failures': self.tf_failures, 'performance_ms': self.performance,
            'tf_ready': self.tf_ready,
            'startup_tf_drop_count': self.startup_tf_drop_count,
            'runtime_tf_drop_count': self.runtime_tf_drop_count,
            'future_extrapolation_count': self.future_extrapolation_count,
            'past_extrapolation_count': self.past_extrapolation_count,
            'missing_frame_count': self.missing_frame_count,
            'last_tf_error': self.last_tf_error,
            'tracks': [
                {'id': track.track_id, 'state': track.state,
                 'x': round(track.x, 4), 'y': round(track.y, 4),
                 'speed_mps': round(track.speed, 4)}
                for track in self.tracks],
            'boundary_source': self.boundary_source,
            'state_history': self.state_history[-12:],
            'obstacle_statuses': obstacle_statuses,
            'passed_track_ids': sorted(self.passed_track_ids),
            'drives_selected_path': False,
            'selected_path_driving_enabled': True,
            'last_plan_summary': self.last_plan_summary,
            'last_track_decisions': self.last_track_decisions,
            'cluster_diagnostics': self.cluster_diagnostics,
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
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
