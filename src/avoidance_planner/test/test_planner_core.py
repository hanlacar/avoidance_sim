import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from avoidance_planner.collision_evaluator import (
    ReplanDebounce, collision_response, evaluate_track_collision)
from avoidance_planner.geometry import (
    Box2, Pose2, footprint_collision, path_curvatures, path_frenet_collision,
    path_min_clearances, steering_angles, vehicle_polygon)
from avoidance_planner.local_planner import (
    adaptive_corridor_fractions, calculate_corridor, interpolate_route,
    observed_surface_to_frenet_box, plan_candidates, project_route,
    quintic_blend, route_lengths, world_box_to_frenet)
from avoidance_planner.perception import (
    DYNAMIC_OBSTACLE, STATIC_OBSTACLE, TrackManager, ScanPoint,
    adaptive_clusters, adaptive_gap, cluster_groups, preprocess_scan,
    ransac_line, split_walls_and_objects)


def straight_route(length=15.0, step=0.05):
    return tuple(Pose2(i*step, 0.0, 0.0)
                 for i in range(round(length/step)+1))


def curved_route(length=30.0, step=0.10):
    points = []
    for index in range(round(length/step)+1):
        x = index*step
        y = 0.80*math.sin(2.0*math.pi*x/length)
        dy = 0.80*(2.0*math.pi/length)*math.cos(2.0*math.pi*x/length)
        points.append(Pose2(x, y, math.atan2(dy, 1.0)))
    return tuple(points)


def test_route_interpolation_uses_csv_tangents_without_waypoint_corners():
    radius = 4.0
    route = tuple(
        Pose2(radius*math.sin(angle), radius*(1.0-math.cos(angle)), angle)
        for angle in (0.0, 0.04, 0.08, 0.12, 0.16))
    lengths = route_lengths(route)
    samples = tuple(
        Pose2(*interpolate_route(route, lengths, index*lengths[-1]/80.0))
        for index in range(81))
    maximum = max(abs(value) for value in path_curvatures(samples))
    assert maximum == pytest.approx(1.0/radius, abs=0.01)
    assert math.degrees(math.atan(0.77*maximum)) < 25.0


def route_aligned_world_box(route, s, d, length=0.65, width=0.39):
    x, y, yaw = interpolate_route(route, route_lengths(route), s)
    x -= d*math.sin(yaw)
    y += d*math.cos(yaw)
    c, sn = abs(math.cos(yaw)), abs(math.sin(yaw))
    half_x = 0.5*(length*c+width*sn)
    half_y = 0.5*(length*sn+width*c)
    return Box2(x-half_x, x+half_x, y-half_y, y+half_y)


def scan_points(count=6, x=3.0, y0=0.55, spacing=0.04):
    return tuple(ScanPoint(i, x, y0+i*spacing, math.hypot(x, y0+i*spacing))
                 for i in range(count))


def track(x=6.0, y=0.78, state=STATIC_OBSTACLE):
    return SimpleNamespace(
        x=x, y=y, min_x=x-0.025, max_x=x+0.025,
        min_y=y-0.195, max_y=y+0.195, state=state, track_id=1)


def test_track_epoch_reset_discards_associations_without_reusing_ids():
    manager = TrackManager(confirmation_frames=1, static_frames=1)
    detection = SimpleNamespace(
        x=3.0, y=0.7, min_x=2.9, max_x=3.1, min_y=0.6, max_y=0.8)
    first = manager.update((detection,), 1.0)
    assert [item.track_id for item in first] == [1]
    manager.reset_epoch()
    second = manager.update((detection,), 2.0)
    assert [item.track_id for item in second] == [2]


def test_nan_inf_and_out_of_range_removed():
    points = preprocess_scan(
        [math.nan, math.inf, 0.05, 1.0, 13.0], -0.02, 0.01,
        0.12, 12.0, 0.1, 5.0, 2.0, -0.5, 0.5, 0.2)
    assert [point.index for point in points] == [3]


def test_self_reflection_removed():
    points = preprocess_scan(
        [0.2, 1.0], 0.0, 0.0, 0.12, 12.0,
        0.1, 5.0, 2.0, -0.5, 0.5, 0.3)
    assert [point.index for point in points] == [1]


def test_minimum_point_count_removes_noise():
    assert adaptive_clusters(scan_points(3), 4, 0.08, 0.1, 0.2, 8.0) == []


def test_minimum_physical_width_removes_tiny_cluster():
    points = scan_points(4, spacing=0.005)
    assert adaptive_clusters(points, 4, 0.08, 0.1, 0.2, 8.0) == []


def test_distance_adaptive_clustering_threshold():
    assert adaptive_gap(1.0, 0.10, 0.20, 8.0) < adaptive_gap(8.0, 0.10, 0.20, 8.0)
    near = scan_points(4, x=1.0, spacing=0.15)
    far = scan_points(4, x=8.0, spacing=0.15)
    assert len(adaptive_clusters(near, 2, 0.08, 0.10, 0.20, 8.0)) == 0
    assert len(adaptive_clusters(far, 2, 0.08, 0.10, 0.20, 8.0)) == 1


def test_missing_scan_index_splits_clusters():
    points = scan_points(3) + tuple(
        ScanPoint(p.index+4, p.x, p.y+0.2, p.distance) for p in scan_points(3))
    assert len(cluster_groups(points, 0.1, 0.2, 8.0)) == 2


def test_track_id_is_maintained():
    manager = TrackManager(static_frames=3)
    detection = adaptive_clusters(scan_points(), 4, 0.08, 0.1, 0.2, 8.0)[0]
    first = manager.update([detection], 0.0)[0]
    second = manager.update([detection], 0.1)[0]
    assert first.track_id == second.track_id == 1


def test_consecutive_frames_confirm_static_obstacle():
    manager = TrackManager(confirmation_frames=3, static_frames=3, alpha=1.0)
    detection = adaptive_clusters(scan_points(), 4, 0.08, 0.1, 0.2, 8.0)[0]
    for index in range(4):
        result = manager.update([detection], index*0.1)
    assert result[0].state == STATIC_OBSTACLE


def test_new_track_does_not_use_first_velocity_for_dynamic_state():
    manager = TrackManager(association_distance=1.0, alpha=1.0,
                           dynamic_enter=0.15, dynamic_frames=1,
                           dynamic_min_observations=3, static_frames=5)
    for index, x in enumerate((2.0, 2.03)):
        result = manager.update(
            [adaptive_clusters(scan_points(x=x), 4, 0.08, 0.1, 0.2, 8.0)[0]],
            index*0.1)
    assert result[0].state == 'UNCONFIRMED'


def test_consistent_motion_confirms_dynamic_obstacle():
    manager = TrackManager(association_distance=1.0, alpha=1.0,
                           dynamic_enter=0.15, dynamic_frames=3,
                           static_frames=5)
    for index in range(5):
        base = adaptive_clusters(scan_points(x=2.0+0.03*index), 4, 0.08, 0.1, 0.2, 8.0)[0]
        result = manager.update([base], index*0.1)
    assert result[0].state == DYNAMIC_OBSTACLE
    assert result[0].speed == pytest.approx(0.3)


def test_dynamic_to_static_hysteresis_requires_multiple_frames():
    manager = TrackManager(association_distance=1.0, alpha=1.0,
                           dynamic_enter=0.15, dynamic_exit=0.08,
                           dynamic_frames=2, static_frames=3)
    for index, x in enumerate((2.0, 2.03, 2.06, 2.09)):
        result = manager.update(
            [adaptive_clusters(scan_points(x=x), 4, 0.08, 0.1, 0.2, 8.0)[0]],
            index*0.1)
    assert result[0].state == DYNAMIC_OBSTACLE
    stationary = adaptive_clusters(scan_points(x=2.06), 4, 0.08, 0.1, 0.2, 8.0)[0]
    manager.update([stationary], 0.3)
    manager.update([stationary], 0.4)
    assert manager.tracks[1].state == DYNAMIC_OBSTACLE
    manager.update([stationary], 0.5)
    assert manager.tracks[1].state == DYNAMIC_OBSTACLE
    manager.update([stationary], 0.6)
    assert manager.tracks[1].state == STATIC_OBSTACLE


def test_lost_track_is_removed_after_configured_frames():
    manager = TrackManager(lost_frames=2)
    detection = adaptive_clusters(scan_points(), 4, 0.08, 0.1, 0.2, 8.0)[0]
    manager.update([detection], 0.0)
    manager.update([], 0.1)
    assert manager.tracks
    manager.update([], 0.2)
    assert not manager.tracks


def test_ransac_wall_detection_rejects_outlier():
    points = [ScanPoint(i, i*0.25, 1.095, i*0.25) for i in range(12)]
    points.append(ScanPoint(12, 1.0, 0.5, 1.2))
    line = ransac_line(points, 0.04, 0.70)
    assert line.length > 2.5 and line.residual < 1.0e-6


def test_wall_and_independent_obstacle_are_separated():
    wall = tuple(ScanPoint(i, i*0.25, 1.095, i*0.25) for i in range(12))
    obstacle = scan_points(8)
    walls, objects, unknown = split_walls_and_objects(
        (wall, obstacle), 0.0, 1.5, 0.05, math.radians(15), 0.08)
    assert len(walls) == 1 and len(objects) == 1 and not unknown


def test_short_uncertain_structure_is_unknown():
    tiny = scan_points(4, spacing=0.005)
    walls, objects, unknown = split_walls_and_objects(
        (tiny,), 0.0, 1.5, 0.05, math.radians(15), 0.08)
    assert not walls and not objects and len(unknown) == 1


def test_path_obstacle_collision_is_detected_with_full_footprint():
    risk = evaluate_track_collision(
        straight_route(10), Pose2(0, 0, 0), track(), 1.30, 0.78, 0.0,
        0.20, 0.15, 8.0, 0.5, 1.095, -1.095, 0.65)
    assert risk.required and risk.collision_path_index > 0


def test_two_metre_trigger_uses_lidar_to_physical_surface_not_center():
    risk = evaluate_track_collision(
        straight_route(10), Pose2(3.025, 0, 0), track(), 1.30, 0.78, 0.0,
        0.20, 0.15, 8.0, 0.5, 1.095, -1.095, 0.65, 0, 0.65)
    assert risk.lidar_surface_distance == pytest.approx(2.0)
    assert risk.vehicle_front_surface_distance == pytest.approx(2.0)
    assert risk.obstacle_center_distance > risk.lidar_surface_distance
    assert risk.collision_point_distance == risk.longitudinal_distance


@pytest.mark.parametrize('distance', [2.0, 3.0, 4.0, 5.0])
def test_offline_planning_distance_comparison(distance):
    obstacle = Box2(5.675, 6.325, 0.585, 0.975)
    current_x = obstacle.min_x-0.65-distance
    result = plan_candidates(
        straight_route(), Pose2(current_x, 0.0, 0.0), obstacle,
        1.095, -1.095)
    assert len(result.candidates) == 9
    assert result.selected is not None


def test_obstacle_outside_path_is_ignored():
    risk = evaluate_track_collision(
        straight_route(10), Pose2(0, 0, 0), track(y=1.40), 1.30, 0.78, 0.0,
        0.10, 0.15, 8.0, 0.5, 1.095, -1.095, 0.65)
    assert not risk.required


def test_narrow_side_return_keeps_unresolved_centreward_collision_relevance():
    item = track(y=0.68)
    item.min_y, item.max_y = 0.64, 0.72
    risk = evaluate_track_collision(
        straight_route(10), Pose2(3.5, 0.0, 0.0), item,
        1.30, 0.78, 0.0, 0.20, 0.15, 8.0, 0.5,
        1.095, -1.095, 0.65, 0, 0.65, 0.39)
    assert risk.required
    assert risk.lidar_surface_distance < 2.0


def test_vehicle_footprint_is_rectangle_not_point():
    polygon = vehicle_polygon(Pose2(0, 0, 0), 1.30, 0.78)
    assert polygon[0] == pytest.approx((-0.65, -0.39))
    assert polygon[2] == pytest.approx((0.65, 0.39))


def test_footprint_collision_with_inflated_obstacle():
    obstacle = Box2(0.60, 1.0, 0.20, 0.40)
    assert footprint_collision(Pose2(0, 0, 0), 1.30, 0.78, 0,
                               (obstacle,), 1.095, -1.095)


def test_safe_corridor_uses_edges_and_vehicle_width():
    corridor = calculate_corridor(
        Box2(5.7, 6.3, 0.585, 0.975), 1.095, -1.095,
        0.78, 0.20, 0.08)
    assert corridor.side == 'RIGHT_PASS'
    assert corridor.lower_center_d == pytest.approx(-0.625)
    assert corridor.upper_center_d == pytest.approx(-0.005)


def test_corridor_narrower_than_vehicle_is_rejected():
    corridor = calculate_corridor(
        Box2(5.7, 6.3, -0.5, 0.975), 1.095, -1.095,
        0.78, 0.20, 0.08)
    assert corridor.width <= 0.0


def test_quintic_connection_has_zero_endpoint_slope():
    epsilon = 1.0e-5
    assert (quintic_blend(epsilon)-quintic_blend(0))/epsilon < 1.0e-3
    assert (quintic_blend(1)-quintic_blend(1-epsilon))/epsilon < 1.0e-3


def test_route_projection_never_uses_behind_minimum_index():
    projection = project_route(straight_route(), 2.0, 0.2, minimum_index=50)
    assert projection.index >= 50


def test_curved_route_projection_reports_signed_frenet_lateral_distance():
    route = curved_route()
    pose = route[100]
    x = pose.x-0.40*math.sin(pose.yaw)
    y = pose.y+0.40*math.cos(pose.yaw)
    projection = project_route(route, x, y)
    assert projection.index in (99, 100)
    assert projection.d == pytest.approx(0.40, abs=2.0e-3)


def test_route_aligned_obstacle_recovers_local_edges_on_curve():
    route = curved_route()
    local = world_box_to_frenet(route, route_aligned_world_box(route, 10.0, 0.78))
    assert 0.63 < local.max_x-local.min_x < 0.67
    assert 0.37 < local.max_y-local.min_y < 0.41
    assert 0.76 < 0.5*(local.min_y+local.max_y) < 0.80


@pytest.mark.parametrize('side', [-0.78, 0.78])
def test_common_planner_builds_curved_frenet_avoidance_and_rejoin(side):
    route = curved_route()
    obstacle_s = 10.0
    current_x, current_y, current_yaw = interpolate_route(
        route, route_lengths(route), obstacle_s-2.975)
    result = plan_candidates(
        route, Pose2(current_x, current_y, current_yaw),
        route_aligned_world_box(route, obstacle_s, side), 1.095, -1.095,
        target_fractions=(0.25, 0.50, 0.75, 0.80, 0.85, 0.90),
        rejoin_straight_extension=2.0)
    assert result.selected is not None
    assert result.selected.max_steering_rad <= math.radians(25.0)
    assert result.selected.obstacle_clearance >= 0.20
    assert result.selected.curb_clearance > 0.0
    assert result.selected.terminal_curvature_error <= 0.04
    assert project_route(route, result.selected.path[-1].x,
                         result.selected.path[-1].y).d == pytest.approx(0.0, abs=1.0e-4)


def test_curved_boundary_check_uses_route_normal_not_world_y():
    route = tuple(Pose2(p.x, p.y+2.0, p.yaw) for p in curved_route())
    assert not path_frenet_collision(
        route[30:40], route, 1.30, 0.78, 0.0, (), 1.095, -1.095)


def test_curved_two_metre_trigger_uses_frenet_obstacle_surface():
    route = curved_route()
    obstacle_s = 10.0
    current_s = obstacle_s-0.325-0.65-2.0
    x, y, yaw = interpolate_route(route, route_lengths(route), current_s)
    box = route_aligned_world_box(route, obstacle_s, 0.78)
    center_x = 0.5*(box.min_x+box.max_x)
    center_y = 0.5*(box.min_y+box.max_y)
    fake_track = SimpleNamespace(
        x=center_x, y=center_y, min_x=box.min_x, max_x=box.max_x,
        min_y=box.min_y, max_y=box.max_y, state=STATIC_OBSTACLE, track_id=9)
    risk = evaluate_track_collision(
        route, Pose2(x, y, yaw), fake_track, 1.30, 0.78, 0.0,
        0.20, 0.15, 8.0, 0.5, 1.095, -1.095, 0.65, 0, 0.65)
    assert risk.required
    # Polyline Frenet projection of a finite-width body on a curve differs
    # slightly from centreline arc distance; stay within 3 cm of the surface.
    assert risk.lidar_surface_distance == pytest.approx(2.0, abs=0.03)


def nominal_plan():
    return plan_candidates(
        straight_route(), Pose2(3.0, 0.0, 0.0),
        Box2(5.675, 6.325, 0.585, 0.975), 1.095, -1.095)


def test_multiple_candidate_paths_are_generated():
    result = plan_candidates(
        straight_route(), Pose2(3.0, 0.0, 0.0),
        Box2(5.675, 6.325, 0.585, 0.975), 1.095, -1.095,
        target_fractions=(0.25, 0.50, 0.75, 0.80, 0.85, 0.90))
    assert len(result.candidates) == 18


def test_dense_corridor_edge_sampling_recovers_twenty_five_degree_candidate():
    route = tuple(Pose2(index*0.1, 0.0, 0.0) for index in range(216))
    result = plan_candidates(
        route, Pose2(14.0622257, 0.0479746, -0.0489444),
        Box2(16.4647, 17.1147, 0.56179645, 0.823), 1.095, -1.095,
        target_fractions=(0.25, 0.50, 0.75, 0.80, 0.85, 0.90))
    assert result.selected is not None
    assert result.selected.max_steering_rad <= math.radians(25.0)
    assert result.selected.obstacle_clearance > 0.20
    assert result.selected.curb_clearance > 0.0


def test_candidate_starts_at_current_pose_and_returns_to_csv():
    selected = nominal_plan().selected
    assert selected.path[0].x == pytest.approx(3.0)
    assert selected.path[0].y == pytest.approx(0.0)
    assert selected.path[-1].y == pytest.approx(0.0, abs=1.0e-6)


def test_candidate_heading_is_continuous_at_start_and_return():
    selected = nominal_plan().selected
    assert selected.path[0].yaw == pytest.approx(0.0, abs=1.0e-6)
    assert selected.path[-1].yaw == pytest.approx(0.0, abs=1.0e-4)


def test_candidate_has_straight_csv_rejoin_extension():
    selected = nominal_plan().selected
    terminal, previous = selected.path[-1], selected.path[-2]
    assert terminal.x > previous.x
    assert abs(terminal.y-previous.y) < 1.0e-6
    assert all(abs(pose.y) < 1.0e-6 for pose in selected.path[-21:])
    assert terminal.yaw == pytest.approx(0.0, abs=1.0e-4)


def test_curvature_and_steering_use_bicycle_model():
    selected = nominal_plan().selected
    curvature = path_curvatures(selected.path)
    steering = steering_angles(selected.path, 0.77)
    assert max(abs(value) for value in curvature) == pytest.approx(selected.max_curvature)
    assert max(abs(value) for value in steering) == pytest.approx(selected.max_steering_rad)


def test_candidates_over_twenty_five_degrees_are_discarded_not_clamped():
    result = nominal_plan()
    rejected = [item for item in result.candidates
                if item.reason == 'STEERING_LIMIT']
    assert rejected and all(item.max_steering_rad > math.radians(25) for item in rejected)


def test_path_facing_surface_extends_obstacle_away_from_csv():
    route = curved_route()
    x, y, _yaw = interpolate_route(route, route_lengths(route), 18.2)
    observed = Box2(x-0.325, x+0.325, y-0.60, y-0.52)
    recovered = observed_surface_to_frenet_box(
        route, observed, obstacle_length=0.65, obstacle_width=0.39)
    assert recovered.max_y < 0.0
    assert recovered.max_y-recovered.min_y == pytest.approx(0.39)
    assert abs(recovered.min_y) > abs(recovered.max_y)


def test_outer_side_surface_extends_toward_csv_when_outward_body_crosses_curb():
    route = curved_route()
    x, y, _yaw = interpolate_route(route, route_lengths(route), 18.2)
    observed = Box2(x-0.325, x+0.325, y+0.78, y+0.86)
    recovered = observed_surface_to_frenet_box(
        route, observed, obstacle_length=0.65, obstacle_width=0.39,
        left_boundary=1.095, right_boundary=-1.095)
    assert recovered.max_y <= 1.095
    assert recovered.max_y-recovered.min_y == pytest.approx(0.39)
    assert recovered.min_y < recovered.max_y-0.30


def test_adaptive_targets_cover_both_corridor_edges_without_touching_them():
    corridor = calculate_corridor(
        Box2(5.0, 5.65, -0.95, -0.56), 1.095, -1.095,
        0.78, 0.20, 0.08)
    values = adaptive_corridor_fractions(corridor, 7)
    assert len(values) == 7
    assert 0.0 < values[0] <= 0.20
    assert 0.80 <= values[-1] < 1.0


def test_minimum_turn_radius_and_curvature_bound_match_twenty_five_degrees():
    wheelbase = 0.77
    r_min = wheelbase/math.tan(math.radians(25.0))
    assert r_min == pytest.approx(1.651, abs=1.0e-3)
    curvature_bound = math.tan(math.radians(25.0))/wheelbase
    assert curvature_bound == pytest.approx(0.6056, abs=1.0e-3)
    assert curvature_bound == pytest.approx(1.0/r_min)


def test_selected_path_respects_minimum_turn_radius():
    selected = nominal_plan().selected
    radius = 1.0/selected.max_curvature
    assert radius >= 0.77/math.tan(math.radians(25))


def test_selected_path_has_positive_obstacle_and_curb_clearance():
    selected = nominal_plan().selected
    assert selected.obstacle_clearance > 0.0
    assert selected.curb_clearance > 0.0


def test_clearance_function_reports_obstacle_and_curb_values():
    path = (Pose2(0, 0, 0),)
    obstacle, curb = path_min_clearances(
        path, 1.30, 0.78, 0, (Box2(2, 3, .5, .8),), 1.095, -1.095)
    assert obstacle > 0.0 and curb == pytest.approx(0.705)


def test_valid_candidate_selection_prefers_finite_score():
    result = nominal_plan()
    assert result.selected is not None and math.isfinite(result.selected.score)
    assert result.selected.score == min(c.score for c in result.candidates if c.valid)


def test_selected_path_prefers_tracking_margin_from_longer_return():
    result = nominal_plan()
    same_target = [candidate for candidate in result.candidates
                   if candidate.valid and candidate.target_d == result.selected.target_d]
    assert result.selected.return_length == max(
        candidate.return_length for candidate in same_target)


def test_no_valid_candidate_returns_path_infeasible_input():
    result = plan_candidates(
        straight_route(), Pose2(5.0, 0, 0),
        Box2(5.5, 6.2, -0.5, 0.975), 1.095, -1.095)
    assert result.selected is None


def test_replan_debounce_requires_confirmation_and_latches():
    debounce = ReplanDebounce(3)
    assert not debounce.update(True, 7)
    assert not debounce.update(True, 7)
    assert debounce.update(True, 7)
    assert debounce.update(False)


def test_dynamic_obstacle_stops_without_path_generation():
    state, planning_allowed = collision_response(DYNAMIC_OBSTACLE)
    assert state == 'DYNAMIC_OBSTACLE_STOP' and not planning_allowed


def test_static_obstacle_allows_stopped_planning():
    state, planning_allowed = collision_response(STATIC_OBSTACLE)
    assert state == 'REPLAN_REQUIRED' and planning_allowed
