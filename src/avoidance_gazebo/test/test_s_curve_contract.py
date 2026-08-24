import csv
import math
from pathlib import Path
import re

import pytest

from avoidance_planner.geometry import Box2, Pose2
from avoidance_planner.local_planner import (
    interpolate_route, plan_candidates, route_lengths)
from avoidance_route.route_following import load_route_csv

from avoidance_gazebo.obstacle_layout import (
    generate_s_layout, same_s_positions)
from avoidance_gazebo.s_curve_course import (
    CURB_INNER_OFFSET, ROAD_HALF_WIDTH, SPAWN_X, WHEELBASE, WORLD_LENGTH,
    center_pose_x, offset_point, polygon_self_intersects, route_samples,
    sampled_centerline, strip_polygon)
from avoidance_gazebo.s_curve_layout_validation import validate_s_layout


ROOT = Path(__file__).parents[1]
WORKSPACE = ROOT.parents[1]


def test_centerline_is_c2_at_straight_curve_joins():
    epsilon = 1.0e-5
    for join in (4.0, 26.0):
        center = center_pose_x(join)
        near = center_pose_x(join+epsilon if join == 4.0 else join-epsilon)
        assert center.y == pytest.approx(0.0, abs=1.0e-12)
        assert near.yaw == pytest.approx(0.0, abs=1.0e-7)
        assert near.curvature == pytest.approx(0.0, abs=1.0e-5)


def test_course_curvature_keeps_reference_steering_below_15_degrees():
    samples = sampled_centerline()
    maximum = max(abs(pose.curvature) for pose in samples)
    assert math.degrees(math.atan(WHEELBASE*maximum)) < 15.0
    assert 1.0/maximum > WHEELBASE/math.tan(math.radians(25.0))


def test_offset_strips_have_constant_width_and_no_self_intersection():
    samples = sampled_centerline()
    assert not polygon_self_intersects(strip_polygon(
        samples, -ROAD_HALF_WIDTH, ROAD_HALF_WIDTH))
    assert not polygon_self_intersects(strip_polygon(
        samples, CURB_INNER_OFFSET, CURB_INNER_OFFSET+0.15))
    for pose in samples[::20]:
        left = offset_point(pose, ROAD_HALF_WIDTH)
        right = offset_point(pose, -ROAD_HALF_WIDTH)
        assert math.dist(left, right) == pytest.approx(2.50, abs=1.0e-9)


def test_course_end_cap_is_outside_front_lidar_roi_at_route_goal():
    goal_world_x = route_samples()[-1].x
    assert WORLD_LENGTH-goal_world_x > 12.0


def test_s_world_is_continuous_polyline_and_has_tangent_obstacles():
    world = (ROOT/'worlds/s_curve_avoidance.sdf').read_text(encoding='utf-8')
    assert 'C2 analytic centerline' in world
    assert world.count('<polyline>') == 8
    assert len(re.findall(r'<model name="obstacle_[12]">', world)) == 2
    assert world.count('<size>0.65 0.39 0.15</size>') == 4


def test_s_route_has_exact_schema_spacing_and_odom_origin():
    path = WORKSPACE/'routes/s_curve_reference.csv'
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == ('index', 'timestamp', 'latitude', 'longitude',
                              'x_m', 'y_m', 'yaw', 'direction', 'mode',
                              'drive_level')
    assert [int(row['index']) for row in rows] == list(range(len(rows)))
    assert float(rows[0]['x_m']) == pytest.approx(0.0)
    assert float(rows[0]['y_m']) == pytest.approx(0.0)
    assert {row['direction'] for row in rows} == {'1'}
    assert {row['mode'] for row in rows} == {'NORMAL'}
    assert {row['drive_level'] for row in rows} == {'2.00'}
    points = [(float(row['x_m']), float(row['y_m'])) for row in rows]
    spacing = [math.dist(a, b) for a, b in zip(points, points[1:])]
    assert max(abs(value-0.10) for value in spacing[:-1]) < 1.0e-5
    assert all(math.isfinite(float(row[key])) for row in rows
               for key in ('x_m', 'y_m', 'yaw'))


def test_s_layout_is_reproducible_opposite_side_and_non_repeating(tmp_path):
    assert same_s_positions(generate_s_layout(42), generate_s_layout(42))
    state = tmp_path/'last_s.yaml'
    layouts = [generate_s_layout(-1, state) for _ in range(10)]
    assert all(not same_s_positions(a, b) for a, b in zip(layouts, layouts[1:]))
    for layout in layouts:
        assert 7.5 <= layout.first_s <= 8.0
        assert 10.5 <= layout.second_s-layout.first_s <= 11.0
        assert {d for _s, d in layout.ordered} == {-0.78, 0.78}


def test_historical_failure_seed_is_reproduced_and_now_feasible():
    seed = 7723672268357995373
    layout = generate_s_layout(seed)
    assert layout.first_side == 'LEFT'
    assert layout.first_s == pytest.approx(7.856342)
    assert layout.second_s == pytest.approx(18.590263)
    assert validate_s_layout(layout, WORKSPACE/'routes/s_curve_reference.csv')


def test_layout_sampling_stops_after_configured_attempt_limit():
    with pytest.raises(RuntimeError, match='3 attempts'):
        generate_s_layout(9, validator=lambda _layout: False, max_attempts=3)


def test_fixed_seed_resampling_remains_deterministic():
    def reject_first(layout):
        return layout.sampling_attempts >= 2
    first = generate_s_layout(9, validator=reject_first, max_attempts=3)
    second = generate_s_layout(9, validator=reject_first, max_attempts=3)
    assert first.sampling_attempts == 2
    assert same_s_positions(first, second)


def test_second_right_actual_lidar_surface_has_adaptive_candidate():
    route, _warnings = load_route_csv(WORKSPACE/'routes/s_curve_reference.csv')
    stopped = Pose2(14.875822, -1.154021, -0.120667873)
    # Reproduced track 20: narrow path-facing surface, not obstacle centre.
    observed = Box2(
        17.7677537-0.325, 17.7677537+0.325,
        -1.34078835-0.04, -1.34078835+0.04)
    result = plan_candidates(
        route, stopped, observed, 1.095, -1.095,
        target_fractions=(), lateral_target_samples=7,
        return_lengths=(2.0, 2.5, 3.0, 3.5, 4.0),
        rejoin_straight_extension=2.5,
        obstacle_surface_observation=True,
        obstacle_length=0.65, obstacle_width=0.39,
        expected_obstacle_lateral_center=0.78)
    assert result.selected is not None
    assert result.selected.max_steering_rad <= math.radians(25.0)
    assert result.selected.obstacle_clearance >= 0.20
    assert result.selected.curb_clearance > 0.0


def test_second_left_outer_side_surface_is_recovered_and_has_candidate():
    route, _warnings = load_route_csv(WORKSPACE/'routes/s_curve_reference.csv')
    stopped = Pose2(14.313678, -1.048185, -0.216043258)
    s, d = 18.32, 0.816
    x, y, yaw = interpolate_route(route, route_lengths(route), s)
    x -= d*math.sin(yaw)
    y += d*math.cos(yaw)
    # Reproduced track 21 is an outer/side return: treating d=+0.816 as the
    # inner face would extend the 0.39 m body through the +1.095 m curb.
    half_x = 0.5*(0.65*abs(math.cos(yaw))+0.08*abs(math.sin(yaw)))
    half_y = 0.5*(0.65*abs(math.sin(yaw))+0.08*abs(math.cos(yaw)))
    observed = Box2(x-half_x, x+half_x, y-half_y, y+half_y)
    result = plan_candidates(
        route, stopped, observed, 1.095, -1.095,
        target_fractions=(), lateral_target_samples=7,
        return_lengths=(2.0, 2.5, 3.0, 3.5, 4.0),
        rejoin_straight_extension=2.5,
        obstacle_surface_observation=True,
        obstacle_length=0.65, obstacle_width=0.39,
        expected_obstacle_lateral_center=0.78)
    assert result.selected is not None
    assert result.selected.max_steering_rad <= math.radians(25.0)
    assert result.selected.obstacle_clearance >= 0.20
    assert result.selected.curb_clearance > 0.0


def test_s_launch_defaults_and_single_common_planner():
    simulation = (ROOT/'launch/s_curve_avoidance.launch.py').read_text()
    planning = (ROOT/'launch/s_curve_planning.launch.py').read_text()
    assert "'spawn_obstacles', default_value='true'" in simulation
    assert "'obstacle_seed', default_value='-1'" in planning
    assert "'replan_trigger_distance_m', default_value='2.0'" in planning
    assert "'max_steering_deg', default_value='25.0'" in planning
    assert planning.count("executable='route_follower'") == 1
    assert planning.count("executable='avoidance_coordinator'") == 1
