import csv
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from std_msgs.msg import Bool

from avoidance_route.route_follower import RouteFollower
from avoidance_route.route_following import (
    REQUIRED_COLUMNS, RouteError, Waypoint, avoidance_rejoin_ready,
    compute_control, goal_reached,
    limit_steering, load_route_csv, lookahead_point, nearest_projection,
    pure_pursuit_steering, safety_reason, start_pose_matches,
    terminal_curvature)


HEADER = ('index', 'x_m', 'y_m', 'yaw', 'direction', 'drive_level')


def write_csv(path, rows, header=HEADER):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def straight_points():
    return tuple(Waypoint(i, float(i), 0.0, 0.0, 1, 1.0) for i in range(6))


def test_csv_normal_parsing(tmp_path):
    path = tmp_path / 'route.csv'
    write_csv(path, [(0, 0, 0, 0, 1, 1), (1, 1, 0, 0, 1, 1)])
    points, warnings = load_route_csv(path)
    assert len(points) == 2 and warnings == ()


def test_empty_csv_rejected(tmp_path):
    path = tmp_path / 'empty.csv'; path.write_text('', encoding='utf-8')
    with pytest.raises(RouteError, match='empty'):
        load_route_csv(path)


def test_header_only_csv_rejected(tmp_path):
    path = tmp_path / 'header.csv'; write_csv(path, [])
    with pytest.raises(RouteError, match='at least two'):
        load_route_csv(path)


def test_malformed_row_is_skipped(tmp_path):
    path = tmp_path / 'bad.csv'
    write_csv(path, [(0, 0, 0, 0, 1, 1), ('bad', 'x', 0, 0, 1, 1),
                     (2, 2, 0, 0, 1, 1)])
    points, warnings = load_route_csv(path)
    assert len(points) == 2 and len(warnings) == 1


def test_nan_inf_coordinates_are_rejected(tmp_path):
    path = tmp_path / 'nonfinite.csv'
    write_csv(path, [(0, 0, 0, 0, 1, 1), (1, 'nan', 0, 0, 1, 1),
                     (2, 2, 'inf', 0, 1, 1), (3, 3, 0, 0, 1, 1)])
    points, warnings = load_route_csv(path)
    assert [point.index for point in points] == [0, 3]
    assert len(warnings) == 2


def test_nearest_point_projection():
    result = nearest_projection(straight_points(), 2.4, 0.25)
    assert result.segment == 2 and result.x == pytest.approx(2.4)
    assert result.distance == pytest.approx(0.25)


def test_lookahead_point_selection():
    points = straight_points()
    projection = nearest_projection(points, 1.25, 0.0)
    x, y, index = lookahead_point(points, projection, 1.5)
    assert (x, y, index) == pytest.approx((2.75, 0.0, 3))


def test_lookahead_extends_terminal_heading_without_moving_real_goal():
    points = straight_points()
    projection = nearest_projection(points, 4.8, 0.04)
    x, y, index = lookahead_point(points, projection, 1.2)
    assert (x, y, index) == pytest.approx((6.0, 0.0, 5))
    assert points[-1].x == pytest.approx(5.0)


def test_curved_terminal_lookahead_continues_csv_curvature():
    radius = 5.0
    points = tuple(Waypoint(i, radius*math.sin(i*0.02),
                            radius*(1.0-math.cos(i*0.02)), i*0.02,
                            1, 2.0) for i in range(6))
    curvature = terminal_curvature(points)
    assert curvature == pytest.approx(1.0/radius, rel=1.0e-4)
    projection = nearest_projection(points, points[-1].x, points[-1].y)
    x, y, _ = lookahead_point(points, projection, 1.0)
    expected_yaw = points[-1].yaw+curvature
    assert x == pytest.approx(points[-1].x+(math.sin(expected_yaw)-
                              math.sin(points[-1].yaw))/curvature)
    assert y > points[-1].y


def test_straight_route_steering_near_zero():
    assert pure_pursuit_steering(0, 0, 0, 2, 0, 0.77, math.radians(20)) == pytest.approx(0)


def test_left_curve_generates_left_steering():
    assert pure_pursuit_steering(0, 0, 0, 2, 1, 0.77, math.radians(20)) > 0


def test_right_curve_generates_right_steering():
    assert pure_pursuit_steering(0, 0, 0, 2, -1, 0.77, math.radians(20)) < 0


def test_steering_is_clamped_to_twenty_degrees():
    limit = math.radians(20)
    assert pure_pursuit_steering(0, 0, 0, 0.01, 1, 0.77, limit) == pytest.approx(limit)
    assert pure_pursuit_steering(0, 0, 0, 0.01, -1, 0.77, limit) == pytest.approx(-limit)


def test_steering_change_rate_is_limited():
    assert limit_steering(0.0, math.radians(20), math.radians(2)) == pytest.approx(math.radians(2))


def test_start_pose_mismatch_refuses_start():
    matched, position_error, yaw_error = start_pose_matches(
        1.0, 0.0, math.radians(20), straight_points()[0], 0.30, math.radians(10))
    assert not matched and position_error == pytest.approx(1.0)
    assert yaw_error == pytest.approx(math.radians(20))


def test_odometry_timeout_requests_stop():
    assert safety_reason(0.51, 0.50, 0.0, 0.60) == 'ODOMETRY_TIMEOUT'


def test_route_deviation_requests_stop():
    assert safety_reason(0.1, 0.50, 0.61, 0.60) == 'ROUTE_DEVIATION'


def test_goal_tolerance_reaches_goal():
    assert goal_reached(4.85, 0.0, straight_points()[-1], 0.20)
    assert not goal_reached(4.79, 0.0, straight_points()[-1], 0.20)


def test_shutdown_stop_command_is_exact_zero():
    fake = SimpleNamespace(speed=1.0, steering=0.2, cmd_pub=Mock(), steering_pub=Mock())
    RouteFollower._publish_stop(fake)
    command = fake.cmd_pub.publish.call_args.args[0]
    assert command.linear.x == 0.0 and command.angular.z == 0.0
    assert fake.steering_pub.publish.call_args.args[0].data == 0.0


def _time_control_fake(now_ns, last_ns):
    from rclpy.time import Time
    now = Time(nanoseconds=now_ns)
    return SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        last_control_time=(None if last_ns is None else Time(nanoseconds=last_ns)),
        last_clock_time=None, last_control_dt=999.0, odom=None,
        state='FOLLOWING', p={
            'clock_reset_threshold_s': 1.0,
            'clock_forward_jump_threshold_s': 2.0,
        }, _publish_stop=Mock(), _publish_status=Mock(),
        _handle_time_jump=Mock())


def test_control_with_none_time_publishes_exactly_one_safe_cycle():
    fake = _time_control_fake(0, None)
    RouteFollower._control(fake)
    fake._publish_stop.assert_called_once()
    assert fake.last_control_time.nanoseconds == 0
    assert fake.last_control_dt is None


def test_second_control_callback_has_valid_bounded_dt():
    fake = _time_control_fake(1_050_000_000, 1_000_000_000)
    RouteFollower._control(fake)
    assert fake.last_control_dt == pytest.approx(0.05)
    fake._publish_stop.assert_called_once()  # no odom remains fail-safe


def test_large_positive_delta_is_clamped_without_stale_command():
    fake = _time_control_fake(2_000_000_000, 1_000_000_000)
    RouteFollower._control(fake)
    assert fake.last_control_dt == pytest.approx(0.2)
    fake._publish_stop.assert_called_once()


@pytest.mark.parametrize('now_ns,last_ns', [
    (1_000_000_000, 1_000_000_000),
    (900_000_000, 1_000_000_000),
])
def test_repeated_or_backward_time_enters_safe_time_jump_path(now_ns, last_ns):
    fake = _time_control_fake(now_ns, last_ns)
    fake._handle_time_jump.side_effect = lambda *_args: fake._publish_stop()
    RouteFollower._control(fake)
    fake._handle_time_jump.assert_called_once()
    fake._publish_stop.assert_called_once()


def test_time_reset_recovery_requires_all_inputs_path_and_exact_tf():
    token = object()
    fake = SimpleNamespace(
        time_reset_is_large=False,
        time_reset_resume_state='FOLLOWING_AVOIDANCE',
        odom_receive_time=token, last_scan_receive_time=token,
        last_rear_scan_receive_time=token, selected_path_receive_time=None,
        _tf_is_valid=Mock(return_value=True))
    assert not RouteFollower._time_reset_recovery_ready(fake)
    fake.selected_path_receive_time = token
    assert RouteFollower._time_reset_recovery_ready(fake)
    fake._tf_is_valid.return_value = False
    assert not RouteFollower._time_reset_recovery_ready(fake)


def test_large_simulation_reset_never_auto_recovers():
    fake = SimpleNamespace(time_reset_is_large=True)
    assert not RouteFollower._time_reset_recovery_ready(fake)


def test_compute_control_reports_indices_and_finite_yaw_rate():
    result = compute_control(straight_points(), 0, 0, 0, 0, 0.8, 0.77,
                             math.radians(20), 0.0, math.radians(2))
    assert result.nearest_index == 0 and result.lookahead_index == 1
    assert result.steering_rad == pytest.approx(0.0)
    assert math.isfinite(result.angular_rate)


def test_manual_teleop_publisher_is_reported_as_conflict():
    endpoint = SimpleNamespace(node_name='manual_teleop', node_namespace='/')
    fake = SimpleNamespace(
        p={'cmd_topic': '/cmd_vel'}, get_name=lambda: 'route_follower',
        get_publishers_info_by_topic=lambda _topic: [endpoint])
    assert RouteFollower._conflicting_publishers(fake) == ['/manual_teleop']


def test_replan_request_latches_exact_stop_without_restart():
    fake = SimpleNamespace(
        p={'replan_stop_enabled': True}, state='FOLLOWING', reason='',
        replan_ignore_until_clear=False,
        start_requested=True, _publish_stop=Mock(), _close_actual_log=Mock(),
        _publish_status=Mock(), get_logger=lambda: Mock())
    RouteFollower._replan_required(fake, Bool(data=True))
    assert fake.state == 'STOPPED_FOR_REPLAN'
    assert fake.reason == 'REPLAN_REQUIRED' and not fake.start_requested
    fake._publish_stop.assert_called_once()
    fake._close_actual_log.assert_not_called()


def test_completed_track_replan_is_ignored_until_false_acknowledgement():
    fake = SimpleNamespace(
        p={'replan_stop_enabled': True}, state='FOLLOWING', reason='',
        replan_ignore_until_clear=True, _publish_stop=Mock(),
        _close_actual_log=Mock(), _publish_status=Mock(), get_logger=lambda: Mock())
    RouteFollower._replan_required(fake, Bool(data=True))
    assert fake.state == 'FOLLOWING'
    RouteFollower._replan_required(fake, Bool(data=False))
    assert not fake.replan_ignore_until_clear


def test_gps_and_lidar_outputs_are_never_cross_paired():
    fake = SimpleNamespace(
        p={'gps_drive_level': 2.0}, gps_drive_pub=Mock(), gps_wheel_pub=Mock(),
        lidar_drive_pub=Mock(), lidar_wheel_pub=Mock(), control_source_pub=Mock(),
        control_source='STOP', _publish_stop=Mock())
    RouteFollower._publish_source_pair(fake, 'GPS', 7.4)
    assert fake.gps_drive_pub.publish.call_args.args[0].data == 2.0
    assert fake.gps_wheel_pub.publish.call_args.args[0].data == 7
    assert fake.lidar_drive_pub.publish.call_args.args[0].data == 0.0
    assert fake.lidar_wheel_pub.publish.call_args.args[0].data == 0
    RouteFollower._publish_source_pair(fake, 'LIDAR', -6.6)
    assert fake.gps_drive_pub.publish.call_args.args[0].data == 0.0
    assert fake.gps_wheel_pub.publish.call_args.args[0].data == 0
    assert fake.lidar_drive_pub.publish.call_args.args[0].data == 1.0
    assert fake.lidar_wheel_pub.publish.call_args.args[0].data == -7


def test_follower_rejects_unclamped_over_limit_control():
    sharp = (Waypoint(0, 0.0, 0.0, 0.0, 1, 1.0),
             Waypoint(1, 0.01, 1.0, math.pi/2, 1, 1.0))
    with pytest.raises(RouteError, match='exceeds'):
        compute_control(sharp, 0, 0, 0, 0, 0.05, 0.77,
                        math.radians(20), 0.0, math.radians(2))


def test_attempt_avoidance_start_succeeds_when_path_ready(tmp_path):
    from nav_msgs.msg import Path as PathMsg
    from rclpy.time import Time
    clock = SimpleNamespace(now=lambda: Time(nanoseconds=1_000_000_000))
    fake = SimpleNamespace(
        state='WAITING_FOR_AVOIDANCE_START', planner_state='PATH_READY',
        selected_points=(Waypoint(0, 0.0, 0.0, 0.0, 1, 1.0),
                         Waypoint(1, 1.0, 0.0, 0.0, 1, 1.0)),
        selected_path_receive_time=Time(nanoseconds=0),
        p={'selected_path_timeout_s': 300.0}, odom=Mock(),
        get_clock=lambda: clock, selected_track_id=3,
        avoidance_segment=99, steering=1.0, speed=1.0,
        avoidance_track_id=-1, avoidance_actual_path=PathMsg(),
        control_source='STOP',
        _publish_stop=Mock(), get_logger=lambda: Mock(),
        _pose=lambda: (0.0, 0.0, 0.0))
    ok, message = RouteFollower._attempt_avoidance_start(fake)
    assert ok and fake.state == 'FOLLOWING_AVOIDANCE'
    assert fake.control_source == 'LIDAR'
    assert fake.avoidance_track_id == 3
    assert fake.avoidance_segment == 0


def test_attempt_avoidance_start_fails_without_path_ready():
    fake = SimpleNamespace(
        state='WAITING_FOR_AVOIDANCE_START', planner_state='PLANNING',
        selected_points=(), _publish_stop=Mock())
    ok, message = RouteFollower._attempt_avoidance_start(fake)
    assert not ok and 'unavailable' in message


def test_auto_start_holds_then_starts_without_manual_service():
    from rclpy.time import Time
    calls = []
    fake = SimpleNamespace(
        p={'auto_start_avoidance': True, 'path_ready_hold_sec': 0.30},
        state='WAITING_FOR_AVOIDANCE_START',
        avoidance_ready_since=Time(nanoseconds=0),
        avoidance_auto_start_failure_logged=False,
        _attempt_avoidance_start=lambda: calls.append(1) or (True, 'ok'))
    RouteFollower._try_auto_start_avoidance(fake, Time(nanoseconds=100_000_000))
    assert calls == []
    RouteFollower._try_auto_start_avoidance(fake, Time(nanoseconds=400_000_000))
    assert calls == [1]
    assert fake.avoidance_ready_since is None


def test_auto_start_disabled_never_calls_attempt():
    from rclpy.time import Time
    fake = SimpleNamespace(
        p={'auto_start_avoidance': False, 'path_ready_hold_sec': 0.30},
        state='WAITING_FOR_AVOIDANCE_START',
        avoidance_ready_since=Time(nanoseconds=0),
        _attempt_avoidance_start=Mock())
    RouteFollower._try_auto_start_avoidance(fake, Time(nanoseconds=10_000_000_000))
    fake._attempt_avoidance_start.assert_not_called()


def test_avoidance_rejoin_requires_forward_continuous_csv_join():
    goal = Waypoint(100, 8.83, 0.0, 0.0, 1, 1.0)
    ready, remaining, lateral, yaw_error = avoidance_rejoin_ready(
        8.18, -0.004, math.radians(-9.5), goal, 0.8, 0.15,
        math.radians(12.0))
    assert ready and remaining == pytest.approx(0.65)
    assert lateral == pytest.approx(0.004)
    assert yaw_error == pytest.approx(math.radians(9.5))
    assert not avoidance_rejoin_ready(
        8.18, -0.004, math.radians(-12.1), goal, 0.8, 0.15,
        math.radians(12.0))[0]


def test_rejoin_allows_bounded_terminal_overshoot_only_when_aligned():
    goal = Waypoint(100, 8.0, 0.0, 0.0, 1, 1.0)
    assert avoidance_rejoin_ready(
        8.25, 0.04, math.radians(1.0), goal, 0.8, 0.10,
        math.radians(2.0), max_overshoot=0.5)[0]
    assert not avoidance_rejoin_ready(
        8.51, 0.04, math.radians(1.0), goal, 0.8, 0.10,
        math.radians(2.0), max_overshoot=0.5)[0]
    assert not avoidance_rejoin_ready(
        8.25, 0.11, math.radians(1.0), goal, 0.8, 0.10,
        math.radians(2.0), max_overshoot=0.5)[0]


def test_rejoin_alignment_zone_provides_two_metres_to_settle():
    config = yaml.safe_load((__import__('pathlib').Path(__file__).parents[1] /
                             'config' / 'route_follower.yaml').read_text())
    assert config['route_follower']['ros__parameters'][
        'avoidance_rejoin_alignment_distance_m'] >= 2.0
