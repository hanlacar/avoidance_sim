import csv
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_msgs.msg import Bool

from avoidance_route.route_follower import RouteFollower
from avoidance_route.route_following import (
    REQUIRED_COLUMNS, RouteError, Waypoint, compute_control, goal_reached,
    limit_steering, load_route_csv, lookahead_point, nearest_projection,
    pure_pursuit_steering, safety_reason, start_pose_matches)


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
        start_requested=True, _publish_stop=Mock(), _close_actual_log=Mock(),
        _publish_status=Mock(), get_logger=lambda: Mock())
    RouteFollower._replan_required(fake, Bool(data=True))
    assert fake.state == 'STOPPED_FOR_REPLAN'
    assert fake.reason == 'REPLAN_REQUIRED' and not fake.start_requested
    fake._publish_stop.assert_called_once()
    fake._close_actual_log.assert_called_once()
