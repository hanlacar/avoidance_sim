from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from std_msgs.msg import String

from avoidance_planner.collision_evaluator import (
    ReplanDebounce, evaluate_track_collision, risk_within_activation_distance)
from avoidance_planner.coordinator_node import AvoidanceCoordinator
from avoidance_planner.geometry import Pose2


def route(length=12.0, step=0.02):
    return tuple(Pose2(index*step, 0.0, 0.0)
                 for index in range(int(length/step)+1))


def obstacle(x=3.30, y=0.0):
    return SimpleNamespace(
        x=x, y=y, min_x=x-0.325, max_x=x+0.325,
        min_y=y-0.195, max_y=y+0.195)


def risk(item, pose=Pose2(0.0, 0.0, 0.0)):
    return evaluate_track_collision(
        route(), pose, item, 1.30, 0.78, 0.0, 0.20, 0.15,
        8.0, 0.50, 1.095, -1.095, 0.65, 0, 0.65, 0.39)


def test_mode_five_contract_uses_string_and_other_mode_resets():
    fake = SimpleNamespace(
        current_mode='5', p={'allowed_avoidance_modes': ['5']},
        _reset_avoidance=Mock(), operational_state='MONITORING',
        _set_state=Mock())
    fake._mode_allowed = lambda: AvoidanceCoordinator._mode_allowed(fake)
    AvoidanceCoordinator._mcu_mode(fake, String(data='4'))
    fake._reset_avoidance.assert_called_once_with('MCU_MODE_NOT_ALLOWED')


def test_three_distinct_lidar_confirmations_are_required():
    debounce = ReplanDebounce(3)
    assert not debounce.update(True, 7)
    assert not debounce.update(True, 7)
    assert debounce.update(True, 7)


def test_confirmation_resets_on_gap_or_different_risk():
    debounce = ReplanDebounce(3)
    debounce.update(True, 7)
    debounce.update(True, 7)
    assert not debounce.update(False)
    assert debounce.count == 0
    assert not debounce.update(True, 8)


def test_swept_footprint_excludes_path_side_obstacle():
    assert not risk(obstacle(y=1.60)).required


def test_collision_beyond_two_metres_is_not_activation_eligible():
    item = risk(obstacle(x=4.30))
    assert item.required and item.collision_path_distance > 2.0
    assert not risk_within_activation_distance(item, 2.0)


def test_obstacle_behind_front_bumper_is_excluded():
    item = risk(obstacle(x=-0.80))
    assert not item.required
    assert not risk_within_activation_distance(item, 2.0)


def test_front_bumper_path_distance_not_lidar_origin_range_drives_trigger():
    item = risk(obstacle(x=2.975))
    assert item.required
    # The existing swept test includes its configured 0.15 m longitudinal
    # safety inflation, so its first collision pose precedes the raw surface.
    assert item.collision_path_distance == pytest.approx(
        item.vehicle_front_surface_distance-0.15, abs=0.03)
    assert item.collision_path_distance != pytest.approx(
        item.lidar_surface_distance + 0.65)
    assert risk_within_activation_distance(item, 2.0)


def test_planning_failure_maps_to_active_safe_stop():
    fake = SimpleNamespace(
        state='VALIDATING_CANDIDATES', reason='', active=True,
        operational_state='PLANNING', avoidance_started=False,
        state_history=[], get_logger=lambda: Mock(), _publish_status=Mock(),
        _mode_allowed=lambda: True)
    AvoidanceCoordinator._set_state(fake, 'PATH_INFEASIBLE', 'no path')
    assert fake.active and fake.operational_state == 'SAFE_STOP'


def test_active_odom_timeout_enters_safe_stop_without_releasing_active():
    from rclpy.time import Time
    fake = SimpleNamespace(
        active=True, odom_receive_time=Time(nanoseconds=0),
        reference_receive_time=None, last_front_scan_time=None,
        state='STOPPING', debounce=ReplanDebounce(3),
        p={'odom_timeout_sec': 0.5, 'reference_path_timeout_sec': 0.0,
           'scan_timeout_sec': 0.3}, operational_state='STOPPING',
        get_clock=lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=1_000_000_000)),
        _set_state=Mock())
    AvoidanceCoordinator._watchdog(fake)
    assert fake.active and fake.operational_state == 'SAFE_STOP'
    fake._set_state.assert_called_once()


def test_active_publish_rate_and_mode_contract_are_configured():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config' /
                             'avoidance_planner.yaml').read_text())
    params = config['avoidance_coordinator']['ros__parameters']
    assert params['active_publish_rate_hz'] == 10.0
    assert params['mode_topic'] == '/mcu/current_mode'
    assert params['allowed_avoidance_modes'] == ['5']
    assert params['collision_confirmation_frames'] == 3
