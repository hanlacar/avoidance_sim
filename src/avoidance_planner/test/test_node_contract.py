import math
from types import SimpleNamespace
from unittest.mock import Mock

from avoidance_planner.coordinator_node import AvoidanceCoordinator, PLANNER_STATES
from pathlib import Path
import yaml


def test_required_planner_states_are_present():
    required = {
        'FOLLOWING_CSV', 'OBSTACLE_CANDIDATE', 'OBSTACLE_CONFIRMED',
        'REPLAN_REQUIRED', 'STOPPING', 'STOPPED_FOR_PLANNING',
        'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
        'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES', 'PATH_READY',
        'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP', 'TIME_RESET_STOP', 'ERROR'}
    assert set(PLANNER_STATES) == required


def test_planner_latches_replan_but_does_not_compete_for_mcu_outputs():
    fake = SimpleNamespace(
        debounce=SimpleNamespace(latched=True), state='PATH_READY',
        replan_pub=Mock())
    AvoidanceCoordinator._publish_stop_contract(fake)
    assert fake.replan_pub.publish.call_args.args[0].data is True


def test_planner_has_no_cmd_vel_publisher_contract():
    source = __import__('inspect').getsource(AvoidanceCoordinator.__init__)
    assert "create_publisher(Twist" not in source


def test_dynamic_to_static_transition_cannot_return_to_latched_csv_deadlock():
    source = __import__('inspect').getsource(AvoidanceCoordinator._tick)
    stationary_block = source.split(
        "self.selected_track.state == STATIC_OBSTACLE):", 1)[1].split(
            "if self.state in ('PATH_READY'", 1)[0]
    assert "self._set_state('REPLAN_REQUIRED'" in stationary_block
    assert "self._set_state('STOPPING'" in stationary_block
    assert "self._set_state('FOLLOWING_CSV'" not in stationary_block


def test_rejoin_extension_gives_rate_limited_follower_settling_distance():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config' /
                             'avoidance_planner.yaml').read_text())
    assert config['avoidance_coordinator']['ros__parameters'][
        'rejoin_straight_extension_m'] >= 2.0


def test_tf_drop_diagnostics_separate_startup_and_runtime():
    fake = SimpleNamespace(
        scan_drops=0, tf_failures=0, last_tf_error='', tf_ready=False,
        startup_tf_drop_count=0, runtime_tf_drop_count=0,
        future_extrapolation_count=0, past_extrapolation_count=0,
        missing_frame_count=0)
    AvoidanceCoordinator._record_tf_drop(fake, 'future extrapolation', 'future')
    assert fake.startup_tf_drop_count == 1
    assert fake.runtime_tf_drop_count == 0
    assert fake.future_extrapolation_count == 1
    fake.tf_ready = True
    AvoidanceCoordinator._record_tf_drop(fake, 'past extrapolation', 'past')
    assert fake.runtime_tf_drop_count == 1
    assert fake.past_extrapolation_count == 1
    assert fake.scan_drops == fake.tf_failures == 2


def test_max_steering_deg_default_is_twenty_five():
    fake = SimpleNamespace(p={'wheelbase_m': 0.77, 'max_steering_deg': 25.0,
                              'left_curb_inner_y_m': 1.095,
                              'right_curb_inner_y_m': -1.095})
    AvoidanceCoordinator._validate_parameters(fake)


def test_max_steering_deg_above_twenty_five_is_rejected():
    fake = SimpleNamespace(p={'wheelbase_m': 0.77, 'max_steering_deg': 25.1,
                              'left_curb_inner_y_m': 1.095,
                              'right_curb_inner_y_m': -1.095})
    try:
        AvoidanceCoordinator._validate_parameters(fake)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_completed_track_is_marked_passed_and_excluded_from_status():
    fake = SimpleNamespace(
        selected_track=SimpleNamespace(track_id=7), avoidance_started=True,
        debounce=SimpleNamespace(), replan_pub=Mock(),
        p={'confirmation_frames': 3},
        passed_track_ids=set(), _set_state=Mock())
    AvoidanceCoordinator._control_source(fake, SimpleNamespace(data='GPS'))
    assert fake.passed_track_ids == {7}


def test_obstacle_statuses_label_passed_track_and_order_by_x():
    track_a = SimpleNamespace(track_id=1, x=2.0, state='STATIC_OBSTACLE')
    track_b = SimpleNamespace(track_id=2, x=8.0, state='STATIC_OBSTACLE')
    fake = SimpleNamespace(
        tracks=(track_a, track_b), selected_track=track_b,
        avoidance_started=True, passed_track_ids={1})
    from avoidance_planner.perception import STATIC_OBSTACLE  # noqa: F401
    statuses = AvoidanceCoordinator._obstacle_statuses(fake)
    by_id = {entry['track_id']: entry for entry in statuses}
    assert by_id[1]['status'] == 'PASSED' and by_id[1]['label'] == 'obstacle_1'
    assert by_id[2]['status'] == 'AVOIDING' and by_id[2]['label'] == 'obstacle_2'
