from types import SimpleNamespace
from unittest.mock import Mock

from avoidance_planner.coordinator_node import AvoidanceCoordinator, PLANNER_STATES


def test_required_planner_states_are_present():
    required = {
        'FOLLOWING_CSV', 'OBSTACLE_CANDIDATE', 'OBSTACLE_CONFIRMED',
        'REPLAN_REQUIRED', 'STOPPING', 'STOPPED_FOR_PLANNING',
        'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
        'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES', 'PATH_READY',
        'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP', 'ERROR'}
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
