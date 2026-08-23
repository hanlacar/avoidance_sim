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


def test_lidar_mcu_contract_publishes_exact_zero_when_latched():
    fake = SimpleNamespace(
        debounce=SimpleNamespace(latched=True), state='PATH_READY',
        replan_pub=Mock(), lidar_drive_pub=Mock(), lidar_wheel_pub=Mock())
    AvoidanceCoordinator._publish_stop_contract(fake)
    assert fake.replan_pub.publish.call_args.args[0].data is True
    assert fake.lidar_drive_pub.publish.call_args.args[0].data == 0.0
    assert fake.lidar_wheel_pub.publish.call_args.args[0].data == 0


def test_planner_has_no_cmd_vel_publisher_contract():
    source = __import__('inspect').getsource(AvoidanceCoordinator.__init__)
    assert "create_publisher(Twist" not in source
