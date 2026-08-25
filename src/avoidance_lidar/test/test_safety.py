import math

from avoidance_lidar.safety import LidarSafetyGate, clamp_steering


class Scan:
    angle_min = -math.pi
    angle_increment = math.radians(1.0)
    range_min = 0.10
    range_max = 10.0

    def __init__(self):
        self.ranges = [math.inf] * 361

    def set_range(self, angle_deg, distance):
        index = round((math.radians(angle_deg) - self.angle_min) /
                      self.angle_increment)
        self.ranges[index] = distance
        return self


def test_confirmed_obstacle_stops_and_clear_has_hysteresis():
    gate = LidarSafetyGate()
    blocked = Scan().set_range(0.0, 0.4)
    gate.update_scan(blocked)
    gate.update_scan(blocked)
    assert gate.should_stop
    clear = Scan().set_range(0.0, 0.8)
    gate.update_scan(clear)
    gate.update_scan(clear)
    assert gate.should_stop
    gate.update_scan(clear)
    assert not gate.should_stop


def test_invalid_scan_and_timeout_fail_safe():
    gate = LidarSafetyGate(scan_timeout_sec=0.5)
    gate.update_scan(Scan())
    assert gate.state == gate.INVALID_SCAN and gate.should_stop
    gate.update_scan(Scan().set_range(0.0, 1.0))
    gate.update_timeout(0.51)
    assert gate.state == gate.LIDAR_TIMEOUT and gate.should_stop


def test_emergency_distance_stops_on_first_scan():
    gate = LidarSafetyGate(stop_confirm_scans=5, emergency_stop_distance_m=0.3)
    gate.update_scan(Scan().set_range(0.0, 0.3))
    assert gate.state == gate.OBSTACLE_STOP and gate.should_stop


def test_filter_preserves_units_and_wheel():
    gate = LidarSafetyGate(stop_confirm_scans=1)
    gate.update_scan(Scan().set_range(0.0, 0.3))
    assert gate.filter_command(2.0, -27) == (0.0, -27)


def test_final_steering_saturation_never_exceeds_27_degrees():
    assert clamp_steering(80, 27) == 27
    assert clamp_steering(-80, 27) == -27
    assert clamp_steering(12, 25) == 12
