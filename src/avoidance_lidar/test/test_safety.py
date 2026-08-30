import math
from types import SimpleNamespace

from avoidance_lidar.safety import LidarSafetyGate, clamp_steering
from avoidance_lidar.safety_node import LidarSafetyNode


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
    assert gate.state == gate.CLEAR and not gate.should_stop
    invalid = Scan()
    invalid.ranges = [math.nan] * len(invalid.ranges)
    gate.update_scan(invalid)
    assert gate.state == gate.INVALID_SCAN and gate.should_stop
    gate.update_scan(Scan().set_range(0.0, 1.0))
    gate.update_timeout(0.51)
    assert gate.state == gate.LIDAR_TIMEOUT and gate.should_stop


def test_speed_increases_roi_and_physical_stopping_distance():
    gate = LidarSafetyGate()
    gate.update_scan(Scan(), ego_speed_mps=2.0)
    assert gate.effective_roi_length_m == 3.0
    assert gate.effective_stop_distance_m == 2.6


def test_corridor_ignores_side_returns_but_follows_steering_arc():
    gate = LidarSafetyGate(corridor_width_m=0.20, wheelbase_m=0.77,
                           stop_confirm_scans=1)
    steering = 27.0
    curvature = math.tan(math.radians(steering)) / 0.77
    progress = 1.0
    x = math.sin(curvature * progress) / curvature
    y = (1.0 - math.cos(curvature * progress)) / curvature
    scan = Scan().set_range(
        math.degrees(math.atan2(y, x)), math.hypot(x, y))
    gate.update_scan(scan, steering_deg=0.0)
    assert gate.state == gate.CLEAR
    gate.update_scan(scan, steering_deg=steering)
    assert gate.front_min_distance < 1.05


def test_ttc_requires_confirmed_closing_trend():
    gate = LidarSafetyGate(stop_confirm_scans=5, ttc_stop_sec=1.5,
                           ttc_confirm_scans=2, roi_min_length_m=3.0)
    gate.update_scan(Scan().set_range(0.0, 2.0), timestamp=1.0)
    gate.update_scan(Scan().set_range(0.0, 1.8), timestamp=1.1)
    assert gate.state == gate.CLEAR
    gate.update_scan(Scan().set_range(0.0, 1.6), timestamp=1.2)
    assert gate.state == gate.CLEAR
    gate.update_scan(Scan().set_range(0.0, 1.4), timestamp=1.3)
    assert gate.state == gate.OBSTACLE_STOP


def test_risk_zones_are_diagnostic_only():
    gate = LidarSafetyGate(stop_confirm_scans=2)
    gate.update_scan(Scan().set_range(0.0, 1.3))
    assert gate.risk_level == 'CAUTION'
    gate.update_scan(Scan().set_range(0.0, 0.8))
    assert gate.risk_level == 'SLOW'
    gate.update_scan(Scan().set_range(0.0, 0.4))
    assert gate.risk_level == 'STOP'


def test_positive_infinity_is_a_valid_laserscan_no_return():
    gate = LidarSafetyGate()
    gate.update_scan(Scan())
    assert gate.state == gate.CLEAR
    assert math.isinf(gate.front_min_distance)


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


def test_safety_authority_requires_mode_five_active_fresh_heartbeat():
    from rclpy.time import Time
    now = Time(nanoseconds=1_000_000_000)
    fake = SimpleNamespace(
        current_mode='5', active=True,
        last_active_time=Time(nanoseconds=800_000_000),
        p={'allowed_avoidance_modes': ['5'], 'active_timeout_sec': 0.30})
    fake._mode_allowed = lambda: LidarSafetyNode._mode_allowed(fake)
    assert LidarSafetyNode._active_authority_valid(fake, now)
    fake.current_mode = '4'
    assert not LidarSafetyNode._active_authority_valid(fake, now)
