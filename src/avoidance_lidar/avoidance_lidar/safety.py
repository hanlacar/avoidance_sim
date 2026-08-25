"""Pure LaserScan safety gate used by the vehicle command layer."""

import math


def clamp_steering(steering_deg, limit_deg=27):
    """Return an integer steering command within the MCU contract."""
    limit = min(27, max(0, int(limit_deg)))
    return max(-limit, min(limit, int(steering_deg)))


class LidarSafetyGate:
    """Fail-safe stop gate; planning and classification stay elsewhere."""

    WAIT_SCAN = 'WAIT_SCAN'
    CLEAR = 'CLEAR'
    OBSTACLE_STOP = 'OBSTACLE_STOP'
    INVALID_SCAN = 'INVALID_SCAN'
    LIDAR_TIMEOUT = 'LIDAR_TIMEOUT'

    def __init__(self, stop_distance_m=0.50, resume_distance_m=0.60,
                 front_sector_deg=90.0, min_valid_range_m=0.05,
                 stop_confirm_scans=2, clear_confirm_scans=3,
                 scan_timeout_sec=0.5, stop_on_scan_timeout=True,
                 emergency_stop_distance_m=0.30,
                 corridor_width_m=1.20, wheelbase_m=0.77,
                 roi_min_length_m=1.50, roi_max_length_m=5.0,
                 roi_time_horizon_sec=1.50, reaction_time_sec=0.30,
                 max_deceleration_mps2=1.0, ttc_enabled=True,
                 ttc_stop_sec=1.50, ttc_confirm_scans=2,
                 closing_speed_alpha=0.50, max_closing_speed_mps=10.0):
        self.stop_distance_m = max(0.0, float(stop_distance_m))
        self.resume_margin_m = max(
            0.0, float(resume_distance_m) - self.stop_distance_m)
        self.front_sector_rad = math.radians(
            min(180.0, max(0.0, float(front_sector_deg))))
        self.min_valid_range_m = max(0.0, float(min_valid_range_m))
        self.stop_confirm_scans = max(1, int(stop_confirm_scans))
        self.clear_confirm_scans = max(1, int(clear_confirm_scans))
        self.scan_timeout_sec = max(0.0, float(scan_timeout_sec))
        self.stop_on_scan_timeout = bool(stop_on_scan_timeout)
        self.emergency_stop_distance_m = min(
            self.stop_distance_m,
            max(0.0, float(emergency_stop_distance_m)))
        self.corridor_width_m = max(0.05, float(corridor_width_m))
        self.wheelbase_m = max(0.05, float(wheelbase_m))
        self.roi_min_length_m = max(0.05, float(roi_min_length_m))
        self.roi_max_length_m = max(
            self.roi_min_length_m, float(roi_max_length_m))
        self.roi_time_horizon_sec = max(0.0, float(roi_time_horizon_sec))
        self.reaction_time_sec = max(0.0, float(reaction_time_sec))
        self.max_deceleration_mps2 = max(
            0.05, float(max_deceleration_mps2))
        self.ttc_enabled = bool(ttc_enabled)
        self.ttc_stop_sec = max(0.0, float(ttc_stop_sec))
        self.ttc_confirm_scans = max(1, int(ttc_confirm_scans))
        self.closing_speed_alpha = min(
            1.0, max(0.0, float(closing_speed_alpha)))
        self.max_closing_speed_mps = max(
            0.1, float(max_closing_speed_mps))

        self.state = self.WAIT_SCAN
        self.front_min_distance = math.inf
        self.effective_roi_length_m = self.roi_min_length_m
        self.effective_stop_distance_m = self.stop_distance_m
        self.closing_speed_mps = 0.0
        self.ttc_sec = math.inf
        self.risk_level = 'UNKNOWN'
        self.scan_received = False
        self._stop_count = 0
        self._clear_count = 0
        self._ttc_count = 0
        self._previous_distance = math.inf
        self._previous_timestamp = None

    def _update_dynamic_limits(self, ego_speed_mps):
        value = float(ego_speed_mps)
        speed = abs(value) if math.isfinite(value) else 0.0
        self.effective_roi_length_m = min(
            self.roi_max_length_m,
            max(self.roi_min_length_m, speed * self.roi_time_horizon_sec))
        braking_distance = (
            speed * self.reaction_time_sec +
            speed * speed / (2.0 * self.max_deceleration_mps2))
        # Keep the physical requirement visible even if the configured sensor
        # ROI is shorter; every obstacle inside that ROI then remains a threat.
        self.effective_stop_distance_m = max(
            self.stop_distance_m, braking_distance)

    def _corridor_coordinates(self, x, y, steering_deg):
        curvature = math.tan(math.radians(float(steering_deg))) / self.wheelbase_m
        if abs(curvature) < 1e-6:
            return x, abs(y)
        radius = 1.0 / curvature
        heading = math.atan2(curvature * x, 1.0 - curvature * y)
        if heading * curvature < 0.0:
            heading += math.copysign(2.0 * math.pi, curvature)
        progress = heading / curvature
        lateral_error = abs(math.hypot(x, y - radius) - abs(radius))
        return progress, lateral_error

    def _corridor_minimum(self, scan, steering_deg):
        minimum = math.inf
        valid_count = 0
        if (not getattr(scan, 'ranges', None) or
                not math.isfinite(float(scan.angle_min)) or
                not math.isfinite(float(scan.angle_increment)) or
                float(scan.angle_increment) == 0.0 or
                not math.isfinite(float(scan.range_min)) or
                not math.isfinite(float(scan.range_max)) or
                float(scan.range_max) <= float(scan.range_min)):
            return minimum, valid_count

        for index, value in enumerate(scan.ranges):
            distance = float(value)
            # LaserScan specifies +inf as a valid no-return measurement.
            if math.isinf(distance) and distance > 0.0:
                valid_count += 1
                continue
            if (not math.isfinite(distance) or distance == 0.0 or
                    distance < float(scan.range_min) or
                    distance > float(scan.range_max) or
                    distance < self.min_valid_range_m):
                continue
            valid_count += 1
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if abs(angle) > self.front_sector_rad:
                continue
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            progress, lateral_error = self._corridor_coordinates(
                x, y, steering_deg)
            if (0.0 <= progress <= self.effective_roi_length_m and
                    lateral_error <= self.corridor_width_m * 0.5):
                minimum = min(minimum, progress)
        return minimum, valid_count

    def _update_ttc(self, timestamp):
        self.ttc_sec = math.inf
        if (not self.ttc_enabled or timestamp is None or
                not math.isfinite(self.front_min_distance)):
            self.closing_speed_mps = 0.0
            self._ttc_count = 0
        elif (self._previous_timestamp is not None and
              math.isfinite(self._previous_distance)):
            dt = float(timestamp) - self._previous_timestamp
            if 0.02 <= dt <= 0.50:
                raw = (self._previous_distance - self.front_min_distance) / dt
                raw = min(self.max_closing_speed_mps, max(0.0, raw))
                alpha = self.closing_speed_alpha
                self.closing_speed_mps = (
                    alpha * raw + (1.0 - alpha) * self.closing_speed_mps)
                if self.closing_speed_mps > 0.01:
                    self.ttc_sec = (
                        self.front_min_distance / self.closing_speed_mps)
            else:
                self.closing_speed_mps = 0.0
                self._ttc_count = 0
        self._previous_distance = self.front_min_distance
        self._previous_timestamp = (
            float(timestamp) if timestamp is not None else None)
        if self.ttc_sec <= self.ttc_stop_sec:
            self._ttc_count += 1
        else:
            self._ttc_count = 0

    def _update_risk_level(self):
        if not math.isfinite(self.front_min_distance):
            self.risk_level = 'CLEAR'
        elif self.front_min_distance <= self.effective_stop_distance_m:
            self.risk_level = 'STOP'
        elif self.front_min_distance <= max(
                1.0, self.effective_stop_distance_m * 2.0):
            self.risk_level = 'SLOW'
        elif self.front_min_distance <= self.effective_roi_length_m:
            self.risk_level = 'CAUTION'
        else:
            self.risk_level = 'CLEAR'

    def update_scan(self, scan, timestamp=None, ego_speed_mps=0.0,
                    steering_deg=0.0):
        previous = self.state
        self.scan_received = True
        self._update_dynamic_limits(ego_speed_mps)
        self.front_min_distance, valid_count = self._corridor_minimum(
            scan, steering_deg)
        if valid_count == 0:
            self.state = self.INVALID_SCAN
            self.risk_level = 'INVALID'
            self._stop_count = self._clear_count = self._ttc_count = 0
            self._previous_distance = math.inf
            self._previous_timestamp = None
            return previous, self.state

        self._update_ttc(timestamp)
        self._update_risk_level()
        emergency = (
            self.front_min_distance <= self.emergency_stop_distance_m)
        ttc_threat = self._ttc_count >= self.ttc_confirm_scans
        distance_threat = (
            self.front_min_distance <= self.effective_stop_distance_m)

        if emergency:
            self.state = self.OBSTACLE_STOP
            self.risk_level = 'STOP'
            self._stop_count = self._clear_count = 0
            return previous, self.state
        if self.state == self.OBSTACLE_STOP:
            self._stop_count = 0
            resume_distance = (
                self.effective_stop_distance_m + self.resume_margin_m)
            if (self.front_min_distance >= resume_distance and
                    not ttc_threat):
                self._clear_count += 1
                if self._clear_count >= self.clear_confirm_scans:
                    self.state = self.CLEAR
                    self._clear_count = 0
            else:
                self._clear_count = 0
            return previous, self.state

        self._clear_count = 0
        if distance_threat or ttc_threat:
            self._stop_count += 1
            if self._stop_count >= self.stop_confirm_scans or ttc_threat:
                self.state = self.OBSTACLE_STOP
                self.risk_level = 'STOP'
                self._stop_count = 0
        else:
            self._stop_count = 0
            self.state = self.CLEAR
        return previous, self.state

    def update_timeout(self, scan_age_sec):
        previous = self.state
        if self.scan_received and float(scan_age_sec) > self.scan_timeout_sec:
            self.state = self.LIDAR_TIMEOUT
            self.risk_level = 'TIMEOUT'
            self._stop_count = self._clear_count = self._ttc_count = 0
        return previous, self.state

    @property
    def should_stop(self):
        if self.state in (self.OBSTACLE_STOP, self.INVALID_SCAN):
            return True
        return self.stop_on_scan_timeout and self.state in (
            self.WAIT_SCAN, self.LIDAR_TIMEOUT)

    def filter_command(self, requested_drive, requested_steering):
        return (0.0 if self.should_stop else requested_drive,
                requested_steering)
