"""Pure LaserScan safety gate used by the vehicle command layer."""

import math


def clamp_steering(steering_deg, limit_deg=27):
    """Return an integer steering command within the MCU contract."""
    limit = min(27, max(0, int(limit_deg)))
    return max(-limit, min(limit, int(steering_deg)))


class LidarSafetyGate:
    WAIT_SCAN = 'WAIT_SCAN'
    CLEAR = 'CLEAR'
    OBSTACLE_STOP = 'OBSTACLE_STOP'
    INVALID_SCAN = 'INVALID_SCAN'
    LIDAR_TIMEOUT = 'LIDAR_TIMEOUT'

    def __init__(self, stop_distance_m=0.50, resume_distance_m=0.60,
                 front_sector_deg=30.0, min_valid_range_m=0.05,
                 stop_confirm_scans=2, clear_confirm_scans=3,
                 scan_timeout_sec=0.5, stop_on_scan_timeout=True,
                 emergency_stop_distance_m=0.30):
        self.stop_distance_m = max(0.0, float(stop_distance_m))
        self.resume_distance_m = max(
            self.stop_distance_m, float(resume_distance_m))
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
        self.state = self.WAIT_SCAN
        self.front_min_distance = math.inf
        self.scan_received = False
        self._stop_count = 0
        self._clear_count = 0

    def _front_minimum(self, scan):
        minimum = math.inf
        valid_count = 0
        if (not math.isfinite(float(scan.angle_min)) or
                not math.isfinite(float(scan.angle_increment)) or
                float(scan.angle_increment) == 0.0 or
                not math.isfinite(float(scan.range_min)) or
                not math.isfinite(float(scan.range_max)) or
                float(scan.range_max) <= float(scan.range_min)):
            return minimum, valid_count
        for index, value in enumerate(scan.ranges):
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if not -self.front_sector_rad <= angle <= self.front_sector_rad:
                continue
            distance = float(value)
            if not math.isfinite(distance) or distance == 0.0:
                continue
            if distance < float(scan.range_min) or distance > float(scan.range_max):
                continue
            if distance < self.min_valid_range_m:
                continue
            valid_count += 1
            minimum = min(minimum, distance)
        return minimum, valid_count

    def update_scan(self, scan):
        previous = self.state
        self.scan_received = True
        self.front_min_distance, valid_count = self._front_minimum(scan)
        if valid_count == 0:
            self.state = self.INVALID_SCAN
            self._stop_count = 0
            self._clear_count = 0
            return previous, self.state
        if self.front_min_distance <= self.emergency_stop_distance_m:
            self.state = self.OBSTACLE_STOP
            self._stop_count = 0
            self._clear_count = 0
            return previous, self.state
        if self.state == self.OBSTACLE_STOP:
            self._stop_count = 0
            if self.front_min_distance >= self.resume_distance_m:
                self._clear_count += 1
                if self._clear_count >= self.clear_confirm_scans:
                    self.state = self.CLEAR
                    self._clear_count = 0
            else:
                self._clear_count = 0
            return previous, self.state
        self._clear_count = 0
        if self.front_min_distance <= self.stop_distance_m:
            self._stop_count += 1
            if self._stop_count >= self.stop_confirm_scans:
                self.state = self.OBSTACLE_STOP
                self._stop_count = 0
        else:
            self._stop_count = 0
            self.state = self.CLEAR
        return previous, self.state

    def update_timeout(self, scan_age_sec):
        previous = self.state
        if self.scan_received and float(scan_age_sec) > self.scan_timeout_sec:
            self.state = self.LIDAR_TIMEOUT
            self._stop_count = 0
            self._clear_count = 0
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
