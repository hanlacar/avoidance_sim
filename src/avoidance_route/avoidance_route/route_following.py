"""ROS-independent CSV parsing, route geometry, and Pure Pursuit math.

The steering equation and progressive segment projection are adapted from the
validated mission_manager Pure Pursuit implementation. Angles in this module
are radians unless a name explicitly ends in ``_deg``.
"""

import csv
from dataclasses import dataclass
import math
from pathlib import Path


class RouteError(ValueError):
    pass


@dataclass(frozen=True)
class Waypoint:
    index: int
    x: float
    y: float
    yaw: float
    direction: int
    drive_level: float


@dataclass(frozen=True)
class Projection:
    segment: int
    ratio: float
    x: float
    y: float
    distance: float


@dataclass(frozen=True)
class ControlResult:
    nearest: Projection
    nearest_index: int
    lookahead_index: int
    target_x: float
    target_y: float
    steering_rad: float
    angular_rate: float
    requested_steering_rad: float


REQUIRED_COLUMNS = ('index', 'x_m', 'y_m', 'yaw', 'direction', 'drive_level')


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def load_route_csv(path):
    """Return (valid points, warning strings), skipping malformed rows."""
    csv_path = Path(path).expanduser()
    warnings = []
    points = []
    try:
        stream = csv_path.open(newline='', encoding='utf-8')
    except OSError as exc:
        raise RouteError(f'CSV load failed: {exc}') from exc
    with stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RouteError('CSV is empty')
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise RouteError(f'CSV missing columns: {", ".join(missing)}')
        for line_number, row in enumerate(reader, start=2):
            try:
                values = (
                    float(row['x_m']), float(row['y_m']), float(row['yaw']),
                    float(row['drive_level']))
                index = int(row['index'])
                direction = int(row['direction'])
                if not all(math.isfinite(value) for value in values):
                    raise ValueError('non-finite numeric value')
                if direction != 1:
                    raise ValueError('only forward direction=1 is supported')
                if values[3] <= 0.0:
                    raise ValueError('drive_level must be positive')
            except (KeyError, TypeError, ValueError) as exc:
                warnings.append(f'line {line_number}: {exc}')
                continue
            points.append(Waypoint(index, values[0], values[1],
                                   normalize_angle(values[2]), direction, values[3]))
    if len(points) < 2:
        raise RouteError('route requires at least two valid forward waypoints')
    for first, second in zip(points, points[1:]):
        if math.hypot(second.x-first.x, second.y-first.y) <= 1.0e-9:
            raise RouteError('route contains duplicate consecutive points')
    return tuple(points), tuple(warnings)


def route_length(points):
    return sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(points, points[1:]))


def project_segment(points, x, y, segment):
    first, second = points[segment], points[segment + 1]
    vx, vy = second.x-first.x, second.y-first.y
    denominator = vx*vx + vy*vy
    ratio = 0.0 if denominator == 0.0 else max(
        0.0, min(1.0, ((x-first.x)*vx + (y-first.y)*vy) / denominator))
    px, py = first.x + ratio*vx, first.y + ratio*vy
    return Projection(segment, ratio, px, py, math.hypot(x-px, y-py))


def nearest_projection(points, x, y, start_segment=0, window=40):
    last_segment = len(points) - 2
    start = max(0, min(int(start_segment), last_segment))
    end = min(last_segment + 1, start + max(1, int(window)))
    return min((project_segment(points, x, y, segment)
                for segment in range(start, end)), key=lambda item: item.distance)


def lookahead_point(points, projection, distance):
    """Interpolate a point ``distance`` metres ahead along the polyline."""
    remaining = max(0.0, float(distance))
    segment = projection.segment
    x, y = projection.x, projection.y
    while segment < len(points) - 1:
        end = points[segment + 1]
        length = math.hypot(end.x-x, end.y-y)
        if length >= remaining and length > 1.0e-12:
            ratio = remaining / length
            return x + ratio*(end.x-x), y + ratio*(end.y-y), segment + 1
        remaining -= length
        segment += 1
        x, y = points[segment].x, points[segment].y
    last = points[-1]
    # A target fixed at the final waypoint becomes singular as the vehicle
    # approaches it laterally.  Continue the lookahead along the declared
    # terminal heading; goal detection still uses the real final waypoint.
    curvature = terminal_curvature(points)
    if abs(curvature) <= 1.0e-6:
        return (last.x + remaining*math.cos(last.yaw),
                last.y + remaining*math.sin(last.yaw), len(points) - 1)
    end_yaw = last.yaw+curvature*remaining
    return (last.x+(math.sin(end_yaw)-math.sin(last.yaw))/curvature,
            last.y+(math.cos(last.yaw)-math.cos(end_yaw))/curvature,
            len(points)-1)


def terminal_curvature(points):
    """Signed terminal curvature averaged over up to the final 0.5 metre."""
    if len(points) < 3:
        return 0.0
    last = points[-1]
    distance = 0.0
    first = last
    for point, following in zip(reversed(points[:-1]), reversed(points[1:])):
        distance += math.hypot(following.x-point.x, following.y-point.y)
        first = point
        if distance >= 0.5:
            break
    if distance > 1.0e-9 and hasattr(first, 'yaw') and hasattr(last, 'yaw'):
        return normalize_angle(last.yaw-first.yaw)/distance
    first, middle, last = points[-3:]
    a = math.hypot(middle.x-first.x, middle.y-first.y)
    b = math.hypot(last.x-middle.x, last.y-middle.y)
    c = math.hypot(last.x-first.x, last.y-first.y)
    cross = ((middle.x-first.x)*(last.y-first.y)-
             (middle.y-first.y)*(last.x-first.x))
    return 0.0 if a*b*c <= 1.0e-12 else 2.0*cross/(a*b*c)


def curvature_at(points, index):
    if len(points) < 3:
        return 0.0
    middle = max(1, min(int(index), len(points)-2))
    return terminal_curvature(points[middle-1:middle+2])


def pure_pursuit_steering(x, y, yaw, target_x, target_y,
                          wheelbase, max_steering_rad):
    steering = required_pure_pursuit_steering(
        x, y, yaw, target_x, target_y, wheelbase)
    return max(-max_steering_rad, min(max_steering_rad, steering))


def required_pure_pursuit_steering(x, y, yaw, target_x, target_y, wheelbase):
    dx, dy = target_x-x, target_y-y
    distance = math.hypot(dx, dy)
    if distance <= 1.0e-9:
        raise RouteError('lookahead target coincides with vehicle')
    alpha = normalize_angle(math.atan2(dy, dx) - yaw)
    return math.atan(2.0 * wheelbase * math.sin(alpha) / distance)


def limit_steering(previous, requested, max_delta):
    delta = max(-max_delta, min(max_delta, requested-previous))
    return previous + delta


def compute_control(points, x, y, yaw, start_segment, lookahead_m,
                    wheelbase_m, max_steering_rad, previous_steering,
                    max_steering_delta_rad):
    nearest = nearest_projection(points, x, y, start_segment)
    tx, ty, lookahead_index = lookahead_point(points, nearest, lookahead_m)
    requested = required_pure_pursuit_steering(
        x, y, yaw, tx, ty, wheelbase_m)
    if abs(requested) > max_steering_rad+1.0e-9:
        raise RouteError(
            f'required steering {math.degrees(requested):.3f} deg exceeds '
            f'{math.degrees(max_steering_rad):.3f} deg')
    steering = limit_steering(
        previous_steering, requested, max_steering_delta_rad)
    nearest_index = nearest.segment + (1 if nearest.ratio >= 0.5 else 0)
    # angular_rate remains an internal kinematic diagnostic.
    return ControlResult(nearest, nearest_index, lookahead_index, tx, ty,
                         steering, math.tan(steering) / wheelbase_m, requested)


def start_pose_errors(x, y, yaw, route_start):
    return (math.hypot(x-route_start.x, y-route_start.y),
            abs(normalize_angle(yaw-route_start.yaw)))


def start_pose_matches(x, y, yaw, route_start, position_tolerance,
                       yaw_tolerance_rad):
    position_error, yaw_error = start_pose_errors(x, y, yaw, route_start)
    return (position_error <= position_tolerance and yaw_error <= yaw_tolerance_rad,
            position_error, yaw_error)


def avoidance_rejoin_ready(x, y, yaw, goal, max_remaining,
                           lateral_tolerance, yaw_tolerance_rad,
                           max_overshoot=0.0):
    dx, dy = goal.x-x, goal.y-y
    remaining = dx*math.cos(goal.yaw)+dy*math.sin(goal.yaw)
    lateral = abs(-dx*math.sin(goal.yaw)+dy*math.cos(goal.yaw))
    yaw_error = abs(normalize_angle(yaw-goal.yaw))
    ready = (-max(0.0, max_overshoot) <= remaining <= max_remaining and
             lateral <= lateral_tolerance and yaw_error <= yaw_tolerance_rad)
    return ready, remaining, lateral, yaw_error


def path_remaining(points, projection):
    """Return arc length from a projection to the final waypoint."""
    remaining = (1.0-projection.ratio)*math.hypot(
        points[projection.segment+1].x-points[projection.segment].x,
        points[projection.segment+1].y-points[projection.segment].y)
    remaining += sum(math.hypot(b.x-a.x, b.y-a.y)
                     for a, b in zip(points[projection.segment+1:],
                                     points[projection.segment+2:]))
    return remaining


def safety_reason(odom_age, odom_timeout, cross_track_error,
                  max_cross_track_error, finite_control=True):
    if odom_age > odom_timeout:
        return 'ODOMETRY_TIMEOUT'
    if cross_track_error > max_cross_track_error:
        return 'ROUTE_DEVIATION'
    if not finite_control:
        return 'NON_FINITE_CONTROL'
    return ''


def goal_reached(x, y, goal, tolerance):
    return math.hypot(goal.x-x, goal.y-y) <= tolerance
