"""ROS-independent LaserScan filtering and clustering helpers.

Consecutive-point Euclidean clustering is restricted to a configurable front
ROI and intentionally has no motion-control output.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanPoint:
    index: int
    x: float
    y: float
    distance: float


@dataclass(frozen=True)
class Cluster:
    points: tuple
    x: float
    y: float
    nearest_distance: float
    width: float


def valid_roi_points(ranges, angle_min, angle_increment, scan_min, scan_max,
                     min_range, max_range, roi_x_min, roi_x_max,
                     roi_half_width, self_x_min, self_x_max,
                     self_y_half_width):
    """Return finite in-range points inside the front rectangle.

    A configurable rectangle around the vehicle removes self returns.  Invalid
    samples are omitted while their original indices are retained so clusters
    cannot bridge a gap in the scan.
    """
    points = []
    lower = max(float(scan_min), float(min_range))
    upper = min(float(scan_max), float(max_range))
    for index, raw in enumerate(ranges):
        distance = float(raw)
        if not math.isfinite(distance) or distance == 0.0:
            continue
        if distance < lower or distance > upper:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if self_x_min <= x <= self_x_max and abs(y) <= self_y_half_width:
            continue
        if roi_x_min <= x <= roi_x_max and abs(y) <= roi_half_width:
            points.append(ScanPoint(index, x, y, distance))
    return points


def cluster_points(points, distance_threshold, min_points):
    """Cluster consecutive scan points separated by at most the threshold."""
    groups = []
    current = []
    previous = None
    for point in points:
        separated = previous is None or point.index != previous.index + 1
        if previous is not None and not separated:
            separated = math.hypot(point.x - previous.x,
                                   point.y - previous.y) > distance_threshold
        if separated and current:
            groups.append(current)
            current = []
        current.append(point)
        previous = point
    if current:
        groups.append(current)

    result = []
    for group in groups:
        if len(group) < max(1, int(min_points)):
            continue
        x = sum(point.x for point in group) / len(group)
        y = sum(point.y for point in group) / len(group)
        result.append(Cluster(
            tuple(group), x, y,
            min(point.distance for point in group),
            math.hypot(group[-1].x - group[0].x,
                       group[-1].y - group[0].y)))
    return result
