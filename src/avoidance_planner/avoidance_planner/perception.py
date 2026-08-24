"""ROS-independent scan filtering, adaptive clustering, walls, and tracking."""

from dataclasses import dataclass, field
import math


UNCONFIRMED = 'UNCONFIRMED'
STATIC_OBSTACLE = 'STATIC_OBSTACLE'
DYNAMIC_OBSTACLE = 'DYNAMIC_OBSTACLE'
WALL = 'WALL'
UNKNOWN = 'UNKNOWN'
LOST = 'LOST'


@dataclass(frozen=True)
class ScanPoint:
    index: int
    x: float
    y: float
    distance: float


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    point_count: int
    width: float
    depth: float
    kind: str = UNCONFIRMED
    points: tuple = ()


@dataclass(frozen=True)
class LineFeature:
    x1: float
    y1: float
    x2: float
    y2: float
    heading: float
    length: float
    residual: float


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    first_time: float
    last_time: float
    hits: int = 1
    misses: int = 0
    vx: float = 0.0
    vy: float = 0.0
    speed: float = 0.0
    confidence: float = 0.0
    state: str = UNCONFIRMED
    dynamic_count: int = 0
    static_count: int = 0
    history: list = field(default_factory=list)


def preprocess_scan(ranges, angle_min, angle_increment, scan_min, scan_max,
                    roi_x_min, roi_x_max, roi_half_width,
                    self_x_min, self_x_max, self_y_half_width):
    """Remove invalid, out-of-range, self-reflection, and out-of-ROI samples."""
    points = []
    for index, raw in enumerate(ranges):
        distance = float(raw)
        if not math.isfinite(distance) or distance < scan_min or distance > scan_max:
            continue
        angle = angle_min + index * angle_increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if self_x_min <= x <= self_x_max and abs(y) <= self_y_half_width:
            continue
        if roi_x_min <= x <= roi_x_max and abs(y) <= roi_half_width:
            points.append(ScanPoint(index, x, y, distance))
    return points


def adaptive_gap(distance, near_gap, far_gap, far_distance):
    ratio = min(1.0, max(0.0, distance / max(1.0e-6, far_distance)))
    return near_gap + ratio * (far_gap - near_gap)


def adaptive_clusters(points, min_points, min_width, near_gap, far_gap,
                      far_distance):
    """Cluster adjacent rays with a range-adaptive Euclidean threshold."""
    groups, current, previous = [], [], None
    for point in points:
        split = previous is None or point.index != previous.index + 1
        if previous is not None and not split:
            threshold = adaptive_gap(
                0.5 * (point.distance + previous.distance), near_gap,
                far_gap, far_distance)
            split = math.hypot(point.x-previous.x, point.y-previous.y) > threshold
        if split and current:
            groups.append(current)
            current = []
        current.append(point)
        previous = point
    if current:
        groups.append(current)

    detections = []
    for group in groups:
        if len(group) < min_points:
            continue
        xs, ys = [p.x for p in group], [p.y for p in group]
        width = math.hypot(group[-1].x-group[0].x, group[-1].y-group[0].y)
        if width < min_width:
            continue
        detections.append(Detection(
            sum(xs)/len(xs), sum(ys)/len(ys), min(xs), max(xs),
            min(ys), max(ys), len(group), width, max(xs)-min(xs),
            points=tuple(group)))
    return detections


def fit_line(points):
    """Orthogonal least-squares line fit, equivalent to 2-D PCA."""
    if len(points) < 2:
        return None
    cx = sum(p.x for p in points) / len(points)
    cy = sum(p.y for p in points) / len(points)
    cxx = sum((p.x-cx)**2 for p in points) / len(points)
    cyy = sum((p.y-cy)**2 for p in points) / len(points)
    cxy = sum((p.x-cx)*(p.y-cy) for p in points) / len(points)
    heading = 0.5 * math.atan2(2.0*cxy, cxx-cyy)
    ux, uy = math.cos(heading), math.sin(heading)
    projections = [(p.x-cx)*ux + (p.y-cy)*uy for p in points]
    normal_errors = [-(p.x-cx)*uy + (p.y-cy)*ux for p in points]
    low, high = min(projections), max(projections)
    residual = math.sqrt(sum(value*value for value in normal_errors)/len(points))
    return LineFeature(cx+low*ux, cy+low*uy, cx+high*ux, cy+high*uy,
                       heading, high-low, residual)


def ransac_line(points, inlier_threshold=0.05, minimum_inlier_ratio=0.70):
    """Deterministic bounded RANSAC followed by an orthogonal inlier refit."""
    if len(points) < 2:
        return None
    best = ()
    # At most 32 well-spread hypotheses bounds callback work for 720 rays.
    stride = max(1, len(points)//8)
    anchors = list(range(0, len(points), stride))[:8]
    if len(points)-1 not in anchors:
        anchors.append(len(points)-1)
    for first_index, first in enumerate(anchors):
        for second in anchors[first_index+1:]:
            a, b = points[first], points[second]
            dx, dy = b.x-a.x, b.y-a.y
            length = math.hypot(dx, dy)
            if length <= 1.0e-9:
                continue
            inliers = tuple(point for point in points
                            if abs((point.x-a.x)*dy-(point.y-a.y)*dx)/length
                            <= inlier_threshold)
            if len(inliers) > len(best):
                best = inliers
    if len(best) < max(2, math.ceil(minimum_inlier_ratio*len(points))):
        return None
    return fit_line(best)


def angle_difference(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def split_walls_and_objects(point_groups, route_heading, min_wall_length,
                            max_wall_residual, parallel_tolerance,
                            min_cluster_width):
    """Classify long, straight, route-parallel groups as walls."""
    walls, objects, unknown = [], [], []
    for points in point_groups:
        line = ransac_line(points, max_wall_residual)
        if line is None:
            line = fit_line(points)
        parallel_error = min(angle_difference(line.heading, route_heading),
                             angle_difference(line.heading+math.pi, route_heading))
        if (line.length >= min_wall_length and
                line.residual <= max_wall_residual and
                parallel_error <= parallel_tolerance):
            walls.append(line)
            continue
        xs, ys = [p.x for p in points], [p.y for p in points]
        width = math.hypot(points[-1].x-points[0].x,
                           points[-1].y-points[0].y)
        detection = Detection(
            sum(xs)/len(xs), sum(ys)/len(ys), min(xs), max(xs), min(ys),
            max(ys), len(points), width, max(xs)-min(xs),
            UNCONFIRMED if width >= min_cluster_width else UNKNOWN,
            tuple(points))
        (objects if detection.kind == UNCONFIRMED else unknown).append(detection)
    return walls, objects, unknown


def cluster_groups(points, near_gap, far_gap, far_distance):
    """Return raw adjacent groups for joint wall/object classification."""
    groups, current, previous = [], [], None
    for point in points:
        split = previous is None or point.index != previous.index + 1
        if previous is not None and not split:
            threshold = adaptive_gap(
                0.5*(point.distance+previous.distance), near_gap,
                far_gap, far_distance)
            split = math.hypot(point.x-previous.x, point.y-previous.y) > threshold
        if split and current:
            groups.append(tuple(current))
            current = []
        current.append(point)
        previous = point
    if current:
        groups.append(tuple(current))
    return groups


class TrackManager:
    """Nearest-neighbour odom-frame tracking with velocity hysteresis."""

    def __init__(self, association_distance=0.45, confirmation_frames=3,
                 lost_frames=5, alpha=0.35, dynamic_enter=0.15,
                 dynamic_exit=0.08, dynamic_frames=3, static_frames=5,
                 dynamic_min_observations=3):
        self.association_distance = association_distance
        self.confirmation_frames = confirmation_frames
        self.lost_frames = lost_frames
        self.alpha = alpha
        self.dynamic_enter = dynamic_enter
        self.dynamic_exit = dynamic_exit
        self.dynamic_frames = dynamic_frames
        self.static_frames = static_frames
        self.dynamic_min_observations = dynamic_min_observations
        self.tracks = {}
        self.next_id = 1

    def reset_epoch(self):
        """Discard associations while preserving globally unique track IDs."""
        self.tracks.clear()

    def update(self, detections, stamp):
        unmatched_tracks = set(self.tracks)
        assignments = []
        for detection in detections:
            candidates = [(math.hypot(detection.x-track.x, detection.y-track.y), track_id)
                          for track_id, track in self.tracks.items()
                          if track_id in unmatched_tracks]
            distance, track_id = min(candidates, default=(math.inf, None))
            if distance <= self.association_distance:
                assignments.append((track_id, detection))
                unmatched_tracks.remove(track_id)
            else:
                track_id = self.next_id
                self.next_id += 1
                self.tracks[track_id] = Track(
                    track_id, detection.x, detection.y, detection.min_x,
                    detection.max_x, detection.min_y, detection.max_y,
                    stamp, stamp, history=[(stamp, detection.x, detection.y)])

        for track_id, detection in assignments:
            track = self.tracks[track_id]
            dt = max(1.0e-6, stamp-track.last_time)
            measured_vx = (detection.x-track.x)/dt
            measured_vy = (detection.y-track.y)/dt
            track.vx = self.alpha*measured_vx + (1.0-self.alpha)*track.vx
            track.vy = self.alpha*measured_vy + (1.0-self.alpha)*track.vy
            track.speed = math.hypot(track.vx, track.vy)
            track.x, track.y = detection.x, detection.y
            track.min_x, track.max_x = detection.min_x, detection.max_x
            track.min_y, track.max_y = detection.min_y, detection.max_y
            track.last_time, track.hits, track.misses = stamp, track.hits+1, 0
            track.history.append((stamp, track.x, track.y))
            track.history[:] = track.history[-20:]
            if track.hits >= self.dynamic_min_observations:
                if track.speed >= self.dynamic_enter:
                    track.dynamic_count += 1
                    track.static_count = 0
                elif track.speed <= self.dynamic_exit:
                    track.static_count += 1
                    track.dynamic_count = 0
                if track.dynamic_count >= self.dynamic_frames:
                    track.state = DYNAMIC_OBSTACLE
                elif track.state == DYNAMIC_OBSTACLE:
                    if track.static_count >= self.static_frames:
                        track.state = STATIC_OBSTACLE
                elif (track.speed <= self.dynamic_exit and
                      track.hits >= max(self.confirmation_frames,
                                        self.static_frames)):
                    track.state = STATIC_OBSTACLE
            track.confidence = min(1.0, track.hits/max(1, self.static_frames))

        for track_id in unmatched_tracks:
            track = self.tracks[track_id]
            track.misses += 1
            if track.misses >= self.lost_frames:
                track.state = LOST
        self.tracks = {key: value for key, value in self.tracks.items()
                       if value.misses < self.lost_frames}
        return tuple(sorted(self.tracks.values(), key=lambda item: item.track_id))
