"""Collision relevance checks between confirmed tracks and remaining CSV path."""

from dataclasses import dataclass
import math

from .geometry import Box2, footprint_collision


@dataclass(frozen=True)
class CollisionRisk:
    required: bool
    nearest_path_index: int
    collision_path_index: int
    longitudinal_distance: float
    emergency: bool


def track_box(track, minimum_depth=0.15, minimum_width=0.08):
    half_depth = max(minimum_depth, track.max_x-track.min_x)/2.0
    half_width = max(minimum_width, track.max_y-track.min_y)/2.0
    return Box2(track.x-half_depth, track.x+half_depth,
                track.y-half_width, track.y+half_width)


def cumulative_lengths(path):
    lengths = [0.0]
    for first, second in zip(path, path[1:]):
        lengths.append(lengths[-1] + math.hypot(second.x-first.x, second.y-first.y))
    return lengths


def nearest_path_index(path, x, y, start=0):
    if not path:
        return 0
    start = max(0, min(start, len(path)-1))
    return min(range(start, len(path)),
               key=lambda index: math.hypot(path[index].x-x, path[index].y-y))


def evaluate_track_collision(path, current_pose, track, vehicle_length,
                             vehicle_width, center_offset, lateral_safety,
                             longitudinal_safety, monitor_distance,
                             emergency_distance, left_boundary,
                             right_boundary, minimum_obstacle_depth=0.15,
                             previous_index=0):
    """Sweep the full rectangular vehicle footprint along remaining poses."""
    if not path:
        return CollisionRisk(False, 0, -1, math.inf, False)
    nearest = nearest_path_index(path, current_pose.x, current_pose.y, previous_index)
    lengths = cumulative_lengths(path)
    obstacle = track_box(track, minimum_obstacle_depth).inflated(
        longitudinal_safety, lateral_safety)
    collision_index = -1
    for index in range(nearest, len(path)):
        forward = lengths[index]-lengths[nearest]
        if forward > monitor_distance:
            break
        if footprint_collision(path[index], vehicle_length, vehicle_width,
                               center_offset, (obstacle,), left_boundary,
                               right_boundary):
            collision_index = index
            break
    distance = math.hypot(track.x-current_pose.x, track.y-current_pose.y)
    ahead = lengths[collision_index]-lengths[nearest] if collision_index >= 0 else math.inf
    return CollisionRisk(collision_index >= 0, nearest, collision_index,
                         ahead, distance <= emergency_distance)


class ReplanDebounce:
    """Latch a confirmed request so it cannot flicker frame to frame."""

    def __init__(self, confirmation_frames=3):
        self.confirmation_frames = max(1, confirmation_frames)
        self.count = 0
        self.latched = False
        self.track_id = None

    def update(self, required, track_id=None):
        if self.latched:
            return True
        if required:
            if self.track_id not in (None, track_id):
                self.count = 0
            self.track_id = track_id
            self.count += 1
            if self.count >= self.confirmation_frames:
                self.latched = True
        else:
            self.count = 0
            self.track_id = None
        return self.latched


def collision_response(track_state):
    """Dynamic tracks stop without planning; static tracks may be planned."""
    if track_state == 'DYNAMIC_OBSTACLE':
        return 'DYNAMIC_OBSTACLE_STOP', False
    if track_state == 'STATIC_OBSTACLE':
        return 'REPLAN_REQUIRED', True
    return 'OBSTACLE_CANDIDATE', False
