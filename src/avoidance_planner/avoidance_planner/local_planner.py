"""Forward-only quintic lateral-offset candidates in the CSV Frenet frame."""

from dataclasses import dataclass
import math
import time

from .geometry import (
    Box2, Pose2, max_curvature_rate, path_collision, path_curvatures,
    path_min_clearances, steering_angles)


@dataclass(frozen=True)
class RouteProjection:
    index: int
    s: float
    d: float
    distance: float


@dataclass(frozen=True)
class Corridor:
    lower_center_d: float
    upper_center_d: float
    side: str

    @property
    def width(self):
        return self.upper_center_d-self.lower_center_d


@dataclass
class Candidate:
    candidate_id: int
    path: tuple
    target_d: float
    return_s: float
    return_length: float
    valid: bool
    reason: str
    max_curvature: float
    max_steering_rad: float
    max_curvature_rate: float
    obstacle_clearance: float
    curb_clearance: float
    length: float
    score: float = math.inf


@dataclass(frozen=True)
class PlanningResult:
    corridor: Corridor
    candidates: tuple
    selected: Candidate | None
    start_pose: Pose2
    pass_pose: Pose2 | None
    return_pose: Pose2 | None
    generation_time_ms: float
    validation_time_ms: float


def route_lengths(route):
    values = [0.0]
    for first, second in zip(route, route[1:]):
        values.append(values[-1]+math.hypot(second.x-first.x, second.y-first.y))
    return values


def project_route(route, x, y, minimum_index=0):
    lengths = route_lengths(route)
    best = None
    for index in range(max(0, minimum_index), len(route)-1):
        first, second = route[index], route[index+1]
        vx, vy = second.x-first.x, second.y-first.y
        denominator = vx*vx+vy*vy
        ratio = 0.0 if denominator <= 1.0e-15 else max(
            0.0, min(1.0, ((x-first.x)*vx+(y-first.y)*vy)/denominator))
        px, py = first.x+ratio*vx, first.y+ratio*vy
        distance = math.hypot(x-px, y-py)
        yaw = math.atan2(vy, vx)
        d = -(x-px)*math.sin(yaw)+(y-py)*math.cos(yaw)
        item = RouteProjection(index, lengths[index]+ratio*math.sqrt(denominator), d, distance)
        if best is None or item.distance < best.distance:
            best = item
    if best is None:
        raise ValueError('reference route requires at least two points')
    return best


def interpolate_route(route, lengths, s):
    s = max(0.0, min(s, lengths[-1]))
    index = 0
    while index < len(lengths)-2 and lengths[index+1] < s:
        index += 1
    first, second = route[index], route[index+1]
    span = max(1.0e-12, lengths[index+1]-lengths[index])
    ratio = (s-lengths[index])/span
    x = first.x+ratio*(second.x-first.x)
    y = first.y+ratio*(second.y-first.y)
    yaw = math.atan2(second.y-first.y, second.x-first.x)
    return x, y, yaw


def quintic_blend(value):
    value = max(0.0, min(1.0, value))
    return 10.0*value**3 - 15.0*value**4 + 6.0*value**5


def calculate_corridor(obstacle, left_boundary, right_boundary,
                       vehicle_width, obstacle_safety, curb_safety):
    """Compute the complete feasible vehicle-centre interval from edges."""
    half = vehicle_width/2.0
    if obstacle.min_y+obstacle.max_y >= 0.0:  # obstacle protrudes from left
        lower = right_boundary+curb_safety+half
        upper = obstacle.min_y-obstacle_safety-half
        side = 'RIGHT_PASS'
    else:
        lower = obstacle.max_y+obstacle_safety+half
        upper = left_boundary-curb_safety-half
        side = 'LEFT_PASS'
    return Corridor(lower, upper, side)


def corridor_targets(corridor, fractions):
    if corridor.width <= 0.0:
        return ()
    return tuple(corridor.lower_center_d+float(f)*corridor.width for f in fractions)


def _path_length(path):
    return sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(path, path[1:]))


def _densify_path(path, interval):
    if len(path) < 2:
        return path
    dense = []
    for first, second in zip(path, path[1:]):
        distance = math.hypot(second.x-first.x, second.y-first.y)
        count = max(1, math.ceil(distance/max(1.0e-4, interval)))
        for index in range(count):
            ratio = index/count
            yaw_delta = math.atan2(math.sin(second.yaw-first.yaw),
                                   math.cos(second.yaw-first.yaw))
            dense.append(Pose2(
                first.x+ratio*(second.x-first.x),
                first.y+ratio*(second.y-first.y),
                first.yaw+ratio*yaw_delta))
    dense.append(path[-1])
    return tuple(dense)


def _build_path(route, current_pose, current_projection, obstacle_s_min,
                obstacle_s_max, target_d, return_length, extension_length,
                interval, vehicle_length, longitudinal_safety):
    lengths = route_lengths(route)
    start_s = current_projection.s
    half_length = vehicle_length/2.0
    outbound_end = obstacle_s_min-half_length-longitudinal_safety
    hold_end = obstacle_s_max+half_length+longitudinal_safety
    transition_end = hold_end+return_length
    return_end = transition_end+max(0.0, extension_length)
    if outbound_end-start_s < 0.60 or return_end > lengths[-1]:
        return (), return_end
    samples = max(2, math.ceil((return_end-start_s)/interval)+1)
    points = []
    for sample in range(samples):
        s = min(return_end, start_s+sample*interval)
        if s <= outbound_end:
            ratio = (s-start_s)/(outbound_end-start_s)
            d = current_projection.d + (target_d-current_projection.d)*quintic_blend(ratio)
        elif s <= hold_end:
            d = target_d
        elif s <= transition_end:
            ratio = (s-hold_end)/(transition_end-hold_end)
            d = target_d*(1.0-quintic_blend(ratio))
        else:
            d = 0.0
        rx, ry, route_yaw = interpolate_route(route, lengths, s)
        points.append([rx-d*math.sin(route_yaw), ry+d*math.cos(route_yaw), route_yaw])
    for index in range(len(points)):
        if index == 0:
            dx, dy = points[1][0]-points[0][0], points[1][1]-points[0][1]
        elif index == len(points)-1:
            dx, dy = points[-1][0]-points[-2][0], points[-1][1]-points[-2][1]
        else:
            dx, dy = points[index+1][0]-points[index-1][0], points[index+1][1]-points[index-1][1]
        points[index][2] = math.atan2(dy, dx)
    # The quintic derivative is zero at both joins; preserve the measured
    # start heading only when it is consistent with the reference direction.
    if abs(math.atan2(math.sin(current_pose.yaw-points[0][2]),
                      math.cos(current_pose.yaw-points[0][2]))) <= math.radians(10):
        points[0][2] = current_pose.yaw
    # The final extension is on the reference route, so it is parallel to
    # the CSV and gives the follower room to settle its heading.
    _, _, return_yaw = interpolate_route(route, lengths, return_end)
    points[-1][2] = return_yaw
    return tuple(Pose2(*point) for point in points), return_end


def plan_candidates(route, current_pose, obstacle_box, left_boundary,
                    right_boundary, vehicle_length=1.30, vehicle_width=0.78,
                    center_offset=0.0, wheelbase=0.77,
                    max_steering_rad=math.radians(25),
                    obstacle_safety_lateral=0.20,
                    obstacle_safety_longitudinal=0.15, curb_safety=0.08,
                    sample_interval=0.05, target_fractions=(0.25, 0.5, 0.75),
                    return_lengths=(2.0, 2.5, 3.0), minimum_index=0,
                    collision_check_interval=0.02,
                    rejoin_straight_extension=1.0):
    """Generate, reject, and score smooth candidates; never clamp steering."""
    projection = project_route(route, current_pose.x, current_pose.y, minimum_index)
    obstacle_corners = ((obstacle_box.min_x, obstacle_box.min_y),
                        (obstacle_box.min_x, obstacle_box.max_y),
                        (obstacle_box.max_x, obstacle_box.min_y),
                        (obstacle_box.max_x, obstacle_box.max_y))
    obstacle_projections = [project_route(route, x, y, projection.index)
                            for x, y in obstacle_corners]
    obstacle_s_min = min(item.s for item in obstacle_projections)
    obstacle_s_max = max(item.s for item in obstacle_projections)
    # Translate lateral obstacle edges into the local route frame. This is
    # exact for the straight course and remains conservative for mild curves.
    lateral_values = [item.d for item in obstacle_projections]
    local_obstacle = Box2(obstacle_s_min, obstacle_s_max,
                          min(lateral_values), max(lateral_values))
    corridor = calculate_corridor(
        local_obstacle, left_boundary, right_boundary, vehicle_width,
        obstacle_safety_lateral, curb_safety)
    targets = corridor_targets(corridor, target_fractions)
    inflated_obstacle = obstacle_box.inflated(
        obstacle_safety_longitudinal, obstacle_safety_lateral)
    effective_left = left_boundary-curb_safety
    effective_right = right_boundary+curb_safety
    candidates = []
    candidate_id = 0
    generation_seconds = 0.0
    validation_seconds = 0.0
    for target in targets:
        for return_length in return_lengths:
            phase_started = time.perf_counter()
            path, return_s = _build_path(
                route, current_pose, projection, obstacle_s_min,
                obstacle_s_max, target, return_length, rejoin_straight_extension,
                sample_interval,
                vehicle_length, obstacle_safety_longitudinal)
            generation_seconds += time.perf_counter()-phase_started
            phase_started = time.perf_counter()
            reason = ''
            if not path:
                reason = 'INSUFFICIENT_LONGITUDINAL_SPACE'
                max_curvature = max_steering = curvature_rate = 0.0
                obstacle_clearance = curb_clearance = 0.0
                length = 0.0
            else:
                curvatures = path_curvatures(path)
                steering = steering_angles(path, wheelbase)
                max_curvature = max((abs(value) for value in curvatures), default=0.0)
                max_steering = max((abs(value) for value in steering), default=0.0)
                curvature_rate = max_curvature_rate(path)
                collision_path = _densify_path(
                    path, min(sample_interval, collision_check_interval))
                collision = path_collision(
                    collision_path, vehicle_length, vehicle_width, center_offset,
                    (inflated_obstacle,), effective_left, effective_right)
                obstacle_clearance, curb_clearance = path_min_clearances(
                    path, vehicle_length, vehicle_width, center_offset,
                    (obstacle_box,), left_boundary, right_boundary)
                length = _path_length(path)
                if collision:
                    reason = 'FOOTPRINT_COLLISION'
                elif max_steering > max_steering_rad+1.0e-9:
                    reason = 'STEERING_LIMIT_EXCEEDED'
                elif max_curvature > math.tan(max_steering_rad)/wheelbase+1.0e-9:
                    reason = 'CURVATURE_LIMIT_EXCEEDED'
            valid = not reason
            candidate = Candidate(
                candidate_id, path, target, return_s, return_length, valid, reason,
                max_curvature, max_steering, curvature_rate,
                obstacle_clearance, curb_clearance, length)
            if valid:
                # Clearance dominates.  Longer return transitions are rewarded
                # because the 20 Hz rate-limited follower can track them with
                # more margin than an equally collision-free short transition.
                candidate.score = (
                    -4.0*min(obstacle_clearance, 2.0)
                    -3.0*min(curb_clearance, 2.0)
                    +2.0*curvature_rate
                    +1.5*max_steering
                    -0.20*return_length
                    +0.02*length + abs(target))
            candidates.append(candidate)
            candidate_id += 1
            validation_seconds += time.perf_counter()-phase_started
    selected = min((item for item in candidates if item.valid),
                   key=lambda item: item.score, default=None)
    pass_pose = None
    return_pose = None
    if selected and selected.path:
        pass_pose = min(selected.path, key=lambda pose: abs(
            project_route(route, pose.x, pose.y, projection.index).s-
            0.5*(obstacle_s_min+obstacle_s_max)))
        return_pose = selected.path[-1]
    return PlanningResult(
        corridor, tuple(candidates), selected, current_pose, pass_pose,
        return_pose, generation_seconds*1000.0, validation_seconds*1000.0)
