"""Forward-only quintic lateral-offset candidates in the CSV Frenet frame."""

from dataclasses import dataclass
import math
import time

from .geometry import (
    Box2, Pose2, first_path_frenet_collision, max_curvature_rate, path_curvatures,
    path_frenet_clearances, path_frenet_collision, steering_angles)


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
    terminal_curvature_error: float = 0.0
    score: float = math.inf
    entry_length: float = 0.0
    hold_length: float = 0.0
    rejoin_index: int = -1
    collision_path_index: int = -1
    collision_pose: Pose2 | None = None
    collision_target: str = ''


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
    # CSV yaw is the measured / generated route tangent.  A linear position
    # interpolation discards it and turns every coarse waypoint into an
    # instantaneous heading corner.  Cubic Hermite interpolation preserves
    # both endpoint positions and tangents, so Frenet curvature represents
    # the intended road instead of the CSV sampling interval.
    t2, t3 = ratio*ratio, ratio*ratio*ratio
    h00 = 2.0*t3-3.0*t2+1.0
    h10 = t3-2.0*t2+ratio
    h01 = -2.0*t3+3.0*t2
    h11 = t3-t2
    m0x, m0y = span*math.cos(first.yaw), span*math.sin(first.yaw)
    m1x, m1y = span*math.cos(second.yaw), span*math.sin(second.yaw)
    x = h00*first.x+h10*m0x+h01*second.x+h11*m1x
    y = h00*first.y+h10*m0y+h01*second.y+h11*m1y
    dh00 = 6.0*t2-6.0*ratio
    dh10 = 3.0*t2-4.0*ratio+1.0
    dh01 = -dh00
    dh11 = 3.0*t2-2.0*ratio
    dx = dh00*first.x+dh10*m0x+dh01*second.x+dh11*m1x
    dy = dh00*first.y+dh10*m0y+dh01*second.y+dh11*m1y
    yaw = math.atan2(dy, dx)
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


def adaptive_corridor_fractions(corridor, sample_count=7,
                                footprint_projection_margin=0.015):
    """Sample the complete safe corridor, inset by numeric footprint margin."""
    count = max(2, int(sample_count))
    if corridor.width <= 0.0:
        return ()
    edge = max(0.05, min(0.20, footprint_projection_margin/corridor.width))
    return tuple(edge+(1.0-2.0*edge)*index/(count-1)
                 for index in range(count))


def world_box_to_frenet(route, obstacle_box, minimum_index=0):
    """Recover a route-tangent rectangle from its world-axis bounding box."""
    center_x = 0.5*(obstacle_box.min_x+obstacle_box.max_x)
    center_y = 0.5*(obstacle_box.min_y+obstacle_box.max_y)
    center = project_route(route, center_x, center_y, minimum_index)
    lengths = route_lengths(route)
    _x, _y, yaw = interpolate_route(route, lengths, center.s)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    world_x = obstacle_box.max_x-obstacle_box.min_x
    world_y = obstacle_box.max_y-obstacle_box.min_y
    determinant = c*c-s*s
    if abs(determinant) > 0.15:
        length = abs((world_x*c-world_y*s)/determinant)
        width = abs((world_y*c-world_x*s)/determinant)
    else:
        length, width = max(world_x, world_y), min(world_x, world_y)
    length = max(0.08, length)
    width = max(0.08, width)
    return Box2(center.s-length/2.0, center.s+length/2.0,
                center.d-width/2.0, center.d+width/2.0)


def observed_surface_to_frenet_box(route, obstacle_box, minimum_index=0,
                                   obstacle_length=0.65,
                                   obstacle_width=0.39,
                                   left_boundary=math.inf,
                                   right_boundary=-math.inf,
                                   expected_lateral_center=None):
    """Recover a body from its path-facing LiDAR surface observation.

    A path-facing return is normally the inner face, but an oblique approach
    can expose the outer face or a side corner.  Prefer extension away from
    the CSV centreline, unless that hypothesis would put the physical body
    through the known curb boundary; in that case the return must be an
    outer/side observation and the width is extended centreward.
    """
    measured = world_box_to_frenet(route, obstacle_box, minimum_index)
    observed_x = 0.5*(obstacle_box.min_x+obstacle_box.max_x)
    observed_y = 0.5*(obstacle_box.min_y+obstacle_box.max_y)
    observed = project_route(route, observed_x, observed_y, minimum_index)
    if expected_lateral_center is not None:
        center_d = math.copysign(
            abs(float(expected_lateral_center)), observed.d or 1.0)
        length = max(0.08, float(obstacle_length))
        width = max(0.08, float(obstacle_width))
        return Box2(observed.s, observed.s+length,
                    center_d-width/2.0, center_d+width/2.0)
    if measured.max_y-measured.min_y >= 0.8*float(obstacle_width):
        center_s = 0.5*(measured.min_x+measured.max_x)
        center_d = 0.5*(measured.min_y+measured.max_y)
        return Box2(
            center_s-max(float(obstacle_length), measured.max_x-measured.min_x)/2.0,
            center_s+max(float(obstacle_length), measured.max_x-measured.min_x)/2.0,
            center_d-max(float(obstacle_width), measured.max_y-measured.min_y)/2.0,
            center_d+max(float(obstacle_width), measured.max_y-measured.min_y)/2.0)
    length = max(0.08, float(obstacle_length))
    width = max(0.08, float(obstacle_width))
    if observed.d < 0.0:
        if observed.d-width < right_boundary:
            minimum_d, maximum_d = observed.d, observed.d+width
        else:
            minimum_d, maximum_d = observed.d-width, observed.d
    else:
        if observed.d+width > left_boundary:
            minimum_d, maximum_d = observed.d-width, observed.d
        else:
            minimum_d, maximum_d = observed.d, observed.d+width
    return Box2(observed.s, observed.s+length, minimum_d, maximum_d)


def _path_length(path):
    return sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(path, path[1:]))


def terminal_heading_curvature(path, span=0.50):
    if len(path) < 2:
        return 0.0
    distance = 0.0
    first = path[-1]
    for candidate, previous in zip(reversed(path[:-1]), reversed(path[1:])):
        distance += math.hypot(previous.x-candidate.x, previous.y-candidate.y)
        first = candidate
        if distance >= span:
            break
    if distance <= 1.0e-9:
        return 0.0
    delta = math.atan2(math.sin(path[-1].yaw-first.yaw),
                       math.cos(path[-1].yaw-first.yaw))
    return delta/distance


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
    # Let the collision sweep, which already uses the complete vehicle
    # footprint and inflated obstacle, decide how late the lateral transition
    # may finish. Forcing the offset to finish a half vehicle length before
    # the inflated front face shortened a 2 m trigger into ~1.5 m and rejected
    # physically feasible <=25 deg S turns.
    outbound_end = obstacle_s_min
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
                    rejoin_straight_extension=1.0,
                    obstacle_surface_observation=False,
                    obstacle_length=0.65, obstacle_width=0.39,
                    lateral_target_samples=7,
                    expected_obstacle_lateral_center=None):
    """Generate, reject, and score smooth candidates; never clamp steering."""
    projection = project_route(route, current_pose.x, current_pose.y, minimum_index)
    lengths = route_lengths(route)
    local_obstacle = world_box_to_frenet(route, obstacle_box, projection.index)
    if obstacle_surface_observation:
        local_obstacle = observed_surface_to_frenet_box(
            route, obstacle_box, projection.index, obstacle_length,
            obstacle_width, left_boundary, right_boundary,
            expected_obstacle_lateral_center)
    obstacle_s_min, obstacle_s_max = local_obstacle.min_x, local_obstacle.max_x
    corridor = calculate_corridor(
        local_obstacle, left_boundary, right_boundary, vehicle_width,
        obstacle_safety_lateral, curb_safety)
    fractions = (tuple(target_fractions) if target_fractions else
                 adaptive_corridor_fractions(corridor, lateral_target_samples))
    targets = corridor_targets(corridor, fractions)
    inflated_obstacle = local_obstacle.inflated(
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
                collision_detail = first_path_frenet_collision(
                    collision_path, route, vehicle_length, vehicle_width, center_offset,
                    (inflated_obstacle,), effective_left, effective_right,
                    projection.index)
                obstacle_clearance, curb_clearance = path_frenet_clearances(
                    path, route, vehicle_length, vehicle_width, center_offset,
                    (local_obstacle,), left_boundary, right_boundary,
                    projection.index)
                length = _path_length(path)
                terminal_path_curvature = terminal_heading_curvature(path)
                terminal_s = project_route(
                    route, path[-1].x, path[-1].y, projection.index).s
                _, _, terminal_route_yaw = interpolate_route(
                    route, lengths, terminal_s)
                _, _, earlier_route_yaw = interpolate_route(
                    route, lengths, max(0.0, terminal_s-0.50))
                terminal_route_curvature = math.atan2(
                    math.sin(terminal_route_yaw-earlier_route_yaw),
                    math.cos(terminal_route_yaw-earlier_route_yaw))/min(
                        0.50, max(1.0e-9, terminal_s))
                terminal_curvature_error = abs(
                    terminal_path_curvature-terminal_route_curvature)
                if collision_detail is not None:
                    reason = (f'{collision_detail[2]}_COLLISION')
                elif max_steering > max_steering_rad+1.0e-9:
                    reason = 'STEERING_LIMIT'
                elif max_curvature > math.tan(max_steering_rad)/wheelbase+1.0e-9:
                    reason = 'STEERING_LIMIT'
                elif terminal_curvature_error > 0.04:
                    reason = 'REJOIN_CURVATURE'
            if not path:
                terminal_curvature_error = math.inf
                collision_detail = None
                reason = 'REJOIN_INDEX'
            valid = not reason
            candidate = Candidate(
                candidate_id, path, target, return_s, return_length, valid, reason,
                max_curvature, max_steering, curvature_rate,
                obstacle_clearance, curb_clearance, length)
            candidate.terminal_curvature_error = terminal_curvature_error
            candidate.entry_length = max(
                0.0, obstacle_s_min-vehicle_length/2.0-
                obstacle_safety_longitudinal-projection.s)
            candidate.hold_length = (
                obstacle_s_max-obstacle_s_min+vehicle_length+
                2.0*obstacle_safety_longitudinal)
            candidate.rejoin_index = (-1 if not path else project_route(
                route, path[-1].x, path[-1].y, projection.index).index)
            if collision_detail is not None:
                candidate.collision_path_index = collision_detail[0]
                candidate.collision_pose = collision_detail[1]
                candidate.collision_target = collision_detail[2]
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
