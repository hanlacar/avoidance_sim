"""Vehicle-footprint collision, clearance, and path differential geometry."""

from dataclasses import dataclass
from functools import lru_cache
import math


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Box2:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def inflated(self, longitudinal, lateral=None):
        lateral = longitudinal if lateral is None else lateral
        return Box2(self.min_x-longitudinal, self.max_x+longitudinal,
                    self.min_y-lateral, self.max_y+lateral)


def vehicle_polygon(pose, length, width, center_x_offset=0.0):
    half_l, half_w = length/2.0, width/2.0
    local = ((center_x_offset-half_l, -half_w),
             (center_x_offset+half_l, -half_w),
             (center_x_offset+half_l, half_w),
             (center_x_offset-half_l, half_w))
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return tuple((pose.x+c*x-s*y, pose.y+s*x+c*y) for x, y in local)


def box_polygon(box):
    return ((box.min_x, box.min_y), (box.max_x, box.min_y),
            (box.max_x, box.max_y), (box.min_x, box.max_y))


def _axes(polygon):
    for first, second in zip(polygon, polygon[1:]+polygon[:1]):
        dx, dy = second[0]-first[0], second[1]-first[1]
        length = math.hypot(dx, dy)
        if length > 1.0e-12:
            yield -dy/length, dx/length


def polygons_intersect(first, second):
    """Separating-axis test; touching boundaries count as collision."""
    for ax, ay in tuple(_axes(first))+tuple(_axes(second)):
        p1 = [x*ax+y*ay for x, y in first]
        p2 = [x*ax+y*ay for x, y in second]
        if max(p1) < min(p2) or max(p2) < min(p1):
            return False
    return True


def _point_segment_distance(point, first, second):
    vx, vy = second[0]-first[0], second[1]-first[1]
    denominator = vx*vx+vy*vy
    ratio = 0.0 if denominator <= 1.0e-15 else max(
        0.0, min(1.0, ((point[0]-first[0])*vx+(point[1]-first[1])*vy)/denominator))
    x, y = first[0]+ratio*vx, first[1]+ratio*vy
    return math.hypot(point[0]-x, point[1]-y)


def polygon_clearance(first, second):
    if polygons_intersect(first, second):
        return 0.0
    distances = []
    for polygon_a, polygon_b in ((first, second), (second, first)):
        for point in polygon_a:
            distances.extend(_point_segment_distance(point, a, b)
                             for a, b in zip(polygon_b, polygon_b[1:]+polygon_b[:1]))
    return min(distances, default=math.inf)


def footprint_collision(pose, vehicle_length, vehicle_width, center_offset,
                        obstacle_boxes, left_boundary, right_boundary):
    footprint = vehicle_polygon(pose, vehicle_length, vehicle_width, center_offset)
    if any(y > left_boundary or y < right_boundary for _, y in footprint):
        return True
    return any(polygons_intersect(footprint, box_polygon(box))
               for box in obstacle_boxes)


def path_collision(path, vehicle_length, vehicle_width, center_offset,
                   obstacle_boxes, left_boundary, right_boundary):
    return any(footprint_collision(
        pose, vehicle_length, vehicle_width, center_offset, obstacle_boxes,
        left_boundary, right_boundary) for pose in path)


def path_min_clearances(path, vehicle_length, vehicle_width, center_offset,
                        obstacle_boxes, left_boundary, right_boundary):
    obstacle_clearance, curb_clearance = math.inf, math.inf
    for pose in path:
        footprint = vehicle_polygon(pose, vehicle_length, vehicle_width, center_offset)
        if obstacle_boxes:
            obstacle_clearance = min(
                obstacle_clearance,
                *(polygon_clearance(footprint, box_polygon(box))
                  for box in obstacle_boxes))
        curb_clearance = min(
            curb_clearance,
            min(left_boundary-y for _, y in footprint),
            min(y-right_boundary for _, y in footprint))
    return obstacle_clearance, curb_clearance


@lru_cache(maxsize=16)
def _cached_route_lengths(route):
    values = [0.0]
    for first, second in zip(route, route[1:]):
        values.append(values[-1]+math.hypot(second.x-first.x, second.y-first.y))
    return tuple(values)


def project_point_to_route(route, x, y, minimum_index=0, maximum_index=None):
    """Return ``(segment, s, d, distance)`` in a polyline's local Frenet frame."""
    lengths = _cached_route_lengths(tuple(route))
    best = None
    end = len(route)-1 if maximum_index is None else min(
        len(route)-1, max(int(minimum_index)+1, int(maximum_index)))
    for index in range(max(0, minimum_index), end):
        first, second = route[index], route[index+1]
        vx, vy = second.x-first.x, second.y-first.y
        denominator = vx*vx+vy*vy
        ratio = 0.0 if denominator <= 1.0e-15 else max(
            0.0, min(1.0, ((x-first.x)*vx+(y-first.y)*vy)/denominator))
        px, py = first.x+ratio*vx, first.y+ratio*vy
        yaw = math.atan2(vy, vx)
        item = (index, lengths[index]+ratio*math.sqrt(denominator),
                -(x-px)*math.sin(yaw)+(y-py)*math.cos(yaw),
                math.hypot(x-px, y-py))
        if best is None or item[3] < best[3]:
            best = item
    if best is None:
        raise ValueError('reference route requires at least two points')
    return best


def footprint_frenet_bounds(pose, route, vehicle_length, vehicle_width,
                             center_offset=0.0, minimum_index=0):
    shifted_x = pose.x+center_offset*math.cos(pose.yaw)
    shifted_y = pose.y+center_offset*math.sin(pose.yaw)
    center = project_point_to_route(
        route, shifted_x, shifted_y, minimum_index,
        min(len(route)-1, minimum_index+25))
    first, second = route[center[0]], route[center[0]+1]
    route_yaw = math.atan2(second.y-first.y, second.x-first.x)
    relative = math.atan2(math.sin(pose.yaw-route_yaw),
                          math.cos(pose.yaw-route_yaw))
    c, s = abs(math.cos(relative)), abs(math.sin(relative))
    half_s = 0.5*(vehicle_length*c+vehicle_width*s)
    half_d = 0.5*(vehicle_length*s+vehicle_width*c)+0.01
    return (Box2(center[1]-half_s, center[1]+half_s,
                 center[2]-half_d, center[2]+half_d), center[0])


def boxes_intersect(first, second):
    return not (first.max_x < second.min_x or second.max_x < first.min_x or
                first.max_y < second.min_y or second.max_y < first.min_y)


def path_frenet_collision(path, route, vehicle_length, vehicle_width,
                           center_offset, obstacle_boxes, left_boundary,
                           right_boundary, minimum_index=0):
    detail = first_path_frenet_collision(
        path, route, vehicle_length, vehicle_width, center_offset,
        obstacle_boxes, left_boundary, right_boundary, minimum_index)
    return detail is not None


def first_path_frenet_collision(path, route, vehicle_length, vehicle_width,
                                 center_offset, obstacle_boxes, left_boundary,
                                 right_boundary, minimum_index=0):
    """Return first ``(index, pose, target, footprint)`` collision detail."""
    route_index = minimum_index
    for index, pose in enumerate(path):
        footprint, route_index = footprint_frenet_bounds(
            pose, route, vehicle_length, vehicle_width, center_offset,
            max(minimum_index, route_index-3))
        if footprint.max_y > left_boundary or footprint.min_y < right_boundary:
            return index, pose, 'CURB', footprint
        if any(boxes_intersect(footprint, obstacle) for obstacle in obstacle_boxes):
            return index, pose, 'OBSTACLE', footprint
    return None


def path_frenet_clearances(path, route, vehicle_length, vehicle_width,
                            center_offset, obstacle_boxes, left_boundary,
                            right_boundary, minimum_index=0):
    obstacle_clearance, curb_clearance = math.inf, math.inf
    route_index = minimum_index
    for pose in path:
        footprint, route_index = footprint_frenet_bounds(
            pose, route, vehicle_length, vehicle_width, center_offset,
            max(minimum_index, route_index-3))
        curb_clearance = min(curb_clearance, left_boundary-footprint.max_y,
                             footprint.min_y-right_boundary)
        for obstacle in obstacle_boxes:
            if boxes_intersect(footprint, obstacle):
                obstacle_clearance = 0.0
            else:
                dx = max(obstacle.min_x-footprint.max_x,
                         footprint.min_x-obstacle.max_x, 0.0)
                dy = max(obstacle.min_y-footprint.max_y,
                         footprint.min_y-obstacle.max_y, 0.0)
                obstacle_clearance = min(obstacle_clearance, math.hypot(dx, dy))
    return obstacle_clearance, curb_clearance


def path_curvatures(path):
    """Signed three-point curvature with endpoint values copied inward."""
    if len(path) < 3:
        return [0.0] * len(path)
    values = []
    for first, middle, last in zip(path, path[1:], path[2:]):
        a = math.hypot(middle.x-first.x, middle.y-first.y)
        b = math.hypot(last.x-middle.x, last.y-middle.y)
        c = math.hypot(last.x-first.x, last.y-first.y)
        cross = ((middle.x-first.x)*(last.y-first.y) -
                 (middle.y-first.y)*(last.x-first.x))
        values.append(0.0 if a*b*c <= 1.0e-12 else 2.0*cross/(a*b*c))
    return [values[0], *values, values[-1]]


def steering_angles(path, wheelbase):
    return [math.atan(wheelbase*value) for value in path_curvatures(path)]


def max_curvature_rate(path):
    curvature = path_curvatures(path)
    rates = []
    for first, second, p1, p2 in zip(curvature, curvature[1:], path, path[1:]):
        distance = math.hypot(p2.x-p1.x, p2.y-p1.y)
        if distance > 1.0e-9:
            rates.append(abs(second-first)/distance)
    return max(rates, default=0.0)
