"""Vehicle-footprint collision, clearance, and path differential geometry."""

from dataclasses import dataclass
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

