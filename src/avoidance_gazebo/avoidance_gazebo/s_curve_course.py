"""Deterministic geometry and assets for the continuous S-curve course."""

from dataclasses import dataclass
import csv
import math
from pathlib import Path


# Keep the polygon end caps outside the 12 m LiDAR ROI at the CSV goal.  A
# strip polygon necessarily has a cross-course closing edge; placing that
# edge near the goal makes a real wall that perception correctly detects.
WORLD_LENGTH = 42.0
SPAWN_X = 3.25
CURVE_START_X = 4.0
CURVE_END_X = 26.0
CURVE_AMPLITUDE = 0.90
ROAD_HALF_WIDTH = 1.25
WHITE_OFFSET = 0.9875
WHITE_WIDTH = 0.025
CURB_INNER_OFFSET = 1.095
CURB_WIDTH = 0.15
WHEELBASE = 0.77
OBSTACLE_LATERAL_OFFSET = 0.78


@dataclass(frozen=True)
class CoursePose:
    x: float
    y: float
    yaw: float
    curvature: float
    s: float = 0.0


def _profile(x):
    """C2 transition whose value, slope, and curvature vanish at both joins."""
    if x <= CURVE_START_X or x >= CURVE_END_X:
        return 0.0, 0.0, 0.0
    length = CURVE_END_X-CURVE_START_X
    u = (x-CURVE_START_X)/length
    phase = 2.0*math.pi*u
    y = CURVE_AMPLITUDE*(math.sin(phase)-0.5*math.sin(2.0*phase))
    dy = CURVE_AMPLITUDE*(2.0*math.pi/length)*(math.cos(phase)-math.cos(2.0*phase))
    ddy = CURVE_AMPLITUDE*(2.0*math.pi/length)**2*(-math.sin(phase)+2.0*math.sin(2.0*phase))
    return y, dy, ddy


def center_pose_x(x):
    y, dy, ddy = _profile(float(x))
    curvature = ddy/(1.0+dy*dy)**1.5
    return CoursePose(float(x), y, math.atan2(dy, 1.0), curvature)


def sampled_centerline(step=0.05):
    count = round(WORLD_LENGTH/step)
    raw = [center_pose_x(min(WORLD_LENGTH, i*step)) for i in range(count+1)]
    result = []
    distance = 0.0
    for index, pose in enumerate(raw):
        if index:
            distance += math.hypot(pose.x-raw[index-1].x, pose.y-raw[index-1].y)
        result.append(CoursePose(pose.x, pose.y, pose.yaw, pose.curvature, distance))
    return tuple(result)


def interpolate_s(samples, s):
    value = max(samples[0].s, min(float(s), samples[-1].s))
    low, high = 0, len(samples)-1
    while low+1 < high:
        middle = (low+high)//2
        if samples[middle].s <= value:
            low = middle
        else:
            high = middle
    first, second = samples[low], samples[min(low+1, len(samples)-1)]
    span = max(1.0e-12, second.s-first.s)
    ratio = (value-first.s)/span
    yaw_delta = math.atan2(math.sin(second.yaw-first.yaw), math.cos(second.yaw-first.yaw))
    return CoursePose(
        first.x+ratio*(second.x-first.x), first.y+ratio*(second.y-first.y),
        first.yaw+ratio*yaw_delta,
        first.curvature+ratio*(second.curvature-first.curvature), value)


def offset_point(pose, offset):
    return (pose.x-offset*math.sin(pose.yaw),
            pose.y+offset*math.cos(pose.yaw))


def strip_polygon(samples, inner_offset, outer_offset):
    return (tuple(offset_point(pose, outer_offset) for pose in samples) +
            tuple(offset_point(pose, inner_offset) for pose in reversed(samples)))


def polygon_self_intersects(points):
    def orientation(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    edges = tuple(zip(points, points[1:]+points[:1]))
    for i, (a, b) in enumerate(edges):
        for j, (c, d) in enumerate(edges):
            if abs(i-j) <= 1 or {i, j} == {0, len(edges)-1}:
                continue
            if (orientation(a, b, c)*orientation(a, b, d) < 0.0 and
                    orientation(c, d, a)*orientation(c, d, b) < 0.0):
                return True
    return False


def _points_xml(points):
    return ''.join(f'<point>{x:.6f} {y:.6f}</point>' for x, y in points)


def _polyline(name, points, height, z, color, collision=True):
    geometry = f'<geometry><polyline><height>{height:.6f}</height>{_points_xml(points)}</polyline></geometry>'
    collision_xml = f'<collision name="{name}_collision"><pose>0 0 {z:.6f} 0 0 0</pose>{geometry}</collision>' if collision else ''
    return (f'{collision_xml}<visual name="{name}_visual"><pose>0 0 {z:.6f} 0 0 0</pose>'
            f'{geometry}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def render_world():
    samples = sampled_centerline()
    road = strip_polygon(samples, -ROAD_HALF_WIDTH, ROAD_HALF_WIDTH)
    left_white = strip_polygon(samples, WHITE_OFFSET-WHITE_WIDTH/2.0,
                               WHITE_OFFSET+WHITE_WIDTH/2.0)
    right_white = strip_polygon(samples, -WHITE_OFFSET-WHITE_WIDTH/2.0,
                                -WHITE_OFFSET+WHITE_WIDTH/2.0)
    left_curb = strip_polygon(samples, CURB_INNER_OFFSET,
                              CURB_INNER_OFFSET+CURB_WIDTH)
    right_curb = strip_polygon(samples, -CURB_INNER_OFFSET-CURB_WIDTH,
                               -CURB_INNER_OFFSET)
    obstacle_1 = interpolate_s(samples, 8.0)
    obstacle_2 = interpolate_s(samples, 19.0)
    return f'''<?xml version="1.0"?>
<sdf version="1.10">
  <world name="s_curve_avoidance">
    <physics name="default_physics" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.78 0.85 0.92 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>15 0 18 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.1 -0.9</direction></light>
    <!-- Generated from a C2 analytic centerline at 0.05 m resolution. -->
    <model name="s_curve_course"><static>true</static><link name="continuous_course">
      {_polyline('road', road, 0.05, -0.05, '0.14 0.14 0.14 1')}
      {_polyline('left_white', left_white, 0.009, 0.004, '1 1 1 1', False)}
      {_polyline('right_white', right_white, 0.009, 0.004, '1 1 1 1', False)}
      {_polyline('left_curb', left_curb, 0.18, 0.0, '0.62 0.62 0.62 1')}
      {_polyline('right_curb', right_curb, 0.18, 0.0, '0.62 0.62 0.62 1')}
    </link></model>
    <model name="start_line"><static>true</static><link name="line"><visual name="start_line_visual"><pose>4 0 0.009 0 0 1.57079633</pose><geometry><box><size>1.95 0.05 0.012</size></box></geometry><material><ambient>0.10 0.90 0.10 1</ambient><diffuse>0.10 0.90 0.10 1</diffuse></material></visual></link></model>
    <model name="finish_line"><static>true</static><link name="line"><visual name="finish_line_visual"><pose>26 0 0.009 0 0 1.57079633</pose><geometry><box><size>1.95 0.05 0.012</size></box></geometry><material><ambient>0.95 0.10 0.10 1</ambient><diffuse>0.95 0.10 0.10 1</diffuse></material></visual></link></model>
    {_obstacle_xml('obstacle_1', obstacle_1, OBSTACLE_LATERAL_OFFSET, '0.82 0.22 0.10 1')}
    {_obstacle_xml('obstacle_2', obstacle_2, -OBSTACLE_LATERAL_OFFSET, '0.10 0.38 0.88 1')}
  </world>
</sdf>
'''


def _obstacle_xml(name, pose, lateral, color):
    x, y = offset_point(pose, lateral)
    return f'''<model name="{name}"><static>true</static><pose>{x:.6f} {y:.6f} 0.075 0 0 {pose.yaw:.8f}</pose><link name="box_link"><collision name="collision"><geometry><box><size>0.65 0.39 0.15</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.65 0.39 0.15</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''


def route_samples(spacing=0.10, world_start=SPAWN_X, world_end=28.75):
    dense = sampled_centerline(0.025)
    start_s = interpolate_s(dense, 0.0).s
    start_s = min(dense, key=lambda pose: abs(pose.x-world_start)).s
    end_s = min(dense, key=lambda pose: abs(pose.x-world_end)).s
    count = int(math.floor((end_s-start_s)/spacing))
    values = [interpolate_s(dense, start_s+i*spacing) for i in range(count+1)]
    if end_s-values[-1].s > spacing*0.25:
        values.append(interpolate_s(dense, end_s))
    return tuple(values)


def write_assets(workspace):
    root = Path(workspace)
    world_path = root/'src/avoidance_gazebo/worlds/s_curve_avoidance.sdf'
    csv_path = root/'routes/s_curve_reference.csv'
    yaml_path = root/'routes/s_curve_reference.yaml'
    world_path.write_text(render_world(), encoding='utf-8')
    samples = route_samples()
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(('index', 'timestamp', 'latitude', 'longitude', 'x_m',
                         'y_m', 'yaw', 'direction', 'mode', 'drive_level'))
        for index, pose in enumerate(samples):
            writer.writerow((index, f'{index*0.1:.9f}', '0.0000000000',
                             '0.0000000000', f'{pose.x-SPAWN_X:.6f}',
                             f'{pose.y:.6f}', f'{pose.yaw:.9f}', 1,
                             'NORMAL', '2.00'))
    yaml_path.write_text(
        "format_version: 1\norigin_lat: 0.0\norigin_lon: 0.0\nloop: false\n"
        "coordinate_source: gazebo_odometry\nyaw_unit: radian\n"
        "generator: avoidance_gazebo.s_curve_course\nspacing_m: 0.10\n",
        encoding='utf-8')
    return world_path, csv_path, yaml_path
