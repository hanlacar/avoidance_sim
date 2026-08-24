"""Offline feasibility gate for S-curve obstacle layout generation."""

import math

from avoidance_planner.geometry import Box2, Pose2
from avoidance_planner.local_planner import (
    interpolate_route, plan_candidates, route_lengths)
from avoidance_route.route_following import load_route_csv


def _route_aligned_box(route, lengths, route_s, lateral,
                       length=0.65, width=0.39):
    x, y, yaw = interpolate_route(route, lengths, route_s)
    x -= lateral*math.sin(yaw)
    y += lateral*math.cos(yaw)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    half_x = 0.5*(length*c+width*s)
    half_y = 0.5*(length*s+width*c)
    return Box2(x-half_x, x+half_x, y-half_y, y+half_y)


def validate_s_layout(layout, route_file, max_steering_deg=25.0):
    """Return true only if both ideal physical boxes have a safe candidate."""
    route, _warnings = load_route_csv(route_file)
    lengths = route_lengths(route)
    previous_return_s = 0.0
    for obstacle_s, lateral in layout.ordered:
        # Conservative start: braking can consume part of the nominal 2 m
        # LiDAR trigger distance, so validate from only 3.0 m centre-to-centre.
        start_s = max(previous_return_s, obstacle_s-3.0)
        x, y, yaw = interpolate_route(route, lengths, start_s)
        result = plan_candidates(
            route, Pose2(x, y, yaw),
            _route_aligned_box(route, lengths, obstacle_s, lateral),
            1.095, -1.095, wheelbase=0.77,
            max_steering_rad=math.radians(max_steering_deg),
            obstacle_safety_lateral=0.20,
            obstacle_safety_longitudinal=0.15, curb_safety=0.08,
            target_fractions=(), lateral_target_samples=7,
            return_lengths=(2.0, 2.5, 3.0, 3.5, 4.0),
            rejoin_straight_extension=2.5,
            collision_check_interval=0.02)
        if result.selected is None:
            return False
        previous_return_s = result.selected.return_s
        if obstacle_s != layout.second_s and previous_return_s > layout.second_s-3.0:
            return False
    return True
