"""Planar, metric goal selection shared by the Nav2 bridge and offline tests."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlanarGoal:
    x: float
    y: float
    yaw: float


def select_goal(waypoints, minimum=1.0, maximum=2.5):
    """Choose a distant trajectory point so Nav2 can plan around nearby obstacles."""
    if not all(math.isfinite(v) for v in (minimum, maximum)) or not 0 < minimum <= maximum:
        raise ValueError('Goal distance limits must be finite and 0 < minimum <= maximum')
    points = np.asarray(waypoints, dtype=float).reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('Waypoints must be finite meters/radians')
    selected = None
    selected_distance = -1.0
    for x, y, yaw in points:
        distance = math.hypot(x, y)
        if minimum <= distance <= maximum and distance >= selected_distance:
            selected = PlanarGoal(float(x), float(y), float(yaw))
            selected_distance = distance
    return selected


def transform_goal(goal, translation_x, translation_y, yaw):
    """Transform a planar base-frame goal using the observation-time robot pose."""
    if not all(math.isfinite(v) for v in (translation_x, translation_y, yaw)):
        raise ValueError('Transform must be finite')
    c, s = math.cos(yaw), math.sin(yaw)
    heading = yaw + goal.yaw
    return PlanarGoal(
        translation_x + c*goal.x - s*goal.y,
        translation_y + s*goal.x + c*goal.y,
        math.atan2(math.sin(heading), math.cos(heading)),
    )
