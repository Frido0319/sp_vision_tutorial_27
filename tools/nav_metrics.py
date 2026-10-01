#!/usr/bin/env python3

import math
from typing import Sequence, Tuple


Point = Tuple[float, float]


def _finite(point: Point) -> bool:
    return math.isfinite(point[0]) and math.isfinite(point[1])


def goal_distance(position: Point, goal: Point) -> float:
    """Return Euclidean distance, or infinity when either point is invalid."""
    if not _finite(position) or not _finite(goal):
        return math.inf
    return math.hypot(position[0] - goal[0], position[1] - goal[1])


def point_to_path_distance(position: Point, path: Sequence[Point]) -> float:
    """Return the minimum Euclidean distance to a point or polyline segment."""
    if not _finite(position) or not path:
        return math.inf

    valid_path = [point for point in path if _finite(point)]
    if not valid_path:
        return math.inf
    if len(valid_path) == 1:
        return goal_distance(position, valid_path[0])

    best = math.inf
    px, py = position
    for start, end in zip(valid_path, valid_path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            candidate = goal_distance(position, start)
        else:
            projection = ((px - start[0]) * dx + (py - start[1]) * dy) / length_squared
            projection = min(1.0, max(0.0, projection))
            closest = (start[0] + projection * dx, start[1] + projection * dy)
            candidate = goal_distance(position, closest)
        best = min(best, candidate)
    return best
