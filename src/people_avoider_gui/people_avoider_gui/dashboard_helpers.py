"""Small helpers for the People Avoider dashboard.

This file intentionally contains only reusable constants and pure maths/data
conversion helpers. Qt widget creation stays in the main dashboard file because
that keeps the GUI layout easy to follow in one place.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

Point = tuple[float, float]

GRAPH_HISTORY_LENGTH = 200
GOAL_TAGS: dict[str, int] = {
    "A": 10,
    "B": 11,
    "C": 12,
    "D": 13,
    "E": 14,
    "F": 15,
}


def distance_2d(x: float, y: float) -> float:
    """Return the distance from the origin to an x/y point."""
    return math.hypot(x, y)


def split_xy(points: Iterable[Point]) -> tuple[list[float], list[float]]:
    """Split [(x, y), ...] into ([x, ...], [y, ...]) for pyqtgraph."""
    points = list(points)
    if not points:
        return [], []

    xs, ys = zip(*points)
    return list(xs), list(ys)


def local_ros_to_graph(ros_x: float, ros_y: float) -> Point:
    """Convert local ROS axes into the graph axes used by this GUI."""
    return -ros_y, ros_x


def yaw_from_quaternion(q) -> float:
    """Extract yaw, in radians, from a ROS quaternion-like object."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(siny_cosp, cosy_cosp)


def global_to_graph(
    x_global: float,
    y_global: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> Point:
    """Convert an odom/global point into the robot-centred graph frame."""
    dx = x_global - robot_x
    dy = y_global - robot_y

    forward = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
    left = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy

    return -left, forward


def closest_distance(points: Sequence[Point]) -> float:
    """Return the closest point distance, or NaN when no points exist."""
    if not points:
        return float("nan")
    return min(distance_2d(x, y) for x, y in points)


def remaining_path_distance(robot_x: float, robot_y: float, path_points: Sequence[Point]) -> float:
    """Return remaining path length from the closest path point onward."""
    if not path_points:
        return float("nan")

    closest_index, closest_point = min(
        enumerate(path_points),
        key=lambda indexed_point: distance_2d(
            indexed_point[1][0] - robot_x,
            indexed_point[1][1] - robot_y,
        ),
    )

    total = distance_2d(
        closest_point[0] - robot_x,
        closest_point[1] - robot_y,
    )

    for start, end in zip(path_points[closest_index:], path_points[closest_index + 1 :]):
        total += distance_2d(end[0] - start[0], end[1] - start[1])

    return total
