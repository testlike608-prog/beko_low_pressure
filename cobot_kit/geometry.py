"""
The rotation convention, written down once.

    R = Rz @ Ry @ Rx       (degrees)

Every frame bug in a vision cell is a convention bug: two pieces of code that
both say "rx, ry, rz" and mean different products of the same three matrices.
The fix is not care, it is having exactly one module that converts, and making
everything go through it.

This is the convention handeye.json was computed with, so the existing
calibration stays valid in this project.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from .datatypes import Pose  # noqa


def rpy_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """Degrees -> 3x3 rotation. R = Rz @ Ry @ Rx."""
    a, b, c = np.radians([float(rx), float(ry), float(rz)])
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(a), -math.sin(a)],
                   [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)],
                   [0, 1, 0],
                   [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0],
                   [math.sin(c), math.cos(c), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_rpy(R) -> tuple[float, float, float]:
    """3x3 -> degrees. Gimbal lock is handled, not left to produce NaN."""
    R = np.asarray(R, dtype=float)
    sy = max(-1.0, min(1.0, float(-R[2, 0])))
    ry = math.asin(sy)
    if abs(sy) > 1.0 - 1e-9:          # rx and rz describe the same rotation here
        rx = 0.0
        rz = math.atan2(-R[0, 1], R[1, 1])
    else:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    return tuple(float(v) for v in np.degrees([rx, ry, rz]))


def pose_to_matrix(pose) -> np.ndarray:
    """A pose -> the 4x4 that takes points from its frame into its parent."""
    vals = pose.list() if isinstance(pose, Pose) else [float(v) for v in pose]
    x, y, z, rx, ry, rz = vals
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rx, ry, rz)
    T[:3, 3] = (x, y, z)
    return T


def matrix_to_pose(T) -> Pose:
    T = np.asarray(T, dtype=float)
    rx, ry, rz = matrix_to_rpy(T[:3, :3])
    return Pose(float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), rx, ry, rz)


def invert(T) -> np.ndarray:
    """
    Invert a rigid transform without inverting a matrix.

    R.T and -R.T @ t are exact; np.linalg.inv is not, and its error is small,
    systematic, and invisible in a log.
    """
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def normalize(v: Sequence[float]) -> np.ndarray:
    a = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return a / n


def tool_axis(pose) -> np.ndarray:
    """
    Where the tool points, as a unit vector.

    z, because on every industrial tool frame z runs out of the tip. This is
    the line an approach travels along, and the line a sniffer is inserted on.
    """
    rpy = pose.rpy if isinstance(pose, Pose) else [float(v) for v in pose[3:6]]
    return rpy_to_matrix(*rpy)[:3, 2]


def angle_between(a, b) -> float:
    """Degrees between two directions. For sanity checks and log lines."""
    return float(np.degrees(math.acos(
        max(-1.0, min(1.0, float(normalize(a) @ normalize(b)))))))


def look_along(direction, up_hint=(0.0, 0.0, 1.0)) -> np.ndarray:
    """
    A rotation whose z axis runs along `direction`.

    Used when a detected point carries a normal -- the service port's axis, for
    instance -- and the sniffer has to come in along it. Only z is fixed by the
    problem; the spin about z is free, so a hint pins it to something
    repeatable instead of something arbitrary.
    """
    z = normalize(direction)
    hint = np.asarray(up_hint, dtype=float)
    if abs(float(normalize(hint) @ z)) > 0.99:
        hint = np.array([1.0, 0.0, 0.0])
        if abs(float(hint @ z)) > 0.99:
            hint = np.array([0.0, 1.0, 0.0])
    x = normalize(np.cross(hint, z))
    return np.column_stack((x, np.cross(z, x), z))


def pose_facing(xyz, normal, up_hint=(0.0, 0.0, 1.0)) -> Pose:
    """A pose at `xyz` with the tool axis pointing along `normal`."""
    rx, ry, rz = matrix_to_rpy(look_along(normal, up_hint))
    x, y, z = (float(v) for v in xyz)
    return Pose(x, y, z, rx, ry, rz)


def move_points(T, points, frame: str) -> list:
    """
    Push a batch of points through one transform, carrying normals correctly.

    A normal gets the ROTATION but not the translation. Applying the full
    transform to a normal is a quiet classic: the direction ends up pointing at
    the frame origin and the arm approaches from a plausible, wrong angle.
    """
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    out = []
    for p in points:
        xyz = (T @ np.array([p.x, p.y, p.z, 1.0]))[:3]
        moved = p.moved(xyz, frame)
        if p.normal is not None:
            moved = moved.with_normal(R @ np.asarray(p.normal, dtype=float))
        out.append(moved)
    return out
