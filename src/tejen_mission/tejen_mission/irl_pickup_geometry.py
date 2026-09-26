"""Small ROS-independent geometry helpers for IRL pickup commissioning."""
from __future__ import annotations

import math

import numpy as np


def quaternion_wxyz_to_rotation_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Return the 3x3 rotation matrix for a finite ``[w,x,y,z]`` quaternion."""
    q = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be finite")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be positive")
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def yaw_from_quaternion_wxyz(quaternion_wxyz: np.ndarray) -> float:
    """Extract world yaw from a finite ``[w,x,y,z]`` quaternion."""
    q = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be finite")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be positive")
    w, x, y, z = q / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def transform_local_point(
    world_position: np.ndarray,
    quaternion_wxyz: np.ndarray,
    local_offset: np.ndarray,
) -> np.ndarray:
    """Transform a rigid-body-local point into the world frame."""
    p = np.asarray(world_position, dtype=float).reshape(3)
    offset = np.asarray(local_offset, dtype=float).reshape(3)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(offset)):
        raise ValueError("positions and offsets must be finite")
    return p + quaternion_wxyz_to_rotation_matrix(quaternion_wxyz).dot(offset)


def transform_level_yaw_point(
    world_position: np.ndarray,
    quaternion_wxyz: np.ndarray,
    local_offset: np.ndarray,
) -> np.ndarray:
    """Transform an offset using yaw only, matching PayloadGeometryProfile."""
    p = np.asarray(world_position, dtype=float).reshape(3)
    offset = np.asarray(local_offset, dtype=float).reshape(3)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(offset)):
        raise ValueError("positions and offsets must be finite")
    yaw = yaw_from_quaternion_wxyz(quaternion_wxyz)
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotated = np.array(
        [c * offset[0] - s * offset[1], s * offset[0] + c * offset[1], offset[2]],
        dtype=float,
    )
    return p + rotated
