"""Pure helpers for physically consistent multirotor reference feedforward.

The external planner publishes translational acceleration, while the payload MPC
tracks an attitude quaternion and a collective-throttle state.  These helpers map
one desired world-frame acceleration plus the planner-provided heading into the
nominal thrust direction and collective required by the MPC's own translational
model.  They are ROS/acados independent so the mapping is easy to regression-test.
"""

from __future__ import annotations

import math

import numpy as np


_GRAVITY_MPS2 = 9.81
_EPS = 1.0e-9


def _normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm


def quaternion_yaw(q: np.ndarray) -> float:
    """Return world-frame yaw from a [w, x, y, z] quaternion."""
    qw, qx, qy, qz = _normalize_quaternion(q)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Return the body-to-world rotation matrix for [w, x, y, z]."""
    qw, qx, qy, qz = _normalize_quaternion(q)
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized [w, x, y, z] quaternion."""
    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(r))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r[2, 1] - r[1, 2]) / scale
        qy = (r[0, 2] - r[2, 0]) / scale
        qz = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / scale
        qx = 0.25 * scale
        qy = (r[0, 1] + r[1, 0]) / scale
        qz = (r[0, 2] + r[2, 0]) / scale
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / scale
        qx = (r[0, 1] + r[1, 0]) / scale
        qy = 0.25 * scale
        qz = (r[1, 2] + r[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / scale
        qx = (r[0, 2] + r[2, 0]) / scale
        qy = (r[1, 2] + r[2, 1]) / scale
        qz = 0.25 * scale

    return _normalize_quaternion(np.array([qw, qx, qy, qz], dtype=float))


def _attitude_from_thrust_axis_and_yaw(body_z_world: np.ndarray, yaw: float) -> np.ndarray:
    """Construct attitude with the requested thrust axis and exact ZYX yaw.

    For the small/moderate tilts used by the planner, solving roll/pitch from the
    desired body-z vector while holding the supplied ZYX yaw is well conditioned.
    This keeps the trajectory quaternion's actual semantic -- heading/yaw -- while
    replacing only the level roll/pitch assumption with the force-consistent tilt.
    """
    b3 = np.asarray(body_z_world, dtype=float).reshape(3)
    b3_norm = float(np.linalg.norm(b3))
    if b3_norm < _EPS:
        return _normalize_quaternion(
            np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])
        )
    b3 = b3 / b3_norm

    # With R = Rz(yaw) Ry(pitch) Rx(roll), the third column is body-z in
    # world coordinates.  Rotating that vector back by -yaw gives
    # [sin(pitch)cos(roll), -sin(roll), cos(pitch)cos(roll)].
    sin_roll = math.sin(yaw) * b3[0] - math.cos(yaw) * b3[1]
    roll = math.asin(float(np.clip(sin_roll, -1.0, 1.0)))
    pitch = math.atan2(
        math.cos(yaw) * b3[0] + math.sin(yaw) * b3[1],
        b3[2],
    )

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    return _rotation_matrix_to_quaternion(rotation)


def acceleration_feedforward_reference(
    *,
    desired_acceleration: np.ndarray,
    thrust_ratio: float,
    heading_quaternion: np.ndarray,
    gravity_mps2: float = _GRAVITY_MPS2,
) -> tuple[float, np.ndarray]:
    """Return nominal collective and attitude for a desired translational acceleration.

    The active quad model is

        v_dot = R(q) e3 * (kT * throttle) - g e3 + ...

    so, deliberately omitting drag/cable-force compensation in this first
    increment, the required specific thrust is

        a_T = a_d + g e3.

    The supplied trajectory quaternion is interpreted as a heading/yaw request;
    roll and pitch are recovered from the thrust direction.  The result is a soft
    MPC reference, not a command outside the optimizer.
    """
    acceleration = np.asarray(desired_acceleration, dtype=float).reshape(3)
    if not np.all(np.isfinite(acceleration)):
        raise ValueError("desired_acceleration contains NaN or Inf")

    k_t = float(thrust_ratio)
    if not np.isfinite(k_t) or k_t <= _EPS:
        raise ValueError(f"thrust_ratio must be positive and finite; got {thrust_ratio}")

    gravity = float(gravity_mps2)
    if not np.isfinite(gravity) or gravity <= 0.0:
        raise ValueError(f"gravity_mps2 must be positive and finite; got {gravity_mps2}")

    yaw = quaternion_yaw(heading_quaternion)
    specific_thrust = acceleration + np.array([0.0, 0.0, gravity], dtype=float)
    thrust_norm = float(np.linalg.norm(specific_thrust))

    if thrust_norm < _EPS:
        # This would correspond to commanded free-fall, outside the current
        # planner's envelope. Use the benign level-hover reference instead of
        # constructing an undefined thrust direction.
        throttle = gravity / k_t
        q_ref = _attitude_from_thrust_axis_and_yaw(
            np.array([0.0, 0.0, 1.0], dtype=float), yaw
        )
        return throttle, q_ref

    throttle = thrust_norm / k_t
    q_ref = _attitude_from_thrust_axis_and_yaw(specific_thrust / thrust_norm, yaw)
    return throttle, q_ref
