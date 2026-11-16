"""ROS-independent validation helpers for M2 fleet trajectory commitments.

M2C only publishes grounded stationary holds.  These helpers make that claim
explicitly checkable without trusting a message's ``terminal_hold`` flag.  M2D can
later extend the same seam for moving authoritative committed trajectories.
"""

from __future__ import annotations

import time

import numpy as np


UINT64_MAX = (1 << 64) - 1


def commitment_sequence_seed(*, now_ns: int | None = None) -> int:
    """Return a large process-restart-safe uint64 sequence seed.

    The seed uses Unix wall-clock nanoseconds so a normal publisher process restart
    begins above the prior process's sequence stream.  This is intentionally a
    lightweight single-host/session convention, not a replacement for a future
    explicit distributed epoch/session identifier.
    """

    value = time.time_ns() if now_ns is None else int(now_ns)
    if value <= 0 or value >= UINT64_MAX - 1_000_000_000:
        raise ValueError("commitment sequence seed must leave uint64 increment headroom")
    return value


def validate_stationary_cubic_hold(
    *,
    control_points,
    knots,
    valid_from_s: float,
    valid_until_s: float,
    expected_position_world,
    now_s: float,
    position_tolerance_m: float = 0.04,
    control_point_tolerance_m: float = 1e-6,
    time_tolerance_s: float = 0.10,
) -> np.ndarray:
    """Validate one exact stationary cubic B-spline hold and return its anchor.

    The validation deliberately checks geometry and time, rather than accepting a
    ``terminal_hold`` Boolean on faith.  For a cubic clamped stationary spline the
    four control points must coincide, the knot vector must be the corresponding
    eight-knot clamped interval, the interval must cover ``now_s``, and the hold
    anchor must agree with the receiving vehicle's measured state.
    """

    cps = np.asarray(control_points, dtype=float)
    knot_values = np.asarray(knots, dtype=float).reshape(-1)
    expected = np.asarray(expected_position_world, dtype=float).reshape(3)
    valid_from = float(valid_from_s)
    valid_until = float(valid_until_s)
    now = float(now_s)
    position_tolerance = float(position_tolerance_m)
    control_tolerance = float(control_point_tolerance_m)
    time_tolerance = float(time_tolerance_s)

    if cps.shape != (4, 3):
        raise ValueError("stationary cubic hold requires exactly four 3D control points")
    if knot_values.shape != (8,):
        raise ValueError("stationary cubic hold requires exactly eight knots")
    if not all(
        np.all(np.isfinite(value))
        for value in (cps, knot_values, expected)
    ) or not all(
        np.isfinite(value)
        for value in (
            valid_from,
            valid_until,
            now,
            position_tolerance,
            control_tolerance,
            time_tolerance,
        )
    ):
        raise ValueError("stationary commitment contains non-finite data")
    if position_tolerance < 0.0 or control_tolerance < 0.0 or time_tolerance < 0.0:
        raise ValueError("stationary commitment tolerances must be non-negative")
    if valid_until <= valid_from:
        raise ValueError("stationary commitment validity interval is empty")
    if np.any(np.diff(knot_values) < -1e-12):
        raise ValueError("stationary commitment knots must be nondecreasing")

    expected_knots = np.asarray(
        [valid_from] * 4 + [valid_until] * 4,
        dtype=float,
    )
    if not np.allclose(knot_values, expected_knots, atol=1e-9, rtol=0.0):
        raise ValueError("stationary cubic hold knots do not match its validity interval")
    if now < valid_from - time_tolerance or now > valid_until + time_tolerance:
        raise ValueError("stationary commitment does not cover current receiver time")

    anchor = cps[0].copy()
    spread = np.linalg.norm(cps - anchor.reshape(1, 3), axis=1)
    if float(np.max(spread)) > control_tolerance:
        raise ValueError("stationary commitment control points are not stationary")
    if float(np.linalg.norm(anchor - expected)) > position_tolerance:
        raise ValueError("stationary commitment anchor disagrees with measured vehicle position")
    return anchor
