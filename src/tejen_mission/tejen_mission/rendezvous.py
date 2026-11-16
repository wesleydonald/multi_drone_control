"""Rendezvous timing utilities for the standalone dynamic planner.

The minimum-time double-integrator calculation is a direct Python translation of
RMADER's ``getMinTimeDoubleIntegrator1D/3D`` helpers.  RMADER multiplies that
minimum time by an allocation factor; its published configuration uses 1.0 when
far from the terminal goal and 2.5 when within 5 m.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .cooperative_trajectory import PointTrajectory, TrajectoryState


def _sgn(value: float) -> int:
    return int(0.0 < value) - int(value < 0.0)


def min_time_double_integrator_1d(
    p0: float,
    v0: float,
    pf: float,
    vf: float,
    v_max: float,
    a_max: float,
) -> float:
    """RMADER's bounded 1-D double-integrator minimum-time heuristic."""

    values = (p0, v0, pf, vf, v_max, a_max)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("All inputs must be finite")
    if v_max <= 0.0 or a_max <= 0.0:
        raise ValueError("v_max and a_max must be positive")

    x1 = float(v0)
    x2 = float(p0)
    x1r = float(vf)
    x2r = float(pf)
    k1 = float(a_max)
    k2 = 1.0
    x1_bar = float(v_max)

    B = (k2 / (2.0 * k1)) * _sgn(-x1 + x1r) * (x1**2 - x1r**2) + x2r
    C = (
        (k2 / (2.0 * k1)) * (x1**2 + x1r**2)
        - (k2 / k1) * x1_bar**2
        + x2r
    )
    D = (
        (-k2 / (2.0 * k1)) * (x1**2 + x1r**2)
        + (k2 / k1) * x1_bar**2
        + x2r
    )

    if x2 <= B and x2 >= C:
        radicand = k2**2 * x1**2 - k1 * k2 * (
            (k2 / (2.0 * k1)) * (x1**2 - x1r**2) + x2 - x2r
        )
        time = (
            -k2 * (x1 + x1r) + 2.0 * math.sqrt(max(0.0, radicand))
        ) / (k1 * k2)
    elif x2 <= B and x2 < C:
        time = (
            (x1_bar - x1 - x1r) / k1
            + (x1**2 + x1r**2) / (2.0 * k1 * x1_bar)
            + (x2r - x2) / (k2 * x1_bar)
        )
    elif x2 > B and x2 <= D:
        radicand = k2**2 * x1**2 + k1 * k2 * (
            (k2 / (2.0 * k1)) * (-x1**2 + x1r**2) + x2 - x2r
        )
        time = (
            k2 * (x1 + x1r) + 2.0 * math.sqrt(max(0.0, radicand))
        ) / (k1 * k2)
    else:
        time = (
            (x1_bar + x1 + x1r) / k1
            + (x1**2 + x1r**2) / (2.0 * k1 * x1_bar)
            + (-x2r + x2) / (k2 * x1_bar)
        )

    return max(0.0, float(time))


def min_time_double_integrator_3d(
    p0: np.ndarray,
    v0: np.ndarray,
    pf: np.ndarray,
    vf: np.ndarray,
    v_max: np.ndarray,
    a_max: np.ndarray,
) -> float:
    """RMADER convention: maximum of the three independent axis times."""

    arrays = [np.asarray(value, dtype=float).reshape(3) for value in (p0, v0, pf, vf, v_max, a_max)]
    p0_a, v0_a, pf_a, vf_a, vmax_a, amax_a = arrays
    times = [
        min_time_double_integrator_1d(
            p0_a[i], v0_a[i], pf_a[i], vf_a[i], vmax_a[i], amax_a[i]
        )
        for i in range(3)
    ]
    return float(max(times))


@dataclass(frozen=True)
class RendezvousEstimate:
    duration_s: float
    goal_state: TrajectoryState
    allocation_factor: float
    iterations: int
    converged: bool


def estimate_rendezvous_time(
    start_state: TrajectoryState,
    goal_trajectory: PointTrajectory,
    *,
    v_max: np.ndarray,
    a_max: np.ndarray,
    factor_alloc: float = 1.0,
    factor_alloc_close: float = 2.5,
    dist_factor_alloc_close_m: float = 5.0,
    initial_guess_s: float | None = None,
    tolerance_s: float = 1e-6,
    max_iterations: int = 50,
) -> RendezvousEstimate:
    """RMADER-style timing with a fixed-point extension for a moving goal."""

    v_max = np.asarray(v_max, dtype=float).reshape(3)
    a_max = np.asarray(a_max, dtype=float).reshape(3)
    if np.any(v_max <= 0.0) or np.any(a_max <= 0.0):
        raise ValueError("v_max and a_max must be positive")
    if factor_alloc < 1.0 or factor_alloc_close < 1.0:
        raise ValueError("allocation factors must be >= 1")
    if dist_factor_alloc_close_m < 0.0:
        raise ValueError("dist_factor_alloc_close_m must be non-negative")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")

    goal_now = goal_trajectory.state(0.0)
    distance_now = float(np.linalg.norm(goal_now.position - start_state.position))
    factor = (
        factor_alloc_close
        if distance_now < dist_factor_alloc_close_m
        else factor_alloc
    )

    if initial_guess_s is None:
        duration = factor * min_time_double_integrator_3d(
            start_state.position,
            start_state.velocity,
            goal_now.position,
            goal_now.velocity,
            v_max,
            a_max,
        )
    else:
        duration = max(0.0, float(initial_guess_s))

    goal_state = goal_trajectory.state(duration)
    for iteration in range(1, max_iterations + 1):
        goal_state = goal_trajectory.state(duration)
        next_duration = factor * min_time_double_integrator_3d(
            start_state.position,
            start_state.velocity,
            goal_state.position,
            goal_state.velocity,
            v_max,
            a_max,
        )
        if abs(next_duration - duration) <= tolerance_s:
            duration = float(next_duration)
            goal_state = goal_trajectory.state(duration)
            return RendezvousEstimate(duration, goal_state, factor, iteration, True)
        duration = float(next_duration)

    goal_state = goal_trajectory.state(duration)
    return RendezvousEstimate(duration, goal_state, factor, max_iterations, False)
