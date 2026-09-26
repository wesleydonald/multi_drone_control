"""ROS-independent moving-obstacle prediction for dynamic avoidance.

This module intentionally contains no ROS imports.  It implements the M1 prediction
model used by the thesis obstacle-avoidance stack:

    centre(t) = p0 + v_filtered * t

    radius(t) = r_geometry
                + position_uncertainty
                + velocity_uncertainty * t
                + 0.5 * acceleration_uncertainty * t^2

The predicted obstacle is represented by time-indexed swept-sphere intervals.  Each
interval stores a capsule from the predicted centre at t_k to the centre at t_{k+1}
with a conservative radius equal to the larger endpoint radius.  The union of these
intervals forms the widening "bugle" reachable tube discussed in the planner design.

Design lineage: the time-indexed reachable-set representation follows the same broad
idea as temporal safe-corridor / dynamic-obstacle planners that keep future obstacle
occupancy separated by time.  The particular linear-plus-quadratic radius law above
is this thesis implementation's explicit bounded-uncertainty model, not a verbatim
copy of one paper's predictor.

Vehicle/quadrotor whole-body inflation is deliberately not applied here.  This
module predicts the obstacle reachable set only; later planning/validation stages
combine it with the vehicle configuration-space model.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional, Sequence, Tuple

import numpy as np


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return array.copy()


@dataclass(frozen=True)
class MovingObstaclePredictionConfig:
    """Configuration for the bounded constant-velocity reachable-set model."""

    horizon_s: float = 3.0
    time_step_s: float = 0.25

    # First-order low-pass filter for measured obstacle velocity.
    velocity_filter_tau_s: float = 0.20

    # Reachable-set growth terms.
    position_uncertainty_m: float = 0.03
    velocity_uncertainty_mps: float = 0.10
    acceleration_uncertainty_mps2: float = 0.15

    def __post_init__(self) -> None:
        for name in (
            "horizon_s",
            "time_step_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        for name in (
            "velocity_filter_tau_s",
            "position_uncertainty_m",
            "velocity_uncertainty_mps",
            "acceleration_uncertainty_mps2",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class MovingObstacleState:
    """One measured obstacle state supplied to the predictor."""

    obstacle_id: str
    position: np.ndarray
    velocity: np.ndarray
    geometry_radius_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "position",
            _vec3(self.position, "position"),
        )
        object.__setattr__(
            self,
            "velocity",
            _vec3(self.velocity, "velocity"),
        )

        radius = float(self.geometry_radius_m)
        if not math.isfinite(radius) or radius < 0.0:
            raise ValueError("geometry_radius_m must be finite and non-negative")
        object.__setattr__(self, "geometry_radius_m", radius)
        object.__setattr__(self, "obstacle_id", str(self.obstacle_id))


@dataclass(frozen=True)
class ReachableInterval:
    """Conservative swept-sphere occupancy over one future time interval.

    `conservative_radius_m` is the capsule radius that covers the complete interval.
    Since the uncertainty model grows monotonically, it is simply the larger
    endpoint radius.
    """

    start_time_s: float
    end_time_s: float
    start_center: np.ndarray
    end_center: np.ndarray
    start_radius_m: float
    end_radius_m: float
    conservative_radius_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "start_center",
            _vec3(self.start_center, "start_center"),
        )
        object.__setattr__(
            self,
            "end_center",
            _vec3(self.end_center, "end_center"),
        )

        start_time = float(self.start_time_s)
        end_time = float(self.end_time_s)
        if (
            not math.isfinite(start_time)
            or not math.isfinite(end_time)
            or start_time < 0.0
            or end_time <= start_time
        ):
            raise ValueError("reachable interval times must satisfy 0 <= start < end")

        for name in (
            "start_radius_m",
            "end_radius_m",
            "conservative_radius_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class MovingObstaclePrediction:
    """Complete time-indexed prediction for one obstacle."""

    obstacle_id: str
    filtered_velocity: np.ndarray
    sample_times_s: np.ndarray
    sample_centers: np.ndarray
    sample_radii_m: np.ndarray
    intervals: Tuple[ReachableInterval, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        velocity = _vec3(self.filtered_velocity, "filtered_velocity")
        times = np.asarray(self.sample_times_s, dtype=float).reshape(-1)
        centers = np.asarray(self.sample_centers, dtype=float)
        radii = np.asarray(self.sample_radii_m, dtype=float).reshape(-1)

        if times.ndim != 1 or times.size < 2 or not np.all(np.isfinite(times)):
            raise ValueError("sample_times_s must contain at least two finite values")
        if abs(float(times[0])) > 1e-12 or np.any(np.diff(times) <= 0.0):
            raise ValueError("sample_times_s must start at zero and be strictly increasing")
        if centers.shape != (times.size, 3) or not np.all(np.isfinite(centers)):
            raise ValueError("sample_centers must have shape (N, 3) and be finite")
        if radii.shape != (times.size,) or not np.all(np.isfinite(radii)):
            raise ValueError("sample_radii_m must have shape (N,) and be finite")
        if np.any(radii < 0.0):
            raise ValueError("sample_radii_m must be non-negative")

        object.__setattr__(self, "obstacle_id", str(self.obstacle_id))
        object.__setattr__(self, "filtered_velocity", velocity)
        object.__setattr__(self, "sample_times_s", times.copy())
        object.__setattr__(self, "sample_centers", centers.copy())
        object.__setattr__(self, "sample_radii_m", radii.copy())
        object.__setattr__(self, "intervals", tuple(self.intervals))


def prediction_radius_m(
    geometry_radius_m: float,
    future_time_s: float,
    config: MovingObstaclePredictionConfig,
) -> float:
    """Return the obstacle reachable-set radius at future time `t`."""

    radius = float(geometry_radius_m)
    t = float(future_time_s)

    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError("geometry_radius_m must be finite and non-negative")
    if not math.isfinite(t) or t < 0.0:
        raise ValueError("future_time_s must be finite and non-negative")

    return (
        radius
        + config.position_uncertainty_m
        + config.velocity_uncertainty_mps * t
        + 0.5 * config.acceleration_uncertainty_mps2 * t * t
    )


def low_pass_velocity(
    previous_filtered_velocity: Sequence[float],
    measured_velocity: Sequence[float],
    dt_s: float,
    tau_s: float,
) -> np.ndarray:
    """One first-order low-pass update for obstacle velocity."""

    previous = _vec3(previous_filtered_velocity, "previous_filtered_velocity")
    measured = _vec3(measured_velocity, "measured_velocity")

    dt = float(dt_s)
    tau = float(tau_s)

    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    if not math.isfinite(tau) or tau < 0.0:
        raise ValueError("tau_s must be finite and non-negative")

    if tau <= 1e-12:
        return measured

    alpha = dt / (tau + dt)
    return previous + alpha * (measured - previous)


def prediction_sample_times(
    config: MovingObstaclePredictionConfig,
) -> np.ndarray:
    """Return deterministic sample times including exactly 0 and the horizon."""

    step = float(config.time_step_s)
    horizon = float(config.horizon_s)

    count = max(1, int(math.floor(horizon / step)))
    times = np.arange(count + 1, dtype=float) * step

    if times[-1] < horizon - 1e-12:
        times = np.append(times, horizon)
    else:
        times[-1] = horizon

    return times


def build_reachable_prediction(
    state: MovingObstacleState,
    filtered_velocity: Sequence[float],
    config: MovingObstaclePredictionConfig,
) -> MovingObstaclePrediction:
    """Build the widening time-indexed reachable tube for one obstacle."""

    velocity = _vec3(filtered_velocity, "filtered_velocity")
    times = prediction_sample_times(config)

    centers = (
        state.position[None, :]
        + times[:, None] * velocity[None, :]
    )

    radii = np.asarray(
        [
            prediction_radius_m(
                state.geometry_radius_m,
                time_s,
                config,
            )
            for time_s in times
        ],
        dtype=float,
    )

    intervals = []
    for index in range(len(times) - 1):
        start_radius = float(radii[index])
        end_radius = float(radii[index + 1])
        intervals.append(
            ReachableInterval(
                start_time_s=float(times[index]),
                end_time_s=float(times[index + 1]),
                start_center=centers[index],
                end_center=centers[index + 1],
                start_radius_m=start_radius,
                end_radius_m=end_radius,
                conservative_radius_m=max(start_radius, end_radius),
            )
        )

    return MovingObstaclePrediction(
        obstacle_id=state.obstacle_id,
        filtered_velocity=velocity,
        sample_times_s=times,
        sample_centers=centers,
        sample_radii_m=radii,
        intervals=tuple(intervals),
    )


class MovingObstaclePredictor:
    """Stateful velocity filter plus pure reachable-tube construction.

    The first update initializes the filter from the measured velocity.  Later
    updates apply a first-order low-pass filter before predicting the obstacle
    centre with constant velocity.
    """

    def __init__(
        self,
        config: MovingObstaclePredictionConfig,
    ) -> None:
        self.config = config
        self._filtered_velocity: Optional[np.ndarray] = None

    @property
    def filtered_velocity(self) -> Optional[np.ndarray]:
        if self._filtered_velocity is None:
            return None
        return self._filtered_velocity.copy()

    def reset(self) -> None:
        self._filtered_velocity = None

    def update(
        self,
        state: MovingObstacleState,
        dt_s: Optional[float] = None,
    ) -> MovingObstaclePrediction:
        if self._filtered_velocity is None:
            self._filtered_velocity = state.velocity.copy()
        else:
            if dt_s is None:
                raise ValueError("dt_s is required after the first predictor update")
            self._filtered_velocity = low_pass_velocity(
                self._filtered_velocity,
                state.velocity,
                dt_s,
                self.config.velocity_filter_tau_s,
            )

        return build_reachable_prediction(
            state,
            self._filtered_velocity,
            self.config,
        )
