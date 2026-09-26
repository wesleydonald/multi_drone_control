"""Time-indexed convex obstacle hulls for the standalone dynamic planner.

R2 uses conservative axis-aligned boxes.  For the analytic sinusoidal obstacle,
the y extrema are computed exactly over each interval rather than estimated from
samples.  Later planner stages can consume the returned vertices directly in a
separator LP.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import numpy as np

from .cooperative_trajectory import SinusoidalLineTrajectory


@dataclass(frozen=True)
class AxisAlignedEnvelope:
    """Physical half extents plus independent isotropic tracking error."""

    physical_half_extents_m: np.ndarray
    tracking_error_m: float = 0.0

    def __post_init__(self) -> None:
        half = np.asarray(self.physical_half_extents_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(half)) or np.any(half < 0.0):
            raise ValueError("physical_half_extents_m must be finite and non-negative")
        if not np.isfinite(self.tracking_error_m) or self.tracking_error_m < 0.0:
            raise ValueError("tracking_error_m must be finite and non-negative")
        object.__setattr__(self, "physical_half_extents_m", half.copy())

    @property
    def effective_half_extents_m(self) -> np.ndarray:
        return self.physical_half_extents_m + float(self.tracking_error_m)


@dataclass(frozen=True)
class TimeIndexedHull:
    interval_index: int
    t_start: float
    t_end: float
    lower: np.ndarray
    upper: np.ndarray
    vertices: np.ndarray

    @property
    def t_mid(self) -> float:
        return 0.5 * (self.t_start + self.t_end)

    def contains(self, point: np.ndarray, tolerance: float = 1e-12) -> bool:
        point = np.asarray(point, dtype=float).reshape(3)
        return bool(
            np.all(point >= self.lower - tolerance)
            and np.all(point <= self.upper + tolerance)
        )


def box_vertices(lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    lower = np.asarray(lower, dtype=float).reshape(3)
    upper = np.asarray(upper, dtype=float).reshape(3)
    if np.any(upper < lower):
        raise ValueError("upper must be >= lower in every coordinate")
    return np.asarray(
        [
            [upper[0] if bx else lower[0], upper[1] if by else lower[1], upper[2] if bz else lower[2]]
            for bx, by, bz in product((False, True), repeat=3)
        ],
        dtype=float,
    )


def uniform_intervals(t_start: float, t_end: float, count: int) -> np.ndarray:
    if count <= 0:
        raise ValueError("count must be positive")
    if not np.isfinite(t_start) or not np.isfinite(t_end) or t_end <= t_start:
        raise ValueError("Expected finite t_end > t_start")
    return np.linspace(float(t_start), float(t_end), int(count) + 1)


def build_sinusoidal_hulls(
    trajectory: SinusoidalLineTrajectory,
    envelope: AxisAlignedEnvelope,
    interval_edges: np.ndarray,
) -> tuple[TimeIndexedHull, ...]:
    """Build guaranteed AABB hulls for an analytic sinusoidal line trajectory."""

    edges = np.asarray(interval_edges, dtype=float).reshape(-1)
    if edges.size < 2 or not np.all(np.isfinite(edges)) or np.any(np.diff(edges) <= 0.0):
        raise ValueError("interval_edges must be finite and strictly increasing")

    half = envelope.effective_half_extents_m
    hulls: list[TimeIndexedHull] = []
    for index, (t0, t1) in enumerate(zip(edges[:-1], edges[1:])):
        y_min, y_max = trajectory.y_extrema(float(t0), float(t1))
        lower = np.array(
            [trajectory.x - half[0], y_min - half[1], trajectory.z - half[2]],
            dtype=float,
        )
        upper = np.array(
            [trajectory.x + half[0], y_max + half[1], trajectory.z + half[2]],
            dtype=float,
        )
        hulls.append(
            TimeIndexedHull(
                interval_index=index,
                t_start=float(t0),
                t_end=float(t1),
                lower=lower,
                upper=upper,
                vertices=box_vertices(lower, upper),
            )
        )
    return tuple(hulls)


def maximum_centreline_containment_violation(
    trajectory: SinusoidalLineTrajectory,
    hulls: tuple[TimeIndexedHull, ...],
    *,
    samples_per_interval: int = 1001,
) -> float:
    """Dense diagnostic check; exact containment comes from analytic construction."""

    if samples_per_interval < 2:
        raise ValueError("samples_per_interval must be at least 2")
    maximum_violation = 0.0
    for hull in hulls:
        for t in np.linspace(hull.t_start, hull.t_end, samples_per_interval):
            point = trajectory.state(float(t)).position
            below = np.maximum(hull.lower - point, 0.0)
            above = np.maximum(point - hull.upper, 0.0)
            maximum_violation = max(
                maximum_violation,
                float(np.max(np.maximum(below, above))),
            )
    return maximum_violation
