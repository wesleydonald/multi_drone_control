"""Stateful safety-planning interface for per-drone trajectory references.

This revision adds diagnostics-only collision checking for separate spherical
quadrotor and electromagnet models, a capsule-style straight cable, and an
optional configurable yaw-oriented payload box against static spherical
obstacles.  All geometric queries are vectorised across the prediction horizon
and obstacle set.  The planner still returns the exact nominal reference and
never intervenes in flight.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

from .payload_geometry import PayloadGeometryProfile, rotate_yaw
from .reference_generators import TrajectoryReference


class SafetyAction(str, Enum):
    """High-level outcome selected by the safety planner."""

    PASS_THROUGH = "PASS_THROUGH"
    REPLAN = "REPLAN"
    HOLD = "HOLD"


def _vec3(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array.copy()


def _positions(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] < 1:
        raise ValueError(f"{name} must have shape (N, 3) with N >= 1, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array.copy()


@dataclass(frozen=True)
class StaticSphereObstacle:
    """One static spherical obstacle in the planner frame."""

    obstacle_id: str
    centre: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        obstacle_id = str(self.obstacle_id).strip()
        if not obstacle_id:
            raise ValueError("obstacle_id must not be empty")

        centre = _vec3(self.centre, "centre")
        radius = float(self.radius)
        if not np.isfinite(radius) or radius < 0.0:
            raise ValueError("radius must be finite and non-negative")

        object.__setattr__(self, "obstacle_id", obstacle_id)
        object.__setattr__(self, "centre", centre)
        object.__setattr__(self, "radius", radius)


@dataclass(frozen=True)
class CollisionCheckResult:
    """Worst signed clearance found for one vehicle component."""

    component_name: str
    nominal_safe: bool
    minimum_clearance: float
    closest_obstacle_id: Optional[str]
    closest_segment_index: Optional[int]
    closest_time_s: Optional[float]
    closest_point: Optional[np.ndarray]
    closest_obstacle_centre: Optional[np.ndarray]
    closest_component_start: Optional[np.ndarray] = None
    closest_component_end: Optional[np.ndarray] = None


@dataclass(frozen=True)
class SafetyPlannerRequest:
    """Inputs needed to validate or replace one nominal rolling reference."""

    nominal_reference: TrajectoryReference
    measured_position: np.ndarray
    measured_velocity: np.ndarray
    magnet_offset_from_quad: np.ndarray
    phase_name: str
    dt: float
    object_attached: bool = False
    payload_geometry_available: bool = False
    payload_offset_from_magnet: Optional[np.ndarray] = None
    payload_yaw: Optional[float] = None
    payload_pose_age_s: Optional[float] = None

    def __post_init__(self) -> None:
        position = _vec3(self.measured_position, "measured_position")
        velocity = _vec3(self.measured_velocity, "measured_velocity")
        magnet_offset = _vec3(
            self.magnet_offset_from_quad,
            "magnet_offset_from_quad",
        )

        if float(self.dt) <= 0.0:
            raise ValueError("dt must be positive")

        phase_name = str(self.phase_name).strip()
        if not phase_name:
            raise ValueError("phase_name must not be empty")

        object.__setattr__(self, "measured_position", position)
        object.__setattr__(self, "measured_velocity", velocity)
        payload_offset = self.payload_offset_from_magnet
        if payload_offset is not None:
            payload_offset = _vec3(
                payload_offset,
                "payload_offset_from_magnet",
            )

        payload_yaw = self.payload_yaw
        if payload_yaw is not None:
            payload_yaw = float(payload_yaw)
            if not np.isfinite(payload_yaw):
                raise ValueError("payload_yaw must be finite when provided")

        payload_pose_age_s = self.payload_pose_age_s
        if payload_pose_age_s is not None:
            payload_pose_age_s = float(payload_pose_age_s)
            if not np.isfinite(payload_pose_age_s) or payload_pose_age_s < 0.0:
                raise ValueError(
                    "payload_pose_age_s must be finite and non-negative when provided"
                )

        payload_geometry_available = bool(self.payload_geometry_available)
        if payload_geometry_available and (
            payload_offset is None or payload_yaw is None
        ):
            raise ValueError(
                "payload geometry marked available without offset and yaw"
            )

        object.__setattr__(self, "magnet_offset_from_quad", magnet_offset)
        object.__setattr__(self, "phase_name", phase_name)
        object.__setattr__(self, "dt", float(self.dt))
        object.__setattr__(self, "object_attached", bool(self.object_attached))
        object.__setattr__(
            self,
            "payload_geometry_available",
            payload_geometry_available,
        )
        object.__setattr__(
            self,
            "payload_offset_from_magnet",
            payload_offset,
        )
        object.__setattr__(self, "payload_yaw", payload_yaw)
        object.__setattr__(self, "payload_pose_age_s", payload_pose_age_s)


@dataclass(frozen=True)
class SafetyPlannerResult:
    """Reference selected for publication plus safety-planner diagnostics."""

    reference: TrajectoryReference
    action: SafetyAction
    intervention_active: bool
    nominal_safe: Optional[bool]
    status: str
    planning_time_ms: float
    minimum_clearance: Optional[float] = None
    critical_component: Optional[str] = None
    quad_minimum_clearance: Optional[float] = None
    magnet_minimum_clearance: Optional[float] = None
    cable_minimum_clearance: Optional[float] = None
    payload_minimum_clearance: Optional[float] = None
    payload_check_active: bool = False
    payload_geometry_available: bool = False
    payload_pose_age_s: Optional[float] = None
    closest_obstacle_id: Optional[str] = None
    closest_segment_index: Optional[int] = None
    closest_time_s: Optional[float] = None
    closest_point: Optional[np.ndarray] = None
    closest_obstacle_centre: Optional[np.ndarray] = None
    predicted_magnet_positions: Optional[np.ndarray] = None
    cable_closest_obstacle_id: Optional[str] = None
    cable_closest_sample_index: Optional[int] = None
    cable_closest_time_s: Optional[float] = None
    cable_closest_point: Optional[np.ndarray] = None
    cable_closest_quad_position: Optional[np.ndarray] = None
    cable_closest_magnet_position: Optional[np.ndarray] = None
    predicted_payload_positions: Optional[np.ndarray] = None
    predicted_payload_yaw: Optional[np.ndarray] = None
    payload_offset_from_magnet: Optional[np.ndarray] = None
    payload_closest_obstacle_id: Optional[str] = None
    payload_closest_sample_index: Optional[int] = None
    payload_closest_time_s: Optional[float] = None
    payload_closest_point: Optional[np.ndarray] = None
    payload_closest_box_centre: Optional[np.ndarray] = None
    payload_closest_yaw: Optional[float] = None


def closest_point_on_segment(
    point: np.ndarray,
    segment_start: np.ndarray,
    segment_end: np.ndarray,
) -> Tuple[np.ndarray, float]:
    """Return the closest point and interpolation fraction on a finite segment."""

    point = _vec3(point, "point")
    segment_start = _vec3(segment_start, "segment_start")
    segment_end = _vec3(segment_end, "segment_end")

    delta = segment_end - segment_start
    length_squared = float(np.dot(delta, delta))
    if length_squared <= 1e-12:
        return segment_start, 0.0

    fraction = float(np.dot(point - segment_start, delta) / length_squared)
    fraction = float(np.clip(fraction, 0.0, 1.0))
    return segment_start + fraction * delta, fraction


def offset_component_positions(
    reference: TrajectoryReference,
    offset_from_quad: np.ndarray,
) -> np.ndarray:
    """Translate the nominal quad path by a fixed measured component offset."""

    offset = _vec3(offset_from_quad, "offset_from_quad")
    return np.asarray(reference.positions, dtype=float) + offset.reshape(1, 3)


def _validate_component_parameters(
    component_name: str,
    component_radius: float,
    safety_margin: float,
    dt: float,
) -> Tuple[str, float, float, float]:
    component_name = str(component_name).strip().upper()
    component_radius = float(component_radius)
    safety_margin = float(safety_margin)
    dt = float(dt)

    if not component_name:
        raise ValueError("component_name must not be empty")
    if not np.isfinite(component_radius) or component_radius < 0.0:
        raise ValueError("component_radius must be finite and non-negative")
    if not np.isfinite(safety_margin) or safety_margin < 0.0:
        raise ValueError("safety_margin must be finite and non-negative")
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    return component_name, component_radius, safety_margin, dt


def _obstacle_arrays(
    obstacles: Sequence[StaticSphereObstacle],
) -> Tuple[Tuple[StaticSphereObstacle, ...], np.ndarray, np.ndarray]:
    obstacle_list = tuple(obstacles)
    if not all(isinstance(obstacle, StaticSphereObstacle) for obstacle in obstacle_list):
        raise TypeError("obstacles must contain only StaticSphereObstacle objects")
    if not obstacle_list:
        return obstacle_list, np.empty((0, 3)), np.empty((0,))
    centres = np.stack([obstacle.centre for obstacle in obstacle_list], axis=0)
    radii = np.asarray([obstacle.radius for obstacle in obstacle_list], dtype=float)
    return obstacle_list, centres, radii


def _empty_collision_result(component_name: str) -> CollisionCheckResult:
    return CollisionCheckResult(
        component_name=component_name,
        nominal_safe=True,
        minimum_clearance=float("inf"),
        closest_obstacle_id=None,
        closest_segment_index=None,
        closest_time_s=None,
        closest_point=None,
        closest_obstacle_centre=None,
    )


def _closest_points_for_segments_and_points(
    segment_starts: np.ndarray,
    segment_ends: np.ndarray,
    points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorised closest points for every segment/point pair.

    ``segment_starts`` and ``segment_ends`` have shape ``(S, 3)`` and ``points``
    has shape ``(P, 3)``.  The returned closest points have shape ``(S, P, 3)``
    and interpolation fractions have shape ``(S, P)``.
    """

    deltas = segment_ends - segment_starts
    length_squared = np.einsum("si,si->s", deltas, deltas)
    relative = points[None, :, :] - segment_starts[:, None, :]
    numerators = np.einsum("spi,si->sp", relative, deltas)

    fractions = np.zeros_like(numerators)
    nondegenerate = length_squared > 1e-12
    if np.any(nondegenerate):
        fractions[nondegenerate, :] = (
            numerators[nondegenerate, :]
            / length_squared[nondegenerate, None]
        )
    np.clip(fractions, 0.0, 1.0, out=fractions)
    closest_points = (
        segment_starts[:, None, :]
        + fractions[:, :, None] * deltas[:, None, :]
    )
    return closest_points, fractions


def check_positions_against_static_spheres(
    positions: np.ndarray,
    obstacles: Sequence[StaticSphereObstacle],
    component_name: str,
    component_radius: float,
    safety_margin: float,
    dt: float,
) -> CollisionCheckResult:
    """Check continuous path segments for one spherical vehicle component.

    Signed clearance is centre distance minus the component radius, obstacle
    radius, and additional safety margin.  A negative value means the inflated
    volumes overlap.  All segment/obstacle pairs are evaluated in one vectorised
    NumPy query; the function is geometric only and never alters the reference.
    """

    positions = _positions(positions, "positions")
    component_name, component_radius, safety_margin, dt = (
        _validate_component_parameters(
            component_name,
            component_radius,
            safety_margin,
            dt,
        )
    )
    obstacle_list, obstacle_centres, obstacle_radii = _obstacle_arrays(obstacles)
    if not obstacle_list:
        return _empty_collision_result(component_name)

    if positions.shape[0] == 1:
        segment_starts = positions
        segment_ends = positions
    else:
        segment_starts = positions[:-1]
        segment_ends = positions[1:]

    closest_points, fractions = _closest_points_for_segments_and_points(
        segment_starts,
        segment_ends,
        obstacle_centres,
    )
    distances = np.linalg.norm(
        closest_points - obstacle_centres[None, :, :],
        axis=2,
    )
    clearances = (
        distances
        - component_radius
        - safety_margin
        - obstacle_radii[None, :]
    )

    minimum_value = float(np.min(clearances))
    # Shared trajectory vertices can produce mathematically identical distances
    # on adjacent segments with tiny floating-point differences.  Prefer the
    # earliest segment/obstacle pair, matching the previous deterministic loop.
    candidates = np.flatnonzero(clearances.ravel() <= minimum_value + 1e-12)
    flat_index = int(candidates[0])
    segment_index, obstacle_index = np.unravel_index(flat_index, clearances.shape)
    minimum_clearance = float(clearances[segment_index, obstacle_index])
    fraction = float(fractions[segment_index, obstacle_index])
    closest_time_s = (
        0.0
        if positions.shape[0] == 1
        else (segment_index + fraction) * dt
    )

    return CollisionCheckResult(
        component_name=component_name,
        nominal_safe=minimum_clearance >= -1e-9,
        minimum_clearance=minimum_clearance,
        closest_obstacle_id=obstacle_list[obstacle_index].obstacle_id,
        closest_segment_index=int(segment_index),
        closest_time_s=float(closest_time_s),
        closest_point=closest_points[segment_index, obstacle_index].copy(),
        closest_obstacle_centre=obstacle_centres[obstacle_index].copy(),
    )


def check_cable_against_static_spheres(
    quad_positions: np.ndarray,
    magnet_positions: np.ndarray,
    obstacles: Sequence[StaticSphereObstacle],
    cable_radius: float,
    safety_margin: float,
    dt: float,
) -> CollisionCheckResult:
    """Check cable capsules at each prediction-horizon sample.

    At sample ``k`` the straight cable centreline is the finite segment from
    ``quad_positions[k]`` to ``magnet_positions[k]``.  Its physical radius and
    safety margin inflate the centreline into a capsule.  Every sample/obstacle
    pair is evaluated in one vectorised query.  This intentionally omits temporal
    subsampling so diagnostics remain lightweight; swept-cable checking between
    samples is a separate later extension.
    """

    quad_positions = _positions(quad_positions, "quad_positions")
    magnet_positions = _positions(magnet_positions, "magnet_positions")
    if quad_positions.shape != magnet_positions.shape:
        raise ValueError(
            "quad_positions and magnet_positions must have the same shape, got "
            f"{quad_positions.shape} and {magnet_positions.shape}"
        )

    component_name, cable_radius, safety_margin, dt = (
        _validate_component_parameters(
            "CABLE",
            cable_radius,
            safety_margin,
            dt,
        )
    )
    obstacle_list, obstacle_centres, obstacle_radii = _obstacle_arrays(obstacles)
    if not obstacle_list:
        return _empty_collision_result(component_name)

    closest_points, _ = _closest_points_for_segments_and_points(
        quad_positions,
        magnet_positions,
        obstacle_centres,
    )
    distances = np.linalg.norm(
        closest_points - obstacle_centres[None, :, :],
        axis=2,
    )
    clearances = (
        distances
        - cable_radius
        - safety_margin
        - obstacle_radii[None, :]
    )

    minimum_value = float(np.min(clearances))
    candidates = np.flatnonzero(clearances.ravel() <= minimum_value + 1e-12)
    flat_index = int(candidates[0])
    sample_index, obstacle_index = np.unravel_index(flat_index, clearances.shape)
    minimum_clearance = float(clearances[sample_index, obstacle_index])

    return CollisionCheckResult(
        component_name=component_name,
        nominal_safe=minimum_clearance >= -1e-9,
        minimum_clearance=minimum_clearance,
        closest_obstacle_id=obstacle_list[obstacle_index].obstacle_id,
        closest_segment_index=int(sample_index),
        closest_time_s=float(sample_index * dt),
        closest_point=closest_points[sample_index, obstacle_index].copy(),
        closest_obstacle_centre=obstacle_centres[obstacle_index].copy(),
        closest_component_start=quad_positions[sample_index].copy(),
        closest_component_end=magnet_positions[sample_index].copy(),
    )


def check_yaw_oriented_boxes_against_static_spheres(
    box_centres: np.ndarray,
    box_yaws: np.ndarray,
    dimensions: np.ndarray,
    obstacles: Sequence[StaticSphereObstacle],
    safety_margin: float,
    dt: float,
) -> CollisionCheckResult:
    """Check level yaw-oriented payload boxes against static spheres.

    The signed distance is evaluated in each box's local yaw frame.  The box is
    inflated by ``safety_margin`` and the sphere radius is then subtracted.  All
    horizon-sample/obstacle pairs are evaluated in vectorised NumPy operations.
    """

    box_centres = _positions(box_centres, "box_centres")
    box_yaws = np.asarray(box_yaws, dtype=float).reshape(-1)
    if box_yaws.shape != (box_centres.shape[0],):
        raise ValueError(
            "box_yaws must contain one yaw per box centre, got "
            f"{box_yaws.shape} for {box_centres.shape[0]} centres"
        )
    if not np.all(np.isfinite(box_yaws)):
        raise ValueError("box_yaws contains NaN or Inf")

    dimensions = _vec3(dimensions, "dimensions")
    if np.any(dimensions <= 0.0):
        raise ValueError("all box dimensions must be positive")
    safety_margin = float(safety_margin)
    dt = float(dt)
    if not np.isfinite(safety_margin) or safety_margin < 0.0:
        raise ValueError("safety_margin must be finite and non-negative")
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")

    obstacle_list, obstacle_centres, obstacle_radii = _obstacle_arrays(obstacles)
    if not obstacle_list:
        return _empty_collision_result("PAYLOAD")

    relative = obstacle_centres[None, :, :] - box_centres[:, None, :]
    cos_yaw = np.cos(box_yaws)[:, None]
    sin_yaw = np.sin(box_yaws)[:, None]
    local = np.empty_like(relative)
    local[:, :, 0] = cos_yaw * relative[:, :, 0] + sin_yaw * relative[:, :, 1]
    local[:, :, 1] = -sin_yaw * relative[:, :, 0] + cos_yaw * relative[:, :, 1]
    local[:, :, 2] = relative[:, :, 2]

    inflated_half_extents = 0.5 * dimensions + safety_margin
    q = np.abs(local) - inflated_half_extents.reshape(1, 1, 3)
    outside_distance = np.linalg.norm(np.maximum(q, 0.0), axis=2)
    inside_distance = np.minimum(np.max(q, axis=2), 0.0)
    signed_box_distance = outside_distance + inside_distance
    clearances = signed_box_distance - obstacle_radii[None, :]

    minimum_value = float(np.min(clearances))
    candidates = np.flatnonzero(clearances.ravel() <= minimum_value + 1e-12)
    flat_index = int(candidates[0])
    sample_index, obstacle_index = np.unravel_index(flat_index, clearances.shape)
    minimum_clearance = float(clearances[sample_index, obstacle_index])

    selected_local = local[sample_index, obstacle_index].copy()
    nearest_local = np.clip(
        selected_local,
        -inflated_half_extents,
        inflated_half_extents,
    )
    if np.all(np.abs(selected_local) <= inflated_half_extents + 1e-12):
        face_distances = inflated_half_extents - np.abs(selected_local)
        face_axis = int(np.argmin(face_distances))
        sign = 1.0 if selected_local[face_axis] >= 0.0 else -1.0
        nearest_local[face_axis] = sign * inflated_half_extents[face_axis]

    yaw = float(box_yaws[sample_index])
    nearest_world = box_centres[sample_index] + rotate_yaw(nearest_local, yaw)

    return CollisionCheckResult(
        component_name="PAYLOAD",
        nominal_safe=minimum_clearance >= -1e-9,
        minimum_clearance=minimum_clearance,
        closest_obstacle_id=obstacle_list[obstacle_index].obstacle_id,
        closest_segment_index=int(sample_index),
        closest_time_s=float(sample_index * dt),
        closest_point=nearest_world,
        closest_obstacle_centre=obstacle_centres[obstacle_index].copy(),
        closest_component_start=box_centres[sample_index].copy(),
    )

def check_reference_against_static_spheres(
    reference: TrajectoryReference,
    obstacles: Sequence[StaticSphereObstacle],
    drone_radius: float,
    safety_margin: float,
    dt: float,
) -> CollisionCheckResult:
    """Backward-compatible wrapper for checking the spherical quadrotor path."""

    return check_positions_against_static_spheres(
        reference.positions,
        obstacles=obstacles,
        component_name="QUAD",
        component_radius=drone_radius,
        safety_margin=safety_margin,
        dt=dt,
    )


def _clearance_text(clearance: float) -> str:
    return "inf" if np.isposinf(clearance) else f"{clearance:.3f}"


class OnlineSafetyPlanner:
    """Stateful boundary between nominal mission planning and control.

    Collision checking is diagnostics-only in this revision.  Every nominal
    reference remains committed and is returned as the exact same object.  When
    an attached payload is expected but its geometry is unavailable, whole-body
    safety is reported as unknown rather than silently safe.  ``REPLAN`` and
    ``HOLD`` remain reserved for later patches.
    """

    def __init__(
        self,
        obstacles: Iterable[StaticSphereObstacle] = (),
        drone_radius: float = 0.20,
        safety_margin: float = 0.10,
        magnet_radius: float = 0.05,
        magnet_safety_margin: float = 0.05,
        cable_radius: float = 0.005,
        cable_safety_margin: float = 0.05,
        payload_profile: Optional[PayloadGeometryProfile] = None,
        payload_pose_timeout_s: float = 0.50,
        collision_diagnostics_enabled: bool = True,
    ) -> None:
        self.obstacles = tuple(obstacles)
        self.drone_radius = float(drone_radius)
        self.safety_margin = float(safety_margin)
        self.magnet_radius = float(magnet_radius)
        self.magnet_safety_margin = float(magnet_safety_margin)
        self.cable_radius = float(cable_radius)
        self.cable_safety_margin = float(cable_safety_margin)
        if payload_profile is not None and not isinstance(
            payload_profile, PayloadGeometryProfile
        ):
            raise TypeError("payload_profile must be a PayloadGeometryProfile or None")
        if payload_profile is not None and not payload_profile.assume_level:
            raise ValueError(
                "V5A payload collision checking supports level yaw-only boxes"
            )
        self.payload_profile = payload_profile
        self.payload_pose_timeout_s = float(payload_pose_timeout_s)
        self.collision_diagnostics_enabled = bool(collision_diagnostics_enabled)

        for name, value in (
            ("drone_radius", self.drone_radius),
            ("safety_margin", self.safety_margin),
            ("magnet_radius", self.magnet_radius),
            ("magnet_safety_margin", self.magnet_safety_margin),
            ("cable_radius", self.cable_radius),
            ("cable_safety_margin", self.cable_safety_margin),
            ("payload_pose_timeout_s", self.payload_pose_timeout_s),
        ):
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not all(isinstance(obstacle, StaticSphereObstacle) for obstacle in self.obstacles):
            raise TypeError("obstacles must contain only StaticSphereObstacle objects")

        self.committed_reference: Optional[TrajectoryReference] = None
        self.current_action = SafetyAction.PASS_THROUGH
        self.last_result: Optional[SafetyPlannerResult] = None
        self.last_phase_name: Optional[str] = None

    def reset(self) -> None:
        """Clear committed planning state, for example after a mission restart."""

        self.committed_reference = None
        self.current_action = SafetyAction.PASS_THROUGH
        self.last_result = None
        self.last_phase_name = None

    def plan(self, request: SafetyPlannerRequest) -> SafetyPlannerResult:
        """Diagnose the whole-body horizon and return the nominal reference unchanged."""

        start_time = time.perf_counter()

        quad_result: Optional[CollisionCheckResult] = None
        magnet_result: Optional[CollisionCheckResult] = None
        cable_result: Optional[CollisionCheckResult] = None
        payload_result: Optional[CollisionCheckResult] = None
        predicted_magnet_positions: Optional[np.ndarray] = None
        predicted_payload_positions: Optional[np.ndarray] = None
        predicted_payload_yaw: Optional[np.ndarray] = None

        payload_check_active = bool(
            self.collision_diagnostics_enabled
            and self.payload_profile is not None
            and self.payload_profile.collision_enabled
            and request.object_attached
        )
        payload_geometry_available = False

        if self.collision_diagnostics_enabled:
            predicted_magnet_positions = offset_component_positions(
                request.nominal_reference,
                request.magnet_offset_from_quad,
            )
            quad_result = check_reference_against_static_spheres(
                request.nominal_reference,
                self.obstacles,
                drone_radius=self.drone_radius,
                safety_margin=self.safety_margin,
                dt=request.dt,
            )
            magnet_result = check_positions_against_static_spheres(
                predicted_magnet_positions,
                self.obstacles,
                component_name="MAGNET",
                component_radius=self.magnet_radius,
                safety_margin=self.magnet_safety_margin,
                dt=request.dt,
            )
            cable_result = check_cable_against_static_spheres(
                request.nominal_reference.positions,
                predicted_magnet_positions,
                self.obstacles,
                cable_radius=self.cable_radius,
                safety_margin=self.cable_safety_margin,
                dt=request.dt,
            )

            if payload_check_active:
                payload_geometry_available = bool(
                    request.payload_geometry_available
                    and request.payload_offset_from_magnet is not None
                    and request.payload_yaw is not None
                    and (
                        request.payload_pose_age_s is None
                        or request.payload_pose_age_s <= self.payload_pose_timeout_s
                    )
                )
                if payload_geometry_available:
                    assert self.payload_profile is not None
                    assert request.payload_offset_from_magnet is not None
                    assert request.payload_yaw is not None
                    predicted_payload_pose_positions = (
                        predicted_magnet_positions
                        + request.payload_offset_from_magnet.reshape(1, 3)
                    )
                    centre_offset = rotate_yaw(
                        self.payload_profile.geometry_origin_from_pose,
                        request.payload_yaw,
                    )
                    predicted_payload_positions = (
                        predicted_payload_pose_positions
                        + centre_offset.reshape(1, 3)
                    )
                    predicted_payload_yaw = np.full(
                        len(predicted_payload_positions),
                        request.payload_yaw,
                        dtype=float,
                    )
                    payload_result = check_yaw_oriented_boxes_against_static_spheres(
                        predicted_payload_positions,
                        predicted_payload_yaw,
                        self.payload_profile.dimensions,
                        self.obstacles,
                        safety_margin=self.payload_profile.collision_margin,
                        dt=request.dt,
                    )

        # V5A is passive.  Do not reconstruct, copy, resample, or otherwise alter
        # the mission generator's trajectory.
        self.committed_reference = request.nominal_reference
        self.current_action = SafetyAction.PASS_THROUGH
        self.last_phase_name = request.phase_name

        planning_time_ms = (time.perf_counter() - start_time) * 1000.0
        if quad_result is None or magnet_result is None or cable_result is None:
            status = (
                f"safety={self.current_action.value} | phase={request.phase_name} | "
                "intervention=false | collision_checking=disabled"
            )
            nominal_safe: Optional[bool] = None
            minimum_clearance: Optional[float] = None
            critical_component: Optional[str] = None
            quad_minimum_clearance: Optional[float] = None
            magnet_minimum_clearance: Optional[float] = None
            cable_minimum_clearance: Optional[float] = None
            payload_minimum_clearance: Optional[float] = None
            closest_obstacle_id: Optional[str] = None
            closest_segment_index: Optional[int] = None
            closest_time_s: Optional[float] = None
            closest_point: Optional[np.ndarray] = None
            closest_obstacle_centre: Optional[np.ndarray] = None
        else:
            known_results = [quad_result, magnet_result, cable_result]
            quad_minimum_clearance = quad_result.minimum_clearance
            magnet_minimum_clearance = magnet_result.minimum_clearance
            cable_minimum_clearance = cable_result.minimum_clearance
            payload_minimum_clearance = (
                None if payload_result is None else payload_result.minimum_clearance
            )
            if payload_result is not None:
                known_results.append(payload_result)

            whole_body_unknown = payload_check_active and not payload_geometry_available
            nominal_safe = (
                None
                if whole_body_unknown
                else all(result.nominal_safe for result in known_results)
            )

            worst_result = min(
                known_results,
                key=lambda result: result.minimum_clearance,
            )
            minimum_clearance = worst_result.minimum_clearance
            critical_component = (
                worst_result.component_name
                if worst_result.closest_obstacle_id is not None
                else None
            )
            closest_obstacle_id = worst_result.closest_obstacle_id
            closest_segment_index = worst_result.closest_segment_index
            closest_time_s = worst_result.closest_time_s
            closest_point = worst_result.closest_point
            closest_obstacle_centre = worst_result.closest_obstacle_centre

            obstacle_text = closest_obstacle_id or "none"
            time_text = "none" if closest_time_s is None else f"{closest_time_s:.3f}"
            payload_clearance_text = (
                "inactive"
                if not payload_check_active
                else (
                    "unknown"
                    if not payload_geometry_available
                    else _clearance_text(payload_minimum_clearance)
                )
            )
            whole_body_text = (
                "unknown"
                if nominal_safe is None
                else ("safe" if nominal_safe else "unsafe")
            )
            status = (
                f"safety={self.current_action.value} | phase={request.phase_name} | "
                "intervention=false | collision_checking=diagnostics_only | "
                f"whole_body_safety={whole_body_text} | "
                f"nominal_safe={'unknown' if nominal_safe is None else str(nominal_safe).lower()} | "
                f"critical_component={critical_component or 'none'} | "
                f"min_clearance_m={_clearance_text(minimum_clearance)} | "
                f"quad_clearance_m={_clearance_text(quad_minimum_clearance)} | "
                f"magnet_clearance_m={_clearance_text(magnet_minimum_clearance)} | "
                f"cable_clearance_m={_clearance_text(cable_minimum_clearance)} | "
                f"payload_check_active={str(payload_check_active).lower()} | "
                f"payload_geometry_available={str(payload_geometry_available).lower()} | "
                f"payload_clearance_m={payload_clearance_text} | "
                f"obstacle={obstacle_text} | closest_time_s={time_text} | "
                f"obstacles={len(self.obstacles)}"
            )

        result = SafetyPlannerResult(
            reference=request.nominal_reference,
            action=self.current_action,
            intervention_active=False,
            nominal_safe=nominal_safe,
            status=status,
            planning_time_ms=planning_time_ms,
            minimum_clearance=minimum_clearance,
            critical_component=critical_component,
            quad_minimum_clearance=quad_minimum_clearance,
            magnet_minimum_clearance=magnet_minimum_clearance,
            cable_minimum_clearance=cable_minimum_clearance,
            payload_minimum_clearance=payload_minimum_clearance,
            payload_check_active=payload_check_active,
            payload_geometry_available=payload_geometry_available,
            payload_pose_age_s=request.payload_pose_age_s,
            closest_obstacle_id=closest_obstacle_id,
            closest_segment_index=closest_segment_index,
            closest_time_s=closest_time_s,
            closest_point=closest_point,
            closest_obstacle_centre=closest_obstacle_centre,
            predicted_magnet_positions=(
                None
                if predicted_magnet_positions is None
                else predicted_magnet_positions.copy()
            ),
            cable_closest_obstacle_id=(
                None if cable_result is None else cable_result.closest_obstacle_id
            ),
            cable_closest_sample_index=(
                None if cable_result is None else cable_result.closest_segment_index
            ),
            cable_closest_time_s=(
                None if cable_result is None else cable_result.closest_time_s
            ),
            cable_closest_point=(
                None
                if cable_result is None or cable_result.closest_point is None
                else cable_result.closest_point.copy()
            ),
            cable_closest_quad_position=(
                None
                if cable_result is None or cable_result.closest_component_start is None
                else cable_result.closest_component_start.copy()
            ),
            cable_closest_magnet_position=(
                None
                if cable_result is None or cable_result.closest_component_end is None
                else cable_result.closest_component_end.copy()
            ),
            predicted_payload_positions=(
                None
                if predicted_payload_positions is None
                else predicted_payload_positions.copy()
            ),
            predicted_payload_yaw=(
                None
                if predicted_payload_yaw is None
                else predicted_payload_yaw.copy()
            ),
            payload_offset_from_magnet=(
                None
                if request.payload_offset_from_magnet is None
                else request.payload_offset_from_magnet.copy()
            ),
            payload_closest_obstacle_id=(
                None if payload_result is None else payload_result.closest_obstacle_id
            ),
            payload_closest_sample_index=(
                None if payload_result is None else payload_result.closest_segment_index
            ),
            payload_closest_time_s=(
                None if payload_result is None else payload_result.closest_time_s
            ),
            payload_closest_point=(
                None
                if payload_result is None or payload_result.closest_point is None
                else payload_result.closest_point.copy()
            ),
            payload_closest_box_centre=(
                None
                if payload_result is None
                or payload_result.closest_component_start is None
                else payload_result.closest_component_start.copy()
            ),
            payload_closest_yaw=(
                None
                if payload_result is None
                or payload_result.closest_segment_index is None
                or predicted_payload_yaw is None
                else float(predicted_payload_yaw[payload_result.closest_segment_index])
            ),
        )
        self.last_result = result
        return result
