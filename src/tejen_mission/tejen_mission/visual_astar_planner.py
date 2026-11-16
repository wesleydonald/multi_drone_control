"""Passive 3D voxel A* candidate planner for the join-planner safety layer.

This module is deliberately ROS-independent.  It detects when the nominal quad-centre
polyline is blocked, chooses a later safe sample on the same nominal trajectory, runs
ordinary Euclidean A* to that rejoin point, and validates a constant-speed rendering of
the candidate against the same quad, magnet, cable, and payload geometry used by the
online safety diagnostics.

The result is diagnostic only.  It never replaces or mutates the nominal reference.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Optional, Sequence, Tuple

import numpy as np
from .voxel_occupancy import VoxelOccupancyGrid

from .online_safety_planner import (
    StaticSphereObstacle,
    check_cable_against_static_spheres,
    check_positions_against_static_spheres,
    check_yaw_oriented_boxes_against_static_spheres,
)
from .payload_geometry import PayloadGeometryProfile, rotate_yaw
from .convex_trajectory_smoother import (
    ConvexTrajectoryConfig,
    ConvexTrajectoryResult,
    resolve_target_acceleration,
    solve_convex_corridor_trajectory,
)
from .voxel_astar import AStarPlanResult, SphereObstacle, VoxelAStar3D

from .convex_corridor import (
    ConvexPolyhedron,
    build_convex_corridor,
    corridor_overlap_margins,
    voxelize_inflated_spheres,
    corridor_segment_containment_margins,
    corridor_voxel_exclusion_margins,
    corridor_point_exclusion_margins,
)


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return array.copy()


def _positions(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[0] < 1 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), N >= 1")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array.copy()


@dataclass(frozen=True)
class VisualAStarConfig:
    enabled: bool = True
    resolution: float = 0.08
    max_expansions: int = 150_000
    max_planning_time_s: float = 0.25
    nearest_free_radius_m: float = 0.40
    local_margin_xy_m: float = 1.25
    local_margin_z_m: float = 0.75
    global_bounds_min: np.ndarray = field(
        default_factory=lambda: np.array([-5.0, -5.0, 0.0], dtype=float)
    )
    global_bounds_max: np.ndarray = field(
        default_factory=lambda: np.array([5.0, 5.0, 4.0], dtype=float)
    )
    rejoin_min_lookahead_s: float = 0.35
    rejoin_preferred_lookahead_s: float = 1.00
    rejoin_min_separation_m: float = 0.20
    candidate_speed_mps: float = 0.35

    # target maximum movement prediction based on ego drone movement and target movement
    target_prediction_max_s: float = 4.0

    # maximum cable swing in radians
    max_cable_swing_angle_rad: float = math.radians(10.0)

    def __post_init__(self) -> None:
        resolution = float(self.resolution)
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("resolution must be finite and positive")
        bounds_min = _vec3(self.global_bounds_min, "global_bounds_min")
        bounds_max = _vec3(self.global_bounds_max, "global_bounds_max")
        if np.any(bounds_max <= bounds_min):
            raise ValueError("global_bounds_max must exceed global_bounds_min")
        for name, value in (
            ("max_planning_time_s", self.max_planning_time_s),
            ("nearest_free_radius_m", self.nearest_free_radius_m),
            ("local_margin_xy_m", self.local_margin_xy_m),
            ("local_margin_z_m", self.local_margin_z_m),
            ("rejoin_min_lookahead_s", self.rejoin_min_lookahead_s),
            ("rejoin_preferred_lookahead_s", self.rejoin_preferred_lookahead_s),
            ("rejoin_min_separation_m", self.rejoin_min_separation_m),
            ("candidate_speed_mps", self.candidate_speed_mps),
            ("target_prediction_max_s", self.target_prediction_max_s),
            ("max_cable_swing_angle_rad", self.max_cable_swing_angle_rad),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.rejoin_preferred_lookahead_s < self.rejoin_min_lookahead_s:
            raise ValueError(
                "rejoin_preferred_lookahead_s must be >= rejoin_min_lookahead_s"
            )
        if self.max_cable_swing_angle_rad >= 0.5 * math.pi:
            raise ValueError(
                "max_cable_swing_angle_rad must be less than pi/2"
            )
        object.__setattr__(self, "enabled", bool(self.enabled))
        object.__setattr__(self, "resolution", resolution)
        object.__setattr__(self, "max_expansions", max(1, int(self.max_expansions)))
        object.__setattr__(
            self, "max_planning_time_s", max(1e-4, float(self.max_planning_time_s))
        )
        object.__setattr__(
            self, "nearest_free_radius_m", float(self.nearest_free_radius_m)
        )
        object.__setattr__(self, "local_margin_xy_m", float(self.local_margin_xy_m))
        object.__setattr__(self, "local_margin_z_m", float(self.local_margin_z_m))
        object.__setattr__(self, "global_bounds_min", bounds_min)
        object.__setattr__(self, "global_bounds_max", bounds_max)
        object.__setattr__(
            self, "rejoin_min_lookahead_s", float(self.rejoin_min_lookahead_s)
        )
        object.__setattr__(
            self,
            "rejoin_preferred_lookahead_s",
            float(self.rejoin_preferred_lookahead_s),
        )
        object.__setattr__(
            self, "rejoin_min_separation_m", float(self.rejoin_min_separation_m)
        )
        object.__setattr__(
            self, "candidate_speed_mps", float(self.candidate_speed_mps)
        )
        object.__setattr__(
            self, "target_prediction_max_s", float(self.target_prediction_max_s)
        )
        object.__setattr__(
            self,
            "max_cable_swing_angle_rad",
            float(self.max_cable_swing_angle_rad),
        )


@dataclass(frozen=True)
class VisualAStarRequest:
    request_id: int
    phase_name: str
    phase_allowed: bool
    nominal_positions: np.ndarray
    dt: float
    obstacles: Tuple[StaticSphereObstacle, ...]
    drone_radius: float
    drone_safety_margin: float
    magnet_offset_from_quad: np.ndarray
    magnet_radius: float
    magnet_safety_margin: float
    cable_radius: float
    cable_safety_margin: float
    nominal_velocities: Optional[np.ndarray] = None
    nominal_accelerations: Optional[np.ndarray] = None
    # planning start position and velocity
    # Measured state from which the new trajectory must begin.
    planning_start_position: Optional[np.ndarray] = None
    planning_start_velocity: Optional[np.ndarray] = None
    planning_start_acceleration: Optional[np.ndarray] = None
    object_attached: bool = False
    payload_profile: Optional[PayloadGeometryProfile] = None
    payload_geometry_available: bool = False
    payload_offset_from_magnet: Optional[np.ndarray] = None
    payload_yaw: Optional[float] = None
    target_position: Optional[np.ndarray] = None
    target_velocity_mean: Optional[np.ndarray] = None
    target_velocity_variance: Optional[np.ndarray] = None
    target_acceleration_mean: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        phase_name = str(self.phase_name).strip()
        if not phase_name:
            raise ValueError("phase_name must not be empty")
        nominal_positions = _positions(self.nominal_positions, "nominal_positions")
        nominal_velocities = self.nominal_velocities
        nominal_accelerations = self.nominal_accelerations

        if nominal_velocities is None:
            nominal_velocities = np.zeros_like(nominal_positions)
        else:
            nominal_velocities = _positions(nominal_velocities, "nominal_velocities")
        if nominal_accelerations is None:
            nominal_accelerations = np.zeros_like(nominal_positions)
        else:
            nominal_accelerations = _positions(
                nominal_accelerations, "nominal_accelerations"
            )

        # validate planning start position adn velocity
        planning_start_position = self.planning_start_position
        if planning_start_position is not None:
            planning_start_position = _vec3(
                planning_start_position, "planning_start_position"
            )

        planning_start_velocity = self.planning_start_velocity
        if planning_start_velocity is not None:
            planning_start_velocity = _vec3(
                planning_start_velocity, "planning_start_velocity"
            )

        planning_start_acceleration = (
            self.planning_start_acceleration
        )

        if planning_start_acceleration is not None:
            planning_start_acceleration = _vec3(
                planning_start_acceleration,
                "planning_start_acceleration",
            )

        
        if nominal_velocities.shape != nominal_positions.shape:
            raise ValueError("nominal_velocities must match nominal_positions")
        if nominal_accelerations.shape != nominal_positions.shape:
            raise ValueError("nominal_accelerations must match nominal_positions")
        dt = float(self.dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        obstacles = tuple(self.obstacles)
        if not all(isinstance(item, StaticSphereObstacle) for item in obstacles):
            raise TypeError("obstacles must contain StaticSphereObstacle objects")
        magnet_offset = _vec3(self.magnet_offset_from_quad, "magnet_offset_from_quad")
        payload_offset = self.payload_offset_from_magnet
        if payload_offset is not None:
            payload_offset = _vec3(payload_offset, "payload_offset_from_magnet")
        payload_yaw = self.payload_yaw
        if payload_yaw is not None:
            payload_yaw = float(payload_yaw)
            if not math.isfinite(payload_yaw):
                raise ValueError("payload_yaw must be finite")
        target_position = self.target_position
        if target_position is not None:
            target_position = _vec3(target_position, "target_position")
        target_velocity_mean = self.target_velocity_mean
        if target_velocity_mean is not None:
            target_velocity_mean = _vec3(
                target_velocity_mean, "target_velocity_mean"
            )
        target_velocity_variance = self.target_velocity_variance
        if target_velocity_variance is not None:
            target_velocity_variance = _vec3(
                target_velocity_variance, "target_velocity_variance"
            )
            if np.any(target_velocity_variance < 0.0):
                raise ValueError("target_velocity_variance must be non-negative")
        target_acceleration_mean = (
            self.target_acceleration_mean
        )

        if target_acceleration_mean is not None:
            target_acceleration_mean = _vec3(
                target_acceleration_mean,
                "target_acceleration_mean",
            )
        for name, value in (
            ("drone_radius", self.drone_radius),
            ("drone_safety_margin", self.drone_safety_margin),
            ("magnet_radius", self.magnet_radius),
            ("magnet_safety_margin", self.magnet_safety_margin),
            ("cable_radius", self.cable_radius),
            ("cable_safety_margin", self.cable_safety_margin),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        object.__setattr__(self, "request_id", int(self.request_id))
        object.__setattr__(self, "phase_name", phase_name)
        object.__setattr__(self, "phase_allowed", bool(self.phase_allowed))
        object.__setattr__(self, "nominal_positions", nominal_positions)
        object.__setattr__(self, "nominal_velocities", nominal_velocities)
        object.__setattr__(self, "nominal_accelerations", nominal_accelerations)

        object.__setattr__(self, "planning_start_position", planning_start_position)
        object.__setattr__(self, "planning_start_velocity", planning_start_velocity)
        object.__setattr__(
            self,
            "planning_start_acceleration",
            planning_start_acceleration,
        )

        object.__setattr__(self, "dt", dt)
        object.__setattr__(self, "obstacles", obstacles)
        object.__setattr__(self, "magnet_offset_from_quad", magnet_offset)
        object.__setattr__(self, "object_attached", bool(self.object_attached))
        object.__setattr__(
            self, "payload_geometry_available", bool(self.payload_geometry_available)
        )
        object.__setattr__(self, "payload_offset_from_magnet", payload_offset)
        object.__setattr__(self, "payload_yaw", payload_yaw)
        object.__setattr__(self, "target_position", target_position)
        object.__setattr__(self, "target_velocity_mean", target_velocity_mean)
        object.__setattr__(self, "target_velocity_variance", target_velocity_variance)
        object.__setattr__(
            self,
            "target_acceleration_mean",
            target_acceleration_mean,
        )


@dataclass(frozen=True)
class VisualAStarResult:
    request_id: int
    phase_name: str
    phase_allowed: bool
    status: str
    message: str
    success: bool = False
    nominal_blocked: bool = False
    first_blocked_segment_index: Optional[int] = None
    rejoin_index: Optional[int] = None
    rejoin_time_s: Optional[float] = None
    rejoin_candidate_indices: Tuple[int, ...] = tuple()
    rejoin_candidate_positions: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=float)
    )
    raw_path: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    simplified_path: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=float)
    )
    candidate_positions: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=float)
    )
    planning_time_ms: float = 0.0
    expanded_nodes: int = 0
    generated_nodes: int = 0
    raw_path_length_m: float = 0.0
    simplified_path_length_m: float = 0.0
    start_requested: Optional[np.ndarray] = None
    goal_requested: Optional[np.ndarray] = None
    start_used: Optional[np.ndarray] = None
    goal_used: Optional[np.ndarray] = None
    start_adjustment_m: float = 0.0
    goal_adjustment_m: float = 0.0
    bounds_min: Optional[np.ndarray] = None
    bounds_max: Optional[np.ndarray] = None
    grid_shape: Tuple[int, int, int] = (0, 0, 0)
    candidate_whole_body_safe: Optional[bool] = None
    candidate_minimum_clearance_m: Optional[float] = None
    candidate_critical_component: Optional[str] = None
    candidate_quad_clearance_m: Optional[float] = None
    candidate_magnet_clearance_m: Optional[float] = None
    candidate_cable_clearance_m: Optional[float] = None
    candidate_payload_clearance_m: Optional[float] = None
    candidate_payload_geometry_available: bool = False
    # Active Liu/DecompUtil SFC -> Bernstein minimum-snap result.
    convex_trajectory: Optional[ConvexTrajectoryResult] = None
    convex_corridor: Tuple[ConvexPolyhedron, ...] = tuple()
    corridor_overlap_margins_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=float)
    )
    corridor_error: Optional[str] = None
    occupancy_points: np.ndarray = field(
        default_factory=lambda: np.empty(
            (0, 3),
            dtype=float,
        )
    )
    occupancy_resolution_m: float = 0.0

    corridor_segment_containment_margins_m: np.ndarray = field(
        default_factory=lambda: np.empty(
            0,
            dtype=float,
        )
    )
    corridor_voxel_exclusion_margins_m: np.ndarray = field(
        default_factory=lambda: np.empty(
            0,
            dtype=float,
        )
    )
    corridor_valid: Optional[bool] = None

    @property
    def grid_node_count(self) -> int:
        if len(self.grid_shape) != 3:
            return 0
        return int(self.grid_shape[0] * self.grid_shape[1] * self.grid_shape[2])


def idle_visual_astar_result(
    phase_name: str = "unknown",
    status: str = "waiting",
    message: str = "visual A* waiting for first request",
) -> VisualAStarResult:
    return VisualAStarResult(
        request_id=-1,
        phase_name=phase_name,
        phase_allowed=False,
        status=status,
        message=message,
    )


def _segment_minimum_clearances(
    positions: np.ndarray,
    obstacles: Tuple[StaticSphereObstacle, ...],
    component_radius: float,
    safety_margin: float,
) -> np.ndarray:
    """Return continuous signed clearance for every polyline segment."""
    positions = _positions(positions, "positions")
    if len(positions) < 2:
        return np.empty((0,), dtype=float)
    if not obstacles:
        return np.full(len(positions) - 1, float("inf"), dtype=float)

    starts = positions[:-1]
    ends = positions[1:]
    deltas = ends - starts
    lengths_squared = np.einsum("si,si->s", deltas, deltas)
    centres = np.stack([obstacle.centre for obstacle in obstacles], axis=0)
    obstacle_radii = np.asarray([obstacle.radius for obstacle in obstacles])
    relative = centres[None, :, :] - starts[:, None, :]
    numerators = np.einsum("soi,si->so", relative, deltas)
    fractions = np.zeros_like(numerators)
    valid = lengths_squared > 1e-12
    if np.any(valid):
        fractions[valid] = numerators[valid] / lengths_squared[valid, None]
    np.clip(fractions, 0.0, 1.0, out=fractions)
    closest = starts[:, None, :] + fractions[:, :, None] * deltas[:, None, :]
    distances = np.linalg.norm(closest - centres[None, :, :], axis=2)
    clearances = (
        distances
        - obstacle_radii[None, :]
        - float(component_radius)
        - float(safety_margin)
    )
    return np.min(clearances, axis=1)


def _point_minimum_clearance(
    point: np.ndarray,
    obstacles: Tuple[StaticSphereObstacle, ...],
    component_radius: float,
    safety_margin: float,
) -> float:
    if not obstacles:
        return float("inf")
    centres = np.stack([obstacle.centre for obstacle in obstacles], axis=0)
    radii = np.asarray([obstacle.radius for obstacle in obstacles])
    return float(
        np.min(
            np.linalg.norm(centres - point.reshape(1, 3), axis=1)
            - radii
            - component_radius
            - safety_margin
        )
    )


def _polyline_has_clearance(
    path: np.ndarray,
    request: VisualAStarRequest,
    required_clearance_m: float,
) -> bool:
    if len(path) < 2:
        return True
    clearances = _segment_minimum_clearances(
        path,
        request.obstacles,
        request.drone_radius,
        request.drone_safety_margin,
    )
    return bool(np.all(clearances >= float(required_clearance_m) - 1e-9))


def _simplify_polyline_for_clearance(
    path: np.ndarray,
    request: VisualAStarRequest,
    required_clearance_m: float,
) -> np.ndarray:
    """Greedily shortcut a known-safe polyline without changing its clearance.

    Extend an already planned detour farther along the nominal safe suffix without
    rerunning A* for every later rejoin candidate.  The helper keeps the same
    topology and uses the shared continuous segment-clearance calculation.
    """

    path = _positions(path, "path")
    if len(path) <= 2:
        return path.copy()
    simplified = [path[0].copy()]
    anchor = 0
    final = len(path) - 1
    while anchor < final:
        selected = anchor + 1
        for candidate in range(final, anchor, -1):
            shortcut = np.vstack((path[anchor], path[candidate]))
            if _polyline_has_clearance(shortcut, request, required_clearance_m):
                selected = candidate
                break
        simplified.append(path[selected].copy())
        anchor = selected
    return np.asarray(simplified, dtype=float)


def _select_rejoin_indices(
    request: VisualAStarRequest,
    config: VisualAStarConfig,
    first_blocked_segment: int,
    segment_clearances: np.ndarray,
) -> Tuple[Tuple[int, ...], Optional[int]]:
    positions = request.nominal_positions
    min_samples = int(math.ceil(config.rejoin_min_lookahead_s / request.dt))
    minimum_index = min(
        len(positions) - 1,
        max(first_blocked_segment + 1, first_blocked_segment + min_samples),
    )
    candidates = []
    start = positions[0]
    for index in range(minimum_index, len(positions)):
        if float(np.linalg.norm(positions[index] - start)) < config.rejoin_min_separation_m:
            continue
        if _point_minimum_clearance(
            positions[index],
            request.obstacles,
            request.drone_radius,
            request.drone_safety_margin,
        ) < -1e-9:
            continue
        if index < len(positions) - 1 and np.any(segment_clearances[index:] < -1e-9):
            continue
        candidates.append(index)

    if not candidates:
        return tuple(), None

    preferred_index = first_blocked_segment + int(
        round(config.rejoin_preferred_lookahead_s / request.dt)
    )
    selected = min(candidates, key=lambda index: (abs(index - preferred_index), -index))
    return tuple(candidates), int(selected)


def _planning_bounds(
    config: VisualAStarConfig,
    start: np.ndarray,
    goal: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build a local search box around the actual start-to-goal query."""
    points = np.vstack((start.reshape(1, 3), goal.reshape(1, 3)))

    margin = np.array(
        [
            config.local_margin_xy_m,
            config.local_margin_xy_m,
            config.local_margin_z_m,
        ],
        dtype=float,
    )

    local_min = np.min(points, axis=0) - margin
    local_max = np.max(points, axis=0) + margin

    bounds_min = np.maximum(local_min, config.global_bounds_min)
    bounds_max = np.minimum(local_max, config.global_bounds_max)

    # Preserve requested endpoints if a configured global bound is slightly tight.
    bounds_min = np.minimum(bounds_min, np.minimum(start, goal))
    bounds_max = np.maximum(bounds_max, np.maximum(start, goal))

    minimum_span = 2.0 * config.resolution
    for axis in range(3):
        if bounds_max[axis] - bounds_min[axis] < minimum_span:
            midpoint = 0.5 * (bounds_max[axis] + bounds_min[axis])
            bounds_min[axis] = midpoint - 0.5 * minimum_span
            bounds_max[axis] = midpoint + 0.5 * minimum_span

    return bounds_min, bounds_max


def validate_visual_astar_candidate(
    candidate_positions: np.ndarray,
    request: VisualAStarRequest,
) -> dict:
    """Validate a rendered candidate with the shared whole-body geometry."""
    positions = _positions(candidate_positions, "candidate_positions")
    quad = check_positions_against_static_spheres(
        positions,
        request.obstacles,
        component_name="QUAD",
        component_radius=request.drone_radius,
        safety_margin=request.drone_safety_margin,
        dt=request.dt,
    )
    magnet_positions = positions + request.magnet_offset_from_quad.reshape(1, 3)
    magnet = check_positions_against_static_spheres(
        magnet_positions,
        request.obstacles,
        component_name="MAGNET",
        component_radius=request.magnet_radius,
        safety_margin=request.magnet_safety_margin,
        dt=request.dt,
    )
    cable = check_cable_against_static_spheres(
        positions,
        magnet_positions,
        request.obstacles,
        cable_radius=request.cable_radius,
        safety_margin=request.cable_safety_margin,
        dt=request.dt,
    )

    known = [quad, magnet, cable]
    payload = None
    payload_geometry_available = False
    payload_check_active = bool(
        request.object_attached
        and request.payload_profile is not None
        and request.payload_profile.collision_enabled
    )
    if payload_check_active:
        payload_geometry_available = bool(
            request.payload_geometry_available
            and request.payload_offset_from_magnet is not None
            and request.payload_yaw is not None
        )
        if payload_geometry_available:
            assert request.payload_profile is not None
            assert request.payload_offset_from_magnet is not None
            assert request.payload_yaw is not None
            payload_pose_positions = (
                magnet_positions + request.payload_offset_from_magnet.reshape(1, 3)
            )
            centre_offset = rotate_yaw(
                request.payload_profile.geometry_origin_from_pose,
                request.payload_yaw,
            )
            payload_centres = payload_pose_positions + centre_offset.reshape(1, 3)
            payload_yaws = np.full(len(payload_centres), request.payload_yaw)
            payload = check_yaw_oriented_boxes_against_static_spheres(
                payload_centres,
                payload_yaws,
                request.payload_profile.dimensions,
                request.obstacles,
                safety_margin=request.payload_profile.collision_margin,
                dt=request.dt,
            )
            known.append(payload)

    whole_body_unknown = payload_check_active and not payload_geometry_available
    whole_body_safe: Optional[bool]
    if whole_body_unknown:
        whole_body_safe = None
    else:
        whole_body_safe = all(item.nominal_safe for item in known)
    worst = min(known, key=lambda item: item.minimum_clearance)
    return {
        "whole_body_safe": whole_body_safe,
        "minimum_clearance": float(worst.minimum_clearance),
        "critical_component": worst.component_name,
        "quad_clearance": float(quad.minimum_clearance),
        "magnet_clearance": float(magnet.minimum_clearance),
        "cable_clearance": float(cable.minimum_clearance),
        "payload_clearance": None if payload is None else float(payload.minimum_clearance),
        "payload_geometry_available": payload_geometry_available,
    }

def _inflated_astar_spheres(
    request: VisualAStarRequest,
    route_clearance_m: float,
) -> Tuple[SphereObstacle, ...]:
    """Build the exact configuration-space spheres used by A*."""

    route_clearance_m = max(0.0, float(route_clearance_m))

    return tuple(
        SphereObstacle(
            center=obstacle.centre,
            radius=(
                obstacle.radius
                + request.drone_radius
                + request.drone_safety_margin
                + route_clearance_m
            ),
            name=obstacle.obstacle_id,
        )
        for obstacle in request.obstacles
    )


def _build_whole_body_occupancy_grid(
    config: VisualAStarConfig,
    request: VisualAStarRequest,
    bounds_min: np.ndarray,
    bounds_max: np.ndarray,
    extra_clearance_m: float = 0.0,
) -> VoxelOccupancyGrid:
    """Rasterise the conservative whole-body quad-centre C-space once."""

    extra = max(0.0, float(extra_clearance_m))

    grid = VoxelOccupancyGrid(
        resolution=config.resolution,
        requested_bounds_min=bounds_min,
        requested_bounds_max=bounds_max,
    )

    magnet_offset = request.magnet_offset_from_quad
    cable_length = float(np.linalg.norm(magnet_offset))

    swing_angle = float(config.max_cable_swing_angle_rad)
    payload_relative_centre = None
    payload_bounding_radius = None

    if (
        request.object_attached
        and request.payload_profile is not None
        and request.payload_profile.collision_enabled
        and request.payload_geometry_available
        and request.payload_offset_from_magnet is not None
        and request.payload_yaw is not None
    ):
        payload_relative_centre = (
            magnet_offset
            + request.payload_offset_from_magnet
            + rotate_yaw(
                request.payload_profile.geometry_origin_from_pose,
                request.payload_yaw,
            )
        )

        payload_bounding_radius = (
            0.5
            * float(
                np.linalg.norm(
                    request.payload_profile.dimensions
                )
            )
            + float(request.payload_profile.collision_margin)
            + extra
        )

    # mark the spherical obstacles in the occupancy grid
    for obstacle in request.obstacles:
        # Quad-centre exclusion.
        grid.mark_sphere(
            obstacle.centre,
            obstacle.radius
            + request.drone_radius
            + request.drone_safety_margin
            + extra,
        )

        if cable_length > 1e-9:
            sample_spacing = min(
                0.5 * config.resolution,
                cable_length,
            )

            sample_count = max(
                1,
                int(math.ceil(cable_length / sample_spacing)),
            )

            actual_spacing = cable_length / sample_count

            for sample_index in range(sample_count + 1):
                fraction = sample_index / sample_count
                distance_from_quad = fraction * cable_length

                nominal_offset = fraction * magnet_offset

                # Cone radius from bounded cable swing.
                swing_radius = (
                    distance_from_quad
                    * math.sin(swing_angle)
                )

                # Half-sample inflation fills the space between adjacent
                # cable samples, giving a continuous conservative envelope.
                cable_envelope_radius = (
                    obstacle.radius
                    + request.cable_radius
                    + request.cable_safety_margin
                    + extra
                    + swing_radius
                    + 0.5 * actual_spacing
                )

                grid.mark_sphere(
                    obstacle.centre - nominal_offset,
                    cable_envelope_radius,
                )

        terminal_swing_radius = (
            cable_length * math.sin(swing_angle)
        )

        grid.mark_sphere(
            obstacle.centre - magnet_offset,
            obstacle.radius
            + request.magnet_radius
            + request.magnet_safety_margin
            + extra
            + terminal_swing_radius,
        )

        if (
            payload_relative_centre is not None
            and payload_bounding_radius is not None
        ):
            grid.mark_sphere(
                obstacle.centre - payload_relative_centre,
                obstacle.radius
                + payload_bounding_radius
                + terminal_swing_radius,
            )

    return grid


def _plan_astar_path(
    config: VisualAStarConfig,
    request: VisualAStarRequest,
    start: np.ndarray,
    goal: np.ndarray,
    bounds_min: np.ndarray,
    bounds_max: np.ndarray,
    occupancy_grid: VoxelOccupancyGrid,
    route_clearance_m: float = 0.0,
) -> Tuple[AStarPlanResult, np.ndarray, Optional[str]]:
    """Run A* with one explicit extra route-clearance requirement.

    ``route_clearance_m`` is measured outside the shared quad-centre collision
    boundary.  It is not a second hidden obstacle inflation constant.
    """
    route_clearance_m = max(0.0, float(route_clearance_m))

    planner = VoxelAStar3D(
        resolution=config.resolution,
        max_expansions=config.max_expansions,
        max_planning_time_s=config.max_planning_time_s,
        heuristic_weight=1.5,
        nearest_free_radius_m=config.nearest_free_radius_m,
        line_of_sight_step_fraction=0.45,
        conservative_voxel_inflation=True,
    )

    # Aug 13 22:42 changelog: replaced the old multi-sphere collision checker with just an
    # A* occupancy map checker.
    plan: AStarPlanResult = planner.plan(
        start=start,
        goal=goal,
        bounds_min=bounds_min,
        bounds_max=bounds_max,
        occupancy_grid=occupancy_grid,
    )

    processed_path = plan.simplified_path.copy()
    connector_failure = None
    if plan.success and plan.start_adjustment_m > 1e-9:
        start_connector = np.vstack((plan.start_requested, plan.start_used))
        if not occupancy_grid.segment_is_free(
            start_connector[0],
            start_connector[1],
        ):
            connector_failure = "continuous connector from requested start is blocked"
        else:
            processed_path = np.vstack((plan.start_requested, processed_path))
    if plan.success and connector_failure is None and plan.goal_adjustment_m > 1e-9:
        goal_connector = np.vstack((plan.goal_used, plan.goal_requested))
        if not occupancy_grid.segment_is_free(
            goal_connector[0],
            goal_connector[1],
        ):
            connector_failure = "continuous connector to requested goal is blocked"
        else:
            processed_path = np.vstack((processed_path, plan.goal_requested))
    # Guarantee a canonical polyline for corridor construction and later smoothing:
    # consecutive waypoints must represent non-zero-length segments.
    if len(processed_path) > 1:
        segment_lengths = np.linalg.norm(
            np.diff(processed_path, axis=0),
            axis=1,
        )

        keep = np.concatenate(
            ([True], segment_lengths > 1e-8)
        )

        processed_path = processed_path[keep]
    return plan, processed_path, connector_failure



def _predict_target_goal(
    config: VisualAStarConfig,
    request: VisualAStarRequest,
    start: np.ndarray,
) -> Tuple[Optional[np.ndarray], float]:
    """Predict a simple constant-velocity rendezvous goal.
    predict where the target will be by the time we arrive, and also return the ETA of our
    drone to get there"""
    if request.target_position is None:
        return None, 0.0

    target_position = request.target_position.copy()

    target_velocity = (
        np.zeros(3, dtype=float)
        if request.target_velocity_mean is None
        else request.target_velocity_mean.copy()
    )

    range_m = float(np.linalg.norm(target_position - start))
    speed_mps = float(config.candidate_speed_mps)

    if speed_mps <= 1e-9:
        eta_s = 0.0
    else:
        eta_s = range_m / speed_mps

    eta_s = min(eta_s, float(config.target_prediction_max_s))

    goal = target_position + target_velocity * eta_s
    return goal, eta_s

"""draw and whole-bdoy check the entire spatial path"""
def _render_spatial_candidate(
    path: np.ndarray,
    request: VisualAStarRequest,
    config: VisualAStarConfig,
) -> np.ndarray:
    """Render the complete spatial route at the configured candidate speed."""
    path_length_m = VoxelAStar3D.path_length(path)
    speed_mps = max(1e-9, float(config.candidate_speed_mps))

    duration_s = path_length_m / speed_mps
    num_samples = max(
        2,
        int(math.ceil(duration_s / request.dt)) + 1,
    )

    rendered = VoxelAStar3D.resample_polyline(
        path,
        num_samples=num_samples,
        dt=request.dt,
        speed=speed_mps,
    )

    rendered[0] = path[0]
    rendered[-1] = path[-1]

    return rendered


def _build_validated_corridor(
    path: np.ndarray,
    occupied_points: np.ndarray,
    resolution: float,
):
    """Build and validate one DecompUtil-style corridor for a spatial skeleton."""

    corridor = tuple()

    overlap_margins = np.empty(0, dtype=float)
    segment_margins = np.empty(0, dtype=float)
    point_margins = np.empty(0, dtype=float)
    voxel_margins = np.empty(0, dtype=float)

    corridor_valid = None
    corridor_error = None

    try:
        corridor = build_convex_corridor(
            path,
            occupied_points,
            local_bbox=[
                resolution,
                0.50,
                0.50,
            ],
        )

        segment_margins = (
            corridor_segment_containment_margins(
                corridor,
            )
        )

        overlap_margins = (
            corridor_overlap_margins(
                path,
                corridor,
            )
        )

        point_margins = (
            corridor_point_exclusion_margins(
                corridor,
                occupied_points,
            )
        )

        # Stricter diagnostic only. The SFC is point-cloud based.
        voxel_margins = (
            corridor_voxel_exclusion_margins(
                corridor,
                occupied_points,
                resolution,
            )
        )

        tolerance = 1e-8

        corridor_valid = bool(
            np.all(segment_margins >= -tolerance)
            and np.all(overlap_margins >= -tolerance)
            and np.all(point_margins >= -tolerance)
        )

        if not corridor_valid:
            segment_min = (
                float(np.min(segment_margins))
                if segment_margins.size
                else float("inf")
            )

            overlap_min = (
                float(np.min(overlap_margins))
                if overlap_margins.size
                else float("inf")
            )

            point_min = (
                float(np.min(point_margins))
                if point_margins.size
                else float("inf")
            )

            corridor_error = (
                "corridor validation failed: "
                f"segment={segment_min:.4f} m, "
                f"overlap={overlap_min:.4f} m, "
                f"point={point_min:.4f} m"
            )

    except (
        ValueError,
        RuntimeError,
        FloatingPointError,
        np.linalg.LinAlgError,
    ) as exc:
        corridor_error = str(exc)

    return (
        corridor,
        overlap_margins,
        segment_margins,
        voxel_margins,
        corridor_valid,
        corridor_error,
    )

def _generate_corridor_trajectory(
    path: np.ndarray,
    corridor: Tuple[ConvexPolyhedron, ...],
    request: VisualAStarRequest,
    astar_config: VisualAStarConfig,
    trajectory_config: Optional[ConvexTrajectoryConfig],
):
    """Generate the candidate eventually supplied to the execution safety gate."""

    if (
        trajectory_config is None
        or not trajectory_config.enabled
    ):
        candidate_positions = (
            _render_spatial_candidate(
                path,
                request,
                astar_config,
            )
        )

        validation = (
            validate_visual_astar_candidate(
                candidate_positions,
                request,
            )
        )

        return None, candidate_positions, validation

    if (
        request.planning_start_velocity is None
        or request.planning_start_acceleration is None
    ):
        result = ConvexTrajectoryResult(
            success=False,
            status="START_STATE_UNAVAILABLE",
            message=(
                "measured planning velocity/acceleration "
                "is unavailable"
            ),
        )

        return (
            result,
            np.empty((0, 3), dtype=float),
            None,
        )

    goal_velocity = (
        np.zeros(3, dtype=float)
        if request.target_velocity_mean is None
        else request.target_velocity_mean.copy()
    )

    goal_acceleration = (
        resolve_target_acceleration(
            trajectory_config,
            request.target_acceleration_mean,
        )
    )

    trajectory = (
        solve_convex_corridor_trajectory(
            corridor=corridor,

            start_position=path[0],
            start_velocity=(
                request.planning_start_velocity
            ),
            start_acceleration=(
                request.planning_start_acceleration
            ),

            goal_position=path[-1],
            goal_velocity=goal_velocity,
            goal_acceleration=goal_acceleration,

            config=trajectory_config,
            dt=request.dt,
        )
    )

    if not trajectory.success:
        return (
            trajectory,
            np.empty((0, 3), dtype=float),
            None,
        )

    candidate_positions = (
        trajectory.positions.copy()
    )

    validation = (
        validate_visual_astar_candidate(
            candidate_positions,
            request,
        )
    )

    return (
        trajectory,
        candidate_positions,
        validation,
    )

def _whole_body_result_fields(
    validation: dict,
) -> dict:
    return {
        "candidate_whole_body_safe": (
            validation["whole_body_safe"]
        ),
        "candidate_minimum_clearance_m": (
            validation["minimum_clearance"]
        ),
        "candidate_critical_component": (
            validation["critical_component"]
        ),
        "candidate_quad_clearance_m": (
            validation["quad_clearance"]
        ),
        "candidate_magnet_clearance_m": (
            validation["magnet_clearance"]
        ),
        "candidate_cable_clearance_m": (
            validation["cable_clearance"]
        ),
        "candidate_payload_clearance_m": (
            validation["payload_clearance"]
        ),
        "candidate_payload_geometry_available": (
            validation["payload_geometry_available"]
        ),
    }

def compute_visual_astar(
    config: VisualAStarConfig,
    request: VisualAStarRequest,
    trajectory_config: Optional[ConvexTrajectoryConfig] = None,
) -> VisualAStarResult:
    """Compute one passive candidate plan from an immutable request snapshot."""
    base = dict(
        request_id=request.request_id,
        phase_name=request.phase_name,
        phase_allowed=request.phase_allowed,
    )

    """Condition  and error checking so that A* isn't unnecessarily called"""
    if not config.enabled:
        return VisualAStarResult(
            **base,
            status="disabled",
            message="visual A* disabled",
        )
    if not request.phase_allowed:
        return VisualAStarResult(
            **base,
            status="phase_not_allowed",
            message="visual A* is restricted to transfer phases",
        )
    
    # The new planner is defined by the measured quad position and the moving
    # destination, not by the old nominal trajectory.
    if request.planning_start_position is None:
        return VisualAStarResult(
            **base,
            status="planning_start_unavailable",
            message="measured planning start position is unavailable",
        )

    start = request.planning_start_position.copy()

    goal, target_eta_s = _predict_target_goal(
        config,
        request,
        start,
    )
    if goal is None:
        return VisualAStarResult(
            **base,
            status="target_unavailable",
            message="moving-target position is unavailable",
        )

    # Keep the existing trajectory-clearance margin as an extra geometric
    # clearance requirement when it is configured.
    route_clearance_m = 0.0
    if trajectory_config is not None and trajectory_config.enabled:
        route_clearance_m = max(
            0.0,
            float(trajectory_config.planning_clearance_margin_m),
        )

    # added august 13 during the development spree
    # The direct route is blocked, so ask A* for an alternative topology
    # between the same measured start and predicted moving-target goal.
    bounds_min, bounds_max = _planning_bounds(
        config,
        start,
        goal,
    )

    occupancy_grid = _build_whole_body_occupancy_grid(
        config,
        request,
        bounds_min,
        bounds_max,
        route_clearance_m,
    )
    occupied_points = occupancy_grid.occupied_points()

    # august 13 22:41 change - replaced the below function to check for the entire
    # quad assembly, magnet/object included
    direct_path = np.vstack((start, goal))
    direct_is_clear = occupancy_grid.segment_is_free(
        start,
        goal,
        step_fraction=0.45,
    )

    if direct_is_clear:
        path_length_m = (
            VoxelAStar3D.path_length(
                direct_path,
            )
        )

        (
            corridor,
            overlap_margins,
            segment_margins,
            voxel_margins,
            corridor_valid,
            corridor_error,
        ) = _build_validated_corridor(
            direct_path,
            occupied_points,
            config.resolution,
        )

        shaping_enabled = bool(
            trajectory_config is not None
            and trajectory_config.enabled
        )

        if shaping_enabled and corridor_valid is not True:
            return VisualAStarResult(
                **base,
                status="corridor_failed",
                message=(
                    "direct path is clear but its convex "
                    "safe-flight corridor is invalid: "
                    f"{corridor_error}"
                ),
                success=False,
                nominal_blocked=False,
                simplified_path=direct_path.copy(),
                raw_path_length_m=path_length_m,
                simplified_path_length_m=path_length_m,
                start_requested=start.copy(),
                goal_requested=goal.copy(),
                start_used=start.copy(),
                goal_used=goal.copy(),

                convex_corridor=corridor,
                corridor_overlap_margins_m=(
                    overlap_margins
                ),
                corridor_error=corridor_error,

                occupancy_points=occupied_points,
                occupancy_resolution_m=(
                    config.resolution
                ),

                corridor_segment_containment_margins_m=(
                    segment_margins
                ),
                corridor_voxel_exclusion_margins_m=(
                    voxel_margins
                ),
                corridor_valid=corridor_valid,
            )

        (
            trajectory,
            candidate_positions,
            validation,
        ) = _generate_corridor_trajectory(
            direct_path,
            corridor,
            request,
            config,
            trajectory_config,
        )

        if (
            trajectory is not None
            and not trajectory.success
        ):
            return VisualAStarResult(
                **base,
                status=(
                    "trajectory_"
                    + trajectory.status.lower()
                ),
                message=(
                    "direct path and corridor are valid "
                    "but trajectory generation failed: "
                    + trajectory.message
                ),
                success=False,
                nominal_blocked=False,

                simplified_path=direct_path.copy(),
                raw_path_length_m=path_length_m,
                simplified_path_length_m=path_length_m,

                start_requested=start.copy(),
                goal_requested=goal.copy(),
                start_used=start.copy(),
                goal_used=goal.copy(),

                convex_trajectory=trajectory,
                convex_corridor=corridor,
                corridor_overlap_margins_m=(
                    overlap_margins
                ),
                corridor_error=corridor_error,

                occupancy_points=occupied_points,
                occupancy_resolution_m=(
                    config.resolution
                ),

                corridor_segment_containment_margins_m=(
                    segment_margins
                ),
                corridor_voxel_exclusion_margins_m=(
                    voxel_margins
                ),
                corridor_valid=corridor_valid,
            )

        assert validation is not None

        whole_body = validation[
            "whole_body_safe"
        ]

        if shaping_enabled and whole_body is not True:
            failure_status = (
                "trajectory_whole_body_collision"
                if whole_body is False
                else "trajectory_whole_body_unknown"
            )

            return VisualAStarResult(
                **base,
                status=failure_status,
                message=(
                    "smoothed direct trajectory rejected "
                    "by whole-body validation"
                ),
                success=False,
                nominal_blocked=False,

                simplified_path=direct_path.copy(),
                candidate_positions=(
                    candidate_positions
                ),

                raw_path_length_m=path_length_m,
                simplified_path_length_m=path_length_m,

                start_requested=start.copy(),
                goal_requested=goal.copy(),
                start_used=start.copy(),
                goal_used=goal.copy(),

                convex_trajectory=trajectory,
                convex_corridor=corridor,
                corridor_overlap_margins_m=(
                    overlap_margins
                ),
                corridor_error=corridor_error,

                occupancy_points=occupied_points,
                occupancy_resolution_m=(
                    config.resolution
                ),

                corridor_segment_containment_margins_m=(
                    segment_margins
                ),
                corridor_voxel_exclusion_margins_m=(
                    voxel_margins
                ),
                corridor_valid=corridor_valid,

                **_whole_body_result_fields(
                    validation
                ),
            )

        return VisualAStarResult(
            **base,
            status="direct_clear",
            message=(
                "direct path to predicted target is clear; "
                f"target ETA={target_eta_s:.3f} s"
                + (
                    "; convex minimum-snap trajectory generated"
                    if trajectory is not None
                    else ""
                )
            ),
            success=True,
            nominal_blocked=False,

            simplified_path=direct_path.copy(),
            candidate_positions=(
                candidate_positions
            ),

            raw_path_length_m=path_length_m,
            simplified_path_length_m=path_length_m,

            start_requested=start.copy(),
            goal_requested=goal.copy(),
            start_used=start.copy(),
            goal_used=goal.copy(),

            convex_trajectory=trajectory,
            convex_corridor=corridor,
            corridor_overlap_margins_m=(
                overlap_margins
            ),
            corridor_error=corridor_error,

            occupancy_points=occupied_points,
            occupancy_resolution_m=(
                config.resolution
            ),

            corridor_segment_containment_margins_m=(
                segment_margins
            ),
            corridor_voxel_exclusion_margins_m=(
                voxel_margins
            ),
            corridor_valid=corridor_valid,

            **_whole_body_result_fields(
                validation
            ),
        )



    plan, processed_path, connector_failure = _plan_astar_path(
        config,
        request,
        start,
        goal,
        bounds_min,
        bounds_max,
        occupancy_grid,
        route_clearance_m=route_clearance_m,
    )

    processed_length = VoxelAStar3D.path_length(processed_path)

    common = dict(
        **base,
        # This field is retained for compatibility for now. In the new
        # architecture it means that the direct start-to-goal route was blocked.
        nominal_blocked=True,
        planning_time_ms=1000.0 * plan.planning_time_s,
        expanded_nodes=plan.expanded_nodes,
        generated_nodes=plan.generated_nodes,
        raw_path_length_m=plan.raw_path_length_m,
        simplified_path_length_m=processed_length,
        start_requested=plan.start_requested.copy(),
        goal_requested=plan.goal_requested.copy(),
        start_used=plan.start_used.copy(),
        goal_used=plan.goal_used.copy(),
        start_adjustment_m=plan.start_adjustment_m,
        goal_adjustment_m=plan.goal_adjustment_m,
        bounds_min=plan.bounds_min.copy(),
        bounds_max=plan.bounds_max.copy(),
        grid_shape=plan.grid_shape,
        raw_path=plan.raw_path.copy(),
        simplified_path=processed_path.copy(),
    )

    if not plan.success:
        return VisualAStarResult(
            **common,
            status="search_failed",
            message=f"A* failed while planning to predicted target: {plan.reason}",
            success=False,
        )

    if connector_failure is not None:
        return VisualAStarResult(
            **common,
            status="connector_failed",
            message=f"A* lattice path found but {connector_failure}",
            success=False,
        )

    # corridor = tuple()
    # overlap_margins = np.empty(0, dtype=float)
    # corridor_error = None
    (
        corridor,
        overlap_margins,
        segment_margins,
        voxel_margins,
        corridor_valid,
        corridor_error,
    ) = _build_validated_corridor(
        processed_path,
        occupied_points,
        config.resolution,
    )

    shaping_enabled = bool(
        trajectory_config is not None
        and trajectory_config.enabled
    )

    if shaping_enabled and corridor_valid is not True:
        return VisualAStarResult(
            **common,
            status="corridor_failed",
            message=(
                "A* route found but its convex "
                "safe-flight corridor is invalid: "
                f"{corridor_error}"
            ),
            success=False,

            convex_corridor=corridor,
            corridor_overlap_margins_m=(
                overlap_margins
            ),
            corridor_error=corridor_error,

            occupancy_points=occupied_points,
            occupancy_resolution_m=(
                config.resolution
            ),

            corridor_segment_containment_margins_m=(
                segment_margins
            ),
            corridor_voxel_exclusion_margins_m=(
                voxel_margins
            ),
            corridor_valid=corridor_valid,
        )

    (
        trajectory,
        candidate_positions,
        validation,
    ) = _generate_corridor_trajectory(
        processed_path,
        corridor,
        request,
        config,
        trajectory_config,
    )

    if (
        trajectory is not None
        and not trajectory.success
    ):
        return VisualAStarResult(
            **common,
            status=(
                "trajectory_"
                + trajectory.status.lower()
            ),
            message=(
                "A* route and corridor are valid "
                "but trajectory generation failed: "
                + trajectory.message
            ),
            success=False,

            convex_trajectory=trajectory,
            convex_corridor=corridor,
            corridor_overlap_margins_m=(
                overlap_margins
            ),
            corridor_error=corridor_error,

            occupancy_points=occupied_points,
            occupancy_resolution_m=(
                config.resolution
            ),

            corridor_segment_containment_margins_m=(
                segment_margins
            ),
            corridor_voxel_exclusion_margins_m=(
                voxel_margins
            ),
            corridor_valid=corridor_valid,
        )

    assert validation is not None

    whole_body = validation[
        "whole_body_safe"
    ]

    if shaping_enabled and whole_body is not True:
        failure_status = (
            "trajectory_whole_body_collision"
            if whole_body is False
            else "trajectory_whole_body_unknown"
        )

        return VisualAStarResult(
            **common,
            status=failure_status,
            message=(
                "smoothed A* trajectory rejected "
                "by whole-body validation"
            ),
            success=False,

            candidate_positions=(
                candidate_positions
            ),

            convex_trajectory=trajectory,
            convex_corridor=corridor,
            corridor_overlap_margins_m=(
                overlap_margins
            ),
            corridor_error=corridor_error,

            occupancy_points=occupied_points,
            occupancy_resolution_m=(
                config.resolution
            ),

            corridor_segment_containment_margins_m=(
                segment_margins
            ),
            corridor_voxel_exclusion_margins_m=(
                voxel_margins
            ),
            corridor_valid=corridor_valid,

            **_whole_body_result_fields(
                validation
            ),
        )

    return VisualAStarResult(
        **common,
        status="success",
        message=(
            "A* route to predicted target found; "
            f"target ETA={target_eta_s:.3f} s"
            + (
                "; convex minimum-snap trajectory generated"
                if trajectory is not None
                else ""
            )
        ),
        success=True,

        candidate_positions=(
            candidate_positions
        ),

        convex_trajectory=trajectory,
        convex_corridor=corridor,
        corridor_overlap_margins_m=(
            overlap_margins
        ),
        corridor_error=corridor_error,

        occupancy_points=occupied_points,
        occupancy_resolution_m=(
            config.resolution
        ),

        corridor_segment_containment_margins_m=(
            segment_margins
        ),
        corridor_voxel_exclusion_margins_m=(
            voxel_margins
        ),
        corridor_valid=corridor_valid,

        **_whole_body_result_fields(
            validation
        ),
    )
