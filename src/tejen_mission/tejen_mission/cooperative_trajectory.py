"""ROS-independent cooperative trajectory primitives for the dynamic planner.

R2 deliberately keeps this module small.  It represents time-parametric point
trajectories and a simple yawing rigid-body trajectory for the basket.  The
planner-facing convention is relative time ``t`` in seconds from the planning
epoch.  ROS timestamps are intentionally not used here.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

import numpy as np


Vector3 = np.ndarray


def _vec3(value: Vector3 | tuple[float, float, float] | list[float]) -> Vector3:
    array = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(array)):
        raise ValueError("Expected a finite 3-vector")
    return array.copy()


@dataclass(frozen=True)
class TrajectoryState:
    """Position, velocity and acceleration of a point at one time."""

    position: Vector3
    velocity: Vector3
    acceleration: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec3(self.position))
        object.__setattr__(self, "velocity", _vec3(self.velocity))
        object.__setattr__(self, "acceleration", _vec3(self.acceleration))


@dataclass(frozen=True)
class RigidBodyState:
    """R2 basket state.

    The production interface will eventually carry full 3-D attitude.  For the
    first standalone tests we only need yaw, which already matches the current
    fake cooperative transport world.  Keeping the rigid-body state separate
    makes that future extension local rather than changing every point-trajectory
    consumer.
    """

    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw: float = 0.0
    yaw_rate: float = 0.0
    yaw_acceleration: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec3(self.position))
        object.__setattr__(self, "velocity", _vec3(self.velocity))
        object.__setattr__(self, "acceleration", _vec3(self.acceleration))
        for name in ("yaw", "yaw_rate", "yaw_acceleration"):
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"{name} must be finite")


class PointTrajectory(Protocol):
    def state(self, t: float) -> TrajectoryState:
        """Evaluate the trajectory at relative time ``t`` [s]."""


class RigidBodyTrajectory(Protocol):
    def state(self, t: float) -> RigidBodyState:
        """Evaluate the rigid-body trajectory at relative time ``t`` [s]."""


@dataclass(frozen=True)
class StaticRigidBodyTrajectory:
    position: Vector3
    yaw: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec3(self.position))
        if not math.isfinite(float(self.yaw)):
            raise ValueError("yaw must be finite")

    def state(self, t: float) -> RigidBodyState:
        del t
        zero = np.zeros(3, dtype=float)
        return RigidBodyState(
            position=self.position,
            velocity=zero,
            acceleration=zero,
            yaw=float(self.yaw),
            yaw_rate=0.0,
            yaw_acceleration=0.0,
        )


@dataclass(frozen=True)
class SinusoidalLineTrajectory:
    """Analytic line-motion test obstacle.

    Defaults are the agreed standalone scenario:

        x(t) = 1.0
        y(t) = 0.70 + 1.40 sin(0.20 t)
        z(t) = 1.50
    """

    x: float = 1.0
    y_center: float = 0.70
    y_amplitude: float = 1.40
    omega: float = 0.20
    z: float = 1.50
    phase: float = -math.pi / 6.0 * 0.0

    def __post_init__(self) -> None:
        values = (self.x, self.y_center, self.y_amplitude, self.omega, self.z, self.phase)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Sinusoidal trajectory parameters must be finite")
        if self.omega <= 0.0:
            raise ValueError("omega must be positive")

    def state(self, t: float) -> TrajectoryState:
        t = float(t)
        angle = self.omega * t + self.phase
        position = np.array(
            [
                self.x,
                self.y_center + self.y_amplitude * math.sin(angle),
                self.z,
            ],
            dtype=float,
        )
        velocity = np.array(
            [0.0, self.y_amplitude * self.omega * math.cos(angle), 0.0],
            dtype=float,
        )
        acceleration = np.array(
            [0.0, -self.y_amplitude * self.omega**2 * math.sin(angle), 0.0],
            dtype=float,
        )
        return TrajectoryState(position, velocity, acceleration)

    def y_extrema(self, t0: float, t1: float) -> tuple[float, float]:
        """Return exact min/max y over the closed interval [t0, t1]."""

        t0 = float(t0)
        t1 = float(t1)
        if t1 < t0:
            raise ValueError("Expected t1 >= t0")

        candidates = [t0, t1]
        # dy/dt = A*omega*cos(omega*t + phase) = 0
        # => omega*t + phase = pi/2 + k*pi.
        k_min = math.ceil((self.omega * t0 + self.phase - math.pi / 2.0) / math.pi)
        k_max = math.floor((self.omega * t1 + self.phase - math.pi / 2.0) / math.pi)
        for k in range(k_min, k_max + 1):
            t_ext = (math.pi / 2.0 + k * math.pi - self.phase) / self.omega
            if t0 - 1e-12 <= t_ext <= t1 + 1e-12:
                candidates.append(t_ext)

        ys = [self.state(t).position[1] for t in candidates]
        return float(min(ys)), float(max(ys))


@dataclass(frozen=True)
class RingNetGeometry:
    """Pure geometry contract for the 50 cm pitch-circle ring/net assembly.

    The ring frame origin is the centre of the steel attachment-plate plane.
    Plate 0 lies on +x_R and plate indices increase counter-clockwise when
    viewed from +z_R. The conservative collision envelope is deliberately
    independent of the attachment pitch circle and may extend only below the
    attachment plane.
    """

    plate_count: int = 12
    plate_pitch_diameter_m: float = 0.50
    plate_diameter_m: float = 0.06
    collision_outer_diameter_m: float = 0.56
    collision_top_offset_m: float = 0.0
    collision_bottom_offset_m: float = -0.27
    angle_zero_rad: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.plate_count, bool) or int(self.plate_count) != self.plate_count or int(self.plate_count) <= 0:
            raise ValueError("plate_count must be a positive integer")
        values = (
            self.plate_pitch_diameter_m,
            self.plate_diameter_m,
            self.collision_outer_diameter_m,
            self.collision_top_offset_m,
            self.collision_bottom_offset_m,
            self.angle_zero_rad,
        )
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("ring geometry values must be finite")
        if self.plate_pitch_diameter_m <= 0.0:
            raise ValueError("plate_pitch_diameter_m must be positive")
        if self.plate_diameter_m <= 0.0:
            raise ValueError("plate_diameter_m must be positive")
        if self.collision_outer_diameter_m <= 0.0:
            raise ValueError("collision_outer_diameter_m must be positive")
        minimum_outer_diameter = self.plate_pitch_diameter_m + self.plate_diameter_m
        if self.collision_outer_diameter_m + 1e-12 < minimum_outer_diameter:
            raise ValueError(
                "collision_outer_diameter_m must enclose the pitch circle and steel plates"
            )
        if self.collision_top_offset_m < 0.0 or self.collision_bottom_offset_m > 0.0:
            raise ValueError("ring collision envelope must include the attachment-plate plane")
        if self.collision_top_offset_m < self.collision_bottom_offset_m:
            raise ValueError("collision_top_offset_m must be >= collision_bottom_offset_m")
        if self.collision_top_offset_m == self.collision_bottom_offset_m:
            raise ValueError("ring collision envelope must have positive height")

    @property
    def plate_pitch_radius_m(self) -> float:
        return 0.5 * float(self.plate_pitch_diameter_m)

    @property
    def collision_half_extents(self) -> Vector3:
        half_xy = 0.5 * float(self.collision_outer_diameter_m)
        half_z = 0.5 * (
            float(self.collision_top_offset_m) - float(self.collision_bottom_offset_m)
        )
        return np.array([half_xy, half_xy, half_z], dtype=float)

    @property
    def collision_center_offset(self) -> Vector3:
        centre_z = 0.5 * (
            float(self.collision_top_offset_m) + float(self.collision_bottom_offset_m)
        )
        return np.array([0.0, 0.0, centre_z], dtype=float)

    def _check_plate_index(self, plate_index: int) -> int:
        index = int(plate_index)
        if index < 0 or index >= int(self.plate_count):
            raise ValueError(f"plate_index must be in [0, {int(self.plate_count) - 1}]")
        return index

    def plate_body_offset(self, plate_index: int) -> Vector3:
        index = self._check_plate_index(plate_index)
        theta = float(self.angle_zero_rad) + 2.0 * math.pi * index / int(self.plate_count)
        radius = self.plate_pitch_radius_m
        return np.array(
            [radius * math.cos(theta), radius * math.sin(theta), 0.0], dtype=float
        )

    def plate_outward_direction_body(self, plate_index: int) -> Vector3:
        offset = self.plate_body_offset(plate_index)
        direction = np.array([offset[0], offset[1], 0.0], dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 0.0:
            raise ValueError("attachment plate has undefined radial direction")
        return direction / norm

    def plate_inward_approach_direction_body(self, plate_index: int) -> Vector3:
        return -self.plate_outward_direction_body(plate_index)

    def radially_tensioned_endpoint_body(
        self,
        plate_index: int,
        *,
        cable_length_m: float,
        angle_from_vertical_rad: float,
    ) -> Vector3:
        """Return an ideal straight-cable upper endpoint in the ring frame.

        This is a pure geometry helper for preview/assignment work, not a collision
        envelope.  The cable starts at the selected attachment plate and is pulled
        radially outward by ``angle_from_vertical_rad``.
        """
        length = float(cable_length_m)
        angle = float(angle_from_vertical_rad)
        if not math.isfinite(length) or length <= 0.0:
            raise ValueError("cable_length_m must be positive and finite")
        if not math.isfinite(angle) or angle < 0.0 or angle > 0.5 * math.pi:
            raise ValueError("angle_from_vertical_rad must lie in [0, pi/2]")

        plate = self.plate_body_offset(plate_index)
        outward = self.plate_outward_direction_body(plate_index)
        horizontal = length * math.sin(angle)
        vertical = length * math.cos(angle)
        return plate + outward * horizontal + np.array([0.0, 0.0, vertical], dtype=float)

    @staticmethod
    def _proper_rotation_matrix(value: np.ndarray, *, name: str) -> np.ndarray:
        rotation = np.asarray(value, dtype=float)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError(f"{name} must be a finite 3x3 matrix")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9) or not math.isclose(
            float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError(f"{name} must be a proper rotation matrix")
        return rotation.copy()

    def plate_rotation_body(self, plate_index: int) -> np.ndarray:
        # R_RP: x radial outward, y tangent in increasing index direction, z ring normal.
        index = self._check_plate_index(plate_index)
        theta = float(self.angle_zero_rad) + 2.0 * math.pi * index / int(self.plate_count)
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def plate_rotation_world(
        self,
        *,
        rotation_world_from_ring: np.ndarray,
        plate_index: int,
    ) -> np.ndarray:
        rotation_world_from_ring = self._proper_rotation_matrix(
            rotation_world_from_ring, name="rotation_world_from_ring"
        )
        return rotation_world_from_ring @ self.plate_rotation_body(plate_index)

    def plate_position_world(
        self,
        *,
        ring_position: Vector3,
        rotation_world_from_ring: np.ndarray,
        plate_index: int,
    ) -> Vector3:
        position = _vec3(ring_position)
        rotation = np.asarray(rotation_world_from_ring, dtype=float)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("rotation_world_from_ring must be a finite 3x3 matrix")
        # Reject grossly invalid transforms without imposing a fragile exact-orthogonality check.
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9) or not math.isclose(
            float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("rotation_world_from_ring must be a proper rotation matrix")
        return position + rotation @ self.plate_body_offset(plate_index)


def rigid_body_offset_state(
    body_state: RigidBodyState,
    body_offset: Vector3 | tuple[float, float, float] | list[float],
) -> TrajectoryState:
    """Return exact yaw-only rigid-body kinematics for a fixed body-frame point.

    The offset rotates with the rigid body.  Velocity therefore includes the
    tangential ``omega x r`` term and acceleration includes the corresponding
    angular-acceleration and centripetal terms.
    """

    offset = _vec3(body_offset)
    c = math.cos(body_state.yaw)
    s = math.sin(body_state.yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    r_world = rotation @ offset

    omega = np.array([0.0, 0.0, body_state.yaw_rate], dtype=float)
    alpha = np.array([0.0, 0.0, body_state.yaw_acceleration], dtype=float)

    position = body_state.position + r_world
    velocity = body_state.velocity + np.cross(omega, r_world)
    acceleration = (
        body_state.acceleration
        + np.cross(alpha, r_world)
        + np.cross(omega, np.cross(omega, r_world))
    )
    return TrajectoryState(position, velocity, acceleration)


@dataclass(frozen=True)
class AttachmentPointTrajectory:
    """Trajectory of one known attachment point on a yawing circular basket."""

    basket: RigidBodyTrajectory
    attachment_id: int
    attachment_count: int = 12
    basket_radius_m: float = 0.25
    attachment_z_offset_m: float = 0.0
    angle_zero_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.attachment_count <= 0:
            raise ValueError("attachment_count must be positive")
        if not 0 <= int(self.attachment_id) < int(self.attachment_count):
            raise ValueError(
                f"attachment_id must be in [0, {self.attachment_count - 1}]"
            )
        if self.basket_radius_m <= 0.0:
            raise ValueError("basket_radius_m must be positive")

    @property
    def body_offset(self) -> Vector3:
        angle = self.angle_zero_rad + 2.0 * math.pi * self.attachment_id / self.attachment_count
        return np.array(
            [
                self.basket_radius_m * math.cos(angle),
                self.basket_radius_m * math.sin(angle),
                self.attachment_z_offset_m,
            ],
            dtype=float,
        )

    def state(self, t: float) -> TrajectoryState:
        return rigid_body_offset_state(self.basket.state(t), self.body_offset)


@dataclass(frozen=True)
class OffsetPointTrajectory:
    """Constant world-frame offset from another point trajectory."""

    base: PointTrajectory
    offset: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "offset", _vec3(self.offset))

    def state(self, t: float) -> TrajectoryState:
        base_state = self.base.state(t)
        return TrajectoryState(
            position=base_state.position + self.offset,
            velocity=base_state.velocity,
            acceleration=base_state.acceleration,
        )


def basket_attachment_positions(
    basket_state: RigidBodyState,
    *,
    attachment_count: int = 12,
    basket_radius_m: float = 0.25,
    attachment_z_offset_m: float = 0.0,
    angle_zero_rad: float = 0.0,
) -> np.ndarray:
    """Return all attachment positions for visualization/testing."""

    if attachment_count <= 0:
        raise ValueError("attachment_count must be positive")
    points = []
    c = math.cos(basket_state.yaw)
    s = math.sin(basket_state.yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    for index in range(attachment_count):
        theta = angle_zero_rad + 2.0 * math.pi * index / attachment_count
        local = np.array(
            [
                basket_radius_m * math.cos(theta),
                basket_radius_m * math.sin(theta),
                attachment_z_offset_m,
            ],
            dtype=float,
        )
        points.append(basket_state.position + rotation @ local)
    return np.asarray(points, dtype=float)
