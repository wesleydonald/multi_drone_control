"""Pure M2B grounded-start geometry.

This module defines only the commissioning initial condition. It does not alter
attachment geometry, RingNetGeometry, or the real X3 model. The runtime helper
uses the returned poses while Gazebo is paused, before the first physics step.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from tejen_mission.cooperative_trajectory import RingNetGeometry


def _yaw_rotation(yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def quaternion_xyzw_from_two_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Shortest-arc quaternion rotating ``source`` onto ``target``."""

    a = np.asarray(source, dtype=float).reshape(3)
    b = np.asarray(target, dtype=float).reshape(3)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot < -1.0 + 1e-10:
        helper = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(a, helper))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0], dtype=float)
        axis = np.cross(a, helper)
        axis /= np.linalg.norm(axis)
        return np.array([axis[0], axis[1], axis[2], 0.0], dtype=float)
    xyz = np.cross(a, b)
    q = np.array([xyz[0], xyz[1], xyz[2], 1.0 + dot], dtype=float)
    q /= np.linalg.norm(q)
    return q


def yaw_quaternion_xyzw(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    return np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=float)


@dataclass(frozen=True)
class GroundStartConfig:
    """Physical seed values from the current Sep-10 simulation assets."""

    ring_x_m: float = 0.0
    ring_y_m: float = 0.0
    # Ring segment centre is -0.020 m with 0.030 m thickness, so +0.035 m
    # places the conservative lowest ring collision exactly on z=0.
    ring_z_m: float = 0.035
    ring_yaw_rad: float = 0.0
    body_z_m: float = 0.105
    body_collision_half_height_m: float = 0.100
    # Current SDF frame geometry: tether_rod origin is +0.010 m from the body,
    # while base_to_tether joint is -0.050 m in the tether_rod frame. In the
    # model's zero configuration this places the actual ball-joint anchor at
    # body z - 0.040 m and leaves 0.450 m from joint anchor to magnet centre.
    tether_origin_initial_body_z_m: float = 0.010
    base_joint_offset_tether_z_m: float = -0.050
    tether_length_m: float = 0.500
    magnet_radius_m: float = 0.025
    initial_body_yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        values = tuple(float(getattr(self, name)) for name in self.__dataclass_fields__)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Ground-start values must be finite")
        if self.tether_length_m <= 0.0 or self.magnet_radius_m <= 0.0:
            raise ValueError("Tether length and magnet radius must be positive")
        if self.body_z_m < self.body_collision_half_height_m:
            raise ValueError("X3 body collision would begin below the ground plane")


@dataclass(frozen=True)
class GroundStartGeometry:
    body_position_world: np.ndarray
    body_yaw_rad: float
    tether_origin_world: np.ndarray
    joint_anchor_world: np.ndarray
    tether_quaternion_xyzw: np.ndarray
    magnet_center_world: np.ndarray
    magnet_quaternion_xyzw: np.ndarray
    plate_position_world: np.ndarray


def compute_ground_start_geometry(
    config: GroundStartConfig,
    *,
    assigned_plate_id: int,
    ring_geometry: RingNetGeometry | None = None,
) -> GroundStartGeometry:
    """Return a mechanically sane, near-horizontal grounded M2B configuration.

    The magnet's spherical underside is placed on the assigned plate plane. The
    tether_rod visual/physical link is then aimed from its existing body-relative
    origin to the magnet centre with exactly the model's 0.50 m length. The X3
    is translated radially outward as needed. Its yaw is never changed to face
    the ring.
    """

    geometry = RingNetGeometry() if ring_geometry is None else ring_geometry
    ring_position = np.array(
        [config.ring_x_m, config.ring_y_m, config.ring_z_m], dtype=float
    )
    ring_rotation = _yaw_rotation(config.ring_yaw_rad)
    plate_position = geometry.plate_position_world(
        ring_position=ring_position,
        rotation_world_from_ring=ring_rotation,
        plate_index=int(assigned_plate_id),
    )

    radial = plate_position[:2] - ring_position[:2]
    radial_norm = float(np.linalg.norm(radial))
    if radial_norm <= 1e-12:
        raise ValueError("Assigned plate does not define an outward radial direction")
    radial /= radial_norm

    magnet_center = plate_position.copy()
    magnet_center[2] += config.magnet_radius_m

    # Preserve the actual current SDF ball-joint anchor geometry while rotating
    # the tether. This avoids introducing a large constraint impulse on the first
    # physics step. In the zero pose the joint sits at +0.010 - 0.050 = -0.040 m
    # relative to X3/base_link.
    joint_anchor_body_z = (
        config.tether_origin_initial_body_z_m + config.base_joint_offset_tether_z_m
    )
    joint_to_magnet_length = (
        config.tether_length_m - abs(config.base_joint_offset_tether_z_m)
    )
    if joint_to_magnet_length <= 0.0:
        raise ValueError("Current tether joint offset leaves no positive joint-to-magnet length")

    joint_anchor_z = config.body_z_m + joint_anchor_body_z
    vertical_delta = float(magnet_center[2] - joint_anchor_z)
    if abs(vertical_delta) >= joint_to_magnet_length:
        raise ValueError("Ground-start vertical separation exceeds joint-to-magnet reach")
    horizontal_reach = math.sqrt(joint_to_magnet_length**2 - vertical_delta**2)

    joint_anchor = magnet_center.copy()
    joint_anchor[:2] += horizontal_reach * radial
    joint_anchor[2] = joint_anchor_z

    body_position = joint_anchor.copy()
    body_position[2] = config.body_z_m

    cable_direction = magnet_center - joint_anchor
    cable_direction /= np.linalg.norm(cable_direction)

    # Joint pose is -0.050 m along tether local z. Since local -z is aligned
    # with cable_direction, the joint is +0.050 m along cable_direction from the
    # tether_rod origin. Rotate the child about that fixed joint anchor.
    joint_offset_along_cable = abs(config.base_joint_offset_tether_z_m)
    tether_origin = joint_anchor - joint_offset_along_cable * cable_direction

    # The cylinder in modelLargeM2BallMagnet.sdf extends along local -z.
    tether_quaternion = quaternion_xyzw_from_two_vectors(
        np.array([0.0, 0.0, -1.0], dtype=float), cable_direction
    )

    # Keep the magnet contact normal vertical downward while the ball joint is
    # free to accommodate the nearly horizontal tether. Preserve body yaw only.
    magnet_quaternion = yaw_quaternion_xyzw(config.initial_body_yaw_rad)

    return GroundStartGeometry(
        body_position_world=body_position,
        body_yaw_rad=float(config.initial_body_yaw_rad),
        tether_origin_world=tether_origin,
        joint_anchor_world=joint_anchor,
        tether_quaternion_xyzw=tether_quaternion,
        magnet_center_world=magnet_center,
        magnet_quaternion_xyzw=magnet_quaternion,
        plate_position_world=plate_position,
    )
