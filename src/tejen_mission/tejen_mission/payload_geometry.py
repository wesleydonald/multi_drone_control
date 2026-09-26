"""Shared configurable geometry for carried payloads.

V5A uses this profile for passive collision diagnostics and RViz.  V5B1
derives a geometry-aware pickup target from the pickup-point and magnet-face
fields.  V5B2 can opt in to using the same target during the pickup phases.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _vec3(value, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array.copy()


def rotate_yaw(vector: np.ndarray, yaw: float) -> np.ndarray:
    """Rotate one three-dimensional vector about the world z axis."""

    vector = _vec3(vector, "vector")
    yaw = float(yaw)
    if not np.isfinite(yaw):
        raise ValueError("yaw must be finite")
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    return np.array(
        [
            c * vector[0] - s * vector[1],
            s * vector[0] + c * vector[1],
            vector[2],
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class PickupTargetGeometry:
    """Diagnostic geometry for one candidate payload pickup target.

    All positions and vectors are expressed in the world frame.  V5B1 exposes
    them for logging and RViz; V5B2 can use the same validated target when the
    planner's opt-in activation flag is enabled.
    """

    pickup_point_world: np.ndarray
    contact_face_target_world: np.ndarray
    magnet_marker_target_world: np.ndarray
    marker_to_contact_face_world: np.ndarray
    contact_gap: float
    requested_overtravel: float
    used_overtravel: float
    support_surface_z: float
    support_surface_clamped: bool

    def approach_marker_target_world(self, approach_height: float) -> np.ndarray:
        """Return a marker target directly above the contact target.

        The approach height is measured vertically from the geometry-derived
        contact marker target.  Reusing the old approach-to-contact height
        difference preserves the legacy descent distance when the two contact
        targets coincide.
        """

        approach_height = float(approach_height)
        if not np.isfinite(approach_height) or approach_height < 0.0:
            raise ValueError("approach_height must be finite and non-negative")
        return self.magnet_marker_target_world + np.array(
            [0.0, 0.0, approach_height],
            dtype=float,
        )

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "pickup_point_world",
            _vec3(self.pickup_point_world, "pickup_point_world"),
        )
        object.__setattr__(
            self,
            "contact_face_target_world",
            _vec3(self.contact_face_target_world, "contact_face_target_world"),
        )
        object.__setattr__(
            self,
            "magnet_marker_target_world",
            _vec3(self.magnet_marker_target_world, "magnet_marker_target_world"),
        )
        object.__setattr__(
            self,
            "marker_to_contact_face_world",
            _vec3(
                self.marker_to_contact_face_world,
                "marker_to_contact_face_world",
            ),
        )
        for name in (
            "contact_gap",
            "requested_overtravel",
            "used_overtravel",
            "support_surface_z",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        object.__setattr__(
            self,
            "support_surface_clamped",
            bool(self.support_surface_clamped),
        )


@dataclass(frozen=True)
class PayloadGeometryProfile:
    """Geometry and contact convention for one selectable carried payload.

    ``geometry_origin_from_pose`` points from the reported payload pose origin to
    the centre of the collision box, expressed in payload coordinates.
    ``pickup_point`` is also expressed in payload coordinates.  V5B1 uses the
    contact fields diagnostically; V5B2 may use them for an explicitly enabled
    pickup reference while retaining the same profile and geometry convention.
    """

    name: str
    dimensions: np.ndarray
    geometry_origin_from_pose: np.ndarray
    pickup_point: np.ndarray
    magnet_marker_to_contact_face: np.ndarray
    pose_reference: str = "centre"
    assume_level: bool = True
    collision_enabled: bool = True
    collision_margin: float = 0.05

    def __post_init__(self) -> None:
        name = str(self.name).strip()
        if not name:
            raise ValueError("name must not be empty")

        dimensions = _vec3(self.dimensions, "dimensions")
        if np.any(dimensions <= 0.0):
            raise ValueError("all payload dimensions must be positive")

        geometry_origin = _vec3(
            self.geometry_origin_from_pose,
            "geometry_origin_from_pose",
        )
        pickup_point = _vec3(self.pickup_point, "pickup_point")
        marker_to_face = _vec3(
            self.magnet_marker_to_contact_face,
            "magnet_marker_to_contact_face",
        )

        pose_reference = str(self.pose_reference).strip().lower()
        if not pose_reference:
            raise ValueError("pose_reference must not be empty")

        collision_margin = float(self.collision_margin)
        if not np.isfinite(collision_margin) or collision_margin < 0.0:
            raise ValueError("collision_margin must be finite and non-negative")

        object.__setattr__(self, "name", name)
        object.__setattr__(self, "dimensions", dimensions)
        object.__setattr__(
            self,
            "geometry_origin_from_pose",
            geometry_origin,
        )
        object.__setattr__(self, "pickup_point", pickup_point)
        object.__setattr__(
            self,
            "magnet_marker_to_contact_face",
            marker_to_face,
        )
        object.__setattr__(self, "pose_reference", pose_reference)
        object.__setattr__(self, "assume_level", bool(self.assume_level))
        object.__setattr__(self, "collision_enabled", bool(self.collision_enabled))
        object.__setattr__(self, "collision_margin", collision_margin)

    @property
    def half_extents(self) -> np.ndarray:
        return 0.5 * self.dimensions

    @property
    def inflated_dimensions(self) -> np.ndarray:
        return self.dimensions + 2.0 * self.collision_margin

    def geometry_centre_from_pose(
        self,
        pose_position: np.ndarray,
        yaw: float,
    ) -> np.ndarray:
        """Convert a reported pose origin into the collision-box centre."""

        return _vec3(pose_position, "pose_position") + rotate_yaw(
            self.geometry_origin_from_pose,
            yaw,
        )

    def pickup_point_world(
        self,
        pose_position: np.ndarray,
        yaw: float,
    ) -> np.ndarray:
        """Return the configured pickup point in world coordinates."""

        return _vec3(pose_position, "pose_position") + rotate_yaw(
            self.pickup_point,
            yaw,
        )

    def pickup_target_geometry(
        self,
        pose_position: np.ndarray,
        yaw: float,
        contact_gap: float,
        overtravel: float,
        max_overtravel: float,
        support_surface_z: float,
    ) -> PickupTargetGeometry:
        """Compute the shared geometry-aware pickup target.

        ``pickup_point`` is rotated by the payload yaw because it is expressed in
        payload coordinates.  The marker-to-contact-face vector is deliberately
        treated as a world-frame vector because the tracked magnet marker has no
        reliable full attitude measurement yet.
        """

        contact_gap = float(contact_gap)
        overtravel = float(overtravel)
        max_overtravel = float(max_overtravel)
        support_surface_z = float(support_surface_z)
        for name, value in (
            ("contact_gap", contact_gap),
            ("overtravel", overtravel),
            ("max_overtravel", max_overtravel),
            ("support_surface_z", support_surface_z),
        ):
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if contact_gap < 0.0:
            raise ValueError("contact_gap must be non-negative")
        if max_overtravel < 0.0:
            raise ValueError("max_overtravel must be non-negative")

        requested_overtravel = max(0.0, overtravel)
        used_overtravel = float(
            np.clip(requested_overtravel, 0.0, max_overtravel)
        )
        pickup_point_world = self.pickup_point_world(pose_position, yaw)

        contact_face_target_world = pickup_point_world + np.array(
            [0.0, 0.0, contact_gap - used_overtravel],
            dtype=float,
        )
        support_surface_clamped = bool(
            contact_face_target_world[2] < support_surface_z
        )
        if support_surface_clamped:
            contact_face_target_world[2] = support_surface_z

        marker_to_contact_face_world = (
            self.magnet_marker_to_contact_face.copy()
        )
        magnet_marker_target_world = (
            contact_face_target_world - marker_to_contact_face_world
        )

        return PickupTargetGeometry(
            pickup_point_world=pickup_point_world,
            contact_face_target_world=contact_face_target_world,
            magnet_marker_target_world=magnet_marker_target_world,
            marker_to_contact_face_world=marker_to_contact_face_world,
            contact_gap=contact_gap,
            requested_overtravel=requested_overtravel,
            used_overtravel=used_overtravel,
            support_surface_z=support_surface_z,
            support_surface_clamped=support_surface_clamped,
        )
