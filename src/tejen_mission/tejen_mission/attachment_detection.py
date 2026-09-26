"""Pure M2A attachment-evidence geometry and state logic.

This module deliberately has no ROS or Gazebo dependency.  It consumes timestamped
poses and produces deterministic plate-relative evidence so the exact same detector
can be exercised with synthetic traces, recorded simulation data, and later mocap
measurements.

Gazebo detachable-joint state is intentionally not represented in this API.  It is
validation ground truth, not detector input.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

import numpy as np

from .cooperative_trajectory import RingNetGeometry


def _vec3(value, *, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return array.copy()


def _rotation3(value, *, name: str) -> np.ndarray:
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(f"{name} must be a proper rotation matrix")
    return rotation.copy()


def _nonnegative_finite(value: float, *, name: str, strictly_positive: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    if strictly_positive and result <= 0.0:
        raise ValueError(f"{name} must be positive")
    if not strictly_positive and result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


@dataclass(frozen=True)
class MagnetContactCalibration:
    """Fixed transform from measured magnet rigid-body frame to contact frame.

    ``contact_position_magnet`` is the contact-frame origin expressed in the
    measured magnet frame.  ``rotation_magnet_from_contact`` is ``R_MC``.
    """

    contact_position_magnet: np.ndarray
    rotation_magnet_from_contact: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "contact_position_magnet",
            _vec3(self.contact_position_magnet, name="contact_position_magnet"),
        )
        object.__setattr__(
            self,
            "rotation_magnet_from_contact",
            _rotation3(
                self.rotation_magnet_from_contact,
                name="rotation_magnet_from_contact",
            ),
        )


@dataclass(frozen=True)
class AttachmentDetectorConfig:
    """Deterministic M2A evidence thresholds.

    Values are deliberately explicit rather than hidden constants so later ROS
    wrappers can log the exact configuration used by each trial.
    """

    candidate_xy_m: float
    candidate_normal_m: float
    candidate_speed_mps: float
    candidate_dwell_s: float
    proof_xy_m: float
    proof_normal_m: float
    proof_speed_mps: float
    proof_dwell_s: float
    proof_min_excitation_m: float
    proof_direction_plate: np.ndarray
    loss_xy_m: float
    loss_normal_m: float
    loss_dwell_s: float
    pose_timeout_s: float
    velocity_filter_tau_s: float
    magnet_off_counts_as_loss: bool = True

    def __post_init__(self) -> None:
        for name in (
            "candidate_xy_m",
            "candidate_normal_m",
            "candidate_speed_mps",
            "proof_xy_m",
            "proof_normal_m",
            "proof_speed_mps",
            "proof_min_excitation_m",
            "loss_xy_m",
            "loss_normal_m",
            "pose_timeout_s",
        ):
            object.__setattr__(
                self,
                name,
                _nonnegative_finite(getattr(self, name), name=name, strictly_positive=True),
            )
        for name in ("candidate_dwell_s", "proof_dwell_s", "loss_dwell_s", "velocity_filter_tau_s"):
            object.__setattr__(
                self,
                name,
                _nonnegative_finite(getattr(self, name), name=name),
            )

        direction = _vec3(self.proof_direction_plate, name="proof_direction_plate")
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            raise ValueError("proof_direction_plate must have non-zero norm")
        object.__setattr__(self, "proof_direction_plate", direction / norm)

        if self.loss_xy_m < self.proof_xy_m:
            raise ValueError("loss_xy_m must be >= proof_xy_m for hysteresis")
        if self.loss_normal_m < self.proof_normal_m:
            raise ValueError("loss_normal_m must be >= proof_normal_m for hysteresis")
        object.__setattr__(
            self, "magnet_off_counts_as_loss", bool(self.magnet_off_counts_as_loss)
        )


@dataclass(frozen=True)
class AttachmentObservation:
    """One timestamped observable sample used by the pure detector."""

    time_s: float
    ring_position_world: np.ndarray
    rotation_world_from_ring: np.ndarray
    ring_time_s: float
    magnet_position_world: np.ndarray
    rotation_world_from_magnet: np.ndarray
    magnet_time_s: float
    drone_position_world: np.ndarray
    drone_time_s: float
    magnet_on: bool
    proof_requested: bool

    def __post_init__(self) -> None:
        for name in ("time_s", "ring_time_s", "magnet_time_s", "drone_time_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        object.__setattr__(
            self,
            "ring_position_world",
            _vec3(self.ring_position_world, name="ring_position_world"),
        )
        object.__setattr__(
            self,
            "rotation_world_from_ring",
            _rotation3(self.rotation_world_from_ring, name="rotation_world_from_ring"),
        )
        object.__setattr__(
            self,
            "magnet_position_world",
            _vec3(self.magnet_position_world, name="magnet_position_world"),
        )
        object.__setattr__(
            self,
            "rotation_world_from_magnet",
            _rotation3(self.rotation_world_from_magnet, name="rotation_world_from_magnet"),
        )
        object.__setattr__(
            self,
            "drone_position_world",
            _vec3(self.drone_position_world, name="drone_position_world"),
        )
        object.__setattr__(self, "magnet_on", bool(self.magnet_on))
        object.__setattr__(self, "proof_requested", bool(self.proof_requested))


class MagnetContactObservationModel(str, Enum):
    """How measured magnet pose is converted into plate-relative contact evidence."""

    RIGID_CALIBRATED = "rigid_calibrated"
    SPHERE_CENTER = "sphere_center"


class AttachmentEvidenceState(str, Enum):
    UNAVAILABLE = "unavailable"
    SEPARATED = "separated"
    CONTACT_CANDIDATE = "contact_candidate"
    PROVING = "proving"
    CONFIRMED = "confirmed"
    LOST = "lost"


@dataclass(frozen=True)
class AttachmentDetectorOutput:
    state: AttachmentEvidenceState
    fresh: bool
    plate_relative_position_m: np.ndarray
    radial_error_m: float
    tangential_error_m: float
    normal_error_m: float
    xy_error_m: float
    plate_relative_velocity_mps: np.ndarray
    relative_speed_mps: float
    orientation_error_rad: float
    candidate_condition: bool
    candidate_dwell_s: float
    proof_condition: bool
    proof_dwell_s: float
    proof_excitation_m: float
    confirmed: bool
    lost_condition: bool
    geometry_separated: bool
    loss_dwell_s: float
    lost: bool

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "plate_relative_position_m",
            _vec3(self.plate_relative_position_m, name="plate_relative_position_m"),
        )
        object.__setattr__(
            self,
            "plate_relative_velocity_mps",
            _vec3(self.plate_relative_velocity_mps, name="plate_relative_velocity_mps"),
        )


class AttachmentDetector:
    """Stateful deterministic evidence detector for one assigned plate."""

    def __init__(
        self,
        *,
        geometry: RingNetGeometry,
        assigned_plate_id: int,
        calibration: MagnetContactCalibration,
        config: AttachmentDetectorConfig,
        contact_observation_model: MagnetContactObservationModel | str = MagnetContactObservationModel.RIGID_CALIBRATED,
        sphere_radius_m: float = 0.025,
    ) -> None:
        self.geometry = geometry
        self.assigned_plate_id = int(assigned_plate_id)
        # Validate through the public geometry contract rather than depending on a
        # private helper.  The returned value is intentionally discarded.
        self.geometry.plate_body_offset(self.assigned_plate_id)
        self.calibration = calibration
        self.config = config
        try:
            self.contact_observation_model = MagnetContactObservationModel(contact_observation_model)
        except ValueError as exc:
            raise ValueError(
                "contact_observation_model must be 'rigid_calibrated' or 'sphere_center'"
            ) from exc
        self.sphere_radius_m = _nonnegative_finite(
            sphere_radius_m, name="sphere_radius_m", strictly_positive=True
        )
        self.reset()

    def reset(self) -> None:
        self._previous_time_s: Optional[float] = None
        self._previous_plate_relative_position_m: Optional[np.ndarray] = None
        self._filtered_relative_velocity_mps = np.zeros(3, dtype=float)
        self._candidate_start_s: Optional[float] = None
        self._candidate_ready = False
        self._proof_start_drone_plate_m: Optional[np.ndarray] = None
        self._proof_condition_start_s: Optional[float] = None
        self._loss_start_s: Optional[float] = None
        self._confirmed = False
        self._lost = False

    @staticmethod
    def _dwell(now_s: float, start_s: Optional[float]) -> float:
        if start_s is None:
            return 0.0
        return max(0.0, float(now_s) - float(start_s))

    def _fresh(self, observation: AttachmentObservation) -> bool:
        now = observation.time_s
        for stamp in (
            observation.ring_time_s,
            observation.magnet_time_s,
            observation.drone_time_s,
        ):
            age = now - stamp
            # Future-dated source data is not accepted as evidence.  Tiny floating
            # point error is tolerated, but a real future timestamp is a clock issue.
            if age < -1e-9 or age > self.config.pose_timeout_s:
                return False
        return True

    def _geometry(self, observation: AttachmentObservation):
        p_wp = self.geometry.plate_position_world(
            ring_position=observation.ring_position_world,
            rotation_world_from_ring=observation.rotation_world_from_ring,
            plate_index=self.assigned_plate_id,
        )
        r_wp = self.geometry.plate_rotation_world(
            rotation_world_from_ring=observation.rotation_world_from_ring,
            plate_index=self.assigned_plate_id,
        )

        if self.contact_observation_model is MagnetContactObservationModel.SPHERE_CENTER:
            # The simulated black magnet is a sphere attached through a ball joint.
            # Its body orientation may spin freely and therefore carries no useful
            # information about where the sphere contacts the plate.  Express the
            # sphere centre in the plate frame and subtract the physical radius only
            # along +z_P.  Finite-difference velocity of this position is therefore
            # translational centre velocity, not fictitious surface-point motion.
            p_pc = r_wp.T @ (observation.magnet_position_world - p_wp)
            p_pc = p_pc.copy()
            p_pc[2] -= self.sphere_radius_m
            orientation_error = 0.0
        else:
            p_wc = observation.magnet_position_world + (
                observation.rotation_world_from_magnet
                @ self.calibration.contact_position_magnet
            )
            r_wc = (
                observation.rotation_world_from_magnet
                @ self.calibration.rotation_magnet_from_contact
            )
            p_pc = r_wp.T @ (p_wc - p_wp)
            r_pc = r_wp.T @ r_wc
            cosine = float(np.clip((np.trace(r_pc) - 1.0) * 0.5, -1.0, 1.0))
            orientation_error = math.acos(cosine)

        p_pq = r_wp.T @ (observation.drone_position_world - p_wp)
        return p_pc, p_pq, orientation_error

    def _relative_velocity(self, *, now_s: float, position_m: np.ndarray) -> np.ndarray:
        if self._previous_time_s is None or self._previous_plate_relative_position_m is None:
            velocity = np.zeros(3, dtype=float)
        else:
            dt = float(now_s) - float(self._previous_time_s)
            if dt <= 1e-9:
                # Duplicate/out-of-order replay rows do not provide a usable finite
                # difference and must not replace the last valid history sample.
                return self._filtered_relative_velocity_mps.copy()
            raw = (position_m - self._previous_plate_relative_position_m) / dt
            tau = self.config.velocity_filter_tau_s
            if tau <= 0.0:
                velocity = raw
            else:
                # Exact first-order low-pass discretisation.  This remains stable
                # for irregular replay sample intervals.
                alpha = 1.0 - math.exp(-dt / tau)
                velocity = (
                    (1.0 - alpha) * self._filtered_relative_velocity_mps
                    + alpha * raw
                )

        self._previous_time_s = float(now_s)
        self._previous_plate_relative_position_m = position_m.copy()
        self._filtered_relative_velocity_mps = velocity.copy()
        return velocity

    def update(self, observation: AttachmentObservation) -> AttachmentDetectorOutput:
        p_pc, p_pq, orientation_error = self._geometry(observation)
        fresh = self._fresh(observation)
        if fresh:
            velocity = self._relative_velocity(
                now_s=observation.time_s,
                position_m=p_pc,
            )
        else:
            # Stale/dropout rows are observable evidence failures, not new kinematic
            # samples.  Keeping them out of the finite-difference history is crucial
            # for deterministic offline replay and clean recovery on the next fresh row.
            velocity = self._filtered_relative_velocity_mps.copy()
        relative_speed = float(np.linalg.norm(velocity))
        xy_error = float(np.linalg.norm(p_pc[:2]))
        normal_error = float(p_pc[2])
        geometry_separated = bool(
            xy_error > self.config.loss_xy_m
            or abs(normal_error) > self.config.loss_normal_m
        )

        candidate_condition = bool(
            fresh
            and observation.magnet_on
            and xy_error <= self.config.candidate_xy_m
            and abs(normal_error) <= self.config.candidate_normal_m
            and relative_speed <= self.config.candidate_speed_mps
        )

        if candidate_condition and not self._confirmed and not self._lost:
            if self._candidate_start_s is None:
                self._candidate_start_s = observation.time_s
        elif not self._candidate_ready:
            self._candidate_start_s = None

        candidate_dwell = self._dwell(observation.time_s, self._candidate_start_s)
        if candidate_condition and candidate_dwell >= self.config.candidate_dwell_s:
            self._candidate_ready = True

        if not fresh:
            # Stale measurements cannot create, progress, or sustain attachment
            # evidence.  Before confirmation, all candidate/proof readiness is
            # invalidated and must be reacquired from fresh observations.  A
            # previously confirmed physical latch is retained internally so a brief
            # transport dropout does not itself become a separation event, but
            # externally the detector reports no current confirmation evidence.
            if not self._confirmed:
                self._candidate_start_s = None
                self._candidate_ready = False
                self._proof_start_drone_plate_m = None
            self._proof_condition_start_s = None
            self._loss_start_s = None
            return self._output(
                state=AttachmentEvidenceState.UNAVAILABLE,
                fresh=False,
                position=p_pc,
                velocity=velocity,
                orientation_error=orientation_error,
                candidate_condition=False,
                candidate_dwell=0.0,
                proof_condition=False,
                proof_dwell=0.0,
                proof_excitation=0.0,
                lost_condition=False,
                geometry_separated=geometry_separated,
                loss_dwell=0.0,
                confirmed_override=False,
            )

        if not self._confirmed and not self._lost and not observation.proof_requested:
            if not candidate_condition:
                self._candidate_start_s = None
                self._candidate_ready = False

        if not observation.magnet_on and not self._confirmed:
            self._candidate_start_s = None
            self._candidate_ready = False
            self._proof_start_drone_plate_m = None
            self._proof_condition_start_s = None

        proving = bool(
            observation.proof_requested
            and observation.magnet_on
            and self._candidate_ready
            and not self._confirmed
            and not self._lost
        )

        if proving and self._proof_start_drone_plate_m is None:
            self._proof_start_drone_plate_m = p_pq.copy()
            self._proof_condition_start_s = None
        elif not observation.proof_requested and not self._confirmed and not self._lost:
            self._proof_start_drone_plate_m = None
            self._proof_condition_start_s = None

        proof_excitation = 0.0
        if self._proof_start_drone_plate_m is not None:
            delta = p_pq - self._proof_start_drone_plate_m
            proof_excitation = max(
                0.0,
                float(np.dot(delta, self.config.proof_direction_plate)),
            )

        proof_condition = bool(
            proving
            and xy_error <= self.config.proof_xy_m
            and abs(normal_error) <= self.config.proof_normal_m
            and relative_speed <= self.config.proof_speed_mps
        )
        if proof_condition:
            if self._proof_condition_start_s is None:
                self._proof_condition_start_s = observation.time_s
        else:
            self._proof_condition_start_s = None
        proof_dwell = self._dwell(observation.time_s, self._proof_condition_start_s)

        if (
            proof_condition
            and proof_excitation >= self.config.proof_min_excitation_m
            and proof_dwell >= self.config.proof_dwell_s
        ):
            self._confirmed = True
            self._loss_start_s = None

        lost_condition = False
        loss_dwell = 0.0
        if self._confirmed:
            lost_condition = bool(
                geometry_separated
                or (
                    self.config.magnet_off_counts_as_loss
                    and not observation.magnet_on
                )
            )
            if lost_condition:
                if self._loss_start_s is None:
                    self._loss_start_s = observation.time_s
            else:
                self._loss_start_s = None
            loss_dwell = self._dwell(observation.time_s, self._loss_start_s)
            if lost_condition and loss_dwell >= self.config.loss_dwell_s:
                self._confirmed = False
                self._lost = True

        if self._lost:
            state = AttachmentEvidenceState.LOST
        elif self._confirmed:
            state = AttachmentEvidenceState.CONFIRMED
        elif proving:
            state = AttachmentEvidenceState.PROVING
        elif candidate_condition:
            state = AttachmentEvidenceState.CONTACT_CANDIDATE
        else:
            state = AttachmentEvidenceState.SEPARATED

        return self._output(
            state=state,
            fresh=True,
            position=p_pc,
            velocity=velocity,
            orientation_error=orientation_error,
            candidate_condition=candidate_condition,
            candidate_dwell=candidate_dwell if candidate_condition else 0.0,
            proof_condition=proof_condition,
            proof_dwell=proof_dwell if proof_condition else 0.0,
            proof_excitation=proof_excitation,
            lost_condition=lost_condition,
            geometry_separated=geometry_separated,
            loss_dwell=loss_dwell,
        )

    def _output(
        self,
        *,
        state: AttachmentEvidenceState,
        fresh: bool,
        position: np.ndarray,
        velocity: np.ndarray,
        orientation_error: float,
        candidate_condition: bool,
        candidate_dwell: float,
        proof_condition: bool,
        proof_dwell: float,
        proof_excitation: float,
        lost_condition: bool,
        geometry_separated: bool,
        loss_dwell: float,
        confirmed_override: Optional[bool] = None,
    ) -> AttachmentDetectorOutput:
        exposed_confirmed = self._confirmed if confirmed_override is None else bool(confirmed_override)
        return AttachmentDetectorOutput(
            state=state,
            fresh=bool(fresh),
            plate_relative_position_m=position,
            radial_error_m=float(position[0]),
            tangential_error_m=float(position[1]),
            normal_error_m=float(position[2]),
            xy_error_m=float(np.linalg.norm(position[:2])),
            plate_relative_velocity_mps=velocity,
            relative_speed_mps=float(np.linalg.norm(velocity)),
            orientation_error_rad=float(orientation_error),
            candidate_condition=bool(candidate_condition),
            candidate_dwell_s=float(candidate_dwell),
            proof_condition=bool(proof_condition),
            proof_dwell_s=float(proof_dwell),
            proof_excitation_m=float(proof_excitation),
            confirmed=bool(exposed_confirmed),
            lost_condition=bool(lost_condition),
            geometry_separated=bool(geometry_separated),
            loss_dwell_s=float(loss_dwell),
            lost=bool(self._lost),
        )
