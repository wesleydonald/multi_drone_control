"""Reusable trajectory-reference primitives for per-drone missions.

All functions are ROS-independent and operate on NumPy arrays.  This makes the
reference layer testable without Gazebo or ROS and allows a future coordinator
to reuse the same primitives for multiple vehicles.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Hashable, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class TargetState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vec3(self.position, "position"))
        object.__setattr__(self, "velocity", _vec3(self.velocity, "velocity"))
        object.__setattr__(self, "acceleration", _vec3(self.acceleration, "acceleration"))

    def predict(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = max(0.0, float(t))
        position = self.position + self.velocity * t + 0.5 * self.acceleration * t * t
        velocity = self.velocity + self.acceleration * t
        return position, velocity, self.acceleration.copy()


@dataclass(frozen=True)
class TrajectoryReference:
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray

    def __post_init__(self) -> None:
        positions = _trajectory_array(self.positions, "positions")
        velocities = _trajectory_array(self.velocities, "velocities")
        accelerations = _trajectory_array(self.accelerations, "accelerations")
        if positions.shape != velocities.shape or positions.shape != accelerations.shape:
            raise ValueError(
                "positions, velocities and accelerations must have identical shapes; "
                f"got {positions.shape}, {velocities.shape}, {accelerations.shape}."
            )
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "accelerations", accelerations)


@dataclass(frozen=True)
class TransferConfig:
    dt: float
    horizon_samples: int
    nominal_speed: float
    duration_scale: float
    min_duration: float
    max_duration: float
    tracking_error_soft: float
    tracking_error_hard: float
    minimum_progress_scale: float
    min_reference_z: float
    max_reference_speed: float

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.horizon_samples < 2:
            raise ValueError("horizon_samples must be at least 2")
        if self.nominal_speed <= 0.0:
            raise ValueError("nominal_speed must be positive")
        if self.min_duration <= 0.0 or self.max_duration < self.min_duration:
            raise ValueError("invalid transfer-duration bounds")
        if self.tracking_error_hard <= self.tracking_error_soft:
            raise ValueError("tracking_error_hard must exceed tracking_error_soft")


def _vec3(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,):
        raise ValueError(f"{name} must contain exactly three elements; got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf: {array}")
    return array.copy()


def _trajectory_array(value: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3); got {array.shape}")
    if array.shape[0] < 1:
        raise ValueError(f"{name} must contain at least one sample")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array.copy()


def clamp_velocity(velocity: np.ndarray, maximum_speed: float) -> np.ndarray:
    velocity = _vec3(velocity, "velocity")
    maximum_speed = max(0.0, float(maximum_speed))
    speed = float(np.linalg.norm(velocity))
    if maximum_speed > 0.0 and speed > maximum_speed and speed > 1e-12:
        velocity *= maximum_speed / speed
    return velocity


def _enforce_min_z(
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    min_reference_z: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = position.copy()
    velocity = velocity.copy()
    acceleration = acceleration.copy()
    if position[2] < min_reference_z:
        position[2] = min_reference_z
        if velocity[2] < 0.0:
            velocity[2] = 0.0
        if acceleration[2] < 0.0:
            acceleration[2] = 0.0
    return position, velocity, acceleration


def cubic_hermite(
    t: float,
    duration: float,
    start_position: np.ndarray,
    start_velocity: np.ndarray,
    end_position: np.ndarray,
    end_velocity: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate a cubic Hermite segment and its first two derivatives."""

    duration = max(float(duration), 1e-9)
    t = float(np.clip(t, 0.0, duration))
    s = t / duration
    p0 = _vec3(start_position, "start_position")
    v0 = _vec3(start_velocity, "start_velocity")
    p1 = _vec3(end_position, "end_position")
    v1 = _vec3(end_velocity, "end_velocity")

    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    h11 = s**3 - s**2
    position = h00 * p0 + h10 * duration * v0 + h01 * p1 + h11 * duration * v1

    dh00 = (6.0 * s**2 - 6.0 * s) / duration
    dh10 = 3.0 * s**2 - 4.0 * s + 1.0
    dh01 = (-6.0 * s**2 + 6.0 * s) / duration
    dh11 = 3.0 * s**2 - 2.0 * s
    velocity = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1

    ddh00 = (12.0 * s - 6.0) / (duration * duration)
    ddh10 = (6.0 * s - 4.0) / duration
    ddh01 = (6.0 - 12.0 * s) / (duration * duration)
    ddh11 = (6.0 * s - 2.0) / duration
    acceleration = ddh00 * p0 + ddh10 * v0 + ddh01 * p1 + ddh11 * v1
    return position, velocity, acceleration


class VirtualTransferGenerator:
    """Persistent virtual-time transfer reference.

    The virtual reference origin advances independently of the measured vehicle
    state.  This avoids the lazy-follower failure caused by rebuilding every
    horizon from the measured pose at local time zero.  Progress slows when the
    vehicle falls far behind, but it never silently resets within a phase.
    """

    def __init__(self, config: TransferConfig) -> None:
        self.config = config
        self.phase_key: Optional[Hashable] = None
        self.reference_position: Optional[np.ndarray] = None
        self.reference_velocity: Optional[np.ndarray] = None
        self.last_duration = float("nan")
        self.last_progress_scale = 1.0

    def reset(self) -> None:
        self.phase_key = None
        self.reference_position = None
        self.reference_velocity = None
        self.last_duration = float("nan")
        self.last_progress_scale = 1.0

    def _progress_scale(self, measured_position: np.ndarray) -> float:
        if self.reference_position is None:
            return 1.0
        error = float(np.linalg.norm(_vec3(measured_position, "measured_position") - self.reference_position))
        soft = self.config.tracking_error_soft
        hard = self.config.tracking_error_hard
        if error <= soft:
            return 1.0
        if error >= hard:
            return float(np.clip(self.config.minimum_progress_scale, 0.0, 1.0))
        fraction = (error - soft) / max(1e-9, hard - soft)
        minimum = float(np.clip(self.config.minimum_progress_scale, 0.0, 1.0))
        return 1.0 - fraction * (1.0 - minimum)

    def _duration(
        self,
        start_position: np.ndarray,
        target: TargetState,
    ) -> float:
        duration = self.config.min_duration
        for _ in range(3):
            target_end, _, _ = target.predict(duration)
            distance = float(np.linalg.norm(target_end - start_position))
            duration = float(
                np.clip(
                    self.config.duration_scale * distance / self.config.nominal_speed,
                    self.config.min_duration,
                    self.config.max_duration,
                )
            )
        return duration

    def generate(
        self,
        *,
        phase_key: Hashable,
        measured_position: np.ndarray,
        measured_velocity: np.ndarray,
        target: TargetState,
        advance: bool = True,
    ) -> TrajectoryReference:
        measured_position = _vec3(measured_position, "measured_position")
        measured_velocity = clamp_velocity(
            measured_velocity,
            self.config.max_reference_speed,
        )

        missing = (
            self.phase_key != phase_key
            or self.reference_position is None
            or self.reference_velocity is None
        )
        if missing:
            start_position = measured_position.copy()
            start_velocity = measured_velocity.copy()
        else:
            start_position = self.reference_position.copy()
            start_velocity = self.reference_velocity.copy()

        duration = self._duration(start_position, target)
        target_end_position, target_end_velocity, _ = target.predict(duration)
        target_end_velocity = clamp_velocity(
            target_end_velocity,
            self.config.max_reference_speed,
        )

        positions = []
        velocities = []
        accelerations = []
        for sample in range(self.config.horizon_samples):
            t = sample * self.config.dt
            if t <= duration:
                position, velocity, acceleration = cubic_hermite(
                    t,
                    duration,
                    start_position,
                    start_velocity,
                    target_end_position,
                    target_end_velocity,
                )
            else:
                extra = t - duration
                position, velocity, acceleration = target.predict(duration + extra)
            position, velocity, acceleration = _enforce_min_z(
                position,
                clamp_velocity(velocity, self.config.max_reference_speed),
                acceleration,
                self.config.min_reference_z,
            )
            positions.append(position)
            velocities.append(velocity)
            accelerations.append(acceleration)

        if advance:
            progress_scale = self._progress_scale(measured_position)
            advance_time = min(self.config.dt * progress_scale, duration)
            next_position, next_velocity, _ = cubic_hermite(
                advance_time,
                duration,
                start_position,
                start_velocity,
                target_end_position,
                target_end_velocity,
            )
            next_position, next_velocity, _ = _enforce_min_z(
                next_position,
                clamp_velocity(next_velocity, self.config.max_reference_speed),
                np.zeros(3, dtype=float),
                self.config.min_reference_z,
            )
            self.phase_key = phase_key
            self.reference_position = next_position
            self.reference_velocity = next_velocity
            self.last_duration = duration
            self.last_progress_scale = progress_scale

        return TrajectoryReference(
            np.asarray(positions),
            np.asarray(velocities),
            np.asarray(accelerations),
        )


def stationary_regulation(
    target_position: np.ndarray,
    *,
    horizon_samples: int,
    min_reference_z: float,
) -> TrajectoryReference:
    target = _vec3(target_position, "target_position")
    target[2] = max(target[2], float(min_reference_z))
    positions = np.repeat(target.reshape(1, 3), int(horizon_samples), axis=0)
    zeros = np.zeros_like(positions)
    return TrajectoryReference(positions, zeros, zeros)


def target_relative_tracking(
    target: TargetState,
    relative_offset: np.ndarray,
    *,
    dt: float,
    horizon_samples: int,
    lead_time: float,
    min_reference_z: float,
    max_reference_speed: float,
) -> TrajectoryReference:
    offset = _vec3(relative_offset, "relative_offset")
    positions = []
    velocities = []
    accelerations = []
    for sample in range(int(horizon_samples)):
        t = max(0.0, float(lead_time)) + sample * float(dt)
        target_position, target_velocity, target_acceleration = target.predict(t)
        position = target_position + offset
        position, velocity, acceleration = _enforce_min_z(
            position,
            clamp_velocity(target_velocity, max_reference_speed),
            target_acceleration,
            min_reference_z,
        )
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)
    return TrajectoryReference(
        np.asarray(positions),
        np.asarray(velocities),
        np.asarray(accelerations),
    )


def smooth_rate_controlled_manoeuvre(
    target: TargetState,
    start_offset: np.ndarray,
    goal_offset: np.ndarray,
    *,
    rate: float,
    acceleration_limit: float,
    elapsed: float,
    dt: float,
    horizon_samples: int,
    min_reference_z: float,
    max_reference_speed: float,
) -> Tuple[TrajectoryReference, float, bool]:
    """Generate a C2 start/stop rate manoeuvre with bounded acceleration.

    The historical ``rate_controlled_manoeuvre`` jumps immediately from zero to
    the requested relative velocity.  That is appropriate to retain for already
    commissioned low-speed attachment phases, but it is a poor takeoff reference:
    p/v/a do not describe one physically coherent onset.

    This variant uses half-cosine velocity ramps.  Relative velocity and
    acceleration are both continuous, peak speed never exceeds ``rate``, and
    peak acceleration never exceeds ``acceleration_limit``.  A cruise segment is
    inserted when the move is long enough; short moves automatically become a
    smooth triangular profile with a lower peak speed.
    """

    start = _vec3(start_offset, "start_offset")
    goal = _vec3(goal_offset, "goal_offset")
    delta = goal - start
    distance = float(np.linalg.norm(delta))
    max_rate = max(0.0, float(rate))
    max_acceleration = max(0.0, float(acceleration_limit))

    if distance <= 1e-12:
        direction = np.zeros(3, dtype=float)
        peak_speed = 0.0
        ramp_time = 0.0
        cruise_time = 0.0
        duration = 0.0
    elif max_rate <= 1e-12 or max_acceleration <= 1e-12:
        direction = delta / distance
        peak_speed = 0.0
        ramp_time = 0.0
        cruise_time = 0.0
        duration = float("inf")
    else:
        direction = delta / distance
        # Half-cosine ramp:
        #   v = V/2 * (1 - cos(pi*t/T_r))
        # has peak acceleration V*pi/(2*T_r).  Choose T_r to exactly respect
        # the requested acceleration limit at the nominal maximum speed.
        nominal_ramp_time = np.pi * max_rate / (2.0 * max_acceleration)
        nominal_ramp_distance_total = max_rate * nominal_ramp_time
        if distance <= nominal_ramp_distance_total:
            # Two ramps with no cruise: d = V_peak*T_r and
            # T_r = pi*V_peak/(2*a_max).
            peak_speed = float(
                np.sqrt(2.0 * max_acceleration * distance / np.pi)
            )
            ramp_time = np.pi * peak_speed / (2.0 * max_acceleration)
            cruise_time = 0.0
        else:
            peak_speed = max_rate
            ramp_time = nominal_ramp_time
            cruise_time = distance / peak_speed - ramp_time
        duration = 2.0 * ramp_time + cruise_time

    def relative_state(t: float) -> Tuple[float, float, float]:
        if not np.isfinite(duration):
            return 0.0, 0.0, 0.0
        if duration <= 1e-12 or t >= duration:
            return distance, 0.0, 0.0
        t = max(0.0, float(t))

        if t < ramp_time:
            phase = np.pi * t / ramp_time
            speed = 0.5 * peak_speed * (1.0 - np.cos(phase))
            acceleration = (
                0.5 * peak_speed * np.pi / ramp_time * np.sin(phase)
            )
            progress = 0.5 * peak_speed * (
                t - ramp_time / np.pi * np.sin(phase)
            )
            return float(progress), float(speed), float(acceleration)

        ramp_distance = 0.5 * peak_speed * ramp_time
        if t < ramp_time + cruise_time:
            cruise_elapsed = t - ramp_time
            progress = ramp_distance + peak_speed * cruise_elapsed
            return float(progress), float(peak_speed), 0.0

        decel_elapsed = t - ramp_time - cruise_time
        phase = np.pi * decel_elapsed / ramp_time
        speed = 0.5 * peak_speed * (1.0 + np.cos(phase))
        acceleration = (
            -0.5 * peak_speed * np.pi / ramp_time * np.sin(phase)
        )
        progress = (
            ramp_distance
            + peak_speed * cruise_time
            + 0.5 * peak_speed * (
                decel_elapsed + ramp_time / np.pi * np.sin(phase)
            )
        )
        return float(progress), float(speed), float(acceleration)

    elapsed = max(0.0, float(elapsed))
    positions = []
    velocities = []
    accelerations = []
    for sample in range(int(horizon_samples)):
        relative_time = elapsed + sample * float(dt)
        progress, relative_speed, relative_acceleration = relative_state(relative_time)
        offset = start + direction * progress
        relative_velocity = direction * relative_speed
        relative_acceleration_vector = direction * relative_acceleration

        target_position, target_velocity, target_acceleration = target.predict(
            sample * float(dt)
        )
        position = target_position + offset
        velocity = target_velocity + relative_velocity
        acceleration = target_acceleration + relative_acceleration_vector
        position, velocity, acceleration = _enforce_min_z(
            position,
            clamp_velocity(velocity, max_reference_speed),
            acceleration,
            min_reference_z,
        )
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)

    complete_now = bool(np.isfinite(duration) and elapsed >= duration - 1e-9)
    return (
        TrajectoryReference(
            np.asarray(positions),
            np.asarray(velocities),
            np.asarray(accelerations),
        ),
        duration,
        complete_now,
    )


def rate_controlled_manoeuvre(
    target: TargetState,
    start_offset: np.ndarray,
    goal_offset: np.ndarray,
    *,
    rate: float,
    elapsed: float,
    dt: float,
    horizon_samples: int,
    min_reference_z: float,
    max_reference_speed: float,
) -> Tuple[TrajectoryReference, float, bool]:
    """Generate a constant-relative-rate manoeuvre in a moving target frame.

    Returns ``(reference, duration, profile_complete_now)``.  The relative
    velocity is consistent with the changing offset, unlike a sequence of small
    zero-velocity step references.
    """

    start = _vec3(start_offset, "start_offset")
    goal = _vec3(goal_offset, "goal_offset")
    delta = goal - start
    distance = float(np.linalg.norm(delta))
    rate = max(0.0, float(rate))
    if distance <= 1e-12:
        direction = np.zeros(3, dtype=float)
        duration = 0.0
    elif rate <= 1e-12:
        direction = np.zeros(3, dtype=float)
        duration = float("inf")
    else:
        direction = delta / distance
        duration = distance / rate

    elapsed = max(0.0, float(elapsed))
    positions = []
    velocities = []
    accelerations = []
    for sample in range(int(horizon_samples)):
        relative_time = elapsed + sample * float(dt)
        if np.isfinite(duration):
            progress = min(relative_time, duration)
        else:
            progress = 0.0
        offset = start + direction * rate * progress
        moving = bool(np.isfinite(duration) and relative_time < duration - 1e-9)
        relative_velocity = direction * rate if moving else np.zeros(3, dtype=float)

        target_position, target_velocity, target_acceleration = target.predict(sample * float(dt))
        position = target_position + offset
        velocity = target_velocity + relative_velocity
        position, velocity, acceleration = _enforce_min_z(
            position,
            clamp_velocity(velocity, max_reference_speed),
            target_acceleration,
            min_reference_z,
        )
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)

    complete_now = bool(np.isfinite(duration) and elapsed >= duration - 1e-9)
    return (
        TrajectoryReference(
            np.asarray(positions),
            np.asarray(velocities),
            np.asarray(accelerations),
        ),
        duration,
        complete_now,
    )
