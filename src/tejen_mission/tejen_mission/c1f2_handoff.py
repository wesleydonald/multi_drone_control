"""Pure C1F.2/C1F.2a/C1F.2b reference-authority helpers.

Kept ROS-independent so source-selection and dropout-continuation contracts can
be regression tested without a running graph. Python remains the only ROS
authority publisher.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .reference_generators import TargetState, TrajectoryReference


@dataclass(frozen=True)
class HandoffErrors:
    position_m: float
    velocity_mps: float
    acceleration_mps2: float




@dataclass(frozen=True)
class TargetRelativeBridgeFeasibility:
    """Dynamic capturability result for the exact C2 bridge model."""

    feasible: bool
    reason: str
    max_abs_velocity: np.ndarray
    max_abs_acceleration: np.ndarray
    max_abs_jerk: np.ndarray
    final_position_error_m: float
    final_velocity_error_mps: float
    final_acceleration_error_mps2: float

    def __post_init__(self) -> None:
        for name in ("max_abs_velocity", "max_abs_acceleration", "max_abs_jerk"):
            value = np.asarray(getattr(self, name), dtype=float).reshape(3)
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value.copy())

@dataclass(frozen=True)
class TargetRelativeBridgeState:
    """Initial relative error state for a C++ -> Python moving-frame bridge."""

    position_error: np.ndarray
    velocity_error: np.ndarray
    acceleration_error: np.ndarray

    def __post_init__(self) -> None:
        for name in ("position_error", "velocity_error", "acceleration_error"):
            value = np.asarray(getattr(self, name), dtype=float).reshape(3)
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value.copy())


def target_relative_bridge_initial_state(
    *,
    outgoing_position: np.ndarray,
    outgoing_velocity: np.ndarray,
    outgoing_acceleration: np.ndarray,
    target: TargetState,
    relative_offset: np.ndarray,
) -> TargetRelativeBridgeState:
    """Express the outgoing absolute p/v/a state in the target-relative frame."""
    outgoing_position = np.asarray(outgoing_position, dtype=float).reshape(3)
    outgoing_velocity = np.asarray(outgoing_velocity, dtype=float).reshape(3)
    outgoing_acceleration = np.asarray(outgoing_acceleration, dtype=float).reshape(3)
    relative_offset = np.asarray(relative_offset, dtype=float).reshape(3)
    for name, value in (
        ("outgoing_position", outgoing_position),
        ("outgoing_velocity", outgoing_velocity),
        ("outgoing_acceleration", outgoing_acceleration),
        ("relative_offset", relative_offset),
    ):
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be finite")
    return TargetRelativeBridgeState(
        position_error=outgoing_position - target.position - relative_offset,
        velocity_error=outgoing_velocity - target.velocity,
        acceleration_error=outgoing_acceleration - target.acceleration,
    )


def _quintic_error_sample_with_jerk(
    state: TargetRelativeBridgeState,
    *,
    time_s: float,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate the C2 quintic error trajectory and its jerk."""
    duration_s = float(duration_s)
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    time_s = max(0.0, float(time_s))
    if not np.isfinite(time_s):
        raise ValueError("time_s must be finite")
    if time_s >= duration_s:
        z = np.zeros(3, dtype=float)
        return z.copy(), z.copy(), z.copy(), z.copy()

    s = time_s / duration_s
    b0 = state.position_error
    b1 = duration_s * state.velocity_error
    b2 = 0.5 * duration_s * duration_s * state.acceleration_error
    b3 = -10.0 * b0 - 6.0 * b1 - 3.0 * b2
    b4 = 15.0 * b0 + 8.0 * b1 + 3.0 * b2
    b5 = -6.0 * b0 - 3.0 * b1 - b2

    error = b0 + b1 * s + b2 * s**2 + b3 * s**3 + b4 * s**4 + b5 * s**5
    error_velocity = (
        b1 + 2.0 * b2 * s + 3.0 * b3 * s**2 + 4.0 * b4 * s**3 + 5.0 * b5 * s**4
    ) / duration_s
    error_acceleration = (
        2.0 * b2 + 6.0 * b3 * s + 12.0 * b4 * s**2 + 20.0 * b5 * s**3
    ) / (duration_s * duration_s)
    error_jerk = (
        6.0 * b3 + 24.0 * b4 * s + 60.0 * b5 * s**2
    ) / (duration_s**3)
    return error, error_velocity, error_acceleration, error_jerk


def _quintic_error_sample(
    state: TargetRelativeBridgeState,
    *,
    time_s: float,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    error, velocity, acceleration, _ = _quintic_error_sample_with_jerk(
        state, time_s=time_s, duration_s=duration_s
    )
    return error, velocity, acceleration


def target_relative_bridge_feasibility(
    *,
    bridge_state: TargetRelativeBridgeState,
    target: TargetState,
    duration_s: float,
    max_abs_velocity: np.ndarray,
    max_abs_acceleration: np.ndarray,
    max_abs_jerk: np.ndarray,
    sample_dt_s: float = 0.02,
    tolerance: float = 1e-9,
) -> TargetRelativeBridgeFeasibility:
    """Check whether the exact target-relative bridge is dynamically capturable.

    This deliberately replaces a tight relative-velocity transition threshold.
    Velocity mismatch is allowed whenever the same quintic bridge that will be
    executed can absorb the full relative p/v/a state within configured absolute
    velocity, acceleration, and jerk limits.  TargetState has constant
    acceleration, hence the bridge absolute jerk equals the relative-error jerk.
    """
    duration_s = float(duration_s)
    sample_dt_s = float(sample_dt_s)
    tolerance = max(0.0, float(tolerance))
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    if not np.isfinite(sample_dt_s) or sample_dt_s <= 0.0:
        raise ValueError("sample_dt_s must be finite and positive")

    limits = []
    for name, value in (
        ("max_abs_velocity", max_abs_velocity),
        ("max_abs_acceleration", max_abs_acceleration),
        ("max_abs_jerk", max_abs_jerk),
    ):
        arr = np.asarray(value, dtype=float).reshape(3)
        if not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
            raise ValueError(f"{name} must be finite and strictly positive")
        limits.append(arr)
    velocity_limit, acceleration_limit, jerk_limit = limits

    count = max(2, int(np.ceil(duration_s / sample_dt_s)) + 1)
    times = np.linspace(0.0, duration_s, count)
    peak_v = np.zeros(3, dtype=float)
    peak_a = np.zeros(3, dtype=float)
    peak_j = np.zeros(3, dtype=float)
    for t in times:
        _, error_v, error_a, error_j = _quintic_error_sample_with_jerk(
            bridge_state, time_s=float(t), duration_s=duration_s
        )
        _, target_v, target_a = target.predict(float(t))
        peak_v = np.maximum(peak_v, np.abs(target_v + error_v))
        peak_a = np.maximum(peak_a, np.abs(target_a + error_a))
        peak_j = np.maximum(peak_j, np.abs(error_j))

    final_e, final_ev, final_ea, _ = _quintic_error_sample_with_jerk(
        bridge_state, time_s=duration_s, duration_s=duration_s
    )
    feasible = True
    reason = "OK"
    if np.any(peak_v > velocity_limit + tolerance):
        feasible = False
        reason = "VELOCITY_LIMIT"
    elif np.any(peak_a > acceleration_limit + tolerance):
        feasible = False
        reason = "ACCELERATION_LIMIT"
    elif np.any(peak_j > jerk_limit + tolerance):
        feasible = False
        reason = "JERK_LIMIT"

    return TargetRelativeBridgeFeasibility(
        feasible=feasible,
        reason=reason,
        max_abs_velocity=peak_v,
        max_abs_acceleration=peak_a,
        max_abs_jerk=peak_j,
        final_position_error_m=float(np.linalg.norm(final_e)),
        final_velocity_error_mps=float(np.linalg.norm(final_ev)),
        final_acceleration_error_mps2=float(np.linalg.norm(final_ea)),
    )


def target_relative_bridge_reference(
    *,
    bridge_state: TargetRelativeBridgeState,
    target: TargetState,
    relative_offset: np.ndarray,
    elapsed_s: float,
    duration_s: float,
    dt: float,
    horizon_samples: int,
) -> tuple[TrajectoryReference, bool]:
    """Generate a live-target C2 bridge from an outgoing absolute p/v/a state.

    The target motion remains live during the bridge. The latched quintic is only
    the *relative error* that must decay to zero. At elapsed=0, stage 0 exactly
    reproduces the outgoing state used to create ``bridge_state``. At and after
    ``duration_s`` the reference is ordinary target-relative tracking.
    """
    relative_offset = np.asarray(relative_offset, dtype=float).reshape(3)
    if not np.all(np.isfinite(relative_offset)):
        raise ValueError("relative_offset must be finite")
    elapsed_s = max(0.0, float(elapsed_s))
    dt = float(dt)
    horizon_samples = int(horizon_samples)
    if not np.isfinite(elapsed_s):
        raise ValueError("elapsed_s must be finite")
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if horizon_samples <= 0:
        raise ValueError("horizon_samples must be positive")

    positions = np.empty((horizon_samples, 3), dtype=float)
    velocities = np.empty((horizon_samples, 3), dtype=float)
    accelerations = np.empty((horizon_samples, 3), dtype=float)
    for i in range(horizon_samples):
        sample_time = elapsed_s + i * dt
        target_position, target_velocity, target_acceleration = target.predict(i * dt)
        error, error_velocity, error_acceleration = _quintic_error_sample(
            bridge_state, time_s=sample_time, duration_s=duration_s
        )
        positions[i] = target_position + relative_offset + error
        velocities[i] = target_velocity + error_velocity
        accelerations[i] = target_acceleration + error_acceleration

    return (
        TrajectoryReference(positions, velocities, accelerations),
        bool(elapsed_s >= float(duration_s)),
    )


@dataclass(frozen=True)
class StationaryHoldState:
    drift_m: float
    speed_mps: float
    within_drift_bound: bool
    speed_settled: bool


@dataclass(frozen=True)
class StationaryGrantState:
    """Live physical-state gate used immediately before authority request."""

    drift_m: float
    speed_mps: float
    acceleration_mps2: float
    within_drift_bound: bool
    speed_settled: bool
    acceleration_settled: bool

    @property
    def ready(self) -> bool:
        return bool(
            self.within_drift_bound
            and self.speed_settled
            and self.acceleration_settled
        )


def stage0_errors(
    incumbent: TrajectoryReference,
    candidate: TrajectoryReference,
) -> HandoffErrors:
    return HandoffErrors(
        position_m=float(np.linalg.norm(candidate.positions[0] - incumbent.positions[0])),
        velocity_mps=float(np.linalg.norm(candidate.velocities[0] - incumbent.velocities[0])),
        acceleration_mps2=float(
            np.linalg.norm(candidate.accelerations[0] - incumbent.accelerations[0])
        ),
    )


def handoff_allowed(
    errors: HandoffErrors,
    *,
    position_tolerance_m: float,
    velocity_tolerance_mps: float,
    acceleration_tolerance_mps2: float,
) -> bool:
    return bool(
        errors.position_m <= float(position_tolerance_m)
        and errors.velocity_mps <= float(velocity_tolerance_mps)
        and errors.acceleration_mps2 <= float(acceleration_tolerance_mps2)
    )


def loaded_lift_ready_for_cpp(
    *,
    object_attached: bool,
    object_airborne: bool,
    profile_complete: bool,
    abs_quad_z_error_m: float,
    z_tolerance_m: float,
) -> bool:
    """C1F.2b prep gate: finish and track the full loaded quad lift first."""
    return bool(
        object_attached
        and object_airborne
        and profile_complete
        and np.isfinite(abs_quad_z_error_m)
        and np.isfinite(z_tolerance_m)
        and z_tolerance_m >= 0.0
        and abs_quad_z_error_m < z_tolerance_m
    )


def loaded_lift_override_allowed(
    *,
    object_attached: bool,
    object_airborne: bool,
    profile_complete: bool,
) -> bool:
    """Operator commissioning override prerequisites for loaded-lift entry.

    The override may waive only the final ideal body-Z tolerance.  It must not
    manufacture attachment, airborne-object evidence, or lift-profile completion.
    """
    return bool(object_attached and object_airborne and profile_complete)


def stationary_hold_state(
    *,
    lift_end_position: np.ndarray,
    measured_position: np.ndarray,
    measured_velocity: np.ndarray,
    max_drift_m: float,
    settle_speed_mps: float,
) -> StationaryHoldState:
    """Evaluate the physical hold before C++ preparation is allowed.

    The lift-end position is only a loose mission-validity anchor.  Once the
    vehicle settles inside that neighbourhood, the node may re-latch the actual
    measured position as the precise Python/C++ handoff origin.
    """
    lift_end_position = np.asarray(lift_end_position, dtype=float).reshape(3)
    measured_position = np.asarray(measured_position, dtype=float).reshape(3)
    measured_velocity = np.asarray(measured_velocity, dtype=float).reshape(3)
    max_drift_m = float(max_drift_m)
    settle_speed_mps = float(settle_speed_mps)
    if not np.all(np.isfinite(lift_end_position)):
        raise ValueError("lift_end_position must be finite")
    if not np.all(np.isfinite(measured_position)):
        raise ValueError("measured_position must be finite")
    if not np.all(np.isfinite(measured_velocity)):
        raise ValueError("measured_velocity must be finite")
    if not np.isfinite(max_drift_m) or max_drift_m < 0.0:
        raise ValueError("max_drift_m must be finite and non-negative")
    if not np.isfinite(settle_speed_mps) or settle_speed_mps < 0.0:
        raise ValueError("settle_speed_mps must be finite and non-negative")

    drift_m = float(np.linalg.norm(measured_position - lift_end_position))
    speed_mps = float(np.linalg.norm(measured_velocity))
    return StationaryHoldState(
        drift_m=drift_m,
        speed_mps=speed_mps,
        within_drift_bound=drift_m <= max_drift_m,
        speed_settled=speed_mps <= settle_speed_mps,
    )


def stationary_grant_state(
    *,
    lift_end_position: np.ndarray,
    measured_position: np.ndarray,
    measured_velocity: np.ndarray,
    measured_acceleration: np.ndarray,
    max_drift_m: float,
    settle_speed_mps: float,
    settle_acceleration_mps2: float,
) -> StationaryGrantState:
    """Revalidate the live physical handoff state at the instant of grant.

    ``c1f2_prepare_settled`` is only a historical event. A moving platform planner
    can take several seconds to find a prepared trajectory, so the vehicle may have
    drifted after that event. Authority must therefore be based on the current
    measured p/v/a, not on stale settle diagnostics.
    """
    hold = stationary_hold_state(
        lift_end_position=lift_end_position,
        measured_position=measured_position,
        measured_velocity=measured_velocity,
        max_drift_m=max_drift_m,
        settle_speed_mps=settle_speed_mps,
    )
    measured_acceleration = np.asarray(measured_acceleration, dtype=float).reshape(3)
    settle_acceleration_mps2 = float(settle_acceleration_mps2)
    if not np.all(np.isfinite(measured_acceleration)):
        raise ValueError("measured_acceleration must be finite")
    if (
        not np.isfinite(settle_acceleration_mps2)
        or settle_acceleration_mps2 < 0.0
    ):
        raise ValueError(
            "settle_acceleration_mps2 must be finite and non-negative"
        )
    acceleration_mps2 = float(np.linalg.norm(measured_acceleration))
    return StationaryGrantState(
        drift_m=hold.drift_m,
        speed_mps=hold.speed_mps,
        acceleration_mps2=acceleration_mps2,
        within_drift_bound=hold.within_drift_bound,
        speed_settled=hold.speed_settled,
        acceleration_settled=(
            acceleration_mps2 <= settle_acceleration_mps2
        ),
    )


def advance_reference_window(
    reference: TrajectoryReference,
    *,
    elapsed_s: float,
    sample_dt_s: float,
    output_count: int | None = None,
) -> TrajectoryReference:
    """Advance an already-committed sampled window without inventing a new path.

    Linear interpolation is used between committed samples. Queries after the
    explicit window are padded with its terminal position as a stopped hold (zero
    velocity and acceleration), so padding preserves the committed fallback rather
    than switching to an unrelated Python trajectory. A window cut mid-transit does
    not end stopped: padding its terminal velocity froze the position while still
    commanding ~0.2 m/s, and the C1F.2b recovery check rejected every fresh
    reference against that phantom velocity (M2 join stall, T0023/T0025/T0032/T0033).
    """
    if not np.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError("elapsed_s must be finite and non-negative")
    if not np.isfinite(sample_dt_s) or sample_dt_s <= 0.0:
        raise ValueError("sample_dt_s must be finite and positive")

    count = int(reference.positions.shape[0]) if output_count is None else int(output_count)
    if count <= 0:
        raise ValueError("output_count must be positive")

    source_count = int(reference.positions.shape[0])
    if source_count <= 0:
        raise ValueError("reference must contain at least one sample")

    source_times = np.arange(source_count, dtype=float) * float(sample_dt_s)
    query_times = float(elapsed_s) + np.arange(count, dtype=float) * float(sample_dt_s)

    def _resample(values: np.ndarray) -> np.ndarray:
        values = np.asarray(values, dtype=float)
        result = np.empty((count, 3), dtype=float)
        for axis in range(3):
            result[:, axis] = np.interp(
                query_times,
                source_times,
                values[:, axis],
                left=float(values[0, axis]),
                right=float(values[-1, axis]),
            )
        return result

    past_end = query_times > source_times[-1]
    velocities = _resample(reference.velocities)
    accelerations = _resample(reference.accelerations)
    velocities[past_end] = 0.0
    accelerations[past_end] = 0.0
    return TrajectoryReference(_resample(reference.positions), velocities, accelerations)


def post_grant_reference_is_fresh(
    *,
    reference_source_stamp_s: float,
    grant_ros_time_s: float,
    reference_receive_time: float,
    ack_receive_time: float,
    tolerance_s: float = 1e-9,
) -> bool:
    """Require a C++ reference that was published after the authority request.

    Two clocks are used deliberately. The ROS source stamp proves the reference
    itself was produced after the grant request, so a delayed pre-grant DDS
    packet cannot pass. Monotonic receive times additionally require that Python
    has observed the backend acknowledgement before accepting the window.
    """
    values = (
        float(reference_source_stamp_s),
        float(grant_ros_time_s),
        float(reference_receive_time),
        float(ack_receive_time),
        float(tolerance_s),
    )
    if not all(np.isfinite(value) for value in values) or tolerance_s < 0.0:
        return False
    return bool(
        reference_source_stamp_s > grant_ros_time_s + tolerance_s
        and reference_receive_time >= ack_receive_time
    )
