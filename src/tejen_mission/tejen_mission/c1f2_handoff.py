"""Pure C1F.2/C1F.2a/C1F.2b reference-authority helpers.

Kept ROS-independent so source-selection and dropout-continuation contracts can
be regression tested without a running graph. Python remains the only ROS
authority publisher.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .reference_generators import TargetState, TrajectoryReference


class ReferenceRecovery:
    """One interrupted-backend transaction, not a second mission phase machine.

    The existing continuation is retained exactly. Only a genuinely stopped
    command may become the common seed. Status binds a grant to both a backend
    epoch and the exact reference publication; an untagged Bool ACK is insufficient.
    """

    def __init__(self) -> None:
        self.active = False
        self.request_id = ""
        self.status = None
        self.status_time_s = None
        self.epoch = -1
        self.hold_position = None
        self.references = {}
        self.full_references = {}
        self.waiting_reason = "inactive"

    def begin(self, *, request_id, reference, now_s, ros_time_ns) -> None:
        if self.active:
            return
        if reference is None or not all(np.all(np.isfinite(v)) for v in
                                       (reference.positions, reference.velocities, reference.accelerations)):
            raise ValueError("recovery requires a finite authoritative reference")
        self.__init__()
        self.active = True
        self.request_id = request_id
        self.continuation = reference
        self.started_s = float(now_s)
        self.started_ros_ns = int(ros_time_ns)
        self.waiting_reason = "reset_backend"

    def reference(self, *, now_s, dt_s):
        continued = advance_reference_window(self.continuation,
            elapsed_s=max(0., now_s - self.started_s), sample_dt_s=dt_s, output_count=61)
        # Do not zero a moving sampled tail or infer a stop from elapsed time.
        if (np.max(np.abs(continued.velocities)) <= 1e-5 and
                np.max(np.abs(continued.accelerations)) <= 1e-5 and
                np.max(np.abs(continued.positions - continued.positions[0])) <= 1e-7):
            self.hold_position = continued.positions[0].copy()
        return continued

    def request(self, *, now_s, ros_time_ns, dt_s):
        if not self.active:
            return None
        self.reference(now_s=now_s, dt_s=dt_s)
        if self.status is None:
            self.waiting_reason = "reset_backend"
            return f"{self.request_id} reset"
        if self.hold_position is None:
            self.waiting_reason = "continuation_not_stationary"
            return None
        if self.status["prepared"] or self.status["granted"]:
            self.waiting_reason = "waiting_current_grant_reference"
            return f"{self.request_id} grant {int(ros_time_ns)}"
        self.waiting_reason = self.status.get("reason", "preparing_common_hold")
        p = " ".join(format(v, ".17g") for v in self.hold_position)
        return f"{self.request_id} seed {p} {int(ros_time_ns)}"

    def observe_status(self, status, *, now_s):
        if (not self.active or status.get("request_id") != self.request_id or
                type(status.get("epoch")) is not int or status["epoch"] < self.epoch or
                type(status.get("prepared")) is not bool or type(status.get("granted")) is not bool or
                type(status.get("reference_stamp_ns")) is not int):
            return
        self.epoch = status["epoch"]
        self.status = dict(status)
        self.status_time_s = float(now_s)

    def observe_reference(self, reference, *, source_stamp_ns, receive_time_s, full_reference=None):
        """Retain a bounded set of publications until their companion status arrives."""
        if (not self.active or reference is None or type(source_stamp_ns) is not int
                or source_stamp_ns <= self.started_ros_ns
                or not np.isfinite(receive_time_s) or reference.positions.shape != (61, 3)):
            return
        # Duplicates must not renew receipt freshness. Eight windows cover the
        # existing 0.2 s timeout at 30 Hz without retaining a trajectory history.
        self.references.setdefault(source_stamp_ns, (reference, float(receive_time_s)))
        if full_reference is not None:
            self.full_references.setdefault(source_stamp_ns, full_reference)
        while len(self.references) > 8:
            oldest = next(iter(self.references))
            self.references.pop(oldest)
            self.full_references.pop(oldest, None)

    def matching_reference(self, *, now_s, timeout_s):
        if not self.active or self.status is None or not self.status["granted"]:
            return None
        stamp = self.status["reference_stamp_ns"]
        item = self.references.get(stamp)
        if (item is None or not 0. <= now_s - item[1] <= timeout_s
                or not self.source_is_fresh(stamp, now_s=now_s, timeout_s=timeout_s)):
            return None
        return item[0], stamp, item[1]

    def source_is_fresh(self, stamp_ns, *, now_s, timeout_s):
        # Use the transaction's clock correspondence, not packet arrival time.
        age = now_s - self.started_s - (stamp_ns - self.started_ros_ns) * 1e-9
        return np.isfinite(age) and -1e-9 <= age <= timeout_s

    def accept_reference(self, reference, *, source_stamp_ns, now_s, timeout_s,
                         position_tolerance_m, velocity_tolerance_mps,
                         acceleration_tolerance_mps2):
        if not (self.active and self.status is not None and self.status["granted"] and
                self.status["reference_stamp_ns"] == source_stamp_ns and
                source_stamp_ns > self.started_ros_ns and self.status_time_s is not None and
                0. <= now_s - self.status_time_s <= timeout_s and
                self.source_is_fresh(source_stamp_ns, now_s=now_s, timeout_s=timeout_s)):
            return False
        if self.hold_position is None:
            self.reference(now_s=now_s, dt_s=1/30)
        if self.hold_position is None or reference.positions.shape != (61, 3):
            return False
        held = TrajectoryReference(np.tile(self.hold_position, (61, 1)),
                                   np.zeros((61, 3)), np.zeros((61, 3)))
        source_age = max(0., now_s - self.started_s - (source_stamp_ns-self.started_ros_ns)*1e-9)
        current = advance_reference_window(reference, elapsed_s=source_age,
                                            sample_dt_s=1/30, output_count=61)
        if not handoff_allowed(stage0_errors(held, current),
                position_tolerance_m=position_tolerance_m,
                velocity_tolerance_mps=velocity_tolerance_mps,
                acceleration_tolerance_mps2=acceleration_tolerance_mps2):
            self.waiting_reason = "reference_discontinuous"
            return False
        self.active = False
        self.waiting_reason = "resumed"
        return True

    def cancel(self):
        if not self.active:
            return None
        request = f"{self.request_id} cancel"
        self.active = False
        self.waiting_reason = "cancelled"
        return request


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


# ---------------------------------------------------------------------------
# The C++ exit (docs/rig/cpp_exit_impl_contract_2026-11-15.md, section 3): one
# fitted C2 cubic B-spline in absolute time that Python publishes from and the
# backend seeds from, plus the relative blends of every cancel and takeover.
# ---------------------------------------------------------------------------

EXIT_CONTINUITY_TOLERANCE = 1e-6
# the backend's message-level C2 tolerance (trajectoryFromMessage); stamps are ns and
# absolute times ~1e9 s carry ~2e-7 s of double rounding
EXIT_JOIN_TOLERANCE = 1e-5
EXIT_IDENTITY_TOLERANCE = 1e-9
EXIT_TIME_TOLERANCE_S = 1e-6        # dynamic_planner::kTrajectoryTimeToleranceS
EXIT_JOIN_GRACE_NS = 100_000_000     # granted with no window joined by the join + this: cancel (3 periods at 30 Hz)
# half the ring commitment's 0.5 s control interval: the 1 s blend is fitted within ~2 mm (0.5 s: 1-4 cm)
EXIT_PREFIX_KNOT_S = 0.25
EXIT_BLEND_SEGMENTS = 8
EXIT_FIT_ATTEMPTS = 3
EXIT_BLEND_GROWTH = 1.5
# the copied pieces run strictly past t_R + protected_s: the backend tests the window's end to 1e-9, and a new piece
# starting exactly there differs by ~5e-6 m/s^2 at ROS-scale times
EXIT_JUNCTION_MARGIN_S = 0.05


def open_uniform_knots(t_start: float, t_end: float, segments: int) -> np.ndarray:
    """dynamic_planner::openUniformKnots (cubic, clamped), operation for operation."""
    segments = int(segments)
    if segments < 1 or not np.isfinite(t_start) or not np.isfinite(t_end) or not t_end > t_start:
        raise ValueError("invalid open-uniform knot request")
    knots = np.full(segments + 7, float(t_start))
    knots[segments + 3:] = float(t_end)
    dt = (float(t_end) - float(t_start)) / float(segments)
    for i in range(1, segments):
        knots[3 + i] = float(t_start) + dt * float(i)
    return knots


def initial_control_points(position, velocity, acceleration, knots) -> np.ndarray:
    """dynamic_planner::initialControlPointsFromState: q0..q2 that give the clamped cubic this p/v/a at its start."""
    knots = np.asarray(knots, dtype=float)
    if knots.size < 9:
        raise ValueError("knot vector is too short")
    p = 3.0
    q0 = np.asarray(position, dtype=float).reshape(3)
    v = np.asarray(velocity, dtype=float).reshape(3)
    a = np.asarray(acceleration, dtype=float).reshape(3)
    t1, t2, tp1, t1p1 = knots[1], knots[2], knots[4], knots[5]
    q1 = q0 + (-t1 + tp1) * v / p
    q2 = (p * p * q1 - (t1p1 - t2) * (a * (t2 - tp1) + v) - p * (q1 + (-t1p1 + t2) * v)) / ((p - 1.0) * p)
    return np.vstack([q0, q1, q2])


def _deboor_many(control_points, knots, degree, times) -> np.ndarray:
    """dynamic_planner::evaluateBSpline for many times (same span rule, clamp and recurrence)."""
    n = control_points.shape[0] - 1
    lo, hi = knots[degree], knots[n + 1]
    t = np.clip(np.asarray(times, dtype=float), lo, hi)
    span = np.clip(np.searchsorted(knots, t, side="right") - 1, degree, n)
    span = np.where(t >= hi, n, span)
    d = control_points[span[:, None] - degree + np.arange(degree + 1)[None, :]].copy()
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            i = span - degree + j
            lower = knots[i]
            denom = knots[i + degree - r + 1] - lower
            alpha = np.where(denom > 0.0, (t - lower) / np.where(denom > 0.0, denom, 1.0), 0.0)[:, None]
            d[:, j] = (1.0 - alpha) * d[:, j - 1] + alpha * d[:, j]
    return d[:, degree]


def _derivative_chain(knots, control_points):
    chain = [(control_points, knots, 3)]
    for _ in range(3):
        cps, kn, degree = chain[-1]
        index = np.arange(cps.shape[0] - 1)
        denom = kn[index + degree + 1] - kn[index + 1]
        if np.any(denom <= 0.0):
            raise ValueError("degenerate derivative knot interval")
        chain.append(((degree / denom)[:, None] * (cps[1:] - cps[:-1]), kn[1:-1], degree - 1))
    return chain


class SplinePiece:
    """One cubic B-spline piece in absolute time, as the backend's TrajectoryPiece: evaluated by de Boor at the time
    clamped to its logical validity."""

    def __init__(self, knots, control_points, valid_from=None, valid_until=None):
        knots = np.array(knots, dtype=float).reshape(-1)
        cps = np.array(control_points, dtype=float).reshape(-1, 3)
        n = cps.shape[0]
        if (n < 4 or knots.size != n + 4 or not np.all(np.isfinite(knots)) or not np.all(np.isfinite(cps))
                or np.any(np.diff(knots) < 0.0)):
            raise ValueError("invalid cubic B-spline piece")
        self.knots, self.control_points = knots, cps
        self.valid_from = float(knots[3] if valid_from is None else valid_from)
        self.valid_until = float(knots[n] if valid_until is None else valid_until)
        if not (np.isfinite(self.valid_from) and np.isfinite(self.valid_until)
                and self.valid_until >= self.valid_from
                and self.valid_from >= knots[3] - EXIT_TIME_TOLERANCE_S
                and self.valid_until <= knots[n] + EXIT_TIME_TOLERANCE_S):
            raise ValueError("piece validity lies outside its spline domain")
        self._chain = _derivative_chain(knots, cps)

    def trimmed(self, valid_from, valid_until) -> "SplinePiece":
        return SplinePiece(self.knots, self.control_points, valid_from, valid_until)

    def state(self, times):
        """(p, v, a, j), each (len(times), 3)."""
        t = np.clip(np.atleast_1d(np.asarray(times, dtype=float)), self.valid_from, self.valid_until)
        return tuple(_deboor_many(c, k, d, t) for c, k, d in self._chain)


class CommittedSpline:
    """Pieces combined as the backend's CommittedTrajectory::replaceSuffix does: a later piece replaces the commitment
    from its valid_from on, C2 at the splice (1e-5, trajectoryFromMessage's tolerance); the latest piece wins at a
    boundary."""

    def __init__(self, pieces):
        self.pieces = []
        for piece in pieces:
            self._replace_suffix(piece)
        if not self.pieces:
            raise ValueError("a committed spline needs a piece")

    def _replace_suffix(self, piece: SplinePiece) -> None:
        if not self.pieces:
            self.pieces.append(piece)
            return
        t = piece.valid_from
        if t < self.start - EXIT_TIME_TOLERANCE_S or t > self.end + EXIT_TIME_TOLERANCE_S:
            raise ValueError("splice lies outside the commitment")
        old, new = self.state([t]), piece.state([t])
        if any(float(np.max(np.abs(o - n))) > EXIT_JOIN_TOLERANCE for o, n in zip(old[:3], new[:3])):
            raise ValueError("pieces are not C2-continuous")
        kept = []
        for previous in self.pieces:
            if previous.valid_from >= t - EXIT_TIME_TOLERANCE_S:
                break
            if previous.valid_until > t:
                previous = previous.trimmed(previous.valid_from, t)
            if previous.valid_until > previous.valid_from + 1e-12:
                kept.append(previous)
        self.pieces = kept + [piece]

    @property
    def start(self) -> float:
        return self.pieces[0].valid_from

    @property
    def end(self) -> float:
        return self.pieces[-1].valid_until

    def covers(self, t0: float, t1: float) -> bool:
        return bool(self.start - EXIT_TIME_TOLERANCE_S <= t0 and t1 <= self.end + EXIT_TIME_TOLERANCE_S)

    def state(self, times):
        times = np.atleast_1d(np.asarray(times, dtype=float))
        if (times.size and (float(np.min(times)) < self.start - EXIT_TIME_TOLERANCE_S
                            or float(np.max(times)) > self.end + EXIT_TIME_TOLERANCE_S)):
            raise ValueError("evaluation outside the committed coverage")
        owner = np.zeros(times.size, dtype=int)
        for k, piece in enumerate(self.pieces):
            owner[times >= piece.valid_from - EXIT_TIME_TOLERANCE_S] = k
        out = [np.empty((times.size, 3)) for _ in range(4)]
        for k in np.unique(owner):
            mask = owner == k
            for dst, src in zip(out, self.pieces[k].state(times[mask])):
                dst[mask] = src
        return tuple(out)

    @classmethod
    def from_message(cls, msg) -> "CommittedSpline":
        """interfaces/CommittedTrajectory -> pieces (duck-typed, no ROS import)."""
        pieces = [SplinePiece([float(k) for k in m.knots],
                              [[float(p.x), float(p.y), float(p.z)] for p in m.control_points],
                              float(m.valid_from_s), float(m.valid_until_s)) for m in msg.pieces]
        return cls(pieces)


def _quintic_many(state: TargetRelativeBridgeState, tau, duration):
    """_quintic_error_sample_with_jerk for many tau; zero from duration on."""
    tau = np.asarray(tau, dtype=float)
    s = np.clip(tau / duration, 0.0, 1.0)[:, None]
    b0 = state.position_error
    b1 = duration * state.velocity_error
    b2 = 0.5 * duration * duration * state.acceleration_error
    b3 = -10.0 * b0 - 6.0 * b1 - 3.0 * b2
    b4 = 15.0 * b0 + 8.0 * b1 + 3.0 * b2
    b5 = -6.0 * b0 - 3.0 * b1 - b2
    e = b0 + b1 * s + b2 * s**2 + b3 * s**3 + b4 * s**4 + b5 * s**5
    ev = (b1 + 2.0 * b2 * s + 3.0 * b3 * s**2 + 4.0 * b4 * s**3 + 5.0 * b5 * s**4) / duration
    ea = (2.0 * b2 + 6.0 * b3 * s + 12.0 * b4 * s**2 + 20.0 * b5 * s**3) / duration**2
    ej = (6.0 * b3 + 24.0 * b4 * s + 60.0 * b5 * s**2) / duration**3
    done = (tau >= duration)[:, None]
    return tuple(np.where(done, 0.0, x) for x in (e, ev, ea, ej))


def _basis(knots, count, times):
    """Cubic B-spline basis values and first derivatives (len(times), count): the least-squares rows."""
    t = np.clip(np.asarray(times, dtype=float), knots[3], knots[count])
    m = t.size
    span = np.clip(np.searchsorted(knots, t, side="right") - 1, 3, count - 1)
    basis = np.zeros((m, knots.size - 1))
    basis[np.arange(m), span] = 1.0
    quadratic = None
    for p in (1, 2, 3):
        if p == 3:
            quadratic = basis
        nxt = np.zeros((m, knots.size - 1 - p))
        for i in range(knots.size - 1 - p):
            left, right = knots[i + p] - knots[i], knots[i + p + 1] - knots[i + 1]
            if left > 0.0:
                nxt[:, i] += (t - knots[i]) / left * basis[:, i]
            if right > 0.0:
                nxt[:, i] += (knots[i + p + 1] - t) / right * basis[:, i + 1]
        basis = nxt
    derivative = np.zeros((m, count))
    for i in range(count):
        left, right = knots[i + 3] - knots[i], knots[i + 4] - knots[i + 1]
        if left > 0.0:
            derivative[:, i] += 3.0 / left * quadratic[:, i]
        if right > 0.0:
            derivative[:, i] -= 3.0 / right * quadratic[:, i + 1]
    return basis, derivative


def _fit_piece(*, t0, t1, segments, start_state, desired, end_zero=False) -> SplinePiece:
    """q0..q2 from the start p/v/a (the backend's boundary equations); with end_zero the last three are zero (p, v, a
    end at exactly 0); the rest least squares to desired(times) -> (p, v)."""
    knots = open_uniform_knots(t0, t1, segments)
    count = segments + 3
    q = np.zeros((count, 3))
    q[:3] = initial_control_points(*start_state, knots)
    free_end = count - 3 if end_zero else count
    if free_end > 3:
        times = np.linspace(t0, t1, 8 * segments + 1)
        want_p, want_v = desired(times)
        basis, derivative = _basis(knots, count, times)
        weight = (t1 - t0) / segments
        lhs = np.vstack([basis[:, 3:free_end], weight * derivative[:, 3:free_end]])
        rhs = np.vstack([want_p - basis @ q, weight * (want_v - derivative @ q)])
        q[3:free_end] = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    return SplinePiece(knots, q)


@dataclass(frozen=True)
class ExitCurveCheck:
    """A fitted curve and its checks (contract 3.3): start continuity to 1e-6 and |v|, |a|, |j| under the planner's
    limits over its span. The fit error is reported, never the acceptance test."""

    piece: object
    reason: str
    blend_s: float
    attempts: int
    start_error: float = float("nan")
    fit_error_m: float = float("nan")
    peak_velocity: float = float("nan")
    peak_acceleration: float = float("nan")
    peak_jerk: float = float("nan")

    @property
    def ok(self) -> bool:
        return self.piece is not None


def _check_times(piece: SplinePiece, sample_dt: float) -> np.ndarray:
    t0, t1 = piece.valid_from, piece.valid_until
    inner = piece.knots[(piece.knots > t0) & (piece.knots < t1)]
    middles = 0.5 * (piece.knots[3:-4] + piece.knots[4:-3])
    return np.unique(np.concatenate([np.linspace(t0, t1, max(2, int(np.ceil((t1 - t0) / sample_dt)) + 1)),
                                     inner, middles[(middles > t0) & (middles < t1)]]))


def _limits_reason(v, a, j, max_velocity, max_acceleration, max_jerk, tolerance=1e-9):
    peaks = [np.max(np.abs(x), axis=0) for x in (v, a, j)]
    for name, peak, limit in zip(("VELOCITY_LIMIT", "ACCELERATION_LIMIT", "JERK_LIMIT"), peaks,
                                 (max_velocity, max_acceleration, max_jerk)):
        if np.any(peak > np.asarray(limit, dtype=float).reshape(3) + tolerance):
            return name, peaks
    return "OK", peaks


def _start_error(piece: SplinePiece, start_state) -> float:
    got = piece.state([piece.valid_from])
    return float(max(np.max(np.abs(g[0] - np.asarray(w, dtype=float).reshape(3))) for g, w in zip(got[:3], start_state)))


def fit_moving_prefix(*, start_time, end_time, start_state, target, blend_s, max_velocity, max_acceleration,
                      max_jerk, sample_dt=0.02, knot_s=EXIT_PREFIX_KNOT_S) -> ExitCurveCheck:
    """P(t) = R(t) + c + b(t) (v2 section 1), fitted from start_state (the executed p/v/a at start_time). target(times)
    -> (p, v, a) of R + c; b is the C1F.6 quintic decaying start_state - target(start_time) over the blend. A failed
    check lengthens the blend and refits, EXIT_FIT_ATTEMPTS times in all."""
    start_state = tuple(np.asarray(x, dtype=float).reshape(3) for x in start_state)
    span = float(end_time) - float(start_time)
    segments = max(2, int(np.ceil(span / knot_s - 1e-9)))
    tp, tv, ta = (x[0] for x in target(np.array([float(start_time)])))
    error = TargetRelativeBridgeState(start_state[0] - tp, start_state[1] - tv, start_state[2] - ta)
    blend = float(blend_s)
    result = None
    for attempt in range(1, EXIT_FIT_ATTEMPTS + 1):
        duration = min(blend, span)

        def desired(times, duration=duration):
            p, v, _ = target(times)
            e, ev, _, _ = _quintic_many(error, times - float(start_time), duration)
            return p + e, v + ev

        piece = _fit_piece(t0=float(start_time), t1=float(end_time), segments=segments, start_state=start_state,
                           desired=desired)
        times = _check_times(piece, sample_dt)
        p, v, a, j = piece.state(times)
        want_p, _ = desired(times)
        start = _start_error(piece, start_state)
        reason, peaks = _limits_reason(v, a, j, max_velocity, max_acceleration, max_jerk)
        if start > EXIT_CONTINUITY_TOLERANCE:
            reason = "START_DISCONTINUOUS"
        result = ExitCurveCheck(piece if reason == "OK" else None, reason, duration, attempt, start,
                                float(np.max(np.linalg.norm(p - want_p, axis=1))), *(float(np.max(x)) for x in peaks))
        if result.ok:
            return result
        blend *= EXIT_BLEND_GROWTH
    return result


def fit_relative_blend(*, error_state, blend_s, target_velocity, target_acceleration, max_velocity,
                       max_acceleration, max_jerk, sample_dt=0.02) -> ExitCurveCheck:
    """A cancel or takeover blend in relative time tau >= 0: the error e(tau) a fitted cubic from error_state (the
    executed curve's p/v/a less the new reference's) to exactly zero, added to the new reference. Checked as
    C1F.6's bridge (target_relative_bridge_feasibility): |v_ref + e'|, |a_ref + e''| and the added jerk |e'''|."""
    error_state = tuple(np.asarray(x, dtype=float).reshape(3) for x in error_state)
    error = TargetRelativeBridgeState(*error_state)
    blend = float(blend_s)
    result = None
    for attempt in range(1, EXIT_FIT_ATTEMPTS + 1):

        def desired(tau, duration=blend):
            e, ev, _, _ = _quintic_many(error, tau, duration)
            return e, ev

        piece = _fit_piece(t0=0.0, t1=blend, segments=EXIT_BLEND_SEGMENTS, start_state=error_state,
                           desired=desired, end_zero=True)
        tau = _check_times(piece, sample_dt)
        _, v, a, j = piece.state(tau)
        start = _start_error(piece, error_state)
        reason, peaks = _limits_reason(v + target_velocity(tau), a + target_acceleration(tau), j,
                                       max_velocity, max_acceleration, max_jerk)
        if start > EXIT_CONTINUITY_TOLERANCE:
            reason = "START_DISCONTINUOUS"
        result = ExitCurveCheck(piece if reason == "OK" else None, reason, blend, attempt, start, 0.0,
                                *(float(np.max(x)) for x in peaks))
        if result.ok:
            return result
        blend *= EXIT_BLEND_GROWTH
    return result


def refresh_moving_prefix(*, bound: CommittedSpline, refresh_time, protected_s, end_time, target, blend_s,
                          max_velocity, max_acceleration, max_jerk, sample_dt=0.02, knot_s=EXIT_PREFIX_KNOT_S):
    """Prefix k+1: prefix k's pieces copied unchanged over [t_R, t_R + protected_s + EXIT_JUNCTION_MARGIN_S], then a
    new piece fitted C2 from prefix k's p/v/a at that junction. Returns (CommittedSpline or None, ExitCurveCheck or
    reason)."""
    refresh_time = float(refresh_time)
    protected_end = refresh_time + float(protected_s)
    junction = protected_end + EXIT_JUNCTION_MARGIN_S
    if not bound.covers(refresh_time, junction) or junction >= float(end_time):
        return None, "PROTECTED_WINDOW_NOT_COVERED"
    copied = [p.trimmed(max(p.valid_from, refresh_time), min(p.valid_until, junction)) for p in bound.pieces
              if p.valid_until > refresh_time + 1e-12 and p.valid_from < junction - 1e-12]
    start = tuple(x[0] for x in bound.state([junction])[:3])
    check = fit_moving_prefix(start_time=junction, end_time=end_time, start_state=start, target=target,
                              blend_s=blend_s, max_velocity=max_velocity, max_acceleration=max_acceleration,
                              max_jerk=max_jerk, sample_dt=sample_dt, knot_s=knot_s)
    if not check.ok:
        return None, check
    spline = CommittedSpline(copied + [check.piece])
    times = np.linspace(refresh_time, protected_end, 21)          # the backend's identity test
    if any(float(np.max(np.abs(o - n))) > EXIT_IDENTITY_TOLERANCE
           for o, n in zip(bound.state(times)[:3], spline.state(times)[:3])):
        return None, "PROTECTED_WINDOW_CHANGED"
    return spline, check


def relative_grant_state(*, reference_position, reference_velocity, reference_acceleration, measured_position,
                         measured_velocity, measured_acceleration, max_drift_m, settle_speed_mps,
                         settle_acceleration_mps2) -> StationaryGrantState:
    """stationary_grant_state's bounds on the measured state relative to the executing reference: no zero-velocity
    requirement and no dwell (v2 section 1)."""
    return stationary_grant_state(
        lift_end_position=reference_position, measured_position=measured_position,
        measured_velocity=np.asarray(measured_velocity, dtype=float) - np.asarray(reference_velocity, dtype=float),
        measured_acceleration=(np.asarray(measured_acceleration, dtype=float)
                               - np.asarray(reference_acceleration, dtype=float)),
        max_drift_m=max_drift_m, settle_speed_mps=settle_speed_mps,
        settle_acceleration_mps2=settle_acceleration_mps2)


class MovingExitHandoff(ReferenceRecovery):
    """The C++ exit's one transaction on the existing recovery protocol: `reset`, then `seedtraj exit <sequence>
    <stamp_ns> <deadline_ns>` once per published prefix, then `grant`. Its 7 s budget starts when the clearance
    completes (start()); prefix fits, rejections and refreshes never move it. Status and reference bookkeeping are
    ReferenceRecovery's (request id, monotonic epoch, references kept by source stamp)."""

    def __init__(self) -> None:
        super().__init__()
        self.state = "idle"     # idle -> preparing -> accepted -> granted -> done; or failed / landed
        self.t0_s = None
        self.deadline_s = None
        self.deadline_ns = 0
        self.prefix = None
        self.bound = []         # recent prefixes, oldest first: a window planned on k joins after k+1 is out
        self.sequence = 0
        self.prefix_stamp_ns = 0
        self.last_refresh_s = None
        self.offset = None
        self.reset_sent = False
        self.sent_sequence = 0
        self.failure = ""

    def start(self, *, now_s, now_ns, timeout_s) -> None:
        self.state = "preparing"
        self.t0_s = float(now_s)
        self.deadline_s = self.t0_s + float(timeout_s)
        self.deadline_ns = int(now_ns) + int(round(float(timeout_s) * 1e9))

    def open(self, *, request_id, now_s, now_ns) -> None:
        self.active = True
        self.request_id = request_id
        self.started_s = float(now_s)
        self.started_ros_ns = int(now_ns)
        self.waiting_reason = "reset_backend"

    def bind(self, prefix: CommittedSpline, *, stamp_ns, now_s) -> int:
        self.prefix = prefix
        self.bound = (self.bound + [prefix])[-3:]
        self.sequence += 1
        self.prefix_stamp_ns = int(stamp_ns)
        self.last_refresh_s = float(now_s)
        return self.sequence

    def requests(self) -> list:
        """This tick's transaction messages: the reset once, each new prefix's seedtraj once (a tick after its prefix
        was published, so the backend holds it before the request)."""
        if not self.active or self.state != "preparing" or self.sequence == 0:
            return []
        out = []
        if not self.reset_sent:
            out.append(f"{self.request_id} reset")
            self.reset_sent = True
        if self.sent_sequence != self.sequence:
            out.append(f"{self.request_id} seedtraj exit {self.sequence} {self.prefix_stamp_ns} {self.deadline_ns}")
            self.sent_sequence = self.sequence
        return out

    def grant_ready(self) -> bool:
        return bool(self.active and self.state == "preparing" and self.sent_sequence >= 1
                    and self.status is not None and (self.status["prepared"] or self.status["granted"]))

    def reject_reason(self) -> str:
        reason = None if self.status is None else self.status.get("exit_reject_reason", "")
        return reason if isinstance(reason, str) else ""

    def source_is_fresh(self, stamp_ns, *, now_s, timeout_s):
        # as ReferenceRecovery's, with the planner's time tolerance: a window received at its own stamp read as
        # 'from the future' by float rounding at ROS-scale times (replay 15 Nov: 21 of 44 runs)
        age = now_s - self.started_s - (stamp_ns - self.started_ros_ns) * 1e-9
        return bool(np.isfinite(age) and -EXIT_TIME_TOLERANCE_S <= age <= timeout_s)

    def window_matches(self, window, *, stamp_ns, dt, tolerance=EXIT_JOIN_TOLERANCE):
        """The granted window against the prefix at the same absolute times (its stamp + i dt), sample 0 to the joining
        time (status splice_time_ns; sample 0 alone without it), on the newest bound prefix that starts by the window
        (the copies agree on their protected windows). A window stamped after its join shows no prefix: refused.
        Returns (ok, (dp, dv, da))."""
        times = int(stamp_ns) * 1e-9 + np.arange(window.positions.shape[0]) * float(dt)
        join = None if self.status is None else self.status.get("splice_time_ns")
        join_s = join * 1e-9 if type(join) is int and join > 0 else times[0]
        prefix = next((b for b in reversed(self.bound) if b.start - EXIT_TIME_TOLERANCE_S <= times[0]), None)
        # float seconds near 1.8e9 resolve 2.4e-7 s: compare to the planner's time tolerance, not 1e-9
        if prefix is None or times[0] > join_s + EXIT_TIME_TOLERANCE_S or times[0] > prefix.end:
            return False, None
        mask = (times <= join_s + EXIT_TIME_TOLERANCE_S) & (times <= prefix.end + EXIT_TIME_TOLERANCE_S)
        mask[0] = True
        p, v, a, _ = prefix.state(times[mask])
        errors = tuple(float(np.max(np.linalg.norm(got[mask] - want, axis=1)))
                       for got, want in ((window.positions, p), (window.velocities, v), (window.accelerations, a)))
        return bool(max(errors) <= tolerance), errors

    def close(self, state, reason="") -> str | None:
        """End the transaction locally; returns its Cancel request (None if none was opened)."""
        self.state = state
        if reason:
            self.failure = reason
        return self.cancel()
