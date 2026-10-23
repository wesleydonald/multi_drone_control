"""Shared measured-contact XY trim; mission authority remains in adapters."""
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


def advance_contact_height(height_m, floor_m, speed_mps, dt_s):
    """One bounded downward command increment; no elapsed-time catch-up."""
    return max(float(floor_m), float(height_m) - float(speed_mps) * max(0., float(dt_s)))


def bounded_xy_trim(offset, filtered_error, error, dt, config):
    """Shared integration kernel; callers own freshness, phase and magnet rules."""
    alpha = 1.0 - math.exp(-dt / config.filter_tau_s)
    filtered = filtered_error + alpha * (error - filtered_error)
    velocity = config.gain_per_s * filtered
    speed = float(np.linalg.norm(velocity))
    if speed > config.max_rate_mps:
        velocity *= config.max_rate_mps / speed
    candidate = offset + velocity * dt
    magnitude = float(np.linalg.norm(candidate))
    candidate = candidate * min(1.0, config.max_offset_m / magnitude) if magnitude else candidate
    return candidate, filtered


@dataclass(frozen=True)
class TipAlignmentConfig:
    gain_per_s: float = 0.8
    filter_tau_s: float = 0.5
    max_rate_mps: float = 0.025
    max_offset_m: float = 0.08
    min_normal_m: float = 0.08
    pre_low_normal_m: float = 0.10
    capture_xy_m: float = 0.025
    capture_dwell_s: float = 0.5
    low_xy_m: float = 0.025
    low_dwell_s: float = 0.3
    timeout_s: float = 10.0
    max_relative_speed_mps: float = 0.08


@dataclass(frozen=True)
class TipAlignmentDecision:
    offset_xy: np.ndarray
    updating: bool
    capture_ready: bool
    pre_low_ready: bool
    pre_low_active: bool
    raise_to_safe_clearance: bool
    timed_out: bool
    reason: str


class TipAlignment:
    """Rate/offset-bounded IRL body-target trim from measured contact error."""

    def __init__(self, config: Optional[TipAlignmentConfig] = None) -> None:
        self.config = config or TipAlignmentConfig()
        self.offset_xy = np.zeros(2)
        self.filtered_error_xy = np.zeros(2)
        self.last_time_s: Optional[float] = None
        self.capture_since_s: Optional[float] = None
        self.pre_low_since_s: Optional[float] = None
        self.phase_started_s: Optional[float] = None
        self.last_phase: Optional[str] = None

    def start_retry(self) -> None:
        self.capture_since_s = None
        self.pre_low_since_s = None
        self.phase_started_s = None
        self.last_phase = None

    def reset_assignment(self) -> None:
        self.offset_xy[:] = 0.0
        self.filtered_error_xy[:] = 0.0
        self.last_time_s = None
        self.start_retry()

    def step(
        self, *, now_s: float, error_world_xy: np.ndarray, xy_error_m: float,
        normal_error_m: float, relative_speed_mps: float, fresh: bool,
        magnet_on: bool, phase: str, preserve_aligned_crossing: bool = False,
    ) -> TipAlignmentDecision:
        cfg = self.config
        now = float(now_s)
        error = np.asarray(error_world_xy, dtype=float)
        valid = bool(
            fresh and error.shape == (2,) and np.all(np.isfinite(error))
            and all(math.isfinite(float(x)) for x in
                    (now, xy_error_m, normal_error_m, relative_speed_mps))
        )
        dt = 0.0 if self.last_time_s is None else min(0.1, max(0.0, now - self.last_time_s))
        self.last_time_s = now
        if phase != self.last_phase:
            self.capture_since_s = None
            self.pre_low_since_s = None
            self.phase_started_s = now if phase in {"capture", "high"} else None
            self.last_phase = phase
        elif self.phase_started_s is None and phase in {"capture", "high"}:
            self.phase_started_s = now
        safe = valid and normal_error_m >= cfg.min_normal_m
        slow = valid and relative_speed_mps <= cfg.max_relative_speed_mps
        updating = bool(
            phase in {"capture", "high"} and safe and slow and not magnet_on
        )
        if updating and dt > 0.0:
            self.offset_xy, self.filtered_error_xy = bounded_xy_trim(
                self.offset_xy, self.filtered_error_xy, error, dt, cfg)
        capture_ok = bool(phase == "capture" and safe and slow and not magnet_on
                          and abs(xy_error_m) <= cfg.capture_xy_m)
        self.capture_since_s = now if capture_ok and self.capture_since_s is None else (
            self.capture_since_s if capture_ok else None
        )
        capture_ready = bool(capture_ok and now - self.capture_since_s >= cfg.capture_dwell_s)
        pre_low_active = bool(phase == "high" and valid and normal_error_m <= cfg.pre_low_normal_m)
        # The clearance floor guards earning alignment and lateral trim, not an
        # already-aligned downward crossing. Otherwise measured Z below the
        # command can repeatedly revoke readiness before the command reaches LOW.
        aligned_crossing = bool(
            preserve_aligned_crossing and self.pre_low_since_s is not None
            and now - self.pre_low_since_s >= cfg.low_dwell_s
        )
        pre_low_ok = bool(phase == "high" and (safe or aligned_crossing)
                          and valid and slow and not magnet_on
                          and abs(xy_error_m) <= cfg.low_xy_m)
        self.pre_low_since_s = now if pre_low_ok and self.pre_low_since_s is None else (
            self.pre_low_since_s if pre_low_ok else None
        )
        pre_low_ready = bool(pre_low_active and pre_low_ok
                             and now - self.pre_low_since_s >= cfg.low_dwell_s)
        raise_to_safe_clearance = bool(
            phase == "high" and valid and normal_error_m < cfg.min_normal_m and not pre_low_ready
        )
        timed_out = bool(
            phase in {"capture", "high"} and self.phase_started_s is not None
            and now - self.phase_started_s >= cfg.timeout_s
            and not (capture_ready if phase == "capture" else pre_low_ready)
        )
        reason = ("stale_evidence" if not valid else
                  "magnet_on" if magnet_on else
                  "raise_to_safe_clearance" if raise_to_safe_clearance else
                  "relative_speed" if not slow else
                  "capture_clearance" if phase == "capture" and not safe else
                  "capture_xy" if phase == "capture" and abs(xy_error_m) > cfg.capture_xy_m else
                  "capture_dwell" if phase == "capture" and not capture_ready else
                  "pre_low_xy" if pre_low_active and abs(xy_error_m) > cfg.low_xy_m else
                  "pre_low_dwell" if pre_low_active and not pre_low_ready else
                  "pre_low_aligned" if pre_low_ready else
                  "high_descent")
        return TipAlignmentDecision(
            self.offset_xy.copy(), updating, capture_ready, pre_low_ready,
            pre_low_active, raise_to_safe_clearance, timed_out, reason,
        )


class TrackingTipAlignment:
    """Local moving-target trim with no dwell, phase transition or attachment authority."""

    def __init__(self, config=None, sample_timeout_s=.5):
        self.config = config or TipAlignmentConfig()
        values = (self.config.gain_per_s, self.config.filter_tau_s,
                  self.config.max_rate_mps, self.config.max_offset_m,
                  self.config.min_normal_m, self.config.max_relative_speed_mps,
                  sample_timeout_s)
        if (not all(math.isfinite(v) and v >= 0 for v in values)
                or self.config.filter_tau_s <= 0 or sample_timeout_s <= 0):
            raise ValueError('invalid target alignment profile')
        self.sample_timeout_s = sample_timeout_s
        self.offset_xy = np.zeros(2)
        self.filtered_error_xy = np.zeros(2)
        self.last_sample = None
        self.last_time_s = None
        self.relative_velocity = np.zeros(3)
        self.speed_ready = False
        self.relative_speed_mps = float('nan')
        self.xy_error_m = self.clearance_m = float('nan')
        self.updating = False
        self.reason = 'inactive'
        self.descent = None

    @property
    def saturated(self):
        return bool(np.linalg.norm(self.offset_xy) >= self.config.max_offset_m - 1e-9)

    def invalidate(self, reason='stale_evidence'):
        self.last_time_s = self.last_sample = None
        self.relative_velocity[:] = 0.
        self.filtered_error_xy[:] = 0.
        self.speed_ready = self.updating = False
        self.relative_speed_mps = float('nan')
        self.reason = reason
        if self.descent is not None:
            self.descent.invalidate()

    def reset(self):
        self.descent = None
        self.offset_xy[:] = 0.
        self.xy_error_m = self.clearance_m = float('nan')
        self.invalidate('inactive')

    def step(self, *, now_s, sample_stamp_s, relative_position, error_world_xy,
             clearance_m, fresh, learning, frozen_reason='phase_frozen'):
        relative = np.asarray(relative_position, dtype=float)
        error = np.asarray(error_world_xy, dtype=float)
        if (not fresh or relative.shape != (3,) or error.shape != (2,)
                or not np.all(np.isfinite(relative)) or not np.all(np.isfinite(error))
                or not all(math.isfinite(v) for v in (now_s, sample_stamp_s, clearance_m))):
            self.invalidate()
            return
        self.xy_error_m = float(np.linalg.norm(error))
        self.clearance_m = float(clearance_m)
        if self.last_sample is not None:
            delta = sample_stamp_s - self.last_sample[0]
            if delta < 0 or delta > self.sample_timeout_s:
                self.invalidate('reacquiring_speed')
            elif delta > 1e-6:
                self.relative_velocity = (.35 * (relative-self.last_sample[1])/delta
                                          + .65*self.relative_velocity)
                self.speed_ready = True
        if self.last_sample is None or sample_stamp_s > self.last_sample[0] + 1e-6:
            self.last_sample = (sample_stamp_s, relative.copy())
        self.relative_speed_mps = (float(np.linalg.norm(self.relative_velocity))
                                   if self.speed_ready else float('nan'))
        self.reason = (frozen_reason if not learning else
                       'reacquiring_speed' if not self.speed_ready else
                       'clearance' if clearance_m < self.config.min_normal_m else
                       'relative_speed' if self.relative_speed_mps > self.config.max_relative_speed_mps else
                       'adapting')
        self.updating = self.reason == 'adapting'
        dt = 0. if self.last_time_s is None else min(.1, max(0., now_s-self.last_time_s))
        self.last_time_s = now_s if self.updating else None
        if self.updating and dt > 0:
            self.offset_xy, self.filtered_error_xy = bounded_xy_trim(
                self.offset_xy, self.filtered_error_xy, error, dt, self.config)

    def start_descent(self, **geometry):
        """Transfer the learned trim to the existing tier kernel; one trim owner."""
        alignment = TipAlignment(self.config)
        alignment.offset_xy = self.offset_xy.copy()
        alignment.filtered_error_xy = self.filtered_error_xy.copy()
        self.descent = TieredDescent(alignment, **geometry)

    def step_descent(self, *, now_s, sample_stamp_s, relative_position,
                     error_world_xy, clearance_m, fresh):
        # Keep the existing relative-motion filter, but only the tier kernel
        # integrates trim during descent. LOW and stale data freeze both clocks.
        self.step(now_s=now_s, sample_stamp_s=sample_stamp_s,
                  relative_position=relative_position, error_world_xy=error_world_xy,
                  clearance_m=clearance_m, fresh=fresh, learning=False)
        if self.descent is None:
            return
        self.descent.step(now_s=now_s, error_world_xy=error_world_xy,
                          clearance_m=clearance_m,
                          relative_speed_mps=self.relative_speed_mps,
                          fresh=fresh and self.speed_ready)
        self.offset_xy = self.descent.alignment.offset_xy.copy()
        self.filtered_error_xy = self.descent.alignment.filtered_error_xy.copy()
        self.reason = self.descent.reason
        self.updating = bool(fresh and self.speed_ready and self.descent.band == 'HIGH'
                             and clearance_m >= self.config.min_normal_m
                             and self.relative_speed_mps <= self.config.max_relative_speed_mps)


class TieredDescent:
    """M1's local HIGH/LOW progress, not a separate mission state machine.

    Uses the same alignment dwell/crossing rules and height increment as M2.
    The adapter owns magnet commands, contact authority, lift and retries.
    """

    def __init__(self, alignment, *, start_height_m, floor_m, speed_mps,
                 high_xy_m=.05, low_speed_mps=None, low_entry_height_m=None,
                 pause_low_for_xy=True):
        self.alignment = alignment
        self.height_m = float(start_height_m)
        self.raise_ceiling_m = max(float(start_height_m), alignment.config.pre_low_normal_m)
        self.floor_m = float(floor_m)
        self.speed_mps = float(speed_mps)
        self.high_xy_m = float(high_xy_m)
        self.low_speed_mps = self.speed_mps if low_speed_mps is None else float(low_speed_mps)
        self.low_entry_height_m = (alignment.config.min_normal_m if low_entry_height_m is None
                                  else float(low_entry_height_m))
        self.pause_low_for_xy = bool(pause_low_for_xy)
        if (not all(math.isfinite(v) for v in
                    (self.height_m, self.floor_m, self.speed_mps, self.high_xy_m,
                     self.low_speed_mps, self.low_entry_height_m))
                or min(self.speed_mps, self.low_speed_mps) <= 0
                or not self.floor_m < self.low_entry_height_m <= alignment.config.min_normal_m
                or self.high_xy_m < alignment.config.low_xy_m
                or self.height_m < self.floor_m
                or self.floor_m >= alignment.config.min_normal_m):
            raise ValueError('invalid tiered pickup descent geometry or speed')
        self.band = 'HIGH'
        self.last_time_s = None
        self.reason = 'high_descent'
        self.alignment.start_retry()
        self.alignment.last_time_s = None

    @property
    def complete(self):
        return self.band == 'LOW' and self.height_m <= self.floor_m + 1e-9

    def invalidate(self):
        self.last_time_s = None
        self.alignment.last_time_s = None
        self.alignment.capture_since_s = None
        self.alignment.pre_low_since_s = None
        self.reason = 'stale_evidence'

    def step(self, *, now_s, error_world_xy, clearance_m, relative_speed_mps, fresh):
        error = np.asarray(error_world_xy, dtype=float)
        if (not fresh or error.shape != (2,) or not np.all(np.isfinite(error))
                or not all(math.isfinite(float(v)) for v in
                           (now_s, clearance_m, relative_speed_mps))):
            self.invalidate()
            return
        dt = 0. if self.last_time_s is None else min(.1, max(0., now_s-self.last_time_s))
        self.last_time_s = now_s
        xy = float(np.linalg.norm(error))
        tip = self.alignment.step(now_s=now_s, error_world_xy=error, xy_error_m=xy,
            normal_error_m=clearance_m, relative_speed_mps=relative_speed_mps,
            fresh=True, magnet_on=self.band == 'LOW', phase=self.band.lower(),
            preserve_aligned_crossing=True)
        cfg = self.alignment.config
        if self.band == 'HIGH':
            if tip.raise_to_safe_clearance:
                # Measured clearance can be low even above the commanded staging gap.
                # Rise by that deficit, capped by the original staging reference.
                self.height_m += min(self.speed_mps*dt,
                                     max(0., cfg.pre_low_normal_m-clearance_m),
                                     max(0., self.raise_ceiling_m-self.height_m))
                self.reason = tip.reason
                return
            if not tip.pre_low_ready and (tip.pre_low_active or
                    self.height_m <= self.low_entry_height_m+self.speed_mps*dt):
                self.reason = tip.reason
                return
        tolerance = cfg.low_xy_m if self.band == 'LOW' else self.high_xy_m
        if xy > tolerance and (self.band == 'HIGH' or self.pause_low_for_xy):
            self.reason = 'Z frozen while XY catches up'
            return
        speed = self.low_speed_mps if self.band == 'LOW' else self.speed_mps
        self.height_m = advance_contact_height(self.height_m, self.floor_m, speed, dt)
        if self.height_m <= self.low_entry_height_m and tip.pre_low_ready:
            self.band = 'LOW'
        self.reason = 'contact_height_reached' if self.complete else 'descent advancing'
