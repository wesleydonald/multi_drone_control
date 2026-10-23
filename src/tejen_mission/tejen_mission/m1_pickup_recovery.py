"""Bounded, opt-in M1 pickup retry bookkeeping, independent of ROS."""

from __future__ import annotations

import math
import numpy as np


class PickupProgressStall:
    """One-shot wait for meaningful commanded descent or measured clearance."""

    def __init__(self, *, wait_s: float = 10., improvement_m: float = .005):
        if not all(math.isfinite(v) and v > 0 for v in (wait_s, improvement_m)):
            raise ValueError("pickup stall timing and improvement must be positive")
        self.wait_s = float(wait_s)
        self.improvement_m = float(improvement_m)
        self.reset()

    def reset(self) -> None:
        self.key = None
        self.since = None
        self.baseline = None
        self.elapsed_s = 0.
        self.fired = False

    def step(self, *, now: float, progress_m: float, blocked: bool, key) -> bool:
        if not blocked or not all(math.isfinite(v) for v in (now, progress_m)):
            self.reset()
            return False
        if self.key != key or (self.since is not None and now < self.since):
            self.reset()
        self.key = key
        if self.fired:
            return False
        if self.since is None or progress_m - self.baseline >= self.improvement_m:
            self.since = now
            self.baseline = progress_m
        self.elapsed_s = max(0., now - self.since)
        self.fired = self.elapsed_s >= self.wait_s
        return self.fired


class PickupRelease:
    """Magnet-OFF dwell and vertical-only clearance before lateral retry."""

    def __init__(
        self, *, release_dwell_s: float = .20, speed_mps: float = .05,
        max_rise_m: float = .20, clearance_m: float = .08,
        separation_dwell_s: float = .12, max_attempts: int = 3,
    ):
        values = (release_dwell_s, speed_mps, max_rise_m, clearance_m,
                  separation_dwell_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("pickup release bounds must be positive and finite")
        if max_attempts < 1:
            raise ValueError("pickup release needs a positive attempt budget")
        self.release_dwell_s = float(release_dwell_s)
        self.speed_mps = float(speed_mps)
        self.max_rise_m = float(max_rise_m)
        self.clearance_m = float(clearance_m)
        self.separation_dwell_s = float(separation_dwell_s)
        self.max_attempts = int(max_attempts)
        self.attempts = 0
        self.reset_stage()

    def reset_stage(self) -> None:
        self.stage = "idle"
        self.target = None
        self.origin_z = None
        self.started_s = None
        self.last_time_s = None
        self.separation_since_s = None
        self.waiting_reason = ""

    def reset_mission(self) -> None:
        self.attempts = 0
        self.reset_stage()

    def begin(self, *, now: float, reference_position) -> None:
        target = np.asarray(reference_position, dtype=float).reshape(3)
        if not math.isfinite(now) or not np.all(np.isfinite(target)):
            raise ValueError("pickup release requires finite start pose/time")
        if self.stage != "idle":
            return
        self.attempts += 1
        self.stage = "release"
        self.target = target.copy()
        self.origin_z = float(target[2])
        self.started_s = self.last_time_s = float(now)
        self.separation_since_s = None
        self.waiting_reason = "magnet OFF release dwell"

    def step(
        self, *, now: float, face_clearance_m: float,
        fresh: bool, magnet_off: bool,
    ) -> bool:
        if self.stage in {"idle", "await_operator"}:
            return False
        if not math.isfinite(now) or not fresh or not magnet_off:
            self.separation_since_s = None
            self.last_time_s = now if math.isfinite(now) else self.last_time_s
            self.waiting_reason = "waiting for fresh release evidence and magnet OFF"
            return False
        dt = min(.1, max(0., now - self.last_time_s))
        self.last_time_s = now
        if now - self.started_s < self.release_dwell_s:
            self.waiting_reason = "magnet OFF release dwell"
            return False
        if math.isfinite(face_clearance_m) and face_clearance_m >= self.clearance_m:
            if self.separation_since_s is None:
                self.separation_since_s = now
            if now - self.separation_since_s >= self.separation_dwell_s:
                if self.attempts >= self.max_attempts:
                    self.stage = "await_operator"
                    self.waiting_reason = "three attempts failed; LAND or restart"
                    return False
                self.stage = "ready"
                self.waiting_reason = "measured vertical separation confirmed"
                return True
        else:
            self.separation_since_s = None
        self.target[2] = min(
            self.origin_z + self.max_rise_m,
            self.target[2] + self.speed_mps * dt,
        )
        self.waiting_reason = (
            "rise limit reached; separation unverified"
            if self.target[2] >= self.origin_z + self.max_rise_m - 1e-9
            else "vertical-only clearance"
        )
        return False


def early_candidate_ready(*, low: bool, candidate: bool, fresh: bool,
                          magnet_on: bool, clearance_m: float,
                          max_clearance_m: float = .005) -> bool:
    """A proximity flag becomes a lift-proof candidate only near the surface."""
    return bool(low and candidate and fresh and magnet_on
                and math.isfinite(clearance_m)
                and math.isfinite(max_clearance_m)
                and clearance_m <= max_clearance_m)


def lift_offsets_from_candidate_hold(hold_body_position, pickup_origin,
                                     quad_from_marker, *, lift_delta_m: float):
    """Keep the first lift body reference equal to the contact hold reference."""
    hold = np.asarray(hold_body_position, dtype=float).reshape(3)
    origin = np.asarray(pickup_origin, dtype=float).reshape(3)
    offset = np.asarray(quad_from_marker, dtype=float).reshape(3)
    if (not np.all(np.isfinite(hold)) or not np.all(np.isfinite(origin))
            or not np.all(np.isfinite(offset))
            or not math.isfinite(lift_delta_m) or lift_delta_m <= 0):
        raise ValueError('invalid early candidate lift geometry')
    start = hold - origin - offset
    return start, start + np.array([0., 0., lift_delta_m])
