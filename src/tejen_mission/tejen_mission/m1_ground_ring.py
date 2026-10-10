"""Small geometry and operator-LAND policy for the opt-in M1 ground-ring trial.

This module does not infer attachment. The trial remains unverified until an
operator examines physical evidence; it only orders release and departure.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def plate0_world(centre, yaw_rad: float, pitch_diameter_m: float = .4475) -> np.ndarray:
    centre = np.asarray(centre, dtype=float).reshape(3)
    if (not np.all(np.isfinite(centre)) or not math.isfinite(yaw_rad)
            or not math.isfinite(pitch_diameter_m) or pitch_diameter_m <= 0):
        raise ValueError("invalid ring plate geometry")
    radius = pitch_diameter_m / 2
    return centre + np.array([radius * math.cos(yaw_rad),
                              radius * math.sin(yaw_rad), 0.], dtype=float)


def minimum_face_gap(ring_plane_z_m: float, min_body_z_m: float,
                     body_to_marker_m: float, marker_to_face_m: float) -> float:
    values = (ring_plane_z_m, min_body_z_m, body_to_marker_m, marker_to_face_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite ground-ring reachability geometry")
    if body_to_marker_m <= 0 or marker_to_face_m <= 0:
        raise ValueError("contact offsets must be positive downward distances")
    return min_body_z_m - body_to_marker_m - marker_to_face_m - ring_plane_z_m




@dataclass(frozen=True)
class MeasuredRingState:
    position: np.ndarray
    yaw_rad: float
    velocity: np.ndarray
    pitch_diameter_m: float

    @property
    def plate0(self) -> np.ndarray:
        return plate0_world(self.position, self.yaw_rad, self.pitch_diameter_m)

    def future_points(self, count: int) -> list[np.ndarray]:
        if count < 1:
            raise ValueError("future must contain at least one point")
        return [self.position.copy() for _ in range(count)]


class MeasuredRingSource:
    """Fresh measured pose; each advertised future is stationary at that pose."""

    def __init__(self, *, timeout_s: float = .5, max_tilt_deg: float = 5.,
                 pitch_diameter_m: float = .4475):
        if (not math.isfinite(timeout_s) or timeout_s <= 0
                or not math.isfinite(max_tilt_deg) or max_tilt_deg < 0):
            raise ValueError("invalid measured-ring freshness/tilt")
        self.timeout_s = float(timeout_s)
        self.max_tilt_deg = float(max_tilt_deg)
        self.pitch_diameter_m = float(pitch_diameter_m)
        self._state: MeasuredRingState | None = None
        self._stamp_s: float | None = None

    def ingest(self, position, quaternion_xyzw, *, now_s: float) -> bool:
        point = np.asarray(position, dtype=float).reshape(3)
        quat = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
        if (not math.isfinite(now_s) or not np.all(np.isfinite(point))
                or not np.all(np.isfinite(quat))):
            self._state = None
            return False
        norm = float(np.linalg.norm(quat))
        if norm < 1e-9:
            self._state = None
            return False
        x, y, z, w = quat / norm
        tilt = math.degrees(math.acos(max(-1., min(1., 1. - 2. * (x*x + y*y)))))
        if tilt > self.max_tilt_deg:
            self._state = None
            return False
        yaw = math.atan2(2. * (w*z + x*y), 1. - 2. * (y*y + z*z))
        self._state = MeasuredRingState(point.copy(), yaw, np.zeros(3),
                                        self.pitch_diameter_m)
        self._stamp_s = float(now_s)
        return True

    def state(self, now_s: float) -> MeasuredRingState | None:
        if self._state is None or self._stamp_s is None or not math.isfinite(now_s):
            return None
        if not 0. <= now_s - self._stamp_s <= self.timeout_s:
            return None
        return self._state


@dataclass(frozen=True)
class GroundRingExitDecision:
    stage: str
    target_position: np.ndarray
    magnet_on: bool = False
    ready_to_land: bool = False
    waiting_reason: str = ""


class GroundRingExit:
    """OFF, vertical-only clearance, return to takeoff XY, then LAND."""

    def __init__(self, start_position, pad_xy, *, started_s: float,
                 release_dwell_s: float = .25, rise_limit_m: float = .20,
                 rise_rate_mps: float = .03, required_face_gap_m: float = .08,
                 separation_dwell_s: float = .12, pad_tolerance_m: float = .05):
        self.start = np.asarray(start_position, dtype=float).reshape(3).copy()
        self.pad_xy = np.asarray(pad_xy, dtype=float).reshape(2).copy()
        if (not np.all(np.isfinite(self.start)) or not np.all(np.isfinite(self.pad_xy))
                or not math.isfinite(started_s)):
            raise ValueError("invalid ground-ring exit origin or pad")
        self.started_s = float(started_s)
        self.release_dwell_s = float(release_dwell_s)
        self.rise_limit_m = float(rise_limit_m)
        self.rise_rate_mps = float(rise_rate_mps)
        self.required_face_gap_m = float(required_face_gap_m)
        self.separation_dwell_s = float(separation_dwell_s)
        self.pad_tolerance_m = float(pad_tolerance_m)
        self.stage = "release_wait"
        self.separated_since_s: float | None = None
        self.return_z_m: float | None = None
        self.blocked_target: np.ndarray | None = None

    def step(self, now_s: float, *, face_gap_m: float, fresh: bool,
             body_position, body_speed_mps: float) -> GroundRingExitDecision:
        body = np.asarray(body_position, dtype=float).reshape(3)
        if not math.isfinite(now_s) or not np.all(np.isfinite(body)):
            return GroundRingExitDecision("blocked", self.start.copy(),
                                          waiting_reason="invalid body/time evidence")
        if self.stage == "release_wait":
            if now_s - self.started_s < self.release_dwell_s:
                return GroundRingExitDecision(self.stage, self.start.copy(),
                                              waiting_reason="magnet OFF release dwell")
            self.stage = "rise"
        if self.stage == "rise":
            separated = bool(fresh and math.isfinite(face_gap_m)
                             and face_gap_m >= self.required_face_gap_m)
            if separated:
                if self.separated_since_s is None:
                    self.separated_since_s = now_s
                elif now_s - self.separated_since_s >= self.separation_dwell_s:
                    self.return_z_m = max(self.start[2], body[2])
                    self.stage = "return"
            else:
                self.separated_since_s = None
            if self.stage == "rise":
                if body[2] >= self.start[2] + self.rise_limit_m - .005 and not separated:
                    self.stage = "blocked"
                elif not fresh:
                    return GroundRingExitDecision(self.stage,
                        np.array([self.start[0], self.start[1], body[2]]),
                        waiting_reason="fresh ring/magnet evidence required")
                else:
                    rise = min(self.rise_limit_m, self.rise_rate_mps * max(
                        0., now_s - self.started_s - self.release_dwell_s))
                    return GroundRingExitDecision(self.stage,
                        np.array([self.start[0], self.start[1], self.start[2] + rise]),
                        waiting_reason="vertical separation not yet proven")
        if self.stage == "return":
            if not fresh or not math.isfinite(face_gap_m) or face_gap_m < self.required_face_gap_m:
                self.stage = "blocked"
            elif (np.linalg.norm(body[:2] - self.pad_xy) <= self.pad_tolerance_m
                    and math.isfinite(body_speed_mps) and body_speed_mps <= .08):
                self.stage = "land_ready"
            else:
                return GroundRingExitDecision(self.stage,
                    np.array([self.pad_xy[0], self.pad_xy[1], self.return_z_m]))
        if self.stage == "land_ready":
            return GroundRingExitDecision(self.stage,
                np.array([self.pad_xy[0], self.pad_xy[1], self.return_z_m]),
                ready_to_land=True)
        if self.blocked_target is None:
            self.blocked_target = body.copy()
        return GroundRingExitDecision("blocked",
            self.blocked_target.copy(),
            waiting_reason="no measured separation; hold for operator")
