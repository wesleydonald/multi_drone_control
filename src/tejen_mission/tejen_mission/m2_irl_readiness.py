"""ROS-independent fail-closed readiness gate for the two-drone M2 IRL mission."""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, Sequence


def link_statistics_fresh(last_stamp_s: float | None, *, now_s: float, timeout_s: float) -> bool:
    if last_stamp_s is None:
        return False
    stamp, now, timeout = float(last_stamp_s), float(now_s), float(timeout_s)
    return bool(
        math.isfinite(stamp) and math.isfinite(now) and math.isfinite(timeout)
        and timeout > 0.0 and 0.0 <= now - stamp <= timeout
    )


@dataclass(frozen=True)
class VehicleReadinessEvidence:
    body_stamp_s: float
    magnet_stamp_s: float
    speed_mps: float
    armed: bool
    arming_stamp_s: float
    planner_stamp_s: float
    backend_stamp_s: float
    controller_stamp_s: float
    telemetry_stamp_s: float


@dataclass(frozen=True)
class ReadinessSnapshot:
    ready: bool
    vehicle_ready: Mapping[str, bool]
    vehicle_reasons: Mapping[str, tuple[str, ...]]
    stationary_dwell_s: float
    reason: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "vehicle_ready", MappingProxyType(dict(self.vehicle_ready))
        )
        object.__setattr__(
            self, "vehicle_reasons", MappingProxyType(dict(self.vehicle_reasons))
        )


class TwoDroneIRLReadinessGate:
    def __init__(
        self,
        *,
        vehicle_ids: Sequence[str] = ("drone_0", "drone_1"),
        timeout_s: float = 0.25,
        status_timeout_s: float = 1.25,
        stationary_dwell_s: float = 1.0,
        max_stationary_speed_mps: float = 0.05,
    ) -> None:
        ids = tuple(str(value) for value in vehicle_ids)
        if ids != ("drone_0", "drone_1"):
            raise ValueError("minimum M2 IRL baseline requires drone_0 and drone_1")
        self.vehicle_ids = ids
        self.timeout_s = float(timeout_s)
        self.status_timeout_s = float(status_timeout_s)
        self.required_dwell_s = float(stationary_dwell_s)
        self.max_stationary_speed_mps = float(max_stationary_speed_mps)
        if self.timeout_s <= 0.0 or self.status_timeout_s <= 0.0 or self.required_dwell_s < 0.0:
            raise ValueError("readiness timing values are invalid")
        if self.max_stationary_speed_mps < 0.0:
            raise ValueError("max_stationary_speed_mps must be non-negative")
        self.evidence: dict[str, VehicleReadinessEvidence] = {}
        self.ring_stamp_s: float | None = None
        self.link_statistics_stamps: dict[str, float] = {}
        self._stationary_start_s: float | None = None

    def update_ring(self, *, stamp_s: float) -> None:
        value = float(stamp_s)
        if not math.isfinite(value):
            raise ValueError("ring stamp must be finite")
        self.ring_stamp_s = value

    def note_link_statistics(self, vehicle_id: str, *, stamp_s: float) -> None:
        key = str(vehicle_id)
        if key not in self.vehicle_ids:
            raise KeyError(key)
        stamp = float(stamp_s)
        if not math.isfinite(stamp):
            raise ValueError("link statistics stamp must be finite")
        self.link_statistics_stamps[key] = stamp

    def update_vehicle(self, vehicle_id: str, **values) -> None:
        key = str(vehicle_id)
        if key not in self.vehicle_ids:
            raise KeyError(key)
        evidence = VehicleReadinessEvidence(**values)
        numeric = (
            evidence.body_stamp_s,
            evidence.magnet_stamp_s,
            evidence.speed_mps,
            evidence.arming_stamp_s,
            evidence.planner_stamp_s,
            evidence.backend_stamp_s,
            evidence.controller_stamp_s,
            evidence.telemetry_stamp_s,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("readiness evidence must be finite")
        self.evidence[key] = evidence

    def evaluate(self, now_s: float) -> ReadinessSnapshot:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        ring_fresh = self.ring_stamp_s is not None and self._fresh(now, self.ring_stamp_s)
        vehicle_ready: dict[str, bool] = {}
        vehicle_reasons: dict[str, tuple[str, ...]] = {}
        for vehicle_id in self.vehicle_ids:
            evidence = self.evidence.get(vehicle_id)
            if evidence is None:
                vehicle_ready[vehicle_id] = False
                vehicle_reasons[vehicle_id] = ("missing_evidence",)
                continue
            reasons: list[str] = []
            for name, stamp in (
                ("body_stale", evidence.body_stamp_s),
                ("magnet_stale", evidence.magnet_stamp_s),
                ("arming_stale", evidence.arming_stamp_s),
                ("telemetry_stale", evidence.telemetry_stamp_s),
            ):
                if not self._fresh(now, stamp):
                    reasons.append(name)
            for name, stamp in (
                ("planner_stale", evidence.planner_stamp_s),
                ("backend_stale", evidence.backend_stamp_s),
                ("controller_stale", evidence.controller_stamp_s),
            ):
                if not self._status_fresh(now, stamp):
                    reasons.append(name)
            if not link_statistics_fresh(
                self.link_statistics_stamps.get(vehicle_id),
                now_s=now, timeout_s=self.timeout_s,
            ):
                reasons.append("link_statistics_stale")
            if evidence.armed:
                reasons.append("armed")
            if abs(evidence.speed_mps) > self.max_stationary_speed_mps:
                reasons.append("moving")
            vehicle_reasons[vehicle_id] = tuple(reasons)
            vehicle_ready[vehicle_id] = not reasons
        instant_ready = ring_fresh and all(vehicle_ready.values())
        if instant_ready:
            if self._stationary_start_s is None:
                self._stationary_start_s = now
        else:
            self._stationary_start_s = None
        dwell = (
            0.0
            if self._stationary_start_s is None
            else max(0.0, now - self._stationary_start_s)
        )
        ready = bool(instant_ready and dwell >= self.required_dwell_s)
        if not ring_fresh:
            reason = "ring_not_fresh"
        elif not all(vehicle_ready.values()):
            reason = "vehicle_evidence_not_ready"
        elif not ready:
            reason = "stationary_dwell"
        else:
            reason = "ready"
        return ReadinessSnapshot(ready, vehicle_ready, vehicle_reasons, dwell, reason)

    def _fresh(self, now: float, stamp: float) -> bool:
        age = now - float(stamp)
        return -1e-6 <= age <= self.timeout_s

    def _status_fresh(self, now: float, stamp: float) -> bool:
        age = now - float(stamp)
        return -1e-6 <= age <= self.status_timeout_s
