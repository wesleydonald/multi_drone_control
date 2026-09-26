"""ROS-independent M2D sequential fleet supervision policy.

M2D deliberately owns only fleet progression. Per-vehicle trajectory generation,
attachment retries/proof, controller authority and physical attachment actuation remain
in the already commissioned M2B/M2C components.

The optional operator-hold mode is the bridge used for IRL-like commissioning: after a
configured number of software-proven attachments the fleet remains in a monitored
success hold until the operator explicitly requests landing. Landing is then sequenced
in reverse attachment order while the per-vehicle planner owns release, clear retreat,
vertical descent and disarm.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional


DEFAULT_VEHICLE_IDS = ("drone_0", "drone_1", "drone_2", "drone_3")
ATTACHED_PHASE = "M2_ATTACHED_HOLD"
LANDING_PHASES = frozenset({"M2_LANDING_STAGE", "LANDING", "LANDED_DISARMED"})
TERMINAL_FAILURE_PHASES = frozenset({"M2_FAULT", "LANDING", "LANDED_DISARMED"})


@dataclass(frozen=True)
class AttachmentEvidence:
    confirmed: bool = False
    lost: bool = False
    stamp_s: Optional[float] = None


@dataclass(frozen=True)
class M2DSupervisorSnapshot:
    state: str
    active_vehicle_id: Optional[str]
    permissions: Mapping[str, bool]
    assigned_plate_ids: Mapping[str, int]
    attached_plate_ids: Mapping[str, int]
    abort_reason: str = ""
    landing_vehicle_id: Optional[str] = None
    landed_vehicle_ids: tuple[str, ...] = ()

    @property
    def passed(self) -> bool:
        return self.state == "M2D_PASS"

    @property
    def aborted(self) -> bool:
        return self.state == "M2D_ABORT"

    @property
    def success_hold(self) -> bool:
        return self.state == "M2D_SUCCESS_HOLD"

    @property
    def landed(self) -> bool:
        return self.state == "M2D_LANDED"


class M2DSequentialSupervisorCore:
    """Deterministic sequential attachment and optional operator landing policy.

    The M2C assignment is frozen exactly once. M2C readiness is latched when it first
    becomes true because the ground-only readiness predicate is expected to become
    false after the first vehicle takes off.

    With ``operator_hold_after_goal=False`` the historical M2D behavior is preserved:
    all configured target attachments lead directly to ``M2D_PASS``. With operator
    hold enabled, the same target leads to ``M2D_SUCCESS_HOLD``. An explicit landing
    request then lands the proven-attached vehicles in reverse attachment order.
    """

    def __init__(
        self,
        *,
        vehicle_ids=DEFAULT_VEHICLE_IDS,
        evidence_timeout_s: float = 0.75,
        target_attachment_count: Optional[int] = None,
        operator_hold_after_goal: bool = False,
        simultaneous: bool = False,
    ) -> None:
        # simultaneous (multi_drone_control, Wesley 2026-09-26, for testing): every
        # unattached vehicle holds mission permission at once instead of one at a time
        self.simultaneous = bool(simultaneous)
        self.vehicle_ids = tuple(str(v) for v in vehicle_ids)
        if len(self.vehicle_ids) == 0 or len(set(self.vehicle_ids)) != len(self.vehicle_ids):
            raise ValueError("vehicle_ids must be non-empty and unique")
        self.evidence_timeout_s = float(evidence_timeout_s)
        if not self.evidence_timeout_s > 0.0:
            raise ValueError("evidence_timeout_s must be positive")

        if target_attachment_count is None:
            target_attachment_count = len(self.vehicle_ids)
        self.target_attachment_count = int(target_attachment_count)
        if not 1 <= self.target_attachment_count <= len(self.vehicle_ids):
            raise ValueError("target_attachment_count must lie in [1, vehicle_count]")
        self.operator_hold_after_goal = bool(operator_hold_after_goal)

        self.assignment: Optional[dict[str, int]] = None
        self.m2c_ready_latched = False
        self.active_index: Optional[int] = None
        self.phases = {vehicle_id: "" for vehicle_id in self.vehicle_ids}
        self.evidence = {vehicle_id: AttachmentEvidence() for vehicle_id in self.vehicle_ids}
        self.attached = set()
        self.landed = set()
        self.abort_reason = ""

        self.landing_requested = False
        self.landing_order: tuple[str, ...] = ()
        self.landing_index: Optional[int] = None

    def freeze_assignment(self, assignment: Mapping[str, int]) -> None:
        normalized = {str(k): int(v) for k, v in dict(assignment).items()}
        if set(normalized) != set(self.vehicle_ids):
            raise ValueError("M2D assignment must contain exactly the configured vehicles")
        if len(set(normalized.values())) != len(normalized):
            raise ValueError("M2D assignment plate IDs must be unique")
        if any(plate < 0 or plate >= 12 for plate in normalized.values()):
            raise ValueError("M2D assignment plate IDs must lie in [0, 11]")
        if self.assignment is None:
            self.assignment = normalized
            return
        if normalized != self.assignment:
            raise RuntimeError("M2D assignment is already frozen and cannot be reassigned")

    def update_m2c_ready(self, ready: bool) -> None:
        if bool(ready) and self.assignment is not None:
            self.m2c_ready_latched = True
            if self.active_index is None and not self.abort_reason and not self.attached:
                self.active_index = 0

    def update_phase(self, vehicle_id: str, phase: str) -> None:
        self._require_vehicle(vehicle_id)
        self.phases[str(vehicle_id)] = str(phase)

    def update_attachment_evidence(
        self,
        vehicle_id: str,
        *,
        confirmed: bool,
        lost: bool,
        stamp_s: float,
    ) -> None:
        self._require_vehicle(vehicle_id)
        self.evidence[str(vehicle_id)] = AttachmentEvidence(
            confirmed=bool(confirmed),
            lost=bool(lost),
            stamp_s=float(stamp_s),
        )

    def request_landing(self) -> bool:
        """Accept an operator landing request only from the monitored success hold."""
        if not self.operator_hold_after_goal or self.abort_reason or self.landing_requested:
            return False
        if self.snapshot().state != "M2D_SUCCESS_HOLD":
            return False

        ordered_attached = [vehicle_id for vehicle_id in self.vehicle_ids if vehicle_id in self.attached]
        if len(ordered_attached) != self.target_attachment_count:
            return False

        self.landing_order = tuple(reversed(ordered_attached))
        self.landing_index = 0
        self.landing_requested = True
        return True

    def current_landing_vehicle_id(self) -> Optional[str]:
        if not self.landing_requested or self.landing_index is None:
            return None
        if not 0 <= self.landing_index < len(self.landing_order):
            return None
        return self.landing_order[self.landing_index]

    def _require_vehicle(self, vehicle_id: str) -> None:
        if str(vehicle_id) not in self.phases:
            raise KeyError(f"unknown M2D vehicle {vehicle_id!r}")

    def _evidence_fresh(self, vehicle_id: str, now_s: float) -> bool:
        evidence = self.evidence[vehicle_id]
        if evidence.stamp_s is None:
            return False
        age = float(now_s) - float(evidence.stamp_s)
        return 0.0 <= age <= self.evidence_timeout_s

    def _attachment_healthy(self, vehicle_id: str, now_s: float) -> bool:
        evidence = self.evidence[vehicle_id]
        return bool(
            self.phases[vehicle_id] == ATTACHED_PHASE
            and self._evidence_fresh(vehicle_id, now_s)
            and evidence.confirmed
            and not evidence.lost
        )

    def _abort(self, reason: str) -> None:
        if not self.abort_reason:
            self.abort_reason = str(reason)

    def _step_landing(self, now_s: float) -> None:
        landing_vehicle = self.current_landing_vehicle_id()
        if landing_vehicle is None:
            return

        # Every still-attached peer except the vehicle intentionally being released
        # must preserve the same proven-attachment invariant as normal M2D flight.
        for vehicle_id in tuple(self.attached):
            if vehicle_id == landing_vehicle:
                continue
            if not self._attachment_healthy(vehicle_id, now_s):
                self._abort(f"PROVEN_ATTACHMENT_LOST:{vehicle_id}")
                return

        phase = self.phases[landing_vehicle]
        if phase == "M2_FAULT":
            self._abort(f"LANDING_VEHICLE_TERMINAL_FAILURE:{landing_vehicle}:{phase}")
            return

        if phase == "LANDED_DISARMED":
            self.attached.discard(landing_vehicle)
            self.landed.add(landing_vehicle)
            assert self.landing_index is not None
            self.landing_index += 1
            if self.landing_index >= len(self.landing_order):
                self.landing_index = None
            return

        # Before the planner acknowledges the release/landing request by leaving
        # ATTACHED_HOLD, the attachment must remain valid. Once it enters the
        # intentional landing phases, evidence is allowed to disappear by design.
        if phase == ATTACHED_PHASE and not self._attachment_healthy(landing_vehicle, now_s):
            self._abort(f"PROVEN_ATTACHMENT_LOST:{landing_vehicle}")
            return
        if phase not in {ATTACHED_PHASE, *LANDING_PHASES}:
            self._abort(f"UNEXPECTED_LANDING_PHASE:{landing_vehicle}:{phase}")

    def step(self, now_s: float) -> M2DSupervisorSnapshot:
        now_s = float(now_s)
        if self.abort_reason:
            return self.snapshot()

        if self.landing_requested:
            self._step_landing(now_s)
            return self.snapshot()

        # A previously proven attachment becoming stale/lost is a fleet-level
        # fault. Gazebo joint truth is deliberately absent from this policy.
        for vehicle_id in tuple(self.attached):
            if not self._attachment_healthy(vehicle_id, now_s):
                self._abort(f"PROVEN_ATTACHMENT_LOST:{vehicle_id}")
                return self.snapshot()

        if self.simultaneous:
            if not self.m2c_ready_latched or self.assignment is None:
                return self.snapshot()
            for vehicle_id in self.vehicle_ids:
                if vehicle_id in self.attached:
                    continue
                phase = self.phases[vehicle_id]
                if phase in TERMINAL_FAILURE_PHASES:
                    self._abort(f"ACTIVE_VEHICLE_TERMINAL_FAILURE:{vehicle_id}:{phase}")
                    return self.snapshot()
                evidence = self.evidence[vehicle_id]
                if (phase == ATTACHED_PHASE and self._evidence_fresh(vehicle_id, now_s)
                        and evidence.confirmed and not evidence.lost):
                    self.attached.add(vehicle_id)
            if len(self.attached) >= self.target_attachment_count:
                self.active_index = None
            return self.snapshot()

        if not self.m2c_ready_latched or self.assignment is None or self.active_index is None:
            return self.snapshot()

        active = self.vehicle_ids[self.active_index]
        phase = self.phases[active]
        evidence = self.evidence[active]

        if phase in TERMINAL_FAILURE_PHASES and active not in self.attached:
            self._abort(f"ACTIVE_VEHICLE_TERMINAL_FAILURE:{active}:{phase}")
            return self.snapshot()

        attachment_complete = bool(
            phase == ATTACHED_PHASE
            and self._evidence_fresh(active, now_s)
            and evidence.confirmed
            and not evidence.lost
        )
        if attachment_complete:
            self.attached.add(active)
            if len(self.attached) >= self.target_attachment_count:
                self.active_index = None
            elif self.active_index + 1 >= len(self.vehicle_ids):
                self.active_index = None
            else:
                self.active_index += 1

        return self.snapshot()

    def snapshot(self) -> M2DSupervisorSnapshot:
        landing_vehicle = self.current_landing_vehicle_id()
        if self.abort_reason:
            state = "M2D_ABORT"
            active = None
        elif self.assignment is None:
            state = "WAIT_ASSIGNMENT"
            active = None
        elif not self.m2c_ready_latched:
            state = "WAIT_M2C_READY"
            active = None
        elif self.landing_requested:
            if landing_vehicle is None:
                state = "M2D_LANDED"
                active = None
            else:
                state = "M2D_LANDING"
                active = landing_vehicle
        elif len(self.attached) >= self.target_attachment_count:
            state = "M2D_SUCCESS_HOLD" if self.operator_hold_after_goal else "M2D_PASS"
            active = None
        elif self.active_index is None:
            state = "READY"
            active = None
        else:
            active = self.vehicle_ids[self.active_index]
            state = f"ACTIVE_{self.active_index}"

        permissions = {vehicle_id: False for vehicle_id in self.vehicle_ids}
        if active is not None and not self.abort_reason and state.startswith("ACTIVE_"):
            permissions[active] = True
        if (self.simultaneous and state.startswith("ACTIVE_") and not self.abort_reason):
            for vehicle_id in self.vehicle_ids:
                permissions[vehicle_id] = vehicle_id not in self.attached

        assigned = {} if self.assignment is None else dict(self.assignment)
        attached_plate_ids = {
            vehicle_id: assigned[vehicle_id]
            for vehicle_id in self.vehicle_ids
            if vehicle_id in self.attached and vehicle_id in assigned
        }
        return M2DSupervisorSnapshot(
            state=state,
            active_vehicle_id=active,
            permissions=permissions,
            assigned_plate_ids=assigned,
            attached_plate_ids=attached_plate_ids,
            abort_reason=self.abort_reason,
            landing_vehicle_id=landing_vehicle,
            landed_vehicle_ids=tuple(
                vehicle_id for vehicle_id in self.vehicle_ids if vehicle_id in self.landed
            ),
        )
