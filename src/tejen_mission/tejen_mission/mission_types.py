"""Mission and reference abstractions for the per-drone planner.

This module intentionally contains no ROS dependencies.  The same phase and
reference definitions can therefore be reused by simulation tests, a future
multi-drone coordinator, or another controller implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, Mapping


class MissionPhase(str, Enum):
    WAIT_FOR_TAKEOFF = "WAIT_FOR_TAKEOFF"
    TAKEOFF = "TAKEOFF"
    APPROACH_ABOVE_PICKUP = "APPROACH_ABOVE_PICKUP"
    SETTLE_ABOVE_PICKUP = "SETTLE_ABOVE_PICKUP"
    DESCEND_TO_PICKUP = "DESCEND_TO_PICKUP"
    MAGNET_ATTACH_WAIT = "MAGNET_ATTACH_WAIT"
    LIFT_OBJECT = "LIFT_OBJECT"
    TRANSIT_TO_DROP_POINT = "TRANSIT_TO_DROP_POINT"
    SETTLE_ABOVE_DROP_POINT = "SETTLE_ABOVE_DROP_POINT"
    DESCEND_TO_DROP_HEIGHT = "DESCEND_TO_DROP_HEIGHT"
    DROP_OBJECT = "DROP_OBJECT"
    CLEAR_DROP_ZONE = "CLEAR_DROP_ZONE"
    TRANSIT_TO_REATTACH = "TRANSIT_TO_REATTACH"
    APPROACH_ABOVE_TARGET = "APPROACH_ABOVE_TARGET"
    MATCH_VELOCITY = "MATCH_VELOCITY"
    DESCEND_TO_ATTACHMENT = "DESCEND_TO_ATTACHMENT"
    ATTACH_READY = "ATTACH_READY"
    LANDING = "LANDING"
    LANDED_DISARMED = "LANDED_DISARMED"

    # M2B single-drone attachment phases.  These live in the shared phase enum so
    # mission logging/diagnostics and future mission composition use one framework.
    M2_BOOTSTRAP_DETACH = "M2_BOOTSTRAP_DETACH"
    M2_BOOTSTRAP_SETTLE = "M2_BOOTSTRAP_SETTLE"
    M2_WAIT_FOR_ARM = "M2_WAIT_FOR_ARM"
    M2_VERTICAL_TAKEOFF = "M2_VERTICAL_TAKEOFF"
    M2_TAKEOFF_HOVER = "M2_TAKEOFF_HOVER"
    M2_CPP_TRANSIT_CAPTURE = "M2_CPP_TRANSIT_CAPTURE"
    M2_SETTLE_CAPTURE = "M2_SETTLE_CAPTURE"
    M2_ATTACH_APPROACH_HIGH = "M2_ATTACH_APPROACH_HIGH"
    M2_ATTACH_APPROACH_LOW = "M2_ATTACH_APPROACH_LOW"
    M2_ATTACH_CAPTURE_WAIT = "M2_ATTACH_CAPTURE_WAIT"
    M2_ATTACH_PROOF = "M2_ATTACH_PROOF"
    M2_ATTACHED_HOLD = "M2_ATTACHED_HOLD"
    M2_DETACHED_RETREAT = "M2_DETACHED_RETREAT"
    M2_LANDING_STAGE = "M2_LANDING_STAGE"
    M2_FAULT = "M2_FAULT"


class ReferenceType(str, Enum):
    TRANSFER = "TRANSFER_TRAJECTORY"
    STATIONARY_REGULATION = "STATIONARY_REGULATION"
    TARGET_RELATIVE_TRACKING = "TARGET_RELATIVE_TRACKING"
    RATE_CONTROLLED_MANOEUVRE = "RATE_CONTROLLED_MANOEUVRE"


class TargetSource(str, Enum):
    CURRENT_POSE = "CURRENT_POSE"
    PICKUP_OBJECT = "PICKUP_OBJECT"
    DROP_POINT = "DROP_POINT"
    ATTACHMENT_POINT = "ATTACHMENT_POINT"
    LANDING_POINT = "LANDING_POINT"


class ControllerAuthority(str, Enum):
    """Which controller is expected to own the vehicle in a phase.

    Only LOCAL_NMPC is used by the current thesis implementation.  The enum is
    deliberately present now so a future swarm-controller handoff can be added
    without changing the mission/reference interfaces.
    """

    DISARMED = "DISARMED"
    LOCAL_NMPC = "LOCAL_NMPC"
    EXTERNAL_CONTROLLER = "EXTERNAL_CONTROLLER"


@dataclass(frozen=True)
class PhaseSpec:
    phase: MissionPhase
    reference_type: ReferenceType
    target_source: TargetSource
    magnet_command: str
    controller_authority: ControllerAuthority = ControllerAuthority.LOCAL_NMPC
    handoff_ready: bool = False
    description: str = ""

    def __post_init__(self) -> None:
        command = self.magnet_command.strip().upper()
        if command not in {"ON", "OFF"}:
            raise ValueError(f"Unsupported magnet command: {self.magnet_command!r}")
        object.__setattr__(self, "magnet_command", command)


@dataclass(frozen=True)
class MissionDefinition:
    name: str
    initial_phase: MissionPhase
    phases: Mapping[MissionPhase, PhaseSpec]

    def __post_init__(self) -> None:
        phases: Dict[MissionPhase, PhaseSpec] = dict(self.phases)
        if self.initial_phase not in phases:
            raise ValueError(
                f"Initial phase {self.initial_phase.value} is absent from mission {self.name!r}."
            )
        for key, spec in phases.items():
            if key != spec.phase:
                raise ValueError(
                    f"Phase map key {key.value} does not match spec {spec.phase.value}."
                )
        object.__setattr__(self, "phases", phases)

    def spec(self, phase: MissionPhase) -> PhaseSpec:
        try:
            return self.phases[phase]
        except KeyError as exc:
            raise KeyError(
                f"Phase {phase.value} is not defined for mission {self.name!r}."
            ) from exc

    def validate_required_phases(self, required: Iterable[MissionPhase]) -> None:
        missing = [phase.value for phase in required if phase not in self.phases]
        if missing:
            raise ValueError(
                f"Mission {self.name!r} is missing required phases: {', '.join(missing)}"
            )
