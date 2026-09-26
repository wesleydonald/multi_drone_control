"""Regression contracts for M2B B1 inside the shared mission framework.

The first B1 runtime exposed a framework mismatch: B1 generated its own local
reference, but the generic status path still tried to build a legacy
``rate_profile`` for M2_ATTACH_APPROACH_HIGH and killed the planner.  These
contracts keep reference generation and status reporting on the same phase
classification.
"""

from pathlib import Path

from tejen_mission.m2b_attachment_mission import (
    M2B_B1_LOCAL_REFERENCE_PHASES,
    is_m2b_b1_local_reference_phase,
)
from tejen_mission.mission_types import MissionPhase


ROOT = Path(__file__).resolve().parents[3]
PLANNER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "online_join_planner.py"


def _method_source(name: str) -> str:
    text = PLANNER.read_text(encoding="utf-8")
    marker = f"    def {name}("
    start = text.index(marker)
    next_method = text.find("\n    def ", start + len(marker))
    if next_method < 0:
        next_method = len(text)
    return text[start:next_method]


def test_all_b1_custom_reference_phases_are_classified_in_one_shared_set():
    expected = {
        MissionPhase.M2_SETTLE_CAPTURE,
        MissionPhase.M2_ATTACH_APPROACH_HIGH,
        MissionPhase.M2_ATTACH_APPROACH_LOW,
        MissionPhase.M2_ATTACH_CAPTURE_WAIT,
        MissionPhase.M2_ATTACH_PROOF,
        MissionPhase.M2_ATTACHED_HOLD,
        MissionPhase.M2_DETACHED_RETREAT,
        MissionPhase.M2_LANDING_STAGE,
        MissionPhase.M2_FAULT,
    }
    assert set(M2B_B1_LOCAL_REFERENCE_PHASES) == expected
    for phase in expected:
        assert is_m2b_b1_local_reference_phase(phase)


def test_legacy_takeoff_and_landing_are_not_misclassified_as_b1_local_references():
    for phase in (
        MissionPhase.M2_BOOTSTRAP_DETACH,
        MissionPhase.M2_BOOTSTRAP_SETTLE,
        MissionPhase.M2_WAIT_FOR_ARM,
        MissionPhase.M2_VERTICAL_TAKEOFF,
        MissionPhase.M2_TAKEOFF_HOVER,
        MissionPhase.LANDING,
        MissionPhase.LANDED_DISARMED,
    ):
        assert not is_m2b_b1_local_reference_phase(phase)


def test_reference_builder_and_status_builder_use_the_same_b1_phase_classifier():
    build_reference = _method_source("build_reference")
    build_status = _method_source("build_status_text")
    assert "is_m2b_b1_local_reference_phase(self.phase)" in build_reference
    assert "is_m2b_b1_local_reference_phase(self.phase)" in build_status


def test_b1_status_path_does_not_request_legacy_rate_profile_for_local_reference_phases():
    build_status = _method_source("build_status_text")
    classifier = build_status.index("is_m2b_b1_local_reference_phase(self.phase)")
    legacy_rate = build_status.index("self.rate_profile()")
    # Local B1 phases must be intercepted before generic RATE_CONTROLLED status.
    assert classifier < legacy_rate
    assert "M2B local reference" in build_status
