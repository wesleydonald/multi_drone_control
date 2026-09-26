from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PLANNER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "online_join_planner.py"


def test_capture_settle_diagnostics_record_actual_gate_evidence_without_changing_reference_logic():
    text = PLANNER.read_text(encoding="utf-8")

    # The actual M2 settle reference remains the existing capture-target transfer.
    settle_ref = text.split("def build_m2b_b1_reference", 1)[1].split(
        "if self.phase in {MissionPhase.M2_ATTACH_APPROACH_HIGH", 1
    )[0]
    assert "if self.phase == MissionPhase.M2_SETTLE_CAPTURE:" in settle_ref
    assert "target_position = self.m2b_capture_body_target()" in settle_ref
    assert "self.transfer_generator.generate(" in settle_ref

    # The gate now records the evidence that actually blocks/admits descent.
    measured = text.split("def m2b_measured_settled", 1)[1].split(
        "def m2b_request_detector_reset", 1
    )[0]
    assert "if gate is self.m2b_capture_settle_gate:" in measured
    assert "self.m2b_capture_settle_position_error_m" in measured
    assert "self.m2b_capture_settle_speed_mps" in measured
    assert "self.m2b_capture_settle_swing_deg" in measured
    assert "self.m2b_capture_settle_reason" in measured

    for field in (
        "m2b_capture_settle_position_error_m",
        "m2b_capture_settle_speed_mps",
        "m2b_capture_settle_swing_deg",
        "m2b_capture_settle_dwell_s",
        "m2b_capture_settle_reason",
        "m2b_capture_settle_settled",
    ):
        assert f'"{field}"' in text
