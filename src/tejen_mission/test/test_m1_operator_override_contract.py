from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PLANNER = ROOT / "src/tejen_mission/tejen_mission/online_join_planner.py"
ARM_HEADER = ROOT / "src/drone_visualisation/include/drone_visualisation/arm_panel.hpp"
ARM_CPP = ROOT / "src/drone_visualisation/src/arm_panel.cpp"


def test_c1f2_loaded_lift_gate_is_entry_only_after_prepare_latches():
    source = PLANNER.read_text()
    block = source.split("if self.phase == MissionPhase.LIFT_OBJECT:", 1)[1].split(
        "if self.phase == MissionPhase.TRANSIT_TO_DROP_POINT:", 1
    )[0]
    c1f2 = block.split("if self.c1f2_cpp_authority_enabled:", 1)[1]
    before_prepare_wait, after_prepare_wait = c1f2.split(
        "if self.c1f2_prepare_start_time is not None:", 1
    )
    # The strict 5 cm gate certifies ENTRY only.  Once C1F2 has latched the
    # measured physical hold, continuation is governed by the existing p/v/a
    # settle contract rather than the old ideal lift endpoint.
    assert before_prepare_wait.index("if not self.c1f2_prepare_active:") < before_prepare_wait.index(
        "loaded_lift_ready_for_cpp("
    )
    assert "loaded_lift_ready_for_cpp(" not in after_prepare_wait


def test_operator_advance_callback_is_one_shot_and_never_transitions_directly():
    source = PLANNER.read_text()
    assert 'self.declare_parameter("enable_operator_advance_override", False)' in source
    assert '"operator_advance_override_topic", "/join_planner/advance_override"' in source
    assert "def operator_advance_override_callback" in source
    callback = source.split("def operator_advance_override_callback", 1)[1].split(
        "\n    def ", 1
    )[0]
    assert "operator_override_request_count += 1" in callback
    assert "operator_advance_override_pending = True" in callback
    assert "transition_to(" not in callback


def test_operator_advance_is_narrow_to_loaded_lift_and_keeps_cpp_handoff():
    source = PLANNER.read_text()
    lift = source.split("if self.phase == MissionPhase.LIFT_OBJECT:", 1)[1].split(
        "if self.phase == MissionPhase.TRANSIT_TO_DROP_POINT:", 1
    )[0]
    assert "loaded_lift_override_allowed(" in lift
    assert "WAIVE_LOADED_LIFT_Z_GATE" in lift
    assert "self.c1f2_prepare_active = True" in lift
    assert "if self.c1f2_handoff_ready:" in lift
    assert "MissionPhase.TRANSIT_TO_DROP_POINT" in lift

    # The button must not turn a failed/missing physical pickup into an override.
    assert "object_attached=self.object_attached" in lift
    assert "object_airborne=object_airborne" in lift
    assert "profile_complete=profile_complete" in lift


def test_operator_override_evidence_is_persistent_in_join_planner_csv():
    source = PLANNER.read_text()
    for field in (
        "operator_override_used",
        "operator_override_request_count",
        "operator_override_accept_count",
        "operator_override_last_from_phase",
        "operator_override_last_action",
        "operator_override_last_result",
    ):
        assert source.count(f'"{field}"') >= 2


def test_arm_panel_exposes_explicit_advance_override_button():
    header = ARM_HEADER.read_text()
    source = ARM_CPP.read_text()
    assert "advance_override_pub_" in header
    assert "advance_override_button_" in header
    assert "onAdvanceOverridePressed" in header
    assert 'new QPushButton("ADVANCE OVERRIDE")' in source
    assert '"/join_planner/advance_override"' in source
    assert "advance_override_msg.data = true" in source
    assert "advance_override_button_->setEnabled(true)" in source
    assert "advance_override_button_->setEnabled(false)" in source


def test_pending_operator_override_cannot_carry_across_a_phase_exit():
    source = PLANNER.read_text()
    transition = source.split("def transition_to(self, new_phase: MissionPhase, reason: str) -> None:", 1)[1].split(
        "def condition_true_for", 1
    )[0]
    assert "old_phase == MissionPhase.LIFT_OBJECT" in transition
    assert 'self.operator_override_last_result = \"REJECTED_PHASE_EXIT\"' in transition
    assert "self.operator_advance_override_pending = False" in transition
