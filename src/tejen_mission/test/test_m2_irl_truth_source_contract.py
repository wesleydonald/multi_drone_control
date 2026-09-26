"""Static integration guard for default-preserving M2 attachment authority."""

import importlib
from pathlib import Path


def test_planner_has_explicit_default_joint_and_irl_measured_truth_sources():
    path = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "online_join_planner.py"
    )
    source = path.read_text(encoding="utf-8")
    assert 'declare_parameter("m2b_attachment_truth_source", "joint_truth")' in source
    assert '"joint_truth", "measured_detector"' in source
    assert "def m2b_geometry_separated" in source
    assert "self.m2b_attachment_truth_source == \"joint_truth\"" in source
    assert "self.m2b_attachment_truth_source == \"measured_detector\"" in source
    assert "if self.m2b_attachment_truth_source == \"joint_truth\":\n                    self.m2b_request_detector_reset()" in source


def test_measured_detector_bootstrap_bypasses_simulator_joint_truth_gate():
    module = importlib.import_module("tejen_mission.online_join_planner")
    planner = object.__new__(module.OnlineJoinPlanner)
    planner.is_m2b = True
    planner.phase = module.MissionPhase.M2_BOOTSTRAP_DETACH
    planner.m2b_attachment_truth_source = "measured_detector"
    planner.m2b_bootstrap_arm_permitted = False
    planner.m2b_bootstrap_gate = object()
    transitions = []
    planner.transition_to = lambda phase, reason: transitions.append((phase, reason))

    planner.update_m2b_bootstrap()

    assert planner.m2b_bootstrap_arm_permitted
    assert transitions == [(module.MissionPhase.M2_WAIT_FOR_ARM,
                            "IRL measured-detector bootstrap: no Gazebo joint truth")]
