"""Static integration contracts for the first M2B B0 commissioning increment.

These tests deliberately avoid ROS/Gazebo imports so the launch/world wiring can be
checked in CI and in lightweight development environments before runtime testing.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2b_b0_single_attachment.launch.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b0.sh"
MANAGER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2a_physical_capture_manager.py"
TRUTH_BRIDGE = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2a_gazebo_joint_truth_bridge.py"
SETUP = ROOT / "src" / "tejen_mission" / "setup.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_b0_world_uses_real_x3_and_never_the_abandoned_support_fixture():
    text = _text(WORLD)
    assert "modelLargeM2BallMagnet.sdf" in text
    assert "m2a_ring_fixture.sdf" in text
    assert "m2a_x3_support" not in text
    assert "kinematic" not in text.lower()
    assert text.count("gz-sim-multicopter-motor-model-system") == 4


def test_b0_launch_uses_shared_mission_planner_and_real_arm_interlock():
    text = _text(LAUNCH)
    assert '"mission_mode": "m2b"' in text
    assert '"m2b_commissioning_stage": "b0"' in text
    assert '"require_external_arm_permission": True' in text
    assert '"external_arm_permission_topic": "/join_planner/arm_permission"' in text
    assert '"payload_mpc_mode_topic": "/join_planner/mpc_mode"' in text
    assert "tejen_dynamic_planner" not in text
    assert "fake_cooperative_transport_world" not in text
    assert "m2a_x3_support" not in text


def test_sim_actuator_has_explicit_raw_bootstrap_detach_request():
    text = _text(MANAGER)
    assert "raw_detach_request_topic" in text
    assert "_raw_detach_request" in text
    assert "raw_detach_requested" in text


def test_joint_truth_bridge_republish_period_is_configurable():
    text = _text(TRUTH_BRIDGE)
    assert "republish_period_s" in text


def test_b0_runner_follows_existing_m2_log_tree_and_does_not_use_tmp():
    text = _text(RUNNER)
    assert 'logs/m2_attachment/m2b_b0_' in text
    assert "/tmp" not in text
    assert "set -u" not in text
    assert "/join_planner/arm_permission" in text
    assert "/join_planner/mpc_mode" in text
    assert "/m2a/sim/joint_detached_truth" in text


def test_m2_sim_helpers_are_installed_as_normal_console_scripts():
    text = _text(SETUP)
    assert "m2a_physical_capture_manager = tejen_mission.m2a_physical_capture_manager:main" in text
    assert "m2a_gazebo_joint_truth_bridge = tejen_mission.m2a_gazebo_joint_truth_bridge:main" in text
