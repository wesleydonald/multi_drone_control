"""Static integration contracts for the M2B B1 local-attachment increment.

These intentionally avoid importing ROS/Gazebo so wiring and authority contracts
can be checked in lightweight CI before flight simulation.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"
PLANNER = PKG / "tejen_mission" / "online_join_planner.py"
OBSERVER = PKG / "tejen_mission" / "m2_attachment_observer.py"
MANAGER = PKG / "tejen_mission" / "m2a_physical_capture_manager.py"
LAUNCH = PKG / "launch" / "m2b_b1_single_attachment.launch.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"
TELEMETRY = PKG / "tejen_mission" / "m2b_b1_telemetry.py"
BRIDGE_CFG = PKG / "config" / "m2b_gz_joint_bridge.yaml"
SETUP = PKG / "setup.py"
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_physical_capture_manager_keeps_legacy_cli_default_but_supports_nonblocking_ros_bridge():
    text = _text(MANAGER)
    assert 'declare_parameter("command_backend", "gz_cli")' in text
    assert '"ros_bridge"' in text
    assert "create_publisher(Empty" in text
    assert "ros_attach_command_topic" in text
    assert "ros_detach_command_topic" in text
    assert "subprocess.run" in text  # legacy M2A backend is retained


def test_b1_ros_gz_bridge_has_explicit_one_way_empty_attach_and_detach_topics():
    text = _text(BRIDGE_CFG)
    assert "/m2b/gz/attach" in text
    assert "/m2b/gz/detach" in text
    assert "/payload/attach" in text
    assert "/payload/detach" in text
    assert text.count('ros_type_name: "std_msgs/msg/Empty"') == 2
    assert text.count('gz_type_name: "gz.msgs.Empty"') == 2
    assert text.count("direction: ROS_TO_GZ") == 2


def test_observer_exports_detector_diagnostics_and_reset_without_joint_truth_as_detector_authority():
    text = _text(OBSERVER)
    assert "/m2a/attachment/diagnostics" in text
    assert "/m2a/attachment/reset" in text
    assert "self.detector.reset()" in text
    assert '"candidate_ready"' in text
    assert '"proof_excitation_m"' in text
    assert '"confirmed"' in text
    # The diagnostic authority message deliberately does not include raw joint truth.
    start = text.index("def _publish_diagnostics")
    end = text.index("def ", start + 10)
    assert "joint_detached_truth" not in text[start:end]


def test_b1_planner_enables_stage_b1_and_consumes_measured_detector_and_pendulum_evidence():
    text = _text(PLANNER)
    assert 'self.m2b_commissioning_stage not in {"b0", "b1", "b2"}' in text
    assert "/m2a/attachment/diagnostics" in text
    assert "/pendulum_swing_state" in text
    assert "M2BSettleGate" in text
    assert "M2BDescentProgress" in text
    assert "M2BRetryTracker" in text
    assert "M2_ATTACH_APPROACH_HIGH" in text
    assert "M2_ATTACH_APPROACH_LOW" in text
    assert "M2_ATTACH_CAPTURE_WAIT" in text
    assert "M2_ATTACH_PROOF" in text
    assert "M2_ATTACHED_HOLD" in text
    assert "M2_DETACHED_RETREAT" in text


def test_b1_planner_has_separate_proof_and_detector_reset_controls_and_never_uses_magnet_object_attached_for_success():
    text = _text(PLANNER)
    assert "/m2a/attachment/proof_requested" in text
    assert "/m2a/attachment/reset" in text
    assert "m2b_attachment_diagnostics" in text
    b1_start = text.index("def update_m2b_b1")
    b1_end = text.index("\n    def ", b1_start + 10)
    b1 = text[b1_start:b1_end]
    assert "object_attached" not in b1
    assert 'get("confirmed"' in b1
    assert "m2b_joint_detached_truth" in b1


def test_b1_launch_reuses_real_ground_spawn_and_adds_detector_bridge_and_structured_logging():
    text = _text(LAUNCH)
    assert 'DeclareLaunchArgument("commissioning_stage", default_value="b1")' in text
    assert '"m2b_commissioning_stage": commissioning_stage' in text
    assert 'executable="m2_attachment_observer"' in text
    assert 'executable="m2b_b1_telemetry"' in text
    assert "m2b_gz_joint_bridge.yaml" in text
    assert '"command_backend": "ros_bridge"' in text
    assert "world_m2b_single_attachment.sdf" in text
    assert "m2a_x3_support.sdf" not in text


def test_b1_runner_uses_repo_log_tree_structured_csv_and_requires_confirmed_hold():
    text = _text(RUNNER)
    assert 'logs/m2_attachment/m2b_b1_' in text
    assert "b1_status.csv" in text
    assert "attachment.csv" in text
    assert "runtime.log" in text
    assert "start_topic_log" not in text
    assert "M2_ATTACHED_HOLD" in text
    assert "detector_confirmed" in text
    assert "joint_detached" in text
    assert "HOLD_SECONDS" in text
    assert "b1_result.txt" in text


def test_b1_telemetry_is_csv_and_contains_attachment_authority_signals():
    text = _text(TELEMETRY)
    assert '"b1_status.csv"' in text
    assert "csv.DictWriter" in text
    for field in (
        '"mission_phase"',
        '"attempt_number"',
        '"magnet_command"',
        '"proof_requested"',
        '"joint_detached"',
        '"detector_state"',
        '"detector_fresh"',
        '"detector_candidate_ready"',
        '"proof_excitation_m"',
        '"detector_confirmed"',
        '"drone_x"',
        '"pendulum_phi"',
        '"pendulum_theta"',
    ):
        assert field in text


def test_setup_installs_b1_runtime_nodes():
    text = _text(SETUP)
    assert "m2_attachment_observer = tejen_mission.m2_attachment_observer:main" in text
    assert "m2b_b1_telemetry = tejen_mission.m2b_b1_telemetry:main" in text


def test_b1_default_anchor_and_contact_length_match_current_real_x3_geometry():
    # Guard the commissioning numbers against silent model drift.  The base-to-
    # tether joint anchor is tether link pose z + joint pose z = -0.04 m in body.
    model = ET.parse(X3).getroot().find("model")
    tether = model.find("./link[@name='tether_rod']")
    joint = model.find("./joint[@name='base_to_tether']")
    magnet = model.find("./link[@name='magnet_tip_link']")
    assert tether is not None and joint is not None and magnet is not None
    tether_pose_z = float(tether.find("pose").text.split()[2])
    joint_pose_z = float(joint.find("pose").text.split()[2])
    tether_length = float(tether.find("./visual/geometry/cylinder/length").text)
    magnet_radius = float(magnet.find("./collision/geometry/sphere/radius").text)
    assert abs((tether_pose_z + joint_pose_z) - (-0.04)) < 1e-12
    assert abs((tether_length - 0.05 + magnet_radius) - 0.475) < 1e-12


def test_b1_runner_refuses_flight_until_ros_to_gz_joint_command_bridge_has_subscribers():
    text = _text(RUNNER)
    assert "wait_for_topic_subscriber" in text
    assert "wait_for_topic_subscriber /m2b/gz/attach" in text
    assert "wait_for_topic_subscriber /m2b/gz/detach" in text
    assert "Subscription count:" in text
