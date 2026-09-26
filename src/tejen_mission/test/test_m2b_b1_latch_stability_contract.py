"""M2B B1 post-latch physics stability contracts.

These tests encode the Sep-10 14:53 runtime failure: the approach and physical
capture succeeded, then the rigid Gazebo weld produced an immediate violent
transient before the proof command could meaningfully execute.  The simulator
fix must (1) preserve the partner-proven joint damping while preventing hard
magnet/ring contact and (2) hold the pre-proof reference until measured
post-latch stability is demonstrated.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from tejen_mission.m2b_attachment_mission import (
    LatchStabilityStatus,
    M2BConfig,
    M2BLatchStabilityGate,
)
from tejen_mission.m2b_ground_spawn import generate_ground_start_assets


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
PLANNER = PKG / "tejen_mission" / "online_join_planner.py"
MISSION_TYPES = PKG / "tejen_mission" / "mission_types.py"
MISSION_DEFINITIONS = PKG / "tejen_mission" / "mission_definitions.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"


def _joint_damping(model: ET.Element, joint_name: str) -> tuple[float, float]:
    joint = model.find(f"./joint[@name='{joint_name}']")
    assert joint is not None
    axis = joint.find("./axis/dynamics/damping")
    axis2 = joint.find("./axis2/dynamics/damping")
    assert axis is not None and axis2 is not None
    return float(axis.text), float(axis2.text)


def test_b1_generated_x3_keeps_magnet_collision_for_floor_support_and_partner_damping(tmp_path):
    before = X3.read_text(encoding="utf-8")
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
        attachment_joint_damping=0.1,
    )
    assert X3.read_text(encoding="utf-8") == before

    model = ET.parse(result.x3_path).getroot().find("model")
    assert model is not None
    magnet = model.find("./link[@name='magnet_tip_link']")
    assert magnet is not None
    collision = magnet.find("./collision[@name='magnet_tip_collision']")
    assert collision is not None
    assert collision.findtext('./surface/contact/collide_bitmask') == '0x0001'
    assert magnet.find("./visual[@name='magnet_tip_visual']") is not None

    assert _joint_damping(model, "base_to_tether") == (0.1, 0.1)
    assert _joint_damping(model, "tether_to_magnet_tip") == (0.1, 0.1)

    # Topology-only discriminator: for B1 generated assets the static ring owns
    # the DetachableJoint and the dynamic X3 is the child.  The repository X3
    # source remains unchanged; only the run-local generated assets are altered.
    assert model.find("./plugin[@name='gz::sim::systems::DetachableJoint']") is None

    assert result.ring_path is not None
    ring_model = ET.parse(result.ring_path).getroot().find("model")
    assert ring_model is not None
    assert ring_model.findtext("static") == "true"
    detachable = ring_model.find("./plugin[@name='gz::sim::systems::DetachableJoint']")
    assert detachable is not None
    assert detachable.findtext("parent_link") == "payload_link"
    assert detachable.findtext("child_model") == "x3"
    assert detachable.findtext("child_link") == "magnet_tip_link"
    assert detachable.findtext("detach_topic") == "/payload/detach"
    assert detachable.findtext("attach_topic") == "/payload/attach"
    assert detachable.findtext("output_topic") == "/payload/detachable_joint_state"

    manifest = __import__("json").loads(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["detachable_joint_topology"] == "static_ring_parent_dynamic_x3_child"


def test_latch_stability_gate_rejects_recorded_sep10_runaway_before_proof():
    cfg = M2BConfig()
    gate = M2BLatchStabilityGate(
        cfg,
        start_time_s=0.0,
        start_position_world=np.array([0.248051, 0.000191, 0.559244]),
        start_velocity_world=np.array([0.000444, -0.001064, 0.172628]),
    )

    # First post-latch sample is already unsettled but below the hard speed abort.
    first = gate.step(
        0.05,
        position_world=np.array([0.248355, 0.000003, 0.593611]),
        velocity_world=np.array([0.027445, -0.004100, 1.012190]),
    )
    assert first.status == LatchStabilityStatus.FAILED
    assert first.ready_for_proof is False
    assert "speed" in first.reason or "acceleration" in first.reason or "displacement" in first.reason


def test_latch_stability_gate_requires_continuous_measured_dwell():
    cfg = M2BConfig(
        latch_settle_dwell_s=0.50,
        latch_settle_timeout_s=1.50,
    )
    p0 = np.array([0.25, 0.0, 0.56])
    gate = M2BLatchStabilityGate(
        cfg,
        start_time_s=0.0,
        start_position_world=p0,
        start_velocity_world=np.zeros(3),
    )

    decision = None
    for k in range(1, 6):
        t = 0.1 * k
        decision = gate.step(
            t,
            position_world=p0 + np.array([0.001 * k, 0.0, 0.0]),
            velocity_world=np.array([0.01, 0.0, 0.0]),
        )
        assert decision.status == LatchStabilityStatus.SETTLING
        assert decision.ready_for_proof is False

    decision = gate.step(
        0.61,
        position_world=p0 + np.array([0.006, 0.0, 0.0]),
        velocity_world=np.array([0.01, 0.0, 0.0]),
    )
    assert decision.status == LatchStabilityStatus.STABLE
    assert decision.ready_for_proof is True


def test_latch_stability_gate_resets_dwell_on_moderate_transient_and_times_out():
    cfg = M2BConfig(
        latch_settle_dwell_s=0.40,
        latch_settle_timeout_s=0.90,
        latch_settle_speed_mps=0.08,
        latch_abort_speed_mps=0.50,
    )
    p0 = np.array([0.25, 0.0, 0.56])
    gate = M2BLatchStabilityGate(
        cfg,
        start_time_s=0.0,
        start_position_world=p0,
        start_velocity_world=np.zeros(3),
    )
    assert gate.step(0.10, position_world=p0, velocity_world=np.array([0.01, 0, 0])).status == LatchStabilityStatus.SETTLING
    # Below hard abort, but above settle threshold: dwell must reset.
    mid = gate.step(0.30, position_world=p0 + [0.01, 0, 0], velocity_world=np.array([0.12, 0, 0]))
    assert mid.status == LatchStabilityStatus.SETTLING
    assert mid.ready_for_proof is False
    timed = gate.step(0.95, position_world=p0 + [0.015, 0, 0], velocity_world=np.array([0.10, 0, 0]))
    assert timed.status == LatchStabilityStatus.FAILED
    assert "timeout" in timed.reason


def test_capture_wait_has_measured_post_latch_substage_before_proof():
    planner = PLANNER.read_text(encoding="utf-8")
    start = planner.index("if self.phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT:")
    end = planner.index("if self.phase == MissionPhase.M2_ATTACH_PROOF:", start)
    block = planner[start:end]
    assert "M2BLatchStabilityGate" in planner
    assert "m2b_latch_stability_gate" in block
    assert "ready_for_proof" in block
    assert "m2b_b1_proof_enabled" in block
    assert "MissionPhase.M2_ATTACH_PROOF" in block


def test_post_latch_reference_is_frozen_and_proof_is_not_requested_early():
    planner = PLANNER.read_text(encoding="utf-8")
    start = planner.index("if self.phase == MissionPhase.M2_ATTACH_CAPTURE_WAIT:", planner.index("def build_m2b_b1_reference"))
    end = planner.index("if self.phase == MissionPhase.M2_ATTACH_PROOF:", start)
    block = planner[start:end]
    assert "m2b_latch_hold_position" in block
    assert "stationary_regulation" in block

    assert "proof.data = self.phase == MissionPhase.M2_ATTACH_PROOF" in planner
    assert "m2b_latch_stable_publisher" in planner


def test_b1_runner_has_latch_only_commissioning_mode_and_uses_stabilized_generated_x3():
    text = RUNNER.read_text(encoding="utf-8")
    assert "capture|latch|success" in text
    assert "simulation-attachment-stabilization" in text
    assert "proof_enabled" in text
    assert "latch_stable" in text
    assert "LATCH_PASS" in text
    assert "M2_ATTACH_PROOF" in text


def test_latch_stability_gate_fails_closed_on_non_monotonic_time():
    cfg = M2BConfig()
    p0 = np.array([0.25, 0.0, 0.56])
    gate = M2BLatchStabilityGate(
        cfg,
        start_time_s=1.0,
        start_position_world=p0,
        start_velocity_world=np.zeros(3),
    )
    decision = gate.step(
        0.9,
        position_world=p0,
        velocity_world=np.zeros(3),
    )
    assert decision.status == LatchStabilityStatus.FAILED
    assert decision.ready_for_proof is False
    assert "time" in decision.reason.lower()


def test_b1_simulation_stabilization_preserves_x3_structure_except_deliberately_moved_detachable_plugin(tmp_path):
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
        attachment_joint_damping=0.1,
    )
    source_model = ET.parse(X3).getroot().find("model")
    generated_model = ET.parse(result.x3_path).getroot().find("model")
    assert source_model is not None and generated_model is not None

    for tag in ("link", "joint"):
        source_names = [node.attrib.get("name") for node in source_model.findall(tag)]
        generated_names = [node.attrib.get("name") for node in generated_model.findall(tag)]
        assert generated_names == source_names

    detachable_name = "gz::sim::systems::DetachableJoint"
    source_plugins = [node.attrib.get("name") for node in source_model.findall("plugin")]
    generated_plugins = [node.attrib.get("name") for node in generated_model.findall("plugin")]
    assert detachable_name in source_plugins
    assert generated_plugins == [name for name in source_plugins if name != detachable_name]

    assert result.ring_path is not None
    ring_model = ET.parse(result.ring_path).getroot().find("model")
    assert ring_model is not None
    moved_plugins = [node.attrib.get("name") for node in ring_model.findall("plugin")]
    assert moved_plugins.count(detachable_name) == 1


def test_latch_only_runner_observes_stability_before_reporting_pass():
    text = RUNNER.read_text(encoding="utf-8")
    assert "LATCH_STABILITY_OBSERVE_SECONDS" in text
    latch_block = text[text.index('if [[ "$MODE" == "latch" ]]'):text.index('echo "Full M2B $COMMISSIONING_STAGE enabled.', text.index('if [[ "$MODE" == "latch" ]]'))]
    assert "runtime_health_check" in latch_block
    assert "joint_detached false" in latch_block
    assert "latch_stable true" in latch_block
    assert "LATCH_PASS" in latch_block


def test_latch_only_commissioning_uses_single_attempt_while_success_keeps_retry_budget():
    runner = RUNNER.read_text(encoding="utf-8")
    launch = (PKG / "launch" / "m2b_b1_single_attachment.launch.py").read_text(encoding="utf-8")
    assert 'MAX_ATTEMPTS=3' in runner
    assert '[[ "$MODE" == "latch" ]] && MAX_ATTEMPTS=1' in runner
    assert 'max_attempts:="$MAX_ATTEMPTS"' in runner
    assert 'LaunchConfiguration("max_attempts")' in launch
    assert 'DeclareLaunchArgument("max_attempts", default_value="3")' in launch
    assert '"m2b_max_attempts": ParameterValue(max_attempts, value_type=int)' in launch
