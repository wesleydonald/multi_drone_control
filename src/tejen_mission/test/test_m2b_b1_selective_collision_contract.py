"""M2B B1 selective-collision and supervised-bootstrap contracts.

The Sep-10 latch-stability attempt showed that removing the magnet collision at
world generation breaks the grounded start: the detached magnet no longer has a
physical support.  B1 therefore uses static collision masks instead of runtime
collision mutation.  While the world is paused the startup DetachableJoint holds
the magnet at the assigned plate; only after measured geometry is verified may
the supervised runner release bootstrap detach.  The free magnet can then settle
on the real floor while ignoring hard contact with the ring / steel plates.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from tejen_mission.m2b_attachment_mission import (
    BootstrapStatus,
    M2BConfig,
    M2BBootstrapGate,
)
from tejen_mission.m2b_ground_spawn import (
    generate_ground_start_assets,
    verify_post_detach_floor_settle,
)

ROOT = Path(__file__).resolve().parents[3]
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
RING = ROOT / "simulation_assets" / "tejen" / "m2a_ring_fixture.sdf"
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"
PLANNER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "online_join_planner.py"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2b_b1_single_attachment.launch.py"


def _mask(collision: ET.Element) -> str | None:
    return collision.findtext("./surface/contact/collide_bitmask")


def test_selective_masks_encode_floor_yes_ring_no_without_touching_sources(tmp_path):
    before = {p: p.read_bytes() for p in (X3, RING, WORLD)}
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
        attachment_joint_damping=0.1,
    )
    assert {p: p.read_bytes() for p in (X3, RING, WORLD)} == before

    x3_model = ET.parse(result.x3_path).getroot().find("model")
    assert x3_model is not None
    magnet_collision = x3_model.find("./link[@name='magnet_tip_link']/collision[@name='magnet_tip_collision']")
    assert magnet_collision is not None
    assert _mask(magnet_collision) == "0x0001"

    assert result.ring_path is not None
    ring_model = ET.parse(result.ring_path).getroot().find("model")
    assert ring_model is not None
    ring_collisions = ring_model.findall(".//collision")
    assert len(ring_collisions) > 10
    assert {_mask(c) for c in ring_collisions} == {"0x0002"}

    world = ET.parse(result.world_path).getroot()
    floor = world.find(".//model[@name='floor']/link/collision[@name='col']")
    assert floor is not None
    assert _mask(floor) == "0x0003"

    # Fortress collision filtering is pairwise bitwise AND.
    magnet = 0x0001
    ring = 0x0002
    floor_mask = 0x0003
    ordinary_x3 = 0xFFFF
    assert magnet & floor_mask
    assert not (magnet & ring)
    assert ring & floor_mask
    assert ordinary_x3 & floor_mask
    assert ordinary_x3 & ring


def test_ground_release_is_kinematically_compatible_with_floor_support(tmp_path):
    """Sanity-check the proposed static-mask bootstrap before runtime.

    At the commissioned plate-0 ground start the sphere bottom sits on the plate
    plane, 35 mm above the floor.  Once the startup weld is released, the current
    0.45 m effective joint-to-centre length can reach the floor while the body
    stays grounded with only a millimetre-scale radial repositioning.
    """
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
    )
    manifest = __import__("json").loads(result.manifest_path.read_text(encoding="utf-8"))
    cfg = manifest["config"]
    joint = np.asarray(manifest["joint_anchor_world_m"], dtype=float)
    magnet = np.asarray(manifest["magnet_center_world_m"], dtype=float)
    radius = float(cfg["magnet_radius_m"])
    floor_z = 0.0
    effective_length = float(cfg["tether_length_m"]) + float(cfg["base_joint_offset_tether_z_m"])

    initial_bottom_z = float(magnet[2] - radius)
    assert abs(initial_bottom_z - float(cfg["ring_z_m"])) < 1e-9
    assert initial_bottom_z - floor_z == pytest.approx(0.035, abs=1e-9)

    floor_dz = (floor_z + radius) - float(joint[2])
    assert abs(floor_dz) < effective_length
    required_floor_horizontal = float(np.sqrt(effective_length**2 - floor_dz**2))
    initial_horizontal = float(np.linalg.norm((magnet - joint)[:2]))
    radial_reposition = abs(initial_horizontal - required_floor_horizontal)
    assert radial_reposition < 0.003


def test_generated_world_uses_generated_ring_before_generated_x3(tmp_path):
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
        simulation_attachment_stabilization=True,
    )
    uris = [
        (node.findtext("uri") or "").strip()
        for node in ET.parse(result.world_path).getroot().findall(".//include")
    ]
    assert result.ring_path is not None
    assert result.ring_path.name in uris
    assert result.x3_path.name in uris
    assert uris.index(result.ring_path.name) < uris.index(result.x3_path.name)
    assert "m2a_ring_fixture.sdf" not in uris


def test_bootstrap_release_interlock_prevents_raw_detach_and_timeout_until_released():
    cfg = M2BConfig(bootstrap_timeout_s=0.5)
    gate = M2BBootstrapGate(cfg, start_time_s=0.0, release_required=True)

    held = gate.step(5.0, joint_truth_fresh=True, joint_detached=False)
    assert held.status == BootstrapStatus.DETACHING
    assert held.request_raw_detach is False
    assert held.arm_permitted is False
    assert "release" in held.reason.lower()

    gate.release(now_s=5.0)
    active = gate.step(5.01, joint_truth_fresh=True, joint_detached=False)
    assert active.status == BootstrapStatus.DETACHING
    assert active.request_raw_detach is True
    assert active.arm_permitted is False


def test_post_detach_floor_gate_requires_real_floor_support_and_quiet_motion():
    ok = verify_post_detach_floor_settle(
        drone_position_world=np.array([0.70, 0.0, 0.105]),
        drone_velocity_world=np.array([0.005, 0.0, 0.0]),
        magnet_position_world=np.array([0.252, 0.0, 0.0255]),
        magnet_speed_mps=0.012,
        joint_detached=True,
        magnet_radius_m=0.025,
    )
    assert ok.ok is True

    falling = verify_post_detach_floor_settle(
        drone_position_world=np.array([0.70, 0.0, 0.105]),
        drone_velocity_world=np.zeros(3),
        magnet_position_world=np.array([0.25, 0.0, 0.050]),
        magnet_speed_mps=0.20,
        joint_detached=True,
        magnet_radius_m=0.025,
    )
    assert falling.ok is False
    assert any("floor" in reason or "speed" in reason for reason in falling.reasons)

    still_attached = verify_post_detach_floor_settle(
        drone_position_world=np.array([0.70, 0.0, 0.105]),
        drone_velocity_world=np.zeros(3),
        magnet_position_world=np.array([0.25, 0.0, 0.025]),
        magnet_speed_mps=0.0,
        joint_detached=False,
        magnet_radius_m=0.025,
    )
    assert still_attached.ok is False
    assert any("detached" in reason for reason in still_attached.reasons)


def test_runner_verifies_pre_detach_geometry_then_releases_then_floor_settles_before_arm():
    text = RUNNER.read_text(encoding="utf-8")
    precheck = text.index("m2b_ground_spawn check")
    release = text.index("/m2b/bootstrap/release")
    detached = text.index('echo "Bootstrap raw joint truth: DETACHED"')
    floor_settle = text.index("check-post-detach-floor")
    arm_gate = text.index("OPERATOR GATE: click ARM")
    arm_observed = text.index("wait_for_csv_value_guarded_operator armed true")
    assert precheck < release < detached < floor_settle < arm_gate < arm_observed
    assert "/world/quadcopter/disable_collision" not in text
    assert "/world/quadcopter/enable_collision" not in text
    assert "POST_DETACH_GOOD_COUNT" in text
    assert "POST_DETACH_REQUIRED_GOOD=5" in text


def test_b1_physical_capture_manager_cannot_bypass_supervised_startup_release():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert '"force_initial_detach": False' in launch
    assert '"raw_detach_request_topic": "/m2b/sim/raw_detach_request"' in launch


def test_b1_launch_requires_supervised_bootstrap_release_but_b0_default_remains_compatible():
    launch = LAUNCH.read_text(encoding="utf-8")
    planner = PLANNER.read_text(encoding="utf-8")
    assert '"m2b_bootstrap_release_required": True' in launch
    assert '"m2b_bootstrap_release_topic": "/m2b/bootstrap/release"' in launch
    assert 'declare_parameter("m2b_bootstrap_release_required", False)' in planner
    assert 'declare_parameter("m2b_bootstrap_release_topic", "/m2b/bootstrap/release")' in planner
    assert "m2b_bootstrap_gate.release" in planner
