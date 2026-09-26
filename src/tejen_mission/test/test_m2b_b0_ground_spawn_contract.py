"""Regression contracts for measured M2B B0 ground-start commissioning.

These tests encode the Sep-10 runtime failure where Gazebo accepted the model
set_pose but rejected set_pose commands for joint-connected child links.  The
fix must configure the real X3's initial articulated pose before Gazebo creates
its joints, then prove the measured pose before unpausing or arming.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from tejen_mission.m2b_ground_spawn import (
    generate_ground_start_assets,
    verify_measured_ground_start,
)


ROOT = Path(__file__).resolve().parents[3]
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b0.sh"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2b_b0_single_attachment.launch.py"
TELEMETRY = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2b_b0_telemetry.py"


def _model_counts(path: Path) -> tuple[int, int, int]:
    model = ET.parse(path).getroot().find("model")
    assert model is not None
    return len(model.findall("link")), len(model.findall("joint")), len(model.findall("plugin"))


def test_generated_x3_preserves_real_model_and_changes_only_tether_initial_pose(tmp_path):
    before = X3.read_text(encoding="utf-8")
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
    )
    after_source = X3.read_text(encoding="utf-8")
    generated = result.x3_path.read_text(encoding="utf-8")

    assert after_source == before
    assert _model_counts(result.x3_path) == _model_counts(X3)
    assert '<model name="x3">' in generated
    assert '<link name="X3/base_link">' in generated
    assert '<link name="tether_rod">' in generated
    assert '<link name="magnet_tip_link">' in generated
    assert '<joint name="base_to_tether" type="ball">' in generated
    assert '<joint name="tether_to_magnet_tip" type="ball">' in generated
    assert 'gz-sim-detachable-joint-system' in generated
    assert '<kinematic>true</kinematic>' not in generated

    # The generator is intentionally surgical. Undoing its one tether pose edit
    # must recover the exact source bytes.
    restored = generated.replace(result.generated_tether_pose_element, result.source_tether_pose_element, 1)
    assert restored == before


def test_generated_tether_pose_places_magnet_near_assigned_plate(tmp_path):
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
    )
    expected = result.geometry

    # The generated model is loaded at the body pose encoded in the generated world.
    world_root = ET.parse(result.world_path).getroot()
    include = next(i for i in world_root.findall('.//include') if i.findtext('uri') == result.x3_path.name)
    pose = np.array([float(v) for v in include.findtext('pose').split()], dtype=float)
    assert np.allclose(pose[:3], expected.body_position_world, atol=1e-9)
    assert math.isclose(pose[5], expected.body_yaw_rad, abs_tol=1e-12)

    model = ET.parse(result.x3_path).getroot().find('model')
    tether = model.find("./link[@name='tether_rod']")
    vals = np.array([float(v) for v in tether.findtext('pose').split()], dtype=float)
    assert tether.find('pose').attrib.get('relative_to') == 'X3/base_link'
    # A vertical default would have roll=pitch=yaw=0.  Ground start must rotate it.
    assert abs(vals[3]) + abs(vals[4]) + abs(vals[5]) > 0.5


def test_measured_ground_gate_rejects_sep10_vertical_underground_runtime():
    result = verify_measured_ground_start(
        assigned_plate_id=0,
        drone_position_world=np.array([0.699972221, 0.0, 0.106320871]),
        drone_yaw_rad=0.0,
        magnet_position_world=np.array([0.699972221, 0.0, -0.383683129]),
    )
    assert result.ok is False
    assert result.magnet_bottom_z_m < 0.0
    assert result.cable_angle_from_horizontal_deg > 80.0
    assert result.magnet_plate_xy_error_m > 0.40


def test_measured_ground_gate_accepts_expected_horizontal_configuration():
    assets = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=Path('/tmp') / 'm2b_b0_ground_spawn_contract_expected',
        assigned_plate_id=0,
    )
    g = assets.geometry
    result = verify_measured_ground_start(
        assigned_plate_id=0,
        drone_position_world=g.body_position_world,
        drone_yaw_rad=g.body_yaw_rad,
        magnet_position_world=g.magnet_center_world,
    )
    assert result.ok is True
    assert result.cable_angle_from_horizontal_deg < 10.0
    assert result.magnet_bottom_z_m >= -1e-9
    assert result.magnet_plate_xy_error_m < 1e-9


def test_runner_generates_articulated_x3_before_launch_and_requires_measured_geometry_gate():
    text = RUNNER.read_text(encoding='utf-8')
    generate_at = text.index('m2b_ground_spawn generate')
    launch_at = text.index('setsid ros2 launch')
    check_at = text.index('m2b_ground_spawn check')
    unpause_at = text.index("--req 'pause: false'")
    arm_at = text.index('ros2 service call /drone_arming_service')

    assert generate_at < launch_at < check_at < unpause_at < arm_at
    assert 'ground_start_measured.json' in text
    assert 'ground_geometry_verified=true' in text
    assert 'm2b_generated_world.sdf' in text
    assert 'm2b_ground_initializer' not in text


def test_launch_accepts_runner_generated_world_path():
    text = LAUNCH.read_text(encoding='utf-8')
    assert 'LaunchConfiguration("world_path")' in text
    assert 'DeclareLaunchArgument("world_path"' in text
    assert 'str(world)' not in text


def test_telemetry_records_controller_fault_state_structurally():
    text = TELEMETRY.read_text(encoding='utf-8')
    assert '"external_reference_fault_latched"' in text
    assert '"pendulum_fault_latched"' in text
    assert '"/tejen_mpc/c1d_status"' in text


def test_runner_does_not_grep_human_note_for_fault_latched_hold():
    text = RUNNER.read_text(encoding='utf-8')
    # The previous implementation falsely matched the controller's informational
    # NOTE containing the phrase FAULT_LATCHED_HOLD and killed a healthy run.
    assert "grep -Eq 'M2_FAULT|Critical pendulum-state fault after TAKEOFF|FAULT_LATCHED_HOLD'" not in text
    assert 'external_reference_fault_latched' in text
    assert 'pendulum_fault_latched' in text


def test_generated_world_preserves_ring_order_and_x3_include_plugins(tmp_path):
    result = generate_ground_start_assets(
        source_x3=X3,
        world_template=WORLD,
        output_dir=tmp_path,
        assigned_plate_id=0,
    )
    root = ET.parse(result.world_path).getroot()
    includes = root.findall('.//include')
    uris = [(include.findtext('uri') or '').strip() for include in includes]
    assert 'm2a_ring_fixture.sdf' in uris
    assert result.x3_path.name in uris
    assert uris.index('m2a_ring_fixture.sdf') < uris.index(result.x3_path.name)

    source_root = ET.parse(WORLD).getroot()
    source_x3_include = next(i for i in source_root.findall('.//include') if i.findtext('uri') == 'modelLargeM2BallMagnet.sdf')
    generated_x3_include = next(i for i in includes if i.findtext('uri') == result.x3_path.name)
    source_plugins = [(p.attrib.get('filename'), p.attrib.get('name')) for p in source_x3_include.findall('plugin')]
    generated_plugins = [(p.attrib.get('filename'), p.attrib.get('name')) for p in generated_x3_include.findall('plugin')]
    assert generated_plugins == source_plugins
