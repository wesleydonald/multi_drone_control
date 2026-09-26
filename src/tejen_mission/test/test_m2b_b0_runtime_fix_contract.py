"""Regression contracts for the M2B B0 runtime commissioning fixes.

These tests are intentionally ROS/Gazebo independent. They encode the runtime
failures observed on 2026-09-10 before implementation is changed:
- payload model must exist before X3 DetachableJoint configures;
- the real X3 must be initialized with a near-horizontal, above-ground tether;
- B0 telemetry must be structured CSV rather than many ros2-topic-echo logs;
- the runner must own/clean a process group and refuse a contaminated ROS graph;
- simulator-side gz command timeouts must fail closed instead of killing the node.
"""

from pathlib import Path
import math

import numpy as np

from tejen_mission.m2b_ground_start import GroundStartConfig, compute_ground_start_geometry


ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b0.sh"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2b_b0_single_attachment.launch.py"
SETUP = ROOT / "src" / "tejen_mission" / "setup.py"
MANAGER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2a_physical_capture_manager.py"
INITIALIZER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2b_ground_initializer.py"
LOGGER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "m2b_b0_telemetry.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_ground_start_geometry_is_near_horizontal_and_contacts_plate_without_ground_penetration():
    cfg = GroundStartConfig()
    geometry = compute_ground_start_geometry(cfg, assigned_plate_id=0)

    cable_vector = geometry.magnet_center_world - geometry.tether_origin_world
    assert math.isclose(float(np.linalg.norm(cable_vector)), cfg.tether_length_m, abs_tol=1e-9)

    # Commissioning intent: cable lies along the ground direction, not vertically.
    horizontal = float(np.linalg.norm(cable_vector[:2]))
    angle_from_horizontal = math.atan2(abs(float(cable_vector[2])), horizontal)
    assert angle_from_horizontal < math.radians(10.0)

    # The magnet sphere is clear of the floor and its downward contact point lies
    # on the assigned plate plane.
    assert geometry.magnet_center_world[2] - cfg.magnet_radius_m >= -1e-9
    assert math.isclose(
        geometry.magnet_center_world[2] - cfg.magnet_radius_m,
        geometry.plate_position_world[2],
        abs_tol=1e-9,
    )

    # The physical X3 body collision box is above the ground plane.
    assert geometry.body_position_world[2] - cfg.body_collision_half_height_m >= 0.0



def test_ground_start_rotation_preserves_existing_base_to_tether_joint_anchor_geometry():
    cfg = GroundStartConfig()
    geometry = compute_ground_start_geometry(cfg, assigned_plate_id=0)
    expected_anchor = geometry.body_position_world + np.array([0.0, 0.0, -0.040])
    assert np.allclose(geometry.joint_anchor_world, expected_anchor, atol=1e-9)
    assert math.isclose(
        float(np.linalg.norm(geometry.magnet_center_world - geometry.joint_anchor_world)),
        0.450,
        abs_tol=1e-9,
    )

def test_ground_start_geometry_preserves_body_yaw_and_rotates_position_for_assigned_plate():
    cfg = GroundStartConfig(initial_body_yaw_rad=0.73)
    g0 = compute_ground_start_geometry(cfg, assigned_plate_id=0)
    g3 = compute_ground_start_geometry(cfg, assigned_plate_id=3)

    assert math.isclose(g0.body_yaw_rad, 0.73, abs_tol=1e-12)
    assert math.isclose(g3.body_yaw_rad, 0.73, abs_tol=1e-12)
    assert not np.allclose(g0.body_position_world[:2], g3.body_position_world[:2])
    assert math.isclose(
        float(np.linalg.norm(g0.body_position_world[:2] - g0.plate_position_world[:2])),
        float(np.linalg.norm(g3.body_position_world[:2] - g3.plate_position_world[:2])),
        abs_tol=1e-9,
    )


def test_world_loads_grounded_payload_before_real_x3_and_has_single_floor_collision():
    text = _text(WORLD)
    ring_at = text.index("m2a_ring_fixture.sdf")
    x3_at = text.index("modelLargeM2BallMagnet.sdf")
    assert ring_at < x3_at
    assert text.count('<model name="ground_plane">') + text.count('<model name="floor">') == 1
    assert "<pose>0 0 0.035 0 0 0</pose>" in text


def test_runner_initializes_horizontal_ground_geometry_before_first_world_step():
    # Sep-10 runtime evidence superseded the child-link set_pose mechanism:
    # Gazebo rejected set_pose on joint-connected tether/magnet links. The
    # strengthened contract now requires articulated SDF generation before
    # launch, followed by a measured geometry gate before unpause/arming.
    text = _text(RUNNER)
    generate_at = text.index("m2b_ground_spawn generate")
    launch_at = text.index("setsid ros2 launch")
    check_at = text.index("m2b_ground_spawn check")
    unpause_at = text.index("--req 'pause: false'")
    assert generate_at < launch_at < check_at < unpause_at
    assert "ground_start_measured.json" in text
    assert "world_m2b_single_attachment.sdf" in text


def test_runner_uses_structured_b0_csv_not_topic_echo_log_files():
    text = _text(RUNNER)
    assert "start_topic_log" not in text
    assert "joint_truth.log" not in text
    assert "arm_permission.log" not in text
    assert "phase.log" not in text
    assert "mpc_mode.log" not in text
    assert "controller_mpc_mode_status.log" not in text
    assert "arming_state.log" not in text
    assert "b0_status.csv" in text
    assert "runtime.log" in text


def test_runner_owns_launch_process_group_and_checks_for_stale_stack():
    text = _text(RUNNER)
    assert "setsid" in text
    assert "kill -- -\"$LAUNCH_PGID\"" in text or "kill -INT -- -\"$LAUNCH_PGID\"" in text
    assert "assert_clean_ros_graph" in text
    assert "assert_clean_gazebo" in text
    assert "pgrep" in text
    assert "online_join_planner" in text
    assert "tejen_mpc" in text


def test_launch_runs_structured_telemetry_logger_in_existing_m2_log_tree():
    text = _text(LAUNCH)
    assert 'LaunchConfiguration("m2b_log_dir")' in text
    assert 'executable="m2b_b0_telemetry"' in text
    assert '"log_dir": m2b_log_dir' in text


def test_b0_telemetry_module_writes_csv_and_metadata():
    text = _text(LOGGER)
    assert '"b0_status.csv"' in text
    assert '"metadata.json"' in text
    assert "csv.DictWriter" in text
    assert '"joint_detached"' in text
    assert '"arm_permission"' in text
    assert '"mission_phase"' in text
    assert '"requested_mpc_mode"' in text
    assert '"effective_mpc_mode"' in text
    assert '"armed"' in text


def test_ground_initializer_is_normal_console_script_and_uses_scoped_real_x3_links():
    setup = _text(SETUP)
    source = _text(INITIALIZER)
    assert "m2b_ground_initializer = tejen_mission.m2b_ground_initializer:main" in setup
    assert '"x3::tether_rod"' in source
    assert '"x3::magnet_tip_link"' in source
    assert "/world/quadcopter/set_pose" in source


def test_gz_timeout_is_caught_and_fails_closed_in_physical_capture_manager():
    text = _text(MANAGER)
    assert "except subprocess.TimeoutExpired" in text
    assert "return False" in text


def test_runner_has_explicit_ground_sanity_mode_before_any_arm_command():
    text = _text(RUNNER)
    assert 'MODE="${1:-sanity}"' in text
    assert '"$MODE" == "sanity"' in text
    sanity_at = text.index('"$MODE" == "sanity"')
    arm_at = text.index("ros2 service call /drone_arming_service")
    assert sanity_at < arm_at
    assert "SANITY_PASS" in text


def test_ground_start_seed_geometry_matches_current_real_x3_sdf():
    import xml.etree.ElementTree as ET

    x3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"
    model = ET.parse(x3).getroot().find("model")
    assert model is not None

    base = model.find("./link[@name='X3/base_link']")
    tether = model.find("./link[@name='tether_rod']")
    magnet = model.find("./link[@name='magnet_tip_link']")
    joint = model.find("./joint[@name='base_to_tether']")
    assert base is not None and tether is not None and magnet is not None and joint is not None

    cfg = GroundStartConfig()
    base_size = [float(v) for v in base.find("./collision/geometry/box/size").text.split()]
    tether_pose = [float(v) for v in tether.find("pose").text.split()]
    joint_pose = [float(v) for v in joint.find("pose").text.split()]
    tether_length = float(tether.find("./visual/geometry/cylinder/length").text)
    magnet_radius = float(magnet.find("./collision/geometry/sphere/radius").text)

    assert math.isclose(cfg.body_collision_half_height_m, base_size[2] / 2.0, abs_tol=1e-12)
    assert math.isclose(cfg.tether_origin_initial_body_z_m, tether_pose[2], abs_tol=1e-12)
    assert math.isclose(cfg.base_joint_offset_tether_z_m, joint_pose[2], abs_tol=1e-12)
    assert math.isclose(cfg.tether_length_m, tether_length, abs_tol=1e-12)
    assert math.isclose(cfg.magnet_radius_m, magnet_radius, abs_tol=1e-12)


def test_grounded_ring_height_matches_current_ring_collision_bottom():
    import xml.etree.ElementTree as ET

    ring = ROOT / "simulation_assets" / "tejen" / "m2a_ring_fixture.sdf"
    model = ET.parse(ring).getroot().find("model")
    assert model is not None
    bottoms = []
    for collision in model.findall(".//collision"):
        pose = collision.find("pose")
        size = collision.find("./geometry/box/size")
        if pose is None or size is None:
            continue
        pose_values = [float(v) for v in pose.text.split()]
        size_values = [float(v) for v in size.text.split()]
        bottoms.append(pose_values[2] - size_values[2] / 2.0)
    assert bottoms
    assert math.isclose(GroundStartConfig().ring_z_m + min(bottoms), 0.0, abs_tol=1e-12)
