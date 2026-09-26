"""Post-implementation audit contracts for M2C.

These tests deliberately target integration seams that the original RED suite could
not prove from interface existence alone: truthful stationary commitments, restart-
safe sequence streams, controller/reference wiring, and the actual generated four-X3
world.  Keep them ROS-independent so they run before user commissioning.
"""

from __future__ import annotations

import hashlib
import importlib
import math
import re
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[3]
LAUNCH = REPO / "src/tejen_mission/launch/m2c_four_drone_ground.launch.py"
CONTROLLER_LAUNCH = REPO / "src/tejen_mission/launch/m2c_vehicle_controller.launch.py"
RUNNER = REPO / "tools/sim_test/run_m2c_ground.sh"
SOURCE_X3 = REPO / "simulation_assets/tejen/modelLargeM2BallMagnet.sdf"
SOURCE_RING = REPO / "simulation_assets/tejen/m2a_ring_fixture.sdf"
WORLD_TEMPLATE = REPO / "simulation_assets/tejen/world_m2b_single_attachment.sdf"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _commitment_module():
    try:
        return importlib.import_module("tejen_mission.m2_fleet_commitment")
    except ModuleNotFoundError:
        pytest.fail(
            "M2C audit contract: ROS-independent commitment validation/session helper is missing"
        )


def _stationary_piece(position=(0.4, -0.2, 0.105), *, t0=10.0, t1=20.0):
    point = np.asarray(position, dtype=float)
    control = np.repeat(point.reshape(1, 3), 4, axis=0)
    knots = np.asarray([t0, t0, t0, t0, t1, t1, t1, t1], dtype=float)
    return control, knots


def test_m2c_stationary_commitment_validator_binds_hold_to_measured_vehicle_state():
    module = _commitment_module()
    validate = getattr(module, "validate_stationary_cubic_hold", None)
    assert callable(validate)

    expected = np.array([0.4, -0.2, 0.105], dtype=float)
    control, knots = _stationary_piece(expected)
    anchor = validate(
        control_points=control,
        knots=knots,
        valid_from_s=10.0,
        valid_until_s=20.0,
        expected_position_world=expected,
        now_s=10.2,
        position_tolerance_m=0.04,
    )
    np.testing.assert_allclose(anchor, expected, atol=1e-12, rtol=0.0)

    moving = control.copy()
    moving[-1, 0] += 0.05
    with pytest.raises(ValueError, match="stationary|control"):
        validate(
            control_points=moving,
            knots=knots,
            valid_from_s=10.0,
            valid_until_s=20.0,
            expected_position_world=expected,
            now_s=10.2,
            position_tolerance_m=0.04,
        )

    offset, offset_knots = _stationary_piece(expected + np.array([0.08, 0.0, 0.0]))
    with pytest.raises(ValueError, match="position|state|anchor"):
        validate(
            control_points=offset,
            knots=offset_knots,
            valid_from_s=10.0,
            valid_until_s=20.0,
            expected_position_world=expected,
            now_s=10.2,
            position_tolerance_m=0.04,
        )


def test_m2c_commitment_sequence_seed_is_nonzero_and_restart_ordered():
    module = _commitment_module()
    seed = getattr(module, "commitment_sequence_seed", None)
    assert callable(seed)
    a = int(seed(now_ns=1_800_000_000_000_000_000))
    b = int(seed(now_ns=1_800_000_000_000_000_123))
    assert 0 < a < b < 2**64

    planner = (
        REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
    ).read_text(encoding="utf-8")
    assert "commitment_sequence_seed" in planner
    assert "committed_trajectory_sequence = 0" not in planner


def test_m2c_controller_consumes_each_namespaced_reference_and_runner_checks_command_path():
    controller_launch = CONTROLLER_LAUNCH.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert '"use_external_reference": True' in controller_launch, (
        "M2C must prove the planner/reference -> MPC subscription path even while disarmed"
    )
    assert '"external_reference_topic": reference_topic' in controller_launch
    assert 'wait_for_subscriber "/drone_${i}/join_planner/reference"' in runner
    assert 'wait_for_subscriber "/drone_${i}/ELRSCommand"' in runner



def test_m2c_ego_commitment_publisher_matches_peer_backend_transient_local_qos():
    planner = (
        REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
    ).read_text(encoding="utf-8")
    # The C++ shared-trajectory consumer requests reliable/transient-local QoS.
    # M2C's real ego publisher must offer compatible durability, otherwise the
    # M2D peer subscription will never connect despite correct topic names.
    marker = "if self.publish_committed_trajectory:"
    start = planner.index(marker)
    block = planner[start:start + 900]
    assert "TRANSIENT_LOCAL" in block or "DurabilityPolicy.TRANSIENT_LOCAL" in block
    assert "RELIABLE" in block or "ReliabilityPolicy.RELIABLE" in block

def test_m2c_generated_world_has_four_real_x3_motor_routes_and_preserves_sources(tmp_path):
    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    generate = getattr(spawn, "generate_m2c_assets")

    before = {path: _sha256(path) for path in (SOURCE_X3, SOURCE_RING, WORLD_TEMPLATE)}
    manifest = generate(
        source_x3=SOURCE_X3,
        source_ring=SOURCE_RING,
        world_template=WORLD_TEMPLATE,
        output_dir=tmp_path,
        ring_yaw_deg=20.0,
        manifest_path=tmp_path / "manifest.json",
    )
    after = {path: _sha256(path) for path in before}
    assert after == before
    assert manifest["source_assets_modified"] is False
    assert manifest["ring_yaw_deg"] == pytest.approx(20.0)

    vehicles = manifest["vehicles"]
    assert len(vehicles) == 4
    assert all(float(v["cable_angle_from_horizontal_deg"]) < 10.0 for v in vehicles)

    world = ET.parse(manifest["generated_world"]).getroot().find("world")
    assert world is not None
    includes = []
    namespaces = []
    for include in world.findall("include"):
        name = (include.findtext("name") or "").strip()
        if name.startswith("x3_"):
            includes.append(name)
            namespaces.extend(
                (plugin.findtext("robotNamespace") or "").strip()
                for plugin in include.findall("plugin")
                if plugin.find("robotNamespace") is not None
            )
    assert set(includes) == {"x3_0", "x3_1", "x3_2", "x3_3"}
    assert len(namespaces) == 16
    for i in range(4):
        assert namespaces.count(f"drone_{i}") == 4

    ring = ET.parse(manifest["generated_ring"]).getroot().find("model")
    assert ring is not None
    detachable = [
        plugin
        for plugin in ring.findall("plugin")
        if "DetachableJoint" in (plugin.attrib.get("name", "") + plugin.attrib.get("filename", ""))
    ]
    assert len(detachable) == 4
    output_topics = {(plugin.findtext("output_topic") or "").strip() for plugin in detachable}
    assert output_topics == {
        f"/drone_{i}/magnet/joint_detached_truth" for i in range(4)
    }


def test_m2c_commissioning_staging_exercises_nonidentity_plate_assignment():
    """The live M2C scene must not accidentally make hardware ID == plate ID."""

    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    assignment = importlib.import_module("tejen_mission.m2_fleet_assignment")
    geometry_mod = importlib.import_module("tejen_mission.cooperative_trajectory")

    def rz(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    vehicles = []
    for i in range(4):
        ground = spawn.grounded_vehicle_geometry(
            i,
            stage_radius_m=spawn.DEFAULT_STAGE_RADIUS_M[i],
            stage_angle_rad=math.radians(spawn.DEFAULT_STAGE_ANGLE_DEG[i]),
            body_yaw_rad=math.radians(spawn.DEFAULT_BODY_YAW_DEG[i]),
        )
        vehicles.append(
            assignment.FleetVehicleState(
                vehicle_id=f"drone_{i}",
                physical_drone_id=i,
                namespace=f"/drone_{i}",
                mocap_rigid_body_id=100 + i,
                position_world=np.asarray(ground.body_position_world, dtype=float),
                yaw_rad=ground.body_yaw_rad,
                stamp_s=10.0,
            )
        )

    result = assignment.choose_fleet_assignment(
        vehicles=vehicles,
        ring_position_world=np.array([0.0, 0.0, 0.035], dtype=float),
        rotation_world_from_ring=rz(math.radians(20.0)),
        now_s=10.0,
        ring_stamp_s=10.0,
        geometry=geometry_mod.RingNetGeometry(),
        config=assignment.FleetAssignmentConfig(),
    )
    mapping = dict(result.vehicle_to_plate)
    assert all(mapping[f"drone_{i}"] != i for i in range(4)), mapping




def test_m2c_ground_commissioning_publishes_real_current_pose_reference_messages():
    launch = LAUNCH.read_text(encoding="utf-8")
    planner = (
        REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
    ).read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert '"m2c_ground_reference_enabled": True' in launch
    assert "m2c_ground_reference_enabled" in planner
    assert "wait_for_reference" in runner
    assert '"/drone_${i}/join_planner/reference"' in runner

def test_m2c_controller_visualisation_topics_and_tf_children_are_namespace_safe():
    visual = (REPO / "src/tejen_utility_objects/tejen_utility_objects/visualization.py").read_text(
        encoding="utf-8"
    )
    for singleton in ("/trajectory_path", "/actual_path", "/mpc_plan", "/trajectory_markers"):
        assert singleton not in visual, f"M2C controller visualisation still aliases {singleton}"
    assert "get_namespace" in visual
    assert "tf_child_prefix" in visual or "namespace" in visual[visual.index("publish_transform_frame"):visual.index("publish_actual_path")]

def test_m2c_gui_runner_keeps_passed_scene_open_for_visual_inspection():
    runner = RUNNER.read_text(encoding="utf-8")
    assert "M2C_KEEP_OPEN" in runner
    pass_pos = runner.index("M2C PASS:")
    assert "Ctrl-C" in runner[pass_pos:]


def test_m2c_commissioning_exercises_each_namespaced_telemetry_route():
    launch = LAUNCH.read_text(encoding="utf-8")
    betaflight = (
        REPO / "src/simulation_communication/simulation_communication/tejen_betaflight_communication.py"
    ).read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert '"publish_telemetry": True' in launch
    assert "telemetry_topic" in betaflight
    assert "Telemetry" in betaflight
    assert "wait_for_telemetry" in runner


def test_m2c_ground_reference_mode_cannot_fall_through_into_normal_mission_reference_generation():
    """Ground-only M2C must have exactly one reference authority path."""
    planner = (
        REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
    ).read_text(encoding="utf-8")
    start = planner.index("def timer_callback(self) -> None:")
    block = planner[start:start + 1800]
    ground_call = block.index("self.publish_m2c_ground_reference()")
    inputs_gate = block.index("if not self.inputs_ready()")
    between = block[ground_call:inputs_gate]
    assert "return" in between, (
        "M2C ground mode must return after its current-pose reference so the normal "
        "join mission cannot publish a second/moving reference in the same callback"
    )


def test_m2c_argument_mode_ros_gz_bridges_use_the_humble_commissioned_type_names():
    """Stay on the already-working Humble bridge spelling used by M1/M2B."""
    launch = LAUNCH.read_text(encoding="utf-8")
    assert "ignition.msgs.Pose_V" in launch
    assert "ignition.msgs.Actuators" in launch
    assert "gz.msgs.Pose_V" not in launch
    assert "gz.msgs.Actuators" not in launch


def test_m2c_keeps_runtime_workdirs_isolated_while_sharing_only_acados_artifacts():
    """Per-drone logs/CWD stay isolated; immutable ACADOS artifacts may be shared."""
    controller_launch = CONTROLLER_LAUNCH.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("controller_work_dir"' in controller_launch
    assert "cwd=str(work_dir)" in controller_launch
    assert 'controller_work_dir:="$CONTROLLER_WORK_ROOT/drone_${i}"' in runner
    assert 'mkdir -p "$CONTROLLER_WORK_ROOT/drone_${i}"' in runner or 'for i in 0 1 2 3' in runner
    assert '"acados_cache_dir": str(cache_dir)' in controller_launch
    assert 'ACADOS_CACHE_ROOT="$REPO/build/acados_cache/payload_mpc"' in runner
    assert 'acados_cache_dir:="$ACADOS_CACHE_ROOT"' in runner


def test_m2c_ground_tethers_extend_radially_outward_and_remain_clear_of_basket_footprint():
    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    for i in range(4):
        ground = spawn.grounded_vehicle_geometry(
            i,
            stage_radius_m=spawn.DEFAULT_STAGE_RADIUS_M[i],
            stage_angle_rad=math.radians(spawn.DEFAULT_STAGE_ANGLE_DEG[i]),
            body_yaw_rad=math.radians(spawn.DEFAULT_BODY_YAW_DEG[i]),
        )
        joint = np.asarray(ground.joint_anchor_world, dtype=float)
        magnet = np.asarray(ground.magnet_center_world, dtype=float)
        cable_xy = magnet[:2] - joint[:2]
        radial_xy = joint[:2]
        assert float(np.dot(cable_xy, radial_xy)) > 0.0
        # Basket half-width is about 0.28 m.  The whole tether starts much farther
        # out than this and then moves radially outward, so no ground cable crosses it.
        assert float(np.linalg.norm(joint[:2])) > 0.80
        assert float(np.linalg.norm(magnet[:2])) > float(np.linalg.norm(joint[:2]))
        assert float(ground.cable_angle_from_horizontal_deg) < 10.0


def test_m2c_runner_requires_post_acados_controller_heartbeat_before_declaring_platform_ready():
    runner = RUNNER.read_text(encoding="utf-8")
    assert "wait_for_controller_ready" in runner
    assert '"/drone_${drone_id}/tejen_mpc/c1d_status"' in runner or \
        '"/drone_${i}/tejen_mpc/c1d_status"' in runner
    ready_line = runner.index("Four controller/platform routes: READY")
    assert runner.rfind("wait_for_controller_ready", 0, ready_line) >= 0


def test_m2c_frozen_assignment_is_late_joiner_safe():
    manager = (
        REPO / "src/tejen_mission/tejen_mission/m2_fleet_manager.py"
    ).read_text(encoding="utf-8")
    assignment_pub = manager.index("assignment_pub")
    local = manager[max(0, assignment_pub - 1000):assignment_pub + 500]
    assert "TRANSIENT_LOCAL" in local, (
        "Frozen assignment is published once, so the publisher must be transient-local "
        "for a later M2D consumer / evidence subscriber to receive it"
    )


def test_m2c_runner_uses_current_status_for_assignment_gate_and_transient_assignment_for_evidence():
    runner = RUNNER.read_text(encoding="utf-8")

    wait_start = runner.index("wait_for_assignment()")
    wait_end = runner.index("wait_for_disarmed_count()", wait_start)
    wait_block = runner[wait_start:wait_end]
    assert "/m2c/status" in wait_block
    assert '"assignment_frozen": true' in wait_block
    assert '"candidate_count": 72' in wait_block
    assert "/m2c/assignment" not in wait_block, (
        "Commissioning must not depend on consuming the one-shot assignment message"
    )

    assignment_echoes = [
        value for value in runner.splitlines()
        if "/m2c/assignment" in value
    ]
    assert assignment_echoes
    assert any("--qos-durability transient_local" in value for value in assignment_echoes)


def test_m2c_launch_package_declares_runtime_dependencies_without_reintroducing_planner_cycle():
    """The package owning the launch must declare every cross-package runtime it starts."""
    package_xml = (REPO / "src/tejen_mission/package.xml").read_text(encoding="utf-8")
    for dep in (
        "launch",
        "launch_ros",
        "ament_index_python",
        "ros_gz_bridge",
        "simulation_communication",
        "tejen_mpc",
        "drone_visualisation",
    ):
        assert f"<exec_depend>{dep}</exec_depend>" in package_xml
    assert "<exec_depend>tejen_dynamic_planner</exec_depend>" not in package_xml, (
        "tejen_dynamic_planner already depends on tejen_mission; adding the reverse "
        "runtime dependency recreates the B2 colcon package cycle"
    )


def test_m2c_default_rviz_renders_fleet_assignment_markers_for_gui_commissioning():
    """RViz is useful, but starts only after the four-controller coexistence gate."""
    rviz = (REPO / "src/drone_visualisation/rviz/default.rviz").read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    assert "/m2c/assignment_markers" in rviz
    assert "M2C Fleet Assignment" in rviz
    final_gate = runner.index("Final four-controller coexistence gate")
    rviz_launch = runner.index("view_frame.launch.py")
    assert final_gate < rviz_launch
    assert "RViz:       deferred until all four controllers pass the coexistence gate" in runner


def test_m2c_generated_world_omits_unused_generic_sensor_systems(tmp_path):
    """Run-local M2C keeps an inert IMU descriptor but no sensor systems."""
    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    manifest = spawn.generate_m2c_assets(
        source_x3=SOURCE_X3,
        source_ring=SOURCE_RING,
        world_template=WORLD_TEMPLATE,
        output_dir=tmp_path,
        ring_yaw_deg=20.0,
        manifest_path=tmp_path / "manifest.json",
    )
    world = ET.parse(manifest["generated_world"]).getroot().find("world")
    assert world is not None

    def kind(plugin):
        return (plugin.attrib.get("name", "") + " " + plugin.attrib.get("filename", "")).lower()

    world_kinds = [kind(plugin) for plugin in world.findall("plugin")]
    assert not any("systems::sensors" in value or "sensors-system" in value for value in world_kinds)
    assert not any("systems::imu" in value or "imu-system" in value for value in world_kinds)

    for include in world.findall("include"):
        name = (include.findtext("name") or "").strip()
        if not name.startswith("x3_"):
            continue
        child_kinds = [kind(plugin) for plugin in include.findall("plugin")]
        assert not any("systems::sensors" in value or "sensors-system" in value for value in child_kinds)
        assert not any("systems::imu" in value or "imu-system" in value for value in child_kinds)


def test_m2c_run_local_performance_trim_removes_unused_sensors_and_caps_pose_rate(tmp_path):
    """M2C should trim only run-local simulation load, never canonical assets."""
    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    source_before = SOURCE_X3.read_bytes()
    world_before = WORLD_TEMPLATE.read_bytes()
    manifest = spawn.generate_m2c_assets(
        source_x3=SOURCE_X3,
        source_ring=SOURCE_RING,
        world_template=WORLD_TEMPLATE,
        output_dir=tmp_path,
        ring_yaw_deg=20.0,
        manifest_path=tmp_path / "manifest.json",
    )

    assert SOURCE_X3.read_bytes() == source_before
    assert WORLD_TEMPLATE.read_bytes() == world_before
    assert manifest["run_local_camera_sensors_enabled"] is False
    assert manifest["run_local_imu_sensors_enabled"] is False
    assert manifest["run_local_pose_update_hz"] == 120.0

    for generated in manifest["generated_x3"]:
        root = ET.parse(generated).getroot()
        assert root.find(".//sensor[@type='camera']") is None
        # Keep the IMU entity purely to preserve Gazebo entity topology and the
        # legacy fixed PoseArray indices.  With no Imu / Sensors world systems
        # below, it is computationally inert in M2C.
        assert root.find(".//sensor[@type='imu']") is not None

    world = ET.parse(manifest["generated_world"]).getroot().find("world")
    assert world is not None

    def plugin_identity(plugin):
        return (plugin.attrib.get("name", "") + " " + plugin.attrib.get("filename", "")).lower()

    world_plugins = [plugin_identity(plugin) for plugin in world.findall("plugin")]
    assert not any("systems::imu" in value or "imu-system" in value for value in world_plugins)
    assert not any("systems::sensors" in value or "sensors-system" in value for value in world_plugins)

    pose_rates = []
    for include in world.findall("include"):
        for plugin in include.findall("plugin"):
            identity = plugin_identity(plugin)
            if "posepublisher" in identity or "pose-publisher" in identity:
                pose_rates.append(float(plugin.findtext("update_frequency")))
    assert len(pose_rates) == 5
    assert pose_rates == [120.0] * 5

    launch = LAUNCH.read_text(encoding="utf-8")
    assert '"publish_rate_hz": 120.0' in launch


def test_m2c_ground_reference_is_trimmed_to_exact_controller_required_sample_count():
    planner = (REPO / "src/tejen_mission/tejen_mission/online_join_planner.py").read_text(encoding="utf-8")
    launch = LAUNCH.read_text(encoding="utf-8")
    controller = (REPO / "src/tejen_mpc/tejen_mpc/main.py").read_text(encoding="utf-8")

    assert 'declare_parameter("m2c_ground_reference_samples", 61)' in planner
    assert "self.m2c_ground_reference_samples" in planner
    ground_block = planner[planner.index("def publish_m2c_ground_reference"):planner.index("def publish_stationary_committed_trajectory")]
    assert "self.m2c_ground_reference_samples" in ground_block
    assert "self.horizon_samples" not in ground_block
    assert '"m2c_ground_reference_samples": 61' in launch
    assert "self.N * self.skip_steps + 1" in controller
    assert "self.N = 20" in controller
    assert "self.skip_steps = 3" in controller

def test_m2c_runner_performs_established_stale_process_cleanup_before_launch():
    """M2C must not stack on a stale Gazebo / ROS commissioning process set."""
    runner = RUNNER.read_text(encoding="utf-8")
    cleanup_path = REPO / "tools/sim_test/cleanup_m2b_stale.sh"
    cleanup = cleanup_path.read_text(encoding="utf-8")

    cleanup_call = runner.index("cleanup_m2b_stale.sh")
    launch_call = runner.index("start_managed ros2 launch")
    assert cleanup_call < launch_call
    assert ".m2c_active_pgid" in cleanup
    assert "m2c_four_drone_ground.launch.py" in cleanup
    assert "m2c_fleet_manager" in cleanup

def test_m2c_runner_uses_humble_subscription_count_label_for_topic_wiring_gates():
    """Humble `ros2 topic info` reports `Subscription count`, not `Subscriber count`."""
    runner = RUNNER.read_text(encoding="utf-8")
    assert "Subscription count:" in runner
    assert "Subscriber count:" not in runner



def test_m2c_runner_buffers_topic_echo_before_matching_under_pipefail():
    """Run 104907 froze assignment, but the streaming observer still returned false.

    Topic messages can contain multiple YAML lines.  Under `set -o pipefail`,
    `grep -q` is allowed to close a live pipe as soon as it finds its match, which
    can make the still-writing ros2 CLI report a non-zero status.  Buffer the
    complete one-shot message first, then inspect the shell variable.
    """
    runner = RUNNER.read_text(encoding="utf-8")
    assert "topic_once_output()" in runner
    assert "output=\"$(topic_once_output" in runner or "status=\"$(topic_once_output" in runner

    # No commissioning topic-observer should stream ros2 output straight into a
    # quiet grep.  The Gazebo discovery parser is separately tested below.
    assert not re.search(
        r"ros2 topic echo[^\n]*(?:\\\n[^\n]*)?\|\s*grep\s+-[A-Za-z]*q",
        runner,
    )


def test_m2c_status_observers_request_full_length_humble_string_output():
    """Run 110216 exposed ros2cli's default 128-character String truncation.

    /m2c/status is JSON and the sorted disarmed_controller_count field occurs
    beyond the default rendering limit.  Every current-state read must therefore
    opt into full String output instead of matching a truncated CLI rendering.
    """
    runner = RUNNER.read_text(encoding="utf-8")
    assert "m2c_status_once_output()" in runner

    helper_start = runner.index("m2c_status_once_output()")
    helper_end = runner.index("gz_topic_subscriber_line()", helper_start)
    helper = runner[helper_start:helper_end]
    assert "--full-length" in helper
    assert "/m2c/status std_msgs/msg/String" in helper

    assert "topic_once_output 3 /m2c/status" not in runner
    assert 'status="$(m2c_status_once_output 3)"' in runner

    for marker in ("m2c_status_failure.txt", '"$LOG_DIR/m2c_status.txt"'):
        pos = runner.index(marker)
        local = runner[max(0, pos - 180):pos + 120]
        assert "--full-length" in local


def test_m2c_failure_path_snapshots_current_fleet_state_before_cleanup():
    runner = RUNNER.read_text(encoding="utf-8")
    assert "snapshot_failure_state()" in runner
    assert "m2c_status_failure.txt" in runner
    assert "m2c_assignment_failure.txt" in runner
    fail_start = runner.index("fail()")
    fail_end = runner.index("wait_for_node()", fail_start)
    assert "snapshot_failure_state" in runner[fail_start:fail_end]


def test_m2c_serializes_heavy_acados_controller_startup_by_health_handshake():
    """The base launch must not schedule heavy MPC constructors by wall clock."""
    launch = LAUNCH.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    controller_launch = CONTROLLER_LAUNCH.read_text(encoding="utf-8")

    assert "tejen_mpc" not in launch
    assert "M2C_CONTROLLER_FIRST_START_DELAY_S" not in launch
    assert "M2C_CONTROLLER_START_SPACING_S" not in launch
    assert "m2c_vehicle_controller.launch.py" in runner
    assert "wait_for_controller_ready" in runner
    assert "tejen_mpc" in controller_launch

    launch_pos = runner.index("m2c_vehicle_controller.launch.py")
    ready_pos = runner.index('wait_for_controller_ready "$i"', launch_pos)
    assert launch_pos < ready_pos


def test_m2c_bootstrap_uses_live_truth_listener_then_retry_until_detached():
    """Do not depend on historical ATTACHED or over-constrain Gazebo subscriber type."""
    runner = RUNNER.read_text(encoding="utf-8")
    assert "raw_attached_truth_missing" not in runner
    assert 'wait_joint_state "$i" false' not in runner
    assert "wait_for_gz_subscriber" in runner
    assert "gz topic -i -t" in runner
    assert "release_joint_until_detached" in runner
    assert 'wait_joint_state "$i" true' not in runner
    # P3 fleet-manager timers use simulation time and therefore stop while Gazebo
    # is paused. Per-release truth is proved directly; persistent fleet truth is
    # confirmed once after unpause.
    assert 'wait_for_joint_detached_count "$((i + 1))"' not in runner
    assert 'wait_for_joint_detached_count 4' in runner

    listener_gate = runner.index('wait_for_gz_subscriber "/drone_${i}/magnet/joint_detached_truth"')
    release_loop = runner.index('release_joint_until_detached "$i"')
    assert listener_gate < release_loop

    # Run 103249 showed the Harmonic CLI echo listener as the generic protobuf
    # subscriber below. Exercise the actual shell parser against that exact shape
    # so a real discovered listener cannot be rejected for not advertising gz.msgs.*.
    function_match = re.search(
        r"gz_topic_subscriber_line\(\) \{\n.*?\n\}", runner, flags=re.DOTALL
    )
    assert function_match is not None
    function_source = function_match.group(0)

    harmonic_info = """Publishers [Address, Message Type]:
  tcp://192.168.0.91:45895, gz.msgs.StringMsg
Subscribers [Address, Message Type]:
  tcp://192.168.0.91:43959, google.protobuf.Message
"""
    typed_info = """Publishers [Address, Message Type]:
  tcp://192.168.0.91:45895, gz.msgs.StringMsg
Subscribers [Address, Message Type]:
  tcp://192.168.0.91:43959, gz.msgs.StringMsg
"""
    publisher_only = """Publishers [Address, Message Type]:
  tcp://192.168.0.91:45895, gz.msgs.StringMsg
Subscribers [Address, Message Type]:
"""

    def parsed_line(sample: str) -> str:
        proc = subprocess.run(
            ["bash", "-c", function_source + '\ngz_topic_subscriber_line "$1"', "_", sample],
            check=False,
            capture_output=True,
            text=True,
        )
        return proc.stdout.strip()

    assert "google.protobuf.Message" in parsed_line(harmonic_info)
    assert "gz.msgs.StringMsg" in parsed_line(typed_info)
    assert parsed_line(publisher_only) == ""


def test_m2c_base_launch_requires_explicit_generated_world_and_has_no_b1_fallback():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("world_path")' in launch
    assert "world_m2b_single_attachment.sdf" not in launch


def test_m2c_fleet_manager_defers_assignment_until_continuous_ground_settle():
    manager = (
        REPO / "src/tejen_mission/tejen_mission/m2_fleet_manager.py"
    ).read_text(encoding="utf-8")
    assert "ground_settle_dwell_s" in manager
    assert "ground_settle_gate" in manager
    tick = manager[manager.index("def _tick(self)"):]
    settle_pos = tick.index("ground_settled")
    assign_pos = tick.index("self._try_assignment")
    assert settle_pos < assign_pos


def test_m2c_fleet_readiness_requires_positive_disarmed_feedback_from_all_four_controllers():
    manager = (
        REPO / "src/tejen_mission/tejen_mission/m2_fleet_manager.py"
    ).read_text(encoding="utf-8")
    assert 'topics["arming_state_feedback"]' in manager
    assert "_arming_callback" in manager
    assert "disarmed_count == 4" in manager


def test_m2c_ground_reference_preserves_initial_measured_yaw():
    planner = (
        REPO / "src/tejen_mission/tejen_mission/online_join_planner.py"
    ).read_text(encoding="utf-8")
    assert "m2c_initial_yaw" in planner
    assert "m2c_ground_reference_enabled" in planner
    publish = planner[planner.index("def publish_reference"):planner.index("def publish_markers", planner.index("def publish_reference"))]
    assert "m2c_initial_yaw" in publish
    assert "yaw_quaternion_xyzw" in publish


def test_m2c_runner_rechecks_all_controller_routes_together_before_pass():
    runner = RUNNER.read_text(encoding="utf-8")
    assert "Final four-controller coexistence gate" in runner
    final_gate = runner.index("Final four-controller coexistence gate")
    ready_gate = runner.index("wait_ready", final_gate)
    for token in ("wait_for_controller_ready", "wait_for_service", "wait_for_subscriber"):
        assert token in runner[final_gate:ready_gate]


def test_m2c_fleet_readiness_requires_fresh_detached_truth_not_only_cached_true():
    manager = (
        REPO / "src/tejen_mission/tejen_mission/m2_fleet_manager.py"
    ).read_text(encoding="utf-8")
    assert "joint_truth_timeout_s" in manager
    assert "joint_truth_stamps" in manager
    assert "_fresh_detached_count" in manager
    tick = manager[manager.index("def _tick(self)"):]
    assert "detached_count = self._fresh_detached_count(now)" in tick


def test_m2c_generated_vehicle_copies_disable_unused_camera_sensors(tmp_path):
    """M2C ground plumbing does not need four 40 Hz rendering cameras."""
    source_tree = ET.parse(SOURCE_X3)
    assert source_tree.getroot().find(".//sensor[@type='camera']") is not None

    spawn = importlib.import_module("tejen_mission.m2c_ground_spawn")
    manifest = spawn.generate_m2c_assets(
        source_x3=SOURCE_X3,
        source_ring=SOURCE_RING,
        world_template=WORLD_TEMPLATE,
        output_dir=tmp_path / "generated",
        manifest_path=tmp_path / "manifest.json",
    )
    for path in manifest["generated_x3"]:
        tree = ET.parse(path)
        assert tree.getroot().find(".//sensor[@type='camera']") is None


def test_m2c_performance_instrumentation_is_observer_only_and_stage_correlated():
    """Performance diagnosis must observe native Gazebo stats without changing M2C physics."""
    runner = RUNNER.read_text(encoding="utf-8")
    assert "capture_gz_world_stats.py" in runner
    assert "--topic /world/quadcopter/stats" in runner
    assert "gazebo_world_stats.csv" in runner
    assert "stages.tsv" in runner
    assert "M2C_PERF_DWELL_S" in runner

    # The diagnostic windows are deliberately placed around stable lifecycle states.
    assert 'performance_window "base_no_controllers"' in runner
    for drone_id in range(4):
        assert f'performance_window "after_controller_${{i}}"' in runner
    assert 'performance_window "after_rviz"' in runner

    # Measurement must not silently alter physics. P3 owns the explicit ROS/Gazebo
    # clock-domain contract separately; the stats observer remains read-only.
    assert "real_time_factor>" not in runner
    assert "capture_gz_world_stats.py" in runner


def test_m2c_performance_stage_log_brackets_each_serial_controller_startup():
    runner = RUNNER.read_text(encoding="utf-8")
    loop_start = runner.index("for i in 0 1 2 3; do", runner.index("Heavy ACADOS constructors"))
    loop_end = runner.index('echo "Four controller/platform routes: READY"', loop_start)
    loop = runner[loop_start:loop_end]
    start = loop.index('mark_stage "controller_${i}_start"')
    launch = loop.index("m2c_vehicle_controller.launch.py")
    ready_gate = loop.index('wait_for_controller_ready "$i"')
    ready_mark = loop.index('mark_stage "controller_${i}_ready"')
    assert start < launch < ready_gate < ready_mark


def test_m2c_controller_enables_only_the_disarmed_reference_fast_path():
    controller_launch = (REPO / "src/tejen_mission/launch/m2c_vehicle_controller.launch.py").read_text(encoding="utf-8")
    controller = (REPO / "src/tejen_mpc/tejen_mpc/main.py").read_text(encoding="utf-8")

    assert '"disarmed_external_reference_fast_path": True' in controller_launch
    assert "declare_parameter('disarmed_external_reference_fast_path', False)" in controller
    assert "_handle_disarmed_external_reference_fast_path" in controller
    callback = controller[controller.index("def external_reference_callback"):controller.index("def get_reference_trajectory")]
    assert "if self._handle_disarmed_external_reference_fast_path(msg):" in callback
    assert "return" in callback
