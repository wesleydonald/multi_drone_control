"""M2B B2 C++ transit-to-capture integration contracts.

B2 must reuse the commissioned C1F.2/C1F.8 authority machinery to move from the
post-takeoff stationary hover to the exact B1 capture-hover target, then hand
back through the existing dynamically checked bridge before any B1 attachment
logic can run. B1 defaults must remain unchanged.
"""

from pathlib import Path
import numpy as np

from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2b_attachment_mission import (
    body_reference_from_contact,
    contact_capture_position,
)
from tejen_mission.mission_definitions import m2b_single_attachment_mission
from tejen_mission.mission_types import MissionPhase


ROOT = Path(__file__).resolve().parents[3]
PLANNER = ROOT / "src" / "tejen_mission" / "tejen_mission" / "online_join_planner.py"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2b_b1_single_attachment.launch.py"
PACKAGE_XML = ROOT / "src" / "tejen_mission" / "package.xml"
B1_RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"
B2_RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b2.sh"
CLEANUP = ROOT / "tools" / "sim_test" / "cleanup_m2b_stale.sh"


def test_b2_phase_exists_in_shared_mission_between_takeoff_and_b1_capture():
    mission = m2b_single_attachment_mission()
    assert MissionPhase.M2_CPP_TRANSIT_CAPTURE in mission.phases
    text = (
        ROOT / "src" / "tejen_mission" / "tejen_mission" / "mission_definitions.py"
    ).read_text(encoding="utf-8")
    takeoff = text.index("MissionPhase.M2_TAKEOFF_HOVER")
    cpp = text.index("MissionPhase.M2_CPP_TRANSIT_CAPTURE")
    capture = text.index("MissionPhase.M2_SETTLE_CAPTURE")
    assert takeoff < cpp < capture


def test_b2_capture_target_math_is_identical_to_existing_b1_geometry():
    geometry = RingNetGeometry()
    plate = geometry.plate_position_world(
        ring_position=np.array([0.0, 0.0, 0.035]),
        rotation_world_from_ring=np.eye(3),
        plate_index=0,
    )
    contact = contact_capture_position(plate, capture_height_m=0.30)
    expected = body_reference_from_contact(
        contact_position_world=contact,
        direction_contact_to_vehicle_world=np.array([0.0, 0.0, 1.0]),
        tether_length_m=0.475,
        rotation_world_from_body=np.eye(3),
        tether_anchor_body=np.array([0.0, 0.0, -0.04]),
    )
    np.testing.assert_allclose(expected, np.array([0.25, 0.0, 0.85]), atol=1e-12)
    planner = PLANNER.read_text(encoding="utf-8")
    assert "return TargetState(self.m2b_capture_body_target(), np.zeros(3), np.zeros(3))" in planner


def test_b2_reuses_stationary_prepare_cpp_authority_and_checked_exit_bridge():
    text = PLANNER.read_text(encoding="utf-8")
    assert 'self.m2b_commissioning_stage not in {"b0", "b1", "b2"}' in text
    assert "MissionPhase.M2_CPP_TRANSIT_CAPTURE" in text
    assert "self.c1f2_prepare_phase_active()" in text
    assert "self.c1f2_handoff_ready" in text
    assert "self.c1f2_authority_acknowledged" in text
    assert "self._evaluate_c1f2_target_relative_exit_bridge()" in text
    assert "self._start_c1f2_target_relative_exit_bridge()" in text
    assert "self.c1f2_exit_bridge_active" in text


def test_b2_exit_bridge_targets_exact_stationary_capture_hover_before_b1_descent():
    text = PLANNER.read_text(encoding="utf-8")
    assert "def c1f2_exit_bridge_target_and_offset" in text
    assert "self.m2b_fixed_ring_position" in text
    assert "self.m2b_capture_body_target() - self.m2b_fixed_ring_position" in text
    assert 'if self.m2b_commissioning_stage == "b2" and self.c1f2_exit_bridge_active' in text
    assert "MissionPhase.M2_SETTLE_CAPTURE" in text


def test_b2_launch_enables_cpp_without_loaded_payload_or_fake_companions_and_keeps_ring_collision():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'b2_cpp_transit_enabled = LaunchConfiguration("b2_cpp_transit_enabled")' in text
    assert '"m2b_commissioning_stage": commissioning_stage' in text
    assert 'package="tejen_dynamic_planner"' in text
    assert '"cooperative_scene_enabled": False' in text
    assert '"payload_collision_enabled": False' in text
    assert '"require_payload_attached_for_enable": False' in text
    assert '"moving_basket_scene_enabled": True' in text
    assert '"moving_basket_committed_trajectory_topic": "/fake_payload/committed_trajectory"' in text
    assert '"trajectory_type": "stationary"' in text
    assert '"center_z": 0.035' in text


def test_b2_launch_has_one_optional_cpp_static_obstacle_while_b1_default_remains_cpp_off():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("b2_cpp_transit_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("b2_static_obstacle_enabled", default_value="false")' in text
    assert '"static_scene_enabled": ParameterValue(b2_static_obstacle_enabled, value_type=bool)' in text
    assert '"obstacle_name": "m2b_b2_transit_obstacle"' in text
    assert '"obstacle_center_x_m": 0.65' in text
    assert '"obstacle_center_y_m": 0.0' in text
    assert '"obstacle_center_z_m": 0.95' in text


def test_b2_runner_is_one_line_supervised_variant_with_obstacle_and_clear_modes():
    assert B2_RUNNER.is_file()
    text = B2_RUNNER.read_text(encoding="utf-8")
    assert "obstacle|clear" in text
    assert "M2B_COMMISSIONING_STAGE=b2" in text
    assert "M2B_B2_CPP_TRANSIT_ENABLED=true" in text
    assert "M2B_B2_STATIC_OBSTACLE_ENABLED" in text
    assert 'exec bash "$SCRIPT_DIR/run_m2b_b1.sh" success "$GUI"' in text
    b1 = B1_RUNNER.read_text(encoding="utf-8")
    assert 'COMMISSIONING_STAGE="${M2B_COMMISSIONING_STAGE:-b1}"' in b1
    assert "M2_CPP_TRANSIT_CAPTURE" in b1
    assert "backend.csv" in b1
    assert "replans.csv" in b1


def test_b2_cleanup_and_preflight_own_new_long_lived_nodes():
    cleanup = CLEANUP.read_text(encoding="utf-8")
    runner = B1_RUNNER.read_text(encoding="utf-8")
    assert "dynamic_planner_transfer_backend" in cleanup
    assert "fake_cooperative_transport_world" in cleanup
    assert "/dynamic_planner_transfer_backend" in runner
    assert "/m2b_b2_stationary_ring_commitment" in runner


def test_b2_launch_avoids_untyped_empty_sequence_parameter_overrides():
    text = LAUNCH.read_text(encoding="utf-8")
    # ROS 2 Humble launch_ros cannot infer the type of an empty sequence in a
    # parameter dictionary.  Cooperative collision is disabled for B2, so do
    # not override these YAML arrays with [].
    assert '"cooperative_drone_state_topics": []' not in text
    assert '"cooperative_committed_trajectory_topics": []' not in text


def test_b2_does_not_create_dynamic_planner_package_dependency_cycle():
    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    # tejen_dynamic_planner already depends on tejen_mission.  B2 may launch
    # the backend from an integrated workspace, but the reverse package edge
    # would make colcon's dependency graph cyclic and unbuildable.
    assert "<exec_depend>tejen_dynamic_planner</exec_depend>" not in package_xml


def _segment_intersects_aabb(start, goal, lower, upper):
    t_min, t_max = 0.0, 1.0
    delta = goal - start
    for axis in range(3):
        if abs(float(delta[axis])) <= 1e-12:
            if not (float(lower[axis]) <= float(start[axis]) <= float(upper[axis])):
                return False
            continue
        t0 = (float(lower[axis]) - float(start[axis])) / float(delta[axis])
        t1 = (float(upper[axis]) - float(start[axis])) / float(delta[axis])
        if t0 > t1:
            t0, t1 = t1, t0
        t_min = max(t_min, t0)
        t_max = min(t_max, t1)
        if t_min > t_max:
            return False
    return True


def test_b2_obstacle_blocks_direct_corridor_without_occupying_start_or_capture_goal():
    """Regression for the 21:11 impossible B2 obstacle commissioning scene.

    Include the commissioned body half extents and the ±0.10 m execution
    tracking certificate.  The obstacle must block the nominal direct body
    corridor while leaving both endpoints outside that inflated body C-space.
    """
    text = LAUNCH.read_text(encoding="utf-8")
    expected_literals = {
        '"obstacle_center_x_m": 0.65': 0.65,
        '"obstacle_center_y_m": 0.0': 0.0,
        '"obstacle_center_z_m": 0.95': 0.95,
        '"obstacle_half_x_m": 0.07': 0.07,
        '"obstacle_half_y_m": 0.12': 0.12,
        '"obstacle_half_z_m": 0.10': 0.10,
    }
    for literal in expected_literals:
        assert literal in text

    centre = np.array([0.65, 0.0, 0.95])
    physical_half = np.array([0.07, 0.12, 0.10])
    body_half = np.array([0.105, 0.105, 0.060])
    tracking_half = np.array([0.10, 0.10, 0.10])
    inflated_half = physical_half + body_half + tracking_half

    # Ground-start X is 0.69997222 m; M2 takeoff adds 0.50 m to body z=0.105 m.
    start = np.array([0.69997222, 0.0, 0.605])
    goal = np.array([0.25, 0.0, 0.85])
    lower, upper = centre - inflated_half, centre + inflated_half

    assert not np.all((start >= lower) & (start <= upper))
    assert not np.all((goal >= lower) & (goal <= upper))
    assert _segment_intersects_aabb(start, goal, lower, upper)

    # Keep useful endpoint margin so normal settle/tracking noise cannot put the
    # stationary endpoint back inside the obstacle certificate.
    assert centre[2] - inflated_half[2] - start[2] >= 0.08
    assert centre[0] - inflated_half[0] - goal[0] >= 0.10


def test_b2_runner_hold_observations_use_elapsed_wall_time_not_loop_count():
    text = B1_RUNNER.read_text(encoding="utf-8")
    assert 'for _ in $(seq 1 $((HOLD_SECONDS * 10)))' not in text
    assert 'for _ in $(seq 1 $((LATCH_STABILITY_OBSERVE_SECONDS * 10)))' not in text
    assert 'hold_observation_started_ns="$(date +%s%N)"' in text
    assert 'hold_observation_duration_ns=$((HOLD_SECONDS * 1000000000))' in text
    assert '$(date +%s%N) - hold_observation_started_ns < hold_observation_duration_ns' in text
    assert 'latch_observation_started_ns="$(date +%s%N)"' in text
    assert 'latch_observation_duration_ns=$((LATCH_STABILITY_OBSERVE_SECONDS * 1000000000))' in text
    assert '$(date +%s%N) - latch_observation_started_ns < latch_observation_duration_ns' in text
