from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
DRONE = ROOT / "src/tejen_mission"
CONFIG = DRONE / "config/irl_commissioning.yaml"
MOCAP = DRONE / "tejen_mission/motion_capture_publisher_irl.py"
MAGNET = DRONE / "tejen_mission/magnet_attachment_manager_irl.py"
LAUNCH = DRONE / "launch/m1_irl_virtual_ring.launch.py"
BACKEND_LAUNCH = ROOT / "src/tejen_dynamic_planner/launch/c1f_irl_m1_virtual_ring_backend.launch.py"
BACKEND_CONFIG = ROOT / "src/tejen_dynamic_planner/config/c1f_irl_m1_virtual_ring.yaml"
RUNNER = ROOT / "tools/sim_test/run_m1_irl_virtual_ring.sh"
PLANNER = DRONE / "tejen_mission/online_join_planner.py"


def _yaml():
    return yaml.safe_load(CONFIG.read_text())


def test_locked_irl_rigid_body_ids_and_live_pickup_topic():
    data = _yaml()
    mocap = data["motion_capture_publisher_irl"]["ros__parameters"]
    assert mocap["pickup_rigid_body_id"] == "6"
    assert mocap["quad_rigid_body_id"] == "7"
    assert mocap["magnet_rigid_body_id"] == "8"
    assert mocap["ring_rigid_body_id"] == "9"
    assert mocap["pickup_object_pose_topic"] == "/irl/pickup_object/pose"


def test_mocap_publisher_routes_pickup_body_to_pose_array():
    source = MOCAP.read_text()
    assert "pickup_rigid_body_id" in source
    assert "ring_rigid_body_id" in source
    assert "pickup_object_pose_topic" in source
    assert "PoseArray, self.pickup_object_pose_topic, 10" in source
    assert "pickup_object_publisher.publish" in source
    assert "pickupParseData" in source


def test_magnet_manager_uses_live_pickup_pose_and_fails_closed_without_it():
    source = MAGNET.read_text()
    assert "pickup_object_pose_topic" in source
    assert "pickup_object_index" in source
    assert "PoseArray, self.pickup_object_pose_topic, self.pickup_cb, 10" in source
    assert "pickup_fresh" in source
    assert "relative_speed" in source
    assert "static_pickup_x" not in source
    assert "self.tip_contact_position" in source
    assert "self.pickup_contact_position" in source


def test_one_lab_yaml_owns_pickup_geometry_for_planner_and_attachment_manager():
    data = _yaml()
    magnet = data["magnet_attachment_manager_irl"]["ros__parameters"]
    planner = data["online_join_planner"]["ros__parameters"]

    assert planner["use_static_pickup_object"] is False
    assert planner["pickup_object_pose_topic"] == "/irl/pickup_object/pose"
    assert planner["pickup_object_index"] == 0
    assert planner["drop_point_source"] == "payload"
    assert planner["payload_pose_topic"] == "/fake_payload/pose"
    assert planner["payload_twist_topic"] == "/fake_payload/twist"
    assert planner["commissioning_hold_after_lift"] is False
    assert planner["commissioning_hold_before_drop"] is False
    assert planner["magnet_drop_below_quad"] == 0.54
    assert planner["loaded_lift_z_tolerance"] == 0.05

    assert magnet["pickup_object_pose_topic"] == planner["pickup_object_pose_topic"]
    assert magnet["pickup_object_index"] == planner["pickup_object_index"]
    for axis in "xyz":
        assert magnet[f"pickup_point_offset_{axis}"] == planner[f"payload_pickup_point_{axis}"]
        assert magnet[f"magnet_marker_to_contact_face_{axis}"] == planner[f"magnet_marker_to_contact_face_{axis}"]


def test_virtual_ring_is_moving_and_fake_companions_are_disabled_for_irl_m1():
    data = _yaml()
    fake = data["fake_cooperative_transport_world"]["ros__parameters"]
    assert fake["trajectory_type"] in {"circle", "line", "figure8"}
    assert fake["enable_fake_obstacles"] is False
    assert fake["fake_drone_count"] == 0
    assert fake["payload_committed_trajectory_topic"] == "/fake_payload/committed_trajectory"

    backend = yaml.safe_load(BACKEND_CONFIG.read_text())["dynamic_planner_transfer_backend"]["ros__parameters"]
    assert backend["use_sim_time"] is False
    assert backend["cooperative_scene_enabled"] is False
    assert backend["moving_basket_scene_enabled"] is True
    assert backend["moving_basket_committed_trajectory_topic"] == "/fake_payload/committed_trajectory"
    assert backend["target_predictor_type"] == "committed_trajectory"
    assert backend["spline_time_factor"] == 3.0


def test_controller_settings_explicitly_select_xy_bias_mode_and_thrust_feedback():
    controller = _yaml()["tejen_mpc"]["ros__parameters"]
    assert controller["use_external_reference"] is True
    assert controller["xy_bias_mode"] == "legacy_integral"
    assert controller["enable_xy_integral_action"] is True
    assert controller["lateral_disturbance_observer_bandwidth_rad_s"] == 0.30
    assert controller["enable_thrust_ratio_ukf"] is True


def test_single_runner_owns_full_irl_stack_without_pipefail():
    launch = LAUNCH.read_text()
    assert "irl_state_pipeline.launch.py" in launch
    assert "fake_cooperative_transport_world" in launch
    assert "c1f_irl_m1_virtual_ring_backend.launch.py" in launch
    assert "online_join_planner" in launch
    assert "tejen_mpc" in launch
    assert "elrs_interface_irl" in launch
    assert "view_frame.launch.py" in launch

    backend_launch = BACKEND_LAUNCH.read_text()
    assert "c1f_irl_m1_virtual_ring.yaml" in backend_launch
    assert "irl_commissioning.yaml" in backend_launch

    runner = RUNNER.read_text()
    assert "m1_irl_operator" in runner
    assert "lab_config_ready" in runner
    assert "ros2 launch tejen_mission m1_irl_virtual_ring.launch.py" in runner
    assert "xy_bias_mode" in runner
    assert "lateral_disturbance" in runner
    assert "shadow/none are test-only" in runner
    assert "pipefail" not in runner
    assert "set -e" not in runner


def test_measured_irl_pickup_object_geometry_is_consistent_across_m1_stack():
    data = _yaml()
    operator = data["m1_irl_operator"]["ros__parameters"]
    magnet = data["magnet_attachment_manager_irl"]["ros__parameters"]
    planner = data["online_join_planner"]["ros__parameters"]
    backend = data["dynamic_planner_transfer_backend"]["ros__parameters"]

    assert operator["pickup_object_geometry_ready"] is True
    assert operator["pickup_object_mass_kg"] == 0.060
    assert operator["pickup_object_diameter_m"] == 0.086
    assert operator["pickup_object_thickness_m"] == 0.022
    assert operator["pickup_plate_diameter_m"] == 0.052
    assert operator["pickup_plate_recess_from_top_m"] == 0.002
    assert operator["pickup_marker_centroid_above_top_m"] == 0.006
    assert operator["pickup_reported_position_below_marker_centroid_m"] == 0.020

    # Reported position: 20 mm below marker centroid, marker centroid 6 mm
    # above top -> reported origin is 14 mm below top.
    assert planner["payload_geometry_origin_from_pose_x"] == 0.0
    assert planner["payload_geometry_origin_from_pose_y"] == 0.0
    assert planner["payload_geometry_origin_from_pose_z"] == 0.003

    # Steel surface is 2 mm below top -> +12 mm from reported origin.
    assert planner["payload_pickup_point_z"] == 0.012
    assert magnet["pickup_point_offset_z"] == 0.012

    # Conservative square box around the roughly circular 86 mm object.
    assert planner["payload_collision_size_x"] == 0.086
    assert planner["payload_collision_size_y"] == 0.086
    assert planner["payload_collision_size_z"] == 0.022
    assert backend["payload_half_x_m"] == 0.043
    assert backend["payload_half_y_m"] == 0.043
    assert backend["payload_half_z_m"] == 0.011

    # Marker -> contact = -30 mm; contact -> object centre = -9 mm.
    assert backend["payload_center_from_magnet_x_m"] == 0.0
    assert backend["payload_center_from_magnet_y_m"] == 0.0
    assert backend["payload_center_from_magnet_z_m"] == -0.039

    # Geometry is ready, but flight gate remains intentionally closed until the
    # moving virtual target placement / final cage setup are reviewed.
    assert operator["lab_config_ready"] is False


def test_loaded_lift_uses_dedicated_strict_handoff_tolerance():
    planner = _yaml()["online_join_planner"]["ros__parameters"]

    # Keep the broad pickup/approach commissioning tolerance independent from
    # the stronger physical-completion contract used before C++ authority prep.
    assert planner["pickup_z_tolerance"] == 0.20
    assert planner["loaded_lift_z_tolerance"] == 0.05
    assert planner["loaded_lift_z_tolerance"] < planner["pickup_z_tolerance"]


def test_loaded_lift_runtime_evidence_is_explicit_in_csv_log():
    source = PLANNER.read_text()
    fields = (
        "magnet_drop_below_quad_m",
        "pickup_object_x",
        "pickup_object_y",
        "pickup_object_z",
        "pickup_object_pose_age_s",
        "lift_object_clearance_m",
        "loaded_lift_z_tolerance_m",
        "loaded_lift_quad_z_error_m",
        "loaded_lift_profile_complete",
        "loaded_lift_object_airborne",
        "loaded_lift_ready_for_cpp",
    )
    # Each field must exist both in DictWriter.fieldnames and the row mapping.
    for field in fields:
        assert source.count(f'"{field}"') >= 2



def test_loaded_lift_cpp_gate_uses_quad_z_error_not_magnet_tip_error():
    source = PLANNER.read_text()
    assert "def loaded_lift_quad_z_error_m(self)" in source
    assert "abs_quad_z_error_m=abs(loaded_lift_quad_z_error_m)" in source

    # Pickup/contact tracking still uses measured magnet-tip geometry; only the
    # loaded-lift completion gate changes coordinates.
    assert "xy_error, relative_speed, z_error = self.tracking_errors()" in source

def test_pickup_geometry_helper_rotates_local_offsets():
    from tejen_mission.irl_pickup_geometry import transform_local_point

    # 90 deg yaw about +Z: local +X becomes world +Y.
    q = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])  # wxyz
    p = transform_local_point(
        np.array([1.0, 2.0, 3.0]),
        q,
        np.array([0.2, 0.0, 0.0]),
    )
    np.testing.assert_allclose(p, [1.0, 2.2, 3.0], atol=1e-9)
