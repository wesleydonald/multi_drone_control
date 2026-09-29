import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
BACKEND = ROOT / "src" / "tejen_dynamic_planner" / "src" / "transfer_backend_node.cpp"
FAKE_WORLD = ROOT / "src" / "tejen_mission" / "tejen_mission" / "fake_cooperative_transport_world.py"
RVIZ = ROOT / "src" / "drone_visualisation" / "rviz" / "default.rviz"
C1F6_CONFIG = ROOT / "src" / "tejen_dynamic_planner" / "config" / "c1f6_moving_rendezvous.yaml"


def _marker_display(topic: str):
    config = yaml.safe_load(RVIZ.read_text())
    for display in config["Visualization Manager"]["Displays"]:
        if display.get("Class") != "rviz_default_plugins/MarkerArray":
            continue
        value = display.get("Topic", {}).get("Value")
        if value == topic:
            return display
    raise AssertionError(f"marker display for {topic} not found")



def test_c1f6_simulation_uses_12_degree_swing_without_changing_backend_default():
    config = yaml.safe_load(C1F6_CONFIG.read_text())
    params = config["dynamic_planner_transfer_backend"]["ros__parameters"]
    assert math.isclose(
        params["max_swing_angle_rad"],
        math.radians(12.0),
        rel_tol=0.0,
        abs_tol=1e-12,
    )

    # The generic node default remains 15 degrees. Only the commissioned M1/C1F.6
    # simulation configuration is being reduced until IRL swing data exists.
    backend_text = BACKEND.read_text()
    assert '"max_swing_angle_rad", 15.0 * kPi / 180.0' in backend_text


def test_mixed_case_uses_close_but_nonblocking_companion_geometry():
    case = yaml.safe_load((ROOT / "tools" / "sim_test" / "cases" / "v1" / "mixed.yaml").read_text())
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]

    # mixed is intended as a positive combined stress case. The historical
    # blocking_left preset placed a companion at y=0.25 m and made the final
    # suspended rendezvous structurally infeasible even after the 12-degree change.
    assert fake["obstacle_scenario"] == "custom"
    assert fake["fake_drone_count"] == 2
    assert fake["fake_drone_offsets_x"] == [0.30, -1.20]
    assert fake["fake_drone_offsets_y"] == [0.45, -0.80]
    assert fake["fake_drone_offsets_z"] == [0.50, 0.50]

    planner = yaml.safe_load(C1F6_CONFIG.read_text())[
        "dynamic_planner_transfer_backend"
    ]["ros__parameters"]
    swing = planner["max_swing_angle_rad"]
    magnet_half_y = 0.05 + 0.50 * math.sin(swing)
    terminal_required_y = (
        magnet_half_y
        + planner["cooperative_body_half_y_m"]
        + planner["ego_tracking_half_y_m"]
        + planner["cooperative_tracking_half_y_m"]
    )
    assert fake["fake_drone_offsets_y"][0] > terminal_required_y + 0.03

    join = case["online_join_planner"]["ros__parameters"]
    assert join["static_obstacle_count"] == 1

def test_three_attached_preview_uses_three_tensioned_companions_on_plates_3_6_9():
    case = yaml.safe_load(
        (ROOT / "tools" / "sim_test" / "cases" / "v1" / "three_attached.yaml").read_text()
    )
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    join = case["online_join_planner"]["ros__parameters"]
    assert fake["obstacle_scenario"] == "three_attached"
    assert fake["fake_drone_count"] == 3
    assert math.isclose(fake["three_attached_cable_length_m"], 0.50)
    assert math.isclose(fake["three_attached_cable_angle_deg"], 45.0)
    assert join["static_obstacle_count"] == 0

    source = FAKE_WORLD.read_text()
    assert "(3, 6, 9)" in source
    assert "radially_tensioned_endpoint_body" in source
    # The case carries physical preview inputs, not duplicated XYZ positions.
    assert "fake_drone_offsets_x" not in fake
    assert "fake_drone_offsets_y" not in fake
    assert "fake_drone_offsets_z" not in fake


def test_three_attached_yawing_keeps_standard_visual_language_and_rigid_live_states():
    case = yaml.safe_load(
        (ROOT / "tools" / "sim_test" / "cases" / "v1" / "three_attached_yawing.yaml").read_text()
    )
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    assert fake["trajectory_type"] == "circle"
    assert fake["payload_yaw_mode"] == "spin"
    assert math.isclose(fake["payload_yaw_rate"], 0.20)

    source = FAKE_WORLD.read_text()
    assert "THREE_ATTACHED_COMPANION_PLATES = (3, 6, 9)" in source
    assert "rigid_body_offset_state" in source
    assert "if self.get_obstacle_scenario() == 'three_attached':" in source
    assert "fake_three_attached_preview_cables" not in source
    assert "fake_three_attached_assignment_labels" not in source
    assert "drone_{i} / plate {plate_index}" not in source
    assert "label_marker.text = f'drone_{i}'" in source


def test_three_attached_plumbs_all_three_companions_into_cpp_backend():
    system_launch = (
        ROOT / "src" / "tejen_mission" / "launch" / "c1f6_system_test.launch.py"
    ).read_text()
    backend_launch = (
        ROOT / "src" / "tejen_dynamic_planner" / "launch" / "c1f6_moving_rendezvous_backend.launch.py"
    ).read_text()
    runner = (ROOT / "tools" / "sim_test" / "run_sim_test.py").read_text()

    assert 'DeclareLaunchArgument("cooperative_preview_count", default_value="2")' in system_launch
    assert '"cooperative_preview_count": cooperative_preview_count' in system_launch
    assert "cooperative_preview_count" in backend_launch
    assert "cooperative_drone_state_topics" in backend_launch
    assert "cooperative_committed_trajectory_topics" in backend_launch
    assert "cooperative_preview_count" in runner


def test_narrow_gap_is_tight_but_not_structurally_impossible():
    text = FAKE_WORLD.read_text()
    assert "attach_y + 0.47" in text
    assert "attach_y - 0.47" in text
    assert "attach_y + 0.42" not in text
    assert "attach_y - 0.42" not in text




def test_narrow_gap_case_snapshots_explicit_047_geometry():
    case = yaml.safe_load((ROOT / "tools" / "sim_test" / "cases" / "v1" / "narrow_gap.yaml").read_text())
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    assert fake["obstacle_scenario"] == "custom"
    assert fake["fake_drone_count"] == 2
    assert fake["fake_drone_offsets_x"] == [0.30, 0.30]
    assert fake["fake_drone_offsets_y"] == [0.47, -0.47]
    assert fake["fake_drone_offsets_z"] == [0.50, 0.50]


def test_backend_separates_physical_and_collision_assembly_namespaces():
    text = BACKEND.read_text()
    required = {
        '"ego_physical_body"',
        '"ego_tracking_body"',
        '"ego_physical_cable"',
        '"ego_collision_cable"',
        '"ego_tracking_cable"',
        '"ego_physical_magnet"',
        '"ego_collision_magnet"',
        '"ego_physical_payload"',
        '"ego_collision_payload"',
    }
    missing = sorted(token for token in required if token not in text)
    assert not missing, f"missing visualization namespaces: {missing}"


def test_cable_collision_visual_is_a_surface_not_only_vertex_points():
    text = BACKEND.read_text()
    assert "Marker::TRIANGLE_LIST" in text
    assert "ego_collision_cable" in text
    assert '"assembly_cable"' not in text


def test_cooperative_planner_visuals_keep_body_and_tracking_distinct():
    text = BACKEND.read_text()
    assert '"cooperative_physical"' in text
    assert '"cooperative_tracking_tube"' in text
    assert '"cooperative_shared_preview"' in text


def test_fake_world_names_duplicate_drone_visuals_as_ground_truth_legacy_only():
    text = FAKE_WORLD.read_text()
    assert "fake_obstacle_ground_truth" in text
    assert "fake_obstacle_legacy_visual_safety_radius" in text
    assert "'fake_obstacle_drone'" not in text
    assert "'fake_obstacle_safety_radius'" not in text


def test_default_rviz_has_clear_display_group_names():
    fake = _marker_display("/fake_world/markers")
    join = _marker_display("/join_planner/markers")
    cpp = _marker_display("/dynamic_planner/markers")
    assert fake["Name"] == "Fake World Ground Truth"
    assert join["Name"] == "Mission / Join Planner"
    assert cpp["Name"] == "C++ Planner / Collision Model"


def test_default_rviz_hides_duplicate_fake_drone_visuals_and_shows_ring():
    fake = _marker_display("/fake_world/markers")
    namespaces = fake["Namespaces"]
    assert namespaces["fake_obstacle_ground_truth"] is False
    assert namespaces["fake_obstacle_legacy_visual_safety_radius"] is False
    assert namespaces["fake_obstacle_label"] is True
    assert namespaces["fake_ring_truss"] is True
    assert namespaces["fake_ring_plates"] is True
    assert namespaces["fake_ring_net"] is True
    assert "fake_payload" not in namespaces
    assert "fake_payload_legacy_visual_margin" not in namespaces


def test_default_rviz_enables_current_cpp_geometry_namespaces():
    cpp = _marker_display("/dynamic_planner/markers")
    namespaces = cpp["Namespaces"]
    for name in (
        "ego_physical_body",
        "ego_tracking_body",
        "ego_physical_cable",
        "ego_collision_cable",
        "ego_tracking_cable",
        "ego_physical_magnet",
        "ego_collision_magnet",
        "cooperative_physical",
        "cooperative_tracking_tube",
        "cooperative_shared_preview",
        "cpp_reference",
        "moving_target",
        "planning_goal",
        "splice_point",
        "local_goal",
    ):
        assert namespaces[name] is True, name
    assert "assembly_body" not in namespaces
    assert "assembly_cable" not in namespaces
