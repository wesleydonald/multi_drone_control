import importlib.util
import math
import subprocess
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "run_sim_test.py"
SPEC = importlib.util.spec_from_file_location("run_sim_test", MODULE_PATH)
runner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(runner)


def test_c1f6_scenario_schema_loads():
    path, scenario = runner.load_scenario("c1f6")
    assert path.is_file()
    assert scenario["schema_version"] == 1
    assert scenario["simulation_only"] is True
    assert scenario["success"]["phase"] == "ATTACH_READY"


def test_owned_process_group_cleanup_does_not_stop_unrelated_process():
    owned = subprocess.Popen(["sleep", "30"], start_new_session=True)
    unrelated = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        runner.stop_owned_process_group(owned.pid, 0.5, 0.5)
        owned.wait(timeout=2.0)
        assert unrelated.poll() is None
    finally:
        if owned.poll() is None:
            owned.terminate()
        if unrelated.poll() is None:
            unrelated.terminate()
        for process in (owned, unrelated):
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()


def test_fallback_result_does_not_overwrite_supervisor_result(tmp_path):
    result = tmp_path / "supervisor" / "result.json"
    result.parent.mkdir()
    result.write_text('{"reason": "authoritative"}\n')
    runner.write_fallback_result(tmp_path, "FAIL", "infrastructure", "fallback", 2)
    assert "authoritative" in result.read_text()


def test_simulation_environment_removes_snap_gui_paths_and_adds_models():
    environment = runner.simulation_environment({
        "SNAP": "/snap/code/current",
        "GTK_PATH": "/snap/code/current/gtk",
        "XDG_DATA_DIRS": "/snap/code/current/share",
        "XDG_DATA_DIRS_VSCODE_SNAP_ORIG": "/usr/local/share:/usr/share",
        "GZ_SIM_RESOURCE_PATH": "/existing/models",
    })
    assert "SNAP" not in environment
    assert "GTK_PATH" not in environment
    assert environment["XDG_DATA_DIRS"] == "/usr/local/share:/usr/share"
    assert environment["GZ_SIM_RESOURCE_PATH"].startswith(str(runner.ROOT / "simulation_assets"))
    assert environment["GZ_SIM_RESOURCE_PATH"].endswith(":/existing/models")


def test_analyze_parser_supports_focused_drilldown():
    args = runner.build_parser().parse_args([
        "analyze",
        "--scenario",
        "c1f6",
        "--detailed",
        "--focus-phase",
        "TRANSIT_TO_REATTACH",
    ])
    assert args.detailed is True
    assert args.focus_phase == "TRANSIT_TO_REATTACH"
    assert args.focus_time is None


def test_analyze_focus_modes_are_mutually_exclusive():
    parser = runner.build_parser()
    try:
        parser.parse_args([
            "analyze", "--focus-phase", "TAKEOFF", "--focus-time", "2.0"
        ])
    except SystemExit as exc:
        assert exc.code == 2
    else:
        raise AssertionError("focus arguments should be mutually exclusive")


def test_m1_case_registry_lists_commissioned_cases():
    assert runner.list_cases() == [
        "baseline",
        "blocking_center",
        "clear",
        "figure8",
        "mixed",
        "narrow_gap",
        "static_two",
        "three_attached",
        "three_attached_yawing",
    ]


def test_baseline_case_loads_environment_only_parameters():
    path, case = runner.load_case("baseline")
    assert path.is_file()
    assert set(case) == {
        "fake_cooperative_transport_world",
        "online_join_planner",
    }
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    join = case["online_join_planner"]["ros__parameters"]
    assert fake["trajectory_type"] == "circle"
    assert fake["obstacle_scenario"] == "custom"
    assert fake["fake_drone_offsets_y"] == [0.60, -0.60]
    assert join["static_obstacle_count"] == 0


def test_three_attached_case_is_environment_only_m2_preview():
    path, case = runner.load_case("three_attached")
    assert path.is_file()
    assert set(case) == {
        "fake_cooperative_transport_world",
        "online_join_planner",
    }
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    join = case["online_join_planner"]["ros__parameters"]
    assert fake["trajectory_type"] == "circle"
    assert fake["obstacle_scenario"] == "three_attached"
    assert fake["fake_drone_count"] == 3
    assert fake["three_attached_cable_length_m"] == 0.50
    assert fake["three_attached_cable_angle_deg"] == 45.0
    assert join["static_obstacle_count"] == 0


def test_three_attached_yawing_case_is_three_attached_plus_yaw_only():
    _, base = runner.load_case("three_attached")
    path, case = runner.load_case("three_attached_yawing")
    assert path.is_file()
    fake = case["fake_cooperative_transport_world"]["ros__parameters"]
    base_fake = base["fake_cooperative_transport_world"]["ros__parameters"]
    join = case["online_join_planner"]["ros__parameters"]

    assert fake["trajectory_type"] == base_fake["trajectory_type"] == "circle"
    assert fake["obstacle_scenario"] == base_fake["obstacle_scenario"] == "three_attached"
    assert fake["fake_drone_count"] == base_fake["fake_drone_count"] == 3
    assert fake["three_attached_cable_length_m"] == base_fake["three_attached_cable_length_m"] == 0.50
    assert fake["three_attached_cable_angle_deg"] == base_fake["three_attached_cable_angle_deg"] == 45.0
    assert fake["payload_yaw_mode"] == "spin"
    assert fake["payload_yaw0"] == 0.0
    assert math.isclose(fake["payload_yaw_rate"], 0.20)
    assert join["static_obstacle_count"] == 0


def test_three_attached_yawing_launch_keeps_three_cpp_cooperative_topics(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("three_attached_yawing")
    command = runner.build_launch_command(
        scenario,
        gui=True,
        control_mode="manual",
        result_path=tmp_path / "result.json",
        case_path=case_path,
    )
    assert "cooperative_preview_count:=3" in command


def test_three_attached_launch_requests_three_cpp_cooperative_topics(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("three_attached")
    command = runner.build_launch_command(
        scenario,
        gui=False,
        control_mode="manual",
        result_path=tmp_path / "result.json",
        case_path=case_path,
    )
    assert "cooperative_preview_count:=3" in command


def test_baseline_launch_keeps_two_cpp_cooperative_topics(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("baseline")
    command = runner.build_launch_command(
        scenario,
        gui=False,
        control_mode="manual",
        result_path=tmp_path / "result.json",
        case_path=case_path,
    )
    assert "cooperative_preview_count:=2" in command


def test_baseline_launch_omits_empty_landing_request_argument(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("baseline")
    command = runner.build_launch_command(
        scenario,
        gui=False,
        control_mode="manual",
        result_path=tmp_path / "result.json",
        case_path=case_path,
    )
    assert not any(arg.startswith("supervisor_landing_request_phase:=") for arg in command)


def test_unknown_case_is_rejected_before_launch():
    try:
        runner.load_case("definitely_not_a_case")
    except runner.PipelineError as exc:
        assert "Case does not exist" in str(exc)
    else:
        raise AssertionError("unknown case should have been rejected")


def test_case_name_rejects_path_traversal():
    try:
        runner.load_case("../baseline")
    except runner.PipelineError as exc:
        assert "Invalid case name" in str(exc)
    else:
        raise AssertionError("path traversal should have been rejected")


def test_case_schema_rejects_non_environment_node(tmp_path, monkeypatch):
    case_root = tmp_path / "cases"
    case_root.mkdir()
    (case_root / "bad.yaml").write_text(
        "tejen_mpc:\n  ros__parameters:\n    use_external_reference: false\n"
    )
    monkeypatch.setattr(runner, "CASE_ROOT", case_root)
    try:
        runner.load_case("bad")
    except runner.PipelineError as exc:
        assert "unsupported node" in str(exc).lower()
    else:
        raise AssertionError("case should not be able to change controller parameters")


def test_static_case_requires_complete_positive_radius_obstacles(tmp_path, monkeypatch):
    case_root = tmp_path / "cases"
    case_root.mkdir()
    (case_root / "bad_static.yaml").write_text(
        "fake_cooperative_transport_world:\n"
        "  ros__parameters:\n"
        "    trajectory_type: circle\n"
        "    obstacle_scenario: clear\n"
        "online_join_planner:\n"
        "  ros__parameters:\n"
        "    static_obstacle_count: 1\n"
        "    static_obstacle_0_id: bad\n"
        "    static_obstacle_0_x: 0.3\n"
        "    static_obstacle_0_y: 0.0\n"
        "    static_obstacle_0_z: 1.6\n"
        "    static_obstacle_0_radius: -0.25\n"
    )
    monkeypatch.setattr(runner, "CASE_ROOT", case_root)
    try:
        runner.load_case("bad_static")
    except runner.PipelineError as exc:
        assert "radius" in str(exc).lower()
    else:
        raise AssertionError("negative obstacle radius should have been rejected")


def test_case_launch_command_contains_resolved_case_config():
    scenario_path, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("static_two")
    command = runner.build_launch_command(
        scenario,
        gui=True,
        control_mode="manual",
        result_path=Path("/tmp/result.json"),
        case_path=case_path,
    )
    assert f"case_config:={case_path.resolve()}" in command
    assert "gui:=true" in command
    assert "control_mode:=manual" in command
    # Case selection must not rewrite the supervisor's scenario-level contract.
    assert f"mission_timeout_s:={float(scenario['timeouts']['mission_s'])}" in command
    assert f"success_dwell_s:={float(scenario['timeouts']['success_dwell_s'])}" in command


def test_case_snapshot_is_written_into_evidence_bundle(tmp_path):
    scenario_path, _ = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("mixed")
    runner.snapshot_run_configuration(tmp_path, scenario_path, case_path)
    assert (tmp_path / "configuration" / "scenario.yaml").read_bytes() == scenario_path.read_bytes()
    assert (tmp_path / "configuration" / "case.yaml").read_bytes() == case_path.read_bytes()


def test_run_parser_accepts_case_and_defaults_to_baseline():
    parser = runner.build_parser()
    default_args = parser.parse_args(["run", "--scenario", "c1f6", "--gui", "--manual-control"])
    assert default_args.case == "baseline"
    selected_args = parser.parse_args([
        "run", "--scenario", "c1f6", "--case", "figure8", "--gui", "--manual-control"
    ])
    assert selected_args.case == "figure8"


def test_cases_parser_lists_without_launching():
    args = runner.build_parser().parse_args(["cases"])
    assert args.command == "cases"


def test_xy_disturbance_ab_launch_command_is_explicit(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("baseline")
    command = runner.build_launch_command(
        scenario,
        gui=False,
        control_mode="automatic",
        result_path=tmp_path / "result.json",
        case_path=case_path,
        xy_bias_mode="lateral_disturbance",
        disturbance_force_x_n=-0.25,
        disturbance_force_y_n=0.10,
    )
    assert "xy_bias_mode:=lateral_disturbance" in command
    assert "disturbance_force_x_n:=-0.25" in command
    assert "disturbance_force_y_n:=0.1" in command
    assert "supervisor_success_phase:=ATTACH_READY" in command
    assert "supervisor_failure_phase:=LANDED_DISARMED" in command


def test_landing_check_retargets_supervisor_without_changing_planner(tmp_path):
    _, scenario = runner.load_scenario("c1f6")
    case_path, _ = runner.load_case("baseline")
    command = runner.build_launch_command(
        scenario,
        gui=False,
        control_mode="automatic",
        result_path=tmp_path / "result.json",
        case_path=case_path,
        landing_check_from="APPROACH_ABOVE_PICKUP",
    )
    assert "supervisor_landing_request_phase:=APPROACH_ABOVE_PICKUP" in command
    assert "supervisor_success_phase:=LANDED_DISARMED" in command
    assert "supervisor_failure_phase:=__NO_FAILURE_PHASE__" in command
    assert "supervisor_require_handoff_ready:=false" in command
