#!/usr/bin/env python3
"""Launch, supervise, collect, and analyse bounded simulation system tests."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCENARIO_ROOT = ROOT / "tools" / "sim_test" / "scenarios" / "v1"
CASE_ROOT = ROOT / "tools" / "sim_test" / "cases" / "v1"

CASE_NODE_KEYS = {
    "fake_cooperative_transport_world",
    "online_join_planner",
}
FAKE_WORLD_CASE_PARAMETERS = {
    "trajectory_type",
    "obstacle_scenario",
    "center_x",
    "center_y",
    "center_z",
    "radius",
    "line_amplitude",
    "omega",
    "z_amplitude",
    "z_omega",
    "payload_yaw_mode",
    "payload_yaw0",
    "payload_yaw_rate",
    "payload_yaw_amplitude",
    "payload_yaw_omega",
    "scenario_drone_offset_z",
    "three_attached_cable_length_m",
    "three_attached_cable_angle_deg",
    "fake_drone_count",
    "fake_drone_offsets_x",
    "fake_drone_offsets_y",
    "fake_drone_offsets_z",
}
CAPTURE = ROOT / "tools" / "c1f_experiment" / "experiment_capture.py"
ANALYSER = ROOT / "analysis" / "sim_test" / "analyse_sim_test.py"
ACTIVE_CAPTURE = ROOT / "logs" / "c1f_experiments" / ".active_run.json"

SNAP_GUI_ENVIRONMENT_KEYS = {
    "GDK_PIXBUF_MODULEDIR",
    "GDK_PIXBUF_MODULE_FILE",
    "GIO_LAUNCHED_DESKTOP_FILE",
    "GIO_MODULE_DIR",
    "GSETTINGS_SCHEMA_DIR",
    "GTK_EXE_PREFIX",
    "GTK_IM_MODULE_FILE",
    "GTK_PATH",
    "LOCPATH",
    "SNAP",
    "SNAP_ARCH",
    "SNAP_COMMON",
    "SNAP_CONTEXT",
    "SNAP_COOKIE",
    "SNAP_DATA",
    "SNAP_EUID",
    "SNAP_INSTANCE_NAME",
    "SNAP_LAUNCHER_ARCH_TRIPLET",
    "SNAP_LIBRARY_PATH",
    "SNAP_NAME",
    "SNAP_REAL_HOME",
    "SNAP_REVISION",
    "SNAP_UID",
    "SNAP_USER_COMMON",
    "SNAP_USER_DATA",
    "SNAP_VERSION",
}


class PipelineError(RuntimeError):
    pass


def simulation_environment(base: Optional[dict[str, str]] = None) -> dict[str, str]:
    """Return a ROS environment isolated from editor Snap GUI libraries."""
    environment = dict(os.environ if base is None else base)
    for key in SNAP_GUI_ENVIRONMENT_KEYS:
        environment.pop(key, None)
    original_xdg = environment.get("XDG_DATA_DIRS_VSCODE_SNAP_ORIG")
    if original_xdg:
        environment["XDG_DATA_DIRS"] = original_xdg
    environment.pop("XDG_DATA_HOME", None)
    resource_path = str(ROOT / "simulation_assets" / "tejen")
    for key in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        existing = environment.get(key, "")
        environment[key] = resource_path if not existing else f"{resource_path}:{existing}"
    return environment


def load_scenario(name: str) -> tuple[Path, dict]:
    safe = "".join(ch for ch in name.lower() if ch.isalnum() or ch in "_-")
    if safe != name.lower() or not safe:
        raise PipelineError(f"Invalid scenario name: {name}")
    path = SCENARIO_ROOT / f"{safe}.yaml"
    if not path.is_file():
        raise PipelineError(f"Scenario does not exist: {path}")
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise PipelineError(f"Unsupported scenario schema: {path}")
    for key in ("scenario", "iteration", "simulation_only", "launch", "timeouts"):
        if key not in data:
            raise PipelineError(f"Scenario is missing required key: {key}")
    return path, data


def list_cases() -> list[str]:
    if not CASE_ROOT.is_dir():
        return []
    return sorted(path.stem for path in CASE_ROOT.glob("*.yaml") if path.is_file())


def _validate_case_node(node_name: str, node_data: object, path: Path) -> None:
    if not isinstance(node_data, dict) or set(node_data) != {"ros__parameters"}:
        raise PipelineError(
            f"Case node {node_name} must contain only ros__parameters: {path}"
        )
    parameters = node_data["ros__parameters"]
    if not isinstance(parameters, dict):
        raise PipelineError(f"Case node {node_name} ros__parameters must be a mapping: {path}")

    if node_name == "fake_cooperative_transport_world":
        unsupported = sorted(set(parameters) - FAKE_WORLD_CASE_PARAMETERS)
        if unsupported:
            raise PipelineError(
                "Case fake-world parameters are environment-only; unsupported parameters: "
                + ", ".join(unsupported)
            )
        trajectory_type = str(parameters.get("trajectory_type", "circle")).strip().lower()
        if trajectory_type not in {"stationary", "circle", "line", "figure8"}:
            raise PipelineError(f"Unsupported target trajectory_type={trajectory_type!r}: {path}")
        obstacle_scenario = str(parameters.get("obstacle_scenario", "custom")).strip().lower()
        if obstacle_scenario not in {
            "custom", "clear", "transport_default", "blocking_center",
            "blocking_left", "blocking_right", "narrow_gap", "three_attached",
        }:
            raise PipelineError(f"Unsupported obstacle_scenario={obstacle_scenario!r}: {path}")
        numeric_keys = (
            "center_x", "center_y", "center_z", "radius", "line_amplitude", "omega",
            "z_amplitude", "z_omega", "payload_yaw0", "payload_yaw_rate",
            "payload_yaw_amplitude", "payload_yaw_omega", "scenario_drone_offset_z",
            "three_attached_cable_length_m", "three_attached_cable_angle_deg",
        )
        for key in numeric_keys:
            if key in parameters and not math.isfinite(float(parameters[key])):
                raise PipelineError(f"{key} must be finite: {path}")
        if obstacle_scenario == "three_attached":
            count = int(parameters.get("fake_drone_count", 3))
            if count != 3:
                raise PipelineError(
                    f"three_attached requires exactly three companion drones, got {count}: {path}"
                )
            cable_length = float(parameters.get("three_attached_cable_length_m", 0.50))
            cable_angle_deg = float(parameters.get("three_attached_cable_angle_deg", 45.0))
            if cable_length <= 0.0:
                raise PipelineError(f"three_attached cable length must be positive: {path}")
            if cable_angle_deg < 0.0 or cable_angle_deg > 90.0:
                raise PipelineError(f"three_attached cable angle must be in [0, 90] deg: {path}")

        if obstacle_scenario == "custom":
            count = int(parameters.get("fake_drone_count", 2))
            if count < 0:
                raise PipelineError(f"fake_drone_count must be non-negative: {path}")
            for key in ("fake_drone_offsets_x", "fake_drone_offsets_y", "fake_drone_offsets_z"):
                values = parameters.get(key)
                if count > 0 and (not isinstance(values, list) or len(values) < count):
                    raise PipelineError(
                        f"{key} must provide at least fake_drone_count={count} entries: {path}"
                    )
                if values is not None:
                    for value in values:
                        if not math.isfinite(float(value)):
                            raise PipelineError(f"{key} values must be finite: {path}")
        return

    if node_name == "online_join_planner":
        unsupported = []
        for key in parameters:
            if key == "static_obstacle_count":
                continue
            if not key.startswith("static_obstacle_"):
                unsupported.append(key)
        if unsupported:
            raise PipelineError(
                "Case join-planner parameters are limited to static obstacles; unsupported: "
                + ", ".join(sorted(unsupported))
            )
        count = int(parameters.get("static_obstacle_count", 0))
        if count < 0:
            raise PipelineError(f"static_obstacle_count must be non-negative: {path}")
        required_suffixes = ("id", "x", "y", "z", "radius")
        for index in range(count):
            prefix = f"static_obstacle_{index}_"
            missing = [suffix for suffix in required_suffixes if prefix + suffix not in parameters]
            if missing:
                raise PipelineError(
                    f"static obstacle {index} is missing {', '.join(missing)}: {path}"
                )
            radius = float(parameters[prefix + "radius"])
            if not math.isfinite(radius) or radius <= 0.0:
                raise PipelineError(
                    f"static obstacle {index} radius must be positive, got {radius}: {path}"
                )
            for suffix in ("x", "y", "z"):
                value = float(parameters[prefix + suffix])
                if not math.isfinite(value):
                    raise PipelineError(
                        f"static obstacle {index} {suffix} must be finite: {path}"
                    )
        for key in parameters:
            if key == "static_obstacle_count":
                continue
            parts = key.split("_")
            if len(parts) >= 4 and parts[0:2] == ["static", "obstacle"]:
                try:
                    index = int(parts[2])
                except ValueError as exc:
                    raise PipelineError(f"Malformed static obstacle parameter {key}: {path}") from exc
                if index >= count:
                    raise PipelineError(
                        f"Static obstacle parameter {key} is outside static_obstacle_count={count}: {path}"
                    )
        return

    raise PipelineError(f"Case contains unsupported node {node_name}: {path}")


def load_case(name: str) -> tuple[Path, dict]:
    safe = "".join(ch for ch in name.lower() if ch.isalnum() or ch in "_-")
    if safe != name.lower() or not safe:
        raise PipelineError(f"Invalid case name: {name}")
    path = CASE_ROOT / f"{safe}.yaml"
    if not path.is_file():
        available = ", ".join(list_cases()) or "none"
        raise PipelineError(f"Case does not exist: {path} (available: {available})")
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not data:
        raise PipelineError(f"Case must be a non-empty ROS parameter mapping: {path}")
    unsupported_nodes = sorted(set(data) - CASE_NODE_KEYS)
    if unsupported_nodes:
        raise PipelineError(
            "Case contains unsupported node(s): " + ", ".join(unsupported_nodes)
        )
    missing_nodes = sorted(CASE_NODE_KEYS - set(data))
    if missing_nodes:
        raise PipelineError(
            "Case is missing required environment node(s): " + ", ".join(missing_nodes)
        )
    for node_name, node_data in data.items():
        _validate_case_node(node_name, node_data, path)
    return path, data


def snapshot_run_configuration(run_dir: Path, scenario_path: Path, case_path: Path) -> None:
    config_dir = run_dir / "configuration"
    config_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(scenario_path, config_dir / "scenario.yaml")
    shutil.copy2(case_path, config_dir / "case.yaml")


def cooperative_preview_count_for_case(case_path: Path) -> int:
    data = yaml.safe_load(case_path.read_text())
    try:
        parameters = data["fake_cooperative_transport_world"]["ros__parameters"]
    except (TypeError, KeyError) as exc:
        raise PipelineError(f"Case is missing fake-world parameters: {case_path}") from exc
    count = int(parameters.get("fake_drone_count", 2))
    if count <= 0:
        raise PipelineError(
            f"C1F.6 cooperative-safe simulation requires at least one companion: {case_path}"
        )
    return count


def build_launch_command(
    scenario: dict,
    *,
    gui: bool,
    control_mode: str,
    result_path: Path,
    case_path: Path,
    xy_bias_mode: str = "legacy_integral",
    disturbance_force_x_n: float = 0.0,
    disturbance_force_y_n: float = 0.0,
    landing_check_from: str = "",
) -> list[str]:
    timeouts = scenario["timeouts"]
    launch = scenario["launch"]
    landing_check_from = str(landing_check_from).strip()
    if landing_check_from:
        supervisor_success_phase = "LANDED_DISARMED"
        supervisor_failure_phase = "__NO_FAILURE_PHASE__"
        supervisor_require_handoff_ready = "false"
    else:
        supervisor_success_phase = str(scenario["success"]["phase"])
        failure_phases = list(scenario.get("failure_phases", []))
        supervisor_failure_phase = (
            str(failure_phases[0]) if failure_phases else "__NO_FAILURE_PHASE__"
        )
        supervisor_require_handoff_ready = (
            "true" if bool(scenario["success"].get("require_handoff_ready", True)) else "false"
        )
    command = [
        "ros2",
        "launch",
        str(launch["package"]),
        str(launch["file"]),
        f"gui:={'true' if gui else 'false'}",
        f"control_mode:={control_mode}",
        f"case_config:={case_path.resolve()}",
        f"cooperative_preview_count:={cooperative_preview_count_for_case(case_path)}",
        f"xy_bias_mode:={xy_bias_mode}",
        f"disturbance_force_x_n:={float(disturbance_force_x_n)}",
        f"disturbance_force_y_n:={float(disturbance_force_y_n)}",
        f"supervisor_success_phase:={supervisor_success_phase}",
        f"supervisor_failure_phase:={supervisor_failure_phase}",
        f"supervisor_require_handoff_ready:={supervisor_require_handoff_ready}",
    ]
    if landing_check_from:
        command.append(f"supervisor_landing_request_phase:={landing_check_from}")
    command.extend([
        f"result_path:={result_path}",
        f"startup_timeout_s:={float(timeouts['startup_s'])}",
        f"manual_start_timeout_s:={float(timeouts['manual_start_s'])}",
        f"mission_timeout_s:={float(timeouts['mission_s'])}",
        f"success_dwell_s:={float(timeouts['success_dwell_s'])}",
        f"freshness_timeout_s:={float(timeouts['freshness_s'])}",
        f"planner_status_timeout_s:={float(timeouts['planner_status_freshness_s'])}",
    ])
    return command


def command_output(command: list[str], timeout: float = 5.0) -> str:
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if completed.returncode != 0:
        raise PipelineError(f"Command failed ({' '.join(command)}):\n{completed.stdout}")
    return completed.stdout


def conflicting_processes(tokens: list[str]) -> list[str]:
    output = command_output(["ps", "-eo", "pid=,args="], timeout=3.0)
    conflicts = []
    own_pid = os.getpid()
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == own_pid:
            continue
        if any(token in command for token in tokens):
            conflicts.append(stripped)
    return conflicts


def preflight(scenario: dict, *, gui: bool, check_conflicts: bool = True) -> list[str]:
    checks: list[str] = []
    for executable in ("ros2", "gz"):
        location = shutil.which(executable)
        if not location:
            raise PipelineError(f"Required executable is not on PATH: {executable}")
        checks.append(f"{executable}: {location}")
    if gui:
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            raise PipelineError("GUI mode requires DISPLAY or WAYLAND_DISPLAY")
        if not shutil.which("rviz2"):
            raise PipelineError("GUI mode requires rviz2 on PATH")
        checks.append("GUI display environment available")
    if not (ROOT / "simulation_assets" / "tejen" / "world_drone_env_detach.sdf").is_file():
        raise PipelineError("Detachable-payload Gazebo world is missing")

    required_packages = (
        "tejen_mission",
        "tejen_dynamic_planner",
        "simulation_communication",
        "tejen_mpc",
    )
    if gui:
        required_packages += ("drone_visualisation",)
    for package in required_packages:
        prefix = command_output(["ros2", "pkg", "prefix", package], timeout=5.0).strip()
        checks.append(f"{package}: {prefix}")

    launch = scenario["launch"]
    launch_path = (
        ROOT
        / "install"
        / str(launch["package"])
        / "share"
        / str(launch["package"])
        / "launch"
        / str(launch["file"])
    )
    if not launch_path.is_file():
        raise PipelineError(f"Installed system-test launch file is missing: {launch_path}")
    planner_executable = ROOT / "install/tejen_dynamic_planner/lib/tejen_dynamic_planner/dynamic_planner_transfer_backend"
    if not planner_executable.is_file():
        raise PipelineError(f"Installed dynamic-planner executable is missing: {planner_executable}")
    checks.append("Installed C1F.6 launch, config, and backend are present")

    if ACTIVE_CAPTURE.exists():
        raise PipelineError(
            f"An experiment capture is already active: {ACTIVE_CAPTURE}. Stop it before running."
        )
    if check_conflicts:
        process_conflicts = conflicting_processes(list(scenario.get("conflicting_process_tokens", [])))
        if process_conflicts:
            raise PipelineError("Conflicting processes are active:\n" + "\n".join(process_conflicts))
        try:
            active_nodes = set(
                command_output(
                    ["ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"],
                    timeout=8.0,
                ).splitlines()
            )
        except PipelineError as exc:
            raise PipelineError(f"Could not inspect the ROS graph: {exc}") from exc
        node_conflicts = sorted(active_nodes.intersection(scenario.get("conflicting_nodes", [])))
        if node_conflicts:
            raise PipelineError("Conflicting ROS nodes are active: " + ", ".join(node_conflicts))
        checks.append("No conflicting simulation processes or ROS nodes found")
    return checks


def run_capture_start(
    scenario_path: Path, scenario: dict, mode: str, case_path: Path, case_name: str
) -> Path:
    command = [
        sys.executable,
        str(CAPTURE),
        "start",
        (
            str(scenario["scenario"])
            if case_name == "baseline"
            else f"{scenario['scenario']}_{case_name}"
        ),
        mode,
        "--notes",
        f"Supervised simulation-test pipeline; case={case_name}",
    ]
    for config in scenario.get("configuration_snapshots", []):
        command.extend(["--extra-config", str(config)])
    completed = subprocess.run(
        command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    if completed.returncode != 0:
        raise PipelineError(completed.stdout.strip())
    print(completed.stdout, end="")
    state = json.loads(ACTIVE_CAPTURE.read_text())
    run_dir = Path(state["run_dir"])
    if not run_dir.is_absolute():
        run_dir = ROOT / run_dir
    run_dir = run_dir.resolve()
    run_dir.relative_to(ROOT.resolve())
    (run_dir / "console").mkdir(parents=True, exist_ok=True)
    (run_dir / "supervisor").mkdir(parents=True, exist_ok=True)
    (run_dir / "analysis").mkdir(parents=True, exist_ok=True)
    snapshot_run_configuration(run_dir, scenario_path, case_path)
    return run_dir


def write_fallback_result(run_dir: Path, outcome: str, category: str, reason: str, exit_code: int) -> None:
    path = run_dir / "supervisor" / "result.json"
    if path.exists():
        return
    payload = {
        "schema_version": 1,
        "outcome": outcome,
        "category": category,
        "reason": reason,
        "exit_code": exit_code,
        "started_unix": None,
        "finished_unix": time.time(),
        "events": [],
    }
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def process_group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def stop_owned_process_group(process_group: int, sigint_s: float, sigterm_s: float) -> None:
    for signum, timeout in ((signal.SIGINT, sigint_s), (signal.SIGTERM, sigterm_s)):
        if not process_group_exists(process_group):
            return
        os.killpg(process_group, signum)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not process_group_exists(process_group):
                return
            time.sleep(0.1)
    if process_group_exists(process_group):
        os.killpg(process_group, signal.SIGKILL)


def stop_capture(run_dir: Path) -> None:
    completed = subprocess.run(
        [sys.executable, str(CAPTURE), "stop"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    (run_dir / "console" / "capture_stop.log").write_text(completed.stdout)
    print(completed.stdout, end="")
    if completed.returncode != 0:
        raise PipelineError("Experiment collection failed; see console/capture_stop.log")


def analyse_bundle(
    run_dir: Path,
    scenario_path: Path,
    no_plots: bool = False,
    detailed: bool = False,
    focus_phase: Optional[str] = None,
    focus_time: Optional[float] = None,
) -> int:
    command = [
        sys.executable,
        str(ANALYSER),
        "--bundle",
        str(run_dir),
        "--scenario-file",
        str(scenario_path),
    ]
    if no_plots:
        command.append("--no-plots")
    if detailed or focus_phase is not None or focus_time is not None:
        command.append("--detailed")
    if focus_phase is not None:
        command.extend(["--focus-phase", focus_phase])
    if focus_time is not None:
        command.extend(["--focus-time", str(focus_time)])
    completed = subprocess.run(
        command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    (run_dir / "console" / "analysis.log").write_text(completed.stdout)
    if completed.stdout:
        print(completed.stdout, end="")
    artifacts = [
        ("Summary", run_dir / "analysis" / "summary.md", True),
        ("Dashboard", run_dir / "analysis" / "dashboard.png", not no_plots),
        ("Key events", run_dir / "analysis" / "key_events.csv", True),
    ]
    print("Compact evidence:")
    for label, path, expected in artifacts:
        print(f"  {label}: {path if expected and path.exists() else 'not generated'}")
    print("Feedback for Codex:")
    print("  - Expected behavior or change under test")
    print("  - First visibly abnormal phase or approximate time")
    print("  - Observed motion or state-machine behavior")
    print("  - Manual intervention or unusual Gazebo/RViz indication")
    return completed.returncode


def run_pipeline(
    args: argparse.Namespace, scenario_path: Path, scenario: dict, case_path: Path
) -> int:
    if args.auto_arm and not scenario.get("simulation_only"):
        raise PipelineError("Automatic arming is restricted to simulation-only scenarios")
    gui = bool(args.gui)
    for check in preflight(scenario, gui=gui):
        print(f"PASS: {check}")

    mode = "auto" if args.auto_arm else "manual"
    control_mode = "automatic" if args.auto_arm else "manual"
    run_dir = run_capture_start(scenario_path, scenario, mode, case_path, args.case)
    print(f"Evidence bundle: {run_dir}")
    console_path = run_dir / "console" / "launch.log"
    result_path = run_dir / "supervisor" / "result.json"
    timeouts = scenario["timeouts"]
    command = build_launch_command(
        scenario,
        gui=gui,
        control_mode=control_mode,
        result_path=result_path,
        case_path=case_path,
        xy_bias_mode=args.xy_bias_mode,
        disturbance_force_x_n=args.disturbance_force_x_n,
        disturbance_force_y_n=args.disturbance_force_y_n,
        landing_check_from=args.landing_check_from,
    )
    environment = simulation_environment()
    ros_log_dir = run_dir / "raw" / "ros"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment["ROS_LOG_DIR"] = str(ros_log_dir)
    runner_metadata = {
        "schema_version": 1,
        "command": command,
        "control_mode": control_mode,
        "display_mode": "gui" if gui else "headless",
        "started_unix": time.time(),
        "case": args.case,
        "case_config": str(case_path.resolve()),
        "xy_bias_mode": args.xy_bias_mode,
        "disturbance_force_x_n": float(args.disturbance_force_x_n),
        "disturbance_force_y_n": float(args.disturbance_force_y_n),
        "landing_check_from": args.landing_check_from,
    }
    (run_dir / "console" / "runner.json").write_text(
        json.dumps(runner_metadata, indent=2, sort_keys=True) + "\n"
    )

    interrupted = False
    launch_failed = False
    log_handle = console_path.open("w")
    process: Optional[subprocess.Popen] = None
    process_group: Optional[int] = None
    try:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        process_group = os.getpgid(process.pid)
        wait_for_start = float(timeouts["manual_start_s"] if args.manual_control else 5.0)
        outer_timeout = (
            float(timeouts["startup_s"])
            + wait_for_start
            + float(timeouts["mission_s"])
            + float(timeouts["success_dwell_s"])
            + 20.0
        )
        deadline = time.monotonic() + outer_timeout
        result_seen_at: Optional[float] = None
        while True:
            return_code = process.poll()
            if result_path.is_file() and result_seen_at is None:
                result_seen_at = time.monotonic()
            if return_code is not None:
                if not result_path.is_file():
                    launch_failed = True
                    write_fallback_result(run_dir, "FAIL", "infrastructure", "LAUNCH_EXITED_WITHOUT_RESULT", 2)
                break
            if result_seen_at is not None and time.monotonic() - result_seen_at > 12.0:
                break
            if time.monotonic() >= deadline:
                launch_failed = True
                write_fallback_result(run_dir, "FAIL", "infrastructure", "OUTER_WATCHDOG_TIMEOUT", 2)
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True
        write_fallback_result(run_dir, "ABORTED", "user", "USER_INTERRUPT", 130)
    except Exception:
        launch_failed = True
        write_fallback_result(run_dir, "FAIL", "infrastructure", "LAUNCH_EXCEPTION", 2)
        raise
    finally:
        if process_group is not None:
            stop_owned_process_group(
                process_group,
                float(timeouts.get("sigint_s", 7.0)),
                float(timeouts.get("sigterm_s", 3.0)),
            )
        if process is not None:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
        log_handle.close()
        try:
            stop_capture(run_dir)
        except PipelineError:
            launch_failed = True

    analysis_code = analyse_bundle(run_dir, scenario_path, no_plots=args.no_plots)
    print(f"Verdict: {run_dir / 'analysis' / 'summary.md'}")
    if interrupted:
        return 130
    if launch_failed:
        return 2
    return analysis_code


def latest_bundle(scenario: str) -> Path:
    candidates = sorted(
        (ROOT / "logs" / "c1f_experiments").glob(f"{scenario}_*"),
        key=lambda path: path.stat().st_mtime_ns,
    )
    if not candidates:
        raise PipelineError(f"No captured bundle found for {scenario}")
    return candidates[-1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("cases", help="list registered M1 environment cases")
    for name in ("preflight", "run", "analyze"):
        command = sub.add_parser(name)
        command.add_argument("--scenario", default="c1f6")
        if name in {"preflight", "run"}:
            command.add_argument("--case", default="baseline")
            display = command.add_mutually_exclusive_group()
            display.add_argument("--headless", action="store_true")
            display.add_argument("--gui", action="store_true")
        if name == "run":
            control = command.add_mutually_exclusive_group(required=True)
            control.add_argument("--auto-arm", action="store_true")
            control.add_argument("--manual-control", action="store_true")
            command.add_argument("--no-plots", action="store_true")
            command.add_argument(
                "--xy-bias-mode",
                choices=(
                    "legacy_integral",
                    "lateral_disturbance",
                    "lateral_disturbance_shadow",
                    "none",
                ),
                default="legacy_integral",
                help="mutually exclusive controller XY bias-rejection mode",
            )
            command.add_argument(
                "--disturbance-force-x-n", type=float, default=0.0,
                help="simulation-only persistent world-frame force on x3 in X [N]",
            )
            command.add_argument(
                "--disturbance-force-y-n", type=float, default=0.0,
                help="simulation-only persistent world-frame force on x3 in Y [N]",
            )
            command.add_argument(
                "--landing-check-from",
                default="",
                help=(
                    "simulation-only: request planner LANDING when this mission phase "
                    "is reached and treat LANDED_DISARMED as success"
                ),
            )
        if name == "analyze":
            command.add_argument("--bundle", type=Path)
            command.add_argument("--no-plots", action="store_true")
            command.add_argument(
                "--detailed",
                action="store_true",
                help="write a focused derived CSV and dashboard",
            )
            focus = command.add_mutually_exclusive_group()
            focus.add_argument("--focus-phase")
            focus.add_argument("--focus-time", type=float)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "cases":
            for case_name in list_cases():
                print(case_name)
            return 0
        scenario_path, scenario = load_scenario(args.scenario)
        case_path = None
        if args.command in {"preflight", "run"}:
            case_path, _ = load_case(args.case)
        if args.command == "preflight":
            for check in preflight(scenario, gui=bool(args.gui)):
                print(f"PASS: {check}")
            return 0
        if args.command == "run":
            assert case_path is not None
            return run_pipeline(args, scenario_path, scenario, case_path)
        bundle = args.bundle.resolve() if args.bundle else latest_bundle(args.scenario)
        return analyse_bundle(
            bundle,
            scenario_path,
            no_plots=args.no_plots,
            detailed=args.detailed,
            focus_phase=args.focus_phase,
            focus_time=args.focus_time,
        )
    except (PipelineError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f"run_sim_test.py: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
