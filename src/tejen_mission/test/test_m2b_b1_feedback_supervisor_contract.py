"""Contracts for guarded kT feedback and fail-fast B1 supervision."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"
LAUNCH = PKG / "launch" / "m2b_b1_single_attachment.launch.py"
TELEMETRY = PKG / "tejen_mission" / "m2b_b1_telemetry.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"
CONTROLLER = ROOT / "src" / "tejen_mpc" / "tejen_mpc" / "main.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_b1_launch_enables_existing_full_model_kt_feedback_with_guarded_limits():
    text = _text(LAUNCH)
    assert '"enable_thrust_ratio_ukf": True' in text
    assert '"thrust_ratio_estimator_backend": "full_model_kt_ukf"' in text
    assert '"enable_thrust_ratio_feedback": True' in text
    # Reuse the existing guarded feedback values already used by the thesis stack,
    # rather than inventing a new aggressive tuning for M2B.
    assert '"thrust_ratio_feedback_min_updates": 15' in text
    assert '"thrust_ratio_feedback_max_std": 1.5' in text
    assert '"thrust_ratio_feedback_rate_per_s": 0.5' in text
    assert '"thrust_ratio_feedback_max_fractional_change": 0.25' in text
    assert '"thrust_ratio_feedback_deadband": 0.20' in text


def test_controller_status_exposes_live_feedback_state_for_structured_b1_telemetry():
    text = _text(CONTROLLER)
    assert "thrust_ratio_feedback:" in text
    for token in (
        "enabled=",
        "applied=",
        "status=",
        "estimate=",
        "std=",
        "target=",
        "mpc=",
        "updates=",
    ):
        assert token in text


def test_b1_csv_records_estimator_and_actual_mpc_thrust_ratio_feedback_state():
    text = _text(TELEMETRY)
    for field in (
        '"thrust_ratio_feedback_enabled"',
        '"thrust_ratio_feedback_applied"',
        '"thrust_ratio_feedback_status"',
        '"thrust_ratio_estimate"',
        '"thrust_ratio_estimate_std"',
        '"thrust_ratio_feedback_target"',
        '"mpc_thrust_ratio"',
        '"thrust_ratio_ukf_update_count"',
    ):
        assert field in text
    assert "thrust_ratio_feedback:" in text


def test_b1_runner_waits_fail_fast_on_controller_or_mission_faults():
    text = _text(RUNNER)
    assert "runtime_health_check" in text
    assert "wait_for_csv_value_guarded" in text
    assert "external_reference_fault_latched" in text
    assert "pendulum_fault_latched" in text
    assert "M2_FAULT" in text
    assert "wait_for_csv_value_guarded mission_phase M2_ATTACHED_HOLD" in text
    # The old unguarded 90 s wait is exactly what kept the failed flight alive.
    assert "wait_for_csv_value mission_phase M2_ATTACHED_HOLD 90" not in text


def test_full_b1_supervisor_fails_fast_if_retry_budget_falls_through_to_landing():
    text = _text(RUNNER)
    start = text.index("runtime_health_check()")
    end = text.index("wait_for_csv_value_guarded()", start)
    block = text[start:end]
    assert "M2_LANDING_STAGE" in block
    assert "LANDED_DISARMED" in block
    assert '[[ "$MODE" == "success" ]]' in block


def test_b1_telemetry_schema_is_versioned_after_feedback_columns_are_added():
    text = _text(TELEMETRY)
    assert '"schema_version": 2' in text


def test_b1_telemetry_parses_live_feedback_status_values_without_ros_runtime():
    # Execute only the telemetry callback AST so this regression can validate the
    # parser in lightweight CI where rclpy is unavailable.
    import ast
    import re
    import types

    tree = ast.parse(_text(TELEMETRY))
    klass = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "M2BB1Telemetry"
    )
    method = next(
        node for node in klass.body
        if isinstance(node, ast.FunctionDef) and node.name == "_controller_status"
    )
    method = ast.fix_missing_locations(method)
    namespace = {"re": re, "String": object}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(TELEMETRY), "exec"), namespace)

    dummy = types.SimpleNamespace(state={})
    msg = types.SimpleNamespace(data=(
        "R6.3C.1d MPC INTEGRATION\n"
        "fault_latched: False reason: none latch_count: 0\n"
        "pendulum_state_age_ms: 8.000 fault_latched: False reason: none latch_count: 0\n"
        "thrust_ratio_feedback: enabled=True applied=True status=applied "
        "estimate=43.125 std=0.460 target=43.125 mpc=43.500 updates=27\n"
    ))
    namespace["_controller_status"](dummy, msg)

    assert dummy.state["external_reference_fault_latched"] is False
    assert dummy.state["pendulum_fault_latched"] is False
    assert dummy.state["thrust_ratio_feedback_enabled"] is True
    assert dummy.state["thrust_ratio_feedback_applied"] is True
    assert dummy.state["thrust_ratio_feedback_status"] == "applied"
    assert dummy.state["thrust_ratio_estimate"] == 43.125
    assert dummy.state["thrust_ratio_estimate_std"] == 0.460
    assert dummy.state["thrust_ratio_feedback_target"] == 43.125
    assert dummy.state["mpc_thrust_ratio"] == 43.500
    assert dummy.state["thrust_ratio_ukf_update_count"] == 27
