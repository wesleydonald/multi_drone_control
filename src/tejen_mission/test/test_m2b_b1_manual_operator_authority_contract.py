"""Static contract for manual ARM / TAKEOFF authority in the M2B B1 runner.

The B1 supervised runner may verify readiness and observe operator actions, but
must not issue ARM or TAKEOFF itself. This keeps the commissioning reference
stationary until the explicit operator TAKEOFF command is received.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "tools" / "sim_test" / "run_m2b_b1.sh"


def test_b1_runner_never_issues_operator_arm_or_takeoff_commands():
    text = RUNNER.read_text(encoding="utf-8")
    assert "ros2 service call /drone_arming_service" not in text
    assert "ros2 topic pub --once /drone_command std_msgs/msg/String '{data: TAKEOFF}'" not in text


def test_b1_runner_waits_for_observed_operator_arm_then_takeoff_without_operator_timeout():
    text = RUNNER.read_text(encoding="utf-8")

    assert "wait_for_csv_value_guarded_operator()" in text
    helper_start = text.index("wait_for_csv_value_guarded_operator()")
    helper_end = text.index("\n}", helper_start) + 2
    helper = text[helper_start:helper_end]
    assert "runtime_health_check" in helper
    assert "timeout" not in helper.lower()

    permission = text.index("wait_for_csv_value arm_permission true 10")
    arm_prompt = text.index("OPERATOR GATE: click ARM")
    arm_observed = text.index("wait_for_csv_value_guarded_operator armed true")
    takeoff_prompt = text.index("OPERATOR GATE: click TAKEOFF")
    takeoff_observed = text.index(
        "wait_for_csv_value_guarded_operator mission_phase M2_VERTICAL_TAKEOFF"
    )
    settle_wait = text.index("wait_for_csv_value_guarded mission_phase M2_SETTLE_CAPTURE 45")

    assert permission < arm_prompt < arm_observed < takeoff_prompt < takeoff_observed < settle_wait
