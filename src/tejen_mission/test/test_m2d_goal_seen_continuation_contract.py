from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
PLANNER = (
    REPO
    / "src"
    / "tejen_dynamic_planner"
    / "standalone"
    / "src"
    / "receding_horizon_planner.cpp"
)
INTEGRATION_TEST = (
    REPO
    / "src"
    / "tejen_dynamic_planner"
    / "standalone"
    / "tests"
    / "test_integration_timing.cpp"
)


def test_continuation_candidate_cannot_latch_goal_seen():
    text = PLANNER.read_text(encoding="utf-8")
    assert "else if (!result.local_continuation_active &&" in text
    assert "result.committed_endpoint_distance_m <= config_.goal_tolerance_m" in text
    assert "A continuation candidate may pass through the mission goal region" in text


def test_cpp_regression_covers_continuation_goal_latch():
    text = INTEGRATION_TEST.read_text(encoding="utf-8")
    assert "continuation-goal regression" in text
    assert "requireTrue(!result.goal_seen && !planner.goalSeen()" in text
    assert "continuation candidate incorrectly latched goal_seen" in text
