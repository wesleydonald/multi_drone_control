from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BACKEND = ROOT / "src" / "tejen_dynamic_planner" / "src" / "transfer_backend_node.cpp"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch" / "m2d_four_drone_sequential.launch.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2d_sequential_attachment.sh"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_backend_has_separate_pre_authority_octopus_budget_without_changing_default():
    backend = text(BACKEND)

    assert '"octopus_max_runtime_s", 2.0' in backend
    assert (
        '"octopus_pre_authority_max_runtime_s", octopus_max_runtime_s_' in backend
    )
    assert "!std::isfinite(octopus_pre_authority_max_runtime_s_)" in backend
    assert "!(octopus_pre_authority_max_runtime_s_ > 0.0)" in backend

    helper = backend.split("double preAuthoritySpliceLookaheadS() const", 1)[1].split(
        "bool tryActivate()", 1
    )[0]
    # If the separate pre-authority budget is not increased, preserve the exact
    # commissioned splice timing.  A diagnostic increase extends the stationary
    # future splice but never beyond the existing 1.0 s planner limit.
    assert (
        "octopus_pre_authority_max_runtime_s_ <= octopus_max_runtime_s_ + 1e-12"
        in helper
    )
    assert "return fixed_splice_lookahead_s_;" in helper
    assert "return std::min(" in helper
    assert "1.0," in helper
    assert "octopus_pre_authority_max_runtime_s_" in helper


def test_pre_authority_search_uses_long_budget_but_active_flight_reverts_to_frozen_budget():
    backend = text(BACKEND)

    activation = backend.split("bool tryActivate()", 1)[1].split(
        "void pollPlannerFuture()", 1
    )[0]
    assert "preAuthoritySpliceLookaheadS()" in activation
    assert "octopus_pre_authority_max_runtime_s_));" in activation

    authority = backend.split(
        "void authorityCallback(const std_msgs::msg::Bool::SharedPtr msg)", 1
    )[1].split("void disableBackend", 1)[0]
    # If prepared-plan reuse cannot be certified, Python still holds hover while
    # the fresh authority candidate searches, so that fallback also gets the
    # stationary pre-authority budget.
    assert "const double authority_handoff_splice_s = preAuthoritySpliceLookaheadS();" in authority
    fresh = authority.split(
        "auto fresh_planner = std::make_unique<RecedingHorizonPlanner>", 1
    )[1].split("// Keep a stationary seed", 1)[0]
    assert "octopus_pre_authority_max_runtime_s_));" in fresh

    # A successfully rebased prepared trajectory immediately installs a normal
    # active-flight planner with the frozen 100 ms-class runtime parameter.
    rebased = authority.split("if (reuse_prepared_commit_on_authority_)", 1)[1].split(
        "// Default C1F.8c fallback", 1
    )[0]
    assert "octopus_max_runtime_s_));" in rebased
    assert "octopus_pre_authority_max_runtime_s_));" not in rebased

    poll = backend.split("void pollPlannerFuture()", 1)[1].split(
        "void replanTimer()", 1
    )[0]
    fresh_commit = poll.split("if (authority_fresh_plan_pending_)", 1)[1].split(
        "authority_fresh_plan_pending_ = false", 1
    )[0]
    assert "octopus_max_runtime_s_));" in fresh_commit
    assert "octopus_pre_authority_max_runtime_s_));" not in fresh_commit


def test_live_retarget_and_replan_evidence_follow_actual_authority_state():
    backend = text(BACKEND)
    timer = backend.split("void replanTimer()", 1)[1].split(
        "void referenceTimer()", 1
    )[0]

    assert "const bool rebuilding_pre_authority = !authority_granted_;" in timer
    assert "? octopus_pre_authority_max_runtime_s_" in timer
    assert ": octopus_max_runtime_s_));" in timer
    assert "const double submitted_octopus_wall_budget_s = authority_granted_" in timer
    assert "? octopus_max_runtime_s_" in timer
    assert ": octopus_pre_authority_max_runtime_s_;" in timer
    assert "outcome.octopus_wall_budget_s = submitted_octopus_wall_budget_s;" in timer

    csv_writer = backend.split("void writeReplanCsvRow", 1)[1].split(
        "KinematicsFilter kinematics_", 1
    )[0]
    assert "1e3 * outcome.octopus_wall_budget_s" in csv_writer
    assert "1e3 * octopus_max_runtime_s_ << ',' << margin_ms" not in csv_writer


def test_m2d_launch_and_runner_expose_only_pre_authority_override():
    launch = text(LAUNCH)
    runner = text(RUNNER)

    # Active-flight M2D remains frozen at 100 ms.
    assert '"octopus_max_runtime_s": 0.10' in launch
    assert '"octopus_pre_authority_max_runtime_s": ParameterValue(' in launch
    assert 'DeclareLaunchArgument(\n            "octopus_pre_authority_max_runtime_s", default_value="0.10"' in launch

    # Normal M2D remains unchanged unless the diagnostic environment variable is
    # explicitly supplied.  The runner records both values in PASS/FAIL evidence.
    assert 'PREAUTH_OCTOPUS_MAX_RUNTIME_S="${M2D_PREAUTH_OCTOPUS_MAX_RUNTIME_S:-0.10}"' in runner
    assert 'ACTIVE_OCTOPUS_MAX_RUNTIME_S="0.10"' in runner
    assert 'octopus_pre_authority_max_runtime_s:="$PREAUTH_OCTOPUS_MAX_RUNTIME_S"' in runner
    assert 'octopus_pre_authority_max_runtime_s=$PREAUTH_OCTOPUS_MAX_RUNTIME_S' in runner
    assert 'octopus_active_max_runtime_s=$ACTIVE_OCTOPUS_MAX_RUNTIME_S' in runner


def test_extended_pre_authority_budget_seeds_stationary_incumbent_so_future_splice_is_real():
    backend = text(BACKEND)
    activation = backend.split("bool tryActivate()", 1)[1].split(
        "void pollPlannerFuture()", 1
    )[0]

    assert "const bool extended_pre_authority_seed =" in activation
    assert (
        "octopus_pre_authority_max_runtime_s_ > octopus_max_runtime_s_ + 1e-12"
        in activation
    )
    assert "if (seed_stationary_on_activation_ || extended_pre_authority_seed)" in activation
    assert "makeStationaryHoverTrajectory" in activation
    assert "planner_->initializeCommittedTrajectory(stationary_seed);" in activation
    assert "shadow_commit_ = std::move(stationary_seed);" in activation


def test_recheck_exception_detail_is_preserved_in_replan_csv():
    core_hpp = text(
        ROOT / "src" / "tejen_dynamic_planner" / "standalone" / "include"
        / "dynamic_planner" / "receding_horizon_planner.hpp"
    )
    core_cpp = text(
        ROOT / "src" / "tejen_dynamic_planner" / "standalone" / "src"
        / "receding_horizon_planner.cpp"
    )
    backend = text(BACKEND)

    assert "std::string recheck_error;" in core_hpp
    assert "result.recheck_error = exc.what();" in core_cpp
    assert "recheck_error" in backend
    assert "safeCsvField(result.recheck_error)" in backend
