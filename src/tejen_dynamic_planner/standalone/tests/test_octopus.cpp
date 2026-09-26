#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/octopus_search.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusConfig;
using dynamic_planner::OctopusResult;
using dynamic_planner::OctopusSearch;
using dynamic_planner::State;
using dynamic_planner::TimeIndexedObstacle;
using dynamic_planner::Vec3;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

OctopusConfig baseConfig() {
    OctopusConfig config;
    config.v_max = Vec3::Constant(2.0);
    config.a_max = Vec3::Constant(2.0);
    config.samples_per_axis = {3, 3, 3};
    config.alpha_shrink = 0.9;
    config.voxel_fraction = 0.10;
    config.heuristic_bias = 1.0;
    config.goal_tolerance_m = 0.10;
    config.max_runtime_s = 1.0;
    config.xyz_min = Vec3(-1.0, -1.0, 0.0);
    config.xyz_max = Vec3(3.0, 1.0, 2.0);
    config.planning_radius_m = 30.0;
    config.random_seed = 1;
    return config;
}

void testEmptySceneProducesFinalFeasibleSpline() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    // The coarse 3x3x3 lattice used by other unit fixtures can legitimately
    // exhaust with only a padded partial. This test specifically exercises the
    // complete-candidate diagnostics, so use the commissioned 9x9x9 density.
    config.samples_per_axis = {9, 9, 9};
    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(1.0, 0.0, 1.0), {}, config).search();

    require(result.success, "empty scene should yield a dynamically feasible completed spline; status=" + result.status);
    require(result.control_points.has_value(), "successful search must return control points");
    require(result.control_points->rows() == 7, "four cubic intervals must return seven control points");
    require(result.expanded_nodes > 0, "empty scene should expand at least one node");
    require(result.popped_nodes > 0, "empty scene should pop at least one node");
    require(result.first_complete_time_s.has_value(),
            "successful complete search must record time to first complete candidate");
    require(result.first_complete_goal_distance_m.has_value(),
            "successful complete search must record first-complete goal distance");
    require(result.first_complete_expanded_nodes > 0,
            "first-complete expanded-node count must be populated");
    require(result.first_complete_popped_nodes > 0,
            "first-complete popped-node count must be populated");
    require(result.closest_complete_improvements > 0,
            "complete search must count at least its first complete candidate");
    require(result.partial_fallback_candidates_tested == 0,
            "complete search must not invoke partial fallback certification");
    require(result.partial_fallback_selected_rank == 0,
            "complete search must not report a partial fallback rank");
    require(*result.first_complete_time_s <= result.search_time_s + 1e-9,
            "first complete time cannot exceed total search time");
}

void testPaddedPartialRejectedWhenFinalCompletionViolatesAccelerationLimit() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3(1.0, 0.0, 0.0);
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    config.v_max = Vec3::Constant(2.0);
    config.a_max = Vec3::Constant(0.2);
    config.max_runtime_s = 1e-9;

    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(2.0, 0.0, 1.0), {}, config).search();

    require(!result.success, "R6.1 padded-partial regression must be rejected");
    require(result.status == "PADDED_CLOSEST_PARTIAL_FINAL_DYNAMICS_FAILED",
            "unexpected padded-partial status: " + result.status);
    require(!result.control_points.has_value(), "rejected padded partial must not expose control points");
    require(!result.first_complete_time_s.has_value(),
            "immediate padded partial should not report a complete candidate");
    require(result.closest_complete_improvements == 0,
            "immediate padded partial should not count complete-candidate improvements");
    require(result.partial_fallback_candidates_retained == 1,
            "immediate timeout should retain only the start partial");
    require(result.partial_fallback_candidates_tested == 1,
            "immediate timeout must certify its one retained partial");
    require(result.partial_fallback_selected_rank == 0,
            "dynamics-rejected fallback must not report a certified rank");
}



void testExternalStopReturnsClosestCompleteFoundSoFar() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    config.samples_per_axis = {9, 9, 9};
    // Strict '<' means zero tolerance prevents GOAL_REACHED even if a sample
    // lands exactly on the goal, so this fixture exits only via the callback.
    config.goal_tolerance_m = 0.0;

    int stop_polls = 0;
    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(1.0, 0.0, 1.0), {}, config,
        [&stop_polls]() { return ++stop_polls > 2000; },
        "USEFUL_DEADLINE_REACHED").search();

    require(result.success, "external stop should return the complete candidate already found");
    require(result.status == "CLOSEST_COMPLETE",
            "external stop did not preserve closest-complete selection: " + result.status);
    require(result.termination_reason == "USEFUL_DEADLINE_REACHED",
            "closest-complete external stop lost termination reason");
    require(result.complete_available_at_termination,
            "external stop failed to report a complete candidate at termination");
    require(result.first_complete_time_s.has_value(),
            "external stop should retain first-complete diagnostics");
    require(result.closest_complete_improvements >= 1,
            "external stop should retain complete-candidate improvement count");
}

void testExternalStopPreservesPaddedPartialFallback() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3(1.0, 0.0, 0.0);
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    config.v_max = Vec3::Constant(2.0);
    config.a_max = Vec3::Constant(0.2);
    config.max_runtime_s = 1.0;

    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(2.0, 0.0, 1.0), {}, config,
        []() { return true; }, "USEFUL_DEADLINE_REACHED").search();

    require(!result.success, "externally stopped padded partial should preserve dynamics rejection");
    require(result.status == "PADDED_CLOSEST_PARTIAL_FINAL_DYNAMICS_FAILED",
            "external stop changed padded-partial fallback semantics: " + result.status);
    require(result.termination_reason == "USEFUL_DEADLINE_REACHED",
            "external stop termination reason was not preserved");
    require(!result.complete_available_at_termination,
            "immediate external stop unexpectedly reported a complete solution");
    require(!result.first_complete_time_s.has_value(),
            "immediate external stop should not report a first complete candidate");
}


void testFinalCheckFailureRetainsDiagnosticWitnessAndCandidate() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    // Force the search to return its padded closest partial before it can inspect
    // the deliberately blocked final interval. The final exact check must then
    // expose the obstacle/interval that rejected the completed candidate.
    config.max_runtime_s = 1e-9;

    TimeIndexedObstacle obstacle;
    obstacle.name = "cooperative_drone_1::attached_tether_vs_ego_body";
    const auto far = dynamic_planner::axisAlignedBoxVertices(
        Vec3(100.0, 100.0, 100.0), Vec3(0.1, 0.1, 0.1));
    const auto blocked = dynamic_planner::axisAlignedBoxVertices(
        Vec3(0.5, 0.0, 1.0), Vec3(20.0, 20.0, 20.0));
    obstacle.interval_vertices = {far, far, far, blocked};

    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(1.0, 0.0, 1.0), {obstacle}, config).search();

    require(!result.success, "blocked padded candidate must fail certification");
    require(result.status == "PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED",
            "unexpected blocked padded status: " + result.status);
    require(result.diagnostic_control_points.has_value(),
            "failed final check must retain the selected candidate for diagnostics");
    require(!result.control_points.has_value(),
            "failed final check must never expose control points as an accepted plan");
    require(!result.separators.empty(),
            "failed final check must retain separator diagnostics");
    require(result.partial_fallback_candidates_retained == 1 &&
                result.partial_fallback_candidates_tested == 1,
            "single immediate fallback must preserve one-candidate diagnostics");
    require(result.partial_fallback_selected_rank == 0,
            "failed final check must not report a certified partial rank");
    const auto& witness = result.separators.back();
    require(!witness.result.feasible,
            "last separator must be the infeasible final-check witness");
    require(witness.obstacle_name == obstacle.name,
            "final-check witness lost obstacle/component identity");
    require(witness.interval_index == 3,
            "final-check witness lost the failing interval index");
}

void testPaddedFallbackTriesMultipleRetainedPartials() {
    // Five segments keep q3/q4 as partial-search states. Stop immediately
    // after the first q3 expansion: q2 plus that expanded q3 give two distinct
    // retained partials, while no q4/q5 complete candidate can yet be popped.
    const auto knots = dynamic_planner::openUniformKnots(0.0, 5.0, 5);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    config.heuristic_bias = 0.0;
    config.goal_tolerance_m = 0.0;

    TimeIndexedObstacle obstacle;
    obstacle.name = "final_interval_blocker";
    const auto far = dynamic_planner::axisAlignedBoxVertices(
        Vec3(100.0, 100.0, 100.0), Vec3(0.1, 0.1, 0.1));
    const auto blocked = dynamic_planner::axisAlignedBoxVertices(
        Vec3(0.0, 0.0, 1.0), Vec3(100.0, 100.0, 100.0));
    obstacle.interval_vertices = {far, far, far, far, blocked};

    int stop_polls = 0;
    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(2.0, 0.0, 1.0), {obstacle}, config,
        [&stop_polls]() { return ++stop_polls > 2; },
        "USEFUL_DEADLINE_REACHED").search();

    require(!result.success, "all retained padded partials are deliberately blocked");
    require(result.termination_reason == "USEFUL_DEADLINE_REACHED",
            "fixture must terminate before a complete candidate is available");
    require(!result.complete_available_at_termination,
            "fixture unexpectedly reached a complete candidate");
    require(result.partial_fallback_candidates_retained >= 2,
            "fixture did not retain multiple partial candidates");
    require(result.partial_fallback_candidates_tested ==
                result.partial_fallback_candidates_retained,
            "all retained partials must be tried when none certifies");
    require(result.partial_fallback_selected_rank == 0,
            "blocked fallback must not report a certified rank");
    require(result.status == "PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED",
            "all-fail fallback must preserve the historical external status");
}

void testInitialQ2BoundaryToleranceAllowsCertifiedRestartResidue() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3(0.5000005, 0.0, 0.0);
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    OctopusConfig config = baseConfig();
    config.max_runtime_s = 1e-9;
    config.xyz_min = Vec3(-0.5, -1.0, 0.0);
    config.xyz_max = Vec3(0.5, 1.0, 2.0);

    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(0.4, 0.0, 1.0), {}, config).search();

    require(result.status != "INVALID_INITIAL_Q2",
            "sub-micrometre boundary residue incorrectly broke restart admissibility");
    require(result.initial_q2_box_valid,
            "initial-q2 diagnostics did not record tolerated boundary residue");
}

void testQ2OutsideWorkspaceRejectedBeforeTimeoutFallback() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3(1.0, 0.0, 0.0);
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);
    require(std::abs(q012(2, 0) - 1.0) < 1e-12, "fixture expected q2.x=1.0");

    OctopusConfig config = baseConfig();
    config.max_runtime_s = 1e-9;
    config.xyz_min = Vec3(-0.5, -1.0, 0.0);
    config.xyz_max = Vec3(0.5, 1.0, 2.0);

    const OctopusResult result = OctopusSearch(
        knots, q012, Vec3(0.4, 0.0, 1.0), {}, config).search();

    require(!result.success, "invalid q2 must not become a successful timeout fallback");
    require(result.status == "INVALID_INITIAL_Q2", "unexpected invalid-q2 status: " + result.status);
    require(!result.control_points.has_value(), "invalid q2 must not return control points");
    require(result.expanded_nodes == 0, "invalid q2 must be rejected before expansion");
    require(result.popped_nodes == 0, "invalid q2 must be rejected before queue pop");
}

}  // namespace

int main() {
    try {
        testEmptySceneProducesFinalFeasibleSpline();
        testPaddedPartialRejectedWhenFinalCompletionViolatesAccelerationLimit();
        testExternalStopReturnsClosestCompleteFoundSoFar();
        testExternalStopPreservesPaddedPartialFallback();
        testFinalCheckFailureRetainsDiagnosticWitnessAndCandidate();
        testPaddedFallbackTriesMultipleRetainedPartials();
        testInitialQ2BoundaryToleranceAllowsCertifiedRestartResidue();
        testQ2OutsideWorkspaceRejectedBeforeTimeoutFallback();
        std::cout << "test_octopus: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_octopus: FAIL: " << error.what() << '\n';
        return 1;
    }
}
