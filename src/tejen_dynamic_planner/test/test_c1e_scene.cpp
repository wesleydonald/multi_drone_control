#include "tejen_dynamic_planner/c1e_scene.hpp"

#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using dynamic_planner::StaticConvexObstacle;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::Vec3;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

SuspendedGeometry c1eGeometry() {
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 0.50;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = 15.0 * kPi / 180.0;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    return geometry;
}

StaticConvexObstacle triangularPrism() {
    // Six vertices deliberately exercise the arbitrary-convex ROS scene path.
    StaticConvexObstacle obstacle;
    obstacle.name = "c1e_triangular_prism_regression";
    obstacle.vertices.resize(6, 3);
    obstacle.vertices <<
        0.40, -0.18, 0.58,
        0.63, -0.18, 0.58,
        0.51,  0.20, 0.58,
        0.40, -0.18, 0.78,
        0.63, -0.18, 0.78,
        0.51,  0.20, 0.78;
    return obstacle;
}

struct PlannerTuning {
    double spline_time_factor = 3.0;
    double jerk_limit_mps3 = 4.0;
    double separator_margin_m = 0.001;
    int max_working_set_recalculations = 500;
};

dynamic_planner::LocalPlannerConfig localConfig(const PlannerTuning& tuning = {}) {
    dynamic_planner::LocalPlannerConfig config;
    config.num_segments = 4;
    config.octopus.samples_per_axis = {9, 9, 9};
    config.octopus.alpha_shrink = 0.9;
    config.octopus.voxel_fraction = 0.10;
    config.octopus.heuristic_bias = 1.0;
    config.octopus.max_runtime_s = 2.0;
    config.octopus.xyz_min = Vec3(-1.0, -1.0, 0.20);
    config.octopus.xyz_max = Vec3(2.0, 1.0, 2.20);
    config.octopus.random_seed = 1;
    config.refinement.xyz_min = config.octopus.xyz_min;
    config.refinement.xyz_max = config.octopus.xyz_max;
    config.refinement.j_max = Vec3::Constant(tuning.jerk_limit_mps3);
    config.refinement.separator_margin_m = tuning.separator_margin_m;
    config.refinement.max_working_set_recalculations =
        tuning.max_working_set_recalculations;
    return config;
}

dynamic_planner::RecedingHorizonConfig recedingConfig(
    const PlannerTuning& tuning = {}) {
    dynamic_planner::RecedingHorizonConfig config;
    config.dc_s = 1.0 / 30.0;
    config.planning_radius_m = 2.0;
    config.factor_alpha = 2.5;
    config.min_splice_lookahead_s = 0.05;
    config.max_splice_lookahead_s = 1.0;
    config.factor_alloc = 1.0;
    config.factor_alloc_close = 2.5;
    config.spline_time_factor = tuning.spline_time_factor;
    config.close_to_goal_m = 0.20;
    config.goal_tolerance_m = 0.05;
    config.continuity_tolerance = 1e-7;
    config.separator_validation_tolerance = 1e-7;
    config.initial_splice_timing_s = 0.16;
    config.enable_octopus_useful_deadline = true;
    config.octopus_post_search_reserve_s = config.dc_s;
    config.v_max = Vec3::Ones();
    config.a_max = Vec3(1.0, 1.0, 1.5);
    return config;
}

std::string describeResult(const dynamic_planner::ReplanResult& result) {
    std::ostringstream out;
    out << "status=" << result.status
        << " duration_s=" << result.local_duration_s;
    if (!result.local_plan.has_value()) {
        return out.str();
    }

    const auto& local = *result.local_plan;
    out << " local_status=" << local.status
        << " local_message={" << local.message << '}'
        << " search_status=" << local.search.status
        << " search_termination=" << local.search.termination_reason
        << " expanded=" << local.search.expanded_nodes
        << " popped=" << local.search.popped_nodes
        << " separator_lps=" << local.search.separator_lp_calls;
    if (local.search.goal_distance_m.has_value()) {
        out << " raw_goal_distance_m=" << *local.search.goal_distance_m;
    }

    if (!local.refinement.has_value()) {
        return out.str();
    }
    const auto& refinement = *local.refinement;
    out << " refinement_status=" << refinement.status
        << " refinement_message={" << refinement.message << '}'
        << " n_wsr=" << refinement.working_set_recalculations;
    return out.str();
}

dynamic_planner::ReplanResult runSingleAttempt(
    const StaticConvexObstacle& obstacle,
    const Vec3& start,
    const Vec3& goal,
    const Vec3& body_half,
    const SuspendedGeometry& suspended,
    const PlannerTuning& tuning) {
    dynamic_planner::WorldSnapshot snapshot;
    snapshot.captured_at_s = 0.0;
    snapshot.ego_half_extents = body_half;
    snapshot.ego_suspended_geometry = suspended;
    snapshot.physical_static_obstacles.push_back(obstacle);
    dynamic_planner::VersionedWorld world(std::move(snapshot));

    dynamic_planner::State measured;
    measured.position = start;
    dynamic_planner::RecedingHorizonPlanner planner(
        goal, recedingConfig(tuning), localConfig(tuning));
    planner.initializeCommittedTrajectory(
        dynamic_planner::makeStationaryHoverTrajectory(start, 0.0, 0.10));
    return planner.replan(0.0, measured, world, []() { return 0.05; });
}

void requirePlannerAvoids(
    const StaticConvexObstacle& obstacle,
    const Vec3& start,
    const Vec3& goal,
    const Vec3& body_half,
    const SuspendedGeometry& suspended,
    const char* label) {
    dynamic_planner::WorldSnapshot snapshot;
    snapshot.captured_at_s = 0.0;
    snapshot.ego_half_extents = body_half;
    snapshot.ego_suspended_geometry = suspended;
    snapshot.physical_static_obstacles.push_back(obstacle);
    dynamic_planner::VersionedWorld world(std::move(snapshot));

    dynamic_planner::State measured;
    measured.position = start;
    const PlannerTuning baseline;
    dynamic_planner::RecedingHorizonPlanner planner(
        goal, recedingConfig(baseline), localConfig(baseline));
    planner.initializeCommittedTrajectory(
        dynamic_planner::makeStationaryHoverTrajectory(start, 0.0, 0.10));

    // Deterministic trajectory-clock script: the solve consumes 50 ms of
    // trajectory time, comfortably inside the approved 400 ms first splice lead.
    const auto result = planner.replan(
        0.0, measured, world, []() { return 0.05; });
    require(result.attempted, "C.1e planner regression did not attempt a solve");
    if (!result.accepted) {
        std::cerr << "C1E_DIAGNOSTIC baseline: "
                  << describeResult(result) << '\n';

        std::vector<std::pair<std::string, PlannerTuning>> diagnostics;
        PlannerTuning no_margin = baseline;
        no_margin.separator_margin_m = 0.0;
        diagnostics.emplace_back("separator_margin=0", no_margin);
        PlannerTuning jerk8 = baseline;
        jerk8.jerk_limit_mps3 = 8.0;
        diagnostics.emplace_back("jerk_limit=8", jerk8);
        PlannerTuning duration25 = baseline;
        duration25.spline_time_factor = 2.5;
        diagnostics.emplace_back("time_factor=2.5", duration25);
        PlannerTuning duration4 = baseline;
        duration4.spline_time_factor = 4.0;
        diagnostics.emplace_back("time_factor=4", duration4);
        PlannerTuning wsr2000 = baseline;
        wsr2000.max_working_set_recalculations = 2000;
        diagnostics.emplace_back("n_wsr=2000", wsr2000);
        PlannerTuning relaxed = baseline;
        relaxed.spline_time_factor = 4.0;
        relaxed.jerk_limit_mps3 = 8.0;
        relaxed.max_working_set_recalculations = 2000;
        diagnostics.emplace_back("time_factor=4,jerk_limit=8,n_wsr=2000", relaxed);

        for (const auto& diagnostic : diagnostics) {
            try {
                const auto alternate = runSingleAttempt(
                    obstacle, start, goal, body_half, suspended,
                    diagnostic.second);
                std::cerr << "C1E_DIAGNOSTIC " << diagnostic.first << ": "
                          << describeResult(alternate) << '\n';
            } catch (const std::exception& exc) {
                std::cerr << "C1E_DIAGNOSTIC " << diagnostic.first
                          << ": EXCEPTION={" << exc.what() << "}\n";
            }
        }
        throw std::runtime_error(
            std::string(label) +
            " C.1e planner regression failed; see C1E_DIAGNOSTIC lines above");
    }
    require(result.candidate_safety.has_value() && result.candidate_safety->safe,
            "C.1e accepted candidate lacks a safe physical-space recheck");
    require(result.planning_world_version == result.recheck_world_version &&
                !result.recheck_retry_performed,
            "static C.1e world version changed during planning");

    const auto& committed = planner.committedTrajectory();
    double max_direct_deviation_m = 0.0;
    for (double t = result.splice_time_s; t < committed.endTime(); t += 0.02) {
        const Vec3 position = committed.evaluate(t).position;
        max_direct_deviation_m = std::max(
            max_direct_deviation_m,
            std::hypot(position.y(), position.z() - start.z()));
    }
    require(max_direct_deviation_m > 0.03,
            "C.1e obstacle candidate did not visibly detour from the straight path");
}

}  // namespace

int main() {
    try {
        const Vec3 start(0.0, 0.0, 1.20);
        const Vec3 goal(1.0, 0.0, 1.20);
        const Vec3 body_half(0.105, 0.105, 0.060);
        const SuspendedGeometry suspended = c1eGeometry();

        const auto positive = tejen_dynamic_planner::makeYawedCuboidObstacle(
            "c1e_commissioning_obstacle", Vec3(0.50, 0.06, 0.68),
            Vec3(0.12, 0.25, 0.10), 20.0 * kPi / 180.0);
        const auto positive_witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
            start, goal, body_half, suspended, positive);
        require(positive_witness.passed(), "positive-Y C.1e witness failed");
        require(!positive_witness.blocking_component.empty(),
                "positive-Y witness did not identify a suspended blocker");
        requirePlannerAvoids(
            positive, start, goal, body_half, suspended, "positive-Y cuboid");

        const auto mirrored = tejen_dynamic_planner::makeYawedCuboidObstacle(
            "c1e_commissioning_obstacle_mirrored", Vec3(0.50, -0.06, 0.68),
            Vec3(0.12, 0.25, 0.10), -20.0 * kPi / 180.0);
        const auto mirrored_witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
            start, goal, body_half, suspended, mirrored);
        require(mirrored_witness.passed(), "mirrored C.1e witness failed");
        requirePlannerAvoids(
            mirrored, start, goal, body_half, suspended, "mirrored cuboid");

        const auto prism_witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
            start, goal, body_half, suspended, triangularPrism());
        require(prism_witness.passed(), "non-box convex C.1e witness failed");
        // The non-box case is an interface/geometry regression, not a second
        // commissioning mission. Full obstacle-avoidance acceptance is required
        // for the actual cuboid and its mirrored counterpart above.

        std::cout << "test_c1e_scene: PASS\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& exc) {
        std::cerr << "test_c1e_scene: FAIL: " << exc.what() << '\n';
        return EXIT_FAILURE;
    }
}
