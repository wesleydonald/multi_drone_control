#include "dynamic_planner/trajectory_refinement.hpp"
#include "dynamic_planner/validation_scenarios.hpp"

#include "test_common.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusSearch;
using dynamic_planner::RefinementConfig;
using dynamic_planner::RefinementResult;
using dynamic_planner::ValidationScenario;

RefinementResult run(const std::string& scene_name) {
    const ValidationScenario scene = dynamic_planner::makeValidationScenario(scene_name, 7);
    const auto raw = OctopusSearch(
        scene.knots,
        scene.initial_control_points,
        scene.goal,
        scene.obstacles,
        scene.octopus_config).search();
    if (!raw.success) {
        throw std::runtime_error("R4 failed in refinement regression: " + raw.status);
    }

    RefinementConfig config;
    config.v_max = scene.octopus_config.v_max;
    config.a_max = scene.octopus_config.a_max;
    config.j_max = Eigen::Vector3d::Constant(4.0);
    config.jerk_weight = 1.0;
    config.goal_weight = 10.0;
    config.goal_acceptance_m = 0.05;
    config.separator_margin_m = 0.001;
    config.xyz_min = scene.octopus_config.xyz_min;
    config.xyz_max = scene.octopus_config.xyz_max;
    config.max_working_set_recalculations = 500;
    config.validation_tolerance = 2e-6;

    return dynamic_planner::refineOctopusTrajectory(
        raw, scene.knots, scene.goal, scene.obstacles, config);
}


void testNonStoppedTerminalUsesFullVelocityConstraintMatrix() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    ControlPoints cps(7, 3);
    cps <<
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.4, 0.0, 1.0,
        0.8, 0.0, 1.0,
        1.0, 0.0, 1.0,
        1.0, 0.0, 1.0;

    dynamic_planner::OctopusResult raw;
    raw.success = true;
    raw.control_points = cps;

    RefinementConfig config;
    config.xyz_min = Eigen::Vector3d(-2.0, -2.0, 0.0);
    config.xyz_max = Eigen::Vector3d(3.0, 2.0, 2.0);

    dynamic_planner::TerminalBoundary stopped;
    stopped.mode = dynamic_planner::TerminalMode::Stopped;
    stopped.position = cps.row(6).transpose();

    dynamic_planner::TerminalBoundary continuation;
    continuation.mode = dynamic_planner::TerminalMode::Continuation;
    continuation.position = cps.row(6).transpose();

    dynamic_planner::TerminalBoundary moving;
    moving.mode = dynamic_planner::TerminalMode::MovingRendezvous;
    moving.position = cps.row(6).transpose();
    moving.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);

    const auto stopped_qp = dynamic_planner::buildRefinementQP(
        raw, knots, stopped.position, config, stopped);
    const auto continuation_qp = dynamic_planner::buildRefinementQP(
        raw, knots, continuation.position, config, continuation);
    const auto moving_qp = dynamic_planner::buildRefinementQP(
        raw, knots, moving.position, config, moving);

    // Full MINVO velocity constraints have one additional 3-row terminal
    // interval versus the legacy stopped/RMADER matrix. Each scalar row creates
    // +/- inequalities on three axes: 3 * 2 * 3 = 18 extra inequalities.
    requireTrue(continuation_qp.a_ub.rows() == stopped_qp.a_ub.rows() + 18,
                "continuation QP did not add the full terminal velocity interval");
    requireTrue(moving_qp.a_ub.rows() == stopped_qp.a_ub.rows() + 18,
                "moving-rendezvous QP did not add the full terminal velocity interval");
}

void requireHealthy(const RefinementResult& result, const std::string& scene) {
    requireTrue(result.success, scene + " refinement failed: " + result.message);
    requireTrue(result.control_points.has_value(), scene + " missing refined control points");
    requireTrue(result.refined_goal_distance_m.has_value(), scene + " missing refined goal distance");
    requireTrue(result.refined_objective.has_value(), scene + " missing refined objective");
    requireTrue(result.refined_jerk_cost.has_value(), scene + " missing refined jerk cost");
    requireTrue(result.max_velocity_axis.has_value(), scene + " missing velocity diagnostic");
    requireTrue(result.max_acceleration_axis.has_value(), scene + " missing acceleration diagnostic");
    requireTrue(result.max_jerk_axis.has_value(), scene + " missing jerk diagnostic");
    requireTrue(result.max_equality_residual.has_value(), scene + " missing equality residual");
    requireTrue(result.max_inequality_violation.has_value(), scene + " missing inequality residual");
    requireTrue(result.min_fixed_separator_slack.has_value(), scene + " missing separator slack");
    requireTrue(result.separator_recheck_passed, scene + " separator recheck failed");
    requireTrue(*result.max_equality_residual <= 2e-6, scene + " equality residual too large");
    requireTrue(*result.max_inequality_violation <= 2e-6, scene + " inequality residual too large");
    requireTrue(*result.min_fixed_separator_slack >= -2e-6, scene + " fixed separator slack too negative");
    requireTrue(*result.max_velocity_axis <= 1.0 + 2e-6, scene + " velocity certificate violated");
    requireTrue(*result.max_acceleration_axis <= 1.5 + 2e-6, scene + " acceleration certificate violated");
    requireTrue(*result.max_jerk_axis <= 4.0 + 2e-6, scene + " jerk certificate violated");
    requireTrue(*result.refined_objective < result.raw_objective, scene + " objective did not improve");
    requireTrue(*result.refined_jerk_cost < result.raw_jerk_cost, scene + " jerk cost did not improve");

    const ControlPoints& cps = *result.control_points;
    requireTrue((cps.row(cps.rows() - 1) - cps.row(cps.rows() - 2)).norm() <= 2e-8,
                  scene + " qN != qN-1");
    requireTrue((cps.row(cps.rows() - 2) - cps.row(cps.rows() - 3)).norm() <= 2e-8,
                  scene + " qN-1 != qN-2");
}

}  // namespace

int main() {
    try {
        testNonStoppedTerminalUsesFullVelocityConstraintMatrix();
        const auto default_result = run("default");
        requireHealthy(default_result, "default");
        requireNear(default_result.raw_goal_distance_m, 0.19764235376052341, 2e-10,
                          "default R4 goal parity");
        // The R4 control points are identical to the Python R6.2 reference, but
        // the fixed separating-plane LP is non-unique. Python R6.2 used
        // SciPy/HiGHS on the validation machine, while this native port uses the
        // RMADER-style GLPK separator. Regress against the deterministic GLPK
        // solution here, while requireHealthy() independently enforces the QP
        // equalities, inequalities, dynamic certificates and separator recheck.
        requireNear(*default_result.refined_goal_distance_m, 0.0416749011841456, 5e-4,
                          "default native-GLPK refinement regression");
        requireTrue(default_result.goal_accepted, "default goal should be accepted");
        requireTrue(*default_result.refined_goal_distance_m <= 0.05 + 1e-10,
                    "default refined goal must remain inside 5 cm acceptance");

        const auto chicane_result = run("static_chicane");
        requireHealthy(chicane_result, "static_chicane");
        requireNear(chicane_result.raw_goal_distance_m, 0.603133095800587, 2e-10,
                          "chicane R4 goal parity");
        requireNear(*chicane_result.refined_goal_distance_m, 0.5407742154234216, 3e-4,
                          "chicane R6.2 goal parity");
        requireTrue(!chicane_result.goal_accepted, "4-segment chicane should remain outside 5 cm");
        requireNear(*chicane_result.max_velocity_axis, 0.878175445135685, 5e-4,
                          "chicane MINVO velocity certificate parity");

        const auto moving_result = run("moving_crossing");
        requireHealthy(moving_result, "moving_crossing");
        requireNear(moving_result.raw_goal_distance_m, 0.19764235376052341, 2e-10,
                          "moving R4 goal parity");
        requireNear(*moving_result.refined_goal_distance_m, 0.02933998172933091, 2e-4,
                          "moving R6.2 goal parity");
        requireTrue(moving_result.goal_accepted, "moving-crossing goal should be accepted");

        std::cout << "test_refinement: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_refinement: FAIL: " << error.what() << '\n';
        return 1;
    }
}
