#include "dynamic_planner/trajectory_refinement.hpp"
#include "dynamic_planner/validation_scenarios.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using dynamic_planner::OctopusSearch;
using dynamic_planner::RefinementConfig;
using dynamic_planner::ValidationScenario;

void runOne(const std::string& name, int samples) {
    const ValidationScenario scene = dynamic_planner::makeValidationScenario(name, samples);
    const auto raw = OctopusSearch(
        scene.knots,
        scene.initial_control_points,
        scene.goal,
        scene.obstacles,
        scene.octopus_config).search();

    std::cout << "\n=== " << name << " ===\n";
    std::cout << "R4 status: " << raw.status << '\n';
    std::cout << "R4 success: " << std::boolalpha << raw.success << '\n';
    std::cout << "R4 search time: " << 1000.0 * raw.search_time_s << " ms\n";
    std::cout << "R4 expanded/popped: " << raw.expanded_nodes << "/" << raw.popped_nodes << '\n';
    std::cout << "R4 separator LP calls: " << raw.separator_lp_calls << '\n';
    if (raw.goal_distance_m.has_value()) {
        std::cout << "R4 goal distance: " << *raw.goal_distance_m << " m\n";
    }
    if (!raw.success) {
        return;
    }

    RefinementConfig config;
    config.v_max = scene.octopus_config.v_max;
    config.a_max = scene.octopus_config.a_max;
    config.j_max = dynamic_planner::Vec3::Constant(4.0);
    config.jerk_weight = 1.0;
    config.goal_weight = 10.0;
    config.goal_acceptance_m = 0.05;
    config.separator_margin_m = 0.001;
    config.xyz_min = scene.octopus_config.xyz_min;
    config.xyz_max = scene.octopus_config.xyz_max;
    config.max_working_set_recalculations = 500;
    config.validation_tolerance = 2e-6;

    const auto refined = dynamic_planner::refineOctopusTrajectory(
        raw, scene.knots, scene.goal, scene.obstacles, config);
    std::cout << "R5/R6.2 backend: qpOASES\n";
    std::cout << "refinement status: " << refined.status << '\n';
    std::cout << "refinement success: " << refined.success << '\n';
    std::cout << "refinement message: " << refined.message << '\n';
    std::cout << "qp solve time: " << 1000.0 * refined.solve_time_s << " ms\n";
    std::cout << "qp working-set recalculations: " << refined.working_set_recalculations << '\n';
    if (refined.refined_goal_distance_m.has_value()) {
        std::cout << "goal distance: " << refined.raw_goal_distance_m
                  << " -> " << *refined.refined_goal_distance_m << " m\n";
    }
    std::cout << "goal accepted <5cm: " << refined.goal_accepted << '\n';
    if (refined.refined_objective.has_value()) {
        std::cout << "objective: " << refined.raw_objective
                  << " -> " << *refined.refined_objective << '\n';
    }
    if (refined.refined_jerk_cost.has_value()) {
        std::cout << "jerk cost: " << refined.raw_jerk_cost
                  << " -> " << *refined.refined_jerk_cost << '\n';
    }
    if (refined.max_velocity_axis.has_value()) {
        std::cout << "max RMADER MINVO |velocity vertex|: " << *refined.max_velocity_axis << " m/s\n";
    }
    if (refined.max_acceleration_axis.has_value()) {
        std::cout << "max |acceleration derivative CP|: " << *refined.max_acceleration_axis << " m/s^2\n";
    }
    if (refined.max_jerk_axis.has_value()) {
        std::cout << "max |jerk derivative CP|: " << *refined.max_jerk_axis << " m/s^3\n";
    }
    if (refined.max_equality_residual.has_value()) {
        std::cout << "max equality residual: " << *refined.max_equality_residual << '\n';
    }
    if (refined.max_inequality_violation.has_value()) {
        std::cout << "max inequality violation: " << *refined.max_inequality_violation << '\n';
    }
    if (refined.min_fixed_separator_slack.has_value()) {
        std::cout << "minimum fixed-separator slack: " << *refined.min_fixed_separator_slack << '\n';
    }
    std::cout << "separator recheck: " << refined.separator_recheck_passed << '\n';
    if (refined.control_points.has_value()) {
        std::cout << "refined control points:\n" << *refined.control_points << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scene = "all";
        int samples = 7;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scene" && i + 1 < argc) {
                scene = argv[++i];
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = std::stoi(argv[++i]);
            } else {
                throw std::invalid_argument(
                    "usage: refinement_demo [--scene all|default|static_chicane|moving_crossing] [--samples odd_integer]");
            }
        }
        std::cout << std::setprecision(15);
        std::cout << "R6.3A.4 full local C++ planner parity demo\n";
        std::cout << "samples per axis: " << samples << '\n';
        if (scene == "all") {
            for (const auto& name : dynamic_planner::validationScenarioNames()) {
                runOne(name, samples);
            }
        } else {
            runOne(scene, samples);
        }
        std::cout << "\nSend this output plus ctest output back to ChatGPT.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "refinement_demo: FAIL: " << error.what() << '\n';
        return 1;
    }
}
