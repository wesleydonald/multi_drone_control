#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/octopus_search.hpp"

namespace dynamic_planner {

struct RefinementConfig {
    Vec3 v_max = Vec3::Ones();
    Vec3 a_max = Vec3::Constant(1.5);
    Vec3 j_max = Vec3::Constant(4.0);
    double jerk_weight = 1.0;
    double goal_weight = 10.0;
    double goal_acceptance_m = 0.05;
    double separator_margin_m = 0.001;
    Vec3 xyz_min = Vec3(-0.75, -0.75, 0.40);
    Vec3 xyz_max = Vec3(3.00, 3.00, 2.60);
    int max_working_set_recalculations = 500;
    double validation_tolerance = 2e-6;
};

struct QuadraticProgram {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd linear;
    double constant = 0.0;
    Eigen::MatrixXd a_eq;
    Eigen::VectorXd b_eq;
    Eigen::MatrixXd a_ub;
    Eigen::VectorXd b_ub;
    Eigen::VectorXd lower_bounds;
    Eigen::VectorXd upper_bounds;
};

struct RefinementResult {
    bool success = false;
    std::string status;
    std::string message;
    std::optional<ControlPoints> control_points;
    double solve_time_s = 0.0;
    int working_set_recalculations = 0;
    double raw_goal_distance_m = 0.0;
    std::optional<double> refined_goal_distance_m;
    bool goal_accepted = false;
    double raw_objective = 0.0;
    std::optional<double> refined_objective;
    double raw_jerk_cost = 0.0;
    std::optional<double> refined_jerk_cost;
    std::optional<double> max_velocity_axis;
    std::optional<double> max_acceleration_axis;
    std::optional<double> max_jerk_axis;
    std::optional<double> max_equality_residual;
    std::optional<double> max_inequality_violation;
    std::optional<double> min_fixed_separator_slack;
    bool separator_recheck_passed = false;
    QuadraticProgram qp;
};

Eigen::MatrixXd derivativeControlMatrix(const std::vector<double>& knots,
                                        int num_control_points,
                                        int derivative_order,
                                        int degree = kCubicDegree);

Eigen::MatrixXd minvoVelocityControlMatrix(const std::vector<double>& knots,
                                           int num_control_points);

Eigen::MatrixXd rmaderMinvoVelocityConstraintMatrix(
    const std::vector<double>& knots,
    int num_control_points);

QuadraticProgram buildRefinementQP(const OctopusResult& raw_result,
                                   const std::vector<double>& knots,
                                   const Vec3& goal,
                                   const RefinementConfig& config,
                                   const TerminalBoundary& terminal_boundary = {});

RefinementResult refineOctopusTrajectory(const OctopusResult& raw_result,
                                         const std::vector<double>& knots,
                                         const Vec3& goal,
                                         const std::vector<TimeIndexedObstacle>& obstacles,
                                         const RefinementConfig& config,
                                         const TerminalBoundary& terminal_boundary = {});

}  // namespace dynamic_planner
