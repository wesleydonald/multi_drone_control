#include "dynamic_planner/trajectory_refinement.hpp"

#include "dynamic_planner/minvo.hpp"
#include "dynamic_planner/separator.hpp"

#include <qpOASES.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dynamic_planner {
namespace {

constexpr double kQpInfinity = 1.0e20;

int flatIndex(int control_index, int axis) {
    return 3 * control_index + axis;
}

void validateConfig(const RefinementConfig& config) {
    if (!config.v_max.allFinite() || !(config.v_max.array() > 0.0).all() ||
        !config.a_max.allFinite() || !(config.a_max.array() > 0.0).all() ||
        !config.j_max.allFinite() || !(config.j_max.array() > 0.0).all()) {
        throw std::invalid_argument("v_max, a_max and j_max must be finite and positive");
    }
    if (!(config.jerk_weight >= 0.0) || !(config.goal_weight > 0.0) ||
        !(config.goal_acceptance_m >= 0.0) || !(config.separator_margin_m >= 0.0) ||
        !(config.validation_tolerance > 0.0) || config.max_working_set_recalculations < 1) {
        throw std::invalid_argument("invalid refinement scalar configuration");
    }
    if (!config.xyz_min.allFinite() || !config.xyz_max.allFinite() ||
        !(config.xyz_max.array() > config.xyz_min.array()).all()) {
        throw std::invalid_argument("xyz_max must exceed xyz_min componentwise");
    }
}

Eigen::VectorXd flattenControlPoints(const ControlPoints& cps) {
    Eigen::VectorXd result(3 * cps.rows());
    for (Eigen::Index i = 0; i < cps.rows(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            result(flatIndex(static_cast<int>(i), axis)) = cps(i, axis);
        }
    }
    return result;
}

ControlPoints unflattenControlPoints(const Eigen::VectorXd& x) {
    if (x.size() % 3 != 0) {
        throw std::invalid_argument("control-point vector size must be divisible by three");
    }
    ControlPoints cps(x.size() / 3, 3);
    for (Eigen::Index i = 0; i < cps.rows(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            cps(i, axis) = x(flatIndex(static_cast<int>(i), axis));
        }
    }
    return cps;
}

std::vector<double> rowMajorData(const Eigen::MatrixXd& matrix) {
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(matrix.rows() * matrix.cols()));
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            values.push_back(matrix(row, col));
        }
    }
    return values;
}

std::pair<double, double> objectiveTerms(const ControlPoints& cps,
                                         const std::vector<double>& knots,
                                         const Vec3& goal,
                                         const RefinementConfig& config) {
    const Eigen::MatrixXd d3 = derivativeControlMatrix(
        knots, static_cast<int>(cps.rows()), 3);
    const ControlPoints jerks = d3 * cps;
    const int num_segments = static_cast<int>(cps.rows()) - 3;
    double jerk_cost = 0.0;
    for (int interval = 0; interval < num_segments; ++interval) {
        const double duration = knots.at(static_cast<std::size_t>(kCubicDegree + interval + 1))
                              - knots.at(static_cast<std::size_t>(kCubicDegree + interval));
        jerk_cost += duration * jerks.row(interval).squaredNorm();
    }
    const double goal_cost = (cps.row(cps.rows() - 1).transpose() - goal).squaredNorm();
    return {config.jerk_weight * jerk_cost + config.goal_weight * goal_cost, jerk_cost};
}

void appendDerivativeInequalities(const Eigen::MatrixXd& op,
                                  const Vec3& limits,
                                  int num_control_points,
                                  std::vector<Eigen::VectorXd>& rows,
                                  std::vector<double>& rhs) {
    const int nvar = 3 * num_control_points;
    for (Eigen::Index r = 0; r < op.rows(); ++r) {
        for (int axis = 0; axis < 3; ++axis) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            for (int cp = 0; cp < num_control_points; ++cp) {
                row(flatIndex(cp, axis)) = op(r, cp);
            }
            rows.push_back(row);
            rhs.push_back(limits(axis));
            rows.push_back(-row);
            rhs.push_back(limits(axis));
        }
    }
}

void appendFixedSeparatorInequalities(const std::vector<IntervalSeparator>& separators,
                                      int num_control_points,
                                      int num_segments,
                                      double margin_m,
                                      std::vector<Eigen::VectorXd>& rows,
                                      std::vector<double>& rhs) {
    const auto converters = positionConverters(num_segments);
    const int nvar = 3 * num_control_points;
    for (const auto& item : separators) {
        if (!item.result.feasible || !item.result.plane.has_value()) {
            throw std::invalid_argument("R4 separator is not feasible");
        }
        const int interval = item.interval_index;
        if (interval < 0 || interval >= num_segments) {
            throw std::invalid_argument("separator interval index out of range");
        }
        const SeparatingPlane& plane = *item.result.plane;
        const Vec3 normal = plane.normal;
        const double bound = 1.0 - plane.offset - margin_m * normal.norm();
        const Matrix4& converter = converters.at(static_cast<std::size_t>(interval));
        for (int vertex = 0; vertex < 4; ++vertex) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            for (int local_cp = 0; local_cp < 4; ++local_cp) {
                const double coeff = converter(local_cp, vertex);
                const int global_cp = interval + local_cp;
                for (int axis = 0; axis < 3; ++axis) {
                    row(flatIndex(global_cp, axis)) += coeff * normal(axis);
                }
            }
            rows.push_back(row);
            rhs.push_back(bound);
        }
    }
}

struct SeparatorPostcheck {
    bool passed = false;
    double min_slack = -std::numeric_limits<double>::infinity();
};

SeparatorPostcheck separatorPostcheck(const ControlPoints& cps,
                                      const OctopusResult& raw_result,
                                      const std::vector<TimeIndexedObstacle>& obstacles,
                                      double margin_m) {
    const int num_segments = static_cast<int>(cps.rows()) - 3;
    const auto converters = positionConverters(num_segments);
    Separator separator(1e-7);
    double min_slack = std::numeric_limits<double>::infinity();

    for (const auto& item : raw_result.separators) {
        const auto obstacle_it = std::find_if(
            obstacles.begin(), obstacles.end(),
            [&item](const TimeIndexedObstacle& obstacle) {
                return obstacle.name == item.obstacle_name;
            });
        if (obstacle_it == obstacles.end() || !item.result.plane.has_value()) {
            return {false, -std::numeric_limits<double>::infinity()};
        }
        const int interval = item.interval_index;
        if (interval < 0 || interval >= num_segments) {
            return {false, -std::numeric_limits<double>::infinity()};
        }

        FourPoints q;
        q = cps.middleRows(interval, 4);
        const FourPoints vertices = converters.at(static_cast<std::size_t>(interval)).transpose() * q;
        const SeparatingPlane& plane = *item.result.plane;
        const Eigen::VectorXd values = vertices * plane.normal
                                     + Eigen::VectorXd::Constant(4, plane.offset);
        const double allowed = 1.0 - margin_m * plane.normal.norm();
        min_slack = std::min(min_slack, (Eigen::VectorXd::Constant(4, allowed) - values).minCoeff());
        if (values.maxCoeff() > allowed + 2e-6) {
            return {false, min_slack};
        }

        const SeparationResult recheck = separator.solve(
            vertices,
            obstacle_it->interval_vertices.at(static_cast<std::size_t>(interval)));
        if (!recheck.feasible) {
            return {false, min_slack};
        }
    }
    return {true, min_slack};
}

std::string qpReturnString(qpOASES::returnValue code) {
    return std::string("qpOASES return code ") + std::to_string(static_cast<int>(code));
}

}  // namespace

Eigen::MatrixXd derivativeControlMatrix(const std::vector<double>& knots,
                                        int num_control_points,
                                        int derivative_order,
                                        int degree) {
    if (degree != kCubicDegree) {
        throw std::invalid_argument("R6.3A.4 supports cubic splines only");
    }
    if (derivative_order < 0 || derivative_order > degree) {
        throw std::invalid_argument("derivative_order must lie in [0, degree]");
    }
    if (static_cast<int>(knots.size()) != num_control_points + degree + 1) {
        throw std::invalid_argument("knot/control-point dimensions are inconsistent");
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(num_control_points, num_control_points);
    std::vector<double> current_knots = knots;
    int current_degree = degree;
    for (int order = 0; order < derivative_order; ++order) {
        Eigen::MatrixXd next(matrix.rows() - 1, matrix.cols());
        for (Eigen::Index i = 0; i < next.rows(); ++i) {
            const double denom = current_knots.at(static_cast<std::size_t>(i + current_degree + 1))
                               - current_knots.at(static_cast<std::size_t>(i + 1));
            if (!(denom > 0.0)) {
                throw std::invalid_argument("degenerate derivative knot interval");
            }
            next.row(i) = (static_cast<double>(current_degree) / denom)
                        * (matrix.row(i + 1) - matrix.row(i));
        }
        matrix = std::move(next);
        current_knots = std::vector<double>(current_knots.begin() + 1, current_knots.end() - 1);
        --current_degree;
    }
    return matrix;
}

Eigen::MatrixXd minvoVelocityControlMatrix(const std::vector<double>& knots,
                                           int num_control_points) {
    const int num_segments = num_control_points - 3;
    if (num_segments < 2) {
        throw std::invalid_argument("MINVO velocity refinement requires at least two segments");
    }
    const Eigen::MatrixXd d1 = derivativeControlMatrix(knots, num_control_points, 1);
    const auto converters = velocityConverters(num_segments);
    Eigen::MatrixXd result(3 * num_segments, num_control_points);
    for (int interval = 0; interval < num_segments; ++interval) {
        const Eigen::MatrixXd local = d1.middleRows(interval, 3);
        result.middleRows(3 * interval, 3) =
            converters.at(static_cast<std::size_t>(interval)).transpose() * local;
    }
    return result;
}

Eigen::MatrixXd rmaderMinvoVelocityConstraintMatrix(const std::vector<double>& knots,
                                                     int num_control_points) {
    const Eigen::MatrixXd full = minvoVelocityControlMatrix(knots, num_control_points);
    if (full.rows() < 6) {
        throw std::invalid_argument("RMADER MINVO velocity constraints require at least two segments");
    }
    return full.topRows(full.rows() - 3);
}

QuadraticProgram buildRefinementQP(const OctopusResult& raw_result,
                                   const std::vector<double>& knots,
                                   const Vec3& goal,
                                   const RefinementConfig& config,
                                   const TerminalBoundary& terminal_boundary) {
    validateConfig(config);
    if (!raw_result.success || !raw_result.control_points.has_value()) {
        throw std::invalid_argument("refinement requires a successful complete R4 result");
    }
    const ControlPoints& cps = *raw_result.control_points;
    const int ncp = static_cast<int>(cps.rows());
    const int nvar = 3 * ncp;
    const int num_segments = ncp - 3;
    if (static_cast<int>(knots.size()) != ncp + kCubicDegree + 1) {
        throw std::invalid_argument("knot/control-point dimensions are inconsistent");
    }

    const Eigen::MatrixXd jerk_op = derivativeControlMatrix(knots, ncp, 3);
    if (jerk_op.rows() != num_segments) {
        throw std::runtime_error("cubic jerk control count must equal interval count");
    }
    Eigen::VectorXd durations(num_segments);
    for (int interval = 0; interval < num_segments; ++interval) {
        durations(interval) = knots.at(static_cast<std::size_t>(kCubicDegree + interval + 1))
                            - knots.at(static_cast<std::size_t>(kCubicDegree + interval));
    }
    const Eigen::MatrixXd h_cp = 2.0 * config.jerk_weight
        * (jerk_op.transpose() * durations.asDiagonal() * jerk_op);

    QuadraticProgram qp;
    qp.hessian = Eigen::MatrixXd::Zero(nvar, nvar);
    qp.linear = Eigen::VectorXd::Zero(nvar);
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < ncp; ++i) {
            for (int j = 0; j < ncp; ++j) {
                qp.hessian(flatIndex(i, axis), flatIndex(j, axis)) += h_cp(i, j);
            }
        }
        const int terminal = flatIndex(ncp - 1, axis);
        qp.hessian(terminal, terminal) += 2.0 * config.goal_weight;
        qp.linear(terminal) += -2.0 * config.goal_weight * goal(axis);
    }
    qp.constant = config.goal_weight * goal.squaredNorm();

    std::vector<Eigen::VectorXd> eq_rows;
    std::vector<double> eq_rhs;
    for (int cp = 0; cp < 3; ++cp) {
        for (int axis = 0; axis < 3; ++axis) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            row(flatIndex(cp, axis)) = 1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(cps(cp, axis));
        }
    }
    if (terminal_boundary.mode == TerminalMode::Stopped) {
        for (int axis = 0; axis < 3; ++axis) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            row(flatIndex(ncp - 1, axis)) = 1.0;
            row(flatIndex(ncp - 2, axis)) = -1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(0.0);

            row.setZero();
            row(flatIndex(ncp - 2, axis)) = 1.0;
            row(flatIndex(ncp - 3, axis)) = -1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(0.0);
        }
    } else if (terminal_boundary.mode == TerminalMode::Continuation) {
        if (!terminal_boundary.position.allFinite()) {
            throw std::invalid_argument("continuation terminal position must be finite");
        }
        // Receding-horizon continuation: hit the local waypoint but let the QP
        // choose terminal velocity subject to the same v/a/j and corridor
        // constraints. This avoids forcing an intermediate point either to hover
        // or to impersonate the moving platform's terminal velocity.
        for (int axis = 0; axis < 3; ++axis) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            row(flatIndex(ncp - 1, axis)) = 1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(terminal_boundary.position(axis));
        }
    } else {
        if (!terminal_boundary.position.allFinite() || !terminal_boundary.velocity.allFinite()) {
            throw std::invalid_argument("moving-rendezvous terminal boundary must be finite");
        }
        const int last_cp = ncp - 1;
        const double derivative_span =
            knots.at(static_cast<std::size_t>(last_cp + kCubicDegree))
            - knots.at(static_cast<std::size_t>(last_cp));
        if (!std::isfinite(derivative_span) || !(derivative_span > 0.0)) {
            throw std::invalid_argument("invalid moving-rendezvous terminal derivative span");
        }
        for (int axis = 0; axis < 3; ++axis) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(nvar);
            row(flatIndex(ncp - 1, axis)) = 1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(terminal_boundary.position(axis));

            row.setZero();
            row(flatIndex(ncp - 1, axis)) = 1.0;
            row(flatIndex(ncp - 2, axis)) = -1.0;
            eq_rows.push_back(row);
            eq_rhs.push_back(
                terminal_boundary.velocity(axis)
                * derivative_span / static_cast<double>(kCubicDegree));
        }
    }

    qp.a_eq.resize(static_cast<Eigen::Index>(eq_rows.size()), nvar);
    qp.b_eq.resize(static_cast<Eigen::Index>(eq_rhs.size()));
    for (std::size_t i = 0; i < eq_rows.size(); ++i) {
        qp.a_eq.row(static_cast<Eigen::Index>(i)) = eq_rows[i].transpose();
        qp.b_eq(static_cast<Eigen::Index>(i)) = eq_rhs[i];
    }

    std::vector<Eigen::VectorXd> ub_rows;
    std::vector<double> ub_rhs;
    appendFixedSeparatorInequalities(raw_result.separators, ncp, num_segments,
                                     config.separator_margin_m, ub_rows, ub_rhs);
    // The historical RMADER matrix omits the final velocity interval because
    // the legacy terminal contract repeats the final control points and stops.
    // Non-stopped C1F.8d terminal modes must constrain the full derivative
    // spline, including the final interval containing free/moving terminal v.
    const Eigen::MatrixXd velocity_constraints =
        terminal_boundary.mode == TerminalMode::Stopped
            ? rmaderMinvoVelocityConstraintMatrix(knots, ncp)
            : minvoVelocityControlMatrix(knots, ncp);
    appendDerivativeInequalities(velocity_constraints,
                                 config.v_max, ncp, ub_rows, ub_rhs);
    appendDerivativeInequalities(derivativeControlMatrix(knots, ncp, 2),
                                 config.a_max, ncp, ub_rows, ub_rhs);
    appendDerivativeInequalities(derivativeControlMatrix(knots, ncp, 3),
                                 config.j_max, ncp, ub_rows, ub_rhs);

    qp.a_ub.resize(static_cast<Eigen::Index>(ub_rows.size()), nvar);
    qp.b_ub.resize(static_cast<Eigen::Index>(ub_rhs.size()));
    for (std::size_t i = 0; i < ub_rows.size(); ++i) {
        qp.a_ub.row(static_cast<Eigen::Index>(i)) = ub_rows[i].transpose();
        qp.b_ub(static_cast<Eigen::Index>(i)) = ub_rhs[i];
    }

    qp.lower_bounds.resize(nvar);
    qp.upper_bounds.resize(nvar);
    for (int cp = 0; cp < ncp; ++cp) {
        for (int axis = 0; axis < 3; ++axis) {
            qp.lower_bounds(flatIndex(cp, axis)) = config.xyz_min(axis);
            qp.upper_bounds(flatIndex(cp, axis)) = config.xyz_max(axis);
        }
    }
    return qp;
}

RefinementResult refineOctopusTrajectory(const OctopusResult& raw_result,
                                         const std::vector<double>& knots,
                                         const Vec3& goal,
                                         const std::vector<TimeIndexedObstacle>& obstacles,
                                         const RefinementConfig& config,
                                         const TerminalBoundary& terminal_boundary) {
    if (!raw_result.success || !raw_result.control_points.has_value()) {
        throw std::invalid_argument("refinement requires a successful R4 trajectory");
    }
    const ControlPoints& raw_cps = *raw_result.control_points;
    RefinementResult output;
    output.raw_goal_distance_m = (raw_cps.row(raw_cps.rows() - 1).transpose() - goal).norm();
    const auto [raw_objective, raw_jerk] = objectiveTerms(raw_cps, knots, goal, config);
    output.raw_objective = raw_objective;
    output.raw_jerk_cost = raw_jerk;
    output.qp = buildRefinementQP(raw_result, knots, goal, config, terminal_boundary);

    const QuadraticProgram& qp = output.qp;
    const int nvar = static_cast<int>(qp.linear.size());
    const int neq = static_cast<int>(qp.a_eq.rows());
    const int nub = static_cast<int>(qp.a_ub.rows());
    const int ncon = neq + nub;

    Eigen::MatrixXd combined(ncon, nvar);
    Eigen::VectorXd lower_a(ncon);
    Eigen::VectorXd upper_a(ncon);
    if (neq > 0) {
        combined.topRows(neq) = qp.a_eq;
        lower_a.head(neq) = qp.b_eq;
        upper_a.head(neq) = qp.b_eq;
    }
    if (nub > 0) {
        combined.bottomRows(nub) = qp.a_ub;
        lower_a.tail(nub).setConstant(-kQpInfinity);
        upper_a.tail(nub) = qp.b_ub;
    }

    const std::vector<double> h_data = rowMajorData(qp.hessian);
    const std::vector<double> a_data = rowMajorData(combined);
    std::vector<double> g_data(static_cast<std::size_t>(nvar));
    std::vector<double> lb_data(static_cast<std::size_t>(nvar));
    std::vector<double> ub_data(static_cast<std::size_t>(nvar));
    std::vector<double> lba_data(static_cast<std::size_t>(ncon));
    std::vector<double> uba_data(static_cast<std::size_t>(ncon));
    for (int i = 0; i < nvar; ++i) {
        g_data[static_cast<std::size_t>(i)] = qp.linear(i);
        lb_data[static_cast<std::size_t>(i)] = qp.lower_bounds(i);
        ub_data[static_cast<std::size_t>(i)] = qp.upper_bounds(i);
    }
    for (int i = 0; i < ncon; ++i) {
        lba_data[static_cast<std::size_t>(i)] = lower_a(i);
        uba_data[static_cast<std::size_t>(i)] = upper_a(i);
    }

    qpOASES::QProblem solver(nvar, ncon);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    solver.setOptions(options);
    int n_wsr = config.max_working_set_recalculations;

    const auto started = std::chrono::steady_clock::now();
    const qpOASES::returnValue code = solver.init(
        h_data.data(), g_data.data(), a_data.data(),
        lb_data.data(), ub_data.data(), lba_data.data(), uba_data.data(), n_wsr);
    output.solve_time_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    output.working_set_recalculations = n_wsr;

    if (code != qpOASES::SUCCESSFUL_RETURN) {
        output.status = "SOLVER_FAILED";
        output.message = qpReturnString(code);
        return output;
    }

    std::vector<double> solution(static_cast<std::size_t>(nvar), 0.0);
    const qpOASES::returnValue solution_code = solver.getPrimalSolution(solution.data());
    if (solution_code != qpOASES::SUCCESSFUL_RETURN) {
        output.status = "SOLVER_FAILED";
        output.message = qpReturnString(solution_code);
        return output;
    }
    Eigen::VectorXd x(nvar);
    for (int i = 0; i < nvar; ++i) {
        x(i) = solution[static_cast<std::size_t>(i)];
    }
    const ControlPoints candidate = unflattenControlPoints(x);

    const double eq_residual = neq > 0
        ? (qp.a_eq * x - qp.b_eq).cwiseAbs().maxCoeff() : 0.0;
    const double ub_violation = nub > 0
        ? (qp.a_ub * x - qp.b_ub).cwiseMax(0.0).maxCoeff() : 0.0;
    const double lower_violation = (qp.lower_bounds - x).cwiseMax(0.0).maxCoeff();
    const double upper_violation = (x - qp.upper_bounds).cwiseMax(0.0).maxCoeff();
    const double max_ineq_violation = std::max({ub_violation, lower_violation, upper_violation});

    const Eigen::MatrixXd velocity_postcheck_matrix =
        terminal_boundary.mode == TerminalMode::Stopped
            ? rmaderMinvoVelocityConstraintMatrix(knots, static_cast<int>(candidate.rows()))
            : minvoVelocityControlMatrix(knots, static_cast<int>(candidate.rows()));
    const ControlPoints v_minvo = velocity_postcheck_matrix * candidate;
    const ControlPoints accel = derivativeControlMatrix(
        knots, static_cast<int>(candidate.rows()), 2) * candidate;
    const ControlPoints jerk = derivativeControlMatrix(
        knots, static_cast<int>(candidate.rows()), 3) * candidate;
    output.max_velocity_axis = v_minvo.cwiseAbs().maxCoeff();
    output.max_acceleration_axis = accel.cwiseAbs().maxCoeff();
    output.max_jerk_axis = jerk.cwiseAbs().maxCoeff();
    output.max_equality_residual = eq_residual;
    output.max_inequality_violation = max_ineq_violation;

    const SeparatorPostcheck separator_check = separatorPostcheck(
        candidate, raw_result, obstacles, config.separator_margin_m);
    output.separator_recheck_passed = separator_check.passed;
    output.min_fixed_separator_slack = separator_check.min_slack;

    const bool finite = candidate.allFinite();
    const bool numerically_valid = eq_residual <= config.validation_tolerance
        && max_ineq_violation <= config.validation_tolerance
        && separator_check.passed;
    output.success = finite && numerically_valid;
    output.refined_goal_distance_m =
        (candidate.row(candidate.rows() - 1).transpose() - goal).norm();
    output.goal_accepted = output.success
        && *output.refined_goal_distance_m <= config.goal_acceptance_m;
    const auto [refined_objective, refined_jerk] = objectiveTerms(candidate, knots, goal, config);
    output.refined_objective = refined_objective;
    output.refined_jerk_cost = refined_jerk;

    std::ostringstream message;
    message << qpReturnString(code)
            << "; eq_res=" << eq_residual
            << "; ineq_violation=" << max_ineq_violation
            << "; fixed_separator_ok=" << std::boolalpha << separator_check.passed;
    output.message = message.str();
    output.status = output.success ? "SUCCESS" : "POSTCHECK_FAILED";
    if (output.success) {
        output.control_points = candidate;
    }
    return output;
}

}  // namespace dynamic_planner
