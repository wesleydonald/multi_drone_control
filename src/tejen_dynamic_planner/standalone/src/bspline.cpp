#include "dynamic_planner/bspline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {
namespace {

void validateSplineShape(const ControlPoints& control_points,
                         const std::vector<double>& knots,
                         int degree) {
    if (degree < 0) {
        throw std::invalid_argument("degree must be non-negative");
    }
    if (control_points.cols() != 3 || control_points.rows() < degree + 1) {
        throw std::invalid_argument("invalid control-point shape for spline degree");
    }
    const std::size_t expected =
        static_cast<std::size_t>(control_points.rows() + degree + 1);
    if (knots.size() != expected) {
        throw std::invalid_argument("knot count does not match control-point count and degree");
    }
    for (std::size_t i = 1; i < knots.size(); ++i) {
        if (knots[i] < knots[i - 1]) {
            throw std::invalid_argument("knots must be nondecreasing");
        }
    }
}

int findSpan(const std::vector<double>& knots, int num_control_points,
             int degree, double t) {
    const int n = num_control_points - 1;
    const double t_start = knots.at(static_cast<std::size_t>(degree));
    const double t_end = knots.at(static_cast<std::size_t>(n + 1));

    if (t <= t_start) {
        return degree;
    }
    if (t >= t_end) {
        return n;
    }

    int low = degree;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (t < knots.at(static_cast<std::size_t>(mid)) ||
           t >= knots.at(static_cast<std::size_t>(mid + 1))) {
        if (t < knots.at(static_cast<std::size_t>(mid))) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

}  // namespace

std::vector<double> openUniformKnots(double t_start, double t_end,
                                     int num_segments, int degree) {
    if (degree != kCubicDegree) {
        throw std::invalid_argument("R6.3A.1 supports cubic splines only");
    }
    if (num_segments < 1) {
        throw std::invalid_argument("num_segments must be positive");
    }
    if (!std::isfinite(t_start) || !std::isfinite(t_end) || t_end <= t_start) {
        throw std::invalid_argument("expected finite t_end > t_start");
    }

    const int num_control_points = num_segments + degree;
    const int knot_count = num_control_points + degree + 1;
    std::vector<double> knots(static_cast<std::size_t>(knot_count), t_start);

    for (int i = knot_count - degree - 1; i < knot_count; ++i) {
        knots.at(static_cast<std::size_t>(i)) = t_end;
    }

    if (num_segments > 1) {
        const double dt = (t_end - t_start) / static_cast<double>(num_segments);
        for (int i = 1; i < num_segments; ++i) {
            knots.at(static_cast<std::size_t>(degree + i)) =
                t_start + dt * static_cast<double>(i);
        }
    }
    return knots;
}

ControlPoints initialControlPointsFromState(const State& state,
                                            const std::vector<double>& knots,
                                            int degree) {
    if (degree != kCubicDegree) {
        throw std::invalid_argument("only cubic B-splines are supported");
    }
    if (knots.size() < static_cast<std::size_t>(2 * degree + 3)) {
        throw std::invalid_argument("knot vector is too short");
    }

    const double p = static_cast<double>(degree);
    const double t1 = knots.at(1);
    const double t2 = knots.at(2);
    const double tp1 = knots.at(static_cast<std::size_t>(degree + 1));
    const double t1p1 = knots.at(static_cast<std::size_t>(degree + 2));

    const Vec3 q0 = state.position;
    const Vec3 q1 = state.position + (-t1 + tp1) * state.velocity / p;
    const Vec3 q2 =
        (p * p * q1
         - (t1p1 - t2) * (state.acceleration * (t2 - tp1) + state.velocity)
         - p * (q1 + (-t1p1 + t2) * state.velocity))
        / ((p - 1.0) * p);

    ControlPoints result(3, 3);
    result.row(0) = q0.transpose();
    result.row(1) = q1.transpose();
    result.row(2) = q2.transpose();
    return result;
}

ControlPoints stoppingCompletion(const ControlPoints& prefix, int num_segments,
                                 int degree) {
    if (prefix.cols() != 3) {
        throw std::invalid_argument("prefix must have three columns");
    }
    const int expected_prefix = num_segments + degree - 2;
    if (prefix.rows() != expected_prefix) {
        throw std::invalid_argument("prefix does not contain q0 through q_(N-2)");
    }

    ControlPoints result(prefix.rows() + 2, 3);
    result.topRows(prefix.rows()) = prefix;
    result.row(prefix.rows()) = prefix.row(prefix.rows() - 1);
    result.row(prefix.rows() + 1) = prefix.row(prefix.rows() - 1);
    return result;
}


namespace {

double terminalDerivativeSpan(const std::vector<double>& knots,
                              int num_control_points,
                              int degree) {
    if (degree != kCubicDegree || num_control_points < degree + 1 ||
        static_cast<int>(knots.size()) != num_control_points + degree + 1) {
        throw std::invalid_argument("invalid terminal-boundary knot/control-point dimensions");
    }
    const int last_cp = num_control_points - 1;
    const double denom = knots.at(static_cast<std::size_t>(last_cp + degree))
                       - knots.at(static_cast<std::size_t>(last_cp));
    if (!std::isfinite(denom) || !(denom > 0.0)) {
        throw std::invalid_argument("degenerate terminal derivative knot span");
    }
    return denom;
}

void validateTerminalBoundary(const TerminalBoundary& boundary) {
    if (!boundary.position.allFinite() || !boundary.velocity.allFinite()) {
        throw std::invalid_argument("terminal boundary must be finite");
    }
}

}  // namespace

TerminalTailControlPoints terminalTailControlPoints(
    const Vec3& search_terminal_control_point,
    const std::vector<double>& knots,
    int num_segments,
    const TerminalBoundary& boundary,
    int degree) {
    if (boundary.mode == TerminalMode::Stopped) {
        return TerminalTailControlPoints{
            search_terminal_control_point,
            search_terminal_control_point,
        };
    }
    if (boundary.mode == TerminalMode::Continuation) {
        throw std::invalid_argument(
            "continuation terminal velocity is free; q_(N-1) must be searched/refined explicitly");
    }

    validateTerminalBoundary(boundary);
    const int num_control_points = num_segments + degree;
    const double derivative_span = terminalDerivativeSpan(
        knots, num_control_points, degree);
    return TerminalTailControlPoints{
        boundary.position
            - boundary.velocity * (derivative_span / static_cast<double>(degree)),
        boundary.position,
    };
}

ControlPoints terminalCompletion(const ControlPoints& prefix,
                                 const std::vector<double>& knots,
                                 int num_segments,
                                 const TerminalBoundary& boundary,
                                 int degree) {
    if (prefix.cols() != 3) {
        throw std::invalid_argument("prefix must have three columns");
    }
    const int expected_prefix = num_segments + degree - 2;
    if (prefix.rows() != expected_prefix) {
        throw std::invalid_argument("prefix does not contain q0 through q_(N-2)");
    }
    ControlPoints result(prefix.rows() + 2, 3);
    result.topRows(prefix.rows()) = prefix;
    const TerminalTailControlPoints tail = terminalTailControlPoints(
        prefix.row(prefix.rows() - 1).transpose(), knots, num_segments, boundary, degree);
    result.row(result.rows() - 2) = tail.penultimate.transpose();
    result.row(result.rows() - 1) = tail.endpoint.transpose();
    return result;
}

Vec3 terminalSearchPrecursor(const std::vector<double>& knots,
                             int num_control_points,
                             const TerminalBoundary& boundary,
                             int degree) {
    validateTerminalBoundary(boundary);
    if (boundary.mode == TerminalMode::Stopped) {
        return boundary.position;
    }
    const double derivative_span = terminalDerivativeSpan(
        knots, num_control_points, degree);
    // For a clamped cubic, choosing q_(N-2)=p_f-v_f*Delta makes the
    // terminal acceleration zero when q_(N-1)=p_f-v_f*Delta/3 and q_N=p_f.
    // This is only a search heuristic target; terminal acceleration remains
    // unconstrained in the refinement QP.
    return boundary.position - boundary.velocity * derivative_span;
}

DerivativeSpline derivativeSpline(const ControlPoints& control_points,
                                  const std::vector<double>& knots,
                                  int degree,
                                  int derivative_order) {
    validateSplineShape(control_points, knots, degree);
    if (derivative_order < 0 || derivative_order > degree) {
        throw std::invalid_argument("derivative_order must be between 0 and degree");
    }

    DerivativeSpline result{control_points, knots, degree};
    for (int order = 0; order < derivative_order; ++order) {
        ControlPoints next(result.control_points.rows() - 1, 3);
        for (Eigen::Index i = 0; i < next.rows(); ++i) {
            const std::size_t upper = static_cast<std::size_t>(i + result.degree + 1);
            const std::size_t lower = static_cast<std::size_t>(i + 1);
            const double denom = result.knots.at(upper) - result.knots.at(lower);
            if (denom <= 0.0) {
                throw std::invalid_argument("degenerate derivative knot interval");
            }
            next.row(i) = (static_cast<double>(result.degree) / denom)
                          * (result.control_points.row(i + 1) - result.control_points.row(i));
        }
        result.control_points = std::move(next);
        result.knots = std::vector<double>(result.knots.begin() + 1,
                                           result.knots.end() - 1);
        --result.degree;
    }
    return result;
}

Vec3 evaluateBSpline(const ControlPoints& control_points,
                     const std::vector<double>& knots,
                     int degree,
                     double t) {
    validateSplineShape(control_points, knots, degree);
    if (!std::isfinite(t)) {
        throw std::invalid_argument("evaluation time must be finite");
    }

    const int num_control_points = static_cast<int>(control_points.rows());
    const int n = num_control_points - 1;
    const double t_start = knots.at(static_cast<std::size_t>(degree));
    const double t_end = knots.at(static_cast<std::size_t>(n + 1));
    const double tc = std::clamp(t, t_start, t_end);
    const int span = findSpan(knots, num_control_points, degree, tc);

    std::vector<Vec3> d(static_cast<std::size_t>(degree + 1), Vec3::Zero());
    for (int j = 0; j <= degree; ++j) {
        d.at(static_cast<std::size_t>(j)) =
            control_points.row(span - degree + j).transpose();
    }

    for (int r = 1; r <= degree; ++r) {
        for (int j = degree; j >= r; --j) {
            const int i = span - degree + j;
            const double lower = knots.at(static_cast<std::size_t>(i));
            const double upper = knots.at(static_cast<std::size_t>(i + degree - r + 1));
            const double denom = upper - lower;
            const double alpha = denom > 0.0 ? (tc - lower) / denom : 0.0;
            d.at(static_cast<std::size_t>(j)) =
                (1.0 - alpha) * d.at(static_cast<std::size_t>(j - 1))
                + alpha * d.at(static_cast<std::size_t>(j));
        }
    }
    return d.at(static_cast<std::size_t>(degree));
}

State evaluateCubicState(const ControlPoints& control_points,
                         const std::vector<double>& knots,
                         double t) {
    validateSplineShape(control_points, knots, kCubicDegree);
    const auto velocity = derivativeSpline(control_points, knots, kCubicDegree, 1);
    const auto acceleration = derivativeSpline(control_points, knots, kCubicDegree, 2);
    return State{
        evaluateBSpline(control_points, knots, kCubicDegree, t),
        evaluateBSpline(velocity.control_points, velocity.knots, velocity.degree, t),
        evaluateBSpline(acceleration.control_points, acceleration.knots, acceleration.degree, t),
    };
}

}  // namespace dynamic_planner
