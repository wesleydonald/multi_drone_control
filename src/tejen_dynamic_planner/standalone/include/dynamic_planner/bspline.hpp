#pragma once

#include <Eigen/Core>
#include <vector>

#include "dynamic_planner/types.hpp"

namespace dynamic_planner {

constexpr int kCubicDegree = 3;

enum class TerminalMode {
    Stopped,
    Continuation,
    MovingRendezvous,
};

struct TerminalBoundary {
    TerminalMode mode = TerminalMode::Stopped;
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
};

// The two control points appended after the searched q_(N-2) prefix.  This is
// the single source of truth for the terminal geometry used by both Octopus
// incremental checks and final certification.
struct TerminalTailControlPoints {
    Vec3 penultimate = Vec3::Zero();
    Vec3 endpoint = Vec3::Zero();
};

using ControlPoints = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

std::vector<double> openUniformKnots(double t_start, double t_end, int num_segments,
                                     int degree = kCubicDegree);

ControlPoints initialControlPointsFromState(const State& state,
                                            const std::vector<double>& knots,
                                            int degree = kCubicDegree);

ControlPoints stoppingCompletion(const ControlPoints& prefix, int num_segments,
                                 int degree = kCubicDegree);

// Construct q_(N-1) and q_N after q_(N-2). Stopped mode preserves the
// validated repeated-control-point tail. Moving-rendezvous mode hard-matches
// the requested endpoint position and velocity. Continuation has a fixed
// endpoint but a free penultimate control point; Octopus handles that mode
// directly rather than through this two-point deterministic tail helper.
TerminalTailControlPoints terminalTailControlPoints(
    const Vec3& search_terminal_control_point,
    const std::vector<double>& knots,
    int num_segments,
    const TerminalBoundary& boundary,
    int degree = kCubicDegree);

// Complete q0..q_(N-2) with the selected terminal contract. Stopped mode
// preserves the validated repeated-control-point completion. Moving-rendezvous
// mode hard-matches endpoint position and velocity while leaving q_(N-2) free,
// so endpoint acceleration remains a result of the search/refinement rather than
// being hard-matched to target acceleration.
ControlPoints terminalCompletion(const ControlPoints& prefix,
                                 const std::vector<double>& knots,
                                 int num_segments,
                                 const TerminalBoundary& boundary,
                                 int degree = kCubicDegree);

// Search heuristic target for q_(N-2). In moving-rendezvous mode this is the
// zero-terminal-acceleration precursor implied by p_f/v_f; it is only a search
// target, not an acceleration equality constraint.
Vec3 terminalSearchPrecursor(const std::vector<double>& knots,
                             int num_control_points,
                             const TerminalBoundary& boundary,
                             int degree = kCubicDegree);

struct DerivativeSpline {
    ControlPoints control_points;
    std::vector<double> knots;
    int degree = 0;
};

DerivativeSpline derivativeSpline(const ControlPoints& control_points,
                                  const std::vector<double>& knots,
                                  int degree,
                                  int derivative_order);

Vec3 evaluateBSpline(const ControlPoints& control_points,
                     const std::vector<double>& knots,
                     int degree,
                     double t);

State evaluateCubicState(const ControlPoints& control_points,
                         const std::vector<double>& knots,
                         double t);

}  // namespace dynamic_planner
