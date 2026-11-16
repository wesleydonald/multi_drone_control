#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/octopus_search.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusConfig;
using dynamic_planner::OctopusSearch;
using dynamic_planner::State;
using dynamic_planner::TerminalBoundary;
using dynamic_planner::TerminalMode;
using dynamic_planner::TimeIndexedObstacle;
using dynamic_planner::Vec3;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(const Vec3& actual,
                 const Vec3& expected,
                 double tolerance,
                 const std::string& message) {
    if ((actual - expected).cwiseAbs().maxCoeff() > tolerance) {
        throw std::runtime_error(message);
    }
}

void requireMatrixNear(const ControlPoints& actual,
                       const ControlPoints& expected,
                       double tolerance,
                       const std::string& message) {
    require(actual.rows() == expected.rows() && actual.cols() == expected.cols(),
            message + " shape mismatch");
    require((actual - expected).cwiseAbs().maxCoeff() <= tolerance, message);
}

OctopusConfig baseConfig() {
    OctopusConfig config;
    config.v_max = Vec3::Constant(1.0);
    config.a_max = Vec3::Constant(1.5);
    config.samples_per_axis = {9, 9, 9};
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

Eigen::MatrixXd box(const Vec3& centre, double half_extent) {
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (int x : {-1, 1}) {
        for (int y : {-1, 1}) {
            for (int z : {-1, 1}) {
                vertices.row(row++) =
                    (centre + half_extent * Vec3(x, y, z)).transpose();
            }
        }
    }
    return vertices;
}

void requireTerminalDynamicsWithinLimits(const ControlPoints& control_points,
                                         const std::vector<double>& knots,
                                         const OctopusConfig& config,
                                         const std::string& label) {
    const auto velocity = dynamic_planner::derivativeSpline(
        control_points, knots, dynamic_planner::kCubicDegree, 1);
    const auto acceleration = dynamic_planner::derivativeSpline(
        control_points, knots, dynamic_planner::kCubicDegree, 2);
    const Eigen::Index first_terminal_velocity =
        std::max<Eigen::Index>(0, velocity.control_points.rows() - 3);
    for (Eigen::Index row = first_terminal_velocity;
         row < velocity.control_points.rows(); ++row) {
        require((velocity.control_points.row(row).transpose().cwiseAbs().array() <=
                 config.v_max.array() + 1e-9).all(),
                label + " velocity control point exceeds the configured limit");
    }
    const Eigen::Index first_terminal_acceleration =
        std::max<Eigen::Index>(0, acceleration.control_points.rows() - 3);
    for (Eigen::Index row = first_terminal_acceleration;
         row < acceleration.control_points.rows(); ++row) {
        require((acceleration.control_points.row(row).transpose().cwiseAbs().array() <=
                 config.a_max.array() + 1e-9).all(),
                label + " acceleration control point exceeds the configured limit");
    }
}

void testStoppedTailAndPrecursorBoundsRemainUnchanged() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    ControlPoints prefix(5, 3);
    prefix <<
        0.0, 0.0, 1.0,
        0.1, 0.0, 1.0,
        0.2, 0.0, 1.0,
        0.3, 0.0, 1.0,
        0.4, 0.0, 1.0;

    const TerminalBoundary stopped;
    const auto tail = dynamic_planner::terminalTailControlPoints(
        prefix.row(4).transpose(), knots, 4, stopped);
    requireNear(tail.penultimate, prefix.row(4).transpose(), 1e-15,
                "stopped penultimate control point changed");
    requireNear(tail.endpoint, prefix.row(4).transpose(), 1e-15,
                "stopped endpoint control point changed");
    requireMatrixNear(dynamic_planner::terminalCompletion(prefix, knots, 4, stopped),
                      dynamic_planner::stoppingCompletion(prefix, 4), 0.0,
                      "stopped terminal completion changed");

    OctopusConfig config = baseConfig();
    config.a_max = Vec3::Constant(0.5);
    ControlPoints q012 = prefix.topRows(3);
    OctopusSearch search(knots, q012, Vec3(0.4, 0.0, 1.0), {}, {}, stopped, config);
    const int index = search.finalSearchIndex() - 1;
    const Vec3 qm2 = prefix.row(1).transpose();
    const Vec3 qm1 = prefix.row(2).transpose();
    const Vec3 qi = prefix.row(3).transpose();
    const auto bounds = search.velocityBounds(index, qm2, qm1, qi);
    require(bounds.has_value(), "stopped predecessor bounds unexpectedly empty");

    const Vec3 vi_m1 = static_cast<double>(dynamic_planner::kCubicDegree) * (qi - qm1) /
        (knots.at(static_cast<std::size_t>(index + dynamic_planner::kCubicDegree)) -
         knots.at(static_cast<std::size_t>(index)));
    const double d =
        (knots.at(static_cast<std::size_t>(index + 3)) -
         knots.at(static_cast<std::size_t>(index + 1))) / 2.0;
    const double c =
        (knots.at(static_cast<std::size_t>(index + 4)) -
         knots.at(static_cast<std::size_t>(index + 2))) / 2.0;
    Vec3 expected_lower = -config.v_max * config.alpha_shrink;
    Vec3 expected_upper = config.v_max * config.alpha_shrink;
    expected_lower = expected_lower.cwiseMax(vi_m1 - config.a_max * d);
    expected_upper = expected_upper.cwiseMin(vi_m1 + config.a_max * d);
    expected_lower = expected_lower.cwiseMax(-config.a_max * c);
    expected_upper = expected_upper.cwiseMin(config.a_max * c);
    requireNear(bounds->first, expected_lower, 1e-12,
                "stopped predecessor lower bound changed");
    requireNear(bounds->second, expected_upper, 1e-12,
                "stopped predecessor upper bound changed");
}

void testMovingTailHardMatchesPositionVelocityWithoutAccelerationEquality() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    ControlPoints prefix(5, 3);
    prefix <<
        0.0, 0.0, 1.0,
        0.2, 0.0, 1.0,
        0.4, 0.0, 1.0,
        0.8, 0.1, 1.0,
        1.4, 0.2, 1.1;

    TerminalBoundary moving;
    moving.mode = TerminalMode::MovingRendezvous;
    moving.position = Vec3(2.0, 0.5, 1.2);
    moving.velocity = Vec3(0.4, -0.1, 0.0);

    const auto tail = dynamic_planner::terminalTailControlPoints(
        prefix.row(4).transpose(), knots, 4, moving);
    const ControlPoints completed = dynamic_planner::terminalCompletion(prefix, knots, 4, moving);
    requireNear(completed.row(5).transpose(), tail.penultimate, 1e-15,
                "completion did not use the shared moving penultimate point");
    requireNear(completed.row(6).transpose(), tail.endpoint, 1e-15,
                "completion did not use the shared moving endpoint");

    const State terminal = dynamic_planner::evaluateCubicState(completed, knots, 4.0);
    requireNear(terminal.position, moving.position, 1e-12,
                "moving terminal position was not hard-matched");
    requireNear(terminal.velocity, moving.velocity, 1e-12,
                "moving terminal velocity was not hard-matched");
    require(terminal.acceleration.norm() > 1e-3,
            "moving terminal acceleration was accidentally constrained to zero");
}

void testMovingPrecursorBoundsRespectExactCompletedTailDynamics() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    OctopusConfig config = baseConfig();
    config.a_max = Vec3::Constant(0.5);
    const Vec3 q0(0.0, 0.0, 1.0);
    const Vec3 q1(0.3, 0.0, 1.0);
    const Vec3 q2(0.7, 0.0, 1.0);
    const Vec3 q3(1.1, 0.0, 1.0);
    ControlPoints q012(3, 3);
    q012.row(0) = q0.transpose();
    q012.row(1) = q1.transpose();
    q012.row(2) = q2.transpose();

    TerminalBoundary moving;
    moving.mode = TerminalMode::MovingRendezvous;
    moving.position = Vec3(2.0, 0.0, 1.0);
    moving.velocity = Vec3(0.4, 0.0, 0.0);
    const Vec3 goal = dynamic_planner::terminalSearchPrecursor(knots, 7, moving);
    OctopusSearch search(knots, q012, goal, {}, {}, moving, config);
    const int index = search.finalSearchIndex() - 1;
    const auto bounds = search.velocityBounds(index, q1, q2, q3);
    require(bounds.has_value(), "moving predecessor bounds unexpectedly empty");
    require(bounds->first.x() > 0.20 && bounds->second.x() > 0.50,
            "moving terminal velocity did not shift precursor bounds away from zero");

    const double u_span =
        knots.at(static_cast<std::size_t>(index + dynamic_planner::kCubicDegree + 1)) -
        knots.at(static_cast<std::size_t>(index + 1));
    const Vec3 midpoint = 0.5 * (bounds->first + bounds->second);
    const std::vector<Vec3> sampled_bounds{bounds->first, midpoint, bounds->second};
    for (const Vec3& u : sampled_bounds) {
        ControlPoints prefix(5, 3);
        prefix.row(0) = q0.transpose();
        prefix.row(1) = q1.transpose();
        prefix.row(2) = q2.transpose();
        prefix.row(3) = q3.transpose();
        prefix.row(4) = (q3 + (u_span / dynamic_planner::kCubicDegree) * u).transpose();
        requireTerminalDynamicsWithinLimits(
            dynamic_planner::terminalCompletion(prefix, knots, 4, moving), knots, config,
            "moving predecessor bound endpoint");
    }
}


void testContinuationSearchOwnsPenultimateControlPoint() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    TerminalBoundary continuation;
    continuation.mode = TerminalMode::Continuation;
    continuation.position = Vec3(1.0, 0.0, 1.0);
    continuation.velocity.setZero();  // ignored by the continuation contract

    OctopusConfig config = baseConfig();
    OctopusSearch search(
        knots, q012, continuation.position, {}, {}, continuation, config);
    require(search.finalSearchIndex() == 5,
            "continuation must search q_(N-1), not stop at q_(N-2)");

    const auto result = search.search();
    require(result.success,
            "free-velocity continuation search failed in empty scene: " + result.status);
    require(result.control_points.has_value(),
            "continuation search did not return control points");
    const ControlPoints& cps = *result.control_points;
    require(cps.rows() == 7, "continuation returned wrong control-point count");
    requireNear(cps.row(6).transpose(), continuation.position, 1e-12,
                "continuation did not hard-match the fixed endpoint q_N");
    require((cps.row(5) - cps.row(6)).norm() > 1e-6,
            "continuation accidentally collapsed q_(N-1) onto q_N (stopped tail)");

    const State terminal = dynamic_planner::evaluateCubicState(cps, knots, 4.0);
    requireNear(terminal.position, continuation.position, 1e-12,
                "continuation terminal position mismatch");
    require(terminal.velocity.norm() > 1e-6,
            "continuation terminal velocity was still forced to zero");
    requireTerminalDynamicsWithinLimits(cps, knots, config, "continuation");
}

void testContinuationPredecessorBoundsRespectFixedEndpointDynamics() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    OctopusConfig config = baseConfig();
    config.a_max = Vec3::Constant(0.75);

    ControlPoints q012(3, 3);
    q012 <<
        0.0, 0.0, 1.0,
        0.2, 0.0, 1.0,
        0.5, 0.0, 1.0;

    TerminalBoundary continuation;
    continuation.mode = TerminalMode::Continuation;
    continuation.position = Vec3(1.5, 0.0, 1.0);
    continuation.velocity.setZero();

    OctopusSearch search(
        knots, q012, continuation.position, {}, {}, continuation, config);
    require(search.finalSearchIndex() == 5,
            "continuation predecessor test expected q5 search endpoint");

    // At index 4, velocityBounds samples u=v4 and must guarantee that the
    // implied terminal derivative v5 to fixed q6 also respects v/a limits.
    const int index = search.finalSearchIndex() - 1;
    const Vec3 q2 = q012.row(2).transpose();
    const Vec3 q3(0.8, 0.0, 1.0);
    const Vec3 q4(1.1, 0.0, 1.0);
    const auto bounds = search.velocityBounds(index, q2, q3, q4);
    require(bounds.has_value(), "continuation predecessor bounds unexpectedly empty");

    const double u_span =
        knots.at(static_cast<std::size_t>(index + dynamic_planner::kCubicDegree + 1)) -
        knots.at(static_cast<std::size_t>(index + 1));
    for (const Vec3& u : std::vector<Vec3>{
             bounds->first, 0.5 * (bounds->first + bounds->second), bounds->second}) {
        ControlPoints cps(7, 3);
        cps.row(0) = q012.row(0);
        cps.row(1) = q012.row(1);
        cps.row(2) = q012.row(2);
        cps.row(3) = q3.transpose();
        cps.row(4) = q4.transpose();
        cps.row(5) = (q4 + (u_span / dynamic_planner::kCubicDegree) * u).transpose();
        cps.row(6) = continuation.position.transpose();
        requireTerminalDynamicsWithinLimits(
            cps, knots, config, "continuation predecessor bound endpoint");
    }
}


void testContinuationPartialStillStops() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    TerminalBoundary continuation;
    continuation.mode = TerminalMode::Continuation;
    continuation.position = Vec3(2.0, 0.0, 1.0);
    continuation.velocity.setZero();

    OctopusConfig config = baseConfig();
    const auto partial = OctopusSearch(
        knots, q012, continuation.position, {}, {}, continuation, config,
        []() { return true; }, "USEFUL_DEADLINE_REACHED").search();

    require(partial.success,
            "continuation padded partial should retain a stopped feasible fallback: " +
                partial.status);
    require(partial.status == "PADDED_CLOSEST_PARTIAL",
            "continuation external stop returned wrong fallback status: " + partial.status);
    require(partial.control_points.has_value(),
            "continuation padded partial did not expose control points");
    const State terminal = dynamic_planner::evaluateCubicState(
        *partial.control_points, knots, 4.0);
    require(terminal.velocity.norm() < 1e-10,
            "continuation padded partial did not terminate stopped");
    require((terminal.position - continuation.position).norm() > 0.5,
            "continuation padded partial was incorrectly forced to the nominal waypoint");
}

void testMovingRequestPartialStopsAtLocalEvasiveEndpoint() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    TerminalBoundary moving;
    moving.mode = TerminalMode::MovingRendezvous;
    moving.position = Vec3(2.0, 0.5, 1.0);
    moving.velocity = Vec3(0.3, 0.1, 0.0);
    const Vec3 goal = dynamic_planner::terminalSearchPrecursor(knots, 7, moving);

    OctopusConfig config = baseConfig();
    const auto partial = OctopusSearch(
        knots, q012, goal, {}, {}, moving, config,
        []() { return true; }, "USEFUL_DEADLINE_REACHED").search();

    require(partial.success, "stationary evasive partial should be dynamically feasible: " + partial.status);
    require(partial.status == "PADDED_CLOSEST_PARTIAL",
            "moving request did not expose a true padded partial: " + partial.status);
    require(partial.control_points.has_value(), "accepted partial did not expose control points");
    const State terminal = dynamic_planner::evaluateCubicState(
        *partial.control_points, knots, 4.0);
    require(terminal.velocity.norm() < 1e-10,
            "moving-request partial inherited moving-target terminal velocity");
    require((terminal.position - moving.position).norm() > 0.5,
            "moving-request partial was still forced to the distant rendezvous endpoint");
}

void testMovingTailObstacleIsRejectedBeforeCompleteSelection() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    TerminalBoundary moving;
    moving.mode = TerminalMode::MovingRendezvous;
    moving.position = Vec3(2.0, 0.0, 1.0);
    moving.velocity = Vec3(0.4, 0.0, 0.0);
    const Vec3 goal = dynamic_planner::terminalSearchPrecursor(knots, 7, moving);

    OctopusConfig config = baseConfig();
    // Search q_(N-2) near the precursor, while the fixed endpoint remains
    // outside the search box.  A stopped-tail approximation cannot touch the
    // final-interval obstacle; the actual moving completion must reject it.
    config.xyz_max.x() = 1.70;
    const auto clear = OctopusSearch(knots, q012, goal, {}, {}, moving, config).search();
    require(clear.complete_available_at_termination,
            "clear moving-terminal fixture did not produce a complete candidate");

    TimeIndexedObstacle endpoint_obstacle;
    endpoint_obstacle.name = "moving_terminal_endpoint";
    const Eigen::MatrixXd far = box(Vec3(10.0, 10.0, 10.0), 0.01);
    endpoint_obstacle.interval_vertices = {
        far,
        far,
        far,
        box(moving.position, 0.01),
    };
    const auto blocked = OctopusSearch(
        knots, q012, goal, {endpoint_obstacle}, {}, moving, config).search();

    // C1F.7 deliberately allows a safe stopped local partial when the moving
    // rendezvous endpoint itself is blocked. The invariant carried forward
    // from C1F.6a is narrower: no COMPLETE moving-rendezvous candidate may
    // survive an obstacle on its exact terminal tail.
    require(blocked.success,
            "blocked moving endpoint did not fall back to a safe local partial");
    require(blocked.status == "PADDED_CLOSEST_PARTIAL",
            "blocked moving endpoint selected something other than an evasive partial: " +
                blocked.status);
    require(blocked.control_points.has_value(),
            "blocked moving endpoint partial did not expose control points");
    const State blocked_terminal = dynamic_planner::evaluateCubicState(
        *blocked.control_points, knots, 4.0);
    require(blocked_terminal.velocity.norm() < 1e-10,
            "blocked moving-endpoint partial did not terminate stopped");
    require((blocked_terminal.position - moving.position).norm() > 0.05,
            "blocked moving-endpoint partial still reached the forbidden rendezvous endpoint");
    require(blocked.closest_complete_improvements == 0,
            "moving-tail collision was discovered only after complete-node selection");
    require(!blocked.complete_available_at_termination,
            "moving-tail collision left a search-level complete candidate available");
}

}  // namespace

int main() {
    try {
        testStoppedTailAndPrecursorBoundsRemainUnchanged();
        testMovingTailHardMatchesPositionVelocityWithoutAccelerationEquality();
        testMovingPrecursorBoundsRespectExactCompletedTailDynamics();
        testContinuationSearchOwnsPenultimateControlPoint();
        testContinuationPredecessorBoundsRespectFixedEndpointDynamics();
        testContinuationPartialStillStops();
        testMovingRequestPartialStopsAtLocalEvasiveEndpoint();
        testMovingTailObstacleIsRejectedBeforeCompleteSelection();
        std::cout << "test_octopus_terminal_contract: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_octopus_terminal_contract: FAIL: " << error.what() << '\n';
        return 1;
    }
}
