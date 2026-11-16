#include "dynamic_planner/bspline.hpp"
#include "test_common.hpp"

#include <Eigen/Core>
#include <iostream>
#include <vector>

using dynamic_planner::ControlPoints;
using dynamic_planner::State;
using dynamic_planner::Vec3;

int main() {
    try {
        {
            const auto knots = dynamic_planner::openUniformKnots(0.0, 8.0, 4);
            const std::vector<double> expected{0, 0, 0, 0, 2, 4, 6, 8, 8, 8, 8};
            requireTrue(knots.size() == expected.size(), "four-segment knot count");
            for (std::size_t i = 0; i < knots.size(); ++i) {
                requireNear(knots[i], expected[i], 1e-14, "four-segment knot value");
            }
        }

        const auto knots = dynamic_planner::openUniformKnots(0.0, 6.0, 4);
        const State initial{
            Vec3(0.2, -0.1, 1.3),
            Vec3(0.3, -0.2, 0.1),
            Vec3(0.15, 0.05, -0.1),
        };
        const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(initial, knots);
        ControlPoints expected_q012(3, 3);
        expected_q012 <<
            0.2, -0.1, 1.3,
            0.35, -0.2, 1.35,
            0.7625, -0.3625, 1.375;
        requireMatrixNear(q012, expected_q012, 1e-13,
                          "initial q0/q1/q2 must match Python R6.2 reference");

        ControlPoints cps(7, 3);
        cps <<
            0.2, -0.1, 1.3,
            0.35, -0.2, 1.35,
            0.7625, -0.3625, 1.375,
            1.2, 0.1, 1.5,
            1.7, 0.8, 1.45,
            2.0, 1.2, 1.4,
            2.0, 1.2, 1.4;

        const State at_start = dynamic_planner::evaluateCubicState(cps, knots, 0.0);
        requireMatrixNear(at_start.position, initial.position, 1e-12, "start position parity");
        requireMatrixNear(at_start.velocity, initial.velocity, 1e-12, "start velocity parity");
        requireMatrixNear(at_start.acceleration, initial.acceleration, 1e-12, "start acceleration parity");

        const State mid = dynamic_planner::evaluateCubicState(cps, knots, 3.0);
        requireMatrixNear(mid.position, Vec3(1.2104166666666667, 0.13958333333333334, 1.4708333333333334),
                          2e-12, "midpoint position parity");
        requireMatrixNear(mid.velocity, Vec3(0.3125, 0.3875, 0.025),
                          2e-12, "midpoint velocity parity");
        requireMatrixNear(mid.acceleration, Vec3(0.02777777777777779, 0.10555555555555554, -0.07777777777777778),
                          2e-12, "midpoint acceleration parity");

        const auto velocity = dynamic_planner::derivativeSpline(cps, knots, 3, 1);
        ControlPoints expected_velocity(6, 3);
        expected_velocity <<
            0.3, -0.2, 0.1,
            0.4125, -0.1625, 0.025,
            0.2916666666666667, 0.30833333333333335, 0.08333333333333333,
            0.3333333333333333, 0.4666666666666667, -0.03333333333333333,
            0.3, 0.4, -0.05,
            0.0, 0.0, 0.0;
        requireMatrixNear(velocity.control_points, expected_velocity, 2e-13,
                          "velocity derivative control points parity");

        const auto acceleration = dynamic_planner::derivativeSpline(cps, knots, 3, 2);
        ControlPoints expected_acceleration(5, 3);
        expected_acceleration <<
            0.15, 0.05, -0.1,
            -0.08055555555555556, 0.3138888888888889, 0.03888888888888889,
            0.02777777777777779, 0.10555555555555554, -0.07777777777777778,
            -0.02222222222222222, -0.04444444444444444, -0.01111111111111111,
            -0.4, -0.5333333333333333, 0.06666666666666667;
        requireMatrixNear(acceleration.control_points, expected_acceleration, 2e-13,
                          "acceleration derivative control points parity");

        ControlPoints prefix(5, 3);
        prefix <<
            0.0, 0.0, 1.5,
            0.0, 0.0, 1.5,
            0.0, 0.0, 1.5,
            0.8, 0.4, 1.6,
            1.5, 1.2, 1.5;
        const auto stop_knots = dynamic_planner::openUniformKnots(0.0, 5.0, 4);
        const auto stopped = dynamic_planner::stoppingCompletion(prefix, 4);
        const State terminal = dynamic_planner::evaluateCubicState(stopped, stop_knots, 5.0);
        requireMatrixNear(terminal.velocity, Vec3::Zero(), 1e-12,
                          "stopping completion terminal velocity");
        requireMatrixNear(terminal.acceleration, Vec3::Zero(), 1e-12,
                          "stopping completion terminal acceleration");

        // C1F.6 moving-rendezvous terminal contract: hard endpoint p/v without
        // hard terminal-acceleration matching.
        dynamic_planner::TerminalBoundary rendezvous;
        rendezvous.mode = dynamic_planner::TerminalMode::MovingRendezvous;
        rendezvous.position = Vec3(2.0, 1.0, 1.6);
        rendezvous.velocity = Vec3(0.125, -0.05, 0.0);
        ControlPoints rendezvous_prefix = prefix;
        // Deliberately choose q_(N-2) away from the zero-acceleration precursor.
        rendezvous_prefix.row(4) = Eigen::RowVector3d(1.70, 0.95, 1.55);
        const auto moving = dynamic_planner::terminalCompletion(
            rendezvous_prefix, stop_knots, 4, rendezvous);
        const State moving_terminal = dynamic_planner::evaluateCubicState(
            moving, stop_knots, 5.0);
        requireMatrixNear(moving_terminal.position, rendezvous.position, 1e-12,
                          "moving terminal position hard match");
        requireMatrixNear(moving_terminal.velocity, rendezvous.velocity, 1e-12,
                          "moving terminal velocity hard match");
        requireTrue(moving_terminal.acceleration.norm() > 1e-3,
                    "moving terminal acceleration must remain unconstrained");

        const Vec3 precursor = dynamic_planner::terminalSearchPrecursor(
            stop_knots, 7, rendezvous);
        requireMatrixNear(precursor,
                          rendezvous.position - rendezvous.velocity * 1.25,
                          1e-12, "moving terminal search precursor");
        rendezvous_prefix.row(4) = precursor.transpose();
        const auto moving_zero_accel_heuristic = dynamic_planner::terminalCompletion(
            rendezvous_prefix, stop_knots, 4, rendezvous);
        const State heuristic_terminal = dynamic_planner::evaluateCubicState(
            moving_zero_accel_heuristic, stop_knots, 5.0);
        requireMatrixNear(heuristic_terminal.acceleration, Vec3::Zero(), 1e-12,
                          "zero-acceleration precursor heuristic parity");

        dynamic_planner::TerminalBoundary continuation;
        continuation.mode = dynamic_planner::TerminalMode::Continuation;
        continuation.position = Vec3(2.0, 1.0, 1.6);
        bool continuation_tail_rejected = false;
        try {
            (void)dynamic_planner::terminalCompletion(
                rendezvous_prefix, stop_knots, 4, continuation);
        } catch (const std::invalid_argument&) {
            continuation_tail_rejected = true;
        }
        requireTrue(continuation_tail_rejected,
                    "continuation must not silently collapse to a deterministic stopped tail");

        std::cout << "test_bspline: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_bspline: FAIL: " << error.what() << '\n';
        return 1;
    }
}
