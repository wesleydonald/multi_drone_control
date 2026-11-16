#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/reference_window.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::ControlPoints;
using dynamic_planner::State;
using dynamic_planner::TerminalBoundary;
using dynamic_planner::TerminalMode;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;

void requireTrue(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireVecNear(const Vec3& actual,
                    const Vec3& expected,
                    double tolerance,
                    const std::string& message) {
    if ((actual - expected).cwiseAbs().maxCoeff() > tolerance) {
        throw std::runtime_error(message + " actual=" +
                                 std::to_string(actual.x()) + "," +
                                 std::to_string(actual.y()) + "," +
                                 std::to_string(actual.z()));
    }
}

}  // namespace

int main() {
    try {
        // C1F.6 contract: a moving rendezvous hard-matches terminal position
        // and velocity but does not hard-match terminal acceleration.
        const auto knots = dynamic_planner::openUniformKnots(10.0, 12.0, 4);
        ControlPoints prefix(5, 3);
        prefix <<
            0.0, 0.0, 1.5,
            0.1, 0.0, 1.5,
            0.2, 0.0, 1.5,
            0.5, 0.1, 1.5,
            0.78, 0.22, 1.52;

        TerminalBoundary boundary;
        boundary.mode = TerminalMode::MovingRendezvous;
        boundary.position = Vec3(1.0, 0.25, 1.5);
        boundary.velocity = Vec3(0.125, -0.02, 0.0);

        TrajectoryPiece nominal;
        nominal.knots = knots;
        nominal.control_points = dynamic_planner::terminalCompletion(
            prefix, knots, 4, boundary);
        nominal.valid_from = 10.0;
        nominal.valid_until = 12.0;

        const State rendezvous = nominal.evaluate(nominal.valid_until);
        requireVecNear(rendezvous.position, boundary.position, 1e-12,
                       "moving rendezvous terminal position mismatch");
        requireVecNear(rendezvous.velocity, boundary.velocity, 1e-12,
                       "moving rendezvous terminal velocity mismatch");
        requireTrue(rendezvous.acceleration.norm() > 1e-4,
                    "moving rendezvous terminal acceleration was accidentally hard-zeroed");

        // A moving nominal endpoint is not allowed to masquerade as an
        // indefinitely executable commitment.  Append the explicit
        // velocity-continuous brake and recover the ordinary stopped-hold
        // safety contract.
        CommittedTrajectory committed;
        requireTrue(committed.replaceSuffix(nominal.valid_from, nominal).accepted,
                    "nominal moving rendezvous commit rejected");
        requireTrue(!committed.endsInStoppedHold(),
                    "moving nominal endpoint incorrectly marked as stopped");

        const CommittedTrajectory brake = dynamic_planner::makeBrakingHoverTrajectory(
            rendezvous,
            nominal.valid_until,
            Vec3(1.0, 1.0, 1.5),
            0.25);
        requireTrue(brake.pieceCount() == 1,
                    "moving rendezvous brake should contain one explicit piece");
        const auto appended = committed.appendVelocityContinuousBackup(
            brake.pieces().front(), 1e-9);
        requireTrue(appended.accepted,
                    "moving rendezvous stopped backup was not velocity-continuous");
        requireTrue(appended.position_error < 1e-10,
                    "moving rendezvous backup position discontinuity");
        requireTrue(appended.velocity_error < 1e-10,
                    "moving rendezvous backup velocity discontinuity");
        requireTrue(committed.endsInStoppedHold(),
                    "moving rendezvous + backup must end in stopped hold");

        const State after_end = committed.evaluate(committed.endTime() + 0.5);
        requireVecNear(after_end.velocity, Vec3::Zero(), 1e-12,
                       "post-backup hold was not stationary");
        requireVecNear(after_end.acceleration, Vec3::Zero(), 1e-12,
                       "post-backup hold acceleration was not zero");

        std::cout << "test_c1f6_moving_rendezvous: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_c1f6_moving_rendezvous: FAIL: " << error.what() << '\n';
        return 1;
    }
}
