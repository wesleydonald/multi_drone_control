#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/reference_window.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <iostream>

using namespace dynamic_planner;

namespace {

TrajectoryPiece makeFirstPiece() {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(0.0, 2.0, 4);
    piece.control_points.resize(7, 3);
    piece.control_points <<
        0.0, 0.0, 1.5,
        0.0, 0.0, 1.5,
        0.0, 0.0, 1.5,
        0.35, 0.05, 1.5,
        0.70, 0.20, 1.5,
        0.70, 0.20, 1.5,
        0.70, 0.20, 1.5;
    piece.valid_from = 0.0;
    piece.valid_until = 2.0;
    return piece;
}

TrajectoryPiece makeContinuousCandidate(const State& A, double splice_time) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(splice_time, splice_time + 2.0, 4);
    const ControlPoints q012 = initialControlPointsFromState(A, piece.knots);
    ControlPoints prefix(5, 3);
    prefix.topRows(3) = q012;
    prefix.row(3) = q012.row(2) + Eigen::RowVector3d(0.18, 0.08, 0.02);
    prefix.row(4) = q012.row(2) + Eigen::RowVector3d(0.30, 0.15, 0.00);
    piece.control_points = stoppingCompletion(prefix, 4);
    piece.valid_from = splice_time;
    piece.valid_until = splice_time + 2.0;
    return piece;
}

}  // namespace

int main() {
    try {
        CommittedTrajectory committed;
        const TrajectoryPiece first = makeFirstPiece();
        auto first_diag = committed.replaceSuffix(0.0, first);
        requireTrue(first_diag.accepted, "initial commit rejected");

        const double splice = 1.0;
        std::vector<State> before;
        for (double t = 0.0; t < splice - 1e-12; t += 0.05) {
            before.push_back(committed.evaluate(t));
        }
        const State A = committed.evaluate(splice);
        const TrajectoryPiece candidate = makeContinuousCandidate(A, splice);
        const auto diag = committed.replaceSuffix(splice, candidate, 1e-9);
        requireTrue(diag.accepted, "continuous candidate rejected");
        requireTrue(diag.position_error < 1e-10, "position splice error too large");
        requireTrue(diag.velocity_error < 1e-10, "velocity splice error too large");
        requireTrue(diag.acceleration_error < 1e-10, "acceleration splice error too large");

        std::size_t k = 0;
        for (double t = 0.0; t < splice - 1e-12; t += 0.05, ++k) {
            const State after = committed.evaluate(t);
            requireTrue((after.position - before[k].position).norm() < 1e-12,
                    "committed prefix position changed after splice");
            requireTrue((after.velocity - before[k].velocity).norm() < 1e-12,
                    "committed prefix velocity changed after splice");
            requireTrue((after.acceleration - before[k].acceleration).norm() < 1e-12,
                    "committed prefix acceleration changed after splice");
        }

        // Deliberately discontinuous candidate must be atomically rejected.
        const auto pieces_before_reject = committed.pieces();
        TrajectoryPiece bad = makeContinuousCandidate(committed.evaluate(1.4), 1.4);
        bad.control_points.row(0).x() += 0.01;
        const auto rejected = committed.replaceSuffix(1.4, bad, 1e-9);
        requireTrue(!rejected.accepted, "discontinuous splice should be rejected");
        requireTrue(committed.pieceCount() == pieces_before_reject.size(),
                "rejected splice mutated piece count");
        for (std::size_t i = 0; i < pieces_before_reject.size(); ++i) {
            requireTrue((committed.pieces()[i].control_points - pieces_before_reject[i].control_points)
                        .cwiseAbs().maxCoeff() < 1e-15,
                    "rejected splice mutated control points");
            requireNear(committed.pieces()[i].valid_until,
                        pieces_before_reject[i].valid_until, 1e-15,
                        "rejected splice mutated validity");
        }

        // B.1 v2 regression: a stopped explicit endpoint extends as an
        // exact stationary hold, and a candidate can splice in the future of
        // that hold without weakening C2 continuity.
        CommittedTrajectory stopped;
        const TrajectoryPiece stopped_piece = makeFirstPiece();
        requireTrue(stopped.replaceSuffix(0.0, stopped_piece).accepted,
                    "stopped-hold bootstrap commit failed");
        requireTrue(stopped.endsInStoppedHold(),
                    "stopping-completed piece was not recognized as a terminal hold");
        const double explicit_end = stopped.endTime();
        const State explicit_end_state = stopped.endState();
        const double future_hold_time = explicit_end + 0.25;
        const State held = stopped.evaluate(future_hold_time);
        requireMatrixNear(held.position, explicit_end_state.position, 1e-12,
                          "terminal hold changed endpoint position");
        requireMatrixNear(held.velocity, Vec3::Zero(), 1e-12,
                          "terminal hold velocity is not exactly zero");
        requireMatrixNear(held.acceleration, Vec3::Zero(), 1e-12,
                          "terminal hold acceleration is not exactly zero");

        const TrajectoryPiece restart = makeContinuousCandidate(held, future_hold_time);
        const auto restart_diag = stopped.replaceSuffix(future_hold_time, restart, 1e-9);
        requireTrue(restart_diag.accepted,
                    "candidate could not restart from stationary terminal hold");
        requireTrue(restart_diag.position_error < 1e-10,
                    "terminal-hold restart position continuity failed");
        requireTrue(restart_diag.velocity_error < 1e-10,
                    "terminal-hold restart velocity continuity failed");
        requireTrue(restart_diag.acceleration_error < 1e-10,
                    "terminal-hold restart acceleration continuity failed");
        requireTrue(stopped.pieceCount() == 3,
                    "terminal-hold restart should materialize one finite hold piece");
        const State mid_hold = stopped.evaluate(explicit_end + 0.125);
        requireMatrixNear(mid_hold.position, explicit_end_state.position, 1e-12,
                          "materialized hold changed endpoint position");
        requireMatrixNear(mid_hold.velocity, Vec3::Zero(), 1e-12,
                          "materialized hold velocity is not zero");

        // C1F.6: a nominal moving-rendezvous piece is followed by an explicit
        // velocity-continuous brake-to-hover backup. The committed trajectory
        // still ends in the ordinary stopped-hold contract even though the
        // nominal rendezvous endpoint itself has nonzero world-frame velocity.
        const auto moving_knots = openUniformKnots(10.0, 12.0, 4);
        ControlPoints moving_prefix(5, 3);
        moving_prefix <<
            0.0, 0.0, 1.5,
            0.1, 0.0, 1.5,
            0.2, 0.0, 1.5,
            0.5, 0.1, 1.5,
            0.8, 0.2, 1.5;
        TerminalBoundary moving_boundary;
        moving_boundary.mode = TerminalMode::MovingRendezvous;
        moving_boundary.position = Vec3(1.0, 0.25, 1.5);
        moving_boundary.velocity = Vec3(0.125, 0.0, 0.0);
        TrajectoryPiece moving_piece;
        moving_piece.knots = moving_knots;
        moving_piece.control_points = terminalCompletion(
            moving_prefix, moving_knots, 4, moving_boundary);
        moving_piece.valid_from = 10.0;
        moving_piece.valid_until = 12.0;
        const State moving_end = moving_piece.evaluate(12.0);
        requireMatrixNear(moving_end.position, moving_boundary.position, 1e-12,
                          "moving candidate terminal position");
        requireMatrixNear(moving_end.velocity, moving_boundary.velocity, 1e-12,
                          "moving candidate terminal velocity");

        CommittedTrajectory moving_commit;
        requireTrue(moving_commit.replaceSuffix(10.0, moving_piece).accepted,
                    "moving rendezvous seed commit failed");
        requireTrue(!moving_commit.endsInStoppedHold(),
                    "moving rendezvous candidate should not masquerade as stopped");
        const CommittedTrajectory brake = makeBrakingHoverTrajectory(
            moving_end, 12.0, Vec3(1.0, 1.0, 1.5), 0.25);
        requireTrue(brake.pieceCount() == 1, "rendezvous brake expected one piece");
        const auto append = moving_commit.appendVelocityContinuousBackup(
            brake.pieces().front(), 1e-9);
        requireTrue(append.accepted, "velocity-continuous rendezvous backup rejected");
        requireTrue(append.position_error < 1e-10, "rendezvous backup position discontinuity");
        requireTrue(append.velocity_error < 1e-10, "rendezvous backup velocity discontinuity");
        requireTrue(moving_commit.endsInStoppedHold(),
                    "rendezvous candidate + brake must end in stopped hold");
        requireMatrixNear(moving_commit.evaluate(12.0).position, moving_end.position, 1e-12,
                          "backup append changed rendezvous boundary position");
        requireMatrixNear(moving_commit.evaluate(12.0).velocity, moving_end.velocity, 1e-12,
                          "backup append changed rendezvous boundary velocity");
        const State after_backup = moving_commit.evaluate(moving_commit.endTime() + 0.5);
        requireMatrixNear(after_backup.velocity, Vec3::Zero(), 1e-12,
                          "rendezvous backup did not produce stationary terminal hold");

        std::cout << "test_committed_trajectory: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_committed_trajectory: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
