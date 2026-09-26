#include "dynamic_planner/target_predictor.hpp"
#include "test_common.hpp"

#include <iostream>

using namespace dynamic_planner;

int main() {
    try {
        constexpr double kPi = 3.14159265358979323846;
        ScriptedCircleTargetPredictor scripted(
            Vec3(2.0, 2.0, 1.5), 0.5, 0.25, Vec3(2.5, 2.0, 1.5), 10.0);
        const TargetPrediction quarter = scripted.evaluate(10.0 + 0.5 * kPi / 0.25);
        requireTrue(quarter.valid, "scripted prediction is unexpectedly invalid");
        requireMatrixNear(quarter.position, Vec3(2.0, 2.5, 1.5), 1e-10,
                          "scripted future position is wrong");
        requireMatrixNear(quarter.velocity, Vec3(-0.125, 0.0, 0.0), 1e-10,
                          "scripted future velocity is wrong");
        requireTrue(scripted.highConfidenceFuture(),
                    "scripted provider must identify its known future as high confidence");

        ConstantVelocityTargetPredictor cv(Vec3(1.0, 2.0, 3.0), Vec3(0.2, -0.1, 0.0), 4.0);
        const TargetPrediction cv_future = cv.evaluate(7.0);
        requireTrue(cv_future.valid, "constant-velocity prediction is unexpectedly invalid");
        requireMatrixNear(cv_future.position, Vec3(1.6, 1.7, 3.0), 1e-12,
                          "constant-velocity future position is wrong");
        requireTrue(!cv.highConfidenceFuture(), "CV provider must retain its low-confidence designation");

        const Vec3 committed_centre(0.8, -0.4, 1.5);
        const Vec3 committed_velocity(0.2, 0.1, 0.0);
        const Vec3 committed_offset(0.0, 0.0, 1.0);
        TrajectoryPiece advertised_piece;
        advertised_piece.valid_from = 12.0;
        advertised_piece.valid_until = 13.0;
        advertised_piece.knots = openUniformKnots(12.0, 13.0, 1);
        advertised_piece.control_points.resize(4, 3);
        for (int i = 0; i < 4; ++i) {
            advertised_piece.control_points.row(i) =
                (committed_centre + committed_velocity * (static_cast<double>(i) / 3.0)).transpose();
        }
        CommittedTrajectory committed;
        requireTrue(
            committed.replaceSuffix(12.0, advertised_piece, 1e-12).accepted,
            "failed to build advertised moving target commitment");
        CommittedTrajectoryTargetPredictor committed_predictor(
            committed, committed_offset);
        const TargetPrediction committed_future = committed_predictor.evaluate(12.4);
        requireTrue(committed_future.valid,
                    "committed-trajectory prediction is unexpectedly invalid");
        requireMatrixNear(
            committed_future.position,
            committed_centre + committed_velocity * 0.4 + committed_offset, 1e-12,
            "committed-trajectory predictor did not preserve advertised future + offset");
        requireMatrixNear(committed_future.velocity, committed_velocity, 1e-12,
                          "committed-trajectory predictor changed advertised velocity");
        requireTrue(committed_predictor.highConfidenceFuture(),
                    "advertised commitment must be treated as a high-confidence future");
        requireTrue(std::string(committed_predictor.type()) == "committed_trajectory",
                    "committed-trajectory predictor type changed unexpectedly");
        const TargetPrediction beyond_moving_commit = committed_predictor.evaluate(13.2);
        requireTrue(!beyond_moving_commit.valid,
                    "non-stopped moving commitment extrapolated beyond its explicit future");

        const auto stopped_commit = makeStationaryHoverTrajectory(
            committed_centre, 14.0, 0.5);
        CommittedTrajectoryTargetPredictor stopped_predictor(
            stopped_commit, committed_offset);
        const TargetPrediction beyond_stopped_commit = stopped_predictor.evaluate(15.0);
        requireTrue(beyond_stopped_commit.valid,
                    "stopped committed target did not preserve the shared terminal-hold contract");
        requireMatrixNear(beyond_stopped_commit.position, committed_centre + committed_offset, 1e-12,
                          "stopped committed target moved after its explicit spline end");
        requireMatrixNear(beyond_stopped_commit.velocity, Vec3::Zero(), 1e-12,
                          "stopped committed target retained motion after its explicit spline end");
        std::cout << "test_target_predictor: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_target_predictor: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
