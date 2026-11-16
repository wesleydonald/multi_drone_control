#include "dynamic_planner/reference_window.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "test_common.hpp"

namespace {

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::ControlPoints;
using dynamic_planner::ReferenceWindowConfig;
using dynamic_planner::State;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;

TrajectoryPiece oneSegmentPiece(
    double t0,
    double t1,
    const Vec3& q0,
    const Vec3& q1,
    const Vec3& q2,
    const Vec3& q3) {
    TrajectoryPiece piece;
    piece.valid_from = t0;
    piece.valid_until = t1;
    piece.knots = dynamic_planner::openUniformKnots(t0, t1, 1);
    piece.control_points = ControlPoints::Zero(4, 3);
    piece.control_points.row(0) = q0.transpose();
    piece.control_points.row(1) = q1.transpose();
    piece.control_points.row(2) = q2.transpose();
    piece.control_points.row(3) = q3.transpose();
    return piece;
}

void requireStateNear(const State& actual, const State& expected, double tolerance) {
    requireMatrixNear(actual.position, expected.position, tolerance, "position mismatch");
    requireMatrixNear(actual.velocity, expected.velocity, tolerance, "velocity mismatch");
    requireMatrixNear(actual.acceleration, expected.acceleration, tolerance, "acceleration mismatch");
}

}  // namespace

int main() {
    try {
        const ReferenceWindowConfig config;
        requireTrue(config.requiredSampleCount() == 61U, "MPC contract must require 61 samples");
        requireTrue(std::abs(config.samplePeriodS() - 1.0 / 30.0) < 1e-15,
                    "reference sample period must be 1/30 s");
        requireTrue(std::abs(config.horizonDurationS() - 2.0) < 1e-15,
                    "reference window must span 2.0 s");

        // Direct-evaluation contract: a rolling window is exactly the committed
        // B-spline evaluated at now + k/30. There is no second smoother.
        const double start = 5.0;
        const auto moving_piece = oneSegmentPiece(
            start,
            start + 3.0,
            Vec3(0.0, 0.0, 1.5),
            Vec3(0.6, 0.2, 1.6),
            Vec3(1.4, -0.1, 1.4),
            Vec3(2.0, 0.3, 1.5));
        CommittedTrajectory moving;
        requireTrue(moving.replaceSuffix(start, moving_piece).accepted,
                    "failed to initialise moving committed trajectory");

        const double now = 5.4;
        const auto window = dynamic_planner::sampleCommittedReferenceWindow(moving, now, config);
        requireTrue(window.samples.size() == 61U, "rolling window must contain exactly 61 samples");
        requireTrue(std::abs(window.samples.front().absolute_time_s - now) < 1e-15,
                    "sample zero must mean now");
        requireTrue(std::abs(window.samples.front().time_from_start_s) < 1e-15,
                    "sample zero time_from_start must be zero");
        requireTrue(std::abs(window.samples.back().time_from_start_s - 2.0) < 1e-12,
                    "terminal sample must be 2.0 s ahead");

        for (std::size_t k = 0; k < window.samples.size(); ++k) {
            const double expected_t = now + static_cast<double>(k) / 30.0;
            requireTrue(std::abs(window.samples[k].absolute_time_s - expected_t) < 1e-13,
                        "sample absolute time does not match 30 Hz contract");
            requireStateNear(window.samples[k].state, moving.evaluate(expected_t), 1e-13);
        }

        // Terminal stopped-hold contract: the 2 s MPC window remains valid even
        // when it extends beyond the finite seed piece.
        const Vec3 hover_position(1.2, -0.4, 1.7);
        const auto hover = dynamic_planner::makeStationaryHoverTrajectory(
            hover_position, 10.0, 0.10);
        requireTrue(hover.endsInStoppedHold(), "stationary hover must end in a certified stop");
        const auto hover_window = dynamic_planner::sampleCommittedReferenceWindow(
            hover, 10.05, config);
        requireTrue(hover_window.samples.size() == 61U, "hover window must contain 61 samples");
        for (const auto& sample : hover_window.samples) {
            requireMatrixNear(sample.state.position, hover_position, 1e-13, "hover position drifted");
            requireMatrixNear(sample.state.velocity, Vec3::Zero(), 1e-13, "hover velocity is nonzero");
            requireMatrixNear(sample.state.acceleration, Vec3::Zero(), 1e-13, "hover acceleration is nonzero");
        }

        // C1F.4 execution fallback: a measured moving state is converted to a
        // continuous cubic brake whose endpoint is an exact stopped hold and
        // whose initial axis acceleration respects the configured limit.
        {
            State measured;
            measured.position = Vec3(0.0, 0.0, 1.5);
            measured.velocity = Vec3(0.6, -0.3, 0.0);
            const Vec3 brake_limit(0.75, 0.75, 1.0);
            const auto braking = dynamic_planner::makeBrakingHoverTrajectory(
                measured, 15.0, brake_limit, 0.25);
            const State initial = braking.evaluate(15.0);
            requireMatrixNear(initial.position, measured.position, 1e-12,
                              "braking fallback changed initial position");
            requireMatrixNear(initial.velocity, measured.velocity, 1e-12,
                              "braking fallback changed initial velocity");
            requireTrue((initial.acceleration.cwiseAbs().array() <=
                         brake_limit.array() + 1e-12).all(),
                        "braking fallback exceeded configured acceleration limit");
            requireTrue(braking.endsInStoppedHold(),
                        "braking fallback does not end in stopped hold");
            const State stopped = braking.evaluate(braking.endTime() + 0.5);
            requireMatrixNear(stopped.velocity, Vec3::Zero(), 1e-12,
                              "braking fallback terminal velocity is nonzero");
            requireMatrixNear(stopped.acceleration, Vec3::Zero(), 1e-12,
                              "braking fallback terminal acceleration is nonzero");
        }

        // Artificial first-plan delay: an active system starts with a stationary
        // incumbent, so a 150 ms solve cannot make the first moving suffix start
        // in the past. It splices at a deliberately future epoch instead.
        auto delayed = dynamic_planner::makeStationaryHoverTrajectory(
            Vec3(0.0, 0.0, 1.5), 20.0, 0.10);
        const double solve_finish = 20.15;   // 150 ms simulated computation delay
        const double splice_time = 20.25;   // still 100 ms in the future at completion
        const auto moving_suffix = oneSegmentPiece(
            splice_time,
            splice_time + 1.0,
            Vec3(0.0, 0.0, 1.5),
            Vec3(0.0, 0.0, 1.5),
            Vec3(0.0, 0.0, 1.5),
            Vec3(0.8, 0.0, 1.5));
        const auto splice = delayed.replaceSuffix(splice_time, moving_suffix, 1e-10);
        requireTrue(splice.accepted, "future moving suffix failed to splice into hover incumbent");
        requireTrue(splice_time > solve_finish, "test setup must keep splice after solve completion");
        requireStateNear(delayed.evaluate(solve_finish),
                         State{Vec3(0.0, 0.0, 1.5), Vec3::Zero(), Vec3::Zero()},
                         1e-12);
        requireStateNear(delayed.evaluate(splice_time),
                         State{Vec3(0.0, 0.0, 1.5), Vec3::Zero(), Vec3::Zero()},
                         1e-12);
        requireTrue(delayed.evaluate(splice_time + 0.50).position.x() > 0.0,
                    "moving suffix did not begin after the future splice");

        // C1F.2b regression: absolute ROS epochs around 1.788e9 s have a
        // representable double spacing of O(1e-7 s). Sampling a stopped
        // committed endpoint within the explicit 1 us trajectory-time tolerance
        // must not create a false "no committed piece" gap.
        const double large_epoch = 1788347600.0;
        const Vec3 epoch_hover_position(0.4, 0.2, 2.3);
        const auto epoch_hover = dynamic_planner::makeStationaryHoverTrajectory(
            epoch_hover_position, large_epoch, 0.10);
        const double just_after_explicit_end =
            epoch_hover.endTime() + 0.5 * dynamic_planner::kTrajectoryTimeToleranceS;
        const State epoch_held = epoch_hover.evaluate(just_after_explicit_end);
        requireMatrixNear(epoch_held.position, epoch_hover_position, 1e-12,
                          "large-epoch endpoint tolerance changed hold position");
        requireMatrixNear(epoch_held.velocity, Vec3::Zero(), 1e-12,
                          "large-epoch endpoint tolerance changed hold velocity");
        const auto epoch_window = dynamic_planner::sampleCommittedReferenceWindow(
            epoch_hover, just_after_explicit_end, config);
        requireTrue(epoch_window.samples.size() == 61U,
                    "large-epoch rolling reference must contain 61 samples");
        for (const auto& sample : epoch_window.samples) {
            requireMatrixNear(sample.state.position, epoch_hover_position, 1e-12,
                              "large-epoch reference hold drifted");
        }

        // Generic timestamp-shift utility regression. This verifies only the
        // geometric/numeric behavior of shiftedToStartTime(). C1F.8c explicitly
        // forbids using this utility to grant execution authority for a trajectory
        // certified against moving obstacles at different absolute times.
        {
            const double prepared_start = 1788356804.25;
            const double grant_start = prepared_start + 123.456789;
            dynamic_planner::CommittedTrajectory prepared;
            const auto prepared_piece = oneSegmentPiece(
                prepared_start, prepared_start + 2.0,
                Vec3(0.4, -0.2, 2.5),
                Vec3(0.4, -0.2, 2.5),
                Vec3(0.8, 0.1, 2.5),
                Vec3(1.2, 0.4, 2.5));
            requireTrue(
                prepared.replaceSuffix(prepared_start, prepared_piece, 1e-10).accepted,
                "failed to create moving prepared commit");
            const auto rebased = prepared.shiftedToStartTime(grant_start);
            requireTrue(std::abs(rebased.startTime() - grant_start) < 2e-6,
                        "rebased commit start mismatch at ROS epoch scale");
            requireTrue(std::abs((rebased.endTime() - rebased.startTime()) -
                                 (prepared.endTime() - prepared.startTime())) < 2e-6,
                        "rebasing changed committed duration");
            for (const double offset_s : {0.0, 0.4, 1.2, 2.0}) {
                requireStateNear(
                    rebased.evaluate(grant_start + offset_s),
                    prepared.evaluate(prepared_start + offset_s),
                    2e-6);
            }
        }

        std::cout << "C.1a reference-window contract: PASS\n";
        std::cout << "samples: " << config.requiredSampleCount() << "\n";
        std::cout << "reference rate: " << config.reference_rate_hz << " Hz\n";
        std::cout << "MPC horizon: " << config.horizonDurationS() << " s\n";
        std::cout << "artificial solve delay: 150 ms; future splice preserved: true\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_reference_window FAILED: " << exc.what() << '\n';
        return 1;
    }
}
