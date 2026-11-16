#pragma once

#include <optional>
#include <vector>

#include "dynamic_planner/bspline.hpp"

namespace dynamic_planner {

// Absolute ROS timestamps are O(1e9 s). At that scale IEEE-754 double spacing
// is O(1e-7 s), so sub-nanosecond boundary epsilons are not meaningful.
inline constexpr double kTrajectoryTimeToleranceS = 1e-6;

struct TrajectoryPiece {
    ControlPoints control_points;
    std::vector<double> knots;
    double valid_from = 0.0;
    double valid_until = 0.0;

    State evaluate(double t) const;
    void validate() const;
};

struct SpliceDiagnostics {
    bool accepted = false;
    double position_error = 0.0;
    double velocity_error = 0.0;
    double acceleration_error = 0.0;
};

class CommittedTrajectory {
public:
    bool empty() const noexcept { return pieces_.empty(); }
    std::size_t pieceCount() const noexcept { return pieces_.size(); }

    double startTime() const;
    // End of the last explicit spline piece. If the final piece ends at a
    // certified stop, evaluate(t) remains defined for later times as a
    // stationary terminal hold.
    double endTime() const;
    bool endsInStoppedHold(double tolerance = 1e-9) const;
    State evaluate(double t) const;
    State endState() const;

    // Return an identical committed trajectory with every spline/logical timestamp
    // translated so that the first piece begins at new_start_time. Geometry and
    // derivatives are unchanged because only the knot time origin moves.
    CommittedTrajectory shiftedToStartTime(double new_start_time) const;

    // Atomically replace t >= splice_time. The old prefix remains represented by
    // its original spline/control points; only its logical valid_until may be trimmed.
    SpliceDiagnostics replaceSuffix(double splice_time,
                                    const TrajectoryPiece& candidate,
                                    double continuity_tolerance = 1e-7);

    // Append an explicit emergency/backup continuation after the current end.
    // Position and velocity must be continuous; acceleration may jump because
    // this is a separately certified fallback tail, not a nominal C2 replan splice.
    // The method is intentionally append-only and never relaxes replaceSuffix().
    SpliceDiagnostics appendVelocityContinuousBackup(
        const TrajectoryPiece& backup,
        double continuity_tolerance = 1e-7);

    std::vector<State> sample(double t0, double t1, double dt) const;
    const std::vector<TrajectoryPiece>& pieces() const noexcept { return pieces_; }

private:
    std::vector<TrajectoryPiece> pieces_;
};

// Seed active planner authority from an exactly stationary incumbent. The
// finite seed piece ends at zero velocity/acceleration, so the existing
// CommittedTrajectory terminal-hold contract extends it indefinitely until a
// future suffix is safely committed.
CommittedTrajectory makeStationaryHoverTrajectory(
    const Vec3& position,
    double start_time_s,
    double seed_duration_s = 0.10);

// Fail-safe reference. Starting exactly from the measured position and
// velocity, build a one-segment cubic that monotonically brakes to a stopped
// terminal hold. The duration is chosen so the initial braking acceleration on
// every axis stays within brake_accel_limit.
CommittedTrajectory makeBrakingHoverTrajectory(
    const State& measured_state,
    double start_time_s,
    const Vec3& brake_accel_limit,
    double minimum_duration_s = 0.25);

}  // namespace dynamic_planner
