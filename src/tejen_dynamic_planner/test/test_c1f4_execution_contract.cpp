#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::CooperativeObstacleTrajectory;
using dynamic_planner::State;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;
using dynamic_planner::WorldSnapshot;

namespace {

void requireTrue(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(double actual, double expected, double tolerance,
                 const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected));
    }
}

CommittedTrajectory stationaryTrajectory(const Vec3& position, double t0, double t1) {
    TrajectoryPiece piece;
    piece.valid_from = t0;
    piece.valid_until = t1;
    piece.knots = dynamic_planner::openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    for (Eigen::Index row = 0; row < 4; ++row) {
        piece.control_points.row(row) = position.transpose();
    }
    CommittedTrajectory result;
    requireTrue(result.replaceSuffix(t0, piece).accepted,
                "failed to create stationary cooperative trajectory");
    return result;
}

}  // namespace

int main() {
    try {
        // C1F.4 execution fallback: preserve measured p/v, bound initial
        // acceleration, and terminate in the existing certified stopped hold.
        State measured;
        measured.position = Vec3(0.0, 0.0, 1.5);
        measured.velocity = Vec3(0.60, -0.30, 0.10);
        const Vec3 brake_limit(0.75, 0.75, 1.00);
        const auto brake = dynamic_planner::makeBrakingHoverTrajectory(
            measured, 4.0, brake_limit, 0.25);
        const State initial = brake.evaluate(4.0);
        requireTrue((initial.position - measured.position).norm() < 1e-10,
                    "brake trajectory does not begin at measured position");
        requireTrue((initial.velocity - measured.velocity).norm() < 1e-10,
                    "brake trajectory does not begin at measured velocity");
        requireTrue((initial.acceleration.cwiseAbs().array() <=
                     brake_limit.array() + 1e-10).all(),
                    "brake trajectory exceeds configured acceleration limit");
        requireTrue(brake.endsInStoppedHold(),
                    "brake trajectory does not preserve stopped-endpoint contract");
        const State held = brake.evaluate(brake.endTime() + 0.5);
        requireTrue(held.velocity.norm() < 1e-10 && held.acceleration.norm() < 1e-10,
                    "post-brake terminal hold is not stationary");

        // C1F.4 robust geometry: the ego execution tube must be included in
        // cooperative C-space generation rather than applied only at runtime.
        WorldSnapshot world;
        world.captured_at_s = 0.0;
        world.ego_half_extents = Vec3::Constant(0.10);
        world.ego_tracking_error_half_extents = Vec3::Constant(0.02);
        CooperativeObstacleTrajectory obstacle;
        obstacle.name = "other";
        obstacle.trajectory = stationaryTrajectory(Vec3(0.50, 0.0, 1.5), 0.0, 5.0);
        obstacle.physical_half_extents = Vec3::Constant(0.10);
        obstacle.tracking_error_half_extents = Vec3::Constant(0.05);
        world.cooperative_obstacles.push_back(obstacle);
        const auto timed = world.timeIndexedObstacles(1, 0.0, 1.0);
        requireTrue(timed.size() == 1U, "unexpected body-only cooperative relation count");
        const auto& vertices = timed.front().interval_vertices.front();
        const double xmin = vertices.col(0).minCoeff();
        const double xmax = vertices.col(0).maxCoeff();
        // center 0.50 +/- (ego 0.10 + ego tracking 0.02 + other 0.10 + other tracking 0.05)
        requireNear(xmin, 0.23, 1e-10, "ego tracking tube missing from cooperative xmin");
        requireNear(xmax, 0.77, 1e-10, "ego tracking tube missing from cooperative xmax");

        // The independent final checker must use the same ego execution tube.
        // Place the other body just outside the old nominal envelope but inside
        // the robust envelope: nominal check is safe; robust check must collide.
        WorldSnapshot checker_world;
        checker_world.captured_at_s = 0.0;
        checker_world.ego_half_extents = Vec3::Constant(0.10);
        CooperativeObstacleTrajectory checker_obstacle;
        checker_obstacle.name = "checker_other";
        checker_obstacle.trajectory = stationaryTrajectory(Vec3(0.28, 0.0, 0.0), 0.0, 5.0);
        checker_obstacle.physical_half_extents = Vec3::Constant(0.10);
        checker_obstacle.tracking_error_half_extents = Vec3::Constant(0.05);
        checker_world.cooperative_obstacles.push_back(checker_obstacle);
        const auto ego_hold = dynamic_planner::makeStationaryHoverTrajectory(
            Vec3::Zero(), 0.0, 1.0);
        dynamic_planner::TrajectorySafetyChecker checker;
        const auto nominal_safety = checker.checkCommitted(
            ego_hold, 0.0, 0.5, checker_world);
        requireTrue(nominal_safety.safe,
                    "nominal final-checker baseline unexpectedly collided");
        checker_world.ego_tracking_error_half_extents = Vec3::Constant(0.05);
        const auto robust_safety = checker.checkCommitted(
            ego_hold, 0.0, 0.5, checker_world);
        requireTrue(!robust_safety.safe && robust_safety.status == "COLLISION",
                    "final checker did not include ego execution tracking tube");

        std::cout << "test_c1f4_execution_contract: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_c1f4_execution_contract: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
