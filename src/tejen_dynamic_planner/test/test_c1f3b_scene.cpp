#include "dynamic_planner/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::CooperativeObstacleTrajectory;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;
using dynamic_planner::WorldSnapshot;

namespace {

void requireTrue(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

CommittedTrajectory constantVelocity(
    const Vec3& p,
    const Vec3& v,
    double t0,
    double t1) {
    const double duration = t1 - t0;
    requireTrue(duration > 0.0, "invalid test trajectory duration");
    TrajectoryPiece piece;
    piece.knots = dynamic_planner::openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = p.transpose();
    piece.control_points.row(1) = (p + v * (duration / 3.0)).transpose();
    piece.control_points.row(2) = (p + v * (2.0 * duration / 3.0)).transpose();
    piece.control_points.row(3) = (p + v * duration).transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    CommittedTrajectory result;
    requireTrue(result.replaceSuffix(t0, piece).accepted,
                "failed to construct cooperative test trajectory");
    return result;
}

SuspendedGeometry loadedGeometry() {
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 0.50;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = 15.0 * 3.14159265358979323846 / 180.0;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    geometry.payload_attached = true;
    geometry.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    geometry.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return geometry;
}

bool hasName(const std::vector<dynamic_planner::TimeIndexedObstacle>& obstacles,
             const std::string& name) {
    return std::any_of(obstacles.begin(), obstacles.end(), [&](const auto& obstacle) {
        return obstacle.name == name;
    });
}

}  // namespace

int main() {
    try {
        WorldSnapshot world;
        world.captured_at_s = 0.0;
        world.ego_half_extents = Vec3(0.105, 0.105, 0.060);
        world.ego_suspended_geometry = loadedGeometry();

        for (int i = 0; i < 2; ++i) {
            CooperativeObstacleTrajectory obstacle;
            obstacle.name = "cooperative_drone_" + std::to_string(i);
            const double y = i == 0 ? 0.50 : -0.50;
            obstacle.trajectory = constantVelocity(
                Vec3(1.0, y, 1.8), Vec3(0.10, 0.0, 0.0), 0.0, 20.0);
            obstacle.physical_half_extents = Vec3(0.105, 0.105, 0.060);
            obstacle.tracking_error_half_extents = Vec3::Constant(0.10);
            obstacle.suspended_geometry.enabled = false;
            world.cooperative_obstacles.push_back(std::move(obstacle));
        }

        world.validate();
        const auto timed = world.timeIndexedObstacles(4, 0.0, 6.0);

        // Each body-only companion is paired independently with the loaded ego's
        // body, cable, magnet and payload components.
        requireTrue(timed.size() == 8U,
                    "expected 2 companions x 4 loaded ego component relations");
        for (int i = 0; i < 2; ++i) {
            const std::string prefix = "cooperative_drone_" + std::to_string(i) + "::body_vs_ego_";
            requireTrue(hasName(timed, prefix + "body"), "missing body cooperative relation");
            requireTrue(hasName(timed, prefix + "cable"), "missing cable cooperative relation");
            requireTrue(hasName(timed, prefix + "magnet"), "missing magnet cooperative relation");
            requireTrue(hasName(timed, prefix + "payload"), "missing payload cooperative relation");
        }

        for (const auto& obstacle : timed) {
            requireTrue(obstacle.interval_vertices.size() == 4U,
                        "cooperative obstacle was not time-indexed across all local segments");
            for (const auto& vertices : obstacle.interval_vertices) {
                requireTrue(vertices.rows() > 0 && vertices.cols() == 3 && vertices.allFinite(),
                            "invalid cooperative C-space interval vertices");
            }
        }

        std::cout << "test_c1f3b_scene: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_c1f3b_scene: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
