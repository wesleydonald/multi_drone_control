#include "dynamic_planner/world_snapshot.hpp"
#include "test_common.hpp"

#include <iostream>

using namespace dynamic_planner;

namespace {

CommittedTrajectory linearTrajectory(const Vec3& a, const Vec3& b,
                                     double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = a.transpose();
    piece.control_points.row(1) = (a + (b - a) / 3.0).transpose();
    piece.control_points.row(2) = (a + 2.0 * (b - a) / 3.0).transpose();
    piece.control_points.row(3) = b.transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    CommittedTrajectory trajectory;
    requireTrue(trajectory.replaceSuffix(t0, piece).accepted,
                "failed to construct synthetic trajectory");
    return trajectory;
}

}  // namespace

int main() {
    try {
        WorldSnapshot snapshot;
        snapshot.captured_at_s = 0.0;
        snapshot.ego_half_extents = Vec3::Constant(0.10);

        CooperativeObstacleTrajectory cooperative;
        cooperative.name = "carrier";
        cooperative.trajectory = linearTrajectory(
            Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0), 0.0, 1.0);
        cooperative.physical_half_extents = Vec3::Constant(0.10);
        cooperative.tracking_error_half_extents = Vec3::Constant(0.05);
        snapshot.cooperative_obstacles.push_back(cooperative);

        const auto timed = snapshot.timeIndexedObstacles(1, 0.0, 1.0);
        requireTrue(timed.size() == 1U, "cooperative obstacle missing from timed world");
        const Eigen::MatrixXd& vertices = timed.front().interval_vertices.front();
        requireNear(vertices.col(0).minCoeff(), -0.25, 1e-10,
                    "cooperative hull x minimum does not include body+tracking+ego tube");
        requireNear(vertices.col(0).maxCoeff(), 1.25, 1e-10,
                    "cooperative hull x maximum does not include body+tracking+ego tube");
        requireNear(vertices.col(1).minCoeff(), -0.25, 1e-10,
                    "cooperative hull y minimum incorrect");
        requireNear(vertices.col(1).maxCoeff(), 0.25, 1e-10,
                    "cooperative hull y maximum incorrect");

        VersionedWorld world(snapshot);
        requireTrue(world.currentVersion() == 0U, "initial version should be zero");
        bool action_ran = false;
        requireTrue(world.runIfVersionCurrent(0U, [&]() { action_ran = true; }),
                    "atomic gate rejected current version");
        requireTrue(action_ran, "atomic gate did not run action");

        cooperative.tracking_error_half_extents = Vec3::Constant(0.06);
        world.upsertCooperativeTrajectory(cooperative);
        requireTrue(world.currentVersion() == 1U, "world update did not advance version");
        action_ran = false;
        requireTrue(!world.runIfVersionCurrent(0U, [&]() { action_ran = true; }),
                    "atomic gate accepted stale version");
        requireTrue(!action_ran, "stale atomic gate ran commit action");

        const WorldSnapshot updated = world.snapshot(0.5);
        requireTrue(updated.version == 1U, "snapshot version mismatch");
        requireNear(updated.cooperative_obstacles.front().tracking_error_half_extents.x(),
                    0.06, 1e-12, "updated tracking tube not preserved");

        // A segmented moving fixture must enter the world as one coherent scene
        // update. Twenty-four per-segment version bumps would make every planner
        // snapshot stale before certification can complete.
        CooperativeObstacleTrajectory segment_a = cooperative;
        segment_a.name = "m2d_ring_segment_00";
        CooperativeObstacleTrajectory segment_b = cooperative;
        segment_b.name = "m2d_ring_segment_01";
        const auto before_batch = world.currentVersion();
        world.upsertCooperativeTrajectories({segment_a, segment_b});
        requireTrue(world.currentVersion() == before_batch + 1U,
                    "cooperative batch update must advance world version exactly once");
        const WorldSnapshot batched = world.snapshot(0.5);
        bool found_a = false;
        bool found_b = false;
        for (const auto& obstacle : batched.cooperative_obstacles) {
            found_a = found_a || obstacle.name == "m2d_ring_segment_00";
            found_b = found_b || obstacle.name == "m2d_ring_segment_01";
        }
        requireTrue(found_a && found_b, "cooperative batch update lost segmented obstacles");

        // C1F.4: ego closed-loop tracking allowance is part of the planned
        // cooperative collision geometry, not merely a runtime warning.
        world.setEgoTrackingErrorHalfExtents(Vec3::Constant(0.02));
        requireTrue(world.currentVersion() == 3U,
                    "ego tracking tube update did not advance world version");
        const WorldSnapshot tracked = world.snapshot(0.5);
        requireNear(tracked.ego_tracking_error_half_extents.x(), 0.02, 1e-12,
                    "ego tracking tube not preserved in snapshot");
        const auto tracked_timed = tracked.timeIndexedObstacles(1, 0.0, 1.0);
        const Eigen::MatrixXd& tracked_vertices =
            tracked_timed.front().interval_vertices.front();
        requireNear(tracked_vertices.col(0).minCoeff(), -0.28, 1e-10,
                    "ego tracking tube missing from cooperative x inflation");
        requireNear(tracked_vertices.col(0).maxCoeff(), 1.28, 1e-10,
                    "ego tracking tube missing from cooperative x inflation");

        std::cout << "test_world_snapshot: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
