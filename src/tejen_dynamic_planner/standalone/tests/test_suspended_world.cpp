#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "test_common.hpp"

#include <iostream>

using namespace dynamic_planner;

namespace {

TrajectoryPiece linearPiece(const Vec3& a, const Vec3& b, double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = a.transpose();
    piece.control_points.row(1) = (a + (b - a) / 3.0).transpose();
    piece.control_points.row(2) = (a + 2.0 * (b - a) / 3.0).transpose();
    piece.control_points.row(3) = b.transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    return piece;
}

CommittedTrajectory constantTrajectory(const Vec3& p, double t0, double t1) {
    CommittedTrajectory result;
    requireTrue(result.replaceSuffix(t0, linearPiece(p, p, t0, t1)).accepted,
                "failed to construct constant trajectory");
    return result;
}

SuspendedGeometry suspended(bool attached = false) {
    SuspendedGeometry g;
    g.enabled = true;
    g.payload_attached = attached;
    return g;
}

}  // namespace

int main() {
    try {
        TrajectorySafetyChecker checker;
        const TrajectoryPiece ego = linearPiece(
            Vec3(0.0, 0.0, 1.5), Vec3(1.0, 0.0, 1.5), 0.0, 1.0);

        // The drone body clears this low obstacle, but the 50 cm cable / magnet
        // assembly intersects it. This is the B.4 overflight trap.
        WorldSnapshot body_only;
        body_only.captured_at_s = 0.0;
        body_only.physical_static_obstacles.push_back(StaticConvexObstacle{
            "low_box",
            axisAlignedBoxVertices(Vec3(0.5, 0.0, 0.98), Vec3(0.08, 0.08, 0.05))});
        const auto body_safe = checker.checkPiece(ego, 0.0, 1.0, body_only);
        requireTrue(body_safe.safe, "body-only path should clear low physical obstacle");

        WorldSnapshot suspended_world = body_only;
        suspended_world.ego_suspended_geometry = suspended(false);
        const auto static_timed = suspended_world.timeIndexedObstacles(4, 0.0, 1.0);
        requireTrue(static_timed.size() == 3U,
                    "unattached B.4 static obstacle should create body+cable+magnet C-space constraints");
        requireTrue(static_timed[1].name == "low_box::ego_cable",
                    "planner-side static cable C-space component missing");

        const auto whole_body_blocked = checker.checkPiece(
            ego, 0.0, 1.0, suspended_world);
        requireTrue(!whole_body_blocked.safe,
                    "whole suspended assembly failed to detect overflight collision");
        requireTrue(whole_body_blocked.obstacle_name.find("low_box::ego_") == 0,
                    "whole-body static collision did not identify component");

        // Other drone body is vertically well clear, but its cable hangs through
        // the planning drone's flight level.
        WorldSnapshot other_body_only;
        other_body_only.captured_at_s = 0.0;
        CooperativeObstacleTrajectory other;
        other.name = "other_drone";
        other.trajectory = constantTrajectory(Vec3(0.5, 0.0, 2.05), 0.0, 1.0);
        other.tracking_error_half_extents = Vec3::Constant(0.05);
        other_body_only.cooperative_obstacles.push_back(other);
        const auto body_pair_safe = checker.checkPiece(ego, 0.0, 1.0, other_body_only);
        requireTrue(body_pair_safe.safe,
                    "body-only drones should be vertically separated in cable test");

        WorldSnapshot other_cable_world = other_body_only;
        other_cable_world.cooperative_obstacles.front().suspended_geometry = suspended(false);
        const auto cooperative_timed = other_cable_world.timeIndexedObstacles(4, 0.0, 1.0);
        bool saw_other_cable_vs_body = false;
        for (const auto& timed : cooperative_timed) {
            saw_other_cable_vs_body = saw_other_cable_vs_body ||
                timed.name == "other_drone::cable_vs_ego_body";
        }
        requireTrue(saw_other_cable_vs_body,
                    "planner-side other-drone cable C-space relation missing");

        const auto cable_blocked = checker.checkPiece(
            ego, 0.0, 1.0, other_cable_world);
        requireTrue(!cable_blocked.safe,
                    "planning drone failed to detect fixed other-drone cable");
        requireTrue(cable_blocked.obstacle_name.find("other_drone::cable") == 0,
                    "other-drone cable was not the reported blocking component");

        // Attachment is snapshot geometry and must advance the authoritative world
        // version, so Check/Recheck can reject a stale pre-attachment candidate.
        VersionedWorld versioned;
        requireTrue(versioned.currentVersion() == 0U, "initial version mismatch");
        SuspendedGeometry attached = suspended(true);
        versioned.setEgoSuspendedGeometry(attached);
        requireTrue(versioned.currentVersion() == 1U,
                    "attachment geometry change did not advance world version");
        const WorldSnapshot attached_snapshot = versioned.snapshot(0.0);
        requireTrue(attached_snapshot.ego_suspended_geometry.payload_attached,
                    "attached payload state not preserved in snapshot");
        requireTrue(assemblyComponents(attached_snapshot.ego_half_extents,
                                       attached_snapshot.ego_suspended_geometry).size() == 4U,
                    "attached snapshot does not expose payload component");

        std::cout << "test_suspended_world: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_suspended_world: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
