#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "test_common.hpp"

#include <iostream>
#include <Eigen/Geometry>

using namespace dynamic_planner;

namespace {

Eigen::MatrixXd boxVertices(const Vec3& center, const Vec3& half) {
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            for (double sz : {-1.0, 1.0}) {
                vertices.row(row++) =
                    (center + Vec3(sx * half.x(), sy * half.y(), sz * half.z())).transpose();
            }
        }
    }
    return vertices;
}

TrajectoryPiece linearPiece(const Vec3& a, const Vec3& b,
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
    return piece;
}

CommittedTrajectory constantTrajectory(const Vec3& p, double t0, double t1) {
    TrajectoryPiece piece = linearPiece(p, p, t0, t1);
    CommittedTrajectory result;
    requireTrue(result.replaceSuffix(t0, piece).accepted,
                "failed to construct constant trajectory");
    return result;
}

void testChangedRelationChecking() {
    TrajectorySafetyChecker checker;
    const auto ego = linearPiece(Vec3(0, 0, 1.3), Vec3(.1, 0, 1.3), 0, 1);
    WorldSnapshot base;
    base.workspace.enabled = true;
    base.workspace.lower = Vec3(-2, -2, 0);
    base.workspace.upper = Vec3(2, 2, 3);
    base.ego_suspended_geometry.enabled = base.ego_suspended_geometry.payload_attached = true;
    base.ego_suspended_geometry.swing_envelope = SwingEnvelope::OuterOctagon;
    base.physical_static_obstacles.push_back({"wall", boxVertices(Vec3(1, 1, 1), Vec3(.1, .1, 1))});
    CooperativeObstacleTrajectory peer;
    peer.name = "peer";
    peer.trajectory = constantTrajectory(Vec3(-1, 1, 1.3), 0, 2);
    base.cooperative_obstacles.push_back(peer);
    requireTrue(checker.checkPiece(ego, 0, 1, base).safe, "invalid incremental baseline");
    auto latest = base;
    latest.ego_body_rotation = Eigen::AngleAxisd(.02, Vec3::UnitY()).toRotationMatrix();
    auto full = checker.checkPiece(ego, 0, 1, latest);
    auto selected = checker.checkPiece(ego, 0, 1, latest, latest.collisionChangesSince(base));
    requireTrue(selected.safe == full.safe && selected.relations_reused > 0 &&
                selected.relations_checked < full.relations_checked,
                "rotation catch-up still rechecks every component");
    requireTrue(selected.status == "SAFE_CHANGED_RELATIONS", "subset masquerades as full certificate");
    latest = base;
    latest.cooperative_obstacles[0].trajectory = constantTrajectory(Vec3(.05, 0, 1.3), 0, 2);
    full = checker.checkPiece(ego, 0, 1, latest);
    selected = checker.checkPiece(ego, 0, 1, latest, latest.collisionChangesSince(base));
    requireTrue(!selected.safe && selected.safe == full.safe, "changed peer collision missed");
    latest = base;
    latest.ego_half_extents = Vec3(.1, .1, .35);
    latest.physical_static_obstacles[0] = {"rotation_blocker", boxVertices(Vec3(.3, 0, 1.3), Vec3(.03, .1, .1))};
    const auto rotated_base = latest;
    requireTrue(checker.checkPiece(ego, 0, 1, rotated_base).safe, "rotation fixture initially blocked");
    latest.ego_body_rotation = Eigen::AngleAxisd(std::acos(-1.)/2, Vec3::UnitY()).toRotationMatrix();
    full = checker.checkPiece(ego, 0, 1, latest);
    selected = checker.checkPiece(ego, 0, 1, latest, latest.collisionChangesSince(rotated_base));
    requireTrue(!selected.safe && selected.safe == full.safe, "changed body collision missed");
    latest = base;
    const auto unchanged = checker.checkPiece(ego, 0, 1, latest, latest.collisionChangesSince(base));
    requireTrue(unchanged.safe && unchanged.relations_checked == 0 && unchanged.relations_reused > 0,
                "unchanged input performed collision work");
    for (int variant = 0; variant < 16; ++variant) {
        latest = base;
        latest.ego_body_rotation = Eigen::AngleAxisd(.03*(variant+1), Vec3::UnitY()).toRotationMatrix();
        latest.cooperative_obstacles[0].trajectory = constantTrajectory(
            Vec3(.05, .08*variant, 1.3), 0, 2);
        full = checker.checkPiece(ego, .2, .9, latest);
        selected = checker.checkPiece(ego, .2, .9, latest, latest.collisionChangesSince(base));
        requireTrue(full.safe == selected.safe, "combined body/peer check differs from full validation");
        if (!full.safe) requireTrue(full.obstacle_name == selected.obstacle_name,
                                  "combined change lost the actual blocker");
        if (full.safe) requireTrue(selected.relations_checked <= full.relations_checked,
                                  "combined selection duplicated relation checks");
    }
}

}  // namespace

int main() {
    try {
        testChangedRelationChecking();
        const TrajectoryPiece ego = linearPiece(
            Vec3(0.0, 0.0, 1.5), Vec3(1.0, 0.0, 1.5), 0.0, 1.0);
        TrajectorySafetyChecker checker;
        {
            WorldSnapshot workspace;
            workspace.workspace.enabled = true;
            workspace.workspace.lower = Vec3(-1., -1., 0.);
            workspace.workspace.upper = Vec3(1., 1., 2.);
            workspace.ego_half_extents = Vec3(.1, .1, .3);
            workspace.ego_tracking_error_half_extents = Vec3(.05, .05, .05);
            const auto near_side = linearPiece(Vec3(.8, 0., 1.), Vec3(.8, 0., 1.), 0., 1.);
            requireTrue(checker.checkPiece(near_side, 0., 1., workspace).safe,
                        "identity body should fit workspace");
            workspace.ego_body_rotation = Eigen::AngleAxisd(
                3.14159265358979323846 / 2., Vec3::UnitY()).toRotationMatrix();
            const auto rotated = checker.checkPiece(near_side, 0., 1., workspace);
            requireTrue(!rotated.safe && rotated.status == "WORKSPACE_BOUNDS",
                        "latest rotation must be used by workspace final checking");
        }

        WorldSnapshot safe_world;
        safe_world.captured_at_s = 0.0;
        safe_world.static_obstacles.push_back(StaticConvexObstacle{
            "far", boxVertices(Vec3(0.5, 1.0, 1.5), Vec3(0.1, 0.1, 0.1))});
        const auto safe = checker.checkPiece(ego, 0.0, 1.0, safe_world);
        requireTrue(safe.safe, "far static obstacle rejected exact cubic path");
        requireTrue(safe.separator_lp_calls == 0U,
                    "AABB-disjoint static obstacle still invoked LP separator");
        requireTrue(safe.aabb_separation_skips > 0U,
                    "AABB broad phase did not report static separation skip");

        WorldSnapshot blocked_world;
        blocked_world.captured_at_s = 0.0;
        blocked_world.static_obstacles.push_back(StaticConvexObstacle{
            "blocked", boxVertices(Vec3(0.5, 0.0, 1.5), Vec3(0.1, 0.1, 0.1))});
        const auto blocked = checker.checkPiece(ego, 0.0, 1.0, blocked_world);
        requireTrue(!blocked.safe, "intersecting static obstacle was missed");
        requireTrue(blocked.obstacle_name == "blocked", "wrong blocking obstacle reported");
        requireTrue(blocked.separator_lp_calls > 0U,
                    "overlapping static AABBs skipped exact LP separator");

        WorldSnapshot cooperative_world;
        cooperative_world.captured_at_s = 0.0;
        cooperative_world.ego_half_extents = Vec3(0.10, 0.10, 0.10);
        CooperativeObstacleTrajectory cooperative;
        cooperative.name = "carrier";
        cooperative.trajectory = constantTrajectory(Vec3(0.5, 0.25, 1.5), 0.0, 1.0);
        cooperative.physical_half_extents = Vec3(0.10, 0.10, 0.10);
        cooperative.tracking_error_half_extents = Vec3::Constant(0.05);
        cooperative_world.cooperative_obstacles.push_back(cooperative);
        const auto cooperative_blocked = checker.checkPiece(
            ego, 0.0, 1.0, cooperative_world);
        requireTrue(!cooperative_blocked.safe,
                    "cooperative body+5cm tracking+ego tube was not applied");
        requireTrue(cooperative_blocked.separator_lp_calls > 0U,
                    "overlapping cooperative AABBs skipped exact LP separator");

        cooperative_world.cooperative_obstacles.front().trajectory =
            constantTrajectory(Vec3(0.5, 1.0, 1.5), 0.0, 1.0);
        const auto cooperative_safe = checker.checkPiece(
            ego, 0.0, 1.0, cooperative_world);
        requireTrue(cooperative_safe.safe,
                    "far cooperative trajectory was incorrectly rejected");
        requireTrue(cooperative_safe.separator_lp_calls == 0U,
                    "AABB-disjoint cooperative trajectory still invoked LP separator");
        requireTrue(cooperative_safe.aabb_separation_skips > 0U,
                    "AABB broad phase did not report cooperative separation skip");

        std::cout << "test_trajectory_safety_checker: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_trajectory_safety_checker: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
