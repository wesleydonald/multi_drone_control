#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/octopus_search.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using dynamic_planner::ControlPoints;
using dynamic_planner::CommittedTrajectory;
using dynamic_planner::CooperativeObstacleTrajectory;
using dynamic_planner::OctopusConfig;
using dynamic_planner::OctopusResult;
using dynamic_planner::OctopusSearch;
using dynamic_planner::State;
using dynamic_planner::TimeIndexedObstacle;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::TrajectorySafetyChecker;
using dynamic_planner::Vec3;
using dynamic_planner::WorldSnapshot;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

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

TrajectoryPiece linearPiece(const Vec3& a, const Vec3& b, double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = dynamic_planner::openUniformKnots(t0, t1, 1);
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
    CommittedTrajectory trajectory;
    const auto diagnostics = trajectory.replaceSuffix(t0, linearPiece(p, p, t0, t1));
    require(diagnostics.accepted, "failed to construct constant cooperative trajectory");
    return trajectory;
}

OctopusConfig config() {
    OctopusConfig cfg;
    cfg.v_max = Vec3::Constant(2.0);
    cfg.a_max = Vec3::Constant(2.0);
    cfg.samples_per_axis = {9, 9, 9};
    cfg.alpha_shrink = 0.9;
    cfg.voxel_fraction = 0.10;
    cfg.heuristic_bias = 1.0;
    cfg.goal_tolerance_m = 0.10;
    cfg.max_runtime_s = 1.0;
    cfg.xyz_min = Vec3(-1.0, -1.0, 0.0);
    cfg.xyz_max = Vec3(3.0, 1.0, 2.0);
    cfg.planning_radius_m = 30.0;
    cfg.random_seed = 1;
    return cfg;
}

void testDoomedEndpointIsRejectedDuringSearch() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    State state;
    state.position = Vec3(0.0, 0.0, 1.0);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);
    const Vec3 goal(1.0, 0.0, 1.0);

    TimeIndexedObstacle guard;
    guard.name = "future_sweep_guard";
    // Six 0.5 s guard intervals would be produced by the C1F.5 world path for
    // a 3 s terminal guard. Repeating the same box makes this unit test isolate
    // Octopus terminal feasibility rather than moving-world construction.
    guard.interval_vertices.assign(
        6U, boxVertices(goal, Vec3(0.20, 0.20, 0.20)));

    const OctopusResult result = OctopusSearch(
        knots, q012, goal, {}, std::vector<TimeIndexedObstacle>{guard}, config()).search();

    require(result.terminal_hold_rejections > 0,
            "C1F.5 should reject at least one complete endpoint swept during the hold guard");
    require(result.terminal_hold_separator_lp_calls > 0,
            "C1F.5 terminal viability should execute separator checks");
    require(result.success,
            "C1F.5 should continue searching and return a viable alternative; status=" + result.status);
    require(result.control_points.has_value(), "successful C1F.5 search must return control points");
    const Vec3 terminal = result.control_points->row(result.control_points->rows() - 1).transpose();
    require((terminal - goal).norm() > 0.10,
            "terminal-guard search unexpectedly returned the guarded goal endpoint");
}

void testAabbBroadPhaseSkipsObviouslySeparatedSharedObstacle() {
    const TrajectoryPiece ego = linearPiece(
        Vec3(0.0, 0.0, 1.5), Vec3(1.0, 0.0, 1.5), 0.0, 1.0);

    WorldSnapshot world;
    world.ego_half_extents = Vec3(0.10, 0.10, 0.10);
    CooperativeObstacleTrajectory cooperative;
    cooperative.name = "far_shared_companion";
    cooperative.trajectory = constantTrajectory(Vec3(0.5, 2.0, 1.5), 0.0, 1.0);
    cooperative.physical_half_extents = Vec3(0.10, 0.10, 0.10);
    cooperative.tracking_error_half_extents = Vec3::Constant(0.05);
    world.cooperative_obstacles.push_back(cooperative);

    const auto safety = TrajectorySafetyChecker().checkPiece(ego, 0.0, 1.0, world);
    require(safety.safe, "far shared companion was incorrectly rejected");
    require(safety.separator_lp_calls == 0U,
            "AABB-disjoint shared companion still invoked exact LP separator");
    require(safety.aabb_separation_skips > 0U,
            "C1F.5p broad phase did not record any exact separation skips");
}

}  // namespace

int main() {
    try {
        testDoomedEndpointIsRejectedDuringSearch();
        testAabbBroadPhaseSkipsObviouslySeparatedSharedObstacle();
        std::cout << "test_c1f5_terminal_viability: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_c1f5_terminal_viability: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
