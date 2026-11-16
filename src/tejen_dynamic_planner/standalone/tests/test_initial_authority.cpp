#include "dynamic_planner/receding_horizon_planner.hpp"
#include "test_common.hpp"

using namespace dynamic_planner;

// Portable regression: both ROS recovery and the benchmark consume this actual
// planner policy; neither should interpret the adaptive sentinel as coverage.
int main() {
    const auto hold = makeStationaryHoverTrajectory(Vec3(0,0,1.5), 0., .1);
    LocalPlannerConfig local;
    VersionedWorld world;
    world.setPhysicalStaticObstacles({{"overlap", axisAlignedBoxVertices(
        Vec3(0,0,1.5), Vec3(.02,.02,.02))}});
    for (double fixed : {0., .1}) {
        RecedingHorizonConfig config;
        config.fixed_splice_lookahead_s = fixed;
        config.temporal_refinement.enabled = true;
        RecedingHorizonPlanner planner(Vec3(3,0,1.5), config, local);
        TrajectorySafetyChecker checker(config.separator_validation_tolerance,
                                         config.temporal_refinement);
        requireNear(planner.chooseSpliceLookahead(), fixed > 0. ? fixed : 2.5/30.,
                    1e-12, "zero-history startup coverage differs from policy");
        requireTrue(!checker.checkCommitted(hold, 0., planner.chooseSpliceLookahead(),
                                            world.snapshot(0.)).safe,
                    "initial collision accepted via empty certification");
        planner.setPreviousSpliceTiming(.09);
        requireNear(planner.chooseSpliceLookahead(), fixed > 0. ? fixed : .25,
                    1e-12, "coverage ignores actual history");
    }
    VersionedWorld crossing_world;
    CooperativeObstacleTrajectory peer;
    peer.name = "clear_start_crossing";
    peer.physical_half_extents = Vec3::Constant(.02);
    peer.tracking_error_half_extents.setZero();
    TrajectoryPiece moving;
    moving.valid_from = 0.; moving.valid_until = 1.;
    moving.knots = openUniformKnots(0.,1.,1);
    moving.control_points.resize(4,3);
    for (int i=0; i<4; ++i)
        moving.control_points.row(i) = Vec3(.14-.35*i/3.,0,1.5).transpose();
    requireTrue(peer.trajectory.replaceSuffix(0.,moving).accepted, "invalid fixture");
    crossing_world.upsertCooperativeTrajectory(peer);
    RecedingHorizonConfig config;
    config.temporal_refinement.enabled = true;
    RecedingHorizonPlanner planner(Vec3(3,0,1.5), config, local);
    TrajectorySafetyChecker checker(config.separator_validation_tolerance, config.temporal_refinement);
    requireTrue(checker.checkCommitted(hold,0.,.001,crossing_world.snapshot(0.)).safe,
                "moving fixture must start clear");
    requireTrue(!checker.checkCommitted(hold,0.,planner.chooseSpliceLookahead(),
                                      crossing_world.snapshot(0.)).safe,
                "moving collision in the initial lead missed");
}
