#include "dynamic_planner/receding_horizon_planner.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <iostream>
#include <map>

using namespace dynamic_planner;

namespace {
#include "fixtures/finish_timing_oct30.hpp"

WorldSnapshot finishWorld(const FinishFixture& fixture) {
    WorldSnapshot world;
    world.ego_half_extents = Vec3(.105, .105, .060);
    world.ego_tracking_error_half_extents = Vec3(.10, .10, .05);
    world.ego_body_rotation = fixture.orientation.normalized().toRotationMatrix();
    auto& geometry = world.ego_suspended_geometry;
    geometry.enabled = geometry.payload_attached = true;
    geometry.swing_envelope = SwingEnvelope::OuterOctagon;
    geometry.cable_length_m = 1.039;
    geometry.magnet_center_below_cable_end_m = 0.;
    geometry.payload_half_extents = Vec3(.043, .043, .011);
    geometry.payload_center_from_magnet_center = Vec3(0., 0., -.0315);
    world.workspace.enabled = true;
    world.workspace.lower = Vec3(-2.5, -2.5, 0.);
    world.workspace.upper = Vec3(2.5, 2.5, 2.);
    for (std::size_t i = 0; i < fixture.vertices.size(); i += 8) {
        Eigen::MatrixXd vertices(8, 3);
        for (int row = 0; row < 8; ++row) vertices.row(row) = fixture.vertices[i + row];
        world.physical_static_obstacles.push_back({"panel_" + std::to_string(i / 8), vertices});
    }
    world.validate();
    return world;
}

LocalPlannerConfig finishLocal(const State& initial, const Vec3& goal) {
    // Same ROS localConfig inputs; snapshot geometry supplies request-time erosion.
    LocalPlannerConfig local;
    local.num_segments = 4;
    local.octopus.samples_per_axis = {7, 7, 7};
    local.octopus.alpha_shrink = .9;
    local.octopus.voxel_fraction = .10;
    local.octopus.heuristic_bias = 1.;
    local.octopus.max_runtime_s = .05;
    local.octopus.random_seed = 1;
    local.octopus.xyz_min = initial.position.cwiseMin(goal) - Vec3::Ones();
    local.octopus.xyz_max = initial.position.cwiseMax(goal) + Vec3::Ones();
    local.octopus.xyz_min.z() = std::max(.2, local.octopus.xyz_min.z());
    local.refinement.xyz_min = local.octopus.xyz_min;
    local.refinement.xyz_max = local.octopus.xyz_max;
    local.refinement.j_max = Vec3(6.5, 6.5, 4.);
    local.refinement.jerk_weight = 1.;
    local.refinement.goal_weight = 10.;
    local.refinement.max_working_set_recalculations = 500;
    return local;
}

CommittedTrajectory initialCommitment(const State& initial) {
    // Original future control points are not logged. Reconstruct an identical
    // C2 stopped seed for both arms, retaining the recorded nonzero P/V/A.
    TrajectoryPiece piece;
    piece.valid_from = 0.; piece.valid_until = 2.;
    piece.knots = openUniformKnots(0., 2., 4);
    ControlPoints prefix(5, 3);
    prefix.topRows(3) = initialControlPointsFromState(initial, piece.knots);
    prefix.row(3) = prefix.row(4) = prefix.row(2);
    piece.control_points = stoppingCompletion(prefix, 4);
    CommittedTrajectory seed;
    requireTrue(seed.replaceSuffix(0., piece).accepted, "seed installation failed");
    const auto actual = seed.evaluate(0.);
    requireMatrixNear(actual.position, initial.position, 1e-10, "seed P");
    requireMatrixNear(actual.velocity, initial.velocity, 1e-10, "seed V");
    requireMatrixNear(actual.acceleration, initial.acceleration, 1e-10, "seed A");
    return seed;
}

struct FinishResult {
    double first_capture_s = -1.;
    double distance_at_two_s = 0.;
    double worst_runtime_s = 0.;
    int accepted = 0;
    int rejected = 0;
    std::map<std::string, int> rejections;
};

FinishResult runFinish(const FinishFixture& fixture, double close_threshold) {
    const Vec3 goal(1.5, -1.5, 1.48);
    const auto snapshot = finishWorld(fixture);
    VersionedWorld world(snapshot);
    RecedingHorizonConfig config;
    config.min_splice_lookahead_s = .1;
    config.factor_alloc = 1.;
    config.factor_alloc_close = 2.5;
    config.spline_time_factor = 1.25;
    config.close_to_goal_m = close_threshold;
    config.v_max = Vec3(2.25, 2.25, 1.);
    config.a_max = Vec3(1.9, 1.9, 1.5);
    config.require_terminal_hold_for_nominal = false;
    config.defer_incumbent_conflict_until_reaction_horizon = true;
    RecedingHorizonPlanner planner(goal, config, finishLocal(fixture.initial, goal));
    const auto seed = initialCommitment(fixture.initial);
    requireTrue(TrajectorySafetyChecker().checkCommitted(seed, 0., seed.endTime(), snapshot).safe,
                "reconstructed initial commitment is not collision-free");
    planner.initializeCommittedTrajectory(seed);
    FinishResult metrics;
    // Native ideal-reference evolution, not a Gazebo/controller/flight replay.
    // Identical synthetic 10ms trajectory-clock solve delay isolates allocation
    // from scheduling noise; real wall runtimes are reported separately.
    for (int tick = 0; tick <= 100; ++tick) {
        const double now = tick * .2;
        const State state = planner.committedTrajectory().evaluate(now);
        const double distance = (state.position - goal).head<2>().norm();
        if (tick == 10) metrics.distance_at_two_s = distance;
        if (distance <= .20 && metrics.first_capture_s < 0.) metrics.first_capture_s = now;
        if (tick == 100) break;
        // Mission handoff begins on capture; no need to score backend replans
        // deep inside that region. Continue sampling the retained reference so
        // both arms still provide the same two-second comparison instant.
        if (metrics.first_capture_s >= 0.) continue;
        const auto previous = planner.committedTrajectory();
        const auto result = planner.replan(now, state, world, [now] { return now + .01; });
        metrics.worst_runtime_s = std::max(metrics.worst_runtime_s, result.replan_runtime_s);
        if (!result.accepted) {
            if (result.attempted) {
                ++metrics.rejected;
                ++metrics.rejections[result.status];
            }
            requireTrue(!result.incumbent_prefix_unsafe, "rejection leaves unsafe incumbent");
            const auto& retained = planner.committedTrajectory();
            requireTrue(retained.pieceCount() == previous.pieceCount(), "rejection replaced authority");
            requireMatrixNear(retained.endState().position, previous.endState().position, 1e-12,
                              "rejection changed endpoint");
            requireTrue(TrajectorySafetyChecker().checkCommitted(
                retained, now, std::max(now, retained.endTime()), snapshot).safe,
                "rejected candidate leaves unsafe retained trajectory");
            continue;
        }
        ++metrics.accepted;
        requireTrue(result.certified_world_version.has_value(), "accepted without atomic certificate");
        requireTrue(result.prefix_safety && result.prefix_safety->safe &&
                    result.candidate_safety && result.candidate_safety->safe,
                    "accepted without full prefix/candidate checks");
        requireTrue(result.splice_diagnostics.position_error < config.continuity_tolerance &&
                    result.splice_diagnostics.velocity_error < config.continuity_tolerance &&
                    result.splice_diagnostics.acceleration_error < config.continuity_tolerance,
                    "C2 continuity weakened");
        const auto& committed = planner.committedTrajectory();
        requireTrue(TrajectorySafetyChecker().checkCommitted(
            committed, now, committed.endTime(), snapshot).safe,
            "independent complete assembly check rejected installed curve");
        const auto& candidate = *result.local_plan;
        for (int derivative = 1; derivative <= 3; ++derivative) {
            const Vec3 limit = derivative == 1 ? config.v_max :
                derivative == 2 ? config.a_max : Vec3(6.5, 6.5, 4.);
            const auto spline = derivativeSpline(*candidate.control_points, candidate.knots, 3, derivative);
            requireTrue((spline.control_points.cwiseAbs().rowwise() - limit.transpose()).maxCoeff() < 1e-6,
                        "candidate derivative exceeds unchanged limits");
        }
    }
    return metrics;
}
}  // namespace

int main() {
    try {
        for (const auto& fixture : finishFixtures()) {
            const auto baseline = runFinish(fixture, .20);
            const auto changed = runFinish(fixture, .50);
            std::cout << fixture.run << " baseline/new: capture_s="
                      << baseline.first_capture_s << '/' << changed.first_capture_s
                      << " distance_at_2s=" << baseline.distance_at_two_s << '/' << changed.distance_at_two_s
                      << " accepted=" << baseline.accepted << '/' << changed.accepted
                      << " worst_wall_runtime_s=" << baseline.worst_runtime_s << '/' << changed.worst_runtime_s
                      << std::endl;
            for (const auto& arm : {std::make_pair("baseline", baseline), std::make_pair("new", changed)})
                for (const auto& reason : arm.second.rejections)
                    std::cout << "  " << arm.first << " " << reason.first << '=' << reason.second << '\n';
            requireTrue(changed.accepted > 0 && baseline.accepted + baseline.rejected > 0,
                        "missing new acceptance or baseline attempts");
            requireTrue(changed.distance_at_two_s < baseline.distance_at_two_s,
                        "earlier close allocation did not improve matched-time reference progress");
            requireTrue(changed.first_capture_s >= 0. &&
                        (baseline.first_capture_s < 0. || changed.first_capture_s < baseline.first_capture_s),
                        "earlier close allocation did not improve native 20cm reference entry");
        }
        std::cout << "navigation finish timing native A/B passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
