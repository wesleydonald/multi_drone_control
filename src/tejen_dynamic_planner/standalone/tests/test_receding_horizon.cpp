#include "dynamic_planner/receding_horizon_planner.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace dynamic_planner;

namespace {

LocalPlannerConfig makeLocalConfig() {
    LocalPlannerConfig config;
    config.num_segments = 4;
    config.octopus.samples_per_axis = {7, 7, 7};
    config.octopus.alpha_shrink = 0.9;
    config.octopus.voxel_fraction = 0.10;
    config.octopus.heuristic_bias = 1.0;
    config.octopus.max_runtime_s = 2.0;
    config.octopus.xyz_min = Vec3(-1.0, -2.0, 0.4);
    config.octopus.xyz_max = Vec3(4.0, 2.0, 2.6);
    config.octopus.random_seed = 1;
    config.refinement.xyz_min = config.octopus.xyz_min;
    config.refinement.xyz_max = config.octopus.xyz_max;
    config.refinement.j_max = Vec3::Constant(4.0);
    config.refinement.max_working_set_recalculations = 500;
    return config;
}

void testStoppedPartialRetainsSelectedEndpoint() {
    LocalPlanner local(makeLocalConfig());
    LocalPlanRequest request;
    request.initial_state = {Vec3(0.0, 0.0, 1.5), Vec3::Zero(), Vec3::Zero()};
    request.local_goal = Vec3(2.0, 0.0, 1.5);
    request.duration_s = 4.0;
    request.terminal_boundary.position = request.local_goal;
    // Deterministic cutoff: only the existing stationary partial is available.
    request.octopus_stop_requested = []() { return true; };
    request.octopus_stop_reason = "TEST_CUTOFF";
    const auto result = local.plan(request);
    requireTrue(result.success && result.partial,
                "stopped partial fixture failed: " + result.status);
    const Vec3 selected_endpoint = result.search.control_points->bottomRows(1).transpose();
    requireMatrixNear(result.effective_terminal_boundary.position, selected_endpoint, 1e-10,
                      "stopped partial still refines toward the distant nominal goal");
    const State terminal = evaluateCubicState(
        *result.control_points, result.knots, request.duration_s);
    requireMatrixNear(terminal.position, selected_endpoint, 1e-7,
                      "refinement moved the selected stationary evasive endpoint");
    requireMatrixNear(terminal.velocity, Vec3::Zero(), 1e-7,
                      "refined partial is not stopped");
}

void testObstructedIntermediateGoalRemainsSoft() {
    // N1's diagonal projected waypoint lies inside a peer's cable-vs-body
    // C-space even though the two vehicle bodies alone are vertically clear.
    const State start{Vec3(-1.85, -1.85, 1.35), Vec3::Zero(), Vec3::Zero()};
    const Vec3 goal(1.85, 1.85, 1.35);
    RecedingHorizonConfig config;
    config.fixed_splice_lookahead_s = 0.10;
    config.moving_rendezvous_enabled = true;
    config.terminal_time_target_prediction_enabled = true;
    config.target_prediction_high_confidence_horizon_s = 30.0;
    config.spline_time_factor = 3.0;
    LocalPlannerConfig local = makeLocalConfig();
    local.octopus.xyz_min = Vec3(-2.5, -2.5, 0.4);
    local.octopus.xyz_max = Vec3(2.5, 2.5, 3.5);
    local.refinement.xyz_min = local.octopus.xyz_min;
    local.refinement.xyz_max = local.octopus.xyz_max;

    WorldSnapshot snapshot;
    snapshot.ego_half_extents = Vec3(0.105, 0.105, 0.10);
    snapshot.ego_tracking_error_half_extents = Vec3(0.10, 0.10, 0.05);
    snapshot.ego_suspended_geometry.enabled = true;
    snapshot.ego_suspended_geometry.cable_length_m = 0.49;
    snapshot.ego_suspended_geometry.max_swing_angle_rad = 15.0 * std::acos(-1.0) / 180.0;
    snapshot.ego_suspended_geometry.magnet_center_below_cable_end_m = 0.0;
    snapshot.ego_suspended_geometry.payload_attached = true;
    snapshot.ego_suspended_geometry.payload_half_extents = Vec3(0.043, 0.043, 0.011);
    snapshot.ego_suspended_geometry.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.036);
    CooperativeObstacleTrajectory peer;
    peer.name = "drone_2";
    peer.physical_half_extents = snapshot.ego_half_extents;
    peer.trajectory = makeStationaryHoverTrajectory(
        Vec3(-0.530330, -0.530330, 1.0), 0.0, 100.0);
    snapshot.cooperative_obstacles.push_back(peer);
    peer.name = "drone_3";
    peer.trajectory = makeStationaryHoverTrajectory(
        Vec3(0.530330, 0.530330, 1.0), 0.0, 100.0);
    snapshot.cooperative_obstacles.push_back(peer);
    VersionedWorld world(snapshot);
    CommittedTrajectoryTargetPredictor predictor(
        makeStationaryHoverTrajectory(goal, 0.0, 100.0), Vec3::Zero());
    RecedingHorizonPlanner planner(goal, config, local);
    planner.initializeCommittedTrajectory(makeStationaryHoverTrajectory(start.position, 0.0, 0.10));
    const auto result = planner.replan(0.0, start, world, []() { return 0.01; }, &predictor);
    requireTrue(result.local_plan.has_value(), "blocked-goal fixture never reached local search");
    requireTrue(result.local_plan->requested_terminal_boundary.mode == TerminalMode::Stopped,
                "obstructed intermediate goal was still requested as a hard endpoint");
    const TrajectoryPiece projected_hold = makeStationaryHoverTrajectory(
        result.local_goal, 0.0, 1.0).pieces().front();
    TrajectorySafetyChecker checker;
    const auto blocked = checker.checkPiece(projected_hold, 0.0, 1.0, snapshot);
    requireTrue(!blocked.safe, "projected waypoint unexpectedly clears the suspended peer geometry");
    requireTrue(result.accepted && result.candidate_kind == CandidateKind::Complete,
                "blocked soft waypoint did not yield a complete accepted plan: " + result.status);
    const State terminal = planner.committedTrajectory().endState();
    requireTrue((terminal.position - result.local_goal).norm() > 0.10,
                "accepted endpoint was still pinned to the obstructed waypoint");
    requireTrue((terminal.position - start.position).norm() > 0.25,
                "soft waypoint produced only a stationary hold instead of progress");
    const Vec3 direction = (goal - start.position).normalized();
    const Vec3 displacement = terminal.position - start.position;
    requireTrue((displacement - displacement.dot(direction) * direction).norm() > 0.01,
                "blocked waypoint did not permit an off-axis endpoint");
    requireMatrixNear(terminal.velocity, Vec3::Zero(), 1e-7, "soft endpoint velocity is nonzero");
    requireMatrixNear(terminal.acceleration, Vec3::Zero(), 1e-7, "soft endpoint acceleration is nonzero");
    requireTrue(!result.goal_seen && !result.local_continuation_active,
                "intermediate soft plan reports completion or hard continuation");
    requireNear(result.splice_time_s, 0.10, 1e-12, "soft goal changed the 100 ms splice lookahead");
    requireTrue(checker.checkCommitted(planner.committedTrajectory(),
                                     planner.committedTrajectory().startTime(),
                                     planner.committedTrajectory().endTime(), snapshot).safe,
                "accepted off-axis trajectory failed independent whole-assembly checking");
    std::cout << "obstructed soft goal / accepted endpoint: " << result.local_goal.transpose()
              << " / " << terminal.position.transpose() << '\n';
}

}  // namespace

int main() {
    try {
        testStoppedPartialRetainsSelectedEndpoint();
        testObstructedIntermediateGoalRemainsSoft();
        State start;
        start.position = Vec3(0.0, 0.0, 1.5);
        start.velocity = Vec3::Zero();
        start.acceleration = Vec3::Zero();
        const Vec3 goal(3.0, 0.0, 1.5);

        RecedingHorizonConfig config;
        config.dc_s = 1.0 / 30.0;
        config.planning_radius_m = 1.0;
        config.factor_alpha = 2.5;
        config.min_splice_lookahead_s = 0.05;
        config.max_splice_lookahead_s = 1.0;
        config.factor_alloc = 1.0;
        config.factor_alloc_close = 2.5;
        config.spline_time_factor = 2.5;
        config.close_to_goal_m = 0.20;
        config.goal_tolerance_m = 0.05;
        config.continuity_tolerance = 1e-7;
        config.v_max = Vec3::Ones();
        config.a_max = Vec3::Constant(1.5);

        RecedingHorizonPlanner planner(goal, config, makeLocalConfig());
        FrozenWorld world;

        auto initial = planner.replan(0.0, start, world);
        requireTrue(initial.accepted, "initial local plan was not committed: " + initial.status);
        requireNear(initial.time_allocation_factor, 2.5, 1e-12,
                    "spline time-allocation floor was not applied");
        requireNear(initial.local_duration_s,
                    initial.minimum_time_s * initial.time_allocation_factor,
                    1e-10,
                    "local duration no longer matches reported minimum/factor");
        requireTrue(planner.committedTrajectory().pieceCount() == 1,
                    "initial commit should contain exactly one piece");

        int accepted_replans = 1;
        double max_pos_splice = initial.splice_diagnostics.position_error;
        double max_vel_splice = initial.splice_diagnostics.velocity_error;
        double max_acc_splice = initial.splice_diagnostics.acceleration_error;
        double now = config.dc_s;
        bool reached = false;
        int guard = 0;
        while (guard++ < 1800) {
            const State state = planner.committedTrajectory().evaluate(now);
            if (planner.goalReached(state)) {
                reached = true;
                break;
            }
            if (!planner.goalSeen()) {
                const auto result = planner.replan(now, state, world);
                if (result.accepted) {
                    ++accepted_replans;
                    max_pos_splice = std::max(max_pos_splice, result.splice_diagnostics.position_error);
                    max_vel_splice = std::max(max_vel_splice, result.splice_diagnostics.velocity_error);
                    max_acc_splice = std::max(max_acc_splice, result.splice_diagnostics.acceleration_error);
                    requireTrue(result.local_plan.has_value(), "accepted replan missing local plan");
                    requireTrue(result.local_plan->control_points->rows() == 7,
                                "local planner no longer has seven cubic control points");
                }
            }
            // Runtime-aware execution model: the vehicle keeps following the
            // committed spline or its certified stationary terminal hold while
            // the local planner computes.
            const double step = std::max(config.dc_s, planner.previousReplanRuntime());
            now += step;
        }

        requireTrue(reached, "3 m receding-horizon mission did not reach goal");
        requireTrue(accepted_replans >= 3, "mission should require multiple accepted local plans");
        requireTrue(planner.goalSeen(), "GOAL_SEEN was never latched");
        requireTrue(max_pos_splice < config.continuity_tolerance, "position splice continuity failed");
        requireTrue(max_vel_splice < config.continuity_tolerance, "velocity splice continuity failed");
        requireTrue(max_acc_splice < config.continuity_tolerance, "acceleration splice continuity failed");
        requireTrue((planner.committedTrajectory().endState().position - goal).norm() <= 0.05 + 1e-9,
                    "committed endpoint is not within global goal tolerance");

        // Explicit terminal-hold restart regression. Deliberately consume the
        // first local spline without replacing it, advance into its certified
        // stationary hold, then require replan() itself to choose a future A in
        // that hold and commit a continuation. This directly covers the B.1 v1
        // deadlock where A was clamped to the finite endpoint.
        RecedingHorizonPlanner restart_planner(goal, config, makeLocalConfig());
        const auto restart_initial = restart_planner.replan(0.0, start, world);
        requireTrue(restart_initial.accepted, "restart bootstrap plan failed");
        const double first_end = restart_planner.committedTrajectory().endTime();
        requireTrue(restart_planner.committedTrajectory().endsInStoppedHold(),
                    "bootstrap plan does not expose certified terminal hold");
        const double stopped_now = first_end + 0.02;
        const State stopped_state = restart_planner.committedTrajectory().evaluate(stopped_now);
        requireMatrixNear(stopped_state.velocity, Vec3::Zero(), 1e-12,
                          "post-end committed state is not stationary");
        requireMatrixNear(stopped_state.acceleration, Vec3::Zero(), 1e-12,
                          "post-end committed acceleration is not zero");
        const auto restarted = restart_planner.replan(stopped_now, stopped_state, world);
        requireTrue(restarted.attempted, "terminal-hold restart did not attempt planning");
        requireTrue(restarted.splice_time_s > stopped_now + 1e-6,
                    "terminal-hold restart did not choose a genuinely future A");
        requireTrue(!restarted.candidate_late,
                    "terminal-hold restart was incorrectly classified as stale");
        requireTrue(restarted.accepted,
                    "terminal-hold restart candidate was not committed: " + restarted.status);
        requireTrue(restart_planner.committedTrajectory().pieceCount() >= 3,
                    "terminal-hold restart did not preserve/materialize the executed hold prefix");

        // Deterministic late-candidate regression. The first bootstrap plan has
        // no future splice deadline. Thereafter a 1 us lookahead is far shorter
        // than an Octopus+QP solve, so the successful-but-stale candidate must
        // be rejected before mutating the committed trajectory.
        RecedingHorizonConfig late_config = config;
        late_config.min_splice_lookahead_s = 1e-6;
        late_config.max_splice_lookahead_s = 1e-6;
        RecedingHorizonPlanner late_planner(goal, late_config, makeLocalConfig());
        const auto late_initial = late_planner.replan(0.0, start, world);
        requireTrue(late_initial.accepted, "late-gate bootstrap plan failed");
        const std::size_t pieces_before = late_planner.committedTrajectory().pieceCount();
        const double end_before = late_planner.committedTrajectory().endTime();
        const State end_state_before = late_planner.committedTrajectory().endState();
        const double late_now = late_config.dc_s;
        const State late_state = late_planner.committedTrajectory().evaluate(late_now);
        const auto late_result = late_planner.replan(late_now, late_state, world);
        requireTrue(!late_result.accepted, "late candidate was incorrectly committed");
        requireTrue(late_result.candidate_late,
                    "late candidate rejection did not set candidate_late");
        requireTrue(late_result.status == "LATE_CANDIDATE_REJECTED",
                    "unexpected late-candidate status: " + late_result.status);
        requireTrue(late_planner.committedTrajectory().pieceCount() == pieces_before,
                    "late rejection mutated committed piece count");
        requireNear(late_planner.committedTrajectory().endTime(), end_before, 1e-12,
                    "late rejection mutated committed end time");
        requireMatrixNear(late_planner.committedTrajectory().endState().position,
                          end_state_before.position, 1e-12,
                          "late rejection mutated committed endpoint");

        std::cout << "accepted replans: " << accepted_replans << "\n";
        std::cout << "max C2 splice errors: " << max_pos_splice << ", "
                  << max_vel_splice << ", " << max_acc_splice << "\n";
        std::cout << "test_receding_horizon: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_receding_horizon: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
