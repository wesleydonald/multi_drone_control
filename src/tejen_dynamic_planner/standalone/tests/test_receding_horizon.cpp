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

void testStoppedPartialRetainsForwardSoftGoal() {
    const auto config = makeLocalConfig();
    LocalPlanner local(config);
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
    const auto expected = buildRefinementQP(result.search, result.knots, request.local_goal,
        config.refinement, result.effective_terminal_boundary);
    requireMatrixNear(result.refinement->qp.linear, expected.linear, 1e-10,
                      "ordinary stopped partial lost MADER's forward soft objective");
    const State terminal = evaluateCubicState(
        *result.control_points, result.knots, request.duration_s);
    requireTrue((terminal.position-selected_endpoint).norm() > .25,
                "ordinary stopped root partial still rewards remaining at its selected endpoint");
    requireMatrixNear(terminal.velocity, Vec3::Zero(), 1e-7,
                      "refined partial is not stopped");
    requireMatrixNear(terminal.acceleration, Vec3::Zero(), 1e-7,
                      "refined partial terminal acceleration is not stopped");
}

WorldSnapshot stalledChicaneWorld() {
    // Frozen NAV3 S50 geometry, not a dependency on ignored runtime logs.
    WorldSnapshot world;
    world.ego_half_extents = Vec3(.105, .105, .1);
    world.ego_tracking_error_half_extents = Vec3(.1, .1, .05);
    world.ego_body_rotation = Eigen::Quaterniond(
        .9999998111977018, .0001879449301379487,
        -.00031321094221603557, .0004941458992396946).normalized().toRotationMatrix();
    auto& geometry = world.ego_suspended_geometry;
    geometry.enabled = geometry.payload_attached = true;
    geometry.swing_envelope = SwingEnvelope::OuterOctagon;
    geometry.cable_length_m = .49;
    geometry.max_swing_angle_rad = 10. * std::acos(-1.) / 180.;
    geometry.magnet_center_below_cable_end_m = 0.;
    geometry.payload_half_extents = Vec3(.043, .043, .011);
    geometry.payload_center_from_magnet_center = Vec3(0., 0., -.036);
    world.workspace.enabled = true;
    world.workspace.lower = Vec3(-2.5, -2.5, 0.);
    world.workspace.upper = Vec3(2.5, 2.5, 1.6);
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
        std::acos(-1.) / 4., Vec3::UnitZ()).toRotationMatrix();
    const Vec3 centres[] = {
        Vec3(.44898899713700224, -2.015655663803669, .8),
        Vec3(-2.3692090543969435, .8025423877302764, .8),
        Vec3(2.119209054396943, -1.0525423877302766, .8),
        Vec3(-.6989889971370024, 1.7656556638036691, .8)};
    const double half_widths[] = {1.8927669529663682, 1.3927669529663687,
                                  1.3927669529663687, 1.8927669529663682};
    for (int i = 0; i < 4; ++i) {
        auto vertices = axisAlignedBoxVertices(Vec3::Zero(), Vec3(.05, half_widths[i], .8));
        vertices = (vertices * rotation.transpose()).eval();
        vertices.rowwise() += centres[i].transpose();
        world.physical_static_obstacles.push_back({"panel_" + std::to_string(i), vertices});
    }
    return world;
}

void testStationaryTimeoutRefinesTowardSoftGoal() {
    const auto world = stalledChicaneWorld();
    auto config = makeLocalConfig();
    config.octopus.max_runtime_s = 1e-9;  // deterministic natural timeout, no callback stop
    config.octopus.planning_radius_m = 2.;
    const auto bounds = assemblyCentreBounds(world.workspace, world.ego_half_extents,
        world.ego_suspended_geometry, world.ego_tracking_error_half_extents, world.ego_body_rotation);
    config.octopus.xyz_min = config.refinement.xyz_min = bounds.first;
    config.octopus.xyz_max = config.refinement.xyz_max = bounds.second;
    config.octopus.a_max = config.refinement.a_max = Vec3(1., 1., 1.5);
    LocalPlanRequest request;
    request.initial_state.position = Vec3(-1.28013386705375, -.838968705864613, 1.34999993469477);
    request.local_goal = Vec3(.236942, .464289, 1.35);
    request.duration_s = 7.55122711762503;
    request.obstacles = world.timeIndexedObstacles(4, 0., request.duration_s);

    for (auto mode : {TerminalMode::Stopped, TerminalMode::MovingRendezvous}) {
        request.terminal_boundary.mode = mode;
        request.terminal_boundary.position = request.local_goal;
        request.terminal_boundary.velocity = Vec3(.2, 0., 0.);
        // Include a near-hover prefix well outside the superseded 2um test.
        request.initial_state.velocity = mode == TerminalMode::Stopped
            ? Vec3::Zero().eval() : Vec3(1e-4, -1e-4, 0.);
        request.initial_state.acceleration = mode == TerminalMode::Stopped
            ? Vec3::Zero().eval() : Vec3(1e-4, 0., 0.);
        const auto result = LocalPlanner(config).plan(request);
        requireTrue(result.success && result.partial, "stationary timeout fixture failed");
        requireTrue(result.search.termination_reason == "RUNTIME_REACHED",
                    "fixture must exercise natural runtime expiry");
        const Vec3 raw_end = result.search.control_points->bottomRows(1).transpose();
        requireTrue(result.search.partial_fallback_selected_index == 2,
                    "timeout fixture must select the unextended root");
        requireTrue((raw_end - request.initial_state.position).norm() < .002,
                    "raw near-hover seed moved farther than expected");
        const State end = evaluateCubicState(*result.control_points, result.knots, request.duration_s);
        requireTrue((end.position - raw_end).norm() > .25,
                    "stationary timeout still rewards holding the current position");
        requireTrue((end.position - request.local_goal).norm() < (raw_end - request.local_goal).norm(),
                    "stationary timeout refinement did not make goal progress");
        requireTrue((end.position - request.local_goal).norm() > .1,
                    "obstructed nominal goal must remain soft, not pinned");
        requireTrue(result.effective_terminal_boundary.mode == TerminalMode::Stopped,
                    "timeout fallback must not retain a hard moving-rendezvous contract");
        requireMatrixNear(end.velocity, Vec3::Zero(), 1e-7, "timeout progress terminal velocity");
        requireMatrixNear(end.acceleration, Vec3::Zero(), 1e-7, "timeout progress terminal acceleration");
        TrajectoryPiece piece;
        piece.control_points = *result.control_points;
        piece.knots = result.knots;
        piece.valid_from = 0.; piece.valid_until = request.duration_s;
        requireTrue(TrajectorySafetyChecker().checkPiece(piece, 0., request.duration_s, world).safe,
                    "timeout progress must pass unchanged independent physical-space checking");
    }
}

void testFrozenSecondWallPrefixRepair() {
    auto world = stalledChicaneWorld();
    // Reconstructed stationary S50 request at 90.2s. Original QP/planes were
    // not logged; this reproduces the current-hull corner contradiction,
    // not an exact replay or evidence of whole-route completion.
    world.ego_body_rotation = Eigen::Quaterniond(
        .9999997551742072, .00036121372879344577,
        .00041405403763317144, .0004332844580647135).normalized().toRotationMatrix();
    auto config = makeLocalConfig();
    config.octopus.max_runtime_s = 1e-9;
    config.octopus.planning_radius_m = 2.;
    const auto bounds = assemblyCentreBounds(world.workspace, world.ego_half_extents,
        world.ego_suspended_geometry, world.ego_tracking_error_half_extents, world.ego_body_rotation);
    config.octopus.xyz_min = config.refinement.xyz_min = bounds.first;
    config.octopus.xyz_max = config.refinement.xyz_max = bounds.second;
    config.octopus.a_max = config.refinement.a_max = Vec3(1., 1., 1.5);
    LocalPlanRequest request;
    request.initial_state.position = Vec3(.388380788607035, .18582036847715, 1.34999999919687);
    request.local_goal = request.initial_state.position + 2. *
        (Vec3(1.85, 1.85, 1.35) - request.initial_state.position).normalized();
    request.terminal_boundary.position = request.local_goal;
    request.duration_s = 7.50811977143881;
    request.obstacles = world.timeIndexedObstacles(4, 0., request.duration_s);
    const auto result = LocalPlanner(config).plan(request);
    requireTrue(result.success, "frozen second-wall repair failed: " + result.status +
                "; " + (result.refinement ? result.refinement->message : "no refinement"));
    requireTrue(result.refinement &&
                result.refinement->message.find("prefix_corridor_repairs=") != std::string::npos,
                "second-wall fixture did not exercise the incompatible-plane repair");
    const auto initial = evaluateCubicState(*result.control_points, result.knots, 0.);
    requireMatrixNear(initial.position, request.initial_state.position, 1e-8, "repair start P");
    requireMatrixNear(initial.velocity, Vec3::Zero(), 1e-8, "repair start V");
    requireMatrixNear(initial.acceleration, Vec3::Zero(), 1e-8, "repair start A");
    const auto end = evaluateCubicState(*result.control_points, result.knots, request.duration_s);
    requireMatrixNear(end.velocity, Vec3::Zero(), 1e-8, "repair stopped tail V");
    requireMatrixNear(end.acceleration, Vec3::Zero(), 1e-8, "repair stopped tail A");
    TrajectoryPiece piece;
    piece.control_points = *result.control_points; piece.knots = result.knots;
    piece.valid_from = 0.; piece.valid_until = request.duration_s;
    requireTrue(TrajectorySafetyChecker().checkPiece(piece, 0., request.duration_s, world).safe,
                "repaired second-wall curve failed independent physical-space checking");
}

void testUnextendedMovingSeedUsesSoftGoal() {
    auto config = makeLocalConfig();
    config.octopus.max_runtime_s = 1e-9;
    LocalPlanRequest request;
    request.initial_state = {Vec3(0., 0., 1.5), Vec3(.2, 0., 0.), Vec3::Zero()};
    request.local_goal = Vec3(2., 0., 1.5);
    request.duration_s = 4.;
    request.terminal_boundary.position = request.local_goal;
    const auto result = LocalPlanner(config).plan(request);
    requireTrue(result.success && result.partial, "braking timeout fixture failed");
    requireTrue(result.search.partial_fallback_selected_index == 2,
                "moving fixture must select an unextended root seed");
    const Vec3 selected = result.search.control_points->bottomRows(1).transpose();
    requireTrue((selected - request.initial_state.position).norm() > .05,
                "braking fixture must contain genuine motion");
    requireMatrixNear(result.effective_terminal_boundary.position, request.local_goal, 1e-10,
                      "unextended moving seed still rewards stopping at its own endpoint");
    const State end = evaluateCubicState(*result.control_points, result.knots, request.duration_s);
    requireTrue((end.position - request.local_goal).norm() < (selected - request.local_goal).norm(),
                "moving root refinement did not make progress");
    requireMatrixNear(end.velocity, Vec3::Zero(), 1e-7, "moving seed terminal velocity");
    requireMatrixNear(end.acceleration, Vec3::Zero(), 1e-7, "moving seed terminal acceleration");
    const State initial = evaluateCubicState(*result.control_points, result.knots, 0.);
    requireMatrixNear(initial.velocity, request.initial_state.velocity, 1e-7,
                      "seed refinement erased measured splice velocity");
}

void testMovingSeedCancellationKeepsStoppedEndpoint() {
    auto config = makeLocalConfig();
    LocalPlanRequest request;
    request.initial_state = {Vec3(0., 0., 1.5), Vec3(.2, 0., 0.), Vec3::Zero()};
    request.local_goal = Vec3(2., 0., 1.5);
    request.duration_s = 4.;
    request.terminal_boundary.mode = TerminalMode::MovingRendezvous;
    request.terminal_boundary.position = request.local_goal;
    request.octopus_stop_requested = [] { return true; };
    const auto result = LocalPlanner(config).plan(request);
    requireTrue(result.success && result.partial, "cancelled moving seed fixture failed");
    const Vec3 selected = result.search.control_points->bottomRows(1).transpose();
    requireMatrixNear(result.effective_terminal_boundary.position, selected, 1e-10,
                      "explicit cancellation must not request new goal progress");
}

void testSearchedPartialUsesOriginalRequestedMode() {
    auto config = makeLocalConfig();
    for (auto mode : {TerminalMode::Stopped, TerminalMode::Continuation,
                      TerminalMode::MovingRendezvous}) {
        LocalPlanRequest request;
        request.initial_state.position = Vec3(0., 0., 1.5);
        request.local_goal = Vec3(2., 0., 1.5);
        request.duration_s = 4.;
        request.terminal_boundary.mode = mode;
        request.terminal_boundary.position = request.local_goal;
        int polls = 0;
        request.octopus_stop_requested = [&polls] { return ++polls > 2; };
        const auto result = LocalPlanner(config).plan(request);
        requireTrue(result.success && result.partial, "searched partial fixture failed: " + result.status);
        requireTrue(result.search.partial_fallback_selected_index > 2,
                    "fixture must select a real searched extension, not the root seed");
        const Vec3 selected = result.search.control_points->bottomRows(1).transpose();
        requireMatrixNear(result.effective_terminal_boundary.position, selected, 1e-10,
                          "searched partial boundary metadata changed");
        const Vec3 expected_goal = mode == TerminalMode::Stopped ? request.local_goal : selected;
        const auto expected = buildRefinementQP(result.search, result.knots, expected_goal,
            config.refinement, result.effective_terminal_boundary);
        requireMatrixNear(result.refinement->qp.linear, expected.linear, 1e-10,
                          "searched partial objective disagrees with its original requested mode");
    }
}

void testFailedSeedGoalRefinementRetainsRefinedStop() {
    auto config = makeLocalConfig();
    config.octopus.max_runtime_s = 1e-9;
    // Pinned qpOASES fixture: this work-set limit cannot refine the far soft
    // goal but can refine the already-certified stopped endpoint. Production
    // limits are unchanged; this exercises the failure path, not a new budget.
    config.refinement.max_working_set_recalculations = 31;
    LocalPlanRequest request;
    request.initial_state.position = Vec3(0., 0., 1.5);
    request.local_goal = Vec3(10., 1., 1.5);
    request.duration_s = 4.;
    const auto result = LocalPlanner(config).plan(request);
    requireTrue(result.success && result.partial, "certified endpoint retry was lost");
    requireTrue(result.message.find("unextended_seed_endpoint_retry=SOLVER_FAILED") != std::string::npos,
                "fixture did not exercise failed goal refinement and bounded endpoint retry");
    requireTrue(result.refinement && result.refinement->success &&
                result.refinement->separator_recheck_passed,
                "endpoint retry bypassed refinement/postchecks");
    const State end = evaluateCubicState(*result.control_points, result.knots, request.duration_s);
    requireMatrixNear(end.position, request.initial_state.position, 1e-7, "endpoint retry stop position");
    requireMatrixNear(end.velocity, Vec3::Zero(), 1e-7, "endpoint retry velocity");
    requireMatrixNear(end.acceleration, Vec3::Zero(), 1e-7, "endpoint retry acceleration");
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

void testStationaryNavigationNearbyGoalRemainsSoft() {
    const State start{Vec3(0., 0., 1.35), Vec3::Zero(), Vec3::Zero()};
    const Vec3 goal(.85, 0., 1.35);
    RecedingHorizonConfig config;
    config.fixed_splice_lookahead_s = .1;
    config.moving_rendezvous_enabled = false;
    config.terminal_time_target_prediction_enabled = true;
    config.require_terminal_hold_for_nominal = false;
    auto local = makeLocalConfig();
    WorldSnapshot snapshot;
    snapshot.physical_static_obstacles.push_back({"obstructed_nearby_goal",
        axisAlignedBoxVertices(goal, Vec3(.15, .15, .15))});
    VersionedWorld world(snapshot);
    CommittedTrajectoryTargetPredictor predictor(
        makeStationaryHoverTrajectory(goal, 0., 100.), Vec3::Zero());
    RecedingHorizonPlanner planner(goal, config, local);
    planner.initializeCommittedTrajectory(makeStationaryHoverTrajectory(start.position, 0., .1));
    const auto result = planner.replan(0., start, world, [] { return .01; }, &predictor);
    requireTrue(result.local_plan.has_value(), "nearby stationary goal never reached local planning");
    requireTrue(result.global_distance_from_splice_m < config.planning_radius_m,
                "fixture did not exercise the nearby-goal region");
    requireTrue(result.local_plan->requested_terminal_boundary.mode == TerminalMode::Stopped &&
                !result.moving_rendezvous_active && !result.target_time_prediction_used,
                "stationary navigation still requested a hard nearby rendezvous");
    requireTrue(result.accepted, "stationary soft goal failed: " + result.status);
    const auto endpoint = planner.committedTrajectory().endState();
    requireTrue((endpoint.position-goal).norm() > .15,
                "soft stationary endpoint entered its obstructed goal");
    requireMatrixNear(endpoint.velocity, Vec3::Zero(), 1e-7, "stationary endpoint velocity");
    requireMatrixNear(endpoint.acceleration, Vec3::Zero(), 1e-7, "stationary endpoint acceleration");
    requireTrue(TrajectorySafetyChecker().checkCommitted(planner.committedTrajectory(),
        planner.committedTrajectory().startTime(), planner.committedTrajectory().endTime(), snapshot).safe,
        "stationary soft endpoint bypassed independent collision checking");
}

}  // namespace

int main() {
    try {
        testStoppedPartialRetainsForwardSoftGoal();
        testStationaryTimeoutRefinesTowardSoftGoal();
        testFrozenSecondWallPrefixRepair();
        testUnextendedMovingSeedUsesSoftGoal();
        testMovingSeedCancellationKeepsStoppedEndpoint();
        testSearchedPartialUsesOriginalRequestedMode();
        testFailedSeedGoalRefinementRetainsRefinedStop();
        testObstructedIntermediateGoalRemainsSoft();
        testStationaryNavigationNearbyGoalRemainsSoft();
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
