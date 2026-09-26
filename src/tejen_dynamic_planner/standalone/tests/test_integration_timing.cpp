#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/reference_window.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "test_common.hpp"

namespace {

using dynamic_planner::LocalPlannerConfig;
using dynamic_planner::RecedingHorizonConfig;
using dynamic_planner::RecedingHorizonPlanner;
using dynamic_planner::State;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::Vec3;
using dynamic_planner::VersionedWorld;
using dynamic_planner::WorldSnapshot;

SuspendedGeometry suspendedGeometry() {
    SuspendedGeometry g;
    g.enabled = true;
    g.cable_length_m = 0.50;
    g.cable_radius_m = 0.0025;
    g.max_swing_angle_rad = 10.0 * 3.14159265358979323846 / 180.0;
    g.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    g.magnet_center_below_cable_end_m = 0.025;
    return g;
}

RecedingHorizonConfig config(double initial_splice_timing_s) {
    RecedingHorizonConfig c;
    c.dc_s = 1.0 / 30.0;
    c.planning_radius_m = 2.0;
    c.factor_alpha = 2.5;
    c.min_splice_lookahead_s = 0.05;
    c.max_splice_lookahead_s = 1.0;
    c.factor_alloc = 1.0;
    c.factor_alloc_close = 2.5;
    c.spline_time_factor = 2.5;
    c.close_to_goal_m = 0.20;
    c.goal_tolerance_m = 0.05;
    c.continuity_tolerance = 1e-7;
    c.separator_validation_tolerance = 1e-7;
    c.initial_splice_timing_s = initial_splice_timing_s;
    c.v_max = Vec3::Ones();
    c.a_max = Vec3(1.0, 1.0, 1.5);
    return c;
}

LocalPlannerConfig localConfig() {
    LocalPlannerConfig c;
    c.num_segments = 4;
    c.octopus.samples_per_axis = {9, 9, 9};
    c.octopus.alpha_shrink = 0.9;
    c.octopus.voxel_fraction = 0.10;
    c.octopus.heuristic_bias = 1.0;
    c.octopus.max_runtime_s = 2.0;
    c.octopus.xyz_min = Vec3(-1.0, -1.0, 0.4);
    c.octopus.xyz_max = Vec3(4.0, 1.0, 2.6);
    c.octopus.random_seed = 1;
    c.refinement.xyz_min = c.octopus.xyz_min;
    c.refinement.xyz_max = c.octopus.xyz_max;
    c.refinement.j_max = Vec3::Constant(4.0);
    c.refinement.max_working_set_recalculations = 500;
    return c;
}

VersionedWorld emptySuspendedWorld() {
    WorldSnapshot w;
    w.captured_at_s = 0.0;
    w.ego_half_extents = Vec3(0.105, 0.105, 0.060);
    w.ego_suspended_geometry = suspendedGeometry();
    return VersionedWorld(w);
}

}  // namespace

int main() {
    try {
        const State hover_state{Vec3(0.0, 0.0, 1.5), Vec3::Zero(), Vec3::Zero()};
        const Vec3 goal(1.0, 0.0, 1.5);

        // With no trajectory-time history the default future splice is only ~83 ms.
        // A ROS trajectory clock saying 150 ms of simulation time has elapsed
        // must reject the candidate and leave the stationary incumbent intact.
        {
            RecedingHorizonPlanner planner(goal, config(0.0), localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(hover_state.position, 0.0, 0.10));
            auto world = emptySuspendedWorld();
            const auto result = planner.replan(
                0.0, hover_state, world, []() { return 0.150; });
            requireTrue(result.attempted, "delayed timing test did not attempt planning");
            requireTrue(!result.accepted, "late first candidate unexpectedly gained authority");
            requireTrue(result.candidate_late, "late first candidate was not labelled late");
            requireTrue(result.status == "LATE_CANDIDATE_REJECTED",
                        "late first candidate returned unexpected status");
            const auto still_hovering = planner.committedTrajectory().evaluate(0.150);
            requireMatrixNear(still_hovering.position, hover_state.position, 1e-12,
                              "late candidate changed hover position");
            requireMatrixNear(still_hovering.velocity, Vec3::Zero(), 1e-12,
                              "late candidate changed hover velocity");
        }

        // Carrying the observed 150 ms trajectory-time elapsed into a fresh shadow solve must
        // enlarge the future splice while still starting from a brand-new hover
        // incumbent. This is how C.1b avoids a fictional executed incumbent.
        {
            RecedingHorizonPlanner planner(goal, config(0.150), localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(hover_state.position, 10.0, 0.10));
            auto world = emptySuspendedWorld();
            const auto result = planner.replan(
                10.0, hover_state, world, []() { return 10.150; });
            requireTrue(result.attempted, "trajectory-time-seeded timing test did not attempt planning");
            requireTrue(result.splice_time_s > 10.150,
                        "trajectory-time-seeded splice is not after simulated solve completion");
            requireTrue(result.accepted,
                        std::string("trajectory-time-seeded future candidate was not accepted: ") + result.status);
            requireTrue(result.splice_diagnostics.position_error <= 1e-7 &&
                        result.splice_diagnostics.velocity_error <= 1e-7 &&
                        result.splice_diagnostics.acceleration_error <= 1e-7,
                        "trajectory-time-seeded accepted splice is not C2");
            const auto before = planner.committedTrajectory().evaluate(10.150);
            requireMatrixNear(before.position, hover_state.position, 1e-10,
                              "vehicle should remain on hover incumbent before future splice");
        }


        // C1E-only useful-deadline policy uses the trajectory clock, not Octopus's
        // steady-clock max_runtime_s. With an 83 ms splice and one 33 ms reserve,
        // a trajectory clock already at 150 ms must stop queue exploration
        // immediately while preserving the incumbent and the normal fallback path.
        {
            RecedingHorizonConfig deadline_config = config(0.0);
            deadline_config.enable_octopus_useful_deadline = true;
            // C1F.5p2 permits a real post-search certification reserve larger
            // than the 50 ms minimum splice lookahead. 200 ms is the initial
            // commissioned value measured from C1F.5p final checking costs.
            deadline_config.octopus_post_search_reserve_s = 0.200;
            RecedingHorizonPlanner planner(goal, deadline_config, localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(hover_state.position, 0.0, 0.10));
            auto world = emptySuspendedWorld();
            const auto result = planner.replan(
                0.0, hover_state, world, []() { return 0.150; });
            requireTrue(result.attempted, "useful-deadline test did not attempt planning");
            requireTrue(result.octopus_useful_deadline_enabled,
                        "useful-deadline policy was not enabled in replan result");
            requireNear(result.octopus_post_search_reserve_s, 0.200, 1e-12,
                        "useful-deadline reserve changed unexpectedly");
            requireNear(result.octopus_useful_budget_s, 0.0, 1e-12,
                        "useful-deadline budget did not reserve certification time");
            requireTrue(result.local_plan.has_value(),
                        "useful-deadline test did not expose local-plan diagnostics");
            requireTrue(result.local_plan->search.termination_reason == "USEFUL_DEADLINE_REACHED",
                        "Octopus did not stop for the trajectory-time useful deadline");
            requireTrue(!result.accepted,
                        "candidate computed after useful deadline unexpectedly gained authority");
            const auto still_hovering = planner.committedTrajectory().evaluate(0.150);
            requireMatrixNear(still_hovering.position, hover_state.position, 1e-12,
                              "useful-deadline stop changed incumbent hover position");
        }

        // C1F.8c: fast profiles decouple the future splice from previous solve
        // runtime and evaluate a moving terminal state at the candidate's own
        // end time. The provider time, terminal p/v and candidate end time must
        // therefore describe one physical rendezvous instant.
        {
            RecedingHorizonConfig moving_config = config(0.0);
            moving_config.fixed_splice_lookahead_s = 0.10;
            moving_config.enable_octopus_useful_deadline = false;
            moving_config.moving_rendezvous_enabled = true;
            moving_config.terminal_time_target_prediction_enabled = true;
            moving_config.target_time_fixed_point_iterations = 6;
            moving_config.target_time_fixed_point_tolerance_s = 0.01;
            moving_config.target_prediction_high_confidence_horizon_s = 10.0;
            RecedingHorizonPlanner planner(
                Vec3(1.2, 0.0, 1.5), moving_config, localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(
                    hover_state.position, 20.0, 0.10));
            auto world = emptySuspendedWorld();
            dynamic_planner::ScriptedCircleTargetPredictor predictor(
                Vec3(1.0, 0.0, 1.5), 0.2, 0.20,
                Vec3(1.2, 0.0, 1.5), 20.0);
            const auto result = planner.replan(
                20.0, hover_state, world, []() { return 20.01; }, &predictor);
            requireTrue(result.attempted,
                        "terminal-time target test did not attempt planning");
            requireNear(result.splice_lookahead_s, 0.10, 1e-12,
                        "fixed C1F.8c splice lookahead changed unexpectedly");
            requireTrue(!result.octopus_useful_deadline_enabled,
                        "C1F.8c unexpectedly re-enabled trajectory-time useful deadline");
            requireTrue(result.target_time_prediction_used &&
                        result.target_time_prediction_valid,
                        "terminal-time target prediction was not used");
            requireTrue(result.target_time_prediction_high_confidence,
                        "scripted target was not treated as high-confidence future");
            requireTrue(result.moving_target_terminal_active,
                        "high-confidence target was not used as a moving terminal state");
            requireNear(
                result.predicted_target_time_s,
                result.splice_time_s + result.local_duration_s,
                1e-9,
                "predicted target time is not the actual candidate terminal time");
            const auto expected = predictor.evaluate(result.predicted_target_time_s);
            requireTrue(expected.valid, "scripted target prediction unexpectedly invalid");
            requireMatrixNear(result.predicted_target_position, expected.position, 1e-10,
                              "terminal target position was evaluated at the wrong time");
            requireMatrixNear(result.predicted_target_velocity, expected.velocity, 1e-10,
                              "terminal target velocity was evaluated at the wrong time");
            if (result.moving_rendezvous_active) {
                requireMatrixNear(result.local_goal, expected.position, 1e-10,
                                  "moving-rendezvous goal is not the terminal target position");
                requireMatrixNear(result.terminal_target_velocity, expected.velocity, 1e-10,
                                  "moving-rendezvous velocity is not the terminal target velocity");
            }
        }

        // C1F.8c: a high-confidence target outside the local planning radius is
        // an intermediate continuation, not a physical rendezvous. The local
        // planner hard-constrains its position but must not hard-match the
        // platform's tangential velocity at an artificial point in free space.
        {
            RecedingHorizonConfig continuation_config = config(0.0);
            continuation_config.fixed_splice_lookahead_s = 0.10;
            continuation_config.enable_octopus_useful_deadline = false;
            continuation_config.moving_rendezvous_enabled = true;
            continuation_config.terminal_time_target_prediction_enabled = true;
            continuation_config.target_time_fixed_point_iterations = 6;
            continuation_config.target_time_fixed_point_tolerance_s = 0.01;
            continuation_config.target_prediction_high_confidence_horizon_s = 10.0;
            RecedingHorizonPlanner planner(
                Vec3(3.2, 0.0, 1.5), continuation_config, localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(
                    hover_state.position, 25.0, 0.50));
            auto world = emptySuspendedWorld();
            dynamic_planner::ScriptedCircleTargetPredictor predictor(
                Vec3(3.0, 0.0, 1.5), 0.2, 0.20,
                Vec3(3.2, 0.0, 1.5), 25.0);
            const auto result = planner.replan(
                25.0, hover_state, world, []() { return 25.01; }, &predictor);
            requireTrue(result.attempted,
                        "continuation target test did not attempt planning");
            requireTrue(result.target_time_prediction_used && result.target_time_prediction_valid,
                        "continuation target prediction was not evaluated");
            requireTrue(result.local_plan.has_value(),
                        "far high-confidence target did not reach the local-planner contract");
            requireTrue(
                result.local_plan->requested_terminal_boundary.mode ==
                    dynamic_planner::TerminalMode::Continuation,
                "far high-confidence target did not request continuation semantics");
            requireTrue(
                result.local_plan->requested_terminal_boundary.mode !=
                    dynamic_planner::TerminalMode::MovingRendezvous,
                "far local waypoint was incorrectly requested as true rendezvous");
            // `local_continuation_active` describes the surviving complete candidate.
            // A later safe padded partial intentionally clears it and switches the
            // effective terminal boundary to STOPPED, so it is not a valid proxy for
            // the mode originally requested from LocalPlanner.
            requireTrue(result.global_distance_from_splice_m > continuation_config.planning_radius_m,
                        "continuation fixture unexpectedly placed target inside local radius");
            requireNear((result.local_goal - result.splice_state.position).norm(),
                        continuation_config.planning_radius_m, 1e-8,
                        "continuation waypoint was not projected to local planning radius");
            requireMatrixNear(result.terminal_target_velocity, Vec3::Zero(), 1e-12,
                              "continuation waypoint retained a fake hard terminal velocity");
        }

        // M2D regression: a continuation candidate can put its nominal/local
        // endpoint inside the mission goal tolerance while still carrying a
        // brake-to-hover safety tail. That candidate must remain replannable;
        // latching goal_seen here would execute the safety tail as the nominal
        // mission trajectory and can stop well past the capture target.
        {
            RecedingHorizonConfig continuation_goal_config = config(0.0);
            continuation_goal_config.fixed_splice_lookahead_s = 0.10;
            continuation_goal_config.enable_octopus_useful_deadline = false;
            continuation_goal_config.planning_radius_m = 1.0;
            continuation_goal_config.moving_rendezvous_enabled = true;
            continuation_goal_config.terminal_time_target_prediction_enabled = true;
            continuation_goal_config.target_time_fixed_point_iterations = 6;
            continuation_goal_config.target_time_fixed_point_tolerance_s = 0.01;
            continuation_goal_config.target_prediction_high_confidence_horizon_s = 10.0;
            // Test-only: make the old latch-distance predicate deterministic while
            // keeping the measured start (1.0 m from goal) outside goalReached().
            // The production guard must reject goal_seen for continuation candidates
            // regardless of the particular tolerance value that made the old branch true.
            continuation_goal_config.goal_tolerance_m = 0.50;
            continuation_goal_config.close_to_goal_m = 0.50;

            const Vec3 continuation_goal(1.0, 0.0, 1.5);
            RecedingHorizonPlanner planner(
                continuation_goal, continuation_goal_config, localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(
                    hover_state.position, 27.0, 0.50));
            auto world = emptySuspendedWorld();
            dynamic_planner::ScriptedCircleTargetPredictor predictor(
                Vec3(1.0, 0.0, 1.5), 0.2, 0.0,
                Vec3(1.2, 0.0, 1.5), 27.0);
            const auto result = planner.replan(
                27.0, hover_state, world, []() { return 27.01; }, &predictor);

            requireTrue(result.attempted,
                        "continuation-goal regression did not attempt planning");
            requireTrue(result.accepted,
                        std::string("continuation-goal regression was not accepted: ") +
                            result.status);
            requireTrue(result.local_continuation_active,
                        "continuation-goal regression did not preserve continuation semantics");
            requireTrue(result.candidate_kind == dynamic_planner::CandidateKind::Complete,
                        "continuation-goal regression unexpectedly returned a padded partial");
            requireTrue(result.committed_endpoint_distance_m <=
                            continuation_goal_config.goal_tolerance_m + 1e-9,
                        "continuation-goal fixture did not reach the old goal_seen latch condition");
            requireTrue(!result.goal_seen && !planner.goalSeen(),
                        "continuation candidate incorrectly latched goal_seen");
            requireTrue(
                result.status == "ACCEPTED" ||
                    result.status == "ACCEPTED_AFTER_RECHECK",
                "continuation candidate returned unexpected status: " + result.status);
        }

        // C1F.8c IRL portability: low-confidence CV prediction may still guide
        // a local pursuit waypoint, but it must not claim a moving terminal
        // state once the candidate end time lies beyond the trusted horizon.
        {
            RecedingHorizonConfig cv_config = config(0.0);
            cv_config.fixed_splice_lookahead_s = 0.10;
            cv_config.enable_octopus_useful_deadline = false;
            cv_config.moving_rendezvous_enabled = true;
            cv_config.terminal_time_target_prediction_enabled = true;
            cv_config.target_time_fixed_point_iterations = 6;
            cv_config.target_time_fixed_point_tolerance_s = 0.01;
            cv_config.target_prediction_low_confidence_horizon_s = 0.20;
            cv_config.target_prediction_low_confidence_max_distance_m = 0.15;
            RecedingHorizonPlanner planner(
                Vec3(1.2, 0.0, 1.5), cv_config, localConfig());
            planner.initializeCommittedTrajectory(
                dynamic_planner::makeStationaryHoverTrajectory(
                    hover_state.position, 30.0, 0.10));
            auto world = emptySuspendedWorld();
            dynamic_planner::ConstantVelocityTargetPredictor predictor(
                Vec3(1.2, 0.0, 1.5), Vec3(0.2, 0.0, 0.0), 30.0);
            const auto result = planner.replan(
                30.0, hover_state, world, []() { return 30.01; }, &predictor);
            requireTrue(result.attempted,
                        "low-confidence target test did not attempt planning");
            requireTrue(result.target_time_prediction_used &&
                        result.target_time_prediction_valid,
                        "low-confidence target prediction was not evaluated");
            requireTrue(!result.target_time_prediction_high_confidence,
                        "constant-velocity target was incorrectly high-confidence");
            requireTrue(!result.moving_target_terminal_active,
                        "low-confidence prediction beyond horizon became a moving terminal state");
            requireTrue(result.predicted_target_horizon_s <= 0.20 + 1e-9,
                        "low-confidence target prediction exceeded trusted horizon");
            requireMatrixNear(result.terminal_target_velocity, Vec3::Zero(), 1e-12,
                              "untrusted low-confidence future retained terminal target velocity");
        }

        std::cout << "C.1b/C.1c integration timing contract: PASS\n";
        std::cout << "150 ms first overrun rejected without authority: true\n";
        std::cout << "trajectory-time-seeded fresh shadow solve keeps future C2 splice: true\n";
        std::cout << "C1F.5p2 useful deadline follows trajectory clock with certification reserve: true\n";
        std::cout << "C1F.8c target state is evaluated at the actual candidate terminal time: true\n";
        std::cout << "C1F.8c far moving target uses continuation rather than fake rendezvous velocity: true\n";
        std::cout << "C1F.8c low-confidence prediction cannot overclaim rendezvous horizon: true\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_integration_timing FAILED: " << exc.what() << '\n';
        return 1;
    }
}
