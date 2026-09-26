#include "dynamic_planner/receding_horizon_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dynamic_planner {

namespace {

double stoppingTime(const State& state, const Vec3& a_max) {
    if (!state.velocity.allFinite() || !a_max.allFinite() ||
        (a_max.array() <= 0.0).any()) {
        throw std::invalid_argument("invalid state/acceleration limits for stopping time");
    }
    // Match makeBrakingHoverTrajectory(): its velocity-continuous clamped
    // cubic has |a(0)| = 2|v0|/T, so T >= 2|v0|/a_max is the actual
    // axis-wise duration needed by the fallback we would execute.
    return (2.0 * state.velocity.cwiseAbs().array() / a_max.array()).maxCoeff();
}

}  // namespace

const char* candidateKindName(CandidateKind kind) noexcept {
    switch (kind) {
        case CandidateKind::None: return "NONE";
        case CandidateKind::Complete: return "COMPLETE";
        case CandidateKind::Partial: return "PARTIAL";
        case CandidateKind::Incumbent: return "INCUMBENT";
        case CandidateKind::Fallback: return "FALLBACK";
    }
    return "UNKNOWN";
}

RecedingHorizonPlanner::RecedingHorizonPlanner(
    Vec3 global_goal,
    RecedingHorizonConfig config,
    LocalPlannerConfig local_config)
    : global_goal_(std::move(global_goal)),
      config_(std::move(config)),
      local_planner_([&]() {
          // Keep one source of truth for B.1/B.2 planning radius and dynamics.
          local_config.octopus.v_max = config_.v_max;
          local_config.octopus.a_max = config_.a_max;
          local_config.octopus.planning_radius_m = config_.planning_radius_m;
          local_config.octopus.goal_tolerance_m = config_.goal_tolerance_m;
          local_config.refinement.v_max = config_.v_max;
          local_config.refinement.a_max = config_.a_max;
          local_config.refinement.goal_acceptance_m = config_.goal_tolerance_m;
          return local_config;
      }()),
      safety_checker_(config_.separator_validation_tolerance),
      previous_splice_timing_s_(config_.initial_splice_timing_s) {
    validateConfig();
    if (!global_goal_.allFinite()) {
        throw std::invalid_argument("global goal must be finite");
    }
}

void RecedingHorizonPlanner::validateConfig() const {
    if (!(config_.dc_s > 0.0) || !(config_.planning_radius_m > 0.0) ||
        !(config_.factor_alpha > 0.0) || !(config_.min_splice_lookahead_s >= 0.0) ||
        !(config_.max_splice_lookahead_s >= config_.min_splice_lookahead_s) ||
        !std::isfinite(config_.fixed_splice_lookahead_s) ||
        !(config_.fixed_splice_lookahead_s >= 0.0) ||
        (config_.fixed_splice_lookahead_s > 0.0 &&
         (config_.fixed_splice_lookahead_s < config_.min_splice_lookahead_s ||
          config_.fixed_splice_lookahead_s > config_.max_splice_lookahead_s)) ||
        !(config_.factor_alloc > 0.0) || !(config_.factor_alloc_close > 0.0) ||
        !(config_.spline_time_factor > 0.0) ||
        !(config_.close_to_goal_m >= config_.goal_tolerance_m) ||
        !(config_.goal_tolerance_m > 0.0) || !(config_.continuity_tolerance >= 0.0) ||
        !(config_.separator_validation_tolerance >= 0.0) ||
        !std::isfinite(config_.initial_splice_timing_s) ||
        !(config_.initial_splice_timing_s >= 0.0) ||
        !std::isfinite(config_.octopus_post_search_reserve_s) ||
        !(config_.octopus_post_search_reserve_s >= 0.0) ||
        !std::isfinite(config_.incumbent_failure_recheck_horizon_s) ||
        !(config_.incumbent_failure_recheck_horizon_s > 0.0) ||
        !std::isfinite(config_.terminal_hold_guard_s) ||
        !(config_.terminal_hold_guard_s >= 0.0) ||
        !std::isfinite(config_.terminal_hold_guard_interval_s) ||
        !(config_.terminal_hold_guard_interval_s > 0.0) ||
        !std::isfinite(config_.emergency_panic_horizon_s) ||
        !(config_.emergency_panic_horizon_s > 0.0) ||
        !config_.moving_rendezvous_terminal_velocity.allFinite() ||
        config_.target_time_fixed_point_iterations < 1 ||
        !std::isfinite(config_.target_time_fixed_point_tolerance_s) ||
        !(config_.target_time_fixed_point_tolerance_s > 0.0) ||
        !std::isfinite(config_.target_prediction_low_confidence_horizon_s) ||
        !(config_.target_prediction_low_confidence_horizon_s > 0.0) ||
        !std::isfinite(config_.target_prediction_high_confidence_horizon_s) ||
        !(config_.target_prediction_high_confidence_horizon_s > 0.0) ||
        !std::isfinite(config_.target_prediction_low_confidence_max_distance_m) ||
        !(config_.target_prediction_low_confidence_max_distance_m >= 0.0) ||
        !config_.rendezvous_backup_brake_accel_limit.allFinite() ||
        (config_.rendezvous_backup_brake_accel_limit.array() <= 0.0).any() ||
        !std::isfinite(config_.rendezvous_backup_min_duration_s) ||
        !(config_.rendezvous_backup_min_duration_s > 0.0) ||
        (config_.enable_octopus_useful_deadline &&
         (!(config_.octopus_post_search_reserve_s > 0.0) ||
          !(config_.octopus_post_search_reserve_s < config_.max_splice_lookahead_s))) ||
        !config_.v_max.allFinite() || !(config_.v_max.array() > 0.0).all() ||
        !config_.a_max.allFinite() || !(config_.a_max.array() > 0.0).all()) {
        throw std::invalid_argument("invalid receding-horizon configuration");
    }
}

double RecedingHorizonPlanner::chooseSpliceLookahead() const {
    if (config_.fixed_splice_lookahead_s > 0.0) {
        return config_.fixed_splice_lookahead_s;
    }
    const double timing = std::max(0.0, previous_splice_timing_s_);
    const double ticks = std::max(1.0, std::ceil(timing / config_.dc_s));
    const double raw = config_.factor_alpha * ticks * config_.dc_s;
    return std::clamp(raw, config_.min_splice_lookahead_s,
                      config_.max_splice_lookahead_s);
}

Vec3 RecedingHorizonPlanner::projectLocalGoal(const Vec3& from) const {
    const Vec3 delta = global_goal_ - from;
    const double distance = delta.norm();
    if (distance <= config_.goal_tolerance_m) {
        return global_goal_;
    }
    const double radius = std::min(
        std::max(0.0, distance - 0.001), config_.planning_radius_m);
    return from + radius * delta / distance;
}

double RecedingHorizonPlanner::allocateLocalDuration(
    const State& start,
    const Vec3& local_goal,
    const Vec3& terminal_velocity,
    double distance_for_allocation,
    double* minimum_time,
    double* applied_factor) const {
    const double minimum = minimumTimeDoubleIntegrator3D(
        start.position, start.velocity, local_goal, terminal_velocity,
        config_.v_max, config_.a_max);
    const double rmader_factor = distance_for_allocation <= config_.close_to_goal_m
        ? config_.factor_alloc_close : config_.factor_alloc;
    const double factor = std::max(rmader_factor, config_.spline_time_factor);
    const double duration = factor * minimum;
    if (minimum_time != nullptr) {
        *minimum_time = minimum;
    }
    if (applied_factor != nullptr) {
        *applied_factor = factor;
    }
    if (!std::isfinite(duration) || !(duration > 1e-6)) {
        throw std::runtime_error("local duration is non-positive or non-finite");
    }
    return duration;
}

bool RecedingHorizonPlanner::goalReached(const State& state) const {
    if (config_.moving_rendezvous_enabled) {
        // C1F.6: the mission-level bridge-feasibility gate owns the actual
        // moving-target handoff. Do not freeze C++ replanning merely because
        // position happens to cross the goal tolerance.
        return false;
    }
    return state.position.allFinite()
        && (state.position - global_goal_).norm() <= config_.goal_tolerance_m;
}

void RecedingHorizonPlanner::initializeCommittedTrajectory(
    CommittedTrajectory committed) {
    if (goal_seen_ || !committed_.empty()) {
        throw std::logic_error(
            "initial committed trajectory may only be set before planner authority starts");
    }
    if (committed.empty()) {
        throw std::invalid_argument("initial committed trajectory must not be empty");
    }
    // Validate enough of the public contract here to catch malformed seed data
    // without imposing a particular seed geometry.
    (void)committed.startTime();
    (void)committed.endState();
    committed_ = std::move(committed);
}

ReplanResult RecedingHorizonPlanner::replan(
    double now_s,
    const State& measured_state,
    const FrozenWorld& world) {
    WorldSnapshot initial;
    initial.captured_at_s = now_s;
    initial.static_obstacles = world.obstacles;
    // Static FrozenWorld vertices already include ego C-space inflation, so the
    // ego half extents are irrelevant unless cooperative obstacles are added.
    initial.ego_half_extents = Vec3::Zero();
    VersionedWorld versioned(std::move(initial));
    return replan(now_s, measured_state, versioned);
}

ReplanResult RecedingHorizonPlanner::replan(
    double now_s,
    const State& measured_state,
    const WorldSnapshotSource& world) {
    return replan(now_s, measured_state, world, TrajectoryTimeSource{});
}

ReplanResult RecedingHorizonPlanner::replan(
    double now_s,
    const State& measured_state,
    const WorldSnapshotSource& world,
    const TrajectoryTimeSource& trajectory_now_s,
    const TargetPredictor* target_predictor) {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    const auto elapsedSeconds = [&]() {
        return std::chrono::duration<double>(Clock::now() - started).count();
    };
    double last_trajectory_time_s = now_s;
    const auto trajectoryNow = [&]() {
        const double value = trajectory_now_s
            ? trajectory_now_s()
            : now_s + elapsedSeconds();
        if (!std::isfinite(value)) {
            throw std::runtime_error("trajectory time source returned non-finite time");
        }
        if (value < last_trajectory_time_s - 1e-9) {
            throw std::runtime_error("trajectory time source moved backwards");
        }
        last_trajectory_time_s = std::max(last_trajectory_time_s, value);
        return value;
    };
    const auto finish = [&](ReplanResult result) {
        result.replan_runtime_s = elapsedSeconds();
        previous_replan_runtime_s_ = result.replan_runtime_s;

        // Future-splice lead belongs to the trajectory clock domain, not the
        // CPU/wall clock domain. Under Gazebo, /clock may advance slower/faster
        // than wall time. Sample the trajectory clock at return when possible;
        // an invalid clock does not erase a last valid observation already made.
        if (trajectory_now_s) {
            try {
                (void)trajectoryNow();
            } catch (const std::exception&) {
                // Primary call sites that require a valid timestamp set an
                // explicit status. For unrelated early failures, preserve the
                // last valid time rather than masking the original failure.
            }
        } else {
            last_trajectory_time_s = now_s + result.replan_runtime_s;
        }
        result.trajectory_elapsed_s =
            std::max(0.0, last_trajectory_time_s - now_s);
        previous_splice_timing_s_ = result.trajectory_elapsed_s;
        result.goal_seen = goal_seen_;
        return result;
    };

    ReplanResult result;
    result.now_s = now_s;
    result.goal_reached = goalReached(measured_state);
    if (!std::isfinite(now_s) || !measured_state.position.allFinite() ||
        !measured_state.velocity.allFinite() || !measured_state.acceleration.allFinite()) {
        result.status = "INVALID_STATE";
        return finish(std::move(result));
    }
    if (result.goal_reached) {
        result.status = "GOAL_REACHED";
        goal_seen_ = true;
        return finish(std::move(result));
    }
    if (goal_seen_) {
        result.status = "GOAL_SEEN_EXECUTE_COMMITTED";
        if (!committed_.empty()) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }

    WorldSnapshot planning_world;
    try {
        const auto snapshot_started = Clock::now();
        planning_world = world.snapshot(now_s);
        result.world_snapshot_s +=
            std::chrono::duration<double>(Clock::now() - snapshot_started).count();
        result.planning_world_version = planning_world.version;
    } catch (const std::exception&) {
        result.status = "WORLD_SNAPSHOT_FAILED";
        return finish(std::move(result));
    }

    result.attempted = true;
    State A = measured_state;
    double splice_time = now_s;
    double lookahead = 0.0;
    const bool had_incumbent = !committed_.empty();
    if (had_incumbent) {
        if (now_s < committed_.startTime() - 1e-9) {
            result.status = "COMMITTED_TIME_OUT_OF_RANGE";
            return finish(std::move(result));
        }
        lookahead = chooseSpliceLookahead();
        if (!committed_.endsInStoppedHold()) {
            result.estimated_brake_time_s = stoppingTime(measured_state, config_.a_max);
            result.fallback_trigger_horizon_s = std::max(
                config_.emergency_panic_horizon_s,
                result.estimated_brake_time_s + config_.dc_s);
            result.incumbent_time_to_conflict_s =
                std::max(0.0, committed_.endTime() - now_s);
            result.incumbent_reaction_horizon_reached =
                result.incumbent_time_to_conflict_s <= result.fallback_trigger_horizon_s;
            if (result.incumbent_reaction_horizon_reached) {
                result.candidate_kind = CandidateKind::Incumbent;
            }

            // Being inside the reaction horizon is a reason to prefer a new
            // complete/partial trajectory urgently, not a reason to skip the
            // planner. Keep the future splice inside the incumbent's explicit
            // finite domain and give Octopus one last chance to evade before
            // the caller installs the emergency fallback.
            const double remaining_s = std::max(0.0, committed_.endTime() - now_s);
            constexpr double kEndpointMarginS = 1e-6;
            lookahead = std::min(
                lookahead, std::max(0.0, remaining_s - kEndpointMarginS));
        }
        splice_time = now_s + lookahead;
        try {
            A = committed_.evaluate(splice_time);
        } catch (const std::exception&) {
            result.status = "COMMITTED_TIME_OUT_OF_RANGE";
            return finish(std::move(result));
        }
    }

    result.splice_time_s = splice_time;
    result.splice_lookahead_s = splice_time - now_s;
    result.splice_state = A;

    // C1F.4: if a replacement plan later fails, continuing the old incumbent is
    // only permissible if that incumbent is still safe in the newest world.
    // Check beyond its finite endpoint as well because CommittedTrajectory
    // extends a certified stopped endpoint indefinitely.
    auto revalidateRemainingIncumbent = [&](
        double check_from_s, const std::string& unsafe_status) -> bool {
        if (!had_incumbent) return true;
        try {
            const WorldSnapshot latest = world.snapshot(check_from_s);
            result.recheck_world_version = latest.version;
            const double check_until_s = committed_.endsInStoppedHold()
                ? std::max(committed_.endTime(),
                           check_from_s + config_.incumbent_failure_recheck_horizon_s)
                : std::min(committed_.endTime(),
                           check_from_s + config_.incumbent_failure_recheck_horizon_s);
            const auto safety_started = Clock::now();
            result.prefix_safety = safety_checker_.checkCommitted(
                committed_, check_from_s, check_until_s, latest);
            result.safety_check_s +=
                std::chrono::duration<double>(Clock::now() - safety_started).count();
            result.incumbent_recheck_performed = true;
            result.incumbent_recheck_world_version = latest.version;
            result.estimated_brake_time_s = stoppingTime(measured_state, config_.a_max);
            result.fallback_trigger_horizon_s = std::max(
                config_.emergency_panic_horizon_s,
                result.estimated_brake_time_s + config_.dc_s);

            if (!result.prefix_safety->safe) {
                result.incumbent_time_to_conflict_s = std::max(
                    0.0, result.prefix_safety->unsafe_interval_start_s - check_from_s);
                result.candidate_kind = CandidateKind::Incumbent;
                const bool defer_conflict =
                    config_.defer_incumbent_conflict_until_reaction_horizon &&
                    result.incumbent_time_to_conflict_s > result.fallback_trigger_horizon_s;
                if (!defer_conflict) {
                    result.incumbent_prefix_unsafe = true;
                    result.incumbent_reaction_horizon_reached =
                        result.incumbent_time_to_conflict_s <= result.fallback_trigger_horizon_s;
                    result.status = unsafe_status;
                    return false;
                }
                // Simulation/cage policy: a conflict outside the actual
                // braking/reaction horizon is a replanning problem, not an
                // immediate-hover condition. Conservative/default configs do
                // not enter this branch.
                return true;
            }
            if (!committed_.endsInStoppedHold()) {
                result.incumbent_time_to_conflict_s = std::max(
                    0.0, committed_.endTime() - check_from_s);
                if (result.incumbent_time_to_conflict_s <= result.fallback_trigger_horizon_s) {
                    result.incumbent_reaction_horizon_reached = true;
                    result.candidate_kind = CandidateKind::Incumbent;
                }
            }
            return true;
        } catch (const std::exception&) {
            // Fail closed: an incumbent that cannot be revalidated after a
            // failed replacement must not be treated as certified.
            result.incumbent_prefix_unsafe = true;
            result.status = "INCUMBENT_RECHECK_FAILED:" + unsafe_status;
            return false;
        }
    };
    result.global_distance_from_splice_m = (global_goal_ - A.position).norm();
    if (!config_.moving_rendezvous_enabled &&
        result.global_distance_from_splice_m <= config_.goal_tolerance_m) {
        goal_seen_ = true;
        result.status = "GOAL_SEEN";
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }

    const auto projectToward = [&](const Vec3& target, bool exact_if_local) -> Vec3 {
        const Vec3 delta = target - A.position;
        const double distance = delta.norm();
        if (distance <= config_.goal_tolerance_m ||
            (exact_if_local && distance <= config_.planning_radius_m)) {
            return target;
        }
        const double radius = std::min(
            std::max(0.0, distance - 0.001), config_.planning_radius_m);
        return (A.position + radius * delta / distance).eval();
    };

    result.local_goal = projectLocalGoal(A.position);
    result.terminal_target_velocity = Vec3::Zero();
    result.moving_target_terminal_active = false;
    result.local_continuation_active = false;
    result.moving_rendezvous_active = false;

    try {
        if (config_.moving_rendezvous_enabled &&
            config_.terminal_time_target_prediction_enabled &&
            target_predictor != nullptr) {
            result.target_time_prediction_used = true;
            result.target_time_prediction_high_confidence =
                target_predictor->highConfidenceFuture();

            double prediction_horizon_s = result.target_time_prediction_high_confidence
                ? config_.target_prediction_high_confidence_horizon_s
                : config_.target_prediction_low_confidence_horizon_s;
            const TargetPrediction at_now = target_predictor->evaluate(now_s);
            if (!at_now.valid) {
                result.status = "TARGET_PREDICTOR_INVALID:" + at_now.detail;
                return finish(std::move(result));
            }
            if (!result.target_time_prediction_high_confidence &&
                config_.target_prediction_low_confidence_max_distance_m > 0.0) {
                const double speed = at_now.velocity.norm();
                if (speed > 1e-9) {
                    prediction_horizon_s = std::min(
                        prediction_horizon_s,
                        config_.target_prediction_low_confidence_max_distance_m / speed);
                }
            }
            const double prediction_valid_until_s = now_s + prediction_horizon_s;

            TargetPrediction seed_prediction = target_predictor->evaluate(
                std::min(splice_time, prediction_valid_until_s));
            if (!seed_prediction.valid) {
                result.status = "TARGET_PREDICTOR_INVALID:" + seed_prediction.detail;
                return finish(std::move(result));
            }
            const double seed_distance = (seed_prediction.position - A.position).norm();
            const bool seed_prediction_covers_terminal =
                splice_time <= prediction_valid_until_s + 1e-9;
            bool seed_rendezvous = seed_prediction_covers_terminal &&
                seed_distance <= config_.planning_radius_m;
            Vec3 seed_goal = projectToward(seed_prediction.position, seed_rendezvous);
            Vec3 seed_terminal_velocity = seed_rendezvous
                ? seed_prediction.velocity : Vec3::Zero();
            double duration_s = allocateLocalDuration(
                A, seed_goal, seed_terminal_velocity, seed_distance,
                &result.minimum_time_s, &result.time_allocation_factor);

            TargetPrediction final_prediction = seed_prediction;
            Vec3 final_goal = seed_goal;
            Vec3 final_terminal_velocity = seed_terminal_velocity;
            bool final_rendezvous = seed_rendezvous;
            double residual_s = std::numeric_limits<double>::infinity();
            int iterations_used = 0;

            for (int iteration = 0;
                 iteration < config_.target_time_fixed_point_iterations;
                 ++iteration) {
                const double candidate_terminal_time_s = splice_time + duration_s;
                const bool prediction_covers_terminal =
                    candidate_terminal_time_s <= prediction_valid_until_s + 1e-9;
                const double evaluation_time_s = std::min(
                    candidate_terminal_time_s, prediction_valid_until_s);
                const TargetPrediction prediction =
                    target_predictor->evaluate(evaluation_time_s);
                if (!prediction.valid) {
                    result.status = "TARGET_PREDICTOR_INVALID:" + prediction.detail;
                    return finish(std::move(result));
                }

                const double target_distance = (prediction.position - A.position).norm();
                const bool rendezvous = prediction_covers_terminal &&
                    target_distance <= config_.planning_radius_m;
                const Vec3 local_goal = projectToward(prediction.position, rendezvous);
                const Vec3 terminal_velocity = rendezvous
                    ? prediction.velocity : Vec3::Zero();
                double minimum_s = 0.0;
                double allocation_factor = 0.0;
                const double updated_duration_s = allocateLocalDuration(
                    A, local_goal, terminal_velocity, target_distance,
                    &minimum_s, &allocation_factor);

                residual_s = std::abs(updated_duration_s - duration_s);
                duration_s = updated_duration_s;
                final_prediction = prediction;
                final_goal = local_goal;
                final_terminal_velocity = terminal_velocity;
                final_rendezvous = rendezvous;
                result.minimum_time_s = minimum_s;
                result.time_allocation_factor = allocation_factor;
                iterations_used = iteration + 1;
                if (residual_s <= config_.target_time_fixed_point_tolerance_s) {
                    break;
                }
            }

            if (!std::isfinite(residual_s) ||
                residual_s > config_.target_time_fixed_point_tolerance_s) {
                result.status = "TARGET_TIME_FIXED_POINT_NOT_CONVERGED";
                return finish(std::move(result));
            }

            // One final provider evaluation anchors the terminal state to the
            // actual accepted candidate end time. The preceding fixed point
            // guarantees this correction is within the configured time
            // tolerance rather than a separate pre-lead heuristic.
            const double final_terminal_time_s = splice_time + duration_s;
            const bool prediction_covers_terminal =
                final_terminal_time_s <= prediction_valid_until_s + 1e-9;
            const double final_evaluation_time_s = std::min(
                final_terminal_time_s, prediction_valid_until_s);
            final_prediction = target_predictor->evaluate(final_evaluation_time_s);
            if (!final_prediction.valid) {
                result.status = "TARGET_PREDICTOR_INVALID:" + final_prediction.detail;
                return finish(std::move(result));
            }
            const double final_distance = (final_prediction.position - A.position).norm();
            final_rendezvous = prediction_covers_terminal &&
                final_distance <= config_.planning_radius_m;
            final_goal = projectToward(final_prediction.position, final_rendezvous);
            final_terminal_velocity = final_rendezvous
                ? final_prediction.velocity : Vec3::Zero();

            result.local_goal = final_goal;
            result.local_duration_s = duration_s;
            result.moving_target_terminal_active = final_rendezvous;
            result.local_continuation_active = prediction_covers_terminal && !final_rendezvous;
            result.moving_rendezvous_active = final_rendezvous;
            result.terminal_target_velocity = final_terminal_velocity;
            result.global_distance_from_splice_m = final_distance;
            result.target_time_prediction_valid = true;
            result.target_time_fixed_point_iterations = iterations_used;
            result.target_time_fixed_point_residual_s = residual_s;
            result.predicted_target_time_s = final_evaluation_time_s;
            result.predicted_target_horizon_s = final_evaluation_time_s - now_s;
            result.predicted_target_position = final_prediction.position;
            result.predicted_target_velocity = final_prediction.velocity;
        } else {
            result.local_goal = projectLocalGoal(A.position);
            result.moving_rendezvous_active = config_.moving_rendezvous_enabled &&
                (result.local_goal - global_goal_).norm() <= config_.goal_tolerance_m;
            result.moving_target_terminal_active = result.moving_rendezvous_active;
            result.local_continuation_active = false;
            result.terminal_target_velocity = result.moving_rendezvous_active
                ? config_.moving_rendezvous_terminal_velocity
                : Vec3::Zero();
            result.local_duration_s = allocateLocalDuration(
                A, result.local_goal, result.terminal_target_velocity,
                result.global_distance_from_splice_m,
                &result.minimum_time_s, &result.time_allocation_factor);
        }
    } catch (const std::exception&) {
        result.status = "TIME_ALLOCATION_FAILED";
        return finish(std::move(result));
    }

    LocalPlanRequest request;
    request.initial_state = A;
    request.local_goal = result.local_goal;
    request.t_start = splice_time;
    request.duration_s = result.local_duration_s;
    if (result.moving_target_terminal_active) {
        request.terminal_boundary.mode = TerminalMode::MovingRendezvous;
        request.terminal_boundary.position = result.local_goal;
        request.terminal_boundary.velocity = result.terminal_target_velocity;
    } else if (result.local_continuation_active) {
        request.terminal_boundary.mode = TerminalMode::Continuation;
        request.terminal_boundary.position = result.local_goal;
        request.terminal_boundary.velocity.setZero();
    } else {
        request.terminal_boundary.mode = TerminalMode::Stopped;
        request.terminal_boundary.position = result.local_goal;
        request.terminal_boundary.velocity.setZero();
    }

    // C1E can stop Octopus at the last trajectory-time instant that still
    // leaves a measured reserve for refinement/checking/authority. The callback
    // intentionally samples the same trajectory clock as the final late gate;
    // max_runtime_s remains an independent steady-clock hard backstop.
    bool useful_deadline_clock_invalid = false;
    if (had_incumbent && config_.enable_octopus_useful_deadline) {
        result.octopus_useful_deadline_enabled = true;
        result.octopus_post_search_reserve_s = config_.octopus_post_search_reserve_s;
        result.octopus_useful_deadline_time_s =
            splice_time - config_.octopus_post_search_reserve_s;
        result.octopus_useful_budget_s = std::max(
            0.0, result.octopus_useful_deadline_time_s - now_s);
        request.octopus_stop_reason = "USEFUL_DEADLINE_REACHED";
        request.octopus_stop_requested = [&]() noexcept {
            try {
                return trajectoryNow() >= result.octopus_useful_deadline_time_s - 1e-12;
            } catch (...) {
                useful_deadline_clock_invalid = true;
                return true;
            }
        };
    }
    try {
        const auto obstacle_started = Clock::now();
        request.obstacles = planning_world.timeIndexedObstacles(
            local_planner_.config().num_segments,
            splice_time,
            splice_time + result.local_duration_s);
        if (!result.moving_target_terminal_active &&
            config_.require_terminal_hold_for_nominal &&
            config_.terminal_hold_guard_s > 1e-9) {
            const int guard_segments = std::max(
                1, static_cast<int>(std::ceil(
                    config_.terminal_hold_guard_s / config_.terminal_hold_guard_interval_s)));
            request.terminal_hold_obstacles = planning_world.timeIndexedObstacles(
                guard_segments,
                splice_time + result.local_duration_s,
                splice_time + result.local_duration_s + config_.terminal_hold_guard_s);
        }
        result.obstacle_build_s +=
            std::chrono::duration<double>(Clock::now() - obstacle_started).count();
    } catch (const std::exception&) {
        result.status = "PLANNING_WORLD_INVALID";
        return finish(std::move(result));
    }

    result.local_plan = local_planner_.plan(request);
    result.octopus_deadline_clock_invalid = useful_deadline_clock_invalid;
    if (useful_deadline_clock_invalid) {
        result.status = "TRAJECTORY_TIME_INVALID";
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }
    if (!result.local_plan->success || !result.local_plan->control_points.has_value()) {
        const std::string local_failure =
            std::string("LOCAL_PLAN_FAILED:") + result.local_plan->status;
        result.status = local_failure;
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
            try {
                result.candidate_finish_time_s = trajectoryNow();
                (void)revalidateRemainingIncumbent(
                    result.candidate_finish_time_s,
                    "INCUMBENT_REMAINING_UNSAFE_AFTER_" + local_failure);
            } catch (const std::exception&) {
                result.incumbent_prefix_unsafe = true;
                result.status = "INCUMBENT_RECHECK_TIME_INVALID_AFTER_" + local_failure;
            }
        }
        return finish(std::move(result));
    }

    result.candidate_kind = result.local_plan->partial
        ? CandidateKind::Partial
        : CandidateKind::Complete;
    if (result.local_plan->partial) {
        // A safe padded partial is deliberately a local evasive continuation.
        // It must not inherit the hard moving-rendezvous endpoint/velocity
        // contract merely because the requested global goal is nearby.
        result.moving_rendezvous_active = false;
        result.moving_target_terminal_active = false;
        result.local_continuation_active = false;
        result.terminal_target_velocity.setZero();
    }

    try {
        result.candidate_finish_time_s = trajectoryNow();
    } catch (const std::exception&) {
        result.status = "TRAJECTORY_TIME_INVALID";
        return finish(std::move(result));
    }
    if (had_incumbent && result.candidate_finish_time_s >= splice_time - 1e-12) {
        result.candidate_late = true;
        result.status = "LATE_CANDIDATE_REJECTED";
        result.committed_endpoint_distance_m =
            (committed_.endState().position - global_goal_).norm();
        (void)revalidateRemainingIncumbent(
            result.candidate_finish_time_s,
            "INCUMBENT_REMAINING_UNSAFE_AFTER_LATE_CANDIDATE");
        return finish(std::move(result));
    }

    TrajectoryPiece candidate;
    candidate.control_points = *result.local_plan->control_points;
    candidate.knots = result.local_plan->knots;
    candidate.valid_from = splice_time;
    candidate.valid_until = splice_time + result.local_duration_s;

    std::optional<TrajectoryPiece> rendezvous_backup_piece;
    const bool moving_nominal_needs_stopped_backup =
        result.moving_target_terminal_active || result.local_continuation_active;
    if (moving_nominal_needs_stopped_backup) {
        try {
            const State terminal_state = evaluateCubicState(
                candidate.control_points, candidate.knots, candidate.valid_until);
            if (result.moving_target_terminal_active) {
                result.terminal_velocity_error_mps =
                    (terminal_state.velocity - result.terminal_target_velocity).norm();
            }
            const CommittedTrajectory backup = makeBrakingHoverTrajectory(
                terminal_state, candidate.valid_until,
                config_.rendezvous_backup_brake_accel_limit,
                config_.rendezvous_backup_min_duration_s);
            if (backup.pieces().size() != 1U) {
                throw std::runtime_error("rendezvous backup must contain exactly one brake piece");
            }
            rendezvous_backup_piece = backup.pieces().front();
            result.rendezvous_backup_duration_s =
                rendezvous_backup_piece->valid_until - rendezvous_backup_piece->valid_from;
        } catch (const std::exception&) {
            result.status = "RENDEZVOUS_BACKUP_BUILD_FAILED";
            if (had_incumbent) {
                (void)revalidateRemainingIncumbent(
                    result.candidate_finish_time_s,
                    "INCUMBENT_REMAINING_UNSAFE_AFTER_RENDEZVOUS_BACKUP_BUILD_FAILED");
            }
            return finish(std::move(result));
        }
    }

    auto checkAgainstSnapshot = [&](const WorldSnapshot& latest,
                                    bool retry) -> bool {
        try {
            if (had_incumbent) {
                const auto prefix_started = Clock::now();
                result.prefix_safety = safety_checker_.checkCommitted(
                    committed_, result.candidate_finish_time_s,
                    splice_time, latest);
                result.safety_check_s +=
                    std::chrono::duration<double>(Clock::now() - prefix_started).count();
                if (!result.prefix_safety->safe) {
                    result.incumbent_prefix_unsafe = true;
                    result.status = retry
                        ? "INCUMBENT_PREFIX_UNSAFE_AFTER_RETRY"
                        : "INCUMBENT_PREFIX_UNSAFE";
                    return false;
                }
            } else {
                TrajectorySafetyResult empty_prefix;
                empty_prefix.safe = true;
                empty_prefix.status = "SAFE_NO_INCUMBENT";
                empty_prefix.checked_from_s = result.candidate_finish_time_s;
                empty_prefix.checked_until_s = splice_time;
                result.prefix_safety = empty_prefix;
            }

            const auto candidate_started = Clock::now();
            result.candidate_safety = safety_checker_.checkPiece(
                candidate, splice_time, candidate.valid_until, latest);
            result.safety_check_s +=
                std::chrono::duration<double>(Clock::now() - candidate_started).count();
            if (!result.candidate_safety->safe) {
                result.status = retry
                    ? "LATEST_CANDIDATE_UNSAFE_AFTER_RETRY"
                    : "LATEST_CANDIDATE_UNSAFE";
                return false;
            }
            if (moving_nominal_needs_stopped_backup && config_.require_rendezvous_backup_for_nominal) {
                if (!rendezvous_backup_piece.has_value()) {
                    result.status = "RENDEZVOUS_BACKUP_MISSING";
                    return false;
                }
                const auto backup_started = Clock::now();
                result.candidate_backup_safety = safety_checker_.checkPiece(
                    *rendezvous_backup_piece, rendezvous_backup_piece->valid_from,
                    rendezvous_backup_piece->valid_until, latest);
                result.safety_check_s +=
                    std::chrono::duration<double>(Clock::now() - backup_started).count();
                if (!result.candidate_backup_safety->safe) {
                    result.status = retry
                        ? "LATEST_RENDEZVOUS_BACKUP_UNSAFE_AFTER_RETRY"
                        : "LATEST_RENDEZVOUS_BACKUP_UNSAFE";
                    return false;
                }
                if (config_.terminal_hold_guard_s > 1e-9) {
                    CommittedTrajectory candidate_with_backup;
                    const SpliceDiagnostics seed = candidate_with_backup.replaceSuffix(
                        splice_time, candidate, config_.continuity_tolerance);
                    const SpliceDiagnostics backup_append = seed.accepted
                        ? candidate_with_backup.appendVelocityContinuousBackup(
                            *rendezvous_backup_piece, config_.continuity_tolerance)
                        : SpliceDiagnostics{};
                    if (!seed.accepted || !backup_append.accepted ||
                        !candidate_with_backup.endsInStoppedHold(1e-6)) {
                        result.status = "RENDEZVOUS_BACKUP_APPEND_FAILED";
                        return false;
                    }
                    const auto terminal_started = Clock::now();
                    result.candidate_terminal_hold_safety = safety_checker_.checkCommitted(
                        candidate_with_backup, rendezvous_backup_piece->valid_until,
                        rendezvous_backup_piece->valid_until + config_.terminal_hold_guard_s, latest);
                    result.safety_check_s +=
                        std::chrono::duration<double>(Clock::now() - terminal_started).count();
                    if (!result.candidate_terminal_hold_safety->safe) {
                        result.status = retry
                            ? "LATEST_RENDEZVOUS_BACKUP_HOLD_UNSAFE_AFTER_RETRY"
                            : "LATEST_RENDEZVOUS_BACKUP_HOLD_UNSAFE";
                        return false;
                    }
                }
            } else if (!moving_nominal_needs_stopped_backup &&
                       config_.require_terminal_hold_for_nominal &&
                       config_.terminal_hold_guard_s > 1e-9) {
                CommittedTrajectory candidate_with_hold;
                const SpliceDiagnostics seed = candidate_with_hold.replaceSuffix(
                    splice_time, candidate, config_.continuity_tolerance);
                if (!seed.accepted || !candidate_with_hold.endsInStoppedHold(1e-6)) {
                    result.status = "CANDIDATE_TERMINAL_NOT_STOPPED";
                    return false;
                }
                const auto terminal_started = Clock::now();
                result.candidate_terminal_hold_safety = safety_checker_.checkCommitted(
                    candidate_with_hold, candidate.valid_until,
                    candidate.valid_until + config_.terminal_hold_guard_s, latest);
                result.safety_check_s +=
                    std::chrono::duration<double>(Clock::now() - terminal_started).count();
                if (!result.candidate_terminal_hold_safety->safe) {
                    result.status = retry
                        ? "LATEST_CANDIDATE_TERMINAL_HOLD_UNSAFE_AFTER_RETRY"
                        : "LATEST_CANDIDATE_TERMINAL_HOLD_UNSAFE";
                    return false;
                }
            }
        } catch (const std::exception& exc) {
            result.recheck_error = exc.what();
            result.status = retry
                ? "FINAL_RECHECK_EXCEPTION"
                : "RECHECK_EXCEPTION";
            return false;
        } catch (...) {
            result.recheck_error = "unknown exception";
            result.status = retry
                ? "FINAL_RECHECK_EXCEPTION"
                : "RECHECK_EXCEPTION";
            return false;
        }
        return true;
    };

    WorldSnapshot latest_world;
    try {
        const auto snapshot_started = Clock::now();
        latest_world = world.snapshot(result.candidate_finish_time_s);
        result.world_snapshot_s +=
            std::chrono::duration<double>(Clock::now() - snapshot_started).count();
        result.recheck_world_version = latest_world.version;
    } catch (const std::exception&) {
        result.status = "RECHECK_WORLD_SNAPSHOT_FAILED";
        return finish(std::move(result));
    }
    if (!checkAgainstSnapshot(latest_world, false)) {
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
            if (!result.incumbent_prefix_unsafe) {
                (void)revalidateRemainingIncumbent(
                    result.candidate_finish_time_s,
                    "INCUMBENT_REMAINING_UNSAFE_AFTER_CANDIDATE_REJECTION");
            }
        }
        return finish(std::move(result));
    }

    bool commit_deadline_passed = false;
    bool commit_time_invalid = false;
    auto attemptAtomicCommit = [&](std::uint64_t version) {
        const auto commit_started = Clock::now();
        const bool stable = world.runIfVersionCurrent(
            version,
            [&]() {
                try {
                    result.authority_decision_time_s = trajectoryNow();
                } catch (const std::exception&) {
                    commit_time_invalid = true;
                    return;
                }
                if (had_incumbent &&
                    result.authority_decision_time_s >= splice_time - 1e-12) {
                    commit_deadline_passed = true;
                    return;
                }
                CommittedTrajectory updated = committed_;
                result.splice_diagnostics = updated.replaceSuffix(
                    splice_time, candidate, config_.continuity_tolerance);
                if (result.splice_diagnostics.accepted && moving_nominal_needs_stopped_backup) {
                    if (!rendezvous_backup_piece.has_value()) {
                        result.splice_diagnostics.accepted = false;
                        return;
                    }
                    const SpliceDiagnostics backup_append =
                        updated.appendVelocityContinuousBackup(
                            *rendezvous_backup_piece, config_.continuity_tolerance);
                    if (!backup_append.accepted || !updated.endsInStoppedHold(1e-6)) {
                        result.splice_diagnostics.accepted = false;
                        return;
                    }
                }
                if (result.splice_diagnostics.accepted) {
                    committed_ = std::move(updated);
                }
            });
        result.atomic_commit_s +=
            std::chrono::duration<double>(Clock::now() - commit_started).count();
        return stable;
    };

    bool stable = attemptAtomicCommit(latest_world.version);
    if (stable && commit_time_invalid) {
        result.status = "TRAJECTORY_TIME_INVALID";
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }
    if (stable && commit_deadline_passed) {
        result.candidate_late = true;
        result.status = "LATE_CANDIDATE_REJECTED_AFTER_RECHECK";
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }
    if (!stable) {
        // One bounded retry only. This is deliberately not full RMADER Delay
        // Check: it handles newly-arrived local world information, not unseen
        // messages in a general simultaneously-replanning multiagent network.
        result.recheck_retry_performed = true;
        WorldSnapshot final_world;
        try {
            const auto snapshot_started = Clock::now();
            final_world = world.snapshot(trajectoryNow());
            result.world_snapshot_s +=
                std::chrono::duration<double>(Clock::now() - snapshot_started).count();
            result.final_recheck_world_version = final_world.version;
        } catch (const std::exception&) {
            result.status = "FINAL_RECHECK_WORLD_SNAPSHOT_FAILED";
            return finish(std::move(result));
        }
        if (!checkAgainstSnapshot(final_world, true)) {
            if (had_incumbent) {
                result.committed_endpoint_distance_m =
                    (committed_.endState().position - global_goal_).norm();
            }
            return finish(std::move(result));
        }
        commit_deadline_passed = false;
        commit_time_invalid = false;
        stable = attemptAtomicCommit(final_world.version);
        if (stable && commit_time_invalid) {
            result.status = "TRAJECTORY_TIME_INVALID";
            if (had_incumbent) {
                result.committed_endpoint_distance_m =
                    (committed_.endState().position - global_goal_).norm();
            }
            return finish(std::move(result));
        }
        if (stable && commit_deadline_passed) {
            result.candidate_late = true;
            result.status = "LATE_CANDIDATE_REJECTED_AFTER_RECHECK";
            if (had_incumbent) {
                result.committed_endpoint_distance_m =
                    (committed_.endState().position - global_goal_).norm();
            }
            return finish(std::move(result));
        }
        if (!stable) {
            result.status = "WORLD_CHANGED_DURING_FINAL_RECHECK";
            if (had_incumbent) {
                result.committed_endpoint_distance_m =
                    (committed_.endState().position - global_goal_).norm();
            }
            return finish(std::move(result));
        }
    }

    if (!result.splice_diagnostics.accepted) {
        result.status = "C2_SPLICE_REJECTED";
        if (had_incumbent) {
            result.committed_endpoint_distance_m =
                (committed_.endState().position - global_goal_).norm();
        }
        return finish(std::move(result));
    }

    result.accepted = true;
    const State nominal_terminal = evaluateCubicState(
        candidate.control_points, candidate.knots, candidate.valid_until);
    result.committed_endpoint_distance_m = result.target_time_prediction_valid
        ? (nominal_terminal.position - result.predicted_target_position).norm()
        : (result.moving_target_terminal_active
            ? (nominal_terminal.position - global_goal_).norm()
            : (committed_.endState().position - global_goal_).norm());
    if (result.moving_rendezvous_active) {
        // Mission-level bridge feasibility, not this planner, owns the final
        // authority transition. Keep replanning rather than latching goal_seen_.
        if (result.candidate_kind == CandidateKind::Partial) {
            result.status = result.recheck_retry_performed
                ? "ACCEPTED_AFTER_RECHECK_PARTIAL_MOVING_RENDEZVOUS"
                : "ACCEPTED_PARTIAL_MOVING_RENDEZVOUS";
        } else {
            result.status = result.recheck_retry_performed
                ? "ACCEPTED_AFTER_RECHECK_MOVING_RENDEZVOUS"
                : "ACCEPTED_MOVING_RENDEZVOUS";
        }
    } else if (result.moving_target_terminal_active) {
        result.status = result.recheck_retry_performed
            ? "ACCEPTED_AFTER_RECHECK_MOVING_TARGET_PURSUIT"
            : "ACCEPTED_MOVING_TARGET_PURSUIT";
    } else if (!result.local_continuation_active &&
               result.committed_endpoint_distance_m <= config_.goal_tolerance_m) {
        // A continuation candidate may pass through the mission goal region while
        // still carrying an appended brake-to-hover safety tail. Do not freeze
        // replanning on the nominal/local continuation endpoint: the committed
        // trajectory that will actually execute can stop materially away from the
        // mission goal. Only a non-continuation terminal candidate may latch
        // goal_seen_.
        goal_seen_ = true;
        result.status = result.recheck_retry_performed
            ? "ACCEPTED_AFTER_RECHECK_GOAL_SEEN"
            : "ACCEPTED_GOAL_SEEN";
    } else if (result.candidate_kind == CandidateKind::Partial) {
        result.status = result.recheck_retry_performed
            ? "ACCEPTED_AFTER_RECHECK_PARTIAL"
            : "ACCEPTED_PARTIAL";
    } else {
        result.status = result.recheck_retry_performed
            ? "ACCEPTED_AFTER_RECHECK"
            : "ACCEPTED";
    }
    return finish(std::move(result));
}

}  // namespace dynamic_planner
