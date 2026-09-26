#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>

#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/frozen_world.hpp"
#include "dynamic_planner/local_planner.hpp"
#include "dynamic_planner/minimum_time.hpp"
#include "dynamic_planner/target_predictor.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/world_snapshot.hpp"

namespace dynamic_planner {

struct RecedingHorizonConfig {
    double dc_s = 1.0 / 30.0;
    double planning_radius_m = 2.0;
    double factor_alpha = 2.5;
    double min_splice_lookahead_s = 0.05;
    double max_splice_lookahead_s = 1.0;
    // Fast real-time profiles may use a fixed future splice instead of the
    // legacy runtime-scaled lookahead. Zero preserves the adaptive behavior.
    double fixed_splice_lookahead_s = 0.0;
    double factor_alloc = 1.0;
    double factor_alloc_close = 2.5;
    // Project-specific representation/search floor. RMADER's double-integrator
    // time remains the lower-bound calculation, but our four-segment cubic
    // stopping spline plus discrete Octopus search needs additional headroom.
    double spline_time_factor = 2.5;
    double close_to_goal_m = 0.20;
    double goal_tolerance_m = 0.05;
    // Numerical C2 acceptance gate. The spline boundary is still constructed
    // from the exact incumbent p/v/a at A; this tolerance only admits floating-
    // point/solver residue from repeated reparameterization.
    double continuity_tolerance = 1e-7;
    double separator_validation_tolerance = 1e-7;
    // C.1b shadow solves are intentionally rebuilt from the measured hover so
    // they never advance a fictional incumbent. Carry the most recent elapsed
    // TRAJECTORY time into each fresh planner so future-splice lookahead adapts
    // correctly even when Gazebo real-time factor is not 1. CPU runtime remains
    // a separate steady-clock diagnostic. Default zero preserves B.1-B.4.
    double initial_splice_timing_s = 0.0;
    // Optional execution-aware Octopus cutoff. Disabled by default so the
    // validated generic/cooperative planner retains its historical behavior.
    // Commissioning enables this with a measured post-search certification
    // reserve, leaving time for QP refinement, continuous safety checks, and
    // the version-gated authority decision before the future splice.
    bool enable_octopus_useful_deadline = false;
    double octopus_post_search_reserve_s = 0.0;
    // When a replacement cannot be committed, the incumbent remains the
    // executable authority. Revalidate at least this much of its future
    // stopped-hold extension against the newest dynamic world.
    double incumbent_failure_recheck_horizon_s = 2.0;
    // C1F.5 finite-horizon viability guard: a candidate may only become an
    // incumbent if its stopped endpoint remains collision-free for this long
    // after the explicit local spline ends. This is deliberately separate from
    // incumbent_failure_recheck_horizon_s; it is not an invariant-set claim.
    double terminal_hold_guard_s = 0.0;
    double terminal_hold_guard_interval_s = 0.50;
    // Conservative/IRL profiles require a finite stopped hold after every
    // nominal candidate.  The simulation/cage profile deliberately separates
    // that emergency proof from nominal dynamic avoidance: it still checks the
    // candidate that will be executed, but reserves braking/hover for a real
    // incumbent conflict.
    bool require_terminal_hold_for_nominal = true;
    // C1F.6 moving-rendezvous mode. It is only applied when the projected local
    // goal is the actual moving target (not an intermediate planning-radius waypoint).
    bool moving_rendezvous_enabled = false;
    Vec3 moving_rendezvous_terminal_velocity = Vec3::Zero();
    // C1F.8b: when a time-parametric target model is supplied to replan(), the
    // terminal target state is evaluated at the candidate's own absolute end
    // time. This removes the older backend-side pre-lead heuristic.
    bool terminal_time_target_prediction_enabled = false;
    int target_time_fixed_point_iterations = 6;
    double target_time_fixed_point_tolerance_s = 0.01;
    double target_prediction_low_confidence_horizon_s = 2.0;
    double target_prediction_high_confidence_horizon_s = 10.0;
    double target_prediction_low_confidence_max_distance_m = 0.15;
    Vec3 rendezvous_backup_brake_accel_limit = Vec3(1.0, 1.0, 1.5);
    double rendezvous_backup_min_duration_s = 0.25;
    // Every accepted moving-rendezvous commitment still appends a velocity-
    // continuous brake-to-stopped-hold tail so the rolling MPC reference has
    // an explicit future beyond the nonzero-velocity rendezvous endpoint.
    // This flag controls whether that contingency tail (and its optional hold)
    // is also a hard collision-safety acceptance condition. The simulation/cage
    // profile may set it false without removing the reference-continuity tail.
    bool require_rendezvous_backup_for_nominal = true;
    // Conservative/default behavior treats any newly discovered incumbent
    // collision as immediately unsafe. The simulation/cage profile may defer
    // escalation while the first predicted conflict remains outside the actual
    // state-dependent braking/reaction horizon, giving the next 5 Hz cycle a
    // chance to install a complete or evasive-partial replacement.
    bool defer_incumbent_conflict_until_reaction_horizon = false;
    // If an incumbent has a predicted conflict and no replacement is available
    // inside this state-dependent horizon, the caller installs the existing
    // measured-state emergency fallback. 0.50 s is only the panic floor; the
    // actual trigger also includes the duration of the cubic brake.
    double emergency_panic_horizon_s = 0.50;
    Vec3 v_max = Vec3::Ones();
    Vec3 a_max = Vec3::Constant(1.5);
};

enum class CandidateKind {
    None,
    Complete,
    Partial,
    Incumbent,
    Fallback,
};

const char* candidateKindName(CandidateKind kind) noexcept;

using TrajectoryTimeSource = std::function<double()>;

struct ReplanResult {
    bool attempted = false;
    bool accepted = false;
    CandidateKind candidate_kind = CandidateKind::None;
    bool goal_seen = false;
    bool goal_reached = false;
    std::string status;
    double now_s = 0.0;
    double splice_time_s = 0.0;
    double splice_lookahead_s = 0.0;
    State splice_state;
    Vec3 local_goal = Vec3::Zero();
    double local_duration_s = 0.0;
    double minimum_time_s = 0.0;
    double time_allocation_factor = 0.0;
    double global_distance_from_splice_m = 0.0;
    double committed_endpoint_distance_m = 0.0;
    bool moving_target_terminal_active = false;
    // True when the local waypoint is an intermediate receding-horizon target.
    // Its terminal position is hard-constrained but terminal velocity is free.
    bool local_continuation_active = false;
    bool moving_rendezvous_active = false;
    Vec3 terminal_target_velocity = Vec3::Zero();
    bool target_time_prediction_used = false;
    bool target_time_prediction_high_confidence = false;
    bool target_time_prediction_valid = false;
    int target_time_fixed_point_iterations = 0;
    double target_time_fixed_point_residual_s = std::numeric_limits<double>::quiet_NaN();
    double predicted_target_time_s = std::numeric_limits<double>::quiet_NaN();
    double predicted_target_horizon_s = std::numeric_limits<double>::quiet_NaN();
    Vec3 predicted_target_position = Vec3::Zero();
    Vec3 predicted_target_velocity = Vec3::Zero();
    double terminal_velocity_error_mps = std::numeric_limits<double>::quiet_NaN();
    double rendezvous_backup_duration_s = 0.0;
    double incumbent_time_to_conflict_s = std::numeric_limits<double>::quiet_NaN();
    double estimated_brake_time_s = 0.0;
    double fallback_trigger_horizon_s = 0.0;
    bool incumbent_reaction_horizon_reached = false;
    double replan_runtime_s = 0.0;
    // C1F.5p performance attribution. These are steady-clock wall durations
    // and are diagnostics only; none changes trajectory-time deadline logic.
    double world_snapshot_s = 0.0;
    double obstacle_build_s = 0.0;
    double safety_check_s = 0.0;
    double atomic_commit_s = 0.0;
    // Elapsed trajectory/world time during this planning attempt. With the
    // legacy clock this equals steady runtime approximately; with ROS /clock it
    // follows simulation time and is the quantity used to predict splice lead.
    double trajectory_elapsed_s = 0.0;
    double candidate_finish_time_s = 0.0;
    double authority_decision_time_s = 0.0;
    bool candidate_late = false;
    bool octopus_useful_deadline_enabled = false;
    double octopus_post_search_reserve_s = 0.0;
    double octopus_useful_deadline_time_s = 0.0;
    double octopus_useful_budget_s = 0.0;
    bool octopus_deadline_clock_invalid = false;
    SpliceDiagnostics splice_diagnostics;
    std::optional<LocalPlanResult> local_plan;

    // B.2 Check/Recheck diagnostics.
    std::uint64_t planning_world_version = 0;
    std::uint64_t recheck_world_version = 0;
    std::uint64_t final_recheck_world_version = 0;
    std::string recheck_error;
    bool recheck_retry_performed = false;
    // C1F.8: explicit certificate that a failed replacement caused the core
    // planner to revalidate the still-executing incumbent against this exact
    // VersionedWorld snapshot. The ROS wrapper may reuse this result only when
    // its current world version is identical.
    bool incumbent_recheck_performed = false;
    std::uint64_t incumbent_recheck_world_version = 0;
    bool incumbent_prefix_unsafe = false;
    std::optional<TrajectorySafetyResult> prefix_safety;
    std::optional<TrajectorySafetyResult> candidate_safety;
    std::optional<TrajectorySafetyResult> candidate_terminal_hold_safety;
    std::optional<TrajectorySafetyResult> candidate_backup_safety;
};

class RecedingHorizonPlanner {
public:
    RecedingHorizonPlanner(Vec3 global_goal,
                           RecedingHorizonConfig config,
                           LocalPlannerConfig local_config);

    // B.2 authoritative path. Planning uses one immutable snapshot, then the
    // completed candidate and the still-to-execute incumbent prefix are checked
    // against the newest snapshot before an atomic version-gated commit.
    ReplanResult replan(double now_s,
                        const State& measured_state,
                        const WorldSnapshotSource& world);

    // C.1b/C.1c ROS integration path. Trajectory/world timestamps come from
    // the supplied time source (normally ROS time when use_sim_time=true),
    // while replan_runtime_s remains measured with std::chrono::steady_clock.
    // This avoids treating 100 ms of wall computation as 100 ms of simulated
    // trajectory time when Gazebo is running above/below real-time factor 1.
    ReplanResult replan(double now_s,
                        const State& measured_state,
                        const WorldSnapshotSource& world,
                        const TrajectoryTimeSource& trajectory_now_s,
                        const TargetPredictor* target_predictor = nullptr);

    // Backward-compatible B.1/static-world adapter used by the validated
    // regression/demo scenes. Static vertices are already ego C-space inflated.
    ReplanResult replan(double now_s,
                        const State& measured_state,
                        const FrozenWorld& world);

    bool goalSeen() const noexcept { return goal_seen_; }
    bool goalReached(const State& state) const;

    // Seed authority with a pre-certified incumbent before the first moving
    // solve. C.1c uses makeStationaryHoverTrajectory() so even an overrun on
    // the first solve leaves an executable hover rather than a trajectory that
    // began in the past. May only be called before any incumbent/goal latch.
    void initializeCommittedTrajectory(CommittedTrajectory committed);
    const Vec3& globalGoal() const noexcept { return global_goal_; }
    const CommittedTrajectory& committedTrajectory() const noexcept { return committed_; }
    double previousReplanRuntime() const noexcept { return previous_replan_runtime_s_; }
    double previousSpliceTiming() const noexcept { return previous_splice_timing_s_; }

private:
    double chooseSpliceLookahead() const;
    Vec3 projectLocalGoal(const Vec3& from) const;
    double allocateLocalDuration(const State& start,
                                 const Vec3& local_goal,
                                 const Vec3& terminal_velocity,
                                 double distance_for_allocation,
                                 double* minimum_time,
                                 double* applied_factor) const;
    void validateConfig() const;

    Vec3 global_goal_ = Vec3::Zero();
    RecedingHorizonConfig config_;
    LocalPlanner local_planner_;
    TrajectorySafetyChecker safety_checker_;
    CommittedTrajectory committed_;
    double previous_replan_runtime_s_ = 0.0;
    double previous_splice_timing_s_ = 0.0;
    bool goal_seen_ = false;
};

}  // namespace dynamic_planner
