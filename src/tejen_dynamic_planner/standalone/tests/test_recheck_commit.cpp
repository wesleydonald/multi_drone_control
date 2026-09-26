#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/trajectory_geometry.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <optional>
#include <vector>

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

class ScriptedWorld final : public WorldSnapshotSource {
public:
    std::vector<WorldSnapshot> snapshots;
    int forced_commit_mismatches = 0;

    WorldSnapshot snapshot(double captured_at_s) const override {
        if (snapshots.empty()) {
            throw std::runtime_error("scripted world has no snapshots");
        }
        const std::size_t index = std::min(snapshot_calls_, snapshots.size() - 1U);
        ++snapshot_calls_;
        WorldSnapshot value = snapshots[index];
        value.captured_at_s = captured_at_s;
        return value;
    }

    std::uint64_t currentVersion() const override {
        const std::size_t index = std::min(snapshot_calls_, snapshots.size() - 1U);
        return snapshots[index].version;
    }

    bool runIfVersionCurrent(
        std::uint64_t expected_version,
        const std::function<void()>& action) const override {
        ++commit_calls_;
        if (commit_calls_ <= forced_commit_mismatches) {
            return false;
        }
        const std::size_t index = std::min(snapshot_calls_ - 1U, snapshots.size() - 1U);
        if (snapshots[index].version != expected_version) {
            return false;
        }
        action();
        return true;
    }

private:
    mutable std::size_t snapshot_calls_ = 0;
    mutable int commit_calls_ = 0;
};

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

RecedingHorizonConfig makeConfig() {
    RecedingHorizonConfig config;
    config.continuity_tolerance = 1e-7;
    config.spline_time_factor = 2.5;
    return config;
}

WorldSnapshot emptySnapshot(std::uint64_t version) {
    WorldSnapshot snapshot;
    snapshot.version = version;
    snapshot.captured_at_s = 0.0;
    return snapshot;
}

WorldSnapshot blockedSnapshot(std::uint64_t version, const Vec3& center,
                              const Vec3& half) {
    WorldSnapshot snapshot = emptySnapshot(version);
    snapshot.static_obstacles.push_back(StaticConvexObstacle{
        "new_blocker", boxVertices(center, half)});
    return snapshot;
}

WorldSnapshot attachmentSnapshot(std::uint64_t version, bool attached) {
    WorldSnapshot snapshot = emptySnapshot(version);
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.payload_attached = attached;
    // Synthetic lateral pickup offset chosen so the physical blocker intersects
    // only the newly attached payload envelope, not body/cable/magnet.
    geometry.payload_center_from_magnet_center = Vec3(0.0, 0.25, -0.030);
    geometry.payload_half_extents = Vec3(0.05, 0.02, 0.01);
    snapshot.ego_suspended_geometry = geometry;
    snapshot.physical_static_obstacles.push_back(StaticConvexObstacle{
        "payload_only_blocker",
        boxVertices(Vec3(0.55, 0.25, 0.94), Vec3(0.08, 0.01, 0.03))});
    return snapshot;
}

}  // namespace

int main() {
    try {
        State start;
        start.position = Vec3(0.0, 0.0, 1.5);
        const Vec3 goal(3.0, 0.0, 1.5);

        // One irrelevant update between first recheck and commit: do one
        // bounded final recheck, then commit the still-safe candidate.
        {
            ScriptedWorld world;
            world.snapshots = {
                emptySnapshot(0),  // planning snapshot
                emptySnapshot(0),  // first newest-world recheck
                blockedSnapshot(1, Vec3(0.0, 1.5, 1.5), Vec3(0.1, 0.1, 0.1))
            };
            world.forced_commit_mismatches = 1;
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            const auto result = planner.replan(0.0, start, world);
            requireTrue(result.accepted, "irrelevant version update prevented safe commit: " + result.status);
            requireTrue(result.recheck_retry_performed,
                        "world-version mismatch did not trigger bounded retry");
            requireTrue(result.status == "ACCEPTED_AFTER_RECHECK",
                        "unexpected retry-accept status: " + result.status);
            requireTrue(result.final_recheck_world_version == 1U,
                        "final recheck did not use updated world version");
        }

        // New world information intersects the candidate. It must be rejected
        // and an empty incumbent must remain empty.
        {
            ScriptedWorld world;
            world.snapshots = {
                emptySnapshot(0),
                emptySnapshot(0),
                blockedSnapshot(1, Vec3(0.45, 0.0, 1.5), Vec3(0.30, 0.25, 0.25))
            };
            world.forced_commit_mismatches = 1;
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            const auto result = planner.replan(0.0, start, world);
            requireTrue(!result.accepted, "candidate colliding with updated world was committed");
            requireTrue(result.status == "LATEST_CANDIDATE_UNSAFE_AFTER_RETRY",
                        "unexpected stale-candidate rejection: " + result.status);
            requireTrue(planner.committedTrajectory().empty(),
                        "rejected initial candidate mutated authority");
        }


        // B.4 attachment is authoritative world state. A candidate planned and
        // first-rechecked while unattached must not gain authority if a payload
        // attaches before commit and makes that candidate unsafe.
        {
            ScriptedWorld world;
            world.snapshots = {
                attachmentSnapshot(0, false),
                attachmentSnapshot(0, false),
                attachmentSnapshot(1, true)
            };
            world.forced_commit_mismatches = 1;
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            const auto result = planner.replan(0.0, start, world);
            requireTrue(!result.accepted,
                        "pre-attachment candidate survived payload attachment race");
            requireTrue(result.recheck_retry_performed,
                        "attachment race did not trigger bounded newest-snapshot retry");
            requireTrue(result.status == "LATEST_CANDIDATE_UNSAFE_AFTER_RETRY",
                        "unexpected attachment-race status: " + result.status);
            requireTrue(result.candidate_safety.has_value() &&
                            result.candidate_safety->obstacle_name.find(
                                "payload_only_blocker::ego_payload") != std::string::npos,
                        "attachment recheck did not identify payload collision");
            requireTrue(planner.committedTrajectory().empty(),
                        "attachment-race rejection mutated authority");
        }

        // Prefix becomes unsafe before A. This is not allowed to fall through
        // to normal incumbent continuation.
        {
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            VersionedWorld bootstrap_world;
            const auto initial = planner.replan(0.0, start, bootstrap_world);
            requireTrue(initial.accepted, "prefix test bootstrap failed");
            const std::size_t pieces_before = planner.committedTrajectory().pieceCount();
            const double end_before = planner.committedTrajectory().endTime();
            const State end_state_before = planner.committedTrajectory().endState();

            const double now = 1.0 / 30.0;
            const State state = planner.committedTrajectory().evaluate(now);
            ScriptedWorld world;
            world.snapshots = {
                emptySnapshot(0),
                blockedSnapshot(1, Vec3(0.02, 0.0, 1.5), Vec3(0.08, 0.20, 0.20))
            };
            const auto result = planner.replan(now, state, world);
            requireTrue(!result.accepted, "unsafe incumbent prefix was allowed to commit");
            requireTrue(result.incumbent_prefix_unsafe,
                        "unsafe-prefix flag was not set");
            requireTrue(result.status == "INCUMBENT_PREFIX_UNSAFE",
                        "unexpected unsafe-prefix status: " + result.status);
            requireTrue(planner.committedTrajectory().pieceCount() == pieces_before,
                        "unsafe-prefix rejection mutated piece count");
            requireNear(planner.committedTrajectory().endTime(), end_before, 1e-12,
                        "unsafe-prefix rejection mutated end time");
            requireMatrixNear(planner.committedTrajectory().endState().position,
                              end_state_before.position, 1e-12,
                              "unsafe-prefix rejection mutated endpoint");
        }

        // C1F.4 regression: if local planning cannot produce a replacement,
        // the remaining incumbent must still be revalidated against the latest
        // world. A newly occupied incumbent may not silently continue merely
        // because Octopus failed first.
        {
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            VersionedWorld bootstrap_world;
            const auto initial = planner.replan(0.0, start, bootstrap_world);
            requireTrue(initial.accepted, "failure-recheck bootstrap failed");

            const double now = 0.20;
            const State state = planner.committedTrajectory().evaluate(now);
            ScriptedWorld world;
            // Fully occupy the local planning bounds so replacement failure is
            // geometric and deterministic. The former narrow x-wall could be
            // routed around once C1F.8 enabled compiler optimisation, making
            // this regression accidentally depend on solver speed.
            const Vec3 local_blocker_center(1.5, 0.0, 1.5);
            const Vec3 local_blocker_half(3.0, 3.0, 2.0);
            world.snapshots = {
                blockedSnapshot(1, local_blocker_center, local_blocker_half),
                blockedSnapshot(1, local_blocker_center, local_blocker_half)
            };
            const auto result = planner.replan(now, state, world);
            requireTrue(!result.accepted,
                        "blocked local-plan failure unexpectedly committed");
            requireTrue(result.incumbent_prefix_unsafe,
                        "failed replacement did not flag unsafe remaining incumbent");
            requireTrue(result.prefix_safety.has_value() &&
                        !result.prefix_safety->safe,
                        "failed replacement did not record incumbent collision");
            requireTrue(result.incumbent_recheck_performed,
                        "failed replacement did not expose its incumbent recheck certificate");
            requireTrue(result.incumbent_recheck_world_version == 1U,
                        "failed replacement exposed the wrong incumbent recheck world version");
        }

        // C1F.7 policy split: a far-future incumbent collision remains an
        // immediate unsafe-incumbent result for the conservative/default
        // profile, but simulation_cage may continue the incumbent while that
        // collision is still outside the actual cubic braking/reaction horizon.
        //
        // The obstacle is introduced only AFTER planning. That guarantees this
        // regression exercises newest-world candidate rejection followed by
        // incumbent revalidation, rather than letting Octopus avoid the wall
        // during the replacement solve. Its location is derived from the
        // incumbent itself so future changes to spline timing/planning radius
        // cannot silently move the fixture beyond the recheck horizon.
        {
            auto bootstrap = [&](RecedingHorizonConfig cfg) {
                return RecedingHorizonPlanner(goal, cfg, makeLocalConfig());
            };

            constexpr double now = 0.20;
            const Vec3 wall_half(0.01, 10.0, 10.0);
            Vec3 wall_center = Vec3::Zero();

            // Conservative/default: preserve the old immediate-unsafety contract.
            {
                const auto cfg = makeConfig();
                auto planner = bootstrap(cfg);
                VersionedWorld bootstrap_world;
                const auto initial = planner.replan(0.0, start, bootstrap_world);
                requireTrue(initial.accepted, "conservative far-conflict bootstrap failed");
                const State state = planner.committedTrajectory().evaluate(now);

                const double brake_time_s =
                    (2.0 * state.velocity.cwiseAbs().array() / cfg.a_max.array()).maxCoeff();
                const double fallback_horizon_s =
                    std::max(cfg.emergency_panic_horizon_s, brake_time_s + cfg.dc_s);
                const double check_until_s = std::min(
                    planner.committedTrajectory().endTime(),
                    now + cfg.incumbent_failure_recheck_horizon_s);
                const auto intervals = dynamic_planner::convexCurveIntervals(
                    planner.committedTrajectory(), now, check_until_s);

                std::optional<std::pair<double, double>> conflict_interval;
                for (const auto& interval : intervals) {
                    if (interval.start_s - now > fallback_horizon_s + 0.20) {
                        conflict_interval = std::make_pair(interval.start_s, interval.end_s);
                        break;
                    }
                }
                requireTrue(conflict_interval.has_value(),
                            "far-conflict fixture could not find an incumbent interval "
                            "outside the braking horizon");
                const double wall_time_s =
                    0.5 * (conflict_interval->first + conflict_interval->second);
                wall_center = planner.committedTrajectory().evaluate(wall_time_s).position;

                ScriptedWorld world;
                world.snapshots = {
                    emptySnapshot(0),
                    blockedSnapshot(1, wall_center, wall_half),
                    blockedSnapshot(1, wall_center, wall_half)
                };
                const auto result = planner.replan(
                    now, state, world, []() { return now; });
                requireTrue(!result.accepted,
                            "newly blocked candidate unexpectedly committed");
                requireTrue(result.candidate_safety.has_value() &&
                                !result.candidate_safety->safe,
                            "newest-world wall did not invalidate the replacement candidate");
                requireTrue(result.prefix_safety.has_value() && !result.prefix_safety->safe,
                            "conservative far-conflict fixture did not intersect incumbent");
                requireTrue(result.incumbent_prefix_unsafe,
                            "conservative profile deferred a newly unsafe incumbent");
                requireTrue(result.incumbent_time_to_conflict_s >
                                result.fallback_trigger_horizon_s,
                            "conservative fixture was not actually a far-future conflict");
                requireTrue(result.incumbent_recheck_performed &&
                                result.incumbent_recheck_world_version == 1U,
                            "conservative far-conflict result lost its same-world recheck certificate");
            }

            // Simulation/cage: the identical newest-world conflict is allowed
            // to remain a replanning problem while TTC exceeds the actual brake
            // horizon. Only this policy bit differs from the conservative case.
            {
                auto cfg = makeConfig();
                cfg.defer_incumbent_conflict_until_reaction_horizon = true;
                auto planner = bootstrap(cfg);
                VersionedWorld bootstrap_world;
                const auto initial = planner.replan(0.0, start, bootstrap_world);
                requireTrue(initial.accepted, "simulation far-conflict bootstrap failed");
                const State state = planner.committedTrajectory().evaluate(now);

                ScriptedWorld world;
                world.snapshots = {
                    emptySnapshot(0),
                    blockedSnapshot(1, wall_center, wall_half),
                    blockedSnapshot(1, wall_center, wall_half)
                };
                const auto result = planner.replan(
                    now, state, world, []() { return now; });
                requireTrue(!result.accepted,
                            "simulation newly blocked candidate unexpectedly committed");
                requireTrue(result.candidate_safety.has_value() &&
                                !result.candidate_safety->safe,
                            "simulation newest-world wall did not invalidate replacement");
                requireTrue(result.prefix_safety.has_value() && !result.prefix_safety->safe,
                            "simulation far-conflict fixture did not intersect incumbent");
                requireTrue(!result.incumbent_prefix_unsafe,
                            "simulation profile escalated a far-future conflict immediately");
                requireTrue(result.candidate_kind == CandidateKind::Incumbent,
                            "deferred conflict did not identify incumbent continuation");
                requireTrue(result.incumbent_time_to_conflict_s >
                                result.fallback_trigger_horizon_s,
                            "far-conflict fixture was not outside the braking horizon");
                requireTrue(result.incumbent_recheck_performed &&
                                result.incumbent_recheck_world_version == 1U,
                            "simulation far-conflict result lost its same-world recheck certificate");
            }
        }

        // If the world changes again after the one permitted retry, terminate
        // the transaction without authority mutation. No retry loop.
        {
            ScriptedWorld world;
            world.snapshots = {
                emptySnapshot(0),
                emptySnapshot(0),
                emptySnapshot(1)
            };
            world.forced_commit_mismatches = 2;
            RecedingHorizonPlanner planner(goal, makeConfig(), makeLocalConfig());
            const auto result = planner.replan(0.0, start, world);
            requireTrue(!result.accepted, "second world race was incorrectly committed");
            requireTrue(result.recheck_retry_performed,
                        "second-race test never used the single retry");
            requireTrue(result.status == "WORLD_CHANGED_DURING_FINAL_RECHECK",
                        "unexpected second-race status: " + result.status);
            requireTrue(planner.committedTrajectory().empty(),
                        "second-race rejection mutated authority");
        }

        std::cout << "test_recheck_commit: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_recheck_commit: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
