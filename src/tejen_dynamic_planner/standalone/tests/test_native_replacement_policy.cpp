#include "dynamic_planner/receding_horizon_planner.hpp"
#include "test_common.hpp"
#include <iostream>
#include <type_traits>

using namespace dynamic_planner;

// Lets this exact fixture reproduce the defect against preserved PB.58 headers
// and libraries. In the new API there is no such member or acceptance option.
template <typename Config, typename = void>
struct LegacyProgressSwitch {
    static void enable(Config&) {}
};
template <typename Config>
struct LegacyProgressSwitch<Config,
    std::void_t<decltype(std::declval<Config>().minimum_endpoint_progress_m)>> {
    static void enable(Config& config) { config.minimum_endpoint_progress_m = .02; }
};

int main() {
    try {
        State start{Vec3(0,0,1.5), Vec3::Zero(), Vec3::Zero()};
        LocalPlannerConfig local;
        local.octopus.max_runtime_s = .05;
        local.octopus.xyz_min = Vec3(-1,-2,.5);
        local.octopus.xyz_max = Vec3(4,2,2.5);
        local.refinement.xyz_min = local.octopus.xyz_min;
        local.refinement.xyz_max = local.octopus.xyz_max;
        RecedingHorizonConfig config;
        config.fixed_splice_lookahead_s = .1;
        config.spline_time_factor = 3.;
        VersionedWorld world;
        RecedingHorizonPlanner seed(Vec3(1,0,1.5), config, local);
        const auto initial = seed.replan(0., start, world, [] { return 0.; });
        requireTrue(initial.accepted, "native seed solve failed: " + initial.status);
        const auto incumbent = seed.committedTrajectory();
        const Vec3 goal = incumbent.endState().position;
        config.moving_rendezvous_enabled = true;
        config.terminal_time_target_prediction_enabled = true;
        config.require_terminal_hold_for_nominal = false;
        LegacyProgressSwitch<RecedingHorizonConfig>::enable(config);
        CommittedTrajectoryTargetPredictor stationary(
            makeStationaryHoverTrajectory(goal, 0., 180.), Vec3::Zero());
        for (double now : {.2, .3, .4, .6}) {
            RecedingHorizonPlanner planner(goal, config, local);
            planner.initializeCommittedTrajectory(incumbent);
            const auto before = incumbent.evaluate(now + .1);
            const auto result = planner.replan(now, incumbent.evaluate(now), world,
                [now] { return now; }, &stationary);
            requireTrue(result.target_time_prediction_valid,
                        "actual stationary provider path was not exercised");
            requireTrue(result.accepted,
                        "extra endpoint-progress policy rejected native replacement: " + result.status);
            requireTrue(result.candidate_safety && result.candidate_safety->safe,
                        "native candidate safety check omitted");
            requireTrue(result.splice_diagnostics.accepted && result.certified_world_version.has_value(),
                        "native continuity/version acceptance bypassed");
            const auto after = planner.committedTrajectory().evaluate(result.splice_time_s);
            requireMatrixNear(after.position, before.position, 1e-7, "position discontinuity");
            requireMatrixNear(after.velocity, before.velocity, 1e-7, "velocity discontinuity");
            requireMatrixNear(after.acceleration, before.acceleration, 1e-7, "acceleration discontinuity");
        }
        std::cout << "test_native_replacement_policy: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_native_replacement_policy: FAIL: " << error.what() << '\n';
        return 1;
    }
}
