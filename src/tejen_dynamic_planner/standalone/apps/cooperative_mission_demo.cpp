#include "dynamic_planner/receding_horizon_planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace dynamic_planner;

namespace {

constexpr double kExecutionDt = 1.0 / 30.0;
const Vec3 kPlanningDroneHalfExtents(0.105, 0.105, 0.060);
const Vec3 kOtherDroneHalfExtents(0.105, 0.105, 0.060);
const Vec3 kTrackingHalfExtents = Vec3::Constant(0.05);

struct OtherDroneSpec {
    std::string name;
    Vec3 start = Vec3::Zero();
    Vec3 end = Vec3::Zero();
    double motion_end_s = 0.0;
};

struct MissionScenario {
    std::string name;
    State planning_start;
    Vec3 planning_goal = Vec3::Zero();
    Vec3 bounds_min = Vec3::Zero();
    Vec3 bounds_max = Vec3::Zero();
    std::vector<OtherDroneSpec> other_drones;
    // Stress scenarios must be demonstrably unsafe for the same planner run in
    // an otherwise empty world. This prevents a nominally "dynamic" test from
    // passing simply because the other-drone timing never created a conflict.
    bool require_unobstructed_conflict = false;
};

struct OtherDroneModel {
    std::string name;
    CommittedTrajectory trajectory;
    Vec3 physical_half_extents = kOtherDroneHalfExtents;
    Vec3 tracking_error_half_extents = kTrackingHalfExtents;
};

struct RuntimeStats {
    double mean = 0.0;
    double median = 0.0;
    double p95 = 0.0;
    double maximum = 0.0;
};

State stoppedState(const Vec3& position) {
    State state;
    state.position = position;
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    return state;
}

MissionScenario makeScenario(const std::string& name) {
    MissionScenario scenario;
    scenario.name = name;
    scenario.planning_start = stoppedState(Vec3(0.0, 0.0, 1.5));
    scenario.planning_goal = Vec3(3.0, 0.0, 1.5);
    scenario.bounds_min = Vec3(-1.0, -2.2, 0.4);
    scenario.bounds_max = Vec3(4.5, 2.2, 2.6);

    if (name == "crossing") {
        // Nominal unimpeded mission is ~9.3 s in the validated B.2 harness. The
        // other drone therefore crosses x=1.5, y=0 near the nominal midpoint.
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(1.5, -1.1, 1.5),
            Vec3(1.5, 1.1, 1.5),
            9.3});
    } else if (name == "same_direction") {
        // Starts safely ahead, then proceeds in +x more slowly than the planning
        // drone's unobstructed progress. This produces a sustained time-varying
        // obstruction rather than a brief crossing event.
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(0.85, 0.0, 1.5),
            Vec3(3.70, 0.0, 1.5),
            12.0});
    } else if (name == "two_drones") {
        // Two simple, separated perpendicular crossings. The first conflict is
        // earlier and the second later, so the mission exercises multiple fixed
        // shared commitments without creating an artificial static maze.
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(1.05, -1.1, 1.5),
            Vec3(1.05, 1.1, 1.5),
            7.0});
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_2",
            Vec3(2.05, 1.1, 1.5),
            Vec3(2.05, -1.1, 1.5),
            13.0});
    } else if (name == "crossing_stress") {
        // Deliberately difficult perpendicular crossing. The other drone moves
        // slowly through the direct x-axis route: its centre remains inside the
        // +/-0.26 m y C-space band for a broad interval around the time the
        // validated unobstructed mission normally reaches x=1.5. A straight
        // nominal mission should therefore collide and the planner must change
        // space and/or time to remain safe.
        scenario.require_unobstructed_conflict = true;
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(1.50, -0.40, 1.5),
            Vec3(1.50, 1.00, 1.5),
            16.0});
    } else if (name == "same_direction_stress") {
        // Starts only 0.55 m ahead on the same centreline and progresses much
        // more slowly. The unobstructed planning trajectory should catch its
        // C-space tube, forcing a safe follow/slow-down or a spatial overtake.
        // The endpoint is beyond the planning goal so its final stopped hold
        // does not permanently make the goal infeasible.
        scenario.require_unobstructed_conflict = true;
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(0.55, 0.0, 1.5),
            Vec3(3.65, 0.0, 1.5),
            16.0});
    } else if (name == "two_drones_stress") {
        // Two slow opposing crossings keep the nominal x-axis blocked at two
        // separated locations for overlapping time windows. This is intended to
        // require repeated meaningful avoidance decisions rather than merely
        // exploiting a convenient pre-existing temporal gap.
        scenario.require_unobstructed_conflict = true;
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_1",
            Vec3(1.00, -0.30, 1.5),
            Vec3(1.00, 1.20, 1.5),
            18.0});
        scenario.other_drones.push_back(OtherDroneSpec{
            "other_drone_2",
            Vec3(2.00, 0.30, 1.5),
            Vec3(2.00, -1.20, 1.5),
            18.0});
    } else {
        throw std::invalid_argument(
            "--scenario must be crossing, same_direction, two_drones, "
            "crossing_stress, same_direction_stress, or two_drones_stress");
    }
    return scenario;
}

CommittedTrajectory makeRestToRestOtherDroneTrajectory(
    const Vec3& start,
    const Vec3& end,
    double start_time_s,
    double end_time_s) {
    if (!start.allFinite() || !end.allFinite() || !std::isfinite(start_time_s) ||
        !std::isfinite(end_time_s) || !(end_time_s > start_time_s)) {
        throw std::invalid_argument("invalid other-drone trajectory request");
    }

    // A three-span clamped cubic B-spline has six control points. Repeating the
    // first and last positions three times gives exact zero velocity and zero
    // acceleration at both endpoints while retaining C2 continuity internally.
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(start_time_s, end_time_s, 3);
    piece.control_points.resize(6, 3);
    piece.control_points.row(0) = start.transpose();
    piece.control_points.row(1) = start.transpose();
    piece.control_points.row(2) = start.transpose();
    piece.control_points.row(3) = end.transpose();
    piece.control_points.row(4) = end.transpose();
    piece.control_points.row(5) = end.transpose();
    piece.valid_from = start_time_s;
    piece.valid_until = end_time_s;
    piece.validate();

    CommittedTrajectory trajectory;
    const SpliceDiagnostics inserted = trajectory.replaceSuffix(start_time_s, piece);
    if (!inserted.accepted || !trajectory.endsInStoppedHold(1e-9)) {
        throw std::runtime_error("failed to build stopped other-drone trajectory");
    }

    const State first = trajectory.evaluate(start_time_s);
    const State last = trajectory.evaluate(end_time_s);
    if ((first.position - start).norm() > 1e-10 ||
        first.velocity.norm() > 1e-10 || first.acceleration.norm() > 1e-9 ||
        (last.position - end).norm() > 1e-10 ||
        last.velocity.norm() > 1e-10 || last.acceleration.norm() > 1e-9) {
        throw std::runtime_error("other-drone rest-to-rest endpoint contract failed");
    }

    // The other drone is a shared physically executable commitment, not merely
    // an arbitrary moving obstacle. Certify its cubic derivative control points
    // against the same per-axis speed/acceleration limits used by the planner.
    const DerivativeSpline velocity = derivativeSpline(
        piece.control_points, piece.knots, kCubicDegree, 1);
    const DerivativeSpline acceleration = derivativeSpline(
        piece.control_points, piece.knots, kCubicDegree, 2);
    if (velocity.control_points.cwiseAbs().maxCoeff() > 1.0 + 1e-12 ||
        acceleration.control_points.cwiseAbs().maxCoeff() > 1.5 + 1e-12) {
        throw std::runtime_error("other-drone trajectory exceeds nominal dynamic limits");
    }
    return trajectory;
}

std::vector<OtherDroneModel> makeOtherDrones(const MissionScenario& scenario) {
    std::vector<OtherDroneModel> result;
    result.reserve(scenario.other_drones.size());
    for (const auto& spec : scenario.other_drones) {
        OtherDroneModel model;
        model.name = spec.name;
        model.trajectory = makeRestToRestOtherDroneTrajectory(
            spec.start, spec.end, 0.0, spec.motion_end_s);
        result.push_back(std::move(model));
    }
    return result;
}

LocalPlannerConfig makeLocalConfig(const MissionScenario& scenario, int samples) {
    LocalPlannerConfig config;
    config.num_segments = 4;
    config.octopus.samples_per_axis = {samples, samples, samples};
    config.octopus.alpha_shrink = 0.9;
    config.octopus.voxel_fraction = 0.10;
    config.octopus.heuristic_bias = 1.0;
    config.octopus.max_runtime_s = 2.0;
    config.octopus.xyz_min = scenario.bounds_min;
    config.octopus.xyz_max = scenario.bounds_max;
    config.octopus.random_seed = 1;
    config.refinement.xyz_min = scenario.bounds_min;
    config.refinement.xyz_max = scenario.bounds_max;
    config.refinement.j_max = Vec3::Constant(4.0);
    config.refinement.max_working_set_recalculations = 500;
    return config;
}

RecedingHorizonConfig makeRecedingConfig(double planning_radius_m) {
    RecedingHorizonConfig config;
    config.dc_s = kExecutionDt;
    config.planning_radius_m = planning_radius_m;
    config.factor_alpha = 2.5;
    config.min_splice_lookahead_s = 0.05;
    config.max_splice_lookahead_s = 1.0;
    config.factor_alloc = 1.0;
    config.factor_alloc_close = 2.5;
    config.spline_time_factor = 2.5;
    config.close_to_goal_m = 0.20;
    config.goal_tolerance_m = 0.05;
    config.continuity_tolerance = 1e-7;
    config.separator_validation_tolerance = 1e-7;
    config.v_max = Vec3::Ones();
    config.a_max = Vec3::Constant(1.5);
    return config;
}

void printReplan(int index, const ReplanResult& r) {
    std::cout << "replan " << index << "\n"
              << "  now: " << r.now_s << " s\n"
              << "  status: " << r.status << "\n"
              << "  accepted: " << std::boolalpha << r.accepted << "\n";
    if (!r.attempted) {
        return;
    }
    std::cout << "  A time: " << r.splice_time_s << " s\n"
              << "  A position: " << r.splice_state.position.transpose() << "\n"
              << "  local goal: " << r.local_goal.transpose() << "\n"
              << "  local duration: " << r.local_duration_s << " s\n"
              << "  planning/recheck/final world versions: "
              << r.planning_world_version << " / " << r.recheck_world_version
              << " / " << r.final_recheck_world_version << "\n";
    if (r.local_plan.has_value()) {
        std::cout << "  Octopus time: "
                  << 1e3 * r.local_plan->search.search_time_s << " ms\n";
        if (r.local_plan->refinement.has_value()) {
            std::cout << "  QP time: "
                      << 1e3 * r.local_plan->refinement->solve_time_s << " ms\n";
        }
        std::cout << "  total local solve: "
                  << 1e3 * r.local_plan->total_solve_time_s << " ms\n";
    }
    if (r.prefix_safety.has_value()) {
        std::cout << "  incumbent-prefix check: " << r.prefix_safety->status << "\n";
    }
    if (r.candidate_safety.has_value()) {
        std::cout << "  candidate check: " << r.candidate_safety->status;
        if (!r.candidate_safety->obstacle_name.empty()) {
            std::cout << " (" << r.candidate_safety->obstacle_name << ')';
        }
        std::cout << "\n";
    }
    if (r.accepted) {
        std::cout << "  C2 errors p/v/a: "
                  << r.splice_diagnostics.position_error << " / "
                  << r.splice_diagnostics.velocity_error << " / "
                  << r.splice_diagnostics.acceleration_error << "\n";
    }
}

std::string safetyStatus(const std::optional<TrajectorySafetyResult>& result) {
    return result.has_value() ? result->status : std::string();
}

void writeReplan(std::ofstream& csv, int index, const ReplanResult& r) {
    const double search_t = r.local_plan.has_value()
        ? r.local_plan->search.search_time_s : 0.0;
    const double qp_t = (r.local_plan.has_value() && r.local_plan->refinement.has_value())
        ? r.local_plan->refinement->solve_time_s : 0.0;
    const std::string prefix_obstacle = (r.prefix_safety.has_value())
        ? r.prefix_safety->obstacle_name : std::string();
    const std::string candidate_obstacle = (r.candidate_safety.has_value())
        ? r.candidate_safety->obstacle_name : std::string();

    csv << index << ',' << r.now_s << ',' << r.status << ',' << r.accepted << ','
        << r.splice_time_s << ',' << r.splice_lookahead_s << ','
        << r.splice_state.position.x() << ',' << r.splice_state.position.y() << ','
        << r.splice_state.position.z() << ','
        << r.local_goal.x() << ',' << r.local_goal.y() << ',' << r.local_goal.z() << ','
        << r.minimum_time_s << ',' << r.time_allocation_factor << ','
        << r.local_duration_s << ',' << r.replan_runtime_s << ','
        << r.candidate_finish_time_s << ',' << r.authority_decision_time_s << ','
        << r.candidate_late << ',' << search_t << ',' << qp_t << ','
        << r.splice_diagnostics.position_error << ','
        << r.splice_diagnostics.velocity_error << ','
        << r.splice_diagnostics.acceleration_error << ','
        << r.committed_endpoint_distance_m << ','
        << r.planning_world_version << ',' << r.recheck_world_version << ','
        << r.final_recheck_world_version << ',' << r.recheck_retry_performed << ','
        << r.incumbent_prefix_unsafe << ','
        << safetyStatus(r.prefix_safety) << ',' << prefix_obstacle << ','
        << safetyStatus(r.candidate_safety) << ',' << candidate_obstacle << '\n';
}

double signedAabbClearance(const Vec3& planning_center,
                           const Vec3& other_center,
                           const Vec3& combined_half_extents) {
    if (!planning_center.allFinite() || !other_center.allFinite() ||
        !combined_half_extents.allFinite() ||
        (combined_half_extents.array() < 0.0).any()) {
        throw std::invalid_argument("invalid AABB clearance diagnostic input");
    }
    const Vec3 q = (planning_center - other_center).cwiseAbs() - combined_half_extents;
    const Vec3 outside = q.cwiseMax(Vec3::Zero());
    if (outside.squaredNorm() > 0.0) {
        return outside.norm();
    }
    // Inside the Minkowski-sum box: max(q) is the signed penetration depth to
    // the closest face (zero at contact, negative in overlap).
    return q.maxCoeff();
}

double minimumTimeAlignedClearance(const Vec3& planning_position,
                                   double time_s,
                                   const std::vector<OtherDroneModel>& other_drones) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& other : other_drones) {
        const Vec3 combined = kPlanningDroneHalfExtents + other.physical_half_extents +
                              other.tracking_error_half_extents;
        const Vec3 other_position = other.trajectory.evaluate(time_s).position;
        best = std::min(best,
                        signedAabbClearance(planning_position, other_position, combined));
    }
    return best;
}

RuntimeStats calculateStats(std::vector<double> values) {
    RuntimeStats stats;
    if (values.empty()) {
        return stats;
    }
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    stats.mean = sum / static_cast<double>(values.size());
    const auto quantile = [&](double p) {
        const double index = p * static_cast<double>(values.size() - 1U);
        const std::size_t lo = static_cast<std::size_t>(std::floor(index));
        const std::size_t hi = static_cast<std::size_t>(std::ceil(index));
        const double fraction = index - static_cast<double>(lo);
        return values[lo] * (1.0 - fraction) + values[hi] * fraction;
    };
    stats.median = quantile(0.50);
    stats.p95 = quantile(0.95);
    stats.maximum = values.back();
    return stats;
}

void addRuntimeSamples(const ReplanResult& result,
                       std::vector<double>* replan,
                       std::vector<double>* search,
                       std::vector<double>* qp) {
    if (!result.attempted) {
        return;
    }
    replan->push_back(result.replan_runtime_s);
    if (result.local_plan.has_value()) {
        search->push_back(result.local_plan->search.search_time_s);
        if (result.local_plan->refinement.has_value()) {
            qp->push_back(result.local_plan->refinement->solve_time_s);
        }
    }
}

void printStats(const char* label, const RuntimeStats& stats) {
    std::cout << "  " << label << " mean/median/p95/max [ms]: "
              << 1e3 * stats.mean << " / " << 1e3 * stats.median << " / "
              << 1e3 * stats.p95 << " / " << 1e3 * stats.maximum << "\n";
}

bool acceptedAuthorityContractHolds(const ReplanResult& result,
                                    const RecedingHorizonConfig& config,
                                    std::uint64_t expected_world_version) {
    if (!result.accepted) {
        return true;
    }
    if (!result.splice_diagnostics.accepted || result.candidate_late ||
        result.incumbent_prefix_unsafe || result.recheck_retry_performed) {
        return false;
    }
    if (!result.prefix_safety.has_value() || !result.prefix_safety->safe ||
        !result.candidate_safety.has_value() || !result.candidate_safety->safe) {
        return false;
    }
    if (result.planning_world_version != expected_world_version ||
        result.recheck_world_version != expected_world_version) {
        return false;
    }
    return result.splice_diagnostics.position_error <= config.continuity_tolerance + 1e-12 &&
           result.splice_diagnostics.velocity_error <= config.continuity_tolerance + 1e-12 &&
           result.splice_diagnostics.acceleration_error <= config.continuity_tolerance + 1e-12;
}

std::vector<std::pair<double, Vec3>> sampleUnobstructedMission(
    const MissionScenario& scenario,
    const RecedingHorizonConfig& config,
    int samples) {
    WorldSnapshot empty_snapshot;
    empty_snapshot.captured_at_s = 0.0;
    empty_snapshot.ego_half_extents = kPlanningDroneHalfExtents;
    VersionedWorld empty_world(std::move(empty_snapshot));

    RecedingHorizonPlanner nominal_planner(
        scenario.planning_goal, config, makeLocalConfig(scenario, samples));
    ReplanResult initial = nominal_planner.replan(
        0.0, scenario.planning_start, empty_world);
    if (!initial.accepted) {
        throw std::runtime_error(
            "failed to construct unobstructed reference mission for B.3 stress witness");
    }

    std::vector<std::pair<double, Vec3>> trajectory;
    double now = 0.0;
    int guard = 0;
    while (guard++ < 3000) {
        const State state = nominal_planner.committedTrajectory().evaluate(now);
        trajectory.emplace_back(now, state.position);
        if (nominal_planner.goalReached(state)) {
            return trajectory;
        }

        double execution_step = config.dc_s;
        if (now > 0.0 && !nominal_planner.goalSeen()) {
            const ReplanResult result = nominal_planner.replan(now, state, empty_world);
            if (result.attempted && !result.accepted) {
                throw std::runtime_error(
                    "unobstructed reference mission had a failed replan during B.3 stress witness");
            }
            execution_step = std::max(config.dc_s, result.replan_runtime_s);
        }
        now += execution_step;
    }
    throw std::runtime_error(
        "unobstructed reference mission did not reach goal during B.3 stress witness");
}

double minimumSampledClearance(
    const std::vector<std::pair<double, Vec3>>& planning_samples,
    const std::vector<OtherDroneModel>& other_drones) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& sample : planning_samples) {
        best = std::min(
            best,
            minimumTimeAlignedClearance(sample.second, sample.first, other_drones));
    }
    return best;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scenario_name = "crossing";
        std::string prefix;
        int samples = 7;
        double planning_radius_m = 2.0;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scenario" && i + 1 < argc) {
                scenario_name = argv[++i];
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = std::stoi(argv[++i]);
            } else if (arg == "--planning-radius" && i + 1 < argc) {
                planning_radius_m = std::stod(argv[++i]);
            } else if (arg == "--csv-prefix" && i + 1 < argc) {
                prefix = argv[++i];
            } else {
                throw std::invalid_argument(
                    "usage: cooperative_mission_demo "
                    "[--scenario crossing|same_direction|two_drones|"
                    "crossing_stress|same_direction_stress|two_drones_stress] "
                    "[--samples 7] [--planning-radius 2.0] [--csv-prefix path]");
            }
        }
        if (samples < 2) {
            throw std::invalid_argument("--samples must be at least 2");
        }
        if (!std::isfinite(planning_radius_m) || planning_radius_m <= 0.0) {
            throw std::invalid_argument("--planning-radius must be finite and > 0");
        }
        if (prefix.empty()) {
            prefix = "/tmp/r6_3b3_" + scenario_name;
        }

        const MissionScenario scenario = makeScenario(scenario_name);
        const RecedingHorizonConfig config = makeRecedingConfig(planning_radius_m);
        std::vector<OtherDroneModel> other_drones = makeOtherDrones(scenario);

        // Commissioning-scene sanity: after every fixed shared motion has ended,
        // its stopped hold must leave the planning goal collision-free. Otherwise
        // a liveness failure would be baked into the scenario rather than exposing
        // planner behaviour.
        double all_other_motion_ended_s = 0.0;
        for (const auto& spec : scenario.other_drones) {
            all_other_motion_ended_s = std::max(all_other_motion_ended_s, spec.motion_end_s);
        }
        const double final_goal_clearance = minimumTimeAlignedClearance(
            scenario.planning_goal, all_other_motion_ended_s, other_drones);
        if (!(final_goal_clearance > 0.0)) {
            throw std::runtime_error(
                "invalid B.3 scenario: fixed other-drone stopped hold blocks planning goal");
        }

        double unobstructed_minimum_clearance =
            std::numeric_limits<double>::infinity();
        if (scenario.require_unobstructed_conflict) {
            const auto unobstructed = sampleUnobstructedMission(
                scenario, config, samples);
            unobstructed_minimum_clearance = minimumSampledClearance(
                unobstructed, other_drones);
            if (!(unobstructed_minimum_clearance < -1e-4)) {
                throw std::runtime_error(
                    "invalid B.3 stress scenario: unobstructed planning mission does not "
                    "actually collide with the configured other-drone C-space tube");
            }
        }

        WorldSnapshot initial_world;
        initial_world.captured_at_s = 0.0;
        initial_world.ego_half_extents = kPlanningDroneHalfExtents;
        for (const auto& other : other_drones) {
            CooperativeObstacleTrajectory obstacle;
            obstacle.name = other.name;
            obstacle.trajectory = other.trajectory;
            obstacle.physical_half_extents = other.physical_half_extents;
            obstacle.tracking_error_half_extents = other.tracking_error_half_extents;
            obstacle.validate();
            initial_world.cooperative_obstacles.push_back(std::move(obstacle));
        }
        VersionedWorld world(std::move(initial_world));

        RecedingHorizonPlanner planner(
            scenario.planning_goal, config, makeLocalConfig(scenario, samples));
        TrajectorySafetyChecker final_checker(config.separator_validation_tolerance);

        std::ofstream trajectory_csv(prefix + "_trajectory.csv");
        std::ofstream replans_csv(prefix + "_replans.csv");
        std::ofstream other_csv(prefix + "_other_drones.csv");
        if (!trajectory_csv || !replans_csv || !other_csv) {
            throw std::runtime_error("failed to open one or more B.3 CSV outputs");
        }
        trajectory_csv << std::setprecision(15);
        replans_csv << std::setprecision(15);
        other_csv << std::setprecision(15);

        trajectory_csv
            << "time,x,y,z,vx,vy,vz,ax,ay,az,goal_distance,goal_seen,"
               "committed_end_time,piece_count,sampled_time_aligned_clearance\n";
        replans_csv
            << "index,now,status,accepted,splice_time,splice_lookahead,"
               "Ax,Ay,Az,Gx,Gy,Gz,minimum_time,time_factor,local_duration,"
               "replan_runtime,candidate_finish_time,authority_decision_time,"
               "candidate_late,search_runtime,qp_runtime,position_splice_error,"
               "velocity_splice_error,acceleration_splice_error,endpoint_distance,"
               "planning_world_version,recheck_world_version,final_recheck_world_version,"
               "recheck_retry,incumbent_prefix_unsafe,prefix_safety,prefix_obstacle,"
               "candidate_safety,candidate_obstacle\n";
        other_csv
            << "time,name,x,y,z,vx,vy,vz,ax,ay,az,"
               "physical_half_x,physical_half_y,physical_half_z,"
               "tracking_half_x,tracking_half_y,tracking_half_z\n";

        std::cout << std::setprecision(15);
        std::cout << "R6.3B.3 cooperative moving-drone mission\n"
                  << "scenario: " << scenario.name << "\n"
                  << "planning drone start: "
                  << scenario.planning_start.position.transpose() << "\n"
                  << "planning drone goal: " << scenario.planning_goal.transpose() << "\n"
                  << "other drones: " << other_drones.size() << "\n"
                  << "other-drone tracking half-extents: "
                  << kTrackingHalfExtents.transpose() << " m\n"
                  << "planning radius: " << config.planning_radius_m << " m\n"
                  << "world version: " << world.currentVersion() << "\n";
        if (scenario.require_unobstructed_conflict) {
            std::cout << "unobstructed-reference minimum simultaneous clearance: "
                      << unobstructed_minimum_clearance << " m (must be < 0)\n";
        }
        for (std::size_t i = 0; i < other_drones.size(); ++i) {
            const auto& spec = scenario.other_drones[i];
            const auto& other = other_drones[i];
            const double midpoint_t = 0.5 * spec.motion_end_s;
            const State midpoint = other.trajectory.evaluate(midpoint_t);
            const State endpoint = other.trajectory.evaluate(spec.motion_end_s);
            std::cout << "  " << other.name << ": " << spec.start.transpose()
                      << " -> " << spec.end.transpose()
                      << ", motion_end=" << spec.motion_end_s << " s"
                      << ", midpoint t=" << midpoint_t
                      << " p=" << midpoint.position.transpose()
                      << ", endpoint |v|/|a|=" << endpoint.velocity.norm()
                      << "/" << endpoint.acceleration.norm() << "\n";
        }

        int replan_index = 0;
        ReplanResult initial = planner.replan(0.0, scenario.planning_start, world);
        printReplan(replan_index, initial);
        writeReplan(replans_csv, replan_index, initial);
        if (!initial.accepted) {
            std::cerr << "initial plan failed: " << initial.status << "\n";
            return 2;
        }

        std::vector<double> replan_runtimes;
        std::vector<double> search_runtimes;
        std::vector<double> qp_runtimes;
        addRuntimeSamples(initial, &replan_runtimes, &search_runtimes, &qp_runtimes);

        double max_p = initial.splice_diagnostics.position_error;
        double max_v = initial.splice_diagnostics.velocity_error;
        double max_a = initial.splice_diagnostics.acceleration_error;
        int accepted = 1;
        int failed = 0;
        int late = initial.candidate_late ? 1 : 0;
        int recheck_retries = initial.recheck_retry_performed ? 1 : 0;
        bool authority_contract_ok = acceptedAuthorityContractHolds(
            initial, config, world.currentVersion());
        bool incumbent_prefix_unsafe_seen = initial.incumbent_prefix_unsafe;
        bool unexpected_world_retry_seen = initial.recheck_retry_performed;
        bool reached = false;
        double now = 0.0;
        int guard = 0;
        double path_length = 0.0;
        double minimum_time_aligned_clearance = std::numeric_limits<double>::infinity();
        std::optional<Vec3> previous_position;
        std::vector<Vec3> executed_positions;

        while (guard++ < 3000) {
            const State state = planner.committedTrajectory().evaluate(now);
            const double distance = (state.position - scenario.planning_goal).norm();
            const double time_aligned_clearance = minimumTimeAlignedClearance(
                state.position, now, other_drones);
            minimum_time_aligned_clearance = std::min(
                minimum_time_aligned_clearance, time_aligned_clearance);

            if (previous_position.has_value()) {
                path_length += (state.position - *previous_position).norm();
            }
            previous_position = state.position;
            executed_positions.push_back(state.position);

            trajectory_csv
                << now << ',' << state.position.x() << ',' << state.position.y() << ','
                << state.position.z() << ',' << state.velocity.x() << ','
                << state.velocity.y() << ',' << state.velocity.z() << ','
                << state.acceleration.x() << ',' << state.acceleration.y() << ','
                << state.acceleration.z() << ',' << distance << ',' << planner.goalSeen()
                << ',' << planner.committedTrajectory().endTime() << ','
                << planner.committedTrajectory().pieceCount() << ','
                << time_aligned_clearance << '\n';

            for (const auto& other : other_drones) {
                const State other_state = other.trajectory.evaluate(now);
                other_csv
                    << now << ',' << other.name << ','
                    << other_state.position.x() << ',' << other_state.position.y() << ','
                    << other_state.position.z() << ',' << other_state.velocity.x() << ','
                    << other_state.velocity.y() << ',' << other_state.velocity.z() << ','
                    << other_state.acceleration.x() << ',' << other_state.acceleration.y() << ','
                    << other_state.acceleration.z() << ','
                    << other.physical_half_extents.x() << ','
                    << other.physical_half_extents.y() << ','
                    << other.physical_half_extents.z() << ','
                    << other.tracking_error_half_extents.x() << ','
                    << other.tracking_error_half_extents.y() << ','
                    << other.tracking_error_half_extents.z() << '\n';
            }

            if (planner.goalReached(state)) {
                reached = true;
                break;
            }

            double execution_step = config.dc_s;
            if (now > 0.0 && !planner.goalSeen()) {
                ReplanResult result = planner.replan(now, state, world);
                ++replan_index;
                execution_step = std::max(config.dc_s, result.replan_runtime_s);
                addRuntimeSamples(result, &replan_runtimes, &search_runtimes, &qp_runtimes);
                if (result.accepted) {
                    ++accepted;
                    max_p = std::max(max_p, result.splice_diagnostics.position_error);
                    max_v = std::max(max_v, result.splice_diagnostics.velocity_error);
                    max_a = std::max(max_a, result.splice_diagnostics.acceleration_error);
                } else if (result.attempted) {
                    ++failed;
                }
                if (result.candidate_late) {
                    ++late;
                }
                if (result.recheck_retry_performed) {
                    ++recheck_retries;
                    unexpected_world_retry_seen = true;
                }
                if (result.incumbent_prefix_unsafe) {
                    incumbent_prefix_unsafe_seen = true;
                }
                authority_contract_ok = authority_contract_ok &&
                    acceptedAuthorityContractHolds(result, config, world.currentVersion());
                if (replan_index <= 8 || result.goal_seen || !result.accepted) {
                    printReplan(replan_index, result);
                }
                writeReplan(replans_csv, replan_index, result);
            }

            now += execution_step;
        }

        const State final_state = planner.committedTrajectory().evaluate(now);
        const WorldSnapshot final_world = world.snapshot(now);
        const TrajectorySafetyResult executed_safety = final_checker.checkCommitted(
            planner.committedTrajectory(), planner.committedTrajectory().startTime(), now,
            final_world);

        // Diagnostic only: deliberately ignore time and compare every sampled
        // planning-drone centre against every sampled centre on every other
        // drone's path. Negative here with positive simultaneous clearance is
        // evidence that the same physical space was used at different times.
        double minimum_time_ignored_swept_clearance =
            std::numeric_limits<double>::infinity();
        for (const Vec3& planning_position : executed_positions) {
            for (const auto& other : other_drones) {
                const Vec3 combined = kPlanningDroneHalfExtents + other.physical_half_extents +
                                      other.tracking_error_half_extents;
                // Sweep the other drone's complete explicit commitment, even if
                // the planning drone reaches its goal earlier. This diagnostic is
                // intentionally time-agnostic and therefore asks only whether the
                // two spatial paths occupy common C-space at any times.
                const double swept_until = std::max(now, other.trajectory.endTime());
                for (double other_time = 0.0;
                     other_time < swept_until - 0.5 * kExecutionDt;
                     other_time += kExecutionDt) {
                    const Vec3 other_position = other.trajectory.evaluate(other_time).position;
                    minimum_time_ignored_swept_clearance = std::min(
                        minimum_time_ignored_swept_clearance,
                        signedAabbClearance(planning_position, other_position, combined));
                }
                const Vec3 other_position = other.trajectory.evaluate(swept_until).position;
                minimum_time_ignored_swept_clearance = std::min(
                    minimum_time_ignored_swept_clearance,
                    signedAabbClearance(planning_position, other_position, combined));
            }
        }

        const RuntimeStats replan_stats = calculateStats(replan_runtimes);
        const RuntimeStats search_stats = calculateStats(search_runtimes);
        const RuntimeStats qp_stats = calculateStats(qp_runtimes);
        const double straight_distance =
            (scenario.planning_goal - scenario.planning_start.position).norm();
        const double path_ratio = straight_distance > 0.0
            ? path_length / straight_distance : 0.0;
        const bool c2_contract_ok =
            max_p <= config.continuity_tolerance + 1e-12 &&
            max_v <= config.continuity_tolerance + 1e-12 &&
            max_a <= config.continuity_tolerance + 1e-12;
        const bool fixed_world_contract_ok =
            world.currentVersion() == 0U && !unexpected_world_retry_seen;
        const bool stress_witness_ok =
            !scenario.require_unobstructed_conflict ||
            unobstructed_minimum_clearance < -1e-4;
        const bool regression_contract_ok =
            reached && executed_safety.safe && authority_contract_ok &&
            !incumbent_prefix_unsafe_seen && c2_contract_ok && fixed_world_contract_ok &&
            stress_witness_ok;

        std::cout << "\nmission summary\n"
                  << "  scenario: " << scenario.name << "\n"
                  << "  reached: " << std::boolalpha << reached << "\n"
                  << "  goal seen: " << planner.goalSeen() << "\n"
                  << "  continuous executed-path safety: " << executed_safety.status << "\n"
                  << "  safety LP calls: " << executed_safety.separator_lp_calls << "\n";
        if (!executed_safety.obstacle_name.empty()) {
            std::cout << "  safety obstacle: " << executed_safety.obstacle_name << "\n"
                      << "  unsafe interval: [" << executed_safety.unsafe_interval_start_s
                      << ", " << executed_safety.unsafe_interval_end_s << "] s\n";
        }
        std::cout << "  planning radius: " << config.planning_radius_m << " m\n"
                  << "  accepted replans: " << accepted << "\n"
                  << "  failed replans: " << failed << "\n"
                  << "  late candidates: " << late << "\n"
                  << "  recheck retries: " << recheck_retries << "\n"
                  << "  final simulated time: " << now << " s\n"
                  << "  final position: " << final_state.position.transpose() << "\n"
                  << "  final goal distance: "
                  << (final_state.position - scenario.planning_goal).norm() << " m\n"
                  << "  executed path length: " << path_length << " m\n"
                  << "  path length / straight distance: " << path_ratio << "\n"
                  << "  sampled minimum simultaneous AABB clearance: "
                  << minimum_time_aligned_clearance << " m\n"
                  << "  sampled minimum time-ignored swept-path AABB clearance: "
                  << minimum_time_ignored_swept_clearance << " m\n"
                  << "  committed pieces: " << planner.committedTrajectory().pieceCount()
                  << "\n"
                  << "  max splice errors p/v/a: " << max_p << " / " << max_v
                  << " / " << max_a << "\n";
        printStats("replan runtime", replan_stats);
        printStats("Octopus runtime", search_stats);
        printStats("qpOASES runtime", qp_stats);
        std::cout << "  world version: " << world.currentVersion() << "\n"
                  << "  accepted-authority contract: " << authority_contract_ok << "\n"
                  << "  incumbent-prefix unsafe seen: " << incumbent_prefix_unsafe_seen << "\n"
                  << "  C2 regression contract: " << c2_contract_ok << "\n"
                  << "  fixed-world/no-retry contract: " << fixed_world_contract_ok << "\n"
                  << "  stress unobstructed-conflict witness: " << stress_witness_ok;
        if (scenario.require_unobstructed_conflict) {
            std::cout << " (nominal min clearance=" << unobstructed_minimum_clearance << " m)";
        }
        std::cout << "\n"
                  << "  B.3 regression contract: " << regression_contract_ok << "\n"
                  << "  trajectory CSV: " << prefix << "_trajectory.csv\n"
                  << "  replans CSV: " << prefix << "_replans.csv\n"
                  << "  other-drone CSV: " << prefix << "_other_drones.csv\n";

        if (!executed_safety.safe) {
            return 4;
        }
        if (!authority_contract_ok || incumbent_prefix_unsafe_seen) {
            return 5;
        }
        if (!c2_contract_ok) {
            return 6;
        }
        if (!fixed_world_contract_ok) {
            return 7;
        }
        return reached ? 0 : 3;
    } catch (const std::exception& exc) {
        std::cerr << "cooperative_mission_demo: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
