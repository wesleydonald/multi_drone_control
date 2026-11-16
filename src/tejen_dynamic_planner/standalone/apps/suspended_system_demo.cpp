#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/ego_collision_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dynamic_planner;

namespace {

constexpr double kControlDt = 1.0 / 30.0;
const Vec3 kBodyHalf(0.105, 0.105, 0.060);

struct RuntimeStats {
    double mean = 0.0;
    double median = 0.0;
    double p95 = 0.0;
    double maximum = 0.0;
};

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

TrajectoryPiece linearPiece(const Vec3& a, const Vec3& b, double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = a.transpose();
    piece.control_points.row(1) = (a + (b - a) / 3.0).transpose();
    piece.control_points.row(2) = (a + 2.0 * (b - a) / 3.0).transpose();
    piece.control_points.row(3) = b.transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    return piece;
}

CommittedTrajectory constantTrajectory(const Vec3& p, double t0, double t1) {
    CommittedTrajectory result;
    if (!result.replaceSuffix(t0, linearPiece(p, p, t0, t1)).accepted) {
        throw std::runtime_error("failed to construct constant cooperative trajectory");
    }
    return result;
}

SuspendedGeometry suspendedGeometry(bool attached = false) {
    SuspendedGeometry g;
    g.enabled = true;
    g.cable_length_m = 0.50;
    g.cable_radius_m = 0.0025;
    g.max_swing_angle_rad = 10.0 * 3.14159265358979323846 / 180.0;
    g.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    g.magnet_center_below_cable_end_m = 0.025;
    g.payload_attached = attached;
    g.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    g.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return g;
}

struct Scenario {
    std::string name;
    State start;
    Vec3 goal;
    Vec3 bounds_min = Vec3(-0.8, -1.8, 0.35);
    Vec3 bounds_max = Vec3(3.8, 1.8, 2.8);
    WorldSnapshot body_only_world;
    WorldSnapshot suspended_world;
};

Scenario makeScenario(const std::string& name) {
    Scenario s;
    s.name = name;
    s.start.position = Vec3(0.0, 0.0, 1.5);
    s.goal = Vec3(3.0, 0.0, 1.5);
    s.body_only_world.captured_at_s = 0.0;
    s.body_only_world.ego_half_extents = kBodyHalf;
    s.suspended_world = s.body_only_world;
    s.suspended_world.ego_suspended_geometry = suspendedGeometry(false);

    if (name == "static_overflight") {
        const StaticConvexObstacle low_box{
            "low_box",
            axisAlignedBoxVertices(Vec3(1.50, 0.0, 0.98), Vec3(0.12, 0.12, 0.05))};
        s.body_only_world.physical_static_obstacles.push_back(low_box);
        s.suspended_world.physical_static_obstacles.push_back(low_box);
    } else if (name == "other_cable") {
        CooperativeObstacleTrajectory other;
        other.name = "other_drone";
        other.trajectory = constantTrajectory(Vec3(1.50, 0.0, 2.05), 0.0, 30.0);
        other.physical_half_extents = kBodyHalf;
        other.tracking_error_half_extents = Vec3::Constant(0.05);
        s.body_only_world.cooperative_obstacles.push_back(other);
        other.suspended_geometry = suspendedGeometry(false);
        s.suspended_world.cooperative_obstacles.push_back(other);
    } else {
        throw std::invalid_argument(
            "usage: suspended_system_demo --scenario static_overflight|other_cable");
    }
    return s;
}

LocalPlannerConfig makeLocalConfig(const Scenario& scene, int samples) {
    LocalPlannerConfig config;
    config.num_segments = 4;
    config.octopus.samples_per_axis = {samples, samples, samples};
    config.octopus.alpha_shrink = 0.9;
    config.octopus.voxel_fraction = 0.10;
    config.octopus.heuristic_bias = 1.0;
    config.octopus.max_runtime_s = 2.0;
    config.octopus.xyz_min = scene.bounds_min;
    config.octopus.xyz_max = scene.bounds_max;
    config.octopus.random_seed = 1;
    config.refinement.xyz_min = scene.bounds_min;
    config.refinement.xyz_max = scene.bounds_max;
    config.refinement.j_max = Vec3::Constant(4.0);
    config.refinement.max_working_set_recalculations = 500;
    return config;
}

RecedingHorizonConfig makeConfig() {
    RecedingHorizonConfig config;
    config.dc_s = kControlDt;
    config.planning_radius_m = 2.0;
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
    // B.4 commissioning mode: keep lateral acceleration below the quasi-static
    // 10-degree pendulum bound with margin. Vertical acceleration retains B.3.
    config.a_max = Vec3(1.0, 1.0, 1.5);
    return config;
}

bool straightWitness(const Scenario& scene, TrajectorySafetyChecker& checker) {
    const TrajectoryPiece direct = linearPiece(
        scene.start.position, scene.goal, 0.0, 10.0);
    const auto body = checker.checkPiece(direct, 0.0, 10.0, scene.body_only_world);
    const auto whole = checker.checkPiece(direct, 0.0, 10.0, scene.suspended_world);
    std::cout << "direct body-only safety: " << body.status << "\n"
              << "direct suspended safety: " << whole.status;
    if (!whole.obstacle_name.empty()) {
        std::cout << " (" << whole.obstacle_name << ')';
    }
    std::cout << "\n";
    return body.safe && !whole.safe;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scenario_name = "static_overflight";
        std::string prefix;
        int samples = 9;
        double replan_rate_hz = 10.0;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scenario" && i + 1 < argc) {
                scenario_name = argv[++i];
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = std::stoi(argv[++i]);
            } else if (arg == "--replan-rate-hz" && i + 1 < argc) {
                replan_rate_hz = std::stod(argv[++i]);
            } else if (arg == "--csv-prefix" && i + 1 < argc) {
                prefix = argv[++i];
            } else {
                throw std::invalid_argument(
                    "usage: suspended_system_demo [--scenario static_overflight|other_cable] "
                    "[--samples 9] [--replan-rate-hz 10] [--csv-prefix path]");
            }
        }
        if (samples < 2) {
            throw std::invalid_argument("--samples must be at least 2");
        }
        if (!std::isfinite(replan_rate_hz) || !(replan_rate_hz > 0.0) ||
            replan_rate_hz > 1.0 / kControlDt + 1e-9) {
            throw std::invalid_argument("--replan-rate-hz must be finite, positive, and <= 30 Hz");
        }
        const double replan_period_s = 1.0 / replan_rate_hz;
        if (prefix.empty()) {
            prefix = "/tmp/r6_3b4_" + scenario_name;
        }

        const Scenario scene = makeScenario(scenario_name);
        const RecedingHorizonConfig config = makeConfig();
        TrajectorySafetyChecker checker(config.separator_validation_tolerance);
        const bool witness = straightWitness(scene, checker);
        if (!witness) {
            throw std::runtime_error(
                "invalid B.4 witness: direct body-only path must be safe while suspended path is unsafe");
        }

        VersionedWorld world(scene.suspended_world);
        RecedingHorizonPlanner planner(scene.goal, config, makeLocalConfig(scene, samples));
        std::ofstream csv(prefix + "_trajectory.csv");
        if (!csv) {
            throw std::runtime_error("failed to open B.4 trajectory CSV");
        }
        csv << std::setprecision(15)
            << "time,x,y,z,vx,vy,vz,ax,ay,az,goal_distance,piece_count\n";

        std::cout << std::setprecision(15)
                  << "R6.3B.4 suspended-system mission\n"
                  << "scenario: " << scene.name << "\n"
                  << "planning radius: " << config.planning_radius_m << " m\n"
                  << "cable length/radius: 0.5 / 0.0025 m\n"
                  << "max swing: 10 deg\n"
                  << "magnet/mocap diameter: 0.10 m\n"
                  << "suspended-mode a_max: " << config.a_max.transpose() << " m/s^2\n"
                  << "Octopus samples per axis: " << samples << " x " << samples << " x " << samples << "\n"
                  << "candidate velocities per expansion: "
                  << static_cast<long long>(samples) * samples * samples << "\n"
                  << "control/reference cadence: " << 1.0 / kControlDt << " Hz\n"
                  << "suspended replan rate: " << replan_rate_hz << " Hz\n"
                  << "nominal replan period: " << 1e3 * replan_period_s << " ms\n";

        std::vector<double> replan_runtimes;
        std::vector<double> search_runtimes;
        std::vector<double> qp_runtimes;

        ReplanResult first = planner.replan(0.0, scene.start, world);
        addRuntimeSamples(first, &replan_runtimes, &search_runtimes, &qp_runtimes);
        if (!first.accepted) {
            std::cerr << "initial plan failed: " << first.status << "\n";
            return 2;
        }

        int accepted = 1;
        int failed = 0;
        int late = 0;
        double max_splice = 0.0;
        double now = 0.0;
        bool reached = false;
        int guard = 0;
        std::size_t coalesced_replan_triggers = 0;
        double next_replan_request_s = replan_period_s;
        while (guard++ < 3000) {
            const State state = planner.committedTrajectory().evaluate(now);
            csv << now << ',' << state.position.x() << ',' << state.position.y() << ','
                << state.position.z() << ',' << state.velocity.x() << ',' << state.velocity.y()
                << ',' << state.velocity.z() << ',' << state.acceleration.x() << ','
                << state.acceleration.y() << ',' << state.acceleration.z() << ','
                << (state.position - scene.goal).norm() << ','
                << planner.committedTrajectory().pieceCount() << '\n';

            if (planner.goalReached(state)) {
                reached = true;
                break;
            }

            double step = config.dc_s;
            if (!planner.goalSeen() && now + 1e-12 >= next_replan_request_s) {
                const ReplanResult result = planner.replan(now, state, world);
                addRuntimeSamples(result, &replan_runtimes, &search_runtimes, &qp_runtimes);

                // The real integration should allow at most one suspended-planner solve
                // in flight. This synchronous harness emulates that contract by
                // coalescing periodic requests that would have arrived before the
                // current solve completed, while the committed trajectory remains
                // authoritative.
                const double finish_time_s = now + result.replan_runtime_s;
                next_replan_request_s += replan_period_s;
                while (next_replan_request_s <= finish_time_s + 1e-12) {
                    ++coalesced_replan_triggers;
                    next_replan_request_s += replan_period_s;
                }
                step = std::max(config.dc_s, result.replan_runtime_s);

                if (result.candidate_late) {
                    ++late;
                }
                if (result.accepted) {
                    ++accepted;
                    max_splice = std::max({
                        max_splice,
                        result.splice_diagnostics.position_error,
                        result.splice_diagnostics.velocity_error,
                        result.splice_diagnostics.acceleration_error});
                } else if (result.attempted) {
                    ++failed;
                }
            } else if (!planner.goalSeen()) {
                // Advance at the controller/reference cadence, but land exactly on
                // the next planner request if it falls before the next control tick.
                step = std::min(config.dc_s,
                                std::max(0.0, next_replan_request_s - now));
            }
            if (!(step > 0.0)) {
                step = config.dc_s;
            }
            now += step;
        }

        TrajectorySafetyResult final_safety;
        if (!planner.committedTrajectory().empty()) {
            final_safety = checker.checkCommitted(
                planner.committedTrajectory(), 0.0,
                planner.committedTrajectory().endTime(), scene.suspended_world);
        }
        const State final_state = planner.committedTrajectory().evaluate(now);
        const bool contract = witness && reached && final_safety.safe && late == 0 &&
                              max_splice <= config.continuity_tolerance + 1e-12;
        const RuntimeStats replan_stats = calculateStats(replan_runtimes);
        const RuntimeStats search_stats = calculateStats(search_runtimes);
        const RuntimeStats qp_stats = calculateStats(qp_runtimes);

        std::cout << "\nmission summary\n"
                  << "  scenario: " << scene.name << "\n"
                  << "  suspended Octopus samples: " << samples << " x " << samples << " x " << samples << "\n"
                  << "  candidate velocities per expansion: "
                  << static_cast<long long>(samples) * samples * samples << "\n"
                  << "  control/reference cadence: " << 1.0 / kControlDt << " Hz\n"
                  << "  suspended replan rate: " << replan_rate_hz << " Hz\n"
                  << "  nominal replan period: " << 1e3 * replan_period_s << " ms\n"
                  << "  coalesced replan triggers: " << coalesced_replan_triggers << "\n"
                  << "  whole-body witness: " << std::boolalpha << witness << "\n"
                  << "  reached: " << reached << "\n"
                  << "  continuous suspended-system safety: " << final_safety.status << "\n"
                  << "  accepted replans: " << accepted << "\n"
                  << "  failed replans: " << failed << "\n"
                  << "  late candidates: " << late << "\n"
                  << "  final time: " << now << " s\n"
                  << "  final goal distance: " << (final_state.position - scene.goal).norm() << " m\n"
                  << "  max splice error: " << max_splice << "\n";
        printStats("replan runtime", replan_stats);
        printStats("Octopus runtime", search_stats);
        printStats("qpOASES runtime", qp_stats);
        std::cout << "  30 Hz control/reference period: " << 1e3 * kControlDt << " ms\n"
                  << "  configured planner period: " << 1e3 * replan_period_s << " ms\n"
                  << "  B.4 regression contract: " << contract << "\n"
                  << "  trajectory CSV: " << prefix << "_trajectory.csv\n";
        return contract ? 0 : 3;
    } catch (const std::exception& exc) {
        std::cerr << "suspended_system_demo: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
