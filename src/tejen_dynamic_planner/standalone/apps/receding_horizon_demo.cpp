#include "dynamic_planner/receding_horizon_planner.hpp"

#include <Eigen/Core>

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

StaticConvexObstacle inflatedBox(const std::string& name,
                                 const Vec3& center,
                                 const Vec3& physical_half) {
    // Same 21 cm nominal ego footprint used by the A validation scenes.
    const Vec3 ego_half(0.105, 0.105, 0.060);
    return StaticConvexObstacle{name, boxVertices(center, physical_half + ego_half)};
}

struct Scene {
    std::string name;
    State start;
    Vec3 goal = Vec3::Zero();
    FrozenWorld world;
    Vec3 bounds_min = Vec3(-1.0, -2.0, 0.4);
    Vec3 bounds_max = Vec3(4.5, 4.0, 2.6);
};

Scene makeScene(const std::string& name) {
    Scene scene;
    scene.name = name;
    scene.start.position = Vec3(0.0, 0.0, 1.5);
    scene.start.velocity = Vec3::Zero();
    scene.start.acceleration = Vec3::Zero();
    if (name == "straight") {
        scene.goal = Vec3(3.0, 0.0, 1.5);
    } else if (name == "diagonal") {
        scene.goal = Vec3(3.2, 2.2, 1.5);
    } else if (name == "long_chicane") {
        scene.goal = Vec3(3.2, 2.4, 1.5);
        scene.world.obstacles.push_back(inflatedBox(
            "chicane-L", Vec3(0.78, 0.88, 1.50), Vec3(0.18, 0.34, 0.24)));
        scene.world.obstacles.push_back(inflatedBox(
            "chicane-R", Vec3(1.48, 1.10, 1.50), Vec3(0.22, 0.34, 0.24)));
    } else {
        throw std::invalid_argument("unknown scene: " + name);
    }
    return scene;
}

LocalPlannerConfig makeLocalConfig(const Scene& scene, int samples) {
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
              << "  splice lookahead: " << r.splice_lookahead_s << " s\n"
              << "  global distance from A: " << r.global_distance_from_splice_m << " m\n"
              << "  local goal: " << r.local_goal.transpose() << "\n"
              << "  RMADER DI minimum time: " << r.minimum_time_s << " s\n"
              << "  applied time factor: " << r.time_allocation_factor << "\n"
              << "  local duration: " << r.local_duration_s << " s\n";
    if (r.local_plan.has_value()) {
        std::cout << "  R4 time: " << 1e3 * r.local_plan->search.search_time_s << " ms\n";
        if (r.local_plan->refinement.has_value()) {
            std::cout << "  QP time: " << 1e3 * r.local_plan->refinement->solve_time_s << " ms\n";
        }
        std::cout << "  total local solve: " << 1e3 * r.local_plan->total_solve_time_s << " ms\n";
    }
    if (r.accepted) {
        std::cout << "  C2 errors p/v/a: "
                  << r.splice_diagnostics.position_error << " / "
                  << r.splice_diagnostics.velocity_error << " / "
                  << r.splice_diagnostics.acceleration_error << "\n"
                  << "  committed endpoint distance: "
                  << r.committed_endpoint_distance_m << " m\n";
    } else if (r.candidate_late) {
        std::cout << "  candidate finish time: " << r.candidate_finish_time_s << " s\n"
                  << "  late by: " << (r.candidate_finish_time_s - r.splice_time_s) << " s\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scene_name = "straight";
        std::string prefix = "r6_3b1";
        int samples = 7;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scene" && i + 1 < argc) {
                scene_name = argv[++i];
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = std::stoi(argv[++i]);
            } else if (arg == "--csv-prefix" && i + 1 < argc) {
                prefix = argv[++i];
            } else {
                throw std::invalid_argument("usage: receding_horizon_demo [--scene straight|diagonal|long_chicane] [--samples 7] [--csv-prefix path]");
            }
        }

        const Scene scene = makeScene(scene_name);
        RecedingHorizonConfig config;
        config.dc_s = 1.0 / 30.0;
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
        config.v_max = Vec3::Ones();
        config.a_max = Vec3::Constant(1.5);

        RecedingHorizonPlanner planner(scene.goal, config, makeLocalConfig(scene, samples));
        std::ofstream trajectory_csv(prefix + "_trajectory.csv");
        std::ofstream replans_csv(prefix + "_replans.csv");
        trajectory_csv << "time,x,y,z,vx,vy,vz,ax,ay,az,goal_distance,goal_seen,committed_end_time,piece_count\n";
        replans_csv << "index,now,status,accepted,splice_time,splice_lookahead,Ax,Ay,Az,Gx,Gy,Gz,minimum_time,time_factor,local_duration,replan_runtime,candidate_finish_time,candidate_late,search_runtime,qp_runtime,position_splice_error,velocity_splice_error,acceleration_splice_error,endpoint_distance\n";

        std::cout << std::setprecision(15);
        std::cout << "R6.3B.1 receding-horizon demo\n"
                  << "scene: " << scene.name << "\n"
                  << "start: " << scene.start.position.transpose() << "\n"
                  << "global goal: " << scene.goal.transpose() << "\n"
                  << "dc: " << config.dc_s << " s\n"
                  << "local planning radius: " << config.planning_radius_m << " m\n"
                  << "spline/search time floor: " << config.spline_time_factor
                  << " x RMADER DI lower bound\n"
                  << "close-to-goal settle zone: " << config.close_to_goal_m << " m\n";

        int replan_index = 0;
        auto initial = planner.replan(0.0, scene.start, scene.world);
        printReplan(replan_index, initial);
        if (!initial.accepted) {
            std::cerr << "initial plan failed: " << initial.status << "\n";
            return 2;
        }

        auto writeReplan = [&](int index, const ReplanResult& r) {
            const double search_t = r.local_plan.has_value() ? r.local_plan->search.search_time_s : 0.0;
            const double qp_t = (r.local_plan.has_value() && r.local_plan->refinement.has_value())
                ? r.local_plan->refinement->solve_time_s : 0.0;
            replans_csv << index << ',' << r.now_s << ',' << r.status << ',' << r.accepted << ','
                        << r.splice_time_s << ',' << r.splice_lookahead_s << ','
                        << r.splice_state.position.x() << ',' << r.splice_state.position.y() << ',' << r.splice_state.position.z() << ','
                        << r.local_goal.x() << ',' << r.local_goal.y() << ',' << r.local_goal.z() << ','
                        << r.minimum_time_s << ',' << r.time_allocation_factor << ','
                        << r.local_duration_s << ',' << r.replan_runtime_s << ','
                        << r.candidate_finish_time_s << ',' << r.candidate_late << ','
                        << search_t << ',' << qp_t << ','
                        << r.splice_diagnostics.position_error << ',' << r.splice_diagnostics.velocity_error << ','
                        << r.splice_diagnostics.acceleration_error << ',' << r.committed_endpoint_distance_m << '\n';
        };
        writeReplan(replan_index, initial);

        double max_p = 0.0, max_v = 0.0, max_a = 0.0;
        int accepted = 1;
        int failed = 0;
        bool reached = false;
        double now = 0.0;
        int guard = 0;
        while (guard++ < 3000) {
            const State state = planner.committedTrajectory().evaluate(now);
            const double distance = (state.position - scene.goal).norm();
            trajectory_csv << now << ',' << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
                           << state.velocity.x() << ',' << state.velocity.y() << ',' << state.velocity.z() << ','
                           << state.acceleration.x() << ',' << state.acceleration.y() << ',' << state.acceleration.z() << ','
                           << distance << ',' << planner.goalSeen() << ',' << planner.committedTrajectory().endTime() << ','
                           << planner.committedTrajectory().pieceCount() << '\n';

            if (planner.goalReached(state)) {
                reached = true;
                break;
            }

            double execution_step = config.dc_s;
            if (now > 0.0 && !planner.goalSeen()) {
                const auto result = planner.replan(now, state, scene.world);
                ++replan_index;
                execution_step = std::max(config.dc_s, result.replan_runtime_s);
                if (result.accepted) {
                    ++accepted;
                    max_p = std::max(max_p, result.splice_diagnostics.position_error);
                    max_v = std::max(max_v, result.splice_diagnostics.velocity_error);
                    max_a = std::max(max_a, result.splice_diagnostics.acceleration_error);
                } else if (result.attempted) {
                    ++failed;
                }
                if (replan_index <= 8 || result.goal_seen || !result.accepted) {
                    printReplan(replan_index, result);
                }
                writeReplan(replan_index, result);
            }

            // A certified stopped endpoint remains a stationary committed
            // hold, so simulated time may advance beyond the last explicit
            // spline while the planner searches for a safe continuation.
            now += execution_step;
        }

        const State final_state = planner.committedTrajectory().evaluate(now);
        std::cout << "\nmission summary\n"
                  << "  reached: " << std::boolalpha << reached << "\n"
                  << "  goal seen: " << planner.goalSeen() << "\n"
                  << "  accepted replans: " << accepted << "\n"
                  << "  failed replans: " << failed << "\n"
                  << "  final simulated time: " << now << " s\n"
                  << "  final position: " << final_state.position.transpose() << "\n"
                  << "  final goal distance: " << (final_state.position - scene.goal).norm() << " m\n"
                  << "  committed pieces: " << planner.committedTrajectory().pieceCount() << "\n"
                  << "  max splice errors p/v/a: " << max_p << " / " << max_v << " / " << max_a << "\n"
                  << "  trajectory CSV: " << prefix << "_trajectory.csv\n"
                  << "  replans CSV: " << prefix << "_replans.csv\n";
        return reached ? 0 : 3;
    } catch (const std::exception& exc) {
        std::cerr << "receding_horizon_demo: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
