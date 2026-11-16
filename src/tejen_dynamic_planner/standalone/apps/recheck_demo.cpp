#include "dynamic_planner/receding_horizon_planner.hpp"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <stdexcept>
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

    WorldSnapshot snapshot(double captured_at_s) const override {
        const std::size_t index = std::min(snapshot_calls_, snapshots.size() - 1U);
        ++snapshot_calls_;
        WorldSnapshot result = snapshots.at(index);
        result.captured_at_s = captured_at_s;
        return result;
    }

    std::uint64_t currentVersion() const override {
        const std::size_t index = std::min(snapshot_calls_, snapshots.size() - 1U);
        return snapshots.at(index).version;
    }

    bool runIfVersionCurrent(
        std::uint64_t expected_version,
        const std::function<void()>& action) const override {
        ++commit_attempts_;
        if (commit_attempts_ == 1) {
            return false;  // deterministic update landed after first recheck
        }
        const std::size_t index = std::min(snapshot_calls_ - 1U, snapshots.size() - 1U);
        if (snapshots.at(index).version != expected_version) {
            return false;
        }
        action();
        return true;
    }

private:
    mutable std::size_t snapshot_calls_ = 0;
    mutable int commit_attempts_ = 0;
};

WorldSnapshot emptySnapshot(std::uint64_t version) {
    WorldSnapshot world;
    world.version = version;
    world.captured_at_s = 0.0;
    return world;
}

WorldSnapshot obstacleSnapshot(std::uint64_t version,
                               const Vec3& center,
                               const Vec3& half) {
    WorldSnapshot world = emptySnapshot(version);
    world.static_obstacles.push_back(
        StaticConvexObstacle{"updated-obstacle", boxVertices(center, half)});
    return world;
}

LocalPlannerConfig localConfig() {
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

ReplanResult runCase(bool block_candidate) {
    State start;
    start.position = Vec3(0.0, 0.0, 1.5);
    RecedingHorizonConfig config;
    config.spline_time_factor = 2.5;
    config.continuity_tolerance = 1e-7;

    ScriptedWorld world;
    world.snapshots.push_back(emptySnapshot(0));
    world.snapshots.push_back(emptySnapshot(0));
    if (block_candidate) {
        world.snapshots.push_back(obstacleSnapshot(
            1, Vec3(0.45, 0.0, 1.5), Vec3(0.30, 0.25, 0.25)));
    } else {
        world.snapshots.push_back(obstacleSnapshot(
            1, Vec3(0.0, 1.5, 1.5), Vec3(0.10, 0.10, 0.10)));
    }

    RecedingHorizonPlanner planner(Vec3(3.0, 0.0, 1.5), config, localConfig());
    return planner.replan(0.0, start, world);
}

void printCase(const char* name, const ReplanResult& result) {
    std::cout << name << "\n"
              << "  status: " << result.status << "\n"
              << "  accepted: " << std::boolalpha << result.accepted << "\n"
              << "  planning version: " << result.planning_world_version << "\n"
              << "  first recheck version: " << result.recheck_world_version << "\n"
              << "  retry performed: " << result.recheck_retry_performed << "\n"
              << "  final recheck version: " << result.final_recheck_world_version << "\n";
    if (result.candidate_safety.has_value()) {
        std::cout << "  candidate safety: " << result.candidate_safety->status;
        if (!result.candidate_safety->obstacle_name.empty()) {
            std::cout << " (" << result.candidate_safety->obstacle_name << ")";
        }
        std::cout << "\n";
    }
}

}  // namespace

int main() {
    try {
        std::cout << std::setprecision(15)
                  << "R6.3B.2 newest-world Check/Recheck demo\n";
        const ReplanResult irrelevant = runCase(false);
        const ReplanResult blocked = runCase(true);
        printCase("irrelevant update", irrelevant);
        printCase("candidate-blocking update", blocked);
        if (!irrelevant.accepted ||
            irrelevant.status != "ACCEPTED_AFTER_RECHECK" ||
            blocked.accepted ||
            blocked.status != "LATEST_CANDIDATE_UNSAFE_AFTER_RETRY") {
            std::cerr << "recheck_demo: FAIL\n";
            return 2;
        }
        std::cout << "recheck_demo: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "recheck_demo: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
