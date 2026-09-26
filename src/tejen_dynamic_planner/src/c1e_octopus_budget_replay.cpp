#include "tejen_dynamic_planner/c1e_scene.hpp"

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/octopus_search.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

namespace {

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusConfig;
using dynamic_planner::OctopusResult;
using dynamic_planner::OctopusSearch;
using dynamic_planner::State;
using dynamic_planner::SuspendedGeometry;
using dynamic_planner::Vec3;
using dynamic_planner::WorldSnapshot;

constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    if (!line.empty() && line.back() == ',') fields.emplace_back();
    return fields;
}

SuspendedGeometry c1eSuspendedGeometry() {
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 0.50;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = 15.0 * kPi / 180.0;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    geometry.payload_attached = false;
    geometry.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    geometry.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return geometry;
}

OctopusConfig c1eOctopusConfig(double budget_s) {
    OctopusConfig config;
    config.v_max = Vec3::Ones();
    config.a_max = Vec3(1.0, 1.0, 1.5);
    config.samples_per_axis = {9, 9, 9};
    config.alpha_shrink = 0.9;
    config.voxel_fraction = 0.10;
    config.heuristic_bias = 1.0;
    config.goal_tolerance_m = 0.05;
    config.max_runtime_s = budget_s;
    config.xyz_min = Vec3(-1.0, -1.0, 0.20);
    config.xyz_max = Vec3(2.0, 1.0, 2.20);
    config.planning_radius_m = 2.0;
    config.random_seed = 1;
    return config;
}

double number(const std::vector<std::string>& row,
              const std::unordered_map<std::string, std::size_t>& index,
              const std::string& name) {
    const auto it = index.find(name);
    if (it == index.end() || it->second >= row.size()) {
        throw std::runtime_error("missing CSV field: " + name);
    }
    return std::stod(row[it->second]);
}

std::string text(const std::vector<std::string>& row,
                 const std::unordered_map<std::string, std::size_t>& index,
                 const std::string& name) {
    const auto it = index.find(name);
    if (it == index.end() || it->second >= row.size()) {
        throw std::runtime_error("missing CSV field: " + name);
    }
    return row[it->second];
}

void writeOptionalMs(std::ostream& out, const std::optional<double>& value) {
    if (value.has_value()) out << 1e3 * *value;
    else out << "nan";
}

void writeOptional(std::ostream& out, const std::optional<double>& value) {
    if (value.has_value()) out << *value;
    else out << "nan";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr << "usage: c1e_octopus_budget_replay <replans.csv>\n";
            return 2;
        }

        const std::filesystem::path input_path(argv[1]);
        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("could not open replans CSV");

        const std::filesystem::path output_path =
            input_path.parent_path() / "octopus_budget_replay.csv";
        std::ofstream output(output_path);
        if (!output) throw std::runtime_error("could not open replay output CSV");

        std::string line;
        if (!std::getline(input, line)) throw std::runtime_error("empty replans CSV");
        const auto headers = splitCsv(line);
        std::unordered_map<std::string, std::size_t> index;
        for (std::size_t i = 0; i < headers.size(); ++i) index[headers[i]] = i;

        WorldSnapshot world;
        world.captured_at_s = 0.0;
        world.ego_half_extents = Vec3(0.105, 0.105, 0.060);
        world.ego_suspended_geometry = c1eSuspendedGeometry();
        world.physical_static_obstacles.push_back(
            tejen_dynamic_planner::makeYawedCuboidObstacle(
                "c1e_commissioning_obstacle",
                Vec3(0.50, 0.06, 0.68),
                Vec3(0.12, 0.25, 0.10),
                20.0 * kPi / 180.0));

        const std::vector<double> fixed_budgets{0.25, 0.40, 0.60, 0.80, 1.00, 2.00};
        constexpr double kOneReferenceTickS = 1.0 / 30.0;
        output << std::setprecision(15)
               << "sequence,policy,budget_s,original_status,replay_status,termination_reason,"
                  "complete_available_at_termination,success,reached_goal,"
                  "search_ms,first_complete_ms,first_complete_goal_distance_m,"
                  "final_goal_distance_m,expanded_nodes,popped_nodes,separator_lp_calls,"
                  "complete_improvements,voxel_size_m\n";

        while (std::getline(input, line)) {
            if (line.empty()) continue;
            const auto row = splitCsv(line);
            if (number(row, index, "attempted") == 0.0) continue;

            State state;
            state.position = Vec3(
                number(row, index, "splice_x"),
                number(row, index, "splice_y"),
                number(row, index, "splice_z"));
            state.velocity = Vec3(
                number(row, index, "splice_vx"),
                number(row, index, "splice_vy"),
                number(row, index, "splice_vz"));
            state.acceleration = Vec3(
                number(row, index, "splice_ax"),
                number(row, index, "splice_ay"),
                number(row, index, "splice_az"));
            const Vec3 local_goal(
                number(row, index, "local_goal_x"),
                number(row, index, "local_goal_y"),
                number(row, index, "local_goal_z"));
            const double duration_s = number(row, index, "local_duration_s");
            const int sequence = static_cast<int>(number(row, index, "sequence"));
            const std::string original_status = text(row, index, "status");
            const double splice_lookahead_s = number(row, index, "splice_lookahead_s");
            const double useful_budget_s = std::max(
                1e-6, splice_lookahead_s - kOneReferenceTickS);

            const auto knots = dynamic_planner::openUniformKnots(0.0, duration_s, 4);
            const ControlPoints q012 =
                dynamic_planner::initialControlPointsFromState(state, knots);
            const auto obstacles = world.timeIndexedObstacles(4, 0.0, duration_s);

            std::vector<std::pair<std::string, double>> policies;
            policies.reserve(fixed_budgets.size() + 1U);
            policies.emplace_back("useful_deadline_proxy", useful_budget_s);
            for (double budget_s : fixed_budgets) {
                policies.emplace_back("fixed", budget_s);
            }

            for (const auto& [policy, budget_s] : policies) {
                const OctopusResult result = OctopusSearch(
                    knots, q012, local_goal, obstacles,
                    c1eOctopusConfig(budget_s)).search();
                output << sequence << ',' << policy << ',' << budget_s << ','
                       << original_status << ',' << result.status << ','
                       << result.termination_reason << ','
                       << result.complete_available_at_termination << ','
                       << result.success << ',' << result.reached_goal << ','
                       << 1e3 * result.search_time_s << ',';
                writeOptionalMs(output, result.first_complete_time_s);
                output << ',';
                writeOptional(output, result.first_complete_goal_distance_m);
                output << ',';
                writeOptional(output, result.goal_distance_m);
                output << ',' << result.expanded_nodes << ',' << result.popped_nodes << ','
                       << result.separator_lp_calls << ','
                       << result.closest_complete_improvements << ','
                       << result.voxel_size_m << '\n';
            }
        }

        output.close();
        std::cout << output_path.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "c1e_octopus_budget_replay: " << error.what() << '\n';
        return 1;
    }
}
