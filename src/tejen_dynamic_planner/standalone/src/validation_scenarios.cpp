#include "dynamic_planner/validation_scenarios.hpp"

#include "dynamic_planner/bspline.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dynamic_planner {
namespace {

Eigen::MatrixXd boxVertices(const Vec3& center, const Vec3& half) {
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            for (double sz : {-1.0, 1.0}) {
                vertices.row(row++) = (center + Vec3(sx * half.x(), sy * half.y(), sz * half.z())).transpose();
            }
        }
    }
    return vertices;
}

TimeIndexedObstacle staticBox(const std::string& name,
                              const Vec3& center,
                              const Vec3& physical_half,
                              const Vec3& ego_half,
                              int num_segments) {
    TimeIndexedObstacle obstacle;
    obstacle.name = name;
    const Eigen::MatrixXd inflated = boxVertices(center, physical_half + ego_half);
    obstacle.interval_vertices.assign(static_cast<std::size_t>(num_segments), inflated);
    return obstacle;
}

std::pair<double, double> sineExtrema(double t0, double t1,
                                      double center, double amplitude,
                                      double omega, double phase) {
    constexpr double pi = 3.141592653589793238462643383279502884;
    std::vector<double> candidates{t0, t1};
    const int k_min = static_cast<int>(std::ceil((omega * t0 + phase - pi / 2.0) / pi));
    const int k_max = static_cast<int>(std::floor((omega * t1 + phase - pi / 2.0) / pi));
    for (int k = k_min; k <= k_max; ++k) {
        const double t_ext = (pi / 2.0 + static_cast<double>(k) * pi - phase) / omega;
        if (t_ext >= t0 - 1e-12 && t_ext <= t1 + 1e-12) {
            candidates.push_back(t_ext);
        }
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (double t : candidates) {
        const double y = center + amplitude * std::sin(omega * t + phase);
        lo = std::min(lo, y);
        hi = std::max(hi, y);
    }
    return {lo, hi};
}

TimeIndexedObstacle sinusoidalObstacle(double horizon,
                                       int num_segments,
                                       const Vec3& ego_half,
                                       double x,
                                       double y_center,
                                       double y_amplitude,
                                       double omega,
                                       double z,
                                       double phase) {
    const Vec3 moving_half(0.105, 0.105, 0.060);
    const Vec3 total_half = moving_half + ego_half;
    TimeIndexedObstacle obstacle;
    obstacle.name = "sinusoid";
    for (int interval = 0; interval < num_segments; ++interval) {
        const double t0 = horizon * static_cast<double>(interval) / static_cast<double>(num_segments);
        const double t1 = horizon * static_cast<double>(interval + 1) / static_cast<double>(num_segments);
        const auto [y_min, y_max] = sineExtrema(t0, t1, y_center, y_amplitude, omega, phase);
        const Vec3 center(x, 0.5 * (y_min + y_max), z);
        const Vec3 half(total_half.x(), 0.5 * (y_max - y_min) + total_half.y(), total_half.z());
        obstacle.interval_vertices.push_back(boxVertices(center, half));
    }
    return obstacle;
}

}  // namespace

ValidationScenario makeValidationScenario(const std::string& name,
                                          int samples_per_axis) {
    if (samples_per_axis < 3 || samples_per_axis % 2 == 0) {
        throw std::invalid_argument("samples_per_axis must be an odd integer >= 3");
    }
    constexpr int num_segments = 4;
    constexpr double horizon = 7.291666666666666;
    const Vec3 ego_half(0.105, 0.105, 0.060);

    State state;
    state.position = Vec3(0.0, 0.0, 1.5);
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();

    ValidationScenario scenario;
    scenario.name = name;
    scenario.knots = openUniformKnots(0.0, horizon, num_segments);
    scenario.initial_control_points = initialControlPointsFromState(state, scenario.knots);
    scenario.goal = Vec3(2.25, 2.0, 1.5);

    if (name == "default") {
        scenario.obstacles.push_back(sinusoidalObstacle(
            horizon, num_segments, ego_half, 1.0, 0.5, 1.0, 0.3, 1.5, 0.0));
        scenario.obstacles.push_back(staticBox(
            "static-A", Vec3(1.00, 0.85, 1.50), Vec3(0.12, 0.12, 0.18), ego_half, num_segments));
        scenario.obstacles.push_back(staticBox(
            "static-B", Vec3(1.70, 0.45, 1.15), Vec3(0.16, 0.14, 0.15), ego_half, num_segments));
    } else if (name == "static_chicane") {
        scenario.obstacles.push_back(sinusoidalObstacle(
            horizon, num_segments, ego_half, 1.0, -1.4, 0.25, 0.20, 1.5, 0.0));
        scenario.obstacles.push_back(staticBox(
            "chicane-L", Vec3(0.78, 0.88, 1.50), Vec3(0.18, 0.34, 0.24), ego_half, num_segments));
        scenario.obstacles.push_back(staticBox(
            "chicane-R", Vec3(1.48, 1.10, 1.50), Vec3(0.22, 0.34, 0.24), ego_half, num_segments));
    } else if (name == "moving_crossing") {
        scenario.obstacles.push_back(sinusoidalObstacle(
            horizon, num_segments, ego_half, 1.05, 1.05, 0.70, 0.30, 1.5, -1.238125));
    } else {
        throw std::invalid_argument("unknown validation scene: " + name);
    }

    scenario.octopus_config.v_max = Vec3::Ones();
    scenario.octopus_config.a_max = Vec3::Constant(1.5);
    scenario.octopus_config.samples_per_axis = {
        samples_per_axis, samples_per_axis, samples_per_axis};
    scenario.octopus_config.alpha_shrink = 0.9;
    scenario.octopus_config.voxel_fraction = 0.10;
    scenario.octopus_config.heuristic_bias = 1.0;
    scenario.octopus_config.goal_tolerance_m = 0.05;
    scenario.octopus_config.max_runtime_s = 8.0;
    scenario.octopus_config.xyz_min = Vec3(-0.75, -0.75, 0.40);
    scenario.octopus_config.xyz_max = Vec3(3.00, 3.00, 2.60);
    scenario.octopus_config.planning_radius_m = 30.0;
    scenario.octopus_config.random_seed = 1;
    return scenario;
}

std::vector<std::string> validationScenarioNames() {
    return {"default", "static_chicane", "moving_crossing"};
}

}  // namespace dynamic_planner
