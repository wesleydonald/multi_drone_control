#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/octopus_search.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusConfig;
using dynamic_planner::OctopusResult;
using dynamic_planner::OctopusSearch;
using dynamic_planner::State;
using dynamic_planner::TimeIndexedObstacle;
using dynamic_planner::Vec3;

namespace {

Eigen::MatrixXd boxVertices(const Vec3& center, const Vec3& half) {
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
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

TimeIndexedObstacle farSinusoid(double horizon, int num_segments,
                                const Vec3& ego_half) {
    constexpr double x = 1.0;
    constexpr double y_center = -1.4;
    constexpr double y_amplitude = 0.25;
    constexpr double omega = 0.20;
    constexpr double z = 1.5;
    constexpr double phase = 0.0;
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

OctopusResult runScene(const std::string& scene, int samples) {
    constexpr int num_segments = 4;
    constexpr double horizon = 7.291666666666666;
    const Vec3 start_position(0.0, 0.0, 1.5);
    const Vec3 goal(2.25, 2.0, 1.5);
    const Vec3 ego_half(0.105, 0.105, 0.060);

    State state;
    state.position = start_position;
    state.velocity = Vec3::Zero();
    state.acceleration = Vec3::Zero();
    const auto knots = dynamic_planner::openUniformKnots(0.0, horizon, num_segments);
    const ControlPoints q012 = dynamic_planner::initialControlPointsFromState(state, knots);

    std::vector<TimeIndexedObstacle> obstacles;
    if (scene == "static_chicane") {
        obstacles.push_back(farSinusoid(horizon, num_segments, ego_half));
        obstacles.push_back(staticBox("chicane-L", Vec3(0.78, 0.88, 1.5),
                                      Vec3(0.18, 0.34, 0.24), ego_half, num_segments));
        obstacles.push_back(staticBox("chicane-R", Vec3(1.48, 1.10, 1.5),
                                      Vec3(0.22, 0.34, 0.24), ego_half, num_segments));
    } else if (scene != "empty") {
        throw std::invalid_argument("scene must be 'empty' or 'static_chicane'");
    }

    OctopusConfig config;
    config.v_max = Vec3::Ones();
    config.a_max = Vec3::Constant(1.5);
    config.samples_per_axis = {samples, samples, samples};
    config.alpha_shrink = 0.9;
    config.voxel_fraction = 0.10;
    config.heuristic_bias = 1.0;
    config.goal_tolerance_m = 0.05;
    config.max_runtime_s = 8.0;
    config.xyz_min = Vec3(-0.75, -0.75, 0.40);
    config.xyz_max = Vec3(3.00, 3.00, 2.60);
    config.planning_radius_m = 30.0;
    config.random_seed = 1;

    return OctopusSearch(knots, q012, goal, obstacles, config).search();
}

double maxAbs(const ControlPoints& points) {
    return points.cwiseAbs().maxCoeff();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string scene = "static_chicane";
        int samples = 7;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scene" && i + 1 < argc) {
                scene = argv[++i];
            } else if (arg == "--samples" && i + 1 < argc) {
                samples = std::stoi(argv[++i]);
            } else {
                throw std::invalid_argument("usage: octopus_demo [--scene empty|static_chicane] [--samples odd_integer]");
            }
        }
        if (samples < 3 || samples % 2 == 0) {
            throw std::invalid_argument("--samples must be an odd integer >= 3");
        }

        const OctopusResult result = runScene(scene, samples);
        std::cout << std::setprecision(15);
        std::cout << "R6.3A.3 native C++ Octopus demo\n";
        std::cout << "scene: " << scene << '\n';
        std::cout << "samples per axis: " << samples << " (" << samples * samples * samples << " combinations)\n";
        std::cout << "status: " << result.status << '\n';
        std::cout << "success: " << std::boolalpha << result.success << '\n';
        std::cout << "reached goal <5cm: " << result.reached_goal << '\n';
        if (result.goal_distance_m.has_value()) {
            std::cout << "goal distance: " << *result.goal_distance_m << " m\n";
        }
        std::cout << "expanded nodes: " << result.expanded_nodes << '\n';
        std::cout << "popped nodes: " << result.popped_nodes << '\n';
        std::cout << "separator LP calls: " << result.separator_lp_calls << '\n';
        std::cout << "search time: " << 1000.0 * result.search_time_s << " ms\n";
        std::cout << "voxel size: " << result.voxel_size_m << " m\n";
        std::cout << "final separators: " << result.separators.size() << '\n';

        if (result.control_points.has_value()) {
            const auto knots = dynamic_planner::openUniformKnots(0.0, 7.291666666666666, 4);
            const auto velocity = dynamic_planner::derivativeSpline(*result.control_points, knots, 3, 1);
            const auto acceleration = dynamic_planner::derivativeSpline(*result.control_points, knots, 3, 2);
            std::cout << "max |velocity derivative CP|: " << maxAbs(velocity.control_points) << " m/s\n";
            std::cout << "max |acceleration derivative CP|: " << maxAbs(acceleration.control_points) << " m/s^2\n";
            std::cout << "control points:\n" << *result.control_points << '\n';
        }

        std::cout << "\nSend this output plus ctest output back to ChatGPT.\n";
        return result.success ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "octopus_demo: FAIL: " << error.what() << '\n';
        return 1;
    }
}
