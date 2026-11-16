#include "dynamic_planner/frozen_world.hpp"

#include <stdexcept>

namespace dynamic_planner {

std::vector<TimeIndexedObstacle> FrozenWorld::timeIndexedObstacles(int num_segments) const {
    if (num_segments < 1) {
        throw std::invalid_argument("num_segments must be positive");
    }
    std::vector<TimeIndexedObstacle> result;
    result.reserve(obstacles.size());
    for (const auto& obstacle : obstacles) {
        if (obstacle.name.empty() || obstacle.vertices.rows() < 1 ||
            obstacle.vertices.cols() != 3 || !obstacle.vertices.allFinite()) {
            throw std::invalid_argument("invalid frozen-world obstacle");
        }
        TimeIndexedObstacle timed;
        timed.name = obstacle.name;
        timed.interval_vertices.assign(static_cast<std::size_t>(num_segments), obstacle.vertices);
        result.push_back(std::move(timed));
    }
    return result;
}

}  // namespace dynamic_planner
