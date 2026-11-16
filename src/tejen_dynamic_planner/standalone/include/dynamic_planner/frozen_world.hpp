#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include "dynamic_planner/octopus_search.hpp"

namespace dynamic_planner {

// B.1 intentionally uses an immutable/frozen world. Vertices are already
// configuration-space inflated by the caller, matching the A-stack contract.
struct StaticConvexObstacle {
    std::string name;
    Eigen::MatrixXd vertices;
};

struct FrozenWorld {
    std::vector<StaticConvexObstacle> obstacles;

    std::vector<TimeIndexedObstacle> timeIndexedObstacles(int num_segments) const;
};

}  // namespace dynamic_planner
