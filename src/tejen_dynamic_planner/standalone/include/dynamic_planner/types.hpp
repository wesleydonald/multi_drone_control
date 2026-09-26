#pragma once

#include <Eigen/Core>

namespace dynamic_planner {

using Vec3 = Eigen::Vector3d;

struct State {
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    Vec3 acceleration = Vec3::Zero();
};

}  // namespace dynamic_planner
