#pragma once

#include <Eigen/Core>
#include <vector>

namespace dynamic_planner {

using Matrix4 = Eigen::Matrix4d;
using Matrix3 = Eigen::Matrix3d;
using FourPoints = Eigen::Matrix<double, 4, 3, Eigen::RowMajor>;
using ThreePoints = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

std::vector<Matrix4> positionConverters(int num_segments);
std::vector<Matrix3> velocityConverters(int num_segments);

FourPoints minvoVertices(const FourPoints& bspline_control_points,
                         int interval_index,
                         int num_segments);

ThreePoints minvoVelocityVertices(const ThreePoints& velocity_control_points,
                                  int interval_index,
                                  int num_segments);

}  // namespace dynamic_planner
