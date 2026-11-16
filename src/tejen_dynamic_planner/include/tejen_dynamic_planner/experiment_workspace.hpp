#pragma once
#include "dynamic_planner/ego_collision_model.hpp"
#include <stdexcept>
#include <utility>

namespace tejen_dynamic_planner {
// Axis-aligned map workspace: erode by the same assembly and tracking box used
// by collision checks. This configures planning, not a runtime flight geofence.
inline std::pair<dynamic_planner::Vec3, dynamic_planner::Vec3> assemblyWorkspaceBounds(
    const dynamic_planner::Vec3& lower, const dynamic_planner::Vec3& upper,
    const dynamic_planner::Vec3& body_half,
    const dynamic_planner::SuspendedGeometry& suspended,
    const dynamic_planner::Vec3& tracking,
    const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity()) {
    dynamic_planner::AssemblyWorkspace workspace;
    workspace.enabled = true;
    workspace.lower = lower;
    workspace.upper = upper;
    return dynamic_planner::assemblyCentreBounds(workspace, body_half, suspended, tracking, rotation);
}
} // namespace tejen_dynamic_planner
