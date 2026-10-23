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
    const dynamic_planner::Vec3& tracking) {
    using dynamic_planner::Vec3;
    if (!lower.allFinite() || !upper.allFinite() || !tracking.allFinite() ||
        (lower.array() >= upper.array()).any() || (tracking.array() < 0.).any())
        throw std::invalid_argument("invalid experiment workspace/tracking bounds");
    Vec3 relative_min = -body_half, relative_max = body_half;
    for (const auto& component : dynamic_planner::assemblyComponents(body_half, suspended)) {
        relative_min = relative_min.cwiseMin(component.vertices.colwise().minCoeff().transpose());
        relative_max = relative_max.cwiseMax(component.vertices.colwise().maxCoeff().transpose());
    }
    Vec3 lo = lower-relative_min+tracking, hi = upper-relative_max-tracking;
    if ((lo.array() >= hi.array()).any())
        throw std::invalid_argument("assembly does not fit experiment workspace");
    return {lo, hi};
}
} // namespace tejen_dynamic_planner
