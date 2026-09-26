#include "tejen_dynamic_planner/c1e_scene.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "dynamic_planner/separator.hpp"

namespace tejen_dynamic_planner {
namespace {

using dynamic_planner::ConvexComponent;
using dynamic_planner::Separator;
using dynamic_planner::StaticConvexObstacle;
using dynamic_planner::Vec3;

Eigen::MatrixXd centreVertices(const Vec3& start, const Vec3& goal) {
    Eigen::MatrixXd vertices(2, 3);
    vertices.row(0) = start.transpose();
    vertices.row(1) = goal.transpose();
    return vertices;
}

Eigen::MatrixXd singleCentreVertex(const Vec3& centre) {
    Eigen::MatrixXd vertices(1, 3);
    vertices.row(0) = centre.transpose();
    return vertices;
}

bool componentSeparated(
    const Eigen::MatrixXd& centre_hull,
    const ConvexComponent& component,
    const StaticConvexObstacle& obstacle,
    Separator* separator) {
    const Eigen::MatrixXd occupied = dynamic_planner::sweptComponentHull(
        centre_hull, component);
    return separator->solve(occupied, obstacle.vertices).feasible;
}

bool assemblySeparated(
    const Eigen::MatrixXd& centre_hull,
    const std::vector<ConvexComponent>& components,
    const StaticConvexObstacle& obstacle,
    Separator* separator) {
    for (const auto& component : components) {
        if (!componentSeparated(centre_hull, component, obstacle, separator)) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::string C1eSceneWitness::summary() const {
    std::ostringstream out;
    out << "start_assembly_safe=" << start_assembly_safe
        << " goal_assembly_safe=" << goal_assembly_safe
        << " body_direct_safe=" << body_direct_safe
        << " suspended_direct_unsafe=" << suspended_direct_unsafe
        << " blocking_component="
        << (blocking_component.empty() ? "none" : blocking_component);
    return out.str();
}

StaticConvexObstacle makeYawedCuboidObstacle(
    const std::string& name,
    const Vec3& centre,
    const Vec3& half_extents,
    double yaw_rad) {
    if (name.empty() || !centre.allFinite() || !half_extents.allFinite() ||
        (half_extents.array() <= 0.0).any() || !std::isfinite(yaw_rad)) {
        throw std::invalid_argument("invalid C.1e yawed-cuboid geometry");
    }

    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    Eigen::MatrixXd vertices(8, 3);
    Eigen::Index row = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            for (double sz : {-1.0, 1.0}) {
                const Vec3 local(
                    sx * half_extents.x(),
                    sy * half_extents.y(),
                    sz * half_extents.z());
                const Vec3 rotated(
                    c * local.x() - s * local.y(),
                    s * local.x() + c * local.y(),
                    local.z());
                vertices.row(row++) = (centre + rotated).transpose();
            }
        }
    }
    return StaticConvexObstacle{name, std::move(vertices)};
}

C1eSceneWitness evaluateC1eSceneWitness(
    const Vec3& start,
    const Vec3& goal,
    const Vec3& body_half_extents,
    const dynamic_planner::SuspendedGeometry& suspended_geometry,
    const StaticConvexObstacle& physical_obstacle,
    double separator_validation_tolerance) {
    if (!start.allFinite() || !goal.allFinite() || !body_half_extents.allFinite() ||
        (body_half_extents.array() < 0.0).any() ||
        !std::isfinite(separator_validation_tolerance) ||
        separator_validation_tolerance < 0.0 ||
        physical_obstacle.name.empty() || physical_obstacle.vertices.rows() < 4 ||
        physical_obstacle.vertices.cols() != 3 ||
        !physical_obstacle.vertices.allFinite()) {
        throw std::invalid_argument("invalid C.1e scene-witness input");
    }
    suspended_geometry.validate();
    if (!suspended_geometry.enabled) {
        throw std::invalid_argument("C.1e scene witness requires suspended geometry");
    }

    const auto components = dynamic_planner::assemblyComponents(
        body_half_extents, suspended_geometry);
    if (components.size() < 2U || components.front().name != "body") {
        throw std::logic_error("unexpected suspended assembly component ordering");
    }

    Separator separator(separator_validation_tolerance);
    C1eSceneWitness witness;
    witness.start_assembly_safe = assemblySeparated(
        singleCentreVertex(start), components, physical_obstacle, &separator);
    witness.goal_assembly_safe = assemblySeparated(
        singleCentreVertex(goal), components, physical_obstacle, &separator);

    const Eigen::MatrixXd direct = centreVertices(start, goal);
    witness.body_direct_safe = componentSeparated(
        direct, components.front(), physical_obstacle, &separator);
    for (std::size_t i = 1U; i < components.size(); ++i) {
        if (!componentSeparated(direct, components[i], physical_obstacle, &separator)) {
            witness.suspended_direct_unsafe = true;
            witness.blocking_component = components[i].name;
            break;
        }
    }
    return witness;
}

}  // namespace tejen_dynamic_planner
