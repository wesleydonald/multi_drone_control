#pragma once

#include <string>

#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/frozen_world.hpp"

namespace tejen_dynamic_planner {

struct C1eSceneWitness {
    bool start_assembly_safe = false;
    bool goal_assembly_safe = false;
    bool body_direct_safe = false;
    bool suspended_direct_unsafe = false;
    std::string blocking_component;

    bool passed() const noexcept {
        return start_assembly_safe && goal_assembly_safe && body_direct_safe &&
               suspended_direct_unsafe;
    }

    std::string summary() const;
};

// Construct the exact eight physical vertices of a Z-yaw-rotated cuboid.
// The same centre, half extents and yaw are consumed by the C.1e Gazebo-world
// generator, keeping the planner and simulator on one geometry definition.
dynamic_planner::StaticConvexObstacle makeYawedCuboidObstacle(
    const std::string& name,
    const dynamic_planner::Vec3& centre,
    const dynamic_planner::Vec3& half_extents,
    double yaw_rad);

// Check that the commissioning scene demonstrates suspended-system planning:
// both endpoint assemblies are safe, the body-only straight sweep is safe, and
// at least one suspended component blocks the same straight sweep.
C1eSceneWitness evaluateC1eSceneWitness(
    const dynamic_planner::Vec3& start,
    const dynamic_planner::Vec3& goal,
    const dynamic_planner::Vec3& body_half_extents,
    const dynamic_planner::SuspendedGeometry& suspended_geometry,
    const dynamic_planner::StaticConvexObstacle& physical_obstacle,
    double separator_validation_tolerance = 1e-7);

}  // namespace tejen_dynamic_planner
