#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include "dynamic_planner/types.hpp"

namespace dynamic_planner {

struct ConvexComponent {
    std::string name;
    Eigen::MatrixXd vertices;  // relative to the vehicle centre

    void validate() const;
};

// R6.3B.4 commissioning abstraction for a taut, nominally vertical suspended
// assembly. The 10-degree cone is an operating-envelope assumption, not a full
// pendulum-state model. Body dimensions remain supplied separately so B.3 body-
// only geometry stays byte-compatible at the API level.
struct SuspendedGeometry {
    bool enabled = false;

    double cable_length_m = 0.50;
    double cable_radius_m = 0.0025;
    double max_swing_angle_rad = 0.17453292519943295;  // 10 deg

    // Approximate 10 cm diameter mocap/magnet disk. The nominal cable endpoint is
    // the marker/disk reference; the assembly centre is placed slightly below it.
    Vec3 magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    double magnet_center_below_cable_end_m = 0.025;

    bool payload_attached = false;
    Vec3 payload_half_extents = Vec3(0.05, 0.01, 0.003);
    Vec3 payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);

    void validate() const;
};

// M2D attached-peer tether: a real straight segment between the peer vehicle
// tether anchor and its assigned ring plate, both expressed relative to the peer
// vehicle centre.  This is intentionally separate from SuspendedGeometry, which
// models a nominally vertical free-swinging payload cable.
struct AttachedTetherGeometry {
    bool enabled = false;
    Vec3 anchor_from_body = Vec3(0.0, 0.0, -0.04);
    Vec3 plate_from_body = Vec3(0.0, 0.0, -0.50);
    double radius_m = 0.015;

    void validate() const;
};

Eigen::MatrixXd axisAlignedBoxVertices(const Vec3& center, const Vec3& half_extents);
Eigen::MatrixXd axisAlignedBoxVerticesFromBounds(const Vec3& min_corner,
                                                 const Vec3& max_corner);
Eigen::MatrixXd minkowskiSumVertices(const Eigen::MatrixXd& a,
                                     const Eigen::MatrixXd& b);
Eigen::MatrixXd negateVertices(const Eigen::MatrixXd& vertices);

// Relative occupied components. When suspended geometry is disabled this returns
// only the body component, preserving B.3 semantics.
std::vector<ConvexComponent> assemblyComponents(
    const Vec3& body_half_extents,
    const SuspendedGeometry& suspended);

// Cooperative peer components add the endpoint-derived M2D tether when enabled.
// The tether prism conservatively contains a radius-r capsule around the segment.
std::vector<ConvexComponent> cooperativeAssemblyComponents(
    const Vec3& body_half_extents,
    const SuspendedGeometry& suspended,
    const AttachedTetherGeometry& attached_tether);

// Axis-aligned conservative bounds of one relative component. Used for efficient
// cooperative inter-assembly C-space construction; static physical obstacles keep
// the tighter component polytope itself.
void componentAabb(const ConvexComponent& component, Vec3* center, Vec3* half_extents);

// Physical static obstacle -> forbidden vehicle-centre region for one component.
Eigen::MatrixXd physicalObstacleToConfigurationSpace(
    const Eigen::MatrixXd& physical_obstacle_vertices,
    const ConvexComponent& ego_component);

// Exact convex swept hull of a centre-trajectory hull plus a fixed relative
// component polytope.
Eigen::MatrixXd sweptComponentHull(const Eigen::MatrixXd& centre_hull,
                                   const ConvexComponent& component);

}  // namespace dynamic_planner
