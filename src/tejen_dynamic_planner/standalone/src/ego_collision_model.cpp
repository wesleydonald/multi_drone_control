#include "dynamic_planner/ego_collision_model.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {
namespace {

void validateVertices(const Eigen::MatrixXd& vertices, const char* label) {
    if (vertices.rows() < 1 || vertices.cols() != 3 || !vertices.allFinite()) {
        throw std::invalid_argument(std::string("invalid ") + label + " vertices");
    }
}

void validateHalfExtents(const Vec3& half, const char* label) {
    if (!half.allFinite() || (half.array() < 0.0).any()) {
        throw std::invalid_argument(std::string("invalid ") + label + " half extents");
    }
}

ConvexComponent makeCableComponent(const SuspendedGeometry& g) {
    // Conservative square-pyramid envelope. Expressing lateral width versus
    // vertical depth uses tan(theta). Extending the wide base to z=-L is a tiny
    // conservative over-approximation of the exact finite-length swing cone.
    const double top_z = g.cable_radius_m;
    const double bottom_z = -g.cable_length_m - g.cable_radius_m;
    const double top_half = g.cable_radius_m;
    const double bottom_half = g.cable_radius_m +
                               (g.cable_length_m + g.cable_radius_m) *
                                   std::tan(g.max_swing_angle_rad);

    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            vertices.row(row++) = Vec3(sx * top_half, sy * top_half, top_z).transpose();
        }
    }
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            vertices.row(row++) =
                Vec3(sx * bottom_half, sy * bottom_half, bottom_z).transpose();
        }
    }
    return ConvexComponent{"cable", std::move(vertices)};
}

ConvexComponent makeMagnetComponent(const SuspendedGeometry& g) {
    const double lateral = g.cable_length_m * std::sin(g.max_swing_angle_rad);
    const double vertical_rise = g.cable_length_m *
                                 (1.0 - std::cos(g.max_swing_angle_rad));
    const double nominal_center_z = -g.cable_length_m -
                                    g.magnet_center_below_cable_end_m;
    const Vec3 min_corner(
        -g.magnet_half_extents.x() - lateral,
        -g.magnet_half_extents.y() - lateral,
        nominal_center_z - g.magnet_half_extents.z());
    const Vec3 max_corner(
        g.magnet_half_extents.x() + lateral,
        g.magnet_half_extents.y() + lateral,
        nominal_center_z + g.magnet_half_extents.z() + vertical_rise);
    return ConvexComponent{
        "magnet", axisAlignedBoxVerticesFromBounds(min_corner, max_corner)};
}

ConvexComponent makePayloadComponent(const SuspendedGeometry& g) {
    const double lateral = g.cable_length_m * std::sin(g.max_swing_angle_rad);
    const double vertical_rise = g.cable_length_m *
                                 (1.0 - std::cos(g.max_swing_angle_rad));
    const Vec3 magnet_nominal_center(
        0.0, 0.0,
        -g.cable_length_m - g.magnet_center_below_cable_end_m);
    const Vec3 nominal_center = magnet_nominal_center +
                                g.payload_center_from_magnet_center;
    Vec3 min_corner = nominal_center - g.payload_half_extents;
    Vec3 max_corner = nominal_center + g.payload_half_extents;
    min_corner.x() -= lateral;
    min_corner.y() -= lateral;
    max_corner.x() += lateral;
    max_corner.y() += lateral;
    max_corner.z() += vertical_rise;
    return ConvexComponent{
        "payload", axisAlignedBoxVerticesFromBounds(min_corner, max_corner)};
}

ConvexComponent makeAttachedTetherComponent(const AttachedTetherGeometry& g) {
    const Vec3 delta = g.plate_from_body - g.anchor_from_body;
    const double length = delta.norm();
    if (!(length > 1e-9)) {
        throw std::invalid_argument("attached tether endpoints must be distinct");
    }
    const Vec3 axis = delta / length;
    const Vec3 seed = std::abs(axis.z()) < 0.9 ? Vec3::UnitZ() : Vec3::UnitY();
    const Vec3 u = axis.cross(seed).normalized();
    const Vec3 v = axis.cross(u).normalized();
    const Vec3 start = g.anchor_from_body - g.radius_m * axis;
    const Vec3 end = g.plate_from_body + g.radius_m * axis;

    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const Vec3 centre = endpoint == 0 ? start : end;
        for (double su : {-1.0, 1.0}) {
            for (double sv : {-1.0, 1.0}) {
                vertices.row(row++) =
                    (centre + su * g.radius_m * u + sv * g.radius_m * v).transpose();
            }
        }
    }
    return ConvexComponent{"attached_tether", std::move(vertices)};
}

}  // namespace

void ConvexComponent::validate() const {
    if (name.empty()) {
        throw std::invalid_argument("collision component name must not be empty");
    }
    validateVertices(vertices, "collision component");
}

void SuspendedGeometry::validate() const {
    if (!std::isfinite(cable_length_m) || !(cable_length_m > 0.0) ||
        !std::isfinite(cable_radius_m) || !(cable_radius_m >= 0.0) ||
        !std::isfinite(max_swing_angle_rad) || !(max_swing_angle_rad >= 0.0) ||
        !(max_swing_angle_rad < 0.5 * 3.14159265358979323846) ||
        !std::isfinite(magnet_center_below_cable_end_m) ||
        !(magnet_center_below_cable_end_m >= 0.0)) {
        throw std::invalid_argument("invalid suspended cable/magnet geometry");
    }
    validateHalfExtents(magnet_half_extents, "magnet");
    validateHalfExtents(payload_half_extents, "payload");
    if (!payload_center_from_magnet_center.allFinite()) {
        throw std::invalid_argument("invalid payload centre offset");
    }
}

void AttachedTetherGeometry::validate() const {
    if (!anchor_from_body.allFinite() || !plate_from_body.allFinite() ||
        !std::isfinite(radius_m) || !(radius_m >= 0.0)) {
        throw std::invalid_argument("invalid attached tether geometry");
    }
    if (enabled && (plate_from_body - anchor_from_body).norm() <= 1e-9) {
        throw std::invalid_argument("enabled attached tether endpoints must be distinct");
    }
}

Eigen::MatrixXd axisAlignedBoxVertices(const Vec3& center, const Vec3& half_extents) {
    if (!center.allFinite()) {
        throw std::invalid_argument("invalid box centre");
    }
    validateHalfExtents(half_extents, "box");
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            for (double sz : {-1.0, 1.0}) {
                vertices.row(row++) =
                    (center + Vec3(sx * half_extents.x(),
                                   sy * half_extents.y(),
                                   sz * half_extents.z())).transpose();
            }
        }
    }
    return vertices;
}

Eigen::MatrixXd axisAlignedBoxVerticesFromBounds(const Vec3& min_corner,
                                                 const Vec3& max_corner) {
    if (!min_corner.allFinite() || !max_corner.allFinite() ||
        (max_corner.array() < min_corner.array()).any()) {
        throw std::invalid_argument("invalid box bounds");
    }
    return axisAlignedBoxVertices(
        0.5 * (min_corner + max_corner), 0.5 * (max_corner - min_corner));
}

Eigen::MatrixXd minkowskiSumVertices(const Eigen::MatrixXd& a,
                                     const Eigen::MatrixXd& b) {
    validateVertices(a, "Minkowski lhs");
    validateVertices(b, "Minkowski rhs");
    Eigen::MatrixXd result(a.rows() * b.rows(), 3);
    Eigen::Index row = 0;
    for (Eigen::Index i = 0; i < a.rows(); ++i) {
        for (Eigen::Index j = 0; j < b.rows(); ++j) {
            result.row(row++) = a.row(i) + b.row(j);
        }
    }
    return result;
}

Eigen::MatrixXd negateVertices(const Eigen::MatrixXd& vertices) {
    validateVertices(vertices, "negated");
    return -vertices;
}

std::vector<ConvexComponent> assemblyComponents(
    const Vec3& body_half_extents,
    const SuspendedGeometry& suspended) {
    validateHalfExtents(body_half_extents, "body");
    suspended.validate();
    std::vector<ConvexComponent> components;
    components.push_back(ConvexComponent{
        "body", axisAlignedBoxVertices(Vec3::Zero(), body_half_extents)});
    if (!suspended.enabled) {
        return components;
    }
    components.push_back(makeCableComponent(suspended));
    components.push_back(makeMagnetComponent(suspended));
    if (suspended.payload_attached) {
        components.push_back(makePayloadComponent(suspended));
    }
    for (const auto& component : components) {
        component.validate();
    }
    return components;
}

std::vector<ConvexComponent> cooperativeAssemblyComponents(
    const Vec3& body_half_extents,
    const SuspendedGeometry& suspended,
    const AttachedTetherGeometry& attached_tether) {
    attached_tether.validate();
    auto components = assemblyComponents(body_half_extents, suspended);
    if (attached_tether.enabled) {
        components.push_back(makeAttachedTetherComponent(attached_tether));
        components.back().validate();
    }
    return components;
}

void componentAabb(const ConvexComponent& component, Vec3* center, Vec3* half_extents) {
    if (center == nullptr || half_extents == nullptr) {
        throw std::invalid_argument("component AABB outputs must not be null");
    }
    component.validate();
    const Vec3 min_corner = component.vertices.colwise().minCoeff().transpose();
    const Vec3 max_corner = component.vertices.colwise().maxCoeff().transpose();
    *center = 0.5 * (min_corner + max_corner);
    *half_extents = 0.5 * (max_corner - min_corner);
}

Eigen::MatrixXd physicalObstacleToConfigurationSpace(
    const Eigen::MatrixXd& physical_obstacle_vertices,
    const ConvexComponent& ego_component) {
    validateVertices(physical_obstacle_vertices, "physical obstacle");
    ego_component.validate();
    return minkowskiSumVertices(
        physical_obstacle_vertices, negateVertices(ego_component.vertices));
}

Eigen::MatrixXd sweptComponentHull(const Eigen::MatrixXd& centre_hull,
                                   const ConvexComponent& component) {
    validateVertices(centre_hull, "centre hull");
    component.validate();
    return minkowskiSumVertices(centre_hull, component.vertices);
}

}  // namespace dynamic_planner
