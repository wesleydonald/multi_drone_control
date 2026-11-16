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

bool fullHeightVerticalPrism(const Eigen::MatrixXd& vertices,
                            const AssemblyWorkspace& workspace) {
    if (!workspace.enabled || vertices.rows() < 6) return false;
    const double lower = vertices.col(2).minCoeff();
    const double upper = vertices.col(2).maxCoeff();
    if (!(lower < upper) || lower > workspace.lower.z() || upper < workspace.upper.z())
        return false;
    bool nonzero_xy_area = false;
    for (Eigen::Index j = 1; j < vertices.rows() && !nonzero_xy_area; ++j) {
        for (Eigen::Index k = j + 1; k < vertices.rows(); ++k) {
            const long double ax = static_cast<long double>(vertices(j, 0)) - vertices(0, 0);
            const long double ay = static_cast<long double>(vertices(j, 1)) - vertices(0, 1);
            const long double bx = static_cast<long double>(vertices(k, 0)) - vertices(0, 0);
            const long double by = static_cast<long double>(vertices(k, 1)) - vertices(0, 1);
            if (ax * by - ay * bx != 0.) {
                nonzero_xy_area = true;
                break;
            }
        }
    }
    if (!nonzero_xy_area) return false;
    // Height of an AABB alone is insufficient. Every XY vertex must occur at
    // both exact endpoint heights, with no unmatched or intermediate layer.
    for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
        if (vertices(i, 2) != lower && vertices(i, 2) != upper) return false;
        bool paired = false;
        for (Eigen::Index j = 0; j < vertices.rows(); ++j) {
            if (vertices(i, 0) == vertices(j, 0) && vertices(i, 1) == vertices(j, 1) &&
                vertices(i, 2) != vertices(j, 2)) {
                paired = true;
                break;
            }
        }
        if (!paired) return false;
    }
    return true;
}

// Intersection of adjacent support lines of an outer octagon plus a physical
// rectangle. Both polygons have normals in this set, so no hull library or
// pairwise-vertex explosion is needed for their exact Minkowski sum.
Eigen::MatrixXd octagonLayer(double radius, double hx, double hy, double z) {
    Eigen::MatrixXd vertices(8, 3);
    constexpr double step = 3.14159265358979323846 / 4.;
    for (int i = 0; i < 8; ++i) {
        const double a = i * step, b = (i + 1) * step;
        const double ax = std::cos(a), ay = std::sin(a);
        const double bx = std::cos(b), by = std::sin(b);
        const double ha = radius + hx * std::abs(ax) + hy * std::abs(ay);
        const double hb = radius + hx * std::abs(bx) + hy * std::abs(by);
        const double det = ax * by - ay * bx;
        vertices.row(i) = Vec3((ha * by - ay * hb) / det,
                               (ax * hb - ha * bx) / det, z).transpose();
    }
    return vertices;
}

ConvexComponent octagonalBox(const char* name, const Vec3& center,
                            const Vec3& half, double lateral, double rise) {
    Eigen::MatrixXd vertices(16, 3);
    vertices.topRows(8) = octagonLayer(lateral, half.x(), half.y(), -half.z());
    vertices.bottomRows(8) = octagonLayer(lateral, half.x(), half.y(), half.z() + rise);
    vertices.rowwise() += center.transpose();
    return ConvexComponent{name, std::move(vertices)};
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

    if (g.swing_envelope == SwingEnvelope::OuterOctagon) {
        Eigen::MatrixXd vertices(16, 3);
        vertices.topRows(8) = octagonLayer(top_half, 0., 0., top_z);
        vertices.bottomRows(8) = octagonLayer(bottom_half, 0., 0., bottom_z);
        return ConvexComponent{"cable", std::move(vertices)};
    }

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
    if (g.swing_envelope == SwingEnvelope::OuterOctagon)
        return octagonalBox("magnet", Vec3(0., 0., nominal_center_z),
                            g.magnet_half_extents, lateral, vertical_rise);
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
    if (g.swing_envelope == SwingEnvelope::OuterOctagon)
        return octagonalBox("payload", nominal_center, g.payload_half_extents,
                            lateral, vertical_rise);
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
    if (swing_envelope != SwingEnvelope::LegacySquare &&
        swing_envelope != SwingEnvelope::OuterOctagon)
        throw std::invalid_argument("invalid swing envelope selection");
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

Eigen::MatrixXd compactHorizontalLayers(const Eigen::MatrixXd& vertices) {
    validateVertices(vertices, "layer compaction");
    std::vector<Vec3> points;
    for (Eigen::Index i = 0; i < vertices.rows(); ++i) points.push_back(vertices.row(i).transpose());
    std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
        if (a.z() != b.z()) return a.z() < b.z();
        if (a.x() != b.x()) return a.x() < b.x();
        return a.y() < b.y();
    });
    points.erase(std::unique(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
        return a == b;
    }), points.end());
    const auto cross = [](const Vec3& a, const Vec3& b, const Vec3& c) {
        // Extended precision avoids cancelling almost collinear support points.
        return (static_cast<long double>(b.x())-a.x())*(static_cast<long double>(c.y())-a.y()) -
               (static_cast<long double>(b.y())-a.y())*(static_cast<long double>(c.x())-a.x());
    };
    std::vector<Vec3> result;
    for (std::size_t begin = 0; begin < points.size();) {
        std::size_t end = begin+1;
        while (end < points.size() && points[end].z() == points[begin].z()) ++end;
        if (end-begin < 3) {
            result.insert(result.end(), points.begin()+begin, points.begin()+end);
        } else {
            std::vector<Vec3> hull;
            for (std::size_t i = begin; i < end; ++i) {
                while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), points[i]) <= 0.) hull.pop_back();
                hull.push_back(points[i]);
            }
            const std::size_t lower_size = hull.size();
            for (std::size_t i = end-1; i-- > begin;) {
                while (hull.size() > lower_size && cross(hull[hull.size()-2], hull.back(), points[i]) <= 0.) hull.pop_back();
                hull.push_back(points[i]);
            }
            hull.pop_back();
            result.insert(result.end(), hull.begin(), hull.end());
        }
        begin = end;
    }
    Eigen::MatrixXd output(result.size(), 3);
    for (std::size_t i = 0; i < result.size(); ++i) output.row(i) = result[i].transpose();
    return output;
}

Eigen::MatrixXd compactStaticConfigurationSpace(
    const Eigen::MatrixXd& configuration_vertices,
    const Eigen::MatrixXd& physical_obstacle_vertices,
    const ConvexComponent& tracked_component,
    const AssemblyWorkspace& workspace,
    const std::pair<Vec3, Vec3>& centre_bounds) {
    validateVertices(configuration_vertices, "static configuration");
    validateVertices(physical_obstacle_vertices, "physical static obstacle");
    tracked_component.validate();
    workspace.validate();
    if (!centre_bounds.first.allFinite() || !centre_bounds.second.allFinite() ||
        (centre_bounds.first.array() > centre_bounds.second.array()).any())
        throw std::invalid_argument("invalid assembly-centre bounds for static compaction");
    const double physical_lower = physical_obstacle_vertices.col(2).minCoeff();
    const double physical_upper = physical_obstacle_vertices.col(2).maxCoeff();
    if (!fullHeightVerticalPrism(physical_obstacle_vertices, workspace) ||
        centre_bounds.first.z() + tracked_component.vertices.col(2).minCoeff() < physical_lower ||
        centre_bounds.second.z() + tracked_component.vertices.col(2).maxCoeff() > physical_upper)
        return compactHorizontalLayers(configuration_vertices);

    // Do not change XY coordinates, margins, component identity or raw Z extent.
    // The product projection is globally enclosing, not an identical 3D hull;
    // collision equivalence relies on the checked legal workspace Z interval.
    Eigen::MatrixXd prism(2 * configuration_vertices.rows(), 3);
    prism.topRows(configuration_vertices.rows()) = configuration_vertices;
    prism.bottomRows(configuration_vertices.rows()) = configuration_vertices;
    prism.topRows(configuration_vertices.rows()).col(2).setConstant(
        configuration_vertices.col(2).minCoeff());
    prism.bottomRows(configuration_vertices.rows()).col(2).setConstant(
        configuration_vertices.col(2).maxCoeff());
    return compactHorizontalLayers(prism);
}

std::vector<ConvexComponent> assemblyComponents(
    const Vec3& body_half_extents,
    const SuspendedGeometry& suspended,
    const Eigen::Matrix3d& body_rotation) {
    validateHalfExtents(body_half_extents, "body");
    validateBodyRotation(body_rotation);
    suspended.validate();
    std::vector<ConvexComponent> components;
    components.push_back(ConvexComponent{
        "body", axisAlignedBoxVertices(Vec3::Zero(), body_half_extents) * body_rotation.transpose()});
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

void validateBodyRotation(const Eigen::Matrix3d& rotation) {
    if (!rotation.allFinite() ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-9 ||
        std::abs(rotation.determinant() - 1.) > 1e-9)
        throw std::invalid_argument("invalid body rotation");
}

void AssemblyWorkspace::validate() const {
    if (enabled && (!lower.allFinite() || !upper.allFinite() ||
                    (lower.array() >= upper.array()).any()))
        throw std::invalid_argument("invalid assembly workspace");
}

std::pair<Vec3, Vec3> assemblyCentreBounds(
    const AssemblyWorkspace& workspace, const Vec3& body_half,
    const SuspendedGeometry& suspended, const Vec3& tracking,
    const Eigen::Matrix3d& rotation) {
    workspace.validate();
    validateHalfExtents(tracking, "tracking");
    const auto components = assemblyComponents(body_half, suspended, rotation);
    Vec3 relative_min = components.front().vertices.colwise().minCoeff().transpose();
    Vec3 relative_max = components.front().vertices.colwise().maxCoeff().transpose();
    for (const auto& component : components) {
        relative_min = relative_min.cwiseMin(component.vertices.colwise().minCoeff().transpose());
        relative_max = relative_max.cwiseMax(component.vertices.colwise().maxCoeff().transpose());
    }
    const Vec3 lower = workspace.lower - relative_min + tracking;
    const Vec3 upper = workspace.upper - relative_max - tracking;
    if ((lower.array() >= upper.array()).any())
        throw std::invalid_argument("assembly does not fit workspace");
    return {lower, upper};
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
