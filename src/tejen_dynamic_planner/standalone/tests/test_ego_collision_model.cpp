#include "dynamic_planner/ego_collision_model.hpp"
#include "test_common.hpp"

#include <cmath>
#include <iostream>
#include <Eigen/Geometry>

using namespace dynamic_planner;

int main() {
    try {
        SuspendedGeometry geometry;
        geometry.enabled = true;
        geometry.cable_length_m = 0.50;
        geometry.cable_radius_m = 0.0025;
        geometry.max_swing_angle_rad = 10.0 * 3.14159265358979323846 / 180.0;
        geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
        geometry.magnet_center_below_cable_end_m = 0.025;

        const Vec3 body_half(0.105, 0.105, 0.060);
        const Eigen::Quaterniond attitude(
            Eigen::AngleAxisd(0.4, Vec3::UnitZ()) *
            Eigen::AngleAxisd(-0.2, Vec3::UnitY()) *
            Eigen::AngleAxisd(0.3, Vec3::UnitX()));
        const auto rotated = assemblyComponents(body_half, geometry, attitude.toRotationMatrix());
        requireTrue((rotated.front().vertices -
            axisAlignedBoxVertices(Vec3::Zero(), body_half) *
                attitude.toRotationMatrix().transpose()).norm() < 1e-12,
            "body must follow full measured rotation");
        requireTrue((rotated[1].vertices - assemblyComponents(body_half, geometry)[1].vertices).norm() == 0.,
            "body attitude must not rotate the hanging cable");
        auto octagonal = geometry;
        octagonal.swing_envelope = SwingEnvelope::OuterOctagon;
        const auto tight = assemblyComponents(body_half, octagonal);
        const Vec3 diagonal = Vec3(1., 1., 0.).normalized();
        const auto support = [&](const ConvexComponent& c, const Vec3& axis) {
            return (c.vertices * axis).maxCoeff();
        };
        const double disk = geometry.cable_length_m * std::sin(geometry.max_swing_angle_rad);
        requireNear(support(tight[2], diagonal),
            diagonal.x() * .05 + diagonal.y() * .05 + disk, 1e-12,
            "octagon diagonal support must contain disk without square corners");
        for (int i = 0; i < 360; ++i) {
            const double a = i * 3.14159265358979323846 / 180.;
            const Vec3 axis(std::cos(a), std::sin(a), 0.);
            requireTrue(support(tight[2], axis) + 1e-12 >=
                .05 * (std::abs(axis.x()) + std::abs(axis.y())) + disk,
                "outer octagon must contain the complete swing disk");
            requireTrue(support(tight[1], axis) + 1e-12 >=
                geometry.cable_radius_m +
                (geometry.cable_length_m + geometry.cable_radius_m) *
                std::tan(geometry.max_swing_angle_rad),
                "octagonal cable must contain every azimuth of its base disk");
        }
        auto loaded_octagon = octagonal;
        loaded_octagon.payload_attached = true;
        const auto loaded = assemblyComponents(body_half, loaded_octagon);
        requireTrue(loaded.size() == 4 && loaded.back().vertices.rows() == 16,
                    "payload must use a bounded-size enclosing octagon too");
        const auto raw_sum = minkowskiSumVertices(loaded.back().vertices,
            axisAlignedBoxVertices(Vec3::Zero(), Vec3(.1, .1, .05)));
        const auto compact = compactHorizontalLayers(raw_sum);
        requireTrue(compact.rows() < raw_sum.rows(), "compaction did not remove redundant vertices");
        for (int i = 0; i < 180; ++i) {
            const Vec3 axis(std::cos(i*.73), std::sin(i*.73), std::sin(i*.37));
            requireNear((compact*axis).maxCoeff(), (raw_sum*axis).maxCoeff(), 1e-12,
                        "layer compaction changed physical support");
        }
        // A tall prism is not enough: the supplied legal centre interval must
        // keep this tracked component inside the obstacle's full Z interval.
        AssemblyWorkspace prism_workspace;
        prism_workspace.enabled=true;
        prism_workspace.lower=Vec3(-2.,-2.,0.);
        prism_workspace.upper=Vec3(2.,2.,1.6);
        auto tracked_body=rotated.front();
        tracked_body.vertices=minkowskiSumVertices(tracked_body.vertices,
            axisAlignedBoxVertices(Vec3::Zero(),Vec3(.1,.1,.05)));
        const auto panel=axisAlignedBoxVertices(Vec3(0.,0.,.8),Vec3(1.,1.,.8));
        const auto raw_configuration=physicalObstacleToConfigurationSpace(panel,tracked_body);
        const auto old_configuration=compactHorizontalLayers(raw_configuration);
        const std::pair<Vec3,Vec3> invalid_z_cover{Vec3(-1.,-1.,.4),Vec3(1.,1.,1.6)};
        requireMatrixNear(compactStaticConfigurationSpace(raw_configuration,panel,tracked_body,
            prism_workspace,invalid_z_cover),old_configuration,0.,
            "partial component Z coverage must not enable prism projection");
        auto disabled_workspace=prism_workspace;
        disabled_workspace.enabled=false;
        requireMatrixNear(compactStaticConfigurationSpace(raw_configuration,panel,tracked_body,
            disabled_workspace,{Vec3(-1.,-1.,.4),Vec3(1.,1.,1.2)}),old_configuration,0.,
            "disabled workspace must retain the original configuration hull");
        auto components = assemblyComponents(body_half, geometry);
        requireTrue(components.size() == 3U,
                    "unattached assembly should contain body+cable+magnet");
        requireTrue(components[0].name == "body", "body component order changed");
        requireTrue(components[1].name == "cable", "cable component missing");
        requireTrue(components[2].name == "magnet", "magnet component missing");

        Vec3 cable_center, cable_half;
        componentAabb(components[1], &cable_center, &cable_half);
        const double expected_base = geometry.cable_radius_m +
            (geometry.cable_length_m + geometry.cable_radius_m) *
                std::tan(geometry.max_swing_angle_rad);
        requireNear(cable_half.x(), expected_base, 1e-12,
                    "cable bounded-swing lateral envelope incorrect");
        requireNear(cable_half.y(), expected_base, 1e-12,
                    "cable bounded-swing lateral envelope incorrect");
        requireTrue(cable_center.z() < -0.24 && cable_center.z() > -0.26,
                    "cable envelope not centred down the 50 cm cable");

        Vec3 magnet_center, magnet_half;
        componentAabb(components[2], &magnet_center, &magnet_half);
        const double lateral = 0.50 * std::sin(geometry.max_swing_angle_rad);
        requireNear(magnet_half.x(), 0.05 + lateral, 1e-12,
                    "10 cm diameter magnet lateral envelope incorrect");
        requireNear(magnet_half.y(), 0.05 + lateral, 1e-12,
                    "10 cm diameter magnet lateral envelope incorrect");

        geometry.payload_attached = true;
        geometry.payload_half_extents = Vec3(0.05, 0.01, 0.003);
        components = assemblyComponents(body_half, geometry);
        requireTrue(components.size() == 4U,
                    "attached assembly should add payload component");
        requireTrue(components.back().name == "payload", "payload component missing");

        AttachedTetherGeometry tether;
        tether.enabled = true;
        tether.anchor_from_body = Vec3(0.0, 0.0, -0.04);
        tether.plate_from_body = Vec3(0.35, -0.20, -0.42);
        tether.radius_m = 0.015;
        const auto peer_components = cooperativeAssemblyComponents(
            body_half, SuspendedGeometry{}, tether);
        requireTrue(peer_components.size() == 2U,
                    "attached peer should contain body+tether");
        requireTrue(peer_components.back().name == "attached_tether",
                    "attached tether component missing");
        Vec3 tether_center, tether_half;
        componentAabb(peer_components.back(), &tether_center, &tether_half);
        const Vec3 segment_mid = 0.5 * (tether.anchor_from_body + tether.plate_from_body);
        requireTrue((tether_center - segment_mid).norm() < 0.02,
                    "attached tether prism is not centred on the real endpoint segment");
        for (int axis = 0; axis < 3; ++axis) {
            const double min_v = peer_components.back().vertices.col(axis).minCoeff();
            const double max_v = peer_components.back().vertices.col(axis).maxCoeff();
            requireTrue(tether.anchor_from_body(axis) >= min_v - 1e-12 &&
                        tether.anchor_from_body(axis) <= max_v + 1e-12,
                        "tether prism does not contain anchor endpoint");
            requireTrue(tether.plate_from_body(axis) >= min_v - 1e-12 &&
                        tether.plate_from_body(axis) <= max_v + 1e-12,
                        "tether prism does not contain plate endpoint");
        }

        // C-space sign sanity: a component centred below the vehicle must move
        // the corresponding forbidden vehicle-centre region upward.
        ConvexComponent below_box{
            "below", axisAlignedBoxVertices(Vec3(0.0, 0.0, -0.50),
                                             Vec3(0.01, 0.01, 0.01))};
        const Eigen::MatrixXd obstacle = axisAlignedBoxVertices(
            Vec3(0.0, 0.0, 0.0), Vec3(0.02, 0.02, 0.02));
        const Eigen::MatrixXd cspace = physicalObstacleToConfigurationSpace(
            obstacle, below_box);
        requireTrue(cspace.col(2).minCoeff() > 0.46,
                    "below-body component did not shift forbidden region upward");
        requireTrue(cspace.col(2).maxCoeff() < 0.54,
                    "unexpected C-space vertical extent");

        std::cout << "test_ego_collision_model: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_ego_collision_model: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
