#include "dynamic_planner/ego_collision_model.hpp"
#include "test_common.hpp"

#include <cmath>
#include <iostream>

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
