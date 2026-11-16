#include "tejen_dynamic_planner/c1e_scene.hpp"

#include "dynamic_planner/ego_collision_model.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using dynamic_planner::SuspendedGeometry;
using dynamic_planner::Vec3;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

SuspendedGeometry loadedGeometry(bool payload_attached) {
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 0.50;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = 15.0 * kPi / 180.0;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    geometry.payload_attached = payload_attached;
    geometry.payload_half_extents = Vec3(0.05, 0.01, 0.003);
    geometry.payload_center_from_magnet_center = Vec3(0.0, 0.0, -0.028);
    return geometry;
}

}  // namespace

int main() {
    try {
        const Vec3 body_half(0.105, 0.105, 0.060);
        const SuspendedGeometry loaded = loadedGeometry(true);
        const SuspendedGeometry unloaded = loadedGeometry(false);

        const auto loaded_components = dynamic_planner::assemblyComponents(body_half, loaded);
        const auto unloaded_components = dynamic_planner::assemblyComponents(body_half, unloaded);
        require(loaded_components.size() == 4U,
                "C1F.3 loaded geometry must contain body+cable+magnet+payload");
        require(loaded_components.back().name == "payload",
                "C1F.3 loaded geometry is missing the payload component");
        require(unloaded_components.size() == 3U,
                "C1F.3 unloaded geometry must omit the payload component");

        // Runtime commissioning scene. The target in fake_cooperative_transport_world
        // circles around (2,2), so exercise eight representative target phases. The
        // straight body sweep must remain clear while at least one suspended component
        // blocks the same direct route. Endpoints must remain safe.
        const Vec3 start(0.023, 0.002, 0.906);
        const auto runtime_obstacle = tejen_dynamic_planner::makeYawedCuboidObstacle(
            "c1f3a_midpath_cuboid", Vec3(0.60, 0.60, 0.80),
            Vec3(0.18, 0.30, 0.08), 20.0 * kPi / 180.0);
        const std::vector<Vec3> target_samples = {
            Vec3(2.50, 2.00, 2.475), Vec3(2.35, 2.35, 2.475),
            Vec3(2.00, 2.50, 2.475), Vec3(1.65, 2.35, 2.475),
            Vec3(1.50, 2.00, 2.475), Vec3(1.65, 1.65, 2.475),
            Vec3(2.00, 1.50, 2.475), Vec3(2.35, 1.65, 2.475),
        };
        for (const auto& target : target_samples) {
            const auto witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
                start, target, body_half, loaded, runtime_obstacle);
            if (!witness.passed()) {
                throw std::runtime_error(
                    "C1F.3 runtime scene witness failed for a representative moving-target phase: " +
                    witness.summary());
            }
        }

        // Payload-toggle regression. This intentionally thin obstacle lies below the
        // conservative magnet envelope but inside the attached payload envelope along
        // the centre of a horizontal transit. With payload disabled, no suspended
        // component blocks the route; with payload enabled, the blocker is payload.
        const Vec3 payload_probe_start(-0.50, 0.0, 1.50);
        const Vec3 payload_probe_goal(0.50, 0.0, 1.50);
        const auto payload_probe = tejen_dynamic_planner::makeYawedCuboidObstacle(
            "c1f3a_payload_toggle_probe", Vec3(0.0, 0.0, 0.946),
            Vec3(0.03, 0.03, 0.0005), 0.0);
        const auto attached_witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
            payload_probe_start, payload_probe_goal, body_half, loaded, payload_probe);
        require(attached_witness.passed(),
                "attached-payload toggle witness should block the direct route");
        require(attached_witness.blocking_component == "payload",
                "attached-payload toggle witness should identify payload as blocker");

        const auto detached_witness = tejen_dynamic_planner::evaluateC1eSceneWitness(
            payload_probe_start, payload_probe_goal, body_half, unloaded, payload_probe);
        require(!detached_witness.suspended_direct_unsafe,
                "detached-payload toggle witness should not report a suspended blocker");
        require(detached_witness.blocking_component.empty(),
                "detached-payload toggle witness should have no blocker");

        std::cout << "test_c1f3a_scene: PASS\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& exc) {
        std::cerr << "test_c1f3a_scene: FAIL: " << exc.what() << '\n';
        return EXIT_FAILURE;
    }
}
