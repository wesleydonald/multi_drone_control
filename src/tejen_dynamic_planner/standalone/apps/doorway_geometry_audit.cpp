// Bounded offline geometry/timing fixture. No ROS, simulator or flight commands.
#include "dynamic_planner/receding_horizon_planner.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dynamic_planner;

WorldSnapshot doorway(double opening, SwingEnvelope shape) {
    WorldSnapshot world;
    world.ego_tracking_error_half_extents = Vec3(.1, .1, .05);
    auto& g = world.ego_suspended_geometry;
    g.enabled = g.payload_attached = true;
    g.swing_envelope = shape;
    g.cable_length_m = .575;
    g.max_swing_angle_rad = 10. * std::acos(-1.) / 180.;
    g.magnet_center_below_cable_end_m = 0.;
    g.payload_half_extents = Vec3(.043, .043, .011);
    g.payload_center_from_magnet_center = Vec3(0., 0., -.0315);
    world.workspace.enabled = true;
    world.workspace.lower = Vec3(-2.5, -2.5, 0.);
    world.workspace.upper = Vec3(2.5, 2.5, 1.6);
    const Vec3 w = Vec3(-1., 1., 0.).normalized();
    const auto rotation = Eigen::AngleAxisd(std::acos(-1.)/4., Vec3::UnitZ()).toRotationMatrix();
    const double outer = 3.7;
    for (int side : {-1, 1}) {
        const double a = side < 0 ? -outer : .3 + opening/2;
        const double b = side < 0 ? .3 - opening/2 : outer;
        auto vertices = axisAlignedBoxVertices(Vec3::Zero(), Vec3(.05, (b-a)/2, .8));
        vertices = (vertices * rotation.transpose()).eval();
        vertices.rowwise() += (w*((a+b)/2)+Vec3(0., 0., .8)).transpose();
        world.physical_static_obstacles.push_back({"panel_"+std::to_string(side), vertices});
    }
    return world;
}

int main() {
    try {
        using Clock = std::chrono::steady_clock;
        const Vec3 d = Vec3(1., 1., 0.).normalized(), w(-d.y(), d.x(), 0.);
        State start;
        start.position = -.6*d + .3*w + Vec3(0., 0., 1.35);
        const Vec3 goal = .6*d + .3*w + Vec3(0., 0., 1.35);
        // A known centre line fits the tight envelope, but not the old square.
        auto line = makeStationaryHoverTrajectory(start.position, 0., .1).pieces().front();
        line.control_points.resize(4, 3);
        line.knots = openUniformKnots(0., 1., 1);
        line.valid_from = 0.; line.valid_until = 1.;
        for (int i = 0; i < 4; ++i)
            line.control_points.row(i) = (start.position + (goal-start.position)*i/3.).transpose();
        TrajectorySafetyChecker checker;
        if (!checker.checkPiece(line, 0., 1., doorway(.65, SwingEnvelope::OuterOctagon)).safe ||
            checker.checkPiece(line, 0., 1., doorway(.65, SwingEnvelope::LegacySquare)).safe)
            throw std::runtime_error("650mm physical-space doorway discriminator failed");

        bool all_on_time = true;
        for (auto shape : {SwingEnvelope::LegacySquare, SwingEnvelope::OuterOctagon}) {
            auto snapshot = doorway(.9, shape); // identical feasible scene for timing
            snapshot.ego_body_rotation = Eigen::AngleAxisd(.05, Vec3::UnitY()).toRotationMatrix();
            std::size_t vertices = 0, accepted = 0, late = 0, lp = 0;
            double total_ms = 0., max_ms = 0.;
            for (const auto& obstacle : snapshot.timeIndexedObstacles(4, 1., 5.))
                vertices += obstacle.interval_vertices.front().rows();
            for (unsigned seed = 1; seed <= 10; ++seed) {
                VersionedWorld world(snapshot);
                RecedingHorizonConfig config;
                config.fixed_splice_lookahead_s = .1;
                config.spline_time_factor = 3.;
                LocalPlannerConfig local;
                local.octopus.max_runtime_s = .05;
                local.octopus.random_seed = seed;
                local.octopus.xyz_min = local.refinement.xyz_min = Vec3(-2.5, -2.5, 0.);
                local.octopus.xyz_max = local.refinement.xyz_max = Vec3(2.5, 2.5, 1.6);
                RecedingHorizonPlanner planner(goal, config, local);
                planner.initializeCommittedTrajectory(makeStationaryHoverTrajectory(start.position, 0., .1));
                const auto begin = Clock::now();
                const auto result = planner.replan(1., start, world, [&] {
                    return 1. + std::chrono::duration<double>(Clock::now()-begin).count();
                });
                const double elapsed = 1e3*std::chrono::duration<double>(Clock::now()-begin).count();
                total_ms += elapsed; max_ms = std::max(max_ms, elapsed);
                accepted += result.accepted; late += result.candidate_late;
                if (result.local_plan) lp += result.local_plan->search.separator_lp_calls;
            }
            std::cout << (shape == SwingEnvelope::OuterOctagon ? "octagon" : "legacy")
                      << " requests=10 accepted=" << accepted << " late=" << late
                      << " mean_ms=" << total_ms/10 << " max_ms=" << max_ms
                      << " cspace_vertices_per_interval=" << vertices << " search_lp_calls=" << lp
                      << " search_budget_ms=50 splice_ms=100\n";
            all_on_time = all_on_time && accepted == 10 && late == 0;
        }
        return all_on_time ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
