// Ego body positions inside his cooperative C-space hulls at R4374's stall state (hover, s9).
#include "dynamic_planner/world_snapshot.hpp"
#include "dynamic_planner/separator.hpp"
#include "dynamic_planner/ego_collision_model.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace dynamic_planner;

CommittedTrajectory still(const Vec3& p, double t0, double t1) {
    // VX, VY (m/s): the carriers translate together at constant velocity (the circling ring, locally)
    const Vec3 v(getenv("VX") ? std::atof(getenv("VX")) : 0.0, getenv("VY") ? std::atof(getenv("VY")) : 0.0, 0.0);
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    for (int i = 0; i < 4; ++i) piece.control_points.row(i) = (p + v * ((t1 - t0) * i / 3.0)).transpose();
    piece.valid_from = t0; piece.valid_until = t1;
    CommittedTrajectory r; r.replaceSuffix(t0, piece); return r;
}

int main(int argc, char** argv) {
    const bool octagon = argc > 1 && std::string(argv[1]) == "octagon";
    const double swing_deg = argc > 2 ? std::atof(argv[2]) : 5.0;
    const double track = argc > 3 ? std::atof(argv[3]) : 0.06;
    const double xprobe = argc > 4 ? std::atof(argv[4]) : 0.0;
    WorldSnapshot w;
    w.ego_half_extents = Vec3(0.105, 0.105, 0.060);
    w.ego_tracking_error_half_extents = Vec3::Constant(0.05);
    SuspendedGeometry g;
    g.enabled = true; g.cable_length_m = getenv("CABLE") ? std::atof(getenv("CABLE")) : 1.04; g.cable_radius_m = 0.0025;
    g.max_swing_angle_rad = swing_deg * M_PI / 180.0;
    g.magnet_half_extents = Vec3(0.05, 0.05, 0.025); g.magnet_center_below_cable_end_m = 0.025;
    g.payload_attached = true; { const double ph = getenv("PH") ? std::atof(getenv("PH")) : 0.08, pz = getenv("PZ") ? std::atof(getenv("PZ")) : 0.08; g.payload_half_extents = Vec3(ph, ph, pz); }
    g.payload_center_from_magnet_center = Vec3(0, 0, getenv("PC") ? std::atof(getenv("PC")) : -0.08);
    g.swing_envelope = octagon ? SwingEnvelope::OuterOctagon : SwingEnvelope::LegacySquare;
    w.ego_suspended_geometry = g;
    const Vec3 ring(0.0001, -0.0004, 1.1988);
    const Vec3 bodies[3] = {Vec3(-0.0003, -0.7333, 2.1243), Vec3(0.6347, 0.3661, 2.1243), Vec3(-0.6344, 0.3665, 2.1243)};
    for (int i = 0; i < 3; ++i) {
        CooperativeObstacleTrajectory o;
        o.name = "cooperative_drone_" + std::to_string(i);
        o.trajectory = still(bodies[i], 0.0, 10.0);
        o.physical_half_extents = Vec3(0.105, 0.105, 0.060);
        o.tracking_error_half_extents = Vec3::Constant(track);
        o.suspended_geometry.enabled = false;
        Vec3 dir = bodies[i] - ring; dir.z() = 0; dir.normalize();
        const Vec3 plate = ring + 0.225 * dir;
        o.attached_tether_geometry.enabled = true;
        o.attached_tether_geometry.anchor_from_body = Vec3(0, 0, -0.04);
        o.attached_tether_geometry.plate_from_body = plate - bodies[i];
        o.attached_tether_geometry.radius_m = 0.015;
        w.cooperative_obstacles.push_back(o);
    }
    w.validate();
    const double horizon = getenv("HORIZON") ? std::atof(getenv("HORIZON")) : 4.0;   // 4 intervals over this
    const auto obs = w.timeIndexedObstacles(4, 0.0, horizon);
    Separator sep;
    auto inside = [&](const Vec3& p, std::string* who) {
        Eigen::MatrixXd pt(1, 3); pt.row(0) = p.transpose();
        for (const auto& o : obs) {
            const auto r = sep.solve(pt, o.interval_vertices[0]);
            if (!r.feasible) { if (who) *who = o.name; return true; }
        }
        return false;
    };
    std::printf("mode=%s swing=%.1f deg track=%.3f x=%.2f  (rows z, cols y from -0.40 to 1.00; # = inside a hull, G = drop point, Q = stall)\n",
                octagon ? "outer_octagon" : "legacy_square", swing_deg, track, xprobe);
    for (double z = 3.90; z > 2.19; z -= 0.05) {
        std::printf("z %.2f ", z);
        for (double y = -0.40; y <= 1.0001; y += 0.025) {
            const bool g_ = std::abs(y) < 0.0126 && std::abs(z - 2.879) < 0.026;
            const bool q_ = std::abs(y - 0.668) < 0.0126 && std::abs(z - 2.879) < 0.026;
            const bool in = inside(Vec3(xprobe, y, z), nullptr);
            std::putchar(g_ ? (in ? 'G' : 'g') : q_ ? (in ? 'Q' : 'q') : in ? '#' : '.');
        }
        std::putchar('\n');
    }
    std::string who;
    for (double y : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.65, 0.7}) {
        const bool in = inside(Vec3(xprobe, y, 2.879), &who);
        std::printf("y %.2f z 2.879: %s %s\n", y, in ? "INSIDE" : "free", in ? who.c_str() : "");
    }
    return 0;
}
