// Te-Jen's C-space hulls (his WorldSnapshot + Separator) on recorded rig states (12 Nov flights).
// Adapted from ~/Downloads/tejen_hover_stall_2026-11-12/probe/probe.cpp: carriers, ring and QUAD4 from a file.
// usage: probe2 legacy|octagon <delta_s> <input> [corridor 0|1]
// input lines: t phase ego(K x 3 points over [t, t+delta]) drop(K x 3) ring(x y z qw qx qy qz) vel(vx vy vz)
//              3 x (body x y z) 3 x plate_id      (K = 5)
// delta 0: static (each carrier still); delta > 0: each carrier moves at vel, his 4 intervals over 4*delta,
// interval 0 = [0, delta] is checked against the hull of QUAD4's flown points over [t, t+delta].
#include "dynamic_planner/world_snapshot.hpp"
#include "dynamic_planner/separator.hpp"
#include "dynamic_planner/ego_collision_model.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
using namespace dynamic_planner;

static CommittedTrajectory moving(const Vec3& p, const Vec3& v, double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    for (int i = 0; i < 4; ++i) piece.control_points.row(i) = (p + v * ((t1 - t0) * i / 3.0)).transpose();
    piece.valid_from = t0; piece.valid_until = t1;
    CommittedTrajectory r; r.replaceSuffix(t0, piece); return r;
}

int main(int argc, char** argv) {  // probe3: map mode
    if (argc < 4) { std::fprintf(stderr, "usage: probe2 legacy|octagon delta input [corridor]\n"); return 2; }
    const bool octagon = std::string(argv[1]) == "octagon";
    const double delta = std::atof(argv[2]);
    const bool corridor = argc > 4 && std::atoi(argv[4]) != 0;
    const double swing = (getenv("SWING_DEG") ? std::atof(getenv("SWING_DEG")) : 5.0) * M_PI / 180.0;
    const double ctrack = getenv("CTRACK") ? std::atof(getenv("CTRACK")) : 0.06;
    const int K = 5;
    std::ifstream in(argv[3]);
    std::string line;
    Separator sep;
    std::printf("t phase ego_in ego_who drop_in drop_who left_m right_m drop_floor_z\n");
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        double t; std::string phase;
        ss >> t >> phase;
        Eigen::MatrixXd ego(K, 3), drop(K, 3);
        for (int k = 0; k < K; ++k) ss >> ego(k, 0) >> ego(k, 1) >> ego(k, 2);
        for (int k = 0; k < K; ++k) ss >> drop(k, 0) >> drop(k, 1) >> drop(k, 2);
        Vec3 ring; double qw, qx, qy, qz; Vec3 vel;
        ss >> ring.x() >> ring.y() >> ring.z() >> qw >> qx >> qy >> qz >> vel.x() >> vel.y() >> vel.z();
        Vec3 bodies[3]; int plates[3];
        for (int i = 0; i < 3; ++i) ss >> bodies[i].x() >> bodies[i].y() >> bodies[i].z();
        for (int i = 0; i < 3; ++i) ss >> plates[i];
        if (!ss) { std::fprintf(stderr, "bad line at t=%f\n", t); continue; }
        const Eigen::Matrix3d R = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();

        WorldSnapshot w;
        w.ego_half_extents = Vec3(0.105, 0.105, 0.060);
        w.ego_tracking_error_half_extents = Vec3::Constant(0.05);
        SuspendedGeometry g;
        g.enabled = true; g.cable_length_m = 1.034; g.cable_radius_m = 0.0025;
        g.max_swing_angle_rad = swing;
        g.magnet_half_extents = Vec3(0.05, 0.05, 0.025); g.magnet_center_below_cable_end_m = 0.025;
        g.payload_attached = true;
        g.payload_half_extents = Vec3(0.0525, 0.0525, 0.025);
        g.payload_center_from_magnet_center = Vec3(0, 0, -0.0475);
        g.swing_envelope = octagon ? SwingEnvelope::OuterOctagon : SwingEnvelope::LegacySquare;
        w.ego_suspended_geometry = g;
        const Vec3 v = delta > 0.0 ? vel : Vec3::Zero();
        for (int i = 0; i < 3; ++i) {
            CooperativeObstacleTrajectory o;
            o.name = "drone_" + std::to_string(i);
            o.trajectory = moving(bodies[i], v, 0.0, 20.0);
            o.physical_half_extents = Vec3(0.105, 0.105, 0.060);
            o.tracking_error_half_extents = Vec3::Constant(ctrack);
            o.suspended_geometry.enabled = false;
            const double th = 2.0 * M_PI * plates[i] / 12.0;   // his cooperativePlateWorldPosition, angle_zero 0
            const Vec3 plate = ring + R * Vec3(0.225 * std::cos(th), 0.225 * std::sin(th), 0.0);
            o.attached_tether_geometry.enabled = true;
            o.attached_tether_geometry.anchor_from_body = Vec3(0, 0, -0.04);
            o.attached_tether_geometry.plate_from_body = plate - bodies[i];
            o.attached_tether_geometry.radius_m = 0.015;
            w.cooperative_obstacles.push_back(o);
        }
        w.validate();
        const double horizon = delta > 0.0 ? 4.0 * delta : 4.0;
        const auto obs = w.timeIndexedObstacles(4, 0.0, horizon);
        auto inside = [&](const Eigen::MatrixXd& pts, std::string* who) {
            for (const auto& o : obs) {
                const auto r = sep.solveWithFastPaths(pts, o.interval_vertices[0]);
                if (!r.feasible) { if (who) *who = o.name; return true; }
            }
            return false;
        };
        const double Z = std::atof(getenv("MAPZ"));
        const double half = getenv("MAPHALF") ? std::atof(getenv("MAPHALF")) : 1.2;
        const double step = getenv("MAPSTEP") ? std::atof(getenv("MAPSTEP")) : 0.02;
        const Vec3 c = ring;
        const Vec3 vv = delta > 0.0 ? vel : Vec3::Zero();
        std::printf("# t %.2f z %.2f delta %.2f half %.2f step %.3f ring %.3f %.3f\n", t, Z, delta, half, step, c.x(), c.y());
        const int nh = static_cast<int>(std::lround(half / step));
        for (int iy = nh; iy >= -nh; --iy) { const double dy = iy * step;
            for (int ix = -nh; ix <= nh; ++ix) { const double dx = ix * step;
                Eigen::MatrixXd p(K, 3);
                for (int k = 0; k < K; ++k) {
                    const Vec3 q = Vec3(c.x() + dx, c.y() + dy, Z) + vv * (delta * k / (K - 1));
                    p.row(k) = q.transpose();
                }
                std::putchar(inside(p, nullptr) ? '#' : '.');
            }
            std::putchar('\n');
        }
        break;
    }
    return 0;
}
