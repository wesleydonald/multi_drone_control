#include "tejen_dynamic_planner/experiment_workspace.hpp"
#include "dynamic_planner/committed_trajectory.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    using namespace dynamic_planner;
    SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = .575;
    geometry.magnet_center_below_cable_end_m = 0.;
    geometry.payload_attached = true;
    geometry.payload_half_extents = Vec3(.043, .043, .011);
    geometry.payload_center_from_magnet_center = Vec3(0., 0., -.0315);
    auto bounds = tejen_dynamic_planner::assemblyWorkspaceBounds(
        Vec3(-2.5, -2.5, 0.), Vec3(2.5, 2.5, 3.5), Vec3(.105, .105, .06),
        geometry, Vec3(.1, .1, .05));
    if (std::abs(bounds.first.z()-.6675) > 1e-9 ||
        std::abs(bounds.second.z()-3.39) > 1e-9 || bounds.first.x() >= -1.85)
        throw std::runtime_error("workspace must contain assembly plus unchanged tracking allowance");
    bool rejected = false;
    try {
        tejen_dynamic_planner::assemblyWorkspaceBounds(Vec3::Zero(), Vec3::Constant(.1),
            Vec3(.105, .105, .06), geometry, Vec3::Constant(.05));
    } catch (const std::invalid_argument&) { rejected = true; }
    if (!rejected) throw std::runtime_error("empty body workspace must reject");
    // The Python generator advertises native central intervals of a uniform cubic,
    // not individually clamped pieces. Exercise the real consumer's C2 contract.
    CommittedTrajectory curve;
    for (int j = 0; j < 8; ++j) {
        TrajectoryPiece piece;
        piece.valid_from = 100. + .25*j;
        piece.valid_until = piece.valid_from + .25;
        piece.control_points.resize(4, 3);
        for (int k = -3; k <= 4; ++k) piece.knots.push_back(100. + .25*(j+k));
        for (int k = -1; k <= 2; ++k) {
            const double x = .25*(j+k);
            piece.control_points.row(k+1) = Vec3(x, x*x, 1.);
        }
        piece.validate();
        if (!curve.replaceSuffix(piece.valid_from, piece).accepted)
            throw std::runtime_error("uniform cubic rolling pieces must join C2");
    }
    const auto sample = curve.evaluate(100.625);
    if (std::abs(sample.position.x()-.625) > 1e-9 ||
        std::abs(sample.velocity.x()-1.) > 1e-9)
        throw std::runtime_error("native uniform cubic time/evaluation disagrees");
    std::cout << "Experiment workspace and native cubic checks pass\n";
}
