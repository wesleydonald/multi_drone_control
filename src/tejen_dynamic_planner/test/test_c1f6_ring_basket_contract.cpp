#include "tejen_dynamic_planner/moving_basket_geometry.hpp"
#include "dynamic_planner/bspline.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::ControlPoints;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;
using tejen_dynamic_planner::MovingBasketGeometry;
using tejen_dynamic_planner::SegmentedRingGeometry;
using tejen_dynamic_planner::translatedCollisionTrajectory;
using tejen_dynamic_planner::translatedCollisionTrajectoryPiece;

void requireTrue(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + " actual=" + std::to_string(actual));
    }
}

void requireVecNear(const Vec3& actual,
                    const Vec3& expected,
                    double tolerance,
                    const std::string& message) {
    if ((actual - expected).cwiseAbs().maxCoeff() > tolerance) {
        throw std::runtime_error(message);
    }
}

TrajectoryPiece makeRingCentrePiece() {
    TrajectoryPiece piece;
    piece.knots = dynamic_planner::openUniformKnots(10.0, 12.0, 4);
    piece.control_points = ControlPoints(7, 3);
    piece.control_points <<
        1.0, 2.0, 1.50,
        1.1, 2.0, 1.50,
        1.2, 2.1, 1.50,
        1.4, 2.2, 1.52,
        1.6, 2.3, 1.55,
        1.8, 2.4, 1.55,
        2.0, 2.5, 1.55;
    piece.valid_from = 10.0;
    piece.valid_until = 12.0;
    piece.validate();
    return piece;
}

}  // namespace

int main() {
    try {
        // Agreed M1 ring/net envelope. z=0 is the steel attachment-plate plane;
        // the conservative basket/net collision volume extends down only.
        MovingBasketGeometry geometry;
        geometry.outer_diameter_m = 0.56;
        geometry.top_offset_m = 0.0;
        geometry.bottom_offset_m = -0.27;
        geometry.validate();

        requireVecNear(
            geometry.halfExtents(), Vec3(0.28, 0.28, 0.135), 1e-12,
            "ring/net collision half extents mismatch");
        requireVecNear(
            geometry.centerOffset(), Vec3(0.0, 0.0, -0.135), 1e-12,
            "ring/net collision centre must sit below attachment plane");

        // Collision geometry is a translated copy of the ring-centre commitment.
        // Target prediction must continue to consume the unmodified ring-centre future.
        const auto ring_centre = makeRingCentrePiece();
        const Vec3 collision_offset = geometry.centerOffset();
        const auto collision_piece = translatedCollisionTrajectoryPiece(
            ring_centre, collision_offset);

        requireNear(collision_piece.valid_from, ring_centre.valid_from, 0.0,
                    "collision translation changed valid_from");
        requireNear(collision_piece.valid_until, ring_centre.valid_until, 0.0,
                    "collision translation changed valid_until");
        requireTrue(collision_piece.knots == ring_centre.knots,
                    "collision translation changed knot timing");

        for (double t : {10.0, 10.3, 11.1, 12.0}) {
            const auto target_state = ring_centre.evaluate(t);
            const auto collision_state = collision_piece.evaluate(t);
            requireVecNear(
                collision_state.position,
                target_state.position + collision_offset,
                1e-11,
                "collision trajectory position offset mismatch");
            requireVecNear(
                collision_state.velocity,
                target_state.velocity,
                1e-11,
                "collision translation must preserve velocity");
            requireVecNear(
                collision_state.acceleration,
                target_state.acceleration,
                1e-10,
                "collision translation must preserve acceleration");
        }

        // Invalid/inverted vertical envelopes must be rejected.
        MovingBasketGeometry invalid = geometry;
        invalid.top_offset_m = -0.20;
        invalid.bottom_offset_m = 0.0;
        bool rejected = false;
        try {
            invalid.validate();
        } catch (const std::exception&) {
            rejected = true;
        }
        requireTrue(rejected, "inverted ring collision z envelope was not rejected");

        // M2D reuses the ring commitment for prediction but models the physical
        // fixture as the 24 hollow ring bars that Gazebo actually contains.
        SegmentedRingGeometry segmented;
        segmented.validate();
        requireTrue(segmented.segment_count == 24,
                    "M2D segmented ring must match the 24 Gazebo bars");
        requireNear(segmented.segment_center_radius_m, 0.25, 1e-12,
                    "M2D ring segment radius mismatch");
        requireNear(segmented.tangential_length_m, 0.068067840828, 1e-12,
                    "M2D ring tangential segment length mismatch");
        requireNear(segmented.radial_width_m, 0.060, 1e-12,
                    "M2D ring radial segment width mismatch");
        requireNear(segmented.height_m, 0.030, 1e-12,
                    "M2D ring segment height mismatch");
        requireNear(segmented.center_z_offset_m, -0.020, 1e-12,
                    "M2D ring segment z offset mismatch");

        const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        requireVecNear(segmented.segmentWorldCenterOffset(identity, 0),
                       Vec3(0.25, 0.0, -0.02), 1e-12,
                       "segment zero centre mismatch");
        const Vec3 segment0_half = segmented.segmentWorldHalfExtents(identity, 0);
        // Segment 0 is tangential (+y), so world x gets the padded radial half
        // extent and world y gets the padded tangential half extent.
        requireNear(segment0_half.x(), 0.035, 1e-12,
                    "segment zero radial half extent mismatch");
        requireNear(segment0_half.y(), 0.039033920414, 1e-12,
                    "segment zero tangential half extent mismatch");
        requireNear(segment0_half.z(), 0.020, 1e-12,
                    "segment zero vertical half extent mismatch");
        requireTrue(segmented.segment_center_radius_m - segment0_half.x() > 0.20,
                    "segmented physical ring unexpectedly fills the hollow centre");
        requireTrue(segmented.obstacleName(7) == "m2d_ring_segment_07",
                    "segmented obstacle witness naming mismatch");

        CommittedTrajectory committed;
        requireTrue(committed.replaceSuffix(ring_centre.valid_from, ring_centre).accepted,
                    "failed to seed ring-centre committed trajectory");
        const Vec3 segment_offset = segmented.segmentWorldCenterOffset(identity, 0);
        const auto segment_commit = translatedCollisionTrajectory(committed, segment_offset);
        for (double t : {10.0, 10.7, 11.4, 12.0}) {
            requireVecNear(
                segment_commit.evaluate(t).position,
                committed.evaluate(t).position + segment_offset,
                1e-11,
                "segmented moving obstacle changed target-trajectory geometry");
        }

        std::cout << "C1F.6 ring basket contract tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "C1F.6 ring basket contract test failed: " << error.what() << '\n';
        return 1;
    }
}
