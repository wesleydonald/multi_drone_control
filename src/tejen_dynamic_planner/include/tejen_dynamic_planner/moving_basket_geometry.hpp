#pragma once

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "dynamic_planner/committed_trajectory.hpp"

namespace tejen_dynamic_planner {

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;

struct MovingBasketGeometry {
    double outer_diameter_m = 0.56;
    double top_offset_m = 0.0;
    double bottom_offset_m = -0.27;

    void validate() const {
        if (!std::isfinite(outer_diameter_m) || !(outer_diameter_m > 0.0) ||
            !std::isfinite(top_offset_m) || !std::isfinite(bottom_offset_m) ||
            !(top_offset_m > bottom_offset_m)) {
            throw std::invalid_argument("invalid moving ring/net collision geometry");
        }
    }

    Vec3 halfExtents() const {
        validate();
        return Vec3(
            0.5 * outer_diameter_m,
            0.5 * outer_diameter_m,
            0.5 * (top_offset_m - bottom_offset_m));
    }

    Vec3 centerOffset() const {
        validate();
        return Vec3(0.0, 0.0, 0.5 * (top_offset_m + bottom_offset_m));
    }
};


struct SegmentedRingGeometry {
    int segment_count = 24;
    double segment_center_radius_m = 0.25;
    double tangential_length_m = 0.068067840828;
    double radial_width_m = 0.060;
    double height_m = 0.030;
    double center_z_offset_m = -0.020;
    // Small physical-geometry conservatism only. Tracking-error tubes remain
    // separate and unchanged in the world model.
    double padding_m = 0.005;

    void validate() const {
        if (segment_count < 3 ||
            !std::isfinite(segment_center_radius_m) || !(segment_center_radius_m > 0.0) ||
            !std::isfinite(tangential_length_m) || !(tangential_length_m > 0.0) ||
            !std::isfinite(radial_width_m) || !(radial_width_m > 0.0) ||
            !std::isfinite(height_m) || !(height_m > 0.0) ||
            !std::isfinite(center_z_offset_m) ||
            !std::isfinite(padding_m) || !(padding_m >= 0.0)) {
            throw std::invalid_argument("invalid segmented ring collision geometry");
        }
        if (padding_m >= segment_center_radius_m) {
            throw std::invalid_argument("segmented ring padding is unreasonably large");
        }
    }

    double segmentAngleRad(int index) const {
        validate();
        if (index < 0 || index >= segment_count) {
            throw std::out_of_range("segmented ring index out of range");
        }
        return 2.0 * std::acos(-1.0) * static_cast<double>(index) /
            static_cast<double>(segment_count);
    }

    Vec3 segmentCenterOffsetRing(int index) const {
        const double theta = segmentAngleRad(index);
        return Vec3(
            segment_center_radius_m * std::cos(theta),
            segment_center_radius_m * std::sin(theta),
            center_z_offset_m);
    }

    Eigen::Matrix3d segmentRotationRing(int index) const {
        const double yaw = segmentAngleRad(index) + 0.5 * std::acos(-1.0);
        return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
    }

    Vec3 paddedLocalHalfExtents() const {
        validate();
        return Vec3(
            0.5 * tangential_length_m + padding_m,
            0.5 * radial_width_m + padding_m,
            0.5 * height_m + padding_m);
    }

    Vec3 segmentWorldCenterOffset(
        const Eigen::Matrix3d& rotation_world_from_ring,
        int index) const {
        if (!rotation_world_from_ring.allFinite()) {
            throw std::invalid_argument("segmented ring rotation must be finite");
        }
        return rotation_world_from_ring * segmentCenterOffsetRing(index);
    }

    Vec3 segmentWorldHalfExtents(
        const Eigen::Matrix3d& rotation_world_from_ring,
        int index) const {
        if (!rotation_world_from_ring.allFinite()) {
            throw std::invalid_argument("segmented ring rotation must be finite");
        }
        const Eigen::Matrix3d rotation_world_from_segment =
            rotation_world_from_ring * segmentRotationRing(index);
        return rotation_world_from_segment.cwiseAbs() * paddedLocalHalfExtents();
    }

    std::string obstacleName(int index) const {
        (void)segmentAngleRad(index);
        char suffix[4];
        std::snprintf(suffix, sizeof(suffix), "%02d", index);
        return std::string("m2d_ring_segment_") + suffix;
    }
};

inline TrajectoryPiece translatedCollisionTrajectoryPiece(
    const TrajectoryPiece& ring_centre_piece,
    const Vec3& position_offset) {
    if (!position_offset.allFinite()) {
        throw std::invalid_argument("moving ring collision offset must be finite");
    }
    TrajectoryPiece translated = ring_centre_piece;
    for (Eigen::Index row = 0; row < translated.control_points.rows(); ++row) {
        translated.control_points.row(row) += position_offset.transpose();
    }
    translated.validate();
    return translated;
}


inline CommittedTrajectory translatedCollisionTrajectory(
    const CommittedTrajectory& ring_centre_trajectory,
    const Vec3& position_offset) {
    if (ring_centre_trajectory.empty() || !position_offset.allFinite()) {
        throw std::invalid_argument("moving ring collision trajectory requires finite non-empty input");
    }
    CommittedTrajectory translated;
    for (const auto& original_piece : ring_centre_trajectory.pieces()) {
        const TrajectoryPiece shifted = translatedCollisionTrajectoryPiece(
            original_piece, position_offset);
        const auto splice = translated.replaceSuffix(shifted.valid_from, shifted);
        if (!splice.accepted) {
            throw std::runtime_error("failed to translate moving ring committed trajectory");
        }
    }
    return translated;
}

}  // namespace tejen_dynamic_planner
