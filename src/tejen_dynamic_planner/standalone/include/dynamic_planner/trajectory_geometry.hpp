#pragma once

#include <Eigen/Core>

#include <vector>

#include "dynamic_planner/committed_trajectory.hpp"

namespace dynamic_planner {

struct ConvexCurveInterval {
    double start_s = 0.0;
    double end_s = 0.0;
    Eigen::MatrixXd vertices;
};

// Exact cubic Bezier control points for a sub-interval that lies entirely
// inside one polynomial span of a TrajectoryPiece.
Eigen::Matrix<double, 4, 3, Eigen::RowMajor> cubicBezierSubcurveVertices(
    const TrajectoryPiece& piece,
    double start_s,
    double end_s);

// Split at spline knots / committed-piece boundaries and return an exact
// convex-hull representation for every cubic subcurve. A certified terminal
// hold is represented by four coincident vertices.
std::vector<ConvexCurveInterval> convexCurveIntervals(
    const TrajectoryPiece& piece,
    double start_s,
    double end_s);

std::vector<ConvexCurveInterval> convexCurveIntervals(
    const CommittedTrajectory& trajectory,
    double start_s,
    double end_s);

// Concatenate the interval vertices into one conservative convex hull vertex
// set covering the whole requested time interval.
Eigen::MatrixXd convexCurveHullVertices(
    const CommittedTrajectory& trajectory,
    double start_s,
    double end_s);

// Minkowski-sum a vertex set with an axis-aligned box. The returned vertices
// exactly describe the convex hull of the input hull plus the AABB.
Eigen::MatrixXd inflateVerticesByAabb(const Eigen::MatrixXd& vertices,
                                      const Vec3& half_extents);

}  // namespace dynamic_planner
