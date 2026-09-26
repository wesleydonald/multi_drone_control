#include "dynamic_planner/trajectory_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {
namespace {

constexpr double kTimeTolerance = kTrajectoryTimeToleranceS;

void appendUniqueBreakpoint(std::vector<double>& values, double value) {
    if (values.empty() || std::abs(values.back() - value) > kTimeTolerance) {
        values.push_back(value);
    }
}

std::vector<double> pieceBreakpoints(const TrajectoryPiece& piece,
                                     double start_s,
                                     double end_s) {
    piece.validate();
    if (!std::isfinite(start_s) || !std::isfinite(end_s) ||
        end_s < start_s - kTimeTolerance ||
        start_s < piece.valid_from - kTimeTolerance ||
        end_s > piece.valid_until + kTimeTolerance) {
        throw std::invalid_argument("invalid TrajectoryPiece subcurve interval");
    }
    std::vector<double> points;
    points.push_back(start_s);
    for (double knot : piece.knots) {
        if (knot > start_s + kTimeTolerance && knot < end_s - kTimeTolerance) {
            if (points.empty() || std::abs(points.back() - knot) > kTimeTolerance) {
                points.push_back(knot);
            }
        }
    }
    if (end_s > start_s + kTimeTolerance) {
        appendUniqueBreakpoint(points, end_s);
    }
    return points;
}

Eigen::MatrixXd concatenateIntervals(const std::vector<ConvexCurveInterval>& intervals) {
    Eigen::Index rows = 0;
    for (const auto& interval : intervals) {
        rows += interval.vertices.rows();
    }
    if (rows == 0) {
        return Eigen::MatrixXd(0, 3);
    }
    Eigen::MatrixXd result(rows, 3);
    Eigen::Index offset = 0;
    for (const auto& interval : intervals) {
        result.middleRows(offset, interval.vertices.rows()) = interval.vertices;
        offset += interval.vertices.rows();
    }
    return result;
}

}  // namespace

Eigen::Matrix<double, 4, 3, Eigen::RowMajor> cubicBezierSubcurveVertices(
    const TrajectoryPiece& piece,
    double start_s,
    double end_s) {
    if (!(end_s > start_s) || !std::isfinite(start_s) || !std::isfinite(end_s)) {
        throw std::invalid_argument("Bezier subcurve requires positive finite duration");
    }
    const auto points = pieceBreakpoints(piece, start_s, end_s);
    if (points.size() != 2U) {
        throw std::invalid_argument(
            "Bezier subcurve interval crosses a spline knot; split it first");
    }
    const State start = piece.evaluate(start_s);
    const State end = piece.evaluate(end_s);
    const double dt = end_s - start_s;

    Eigen::Matrix<double, 4, 3, Eigen::RowMajor> vertices;
    vertices.row(0) = start.position.transpose();
    vertices.row(1) = (start.position + (dt / 3.0) * start.velocity).transpose();
    vertices.row(2) = (end.position - (dt / 3.0) * end.velocity).transpose();
    vertices.row(3) = end.position.transpose();
    return vertices;
}

std::vector<ConvexCurveInterval> convexCurveIntervals(
    const TrajectoryPiece& piece,
    double start_s,
    double end_s) {
    std::vector<ConvexCurveInterval> result;
    if (end_s <= start_s + kTimeTolerance) {
        return result;
    }
    const auto points = pieceBreakpoints(piece, start_s, end_s);
    for (std::size_t i = 0; i + 1U < points.size(); ++i) {
        if (points[i + 1U] <= points[i] + kTimeTolerance) {
            continue;
        }
        ConvexCurveInterval interval;
        interval.start_s = points[i];
        interval.end_s = points[i + 1U];
        interval.vertices = cubicBezierSubcurveVertices(
            piece, interval.start_s, interval.end_s);
        result.push_back(std::move(interval));
    }
    return result;
}

std::vector<ConvexCurveInterval> convexCurveIntervals(
    const CommittedTrajectory& trajectory,
    double start_s,
    double end_s) {
    if (trajectory.empty()) {
        throw std::invalid_argument("cannot build hulls from an empty committed trajectory");
    }
    if (!std::isfinite(start_s) || !std::isfinite(end_s) ||
        end_s < start_s - kTimeTolerance ||
        start_s < trajectory.startTime() - kTimeTolerance) {
        throw std::invalid_argument("invalid committed-trajectory hull interval");
    }
    if (end_s <= start_s + kTimeTolerance) {
        return {};
    }
    if (end_s > trajectory.endTime() + kTimeTolerance &&
        !trajectory.endsInStoppedHold()) {
        throw std::invalid_argument(
            "requested hull extends beyond a trajectory without a stopped hold");
    }

    std::vector<ConvexCurveInterval> result;
    for (const auto& piece : trajectory.pieces()) {
        const double a = std::max(start_s, piece.valid_from);
        const double b = std::min(end_s, piece.valid_until);
        if (b <= a + kTimeTolerance) {
            continue;
        }
        auto intervals = convexCurveIntervals(piece, a, b);
        result.insert(result.end(), intervals.begin(), intervals.end());
    }

    if (end_s > trajectory.endTime() + kTimeTolerance) {
        const double a = std::max(start_s, trajectory.endTime());
        const double b = end_s;
        if (b > a + kTimeTolerance) {
            ConvexCurveInterval hold;
            hold.start_s = a;
            hold.end_s = b;
            hold.vertices.resize(4, 3);
            const Vec3 p = trajectory.endState().position;
            for (Eigen::Index row = 0; row < 4; ++row) {
                hold.vertices.row(row) = p.transpose();
            }
            result.push_back(std::move(hold));
        }
    }

    if (result.empty()) {
        throw std::runtime_error("requested committed interval has no curve coverage");
    }
    return result;
}

Eigen::MatrixXd convexCurveHullVertices(
    const CommittedTrajectory& trajectory,
    double start_s,
    double end_s) {
    return concatenateIntervals(convexCurveIntervals(trajectory, start_s, end_s));
}

Eigen::MatrixXd inflateVerticesByAabb(const Eigen::MatrixXd& vertices,
                                      const Vec3& half_extents) {
    if (vertices.cols() != 3 || vertices.rows() < 1 || !vertices.allFinite() ||
        !half_extents.allFinite() || (half_extents.array() < 0.0).any()) {
        throw std::invalid_argument("invalid vertices/AABB inflation");
    }
    Eigen::MatrixXd inflated(vertices.rows() * 8, 3);
    Eigen::Index row = 0;
    for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
        const Vec3 center = vertices.row(i).transpose();
        for (double sx : {-1.0, 1.0}) {
            for (double sy : {-1.0, 1.0}) {
                for (double sz : {-1.0, 1.0}) {
                    const Vec3 corner = center + Vec3(
                        sx * half_extents.x(),
                        sy * half_extents.y(),
                        sz * half_extents.z());
                    inflated.row(row++) = corner.transpose();
                }
            }
        }
    }
    return inflated;
}

}  // namespace dynamic_planner
