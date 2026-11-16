#include "dynamic_planner/temporal_collision_certificate.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dynamic_planner {
namespace {

constexpr double kTemporalTolerance = 1e-12;

void validateHull(const Eigen::MatrixXd& vertices, const char* label) {
    if (vertices.rows() < 1 || vertices.cols() != 3 || !vertices.allFinite()) {
        throw std::invalid_argument(std::string(label) + " must be finite with shape (N,3)");
    }
}

void validateConfig(const TemporalRefinementConfig& config) {
    if (config.max_depth < 0 || config.max_child_tests == 0U) {
        throw std::invalid_argument("temporal refinement depth/budget is invalid");
    }
}

bool aabbDisjoint(const Eigen::MatrixXd& first, const Eigen::MatrixXd& second) {
    validateHull(first, "first hull");
    validateHull(second, "second hull");
    const Vec3 first_min = first.colwise().minCoeff().transpose();
    const Vec3 first_max = first.colwise().maxCoeff().transpose();
    const Vec3 second_min = second.colwise().minCoeff().transpose();
    const Vec3 second_max = second.colwise().maxCoeff().transpose();
    for (int axis = 0; axis < 3; ++axis) {
        if (first_max(axis) < second_min(axis) - kTemporalTolerance ||
            second_max(axis) < first_min(axis) - kTemporalTolerance) {
            return true;
        }
    }
    return false;
}

Eigen::MatrixXd deCasteljauLeft(const Eigen::MatrixXd& control, double u) {
    Eigen::MatrixXd level = control;
    Eigen::MatrixXd left(4, control.cols());
    left.row(0) = level.row(0);
    for (int order = 1; order < 4; ++order) {
        // Evaluate before assigning because topRows()/bottomRows() alias
        // level; an in-place resize would otherwise corrupt the subdivision.
        const Eigen::MatrixXd next = (1.0 - u) * level.topRows(level.rows() - 1)
                                   + u * level.bottomRows(level.rows() - 1);
        level = next;
        left.row(order) = level.row(0);
    }
    return left;
}

Eigen::MatrixXd deCasteljauRight(const Eigen::MatrixXd& control, double u) {
    Eigen::MatrixXd level = control;
    Eigen::MatrixXd right(4, control.cols());
    right.row(3) = level.row(3);
    for (int order = 1; order < 4; ++order) {
        const Eigen::MatrixXd next = (1.0 - u) * level.topRows(level.rows() - 1)
                                   + u * level.bottomRows(level.rows() - 1);
        level = next;
        right.row(3 - order) = level.row(level.rows() - 1);
    }
    return right;
}

Eigen::MatrixXd restrictBezierRows(const Eigen::MatrixXd& parent,
                                   double parent_start_s,
                                   double parent_end_s,
                                   double child_start_s,
                                   double child_end_s) {
    if (parent.rows() != 4 || parent.cols() < 1 || !parent.allFinite() ||
        !std::isfinite(parent_start_s) || !std::isfinite(parent_end_s) ||
        !std::isfinite(child_start_s) || !std::isfinite(child_end_s) ||
        !(parent_end_s > parent_start_s) || !(child_end_s > child_start_s) ||
        child_start_s < parent_start_s - kTemporalTolerance ||
        child_end_s > parent_end_s + kTemporalTolerance) {
        throw std::invalid_argument("invalid cubic Bezier temporal restriction");
    }
    const double duration = parent_end_s - parent_start_s;
    const double u0 = std::clamp((child_start_s - parent_start_s) / duration, 0.0, 1.0);
    const double u1 = std::clamp((child_end_s - parent_start_s) / duration, 0.0, 1.0);
    if (!(u1 > u0 + kTemporalTolerance)) {
        throw std::invalid_argument("degenerate cubic Bezier child interval");
    }
    Eigen::MatrixXd restricted = parent;
    if (u0 > kTemporalTolerance) {
        restricted = deCasteljauRight(restricted, u0);
    }
    if (u1 < 1.0 - kTemporalTolerance) {
        const double relative_end = (u1 - u0) / (1.0 - u0);
        restricted = deCasteljauLeft(restricted, relative_end);
    }
    return restricted;
}

TemporalSeparationLeaf testSlice(const Eigen::MatrixXd& first,
                                 const Eigen::MatrixXd& second,
                                 double start_s,
                                 double end_s,
                                 int depth,
                                 bool materialize_planes,
                                 Separator* separator,
                                 TemporalRefinementStats* stats) {
    validateHull(first, "first hull");
    validateHull(second, "second hull");
    TemporalSeparationLeaf leaf;
    leaf.start_s = start_s;
    leaf.end_s = end_s;
    leaf.depth = depth;
    leaf.first_vertices = first;
    leaf.second_vertices = second;
    if (aabbDisjoint(first, second)) {
        if (!materialize_planes) {
            ++stats->aabb_separation_skips;
            leaf.separation.feasible = true;
            leaf.separation.status = "AABB_DISJOINT";
            return leaf;
        }
    }
    leaf.separation = separator->solve(first, second);
    ++stats->separator_lp_calls;
    return leaf;
}

std::vector<std::pair<double, double>> initialChildren(
    double start_s, double end_s, std::vector<double> mandatory_breakpoints_s) {
    std::vector<double> points;
    points.reserve(mandatory_breakpoints_s.size() + 2U);
    points.push_back(start_s);
    std::sort(mandatory_breakpoints_s.begin(), mandatory_breakpoints_s.end());
    for (double point : mandatory_breakpoints_s) {
        if (std::isfinite(point) && point > start_s + kTemporalTolerance &&
            point < end_s - kTemporalTolerance &&
            point > points.back() + kTemporalTolerance) {
            points.push_back(point);
        }
    }
    points.push_back(end_s);
    if (points.size() == 2U) {
        const double midpoint = 0.5 * (start_s + end_s);
        return {{start_s, midpoint}, {midpoint, end_s}};
    }
    std::vector<std::pair<double, double>> children;
    children.reserve(points.size() - 1U);
    for (std::size_t i = 0; i + 1U < points.size(); ++i) {
        children.emplace_back(points[i], points[i + 1U]);
    }
    return children;
}

}  // namespace

void accumulateTemporalRefinementStats(TemporalRefinementStats* destination,
                                       const TemporalRefinementStats& source) {
    if (destination == nullptr) {
        throw std::invalid_argument("temporal refinement destination is null");
    }
    destination->parent_tests += source.parent_tests;
    destination->parent_fast_path_accepts += source.parent_fast_path_accepts;
    destination->child_tests += source.child_tests;
    destination->max_depth_reached = std::max(
        destination->max_depth_reached, source.max_depth_reached);
    destination->accepted_leaves += source.accepted_leaves;
    destination->depth_or_budget_rejections += source.depth_or_budget_rejections;
    destination->separator_lp_calls += source.separator_lp_calls;
    destination->aabb_separation_skips += source.aabb_separation_skips;
    destination->wall_time_s += source.wall_time_s;
    destination->unresolved_leaves.insert(
        destination->unresolved_leaves.end(),
        source.unresolved_leaves.begin(), source.unresolved_leaves.end());
}

AdaptiveTemporalCertificateResult certifyAdaptiveTemporalSeparation(
    double parent_start_s,
    double parent_end_s,
    const Eigen::MatrixXd& parent_first_vertices,
    const Eigen::MatrixXd& parent_second_vertices,
    const TemporalHullBuilder& first_child_hull,
    const TemporalHullBuilder& second_child_hull,
    std::vector<double> mandatory_breakpoints_s,
    const TemporalRefinementConfig& config,
    double separator_validation_tolerance,
    bool materialize_planes,
    TemporalLeafDiagnosticBuilder unresolved_leaf_builder) {
    if (!std::isfinite(parent_start_s) || !std::isfinite(parent_end_s) ||
        !(parent_end_s > parent_start_s) || !first_child_hull || !second_child_hull ||
        !(separator_validation_tolerance > 0.0) || !std::isfinite(separator_validation_tolerance)) {
        throw std::invalid_argument("invalid adaptive temporal certificate request");
    }
    validateHull(parent_first_vertices, "parent first hull");
    validateHull(parent_second_vertices, "parent second hull");
    validateConfig(config);

    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    AdaptiveTemporalCertificateResult result;
    Separator separator(separator_validation_tolerance);
    const auto recordRejectedLeaf = [&](const TemporalSeparationLeaf& leaf,
                                        const std::string& reason) {
        TemporalLeafDiagnostic diagnostic;
        if (unresolved_leaf_builder) {
            diagnostic = unresolved_leaf_builder(leaf);
        }
        diagnostic.start_s = leaf.start_s;
        diagnostic.end_s = leaf.end_s;
        diagnostic.depth = leaf.depth;
        diagnostic.limit_reason = reason;
        result.stats.unresolved_leaves.push_back(std::move(diagnostic));
    };
    ++result.stats.parent_tests;
    const TemporalSeparationLeaf parent = testSlice(
        parent_first_vertices, parent_second_vertices, parent_start_s, parent_end_s,
        0, materialize_planes, &separator, &result.stats);
    if (parent.separation.feasible) {
        result.certified = true;
        result.status = "PARENT_SEPARATED";
        ++result.stats.parent_fast_path_accepts;
        ++result.stats.accepted_leaves;
        result.accepted_leaves.push_back(parent);
        result.stats.wall_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return result;
    }

    const auto fail = [&](std::string status,
                          std::optional<TemporalSeparationLeaf> rejected) {
        result.certified = false;
        result.status = std::move(status);
        result.rejected_leaf = std::move(rejected);
        result.stats.wall_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return result;
    };
    if (config.max_depth == 0) {
        ++result.stats.depth_or_budget_rejections;
        recordRejectedLeaf(parent, "depth_limit");
        return fail("TEMPORAL_DEPTH_EXHAUSTED", parent);
    }

    std::function<bool(double, double, int)> certifyChild;
    certifyChild = [&](double start_s, double end_s, int depth) {
        if (result.stats.child_tests >= config.max_child_tests) {
            ++result.stats.depth_or_budget_rejections;
            TemporalSeparationLeaf exhausted;
            exhausted.start_s = start_s;
            exhausted.end_s = end_s;
            exhausted.depth = depth;
            recordRejectedLeaf(exhausted, "budget_limit");
            result.rejected_leaf = std::move(exhausted);
            result.status = "TEMPORAL_BUDGET_EXHAUSTED";
            return false;
        }
        ++result.stats.child_tests;
        result.stats.max_depth_reached = std::max(result.stats.max_depth_reached, depth);
        const TemporalSeparationLeaf child = testSlice(
            first_child_hull(start_s, end_s), second_child_hull(start_s, end_s),
            start_s, end_s, depth, materialize_planes, &separator, &result.stats);
        if (child.separation.feasible) {
            ++result.stats.accepted_leaves;
            result.accepted_leaves.push_back(child);
            return true;
        }
        if (depth >= config.max_depth) {
            ++result.stats.depth_or_budget_rejections;
            recordRejectedLeaf(child, "depth_limit");
            result.rejected_leaf = child;
            result.status = "TEMPORAL_DEPTH_EXHAUSTED";
            return false;
        }
        const double midpoint = 0.5 * (start_s + end_s);
        const bool left_certified = certifyChild(start_s, midpoint, depth + 1);
        if (!left_certified && !config.collect_full_tree_diagnostics) {
            return false;
        }
        const bool right_certified = certifyChild(midpoint, end_s, depth + 1);
        return left_certified && right_certified;
    };

    bool all_children_certified = true;
    for (const auto& child : initialChildren(
             parent_start_s, parent_end_s, std::move(mandatory_breakpoints_s))) {
        const bool child_certified = certifyChild(child.first, child.second, 1);
        all_children_certified = all_children_certified && child_certified;
        if (!child_certified && !config.collect_full_tree_diagnostics) {
            return fail(result.status.empty() ? "TEMPORAL_REFINEMENT_FAILED" : result.status,
                        result.rejected_leaf);
        }
    }
    if (!all_children_certified) {
        return fail(result.status.empty() ? "TEMPORAL_REFINEMENT_FAILED" : result.status,
                    result.rejected_leaf);
    }
    result.certified = true;
    result.status = "TEMPORAL_REFINEMENT_SEPARATED";
    result.stats.wall_time_s = std::chrono::duration<double>(Clock::now() - started).count();
    return result;
}

FourPoints restrictCubicBezierVertices(const FourPoints& parent_vertices,
                                       double parent_start_s,
                                       double parent_end_s,
                                       double child_start_s,
                                       double child_end_s) {
    const Eigen::MatrixXd restricted = restrictBezierRows(
        parent_vertices, parent_start_s, parent_end_s, child_start_s, child_end_s);
    FourPoints result;
    result = restricted;
    return result;
}

Matrix4 cubicBSplineIntervalBezierMap(const std::vector<double>& knots,
                                      int num_control_points,
                                      int parent_interval) {
    const int num_segments = num_control_points - kCubicDegree;
    if (num_segments < 1 || parent_interval < 0 || parent_interval >= num_segments ||
        static_cast<int>(knots.size()) != num_control_points + kCubicDegree + 1) {
        throw std::invalid_argument("invalid cubic B-spline interval map request");
    }
    const double start_s = knots.at(static_cast<std::size_t>(kCubicDegree + parent_interval));
    const double end_s = knots.at(static_cast<std::size_t>(kCubicDegree + parent_interval + 1));
    if (!(end_s > start_s)) {
        throw std::invalid_argument("degenerate cubic B-spline parent interval");
    }
    Matrix4 map;
    for (int local_cp = 0; local_cp < 4; ++local_cp) {
        ControlPoints basis = ControlPoints::Zero(num_control_points, 3);
        basis(parent_interval + local_cp, 0) = 1.0;
        const State start = evaluateCubicState(basis, knots, start_s);
        const State end = evaluateCubicState(basis, knots, end_s);
        const double duration = end_s - start_s;
        map(0, local_cp) = start.position.x();
        map(1, local_cp) = start.position.x() + duration * start.velocity.x() / 3.0;
        map(2, local_cp) = end.position.x() - duration * end.velocity.x() / 3.0;
        map(3, local_cp) = end.position.x();
    }
    if (!map.allFinite()) {
        throw std::runtime_error("non-finite cubic B-spline Bezier map");
    }
    return map;
}

Matrix4 restrictCubicBezierControlMap(const Matrix4& parent_bezier_map,
                                      double parent_start_s,
                                      double parent_end_s,
                                      double child_start_s,
                                      double child_end_s) {
    const Eigen::MatrixXd restricted = restrictBezierRows(
        parent_bezier_map, parent_start_s, parent_end_s, child_start_s, child_end_s);
    Matrix4 result;
    result = restricted;
    return result;
}

Matrix4 cubicBSplineIntervalMinvoMap(const std::vector<double>& knots,
                                     int num_control_points,
                                     int parent_interval,
                                     int num_segments,
                                     double child_start_s,
                                     double child_end_s) {
    if (num_segments != num_control_points - kCubicDegree || num_segments < 4 ||
        parent_interval < 0 || parent_interval >= num_segments) {
        throw std::invalid_argument("invalid cubic B-spline MINVO map request");
    }
    const double parent_start_s =
        knots.at(static_cast<std::size_t>(kCubicDegree + parent_interval));
    const double parent_end_s =
        knots.at(static_cast<std::size_t>(kCubicDegree + parent_interval + 1));
    const Matrix4 bezier_map = cubicBSplineIntervalBezierMap(
        knots, num_control_points, parent_interval);
    Eigen::FullPivLU<Matrix4> lu(bezier_map);
    if (!lu.isInvertible()) {
        throw std::runtime_error("cubic B-spline Bezier extraction map is singular");
    }
    const Matrix4 child_bezier_map = restrictCubicBezierControlMap(
        bezier_map, parent_start_s, parent_end_s, child_start_s, child_end_s);
    const Matrix4 parent_minvo_map = positionConverters(num_segments).at(
        static_cast<std::size_t>(parent_interval)).transpose();
    Eigen::FullPivLU<Matrix4> transpose_lu(bezier_map.transpose());
    if (!transpose_lu.isInvertible()) {
        throw std::runtime_error("transposed cubic B-spline Bezier map is singular");
    }
    const Matrix4 bernstein_to_minvo =
        transpose_lu.solve(parent_minvo_map.transpose()).transpose();
    const Matrix4 result = bernstein_to_minvo * child_bezier_map;
    if (!result.allFinite()) {
        throw std::runtime_error("non-finite restricted MINVO map");
    }
    return result;
}

}  // namespace dynamic_planner
