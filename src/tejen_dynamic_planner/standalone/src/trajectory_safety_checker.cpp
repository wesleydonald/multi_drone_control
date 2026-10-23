#include "dynamic_planner/trajectory_safety_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dynamic_planner/ego_collision_model.hpp"
#include "dynamic_planner/separator.hpp"
#include "dynamic_planner/trajectory_geometry.hpp"

namespace dynamic_planner {
namespace {

Eigen::MatrixXd shiftedVertices(const Eigen::MatrixXd& vertices, const Vec3& shift) {
    Eigen::MatrixXd result = vertices;
    result.rowwise() += shift.transpose();
    return result;
}

bool aabbDisjoint(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    if (a.rows() < 1 || b.rows() < 1 || a.cols() != 3 || b.cols() != 3 ||
        !a.allFinite() || !b.allFinite()) {
        throw std::invalid_argument("invalid convex hull for AABB broad phase");
    }
    const Vec3 a_min = a.colwise().minCoeff().transpose();
    const Vec3 a_max = a.colwise().maxCoeff().transpose();
    const Vec3 b_min = b.colwise().minCoeff().transpose();
    const Vec3 b_max = b.colwise().maxCoeff().transpose();
    constexpr double tolerance = 1e-12;
    for (int axis = 0; axis < 3; ++axis) {
        if (a_max(axis) < b_min(axis) - tolerance ||
            b_max(axis) < a_min(axis) - tolerance) {
            return true;
        }
    }
    return false;
}

bool broadPhaseSeparated(TrajectorySafetyResult* result,
                         const Eigen::MatrixXd& a,
                         const Eigen::MatrixXd& b) {
    if (aabbDisjoint(a, b)) {
        ++result->aabb_separation_skips;
        return true;
    }
    return false;
}

TrajectorySafetyResult collisionResult(
    const TrajectorySafetyResult& base,
    const ConvexCurveInterval& interval,
    const std::string& obstacle_name) {
    TrajectorySafetyResult result = base;
    result.status = "COLLISION";
    result.obstacle_name = obstacle_name;
    result.unsafe_interval_start_s = interval.start_s;
    result.unsafe_interval_end_s = interval.end_s;
    return result;
}

void addTemporalCertificateStats(TrajectorySafetyResult* result,
                                 const AdaptiveTemporalCertificateResult& certificate) {
    if (result == nullptr) {
        throw std::invalid_argument("null trajectory-safety temporal result");
    }
    accumulateTemporalRefinementStats(
        &result->temporal_refinement, certificate.stats);
    result->separator_lp_calls += certificate.stats.separator_lp_calls;
    result->aabb_separation_skips += certificate.stats.aabb_separation_skips;
}

TrajectorySafetyResult temporalUncertifiedResult(
    const TrajectorySafetyResult& base,
    const AdaptiveTemporalCertificateResult& certificate,
    const ConvexCurveInterval& fallback_interval,
    const std::string& obstacle_name) {
    TrajectorySafetyResult result = base;
    result.safe = false;
    result.status = "UNCERTIFIED_TEMPORAL_" + certificate.status;
    result.obstacle_name = obstacle_name;
    result.unsafe_interval_start_s = certificate.rejected_leaf.has_value()
        ? certificate.rejected_leaf->start_s
        : fallback_interval.start_s;
    result.unsafe_interval_end_s = certificate.rejected_leaf.has_value()
        ? certificate.rejected_leaf->end_s
        : fallback_interval.end_s;
    return result;
}

TrajectorySafetyResult checkIntervals(
    const std::vector<ConvexCurveInterval>& ego_intervals,
    double from_s,
    double until_s,
    const WorldSnapshot& world,
    double separator_tolerance,
    const TemporalRefinementConfig& temporal_config) {
    TrajectorySafetyResult result;
    result.checked_from_s = from_s;
    result.checked_until_s = until_s;
    if (until_s <= from_s + 1e-12) {
        result.safe = true;
        result.status = "SAFE_EMPTY_INTERVAL";
        return result;
    }

    world.validate();
    if (temporal_config.max_depth < 0 || temporal_config.max_child_tests == 0U) {
        throw std::invalid_argument("invalid trajectory-safety temporal refinement config");
    }
    const auto ego_components = assemblyComponents(
        world.ego_half_extents, world.ego_suspended_geometry);
    Separator separator(separator_tolerance);

    for (const auto& interval : ego_intervals) {
        // Legacy pre-inflated static obstacles retain the old body-centre contract.
        for (const auto& obstacle : world.static_obstacles) {
            if (broadPhaseSeparated(&result, interval.vertices, obstacle.vertices)) {
                continue;
            }
            const SeparationResult separation = separator.solve(
                interval.vertices, obstacle.vertices);
            ++result.separator_lp_calls;
            if (!separation.feasible) {
                return collisionResult(result, interval, obstacle.name);
            }
        }

        // B.4 independent physical-space verification. Unlike the planner-side
        // C-space transformation, the checker sweeps each actual ego component
        // with the exact cubic centre hull and tests it directly against the
        // physical obstacle.
        for (const auto& obstacle : world.physical_static_obstacles) {
            for (const auto& ego_component : ego_components) {
                const Eigen::MatrixXd ego_component_hull = inflateVerticesByAabb(
                    sweptComponentHull(interval.vertices, ego_component),
                    world.ego_tracking_error_half_extents);
                if (broadPhaseSeparated(
                        &result, ego_component_hull, obstacle.vertices)) {
                    continue;
                }
                const SeparationResult separation = separator.solve(
                    ego_component_hull, obstacle.vertices);
                ++result.separator_lp_calls;
                if (!separation.feasible) {
                    return collisionResult(
                        result, interval,
                        obstacle.name + "::ego_" + ego_component.name);
                }
            }
        }

        for (const auto& obstacle : world.cooperative_obstacles) {
            const auto obstacle_breakpoints = trajectoryBreakpoints(
                obstacle.trajectory, interval.start_s, interval.end_s);
            if (!world.ego_suspended_geometry.enabled &&
                !obstacle.suspended_geometry.enabled &&
                !obstacle.attached_tether_geometry.enabled) {
                const Vec3 inflation = world.ego_half_extents +
                                       world.ego_tracking_error_half_extents +
                                       obstacle.physical_half_extents +
                                       obstacle.tracking_error_half_extents;
                const Eigen::MatrixXd obstacle_center_hull = convexCurveHullVertices(
                    obstacle.trajectory, interval.start_s, interval.end_s);
                const Eigen::MatrixXd obstacle_hull = inflateVerticesByAabb(
                    obstacle_center_hull, inflation);
                if (!temporal_config.enabled) {
                    if (broadPhaseSeparated(&result, interval.vertices, obstacle_hull)) continue;
                    const auto separation = separator.solve(interval.vertices, obstacle_hull);
                    ++result.separator_lp_calls;
                    if (!separation.feasible) return collisionResult(result, interval, obstacle.name);
                    continue;
                }
                const auto certificate = certifyAdaptiveTemporalSeparation(
                    interval.start_s, interval.end_s, interval.vertices, obstacle_hull,
                    [interval](double start_s, double end_s) {
                        return Eigen::MatrixXd(restrictCubicBezierVertices(
                            interval.vertices, interval.start_s, interval.end_s,
                            start_s, end_s));
                    },
                    [trajectory = obstacle.trajectory, inflation](double start_s, double end_s) {
                        return inflateVerticesByAabb(
                            convexCurveHullVertices(trajectory, start_s, end_s), inflation);
                    },
                    obstacle_breakpoints, temporal_config, separator_tolerance, false);
                addTemporalCertificateStats(&result, certificate);
                if (!certificate.certified) {
                    return temporalUncertifiedResult(
                        result, certificate, interval, obstacle.name);
                }
                continue;
            }

            const auto other_components = cooperativeAssemblyComponents(
                obstacle.physical_half_extents, obstacle.suspended_geometry,
                obstacle.attached_tether_geometry);
            const Eigen::MatrixXd other_center_hull = convexCurveHullVertices(
                obstacle.trajectory, interval.start_s, interval.end_s);

            for (const auto& ego_component : ego_components) {
                Vec3 ego_center;
                Vec3 ego_half;
                componentAabb(ego_component, &ego_center, &ego_half);
                const Eigen::MatrixXd ego_physical_hull = inflateVerticesByAabb(
                    shiftedVertices(interval.vertices, ego_center),
                    ego_half + world.ego_tracking_error_half_extents);

                for (const auto& other_component : other_components) {
                    Vec3 other_center;
                    Vec3 other_half;
                    componentAabb(other_component, &other_center, &other_half);
                    const Eigen::MatrixXd other_physical_hull = inflateVerticesByAabb(
                        shiftedVertices(other_center_hull, other_center),
                        other_half + obstacle.tracking_error_half_extents);
                    if (!temporal_config.enabled) {
                        if (broadPhaseSeparated(&result, ego_physical_hull, other_physical_hull)) continue;
                        const auto separation = separator.solve(ego_physical_hull, other_physical_hull);
                        ++result.separator_lp_calls;
                        if (!separation.feasible) return collisionResult(result, interval,
                            obstacle.name + "::" + other_component.name + "_vs_ego_" + ego_component.name);
                        continue;
                    }
                    const auto certificate = certifyAdaptiveTemporalSeparation(
                        interval.start_s, interval.end_s,
                        ego_physical_hull, other_physical_hull,
                        [interval, ego_component,
                         tracking = world.ego_tracking_error_half_extents](
                            double start_s, double end_s) {
                            const FourPoints ego_child = restrictCubicBezierVertices(
                                interval.vertices, interval.start_s, interval.end_s,
                                start_s, end_s);
                            return inflateVerticesByAabb(
                                sweptComponentHull(ego_child, ego_component), tracking);
                        },
                        [trajectory = obstacle.trajectory, other_center, other_half,
                         tracking = obstacle.tracking_error_half_extents](
                            double start_s, double end_s) {
                            return inflateVerticesByAabb(
                                shiftedVertices(
                                    convexCurveHullVertices(
                                        trajectory, start_s, end_s), other_center),
                                other_half + tracking);
                        },
                        obstacle_breakpoints, temporal_config, separator_tolerance, false);
                    addTemporalCertificateStats(&result, certificate);
                    if (!certificate.certified) {
                        return temporalUncertifiedResult(
                            result, certificate, interval,
                            obstacle.name + "::" + other_component.name +
                                "_vs_ego_" + ego_component.name);
                    }
                }
            }
        }
    }

    result.safe = true;
    result.status = "SAFE";
    return result;
}

}  // namespace

TrajectorySafetyResult TrajectorySafetyChecker::checkPiece(
    const TrajectoryPiece& ego,
    double from_s,
    double until_s,
    const WorldSnapshot& world) const {
    if (!std::isfinite(separator_validation_tolerance_) ||
        !(separator_validation_tolerance_ >= 0.0)) {
        throw std::invalid_argument("invalid separator validation tolerance");
    }
    const auto intervals = convexCurveIntervals(ego, from_s, until_s);
    return checkIntervals(
        intervals, from_s, until_s, world, separator_validation_tolerance_,
        temporal_refinement_);
}

TrajectorySafetyResult TrajectorySafetyChecker::checkCommitted(
    const CommittedTrajectory& ego,
    double from_s,
    double until_s,
    const WorldSnapshot& world) const {
    if (!std::isfinite(separator_validation_tolerance_) ||
        !(separator_validation_tolerance_ >= 0.0)) {
        throw std::invalid_argument("invalid separator validation tolerance");
    }
    const auto intervals = convexCurveIntervals(ego, from_s, until_s);
    return checkIntervals(
        intervals, from_s, until_s, world, separator_validation_tolerance_,
        temporal_refinement_);
}

}  // namespace dynamic_planner
