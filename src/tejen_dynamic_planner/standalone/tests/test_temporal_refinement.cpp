#include "dynamic_planner/temporal_collision_certificate.hpp"
#include "dynamic_planner/trajectory_geometry.hpp"
#include "dynamic_planner/trajectory_refinement.hpp"
#include "dynamic_planner/trajectory_safety_checker.hpp"
#include "dynamic_planner/validation_scenarios.hpp"
#include "test_common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

using dynamic_planner::AdaptiveTemporalCertificateResult;
using dynamic_planner::CommittedTrajectory;
using dynamic_planner::ControlPoints;
using dynamic_planner::FourPoints;
using dynamic_planner::IntervalSeparator;
using dynamic_planner::OctopusResult;
using dynamic_planner::OctopusSearch;
using dynamic_planner::RefinementConfig;
using dynamic_planner::TemporalRefinementConfig;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::TrajectorySafetyChecker;
using dynamic_planner::Vec3;
using dynamic_planner::WorldSnapshot;

FourPoints lineBezier(double x0, double x1) {
    FourPoints result;
    result << x0, 0.0, 0.0,
              x0 + (x1 - x0) / 3.0, 0.0, 0.0,
              x0 + 2.0 * (x1 - x0) / 3.0, 0.0, 0.0,
              x1, 0.0, 0.0;
    return result;
}

AdaptiveTemporalCertificateResult certifyLines(
    const FourPoints& ego,
    const FourPoints& obstacle,
    const TemporalRefinementConfig& config,
    std::vector<double> breakpoints = {}) {
    const auto ego_hull = [ego](double start_s, double end_s) {
        return Eigen::MatrixXd(dynamic_planner::restrictCubicBezierVertices(
            ego, 0.0, 1.0, start_s, end_s));
    };
    const auto obstacle_hull = [obstacle](double start_s, double end_s) {
        return Eigen::MatrixXd(dynamic_planner::restrictCubicBezierVertices(
            obstacle, 0.0, 1.0, start_s, end_s));
    };
    return dynamic_planner::certifyAdaptiveTemporalSeparation(
        0.0, 1.0, ego, obstacle, ego_hull, obstacle_hull,
        std::move(breakpoints), config, 1e-7, true);
}

std::size_t restriction_checks = 0;
Vec3 restriction_component_min = Vec3::Constant(
    std::numeric_limits<double>::infinity());
Vec3 restriction_component_max = Vec3::Constant(
    -std::numeric_limits<double>::infinity());

void requireFiniteAndPlausible(const Eigen::MatrixXd& values,
                               const Eigen::MatrixXd& parent,
                               const std::string& label) {
    requireTrue(parent.allFinite(), label + " parent control points are nonfinite");
    requireTrue(values.allFinite(), label + " control points are nonfinite");
    const Eigen::RowVectorXd lower = parent.colwise().minCoeff();
    const Eigen::RowVectorXd upper = parent.colwise().maxCoeff();
    for (Eigen::Index row = 0; row < values.rows(); ++row) {
        for (Eigen::Index column = 0; column < values.cols(); ++column) {
            requireTrue(values(row, column) >= lower(column) - 1e-12 &&
                            values(row, column) <= upper(column) + 1e-12,
                        label + " control point escaped parent convex-hull bounds");
            restriction_component_min(column) =
                std::min(restriction_component_min(column), values(row, column));
            restriction_component_max(column) =
                std::max(restriction_component_max(column), values(row, column));
        }
    }
    ++restriction_checks;
}

Eigen::MatrixXd deCasteljauLeft(const Eigen::MatrixXd& control, double u) {
    Eigen::MatrixXd level = control;
    Eigen::MatrixXd left(control.rows(), control.cols());
    left.row(0) = level.row(0);
    for (Eigen::Index order = 1; order < control.rows(); ++order) {
        const Eigen::MatrixXd next =
            (1.0 - u) * level.topRows(level.rows() - 1) +
            u * level.bottomRows(level.rows() - 1);
        requireFiniteAndPlausible(next, control, "de Casteljau left intermediate");
        level = next;
        left.row(order) = level.row(0);
    }
    requireFiniteAndPlausible(left, control, "de Casteljau left result");
    return left;
}

Eigen::MatrixXd deCasteljauRight(const Eigen::MatrixXd& control, double u) {
    Eigen::MatrixXd level = control;
    Eigen::MatrixXd right(control.rows(), control.cols());
    right.row(control.rows() - 1) = level.row(level.rows() - 1);
    for (Eigen::Index order = 1; order < control.rows(); ++order) {
        const Eigen::MatrixXd next =
            (1.0 - u) * level.topRows(level.rows() - 1) +
            u * level.bottomRows(level.rows() - 1);
        requireFiniteAndPlausible(next, control, "de Casteljau right intermediate");
        level = next;
        right.row(control.rows() - 1 - order) = level.row(level.rows() - 1);
    }
    requireFiniteAndPlausible(right, control, "de Casteljau right result");
    return right;
}

Eigen::MatrixXd restrictBezier(const Eigen::MatrixXd& parent,
                               double parent_start_s, double parent_end_s,
                               double child_start_s, double child_end_s) {
    const double duration = parent_end_s - parent_start_s;
    const double u0 = (child_start_s - parent_start_s) / duration;
    const double u1 = (child_end_s - parent_start_s) / duration;
    Eigen::MatrixXd restricted = parent;
    requireFiniteAndPlausible(restricted, parent, "Bezier restriction input");
    if (u0 > 0.0) restricted = deCasteljauRight(restricted, u0);
    if (u1 < 1.0) restricted = deCasteljauLeft(restricted, (u1 - u0) / (1.0 - u0));
    requireFiniteAndPlausible(restricted, parent, "Bezier restriction result");
    return restricted;
}

Eigen::MatrixXd quinticRestToRestBezier(const Vec3& start, const Vec3& end) {
    Eigen::MatrixXd vertices(6, 3);
    for (int row = 0; row < 3; ++row) vertices.row(row) = start.transpose();
    for (int row = 3; row < 6; ++row) vertices.row(row) = end.transpose();
    return vertices;
}

Vec3 quinticFixturePosition(const Vec3& start, const Vec3& end, double fraction) {
    const double s = 10.0 * std::pow(fraction, 3) - 15.0 * std::pow(fraction, 4) +
                     6.0 * std::pow(fraction, 5);
    return start + s * (end - start);
}

TrajectoryPiece linearPiece(const Vec3& a, const Vec3& b,
                            double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = dynamic_planner::openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = a.transpose();
    piece.control_points.row(1) = (a + (b - a) / 3.0).transpose();
    piece.control_points.row(2) = (a + 2.0 * (b - a) / 3.0).transpose();
    piece.control_points.row(3) = b.transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    return piece;
}

CommittedTrajectory linearTrajectory(const Vec3& a, const Vec3& b,
                                     double t0, double t1) {
    CommittedTrajectory result;
    requireTrue(result.replaceSuffix(t0, linearPiece(a, b, t0, t1)).accepted,
                "failed to construct linear committed trajectory");
    return result;
}

RefinementConfig refinementConfig(const dynamic_planner::ValidationScenario& scene) {
    RefinementConfig config;
    config.v_max = scene.octopus_config.v_max;
    config.a_max = scene.octopus_config.a_max;
    config.j_max = Vec3::Constant(4.0);
    config.jerk_weight = 1.0;
    config.goal_weight = 10.0;
    config.goal_acceptance_m = 0.05;
    config.separator_margin_m = 0.001;
    config.xyz_min = scene.octopus_config.xyz_min;
    config.xyz_max = scene.octopus_config.xyz_max;
    config.max_working_set_recalculations = 500;
    config.validation_tolerance = 2e-6;
    return config;
}

void testFalseSweptHullRefinesToCertificate() {
    TemporalRefinementConfig config;
    config.max_depth = 3;
    config.max_child_tests = 16;
    const auto result = certifyLines(lineBezier(0.0, 1.0), lineBezier(1.0, 2.0), config);
    requireTrue(result.certified, "time-separated parallel curves were not certified");
    requireTrue(result.stats.parent_tests == 1U, "missing parent certificate test");
    requireTrue(result.stats.parent_fast_path_accepts == 0U,
                "coarse overlapping sweep incorrectly took parent fast path");
    requireTrue(result.stats.child_tests == 2U,
                "one bisection should test exactly two children");
    requireTrue(result.stats.max_depth_reached == 1,
                "false swept-hull case should certify at first child depth");
    requireTrue(result.stats.accepted_leaves == 2U,
                "both refined child leaves should be certified");
}

void testTrueSynchronizedCollisionStaysRejected() {
    TemporalRefinementConfig config;
    config.max_depth = 3;
    config.max_child_tests = 16;
    const auto result = certifyLines(lineBezier(0.0, 1.0), lineBezier(1.0, 0.0), config);
    requireTrue(!result.certified, "equal-time collision was incorrectly certified");
    requireTrue(result.stats.depth_or_budget_rejections > 0U,
                "synchronized collision should fail closed at bounded depth");
}

void testParentFastPath() {
    TemporalRefinementConfig config;
    const auto result = certifyLines(lineBezier(0.0, 1.0), lineBezier(3.0, 4.0), config);
    requireTrue(result.certified, "comfortably separated parent was rejected");
    requireTrue(result.stats.parent_fast_path_accepts == 1U,
                "parent fast path was not recorded");
    requireTrue(result.stats.child_tests == 0U,
                "parent-separated case must not recurse");
    requireTrue(result.stats.max_depth_reached == 0,
                "parent-separated case should remain at depth zero");
}

void testObstacleKnotSplitAndSelectiveRecursion() {
    TemporalRefinementConfig config;
    config.max_depth = 3;
    config.max_child_tests = 16;
    const auto knot_split = certifyLines(
        lineBezier(0.0, 1.0), lineBezier(1.0, 2.0), config, {0.5});
    requireTrue(knot_split.certified, "knot-aligned child intervals were not certified");
    requireTrue(knot_split.stats.child_tests == 2U,
                "obstacle breakpoint must create two common-time children");
    requireTrue(knot_split.accepted_leaves.size() == 2U,
                "obstacle breakpoint left a time interval unchecked");
    requireNear(knot_split.accepted_leaves.front().end_s, 0.5, 1e-12,
                "first knot-aligned leaf has wrong end time");
    requireNear(knot_split.accepted_leaves.back().start_s, 0.5, 1e-12,
                "second knot-aligned leaf has wrong start time");

    const auto deeper = certifyLines(
        lineBezier(0.0, 1.0), lineBezier(0.4, 1.4), config);
    requireTrue(deeper.certified, "safe case requiring recursive refinement was rejected");
    requireTrue(deeper.stats.max_depth_reached == 2,
                "safe curved-time proxy should require exactly two refinement depths");
    requireTrue(deeper.stats.child_tests == 6U,
                "only ambiguous children should recurse in depth-two regression");
}

void testDepthLimitFailsClosed() {
    TemporalRefinementConfig config;
    config.max_depth = 1;
    config.max_child_tests = 16;
    const auto result = certifyLines(
        lineBezier(0.0, 1.0), lineBezier(0.4, 1.4), config);
    requireTrue(!result.certified, "depth-exhausted interval was accepted");
    requireTrue(result.stats.depth_or_budget_rejections > 0U,
                "depth exhaustion was not instrumented");
}

void testChildBudgetFailsClosed() {
    TemporalRefinementConfig config;
    config.max_depth = 3;
    config.max_child_tests = 1;
    const auto result = certifyLines(
        lineBezier(0.0, 1.0), lineBezier(1.0, 2.0), config);
    requireTrue(!result.certified, "child-budget exhaustion was accepted");
    requireTrue(result.status == "TEMPORAL_BUDGET_EXHAUSTED",
                "child-budget exhaustion has the wrong status");
    requireTrue(result.stats.depth_or_budget_rejections == 1U,
                "child-budget exhaustion was not counted exactly once");
}

void testFullTreeDiagnosticsEnumeratesAllTerminalBranches() {
    TemporalRefinementConfig config;
    config.max_depth = 2;
    config.max_child_tests = 16;
    config.collect_full_tree_diagnostics = true;
    const auto result = certifyLines(
        lineBezier(0.0, 1.0), lineBezier(1.0, 0.0), config);
    requireTrue(!result.certified, "full-tree diagnostic must remain fail-closed");
    requireTrue(result.stats.child_tests == 6U,
                "full-tree diagnostic must test all depth-two children");
    requireTrue(result.stats.accepted_leaves == 2U,
                "full-tree diagnostic must retain the two separated outer leaves");
    requireTrue(result.stats.unresolved_leaves.size() == 2U,
                "full-tree diagnostic must report both unresolved inner leaves");
    requireTrue(result.stats.max_depth_reached == 2,
                "full-tree diagnostic reached the wrong depth");
}

void testKnownDynamicCrossingFixtureDetourCertifies() {
    restriction_checks = 0;
    restriction_component_min = Vec3::Constant(
        std::numeric_limits<double>::infinity());
    restriction_component_max = Vec3::Constant(
        -std::numeric_limits<double>::infinity());
    // This is the unchanged PB.1 fixture detour in
    // tools/planner_benchmark/adapters.py: _fixture_points() plus
    // piecewise_quintic(..., speed=0.50). Its rest-to-rest quintics have exact
    // degree-five Bezier controls [p0,p0,p0,p1,p1,p1]. The production Octopus
    // only accepts four-segment cubic B-splines, so this test intentionally
    // exercises the same final adaptive cooperative certificate kernel without
    // approximating the fixture into a different trajectory.
    const Vec3 start(0.0, 0.0, 1.5);
    const Vec3 waypoint(1.5, 1.2, 1.5);
    const Vec3 goal(3.0, 0.0, 1.5);
    const double segment_duration = (waypoint - start).norm() / 0.50;
    const double total_duration = 2.0 * segment_duration;
    const Vec3 obstacle_start(-3.0, 0.0, 1.5);
    const Vec3 obstacle_velocity(1.5, 0.0, 0.0);
    const Vec3 obstacle_half(0.12, 0.12, 0.12);
    const double swing = 0.50 * std::sin(10.0 * M_PI / 180.0);
    const Vec3 ego_half(0.105 + swing + 0.050 + 0.010,
                        0.105 + swing + 0.050 + 0.010,
                        0.060 + 0.50 + 0.025 + 0.003 + 0.028 + 0.010);
    const Vec3 combined_half = ego_half + obstacle_half;

    const auto obstacle = linearTrajectory(
        obstacle_start, obstacle_start + 180.0 * obstacle_velocity, 0.0, 180.0);
    // Diagnostic-only sweep: production defaults remain max_depth=3 and their
    // normal child budget. A child budget of 64 is nonbinding for a depth-5
    // binary tree (at most 62 recursive child tests per parent interval).
    for (int diagnostic_depth = 3; diagnostic_depth <= 5; ++diagnostic_depth) {
        TemporalRefinementConfig config;
        config.max_depth = diagnostic_depth;
        config.max_child_tests = 64;
        dynamic_planner::TemporalRefinementStats aggregate;
        std::vector<std::string> certificate_statuses;
        bool all_segments_certified = true;
        const auto certificate_started = std::chrono::steady_clock::now();

        const auto certify_segment = [&](const Vec3& a, const Vec3& b,
                                         double begin_s, double end_s) {
        const Eigen::MatrixXd ego_parent = quinticRestToRestBezier(a, b);
        const Eigen::MatrixXd obstacle_parent = dynamic_planner::inflateVerticesByAabb(
            dynamic_planner::convexCurveHullVertices(obstacle, begin_s, end_s),
            combined_half);
        const auto certificate = dynamic_planner::certifyAdaptiveTemporalSeparation(
            begin_s, end_s, ego_parent, obstacle_parent,
            [ego_parent, begin_s, end_s](double child_begin_s, double child_end_s) {
                return restrictBezier(
                    ego_parent, begin_s, end_s, child_begin_s, child_end_s);
            },
            [obstacle, combined_half](double child_begin_s, double child_end_s) {
                return dynamic_planner::inflateVerticesByAabb(
                    dynamic_planner::convexCurveHullVertices(
                        obstacle, child_begin_s, child_end_s), combined_half);
            },
            dynamic_planner::trajectoryBreakpoints(obstacle, begin_s, end_s),
            // The temporal accept/reject decision is identical to finalCheck;
            // no leaf plane is needed here because this fixed quintic fixture
            // is not sent on to the cubic-QP refinement stage.
            config, 1e-7, false,
            [a, b, begin_s, end_s, obstacle_start, obstacle_velocity, combined_half](
                    const dynamic_planner::TemporalSeparationLeaf& leaf) {
                dynamic_planner::TemporalLeafDiagnostic diagnostic;
                diagnostic.stage = "fixture_final_cooperative_certificate";
                diagnostic.candidate_identity = "python_fixture_quintic_detour";
                diagnostic.primitive_identity = "crossing_drone";
                constexpr double sample_period_s = 0.01;
                const int samples = std::max(
                    1, static_cast<int>(std::ceil(
                           (leaf.end_s - leaf.start_s) / sample_period_s)));
                for (int index = 0; index <= samples; ++index) {
                    const double time_s = leaf.start_s +
                        (leaf.end_s - leaf.start_s) * static_cast<double>(index) /
                            static_cast<double>(samples);
                    const Vec3 ego = quinticFixturePosition(
                        a, b, (time_s - begin_s) / (end_s - begin_s));
                    const Vec3 peer = obstacle_start + obstacle_velocity * time_s;
                    const Vec3 gap = (ego - peer).cwiseAbs() - combined_half;
                    const Vec3 positive_gap = gap.cwiseMax(Vec3::Zero());
                    const double clearance = positive_gap.squaredNorm() > 0.0
                        ? positive_gap.norm()
                        : gap.maxCoeff();
                    if (!std::isfinite(
                            diagnostic.equal_time_clearance.minimum_sampled_clearance_m) ||
                        clearance < diagnostic.equal_time_clearance.minimum_sampled_clearance_m) {
                        diagnostic.equal_time_clearance.minimum_sampled_clearance_m = clearance;
                        diagnostic.equal_time_clearance.minimum_sample_time_s = time_s;
                    }
                }
                diagnostic.equal_time_clearance.sampled_points =
                    static_cast<std::size_t>(samples + 1);
                return diagnostic;
            });
        dynamic_planner::accumulateTemporalRefinementStats(&aggregate, certificate.stats);
        certificate_statuses.push_back(certificate.status);
        all_segments_certified = all_segments_certified && certificate.certified;
        if (diagnostic_depth == 3 && certificate.rejected_leaf.has_value()) {
            const auto& leaf = *certificate.rejected_leaf;
            std::cout << std::setprecision(17)
                      << "fixture_detour_depth3_separator_leaf=[" << leaf.start_s << ','
                      << leaf.end_s << "] first_vertices=" << leaf.first_vertices
                      << " second_vertices=" << leaf.second_vertices << '\n';
        }
        };
        certify_segment(start, waypoint, 0.0, segment_duration);
        certify_segment(waypoint, goal, segment_duration, total_duration);
        const double certificate_wall_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - certificate_started).count();
        requireTrue(all_segments_certified,
                    "corrected known-safe fixture must certify at every diagnostic depth");

        std::cout << "fixture_detour_depth=" << diagnostic_depth
                  << ": certificate="
                  << (all_segments_certified ? "SAFE" : "REJECTED")
                  << " max_depth=" << aggregate.max_depth_reached
                  << " parent_tests=" << aggregate.parent_tests
                  << " child_tests=" << aggregate.child_tests
                  << " accepted_leaves=" << aggregate.accepted_leaves
                  << " unresolved_leaves=" << aggregate.depth_or_budget_rejections
                  << " certificate_wall_s=" << certificate_wall_s << '\n';
        for (const auto& status : certificate_statuses) {
            std::cout << "fixture_detour_depth=" << diagnostic_depth
                      << "_certificate_status=" << status << '\n';
        }
        for (const auto& leaf : aggregate.unresolved_leaves) {
            std::cout << "fixture_detour_depth=" << diagnostic_depth
                      << "_unresolved_leaf: [" << leaf.start_s << ','
                      << leaf.end_s << "] reason=" << leaf.limit_reason
                      << " sampled_clearance_m="
                      << leaf.equal_time_clearance.minimum_sampled_clearance_m
                      << " sampled_time_s="
                      << leaf.equal_time_clearance.minimum_sample_time_s
                      << " samples=" << leaf.equal_time_clearance.sampled_points << '\n';
        }
    }

    const double peak_speed_norm = 0.9375;
    const double peak_acceleration_norm =
        (10.0 * std::sqrt(3.0) / 3.0) * (waypoint - start).norm() /
        (segment_duration * segment_duration);
    const double peak_jerk_norm = 60.0 * (waypoint - start).norm() /
        (segment_duration * segment_duration * segment_duration);
    const Vec3 peak_axis_speed(1.875 * 1.5 / segment_duration,
                               1.875 * 1.2 / segment_duration, 0.0);
    const Vec3 peak_axis_acceleration(
        (10.0 * std::sqrt(3.0) / 3.0) * 1.5 /
            (segment_duration * segment_duration),
        (10.0 * std::sqrt(3.0) / 3.0) * 1.2 /
            (segment_duration * segment_duration),
        0.0);
    const Vec3 peak_axis_jerk(60.0 * 1.5 /
                                   (segment_duration * segment_duration * segment_duration),
                               60.0 * 1.2 /
                                   (segment_duration * segment_duration * segment_duration),
                               0.0);
    requireTrue((peak_axis_speed.array() <= 1.0 + 1e-12).all(),
                "fixture detour exceeds planner per-axis velocity bound");
    requireTrue((peak_axis_acceleration.array() <= Vec3(1.0, 1.0, 1.5).array() + 1e-12).all(),
                "fixture detour exceeds planner per-axis acceleration bound");
    requireTrue((peak_axis_jerk.array() <= 4.0 + 1e-12).all(),
                "fixture detour exceeds planner per-axis jerk bound");

    double minimum_clearance = std::numeric_limits<double>::infinity();
    double minimum_clearance_time = 0.0;
    constexpr double sample_period_s = 0.01;
    const int samples = static_cast<int>(std::ceil(total_duration / sample_period_s));
    for (int index = 0; index <= samples; ++index) {
        const double time_s = total_duration * static_cast<double>(index) /
                              static_cast<double>(samples);
        const bool first = time_s <= segment_duration;
        const double local_time = first ? time_s : time_s - segment_duration;
        const Vec3 ego = quinticFixturePosition(
            first ? start : waypoint, first ? waypoint : goal,
            local_time / segment_duration);
        const Vec3 peer = obstacle_start + obstacle_velocity * time_s;
        const Vec3 gap = (ego - peer).cwiseAbs() - combined_half;
        const Vec3 positive_gap = gap.cwiseMax(Vec3::Zero());
        const double clearance = positive_gap.squaredNorm() > 0.0
            ? positive_gap.norm()
            : gap.maxCoeff();
        if (clearance < minimum_clearance) {
            minimum_clearance = clearance;
            minimum_clearance_time = time_s;
        }
    }
    requireTrue(minimum_clearance > 0.0,
                "known fixture detour has a sampled collision under planner geometry");

    std::cout << "fixture_detour_restriction_checks=" << restriction_checks << '\n';
    std::cout << "fixture_detour_restriction_component_min="
              << restriction_component_min.transpose() << '\n';
    std::cout << "fixture_detour_restriction_component_max="
              << restriction_component_max.transpose() << '\n';
    std::cout << "fixture_detour_kinematics: min_sampled_clearance_m=" << minimum_clearance
              << " min_clearance_time_s=" << minimum_clearance_time
              << " peak_speed_norm_mps=" << peak_speed_norm
              << " peak_acceleration_norm_mps2=" << peak_acceleration_norm
              << " peak_jerk_norm_mps3=" << peak_jerk_norm << '\n';
}

void testMinvoRestrictionAndQpLeafConstraint() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    const auto parent_map = dynamic_planner::cubicBSplineIntervalMinvoMap(
        knots, 7, 0, 4, 0.0, 1.0);
    const dynamic_planner::Matrix4 expected =
        dynamic_planner::positionConverters(4).front().transpose();
    const double parent_map_error = (parent_map - expected).cwiseAbs().maxCoeff();
    requireTrue(parent_map_error < 1e-10,
                "full parent MINVO map changed under temporal helper: max error=" +
                    std::to_string(parent_map_error));

    const auto child_map = dynamic_planner::cubicBSplineIntervalMinvoMap(
        knots, 7, 0, 4, 0.0, 0.5);
    requireTrue(child_map.allFinite(), "restricted MINVO control map is non-finite");

    const auto scene = dynamic_planner::makeValidationScenario("default", 7);
    OctopusResult raw = OctopusSearch(
        scene.knots, scene.initial_control_points, scene.goal,
        scene.obstacles, scene.octopus_config).search();
    requireTrue(raw.success && raw.control_points.has_value(),
                "validation raw trajectory unavailable for QP leaf regression");

    const auto leaf_map = dynamic_planner::cubicBSplineIntervalMinvoMap(
        scene.knots, static_cast<int>(raw.control_points->rows()), 0, 4,
        scene.knots.at(3), 0.5 * (scene.knots.at(3) + scene.knots.at(4)));
    const FourPoints raw_leaf = leaf_map * raw.control_points->middleRows(0, 4);
    Eigen::MatrixXd remote_obstacle(1, 3);
    remote_obstacle.row(0) =
        raw_leaf.colwise().mean() + Eigen::RowVector3d(0.0, 4.0, 0.0);
    dynamic_planner::Separator separator;
    const auto separation = separator.solve(raw_leaf, remote_obstacle);
    requireTrue(separation.feasible && separation.plane.has_value(),
                "failed to build leaf separator for QP regression");

    IntervalSeparator leaf;
    leaf.obstacle_name = "temporal_leaf";
    leaf.interval_index = 0;
    leaf.result = separation;
    leaf.time_start_s = scene.knots.at(3);
    leaf.time_end_s = 0.5 * (scene.knots.at(3) + scene.knots.at(4));
    leaf.ego_vertex_map = leaf_map;
    leaf.obstacle_vertices = remote_obstacle;
    raw.separators = {leaf};

    const auto refined = dynamic_planner::refineOctopusTrajectory(
        raw, scene.knots, scene.goal, {}, refinementConfig(scene));
    requireTrue(refined.success && refined.control_points.has_value(),
                "QP failed with an affine temporal leaf separator: " + refined.message);
    requireTrue(refined.separator_recheck_passed,
                "QP postcheck did not validate the temporal leaf separator");
    const FourPoints refined_leaf = leaf_map * refined.control_points->middleRows(0, 4);
    requireTrue(separator.solve(refined_leaf, remote_obstacle).feasible,
                "refined trajectory violates independently recomputed leaf separation");
}

void testOctopusFeatureGate() {
    auto scene = dynamic_planner::makeValidationScenario("default", 7);
    WorldSnapshot world;
    world.ego_half_extents = Vec3::Zero();
    dynamic_planner::CooperativeObstacleTrajectory peer;
    peer.name = "remote_feature_control";
    peer.physical_half_extents = Vec3::Zero();
    peer.tracking_error_half_extents = Vec3::Zero();
    peer.trajectory = linearTrajectory(Vec3(100,0,0), Vec3(100,1,0),
                                       scene.knots.front(), scene.knots.back());
    world.cooperative_obstacles.push_back(peer);
    auto timed = world.timeIndexedObstacles(4,scene.knots.front(),scene.knots.back());
    scene.obstacles.insert(scene.obstacles.end(),timed.begin(),timed.end());
    scene.octopus_config.temporal_refinement.enabled = false;
    const auto native = OctopusSearch(scene.knots,scene.initial_control_points,scene.goal,
        scene.obstacles,scene.octopus_config).search();
    requireTrue(native.success && native.temporal_refinement.parent_tests == 0,
                "native control entered temporal pruning or final certification");
    scene.octopus_config.temporal_refinement.enabled = true;
    const auto refined = OctopusSearch(scene.knots,scene.initial_control_points,scene.goal,
        scene.obstacles,scene.octopus_config).search();
    requireTrue(refined.success && refined.temporal_refinement.parent_tests > 0,
                "temporal opt-in did not reach cooperative certificates");
}

void testIndependentSafetyCheckerUsesTemporalRefinement() {
    const TrajectoryPiece ego = linearPiece(
        Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), 0.0, 1.0);
    WorldSnapshot safe_world;
    safe_world.captured_at_s = 0.0;
    safe_world.ego_half_extents = Vec3::Zero();
    dynamic_planner::CooperativeObstacleTrajectory cooperative;
    cooperative.name = "time_shifted";
    cooperative.trajectory = linearTrajectory(
        Vec3(1.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0), 0.0, 1.0);
    cooperative.physical_half_extents = Vec3::Zero();
    cooperative.tracking_error_half_extents = Vec3::Zero();
    safe_world.cooperative_obstacles.push_back(cooperative);

    TemporalRefinementConfig enabled;
    enabled.enabled = true;
    TrajectorySafetyChecker coarse;
    requireTrue(!coarse.checkPiece(ego, 0.0, 1.0, safe_world).safe,
                "native default unexpectedly enabled adaptive certification");
    TrajectorySafetyChecker checker(1e-7, enabled);
    const auto safe = checker.checkPiece(ego, 0.0, 1.0, safe_world);
    requireTrue(safe.safe, "independent checker retained coarse false collision");
    requireTrue(safe.temporal_refinement.child_tests > 0U,
                "independent checker did not instrument child certification");

    safe_world.cooperative_obstacles.front().trajectory = linearTrajectory(
        Vec3(1.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), 0.0, 1.0);
    const auto collision = checker.checkPiece(ego, 0.0, 1.0, safe_world);
    requireTrue(!collision.safe, "independent checker accepted synchronized collision");
}

}  // namespace

int main() {
    try {
        testFalseSweptHullRefinesToCertificate();
        testTrueSynchronizedCollisionStaysRejected();
        testParentFastPath();
        testObstacleKnotSplitAndSelectiveRecursion();
        testDepthLimitFailsClosed();
        testChildBudgetFailsClosed();
        testFullTreeDiagnosticsEnumeratesAllTerminalBranches();
        testKnownDynamicCrossingFixtureDetourCertifies();
        testMinvoRestrictionAndQpLeafConstraint();
        testIndependentSafetyCheckerUsesTemporalRefinement();
        testOctopusFeatureGate();
        std::cout << "test_temporal_refinement: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_temporal_refinement: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
