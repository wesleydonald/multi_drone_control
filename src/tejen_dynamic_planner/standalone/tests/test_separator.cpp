#include "dynamic_planner/separator.hpp"
#include "test_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

Eigen::MatrixXd boxVertices(const Eigen::Vector3d& center, const Eigen::Vector3d& half) {
    Eigen::MatrixXd vertices(8, 3);
    int row = 0;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                vertices.row(row++) =
                    (center + Eigen::Vector3d(sx * half.x(), sy * half.y(), sz * half.z())).transpose();
            }
        }
    }
    return vertices;
}

Eigen::MatrixXd rotateAboutZ(const Eigen::MatrixXd& vertices, double radians) {
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return vertices * rotation.transpose();
}

void requirePlaneValid(const dynamic_planner::SeparationResult& result,
                       const Eigen::MatrixXd& first,
                       const Eigen::MatrixXd& second,
                       double tolerance = 1e-7) {
    requireTrue(result.feasible, "separator expected feasible");
    requireTrue(result.plane.has_value(), "feasible result must contain plane");
    const auto& plane = result.plane.value();
    const Eigen::VectorXd first_values =
        first * plane.normal + Eigen::VectorXd::Constant(first.rows(), plane.offset);
    const Eigen::VectorXd second_values =
        second * plane.normal + Eigen::VectorXd::Constant(second.rows(), plane.offset);
    requireTrue(first_values.maxCoeff() <= -1.0 + tolerance,
                "first hull violates <= -1 separator side");
    requireTrue(second_values.minCoeff() >= 1.0 - tolerance,
                "second hull violates >= +1 separator side");
    requireTrue(result.geometric_gap_m.has_value() && result.geometric_gap_m.value() > 0.0,
                "feasible separator must have positive geometric gap");
}

void testLPBuilder() {
    Eigen::MatrixXd first(2, 3);
    first << 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0;
    Eigen::MatrixXd second(1, 3);
    second << 2.0, 0.0, 0.0;

    const auto lp = dynamic_planner::buildSeparationLP(first, second);
    Eigen::RowVector4d row0; row0 << 0.0, 0.0, 0.0, 1.0;
    Eigen::RowVector4d row1; row1 << 0.0, 1.0, 0.0, 1.0;
    Eigen::RowVector4d row2; row2 << -2.0, 0.0, 0.0, -1.0;
    requireMatrixNear(lp.a_ub.row(0), row0, 1e-15, "LP row 0");
    requireMatrixNear(lp.a_ub.row(1), row1, 1e-15, "LP row 1");
    requireMatrixNear(lp.a_ub.row(2), row2, 1e-15, "LP row 2");
    requireMatrixNear(lp.b_ub, Eigen::Vector3d::Constant(-1.0), 1e-15, "LP b_ub");
}

void testDisjointBoxes() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0.0, 0.0, 1.0}, {0.2, 0.2, 0.2});
    const auto second = boxVertices({1.0, 0.1, 1.0}, {0.2, 0.2, 0.2});
    const auto result = separator.solve(first, second);
    requirePlaneValid(result, first, second);
    requireTrue(result.backend == "glpk", "separator backend must be glpk");
    requireTrue(separator.numLPsRun() == 1, "LP counter mismatch");
}

void testNearTouchingBoxes() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0.0, 0.0, 1.0}, {0.25, 0.20, 0.15});
    const auto second = boxVertices({0.5005, 0.0, 1.0}, {0.25, 0.20, 0.15});
    requirePlaneValid(separator.solve(first, second), first, second);
}

void testTouchingBoxesInfeasible() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0.0, 0.0, 1.0}, {0.25, 0.20, 0.15});
    const auto second = boxVertices({0.50, 0.0, 1.0}, {0.25, 0.20, 0.15});
    const auto result = separator.solve(first, second);
    requireTrue(!result.feasible, "touching convex hulls must be infeasible");
    requireTrue(!result.plane.has_value(), "infeasible result must not contain plane");
}

void testOverlappingRotatedBoxesInfeasible() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0.0, 0.0, 1.0}, {0.30, 0.22, 0.18});
    const auto second = rotateAboutZ(
        boxVertices({0.28, 0.06, 1.0}, {0.28, 0.18, 0.16}),
        31.0 * M_PI / 180.0);
    const auto result = separator.solve(first, second);
    requireTrue(!result.feasible, "overlapping rotated boxes must be infeasible");
}

void testRotatedDisjointPolyhedra() {
    dynamic_planner::Separator separator;
    const auto first = rotateAboutZ(
        boxVertices({0.0, 0.0, 1.0}, {0.24, 0.16, 0.12}),
        22.0 * M_PI / 180.0);
    const auto second = rotateAboutZ(
        boxVertices({0.75, 0.55, 1.12}, {0.22, 0.14, 0.16}),
        -37.0 * M_PI / 180.0);
    requirePlaneValid(separator.solve(first, second), first, second);
}

void testSwappingSets() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0.0, 0.0, 1.0}, {0.2, 0.2, 0.2});
    const auto second = boxVertices({0.9, 0.3, 1.1}, {0.15, 0.18, 0.15});
    requirePlaneValid(separator.solve(first, second), first, second);
    requirePlaneValid(separator.solve(second, first), second, first);
}

void testProjectionCertificateUsesCurrentSupportsAndMargin() {
    const auto first = boxVertices({0., 0., 1.}, {.2, .2, .2});
    const auto second = boxVertices({.405, 0., 1.}, {.2, .2, .2});
    const auto certificate = dynamic_planner::certifyProjectedSeparation(
        first, second, Eigen::Vector3d::UnitX(), .005 - 1e-12);
    requirePlaneValid(certificate, first, second);
    requireTrue(!dynamic_planner::certifyProjectedSeparation(
        first, second, Eigen::Vector3d::UnitX(), .006).feasible,
        "projection must not certify less than the requested physical margin");
    auto moved_first = first;
    moved_first.col(0).array() += .01;
    requireTrue(!dynamic_planner::certifyProjectedSeparation(
        moved_first, second, certificate.plane->normal).feasible,
        "changed first hull must invalidate the old separating direction");
    auto moved_second = second;
    moved_second.col(0).array() -= .01;
    requireTrue(!dynamic_planner::certifyProjectedSeparation(
        first, moved_second, certificate.plane->normal).feasible,
        "changed second hull must invalidate the old separating direction");
    requireTrue(!dynamic_planner::certifyProjectedSeparation(
        first, second, Eigen::Vector3d::Zero()).feasible,
        "zero direction cannot certify separation");
    requireTrue(!dynamic_planner::certifyProjectedSeparation(
        first, second, Eigen::Vector3d::Constant(INFINITY)).feasible,
        "nonfinite direction cannot certify separation");

    // A refined hull need not remain at the raw plane's -1 support. Recenter
    // the certificate using the current supports, not the old offset/verdict.
    const auto far = boxVertices({1., 0., 1.}, {.2, .2, .2});
    auto refined = first;
    refined.col(0).array() += .1;
    requirePlaneValid(dynamic_planner::certifyProjectedSeparation(
        refined, far, certificate.plane->normal, .001), refined, far);
}

void testFastCertificatesAndLPFallback() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0., 0., 1.}, {.2, .2, .2});
    const auto second = boxVertices({1., 0., 1.}, {.2, .2, .2});
    requirePlaneValid(separator.solveWithFastPaths(first, second), first, second);
    requireTrue(separator.numLPsRun() == 0, "axis-disjoint hulls need no LP");

    // Parallel diagonal slabs have overlapping AABBs. Their known normal is
    // still a valid certificate, including after refinement moves the first
    // hull beyond its old canonical -1 support without touching the second.
    const double angle = M_PI / 4.;
    const auto slab_a = rotateAboutZ(boxVertices({0., 0., 1.}, {.1, 2., .2}), angle);
    const auto slab_b = rotateAboutZ(boxVertices({.3, 0., 1.}, {.1, 2., .2}), angle);
    const Eigen::Vector3d direction(std::cos(angle), std::sin(angle), 0.);
    requirePlaneValid(separator.solveWithFastPaths(slab_a, slab_b, direction), slab_a, slab_b);
    requireTrue(separator.numLPsRun() == 0, "valid oblique hint should avoid LP");

    // A failed hint is not evidence of collision, and a colliding pair cannot
    // be accepted by hints, axes or centroid directions.
    requirePlaneValid(separator.solveWithFastPaths(first, second, -direction), first, second);
    const auto touching = boxVertices({.4, 0., 1.}, {.2, .2, .2});
    requireTrue(!separator.solveWithFastPaths(first, touching, direction).feasible,
                "touching hulls must fall through to LP and remain rejected");
    requireTrue(separator.numLPsRun() == 1, "touching pair must use LP fallback");
    requireTrue(!separator.solveWithFastPaths(first, first, direction).feasible,
                "overlapping hulls cannot reuse a stale hint");
    requireTrue(separator.numLPsRun() == 2, "overlap must use LP fallback");

    // A valid separating plane can require a direction other than hint,
    // coordinate axes or centroid displacement. LP must remain authoritative.
    Eigen::MatrixXd tilted_a(3, 3), tilted_b(3, 3);
    tilted_a << 0., 0., 0., 2., 0., 0., 0., 2., 0.;
    tilted_b << 1.1, 1.1, 0., 1.1, 3., 0., 3., 1.1, 0.;
    // A skewed vertex distribution points the centroid hint into overlapping
    // projections; the oblique x+y separating direction remains available.
    Eigen::MatrixXd skewed(22, 3);
    skewed.topRows(3) = tilted_b;
    skewed.bottomRows(19).rowwise() = tilted_b.row(1);
    const auto lps_before = separator.numLPsRun();
    requirePlaneValid(separator.solveWithFastPaths(tilted_a, skewed,
                                                  Eigen::Vector3d::UnitZ()), tilted_a, skewed);
    requireTrue(separator.numLPsRun() == lps_before + 1,
                "oblique separation unavailable to cheap hints must use LP");
}

void testMinimumGapRepairAndLegacyParity() {
    dynamic_planner::Separator separator;
    const auto first = boxVertices({0., 0., 1.}, {0., 0., 0.});
    const double tangent_half = (.6 - .00099) * std::sqrt(2.) - .05;
    auto wall = rotateAboutZ(boxVertices({0., 0., 0.}, {.05, tangent_half, 2.}),
                             M_PI / 4.);
    wall.rowwise() += Eigen::Vector3d(.6, .6, 1.).transpose();
    const auto legacy = separator.solveWithFastPaths(first, wall);
    requirePlaneValid(legacy, first, wall);
    requireTrue(legacy.geometric_gap_m < .001, "fixture needs a tight valid axis plane");
    const auto zero = separator.solveWithMinimumGap(first, wall, 0.);
    requireMatrixNear(zero.plane->normal, legacy.plane->normal, 0.,
                      "zero-gap request changed legacy normal");
    requireNear(zero.plane->offset, legacy.plane->offset, 0.,
                "zero-gap request changed legacy offset");
    const auto repaired = separator.solveWithMinimumGap(first, wall, .001, legacy.plane->normal);
    requirePlaneValid(repaired, first, wall);
    requireTrue(repaired.geometric_gap_m >= .001, "repair weakened requested gap");
    requireTrue(separator.numLPsRun() == 0, "diagonal repair should need projections only");

    // A hint is never a cached safe verdict for changed current geometry.
    requireTrue(!separator.solveWithMinimumGap(first, first, .001,
                    repaired.plane->normal).feasible, "changed hull reused a stale verdict");
    requireTrue(separator.numLPsRun() == 1, "overlap should use at most one LP");
    const auto narrow = boxVertices({.5005, 0., 1.}, {.5, 2., 2.});
    const auto before = separator.numLPsRun();
    const auto rejected = separator.solveWithMinimumGap(first, narrow, .001);
    requireTrue(!rejected.feasible && !rejected.plane,
                "insufficient true gap retained a feasible result");
    requireTrue(separator.numLPsRun() <= before + 1, "repair exceeded one LP");
    for (double invalid : {-1., static_cast<double>(NAN)}) {
        bool threw = false;
        try { separator.solveWithMinimumGap(first, wall, invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        requireTrue(threw, "invalid physical gap was accepted");
    }
}

}  // namespace

int main() {
    try {
        testLPBuilder();
        testDisjointBoxes();
        testNearTouchingBoxes();
        testTouchingBoxesInfeasible();
        testOverlappingRotatedBoxesInfeasible();
        testRotatedDisjointPolyhedra();
        testSwappingSets();
        testProjectionCertificateUsesCurrentSupportsAndMargin();
        testFastCertificatesAndLPFallback();
        testMinimumGapRepairAndLegacyParity();
        std::cout << "test_separator: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_separator: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
