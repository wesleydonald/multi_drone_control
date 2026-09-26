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
        std::cout << "test_separator: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_separator: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
