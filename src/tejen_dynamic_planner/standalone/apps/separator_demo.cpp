#include "dynamic_planner/separator.hpp"

#include <Eigen/Core>

#include <iomanip>
#include <iostream>

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

void printResult(const char* label, const dynamic_planner::SeparationResult& result) {
    std::cout << label << ": feasible=" << std::boolalpha << result.feasible
              << " backend=" << result.backend << " status=" << result.status << '\n';
    if (!result.feasible || !result.plane.has_value()) {
        return;
    }
    const auto& plane = result.plane.value();
    std::cout << "  normal = " << plane.normal.transpose() << '\n';
    std::cout << "  offset = " << plane.offset << '\n';
    std::cout << "  unit normal = " << plane.unitNormal().transpose() << '\n';
    std::cout << "  first max raw = " << result.first_max_value.value() << '\n';
    std::cout << "  second min raw = " << result.second_min_value.value() << '\n';
    std::cout << "  geometric gap = " << result.geometric_gap_m.value() << " m\n";
}
}  // namespace

int main() {
    try {
        std::cout << std::setprecision(15);
        std::cout << "R6.3A.2 C++ separator demo\n";
        dynamic_planner::Separator separator;

        const auto first = boxVertices({0.0, 0.0, 1.0}, {0.2, 0.2, 0.2});
        const auto disjoint = boxVertices({1.0, 0.1, 1.0}, {0.2, 0.2, 0.2});
        printResult("disjoint boxes", separator.solve(first, disjoint));

        const auto near_a = boxVertices({0.0, 0.0, 1.0}, {0.25, 0.20, 0.15});
        const auto near_b = boxVertices({0.5005, 0.0, 1.0}, {0.25, 0.20, 0.15});
        printResult("near-touching boxes", separator.solve(near_a, near_b));

        const auto touching = boxVertices({0.50, 0.0, 1.0}, {0.25, 0.20, 0.15});
        printResult("touching boxes", separator.solve(near_a, touching));

        std::cout << "LPs run = " << separator.numLPsRun() << '\n';
        std::cout << "\nSend this output plus ctest output back to ChatGPT.\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "separator_demo: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
