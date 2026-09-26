#include "tejen_dynamic_planner/kinematics_filter.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}

int main() {
    try {
        tejen_dynamic_planner::KinematicsFilter filter(0.15);
        auto s0 = filter.update(
            dynamic_planner::Vec3(0.0, 0.0, 1.5),
            dynamic_planner::Vec3(0.0, 0.0, 0.0), 5.0);
        require(s0.acceleration.norm() < 1e-15, "first acceleration must be zero");

        auto s1 = filter.update(
            dynamic_planner::Vec3(0.0, 0.0, 1.5),
            dynamic_planner::Vec3(0.1, 0.0, 0.0), 5.1);
        const double alpha = 1.0 - std::exp(-0.1 / 0.15);
        require(std::abs(s1.acceleration.x() - alpha) < 1e-12,
                "filtered acceleration does not match first-order filter math");
        require(std::abs(s1.acceleration.y()) < 1e-15 &&
                std::abs(s1.acceleration.z()) < 1e-15,
                "unexpected cross-axis acceleration");

        auto duplicate = filter.update(
            dynamic_planner::Vec3(0.0, 0.0, 1.5),
            dynamic_planner::Vec3(0.2, 0.0, 0.0), 5.1);
        require(duplicate.acceleration.norm() < 1e-15,
                "duplicate timestamp should reset differentiator rather than spike");

        std::cout << "C.1b kinematics filter: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_kinematics_filter FAILED: " << exc.what() << '\n';
        return 1;
    }
}
