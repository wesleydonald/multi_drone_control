#include "dynamic_planner/minimum_time.hpp"
#include "test_common.hpp"

#include <cmath>
#include <iostream>

using namespace dynamic_planner;

int main() {
    try {
        requireNear(minimumTimeDoubleIntegrator1D(0, 0, 0, 0, 1.0, 1.5),
                    0.0, 1e-12, "zero-distance minimum time");
        requireNear(minimumTimeDoubleIntegrator1D(0, 0, 0.5, 0, 1.0, 1.5),
                    1.1547005383792515, 1e-12, "triangular minimum time");
        requireNear(minimumTimeDoubleIntegrator1D(0, 0, 1.0, 0, 1.0, 1.5),
                    1.6666666666666665, 1e-12, "speed-limited minimum time");
        requireNear(minimumTimeDoubleIntegrator1D(1.0, 0, 0.0, 0, 1.0, 1.5),
                    1.6666666666666665, 1e-12, "reverse symmetry");

        const double t3 = minimumTimeDoubleIntegrator3D(
            Vec3::Zero(), Vec3::Zero(), Vec3(1.0, 0.5, 0.1), Vec3::Zero(),
            Vec3::Ones(), Vec3::Constant(1.5));
        requireNear(t3, 1.6666666666666665, 1e-12, "3D max-axis minimum time");
        std::cout << "test_minimum_time: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_minimum_time: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
