#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/minvo.hpp"

#include <Eigen/Core>
#include <iomanip>
#include <iostream>

int main() {
    using dynamic_planner::ControlPoints;
    using dynamic_planner::State;
    using dynamic_planner::Vec3;

    std::cout << std::setprecision(15);

    const auto knots = dynamic_planner::openUniformKnots(0.0, 6.0, 4);
    const State initial{
        Vec3(0.2, -0.1, 1.3),
        Vec3(0.3, -0.2, 0.1),
        Vec3(0.15, 0.05, -0.1),
    };
    const auto q012 = dynamic_planner::initialControlPointsFromState(initial, knots);

    std::cout << "R6.3A.1 C++ math parity demo\n";
    std::cout << "knots:";
    for (double knot : knots) {
        std::cout << ' ' << knot;
    }
    std::cout << "\n\nq0/q1/q2:\n" << q012 << "\n";

    ControlPoints cps(7, 3);
    cps <<
        0.2, -0.1, 1.3,
        0.35, -0.2, 1.35,
        0.7625, -0.3625, 1.375,
        1.2, 0.1, 1.5,
        1.7, 0.8, 1.45,
        2.0, 1.2, 1.4,
        2.0, 1.2, 1.4;

    const auto velocity = dynamic_planner::derivativeSpline(cps, knots, 3, 1);
    const auto acceleration = dynamic_planner::derivativeSpline(cps, knots, 3, 2);
    std::cout << "\nvelocity derivative control points:\n" << velocity.control_points << "\n";
    std::cout << "\nacceleration derivative control points:\n" << acceleration.control_points << "\n";

    const dynamic_planner::FourPoints q_interval0 = cps.middleRows<4>(0);
    std::cout << "\nposition MINVO vertices, interval 0:\n"
              << dynamic_planner::minvoVertices(q_interval0, 0, 4) << "\n";

    const dynamic_planner::ThreePoints v_interval0 = velocity.control_points.middleRows<3>(0);
    std::cout << "\nvelocity MINVO vertices, interval 0:\n"
              << dynamic_planner::minvoVelocityVertices(v_interval0, 0, 4) << "\n";

    const auto state_mid = dynamic_planner::evaluateCubicState(cps, knots, 3.0);
    std::cout << "\nstate at t=3.0:\n";
    std::cout << "p = " << state_mid.position.transpose() << '\n';
    std::cout << "v = " << state_mid.velocity.transpose() << '\n';
    std::cout << "a = " << state_mid.acceleration.transpose() << '\n';

    std::cout << "\nIf ctest passes, send the full ctest output plus this demo output back to ChatGPT.\n";
    return 0;
}
