#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/minvo.hpp"
#include "test_common.hpp"

#include <Eigen/Core>
#include <iostream>

int main() {
    try {
        const auto position_mats = dynamic_planner::positionConverters(4);
        const auto velocity_mats = dynamic_planner::velocityConverters(4);
        requireTrue(position_mats.size() == 4, "four position converter matrices expected");
        requireTrue(velocity_mats.size() == 4, "four velocity converter matrices expected");

        dynamic_planner::ControlPoints cps(7, 3);
        cps <<
            0.2, -0.1, 1.3,
            0.35, -0.2, 1.35,
            0.7625, -0.3625, 1.375,
            1.2, 0.1, 1.5,
            1.7, 0.8, 1.45,
            2.0, 1.2, 1.4,
            2.0, 1.2, 1.4;

        dynamic_planner::FourPoints q0 = cps.middleRows<4>(0);
        const auto mv0 = dynamic_planner::minvoVertices(q0, 0, 4);
        dynamic_planner::FourPoints expected_mv0;
        expected_mv0 <<
            0.16059733474363710, -0.083680779159099328, 1.2928998849815712,
            0.29198723603238119, -0.17296491815091244, 1.3309687252878808,
            0.62075563665827194, -0.27426242895006414, 1.3758175467443434,
            0.77352502536618839, -0.25151805386541448, 1.3950724381217816;
        requireMatrixNear(mv0, expected_mv0, 3e-12,
                          "position MINVO interval 0 parity");

        dynamic_planner::FourPoints q2 = cps.middleRows<4>(2);
        const auto mv2 = dynamic_planner::minvoVertices(q2, 2, 4);
        dynamic_planner::FourPoints expected_mv2;
        expected_mv2 <<
            1.1744756491554404, 0.091984357523962793, 1.4717095284359734,
            1.3063118290406419, 0.25785164646729331, 1.4802121117356013,
            1.5942076007746093, 0.64786208150048130, 1.4614770112203617,
            1.7277541396533440, 0.83254331937268322, 1.4426138506234480;
        requireMatrixNear(mv2, expected_mv2, 3e-12,
                          "position MINVO interval 2 parity");

        const auto knots = dynamic_planner::openUniformKnots(0.0, 6.0, 4);
        const auto velocity = dynamic_planner::derivativeSpline(cps, knots, 3, 1);

        dynamic_planner::ThreePoints v0 = velocity.control_points.middleRows<3>(0);
        const auto vmv0 = dynamic_planner::minvoVelocityVertices(v0, 0, 4);
        dynamic_planner::ThreePoints expected_vmv0;
        expected_vmv0 <<
            0.29597135998102303, -0.22110967937749190, 0.10354518601245423,
            0.38368030914179468, -0.12951461828693317, 0.042361303216203225,
            0.35611261111916359, 0.094024542090935317, 0.050621143029700527;
        requireMatrixNear(vmv0, expected_vmv0, 3e-12,
                          "velocity MINVO interval 0 parity");

        dynamic_planner::ThreePoints v2 = velocity.control_points.middleRows<3>(2);
        const auto vmv2 = dynamic_planner::minvoVelocityVertices(v2, 2, 4);
        dynamic_planner::ThreePoints expected_vmv2;
        expected_vmv2 <<
            0.31217770029310721, 0.38395481091225564, 0.030156598000260767,
            0.32708330130379226, 0.44791646980454142, -0.024999771925430254,
            0.31698912760072895, 0.43687889868590435, -0.046823282464280150;
        requireMatrixNear(vmv2, expected_vmv2, 3e-12,
                          "velocity MINVO interval 2 parity");

        // R6.2 synthetic regression: old B-spline velocity hull violates 1.0,
        // while RMADER's MINVO velocity hull remains within the same bound.
        dynamic_planner::ThreePoints synthetic;
        synthetic <<
            0.90570362, 0.0, 0.0,
            -0.97641655, 0.0, 0.0,
            1.11490582, 0.0, 0.0;
        requireTrue(synthetic.col(0).cwiseAbs().maxCoeff() > 1.0,
                    "synthetic old B-spline hull should violate 1.0");
        const auto synthetic_mv = dynamic_planner::minvoVelocityVertices(synthetic, 0, 4);
        requireTrue(synthetic_mv.col(0).cwiseAbs().maxCoeff() < 1.0,
                    "synthetic MINVO hull should fit inside 1.0");

        // Boundary-aware converter counts for future local segment-count diagnostics.
        requireTrue(dynamic_planner::positionConverters(5).size() == 5,
                    "five-segment position converters");
        requireTrue(dynamic_planner::positionConverters(8).size() == 8,
                    "eight-segment position converters");
        requireTrue(dynamic_planner::velocityConverters(8).size() == 8,
                    "eight-segment velocity converters");

        std::cout << "test_minvo: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_minvo: FAIL: " << error.what() << '\n';
        return 1;
    }
}
