#include "dynamic_planner/trajectory_refinement.hpp"
#include "dynamic_planner/validation_scenarios.hpp"

#include "test_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using dynamic_planner::ControlPoints;
using dynamic_planner::OctopusSearch;
using dynamic_planner::RefinementConfig;
using dynamic_planner::RefinementResult;
using dynamic_planner::ValidationScenario;

Eigen::MatrixXd testBox(const Eigen::Vector3d& centre, const Eigen::Vector3d& half) {
    Eigen::MatrixXd result(8,3);
    for (int i=0; i<8; ++i) for (int a=0; a<3; ++a)
        result(i,a)=centre(a)+((i & (1<<a)) ? half(a) : -half(a));
    return result;
}

void testNavigationSoftGoalWeights() {
    using namespace dynamic_planner;
    const double duration = 2.1698;
    const auto knots = openUniformKnots(0., duration, 4);
    OctopusResult raw;
    raw.success = true;
    raw.control_points = ControlPoints::Zero(7, 3);
    RefinementConfig config;
    config.xyz_min = Vec3::Constant(-5.);
    config.xyz_max = Vec3::Constant(5.);
    config.v_max = Vec3(2.25, 2.25, 1.);
    config.a_max = Vec3(1.9, 1.9, 1.5);
    config.j_max = Vec3(6.5, 6.5, 4.);
    const Vec3 goal(std::sqrt(2.), -std::sqrt(2.), 0.);
    TerminalBoundary boundary;
    boundary.mode = TerminalMode::Stopped;
    double previous = 0.;
    for (double weight : {10., 30., 100.}) {
        config.goal_weight = weight;
        const auto r = refineOctopusTrajectory(raw, knots, goal, {}, config, boundary);
        requireTrue(r.success, "navigation weight probe must solve: " + r.message);
        const auto end = evaluateCubicState(*r.control_points, knots, duration);
        const double progress = end.position.norm();
        requireTrue(progress > previous, "stronger soft goal must increase held-start progress");
        const double relaxed = 2.*weight/(weight+1024./std::pow(duration, 5));
        if (weight < 100.)
            requireNear(progress, relaxed, 1e-5, "unconstrained native endpoint prediction");
        else {
            requireTrue(progress <= relaxed + 1e-5,
                        "hard dynamics must limit aggressive soft endpoint progress");
            requireNear(*r.max_acceleration_axis, 1.9, 1e-5,
                        "high weight fixture reaches existing acceleration bound");
        }
        requireTrue(*r.max_inequality_violation <= config.validation_tolerance,
                    "weight must preserve derivative constraints");
        requireMatrixNear(end.velocity, Vec3::Zero(), 0., "stopped velocity preserved");
        requireMatrixNear(end.acceleration, Vec3::Zero(), 0., "stopped acceleration preserved");
        std::cout << "NAV21 weight=" << weight << " initial_progress=" << progress << '\n';
        previous = progress;
    }
}

void testStoppedEndpointIsExactAfterNumericalRefinement() {
    using namespace dynamic_planner;
    const auto knots = openUniformKnots(21.8, 23.3, 4);
    State initial;
    initial.position = Vec3(-1.1, .8, 1.35);
    initial.velocity = Vec3(.35, -.28, 0.);
    initial.acceleration = Vec3(.1, -.05, 0.);
    const auto prefix = initialControlPointsFromState(initial, knots);
    OctopusResult raw;
    raw.success = true;
    raw.control_points = ControlPoints::Zero(7, 3);
    raw.control_points->topRows(3) = prefix;
    for (int i = 3; i < 7; ++i) raw.control_points->row(i) = prefix.row(2);
    RefinementConfig config;
    config.xyz_min = Vec3(-2.5, -2.5, .587);
    config.xyz_max = Vec3(2.5, 2.5, 1.45);
    config.v_max = Vec3(1.75, 1.75, 1.);
    config.a_max = Vec3(1., 1., 1.5);
    TerminalBoundary boundary;
    boundary.mode = TerminalMode::Stopped;
    const auto result = refineOctopusTrajectory(raw, knots, Vec3(.5, -.5, 1.35), {}, config, boundary);
    requireTrue(result.success, "stopped endpoint fixture must solve: " + result.message);
    const auto& cp = *result.control_points;
    requireMatrixNear(cp.bottomRows(2), cp.middleRows(cp.rows()-3, 2), 0.,
                      "stopped QP must expose exactly repeated terminal control points");
    const auto endpoint = evaluateCubicState(cp, knots, 23.3);
    requireMatrixNear(endpoint.velocity, Vec3::Zero(), 0., "terminal velocity must be exact zero");
    requireMatrixNear(endpoint.acceleration, Vec3::Zero(), 0., "terminal acceleration must be exact zero");
    requireMatrixNear(cp.topRows(3), prefix, config.validation_tolerance,
                      "stopped endpoint normalization must preserve initial P/V/A constraints");
    requireTrue(*result.max_equality_residual <= config.validation_tolerance &&
                *result.max_inequality_violation <= config.validation_tolerance &&
                result.separator_recheck_passed,
                "normalized output must still pass existing constraint checks");
}

void testIncompatibleCornerPlaneIsRepairedWithoutRelaxingMargin() {
    using namespace dynamic_planner;
    const auto knots = openUniformKnots(0., 4., 4);
    OctopusResult raw;
    raw.success = true;
    raw.control_points = ControlPoints::Zero(7, 3);
    raw.control_points->col(2).setOnes();
    const auto original = *raw.control_points;
    const double tiny_axis_gap = .00099;
    // A long diagonal wall's AABB corner is just .99mm away in X, while
    // its actual diagonal face is much farther away. The +X plane is valid
    // for search, but cannot admit the immutable start with QP's 1mm inset.
    const double tangent_half = (.6 - tiny_axis_gap) * std::sqrt(2.) - .05;
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
        std::acos(-1.) / 4., Vec3::UnitZ()).toRotationMatrix();
    Eigen::MatrixXd wall = testBox(Vec3::Zero(), Vec3(.05, tangent_half, 2.));
    wall = (wall * rotation.transpose()).eval();
    wall.rowwise() += Vec3(.6, .6, 1.).transpose();
    TimeIndexedObstacle obstacle;
    obstacle.name = "diagonal_wall";
    obstacle.interval_vertices.assign(4, wall);
    for (int i = 0; i < 4; ++i) {
        IntervalSeparator item;
        item.obstacle_name = obstacle.name;
        item.interval_index = i;
        item.result = certifyProjectedSeparation(
            minvoVertices(raw.control_points->middleRows(i, 4), i, 4), wall, Vec3::UnitX());
        requireTrue(item.result.feasible, "corner fixture must have a valid raw separator");
        item.obstacle_vertices = wall;
        raw.separators.push_back(item);
    }
    const auto original_normal = raw.separators.front().result.plane->normal;
    RefinementConfig config;
    config.xyz_min = Vec3(-2., -2., 0.);
    config.xyz_max = Vec3(3., 3., 3.);
    config.separator_margin_m = .001;
    const auto result = refineOctopusTrajectory(raw, knots, Vec3(.4, .4, 1.), {obstacle}, config);
    requireTrue(result.success, "valid alternative corridor was lost: " + result.message);
    requireTrue(result.separator_recheck_passed, "repaired corridor bypassed postchecking");
    requireMatrixNear(result.control_points->topRows(3), original.topRows(3), 1e-8,
                      "repair moved immutable starting prefix");
    requireMatrixNear(raw.separators.front().result.plane->normal, original_normal, 0.,
                      "refinement mutated search's original diagnostic plane");
    const auto end = evaluateCubicState(*result.control_points, knots, 4.);
    requireTrue((end.position - Vec3(0.,0.,1.)).norm() > .1,
                "corner repair returned a hold instead of useful refinement");
    requireMatrixNear(end.velocity, Vec3::Zero(), 1e-8, "corner repair terminal velocity");
    requireMatrixNear(end.acceleration, Vec3::Zero(), 1e-8, "corner repair terminal acceleration");
    Separator oracle;
    for (int i=0; i<4; ++i) {
        const auto hull = minvoVertices(result.control_points->middleRows(i,4), i,4);
        const auto checked = oracle.solve(hull, wall);
        requireTrue(checked.feasible, "independent LP rejects repaired curve");
    }
}

void testNoCompatiblePlaneDoesNotRelaxTheInset() {
    using namespace dynamic_planner;
    const auto knots = openUniformKnots(0., 4., 4);
    OctopusResult raw;
    raw.success = true;
    raw.control_points = ControlPoints::Zero(7,3);
    raw.control_points->col(2).setOnes();
    const auto wall = testBox(Vec3(.5005,0.,1.), Vec3(.5,2.,2.));
    TimeIndexedObstacle obstacle;
    obstacle.name = "too_close_wall";
    obstacle.interval_vertices.assign(4,wall);
    for (int i=0; i<4; ++i) {
        IntervalSeparator item;
        item.obstacle_name=obstacle.name; item.interval_index=i;
        item.result=certifyProjectedSeparation(
            minvoVertices(raw.control_points->middleRows(i,4),i,4),wall,Vec3::UnitX());
        item.obstacle_vertices=wall;
        raw.separators.push_back(item);
    }
    RefinementConfig config;
    config.xyz_min=Vec3(-2.,-2.,0.);config.xyz_max=Vec3(3.,3.,3.);
    const auto result=refineOctopusTrajectory(raw,knots,Vec3(-.5,0.,1.),{obstacle},config);
    requireTrue(!result.success && !result.control_points,
                "incompatible prefix must not silently relax 1mm inset");
    requireTrue(result.status=="PREFIX_CORRIDOR_INCOMPATIBLE",
                "known incompatible prefix must have an explicit diagnostic: "+result.status);
    requireTrue(result.message.find(obstacle.name)!=std::string::npos,
                "incompatible-prefix diagnostic lost obstacle identity");
    requireTrue(result.working_set_recalculations==0,
                "known incompatible prefix should not enter the QP solver");
}

void testMovingPrefixContradictionIsNotJustAnInitialPointCheck() {
    using namespace dynamic_planner;
    const auto knots=openUniformKnots(0.,7.24947101899827,4);
    State initial;
    initial.position=Vec3(-.883498243772362,-.898618727409275,1.25423546776686);
    initial.velocity=Vec3(-.000977130591935,.001185960404681,-.000444941588373);
    initial.acceleration=Vec3(.0000667225000543,-.000322855473341,.000649494527470);
    const auto prefix=initialControlPointsFromState(initial,knots);
    OctopusResult raw;
    raw.success=true;
    raw.control_points=ControlPoints(7,3);
    raw.control_points->topRows(3)=prefix;
    for(int i=3;i<7;++i)raw.control_points->row(i)=prefix.row(2);
    const Vec3 normal=Vec3(1.,1.,0.).normalized();
    const Vec3 centre=initial.position+(.001030112735852+.2)*normal;
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(std::acos(-1.)/4.,Vec3::UnitZ()).toRotationMatrix();
    Eigen::MatrixXd hull=testBox(Vec3::Zero(),Vec3(.2,2.,2.));
    hull=(hull*rotation.transpose()).eval();hull.rowwise()+=centre.transpose();
    TimeIndexedObstacle obstacle;
    obstacle.name="peer_swept_body";obstacle.interval_vertices.assign(4,hull);
    for(int i=0;i<4;++i){
        IntervalSeparator item;
        item.obstacle_name=obstacle.name;item.interval_index=i;
        item.result=certifyProjectedSeparation(
            minvoVertices(raw.control_points->middleRows(i,4),i,4),hull,normal);
        requireTrue(item.result.feasible,"near-moving fixture raw hull must separate");
        item.obstacle_vertices=hull;raw.separators.push_back(item);
    }
    RefinementConfig config;
    config.xyz_min=Vec3(-3.,-3.,0.);config.xyz_max=Vec3(3.,3.,3.);
    const auto result=refineOctopusTrajectory(raw,knots,initial.position-Vec3(.1,.1,0.),{obstacle},config);
    requireTrue(!result.success && result.status=="PREFIX_CORRIDOR_INCOMPATIBLE",
                "moving-prefix contradiction must be identified even though q0 admits 1mm");
    requireMatrixNear(raw.control_points->topRows(3),prefix,0.,"moving-prefix check changed P/V/A");
}

RefinementResult run(const std::string& scene_name) {
    const ValidationScenario scene = dynamic_planner::makeValidationScenario(scene_name, 7);
    const auto raw = OctopusSearch(
        scene.knots,
        scene.initial_control_points,
        scene.goal,
        scene.obstacles,
        scene.octopus_config).search();
    if (!raw.success) {
        throw std::runtime_error("R4 failed in refinement regression: " + raw.status);
    }

    RefinementConfig config;
    config.v_max = scene.octopus_config.v_max;
    config.a_max = scene.octopus_config.a_max;
    config.j_max = Eigen::Vector3d::Constant(4.0);
    config.jerk_weight = 1.0;
    config.goal_weight = 10.0;
    config.goal_acceptance_m = 0.05;
    config.separator_margin_m = 0.001;
    config.xyz_min = scene.octopus_config.xyz_min;
    config.xyz_max = scene.octopus_config.xyz_max;
    config.max_working_set_recalculations = 500;
    config.validation_tolerance = 2e-6;

    auto result = dynamic_planner::refineOctopusTrajectory(
        raw, scene.knots, scene.goal, scene.obstacles, config);
    if (result.success) {
        // Independent LP-only oracle for every refined interval. Do not use
        // the new projection helper to decide whether its own output is safe.
        dynamic_planner::Separator oracle;
        for (const auto& item : raw.separators) {
            const auto map = item.ego_vertex_map.value_or(
                dynamic_planner::positionConverters(4).at(item.interval_index).transpose());
            const dynamic_planner::FourPoints vertices = map *
                result.control_points->middleRows(item.interval_index, 4);
            requireTrue(item.obstacle_vertices.has_value(), "missing final obstacle hull");
            requireTrue(oracle.solve(vertices, *item.obstacle_vertices).feasible,
                        scene_name + " fast postcheck disagrees with independent LP");
        }
        auto invalidated = raw;
        requireTrue(!invalidated.separators.empty(), "fixture must contain a separator");
        Eigen::MatrixXd enclosing_box(8, 3);
        for (int i = 0; i < 8; ++i) {
            enclosing_box.row(i) = Eigen::RowVector3d(
                (i & 1) ? 100. : -100., (i & 2) ? 100. : -100., (i & 4) ? 100. : -100.);
        }
        invalidated.separators.front().obstacle_vertices = enclosing_box;
        const auto rejected = dynamic_planner::refineOctopusTrajectory(
            invalidated, scene.knots, scene.goal, scene.obstacles, config);
        requireTrue(!rejected.success && !rejected.separator_recheck_passed,
                    "refinement cannot certify a changed hull using stale raw-plane evidence");
    }
    return result;
}


void testNonStoppedTerminalUsesFullVelocityConstraintMatrix() {
    const auto knots = dynamic_planner::openUniformKnots(0.0, 4.0, 4);
    ControlPoints cps(7, 3);
    cps <<
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.4, 0.0, 1.0,
        0.8, 0.0, 1.0,
        1.0, 0.0, 1.0,
        1.0, 0.0, 1.0;

    dynamic_planner::OctopusResult raw;
    raw.success = true;
    raw.control_points = cps;

    RefinementConfig config;
    config.xyz_min = Eigen::Vector3d(-2.0, -2.0, 0.0);
    config.xyz_max = Eigen::Vector3d(3.0, 2.0, 2.0);

    dynamic_planner::TerminalBoundary stopped;
    stopped.mode = dynamic_planner::TerminalMode::Stopped;
    stopped.position = cps.row(6).transpose();

    dynamic_planner::TerminalBoundary continuation;
    continuation.mode = dynamic_planner::TerminalMode::Continuation;
    continuation.position = cps.row(6).transpose();

    dynamic_planner::TerminalBoundary moving;
    moving.mode = dynamic_planner::TerminalMode::MovingRendezvous;
    moving.position = cps.row(6).transpose();
    moving.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);

    const auto stopped_qp = dynamic_planner::buildRefinementQP(
        raw, knots, stopped.position, config, stopped);
    const auto continuation_qp = dynamic_planner::buildRefinementQP(
        raw, knots, continuation.position, config, continuation);
    const auto moving_qp = dynamic_planner::buildRefinementQP(
        raw, knots, moving.position, config, moving);

    // Full MINVO velocity constraints have one additional 3-row terminal
    // interval versus the legacy stopped/RMADER matrix. Each scalar row creates
    // +/- inequalities on three axes: 3 * 2 * 3 = 18 extra inequalities.
    requireTrue(continuation_qp.a_ub.rows() == stopped_qp.a_ub.rows() + 18,
                "continuation QP did not add the full terminal velocity interval");
    requireTrue(moving_qp.a_ub.rows() == stopped_qp.a_ub.rows() + 18,
                "moving-rendezvous QP did not add the full terminal velocity interval");
}

void requireHealthy(const RefinementResult& result, const std::string& scene) {
    requireTrue(result.success, scene + " refinement failed: " + result.message);
    requireTrue(result.control_points.has_value(), scene + " missing refined control points");
    requireTrue(result.refined_goal_distance_m.has_value(), scene + " missing refined goal distance");
    requireTrue(result.refined_objective.has_value(), scene + " missing refined objective");
    requireTrue(result.refined_jerk_cost.has_value(), scene + " missing refined jerk cost");
    requireTrue(result.max_velocity_axis.has_value(), scene + " missing velocity diagnostic");
    requireTrue(result.max_acceleration_axis.has_value(), scene + " missing acceleration diagnostic");
    requireTrue(result.max_jerk_axis.has_value(), scene + " missing jerk diagnostic");
    requireTrue(result.max_equality_residual.has_value(), scene + " missing equality residual");
    requireTrue(result.max_inequality_violation.has_value(), scene + " missing inequality residual");
    requireTrue(result.min_fixed_separator_slack.has_value(), scene + " missing separator slack");
    requireTrue(result.separator_recheck_passed, scene + " separator recheck failed");
    requireTrue(*result.max_equality_residual <= 2e-6, scene + " equality residual too large");
    requireTrue(*result.max_inequality_violation <= 2e-6, scene + " inequality residual too large");
    requireTrue(*result.min_fixed_separator_slack >= -2e-6, scene + " fixed separator slack too negative");
    requireTrue(*result.max_velocity_axis <= 1.0 + 2e-6, scene + " velocity certificate violated");
    requireTrue(*result.max_acceleration_axis <= 1.5 + 2e-6, scene + " acceleration certificate violated");
    requireTrue(*result.max_jerk_axis <= 4.0 + 2e-6, scene + " jerk certificate violated");
    requireTrue(*result.refined_objective < result.raw_objective, scene + " objective did not improve");
    requireTrue(*result.refined_jerk_cost < result.raw_jerk_cost, scene + " jerk cost did not improve");

    const ControlPoints& cps = *result.control_points;
    requireTrue((cps.row(cps.rows() - 1) - cps.row(cps.rows() - 2)).norm() <= 2e-8,
                  scene + " qN != qN-1");
    requireTrue((cps.row(cps.rows() - 2) - cps.row(cps.rows() - 3)).norm() <= 2e-8,
                  scene + " qN-1 != qN-2");
}

}  // namespace

int main() {
    try {
        testNavigationSoftGoalWeights();
        testStoppedEndpointIsExactAfterNumericalRefinement();
        testIncompatibleCornerPlaneIsRepairedWithoutRelaxingMargin();
        testNoCompatiblePlaneDoesNotRelaxTheInset();
        testMovingPrefixContradictionIsNotJustAnInitialPointCheck();
        testNonStoppedTerminalUsesFullVelocityConstraintMatrix();
        const auto default_result = run("default");
        requireHealthy(default_result, "default");
        requireNear(default_result.raw_goal_distance_m, 0.19764235376052341, 2e-10,
                          "default R4 goal parity");
        // The R4 control points are identical to the Python R6.2 reference, but
        // the fixed separating-plane LP is non-unique. Python R6.2 used
        // SciPy/HiGHS on the validation machine, while this native port uses the
        // RMADER-style GLPK separator. Regress against the deterministic GLPK
        // solution here, while requireHealthy() independently enforces the QP
        // equalities, inequalities, dynamic certificates and separator recheck.
        requireNear(*default_result.refined_goal_distance_m, 0.0416749011841456, 5e-4,
                          "default native-GLPK refinement regression");
        requireTrue(default_result.goal_accepted, "default goal should be accepted");
        requireTrue(*default_result.refined_goal_distance_m <= 0.05 + 1e-10,
                    "default refined goal must remain inside 5 cm acceptance");

        const auto chicane_result = run("static_chicane");
        requireHealthy(chicane_result, "static_chicane");
        requireNear(chicane_result.raw_goal_distance_m, 0.603133095800587, 2e-10,
                          "chicane R4 goal parity");
        requireNear(*chicane_result.refined_goal_distance_m, 0.5407742154234216, 3e-4,
                          "chicane R6.2 goal parity");
        requireTrue(!chicane_result.goal_accepted, "4-segment chicane should remain outside 5 cm");
        requireNear(*chicane_result.max_velocity_axis, 0.878175445135685, 5e-4,
                          "chicane MINVO velocity certificate parity");

        const auto moving_result = run("moving_crossing");
        requireHealthy(moving_result, "moving_crossing");
        requireNear(moving_result.raw_goal_distance_m, 0.19764235376052341, 2e-10,
                          "moving R4 goal parity");
        requireNear(*moving_result.refined_goal_distance_m, 0.02933998172933091, 2e-4,
                          "moving R6.2 goal parity");
        requireTrue(moving_result.goal_accepted, "moving-crossing goal should be accepted");

        std::cout << "test_refinement: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_refinement: FAIL: " << error.what() << '\n';
        return 1;
    }
}
