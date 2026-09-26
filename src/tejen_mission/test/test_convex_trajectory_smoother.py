import unittest

import numpy as np

from tejen_mission.convex_corridor import ConvexPolyhedron
from tejen_mission.convex_trajectory_smoother import (
    ConvexTrajectoryConfig,
    resolve_target_acceleration,
    solve_convex_corridor_trajectory,
)


def box_cell(minimum, maximum, start, end):
    minimum = np.asarray(minimum, dtype=float)
    maximum = np.asarray(maximum, dtype=float)
    A = np.vstack((np.eye(3), -np.eye(3)))
    b = np.concatenate((maximum, -minimum))
    return ConvexPolyhedron(A, b, np.asarray(start, dtype=float), np.asarray(end, dtype=float))


def endpoint_state(control, duration, at_start):
    p = np.asarray(control, dtype=float)
    t = float(duration)
    if at_start:
        return (
            p[0],
            7.0 * (p[1] - p[0]) / t,
            42.0 * (p[2] - 2.0 * p[1] + p[0]) / t**2,
            210.0 * (p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]) / t**3,
        )
    return (
        p[7],
        7.0 * (p[7] - p[6]) / t,
        42.0 * (p[7] - 2.0 * p[6] + p[5]) / t**2,
        210.0 * (p[7] - 3.0 * p[6] + 3.0 * p[5] - p[4]) / t**3,
    )


class ConvexTrajectorySmootherTests(unittest.TestCase):
    def test_straight_corridor_uses_single_time_scaling_retry_and_passes_headroom(self):
        corridor = (
            box_cell(
                [-0.10, -0.20, 0.80],
                [1.10, 0.20, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ),
        )
        config = ConvexTrajectoryConfig(
            max_speed_mps=1.0,
            max_acceleration_mps2=1.5,
            validation_headroom_ratio=1.10,
            max_time_scaling_retries=1,
            solver_time_s=2.0,
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            config,
            dt=0.02,
        )

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.status, "SUCCESS")
        self.assertEqual(result.qp_solve_count, 2)
        self.assertLessEqual(
            result.velocity_control_bound_mps,
            config.validation_headroom_ratio * config.max_speed_mps + 1e-9,
        )
        self.assertLessEqual(
            result.acceleration_control_bound_mps2,
            config.validation_headroom_ratio * config.max_acceleration_mps2 + 1e-9,
        )
        self.assertLessEqual(result.corridor_violation_m, 1e-4)
        np.testing.assert_allclose(result.positions[0], [0.0, 0.0, 1.0], atol=2e-4)
        np.testing.assert_allclose(result.positions[-1], [1.0, 0.0, 1.0], atol=2e-4)

    def test_two_cell_solution_is_c3_and_control_points_stay_in_owning_cells(self):
        corridor = (
            box_cell(
                [-0.10, -0.15, 0.80],
                [1.10, 0.60, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ),
            box_cell(
                [0.40, -0.10, 0.80],
                [1.15, 1.10, 1.20],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
            ),
        )
        config = ConvexTrajectoryConfig(
            max_speed_mps=3.0,
            max_acceleration_mps2=6.0,
            nominal_speed_mps=1.0,
            solver_time_s=2.0,
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            config,
            dt=0.02,
        )

        self.assertTrue(result.success, result.message)
        for control, cell in zip(result.control_points, corridor):
            self.assertLessEqual(
                float(np.max(control @ cell.A.T - cell.b[None, :])),
                1e-4,
            )

        left = endpoint_state(result.control_points[0], result.segment_times[0], False)
        right = endpoint_state(result.control_points[1], result.segment_times[1], True)
        np.testing.assert_allclose(left[0], right[0], atol=2e-4)
        np.testing.assert_allclose(left[1], right[1], atol=1e-2)
        np.testing.assert_allclose(left[2], right[2], atol=5e-2)
        np.testing.assert_allclose(left[3], right[3], atol=5.0)
        self.assertTrue(corridor[0].contains(left[0], tolerance=2e-4))
        self.assertTrue(corridor[1].contains(left[0], tolerance=2e-4))

    def test_general_non_axis_aligned_halfspace_is_enforced(self):
        base = box_cell(
            [-0.10, -0.50, 0.80],
            [1.10, 0.50, 1.20],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
        )
        diagonal = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
        cell = ConvexPolyhedron(
            np.vstack((base.A, diagonal)),
            np.concatenate((base.b, [1.05 / np.sqrt(2.0)])),
            base.segment_start,
            base.segment_end,
        )
        result = solve_convex_corridor_trajectory(
            (cell,),
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            ConvexTrajectoryConfig(
                max_speed_mps=3.0,
                max_acceleration_mps2=6.0,
                nominal_speed_mps=1.0,
                solver_time_s=2.0,
            ),
            dt=0.02,
        )
        self.assertTrue(result.success, result.message)
        self.assertLessEqual(
            float(np.max(result.control_points[0] @ cell.A.T - cell.b[None, :])),
            1e-4,
        )

    def test_structurally_infeasible_boundary_state_is_rejected_without_geometry_repair(self):
        corridor = (
            box_cell(
                [-0.10, -0.01, 0.80],
                [1.10, 0.01, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ),
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            ConvexTrajectoryConfig(
                max_speed_mps=100.0,
                max_acceleration_mps2=100.0,
                nominal_speed_mps=100.0,
                minimum_segment_time_s=0.18,
                solver_time_s=2.0,
            ),
            dt=0.02,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.status, "BOUNDARY_TIME_INFEASIBLE")


    def test_boundary_aware_time_clips_duration_before_qp(self):
        corridor = (
            box_cell(
                [-0.10, -0.10, 0.80],
                [1.10, 0.10, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ),
        )
        config = ConvexTrajectoryConfig(
            max_speed_mps=10.0,
            max_acceleration_mps2=20.0,
            nominal_speed_mps=0.50,
            solver_time_s=2.0,
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 0.20, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            config,
            dt=0.02,
        )

        self.assertTrue(result.success, result.message)
        self.assertLess(result.segment_times[0], 2.0)
        self.assertAlmostEqual(result.segment_times[0], 1.75, places=4)
        self.assertIn("boundary-aware times selected", result.solver_message)

    def test_inherited_start_acceleration_recovers_inside_planning_limit(self):
        corridor = (
            box_cell(
                [-0.10, -0.20, 0.80],
                [1.10, 0.20, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
            ),
        )
        config = ConvexTrajectoryConfig(
            max_speed_mps=1.0,
            max_acceleration_mps2=1.5,
            validation_headroom_ratio=1.10,
            solver_time_s=2.0,
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [1.70, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            config,
            dt=0.02,
        )

        self.assertTrue(result.success, result.message)
        acceleration_cp = (
            42.0
            * np.diff(result.control_points[0], n=2, axis=0)
            / result.segment_times[0] ** 2
        )
        acceleration_components = np.abs(acceleration_cp)
        self.assertGreater(
            acceleration_components[0, 0],
            config.max_acceleration_mps2,
        )
        self.assertLessEqual(
            float(np.max(acceleration_components[1:])),
            config.max_acceleration_mps2 + 1e-8,
        )
        self.assertLessEqual(
            result.acceleration_control_bound_mps2,
            config.max_acceleration_mps2 + 1e-8,
        )
        self.assertGreaterEqual(result.peak_acceleration_mps2, 1.69)


    def test_diagonal_motion_uses_btraj_cartesian_derivative_bounds(self):
        corridor = (
            box_cell(
                [-0.20, -0.20, 0.80],
                [1.20, 1.20, 1.20],
                [0.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
            ),
        )
        config = ConvexTrajectoryConfig(
            max_speed_mps=1.0,
            max_acceleration_mps2=1.5,
            validation_headroom_ratio=1.10,
            nominal_speed_mps=1.0,
            solver_time_s=2.0,
        )
        result = solve_convex_corridor_trajectory(
            corridor,
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            config,
            dt=0.02,
        )

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.qp_solve_count, 2)
        self.assertIn("OSQP QP", result.solver_message)

        velocity_cp = (
            7.0
            * np.diff(result.control_points[0], axis=0)
            / result.segment_times[0]
        )
        acceleration_cp = (
            42.0
            * np.diff(result.control_points[0], n=2, axis=0)
            / result.segment_times[0] ** 2
        )

        # Gao/Btraj constrains each Cartesian hodograph component.  A diagonal
        # derivative may therefore have Euclidean magnitude above the per-axis
        # limit without violating the literature formulation.
        self.assertLessEqual(
            float(np.max(np.abs(velocity_cp))),
            config.max_speed_mps + 1e-8,
        )
        self.assertLessEqual(
            float(np.max(np.abs(acceleration_cp))),
            config.max_acceleration_mps2 + 1e-8,
        )
        self.assertGreater(
            float(np.max(np.linalg.norm(velocity_cp, axis=1))),
            config.max_speed_mps,
        )
        self.assertAlmostEqual(
            result.velocity_control_bound_mps,
            float(np.max(np.abs(velocity_cp))),
            places=8,
        )

    def test_target_acceleration_use_is_configurable(self):
        estimate = np.array([0.2, -0.1, 0.05])
        np.testing.assert_allclose(
            resolve_target_acceleration(
                ConvexTrajectoryConfig(use_target_acceleration=True), estimate
            ),
            estimate,
        )
        np.testing.assert_allclose(
            resolve_target_acceleration(
                ConvexTrajectoryConfig(use_target_acceleration=False), estimate
            ),
            np.zeros(3),
        )
        np.testing.assert_allclose(
            resolve_target_acceleration(
                ConvexTrajectoryConfig(use_target_acceleration=True), None
            ),
            np.zeros(3),
        )


if __name__ == "__main__":
    unittest.main()
