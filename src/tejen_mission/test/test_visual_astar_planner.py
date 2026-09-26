import unittest
from unittest.mock import patch

import numpy as np

from tejen_mission.online_safety_planner import StaticSphereObstacle
from tejen_mission.payload_geometry import PayloadGeometryProfile
from tejen_mission.visual_astar_planner import (
    VisualAStarConfig,
    VisualAStarRequest,
    _build_whole_body_occupancy_grid,
    compute_visual_astar,
    validate_visual_astar_candidate,
)
from tejen_mission.voxel_astar import SphereObstacle, VoxelAStar3D


class VisualAStarPlannerTests(unittest.TestCase):
    def make_request(self, positions, obstacles=(), phase_allowed=True, **kwargs):
        positions = np.asarray(positions, dtype=float)
        defaults = dict(
            request_id=1,
            phase_name="APPROACH_ABOVE_PICKUP",
            phase_allowed=phase_allowed,
            nominal_positions=positions,
            planning_start_position=positions[0].copy(),
            planning_start_velocity=np.zeros(3, dtype=float),
            target_position=positions[-1].copy(),
            target_velocity_mean=np.zeros(3, dtype=float),
            target_velocity_variance=np.zeros(3, dtype=float),
            dt=0.05,
            obstacles=tuple(obstacles),
            drone_radius=0.15,
            drone_safety_margin=0.05,
            magnet_offset_from_quad=np.array([0.0, 0.0, -0.50]),
            magnet_radius=0.05,
            magnet_safety_margin=0.02,
            cable_radius=0.005,
            cable_safety_margin=0.02,
        )
        defaults.update(kwargs)
        return VisualAStarRequest(**defaults)

    def test_voxel_astar_baseline_is_26_connected_ordinary_astar(self):
        planner = VoxelAStar3D()
        self.assertEqual(len(planner._neighbours), 26)
        self.assertEqual(planner.heuristic_weight, 1.0)

    def test_diagonal_corner_cutting_is_rejected(self):
        planner = VoxelAStar3D(
            resolution=1.0,
            conservative_voxel_inflation=False,
        )
        planner._configure_problem(
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([1.0, 1.0, 1.0]),
            spheres=(
                SphereObstacle(np.array([1.0, 0.0, 0.0]), 0.10),
                SphereObstacle(np.array([0.0, 1.0, 0.0]), 0.10),
            ),
            boxes=(),
        )
        self.assertFalse(planner.index_is_occupied((0, 0, 0)))
        self.assertFalse(planner.index_is_occupied((1, 1, 0)))
        self.assertFalse(planner._transition_is_free((0, 0, 0), (1, 1, 0)))

    def test_clear_direct_path_does_not_run_astar(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 31), np.zeros(31), np.ones(31))
        )
        result = compute_visual_astar(
            VisualAStarConfig(resolution=0.10),
            self.make_request(positions),
        )
        self.assertEqual(result.status, "direct_clear")
        self.assertTrue(result.success)
        self.assertFalse(result.nominal_blocked)
        self.assertEqual(result.expanded_nodes, 0)
        np.testing.assert_allclose(result.candidate_positions[0], positions[0])
        np.testing.assert_allclose(result.candidate_positions[-1], positions[-1])

    def test_non_transfer_phase_is_not_searched(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 31), np.zeros(31), np.ones(31))
        )
        result = compute_visual_astar(
            VisualAStarConfig(resolution=0.10),
            self.make_request(positions, phase_allowed=False),
        )
        self.assertEqual(result.status, "phase_not_allowed")
        self.assertEqual(result.expanded_nodes, 0)

    def test_blocked_direct_path_generates_astar_route_to_target(self):
        positions = np.column_stack(
            (np.linspace(0.0, 2.0, 61), np.zeros(61), np.ones(61))
        )
        obstacle = StaticSphereObstacle(
            "blocking_sphere", np.array([1.0, 0.0, 1.0]), 0.20
        )
        result = compute_visual_astar(
            VisualAStarConfig(
                resolution=0.10,
                max_planning_time_s=1.0,
                local_margin_xy_m=0.80,
                local_margin_z_m=0.50,
            ),
            self.make_request(positions, obstacles=(obstacle,)),
        )
        self.assertTrue(result.nominal_blocked)
        self.assertTrue(result.success, result.message)
        self.assertGreater(result.expanded_nodes, 0)
        self.assertIsNone(result.rejoin_index)
        self.assertGreaterEqual(result.raw_path.shape[0], 2)
        self.assertGreaterEqual(result.simplified_path.shape[0], 2)
        np.testing.assert_allclose(result.start_requested, positions[0])
        np.testing.assert_allclose(result.goal_requested, positions[-1])
        np.testing.assert_allclose(result.candidate_positions[0], positions[0])
        np.testing.assert_allclose(result.candidate_positions[-1], positions[-1])
        self.assertTrue(result.candidate_whole_body_safe)
        self.assertLessEqual(
            result.simplified_path.shape[0], result.raw_path.shape[0]
        )

        # Freeze the path/corridor contract used by the later smoother.
        segment_lengths = np.linalg.norm(
            np.diff(result.simplified_path, axis=0),
            axis=1,
        )
        self.assertTrue(np.all(segment_lengths > 1e-8))
        self.assertEqual(
            len(result.convex_corridor),
            result.simplified_path.shape[0] - 1,
        )
        self.assertTrue(result.corridor_valid, result.corridor_error)
        self.assertEqual(
            result.corridor_segment_containment_margins_m.shape,
            (len(result.convex_corridor),),
        )
        self.assertTrue(
            np.all(result.corridor_segment_containment_margins_m >= -1e-8)
        )
        self.assertTrue(
            np.all(result.corridor_overlap_margins_m >= -1e-8)
        )
        self.assertEqual(
            result.corridor_voxel_exclusion_margins_m.shape,
            (len(result.convex_corridor),),
        )

    def test_planner_uses_measured_start_not_nominal_start(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 31), np.zeros(31), np.ones(31))
        )
        measured_start = np.array([0.30, 0.20, 1.0])

        result = compute_visual_astar(
            VisualAStarConfig(resolution=0.10),
            self.make_request(
                positions,
                planning_start_position=measured_start,
            ),
        )

        self.assertTrue(result.success, result.message)
        np.testing.assert_allclose(result.start_requested, measured_start)
        np.testing.assert_allclose(result.candidate_positions[0], measured_start)
        self.assertGreater(
            np.linalg.norm(result.candidate_positions[0] - positions[0]),
            0.1,
        )

    def test_filtered_target_velocity_shifts_predicted_goal(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 31), np.zeros(31), np.ones(31))
        )
        start = np.array([0.0, 0.0, 1.0])
        target = np.array([1.0, 0.0, 1.0])
        target_velocity = np.array([0.0, 0.10, 0.0])

        result = compute_visual_astar(
            VisualAStarConfig(
                resolution=0.10,
                candidate_speed_mps=0.50,
                target_prediction_max_s=4.0,
            ),
            self.make_request(
                positions,
                planning_start_position=start,
                target_position=target,
                target_velocity_mean=target_velocity,
            ),
        )

        # Range = 1 m and assumed drone speed = 0.5 m/s, so ETA = 2 s.
        # At +0.10 m/s in Y the predicted target moves +0.20 m.
        expected_goal = np.array([1.0, 0.20, 1.0])
        self.assertTrue(result.success, result.message)
        np.testing.assert_allclose(result.goal_requested, expected_goal, atol=1e-9)
        np.testing.assert_allclose(
            result.candidate_positions[-1], expected_goal, atol=1e-9
        )

    def test_whole_body_grid_blocks_magnet_configuration_when_quad_is_clear(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 21), np.zeros(21), np.ones(21))
        )
        obstacle = StaticSphereObstacle(
            "magnet_blocker", np.array([0.5, 0.0, 0.5]), 0.05
        )
        request = self.make_request(positions, obstacles=(obstacle,))
        config = VisualAStarConfig(
            resolution=0.10,
            max_cable_swing_angle_rad=0.0,
        )

        bounds_min = np.array([-0.2, -0.5, 0.0])
        bounds_max = np.array([1.2, 0.5, 1.5])
        occupancy = _build_whole_body_occupancy_grid(
            config,
            request,
            bounds_min,
            bounds_max,
        )

        quad_position = np.array([0.5, 0.0, 1.0])
        quad_clearance = (
            np.linalg.norm(quad_position - obstacle.centre)
            - obstacle.radius
            - request.drone_radius
            - request.drone_safety_margin
        )

        self.assertGreater(quad_clearance, 0.0)
        self.assertTrue(occupancy.point_is_occupied(quad_position))

    def test_corridor_validity_does_not_require_full_voxel_cube_exclusion(self):
        positions = np.column_stack(
            (np.linspace(0.0, 2.0, 61), np.zeros(61), np.ones(61))
        )
        obstacle = StaticSphereObstacle(
            "blocking_sphere", np.array([1.0, 0.0, 1.0]), 0.20
        )

        # The DecompUtil-style SFC is constructed against obstacle points.
        # Full occupied-voxel cube exclusion is deliberately only a stricter
        # diagnostic, so force that diagnostic negative and ensure the
        # literature-aligned corridor-validity contract still passes.
        with patch(
            "tejen_mission.visual_astar_planner."
            "corridor_voxel_exclusion_margins",
            return_value=np.array([-0.05], dtype=float),
        ):
            result = compute_visual_astar(
                VisualAStarConfig(
                    resolution=0.10,
                    max_planning_time_s=1.0,
                    local_margin_xy_m=0.80,
                    local_margin_z_m=0.50,
                ),
                self.make_request(positions, obstacles=(obstacle,)),
            )

        self.assertTrue(result.success, result.message)
        self.assertGreater(len(result.convex_corridor), 0)
        self.assertTrue(result.corridor_valid, result.corridor_error)
        self.assertTrue(
            np.any(result.corridor_voxel_exclusion_margins_m < 0.0)
        )

    def test_astar_failure_returns_no_candidate(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 31), np.zeros(31), np.ones(31))
        )
        obstacle = StaticSphereObstacle(
            "goal_blocker", np.array([1.0, 0.0, 1.0]), 0.30
        )

        result = compute_visual_astar(
            VisualAStarConfig(
                resolution=0.10,
                nearest_free_radius_m=0.0,
                max_planning_time_s=1.0,
            ),
            self.make_request(positions, obstacles=(obstacle,)),
        )

        self.assertFalse(result.success)
        self.assertEqual(result.status, "search_failed")
        self.assertEqual(result.candidate_positions.shape, (0, 3))
        self.assertIn("goal", result.message.lower())

    def test_payload_can_invalidate_quad_safe_candidate(self):
        positions = np.column_stack(
            (np.linspace(0.0, 1.0, 21), np.zeros(21), np.ones(21))
        )
        obstacle = StaticSphereObstacle(
            "payload_only", np.array([0.5, 0.30, 0.50]), 0.05
        )
        profile = PayloadGeometryProfile(
            name="test_payload",
            dimensions=np.array([0.20, 0.20, 0.10]),
            geometry_origin_from_pose=np.zeros(3),
            pickup_point=np.zeros(3),
            magnet_marker_to_contact_face=np.array([0.0, 0.0, -0.03]),
            collision_margin=0.02,
        )
        request = self.make_request(
            positions,
            obstacles=(obstacle,),
            object_attached=True,
            payload_profile=profile,
            payload_geometry_available=True,
            payload_offset_from_magnet=np.array([0.0, 0.30, 0.0]),
            payload_yaw=0.0,
        )
        validation = validate_visual_astar_candidate(positions, request)
        self.assertTrue(validation["quad_clearance"] > 0.0)
        self.assertTrue(validation["magnet_clearance"] > 0.0)
        self.assertTrue(validation["cable_clearance"] > 0.0)
        self.assertLess(validation["payload_clearance"], 0.0)
        self.assertFalse(validation["whole_body_safe"])
        self.assertEqual(validation["critical_component"], "PAYLOAD")


if __name__ == "__main__":
    unittest.main()
