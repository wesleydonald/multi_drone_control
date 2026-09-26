import unittest

import numpy as np

from tejen_mission.online_safety_planner import (
    OnlineSafetyPlanner,
    SafetyAction,
    SafetyPlannerRequest,
    StaticSphereObstacle,
    check_cable_against_static_spheres,
    check_positions_against_static_spheres,
    check_yaw_oriented_boxes_against_static_spheres,
    check_reference_against_static_spheres,
    closest_point_on_segment,
    offset_component_positions,
)
from tejen_mission.payload_geometry import PayloadGeometryProfile
from tejen_mission.reference_generators import TrajectoryReference


def make_reference(points) -> TrajectoryReference:
    positions = np.asarray(points, dtype=float)
    return TrajectoryReference(
        positions=positions,
        velocities=np.zeros_like(positions),
        accelerations=np.zeros_like(positions),
    )


class OnlineSafetyPlannerTests(unittest.TestCase):
    def test_pass_through_returns_exact_nominal_reference(self) -> None:
        nominal = make_reference(
            [
                [0.0, 0.0, 1.0],
                [0.1, 0.0, 1.0],
                [0.2, 0.0, 1.0],
            ]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle(
                    obstacle_id="far",
                    centre=np.array([5.0, 0.0, 1.0]),
                    radius=0.25,
                )
            ],
            drone_radius=0.20,
            safety_margin=0.10,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.zeros(3),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, 10.0]),
                phase_name="TEST_PHASE",
                dt=0.1,
            )
        )

        self.assertIs(result.reference, nominal)
        self.assertIs(planner.committed_reference, nominal)
        self.assertEqual(result.action, SafetyAction.PASS_THROUGH)
        self.assertFalse(result.intervention_active)
        self.assertTrue(result.nominal_safe)
        self.assertGreater(result.minimum_clearance, 0.0)
        self.assertIn("collision_checking=diagnostics_only", result.status)

    def test_intersection_is_reported_but_reference_is_unchanged(self) -> None:
        nominal = make_reference(
            [
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("blocked", np.array([1.0, 0.0, 0.0]), 0.25)
            ],
            drone_radius=0.20,
            safety_margin=0.05,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.zeros(3),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, 10.0]),
                phase_name="TRANSIT",
                dt=0.2,
            )
        )

        self.assertIs(result.reference, nominal)
        self.assertEqual(result.action, SafetyAction.PASS_THROUGH)
        self.assertFalse(result.nominal_safe)
        self.assertAlmostEqual(result.minimum_clearance, -0.50)
        self.assertEqual(result.closest_obstacle_id, "blocked")
        self.assertEqual(result.closest_segment_index, 0)
        self.assertAlmostEqual(result.closest_time_s, 0.1)
        np.testing.assert_allclose(result.closest_point, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(
            result.closest_obstacle_centre, [1.0, 0.0, 0.0]
        )

    def test_tangent_segment_has_zero_clearance(self) -> None:
        nominal = make_reference(
            [
                [-1.0, 0.5, 0.0],
                [1.0, 0.5, 0.0],
            ]
        )
        result = check_reference_against_static_spheres(
            nominal,
            obstacles=[StaticSphereObstacle("tangent", np.zeros(3), 0.25)],
            drone_radius=0.20,
            safety_margin=0.05,
            dt=0.1,
        )

        self.assertTrue(result.nominal_safe)
        self.assertAlmostEqual(result.minimum_clearance, 0.0, places=12)
        self.assertAlmostEqual(result.closest_time_s, 0.05)
        np.testing.assert_allclose(result.closest_point, [0.0, 0.5, 0.0])
        np.testing.assert_allclose(result.closest_obstacle_centre, [0.0, 0.0, 0.0])

    def test_closest_point_is_clamped_to_segment_endpoint(self) -> None:
        closest, fraction = closest_point_on_segment(
            point=np.array([2.0, 1.0, 0.0]),
            segment_start=np.array([0.0, 0.0, 0.0]),
            segment_end=np.array([1.0, 0.0, 0.0]),
        )

        np.testing.assert_array_equal(closest, [1.0, 0.0, 0.0])
        self.assertEqual(fraction, 1.0)

    def test_worst_of_multiple_obstacles_is_reported(self) -> None:
        nominal = make_reference(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ]
        )
        result = check_reference_against_static_spheres(
            nominal,
            obstacles=[
                StaticSphereObstacle("clear", np.array([0.5, 1.0, 0.0]), 0.10),
                StaticSphereObstacle("worst", np.array([1.5, 0.1, 0.0]), 0.20),
            ],
            drone_radius=0.20,
            safety_margin=0.05,
            dt=0.2,
        )

        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.closest_obstacle_id, "worst")
        self.assertEqual(result.closest_segment_index, 1)
        self.assertAlmostEqual(result.minimum_clearance, -0.35)
        self.assertAlmostEqual(result.closest_time_s, 0.3)
        np.testing.assert_allclose(result.closest_point, [1.5, 0.0, 0.0])
        np.testing.assert_allclose(
            result.closest_obstacle_centre, [1.5, 0.1, 0.0]
        )

    def test_request_copies_measured_state(self) -> None:
        nominal = make_reference([[0.0, 0.0, 0.0]])
        position = np.array([1.0, 2.0, 3.0])
        velocity = np.array([0.1, 0.2, 0.3])
        magnet_offset = np.array([0.0, 0.0, -0.5])

        request = SafetyPlannerRequest(
            nominal_reference=nominal,
            measured_position=position,
            measured_velocity=velocity,
            magnet_offset_from_quad=magnet_offset,
            phase_name="TEST_PHASE",
            dt=0.1,
        )
        position[:] = 99.0
        velocity[:] = 99.0
        magnet_offset[:] = 99.0

        np.testing.assert_array_equal(request.measured_position, [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(request.measured_velocity, [0.1, 0.2, 0.3])
        np.testing.assert_array_equal(
            request.magnet_offset_from_quad, [0.0, 0.0, -0.5]
        )

    def test_quad_safe_magnet_collides(self) -> None:
        nominal = make_reference(
            [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("low_obstacle", np.array([1.0, 0.0, 0.0]), 0.20)
            ],
            drone_radius=0.10,
            safety_margin=0.05,
            magnet_radius=0.10,
            magnet_safety_margin=0.05,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 1.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -1.0]),
                phase_name="TRANSIT",
                dt=0.2,
            )
        )

        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.critical_component, "MAGNET")
        self.assertGreater(result.quad_minimum_clearance, 0.0)
        self.assertLess(result.magnet_minimum_clearance, 0.0)
        np.testing.assert_allclose(
            result.predicted_magnet_positions,
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        )
        self.assertIs(result.reference, nominal)

    def test_quad_collides_magnet_is_clear(self) -> None:
        nominal = make_reference(
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("quad_obstacle", np.array([1.0, 0.0, 0.0]), 0.20)
            ],
            drone_radius=0.10,
            safety_margin=0.05,
            magnet_radius=0.10,
            magnet_safety_margin=0.05,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.zeros(3),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, 1.0]),
                phase_name="TRANSIT",
                dt=0.2,
            )
        )

        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.critical_component, "QUAD")
        self.assertLess(result.quad_minimum_clearance, 0.0)
        self.assertGreater(result.magnet_minimum_clearance, 0.0)

    def test_larger_magnet_radius_can_make_magnet_worst(self) -> None:
        nominal = make_reference([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("shared", np.array([1.0, 0.0, 0.0]), 0.10)
            ],
            drone_radius=0.05,
            safety_margin=0.0,
            magnet_radius=0.20,
            magnet_safety_margin=0.0,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.zeros(3),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.zeros(3),
                phase_name="TRANSIT",
                dt=0.2,
            )
        )

        self.assertEqual(result.critical_component, "MAGNET")
        self.assertLess(
            result.magnet_minimum_clearance, result.quad_minimum_clearance
        )

    def test_offset_component_positions_preserves_measured_offset(self) -> None:
        nominal = make_reference(
            [[0.0, 0.0, 1.0], [0.5, 0.2, 1.2], [1.0, 0.4, 1.4]]
        )
        offset = np.array([0.1, -0.2, -0.8])

        magnet_positions = offset_component_positions(nominal, offset)

        np.testing.assert_allclose(
            magnet_positions - nominal.positions,
            np.tile(offset, (3, 1)),
        )

    def test_component_checker_reports_component_name(self) -> None:
        result = check_positions_against_static_spheres(
            positions=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
            obstacles=[
                StaticSphereObstacle("obstacle", np.array([0.5, 0.0, 0.0]), 0.10)
            ],
            component_name="magnet",
            component_radius=0.05,
            safety_margin=0.0,
            dt=0.1,
        )

        self.assertEqual(result.component_name, "MAGNET")
        self.assertFalse(result.nominal_safe)

    def test_quad_and_magnet_safe_but_cable_collides(self) -> None:
        nominal = make_reference(
            [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle(
                    "mid_cable", np.array([1.0, 0.0, 0.5]), 0.10
                )
            ],
            drone_radius=0.05,
            safety_margin=0.02,
            magnet_radius=0.05,
            magnet_safety_margin=0.02,
            cable_radius=0.01,
            cable_safety_margin=0.02,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 1.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -1.0]),
                phase_name="TRANSIT",
                dt=0.1,
            )
        )

        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.critical_component, "CABLE")
        self.assertGreater(result.quad_minimum_clearance, 0.0)
        self.assertGreater(result.magnet_minimum_clearance, 0.0)
        self.assertAlmostEqual(result.cable_minimum_clearance, -0.13)
        self.assertEqual(result.cable_closest_obstacle_id, "mid_cable")
        self.assertEqual(result.cable_closest_sample_index, 1)
        self.assertAlmostEqual(result.cable_closest_time_s, 0.1)
        np.testing.assert_allclose(result.cable_closest_point, [1.0, 0.0, 0.5])
        np.testing.assert_allclose(
            result.cable_closest_quad_position, [1.0, 0.0, 1.0]
        )
        np.testing.assert_allclose(
            result.cable_closest_magnet_position, [1.0, 0.0, 0.0]
        )
        self.assertIs(result.reference, nominal)

    def test_cable_capsule_tangent_has_zero_clearance(self) -> None:
        result = check_cable_against_static_spheres(
            quad_positions=np.array([[0.0, 0.0, 1.0]]),
            magnet_positions=np.array([[0.0, 0.0, 0.0]]),
            obstacles=[
                StaticSphereObstacle(
                    "tangent", np.array([0.30, 0.0, 0.5]), 0.20
                )
            ],
            cable_radius=0.05,
            safety_margin=0.05,
            dt=0.1,
        )

        self.assertTrue(result.nominal_safe)
        self.assertAlmostEqual(result.minimum_clearance, 0.0, places=12)
        self.assertEqual(result.closest_segment_index, 0)
        self.assertAlmostEqual(result.closest_time_s, 0.0)
        np.testing.assert_allclose(result.closest_point, [0.0, 0.0, 0.5])
        np.testing.assert_allclose(result.closest_component_start, [0.0, 0.0, 1.0])
        np.testing.assert_allclose(result.closest_component_end, [0.0, 0.0, 0.0])

    def test_cable_checker_selects_worst_horizon_sample(self) -> None:
        result = check_cable_against_static_spheres(
            quad_positions=np.array(
                [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
            ),
            magnet_positions=np.array(
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
            ),
            obstacles=[
                StaticSphereObstacle("near_first", np.array([0.0, 0.4, 0.5]), 0.10),
                StaticSphereObstacle("worst", np.array([2.0, 0.1, 0.5]), 0.10),
            ],
            cable_radius=0.02,
            safety_margin=0.03,
            dt=0.2,
        )

        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.closest_obstacle_id, "worst")
        self.assertEqual(result.closest_segment_index, 2)
        self.assertAlmostEqual(result.closest_time_s, 0.4)
        self.assertAlmostEqual(result.minimum_clearance, -0.05)
        np.testing.assert_allclose(result.closest_point, [2.0, 0.0, 0.5])

    def test_cable_checker_validates_matching_shapes(self) -> None:
        with self.assertRaises(ValueError):
            check_cable_against_static_spheres(
                quad_positions=np.zeros((2, 3)),
                magnet_positions=np.zeros((3, 3)),
                obstacles=[],
                cable_radius=0.01,
                safety_margin=0.02,
                dt=0.1,
            )



    @staticmethod
    def make_payload_profile(
        dimensions=(0.40, 0.20, 0.10),
        margin=0.0,
    ) -> PayloadGeometryProfile:
        return PayloadGeometryProfile(
            name="test_payload",
            dimensions=np.asarray(dimensions, dtype=float),
            geometry_origin_from_pose=np.zeros(3),
            pickup_point=np.array([0.0, 0.0, 0.05]),
            magnet_marker_to_contact_face=np.array([0.0, 0.0, -0.03]),
            collision_margin=margin,
        )

    def test_payload_profile_converts_pose_origin_to_box_centre(self) -> None:
        profile = PayloadGeometryProfile(
            name="offset_payload",
            dimensions=np.array([0.10, 0.02, 0.006]),
            geometry_origin_from_pose=np.array([0.02, 0.0, 0.003]),
            pickup_point=np.array([0.025, 0.0, 0.003]),
            magnet_marker_to_contact_face=np.array([0.0, 0.0, -0.03]),
            collision_margin=0.01,
        )

        centre = profile.geometry_centre_from_pose(
            np.array([1.0, 2.0, 3.0]),
            np.pi / 2.0,
        )

        np.testing.assert_allclose(centre, [1.0, 2.02, 3.003], atol=1e-12)
        np.testing.assert_allclose(profile.inflated_dimensions, [0.12, 0.04, 0.026])

    def test_payload_box_tangent_has_zero_clearance(self) -> None:
        result = check_yaw_oriented_boxes_against_static_spheres(
            box_centres=np.array([[0.0, 0.0, 0.0]]),
            box_yaws=np.array([0.0]),
            dimensions=np.array([1.0, 0.4, 0.2]),
            obstacles=[
                StaticSphereObstacle("tangent", np.array([0.0, 0.5, 0.0]), 0.20)
            ],
            safety_margin=0.10,
            dt=0.1,
        )

        self.assertTrue(result.nominal_safe)
        self.assertAlmostEqual(result.minimum_clearance, 0.0, places=12)
        self.assertEqual(result.component_name, "PAYLOAD")
        np.testing.assert_allclose(result.closest_point, [0.0, 0.3, 0.0])

    def test_payload_box_yaw_changes_clearance_for_elongated_payload(self) -> None:
        obstacle = StaticSphereObstacle(
            "side_obstacle",
            np.array([0.0, 0.80, 0.0]),
            0.10,
        )
        clear = check_yaw_oriented_boxes_against_static_spheres(
            box_centres=np.array([[0.0, 0.0, 0.0]]),
            box_yaws=np.array([0.0]),
            dimensions=np.array([2.0, 0.20, 0.20]),
            obstacles=[obstacle],
            safety_margin=0.0,
            dt=0.1,
        )
        colliding = check_yaw_oriented_boxes_against_static_spheres(
            box_centres=np.array([[0.0, 0.0, 0.0]]),
            box_yaws=np.array([np.pi / 2.0]),
            dimensions=np.array([2.0, 0.20, 0.20]),
            obstacles=[obstacle],
            safety_margin=0.0,
            dt=0.1,
        )

        self.assertTrue(clear.nominal_safe)
        self.assertGreater(clear.minimum_clearance, 0.0)
        self.assertFalse(colliding.nominal_safe)
        self.assertLess(colliding.minimum_clearance, 0.0)

    def test_quad_magnet_and_cable_clear_but_payload_collides(self) -> None:
        nominal = make_reference(
            [[0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [2.0, 0.0, 2.0]]
        )
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle(
                    "payload_only",
                    np.array([1.0, 0.0, 0.50]),
                    0.10,
                )
            ],
            drone_radius=0.05,
            safety_margin=0.01,
            magnet_radius=0.05,
            magnet_safety_margin=0.01,
            cable_radius=0.01,
            cable_safety_margin=0.01,
            payload_profile=self.make_payload_profile(),
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 2.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -1.0]),
                phase_name="LOADED_TRANSIT",
                dt=0.1,
                object_attached=True,
                payload_geometry_available=True,
                payload_offset_from_magnet=np.array([0.0, 0.0, -0.5]),
                payload_yaw=0.0,
                payload_pose_age_s=0.0,
            )
        )

        self.assertIs(result.reference, nominal)
        self.assertEqual(result.action, SafetyAction.PASS_THROUGH)
        self.assertFalse(result.intervention_active)
        self.assertFalse(result.nominal_safe)
        self.assertEqual(result.critical_component, "PAYLOAD")
        self.assertTrue(result.payload_check_active)
        self.assertTrue(result.payload_geometry_available)
        self.assertGreater(result.quad_minimum_clearance, 0.0)
        self.assertGreater(result.magnet_minimum_clearance, 0.0)
        self.assertGreater(result.cable_minimum_clearance, 0.0)
        self.assertLess(result.payload_minimum_clearance, 0.0)
        self.assertEqual(result.payload_closest_obstacle_id, "payload_only")
        self.assertEqual(result.payload_closest_sample_index, 1)
        np.testing.assert_allclose(
            result.predicted_payload_positions,
            [[0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [2.0, 0.0, 0.5]],
        )

    def test_attached_payload_clearance_is_reported_when_clear(self) -> None:
        nominal = make_reference([[0.0, 0.0, 2.0], [1.0, 0.0, 2.0]])
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("clear", np.array([0.5, 1.0, 0.5]), 0.10)
            ],
            drone_radius=0.05,
            safety_margin=0.01,
            magnet_radius=0.05,
            magnet_safety_margin=0.01,
            cable_radius=0.01,
            cable_safety_margin=0.01,
            payload_profile=self.make_payload_profile(),
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 2.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -1.0]),
                phase_name="LOADED_TRANSIT",
                dt=0.1,
                object_attached=True,
                payload_geometry_available=True,
                payload_offset_from_magnet=np.array([0.0, 0.0, -0.5]),
                payload_yaw=0.0,
                payload_pose_age_s=0.0,
            )
        )

        self.assertTrue(result.nominal_safe)
        self.assertTrue(result.payload_check_active)
        self.assertTrue(result.payload_geometry_available)
        self.assertGreater(result.payload_minimum_clearance, 0.0)
        self.assertIs(result.reference, nominal)

    def test_payload_is_inactive_before_attachment(self) -> None:
        nominal = make_reference([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
        planner = OnlineSafetyPlanner(
            obstacles=[
                StaticSphereObstacle("near_payload", np.array([0.5, 0.0, 0.0]), 0.1)
            ],
            drone_radius=0.05,
            safety_margin=0.01,
            magnet_radius=0.05,
            magnet_safety_margin=0.01,
            cable_radius=0.01,
            cable_safety_margin=0.01,
            payload_profile=self.make_payload_profile(),
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 1.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -0.4]),
                phase_name="APPROACH",
                dt=0.1,
                object_attached=False,
            )
        )

        self.assertFalse(result.payload_check_active)
        self.assertFalse(result.payload_geometry_available)
        self.assertIsNone(result.payload_minimum_clearance)
        self.assertIsNone(result.predicted_payload_positions)
        self.assertIsNotNone(result.nominal_safe)

    def test_attached_payload_with_unavailable_geometry_is_unknown(self) -> None:
        nominal = make_reference([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
        planner = OnlineSafetyPlanner(
            obstacles=[],
            payload_profile=self.make_payload_profile(),
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 1.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -0.5]),
                phase_name="LOADED_TRANSIT",
                dt=0.1,
                object_attached=True,
                payload_geometry_available=False,
                payload_pose_age_s=1.0,
            )
        )

        self.assertIs(result.reference, nominal)
        self.assertEqual(result.action, SafetyAction.PASS_THROUGH)
        self.assertTrue(result.payload_check_active)
        self.assertFalse(result.payload_geometry_available)
        self.assertIsNone(result.nominal_safe)
        self.assertIn("whole_body_safety=unknown", result.status)

    def test_stale_payload_geometry_is_rejected_by_timeout(self) -> None:
        nominal = make_reference([[0.0, 0.0, 1.0]])
        planner = OnlineSafetyPlanner(
            obstacles=[],
            payload_profile=self.make_payload_profile(),
            payload_pose_timeout_s=0.5,
        )

        result = planner.plan(
            SafetyPlannerRequest(
                nominal_reference=nominal,
                measured_position=np.array([0.0, 0.0, 1.0]),
                measured_velocity=np.zeros(3),
                magnet_offset_from_quad=np.array([0.0, 0.0, -0.5]),
                phase_name="LOADED_TRANSIT",
                dt=0.1,
                object_attached=True,
                payload_geometry_available=True,
                payload_offset_from_magnet=np.array([0.0, 0.0, -0.1]),
                payload_yaw=0.0,
                payload_pose_age_s=0.75,
            )
        )

        self.assertTrue(result.payload_check_active)
        self.assertFalse(result.payload_geometry_available)
        self.assertIsNone(result.nominal_safe)
        self.assertIsNone(result.predicted_payload_positions)




if __name__ == "__main__":
    unittest.main()
