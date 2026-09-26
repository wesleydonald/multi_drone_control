import math
import unittest

import numpy as np

from tejen_mission.payload_geometry import PayloadGeometryProfile


def make_profile(
    pickup_point=(0.025, 0.0, 0.003),
    marker_to_face=(0.0, 0.0, -0.030),
) -> PayloadGeometryProfile:
    return PayloadGeometryProfile(
        name="test_payload",
        dimensions=np.array([0.100, 0.020, 0.006]),
        geometry_origin_from_pose=np.zeros(3),
        pickup_point=np.asarray(pickup_point, dtype=float),
        magnet_marker_to_contact_face=np.asarray(marker_to_face, dtype=float),
        pose_reference="centre",
        assume_level=True,
        collision_enabled=True,
        collision_margin=0.05,
    )


class PayloadPickupGeometryTests(unittest.TestCase):
    def test_zero_yaw_uses_configured_pickup_point(self) -> None:
        profile = make_profile()
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([1.0, 2.0, 0.10]),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.002,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        np.testing.assert_allclose(
            geometry.pickup_point_world,
            np.array([1.025, 2.0, 0.103]),
        )
        np.testing.assert_allclose(
            geometry.contact_face_target_world,
            np.array([1.025, 2.0, 0.101]),
        )
        np.testing.assert_allclose(
            geometry.magnet_marker_target_world,
            np.array([1.025, 2.0, 0.131]),
        )

    def test_ninety_degree_yaw_rotates_off_centre_pickup_point(self) -> None:
        profile = make_profile(pickup_point=(0.025, 0.010, 0.003))
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([1.0, 2.0, 0.10]),
            yaw=0.5 * math.pi,
            contact_gap=0.0,
            overtravel=0.0,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        np.testing.assert_allclose(
            geometry.pickup_point_world,
            np.array([0.990, 2.025, 0.103]),
            atol=1e-12,
        )

    def test_marker_to_face_vector_has_correct_sign(self) -> None:
        profile = make_profile(marker_to_face=(0.0, 0.0, -0.030))
        geometry = profile.pickup_target_geometry(
            pose_position=np.zeros(3),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.0,
            max_overtravel=0.005,
            support_surface_z=-1.0,
        )

        self.assertAlmostEqual(
            geometry.magnet_marker_target_world[2]
            - geometry.contact_face_target_world[2],
            0.030,
        )

    def test_contact_gap_raises_face_target(self) -> None:
        profile = make_profile()
        no_gap = profile.pickup_target_geometry(
            np.array([0.0, 0.0, 0.10]),
            0.0,
            contact_gap=0.0,
            overtravel=0.0,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )
        with_gap = profile.pickup_target_geometry(
            np.array([0.0, 0.0, 0.10]),
            0.0,
            contact_gap=0.004,
            overtravel=0.0,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        self.assertAlmostEqual(
            with_gap.contact_face_target_world[2]
            - no_gap.contact_face_target_world[2],
            0.004,
        )

    def test_overtravel_is_clamped_to_maximum(self) -> None:
        profile = make_profile()
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([0.0, 0.0, 0.10]),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.020,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        self.assertAlmostEqual(geometry.requested_overtravel, 0.020)
        self.assertAlmostEqual(geometry.used_overtravel, 0.005)

    def test_negative_overtravel_is_treated_as_zero(self) -> None:
        profile = make_profile()
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([0.0, 0.0, 0.10]),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=-0.010,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        self.assertAlmostEqual(geometry.requested_overtravel, 0.0)
        self.assertAlmostEqual(geometry.used_overtravel, 0.0)

    def test_support_surface_prevents_face_target_below_surface(self) -> None:
        profile = make_profile(pickup_point=(0.0, 0.0, 0.001))
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([0.0, 0.0, 0.0]),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.005,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        self.assertTrue(geometry.support_surface_clamped)
        self.assertAlmostEqual(geometry.contact_face_target_world[2], 0.0)
        self.assertAlmostEqual(geometry.magnet_marker_target_world[2], 0.030)

    def test_approach_target_is_vertical_offset_from_contact_marker(self) -> None:
        profile = make_profile()
        geometry = profile.pickup_target_geometry(
            pose_position=np.array([1.0, 2.0, 0.10]),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.002,
            max_overtravel=0.005,
            support_surface_z=0.0,
        )

        approach = geometry.approach_marker_target_world(0.20)
        np.testing.assert_allclose(
            approach,
            geometry.magnet_marker_target_world + np.array([0.0, 0.0, 0.20]),
        )
        np.testing.assert_allclose(
            approach[:2],
            geometry.magnet_marker_target_world[:2],
        )

    def test_approach_target_rejects_invalid_height(self) -> None:
        profile = make_profile()
        geometry = profile.pickup_target_geometry(
            pose_position=np.zeros(3),
            yaw=0.0,
            contact_gap=0.0,
            overtravel=0.0,
            max_overtravel=0.005,
            support_surface_z=-1.0,
        )
        with self.assertRaises(ValueError):
            geometry.approach_marker_target_world(-0.001)
        with self.assertRaises(ValueError):
            geometry.approach_marker_target_world(float("nan"))

    def test_rejects_nonfinite_or_invalid_inputs(self) -> None:
        profile = make_profile()
        with self.assertRaises(ValueError):
            profile.pickup_target_geometry(
                pose_position=np.zeros(3),
                yaw=float("nan"),
                contact_gap=0.0,
                overtravel=0.0,
                max_overtravel=0.005,
                support_surface_z=0.0,
            )
        with self.assertRaises(ValueError):
            profile.pickup_target_geometry(
                pose_position=np.zeros(3),
                yaw=0.0,
                contact_gap=-0.001,
                overtravel=0.0,
                max_overtravel=0.005,
                support_surface_z=0.0,
            )
        with self.assertRaises(ValueError):
            profile.pickup_target_geometry(
                pose_position=np.zeros(3),
                yaw=0.0,
                contact_gap=0.0,
                overtravel=0.0,
                max_overtravel=-0.001,
                support_surface_z=0.0,
            )


if __name__ == "__main__":
    unittest.main()
