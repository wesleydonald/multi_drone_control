import unittest

import numpy as np

from tejen_mission.mission_definitions import pickup_delivery_mission
from tejen_mission.mission_types import MissionPhase, ReferenceType
from tejen_mission.reference_generators import (
    TargetState,
    TransferConfig,
    VirtualTransferGenerator,
    rate_controlled_manoeuvre,
    smooth_rate_controlled_manoeuvre,
    stationary_regulation,
    target_relative_tracking,
)


class ReferenceGeneratorTests(unittest.TestCase):
    def test_mission_maps_every_phase_to_one_reference_type(self):
        mission = pickup_delivery_mission()
        self.assertEqual(
            mission.spec(MissionPhase.APPROACH_ABOVE_PICKUP).reference_type,
            ReferenceType.TRANSFER,
        )
        self.assertEqual(
            mission.spec(MissionPhase.SETTLE_ABOVE_DROP_POINT).reference_type,
            ReferenceType.TARGET_RELATIVE_TRACKING,
        )
        self.assertTrue(mission.spec(MissionPhase.ATTACH_READY).handoff_ready)

    def test_stationary_regulation_repeats_one_equilibrium(self):
        reference = stationary_regulation(
            np.array([1.0, 2.0, 3.0]),
            horizon_samples=10,
            min_reference_z=0.0,
        )
        np.testing.assert_allclose(reference.positions, np.tile([1.0, 2.0, 3.0], (10, 1)))
        np.testing.assert_allclose(reference.velocities, 0.0)

    def test_target_relative_tracking_keeps_moving_target_velocity(self):
        target = TargetState(
            np.array([1.0, 0.0, 0.0]),
            np.array([0.2, -0.1, 0.0]),
            np.zeros(3),
        )
        reference = target_relative_tracking(
            target,
            np.array([0.0, 0.0, 1.0]),
            dt=0.1,
            horizon_samples=5,
            lead_time=0.0,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        np.testing.assert_allclose(reference.positions[0], [1.0, 0.0, 1.0])
        np.testing.assert_allclose(reference.positions[4], [1.08, -0.04, 1.0])
        np.testing.assert_allclose(reference.velocities, np.tile([0.2, -0.1, 0.0], (5, 1)))

    def test_rate_controlled_reference_has_consistent_vertical_velocity(self):
        target = TargetState(np.zeros(3), np.zeros(3), np.zeros(3))
        reference, duration, complete = rate_controlled_manoeuvre(
            target,
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 0.5]),
            rate=0.1,
            elapsed=1.0,
            dt=0.1,
            horizon_samples=5,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        self.assertAlmostEqual(duration, 5.0)
        self.assertFalse(complete)
        self.assertAlmostEqual(reference.positions[0, 2], 0.9)
        self.assertAlmostEqual(reference.positions[1, 2], 0.89)
        self.assertAlmostEqual(reference.velocities[0, 2], -0.1)


    def test_smooth_rate_controlled_reference_starts_at_rest_with_consistent_acceleration(self):
        target = TargetState(np.zeros(3), np.zeros(3), np.zeros(3))
        reference, duration, complete = smooth_rate_controlled_manoeuvre(
            target,
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.5]),
            rate=0.15,
            acceleration_limit=0.50,
            elapsed=0.0,
            dt=1.0 / 30.0,
            horizon_samples=61,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        self.assertGreater(duration, 0.5 / 0.15)
        self.assertFalse(complete)
        np.testing.assert_allclose(reference.positions[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(reference.velocities[0], 0.0, atol=1e-12)
        np.testing.assert_allclose(reference.accelerations[0], 0.0, atol=1e-12)
        self.assertGreater(reference.velocities[1, 2], 0.0)
        self.assertGreater(reference.accelerations[1, 2], 0.0)
        self.assertLessEqual(np.max(np.linalg.norm(reference.velocities, axis=1)), 0.15 + 1e-12)
        self.assertLessEqual(np.max(np.linalg.norm(reference.accelerations, axis=1)), 0.50 + 1e-12)

    def test_smooth_rate_controlled_reference_finishes_at_rest(self):
        target = TargetState(np.zeros(3), np.zeros(3), np.zeros(3))
        _, duration, _ = smooth_rate_controlled_manoeuvre(
            target,
            np.zeros(3),
            np.array([0.0, 0.0, 0.5]),
            rate=0.15,
            acceleration_limit=0.50,
            elapsed=0.0,
            dt=1.0 / 30.0,
            horizon_samples=2,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        reference, _, complete = smooth_rate_controlled_manoeuvre(
            target,
            np.zeros(3),
            np.array([0.0, 0.0, 0.5]),
            rate=0.15,
            acceleration_limit=0.50,
            elapsed=duration,
            dt=1.0 / 30.0,
            horizon_samples=3,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        self.assertTrue(complete)
        np.testing.assert_allclose(reference.positions, np.tile([0.0, 0.0, 0.5], (3, 1)), atol=1e-12)
        np.testing.assert_allclose(reference.velocities, 0.0, atol=1e-12)
        np.testing.assert_allclose(reference.accelerations, 0.0, atol=1e-12)

    def test_virtual_transfer_does_not_restart_at_measured_pose(self):
        config = TransferConfig(
            dt=0.1,
            horizon_samples=20,
            nominal_speed=0.5,
            duration_scale=1.5,
            min_duration=1.0,
            max_duration=6.0,
            tracking_error_soft=0.1,
            tracking_error_hard=0.5,
            minimum_progress_scale=0.1,
            min_reference_z=0.0,
            max_reference_speed=2.0,
        )
        generator = VirtualTransferGenerator(config)
        measured = np.zeros(3)
        target = TargetState(np.array([1.0, 0.0, 0.0]), np.zeros(3), np.zeros(3))

        first = generator.generate(
            phase_key="transfer",
            measured_position=measured,
            measured_velocity=np.zeros(3),
            target=target,
        )
        second = generator.generate(
            phase_key="transfer",
            measured_position=measured,
            measured_velocity=np.zeros(3),
            target=target,
        )

        self.assertAlmostEqual(first.positions[0, 0], 0.0)
        self.assertGreater(second.positions[0, 0], 0.0)


if __name__ == "__main__":
    unittest.main()
