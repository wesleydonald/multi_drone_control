import unittest

import numpy as np

from tejen_mission.c1f2_handoff import (
    advance_reference_window,
    handoff_allowed,
    loaded_lift_override_allowed,
    loaded_lift_ready_for_cpp,
    post_grant_reference_is_fresh,
    stationary_grant_state,
    stationary_hold_state,
    stage0_errors,
    target_relative_bridge_feasibility,
    target_relative_bridge_initial_state,
    target_relative_bridge_reference,
)
from tejen_mission.reference_generators import TargetState, TrajectoryReference


def reference(p, v, a, n=61):
    return TrajectoryReference(
        np.repeat(np.asarray(p, dtype=float).reshape(1, 3), n, axis=0),
        np.repeat(np.asarray(v, dtype=float).reshape(1, 3), n, axis=0),
        np.repeat(np.asarray(a, dtype=float).reshape(1, 3), n, axis=0),
    )


def ramp_reference(n=61, dt=1.0 / 30.0):
    t = np.arange(n, dtype=float) * dt
    positions = np.column_stack((t, 2.0 * t, np.full(n, 2.5)))
    velocities = np.column_stack((np.ones(n), 2.0 * np.ones(n), np.zeros(n)))
    accelerations = np.zeros((n, 3), dtype=float)
    # The committed C++ contract ends in a stopped hold.
    velocities[-1] = 0.0
    return TrajectoryReference(positions, velocities, accelerations)


class C1F2HandoffTests(unittest.TestCase):
    def test_post_grant_reference_rejects_delayed_pregrant_packet(self):
        self.assertFalse(
            post_grant_reference_is_fresh(
                reference_source_stamp_s=100.0,
                grant_ros_time_s=100.1,
                reference_receive_time=50.3,
                ack_receive_time=50.2,
            )
        )

    def test_post_grant_reference_requires_ack_then_new_source_stamp(self):
        self.assertFalse(
            post_grant_reference_is_fresh(
                reference_source_stamp_s=100.2,
                grant_ros_time_s=100.1,
                reference_receive_time=50.1,
                ack_receive_time=50.2,
            )
        )
        self.assertTrue(
            post_grant_reference_is_fresh(
                reference_source_stamp_s=100.2,
                grant_ros_time_s=100.1,
                reference_receive_time=50.3,
                ack_receive_time=50.2,
            )
        )

    def test_exact_match_passes(self):
        incumbent = reference([1, 2, 3], [0.2, 0, 0], [0, 0, 0])
        candidate = reference([1, 2, 3], [0.2, 0, 0], [0, 0, 0])
        errors = stage0_errors(incumbent, candidate)
        self.assertTrue(
            handoff_allowed(
                errors,
                position_tolerance_m=0.05,
                velocity_tolerance_mps=0.20,
                acceleration_tolerance_mps2=0.50,
            )
        )

    def test_each_gate_can_reject(self):
        incumbent = reference([0, 0, 0], [0, 0, 0], [0, 0, 0])
        cases = [
            reference([0.051, 0, 0], [0, 0, 0], [0, 0, 0]),
            reference([0, 0, 0], [0.201, 0, 0], [0, 0, 0]),
            reference([0, 0, 0], [0, 0, 0], [0.501, 0, 0]),
        ]
        for candidate in cases:
            with self.subTest(candidate=candidate.positions[0].tolist()):
                self.assertFalse(
                    handoff_allowed(
                        stage0_errors(incumbent, candidate),
                        position_tolerance_m=0.05,
                        velocity_tolerance_mps=0.20,
                        acceleration_tolerance_mps2=0.50,
                    )
                )

    def test_stage0_only_is_deliberate(self):
        incumbent = reference([0, 0, 0], [0, 0, 0], [0, 0, 0])
        candidate = reference([0, 0, 0], [0, 0, 0], [0, 0, 0])
        candidate.positions[-1] = [10, 10, 10]
        errors = stage0_errors(incumbent, candidate)
        self.assertEqual(errors.position_m, 0.0)

    def test_loaded_lift_prepare_requires_full_quad_tracked_profile(self):
        self.assertFalse(
            loaded_lift_ready_for_cpp(
                object_attached=True,
                object_airborne=True,
                profile_complete=False,
                abs_quad_z_error_m=0.01,
                z_tolerance_m=0.05,
            )
        )
        self.assertFalse(
            loaded_lift_ready_for_cpp(
                object_attached=False,
                object_airborne=True,
                profile_complete=True,
                abs_quad_z_error_m=0.01,
                z_tolerance_m=0.05,
            )
        )
        self.assertFalse(
            loaded_lift_ready_for_cpp(
                object_attached=True,
                object_airborne=False,
                profile_complete=True,
                abs_quad_z_error_m=0.01,
                z_tolerance_m=0.05,
            )
        )
        self.assertFalse(
            loaded_lift_ready_for_cpp(
                object_attached=True,
                object_airborne=True,
                profile_complete=True,
                # 2026-09-17 IRL regression: the old shared 0.20 m pickup
                # tolerance accepted a vehicle still ~0.187 m below the lift
                # endpoint and prematurely latched the measured quad pose.
                abs_quad_z_error_m=0.187,
                z_tolerance_m=0.05,
            )
        )
        self.assertTrue(
            loaded_lift_ready_for_cpp(
                object_attached=True,
                object_airborne=True,
                profile_complete=True,
                abs_quad_z_error_m=0.04,
                z_tolerance_m=0.05,
            )
        )

    def test_loaded_lift_operator_override_keeps_physical_prerequisites(self):
        self.assertTrue(
            loaded_lift_override_allowed(
                object_attached=True,
                object_airborne=True,
                profile_complete=True,
            )
        )
        self.assertFalse(
            loaded_lift_override_allowed(
                object_attached=False,
                object_airborne=True,
                profile_complete=True,
            )
        )
        self.assertFalse(
            loaded_lift_override_allowed(
                object_attached=True,
                object_airborne=False,
                profile_complete=True,
            )
        )
        self.assertFalse(
            loaded_lift_override_allowed(
                object_attached=True,
                object_airborne=True,
                profile_complete=False,
            )
        )

    def test_prepared_window_is_compared_at_frozen_sample_zero(self):
        hold = reference([0.0, 0.0, 2.5], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        prepared = ramp_reference()
        prepared.positions[:, 2] = 2.5
        prepared.positions[0] = hold.positions[0]
        prepared.velocities[0] = 0.0
        raw_errors = stage0_errors(hold, prepared)
        self.assertTrue(
            handoff_allowed(
                raw_errors,
                position_tolerance_m=0.03,
                velocity_tolerance_mps=0.10,
                acceleration_tolerance_mps2=0.50,
            )
        )
        # If an unexecuted prepared trajectory were incorrectly aged by 1 s,
        # its stage 0 would move far away and recreate the C1F.2b commissioning bug.
        incorrectly_aged = advance_reference_window(
            prepared, elapsed_s=1.0, sample_dt_s=1.0 / 30.0, output_count=61
        )
        self.assertGreater(stage0_errors(hold, incorrectly_aged).position_m, 0.03)

    def test_stationary_hold_uses_lift_end_only_as_loose_drift_anchor(self):
        state = stationary_hold_state(
            lift_end_position=np.array([0.0, 0.0, 2.50]),
            measured_position=np.array([0.04, 0.0, 2.50]),
            measured_velocity=np.array([0.02, 0.0, 0.0]),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
        )
        self.assertAlmostEqual(state.drift_m, 0.04)
        self.assertAlmostEqual(state.speed_mps, 0.02)
        self.assertTrue(state.within_drift_bound)
        self.assertTrue(state.speed_settled)

    def test_stationary_hold_rejects_large_drift_or_unsettled_speed(self):
        drifted = stationary_hold_state(
            lift_end_position=np.zeros(3),
            measured_position=np.array([0.151, 0.0, 0.0]),
            measured_velocity=np.zeros(3),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
        )
        moving = stationary_hold_state(
            lift_end_position=np.zeros(3),
            measured_position=np.array([0.04, 0.0, 0.0]),
            measured_velocity=np.array([0.081, 0.0, 0.0]),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
        )
        self.assertFalse(drifted.within_drift_bound)
        self.assertTrue(drifted.speed_settled)
        self.assertTrue(moving.within_drift_bound)
        self.assertFalse(moving.speed_settled)


    def test_stationary_grant_state_revalidates_live_pva(self):
        state = stationary_grant_state(
            lift_end_position=np.array([0.0, 0.0, 0.934]),
            measured_position=np.array([0.022, 0.002, 0.883]),
            measured_velocity=np.array([0.02, 0.01, 0.03]),
            measured_acceleration=np.array([0.08, -0.04, 0.10]),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
            settle_acceleration_mps2=0.50,
        )
        self.assertTrue(state.ready)
        self.assertLess(state.drift_m, 0.15)
        self.assertLess(state.speed_mps, 0.08)
        self.assertLess(state.acceleration_mps2, 0.50)

    def test_stationary_grant_state_rejects_stale_settle_event(self):
        fast = stationary_grant_state(
            lift_end_position=np.zeros(3),
            measured_position=np.array([0.02, 0.0, 0.0]),
            measured_velocity=np.array([0.081, 0.0, 0.0]),
            measured_acceleration=np.zeros(3),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
            settle_acceleration_mps2=0.50,
        )
        accelerating = stationary_grant_state(
            lift_end_position=np.zeros(3),
            measured_position=np.array([0.02, 0.0, 0.0]),
            measured_velocity=np.zeros(3),
            measured_acceleration=np.array([0.501, 0.0, 0.0]),
            max_drift_m=0.15,
            settle_speed_mps=0.08,
            settle_acceleration_mps2=0.50,
        )
        self.assertFalse(fast.ready)
        self.assertFalse(accelerating.ready)


    def test_grant_relatch_avoids_stale_hold_tracking_bias_deadlock(self):
        # Regression for the 2026-09-06 C1F.8c1 run: the old latched Python
        # hold sat at z~=0.9152 while the stationary vehicle and prepared C++
        # plan were at z~=0.8835. The old 3 cm comparison rejected the correct
        # candidate by ~1.7 mm. The grant comparison must use the current
        # measured stationary state instead.
        stale_hold = reference(
            [0.022785, 0.000005, 0.915199], [0, 0, 0], [0, 0, 0]
        )
        current_measured_hold = reference(
            [0.021366, -0.001240, 0.883509], [0, 0, 0], [0, 0, 0]
        )
        prepared = reference(
            [0.021366, -0.001240, 0.883509], [0, 0, 0], [0, 0, 0]
        )
        self.assertGreater(stage0_errors(stale_hold, prepared).position_m, 0.03)
        self.assertTrue(
            handoff_allowed(
                stage0_errors(current_measured_hold, prepared),
                position_tolerance_m=0.03,
                velocity_tolerance_mps=0.10,
                acceleration_tolerance_mps2=0.50,
            )
        )


    def test_target_relative_bridge_starts_at_exact_outgoing_pva(self):
        target = TargetState(
            np.array([2.0, 3.0, 1.5]),
            np.array([0.12, -0.03, 0.0]),
            np.array([0.0, 0.0, 0.0]),
        )
        offset = np.array([0.0, 0.0, 1.0])
        p0 = np.array([2.11, 3.06, 2.43])
        v0 = np.array([0.05, 0.02, -0.01])
        a0 = np.array([0.10, -0.08, 0.04])
        state = target_relative_bridge_initial_state(
            outgoing_position=p0,
            outgoing_velocity=v0,
            outgoing_acceleration=a0,
            target=target,
            relative_offset=offset,
        )
        bridge, complete = target_relative_bridge_reference(
            bridge_state=state,
            target=target,
            relative_offset=offset,
            elapsed_s=0.0,
            duration_s=1.5,
            dt=1.0 / 30.0,
            horizon_samples=61,
        )
        self.assertFalse(complete)
        np.testing.assert_allclose(bridge.positions[0], p0, atol=1e-12)
        np.testing.assert_allclose(bridge.velocities[0], v0, atol=1e-12)
        np.testing.assert_allclose(bridge.accelerations[0], a0, atol=1e-12)

    def test_target_relative_bridge_converges_to_moving_target_tracking(self):
        target = TargetState(
            np.array([1.0, -2.0, 1.5]),
            np.array([0.125, 0.04, 0.0]),
            np.array([0.0, 0.0, 0.0]),
        )
        offset = np.array([0.0, 0.0, 1.0])
        state = target_relative_bridge_initial_state(
            outgoing_position=np.array([1.14, -1.91, 2.46]),
            outgoing_velocity=np.array([0.02, -0.01, 0.0]),
            outgoing_acceleration=np.array([0.0, 0.0, 0.0]),
            target=target,
            relative_offset=offset,
        )
        bridge, complete = target_relative_bridge_reference(
            bridge_state=state,
            target=target,
            relative_offset=offset,
            elapsed_s=1.5,
            duration_s=1.5,
            dt=1.0 / 30.0,
            horizon_samples=61,
        )
        self.assertTrue(complete)
        times = np.arange(61, dtype=float) / 30.0
        expected_positions = np.array([target.predict(t)[0] + offset for t in times])
        expected_velocities = np.array([target.predict(t)[1] for t in times])
        expected_accelerations = np.array([target.predict(t)[2] for t in times])
        np.testing.assert_allclose(bridge.positions, expected_positions, atol=1e-12)
        np.testing.assert_allclose(bridge.velocities, expected_velocities, atol=1e-12)
        np.testing.assert_allclose(bridge.accelerations, expected_accelerations, atol=1e-12)

    def test_target_relative_bridge_stationary_target_is_valid_limiting_case(self):
        target = TargetState(np.zeros(3), np.zeros(3), np.zeros(3))
        offset = np.array([0.0, 0.0, 2.5])
        p0 = np.array([0.08, -0.04, 2.55])
        state = target_relative_bridge_initial_state(
            outgoing_position=p0,
            outgoing_velocity=np.array([0.05, 0.0, 0.0]),
            outgoing_acceleration=np.zeros(3),
            target=target,
            relative_offset=offset,
        )
        start, _ = target_relative_bridge_reference(
            bridge_state=state, target=target, relative_offset=offset,
            elapsed_s=0.0, duration_s=1.0, dt=1.0 / 30.0, horizon_samples=61,
        )
        end, complete = target_relative_bridge_reference(
            bridge_state=state, target=target, relative_offset=offset,
            elapsed_s=1.0, duration_s=1.0, dt=1.0 / 30.0, horizon_samples=61,
        )
        np.testing.assert_allclose(start.positions[0], p0, atol=1e-12)
        self.assertTrue(complete)
        np.testing.assert_allclose(end.positions, np.repeat(offset[None, :], 61, axis=0), atol=1e-12)
        np.testing.assert_allclose(end.velocities, 0.0, atol=1e-12)
        np.testing.assert_allclose(end.accelerations, 0.0, atol=1e-12)


    def test_bridge_feasibility_allows_nominal_moving_target_velocity_mismatch(self):
        target = TargetState(
            np.zeros(3), np.array([0.125, 0.0, 0.0]), np.zeros(3)
        )
        offset = np.array([0.0, 0.0, 1.0])
        state = target_relative_bridge_initial_state(
            outgoing_position=np.array([0.05, 0.0, 1.0]),
            outgoing_velocity=np.zeros(3),
            outgoing_acceleration=np.zeros(3),
            target=target,
            relative_offset=offset,
        )
        result = target_relative_bridge_feasibility(
            bridge_state=state,
            target=target,
            duration_s=1.5,
            max_abs_velocity=np.array([1.0, 1.0, 1.0]),
            max_abs_acceleration=np.array([1.0, 1.0, 1.5]),
            max_abs_jerk=np.array([4.0, 4.0, 4.0]),
        )
        self.assertTrue(result.feasible, result.reason)
        self.assertEqual(result.reason, "OK")
        self.assertLess(result.max_abs_velocity[0], 0.20)
        self.assertLess(result.max_abs_acceleration[0], 0.30)

    def test_bridge_feasibility_rejects_large_uncapturable_velocity_mismatch(self):
        target = TargetState(
            np.zeros(3), np.array([1.0, 0.0, 0.0]), np.zeros(3)
        )
        offset = np.array([0.0, 0.0, 1.0])
        state = target_relative_bridge_initial_state(
            outgoing_position=np.array([0.05, 0.0, 1.0]),
            outgoing_velocity=np.zeros(3),
            outgoing_acceleration=np.zeros(3),
            target=target,
            relative_offset=offset,
        )
        result = target_relative_bridge_feasibility(
            bridge_state=state,
            target=target,
            duration_s=1.5,
            max_abs_velocity=np.array([1.0, 1.0, 1.0]),
            max_abs_acceleration=np.array([1.0, 1.0, 1.5]),
            max_abs_jerk=np.array([4.0, 4.0, 4.0]),
        )
        self.assertFalse(result.feasible)
        self.assertIn(result.reason, {"VELOCITY_LIMIT", "ACCELERATION_LIMIT", "JERK_LIMIT"})

    def test_cached_window_advances_through_six_message_dropout(self):
        committed = ramp_reference()
        advanced = advance_reference_window(
            committed,
            elapsed_s=6.0 / 30.0,
            sample_dt_s=1.0 / 30.0,
            output_count=61,
        )
        np.testing.assert_allclose(advanced.positions[0], committed.positions[6], atol=1e-12)
        self.assertEqual(advanced.positions.shape, (61, 3))
        np.testing.assert_allclose(advanced.positions[-1], committed.positions[-1])
        np.testing.assert_allclose(advanced.velocities[-1], committed.velocities[-1])

    def test_cached_window_interpolates_fractional_age(self):
        committed = ramp_reference()
        advanced = advance_reference_window(
            committed,
            elapsed_s=0.5 / 30.0,
            sample_dt_s=1.0 / 30.0,
            output_count=61,
        )
        expected = 0.5 * (committed.positions[0] + committed.positions[1])
        np.testing.assert_allclose(advanced.positions[0], expected, atol=1e-12)

    def test_cached_window_eventually_becomes_terminal_hold(self):
        committed = ramp_reference()
        advanced = advance_reference_window(
            committed,
            elapsed_s=5.0,
            sample_dt_s=1.0 / 30.0,
            output_count=61,
        )
        np.testing.assert_allclose(
            advanced.positions,
            np.repeat(committed.positions[-1][None, :], 61, axis=0),
        )
        np.testing.assert_allclose(advanced.velocities, 0.0)
        np.testing.assert_allclose(advanced.accelerations, 0.0)


if __name__ == '__main__':
    unittest.main()
