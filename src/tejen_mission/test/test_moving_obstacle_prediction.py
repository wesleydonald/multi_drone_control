import unittest

import numpy as np

from tejen_mission.moving_obstacle_prediction import (
    MovingObstaclePredictionConfig,
    MovingObstaclePredictor,
    MovingObstacleState,
    build_reachable_prediction,
    low_pass_velocity,
    prediction_radius_m,
)


class MovingObstaclePredictionTests(unittest.TestCase):
    def test_radius_uses_quadratic_uncertainty_growth(self):
        config = MovingObstaclePredictionConfig(
            position_uncertainty_m=0.03,
            velocity_uncertainty_mps=0.10,
            acceleration_uncertainty_mps2=0.20,
        )

        radius = prediction_radius_m(
            geometry_radius_m=0.20,
            future_time_s=2.0,
            config=config,
        )

        self.assertAlmostEqual(
            radius,
            0.20 + 0.03 + 0.10 * 2.0 + 0.5 * 0.20 * 2.0**2,
        )

    def test_constant_velocity_prediction_has_expected_centres(self):
        config = MovingObstaclePredictionConfig(
            horizon_s=1.0,
            time_step_s=0.25,
        )
        state = MovingObstacleState(
            obstacle_id="test",
            position=[1.0, 2.0, 3.0],
            velocity=[0.5, -0.2, 0.1],
            geometry_radius_m=0.15,
        )

        result = build_reachable_prediction(
            state,
            filtered_velocity=state.velocity,
            config=config,
        )

        expected = (
            state.position[None, :]
            + result.sample_times_s[:, None] * state.velocity[None, :]
        )

        np.testing.assert_allclose(
            result.sample_centers,
            expected,
            atol=1e-12,
        )
        self.assertAlmostEqual(
            result.sample_times_s[-1],
            config.horizon_s,
        )

    def test_stationary_obstacle_becomes_expanding_ball(self):
        config = MovingObstaclePredictionConfig(
            horizon_s=1.0,
            time_step_s=0.2,
        )
        state = MovingObstacleState(
            obstacle_id="stationary",
            position=[0.4, -0.2, 1.0],
            velocity=[0.0, 0.0, 0.0],
            geometry_radius_m=0.10,
        )

        result = build_reachable_prediction(
            state,
            filtered_velocity=[0.0, 0.0, 0.0],
            config=config,
        )

        np.testing.assert_allclose(
            result.sample_centers,
            np.repeat(state.position[None, :], len(result.sample_times_s), axis=0),
        )
        self.assertTrue(
            np.all(np.diff(result.sample_radii_m) > 0.0)
        )

    def test_intervals_use_conservative_larger_endpoint_radius(self):
        config = MovingObstaclePredictionConfig(
            horizon_s=1.0,
            time_step_s=0.25,
        )
        state = MovingObstacleState(
            obstacle_id="test",
            position=[0.0, 0.0, 0.0],
            velocity=[1.0, 0.0, 0.0],
            geometry_radius_m=0.10,
        )

        result = build_reachable_prediction(
            state,
            filtered_velocity=state.velocity,
            config=config,
        )

        self.assertEqual(
            len(result.intervals),
            len(result.sample_times_s) - 1,
        )

        for interval in result.intervals:
            self.assertAlmostEqual(
                interval.conservative_radius_m,
                interval.end_radius_m,
            )
            self.assertGreaterEqual(
                interval.conservative_radius_m,
                interval.start_radius_m,
            )

    def test_low_pass_velocity_matches_first_order_filter(self):
        result = low_pass_velocity(
            previous_filtered_velocity=[0.0, 0.0, 0.0],
            measured_velocity=[1.0, -2.0, 0.5],
            dt_s=0.1,
            tau_s=0.2,
        )

        alpha = 0.1 / (0.2 + 0.1)
        np.testing.assert_allclose(
            result,
            alpha * np.array([1.0, -2.0, 0.5]),
        )

    def test_predictor_initialises_from_measurement_then_filters(self):
        config = MovingObstaclePredictionConfig(
            velocity_filter_tau_s=0.20,
        )
        predictor = MovingObstaclePredictor(config)

        first_state = MovingObstacleState(
            obstacle_id="test",
            position=[0.0, 0.0, 0.0],
            velocity=[1.0, 0.0, 0.0],
            geometry_radius_m=0.1,
        )

        first = predictor.update(first_state)
        np.testing.assert_allclose(
            first.filtered_velocity,
            [1.0, 0.0, 0.0],
        )

        second_state = MovingObstacleState(
            obstacle_id="test",
            position=[0.1, 0.0, 0.0],
            velocity=[0.0, 1.0, 0.0],
            geometry_radius_m=0.1,
        )

        second = predictor.update(
            second_state,
            dt_s=0.1,
        )

        alpha = 0.1 / 0.3
        expected = (
            np.array([1.0, 0.0, 0.0])
            + alpha
            * (
                np.array([0.0, 1.0, 0.0])
                - np.array([1.0, 0.0, 0.0])
            )
        )

        np.testing.assert_allclose(
            second.filtered_velocity,
            expected,
        )

    def test_bad_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            MovingObstaclePredictionConfig(
                horizon_s=-1.0,
            )


if __name__ == "__main__":
    unittest.main()
