"""Scalar thrust-ratio UKF with full nonlinear model propagation.

The filter state is only the effective thrust ratio ``kT``.  For each of the
three scalar sigma points, the controller supplies a callback that propagates
the full quadrotor + payload model across the applied-control history and
returns the predicted position/velocity measurement.

This retains attitude- and payload-aware process dynamics without the 29 sigma
points required by the legacy 14-state UKF.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np


class FullModelThrustRatioUKF:
    """One-state UKF for effective acceleration-per-throttle gain."""

    STATE_DIM = 1

    def __init__(
        self,
        *,
        thrust_ratio_min: float,
        thrust_ratio_max: float,
        alpha: float = 0.7,
        beta: float = 2.0,
        kappa: float = 0.0,
        initial_thrust_ratio_std: float = 4.0,
        process_thrust_ratio_std_per_sqrt_s: float = 0.20,
        measurement_position_std: float = 0.01,
        measurement_velocity_std: float = 0.10,
        filtered_estimate_alpha: float = 0.10,
        nis_threshold: float = 22.46,
        minimum_variance: float = 1e-8,
    ) -> None:
        self.thrust_ratio_min = float(thrust_ratio_min)
        self.thrust_ratio_max = float(thrust_ratio_max)
        if self.thrust_ratio_min >= self.thrust_ratio_max:
            raise ValueError('thrust_ratio_min must be smaller than thrust_ratio_max')

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.kappa = float(kappa)
        self.initial_variance = float(initial_thrust_ratio_std) ** 2
        self.process_variance_per_second = (
            float(process_thrust_ratio_std_per_sqrt_s) ** 2
        )
        self.filtered_estimate_alpha = float(
            np.clip(filtered_estimate_alpha, 0.0, 1.0)
        )
        self.nis_threshold = float(nis_threshold)
        self.minimum_variance = max(float(minimum_variance), 1e-12)

        self.base_measurement_covariance = np.diag(np.array([
            *([float(measurement_position_std) ** 2] * 3),
            *([float(measurement_velocity_std) ** 2] * 3),
        ], dtype=float))

        self.mean: Optional[float] = None
        self.variance: Optional[float] = None
        self.filtered_thrust_ratio: Optional[float] = None
        self.update_count = 0

    @property
    def initialized(self) -> bool:
        return self.mean is not None and self.variance is not None

    def reset(self) -> None:
        self.mean = None
        self.variance = None
        self.filtered_thrust_ratio = None
        self.update_count = 0

    def initialize(self, thrust_ratio_initial: float) -> Dict[str, object]:
        initial = float(np.clip(
            thrust_ratio_initial,
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        ))
        self.mean = initial
        self.variance = max(self.initial_variance, self.minimum_variance)
        self.filtered_thrust_ratio = initial
        self.update_count = 0
        return self._result(
            status='initialized',
            updated=False,
            innovation_norm=float('nan'),
            normalized_innovation_squared=float('nan'),
            update_time_s=0.0,
        )

    def snapshot(self, status: str = 'idle') -> Dict[str, object]:
        return self._result(
            status=status,
            updated=False,
            innovation_norm=float('nan'),
            normalized_innovation_squared=float('nan'),
            update_time_s=0.0,
        )

    def update(
        self,
        measurement: np.ndarray,
        *,
        process_model: Callable[[float], np.ndarray],
        dt: float,
        measurement_covariance_scale: float = 1.0,
    ) -> Dict[str, object]:
        """Perform one scalar-UKF update using full-model measurement predictions.

        Args:
            measurement: Latest measured ``[px, py, pz, vx, vy, vz]``.
            process_model: Maps a candidate ``kT`` to the corresponding predicted
                six-element position/velocity measurement after the full applied
                control interval.
            dt: Elapsed estimator interval, used to scale random-walk process noise.
            measurement_covariance_scale: Multiplier applied to the base measurement
                covariance during aggressive or poorly modelled motion.
        """
        if not self.initialized:
            raise RuntimeError('UKF must be initialized before update()')

        start_time = time.perf_counter()
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        if measurement.shape != (6,):
            raise ValueError(f'Expected six measurement values, got {measurement.shape}')
        if not np.all(np.isfinite(measurement)):
            raise ValueError('Measurement contains NaN or Inf')

        assert self.mean is not None
        assert self.variance is not None

        sigma_points, weights_mean, weights_covariance = self._sigma_points(
            self.mean,
            self.variance,
        )

        predictions = []
        for sigma_point in sigma_points:
            predicted = np.asarray(
                process_model(float(sigma_point)), dtype=float
            ).reshape(-1)
            if predicted.shape != (6,):
                raise ValueError(
                    f'Process model must return six values, got {predicted.shape}'
                )
            if not np.all(np.isfinite(predicted)):
                raise ValueError('Process model returned NaN or Inf')
            predictions.append(predicted)
        predictions = np.asarray(predictions, dtype=float)

        predicted_mean = float(np.sum(weights_mean * sigma_points))
        predicted_mean = float(np.clip(
            predicted_mean,
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        ))
        predicted_variance = float(np.sum(
            weights_covariance * (sigma_points - predicted_mean) ** 2
        ))
        predicted_variance += self.process_variance_per_second * max(float(dt), 0.0)
        predicted_variance = max(predicted_variance, self.minimum_variance)

        predicted_measurement = np.sum(
            weights_mean[:, None] * predictions,
            axis=0,
        )
        measurement_residuals = predictions - predicted_measurement
        state_residuals = sigma_points - predicted_mean

        scale = max(float(measurement_covariance_scale), 1.0)
        innovation_covariance = self.base_measurement_covariance * scale
        cross_covariance = np.zeros(6, dtype=float)
        for index in range(sigma_points.shape[0]):
            innovation_covariance += (
                weights_covariance[index]
                * np.outer(
                    measurement_residuals[index],
                    measurement_residuals[index],
                )
            )
            cross_covariance += (
                weights_covariance[index]
                * state_residuals[index]
                * measurement_residuals[index]
            )

        innovation_covariance = 0.5 * (
            innovation_covariance + innovation_covariance.T
        )
        innovation = measurement - predicted_measurement
        solved_innovation = np.linalg.solve(innovation_covariance, innovation)
        normalized_innovation_squared = float(innovation @ solved_innovation)
        innovation_norm = float(np.linalg.norm(innovation))

        # Preserve the random-walk prediction but reject a measurement that is
        # clearly inconsistent with the process model.  This prevents one violent
        # manoeuvre or timing glitch from permanently shifting kT.
        if (
            self.nis_threshold > 0.0
            and normalized_innovation_squared > self.nis_threshold
        ):
            self.mean = predicted_mean
            self.variance = predicted_variance
            return self._result(
                status='rejected_nis',
                updated=False,
                innovation_norm=innovation_norm,
                normalized_innovation_squared=normalized_innovation_squared,
                update_time_s=time.perf_counter() - start_time,
            )

        kalman_gain = np.linalg.solve(
            innovation_covariance,
            cross_covariance,
        )
        updated_mean = predicted_mean + float(kalman_gain @ innovation)
        updated_variance = predicted_variance - float(
            cross_covariance @ kalman_gain
        )

        self.mean = float(np.clip(
            updated_mean,
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        ))
        self.variance = max(updated_variance, self.minimum_variance)
        self.update_count += 1

        if self.filtered_thrust_ratio is None:
            self.filtered_thrust_ratio = self.mean
        else:
            filter_alpha = self.filtered_estimate_alpha
            self.filtered_thrust_ratio = (
                (1.0 - filter_alpha) * self.filtered_thrust_ratio
                + filter_alpha * self.mean
            )

        return self._result(
            status='updated',
            updated=True,
            innovation_norm=innovation_norm,
            normalized_innovation_squared=normalized_innovation_squared,
            update_time_s=time.perf_counter() - start_time,
        )

    def _sigma_points(
        self,
        mean: float,
        variance: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        dimension = self.STATE_DIM
        lambda_value = self.alpha ** 2 * (dimension + self.kappa) - dimension
        scale = dimension + lambda_value
        if scale <= 0.0:
            raise ValueError('UKF scaling produced non-positive sigma-point spread')

        spread = float(np.sqrt(scale * max(variance, self.minimum_variance)))
        sigma_points = np.array([
            mean,
            mean + spread,
            mean - spread,
        ], dtype=float)
        sigma_points = np.clip(
            sigma_points,
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        )

        weights_mean = np.full(3, 1.0 / (2.0 * scale), dtype=float)
        weights_covariance = weights_mean.copy()
        weights_mean[0] = lambda_value / scale
        weights_covariance[0] = (
            weights_mean[0] + (1.0 - self.alpha ** 2 + self.beta)
        )
        return sigma_points, weights_mean, weights_covariance

    def _result(
        self,
        *,
        status: str,
        updated: bool,
        innovation_norm: float,
        normalized_innovation_squared: float,
        update_time_s: float,
    ) -> Dict[str, object]:
        raw = float('nan') if self.mean is None else float(self.mean)
        variance = (
            float('nan') if self.variance is None else float(self.variance)
        )
        filtered = (
            float('nan')
            if self.filtered_thrust_ratio is None
            else float(self.filtered_thrust_ratio)
        )
        return {
            'status': str(status),
            'initialized': self.initialized,
            'updated': bool(updated),
            'raw_thrust_ratio': raw,
            'filtered_thrust_ratio': filtered,
            'thrust_ratio_variance': variance,
            'innovation_norm': float(innovation_norm),
            'normalized_innovation_squared': float(
                normalized_innovation_squared
            ),
            'update_time_s': float(update_time_s),
            'update_count': int(self.update_count),
        }
