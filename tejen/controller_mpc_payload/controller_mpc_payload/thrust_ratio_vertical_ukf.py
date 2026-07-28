"""Lightweight vertical UKF for online effective thrust-ratio estimation.

State:
    x = [z, vz, thrust_ratio]

Measurement:
    y = [z_mocap, vz_mocap]

The estimator is deliberately independent of ROS and acados. It exposes the
same result dictionary fields as the full supervisor-style thrust-ratio UKF so
both estimators can share controller integration and logging.
"""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

import numpy as np


class VerticalThrustRatioUKF:
    """Three-state UKF for vertical position, velocity and thrust ratio."""

    STATE_DIM = 3
    MEASUREMENT_DIM = 2
    THRUST_RATIO_INDEX = 2

    def __init__(
        self,
        *,
        thrust_ratio_min: float,
        thrust_ratio_max: float,
        alpha: float = 0.8,
        beta: float = 2.0,
        kappa: float = 0.0,
        initial_z_std: float = 0.02,
        initial_vz_std: float = 0.15,
        initial_thrust_ratio_std: float = 4.0,
        process_z_std_per_sqrt_s: float = 0.02,
        process_vz_std_per_sqrt_s: float = 0.20,
        process_thrust_ratio_std_per_sqrt_s: float = 0.20,
        measurement_z_std: float = 0.01,
        measurement_vz_std: float = 0.10,
        filtered_estimate_alpha: float = 0.10,
        minimum_covariance_eigenvalue: float = 1e-10,
    ) -> None:
        self.thrust_ratio_min = float(thrust_ratio_min)
        self.thrust_ratio_max = float(thrust_ratio_max)
        if self.thrust_ratio_min >= self.thrust_ratio_max:
            raise ValueError(
                'thrust_ratio_min must be smaller than thrust_ratio_max'
            )

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.kappa = float(kappa)
        self.filtered_estimate_alpha = float(
            np.clip(filtered_estimate_alpha, 0.0, 1.0)
        )
        self.minimum_covariance_eigenvalue = float(
            minimum_covariance_eigenvalue
        )

        self._initial_variance = np.array(
            [
                initial_z_std**2,
                initial_vz_std**2,
                initial_thrust_ratio_std**2,
            ],
            dtype=float,
        )
        self._process_std_per_sqrt_s = np.array(
            [
                process_z_std_per_sqrt_s,
                process_vz_std_per_sqrt_s,
                process_thrust_ratio_std_per_sqrt_s,
            ],
            dtype=float,
        )
        self.R = np.diag(
            np.array(
                [measurement_z_std**2, measurement_vz_std**2],
                dtype=float,
            )
        )

        self.x: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None
        self.filtered_thrust_ratio: Optional[float] = None
        self.update_count = 0

    @property
    def initialized(self) -> bool:
        return self.x is not None and self.P is not None

    def reset(self) -> None:
        self.x = None
        self.P = None
        self.filtered_thrust_ratio = None
        self.update_count = 0

    def initialize(
        self,
        measurement: np.ndarray,
        thrust_ratio_initial: float,
    ) -> Dict[str, object]:
        vertical_measurement = self._extract_measurement(measurement)
        thrust_ratio = float(
            np.clip(
                thrust_ratio_initial,
                self.thrust_ratio_min,
                self.thrust_ratio_max,
            )
        )

        self.x = np.array(
            [vertical_measurement[0], vertical_measurement[1], thrust_ratio],
            dtype=float,
        )
        self.P = np.diag(self._initial_variance.copy())
        self.filtered_thrust_ratio = thrust_ratio
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
        vertical_thrust_factor: float,
        drag_coeff_z: float,
        dt: float,
    ) -> Dict[str, object]:
        """Perform one predict/update step.

        vertical_thrust_factor is the mean of R33 * throttle over the estimator
        interval. The acceleration model is:

            az = thrust_ratio * vertical_thrust_factor - g - drag_coeff_z * vz
        """
        if not self.initialized:
            raise RuntimeError('UKF must be initialized before update()')

        start_time = time.perf_counter()
        vertical_measurement = self._extract_measurement(measurement)
        dt = float(np.clip(dt, 1e-3, 0.5))
        vertical_thrust_factor = float(vertical_thrust_factor)
        drag_coeff_z = float(drag_coeff_z)

        assert self.x is not None
        assert self.P is not None

        sigma_points, weights_mean, weights_covariance = (
            self._generate_sigma_points(self.x, self.P)
        )
        propagated = np.empty_like(sigma_points)

        for index, sigma_point in enumerate(sigma_points):
            z, vz, thrust_ratio = sigma_point
            acceleration_z = (
                thrust_ratio * vertical_thrust_factor
                - 9.81
                - drag_coeff_z * vz
            )
            propagated[index, 0] = (
                z + vz * dt + 0.5 * acceleration_z * dt * dt
            )
            propagated[index, 1] = vz + acceleration_z * dt
            propagated[index, 2] = np.clip(
                thrust_ratio,
                self.thrust_ratio_min,
                self.thrust_ratio_max,
            )

        x_predicted = np.sum(weights_mean[:, None] * propagated, axis=0)
        x_predicted[self.THRUST_RATIO_INDEX] = np.clip(
            x_predicted[self.THRUST_RATIO_INDEX],
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        )

        process_variance = (self._process_std_per_sqrt_s**2) * dt
        P_predicted = np.diag(process_variance)
        for index in range(propagated.shape[0]):
            residual = propagated[index] - x_predicted
            P_predicted += weights_covariance[index] * np.outer(
                residual, residual
            )
        P_predicted = self._project_covariance(P_predicted)

        sigma_measurements = propagated[:, 0:2]
        predicted_measurement = np.sum(
            weights_mean[:, None] * sigma_measurements,
            axis=0,
        )

        innovation_covariance = self.R.copy()
        cross_covariance = np.zeros(
            (self.STATE_DIM, self.MEASUREMENT_DIM),
            dtype=float,
        )
        for index in range(propagated.shape[0]):
            state_residual = propagated[index] - x_predicted
            measurement_residual = (
                sigma_measurements[index] - predicted_measurement
            )
            innovation_covariance += weights_covariance[index] * np.outer(
                measurement_residual, measurement_residual
            )
            cross_covariance += weights_covariance[index] * np.outer(
                state_residual, measurement_residual
            )

        innovation_covariance = self._project_covariance(
            innovation_covariance
        )
        innovation = vertical_measurement - predicted_measurement
        kalman_gain = np.linalg.solve(
            innovation_covariance.T,
            cross_covariance.T,
        ).T

        updated_state = x_predicted + kalman_gain @ innovation
        updated_state[self.THRUST_RATIO_INDEX] = np.clip(
            updated_state[self.THRUST_RATIO_INDEX],
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        )
        updated_covariance = (
            P_predicted
            - kalman_gain @ innovation_covariance @ kalman_gain.T
        )

        self.x = updated_state
        self.P = self._project_covariance(updated_covariance)
        self.update_count += 1

        raw_thrust_ratio = float(self.x[self.THRUST_RATIO_INDEX])
        if self.filtered_thrust_ratio is None:
            self.filtered_thrust_ratio = raw_thrust_ratio
        else:
            alpha = self.filtered_estimate_alpha
            self.filtered_thrust_ratio = (
                (1.0 - alpha) * self.filtered_thrust_ratio
                + alpha * raw_thrust_ratio
            )

        normalized_innovation_squared = float(
            innovation
            @ np.linalg.solve(innovation_covariance, innovation)
        )

        return self._result(
            status='updated',
            updated=True,
            innovation_norm=float(np.linalg.norm(innovation)),
            normalized_innovation_squared=(
                normalized_innovation_squared
            ),
            update_time_s=time.perf_counter() - start_time,
        )

    def _result(
        self,
        *,
        status: str,
        updated: bool,
        innovation_norm: float,
        normalized_innovation_squared: float,
        update_time_s: float,
    ) -> Dict[str, object]:
        raw = float('nan')
        variance = float('nan')
        if self.x is not None:
            raw = float(self.x[self.THRUST_RATIO_INDEX])
        if self.P is not None:
            variance = float(
                self.P[
                    self.THRUST_RATIO_INDEX,
                    self.THRUST_RATIO_INDEX,
                ]
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

    def _generate_sigma_points(
        self,
        mean: np.ndarray,
        covariance: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        dimension = self.STATE_DIM
        lambda_value = (
            self.alpha**2 * (dimension + self.kappa) - dimension
        )
        scale = dimension + lambda_value
        if scale <= 0.0:
            raise ValueError('UKF scaling produced a non-positive spread')

        square_root = np.linalg.cholesky(
            scale * self._project_covariance(covariance)
        )
        sigma_points = np.empty(
            (2 * dimension + 1, dimension),
            dtype=float,
        )
        sigma_points[0] = mean
        for index in range(dimension):
            sigma_points[1 + 2 * index] = mean + square_root[:, index]
            sigma_points[2 + 2 * index] = mean - square_root[:, index]

        sigma_points[:, self.THRUST_RATIO_INDEX] = np.clip(
            sigma_points[:, self.THRUST_RATIO_INDEX],
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        )

        weights_mean = np.full(
            2 * dimension + 1,
            1.0 / (2.0 * scale),
            dtype=float,
        )
        weights_covariance = weights_mean.copy()
        weights_mean[0] = lambda_value / scale
        weights_covariance[0] = (
            lambda_value / scale
            + (1.0 - self.alpha**2 + self.beta)
        )
        return sigma_points, weights_mean, weights_covariance

    def _extract_measurement(self, measurement: np.ndarray) -> np.ndarray:
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        if measurement.shape == (2,):
            result = measurement.copy()
        elif measurement.size >= 10:
            result = np.array([measurement[2], measurement[9]], dtype=float)
        else:
            raise ValueError('Expected [z, vz] or a full mocap state')

        if not np.all(np.isfinite(result)):
            raise ValueError('Vertical measurement contains NaN or Inf')
        return result

    def _project_covariance(self, covariance: np.ndarray) -> np.ndarray:
        covariance = np.asarray(covariance, dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(
            eigenvalues,
            self.minimum_covariance_eigenvalue,
        )
        projected = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        return 0.5 * (projected + projected.T)
