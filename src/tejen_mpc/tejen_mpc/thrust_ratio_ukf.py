"""Focused augmented-state UKF for online thrust-ratio estimation.

The filter estimates the 13 measured quadrotor states plus one effective
thrust-ratio parameter. The process model is supplied by the controller so the
same acados dynamics can be reused without coupling this module to ROS or
acados.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np


class ThrustRatioUKF:
    """Unscented Kalman filter for a 13-state quadrotor plus thrust ratio."""

    PHYSICAL_STATE_DIM = 13
    AUGMENTED_STATE_DIM = 14
    MEASUREMENT_DIM = 13
    QUATERNION_SLICE = slice(3, 7)
    THRUST_RATIO_INDEX = 13

    def __init__(
        self,
        *,
        thrust_ratio_min: float,
        thrust_ratio_max: float,
        alpha: float = 0.7,
        beta: float = 2.0,
        kappa: float = 0.0,
        initial_position_std: float = 0.02,
        initial_quaternion_std: float = 0.02,
        initial_velocity_std: float = 0.15,
        initial_angular_velocity_std: float = 0.20,
        initial_thrust_ratio_std: float = 4.0,
        process_position_std: float = 0.003,
        process_quaternion_std: float = 0.003,
        process_velocity_std: float = 0.05,
        process_angular_velocity_std: float = 0.10,
        process_thrust_ratio_std: float = 0.03,
        measurement_position_std: float = 0.005,
        measurement_quaternion_std: float = 0.01,
        measurement_velocity_std: float = 0.08,
        measurement_angular_velocity_std: float = 0.15,
        filtered_estimate_alpha: float = 0.10,
        minimum_covariance_eigenvalue: float = 1e-10,
    ) -> None:
        self.thrust_ratio_min = float(thrust_ratio_min)
        self.thrust_ratio_max = float(thrust_ratio_max)
        if self.thrust_ratio_min >= self.thrust_ratio_max:
            raise ValueError('thrust_ratio_min must be smaller than thrust_ratio_max')

        self.alpha = float(alpha)
        self.beta = float(beta)
        self.kappa = float(kappa)
        self.filtered_estimate_alpha = float(np.clip(filtered_estimate_alpha, 0.0, 1.0))
        self.minimum_covariance_eigenvalue = float(minimum_covariance_eigenvalue)

        self._initial_variance = np.array([
            *([initial_position_std ** 2] * 3),
            *([initial_quaternion_std ** 2] * 4),
            *([initial_velocity_std ** 2] * 3),
            *([initial_angular_velocity_std ** 2] * 3),
            initial_thrust_ratio_std ** 2,
        ], dtype=float)

        self.Q = np.diag(np.array([
            *([process_position_std ** 2] * 3),
            *([process_quaternion_std ** 2] * 4),
            *([process_velocity_std ** 2] * 3),
            *([process_angular_velocity_std ** 2] * 3),
            process_thrust_ratio_std ** 2,
        ], dtype=float))

        self.R = np.diag(np.array([
            *([measurement_position_std ** 2] * 3),
            *([measurement_quaternion_std ** 2] * 4),
            *([measurement_velocity_std ** 2] * 3),
            *([measurement_angular_velocity_std ** 2] * 3),
        ], dtype=float))

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

    def snapshot(self, status: str = 'idle') -> Dict[str, object]:
        """Return the current estimate without performing a predict/update step."""
        return self._result(
            status=status,
            updated=False,
            innovation_norm=float('nan'),
            normalized_innovation_squared=float('nan'),
            update_time_s=0.0,
        )

    def initialize(self, measurement: np.ndarray, thrust_ratio_initial: float) -> Dict[str, object]:
        measurement = self._validate_measurement(measurement)
        q = self._normalize_quaternion(measurement[self.QUATERNION_SLICE])
        measurement = measurement.copy()
        measurement[self.QUATERNION_SLICE] = q

        thrust_ratio = float(np.clip(
            thrust_ratio_initial,
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        ))

        self.x = np.concatenate((measurement, np.array([thrust_ratio], dtype=float)))
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

    def update(
        self,
        measurement: np.ndarray,
        process_model: Callable[[np.ndarray], np.ndarray],
    ) -> Dict[str, object]:
        if not self.initialized:
            raise RuntimeError('UKF must be initialized before update()')

        start_time = time.perf_counter()
        measurement = self._validate_measurement(measurement)
        assert self.x is not None
        assert self.P is not None

        sigma_points, weights_mean, weights_covariance = self._generate_sigma_points(
            self.x,
            self.P,
        )

        propagated = []
        for sigma_point in sigma_points:
            predicted = np.asarray(process_model(sigma_point.copy()), dtype=float).reshape(-1)
            if predicted.shape != (self.AUGMENTED_STATE_DIM,):
                raise ValueError(
                    'Process model must return a 14-element augmented state; '
                    f'got {predicted.shape}'
                )
            if not np.all(np.isfinite(predicted)):
                raise ValueError('Process model returned NaN or Inf')
            propagated.append(predicted)

        sigma_points_predicted = np.asarray(propagated, dtype=float)
        reference_quaternion = self._normalize_quaternion(
            sigma_points_predicted[0, self.QUATERNION_SLICE]
        )
        for index in range(sigma_points_predicted.shape[0]):
            sigma_points_predicted[index] = self._sanitize_augmented_state(
                sigma_points_predicted[index],
                reference_quaternion=reference_quaternion,
            )

        x_predicted = self._augmented_mean(sigma_points_predicted, weights_mean)
        P_predicted = self.Q.copy()
        for index in range(sigma_points_predicted.shape[0]):
            dx = self._augmented_residual(sigma_points_predicted[index], x_predicted)
            P_predicted += weights_covariance[index] * np.outer(dx, dx)
        P_predicted = self._project_covariance(P_predicted)

        sigma_measurements = sigma_points_predicted[:, :self.MEASUREMENT_DIM].copy()
        z_predicted = self._measurement_mean(sigma_measurements, weights_mean)

        P_zz = self.R.copy()
        P_xz = np.zeros((self.AUGMENTED_STATE_DIM, self.MEASUREMENT_DIM), dtype=float)
        for index in range(sigma_points_predicted.shape[0]):
            dx = self._augmented_residual(sigma_points_predicted[index], x_predicted)
            dz = self._measurement_residual(sigma_measurements[index], z_predicted)
            P_zz += weights_covariance[index] * np.outer(dz, dz)
            P_xz += weights_covariance[index] * np.outer(dx, dz)
        P_zz = self._project_covariance(P_zz)

        measurement = measurement.copy()
        measurement[self.QUATERNION_SLICE] = self._align_quaternion(
            measurement[self.QUATERNION_SLICE],
            z_predicted[self.QUATERNION_SLICE],
        )
        innovation = self._measurement_residual(measurement, z_predicted)

        kalman_gain = np.linalg.solve(P_zz.T, P_xz.T).T
        updated_state = x_predicted + kalman_gain @ innovation
        updated_state = self._sanitize_augmented_state(
            updated_state,
            reference_quaternion=x_predicted[self.QUATERNION_SLICE],
        )
        updated_covariance = P_predicted - kalman_gain @ P_zz @ kalman_gain.T

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
            innovation @ np.linalg.solve(P_zz, innovation)
        )
        return self._result(
            status='updated',
            updated=True,
            innovation_norm=float(np.linalg.norm(innovation)),
            normalized_innovation_squared=normalized_innovation_squared,
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
            variance = float(self.P[self.THRUST_RATIO_INDEX, self.THRUST_RATIO_INDEX])

        filtered = (
            float('nan')
            if self.filtered_thrust_ratio is None
            else float(self.filtered_thrust_ratio)
        )
        return {
            'status': status,
            'initialized': self.initialized,
            'updated': bool(updated),
            'raw_thrust_ratio': raw,
            'filtered_thrust_ratio': filtered,
            'thrust_ratio_variance': variance,
            'innovation_norm': float(innovation_norm),
            'normalized_innovation_squared': float(normalized_innovation_squared),
            'update_time_s': float(update_time_s),
            'update_count': int(self.update_count),
        }

    def _generate_sigma_points(
        self,
        mean: np.ndarray,
        covariance: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n = self.AUGMENTED_STATE_DIM
        lambda_value = self.alpha ** 2 * (n + self.kappa) - n
        scale = n + lambda_value
        if scale <= 0.0:
            raise ValueError('UKF scaling produced non-positive sigma-point spread')

        covariance = self._project_covariance(covariance)
        try:
            square_root = np.linalg.cholesky(scale * covariance)
        except np.linalg.LinAlgError:
            covariance = self._project_covariance(
                covariance + np.eye(n, dtype=float) * self.minimum_covariance_eigenvalue
            )
            square_root = np.linalg.cholesky(scale * covariance)

        sigma_points = np.empty((2 * n + 1, n), dtype=float)
        sigma_points[0] = mean
        for index in range(n):
            sigma_points[1 + 2 * index] = mean + square_root[:, index]
            sigma_points[2 + 2 * index] = mean - square_root[:, index]

        reference_quaternion = mean[self.QUATERNION_SLICE]
        for index in range(sigma_points.shape[0]):
            sigma_points[index] = self._sanitize_augmented_state(
                sigma_points[index],
                reference_quaternion=reference_quaternion,
            )

        weights_mean = np.full(2 * n + 1, 1.0 / (2.0 * scale), dtype=float)
        weights_covariance = weights_mean.copy()
        weights_mean[0] = lambda_value / scale
        weights_covariance[0] = (
            lambda_value / scale + (1.0 - self.alpha ** 2 + self.beta)
        )
        return sigma_points, weights_mean, weights_covariance

    def _augmented_mean(self, sigma_points: np.ndarray, weights: np.ndarray) -> np.ndarray:
        mean = np.sum(weights[:, None] * sigma_points, axis=0)
        reference = sigma_points[0, self.QUATERNION_SLICE]
        aligned_quaternions = np.array([
            self._align_quaternion(point[self.QUATERNION_SLICE], reference)
            for point in sigma_points
        ])
        quaternion_mean = np.sum(weights[:, None] * aligned_quaternions, axis=0)
        mean[self.QUATERNION_SLICE] = self._normalize_quaternion(
            quaternion_mean,
            fallback=reference,
        )
        return self._sanitize_augmented_state(mean, reference_quaternion=reference)

    def _measurement_mean(self, sigma_points: np.ndarray, weights: np.ndarray) -> np.ndarray:
        mean = np.sum(weights[:, None] * sigma_points, axis=0)
        reference = sigma_points[0, self.QUATERNION_SLICE]
        aligned_quaternions = np.array([
            self._align_quaternion(point[self.QUATERNION_SLICE], reference)
            for point in sigma_points
        ])
        quaternion_mean = np.sum(weights[:, None] * aligned_quaternions, axis=0)
        mean[self.QUATERNION_SLICE] = self._normalize_quaternion(
            quaternion_mean,
            fallback=reference,
        )
        return mean

    def _augmented_residual(self, value: np.ndarray, reference: np.ndarray) -> np.ndarray:
        aligned = value.copy()
        aligned[self.QUATERNION_SLICE] = self._align_quaternion(
            aligned[self.QUATERNION_SLICE],
            reference[self.QUATERNION_SLICE],
        )
        return aligned - reference

    def _measurement_residual(self, value: np.ndarray, reference: np.ndarray) -> np.ndarray:
        aligned = value.copy()
        aligned[self.QUATERNION_SLICE] = self._align_quaternion(
            aligned[self.QUATERNION_SLICE],
            reference[self.QUATERNION_SLICE],
        )
        return aligned - reference

    def _sanitize_augmented_state(
        self,
        state: np.ndarray,
        *,
        reference_quaternion: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        state = np.asarray(state, dtype=float).copy()
        fallback = reference_quaternion
        quaternion = self._normalize_quaternion(
            state[self.QUATERNION_SLICE],
            fallback=fallback,
        )
        if reference_quaternion is not None:
            quaternion = self._align_quaternion(quaternion, reference_quaternion)
        state[self.QUATERNION_SLICE] = quaternion
        state[self.THRUST_RATIO_INDEX] = np.clip(
            state[self.THRUST_RATIO_INDEX],
            self.thrust_ratio_min,
            self.thrust_ratio_max,
        )
        return state

    def _validate_measurement(self, measurement: np.ndarray) -> np.ndarray:
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        if measurement.shape != (self.MEASUREMENT_DIM,):
            raise ValueError(
                f'Expected 13-element measurement, got {measurement.shape}'
            )
        if not np.all(np.isfinite(measurement)):
            raise ValueError('Measurement contains NaN or Inf')
        return measurement

    def _project_covariance(self, covariance: np.ndarray) -> np.ndarray:
        covariance = np.asarray(covariance, dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, self.minimum_covariance_eigenvalue)
        projected = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        return 0.5 * (projected + projected.T)

    @staticmethod
    def _normalize_quaternion(
        quaternion: np.ndarray,
        fallback: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        quaternion = np.asarray(quaternion, dtype=float).reshape(4)
        norm = float(np.linalg.norm(quaternion))
        if norm > 1e-12:
            return quaternion / norm
        if fallback is not None:
            fallback = np.asarray(fallback, dtype=float).reshape(4)
            fallback_norm = float(np.linalg.norm(fallback))
            if fallback_norm > 1e-12:
                return fallback / fallback_norm
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    @classmethod
    def _align_quaternion(
        cls,
        quaternion: np.ndarray,
        reference: np.ndarray,
    ) -> np.ndarray:
        quaternion = cls._normalize_quaternion(quaternion, fallback=reference)
        reference = cls._normalize_quaternion(reference)
        return quaternion if float(np.dot(quaternion, reference)) >= 0.0 else -quaternion
