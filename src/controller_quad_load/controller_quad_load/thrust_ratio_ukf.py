"""Augmented-state parameter UKF for the cable-aware tracker -- the controller_ukf method.

This is the estimator controller_ukf/main.py uses, ported to the quad_load model. The
filter state is the drone's physical state AUGMENTED with the six dynamics parameters:

    x = [ p(3), q(4), v(3), omega(3) | kT, drag_z, tau_rate, centre_deg, max_deg, expo ]
        \________ 13 measured ______/  \________ 6 estimated parameters ________/

Each of the 2n+1 = 39 sigma points is propagated through the REAL nonlinear model (the
acados sim integrator built from the same expression the MPC uses) with that sigma
point's own parameter values, then corrected against the mocap pose. Parameters are held
constant through the prediction, so the only thing that can explain a mismatch between
predicted and measured motion is the parameters -- which is what identifies them.

WHY THIS RATHER THAN THE IMU/CABLE DECOMPOSITION
    The IMU estimator formed kT algebraically, kT = (a_imu_z - a_cable_z) / throttle, so
    an error in the modelled cable tension landed on kT amplified by 1/throttle (~4x at
    hover) on every single sample. Here the cable term enters as a parameter of a
    one-step prediction that is then corrected against mocap, and the innovation is
    distributed across 19 states by the Kalman gain with the parameters' small process
    noise deliberately making them the SLOW states. A noisy or biased tension therefore
    perturbs the estimate instead of driving it.

    It does NOT remove the dependence on a_cable: a thrust estimator cannot ignore the
    other large force on the drone. What changes is that the tension enters through the
    dynamics with statistical weighting, rather than by direct subtraction.

COST
    39 integrator calls per update, so this is genuinely expensive -- see
    controller_mpc_payload, whose equivalent backend warns about exactly this. Run it at
    ukf_rate_hz below the control rate if the control loop starts missing its deadline;
    the prediction step integrates the actual elapsed interval either way.
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np
from scipy.linalg import cholesky

# Augmented layout.
N_POSE = 13          # p(3), q(4), v(3), omega(3)
N_PARAM = 6          # kT, drag_z, tau_rate, centre_deg, max_deg, expo
N_STATE = N_POSE + N_PARAM

# Parameter bounds, same intent as controller_ukf/main.py's post-update clip. kT's floor
# is the important one: below ~12 is not an airframe, it is a broken estimate, and acting
# on it commands an enormous throttle.
PARAM_MIN = np.array([12.0, 0.00, 0.07, 0.0, 0.0, 0.5])
PARAM_MAX = np.array([60.0, 1.00, 0.30, 1000.0, 1000.0, 0.5])


class ThrustRatioUKF:
    """Joint state+parameter UKF over the quad_load model."""

    def __init__(
        self,
        *,
        initial_params: np.ndarray,
        alpha: float = 0.1,
        beta: float = 2.0,
        kappa: float = 0.0,
        param_process_std: Optional[np.ndarray] = None,
        measurement_std: float = 0.05,
        param_min: Optional[np.ndarray] = None,
        param_max: Optional[np.ndarray] = None,
    ) -> None:
        self.initial_params = np.asarray(initial_params, dtype=float).copy()
        if self.initial_params.shape != (N_PARAM,):
            raise ValueError(f'expected {N_PARAM} parameters, got {self.initial_params.shape}')
        self.alpha, self.beta, self.kappa = float(alpha), float(beta), float(kappa)
        self.param_min = (PARAM_MIN if param_min is None
                          else np.asarray(param_min, dtype=float))
        self.param_max = (PARAM_MAX if param_max is None
                          else np.asarray(param_max, dtype=float))

        # Process noise. The pose block matches controller_ukf; the parameter block is
        # what sets how fast the parameters may move. Small = slow, trusted parameters.
        if param_process_std is None:
            param_q = np.array([1e-4, 1e-5, 1e-5, 1.0, 1.0, 0.1])
        else:
            param_q = np.asarray(param_process_std, dtype=float) ** 2
        self.Q = np.diag(np.concatenate([
            np.array([1e-4, 1e-4, 1e-4]),               # position
            np.array([1e-5, 1e-5, 1e-5, 1e-5]),         # quaternion
            np.array([1e-3, 1e-3, 1e-3]),               # velocity
            np.array([1e-3, 1e-3, 1e-3]),               # body rates
            param_q,
        ]))
        self.R = np.diag([float(measurement_std)] * N_POSE)

        self.x = np.zeros(N_STATE)
        self.x[3] = 1.0                                  # identity quaternion
        self.x[N_POSE:] = self.initial_params
        self.P = np.diag(np.full(N_STATE, 0.1))

        self.initialized = False
        self.update_count = 0
        self.reject_count = 0
        self.last_innovation_norm = float('nan')
        self.last_status = 'not_initialized'

    # ── lifecycle ─────────────────────────────────────────────────────────

    @property
    def params(self) -> np.ndarray:
        return self.x[N_POSE:].copy()

    @property
    def thrust_ratio(self) -> float:
        return float(self.x[N_POSE])

    def reset(self) -> None:
        self.x = np.zeros(N_STATE)
        self.x[3] = 1.0
        self.x[N_POSE:] = self.initial_params
        self.P = np.diag(np.full(N_STATE, 0.1))
        self.initialized = False
        self.update_count = 0
        self.reject_count = 0
        self.last_innovation_norm = float('nan')
        self.last_status = 'not_initialized'

    def initialize(self, pose: np.ndarray) -> None:
        """Full (re)start: seed the pose block from the measurement AND return the
        parameters to their seed. Use only when the learned parameters should be
        discarded -- e.g. the fleet has landed and re-armed.

        Seeding the pose matters: without it the filter spends its first updates
        chasing an arbitrary zero state, and that transient lands in the parameters.
        """
        pose = np.asarray(pose, dtype=float).reshape(-1)[:N_POSE]
        self.x[:N_POSE] = pose
        self.x[N_POSE:] = self.initial_params
        self.P = np.diag(np.full(N_STATE, 0.1))
        self.initialized = True
        self.update_count = 0
        self.last_status = 'initialized'

    def seed_pose(self, pose: np.ndarray) -> None:
        """Re-seed ONLY the pose block, keeping the learned parameters and their
        covariance.

        Used whenever the filter is being held (not airborne, no valid control) and
        the pose would otherwise go stale. It must NOT touch the parameters: doing so
        means any momentary dip below the airborne gate throws away everything learned
        so far and restarts kT from its seed -- which is exactly what happened when
        this path called initialize(), and it left kT parked at the seed value for a
        whole flight whenever the drone oscillated across the threshold.
        """
        if not self.initialized:
            self.initialize(pose)
            return
        pose = np.asarray(pose, dtype=float).reshape(-1)[:N_POSE]
        self.x[:N_POSE] = pose
        # Pose uncertainty resets (we just measured it) and its correlation with the
        # parameters is dropped, but the parameter block is left untouched.
        self.P[:N_POSE, :] = 0.0
        self.P[:, :N_POSE] = 0.0
        self.P[:N_POSE, :N_POSE] = np.diag(np.full(N_POSE, 0.1))
        self.last_status = 'pose_reseeded'

    # ── one filter step ───────────────────────────────────────────────────

    def update(self, measurement: np.ndarray, propagate) -> Dict[str, object]:
        """Predict through `propagate` then correct against `measurement`.

        Args:
            measurement: measured pose, 13 elements [p, q, v, omega].
            propagate: callable(pose13, params6) -> next pose13, which integrates the
                real model one step with that sigma point's parameters. The caller owns
                the applied control and cable term, so the filter stays model-agnostic.
        """
        if not self.initialized:
            raise RuntimeError('initialize() before update()')

        measurement = np.asarray(measurement, dtype=float).reshape(-1)[:N_POSE]
        if not np.all(np.isfinite(measurement)):
            self.last_status = 'bad_measurement'
            return self.snapshot()

        sigma, wm, wc = self._sigma_points()

        propagated = np.empty_like(sigma)
        for i in range(sigma.shape[0]):
            params = np.clip(sigma[i, N_POSE:], self.param_min, self.param_max)
            nxt = propagate(sigma[i, :N_POSE], params)
            if nxt is None or not np.all(np.isfinite(nxt)):
                self.last_status = 'propagation_failed'
                return self.snapshot()
            # Parameters are FROZEN across the prediction (random-walk only, via Q).
            # That is what makes a prediction/measurement mismatch attributable to them.
            propagated[i, :N_POSE] = np.asarray(nxt, dtype=float)[:N_POSE]
            propagated[i, N_POSE:] = params

        x_pred, P_pred = self._unscented(propagated, wm, wc, self.Q)

        # Measurement model is the identity on the pose block (mocap measures it all).
        sigma_meas = propagated[:, :N_POSE]
        z_pred, P_zz = self._unscented(sigma_meas, wm, wc, self.R)

        P_xz = np.zeros((N_STATE, N_POSE))
        for i in range(propagated.shape[0]):
            P_xz += wc[i] * np.outer(propagated[i] - x_pred, sigma_meas[i] - z_pred)

        try:
            gain = np.linalg.solve(P_zz.T, P_xz.T).T
        except np.linalg.LinAlgError:
            self.last_status = 'singular_innovation'
            return self.snapshot()

        innovation = measurement - z_pred
        self.last_innovation_norm = float(np.linalg.norm(innovation))

        self.x = x_pred + gain @ innovation
        self.P = P_pred - gain @ P_zz @ gain.T

        # Keep P symmetric positive definite -- the sigma-point Cholesky needs it.
        self.P = 0.5 * (self.P + self.P.T)
        if np.min(np.linalg.eigvalsh(self.P)) < 1e-8:
            self.P += np.eye(N_STATE) * 1e-5

        norm = float(np.linalg.norm(self.x[3:7]))
        if norm > 1e-9:
            self.x[3:7] /= norm

        self.x[N_POSE:] = np.clip(self.x[N_POSE:], self.param_min, self.param_max)
        # centre_rate_deg must not exceed max_rate_deg (same guard as controller_ukf).
        if self.x[N_POSE + 4] < self.x[N_POSE + 3]:
            self.x[N_POSE + 4] = self.x[N_POSE + 3]

        self.update_count += 1
        self.last_status = 'updated'
        return self.snapshot()

    # ── UKF internals (Julier scaled sigma points, as controller_ukf) ──────

    def _sigma_points(self):
        n = N_STATE
        lam = self.alpha ** 2 * (n + self.kappa) - n
        P = self.P + np.eye(n) * 1e-9
        try:
            sqrt_P = cholesky((n + lam) * P, lower=True)
        except np.linalg.LinAlgError:
            vals, vecs = np.linalg.eigh((n + lam) * P)
            sqrt_P = vecs @ np.diag(np.sqrt(np.maximum(vals, 1e-9)))

        pts = np.empty((2 * n + 1, n))
        pts[0] = self.x
        for i in range(n):
            pts[1 + 2 * i] = self.x + sqrt_P[:, i]
            pts[2 + 2 * i] = self.x - sqrt_P[:, i]

        wm = np.full(2 * n + 1, 1.0 / (2.0 * (n + lam)))
        wc = wm.copy()
        wm[0] = lam / (n + lam)
        wc[0] = lam / (n + lam) + (1.0 - self.alpha ** 2 + self.beta)
        return pts, wm, wc

    @staticmethod
    def _unscented(points, wm, wc, noise):
        mean = np.sum(wm[:, None] * points, axis=0)
        d = points - mean
        cov = (wc[:, None, None] * d[:, :, None] * d[:, None, :]).sum(axis=0)
        return mean, cov + noise

    def snapshot(self) -> Dict[str, object]:
        return {
            'status': self.last_status,
            'initialized': self.initialized,
            'thrust_ratio': self.thrust_ratio,
            'thrust_ratio_std': float(np.sqrt(max(self.P[N_POSE, N_POSE], 0.0))),
            'params': self.params,
            'innovation_norm': float(self.last_innovation_norm),
            'update_count': int(self.update_count),
        }
