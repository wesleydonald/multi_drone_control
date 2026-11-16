"""Low-bandwidth lateral acceleration-disturbance observer.

The observer estimates a slowly varying XY acceleration mismatch between the
nominal payload-MPC translational model and measured motion.  It deliberately
uses measured velocity directly instead of differentiating mocap velocity.

For each horizontal axis the continuous observer is

    v_hat_dot = a_nom + d_hat + 2*w*(v_meas - v_hat)
    d_hat_dot = w^2*(v_meas - v_hat)

which places the nominal observer-error poles at ``-w`` (repeated).  Keeping
``w`` well below the suspended-load swing frequency makes this a trim/model-
mismatch estimator rather than a second anti-swing controller.
"""

from __future__ import annotations

import numpy as np


XY_BIAS_MODES = (
    "legacy_integral",
    "lateral_disturbance",
    "lateral_disturbance_shadow",
    "none",
)


def parse_xy_bias_mode(value: object) -> str:
    """Return one canonical mutually-exclusive XY bias-rejection mode."""
    mode = str(value).strip().lower()
    if mode not in XY_BIAS_MODES:
        valid = ", ".join(XY_BIAS_MODES)
        raise ValueError(f"Unknown xy_bias_mode {value!r}; expected one of: {valid}")
    return mode


def nominal_lateral_acceleration(
    quaternion_wxyz,
    throttle_state: float,
    thrust_ratio: float,
) -> np.ndarray:
    """Match the MPC's nominal world-frame XY thrust acceleration.

    The active ``QuadDynamics`` has no horizontal drag term, so only the world
    projection of body-Z thrust contributes to the nominal XY acceleration.
    """
    q = np.asarray(quaternion_wxyz, dtype=float).reshape(4)
    qw, qx, qy, qz = q
    thrust_accel = float(thrust_ratio) * float(throttle_state)
    return np.array(
        [
            thrust_accel * 2.0 * (qx * qz + qw * qy),
            thrust_accel * 2.0 * (qy * qz - qw * qx),
        ],
        dtype=float,
    )


class LateralDisturbanceObserver:
    """Second-order Luenberger observer for constant/slow XY acceleration bias."""

    def __init__(
        self,
        bandwidth_rad_s: float = 0.30,
        max_abs_disturbance_mps2: float = 2.0,
        max_dt_s: float = 0.15,
    ) -> None:
        self.bandwidth_rad_s = max(0.0, float(bandwidth_rad_s))
        self.max_abs_disturbance_mps2 = max(
            0.0, float(max_abs_disturbance_mps2)
        )
        self.max_dt_s = max(1e-6, float(max_dt_s))
        self.velocity_hat = np.zeros(2, dtype=float)
        self.disturbance_hat = np.zeros(2, dtype=float)
        self.innovation = np.zeros(2, dtype=float)
        self.initialized = False
        self.update_count = 0
        self.status = "reset"

    def reset(self, measured_velocity_xy=None, *, status: str = "reset") -> None:
        if measured_velocity_xy is None:
            self.velocity_hat[:] = 0.0
        else:
            velocity = np.asarray(measured_velocity_xy, dtype=float).reshape(2)
            if np.all(np.isfinite(velocity)):
                self.velocity_hat = velocity.copy()
            else:
                self.velocity_hat[:] = 0.0
        self.disturbance_hat[:] = 0.0
        self.innovation[:] = 0.0
        self.initialized = False
        self.update_count = 0
        self.status = str(status)

    def update(
        self,
        measured_velocity_xy,
        nominal_acceleration_xy,
        dt_s: float,
    ) -> dict:
        velocity = np.asarray(measured_velocity_xy, dtype=float).reshape(2)
        acceleration = np.asarray(nominal_acceleration_xy, dtype=float).reshape(2)
        dt = float(dt_s)

        if not np.all(np.isfinite(velocity)) or not np.all(np.isfinite(acceleration)):
            self.status = "invalid_measurement"
            return self.result(updated=False)

        if not np.isfinite(dt) or dt <= 0.0 or dt > self.max_dt_s:
            self.reset(velocity, status="invalid_dt_reset")
            return self.result(updated=False)

        if not self.initialized:
            self.velocity_hat = velocity.copy()
            self.disturbance_hat[:] = 0.0
            self.innovation[:] = 0.0
            self.initialized = True
            self.status = "initialized"
            return self.result(updated=False)

        self.innovation = velocity - self.velocity_hat
        w = self.bandwidth_rad_s
        self.velocity_hat = self.velocity_hat + dt * (
            acceleration
            + self.disturbance_hat
            + (2.0 * w) * self.innovation
        )
        self.disturbance_hat = self.disturbance_hat + dt * (
            (w * w) * self.innovation
        )

        if self.max_abs_disturbance_mps2 > 0.0:
            self.disturbance_hat = np.clip(
                self.disturbance_hat,
                -self.max_abs_disturbance_mps2,
                self.max_abs_disturbance_mps2,
            )

        self.update_count += 1
        self.status = "tracking"
        return self.result(updated=True)

    def result(self, *, updated: bool) -> dict:
        return {
            "status": self.status,
            "initialized": bool(self.initialized),
            "updated": bool(updated),
            "disturbance_hat": self.disturbance_hat.copy(),
            "velocity_hat": self.velocity_hat.copy(),
            "innovation": self.innovation.copy(),
            "innovation_norm": float(np.linalg.norm(self.innovation)),
            "update_count": int(self.update_count),
        }
