"""lumped_integral.py -- the ring position integral as a force in the planner's model (card
docs/experiments/2026-10-04_z_int_model.md).

Integral control routed through the model's disturbance input: the bounded integral b of the ring
position error (m, per axis, as ZBias) is applied to the planner's model as a constant force on the
ring, d = -k * b (N), instead of being added to the reference. d is the lumped constant disturbance:
everything the model gets wrong about the ring's force balance, real or not (fictitious for an
internal error, M-hautus). Any stable equilibrium with the gate open and b off its bound has zero
position error, whatever the constant cause (internal model principle, Francis & Wonham 1976).
Pure: no ROS.
"""
import numpy as np


class LumpedForceIntegral:
    """b_dot = ki * (ref - measured) per axis while that axis is gated; |b| <= i_max; d = -k * b."""

    def __init__(self, ki, i_max, hz, k_xy, k_z):
        self.ki = float(ki)
        self.i_max = abs(float(i_max))
        self.hz = float(hz)
        self.k = np.array([float(k_xy), float(k_xy), float(k_z)])
        self.reset()

    def reset(self):
        self.b = np.zeros(3)
        self.n_updates = np.zeros(3, dtype=int)

    @property
    def near_bound(self):
        """Per axis: the fault-masking guard, as ZBias (70 % of the bound)."""
        return np.abs(self.b) > 0.7 * self.i_max

    @property
    def force_bound(self):
        return self.k * self.i_max

    def update(self, err, gated_xy, gated_z):
        """err = reference - measured ring position (3,). Gated axes integrate, the rest hold."""
        gate = np.array([gated_xy, gated_xy, gated_z], dtype=bool)
        if self.ki > 0.0 and gate.any():
            step = self.ki * np.asarray(err, float).reshape(3) / self.hz
            self.b = np.where(gate, np.clip(self.b + step, -self.i_max, self.i_max), self.b)
            self.n_updates += gate
        return self.b

    def force(self, down=False):
        """The force on the ring the plan uses (N, world); zero with the ring down."""
        return np.zeros(3) if down else -self.k * self.b
