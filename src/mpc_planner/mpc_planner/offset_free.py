"""offset_free.py -- offset-free MPC disturbance observer (card
docs/experiments/2026-10-04_offset_free_innovation.md).

The load model is augmented with an integrating force, m v_dot = f(x, u) + d, d_dot = 0 (Muske &
Badgwell 2002; Pannocchia & Rawlings 2003; Maeder, Borrelli & Morari 2009). d is estimated from the
planner's own prediction error: what a published horizon predicted the load would do, against what
it did. The horizon already contains the estimate it was solved with, so the innovation is what is
still unexplained, whatever its cause: an external force, a mass or map error, or trackers that do
not execute the plan. Integral action through the planner's model (critic, 4 Oct). Pure: no ROS.

Two innovations:
  window 0   one step: the load velocity predicted for now by the last horizon. Converged, the
             plan's first move is zero; a plan that starts a move in the cable states reads zero
             too, so it can settle off the reference (SIL R0875: 12.5 cm low, R0878: -2.9 cm).
  window W   the load position predicted for now by the horizon solved W s ago, against the
             measured one: d_hat += (dt / tau) (2 m dp / T^2 - (d_hat - d_hat_then)). Converged,
             the plan predicts no load motion over W, which off the reference it always does.
"""
from collections import deque

import numpy as np


def predicted_at(t_plan, dt, Y, t):
    """Y (N+1, 3) along a horizon solved at t_plan with node spacing dt, at time t; None outside."""
    s = (float(t) - float(t_plan)) / float(dt)
    if s < 0.0 or s > Y.shape[0] - 1:
        return None
    k = min(int(s), Y.shape[0] - 2)
    a = s - k
    return (1.0 - a) * Y[k] + a * Y[k + 1]


predicted_velocity = predicted_at


class OffsetFreeObserver:
    """First-order observer of time constant tau on the unexplained load force. Frozen while not
    gated (the first gated tick after a freeze is not integrated); bounded per axis."""

    def __init__(self, mass, tau=2.0, bound=3.0, max_gap=0.3, window=0.0):
        self.m = float(mass)
        self.tau = float(tau)
        self.bound = float(bound)
        self.max_gap = float(max_gap)    # s; older than this past its window is not a prediction
        self.window = float(window)
        self.reset()

    def reset(self):
        self.d = np.zeros(3)
        self.raw = None                  # last unexplained force (N)
        self.railed = False
        self._plans = deque()            # (t_plan, dt, P (N+1, 3), V (N+1, 3), d_hat then)
        self._was_gated = False
        self._t_last = None

    def set_plan(self, t_plan, dt, V, P=None):
        """The horizon just published: its solve time, node spacing, load velocities (and
        positions, for a window > 0)."""
        V = np.asarray(V, float).reshape(-1, 3).copy()
        P = None if P is None else np.asarray(P, float).reshape(-1, 3).copy()
        if self.window <= 0.0:
            self._plans.clear()
        self._plans.append((float(t_plan), float(dt), P, V, self.d.copy()))
        while self._plans and self._plans[0][0] < float(t_plan) - self.window - self.max_gap - 0.5:
            self._plans.popleft()

    def drop_plan(self):
        """No horizon was published this tick (failed solve, fallback). One step: nothing to
        compare against next tick. A window keeps the older horizons (the trackers fly a shifted
        one through the gap)."""
        if self.window <= 0.0:
            self._plans.clear()

    def _residual(self, t, v_meas, p_meas):
        if self.window <= 0.0:
            if not self._plans:
                return None, None
            t_plan, dt, _, V, _ = self._plans.pop()
            gap = float(t) - t_plan
            if gap <= 1e-6 or gap > self.max_gap:
                return None, None
            v_pred = predicted_at(t_plan, dt, V, t)
            if v_pred is None:
                return None, None
            return self.m * (np.asarray(v_meas, float) - v_pred) / gap, gap
        old = [p for p in self._plans if p[0] <= float(t) - self.window + 1e-6]
        if not old or p_meas is None:
            return None, None
        t_plan, dt, P, _, d_then = old[-1]
        T = float(t) - t_plan
        if P is None or T > self.window + self.max_gap:
            return None, None
        p_pred = predicted_at(t_plan, dt, P, t)
        if p_pred is None:
            return None, None
        r = 2.0 * self.m * (np.asarray(p_meas, float) - p_pred) / (T * T) - (self.d - d_then)
        gap = float(t) - self._t_last if self._t_last is not None else 0.0
        return r, min(max(gap, 0.0), self.max_gap)

    def update(self, t, gated, v_meas, p_meas=None):
        """One tick at time t with the measured load velocity (and position). Returns d_hat."""
        self.raw, gap = self._residual(t, v_meas, p_meas)
        self._t_last = float(t)
        if self.raw is None:
            return self.d
        first, self._was_gated = gated and not self._was_gated, gated
        if gated and not first and gap > 0.0:   # the first tick after a freeze is not integrated
            d = self.d + min(gap / self.tau, 1.0) * self.raw
            self.railed = bool(np.any(np.abs(d) >= self.bound))
            self.d = np.clip(d, -self.bound, self.bound)
        return self.d
