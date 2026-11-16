"""tilt_cut.py -- QUAD4's tilt fault while another controller may be flying it (card 2026-10-11_quad4_tilt_cut).

A pure rule, no ROS: the tracker feeds it every fresh pose, the offline replay (tools/tilt_replay.py) feeds it a
bag. It runs from TAKEOFF until the next ARM, whatever the tracker's own armed flag and envelope latch say: a
disarm or an earlier speed fault clears those on the same tick (critic round 1, C1).

  * cut: tilt above cut_deg held for cut_hold_s (time, not poses: the mocap rate and its gaps vary);
  * slow (optional, slow_deg > 0): tilt above slow_deg held for slow_s;
  * an attitude change faster than glitch_rate_dps between two poses is a mocap mis-solve, not a body: no count, and
    nothing counts for glitch_holdoff_s after it. A rate, not a step: a real tumble across a mocap gap is a large
    step at a body's rate and is counted (critic round 2, M-A).
"""
from dataclasses import dataclass
import math
from typing import Optional

from utility_objects.safety import tilt_deg_from_quat


def attitude_step_deg(q0, q1):
    """Rotation angle between two attitudes, degrees (q = w, x, y, z; sign-invariant)."""
    a = [float(v) for v in q0]
    b = [float(v) for v in q1]
    na = math.sqrt(sum(v * v for v in a))
    nb = math.sqrt(sum(v * v for v in b))
    if na < 1e-9 or nb < 1e-9:
        return float('nan')
    dot = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


MIN_DT_S = 1e-3      # two poses closer than this (or out of order) are rated over 1 ms


@dataclass
class TiltCutConfig:
    cut_deg: float = 60.0
    cut_hold_s: float = 0.025      # three 120 Hz pose intervals
    glitch_rate_dps: float = 3000.0
    glitch_holdoff_s: float = 0.5
    slow_deg: float = 0.0          # 0 = off
    slow_s: float = 0.5

    def __post_init__(self):
        if not (0.0 < self.cut_deg < 180.0) or self.cut_hold_s < 0.0 or self.glitch_rate_dps <= 0.0:
            raise ValueError('tilt cut: cut_deg in (0, 180), cut_hold_s >= 0, glitch_rate_dps > 0')
        if self.glitch_holdoff_s < 0.0 or self.slow_deg < 0.0 or self.slow_s < 0.0:
            raise ValueError('tilt cut: holdoff, slow_deg and slow_s must be >= 0')


class TiltCutRule:
    """arm() at ARM, takeoff() at TAKEOFF, step(t, q) on every fresh pose: returns 'tilt' or 'slow' once, when the
    fault latches, else None."""

    def __init__(self, config: Optional[TiltCutConfig] = None):
        self.config = config or TiltCutConfig()
        self.arm()

    def arm(self):
        """A new flight: nothing latched, not flying."""
        self.flying = False
        self.fault = None
        self.fault_tilt = math.nan
        self.max_tilt = 0.0
        self.glitches = 0
        self.max_rate = 0.0
        self._reset_counts()
        self._q_prev = None
        self._t_prev = None

    def takeoff(self):
        self.flying = True

    def _reset_counts(self):
        self._above_t0 = None
        self._slow_t0 = None
        self._holdoff_until = -math.inf

    def step(self, t: float, q) -> Optional[str]:
        t = float(t)
        prev, self._q_prev = self._q_prev, q
        t_prev, self._t_prev = self._t_prev, t
        tilt = tilt_deg_from_quat(q)
        if not self.flying or self.fault is not None or math.isnan(tilt):
            return None
        self.max_tilt = max(self.max_tilt, tilt)
        cfg = self.config
        if prev is not None:
            rate = attitude_step_deg(prev, q) / max(t - t_prev, MIN_DT_S)
            if not math.isnan(rate):
                self.max_rate = max(self.max_rate, rate)
                if rate > cfg.glitch_rate_dps:
                    self.glitches += 1
                    self._reset_counts()
                    self._holdoff_until = t + cfg.glitch_holdoff_s
                    return None
        if t < self._holdoff_until:
            return None
        if tilt > cfg.cut_deg:
            self._above_t0 = t if self._above_t0 is None else self._above_t0
            if t - self._above_t0 >= cfg.cut_hold_s - 1e-6:
                return self._latch('tilt', tilt)
        else:
            self._above_t0 = None
        if cfg.slow_deg > 0.0:
            if tilt > cfg.slow_deg:
                self._slow_t0 = t if self._slow_t0 is None else self._slow_t0
                if t - self._slow_t0 >= cfg.slow_s - 1e-9:
                    return self._latch('slow', tilt)
            else:
                self._slow_t0 = None
        return None

    def _latch(self, why, tilt):
        self.fault = why
        self.fault_tilt = tilt
        return why
