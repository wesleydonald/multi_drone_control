"""Linear velocity from successive mocap positions (card docs/experiments/2026-10-14_tracker_timing.md).

'ema'   the rig's filter since the start: the frame difference, then v = a*v_new + (1 - a)*v_prev. At ~90 Hz and
        a = 0.2 it lags ~(1 - a)/a = 4 frames (~45 ms), plus half a frame for the difference.
'slope' a least-squares slope over the last `frames` positions: ~(frames - 1)/2 frames of lag (1.5 frames, ~17 ms,
        for 4), noisier than 'ema'.
'none'  the plain frame difference (the twin's emulator).
"""
from collections import deque

import numpy as np

MODES = ('ema', 'slope', 'none')


class VelocityFilter:
    def __init__(self, mode='ema', alpha=0.2, frames=4):
        if mode not in MODES:
            raise ValueError(f'velocity filter must be one of {MODES}, got {mode!r}')
        if not 0.0 < float(alpha) <= 1.0 or int(frames) < 2:
            raise ValueError('velocity filter: 0 < alpha <= 1 and frames >= 2')
        self.mode, self.alpha = mode, float(alpha)
        self._hist = deque(maxlen=int(frames))
        self._last = None
        self._v = None

    def update(self, t, p):
        """Velocity (3,) after the sample at time t (s) and position p (3,); None for the first sample or t not
        after the last one."""
        p = np.asarray(p, dtype=float).reshape(3)
        if self._last is not None and t <= self._last[0]:
            return None
        prev, self._last = self._last, (float(t), p)
        self._hist.append((float(t), p))
        if prev is None:
            return None
        diff = (p - prev[1]) / (float(t) - prev[0])
        if self.mode == 'none':
            return diff
        if self.mode == 'ema':
            self._v = diff if self._v is None else self.alpha * diff + (1.0 - self.alpha) * self._v
            return self._v
        ts = np.array([h[0] for h in self._hist])
        ps = np.array([h[1] for h in self._hist])
        dt = ts - ts.mean()
        den = float(dt @ dt)
        return diff if den <= 0.0 else (dt @ (ps - ps.mean(axis=0))) / den
