"""rod_reread.py -- each rod's length read under the pretension's held pull (card docs/experiments/2026-10-31_rod_len_reread.md).

The planner reads the rods at hand-over, with their pull still near zero, and on 12 real 1 m flights they then read 5.7 mm longer
on average (-7 to +16). The plan places each drone at `ring + attach - l s`, so a short l is a steady position command toward
the ring, and the ring rides about 4 x l low. The last 0.5 s of the pretension's hold reads what the flight then flies (-0.8 mm mean, SD 2.4,
the same flights). This reads them there and ramps to it as the lift starts. Once. Pure: no ROS.
"""
import numpy as np

RAMP_S = 0.5           # to the new lengths: a step would step every drone's reference along its rod
MAX_SPREAD_M = 0.015   # one rod's samples in the hold: a wider spread is a glitch or a swing, not a length
MAX_LONGER_M = 0.020   # seen +16 mm; a larger change is not this effect: keep the hand-over's lengths and say so
MAX_SHORTER_M = 0.008  # seen -6 mm


class RodReread:
    """sample() each tick of the hold, finish() as the hold ends, step() each tick after it."""

    def __init__(self, ramp_s=RAMP_S, max_longer_m=MAX_LONGER_M, max_shorter_m=MAX_SHORTER_M, max_spread_m=MAX_SPREAD_M):
        self.ramp_s = float(ramp_s)
        self.max_longer_m = float(max_longer_m)
        self.max_shorter_m = float(max_shorter_m)
        self.max_spread_m = float(max_spread_m)
        self.state = 'read'
        self.why = None
        self._samples = []
        self._t0 = None
        self._from = None
        self._to = None

    @property
    def target(self):
        return None if self._to is None else list(self._to)

    @property
    def done(self):
        return self.state in ('done', 'refused', 'cancelled')

    def cancel(self, why):
        """Before the ramp ended (a resize, a descent): the lengths stay where they are."""
        if not self.done:
            self.state = 'cancelled'
            self.why = why

    def sample(self, dists):
        """dists: |pivot - attach point| per rod (m), one tick of the hold."""
        if self.state == 'read':
            self._samples.append(np.asarray(dists, float))

    def finish(self, now, lengths):
        """The hold ended: the median of the samples, within the limits, starts the ramp from the lengths flown now."""
        if self.state != 'read':
            return
        frm = np.asarray(lengths, float)
        if not self._samples or self._samples[0].shape != frm.shape:
            self.cancel('no reading in the hold')
            return
        samples = np.asarray(self._samples)
        to = np.median(samples, axis=0)
        spread = samples.max(axis=0) - samples.min(axis=0)
        if spread.max() > self.max_spread_m:
            self.state = 'refused'
            self.why = (f'the hold spread {np.round(spread * 1e3, 1).tolist()} mm, over {self.max_spread_m * 1e3:.0f} mm: '
                        f'lengths {np.round(frm, 3).tolist()} m kept')
            return
        d = to - frm
        if d.max() > self.max_longer_m or d.min() < -self.max_shorter_m:
            self.state = 'refused'
            self.why = (f'lengths {np.round(frm, 3).tolist()} m read {np.round(to, 3).tolist()} m, outside '
                        f'+{self.max_longer_m * 1e3:.0f} / -{self.max_shorter_m * 1e3:.0f} mm: kept')
            return
        self.state, self._t0, self._from, self._to = 'ramp', float(now), frm, to

    def step(self, now):
        """The lengths to fly this tick during the ramp, else None."""
        if self.state != 'ramp':
            return None
        f = min((now - self._t0) / self.ramp_s, 1.0) if self.ramp_s > 0.0 else 1.0
        if f >= 1.0:
            self.state = 'done'
        return list(self._from + f * (self._to - self._from))
