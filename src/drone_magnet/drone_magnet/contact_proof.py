"""The contact-and-lift attach proof (card 2026-10-10_combined_m1_contact, Goal 2), without ROS.

A contact is only a candidate. The catch is proven when the tip keeps the tip-to-plate vector it
touched with while the plate itself rises (Tejen's PickupProofTracker pattern,
tejen_mission/m1_pickup_proof.py): a tip that rises alone (no catch) drifts, and a tip resting on or
sunk into the plate never sees the plate rise.

The caller feeds one sample per tip pose on its own clock (sim time in the twin) and calls tick()
regularly so the timeout runs without samples.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

OFF, WATCH, PROVE, ATTACHED = 'off', 'watch', 'prove', 'attached'
_EPS = 1e-9


@dataclass(frozen=True)
class ProofConfig:
    radius: float = 0.01            # m, tip to plate point (the manager's attach_radius)
    speed: float = 0.05             # m/s, tip relative to the plate (attach_speed_threshold)
    contact_dwell_s: float = 0.15
    d_ok: float = 0.003
    d_fail: float = 0.008
    h_plate: float = 0.003
    proof_dwell_s: float = 0.3
    fail_dwell_s: float = 0.12
    timeout_s: float = 3.0
    taut_len: float = 0.0           # m, body to tip; 0 = not checked (the twin's rod is rigid)
    ma_s: float = 0.1
    # contact_still (critic F7, off = 0): c0 and z0 from the dwell's last c0_tail_s only (not its whole window, which
    # still holds the arrival), and the contact also needs the tip's VERTICAL speed relative to the plate <= vz_max
    c0_tail_s: float = 0.0
    vz_max: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(float(v)) and float(v) >= 0.0 for v in self.__dict__.values()):
            raise ValueError('contact proof values must be finite and non-negative')
        if self.d_fail <= self.d_ok:
            raise ValueError('proof_d_fail must exceed proof_d_ok')
        if self.ma_s <= 0.0 or self.contact_dwell_s <= 0.0:
            raise ValueError('proof_ma_s and contact_dwell_s must be positive')


def slope_speed(t, r) -> float:
    """|least-squares velocity| of the vectors r over the times t (nan below two samples). A
    per-sample difference of a 0.5 mm-noise tip at 120 Hz is ~85 mm/s of noise per axis."""
    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        return math.nan
    dt = t - t.mean()
    den = float(dt @ dt)
    if den <= 0.0:
        return math.nan
    r = np.asarray(r, dtype=float)
    return float(np.linalg.norm(dt @ (r - r.mean(axis=0)) / den))


class ContactProof:
    """OFF -> (magnet ON) WATCH -> (contact) PROVE -> (proven) ATTACHED, or PROVE -> (failed) WATCH.
    ATTACHED is terminal: a magnet OFF is refused and a later separation changes nothing."""

    def __init__(self, config: ProofConfig) -> None:
        self.config = config
        self.state = OFF
        self.tries = 0
        self.fails = 0
        self.reason = ''
        self.last_fail: Optional[dict] = None
        self.distance = math.nan
        self.rel_speed = math.nan
        self.rel_vz = math.nan
        self.body_to_tip = math.nan
        self._win = deque()       # (t, tip-to-plate, plate z, body-to-tip) over the last ma_s
        self._near = deque()      # (t, tip-to-plate, plate z) while the tip stays within the radius
        self._last_t: Optional[float] = None
        self._forget_contact()

    def _forget_contact(self) -> None:
        self.c0: Optional[np.ndarray] = None
        self.z0: Optional[float] = None
        self.t_contact: Optional[float] = None
        self._ok_t0: Optional[float] = None
        self._bad_t0: Optional[float] = None
        self._near.clear()
        self.drift = math.nan
        self.rise = math.nan

    def _forget_samples(self) -> None:
        """No dwell spans stale data or a gap."""
        self._win.clear()
        self._near.clear()
        self._last_t = None
        self._ok_t0 = None
        self._bad_t0 = None

    def magnet(self, on: bool) -> Optional[str]:
        """A magnet command: 'on' / 'off' on a change, 'reset' for an OFF during a proof (the contact
        is void), 'refused' for an OFF after the attach (the magnet stays ON), else None."""
        if on:
            if self.state != OFF:
                return None
            self.state = WATCH
            return 'on'
        if self.state == ATTACHED:
            return 'refused'
        if self.state == OFF:
            return None
        was = self.state
        self.state = OFF
        self._forget_contact()
        self._forget_samples()
        return 'reset' if was == PROVE else 'off'

    def tick(self, now: float) -> Optional[str]:
        """The proof's timeout, which also runs without samples: 'failed' or None."""
        if self.state == PROVE and now - self.t_contact >= self.config.timeout_s - _EPS:
            return self._fail('timeout', now)
        return None

    def step(self, now: float, tip, plate, body=None, fresh: bool = True) -> Optional[str]:
        """One tip sample against the latest plate point (and body, for the taut check):
        'contact', 'proven', 'failed' or None."""
        cfg = self.config
        if self.state == OFF:
            return None
        now = float(now)
        if not fresh or tip is None or plate is None:
            self._forget_samples()
            return self.tick(now)
        if self._last_t is not None:
            if now <= self._last_t:
                return None
            if now - self._last_t > cfg.ma_s:
                self._forget_samples()
        self._last_t = now
        tip = np.asarray(tip, dtype=float).reshape(3)
        plate = np.asarray(plate, dtype=float).reshape(3)
        rel = tip - plate
        self.distance = float(np.linalg.norm(rel))
        self.body_to_tip = (math.nan if body is None
                            else float(np.linalg.norm(tip - np.asarray(body, dtype=float).reshape(3))))
        self._win.append((now, rel, float(plate[2]), self.body_to_tip))
        while self._win[0][0] < now - cfg.ma_s + _EPS:
            self._win.popleft()
        if self.state == WATCH:
            return self._watch(now, rel, float(plate[2]))

        ma_rel = np.mean([w[1] for w in self._win], axis=0)
        self.drift = float(np.linalg.norm(ma_rel - self.c0))
        self.rise = float(np.mean([w[2] for w in self._win]) - self.z0)
        if self.state == ATTACHED:
            return None
        taut = cfg.taut_len <= 0.0 or float(np.mean([w[3] for w in self._win])) >= cfg.taut_len
        ok = self.drift <= cfg.d_ok and self.rise >= cfg.h_plate and taut
        bad = self.drift >= cfg.d_fail
        self._ok_t0 = (now if self._ok_t0 is None else self._ok_t0) if ok else None
        self._bad_t0 = (now if self._bad_t0 is None else self._bad_t0) if bad else None
        if bad and now - self._bad_t0 >= cfg.fail_dwell_s - _EPS:
            return self._fail('drift', now)
        if ok and now - self._ok_t0 >= cfg.proof_dwell_s - _EPS:
            self.state = ATTACHED
            return 'proven'
        return self.tick(now)

    def _watch(self, now: float, rel: np.ndarray, plate_z: float) -> Optional[str]:
        cfg = self.config
        if self.distance > cfg.radius:
            self._near.clear()
            self.rel_speed = math.nan
            return None
        self._near.append((now, rel, plate_z))
        # keep the newest sample at or before now - dwell, so the window spans the dwell
        while len(self._near) >= 2 and self._near[1][0] <= now - cfg.contact_dwell_s + _EPS:
            self._near.popleft()
        t = [n[0] for n in self._near]
        rels = np.array([n[1] for n in self._near])
        self.rel_speed = slope_speed(t, rels)
        still = True
        if cfg.vz_max > 0.0:
            self.rel_vz = slope_speed(t, rels[:, 2:3])
            still = self.rel_vz <= cfg.vz_max
        if now - t[0] < cfg.contact_dwell_s - _EPS or not self.rel_speed <= cfg.speed or not still:
            return None
        tail = (np.array(t) >= now - cfg.c0_tail_s - _EPS) if cfg.c0_tail_s > 0.0 else np.ones(len(t), bool)
        self.c0 = rels[tail].mean(axis=0)
        self.z0 = float(np.mean([n[2] for n, k in zip(self._near, tail) if k]))
        self.t_contact = now
        self.tries += 1
        self.state = PROVE
        self._ok_t0 = self._bad_t0 = None
        self._near.clear()
        self.drift = self.rise = 0.0
        return 'contact'

    def _fail(self, reason: str, now: float) -> str:
        self.fails += 1
        self.reason = reason
        self.last_fail = {'reason': reason, 'drift_m': self.drift, 'plate_rise_m': self.rise,
                          'contact_age_s': now - self.t_contact}
        self.state = WATCH
        self._forget_contact()
        return 'failed'

    def report(self, now: float) -> dict:
        """The /magnet/attach_proof record (nan -> None)."""
        def num(x):
            return None if x is None or not math.isfinite(x) else round(float(x), 5)
        return {'state': self.state, 'drift_m': num(self.drift), 'plate_rise_m': num(self.rise),
                'body_to_tip_m': num(self.body_to_tip), 'tries': self.tries,
                'contact_age_s': None if self.t_contact is None else num(now - self.t_contact),
                'fails': self.fails, 'reason': self.reason, 'distance_m': num(self.distance),
                'rel_speed_mps': num(self.rel_speed),
                **({'rel_vz_mps': num(self.rel_vz)} if self.config.vz_max > 0.0 else {})}
