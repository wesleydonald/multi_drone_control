"""Tejen's attach confirmation for an airborne ring (plan docs/plan_after_rig_2026-10-14.md, WP3), without ROS.

His rules (m2_irl_attachment_policy.py contact gate, m1_pickup_proof.py co-motion), adjusted for a ring carried in the
air by three drones and a QUAD4 hanging its magnet on a slack tether:
- contact: the tip marker near the weld point in the plate frame (xy, normal along the ring's z), slow, held;
- proof, after his small lift is done: the tip stays with the plate (drift, normal, relative speed) while the tether is
  taut (body-to-tip at least this flight's own free-hang length - 3 mm) and the plate moves (co-motion): a taut tether
  that cannot lift the tip off the plate is the catch;
- failure only on evidence: separated after the lift, lost at any time, inconclusive after the lift, stale data.

The 14 Oct rig catch (RIG-1014-t6h_4) failed the previous rules three times on a typed tether length 2-3 mm above the
real one and a plate "rise" on a ring that bobbed +-25 mm; replayed here it is proven 1 s after the lift.

The caller feeds one sample per tip pose on its own clock (sim time in the twin), with the plate frame (the ring's
rotation) and whether his lift is done, and calls tick() regularly so the timers run without samples.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from drone_magnet.contact_proof import ATTACHED, OFF, PROVE, WATCH, slope_speed

_EPS = 1e-9


@dataclass(frozen=True)
class ComotionConfig:
    contact_xy: float = 0.035        # m, tip to weld point in the plate plane
    normal_min: float = -0.015       # m, tip below the weld point along the plate normal
    normal_max: float = 0.012        # m, above (a tilted ring raises a caught marker a little)
    speed: float = 0.03              # m/s, the contact gate's relative speed
    contact_dwell_s: float = 0.30
    c0_tail_s: float = 0.10          # s: the contact seed from the dwell's last samples
    hang_normal: float = 0.05        # m above the plate: a free-hang sample for the tether length
    hang_window_s: float = 2.0
    taut_margin: float = 0.003       # m below the free-hang length that still reads taut
    taut_fallback: float = 0.0       # m, the typed taut length if no free hang was seen (minus 5 mm); 0 = no check
    lift_fallback_s: float = 2.5     # s after contact when no lift-done signal comes (0 = wait for the signal)
    d_ok: float = 0.006              # m, drift from the contact seed
    normal_ok: float = 0.010
    speed_ok: float = 0.03           # m/s
    proof_dwell_s: float = 1.0
    comotion_range: float = 0.005    # m, plate z range over the proof window ...
    h_plate: float = 0.003           # m ... or plate z above the contact's
    sep_normal: float = 0.010        # m: after the lift, a tip this far above the plate rose with QUAD4
    sep_xy: float = 0.045
    sep_dwell_s: float = 0.2
    lost: float = 0.045              # m, xy or |normal|: his loss rule, any time
    lost_dwell_s: float = 0.12
    inconclusive_s: float = 3.0      # s after the lift
    stale_s: float = 0.5
    ma_s: float = 0.1
    speed_window_s: float = 0.2
    vz_max: float = 0.0              # interface parity with ProofConfig (unused)

    def __post_init__(self) -> None:
        vals = {k: float(v) for k, v in self.__dict__.items()}
        if not all(math.isfinite(v) for v in vals.values()):
            raise ValueError('co-motion proof values must be finite')
        if self.normal_min > 0.0 or self.normal_max < 0.0 or self.contact_xy <= 0.0:
            raise ValueError('the contact gate must contain the weld point')
        for k in ('contact_dwell_s', 'proof_dwell_s', 'ma_s', 'speed_window_s', 'stale_s', 'inconclusive_s'):
            if vals[k] <= 0.0:
                raise ValueError(f'{k} must be positive')


def _hold(t0: Optional[float], cond: bool, now: float) -> Optional[float]:
    return (now if t0 is None else t0) if cond else None


class ComotionProof:
    """OFF -> (magnet ON) WATCH -> (contact) PROVE -> (proven) ATTACHED, or PROVE -> (failed) WATCH. ATTACHED is
    terminal. Same interface as ContactProof, plus the plate frame and his lift-done flag in step()."""

    def __init__(self, config: ComotionConfig) -> None:
        self.config = config
        self.state = OFF
        self.tries = 0
        self.fails = 0
        self.reason = ''
        self.last_fail: Optional[dict] = None
        self.distance = self.xy = self.normal = math.nan
        self.rel_speed = math.nan
        self.rel_vz = math.nan
        self.body_to_tip = math.nan
        self.l_hang: Optional[float] = None
        self._hang = deque()      # (t, body-to-tip) while the tip hangs free above the plate
        self._near = deque()      # (t, rel, plate z) while inside the contact gate
        self._win = deque()       # (t, rel, plate z, body-to-tip) over ma_s
        self._spd = deque()       # (t, rel) over speed_window_s
        self._last_t: Optional[float] = None
        self._forget_contact()

    def _forget_contact(self) -> None:
        self.c0: Optional[np.ndarray] = None
        self.z0: Optional[float] = None
        self.t_contact: Optional[float] = None
        self.t_lift: Optional[float] = None
        self._ok_t0 = self._sep_t0 = self._lost_t0 = None
        self._plate_z = deque()   # (t, plate z) over the proof window
        self.drift = math.nan
        self.rise = math.nan
        self.comotion = math.nan

    def _forget_samples(self) -> None:
        self._win.clear()
        self._spd.clear()
        self._near.clear()
        self._ok_t0 = self._sep_t0 = self._lost_t0 = None

    def magnet(self, on: bool) -> Optional[str]:
        """'on' / 'off' on a change, 'reset' for an OFF during a proof, 'refused' for an OFF after the attach."""
        if on:
            if self.state != OFF:
                return None
            self.state = WATCH
            self._hang.clear()
            return 'on'
        if self.state == ATTACHED:
            return 'refused'
        if self.state == OFF:
            return None
        was = self.state
        self.state = OFF
        self._forget_contact()
        self._forget_samples()
        self._last_t = None
        return 'reset' if was == PROVE else 'off'

    def abort(self, reason: str, now: float) -> Optional[str]:
        """His planner left the proof (a give-up or a take-back): the contact is void."""
        return self._fail(reason, now) if self.state == PROVE else None

    def tick(self, now: float) -> Optional[str]:
        """Timers that also run without samples: 'failed' (stale, inconclusive) or None."""
        cfg = self.config
        if self.state != PROVE:
            return None
        if self._last_t is None or now - self._last_t > cfg.stale_s + _EPS:
            return self._fail('stale', now)
        if self.t_lift is not None and now - self.t_lift >= cfg.inconclusive_s - _EPS:
            return self._fail('inconclusive', now)
        return None

    def step(self, now: float, tip, plate, body=None, fresh: bool = True, rotation=None,
             lift_done: bool = False) -> Optional[str]:
        """One tip sample against the latest weld point (and the body, and the ring's rotation for the plate frame):
        'contact', 'proven', 'failed' or None."""
        cfg = self.config
        if self.state == OFF:
            return None
        now = float(now)
        if not fresh or tip is None or plate is None:
            self._win.clear()
            self._spd.clear()
            self._near.clear()
            return self.tick(now)
        if self._last_t is not None and now <= self._last_t:
            return None
        self._last_t = now
        tip = np.asarray(tip, dtype=float).reshape(3)
        plate = np.asarray(plate, dtype=float).reshape(3)
        r_mat = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float).reshape(3, 3)
        rel = r_mat.T @ (tip - plate)
        self.distance = float(np.linalg.norm(rel))
        self.xy = float(math.hypot(rel[0], rel[1]))
        self.normal = float(rel[2])
        self.body_to_tip = (math.nan if body is None
                            else float(np.linalg.norm(tip - np.asarray(body, dtype=float).reshape(3))))
        self._win.append((now, rel, float(plate[2]), self.body_to_tip))
        while self._win[0][0] < now - cfg.ma_s + _EPS:
            self._win.popleft()
        self._spd.append((now, rel))
        while self._spd[0][0] < now - cfg.speed_window_s + _EPS:
            self._spd.popleft()
        if self.state == WATCH:
            return self._watch(now, rel, float(plate[2]))
        if self.state == ATTACHED:
            return None
        return self._prove(now, float(plate[2]), lift_done)

    def _watch(self, now: float, rel: np.ndarray, plate_z: float) -> Optional[str]:
        cfg = self.config
        if self.normal > cfg.hang_normal and math.isfinite(self.body_to_tip):
            self._hang.append((now, self.body_to_tip))
        while self._hang and self._hang[0][0] < now - cfg.hang_window_s:
            self._hang.popleft()
        inside = self.xy <= cfg.contact_xy and cfg.normal_min <= self.normal <= cfg.normal_max
        if not inside:
            self._near.clear()
            self.rel_speed = math.nan
            return None
        self._near.append((now, rel, plate_z))
        while len(self._near) >= 2 and self._near[1][0] <= now - cfg.contact_dwell_s + _EPS:
            self._near.popleft()
        t = [n[0] for n in self._near]
        rels = np.array([n[1] for n in self._near])
        self.rel_speed = slope_speed(t, rels)
        if now - t[0] < cfg.contact_dwell_s - _EPS or not self.rel_speed <= cfg.speed:
            return None
        tail = np.array(t) >= now - cfg.c0_tail_s - _EPS
        self.c0 = rels[tail].mean(axis=0)
        self.z0 = float(np.mean([n[2] for n, k in zip(self._near, tail) if k]))
        hang = [h[1] for h in self._hang]
        self.l_hang = (float(np.median(hang)) if hang
                       else cfg.taut_fallback - 0.005 + cfg.taut_margin if cfg.taut_fallback > 0.0 else None)
        self.t_contact = now
        self.t_lift = None
        self.tries += 1
        self.state = PROVE
        self._ok_t0 = self._sep_t0 = self._lost_t0 = None
        self._plate_z.clear()
        self._near.clear()
        self.drift = self.rise = 0.0
        return 'contact'

    def _prove(self, now: float, plate_z: float, lift_done: bool) -> Optional[str]:
        cfg = self.config
        ma_rel = np.mean([w[1] for w in self._win], axis=0)
        self.drift = float(np.linalg.norm(ma_rel - self.c0))
        self.rise = float(np.mean([w[2] for w in self._win]) - self.z0)
        xy = float(math.hypot(ma_rel[0], ma_rel[1]))
        normal = float(ma_rel[2])
        btt = float(np.median([w[3] for w in self._win]))
        self.rel_speed = slope_speed([s[0] for s in self._spd], [s[1] for s in self._spd])
        lost = xy > cfg.lost or abs(normal) > cfg.lost
        self._lost_t0 = _hold(self._lost_t0, lost, now)
        if lost and now - self._lost_t0 >= cfg.lost_dwell_s - _EPS:
            return self._fail('lost', now)
        if self.t_lift is None and (lift_done or (cfg.lift_fallback_s > 0.0
                                                  and now - self.t_contact >= cfg.lift_fallback_s - _EPS)):
            self.t_lift = now
            self._plate_z.clear()
        if self.t_lift is None:
            return self.tick(now)
        self._plate_z.append((now, plate_z))
        while self._plate_z[0][0] < now - cfg.proof_dwell_s + _EPS:
            self._plate_z.popleft()
        zs = [p[1] for p in self._plate_z]
        self.comotion = float(max(zs) - min(zs))
        sep = normal > cfg.sep_normal or xy > cfg.sep_xy
        self._sep_t0 = _hold(self._sep_t0, sep, now)
        if sep and now - self._sep_t0 >= cfg.sep_dwell_s - _EPS:
            return self._fail('separated', now)
        taut = self.l_hang is None or (math.isfinite(btt) and btt >= self.l_hang - cfg.taut_margin)
        moved = self.comotion >= cfg.comotion_range or self.rise >= cfg.h_plate
        ok = (self.drift <= cfg.d_ok and abs(normal) <= cfg.normal_ok and taut
              and math.isfinite(self.rel_speed) and self.rel_speed <= cfg.speed_ok)
        self._ok_t0 = _hold(self._ok_t0, ok, now)
        if ok and moved and now - self._ok_t0 >= cfg.proof_dwell_s - _EPS:
            self.state = ATTACHED
            return 'proven'
        return self.tick(now)

    def _fail(self, reason: str, now: float) -> str:
        self.fails += 1
        self.reason = reason
        self.last_fail = {'reason': reason, 'drift_m': self.drift, 'plate_rise_m': self.rise,
                          'contact_age_s': now - self.t_contact if self.t_contact is not None else math.nan,
                          'xy_m': self.xy, 'normal_m': self.normal}
        self.state = WATCH
        self._forget_contact()
        self._hang.clear()
        return 'failed'

    def report(self, now: float) -> dict:
        """The /magnet/attach_proof record (nan -> None)."""
        def num(x):
            return None if x is None or not math.isfinite(x) else round(float(x), 5)
        return {'rules': 'comotion', 'state': self.state, 'drift_m': num(self.drift),
                'plate_rise_m': num(self.rise), 'comotion_m': num(self.comotion),
                'body_to_tip_m': num(self.body_to_tip), 'l_hang_m': num(self.l_hang),
                'xy_m': num(self.xy), 'normal_m': num(self.normal), 'tries': self.tries,
                'contact_age_s': None if self.t_contact is None else num(now - self.t_contact),
                'lift_done': self.t_lift is not None, 'fails': self.fails, 'reason': self.reason,
                'distance_m': num(self.distance), 'rel_speed_mps': num(self.rel_speed)}
