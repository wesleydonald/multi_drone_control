#!/usr/bin/env python3
"""tools/release_bands.py -- the release metrics (T6) and the W4/W5 ledger bands of docs/freeze_2026-10-14.md.

    python3 tools/release_bands.py metrics R1459 rig:2026-10-08/r303_4    # T6 metrics per run
    python3 tools/release_bands.py factors                               # 8 Oct rig/twin factors per class
    python3 tools/release_bands.py sigma                                 # sigma_rig from the 8 Oct rig flights
    python3 tools/release_bands.py band W4 R1459 R1460 R1xxx              # a frozen band from its twin runs
    python3 tools/release_bands.py band W5 R1457 R1xxx

Reads only run.csv and events.csv: results/<date>/R####_*/logs for twin runs, and for rig flights
results/rig/<date>/<run>_run/logs as tools/rig_to_run.py writes them (the ring pose needs the bag). The open
choices of the freeze are the flags below; their defaults are the recommendations awaiting Wesley's signature,
and every output names the choices it used.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.environ.get('MDC_RESULTS_ROOT', os.path.join(REPO, 'results'))

PRE_S = 5.0            # pre-event window
HOLD_S = 1.0           # time back in band: held this long
BAND_MIN_DEG = 0.5     # time back in band: smallest half-width
POST_S = 5.0           # post-event steady tilt: the last POST_S before LAND
POST_MIN_S = 10.0      # ... scored only when LAND comes at least this long after the event
BAND_HORIZON_S = 15.0  # Q1: W4/W5 band and rig values cut at event + the D5 minimum hold
NULL_PAD_S = 2.0       # Q4 alternative: pseudo-events start this long after the lift top
NULL_W_S = 2.0         # Q4 alternative: pseudo-event window

# The 8 Oct rig/twin pairs (registry rows RIG-1008-*); the twin runs are the registry's pairs.
PAIRS = [
    ('rig:2026-10-08/r302_3', ('R1207', 'R1212'), 'announced', 'hover'),
    ('rig:2026-10-08/r303_4', ('R1140', 'R1145'), 'unannounced', 'hover'),
    ('rig:2026-10-08/r304_3', ('R1206', 'R1211'), 'unannounced', 'hover'),
    ('rig:2026-10-08/r305', ('R1208', 'R1213'), 'unannounced', 'circle'),
]
CELLS = {'W4': ('unannounced', 'hover'), 'W5': ('unannounced', 'circle')}


def locate(tag):
    """A run's logs directory from R####, rig:<date>/<run> or a path."""
    if tag.startswith('rig:'):
        d = os.path.join(RESULTS, 'rig', tag[4:] + '_run', 'logs')
    elif tag[:1] == 'R' and tag[1:5].isdigit():
        hits = [h for h in glob.glob(os.path.join(RESULTS, '2*', tag + '_*', 'logs')) if os.path.isdir(h)]
        if len(hits) != 1:
            sys.exit(f'{tag}: {len(hits)} run directories found under {RESULTS}')
        d = hits[0]
    else:
        d = tag if os.path.basename(tag.rstrip('/')) == 'logs' else os.path.join(tag, 'logs')
    if not os.path.isfile(os.path.join(d, 'run.csv')):
        sys.exit(f'{tag}: no run.csv in {d}')
    return d


def tilt_vector(q):
    """World-frame tilt vector (deg): the ring normal's horizontal direction, scaled to its tilt angle."""
    w, x, y, z = q.T
    nx, ny, nz = 2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)
    h = np.hypot(nx, ny)
    ang = np.degrees(np.arctan2(h, nz))
    s = np.where(h > 1e-12, ang / np.maximum(h, 1e-12), 0.0)
    return np.stack([nx * s, ny * s], 1)


class Run:
    def __init__(self, tag):
        self.tag = tag
        d = locate(tag)
        r = pd.read_csv(os.path.join(d, 'run.csv'))
        cols = ['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']
        r = r.dropna(subset=cols).reset_index(drop=True)
        self.t = r['t'].to_numpy(float)
        self.th = tilt_vector(r[cols].to_numpy(float))
        self.ref = r[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
        ev = pd.read_csv(os.path.join(d, 'events.csv'))
        self.ev = {}
        for _, row in ev.iterrows():
            self.ev.setdefault(str(row['event']), float(row['sim_time']))
        if 'RELEASE' in self.ev:
            self.kind, self.te = 'unannounced', self.ev['RELEASE']
        elif 'DETACH' in self.ev:
            self.kind, self.te = 'announced', self.ev['DETACH']
        else:
            sys.exit(f'{tag}: no RELEASE or DETACH event (soft detaches: the ramp start is not implemented yet)')
        if 'LAND' not in self.ev:
            sys.exit(f'{tag}: no LAND event')
        self.tland = self.ev['LAND']

    def win(self, a, b):
        return (self.t >= a) & (self.t < b)

    def pre(self):
        m = self.win(self.te - PRE_S, self.te)
        mean = self.th[m].mean(0)
        sig = float(np.sqrt(np.mean(np.sum((self.th[m] - mean) ** 2, 1))))
        return mean, sig, int(m.sum())

    def ref_still_s(self):
        """Q12: how long before the event the ring reference last changed (lift ramp, orbit spin-up): the
        reference height more than 5 mm from its value at the event, or its speed (a 2 s chord: the planner
        updates it at 10 Hz) more than 0.02 m/s from its median over the last 2 s."""
        ok = np.all(np.isfinite(self.ref), 1) & (self.t <= self.te)
        t, ref = self.t[ok], self.ref[ok]
        back = np.stack([np.interp(t - 2.0, t, ref[:, i]) for i in range(2)], 1)
        v = np.linalg.norm(ref[:, :2] - back, axis=1) / 2.0
        v_e = float(np.median(v[t >= self.te - 2.0]))
        moved = (np.abs(ref[:, 2] - ref[-1, 2]) > 0.005) | (np.abs(v - v_e) > 0.02)
        moved &= t >= t[0] + 2.0
        return float(self.te - (t[np.nonzero(moved)[0][-1]] if moved.any() else t[0]))

    def metrics(self, horizon=None):
        mean, sig, npre = self.pre()
        end = self.tland if horizon is None else min(self.tland, self.te + horizon)
        m = self.win(self.te, end + 1e-9)
        dev = np.linalg.norm(self.th[m] - mean, axis=1)
        mag = np.linalg.norm(self.th, axis=1)
        k = int(np.argmax(dev))
        out = dict(run=self.tag, kind=self.kind, t_event=self.te, window_s=end - self.te,
                   dtheta=float(dev[k]), t_peak=float(self.t[m][k] - self.te),
                   abs_peak=float(mag[m].max()), rise=float(mag[m].max() - mag[self.win(self.te - PRE_S, self.te)].mean()),
                   sigma_pre=sig, n_pre=npre, ref_still=self.ref_still_s(), t_back=None, post=None)
        out['steady'] = out['ref_still'] >= PRE_S
        if self.tland - self.te >= POST_MIN_S:
            post = self.th[self.win(self.tland - POST_S, self.tland)].mean(0)
            b = max(2 * sig, BAND_MIN_DEG)
            inb = np.linalg.norm(self.th - post, axis=1) <= b
            for i in np.nonzero((self.t >= self.te + out['t_peak']) & (self.t + HOLD_S <= self.tland))[0]:
                j = self.win(self.t[i], self.t[i] + HOLD_S)
                if inb[j].all():
                    out['t_back'] = float(self.t[i] - self.te)
                    break
            out['post'] = float(np.linalg.norm(post))
            out['band_deg'] = b
        return out

    def null(self):
        """Q4 alternative: delta-theta at pseudo-events every 0.5 s in the steady hover before the event."""
        z_at = self.ref[np.argmin(np.abs(self.t - self.te)), 2]
        ok = np.abs(self.ref[:, 2] - z_at) <= 0.005
        t0 = self.t[ok & (self.t < self.te)].min() + NULL_PAD_S
        vals = []
        for ts in np.arange(t0 + PRE_S, self.te - NULL_W_S, 0.5):
            p = self.win(ts - PRE_S, ts)
            w = self.win(ts, ts + NULL_W_S)
            if p.sum() > 10 and w.sum() > 5:
                vals.append(float(np.linalg.norm(self.th[w] - self.th[p].mean(0), axis=1).max()))
        return vals


def class_key(announce, traj, by):
    return {'announce': announce, 'announce_traj': f'{announce}/{traj}', 'all': 'all'}[by]


def factors(a):
    rows, groups = [], {}
    for rig_tag, twins, announce, traj in PAIRS:
        rig = Run(rig_tag)
        hz = (rig.tland - rig.te) if a.pair_horizon == 'rig' else None
        r = rig.metrics(hz)['dtheta']
        tw = [Run(t).metrics(hz)['dtheta'] for t in twins]
        if a.ratio == 'mean':
            lo = hi = r / float(np.mean(tw))
        else:
            lo, hi = r / max(tw), r / min(tw)
        key = class_key(announce, traj, a.classes)
        groups.setdefault(key, []).append((lo, hi))
        rows.append((rig_tag, twins, key, hz, r, tw, lo, hi))
    out = {}
    for key, v in groups.items():
        out[key] = ((min(x[0] for x in v), max(x[1] for x in v), len(v)) if len(v) >= 2 else (1.0, 1.0, len(v)))
    return rows, out


def sigma_rig(a):
    if a.sigma == 'pre':
        sq, n = 0.0, 0
        for rig_tag, *_ in PAIRS:
            r = Run(rig_tag)
            mean, _, _ = r.pre()
            m = r.win(r.te - PRE_S, r.te)
            sq += float(np.sum(np.sum((r.th[m] - mean) ** 2, 1)))
            n += int(m.sum())
        return math.sqrt(sq / n), n
    vals = [v for rig_tag, *_ in PAIRS for v in Run(rig_tag).null()]
    return float(np.std(vals, ddof=1)), len(vals)


def choices(a):
    return (f'choices: pair_horizon={a.pair_horizon} classes={a.classes} ratio={a.ratio} sigma={a.sigma} '
            f'band_horizon={a.band_horizon} (defaults = the answers signed 11 Oct)')


def fmt(x, nd=2):
    return '-' if x is None else (f'{x:.{nd}f}' if isinstance(x, float) else str(x))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['metrics', 'factors', 'sigma', 'band'])
    ap.add_argument('args', nargs='*')
    ap.add_argument('--pair-horizon', choices=['rig', 'land'], default='rig', help='Q1 for the 8 Oct pairs')
    ap.add_argument('--band-horizon', type=float, default=BAND_HORIZON_S, help='Q1 for W4/W5 (0 = to LAND)')
    ap.add_argument('--classes', choices=['announce', 'announce_traj', 'all'], default='announce', help='Q5')
    ap.add_argument('--ratio', choices=['mean', 'minmax'], default='mean', help='Q6')
    ap.add_argument('--sigma', choices=['pre', 'null'], default='pre', help='Q4')
    a = ap.parse_args()
    print(choices(a))

    if a.cmd == 'metrics':
        hz = a.band_horizon or None
        print('| run | kind | window (s) | dtheta (deg) | t peak (s) | abs peak | rise | sigma_pre | reference still '
              'before the event (s) | post tilt | time back in band (s) |')
        print('|---|---|---|---|---|---|---|---|---|---|---|')
        for tag in a.args:
            m = Run(tag).metrics(hz)
            print(f"| {tag} | {m['kind']} | {fmt(m['window_s'], 1)} | {fmt(m['dtheta'])} | {fmt(m['t_peak'])} | "
                  f"{fmt(m['abs_peak'])} | {fmt(m['rise'])} | {fmt(m['sigma_pre'])} | "
                  f"{fmt(m['ref_still'], 1)}{'' if m['steady'] else ' FLAG'} | {fmt(m['post'])} | {fmt(m['t_back'])} |")
    elif a.cmd == 'factors':
        rows, out = factors(a)
        print('| rig flight | twin runs | class | window (s) | rig dtheta | twin dtheta | f_lo | f_hi |')
        print('|---|---|---|---|---|---|---|---|')
        for rig_tag, twins, key, hz, r, tw, lo, hi in rows:
            print(f"| {rig_tag} | {', '.join(twins)} | {key} | {fmt(hz, 1)} | {fmt(r)} | "
                  f"{' / '.join(fmt(x) for x in tw)} | {fmt(lo)} | {fmt(hi)} |")
        for key, (lo, hi, n) in out.items():
            print(f'class {key}: f_lo {lo:.3f}, f_hi {hi:.3f} from {n} pair(s)' + ('' if n >= 2 else ' -> f = 1'))
    elif a.cmd == 'sigma':
        s, n = sigma_rig(a)
        print(f'sigma_rig {s:.3f} deg from {n} samples')
    else:
        if len(a.args) < 2 or a.args[0] not in CELLS:
            sys.exit('band W4|W5 TWIN_RUN...')
        cell, twins = a.args[0], a.args[1:]
        _, out = factors(a)
        lo_f, hi_f, n = out.get(class_key(*CELLS[cell], a.classes), (1.0, 1.0, 0))
        s, _ = sigma_rig(a)
        hz = a.band_horizon or None
        tw = [Run(t).metrics(hz)['dtheta'] for t in twins]
        lo = max(0.0, min(tw) * lo_f - 2 * s)
        hi = max(tw) * hi_f + 2 * s
        print(f"{cell}: twin dtheta {' / '.join(fmt(x) for x in tw)} ({', '.join(twins)}); f_lo {lo_f:.3f}, "
              f"f_hi {hi_f:.3f} ({n} pair(s)); sigma_rig {s:.3f}")
        print(f'{cell} band: [{lo:.2f}, {hi:.2f}] deg (dtheta, event to +{fmt(hz, 0)} s or LAND); '
              f'twin midpoint {(min(tw) + max(tw)) / 2:.2f}')


if __name__ == '__main__':
    main()
