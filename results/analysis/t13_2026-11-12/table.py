#!/usr/bin/env python3
"""One row per twin run of the 13 Nov trial checks (card docs/experiments/2026-11-12_t13_trials.md):

    python3 results/analysis/t13_2026-11-12/table.py results/2026-11-12/R43*_t13_*   > results/analysis/t13_2026-11-12/table.md

QUAD4 from his TAKEOFF to the weld (or the take-back, or the end), against every carrier: centre-line clearances (Te-Jen
12 Nov: body, cable and object, not body centres alone) -- QUAD4's body to the carriers' bodies and cables, QUAD4's
cable (body to magnet tip) to the carriers' bodies and cables, the ball to the carriers' bodies and cables. A carrier's
cable runs from its body to its plate (the ring pose and the plates at r 0.225, the plate nearest the carrier). Radii
not subtracted: the twin's quads ~0.1 m, the ball 0.08 m. Then the drop (ring speed, ball from the ring centre and under its plane 5 s
later); the weld; ring tilt peak; for a take-back, how far the ring's reference moved in the 20 s after it."""
import csv
import math
import os
import sys

import numpy as np


def load(run):
    rows = list(csv.DictReader(open(os.path.join(run, 'logs', 'run.csv'))))
    col = lambda k: np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan for r in rows])  # noqa
    keys = ['t', 'payload_x', 'payload_y', 'payload_z', 'payload_ref_x', 'payload_ref_y', 'payload_tilt_deg',
            'payload_qw', 'payload_qx', 'payload_qy', 'payload_qz', 'tip_x', 'tip_y', 'tip_z',
            'ball_x', 'ball_y', 'ball_z'] + [f'd{i}_{a}' for i in range(4) for a in 'xyz']
    d = {k: col(k) for k in keys}
    ev = [(float(r[0]), r[1]) for r in csv.reader(open(os.path.join(run, 'logs', 'events.csv')))
          if len(r) > 1 and r[0] not in ('sim_time', 'nan', '')]
    return d, ev


def rot(qw, qx, qy, qz):
    """Rotation matrices (n, 3, 3) from unit quaternions."""
    return np.stack([np.stack([1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)], -1),
                     np.stack([2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)], -1),
                     np.stack([2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)], -1)], 1)


def seg_point(a, b, p):
    """Distance from points p to segments a-b, all (n, 3)."""
    ab = b - a
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - p, axis=1)


def seg_seg(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2, all (n, 3) (sampled: 21 points along the first)."""
    return np.min([seg_point(p2, q2, p1 + s_ * (q1 - p1)) for s_ in np.linspace(0.0, 1.0, 21)], axis=0)


def first(ev, *names):
    return next((t for t, e in ev if e in names), None)


def clearances(d, m):
    """The closest centre-line distances over the samples m (see the module note)."""
    q = np.stack([d['d3_x'], d['d3_y'], d['d3_z']], 1)[m]
    tip = np.stack([d['tip_x'], d['tip_y'], d['tip_z']], 1)[m]
    ball = np.stack([d['ball_x'], d['ball_y'], d['ball_z']], 1)[m]
    ring = np.stack([d['payload_x'], d['payload_y'], d['payload_z']], 1)[m]
    R = rot(d['payload_qw'][m], d['payload_qx'][m], d['payload_qy'][m], d['payload_qz'][m])
    plates = [ring + np.einsum('nij,j->ni', R, [0.225 * math.cos(math.radians(-30 * k)),
                                                 0.225 * math.sin(math.radians(-30 * k)), 0.0]) for k in (3, 7, 11)]
    best = {k: np.inf for k in ('body-body', 'body-cable', 'cable-body', 'cable-cable', 'ball-body', 'ball-cable')}
    for i in range(3):
        c = np.stack([d[f'd{i}_x'], d[f'd{i}_y'], d[f'd{i}_z']], 1)[m]
        k = int(np.argmin([np.nanmedian(np.linalg.norm(c - pl, axis=1)) for pl in plates]))
        pl = plates[k]
        ok = np.isfinite(c).all(1) & np.isfinite(q).all(1) & np.isfinite(tip).all(1) & np.isfinite(pl).all(1)
        if not ok.any():
            continue
        ci, qi, ti, pli, bi = c[ok], q[ok], tip[ok], pl[ok], ball[ok]
        for key, v in (('body-body', np.linalg.norm(qi - ci, axis=1)), ('body-cable', seg_point(ci, pli, qi)),
                       ('cable-body', seg_point(qi, ti, ci)), ('cable-cable', seg_seg(qi, ti, ci, pli)),
                       ('ball-body', np.linalg.norm(bi - ci, axis=1)), ('ball-cable', seg_point(ci, pli, bi))):
            best[key] = min(best[key], float(np.nanmin(v)))
    return best


def row(run):
    d, ev = load(run)
    t = d['t']
    name = os.path.basename(run)
    out = {'run': name[:5], 'case': name.split('_t13_')[-1].replace('_twin', '')}
    t_to = first(ev, 'PHASE:TAKEOFF')
    t_drop = first(ev, 'PHASE:DROP_OBJECT')
    t_weld = first(ev, 'WELD', 'REWELD')
    t_tb = first(ev, 'PHASE:TAKEN_BACK', 'TAKE_BACK')
    t_end = next((x for x in (t_weld, t_tb) if x is not None), t[-1])
    m = (t >= (t_to or 0.0)) & (t <= t_end)
    t_settle = first(ev, 'PHASE:SETTLE_ABOVE_DROP_POINT') or t_end
    for label, mask in (('route in', (t >= (t_to or 0.0)) & (t <= t_settle)), ('whole', m)):
        best = clearances(d, mask)
        out[f'{label}: QUAD4 body to carrier body / cable (m)'] = f"{best['body-body']:.2f} / {best['body-cable']:.2f}"
        out[f'{label}: QUAD4 cable to carrier body / cable (m)'] = f"{best['cable-body']:.2f} / {best['cable-cable']:.2f}"
        out[f'{label}: ball to carrier body / cable (m)'] = f"{best['ball-body']:.2f} / {best['ball-cable']:.2f}"
    vx, vy = np.gradient(d['payload_x'], t), np.gradient(d['payload_y'], t)
    if t_drop is not None:
        k = (t > t_drop - 0.5) & (t < t_drop + 0.5)
        i = min(np.searchsorted(t, t_drop + 5.0), len(t) - 1)
        out['drop: ring m/s'] = f'{np.nanmean(np.hypot(vx[k], vy[k])):.3f}'
        out['ball from centre / under plane (cm)'] = (f'{100 * math.hypot(d["ball_x"][i] - d["payload_x"][i], d["ball_y"][i] - d["payload_y"][i]):.1f}'
                                                     f' / {100 * (d["payload_z"][i] - d["ball_z"][i]):.1f}')
    else:
        out['drop: ring m/s'] = '-'
        out['ball from centre / under plane (cm)'] = '-'
    out['weld (s after drop)'] = '-' if t_weld is None or t_drop is None else f'{t_weld - t_drop:.1f}'
    if t_weld is not None:
        k = (t > t_weld - 0.5) & (t < t_weld + 0.5)
        out['ring m/s at weld'] = f'{np.nanmean(np.hypot(vx[k], vy[k])):.3f}'
    else:
        out['ring m/s at weld'] = '-'
    if t_tb is not None:
        a, z = np.searchsorted(t, t_tb + 3.5), np.searchsorted(t, t_tb + 23.5)   # after the 3 s slew to rest
        z = min(z, len(t) - 1)
        out['ref moved after take-back (cm)'] = f'{100 * math.hypot(d["payload_ref_x"][z] - d["payload_ref_x"][a], d["payload_ref_y"][z] - d["payload_ref_y"][a]):.1f}'
    else:
        out['ref moved after take-back (cm)'] = '-'
    out['tilt peak (deg)'] = f'{np.nanmax(d["payload_tilt_deg"]):.1f}'
    out['abort'] = 'yes' if any(e in ('FLEET_ABORT', 'ABORT') for _, e in ev) else 'no'
    return out


def main():
    rows = [row(r.rstrip('/')) for r in sys.argv[1:]]
    keys = list(rows[0])
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '---|' * len(keys))
    for r in rows:
        print('| ' + ' | '.join(str(r.get(k, '')) for k in keys) + ' |')


if __name__ == '__main__':
    main()
