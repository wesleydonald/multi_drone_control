#!/usr/bin/env python3
"""W1: each carrier's hover throttle by formation, predicted from statics and the rig's affine thrust map, against
the throttle measured in steady hover (results/rig/2026-10-29; the 0.86 kg ring's pairs of 28 Oct as an outside check).

Prediction: the ring hangs level on straight cables at the planner's elevation (cable_elev_deg 60) from plates at
attach_radius; the tensions are the planner's own geometry.balanced_tensions (least squares to the equal split, floor
0.1 N); each carrier's thrust |m_d g z - t_i s_i| (m_d 0.583 kg with its 1 m cable, real.yaml) goes through the map
    throttle = 0.185 + 0.022 (23.5 - V) + 0.507 |F| / g          (RIG-0930-ladder4; tracker_node._thrust_off)
Measured: per drone the median thr_out (tracker log) in 4 s bins of steady hover (windows below), with the median pack
voltage of the same bin (battery_v) and the bin's 95th percentile (control effort).
Calibration, once, on the first bin of every square and triangle hover: a bias per quad and one common correction to
the gain (throttle per kg of thrust); then the T (sq1) and the pairs (28 Oct) are predicted out of sample.

    python3 w1_formation_thrust.py            # writes w1_formation_thrust.json next to it
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common  # noqa: E402
from common import G, OUT  # noqa: E402

from mpc_planner.geometry import balanced_tensions, nominal_cable_dirs  # noqa: E402

OFF0, VSLOPE, VREF, GAIN = 0.185, 0.022, 23.5, 0.507
M_DRONE = 0.583
RINGS = {'1.15 kg': dict(M=1.15, r=0.204, z=0.025), '0.86 kg M2A': dict(M=0.86, r=0.225, z=0.0175)}
# plates clockwise from the ring's +x, 30 deg each (the maths inside counter-clockwise)
LAYOUTS = {'square 0/3/6/9': [0, 3, 6, 9], 'four 3/7/9/11 (after the attach)': [3, 7, 9, 11],
           'triangle 3/11/7': [3, 11, 7], 'T 0/3/6': [0, 3, 6], 'pair 0/6': [0, 6]}
DAY29 = os.path.join(common.REPO, 'results', 'rig', '2026-10-29')
DAY28 = os.path.join(common.REPO, 'results', 'rig', '2026-10-28')
# (day, run, layout, plates in drone order, steady window in s after the planner log's first row, ring, drone mass,
#  quad of each drone)
FLIGHTS = [
    (DAY29, 'h4f_3', 'square', [0, 3, 6, 9], (65.5, 70.5), '1.15 kg', M_DRONE, [1, 2, 3, 4]),
    (DAY29, 'f4', 'square', [0, 3, 6, 9], (69.5, 74.5), '1.15 kg', M_DRONE, [1, 2, 3, 4]),   # traj_hold 5 s
    (DAY29, 'ch1b', 'square', [0, 3, 6, 9], (45.0, 53.0), '1.15 kg', M_DRONE, [1, 2, 3, 4]),  # before the release
    (DAY29, 'ch1b_2', 'square', [0, 3, 6, 9], (71.0, 86.0), '1.15 kg', M_DRONE, [1, 2, 3, 4]),
    (DAY29, 'sq2_5', 'triangle', [3, 11, 7], (70.0, 114.0), '1.15 kg', M_DRONE, [1, 2, 3]),   # to his MATCH 115.4 s
    (DAY29, 'sq5_2', 'triangle', [3, 11, 7], (47.0, 61.0), '1.15 kg', M_DRONE, [1, 2, 3]),    # to his MATCH 61.5 s
    (DAY29, 'sq3', 'triangle', [3, 11, 7], (147.0, 183.0), '1.15 kg', M_DRONE, [1, 2, 3]),
    (DAY29, 'sq4_5', 'triangle', [3, 11, 7], (109.0, 141.0), '1.15 kg', M_DRONE, [1, 2, 3]),
    (DAY29, 'k1_4', 'triangle', [3, 11, 7], (111.0, 135.0), '1.15 kg', M_DRONE, [1, 2, 3]),
    (DAY29, 'sq1', 'T', [0, 3, 6], (143.5, 146.2), '1.15 kg', M_DRONE, [1, 2, 3]),         # top 0.90 m; drop 146.4
    (DAY29, 'ch1b_2', 'T (sinking, after the release)', [0, 3, 6], (90.0, 93.0), '1.15 kg', M_DRONE, [1, 2, 3]),
    (DAY28, 'hS1', 'pair (0.55 m cables)', [11, 5], (24.0, 29.0), '0.86 kg M2A', 0.55, [2, 3]),
    (DAY28, 'hS2', 'pair (0.55 m cables)', [11, 5], (44.5, 53.0), '0.86 kg M2A', 0.55, [2, 3]),
]
BIN = 4.0
CAL = ('square', 'triangle')


def offset(v):
    return OFF0 + VSLOPE * (VREF - v)


def statics(plates, M, r, z, elev=60.0, m_d=M_DRONE):
    """Per plate (tension N, thrust N, thrust / g in kg) for a level ring on straight cables."""
    rho = [np.array([r * math.cos(math.radians(-30 * p)), r * math.sin(math.radians(-30 * p)), z]) for p in plates]
    s = nominal_cable_dirs(rho, elev)
    t = balanced_tensions(rho, s, M, g=G)
    out = []
    for ti, si in zip(t, s):
        f = float(np.linalg.norm(m_d * G * np.array([0, 0, 1.0]) - ti * np.asarray(si)))
        out.append((ti, f, f / G))
    return out


def bins(day, run, plates, win, quads):
    common.DAY = day
    d, tr = common.planner(run), common.trackers(run)
    t0 = d.sim_time.iloc[0]
    out, a = [], win[0]
    while a < win[1] - 1e-6:
        b = min(a + BIN, win[1])
        if b - a < 2.0 and out:
            break
        w = d[(d.sim_time >= t0 + a) & (d.sim_time < t0 + b)]
        row = dict(t=(a, b), z=float(w.load_z.median()), tilt=float(w.tilt_deg.median()), drones=[])
        for i, p in enumerate(plates):
            x = tr[i][(tr[i].sim_time >= t0 + a) & (tr[i].sim_time < t0 + b)]
            row['drones'].append(dict(quad=quads[i], plate=p, thr=float(x.thr_out.median()),
                                      thr_p95=float(x.thr_out.quantile(0.95)), v=float(x.battery_v.median()),
                                      elev=float(w[f'elev{i}'].median())))
        out.append(row)
        a = b
    return out


def main():
    rows = []
    for day, run, lay, plates, win, ring, m_d, quads in FLIGHTS:
        st = statics(plates, **RINGS[ring], m_d=m_d)
        for k, bn in enumerate(bins(day, run, plates, win, quads)):
            for j, dr in enumerate(bn['drones']):
                pred = offset(dr['v']) + GAIN * st[j][2]
                rows.append(dict(run=run, layout=lay, ring=ring, bin=k, t0=bn['t'][0], t1=bn['t'][1], z=bn['z'],
                                 tilt=bn['tilt'], kg=st[j][2], tension=st[j][0], **dr, pred=pred,
                                 res=dr['thr'] - pred))
    # calibration: first bins of the square and triangle hovers, res = b_quad + k (kg - kg_square)
    first = [r for r in rows if r['layout'] in CAL and r['bin'] == 0]
    quads = sorted({r['quad'] for r in first})
    kg0 = statics(LAYOUTS['square 0/3/6/9'], **RINGS['1.15 kg'])[0][2]
    A = np.array([[1.0 if r['quad'] == q else 0.0 for q in quads] + [r['kg'] - kg0] for r in first])
    y = np.array([r['res'] for r in first])
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    bias, k_gain = dict(zip(quads, sol[:-1])), float(sol[-1])
    for r in rows:
        r['pred_cal'] = r['pred'] + bias.get(r['quad'], 0.0) + k_gain * (r['kg'] - kg0)
        r['res_cal'] = r['thr'] - r['pred_cal']
    # hover fade: throttle rise per minute within each square / triangle window (first to last bin)
    fade = []
    for day, run, lay, plates, win, ring, m_d, qs in FLIGHTS:
        if lay not in CAL:
            continue
        rr = [r for r in rows if r['run'] == run and r['layout'] == lay]
        nb = max(r['bin'] for r in rr)
        if nb < 2:
            continue
        for q in qs:
            a = next(r for r in rr if r['quad'] == q and r['bin'] == 0)
            b = next(r for r in rr if r['quad'] == q and r['bin'] == nb)
            fade.append(dict(run=run, layout=lay, quad=q, dt=b['t0'] - a['t0'], dthr=b['thr'] - a['thr'],
                             dv=b['v'] - a['v'], per_min=60 * (b['thr'] - a['thr']) / (b['t0'] - a['t0']),
                             model_per_min=60 * VSLOPE * (a['v'] - b['v']) / (b['t0'] - a['t0'])))
    effort = {}
    for lay in sorted({r['layout'] for r in rows}):
        e = [r['thr_p95'] - r['thr'] for r in rows if r['layout'] == lay]
        effort[lay] = (float(np.median(e)), float(np.max(e)))
    # the formation table: model and calibrated (mean quad bias) at 23.5 V and 22.0 V
    mean_bias = float(np.mean(list(bias.values())))
    table = []
    for rn, rg in RINGS.items():
        for lay, plates in LAYOUTS.items():
            for p, (t, f, kg) in zip(plates, statics(plates, **rg)):
                cal = mean_bias + k_gain * (kg - kg0)
                table.append(dict(ring=rn, layout=lay, plate=p, tension=t, kg=kg,
                                  model_23v5=offset(23.5) + GAIN * kg, model_22v0=offset(22.0) + GAIN * kg,
                                  cal_23v5=offset(23.5) + GAIN * kg + cal, cal_22v0=offset(22.0) + GAIN * kg + cal))
    # margin under 0.95 for the worst carrier: calibrated start throttle (23.5 V; 22.0 V for the T and the pair, where
    # sq1's loaded pair sat from the first second) plus the bin's p95 effort, and after 90 s plus the median fade
    fade_med = float(np.median([f_['per_min'] for f_ in fade if f_['dt'] >= 20]))
    eff = {'square 0/3/6/9': effort['square'][0], 'four 3/7/9/11 (after the attach)': effort['square'][0],
           'triangle 3/11/7': effort['triangle'][0], 'T 0/3/6': effort['T'][0], 'pair 0/6': effort['T'][0]}
    margin = []
    for rn in RINGS:
        for lay in LAYOUTS:
            heavy = lay in ('T 0/3/6', 'pair 0/6')
            worst = max(t['cal_22v0' if heavy else 'cal_23v5'] for t in table if t['ring'] == rn and t['layout'] == lay)
            start = worst + eff[lay]
            margin.append(dict(ring=rn, layout=lay, v=22.0 if heavy else 23.5, start_thr=start, margin_start=0.95 - start,
                               margin_90s=0.95 - start - 1.5 * fade_med))
    res = dict(bias={int(q): float(b) for q, b in bias.items()}, fade_median_per_min=fade_med, margin=margin, k_gain_per_kg=k_gain, kg_square=kg0,
               rows=rows, fade=fade, effort=effort, table=table,
               rms_model_cal_set=float(np.sqrt(np.mean([r['res'] ** 2 for r in first]))),
               rms_cal_cal_set=float(np.sqrt(np.mean([r['res_cal'] ** 2 for r in first]))))
    with open(os.path.join(OUT, 'w1_formation_thrust.json'), 'w') as f:
        json.dump(res, f, indent=1)
    print('bias per quad', {q: round(b, 3) for q, b in bias.items()}, 'gain correction', round(k_gain, 3), 'per kg')
    print('rms on the calibration set', round(res['rms_model_cal_set'], 3), '->', round(res['rms_cal_cal_set'], 3))
    print('\n# formation table')
    for t in table:
        print(f"{t['ring']:12s} {t['layout']:34s} p{t['plate']:2d} t {t['tension']:5.2f} N  model {t['model_23v5']:.3f} / "
              f"{t['model_22v0']:.3f}  cal {t['cal_23v5']:.3f} / {t['cal_22v0']:.3f}")
    print('\n# per flight: first and last bin per quad')
    for day, run, lay, plates, win, ring, m_d, qs in FLIGHTS:
        rr = [r for r in rows if r['run'] == run and r['layout'] == lay]
        nb = max(r['bin'] for r in rr)
        for q in qs:
            a = next(r for r in rr if r['quad'] == q and r['bin'] == 0)
            b = next(r for r in rr if r['quad'] == q and r['bin'] == nb)
            print(f"{run:7s} {lay[:22]:22s} Q{q} p{a['plate']:2d} {a['t0']:.0f}-{b['t1']:.0f} s z{a['z']:.2f} tilt {a['tilt']:.1f} "
                  f"el {a['elev']:.1f} | start V {a['v']:.1f} meas {a['thr']:.3f} p95 {a['thr_p95']:.3f} model {a['pred']:.3f} "
                  f"cal {a['pred_cal']:.3f} | end V {b['v']:.1f} meas {b['thr']:.3f} model {b['pred']:.3f}")
    print('\n# fade per minute (measured, model voltage term)')
    for f_ in fade:
        print(f"{f_['run']:7s} Q{f_['quad']} {f_['dt']:.0f} s dV {f_['dv']:+.1f}  {f_['per_min']:+.3f}/min  model {f_['model_per_min']:+.3f}/min")
    print('\n# margin under 0.95 (worst carrier), fade median', round(fade_med, 3), '/min')
    for m in margin:
        print(f"{m['ring']:12s} {m['layout']:34s} {m['v']:.1f} V start {m['start_thr']:.3f}  margin {m['margin_start']:+.3f}  after 90 s {m['margin_90s']:+.3f}")
    print('\n# effort p95 - median per layout (median, max)', {k: (round(a, 3), round(b, 3)) for k, (a, b) in effort.items()})


if __name__ == '__main__':
    main()
