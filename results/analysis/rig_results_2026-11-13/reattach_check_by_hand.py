#!/usr/bin/env python3
"""Independent check of detach_planned.py and reattach.py on nrtfc_2 and nrtfc_5 (a planned detach and a proven
re-attach each): the tilt peak, dip and horizontal excursion recomputed from the 20 Hz poses files
(reattach_poses.py, read straight from the bag, no 50 Hz grid) and the log lines, sharing no code with paper_rig,
drops.py or reattach.py. "Before" for a detach is the script's pre_s (paper_rig cuts the 5 s where the set-point
last moved: the lift's end) and the plain 5 s is printed beside it; for the attach the plain 5 s, as reattach.py.

    cd ~/multi_drone_control && python3 results/analysis/rig_results_2026-11-13/reattach_check_by_hand.py
"""
import os
import re

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.expanduser('~/multi_drone_control/results/rig')


def stamp(log, rx, after=0.0):
    for m in re.finditer(r'\[(\d{9,}\.\d+)\][^\n]*?' + rx, log):
        if float(m.group(1)) > after:
            return float(m.group(1))
    return None


def ring(p, a, b):
    w = (p.t_epoch >= a) & (p.t_epoch < b)
    q = p.loc[w, ['r_qx', 'r_qy']].to_numpy(float)
    tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1.0, 1.0)))
    return tilt, p.loc[w, 'r_z'].to_numpy(float), p.loc[w, ['r_x', 'r_y']].to_numpy(float)


def check(name, te, tend, row, kind):
    p = pd.read_csv(os.path.join(HERE, f'{name}_poses.csv'))
    pre = float(row['pre_s']) if kind == 'detach' else 5.0
    _, z0, xy0 = ring(p, te - pre, te)
    _, z5, _ = ring(p, te - 5.0, te)
    tilt, z, xy = ring(p, te, tend)
    dip_end = te + 10.0 if kind == 'detach' else tend
    _, zd, xyd = ring(p, te, min(tend, dip_end))
    exc = 100.0 * np.max(np.linalg.norm(xyd - xy0.mean(0), axis=1))
    print(f'{name} {kind}: window {tend - te:.2f} s (script {row["window_s"]}), before {pre:.2f} s; dip against '
          f'the plain 5 s before: {1000.0 * (z5.mean() - zd.min()):.2f} mm')
    for lab, mine, script in (('tilt peak deg', tilt.max(), row['tilt_peak_deg']),
                              ('dip mm', 1000.0 * (z0.mean() - zd.min()), row['dip_mm']),
                              ('horizontal excursion cm', exc, row['xy_excursion_cm' if kind == 'detach'
                                                                     else 'horiz_offset_cm'])):
        print(f'   {lab:26s} by hand {mine:7.2f}   script {float(script):7.2f}')


det = pd.read_csv(os.path.join(HERE, 'detach_planned.csv'))
rz = pd.read_csv(os.path.join(HERE, 'reattach_resize.csv'))
for name in ('nrtfc_2', 'nrtfc_5'):
    log = open(os.path.join(RIG, '2026-11-11', name + '.log'), errors='replace').read()
    to = stamp(log, r"Fleet command received: 'TAKEOFF'")
    land = stamp(log, r"Fleet command received: 'LAND'", to)
    # detach: our magnet OFF (the planner's or the magnet manager's line) after UNLOADED
    unl = stamp(log, r'UNLOADED drone')
    off = min(x for x in (stamp(log, r'magnet OFF -> /drone_3/magnet', unl - 0.05),
                          stamp(log, r'DETACH: magnet command OFF', unl - 0.05)) if x)
    desc = stamp(log, r'Planner phase: \w+ -> DESCEND_TO_ATTACHMENT', off)
    r = det[det.flight == name].iloc[0]
    print(f'{name}: OFF +{off - to:.2f} s (script {r.t_off_s})')
    check(name, off, min(off + 15.0, desc, land), r, 'detach')
    pv = stamp(log, r'ATTACH: proven', off)
    fold = stamp(log, r'FLEET RESIZED to n=4', pv - 0.5)
    r = rz[rz.flight == name].iloc[0]
    print(f'{name}: fold-in +{fold - to:.2f} s (script {r.t_foldin_s})')
    check(name, fold, min(fold + 15.0, land), r, 'attach')
