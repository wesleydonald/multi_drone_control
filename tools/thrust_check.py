#!/usr/bin/env python3
"""tools/thrust_check.py -- how much thrust the airframes really have (rig thrust check, 2026-09-30).

    python3 tools/thrust_check.py --free results/rig/2026-09-30/t1_free_logs \\
        --loaded results/rig/2026-09-30/t2_loaded_logs --hung-kg 0.150 [--masses 0.62,0.60,0.61,0.63]

Two free hovers of the same airframes, rods on: one plain (--free), one with a known mass taped to each rod tip
(--loaded, --hung-kg). In steady hover throttle u carries the weight, so with a linear thrust map
    u_free = m g / T1,   u_loaded = (m + dm) g / T1        (T1 = thrust at full throttle)
which gives, per drone, the thrust per unit throttle kT = g / u_free, the full-throttle thrust T1 = m g / u_free and
the airframe mass m = dm u_free / (u_loaded - u_free) with no scale. With --masses (weighed) the tool also checks the
thrust map is linear: predicted u_loaded = u_free (m + dm) / m against the measured one. Last, the throttle each drone
needs to carry the 0.86 kg ring on 45 deg rods for the three layouts, against the tracker's 0.6 cap.

Folders are MDC_RUN_DIR folders (logs/controller_quad_load/planner_droneN_*/log.csv) or the tracker log dirs themselves.
Steady hover = the plateau of the reference height, drone within 8 cm of it, after 3 s there."""
import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd

G = 9.81
CAP = 0.6
LAYOUTS = (('three on 1/5/9', [30, 150, 270]), ('even four 0/3/6/9', [0, 90, 180, 270]),
           ('four on 1/3/5/9 (M1)', [30, 90, 150, 270]))


def tracker_logs(folder):
    pats = [os.path.join(folder, 'logs', 'controller_quad_load', 'planner_drone*', 'log.csv'),
            os.path.join(folder, 'planner_drone*', 'log.csv'), os.path.join(folder, 'log.csv')]
    for p in pats:
        found = sorted(glob.glob(p))
        if found:
            out = {}
            for f in found:
                tag = os.path.basename(os.path.dirname(f))
                i = int(tag.split('planner_drone')[1].split('_')[0]) if 'planner_drone' in tag else 0
                out[i] = f
            return out
    return {}


def hover_throttle(path):
    """(median throttle, samples, fraction at the cap) over the steady part of the hover plateau."""
    d = pd.read_csv(path)
    d = d[d.u2 > 0.1]
    if len(d) < 50:
        return math.nan, 0, math.nan
    top = d.ref_z.max()
    plateau = d[(d.ref_z > top - 0.01)]
    if len(plateau) < 50:
        return math.nan, 0, math.nan
    t0 = plateau.sim_time.iloc[0] + 3.0
    steady = plateau[(plateau.sim_time > t0) & ((plateau.pose_z - plateau.ref_z).abs() < 0.08)]
    if len(steady) < 25:
        return math.nan, len(steady), math.nan
    return float(steady.u2.median()), len(steady), float((steady.u2 >= CAP - 0.001).mean())


def carry_throttle(masses, u_free, az, ring=0.86):
    try:
        from headroom import predict
    except ImportError:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from headroom import predict
    return [u for _, _, u in predict(masses, az, ring, u_free=u_free)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--free', required=True, help='folder of the plain free hover')
    ap.add_argument('--loaded', help='folder of the free hover with the hung mass')
    ap.add_argument('--hung-kg', type=float, help='the taped-on mass (kg), weighed')
    ap.add_argument('--masses', help='weighed airframe masses (kg) with pack, rod, magnet, drone order')
    ap.add_argument('--ring', type=float, default=0.86)
    a = ap.parse_args(argv)

    free = tracker_logs(a.free)
    if not free:
        sys.exit(f'no tracker logs under {a.free}')
    loaded = tracker_logs(a.loaded) if a.loaded else {}
    weighed = [float(v) for v in a.masses.split(',')] if a.masses else None
    ids = sorted(free)
    print(f'{"drone":>5} {"u_free":>7} {"u_hung":>7} {"kT":>6} {"m implied":>10} {"m weighed":>10} '
          f'{"T1 (kgf)":>9} {"u_hung pred":>11}  notes')
    m_use, uf_use = [], []
    for i in ids:
        uf, n, cap_f = hover_throttle(free[i])
        ul, nl, cap_l = hover_throttle(loaded[i]) if i in loaded else (math.nan, 0, math.nan)
        kt = G / uf if uf == uf else math.nan
        m_imp = (a.hung_kg * uf / (ul - uf)) if (a.hung_kg and ul == ul and ul > uf) else math.nan
        m_w = weighed[i] if weighed and i < len(weighed) else math.nan
        m = m_w if m_w == m_w else m_imp
        t1 = m / uf if (m == m and uf == uf) else math.nan
        pred = uf * (m_w + a.hung_kg) / m_w if (a.hung_kg and m_w == m_w and uf == uf) else math.nan
        notes = []
        if n == 0:
            notes.append('no steady hover found')
        if cap_f == cap_f and cap_f > 0.05 or cap_l == cap_l and cap_l > 0.05:
            notes.append('AT THE 0.6 CAP part of the time')
        if pred == pred and ul == ul and abs(ul - pred) > 0.03:
            notes.append(f'thrust map not linear here ({ul - pred:+.3f})')
        print(f'{i:>5} {uf:>7.3f} {ul:>7.3f} {kt:>6.1f} {m_imp:>10.3f} {m_w:>10.3f} {t1:>9.2f} {pred:>11.3f}  '
              + '; '.join(notes))
        m_use.append(m)
        uf_use.append(uf)
    if all(v == v for v in m_use + uf_use):
        print(f'\nring {a.ring:.2f} kg on 45 deg rods, throttle needed per drone (tracker cap {CAP}):')
        for name, az in LAYOUTS:
            if len(az) > len(m_use):
                continue
            n = len(az)
            us = carry_throttle(m_use[:n], uf_use[:n], az, a.ring)
            worst = max(us)
            verdict = 'fits' if worst < CAP - 0.03 else ('at the cap' if worst < CAP else 'OVER the cap')
            print(f'  {name:24s} ' + ' '.join(f'{u:.3f}' for u in us) + f'   worst {worst:.3f}: {verdict}')
    else:
        print('\n(no carry prediction: needs a mass per drone, weighed or implied by the hung-mass hover)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
