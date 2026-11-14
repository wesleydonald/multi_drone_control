#!/usr/bin/env python3
"""tools/rig_wed_readout.py -- what a real flight's logs say about the cable lengths and the sideways force.

    python3 tools/rig_wed_readout.py results/rig/2026-11-04 h3t h3t180 h4m1

Per flight, from the planner's log only:
  lengths   the hand-over reading, what rod_len_reread would have read (mod.RodReread on the last 0.5 s before the height ramp) and what
            the rods read at the top. R1 of card 2026-10-31_rod_len_reread: the mean over the rods within 3 mm, at most one rod over 5 mm.
  ring      height error, tilt (x, y), offset from its point, the last half of the steady stretch at the top.
  force     the integral's force and F-inf = F + K x offset (K the ring's sideways stiffness, N/m: 7.3 at 3 carriers, 9.6 at 4, on 1 m and
            1.15 kg, docs/analysis/why_integral_real_2026-10-31.md). Compare h3t with h3t180: room-fixed stays, ring-fixed flips.
  tick      the planner's tick and solve, p50, p95, max (ms), the share of ticks over the 100 ms period.
Exploratory: read the flight's own log before believing a line here.
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'mpc_planner'))
from mpc_planner.rod_reread import RodReread  # noqa: E402

K_XY = {3: 7.3, 4: 9.6}


def read_flight(day, fl):
    fs = glob.glob(os.path.join(day, f'{fl}_logs', 'logs', 'dissipative_planner', '*', 'log.csv'))
    if not fs:
        return f'{fl}: no planner log'
    d = pd.read_csv(fs[0], low_memory=False)
    txt = open(os.path.join(day, f'{fl}.log'), errors='replace').read() if os.path.exists(os.path.join(day, f'{fl}.log')) else ''
    m = re.search(r'measure_rod_len: rods \[([^\]]+)\] m from mocap at handover', txt)
    n = int(d.n.max())
    d = d[d.n == n]
    if d.load_z.max() < 0.3:
        return f'{fl}: the ring never lifted'
    t = d.sim_time.to_numpy(float)
    L = np.array([d[f'len{i}'].to_numpy(float) for i in range(n)]).T
    out = [f'{fl}: n {n}, yaw datum {re.findall(r"load yaw datum latched at ([-+0-9.]+)", txt)[:1]}']
    tgt = float(d.z_tgt.max())
    top = d[(d.load_z > tgt - 0.1) & (d.land.astype(float) == 0) & (d.phase.astype(str) == 'planner')]
    if len(top) < 20:
        return '\n'.join(out + ['  under 2 s at the top: nothing to read'])
    tt = top.sim_time.to_numpy(float)
    half = top[top.sim_time >= tt[len(tt) // 2]]
    flown = L[(t >= tt[len(tt) // 3]) & (t <= tt[-1])].mean(0)
    on = np.nonzero(d.lift_progress.to_numpy(float) > 1e-4)[0]
    if m and on.size:
        lat = np.array([float(v) for v in m.group(1).split(',')])
        rr = RodReread()
        for x in L[(t >= t[on[0]] - 0.5) & (t < t[on[0]])]:
            rr.sample(x)
        rr.finish(0.0, lat)
        out.append(f'  lengths: hand-over {np.round(lat, 3)}, flown {np.round(flown, 3)} (flown - hand-over {np.round(1e3 * (flown - lat), 1)} mm)')
        if rr.target is None:
            out.append(f'  reading: {rr.state}: {rr.why}')
        else:
            e = 1e3 * (np.array(rr.target) - flown)
            ok = abs(e.mean()) <= 3.0 and (np.abs(e) > 5.0).sum() <= 1
            out.append(f'  reading: {np.round(rr.target, 3)}, reading - flown {np.round(e, 1)} mm, mean {e.mean():+.1f}: R1 {"HOLDS" if ok else "MISSES"}')
    q = half
    dz = 1e3 * float((q.load_z - q.z_tgt).mean())
    off = 1e3 * np.array([float((q.load_x - q.hover_x).mean()), float((q.load_y - q.hover_y).mean())])
    fxy = np.array([float(q.int_fx.mean()), float(q.int_fy.mean())])
    k = K_XY.get(n)
    finf = None if k is None else fxy + k * off / 1e3
    out.append(f'  ring: {tt[-1] - tt[0]:.0f} s at the top, height {dz:+.1f} mm, tilt mean {float(q.tilt_deg.mean()):.2f} deg, off {np.round(off, 1)} mm')
    out.append(f'  force: F {np.round(fxy, 3)} N, z {float(q.int_fz.mean()):+.2f} N, F-inf {None if finf is None else np.round(finf, 3)} N')
    tk = top.tick_ms.dropna()
    sv = top.solve_ms.dropna()
    out.append(f'  tick: {tk.quantile(.5):.1f}, {tk.quantile(.95):.1f}, {tk.max():.1f} ms, over 100 ms {100 * float((tk > 100).mean()):.1f} %; solve {sv.quantile(.5):.1f}, {sv.quantile(.95):.1f} ms')
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('day', help='results/rig/<date>')
    ap.add_argument('flights', nargs='+')
    a = ap.parse_args()
    for fl in a.flights:
        print(read_flight(a.day, fl))


if __name__ == '__main__':
    main()
