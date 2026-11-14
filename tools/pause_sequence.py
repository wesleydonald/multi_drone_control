#!/usr/bin/env python3
"""tools/pause_sequence.py -- the paused drop-off's order of events in a twin run (13 Nov 2026, Te-Jen's Codex entry 16):
the ring's reference and measured speeds and the carriers' speeds (each smoothed over 0.5 s) before his TAKEOFF, when our
attach pause starts, when the ring reference and the measured ring and carriers are at rest (< 0.01 m/s), when his C++
transit starts and the ring's speeds then, the largest ring speeds from his transit to his LANDED_DISARMED (the pause
held through the drop, the return and his landing), and when the pause ends ("attach pause over") against his
LANDED_DISARMED or our LAND. Times are sim seconds from his TAKEOFF.
    python3 tools/pause_sequence.py R4435 ...
"""
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import m1_metrics as M  # noqa: E402


def speed(t, xy, win=0.5):
    v = np.r_[0.0, np.linalg.norm(np.diff(xy, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)]
    k = max(1, int(round(win / max(np.median(np.diff(t)), 1e-3))))
    return np.convolve(v, np.ones(k) / k, mode='same')


for rid in sys.argv[1:]:
    r = M.Run(rid)
    t = r.tt
    tk = r.t('PHASE:TAKEOFF')
    ref = speed(t, r.xyz('payload_ref_')[:, :2])
    meas = speed(t, r.xyz('payload_')[:, :2])
    car = np.max([speed(t, r.xyz(f'd{i}_')[:, :2]) for i in range(len(r.az))], axis=0)
    at = lambda arr, x: float(arr[int(np.argmin(np.abs(t - x)))])
    pre = (t >= tk - 3) & (t < tk)

    def rest_after(arr, a):
        i = np.where((t > a) & (arr < 0.01))[0]
        return round(float(t[i[0]] - tk), 1) if len(i) else None
    tr = r.t('PHASE:TRANSIT_TO_DROP_POINT', after=tk)
    ld = r.t('PHASE:LANDED_DISARMED', after=tk)
    land = r.t('LAND', after=tk)
    end = min(x for x in (ld, land, float(t[-1])) if x is not None)
    hold = (t >= (tr or end)) & (t < end)
    # the pause messages: launch.log wall stamps, placed in sim time by the LAND command's sim time
    log = open(os.path.join(r.d, 'logs', 'launch.log'), errors='ignore').read()
    wl = re.search(r"\[(\d{10}\.\d+)\] \[[a-z_]+\]: Fleet command received: 'LAND'", log)
    ps = [float(x) for x in re.findall(r'\[(\d{10}\.\d+)\] \[dissipative_planner\]: \[dissipative\] attach pause:', log)]
    po = [float(x) for x in re.findall(r'\[(\d{10}\.\d+)\] \[dissipative_planner\]: \[dissipative\] attach pause over', log)]
    print(f'## {rid} ({r.config})')
    print(f'- before his TAKEOFF (3 s): ring reference {ref[pre].mean():.3f} m/s, measured ring {meas[pre].mean():.3f}, '
          f'fastest carrier {car[pre].mean():.3f}')
    print(f'- at rest after his TAKEOFF (< 0.01 m/s): reference +{rest_after(ref, tk)} s, measured ring +{rest_after(meas, tk)} s, '
          f'carriers +{rest_after(car, tk)} s; attach pause messages (wall, from the first): start {len(ps)}, over {len(po)}')
    if tr:
        print(f'- his C++ transit starts +{tr - tk:.1f} s: ring reference {at(ref, tr):.3f}, measured {at(meas, tr):.3f}, '
              f'carriers {at(car, tr):.3f} m/s')
    if hold.any():
        print(f'- from his transit to {"his LANDED_DISARMED" if ld and end == ld else "our LAND" if land and end == land else "the end"} '
              f'(+{end - tk:.1f} s): ring reference max {ref[hold].max():.3f}, measured max {meas[hold].max():.3f}, '
              f'carriers max {car[hold].max():.3f} m/s')
    if ld:
        aft = (t > ld) & (t < (land or t[-1]))
        print(f'- after his LANDED_DISARMED (+{ld - tk:.1f} s) to our LAND: ring reference max {ref[aft].max() if aft.any() else float("nan"):.3f} m/s (the resume)')
    if po and wl:
        print(f'- "attach pause over" {po[-1] - float(wl.group(1)):+.2f} s (wall) from our LAND command')
