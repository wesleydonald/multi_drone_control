#!/usr/bin/env python3
"""Pick the best flight per cell (carriers x motion): score every candidate rig flight that flew its motion to the end,
and the twin runs for the cells the rig has not flown, on one rule (best_data.score: clock-free on a path, distance to
the hold point in a hover). Writes selection.json beside this script and prints the candidate table for index.md.

Candidates are the registry's RIG-* carry flights that lifted the ring and flew a motion (VOIDs, flights that never
lifted, and the M1 attach flights are left out); the twin candidates are the 22 Oct baselines at 1 m (b2/b3/b4,
R2329-R2364, R2405-R2434) and the 1 m re-fly's three-quad cells (R2508-R2537).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_data as B  # noqa: E402

RIG = [
    # 2 carriers (22 Oct; the 21 Oct pairs all ended early: rocking, magnet loss)
    '2026-10-22/h2slope', '2026-10-22/h2', '2026-10-22/h2t9p', '2026-10-22/h2m', '2026-10-22/h2rts',
    '2026-10-22/f2slope',
    # 3 carriers (21-22 Oct)
    '2026-10-22/h3', '2026-10-22/h3n', '2026-10-14/lift3_3',
    '2026-10-22/f3', '2026-10-22/f3h', '2026-10-22/f3big', '2026-10-21/f125', '2026-10-21/f125_2', '2026-10-21/f200',
    '2026-10-21/c125_2', '2026-10-21/c200_2', '2026-10-21/c200_4',
    '2026-10-21/l125', '2026-10-21/l200',
    '2026-10-22/ls3', '2026-10-22/ls3c', '2026-10-21/s125', '2026-10-21/s200_3',
    # 4 carriers (7 Oct)
    '2026-10-07/r20004', '2026-10-07/r002', '2026-10-07/r0001', '2026-10-07/r011',
    '2026-10-07/r206e60', '2026-10-07/r006', '2026-10-07/r0005',
    '2026-10-07/r2008e60', '2026-10-07/r0007', '2026-10-07/r000014',
]
TWIN = ['R2329', 'R2330', 'R2331', 'R2332', 'R2336', 'R2337', 'R2338', 'R2339', 'R2348', 'R2349', 'R2350', 'R2351',
        'R2333', 'R2334', 'R2335', 'R2360', 'R2340', 'R2341', 'R2342', 'R2343', 'R2352', 'R2353', 'R2354', 'R2355',
        'R2344', 'R2345', 'R2346', 'R2347', 'R2356', 'R2357', 'R2358', 'R2359', 'R2361', 'R2362', 'R2363', 'R2364',
        'R2405', 'R2406', 'R2407', 'R2408', 'R2409', 'R2410', 'R2411', 'R2412', 'R2413', 'R2414', 'R2415', 'R2416',
        'R2423', 'R2424', 'R2425', 'R2426', 'R2427', 'R2428', 'R2429', 'R2430', 'R2431', 'R2432', 'R2433', 'R2434']
MIN_HOVER_S = 4.0       # a hover scored over less than this is not a hover (h3n: LAND 1-2 s after the lift)


def row(f, s):
    out = dict(key=f.key, source=f.source, n=f.n, motion=f.motion, kind=f.kind, describe=f.describe(),
               cable=f.cable(), err_mm=1000 * s['err'], window_s=s['window'][1] - s['window'][0],
               tilt_peak=s['tilt_peak'], filt=None)
    if f.motion == 'hover':
        out.update(xy_mm=1000 * s['xy'], z_mm=1000 * s['z'])
    else:
        out.update(across_mm=1000 * s['across_rms'], dz_mm=1000 * s['dz_rms'], lag_s=s['lag_s'],
                   size=f.size(), speed=f.speed)
    cfg = os.path.basename(f.run_dir)
    if f.source == 'twin':
        out['config'] = cfg.split('sim_gz_', 1)[-1]
        out['filt'] = 'slope' if cfg.endswith('_slope') else 'ema' if cfg.endswith('_ema') else None
    return out


def main():
    rows = []
    for k in RIG + TWIN:
        try:
            f = B.Flight(k)
            s = B.score(f)
        except Exception as exc:  # noqa: BLE001 -- a flight that cannot be scored is listed, not fatal
            print(f'{k}: not scored ({type(exc).__name__}: {exc})')
            continue
        rows.append(row(f, s))
    for r in rows:
        r['valid'] = not (r['motion'] == 'hover' and r['window_s'] < MIN_HOVER_S)
    json.dump(rows, open(os.path.join(B.HERE, 'candidates.json'), 'w'), indent=1)
    print('| flight | src | n | motion | cable (m) | window (s) | error (mm) | parts (mm) | lag (s) | tilt peak (deg) |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for r in sorted(rows, key=lambda r: (r['n'], r['motion'], r['source'], r['err_mm'])):
        parts = (f"xy {r['xy_mm']:.1f}, z {r['z_mm']:.1f}" if r['motion'] == 'hover'
                 else f"side {r['across_mm']:.1f}, z {r['dz_mm']:.1f}")
        lag = '' if r['motion'] == 'hover' or not math.isfinite(r['lag_s']) else f"{r['lag_s']:+.2f}"
        print(f"| {r['key']}{'' if r['valid'] else ' (short)'} | {r['source']} | {r['n']} | {r['describe']} | "
              f"{r['cable']:.2f} | {r['window_s']:.1f} | {r['err_mm']:.1f} | {parts} | {lag} | {r['tilt_peak']:.2f} |")


if __name__ == '__main__':
    main()
