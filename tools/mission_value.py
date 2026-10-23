#!/usr/bin/env python3
"""tools/mission_value.py -- what an in-flight reconfiguration buys against landing the team to swap a UAV.

    python3 tools/mission_value.py [--out results/analysis/mission_value] [--swap 30 60 120]

Desk analysis from the rig's own timings, no flight and no simulation.
  * Landing to swap: the team lands (LAND to LANDED), the UAV is swapped on the ground (a time the operator sets,
    `--swap`), the team takes off again (TAKEOFF to HANDOVER, then the lift to the carry height) and settles. The
    ring stops moving for all of it. Segment times come from the converted 7-8 Oct rig flights (events.csv and the
    ring height in run.csv).
  * In flight: the ring keeps flying while one UAV leaves, does its errand and rejoins. The mission is not paused,
    but the carriers hover for the whole errand. The errand's length comes from the twin's full M1 (registry rows
    R1573 hover and R1579 circle: release to the proven attach), the only place the whole sequence has flown.
  * The cost side: carrier hover-seconds, and the carriers' mean throttle with four and with three on the ring from
    the same rig flights (the thrust each survivor takes on while the leaver is away). Conversion to joules is left
    to a measured hover current; the proxy here is drone-seconds of hover at the logged throttle.
The output is a table of pause time against hover-seconds for each swap time, and the per-segment rig timings.
"""
import argparse
import csv
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# converted rig flights with a full TAKEOFF .. LANDED record (tools/rig_to_run.py); registry RIG-1007-*, RIG-1008-*
RIG_RUNS = {
    'r0012': 'results/rig/2026-10-07/r0012_run',
    'r206e60': 'results/rig/2026-10-07/r206e60_run',
    'r302_3': 'results/rig/2026-10-08/r302_3_run',
    'r303_4': 'results/rig/2026-10-08/r303_4_run',
    'r304_3': 'results/rig/2026-10-08/r304_3_run',
    'r305': 'results/rig/2026-10-08/r305_run',
}
TARGET_Z = 1.0
# the twin's full M1 stage times (s, run clock), by hand from run.csv per the registry rows:
# R1573 (hover): DETACH 29.93, release 36.27, pickup weld 51.10, drop 112.67, contact 125.63, proof +0.59
# R1579 (circle): contact 151.05, proof +0.90 (stages otherwise as R1573; the orbit adds the circle's start)
M1_TWIN = {
    'R1573 (hover)': dict(release=36.27, contact=125.63, proof=125.63 + 0.59),
    'R1579 (circle)': dict(release=36.27, contact=151.05, proof=151.05 + 0.90),
}


def segments(run_dir):
    ev = pd.read_csv(os.path.join(run_dir, 'logs', 'events.csv'))
    run = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv')).drop_duplicates('t')
    t_of = lambda name: float(ev[ev.event == name].sim_time.iloc[0]) if (ev.event == name).any() else np.nan
    takeoff, handover, land, landed = t_of('TAKEOFF'), t_of('HANDOVER'), t_of('LAND'), t_of('LANDED')
    release = t_of('DETACH') if np.isfinite(t_of('DETACH')) else t_of('RELEASE')
    z = run.payload_z.to_numpy(float)
    t = run.t.to_numpy(float)
    reached = t[(t > handover) & (z >= 0.95 * TARGET_Z)]
    t_lift = float(reached[0]) if len(reached) else np.nan
    n = len([c for c in run.columns if c.endswith('_thr')])
    thr = run[[f'd{i}_thr' for i in range(n)]].to_numpy(float)
    def mean_thr(t0, t1, drones):
        m = (t >= t0) & (t <= t1)
        return float(np.nanmean(thr[m][:, drones])) if m.any() else np.nan
    carry4 = mean_thr(t_lift + 3.0, (release if np.isfinite(release) else land) - 1.0, list(range(n)))
    out = dict(run=os.path.basename(run_dir).replace('_run', ''),
               creep_s=round(handover - takeoff, 1), lift_s=round(t_lift - handover, 1),
               takeoff_to_height_s=round(t_lift - takeoff, 1), land_s=round(landed - land, 1),
               thr_four=round(carry4, 3))
    if np.isfinite(release) and n == 4:
        leaver = None
        arg = str(ev[ev.event.isin(['DETACH', 'RELEASE'])].arg.iloc[0])
        for i in range(n):
            if f'/drone_{i}' in arg or arg.strip() in (str(i), f'drone {i}'):
                leaver = i
        if leaver is None:
            leaver = 3                                   # drone 4 (/drone_3) left in every 8 Oct flight
        surv = [i for i in range(n) if i != leaver]
        out['thr_three_after'] = round(mean_thr(release + 3.0, land - 1.0, surv), 3)
        out['thr_survivors_before'] = round(mean_thr(t_lift + 3.0, release - 1.0, surv), 3)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'mission_value'))
    ap.add_argument('--results-repo', default='/home/wesley/multi_drone_control', help='read-only results root')
    ap.add_argument('--swap', type=float, nargs='+', default=[30.0, 60.0, 120.0], help='ground swap times (s)')
    ap.add_argument('--settle', type=float, default=5.0, help='settle after the lift before the path resumes (s)')
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for name, rel in RIG_RUNS.items():
        d = os.path.join(a.results_repo, rel)
        if os.path.isdir(d):
            try:
                rows.append(segments(d))
            except Exception as e:
                rows.append(dict(run=name, note=f'failed: {type(e).__name__}: {e}'))
    with open(os.path.join(a.out, 'rig_segments.csv'), 'w', newline='') as f:
        keys = []
        for r in rows:
            keys += [k for k in r if k not in keys]
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    good = [r for r in rows if 'creep_s' in r]
    med = {k: float(np.nanmedian([r[k] for r in good if k in r and np.isfinite(r[k])])) for k in
           ('creep_s', 'lift_s', 'takeoff_to_height_s', 'land_s', 'thr_four')}
    med['thr_three_after'] = float(np.nanmedian([r['thr_three_after'] for r in good if 'thr_three_after' in r]))
    med['thr_survivors_before'] = float(np.nanmedian([r['thr_survivors_before'] for r in good if 'thr_survivors_before' in r]))
    print('rig segments (s), one flight each:')
    for r in good:
        print('  ' + ', '.join(f'{k} {v}' for k, v in r.items()))
    print('medians: ' + ', '.join(f'{k} {v:.2f}' for k, v in med.items()))
    cycle = med['land_s'] + med['takeoff_to_height_s'] + a.settle
    lines = []
    for m1, st in M1_TWIN.items():
        away = st['proof'] - st['release']
        for S in a.swap:
            pause_land = med['land_s'] + S + med['takeoff_to_height_s'] + a.settle
            hover_inflight = 3 * away                          # three carriers hover through the errand
            hover_land = 3 * (med['land_s'] + med['takeoff_to_height_s'] + a.settle)   # descent and the new lift
            lines.append(dict(errand=m1, away_s=round(away, 1), swap_s=S, pause_landing_s=round(pause_land, 1),
                              pause_inflight_s=0.0, carrier_hover_s_inflight=round(hover_inflight, 1),
                              carrier_hover_s_landing=round(hover_land, 1),
                              extra_hover_s_inflight=round(hover_inflight - hover_land, 1)))
    with open(os.path.join(a.out, 'value.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(lines[0].keys()))
        w.writeheader()
        w.writerows(lines)
    print(f'\nland-and-swap cycle without the swap itself: {cycle:.1f} s (land {med["land_s"]:.1f} + takeoff to height '
          f'{med["takeoff_to_height_s"]:.1f} + settle {a.settle:.0f})')
    print('errand | away (s) | swap (s) | mission pause: landing (s) | in flight (s) | carrier hover-s: in flight | landing | extra')
    for l in lines:
        print(f"  {l['errand']:15s} {l['away_s']:6.1f} {l['swap_s']:6.0f} {l['pause_landing_s']:8.1f} "
              f"{l['pause_inflight_s']:6.1f} {l['carrier_hover_s_inflight']:8.1f} {l['carrier_hover_s_landing']:8.1f} "
              f"{l['extra_hover_s_inflight']:8.1f}")
    print(f"survivors' mean throttle before the release {med['thr_survivors_before']:.3f}, after (three on the ring) "
          f"{med['thr_three_after']:.3f}; four carrying {med['thr_four']:.3f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
