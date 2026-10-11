#!/usr/bin/env python3
"""tools/release_transient.py -- the ring's transient at a release against the planned tension step (desk analysis).

    python3 tools/release_transient.py [--out results/analysis/release_transient] [--results /path/to/results]

For every twin and rig release in RUNS (the 8 Oct pairs, the 7 Oct rig detaches including the r0013 capsize), from
the logs already on disk and nothing else:
  * the planner's planned tensions per drone before the resize and in its first second after it (log.csv t0..t3,
    per slot, mapped to drones through slot2drone), so each survivor's planned step in newtons;
  * the static prediction of that step from the layout alone (tools/admissibility_map: the leaver's level-hover
    share handed to the survivors), to show that the resize is the static redistribution;
  * the detection delay: on the rig the bag's DETECTED event against RELEASE; in the twin the planner's first
    smaller problem against RELEASE at the log's 10 Hz (an announced DETACH has none); and the time from the event
    to the resize in both;
  * the measured transient from the ring's motion-capture pose: the peak tilt rise above the 3 s before the event,
    the absolute peak, and the height dip, all within 5 s of the event; the survivors' support margin and largest
    gap after the leaver (metrics.cog_margin).
Clocks: events.csv and run.csv count from the runner's start, the planner log on the sim clock (the rig's on the
wall clock), so the two are aligned on the ring's lift-off (first 3 cm above its rest), as tools/detach_tensions.py
does. A rig flight without a converted run directory (r0013) is read from its planner log alone, at 10 Hz.
Rig layouts and leavers are typed here from the registry rows they cite; the plates are given as angles
anticlockwise from the ring's +x (the 8 Oct sheets count plates clockwise, so rig plate 9 is 90 deg).
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from metrics import cog_margin                       # noqa: E402
import admissibility_map as am                       # noqa: E402

G = 9.81
RING_KG = 0.86
PRE_S, POST_S, STEP_S = 3.0, 5.0, 1.0

# run id -> where it is, its layout (deg, anticlockwise) and how the release was commanded.
# rig: 'run' is the converted run directory (tools/rig_to_run.py), 'planner' the T2 planner log folder.
RUNS = [
    # twin, 8 Oct pairs (registry R1140/R1145, R1206/R1211, R1207/R1212, R1208/R1213; layouts from manifest.json)
    dict(run='R1207', source='twin', kind='announced', path='hover'),
    dict(run='R1212', source='twin', kind='announced', path='hover'),
    dict(run='R1140', source='twin', kind='unannounced', path='hover'),
    dict(run='R1145', source='twin', kind='unannounced', path='hover'),
    dict(run='R1206', source='twin', kind='unannounced', path='hover'),
    dict(run='R1211', source='twin', kind='unannounced', path='hover'),
    dict(run='R1208', source='twin', kind='unannounced', path='circle'),
    dict(run='R1213', source='twin', kind='unannounced', path='circle'),
    # rig, 7 Oct (registry RIG-1007-r0012, r0013: plates 1/3/5/9 and 1/3/6/9 in the anticlockwise numbering,
    # the leaver on plate 3 = 90 deg; round 1: planned pull, rods planned at 45 deg, reference integral)
    dict(run='RIG-1007-r0012', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-07', rig_name='r0012'),
    dict(run='RIG-1007-r0013', source='rig', kind='announced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-07', rig_name='r0013', note='capsized: a survivor magnet let go'),
    # rig, 8 Oct (registry RIG-1008-r302_3, r303_4, r304_3, r305: plates 3/11/7/9 and 3/11/6/9 clockwise,
    # drone 4 leaves from plate 9 = 90 deg; measured pull, rods planned at 60 deg, reference integral)
    dict(run='RIG-1008-r302_3', source='rig', kind='announced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r302_3'),
    dict(run='RIG-1008-r303_4', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r303_4'),
    dict(run='RIG-1008-r304_3', source='rig', kind='unannounced', path='hover', azimuths=(30, 90, 180, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r304_3'),
    dict(run='RIG-1008-r305', source='rig', kind='unannounced', path='circle', azimuths=(30, 90, 150, 270),
         leaver_deg=90, rig_dir='results/rig/2026-10-08', rig_name='r305'),
]


def liftoff_time(t, z, rise=0.03):
    t, z = np.asarray(t, float), np.asarray(z, float)
    rest = np.nanmedian(z[:20])
    idx = np.where(z > rest + rise)[0]
    return float(t[idx[0]]) if len(idx) else np.nan


def load_twin(spec, results):
    d = glob.glob(os.path.join(results, '*', f"{spec['run']}_*"))[0]
    import json
    man = json.load(open(os.path.join(d, 'manifest.json')))
    la = man.get('launch_args', {})
    if not isinstance(la, dict):
        la = dict(a.split(':=', 1) for a in la if ':=' in a)
    az = tuple(float(v) for v in str(la['attach_azimuths_deg']).split(','))
    planner = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    return planner, run, ev, az


def load_rig(spec, results_root):
    base = os.path.join(results_root, spec['rig_dir'])
    planner = pd.read_csv(glob.glob(os.path.join(base, f"{spec['rig_name']}_logs", 'logs', 'dissipative_planner',
                                                 '*', 'log.csv'))[0])
    run_dir = os.path.join(base, f"{spec['rig_name']}_run", 'logs')
    run = ev = None
    if os.path.isdir(run_dir):
        run = pd.read_csv(os.path.join(run_dir, 'run.csv')).drop_duplicates('t')
        ev = pd.read_csv(os.path.join(run_dir, 'events.csv'))
    return planner, run, ev, tuple(spec['azimuths'])


def slot_tensions(row, n):
    return np.array([float(row[f't{k}']) for k in range(n)])


def analyse(spec, results_root):
    if spec['source'] == 'twin':
        planner, run, ev, az = load_twin(spec, results_root)
    else:
        planner, run, ev, az = load_rig(spec, os.path.dirname(results_root))
    P = planner.drop_duplicates('sim_time').reset_index(drop=True)
    n_before = int(P.n.iloc[0]) if 'n' in P else 4
    # the resize: the first planner row with fewer drones
    n0 = int(P.n.max())
    after = P[P.n < n0]
    if after.empty:
        return dict(run=spec['run'], note='no resize in the planner log')
    i_res = int(after.index[0])
    t_res = float(P.sim_time.iloc[i_res])
    pre = P[(P.sim_time >= t_res - 2.0) & (P.sim_time < t_res - 0.2)]
    post = P[(P.sim_time >= t_res) & (P.sim_time < t_res + STEP_S)]
    slots_before = [int(v) for v in str(pre.slot2drone.iloc[-1]).split()]
    slots_after = [int(v) for v in str(post.slot2drone.iloc[0]).split()]
    leaver = [dr for dr in slots_before if dr not in slots_after][0]
    t_before = {dr: float(np.nanmean(pre[f't{k}'].astype(float))) for k, dr in enumerate(slots_before)}
    t_after = {dr: float(np.nanmean(post[f't{k}'].astype(float))) for k, dr in enumerate(slots_after)}
    elev = float(np.nanmean(pre[[f'elev{k}' for k in range(n0)]].astype(float).to_numpy()))
    az_of = {dr: az[k] for k, dr in enumerate(slots_before)}
    leaver_deg = az_of[leaver]
    survivors = [dr for dr in slots_before if dr != leaver]
    planned_step = {dr: t_after[dr] - t_before[dr] for dr in survivors}
    # the static prediction from the layout alone: shares of the weight handed over, as tensions at the plan angle
    stat = {r['leaver_deg']: r for r in am.removal_rows('x', az, 0.225, elev if np.isfinite(elev) else 60.0)}
    srow = stat[leaver_deg]
    static_step = {}
    for item in srow['survivor_steps'].split():
        a, v = item.split(':')
        static_step[float(a)] = float(v) * RING_KG * G / np.sin(np.deg2rad(elev if np.isfinite(elev) else 60.0))
    margin, gap, ok = cog_margin(am.rho_of([az_of[dr] for dr in survivors], 0.225))
    out = dict(run=spec['run'], source=spec['source'], kind=spec['kind'], path=spec['path'],
               layout_deg=' '.join(str(int(a)) for a in az), leaver_deg=int(leaver_deg), elev_log_deg=round(elev, 1),
               gap_after_deg=round(gap, 1), margin_after_mm=round(1000 * margin, 1),
               leaver_t_before_N=round(t_before[leaver], 2),
               planned_step_max_N=round(max(planned_step.values()), 2),
               planned_step_sum_N=round(sum(planned_step.values()), 2),
               static_step_max_N=round(max(static_step.values()), 2),
               static_step_sum_N=round(sum(static_step.values()), 2),
               planned_steps=' '.join(f'{int(az_of[dr])}:{planned_step[dr]:+.2f}' for dr in survivors),
               static_steps=' '.join(f'{int(a)}:{v:+.2f}' for a, v in sorted(static_step.items())),
               note=spec.get('note', ''))
    # measured transient, on the planner clock
    if run is not None:
        off = liftoff_time(P.sim_time, P.load_z) - liftoff_time(run.t, run.payload_z)
        t = run.t.to_numpy() + off
        tilt = run.payload_tilt_deg.to_numpy(float)
        z = run.payload_z.to_numpy(float)
        rel = ev[ev.event == 'RELEASE']
        det = ev[ev.event == 'DETACH']
        detd = ev[ev.event == 'DETECTED']
        if spec['kind'] == 'unannounced' and not rel.empty:
            t_ev = float(rel.sim_time.iloc[0]) + off
            if not detd.empty:                        # the rig bag logs the planner's detection to the millisecond
                out['detection_s'] = round(float(detd.sim_time.iloc[0]) - float(rel.sim_time.iloc[0]), 2)
                out['detection_source'] = 'DETECTED event'
            else:                                     # the twin runner logs no DETECTED: the planner's first smaller
                out['detection_s'] = round(t_res - t_ev, 2)      # problem, at the log's 10 Hz, clock-aligned
                out['detection_source'] = 'planner resize, 10 Hz'
        else:
            t_ev = float(det.sim_time.iloc[0]) + off if not det.empty else t_res
            out['detection_s'] = np.nan
            out['detection_source'] = ''
        out['clock_offset_s'] = round(off, 2)
    else:
        t, tilt, z = P.sim_time.to_numpy(float), P.tilt_deg.to_numpy(float), P.load_z.to_numpy(float)
        t_ev = t_res
        out['detection_s'] = np.nan
        out['note'] = (out['note'] + '; planner log only (10 Hz)').strip('; ')
    pre_m = (t >= t_ev - PRE_S) & (t < t_ev)
    post_m = (t >= t_ev) & (t <= t_ev + POST_S)
    tilt_pre = float(np.nanmean(tilt[pre_m]))
    out['tilt_pre_mean_deg'] = round(tilt_pre, 2)
    out['tilt_peak_deg'] = round(float(np.nanmax(tilt[post_m])), 2)
    out['tilt_rise_deg'] = round(float(np.nanmax(tilt[post_m]) - tilt_pre), 2)
    out['dip_cm'] = round(100 * float(np.nanmean(z[pre_m]) - np.nanmin(z[post_m])), 1)
    out['resize_after_event_s'] = round(t_res - t_ev, 2)
    return out


def plot(rows, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in rows if 'tilt_rise_deg' in r]
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6))
    mk = {('twin', 'announced'): 'o', ('twin', 'unannounced'): 's', ('rig', 'announced'): 'o', ('rig', 'unannounced'): 's'}
    col = {'twin': '0.55', 'rig': 'C1'}
    for r in rows:
        kw = dict(marker=mk[(r['source'], r['kind'])], color=col[r['source']], ms=6,
                  mfc=col[r['source']] if r['source'] == 'rig' else 'none', ls='none')
        axes[0].plot(r['planned_step_max_N'], r['tilt_rise_deg'], **kw)
        axes[1].plot(r['margin_after_mm'], r['tilt_rise_deg'], **kw)
        axes[2].plot(r['static_step_max_N'], r['planned_step_max_N'], **kw)
        if r['tilt_rise_deg'] > 25:
            axes[0].annotate('magnet let go', (r['planned_step_max_N'], r['tilt_rise_deg']), fontsize=7,
                             xytext=(4, -10), textcoords='offset points')
    axes[0].set_xlabel('largest planned tension step at the resize (N)')
    axes[0].set_ylabel('ring tilt rise within 5 s (deg)')
    axes[1].set_xlabel('survivors\' support margin (mm)')
    axes[1].set_ylabel('ring tilt rise within 5 s (deg)')
    lim = max(max(r['static_step_max_N'], r['planned_step_max_N']) for r in rows) * 1.1
    axes[2].plot([0, lim], [0, lim], '-', color='0.7', lw=0.8)
    axes[2].set_xlabel('static prediction of the largest step (N)')
    axes[2].set_ylabel('planner\'s largest step (N)')
    axes[0].set_title('filled: rig; open: twin; square: unannounced', fontsize=8)
    for ax in axes:
        ax.grid(True, lw=0.3, alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'release_transient.png'), dpi=150)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'release_transient'))
    ap.add_argument('--results', default='/home/wesley/multi_drone_control/results',
                    help='the results root with the run directories (read only)')
    ap.add_argument('--no-plot', action='store_true')
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for spec in RUNS:
        try:
            rows.append(analyse(spec, a.results))
        except Exception as e:                       # one bad run must not hide the rest
            rows.append(dict(run=spec['run'], note=f'failed: {type(e).__name__}: {e}'))
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(a.out, 'table.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    show = ['run', 'kind', 'path', 'layout_deg', 'gap_after_deg', 'margin_after_mm', 'elev_log_deg',
            'leaver_t_before_N', 'planned_step_max_N', 'static_step_max_N', 'detection_s', 'resize_after_event_s',
            'tilt_rise_deg', 'tilt_peak_deg', 'dip_cm', 'note']
    with open(os.path.join(a.out, 'table.md'), 'w') as f:
        f.write('| ' + ' | '.join(show) + ' |\n|' + '---|' * len(show) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(str(r.get(k, '')) for k in show) + ' |\n')
    print(open(os.path.join(a.out, 'table.md')).read())
    if not a.no_plot:
        plot(rows, a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
