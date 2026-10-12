#!/usr/bin/env python3
"""tools/resize_doublecount.py -- the double-count test at the resize tick (thesis plan P6; freeze 1.5, Q9: "the resize
tick adds the load change a second time on the measured pull").

    python3 tools/resize_doublecount.py [--runs 2026-10-08/r302_3 2026-10-08/r303_4 ... R1140 ...] [--rig DIR] [--out DIR]

On the measured pull (tracker cable_source mocap) a survivor's node-0 cable term is its measured rod pull; the plan's
tension enters the tracker only through the references (positions, the attitude and throttle feedforward from the
planned thrust, and the horizon's change from node 0). The question: when the resize steps the planned tensions, does
the measured term already hold that change, and does the survivor's commanded thrust then take it a second time?
For each survivor of each rig release, from the logs on disk (no bag needed):
  * the plan's step: the planner's node-0 tension t and its vertical part -t s_z (log.csv t<slot>, tsz<slot>) as the
    mean over the 2 s before the release (to 0.2 s before) and on the first resized row;
  * the measured rod-pull term: the cable acceleration the tracker applied at node 0 (tracker log acx, acy, acz: the
    measured pull, slew-limited), times the drone mass, as a vertical pull (-m a_z), against its mean over the same
    2 s;
  * the commanded thrust m kT u2 (u2 is the tracker's model throttle, kT its thrust_ratio), same baseline;
read at the release (the OFF: RELEASE in the bag, or the planner's own OFF at an announced DETACH), at the resize
(the planner's FLEET RESIZED line), at the new plan's arrival (the stamp of the first resized log row: the node
publishes after that tick's solve), 0.1 s and 0.5 s after it, and as the mean over 1-3 s after it; and when the
ring's T6 tilt rise peaked (tools/release_bands.py) against the plan's arrival.
Two answers per survivor: the share of the plan's vertical step already in the measured term at the resize, and the
change of the commanded thrust across the plan's arrival (mean over 0.08-0.12 s after it minus mean over the 60 ms
before it) beside the plan's step. Times on the rig_to_run run clock (manifest t0_epoch). A twin run (R####) is read
the same way on the runner's clock (the planner log aligned on the ring's lift-off); its console is on wall time, so
its resize is taken as 0.1 s before the plan's arrival.
"""
import argparse
import csv
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
RIG_DEFAULT = '/home/wesley/multi_drone_control/results/rig'
RUNS = ['2026-10-08/r302_3', '2026-10-08/r303_4', '2026-10-08/r304_3', '2026-10-08/r305',
        'R1118', 'R1121', 'R1140', 'R1145', 'R1139', 'R1141']        # the 7 Oct twin arms on the measured pull
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')


def resize_line(t2_log):
    for line in open(t2_log, errors='replace'):
        if 'FLEET RESIZED' in line:
            return float(STAMP.search(line).group(1))
    return None


def liftoff(t, z, rise=0.03):
    t, z = np.asarray(t, float), np.asarray(z, float)
    return float(t[np.nonzero(z > np.nanmedian(z[:20]) + rise)[0][0]])


def analyse(rig, tag):
    if tag.startswith('R'):                           # a twin run: node logs on sim time, events on the runner's
        d = glob.glob(os.path.join(os.path.dirname(rig.rstrip('/')), '2*', tag + '_*'))[0]
        logs = os.path.join(d, 'logs')
        ev = pd.read_csv(os.path.join(logs, 'events.csv'))
        P = pd.read_csv(glob.glob(os.path.join(logs, 'dissipative_planner', '*', 'log.csv'))[0])
        prm = json.load(open(glob.glob(os.path.join(logs, 'dissipative_planner', '*', 'params.json'))[0]))
        run_csv = pd.read_csv(os.path.join(logs, 'run.csv')).drop_duplicates('t')
        t0 = liftoff(P.sim_time, P.load_z) - liftoff(run_csv.t, run_csv.payload_z)
        trackers = os.path.join(logs, 'tracker')
        run, t_res = tag, None                         # the console is on wall time: resize = arrival - 0.1 s
    else:
        date, run = tag.split('/')
        base = os.path.join(rig, date, run)
        t0 = json.load(open(base + '_run/manifest.json'))['t0_epoch']
        ev = pd.read_csv(base + '_run/logs/events.csv')
        P = pd.read_csv(glob.glob(base + '_logs/logs/dissipative_planner/*/log.csv')[0])
        prm = json.load(open(glob.glob(base + '_logs/logs/dissipative_planner/*/params.json')[0]))
        trackers = base + '_logs/logs/tracker'
        t_res = resize_line(base + '.log') - t0
    kind = 'unannounced' if (ev.event == 'RELEASE').any() else 'announced'
    t_off = float(ev[ev.event == ('RELEASE' if kind == 'unannounced' else 'DETACH')].sim_time.iloc[0])
    t_land = float(ev[ev.event == 'LAND'].sim_time.iloc[0])
    P['rt'] = P.sim_time - t0
    n0 = int(P.n.max())
    res = P[(P.n < n0) & (P.rt > t_off - 1.0)].iloc[0]
    t_arr = float(res.rt)
    if t_res is None:
        t_res = t_arr - 0.1
    # when the ring's tilt rise peaked (T6, tools/release_bands.py), against the plan's arrival
    os.environ.setdefault('MDC_RESULTS_ROOT', os.path.dirname(rig.rstrip('/')))
    import release_bands as rb
    m = rb.Run(tag if tag.startswith('R') else os.path.join(rig, tag + '_run')).metrics()
    t_peak = t_off + m['t_peak']
    before = P[(P.rt >= t_off - 2.0) & (P.rt < t_off - 0.2) & (P.n == n0)]
    s2d_b = [int(v) for v in before.slot2drone.iloc[-1].split()]
    s2d_a = [int(v) for v in str(res.slot2drone).split()]
    leaver = [d for d in s2d_b if d not in s2d_a][0]
    rows = []
    for d in s2d_a:
        kb, ka = s2d_b.index(d), s2d_a.index(d)
        tr = glob.glob(os.path.join(trackers, f'drone{d}_*', 'log.csv'))[0]
        T = pd.read_csv(tr).drop_duplicates('sim_time')
        tp = json.load(open(os.path.join(os.path.dirname(tr), 'params.json')))
        m_d = float(prm.get('drone_mass', 0.55))
        kt = float(tp.get('thrust_ratio', 35.2))
        T['rt'] = T.sim_time - t0
        T['fz'] = -m_d * T.acz                      # vertical rod pull on the drone, N (down positive)
        T['thrust'] = m_d * kt * T.u2
        b = T[(T.rt >= t_off - 2.0) & (T.rt < t_off - 0.2)]
        fz0, th0 = float(b.fz.mean()), float(b.thrust.mean())
        at = lambda t, c: float(np.interp(t, T.rt, T[c]))                       # noqa: E731
        mean = lambda a, z, c: float(T[(T.rt >= a) & (T.rt < z)][c].mean())     # noqa: E731
        plan_t0, plan_t1 = float(before[f't{kb}'].mean()), float(res[f't{ka}'])
        plan_v0, plan_v1 = -float(before[f'tsz{kb}'].mean()), -float(res[f'tsz{ka}'])
        dv = plan_v1 - plan_v0
        meas_res = at(t_res, 'fz') - fz0
        thr_jump = mean(t_arr + 0.08, t_arr + 0.12, 'thrust') - mean(t_arr - 0.06, t_arr, 'thrust')
        meas_jump = mean(t_arr + 0.08, t_arr + 0.12, 'fz') - mean(t_arr - 0.06, t_arr, 'fz')
        rows.append(dict(
            run=run, kind=kind, drone=d, plate_az=float(str(prm.get('attach_azimuths_deg')).split(',')[kb]),
            leaver=leaver, t_off=round(t_off, 3), t_resize=round(t_res, 3), t_plan_arrival=round(t_arr, 3),
            dtheta_deg=round(m['dtheta'], 2), dtheta_peak_minus_arrival_s=round(t_peak - t_arr, 2),
            plan_t_before_N=round(plan_t0, 2), plan_t_after_N=round(plan_t1, 2), plan_step_N=round(plan_t1 - plan_t0, 2),
            plan_vert_step_N=round(dv, 2),
            meas_vert_at_off_N=round(at(t_off, 'fz') - fz0, 2),
            meas_vert_at_resize_N=round(meas_res, 2),
            share_in_meas_at_resize=round(meas_res / dv, 2) if abs(dv) > 0.05 else '',
            meas_vert_at_arrival_N=round(at(t_arr, 'fz') - fz0, 2),
            meas_vert_arrival_plus_0p1_N=round(at(t_arr + 0.1, 'fz') - fz0, 2),
            meas_vert_arrival_plus_0p5_N=round(at(t_arr + 0.5, 'fz') - fz0, 2),
            meas_vert_steady_N=round(mean(t_arr + 1.0, min(t_arr + 3.0, t_land), 'fz') - fz0, 2),
            thrust_at_resize_N=round(at(t_res, 'thrust') - th0, 2),
            thrust_step_across_arrival_N=round(thr_jump, 2),
            meas_step_across_arrival_N=round(meas_jump, 2),
            thrust_steady_N=round(mean(t_arr + 1.0, min(t_arr + 3.0, t_land), 'thrust') - th0, 2),
            baseline_vert_pull_N=round(fz0, 2), baseline_thrust_N=round(th0, 2)))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--runs', nargs='+', default=RUNS)
    ap.add_argument('--rig', default=RIG_DEFAULT, help='results/rig with <run>_run, <run>_logs and <run>.log (read only)')
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'p6_2026-10-12', '3_doublecount'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = [r for tag in a.runs for r in analyse(a.rig, tag)]
    with open(os.path.join(a.out, 'doublecount.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    show = ['run', 'kind', 'drone', 'plate_az', 't_off', 't_resize', 't_plan_arrival', 'dtheta_peak_minus_arrival_s',
            'plan_step_N', 'plan_vert_step_N',
            'meas_vert_at_resize_N', 'share_in_meas_at_resize', 'meas_vert_arrival_plus_0p1_N',
            'meas_vert_arrival_plus_0p5_N', 'meas_vert_steady_N', 'thrust_at_resize_N', 'thrust_step_across_arrival_N',
            'thrust_steady_N']
    print('| ' + ' | '.join(show) + ' |')
    print('|' + '---|' * len(show))
    for r in rows:
        print('| ' + ' | '.join(str(r[k]) for k in show) + ' |')


if __name__ == '__main__':
    sys.exit(main())
