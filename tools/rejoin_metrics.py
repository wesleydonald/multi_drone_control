#!/usr/bin/env python3
"""tools/rejoin_metrics.py -- score a departed carrier's rejoin (card docs/experiments/2026-10-23_rejoin_departed.md,
E1 / E2 / E5's bars, frozen 23 Oct; the thesis's tab:res_rejoin).

    python3 tools/rejoin_metrics.py R2596 R2594 R2595 R2602 R2712 R2603 R2713 [--json results/analysis/rejoin_<date>.json]

Per ATTACH (the rejoin card's staged scoring, as tools/x2_metrics.py's rejoin(); every window on run.csv's clock):
  (a) on plate     REJOIN_START -> REJOIN_ON_PLATE (s), bar 30 s
  (b) joint        REJOIN_ON_PLATE -> JOINT_ATTACHED (s), bar 0.5 s, then REJOIN_WELD and FOLD_IN
  (c) fold-in tilt the ring's tilt magnitude peak, REJOIN_WELD .. +15 s (deg), bar 5 deg; beside it the time over 5 deg
      end off      the rod end's largest distance from its plate point through the blend (the planner's REJOIN BLEND
                   OVER line, cm), bar 2 cm
      tension      the rejoiner's planned tension at the blend's end against the balanced share the planner logged at the
                   fold-in (RESIZE_INPUTS t_nom), bar 10 %
  lean             the ring's mean tilt magnitude over the last 10 s before LAND (deg), bar 2 deg
  faults           no drop, fleet abort, REJOIN REFUSED or REJOIN ABORT; every quad lands (disarmed within 3 cm of its rest)
  reported         weld after REJOIN_START, the pair's lean over the 10 s before the ATTACH (mean, max), the ring's signed
                   lean toward plate 3 over the 0.5 s before the weld (x2_metrics.signed's roll, sign flipped: + = plate 3
                   down, as the registry's E5 rows read it)
The table rows are a run's case from its name: rj4_softdetach9 the square, rj3_pairrelease ..._tip the T, _lean the T
leaning toward the rejoining plate, _leanneg leaning away.
"""
import argparse
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import s_metrics as S  # noqa: E402
import x2_metrics as X2  # noqa: E402

LEAN_DEG, LAND_TOL_M = 2.0, 0.03
CASES = (('rj4_softdetach9', 'Square'), ('_leanneg', 'T, leaning away'), ('_lean', 'T, leaning toward'),
         ('rj3_pairrelease', 'T'))


def case_of(d):
    name = os.path.basename(d.rstrip('/'))
    for key, label in CASES:
        if key in name:
            return label
    return name.split('_sim_gz_')[-1]


def landed(run):
    """Every quad disarmed and within 3 cm of its rest height on the run's last row, after /fleet/landed."""
    n = len([c for c in run.df.columns if re.fullmatch(r'd\d+_x', c)])
    ok = []
    for i in range(n):
        z = run.df[f'd{i}_z'].to_numpy(float)
        ok.append(bool(abs(z[-1] - np.nanmedian(z[run.t < 2.0])) <= LAND_TOL_M and run.df[f'd{i}_armed'].iloc[-1] < 0.5))
    return bool(run.first('LANDED') is not None and all(ok))


def score(d):
    run = X2.Run(d)
    sm = S.score(run.dir)
    atts = [e for e in sm['events'] if e['event'] == 'ATTACH']
    bounds = [e['t'] for e in atts[1:]] + [run.t_land]
    out = {'run': run.rid, 'case': case_of(d), 'rejoins': [], 'dropped': sm['dropped'],
           'fleet_abort': [a for _, k, a in run.events if k == 'FLEET_ABORT'],
           'refused': len(re.findall(r'REJOIN REFUSED', run.launch)), 'aborted': len(re.findall(r'REJOIN ABORT', run.launch)),
           'landed': landed(run), 'late': max(sm['planner']['late'], sm['tracker_late_max']), 'failed': sm['planner']['failed']}
    for a, nxt in zip(atts, bounds):
        r = X2.rejoin(run, a['drone'], a['t'], nxt, a)
        pre = run.win(a['t'] - 10.0, a['t'])
        r['weld_after_start_s'] = r['t_weld'] - r['t_start']
        r['pair_lean_mean_deg'], r['pair_lean_max_deg'] = float(np.nanmean(run.mag[pre])), float(np.nanmax(run.mag[pre]))
        r['lean_to_plate3_before_weld_deg'] = -X2.signed(run, r['t_weld'] - 0.5, r['t_weld'])['roll_mean_deg']
        out['rejoins'].append(r)
    out['lean_deg'] = float(np.nanmean(run.mag[run.win(run.t_land - 10.0, run.t_land)]))
    out['pass'] = {'faults': not (out['dropped'] or out['fleet_abort'] or out['refused'] or out['aborted']),
                   'landed': out['landed'], 'lean': out['lean_deg'] <= LEAN_DEG,
                   'rejoins': all(r['pass_a'] and r['pass_b'] and r['pass_tilt'] and r['pass_rod'] and r['pass_share']
                                  for r in out['rejoins']) and bool(out['rejoins'])}
    out['holds'] = all(out['pass'].values())
    out['void'] = out['late'] > S.LATE_VOID or bool(out['failed'])
    return out


def table(rows):
    """tab:res_rejoin's form: one line per case, the range over its runs; a value over its limit in capitals (MISS)."""
    f = lambda v, nd: '-' if not v else (f'{min(v):.{nd}f}' if f'{min(v):.{nd}f}' == f'{max(v):.{nd}f}'  # noqa: E731
                                         else f'{min(v):.{nd}f}-{max(v):.{nd}f}')
    lines = ['| Rejoin | Runs | On plate (s) | Fold-in tilt (deg) | End off (cm) | Lean (deg) | Bars |',
             '|---|---|---|---|---|---|---|', '| Limit | - | 30 | 5 | 2 | 2 | |']
    for case in dict.fromkeys(r['case'] for r in rows):
        rs = [r for r in rows if r['case'] == case]
        rj = [x for r in rs for x in r['rejoins']]
        miss = [f"{r['run']} {', '.join(k for k, v in r['pass'].items() if not v)}" for r in rs if not r['holds']]
        lines.append(f"| {case} | {len(rs)} ({', '.join(r['run'] for r in rs)}) | {f([x['plate_s'] for x in rj], 1)} "
                     f"| {f([x['fold_peak_deg'] for x in rj], 2)} | {f([x['rod_end_cm'] for x in rj if x['rod_end_cm']], 1)} "
                     f"| {f([r['lean_deg'] for r in rs], 2)} | {'all hold' if not miss else 'MISS: ' + '; '.join(miss)} |")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories')
    ap.add_argument('--json', help='write every run\'s score here')
    a = ap.parse_args()
    rows = []
    for r in a.runs:
        d = S.locate(r)
        if not os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            print(f'{r}: no logs/run.csv, not scored (void?)')
            continue
        rows.append(score(d))
    f = lambda v, nd=2: '-' if v is None else f'{v:.{nd}f}'  # noqa: E731
    for o in rows:
        for x in o['rejoins']:
            print(f"{o['run']} {o['case']} /drone_{x['drone']}: (a) {f(x['plate_s'])} s, (b) {f(x['joint_s'], 3)} s, (c) "
                  f"{f(x['fold_peak_deg'])} deg (over 5 for {f(x['fold_over5_time_s'])} s), end {f(x['rod_end_cm'], 1)} cm, "
                  f"tension {f(x['share']['t_end_N'], 3)} vs {f(x['share']['share_N'], 3)} N, weld after START "
                  f"{f(x['weld_after_start_s'])} s; pair lean {f(x['pair_lean_mean_deg'])} (max {f(x['pair_lean_max_deg'])}),"
                  f" leaning toward plate 3 before the weld {f(x['lean_to_plate3_before_weld_deg'])} deg")
        print(f"   lean {f(o['lean_deg'])} deg; drop {o['dropped']}, aborts {o['fleet_abort'] or 'none'}, refused "
              f"{o['refused']}, rejoin aborts {o['aborted']}, landed {o['landed']}; late {100 * o['late']:.2f} %, failed "
              f"{o['failed']}: {'holds' if o['holds'] else 'MISS ' + str([k for k, v in o['pass'].items() if not v])}"
              + (' (VOID)' if o['void'] else ''))
    print(table(rows))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(X2.strip(rows), fh, indent=1)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
