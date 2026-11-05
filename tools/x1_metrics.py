#!/usr/bin/env python3
"""tools/x1_metrics.py -- score X1 release-ablation runs (card docs/experiments/2026-10-30_x1_release_ablation.md).

    python3 tools/x1_metrics.py R2915 R2916 [--json results/analysis/x1_2026-10-30.json] [--table] [--bars]

--bars scores the card's frozen bars per cell (B1, B2, B4; the sqT cells U1, U2, U5; B3 / U3 / U4 where a planned-pull
arm flew; with --json also <json>_bars.json); cell and arm come from the run's name (x1_<cell>_<arm>), so give every
arm of a cell together.

Per run, t_e = the RELEASE event (runner clock), the window ends at LAND:
  tilt rise    release_bands' delta-theta: the tilt vector's largest departure from its mean over the 5 s before t_e
  abs peak     the largest tilt magnitude, t_e to LAND; beside it the tilt's RMS and p95 over the same window
  dip          the ring's mean height over the 5 s before t_e minus its lowest height in t_e .. t_e + 10 s (mm)
  drop         DROP AND LAND in the launch log, or the ring under 0.3 m before LAND
  t_level      the first time after the peak from which the tilt vector stays within max(2 sigma_pre, 1 deg) of its
               pre-release mean for 1 s (censored at LAND: None); beside it the settled tilt (mean magnitude, last 5 s
               before LAND), release_bands' T6 time back in band (first entry held 1 s), and T6's last exit: when the tilt
               last left that band, which a later swing moves and the first entry does not (the band can be as small as
               0.5 deg, so a wobble moves it too); tilt after 6 s, the largest tilt magnitude from t_e + 6 s to LAND, shows
               a swing's size
  off point    in a hover, the ring's mean horizontal distance from its reference over the last 5 s before LAND (mm)
  path error   RMS of the ring's 3-D distance to its reference path over t_e .. t_e + 10 s: in a hover to the reference
               point, on a path clock-free (across the path and height, as tools/n2_bands.py's b22 set)
  validity     detection (the planner's 'detected at' line for a delayed arm, else its resize tick), the resize (the
               planner's first tick at n-1, s_metrics), late ticks, failed solves before t_e, read-back arm values
"""
import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402
import release_bands as RB  # noqa: E402
import s_metrics as S  # noqa: E402

AFTER_S = 10.0          # dip and path-error window after the release
AFTER_PEAK_S = 6.0      # s after the release: the first swing is over in every flown cell
LEVEL_BAND_MIN = 1.0    # deg: t_level's smallest band
HOLD_S = 1.0            # s: t_level holds this long
DROP_Z = 0.3            # m: a ring under this before LAND was dropped


def _clock_free_err(df, t, w):
    """The ring's 3-D distance to the path its reference traces, at the rows w (None on a hover)."""
    des = df[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    sweep = M.sweep_window(t, des)
    if not sweep.any():
        return None
    tk, xk = M.schedule_knots(t, des, sweep)
    path = M.path_through(xk)
    s_sched = np.interp(t, tk, path[5])
    across, _, _ = M.project_on_path(path, df[['payload_x', 'payload_y']].to_numpy(float), np.where(w, s_sched, np.nan))
    dz = df.payload_z.to_numpy(float) - df.payload_ref_z.to_numpy(float)
    return np.sqrt(across ** 2 + dz ** 2)[w]


def _readback(run_dir):
    out = {}
    pf = os.path.join(run_dir, 'params', 'dissipative_planner.yaml')
    if os.path.isfile(pf):
        txt = open(pf).read()
        m = re.search(r'resize_delay_s:\s*([-\d.eE+]+)', txt)
        out['resize_delay_s'] = float(m.group(1)) if m else None
    srcs = set()
    for tf in glob.glob(os.path.join(run_dir, 'params', 'tracker_*.yaml')):
        m = re.search(r'cable_source:\s*(\S+)', open(tf).read())
        if m:
            srcs.add(m.group(1).strip("'\""))
    out['cable_source'] = ','.join(sorted(srcs)) or None
    return out


def score(run_dir):
    logs = os.path.join(run_dir, 'logs')
    df = pd.read_csv(os.path.join(logs, 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    with open(os.path.join(run_dir, 'manifest.json')) as fh:
        man = json.load(fh)
    name = os.path.basename(run_dir.rstrip('/'))
    m = re.search(r'x1_([A-Za-z0-9_]+)_([a-z0-9]+)$', name)
    cell, arm = (m.group(1), m.group(2)) if m else (None, None)
    try:
        rb = RB.Run(run_dir)
    except SystemExit:
        # no LAND: the ring was dropped and the fleet landed on the drop; the window ends at LANDED (or the last row)
        ev0 = pd.read_csv(os.path.join(logs, 'events.csv'))
        landed = ev0[ev0.event.astype(str) == 'LANDED'].sim_time
        rb = RB.Run(run_dir, tland=float(landed.iloc[0]) if len(landed) else float(df.t.max()))
    te, tland = rb.te, rb.tland
    out = dict(run=name.split('_')[0], cell=cell, arm=arm, t_release=te, t_land=tland)
    met = rb.metrics()
    out.update(tilt_rise=met['dtheta'], t_peak=met['t_peak'], abs_peak=met['abs_peak'], sigma_pre=met['sigma_pre'],
               settled_tilt=met['post'], t_back_t6=met['t_back'])
    after = np.linalg.norm(rb.th[rb.win(te, tland + 1e-9)], axis=1)
    out['tilt_rms'] = float(np.sqrt(np.mean(after ** 2)))
    out['tilt_p95'] = float(np.percentile(after, 95))
    late = np.linalg.norm(rb.th[rb.win(te + AFTER_PEAK_S, tland + 1e-9)], axis=1)
    out['tilt_after_6s'] = float(late.max()) if late.size else None
    out['t6_last_exit'] = None
    if met['post'] is not None:
        post = rb.th[rb.win(tland - RB.POST_S, tland)].mean(0)
        outside = np.nonzero((rb.t >= te) & (rb.t < tland) & (np.linalg.norm(rb.th - post, axis=1) > met['band_deg']))[0]
        out['t6_last_exit'] = float(rb.t[outside[-1]] - te) if outside.size else 0.0
    mean, sig, _ = rb.pre()
    band = max(2.0 * sig, LEVEL_BAND_MIN)
    inb = np.linalg.norm(rb.th - mean, axis=1) <= band
    out['t_level'] = None
    for i in np.nonzero((rb.t >= te + met['t_peak']) & (rb.t + HOLD_S <= tland))[0]:
        if inb[rb.win(rb.t[i], rb.t[i] + HOLD_S)].all():
            out['t_level'] = float(rb.t[i] - te)
            break
    out['level_band_deg'] = band
    t = df.t.to_numpy(float)
    z = df.payload_z.to_numpy(float)
    pre = (t >= te - RB.PRE_S) & (t < te)
    aft = (t >= te) & (t <= min(te + AFTER_S, tland))
    out['dip_mm'] = 1000.0 * float(np.nanmean(z[pre]) - np.nanmin(z[aft]))
    err = _clock_free_err(df, t, aft)
    if err is None:
        e = (df[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
             - df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))[aft]
        err = np.linalg.norm(e, axis=1)
        out['path_kind'] = 'point'
    else:
        out['path_kind'] = 'clock-free'
    err = err[np.isfinite(err)]
    out['path_err_mm'] = 1000.0 * float(np.sqrt(np.mean(err ** 2))) if err.size else None
    out['off_point_mm'] = None
    if out['path_kind'] == 'point':
        last = (t >= tland - RB.POST_S) & (t < tland)
        off = (df[['payload_x', 'payload_y']].to_numpy(float) - df[['payload_ref_x', 'payload_ref_y']].to_numpy(float))[last]
        out['off_point_mm'] = 1000.0 * float(np.linalg.norm(np.nanmean(off, axis=0)))
    s = S.score(run_dir)
    rel = next((e for e in s['events'] if e['event'] == 'RELEASE'), None)
    out['resize_s'] = rel['resize_s'] if rel else None
    out['n_after'] = rel['n_after'] if rel else None
    launch = os.path.join(logs, 'launch.log')
    txt = open(launch, errors='replace').read() if os.path.isfile(launch) else ''
    low = (t < tland) & (t > te) & (z < DROP_Z)
    out['dropped'] = bool(s['dropped'] or low.any())
    out['drop_reason'] = (re.findall(r'DROP AND LAND[^\n]*', txt) or [None])[0]
    ev = pd.read_csv(os.path.join(logs, 'events.csv'))
    ab = ev[ev.event.astype(str) == 'FLEET_ABORT']
    out['aborted'] = (f"{float(ab.sim_time.iloc[0]) - te:+.2f} s: {ab.arg.iloc[0]}" if len(ab) else None)
    out['loss'] = bool(out['dropped'] or out['aborted'])
    # sqT's tilt drop is at 80 deg: when the ring first passed it (a dropped ring's later tilt is the fall, not the flight)
    over = np.nonzero((rb.t >= te) & (np.linalg.norm(rb.th, axis=1) > 80.0))[0]
    out['t_over80_s'] = float(rb.t[over[0]] - te) if over.size else None
    det = re.search(r'stays in the model, the loss is acted on at t=([\d.]+) s \(detected at ([\d.]+) s\)', txt)
    t_ready = float(man.get('t_ready_sim', 0.0))
    out['detect_s'] = (float(det.group(2)) - t_ready - te) if det else (out['resize_s'] if arm != 'nors' else None)
    out['unannounced_lines'] = len(re.findall(r'UNANNOUNCED DETACH', txt))
    out['late_resize_lines'] = len(re.findall(r'LATE RESIZE', txt))
    pl = s.get('planner', {})
    out['late_planner'] = pl.get('late')
    out['late_tracker'] = s.get('tracker_late_max')
    out['failed_solves'] = pl.get('failed')
    out['void_late'] = s['void']
    out.update(_readback(run_dir))
    out['rtf'] = man.get('mean_rtf')
    return out


TABLE = (('tilt_rise', 'tilt rise', 2), ('abs_peak', 'abs peak', 2), ('tilt_rms', 'tilt RMS', 2), ('tilt_p95', 'tilt p95', 2),
         ('dip_mm', 'dip mm', 1), ('settled_tilt', 'settled', 2), ('t_back_t6', 'T6', 1), ('t6_last_exit', 'T6 last exit', 1),
         ('tilt_after_6s', 'tilt after 6 s', 2),
         ('off_point_mm', 'off point mm', 0), ('path_err_mm', 'path mm', 1))


def table(rows):
    """One markdown table per cell, arms as rows, in the supervisors' ablation format: mean +- SD for n >= 3, both values
    for n = 2, and each metric's mean against the cell's no-resize arm in percent where the cell has one."""
    lines = []
    for cell in sorted({r['cell'] for r in rows}, key=str):
        here = [r for r in rows if r['cell'] == cell]
        vals = lambda arm, key: [r[key] for r in here if r['arm'] == arm and r[key] is not None]  # noqa: E731
        lines += [f'\n{cell}', '| arm | n | ' + ' | '.join(h for _, h, _ in TABLE) + ' |', '|' + '---|' * (len(TABLE) + 2)]
        for arm in dict.fromkeys(r['arm'] for r in here):
            cells = []
            for key, _, nd in TABLE:
                txt = M.spread_text(vals(arm, key), nd)
                base = vals('nors', key)
                if arm != 'nors' and base and vals(arm, key) and np.mean(base) != 0:
                    txt += f' ({100.0 * (np.mean(vals(arm, key)) / np.mean(base) - 1.0):+.0f} %)'
                cells.append(txt)
            lines.append(f"| {arm} | {sum(r['arm'] == arm for r in here)} | " + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


LATE_ORDER = ('ours', 'd05', 'd10', 'd20', 'nors')
HOVER_CELLS = ('h4', 'h8', 'h4p115')     # B4's settle bar reads only these
FLEETS = ('h4', 'h8')                    # B1's falsifier reads the two hover fleets


def _mean(rows, key):
    v = [r[key] for r in rows if r.get(key) is not None]
    return float(np.mean(v)) if v else None


def _worse(arm_rows, ref_rows, settled):
    """Card section 5 / U1: each arm repeat worse than the reference's max by a margin, or a loss the reference has none of
    (tilt rise +1 deg, dip +5 mm, with `settled` the settled tilt +1 deg; a drop for B1's rule, a drop or abort for U1's).
    -> (confirmed: every repeat worse, falsified: the means inside every band and no loss, the per-item numbers)."""
    lossy = 'loss' if settled else 'dropped'
    items = [('tilt_rise', 1.0), ('dip_mm', 5.0)] + ([('settled_tilt', 1.0)] if settled else [])
    ref_max = {k: max((r[k] for r in ref_rows if r.get(k) is not None), default=None) for k, _ in items}
    ref_loss = any(r[lossy] for r in ref_rows)
    each = [any(r.get(k) is not None and ref_max[k] is not None and r[k] > ref_max[k] + m for k, m in items)
            or (r[lossy] and not ref_loss) for r in arm_rows]
    diffs = {k: (None if _mean(arm_rows, k) is None or _mean(ref_rows, k) is None
                 else _mean(arm_rows, k) - _mean(ref_rows, k)) for k, _ in items}
    inside = all(d is not None and abs(d) <= m for (k, m), d in zip(items, diffs.values()))
    return {'confirmed': bool(arm_rows and ref_rows and all(each)),
            'falsified': bool(inside and not ref_loss and not any(r[lossy] for r in arm_rows)),
            'ref_max': ref_max, 'mean_diff': diffs, 'each_worse': each}


def _verdict(w):
    return 'CONFIRMED' if w['confirmed'] else 'FALSIFIED' if w['falsified'] else 'INCONCLUSIVE'


def _order(here, uneven):
    """B2 / U2: the late arms' tilt-rise means in order ours <= 0.5 <= 1 <= 2 s <= none, the largest inversion of two
    neighbours against its allowance (0.5 deg; U2: max(0.5, 5 % of ours' mean)), and the 2 s arm against ours (B2: its
    tilt rise over ours_max + 1 deg in both repeats, falsified within 1 deg of ours' mean; U2: U1's rule)."""
    means = [(a, _mean([r for r in here if r['arm'] == a], 'tilt_rise')) for a in LATE_ORDER]
    means = [(a, m) for a, m in means if m is not None]
    ours = [r for r in here if r['arm'] == 'ours']
    d20 = [r for r in here if r['arm'] == 'd20']
    if len(means) < len(LATE_ORDER) or not ours or not d20:
        return None
    inv = max(0.0, max(a[1] - b[1] for a, b in zip(means[:-1], means[1:])))
    allow = max(0.5, 0.05 * means[0][1]) if uneven else 0.5
    if uneven:
        w = _worse(d20, ours, settled=True)
    else:
        top = max(r['tilt_rise'] for r in ours)
        w = {'confirmed': all(r['tilt_rise'] > top + 1.0 for r in d20),
             'falsified': abs(_mean(d20, 'tilt_rise') - _mean(ours, 'tilt_rise')) <= 1.0,
             'mean_diff': {'tilt_rise': _mean(d20, 'tilt_rise') - _mean(ours, 'tilt_rise')}}
    ok = inv <= allow
    return {'means': dict(means), 'largest_inversion': inv, 'allowed': allow, 'd20_vs_ours': w,
            'verdict': 'CONFIRMED' if ok and w['confirmed'] else 'FALSIFIED' if not ok or w['falsified']
            else 'INCONCLUSIVE'}


ARM_DELAY = {'ours': 0.0, 'plan': 0.0, 'd05': 0.5, 'd10': 1.0, 'd20': 2.0, 'nors': 999.0, 'nplan': 999.0}


def validity(r):
    """Section 5's validity items that the run's own logs carry; [] is valid (a drop after the release is an outcome)."""
    why = []
    if not r['unannounced_lines'] or r['detect_s'] is None and r['arm'] not in ('nors', 'nplan'):
        why.append('no UNANNOUNCED DETACH')
    elif r['detect_s'] is not None and r['detect_s'] > 0.5:
        why.append('detection later than 0.5 s')
    delay = ARM_DELAY.get(r['arm'])
    if r['arm'] in ('nors', 'nplan'):
        if r['n_after'] is not None:
            why.append('a no-resize arm resized')
    elif r['resize_s'] is None:
        why.append('never resized')
    elif delay == 0.0 and r['resize_s'] > 0.5:
        why.append('resize later than 0.5 s')
    elif delay and (not r['late_resize_lines'] or r['detect_s'] is None or abs(r['resize_s'] - r['detect_s'] - delay) > 0.2):
        why.append('late resize not at detection + delay (0.2 s)')
    if delay is not None and r['resize_delay_s'] is not None and abs(r['resize_delay_s'] - delay) > 1e-6:
        why.append(f"read-back resize_delay_s {r['resize_delay_s']}")
    if r['void_late']:
        why.append('late ticks over 1 %')
    if r['failed_solves']:
        why.append('failed planner solve')
    return why


def bars(rows):
    """The card's frozen bars per cell (section 5: B1-B4; the sqT addendum: U1-U5; a planned-pull arm only where flown)
    over the valid runs. A cell whose name starts with sqT reads the U bars. B1's claim across fleets: FALSIFIED if h4 and
    h8 both falsify."""
    out = {'void': {r['run']: validity(r) for r in rows if validity(r)}}
    rows = [r for r in rows if r['run'] not in out['void']]
    cells = {}
    out['cells'] = cells
    for cell in sorted({r['cell'] for r in rows if r['cell']}, key=str):
        here = [r for r in rows if r['cell'] == cell]
        arm = lambda a: [r for r in here if r['arm'] == a]  # noqa: E731
        uneven = cell.startswith('sqT')
        b = {'cell': cell, 'runs': {a: [r['run'] for r in arm(a)] for a in dict.fromkeys(r['arm'] for r in here)}}
        one = 'U1' if uneven else 'B1'
        if arm('ours') and arm('nors'):
            b[one] = _worse(arm('nors'), arm('ours'), settled=uneven)
            b[one]['verdict'] = _verdict(b[one])
        if cell != 'h4p115':
            b['U2' if uneven else 'B2'] = _order(here, uneven)
        if arm('ours') and arm('plan'):
            b['U3' if uneven else 'B3'] = _worse(arm('plan'), arm('ours'), settled=uneven)
        if uneven and arm('plan') and arm('nplan'):
            b['U4'] = _worse(arm('nplan'), arm('plan'), settled=True)
        for k in ('B3', 'U3', 'U4'):
            if k in b:
                b[k]['verdict'] = _verdict(b[k])
        safe = []
        for r in arm('ours'):
            if uneven:
                miss = (['loss'] if r['loss'] else []) + (['abs peak > 80'] if r['abs_peak'] > 80.0 else [])
            else:
                miss = (['drop'] if r['dropped'] else []) + (['abs peak >= 30'] if r['abs_peak'] >= 30.0 else [])
            frozen = list(miss)
            if uneven or cell in HOVER_CELLS:
                if r['t_back_t6'] is None or r['t_back_t6'] > 5.0:
                    miss.append('T6 > 5.0 s')
                if not uneven and (r['t_level'] is None or r['t_level'] > 5.0):
                    frozen.append('t_level > 5.0 s')
            safe.append({'run': r['run'], 'miss': miss, 'miss_frozen_t_level': None if uneven else frozen})
        if safe:
            key = 'U5' if uneven else 'B4'
            b[key] = {'runs': safe, 'verdict': 'MISS' if any(s['miss'] for s in safe) else 'HOLDS'}
            if not uneven:
                b[key]['verdict_frozen_t_level'] = 'MISS' if any(s['miss_frozen_t_level'] for s in safe) else 'HOLDS'
        cells[cell] = b
    fl = [cells[c]['B1']['verdict'] for c in FLEETS if c in cells and 'B1' in cells[c]]
    out['B1_claim'] = ('FALSIFIED' if len(fl) == 2 and all(v == 'FALSIFIED' for v in fl) else
                       'CONFIRMED' if fl and all(v == 'CONFIRMED' for v in fl) else
                       'CONFIRMED for ' + ', '.join(c for c in FLEETS if c in cells
                                                    and cells[c].get('B1', {}).get('verdict') == 'CONFIRMED')
                       if 'CONFIRMED' in fl else 'INCONCLUSIVE' if fl else None)
    return out


def bars_text(b):
    f = lambda v, nd=2: '-' if v is None else f'{v:+.{nd}f}'  # noqa: E731
    lines = []
    for cell, c in b['cells'].items():
        role = 'B1 secondary, reported' if cell in ('f4', 'h4p115') else 'a claim cell'
        lines.append(f"\n{cell} ({role}): " + ', '.join(f"{a} {' '.join(rs)}" for a, rs in c['runs'].items()))
        for k in ('B1', 'U1', 'B3', 'U3', 'U4'):
            if k in c:
                d = c[k]['mean_diff']
                lines.append(f"  {k} {c[k]['verdict']}: every repeat worse {c[k]['each_worse']}; mean diff rise "
                             f"{f(d['tilt_rise'])} deg, dip {f(d['dip_mm'], 1)} mm"
                             + (f", settled {f(d['settled_tilt'])} deg" if 'settled_tilt' in d else ''))
        for k in ('B2', 'U2'):
            if c.get(k):
                o = c[k]
                rep = '' if cell in ('h4', 'f4') or cell.startswith('sqT') else ' (reported)'
                lines.append(f"  {k}{rep} {o['verdict']}: rise means " + ' <= '.join(f'{a} {m:.2f}' for a, m in o['means'].items())
                             + f"; largest inversion {o['largest_inversion']:.2f} (allowed {o['allowed']:.2f}); 2 s vs ours "
                             + _verdict(o['d20_vs_ours']))
            elif k in c:
                lines.append(f'  {k}: not scorable (an arm is missing)')
        for k in ('B4', 'U5'):
            if k in c:
                o = c[k]
                lines.append(f"  {k} {o['verdict']}" + (f" (frozen t_level rule: {o['verdict_frozen_t_level']})"
                                                         if 'verdict_frozen_t_level' in o else '') + ': '
                             + '; '.join(f"{s['run']} {', '.join(s['miss']) or 'ok'}" for s in o['runs']))
    lines.append(f"\nB1 across the fleets (h4, h8): {b['B1_claim']}")
    for run, why in b['void'].items():
        lines.append(f"VOID {run} (left out of the bars; re-fly once): {'; '.join(why)}")
    return '\n'.join(lines)


def locate(tag):
    if os.path.isdir(tag):
        return tag
    hits = [h for h in glob.glob(os.path.join(RB.RESULTS, '2*', tag + '_*')) if os.path.isdir(h)]
    if len(hits) != 1:
        sys.exit(f'{tag}: {len(hits)} run directories')
    return hits[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    ap.add_argument('--table', action='store_true', help='also print the per-cell tables')
    ap.add_argument('--bars', action='store_true', help="also score the card's frozen bars per cell")
    a = ap.parse_args()
    rows = []
    for r in a.runs:
        d = locate(r)
        if not os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            print(f'{r}: no logs/run.csv, not scored (void?)')
            continue
        rows.append(score(d))
    for r in rows:
        f = lambda v, nd=2: '-' if v is None else (f'{v:.{nd}f}' if isinstance(v, float) else str(v))
        print(f"{r['run']} {r['cell']}/{r['arm']}: rise {f(r['tilt_rise'])} abs {f(r['abs_peak'])} rms {f(r['tilt_rms'])} "
              f"p95 {f(r['tilt_p95'])} dip {f(r['dip_mm'], 1)} drop {r['dropped']} t_level {f(r['t_level'])} settled "
              f"{f(r['settled_tilt'])} T6 {f(r['t_back_t6'], 1)} last exit {f(r['t6_last_exit'], 1)} after 6 s {f(r['tilt_after_6s'])} off point "
              f"{f(r['off_point_mm'], 0)} path {f(r['path_err_mm'], 1)} "
              f"({r['path_kind']}) | detect {f(r['detect_s'])} resize {f(r['resize_s'])} n {r['n_after']} late "
              f"{f(r['late_planner'], 4)}/{f(r['late_tracker'], 4)} failed {r['failed_solves']} delay "
              f"{r['resize_delay_s']} src {r['cable_source']} rtf {f(r['rtf'])}")
    if a.table:
        print(table(rows))
    b = bars(rows) if a.bars else None
    if b:
        print(bars_text(b))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(rows, fh, indent=1)
        if b:
            with open(os.path.splitext(a.json)[0] + '_bars.json', 'w') as fh:
                json.dump(b, fh, indent=1)


if __name__ == '__main__':
    main()
