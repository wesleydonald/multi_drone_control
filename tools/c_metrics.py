#!/usr/bin/env python3
"""tools/c_metrics.py -- thesis result C (card docs/experiments/2026-10-23_paper_method_compare.md): the paper's
method (our port of Quan et al.: the network on the trackers' velocity loop) against our planner, in the twin.

    python3 tools/c_metrics.py R2501 [R2502 ...]       # one LF registry row per run (metric, value, verdict)
    python3 tools/c_metrics.py --table R2501-R2527 [--out results/analysis/c_compare_<date>.json]
                                                       # arm x case: mean [min-max] (n), who beats whom; + markdown

Read-only on the runs. Windows (frozen in the card before any C run):
  hover  from the hand-over (the NET_HANDOVER event; our planner hands over to nothing, so the moment the network
         arm would: the reference's top + auto_handover_settle_s) for 60 s, never past LAND; the first 5 s apart
  line   from PATH_START (the planner's 'hover hold done' line; with no hold carried, the hand-over) over the whole
         cycles that fit before LAND
Metrics (mm unless stated). Both arms are scored against the target from just before the hand-over (the network
moves its datum onto the measured ring there, our planner does not), each arm's own published target beside it:
  hover  3-D error to the target, RMS over the window and over its first 5 s; height error, mean
  line   clock-free path error (metrics.segment_error without the along-line term: sideways + height; the reach counts
         an overshoot), RMS and max; reach at the line's ends (metrics.turnaround_reach), RMS and mean (+ = short);
         headline = clock-free RMS + reach RMS; lag (s, x against the continuous schedule through the published
         points); 3-D RMSE against that schedule; amplitude ratio
  both   ring tilt mean and peak (deg), the ring's largest yaw change (deg), quad tilt peak (deg), throttle peak; the
         hand-over's datum shift and the ring's height dip and tilt peak in its first 5 s; the path's start after the
         top (s); planner tick_ms p50 / p95 and the trackers' MPC solve_ms p50 / p95 over the window (the velocity loop
         logs no rows)
A run (card §5): a fleet abort, ring tilt over 30 deg on 2 samples (the drop rule, which only our planner's arm acts
on), a quad over 60 deg, or a drop, loss or envelope line in the log before the window ends is a LOSS: the method
failed, and the run ranks behind every run of the other arm that did not. VOID is for plumbing only: under 95 % of
the window's rows (with no loss), or a network arm with no hand-over; a VOID run is re-flown once.
"""
import csv
import datetime
import glob
import json
import math
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402
import run_logs  # noqa: E402

CARD = 'docs/experiments/2026-10-23_paper_method_compare.md'
HOVER_S = 60.0
FIRST_S = 5.0
DROP_TILT_DEG = 30.0       # the trackers' drop rule (2 samples), applied to both arms
QUAD_TILT_DEG = 60.0       # the trackers' envelope
COVERAGE = 0.95
HANDOVER_TOL_S = 0.15      # NET_HANDOVER against the reference's top + the settle time
LOG_FAULTS = ('DROP AND LAND', 'ENVELOPE FAULT', 'UNANNOUNCED DETACH', 'FLEET ABORT')
# a difference is stated only when every run of one arm beats every run of the other by at least this much (card §5,
# PROPOSED): the larger of MIN_EFFECT_MM and MIN_EFFECT_FRAC of the better arm's mean
MIN_EFFECT_MM, MIN_EFFECT_FRAC = 1.0, 0.10
ARMS = ('planner', 'port', 'paper')
ARM_TEXT = {'planner': 'our planner', 'port': "the network, our port of Quan et al. (damping against still nodes)",
            'paper': "the network, the paper's form (Eq. 9 damping, Eq. 10 acceleration)",
            'rel': 'the network, Eq. 9 damping only', 'lean': 'the network, Eq. 10 acceleration only'}


def run_dirs(args):
    out = []
    for a in args:
        m = re.fullmatch(r'R(\d{4})-R(\d{4})', a)
        if m:
            for i in range(int(m.group(1)), int(m.group(2)) + 1):
                out += glob.glob(os.path.join(REPO, 'results', '*', f'R{i:04d}_*'))
        elif os.path.isdir(a):
            out.append(a)
        else:
            hits = glob.glob(os.path.join(REPO, 'results', '*', f'{a}_*'))
            if not hits:
                raise SystemExit(f'no run {a}')
            out.append(hits[0])
    return [d for d in out if os.path.exists(os.path.join(d, 'logs', 'run.csv'))]


def planner_params(d):
    """The planner's params.json as flown ({} if none), the launch args under it."""
    out = dict((_json(os.path.join(d, 'manifest.json')).get('launch_args') or {}))
    for f in sorted(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'params.json'))):
        p = _json(f)
        out.update(p.get('params', p) if isinstance(p, dict) else {})
    return out


def _json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def _true(v):
    return str(v).strip().lower() in ('1', 'true', 'yes')


def arm_of(p):
    """planner / port / paper (rel, lean: one half of the paper's form) from the parameters that flew."""
    if not _true(p.get('auto_network_handover', False)) and str(p.get('mode', '')) != 'network':
        return 'planner'
    rel, lean = _true(p.get('net_damp_relative', False)), _true(p.get('net_traj_lean', False))
    return {(False, False): 'port', (True, True): 'paper', (True, False): 'rel', (False, True): 'lean'}[(rel, lean)]


def case_of(p):
    kind = str(p.get('load_traj', 'hover'))
    if kind == 'hover':
        return 'hover'
    if kind == 'line_x':
        return f"linex_v{1000 * float(p.get('traj_speed', 0.0)):.0f}"
    return None


def _first(ev, name):
    return next((t for t, n, _ in sorted(ev, key=lambda e: e[0]) if n == name and np.isfinite(t)), None)


def _q(x, qs=(50, 95)):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return [float(np.percentile(x, q)) for q in qs] if x.size else [math.nan] * len(qs)


def _log_faults(d):
    """The fault lines of launch.log, each marked before or after the planner heard LAND (the file is in time
    order; its stamps are wall time, not the run clock)."""
    path = os.path.join(d, 'logs', 'launch.log')
    if not os.path.exists(path):
        return []
    out, landed = [], False
    for line in open(path, errors='replace'):
        if "/fleet/command received: 'LAND'" in line:
            landed = True
        for f in LOG_FAULTS:
            if f in line:
                out.append((f, landed))
    return out


def _window(t, a, b):
    return (t >= a) & (t <= b)


def compute(d, run, a, b):
    """Planner tick and tracker solve times over [a, b] on the run clock: [tick p50, p95, solve p50, p95] (ms)."""
    import corner_metrics
    out = [math.nan] * 4
    L = corner_metrics.planner_log(d, run)
    if L is None:
        return out
    off = float(np.nanmedian(L.sim_time - L.run_t))
    if 'tick_ms' in L:
        out[0:2] = _q(L.tick_ms[(L.run_t >= a) & (L.run_t <= b)])
    solve = []
    for f in run_logs.trackers(d).values():
        try:
            T = pd.read_csv(f, usecols=['sim_time', 'solve_ms'])
        except (ValueError, OSError):
            continue
        rt = T.sim_time - off
        solve.append(T.solve_ms[(rt >= a) & (rt <= b)].to_numpy())
    if solve:
        out[2:4] = _q(np.concatenate(solve))
    return out


def score(d):
    """C's numbers for one run, or None for a run that is neither a hover nor a line_x."""
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    p = planner_params(d)
    arm, case = arm_of(p), case_of(p)
    if case is None:
        return None
    ev = M.read_event_list(d)
    t = run.t.to_numpy()
    ref = run[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy()
    act = run[['payload_x', 'payload_y', 'payload_z']].to_numpy()
    tilt = run.payload_tilt_deg.to_numpy()
    n = len([c for c in run.columns if re.fullmatch(r'd\d+_x', c)])
    t_land = _first(ev, 'LAND')
    t_land = float(t[-1]) if t_land is None else t_land
    pre = t < t_land
    top_z = np.nanmax(ref[pre, 2]) if np.any(pre) else np.nanmax(ref[:, 2])
    i_top = int(np.argmax(ref[:, 2] >= top_z - 1e-3))
    t_top = float(t[i_top])
    settle = float(p.get('auto_handover_settle_s', 1.5) or 1.5)
    t_ho_ev = _first(ev, 'NET_HANDOVER')
    t_ho = t_ho_ev if (arm != 'planner' and t_ho_ev is not None) else t_top + settle
    r = {'run': os.path.basename(d.rstrip('/')).split('_')[0], 'dir': d,
         'config': os.path.basename(d.rstrip('/')).split('_sim_gz_')[-1], 'arm': arm, 'case': case,
         't_top': t_top, 't_handover': t_ho, 'notes': []}
    if arm != 'planner' and t_ho_ev is not None and abs(t_ho_ev - (t_top + settle)) > HANDOVER_TOL_S:
        r['notes'].append(f'NET_HANDOVER {t_ho_ev - t_top:.2f} s after the top (expected {settle:.1f})')

    def target_before(t_at):
        """The published target just before t_at (the last samples of the second before), which both arms are scored
        against: the network moves its datum onto the ring at the hand-over, our planner does not (the critic, M1)."""
        sel = _window(t, t_at - 1.0, t_at - 1e-6)
        return np.nanmedian(ref[sel][-5:], axis=0) if np.any(sel) else ref[i_top]

    before = _window(t, t_ho - 1.0, t_ho - 1e-6)
    if case == 'hover':
        tgt0 = target_before(t_ho)
        w0, w1_plan = t_ho, t_ho + HOVER_S
        w1 = min(w1_plan, t_land)
        win = _window(t, w0, w1)
        e = np.linalg.norm(act - tgt0, axis=1)
        first = _window(t, w0, w0 + FIRST_S)
        r.update(err_rms_mm=1000 * _rms(e[win]), err_first5_rms_mm=1000 * _rms(e[first]),
                 z_err_mean_mm=1000 * float(np.nanmean((act[:, 2] - tgt0[2])[win])),
                 err_own_rms_mm=1000 * _rms(np.linalg.norm(act - ref, axis=1)[win]))
    else:
        v = float(p.get('traj_speed'))
        dist = float(p.get('traj_distance', 1.0))
        T = math.pi * dist / v
        t_p0 = _first(ev, 'PATH_START')
        if t_p0 is None:
            r['notes'].append('no PATH_START event: the path from the reference\'s first move')
            after = t > t_ho + 0.2 if arm != 'planner' else np.ones(t.size, bool)
            sw = M.sweep_window(t[after], ref[after, :2])
            t_p0 = float(t[after][sw][0]) - 0.05 if sw.any() else t_ho
        k = int(math.floor((t_land - t_p0) / T + 0.02))
        if k < 1:
            r['notes'].append('LAND before one whole cycle')
            k = 1
        w0, w1_plan = t_p0, t_p0 + k * T
        w1 = min(w1_plan, t_land)
        win = _window(t, w0, w1)
        # own: the line the arm published; off: how far its datum moved from the target before the hand-over (or
        # before the path, where a run's path started first)
        tgt0 = target_before(min(t_ho, t_p0))
        own0 = np.array([float(np.interp(t_p0, t, ref[:, c])) for c in range(3)])
        off = own0 - tgt0

        def path(x0, y, z):
            cf = M.segment_error(act, x0, x0 + dist, y, z, past_end=False)
            reach = M.turnaround_reach(t, act[:, 0], t_p0, T, x0, x0 + dist, t_end=w1)
            return cf, reach, np.array([x[2] for x in reach]) if reach else np.array([math.nan])
        cf, reach, rs = path(own0[0] - off[0], ref[:, 1] - off[1], ref[:, 2] - off[2])
        cf_own, _, rs_own = path(own0[0], ref[:, 1], ref[:, 2])
        ext = _window(t, w0 - 1.0, w1 + 1.0)
        tk, xk = M.schedule_knots(t, ref[:, :2], ext)
        if tk.size >= 2:
            xs = np.interp(t, tk, xk[:, 0])
            sched = np.c_[xs, ref[:, 1], ref[:, 2]]
            tg = np.arange(w0, w1, 0.02)
            lag = M.phase_lag_s(tg, np.interp(tg, t, xs), np.interp(tg, t, act[:, 0]))
            rmse_t = _rms(np.linalg.norm(act - (sched - off), axis=1)[win])
            amp = M.radius_ratio(act[:, :2], sched[:, :2], win)
        else:
            lag = rmse_t = amp = math.nan
        r.update(cycles=k, period_s=T, cf_rms_mm=1000 * _rms(cf[win]), cf_max_mm=1000 * float(np.nanmax(cf[win])),
                 reach_rms_mm=1000 * _rms(rs), reach_mean_mm=1000 * float(np.nanmean(rs)),
                 reach_absmax_mm=1000 * float(np.nanmax(np.abs(rs))), n_reversals=len(reach),
                 lag_s=float(lag), rmse_t_mm=1000 * float(rmse_t), amp_ratio=float(amp),
                 path_start_s=float(t_p0 - t_top))
        r['headline_mm'] = r['cf_rms_mm'] + r['reach_rms_mm']
        r['headline_own_mm'] = 1000 * (_rms(cf_own[win]) + _rms(rs_own))

    quad_tilt = np.nanmax(np.column_stack([run[f'd{i}_tilt_deg'] for i in range(n)]), axis=1) if n else np.zeros(t.size)
    thr = np.nanmax(np.column_stack([run[f'd{i}_thr'] for i in range(n)]), axis=1) if n else np.zeros(t.size)
    r.update(window=[w0, w1], tilt_mean_deg=float(np.nanmean(tilt[win])), tilt_peak_deg=float(np.nanmax(tilt[win])),
             quad_tilt_peak_deg=float(np.nanmax(quad_tilt[win])), thr_peak=float(np.nanmax(thr[win])))
    if all(f'payload_q{c}' in run for c in 'wxyz') and np.any(win):
        # the ring's yaw from where it was at the window's start: the network's formation turns with it, a mode
        # network_tune.py found growing at today's gains (card)
        w, x, y, z = (run[f'payload_q{c}'].to_numpy() for c in 'wxyz')
        yaw = np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))[win])
        r['yaw_excursion_deg'] = float(np.degrees(np.max(np.abs(yaw - yaw[0]))))
    # the hand-over: the datum shift the network takes over with, the ring's dip and tilt in the next 5 s
    after = _window(t, t_ho, t_ho + FIRST_S)
    if np.any(before) and np.any(after):
        r['datum_shift_mm'] = 1000 * float(np.linalg.norm(np.nanmedian(ref[after, :2][:5], axis=0)
                                                          - np.nanmedian(ref[before, :2][-5:], axis=0)))
        r['handover_dip_mm'] = 1000 * float(np.nanmean(act[before, 2]) - np.nanmin(act[after, 2]))
        r['handover_tilt_peak_deg'] = float(np.nanmax(tilt[after]))
    r['tick_p50_ms'], r['tick_p95_ms'], r['solve_p50_ms'], r['solve_p95_ms'] = compute(d, run, w0, w1)

    # a loss: the method failed before its window ended (the card's critic, M3); what happens after the window (the
    # network's LAND drops its quads from the cone) is reported, never scored
    lost = []
    aborts = [x for x, name, _ in ev if name == 'FLEET_ABORT' and x <= w1]
    if aborts:
        lost.append(f'fleet abort at {aborts[0]:.1f} s')
    k_rule = np.convolve((tilt > DROP_TILT_DEG).astype(int), [1, 1], 'valid') >= 2
    early = _window(t, t_top - 30.0, w1)[1:]
    if np.any(k_rule & early):
        lost.append(f'ring tilt over {DROP_TILT_DEG:.0f} deg at {float(t[1:][k_rule & early][0]):.1f} s')
    if np.any((quad_tilt > QUAD_TILT_DEG) & _window(t, t_top - 30.0, w1)):
        lost.append(f'a quad over {QUAD_TILT_DEG:.0f} deg')
    faults = _log_faults(d)
    for f in sorted({f for f, after_land in faults if not after_land}):
        # only our planner's arm runs B's loss detector: a trip is named, so a false one shows (critic m3)
        lost.append(f'{f} in the log before LAND' + (' (the loss detector)' if f == 'UNANNOUNCED DETACH' else ''))
    later = sorted({f for f, after_land in faults if after_land})
    if later:
        r['notes'].append('after LAND: ' + ', '.join(later))
    void = []
    if arm != 'planner' and t_ho_ev is None:
        void.append('no hand-over (no NET_HANDOVER event)')
    dt = float(np.median(np.diff(t))) if t.size > 1 else math.nan
    cover = float(np.count_nonzero(_window(t, w0, w1_plan)) * dt / max(w1_plan - w0, 1e-9))
    r['coverage'] = cover
    if cover < COVERAGE and not lost:
        void.append(f'{100 * cover:.0f} % of the window\'s rows')
    r['lost'] = lost
    r['valid'] = not void
    r['why'] = void
    return r


def _rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if x.size else math.nan


METRICS = {'hover': [('err_rms_mm', '3-D error from the hand-over, RMS', '{:.1f} mm'),
                     ('err_first5_rms_mm', 'its first 5 s, RMS', '{:.1f} mm'),
                     ('z_err_mean_mm', 'height error, mean', '{:+.1f} mm'),
                     ('err_own_rms_mm', '3-D error to its own target, RMS', '{:.1f} mm')],
           'line': [('cf_rms_mm', 'clock-free path error, RMS', '{:.1f} mm'),
                    ('reach_rms_mm', 'reach at the ends, RMS', '{:.1f} mm'),
                    ('headline_mm', 'headline (clock-free + reach)', '{:.1f} mm'),
                    ('headline_own_mm', 'headline against its own line', '{:.1f} mm'),
                    ('reach_mean_mm', 'reach, mean (+ short)', '{:+.1f} mm'),
                    ('lag_s', 'lag', '{:+.3f} s'),
                    ('rmse_t_mm', 'error against the clock, RMS', '{:.1f} mm'),
                    ('amp_ratio', 'amplitude ratio', '{:.3f}'),
                    ('path_start_s', 'path start after the top', '{:.2f} s')],
           'both': [('tilt_mean_deg', 'ring tilt, mean', '{:.2f} deg'),
                    ('tilt_peak_deg', 'ring tilt, peak', '{:.2f} deg'),
                    ('yaw_excursion_deg', 'ring yaw, largest change', '{:.1f} deg'),
                    ('quad_tilt_peak_deg', 'quad tilt, peak', '{:.1f} deg'),
                    ('thr_peak', 'throttle, peak', '{:.3f}'),
                    ('handover_dip_mm', 'ring dip in the 5 s after the hand-over', '{:.1f} mm'),
                    ('handover_tilt_peak_deg', 'ring tilt peak in those 5 s', '{:.2f} deg'),
                    ('tick_p50_ms', 'planner tick, median', '{:.1f} ms'),
                    ('solve_p50_ms', 'tracker solve, median', '{:.1f} ms')]}


def metric_list(case):
    return METRICS['hover' if case == 'hover' else 'line'] + METRICS['both']


def _fmt(f, v):
    return '' if v is None or (isinstance(v, float) and not np.isfinite(v)) else f.format(v)


def registry_row(r):
    """run,date,config,card,variable,baseline,metric,value,repeat_of,verdict,reviewer,notes as one LF CSV line."""
    import io
    ms = metric_list(r['case'])
    date = os.path.basename(os.path.dirname(os.path.abspath(r['dir'])))
    variable = f"{ARM_TEXT.get(r['arm'], r['arm'])}; {r['case']}"
    baseline = '' if r['arm'] == 'planner' else f"c_pl_{r['case']}_L100 (our planner)"
    value = ' / '.join(_fmt(f, r.get(k)) or '-' for k, _, f in ms)
    verdict = ('VOID (card rule: ' + '; '.join(r['why']) + ')' if not r['valid']
               else 'LOSS (card rule: ' + '; '.join(r['lost']) + ')' if r.get('lost') else 'valid (card rule)')
    notes = '; '.join(r['notes'] + ([f"{r['cycles']} cycles"] if 'cycles' in r else [])
                      + ([f"datum shift {r['datum_shift_mm']:.0f} mm"] if r['arm'] != 'planner'
                         and np.isfinite(r.get('datum_shift_mm', math.nan)) else []))
    buf = io.StringIO()
    csv.writer(buf, lineterminator='\n').writerow(
        [r['run'], date, r['config'], CARD, variable, baseline, ' / '.join(name for _, name, _ in ms), value, '',
         verdict, '', notes])
    return buf.getvalue()


def _cell(vals):
    v = [x for x in vals if x is not None and np.isfinite(x)]
    return dict(n=len(v), mean=float(np.mean(v)) if v else math.nan, min=float(np.min(v)) if v else math.nan,
                max=float(np.max(v)) if v else math.nan, values=v)


RANKED = ('headline_mm', 'cf_rms_mm', 'reach_rms_mm', 'err_rms_mm', 'rmse_t_mm', 'tilt_peak_deg')


def _beats(va, vb, k):
    """0 or 1 for the side whose every run beats every run of the other (lower is better; a lost run is inf), on a mm
    metric by at least the minimum effect; else None."""
    def by(lo, hi):
        if max(lo) >= min(hi):
            return False
        if not k.endswith('_mm') or not np.isfinite(min(hi)):
            return True
        return min(hi) - max(lo) >= max(MIN_EFFECT_MM, MIN_EFFECT_FRAC * float(np.mean(lo)))
    return 0 if by(va, vb) else 1 if by(vb, va) else None


def table(rows):
    """{case: {arm: {metric: cell, 'runs': {n, lost}}}} over the valid runs (a lost run's numbers are left out of its
    cell), and per case and metric which arm's runs all beat the other's (lower is better, a lost run behind every
    run that was not; the card states a difference only then)."""
    cells, beats = {}, {}
    for case in sorted({r['case'] for r in rows}):
        rr = [r for r in rows if r['case'] == case and r['valid']]
        arms = [a for a in ARMS + ('rel', 'lean') if any(r['arm'] == a for r in rr)]
        cells[case] = {}
        for a in arms:
            mine = [r for r in rr if r['arm'] == a]
            ok = [r for r in mine if not r.get('lost')]
            cells[case][a] = {k: _cell([r.get(k) for r in ok]) for k, _, _ in metric_list(case)}
            cells[case][a]['runs'] = dict(n=len(mine), lost=len(mine) - len(ok))

        def ranked(a, k):
            v = [math.inf if r.get('lost') else r.get(k) for r in rr if r['arm'] == a]
            return [x for x in v if x is not None and not np.isnan(x)]
        beats[case] = {}
        for k in RANKED:
            for i, a in enumerate(arms):
                for b in arms[i + 1:]:
                    va, vb = ranked(a, k), ranked(b, k)
                    if not va or not vb:
                        continue
                    beats[case][f'{k} {a} vs {b}'] = (a, b, None)[{0: 0, 1: 1, None: 2}[_beats(va, vb, k)]]
    return cells, beats


def claim(cells, beats):
    """The card's claim per case and network form, on the headline alone (line: clock-free + reach; hover: the 3-D
    error): 'confirms' when our planner's runs all beat the form's, 'falsifies' when the form's all beat ours, else
    'no difference shown'. In hover the paper's form takes the port's cell when it was not flown (one law in hover at
    identical frozen gains)."""
    out = {}
    for case, arms in cells.items():
        if 'planner' not in arms:
            continue
        key = 'err_rms_mm' if case == 'hover' else 'headline_mm'
        out[case] = {}
        for f in [a for a in arms if a != 'planner']:
            who = beats.get(case, {}).get(f'{key} planner vs {f}')
            out[case][f] = 'confirms' if who == 'planner' else 'falsifies' if who == f else 'no difference shown'
        if case == 'hover' and 'port' in out[case] and 'paper' not in out[case]:
            out[case]['paper'] = out[case]['port'] + " (the port's hover cell)"
    return out


def markdown(cells, beats):
    out = []
    for case, arms in cells.items():
        out.append(f'\n**{case}**\n')
        out.append('| metric | ' + ' | '.join(ARM_TEXT.get(a, a) for a in arms) + ' |')
        out.append('|---|' + '---|' * len(arms))
        out.append('| runs (lost) | ' + ' | '.join(f"{arms[a]['runs']['n']} ({arms[a]['runs']['lost']})"
                                                for a in arms) + ' |')
        for k, name, f in metric_list(case):
            row = []
            for a in arms:
                c = arms[a][k]
                row.append('' if not c['n'] else f"{_fmt(f, c['mean'])} [{_fmt(f, c['min'])} to {_fmt(f, c['max'])}] "
                           f"({c['n']})")
            out.append(f'| {name} | ' + ' | '.join(row) + ' |')
        sep = [f'{k}: {who}' for k, who in beats.get(case, {}).items() if who]
        out.append('\nall runs of one arm beat all of the other: ' + ('; '.join(sep) if sep else 'none'))
    for case, forms in claim(cells, beats).items():
        out.append(f'\nclaim, {case}: ' + '; '.join(f'{ARM_TEXT.get(f, f)}: {v}' for f, v in forms.items()))
    return '\n'.join(out)


LATEX_ARMS = (('planner', 'Our planner'), ('port', 'Our port of Quan et al.'), ('paper', "The paper's form"))
LATEX_CASES = (('hover', 'Hover'), ('linex_v200', 'Line, 0.2\\,m/s'), ('linex_v400', 'Line, 0.4\\,m/s'))
LATEX_ROWS = {'hover': (('err_rms_mm', 'Error from the hand-over (mm)', '{:.0f}'),
                        ('err_first5_rms_mm', '\\quad its first 5\\,s (mm)', '{:.0f}'),
                        ('tilt_peak_deg', 'Ring tilt, peak (\\si{\\degree})', '{:.1f}')),
              'line': (('headline_mm', 'Path error + reach (mm)', '{:.0f}'),
                       ('cf_rms_mm', '\\quad path error, clock-free (mm)', '{:.0f}'),
                       ('reach_rms_mm', '\\quad reach at the ends (mm)', '{:.0f}'),
                       ('lag_s', 'Lag (s)', '{:.2f}'),
                       ('rmse_t_mm', 'Error against the clock (mm)', '{:.0f}'),
                       ('tilt_peak_deg', 'Ring tilt, peak (\\si{\\degree})', '{:.1f}'))}


def latex(cells):
    """The thesis table (tools/thesis_sim_tables.py table_compare): arms as columns, each case's metrics as rows,
    mean [min--max] over the valid runs that did not fail, and each arm's runs with its failures, no run ids. The two
    network forms are one law in hover, so the port's hover cell stands for the paper's form when it was not flown."""
    arms = [(a, t) for a, t in LATEX_ARMS if any(a in cells.get(c, {}) for c, _ in LATEX_CASES)]

    def rng(c, fmt):
        if not c or not c.get('n'):
            return '--'
        s = lambda v: fmt.format(v).replace('-', '$-$')  # noqa: E731
        return s(c['mean']) if c['n'] == 1 else f"{s(c['mean'])} [{s(c['min'])}--{s(c['max'])}]"
    lines = []
    for case, title in LATEX_CASES:
        if case not in cells:
            continue
        lines.append(f'\\multicolumn{{{len(arms) + 1}}}{{l}}{{\\emph{{{title}}}}} \\\\')
        for k, name, fmt in LATEX_ROWS['hover' if case == 'hover' else 'line']:
            row = []
            for a, _ in arms:
                c = cells[case].get(a, {}).get(k)
                if c is None and case == 'hover' and a == 'paper':
                    c = cells[case].get('port', {}).get(k)
                row.append(rng(c, fmt))
            lines.append(f'{name} & ' + ' & '.join(row) + ' \\\\')
        runs = [cells[case].get(a) or (cells[case].get('port') if case == 'hover' and a == 'paper' else None)
                for a, _ in arms]
        lines.append('Runs (failed) & ' + ' & '.join('--' if not c else f"{c['runs']['n']} ({c['runs']['lost']})"
                                                     for c in runs) + ' \\\\')
    head = ' & '.join(t for _, t in arms)
    return ('\\begin{tabular}{l' + 'c' * len(arms) + '}\n\\toprule\n & ' + head + ' \\\\\n\\midrule\n'
            + '\n'.join(lines) + '\n\\bottomrule\n\\end{tabular}')


def main(argv):
    out_path = None
    if '--out' in argv:
        i = argv.index('--out')
        out_path = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    want_table = '--table' in argv
    rows = [r for r in (score(d) for d in run_dirs([a for a in argv if not a.startswith('--')])) if r]
    if not want_table:
        for r in rows:
            sys.stdout.write(registry_row(r))
        return 0
    cells, beats = table(rows)
    out_path = out_path or os.path.join(REPO, 'results', 'analysis',
                                        f'c_compare_{datetime.date.today().isoformat()}.json')
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    keep = [{k: v for k, v in r.items() if k != 'dir'} for r in rows]
    with open(out_path, 'w') as fh:
        json.dump(dict(card=CARD, runs=keep, cells=cells, beats=beats, claim=claim(cells, beats),
                       min_effect=dict(mm=MIN_EFFECT_MM, frac=MIN_EFFECT_FRAC)), fh, indent=1, default=float)
    print(markdown(cells, beats))
    print(f'\n-> {out_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
