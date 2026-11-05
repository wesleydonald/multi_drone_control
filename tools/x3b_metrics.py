#!/usr/bin/env python3
"""tools/x3b_metrics.py -- score X3b, eight and nine carriers at the real-time setting
(card docs/experiments/2026-10-30_x3b_realtime_8_9.md), against its bars as frozen 30 Oct ~15:46.

    python3 tools/x3b_metrics.py R3568 ... R3585 [--json results/analysis/x3b_<date>.json]

The cell comes from the run's name (x3b_s8_hover_rt, x3_s8_fig8_it5, x3b_s9_chain4back_rt_tip, ...; a trailing _tip is
dropped); 'it5' is the x3_*_it5 pair of the same cell, the comparator; x3_s4_hover_it5 sets N1's bars (2 x its mean
+ 3 mm, 2 x its mean + 1 deg, never above 25 mm / 6 deg), so give its runs too. Per run: tools/s_metrics.score()
untouched (events, stages, N1, N4, lift, late ticks, planner, drops), tools/n2_bands' clock-free path error (figure-8),
the rejoin stages J reads, the V items (read-back, the launch log's iteration count, sqp_calls on the published ticks,
N6), and the planner's tick_ms / solve_ms / res_ms. This is the 31 Oct scorer (results/analysis/x3b_2026-10-31/
score_x3b.py) with its run ids made arguments and its side analyses (the missed releases' traces, the four-carrier
timing, the scans) left out; one change: the filter check reads the mocaps' read-back (a config that types no lab_env
flies the emulator's default, slope), where the 31 Oct scorer read the manifest's lab_env.
C8 (8 carriers): the rt runs' verdict on every bar equals the it5 pair's, and P (close to it5: error <= max(1.25 x the
it5 mean, + 3 mm), tilt, each release's and fold-in's peak and the lift's tilt peak <= the it5 mean + 1 deg). N9 (9
carriers): N1, L, J in both repeats (N4 reported). R8 / R9 are the bench and the tuf runs: not scored here.
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
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import n2_bands  # noqa: E402
import run_logs  # noqa: E402
import s_metrics as S  # noqa: E402

L_RESIZE_S, L_PEAK_DEG, L_PEAK_ABS_DEG, SETTLED_DEG = 2.0, 8.83, 15.0, 3.0
J_PLATE_S, J_JOINT_S, J_BIGGER_S, J_FOLD_DEG, J_ROD_CM = 30.0, 0.5, 20.0, 5.0, 2.0
N1_ABS_MM, N1_ABS_DEG = 25.0, 6.0
LOW_RTF = 0.25            # S card section 9: under this a run is flagged and its RELEASE detections checked


def pct(v, q):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(np.percentile(v, q)) if v.size else None


def stats(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if not v.size:
        return {'n': 0, 'p50': None, 'p95': None, 'max': None}
    return {'n': int(v.size), 'p50': float(np.percentile(v, 50)), 'p95': float(np.percentile(v, 95)),
            'max': float(v.max())}


def events(d):
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    return [(float(r.sim_time), str(r.event), r.arg) for r in ev.itertuples()]


def planner_log(d, clock):
    pdirs = run_logs.node_dirs(d, 'dissipative_planner') + run_logs.node_dirs(d, 'mpc_planner')
    return S.node_log(os.path.join(pdirs[-1], 'log.csv'), *clock), pdirs[-1]


def rejoin_stages(d, ok_t, mag, sm):
    """J per ATTACH: REJOIN_START -> ON_PLATE, ON_PLATE -> JOINT_ATTACHED, ATTACH -> planner one bigger (s_metrics'
    resize), ring tilt magnitude peak REJOIN_WELD..+15 s, the BLEND OVER line's rod-end max, settled (s_metrics)."""
    ev = events(d)
    launch = os.path.join(d, 'logs', 'launch.log')
    rod = {}
    pat = re.compile(r'REJOIN BLEND OVER drone \d+ \(/drone_(\d+)\): its rod end at most ([0-9.]+) cm')
    for line in open(launch, errors='replace'):
        m = pat.search(line)
        if m:
            rod[int(m.group(1))] = float(m.group(2))
    out = []
    atts = [e for e in sm['events'] if e['event'] == 'ATTACH']
    for k, a in enumerate(atts):
        dr = a['drone']
        arg = lambda x: None if pd.isna(x) else str(x).split('.')[0]
        same = [(t, kind) for t, kind, ar in ev if arg(ar) == str(dr) and t >= a['t']
                and (k + 1 == len(atts) or t < atts[k + 1]['t'])]
        first = lambda name: next((t for t, kind in same if kind == name), None)
        start, plate = first('REJOIN_START'), first('REJOIN_ON_PLATE')
        joint, weld, blend = first('JOINT_ATTACHED'), first('REJOIN_WELD'), first('REJOIN_BLEND_OVER')
        m = (ok_t >= weld) & (ok_t <= weld + 15.0) if weld is not None else np.zeros_like(ok_t, bool)
        out.append({'drone': dr, 't_attach': a['t'],
                    'plate_s': None if None in (start, plate) else plate - start,
                    'joint_s': None if None in (plate, joint) else joint - plate,
                    'bigger_s': a['resize_s'], 'n_after': a['n_after'],
                    'fold_peak_deg': float(mag[m].max()) if m.any() else None,
                    'rod_end_cm': rod.get(dr), 'settled_deg': a['settled_deg'], 'blend_over': blend,
                    'peak_from_mean_deg': a['peak_deg']})
    return out


def validity(d, sm, plan, pdir, mf, expect_iters, expect_res):
    """The card's V: the numbers; 'ok' is every barred item (the runner's own PASS is not one of them)."""
    params = os.path.join(d, 'params', 'dissipative_planner.yaml')
    rb = {}
    if os.path.isfile(params):
        txt = open(params).read()
        for k in ('plan_sqp_iters', 'plan_res_every', 'plan_fast_refs'):
            g = re.search(rf'{k}: (\S+)', txt)
            rb[k] = g.group(1) if g else None
        rb['source'] = 'runner read-back'
    else:
        pj = json.load(open(os.path.join(pdir, 'params.json')))
        found = {}

        def walk(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    if k in ('plan_sqp_iters', 'plan_res_every', 'plan_fast_refs'):
                        found[k] = str(v).lower()
                    walk(v)
        walk(pj)
        rb.update(found)
        rb['source'] = "planner's own params.json (the runner's read-back missed the node)"
    launch = os.path.join(d, 'logs', 'launch.log')
    txt = open(launch, errors='replace').read()
    said = sorted({int(x) for x in re.findall(r'n=\d+: (\d+) SQP iterations per tick', txt)})
    faults = {k: len(re.findall(k, txt)) for k in ('DROP AND LAND', 'REJOIN ABORT', 'REJOIN REFUSED', 'envelope',
                                                  'ABORT')}
    # warm ticks: planner phase, flying or landing, the solver's own call count on the published ticks
    calls = pd.to_numeric(plan['sqp_calls'], errors='coerce')
    pub = plan['published'].astype(str) == 'solved'
    vals = calls[pub].value_counts().sort_index()
    sq = {str(int(k)): int(v) for k, v in vals.items()}
    filters = mocap_filters(d, mf)
    late_p, late_t = sm['planner']['late'], sm['tracker_late_max']
    p = sm['planner']
    chk = {
        'late_ok': bool(late_p <= S.LATE_VOID and late_t <= S.LATE_VOID),
        'no_failed': p['failed'] == 0,
        'readback_ok': (rb.get('plan_sqp_iters') == str(expect_iters) and rb.get('plan_res_every') == str(expect_res)
                        and rb.get('plan_fast_refs') == 'true' and filters == ['slope']),
        'launch_says': said == [expect_iters],
        'n6_ok': p['published_res_over'] == 0 and p['published_res_missing'] == 0,
    }
    non_warm = {k: v for k, v in sq.items() if k != str(expect_iters)}
    return {'late_planner': late_p, 'late_tracker_max': late_t, 'readback': rb, 'velocity_filters': filters,
            'launch_iters_said': said,
            'sqp_calls_on_published': sq, 'sqp_calls_other_than_expected': non_warm, 'faults_in_launch_log': faults,
            'published': p['published'], 'published_res_over': p['published_res_over'],
            'published_res_missing': p['published_res_missing'], 'published_res_eq_max': p['published_res_eq_max'],
            **chk}


def tick_tables(plan, sm, t_ref):
    """tick_ms, solve_ms, res_ms over the planner-phase rows; tick_ms apart at each fleet size; the steady windows."""
    ph = plan['phase'].to_numpy() == 'planner'
    land = pd.to_numeric(plan['land'], errors='coerce').to_numpy() == 0
    tick = pd.to_numeric(plan['tick_ms'], errors='coerce').to_numpy(float)
    solve = pd.to_numeric(plan['solve_ms'], errors='coerce').to_numpy(float)
    res = pd.to_numeric(plan['res_ms'], errors='coerce').to_numpy(float) if 'res_ms' in plan else np.full(len(plan),
                                                                                                         np.nan)
    calls = pd.to_numeric(plan['sqp_calls'], errors='coerce').to_numpy(float)
    n = plan['n'].to_numpy(int)
    t = plan['sim_time'].to_numpy(float)
    fly = ph & land
    out = {'all_planner': {'tick_ms': stats(tick[fly]), 'solve_ms': stats(solve[fly]), 'res_ms': stats(res[fly]),
                           'tick_over_100ms_frac': float(np.mean(tick[fly] > 100.0)) if fly.any() else None,
                           'tick_over_150ms_frac': float(np.mean(tick[fly] > 150.0)) if fly.any() else None},
           'per_n': {}}
    for k in sorted(set(n[fly])):
        m = fly & (n == k)
        out['per_n'][str(int(k))] = {'tick_ms': stats(tick[m]), 'solve_ms': stats(solve[m]), 'res_ms': stats(res[m])}
    for name in ('n1', 'n4'):
        s = sm.get(name)
        if s is None:
            continue
        m = ph & (t >= s['from']) & (t < s['to'])
        out[name + '_window'] = {'from': s['from'], 'to': s['to'], 'tick_ms': stats(tick[m]),
                                 'solve_ms': stats(solve[m]), 'res_ms': stats(res[m])}
    # cold ticks (reseeds: more SQP calls than the setting) apart
    if calls[fly].size:
        base = np.nanmedian(calls[fly])
        cold = fly & (calls > base)
        out['cold_ticks'] = {'sqp_calls_median_warm': float(base), 'count': int(cold.sum()),
                             'tick_ms': stats(tick[cold]), 'calls': sorted({int(c) for c in calls[cold]})}
    return out


def mocap_filters(d, mf):
    """The carriers' velocity filters: the mocaps' read-back (velocity_filter), else the manifest's lab_env, else the
    emulator's default (slope)."""
    got = set()
    for f in glob.glob(os.path.join(d, 'params', 'mocap_*.yaml')):
        m = re.search(r'^\s+velocity_filter: (\S+)', open(f).read(), re.M)
        if m:
            got.add(m.group(1).strip("'\""))
    return sorted(got) or [(mf.get('lab_env') or {}).get('SIM_MOCAP_VEL_FILTER', 'slope')]


def cell_of(d):
    """The cell from the run's name: x3b_s8_hover_rt, x3_s8_chain4back_it5 (a trailing _tip, the marker check,
    dropped: every chain cell has flown it since 3 Nov)."""
    name = os.path.basename(d.rstrip('/')).split('_sim_gz_')[-1]
    return name[:-4] if name.endswith('_tip') else name


def score_run(cell, d):
    rid = os.path.basename(d.rstrip('/')).split('_')[0]
    mf = json.load(open(os.path.join(d, 'manifest.json')))
    sm = S.score(d)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'))
    clock = (float(mf['t_ready_sim']), float(run['t'].min()), float(run['t'].max()))
    plan, pdir = planner_log(d, clock)
    ok = run.dropna(subset=['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz'])
    th = S.RB.tilt_vector(ok[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float))
    t = ok['t'].to_numpy(float)
    mag = np.linalg.norm(th, axis=1)
    iters = 5 if cell.endswith('it5') else 1
    res_every = 0 if cell.endswith('rt0') else 1
    out = {'cell': cell, 'run': rid, 'dir': os.path.relpath(d, REPO),
           'manifest': {k: mf.get(k) for k in ('mean_rtf', 'sim_duration_s', 'wall_s', 'exit_reason', 'passed',
                                               'failures', 'git_sha', 'dirty_files', 'diff_sha256', 't_ready_sim')},
           'slot': (mf.get('lab') or {}).get('slot'),
           'readback_missing_required': (mf.get('param_readback') or {}).get('missing_required'),
           'checks': mf.get('checks'),
           'n1': sm['n1'], 'n4': sm['n4'], 'n4_resize': sm['n4_resize'], 'lift': sm['lift'],
           'planner': sm['planner'], 'tracker_late_max': sm['tracker_late_max'], 'dropped': sm['dropped'],
           'void_by_s_metrics': sm['void'], 'rejoin_lines': sm['rejoins'], 'stages': sm['stages'],
           'releases': [e for e in sm['events'] if e['event'] == 'RELEASE']}
    out['v'] = validity(d, sm, plan, pdir, mf, iters, res_every)
    out['ticks'] = tick_tables(plan, sm, t)
    out['rejoins'] = rejoin_stages(d, t, mag, sm)
    evs = events(d)
    out['release_detected'] = []
    for e in out['releases']:
        resize = [te for te, k, a in evs if k == 'DETACH_RESIZE' and not pd.isna(a) and int(float(a)) == e['drone']
                  and te >= e['t']]
        out['release_detected'].append({'drone': e['drone'], 'resize_event_s': resize[0] - e['t'] if resize else None})
    out['low_rtf_flag'] = bool((mf.get('mean_rtf') or 0) < LOW_RTF)
    if 'fig8' in cell:
        err, tilt = n2_bands.path_metrics_clock_free(d)
        out['path'] = {'clock_free_rmse_m': err, 'tilt_peak_deg': tilt}
    out['events_key'] = {k: next((te for te, kk, _ in evs if kk == k), None)
                         for k in ('LIFTED', 'PATH_START', 'LAND', 'LANDED')}
    return out


def j(run, cell):
    """(all rejoins' J items) -> list of per-rejoin dicts with a 'holds' flag and any missed item."""
    res = []
    for r in run['rejoins']:
        miss = []
        if r['plate_s'] is None or r['plate_s'] > J_PLATE_S:
            miss.append('plate')
        if r['joint_s'] is None or r['joint_s'] > J_JOINT_S:
            miss.append('joint')
        if r['bigger_s'] is None or r['bigger_s'] > J_BIGGER_S:
            miss.append('bigger')
        if r['fold_peak_deg'] is None or r['fold_peak_deg'] > J_FOLD_DEG:
            miss.append('fold')
        if r['rod_end_cm'] is None or r['rod_end_cm'] > J_ROD_CM:
            miss.append('rod')
        if r['settled_deg'] is None or r['settled_deg'] > SETTLED_DEG:
            miss.append('settled')
        res.append(dict(r, miss=miss))
    return res


def l(run):
    res = []
    for e in run['releases']:
        miss = []
        if e['resize_s'] is None or e['resize_s'] > L_RESIZE_S:
            miss.append('resize')
        if e['peak_deg'] is None or e['peak_deg'] > L_PEAK_DEG or e['peak_deg'] > L_PEAK_ABS_DEG:
            miss.append('peak')
        if e['settled_deg'] is None or e['settled_deg'] > SETTLED_DEG:
            miss.append('settled')
        res.append({'drone': e['drone'], 'resize_s': e['resize_s'], 'n_after': e['n_after'],
                    'peak_deg': e['peak_deg'], 'settled_deg': e['settled_deg'], 'miss': miss})
    return res



def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [strip(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def score_all(dirs):
    """{cells, bars, P, verdicts, runs} for the run directories given (the 31 Oct scorer's main, its ids made inputs)."""
    CELLS = {}
    for d in dirs:
        CELLS.setdefault(cell_of(d), []).append(os.path.basename(d.rstrip('/')).split('_')[0])
    runs = {}
    for d in dirs:
        rid = os.path.basename(d.rstrip('/')).split('_')[0]
        print('scoring', cell_of(d), rid, flush=True)
        runs[rid] = score_run(cell_of(d), d)
    bars = None
    if 'x3_s4_hover_it5' in CELLS:
        s4 = [runs[r] for r in CELLS['x3_s4_hover_it5']]
        s4_rmse = float(np.mean([1e3 * r['n1']['pos_rmse_m'] for r in s4]))
        s4_tilt = float(np.mean([r['n1']['tilt_peak_deg'] for r in s4]))
        bars = {'s4_it5_n1_rmse_mm': [1e3 * r['n1']['pos_rmse_m'] for r in s4],
                's4_it5_n1_tilt_deg': [r['n1']['tilt_peak_deg'] for r in s4],
                's4_it5_mean_rmse_mm': s4_rmse, 's4_it5_mean_tilt_deg': s4_tilt,
                'n1_rmse_bar_mm': min(2 * s4_rmse + 3.0, N1_ABS_MM),
                'n1_tilt_bar_deg': min(2 * s4_tilt + 1.0, N1_ABS_DEG), 'n1_abs_mm': N1_ABS_MM, 'n1_abs_deg': N1_ABS_DEG}

    def n_hold(s):
        if bars is None or s is None or s['pos_rmse_m'] is None:
            return None, None
        return bool(1e3 * s['pos_rmse_m'] <= bars['n1_rmse_bar_mm']), bool(s['tilt_peak_deg'] <= bars['n1_tilt_bar_deg'])

    verdicts = {}
    for rid, r in runs.items():
        v = {}
        v['n1_pos_holds'], v['n1_tilt_holds'] = n_hold(r['n1'])
        if r['n4'] is not None:
            v['n4_pos_holds'], v['n4_tilt_holds'] = n_hold(r['n4'])
            v['n4_at_full_n'] = r['n4']['n']
        v['L'] = l(r)
        v['J'] = j(r, r['cell'])
        v['L_holds'] = all(not e['miss'] for e in v['L']) and not r['dropped'] if v['L'] else None
        v['J_holds'] = all(not e['miss'] for e in v['J']) if v['J'] else None
        verdicts[rid] = v
    for rid, r in runs.items():
        vv = r['v']
        reasons = []
        if not vv['late_ok']:
            reasons.append('late ticks over 1 %')
        if not vv['no_failed']:
            reasons.append('failed planner solve')
        if not vv['readback_ok'] or not vv['launch_says']:
            reasons.append('read-back or launch log off the setting')
        missed = [e['drone'] for e, dd in zip(r['releases'], r['release_detected']) if dd['resize_event_s'] is None]
        r['releases_missed'] = missed
        if missed and r['low_rtf_flag']:
            reasons.append(f"release of /drone_{missed[0]} missed under RTF {r['manifest']['mean_rtf']:.3f} (< 0.25)")
        r['void_reasons'] = reasons
        r['valid'] = not reasons
    valid = {rid for rid, r in runs.items() if r['valid']}

    def pairmean(cell, f, only_valid=True):
        v = [f(runs[x]) for x in CELLS[cell] if (x in valid or not only_valid)]
        v = [x for x in v if x is not None]
        return float(np.mean(v)) if v else None

    def p_row(cell_rt, cell_it5, metric, f, form):
        bar_of = lambda b: None if b is None else (max(1.25 * b, b + 3.0) if form == 'err_mm' else b + 1.0)  # noqa
        base, base_all = pairmean(cell_it5, f), pairmean(cell_it5, f, only_valid=False)
        bar, bar_all = bar_of(base), bar_of(base_all)
        rows = {x: f(runs[x]) for x in CELLS[cell_rt]}
        return {'metric': metric, 'it5': {x: f(runs[x]) for x in CELLS[cell_it5]},
                'it5_valid_runs': [x for x in CELLS[cell_it5] if x in valid], 'it5_mean': base, 'bar': bar,
                'bar_if_void_runs_counted': bar_all, 'rt': rows,
                'holds': {x: (None if v is None or bar is None else bool(v <= bar)) for x, v in rows.items()},
                'holds_if_void_runs_counted': {x: (None if v is None or bar_all is None else bool(v <= bar_all))
                                               for x, v in rows.items()}}

    P = {}
    have = lambda *c: all(x in CELLS for x in c)  # noqa: E731
    if have('x3b_s8_hover_rt', 'x3_s8_hover_it5'):
        P['hover'] = [
            p_row('x3b_s8_hover_rt', 'x3_s8_hover_it5', 'N1 RMSE (mm)', lambda r: 1e3 * r['n1']['pos_rmse_m'], 'err_mm'),
            p_row('x3b_s8_hover_rt', 'x3_s8_hover_it5', 'N1 tilt peak (deg)', lambda r: r['n1']['tilt_peak_deg'], 'deg'),
            p_row('x3b_s8_hover_rt', 'x3_s8_hover_it5', 'lift tilt peak (deg)', lambda r: r['lift']['tilt_peak_deg'],
                  'deg')]
    if have('x3b_s8_fig8_rt', 'x3_s8_fig8_it5'):
        P['fig8'] = [
            p_row('x3b_s8_fig8_rt', 'x3_s8_fig8_it5', 'clock-free path RMSE (mm)',
                  lambda r: 1e3 * r['path']['clock_free_rmse_m'], 'err_mm'),
            p_row('x3b_s8_fig8_rt', 'x3_s8_fig8_it5', 'path tilt peak (deg)', lambda r: r['path']['tilt_peak_deg'],
                  'deg'),
            p_row('x3b_s8_fig8_rt', 'x3_s8_fig8_it5', 'lift tilt peak (deg)', lambda r: r['lift']['tilt_peak_deg'],
                  'deg')]
    if have('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5'):
        first = runs[CELLS['x3_s8_chain4back_it5'][0]]
        chain = [
            p_row('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5', 'N4 RMSE (mm)',
                  lambda r: 1e3 * r['n4']['pos_rmse_m'] if r['n4'] else None, 'err_mm'),
            p_row('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5', 'N4 tilt peak (deg)',
                  lambda r: r['n4']['tilt_peak_deg'] if r['n4'] else None, 'deg'),
            p_row('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5', 'lift tilt peak (deg)',
                  lambda r: r['lift']['tilt_peak_deg'], 'deg')]
        for k, e in enumerate(first['releases']):
            chain.append(p_row('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5', f"release /drone_{e['drone']} peak (deg)",
                               lambda r, k=k: r['releases'][k]['peak_deg'] if k < len(r['releases']) else None, 'deg'))
        for k, e in enumerate(first['rejoins']):
            chain.append(p_row('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5',
                               f"rejoin /drone_{e['drone']} fold-in peak (deg)",
                               lambda r, k=k: r['rejoins'][k]['fold_peak_deg'] if k < len(r['rejoins']) else None, 'deg'))
        P['chain'] = chain
    return {'card': 'docs/experiments/2026-10-30_x3b_realtime_8_9.md',
            'scored': 'tools/x3b_metrics.py; bars as frozen 30 Oct ~15:46',
            'cells': CELLS, 'bars': bars, 'P': P, 'verdicts': verdicts, 'runs': runs}


def c8(doc):
    """C8's two parts per 8-carrier cell: the rt runs' verdict on each S bar equals the it5 pair's (where the pair splits,
    the bar itself), and every P item holds; N9 for the nine-carrier cells (N1, L, J in both valid repeats)."""
    runs, ver, cells = doc['runs'], doc['verdicts'], doc['cells']
    out = {}
    for rt, it5, keys in (('x3b_s8_hover_rt', 'x3_s8_hover_it5', ('n1_pos_holds', 'n1_tilt_holds')),
                          ('x3b_s8_fig8_rt', 'x3_s8_fig8_it5', ()),
                          ('x3b_s8_chain4back_rt', 'x3_s8_chain4back_it5',
                           ('L_holds', 'J_holds', 'n4_pos_holds', 'n4_tilt_holds'))):
        if rt not in cells or it5 not in cells:
            continue
        same = {}
        for k in keys:
            ref = {ver[x].get(k) for x in cells[it5] if runs[x]['valid']}
            got = {x: ver[x].get(k) for x in cells[rt] if runs[x]['valid']}
            same[k] = {'it5': sorted(ref, key=str), 'rt': got,
                       'equal': (all(v == next(iter(ref)) for v in got.values()) if len(ref) == 1
                                 else all(v for v in got.values()))}
        p = doc['P'].get({'x3b_s8_hover_rt': 'hover', 'x3b_s8_fig8_rt': 'fig8'}.get(rt, 'chain'), [])
        p_miss = [(row['metric'], x) for row in p for x, h in row['holds'].items() if runs[x]['valid'] and h is False]
        bad = [x for x in cells[rt] if runs[x]['valid'] and (runs[x]['dropped'] or runs[x]['v']['faults_in_launch_log'].get('ABORT')
                                                             or not runs[x]['v']['no_failed'])]
        n_valid = sum(runs[x]['valid'] for x in cells[rt])
        ok = all(v['equal'] for v in same.values()) and not p_miss and not bad
        out[rt] = {'same_verdicts': same, 'P_missed': p_miss, 'drop_abort_failed': bad, 'valid_runs': n_valid,
                   'verdict': ('CONFIRMED' if ok and n_valid >= 2 else 'holds, fewer than 2 valid repeats' if ok
                               else 'FALSIFIED')}
    for cell in ('x3b_s9_hover_rt', 'x3b_s9_chain4back_rt'):
        if cell not in cells:
            continue
        items = ('n1_pos_holds', 'n1_tilt_holds') if 'hover' in cell else ('L_holds', 'J_holds')
        val = [x for x in cells[cell] if runs[x]['valid']]
        miss = {x: [k for k in items if ver[x].get(k) is False] for x in val}
        bad = [x for x in val if runs[x]['dropped'] or not runs[x]['v']['no_failed']]
        ok = not any(miss.values()) and not bad
        out[cell] = {'items': items, 'missed': miss, 'drop_failed': bad, 'valid_runs': len(val),
                     'n4_reported': {x: (ver[x].get('n4_pos_holds'), ver[x].get('n4_tilt_holds')) for x in val},
                     'verdict': 'CONFIRMED' if ok and len(val) >= 2 else 'holds, fewer than 2 valid repeats' if ok
                     else 'FALSIFIED'}
    return out


def report(doc, c):
    f = lambda v, nd=1: '-' if v is None else f'{v:.{nd}f}'  # noqa: E731
    runs, ver = doc['runs'], doc['verdicts']
    if doc['bars']:
        b = doc['bars']
        print(f"N1 bars from x3_s4_hover_it5 ({', '.join(doc['cells']['x3_s4_hover_it5'])}): <= {b['n1_rmse_bar_mm']:.2f} mm, "
              f"<= {b['n1_tilt_bar_deg']:.2f} deg")
    else:
        print('N1 bars: not set (give the x3_s4_hover_it5 runs)')
    print('| run | cell | valid | solve p95 (ms) | N1 / N4 RMSE (mm), tilt peak (deg) | releases peak (deg) | fold-ins: J\'s '
          'peak weld..+15 s; the ATTACH\'s rise from the 5 s mean (deg) '
          '| path (mm), tilt (deg) | L, J, N1 (hover), N4 |')
    print('|' + '---|' * 9)
    for rid, r in runs.items():
        v = ver[rid]
        nn = r['n4'] or r['n1']
        rel = ', '.join(f(e['peak_deg'], 2) for e in r['releases'])
        fold = ', '.join(f(e['fold_peak_deg'], 2) for e in r['rejoins'])
        if r['rejoins']:
            fold += '; ' + ', '.join(f(e['peak_from_mean_deg'], 2) for e in r['rejoins'])
        path = (f"{f(1e3 * r['path']['clock_free_rmse_m'])}, {f(r['path']['tilt_peak_deg'], 2)}" if r.get('path') else '-')
        hold = lambda k: '-' if v.get(k) is None else ('holds' if v[k] else 'MISS')  # noqa: E731
        print(f"| {rid} | {r['cell']} | {'yes' if r['valid'] else 'VOID: ' + '; '.join(r['void_reasons'])} "
              f"| {f(r['planner']['solve_ms_p95'])} | {'N4' if r['n4'] else 'N1'} "
              f"{f(1e3 * nn['pos_rmse_m'] if nn['pos_rmse_m'] else None)}, {f(nn['tilt_peak_deg'], 2)} | {rel or '-'} "
              f"| {fold or '-'} | {path} | L {hold('L_holds')}, J {hold('J_holds')}, "
              + (f"N1 {hold('n1_pos_holds')}/{hold('n1_tilt_holds')}, " if 'hover' in r['cell'] else '')
              + f"N4 {hold('n4_pos_holds')}/{hold('n4_tilt_holds')} |")
    for key, rows in doc['P'].items():
        for row in rows:
            print(f"P {key} {row['metric']}: it5 mean {f(row['it5_mean'], 2)}, bar {f(row['bar'], 2)}; rt "
                  + ', '.join(f"{x} {f(v, 2)} {'holds' if h else 'MISS' if h is False else '-'}"
                              for (x, v), h in zip(row['rt'].items(), row['holds'].values())))
    for cell, x in c.items():
        print(f"{'C8' if 's8' in cell else 'N9'} {cell}: {x['verdict']}"
              + (f"; verdicts equal {', '.join(k + ' ' + str(v['equal']) for k, v in x['same_verdicts'].items())}"
                 if 'same_verdicts' in x else f"; missed {x['missed']}")
              + (f"; P missed {x['P_missed']}" if x.get('P_missed') else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories (every cell with its it5 pair, and x3_s4_hover_it5)')
    ap.add_argument('--json', help='write the whole score here')
    a = ap.parse_args()
    dirs = []
    for r in a.runs:
        d = S.locate(r)
        if os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            dirs.append(d)
        else:
            print(f'{r}: no logs/run.csv, not scored (void?)')
    doc = score_all(dirs)
    doc['C8_N9'] = c8(doc)
    report(doc, doc['C8_N9'])
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(strip(doc), fh, indent=1)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
