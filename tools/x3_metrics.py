#!/usr/bin/env python3
"""tools/x3_metrics.py -- score X3, the eight-carrier set (card docs/experiments/2026-10-23_scalability.md: N1/N4 as
frozen 23 Oct, the harder cases' draft bars of "More complex cases" / "Shared bars", judged after the run).

    python3 tools/x3_metrics.py R3648 ... R3673 R3582 R3583 R3584 R3585 [--json results/analysis/x3_<date>.json]

The case comes from the run's name, old or new (s8_chain4b2b_R50_L100_it5_tip and x3_s8_chain4b2b_it5_tip are both
'chain4b2b'). Comparators are read from the runs given, so give them with the cases:
  s4_hover (x3_s4_hover_it5)   N1 / N4's bars: 2 x its mean + 3 mm, 2 x its mean + 1 deg, never above 25 mm / 6 deg
  hover, fig8, orbit, fig8laps the bases (no event): P against them, max(1.25 x the base's mean, + 3 mm), tilt + 1 deg
  chain4back                    the claim chain (x3_s8_chain4back_it5_tip): the overlap bar of the b2b cases (each
                                fold-in peak <= its mean fold-in peak + 1 deg)
Per run tools/s_metrics.score() (events, stages, N1, N4, lift, late ticks, planner), tools/x3b_metrics' rejoin stages
and its L and J items (the shared bars: L the planner at n-1 within 2.0 s, the peak from the 5 s mean <= 8.83 and 15 deg,
settled <= 3 deg; J on plate 30 s, joint 0.5 s, one bigger 20 s, fold-in peak weld..+15 s <= 5 deg, rod end 2 cm,
settled 3 deg), and each case's own items (the registry rows of R3648-R3673 name them). A pair of RELEASE rows within
PAIR_S of each other is one event (adjpair, pairs): its peak from the 5 s mean before the first, the planner at n-1
and n-2 from the first.
"""
import argparse
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
import run_logs  # noqa: E402
import s_metrics as S  # noqa: E402
import x3b_metrics as XB  # noqa: E402

PAIR_S = 1.0                         # s: RELEASE rows closer than this are one event
N1_ABS_MM, N1_ABS_DEG = 25.0, 6.0
L_PEAK_DEG, L_ABS_DEG, L_RESIZE_S, SETTLED_DEG = 8.83, 15.0, 2.0, 3.0
ORBIT_BAND_MM, ORBIT_BAND_DEG = 33.0, 4.06   # the 4-quad figure-8's b22 band (orbit's falsifier)
E4_BOX_M, E4_CARRIER_M = 1.6, 0.8
TASK_REF_M, TASK_BODY_M, TASK_CLEAR_M = 0.02, 0.05, 0.5
RELBLEND_ABS_DEG = 10.0
CASES = ('hover', 's4_hover', 'fig8', 'fig8laps', 'orbit', 'adjpair', 'chain4b2b', 'chain4b2b_orbit', 'chain4back_fig8laps',
         'pairs', 'relblend', 'task', 'fig8rel2', 'relay', 'chain2back', 'chain4back')
N4_CASES = ('chain4back', 'chain4b2b', 'pairs', 'relay', 'task', 'chain2back')   # the cases whose bars name N4
DROP_TOKENS = ('R50', 'L100', 'it5', 'tip', 'noxy', 'h5', 'v200', 'r30')


def case_of(d):
    name = os.path.basename(d.rstrip('/')).split('_sim_gz_')[-1]
    parts = [p for p in name.split('_') if p not in DROP_TOKENS and p != 'x3']
    if parts and parts[0] == 's8':
        parts = parts[1:]
    case = '_'.join(parts)
    return {'chain4back_pairs': 'pairs', 'chain4back_relblend': 'relblend', 'chain4task': 'task',
            's4_hover': 's4_hover'}.get(case, case)


class Run:
    """One run: run.csv, events.csv, the planner's log on run.csv's clock, the launch log, s_metrics' score."""

    def __init__(self, d):
        self.dir = d
        self.rid = os.path.basename(d.rstrip('/')).split('_')[0]
        self.case = case_of(d)
        self.man = json.load(open(os.path.join(d, 'manifest.json')))
        self.df = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        self.t = self.df.t.to_numpy(float)
        q = self.df[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
        ok = np.all(np.isfinite(q), axis=1)
        self.th = np.full((len(self.t), 2), np.nan)
        self.th[ok] = RB.tilt_vector(q[ok])
        self.mag = np.linalg.norm(self.th, axis=1)
        self.pos = self.df[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
        self.ref = self.df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
        ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
        self.events = [(float(r.sim_time), str(r.event), None if pd.isna(r.arg) else str(r.arg)) for r in ev.itertuples()]
        self.launch = open(os.path.join(d, 'logs', 'launch.log'), errors='replace').read()
        self.n = len([c for c in self.df.columns if re.fullmatch(r'd\d+_x', c)])
        self.sm = S.score(d)
        clock = (float(self.man['t_ready_sim']), float(self.t.min()), float(self.t.max()))
        pdirs = run_logs.node_dirs(d, 'dissipative_planner') + run_logs.node_dirs(d, 'mpc_planner')
        self.plan = S.node_log(os.path.join(pdirs[-1], 'log.csv'), *clock)
        self.t_land = self.first('LAND') or float(self.t[-1])
        # a path's end ('load trajectory complete - descending': the planner's clock at its largest) can come before
        # LAND; it is the descent when the ring's reference then sinks 2 cm before LAND (n2_bands' window end)
        self.t_descent = self.t_land
        pt, tt = self.plan['sim_time'].to_numpy(float), pd.to_numeric(self.plan['traj_t'], errors='coerce').to_numpy(float)
        if np.nanmax(tt, initial=0.0) > 0.0:
            t_end = float(pt[int(np.nanargmax(tt))])
            after = (self.t >= t_end) & (self.t < self.t_land)
            if t_end < self.t_land and np.any(self.ref[after, 2] < np.nanmax(self.ref[:, 2]) - 0.02):
                self.t_descent = t_end

    def first(self, kind, drone=None, after=None):
        for te, k, a in self.events:
            if k == kind and (drone is None or (a is not None and a.split('.')[0].split()[0] == str(drone))) \
                    and (after is None or te >= after):
                return te
        return None

    def all(self, kind):
        return [(te, a) for te, k, a in self.events if k == kind]

    def win(self, a, b):
        return (self.t >= a) & (self.t < b)

    def xyz(self, i):
        return self.df[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy(float)


def clock_free(run, a, b):
    """(RMS mm, tilt peak deg) of the ring's clock-free error over [a, b] (n2_bands' b22 method, the window given)."""
    des = run.df[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    sweep = M.sweep_window(run.t, des)
    w = (run.t >= a) & (run.t <= b)
    if not sweep.any() or not w.any():
        return None, None
    tk, xk = M.schedule_knots(run.t, des, sweep)
    path = M.path_through(xk)
    s_sched = np.interp(run.t, tk, path[5])
    across, _, _ = M.project_on_path(path, run.pos[:, :2], np.where(w, s_sched, np.nan))
    e = np.sqrt(across ** 2 + (run.pos[:, 2] - run.ref[:, 2]) ** 2)[w]
    e = e[np.isfinite(e)]
    return 1e3 * float(np.sqrt(np.mean(e ** 2))), float(np.nanmax(run.df.payload_tilt_deg.to_numpy(float)[w]))


def airborne_box(run, drones=None, lo=None, hi=None):
    """(largest |x|, largest |y|) of the quads while airborne (z over 5 cm above their first height) in [lo, hi)
    (default LIFTED to LAND: the ring up)."""
    m = run.win((run.first('LIFTED') or run.t[0]) if lo is None else lo, run.t_land if hi is None else hi)
    bx = by = 0.0
    for i in (range(run.n) if drones is None else drones):
        p = run.xyz(i)
        up = m & (p[:, 2] > np.nanmedian(p[:20, 2]) + 0.05)
        if up.any():
            bx, by = max(bx, float(np.nanmax(np.abs(p[up, 0])))), max(by, float(np.nanmax(np.abs(p[up, 1]))))
    return bx, by


def releases(run):
    """The RELEASE events, a pair within PAIR_S merged: [{t, drones, peak_deg, abs_deg, settled_deg, n_steps}]: the peak
    from the 5 s mean before it to the next chain event, settled over the 5 s before the next event (or LAND), and the
    planner's first tick at each smaller size after it."""
    chain = [(te, k) for te, k, _ in run.events if k in S.CHAIN_EVENTS]
    rel = [(te, int(float(a))) for te, a in run.all('RELEASE')]
    groups = []
    for te, d in rel:
        if groups and te - groups[-1]['t'] < PAIR_S:
            groups[-1]['drones'].append(d)
            groups[-1]['t_last'] = te
        else:
            groups.append({'t': te, 'drones': [d], 't_last': te})
    t_end = min(run.t_land, run.t_descent)
    pt, pn = run.plan['sim_time'].to_numpy(float), run.plan['n'].to_numpy(int)
    for g in groups:
        nxt = min([te for te, _ in chain if te > g['t_last']] + [t_end])
        pre, w = run.win(g['t'] - S.PRE_S, g['t']), run.win(g['t'], nxt)
        dev = np.linalg.norm(run.th[w] - np.nanmean(run.th[pre], axis=0), axis=1)
        g['peak_deg'], g['abs_deg'] = float(np.nanmax(dev)), float(np.nanmax(run.mag[w]))
        tail = w & (run.t >= nxt - S.SETTLE_S)
        g['settled_deg'] = float(np.nanmean(run.mag[tail]))
        before = pn[pt < g['t']]
        n0 = int(before[-1]) if before.size else None
        g['n_steps'] = []
        for k in range(1, len(g['drones']) + 1):
            hit = np.nonzero((pt >= g['t']) & (pt < nxt) & (pn <= n0 - k))[0] if n0 is not None else []
            g['n_steps'].append(float(pt[hit[0]] - g['t']) if len(hit) else None)
        g['next'] = nxt
    return groups


def l_items(rels, dropped):
    out = []
    for g in rels:
        miss = []
        if any(s is None or s > L_RESIZE_S for s in g['n_steps']):
            miss.append('resize')
        if g['peak_deg'] > L_PEAK_DEG or g['peak_deg'] > L_ABS_DEG:
            miss.append('peak')
        if g['settled_deg'] > SETTLED_DEG:
            miss.append('settled')
        out.append(dict(g, miss=miss))
    return {'items': out, 'holds': (all(not g['miss'] for g in out) and not dropped) if out else None}


def j_items(run):
    """J per rejoin (x3b_metrics.rejoin_stages and its J bar), with the rejoin's own times."""
    ok = ~np.isnan(run.mag)
    rj = XB.rejoin_stages(run.dir, run.t[ok], run.mag[ok], run.sm)
    js = XB.j({'rejoins': rj}, run.case)
    for r in js:
        r['t_weld'] = run.first('REJOIN_WELD', r['drone'], r['t_attach'])
        r['t_start'] = run.first('REJOIN_START', r['drone'], r['t_attach'])
    return {'items': js, 'holds': all(not r['miss'] for r in js) if js else None}


def n_bars(s4):
    if not s4:
        return None
    rmse = float(np.mean([1e3 * r.sm['n1']['pos_rmse_m'] for r in s4]))
    tilt = float(np.mean([r.sm['n1']['tilt_peak_deg'] for r in s4]))
    return {'from': [r.rid for r in s4], 'rmse_mm': min(2 * rmse + 3.0, N1_ABS_MM), 'tilt_deg': min(2 * tilt + 1.0, N1_ABS_DEG)}


def n_check(s, nb):
    if s is None or s.get('pos_rmse_m') is None:
        return None
    out = {'rmse_mm': 1e3 * s['pos_rmse_m'], 'tilt_deg': s['tilt_peak_deg'], 'n': s.get('n')}
    if nb:
        out['pos_holds'], out['tilt_holds'] = out['rmse_mm'] <= nb['rmse_mm'], out['tilt_deg'] <= nb['tilt_deg']
    return out


def p_form(base_vals):
    if not base_vals:
        return None
    e, t = float(np.mean([v[0] for v in base_vals])), float(np.mean([v[1] for v in base_vals]))
    return {'base_mm': e, 'bar_mm': max(1.25 * e, e + 3.0), 'base_tilt_deg': t, 'bar_tilt_deg': t + 1.0}


def landing(run):
    """Every quad down on the last row: disarmed and within 3 cm of its first height."""
    out = []
    for i in range(run.n):
        z = run.df[f'd{i}_z'].to_numpy(float)
        out.append(bool(abs(z[-1] - np.nanmedian(z[:20])) <= 0.03 and run.df[f'd{i}_armed'].iloc[-1] < 0.5))
    return all(out)


def validity(run):
    p = run.sm.get('planner', {})
    return {'rtf': run.man.get('mean_rtf'), 'late_planner': p.get('late'), 'late_tracker': run.sm['tracker_late_max'],
            'failed': p.get('failed'), 'published': p.get('published'), 'published_res_over': p.get('published_res_over'),
            'published_res_missing': p.get('published_res_missing'), 'void': run.sm['void'],
            'dropped': run.sm['dropped'], 'aborts': [a for _, a in run.all('FLEET_ABORT')],
            'timeouts': sorted({k for _, k, _ in run.events if 'TIMEOUT' in k}),
            'rejoin_refused': len(re.findall(r'REJOIN REFUSED', run.launch)),
            'rejoin_abort': len(re.findall(r'REJOIN ABORT', run.launch)),
            'low_rtf_flag': (run.man.get('mean_rtf') or 1.0) < XB.LOW_RTF}


def overlap(run, js, ref_mean):
    """b2b: each fold-in peak against the claim chain's mean + 1 deg; the return (last RELEASE to the last BLEND_OVER);
    each rejoiner's closest approach (body to body, 3-D) to the quad folding in before it (its weld to its blend over)."""
    out = {'bar_deg': None if ref_mean is None else ref_mean + 1.0}
    if ref_mean is not None:
        out['missed'] = [r['drone'] for r in js if r['fold_peak_deg'] is None or r['fold_peak_deg'] > ref_mean + 1.0]
    last_rel = max(te for te, _ in run.all('RELEASE'))
    blends = [te for te, _ in run.all('REJOIN_BLEND_OVER')]
    out['return_s'] = (max(blends) - last_rel) if blends else None
    close = []
    for prev, nxt in zip(js[:-1], js[1:]):
        blend = run.first('REJOIN_BLEND_OVER', prev['drone'], prev['t_weld']) if prev['t_weld'] else None
        if blend is None:
            close.append(None)
            continue
        m = run.win(prev['t_weld'], blend)
        close.append(float(np.nanmin(np.linalg.norm(run.xyz(nxt['drone'])[m] - run.xyz(prev['drone'])[m], axis=1))))
    out['closest_m'] = close
    return out


def walked(run):
    """The ring's reference: where it ended (the last row before LAND) against where it was at the top."""
    i0 = int(np.searchsorted(run.t, S.top_time(run.t, run.ref[:, 2], run.t_land)))
    i1 = max(int(np.searchsorted(run.t, run.t_land)) - 1, 0)
    return {'walked_m': float(np.linalg.norm(run.ref[i1, :2] - run.ref[i0, :2])), 'to': [float(v) for v in run.ref[i1, :2]]}


def staged(run, drones, t_from, pad=0.0):
    """E4 for staged quads from their release (+pad) to the descent: the largest |x|, |y| and the closest horizontal
    distance to a carrier's body."""
    carriers = [i for i in range(run.n) if i not in drones]
    out = {}
    for d, t0 in zip(drones, t_from):
        m = run.win(t0 + pad, run.t_land)
        p = run.xyz(d)[m]
        near = min(float(np.nanmin(np.linalg.norm(p[:, :2] - run.xyz(c)[m][:, :2], axis=1))) for c in carriers)
        out[str(d)] = {'box_x_m': float(np.nanmax(np.abs(p[:, 0]))), 'box_y_m': float(np.nanmax(np.abs(p[:, 1]))),
                       'carrier_min_m': near}
    return out


def tension(run, drone, t):
    """The planner's planned tension for drone at the tick at or before t (its slot from slot2drone)."""
    p = run.plan
    i = np.nonzero(p['sim_time'].to_numpy(float) <= t)[0]
    if not i.size:
        return None
    row = p.iloc[i[-1]]
    slots = S.slot_drones(row['slot2drone'])
    return float(row[f't{slots.index(drone)}']) if slots and drone in slots else None


def tasks(run, nb):
    """Each TASK: at the waypoint after TASK START (s), TASK DONE after it (s), the body and the reference off the
    waypoint through the dwell (mm), the closest to another quad's body (m), the largest |x|, |y| (m), and the ring's
    position RMSE meanwhile (N1's bar at its size)."""
    out = []
    for te, arg in run.all('TASK'):
        f = arg.split()
        d = int(f[0])
        wp = np.array([float(v) for v in f[1].split(',')[:3]])
        dwell = float(f[1].split(',')[3])
        t0, t_at, t_done = run.first('TASK_START', d, te), run.first('TASK_AT', d, te), run.first('TASK_DONE', d, te)
        row = {'drone': d, 'at_s': None if None in (t0, t_at) else t_at - t0,
               'done_s': None if None in (t0, t_done) else t_done - t0}
        if t_at is not None:
            m = run.win(t_at, t_at + dwell)
            row['body_off_mm'] = 1e3 * float(np.nanmax(np.linalg.norm(run.xyz(d)[m] - wp, axis=1)))
            rf = run.df[[f'd{d}_ref_x', f'd{d}_ref_y', f'd{d}_ref_z']].to_numpy(float)
            row['ref_off_mm'] = 1e3 * float(np.nanmax(np.linalg.norm(rf[m] - wp, axis=1)))
        if None not in (t0, t_done):
            m = run.win(t0, t_done)
            p = run.xyz(d)[m]
            row['closest_m'] = min(float(np.nanmin(np.linalg.norm(p - run.xyz(o)[m], axis=1)))
                                   for o in range(run.n) if o != d)
            row['box_m'] = [float(np.nanmax(np.abs(p[:, 0]))), float(np.nanmax(np.abs(p[:, 1])))]
            e = (run.pos - run.ref)[m]
            e = e[np.all(np.isfinite(e), axis=1)]
            row['ring_rmse_mm'] = 1e3 * float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
            pn = run.plan['n'].to_numpy(int)[(run.plan['sim_time'] >= t0).to_numpy() & (run.plan['sim_time'] < t_done).to_numpy()]
            row['n'] = int(np.median(pn)) if pn.size else None
            row['ring_in_n1'] = None if nb is None else row['ring_rmse_mm'] <= nb['rmse_mm']
        miss = []
        if row.get('ref_off_mm') is None or row['ref_off_mm'] > 1e3 * TASK_REF_M:
            miss.append('reference')
        if row.get('body_off_mm') is None or row['body_off_mm'] > 1e3 * TASK_BODY_M:
            miss.append('body')
        if row.get('closest_m') is not None and row['closest_m'] < TASK_CLEAR_M:
            miss.append('clearance')
        if row.get('box_m') and max(row['box_m']) > E4_BOX_M:
            miss.append('box')
        if t_done is None:
            miss.append('no TASK DONE')
        row['miss'] = miss
        out.append(row)
    return out


def score_case(run, ctx):
    """The case's own items beside V."""
    c, nb = run.case, ctx['n_bars']
    out = {'run': run.rid, 'case': c, 'dir': run.dir, 'v': validity(run)}
    sm = run.sm
    if c in ('hover', 's4_hover'):
        st = sm['stages'][0]
        out.update(n1=n_check(sm['n1'], nb), lift={'tilt_peak_deg': sm['lift']['tilt_peak_deg'],
                                                    'rmse_mm': 1e3 * sm['lift']['pos_rmse_m']},
                   throttle={'p95': st['thr_p95'], 'cap_frac': st['thr_cap_frac']})
        return out
    if c in ('fig8', 'fig8laps', 'orbit'):
        e, tl = XB.n2_bands.path_metrics_clock_free(run.dir)
        out['path'] = {'mm': 1e3 * e, 'tilt_deg': tl}
        out['box'] = airborne_box(run)
        if c == 'orbit':
            out['path']['in_band'] = bool(1e3 * e <= ORBIT_BAND_MM and tl <= ORBIT_BAND_DEG)
            out['path']['box_ok'] = max(out['box']) <= E4_BOX_M
        return out
    rels = releases(run)
    out['L'] = l_items(rels, sm['dropped'])
    js = j_items(run)
    out['J'] = js
    out['n4'] = n_check(sm['n4'], nb)
    if c == 'adjpair':
        g = rels[0]
        rz = [te for te, k, _ in run.events if k == 'DETACH_RESIZE' and te >= g['t']]
        a = (g['t'] + g['n_steps'][-1] + S.TOP_PAD_S) if g['n_steps'][-1] is not None else g['t'] + S.TOP_PAD_S
        six = S._steady(run.t[~np.isnan(run.mag)], (run.pos - run.ref)[~np.isnan(run.mag)], run.mag[~np.isnan(run.mag)],
                        a, run.t_land)
        thr = sm['stages'][-1]
        tail = run.win(run.t_land - S.SETTLE_S, run.t_land)
        out['pair'] = {'n_steps': g['n_steps'], 'peak_deg': g['peak_deg'], 'abs_deg': g['abs_deg'],
                       'settled_deg': float(np.nanmean(run.mag[tail])), 'resize_events_s': [x - g['t'] for x in rz],
                       'six_rmse_mm': 1e3 * six['pos_rmse_m'] if six['pos_rmse_m'] else None,
                       'six_tilt_deg': six['tilt_peak_deg'], 'thr_p95': thr['thr_p95'], 'thr_cap_frac': thr['thr_cap_frac']}
        x = out['pair']
        x['falsified'] = bool(sm['dropped'] or validity(run)['aborts'] or x['n_steps'][-1] is None
                              or x['n_steps'][-1] > L_RESIZE_S or x['peak_deg'] > L_ABS_DEG or x['settled_deg'] > SETTLED_DEG)
        return out
    if c in ('chain4b2b', 'chain4b2b_orbit'):
        out['overlap'] = overlap(run, js['items'], ctx['claim_fold_mean'])
        out['walk'] = walked(run)
    if c == 'chain4b2b_orbit':
        last = max(te for te, _ in run.all('REJOIN_BLEND_OVER'))
        e, tl = XB.n2_bands.path_metrics_clock_free(run.dir, t_from=last + S.TOP_PAD_S)
        e = 1e3 * e
        pf = ctx['p'].get('orbit')
        out['P'] = {'mm': e, 'tilt_deg': tl, 'form': pf,
                    'holds': None if pf is None else bool(e <= pf['bar_mm'] and tl <= pf['bar_tilt_deg'])}
        out['box_ring'] = [float(np.nanmax(np.abs(run.pos[run.win(run.t[0], run.t_land), i]))) for i in (0, 1)]
        out['box'] = airborne_box(run)
    if c in ('chain4back_fig8laps', 'fig8rel2'):
        e, tl = XB.n2_bands.path_metrics_clock_free(run.dir)
        pf = ctx['p'].get('fig8laps' if c == 'chain4back_fig8laps' else 'fig8')
        out['P'] = {'mm': 1e3 * e, 'tilt_deg': tl, 'form': pf, 'holds': None if pf is None else bool(1e3 * e <= pf['bar_mm']),
                    'tilt_note': 'not barred: the window holds the events'}
    if c == 'chain4back_fig8laps':
        tt = run.plan['traj_t'].to_numpy(float)
        live = run.plan['phase'].to_numpy() == 'planner'
        out['lap_clock'] = {'stepped_back': bool(np.any(np.diff(tt[live]) < -1e-6)),
                            'lap_end_holds': len(run.all('LAP_DONE'))}
    if c == 'fig8rel2':
        rel = [(te, int(float(a))) for te, a in run.all('RELEASE')]
        out['staged'] = staged(run, [d for _, d in rel], [te for te, _ in rel])
        out['staged_from_5s'] = staged(run, [d for _, d in rel], [te for te, _ in rel], pad=5.0)
        out['box'] = airborne_box(run)
    if c == 'pairs':
        out['throttle'] = {}
        for st in sm['stages']:
            if st['n'] in (6, 4) and str(st['n']) not in out['throttle']:
                out['throttle'][str(st['n'])] = (st['thr_p95'], st['thr_cap_frac'])
    if c == 'relblend':
        welds = [te for te, _ in run.all('REJOIN_WELD')]
        rel = [(te, int(float(a))) for te, a in run.all('RELEASE')]
        te, d = next(x for x in rel if x[0] > min(welds))
        g = next(x for x in rels if abs(x['t'] - te) < 1e-6)
        rj = js['items'][-1]
        w15 = run.win(te, te + 15.0)
        pt, pn = run.plan['sim_time'].to_numpy(float), run.plan['n'].to_numpy(int)
        after = np.nonzero((pt >= te) & (pn == 7))[0]
        t_rs = float(pt[after[0]]) if after.size else None
        out['added'] = {'drone': d, 't_after_weld_s': te - rj['t_weld'] if rj.get('t_weld') else None,
                        'planner_at_7_s': None if t_rs is None else t_rs - te,
                        'rejoiner_kept': bool(after.size and rj['drone'] in (S.slot_drones(run.plan['slot2drone'].iloc[after[0]]) or [])),
                        'abs_15s_deg': float(np.nanmax(run.mag[w15])), 'peak_deg': g['peak_deg'],
                        'settled_deg': float(np.nanmean(run.mag[run.win(run.t_land - S.SETTLE_S, run.t_land)])),
                        'all_landed': landing(run), 'rejoiner': rj['drone'],
                        'rejoiner_tension_before_N': tension(run, rj['drone'], te - 1e-3),   # the tick before
                        'rejoiner_tension_after_N': None if t_rs is None else tension(run, rj['drone'], t_rs),
                        'rejoiner_tension_after_low_N': None if t_rs is None else min(
                            tension(run, rj['drone'], x) for x in pt[(pt >= t_rs) & (pt <= t_rs + 1.0)]),
                        'rejoiner_rod_end_cm': rj['rod_end_cm']}
        x = out['added']
        x['holds'] = bool(not sm['dropped'] and x['planner_at_7_s'] is not None and x['planner_at_7_s'] <= L_RESIZE_S
                          and x['rejoiner_kept'] and x['abs_15s_deg'] <= RELBLEND_ABS_DEG and x['peak_deg'] <= L_ABS_DEG
                          and x['settled_deg'] <= SETTLED_DEG and x['all_landed'])
        # the claim part's L leaves the added release out
        out['L'] = l_items([x for x in rels if abs(x['t'] - te) > 1e-6], sm['dropped'])
    if c == 'task':
        out['tasks'] = tasks(run, nb)
    if c == 'chain2back':
        det = []
        for e in sm['events']:
            if e['event'] == 'DETACH':
                det.append({'drone': e['drone'], 't': e['t'], 'n_after': e['n_after'], 'resize_s': e['resize_s'],
                            'peak_deg': e['peak_deg']})
        out['detaches'] = det
        out['unload_cancelled'] = re.findall(r'UNLOAD CANCELLED[^\n]*', run.launch)[:2]
        if det:
            d0 = det[0]
            t_t = d0['t'] + (d0['resize_s'] or 0.0) + 5.0
            w2 = run.first('REJOIN_WELD', d0['drone'], d0['t'])
            m = run.win(t_t, w2 or run.t_land)
            out['T_tilt'] = {'mean_deg': float(np.nanmean(run.mag[m])), 'max_deg': float(np.nanmax(run.mag[m])),
                             'from': t_t, 'to': w2}
        pn = run.plan['n'].to_numpy(int)[(run.plan['phase'] == 'planner').to_numpy()]
        out['n_min'] = int(pn.min()) if pn.size else None
        out['throttle_n34'] = [(s['n'], s['thr_p95'], s['thr_cap_frac']) for s in sm['stages'] if s['n'] in (3, 4)]
        v = out['v']
        out['falsified'] = bool(v['aborts'] or v['dropped'] or v['failed'] or v['void'] or v['rejoin_refused']
                                or v['timeouts'] or any(g['peak_deg'] > L_PEAK_DEG for g in rels)
                                or out['n_min'] is None or out['n_min'] > 2)
    return out


def score_all(dirs):
    runs = [Run(d) for d in dirs]
    s4 = [r for r in runs if r.case == 's4_hover' and not r.sm['void']]     # a void run sets no comparator
    nb = n_bars(s4)
    ctx = {'n_bars': nb, 'p': {}, 'claim_fold_mean': None}
    out = {}
    for r in runs:
        if r.case in ('hover', 's4_hover', 'fig8', 'fig8laps', 'orbit'):
            out[r.rid] = score_case(r, ctx)
    for base in ('fig8', 'fig8laps', 'orbit'):
        ctx['p'][base] = p_form([(out[r.rid]['path']['mm'], out[r.rid]['path']['tilt_deg']) for r in runs
                                 if r.case == base and not r.sm['void']])
    claim = [r for r in runs if r.case == 'chain4back']
    for r in claim:
        out[r.rid] = score_case(r, ctx)
    folds = [x['fold_peak_deg'] for r in claim if not r.sm['void'] for x in out[r.rid]['J']['items']
             if x['fold_peak_deg'] is not None]
    ctx['claim_fold_mean'] = float(np.mean(folds)) if folds else None
    for r in runs:
        if r.rid not in out:
            out[r.rid] = score_case(r, ctx)
    return {'card': 'docs/experiments/2026-10-23_scalability.md', 'n_bars': nb, 'p_bases': ctx['p'],
            'claim_fold_mean_deg': ctx['claim_fold_mean'], 'runs': out}


def _f(v, nd=2):
    return '-' if v is None else f'{v:.{nd}f}'


def _rng(v, nd=2):
    v = [x for x in v if x is not None]
    return '-' if not v else (_f(v[0], nd) if len(v) == 1 else ', '.join(_f(x, nd) for x in v))


def report(doc):
    nb = doc['n_bars']
    print('N1 / N4 bars: ' + (f"<= {nb['rmse_mm']:.2f} mm, <= {nb['tilt_deg']:.2f} deg (from {', '.join(nb['from'])})"
                              if nb else 'not set (give the x3_s4_hover_it5 runs)'))
    for k, p in doc['p_bases'].items():
        if p:
            print(f"P's base {k}: mean {p['base_mm']:.2f} mm -> bar {p['bar_mm']:.2f} mm, tilt {p['bar_tilt_deg']:.2f} deg")
    if doc['claim_fold_mean_deg'] is not None:
        print(f"the claim chain's mean fold-in peak {doc['claim_fold_mean_deg']:.2f} deg -> the overlap bar "
              f"{doc['claim_fold_mean_deg'] + 1:.2f} deg")
    hold = lambda v: '-' if v is None else ('holds' if v else 'MISS')  # noqa: E731
    for rid, o in doc['runs'].items():
        v = o['v']
        lines = [f"{rid} {o['case']}: RTF {_f(v['rtf'], 3)}{' (under 0.25: flagged)' if v['low_rtf_flag'] else ''} / late "
                 f"{100 * v['late_planner']:.2f}, {100 * v['late_tracker']:.2f} % / failed {v['failed']} / published over "
                 f"1e-2 {v['published_res_over']} of {v['published']}" + (' / VOID (late ticks)' if v['void'] else '')
                 + (' / DROPPED' if v['dropped'] else '') + (f" / aborts {v['aborts']}" if v['aborts'] else '')
                 + (f" / {', '.join(v['timeouts'])}" if v['timeouts'] else '')
                 + (f" / REJOIN REFUSED {v['rejoin_refused']}" if v['rejoin_refused'] else '')]
        if o.get('n1'):
            n = o['n1']
            lines.append(f"  N1 {_f(n['rmse_mm'])} mm, {_f(n['tilt_deg'], 3)} deg: {hold(n.get('pos_holds'))}/"
                         f"{hold(n.get('tilt_holds'))}; lift {_f(o['lift']['tilt_peak_deg'])} deg, {_f(o['lift']['rmse_mm'], 1)} mm;"
                         f" throttle p95 {_f(o['throttle']['p95'], 3)}, at the cap {_f(100 * (o['throttle']['cap_frac'] or 0), 1)} %")
        if o.get('path'):
            p = o['path']
            lines.append(f"  path {_f(p['mm'])} mm, tilt {_f(p['tilt_deg'])} deg; quads' largest |x|, |y| "
                         f"{_f(o['box'][0])}, {_f(o['box'][1])} m" + (f"; orbit band (33.0 mm, 4.06 deg) "
                                                                      f"{hold(p['in_band'])}, box {hold(p['box_ok'])}"
                                                                      if 'in_band' in p else ''))
        if o.get('L') and o['L']['items']:
            it = o['L']['items']
            lines.append(f"  L {hold(o['L']['holds'])}: peaks {_rng([g['peak_deg'] for g in it])}; n-1 "
                         f"{_rng([s for g in it for s in g['n_steps']])} s; settled {_rng([g['settled_deg'] for g in it])}"
                         + ''.join(f"; {g['drones']} missed {g['miss']}" for g in it if g['miss']))
        if o.get('J') and o['J']['items']:
            it = o['J']['items']
            lines.append(f"  J {hold(o['J']['holds'])}: on plate {_rng([r['plate_s'] for r in it], 1)}; joint <= "
                         f"{_f(max([r['joint_s'] for r in it if r['joint_s'] is not None] or [None]))}; one bigger "
                         f"{_rng([r['bigger_s'] for r in it], 1)}; fold {_rng([r['fold_peak_deg'] for r in it])}; rod "
                         f"{_rng([r['rod_end_cm'] for r in it], 1)}; settled {_rng([r['settled_deg'] for r in it])}"
                         + ''.join(f"; /drone_{r['drone']} missed {r['miss']}" for r in it if r['miss']))
        if o.get('n4') and o['case'] in N4_CASES:
            n = o['n4']
            lines.append(f"  N4 (n={n['n']}) {_f(n['rmse_mm'])} mm, {_f(n['tilt_deg'])} deg: {hold(n.get('pos_holds'))}/"
                         f"{hold(n.get('tilt_holds'))}")
        if o.get('pair'):
            x = o['pair']
            lines.append(f"  pair: planner at n-1, n-2 {_rng(x['n_steps'])} s; peak {_f(x['peak_deg'])} ({_f(x['abs_deg'])} "
                         f"abs) deg; settled {_f(x['settled_deg'])} deg; the six {_f(x['six_rmse_mm'], 1)} mm, "
                         f"{_f(x['six_tilt_deg'])} deg; throttle p95 {_f(x['thr_p95'], 3)}, cap "
                         f"{_f(100 * (x['thr_cap_frac'] or 0), 1)} %: {'FALSIFIED' if x['falsified'] else 'no falsifier'}")
        if o.get('overlap'):
            x = o['overlap']
            lines.append(f"  overlap (fold-in <= {_f(x['bar_deg'])} deg): missed on {len(x.get('missed') or [])} of "
                         f"{len(o['J']['items'])}; return {_f(x['return_s'], 1)} s; closest rejoiner {_rng(x['closest_m'])} m;"
                         f" the hover point walked {o['walk']['walked_m']:.2f} m to ({o['walk']['to'][0]:.2f}, "
                         f"{o['walk']['to'][1]:.2f})")
        if o.get('P'):
            x = o['P']
            lines.append(f"  P {_f(x['mm'])} mm, {_f(x['tilt_deg'])} deg: "
                         + (f"bar {_f(x['form']['bar_mm'])} mm" + (f", {_f(x['form']['bar_tilt_deg'])} deg"
                                                                   if o['case'] == 'chain4b2b_orbit' else '')
                            + f" -> {hold(x['holds'])}" if x['form'] else 'no base given: not scorable'))
        if o.get('box_ring'):
            lines.append(f"  ring to |x| {o['box_ring'][0]:.2f}, |y| {o['box_ring'][1]:.2f} m; quads airborne to "
                         f"{o['box'][0]:.2f}, {o['box'][1]:.2f} m")
        if o.get('lap_clock'):
            lines.append(f"  lap clock stepped back: {o['lap_clock']['stepped_back']}; lap-end holds "
                         f"{o['lap_clock']['lap_end_holds']}")
        if o.get('staged'):
            lines.append('  staged: ' + '; '.join(f"/drone_{d} |x| {x['box_x_m']:.2f}, |y| {x['box_y_m']:.2f}, carrier "
                                                   f"{x['carrier_min_m']:.2f} m ({o['staged_from_5s'][d]['carrier_min_m']:.2f}"
                                                   f" from +5 s)" for d, x in o['staged'].items())
                         + f"; E4 (+-1.6 m, 0.8 m) {hold(all(max(x['box_x_m'], x['box_y_m']) <= E4_BOX_M and x['carrier_min_m'] >= E4_CARRIER_M for x in o['staged'].values()))}")
        if o.get('throttle') and o['case'] == 'pairs':
            lines.append('  throttle p95, cap at n: ' + '; '.join(f"{k}: {_f(a, 3)}, {_f(100 * (b or 0), 1)} %"
                                                                   for k, (a, b) in o['throttle'].items()))
        if o.get('added'):
            x = o['added']
            lines.append(f"  added RELEASE /drone_{x['drone']} {_f(x['t_after_weld_s'], 1)} s after the last weld: planner "
                         f"at 7 {_f(x['planner_at_7_s'])} s, rejoiner kept {x['rejoiner_kept']}, abs {_f(x['abs_15s_deg'])} "
                         f"deg (<= 10), from the 5 s mean {_f(x['peak_deg'])} deg, settled {_f(x['settled_deg'])} deg, all "
                         f"landed {x['all_landed']}; rejoiner /drone_{x['rejoiner']} tension {_f(x['rejoiner_tension_before_N'])}"
                         f" -> {_f(x['rejoiner_tension_after_N'])} N (its lowest in the next 1 s {_f(x['rejoiner_tension_after_low_N'])}), rod end {_f(x['rejoiner_rod_end_cm'], 1)} cm: "
                         f"{'holds' if x['holds'] else 'MISS'}")
        for x in o.get('tasks') or []:
            lines.append(f"  task /drone_{x['drone']}: at {_f(x['at_s'], 1)} s, done {_f(x['done_s'], 1)} s; body "
                         f"{_f(x.get('body_off_mm'), 1)} mm, reference {_f(x.get('ref_off_mm'), 1)} mm; closest {_f(x.get('closest_m'))} m; box {_rng(x.get('box_m') or [])}"
                         f" m; ring {_f(x.get('ring_rmse_mm'))} mm at n={x.get('n')} (N1 {hold(x.get('ring_in_n1'))})"
                         + (f"; missed {x['miss']}" if x['miss'] else ''))
        if o['case'] == 'chain2back':
            for x in o['detaches']:
                lines.append(f"  DETACH /drone_{x['drone']}: n={x['n_after']} at {_f(x['resize_s'])} s, peak {_f(x['peak_deg'])} deg")
            if o.get('T_tilt'):
                lines.append(f"  the T: tilt mean {o['T_tilt']['mean_deg']:.1f} deg (max {o['T_tilt']['max_deg']:.1f})")
            lines.append(f"  UNLOAD CANCELLED lines {len(o['unload_cancelled'])}; the planner's least n {o['n_min']}; throttle"
                         f" at n=3-4 {[(n, _f(a, 3), _f(100 * (b or 0), 1)) for n, a, b in o['throttle_n34']]}: "
                         + ('FAIL as 8 -> 2 -> 8' if o['falsified'] else 'no falsifier'))
        print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='R#### or run directories, with the comparators')
    ap.add_argument('--json', help='write the whole score here')
    a = ap.parse_args()
    dirs = []
    for r in a.runs:
        d = S.locate(r)
        if case_of(d) not in CASES:
            print(f'{r}: {os.path.basename(d)} is not an X3 case, left out')
        elif os.path.isfile(os.path.join(d, 'logs', 'run.csv')):
            dirs.append(d)
        else:
            print(f'{r}: no logs/run.csv, not scored (void?)')
    doc = score_all(dirs)
    report(doc)
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(XB.strip(doc), fh, indent=1, default=str)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
