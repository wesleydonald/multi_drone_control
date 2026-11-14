#!/usr/bin/env python3
"""tools/x2_metrics.py -- score X2-full, the chain 4 -> 3 -> 2 -> 3 -> 4 on the figure-8 in one run
(card docs/experiments/2026-10-30_x2_full_chain_fig8.md), against its bars as frozen 30 Oct ~15:46 and 31 Oct.

    python3 tools/x2_metrics.py R3586 R3587 R3588 R3589 R3590 R3591 [--json results/analysis/x2_<date>.json]

The variant comes from the run's name: x2_full_fig8_base (the base, no event), _soft (A: the square-to-T step
announced), _unann (B: unannounced, the drop at 80 deg). Give the base's runs with the chains: A5 and lap 1's check read
the base's laps 1 and 6 (P's form, max(1.25 x the mean, the mean + 3 mm) and the mean tilt + 1 deg).

Per run: tools/s_metrics.score() (events, stages, late ticks, planner), tools/x1_metrics.score() for B's step,
tools/release_bands.Run for T6 windowed to the next event, and the lap windows the card asks for (section 8: lap k = the
planner's trajectory clock from (k-1) to k laps, moved onto run.csv's clock by the manifest's t_ready_sim, checked per
run against the ring pose in clock_check). This is the 31 Oct scorer (results/analysis/x2_full_2026-10-31/score_x2.py
with lap_tools.py, the reviewer's copy of 4 Nov), its run ids made arguments; the definitions are unchanged.
B1's bar for the later runs (31 Oct, value 5 deg set 4 Nov ~22:20 after the runs): the largest tilt from 6 s after
the joint's release to the next event (step1 tilt_after_6s_deg); the original T6 (<= 5.0 s) is reported beside it.
"""
import argparse
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
import release_bands as RB  # noqa: E402
import run_logs  # noqa: E402
import s_metrics as S  # noqa: E402
import x1_metrics as X  # noqa: E402

LAP_S = 34.5          # the launch log's 'lap 1 done at t=34.50 s'; checked per run in load()
TT_EPS = 0.006        # the planner logs traj_t to 2 decimals


class Run:
    """One run: run.csv (the runner's clock), events.csv (the same clock), the planner's log moved onto that clock by
    the manifest's t_ready_sim (s_metrics.node_log; the offset is checked against the ring pose in clock_check())."""

    def __init__(self, rid):
        self.dir = S.locate(rid)
        self.rid = os.path.basename(self.dir.rstrip('/')).split('_')[0]
        self.logs = os.path.join(self.dir, 'logs')
        self.man = json.load(open(os.path.join(self.dir, 'manifest.json')))
        self.df = pd.read_csv(os.path.join(self.logs, 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        self.t = self.df.t.to_numpy(float)
        self.ev = pd.read_csv(os.path.join(self.logs, 'events.csv'))
        self.events = [(float(r.sim_time), str(r.event), None if pd.isna(r.arg) else str(r.arg))
                       for r in self.ev.itertuples()]
        self.t_ready = float(self.man['t_ready_sim'])
        pdirs = run_logs.node_dirs(self.dir, 'dissipative_planner') + run_logs.node_dirs(self.dir, 'mpc_planner')
        self.pdir = pdirs[-1]
        self.plan = pd.read_csv(os.path.join(self.pdir, 'log.csv'))
        self.plan['t_run'] = self.plan['sim_time'] - self.t_ready
        self.launch = open(os.path.join(self.logs, 'launch.log'), errors='replace').read()
        q = self.df[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
        self.ok = np.all(np.isfinite(q), axis=1)
        self.th = np.full((len(self.t), 2), np.nan)
        self.th[self.ok] = RB.tilt_vector(q[self.ok])
        self.mag = np.linalg.norm(self.th, axis=1)
        w, x, y, z = q.T
        self.yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.pos = self.df[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
        self.ref = self.df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
        self.t_land = self.first('LAND')
        self._cf = None

    def first(self, kind, arg=None, after=None):
        for te, k, a in self.events:
            if k == kind and (arg is None or (a is not None and a.split('.')[0] == str(arg))) \
                    and (after is None or te >= after):
                return te
        return None

    def win(self, a, b):
        return (self.t >= a) & (self.t < b)

    def clock_check(self):
        """(lag, rms mm): the planner's load pose against run.csv's ring pose, the lag that fits best in 10 ms steps;
        lag = t_ready_sim means s_metrics.node_log's alignment holds for this run."""
        grid = np.arange(self.t[0] + 5.0, self.t[-1] - 5.0, 0.02)
        r = np.stack([np.interp(grid, self.t, self.pos[:, i]) for i in range(3)], 1)
        best = (None, np.inf)
        for lag in np.arange(self.t_ready - 1.0, self.t_ready + 1.0 + 1e-9, 0.01):
            p = np.stack([np.interp(grid + lag, self.plan.sim_time, self.plan[c]) for c in ('load_x', 'load_y', 'load_z')], 1)
            c = float(np.mean(np.sum((p - r) ** 2, axis=1)))
            if c < best[1]:
                best = (float(lag), c)
        return best[0], 1e3 * np.sqrt(best[1])

    def laps(self, n=6, lap_s=LAP_S):
        """[(k, t_start, t_end)] on the run clock."""
        pt, tt = self.plan.t_run.to_numpy(float), self.plan.traj_t.to_numpy(float)
        out = []
        for k in range(1, n + 1):
            a = np.nonzero(tt > (k - 1) * lap_s + TT_EPS)[0]
            b = np.nonzero(tt >= k * lap_s - TT_EPS)[0]
            out.append((k, float(pt[a[0]]) if a.size else None, float(pt[b[0]]) if b.size else None))
        return out

    def clock_free(self):
        """The ring's 3-D distance to the path its reference traces, on every row (x1_metrics._clock_free_err)."""
        if self._cf is None:
            self._cf = X._clock_free_err(self.df, self.t, np.ones(len(self.t), bool))
        return self._cf

    def lap_table(self):
        rows = []
        cf = self.clock_free()
        for k, a, b in self.laps():
            if a is None or b is None:
                rows.append({'lap': k, 'from': a, 'to': b, 'path_mm': None, 'tilt_peak_deg': None, 'tilt_rms_deg': None})
                continue
            m = self.win(a, b)
            e = cf[m]
            e = e[np.isfinite(e)]
            rows.append({'lap': k, 'from': a, 'to': b, 'path_mm': 1e3 * float(np.sqrt(np.mean(e ** 2))),
                         'tilt_peak_deg': float(np.nanmax(self.mag[m])),
                         'tilt_rms_deg': float(np.sqrt(np.nanmean(self.mag[m] ** 2))),
                         'z_mm': 1e3 * float(np.sqrt(np.nanmean((self.pos[m, 2] - self.ref[m, 2]) ** 2)))})
        return rows


def variant_of(d):
    """'A' (soft), 'B' (unann) or 'base', from the run's name."""
    name = os.path.basename(d.rstrip('/'))
    for key, v in (('_soft', 'A'), ('_unann', 'B'), ('_base', 'base')):
        if key in name:
            return v
    raise ValueError(f'{name}: not an x2_full_fig8_{{base,soft,unann}} run')


# the card's frozen numbers, section 5
A1_N3_S, A1_PEAK = 6.0, 30.0
A2_N2_S, A2_PEAK, A2_LEAN, L_PEAK, L_SETTLED = 1.0, 15.0, 5.0, 8.83, 3.0
RJ_PLATE_S, RJ_JOINT_S, RJ_FOLD_DEG, RJ_ROD_CM, RJ_SHARE, J_BIGGER_S = 30.0, 0.5, 5.0, 2.0, 0.10, 20.0
A4_LEAN = 2.0
E4_BOX, E4_RING, E4_CARRIER = 1.6, 0.6, 0.8
B1_PEAK, B1_T6 = 80.0, 5.0
B1_AFTER6_DEG = 5.0       # the later runs' bar in place of T6 (31 Oct; its value set by Wesley 4 Nov ~22:20)
READBACK = {'traj_laps': '6', 'traj_lap_hold_s': '3.0', 'detach_unload_s': '3.0', 'rejoin_departed': 'true',
            'tip_detect_m': '0.04', 'drop_gap_deg': '200.0', 'min_survivors': '2'}


def rd(v, nd=3):
    return None if v is None or not np.isfinite(v) else round(float(v), nd)


def params(run, name):
    """Flat key: value lines of a read-back yaml."""
    out = {}
    path = os.path.join(run.dir, 'params', name + '.yaml')
    if not os.path.isfile(path):
        return out
    for line in open(path):
        m = re.match(r'\s+([A-Za-z_0-9]+): (.*)$', line)
        if m:
            out[m.group(1)] = m.group(2).strip().strip("'\"")
    return out


def tracker_params_json(run, i):
    """A tracker's own params.json (logs/tracker/drone<i>_*), as strings, when the runner's read-back missed the node
    (R3590's /tracker_3)."""
    for d in run_logs.node_dirs(run.dir, 'tracker'):
        f = os.path.join(d, 'params.json')
        if os.path.basename(d).startswith(f'drone{i}_') and os.path.isfile(f):
            return {k: str(v) for k, v in json.load(open(f)).items() if not isinstance(v, (dict, list))}
    return {}


def validity(run, sm):
    pl = params(run, 'dissipative_planner')
    rb = {k: pl.get(k) for k in READBACK}
    tr = [params(run, f'tracker_{i}') or tracker_params_json(run, i) for i in range(4)]
    mc = [params(run, f'mocap_{i}') for i in range(4)]
    rb['drop_tilt_deg_trackers'] = sorted({t.get('drop_tilt_deg') for t in tr}, key=str)
    rb['max_payload_tilt_deg_trackers'] = sorted({t.get('max_payload_tilt_deg') for t in tr}, key=str)
    rb['max_tilt_deg_trackers'] = sorted({t.get('max_tilt_deg') for t in tr}, key=str)
    rb['velocity_filter_mocaps'] = sorted({m.get('velocity_filter') for m in mc}, key=str)
    rb['lab_env'] = (run.man.get('lab_env') or {})
    rb['int_k_xy'] = pl.get('int_k_xy')
    rb['plan_sqp_iters'], rb['plan_fast_refs'] = pl.get('plan_sqp_iters'), pl.get('plan_fast_refs')
    rb_ok = (all(pl.get(k) == v for k, v in READBACK.items()) and rb['drop_tilt_deg_trackers'] == ['80.0']
             and rb['velocity_filter_mocaps'] == ['slope'])
    kinds = {k for _, k, _ in run.events}
    timeouts = sorted(k for k in kinds if 'TIMEOUT' in k)
    refused = len(re.findall(r'REJOIN REFUSED', run.launch))
    aborts = len(re.findall(r'REJOIN ABORT', run.launch))
    drops = re.findall(r'DROP AND LAND[^\n]*', run.launch)
    gap = [d for d in drops if 'gap' in d]
    fleet_abort = [a for _, k, a in run.events if k == 'FLEET_ABORT']
    p = sm['planner']
    out = {'late_planner': p['late'], 'late_tracker_max': sm['tracker_late_max'], 'failed': p['failed'],
           'unconverged': p['unconverged'], 'solve_ms_p50': p['solve_ms_p50'], 'solve_ms_p95': p['solve_ms_p95'],
           'solve_ms_max': p['solve_ms_max'], 'published': p['published'], 'published_res_over': p['published_res_over'],
           'published_res_missing': p['published_res_missing'], 'readback': rb, 'readback_ok': bool(rb_ok),
           'timeouts': timeouts, 'rejoin_refused': refused, 'rejoin_abort': aborts, 'drop_lines': drops,
           'gap_drop': len(gap), 'fleet_abort': fleet_abort, 'rtf': run.man.get('mean_rtf'),
           'wall_s': run.man.get('wall_s'), 'sim_s': run.man.get('sim_duration_s'), 'slot': (run.man.get('lab') or {}).get('slot'),
           'git_sha': run.man.get('git_sha'), 'diff_sha256': run.man.get('diff_sha256'),
           'dirty_files': run.man.get('dirty_files'), 'exit_reason': run.man.get('exit_reason'),
           'passed': run.man.get('passed'), 't_ready_sim': run.t_ready,
           'slow_tick_lines': re.findall(r'SLOW TICK ([\d.]+) s', run.launch),
           'envelope_warnings': re.findall(r'envelope warning: payload tilt ([\d.]+) deg', run.launch),
           'envelope_faults': re.findall(r'[^\n]*(?:ENVELOPE|envelope fault|FLEET ABORT|FLEET_ABORT)[^\n]*', run.launch)[:5],
           'landed_anyway': len(re.findall(r'landed anyway', run.launch))}
    lag, rms = run.clock_check()
    out['clock_check'] = {'lag_s': lag, 'rms_mm': rms, 't_ready_sim': run.t_ready}
    out['void_reasons'] = []
    if p['late'] > S.LATE_VOID or sm['tracker_late_max'] > S.LATE_VOID:
        out['void_reasons'].append('late ticks over 1 %')
    if p['failed']:
        out['void_reasons'].append('failed planner solve')
    if not rb_ok:
        out['void_reasons'].append('read-back off the card')
    if timeouts:
        out['void_reasons'].append('event timeout: ' + ','.join(timeouts))
    if refused:
        out['void_reasons'].append('REJOIN_REFUSED')
    if gap:
        out['void_reasons'].append('gap drop')
    return out


def tilt_over(run, a, b, deg):
    """Seconds of [a, b) with the tilt magnitude over deg."""
    m = run.win(a, b)
    if not m.any():
        return None
    t, mg = run.t[m], run.mag[m]
    dt = np.diff(t, append=t[-1])
    return float(np.sum(dt[np.nan_to_num(mg) > deg]))


def first_over(run, a, b, deg):
    """Seconds after a when the tilt magnitude first passes deg in [a, b)."""
    m = np.nonzero(run.win(a, b) & (np.nan_to_num(run.mag) > deg))[0]
    return float(run.t[m[0]] - a) if m.size else None


def path_10s(run, te, nxt):
    """x1_metrics' path error: RMS clock-free over te .. te + 10 s."""
    m = (run.t >= te) & (run.t <= min(te + X.AFTER_S, nxt))
    e = run.clock_free()[m]
    e = e[np.isfinite(e)]
    return 1e3 * float(np.sqrt(np.mean(e ** 2)))


def event_row(run, te, nxt):
    """Tilt rise (release_bands' delta-theta, s_metrics' peak_deg), abs peak, when, and x1_metrics' dip, for an event."""
    pre = run.win(te - RB.PRE_S, te)
    w = run.win(te, nxt)
    mean = np.nanmean(run.th[pre], axis=0)
    dev = np.linalg.norm(run.th[w] - mean, axis=1)
    k = int(np.nanargmax(dev))
    j = int(np.nanargmax(run.mag[w]))
    z = run.pos[:, 2]
    aft = (run.t >= te) & (run.t <= min(te + X.AFTER_S, nxt))
    zmin_i = np.nonzero(aft)[0][int(np.nanargmin(z[aft]))]
    return {'t': te, 'next': nxt, 'rise_deg': float(dev[k]), 't_rise_s': float(run.t[w][k] - te),
            'abs_peak_deg': float(run.mag[w][j]), 't_abs_s': float(run.t[w][j] - te),
            'pre_tilt_deg': float(np.nanmean(run.mag[pre])), 'dip_mm': 1e3 * float(np.nanmean(z[pre]) - z[zmin_i]),
            't_dip_s': float(run.t[zmin_i] - te)}


def t6(run, te, nxt):
    """release_bands' T6 with the window ended at the next event: the post-release steady tilt is the stage's own, the
    mean over the last 5 s before the next event, as the settled tilt in tools/s_metrics.py."""
    rb = RB.Run(run.dir, te=te, kind='given', tland=nxt)
    met = rb.metrics()
    post = rb.th[rb.win(nxt - RB.POST_S, nxt)].mean(0)
    outside = np.nonzero((rb.t >= te) & (rb.t < nxt) & (np.linalg.norm(rb.th - post, axis=1) > met['band_deg']))[0]
    return {'t_back_s': met['t_back'], 'band_deg': met['band_deg'], 'sigma_pre_deg': met['sigma_pre'],
            'post_deg': met['post'], 'post_vec_deg': [float(v) for v in post], 't_peak_s': met['t_peak'],
            'last_exit_s': float(rb.t[outside[-1]] - te) if outside.size else 0.0}


def roll_ppm(run, a, b, d0=0, d1=2):
    """pair_swing's roll (about the survivors' line) and see-saw, for the pair drones d0, d1 (the tool's form names
    drones 0 and 1): p-p over [a, b), the largest p-p per 2 s, the peak frequency."""
    from scipy.signal import welch
    df = run.df
    q = df[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    w, x, y, z = q.T
    nx, ny = 2 * (x * z - w * y), 2 * (y * z + w * x)
    az = np.unwrap(np.arctan2(df[f'd{d0}_y'] - df[f'd{d1}_y'], df[f'd{d0}_x'] - df[f'd{d1}_x']).to_numpy())
    k = 25
    az = np.convolve(np.pad(az, k, mode='edge'), np.ones(2 * k + 1) / (2 * k + 1), mode='valid')
    see = np.degrees(np.arcsin(np.clip(-(nx * np.cos(az) + ny * np.sin(az)), -1, 1)))
    rol = np.degrees(np.arcsin(np.clip(nx * np.sin(az) - ny * np.cos(az), -1, 1)))
    m = run.win(a, b)
    fs = 1.0 / float(np.median(np.diff(run.t)))
    w2 = int(2 * fs)
    idx = np.nonzero(m)[0]
    per2 = [float(np.ptp(rol[i:i + w2])) for i in range(idx[0], idx[-1] - w2 + 1, w2)]
    f, P = welch(rol[m] - rol[m].mean(), fs=fs, nperseg=min(int(m.sum()), int(8 * fs)))
    band = (f >= 0.3) & (f <= 3.0)
    return {'roll_pp_deg': float(np.ptp(rol[m])), 'roll_pp_per2s_max_deg': max(per2) if per2 else None,
            'roll_mean_deg': float(np.mean(rol[m])), 'seesaw_pp_deg': float(np.ptp(see[m])),
            'roll_peak_hz': float(f[band][np.argmax(P[band])])}


def signed(run, a, b, cut=None):
    """Over [a, b), less the span cut: the ring's roll, + = plate 3 up (the sign of
    docs/analysis/why_integral_code_2026-10-31.md; plate 3 is at -y at yaw 0, so plate 3 up leans the normal towards
    +y), and its offset from its reference, towards plate 3 (the same sign as y there: y < 0 is towards plate 3) and
    along x."""
    m = run.win(a, b) & run.ok
    if cut is not None:
        m &= ~run.win(*cut)
    yaw = run.yaw[m]
    away = np.stack([-np.sin(yaw), np.cos(yaw)], 1)         # world direction away from plate 3
    roll = np.sum(run.th[m] * away, axis=1)
    along = np.stack([np.cos(yaw), np.sin(yaw)], 1)
    pitch = np.sum(run.th[m] * along, axis=1)
    e = run.pos[m] - run.ref[m]
    ey = np.sum(e[:, :2] * away, axis=1)
    ex = np.sum(e[:, :2] * along, axis=1)
    return {'from': a, 'to': b, 'roll_mean_deg': float(np.mean(roll)), 'roll_min_deg': float(np.min(roll)),
            'roll_max_deg': float(np.max(roll)), 'pitch_mean_deg': float(np.mean(pitch)),
            'tilt_mean_deg': float(np.mean(run.mag[m])), 'off_y_mm': 1e3 * float(np.mean(ey)),
            'off_x_mm': 1e3 * float(np.mean(ex)), 'off_mm': 1e3 * float(np.hypot(np.mean(ex), np.mean(ey))),
            'off_z_mm': 1e3 * float(np.mean(e[:, 2]))}


def rejoin_share(run, d, t_blend):
    """The rejoiner's planned tension at the blend's end against the balanced share the planner logged at the fold-in
    (RESIZE_INPUTS t_nom), in the planner's log: the last tick at or before REJOIN_BLEND_OVER."""
    share = None
    for m in re.finditer(r'RESIZE_INPUTS (\{[^\n]*\})', run.launch):
        js = json.loads(m.group(1))
        if js.get('what') == 'rejoin' and int(js.get('drone')) == d:
            share = js
    p = run.plan
    i = np.nonzero(p.t_run.to_numpy(float) <= t_blend)[0][-1]
    slots = [int(x) for x in str(p.slot2drone.iloc[i]).split()]
    sl = slots.index(d)
    ten = float(pd.to_numeric(p[f't{sl}'].iloc[i], errors='coerce'))
    tail = p.t_run.to_numpy(float)
    win = (tail > t_blend - 1.0) & (tail <= t_blend)
    return {'slot': sl, 't_end_N': ten, 't_last1s_N': float(pd.to_numeric(p[f't{sl}'][win], errors='coerce').mean()),
            'share_N': float(share['t_nom'][sl]) if share else None, 'hover_xy': share.get('hover_xy') if share else None,
            'q': share['load'][3:7] if share else None, 'tick_t': float(tail[i])}


def rejoin(run, d, t_attach, t_next, sm_event):
    """A3 / A4 for one rejoin, by the rejoin card's staged scoring."""
    ev = lambda k: run.first(k, d, t_attach)  # noqa: E731
    start, plate, joint, weld, fold, blend = (ev('REJOIN_START'), ev('REJOIN_ON_PLATE'), ev('JOINT_ATTACHED'),
                                             ev('REJOIN_WELD'), ev('FOLD_IN'), ev('REJOIN_BLEND_OVER'))
    pat = re.compile(rf'REJOIN BLEND OVER drone \d+ \(/drone_{d}\): its rod end at most ([0-9.]+) cm from its plate '
                     r'point through the blend; ([^\n]*)')
    m = pat.search(run.launch)
    rod = float(m.group(1)) if m else None
    cap = m.group(2) if m else None
    w = run.win(weld, weld + 15.0)
    j = int(np.nanargmax(run.mag[w]))
    over = run.t[w][np.nan_to_num(run.mag[w]) > RJ_FOLD_DEG]
    sh = rejoin_share(run, d, blend)
    pre = run.win(weld - RB.PRE_S, weld)
    just = run.win(weld - 0.5, weld)
    rise = np.linalg.norm(run.th[w] - np.nanmean(run.th[pre], axis=0), axis=1)
    z = run.pos[:, 2]
    aft = (run.t >= weld) & (run.t <= weld + X.AFTER_S)
    weld_line = re.search(rf'REJOIN WELD drone \d+ \(/drone_{d}\): [^\n]*?\(rod end ([0-9.]+) cm from plate \d+, ([0-9.]+) cm/s\)'
                          r'[^\n]*?rod ([0-9.]+) m', run.launch)
    out = {'drone': d, 'fold_rise_deg': float(np.nanmax(rise)), 'fold_rise_at_s': float(run.t[w][int(np.nanargmax(rise))] - weld),
           'weld_dip_mm': 1e3 * float(np.nanmean(z[pre]) - np.nanmin(z[aft])),
           'pre_weld_tilt_deg': float(np.nanmean(run.mag[just])),
           'pre_weld_z_err_mm': 1e3 * float(np.nanmean(run.pos[just, 2] - run.ref[just, 2])),
           'gate_cm': float(weld_line.group(1)) if weld_line else None,
           'gate_cm_s': float(weld_line.group(2)) if weld_line else None,
           'rod_m': float(weld_line.group(3)) if weld_line else None, 't_attach': t_attach, 't_start': start, 't_on_plate': plate, 't_joint': joint, 't_weld': weld,
           't_fold_in': fold, 't_blend_over': blend, 'plate_s': plate - start, 'joint_s': joint - plate,
           'weld_after_attach_s': weld - t_attach, 'blend_s': blend - fold, 'fold_peak_deg': float(run.mag[w][j]),
           'fold_peak_after_weld_s': float(run.t[w][j] - weld),
           'fold_over5_s': [float(over[0] - weld), float(over[-1] - weld)] if over.size else None,
           'fold_over5_time_s': tilt_over(run, weld, weld + 15.0, RJ_FOLD_DEG),
           'rod_end_cm': rod, 'cap_line': cap, 'share': sh,
           'share_err_frac': (sh['t_end_N'] / sh['share_N'] - 1.0) if sh['share_N'] else None,
           'bigger_s': sm_event['resize_s'], 'n_after': sm_event['n_after'],
           'event': event_row(run, t_attach, t_next), 'at_weld': event_row(run, weld, t_next)}
    out['pass_a'] = bool(out['plate_s'] <= RJ_PLATE_S)
    out['pass_b'] = bool(out['joint_s'] <= RJ_JOINT_S and weld >= joint and fold is not None)
    out['pass_tilt'] = bool(out['fold_peak_deg'] <= RJ_FOLD_DEG)
    out['pass_rod'] = bool(rod is not None and rod <= RJ_ROD_CM)
    out['pass_share'] = bool(abs(out['share_err_frac']) <= RJ_SHARE)
    return out


def dist_staged(run, d, a, b, carriers_of):
    """E4 for a staged drone over [a, b): the box (the larger of |x|, |y|), the horizontal and 3-D distance from the
    ring's centre, the distance from every carrier body (carriers_of(t) -> list of drones)."""
    m = run.win(a, b)
    t = run.t[m]
    df = run.df
    p = df[[f'd{d}_x', f'd{d}_y', f'd{d}_z']].to_numpy(float)[m]
    box = np.nanmax(np.abs(p[:, :2]), axis=1)
    c = run.pos[m]
    rh = np.linalg.norm(p[:, :2] - c[:, :2], axis=1)
    r3 = np.linalg.norm(p - c, axis=1)
    near_h, near_3 = np.full(len(t), np.inf), np.full(len(t), np.inf)
    who = np.full(len(t), -1)
    for o in range(4):
        if o == d:
            continue
        carrying = np.array([o in carriers_of(x) for x in t])
        if not carrying.any():
            continue
        q = df[[f'd{o}_x', f'd{o}_y', f'd{o}_z']].to_numpy(float)[m]
        h = np.linalg.norm(p[:, :2] - q[:, :2], axis=1)
        h3 = np.linalg.norm(p - q, axis=1)
        better = carrying & (h < near_h)
        who[better] = o
        near_h = np.where(better, h, near_h)
        near_3 = np.where(carrying & (h3 < near_3), h3, near_3)
    k = int(np.nanargmin(near_h))
    return {'drone': d, 'from': a, 'to': b, 'box_max_m': float(np.nanmax(box)), 't_box': float(t[int(np.nanargmax(box))]),
            'ring_min_h_m': float(np.nanmin(rh)), 'ring_min_3d_m': float(np.nanmin(r3)),
            'carrier_min_h_m': float(near_h[k]), 'carrier_min_3d_m': float(np.nanmin(near_3)), 'carrier_who': int(who[k]),
            't_carrier': float(t[k]), 'z_range_m': [float(np.nanmin(p[:, 2])), float(np.nanmax(p[:, 2]))]}


def landing(run):
    """Every quad down: /fleet/landed came, and on the run's last row (2 s after it) each drone is within 3 cm of its
    rest height and disarmed. Its height at the fleet's disarm edge is reported beside that."""
    t_landed = run.first('LANDED')
    out = {'t_land': run.t_land, 't_landed': t_landed, 'drones': [],
           'landed_anyway': len(re.findall(r'landed anyway', run.launch)), 'ring_z_last_m': float(run.pos[-1, 2])}
    df = run.df
    for i in range(4):
        z = df[f'd{i}_z'].to_numpy(float)
        rest = float(np.nanmedian(z[run.t < 2.0]))
        armed = df[f'd{i}_armed'].to_numpy(float) > 0.5
        edges = np.nonzero(armed[:-1] & ~armed[1:])[0]
        out['drones'].append({'drone': i, 'rest_m': rest, 'dz_last_m': float(z[-1] - rest),
                              'disarm_t': float(run.t[edges[-1]]) if edges.size else None,
                              'disarm_dz_m': float(z[edges[-1]] - rest) if edges.size else None,
                              'armed_last': float(df[f'd{i}_armed'].iloc[-1]),
                              'tilt_peak_deg': float(np.nanmax(df[f'd{i}_tilt_deg'])),
                              'thr_peak': float(np.nanmax(df[f'd{i}_thr']))})
    out['all_down'] = bool(t_landed is not None and not out['landed_anyway']
                           and all(abs(d['dz_last_m']) <= 0.03 and d['armed_last'] < 0.5 for d in out['drones']))
    return out


def throttle(run, spans):
    """Each stage's carriers: largest p95 throttle and the fraction of carrier samples at the 0.8 cap."""
    df = run.df
    rows = []
    for name, a, b, drones in spans:
        m = run.win(a, b)
        rows.append({'stage': name, 'from': a, 'to': b, 'p95': max(float(np.nanpercentile(df[f'd{i}_thr'][m], 95)) for i in drones),
                     'cap_frac': float(np.nanmean(df[[f'd{i}_thr' for i in drones]].to_numpy(float)[m] >= S.THROTTLE_MAX - 1e-3)),
                     'per_drone_cap_frac': {str(i): float(np.nanmean(df[f'd{i}_thr'][m] >= S.THROTTLE_MAX - 1e-3))
                                            for i in drones}})
    return rows


def t2_cap(run, w_t1, w_t2):
    """Drone 2 (plate 6, the largest thrust offset) on the throttle cap in the second T: when it stays there, the pack
    (the tracker's battery_v), and drone 0 beside it."""
    df = run.df
    thr = df['d2_thr'].to_numpy(float)
    at = thr >= S.THROTTLE_MAX - 1e-3
    m = run.win(w_t1, w_t2 + 5.0)
    idx = np.nonzero(m & at)[0]
    # the last unbroken stretch at the cap, gaps up to 1 s bridged
    start = stop = None
    if idx.size:
        groups = np.split(idx, np.nonzero(np.diff(run.t[idx]) > 1.0)[0] + 1)
        start, stop = float(run.t[groups[-1][0]]), float(run.t[groups[-1][-1]])
    volts = {}
    for d in (0, 2):
        tf = [p for p in run_logs.node_dirs(run.dir, 'tracker') if os.path.basename(p).startswith(f'drone{d}_')][0]
        tr = pd.read_csv(os.path.join(tf, 'log.csv'), usecols=['sim_time', 'battery_v', 'thr_off'])
        tq = tr.sim_time.to_numpy(float) - run.t_ready
        w5 = (tq >= w_t2 - 5.0) & (tq < w_t2)
        t_lap1 = run.laps()[0]
        wl = (tq >= t_lap1[1]) & (tq < t_lap1[2])
        volts[str(d)] = {'v_last5': float(tr.battery_v[w5].mean()), 'thr_off_last5': float(tr.thr_off[w5].mean()),
                         'v_lap1': float(tr.battery_v[wl].mean()), 'thr_off_lap1': float(tr.thr_off[wl].mean())}
    tr1 = (run.t >= t_lap1[1]) & (run.t < t_lap1[2])
    last5 = run.win(w_t2 - 5.0, w_t2)
    tl = run.win(w_t1, w_t2)
    hold_from = run.laps()[4][2]
    before = np.nonzero((run.t < hold_from) & np.any(df[[f'd{i}_thr' for i in range(4)]].to_numpy(float) >= S.THROTTLE_MAX - 1e-3, axis=1))[0]
    return {'cap_samples_any_carrier_before_this_hold': int(before.size), 'cap_from': start, 'cap_to': stop, 'cap_from_before_weld_s': (w_t2 - start) if start else None,
            'cap_to_after_weld_s': (stop - w_t2) if stop else None,
            'cap_frac_t2': float(np.mean(at[tl])), 'd0_thr_last5': float(np.nanmean(df['d0_thr'][last5])),
            'd2_thr_last5': float(np.nanmean(df['d2_thr'][last5])), 'd2_thr_lap1': float(np.nanmean(df['d2_thr'][tr1])),
            'z_err_last5_mm': 1e3 * float(np.nanmean(run.pos[last5, 2] - run.ref[last5, 2])),
            'volts': volts}


def path_vs_lap1(run, laps):
    """How far each lap's reference, and the ring, sit from lap 1's reference: the fold-in re-zeroes the path's datum on
    the measured ring (RESIZE_INPUTS hover_xy), and the clock-free error is against the reference of the same lap, so it
    cannot see this. Same trajectory-clock phase for the reference's shift; the ring's nearest distance to lap 1's
    reference path in the plane for its distance."""
    pt, tt = run.plan.t_run.to_numpy(float), run.plan.traj_t.to_numpy(float)
    phase = np.arange(0.5, 34.0, 0.1)

    def at(lap):
        m = (tt > (lap - 1) * LAP_S + TT_EPS) & (tt < lap * LAP_S - TT_EPS)
        return np.interp(phase + (lap - 1) * LAP_S, tt[m], pt[m])

    k1 = at(1)
    ref1 = np.stack([np.interp(k1, run.t, run.ref[:, i]) for i in range(2)], 1)
    # lap 1's reference as a dense polyline
    a, b = laps[0][1], laps[0][2]
    m1 = run.win(a, b)
    poly = run.ref[m1, :2]
    keep = np.r_[True, np.linalg.norm(np.diff(poly, axis=0), axis=1) > 1e-9]
    poly = poly[keep]
    out = []
    for k, a, b in laps:
        kq = at(k)
        ref = np.stack([np.interp(kq, run.t, run.ref[:, i]) for i in range(2)], 1)
        shift = ref - ref1
        m = run.win(a, b)
        pos = run.pos[m, :2]
        sel = pos[::5]
        d = np.min(np.linalg.norm(sel[:, None, :] - poly[None, :, :], axis=2), axis=1)
        out.append({'lap': k, 'ref_shift_x_mm': 1e3 * float(np.mean(shift[:, 0])), 'ref_shift_y_mm': 1e3 * float(np.mean(shift[:, 1])),
                    'ref_shift_mm': 1e3 * float(np.linalg.norm(np.mean(shift, axis=0))),
                    'ring_to_lap1_ref_rms_mm': 1e3 * float(np.sqrt(np.mean(d ** 2))), 'ring_to_lap1_ref_max_mm': 1e3 * float(np.max(d))})
    return out


def score_chain(rid, base):
    run = Run(rid)
    sm = S.score(run.dir)
    variant = variant_of(run.dir)
    v = validity(run, sm)
    laps = run.laps()
    lt = run.lap_table()
    ch = sm['events']
    if [e['drone'] for e in ch] != [1, 3, 3, 1]:
        raise ValueError(f"{rid}: chain events on drones {[e['drone'] for e in ch]}, not the card's 1, 3, 3, 1")
    first, rel3, att3, att1 = ch
    t1, t2, t3, t4 = first['t'], rel3['t'], att3['t'], att1['t']
    tl = run.t_land
    out = {'run': run.rid, 'variant': variant, 'dir': os.path.relpath(run.dir, REPO), 'validity': v, 'laps': lt,
           'lap_lines': re.findall(r'lap (\d) done at t=([\d.]+) s', run.launch),
           'events_csv': {k: run.first(k) for k in ('ARM', 'TAKEOFF', 'LIFTED', 'PATH_START', 'LAND', 'LANDED')},
           's_metrics_stages': sm['stages'], 'lift': sm['lift'], 'dropped': sm['dropped'], 's_metrics_events': ch}
    # lap 1, the validity check: P's form against the base's lap 1
    b1 = base['lap1']
    l1 = lt[0]
    out['lap1_check'] = {'path_mm': l1['path_mm'], 'path_bar_mm': b1['path_bar_mm'], 'tilt_deg': l1['tilt_peak_deg'],
                         'tilt_bar_deg': b1['tilt_bar_deg'],
                         'holds': bool(l1['path_mm'] <= b1['path_bar_mm'] and l1['tilt_peak_deg'] <= b1['tilt_bar_deg'])}
    if not out['lap1_check']['holds']:
        v['void_reasons'].append("lap 1 outside P's form of the base (look into it before scoring)")
    # step 1: square -> T
    joint1 = run.first('JOINT_DETACHED', 1, t1)
    resize1 = run.first('DETACH_RESIZE', 1, t1)
    e1 = event_row(run, t1, t2)
    t6_e = t6(run, t1, t2)
    unan1 = re.findall(r'UNANNOUNCED DETACH: drone \d+ \(/drone_1\)[^\n]*', run.launch)
    out['step1'] = {'kind': first['event'], 't': t1, 't_joint': joint1, 'joint_after_event_s': joint1 - t1,
                    't_resize_event': resize1, 'resize_event_s': resize1 - t1, 'planner_n_after': first['n_after'],
                    'planner_resize_s': first['resize_s'], **e1, 's_peak_deg': first['peak_deg'],
                    's_abs_peak_deg': first['abs_peak_deg'], 'settled_deg': first['settled_deg'],
                    'time_over_15_s': tilt_over(run, t1, t2, 15.0), 'time_over_30_s': tilt_over(run, t1, t2, 30.0),
                    'time_over_45_s': tilt_over(run, t1, t2, 45.0), 't6': t6_e,
                    't6_from_joint_s': (t6_e['t_back_s'] - (joint1 - t1)) if t6_e['t_back_s'] is not None else None,
                    'unannounced_lines': unan1,
                    't_first_over_s': {str(dg): first_over(run, t1, t2, dg) for dg in (15, 30, 45)},
                    'path_10s_mm': path_10s(run, t1, t2),
                    'tilt_after_6s_deg': float(np.nanmax(run.mag[run.win(joint1 + 6.0, t2)])),
                    'tilt_at_resize_deg': float(np.interp(t1 + first['resize_s'], run.t, run.mag)),
                    'tilt_after_deg': {str(dt): float(np.interp(joint1 + dt, run.t, run.mag)) for dt in (1, 2, 3, 5, 10, 20)}}
    if variant == 'A':
        out['step1']['unload_lines'] = re.findall(r'UNLOAD(?:ED)? drone[^\n]*', run.launch)
        out['step1']['unload_cancelled'] = len(re.findall(r'UNLOAD CANCELLED', run.launch))
        s = out['step1']
        s['pass'] = bool(s['planner_resize_s'] <= A1_N3_S and s['planner_n_after'] == 3 and not s['unload_cancelled']
                         and len(s['unload_lines']) >= 2 and s['rise_deg'] <= A1_PEAK and not sm['dropped'] and not v['fleet_abort'])
    else:
        x1 = X.score(run.dir)
        s = out['step1']
        s['x1_metrics'] = {k: x1.get(k) for k in ('t_release', 't_land', 'tilt_rise', 't_peak', 'abs_peak', 'sigma_pre',
                                                  'settled_tilt', 't_back_t6', 't6_last_exit', 't_level', 'level_band_deg',
                                                  'dip_mm', 'path_err_mm', 'path_kind', 'resize_s', 'n_after', 'detect_s',
                                                  'unannounced_lines', 'dropped', 'aborted', 'loss', 't_over80_s', 'rtf')}
        s['pass_t6'] = bool(t6_e['t_back_s'] is not None and t6_e['t_back_s'] <= B1_T6)
        s['pass_peak'] = bool(s['abs_peak_deg'] <= B1_PEAK)
        s['pass_loss'] = bool(not sm['dropped'] and not v['fleet_abort'])
        s['pass_after_6s'] = bool(s['tilt_after_6s_deg'] <= B1_AFTER6_DEG)
        s['pass_original_t6'] = s['pass_t6'] and s['pass_peak'] and s['pass_loss']
        s['pass'] = s['pass_after_6s'] and s['pass_peak'] and s['pass_loss']
    # step 2: T -> pair
    e2 = event_row(run, t2, t3)
    lean_w = (t3 - 10.0, t3)
    un3 = re.findall(r'UNANNOUNCED DETACH: drone \d+ \(/drone_3\)[^\n]*', run.launch)
    rp = roll_ppm(run, *lean_w)
    rp_all = roll_ppm(run, t2 + 5.0, t3)
    lean = float(np.nanmean(run.mag[run.win(*lean_w)]))
    th = run.th[run.win(*lean_w)]
    out['step2'] = {'t': t2, 't_joint': run.first('JOINT_DETACHED', 3, t2), 'planner_resize_s': rel3['resize_s'],
                    'planner_n_after': rel3['n_after'], 'resize_event_s': run.first('DETACH_RESIZE', 3, t2) - t2, **e2,
                    's_peak_deg': rel3['peak_deg'], 's_abs_peak_deg': rel3['abs_peak_deg'],
                    'settled_deg': rel3['settled_deg'], 'unannounced_lines': un3,
                    'detected_by': ('marker' if un3 and 'magnet' in un3[0] else 'stretch' if un3 else None),
                    'lean_deg': lean, 'lean_of_mean_deg': float(np.linalg.norm(np.nanmean(th, axis=0))),
                    'roll_last10': rp, 'roll_pair_stage': rp_all,
                    'L_peak_holds': bool(rel3['peak_deg'] <= L_PEAK), 'L_settled_holds': bool(rel3['settled_deg'] <= L_SETTLED),
                    't6': t6(run, t2, t3)}
    s = out['step2']
    s['pass'] = bool(len(un3) >= 1 and s['planner_resize_s'] <= A2_N2_S and s['planner_n_after'] == 2
                     and s['rise_deg'] < A2_PEAK and s['abs_peak_deg'] < A2_PEAK and lean < A2_LEAN and not sm['dropped'])
    # steps 3, 4: the rejoins
    out['step3'] = rejoin(run, 3, t3, t4, att3)
    out['step4'] = rejoin(run, 1, t4, tl, att1)
    lean_land = float(np.nanmean(run.mag[run.win(tl - 10.0, tl)]))
    out['step4']['lean_before_land_deg'] = lean_land
    out['step4']['pass_lean'] = bool(lean_land <= A4_LEAN)
    for key in ('step3', 'step4'):
        s = out[key]
        s['refused_or_abort'] = v['rejoin_refused'] + v['rejoin_abort']
        s['pass'] = bool(s['pass_a'] and s['pass_b'] and s['pass_tilt'] and s['pass_rod'] and s['pass_share']
                         and not s['refused_or_abort'] and s.get('pass_lean', True))
    # step 5: lap 6, P's form
    l6 = lt[5]
    b6 = base['lap6']
    out['step5'] = {'path_mm': l6['path_mm'], 'path_bar_mm': b6['path_bar_mm'], 'tilt_deg': l6['tilt_peak_deg'],
                    'tilt_bar_deg': b6['tilt_bar_deg'], 'from': l6['from'], 'to': l6['to'],
                    'pass': bool(l6['path_mm'] <= b6['path_bar_mm'] and l6['tilt_peak_deg'] <= b6['tilt_bar_deg'])}
    # step 6: the whole run
    land = landing(run)
    j1, j3, st1, st3 = run.first('JOINT_DETACHED', 1), run.first('JOINT_DETACHED', 3), out['step4']['t_start'], out['step3']['t_start']
    w3 = out['step3']['t_weld']

    def carriers_when_1(t):
        return [0, 2] + ([3] if (t < j3 or t >= w3) else [])

    stg = [dist_staged(run, 1, j1, st1, carriers_when_1), dist_staged(run, 3, j3, st3, lambda t: [0, 2])]
    # after the release's own kick the staged drones are clear; the laps alone, as E4 reads "through the lap"
    ap = [dist_staged(run, 3, st3, w3, lambda t: [0, 2]),
          dist_staged(run, 1, st1, out['step4']['t_weld'], lambda t: [0, 2, 3])]
    d13 = run.win(j3, st3)
    p1 = run.df[['d1_x', 'd1_y', 'd1_z']].to_numpy(float)[d13]
    p3 = run.df[['d3_x', 'd3_y', 'd3_z']].to_numpy(float)[d13]
    out['step6'] = {'dropped': sm['dropped'], 'fleet_abort': v['fleet_abort'], 'envelope_faults': v['envelope_faults'],
                    'landing': land, 'staged': stg, 'approach': ap,
                    'staged_to_staged_min_m': float(np.nanmin(np.linalg.norm(p1 - p3, axis=1))),
                    'pass_box': all(x['box_max_m'] <= E4_BOX for x in stg),
                    'pass_ring': all(x['ring_min_h_m'] >= E4_RING for x in stg),
                    'pass_carrier': all(x['carrier_min_h_m'] >= E4_CARRIER for x in stg)}
    s = out['step6']
    s['pass'] = bool(not s['dropped'] and not s['fleet_abort'] and not s['envelope_faults'] and land['all_down']
                     and s['pass_box'] and s['pass_ring'] and s['pass_carrier'])
    # the T stages and the pair: signed tilt, offset from the point
    w_t1, w_t2 = out['step3']['t_weld'], out['step4']['t_weld']
    b_lap2 = laps[1][2]
    pre_hold_end = laps[2][1]
    stages = {
        'T1_path_last5': signed(run, t2 - 5.0, t2),
        'T1_moving': signed(run, joint1 + 5.0, t2, cut=(b_lap2, pre_hold_end)),
        'T1_lap2_hold': signed(run, b_lap2 + 1.0, pre_hold_end),
        'pair_hold_before_weld': signed(run, w_t1 - 5.0, w_t1),
        'T2_blend_last2': signed(run, out['step3']['t_blend_over'] - 2.0, out['step3']['t_blend_over']),
        'T2_lap5': signed(run, laps[4][1], laps[4][2]),
        'T2_hold_before_weld': signed(run, w_t2 - 5.0, w_t2),
        'T2_lap5_end_hold_first5': signed(run, laps[4][2], laps[4][2] + 5.0),
        'square_blend_last2': signed(run, out['step4']['t_blend_over'] - 2.0, out['step4']['t_blend_over']),
        'square_lap6': signed(run, laps[5][1], laps[5][2]),
        'square_lap1': signed(run, laps[0][1], laps[0][2]),
    }
    out['stages_signed'] = stages
    out['path_vs_lap1'] = path_vs_lap1(run, laps)
    out['throttle'] = throttle(run, [('square', run.first('LIFTED'), t1, [0, 1, 2, 3]),
                                     ('T1', joint1 + 2.0, t2, [0, 2, 3]),
                                     ('pair', t2 + 2.0, w_t1, [0, 2]),
                                     ('T2', w_t1, w_t2, [0, 2, 3]),
                                     ('square2', w_t2, tl, [0, 1, 2, 3])])
    out['t2_cap'] = t2_cap(run, w_t1, w_t2)
    out['rejoiners_throttle'] = {str(d): float(np.nanmax(run.df[f'd{d}_thr'][run.win(a, b)]))
                                 for d, a, b in ((3, out['step3']['t_start'], w_t1), (1, out['step4']['t_start'], w_t2))}
    # the chain's verdict items
    out['verdicts'] = {'step1': out['step1']['pass'], 'step2': out['step2']['pass'], 'step3': out['step3']['pass'],
                       'step4': out['step4']['pass'], 'step5': out['step5']['pass'], 'step6': out['step6']['pass']}
    out['valid'] = not v['void_reasons']
    return out


def score_base(rid):
    run = Run(rid)
    sm = S.score(run.dir)
    lt = run.lap_table()
    v = validity(run, sm)
    laps = run.laps()
    land = landing(run)
    tl = run.t_land
    return {'run': run.rid, 'variant': 'base', 'dir': os.path.relpath(run.dir, REPO), 'validity': v, 'laps': lt,
            'dropped': sm['dropped'], 'landing': land, 'lift': sm['lift'],
            'lean_before_land_deg': float(np.nanmean(run.mag[run.win(tl - 10.0, tl)])),
            'stages_signed': {'square_lap1': signed(run, laps[0][1], laps[0][2]), 'square_lap6': signed(run, laps[5][1], laps[5][2])},
            'path_vs_lap1': path_vs_lap1(run, laps),
            'throttle': throttle(run, [('square', run.first('LIFTED'), tl, [0, 1, 2, 3])]),
            'valid': not v['void_reasons'],
            'n2_bands_whole_path': dict(zip(('path_m', 'tilt_deg'), n2_bands.path_metrics_clock_free(run.dir))),
            'tilt_rms_by_lap': [x['tilt_rms_deg'] for x in lt]}


def p_form(paths, tilts):
    m, t = float(np.mean(paths)), float(np.mean(tilts))
    return {'path_mean_mm': m, 'path_bar_mm': max(1.25 * m, m + 3.0), 'tilt_mean_deg': t, 'tilt_bar_deg': t + 1.0,
            'paths_mm': list(paths), 'tilts_deg': list(tilts)}


def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [strip(v) for v in o]
    if isinstance(o, (float, np.floating)):
        return None if not np.isfinite(o) else round(float(o), 5)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o




def bar_table(doc):
    """The card's 'Every bar' table, one column per chain run."""
    runs = list(doc['runs'].values())
    yes = lambda v: 'holds' if v else 'MISS'  # noqa: E731
    f = lambda v, nd=2: '-' if v is None else f'{v:.{nd}f}'  # noqa: E731
    cols = [r['run'] + ' (' + r['variant'] + ')' for r in runs]
    lines = ['| Bar | Frozen | ' + ' | '.join(cols) + ' |', '|' + '---|' * (len(cols) + 2)]

    def row(name, bar, fn, only=None):
        lines.append(f'| {name} | {bar} | ' + ' | '.join(fn(r) if only in (None, r['variant']) else '-' for r in runs)
                     + ' |')
    row('valid (void reasons)', 'none', lambda r: 'yes' if r['valid'] else '; '.join(r['validity']['void_reasons']))
    row("lap 1 within P's form of the base's lap 1", 'yes', lambda r: yes(r['lap1_check']['holds']))
    row('A1 planner at n = 3 after the DETACH (s)', '<= 6.0', lambda r: f(r['step1']['planner_resize_s']), 'A')
    row('A1 UNLOAD seen, unloads cancelled', 'yes, 0', lambda r: f"{'yes' if len(r['step1']['unload_lines']) >= 2 else 'NO'}, "
        f"{r['step1']['unload_cancelled']}", 'A')
    row('A1 tilt rise from the 5 s mean (deg)', '<= 30', lambda r: f(r['step1']['rise_deg']), 'A')
    row('A1 abs peak (deg, reported)', '-', lambda r: f(r['step1']['abs_peak_deg']), 'A')
    row('B1 loss: drop, abort', 'none', lambda r: 'none' if r['step1']['pass_loss'] else 'LOSS', 'B')
    row('B1 abs peak (deg)', '<= 80', lambda r: f(r['step1']['abs_peak_deg']), 'B')
    row('B1 largest tilt from +6 s to the next event (deg; the later runs\' bar)', '<= 5',
        lambda r: f(r['step1']['tilt_after_6s_deg']), 'B')
    row('B1 T6, the T stage\'s steady tilt (s; the original bar, recorded)', '<= 5.0',
        lambda r: f(r['step1']['t6']['t_back_s']), 'B')
    row('step 1 (A1 / B1)', '', lambda r: yes(r['verdicts']['step1']))
    row('2 planner at n = 2 after the RELEASE (s)', '<= 1.0', lambda r: f(r['step2']['planner_resize_s']))
    row('2 tilt rise from the 5 s mean (deg)', '< 15', lambda r: f(r['step2']['rise_deg']))
    row('2 abs tilt peak (deg)', '< 15', lambda r: f(r['step2']['abs_peak_deg']))
    row("2 pair's lean, last 10 s (deg)", '< 5', lambda r: f(r['step2']['lean_deg']))
    row('step 2', '', lambda r: yes(r['verdicts']['step2']))
    for k, name in (('step3', '3'), ('step4', '4')):
        row(f'{name} (a) on plate after START (s)', '<= 30', lambda r, k=k: f(r[k]['plate_s']))
        row(f'{name} (b) joint after on plate (s)', '<= 0.5', lambda r, k=k: f(r[k]['joint_s'], 3))
        row(f'{name} (c) tilt, weld to +15 s (deg)', '<= 5', lambda r, k=k: f(r[k]['fold_peak_deg']))
        row(f'{name} (c) rod end through the blend (cm)', '<= 2', lambda r, k=k: f(r[k]['rod_end_cm'], 1))
        row(f"{name} (c) tension at the blend's end against the share (N)", '+- 10 %',
            lambda r, k=k: f"{f(r[k]['share']['t_end_N'], 3)} / {f(r[k]['share']['share_N'], 3)}")
        row(f'{name} REJOIN_REFUSED or REJOIN_ABORT', 'none', lambda r, k=k: str(r[k]['refused_or_abort'] or 'none'))
    row("4 square's lean, last 10 s before LAND (deg)", '<= 2', lambda r: f(r['step4']['lean_before_land_deg']))
    row('steps 3, 4', '', lambda r: f"{yes(r['verdicts']['step3'])}, {yes(r['verdicts']['step4'])}")
    row('5 lap 6 path error (mm)', f"<= {doc['bars']['lap6']['path_bar_mm']:.2f}", lambda r: f(r['step5']['path_mm']))
    row('5 lap 6 tilt peak (deg)', f"<= {doc['bars']['lap6']['tilt_bar_deg']:.2f}", lambda r: f(r['step5']['tilt_deg']))
    row("5 ring to lap 1's reference, lap 6 RMS (mm, reported)", '-',
        lambda r: f(r['path_vs_lap1'][5]['ring_to_lap1_ref_rms_mm'], 1))
    row('6 drop, abort, envelope fault', 'none', lambda r: 'none' if not (r['step6']['dropped'] or r['step6']['fleet_abort']
                                                                          or r['step6']['envelope_faults']) else 'YES')
    row('6 every quad lands', 'all', lambda r: 'all' if r['step6']['landing']['all_down'] else 'NOT ALL')
    row('6 staged, worst of x and y (m)', '<= 1.6', lambda r: f(max(x['box_max_m'] for x in r['step6']['staged'])))
    row('6 staged to the ring centre, least (m)', '>= 0.6', lambda r: f(min(x['ring_min_h_m'] for x in r['step6']['staged'])))
    row('6 staged to a carrier, least (m)', '>= 0.8', lambda r: f(min(x['carrier_min_h_m'] for x in r['step6']['staged'])))
    row('steps 5, 6', '', lambda r: f"{yes(r['verdicts']['step5'])}, {yes(r['verdicts']['step6'])}")
    out = '\n'.join(lines)
    for v in ('A', 'B'):
        rs = [r for r in runs if r['variant'] == v]
        if rs:
            ok = [r for r in rs if r['valid']]
            allp = ok and all(all(r['verdicts'].values()) for r in ok)
            out += (f"\n{v}: {len(ok)} valid of {len(rs)}; " + ('CONFIRMED (every step in every valid repeat)' if allp and
                    len(ok) >= 2 else 'every step holds, fewer than 2 valid repeats' if allp else
                    'FALSIFIED or missed: ' + '; '.join(f"{r['run']} {[k for k, x in r['verdicts'].items() if not x]}"
                                                       for r in ok if not all(r['verdicts'].values()))))
    for r in runs:
        if r['variant'] == 'B' and r['step1']['pass_original_t6'] is False:
            out += f"\n{r['run']}: the original T6 bar is missed ({f(r['step1']['t6']['t_back_s'])} s), recorded as missed"
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help="R#### or run directories: the base's and the chains'")
    ap.add_argument('--json', help='write the whole score here')
    a = ap.parse_args()
    dirs = [S.locate(r) for r in a.runs]
    base_ids = [d for d in dirs if variant_of(d) == 'base']
    chains = [d for d in dirs if variant_of(d) != 'base']
    if not base_ids:
        sys.exit("give the base's runs (x2_full_fig8_base): A5 and lap 1's check read their laps")
    base_rows = {os.path.basename(d).split('_')[0]: score_base(d) for d in base_ids}
    keys = list(base_rows)
    lap = lambda k: p_form([base_rows[r]['laps'][k]['path_mm'] for r in keys],  # noqa: E731
                           [base_rows[r]['laps'][k]['tilt_peak_deg'] for r in keys])
    bars = {'lap1': lap(0), 'lap6': lap(5), 'base_lap_means': [{'lap': k + 1, **lap(k)} for k in range(6)]}
    runs = {os.path.basename(d).split('_')[0]: score_chain(d, bars) for d in chains}
    doc = {'card': 'docs/experiments/2026-10-30_x2_full_chain_fig8.md',
           'scored': 'tools/x2_metrics.py; bars as frozen 30 Oct ~15:46, B1 as frozen 31 Oct for the later runs',
           'about': ('Chain runs under runs, base under base. Windows on run.csv clock. T6 in step1.t6 is release_bands '
                     'windowed to the next chain event (the T stage\'s own steady tilt); step1.x1_metrics is tools/x1_metrics.py '
                     'as coded (window to LAND, so its post tilt is the square on lap 6). Lean = mean tilt magnitude. '
                     'Roll in stages_signed: + = plate 3 up; off_y_mm: < 0 = towards plate 3.'),
           'bars': bars, 'base': base_rows, 'runs': runs}
    for r in base_rows.values():
        lt = r['laps']
        print(f"base {r['run']}: {'valid' if r['valid'] else 'VOID ' + '; '.join(r['validity']['void_reasons'])}; lap 1 "
              f"{lt[0]['path_mm']:.2f} mm, {lt[0]['tilt_peak_deg']:.2f} deg; lap 6 {lt[5]['path_mm']:.2f} mm, "
              f"{lt[5]['tilt_peak_deg']:.2f} deg; laps {min(x['path_mm'] for x in lt):.2f}-{max(x['path_mm'] for x in lt):.2f}"
              f" mm; dropped {r['dropped']}")
    print(f"P's form from the base: lap 1 <= {bars['lap1']['path_bar_mm']:.2f} mm, <= {bars['lap1']['tilt_bar_deg']:.2f} "
          f"deg; lap 6 <= {bars['lap6']['path_bar_mm']:.2f} mm, <= {bars['lap6']['tilt_bar_deg']:.2f} deg")
    if runs:
        print(bar_table(doc))
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as fh:
            json.dump(strip(doc), fh, indent=1)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
