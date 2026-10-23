#!/usr/bin/env python3
"""tools/catch_offset.py -- thesis sim result U (card docs/experiments/2026-10-23_catch_offset.md): how far off its
slot the magnet can catch before the task fails. Our own approach at 1 m in the twin; the fold-in at the measured catch
point (as flown, the rig's) or at the slot (the slot assumed).

    python3 tools/catch_offset.py R2501 [R2502 ...]                  # one line per run
    python3 tools/catch_offset.py --row R2501 [...]                  # one LF registry row per run
    python3 tools/catch_offset.py --table R2501-R2552 [--json out]   # arm x direction x d, and d_tol per arm/direction

Read-only on the runs. Per run (runner time; the newcomer is d<num_drones>):
  completion   the first failure (the card's failures 1-6), in this order: no weld (WELD_TIMEOUT, or none within 60 s
               of MAGNET ON); no fold-in within 2 s of the weld; a runner failure (manifest); a 'DROP AND LAND' planner
               line; the planner's 'load trajectory complete' missing or logged after its LAND; not LANDED
  separation   the newcomer's body within 0.30 m of a carrier's from the weld to LAND (Wesley 22 Oct: a task failure),
               scored apart as d_sep (critic B1): the two arms put the newcomer in different places by construction
  catch        the tip in the ring frame over [weld + 0.5, weld + 5] s (the weld is rigid): d_true its distance from the
               slot, d_tan the chord along the rim (cw +), d_rad (out +), dz against weld_point_z; aim_err its distance
               from the aim the knob typed (a plumbing check only: the trigger is a sphere around the aim)
  model        e_model: the planner's attach point for the newcomer (RESIZE_INPUTS rho) against the true one, xy, 3-D
  fold-in      metrics.event_window_metrics over [weld - 1, weld + 15]: ring tilt peak, settle, carrier dip; the ring
               tilt mean over [weld - 6, weld - 1]
  steady       the 4-quad hover over [weld + 15, path - 0.5]: ring tilt mean and max, |p - p_ref| mean (3-D, xy, z)
  path         n2_bands.path_metrics_clock_free from path - 0.5 (clock-free RMS, tilt peak); lag (metrics.contour_lag)
  diagnostics  the newcomer-carrier separation minimum from the weld to LAND, the newcomer's throttle peak after the
               weld and its rod tension mean in the steady window
  VOID         a run with no failure whose weld came later than path - 15 s (the fold-in did not settle in the
               hover): re-fly once. A failure takes precedence (critic M1)
path = the PATH_START event (the planner's 'hover hold done'), else the first payload_ref xy step over 1 mm from
weld + 5 s.
d_tol(arm, direction) = the largest d where every d' <= d completed (failures 1-6) in both repeats with the fold-in tilt
peak within FOLD_TILT_BAR_DEG (VOIDs left out); d_tol_true the same, run by run, on d_true. d_sep = the largest d where
every d' <= d kept the separation in both repeats (a run with no weld ends it). The mechanism (critic M4), per metric
and direction: the slot-minus-measured difference of the cell means, less its d = 0 value, against the d = 0 repeat
spread (the larger range of the two arms), shown when it is over the spread at the two largest d both arms flew.
"""
import argparse
import csv
import glob
import io
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
import n2_bands  # noqa: E402

CARD = 'docs/experiments/2026-10-23_catch_offset.md'
SLOT = (0.0, 0.225)              # sim.yaml modes.attach: plate 9
WELD_WAIT_S = 60.0
FOLD_WAIT_S = 2.0
CATCH_WIN = (0.5, 5.0)
FOLD_WIN = (-1.0, 15.0)
PRE_WIN = (-6.0, -1.0)
STEADY_AFTER_S = 15.0
PATH_LEAD_S = 0.5
VOID_BEFORE_PATH_S = 15.0
PATH_STEP_M = 1e-3
SEPARATION_M = 0.30
FOLD_TILT_BAR_DEG = 5.0          # PROPOSED (card): the sim claim bar of 28 Sep
REPEATS = 2
MECHANISM = ('fold_tilt_peak_deg', 'steady_tilt_mean_deg')
GRID = ('t', 'ri')               # the swept directions (d_tol); 'ta' and 'ro' are neighbours, flown once
DIRECTIONS = {'d0': 'on the slot', 't': 'along the rim, clockwise', 'ta': 'along the rim, anticlockwise',
              'ri': 'radially in', 'ro': 'radially out', 'mixed': 'along and across'}
ARMS = {'measured': 'the measured catch point', 'slot': 'the slot assumed'}


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


def _json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def run_args(d):
    """The knobs a run flew: its run-dir params_file.yaml under the manifest's launch args."""
    import yaml
    out = {}
    try:
        with open(os.path.join(d, 'params_file.yaml')) as fh:
            out.update(yaml.safe_load(fh) or {})
    except OSError:
        pass
    out.update(_json(os.path.join(d, 'manifest.json')).get('launch_args') or {})
    return out


def _f(v, default=0.0):
    try:
        return float(v) if v not in (None, '') else default
    except (TypeError, ValueError):
        return default


def catch_aim_xy(x0, y0, dt, dr):
    """bringup.graph_attach.catch_aim_xy: the slot moved dt along the rim (a chord, cw +) and dr radially (out +)."""
    r0 = math.hypot(x0, y0)
    a = math.atan2(y0, x0) - 2.0 * math.asin(dt / (2.0 * r0))
    return (r0 + dr) * math.cos(a), (r0 + dr) * math.sin(a)


def rim_offsets(xy, slot):
    """(d_true, d_tan, d_rad) of a ring-frame point from the slot: distance; chord along the rim to the point's
    azimuth, clockwise +; radius minus the slot's."""
    x0, y0 = slot
    r0 = math.hypot(x0, y0)
    da = math.atan2(math.sin(math.atan2(y0, x0) - math.atan2(xy[1], xy[0])),
                    math.cos(math.atan2(y0, x0) - math.atan2(xy[1], xy[0])))
    return (float(math.hypot(xy[0] - x0, xy[1] - y0)), 2.0 * r0 * math.sin(0.5 * da),
            float(math.hypot(xy[0], xy[1]) - r0))


def arm_of(a):
    return str(a.get('attach_point_source', '') or 'measured').strip().lower()


def _cm(m):
    """m to cm at 1 mm: an int for the whole-cm cells, 11.6 for the wrong-plate chord."""
    v = round(100.0 * m, 1)
    return int(v) if v == int(v) else v


def direction_of(a):
    """(direction, nominal d in cm) of the knob."""
    dt, dr = _f(a.get('catch_offset_t')), _f(a.get('catch_offset_r'))
    if dt == 0.0 and dr == 0.0:
        return 'd0', 0
    if dr == 0.0:
        return ('t' if dt > 0 else 'ta'), _cm(abs(dt))
    if dt == 0.0:
        return ('ro' if dr > 0 else 'ri'), _cm(abs(dr))
    return 'mixed', _cm(math.hypot(dt, dr))


def ring_frame(run, cols):
    """R(q)^T (p - payload) per row, p the columns cols (x, y, z)."""
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                  np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                  np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)
    v = run[cols].to_numpy(float) - run[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
    return np.einsum('nji,nj->ni', R, v)


def events(d):
    with open(os.path.join(d, 'logs', 'events.csv')) as fh:
        return [(float(e['sim_time']), e['event'], e.get('arg') or '') for e in csv.DictReader(fh)]


def _first(ev, name, after=-math.inf, arg=None):
    return next((t for t, n, a in sorted(ev) if n == name and t >= after and (arg is None or a == arg)), None)


def planner_lines(d):
    path = os.path.join(d, 'logs', 'launch.log')
    if not os.path.exists(path):
        return None
    with open(path, errors='replace') as fh:
        return [ln for ln in fh if 'dissipative_planner' in ln]


def resize_rho(lines, drone):
    """The planner's attach point for the newcomer (the last rho of its first attach RESIZE_INPUTS), or None."""
    for ln in lines or ():
        if 'RESIZE_INPUTS ' not in ln:
            continue
        try:
            j = json.loads(ln.split('RESIZE_INPUTS ', 1)[1])
        except ValueError:
            continue
        if j.get('what') == 'attach' and int(j.get('drone', -1)) == drone and j.get('rho'):
            return np.asarray(j['rho'][-1], float)
    return None


def logged_point(lines, drone):
    for ln in lines or ():
        m = re.search(rf'\] ATTACH drone {drone} \(OCP resize\).*m, (slot assumed|measured catch point)', ln)
        if m:
            return 'slot' if m.group(1) == 'slot assumed' else 'measured'
    return None


def path_start(run, ev, t_weld):
    t = _first(ev, 'PATH_START')
    if t is not None:
        return t
    tt = run.t.to_numpy()
    des = run[['payload_ref_x', 'payload_ref_y']].to_numpy(float)
    step = np.r_[0.0, np.linalg.norm(np.diff(des, axis=0), axis=1)]
    i = np.where((step > PATH_STEP_M) & (tt >= t_weld + 5.0))[0]
    return float(tt[i[0]]) if i.size else None


def _rows(run, a, b):
    t = run.t.to_numpy()
    return (t >= a) & (t <= b)


def analyse(d):
    """U's numbers for one run (dict)."""
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = events(d)
    a = run_args(d)
    man = _json(os.path.join(d, 'manifest.json'))
    n = int(_f(a.get('num_drones'), 3))
    k = f'd{n}_'
    slot = (_f(a.get('attach_x_offset'), SLOT[0]), _f(a.get('attach_y_offset'), SLOT[1]))
    dt, dr = _f(a.get('catch_offset_t')), _f(a.get('catch_offset_r'))
    direction, d_cm = direction_of(a)
    name = os.path.basename(os.path.normpath(d))
    r = {'run': name.split('_')[0], 'dir': d, 'config': name.split('_sim_gz_')[-1], 'arm': arm_of(a),
         'direction': direction, 'd_cm': d_cm, 'dt': dt, 'dr': dr, 'slot': list(slot),
         'aim': list(catch_aim_xy(slot[0], slot[1], dt, dr)), 'failures': [], 'void': [], 'crowded': [],
         'notes': []}
    lines = planner_lines(d)
    t_mag = _first(ev, 'MAGNET', arg='ON')
    t_weld = _first(ev, 'WELD')
    t_land = _first(ev, 'LAND')
    t_land = t_land if t_land is not None else _first(ev, 'PLANNER_LAND')
    t_end = t_land if t_land is not None else float(run.t.iloc[-1])
    r.update(t_magnet=t_mag, t_weld=t_weld, t_land=t_land)
    if _first(ev, 'WELD_TIMEOUT') is not None or t_weld is None or (
            t_mag is not None and t_weld - t_mag > WELD_WAIT_S):
        r['failures'].append('no weld' + (f' within {WELD_WAIT_S:.0f} s of MAGNET ON' if t_weld is not None else ''))
    t_fold = _first(ev, 'FOLD_IN', after=t_weld - 0.5, arg=str(n)) if t_weld is not None else None
    r['t_fold'] = t_fold
    if t_weld is not None and (t_fold is None or t_fold - t_weld > FOLD_WAIT_S):
        r['failures'].append(f'no fold-in within {FOLD_WAIT_S:.0f} s of the weld')
    for f in man.get('failures') or ():
        r['failures'].append(f'runner: {f}')
    if lines is not None and any('DROP AND LAND' in ln for ln in lines):
        r['failures'].append('DROP AND LAND')
    t_path = path_start(run, ev, t_weld if t_weld is not None else (t_mag or float(run.t.iloc[0])))
    r['t_path'] = t_path
    if t_path is None or not completed(run, lines, t_path, t_end):
        r['failures'].append('figure-8 not completed before LAND')
    if _first(ev, 'LANDED') is None:
        r['failures'].append('not LANDED')
    if t_weld is None:
        return r
    w = _rows(run, t_weld, t_end)
    nb = run[[f'{k}x', f'{k}y', f'{k}z']].to_numpy(float)
    sep = [np.linalg.norm(nb - run[[f'd{i}_x', f'd{i}_y', f'd{i}_z']].to_numpy(float), axis=1) for i in range(n)]
    sep = np.min(np.vstack(sep), axis=0)[w]
    r['separation_min_m'] = float(np.nanmin(sep)) if np.any(np.isfinite(sep)) else math.nan
    if r['separation_min_m'] < SEPARATION_M:
        r['crowded'].append(f'newcomer {r["separation_min_m"]:.2f} m from a carrier (< {SEPARATION_M:.2f})')
    if t_path is not None and t_weld > t_path - VOID_BEFORE_PATH_S:
        r['void'].append(f'weld {t_path - t_weld:.1f} s before the path (< {VOID_BEFORE_PATH_S:.0f})')
    catch(r, run, a, t_weld, slot)
    r['rho_model'] = None
    rho = resize_rho(lines, n)
    if rho is not None and 'rho_true' in r:
        r['rho_model'] = rho.tolist()
        true = np.asarray(r['rho_true'], float)
        r['e_model_xy_m'] = float(np.linalg.norm(rho[:2] - true[:2]))
        r['e_model_m'] = float(np.linalg.norm(rho - true))
    r['logged_point'] = logged_point(lines, n)
    if r['logged_point'] and r['logged_point'] != r['arm']:
        r['notes'].append(f'the ATTACH line says {r["logged_point"]}, the args {r["arm"]}')
    fold(r, run, t_weld, n)
    r['thr_peak'] = float(np.nanmax(run[f'{k}thr'].to_numpy(float)[w])) if f'{k}thr' in run else math.nan
    if t_path is not None:
        steady(r, run, t_weld + STEADY_AFTER_S, t_path - PATH_LEAD_S, k)
        if t_path < t_end:
            path(r, d, run, t_path - PATH_LEAD_S)
    return r


def completed(run, lines, t_path, t_end):
    """The figure-8 ran to the planner's own descent before LAND: its log line ahead of the planner's LAND in its own
    log, else (no log) payload_ref_z falling 2 cm below its top after the path start, before LAND."""
    if lines is not None:
        done = next((i for i, ln in enumerate(lines) if 'load trajectory complete' in ln), None)
        land = next((i for i, ln in enumerate(lines) if "received: 'LAND'" in ln), None)
        return done is not None and (land is None or done < land)
    t = run.t.to_numpy()
    zr = run.payload_ref_z.to_numpy(float)
    after = t >= t_path
    if not np.any(after):
        return False
    down = after & (zr < np.nanmax(zr[after]) - 0.02)
    return bool(np.any(down)) and float(t[down][0]) <= t_end


def catch(r, run, a, t_weld, slot):
    w = _rows(run, t_weld + CATCH_WIN[0], t_weld + CATCH_WIN[1])
    tip = ring_frame(run[w], ['tip_x', 'tip_y', 'tip_z'])
    tip = tip[np.all(np.isfinite(tip), axis=1)]
    if not len(tip):
        r['notes'].append('no tip rows after the weld')
        return
    p = tip.mean(axis=0)
    r['rho_true'] = p.tolist()
    r['d_true_m'], r['d_tan_m'], r['d_rad_m'] = rim_offsets(p[:2], slot)
    r['dz_m'] = float(p[2] - _f(a.get('weld_point_z'), math.nan))
    r['aim_err_m'] = float(math.hypot(p[0] - r['aim'][0], p[1] - r['aim'][1]))


def fold(r, run, t_weld, n):
    data = {c: run[c].to_numpy(float) for c in run.columns if run[c].dtype.kind in 'fi'}
    t = run.t.to_numpy()
    o = M.event_window_metrics(t, data, t_weld + FOLD_WIN[0], t_weld + FOLD_WIN[1], exclude=(n,),
                               horizon_s=FOLD_WIN[1] - FOLD_WIN[0])
    r['fold_tilt_peak_deg'] = o.get('ring_tilt_peak_deg', math.nan)
    r['fold_settle_s'] = o.get('settle_s', math.nan)
    r['fold_settle_censored'] = bool(o.get('settle_censored', False))
    r['carrier_dip_m'] = o.get('carrier_dip_m', math.nan)
    pre = _rows(run, t_weld + PRE_WIN[0], t_weld + PRE_WIN[1])
    r['pre_tilt_mean_deg'] = float(np.nanmean(run.payload_tilt_deg.to_numpy(float)[pre])) if pre.any() else math.nan


def steady(r, run, a, b, k):
    w = _rows(run, a, b)
    r['steady_s'] = max(0.0, b - a)
    if not w.any():
        return
    tilt = run.payload_tilt_deg.to_numpy(float)[w]
    e = (run[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
         - run[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float))[w]
    r.update(steady_tilt_mean_deg=float(np.nanmean(tilt)), steady_tilt_max_deg=float(np.nanmax(tilt)),
             steady_err_m=float(np.nanmean(np.linalg.norm(e, axis=1))),
             steady_err_xy_m=float(np.nanmean(np.linalg.norm(e[:, :2], axis=1))),
             steady_err_z_m=float(np.nanmean(e[:, 2])))
    if f'{k}tension' in run:
        r['steady_tension_n'] = float(np.nanmean(run[f'{k}tension'].to_numpy(float)[w]))


def path(r, d, run, t_from):
    r['path_err_m'], r['path_tilt_peak_deg'] = n2_bands.path_metrics_clock_free(d, t_from=t_from)
    t = run.t.to_numpy()
    des = np.where((t >= t_from)[:, None], run[['payload_ref_x', 'payload_ref_y']].to_numpy(float), np.nan)
    o = M.contour_lag(t, run[['payload_x', 'payload_y']].to_numpy(float), des, M.sweep_window(t, des))
    r['lag_s'] = o.get('lag_s', math.nan) if o else math.nan


def verdict(r):
    """FAIL (failures 1-6) before VOID: a late weld that also failed is not re-flown into a pass (critic M1)."""
    if r['failures']:
        return 'FAIL'
    if r['void']:
        return 'VOID'
    return 'complete'


def within_bar(r):
    return verdict(r) == 'complete' and _fin(r.get('fold_tilt_peak_deg')) <= FOLD_TILT_BAR_DEG


def separated(r):
    """The separation kept from the weld to LAND (no weld: not measured, not kept)."""
    v = r.get('separation_min_m')
    return v is not None and bool(np.isfinite(v)) and v >= SEPARATION_M


def _fin(v):
    return float(v) if v is not None and np.isfinite(v) else math.inf


METRICS = (('d_true_m', 'true catch offset', 100.0, '{:.1f} cm'), ('d_tan_m', 'along the rim', 100.0, '{:+.1f} cm'),
           ('d_rad_m', 'radial', 100.0, '{:+.1f} cm'), ('aim_err_m', 'catch to aim (plumbing)', 100.0, '{:.1f} cm'),
           ('e_model_xy_m', 'attach-point model error xy', 100.0, '{:.1f} cm'),
           ('fold_tilt_peak_deg', 'fold-in tilt peak', 1.0, '{:.2f} deg'),
           ('fold_settle_s', 'fold-in settle', 1.0, '{:.1f} s'), ('carrier_dip_m', 'carrier dip', 1000.0, '{:.0f} mm'),
           ('steady_tilt_mean_deg', 'steady tilt mean', 1.0, '{:.2f} deg'),
           ('steady_err_m', 'steady position error', 1000.0, '{:.0f} mm'),
           ('path_err_m', 'clock-free path error', 1000.0, '{:.1f} mm'),
           ('path_tilt_peak_deg', 'path tilt peak', 1.0, '{:.2f} deg'), ('lag_s', 'lag', 1.0, '{:+.2f} s'),
           ('separation_min_m', 'newcomer-carrier min', 1.0, '{:.2f} m'))


def _fmt(r, key, scale, f):
    v = r.get(key)
    return f.format(scale * v) if v is not None and np.isfinite(v) else '-'


def line(r):
    return (f"{r['run']} {r['config']}: {ARMS.get(r['arm'], r['arm'])}, {r['d_cm']} cm {DIRECTIONS[r['direction']]}: "
            f"{verdict(r)}" + (f" ({'; '.join(r['failures'] + r['void'])})" if r['void'] or r['failures'] else '')
            + (f"; separation FAIL ({'; '.join(r['crowded'])})" if r.get('crowded') else '')
            + '\n  ' + ', '.join(f'{name} {_fmt(r, k, s, f)}' for k, name, s, f in METRICS))


def registry_row(r, exploratory=True):
    """run,date,config,card,variable,baseline,metric,value,repeat_of,verdict,reviewer,notes as one LF CSV line."""
    date = os.path.basename(os.path.dirname(os.path.abspath(r['dir'])))
    variable = f"catch offset {r['d_cm']} cm {DIRECTIONS[r['direction']]}; fold-in at {ARMS.get(r['arm'], r['arm'])}"
    v = verdict(r)
    text = ('VOID (' + '; '.join(r['void']) + ')' if v == 'VOID' else 'FAIL (' + '; '.join(r['failures']) + ')'
            if v == 'FAIL' else f"complete, fold-in peak {'within' if within_bar(r) else 'over'} "
            f'{FOLD_TILT_BAR_DEG:g} deg')
    if r.get('crowded'):
        text += '; separation FAIL (' + '; '.join(r['crowded']) + ')'
    buf = io.StringIO()
    csv.writer(buf, lineterminator='\n').writerow(
        [r['run'], date, r['config'], CARD, variable, 'u_meas_d00 (the measured arm on the slot)',
         ' / '.join(name for _, name, _, _ in METRICS), ' / '.join(_fmt(r, k, s, f) for k, _, s, f in METRICS), '',
         text + (' (exploratory)' if exploratory else ''), '', '; '.join(r['notes'])])
    return buf.getvalue()


def _limit(cells, count):
    """The largest d whose cell and every smaller one has REPEATS non-VOID runs, all counted by `count`."""
    tol = None
    for d, c in sorted(cells.items()):
        if c['n'] < REPEATS or c[count] < c['n']:
            break
        tol = d
    return tol


def _limit_true(rs):
    """d_tol run by run on d_true (cm; nominal d for a run with no catch): the largest d_true below which every run
    completed within the bar."""
    keyed = sorted((100.0 * r['d_true_m'] if _fin(r.get('d_true_m')) < math.inf else float(r['d_cm']), within_bar(r))
                   for r in rs if verdict(r) != 'VOID')
    tol = None
    for d, ok in keyed:
        if not ok:
            break
        tol = round(d, 1)
    return tol


def mechanism(cells):
    """Critic M4, per swept direction: for each MECHANISM metric the slot-minus-measured difference of the cell means
    at each d both arms flew, less its d = 0 value ('growth'), against the d = 0 spread (the larger of the two arms'
    repeat ranges); 'shown' when the growth is over the spread at the two largest d (None: too few cells). The
    direction's 'shown' is any metric's."""
    out = {}
    meas, slot = cells.get('measured', {}), cells.get('slot', {})
    for direction in GRID:
        res = {}
        for k in MECHANISM:
            def vals(by_dir, d):
                c = by_dir.get('d0', {}).get(0) if d == 0 else by_dir.get(direction, {}).get(d)
                return c[k]['values'] if c else []
            zero = [vals(meas, 0), vals(slot, 0)]
            if min(len(v) for v in zero) < REPEATS:
                res[k] = {'shown': None}
                continue
            spread = max(max(v) - min(v) for v in zero)
            ds = sorted(set(meas.get(direction, {})) & set(slot.get(direction, {})))
            delta = {0: float(np.mean(zero[1]) - np.mean(zero[0]))}
            for d in ds:
                vm, vs = vals(meas, d), vals(slot, d)
                if len(vm) >= REPEATS and len(vs) >= REPEATS:
                    delta[d] = float(np.mean(vs) - np.mean(vm))
            growth = {d: v - delta[0] for d, v in delta.items() if d != 0}
            top = sorted(growth)[-2:]
            res[k] = {'delta': delta, 'growth': growth, 'spread0': float(spread),
                      'shown': (all(growth[d] > spread for d in top) if len(top) == 2 else None)}
        shown = [v['shown'] for v in res.values()]
        res['shown'] = True if any(shown) else (None if None in shown else False)
        out[direction] = res
    return out


def table(rows):
    """{'cells': {arm: {direction: {d: cell}}}, 'd_tol' / 'd_sep' / 'd_task' / 'd_tol_true': {arm: {direction: d or
    None}}, 'mechanism': {direction: ...}}: the d = 0 cell counts for every direction; d_task = min(d_tol, d_sep)."""
    cells = {}
    for r in rows:
        cells.setdefault(r['arm'], {}).setdefault(r['direction'], {}).setdefault(r['d_cm'], []).append(r)
    out = {'cells': {}, 'd_tol': {}, 'd_sep': {}, 'd_task': {}, 'd_tol_true': {}}
    for arm, by_dir in cells.items():
        out['cells'][arm] = {}
        for key in ('d_tol', 'd_sep', 'd_task', 'd_tol_true'):
            out[key][arm] = {}
        zero = by_dir.get('d0', {}).get(0, [])
        for direction, by_d in by_dir.items():
            out['cells'][arm][direction] = {d: _cell(rs) for d, rs in sorted(by_d.items())}
        for direction in GRID:
            if direction not in by_dir:
                continue
            swept = {d: _cell(rs) for d, rs in {0: zero, **by_dir[direction]}.items()}
            tol, sep = _limit(swept, 'n_ok'), _limit(swept, 'n_sep')
            out['d_tol'][arm][direction], out['d_sep'][arm][direction] = tol, sep
            out['d_task'][arm][direction] = None if tol is None or sep is None else min(tol, sep)
            out['d_tol_true'][arm][direction] = _limit_true(zero + [r for rs in by_dir[direction].values() for r in rs])
    out['mechanism'] = mechanism(out['cells'])
    return out


def _cell(rs):
    ok = [r for r in rs if verdict(r) != 'VOID']
    c = {'n': len(ok), 'n_ok': sum(within_bar(r) for r in ok), 'n_sep': sum(separated(r) for r in ok),
         'void': len(rs) - len(ok), 'runs': [r['run'] for r in rs],
         'failures': sorted({f for r in ok for f in r['failures']}),
         'crowded': sorted({f for r in ok for f in r.get('crowded', ())})}
    for k, _, _, _ in METRICS:
        v = [r[k] for r in ok if r.get(k) is not None and np.isfinite(r[k])]
        c[k] = {'mean': float(np.mean(v)) if v else math.nan, 'values': v}
    return c


def markdown(t):
    out = ['| arm | direction | d (cm) | n | complete within bar | separation kept | true offset (cm) | '
           'fold-in peak (deg) | path error (mm) | steady tilt (deg) | model error xy (cm) | failures |',
           '|' + '---|' * 12]
    for arm, by_dir in t['cells'].items():
        for direction, by_d in by_dir.items():
            for d, c in by_d.items():
                def m(k, s=1.0, f='{:.1f}'):
                    v = c[k]['values']
                    return (f.format(s * c[k]['mean']) + ' [' + ', '.join(f.format(s * x) for x in v) + ']'
                            if v else '-')
                out.append(f"| {ARMS.get(arm, arm)} | {DIRECTIONS[direction]} | {d} | {c['n']} | {c['n_ok']} | "
                           f"{c['n_sep']} | {m('d_true_m', 100.0)} | {m('fold_tilt_peak_deg', 1.0, '{:.2f}')} | "
                           f"{m('path_err_m', 1000.0)} | {m('steady_tilt_mean_deg', 1.0, '{:.2f}')} | "
                           f"{m('e_model_xy_m', 100.0)} | {'; '.join(c['failures'] + c['crowded']) or '-'} |")
    out.append('')
    rows = (('d_tol', 'd_tol (failures 1-6 and the tilt bar)'), ('d_tol_true', 'd_tol on d_true, run by run'),
            ('d_sep', 'd_sep (the 0.30 m separation)'), ('d_task', 'the task limit min(d_tol, d_sep)'))
    for key, label in rows:
        for arm, by_dir in ((a, v) for a, v in t[key].items() if v):
            out.append(f"{label}, {ARMS.get(arm, arm)}: " + ', '.join(
                f"{DIRECTIONS[k]} {'none' if v is None else f'{v} cm'}" for k, v in by_dir.items()))
    for direction, res in t.get('mechanism', {}).items():
        parts = []
        for k in MECHANISM:
            v = res.get(k, {})
            if v.get('shown') is None:
                parts.append(f'{k} too few cells')
                continue
            parts.append(f"{k} growth " + ', '.join(f'{d}: {g:+.2f}' for d, g in v['growth'].items())
                         + f" against the d = 0 spread {v['spread0']:.2f}: {'shown' if v['shown'] else 'not shown'}")
        verdict_text = {True: 'shown', False: 'not shown: the model point does not matter to the largest d flown',
                        None: 'too few cells'}[res['shown']]
        out.append(f'mechanism, {DIRECTIONS[direction]}: {verdict_text} ({"; ".join(parts)})')
    return '\n'.join(out)


def _clean(o):
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, float) and not np.isfinite(o):
        return None
    if isinstance(o, np.generic):
        return _clean(o.item())
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--row', action='store_true', help='one LF registry row per run')
    ap.add_argument('--claim', action='store_true', help='rows for the claim grid (not marked exploratory)')
    ap.add_argument('--table', action='store_true')
    ap.add_argument('--json', help='write the runs and the table here (the figure reads it)')
    o = ap.parse_args(argv)
    rows = [analyse(d) for d in run_dirs(o.runs)]
    if o.row:
        for r in rows:
            sys.stdout.write(registry_row(r, exploratory=not o.claim))
    elif not o.table:
        for r in rows:
            print(line(r))
    t = table(rows)
    if o.table:
        print(markdown(t))
    if o.json:
        with open(o.json, 'w') as fh:
            json.dump(_clean({'card': CARD, 'runs': [{k: v for k, v in r.items() if k != 'dir'} for r in rows],
                              'table': t}), fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
