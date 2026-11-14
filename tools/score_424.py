#!/usr/bin/env python3
"""tools/score_424.py -- read a 4 -> 2 -> 4 twin run (configs/experiments/rig_424_*_twin.yaml) stage by stage.

    python3 tools/score_424.py RUN_DIR [RUN_DIR ...] [--json OUT.json]

Per run, from logs/run.csv, logs/events.csv and manifest.json (no bars: the numbers as flown):
  - the 4 -> 2 step: the set detach (DETACH_SET, or the first DETACH of a succession), each leaver's release, and the
    ring's tilt peak from the command to 5 s after the last release;
  - the pair while both are away (from the last release to the first partner's contact, or LAND): the ring's tilt, its
    lean about the pair's own line (the pair has no moment about it), its height, each pair quad's throttle and the
    share of rows at the throttle cap;
  - each partner's return: its release, step-out (PARTNER RELEASE), his TAKEOFF, contact (and how many), proof, weld,
    fold-in (and the wait between weld and fold-in, ~0 since the fold-in starts at the weld), give-up and take-back, a
    later release of its ring weld; the ring's tilt at contact, its peak from contact to weld, and from the fold-in to
    10 s after it;
  - clearances while a partner is off the ring: the two partners' bodies and cables to each other, and each partner's
    body and cable to the pair's cables (body to plate);
  - every drop, LAND, fault, timeout and the planner's own lines on these steps.
Partners keep the contract's topic names (QUAD4 = drone n-1 its own, another partner d under /drone_<d> and /tejen_d<d>),
as the runner logs them in events.csv. Older one-partner runs score too (their returns section is QUAD4's).
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment.config import instance_topic  # noqa: E402

STEP_S = 5.0          # s after the last release that still belongs to the 4 -> 2 step
BLEND_S = 10.0        # s: the fold-in's blend hold (attach_traj_hold_s)
CAP_BAND = 0.01       # a throttle this close to throttle_max counts as at the cap
TROUBLE = ('FLEET_ABORT', 'FAULT_LAND', 'GIVE_UP', 'TAKE_BACK', 'REWELD_TIMEOUT', 'LOG_TIMEOUT', 'LIFT_TIMEOUT',
           'PICKUP_FREE_TIMEOUT', 'NEWCOMER_OUT', 'TILT_FAULT', 'REJOIN_ABORT', 'PLANNER_LAND', 'LAND', 'LANDED',
           'RELEASE', 'OPERATOR_GIVE_UP', 'GIVE_UP_SENT')
PLANNER_WORDS = ('DROP', 'DETACH', 'UNLOAD', 'RELEASE', 'ATTACH', 'QUEUE', 'RESIZE', 'TAKE-BACK', 'refused', 'REFUSED',
                 'ABORT', 'SET')


# ── loading ──────────────────────────────────────────────────────────────────

def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load_run(run_dir):
    """(manifest dict, {column: np.array or list of str}, [(t, event, arg)])."""
    man = {}
    p = os.path.join(run_dir, 'manifest.json')
    if os.path.exists(p):
        with open(p) as fh:
            man = json.load(fh)
    with open(os.path.join(run_dir, 'logs', 'run.csv'), newline='') as fh:
        rows = list(csv.DictReader(fh))
    cols = {}
    for k in (rows[0].keys() if rows else ()):
        vals = [r[k] for r in rows]
        nums = [_num(v) for v in vals]
        text = any(v not in ('', None) and math.isnan(x) and v.lower() != 'nan' for v, x in zip(vals, nums))
        cols[k] = vals if text else np.array(nums)
    events = []
    with open(os.path.join(run_dir, 'logs', 'events.csv'), newline='') as fh:
        for r in csv.DictReader(fh):
            events.append((_num(r['sim_time']), r['event'], r.get('arg') or ''))
    return man, cols, events


# ── geometry ─────────────────────────────────────────────────────────────────

def quat_rot(qw, qx, qy, qz):
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    w, x, y, z = qw / n, qx / n, qy / n, qz / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def seg_dist(p1, q1, p2, q2):
    """Least distance between segments p1-q1 and p2-q2 (a segment may be a point)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e <= 1e-12:
            s, t = min(max(-c / a, 0.0), 1.0), 0.0
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = min(max(-c / a, 0.0), 1.0), 0.0
            elif t > 1.0:
                s, t = min(max((b - c) / a, 0.0), 1.0), 1.0
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


class Run:
    def __init__(self, run_dir):
        self.dir = run_dir
        self.man, self.c, self.ev = load_run(run_dir)
        la = self.man.get('launch_args') or {}
        self.t = self.c['t']
        self.n = sum(1 for k in self.c if k.endswith('_x') and k[0] == 'd' and k[1:-2].isdigit())
        self.num_drones = int(la.get('num_drones', self.n))
        self.quad4 = self.n - 1
        self.r = float(la.get('attach_radius', 0.225))
        self.attach_z = float(la.get('attach_z', 0.02))
        self.thr_max = float(la.get('throttle_max', 0.95))
        self.target_z = _num(la.get('target_z'))
        self.az = self._azimuths()

    # rows
    def at(self, name, t):
        """Column `name` at the row nearest t (nan outside the run)."""
        if name not in self.c or not len(self.t) or not math.isfinite(t):
            return math.nan
        i = int(np.argmin(np.abs(self.t - t)))
        return float(self.c[name][i])

    def window(self, t0, t1):
        return (self.t >= t0) & (self.t <= t1) if math.isfinite(t0) and math.isfinite(t1) else np.zeros(len(self.t), bool)

    def body(self, i, k):
        return np.array([self.c[f'd{i}_{a}'][k] for a in 'xyz'])

    def tip(self, d, k):
        pre = 'tip_' if d == self.quad4 else f'tip_d{d}_'
        if f'{pre}x' not in self.c:
            return None
        p = np.array([self.c[f'{pre}{a}'][k] for a in 'xyz'])
        return p if np.all(np.isfinite(p)) else None

    def ring_rot(self, k):
        return quat_rot(*(self.c[f'payload_q{a}'][k] for a in 'wxyz'))

    def plate(self, i, k):
        """Drone i's plate point in the world at row k (its azimuth on the ring, attach_z above the ring origin)."""
        a = math.radians(self.az[i])
        local = np.array([self.r * math.cos(a), self.r * math.sin(a), self.attach_z])
        return np.array([self.c[f'payload_{a_}'][k] for a_ in 'xyz']) + self.ring_rot(k) @ local

    def _azimuths(self):
        """Each drone's plate azimuth (counter-clockwise deg in the ring frame), from its bearing at the first row: every
        twin drone starts laid out on its plate's radial line; snapped to the 30 deg plates."""
        out = {}
        if not len(self.t):
            return out
        rot = self.ring_rot(0)
        ring = np.array([self.c[f'payload_{a}'][0] for a in 'xyz'])
        for i in range(self.n):
            v = rot.T @ (self.body(i, 0) - ring)
            out[i] = (round(math.degrees(math.atan2(v[1], v[0])) / 30.0) * 30.0) % 360.0
        return out

    def plate_no(self, i):
        return int(round((-self.az[i]) % 360.0 / 30.0)) % 12

    # events
    def first(self, name, arg=None, after=-math.inf, pred=None):
        for t, e, a in self.ev:
            if e == name and (arg is None or a in (arg if isinstance(arg, (set, tuple, list)) else (arg,))) \
                    and (math.isfinite(t) and t >= after) and (pred is None or pred(a)):
                return t
        return math.nan

    def all(self, name, arg=None, after=-math.inf):
        return [t for t, e, a in self.ev if e == name and (arg is None or a == arg) and math.isfinite(t) and t >= after]

    def topic(self, today, d):
        return instance_topic(today, d, self.quad4)


def _stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return {'mean': None, 'p95': None, 'max': None}
    return {'mean': round(float(np.mean(x)), 3), 'p95': round(float(np.percentile(x, 95)), 3),
            'max': round(float(np.max(x)), 3)}


def _r(x, k=2):
    return None if x is None or not math.isfinite(x) else round(float(x), k)


def lean_about(run, k, az_deg):
    """The ring's lean about the line through the plate at az_deg and the centre (deg): the rise of its in-plane normal;
    positive = the side 90 deg counter-clockwise of az_deg high."""
    a = math.radians(az_deg)
    v = run.ring_rot(k) @ np.array([-math.sin(a), math.cos(a), 0.0])
    return math.degrees(math.asin(max(-1.0, min(1.0, float(v[2])))))


# ── the stages ───────────────────────────────────────────────────────────────

def score(run_dir):
    run = Run(run_dir)
    tilt = run.c.get('payload_tilt_deg', np.full(len(run.t), math.nan))
    out = {'run': os.path.basename(os.path.normpath(run_dir)), 'scenario': os.path.basename(run.man.get('scenario', '')),
           'exit': run.man.get('exit_reason'), 'passed': run.man.get('passed'), 'failures': run.man.get('failures') or [],
           'sim_s': _r(run.man.get('sim_duration_s'), 1), 'wall_s': _r(run.man.get('wall_s'), 0),
           'rtf': _r(run.man.get('mean_rtf'), 3)}

    # who leaves: the set detach's drones, else every DETACH of the run
    t_set = run.first('DETACH_SET')
    if math.isfinite(t_set):
        arg = next(a for t, e, a in run.ev if e == 'DETACH_SET' and t == t_set)
        leavers = [int(v) for v in arg.split(',') if v.strip()]
        t_cmd = t_set
    else:
        leavers = [int(a) for t, e, a in run.ev if e == 'DETACH' and a.strip().isdigit()]
        t_cmd = run.first('DETACH')
    leavers = list(dict.fromkeys(leavers))
    pair = [i for i in range(run.n) if i not in leavers]
    t_land = run.first('LAND')
    t_end = float(run.t[-1]) if len(run.t) else math.nan
    out['fleet'] = {'pair': {i: run.plate_no(i) for i in pair}, 'leavers': {d: run.plate_no(d) for d in leavers},
                    'command': 'DETACH_SET' if math.isfinite(t_set) else 'DETACH', 't_command': _r(t_cmd)}

    # each leaver's release: its own weld topic going False, else the planner's resize line for it
    rel = {}
    for d in leavers:
        att = {'magnet/object_attached', '/magnet/object_attached'} if d == run.quad4 else \
            {run.topic('/magnet/object_attached', d)}
        t_rel = run.first('RELEASED', att, after=t_cmd)
        if not math.isfinite(t_rel):
            t_rel = run.first('DETACH_RESIZE', str(d), after=t_cmd)
        rel[d] = t_rel
    t_all_off = max((v for v in rel.values() if math.isfinite(v)), default=math.nan)
    w = run.window(t_cmd - 0.5, (t_all_off if math.isfinite(t_all_off) else t_cmd) + STEP_S)
    k_peak = (int(np.nanargmax(np.where(w, tilt, -np.inf))) if w.any() and np.any(np.isfinite(tilt[w]))
              else None)
    out['step_4_to_2'] = {
        'released': {d: _r(v) for d, v in rel.items()},
        'resize_lines': [_r(t) for t in run.all('DETACH_RESIZE', after=t_cmd - 0.5)],
        'tilt_peak_deg': _r(float(tilt[k_peak])) if k_peak is not None else None,
        't_tilt_peak': _r(float(run.t[k_peak])) if k_peak is not None else None,
        'window': [_r(t_cmd - 0.5), _r((t_all_off if math.isfinite(t_all_off) else t_cmd) + STEP_S)]}

    # the partners' returns (reserved drones only; a succession's carriers just land)
    partners = [d for d in leavers if d >= run.num_drones]
    returns, first_back = {}, math.inf
    for d in partners:
        top = (lambda today, d=d: run.topic(today, d))
        go = run.first('PARTNER_GO', after=t_cmd,
                       pred=lambda a, d=d: (a == '' and d == run.quad4) or str(d) in a.split(','))
        t0 = go if math.isfinite(go) else (rel[d] if math.isfinite(rel[d]) else t_cmd)
        contacts = run.all('CONTACT', top('/magnet/contact'), after=t0)
        t_contact = contacts[0] if contacts else math.nan
        t_proof = run.first('PROOF:attached', top('/magnet/attach_proof'), after=t0)
        if not math.isfinite(t_proof):
            col = 'proof_state' if d == run.quad4 else f'proof_d{d}_state'
            if col in run.c and not isinstance(run.c[col], np.ndarray):
                ks = [k for k, v in enumerate(run.c[col]) if v == 'attached' and run.t[k] >= t0]
                t_proof = float(run.t[ks[0]]) if ks else math.nan
        att = {'magnet/object_attached', '/magnet/object_attached'} if d == run.quad4 else {top('/magnet/object_attached')}
        t_weld = run.first('REWELD', att, after=t0)
        t_fold = run.first('FOLD_IN', str(d), after=t_weld if math.isfinite(t_weld) else t0)
        t_back = run.first('TAKE_BACK', top(f'/drone_{run.quad4}/take_back'), after=t_cmd)
        t_give = run.first('GIVE_UP', top('/join_planner/give_up'), after=t_cmd)
        for v in (t_contact, t_weld):
            if math.isfinite(v):
                first_back = min(first_back, v)
                break
        r = {'plate': run.plate_no(d), 'released': _r(rel[d]),
             'stepped_out': _r(run.first('PARTNER_RELEASE', str(d), after=t_cmd)),
             'his_takeoff': _r(go), 'his_phase_takeoff': _r(run.first('PHASE:TAKEOFF', top('/join_planner/phase'),
                                                                      after=t_cmd)),
             'contact': _r(t_contact), 'contacts': len(contacts), 'proof': _r(t_proof), 'weld': _r(t_weld),
             'fold_in': _r(t_fold), 'fold_in_wait_s': _r(t_fold - t_weld) if math.isfinite(t_fold - t_weld) else None,
             'give_up': _r(t_give), 'take_back': _r(t_back),
             'released_again': [_r(t) for t, e, a in run.ev if e == 'RELEASED' and a in att and t > t_weld],
             'tilt_at_contact_deg': _r(run.at('payload_tilt_deg', t_contact)),
             'tilt_peak_contact_to_weld_deg': None, 'tilt_peak_foldin_10s_deg': None}
        if math.isfinite(t_contact):
            end = t_weld if math.isfinite(t_weld) else t_contact + BLEND_S
            r['tilt_peak_contact_to_weld_deg'] = _stats(tilt[run.window(t_contact, end)])['max']
        if math.isfinite(t_fold):
            r['tilt_peak_foldin_10s_deg'] = _stats(tilt[run.window(t_fold, t_fold + BLEND_S)])['max']
        returns[d] = r
    out['returns'] = returns

    # the pair while every leaver is away: from the last release to the first contact (or weld), LAND, or the end
    p0 = t_all_off if math.isfinite(t_all_off) else t_cmd
    p1 = min(v for v in (first_back, t_land, t_end) if math.isfinite(v)) if math.isfinite(p0) else math.nan
    wp = run.window(p0, p1)
    ks = np.nonzero(wp)[0]
    pair_az = run.az[pair[0]] if pair else 0.0
    lean = np.array([lean_about(run, k, pair_az) for k in ks]) if len(pair) == 2 else np.array([])
    thr = {i: _stats(run.c[f'd{i}_thr'][wp]) for i in pair}
    at_cap = {i: _r(100.0 * float(np.mean(run.c[f'd{i}_thr'][wp] >= run.thr_max - CAP_BAND)), 1) if len(ks) else None
              for i in pair}
    z = run.c['payload_z'][wp]
    out['pair_alone'] = {
        'carriers': len(pair), 'from': _r(p0), 'to': _r(p1), 's': _r(p1 - p0, 1) if math.isfinite(p1 - p0) else None,
        'tilt_deg': _stats(tilt[wp]),
        'lean_about_pair_line_deg': {**_stats(np.abs(lean)), 'mean_signed': _r(float(np.mean(lean)), 2)
                                     if len(lean) else None},
        'ring_z_min': _r(float(np.nanmin(z)), 3) if len(z) else None, 'ring_z_mean': _r(float(np.nanmean(z)), 3)
        if len(z) else None, 'target_z': _r(run.target_z, 2),
        'throttle': thr, 'pct_at_cap': at_cap, 'throttle_max': run.thr_max}

    # clearances while a partner is off the ring and the fleet flies: its release to its weld, LAND or the end
    free = {}
    for d in partners:
        w_end = returns[d]['weld'] if returns[d]['weld'] is not None else (t_land if math.isfinite(t_land) else t_end)
        free[d] = run.window(rel[d] if math.isfinite(rel[d]) else t_cmd, w_end)
    clear = {}
    if len(partners) >= 2:
        a, b = partners[:2]
        best_body, best_cable = (math.inf, None), (math.inf, None)
        for k in np.nonzero(free[a] | free[b])[0]:
            pa, pb = run.body(a, k), run.body(b, k)
            if not (np.all(np.isfinite(pa)) and np.all(np.isfinite(pb))):
                continue
            dist = float(np.linalg.norm(pa - pb))
            if dist < best_body[0]:
                best_body = (dist, float(run.t[k]))
            ta, tb = run.tip(a, k), run.tip(b, k)
            dc = seg_dist(pa, ta if ta is not None else pa, pb, tb if tb is not None else pb)
            if dc < best_cable[0]:
                best_cable = (dc, float(run.t[k]))
        clear[f'partners_{a}_{b}'] = {'body_min_m': _r(best_body[0], 3), 't_body': _r(best_body[1]),
                                      'cable_min_m': _r(best_cable[0], 3), 't_cable': _r(best_cable[1])}
    for d in partners:
        best_body, best_cable = (math.inf, None, None), (math.inf, None, None)
        for k in np.nonzero(free[d])[0]:
            pd_ = run.body(d, k)
            if not np.all(np.isfinite(pd_)):
                continue
            td = run.tip(d, k)
            for i in pair:
                pi, pl = run.body(i, k), run.plate(i, k)
                if not (np.all(np.isfinite(pi)) and np.all(np.isfinite(pl))):
                    continue
                db = seg_dist(pd_, pd_, pi, pl)
                if db < best_body[0]:
                    best_body = (db, float(run.t[k]), i)
                dc = seg_dist(pd_, td if td is not None else pd_, pi, pl)
                if dc < best_cable[0]:
                    best_cable = (dc, float(run.t[k]), i)
        clear[f'partner_{d}_to_pair_cables'] = {
            'body_min_m': _r(best_body[0], 3), 't_body': _r(best_body[1]), 'cable_of': best_body[2],
            'its_cable_min_m': _r(best_cable[0], 3), 't_cable': _r(best_cable[1]), 'its_cable_to': best_cable[2]}
    out['clearance'] = clear

    out['trouble'] = [(_r(t), e, a) for t, e, a in run.ev if e in TROUBLE or e.startswith('PHASE:TAKEN_BACK')]
    out['planner'] = [(_r(t), a) for t, e, a in run.ev
                      if e == 'PLANNER' and any(wd in a for wd in PLANNER_WORDS)]
    drops = [a for t, e, a in run.ev if e == 'PLANNER' and 'DROP AND LAND' in a]
    out['drop'] = drops[0] if drops else None
    return out


# ── output ───────────────────────────────────────────────────────────────────

def _v(x, unit=''):
    return '-' if x is None else f'{x}{unit}'


def report(s):
    f = s['fleet']
    lines = [f"{s['run']}  {s['scenario']}  exit: {s['exit']}  passed: {s['passed']}  sim {_v(s['sim_s'])} s, "
             f"wall {_v(s['wall_s'])} s, RTF {_v(s['rtf'])}"]
    lines.append(f"  pair {', '.join(f'd{i} (plate {p})' for i, p in f['pair'].items())}; leavers "
                 f"{', '.join(f'd{i} (plate {p})' for i, p in f['leavers'].items())}; {f['command']} at t={_v(f['t_command'])}")
    st = s['step_4_to_2']
    lines.append(f"  4 -> 2: released {', '.join(f'd{d} t={_v(t)}' for d, t in st['released'].items())}; ring tilt peak "
                 f"{_v(st['tilt_peak_deg'])} deg at t={_v(st['t_tilt_peak'])} (t={st['window'][0]}..{st['window'][1]})")
    pa = s['pair_alone']
    ln = pa['lean_about_pair_line_deg']
    who = 'pair' if pa['carriers'] == 2 else f"{pa['carriers']} carriers"
    lines.append(f"  {who} alone t={_v(pa['from'])}..{_v(pa['to'])} ({_v(pa['s'])} s): tilt mean/p95/max "
                 f"{_v(pa['tilt_deg']['mean'])}/{_v(pa['tilt_deg']['p95'])}/{_v(pa['tilt_deg']['max'])} deg; lean about its "
                 f"line |mean|/max {_v(ln['mean'])}/{_v(ln['max'])} deg (signed mean {_v(ln['mean_signed'])}); ring z min "
                 f"{_v(pa['ring_z_min'])} mean {_v(pa['ring_z_mean'])} (target {_v(pa['target_z'])})")
    for i, t in pa['throttle'].items():
        lines.append(f"    d{i} throttle mean/p95/max {_v(t['mean'])}/{_v(t['p95'])}/{_v(t['max'])}, "
                     f"{_v(pa['pct_at_cap'][i])} % at the {pa['throttle_max']} cap")
    for d, r in s['returns'].items():
        lines.append(f"  d{d} (plate {r['plate']}): released {_v(r['released'])}, stepped out {_v(r['stepped_out'])}, his "
                     f"TAKEOFF {_v(r['his_takeoff'])} (phase {_v(r['his_phase_takeoff'])}), contact {_v(r['contact'])} "
                     f"({r['contacts']}), proof {_v(r['proof'])}, weld {_v(r['weld'])}, fold-in {_v(r['fold_in'])} "
                     f"(+{_v(r['fold_in_wait_s'])} s); give-up {_v(r['give_up'])}, take-back {_v(r['take_back'])}"
                     + (f"; released again {r['released_again']}" if r['released_again'] else ''))
        lines.append(f"      ring tilt at contact {_v(r['tilt_at_contact_deg'])} deg, peak contact -> weld "
                     f"{_v(r['tilt_peak_contact_to_weld_deg'])}, fold-in + 10 s {_v(r['tilt_peak_foldin_10s_deg'])}")
    for k, c in s['clearance'].items():
        if k.startswith('partners_'):
            lines.append(f"  {k}: bodies {_v(c['body_min_m'])} m at t={_v(c['t_body'])}, cables {_v(c['cable_min_m'])} m "
                         f"at t={_v(c['t_cable'])}")
        else:
            lines.append(f"  {k}: body {_v(c['body_min_m'])} m (to d{_v(c['cable_of'])}'s) at t={_v(c['t_body'])}, its "
                         f"cable {_v(c['its_cable_min_m'])} m (to d{_v(c['its_cable_to'])}'s) at t={_v(c['t_cable'])}")
    if s['drop']:
        lines.append(f"  DROP: {s['drop']}")
    lines.append('  events: ' + ('; '.join(f'{t} {e} {a}'.rstrip() for t, e, a in s['trouble']) or '-'))
    for f_ in s['failures']:
        lines.append(f'  !! {f_}')
    if s['planner']:
        lines.append('  planner:')
        lines.extend(f'    {t}  {a}' for t, a in s['planner'])
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run directories (results/<date>/R####_sim_gz_rig_424_*)')
    ap.add_argument('--json', help='write every run\'s numbers to this file')
    a = ap.parse_args()
    scores = []
    for d in a.runs:
        s = score(d)
        scores.append(s)
        print(report(s))
        print()
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(scores, fh, indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
