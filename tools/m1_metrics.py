#!/usr/bin/env python3
"""tools/m1_metrics.py -- the combined-M1 run metrics, one run directory at a time (card
2026-10-10_combined_m1_contact rev 10 §5, §6.2, §6.4; card 2026-10-11_quad4_tilt_cut rev 3/4). tools/m1_score.py
prints the registry rows; this module only measures.

Run time is the runner's sim time (events.csv, run.csv). Events with no time (the startup rows, sim_time nan: the
welded start's WELD and MUX:ours) are not events of the flight: the attach is the first timed WELD, else the first
timed REWELD (the full M1 logs REWELD).

Families:
  contact()      §5 contact, proof, hand-over throttle, tilt at the proof, ring tilt and dip, keyed on the attach
  authority()    §5 authority counts from launch.log (the ATTACH line, HOLDING lines, OFF/DETACH after the attach)
  stages()       the full M1's stage times and the detach-window ring tilt
  flight()       his flight (QUAD4 free, before the switch): body, rod-segment and ball clearance, overflight
  take_back()    the TB4 take-back bars: take-back to QUAD4 down (rod-segment, body, attitude steps, ring tilt,
                 overflight, down within 60 s, zone, decouple, phase and TB5's prediction on orbit runs)
  tilt()         the tilt-cut readouts from the injection (MOCAP_TILT, MOCAP_LOSS or a nonzero WRENCH)
  tb14()         the landing tip-over (card §6.4) and the tilt card's rev 4 landing rule (9.2)

Geometry (the §5 pairs, as tools/tb5_replay.py): QUAD4's rod is the segment body -> magnet tip (run.csv tip_*),
extended tip -> the ball's far side while he carries it (PICKUP .. DROP); a carrier's rod is its body -> its plate
(ring position + the plate's offset at attach_azimuths_deg, attach_radius, attach_z; the ring's attitude is not
applied, as in the TB5 replay, so logged and predicted clearances share one geometry). Carriers are numbered from 1
(carrier 1 = /drone_0).
"""
import glob
import json
import os
import re
import subprocess
import sys

import numpy as np
import pandas as pd

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from experiment.config import wrench_arg  # noqa: E402  (the runner's own WRENCH grammar)
from tip_metrics import is_push  # noqa: E402  (a WRENCH that applies a force or a torque)
from run_dir import manifest_azimuths_ccw  # noqa: E402

ROTOR_R = 0.2           # m, a carrier's rotor disc about its body centre (§5 take-back, reported)
BALL_R = 0.08           # m, his twin ball (tejen payload_model.sdf)
AIR_Z = 0.2             # m, QUAD4 counted airborne above this
STEP_PRE, STEP_POST = 0.1, 0.3      # s, the attitude / hand-over step windows (§5)
DOWN_WITHIN_S = 60.0    # s, TB4 (round 4 F5): QUAD4 down within 60 s of the take-back
TB14_ABORT_AFTER_LAND_S = 5.0
TB14_TILT_DEG = 60.0
TB14_RISE_S = 1.0
TB14_LEVEL_DEG = 10.0   # the rise is timed from QUAD4's last tilt under this before the 60 deg crossing
RING_DOWN_Z = 0.07      # m, tilt card rev 4 9.2: the ring is down
TIP_AT_PLATE_Z = 0.10   # m, tilt card rev 4 9.2: the welded tip still at plate 9 (it reads 0.05-0.06 m on the floor)
TIP_FLOOR_Z_F10 = 0.03  # m, critic round 4 F10's wording (the welded twin tip never gets this low)
UNWIND_TILT_DEG = 30.0  # tilt card rev 4 9.2
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')
INJECTIONS = ('MOCAP_TILT', 'MOCAP_LOSS', 'WRENCH')


# ── where runs live ──────────────────────────────────────────────────────────────────────────────────────────────

def results_roots():
    """MDC_RESULTS_ROOT, this tree's results/ and the main tree's (a worktree holds only a stale registry)."""
    roots = []
    if os.environ.get('MDC_RESULTS_ROOT'):
        roots.append(os.path.abspath(os.path.expanduser(os.environ['MDC_RESULTS_ROOT'])))
    roots.append(os.path.join(REPO, 'results'))
    common = subprocess.run(['git', '-C', REPO, 'rev-parse', '--git-common-dir'], capture_output=True, text=True)
    if common.returncode == 0 and common.stdout.strip():
        roots.append(os.path.join(os.path.dirname(os.path.abspath(os.path.join(REPO, common.stdout.strip()))),
                                  'results'))
    out = []
    for r in roots:
        if r not in out:
            out.append(r)
    return out


def run_dir(rid):
    """A run directory, or the newest results/<date>/<rid>_* under the results roots."""
    if os.path.isdir(rid):
        return os.path.abspath(rid)
    hits = sorted(h for root in results_roots() for h in glob.glob(os.path.join(root, '*', f'{rid}_*'))
                  if os.path.isdir(h))
    if not hits:
        raise SystemExit(f'{rid}: no run directory under {results_roots()}')
    return hits[-1]


def registry_path():
    """The live registry: MDC_RESULTS_ROOT's, else the main tree's (the session appends there), else this tree's."""
    roots = results_roots()
    order = roots[:1] if os.environ.get('MDC_RESULTS_ROOT') else []
    order += roots[::-1]
    for root in order:
        p = os.path.join(root, 'registry.csv')
        if os.path.exists(p):
            return p
    return None


def read_registry(path=None):
    """{run id: row dict}: a '<id>-corr' row is read in preference to its original, wherever it sits in the file
    (the originals of misaligned rows stay untouched; decisions.md D15)."""
    import csv
    rows = {}
    with open(path or registry_path(), newline='', encoding='utf-8', errors='replace') as fh:
        rd = csv.reader(fh)
        head = next(rd)
        for row in rd:
            if not row or not row[0].startswith('R'):
                continue
            corr = row[0].endswith('-corr')
            base = row[0][:-5] if corr else row[0]
            d = dict(zip(head, row + [''] * (len(head) - len(row))))
            d['_corr'], d['_fields'] = corr, len(row)
            if corr or base not in rows:
                rows[base] = d
    return rows


# ── one run ──────────────────────────────────────────────────────────────────────────────────────────────────────

class Run:
    def __init__(self, rid):
        self.d = run_dir(rid)
        self.id = os.path.basename(self.d).split('_')[0]
        self.date = os.path.basename(os.path.dirname(self.d))
        self.df = pd.read_csv(os.path.join(self.d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        ev = pd.read_csv(os.path.join(self.d, 'logs', 'events.csv'), dtype={'arg': str})
        ev['sim_time'] = pd.to_numeric(ev.sim_time, errors='coerce')
        self.ev_all = ev
        self.ev = ev[np.isfinite(ev.sim_time)].reset_index(drop=True)
        mpath = os.path.join(self.d, 'manifest.json')
        self.manifest = json.load(open(mpath)) if os.path.exists(mpath) else {}
        self.man = self.manifest.get('launch_args', {})
        self.n = int(self.man.get('num_drones') or 3)
        self.k = f'd{self.n}_'
        self.az = (manifest_azimuths_ccw(self.man.get('attach_azimuths_deg'), self.manifest)
                   or [270.0, 30.0, 150.0])[:self.n]
        self.r_att = float(self.man.get('attach_radius') or 0.225)
        self.z_att = float(self.man.get('attach_z') or 0.02)
        self._log = None

    @property
    def log(self):
        if self._log is None:
            p = os.path.join(self.d, 'logs', 'launch.log')
            self._log = open(p, errors='replace').read().splitlines() if os.path.exists(p) else []
        return self._log

    @property
    def config(self):
        sc = self.manifest.get('scenario')
        if sc:
            return os.path.splitext(os.path.basename(sc))[0]
        return os.path.basename(self.d).split('_', 3)[-1] if os.path.basename(self.d).startswith('R') else ''

    def times(self, name, after=None, before=None):
        t = self.ev.sim_time[self.ev.event == name].to_numpy(float)
        if after is not None:
            t = t[t >= after]
        if before is not None:
            t = t[t <= before]
        return t

    def t(self, name, after=None, before=None, last=False):
        x = self.times(name, after, before)
        return (float(x[-1]) if last else float(x[0])) if len(x) else None

    def arg(self, name, at):
        e = self.ev[(self.ev.event == name) & (np.abs(self.ev.sim_time - at) < 1e-9)]
        return str(e.arg.iloc[0]) if len(e) else ''

    # QUAD4, the carriers, the ring, as arrays over run.csv rows
    def col(self, name):
        return self.df[name].to_numpy(float) if name in self.df else np.full(len(self.df), np.nan)

    def xyz(self, pre):
        return np.stack([self.col(pre + 'x'), self.col(pre + 'y'), self.col(pre + 'z')], -1)

    @property
    def tt(self):
        return self.df.t.to_numpy(float)

    def attach(self):
        """(time, 'WELD' | 'REWELD') of the attach, or (None, None). Untimed startup rows are not an attach."""
        w = self.t('WELD')
        if w is not None:
            return w, 'WELD'
        r = self.t('REWELD')
        return (r, 'REWELD') if r is not None else (None, None)


def r2(v, n=2):
    return None if v is None or not np.isfinite(v) else round(float(v), n)


def _window(df, a, b):
    return df[(df.t >= a) & (df.t <= b)]


def step(r, col, t0):
    """Max |x - mean(x over the 0.1 s before t0)| over the 0.3 s from t0 (§5 hand-over / TB4 attitude step)."""
    df = r.df
    pre = df[(df.t >= t0 - STEP_PRE) & (df.t < t0)][col]
    post = df[(df.t >= t0) & (df.t < t0 + STEP_POST)][col]
    if not len(pre) or not len(post):
        return None
    return float((post - pre.mean()).abs().max())


# ── geometry ─────────────────────────────────────────────────────────────────────────────────────────────────────

def _dot(a, b):
    return np.einsum('...i,...i->...', a, b)


def seg_seg(p1, q1, p2, q2, eps=1e-12):
    """Closest distance between segments p1-q1 and p2-q2, row by row (any dimension; a point is a segment of zero
    length). The same cases as tools/tb5_replay.seg_seg."""
    p1, q1, p2, q2 = np.broadcast_arrays(*(np.asarray(x, float) for x in (p1, q1, p2, q2)))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f, c, b = _dot(d1, d1), _dot(d2, d2), _dot(d2, r), _dot(d1, r), _dot(d1, d2)
    A, E = a < eps, e < eps
    a_, e_ = np.where(A, 1.0, a), np.where(E, 1.0, e)
    den = a * e - b * b
    ok = den > eps
    s = np.where(ok, np.clip((b * f - c * e) / np.where(ok, den, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e_
    s = np.where(t < 0.0, np.clip(-c / a_, 0.0, 1.0), np.where(t > 1.0, np.clip((b - c) / a_, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    s = np.where(E & ~A, np.clip(-c / a_, 0.0, 1.0), s)
    t = np.where(E, 0.0, t)
    s = np.where(A, 0.0, s)
    t = np.where(A & ~E, np.clip(f / e_, 0.0, 1.0), t)
    return np.linalg.norm(p1 + d1 * s[..., None] - p2 - d2 * t[..., None], axis=-1)


def plates(r, ring):
    """Each carrier's plate, ring position + its plate offset (ring attitude not applied, as tools/tb5_replay)."""
    return [ring + np.array([r.r_att * np.cos(np.radians(a)), r.r_att * np.sin(np.radians(a)), r.z_att])
            for a in r.az]


def ball_carried(r, t):
    """Mask: his object magnet holds the ball (PICKUP .. DROP)."""
    up, out = r.t('PICKUP'), np.zeros(len(t), bool)
    if up is None:
        return out
    drop = r.t('DROP', after=up)
    return (t >= up) & ((t < drop) if drop is not None else True)


def far_side(tip, ball):
    """The ball's far side on the line tip -> ball centre."""
    v = ball - tip
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return ball + np.where(n > 1e-9, v / np.where(n > 1e-9, n, 1.0), 0.0) * BALL_R


def clearance(r, mask, ball_far=False):
    """Over the rows in mask: QUAD4's rod (body -> tip) and, while carried, tip -> ball (centre, or its far side) to
    every carrier body and carrier rod; QUAD4's body to every carrier body. Returns per-row arrays."""
    t = r.tt
    body, tip, ball = r.xyz(r.k), r.xyz('tip_'), r.xyz('ball_')
    ring = r.xyz('payload_')
    carried = ball_carried(r, t) & np.all(np.isfinite(ball), 1)
    end = far_side(tip, ball) if ball_far else ball
    rod = np.full(len(t), np.inf)
    rod_who = np.full(len(t), -1)
    ball_d = np.full(len(t), np.inf)
    bod = np.full(len(t), np.inf)
    bod_who = np.full(len(t), -1)
    pl = plates(r, ring)
    for i in range(r.n):
        cb = r.xyz(f'd{i}_')
        for target in (seg_seg(body, tip, cb, cb), seg_seg(body, tip, cb, pl[i])):
            better = target < rod
            rod_who[better], rod[better] = i, target[better]
        for target in (seg_seg(tip, end, cb, cb), seg_seg(tip, end, cb, pl[i])):
            target = np.where(carried, target, np.inf)
            better = target < rod
            rod_who[better], rod[better] = i, target[better]
            ball_d = np.minimum(ball_d, target)
        d = np.linalg.norm(body - cb, axis=1)
        better = d < bod
        bod_who[better], bod[better] = i, d[better]
    ok = mask & np.all(np.isfinite(body), 1)
    rod_ok = ok & np.all(np.isfinite(tip), 1) & np.isfinite(rod)
    return {'rod': np.where(rod_ok, rod, np.inf), 'rod_who': rod_who, 'ball': np.where(rod_ok, ball_d, np.inf),
            'body': np.where(ok & np.isfinite(bod), bod, np.inf), 'body_who': bod_who}


def overflight(r, mask):
    """Rows where a carrier's rotor disc (0.2 m about its body) lies under QUAD4's body or rod in xy, the carrier
    below QUAD4's body. Returns (seconds, min tip-to-carrier vertical gap, seconds with the body alone over a disc)."""
    t = r.tt
    body, tip = r.xyz(r.k), r.xyz('tip_')
    over = np.zeros(len(t), bool)
    over_body = np.zeros(len(t), bool)
    gap = np.full(len(t), np.inf)
    for i in range(r.n):
        cb = r.xyz(f'd{i}_')
        below = cb[:, 2] < body[:, 2]
        tip_xy = np.where(np.isfinite(tip[:, :2]), tip[:, :2], body[:, :2])
        dxy = seg_seg(body[:, :2], tip_xy, cb[:, :2], cb[:, :2])
        o = mask & below & (dxy < ROTOR_R)
        ob = mask & below & (np.linalg.norm(body[:, :2] - cb[:, :2], axis=1) < ROTOR_R)
        over |= o
        over_body |= ob
        gap = np.where(o, np.minimum(gap, tip[:, 2] - cb[:, 2]), gap)
    dt = float(np.median(np.diff(t[mask]))) if mask.sum() > 1 else 0.0
    g = gap[over & np.isfinite(gap)]
    return round(over.sum() * dt, 2), (round(float(g.min()), 3) if len(g) else None), round(over_body.sum() * dt, 2)


def _min_at(r, arr, who=None):
    if not np.isfinite(arr).any():
        return None, None, None
    i = int(np.argmin(arr))
    return round(float(arr[i]), 3), round(float(r.tt[i]), 2), (int(who[i]) + 1 if who is not None else None)


# ── §5: contact, proof, hand-over ────────────────────────────────────────────────────────────────────────────────

def contact(r):
    """contact_score's §5 numbers, keyed on the attach (WELD, else REWELD)."""
    df = r.df
    tw, kind = r.attach()
    desc = r.t('PHASE:DESCEND_TO_ATTACHMENT')
    tc = r.t('CONTACT', before=tw, last=True) if tw is not None else r.t('CONTACT', last=True)
    tm = r.t('MUX:ours')
    out = {'attach': tw, 'attach_event': kind, 'reweld_timeout': r.t('REWELD_TIMEOUT'), 'contact': tc,
           'descend_to_contact_s': r2(tc - desc) if tc is not None and desc is not None else None,
           'tries': int(len(r.times('CONTACT'))),
           'contact_to_proof_s': r2(tw - tc) if tw is not None and tc is not None else None,
           'proof_to_mux_s': r2(tm - tw, 3) if tm is not None and tw is not None else None}
    k = r.k
    if tw is not None and k + 'thr' in df:
        pre = df[(df.t >= tw - STEP_PRE) & (df.t < tw)]
        post = df[(df.t >= tw) & (df.t < tw + STEP_POST)]
        hover = df[(df.t >= tc - 2) & (df.t < tc)][k + 'thr'].mean() if tc is not None else np.nan
        out['thr_step_pct_hover'] = r2(100 * abs(post[k + 'thr'].max() - pre[k + 'thr'].mean()) / hover, 1)
        out['thr_before'] = r2(pre[k + 'thr'].mean(), 3)       # rounded once, from the raw means
        out['thr_after'] = r2(post[k + 'thr'].mean(), 3)
        at = df.iloc[int((df.t - tw).abs().argmin())]
        out['q4_tilt_at_confirm_deg'] = r2(at[k + 'tilt_deg'])
        a = tc if tc is not None else tw
        w = _window(df, a, tw + 8)
        out['ring_tilt_peak_contact_to_confirm8_deg'] = r2(w.payload_tilt_deg.max())
        base = df[(df.t >= a - 2) & (df.t < a)].payload_z.mean()
        out['dip_m'] = r2(base - w.payload_z.min(), 4)
        p = df[(df.t >= a) & (df.t <= tw)]
        for c in ('proof_drift_m', 'proof_plate_rise_m'):
            if c in df:
                v = pd.to_numeric(p[c], errors='coerce').dropna()
                out[c + '_at_proof'] = r2(v.iloc[-1], 4) if len(v) else None
    else:
        a = r.t('PHASE:TRANSIT_TO_REATTACH')       # no confirmation: the ring tilt over his rejoin (§5)
        if a is not None:
            ends = [x for x in (r.t('GIVE_UP', after=a), r.t('TAKE_BACK', after=a), r.t('LAND', after=a),
                                r.t('FLEET_ABORT', after=a)) if x is not None]
            w = _window(df, a, min(ends) if ends else float(df.t.iloc[-1]))
            out['ring_tilt_rejoin_deg'] = r2(w.payload_tilt_deg.max())
    out['landed'] = r.t('LANDED') is not None
    out['abort'] = r.t('FLEET_ABORT') is not None
    return out


def authority(r):
    """claim_checks' §5 authority and safety counts, keyed on the attach (WELD, else REWELD)."""
    log = r.log
    tw, _ = r.attach()
    mux, tb = r.t('MUX:ours'), r.t('TAKE_BACK')
    att = next((i for i, ln in enumerate(log) if 'ATTACH: proven' in ln), None)
    after = log[att:] if att is not None else []
    n4 = r.n + 1
    return {
        'holding_lines': sum('HOLDING approach authority' in ln for ln in log),
        'mux_before_attach': bool((mux is not None and tw is not None and mux < tw - 1e-3)
                                  or (mux is not None and tw is None and tb is None)),
        'off_or_detach_after_attach': sum(bool(re.search(r'Magnet command: OFF|DETACH|RELEASE \(sim joint\)|magnet OFF',
                                                         ln)) and 'refused' not in ln for ln in after),
        'trajectory_held': sum('trajectory held' in ln and 'ATTACH commanded' in ln for ln in log),
        'approach_started': sum(f'approach: drone {r.n} flies to the weld target' in ln for ln in log),
        'pose_timeouts_d4': sum(f'Drone {n4}' in ln and 'pose' in ln.lower() and 'timeout' in ln.lower()
                                and 'lines above' not in ln for ln in log),
        'envelope_d4': sum(f'[Drone {n4}] ENVELOPE FAULT' in ln for ln in log),
        'contacts': int(len(r.times('CONTACT'))), 'give_up': r.t('GIVE_UP'), 'take_back': tb, 'attach': tw,
        'mux_ours': mux, 'landed': r.t('LANDED'), 'abort': r.t('FLEET_ABORT')}


# ── the full M1 ──────────────────────────────────────────────────────────────────────────────────────────────────

def stages(r):
    """Stage times of the full M1 (welded start) and the ring tilt peak over the detach window (DETACH .. +8 s, cut
    at LAND; m1_stages' window)."""
    pj = r.ev[(r.ev.event == 'PICKUP_JOINT') & (r.ev.arg.astype(str) == 'attached')]
    tw, kind = r.attach()
    out = {'welded_start': bool(((r.ev_all.event == 'WELD') & ~np.isfinite(r.ev_all.sim_time)).any()),
           'orbit': str(r.man.get('load_traj', '')) == 'orbit',
           'detach': r.t('DETACH'), 'release': r.t('PARTNER_RELEASE'),
           'pickup': float(pj.sim_time.iloc[0]) if len(pj) else r.t('PICKUP'),
           'drop': r.t('PHASE:DROP_OBJECT') if r.t('PHASE:DROP_OBJECT') is not None else r.t('DROP'),
           'contact': r.t('CONTACT', before=tw, last=True) if tw is not None else r.t('CONTACT', last=True),
           'contacts': int(len(r.times('CONTACT'))), 'attach': tw, 'attach_event': kind,
           'reweld_timeout': r.t('REWELD_TIMEOUT'), 'fold_in': r.t('FOLD_IN'), 'take_back': r.t('TAKE_BACK')}
    det, land = out['detach'], r.t('LAND')
    if det is not None:
        w = _window(r.df, det, min(det + 8, land if land is not None else 1e9))
        out['ring_tilt_detach_deg'] = r2(w.payload_tilt_deg.max())
    return out


def stage_text(s):
    f = lambda v: f'{v:.2f} s'  # noqa: E731
    parts = ['welded start' if s['welded_start'] else 'floor start']
    if s['orbit']:
        parts.append('circle')
    for key, name in (('detach', 'DETACH'), ('release', 'release')):
        if s[key] is not None:
            parts.append(f'{name} {f(s[key])}')
    if s['pickup'] is not None:
        parts.append(f"pickup (weld {f(s['pickup'])})")
    if s['drop'] is not None:
        parts.append(f"drop {f(s['drop'])}")
    if s['contact'] is not None:
        parts.append(f"contact {f(s['contact'])}" + (f" (contacts {s['contacts']})" if s['contacts'] > 1 else ''))
    if s['attach'] is not None:
        parts.append(f"{s['attach_event']} {f(s['attach'])}")
    if s['reweld_timeout'] is not None:
        parts.append(f"REWELD_TIMEOUT {f(s['reweld_timeout'])}")
    if s['take_back'] is not None:
        parts.append(f"take-back {f(s['take_back'])}")
    if s['fold_in'] is not None and s['attach'] is not None and s['fold_in'] >= s['attach']:
        parts.append('fold-in')
    return ', '.join(parts)


# ── his flight: the §5 clearance windows before the switch ───────────────────────────────────────────────────────

def flight(r, legacy=True):
    """§5 clearance over his flight: QUAD4 free from his takeoff (the first PHASE:TAKEOFF, after the welded start's
    release; else the fleet TAKEOFF) to the switch (the first MUX:ours, attach or take-back); its rod (body -> tip)
    until his first contact; the ball (tip -> its far side) while carried. Every window also ends at LAND,
    FLEET_ABORT or LANDED. legacy: tools/attach_metrics' clearance as the claim rows carried it (MAGNET or TAKEOFF ..
    LANDED, body point to carrier bodies and rods)."""
    t = r.tt
    a = r.t('PHASE:TAKEOFF')
    if a is None:
        a = r.t('TAKEOFF') or 0.0
    ends = [x for x in (r.t('MUX:ours', after=a), r.t('TAKE_BACK', after=a), r.t('LAND', after=a),
                        r.t('FLEET_ABORT', after=a), r.t('LANDED', after=a)) if x is not None]
    b = min(ends) if ends else float(t[-1])
    c = r.t('CONTACT', after=a)
    b_rod = min(b, c) if c is not None else b
    out = {'from': r2(a), 'to': r2(b), 'rod_to': r2(b_rod)}
    whole = clearance(r, (t >= a) & (t <= b), ball_far=True)
    out['body_min'], out['body_at'], out['body_carrier'] = _min_at(r, whole['body'], whole['body_who'])
    rod = clearance_rod_only(r, (t >= a) & (t <= b_rod))         # QUAD4's own rod; the ball has its own bar
    out['rod_min'], out['rod_at'], out['rod_carrier'] = _min_at(r, rod['rod'], rod['rod_who'])
    out['ball_min'], out['ball_at'], _ = _min_at(r, whole['ball'])
    air = (t >= a) & (t <= b) & (r.col(r.k + 'z') > AIR_Z)
    out['overflight_s'], out['overflight_gap_min'], out['overflight_body_s'] = overflight(r, air)
    if legacy:
        import attach_metrics
        c = attach_metrics.analyse(r.d).get('clearance')
        out['legacy_body'], out['legacy_rod'] = (c['body'], c['rod']) if c else (None, None)
    return out


def clearance_rod_only(r, mask):
    """QUAD4's rod alone (body -> tip) to every carrier body and rod."""
    body, tip, ring = r.xyz(r.k), r.xyz('tip_'), r.xyz('payload_')
    rod = np.full(len(r.tt), np.inf)
    who = np.full(len(r.tt), -1)
    pl = plates(r, ring)
    for i in range(r.n):
        cb = r.xyz(f'd{i}_')
        for target in (seg_seg(body, tip, cb, cb), seg_seg(body, tip, cb, pl[i])):
            better = target < rod
            who[better], rod[better] = i, target[better]
    ok = mask & np.all(np.isfinite(body), 1) & np.all(np.isfinite(tip), 1) & np.isfinite(rod)
    return {'rod': np.where(ok, rod, np.inf), 'rod_who': who}


# ── TB4: the take-back ───────────────────────────────────────────────────────────────────────────────────────────

def _stamp(ln):
    m = STAMP.search(ln)
    return float(m.group(1)) if m else None


def take_back(r, tb5=True, ball_far=True):
    """The §5 take-back bars (TB4) from the take-back to QUAD4 down (its 'is down' / 'disarmed at its rest' line,
    else the end of the data), the planner's log lines timed on the TAKE-BACK line (= the TAKE_BACK event).
    ball_far True: the ball segment to its far side (the card's words); False: to its centre (the 11 Oct TB6 rows'
    scorer; the same minimum on every TB6 run)."""
    t_tb = r.t('TAKE_BACK')
    out = {'take_back': t_tb, 'give_up': r.t('GIVE_UP'), 'land': r.t('LAND'), 'landed': r.t('LANDED'),
           'abort': r.t('FLEET_ABORT'), 'attach': r.attach()[0], 'contacts': int(len(r.times('CONTACT'))),
           'emergency': bool(r.ev.event.astype(str).str.contains('EMERGENCY').any()
                             or any('EMERGENCY STOP' in ln for ln in r.log))}
    if t_tb is None:
        return out
    df, log, t = r.df, r.log, r.tt
    q4 = f'drone_{r.n}'
    i_tb = next((i for i, ln in enumerate(log) if '[dissipative] TAKE-BACK of' in ln), None)
    tb_ln = log[i_tb] if i_tb is not None else None
    out['take_back_line'] = re.sub(r'^.*\[dissipative\] ', '', tb_ln)[:260] if tb_ln else None
    z = re.search(r'zone ([abc])', tb_ln) if tb_ln else None
    out['zone'] = z.group(1) if z else ('hover' if tb_ln else None)
    off = None
    if tb_ln and _stamp(tb_ln) is not None and 'wall' in df:
        off = _stamp(tb_ln) - float(np.interp(t_tb, df.t, df.wall))

    def to_t(ln):
        return float(np.interp(_stamp(ln) - off, df.wall, df.t))
    later = log[i_tb:] if i_tb is not None else []
    dc = next((ln for ln in later if 'decoupled from the ring frame' in ln), None)
    out['decouple'] = r2(to_t(dc)) if dc and off is not None else None
    out['decouple_line'] = re.sub(r'^.*\[dissipative\] ', '', dc)[:160] if dc else None
    down = [ln for ln in later if q4 in ln and (' is down' in ln or 'disarmed at its rest' in ln) and _stamp(ln)]
    out['down'] = r2(min(to_t(ln) for ln in down)) if down and off is not None else None
    out['down_s'] = round(out['down'] - t_tb, 3) if out['down'] is not None else None
    out['down_within_60s'] = out['down'] is not None and out['down'] - t_tb <= DOWN_WITHIN_S
    t_end = out['down'] if out['down'] is not None else float(t[-1])
    m = (t >= t_tb) & (t <= t_end)
    c = clearance(r, m, ball_far=ball_far)
    out['rod_min'], out['rod_at'], out['rod_carrier'] = _min_at(r, c['rod'], c['rod_who'])
    out['body_min'], out['body_at'], out['body_carrier'] = _min_at(r, c['body'], c['body_who'])
    out['overflight_s'], out['overflight_gap_min'], out['overflight_body_s'] = overflight(r, m)
    s = step(r, r.k + 'tilt_deg', t_tb)
    out['att_step_take_back'] = r2(s)
    if out['decouple'] is not None:
        out['att_step_decouple'] = r2(step(r, r.k + 'tilt_deg', out['decouple']))
    w = df[m]
    out['ring_tilt_max'] = r2(w.payload_tilt_deg.max()) if len(w) else None
    out['tip_z_min'] = r2(w.tip_z.min(), 3) if len(w) and 'tip_z' in w else None
    out['bars'] = tb4_bars(out)
    if tb5 and str(r.man.get('load_traj', '')) == 'orbit':
        out.update(tb5_prediction(r, out))
    return out


def tb4_bars(o):
    """The frozen TB4 bars (card rev 10 §5): confirms / between / FALSIFIES per bar; None where not measured."""
    def bar(v, ok, bad, low=True):
        if v is None:
            return None
        if low:
            return 'confirms' if v >= ok else ('FALSIFIES' if v < bad else 'between')
        return 'confirms' if v <= ok else ('FALSIFIES' if v > bad else 'between')
    return {'rod': bar(o.get('rod_min'), 0.15, 0.10), 'body': bar(o.get('body_min'), 0.35, 0.30),
            'step_take_back': bar(o.get('att_step_take_back'), 2.0, 5.0, low=False),
            'step_decouple': bar(o.get('att_step_decouple'), 2.0, 5.0, low=False),
            'ring_tilt': bar(o.get('ring_tilt_max'), 5.0, 5.0, low=False),
            'down_within_60s': o.get('down_within_60s')}


def tb5_prediction(r, o):
    """TB5's prediction from this run's own latch on its own kinematic formation (tools/tb5_replay; needs the ROS
    environment for the planner module). A logged-vs-predicted gap over 5 cm voids the replay for the gate (F4)."""
    out = {}
    try:
        import tb5_replay as T
        run = T.Run(r.id)
        lap = run.lap
        ph = lambda tt: round(float((run.tau_at(tt) % lap) / lap * 360.0), 1)  # noqa: E731
        out['take_back_phase_deg'] = ph(o['take_back'])
        if o.get('decouple') is not None:
            out['decouple_phase_deg'] = ph(o['decouple'])
        tau, p, v, ball = T.latch_from_log(run, o['take_back'])
        why = 'LAND' if o['land'] is not None and o['land'] <= o['take_back'] + 0.05 else 'his give-up'
        pred = T.replay(run, T.planner_module(), tau, p, v, ball, why=why)
        out.update({'tb5_fit_rms': round(run.fit_rms, 3), 'tb5_zone': pred['zone'], 'tb5_rod_min': pred['rod_min'],
                    'tb5_body_min': pred['body_min'],
                    'tb5_after_decouple_ring_xy_min': pred['after_decouple_ring_xy_min']})
        if o.get('rod_min') is not None:
            out['tb5_gap_rod'] = round(abs(o['rod_min'] - pred['rod_min']), 3)
            out['tb5_gap_body'] = round(abs(o['body_min'] - pred['body_min']), 3)
            out['tb5_void'] = out['tb5_gap_rod'] > 0.05 or out['tb5_gap_body'] > 0.05
    except SystemExit as e:
        out['tb5_error'] = str(e)
    except Exception as e:  # reported, never hidden
        out['tb5_error'] = f'{type(e).__name__}: {e}'
    return out


# ── the tilt cut ─────────────────────────────────────────────────────────────────────────────────────────────────

def injection(r):
    """(kind, time, arg) of the first fault injection: MOCAP_TILT, MOCAP_LOSS or a nonzero WRENCH."""
    best = None
    for kind in INJECTIONS:
        e = r.ev[r.ev.event == kind]
        if kind == 'WRENCH':
            e = e[e.arg.map(is_push)]
        if len(e) and (best is None or float(e.sim_time.iloc[0]) < best[1]):
            best = (kind, float(e.sim_time.iloc[0]), str(e.arg.iloc[0]))
    return best


def tilt(r):
    """The tilt card's readouts (tilt_row) and the injection's own: injection -> TILT_FAULT -> MUX:cut -> drone 3
    disarmed (the first disarmed row from 0.5 s before the cut), his LANDED_DISARMED, the FAULT_LAND count, LAND;
    injection -> FAULT_LAND and -> drone 3 disarmed (MOCAP_LOSS arms, no cut)."""
    inj = injection(r)
    t_inj = inj[1] if inj else None
    tf, cut = r.t('TILT_FAULT'), r.t('MUX:cut')
    armed = r.col(r.k + 'armed')
    t = r.tt

    def first_disarm(after):
        i = np.where((t >= after) & (armed < 0.5))[0]
        return float(t[i[0]]) if len(i) else None
    dis_cut = first_disarm(cut - 0.5) if cut is not None else None
    dis_inj = first_disarm(t_inj) if t_inj is not None else None
    fl = r.times('FAULT_LAND')
    fl_after = r.t('FAULT_LAND', after=t_inj) if t_inj is not None else None
    landed, abort = r.t('LANDED'), r.t('FLEET_ABORT')
    target = None
    if inj and inj[0] in ('MOCAP_TILT', 'MOCAP_LOSS'):
        target = int(float(inj[2].split()[0]))
    elif inj:
        target = wrench_arg(inj[2])[2]
    return {'injection': inj[0] if inj else None, 'injection_t': t_inj, 'injection_arg': inj[2] if inj else None,
            'injection_target': target, 'tilt_fault': tf, 'mux_cut': cut,
            'inj_to_tilt_fault_s': r2(tf - t_inj, 3) if tf is not None and t_inj is not None else None,
            'tilt_fault_to_cut_s': r2(cut - tf, 3) if tf is not None and cut is not None else None,
            'cut_to_d3_disarm_s': r2(dis_cut - cut, 3) if dis_cut is not None and cut is not None else None,
            'landed_disarmed': r.t('PHASE:LANDED_DISARMED') is not None,
            'fault_land_count': int(len(fl)),
            'inj_to_fault_land_s': r2(fl_after - t_inj, 3) if fl_after is not None else None,
            'inj_to_d3_disarm_s': r2(dis_inj - t_inj, 3) if dis_inj is not None else None,
            'take_back': r.t('TAKE_BACK'), 'landed': landed, 'abort': abort,
            'land': ('LANDED' if landed is not None and abort is None
                     else 'FLEET_ABORT' if abort is not None else 'none')}


# ── TB14: the landing tip-over ───────────────────────────────────────────────────────────────────────────────────

def tb14(r):
    """The landing tip-over (card rev 10 §6.4, frozen): QUAD4 welded at LAND; the abort >= 5 s after LAND; its tip on
    the floor; its tilt passing 60 deg within 1 s while its body is above its rest.

    Read on the twin's logs as (each part reported):
      welded at LAND   an attach (timed WELD / REWELD) before LAND, no DETACH between, run.csv's attached flag at LAND
      tip on the floor the ring down (payload z < 0.07 m) with the welded tip still at plate 9 (tip z < 0.10 m), the
                       tilt card rev 4 9.2 thresholds: the welded tip reads 0.05-0.06 m with the ring on the floor
      within 1 s       QUAD4's tilt rises from its last value under 10 deg to past 60 deg in <= 1 s, the crossing after
                       the tip is down and before the abort (+ one row)
      above its rest   body z at the crossing > its first logged z (its rest) + 0.02 m
    tb14_f10 is critic round 4 F10's literal reference (the 60 deg crossing within 1 s after the tip first reaches
    tip z <= 0.03 m), kept beside it: no welded twin tip gets below its plate's height on the floor.

    Also the tilt card's rev 4 landing rule (9.2): QUAD4 passes 30 deg after the attach and the LAND with the ring
    down (payload z < 0.07 m) and the tip at plate 9 (tip z < 0.10 m)."""
    tw, _ = r.attach()
    land, abort = r.t('LAND'), r.t('FLEET_ABORT')
    t = r.tt
    out = {'attach': tw, 'land': land, 'abort': abort, 'welded_at_land': False, 'tb14': False, 'tb14_f10': False,
           'unwind_tipover': False}
    if tw is None or land is None or tw > land:
        return out
    det = r.t('DETACH', after=tw, before=land)
    att = r.col(r.k + 'attached')
    i_land = int(np.searchsorted(t, land))
    flag = bool(att[min(i_land, len(t) - 1)] > 0.5) if np.isfinite(att).any() else True
    out['welded_at_land'] = det is None and flag
    tilt = r.col(r.k + 'tilt_deg')
    z = r.col(r.k + 'z')
    tip_z = r.col('tip_z')
    ring_z = r.col('payload_z')
    rest = float(z[np.isfinite(z)][0]) if np.isfinite(z).any() else np.nan
    after = t >= land
    stop = (abort + 0.1) if abort is not None else float(t[-1])
    # the rev 4 9.2 rule
    tip30 = np.where(after & (t >= tw) & (tilt > UNWIND_TILT_DEG))[0]
    if len(tip30):
        i = tip30[0]
        out['unwind_tilt30_t'] = r2(t[i])
        out['unwind_tipover'] = bool(ring_z[i] < RING_DOWN_Z and tip_z[i] < TIP_AT_PLATE_Z)
    if not out['welded_at_land']:
        return out
    out['abort_after_land_s'] = r2(abort - land) if abort is not None else None
    down = np.where(after & (ring_z < RING_DOWN_Z) & (tip_z < TIP_AT_PLATE_Z))[0]
    out['tip_down_t'] = r2(t[down[0]]) if len(down) else None
    out['tip_z_min_after_land'] = r2(np.nanmin(tip_z[after]), 3) if np.isfinite(tip_z[after]).any() else None
    f10 = np.where(after & (tip_z <= TIP_FLOOR_Z_F10))[0]
    out['tip_floor_f10_t'] = r2(t[f10[0]]) if len(f10) else None
    t0 = t[down[0]] if len(down) else None
    cross = np.where(after & (t <= stop) & (tilt > TB14_TILT_DEG) & ((t >= t0) if t0 is not None else False))[0]
    if len(cross):
        i = cross[0]
        lvl = np.where((t < t[i]) & (tilt < TB14_LEVEL_DEG))[0]
        rise = float(t[i] - t[lvl[-1]]) if len(lvl) else None
        out.update({'tilt60_t': r2(t[i]), 'tilt60_after_tip_down_s': r2(t[i] - t0), 'tilt_rise_s': r2(rise),
                    'body_z_at_60': r2(z[i], 3), 'rest_z': r2(rest, 3)})
        above = bool(z[i] > rest + 0.02)
        abort_ok = abort is not None and abort - land >= TB14_ABORT_AFTER_LAND_S
        out['tb14'] = bool(abort_ok and rise is not None and rise <= TB14_RISE_S and above)
        if len(f10):
            out['tb14_f10'] = bool(abort_ok and above and t[i] >= t[f10[0]] and t[i] - t[f10[0]] <= TB14_RISE_S)
    return out


def land_text(r, tb):
    """The LAND column: LANDED, FLEET_ABORT, none; a TB14 run reads 'tip-over (TB14)' (card §6.4)."""
    if tb.get('tb14'):
        return 'tip-over (TB14)'
    return 'LANDED' if r.t('LANDED') is not None else ('FLEET_ABORT' if r.t('FLEET_ABORT') is not None else 'none')
