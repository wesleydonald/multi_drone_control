"""The flights behind the 23 Oct best-flight review figures, and the numbers every figure reads from them.

A flight is a rig run (results/rig/<day>/<name>_run, made by tools/rig_to_run.py) or a twin run (results/<day>/R####_*).
Rig flights that were never converted (7 Oct) are converted into a cache outside the repo (BEST_CACHE, default the
session scratchpad), so results/ stays read-only. Node logs are put on run.csv's clock: a rig node stamps epoch time
(run t = stamp - the manifest's t0_epoch), a twin node stamps sim time (run t = stamp - t_ready_sim).

Errors follow tools/n2_bands.py's 22 Oct rule (Wesley: the path is scored clock-free, the lag is reported apart):
  path   the ring's sideways distance to the path its reference traces and its height error, RMS from the path's first
         move to the planner's own descent (or LAND); the lag (s) is the time the schedule passed the ring's point.
  hover  the ring's 3-D distance to its reference from 2 s after the reference tops out to LAND.
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import metrics as M  # noqa: E402
import run_logs  # noqa: E402

CACHE = os.environ.get('BEST_CACHE', '/tmp/claude-1000/-home-wesley-multi-drone-control/'
                       'f2a54328-3214-4ac3-989e-f5a0c5208168/scratchpad/best_figs/runs')

# THE FLIGHTS. Every figure reads its flights from here: a re-render on new flights (Wed 28's 1 m rig flights) needs
# only new keys below ('2026-10-28/<name>' for a rig flight under results/rig/, 'R####' for a twin run), then each
# fig_*.py and the contact sheet. (motion, carriers) -> flight key.
# The best rig flight per cell (select_flights.py, candidates.json; Wesley 23 Oct: the spinny line is ls3c, today's
# constant-rate full turn, not the lower-error ls3).
RIG_BEST = {('hover', 3): '2026-10-22/h3', ('hover', 4): '2026-10-07/r20004',
            ('figure-8', 2): '2026-10-22/f2slope', ('figure-8', 3): '2026-10-22/f3h',
            ('figure-8', 4): '2026-10-07/r2008e60',
            ('circle', 3): '2026-10-21/c200_4', ('circle', 4): '2026-10-07/r206e60',
            ('line', 3): '2026-10-21/l200', ('spinny line', 3): '2026-10-22/ls3c',
            ('spin', 3): '2026-10-21/s125', ('spin', 4): '2026-10-07/r000014'}
# the twin (1 m cables) where the rig has no flight
TWIN_FILL = {('hover', 2): 'R2330', ('spinny line', 2): 'R2363', ('spinny line', 4): 'R2357'}
# to be replaced by Wed 28's 1 m rig flights (Wesley 23 Oct): the pair hover and every four-carrier cell
WED28 = {('hover', 2), ('hover', 4), ('figure-8', 4), ('circle', 4), ('spin', 4), ('spinny line', 4)}
# the twin's grid at the rig's path sizes, best of each cell's repeats: the 22 Oct baselines (hover, figure-8 size 1.0,
# spinny line 1 m) and, for three carriers, the 23 Oct 1 m re-fly's cells flown as the rig flew them (circle r 0.5 at
# 0.2 m/s, line 1 m at 0.2 m/s, spin r 0.5 at 0.125 m/s)
TWIN_GRID = {('hover', 2): 'R2330', ('hover', 3): 'R2336', ('hover', 4): 'R2349',
             ('figure-8', 2): 'R2334', ('figure-8', 3): 'R2341', ('figure-8', 4): 'R2353',
             ('spinny line', 2): 'R2363', ('spinny line', 3): 'R2345', ('spinny line', 4): 'R2357',
             ('circle', 3): 'R2528', ('line', 3): 'R2530', ('spin', 3): 'R2512'}
RELEASE = '2026-10-08/r303_4'       # unannounced, 4 -> 3 in hover (the best rig release)
DETACH = '2026-10-08/r302_3'        # announced, 4 -> 3 in hover (Wesley 23 Oct)
# every rig flight that lost a carrier as planned, for fig_reaction: (key, row label)
RELEASES = (('2026-10-08/r303_4', 'release, hover'), ('2026-10-08/r304_3', r'release, 150$^\circ$ gap'),
            ('2026-10-08/r305', 'release, circle'), (DETACH, 'announced detach'))
YAW_OLD_LAW = {'2026-10-22/ls3'}    # line_spin flown before 22 Oct's constant-rate law (tools/yaw_track.py --old-law)
G = 9.81
MOTION = {'hover': 'hover', 'circle': 'circle', 'orbit': 'circle', 'spin': 'spin', 'fig_8': 'figure-8',
          'line_x': 'line', 'line_spin': 'spinny line'}


class Flight:
    def __init__(self, key):
        """key: 'R2434' (twin) or '2026-10-22/f3h' (rig)."""
        self.key = key
        if key.startswith('R') and key[1:].isdigit():
            hit = sorted(glob.glob(os.path.join(REPO, 'results', '*', f'{key}_*')))
            if not hit:
                raise FileNotFoundError(key)
            self.source, self.run_dir, self.logs = 'twin', hit[0], hit[0]
            man = json.load(open(os.path.join(hit[0], 'manifest.json')))
            self.t_origin = float(man.get('t_ready_sim') or 0.0)
            self.name = key
        else:
            base = os.path.join(REPO, 'results', 'rig', key)
            self.source, self.logs, self.name = 'rig', base + '_logs', os.path.basename(key)
            run = base + '_run'
            if not os.path.exists(os.path.join(run, 'logs', 'run.csv')):
                run = os.path.join(CACHE, self.name + '_run')
                if not os.path.exists(os.path.join(run, 'logs', 'run.csv')):
                    convert(base, run)
            self.run_dir = run
            self.t_origin = float(json.load(open(os.path.join(run, 'manifest.json')))['t0_epoch'])
        pc = run_logs.planner_csvs(self.logs)
        self.planner_csv = max(pc, key=os.path.getsize) if pc else None
        self.params = (json.load(open(os.path.join(os.path.dirname(self.planner_csv), 'params.json')))
                       if self.planner_csv and os.path.exists(os.path.join(os.path.dirname(self.planner_csv),
                                                                            'params.json')) else {})
        self.kind = str(self.params.get('load_traj') or 'hover')
        self.motion = MOTION.get(self.kind, self.kind)
        self.speed = float(self.params.get('traj_speed') or 0.0)
        self._run = self._events = None

    # ── the recorded data ────────────────────────────────────────────────────
    def run(self):
        if self._run is None:
            self._run = pd.read_csv(os.path.join(self.run_dir, 'logs', 'run.csv')).drop_duplicates('t') \
                .reset_index(drop=True)
        return self._run

    @property
    def n(self):
        d = self.run()
        return sum(1 for i in range(12) if f'd{i}_x' in d.columns)

    def events(self):
        if self._events is None:
            self._events = [(float(e['sim_time']), e['event'], e.get('arg', '') or '')
                            for e in csv.DictReader(open(os.path.join(self.run_dir, 'logs', 'events.csv')))]
        return self._events

    def t_event(self, name, after=-math.inf):
        return next((t for t, e, _ in sorted(self.events()) if e == name and t > after), None)

    def planner(self):
        """The planner's log on run.csv's clock (column t)."""
        p = pd.read_csv(self.planner_csv)
        p['t'] = p.sim_time - self.t_origin
        return p

    def trackers(self):
        """{drone: tracker log on run.csv's clock (column t)}."""
        out = {}
        for i, f in sorted(run_logs.trackers(self.logs).items()):
            d = pd.read_csv(f)
            d['t'] = d.sim_time - self.t_origin
            out[i] = d.sort_values('t').reset_index(drop=True)
        return out

    def size(self):
        """The path's size as typed (figure-8 half-width, circle radius, line length), m."""
        if self.kind == 'fig_8':
            return float(self.params.get('fig8_size') or 0.0) or float(self.params.get('traj_radius') or 0.0)
        if self.kind in ('line_x', 'line_spin'):
            return float(self.params.get('traj_distance') or 0.0)
        return float(self.params.get('traj_radius') or 0.0)

    def cable(self):
        return float(self.params.get('cable_len') or 0.0)

    def describe(self):
        if self.motion == 'hover':
            return 'hover'
        unit = {'figure-8': 'size', 'line': 'length', 'spinny line': 'length'}.get(self.motion, 'radius')
        return f'{self.motion}, {unit} {self.size():.2g} m at {self.speed:.3g} m/s'


def convert(base, out):
    import rig_to_run
    trackers = sorted(glob.glob(os.path.join(base + '_logs', 'logs', 'tracker', 'drone*')))
    rig_to_run.convert(trackers, out, bag=base if os.path.isdir(base) else None,
                       t2_log=base + '.log' if os.path.exists(base + '.log') else None)


# ── the error parts ──────────────────────────────────────────────────────────
def hover_parts(f):
    """n2_bands.path_metrics' hover window and error, with its horizontal and height parts (m)."""
    d = f.run()
    t = d.t.to_numpy()
    zr = d.payload_ref_z.to_numpy()
    land = f.t_event('LAND')
    top = t[np.argmax(zr >= np.nanmax(zr) - 1e-3)] + 2.0
    w = (t >= top) & (t <= (land if land is not None else t[-1]))
    e = d[['payload_x', 'payload_y', 'payload_z']].to_numpy() - d[['payload_ref_x', 'payload_ref_y',
                                                                    'payload_ref_z']].to_numpy()
    ok = w & np.all(np.isfinite(e), axis=1)
    r = lambda x: float(np.sqrt(np.mean(x[ok] ** 2)))  # noqa: E731
    return dict(t=t, w=ok, ex=e[:, 0], ey=e[:, 1], ez=e[:, 2], exy=np.hypot(e[:, 0], e[:, 1]),
                err=float(np.sqrt(np.mean(np.sum(e[ok] ** 2, axis=1)))), xy=r(np.hypot(e[:, 0], e[:, 1])),
                z=r(e[:, 2]), z_mean=float(np.mean(e[ok, 2])), window=(float(t[ok][0]), float(t[ok][-1])),
                tilt_peak=float(np.nanmax(d.payload_tilt_deg.to_numpy()[ok])))


def path_window(f):
    """n2_bands.path_metrics_clock_free's window: the path's first moving sample to the planner's own descent
    (or LAND if earlier)."""
    d = f.run()
    t = d.t.to_numpy()
    des = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
    sweep = M.sweep_window(t, des)
    if not sweep.any():
        return None, sweep
    land = f.t_event('LAND')
    t0 = t[sweep][0]
    zr = d.payload_ref_z.to_numpy()
    i0 = int(np.searchsorted(t, t0))
    down = np.where(zr[i0:] < np.nanmax(zr[i0:]) - 0.02)[0]
    t1 = min(t[i0 + down[0]] if len(down) else t[-1], land if land is not None else t[-1])
    return (t >= t0) & (t <= t1), sweep


def legs(t, des, w):
    """The window split where the path turns back on itself (a line's turnaround), as boolean masks."""
    idx = np.where(w)[0]
    tk, xk = M.schedule_knots(t, des, w)
    if tk.size < 3:
        return [w]
    dk = np.diff(xk, axis=0)
    dk = dk / np.maximum(np.linalg.norm(dk, axis=1), 1e-12)[:, None]
    rev = np.where(np.sum(dk[1:] * dk[:-1], axis=1) < -0.5)[0]
    if not rev.size:
        return [w]
    cuts = [tk[0] - 1.0] + [tk[i + 1] for i in rev] + [tk[-1] + 1.0]
    return [w & (t >= a) & (t < b) for a, b in zip(cuts[:-1], cuts[1:]) if np.count_nonzero(w & (t >= a) & (t < b))
            > 50] if idx.size else [w]


def path_parts(f):
    """The clock-free error and its parts over the scored window:
    across (signed sideways distance to the path, + inside the curve), dz (height error), err = RMS of
    sqrt(across^2 + dz^2) (the n2_bands number), and the lag on the schedule (s; a line per leg)."""
    d = f.run()
    t = d.t.to_numpy()
    des = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
    w, sweep = path_window(f)
    tk, xk = M.schedule_knots(t, des, sweep)
    path = M.path_through(xk)
    s_sched = np.interp(t, tk, path[5])
    xy = d[['payload_x', 'payload_y']].to_numpy()
    across, s_at, _ = M.project_on_path(path, xy, np.where(w, s_sched, np.nan))
    dz = d.payload_z.to_numpy() - d.payload_ref_z.to_numpy()
    e = np.sqrt(across ** 2 + dz ** 2)
    ok = w & np.isfinite(e)
    rms = lambda x: float(np.sqrt(np.mean(x[ok] ** 2)))  # noqa: E731
    lag, lag_sd, n_lag = [], [], []
    for leg in legs(t, des, w):
        c = M.contour_lag(t, xy, des, mask=leg)
        if c and math.isfinite(c.get('lag_s', math.nan)):
            lag.append(c['lag_s'])
            lag_sd.append(c['lag_sd_s'])
            n_lag.append(c['n'])
    lag_s = float(np.average(lag, weights=n_lag)) if lag else math.nan
    tilt = d.payload_tilt_deg.to_numpy()
    return dict(t=t, w=ok, across=across, dz=dz, e=e, s_at=s_at, s_sched=s_sched, path=path,
                err=float(np.sqrt(np.mean(e[ok] ** 2))), across_rms=rms(across), dz_rms=rms(dz),
                dz_mean=float(np.mean(dz[ok])), across_max=float(np.nanmax(np.abs(across[ok]))),
                lag_s=lag_s, window=(float(t[ok][0]), float(t[ok][-1])),
                tilt_peak=float(np.nanmax(tilt[ok])), tilt_mean=float(np.nanmean(tilt[ok])))


def score(f):
    """The one tracking number Wesley ranks flights by: clock-free on a path, distance to the hold point in a hover."""
    return hover_parts(f) if f.motion == 'hover' else path_parts(f)


def path_speed(f):
    """(mean, peak) speed of the published reference along its path over the schedule, m/s: what the ring was asked
    to fly (7 Oct's figure-8 typed 0.25 m/s but took 61 s for its 6.1 m)."""
    d = f.run()
    t = d.t.to_numpy()
    des = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
    tk, xk = M.schedule_knots(t, des, M.sweep_window(t, des))
    sk = M.path_through(xk)[5]
    v = np.diff(sk) / np.maximum(np.diff(tk), 1e-6)
    return float(sk[-1] / (tk[-1] - tk[0])), float(np.percentile(v[np.diff(tk) <= M.HOLD_GAP_S], 99))
