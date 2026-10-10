#!/usr/bin/env python3
"""tools/thesis_rig_twin_figures.py -- thesis figures that set the rig against its twin (thesis-figure skill: print size,
Computer Modern, no run ids in the figure or caption; the runs are in configs/thesis_figures.yaml and
docs/figures/PROVENANCE.md).

    python3 tools/thesis_rig_twin_figures.py circle      # one figure
    python3 tools/thesis_rig_twin_figures.py             # all three

Parts flown after 10 Oct are PENDING: their run lists below are empty (or None) and the figure draws a marked
placeholder. Adding the run directory to the list is the only change needed once the run exists; a rig run must
have been converted first (tools/rig_summary.py <path> does it, and rig_flight.sh runs it after each flight).

F_rig_circle_detach -- "During the circle on the rig, a UAV whose magnet was switched off without telling the planner
was found 0.34 s later; after a 1.5 s hold of the path the three survivors carried the ring on round the circle with
the tilt peaking at 8.1 deg (twin 6.0-7.3) and the ring dipping 3.9 cm (twin 2.1-2.5), while the freed UAV stepped out,
cleared the circle and landed by itself."
  hero: the plan view (a): the circle, the ring's path, the fleet at three numbered instants (release, the freed UAV
  starting its own landing, LAND; carriers darker for later instants) and the freed UAV's exit in its own colour.
  Support (b-d), t from the release: planned tension per UAV labelled by plate (the freed UAV dashed, its colour),
  ring tilt (with the rig's largest tilt away from the release dashed) and ring height error, rig gold against the
  twin's two runs (grey band and mean). Left out: the twin in the plan view (its circle starts elsewhere, so its
  paths do not overlay), the UAVs' heights, the hover before the circle, the landing (F_departed_landing). Window:
  the plan view from 2.5 s before the release to LAND; the strips from 2.5 s before to 8 s after (critic 10 Oct: the
  release and the detection must be readable).

F_twin_rig_parity -- "The twin predicts the tilt rise and the detection delay of the unannounced detaches on the rig
to within about a third, but under-predicts the ring's dip by up to 2.5 times and its horizontal error by 2-3 times."
  hero: four small parity panels on log axes, x the twin (mean of two runs, bar: their range), y the rig (one run);
  y = x dashed, a +-25 % band, y = 2x dotted; one marker shape per flight. The attach (the weld of R8b, rig flights
  on 14 Oct) is a hollow pentagon on y = x at the twin's value until the rig runs are listed. Left out: carry-only
  flights (no twin flown on their configuration). Metrics (one function for rig and twin): tilt rise = peak in the
  3 s after the release minus the mean over the 3 s before; dip = lowest ring height error in the 3 s after;
  detection delay = the planner's "UNANNOUNCED DETACH" line minus the release; horizontal error = mean distance of
  the ring from its reference over the 10 s before the release.

F_disarm_lanes -- "When a UAV off the ring fails, only it is disarmed and the carriers land the ring (the old rule
disarmed all four in the air); a freed carrier is disarmed alone once it is down at its rest, while the others carry
on until LAND and are disarmed together when the ring is down."
  hero: one lane per UAV, state by shade and bar height (dark: carrying; mid or its own colour: flying free; light
  and thin: landing), the UAV off the ring in its own colour once off, carriers grey; the disarm marked (cross: in
  the air, bar: at rest), the repeat run's times in brackets. (a) a fault on the newcomer in transit (its motion
  capture lost), old rule against newcomer_fault_land; (b) a freed carrier in hover and in the circle, twin against
  the rig (pending: 14 Oct w4, w5). Left out: heights (one number in a label), the ring, the approach before the
  fault. Window: from shortly before the fault or release to the last disarm; zero at the same place on both axes.
"""
import csv
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Rectangle  # noqa: E402

sys.dont_write_bytecode = True
TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import plot_style as ps  # noqa: E402

REPO = os.path.dirname(TOOLS)
OUT = os.path.join(REPO, 'docs', 'figures')

INK = '0.1'
GREY_TEXT = '0.3'
TWIN_FILL, TWIN_LINE = '0.75', '0.35'

# ── runs (rig: results/rig/<day>/<name>, converted to <name>_run; twin: R####) ─────────────────────────────────────

# 8 Oct rig r305: four UAVs on plates 3/11/7/9 (30/90/150/270 deg), the 0.5 m circle at 0.125 m/s, the UAV on plate 9
# (90 deg) released unannounced ~21 s into the circle; twin: the same configuration (w4_3915_release_orbit_drop)
CIRCLE = dict(rig='results/rig/2026-10-08/r305', twin=('R1208', 'R1213'))

# the 8 Oct rig detaches and their twins (the same configurations, two runs each); event = the release
PARITY = [
    ('hover, announced', 'o', 'results/rig/2026-10-08/r302_3', ('R1207', 'R1212'), 'DETACH'),
    ('hover, unannounced', 's', 'results/rig/2026-10-08/r303_4', ('R1140', 'R1145'), 'RELEASE'),
    ('hover, unannounced, 150$^\\circ$ gap', '^', 'results/rig/2026-10-08/r304_3', ('R1206', 'R1211'), 'RELEASE'),
    ('circle path, unannounced', 'D', 'results/rig/2026-10-08/r305', ('R1208', 'R1213'), 'RELEASE'),
]
# R8b, QUAD4's weld on plate 9 with the measured-tip approach: twin twin_tf_weld_on (rev 5, two runs).
# PENDING: Wednesday 14 Oct r8b (x2-3), e.g. 'results/rig/2026-10-14/r8b', 'results/rig/2026-10-14/r8b_2'
PARITY_ATTACH = dict(twin=('R1404', 'R1410'), rig=[])

# who disarms when. fault: QUAD4 (/drone_3) loses its motion capture 2 s into its transit (twin_fs_c0, newcomer_fault_land
# off: the old rule; twin_fs_c1 on the rig candidate tree). freed: a carrier released unannounced, the departed landing
# with the alone-disarm (hover: w4_da_release_hover; circle: w4_gi_g2_release_orbit). PENDING: the rig rows, Wednesday
# 14 Oct w4 (hover) and w5 (circle), e.g. ['results/rig/2026-10-14/w4'].
LANES_FAULT = [('old rule, twin', ['R1258']), ('new rule, twin', ['R1284', 'R1285'])]
LANES_FREED = [('hover, twin', ['R1310', 'R1325']), ('hover, rig', []),
               ('circle, twin', ['R1309', 'R1326']), ('circle, rig', [])]
PENDING_TEXT = 'rig, 14 October: pending'


# ── loading: one reader for a rig run and a twin run ───────────────────────────────────────────────────────────────

class Run:
    """A run on the run.csv clock: run.csv, events.csv, the planner log and the console log lines, rig or twin."""

    def __init__(self, spec):
        if re.fullmatch(r'R\d{4}', spec):
            hits = glob.glob(os.path.join(REPO, 'results', '*', f'{spec}_*'))
            if not hits:
                raise SystemExit(f'no twin run {spec}')
            self.dir, self.kind = hits[0], 'twin'
        else:
            base = os.path.join(REPO, spec) if not os.path.isabs(spec) else spec
            self.dir, self.kind = base.rstrip('/') + '_run', 'rig'
            if not os.path.exists(os.path.join(self.dir, 'logs', 'run.csv')):
                raise SystemExit(f'{spec}: no converted run; run  python3 tools/rig_summary.py {spec}  first')
        self.name = spec
        self.run = pd.read_csv(os.path.join(self.dir, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
        self.events = [(float(e['sim_time']), e['event'], e['arg'] or '')
                       for e in csv.DictReader(open(os.path.join(self.dir, 'logs', 'events.csv')))]
        self.man = json.load(open(os.path.join(self.dir, 'manifest.json')))
        if self.kind == 'rig':
            self.logs = os.path.dirname(os.path.dirname(self.man['trackers'][0]))
            self.console = open(self.man['t2_log'], errors='replace').read()
        else:
            self.logs = os.path.join(self.dir, 'logs')
            self.console = open(os.path.join(self.dir, 'logs', 'launch.log'), errors='replace').read()
        self._off = None

    def ev(self, name, after=-np.inf):
        """First time of an event at or after `after` (None if it never happened)."""
        return next((t for t, e, _ in self.events if e == name and t >= after), None)

    def ev_arg(self, name):
        return next((a for _, e, a in self.events if e == name), '')

    def _wall_offset(self):
        """Twin: console stamps are wall clock, run.csv counts sim time. The planner logs each fleet command the runner
        sends; the run.csv row at that event gives its wall time since the start (rig: everything is wall clock)."""
        if self._off is None:
            r = self.run
            offs = []
            for m in re.finditer(r"\[(\d{9,}\.\d+)\] \[\w+\]: \[planner\] /fleet/command received: '(\w+)'", self.console):
                t_ev = self.ev(m.group(2))
                if t_ev is not None and not any(n == m.group(2) for n, _ in offs):
                    offs.append((m.group(2), float(m.group(1)) - float(np.interp(t_ev, r.t, r.wall))))
            if not offs:
                raise SystemExit(f'{self.name}: no fleet command in the console log to align its clock')
            med = float(np.median([o for _, o in offs]))
            near = [o for _, o in offs if abs(o - med) <= 0.1]
            if len(near) * 2 <= len(offs) and len(offs) > 1:
                raise SystemExit(f'{self.name}: console clock anchors disagree: {offs}')
            self._off = float(np.mean(near))
        return self._off

    def log_times(self, pattern):
        """Run-clock times of every console line matching `pattern` (group 1 must be the stamp), with its match."""
        out = []
        for m in re.finditer(pattern, self.console):
            stamp = float(m.group(1))
            if self.kind == 'rig':
                t = stamp - self.man['t0_epoch']
            else:
                t = float(np.interp(stamp - self._wall_offset(), self.run.wall, self.run.t))
            out.append((t, m))
        return out

    def log_time(self, pattern, after=-np.inf):
        return next((t for t, _ in self.log_times(pattern) if t >= after), None)

    def planner(self):
        f = sorted(glob.glob(os.path.join(self.logs, '*planner', '*', 'log.csv')), key=os.path.getsize)[-1]
        L = pd.read_csv(f)
        if self.kind == 'rig':
            L['t'] = L.sim_time - self.man['t0_epoch']
        else:                       # node clock: aligned on the ring's lift-off, as tools/detach_tensions.py does
            lift = lambda t, z: float(np.asarray(t)[np.asarray(z) > np.median(np.asarray(z)[:20]) + 0.03][0])  # noqa: E731
            L['t'] = L.sim_time - (lift(L.sim_time, L.load_z) - lift(self.run.t, self.run.payload_z))
        return L

    def params(self):
        f = sorted(glob.glob(os.path.join(self.logs, '*planner', '*', 'params.json')), key=os.path.getsize)[-1]
        return json.load(open(f))


def released_drone(run):
    """Index of the UAV let go: the rig names it 'drone 4 (/drone_3)', the twin's runner gives the index."""
    a = run.ev_arg('RELEASE') or run.ev_arg('DETACH')
    m = re.search(r'/drone_(\d+)', a)
    return int(m.group(1)) if m else int(a)


# ── F_rig_circle_detach ────────────────────────────────────────────────────────────────────────────────────────────

def rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def badge(ax, xy, n, dx=0, dy=0, coords='data', leader=False):
    ax.annotate(rf'\textbf{{{n}}}', xy, xycoords=coords, xytext=(dx, dy), textcoords='offset points', ha='center',
                va='center', fontsize=7, color='white', zorder=20, bbox=dict(boxstyle='circle,pad=0.16', fc=INK, ec='none'),
                arrowprops=dict(arrowstyle='-', lw=0.4, color=INK, shrinkA=0, shrinkB=1.5) if leader else None)


def arrow(ax, p0, p1, colour, zorder=4, head=(3.4, 1.9)):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle='-|>,head_length=%g,head_width=%g' % head, lw=0, color=colour,
                                 zorder=zorder, mutation_scale=1, shrinkA=0, shrinkB=0))


def twin_band(ax, runs, t_of, y_of, grid):
    ys = np.array([np.interp(grid, t_of(r), y_of(r)) for r in runs])
    ax.fill_between(grid, ys.min(0), ys.max(0), color=TWIN_FILL, lw=0, zorder=1)
    ax.plot(grid, ys.mean(0), color=TWIN_LINE, lw=0.6, zorder=1)
    return ys


def fig_circle():
    ps.thesis()
    rig = Run(CIRCLE['rig'])
    twins = [Run(r) for r in CIRCLE['twin']]
    par = rig.params()
    r_ring = float(par['attach_radius'])
    az = [float(a) for a in str(par['attach_azimuths_deg']).split(',')]
    leaver = released_drone(rig)
    t_rel = rig.ev('RELEASE')
    t_det = rig.ev('DETECTED', t_rel)
    t_land = rig.ev('LAND', t_rel)
    pat_d = rf'\[(\d{{9,}}\.\d+)\] \[\w+\]: \[dissipative\] departed drone \d+ \(/drone_{leaver}\) lands now'
    t_down = rig.log_time(pat_d, t_rel)
    hold_s = float(re.search(r'trajectory held ([0-9.]+) s', rig.console).group(1))
    d = rig.run
    t = d.t.to_numpy()
    # the path hold, as the reference shows it: from the detection to the reference's first move after it
    ref_t = t[np.r_[True, np.abs(np.diff(d[['payload_ref_x', 'payload_ref_y']].to_numpy(), axis=0)).sum(1) > 1e-7]]
    t_hold0, t_hold1 = t_det, float(ref_t[ref_t > t_det][0])
    L = rig.planner()
    L = L[L.phase == 'planner']
    lt = L.t.to_numpy()
    # the three instants: the release, the freed UAV starting its own landing, LAND
    t_snap = [t_rel, t_down, t_land]
    k_snap = [int(np.searchsorted(t, ts)) for ts in t_snap]
    # the strips: the release and its recovery (instant 3, LAND, is in the plan view only)
    T0, T1 = -2.5, 8.0
    stats = dict(found_s=t_det - t_rel, hold_s=hold_s, held_from=t_hold0 - t_rel, held_to=t_hold1 - t_rel,
                 lands_now=t_down - t_rel, land=t_land - t_rel)

    W, H = ps.size('full', height=3.75)
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.3], height_ratios=[1.0, 1.15, 1.0])
    axp = fig.add_subplot(gs[:, 0])
    axT = fig.add_subplot(gs[0, 1])
    axt = fig.add_subplot(gs[1, 1], sharex=axT)
    axz = fig.add_subplot(gs[2, 1], sharex=axT)
    q4 = ps.drone_colour(leaver)
    gold = ps.PAYLOAD_LINE
    carrier = ('0.62', '0.42', '0.15')              # the carriers at instants 1, 2, 3: later is darker

    # (a) plan view ----------------------------------------------------------------------------------------------
    ref = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
    fly = (t >= rig.ev('TAKEOFF')) & (t <= t_land)
    moving = np.r_[False, np.hypot(*np.diff(ref, axis=0).T) > 1e-6] & fly
    # the circle's centre: a least-squares circle through the moving reference
    A = np.column_stack([2 * ref[moving], np.ones(moving.sum())])
    c0 = np.linalg.lstsq(A, (ref[moving] ** 2).sum(1), rcond=None)[0][:2]
    ang = np.linspace(0, 2 * np.pi, 361)
    rad = float(par['traj_radius'])
    axp.plot(c0[0] + rad * np.cos(ang), c0[1] + rad * np.sin(ang), color='0.45', lw=0.7, ls=(0, (3.5, 2.2)), zorder=1)
    axp.plot(*c0, '+', color='0.45', ms=4, mew=0.6, zorder=2)
    w = (t >= t_rel + T0) & (t <= t_land)
    P = d[['payload_x', 'payload_y', 'payload_z']].to_numpy()
    axp.plot(P[w, 0], P[w, 1], color=gold, lw=1.0, zorder=3)
    k_start = int(np.flatnonzero(w)[0])
    axp.plot(P[k_start, 0], P[k_start, 1], 'o', ms=3.2, mfc='white', mec=gold, mew=0.8, zorder=6)
    ex = (t >= t_rel) & (t <= t_land)
    fx, fy = d[f'd{leaver}_x'].to_numpy(), d[f'd{leaver}_y'].to_numpy()
    axp.plot(fx[ex], fy[ex], color=q4, lw=1.1, zorder=5)
    ke = np.flatnonzero(ex)
    kk = ke[int(len(ke) * 0.22)]
    arrow(axp, (fx[kk], fy[kk]), (fx[kk + 6], fy[kk + 6]), q4, zorder=6)
    # direction of travel, on the top of the circle
    sgn = np.sign(np.cross(ref[moving][0] - c0, ref[moving][40] - ref[moving][0])) or 1.0
    aa = np.pi / 2
    p0 = c0 + rad * np.array([np.cos(aa), np.sin(aa)])
    p1 = c0 + rad * np.array([np.cos(aa + sgn * 0.06), np.sin(aa + sgn * 0.06)])
    arrow(axp, tuple(p0), tuple(p1), '0.45', zorder=2)
    # each UAV's plate (its rim anchor in the ring frame) from the slot map before the release: survivors keep theirs
    slots0 = [int(x) for x in str(L.slot2drone.iloc[np.searchsorted(lt, t_rel) - 1]).split()]
    plate = {i: np.radians(az[sl]) for sl, i in enumerate(slots0)}
    stats['plates_deg'] = {i: round(float(np.degrees(v)), 0) for i, v in plate.items()}
    for j, k in enumerate(k_snap):
        Rq = rot(d.loc[k, ['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float))
        ring = (r_ring * np.column_stack([np.cos(ang), np.sin(ang), 0 * ang])) @ Rq.T + P[k]
        axp.plot(ring[:, 0], ring[:, 1], color=gold, lw=0.6, zorder=4)
        for i in range(4):
            dp = np.array([d.loc[k, f'd{i}_x'], d.loc[k, f'd{i}_y']])
            col = q4 if i == leaver else carrier[j]
            if not (i == leaver and j > 0):
                anc = Rq @ (r_ring * np.array([np.cos(plate[i]), np.sin(plate[i]), 0.0])) + P[k]
                axp.plot([anc[0], dp[0]], [anc[1], dp[1]], color=col, lw=0.7, zorder=4)
            axp.plot(*dp, 'o', ms=3.4, mfc=col if i == leaver else 'white', mec=col, mew=0.9, zorder=7)
        badge(axp, tuple(P[k, :2]), j + 1)
        badge(axp, (d.loc[k, f'd{leaver}_x'], d.loc[k, f'd{leaver}_y']), j + 1, dx=(-11, 10, 11)[j],
              dy=(4, -6, 3)[j], leader=True)
    # how far the freed UAV cleared the circle: an arc of that radius about the circle's centre, through it at LAND
    r_ex = float(np.hypot(fx[ke[-1]] - c0[0], fy[ke[-1]] - c0[1]))
    a_ex = float(np.arctan2(fy[ke[-1]] - c0[1], fx[ke[-1]] - c0[0]))
    arc = np.linspace(a_ex - 0.30, a_ex + 0.12, 60)
    axp.plot(c0[0] + r_ex * np.cos(arc), c0[1] + r_ex * np.sin(arc), color='0.45', lw=0.6, ls=(0, (1, 1.5)), zorder=2)
    ea = c0 + r_ex * np.array([np.cos(arc[0]), np.sin(arc[0])])
    axp.annotate(f'{r_ex:.1f}\\,m from the centre', tuple(ea), xytext=(3, 0), textcoords='offset points', ha='left',
                 va='center', fontsize=8, color=GREY_TEXT)
    stats['exit_from_centre_at_land'] = r_ex
    stats['freed_from_centre_at_2'] = float(np.hypot(fx[k_snap[1]] - c0[0], fy[k_snap[1]] - c0[1]))
    km = ke[int(np.argmin(np.abs(fy[ke] - (fy[ke[0]] + 0.45 * (fy[ke].max() - fy[ke[0]])))))]
    axp.annotate('freed UAV', (fx[km], fy[km]), xytext=(6, 0), textcoords='offset points', ha='left', va='center',
                 fontsize=8, color=q4)
    xs = np.r_[P[w, 0], fx[ex], c0[0] + rad * np.cos(ang)]
    ys = np.r_[P[w, 1], fy[ex], c0[1] + rad * np.sin(ang), c0[1] + r_ex * np.sin(arc)]
    for k in k_snap:
        xs = np.r_[xs, [d.loc[k, f'd{i}_x'] for i in range(4)]]
        ys = np.r_[ys, [d.loc[k, f'd{i}_y'] for i in range(4)]]
    span = max(np.ptp(xs), np.ptp(ys))
    axp.set_xlim(xs.min() - 0.05 * span, xs.max() + 0.05 * span)
    axp.set_ylim(ys.min() - 0.04 * span, ys.max() + 0.05 * span)
    axp.set_aspect('equal', adjustable='box')
    axp.xaxis.set_major_locator(plt.MultipleLocator(0.5))
    axp.yaxis.set_major_locator(plt.MultipleLocator(0.5))
    fmt = plt.FuncFormatter(lambda v, _: f'${v:g}$')
    axp.xaxis.set_major_formatter(fmt)
    axp.yaxis.set_major_formatter(fmt)
    axp.set_xlabel('$x$ (m)')
    axp.set_ylabel('$y$ (m)')

    # (b) planned tension per UAV ------------------------------------------------------------------------------------
    Lw = L[(lt >= t_rel + T0) & (lt <= t_rel + T1)]
    tt = Lw.t.to_numpy() - t_rel
    ten = {i: np.full(len(Lw), np.nan) for i in range(4)}
    for k, (s2d, row) in enumerate(zip(Lw.slot2drone.astype(str), Lw.itertuples())):
        for sl, i in enumerate(int(x) for x in s2d.split()):
            ten[i][k] = getattr(row, f't{sl}')
    for i, y in ten.items():
        if i == leaver:
            kl = np.flatnonzero(np.isfinite(y))[-1]
            axT.plot(np.r_[tt[:kl + 1], tt[kl + 1]], np.r_[y[:kl + 1], 0.0], color=q4, lw=1.1, ls='--',
                     drawstyle='steps-post', zorder=3)
            axT.plot(tt[kl + 1], 0.0, 'x', color=q4, ms=4, mew=1.1, zorder=4, clip_on=False)
            stats['leaver_before_N'] = float(np.nanmean(y[tt < 0]))
            stats['leaver_dropped_at'] = float(tt[kl + 1])
        else:
            axT.plot(tt, y, color='0.25', lw=0.8, drawstyle='steps-post', zorder=2)
    surv = [i for i in ten if i != leaver]
    pre = tt < 0
    post = tt > stats['leaver_dropped_at'] + 1.0
    before = {i: float(np.nanmean(ten[i][pre])) for i in surv}
    stats['survivors_before_N'] = sorted(round(v, 2) for v in before.values())
    stats['survivors_after_N'] = sorted(round(float(np.nanmean(ten[i][post])), 2) for i in surv)
    # the survivors by plate, before the release: two share a level
    groups = []
    for i in sorted(surv, key=lambda i: -before[i]):
        g = next((g for g in groups if abs(g[0] - before[i]) < 0.15), None)
        if g:
            g[1].append(i)
        else:
            groups.append([before[i], [i]])
    for v, ids in groups:
        names = ', '.join(f'{stats["plates_deg"][i]:.0f}' + r'$^\circ$' for i in sorted(ids, key=lambda i: stats['plates_deg'][i]))
        axT.annotate(names, (T0, v), xytext=(1, 2), textcoords='offset points', ha='left',
                     va='bottom', fontsize=8, color='0.3')
    axT.annotate(f'{stats["plates_deg"][leaver]:.0f}$^\\circ$, freed', (T0, stats['leaver_before_N']), xytext=(1, -2),
                 textcoords='offset points', ha='left', va='top', fontsize=8, color=q4)
    axT.set_ylabel('Tension (N)')
    axT.set_ylim(0, 4.5)
    axT.set_yticks([0, 2, 4])

    # (c) tilt, (d) height error: rig gold, twin grey ----------------------------------------------------------------
    grid = np.arange(T0, T1, 0.02)
    tw_rel = [r.ev('RELEASE') for r in twins]
    twin_band(axt, twins, lambda r: r.run.t.to_numpy() - r.ev('RELEASE'),
              lambda r: r.run.payload_tilt_deg.to_numpy(), grid)
    twin_band(axz, twins, lambda r: r.run.t.to_numpy() - r.ev('RELEASE'),
              lambda r: 100 * (r.run.payload_z - r.run.payload_ref_z).to_numpy(), grid)
    tr = t - t_rel
    wr = (tr >= T0) & (tr <= T1)
    tilt = d.payload_tilt_deg.to_numpy()
    dz = 100 * (d.payload_z - d.payload_ref_z).to_numpy()
    axt.plot(tr[wr], tilt[wr], color=gold, lw=1.2, zorder=3)
    axz.plot(tr[wr], dz[wr], color=gold, lw=1.2, zorder=3)
    after = (tr >= 0) & (tr <= 3.0)
    kp = int(np.flatnonzero(after)[np.nanargmax(tilt[after])])
    kd = int(np.flatnonzero(after)[np.nanargmin(dz[after])])
    stats['tilt_peak'] = float(tilt[kp])
    stats['tilt_peak_t'] = float(tr[kp])
    early = (tr >= 0) & (tr < tr[kp] - 0.3)
    stats['tilt_first_spike'] = (float(tr[early][np.nanargmax(tilt[early])]), float(np.nanmax(tilt[early])))
    tpk = []
    for r, tr0 in zip(twins, tw_rel):
        m = (r.run.t >= tr0) & (r.run.t <= tr0 + 3)
        k = int(np.nanargmax(r.run.payload_tilt_deg[m].to_numpy()))
        tpk.append((float(r.run.t[m].iloc[k] - tr0), float(r.run.payload_tilt_deg[m].iloc[k])))
    stats['twin_tilt_peak'] = [v for _, v in tpk]
    stats['twin_tilt_peak_t'] = [t_ for t_, _ in tpk]
    stats['dip_cm'] = float(dz[kd])
    stats['dip_t'] = float(tr[kd])
    stats['twin_dip_cm'] = [float(100 * np.nanmin((r.run.payload_z - r.run.payload_ref_z)[(r.run.t >= tr0) & (r.run.t <= tr0 + 3)]))
                            for r, tr0 in zip(twins, tw_rel)]
    outside = wr & ((tr < 0) | (tr > 3.0))
    stats['tilt_max_outside'] = float(np.nanmax(tilt[outside]))
    stats['dz_range_outside'] = (float(np.nanmin(dz[outside])), float(np.nanmax(dz[outside])))
    axt.axhline(stats['tilt_max_outside'], color=gold, lw=0.6, ls=(0, (3.5, 2)), zorder=2)
    axt.annotate(f"rig away from the release, up to {stats['tilt_max_outside']:.1f}$^\\circ$",
                 (T1, stats['tilt_max_outside']), xytext=(-2, 2), textcoords='offset points', ha='right', va='bottom',
                 fontsize=8, color=GREY_TEXT)
    axt.plot(tr[kp], tilt[kp], 'o', color=gold, ms=3, zorder=4)
    lo, hi = min(stats['twin_tilt_peak']), max(stats['twin_tilt_peak'])
    axt.annotate(f'peak {tilt[kp]:.1f}$^\\circ$ (twin {lo:.1f}--{hi:.1f}$^\\circ$)', (tr[kp], tilt[kp]), xytext=(5, 3),
                 textcoords='offset points', ha='left', va='bottom', fontsize=8, color=GREY_TEXT)
    axz.plot(tr[kd], dz[kd], 'o', color=gold, ms=3, zorder=4)
    lo, hi = sorted(-v for v in stats['twin_dip_cm'])
    axz.annotate(f'dip {-dz[kd]:.1f}\\,cm (twin {lo:.1f}--{hi:.1f}\\,cm)', (tr[kd], dz[kd]), xytext=(6, -1),
                 textcoords='offset points', ha='left', va='center', fontsize=8, color=GREY_TEXT)
    axt.set_ylabel(r'Ring tilt (\si{\degree})')
    axt.set_ylim(0, 11)
    axt.set_yticks([0, 5, 10])
    axz.axhline(0.0, color='0.75', lw=0.5, zorder=0)
    axz.set_ylabel('Height error (cm)')
    axz.set_ylim(-6, 4)
    axz.set_yticks([-4, 0, 4])
    axz.set_xlabel('$t$ after the release (s)')
    axz.set_xlim(T0, T1)
    axz.set_xticks([-2, 0, 2, 4, 6, 8])
    axz.annotate('rig', (T1, -4.2), xytext=(-2, 0), textcoords='offset points', ha='right', va='center', fontsize=8,
                 color=gold)
    axz.annotate('twin, two runs', (T1, -5.3), xytext=(-2, 0), textcoords='offset points', ha='right', va='center',
                 fontsize=8, color=TWIN_LINE)

    # events and the path hold, on the three strips; named once on the top one
    for a in (axT, axt, axz):
        a.axvspan(t_hold0 - t_rel, t_hold1 - t_rel, color='0.93', lw=0, zorder=0)
        a.axvline(0.0, color='0.5', lw=0.5, zorder=0)
        a.axvline(t_det - t_rel, color='0.5', lw=0.5, ls=(0, (1, 1.5)), zorder=0)
    names = ('release', 'freed UAV starts landing')
    for j, ts in enumerate(t_snap[:2]):
        badge(axT, (ts - t_rel, 1.0), j + 1, coords=('data', 'axes fraction'), dy=8, leader=True)
        axT.annotate(names[j], (ts - t_rel, 1.0), xycoords=('data', 'axes fraction'), xytext=(-7, 8),
                     textcoords='offset points', ha='right', va='center', fontsize=8)
    axT.annotate(f'found +{t_det - t_rel:.2f}\\,s', (stats['leaver_dropped_at'], 0.6), xytext=(4, 0),
                 textcoords='offset points', ha='left', va='center', fontsize=8, color=GREY_TEXT)
    axz.annotate('path held', (t_hold0 - t_rel, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, -2),
                 textcoords='offset points', ha='left', va='top', fontsize=8, color=GREY_TEXT)
    axT.annotate(f"three survivors, {min(stats['survivors_after_N']):.1f}--{max(stats['survivors_after_N']):.1f}\\,N",
                 (T1, max(stats['survivors_after_N'])), xytext=(-2, 4), textcoords='offset points', ha='right',
                 va='bottom', fontsize=8, color='0.25')
    for a in (axT, axt):
        plt.setp(a.get_xticklabels(), visible=False)
    axp.set_anchor('N')
    for a, letter in zip((axp, axT, axt, axz), 'abcd'):
        ps.panel_label(a, letter)
    fig.align_ylabels([axT, axt, axz])
    stem = os.path.join(OUT, 'F_rig_circle_detach', 'rig_circle_detach')
    ps.save_print(fig, stem)
    stats.update(circle_centre=tuple(round(float(v), 3) for v in c0), radius=rad, speed=float(par['traj_speed']),
                 mass=float(par['load_mass']), target_z=float(par['target_z']),
                 snap_tilt=[round(float(tilt[k]), 1) for k in k_snap])
    return stem, stats


# ── F_twin_rig_parity ──────────────────────────────────────────────────────────────────────────────────────────────

PAT_FOUND = r'\[(\d{9,}\.\d+)\] \[\w+\]: \[dissipative\] UNANNOUNCED DETACH'
PAT_ATTACH = r'\[(\d{9,}\.\d+)\] \[\w+\]: \[planner\] FLEET RESIZED to n=4'


def transient(run, t_ev):
    """Tilt rise (deg), dip (cm) in the 3 s after t_ev; horizontal error (cm), mean over the 10 s before it."""
    d = run.run
    t = d.t.to_numpy() - t_ev
    tilt = d.payload_tilt_deg.to_numpy()
    dz = 100 * (d.payload_z - d.payload_ref_z).to_numpy()
    pre, post = (t >= -3.0) & (t < 0.0), (t >= 0.0) & (t <= 3.0)
    exy = 100 * np.hypot(d.payload_x - d.payload_ref_x, d.payload_y - d.payload_ref_y).to_numpy()
    return dict(rise=float(np.nanmax(tilt[post]) - np.nanmean(tilt[pre])), dip=float(-np.nanmin(dz[post])),
                exy=float(np.nanmean(exy[(t >= -10.0) & (t < 0.0)])))


def detach_metrics(spec, event):
    r = Run(spec)
    t_ev = r.ev(event)
    m = transient(r, t_ev)
    if event == 'RELEASE':
        m['found'] = r.log_time(PAT_FOUND, t_ev) - t_ev
    return m


def attach_metrics(spec):
    r = Run(spec)
    t_mag = r.ev('MAGNET') if r.ev('MAGNET') is not None else -np.inf
    t_ev = r.log_time(PAT_ATTACH, t_mag)
    m = transient(r, t_ev)
    m.pop('exy')                    # the hover before a weld is not the detach flights' carry
    return m


def spread(vals):
    v = np.asarray(vals, float)
    return float(v.mean()), float(v.min()), float(v.max())


def fig_parity():
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FixedLocator, FuncFormatter, NullLocator
    ps.thesis()
    # key, panel name, unit, axis limits (log), ticks
    keys = [('rise', 'Tilt rise', r'\si{\degree}', (1.0, 30.0), (1, 3, 10, 30)),
            ('dip', 'Ring dip', 'cm', (0.1, 10.0), (0.1, 0.3, 1, 3, 10)),
            ('found', 'Detection delay', 's', (0.2, 0.6), (0.2, 0.3, 0.4, 0.6)),
            ('exy', 'Horizontal error', 'cm', (1.0, 10.0), (1, 3, 10))]
    rows = []
    for name, mk, rig, twins, event in PARITY:
        rows.append(dict(name=name, mk=mk, rig=[detach_metrics(rig, event)],
                         twin=[detach_metrics(r, event) for r in twins]))
    att = dict(name='attach', mk='p', rig=[attach_metrics(r) for r in PARITY_ATTACH['rig']],
               twin=[attach_metrics(r) for r in PARITY_ATTACH['twin']])
    fig, axs = plt.subplots(1, 4, figsize=ps.size('full', height=2.1), layout='constrained')
    stats = {}
    band = 0.25
    fmt = FuncFormatter(lambda v, _: f'${v:g}$')
    for ax, (key, label, unit, (lo, hi), ticks) in zip(axs, keys):
        ax.set_xscale('log')
        ax.set_yscale('log')
        x = np.geomspace(lo, hi, 50)
        ax.fill_between(x, (1 - band) * x, (1 + band) * x, color='0.88', lw=0, zorder=0)
        ax.plot([lo, hi], [lo, hi], color='0.4', lw=0.7, ls=(0, (3.5, 2)), zorder=1)
        ax.plot([lo, hi / 2], [2 * lo, hi], color='0.45', lw=0.6, ls=(0, (1, 1.5)), zorder=1)
        for row in rows + [att]:
            tw = [m[key] for m in row['twin'] if key in m]
            if not tw:
                continue
            xm, x0, x1 = spread(tw)
            rg = [m[key] for m in row['rig'] if key in m]
            if row is att and not rg:  # PENDING: the twin's value, on y = x, hollow
                ax.errorbar(xm, xm, xerr=[[xm - x0], [x1 - xm]], fmt=row['mk'], ms=5, mfc='white', mec='0.3',
                            mew=0.8, ecolor='0.3', elinewidth=0.6, capsize=0, zorder=4)
                stats[f'{key}:{row["name"]}'] = dict(twin=(round(xm, 3), round(x0, 3), round(x1, 3)), rig='pending')
                continue
            ym, y0, y1 = spread(rg)
            ax.errorbar(xm, ym, xerr=[[xm - x0], [x1 - xm]], yerr=[[ym - y0], [y1 - ym]], fmt=row['mk'], ms=4.6,
                        mfc=INK, mec='white', mew=0.5, ecolor=INK, elinewidth=0.7, capsize=0, zorder=4)
            stats[f'{key}:{row["name"]}'] = dict(twin=(round(xm, 3), round(x0, 3), round(x1, 3)),
                                                 rig=round(ym, 3), ratio=round(ym / xm, 2))
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.set_aspect('equal', adjustable='box')
        for axis in (ax.xaxis, ax.yaxis):
            axis.set_major_locator(FixedLocator(ticks))
            axis.set_minor_locator(NullLocator())
            axis.set_major_formatter(fmt)
        ax.set_xlabel(f'Twin ({unit})')
        ax.set_ylabel(f'Rig ({unit})', labelpad=1)
        ax.annotate(f'({"abcd"[[k[0] for k in keys].index(key)]}) {label}', xy=(0, 1), xycoords='axes fraction',
                    xytext=(0, 4), textcoords='offset points', ha='left', va='bottom', fontsize=9)
    # the three references, named once, in (c) where there is room
    axs[2].annotate(r'$1\times$', (0.5, 0.5), xytext=(-4, 3), textcoords='offset points', ha='right', va='bottom',
                    fontsize=8, color=GREY_TEXT)
    axs[2].annotate(r'$2\times$', (0.27, 0.54), xytext=(-3, 0), textcoords='offset points', ha='right', va='center',
                    fontsize=8, color=GREY_TEXT)
    axs[2].annotate(f'$\\pm${band * 100:.0f}\\,\\%', (0.5, (1 - band) * 0.5), xytext=(2, -2), textcoords='offset points',
                    ha='left', va='top', fontsize=8, color=GREY_TEXT)
    if not att['rig']:
        xa = spread([m['rise'] for m in att['twin']])[0]
        axs[0].annotate('rig pending', (xa, xa), xytext=(6, -1), textcoords='offset points', ha='left', va='top',
                        fontsize=8, color=GREY_TEXT)
    axs[2].annotate('announced: none', (0.97, 0.04), xycoords='axes fraction', ha='right', va='bottom', fontsize=8,
                    color=GREY_TEXT)
    hs = [Line2D([], [], ls='none', marker=row['mk'], ms=4.6, mfc=INK, mec='white', mew=0.5) for row in rows]
    hs.append(Line2D([], [], ls='none', marker=att['mk'], ms=5, mfc='white' if not att['rig'] else INK, mec='0.3',
                     mew=0.8))
    names = [row['name'] for row in rows] + ['attach, twin only (rig pending)' if not att['rig'] else 'attach']
    fig.legend(hs, names, loc='outside upper center', ncol=3, frameon=False, fontsize=8, handletextpad=0.2,
               columnspacing=1.2, borderaxespad=0.2)
    stem = os.path.join(OUT, 'F_twin_rig_parity', 'twin_rig_parity')
    ps.save_print(fig, stem)
    return stem, stats


# ── F_disarm_lanes ─────────────────────────────────────────────────────────────────────────────────────────────────

PAT_DISARM = r'\[(\d{{9,}}\.\d+)\] \[tracker_{i}\]: \[Drone \d+\] (?:Disarm triggered|Disarmed via service)'
PAT_LAND = r"\[(\d{9,}\.\d+)\] \[fleet_manager\]: Fleet command received: 'LAND'"
IN_AIR_M = 0.10                     # above its resting height at the disarm: disarmed in the air


def height_above_rest(run, i, t_q):
    """UAV i's height above where it stood before take-off, at run time t_q. run.csv first; a rig run.csv ends when the
    first tracker log does (a UAV disarmed alone shuts its tracker down), so later times read that UAV's tracker log."""
    d = run.run
    rest = float(np.nanmedian(d[f'd{i}_z'][d.t <= d.t.iloc[0] + 1.0]))
    if t_q <= d.t.iloc[-1]:
        return float(np.interp(t_q, d.t, d[f'd{i}_z'])) - rest
    T = pd.read_csv(sorted(glob.glob(os.path.join(run.logs, 'tracker', f'drone{i}_*', 'log.csv')))[-1])
    tt = T.sim_time - run.man['t0_epoch'] if run.kind == 'rig' else None
    if tt is None:
        raise SystemExit(f'{run.name}: disarm of UAV {i} after the end of run.csv')
    return float(np.interp(t_q, tt, T.pose_z)) - rest


def lanes(spec, kind):
    """Per UAV: [(t0, t1, state)] on the clock of the trigger (the fault or the release), and its disarm (t, in air)."""
    run = Run(spec)
    if kind == 'fault':
        t0 = run.ev('MOCAP_LOSS')
        subj = int(run.ev_arg('MOCAP_LOSS'))
    else:
        t0 = run.ev('RELEASE')
        subj = released_drone(run)
    n = len([c for c in run.run.columns if re.fullmatch(r'd\d+_x', c)])
    t_land = run.log_time(PAT_LAND, t0)
    t_down = run.log_time(rf'\[(\d{{9,}}\.\d+)\] \[\w+\]: \[dissipative\] departed drone \d+ \(/drone_{subj}\) lands now', t0)
    out = {}
    for i in range(n):
        t_off = run.log_time(PAT_DISARM.format(i=i), t0 - 1.0)
        if t_off is None:
            raise SystemExit(f'{spec}: no disarm line for UAV {i}')
        h = height_above_rest(run, i, t_off - 0.05)
        seg = []
        if i == subj and kind == 'fault':          # the newcomer in transit: flying free until it is disarmed
            seg.append((-np.inf, t_off, 'free'))
        elif i == subj:                             # the freed carrier
            seg += [(-np.inf, t0, 'carrying'), (t0, t_down if t_down is not None else t_off, 'free')]
            if t_down is not None and t_down < t_off:
                seg.append((t_down, t_off, 'landing'))
        else:
            if t_land is not None and t_land < t_off:
                seg += [(-np.inf, t_land, 'carrying'), (t_land, t_off, 'landing')]
            else:
                seg.append((-np.inf, t_off, 'carrying'))
        armed = run.run[f'd{i}_armed'].to_numpy()
        edge = np.flatnonzero((armed[:-1] > 0.5) & (armed[1:] <= 0.5) & (run.run.t.to_numpy()[1:] > t0 - 1.0))
        t_edge = float(run.run.t.iloc[edge[0] + 1]) if len(edge) else None
        out[i] = dict(seg=[(a - t0, b - t0, st) for a, b, st in seg], off=t_off - t0, air=h > IN_AIR_M, h=h,
                      edge=None if t_edge is None else t_edge - t0)
    return dict(subj=subj, n=n, uav=out, land=None if t_land is None else t_land - t0,
                down=None if t_down is None else t_down - t0)


def tint(colour, f):
    """`colour` mixed with white: f = 1 the colour, f = 0 white."""
    c = np.array(matplotlib.colors.to_rgb(colour))
    return tuple(1 - f * (1 - c))


# a state is a shade and a bar height: carrying dark, flying free mid (or the UAV's own colour), landing light and
# thin, so the three still read apart in greyscale
STATE = dict(carrying=('0.2', 0.56), free=('0.55', 0.56), landing=('0.8', 0.30))
REPEAT_H = 0.12


def state_colour(st, colour):
    if colour is None or st == 'carrying':      # off the ring only after the release: a carrier until then
        return STATE[st][0]
    return colour if st == 'free' else tint(colour, 0.35)


def draw_lane(ax, y, segs, colour, t_min, h_scale=1.0, dy=0.0):
    for a, b, st in segs:
        a = max(a, t_min)
        if b > a:
            h = STATE[st][1] * h_scale
            ax.add_patch(Rectangle((a, y + dy - h / 2), b - a, h, facecolor=state_colour(st, colour),
                                   edgecolor='none', zorder=2))


def draw_block(ax, y0, L, kind, repeat=None, t_min=-2.0):
    """Four lanes from y0 downwards: the subject first, then the carriers."""
    order = [L['subj']] + [i for i in range(L['n']) if i != L['subj']]
    for row, i in enumerate(order):
        y = y0 + row
        colour = ps.drone_colour(i) if i == L['subj'] else None
        draw_lane(ax, y - 0.08, L['uav'][i]['seg'], colour, t_min)
        u = L['uav'][i]
        mk = dict(marker='x', ms=4.2, mew=1.1) if u['air'] else dict(marker='|', ms=5.0, mew=1.3)
        ax.plot(u['off'], y - 0.08, ls='none', color=INK, zorder=4, **mk)
    return order


def fig_lanes():
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    ps.thesis()
    blocks_a = [(name, [lanes(r, 'fault') for r in runs]) for name, runs in LANES_FAULT]
    blocks_b = [(name, [lanes(r, 'freed') for r in runs]) for name, runs in LANES_FREED]
    stats = {}
    for name, Ls in blocks_a + blocks_b:
        for k, L in enumerate(Ls):
            s_ = L['uav'][L['subj']]
            car = [L['uav'][i] for i in L['uav'] if i != L['subj']]
            stats[f'{name} #{k + 1}'] = dict(subject=L['subj'], subject_off=round(s_['off'], 2), subject_air=s_['air'],
                                            subject_h=round(s_['h'], 2),
                                            subject_edge=None if s_['edge'] is None else round(s_['edge'], 2),
                                            carriers_off=[round(c['off'], 2) for c in car],
                                            carriers_air=[c['air'] for c in car],
                                            carriers_edge=[None if c['edge'] is None else round(c['edge'], 2) for c in car],
                                            land=None if L['land'] is None else round(L['land'], 2),
                                            down=None if L['down'] is None else round(L['down'], 2))
    gap = 1.8
    span = lambda nb: nb * 4 + (nb - 1) * gap      # noqa: E731
    fig, (axa, axb) = plt.subplots(2, 1, figsize=ps.size('full', height=4.0), layout='constrained',
                                   gridspec_kw=dict(height_ratios=[span(len(blocks_a)) + 1.5,
                                                                   span(len(blocks_b)) + 1.5]))
    fig.get_layout_engine().set(h_pad=0.06)
    t_end_a = max(u['off'] for _, Ls in blocks_a for L in Ls for u in L['uav'].values())
    t_end_b = max(u['off'] for _, Ls in blocks_b for L in Ls for u in L['uav'].values())
    # zero at the same place on both axes
    ra, rb = np.ceil(t_end_a + 1.6), np.ceil(t_end_b + 4.0)
    frac = 1.0 / 12.0
    lims = {axa: (-frac * ra / (1 - frac), ra), axb: (-frac * rb / (1 - frac), rb)}
    lab = dict(textcoords='offset points', fontsize=8, color=GREY_TEXT, va='center')
    for ax, blocks, kind in ((axa, blocks_a, 'fault'), (axb, blocks_b, 'freed')):
        t_min, t_max = lims[ax]
        y0 = 0.0
        yt, yl, yc = [], [], []
        for name, Ls in blocks:
            ax.annotate(rf'\textbf{{{name}}}', (0.0, y0 - 1.2), xycoords=('axes fraction', 'data'), xytext=(-3, 0),
                        textcoords='offset points', ha='right', va='center', fontsize=8)
            if Ls:
                L = Ls[0]
                draw_block(ax, y0, L, kind, None, t_min)
                R = Ls[1] if len(Ls) > 1 else None
                if kind == 'freed' and L['land'] is not None:      # the fleet's LAND, across this block only
                    ax.plot([L['land']] * 2, [y0 - 0.5, y0 + 3.5], color='0.4', lw=0.6, ls=(0, (3, 2)), zorder=0)
                subj = L['subj']
                us = L['uav'][subj]
                car = [L['uav'][i] for i in L['uav'] if i != subj]
                c_off = max(c['off'] for c in car)
                nd = 2 if us['off'] < 1 else 1
                if abs(c_off - us['off']) < 0.5:    # all four at once
                    ax.annotate(f"all four disarmed in the air, +{us['off']:.2f}\\,s", (us['off'], y0 + 1.5),
                                xytext=(9, 0), ha='left', **lab)
                else:
                    where = f", {us['h']:.1f}\\,m up" if us['air'] else ''
                    rep = f" (+{R['uav'][subj]['off']:.{nd}f}\\,s)" if R else ''
                    ax.annotate(f"+{us['off']:.{nd}f}\\,s, alone{where}{rep}", (us['off'], y0 - 0.08),
                                xytext=(8, 0), ha='left', **lab)
                    rc = max(R['uav'][i]['off'] for i in R['uav'] if i != subj) if R else None
                    ax.annotate(f'+{c_off:.1f}\\,s' + (f'\n(+{rc:.1f}\\,s)' if R else ''), (c_off, y0 + 2.0),
                                xytext=(6, 0), ha='left', linespacing=1.0, **lab)
            else:                   # PENDING: the same four lanes, outlined and lightly hatched, until the run is listed
                for row in range(4):
                    h = STATE['carrying'][1]
                    ax.add_patch(Rectangle((t_min, y0 + row - 0.08 - h / 2), t_max - t_min, h, facecolor='white',
                                           edgecolor='0.7', hatch='//', lw=0.4, zorder=1))
                ax.text((t_min + t_max) / 2, y0 + 1.42, PENDING_TEXT, ha='center', va='center', fontsize=8,
                        color=GREY_TEXT, zorder=3, bbox=dict(boxstyle='square,pad=0.25', fc='white', ec='none'))
                subj = None
            yt += [y0 - 0.08, y0 + 1.92]
            yl += ['newcomer' if kind == 'fault' else 'freed UAV', 'carriers']
            yc += [ps.drone_colour(subj) if subj is not None else GREY_TEXT, '0.2']
            y0 += 4 + gap
        ax.set_yticks(yt)
        ax.set_yticklabels(yl, fontsize=8)
        for t_lab, c in zip(ax.get_yticklabels(), yc):
            t_lab.set_color(c)
        ax.tick_params(axis='y', length=0, pad=3)
        ax.set_ylim(y0 - gap - 0.45, -1.45)
        ax.set_xlim(t_min, t_max)
        ax.spines['left'].set_visible(False)
        ax.axvline(0.0, color='0.5', lw=0.5, zorder=0)
    t_l = blocks_b[0][1][0]['land']
    axb.annotate('LAND', (t_l, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, 2), textcoords='offset points',
                 ha='center', va='bottom', fontsize=8)
    axb.annotate('release', (0.0, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, 2), textcoords='offset points',
                 ha='center', va='bottom', fontsize=8)
    axa.annotate('motion capture lost', (0.0, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, 2),
                 textcoords='offset points', ha='left', va='bottom', fontsize=8)
    axa.set_xlabel("$t$ after the newcomer's motion capture is lost (s)")
    axb.set_xlabel('$t$ after the release (s)')
    ps.panel_label(axa, 'a', dx_pt=-62)
    ps.panel_label(axb, 'b', dx_pt=-62)
    hs = [Patch(facecolor=STATE[k][0], edgecolor='none') for k in ('carrying', 'free')]
    hs.append(Rectangle((0, 0), 1, 1, facecolor=STATE['landing'][0], edgecolor='none'))
    hs += [Line2D([], [], ls='none', marker='x', ms=4.2, mew=1.1, color=INK),
           Line2D([], [], ls='none', marker='|', ms=5.0, mew=1.3, color=INK)]
    names = ['carrying', 'flying free', 'landing (thin)', 'disarmed in the air', 'disarmed at rest']
    fig.legend(hs, names, loc='outside upper center', ncol=5, frameon=False, fontsize=8, handlelength=1.3,
               handletextpad=0.4, columnspacing=0.9, borderaxespad=0.2)
    stem = os.path.join(OUT, 'F_disarm_lanes', 'disarm_lanes')
    ps.save_print(fig, stem)
    return stem, stats


FIGS = {'circle': fig_circle, 'parity': fig_parity, 'lanes': fig_lanes}

if __name__ == '__main__':
    names = sys.argv[1:] or list(FIGS)
    for n in names:
        stem, stats = FIGS[n]()
        print(stem)
        for k, v in stats.items():
            print(f'  {k}: {round(v, 3) if isinstance(v, float) else v}')
