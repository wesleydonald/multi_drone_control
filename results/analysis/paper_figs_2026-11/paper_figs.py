#!/usr/bin/env python3
"""Single-run figures of the combined paper (IEEE journal, main.tex "Experiments and results") from one real flight:

    attach_run       "Attach results over a single run"
    detach_run       "Detach results over a single run"
    application_run  the application: detach, pickup, transit, drop, re-attach, land

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control
    nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py attach results/rig/2026-11-05/sq8_2
    nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py detach results/rig/2026-11-05/nr9m
    nice -n 10 python3 results/analysis/paper_figs_2026-11/paper_figs.py application results/rig/2026-11-05/fm9y_2

Each writes <out>/<kind>_<day>_<flight>.pdf and .png (default out: figs/ beside this file) at an IEEE width: one
column (3.5 in) for attach and detach, the page (7.16 in) for the application; --width single|double swaps the layout
(single: plan view above the time panels; double: plan view left, time panels right).

Everything is read, not typed: the events from the launch log (paper_rig.parse_log and joins.parse_log), the ring,
QUAD4's body, its magnet tip and the object from the bag (joins.read_bag, cached in JOINS_CACHE), the ring's set-point
from the drone-0 tracker log, and every quad's mocap pose from the bag (read here, cached in FIGS_CACHE, default
~/mdc_overnight/paper_figs_cache). Definitions are paper_rig.py's: the ring's tilt is the normal's angle from vertical
(release_bands.tilt_vector), the position error the 3-D distance to the planner's set-point, the magnet's offsets from
its plate joins.plate_level's (horizontal: in the level frame; height: along the ring's normal above the weld point).
"""
import argparse
import glob
import math
import os
import pickle
import re
import sqlite3
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.expanduser('~/multi_drone_control')
sys.path.insert(0, os.path.join(REPO, 'results', 'analysis', 'paper_rig_2026-11'))
sys.path.insert(1, os.path.join(REPO, 'tools'))
import paper_rig as PR  # noqa: E402  (sets JOINS_CACHE's default and imports joins)
import joins  # noqa: E402
import plot_style as PS  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

CACHE = os.environ.get('FIGS_CACHE', os.path.expanduser('~/mdc_overnight/paper_figs_cache'))
OUT = os.path.join(HERE, 'figs')
NEWCOMER = 3                      # /drone_3 is QUAD4, the quadrotor that leaves and returns (paper_rig: tip_drone=3)
COL_IN = 3.5                      # IEEE single column
PAGE_IN = 7.16                    # IEEE double column
PLATE_R = 0.0275                  # M2A plates: 55 mm discs
OBJ_AFTER_DROP_S = 3.0            # the object is drawn until this long after the drop (see application_run)

RING = PS.PAYLOAD_LINE
QUAD4 = PS.drone_colour(NEWCOMER)  # red, as in RViz
MAGNET = '#7a0f10'                 # QUAD4's magnet: a darker red (its track solid, its height dashed)
CARRIER = '0.55'                   # the other quadrotors: context, grey
OBJECT = PS.drone_colour(4)        # purple
REF = '0.6'
DASH = (0, (2.5, 1.2))             # the magnet's height, in every figure


# ------------------------------------------------------------------------------------------------------- style
def style():
    """plot_style.thesis() at IEEE sizes: Times (newtx) to match IEEEtran, 8 pt labels, 7 pt ticks and legend."""
    PS.thesis()
    plt.rcParams.update({
        'text.latex.preamble': r'\usepackage{newtxtext}\usepackage{newtxmath}\usepackage{siunitx}',
        'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8,
        'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 7,
        'legend.handlelength': 1.6, 'legend.columnspacing': 1.2, 'legend.borderpad': 0.35,
        'lines.linewidth': 0.9,
    })


# ------------------------------------------------------------------------------------------------------- data
def read_quads(folder):
    """{drone: array (t, x, y, z, qx, qy, qz, qw)} of every /drone_N/motion_capture_state in the bag. Cached."""
    fn = os.path.join(CACHE, folder.replace('/', '_') + '.pkl')
    if os.path.exists(fn):
        with open(fn, 'rb') as f:
            return pickle.load(f)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {}
    for db in sorted(glob.glob(os.path.join(folder, '*.db3'))):
        c = sqlite3.connect(db)
        for i, name, ty in c.execute('select id,name,type from topics'):
            m = re.fullmatch(r'/drone_(\d+)/motion_capture_state', name)
            if not m:
                continue
            cls = get_message(ty)
            rows = out.setdefault(int(m.group(1)), [])
            for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
                msg = deserialize_message(d, cls)
                p, q = msg.pose.position, msg.pose.orientation
                rows.append((ts * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        c.close()
    out = {k: np.array(sorted(v)) for k, v in out.items()}
    os.makedirs(CACHE, exist_ok=True)
    with open(fn + '.tmp%d' % os.getpid(), 'wb') as f:
        pickle.dump(out, f)
    os.replace(fn + '.tmp%d' % os.getpid(), fn)
    return out


class Run:
    """One flight: paper_rig's Flight (log events, bag, ring grid) plus every quad's pose and QUAD4's plate."""

    def __init__(self, path):
        parts = os.path.normpath(os.path.abspath(path) if os.path.exists(path) else path).split(os.sep)
        self.day, self.name = parts[-2], parts[-1]
        self.fid = f'{self.day}/{self.name}'
        launch = {x['name']: x for x in PR.launches(self.day)}.get(self.name, {})
        self.F = F = PR.Flight(self.fid, launch, PR.registry(), PR.conditions())
        if F.B is None or F.G is None:
            raise SystemExit(f'{self.fid}: no bag or no ring pose')
        self.B = F.B
        self.Q = read_quads(os.path.join(PR.RIG, self.day, self.name))
        self.t0 = F.t_takeoff if F.t_takeoff is not None else F.B['t0']
        self.r_att = (F.geo or {}).get('attach_radius', 0.225)
        self.plate = joins.plate_of(F.args)
        self.weld = self._weld()

    def _weld(self):
        """QUAD4's weld point in the ring frame, as joins.py scores it (the typed contact target; 5 Nov's flights that
        typed 4 Nov's values are scored against 5 Nov's measured point)."""
        a = self.F.args
        ct = self.F.J['contact_target'] or (float(a.get('attach_x_offset', 0.0)), float(a.get('attach_y_offset', 0.225)),
                                            float(a.get('weld_point_z', 0.0365)))
        if self.day == '2026-11-05' and abs(ct[2] - joins.WELD_5NOV[2]) > 1e-4:
            xy = joins.turned(joins.WELD_5NOV[:2], self.plate)
            ct = (xy[0], xy[1], joins.WELD_5NOV[2])
        return ct

    # -- series, time in s since TAKEOFF unless t0 is given
    def ring_series(self, t0=None):
        G = self.F.G
        t = G.t.to_numpy(float) - (self.t0 if t0 is None else t0)
        p = G[['payload_x', 'payload_y', 'payload_z']].to_numpy(float)
        r = G[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
        tilt = np.linalg.norm(G.attrs['th'], axis=1)
        return t, p, np.linalg.norm(p - r, axis=1), tilt

    def magnet_to_plate(self, t0=None):
        """(t, horizontal offset, height above the plate) of QUAD4's magnet tip, metres."""
        tip, ring = self.B['tip'], self.B['ring']
        lvl, normal, _ = joins.plate_level(ring, tip[:, 0], tip[:, 1:4], self.weld)
        return tip[:, 0] - (self.t0 if t0 is None else t0), np.hypot(lvl[:, 0], lvl[:, 1]), normal

    def magnet_to_point(self, xy_world, t0=None):
        tip = self.B['tip']
        return tip[:, 0] - (self.t0 if t0 is None else t0), np.hypot(tip[:, 1] - xy_world[0], tip[:, 2] - xy_world[1])

    def carriers(self):
        return sorted(k for k in self.Q if k != NEWCOMER)

    # -- events (epoch s)
    def first_phase(self, to=None, frm=None, after=-math.inf):
        return next((p[0] for p in self.F.J['phases'] if p[0] > after and (to is None or p[2] == to)
                     and (frm is None or p[1] == frm)), None)

    def release(self):
        """Our magnet OFF that released QUAD4 from the ring (the planned detach), or the operator's OFF."""
        rows = [r for r in self.F.detaches()]
        return self.t0 + rows[0]['t_event_s'] if rows else None

    def fold_in(self, after=-math.inf):
        pv = [p for p in self.F.J['proven'] if p[0] >= after]
        if not pv:
            return None, None
        t_pv = pv[0][0]
        fold = next((r[0] for r in self.F.L['resize'] if t_pv - 0.5 <= r[0] <= t_pv + 5.0 and int(r[1]) >= 4), None)
        return t_pv, fold

    def first_loss(self, after):
        lo = self.F.loss_between(after, math.inf)
        return lo[0] if lo else None


# ------------------------------------------------------------------------------------------------------- drawing
def ring_points(ring, t, r, z, n=121):
    """The ring's circle of radius r (ring frame height z) at time t, projected on the floor (x, y)."""
    k = max(0, int(np.searchsorted(ring[:, 0], t, side='right')) - 1)
    p, q = ring[k, 1:4], ring[k, 4:8]
    a = np.linspace(0, 2 * np.pi, n)
    pts = np.stack([r * np.cos(a), r * np.sin(a), np.full(n, z)], 1)
    return p + Rot.from_quat(q).apply(pts)


def draw_ring(ax, run, t, target=True):
    """The ring at time t (black), its 12 plates, QUAD4's plate filled in its colour."""
    ring = run.B['ring']
    k = max(0, int(np.searchsorted(ring[:, 0], t, side='right')) - 1)
    p, rot = ring[k, 1:4], Rot.from_quat(ring[k, 4:8])
    w = np.array(run.weld)
    rim = ring_points(ring, t, run.r_att, w[2])
    ax.plot(rim[:, 0], rim[:, 1], color=RING, lw=1.0, zorder=3)
    phi0 = math.atan2(w[1], w[0])
    for j in range(12):
        a = phi0 + j * math.pi / 6
        c = p + rot.apply([run.r_att * math.cos(a), run.r_att * math.sin(a), w[2]])
        mine = target and j == 0
        ax.add_patch(Circle((c[0], c[1]), PLATE_R, facecolor=QUAD4 if mine else 'white', edgecolor=RING, lw=0.5,
                            zorder=4))


def window(arr, a, b):
    w = (arr[:, 0] >= a) & (arr[:, 0] <= b)
    return arr[w]


def plan_limits(ax, xs, ys, pad=0.06):
    x0, x1 = np.nanmin(xs), np.nanmax(xs)
    y0, y1 = np.nanmin(ys), np.nanmax(ys)
    m = pad * max(x1 - x0, y1 - y0)
    ax.set_xlim(x0 - m, x1 + m)
    ax.set_ylim(y0 - m, y1 + m)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel(r'$x$ (m)')
    ax.set_ylabel(r'$y$ (m)')


def state_panel(ax, spans, order, t_a, t_b):
    """QUAD4's (or the mission's) state as a step line over labelled levels: every transition shows as a jump."""
    lvl = {s: i for i, s in enumerate(order)}
    ts, ys = [], []
    for a, b, s in spans:
        if b < t_a or a > t_b:
            continue
        ts += [max(a, t_a), min(b, t_b)]
        ys += [lvl[s], lvl[s]]
    ax.plot(ts, ys, color='0.15', lw=1.0, solid_joinstyle='miter', drawstyle='default')
    ax.set_yticks(range(len(order)))
    ax.set_yticklabels(order)
    ax.set_ylim(len(order) - 0.5, -0.5)
    ax.tick_params(axis='y', length=0, pad=3)
    for i in range(len(order)):
        ax.axhline(i, color='0.92', lw=0.5, zorder=0)


def mark(axes, ts, **kw):
    for ax in axes:
        for t in ts:
            if t is not None:
                ax.axvline(t, color=kw.get('color', '0.6'), lw=0.5, ls=kw.get('ls', ':'), zorder=0)


def layout(width, panels_in, plan_aspect, plans=1):
    """A figure with the plan view(s) and time panels (heights in inches) sharing x. single: plan above (one column);
    double: plan left (the page). plan_aspect: height / width of one plan view's data."""
    if width == 'single':
        W = COL_IN
        plan_w = (W - 0.15) / plans - 0.6
        plan_h = min(2.2, plan_w * plan_aspect) + 0.4
        H = plan_h + sum(panels_in) + 0.05 * len(panels_in) + 0.75
        fig = plt.figure(figsize=(W, H))
        top, bot = fig.subfigures(2, 1, height_ratios=[plan_h, H - plan_h])
    else:
        W = PAGE_IN
        H = max(2.8, sum(panels_in) + 0.05 * len(panels_in) + 0.75)
        fig = plt.figure(figsize=(W, H))
        plan_w = min(W * 0.4, (H - 0.75) / plans / plan_aspect + 0.65)
        top, bot = fig.subfigures(1, 2, width_ratios=[plan_w, W - plan_w])
    plan = top.subplots(1, plans) if width == 'single' else top.subplots(plans, 1)
    axes = bot.subplots(len(panels_in), 1, sharex=True, gridspec_kw={'height_ratios': panels_in})
    return fig, (list(np.atleast_1d(plan)) if plans > 1 else plan), list(np.atleast_1d(axes))


def state_height(n):
    """Inches for a state panel of n labelled levels at 7 pt."""
    return 0.105 * n


def letters(fig, groups):
    """'(a)', '(b)', ... at the top-left of each axes, left of its tick and axis labels (one column per group of
    axes), so stacked panels need no gap for them. groups: [([axes], 'abc'), ...]."""
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    for axs, ls in groups:
        off = min(ax.get_tightbbox(r).x0 - ax.get_window_extent(r).x0 for ax in axs) * 72.0 / fig.dpi
        for ax, l in zip(axs, ls):
            ax.annotate(f'({l})', xy=(0, 1), xycoords='axes fraction', xytext=(off - 2.5, 0),
                        textcoords='offset points', ha='right', va='top', fontsize=8)


def ylim_in(ax, t, y, a, b, lo=None, pad=0.12, floor=None):
    """y limits from the data inside the time window only (the flight's floor and LAND stay out of the scale)."""
    w = (t >= a) & (t <= b) & np.isfinite(y)
    if not w.any():
        return
    y0, y1 = float(np.min(y[w])), float(np.max(y[w]))
    if floor is not None:
        y1 = max(y1, floor)
    m = pad * max(y1 - y0, 1e-6)
    ax.set_ylim(lo if lo is not None else y0 - m, y1 + m)


def headroom(axes, frac=0.1):
    """Stacked panels: keep each top tick label clear of the panel above (its bottom tick sits on the shared edge)."""
    for ax in axes:
        lo, hi = ax.get_ylim()
        ticks = [t for t in ax.get_yticks() if lo <= t <= hi]
        if ticks and hi - max(ticks) < frac * (hi - lo):
            ax.set_yticks(ticks)
            ax.set_ylim(lo, max(ticks) + 1.5 * frac * (hi - lo))


def finish(fig, plans, axes, handles, ncol, stem):
    headroom(axes[1:])
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    fig.align_ylabels(axes)
    if fig.get_figwidth() < 4.0:
        ncol = min(ncol, 3)
    PS.legend_below(fig, handles, [h.get_label() for h in handles], ncol=ncol)
    abc = 'abcdefgh'
    groups = [([p], abc[i]) for i, p in enumerate(plans)] + [(axes, abc[len(plans):len(plans) + len(axes)])]
    letters(fig, groups)
    os.makedirs(os.path.dirname(stem), exist_ok=True)
    return PS.save_print(fig, stem)


def time_ticks(ax):
    ax.xaxis.set_major_locator(MaxNLocator(nbins=8, steps=[1, 2, 5, 10], integer=True))


def round_ticks(*axes):
    for ax in axes:
        ax.yaxis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 5, 10], integer=True))


# ------------------------------------------------------------------------------------------------------- attach
ATTACH_STATES = ['Carrier', 'Released', 'Take-off', 'Approach', 'Match', 'Descent', 'Proof', 'Attached', 'Land']
HIS = {'TAKEOFF': 'Take-off', 'APPROACH_ABOVE_TARGET': 'Approach', 'TRANSIT_TO_REATTACH': 'Approach',
       'MATCH_VELOCITY': 'Match', 'DESCEND_TO_ATTACHMENT': 'Descent', 'ATTACH_READY': 'Descent',
       'LIFT_PROOF': 'Proof', 'TAKEN_BACK': 'Taken back'}


def attach_states(run, t_start, t_end):
    """[(a, b, state)] epoch s for QUAD4: Carrier until our release; Released until his take-off; Take-off until our
    hand-over (PARTNER RELEASE) or his next phase, whichever is later; then his planner's phases (approach, match,
    descent with its bottom hold, proof); Attached from the fold-in (FLEET RESIZED back to 4 after ATTACH: proven);
    Land from the operator's LAND. A flight from the mat starts on the ground (state 'Ground')."""
    F = run.F
    t_rel = run.release()
    his_to = F.J['his_takeoff'][0] if F.J['his_takeoff'] else None
    hand = F.J['release'][0] if F.J['release'] else None
    t_pv, fold = run.fold_in(after=his_to or -math.inf)
    ev = [(t_start, 'Carrier' if t_rel is not None else 'Ground')]
    if t_rel is not None:
        ev.append((t_rel, 'Released'))
    if his_to is not None:
        ev.append((his_to, 'Take-off'))
        t_first = next((p[0] for p in F.J['phases'] if p[0] > his_to and p[1] == 'TAKEOFF'), his_to)
        t_his = max(t_first, hand) if hand else t_first
        stop = min(x for x in (fold, F.t_land, math.inf) if x is not None)
        cur = None
        for t, a, b, why in F.J['phases']:
            if t < his_to or b == 'TAKEOFF' or b not in HIS or t >= stop:
                continue
            if t <= t_his:
                cur = HIS[b]          # the phase in force when he took QUAD4 over
                continue
            ev.append((t, HIS[b]))
        if cur:
            ev.append((t_his, cur))
    if fold is not None:
        ev.append((fold, 'Attached'))
    if F.t_land is not None:
        ev.append((F.t_land, 'Land'))
    ev.sort(key=lambda e: e[0])
    spans = []
    for (a, s), (b, _) in zip(ev, ev[1:] + [(t_end, None)]):
        if b > a:
            spans.append((a, b, s))
    return spans, dict(release=t_rel, his_takeoff=his_to, handover=hand, proven=t_pv, fold=fold,
                       contact=[c[0] for c in F.J['contacts']], land=F.t_land)


def attach_run(path, out=OUT, width='single'):
    """Top: the plan view from the release to the fold-in (the ring at the contact, its plates, QUAD4's plate in red;
    the quadrotors' tracks; QUAD4's body and its magnet tip) and the magnet's last approach to its plate in the
    plate's frame. Below, sharing time since TAKEOFF: QUAD4's state, the magnet's distance to its plate (horizontal,
    and height above it), the ring's tilt and its position error. The window runs from 2 s before the release (his
    take-off for a start from the mat) to 2 s after LAND, cut at a loss."""
    style()
    run = Run(path)
    F = run.F
    t_rel = run.release()
    his_to = F.J['his_takeoff'][0] if F.J['his_takeoff'] else None
    t_a = (t_rel if t_rel is not None else his_to) - 2.0
    t_b = min(x for x in (F.t_land + 2.0 if F.t_land else None, run.F.t_end) if x is not None)
    lo = run.first_loss(t_a)
    if lo is not None:
        t_b = min(t_b, lo)
    spans, ev = attach_states(run, t_a, t_b)
    order = [s for s in (['Ground'] if t_rel is None else []) + ATTACH_STATES + ['Taken back']
             if any(sp[2] == s for sp in spans)]
    # the contact that was proven (the last before ATTACH: proven), else the last contact of the flight
    before = [c for c in ev['contact'] if ev['proven'] and c <= ev['proven']]
    t_con = before[-1] if before else (ev['contact'][-1] if ev['contact'] else None)
    t_snap = t_con or ev['fold'] or (F.t_land or t_b)
    t_plan_end = ev['fold'] + 1.0 if ev['fold'] else t_b

    # ---- the plan views
    body = window(run.B['body'], t_a + 2.0, t_plan_end)
    tip = window(run.B['tip'], t_a + 2.0, t_plan_end)
    xs = [body[:, 1], tip[:, 1]]
    ys = [body[:, 2], tip[:, 2]]
    car = {k: window(run.Q[k], t_a + 2.0, t_plan_end) for k in run.carriers()}
    for v in car.values():
        xs.append(v[:, 1])
        ys.append(v[:, 2])
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    aspect = (np.ptp(ys) + 0.1) / (np.ptp(xs) + 0.1)
    fig, (ax_p, ax_z), axes = layout(width, [state_height(len(order)), 0.62, 0.55, 0.55], aspect, plans=2)
    for k, v in car.items():
        ax_p.plot(v[:, 1], v[:, 2], color=CARRIER, lw=0.6)
        s = joins.at(run.Q[k], t_snap)
        if s is not None:
            ax_p.plot(s[1], s[2], 'o', ms=3.2, color=CARRIER, mec='white', mew=0.4, zorder=5)
    draw_ring(ax_p, run, t_snap)
    ax_p.plot(body[:, 1], body[:, 2], color=QUAD4, lw=0.9)
    ax_p.plot(tip[:, 1], tip[:, 2], color=MAGNET, lw=0.5)
    s = joins.at(run.B['body'], t_snap)
    if s is not None:
        ax_p.plot(s[1], s[2], 'o', ms=3.2, color=QUAD4, mec='white', mew=0.4, zorder=5)
    ax_p.plot(body[0, 1], body[0, 2], marker='o', ms=3, mfc='white', mec=QUAD4, mew=0.7, zorder=5)
    plan_limits(ax_p, xs, ys)

    # the magnet's last approach, in the plate's frame (radial outward, tangential anticlockwise), cm
    t_m = next((p[0] for p in F.J['phases'] if p[2] == 'MATCH_VELOCITY' and p[0] > (his_to or -math.inf)), None)
    if t_m is not None:
        w = (run.B['tip'][:, 0] >= t_m) & (run.B['tip'][:, 0] <= (ev['proven'] or t_b))
        T = run.B['tip'][w]
        lvl, normal, _ = joins.plate_level(run.B['ring'], T[:, 0], T[:, 1:4], run.weld)
        tan, rad = 100 * lvl[:, 1], 100 * lvl[:, 0]
        ax_z.plot(tan, rad, color=MAGNET, lw=0.6)
        ax_z.add_patch(Circle((0, 0), 100 * PLATE_R, facecolor=QUAD4, edgecolor=RING, lw=0.5, alpha=0.25, zorder=0))
        ax_z.add_patch(Circle((0, 0), 100 * PLATE_R, facecolor='none', edgecolor=RING, lw=0.6, zorder=1))
        if t_con is not None:
            j = int(np.clip(np.searchsorted(T[:, 0], t_con), 0, len(T) - 1))
            ax_z.plot(tan[j], rad[j], marker='x', ms=4, color='k', mew=0.9, zorder=6)
        lim = 12.0
        ax_z.set_xlim(-lim, lim)
        ax_z.set_ylim(-lim, lim)
        ax_z.set_aspect('equal', adjustable='box')
        ax_z.set_xlabel('Tangential (cm)')
        ax_z.set_ylabel('Radial (cm)')

    # ---- the time panels (s since TAKEOFF)
    r = lambda t: None if t is None else t - run.t0  # noqa: E731
    ax_s, ax_m, ax_t, ax_e = axes
    state_panel(ax_s, [(r(a), r(b), s) for a, b, s in spans], order, r(t_a), r(t_b))
    tm, hor, hgt = run.magnet_to_plate()
    ax_m.plot(tm, 100 * hor, color=MAGNET, lw=0.9)
    ax_m.plot(tm, 100 * hgt, color=MAGNET, lw=0.7, ls=DASH)
    ax_m.set_ylim(-3, 45)          # the step-out (about 1 m away) runs off the top: the approach is what matters
    ax_m.set_yticks([0, 20, 40])
    ax_m.set_ylabel('Offset from\nplate (cm)')
    tr, _, err, tilt = run.ring_series()
    ax_t.plot(tr, tilt, color=RING, lw=0.9)
    ax_t.set_ylabel(r'Tilt (\si{\degree})')
    ax_e.plot(tr, 100 * err, color=RING, lw=0.9)
    ax_e.set_ylabel('Position\nerror (cm)')
    ax_e.set_xlabel(r'$t$ (s)')
    ax_e.set_xlim(r(t_a), r(t_b))
    time_ticks(ax_e)
    ylim_in(ax_t, tr, tilt, r(t_a), r(t_b), lo=0)
    ylim_in(ax_e, tr, 100 * err, r(t_a), r(t_b), lo=0)
    round_ticks(ax_t, ax_e)
    # the transitions that matter, through every data panel: release, contact, fold-in, LAND
    mark(axes[1:], [r(t) for t in (t_rel, t_con, ev['fold'], F.t_land) if t is not None and t < t_b], color='0.6')
    handles = [Line2D([], [], color=RING, lw=1.0, label='Ring'),
               Line2D([], [], color=CARRIER, lw=0.8, label='Quadrotors 1--3'),
               Line2D([], [], color=QUAD4, lw=1.0, label='Returning quadrotor'),
               Line2D([], [], color=MAGNET, lw=0.8, label='Magnet'),
               Line2D([], [], color=MAGNET, lw=0.7, ls=DASH, label='Magnet height'),
               Line2D([], [], color='k', marker='x', ls='none', ms=4, label='Contact')]
    stem = os.path.join(out, f'attach_{run.day}_{run.name}')
    return finish(fig, [ax_p, ax_z], axes, handles, 3, stem)


# ------------------------------------------------------------------------------------------------------- detach
def detach_run(path, out=OUT, width='single', event=0):
    """One release (paper_rig's detach rows: an operator's OFF on any quad's magnet, or our planned detach of QUAD4
    after its unload). Top: the plan view over the window (the ring at the release, the other quadrotors' tracks, the
    detached one leaving, with its magnet if it is QUAD4) and the ring's and the other quadrotors' horizontal shift.
    Below, time after the release: the ring's tilt, its height change and the other quadrotors' shift. A shift is
    measured from the mean over the "before" window and relative to the ring's set-point (in a hover: the plain
    shift from where it was). Marked: the release (t = 0), the detection (the planner's UNANNOUNCED DETACH line) for
    an unannounced release, the unload for a planned one. The window is paper_rig's: from the start of its "before"
    (5 s, cut where the set-point last moved) to the first of 15 s, LAND, a loss, the next resize or QUAD4's next
    descent."""
    style()
    run = Run(path)
    F = run.F
    rows = F.detaches()
    if not rows:
        raise SystemExit(f'{run.fid}: no detach in this flight')
    row = rows[event]
    te = run.t0 + row['t_event_s']
    gone = int(row['drone'].split('_')[-1])            # the detached quad
    hero = PS.drone_colour(gone)
    planned = row['event'] == 'detach (planned)'
    t_b = F.window_end(te, 'detach')
    t_unload = te - row['unload_s'] if planned and row.get('unload_s') is not None else None
    t_det = te + row['detect_delay_s'] if row.get('detect_delay_s') is not None else None
    # "before" = paper_rig's: the 5 s before the release, cut where the set-point last moved (the lift's end); the
    # window starts there too, so the lift's tail does not set the scale
    lo = max(te - PR.PRE_S, PR.ref_still_from(F.G, te))
    if te - lo < 1.0:
        lo = te - 1.0
    t_a = lo
    others = [k for k in sorted(run.Q) if k != gone]

    # ---- plan views
    car = {k: window(run.Q[k], t_a, t_b) for k in others}
    body = window(run.Q[gone], te, t_b) if gone in run.Q else np.zeros((0, 8))
    tip = window(run.B['tip'], te, t_b) if gone == NEWCOMER else np.zeros((0, 8))
    xs = np.concatenate([v[:, 1] for v in car.values()] + [body[:, 1], tip[:, 1]])
    ys = np.concatenate([v[:, 2] for v in car.values()] + [body[:, 2], tip[:, 2]])
    aspect = (np.ptp(ys) + 0.1) / (np.ptp(xs) + 0.1)
    fig, (ax_p, ax_z), axes = layout(width, [0.7, 0.62, 0.62], aspect, plans=2)
    for k, v in car.items():
        ax_p.plot(v[:, 1], v[:, 2], color=CARRIER, lw=0.6)
        s = joins.at(run.Q[k], te)
        if s is not None:
            ax_p.plot(s[1], s[2], 'o', ms=3.2, color=CARRIER, mec='white', mew=0.4, zorder=5)
    draw_ring(ax_p, run, te, target=gone == NEWCOMER and F.J['contact_target'] is not None)
    if len(body):
        ax_p.plot(body[:, 1], body[:, 2], color=hero, lw=0.9)
        ax_p.plot(body[0, 1], body[0, 2], 'o', ms=3.2, color=hero, mec='white', mew=0.4, zorder=5)
    if len(tip):
        ax_p.plot(tip[:, 1], tip[:, 2], color=MAGNET, lw=0.5)
    plan_limits(ax_p, xs, ys)

    sp = PR.tracker_ref(run.day, run.name)          # (t, x, y, z) of the ring's set-point, as logged
    if len(sp) > 1:                                  # the tracker repeats each planner update: keep the changes only
        sp = sp[np.r_[True, np.any(np.diff(sp[:, 1:4], axis=0) != 0, axis=1)]]
    if len(sp) < 2:                                 # no tracker log: a fixed point (the ring's mean before)
        pre = window(run.B['ring'], lo, te)
        sp = np.array([[lo] + list(pre[:, 1:4].mean(0)), [t_b] + list(pre[:, 1:4].mean(0))])

    def shift(arr):
        """(t, dx, dy, dz) cm: the position relative to the ring's set-point, minus its mean over "before"."""
        w = window(arr, t_a, t_b)
        ref = np.stack([np.interp(w[:, 0], sp[:, 0], sp[:, 1 + i]) for i in range(3)], 1)
        rel = w[:, 1:4] - ref
        pre = (w[:, 0] >= lo) & (w[:, 0] < te)
        return w[:, 0], 100 * (rel - np.nanmean(rel[pre], 0))
    # (b): where the ring and the quadrotors went after the release (cm)
    tracks = []
    for k in car:
        t, d = shift(run.Q[k])
        tracks.append((d[t >= te, :2], CARRIER, 0.6))
    t, d = shift(run.B['ring'])
    tracks.append((d[t >= te, :2], RING, 0.9))
    for d, c, lw in tracks:
        ax_z.plot(d[:, 0], d[:, 1], color=c, lw=lw)
    ax_z.plot(0, 0, '+', color='k', ms=6, mew=0.7, zorder=6)
    allxy = np.concatenate([d for d, _, _ in tracks] + [np.zeros((1, 2))])
    c = (np.nanmax(allxy, 0) + np.nanmin(allxy, 0)) / 2
    half = max(3.0, 0.55 * float(np.nanmax(np.ptp(allxy, 0))))
    ax_z.set_xlim(c[0] - half, c[0] + half)
    ax_z.set_ylim(c[1] - half, c[1] + half)
    ax_z.set_aspect('equal', adjustable='box')
    ax_z.set_xlabel(r'$\Delta x$ (cm)')
    ax_z.set_ylabel(r'$\Delta y$ (cm)')

    # ---- time panels: t after the release
    ax_t, ax_h, ax_c = axes
    a, b = t_a - te, t_b - te
    tr, p, err, tilt = run.ring_series(t0=te)
    ax_t.plot(tr, tilt, color=RING, lw=0.9)
    ax_t.set_ylabel(r'Tilt (\si{\degree})')
    ylim_in(ax_t, tr, tilt, a, b, lo=0)
    round_ticks(ax_t)
    t, d = shift(run.B['ring'])
    ax_h.plot(t - te, d[:, 2], color=RING, lw=0.9)
    ax_h.axhline(0, color=REF, lw=0.5, ls='--', zorder=0)
    ax_h.set_ylabel('Height\nchange (cm)')
    ylim_in(ax_h, t - te, d[:, 2], a, b)
    round_ticks(ax_h)
    top = 0.0
    for k in car:
        t, d = shift(run.Q[k])
        n = np.linalg.norm(d, axis=1)
        ax_c.plot(t - te, n, color=CARRIER, lw=0.7)
        top = max(top, float(np.nanmax(n)))
    ax_c.set_ylim(0, 1.12 * top)
    ax_c.set_ylabel('Quadrotor\nshift (cm)')
    ax_c.set_xlabel(r'$t$ (s)')
    ax_c.set_xlim(a, b)
    time_ticks(ax_c)
    lab = dict(xycoords=('data', 'axes fraction'), textcoords='offset points', va='bottom', fontsize=7)
    if t_unload is not None:
        for ax in axes:
            ax.axvspan(t_unload - te, 0, color='0.91', lw=0, zorder=0)
        ax_t.annotate('Unload', xy=(0, 1.0), xytext=(-3, 2), ha='right', **lab)
    mark(axes, [0.0], color='0.35', ls='-')
    if t_det is not None:      # the release's label left of its line, the detection's right of its own
        mark(axes, [t_det - te], color='0.35', ls='--')
        ax_t.annotate('Release', xy=(0, 1.0), xytext=(-2, 2), ha='right', **lab)
        ax_t.annotate('Detection', xy=(t_det - te, 1.0), xytext=(2, 2), ha='left', **lab)
    else:
        ax_t.annotate('Release', xy=(0, 1.0), xytext=(3, 2), ha='left', **lab)
    handles = [Line2D([], [], color=RING, lw=1.0, label='Ring'),
               Line2D([], [], color=CARRIER, lw=0.8, label='Other quadrotors'),
               Line2D([], [], color=hero, lw=1.0, label='Detached quadrotor')]
    if len(tip):
        handles.append(Line2D([], [], color=MAGNET, lw=0.6, label='Magnet'))
    stem = os.path.join(out, f'detach_{run.day}_{run.name}' + (f'_{event}' if event else ''))
    return finish(fig, [ax_p, ax_z], axes, handles, len(handles), stem)


# ------------------------------------------------------------------------------------------------------- application
APP_STATES = ['Lift', 'Hover', 'Detach', 'Pickup', 'Transit', 'Drop', 'Re-attach', 'Attached', 'Land']


def application_states(run, t_end):
    """[(a, b, state)] epoch s, paper_rig's phases: Lift = TAKEOFF to the ring's set-point at the top; Detach = our
    UNLOAD to the hand-over (PARTNER RELEASE); Pickup = to 'object lifted' (from his TAKEOFF for a pickup from the
    mat); Transit = to DROP_OBJECT; Drop = to CLEAR_DROP_ZONE; Re-attach = to the fold-in; Attached = to LAND;
    Land = to 'Landed - disarming'. Hover fills a gap between the lift and the detach."""
    F = run.F
    kind = F.kind()
    ev = [(run.t0, 'Lift')]
    top = F.ring_up()
    unload = F.L['unload'][0][0] if F.L['unload'] else None
    hand = F.J['release'][0] if F.J['release'] else None
    his_to = F.J['his_takeoff'][0] if F.J['his_takeoff'] else None
    p_start = hand if kind == 'application' else his_to
    nxt = min(x for x in ((unload if kind == 'application' else None), p_start, math.inf) if x is not None)
    if top is not None and top < nxt:
        ev.append((top, 'Hover'))
    if kind == 'application' and unload is not None:
        ev.append((unload, 'Detach'))
    if p_start is not None and kind != 'delivery-rejoin':
        ev.append((p_start, 'Pickup'))
    lifted = (run.first_phase(to='TRANSIT_TO_DROP_POINT', frm='LIFT_OBJECT', after=p_start or -math.inf)
              if kind != 'delivery-rejoin' else run.first_phase(to='TRANSIT_TO_DROP_POINT'))
    t_drop = run.first_phase(to='DROP_OBJECT', after=lifted or math.inf)
    t_clear = run.first_phase(frm='DROP_OBJECT', after=t_drop or math.inf)
    for t, s in ((lifted, 'Transit'), (t_drop, 'Drop'), (t_clear, 'Re-attach')):
        if t is not None:
            ev.append((t, s))
    fold = None
    if t_clear is not None and kind in ('application', 'delivery-rejoin'):
        _, fold = run.fold_in(after=t_clear)
        if fold is not None:
            ev.append((fold, 'Attached'))
    if F.t_land is not None:
        ev.append((F.t_land, 'Land'))
    if kind not in ('application', 'delivery-rejoin'):   # a pickup and drop only: back to the hover after the drop
        ev = [(t, 'Hover' if s == 'Re-attach' else s) for t, s in ev]
    ev.sort(key=lambda e: e[0])
    spans = [(a, b, s) for (a, s), (b, _) in zip(ev, ev[1:] + [(t_end, None)]) if b > a]
    return spans, dict(unload=unload, handover=hand, lifted=lifted, drop=t_drop, clear=t_clear, fold=fold,
                       land=F.t_land)


def application_run(path, out=OUT, width='double'):
    """The whole mission. Left: the plan view from QUAD4's release to LAND (the ring at the drop, the quadrotors, QUAD4's
    body, the object until it lay in the net). Right, time since TAKEOFF: the mission's state, the heights (QUAD4, its
    magnet, the object, the ring), the ring's tilt and its position error, to 1 s after the fleet landed."""
    style()
    run = Run(path)
    F = run.F
    t_a = run.t0 - 1.0
    t_b = min(x for x in ((F.t_landed or F.t_end) + 1.0, F.t_end) if x is not None)
    lo = run.first_loss(run.t0)
    if lo is not None:              # cut at a loss: once the ring is dropped its pose can freeze (fm9_2)
        t_b = min(t_b, lo)
    spans, ev = application_states(run, t_b)
    order = [s for s in APP_STATES if any(sp[2] == s for sp in spans)]
    # the plan view: QUAD4 (its body) from its release to LAND, the quadrotors over the same time, the ring when the
    # object was dropped (or at the release); the take-off and landing moves stay out (they only stretch the view)
    t_rel = run.release() or ev['handover'] or run.t0
    t_snap = ev['drop'] or t_rel
    t_land = F.t_land or t_b
    body = window(run.B['body'], t_rel, t_land)
    # the object until it has settled in the net (drop + OBJ_AFTER_DROP_S): later its Motive body snaps onto the
    # magnet's markers whenever the magnet passes over the net (fm9y_2: three jumps of up to 0.4 m onto the tip)
    t_obj_end = (ev['drop'] + OBJ_AFTER_DROP_S) if ev['drop'] else t_land
    obj = window(run.B['obj'], t_a, t_obj_end) if len(run.B['obj']) else np.zeros((0, 8))
    car = {k: window(run.Q[k], t_rel, t_land) for k in run.carriers()}
    xs = np.concatenate([v[:, 1] for v in car.values()] + [body[:, 1]] + ([obj[:, 1]] if len(obj) else []))
    ys = np.concatenate([v[:, 2] for v in car.values()] + [body[:, 2]] + ([obj[:, 2]] if len(obj) else []))
    aspect = (np.ptp(ys) + 0.1) / (np.ptp(xs) + 0.1)
    fig, ax_p, axes = layout(width, [state_height(len(order)), 0.8, 0.5, 0.5], aspect, plans=1)
    for k, v in car.items():
        ax_p.plot(v[:, 1], v[:, 2], color=CARRIER, lw=0.6)
        s = joins.at(run.Q[k], t_snap)
        if s is not None:
            ax_p.plot(s[1], s[2], 'o', ms=3.2, color=CARRIER, mec='white', mew=0.4, zorder=5)
    draw_ring(ax_p, run, t_snap)
    ax_p.plot(body[:, 1], body[:, 2], color=QUAD4, lw=0.7)
    ax_p.plot(body[0, 1], body[0, 2], marker='o', ms=3, mfc='white', mec=QUAD4, mew=0.7, zorder=5)
    if len(obj):
        ax_p.plot(obj[:, 1], obj[:, 2], color=OBJECT, lw=0.9)
        ax_p.plot(obj[0, 1], obj[0, 2], 's', ms=3.5, color=OBJECT, mec='white', mew=0.4, zorder=6)
    plan_limits(ax_p, xs, ys)

    r = lambda t: None if t is None else t - run.t0  # noqa: E731
    ax_s, ax_h, ax_t, ax_e = axes
    state_panel(ax_s, [(r(a), r(b), s) for a, b, s in spans], order, r(t_a), r(t_b))
    tr, p, err, tilt = run.ring_series()
    ax_h.plot(run.B['body'][:, 0] - run.t0, run.B['body'][:, 3], color=QUAD4, lw=0.8)
    ax_h.plot(run.B['tip'][:, 0] - run.t0, run.B['tip'][:, 3], color=MAGNET, lw=0.7, ls=DASH)
    if len(obj):
        ax_h.plot(obj[:, 0] - run.t0, obj[:, 3], color=OBJECT, lw=0.8)
    ax_h.plot(tr, p[:, 2], color=RING, lw=0.9)
    ax_h.set_ylabel(r'$z$ (m)')
    ax_t.plot(tr, tilt, color=RING, lw=0.8)
    ax_t.set_ylabel(r'Tilt (\si{\degree})')
    ax_e.plot(tr, 100 * err, color=RING, lw=0.8)
    ax_e.set_ylabel('Position\nerror (cm)')
    ax_e.set_xlabel(r'$t$ (s)')
    ax_e.set_xlim(r(t_a), r(t_b))
    time_ticks(ax_e)
    end = r(F.t_land or t_b)       # the ring on the floor after LAND stays out of the scale
    ylim_in(ax_t, tr, tilt, 0.0, end, lo=0)
    ylim_in(ax_e, tr, 100 * err, 0.0, end, lo=0)
    round_ticks(ax_t, ax_e)
    ax_h.set_ylim(bottom=0)
    mark(axes[1:], [r(t) for a, t, s in spans[:-1]], color='0.75')
    handles = [Line2D([], [], color=RING, lw=1.0, label='Ring'),
               Line2D([], [], color=CARRIER, lw=0.8, label='Quadrotors 1--3'),
               Line2D([], [], color=QUAD4, lw=1.0, label='Returning quadrotor'),
               Line2D([], [], color=MAGNET, lw=0.7, ls=DASH, label='Magnet height')]
    if len(run.B['obj']):
        handles.append(Line2D([], [], color=OBJECT, lw=0.8, marker='s', ms=3.5, label='Object'))
    stem = os.path.join(out, f'application_{run.day}_{run.name}')
    return finish(fig, [ax_p], axes, handles, len(handles), stem)


# ------------------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('kind', choices=['attach', 'detach', 'application'])
    ap.add_argument('flight', help='results/rig/<day>/<flight> (or <day>/<flight>)')
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--width', choices=['single', 'double'])
    ap.add_argument('--event', type=int, default=0, help='detach: which release of the flight (0 = the first)')
    a = ap.parse_args(argv)
    if a.kind == 'attach':
        paths = attach_run(a.flight, a.out, a.width or 'single')
    elif a.kind == 'detach':
        paths = detach_run(a.flight, a.out, a.width or 'single', a.event)
    else:
        paths = application_run(a.flight, a.out, a.width or 'double')
    print('\n'.join(paths))


if __name__ == '__main__':
    main()
