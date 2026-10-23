#!/usr/bin/env python3
"""tools/thesis_lever_figures.py -- the leave chapter's lever figures from the detach freeze tree (thesis-figure skill:
print size, Computer Modern, no run ids in the figure or caption; the runs are in configs/thesis_figures.yaml and
docs/figures/PROVENANCE.md).

    python3 tools/thesis_lever_figures.py crossover   # F10b
    python3 tools/thesis_lever_figures.py levers      # F12
    python3 tools/thesis_lever_figures.py             # both

Every number comes from the run directories: Delta-theta is tools/release_bands.py's (freeze 1.1, event to +15 s); a run
whose ring was dropped or whose fleet latched inside that window is cut at that instant (the planner's DROP AND LAND or
the first ENVELOPE FAULT line, mapped to run time as tools/drop_metrics.py does) and drawn as a lower bound. The
detection instant is the planner's resize (tools/thesis_freeze_tables.py). The resolution rule is freeze 1.2.

Briefs (thesis-figure skill, step 1):

F10b  Sentence: in one twin cell (an unannounced release of the UAV carrying the smallest share, the three left leaving
      a 120 deg gap, hover at 1.0 m), the ring's tilt rise was 5-8 deg when the survivors flew the measured rod pull
      and 17 deg when they flew the planner's pull, which holds the four-UAV shares until the planner finds the loss
      about 0.3 s after the release; the two arms separate under the frozen resolution rule.
  Hero: (a) the tilt departure against time from the release, three runs per pull source, with the planner's
  detection instants. Support: (b) the scored Delta-theta per run and the arm means, with the gap the rule needs.
  Left out: the announced ramped pair (not flown: the crossover's other half), the 12 Oct exploratory runs (another
  tree), the absolute tilt (the planned-pull ring's standing 2 deg is in the caption), the rig (no flight of this
  cell yet: RIG_F10B is empty). Window: 0.5 s before the release to 4 s after (every peak is inside 0.6 s; the score
  runs to +15 s).
  Must not say (cards): a general "a third"; any rig number; the detection timing as the cause (the formations
  differ); "the twin confirms the crossover"; "the measured pull detects the loss sooner" (the reverse).

F12   Sentence: (a) in the twin each one-variable lever moved the tilt rise by more than its run-to-run spread, except
      detection on the measured pull, which made no resolvable difference at n = 3; (b) with the measured pull the
      tilt rise grows as the support margin the leaver leaves shrinks, and at zero margin the ring cannot be held.
  Hero: (b) Delta-theta against the support margin. Support: (a) the forest of one-variable pairs on the freeze tree.
  Left out: the ramp, integral-mode and model-error pairs (not flown on this tree), the 60 deg circle pair (a path
  error, not Delta-theta), landing lean (K1 not credited). Lower bounds (a drop or a latch) are drawn as such
  (reviewer, Tier B card): G150p is cut at the trackers' 30 deg drop, G180 at the 60 deg latch, which exists only with
  the drop rule off; G180 on the defaults is cut at the gap rule's drop. Rig points: the 8 Oct unannounced hover
  releases at 120 and 150 deg (another integral mode and freed-UAV policy, their own hold), and the 7 Oct announced
  planned-pull release at 150 deg whose magnets let go, off scale.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.dont_write_bytecode = True
TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import plot_style as ps  # noqa: E402
import release_bands as rb  # noqa: E402
import thesis_freeze_tables as ft  # noqa: E402

OUT = os.path.join(os.path.dirname(TOOLS), 'docs', 'figures')
HZ = rb.BAND_HORIZON_S
PLATE_R_MM = 225.0
DROP_DEG = 30.0
GREY_TEXT = '0.35'
PLANNED = '#c0392b'          # the one accent: the planner's pull (measured: black). Filled = measured, hollow = planned.

# ── runs (provenance: configs/thesis_figures.yaml) ────────────────────────────────────────────────────────────────
A1, A2 = ['R1887', 'R1891', 'R1895'], ['R1888', 'R1892', 'R1896']          # T3: measured / planned, detection on
A5, A6 = ['R1889', 'R1893', 'R1897'], ['R1890', 'R1894', 'R1898']          # detection off: measured / planned
TIER_B = {120: A1, 135: ['R1902', 'R1907'], 150: ['R1899', 'R1904'], 165: ['R1903', 'R1908'],
          180: ['R1901', 'R1906']}                                         # measured pull; G180 drop rule off
G150P, G180D = ['R1900', 'R1905'], ['R1919']                               # planned at 150; 180 on the defaults
RIG_F10B = []                                                              # rig:<date>/<run> of this cell, once flown
RIG_F12 = [('rig:2026-10-08/r303_4', 120, 'measured'), ('rig:2026-10-08/r304_3', 150, 'measured')]
RIG_F12_OFF = [('RIG-1007-r0013', 150, 'planned')]      # off scale: magnets let go (F11); value from the P6 table
P6_RELEASES = os.path.join(os.path.dirname(TOOLS), 'results', 'analysis', 'p6_2026-10-12', '1_releases', 'table.csv')


def margin_mm(gap_deg):
    return PLATE_R_MM * np.cos(np.radians(gap_deg) / 2.0)


def score(rid):
    """Delta-theta to +15 s, or to the drop / latch inside that window (then `bound` is True)."""
    t_cut, kind = ft.cut_instant(rid)
    import pandas as pd
    ev = pd.read_csv(os.path.join(rb.locate(rid), 'events.csv'))
    r = rb.Run(rid) if (ev.event == 'LAND').any() else rb.Run(rid, tland=t_cut)   # a latch in the air: no LAND
    hz, bound = HZ, False
    if t_cut is not None and r.te < t_cut < r.te + HZ:
        hz, bound = t_cut - r.te, True
    m = r.metrics(hz)
    return dict(run=rid, dth=m['dtheta'], t_peak=m['t_peak'], abs_peak=m['abs_peak'], bound=bound, cut=kind if bound
                else None, cut_s=hz if bound else None, run_obj=r)


def rig_score(tag):
    """A rig release on its own hold (event to LAND; the freeze's rule for a rig row with no frozen band)."""
    r = rb.Run(tag)
    return dict(run=tag, dth=r.metrics(None)['dtheta'], hold=r.tland - r.te)


def rig_off_scale(tag):
    """A rig flight that capsized: its row of the 12 Oct offline release table (the same T6 metric, event to the
    fleet's latch; the bag conversion lives outside this tree), a lower bound."""
    import pandas as pd
    T = pd.read_csv(P6_RELEASES)
    row = T[T.run == tag].iloc[0]
    return dict(run=tag, dth=float(row.dtheta_deg), window=float(row.window_s), end=str(row.window_end))


def resolve(a, b, sign):
    """Freeze 1.2 on arm values a, b (lists): (resolved, gap B - A, the gap needed)."""
    ma, mb = np.mean(a), np.mean(b)
    need = max(1.0, 2 * max(np.ptp(a), np.ptp(b)))
    overlap = not (max(a) < min(b) or max(b) < min(a))
    ok = (np.sign(mb - ma) == sign) and not overlap and abs(mb - ma) >= need
    return ok, mb - ma, need


def detection_s(rid):
    """The planner's resize after the release, in run time (s after the event)."""
    s = ft.switch(rid)
    return s['t_res'] - ft.node_offset(rid) - rb.Run(rid).te


# ── F10b: the crossover, unannounced half ─────────────────────────────────────────────────────────────────────────
def fig_crossover():
    ps.thesis()
    arms = [('measured', A1, 'k'), ('planned', A2, PLANNED)]
    S = {name: [score(r) for r in runs] for name, runs, _ in arms}
    det = {name: [detection_s(r) for r in runs] for name, runs, _ in arms}
    for name, _, _ in arms:
        for s, d in zip(S[name], det[name]):
            r = s['run_obj']
            pre_mean, _, _ = r.pre()
            print(f"F10b {s['run']} {name}: dtheta {s['dth']:.2f} t_peak {s['t_peak']:.2f} abs peak {s['abs_peak']:.2f} "
                  f"pre-mean tilt {np.linalg.norm(pre_mean):.2f} detection {d:.2f}")
    ok, gap, need = resolve([s['dth'] for s in S['measured']], [s['dth'] for s in S['planned']], +1)
    print(f'F10b T3: resolved {ok}, B - A {gap:.2f}, needed {need:.2f}')

    fig = plt.figure(figsize=ps.size('full', height=2.45))
    gs = fig.add_gridspec(1, 2, width_ratios=[3.7, 0.9])
    ax, axs = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    t0, t1 = -0.5, 4.0
    dmin = min(min(v) for v in det.values())
    dmax = max(max(v) for v in det.values())
    ax.axvspan(dmin, dmax, color='0.88', lw=0, zorder=0)
    ax.annotate(f'planner finds the loss, {dmin:.2f}--{dmax:.2f}\\,s', (dmax, 19.6), xytext=(3, 0),
                textcoords='offset points', ha='left', va='top', fontsize=7, color=GREY_TEXT)
    ax.axvline(0.0, color='0.5', lw=0.5, zorder=0)
    for name, runs, col in arms:
        for s in S[name]:
            r = s['run_obj']
            mean, _, _ = r.pre()
            t = r.t - r.te
            w = (t >= t0) & (t <= t1)
            dev = np.linalg.norm(r.th[w] - mean, axis=1)
            ax.plot(t[w], dev, color=col, lw=0.9, alpha=0.9, zorder=3 if name == 'measured' else 2,
                    ls='-' if name == 'measured' else (0, (4, 1.5)))           # dashed: the planner's pull (greyscale)
            ax.plot(s['t_peak'], s['dth'], 'o', ms=2.8, color=col, mfc=col if name == 'measured' else 'white',
                    mew=0.7, zorder=4)
    lab = {'measured': 'measured pull', 'planned': "planner's pull"}
    for name, _, col in arms:
        hi = max(S[name], key=lambda s: s['dth'])
        if name == 'planned':
            ax.annotate(lab[name], (hi['t_peak'], hi['dth']), xytext=(16, -4), textcoords='offset points', ha='left',
                        va='top', fontsize=8, color=col)
        else:                                  # clear of the planner's-pull descent
            ax.annotate(lab[name], (hi['t_peak'], hi['dth']), xytext=(1.05, 5.2), textcoords='data', ha='left',
                        va='center', fontsize=8, color=col)
    ax.set_xlim(t0, t1)
    ax.set_ylim(0, 20)
    ax.set_yticks([0, 5, 10, 15, 20])
    ax.set_xticks([0, 1, 2, 3, 4])
    ax.set_xlabel('$t$ from the release (s)')
    ax.set_ylabel('Tilt departure (\\si{\\degree})')

    # (b) the scored value per run, the arm means and the gap the rule needs
    for k, (name, _, col) in enumerate(arms):
        v = [s['dth'] for s in S[name]]
        x = np.full(len(v), float(k)) + np.linspace(-0.08, 0.08, len(v))
        axs.plot(x, v, 'o', ms=3.6, color=col, mfc=col if name == 'measured' else 'white', mew=0.9, zorder=3)
        axs.plot([k - 0.22, k + 0.22], [np.mean(v)] * 2, color=col, lw=1.2, zorder=2)
    ma, mb = np.mean([s['dth'] for s in S['measured']]), np.mean([s['dth'] for s in S['planned']])
    axs.annotate('', xy=(0.5, mb), xytext=(0.5, ma), arrowprops=dict(arrowstyle='<->', lw=0.6, color='0.4',
                                                                       shrinkA=0, shrinkB=0))
    axs.annotate(f'{gap:.1f}\\si{{\\degree}}\n(needed:\n$\\geq${need:.1f}\\si{{\\degree}})', (0.5, 0.5 * (ma + mb)),
                 xytext=(3, 0),
                 textcoords='offset points', ha='left', va='center', fontsize=7, color=GREY_TEXT)
    axs.set_xlim(-0.45, 1.45)
    axs.set_xticks([0, 1])
    axs.set_xticklabels(['measured\npull', "planner's\npull"])
    axs.set_ylim(0, 20)
    axs.set_yticks([0, 5, 10, 15, 20])
    axs.set_ylabel('Tilt rise $\\Delta\\theta$ (\\si{\\degree})')
    ps.panel_label(ax, 'a')
    ps.panel_label(axs, 'b')
    stem = os.path.join(OUT, 'F10b_crossover', 'crossover')
    print('\n'.join(ps.save_print(fig, stem)))
    return S, det, (ok, gap, need)


# ── F12: the other levers ─────────────────────────────────────────────────────────────────────────────────────────
def lower_bound_arrow(ax, x, y, col, dx=1.5, dy=0.0):
    """A lower bound: an arrow from the marker in the direction of increase (right in (a), up in (b))."""
    x0, y0 = (x + 0.35, y) if dx else (x, y + 0.45)
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x0, y0), arrowprops=dict(arrowstyle='-|>', lw=0.6, color=col,
                                                                          mutation_scale=5, shrinkA=0, shrinkB=0))


def fig_levers():
    ps.thesis()
    sc = {}
    for rid in A1 + A2 + A5 + A6 + sum(TIER_B.values(), []) + G150P + G180D:
        if rid not in sc:
            sc[rid] = score(rid)
            s = sc[rid]
            print(f"F12 {rid} {ft.config_name(rid)}: dtheta {'>=' if s['bound'] else ''}{s['dth']:.2f} "
                  f"(cut {s['cut']} at +{s['cut_s']:.2f} s)" if s['bound'] else
                  f"F12 {rid} {ft.config_name(rid)}: dtheta {s['dth']:.2f} t_peak {s['t_peak']:.2f}")
    v = lambda runs: [sc[r]['dth'] for r in runs]  # noqa: E731
    meas, plan = ('measured', 'k', True, 'o'), ("planner's", PLANNED, False, 'o')
    pairs = [  # (header, arm A, arm B, frozen sign (0: none), A style, B style)
        ('Pull source, 120$^\\circ$ gap', A1, A2, +1, meas, plan),
        ('Pull source, 150$^\\circ$ gap', TIER_B[150], G150P, +1, meas, plan),
        ('Detection, measured pull', A1, A5, 0, ('on', 'k', True, 'o'), ('off', 'k', True, 's')),
        ("Detection, planner's pull", A2, A6, +1, ('on', PLANNED, False, 'o'), ('off', PLANNED, False, 's')),
    ]
    fig = plt.figure(figsize=ps.size('full', height=3.0))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.05])
    axa, axb = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    # (a) one-variable pairs: a header line per pair, one sub-row per arm
    yt, ylab, y = [], [], 0.0
    for header, ra, rbs, sign, sa, sb in pairs:
        va, vb = v(ra), v(rbs)
        ok, gap, need = resolve(va, vb, sign if sign else np.sign(np.mean(vb) - np.mean(va)))
        verdict = ('resolved' if ok else 'not resolved') if sign else ('resolved' if ok else 'no resolvable difference')
        print(f'F12a {header}: A {np.mean(va):.2f} [{min(va):.2f}, {max(va):.2f}] B {np.mean(vb):.2f} '
              f'[{min(vb):.2f}, {max(vb):.2f}] gap {gap:+.2f} needed {need:.2f} -> {verdict}')
        axa.annotate(header, (0.0, y), xycoords=('axes fraction', 'data'), xytext=(-4, 0), textcoords='offset points',
                     ha='right', va='center', fontsize=8, style='italic')
        axa.annotate(verdict, (0.6, y), xytext=(0, 0), textcoords='offset points', ha='left', va='center', fontsize=7,
                     color=GREY_TEXT)
        for (word, col, filled, shape), runs, dy in ((sa, ra, -0.42), (sb, rbs, -0.84)):
            for r, jy in zip(runs, np.linspace(-0.1, 0.1, len(runs))):     # a little vertical jitter: every run shows
                s = sc[r]
                axa.plot(s['dth'], y + dy + jy, shape, ms=3.0 if shape == 'o' else 2.6, color=col,
                         mfc=col if filled else 'white', mew=0.8 if filled is False else 0.4, zorder=3)
                if s['bound']:
                    lower_bound_arrow(axa, s['dth'], y + dy + jy, col)
            yt.append(y + dy)
            ylab.append(word)
        y -= 1.45
    axa.axvline(DROP_DEG, color='0.6', lw=0.6, ls=(0, (1, 1.5)), zorder=0)
    axa.annotate('drop rule (absolute tilt)', (DROP_DEG, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, 2),
                 textcoords='offset points', ha='right', va='bottom', fontsize=7, color=GREY_TEXT)
    axa.set_yticks(yt)
    axa.set_yticklabels(ylab, fontsize=7, color=GREY_TEXT)
    axa.set_ylim(y + 0.35, 0.35)
    axa.set_xlim(0, 34)
    axa.set_xticks([0, 10, 20, 30])
    axa.set_xlabel('Tilt rise $\\Delta\\theta$ (\\si{\\degree})')
    axa.spines['left'].set_visible(False)
    axa.tick_params(axis='y', length=0, pad=2)

    # (b) Delta-theta against the support margin
    top = 37.0
    xs, means = [], []
    for g, runs in TIER_B.items():
        m = margin_mm(g)
        vals = [sc[r] for r in runs]
        if min(s['dth'] for s in vals) > top:          # off scale: the 60 deg latch with the drop rule off
            for j, s in enumerate(vals):
                xj = m + (j - 0.5) * 2.6
                axb.annotate('', xy=(xj, top - 1.2), xytext=(xj, top - 4.4),
                             arrowprops=dict(arrowstyle='-|>', lw=0.7, color='k', mutation_scale=6, shrinkA=0,
                                             shrinkB=0))
            axb.annotate(f"$\\geq${min(s['dth'] for s in vals):.0f}\\si{{\\degree}}, latched\n(drop rule off)",
                         (m + 4.5, top - 2.8), xytext=(0, 0), textcoords='offset points', ha='left', va='center',
                         fontsize=7, color=GREY_TEXT, linespacing=1.35)
            continue
        xx = m + np.linspace(-1.3, 1.3, len(vals))
        for x, s in zip(xx, vals):
            axb.plot(x, s['dth'], 'o', ms=3.4, color='k', mew=0.8, zorder=3)
        xs.append(m)
        means.append(np.mean([s['dth'] for s in vals]))
    axb.plot(xs, means, color='0.6', lw=0.8, zorder=2)
    for runs, g, col, filled, label, off in ((G150P, 150, PLANNED, False, "planner's pull,\ndropped", (7, -1)),
                                             (G180D, 180, 'k', True, 'drop rule on:\ndropped at\ndetection', (4, -7))):
        m = margin_mm(g)
        for j, r in enumerate(runs):
            s = sc[r]
            x = m + (j - 0.5 * (len(runs) - 1)) * 2.6
            axb.plot(x, s['dth'], 'o', ms=3.4, color=col, mfc=col if filled else 'white', mew=0.8, zorder=3)
            lower_bound_arrow(axb, x, s['dth'], col, dx=0.0, dy=2.6)
        axb.annotate(label, (m + 1.3 * (len(runs) - 1), min(sc[r]['dth'] for r in runs)), xytext=off,
                     textcoords='offset points', ha='left', va='top', fontsize=7,
                     color=col if col != 'k' else GREY_TEXT)
    rig_pts = []
    for tag, g, pull in RIG_F12:
        s = rig_score(tag)
        print(f"F12b rig {tag}: dtheta {s['dth']:.2f} to LAND ({s['hold']:.1f} s)")
        axb.plot(margin_mm(g) + 4.5, s['dth'], 'D', ms=3.2, color='k', mfc='k', mew=0.6, zorder=4)
        rig_pts.append((margin_mm(g) + 4.5, s['dth']))
    for xr, yr in rig_pts:
        axb.annotate('rig', (xr, yr), xytext=(4, -1), textcoords='offset points', ha='left', va='top', fontsize=7,
                     color=GREY_TEXT)
    for tag, g, pull in RIG_F12_OFF:
        s = rig_off_scale(tag)
        print(f"F12b rig off scale {tag}: dtheta >= {s['dth']:.2f} ({s['end']}, {s['window']:.2f} s)")
        xo = margin_mm(g) + 4.5
        axb.annotate('', xy=(xo, top - 1.2), xytext=(xo, top - 4.4),
                     arrowprops=dict(arrowstyle='-|>', lw=0.7, color=PLANNED, mutation_scale=6, shrinkA=0, shrinkB=0))
        axb.annotate(f"rig, magnets\nlet go, $\\geq${s['dth']:.0f}\\si{{\\degree}}", (xo, top - 2.8), xytext=(4, 0),
                     textcoords='offset points', ha='left', va='center', fontsize=7, color=PLANNED, linespacing=1.35)
    axb.axhline(DROP_DEG, color='0.6', lw=0.6, ls=(0, (1, 1.5)), zorder=0)
    axb.annotate('drop rule (absolute tilt)', (124, DROP_DEG), xytext=(0, -2), textcoords='offset points', ha='right',
                 va='top', fontsize=7, color=GREY_TEXT)
    axb.set_xlim(-6, 124)
    axb.set_xticks([0, 30, 60, 90, 120])
    axb.set_ylim(0, top)
    axb.set_yticks([0, 10, 20, 30])
    axb.set_xlabel('Support margin $m$ (mm)')
    axb.set_ylabel('Tilt rise $\\Delta\\theta$ (\\si{\\degree})')
    sec = axb.secondary_xaxis('top', functions=(lambda m: 2 * np.degrees(np.arccos(np.clip(m / PLATE_R_MM, -1, 1))),
                                                lambda g: margin_mm(g)))
    sec.set_xticks(sorted(TIER_B))
    sec.set_xticklabels([f'{g}' for g in sorted(TIER_B)], fontsize=7)
    sec.set_xlabel('Largest gap after the release (\\si{\\degree})', fontsize=8)
    sec.spines['top'].set_linewidth(0.6)
    ps.panel_label(axa, 'a', dx_pt=-100)
    ps.panel_label(axb, 'b', dx_pt=-30)
    stem = os.path.join(OUT, 'F12_levers', 'levers')
    print('\n'.join(ps.save_print(fig, stem)))
    return sc


if __name__ == '__main__':
    which = sys.argv[1:] or ['crossover', 'levers']
    for w in which:
        {'crossover': fig_crossover, 'levers': fig_levers}[w]()
