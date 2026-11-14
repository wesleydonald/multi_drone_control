#!/usr/bin/env python3
"""X4, the endurance relay. 6 Nov: re-drawn from the frozen tree's claim runs (F054: R3772, R3773 relays, R3775,
R3776 fixed teams, results/analysis/x4_2026-11-05.json); each pack now ~700 s against the 600 s budget.
(5 Nov, Wesley: draw the headline as a figure; current tree, re-run on the weekend re-fly).

The one sentence: swapping one quadrotor per lap keeps the ring flying for about 1760 s, 2.9 battery budgets, while no
quadrotor flies longer than its 600 s budget, and the ring follows its path no worse at the end than at the start.
7 Nov: re-drawn from the final runs on the earlier masses (F062/F063: R3910, R3911 relays, R3913, R3914 fixed teams).
Hero: (a) each quadrotor's timeline in one relay (0.9 kg ring), from take-off to touchdown as the mission is scored:
carrying the ring (blue), away from it (light grey: out to its pad and back, or one of Q3's two errands) with each
recharge marked, and the fixed team's flight above it (red), on one time axis from the
lift. (b) the contour error per lap against time for both relays at 0.9 kg (blue) and both fixed teams (red).
Revised after its blind critic (5 Nov): the relay's and fixed team's flights as rows with their lap counts (the
relay pauses its path for each return, so it flies 28 laps to the fixed team's 24), the errands named, one battery
bracketed against the 600 s budget, (b) against the lap number, not time.
Left out: the 1.15 kg relay (the same shape: in the table), tilt per event (in the table and text).
Data: results/2026-11-03 R3519, R3564 (relays), R3566, R3567 (fixed teams); events.csv (LIFTED, DETACH_RESIZE, TASK_AT,
TASK_PARKED, REJOIN_WELD, LAND); per-lap contour error and packs as tools/x4_metrics.py scores them
(results/analysis/x4_2026-11-04.json).
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402
sys.path.insert(0, S.TOOLS)
import x4_metrics as XM  # noqa: E402

S.HERE = HERE
X4 = os.path.join(S.REPO, 'results', 'analysis', 'x4_oldlinks_2026-11-06.json')     # F062/F063: final, earlier masses
RELAY, FIXED = 'R3910', 'R3913'
RELAYS, FIXEDS = ('R3910', 'R3911'), ('R3913', 'R3914')
BUDGET = 600.0


def tint(colour, a):
    rgb = np.array(S.plt.matplotlib.colors.to_rgb(colour))
    return tuple(1.0 - a * (1.0 - rgb))


def run_dir(rid):
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '2026-11-0*', f'{rid}_*'))
    return d


def events(rid):
    d = run_dir(rid)
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()].copy()
    ev['sim_time'] = ev.sim_time.astype(float)
    return ev


def drone_of(arg):
    s = str(arg).strip()
    return int(s.split()[0]) if s and s.split()[0].isdigit() else None


def timeline(rid):
    """drone -> list of (start, end, kind) with kind 'carry' or 'away', and the recharge times (the pad leaves of the
    relay's plan; its two errands are away but not a recharge); times from the take-off, as x4_metrics' mission."""
    ev = events(rid)
    t0 = float(ev.loc[ev.event == 'TAKEOFF', 'sim_time'].iloc[0])
    t_end = float(ev.loc[ev.event == 'LANDED', 'sim_time'].iloc[0])
    plan = XM.load_config(run_dir(rid)).relay
    kinds = {}
    for lv in (plan.leaves if plan is not None else []):
        kinds.setdefault(lv.d, []).append(lv.kind)
    seen = {}
    n = 8
    spans = {i: [] for i in range(n)}
    charge = {i: [] for i in range(n)}
    start = {i: t0 for i in range(n)}
    for r in ev.itertuples():
        i = drone_of(r.arg)
        if r.event == 'DETACH_RESIZE' and i is not None:
            spans[i].append((start[i] - t0, r.sim_time - t0, 'carry'))
            start[i] = r.sim_time
        elif r.event == 'REJOIN_WELD' and i is not None:
            spans[i].append((start[i] - t0, r.sim_time - t0, 'away'))
            start[i] = r.sim_time
        elif r.event == 'TASK_PARKED' and i is not None:
            k = seen.get(i, 0)
            seen[i] = k + 1
            if kinds.get(i, [])[k:k + 1] == ['pad']:
                charge[i].append(r.sim_time - t0)
    for i in range(n):
        spans[i].append((start[i] - t0, t_end - t0, 'carry'))
    return spans, charge, t_end - t0


def main():
    rows = {r['run'].split('_')[0]: r for r in json.load(open(X4))}
    S.setup()
    fig = S.figure('full', 4.0)
    a, b = fig.subplots(2, 1, gridspec_kw={'height_ratios': [2.3, 1.0]})
    spans, charge, t_relay = timeline(RELAY)
    _, _, t_fixed = timeline(FIXED)
    print('recharges', {i: len(v) for i, v in charge.items()}, 'mission', round(t_relay, 1), round(t_fixed, 1))
    h = 0.62
    away_c = tint(S.GREY, 0.35)
    for i in range(8):
        y = 8 - i
        for s0, s1, kind in spans[i]:
            errand = kind == 'away' and not any(s0 <= tc <= s1 for tc in charge[i])
            a.barh(y, s1 - s0, left=s0, height=h, color=S.BLUE if kind == 'carry' else away_c, lw=0,
                   hatch='/////' if errand else None, edgecolor=S.GREY if errand else None)
        for tc in charge[i]:
            a.plot([tc, tc], [y - h / 2, y + h / 2], color=S.INK, lw=1.0, solid_capstyle='butt')
    y_relay, y_fixed = 9.55, 10.55
    a.barh(y_relay, t_relay, left=0, height=h, color=S.BLUE, lw=0)
    a.barh(y_fixed, t_fixed, left=0, height=h, color=S.RED, lw=0)
    n_relay, n_fixed = len(rows[RELAY]['laps']), len(rows[FIXED]['laps'])
    a.annotate(f'{t_relay:.0f}\\,s, {n_relay} laps', xy=(t_relay, y_relay), xytext=(-4, 0), textcoords='offset points',
               ha='right', va='center', fontsize=8, color='white')
    a.annotate(f'{t_fixed:.0f}\\,s, {n_fixed} laps', xy=(t_fixed, y_fixed), xytext=(4, 0), textcoords='offset points',
               ha='left', va='center', fontsize=8, color=S.RED)
    # one battery on Q1: from the end of its first recharge to its next pad arrival (x4_metrics' pack)
    packs = rows[RELAY]['packs']
    print('packs', {k: [round(x) for x in v] for k, v in packs.items()}, 'max', rows[RELAY]['max_pack_s'])
    p0, p1 = charge[7][0], charge[7][0] + packs['7'][1]
    yb = 1 - h / 2 - 0.3
    a.annotate('', xy=(p1, yb), xytext=(p0, yb), arrowprops=dict(arrowstyle='<->', lw=0.6, color=S.INK, shrinkA=0,
                                                                 shrinkB=0))
    a.annotate(f'One battery, {packs["7"][1]:.0f}\\,s of the 600\\,s budget', xy=(0.5 * (p0 + p1), yb),
               xytext=(0, -1.5), textcoords='offset points', ha='center', va='top', fontsize=8, color=S.INK)
    a.set_yticks(list(range(1, 9)) + [y_relay, y_fixed])
    a.set_yticklabels([f'Q{9 - y}' for y in range(1, 9)] + ['Relay', 'Fixed formation'])
    a.set_ylim(-0.75, 11.05)
    a.tick_params(axis='y', length=0)
    a.set_xlim(0, 1820)
    a.set_xlabel('Time after takeoff (s)')

    for rids, col, name in ((RELAYS, S.BLUE, 'Relay'), (FIXEDS, S.RED, 'Fixed formation')):
        for k, rid in enumerate(rids):
            laps = rows[rid]['laps']
            n = [l['lap'] for l in laps]
            e = [1000 * l['path_rms_m'] for l in laps]
            b.plot(n, e, '-o', color=col, lw=S.LW_CTX, ms=2.4, alpha=1.0 if k == 0 else 0.5)
            print(rid, f'{len(laps)} laps, first {e[0]:.1f}, last {e[-1]:.1f} mm')
            if k == 0 and name == 'Relay':
                b.annotate(name, xy=(n[-1], e[-1]), xytext=(4, 0), textcoords='offset points', ha='left',
                           va='center', fontsize=8, color=col)
            elif k == 0:
                b.annotate(name, xy=(n[5], e[5]), xytext=(0, -5), textcoords='offset points', ha='center',
                           va='top', fontsize=8, color=col)
    b.annotate('Pale: second run', xy=(1.0, 0.0), xycoords='axes fraction', xytext=(-3, 3),
               textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.INK)
    b.set_ylabel('Contour\nerror (mm)')
    b.set_xlim(0, 33)
    b.set_ylim(0, 38)
    b.set_yticks([0, 10, 20, 30])
    b.set_xticks([1, 5, 10, 15, 20, 25, 28])
    b.set_xlabel('Lap')
    S.panel(a, 'a', 'Quadrotors over the flight')
    S.panel(b, 'b', 'Contour error per lap')
    fig.align_ylabels((a, b))
    import matplotlib.patches as mp
    import matplotlib.lines as ml
    keys = [mp.Patch(color=S.BLUE), mp.Patch(color=away_c),
            mp.Patch(facecolor=away_c, hatch='/////', edgecolor=S.GREY, lw=0),
            ml.Line2D([], [], ls='none', marker='|', ms=7, mew=1.0, color=S.INK), mp.Patch(color=S.RED)]
    S.ps.legend_below(fig, keys, ['Relay, on the payload', 'Away', 'Away, errand', 'Recharge', 'Fixed formation'], ncol=5)
    S.save(fig, 'fig_x4_relay')


# ---- layout options (Wesley 14 Nov: "(a) is not very nice"); FIG25_VARIANT=A|B|C saves fig_x4_relay_<V>, unset = the
# figure above, unchanged
RED_TEXT, BLUE_TEXT = '#7d111e', '#174778'


def carriers(spans, t_end):
    """Quadrotors on the payload against time (step, post), from takeoff to touchdown, then 0."""
    ts = sorted({x for v in spans.values() for a, b, _ in v for x in (a, b)})
    t, n = [], []
    for a, b in zip(ts[:-1], ts[1:]):
        m = 0.5 * (a + b)
        t.append(a)
        n.append(sum(1 for v in spans.values() for s0, s1, k in v if k == 'carry' and s0 <= m < s1))
    t += [t_end]
    n += [0]                        # the drop at touchdown, nothing drawn after it
    return np.array(t), np.array(n)


def error_panel(b, rows, repeats=True):
    """(b) contour error per lap, both runs of each (or the first only); the key is below, nothing written in the box."""
    h = {}
    for rids, col, name in ((RELAYS, S.BLUE, 'Relay'), (FIXEDS, S.RED, 'Fixed formation')):
        for k, rid in enumerate(rids if repeats else rids[:1]):
            laps = rows[rid]['laps']
            n = [l['lap'] for l in laps]
            e = [1000 * l['path_rms_m'] for l in laps]
            c = col if k == 0 else tint(col, 0.45)
            ln, = b.plot(n, e, '-o', color=c, lw=S.LW_CTX, ms=2.4, zorder=3 - k)
            h[name + ('' if k == 0 else ', repeat')] = ln
    b.set_ylabel('Contour\nerror (mm)')
    b.set_xlim(0, 29)
    b.set_ylim(0, 38)
    b.set_yticks([0, 10, 20, 30])
    b.set_xticks([1, 5, 10, 15, 20, 25, 28])
    b.set_xlabel('Lap')
    return h


def variant(v):
    import matplotlib.lines as ml
    import matplotlib.patches as mp
    rows = {r['run'].split('_')[0]: r for r in json.load(open(X4))}
    S.setup()
    if v == 'A':
        fig = S.figure('full', 3.3)
        a, b = fig.subplots(2, 1, gridspec_kw={'height_ratios': [1.2, 1.0]})
        for rid, col, txt in ((RELAY, S.BLUE, BLUE_TEXT), (FIXED, S.RED, RED_TEXT)):
            spans, _, t_end = timeline(rid)
            t, n = carriers(spans, t_end)
            a.step(t, n, where='post', color=col, lw=S.LW_HERO, zorder=3)
            a.annotate(f'{t_end:.0f}\\,s', xy=(t_end, 3.6), xytext=(-4, 0), textcoords='offset points', ha='right',
                       va='center', fontsize=8, color=txt)
            print(rid, 'quadrotors on the payload', sorted(set(n[:-1].tolist())), 'until', round(t_end, 1))
        a.axvline(BUDGET, **S.REF)
        a.set_ylim(0, 9.4)
        a.set_yticks([0, 2, 4, 6, 8])
        a.set_ylabel('Quadrotors')
        a.set_xlim(0, 1820)
        a.set_xlabel('Time after takeoff (s)')
        S.panel(a, 'a', 'Quadrotors carrying the payload')
        h = error_panel(b, rows)
        S.panel(b, 'b', 'Contour error per lap')
        keys = [h['Relay'], h['Relay, repeat'], h['Fixed formation'], h['Fixed formation, repeat'],
                ml.Line2D([], [], **{k: v for k, v in S.REF.items() if k != 'zorder'})]
        labels = ['Relay', 'Relay, repeat', 'Fixed formation', 'Fixed formation, repeat', 'Battery budget, 600\\,s']
        ncol = 3
    elif v == 'B':
        fig = S.figure('full', 3.8)
        a, b = fig.subplots(2, 1, gridspec_kw={'height_ratios': [1.9, 1.0]})
        spans, _, t_relay = timeline(RELAY)
        _, _, t_fixed = timeline(FIXED)
        hb = 0.6
        for i in range(8):
            y = 8 - i
            for s0, s1, kind in spans[i]:
                if kind == 'carry':
                    a.barh(y, s1 - s0, left=s0, height=hb, color=S.BLUE, lw=0)
        a.axvline(t_fixed, color=S.RED, lw=1.0, zorder=4)
        a.set_yticks(range(1, 9))
        a.set_yticklabels([f'Q{9 - y}' for y in range(1, 9)])
        a.tick_params(axis='y', length=0)
        a.set_ylim(0.4, 8.6)
        a.set_xlim(0, 1820)
        a.set_xlabel('Time after takeoff (s)')
        S.panel(a, 'a', 'Time on the payload in the relay')
        h = error_panel(b, rows)
        S.panel(b, 'b', 'Contour error per lap')
        keys = [mp.Patch(color=S.BLUE), ml.Line2D([], [], color=S.RED, lw=1.0), h['Relay'], h['Relay, repeat'],
                h['Fixed formation'], h['Fixed formation, repeat']]
        labels = ['On the payload', 'Fixed formation lands', 'Relay', 'Relay, repeat', 'Fixed formation',
                  'Fixed formation, repeat']
        ncol = 3
        print('relay', round(t_relay, 1), 'fixed', round(t_fixed, 1))
    else:   # C: laps flown against time, the battery budget marked (3.8 in: the page keeps its old break)
        fig = S.figure('full', 3.8)
        a, b = fig.subplots(2, 1, gridspec_kw={'height_ratios': [1.3, 1.0]})
        for rids, col in ((RELAYS, S.BLUE), (FIXEDS, S.RED)):
            for k, rid in enumerate(rids[:1]):      # Wesley 14 Nov: option C without the repeats
                laps = rows[rid]['laps']
                t = [0.0] + [l['t1'] for l in laps]
                n = list(range(len(laps) + 1))
                c = col if k == 0 else tint(col, 0.45)
                a.step(t, n, where='post', color=c, lw=S.LW_HERO if k == 0 else S.LW_CTX, zorder=3 - k)
                a.plot(t[-1], n[-1], 'o', ms=S.MS, color=c, zorder=4 - k)
                print(rid, f'{len(laps)} laps by {t[-1]:.1f} s, mission {rows[rid]["mission_s"]:.1f} s')
        a.axvline(BUDGET, **S.REF)
        a.set_ylim(0, 31)
        a.set_yticks([0, 10, 20, 30])
        a.set_ylabel('Laps')
        a.set_xlim(0, 1820)
        a.set_xlabel('Time after takeoff (s)')
        S.panel(a, 'a', 'Laps completed')
        h = error_panel(b, rows, repeats=False)
        S.panel(b, 'b', 'Contour error per lap')
        keys = [h['Relay'], h['Fixed formation'], ml.Line2D([], [], **{k: v for k, v in S.REF.items() if k != 'zorder'})]
        labels = ['Relay', 'Fixed formation', 'Battery budget, 600\\,s']
        ncol = 3
    fig.align_ylabels((a, b))
    S.ps.legend_below(fig, keys, labels, ncol=ncol)
    S.save(fig, 'fig_x4_relay' if v == 'C' else f'fig_x4_relay_{v}')


if __name__ == '__main__':
    V = os.environ.get('FIG25_VARIANT', 'C')     # Wesley 14 Nov: option C (laps against time) without the repeats
    variant(V) if V != 'old' else main()
