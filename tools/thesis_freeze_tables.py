#!/usr/bin/env python3
"""tools/thesis_freeze_tables.py -- thesis tables T10 (the switch itself) and T13 (the twin against the rig), from the
run data of the detach freeze tree (R1887-R1919, day-1016 8c468bd = week5 4b3560e's flight code), the frozen W4/W5
bands (docs/rig/freeze_2026-10-14.md) and the 7-8 Oct rig releases. No hand-typed numbers; no run ids in the output
(they are in configs/thesis_figures.yaml and docs/figures/PROVENANCE.md).

    python3 tools/thesis_freeze_tables.py          # both tables
    python3 tools/thesis_freeze_tables.py t10      # one
    python3 tools/thesis_freeze_tables.py t13

Writes docs/figures/tables/T10_switch.tex and T13_parity.tex (complete table floats with generated captions, for
\\input in place of the skeleton tables tab:T10 and tab:T13) and prints a per-run listing (with run ids) for the note
docs/analysis/thesis_tables_T10_T13_2026-10-16.md.

T10, the switch: the planner's resize when a carrier leaves (its log.csv `n` falls). Per switch, on the planner tick at
which `n` first falls (the first live solve of the smaller problem) and over the 1 s after it: acados status, the
logged KKT residuals (stationarity `res_stat`, dynamics `res_eq`; inequality and complementarity are not logged), QP
iterations and SQP calls, the solve time, and the survivors' throttle step. Only ticks with a fresh solve count
(`published` set): log.csv repeats the last solve's numbers on the creep and hold ticks and after a path ends, so
whole-run statistics over every row are wrong. The throttle step is m1_metrics' definition moved to the switch: per
survivor, the largest |thr_out - its mean over the 0.1 s before the switch| in the 0.3 s after it, over the survivor's
hover throttle (its mean 3 s to 1 s before the switch), largest survivor; beside it the same statistic at every planner
tick 6 s to 1 s before the switch (no event in the window: the null). Twin solve times are not reported (the lab PC
flew four runs at once); the rig's are, against its 0.06 s solve budget.

T13: the Delta-theta parity (freeze 1.1, tools/release_bands.py). Scored cells W4/W5: the twin on the freeze tree
(W4 = R1887 only: the reviewer, departed_disarm_alone card), the band frozen 11 Oct, the rig (none flown yet). The
8 Oct pairs: agreement only, each pair cut at the rig flight's hold (Q1), ratio = rig / twin mean (Q6).
"""
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import release_bands as rb  # noqa: E402
from run_dir import manifest_azimuths_ccw  # noqa: E402

REPO = os.path.dirname(TOOLS)
RESULTS = rb.RESULTS
OUT = os.path.join(REPO, 'docs', 'figures', 'tables')

# ── run lists (provenance: configs/thesis_figures.yaml, docs/figures/PROVENANCE.md) ─────────────────────────────────
FREEZE_TWIN = [f'R{i}' for i in range(1887, 1920)]        # every run of the detach freeze batch (lab batch 15)
# rig flights whose planner resized (7-8 Oct; on 14-15 Oct the planner never changed size): (tag, kind, pull, rod plan)
RIG_SWITCH = [('2026-10-07/r0012', 'announced', 'planned', 45),
              ('2026-10-07/r0013', 'announced', 'planned', 45),
              ('2026-10-08/r302_3', 'announced', 'measured', 60),
              ('2026-10-08/r303_4', 'unannounced', 'measured', 60),
              ('2026-10-08/r304_3', 'unannounced', 'measured', 60),
              ('2026-10-08/r305', 'unannounced', 'measured', 60)]
RIG_NO_SWITCH_DAYS = ['2026-10-14', '2026-10-15']
# T13
W4_TWIN, W5_TWIN = ['R1887'], ['R1913', 'R1914']           # the freeze tree (W4: R1887 only, the reviewer)
W4_BAND_SET, W5_BAND_SET = ['R1459', 'R1460', 'R1626'], ['R1457', 'R1627']   # freeze 2.4, the bands' inputs
W4_RIG, W5_RIG = [], []                                    # rig:<date>/<run> once flown and converted
PAIR_LABELS = {'r302_3': 'Hover, announced', 'r303_4': 'Hover, unannounced',
               'r304_3': 'Hover, unannounced, 150$^\\circ$ gap', 'r305': 'Circle, unannounced'}

SWITCH_POST_S = 1.0          # the window after the switch for the worst tick
STEP_PRE, STEP_POST = 0.1, 0.3   # m1_metrics' hand-over step windows
HOVER_WIN = (3.0, 1.0)       # the survivor's hover throttle: mean from 3 s to 1 s before the switch
NULL_WIN = (6.0, 1.0)        # null instants: planner ticks from 6 s to 1 s before the switch
RIG_BUDGET_MS = 60.0         # bringup/config/real.yaml solve_budget_s
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')


# ── locating the logs ─────────────────────────────────────────────────────────────────────────────────────────────
def twin_dir(rid):
    hits = [h for h in glob.glob(os.path.join(RESULTS, '2*', rid + '_*')) if os.path.isdir(h)]
    if len(hits) != 1:
        sys.exit(f'{rid}: {len(hits)} run directories under {RESULTS}')
    return hits[0]


def logs_dir(tag):
    """The node logs (planner, trackers) of a twin run (R####) or a rig flight (<date>/<run>)."""
    if tag.startswith('R'):
        return os.path.join(twin_dir(tag), 'logs')
    return os.path.join(RESULTS, 'rig', tag + '_logs', 'logs')


def planner_log(tag):
    f = sorted(glob.glob(os.path.join(logs_dir(tag), '*planner', '*', 'log.csv')), key=os.path.getsize)
    if not f:
        return None
    return pd.read_csv(f[-1]).drop_duplicates('sim_time').reset_index(drop=True)


def tracker_logs(tag):
    out = {}
    for f in sorted(glob.glob(os.path.join(logs_dir(tag), 'tracker', 'drone*_*', 'log.csv'))):
        k = int(re.search(r'drone(\d+)_', f).group(1))
        T = pd.read_csv(f, usecols=lambda c: c in ('sim_time', 'thr_out'))
        out[k] = T.drop_duplicates('sim_time').reset_index(drop=True)
    return out


def config_name(rid):
    return os.path.basename(twin_dir(rid)).split('_sim_gz_', 1)[-1]


def manifest(rid):
    return json.load(open(os.path.join(twin_dir(rid), 'manifest.json')))


def cut_instant(rid):
    """Run time (run.csv t) of the first planner DROP AND LAND or tracker ENVELOPE FAULT line, else None: the log's
    epoch stamps mapped to run time on the fleet manager's ARM line, as tools/drop_metrics.py does."""
    d = twin_dir(rid)
    log = open(os.path.join(d, 'logs', 'launch.log'), errors='replace').read().splitlines()
    hit = next((ln for ln in log if re.search(r'DROP AND LAND|ENVELOPE FAULT', ln) and STAMP.search(ln)), None)
    if hit is None:
        return None, None
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'wall']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    t_arm = float(ev.sim_time[ev.event == 'ARM'].iloc[0])
    arm = next(float(STAMP.search(ln).group(1)) for ln in log
               if "Fleet command received: 'ARM'" in ln and STAMP.search(ln))
    epoch0 = arm - float(np.interp(t_arm, run.t, run.wall))
    t = float(np.interp(float(STAMP.search(hit).group(1)) - epoch0, run.wall, run.t))
    return t, ('drop' if 'DROP AND LAND' in hit else 'fault')


def node_offset(rid):
    """Node clock minus run.csv time for a twin run: lift-off of the ring in the planner log against run.csv
    (release_transient.py's alignment)."""
    d = twin_dir(rid)
    P = planner_log(rid)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_z']).drop_duplicates('t')
    lift = lambda t, z: float(t[z > np.median(z[:20]) + 0.03].iloc[0])  # noqa: E731
    return lift(P.sim_time, P.load_z) - lift(run.t, run.payload_z)


# ── T10: the switch ───────────────────────────────────────────────────────────────────────────────────────────────
def fresh(P):
    """Ticks with a fresh solve (published solved / shifted / none); creep and hold rows repeat the last solve."""
    return P[P.published.notna() & (P.published.astype(str) != '')]


def thr_step(T, t0, post_end=None):
    """Largest |thr - mean over [t0 - STEP_PRE, t0)| in [t0, t0 + STEP_POST) over the hover throttle (%)."""
    t, thr = T.sim_time.to_numpy(float), T.thr_out.to_numpy(float)
    hov = thr[(t >= t0 - HOVER_WIN[0]) & (t < t0 - HOVER_WIN[1])]
    pre = thr[(t >= t0 - STEP_PRE) & (t < t0)]
    end = t0 + STEP_POST if post_end is None else min(t0 + STEP_POST, post_end)
    post = thr[(t >= t0) & (t < end)]
    if len(hov) < 5 or len(pre) < 1 or len(post) < 1 or not np.isfinite(hov).all():
        return None
    return 100.0 * float(np.max(np.abs(post - pre.mean()))) / float(hov.mean())


def switch(tag, cut_node=None):
    """The resize in one run: None if the planner never changed size."""
    P = planner_log(tag)
    if P is None:
        return None
    n = P.n.astype(int).to_numpy()
    i = np.flatnonzero(np.diff(n) < 0)
    if not len(i):
        return None
    i = int(i[0]) + 1
    t_res = float(P.sim_time[i])
    F = fresh(P)
    first = P.iloc[i]
    win = F[(F.sim_time >= t_res) & (F.sim_time < t_res + SWITCH_POST_S)]
    pre = F[(F.sim_time >= t_res - NULL_WIN[0]) & (F.sim_time < t_res - NULL_WIN[1])]
    after = [int(v) for v in str(first.slot2drone).split()]
    T = tracker_logs(tag)
    steps = {k: thr_step(T[k], t_res, cut_node) for k in after if k in T}
    steps = {k: v for k, v in steps.items() if v is not None}
    null = []
    for tk in pre.sim_time:
        v = [thr_step(T[k], float(tk)) for k in after if k in T]
        v = [x for x in v if x is not None]
        if v:
            null.append(max(v))
    out = dict(tag=tag, t_res=t_res, n_from=int(n[i - 1]), n_to=int(n[i]), survivors=after,
               published=str(first.published), status=int(first.solve_status),
               fails_1s=int((win.solve_status.astype(int) != 0).sum()), ticks_1s=len(win),
               res_stat=float(first.res_stat), res_eq=float(first.res_eq),
               res_stat_prev=float(P.res_stat[i - 1]), res_eq_prev=float(P.res_eq[i - 1]),
               res_stat_max_1s=float(win.res_stat.max()), res_eq_max_1s=float(win.res_eq.max()),
               res_stat_pre_med=float(pre.res_stat.median()), res_stat_pre_max=float(pre.res_stat.max()),
               res_eq_pre_med=float(pre.res_eq.median()), res_eq_pre_max=float(pre.res_eq.max()),
               qp_iter=int(first.qp_iter), sqp_calls=int(first.sqp_calls),
               qp_iter_max_1s=int(win.qp_iter.max()), sqp_min_1s=int(win.sqp_calls.min()),
               solve_ms=float(first.solve_ms), solve_ms_max_1s=float(win.solve_ms.max()),
               solve_ms_pre_p50=float(pre.solve_ms.median()), solve_ms_pre_p95=float(np.percentile(pre.solve_ms, 95)),
               over_budget_1s=int((win.solve_ms > RIG_BUDGET_MS).sum()),
               thr_step=max(steps.values()) if steps else None, thr_steps=steps,
               thr_null_p50=float(np.median(null)) if null else None,
               thr_null_max=float(np.max(null)) if null else None,
               fails_run=int((F.solve_status.astype(int) != 0).sum()), solves_run=len(F))
    return out


def largest_gap(az):
    a = sorted(v % 360.0 for v in az)
    return max([b - x for x, b in zip(a, a[1:])] + [a[0] + 360.0 - a[-1]])


def twin_switches():
    rows, none = [], []
    for rid in FREEZE_TWIN:
        man = manifest(rid)
        src = re.search(r'cable_source:\s*(\w+)', open(os.path.join(twin_dir(rid), 'params', 'tracker_0.yaml')).read())
        pull = 'planned' if src and src.group(1) == 'model' else 'measured'      # the tracker's read-back
        cut, kind = cut_instant(rid)
        cut_node = cut + node_offset(rid) if cut is not None else None
        s = switch(rid, cut_node)
        tag = dict(run=rid, config=config_name(rid), pull=pull, git=man.get('git_sha', '')[:7],
                   dirty=len(man.get('dirty_files') or []), cut=kind)
        if s is None:
            none.append(tag)
            continue
        la = man.get('launch_args', {})
        if not isinstance(la, dict):
            la = dict(x.split(':=', 1) for x in la if ':=' in x)
        az = manifest_azimuths_ccw(la.get('attach_azimuths_deg'), man)     # drone k sits at az[k]
        s.update(tag)
        s['gap'] = largest_gap([az[k] for k in range(len(az)) if k in s['survivors']])
        s['cut_after_switch_s'] = (cut_node - s['t_res']) if cut_node is not None else None
        rows.append(s)
    return rows, none


NO_SWITCH_CONTROL = [(['R1889', 'R1893', 'R1897'], ['R1887', 'R1891', 'R1895']),    # detection off (A5) / on (A1)
                     (['R1890', 'R1894', 'R1898'], ['R1888', 'R1892', 'R1896'])]    # planned pull: A6 / A2


def no_switch_steps(twin):
    """The same throttle statistic in the detection-off runs (no resize), at the release plus the paired arm's mean
    detection delay: what the release alone does to the survivors' throttle in that window."""
    by = {r['run']: r for r in twin}
    out = []
    for off_runs, on_runs in NO_SWITCH_CONTROL:
        d = float(np.mean([by[r]['t_res'] - node_offset(r) - rb.Run(r).te for r in on_runs]))
        surv = by[on_runs[0]]['survivors']
        vals = []
        for rid in off_runs:
            t0 = rb.Run(rid).te + node_offset(rid) + d
            T = tracker_logs(rid)
            v = [thr_step(T[k], t0) for k in surv if k in T]
            vals.append(max(x for x in v if x is not None))
        out.append(dict(off=off_runs, on=on_runs, delay=d, steps=vals,
                        on_steps=[by[r]['thr_step'] for r in on_runs]))
    return out


def rig_switches():
    rows = []
    for tag, kind, pull, plan in RIG_SWITCH:
        s = switch(tag)
        if s is None:
            continue
        s.update(run=tag, kind=kind, pull=pull, plan=plan)
        rows.append(s)
    return rows


def rig_days_without_switch():
    """Every planner log of the given rig days whose team never changed size (the switch was not flown)."""
    out = {}
    for day in RIG_NO_SWITCH_DAYS:
        logs = glob.glob(os.path.join(RESULTS, 'rig', day, '*_logs', 'logs', '*planner', '*', 'log.csv'))
        same = 0
        for f in logs:
            n = pd.read_csv(f, usecols=['n']).n.dropna().astype(int)
            same += int(n.nunique() <= 1)
        out[day] = (same, len(logs))
    return out


def sci(x):
    """1.2e-4 -> 1.2\\times10^{-4} (LaTeX, math mode)."""
    if x is None or not np.isfinite(x):
        return '--'
    m, e = f'{x:.1e}'.split('e')
    return f'{m}\\times10^{{{int(e)}}}'


def rng(vals, fmt='{:.0f}'):
    vals = [v for v in vals if v is not None and np.isfinite(v)]
    if not vals:
        return '--'
    lo, hi = min(vals), max(vals)
    a, b = fmt.format(lo), fmt.format(hi)
    return a if a == b else f'{a}--{b}'


def t10_cells(rows, rig):
    fails = sum(r['fails_1s'] for r in rows)
    ticks = sum(r['ticks_1s'] for r in rows)
    status = ('0' if all(r['status'] == 0 for r in rows) else
              ', '.join(str(r['status']) for r in rows)) + f'; {fails} of {ticks}'
    kkt = f'${sci(max(r["res_stat"] for r in rows))}$ / ${sci(max(r["res_eq"] for r in rows))}$'
    solve = f'{rng([r["solve_ms"] for r in rows])}; {rng([r["solve_ms_max_1s"] for r in rows])}' if rig else '--'
    steps = [r['thr_step'] for r in rows if r['thr_step'] is not None]
    nulls = [r['thr_null_max'] for r in rows if r['thr_null_max'] is not None]
    thr = f'{rng(steps, "{:.1f}")} ({rng(nulls, "{:.1f}")})' if steps else '--'
    return status, kkt, solve, thr


def table_t10():
    twin, none = twin_switches()
    rig = rig_switches()
    gaps = lambda rows: rng([r['gap'] for r in rows])  # noqa: E731
    meas = [r for r in twin if r['pull'] == 'measured' and r['gap'] < 179.5]
    meas180 = [r for r in twin if r['pull'] == 'measured' and r['gap'] >= 179.5]
    plan = [r for r in twin if r['pull'] == 'planned']
    groups = [
        ('Leave, announced', "Rig, planner's pull$^a$", [r for r in rig if r['kind'] == 'announced' and r['pull'] == 'planned'], True),
        ('', 'Rig, measured pull', [r for r in rig if r['kind'] == 'announced' and r['pull'] == 'measured'], True),
        ('Leave, unannounced', f'Twin, measured pull, gap {gaps(meas)}$^\\circ$', meas, False),
        ('', f'Twin, measured pull, gap {gaps(meas180)}$^\\circ$$^b$', meas180, False),
        ('', f"Twin, planner's pull, gap {gaps(plan)}$^\\circ$$^c$", plan, False),
        ('', 'Rig, measured pull', [r for r in rig if r['kind'] == 'unannounced'], True),
    ]
    lines = []
    for sw, src, rows, is_rig in groups:
        if not rows:
            continue
        st, kkt, solve, thr = t10_cells(rows, is_rig)
        lines.append(f'{sw} & {src} ({len(rows)}) & {st} & {kkt} & {solve} & {thr} \\\\')
    lines.append('Join (hand-over) & [the join tree] & [\\dots] & [\\dots] & [\\dots] & [\\dots] \\\\')
    ns = no_switch_steps(twin)
    prev_stat = [r['res_stat_prev'] for r in twin + rig]
    prev_eq = [r['res_eq_prev'] for r in twin + rig]
    pre_stat = [r['res_stat_pre_max'] for r in twin + rig]
    pre_eq = [r['res_eq_pre_max'] for r in twin + rig]
    pre_p95 = [r['solve_ms_pre_p95'] for r in rig]
    over = sum(r['over_budget_1s'] for r in rig)
    ticks = sum(r['ticks_1s'] for r in rig)
    days = rig_days_without_switch()
    planned_rig = [r for r in rig if r['kind'] == 'announced' and r['pull'] == 'planned']
    cut_b = [r['cut_after_switch_s'] for r in meas180 if r['cut_after_switch_s'] is not None]
    cut_c = [r['cut_after_switch_s'] for r in plan if r['cut_after_switch_s'] is not None
             and r['cut_after_switch_s'] < SWITCH_POST_S]
    notes = [
        f"$^a$ Rods planned at {planned_rig[0]['plan']}$^\\circ$ (7 October); in one of the two, two magnets then let "
        "go with no command and the ring capsized (Figure~\\ref{fig:F11})." if planned_rig else '',
        (f"$^b$ No support margin; the drop rule off, so the fleet latched at 60$^\\circ$ of tilt "
         f"{rng(cut_b, '{:.1f}')}\\,s after the switch." if cut_b else ''),
        (f"$^c$ In {len(cut_c)} of {len(plan)} runs (the 150$^\\circ$ gap) the trackers' 30$^\\circ$ rule dropped the "
         f"ring {rng(cut_c, '{:.1f}')}\\,s after the switch; the throttle window ends there." if cut_c else ''),
    ]
    cap = (
        "The switch itself (T10): the planner's first solve of the smaller problem when a UAV leaves, and the second "
        "after it. Data: rig, the 7--8 October releases; twin, the detach freeze code tree; $n$ switches. Status: "
        "acados's status at the first solve; then the failed solves among all solves in the second after it. KKT "
        "residual: stationarity and dynamics at the first solve, the largest in the row (the inequality and "
        "complementarity residuals are not logged); the last solve of the larger problem, one tick earlier, reached "
        f"${sci(max(prev_stat))}$ and ${sci(max(prev_eq))}$, and steady hover ${sci(max(pre_stat))}$ and "
        f"${sci(max(pre_eq))}$. Solve time, rig only: the first solve; the longest in the "
        f"second after it, against the rig's 60\\,ms budget ({over} of {ticks} solves over it; the four-UAV problem "
        f"before the switch: {rng(pre_p95)}\\,ms at its 95th percentile). Throttle step: the survivors' largest "
        "throttle departure in the 0.3\\,s after the switch from their mean over the 0.1\\,s before it, over their "
        "hover throttle; in brackets, the largest of the same statistic at every planner tick of the 5\\,s of steady "
        "hover that end 1\\,s before the switch. In the twin runs with detection off, which never switch, the same "
        f"window after the release reads {rng(ns[0]['steps'], '{:.1f}')}\\,\\% on the measured pull and "
        f"{rng(ns[1]['steps'], '{:.1f}')}\\,\\% on the planner's. Not computed: the gap to an offline SQP solve (the planner keeps a "
        "solve's QP only when it fails); the twin's solve times (four runs shared one computer). The rig flights of "
        f"14--15 October never changed the team's size ({sum(v[1] for v in days.values())} planner logs).")
    tex = (r"""\begin{table}[htbp]
  \centering
  \footnotesize
  \caption{""" + cap + r"""}
  \label{tab:T10}
  \begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}p{1.6cm}>{\raggedright\arraybackslash}Xcccc}
    \toprule
    Switch & Data ($n$) & Status; failed & KKT residual (stat / eq) & Solve time (ms) & Throttle step (\%) \\
    \midrule
""" + '\n'.join('    ' + ln for ln in lines) + r"""
    \bottomrule
""" + '\n'.join('    \\multicolumn{6}{p{\\dimexpr\\textwidth-2\\tabcolsep}}{\\footnotesize ' + nt + '} \\\\' for nt in notes if nt) + r"""
  \end{tabularx}
\end{table}""")
    return 'T10_switch', tex, dict(twin=twin, none=none, rig=rig, days=days, ns=ns)


# ── T13: the twin against the rig ─────────────────────────────────────────────────────────────────────────────────
def dth(tag, horizon):
    return rb.Run(tag).metrics(horizon)['dtheta']


def fmt_set(vals, nd=1):
    if not vals:
        return '--'
    lo, hi = min(vals), max(vals)
    s = f'{lo:.{nd}f}' if len(vals) == 1 or f'{lo:.{nd}f}' == f'{hi:.{nd}f}' else f'{lo:.{nd}f}--{hi:.{nd}f}'
    return f'{s}, {len(vals)}'


class A:          # release_bands' signed defaults (freeze 2.4)
    pair_horizon, classes, ratio, sigma, band_horizon = 'rig', 'announce', 'mean', 'pre', rb.BAND_HORIZON_S


def table_t13():
    rows, fac = rb.factors(A)
    lo_f, hi_f, npairs = fac[rb.class_key('unannounced', 'hover', 'announce')]
    sig, _ = rb.sigma_rig(A)
    hz = rb.BAND_HORIZON_S
    cells = []
    for cell, twin, band_set, rig in (('W4', W4_TWIN, W4_BAND_SET, W4_RIG), ('W5', W5_TWIN, W5_BAND_SET, W5_RIG)):
        tw = [dth(r, hz) for r in twin]
        bs = [dth(r, hz) for r in band_set]
        lo = max(0.0, min(bs) * lo_f - 2 * sig)
        hi = max(bs) * hi_f + 2 * sig
        rg = [dth(r, hz) for r in rig]
        cells.append(dict(cell=cell, twin=tw, band_set=bs, band=(lo, hi), rig=rg,
                          inside=all(lo <= v <= hi for v in tw)))
    agree = []
    for rig_tag, twins, key, h, r, tw, ratio, _ in rows:
        agree.append(dict(rig=rig_tag, twins=twins, cls=key, hold=h, rig_dth=r, twin_dth=tw, ratio=ratio))
    lines = []
    names = {'W4': 'Hover, unannounced', 'W5': 'Circle, unannounced'}
    for c in cells:
        rig = fmt_set(c['rig']) if c['rig'] else 'not yet flown'
        ratio = (f'{np.mean(c["rig"]) / np.mean(c["twin"]):.2f}' if c['rig'] else '--')
        lines.append(f'{names[c["cell"]]} & {fmt_set(c["twin"])} & {rig} & {ratio} & after & '
                     f'{c["band"][0]:.2f}--{c["band"][1]:.2f} & {"yes" if c["rig"] else "when flown"} \\\\')
    lines.append(r'\midrule')
    lines.append(r'\multicolumn{7}{l}{\emph{8 October pairs (agreement, not scored)}} \\')
    for a in agree:
        name = PAIR_LABELS[a['rig'].split('/')[-1]]
        lines.append(f'{name} & {fmt_set(a["twin_dth"])} & {fmt_set([a["rig_dth"]])} & {a["ratio"]:.2f} & before & '
                     f'-- & no \\\\')
    w4, w5 = cells
    unann = [a['ratio'] for a in agree if a['cls'] == 'unannounced']
    cap = (
        "The twin against the rig (T13), on the tilt rise $\\Delta\\theta$ (the departure of the ring's tilt vector "
        "from its mean over the 5\\,s before the release). Upper rows, scored: the hover and circle releases of the "
        "ledger (unannounced, the model integral, the freed UAV disarmed alone), $\\Delta\\theta$ from the release to "
        "15\\,s after it; twin: the detach freeze code tree, for the hover only the run released at the band's timing; "
        "band: frozen on 11 October from twin sets on the 14 October tree (hover "
        f"{fmt_set(w4['band_set'], 1).split(',')[0]}$^\\circ$, {len(w4['band_set'])} runs; circle "
        f"{fmt_set(w5['band_set'], 1).split(',')[0]}$^\\circ$, {len(w5['band_set'])} runs), the unannounced class "
        f"factors {lo_f:.2f} and {hi_f:.2f} and $2\\sigma_\\mathrm{{rig}} = {2 * sig:.1f}^\\circ$; the freeze-tree twin "
        f"runs {'lie' if w4['inside'] and w5['inside'] else 'do not all lie'} inside their bands. Neither release has "
        "flown on the rig. Lower rows: the four 8 October flights against the twin flying the same configuration "
        "(the reference integral, the freed UAV landing with the fleet), both cut at the rig flight's own hold "
        f"({min(a['hold'] for a in agree):.1f}--{max(a['hold'] for a in agree):.1f}\\,s); factor: the rig's "
        "$\\Delta\\theta$ over the twin pair's mean, the ledger's class factor (the unannounced class spans "
        f"{min(unann):.2f}--{max(unann):.2f}). Rate fix: before or after the rig's body rates were sent at full "
        "precision (8 October, evening). Cells give the range and $n$.")
    tex = (r"""\begin{table}[htbp]
  \centering
  \footnotesize
  \caption{""" + cap + r"""}
  \label{tab:T13}
  \begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}Xcccccc}
    \toprule
    Cell & Twin $\Delta\theta$ ($^\circ$), $n$ & Rig $\Delta\theta$ ($^\circ$), $n$ & Factor & Rate fix & Band ($^\circ$) & Scored \\
    \midrule
""" + '\n'.join('    ' + ln for ln in lines) + r"""
    \bottomrule
  \end{tabularx}
\end{table}""")
    return 'T13_parity', tex, dict(cells=cells, agree=agree, lo_f=lo_f, hi_f=hi_f, sigma=sig, npairs=npairs)


# ── the per-run listing for the note ──────────────────────────────────────────────────────────────────────────────
def print_t10(d):
    f = lambda x, nd=2: '-' if x is None else f'{x:.{nd}f}'  # noqa: E731
    print('\n| run | config / kind | pull | n | status | stat / eq at the switch | max stat / eq in 1 s | pre max stat / eq '
          '| tick before: stat / eq | qp iter / sqp | solve ms: first, max 1 s (pre p50 / p95) | throttle step % (per survivor) | null p50 / max '
          '| cut after switch (s) |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in d['twin'] + d['rig']:
        steps = ' '.join(f'd{k}:{v:.1f}' for k, v in r['thr_steps'].items())
        cut = r.get('cut_after_switch_s')
        print(f"| {r['run']} | {r.get('config', r.get('kind'))} | {r['pull']} | {r['n_from']}->{r['n_to']} | "
              f"{r['status']} ({r['fails_1s']}/{r['ticks_1s']} failed in 1 s) | {r['res_stat']:.1e} / {r['res_eq']:.1e} | "
              f"{r['res_stat_max_1s']:.1e} / {r['res_eq_max_1s']:.1e} | {r['res_stat_pre_max']:.1e} / "
              f"{r['res_eq_pre_max']:.1e} | {r['res_stat_prev']:.1e} / {r['res_eq_prev']:.1e} | "
              f"{r['qp_iter']} / {r['sqp_calls']} (min sqp in 1 s {r['sqp_min_1s']}) | {r['solve_ms']:.1f}, "
              f"{r['solve_ms_max_1s']:.1f} ({r['solve_ms_pre_p50']:.1f} / {r['solve_ms_pre_p95']:.1f}) | "
              f"{f(r['thr_step'], 1)} ({steps}) | {f(r['thr_null_p50'], 1)} / {f(r['thr_null_max'], 1)} | "
              f"{'' if cut is None else f'{cut:.2f} (' + str(r.get('cut')) + ')'} |")
    print('\nno resize:', ', '.join(f"{t['run']} {t['config']}" + (f" ({t['cut']})" if t['cut'] else '') for t in d['none']))
    for c in d['ns']:
        print(f"no-switch control: {', '.join(c['off'])} at release + {c['delay']:.2f} s: "
              f"{' / '.join(f'{v:.1f}' for v in c['steps'])} % against the switching arm {', '.join(c['on'])}: "
              f"{' / '.join(f'{v:.1f}' for v in c['on_steps'])} %")
    print('trees:', sorted({(t['git'], t['dirty']) for t in d['twin'] + d['none']}))
    print('rig days without a switch (logs with one team size / logs):', d['days'])


def print_t13(d):
    print('\n| cell | twin (freeze tree) | band set | band | inside | rig |')
    print('|---|---|---|---|---|---|')
    for c in d['cells']:
        tw = W4_TWIN if c['cell'] == 'W4' else W5_TWIN
        bs = W4_BAND_SET if c['cell'] == 'W4' else W5_BAND_SET
        print(f"| {c['cell']} | {', '.join(f'{r} {v:.2f}' for r, v in zip(tw, c['twin']))} | "
              f"{', '.join(f'{r} {v:.2f}' for r, v in zip(bs, c['band_set']))} | [{c['band'][0]:.2f}, {c['band'][1]:.2f}] | "
              f"{c['inside']} | {c['rig'] or 'not flown'} |")
    print('\n| rig flight | twin runs | class | hold (s) | rig dtheta | twin dtheta | ratio |')
    print('|---|---|---|---|---|---|---|')
    for a in d['agree']:
        print(f"| {a['rig']} | {', '.join(a['twins'])} | {a['cls']} | {a['hold']:.1f} | {a['rig_dth']:.2f} | "
              f"{' / '.join(f'{v:.2f}' for v in a['twin_dth'])} | {a['ratio']:.2f} |")
    print(f"class unannounced f_lo {d['lo_f']:.3f} f_hi {d['hi_f']:.3f} ({d['npairs']} pairs); sigma_rig {d['sigma']:.3f}")


def main():
    which = sys.argv[1:] or ['t10', 't13']
    os.makedirs(OUT, exist_ok=True)
    for w in which:
        name, tex, data = {'t10': table_t10, 't13': table_t13}[w]()
        open(os.path.join(OUT, name + '.tex'), 'w').write(
            '% generated by tools/thesis_freeze_tables.py; run ids in docs/figures/PROVENANCE.md\n' + tex + '\n')
        print(f'== {name}\n{tex}')
        (print_t10 if w == 't10' else print_t13)(data)


if __name__ == '__main__':
    main()
