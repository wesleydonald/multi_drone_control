#!/usr/bin/env python3
"""tools/thesis_sim_tables.py -- the simulation-results tables for thesis/Source/src/sim_results.tex,
written to thesis/Source/src/tables/*.tex from the run data (no hand-typed numbers; no run ids in the
output, they are listed in configs/thesis_figures.yaml).

    python3 tools/thesis_sim_tables.py

An arm is a config name (every non-void run of it, for today's box-at-zero configs) or an explicit
run list (the old-box baselines, whose configs were also flown on older code). Cells show the mean
over the arm's runs and n.
"""
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from method_metrics import metrics  # noqa: E402
from push_metrics import push, plan_prediction  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, 'thesis', 'Source', 'src', 'tables')


def runs_of(config):
    ids = []
    for d in sorted(glob.glob(os.path.join(REPO, 'results', '*', f'R*_sim_gz_{config}'))):
        rid = os.path.basename(d).split('_')[0]
        if not os.path.exists(os.path.join(d, 'metrics.json')):
            continue                                   # still flying
        m = metrics(rid)
        if m.get('arm') == 'void':
            continue
        ev = {}
        for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))):
            ev.setdefault(e['event'], float(e['sim_time']))
        if 'FLEET_ABORT' in ev and ev['FLEET_ABORT'] < ev.get('LAND', 1e9):
            continue                                   # aborted in flight
        ids.append(rid)
    return ids


def land_aborts(arm):
    n = 0
    for r in ids(arm):
        d = glob.glob(os.path.join(REPO, 'results', '*', f'{r}_*'))[0]
        n += any(e['event'] == 'FLEET_ABORT' for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    return n


def ids(arm):
    return list(arm) if isinstance(arm, (list, tuple)) else runs_of(arm)


def cell(vals, fmt='{:.1f}', signed=False):
    vals = [v for v in vals if v is not None and np.isfinite(v)]
    if not vals:
        return '--'
    f = ('{:+.1f}' if signed else fmt)
    v = float(np.mean(vals))
    s = f.format(0.0 if abs(v) < 0.05 and signed else v).replace('+0.0', '0.0')
    s = s.replace('-', '$-$') if s.startswith('-') else s
    return s + (f' ({len(vals)})' if len(vals) > 1 else ' (1)')


def registry():
    return {r[0]: r for r in csv.reader(open(os.path.join(REPO, 'results', 'registry.csv')))}


def table_x0():
    hold = {'old': ['R0841', 'R0844'], 'new': 'b2_cap075_fix_n3_x0b0'}
    orbit = {'old': ['R0832'], 'new': 'b2_cap075_fix_n3_orbit_x0b0'}
    rows = [
        ('Gazebo, hover at 1.0\\,m', 'height error (cm)',
         cell([metrics(r)['z_err_cm'] for r in ids(hold['old'])], signed=True),
         cell([metrics(r)['z_err_cm'] for r in ids(hold['new'])], signed=True)),
        ('Gazebo, circle', 'horizontal error, mean (cm)',
         cell([metrics(r)['err_cm'] for r in ids(orbit['old'])]),
         cell([metrics(r)['err_cm'] for r in ids(orbit['new'])])),
        ('', 'height error (cm)',
         cell([metrics(r)['z_err_cm'] for r in ids(orbit['old'])], signed=True),
         cell([metrics(r)['z_err_cm'] for r in ids(orbit['new'])], signed=True)),
    ]
    body = '\n'.join(f'{a} & {b} & {c} & {d} \\\\' for a, b, c, d in rows)
    return ('x0', r"""\begin{tabular}{llcc}
\toprule
Case & Metric & $\epsilon = 0.025$ & $\epsilon = 0$ \\
\midrule
""" + body + r"""
\bottomrule
\end{tabular}""")


def push_cells(arm):
    P = [push(r) for r in ids(arm)]
    return [cell([p['steady_cm'] for p in P]), cell([p['peak_cm'] for p in P]),
            cell([p['release_overshoot_cm'] for p in P]), cell([onset_settle(p['run']) for p in P])]


def onset_settle(rid):
    import pandas as pd
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    ev = [e for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))) if e['event'] == 'WRENCH']
    on, off = float(ev[0]['sim_time']), float(ev[1]['sim_time'])
    f = np.array([float(v) for v in ev[0]['arg'].split()[:3]])
    ax = int(np.argmax(np.abs(f)))
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    t = run.t.to_numpy()
    p = run['payload_' + 'xyz'[ax]].to_numpy()
    y = np.sign(f[ax]) * (p - p[(t >= on - 5) & (t < on)].mean()) * 100
    out = np.flatnonzero((t >= on) & (t < off) & (np.abs(y) > 1.0))
    return float(t[out[-1] + 1] - on) if out.size and out[-1] < len(t) - 1 and t[out[-1] + 1] < off else np.nan


def learned(arm, axis):
    import pandas as pd
    vals = []
    for r in ids(arm):
        d = glob.glob(os.path.join(REPO, 'results', '*', f'{r}_*'))[0]
        L = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
        run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
        off_t = float(L.sim_time[L.load_z > 0.3].iloc[0]) - float(run.t[run.payload_z > 0.3].iloc[0])
        ev = [e for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))) if e['event'] == 'WRENCH']
        on, off = float(ev[0]['sim_time']) + off_t, float(ev[1]['sim_time']) + off_t
        c = L[f'int_f{axis}']
        pre = c[(L.sim_time >= on - 5) & (L.sim_time < on)].mean()
        end = c[(L.sim_time >= off - 3) & (L.sim_time < off)].mean()
        vals.append(abs(end - pre))
    return vals


def table_disturbance():
    Z = dict(none='b2_cap075_fix_n3_pushz_x0b0', ref='b2_cap075_fix_n3_pushz_x0b0_zki',
             model='b2_cap075_fix_n3_pushz_x0b0_int')
    X = dict(none='b2_cap075_fix_n3_push04_x0b0', model='b2_cap075_fix_n3_push04_x0b0_int')
    M = dict(ref='b2_cap075_fix_n3_mapneg_x0b0_zki', model='b2_cap075_fix_n3_mapneg_x0b0_int')
    zc = {k: push_cells(v) for k, v in Z.items()}
    xc = {k: push_cells(v) for k, v in X.items()}
    names = ['steady displacement (cm)', 'peak displacement (cm)', 'overshoot on release (cm)',
             'time to within 1\\,cm after onset (s)']
    lines = [r'\multicolumn{4}{l}{\emph{0.6\,N downward push on the ring}} \\']
    lines += [f'\\quad {n} & {zc["none"][i]} & {zc["ref"][i]} & {zc["model"][i]} \\\\' for i, n in enumerate(names)]
    lines += [f'\\quad change of the integral force (N) & -- & -- & {cell(learned(Z["model"], "z"), "{:.2f}")} \\\\']
    lines += [r'\midrule', r'\multicolumn{4}{l}{\emph{0.4\,N horizontal push on the ring}} \\']
    lines += [f'\\quad {n} & {xc["none"][i]} & n/a & {xc["model"][i]} \\\\' for i, n in enumerate(names)]
    lines += [f'\\quad change of the integral force (N) & -- & n/a & {cell(learned(X["model"], "x"), "{:.2f}")} \\\\']
    lines += [r'\midrule', r'\multicolumn{4}{l}{\emph{One UAV producing more thrust than its map assumes}} \\']
    mm = {k: [metrics(r) for r in ids(v)] for k, v in M.items()}
    for key, name, signed in (('z_err_cm', 'height error (cm)', True), ('err_cm', 'horizontal error (cm)', False),
                              ('tilt', 'ring tilt (\\si{\\degree})', False)):
        lines += [f'\\quad {name} & -- & {cell([m[key] for m in mm["ref"]], signed=signed)} & '
                  f'{cell([m[key] for m in mm["model"]], signed=signed)} \\\\']
    return ('disturbance', r"""\small
\begin{tabular}{lccc}
\toprule
 & No integral & Target shift & Model integral \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabular}""")


def table_gain():
    arms = [('$k_i = 0.4$', ['R0969', 'R0982']), ('$k_i = 0.8$', ['R0976']),
            ('$k_i = 0.4$, speed gate only in the first 5\\,s after the lift', ['R0977'])]
    lines = []
    for name, rr in arms:
        P = [push(r) for r in rr]
        lines.append(f'{name} & {cell([p["peak_cm"] for p in P])} & {cell([p["release_overshoot_cm"] for p in P])} & '
                     f'{cell([p["settle_s"] for p in P])} \\\\')
    return ('gain', r"""\begin{tabularx}{\textwidth}{Xccc}
\toprule
Model integral & Peak (cm) & Overshoot (cm) & Settling (s) \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabularx}""")


def trajectory_error(rid):
    """Horizontal error (cm) over the whole trajectory, where the reference moves, sampled when the
    planner's 10 Hz reference updates (not against a stale step)."""
    import pandas as pd
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    ch = (run.payload_ref_x.diff().abs() > 1e-6) | (run.payload_ref_y.diff().abs() > 1e-6)
    t = run.t[ch]
    w = run[ch & (run.t >= t.iloc[0]) & (run.t <= t.iloc[-1])]
    return 100 * np.hypot(w.payload_x - w.payload_ref_x, w.payload_y - w.payload_ref_y).to_numpy()


def table_four():
    reg = registry()
    rows = []
    for name, rr in (('Hover, even ring', ['R0931', 'R0932']), ('Hover, plates 1/3/5/9', ['R0936'])):
        M = [metrics(r) for r in rr]
        rows.append(f'{name} & height error {cell([m["z_err_cm"] for m in M], signed=True)}\\,cm, '
                    f'tilt {cell([m["tilt"] for m in M])}\\,\\si{{\\degree}} \\\\')
    m = metrics('R0933')
    rows.append(f'Circle, $r = 0.5$\\,m, 0.125\\,m/s (1) & horizontal error {m["err_cm"]:.1f}\\,cm mean, '
                f'{m["err_max_cm"]:.1f}\\,cm max; height error {m["z_err_cm"]:+.1f}\\,cm \\\\')
    e = trajectory_error('R0935')
    rows.append(f'Figure-8, 2\\,m $\\times$ 1\\,m, 0.25\\,m/s (1) & horizontal error {np.mean(e):.1f}\\,cm mean, '
                f'{np.max(e):.1f}\\,cm max \\\\')
    dt = [v.strip() for v in reg['R0939'][7].split('/')]
    rows.append(f'Detach, 4 to 3 (plates 1/3/5/9) (1) & peak tilt {dt[0].split()[0]}\\,\\si{{\\degree}}, '
                f'under 2\\,\\si{{\\degree}} after {dt[1].split()[0]}\\,s, ring dip {dt[2].split()[0]}\\,cm \\\\')
    return ('four', r"""\begin{tabularx}{\textwidth}{lX}
\toprule
Case & Result \\
\midrule
""" + '\n'.join(r.replace('+', '$+$') for r in rows) + r"""
\bottomrule
\end{tabularx}""")


STRAIGHT = ['R0960', 'R0961', 'R0962', 'R0963', 'R0967', 'R0968', 'R0969', 'R0971', 'R0972', 'R0974', 'R0976',
            'R0977', 'R0978', 'R0979', 'R0980', 'R0981', 'R0982', 'R0983', 'R0984', 'R0985', 'R0986']


def card_runs(card):
    """Non-void runs registered against a card, in order."""
    out = []
    for r in csv.reader(open(os.path.join(REPO, 'results', 'registry.csv'))):
        if len(r) > 9 and card in r[3] and r[0].startswith('R') and 'VOID' not in r[9]:
            out.append(r[0])
    return out


def table_land():
    from land_metrics import land
    sets = [('straight descent', STRAIGHT),
            ('unwind', card_runs('2026-10-04_land_matrix')),
            ('unwind, feedforward ramp, ground idle', card_runs('2026-10-04_land_fix'))]
    lines = []
    for name, rr in sets:
        R = [land(r) for r in rr]
        R = [x for x in R if 'outside_cm' in x or 'ABORT' in x.get('outcome', '')]
        ok = [x for x in R if x.get('outcome') == 'landed' and 'outside_cm' in x]
        ab = sum('ABORT' in x.get('outcome', '') for x in R)
        tip = sum(('ABORT' in x.get('outcome', '')) or x.get('tilt_deg', 0) > 45 for x in R)
        clean = [x for x in ok if x['tilt_deg'] <= 45]
        rng = lambda k: f"{min(x[k] for x in clean):.1f}--{max(x[k] for x in clean):.1f}"
        lines.append(f'{name} & {len(R)} & {tip} ({ab}) & {rng("outside_cm")} & {rng("tilt_deg")} \\\\')
    return ('land', r"""\begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}Xcccc}
\toprule
Landing & Runs & Tipped (aborted) & Pushed outward (cm) & Tilt after 1.2\,s (\si{\degree}) \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabularx}""")


ROBUST = [('none', ['R1013', 'R1023'], ''),
          ('payload mass typed $+10$\\,\\%', 'm+10', ''),
          ('payload mass typed $+20$\\,\\%', 'm+20', '$^a$'),
          ('payload mass typed $-20$\\,\\%', 'm-20', ''),
          ('rods typed $+5$\\,cm, measured at the hand-over', 'rod+5', ''),
          ('rods typed $+5$\\,cm, not measured', 'rod+5nomeas', '$^b$'),
          ('attachment radius typed $+2$\\,cm', 'r+2', '')]


def robust_runs(arm, zki):
    if isinstance(arm, list) and not zki:
        return arm
    return runs_of(f'b2_cap075_fix_n3_rob_{arm if isinstance(arm, str) else "none"}_{"zki" if zki else "z0"}')


def target_shift(rid):
    """The height integral's target shift at LAND (cm, + = target raised)."""
    import pandas as pd
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    L = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
    return 100 * float(L.z_bias.dropna().iloc[-1])


def table_robust():
    """T9. Every cell is the mean of two Gazebo runs (the caption says so; a cell with another n fails)."""
    from land_metrics import land
    lines = []
    for name, arm, mark in ROBUST:
        r0, r1 = robust_runs(arm, False), robust_runs(arm, True)
        assert len(r0) == 2 and len(r1) == 2, (name, r0, r1)
        sg = lambda v: cell(v, signed=True).split(' (')[0].replace('+', '$+$')
        R = [land(r) for r in r0 + r1]
        ok = sum(x.get('outcome') == 'landed' and x.get('tilt_deg', 90) <= 45 for x in R)
        lines.append(f'{name} & {sg([metrics(r)["z_err_cm"] for r in r0])} & '
                     f'{sg([metrics(r)["z_err_cm"] for r in r1])}{mark} & {sg([target_shift(r) for r in r1])} & '
                     f'{cell([metrics(r)["tilt"] for r in r0 + r1]).split(" (")[0]} & {ok}/{len(R)} \\\\')
    return ('robust', r"""\begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}Xccccc}
\toprule
 & \multicolumn{2}{c}{Height error (cm)} & Target & & \\
\cmidrule(lr){2-3}
Error in the controller's model & no integral & integral & shift (cm) & Tilt (\si{\degree}) & Landed \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabularx}""")


PARAMS = [('Fleet and payload', [('drone_mass', 'UAV mass, with rod and magnet', 'kg'), ('load_mass', 'payload mass', 'kg'),
                                 ('cable_len', 'rod length', 'm'), ('pivot_offset_z', 'rod pivot below the UAV centre', 'm'),
                                 ('attach_radius', 'attachment radius', 'm')]),
          ('Thrust map', [('thrust_offset', 'throttle offset $a_i$ (shared)', ''),
                          ('thrust_offset_v_slope', 'voltage slope', '1/V'),
                          ('thrust_ratio', 'gain above the offset, $k_T$', 'm/s$^2$'), ('throttle_max', 'throttle cap', '')]),
          ('Takeoff and lift', [('creep_vel', 'creep speed', 'm/s'), ('handover_elev_deg', 'rod elevation at the hand-over', '\\si{\\degree}'),
                                ('pretension_s', 'pretension', 's'), ('z_taut_gate', 'tautness gate, measured over typed rod', ''),
                                ('lift_ramp_vel', 'lift ramp', 'm/s')]),
          ('Height integral', [('z_ki', 'gain $k_i$', '1/s'), ('z_i_max', 'bound', 'm'), ('z_i_gate', 'gate: height error under', 'm')]),
          ('Landing', [('land_vel', 'descent speed', 'm/s'), ('land_tol', 'touchdown tolerance', 'm')]),
          ('Reconfiguration', [('min_survivors', 'fewest UAVs after a detach', ''), ('weld_radius', 'weld radius', 'm')]),
          ('Watchdogs', [('pose_timeout_s', 'motion-capture timeout', 's'), ('solve_budget_s', 'planner solve budget', 's')])]


def rig_profile():
    import yaml
    cfg = {}
    for f in ('common.yaml', 'real.yaml'):
        for sec, kv in yaml.safe_load(open(os.path.join(REPO, 'src', 'bringup', 'config', f))).items():
            if sec != 'modes':
                cfg.update(kv)
    return cfg


def table_params():
    cfg = rig_profile()
    lines = []
    for group, rows in PARAMS:
        lines.append(f'\\multicolumn{{3}}{{l}}{{\\textit{{{group}}}}} \\\\')
        for key, label, unit in rows:
            v = cfg[key]
            lines.append(f'\\quad {label} & {abs(v):g} & {unit} \\\\')
    return ('params', r"""\begin{tabular}{llc}
\toprule
Parameter & Value & Unit \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabular}""")


TRAJ = [('Hover', 3, '--', 'b2_cap075_fix_n3_x0b0_lf', 'hover'),
        ('Hover, plates 1/3/5/9', 4, '--', 'w4_3915_hover_lf', 'hover'),
        ('Line, 1\\,m shuttle', 4, '0.25', 'w4_even_line_lf', 'all'),
        ('Circle, $r = 0.5$\\,m', 3, '0.125', ['b2_cap075_fix_n3_orbit_x0b0', 'b2_cap075_fix_n3_orbit_x0b0_lu',
                                               'b2_cap075_fix_n3_orbit_x0b0_lf'], 'lap'),
        ('Circle, $r = 0.5$\\,m', 4, '0.125', 'w4_even_orbit_lf', 'lap'),
        ('Figure-8, 2\\,m $\\times$ 1\\,m', 4, '0.25', 'w4_even_fig8_lf', 'all'),
        ('Spin, circle with one turn of yaw', 4, '0.125', 'w4_even_spin_lf', 'all')]


def traj_stats(rid, window):
    """Horizontal error per sample (mean, max), mean height error and mean tilt: the 15 s before LAND
    (hover), the circle's first lap from motion onset, or the whole motion; sampled where the 10 Hz
    reference updates (F_traj)."""
    import pandas as pd
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    land = min(float(e['sim_time']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))
               if e['event'] == 'LAND')
    ch = (run.payload_ref_x.diff().abs() > 1e-6) | (run.payload_ref_y.diff().abs() > 1e-6)
    if window == 'hover':
        w = run[(run.t >= land - 15) & (run.t < land)]
    else:
        u = run[ch]
        sp = np.hypot(u.payload_ref_x.diff(), u.payload_ref_y.diff()) / u.t.diff()
        moving = u.t[(sp > 0.01) & (u.t < land)]
        t0 = float(moving.iloc[0])
        t1 = min(t0 + 27.2, land) if window == 'lap' else float(moving.iloc[-1])
        w = u[(u.t >= t0) & (u.t <= t1)]
    e = 100 * np.hypot(w.payload_x - w.payload_ref_x, w.payload_y - w.payload_ref_y)
    return dict(mean=float(e.mean()), max=float(e.max()), z=100 * float((w.payload_z - w.payload_ref_z).mean()),
                tilt=float(w.payload_tilt_deg.mean()))


def spin_yaw_err(rid):
    """Ring yaw against the spin's yaw_at(traj_t) over the turn (deg; |mean|, max), the start yaw removed."""
    import pandas as pd
    sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))
    from mpc_planner.load_trajectory import LoadTrajectory
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    L = pd.read_csv(glob.glob(os.path.join(d, 'logs', '*planner', '*', 'log.csv'))[0])
    off = float(L.sim_time[L.load_z > 0.3].iloc[0]) - float(run.t[run.payload_z > 0.3].iloc[0])
    tr = LoadTrajectory('spin', 0.125, 1.0, 0.5)
    M = L[(L.traj_t > 0) & (L.traj_t < tr._circle_theta(0.0)[3])]
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy()
    yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
    y = np.interp(M.sim_time - off, run.t, yaw)
    e = np.degrees(np.angle(np.exp(1j * (y - y[0] - np.array([tr.yaw_at(t)[0] for t in M.traj_t])))))
    return float(np.abs(e).mean()), float(np.abs(e).max())


DETACH = [('$\\epsilon = 0.025$, height integral on', ['R0939']),
          ('$\\epsilon = 0$, no integral', ['R0997', 'R1007', 'R1012', 'R1029', 'R1034'])]


def detach_stats(rid):
    """F_detach's measures for one run: peak ring tilt in the 5 s after DETACH; time until the tilt stays
    under the hover's own maximum (5 s before) plus 0.5 deg; largest ring height change from its mean
    over the 5 s before, in the 12 s after."""
    import pandas as pd
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    t_det = [float(e['sim_time']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))
             if e['event'] == 'DETACH'][0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    w = run[(run.t >= t_det - 5) & (run.t <= t_det + 12)]
    tr, tilt = w.t.to_numpy() - t_det, w.payload_tilt_deg.to_numpy()
    band = float(tilt[tr < 0].max()) + 0.5
    over = np.flatnonzero((tr >= 0) & (tilt > band))
    z = w.payload_z.to_numpy()
    return dict(peak=float(tilt[(tr >= 0) & (tr < 5)].max()), settle=float(tr[over[-1] + 1]) if over.size else 0.0,
                dz=100 * float(np.max(np.abs(z[tr >= 0] - z[tr < 0].mean()))))


def table_detach():
    lines = []
    for name, rr in DETACH:
        S = [detach_stats(r) for r in rr]
        rng = lambda k: (f'{S[0][k]:.1f}' if len(S) == 1 else
                         f'{min(x[k] for x in S):.1f}--{max(x[k] for x in S):.1f}')
        lines.append(f'{name} & {len(S)} & {rng("peak")} & {rng("settle")} & {rng("dz")} \\\\')
    return ('detach', r"""\begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}Xcccc}
\toprule
 & & Peak tilt & Back in band & Height change \\
Tracker box & Runs & (\si{\degree}) & (s) & (cm) \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\end{tabularx}""")


def table_traj():
    lines, note = [], ''
    for name, n, v, arm, window in TRAJ:
        rr = sum((runs_of(a) for a in arm), []) if isinstance(arm, list) else runs_of(arm)
        S = [traj_stats(r, window) for r in rr]
        c = lambda k, **kw: cell([x[k] for x in S], **kw).split(' (')[0].replace('+', '$+$')
        mark = '$^a$' if 'spin' in str(arm) else ''
        lines.append(f'{name}{mark} & {n} & {v} & {c("mean")} & {c("max")} & {c("z", signed=True)} & {c("tilt")} & '
                     f'{len(S) if S else "--"} \\\\')
        if mark:
            Y = [spin_yaw_err(r) for r in rr]
            note = (f'$^a$ Yaw error {np.mean([y[0] for y in Y]):.1f}\\,\\si{{\\degree}} mean, '
                    f'{max(y[1] for y in Y):.1f}\\,\\si{{\\degree}} max over the turn.')
    return ('traj', r"""\begin{tabularx}{\textwidth}{>{\raggedright\arraybackslash}Xccccccc}
\toprule
 & & Speed & \multicolumn{2}{c}{Horizontal error (cm)} & Height & Tilt & \\
\cmidrule(lr){4-5}
Reference & UAVs & (m/s) & mean & max & error (cm) & (\si{\degree}) & Runs \\
\midrule
""" + '\n'.join(lines) + r"""
\bottomrule
\multicolumn{8}{l}{\footnotesize """ + note + r"""}
\end{tabularx}""")


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for fn in (table_x0, table_disturbance, table_gain, table_four, table_land, table_robust, table_params, table_traj, table_detach):
        name, tex = fn()
        open(os.path.join(OUT, name + '.tex'), 'w').write('% generated by tools/thesis_sim_tables.py\n' + tex + '\n')
        print(f'== {name}\n{tex}\n')
