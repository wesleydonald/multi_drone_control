#!/usr/bin/env python3
"""
tools/planner_replay.py -- run the load planner's OCP offline, tick by tick, on the
states a rig flight logged, and toggle one thing at a time.

    tools/planner_replay.py                              # model-f1, arms base,a..f
    tools/planner_replay.py --arms base,a --src <pkg>    # another copy of mpc_planner
    tools/planner_replay.py --flight f7 --heave          # f7: replayed refs vs the logged ones

What it feeds the solver, per logged planner tick (the load planner's log.csv, one row per
_plan call, so the ticks the rig skipped while blocked are skipped here too):
  load p, vz      the logged row
  load vx, vy     differenced from drone 0's payload_x/y (50 Hz), 5-sample mean
  load q          NOT logged. Yaw = the latched datum + the mean change of the four drones'
                  azimuths about the ring; roll/pitch axis from a least-squares fit of the
                  four rod lengths (|p + R rho_i - drone_i| = measured rod), magnitude
                  rescaled to the logged tilt_deg
  load w          NOT logged: zero (arm e adds noise)
  drones          tracker pose_x/y/z interpolated at the tick
  lift schedule   z_tgt, lift_progress, z_bias from the row (lift_z0 = z_tgt - lift_progress,
                  lift_vel = the per-row change x 10 Hz), so no node state machine is re-run
Before the hand-over every row primes the solver on the hold reference, as
LoadPlanner._prime_solver does. The replay stops where the node stops solving (LAND with
the ring back within 5 cm of its lift height). A failed solve drops the warm start and
the next tick reseeds from x_init, as the node does.

Arms: base; a = solver.reset() (u = 0, multipliers cleared) before every reseed; b = cable
references at the hand-over elevation instead of 45 deg; c = rods from a pivot 4 cm below
the drone centre (body z), each the rig's rod (0.55 m, the flight's cable_len when longer: the 1 m cables);
d = load pose 0.5 s stale (drones current); e = load
angular-rate noise (sd --omega-sd rad/s); f = old geometry (radius 0.25, rod 0.53, drone
0.525 kg, no measured rods); g = the ring quaternion kept in one hemisphere (the rig's mocap
sends w >= 0, so q changes sign when the yaw crosses 180 deg); h = attach azimuths rotated
to where the drones sat at rest on the first row; y = ring yaw held at the datum instead of
following the drones; i<k> = k warm SQP iterations a tick (plan_sqp_iters; card
2026-10-23_scalability); v = the planner's plan_fast_refs (references from the mapped extraction, the
residual after them). Letters combine with '+' (a+g, i1+v).

Scalability S (card 2026-10-23_scalability):
  --quality       per warm planner tick, each arm's solve against the converged plan of the same tick
                  (tools/plan_quality.py: cost ratio, residual, reference deltas at nodes 1/5/N, node-0
                  feed-forward and tension deltas, thrust excursion); the arm's own warm-start chain is kept
  --q-source auto the ring attitude and rate from the planner log's qw..qz, wx..wz where logged (else the fit)
  --flown-pivot   rods from the flight's pivot_offset (params.json) and the references at the drone centres
  --ref-hash      a hash per tick of the plan X and of every drone's extracted references (bit-identity checks)
  --summary-json  the arms' summaries (and quality) for every flight given (--flight a,b,c)
The defaults replay as before. A flight's ring inertia comes from params.json when it typed one (load_ixx).

The solver is generated and built in --work (never the repo's c_generated_code, and its cache check and
signature are kept there too), and each arm runs in its own process: two builds of the same model name
cannot share one process. Imports come from --src first (default: the repo's src/mpc_planner).
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan_quality as PQ  # noqa: E402
import run_logs  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RIG = os.path.join(REPO, 'results', 'rig', '2026-09-30')
FLIGHTS = {'model_f1': os.path.join(RIG, 'model_f1_logs'),
           'f7': os.path.join(RIG, 'lift_f7_logs')}
DEFAULT_SRC = os.path.join(REPO, 'src', 'mpc_planner')
RIG_ROD_055 = 0.55          # the rig's rods until the 1 m cables (model-f1 typed 0.47)
DEFAULT_WORK = os.path.join(os.environ.get('TMPDIR', '/tmp'), f'mdc_planner_replay_{os.getuid()}')
G = 9.81
PIVOT_DZ = 0.04
ARMS = ['base', 'a', 'b', 'c', 'd', 'e', 'f']
ARM_TEXT = {'base': 'as flown', 'a': 'reset + u=0 on reseed', 'b': 'refs at hand-over elevation',
            'c': 'pivot 4 cm, rods 0.55', 'd': 'load pose 0.5 s stale', 'e': 'ring omega noise',
            'f': 'old geometry r0.25 l0.53 m0.525', 'g': 'quaternion sign kept continuous',
            'h': 'attach azimuths where the drones sat at rest', 'y': 'ring yaw held at the datum',
            'v': 'plan_fast_refs'}


def load_modules(src):
    """Import the planner's pure-python modules from `src` (a mpc_planner package
    directory), ahead of any installed copy."""
    sys.path.insert(0, os.path.abspath(src))
    for k in [k for k in sys.modules if k.startswith('mpc_planner')]:
        del sys.modules[k]
    import mpc_planner.geometry as geometry
    import mpc_planner.load_cable_dynamics as lcd
    import mpc_planner.params as params
    import mpc_planner.planner_solver as planner_solver
    import mpc_planner.reference_builder as reference_builder
    import mpc_planner.load_trajectory as load_trajectory
    return argparse.Namespace(geometry=geometry, lcd=lcd, params=params, ps=planner_solver,
                              rb=reference_builder, lt=load_trajectory)


# ── flight logs ───────────────────────────────────────────────────────────────

def _stamp(line):
    m = re.search(r'\[(\d{9,}\.\d+)\]', line)
    return float(m.group(1)) if m else None


def read_flight(run_dir):
    """Planner rows, per-drone tracker logs, params and the event times from the launch
    log (<run>.log beside <run>_logs)."""
    pdir = (run_logs.node_dirs(run_dir, 'mpc_planner') + run_logs.node_dirs(run_dir, 'dissipative_planner'))[-1]
    rows = pd.read_csv(os.path.join(pdir, 'log.csv'))
    with open(os.path.join(pdir, 'params.json')) as f:
        prm = json.load(f)
    n = int(prm['num_drones'])
    # the tracker sessions of the same launch start within seconds of the planner's
    t_p = rows['sim_time'].iloc[0]
    drones = []
    for i in range(n):
        best = None
        for d in run_logs.node_dirs(run_dir, 'tracker'):
            if run_logs.drone_of(d) != i:
                continue
            df = pd.read_csv(os.path.join(d, 'log.csv'))
            if len(df) < 2:
                continue
            if best is None or abs(df['sim_time'].iloc[0] - t_p) < abs(best['sim_time'].iloc[0] - t_p):
                best = df
        drones.append(best)
    # rig: <run>.log beside <run>_logs; twin: <run>/logs/launch.log
    log_path = (run_dir[:-len('_logs')] + '.log' if run_dir.rstrip('/').endswith('_logs')
                else os.path.join(run_dir, 'logs', 'launch.log'))
    ev = {'fail_times': []}
    with open(log_path, errors='replace') as f:
        for line in f:
            if not any(tag in line for tag in ('[mpc_planner]', '[load_planner]', '[dissipative_planner]')):
                continue
            t = _stamp(line)
            if 'yaw datum latched at' in line:
                ev['psi0_deg'] = float(re.search(r'latched at ([-+0-9.]+)', line).group(1))
            elif 'auto slot assignment' in line:
                ids = [int(v) for v in re.search(r'\): \[([0-9, ]+)\]', line).group(1).split(',')]
                # the line prints drones numbered from 1 since 1 Oct; the slots index from 0
                ev['slot2drone'] = [v - 1 for v in ids] if 'numbered from 1' in line else ids
            elif 'measure_rod_len: rods' in line:
                ev['rods_applied'] = [float(v) for v in re.search(r'rods \[([0-9., ]+)\]', line).group(1).split(',')]
            elif 'measure_rod_len: keeping typed' in line:
                ev['rods_measured'] = [float(v) for v in re.search(r'measured \[([0-9., ]+)\]', line).group(1).split(',')]
            elif 'coupled planner active' in line:
                ev['t_handover'] = t
                m = re.search(r'elevation ([0-9.]+)deg reached', line)
                ev['handover_elev'] = float(m.group(1)) if m else 45.0
            elif 'LAND - descending' in line:
                ev['t_land'] = t
            elif 'load down' in line and 'survivors' in line:
                ev['t_down'] = t
            elif 'solve status' in line:
                ev['fail_times'].append(t)
    ev.setdefault('rods_measured', ev.get('rods_applied'))
    ev.setdefault('slot2drone', list(range(n)))
    return argparse.Namespace(rows=rows, drones=drones, prm=prm, n=n, ev=ev, dir=run_dir)


def row_slot2drone(r, fallback):
    """slot -> drone (0-based) of one planner log row ('3 0 2 1'), else the fallback."""
    v = r.get('slot2drone') if hasattr(r, 'get') else None
    if isinstance(v, str) and v.strip():
        return [int(x) for x in v.split()]
    if isinstance(v, (int, float)) and np.isfinite(v):
        return [int(v)]
    return fallback


def _interp(df, col, t):
    return np.interp(t, df['sim_time'].to_numpy(), df[col].to_numpy())


def drone_positions(fl, t, pivot_dz=0.0):
    """Physical drones' positions at time t (array n x 3). With pivot_dz, the point that far
    below the centre along each drone's body z (nearest logged attitude)."""
    out = []
    for df in fl.drones:
        p = np.array([_interp(df, c, t) for c in ('pose_x', 'pose_y', 'pose_z')])
        if pivot_dz:
            k = int(np.clip(np.searchsorted(df['sim_time'].to_numpy(), t), 0, len(df) - 1))
            w, x, y, z = (df[c].iloc[k] for c in ('pose_qw', 'pose_qx', 'pose_qy', 'pose_qz'))
            bz = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
            p = p - pivot_dz * bz / np.linalg.norm(bz)
        out.append(p)
    return np.array(out)


def _axis_angle(ax, ay, ang):
    k = np.array([ax, ay, 0.0])
    nk = np.linalg.norm(k)
    if nk < 1e-12 or abs(ang) < 1e-12:
        return np.eye(3)
    k /= nk
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _rot_to_quat(R):
    w = np.sqrt(max(1e-12, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                     (R[1, 0] - R[0, 1]) / (4 * w)])


def load_states(fl, mods, rho_fit, rods_fit, yaw_fixed=False):
    """Per planner row: the 13-vector [p, q(wxyz), v, w=0] the replay feeds the node,
    plus the rod-fit tilt (deg) for checking against the logged one."""
    from scipy.optimize import least_squares
    rows, ev = fl.rows, fl.ev
    t = rows['sim_time'].to_numpy()
    s2d = ev['slot2drone']
    p0 = rows[['load_x', 'load_y']].to_numpy()[0]
    D0 = drone_positions(fl, t[0])
    az0 = np.array([np.arctan2(D0[d][1] - p0[1], D0[d][0] - p0[0]) for d in s2d])
    pay = fl.drones[0]
    tp = pay['sim_time'].to_numpy()
    vxy = []
    for c in ('payload_x', 'payload_y'):
        sm = pd.Series(pay[c].to_numpy()).rolling(5, center=True, min_periods=1).mean().to_numpy()
        vxy.append(np.interp(t, tp, np.gradient(sm, tp)))
    psi0 = np.radians(ev['psi0_deg'])
    X, fit_tilt = [], []
    for k, tk in enumerate(t):
        r = rows.iloc[k]
        p = np.array([r.load_x, r.load_y, r.load_z])
        D = drone_positions(fl, tk)
        az = np.array([np.arctan2(D[d][1] - p[1], D[d][0] - p[0]) for d in s2d])
        yaw = psi0 if yaw_fixed else psi0 + float(np.mean((az - az0 + np.pi) % (2 * np.pi) - np.pi))
        Rz = mods.geometry.rot_z(yaw)

        def res(th):
            R = _axis_angle(th[0], th[1], np.hypot(th[0], th[1])) @ Rz
            return [np.linalg.norm(p + R @ rho_fit[i] - D[s2d[i]]) - rods_fit[i] for i in range(fl.n)]
        th = least_squares(res, [0.0, 0.0], bounds=([-0.6, -0.6], [0.6, 0.6])).x
        fit_tilt.append(np.degrees(np.hypot(*th)))
        R = _axis_angle(th[0], th[1], np.radians(float(r.tilt_deg))) @ Rz
        X.append(np.concatenate([p, _rot_to_quat(R), [vxy[0][k], vxy[1][k], r.load_vz], np.zeros(3)]))
    return np.array(X), np.array(fit_tilt)


# ── one arm ───────────────────────────────────────────────────────────────────

def rest_azimuth_offset(fl):
    """Mean angle (deg) by which the drones sat off their modelled attach azimuths on the
    first logged row (drones on the floor, rods lying out from the magnets)."""
    r = fl.rows.iloc[0]
    D = drone_positions(fl, r.sim_time)
    slot_az = [np.degrees(np.arctan2(v[1], v[0])) for v in _rho_flown(fl)]
    offs = [np.degrees(np.arctan2(D[d][1] - r.load_y, D[d][0] - r.load_x)) - fl.ev['psi0_deg'] - slot_az[i]
            for i, d in enumerate(fl.ev['slot2drone'])]
    return float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(offs))))))


def flown_path_radius(prm):
    """The radius the flight's path flew (load_trajectory.path_radius, kept here because --src may be an older
    tree): fig8_size for a figure-8 when set (22 Oct on), else traj_radius."""
    size = float(prm.get('fig8_size') or 0.0)
    return size if prm.get('load_traj') == 'fig_8' and size > 0.0 else float(prm.get('traj_radius', 0.5))


def _rho_flown(fl):
    az = fl.prm.get('attach_azimuths_deg', '') or ''
    th = ([float(v) for v in re.split(r'[,\s]+', az.strip()) if v] if az.strip().lower() not in ('', 'even', 'none', 'auto')
          else [360.0 * k / fl.n for k in range(fl.n)])
    rad = float(fl.prm['attach_radius'])
    return [np.array([rad * np.cos(np.radians(t)), rad * np.sin(np.radians(t)), float(fl.prm['attach_z'])]) for t in th]


def arm_setup(fl, arm):
    """Arm letters combine with '+', e.g. 'a+g'."""
    prm, ev = fl.prm, fl.ev
    ixx, izz = float(prm.get('load_ixx', 0.0) or 0.0), float(prm.get('load_izz', 0.0) or 0.0)
    geo = dict(radius=float(prm['attach_radius']), attach_z=float(prm['attach_z']),
               az_deg=[float(np.degrees(np.arctan2(v[1], v[0]))) for v in _rho_flown(fl)],
               rod=float(prm['cable_len']), drone=float(prm['drone_mass']), load=float(prm['load_mass']),
               rods=ev.get('rods_applied'), elev=45.0, pivot=0.0, stale=0.0, omega_sd=0.0,
               reset=False, qsign=False, yaw_fixed=False, iters=None, fast=False,
               inertia=[ixx, ixx, izz] if ixx > 0.0 else None)
    for t in [a for a in arm.split('+') if a != 'base']:
        if re.fullmatch(r'i\d+', t):
            geo['iters'] = int(t[1:])
        elif t == 'v':
            geo['fast'] = True
        elif t == 'a':
            geo['reset'] = True
        elif t == 'b':
            geo['elev'] = ev.get('handover_elev', 45.0)
        elif t == 'c':
            geo.update(pivot=PIVOT_DZ, rods=[max(RIG_ROD_055, float(prm['cable_len']))] * fl.n)
        elif t == 'd':
            geo['stale'] = 0.5
        elif t == 'e':
            geo['omega_sd'] = None          # filled from --omega-sd
        elif t == 'f':
            geo.update(radius=0.25, rod=0.53, drone=0.525, rods=None)
        elif t == 'g':
            geo['qsign'] = True
        elif t == 'h':
            off = rest_azimuth_offset(fl)
            geo['az_deg'] = [a + off for a in geo['az_deg']]
        elif t == 'y':
            geo['yaw_fixed'] = True
        else:
            raise ValueError(f'unknown arm {t!r}')
    return geo


def logged_attitude(rows, LS):
    """LS with the ring attitude and rate the planner logged (qw..qz, wx..wz: what it was fed) on every row that
    has them; the rest keep the rod fit."""
    cols = ['qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz']
    if not set(cols) <= set(rows.columns):
        return LS
    V = rows[cols].to_numpy(float)
    ok = np.all(np.isfinite(V), axis=1)
    out = np.array(LS, float)
    out[ok, 3:7] = V[ok, :4]
    out[ok, 10:13] = V[ok, 4:]
    return out


def ref_digest(mods, solver, X, n, fast):
    """sha1 over every drone's (pos, vel, thrust acc, cable acc, rod acc) at every node, extracted as the planner
    does with plan_fast_refs on (one mapped call) or off (node by node)."""
    h = hashlib.sha1()
    for i in range(n):
        nodes = (solver.horizon_kinematics(X, i) if fast else
                 [(*solver.drone_kinematics(X[:, k], i), mods.ps.rod_accel_at(solver, X[:, k], i))
                  for k in range(X.shape[1])])
        for node in nodes:
            for v in node:
                h.update(np.ascontiguousarray(v, float).tobytes())
    return h.hexdigest()


def run_arm(fl, mods, arm, work, omega_sd=0.3, seed=0, keep_nodes=False, quality=False, q_source='fit',
            flown_pivot=False, ref_hash=False):
    geo = arm_setup(fl, arm)
    if geo['omega_sd'] is None:
        geo['omega_sd'] = omega_sd
    po = fl.prm.get('pivot_offset') or [0.0, 0.0, 0.0]
    pivot_offset = [float(v) for v in (json.loads(po) if isinstance(po, str) else po)]   # params.json: '[0, 0, -0.04]'
    if flown_pivot:
        geo['pivot'] = -pivot_offset[2]
    n, ev, rows = fl.n, fl.ev, fl.rows
    rho = mods.geometry.attach_points(n, geo['radius'], geo['attach_z'], geo['az_deg'])
    # the attitude fit uses the flown radius in every arm (and h's azimuths)
    rho_fly = mods.geometry.attach_points(n, float(fl.prm['attach_radius']), float(fl.prm['attach_z']),
                                          geo['az_deg'])
    rods_fit = ev.get('rods_measured') or [float(fl.prm['cable_len'])] * n
    LS, fit_tilt = load_states(fl, mods, rho_fly, rods_fit, yaw_fixed=geo['yaw_fixed'])
    if q_source == 'auto':
        LS = logged_attitude(rows, LS)

    J = geo['inertia'] or mods.params.load_inertia(geo['load'])
    dyn = mods.lcd.LoadCableDynamics(n, geo['load'], J, [geo['rod']] * n, rho, geo['drone'])
    # acados also rebuilds when the nominal node-0 bounds change, and they follow the azimuths
    az_tag = '_'.join(f'{a:.0f}' for a in geo['az_deg'])
    i_tag = '' if geo['inertia'] is None else f'_I{J[0]:.4f}_{J[2]:.4f}'
    code = os.path.join(work, f"n{n}_m{geo['drone']:.3f}_M{geo['load']:.3f}_az{az_tag}{i_tag}")
    os.makedirs(code, exist_ok=True)
    os.environ['MDC_ACADOS_ROOT'] = code
    # the solver's cache check and signature live in its import-time CODE_DIR: here too, never the planner's cache
    mods.ps.CODE_DIR = os.path.join(code, 'c_generated_code_load_planner')
    cwd = os.getcwd()
    os.chdir(code)
    try:
        solver = mods.ps.PlannerSolver(dyn)
    finally:
        os.chdir(cwd)
    if geo['iters']:
        solver.steady_iters = geo['iters']
    solver.defer_res = geo['fast']
    if flown_pivot:
        solver.pivot_offset = np.asarray(pivot_offset, float)
    s_nom = mods.geometry.nominal_cable_dirs(rho, geo['elev'])
    laps = int(float(fl.prm.get('traj_laps') or 1))         # passed only when lapped: --src may predate traj_laps
    traj = mods.lt.LoadTrajectory(fl.prm.get('load_traj', 'hover'), float(fl.prm.get('traj_speed', 0.1)),
                                  float(fl.prm.get('traj_distance', 1.0)), flown_path_radius(fl.prm),
                                  **({'laps': laps} if laps > 1 else {}))
    refs = mods.rb.ReferenceBuilder(dyn, n, s_nom, solver.dt, traj)
    refs.set_yaw_datum(np.radians(ev['psi0_deg']))
    s2d = ev['slot2drone']
    t = rows['sim_time'].to_numpy()
    t_ho, t_land, t_down = ev['t_handover'], ev.get('t_land', np.inf), ev.get('t_down', np.inf)
    target_z = float(fl.prm['target_z'])
    hover_xy = (float(rows['load_x'].iloc[0]), float(rows['load_y'].iloc[0]))
    rng = np.random.default_rng(seed)
    acados = solver.solver
    ti = [LOAD_T(mods, i) for i in range(n)]
    si = [mods.lcd.LOAD_DIM + mods.lcd.CABLE_DIM * i + 2 for i in range(n)]
    recs, prev_prog, geom_set, q_prev = [], 0.0, False, None
    for k, tk in enumerate(t):
        r = rows.iloc[k]
        planner = str(r.phase) == 'planner'
        if planner and not geom_set:
            geom_set = True
            if geo['rods'] is not None:
                solver.set_geometry(cable_lengths=list(geo['rods']))
        ls = LS[k].copy()
        if geo['stale'] > 0.0:
            j = max(0, int(np.searchsorted(t, tk - geo['stale'], side='right')) - 1)
            ls = LS[j].copy()
        if geo['qsign'] and q_prev is not None and float(np.dot(ls[3:7], q_prev)) < 0.0:
            ls[3:7] = -ls[3:7]
        q_prev = ls[3:7].copy()
        if geo['omega_sd'] > 0.0:
            ls[10:13] = rng.normal(0.0, geo['omega_sd'], 3)
        D = drone_positions(fl, tk, geo['pivot'])
        s2d = row_slot2drone(r, s2d)
        slot_pos = [D[s2d[i]] for i in range(n)]
        if planner:
            prog = float(r.lift_progress)
            lift_vel = (prog - prev_prog) * 10.0
            prev_prog = prog
            if int(r.land) and float(r.load_z) <= float(r.z_tgt) - prog + 0.05:
                break                               # LAND hold: the node stops solving here
            zb = float(r.z_bias) if np.isfinite(r.z_bias) else 0.0
            # the logged trajectory clock: 0 here replayed every orbit as a hover (8 Oct)
            traj_t = float(r.traj_t) if 'traj_t' in r and np.isfinite(r.traj_t) else 0.0
            refs.update(hover_xy, float(r.z_tgt) - prog, prog, target_z, lift_vel, traj_t, z_bias=zb)
            yref_at = refs.yref_at
        else:
            hold = refs.hold_yref(ls)
            yref_at = (lambda _k, h=hold: h)
        reseed = solver.last_X is None or (planner and solver.recover)
        x_init = solver.build_x_init(ls, slot_pos)
        before = PQ.snapshot(solver) if quality and planner and not reseed else None
        t0 = time.perf_counter()
        if geo['reset'] and reseed:
            acados.reset()
            for j in range(solver.N):
                acados.set(j, 'u', np.zeros(dyn.nu))
        X, status = solver.solve_horizon(yref_at, refs.q_ref_at, x_init, reseed=reseed)
        dt_ms = (time.perf_counter() - t0) * 1e3
        if getattr(solver, 'finish_tick', None) is not None:
            solver.finish_tick()                            # a deferred residual (arm v), as the node does
        res = getattr(solver, 'last_res', None)
        rec = dict(t=float(tk), rel=float(tk - t_ho), planner=planner, status=int(status),
                   reseed=bool(reseed), ms=dt_ms, load_z=float(r.load_z), z_tgt=float(r.z_tgt),
                   land=int(r.land), fit_tilt=float(fit_tilt[k]), tilt=float(r.tilt_deg),
                   z_bias=float(r.z_bias) if np.isfinite(r.z_bias) else 0.0,
                   res_eq=None if res is None else float(res[1]), iters=int(getattr(solver, 'last_iters', 0)))
        if X is not None:
            rec['ratio'] = float(-sum(X[ti[i], 0] * X[si[i], 0] for i in range(n)) / (float(fl.prm['load_mass']) * G))
            rec['t0'] = [float(X[ti[i], 0]) for i in range(n)]
            rec['zN'] = float(X[2, -1])
            rec['plan5'] = [float(v) for v in X[0:3, min(5, X.shape[1] - 1)]]   # ring, node 5 (log plan5_*)
            if ref_hash:
                rec['x_sha'] = hashlib.sha1(np.ascontiguousarray(X, float).tobytes()).hexdigest()
                rec['ref_sha'] = ref_digest(mods, solver, X, n, geo['fast'])
            if before is not None:
                rec.update(PQ.compare(solver, before, X, lambda: solver.solve_horizon(
                    yref_at, refs.q_ref_at, x_init, reseed=False), ti))
            if keep_nodes:
                kin0 = [solver.drone_kinematics(X[:, 0], i) for i in range(n)]
                kin5 = [solver.drone_kinematics(X[:, 5], i) for i in range(n)]
                rec['ref0'] = {int(s2d[i]): [float(v) for v in kin0[i][0]] for i in range(n)}
                rec['ref5'] = {int(s2d[i]): [float(v) for v in kin5[i][0]] for i in range(n)}
                rec['acc0z'] = {int(s2d[i]): float(kin0[i][2][2]) for i in range(n)}
                rec['pos'] = {int(s2d[i]): [float(v) for v in D[s2d[i]]] for i in range(n)}
        recs.append(rec)
        pending = getattr(solver, 'pending_X', None)       # an unconverged solve is continued, as in the node
        if planner:
            if X is None:
                solver.recover, solver.last_X = pending is None, pending
            else:
                solver.recover, solver.last_X = False, X
        else:
            solver.last_X = X if X is not None else pending
    return dict(arm=arm, geo={k: v for k, v in geo.items()}, recs=recs,
                t_handover=t_ho, t_land=t_land, t_down=t_down, fail_times=ev['fail_times'])


def LOAD_T(mods, i):
    return mods.lcd.LOAD_DIM + mods.lcd.CABLE_DIM * i + 12


def static_hold(mods, elev_deg, work, ticks=50, n=4, load=0.86, drone=0.55, radius=0.225,
                attach_z=0.0, rod=0.55, z=0.5):
    """A level ring held still at z with every rod at elev_deg, fed to the planner `ticks`
    times against its usual 45 deg references. Returns (statuses, node-0 sum(t*s_z)/weight
    of the last good solve)."""
    rho = mods.geometry.attach_points(n, radius, attach_z)
    s = mods.geometry.nominal_cable_dirs(rho, elev_deg)
    dyn = mods.lcd.LoadCableDynamics(n, load, mods.params.load_inertia(load), [rod] * n, rho, drone)
    code = os.path.join(work, f'n{n}_m{drone:.3f}_M{load:.3f}_even')
    os.makedirs(code, exist_ok=True)
    os.environ['MDC_ACADOS_ROOT'] = code
    mods.ps.CODE_DIR = os.path.join(code, 'c_generated_code_load_planner')
    cwd = os.getcwd()
    os.chdir(code)
    try:
        solver = mods.ps.PlannerSolver(dyn)
    finally:
        os.chdir(cwd)
    refs = mods.rb.ReferenceBuilder(dyn, n, mods.geometry.nominal_cable_dirs(rho, 45.0), solver.dt,
                                    mods.lt.LoadTrajectory('hover', 0.1, 1.0, 0.5))
    refs.update((0.0, 0.0), z, 0.0, z, 0.0, 0.0)
    ls = np.array([0.0, 0.0, z, 1.0, 0.0, 0.0, 0.0] + [0.0] * 6)
    drones = [ls[0:3] + rho[i] - rod * s[i] for i in range(n)]
    statuses, ratio = [], None
    for _ in range(ticks):
        x_init = solver.build_x_init(ls, drones)
        X, st = solver.solve_horizon(refs.yref_at, refs.q_ref_at, x_init,
                                     reseed=solver.last_X is None or solver.recover)
        statuses.append(int(st))
        solver.recover, solver.last_X = X is None, X
        if X is not None:
            ratio = float(-sum(X[LOAD_T(mods, i), 0] * X[LOAD_T(mods, i) - 10, 0] for i in range(n)) / (load * G))
    return statuses, ratio


# ── summaries ─────────────────────────────────────────────────────────────────

def summarize(res):
    t_land = res['t_land']
    P = [r for r in res['recs'] if r['planner']]
    pre = [r for r in P if r['t'] < t_land]
    post = [r for r in P if r['t'] >= t_land]
    fails = [r for r in P if r['status'] != 0]
    logged = np.array(res['fail_times'])
    matched = sum(1 for r in fails if logged.size and np.min(np.abs(logged - r['t'])) < 0.15)
    z0 = min((r['z_tgt'] for r in P if np.isfinite(r['z_tgt'])), default=0.0)
    hold = [r['ratio'] for r in pre if 'ratio' in r and r['load_z'] > z0 + 0.05]
    allr = [r['ratio'] for r in pre if 'ratio' in r]
    return dict(
        arm=res['arm'], ticks=len(P), fail_pre=sum(r['status'] != 0 for r in pre),
        fail_post=sum(r['status'] != 0 for r in post),
        first_fail=(min(r['rel'] for r in fails) if fails else None), matched=matched,
        max_ms=max((r['ms'] for r in P), default=0.0),
        over_100ms=sum(r['ms'] > 100.0 for r in P),
        ratio_med=float(np.median(allr)) if allr else None,
        ratio_air_med=float(np.median(hold)) if hold else None,
        ratio_min=float(np.min(allr)) if allr else None, ratio_max=float(np.max(allr)) if allr else None)


def timing_vs_logged(rows, recs):
    """The flight's logged solve_ms against this replay's per-tick wall time (both PlannerSolver.solve_horizon, the
    residual inside unless deferred), over the warm planner ticks both solved: p50 / p95 of each, of their per-tick
    ratio (logged / replayed), and the same on the ticks where both ran the same SQP iterations (the flight's budget
    cut not firing). None when the log has no solve_ms or no tick ran the flight's iterations (an i<k> arm other than
    the flown count times a different problem): card 2026-10-23_scalability, the B calibration."""
    if 'solve_ms' not in rows:
        return None
    lt = rows['sim_time'].to_numpy(float)
    lms = pd.to_numeric(rows['solve_ms'], errors='coerce').to_numpy(float)
    lit = (pd.to_numeric(rows['sqp_calls'], errors='coerce').to_numpy(float) if 'sqp_calls' in rows
           else np.full(len(rows), np.nan))
    pairs = []
    for r in recs:
        if not r['planner'] or r['reseed'] or r['status'] != 0:
            continue
        k = int(np.searchsorted(lt, r['t']))
        if k < len(lt) and lt[k] == r['t'] and np.isfinite(lms[k]):
            pairs.append((lms[k], r['ms'], lit[k] == r.get('iters', -1)))
    P = np.array(pairs, float).reshape(-1, 3)
    if not np.any(P[:, 2] > 0.5):
        return None

    def stats(m):
        lg, rp = P[m, 0], P[m, 1]
        if not lg.size:
            return None
        pc = lambda a, q: float(np.percentile(a, q))
        return {'ticks': int(lg.size), 'logged_p50': pc(lg, 50), 'logged_p95': pc(lg, 95),
                'replay_p50': pc(rp, 50), 'replay_p95': pc(rp, 95),
                'ratio_p50': pc(lg / rp, 50), 'ratio_p95': pc(lg / rp, 95)}
    return {'all': stats(np.ones(len(P), bool)), 'same_iters': stats(P[:, 2] > 0.5)}


def timing_table(arms):
    """The arms' replayed solve times against the flight's logged ones (timing_vs_logged)."""
    out = ['| arm | ticks | logged solve_ms p50 / p95 | replayed p50 / p95 | per-tick logged/replayed p50 / p95 '
           '| same iterations: ticks, ratio p50 / p95 |', '|' + '---|' * 6]
    for arm, tm in arms.items():
        if not tm or not tm.get('all'):
            continue
        a, s = tm['all'], tm.get('same_iters') or {}
        same = (f"{s['ticks']}, {s['ratio_p50']:.2f} / {s['ratio_p95']:.2f}" if s else '')
        out.append(f"| {arm} | {a['ticks']} | {a['logged_p50']:.1f} / {a['logged_p95']:.1f} "
                   f"| {a['replay_p50']:.1f} / {a['replay_p95']:.1f} | {a['ratio_p50']:.2f} / {a['ratio_p95']:.2f} "
                   f"| {same} |")
    return '\n'.join(out)


def table(sums, logged_pre, logged_post):
    out = ['| arm | what | status!=0 hand-over..LAND | LAND..ring down | first (s after hand-over) '
           '| at a logged failure (+-0.15 s) | max solve ms | ticks > 100 ms | node-0 sum(t*s_z)/W median '
           '| airborne median | min..max |',
           '|' + '---|' * 11]
    out.append(f'| rig | as logged | {logged_pre} | {logged_post} | | | | | | | |')
    f = lambda v, fmt: '' if v is None else format(v, fmt)
    for s in sums:
        out.append(f"| {s['arm']} | {' + '.join(ARM_TEXT.get(t, t) for t in s['arm'].split('+'))} | {s['fail_pre']} | {s['fail_post']} "
                   f"| {f(s['first_fail'], '.1f')} | {s['matched']} | {s['max_ms']:.0f} | {s['over_100ms']} "
                   f"| {f(s['ratio_med'], '.3f')} | {f(s['ratio_air_med'], '.3f')} "
                   f"| {f(s['ratio_min'], '.2f')}..{f(s['ratio_max'], '.2f')} |")
    return '\n'.join(out)


def heave_report(fl, res):
    """Replayed node-0 drone references against what each tracker logged (ref_x/y/z is node 0
    of the last reference it received), and how the references move with the ring."""
    lines = []
    ok = [r for r in res['recs'] if r['planner'] and 'ref0' in r and r['t'] < res['t_land']]
    z0 = min(r['z_tgt'] for r in ok)
    # the hold after the lift ramp topped out, ring off the floor
    target = float(fl.prm['target_z'])
    air = [r for r in ok if r['load_z'] > z0 + 0.05 and r['z_tgt'] >= target - 1e-3]
    lines.append('| drone | ticks | replayed - logged node-0 ref, xyz median abs (cm) | z p95 abs (cm) '
                 '| logged ref_z - pose_z sd (cm) | replayed ref0_z - pose_z sd (cm) '
                 '| corr(replayed ref0_z - pose_z, ring z) | corr(node-5 ref z - pose_z, ring vz) |')
    lines.append('|' + '---|' * 8)
    tz = np.array([r['t'] for r in air])
    ring_z = np.array([r['load_z'] for r in air])
    ring_vz = np.interp(tz, fl.rows['sim_time'], fl.rows['load_vz'])
    for d in range(fl.n):
        df = fl.drones[d]
        tt = df['sim_time'].to_numpy()
        refs = df[['ref_x', 'ref_y', 'ref_z']].to_numpy()
        change = np.r_[True, np.any(np.abs(np.diff(refs, axis=0)) > 1e-9, axis=1)]
        tc, rc = tt[change], refs[change]
        err, lz, rz, r5 = [], [], [], []
        for r in air:
            j = np.searchsorted(tc, r['t'])
            if j < len(tc) and tc[j] - r['t'] < 0.3:
                err.append(np.array(r['ref0'][str(d)]) - rc[j])
            pz = r['pos'][str(d)][2]
            lz.append(_interp(df, 'ref_z', r['t'] + 0.15) - _interp(df, 'pose_z', r['t'] + 0.15))
            rz.append(r['ref0'][str(d)][2] - pz)
            r5.append(r['ref5'][str(d)][2] - pz)
        err = np.abs(np.array(err)) * 100
        c1 = np.corrcoef(rz, ring_z)[0, 1]
        c2 = np.corrcoef(r5, ring_vz)[0, 1]
        lines.append(f'| {d} | {len(err)} | {" / ".join(f"{v:.1f}" for v in np.median(err, axis=0))} '
                     f'| {np.percentile(err[:, 2], 95):.1f} | {np.std(lz) * 100:.1f} | {np.std(rz) * 100:.1f} '
                     f'| {c1:+.2f} | {c2:+.2f} |')
    zN = np.array([r['zN'] for r in air])
    tgt = np.array([r['z_tgt'] + r.get('z_bias', 0.0) for r in air])
    miss = ring_z - tgt
    corr_n = zN - ring_z
    slope = float(np.polyfit(miss, corr_n, 1)[0])
    az0 = np.array([np.mean(list(r['acc0z'].values())) for r in air])
    lines.append('')
    tu = np.arange(tz[0], tz[-1], 0.1)
    zu = np.interp(tu, tz, ring_z) - np.mean(ring_z)
    spec = np.abs(np.fft.rfft(zu * np.hanning(len(zu))))
    fpk = np.fft.rfftfreq(len(zu), 0.1)[1 + int(np.argmax(spec[1:]))]
    lines.append(f'Hold ticks {len(air)} over {tz[-1] - tz[0]:.1f} s; ring z sd {np.std(ring_z) * 100:.1f} cm, '
                 f'p-p {np.ptp(ring_z) * 100:.1f} cm, spectral peak {fpk:.2f} Hz; height target (z_tgt + z_bias) '
                 f'sd {np.std(tgt) * 100:.1f} cm.')
    lines.append(f'Planned ring z at the horizon end minus the ring now, against the miss (ring - target): '
                 f'corr {np.corrcoef(miss, corr_n)[0, 1]:+.2f}, slope {slope:+.2f} (-1 = plans to remove the whole miss in 2 s).')
    lines.append(f'Node-0 thrust feedforward a_z (mean of the drones): sd {np.std(az0):.2f} m/s^2, corr with ring z '
                 f'{np.corrcoef(az0, ring_z)[0, 1]:+.2f}, with ring vz {np.corrcoef(az0, ring_vz)[0, 1]:+.2f}.')
    ft = np.array([r['fit_tilt'] for r in ok]); lt = np.array([r['tilt'] for r in ok])
    lines.append(f'Rod-fit tilt vs logged tilt over the planner ticks: corr {np.corrcoef(ft, lt)[0, 1]:+.2f}, '
                 f'median |diff| {np.median(np.abs(ft - lt)):.1f} deg.')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--flight', default='model_f1', help='model_f1, f7 or a <run>_logs directory')
    ap.add_argument('--arms', default=','.join(ARMS))
    ap.add_argument('--src', default=DEFAULT_SRC, help='mpc_planner package directory to import')
    ap.add_argument('--work', default=DEFAULT_WORK, help='scratch dir for the generated solver')
    ap.add_argument('--omega-sd', type=float, default=0.3, help='arm e: ring angular-rate noise, rad/s')
    ap.add_argument('--jobs', type=int, default=1, help='arms in parallel (inflates solve times)')
    ap.add_argument('--out', default=None, help='write per-arm JSON (every tick) here')
    ap.add_argument('--heave', action='store_true', help='compare replayed node-0 refs with the logged ones')
    ap.add_argument('--static-hold', default=None, metavar='DEG,DEG',
                    help='static-hold check at these rod elevations; prints one JSON line each')
    ap.add_argument('--quality', action='store_true',
                    help="each arm's warm solves against the converged plan of the same tick (tools/plan_quality.py)")
    ap.add_argument('--q-source', choices=('fit', 'auto'), default='fit',
                    help="the ring attitude fed: the rod fit (as before) or the planner log's qw..qz / wx..wz")
    ap.add_argument('--flown-pivot', action='store_true',
                    help="rods from the flight's pivot_offset, references at the drone centres")
    ap.add_argument('--ref-hash', action='store_true', help="hash every tick's plan and extracted references")
    ap.add_argument('--summary-json', default=None, help='write every flight\'s arm summaries here')
    ap.add_argument('--one', default=None, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if not a.one:
        # one build tree per source tree: the solver cache is judged on mtimes alone
        a.work = os.path.join(a.work, 'src_' + hashlib.md5(os.path.abspath(a.src).encode()).hexdigest()[:8])
    os.makedirs(a.work, exist_ok=True)
    if a.static_hold:
        mods = load_modules(a.src)
        for e in [float(v) for v in a.static_hold.split(',')]:
            st, ratio = static_hold(mods, e, a.work)
            print(json.dumps({'elev_deg': e, 'status': st, 'ratio': ratio}))
        return 0
    if a.one:
        mods = load_modules(a.src)
        fl = read_flight(FLIGHTS.get(a.flight, a.flight))
        res = run_arm(fl, mods, a.one, a.work, a.omega_sd, keep_nodes=a.heave, quality=a.quality,
                      q_source=a.q_source, flown_pivot=a.flown_pivot, ref_hash=a.ref_hash)
        with open(a.out, 'w') as f:
            json.dump(res, f)
        return 0
    arms = [s.strip() for s in a.arms.split(',') if s.strip()]
    summary, bad = {}, 0
    for flight in [s.strip() for s in a.flight.split(',') if s.strip()]:
        results = replay_flight(FLIGHTS.get(flight, flight), arms, a)
        if not results:
            bad += 1
            continue
        summary[flight] = {k: dict(summarize(v), timing=v.get('timing'), quality=(PQ.summarize(
            [r for r in v['recs'] if r['planner'] and 'q_conv_status' in r]) if a.quality else None))
            for k, v in results.items()}
        if a.quality:
            print('\n' + quality_table(summary[flight]))
    if a.summary_json:
        os.makedirs(os.path.dirname(os.path.abspath(a.summary_json)), exist_ok=True)
        with open(a.summary_json, 'w') as f:
            json.dump({'arms': arms, 'quality': a.quality, 'q_source': a.q_source, 'flown_pivot': a.flown_pivot,
                       'src': os.path.abspath(a.src), 'flights': summary}, f, indent=1)
        print(f'wrote {a.summary_json}')
    return 1 if bad else 0


def replay_flight(run_dir, arms, a):
    """Every arm on one flight, each in its own process; prints the arms' table. {arm: result} (empty: none ran)."""
    out = (os.path.join(a.out, os.path.basename(run_dir.rstrip('/'))) if a.out and ',' in a.flight
           else a.out or os.path.join(a.work, f'replay_{os.path.basename(run_dir.rstrip("/"))}'))
    os.makedirs(out, exist_ok=True)
    # the first arm of each solver build runs alone, so no arm loads a half-built solver
    fl = read_flight(run_dir)
    built, procs, results = set(), [], {}
    for arm in arms:
        g = arm_setup(fl, arm)
        key = (g['drone'], g['load'], tuple(round(v) for v in g['az_deg']), tuple(g['inertia'] or ()))
        cmd = [sys.executable, os.path.abspath(__file__), '--flight', run_dir, '--src', a.src,
               '--work', a.work, '--omega-sd', str(a.omega_sd), '--one', arm, '--q-source', a.q_source,
               '--out', os.path.join(out, f'{arm}.json')] + (['--heave'] if a.heave else []) + (
               ['--quality'] if a.quality else []) + (['--flown-pivot'] if a.flown_pivot else []) + (
               ['--ref-hash'] if a.ref_hash else [])
        while len([p for _, p in procs if p.poll() is None]) >= (max(1, a.jobs) if key in built else 1):
            time.sleep(0.2)
        procs.append((arm, subprocess.Popen(cmd)))
        if key not in built:
            procs[-1][1].wait()
            built.add(key)
    for arm, p in procs:
        if p.wait() != 0:
            print(f'arm {arm} failed (exit {p.returncode})', file=sys.stderr)
            continue
        with open(os.path.join(out, f'{arm}.json')) as f:
            results[arm] = json.load(f)
    if not results:
        return results
    any_res = next(iter(results.values()))
    logged = np.array(any_res['fail_times'])
    lp = int(np.sum((logged >= any_res['t_handover']) & (logged < any_res['t_land'])))
    lq = int(np.sum(logged >= any_res['t_land']))
    print(f'\n{os.path.basename(run_dir)}  hand-over {any_res["t_handover"]:.2f}  '
          f'LAND +{any_res["t_land"] - any_res["t_handover"]:.1f} s  per-tick JSON in {out}\n')
    print(table([summarize(results[k]) for k in arms if k in results], lp, lq))
    for k in results:
        results[k]['timing'] = timing_vs_logged(fl.rows, results[k]['recs'])
    if any(v['timing'] for v in results.values()):
        print('\n' + timing_table({k: results[k]['timing'] for k in arms if k in results}))
    if a.heave and 'base' in results:
        print('\n' + heave_report(read_flight(run_dir), results['base']))
    return results


def quality_table(arms):
    """The arms' quality against the converged plan (p50 / p95; tools/plan_quality.py)."""
    f = lambda q, k, s, sc=1.0, fmt='.3f': '' if not q or not q.get(k) else format(q[k][s] * sc, fmt)
    out = ['| arm | ticks | J/J_conv p50 / p95 | paired with res_eq <= 1e-3: p50 / p95 (ticks over) | res_eq p95 '
           '| ref node 1 p95 (mm) | node 5 p95 (mm) | node N p95 (mm) | ff0 p95 (m/s^2) | t0 p95 (N) '
           '| thrust excursion max (N) | failed solves |', '|' + '---|' * 12]
    for arm, s in arms.items():
        q = s.get('quality') or {}
        cp = q.get('cost_ratio_paired') or {}
        paired = f"{cp['p50']:.3f} / {cp['p95']:.3f} ({cp['res_over']})" if cp else ''
        out.append(f"| {arm} | {q.get('ticks', '')} | {f(q, 'cost_ratio', 'p50')} / {f(q, 'cost_ratio', 'p95')} "
                   f"| {paired} | {f(q, 'res_eq', 'p95', fmt='.1e')} | {f(q, 'ref1', 'p95', 1e3, '.1f')} "
                   f"| {f(q, 'ref5', 'p95', 1e3, '.1f')} | {f(q, 'refN', 'p95', 1e3, '.1f')} | {f(q, 'ff0', 'p95')} "
                   f"| {f(q, 't0', 'p95')} | {f(q, 'thrust_viol', 'max')} | {s['fail_pre'] + s['fail_post']} |")
    return '\n'.join(out)


if __name__ == '__main__':
    sys.exit(main())
