#!/usr/bin/env python3
"""tools/latency_bench.py -- our planner's closed loop with latency, offline, no Gazebo (corner-cutting plan of
16 Oct, step 2: rank fixes before the twin, and say whether the cut is delay or model mismatch).

    python3 tools/latency_bench.py --kind orbit --speed 0.3                    # today's base, plant p1
    python3 tools/latency_bench.py --kind fig_8 --rts --ref-lag 0.05 --iters 2 # today's timing combination
    python3 tools/latency_bench.py --validate                                  # gate G1 (see VALIDATE)

The planner is the real one: PlannerSolver, ReferenceBuilder and LoadTrajectory ticked at 10 Hz as LoadPlanner ticks
them (traj_t advanced one tick before the solve; the reference read at traj_t + k dt - plan_ref_lag_s; the desired
published at the tick, before the advance). Each plan starts from the plant measured `--pose-age` before its tick and
reaches the trackers `--sigma` after it. The solver is generated and built in --work (generated/latency_bench, never
the planner's own cache).

The trackers fly the latest plan along its own timeline from its arrival: the reference at time t is the plan at
plan time t - arrival (--rts, ref_time_shift), or t - arrival - --hold-extra under today's held horizon: the hold's
mean extra age as a smooth delay (the 10 Hz staircase itself is not reproduced; a stiff follower of the clamped
staircase slams the drones back to node 0 and diverges). A second-order follower (--track-w, critically damped, with
the plan's velocity fed forward) stands for the tracker and the attitude loop. --hold-extra and --track-w are fitted
so the plan's age as flown (D, as tools/lag_metrics.py measures it) matches the twin's (0.083-0.088 s held,
0.022-0.025 s shifted). (Following node 0 alone does not work: node 0 is pinned to the measured state, so it carries
no intent; a double-integrator MPC on the horizon pulled the drones back to node 0 and lagged 0.4 s.)

The plant: the ring as a rigid body on stiff spring-damper rods (the SIL plant's rod law, tools/sil/plant.py) hung from
the pivots.
  p0  the planner's own assumptions: massless rods, the planner's ring inertia.
  p1  the twin (simulation_assets/three_rigid_ground_rig.sdf): each rod's 0.075 kg lumped two-point (0.055 kg at the
      ring end, CoM 0.147 m from it), the world's ring inertia (2.227e-2, 4.431e-2), ball-joint damping 0.1 N m s/rad
      between the ring and each rod. Not modelled: the rods' own rotational inertia, the drone-end joint, the pivot
      4 cm below a tilting drone, contact.
Scored with metrics.contour_lag on a 50 Hz log with the desired held between ticks, exactly as a twin run.

STATUS (16 Oct, ~18:30): G1 FAILED; use it for code paths and solve time only, never to rank fixes. With no delay
and a stiff tracker it matches the planner's model (the ring 0.092 s ahead, 4.9 mm outside: the 0.1 s clock lead),
but its catch-up is about six times weaker than the twin's: on the orbit at 0.3 m/s the lag moves 15x the delivery
delay (sigma 0 / 0.010 / 0.023 s -> lag -0.092 / +0.060 / +0.240 s, across -4.9 / +25 / +68 mm), the twin about 2x
(sigma 0.023: -0.042 s, -4.9 mm, R1990). Likely cause, not proven: the drones here are kinematic, while the twin's
trackers fly the planned thrust and rod pull as feed-forward.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402

DEFAULT_SRC = os.path.join(REPO, 'src', 'mpc_planner')
DEFAULT_WORK = os.path.join(REPO, 'generated', 'latency_bench')    # git-ignored; never the planner's own cache
G = 9.81
TWIN_RING_I = (2.227e-2, 2.227e-2, 4.431e-2)
ROD_MASS, ROD_COM_FROM_RING, JOINT_DAMP = 0.075, 0.147, 0.1
ROD_K, ROD_C = 10000.0, 40.0
DT_SIM, LOG_DT, TICK = 0.0005, 0.02, 0.1


def load_modules(src):
    sys.path.insert(0, os.path.abspath(src))
    for k in [k for k in sys.modules if k.startswith('mpc_planner')]:
        del sys.modules[k]
    import mpc_planner.geometry as geometry
    import mpc_planner.load_cable_dynamics as lcd
    import mpc_planner.load_trajectory as lt
    import mpc_planner.params as params
    import mpc_planner.planner_ocp as ocp
    import mpc_planner.planner_solver as ps
    import mpc_planner.reference_builder as rb
    return argparse.Namespace(geometry=geometry, lcd=lcd, lt=lt, params=params, ocp=ocp, ps=ps, rb=rb)


def build_solver(mods, n, work, load=0.86, drone=0.55, rod=0.55, radius=0.225, attach_z=0.02):
    rho = mods.geometry.attach_points(n, radius, attach_z)
    dyn = mods.lcd.LoadCableDynamics(n, load, mods.params.load_inertia(load), [rod] * n, rho, drone)
    code = os.path.join(work, f'n{n}_m{drone:.3f}_M{load:.3f}')
    os.makedirs(code, exist_ok=True)
    os.environ['MDC_ACADOS_ROOT'] = code
    cwd = os.getcwd()
    os.chdir(code)
    try:
        solver = mods.ps.PlannerSolver(dyn)
    finally:
        os.chdir(cwd)
    return dyn, solver, rho


def _skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def _rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(max(1.0 + R[i, i] - R[j, j] - R[k, k], 1e-12))
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


class RingPlant:
    """The ring (rigid body) on n stiff spring-damper rods from prescribed pivots."""

    def __init__(self, rho, rod_len, plant='p1', load=0.86, planner_inertia=None):
        self.rho = [np.asarray(r, float) for r in rho]
        self.L = float(rod_len)
        m, I = load, np.diag(planner_inertia if plant == 'p0' else TWIN_RING_I)
        self.joint_c = 0.0
        if plant == 'p1':
            me = ROD_MASS * (1.0 - ROD_COM_FROM_RING / rod_len)
            for r in self.rho:
                I = I + me * (float(r @ r) * np.eye(3) - np.outer(r, r))
            m = m + me * len(self.rho)
            self.joint_c = JOINT_DAMP
        self.m, self.I = m, I
        self.Iinv = np.linalg.inv(I)
        self.x, self.v, self.R, self.w = np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3)   # w: body frame

    def step(self, dt, piv, piv_v):
        f = np.array([0.0, 0.0, -self.m * G])
        tau = np.zeros(3)
        w_world = self.R @ self.w
        for i, r in enumerate(self.rho):
            rw = self.R @ r
            d = piv[i] - (self.x + rw)
            dist = float(np.linalg.norm(d))
            n = d / dist
            v_att = self.v + np.cross(w_world, rw)
            vrel = piv_v[i] - v_att
            fr = (ROD_K * (dist - self.L) + ROD_C * float(vrel @ n)) * n
            f += fr
            tau += np.cross(rw, fr)
            if self.joint_c:
                w_rod = np.cross(n, vrel) / dist
                tau += -self.joint_c * (w_world - w_rod)
        self.v += f / self.m * dt
        self.x += self.v * dt
        tb = self.R.T @ tau
        self.w += self.Iinv @ (tb - np.cross(self.w, self.I @ self.w)) * dt
        self.R = self.R @ (np.eye(3) + _skew(self.w) * dt)
        u, _, vt = np.linalg.svd(self.R)
        self.R = u @ vt

    def state(self):
        """Mocap order: p, q (wxyz), v, w (body)."""
        return np.concatenate([self.x, _rot_to_quat(self.R), self.v, self.w])

    def tilt_deg(self):
        return math.degrees(math.acos(min(1.0, max(-1.0, self.R[2, 2]))))


def plan_nodes(solver, X, n, dt):
    """Per drone: pivot positions, velocities and accelerations at the nodes (acceleration by finite difference of
    the planned velocity, the last node held)."""
    g = solver._geom
    out = []
    for i in range(n):
        P = np.array([np.array(solver.pos_fun[i](X[:, k], g)).flatten() for k in range(X.shape[1])])
        V = np.array([np.array(solver.vel_fun[i](X[:, k], g)).flatten() for k in range(X.shape[1])])
        A = np.vstack([np.diff(V, axis=0) / dt, np.zeros((1, 3))])
        out.append((P, V, A))
    return out


def densify(plan, dt):
    """One drone's plan on the simulation step grid: (r, v, a) arrays indexed by int(plan time / DT_SIM)."""
    P, V, A = plan
    t = np.arange(P.shape[0]) * dt
    e = np.arange(0.0, t[-1] + 1e-9, DT_SIM)
    seg = np.minimum((e / dt).astype(int), A.shape[0] - 1)
    return (np.column_stack([np.interp(e, t, P[:, j]) for j in range(3)]),
            np.column_stack([np.interp(e, t, V[:, j]) for j in range(3)]), A[seg])


def flown(dense, e):
    """(r, v, a) of one drone's densified plan at plan time e (0 = node 0; before it, node 0 run back along its
    velocity, so a plan flown late is a smooth delay, not a jump back to where the drone was measured)."""
    if e < 0.0:
        return dense[0][0] + dense[1][0] * e, dense[1][0], dense[2][0]
    k = min(int(round(e / DT_SIM)), dense[0].shape[0] - 1)
    return dense[0][k], dense[1][k], dense[2][k]


def run(args, mods=None, built=None):
    mods = mods or load_modules(args.src)
    n = 3
    dyn, solver, rho = built or build_solver(mods, n, args.work)
    solver.steady_iters = int(args.iters)
    solver.last_X, solver.recover = None, False
    traj = mods.lt.LoadTrajectory(args.kind, args.speed, 1.0, args.radius)
    s_nom = mods.geometry.nominal_cable_dirs(rho, args.elev)
    refs = mods.rb.ReferenceBuilder(dyn, n, s_nom, solver.dt, traj)
    refs.ref_lag_s = float(args.ref_lag)
    z0 = 1.0
    plant = RingPlant(rho, dyn.l[0] if hasattr(dyn, 'l') else 0.55, args.plant, planner_inertia=mods.params.load_inertia(0.86))
    plant.x = np.array([0.0, 0.0, z0])
    x_nom = mods.ocp.nominal_hover_state(dyn, load_pos=(0.0, 0.0, z0), elevation_deg=args.elev)
    piv = np.array([plant.x + rho[i] - 0.55 * x_nom[mods.lcd.LOAD_DIM + mods.lcd.CABLE_DIM * i:
                                                     mods.lcd.LOAD_DIM + mods.lcd.CABLE_DIM * i + 3] for i in range(n)])
    piv_v = np.zeros((n, 3))
    hist, hist_t = [], []                          # measured states for the pose age
    plans, active = [], None                       # (t_arrival, nodes), the plan the trackers fly
    log = {k: [] for k in ('t', 'ring_x', 'ring_y', 'ring_z', 'des_x', 'des_y', 'tilt', 'ref', 'piv')}
    traj_t, des = 0.0, np.array([0.0, 0.0])
    hover_s, t_end_path = args.hover_s, None
    T_path = traj._fig8_theta(0.0)[3] if args.kind == 'fig_8' else (
        traj._circle_theta(0.0)[3] if args.kind in ('circle', 'spin') else args.orbit_s)
    t_stop = hover_s + T_path + 2.0
    fails, solve_ms, ticks, t, next_tick, next_log = 0, [], 0, 0.0, 0.0, 0.0
    rng = np.random.default_rng(args.seed)
    while t < t_stop:
        if t >= next_tick - 1e-9:
            # the planner tick: publish the desired (before the advance), advance the clock, solve from the
            # state measured pose_age ago, deliver sigma later
            des = np.array(traj.offset_at(traj_t)[:2])
            if t >= hover_s and not traj.complete(traj_t):
                traj_t += TICK
            j = max(0, len(hist_t) - 1 - int(round(args.pose_age / DT_SIM)))
            ls, piv_m = hist[j] if hist else (plant.state(), piv.copy())
            refs.update((0.0, 0.0), z0, 0.0, z0, 0.0, traj_t)
            x_init = solver.build_x_init(ls, list(piv_m))
            t0 = time.perf_counter()
            X, status = solver.solve_horizon(refs.yref_at, refs.q_ref_at, x_init,
                                             reseed=solver.last_X is None or solver.recover)
            solve_ms.append(1000.0 * (time.perf_counter() - t0))
            if X is None:
                fails += 1
                solver.last_X = solver.pending_X
                solver.recover = solver.last_X is None
            else:
                jit = rng.normal(0.0, args.sigma_sd) if args.sigma_sd else 0.0
                plans.append((t + max(0.0, args.sigma + jit),
                              [densify(p, solver.dt) for p in plan_nodes(solver, X, n, solver.dt)]))
            ticks += 1
            next_tick += TICK
        while plans and plans[0][0] <= t + 1e-12:
            active = plans.pop(0)
        if active is not None:
            e = (t - active[0]) - (0.0 if args.rts else args.hold_extra)
            w = args.track_w
            for i in range(n):
                r, vr, _ = flown(active[1][i], e)
                piv_v[i] += (w * w * (r - piv[i]) + 2.0 * w * (vr - piv_v[i])) * DT_SIM
                piv[i] += piv_v[i] * DT_SIM
        plant.step(DT_SIM, piv, piv_v)
        hist.append((plant.state(), piv.copy()))
        hist_t.append(t)
        if len(hist) > 2000:
            del hist[:1000]
            del hist_t[:1000]
        if t >= next_log - 1e-9:
            log['t'].append(t)
            log['ring_x'].append(plant.x[0])
            log['ring_y'].append(plant.x[1])
            log['ring_z'].append(plant.x[2])
            log['des_x'].append(des[0])
            log['des_y'].append(des[1])
            log['tilt'].append(plant.tilt_deg())
            log['ref'].append(flown(active[1][0], (t - active[0]) - (0.0 if args.rts else args.hold_extra))[0]
                              if active is not None else piv[0].copy())
            log['piv'].append(piv[0].copy())
            next_log += LOG_DT
        t += DT_SIM
    if getattr(args, 'return_log', False):
        return score(log, fails, solve_ms, ticks), log
    return score(log, fails, solve_ms, ticks)


def plan_age_D(log):
    """lag_metrics.tracker_delay's D on drone 0: the shift that best matches pivot(t - D) to the flown ref(t)."""
    t = np.asarray(log['t'])
    p, r = np.asarray(log['piv']), np.asarray(log['ref'])
    best = None
    for D in np.arange(-0.1, 0.4, 0.005):
        q = np.column_stack([np.interp(t - D, t, p[:, j], left=np.nan) for j in range(2)])
        e = np.nanmean(np.linalg.norm(q - r[:, :2], axis=1))
        if best is None or e < best[0]:
            best = (e, D)
    return float(best[1])


def score(log, fails, solve_ms, ticks):
    t = np.asarray(log['t'])
    ring = np.column_stack([log['ring_x'], log['ring_y']])
    des = np.column_stack([log['des_x'], log['des_y']])
    sw = M.sweep_window(t, des)
    o = M.contour_lag(t, ring, des, sw)
    out = {(k[:-2] + '_mm' if k.endswith('_m') else k): (1000.0 * v if k.endswith('_m') else v) for k, v in o.items()}
    out['tilt_max_deg'] = float(np.max(np.asarray(log['tilt'])[sw])) if np.any(sw) else math.nan
    out['D_s'] = plan_age_D(log)
    out['fails'] = fails
    out['solve_p50_ms'] = float(np.median(solve_ms)) if solve_ms else math.nan
    out['ticks'] = ticks
    return out


# G1: the bench must reproduce today's twin before its numbers rank anything (plan of 16 Oct). Twin ranges from
# docs/experiments/2026-10-16_corner_cutting.md (correction section; corner_metrics on R1859-R1991).
VALIDATE = [
    ('base fig_8', dict(kind='fig_8', rts=False, sigma=0.023, ref_lag=0.0, iters=5), {'cut_topq_mean_mm': (14, 39)}),
    ('base orbit', dict(kind='orbit', rts=False, sigma=0.023, ref_lag=0.0, iters=5), {'across_mean_mm': (9, 19)}),
    ('base circle', dict(kind='circle', rts=False, sigma=0.023, ref_lag=0.0, iters=5), {'across_mean_mm': (6, 16)}),
    ('combo circle', dict(kind='circle', rts=True, sigma=0.023, ref_lag=0.05, iters=2),
     {'across_mean_mm': (-8, -1), 'lag_s': (-0.09, -0.03)}),
    ('combo orbit', dict(kind='orbit', rts=True, sigma=0.023, ref_lag=0.05, iters=2),
     {'across_mean_mm': (-8, -1), 'lag_s': (-0.07, -0.02)}),
    ('combo fig_8', dict(kind='fig_8', rts=True, sigma=0.023, ref_lag=0.05, iters=2), {'cut_topq_mean_mm': (-4, 9)}),
]


def parser():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--kind', default='orbit', choices=['orbit', 'circle', 'fig_8'])
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--radius', type=float, default=0.5)
    ap.add_argument('--plant', default='p1', choices=['p0', 'p1'])
    ap.add_argument('--rts', action='store_true', help='trackers fly the plan advanced by its age (ref_time_shift)')
    ap.add_argument('--sigma', type=float, default=0.023, help='tick to arrival at the trackers (s)')
    ap.add_argument('--sigma-sd', type=float, default=0.0)
    ap.add_argument('--pose-age', type=float, default=0.0)
    ap.add_argument('--ref-lag', type=float, default=0.0, help='plan_ref_lag_s')
    ap.add_argument('--iters', type=int, default=5, help='plan_sqp_iters')
    ap.add_argument('--elev', type=float, default=60.0)
    ap.add_argument('--track-w', type=float, default=200.0, help='tracker follower bandwidth (rad/s)')
    ap.add_argument('--hold-extra', type=float, default=0.06, help='held horizon: the extra age the drones fly at (s)')
    ap.add_argument('--hover-s', type=float, default=3.0)
    ap.add_argument('--orbit-s', type=float, default=24.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--src', default=DEFAULT_SRC)
    ap.add_argument('--work', default=DEFAULT_WORK)
    ap.add_argument('--validate', action='store_true')
    ap.add_argument('--json', action='store_true')
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    mods = load_modules(args.src)
    built = build_solver(mods, 3, args.work)
    if not args.validate:
        out = run(args, mods, built)
        print(json.dumps(out, indent=1) if args.json else
              '  '.join(f'{k} {v:.3f}' if isinstance(v, float) else f'{k} {v}' for k, v in out.items()))
        return 0
    misses = 0
    for name, over, bars in VALIDATE:
        a = parser().parse_args([])
        for k, v in vars(args).items():
            if k not in over and k not in ('validate', 'json'):
                setattr(a, k, v)
        for k, v in over.items():
            setattr(a, k, v)
        out = run(a, mods, built)
        ok = all(lo <= out.get(k, math.nan) <= hi for k, (lo, hi) in bars.items())
        misses += 0 if ok else 1
        print(f"{name:13s} {'ok  ' if ok else 'MISS'} " + '  '.join(
            f'{k} {out.get(k, math.nan):+.3f}' for k in ('cut_topq_mean_mm', 'across_mean_mm', 'across_rms_mm', 'lag_s',
                                                           'D_s', 'tilt_max_deg', 'solve_p50_ms', 'fails'))
              + f"   bars {bars}")
    print(f'G1: {len(VALIDATE) - misses}/{len(VALIDATE)} cells inside the twin ranges'
          + ('' if misses < 2 else ' -- FAILS the gate (two or more misses): rank nothing on this bench'))
    return 0 if misses < 2 else 1


if __name__ == '__main__':
    sys.exit(main())
