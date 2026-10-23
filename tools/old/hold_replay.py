#!/usr/bin/env python3
"""tools/hold_replay.py -- the load planner offline at one logged hold tick of a Gazebo or SIL
run, and the same planner closed on its own model (overnight 2026-10-04: where does the twin's
+5 cm hold with z_ki 0 come from?).

    python3 tools/hold_replay.py R0868              # horizon at a hold tick + model-plant loop
    python3 tools/hold_replay.py R0868 --ticks 150

1. Horizon: the measured state of one hold tick (ring pose and rate, drone pivots) solved to
   convergence; prints the planned ring height, tensions and drone heights along the horizon.
   If the plan ends at the target while the ring sits off it, the planner plans the move and
   the trackers + plant do not execute it; if it ends at the measured ring, the planner's own
   optimum is off the target.
2. Model plant: each tick the next measured state is the plan's node 1 (the OCP's own model,
   perfect trackers). Where it settles is the planner's own static bias.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # tools/
from planner_replay import load_modules, DEFAULT_SRC, DEFAULT_WORK  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # this file is in tools/old/


def quat_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def setup(mods, prm, work, pin=False):
    n = int(prm['num_drones'])
    az = prm.get('attach_azimuths_deg') or None
    az = [float(v) for v in az.split(',')] if isinstance(az, str) and az else None
    rho = mods.geometry.attach_points(n, float(prm['attach_radius']), float(prm['attach_z']), az)
    m_r = float(prm.get('rod_mass') or 0.0)
    lam = float(prm.get('rod_com') or 0.0) / float(prm['cable_len']) if m_r > 0.0 else 0.0
    load = float(prm['load_mass'])
    dyn = mods.lcd.LoadCableDynamics(n, load, mods.params.load_inertia(load), [float(prm['cable_len'])] * n,
                                     rho, float(prm['drone_mass']), rod_mass=m_r, rod_lam=lam)
    code = os.path.join(work, f"hold_n{n}_m{float(prm['drone_mass']):.3f}_M{load:.3f}_r{m_r:.3f}{'_pr' if pin else ''}")
    os.makedirs(code, exist_ok=True)
    os.environ['MDC_ACADOS_ROOT'] = code
    # the solver's cache check and signature live in its import-time CODE_DIR: here too, never the planner's cache
    mods.ps.CODE_DIR = os.path.join(code, 'c_generated_code_load_planner')
    cwd = os.getcwd()
    os.chdir(code)
    try:
        solver = mods.ps.PlannerSolver(dyn, pin_rates=pin)
    finally:
        os.chdir(cwd)
    solver.pivot_offset = np.array(json.loads(prm.get('pivot_offset', '[0, 0, 0]')), float)
    s_nom = mods.geometry.nominal_cable_dirs(rho, float(prm.get('handover_elev_deg', 45.0)))
    refs = mods.rb.ReferenceBuilder(dyn, n, s_nom, solver.dt, mods.lt.LoadTrajectory('hover', 0.1, 1.0, 0.5))
    return n, dyn, solver, refs


def hold_tick(run_dir, back_s):
    L = pd.read_csv(glob.glob(os.path.join(run_dir, 'logs', '*planner*', '*', 'log.csv'))[0])
    air = L[L.land.astype(str).isin(['0', 'False', 'false']) & (L.lift_progress >= L.lift_progress.max() - 1e-6)]
    r = air[air.sim_time <= air.sim_time.max() - back_s].iloc[-1]
    return r, L


def measured(r, n, piv):
    ls = np.array([r.load_x, r.load_y, r.load_z, r.qw, r.qx, r.qy, r.qz, 0.0, 0.0, r.load_vz,
                   r.wx, r.wy, r.wz], float)
    pivots = []
    for i in range(n):
        q = [r[f'd{i}_qw'], r[f'd{i}_qx'], r[f'd{i}_qy'], r[f'd{i}_qz']]
        c = np.array([r[f'd{i}_x'], r[f'd{i}_y'], r[f'd{i}_z']], float)
        pivots.append(c + quat_rot(q) @ piv)
    return ls, pivots


def solve(solver, refs, ls, pivots, iters, vel=None):
    X = None
    for _ in range(iters):
        x_init = (solver.build_x_init(ls, pivots, vel) if getattr(solver, 'pin_rates', False)
                  else solver.build_x_init(ls, pivots))
        X, st = solver.solve_horizon(refs.yref_at, refs.q_ref_at, x_init, reseed=solver.last_X is None)
        solver.last_X = X if X is not None else None
    return X


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--back', type=float, default=5.0, help='hold tick this many s before LAND')
    ap.add_argument('--ticks', type=int, default=120)
    ap.add_argument('--src', default=DEFAULT_SRC)
    ap.add_argument('--work', default=DEFAULT_WORK)
    ap.add_argument('--pin-rates', action='store_true', help='node 0 pins the rod rates (drones at rest)')
    a = ap.parse_args()
    run_dir = glob.glob(os.path.join(REPO, 'results', '*', f'{a.run}_*'))[0]
    prm = json.load(open(glob.glob(os.path.join(run_dir, 'logs', '*planner*', '*', 'params.json'))[0]))
    mods = load_modules(a.src)
    n, dyn, solver, refs = setup(mods, prm, a.work, a.pin_rates)
    LD, CD = mods.lcd.LOAD_DIM, mods.lcd.CABLE_DIM
    r, _ = hold_tick(run_dir, a.back)
    ls, pivots = measured(r, n, solver.pivot_offset)
    tgt = float(prm['target_z'])
    refs.update((float(r.load_x), float(r.load_y)), tgt, 0.0, tgt, 0.0, 0.0)

    def elev(xk, i):
        return float(np.degrees(np.arcsin(-xk[LD + CD * i + 2])))

    X = solve(solver, refs, ls, pivots, 30, [np.zeros(3)] * n)
    print(f'{a.run} hold tick t {r.sim_time:.1f}: ring z {r.load_z:.4f} (target {tgt}), logged t '
          f'{[round(r[f"t{i}"], 2) for i in range(n)]}, elev {[round(r[f"elev{i}"], 1) for i in range(n)]}')
    for i in range(n):
        b = LD + CD * i
        vq = np.array(solver.vel_fun[i](X[:, 0], solver._geom)).flatten()
        _, _, acc0, _ = solver.drone_kinematics(X[:, 0], i)
        print(f'slot {i}: node-0 r {np.round(X[b + 3:b + 6, 0], 3)} rad/s, rd {np.round(X[b + 6:b + 9, 0], 2)}, '
              f'implied drone velocity {np.round(vq, 3)} m/s, thrust accel {np.round(acc0, 2)}, tension {X[b + 12, 0]:.3f}')
    print('node  ring_z   ring_vz  t0     elev0  drone0_z  acc0_z  c0_z')
    for k in (0, 1, 2, 5, 10, 15, 20):
        pos, vel, acc, cab = solver.drone_kinematics(X[:, k], 0)
        c = cab + mods.ps.rod_accel_at(solver, X[:, k], 0)
        print(f'{k:4d}  {X[2, k]:.4f}  {X[5, k]:+.4f}  {X[LD + 12, k]:.3f}  {elev(X[:, k], 0):5.1f}  '
              f'{pos[2]:.4f}    {acc[2]:6.3f}  {c[2]:6.3f}')

    # model plant: the next measured state is node 1; drones at the plan's node-1 pivots
    solver.last_X = None
    zs = []
    vel = [np.zeros(3)] * n
    for k in range(a.ticks):
        X = solve(solver, refs, ls, pivots, 1, vel)
        if X is None:
            print(f'model plant: solve failed at tick {k}')
            break
        x1 = X[:, 1]
        ls = np.concatenate([x1[0:3], x1[6:10], x1[3:6], x1[10:13]])
        pivots = [np.array(solver.pos_fun[i](x1, solver._geom)).flatten() for i in range(n)]
        vel = [np.array(solver.vel_fun[i](x1, solver._geom)).flatten() for i in range(n)]
        zs.append(float(x1[2]))
    zs = np.array(zs)
    print(f'model plant from the logged state: ring z after 2/5/10 s {zs[19]:.4f} / {zs[49]:.4f} / '
          f'{zs[min(99, len(zs) - 1)]:.4f}; last {zs[-1]:.4f} (target {tgt}); tension '
          f'{[round(float(X[LD + CD * i + 12, 0]), 3) for i in range(n)]}, elev '
          f'{[round(elev(X[:, 0], i), 1) for i in range(n)]}')


if __name__ == '__main__':
    main()
