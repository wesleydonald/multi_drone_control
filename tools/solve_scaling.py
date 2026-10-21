#!/usr/bin/env python3
"""tools/solve_scaling.py -- our planner's tick cost against fleet size, wall clock, no Gazebo (plan 2026-10-21 S1).

    python3 tools/solve_scaling.py --n 2 3 4 5 6 7 8 9 10 --iters 5 2 1     # the sweep (the first run builds each size)
    python3 tools/solve_scaling.py --n 4 --iters 5 --ticks 300                # one size

Each fleet size gets the real PlannerSolver, built in its own cache (tools/latency_bench.build_solver, under
generated/solve_scaling), on a ring sized so neighbouring quads stay at least --spacing apart at the planned
elevation with --cable m cables (the 0.225 m ring while it is big enough). It is ticked at the planner's 10 Hz along a
figure-8 at --speed with a kinematic plant: the next measured state is the plan's own node 1, the warm start a
converged closed loop sees. Each tick times the solve (PlannerSolver.solve_horizon, warm SQP at --iters) and the
per-drone reference extraction (drone_kinematics at every node for every drone, as planner_node._plan does). The
table gives p50 / p95 / max per (n, iterations) against --budget (the rig's 60 ms solve cap). Not timed: ROS
messaging and logging, so a real tick costs a little more. The ring inertia is the 0.86 kg ring's per kg
(params.load_inertia): solve time does not depend on it, a flight would.

Run it with nothing else on the machine (never with a twin run or the rig stack up).
"""
import argparse
import json
import math
import os
import platform
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import latency_bench as LB  # noqa: E402

TICK = 0.1


def ring_radius(n, spacing, cable, elev_deg):
    """Smallest ring (>= 0.225 m) whose quads, at elev_deg on cable-m cables, sit spacing apart."""
    reach = cable * math.cos(math.radians(elev_deg))
    return max(0.225, spacing * n / (2.0 * math.pi) - reach)


def tick_cost(mods, n, iters, args):
    R = ring_radius(n, args.spacing, args.cable, args.elev)
    dyn, solver, rho = LB.build_solver(mods, n, args.work, load=args.load, rod=args.cable, radius=R)
    solver.steady_iters = int(iters)
    solver.last_X, solver.recover = None, False
    traj = mods.lt.LoadTrajectory('fig_8', args.speed, 1.0, 0.5)
    refs = mods.rb.ReferenceBuilder(dyn, n, mods.geometry.nominal_cable_dirs(rho, args.elev), solver.dt, traj)
    x = np.asarray(mods.ocp.nominal_hover_state(dyn, load_pos=(0.0, 0.0, 1.0), elevation_deg=args.elev), float)
    solve, extract, fails, traj_t = [], [], 0, 0.0
    for k in range(args.warm + args.ticks):
        traj_t += TICK
        refs.update((0.0, 0.0), 1.0, 0.0, 1.0, 0.0, traj_t)
        t0 = time.perf_counter()
        X, _ = solver.solve_horizon(refs.yref_at, refs.q_ref_at, x, reseed=solver.last_X is None or solver.recover)
        t1 = time.perf_counter()
        if X is None:
            fails += 1
            solver.last_X = solver.pending_X
            solver.recover = solver.last_X is None
            continue
        for i in range(n):
            for kk in range(solver.N + 1):
                solver.drone_kinematics(X[:, kk], i, 0.0)
        t2 = time.perf_counter()
        x = np.asarray(X[:, 1], float)
        if k >= args.warm:
            solve.append(1000.0 * (t1 - t0))
            extract.append(1000.0 * (t2 - t1))
    tot = np.asarray(solve) + np.asarray(extract)

    def pc(a):
        return {'p50': float(np.percentile(a, 50)), 'p95': float(np.percentile(a, 95)), 'max': float(np.max(a))}
    return {'n': n, 'iters': int(iters), 'ring_r': R, 'nx': int(dyn.nx) if hasattr(dyn, 'nx') else None,
            'ticks': len(solve), 'fails': fails, 'solve_ms': pc(solve), 'extract_ms': pc(extract),
            'tick_ms': pc(tot)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, nargs='+', default=list(range(2, 11)))
    ap.add_argument('--iters', type=int, nargs='+', default=[5, 2, 1], help='warm SQP iterations per tick')
    ap.add_argument('--ticks', type=int, default=200)
    ap.add_argument('--warm', type=int, default=20, help='ticks dropped first (the cold reseed and settling)')
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--cable', type=float, default=1.0)
    ap.add_argument('--elev', type=float, default=60.0)
    ap.add_argument('--spacing', type=float, default=0.7, help='m between neighbouring quads')
    ap.add_argument('--load', type=float, default=0.86)
    ap.add_argument('--budget', type=float, default=60.0, help='ms: the rig solve cap (real.yaml)')
    ap.add_argument('--src', default=LB.DEFAULT_SRC)
    ap.add_argument('--work', default=os.path.join(REPO, 'generated', 'solve_scaling'))
    ap.add_argument('--out', help='JSON (default results/analysis/solve_scaling_<host>.json)')
    a = ap.parse_args()
    mods = LB.load_modules(a.src)
    rows = []
    print(f'{"n":>3} {"it":>3} {"ring":>6} {"solve p50/p95/max":>22} {"extract p50/p95":>16} {"tick p95":>9}  fit')
    for n in a.n:
        for it in a.iters:
            r = tick_cost(mods, n, it, a)
            rows.append(r)
            s, e, t = r['solve_ms'], r['extract_ms'], r['tick_ms']
            print(f'{n:>3} {it:>3} {r["ring_r"]:6.3f} {s["p50"]:6.1f} /{s["p95"]:6.1f} /{s["max"]:6.1f} '
                  f'{e["p50"]:7.1f} /{e["p95"]:6.1f} {t["p95"]:9.1f}  '
                  f'{"yes" if t["p95"] <= a.budget and not r["fails"] else "NO"}'
                  + (f'  ({r["fails"]} failed solves)' if r['fails'] else ''), flush=True)
    out = a.out or os.path.join(REPO, 'results', 'analysis', f'solve_scaling_{platform.node()}.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as fh:
        json.dump({'host': platform.node(), 'args': vars(a), 'rows': rows}, fh, indent=1)
    for it in a.iters:
        ok = [r['n'] for r in rows if r['iters'] == it and r['tick_ms']['p95'] <= a.budget and not r['fails']]
        print(f'{it} iteration(s): N_max = {max(ok) if ok else "none"} within {a.budget:g} ms p95')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
