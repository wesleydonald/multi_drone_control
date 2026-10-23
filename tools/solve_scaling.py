#!/usr/bin/env python3
"""tools/solve_scaling.py -- our planner's tick cost against fleet size, wall clock, no Gazebo (plan 2026-10-21 S1).

    python3 tools/solve_scaling.py --n 2 3 4 5 6 7 8 9 10 --iters 5 2 1     # the 22 Oct sweep (first run builds each n)
    python3 tools/solve_scaling.py --n 4 --iters 5 --ticks 300                # one size
    python3 tools/solve_scaling.py --n 2 3 4 5 6 7 8 9 10 --iters 5 2 1 --fast --pivot-dz 0.04 --traj-radius 1.0 \\
        --ring-inertia annulus                                                # card 2026-10-23_scalability's bench

Each fleet size gets the real PlannerSolver, built in its own cache (tools/latency_bench.build_solver, under
generated/solve_scaling), on a ring sized so neighbouring quads stay at least --spacing apart at the planned
elevation with --cable m cables (the 0.225 m ring while it is big enough). It is ticked at the planner's 10 Hz along a
figure-8 of size --traj-radius at --speed with a kinematic plant: the next measured state is the plan's own node 1
(plus --noise), the warm start a converged closed loop sees. Each tick times what the planner node does:
  solve_ms    PlannerSolver.solve_horizon (warm SQP at --iters; with the KKT residual inside unless --fast)
  extract_ms  every drone's references at every node: node by node (drone_kinematics + the rod term), or with
              --fast one mapped call per drone (horizon_kinematics), the drone centre --pivot-dz below each pivot
  latency_ms  solve + extraction: when the references go out
  res_ms      --fast only: the residual computed after them (finish_tick, --res-every as the node's plan_res_every)
  tick_ms     latency + res: the whole tick
The table gives p50 / p95 / max per (n, iterations). N_max, each with no failed solve: the largest n whose latency p95
fits --budget (the rig's 60 ms solve cap), and whose whole tick p95 fits --tick-budget (the 100 ms period), both idle;
then the rig's: --rig-factor x the whole tick p95 within --tick-budget (card 2026-10-23_scalability: the rig laptop in
flight against this bench, 3.5 from the 22 Oct flights h3 and f3h; 1 = idle). Not timed: ROS messaging and logging,
so a real tick costs a little more. --ring-inertia small is the 0.86 kg ring's per kg (params.load_inertia,
the 22 Oct sweep); annulus is the ring the bench sizes (outer +0.03 m, inner -0.03 m, 40 mm thick, --load kg).

--quality also solves every warm tick on to convergence from the same iterate (tools/plan_quality.py) and reports
cost ratio, residual, reference and feed-forward deltas; it adds wall time to the run but not to the timed columns
(time with it off for the card's bench).

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
import plan_quality as PQ  # noqa: E402

TICK = 0.1
RING_WIDTH, RING_THICK = 0.06, 0.04       # the rig-twin ring's radial width and thickness (generate_rigid_world)


def ring_radius(n, spacing, cable, elev_deg):
    """Smallest ring (>= 0.225 m) whose quads, at elev_deg on cable-m cables, sit spacing apart."""
    reach = cable * math.cos(math.radians(elev_deg))
    return max(0.225, spacing * n / (2.0 * math.pi) - reach)


def annulus_inertia(radius, mass):
    """[Ixx, Iyy, Izz] of the rig-twin ring of attach radius `radius` about its CoG
    (generate_rigid_world.ring_inertia)."""
    ro, ri = radius + RING_WIDTH / 2.0, radius - RING_WIDTH / 2.0
    rr = ro * ro + ri * ri
    ixx = mass * (3.0 * rr + RING_THICK ** 2) / 12.0
    return [ixx, ixx, mass * rr / 2.0]


def tick_cost(mods, n, iters, args):
    R = ring_radius(n, args.spacing, args.cable, args.elev)
    inertia = annulus_inertia(R, args.load) if args.ring_inertia == 'annulus' else None
    dyn, solver, rho = LB.build_solver(mods, n, args.work, load=args.load, rod=args.cable, radius=R, inertia=inertia)
    solver.steady_iters = int(iters)
    solver.last_X, solver.recover = None, False
    solver.defer_res, solver.res_every = bool(args.fast), int(args.res_every)
    solver.pivot_offset = np.array([0.0, 0.0, -float(args.pivot_dz)])
    traj = mods.lt.LoadTrajectory('fig_8', args.speed, 1.0, args.traj_radius)
    refs = mods.rb.ReferenceBuilder(dyn, n, mods.geometry.nominal_cable_dirs(rho, args.elev), solver.dt, traj)
    x = np.asarray(mods.ocp.nominal_hover_state(dyn, load_pos=(0.0, 0.0, 1.0), elevation_deg=args.elev), float)
    tension_idx = [mods.lcd.LOAD_DIM + mods.lcd.CABLE_DIM * i + 12 for i in range(n)]
    rng = np.random.default_rng(args.seed)
    col = {k: [] for k in ('solve', 'extract', 'res')}
    qual, fails, traj_t = [], 0, 0.0
    for k in range(args.warm + args.ticks):
        traj_t += TICK
        refs.update((0.0, 0.0), 1.0, 0.0, 1.0, 0.0, traj_t)
        reseed = solver.last_X is None or solver.recover
        before = PQ.snapshot(solver) if args.quality and not reseed and k >= args.warm else None
        t0 = time.perf_counter()
        X, _ = solver.solve_horizon(refs.yref_at, refs.q_ref_at, x, reseed=reseed)
        t1 = time.perf_counter()
        if X is None:
            fails += 1
            solver.last_X = solver.pending_X
            solver.recover = solver.last_X is None
            solver.finish_tick()
            continue
        solver.recover, solver.last_X = False, X       # as planner_node._plan: without it every tick is a cold reseed
        for i in range(n):
            if args.fast:
                solver.horizon_kinematics(X, i, 0.0)
            else:
                for kk in range(solver.N + 1):
                    solver.drone_kinematics(X[:, kk], i, 0.0)
                    mods.ps.rod_accel_at(solver, X[:, kk], i)
        t2 = time.perf_counter()
        solver.finish_tick()
        t3 = time.perf_counter()
        if before is not None:
            qual.append(PQ.compare(solver, before, X, lambda: solver.solve_horizon(
                refs.yref_at, refs.q_ref_at, x, reseed=False), tension_idx))
        x = np.asarray(X[:, 1], float).copy()
        if args.noise > 0.0:
            x[0:3] += rng.normal(0.0, args.noise, 3)
            x[3:6] += rng.normal(0.0, 10.0 * args.noise, 3)
        if k >= args.warm:
            col['solve'].append(1000.0 * (t1 - t0))
            col['extract'].append(1000.0 * (t2 - t1))
            col['res'].append(1000.0 * (t3 - t2))
    s, e, r = (np.asarray(col[c]) for c in ('solve', 'extract', 'res'))

    def pc(a):
        return {'p50': float(np.percentile(a, 50)), 'p95': float(np.percentile(a, 95)), 'max': float(np.max(a))}
    out = {'n': n, 'iters': int(iters), 'ring_r': R, 'inertia': inertia, 'nx': int(dyn.nx),
           'ticks': len(s), 'fails': fails, 'solve_ms': pc(s), 'extract_ms': pc(e), 'res_ms': pc(r),
           'latency_ms': pc(s + e), 'tick_ms': pc(s + e + r)}
    if args.quality:
        out['quality'] = PQ.summarize(qual)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, nargs='+', default=list(range(2, 11)))
    ap.add_argument('--iters', type=int, nargs='+', default=[5, 2, 1], help='warm SQP iterations per tick')
    ap.add_argument('--ticks', type=int, default=200)
    ap.add_argument('--warm', type=int, default=20, help='ticks dropped first (the cold reseed and settling)')
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--traj-radius', type=float, default=0.5, help='the figure-8 size (22 Oct sweep 0.5; card 1.0)')
    ap.add_argument('--cable', type=float, default=1.0)
    ap.add_argument('--elev', type=float, default=60.0)
    ap.add_argument('--spacing', type=float, default=0.7, help='m between neighbouring quads')
    ap.add_argument('--load', type=float, default=0.86)
    ap.add_argument('--ring-inertia', choices=('small', 'annulus'), default='small')
    ap.add_argument('--fast', action='store_true',
                    help="the planner's plan_fast_refs: mapped extraction, residual after")
    ap.add_argument('--res-every', type=int, default=1, help="the planner's plan_res_every")
    ap.add_argument('--pivot-dz', type=float, default=0.0,
                    help='m: the rod pivot below the drone centre (rig twin 0.04)')
    ap.add_argument('--noise', type=float, default=0.0, help='m sd on the measured ring position (10x on velocity)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--quality', action='store_true', help='each warm tick against the converged plan')
    ap.add_argument('--budget', type=float, default=60.0, help='ms: the rig solve cap (real.yaml)')
    ap.add_argument('--tick-budget', type=float, default=100.0, help="ms: the whole tick's budget (the 10 Hz period)")
    ap.add_argument('--rig-factor', type=float, default=1.0,
                    help='the rig laptop in flight over this bench at p95 (card 2026-10-23_scalability: 3.5)')
    ap.add_argument('--src', default=LB.DEFAULT_SRC)
    ap.add_argument('--work', default=os.path.join(REPO, 'generated', 'solve_scaling'))
    ap.add_argument('--out', help='JSON (default results/analysis/solve_scaling_<host>.json)')
    a = ap.parse_args()
    mods = LB.load_modules(a.src)
    rows = []
    print(f'{"n":>3} {"it":>3} {"ring":>6} {"solve p50/p95/max":>22} {"extract p50/p95":>16} {"latency p95":>11} '
          f'{"tick p95":>9}  fit (rig x{a.rig_factor:g} tick p95 <= {a.tick_budget:g} ms)')
    for n in a.n:
        for it in a.iters:
            r = tick_cost(mods, n, it, a)
            rows.append(r)
            s, e, lat, t = r['solve_ms'], r['extract_ms'], r['latency_ms'], r['tick_ms']
            q = r.get('quality')
            print(f'{n:>3} {it:>3} {r["ring_r"]:6.3f} {s["p50"]:6.1f} /{s["p95"]:6.1f} /{s["max"]:6.1f} '
                  f'{e["p50"]:7.1f} /{e["p95"]:6.1f} {lat["p95"]:11.1f} {t["p95"]:9.1f}  '
                  f'{"yes" if a.rig_factor * t["p95"] <= a.tick_budget and not r["fails"] else "NO"}'
                  + (f'  ({r["fails"]} failed solves)' if r['fails'] else '')
                  + (f'  J/Jc {q["cost_ratio"]["p50"]:.3f}/{q["cost_ratio"]["p95"]:.3f} (paired p95 '
                     f'{(q.get("cost_ratio_paired") or {}).get("p95", float("nan")):.3f}) ref1 '
                     f'{1e3 * q["ref1"]["p95"]:.1f} mm ref5 {1e3 * q["ref5"]["p95"]:.1f} mm'
                     if q and q.get('cost_ratio') else ''), flush=True)
    out = a.out or os.path.join(REPO, 'results', 'analysis', f'solve_scaling_{platform.node()}.json')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    n_max = {it: n_max_of(rows, it, a.budget, a.tick_budget, a.rig_factor) for it in a.iters}
    with open(out, 'w') as fh:
        json.dump({'host': platform.node(), 'args': vars(a), 'rows': rows, 'n_max': n_max}, fh, indent=1)
    for it, m in n_max.items():
        print(f"{it} iteration(s): N_max {m['latency']} (idle latency p95 <= {a.budget:g} ms), {m['tick']} (idle "
              f"tick p95 <= {a.tick_budget:g} ms), {m['rig_tick']} (rig: {a.rig_factor:g} x tick p95 <= "
              f"{a.tick_budget:g} ms)")
    print(f'wrote {out}')


def n_max_of(rows, it, budget, tick_budget, rig_factor):
    """The largest n at `it` iterations with no failed solve whose latency p95 fits budget, whose whole tick p95 fits
    tick_budget, and whose whole tick p95 times rig_factor fits tick_budget (None when none does)."""
    def top(fits):
        ok = [r['n'] for r in rows if r['iters'] == it and not r['fails'] and fits(r)]
        return max(ok) if ok else None
    return {'latency': top(lambda r: r['latency_ms']['p95'] <= budget),
            'tick': top(lambda r: r['tick_ms']['p95'] <= tick_budget),
            'rig_tick': top(lambda r: rig_factor * r['tick_ms']['p95'] <= tick_budget)}


if __name__ == '__main__':
    main()
