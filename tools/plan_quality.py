#!/usr/bin/env python3
"""tools/plan_quality.py -- how far one planner solve is from the converged plan of the same tick (card
2026-10-23_scalability, the 'not heavily worse' bars), for tools/planner_replay.py --quality and
tools/solve_scaling.py --quality.

compare() runs after an arm's solve: it puts the solver back where that solve started (its iterate and the state its
next solve reads), solves the same tick on to convergence (CONV_ITERS warm SQP iterations, no wall budget), measures,
and restores the arm's own iterate and state, so the arm's warm-start chain goes on as if nothing had happened
(SQP_RTI with cold QP starts: the iterate is the whole warm start). Per tick, as q_<name>:

  cost_ratio    J / J_converged (each at its own iterate, the same references)
  res_eq        the arm's dynamics residual
  ref1, ref5, refN   max over drones of |the arm's reference - the converged one| at nodes 1, 5 and N (m), the drone
                position the tracker flies (horizon_kinematics, with the solver's pivot offset)
  ff0           max over drones of the node-0 thrust feed-forward difference (m/s^2)
  t0            max over drones of the node-0 planned tension difference (N)
  thrust_viol   the arm's worst excursion outside the OCP's soft thrust bounds over the horizon (N)
  conv_status   the converged solve's status (a tick whose reference did not converge is left out of the bars)

summarize() also pairs the cost ratio with the residual tick by tick (cost_ratio_paired): a tick whose arm iterate
breaks the dynamics (res_eq over PAIR_RES_EQ, the card's Q2 bar) counts as an infinite ratio, since get_cost on such an
iterate can come out below J_converged (the card's critic, 22 Oct).
"""
import copy

import numpy as np

CONV_ITERS = 30
THRUST_MIN_N, THRUST_MAX_N = 0.5, 15.0          # planner_ocp.generate_load_ocp's soft bounds
# what the next solve_horizon reads or what a tick logs, besides the acados iterate
STATE = ('last_X', 'pending_X', 'recover', '_unconverged', '_q_prev', 'last_res', 'last_res_ms', '_res_due',
         '_res_tick', 'last_status', 'last_iters', 'last_qp_iter', 'last_solve_ms', 'steady_iters', 'solve_budget_s')
QUALITY_KEYS = ('cost_ratio', 'res_eq', 'ref1', 'ref5', 'refN', 'ff0', 't0', 'thrust_viol')
PAIR_RES_EQ = 1e-3        # card 2026-10-23_scalability Q2: above it a tick's cost ratio does not count as met


def snapshot(solver):
    """The acados iterate and the PlannerSolver state a solve reads and writes."""
    return (solver.solver.store_iterate_to_flat_obj(),
            {k: copy.deepcopy(getattr(solver, k)) for k in STATE if hasattr(solver, k)})


def restore(solver, snap):
    solver.solver.load_iterate_from_flat_obj(snap[0])
    for k, v in snap[1].items():
        setattr(solver, k, copy.deepcopy(v))


def _cost(solver):
    try:
        return float(solver.solver.get_cost())
    except Exception:
        return float('nan')


def _nodes(solver, X, n, yaw):
    """Per drone: (positions N+1 x 3, node-0 thrust acceleration)."""
    out = []
    mapped = getattr(solver, 'horizon_kinematics', None)          # a --src tree before 23 Oct has none
    for i in range(n):
        kin = (mapped(X, i, yaw) if mapped is not None
               else [solver.drone_kinematics(X[:, k], i, yaw) for k in range(X.shape[1])])
        out.append((np.array([k[0] for k in kin]), np.asarray(kin[0][2], float)))
    return out


def thrust_excursion(solver, X):
    """The worst excursion of any drone's planned thrust outside [THRUST_MIN_N, THRUST_MAX_N] over the horizon (N)."""
    worst = 0.0
    for i in range(solver.dyn.n):
        for k in range(X.shape[1]):
            f = float(np.linalg.norm(np.array(solver.acc_fun[i](X[:, k], solver._geom)).ravel())) * solver.dyn.mi[i]
            worst = max(worst, THRUST_MIN_N - f, f - THRUST_MAX_N)
    return worst


def compare(solver, before, X, resolve, tension_idx, yaw=0.0, iters=CONV_ITERS):
    """Metrics of the arm's solve X (just made, from the snapshot `before`) against the converged plan of the same
    tick. `resolve()` repeats the tick's solve_horizon call (same references and node-0 state, reseed False);
    tension_idx[i] is drone i's tension row in X. The arm's iterate and state are restored before returning."""
    after = snapshot(solver)
    res = solver.last_res if solver.last_res is not None else solver._residuals()   # deferred or skipped: now
    J = _cost(solver)
    n = solver.dyn.n
    arm = _nodes(solver, X, n, yaw)
    restore(solver, before)
    solver.steady_iters, solver.solve_budget_s = int(iters), 0.0
    Xc, status = resolve()
    Jc = _cost(solver)
    conv_res = solver.last_res if solver.last_res is not None else solver._residuals()
    out = {'q_conv_status': int(status), 'q_res_eq': None if res is None else float(res[1]),
           'q_conv_res_eq': None if conv_res is None else float(conv_res[1]),
           'q_thrust_viol': thrust_excursion(solver, X)}
    if Xc is not None:
        conv = _nodes(solver, Xc, n, yaw)
        d = lambda k: max(float(np.linalg.norm(a[0][min(k, len(a[0]) - 1)] - c[0][min(k, len(c[0]) - 1)]))
                          for a, c in zip(arm, conv))
        out.update(q_cost_ratio=J / Jc if Jc > 0.0 else float('nan'), q_ref1=d(1), q_ref5=d(5),
                   q_refN=d(X.shape[1] - 1), q_ff0=max(float(np.linalg.norm(a[1] - c[1])) for a, c in zip(arm, conv)),
                   q_t0=max(abs(float(X[j, 0]) - float(Xc[j, 0])) for j in tension_idx))
    restore(solver, after)
    return out


def summarize(recs, fails=None):
    """p50 / p95 / max of each metric (compare()'s q_<key>) over the ticks whose converged reference solved."""
    ok = [r for r in recs if r.get('q_conv_status') == 0 and r.get('q_cost_ratio') is not None]
    out = {'ticks': len(ok), 'ticks_ref_failed': len(recs) - len(ok)}
    for k in QUALITY_KEYS:
        v = np.array([r['q_' + k] for r in ok if r.get('q_' + k) is not None and np.isfinite(r['q_' + k])], float)
        out[k] = ({'p50': float(np.percentile(v, 50)), 'p95': float(np.percentile(v, 95)), 'max': float(v.max())}
                  if v.size else None)
    paired = np.array([r['q_cost_ratio'] if r.get('q_res_eq') is not None and r['q_res_eq'] <= PAIR_RES_EQ
                       else np.inf for r in ok if r['q_cost_ratio'] is not None and np.isfinite(r['q_cost_ratio'])],
                      float)
    # 'higher': an interpolated percentile between a ratio and inf is nan
    pc = lambda q: float(np.percentile(paired, q, method='higher'))
    out['cost_ratio_paired'] = ({'p50': pc(50), 'p95': pc(95), 'res_over': int(np.sum(np.isinf(paired)))}
                                if paired.size else None)
    if fails is not None:
        out['fails'] = int(fails)
    return out
