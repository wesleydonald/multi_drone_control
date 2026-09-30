"""
planner_solver.py
-----------------
Thin wrapper around the acados load-cable OCP, factored out of the planner node.
PlannerSolver owns everything solver-side -- it builds (or loads a cached) solver
for the given geometry, holds the CasADi functions that extract each drone's
reference from a solved horizon, and carries the warm-start state (last_X /
recover). It exposes build_x_init + solve_horizon.

Reference GENERATION (yref / q_ref / the hold reference) stays in the node: it is
coupled to the flight phase, lift schedule and load trajectory, none of which the
solver needs to know about. The node passes those in as callables, so this module
stays purely about the mechanics of seeding, solving and reading back the OCP.
"""
import os
import time
import hashlib

import numpy as np
import casadi as ca

from . import planner_ocp as _planner_ocp
from . import load_cable_dynamics as _lcd
from .load_cable_dynamics import observed_state_indices, LOAD_DIM, CABLE_DIM
from .planner_ocp import generate_load_ocp, nominal_hover_state
from .geometry import quat_to_rot_np, centre_from_pivot, quat_same_hemisphere

# Reference horizon, baked into the compiled OCP (the tracker expects N+1 nodes).
# Defined here rather than read off the OCP so the wire format is fixed regardless
# of when the solver finishes building.
PLAN_N = 20
PLAN_TF = 2.0            # s, so node dt 0.1
STEADY_ITERS = 5        # SQP iters/cycle warm; 1 RTI step can't track the transition
COLD_ITERS = 15         # SQP iters on a cold reseed / recovery
# s wall cap on a solve's SQP iterations (they block the 10 Hz loop). 0 = off: sim, SIL and
# the lab keep a load-independent iteration count; the rig launch sets 0.06.
SOLVE_BUDGET_S = 0.0
# A reseeded or budget-cut solve can return status 0 with the QP solved but the NLP far
# from converged; above this dynamics residual it is not published. Replay model-f1: a cold
# reseed reads ~1 after 5 iterations (next-node refs 1-4 cm off), <= 1e-2 within 0.5 cm,
# ~1e-5 after 15; warm solves <= 1e-5.
RES_EQ_MAX = 1e-2
STATUS_UNCONVERGED = 20  # planner's own code (acados statuses stop at 7)
MAX_BRIDGE_S = 0.5      # longest the fallback may bridge; + the 1.0 s tracker watchdog < 1.5 s
JITTER_NODES = 0.25     # timer slack on the fallback's elapsed-time bound
MAX_FAIL_DUMPS = 50     # planner_fail_<n>.npz files per process
MAX_QP_DUMPS = 5        # of those with the last QP as JSON (~5 MB each at n=4)

# acados solver cache: the .so bakes in the geometry (n, cable_len, load_mass, rho,
# inertia) as compile-time constants, so a rebuild is needed when a SOURCE file
# changes OR the geometry signature changes.
CODE_DIR = 'c_generated_code_load_planner'
SRC_FILES = [_planner_ocp.__file__, _lcd.__file__]


# ── acados solver-cache freshness ─────────────────────────────────────────────
# The compiled .so records a geometry+horizon signature file next to it; we reuse
# the .so only when it exists, is newer than its sources, and its signature matches
# the current geometry (else the CasADi model's compile-time constants are stale).

def _ocp_signature(dyn, plan_n, plan_tf):
    """Hash of everything STILL baked into the compiled OCP, so a cached .so from a
    different load_mass / inertia / fleet size is never reused.

    rho_i and l_i are deliberately absent: they are runtime parameters now, so a
    cached solver is valid across attachment layouts and cable lengths, and changing
    attach_radius no longer costs a ~50 s rebuild."""
    d = dyn
    parts = (d.n, round(float(d.m), 6),
             tuple(round(float(v), 6) for v in np.asarray(d.J).ravel()),
             tuple(round(float(v), 6) for v in d.mi),
             plan_n, round(float(plan_tf), 6))
    return hashlib.md5(repr(parts).encode()).hexdigest()


def cached_solver_fresh(code_dir, src_files, n, dyn, plan_n, plan_tf):
    """True if the compiled solver exists, is newer than its sources, and was built
    for the current geometry -> load it instead of recompiling."""
    so = os.path.join(code_dir, f'libacados_ocp_solver_load_cable_{n}.so')
    sig = os.path.join(code_dir, f'.build_sig_{n}')
    if not (os.path.exists(so) and os.path.exists(sig)):
        return False
    so_mtime = os.path.getmtime(so)
    if any(not os.path.exists(s) or os.path.getmtime(s) > so_mtime
           for s in src_files):
        return False
    try:
        with open(sig) as f:
            return f.read().strip() == _ocp_signature(dyn, plan_n, plan_tf)
    except OSError:
        return False


def write_solver_signature(code_dir, n, dyn, plan_n, plan_tf, logger=None):
    """Record the current geometry+horizon signature next to the freshly built .so
    so the next launch can decide whether to reuse it."""
    try:
        with open(os.path.join(code_dir, f'.build_sig_{n}'), 'w') as f:
            f.write(_ocp_signature(dyn, plan_n, plan_tf))
    except OSError as e:
        if logger is not None:
            logger.warn(f'[planner] could not write build signature: {e}')


class PlannerSolver:
    def __init__(self, dyn, logger=None):
        self.dyn = dyn
        n = dyn.n
        # Build the OCP. acados recompiles it every launch (~50 s for n=4) unless the
        # cached .so matches the geometry + sources, in which case we reuse it.
        fresh = cached_solver_fresh(CODE_DIR, SRC_FILES, n, dyn, PLAN_N, PLAN_TF)
        if logger is not None:
            if fresh:
                logger.info('[planner] loading cached OCP solver (geometry + '
                            'sources unchanged) — skipping the acados rebuild.')
            else:
                logger.info(f'[planner] compiling OCP (nx={dyn.nx}, nu={dyn.nu}) '
                            f'— first build for this geometry, tens of seconds...')
        self.ocp, self.solver = generate_load_ocp(
            dyn, N=PLAN_N, tf=PLAN_TF, generate=not fresh, build=not fresh)
        if not fresh:
            write_solver_signature(CODE_DIR, n, dyn, PLAN_N, PLAN_TF, logger)
        self.N = self.ocp.solver_options.N_horizon
        self.dt = self.ocp.solver_options.tf / self.N
        # Current attachment geometry, sent with every solve. Starts at the nominal
        # ring this dyn was built with, so behaviour is unchanged until something
        # calls set_geometry().
        self._geom = self.dyn.geom_values()

        # states pinned at OCP node 0 (observed): load pose/twist + cable dirs s_i.
        # Must match idxbx_0 in generate_load_ocp. r_i/tensions stay free.
        self._obs_idx = observed_state_indices(n)

        # Reference-extraction functions from a solved horizon state. They take the
        # geometry parameter as a second input for the same reason the model does:
        # quad_position and friends are functions OF rho and l, so a drone's extracted
        # reference has to be computed against the layout the fleet actually has.
        self.pos_fun = [ca.Function(f'p{i}', [dyn.x, dyn.p_geom],
                                    [dyn.quad_position(i)]) for i in range(n)]
        self.vel_fun = [ca.Function(f'v{i}', [dyn.x, dyn.p_geom],
                                    [dyn.quad_velocity(i)]) for i in range(n)]
        # required specific thrust acceleration a_i = f_i / m_i (feedforward)
        self.acc_fun = [ca.Function(f'a{i}', [dyn.x, dyn.p_geom],
                                    [dyn.thrust_vec(i) / dyn.mi[i]])
                        for i in range(n)]
        # cable tension acceleration a_cable_i = t_i s_i / m_i (world frame) — the
        # known external pull the cable-aware tracker adds to its drone model
        self.cable_fun = [ca.Function(f'ac{i}', [dyn.x, dyn.p_geom],
                                      [dyn.cable_accel(i)]) for i in range(n)]

        # warm-start state. last_X is None => no valid warm start => reseed.
        self.last_X = None
        self.recover = False
        self._q_prev = None                  # last fed load quaternion (sign continuity)
        self.solve_budget_s = SOLVE_BUDGET_S
        self.pivot_offset = np.zeros(3)      # rod pivot in the drone body frame
        self.fail_dump_dir = None            # used when MDC_RUN_DIR is unset
        self._reset_stats()

    def _reset_stats(self):
        self.last_status = None
        self.last_solve_ms = float('nan')
        self.last_qp_iter = -1
        self.last_iters = 0
        self.last_res = None               # [stat, eq, ineq, comp] of the last solve
        self.pending_X = None              # iterate of an unconverged solve, to continue from
        self._unconverged = False

    def build_x_init(self, load_state, drone_slot_pos):
        """OCP initial guess from measured state: resample the previous solution
        (advance one node) or nominal hover, then overwrite the load block p,v,q,w
        and each cable DIRECTION s_i from mocap. The cable RATES r_i and everything
        above them (rd_i, rdd_i, t_i, td_i) stay RESAMPLED from the previous solution
        -- per the paper (Fig 8), which resamples these rather than differentiating
        noisy estimator values. drone_slot_pos[i] is the measured position of the
        physical drone occupying OCP slot i."""
        dyn = self.dyn
        ls = load_state
        p = ls[0:3]
        # keep q in the hemisphere of the warm start (else the last fed q): a sign flip
        # at node 0 against last_X failed the solve on every yaw crossing of 180 deg
        q_ref = self.last_X[6:10, 1] if self.last_X is not None else self._q_prev
        q = quat_same_hemisphere(ls[3:7], q_ref)
        self._q_prev = q.copy()
        R = quat_to_rot_np(q)
        if self.last_X is not None:
            x = self.last_X[:, 1].copy()
        else:
            x = nominal_hover_state(dyn, load_pos=tuple(p))
        x[0:3] = p
        x[3:6] = ls[7:10]      # v
        x[6:10] = q            # q (wxyz)
        x[10:13] = ls[10:13]   # w
        for i in range(dyn.n):
            b = LOAD_DIM + CABLE_DIM * i
            attach = p + R @ dyn.rho[i]
            d = attach - drone_slot_pos[i]              # points drone -> load
            nrm = np.linalg.norm(d)
            if nrm > 1e-6:
                x[b:b + 3] = d / nrm
        return x

    def set_geometry(self, rho=None, cable_lengths=None):
        """Change the attachment layout the solver plans against, without rebuilding.

        This is the point of parameterising the geometry: a welded newcomer attaches
        wherever its magnet landed, and after a detach the survivors sit on a
        different ring. Both are a parameter change now, not a ~50 s acados rebuild
        that cannot happen mid-flight.

        `dyn.rho` / `dyn.l` are updated to match, because the reference extraction
        below reads them numerically to place each drone's attach point."""
        if rho is not None:
            self.dyn.rho = [np.asarray(r, float).reshape(3) for r in rho]
        if cable_lengths is not None:
            self.dyn.l = [float(v) for v in cable_lengths]
        self._geom = self.dyn.geom_values()
        return self._geom

    def solve_horizon(self, yref_at, q_ref_at, x_init, reseed, context=None):
        """Set the per-node references + the pinned node-0 observed state, run the
        SQP iterations, and return (X, status): the converged horizon X (nx, N+1), or
        (None, status) on failure. `reseed` forces a hard reconverge from x_init
        across all nodes (cold start / recovery, more iterations) instead of warm-
        starting from the solver's last solution. yref_at(k) / q_ref_at(k) supply the
        stage-k tracking reference and the load-attitude parameter. `context` (dict of
        arrays: poses, slot map) goes into the failure dump only."""
        t0 = time.perf_counter()
        yrefs = [np.asarray(yref_at(k), float) for k in range(self.N + 1)]
        qrefs = [np.asarray(q_ref_at(k), float) for k in range(self.N + 1)]
        for k in range(self.N):
            self.solver.set(k, 'yref', yrefs[k])
            self.solver.set(k, 'p', np.concatenate([qrefs[k], self._geom]))
        self.solver.set(self.N, 'yref', yrefs[self.N][:-self.dyn.nu])
        self.solver.set(self.N, 'p', np.concatenate([qrefs[self.N], self._geom]))
        # Reseed all nodes from x_init when there is no trustworthy warm start. Setting x
        # alone kept the u, slacks and multipliers of a NaN/MINSTEP iterate (rig model-f1:
        # 25 failures, 60 of 68 at QP iteration 1), so the whole iterate is zeroed first.
        if reseed:
            self.solver.reset()
            u0 = np.zeros(self.dyn.nu)
            for k in range(self.N + 1):
                self.solver.set(k, 'x', x_init)
                if k < self.N:
                    self.solver.set(k, 'u', u0)
        # Pin only the observed states at node 0 (see _obs_idx / idxbx_0). The full
        # x_init still seeds the warm start above; the unobserved tensions and cable
        # rates are left free so they cannot ratchet against a measured pose.
        self.solver.set(0, 'lbx', x_init[self._obs_idx])
        self.solver.set(0, 'ubx', x_init[self._obs_idx])
        # A single RTI iteration cannot track the stiff cable transition online, so
        # take several SQP iterations per cycle (more on a cold reseed). A failed step
        # ends the solve: the steps after it only cost time (replay: 500 ms ticks).
        n_max = COLD_ITERS if reseed else STEADY_ITERS
        n_min = STEADY_ITERS if reseed else 1
        status = 0
        qp_iter = 0
        iters = 0
        cut = False
        while iters < n_max:
            status = self.solver.solve()
            iters += 1
            try:
                qp_iter += int(np.sum(self.solver.get_stats('qp_iter')))
            except Exception:
                qp_iter = -1
            if status != 0:
                break
            if (self.solve_budget_s > 0.0 and n_min <= iters < n_max
                    and time.perf_counter() - t0 > self.solve_budget_s):
                cut = True
                break
        self.last_res = self._residuals()
        if (status == 0 and (reseed or cut or self._unconverged) and self.last_res is not None
                and not self.last_res[1] <= RES_EQ_MAX):
            status = STATUS_UNCONVERGED
        self._unconverged = status == STATUS_UNCONVERGED
        self.last_status = int(status)
        self.last_iters = iters
        self.last_qp_iter = qp_iter
        self.last_solve_ms = (time.perf_counter() - t0) * 1e3
        X = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)]).T if status in (
            0, STATUS_UNCONVERGED) else None
        self.pending_X = X if status == STATUS_UNCONVERGED else None
        if status != 0:
            if status != STATUS_UNCONVERGED:
                self._dump_failure(x_init, yrefs, qrefs, reseed, context)
            return None, status
        return X, status

    def _residuals(self):
        """[stat, eq, ineq, comp] of the current iterate, None if the solver has none."""
        try:
            return np.asarray(self.solver.get_residuals(recompute=True), float).ravel()
        except Exception:
            return None

    _fail_count = 0                          # shared by every solver in the process

    def _dump_failure(self, x_init, yrefs, qrefs, reseed, context):
        """planner_fail_<n>.npz (+ the last QP as JSON) in MDC_RUN_DIR or the planner's
        log dir, for offline replay of the exact failing solve. Never raises."""
        try:
            out = os.environ.get('MDC_RUN_DIR') or self.fail_dump_dir
            if not out or PlannerSolver._fail_count >= MAX_FAIL_DUMPS:
                return
            PlannerSolver._fail_count += 1
            k = PlannerSolver._fail_count
            os.makedirs(out, exist_ok=True)
            extra = {}
            for key, v in (context or {}).items():
                try:
                    extra[f'ctx_{key}'] = np.asarray(v, float)
                except (TypeError, ValueError):
                    extra[f'ctx_{key}'] = np.asarray(str(v))
            last_X = self.last_X if self.last_X is not None else np.zeros((0,))
            np.savez(os.path.join(out, f'planner_fail_{k}.npz'),
                     x_init=np.asarray(x_init, float), last_X=np.asarray(last_X, float),
                     yref=np.array(yrefs), q_ref=np.array(qrefs),
                     geom=np.asarray(self._geom, float),
                     rho=np.array([np.asarray(r, float) for r in self.dyn.rho]),
                     cable_len=np.asarray(self.dyn.l, float),
                     status=self.last_status, iters=self.last_iters, reseed=bool(reseed),
                     solve_ms=self.last_solve_ms, n=self.dyn.n, **extra)
            dump = getattr(self.solver, 'dump_last_qp_to_json', None)
            if dump is not None and k <= MAX_QP_DUMPS:
                dump(os.path.join(out, f'planner_fail_{k}_qp.json'), overwrite=True)
        except Exception:
            pass

    def drone_kinematics(self, xk, i, yaw=0.0):
        """(pos, vel, thrust_accel, cable_accel) for the drone in OCP slot i at
        horizon-node state xk, extracted from the solved horizon. The OCP's drone is the
        rod pivot; pos is moved to the drone CENTRE (what the tracker flies) at the
        attitude that node's thrust asks for, heading `yaw`. vel is left at the pivot's
        (the attitude-rate term is a few mm/s)."""
        g = self._geom
        acc = np.array(self.acc_fun[i](xk, g)).flatten()
        pos = np.array(self.pos_fun[i](xk, g)).flatten()
        return (centre_from_pivot(pos, acc, yaw, self.pivot_offset),
                np.array(self.vel_fun[i](xk, g)).flatten(),
                acc,
                np.array(self.cable_fun[i](xk, g)).flatten())


def shift_horizon(X, k):
    """The horizon X (nx, N+1) advanced k nodes, the last node repeated at the end."""
    X = np.asarray(X)
    k = int(np.clip(k, 0, X.shape[1] - 1))
    return np.concatenate([X[:, k:], np.repeat(X[:, -1:], k, axis=1)], axis=1)


class HorizonFallback:
    """What to publish when a solve fails: the last good horizon advanced by the time
    since that solve, for at most `max_shift` node periods, then nothing, so the
    trackers' reference watchdog (message arrival age, 1.0 s on the rig) still trips on
    a planner that keeps failing (critic, card 2026-10-01 must-fix 1). Elapsed time, not
    the failure count, sets the shift, so an overrunning failed tick stays in step."""

    def __init__(self, max_shift, dt):
        self.max_shift = int(max_shift)
        self.dt = float(dt)
        if self.max_shift < 0 or (self.max_shift + JITTER_NODES) * self.dt > MAX_BRIDGE_S:
            raise ValueError(f'max_shift_publishes {max_shift} must be 0..'
                             f'{int(MAX_BRIDGE_S / self.dt - JITTER_NODES)} '
                             f'(bridge <= {MAX_BRIDGE_S} s at dt {self.dt})')
        self.reset()

    def reset(self):
        self.X = None
        self.t_good = None
        self.streak = 0

    def success(self, X, t):
        self.X = X
        self.t_good = float(t)
        self.streak = 0

    def failure(self, now):
        """Horizon to publish for a tick at `now` whose solve failed, or None."""
        self.streak += 1
        if self.X is None:
            return None
        el = float(now) - self.t_good
        if el > (self.max_shift + JITTER_NODES) * self.dt:
            return None
        return shift_horizon(self.X, max(1, int(round(el / self.dt))))

    def age(self, now):
        return float('nan') if self.t_good is None else float(now) - self.t_good
