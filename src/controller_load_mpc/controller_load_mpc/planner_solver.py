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
import hashlib

import numpy as np
import casadi as ca

from . import planner_ocp as _planner_ocp
from . import load_cable_dynamics as _lcd
from .load_cable_dynamics import observed_state_indices, LOAD_DIM, CABLE_DIM
from .planner_ocp import generate_load_ocp, nominal_hover_state
from .geometry import quat_to_rot_np

# Reference horizon, baked into the compiled OCP (the tracker expects N+1 nodes).
# Defined here rather than read off the OCP so the wire format is fixed regardless
# of when the solver finishes building.
PLAN_N = 20
PLAN_TF = 2.0            # s, so node dt 0.1
STEADY_ITERS = 5        # SQP iters/cycle warm; 1 RTI step can't track the transition
COLD_ITERS = 15         # SQP iters on a cold reseed / recovery

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
        p, q = ls[0:3], ls[3:7]
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

    def solve_horizon(self, yref_at, q_ref_at, x_init, reseed):
        """Set the per-node references + the pinned node-0 observed state, run the
        SQP iterations, and return (X, status): the converged horizon X (nx, N+1), or
        (None, status) on failure. `reseed` forces a hard reconverge from x_init
        across all nodes (cold start / recovery, more iterations) instead of warm-
        starting from the solver's last solution. yref_at(k) / q_ref_at(k) supply the
        stage-k tracking reference and the load-attitude parameter."""
        for k in range(self.N):
            self.solver.set(k, 'yref', yref_at(k))
            self.solver.set(k, 'p', np.concatenate([q_ref_at(k), self._geom]))
        self.solver.set(self.N, 'yref', yref_at(self.N)[:-self.dyn.nu])
        self.solver.set(self.N, 'p', np.concatenate([q_ref_at(self.N), self._geom]))
        # Reseed all nodes from x_init when there is no trustworthy warm start; a
        # NaN/garbage warm start would otherwise poison the solve.
        if reseed:
            for k in range(self.N + 1):
                self.solver.set(k, 'x', x_init)
        # Pin only the observed states at node 0 (see _obs_idx / idxbx_0). The full
        # x_init still seeds the warm start above; the unobserved tensions and cable
        # rates are left free so they cannot ratchet against a measured pose.
        self.solver.set(0, 'lbx', x_init[self._obs_idx])
        self.solver.set(0, 'ubx', x_init[self._obs_idx])
        # A single RTI iteration cannot track the stiff cable transition online, so
        # take several SQP iterations per cycle (more on a cold reseed).
        status = 0
        for _ in range(COLD_ITERS if reseed else STEADY_ITERS):
            status = self.solver.solve()
        if status != 0:
            return None, status
        X = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)]).T
        return X, status

    def drone_kinematics(self, xk, i):
        """(pos, vel, thrust_accel, cable_accel) for the drone in OCP slot i at
        horizon-node state xk, extracted from the solved horizon."""
        g = self._geom
        return (np.array(self.pos_fun[i](xk, g)).flatten(),
                np.array(self.vel_fun[i](xk, g)).flatten(),
                np.array(self.acc_fun[i](xk, g)).flatten(),
                np.array(self.cable_fun[i](xk, g)).flatten())
