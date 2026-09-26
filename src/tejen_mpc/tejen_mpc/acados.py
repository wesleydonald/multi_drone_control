import os
from pathlib import Path

import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics, QuadPayloadDynamics
from .mpc_modes import base_payload_cost_matrices
from .reference_feedforward import acceleration_feedforward_reference
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver
from .acados_cache import (
    build_payload_mpc_cache_identity,
    locked_payload_mpc_cache,
    payload_mpc_cache_is_reusable,
    write_payload_mpc_cache_manifest,
)



# ---------------------------
# Quaternion helper functions
# ---------------------------
def quat_conj(q):
    """Conjugate of quaternion q = [w, x, y, z]."""
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])


def quat_mul(q1, q2):
    """Hamilton product q1 ⊗ q2, both [w, x, y, z]"""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return ca.vertcat(
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )


# --------------------------------
# Public API: build OCP & simulator
# --------------------------------
def generate_payload_ocp_controller(
    xy_integral_weight=15.0,
    xy_integral_terminal_weight=30.0,
    acados_cache_dir=None,
):
    quad_dynamics = QuadPayloadDynamics()
    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_payload_integral_dynamics'

    # Augment the existing 21-state payload model with two integral states:
    #   xi_x_dot = x_ref - x
    #   xi_y_dot = y_ref - y
    # These states provide offset-free XY tracking without changing the base
    # QuadPayloadDynamics class used elsewhere (including the UKF simulator).
    xy_integral = ca.MX.sym('xy_integral', 2)
    model.x = ca.vertcat(quad_dynamics.x, xy_integral)
    model.u = quad_dynamics.u_dot

    # Payload dynamics params:
    # [thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg,
    #  max_rate_deg, rate_expo, cable_length]
    p_dyn = quad_dynamics.p_param

    # OCP-only reference / disturbance parameters:
    # quaternion reference [qw, qx, qy, qz], current XY position reference,
    # and a slowly varying world-frame lateral acceleration disturbance [m/s^2].
    p_qref = ca.MX.sym('p_qref', 4)
    p_xy_ref = ca.MX.sym('p_xy_ref', 2)
    p_lateral_disturbance = ca.MX.sym('p_lateral_disturbance', 2)
    model.p = ca.vertcat(p_dyn, p_qref, p_xy_ref, p_lateral_disturbance)

    # Existing 21-state dynamics plus an additive XY acceleration mismatch and
    # the legacy XY integral-error dynamics.  The disturbance is deliberately
    # applied only to quad translation in this first, minimal implementation;
    # payload dynamics retain the commissioned model and swing cost unchanged.
    base_dynamics = dynamics_expr(model.x[:21], model.u, model.p[:7])
    disturbed_base_dynamics = ca.vertcat(
        base_dynamics[0:7],
        base_dynamics[7:9] + model.p[13:15],
        base_dynamics[9:21],
    )
    xy_integral_dot = model.p[11:13] - model.x[0:2]
    model.f_expl_expr = ca.vertcat(disturbed_base_dynamics, xy_integral_dot)

    xdot = ca.MX.sym('xdot', model.x.size()[0])
    model.xdot = xdot
    model.f_impl_expr = model.f_expl_expr - xdot

    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 2.0

    nu = 4
    nx = 23

    # ---------- COST: NONLINEAR_LS with sign-invariant quaternion error ----------
    x = model.x
    u = model.u

    q_idx = 3
    q = x[q_idx:q_idx + 4]

    # q_ref now starts after 7 dynamic parameters
    q_ref = model.p[7:11]

    q_err = quat_mul(q_ref, quat_conj(q))
    e_att = 2 * q_err[1:4]

    payload_state = x[17:21]  # [phi, theta, phi_dot, theta_dot]
    xy_integral_state = x[21:23]  # integral of [x_ref - x, y_ref - y]

    # Layout:
    # [pos(3), vel(3), omega(3), u_state(4), u_dot(4),
    #  e_att(3), payload(4), xy_integral(2)]
    # total ny = 26
    y_expr = ca.vertcat(
        x[0:3],
        x[7:10],
        x[10:13],
        x[13:17],
        u,
        e_att,
        payload_state,
        xy_integral_state
    )

    # Terminal:
    # [pos(3), vel(3), omega(3), u_state(4), e_att(3),
    #  payload(4), xy_integral(2)]
    # total ny_e = 22
    y_expr_e = ca.vertcat(
        x[0:3],
        x[7:10],
        x[10:13],
        x[13:17],
        e_att,
        payload_state,
        xy_integral_state
    )

    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ny = 3 + 3 + 3 + 4 + 4 + 3 + 4 + 2
    ny_e = 3 + 3 + 3 + 4 + 3 + 4 + 2

    # Keep one canonical source of truth for the commissioned FREE_SWING
    # weights. Runtime M2 modes derive copies from these matrices and may only
    # suppress the payload-swing block.
    W, W_e = base_payload_cost_matrices(
        xy_integral_weight=xy_integral_weight,
        xy_integral_terminal_weight=xy_integral_terminal_weight,
    )

    ocp.cost.W = W
    ocp.cost.W_e = W_e  

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # Initial condition
    x0 = np.zeros(nx)
    ocp.constraints.x0 = x0

    # Parameters:
    # [thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg,
    #  max_rate_deg, rate_expo, cable_length,
    #  q_ref_w, q_ref_x, q_ref_y, q_ref_z,
    #  x_ref, y_ref, disturbance_ax, disturbance_ay]
    ocp.parameter_values = np.array([
        38.0, 0.5, 0.07, 100.0, 100.0, 0.5,
        0.533,      # cable length taken from model thing
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
    ])

    # ---------- Solver options ----------
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.qp_solver_iter_max = 250

    ocp.solver_options.qp_solver_tol_stat = 1e-3
    ocp.solver_options.qp_solver_tol_eq = 1e-3
    ocp.solver_options.qp_solver_tol_ineq = 1e-3
    ocp.solver_options.qp_solver_tol_comp = 1e-3

    ocp.solver_options.nlp_solver_tol_stat = 1e-3
    ocp.solver_options.nlp_solver_tol_eq = 1e-3
    ocp.solver_options.nlp_solver_tol_ineq = 1e-3
    ocp.solver_options.nlp_solver_tol_comp = 1e-3

    ocp.solver_options.levenberg_marquardt = 1e-3

    # ---------- State constraints ----------
    # State layout:
    # 0:3 pos, 3:7 quat, 7:10 vel, 10:13 omega,
    # 13:17 control state, 17:21 payload state, 21:23 XY integral state
    max_rate = 1.0

    ocp.constraints.lbx = np.array([0.05, -max_rate, -max_rate, -max_rate])
    ocp.constraints.ubx = np.array([0.6,  max_rate,  max_rate,  max_rate])
    ocp.constraints.idxbx = np.array([15, 13, 14, 16])

    # Input bounds:
    # u_dot = [roll_dot, pitch_dot, throttle_dot, yaw_dot]
    ocp.constraints.lbu = np.array([-1.0, -1.0, -0.5, -1.0])
    ocp.constraints.ubu = np.array([ 1.0,  1.0,  0.5,  1.0])
    ocp.constraints.idxbu = np.arange(nu)

    # The 21-state simulation integrator is configured below before either solver
    # is constructed.  M2C may opt into a shared generated-artifact cache so four
    # independent controller processes load identical compiled libraries rather
    # than recompiling them in four separate working directories.

    # Create the 21-state simulation integrator separately. The UKF propagation
    # in main.py expects the original state layout and does not need the controller's
    # two XY integral states.
    sim_dynamics = QuadPayloadDynamics()
    sim_dynamics_expr = sim_dynamics.quad_dynamics()

    sim_model = AcadosModel()
    sim_model.name = 'quad_payload_dynamics_sim'
    sim_model.x = sim_dynamics.x
    sim_model.u = sim_dynamics.u_dot

    sim_qref = ca.MX.sym('sim_qref', 4)
    sim_model.p = ca.vertcat(sim_dynamics.p_param, sim_qref)
    sim_model.f_expl_expr = sim_dynamics_expr(
        sim_model.x,
        sim_model.u,
        sim_model.p[:7]
    )

    sim_xdot = ca.MX.sym('sim_xdot', sim_model.x.size()[0])
    sim_model.xdot = sim_xdot
    sim_model.f_impl_expr = sim_model.f_expl_expr - sim_xdot

    sim = AcadosSim()
    sim.model = sim_model
    sim.solver_options.T = 1.0 / 30.0

    sim.parameter_values = np.array([
        38.0, 0.5, 0.12, 100.0, 100.0, 0.5,
        0.50,
        1.0, 0.0, 0.0, 0.0
    ])

    if not acados_cache_dir:
        # Preserve the historical behavior for every non-M2C launch unless it
        # explicitly opts into artifact reuse.
        ocp_solver = AcadosOcpSolver(ocp)
        sim_solver = AcadosSimSolver(sim)
        return ocp_solver, sim_solver

    source_dir = Path(__file__).resolve().parent
    identity = build_payload_mpc_cache_identity(
        source_paths=(
            Path(__file__),
            source_dir / 'dynamics.py',
            source_dir / 'mpc_modes.py',
        ),
        xy_integral_weight=xy_integral_weight,
        xy_integral_terminal_weight=xy_integral_terminal_weight,
    )
    force_rebuild = os.environ.get('PAYLOAD_MPC_REBUILD', '0').lower() in (
        '1', 'true', 'yes'
    )

    # Wesley's proven multi-drone pattern: one generated-code location guarded by
    # an inter-process flock, then generate/build=False for every compatible
    # follower process.  We add a content fingerprint so newer thesis MPC source
    # or integral weights cannot silently reuse a stale solver.
    with locked_payload_mpc_cache(acados_cache_dir, identity.fingerprint) as layout:
        need_build = force_rebuild or not payload_mpc_cache_is_reusable(
            layout, identity
        )
        print(
            '[payload_mpc] acados cache: '
            f'{"BUILD" if need_build else "reuse cached"} '
            f'fingerprint={identity.fingerprint[:12]} root={layout.root} '
            f'(force_rebuild={force_rebuild})'
        )

        # acados generates several relative build products even when an export
        # directory is supplied.  Temporarily enter the shared cache root, just as
        # Wesley does, then restore this controller's isolated M2C CWD before any
        # runtime logging is created.
        previous_cwd = Path.cwd()
        try:
            os.chdir(layout.root)
            ocp.code_export_directory = 'c_generated_code'
            sim.code_export_directory = 'c_generated_code'
            ocp_solver = AcadosOcpSolver(
                ocp,
                json_file=str(layout.ocp_json.name),
                generate=need_build,
                build=need_build,
            )
            sim_solver = AcadosSimSolver(
                sim,
                json_file=str(layout.sim_json.name),
                generate=need_build,
                build=need_build,
            )
        finally:
            os.chdir(previous_cwd)

        if need_build:
            write_payload_mpc_cache_manifest(layout, identity)

    return ocp_solver, sim_solver

# def generate_ocp_controller(dynamics=None):
#     if dynamics is None:
#         quad_dynamics = QuadDynamics()
#     else:
#         quad_dynamics = dynamics

#     dynamics_expr = quad_dynamics.quad_dynamics()

#     model = AcadosModel()
#     model.name = 'quad_dynamics'
#     model.x = quad_dynamics.x 
#     model.u = quad_dynamics.u_dot
#     p_dyn = quad_dynamics.p_param 
#     p_qref = ca.MX.sym('p_qref', 4)           
#     model.p = ca.vertcat(p_dyn, p_qref) 

#     model.f_expl_expr = dynamics_expr(model.x, model.u, model.p[:6])

#     xdot = ca.MX.sym('xdot', model.x.size()[0])
#     f_impl_expr = model.f_expl_expr - xdot
#     model.xdot = xdot
#     model.f_impl_expr = f_impl_expr

#     ocp = AcadosOcp()
#     ocp.model = model
#     ocp.solver_options.N_horizon = 20
#     ocp.solver_options.tf = 2.0

#     nu = 4
#     nx = 21

#     # ---------- COST: NONLINEAR_LS with sign-invariant quaternion error ----------
#     q_idx = 3
#     x = model.x
#     u = model.u
#     q = x[q_idx:q_idx+4]
#     q_ref = model.p[6:10]

#     q_err = quat_mul(q_ref, quat_conj(q))
#     e_att = 2 * q_err[1:4]        

#     # Layout: [pos(3), vel(3), omega(3), u_state(4), u(4), e_att(3)] -> total ny = 20
#     y_expr   = ca.vertcat(x[0:3], x[7:10], x[10:13], x[13:17], u, e_att)
#     y_expr_e = ca.vertcat(x[0:3], x[7:10], x[10:13], x[13:17], e_att) 

#     ocp.model.cost_y_expr = y_expr
#     ocp.model.cost_y_expr_e = y_expr_e

#     ocp.cost.cost_type = 'NONLINEAR_LS'
#     ocp.cost.cost_type_e = 'NONLINEAR_LS'

#     ny = 3 + 3 + 3 + 4 + 4 + 3      
#     ny_e = 3 + 3 + 3 + 4 + 3        

#     W = np.diag([
#         80.0, 80.0, 40.0,
#         2.0, 2.0, 2.0,
#         0.2, 0.2, 0.2,
#         2e-4, 2e-4, 2e-4, 2e-4,
#         0.1, 0.1, 5.0, 0.1,  # Reduced control effort penalty
#         0.5, 0.5, 5.0
#     ])
#     W_e = np.diag([
#         80.0, 80.0, 40.0,         # pos
#         2.0, 2.0, 2.0,        # vel
#         0.2, 0.2, 0.2,         # omega
#         2e-4, 2e-4, 2e-4, 2e-4,# u_state
#         0.5, 0.5, 5.0          # attitude error
#     ])
#     ocp.cost.W = W
#     ocp.cost.W_e = W_e

#     ocp.cost.yref = np.zeros((ny,))
#     ocp.cost.yref_e = np.zeros((ny_e,))

#     # Initial condition
#     x0 = np.zeros(nx)
#     ocp.constraints.x0 = x0

#     # -------- Parameters default (6 dyn + 4 q_ref) --------
#     ocp.parameter_values = np.array([38.0, 0.5, 0.07, 100.0, 100.0, 0.5, 1.0, 0.0, 0.0, 0.0])

#     # ---------- Solver options ----------
#     ocp.solver_options.nlp_solver_type = 'SQP_RTI'
#     ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
#     ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
#     ocp.solver_options.nlp_solver_max_iter = 50  # Reduced for real-time performance with full SQP
#     ocp.solver_options.qp_solver_iter_max = 250
#     ocp.solver_options.qp_solver_tol_stat = 1e-3
#     ocp.solver_options.qp_solver_tol_eq = 1e-3
#     ocp.solver_options.qp_solver_tol_ineq = 1e-3
#     ocp.solver_options.qp_solver_tol_comp = 1e-3
#     ocp.solver_options.nlp_solver_tol_stat = 1e-3
#     ocp.solver_options.nlp_solver_tol_eq = 1e-3
#     ocp.solver_options.nlp_solver_tol_ineq = 1e-3
#     ocp.solver_options.nlp_solver_tol_comp = 1e-3
#     ocp.solver_options.levenberg_marquardt = 1e-3

#     # ---------- State constraints (unchanged from your setup) ----------
#     max_rate = 1.0
#     ocp.constraints.lbx = np.array([0.05, -max_rate, -max_rate, -max_rate])
#     ocp.constraints.ubx = np.array([0.6,  max_rate,  max_rate,  max_rate])
#     ocp.constraints.idxbx = np.array([15, 13, 14, 16]) 

#     # Input bounds

#     ocp.constraints.lbu = np.array([-1.0, -1.0, -0.5, -1.0])  # [throttle_dot, roll_rate_dot, pitch_rate_dot, yaw_rate_dot]
#     ocp.constraints.ubu = np.array([ 1.0,  1.0,  0.5,  1.0])
#     ocp.constraints.idxbu = np.arange(nu)

#     # Create OCP solver
#     ocp_solver = AcadosOcpSolver(ocp)

#     # Create simulation configuration
#     sim = AcadosSim()
#     sim.model = ocp.model
#     sim.solver_options.T = 1.0 / 30.0
#     sim.parameter_values = np.array([38.0, 0.5, 0.12, 100.0, 100.0, 0.5, 1.0, 0.0, 0.0, 0.0])
#     sim_solver = AcadosSimSolver(sim)

#     return ocp_solver, sim_solver


# ------------------------
# Warm start convenience
# ------------------------
def set_initial_guess(ocp_solver, N_horizon=20):
    u_init = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

    for i in range(N_horizon):
        ocp_solver.set(i, "u", u_init)


def warm_start_from_previous_solution(ocp_solver, N_horizon=20):
    """Shift control warm start by one stage."""
    for i in range(N_horizon - 1):
        u_prev = ocp_solver.get(i + 1, "u")
        ocp_solver.set(i, "u", u_prev)
    u_last = ocp_solver.get(N_horizon - 1, "u")
    ocp_solver.set(N_horizon - 1, "u", u_last)


# -----------------------------------------------
# Reference handling: sign-continuous quaternion
# -----------------------------------------------
def _normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    return q if n == 0.0 else (q / n)


def _align_quat_to(prev_q: np.ndarray, q: np.ndarray) -> np.ndarray:
    qn = _normalize(q)
    return qn if float(np.dot(prev_q, qn)) >= 0.0 else -qn


def _make_quat_sequence_continuous(q_list):
    out = []
    if not q_list:
        return out
    out.append(_normalize(q_list[0]))
    for k in range(1, len(q_list)):
        out.append(_align_quat_to(out[-1], q_list[k]))
    return np.array(out)

def update_payload_ocp_parameters(
    ocp_solver, est_params, N_horizon, lateral_disturbance_xy=None
):
    """
    Update payload-aware dynamic parameters for all stages.

    est_params:
        [thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg,
         max_rate_deg, rate_expo, cable_length]
    """
    dyn_par = np.array(est_params, dtype=float)
    default_qref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    default_xy_ref = np.zeros(2, dtype=float)
    disturbance_xy = (
        np.zeros(2, dtype=float)
        if lateral_disturbance_xy is None
        else np.asarray(lateral_disturbance_xy, dtype=float).reshape(2)
    )

    for j in range(N_horizon):
        ocp_solver.set(
            j,
            "p",
            np.concatenate([dyn_par, default_qref, default_xy_ref, disturbance_xy])
        )

    ocp_solver.set(
        N_horizon,
        "p",
        np.concatenate([dyn_par, default_qref, default_xy_ref, disturbance_xy])
    )

# def update_ocp_parameters(ocp_solver, est_params, N_horizon):
#     """
#     Update the dynamic parameters for all stages in the OCP solver.
#     """
#     dyn_par = np.array(est_params, dtype=float)
#     default_qref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    
#     # Update parameters for all intermediate stages
#     for j in range(N_horizon):
#         full_params = np.concatenate([dyn_par, default_qref])
#         ocp_solver.set(j, "p", full_params)
    
#     # Update terminal stage parameters
#     ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, default_qref]))


def set_payload_trajectory_reference_aligned(
    ocp_solver,
    traj_states: np.ndarray,
    N_horizon: int,
    step_counter: int,
    skip_steps: int,
    est_params=None,
    *,
    use_acceleration_feedforward: bool = False,
    lateral_disturbance_xy=None,
):
    """
    Payload-aware reference setter.

    Cost layout:
    yref:
        [pos(3), vel(3), omega(3), u_state(4), u_dot(4),
         e_att(3), payload(4), xy_integral(2)]
        total 26

    yref_e:
        [pos(3), vel(3), omega(3), u_state(4), e_att(3),
         payload(4), xy_integral(2)]
        total 22
    """

    horizon_indices = [step_counter + j * skip_steps for j in range(N_horizon)]
    terminal_index = step_counter + N_horizon * skip_steps
    all_indices = horizon_indices + [terminal_index]

    dyn_par = np.array(est_params, dtype=float)
    disturbance_xy = (
        np.zeros(2, dtype=float)
        if lateral_disturbance_xy is None
        else np.asarray(lateral_disturbance_xy, dtype=float).reshape(2)
    )
    if not np.all(np.isfinite(disturbance_xy)):
        raise ValueError(
            f'lateral_disturbance_xy must be finite, got {disturbance_xy}'
        )

    # External C++ trajectories carry a dynamically meaningful translational
    # acceleration in rows 10:13.  For that path only, map a_d + g*e3 into the
    # nominal thrust direction and collective so the attitude/throttle reference
    # is consistent with the requested motion.  The supplied quaternion remains
    # authoritative for yaw/heading.  Internal legacy trajectories retain their
    # historical attitude/control-state references unless explicitly opted in.
    feedforward_throttles = None
    if use_acceleration_feedforward:
        feedforward_quaternions = []
        feedforward_throttles = []
        for idx in all_indices:
            throttle_ff, q_ff = acceleration_feedforward_reference(
                desired_acceleration=traj_states[10:13, idx],
                thrust_ratio=dyn_par[0],
                heading_quaternion=traj_states[3:7, idx],
            )
            feedforward_throttles.append(throttle_ff)
            feedforward_quaternions.append(q_ff)
        qs_cont = _make_quat_sequence_continuous(feedforward_quaternions)
    else:
        qs_raw = [traj_states[3:7, idx].copy() for idx in all_indices]
        qs_cont = _make_quat_sequence_continuous(qs_raw)

    for j, sc in enumerate(horizon_indices):
        yref = np.zeros((26,), dtype=float)

        # Position reference
        yref[0:3] = traj_states[0:3, sc]

        # Velocity reference
        yref[3:6] = traj_states[7:10, sc]

        # Body-rate reference
        # Existing trajectory format currently places accel in rows 10:13.
        # For now, keep this as zero unless you explicitly generate omega refs.
        yref[6:9] = np.array([0.0, 0.0, 0.0])

        # u_state reference.  Roll/pitch/yaw channel states remain zero as before;
        # attitude is represented by q_ref.  External acceleration feedforward
        # supplies only the nominal collective-throttle state, matching Wesley's
        # proven tracker structure without changing OCP weights or dynamics.
        yref[9:13] = np.array([0.0, 0.0, 0.0, 0.0])
        if feedforward_throttles is not None:
            yref[11] = feedforward_throttles[j]

        # u_dot reference
        yref[13:17] = np.array([0.0, 0.0, 0.0, 0.0])

        # attitude error reference is implicitly zero
        yref[17:20] = np.array([0.0, 0.0, 0.0])

        # payload reference: zero swing
        yref[20:24] = np.array([0.0, 0.0, 0.0, 0.0])

        # Integral-state reference is zero: persistent XY tracking error should
        # be removed rather than accepted as a steady equilibrium.
        yref[24:26] = np.array([0.0, 0.0])

        ocp_solver.set(j, "yref", yref)

        qref = qs_cont[j]
        xy_ref = traj_states[0:2, sc]
        ocp_solver.set(
            j, "p",
            np.concatenate([dyn_par, qref, xy_ref, disturbance_xy]),
        )

    yref_N = np.zeros((22,), dtype=float)

    yref_N[0:3] = traj_states[0:3, terminal_index]
    yref_N[3:6] = traj_states[7:10, terminal_index]
    yref_N[6:9] = np.array([0.0, 0.0, 0.0])
    yref_N[9:13] = np.array([0.0, 0.0, 0.0, 0.0])
    if feedforward_throttles is not None:
        yref_N[11] = feedforward_throttles[-1]
    yref_N[13:16] = np.array([0.0, 0.0, 0.0])
    yref_N[16:20] = np.array([0.0, 0.0, 0.0, 0.0])
    yref_N[20:22] = np.array([0.0, 0.0])

    ocp_solver.set(N_horizon, "yref", yref_N)
    terminal_xy_ref = traj_states[0:2, terminal_index]
    ocp_solver.set(
        N_horizon,
        "p",
        np.concatenate([dyn_par, qs_cont[-1], terminal_xy_ref, disturbance_xy])
    )

# def set_trajectory_reference_aligned(ocp_solver, traj_states: np.ndarray, N_horizon: int, step_counter: int, skip_steps: int, est_params=None):

#     horizon_indices = [step_counter + j * skip_steps for j in range(N_horizon)]
#     terminal_index = step_counter + N_horizon * skip_steps
#     all_indices = horizon_indices + [terminal_index]

#     qs_raw = [traj_states[3:7, idx].copy() for idx in all_indices]
#     qs_cont = _make_quat_sequence_continuous(qs_raw)
#     dyn_par = np.array(est_params, dtype=float)

#     for j, sc in enumerate(horizon_indices):
#         # 1) yref
#         yref = np.zeros((20,), dtype=float)
#         yref[0:3] = traj_states[0:3, sc]
#         yref[3:6]  = traj_states[7:10, sc]
#         yref[6:9]  = traj_states[10:13, sc]
#         yref[13:17]= [0.0, 0.0, 0.0, 0.0]
#         ocp_solver.set(j, "yref", yref)

#         # 2) parameters: [dyn(6), q_ref(4)]
#         qref = qs_cont[j]
#         ocp_solver.set(j, "p", np.concatenate([dyn_par, qref]))

#     yref_N = np.zeros((16,), dtype=float)
#     yref_N[0:3] = traj_states[0:3, terminal_index]
#     ocp_solver.set(N_horizon, "yref", yref_N)
#     ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, qs_cont[-1]]))


