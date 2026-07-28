import os
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics, QuadPayloadDynamics
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver



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
def generate_payload_ocp_controller():
    quad_dynamics = QuadPayloadDynamics()
    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_payload_dynamics'

    model.x = quad_dynamics.x
    model.u = quad_dynamics.u_dot

    # Payload dynamics params:
    # [thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg,
    #  max_rate_deg, rate_expo, cable_length]
    p_dyn = quad_dynamics.p_param

    # Quaternion reference parameter: [qw, qx, qy, qz]
    p_qref = ca.MX.sym('p_qref', 4)

    model.p = ca.vertcat(p_dyn, p_qref)

    # Payload dynamics uses first 7 parameters
    model.f_expl_expr = dynamics_expr(model.x, model.u, model.p[:7])

    xdot = ca.MX.sym('xdot', model.x.size()[0])
    model.xdot = xdot
    model.f_impl_expr = model.f_expl_expr - xdot

    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 2.0

    nu = 4
    nx = 21

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

    # Layout:
    # [pos(3), vel(3), omega(3), u_state(4), u_dot(4), e_att(3), payload(4)]
    # total ny = 24
    y_expr = ca.vertcat(
        x[0:3],
        x[7:10],
        x[10:13],
        x[13:17],
        u,
        e_att,
        payload_state
    )

    # Terminal:
    # [pos(3), vel(3), omega(3), u_state(4), e_att(3), payload(4)]
    # total ny_e = 20
    y_expr_e = ca.vertcat(
        x[0:3],
        x[7:10],
        x[10:13],
        x[13:17],
        e_att,
        payload_state
    )

    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ny = 3 + 3 + 3 + 4 + 4 + 3 + 4
    ny_e = 3 + 3 + 3 + 4 + 3 + 4

    W = np.diag([
        160.0, 160.0, 70.0,          # position
        2.0, 2.0, 2.0,             # velocity
        0.2, 0.2, 0.2,             # body rates
        2e-4, 2e-4, 2e-4, 2e-4,    # control state
        0.1, 0.1, 5.0, 0.1,        # input rate
        0.5, 0.5, 5.0,             # attitude error

        20.0, 20.0,                # phi, theta
        2.0, 2.0                   # phi_dot, theta_dot
    ])

    W_e = np.diag([
        250.0, 250.0, 80.0,          # position
        2.0, 2.0, 2.0,             # velocity
        0.2, 0.2, 0.2,             # body rates
        2e-4, 2e-4, 2e-4, 2e-4,    # control state
        0.5, 0.5, 5.0,             # attitude error

        20.0, 20.0,                # phi, theta
        2.0, 2.0                   # phi_dot, theta_dot
    ])

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
    #  q_ref_w, q_ref_x, q_ref_y, q_ref_z]
    ocp.parameter_values = np.array([
        38.0, 0.5, 0.07, 100.0, 100.0, 0.5,
        0.533,      # cable length taken from model thing
        1.0, 0.0, 0.0, 0.0
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
    # 13:17 control state, 17:21 payload state
    max_rate = 1.0

    ocp.constraints.lbx = np.array([0.05, -max_rate, -max_rate, -max_rate])
    ocp.constraints.ubx = np.array([0.6,  max_rate,  max_rate,  max_rate])
    ocp.constraints.idxbx = np.array([15, 13, 14, 16])

    # Input bounds:
    # u_dot = [roll_dot, pitch_dot, throttle_dot, yaw_dot]
    ocp.constraints.lbu = np.array([-1.0, -1.0, -0.5, -1.0])
    ocp.constraints.ubu = np.array([ 1.0,  1.0,  0.5,  1.0])
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver. Give it its OWN export dir + json so its generated C code and
    # solver artifacts never collide with the other acados solvers in this workspace (the
    # load planner and the per-drone trackers) -- a shared default 'c_generated_code' /
    # 'acados_ocp.json' can otherwise clobber another solver at launch.
    ocp.code_export_directory = export_dir = 'c_generated_code_approach_mpc'

    # Reuse the already-generated + compiled solver instead of regenerating C and recompiling
    # on every launch (that ~30-60 s codegen/make was running each start). We only build when
    # the compiled .so is missing or a rebuild is explicitly forced via APPROACH_MPC_REBUILD=1
    # -- do that after changing the model/cost/constraints/horizon in this file.
    ocp_so = os.path.join(export_dir, f'libacados_ocp_solver_{ocp.model.name}.so')
    sim_so = os.path.join(export_dir, f'libacados_sim_solver_{ocp.model.name}.so')
    force_rebuild = os.environ.get('APPROACH_MPC_REBUILD', '0').lower() in ('1', 'true', 'yes')
    # Decide OCP and SIM builds independently -- their .so files are generated separately, so a
    # cached OCP does not imply a cached SIM (and vice versa).
    need_ocp_build = force_rebuild or not os.path.exists(ocp_so)
    need_sim_build = force_rebuild or not os.path.exists(sim_so)
    print(f'[approach_mpc] acados OCP: {"BUILD" if need_ocp_build else "reuse cached"}; '
          f'SIM: {"BUILD" if need_sim_build else "reuse cached"} (force_rebuild={force_rebuild})')
    ocp_solver = AcadosOcpSolver(
        ocp, json_file='acados_ocp_approach_mpc.json',
        build=need_ocp_build, generate=need_ocp_build)

    # Create simulation integrator (shares the same export dir; only build it when we rebuild).
    sim = AcadosSim()
    sim.model = ocp.model
    sim.code_export_directory = export_dir
    sim.solver_options.T = 1.0 / 30.0

    sim.parameter_values = np.array([
        38.0, 0.5, 0.12, 100.0, 100.0, 0.5,
        0.50,
        1.0, 0.0, 0.0, 0.0
    ])

    sim_solver = AcadosSimSolver(sim, build=need_sim_build, generate=need_sim_build)

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

def update_payload_ocp_parameters(ocp_solver, est_params, N_horizon):
    """
    Update payload-aware dynamic parameters for all stages.

    est_params:
        [thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg,
         max_rate_deg, rate_expo, cable_length]
    """
    dyn_par = np.array(est_params, dtype=float)
    default_qref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    for j in range(N_horizon):
        ocp_solver.set(j, "p", np.concatenate([dyn_par, default_qref]))

    ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, default_qref]))

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
    est_params=None
):
    """
    Payload-aware reference setter.

    Cost layout:
    yref:
        [pos(3), vel(3), omega(3), u_state(4), u_dot(4), e_att(3), payload(4)]
        total 24

    yref_e:
        [pos(3), vel(3), omega(3), u_state(4), e_att(3), payload(4)]
        total 20
    """

    horizon_indices = [step_counter + j * skip_steps for j in range(N_horizon)]
    terminal_index = step_counter + N_horizon * skip_steps
    all_indices = horizon_indices + [terminal_index]

    qs_raw = [traj_states[3:7, idx].copy() for idx in all_indices]
    qs_cont = _make_quat_sequence_continuous(qs_raw)

    dyn_par = np.array(est_params, dtype=float)

    for j, sc in enumerate(horizon_indices):
        yref = np.zeros((24,), dtype=float)

        # Position reference
        yref[0:3] = traj_states[0:3, sc]

        # Velocity reference
        yref[3:6] = traj_states[7:10, sc]

        # Body-rate reference
        # Existing trajectory format currently places accel in rows 10:13.
        # For now, keep this as zero unless you explicitly generate omega refs.
        yref[6:9] = np.array([0.0, 0.0, 0.0])

        # u_state reference
        yref[9:13] = np.array([0.0, 0.0, 0.0, 0.0])

        # u_dot reference
        yref[13:17] = np.array([0.0, 0.0, 0.0, 0.0])

        # attitude error reference is implicitly zero
        yref[17:20] = np.array([0.0, 0.0, 0.0])

        # payload reference: zero swing
        yref[20:24] = np.array([0.0, 0.0, 0.0, 0.0])

        ocp_solver.set(j, "yref", yref)

        qref = qs_cont[j]
        ocp_solver.set(j, "p", np.concatenate([dyn_par, qref]))

    yref_N = np.zeros((20,), dtype=float)

    yref_N[0:3] = traj_states[0:3, terminal_index]
    yref_N[3:6] = traj_states[7:10, terminal_index]
    yref_N[6:9] = np.array([0.0, 0.0, 0.0])
    yref_N[9:13] = np.array([0.0, 0.0, 0.0, 0.0])
    yref_N[13:16] = np.array([0.0, 0.0, 0.0])
    yref_N[16:20] = np.array([0.0, 0.0, 0.0, 0.0])

    ocp_solver.set(N_horizon, "yref", yref_N)
    ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, qs_cont[-1]]))

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


