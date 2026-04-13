import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics
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
def generate_ocp_controller(dynamics=None):
    if dynamics is None:
        quad_dynamics = QuadDynamics()
    else:
        quad_dynamics = dynamics

    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = quad_dynamics.x 
    model.u = quad_dynamics.u_dot
    p_dyn = quad_dynamics.p_param 
    p_qref = ca.MX.sym('p_qref', 4)           
    model.p = ca.vertcat(p_dyn, p_qref) 

    model.f_expl_expr = dynamics_expr(model.x, model.u, model.p[:6])

    xdot = ca.MX.sym('xdot', model.x.size()[0])
    f_impl_expr = model.f_expl_expr - xdot
    model.xdot = xdot
    model.f_impl_expr = f_impl_expr

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 2.0

    nu = 4
    nx = 17

    # ---------- COST: NONLINEAR_LS with sign-invariant quaternion error ----------
    q_idx = 3
    x = model.x
    u = model.u
    q = x[q_idx:q_idx+4]
    q_ref = model.p[6:10]

    q_err = quat_mul(q_ref, quat_conj(q))
    e_att = 2 * q_err[1:4]        

    # Layout: [pos(3), vel(3), omega(3), u_state(4), u(4), e_att(3)] -> total ny = 20
    y_expr   = ca.vertcat(x[0:3], x[7:10], x[10:13], x[13:17], u, e_att)
    y_expr_e = ca.vertcat(x[0:3], x[7:10], x[10:13], x[13:17], e_att) 

    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ny = 3 + 3 + 3 + 4 + 4 + 3      
    ny_e = 3 + 3 + 3 + 4 + 3        

    W = np.diag([
        80.0, 80.0, 40.0,
        2.0, 2.0, 2.0,
        0.2, 0.2, 0.2,
        2e-4, 2e-4, 2e-4, 2e-4,
        0.1, 0.1, 5.0, 0.1,  # Reduced control effort penalty
        0.5, 0.5, 5.0
    ])
    W_e = np.diag([
        80.0, 80.0, 40.0,         # pos
        2.0, 2.0, 2.0,        # vel
        0.2, 0.2, 0.2,         # omega
        2e-4, 2e-4, 2e-4, 2e-4,# u_state
        0.5, 0.5, 5.0          # attitude error
    ])
    ocp.cost.W = W
    ocp.cost.W_e = W_e

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # Initial condition
    x0 = np.zeros(nx)
    ocp.constraints.x0 = x0

    # -------- Parameters default (6 dyn + 4 q_ref) --------
    ocp.parameter_values = np.array([38.0, 0.5, 0.07, 100.0, 100.0, 0.5, 1.0, 0.0, 0.0, 0.0])

    # ---------- Solver options ----------
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.nlp_solver_max_iter = 50  # Reduced for real-time performance with full SQP
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

    # ---------- State constraints (unchanged from your setup) ----------
    max_rate = 1.0
    ocp.constraints.lbx = np.array([0.05, -max_rate, -max_rate, -max_rate])
    ocp.constraints.ubx = np.array([0.6,  max_rate,  max_rate,  max_rate])
    ocp.constraints.idxbx = np.array([15, 13, 14, 16]) 

    # Input bounds

    ocp.constraints.lbu = np.array([-1.0, -1.0, -0.5, -1.0])  # [throttle_dot, roll_rate_dot, pitch_rate_dot, yaw_rate_dot]
    ocp.constraints.ubu = np.array([ 1.0,  1.0,  0.5,  1.0])
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)

    # Create simulation configuration
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = 1.0 / 30.0
    sim.parameter_values = np.array([38.0, 0.5, 0.12, 100.0, 100.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    sim_solver = AcadosSimSolver(sim)

    return ocp_solver, sim_solver


# ------------------------
# Warm start convenience
# ------------------------
def set_initial_guess(ocp_solver, N_horizon=20):
    u_init = np.array([0.0, 0.0, -1.0, 0.0], dtype=float)
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


def update_ocp_parameters(ocp_solver, est_params, N_horizon):
    """
    Update the dynamic parameters for all stages in the OCP solver.
    """
    dyn_par = np.array(est_params, dtype=float)
    default_qref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    
    # Update parameters for all intermediate stages
    for j in range(N_horizon):
        full_params = np.concatenate([dyn_par, default_qref])
        ocp_solver.set(j, "p", full_params)
    
    # Update terminal stage parameters
    ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, default_qref]))


def set_trajectory_reference_aligned(ocp_solver, traj_states: np.ndarray, N_horizon: int, step_counter: int, skip_steps: int, est_params=None):

    horizon_indices = [step_counter + j * skip_steps for j in range(N_horizon)]
    terminal_index = step_counter + N_horizon * skip_steps
    all_indices = horizon_indices + [terminal_index]

    qs_raw = [traj_states[3:7, idx].copy() for idx in all_indices]
    qs_cont = _make_quat_sequence_continuous(qs_raw)
    dyn_par = np.array(est_params, dtype=float)

    for j, sc in enumerate(horizon_indices):
        # 1) yref
        yref = np.zeros((20,), dtype=float)
        yref[0:3] = traj_states[0:3, sc]
        yref[3:6]  = traj_states[7:10, sc]
        yref[6:9]  = traj_states[10:13, sc]
        yref[13:17]= [0.0, 0.0, 0.0, 0.0]
        ocp_solver.set(j, "yref", yref)

        # 2) parameters: [dyn(6), q_ref(4)]
        qref = qs_cont[j]
        ocp_solver.set(j, "p", np.concatenate([dyn_par, qref]))

    yref_N = np.zeros((16,), dtype=float)
    yref_N[0:3] = traj_states[0:3, terminal_index]
    ocp_solver.set(N_horizon, "yref", yref_N)
    ocp_solver.set(N_horizon, "p", np.concatenate([dyn_par, qs_cont[-1]]))
