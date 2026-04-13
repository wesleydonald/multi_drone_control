import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver


# ---------- Quaternion helpers (step 1 band-aid) ----------
def _align_quat_sign(q_ref: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
    """
    Force q_ref (wxyz) to the same hemisphere as q_cur (wxyz).
    Both are 1D arrays of shape (4,).
    """
    return q_ref if float(np.dot(q_ref, q_cur)) >= 0.0 else -q_ref


def _norm_quat(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    return q if n == 0.0 else (q / n)


def generate_ocp_controller(dynamics=None, N_horizon: int = 30, T_horizon: float = 3.0):
    # Define the dynamics model
    quad_dynamics = QuadDynamics() if dynamics is None else dynamics
    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = quad_dynamics.x  # [p(3), q(4=wxyz), v(3), r(3), actuators(8), u_desired(8)] -> 29 states
    model.u = quad_dynamics.u_dot  # control input is now u_dot (8 inputs)
    model.p = quad_dynamics.params  # adaptive parameters [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant, ixx, iyy, izz] -> 8 parameters
    model.f_expl_expr = dynamics_expr(quad_dynamics.x, quad_dynamics.u_dot, quad_dynamics.params)

    # Create OCP object
    ocp = AcadosOcp()
    ocp.model = model

    # --- Properly set horizon dims/options ---
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = T_horizon

    # dims
    nx = 29  # Updated from 21 to 29 (13 + 8 actual actuators + 8 desired actuators)
    nu = int(model.u.size()[0])  # should be 8 for your setup
    np_param = int(model.p.size()[0])  # should be 8 for adaptive parameters [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant, ixx, iyy, izz]
    ny = nx  # we will NOT penalize inputs in LINEAR_LS (no R term)

    # ---------- Cost (LINEAR_LS on states only) ----------
    # Base per-state weights (tune as you like)
    # [px,py,pz, qw,qx,qy,qz, vx,vy,vz, rx,ry,rz, actual_actuators(8), desired_actuators(8)]
    q_cost = np.array([
        2.1, 2.1, 2.1,      # position
        6.1, 6.1, 6.1, 6.1, # quaternion (we'll apply norm-weighting below)
        0.5, 0.5, 0.5,      # velocity
        0.5, 0.5, 0.5,      # body rates
        0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001,  # actual actuator states (small weight)
        0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001   # desired actuator states (medium weight)
    ])

    # ---- "Quaternion norm weighting" (reduce axis bias) ----
    # Use a single shared weight for the quaternion *vector* part (x,y,z).
    # Keep qw's own weight as is.
    qw_w = q_cost[3]
    qv_mean = float(np.mean(q_cost[4:7]))
    q_weights = np.concatenate([
        q_cost[:4], 
        np.array([qv_mean, qv_mean, qv_mean]), 
        q_cost[7:13],  # velocity and body rates
        q_cost[13:21], # actual actuator states
        q_cost[21:]    # desired actuator states
    ])

    # Build W and W_e for LINEAR_LS with y = Vx x - yref (no inputs in y)
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.W = np.diag(q_weights)        # (ny x ny) with ny = nx
    ocp.cost.W_e = np.diag(q_weights)      # terminal state cost

    ocp.cost.Vx = np.eye(nx)               # y = x - yref
    ocp.cost.Vu = np.zeros((ny, nu))       # no input in cost
    ocp.cost.Vx_e = np.eye(nx)

    # Initial references (will be set each step by your code)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # ---------- Initial state constraint ----------
    x0 = np.zeros(nx)
    x0[3] = 1.0  # unit quaternion, w=1
    # Initialize both actual and desired actuator states to hover values
    hover_values = np.array([-0.07, 0.07, -0.07, 0.07, 0.07, -0.07, 0.07, -0.07])
    x0[13:21] = hover_values  # actual actuator states start at hover
    x0[21:29] = hover_values  # desired actuator states start at hover
    ocp.constraints.x0 = x0

    # ---------- Initial parameter values (adaptive estimator parameters) ----------
    p0 = np.array([0.0, 0.0, 0.0, 7.42678162, 0.08])  # [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant]
    ocp.parameter_values = p0

    # ---------- Input constraints ----------
    max_rate = 0.1 # Maximum rate of change for actuator values
    ocp.constraints.lbu = np.full((nu,), -max_rate)
    ocp.constraints.ubu = np.full((nu,),  max_rate)
    ocp.constraints.idxbu = np.arange(nu, dtype=int)

    # ---------- State constraints (for actuator values) ----------
    # Constrain both actual and desired actuator states to be within reasonable bounds
    actuator_min = -0.8
    actuator_max = 0.8
    
    # State bounds for all shooting nodes (not initial)
    # Use large finite values instead of inf to avoid JSON serialization issues
    large_val = 1e8
    lbx = np.full(nx, -large_val)
    ubx = np.full(nx, large_val)
    lbx[13:21] = actuator_min  # actual actuator states lower bounds
    ubx[13:21] = actuator_max  # actual actuator states upper bounds
    lbx[21:29] = actuator_min  # desired actuator states lower bounds
    ubx[21:29] = actuator_max  # desired actuator states upper bounds
    
    # Apply state constraints to all shooting nodes except the first (which is constrained to current state)
    ocp.constraints.lbx = lbx
    ocp.constraints.ubx = ubx
    ocp.constraints.idxbx = np.arange(nx, dtype=int)

    # ---------- Solver options ----------
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'

    # Iterations / tolerances
    ocp.solver_options.nlp_solver_max_iter = 1000
    ocp.solver_options.qp_solver_iter_max = 500

    ocp.solver_options.qp_solver_tol_stat = 5e-4
    ocp.solver_options.qp_solver_tol_eq = 5e-4
    ocp.solver_options.qp_solver_tol_ineq = 5e-4
    ocp.solver_options.qp_solver_tol_comp = 5e-4
    ocp.solver_options.nlp_solver_tol_stat = 5e-4
    ocp.solver_options.nlp_solver_tol_eq = 5e-4
    ocp.solver_options.nlp_solver_tol_ineq = 5e-4
    ocp.solver_options.nlp_solver_tol_comp = 5e-4

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)

    # Create simulation configuration (note: 0.01 s -> 100 Hz)
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = 1.0 / 30.0
    sim.parameter_values = p0  # Set initial parameter values for simulator
    sim_solver = AcadosSimSolver(sim)

    return ocp_solver, sim_solver


def calculate_hover_initial_guess():
    """
    Initial guess for control inputs (actuator rates) - start with zero rates
    Returns:
        u_hover: (nu,) Initial guess for control inputs (all zero rates)
    """
    # Since we're controlling rates now, start with zero rates (no change)
    return np.zeros(8)


def set_initial_guess(ocp_solver, N_horizon=20):
    """
    Set initial guess for the MPC solver based on hover solution
    """
    u_hover = calculate_hover_initial_guess()
    for i in range(N_horizon):
        ocp_solver.set(i, "u", u_hover)


def warm_start_from_previous_solution(ocp_solver, N_horizon=20):
    """
    Warm start the MPC solver using the previous solution shifted by one time step
    """
    for i in range(N_horizon - 1):
        u_prev = ocp_solver.get(i + 1, "u")
        ocp_solver.set(i, "u", u_prev)
    u_last = ocp_solver.get(N_horizon - 1, "u")
    ocp_solver.set(N_horizon - 1, "u", u_last)


def set_adaptive_parameters(ocp_solver, sim_solver, theta_params: np.ndarray, N_horizon: int):
    """
    Set the adaptive estimator parameters for both OCP solver and simulator.
    
    Args:
        ocp_solver: Acados OCP solver
        sim_solver: Acados simulation solver  
        theta_params: np.ndarray of shape (8,) containing [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant, ixx, iyy, izz]
        N_horizon: Number of shooting nodes in the horizon
    """
    theta_params = np.array(theta_params, dtype=float)
    
    # Set parameters for all shooting nodes in OCP solver
    for i in range(N_horizon + 1):  # Include terminal node
        ocp_solver.set(i, "p", theta_params)
    
    # Set parameters for the simulator
    sim_solver.set("p", theta_params)


# ---------- NEW: helpers to set yref with quaternion sign-alignment ----------
def set_state_reference_aligned(ocp_solver, x_ref: np.ndarray, N_horizon: int):
    """
    Set the same state reference at all stages, aligning quaternion sign per stage to the
    current solver state (hemisphere-invariant LINEAR_LS).
    x_ref shape: (29,) with quaternion at indices 3:7 in (w,x,y,z).
    """
    x_ref = np.array(x_ref, dtype=float).copy()
    x_ref[3:7] = _norm_quat(x_ref[3:7])

    # Stage-wise set
    for j in range(N_horizon):
        try:
            x_cur = ocp_solver.get(j, "x")
            q_cur = np.array(x_cur[3:7], dtype=float)
        except Exception:
            # if not available, fall back to the (aligned) reference itself
            q_cur = x_ref[3:7]

        q_ref_aligned = _align_quat_sign(x_ref[3:7], q_cur)
        yref = x_ref.copy()
        yref[3:7] = q_ref_aligned
        ocp_solver.set(j, "yref", yref)

    # Terminal node
    try:
        x_curN = ocp_solver.get(N_horizon, "x")
        q_curN = np.array(x_curN[3:7], dtype=float)
    except Exception:
        q_curN = x_ref[3:7]
    yref_e = x_ref.copy()
    yref_e[3:7] = _align_quat_sign(x_ref[3:7], q_curN)
    ocp_solver.set(N_horizon, "yref", yref_e)


def set_trajectory_reference_aligned(ocp_solver, X_ref: np.ndarray):
    """
    Set a *time-varying* state reference, aligning the quaternion sign per stage.
    X_ref shape: (N_horizon+1, 29). Row j is the reference at stage j. Last row is terminal.
    """
    N_horizon = X_ref.shape[0] - 1
    X_ref = np.array(X_ref, dtype=float).copy()

    # Normalise all quats once
    for j in range(N_horizon + 1):
        X_ref[j, 3:7] = _norm_quat(X_ref[j, 3:7])

    for j in range(N_horizon):
        try:
            x_cur = ocp_solver.get(j, "x")
            q_cur = np.array(x_cur[3:7], dtype=float)
        except Exception:
            q_cur = X_ref[j, 3:7]

        xj = X_ref[j, :].copy()
        xj[3:7] = _align_quat_sign(xj[3:7], q_cur)
        ocp_solver.set(j, "yref", xj)

    # Terminal
    try:
        x_curN = ocp_solver.get(N_horizon, "x")
        q_curN = np.array(x_curN[3:7], dtype=float)
    except Exception:
        q_curN = X_ref[N_horizon, 3:7]
    xN = X_ref[N_horizon, :].copy()
    xN[3:7] = _align_quat_sign(xN[3:7], q_curN)
    ocp_solver.set(N_horizon, "yref", xN)
