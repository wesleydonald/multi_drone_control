"""
planner_ocp.py
--------------
acados OCP for the centralized cable-suspended load planner tudelft. 
Wraps the LoadCableDynamics model.

Cost (Eq 6): NONLINEAR_LS tracking the LOAD pose/twist (position, sign-invariant
quaternion attitude error, velocity, angular velocity), a tension regulariser
(keep cables taut near a nominal), and control effort (gamma, lambda).

Path constraints (softened with slack where nonlinear):
  - thrust:    thrust_min <= ||m_i(v_dot_i - g) - t_i s_i|| <= thrust_max  (Eqs 8-9)
  - tautness:  tension_min <= t_i <= tension_max                          (Eq 10)
  - input box: |gamma_i| <= gamma_max, |lambda_i| <= lambda_max
Collision (Eq 11) / no-fly (Eq 12) are deferred until this converges.

The load reference attitude q_ref is an acados parameter (set per solve by the
node); the reference position/velocity/angular-velocity/tension are set via yref.
"""
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

from .load_cable_dynamics import LoadCableDynamics, quat_mul, LOAD_DIM, CABLE_DIM

GRAV = 9.81

def quat_conj(q):
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])


def nominal_hover_state(dyn: LoadCableDynamics, load_pos=(0.0, 0.0, 1.0),
                        elevation_deg=45.0):
    """A self-consistent taut-hover state: drones up-and-out from their attach
    points at `elevation_deg` above horizontal, cables taut, tensions balancing
    the load weight. Used as the construction default and a warm start.
    """
    phi = np.deg2rad(elevation_deg)
    x = np.zeros(dyn.nx)
    x[0:3] = load_pos
    x[6] = 1.0  # q = identity
    t_nom = dyn.m * GRAV / (dyn.n * np.sin(phi))
    for i in range(dyn.n):
        rho = dyn.rho[i]
        th = np.arctan2(rho[1], rho[0])          # attachment azimuth
        # s_i (quad->load) points down-and-inward
        s = np.array([-np.cos(th) * np.cos(phi),
                      -np.sin(th) * np.cos(phi),
                      -np.sin(phi)])
        b = LOAD_DIM + CABLE_DIM * i
        x[b:b + 3] = s                            # s_i (unit)
        x[b + 12] = t_nom                         # t_i
    return x


def generate_load_ocp(dyn: LoadCableDynamics, N=20, tf=2.0,
                      thrust_min=0.5, thrust_max=15.0,
                      tension_min=0.1, tension_max=25.0,
                      gamma_max=50.0, lambda_max=80.0,
                      build=True):
    n = dyn.n

    # ── model ──────────────────────────────────────────────────────────────
    model = AcadosModel()
    model.name = f'load_cable_{n}'
    model.x = dyn.x
    model.u = dyn.u
    q_ref = ca.MX.sym('q_ref', 4)                 # load reference attitude (param)
    model.p = q_ref
    model.f_expl_expr = dyn.f_expl
    xdot = ca.MX.sym('xdot', dyn.nx)
    model.xdot = xdot
    model.f_impl_expr = dyn.f_expl - xdot

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = tf

    # ── cost: NONLINEAR_LS ─────────────────────────────────────────────────
    q_err = quat_mul(q_ref, quat_conj(dyn.q))
    e_att = 2 * q_err[1:4]                         # sign-invariant attitude error
    t_vec = ca.vertcat(*[dyn.t[i] for i in range(n)])
    r_vec = ca.vertcat(*[dyn.r[i] for i in range(n)])   # cable angular velocities
    y = ca.vertcat(dyn.p, dyn.v, e_att, dyn.w, t_vec, r_vec, model.u)
    y_e = ca.vertcat(dyn.p, dyn.v, e_att, dyn.w, t_vec, r_vec)
    ocp.model.cost_y_expr = y
    ocp.model.cost_y_expr_e = y_e
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # pos(3), vel(3), att(3), omega(3). Velocity weight raised from [4,4,4] to
    # [18,18,22]: the original let the load/drones build climb speed and overshoot
    # the slack->taut transition, spiking cable tension past the drones' thrust
    # authority. Heavier velocity damping keeps the lift slow and bounded.
    w_pose = [60., 60., 80.] + [18., 18., 22.] + [30., 30., 30.] + [1., 1., 1.]
    w_t = [0.05] * n
    w_r = [3.0] * (3 * n)                          # damp cable swing (key)
    w_u = []
    for _ in range(n):
        w_u += [1e-3, 1e-3, 1e-3, 5e-3]           # gamma(3), lambda
    W = np.diag(w_pose + w_t + w_r + w_u)
    W_e = np.diag(w_pose + w_t + w_r)
    ocp.cost.W = W
    ocp.cost.W_e = W_e
    ocp.cost.yref = np.zeros(W.shape[0])
    ocp.cost.yref_e = np.zeros(W_e.shape[0])

    # ── initial condition + parameter defaults ─────────────────────────────
    ocp.constraints.x0 = nominal_hover_state(dyn)
    ocp.parameter_values = np.array([1.0, 0.0, 0.0, 0.0])   # q_ref = identity

    # ── tautness: state bounds on each t_i ─────────────────────────────────
    t_idx = [LOAD_DIM + CABLE_DIM * i + 12 for i in range(n)]
    ocp.constraints.idxbx = np.array(t_idx)
    ocp.constraints.lbx = np.array([tension_min] * n)
    ocp.constraints.ubx = np.array([tension_max] * n)

    # ── input box bounds ───────────────────────────────────────────────────
    lbu, ubu = [], []
    for _ in range(n):
        lbu += [-gamma_max] * 3 + [-lambda_max]
        ubu += [gamma_max] * 3 + [lambda_max]
    ocp.constraints.lbu = np.array(lbu)
    ocp.constraints.ubu = np.array(ubu)
    ocp.constraints.idxbu = np.arange(dyn.nu)

    # ── thrust: soft nonlinear constraint ──────────────────────────────────
    ocp.model.con_h_expr = ca.vertcat(*[dyn.thrust(i) for i in range(n)])
    ocp.constraints.lh = np.array([thrust_min] * n)
    ocp.constraints.uh = np.array([thrust_max] * n)
    ocp.constraints.idxsh = np.arange(n)
    ocp.cost.zl = 1e2 * np.ones(n)
    ocp.cost.zu = 1e2 * np.ones(n)
    ocp.cost.Zl = 1e1 * np.ones(n)
    ocp.cost.Zu = 1e1 * np.ones(n)

    # ── solver options ─────────────────────────────────────────────────────
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.qp_solver_iter_max = 200
    ocp.solver_options.levenberg_marquardt = 1e-2   # more regularisation

    # isolate codegen so it never clashes with controller_mpc_multi's
    # c_generated_code/ (different model, shared dir would race on the Makefile)
    ocp.code_export_directory = 'c_generated_code_load_planner'

    solver = (AcadosOcpSolver(ocp, json_file=f'{model.name}_ocp.json')
              if build else None)
    return ocp, solver


if __name__ == "__main__":
    # build + smoke-solve for the three_soft geometry (3 drones, 120deg)
    n = 3
    rho = [[0.08 * np.cos(2 * np.pi * k / 3), 0.08 * np.sin(2 * np.pi * k / 3), 0.025]
           for k in range(n)]
    dyn = LoadCableDynamics(n, load_mass=0.4, load_inertia=[1.67e-3, 1.67e-3, 3.33e-3],
                            cable_lengths=[0.42] * n, attach_points=rho, drone_mass=0.6)
    ocp, solver = generate_load_ocp(dyn)
    print(f"OCP built: nx={dyn.nx} nu={dyn.nu} N={ocp.solver_options.N_horizon}")

    # hover at z=1.0: x_init = nominal hover, yref = same load pose, zero twist
    x_init = nominal_hover_state(dyn, load_pos=(0.0, 0.0, 1.0))
    t_nom = x_init[LOAD_DIM + 12]
    yref = np.concatenate([[0, 0, 1.0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                           [t_nom] * n, np.zeros(3 * n), np.zeros(dyn.nu)])
    yref_e = yref[:-dyn.nu]
    for k in range(ocp.solver_options.N_horizon):
        solver.set(k, "yref", yref)
        solver.set(k, "p", np.array([1.0, 0.0, 0.0, 0.0]))
        solver.set(k, "x", x_init)
    solver.set(ocp.solver_options.N_horizon, "yref", yref_e)
    solver.set(ocp.solver_options.N_horizon, "p", np.array([1.0, 0.0, 0.0, 0.0]))
    solver.set(ocp.solver_options.N_horizon, "x", x_init)
    solver.set(0, "lbx", x_init)
    solver.set(0, "ubx", x_init)

    status = 0
    for _ in range(20):              # a few RTI iterations to converge the hover
        status = solver.solve()
    print("solve status:", status, "(0 = success)")
    x1 = solver.get(1, "x")
    print("tensions:", np.round([x1[LOAD_DIM + CABLE_DIM * i + 12] for i in range(n)], 3))
    import casadi as _ca
    Tfun = _ca.Function('T', [dyn.x], [ca.vertcat(*[dyn.thrust(i) for i in range(n)])])
    print("thrusts :", np.round(np.array(Tfun(x1)).flatten(), 3), "N")
