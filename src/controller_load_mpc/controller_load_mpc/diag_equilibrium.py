"""
diag_equilibrium.py
-------------------
OFFLINE planner sanity check (no Gazebo, no ROS). Builds the load-cable OCP for a
given world geometry, freezes the load at a hover setpoint, solves, and prints:

  * OCP solve status (0 == converged)
  * planned per-cable tension t_i vs the analytic taut-hover value
  * each drone REFERENCE position vs its physical spawn position (residual should
    be ~0 if CABLE_LEN / attach geometry match the world)
  * the thrust feedforward a_i -> tracker throttle and tilt angle, vs the analytic
    LOADED hover (this is the §8b question: is the FF actually loaded?)

Run for the elevated paper world (taut 1.0 m cables @ 45deg):
    python3 -m controller_load_mpc.diag_equilibrium --cable-len 1.0 --elev 45 --hover-z 0.6
Or the ground-start slack world:
    python3 -m controller_load_mpc.diag_equilibrium --cable-len 0.6 --elev 45 --hover-z 0.6
"""
import argparse
import numpy as np
import casadi as ca

from .load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from .planner_ocp import generate_load_ocp, nominal_hover_state

GRAV = 9.81


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--load-mass', type=float, default=0.4)
    ap.add_argument('--drone-mass', type=float, default=0.6)
    ap.add_argument('--cable-len', type=float, default=1.0)
    ap.add_argument('--attach-radius', type=float, default=0.08)
    ap.add_argument('--attach-z', type=float, default=0.025)
    ap.add_argument('--elev', type=float, default=45.0, help='cable elevation deg')
    ap.add_argument('--hover-z', type=float, default=0.6)
    ap.add_argument('--thrust-ratio', type=float, default=24.0,
                    help='tracker throttle->accel gain (est_params[0])')
    ap.add_argument('--iters', type=int, default=60)
    a = ap.parse_args()

    inertia = [1.67e-3, 1.67e-3, 3.33e-3]
    rho = [np.array([a.attach_radius * np.cos(2 * np.pi * k / a.n),
                     a.attach_radius * np.sin(2 * np.pi * k / a.n),
                     a.attach_z]) for k in range(a.n)]
    dyn = LoadCableDynamics(a.n, a.load_mass, inertia,
                            [a.cable_len] * a.n, rho, a.drone_mass)
    ocp, solver = generate_load_ocp(dyn)
    N = ocp.solver_options.N_horizon

    # spawn / hover state: load at (0,0,hover_z), taut cables at `elev`
    x_eq = nominal_hover_state(dyn, load_pos=(0.0, 0.0, a.hover_z),
                               elevation_deg=a.elev)

    pos_fun = [ca.Function(f'p{i}', [dyn.x], [dyn.quad_position(i)]) for i in range(a.n)]
    acc_fun = [ca.Function(f'a{i}', [dyn.x], [dyn.thrust_vec(i) / dyn.mi[i]]) for i in range(a.n)]

    # yref: hover at setpoint, zero twist, nominal tension
    t_nom = dyn.m * GRAV / (a.n * np.sin(np.deg2rad(a.elev)))
    pose = [0.0, 0.0, a.hover_z, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    yref = np.array(pose + [t_nom] * a.n + [0.0] * (3 * a.n) + [0.0] * dyn.nu)
    q_ref = np.array([1.0, 0.0, 0.0, 0.0])
    for k in range(N):
        solver.set(k, 'yref', yref)
        solver.set(k, 'p', q_ref)
    solver.set(N, 'yref', yref[:-dyn.nu])
    solver.set(N, 'p', q_ref)
    for k in range(N + 1):
        solver.set(k, 'x', x_eq)
    solver.set(0, 'lbx', x_eq)
    solver.set(0, 'ubx', x_eq)

    status = 0
    for _ in range(a.iters):
        status = solver.solve()
    X = np.array([solver.get(k, 'x') for k in range(N + 1)]).T
    x0 = X[:, 0]

    print(f"\n=== equilibrium check: cable_len={a.cable_len}  elev={a.elev}deg  "
          f"hover_z={a.hover_z} ===")
    print(f"OCP solve status: {status}  (0 == converged)")
    print(f"analytic taut-hover tension t_nom = {t_nom:.3f} N/cable")

    # analytic loaded hover per drone: thrust supports drone weight + its share of
    # the load pulled through the cable. Vertical: T_z = m_d*g + t*sin(elev);
    # horizontal (outward): T_h = t*cos(elev).
    phi = np.deg2rad(a.elev)
    Tz = a.drone_mass * GRAV + t_nom * np.sin(phi)
    Th = t_nom * np.cos(phi)
    a_ff_analytic = np.hypot(Tz, Th) / a.drone_mass
    thr_analytic = a_ff_analytic / a.thrust_ratio
    tilt_analytic = np.degrees(np.arctan2(Th, Tz))
    print(f"analytic loaded-hover FF: |a|={a_ff_analytic:.2f} m/s^2  "
          f"throttle={thr_analytic:.3f}  tilt={tilt_analytic:.1f}deg (outward)")
    # unloaded level hover for contrast (what creep FF commands)
    print(f"  (unloaded level hover would be throttle={GRAV/a.thrust_ratio:.3f} "
          f"tilt=0.0deg)")

    print("\nper-drone @ node 0:")
    for i in range(a.n):
        t_i = float(x0[LOAD_DIM + CABLE_DIM * i + 12])
        p_ref = np.array(pos_fun[i](x0)).flatten()
        p_spawn = np.array(nominal_hover_state(dyn, load_pos=(0, 0, a.hover_z),
                                               elevation_deg=a.elev))
        p_spawn = np.array(pos_fun[i](p_spawn)).flatten()
        resid = np.linalg.norm(p_ref - p_spawn)
        a_ff = np.array(acc_fun[i](x0)).flatten()
        thr = np.linalg.norm(a_ff) / a.thrust_ratio
        # tilt of the thrust vector from vertical
        tilt = np.degrees(np.arctan2(np.hypot(a_ff[0], a_ff[1]), a_ff[2]))
        print(f"  d{i}: t={t_i:6.3f}N  ref_pos=[{p_ref[0]:+.3f} {p_ref[1]:+.3f} "
              f"{p_ref[2]:+.3f}]  |ref-consistent|={resid:.4f}  "
              f"FF |a|={np.linalg.norm(a_ff):5.2f} throttle={thr:.3f} tilt={tilt:.1f}deg")


if __name__ == '__main__':
    main()
