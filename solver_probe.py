#!/usr/bin/env python3
"""
solver_probe.py - why does the tracker command a standing pitch rate at zero error?

No Gazebo, no ROS. Builds the same acados solver the tracker uses, hands it a
PERFECT hover state (drone exactly on its reference, level, at rest) and reads
back what it commands. If the real bug is in the OCP setup rather than the plant,
it reproduces here in a couple of seconds.

The plant has been cleared by rate_probe.py: the rate loop tracks a commanded
body rate to 3 significant figures with the correct sign. So attitude is just the
integral of what this solver asks for, and a standing u1 IS the tip-over.

    python3 solver_probe.py
"""
import os
import sys
import numpy as np

ACADOS_DIR = '/home/wesley/multi_drone_control/c_generated_code_quad_load'
os.makedirs(ACADOS_DIR, exist_ok=True)
os.chdir(ACADOS_DIR)
sys.path.insert(0, '/home/wesley/multi_drone_control/src/controller_quad_load')

from controller_quad_load.acados import (          # noqa: E402
    generate_ocp_controller, set_initial_guess, set_planner_reference,
    warm_start_from_previous_solution)

N = 20
THRUST_RATIO = 24.0
# same vector the tracker builds: [kT, drag_z, tau_rate, centre_deg, max_deg, expo]
EST = np.array([THRUST_RATIO, 0.0, 0.12, 70.0, 670.0, 0.5])

# drone 0's spawn in three_soft_paper.sdf
P0 = np.array([0.787107, 0.0, 0.757107])


def hover_refs(pos, n=N + 1, attitude_ff=False):
    """Stationary hover reference: hold pos, zero velocity, pure-vertical 1g."""
    ref_pos = np.tile(pos, (n, 1))
    ref_vel = np.zeros((n, 3))
    ref_acc = np.tile([0.0, 0.0, 9.81], (n, 1))
    if not attitude_ff:                      # what the launch currently runs
        ref_acc = np.stack([[0.0, 0.0, float(np.linalg.norm(a))] for a in ref_acc])
    return ref_pos, ref_vel, ref_acc


def state(pos, q=(1.0, 0.0, 0.0, 0.0), vel=(0, 0, 0), rate=(0, 0, 0),
          u=(0.0, 0.0, 9.81 / THRUST_RATIO, 0.0)):
    """17-vector [p(3), q(4), v(3), r(3), u_state(4)] as the tracker builds it."""
    return np.concatenate([pos, q, vel, rate, u])


def solve(ocp, x0, ref, iters=1, relax=None):
    ref_pos, ref_vel, ref_acc = ref
    set_planner_reference(ocp, ref_pos, ref_vel, ref_acc, N, EST,
                          ref_cable=np.zeros((N + 1, 3)))
    if relax is None:                        # exact pin
        ocp.set(0, "lbx", x0)
        ocp.set(0, "ubx", x0)
    else:                                    # tracker's multiplicative band
        ocp.set(0, "lbx", x0 * (1 - relax))
        ocp.set(0, "ubx", x0 * (1 + relax))
    st = None
    for _ in range(iters):
        st = ocp.solve()
    return st, ocp.get(1, "x")[-4:]


def show(tag, st, u):
    hov = 9.81 / THRUST_RATIO
    print(f"  {tag:<34s} status={st}  "
          f"roll={u[0]:+.4f} pitch={u[1]:+.4f} thr={u[2]:.4f} yaw={u[3]:+.4f}"
          f"   (hover thr {hov:.3f})")


def main():
    print("building solver (may compile)...")
    ocp = generate_ocp_controller(generate=True, build=True)
    print("\nPERFECT HOVER: drone exactly on reference, level, at rest.")
    print("Expect roll/pitch/yaw ~0 and thr ~hover. Anything else is the bug.\n")

    ref = hover_refs(P0)
    x0 = state(P0)

    set_initial_guess(ocp, N)
    st, u = solve(ocp, x0, ref, iters=1)
    show("1 SQP_RTI iter, exact x0", st, u)

    for k in range(2, 6):
        st, u = solve(ocp, x0, ref, iters=1)
        show(f"iter {k} (warm)", st, u)

    print()
    ocp2 = generate_ocp_controller(generate=False, build=False)
    set_initial_guess(ocp2, N)
    st, u = solve(ocp2, x0, ref, iters=1, relax=0.025)
    show("multiplicative x0 band (as shipped)", st, u)

    print("\n-- sensitivity: which input moves the standing command? --")
    cases = {
        "yaw 90deg (q=[.707,0,0,.707])":
            state(P0, q=(0.70710678, 0.0, 0.0, 0.70710678)),
        "at origin (pos=0,0,0)": state(P0 * 0.0),
        "pos +x only (1,0,0)": state(np.array([1.0, 0.0, 0.0])),
        "u_state seeded zero": state(P0, u=(0.0, 0.0, 0.0, 0.0)),
        "small pitch tilt (5deg)":
            state(P0, q=(0.99619, 0.0, 0.08716, 0.0)),
    }
    for tag, xs in cases.items():
        r = hover_refs(xs[0:3])
        o = generate_ocp_controller(generate=False, build=False)
        set_initial_guess(o, N)
        st, u = solve(o, xs, r, iters=1)
        show(tag, st, u)

    print("\n-- does it persist over a closed-loop rollout? --")
    o = generate_ocp_controller(generate=False, build=False)
    set_initial_guess(o, N)
    x = state(P0)
    for k in range(6):
        st, u = solve(o, x, ref, iters=1)
        if k % 1 == 0:
            show(f"rollout step {k}", st, u)
        x = np.concatenate([x[:13], u])       # feed the command back as u_state
        warm_start_from_previous_solution(o, N)


if __name__ == "__main__":
    main()
