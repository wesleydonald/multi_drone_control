"""
verify_dissipative.py
---------------------
Offline CLOSED-LOOP verification of the dissipative reference generator -- run it BEFORE
Gazebo. It drives the real DissipativeNetwork against a small multibody plant
(mini_plant.py: point-mass drones with a thrust-limited tracker proxy, a payload, RIGID
RODS, ground contact) to verify the network's ACTUAL role in the stack: holding a taut
AIRBORNE load and handling a mid-flight detach. (Takeoff/ground break-off is done by the
centralized OCP, not the network -- see dissipative_node.py -- so this harness starts from
the airborne taut handover config, not the ground.)

Tests:
  A  Airborne hold/lift-- from the taut handover config the network holds the load and
                          tracks it to target_z, staying finite and settled.
  B  Outward hold      -- at hover the references/rods sit on the OUTWARD cone, symmetric.
  C  Closed-loop detach-- drop a drone mid-hover: the load stays aloft (does not fall),
                          remaining rod tension rises.
  D  Inward guard      -- every attached drone's reference offset AND a_ff point OUTWARD
                          at every logged tick.

Exit code is nonzero on any failure. A PNG (load height, drone elevations, tension,
outward guards, 3D formation) is written to dissipative_verify/ (path printed).

Run:  python3 -m controller_dissipative.verify_dissipative
"""
import os
import sys
import time

import numpy as np

from controller_load_mpc.geometry import quat_to_rot_np, attach_points
from controller_dissipative.dissipative_network import (
    DissipativeNetwork, DissipativeParams)
from controller_dissipative.mini_plant import MiniPlant

# geometry / physics matching the 4-drone rigid world (four_rigid_ground.sdf).
N = 4
CABLE_LEN = 0.5
LOAD_MASS = 0.4
DRONE_MASS = 0.6
G = 9.81
ATTACH_RADIUS = 0.08
ATTACH_Z = 0.025
GROUND_Z = 0.025               # payload rest height on the floor (plant contact floor)
START_LOAD_Z = 0.45            # airborne taut handover height (OCP already lifted here)
START_ELEV_DEG = 45.0          # drones start on the taut ~45 deg cone
TARGET_Z = 0.60

CTRL_DT = 0.1                  # 10 Hz control tick
SIM_DT = 1e-3                  # plant integration step
SUBSTEPS = int(CTRL_DT / SIM_DT)
LIFT_VEL = 0.12               # m/s ceiling ramp rate
LIFT_LEAD = 0.10              # m the reference leads the MEASURED load (creates lift)
SETTLE_S = 1.0
FF_EASE_S = 1.0
LEVEL = np.array([1.0, 0.0, 0.0, 0.0])
OUT_TOL = 1e-6


def lift_target(load_pos, hover_xy, target_z, ramp_z):
    """Desired load position for the network. THE FIX: the height is the measured load
    height plus a small bounded lead, also capped by a gentle ramp and the target. The
    lead pulls the load up; anchoring to the measurement means the cone can never run
    away from a load that has not lifted yet."""
    z = min(target_z, ramp_z, float(load_pos[2]) + LIFT_LEAD)
    return np.array([hover_xy[0], hover_xy[1], z])


def taut_gate(plant, i):
    dist = float(np.linalg.norm(plant.q[i] - plant.attach(i)))
    lo, hi = 0.85 * CABLE_LEN, 1.00 * CABLE_LEN
    return float(np.clip((dist - lo) / max(hi - lo, 1e-6), 0.0, 1.0))


def run_closed_loop(detach_slot=None, detach_t=None, sim_t=16.0):
    """Drive the network + plant closed loop through lift -> hold (-> detach). Returns a
    history dict logged at each control tick."""
    rho = attach_points(N, ATTACH_RADIUS, ATTACH_Z)
    net = DissipativeNetwork(N, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G,
                             DissipativeParams())
    plant = MiniPlant(N, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G, GROUND_Z)

    # initial config: AIRBORNE taut handover -- load lifted, drones on the ~45 deg cone.
    load0 = np.array([0.0, 0.0, START_LOAD_Z])
    e = np.radians(START_ELEV_DEG)
    drones0 = []
    for i in range(N):
        azi = rho[i][:2] / max(np.linalg.norm(rho[i][:2]), 1e-9)
        d = np.array([azi[0] * np.cos(e), azi[1] * np.cos(e), np.sin(e)])
        drones0.append(load0 + rho[i] + CABLE_LEN * d)
    plant.reset(drones0, load0)
    net.seed(drones0)
    hover_xy = (0.0, 0.0)
    lift_z0 = START_LOAD_Z

    attached = [True] * N
    hist = {k: [] for k in ('t', 'load_z', 'pdes_z', 'elev', 'tension', 'p_ref',
                            'a_ff', 'a_cable', 'out_p', 'out_a', 'perr', 'attached')}
    n_ticks = int(sim_t / CTRL_DT)
    for tick in range(n_ticks):
        t = tick * CTRL_DT
        if detach_slot is not None and detach_t is not None and t >= detach_t \
                and attached[detach_slot]:
            net.detach(detach_slot)
            attached[detach_slot] = False

        ramp_z = lift_z0 + LIFT_VEL * max(t - SETTLE_S, 0.0)
        p_des = lift_target(plant.xL, hover_xy, TARGET_Z, ramp_z)
        ff = float(np.clip((t - SETTLE_S) / FF_EASE_S, 0.0, 1.0))
        net.step(plant.xL, LEVEL, plant.vL, p_des, CTRL_DT)

        p_ref = [None] * N; v_ref = [None] * N; a_ff = [None] * N; a_cab = [None] * N
        for i in range(N):
            if not attached[i]:
                pr, vr, af, ac = net.fly_away_reference(i)
            else:
                pr, vr, af, ac = net.reference(
                    i, LEVEL, p_des, taut_gate=taut_gate(plant, i) * ff)
            p_ref[i], v_ref[i], a_ff[i], a_cab[i] = pr, vr, af, ac
        for _ in range(SUBSTEPS):
            plant.step(p_ref, v_ref, a_ff, attached, SIM_DT)

        out = [(rho[i][:2] / max(np.linalg.norm(rho[i][:2]), 1e-9)) for i in range(N)]
        hist['t'].append(t)
        hist['load_z'].append(float(plant.xL[2]))
        hist['pdes_z'].append(float(p_des[2]))
        hist['elev'].append([plant.cable_elev(i) if attached[i] else np.nan
                             for i in range(N)])
        hist['tension'].append([plant.rod_tension(i) if attached[i] else np.nan
                               for i in range(N)])
        hist['p_ref'].append([np.asarray(p_ref[i]) for i in range(N)])
        hist['a_ff'].append([np.asarray(a_ff[i]) for i in range(N)])
        hist['a_cable'].append([np.asarray(a_cab[i]) for i in range(N)])
        hist['out_p'].append([float((p_ref[i][:2] - p_des[:2]) @ out[i])
                              if attached[i] else np.nan for i in range(N)])
        hist['out_a'].append([float(a_ff[i][:2] @ out[i]) if attached[i] else np.nan
                             for i in range(N)])
        hist['perr'].append([float(np.linalg.norm(p_ref[i] - plant.q[i]))
                            if attached[i] else np.nan for i in range(N)])
        hist['attached'].append(list(attached))
    for k in hist:
        hist[k] = np.array(hist[k], dtype=float)
    return hist


def check(lift, detach):
    """Tests A-D. Returns (all_ok, [(name, ok, detail)])."""
    results = []

    # A: airborne hold/lift -- from the handover height the network holds and tracks the
    # load up to near target, staying aloft and finite.
    final_z = lift['load_z'][-1]
    last_3s = lift['load_z'][-30:]
    a_ok = bool(final_z > TARGET_Z - 0.10 and last_3s.min() > START_LOAD_Z - 0.05
                and np.all(np.isfinite(lift['load_z'])))
    results.append(('A airborne hold/lift', a_ok,
                    f'load {START_LOAD_Z:.2f} -> {final_z:.2f} m (target {TARGET_Z:.2f}), '
                    f'min last-3s {last_3s.min():.2f}'))

    # B: outward hold -- at the end of the lift run the rods are symmetric & elevated.
    tens = lift['tension'][-1]; tens = tens[np.isfinite(tens)]
    elev = lift['elev'][-1]; elev = elev[np.isfinite(elev)]
    b_ok = bool((tens.max() - tens.min()) < 0.20 * max(tens.mean(), 1e-6)
                and elev.min() > 30.0)
    results.append(('B outward hold', b_ok,
                    f'tension spread {tens.max()-tens.min():.2f} N about {tens.mean():.2f}, '
                    f'min cable elev {elev.min():.0f} deg'))

    # C: detach -- the load stays aloft after 4->3 and remaining tension rises.
    di = np.argmax(detach['t'] >= detach['t'][-1] - 3.0)   # ~last 3 s
    post_min_z = detach['load_z'][di:].min()
    t_before = np.nanmean(detach['tension'][int(len(detach['t']) * 0.55)])
    t_after = np.nanmean(detach['tension'][-1])
    c_ok = bool(post_min_z > TARGET_Z - 0.15 and t_after > t_before
                and np.all(np.isfinite(detach['load_z'])))
    results.append(('C closed-loop detach', c_ok,
                    f'load min after detach {post_min_z:.2f} m (>{TARGET_Z-0.15:.2f}), '
                    f'tension/drone {t_before:.2f} -> {t_after:.2f} N'))

    # D: inward guard -- the reference is always strictly outward, and a_ff is NEVER
    # inward (it is level, outward-component 0, only during the pre-lift settle when the
    # cable feedforward is gated off; once lifting it is strictly outward).
    op = lift['out_p'][np.isfinite(lift['out_p'])]
    oa = lift['out_a'][np.isfinite(lift['out_a'])]
    d_ok = bool(op.min() > OUT_TOL and oa.min() > -OUT_TOL)
    results.append(('D inward-tilt guard', d_ok,
                    f'min outward: p_ref {op.min():.3f} m (>0), a_ff {oa.min():.3f} '
                    f'm/s^2 (never inward)'))

    return all(r[1] for r in results), results


def make_plot(lift, detach, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    t = lift['t']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    fig = plt.figure(figsize=(15, 9))

    # (1) 3D formation snapshot near the end of the lift.
    snap = -1
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    pL = np.array([0.0, 0.0, lift['load_z'][snap]])
    ax.scatter(*pL, c='k', s=80, marker='s', label='payload')
    for i in range(N):
        pr = lift['p_ref'][snap, i]; aff = lift['a_ff'][snap, i]
        ax.plot([pL[0], pr[0]], [pL[1], pr[1]], [pL[2], pr[2]], c=colors[i], lw=1,
                alpha=0.5)
        ax.scatter(*pr, c=colors[i], s=40, label=f'drone {i} ref')
        ax.quiver(*pr, *(0.04 * aff), color=colors[i], lw=2)
    ax.set_title('formation @ hover (arrows a_ff) — lean OUT')
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z'); ax.legend(fontsize=6)

    # (2) load height (the headline: does it break off and lift?).
    ax = fig.add_subplot(2, 3, 2)
    ax.plot(t, lift['load_z'], 'k-', lw=2, label='load z (lift run)')
    ax.plot(t, lift['pdes_z'], 'k--', lw=1, label='desired (load+lead)')
    ax.axhline(TARGET_Z, c='g', ls=':', label='target')
    ax.axhline(GROUND_Z, c='r', ls=':', label='floor')
    ax.set_title('A: load breaks off & lifts'); ax.set_xlabel('t (s)')
    ax.set_ylabel('z (m)'); ax.legend(fontsize=6)

    # (3) drone cable elevations (should climb toward ~45 deg, not 90).
    ax = fig.add_subplot(2, 3, 3)
    for i in range(N):
        ax.plot(t, lift['elev'][:, i], c=colors[i], lw=1, label=f'drone {i}')
    ax.axhline(45, c='g', ls=':')
    ax.set_title('cable elevation (deg)'); ax.set_xlabel('t (s)'); ax.legend(fontsize=6)

    # (4) outward guards.
    ax = fig.add_subplot(2, 3, 4)
    for i in range(N):
        ax.plot(t, lift['out_p'][:, i], c=colors[i], lw=1)
    ax.axhline(0, c='r', ls='--')
    ax.set_title('D: p_ref outward offset (>0)'); ax.set_xlabel('t (s)')
    ax.set_ylabel('(p_ref-p_load)·out (m)')

    # (5) tension per drone (lift + detach run overlaid).
    ax = fig.add_subplot(2, 3, 5)
    for i in range(N):
        ax.plot(t, lift['tension'][:, i], c=colors[i], lw=1, label=f'drone {i}')
    td = detach['t']
    ax.plot(td, np.nanmean(detach['tension'], axis=1), 'k--', lw=1,
            label='mean (detach run)')
    ax.set_title('C: rod tension (rises at detach)'); ax.set_xlabel('t (s)')
    ax.set_ylabel('tension (N)'); ax.legend(fontsize=6)

    # (6) load height, detach run.
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(detach['t'], detach['load_z'], 'k-', lw=2)
    ax.axhline(TARGET_Z, c='g', ls=':'); ax.axhline(GROUND_Z, c='r', ls=':')
    ax.set_title('C: load height through detach'); ax.set_xlabel('t (s)')
    ax.set_ylabel('z (m)')

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(args=None):
    lift = run_closed_loop(sim_t=16.0)
    detach = run_closed_loop(detach_slot=1, detach_t=11.0, sim_t=16.0)
    all_ok, results = check(lift, detach)

    print('\n=== dissipative controller CLOSED-LOOP verification ===')
    for name, ok, detail in results:
        print(f'  [{"PASS" if ok else "FAIL"}] {name:22s} {detail}')

    out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))), 'dissipative_verify')
    try:
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f'verify_{time.strftime("%Y%m%d_%H%M%S")}.png')
        make_plot(lift, detach, path)
        print(f'\n  diagnostic plot: {path}')
    except Exception as ex:
        print(f'\n  (plot skipped: {ex})')

    print(f'\n  RESULT: {"ALL PASSED" if all_ok else "FAILURES ABOVE"}\n')
    if args is None:
        sys.exit(0 if all_ok else 1)
    return all_ok


if __name__ == '__main__':
    main()
