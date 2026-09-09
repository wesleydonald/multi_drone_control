#!/usr/bin/env python3
"""
tools/velocity_probe.py — ladder step V-b, offline (THESIS_PLAN §12.1 Stage V)

Closes the velocity loop around the SIL plant with NO ROS and NO acados: one free
drone, a commanded circle, measure the radius ratio and phase lag of the drone against
its own reference. That is the tracker stage in isolation — exactly the stage R0054
measures at 0.869 / +0.538 s under the MPC.

    ./tools/velocity_probe.py                    # the default sweep
    ./tools/velocity_probe.py --gains 2,4,1      # kp_pos,kv,ki
    ./tools/velocity_probe.py --radius 0.5 --speed 0.6 --plot out.png

WHAT THIS CAN AND CANNOT SETTLE. It uses the same plant the SIL bench does, so it
inherits every blind spot in docs/design/sil_bench.md §7 — no contact physics, no
aerodynamics, no mocap noise, and the body-rate time constant is shared with the loop's
own assumptions. It also flies a FREE drone: no cable, no payload, no fleet. So it can
show that the velocity loop tracks a moving reference and roughly how well, and it
cannot show what happens to a tethered fleet. It is step V-b of a seven-step ladder for
that reason.

The comparison it is FAIR to draw is against the MPC's tracker stage on the same
quantity (drone vs its own reference), not against a whole-fleet number.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'src', 'controller_quad_load'))

import metrics as M                                          # noqa: E402
from controller_quad_load.velocity_loop import VelocityLoop   # noqa: E402
from sil.plant import (Link, PayloadParams, QuadParams,       # noqa: E402
                       SilPlant)

G = 9.81


def eased_circle(t, T, radius):
    """The planner's own trajectory: one smootherstep sweep, so the probe tracks the
    same shape Gazebo flies rather than a steady circle it would find easier."""
    u = min(max(t / T, 0.0), 1.0)
    s = u * u * u * (u * (6.0 * u - 15.0) + 10.0)
    ds = 30.0 * u * u * (u - 1.0) * (u - 1.0) / T
    dds = 60.0 * u * (2.0 * u - 1.0) * (u - 1.0) / (T * T)
    th, dth, ddth = 2 * np.pi * s, 2 * np.pi * ds, 2 * np.pi * dds
    p = radius * np.array([np.cos(th), np.sin(th), 0.0])
    v = radius * dth * np.array([-np.sin(th), np.cos(th), 0.0])
    a = radius * np.array([-ddth * np.sin(th) - dth * dth * np.cos(th),
                           ddth * np.cos(th) - dth * dth * np.sin(th), 0.0])
    return p, v, a


def run_tethered(kp_pos, kv, ki, k_att, v_max=2.0, n=3, radius=0.5, speed=0.6,
                 hover_s=4.0, tail_s=3.0, dt=0.001, control_hz=50.0,
                 thrust_ratio=32.9,
                 cable_len=0.5, attach_radius=0.08, attach_z=0.025, load_mass=0.4,
                 load_z=0.45, elev_deg=45.0):
    """Ladder step V-c, offline: THREE drones on cables carrying a payload.

    The free-drone probe cannot show the failure mode that matters. A tethered fleet
    has a coupled pendulum/cable mode the free drone does not, and the first bench run
    at the free-drone gains rang up through it -- tilt 21 -> 33 -> 69 deg with the
    tracking error still under 0.1 m, until the envelope check disarmed the fleet
    (R0067). Same plant the bench uses, ~2 s per gain set instead of ~60 s."""
    rho = [np.array([attach_radius * np.cos(2 * np.pi * k / n),
                     attach_radius * np.sin(2 * np.pi * k / n), attach_z])
           for k in range(n)]
    load0 = np.array([0.0, 0.0, load_z])
    e = np.radians(elev_deg)
    pos = []
    for r in rho:
        az = r[:2] / max(float(np.linalg.norm(r[:2])), 1e-9)
        pos.append(load0 + r + cable_len * np.array([az[0] * np.cos(e),
                                                     az[1] * np.cos(e), np.sin(e)]))
    quads = [QuadParams() for _ in range(n)]
    plant = SilPlant(quads, [Link(rho[i], cable_len, attached=True) for i in range(n)],
                     PayloadParams(mass=load_mass))
    plant.reset(pos, load0)
    for i in range(n):
        plant.armed[i] = True

    loops = [VelocityLoop(kp_pos=kp_pos, kv=kv, ki=ki, k_att=k_att, v_max=v_max)
             for _ in range(n)]
    offs = [np.asarray(p) - load0 for p in pos]      # hold the taut cone shape
    T = 1.875 * 2.0 * np.pi * radius / speed
    # CONTROL RUNS AT control_hz, THE PLANT AT dt. The first version of this probe
    # recomputed the command every 2 ms -- 500 Hz -- and reported the free-drone gains
    # as comfortably stable. The real tracker runs at 50 Hz, where the same gains ring
    # the fleet up until the envelope check fires (R0067/R0068). A probe that does not
    # model the control period cannot answer a gain question.
    ctrl_every = max(1, int(round((1.0 / control_hz) / dt)))
    cmd = [(0.0, 0.0, 9.81 / thrust_ratio, 0.0)] * n
    rows = []
    for k in range(int((hover_s + T + tail_s) / dt)):
        t = k * dt
        traj_t = min(max(t - hover_s, 0.0), T)
        pL, vL, aL = eased_circle(traj_t, T, radius)
        if t < hover_s or t > hover_s + T:
            vL, aL = np.zeros(3), np.zeros(3)
        pL = pL + load0
        tilt = 0.0
        for i in range(n):
            if k % ctrl_every:
                plant.set_command(i, cmd[i][0], cmd[i][1], cmd[i][2] * 2.0 - 1.0,
                                  cmd[i][3], True)
                tilt = max(tilt, plant.drone_tilt_deg(i))
                continue
            st = plant.drone_state(i)
            p, q, v = st[0:3], st[3:7], st[7:10]
            # The whole fleet translates with the load, so each drone's reference is
            # the load reference plus its own fixed offset in the taut cone. The cable
            # tension feedforward is the static one that holds that cone.
            p_ref, v_ref = pL + offs[i], vL
            # Thrust feedforward: accelerate itself with the load reference, cancel its
            # own gravity, and supply its share of the cable tension holding the load
            # up -- along its own cable, which is what tilts the taut cone outward.
            a_self = aL + np.array([0.0, 0.0, G])
            u_c = offs[i] / max(float(np.linalg.norm(offs[i])), 1e-9)
            a_ff = a_self + (load_mass / n / quads[i].mass) * float(
                np.linalg.norm(a_self)) * u_c
            roll, pitch, thr, yaw = loops[i].step(
                p, v, q, p_ref, v_ref, a_ff, ctrl_every * dt, thrust_ratio)
            cmd[i] = (roll, pitch, thr, yaw)
            plant.set_command(i, roll, pitch, thr * 2.0 - 1.0, yaw, True)
            tilt = max(tilt, plant.drone_tilt_deg(i))
        plant.advance(dt, 1)
        if not plant.is_finite():
            raise SystemExit(f'plant diverged at t={t:.2f}s')
        rows.append((t, *pL, *plant.xL, tilt, plant.load_tilt_deg()))

    a = np.array(rows)
    t, ref, act = a[:, 0], a[:, 1:4], a[:, 4:7]
    sweep = (t >= hover_s) & (t <= hover_s + T)
    return {
        't': t, 'ref': ref, 'act': act, 'sweep': sweep,
        'radius_ratio': M.radius_ratio(act[:, :2], ref[:, :2], sweep),
        'phase_lag_s': M.phase_lag_s(t[sweep], ref[sweep, 0], act[sweep, 0]),
        'rmse_m': M.payload_rmse(act, ref, sweep),
        'peak_err_m': float(np.max(np.linalg.norm(act[sweep] - ref[sweep], axis=1))),
        'peak_drone_tilt_deg': float(np.max(a[:, 7])),
        'peak_load_tilt_deg': float(np.max(a[:, 8])),
        'max_throttle': float('nan'),
    }


def run(kp_pos, kv, ki, k_att, radius=0.5, speed=0.6, hover_s=4.0, tail_s=4.0,
        dt=0.002, thrust_ratio=32.9, hold_z=1.0):
    T = 1.875 * 2.0 * np.pi * radius / speed
    quad = QuadParams()
    plant = SilPlant([quad], [Link(np.zeros(3), 0.5, attached=False)])
    p0, _, _ = eased_circle(0.0, T, radius)
    p0 = p0 + np.array([0.0, 0.0, hold_z])
    plant.reset([p0], np.array([0.0, 0.0, -5.0]))     # payload parked far below, unused
    plant.armed[0] = True

    loop = VelocityLoop(kp_pos=kp_pos, kv=kv, ki=ki, k_att=k_att)
    rows = []
    n = int((hover_s + T + tail_s) / dt)
    for k in range(n):
        t = k * dt
        traj_t = min(max(t - hover_s, 0.0), T)
        p_ref, v_ref, a_ref = eased_circle(traj_t, T, radius)
        p_ref = p_ref + np.array([0.0, 0.0, hold_z])
        if t < hover_s or t > hover_s + T:
            v_ref = np.zeros(3)
            a_ref = np.zeros(3)
        # a_ff is the required SPECIFIC THRUST acceleration: trajectory accel plus the
        # gravity the thrust has to cancel. Same quantity the planner hands the MPC.
        a_ff = a_ref + np.array([0.0, 0.0, G])

        st = plant.drone_state(0)
        p, q, v = st[0:3], st[3:7], st[7:10]
        roll, pitch, thr, yaw = loop.step(p, v, q, p_ref, v_ref, a_ff, dt,
                                          thrust_ratio)
        plant.set_command(0, roll, pitch, thr * 2.0 - 1.0, yaw, True)
        plant.advance(dt, 1)
        if not plant.is_finite():
            raise SystemExit(f'plant diverged at t={t:.2f}s')
        rows.append((t, *p_ref, *p, thr))

    a = np.array(rows)
    t, ref, act, thr = a[:, 0], a[:, 1:4], a[:, 4:7], a[:, 7]
    sweep = (t >= hover_s) & (t <= hover_s + T)
    return {
        't': t, 'ref': ref, 'act': act, 'thr': thr, 'sweep': sweep, 'T': T,
        'radius_ratio': M.radius_ratio(act[:, :2], ref[:, :2], sweep),
        'phase_lag_s': M.phase_lag_s(t[sweep], ref[sweep, 0], act[sweep, 0]),
        'rmse_m': M.payload_rmse(act, ref, sweep),
        'peak_err_m': float(np.max(np.linalg.norm(act[sweep] - ref[sweep], axis=1))),
        'z_drift_m': float(np.max(np.abs(act[:, 2] - hold_z))),
        'max_throttle': float(np.max(thr)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--gains', default='2,4,1', help='kp_pos,kv,ki')
    ap.add_argument('--k-att', type=float, default=8.0)
    ap.add_argument('--radius', type=float, default=0.5)
    ap.add_argument('--speed', type=float, default=0.6)
    ap.add_argument('--sweep-gains', action='store_true',
                    help='try a small grid instead of one gain set')
    ap.add_argument('--plot')
    args = ap.parse_args()

    print(__doc__.split('    ./tools')[0].strip())
    print(f'\n  free drone, eased circle r={args.radius} v={args.speed} peak\n')
    hdr = (f'  {"kp_pos":>6} {"kv":>5} {"ki":>5} {"k_att":>6} | '
           f'{"radius ratio":>12} {"phase lag":>10} {"rmse":>8} {"peak err":>9} '
           f'{"max thr":>8}')
    print(hdr)
    print('  ' + '-' * (len(hdr) - 2))

    if args.sweep_gains:
        grid = [(kp, kv, ki) for kp in (1.0, 2.0, 4.0)
                for kv in (2.0, 4.0, 8.0) for ki in (0.0, 1.0)]
    else:
        grid = [tuple(float(x) for x in args.gains.split(','))]

    best = None
    for kp, kv, ki in grid:
        try:
            r = run(kp, kv, ki, args.k_att, args.radius, args.speed)
        except SystemExit as e:
            print(f'  {kp:6.1f} {kv:5.1f} {ki:5.1f} {args.k_att:6.1f} | {e}')
            continue
        # Saturated throttle or a grossly oversized orbit is the loop going unstable,
        # not a tracking result. Flag it, and never let it win the selection: at
        # kp_pos=4/kv=8 the orbit blows out to 2-4x radius with throttle pinned at the
        # 0.6 clamp, and read as a bare number that looks like excellent radius ratio.
        unstable = r['max_throttle'] >= 0.599 or not 0.5 < r['radius_ratio'] < 1.5
        print(f'  {kp:6.1f} {kv:5.1f} {ki:5.1f} {args.k_att:6.1f} | '
              f'{r["radius_ratio"]:12.3f} {r["phase_lag_s"]:+10.3f} '
              f'{r["rmse_m"]:8.3f} {r["peak_err_m"]:9.3f} {r["max_throttle"]:8.3f}'
              + ('   UNSTABLE' if unstable else ''))
        # Ranked on RMSE, not on lag: the loop has essentially no lag anywhere in the
        # stable region, so ranking on it just picks sampling noise.
        if not unstable and (best is None or r['rmse_m'] < best[1]['rmse_m']):
            best = ((kp, kv, ki), r)

    if best:
        (kp, kv, ki), r = best
        print(f'\n  best (lowest RMSE, stable): kp_pos={kp} kv={kv} ki={ki} -> '
              f'ratio {r["radius_ratio"]:.3f}, lag {r["phase_lag_s"]:+.3f} s, '
              f'rmse {r["rmse_m"]:.3f} m')
        print('\n  The MPC tracker stage on the same quantity (R0054, Gazebo, 3-drone '
              'carry):\n     ratio 0.869, lag +0.538 s.')
        print('  NOT a like-for-like comparison — this is a free drone with no cable, '
              'no payload\n  and no fleet, in a plant with no contact physics. Step '
              'V-b of seven.')
    if args.plot and best:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        r = best[1]
        fig, ax = plt.subplots(figsize=(5.5, 5.5))
        m = r['sweep']
        ax.plot(r['ref'][m, 0], r['ref'][m, 1], 'k--', label='reference')
        ax.plot(r['act'][m, 0], r['act'][m, 1], 'C2-', label='velocity loop')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'V-b: ratio {r["radius_ratio"]:.3f}, '
                     f'lag {r["phase_lag_s"]:+.3f} s')
        fig.savefig(args.plot, dpi=150, bbox_inches='tight')
        print(f'\n  -> {args.plot}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
