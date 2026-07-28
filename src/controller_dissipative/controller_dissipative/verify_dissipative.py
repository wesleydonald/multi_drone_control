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
  F  Closed-loop attach-- start from a 3-drone hold with a 4th drone loitering off-network;
                          weld it on mid-hover (net.attach): the load stays aloft through
                          the 3->4 transient, per-drone tension DROPS, the newcomer settles
                          onto the cone, and the fleet re-spaces to an even 4-gon.
  G  Unequal sharing   -- moment-balanced tensions hold an ASYMMETRIC attach set (3 tethers
                          + an off-centre 4th) level with UNEQUAL per-drone tensions, each
                          drone pinned to its OWN fixed attach azimuth (no even-ring drift).
  H  Off-centre attach -- closed-loop balanced weld of a 4th drone at an off-centre RING
                          point on the RIGID-BODY payload. Reproduces the sim tilt failure:
                          with the formation built against gravity (yaw-only) the load
                          settles near LEVEL; against the tilted body frame it tilts heavily.

NOTE: the payload is a RIGID BODY (mass + inertia) here, so it translates AND rotates -- the
attitude simulation is what makes the post-attach tilt/bunching failures reproducible offline.

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


def run_closed_loop(detach_slot=None, detach_t=None, sim_t=16.0, detaches=None,
                    params=None):
    """Drive the network + plant closed loop through lift -> hold (-> detach). Returns a
    history dict logged at each control tick. `detaches` is a list of (slot, t) events
    (supersedes the single detach_slot/detach_t); `params` overrides the network tuning
    (used to compare detach-smoothing off vs on)."""
    if detaches is None:
        detaches = ([(detach_slot, detach_t)]
                    if detach_slot is not None and detach_t is not None else [])
    rho = attach_points(N, ATTACH_RADIUS, ATTACH_Z)
    net = DissipativeNetwork(N, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G,
                             params or DissipativeParams())
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
        for slot, dt_ev in detaches:
            if t >= dt_ev and attached[slot]:
                net.detach(slot)
                attached[slot] = False

        ramp_z = lift_z0 + LIFT_VEL * max(t - SETTLE_S, 0.0)
        p_des = lift_target(plant.xL, hover_xy, TARGET_Z, ramp_z)
        ff = float(np.clip((t - SETTLE_S) / FF_EASE_S, 0.0, 1.0))
        lq = plant.load_quat()                   # MEASURED payload attitude (now simulated)
        net.step(plant.xL, lq, plant.vL, p_des, CTRL_DT)

        p_ref = [None] * N; v_ref = [None] * N; a_ff = [None] * N; a_cab = [None] * N
        for i in range(N):
            if not attached[i]:
                pr, vr, af, ac = net.fly_away_reference(i)
            else:
                pr, vr, af, ac = net.reference(
                    i, lq, p_des, taut_gate=taut_gate(plant, i) * ff)
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


def run_attach(attach_t=6.0, sim_t=16.0, params=None, rho4=None, loiter4=None,
               handout=False, plant_kwargs=None):
    """Drive the network + plant closed loop through a mid-hover 3->4 ATTACH. Node 3 begins
    OFF the network -- a free drone that has flown its approach and is loitering just inboard
    of its future cone slot (as after a magnet weld, before the network pulls it out). At
    attach_t the weld joins it: net.attach(3, measured) and the plant's rod for node 3
    engages. `rho4` overrides node 3's body attach point (an OFF-CENTRE weld); `loiter4`
    overrides where the newcomer hovers pre-weld. The payload is a rigid body here, so the
    logged `tilt` reveals whether the attach keeps the load LEVEL. Returns (hist, attach_tick)."""
    rho = attach_points(N, ATTACH_RADIUS, ATTACH_Z)
    if rho4 is not None:
        rho[3] = np.asarray(rho4, float)
    net = DissipativeNetwork(N, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G,
                             params or DissipativeParams())
    plant = MiniPlant(N, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G, GROUND_Z,
                      **(plant_kwargs or {}))

    # 3 tethered drones on the ~45 deg cone; node 3 loiters just inboard/below its slot.
    load0 = np.array([0.0, 0.0, START_LOAD_Z])
    e = np.radians(START_ELEV_DEG)
    drones0 = []
    for i in range(N):
        azi = rho[i][:2] / max(np.linalg.norm(rho[i][:2]), 1e-9)
        d = np.array([azi[0] * np.cos(e), azi[1] * np.cos(e), np.sin(e)])
        drones0.append(load0 + rho[i] + CABLE_LEN * d)
    # newcomer's pre-weld hover: default just off its slot; an off-centre weld hovers directly
    # over its (off-centre) attach point, as the magnet drone actually does before welding.
    loiter = (np.asarray(loiter4, float) if loiter4 is not None
              else drones0[3] + np.array([-0.06, -0.05, 0.10]))
    drones0[3] = loiter
    plant.reset(drones0, load0)
    net.seed(drones0)
    net.detach(3)                                          # node 3 starts off the network
    attached = [True, True, True, False]

    hover_xy = (0.0, 0.0)
    lift_z0 = START_LOAD_Z
    hist = {k: [] for k in ('t', 'load_z', 'tilt', 'tension', 'perr3', 'r_ref',
                            'attached', 'handout3', 'elev3', 'az3', 'qaz_all')}
    n_ticks = int(sim_t / CTRL_DT)
    attach_tick = None
    for tick in range(n_ticks):
        t = tick * CTRL_DT
        if attach_tick is None and t >= attach_t:
            net.attach(3, plant.q[3], handout=handout)     # WELD: join the network
            attached[3] = True
            attach_tick = tick

        # hold at the (already-lifted) handover height -- this test isolates the attach.
        p_des = np.array([hover_xy[0], hover_xy[1], TARGET_Z])
        lq = plant.load_quat()                             # MEASURED payload attitude
        net.step(plant.xL, lq, plant.vL, p_des, CTRL_DT)

        p_ref = [None] * N; v_ref = [None] * N; a_ff = [None] * N
        for i in range(N):
            if not attached[i]:
                # free drone: hold at its loiter pose, level thrust, no cable term.
                p_ref[i] = loiter; v_ref[i] = np.zeros(3)
                a_ff[i] = np.array([0.0, 0.0, G])
            else:
                pr, vr, af, _ = net.reference(i, lq, p_des,
                                              taut_gate=taut_gate(plant, i))
                p_ref[i], v_ref[i], a_ff[i] = pr, vr, af
        for _ in range(SUBSTEPS):
            plant.step(p_ref, v_ref, a_ff, attached, SIM_DT)

        hist['t'].append(t)
        hist['load_z'].append(float(plant.xL[2]))
        hist['tilt'].append(plant.load_tilt_deg())
        hist['tension'].append([plant.rod_tension(i) if attached[i] else np.nan
                               for i in range(N)])
        hist['perr3'].append(float(np.linalg.norm(p_ref[3] - plant.q[3])))
        hist['r_ref'].append([float(np.linalg.norm((p_ref[i] - plant.xL)[:2]))
                              if attached[i] else np.nan for i in range(N)])
        hist['attached'].append(list(attached))
        # COMMANDED (network) azimuth of each node's cable direction about the load centre --
        # the direction the fleet is being told to hold. Its per-tick change over a respace
        # reveals whether the existing members are eased outward or yanked (the even-respace fix).
        hist['qaz_all'].append([
            float(np.degrees(np.arctan2((net.q[i] - plant.xL)[1],
                                        (net.q[i] - plant.xL)[0])) % 360.0)
            if attached[i] else np.nan for i in range(N)])
        hist['handout3'].append(float(net.handout[3]))
        hist['elev3'].append(plant.cable_elev(3) if attached[3] else np.nan)
        d3 = plant.q[3] - plant.attach(3)
        hist['az3'].append(float(np.degrees(np.arctan2(d3[1], d3[0])) % 360.0)
                           if attached[3] else np.nan)
    for k in hist:
        hist[k] = np.array(hist[k], dtype=float)
    return hist, attach_tick


def run_balanced_asym(sim_t=30.0):
    """Settle the network with UNEQUAL force sharing on an ASYMMETRIC attach set -- 3
    tethers at 120 deg + a 4th welded at an interstitial off-centre point (as when a
    mid-flight newcomer welds between two existing drones). The mini-plant holds the load
    level by construction, so this test checks the FEEDFORWARD directly: the wrench the
    per-drone tensions apply must have ~zero net moment (the load would stay LEVEL) and
    full weight support, with UNEQUAL positive tensions. Returns (F,M,rF,rM,tensions)."""
    rho = attach_points(3, ATTACH_RADIUS, ATTACH_Z)
    rho.append(np.array([ATTACH_RADIUS * np.cos(np.radians(60.0)),
                         ATTACH_RADIUS * np.sin(np.radians(60.0)), ATTACH_Z]))
    net = DissipativeNetwork(4, rho, CABLE_LEN, DRONE_MASS, LOAD_MASS, G,
                             DissipativeParams(balanced_tensions=True))
    p_des = np.array([0.0, 0.0, TARGET_Z])
    # seed the newcomer OFF its slot (loitering low/inboard, as just after a weld) so the run
    # also proves it CONVERGES to its own fixed attach azimuth WITHOUT the fleet chasing an
    # unreachable even 4-gon (the sim drift/lever crash).
    seed = [net.cone_target(i, LEVEL, p_des) for i in range(4)]
    seed[3] = p_des + np.array([0.0, 0.10, 0.15])
    net.seed(seed)
    for _ in range(int(sim_t / CTRL_DT)):
        net.step(p_des, LEVEL, np.zeros(3), p_des, CTRL_DT)
    F, M, rF, rM = net.net_wrench(LEVEL, p_des)
    tens = np.array([net._solve_tensions(quat_to_rot_np(LEVEL), p_des)[i]
                     for i in range(4)])
    # each node's settled azimuth vs its FIXED attach azimuth (rho) -- must match (no drift to
    # an even ring) -- and its horizontal radius must sit on the cone rim (no runaway).
    def _az(v):
        return np.degrees(np.arctan2(v[1], v[0])) % 360.0
    az_err = max(abs((_az(net.q[i][:2] - p_des[:2]) - _az(rho[i])) % 360.0) for i in range(4))
    az_err = min(az_err, 360.0 - az_err)
    r_ref = [float(np.linalg.norm(net.q[i][:2] - p_des[:2])) for i in range(4)]
    return F, M, rF, rM, tens, az_err, r_ref


def transient_after(hist, t0, window=4.0):
    """Post-detach transient in [t0, t0+window]: the worst survivor tracking-error spike
    (|p_ref - drone|) and the deepest load-height dip below the pre-detach height. Lower
    is more robust -- the numbers the detach-smoothing is meant to shrink."""
    t = hist['t']
    idx_pre = int(np.argmax(t >= t0 - 0.11)) if np.any(t >= t0 - 0.11) else 0
    pre_z = float(hist['load_z'][idx_pre])
    m = (t >= t0) & (t <= t0 + window)
    perr = hist['perr'][m]
    peak_perr = float(np.nanmax(perr)) if perr.size and np.any(np.isfinite(perr)) else 0.0
    dip = float(pre_z - np.min(hist['load_z'][m])) if np.any(m) else 0.0
    return peak_perr, dip


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

    # E: the phased-slot azimuth pinning reduces the detach transient. Run the harder
    # 4->3->2 sequence twice -- slot spring OFF (k_slot=0, the free-rotation degeneracy) vs
    # ON (default) -- and compare the transient after the 3->2 detach (where the real run
    # limit-cycled). ON must lower BOTH the peak tracking error and the load dip.
    evs = [(1, 6.0), (2, 11.0)]
    off = run_closed_loop(detaches=evs, sim_t=18.0,
                          params=DissipativeParams(k_slot=0.0))
    on = run_closed_loop(detaches=evs, sim_t=18.0, params=DissipativeParams())
    off_perr, off_dip = transient_after(off, 11.0)
    on_perr, on_dip = transient_after(on, 11.0)
    e_ok = bool(on_perr < off_perr and on_dip <= off_dip + 1e-3
                and np.all(np.isfinite(on['load_z'])))
    results.append(('E azimuth-pin robustness', e_ok,
                    f'3->2 peak track err {off_perr:.3f} -> {on_perr:.3f} m '
                    f'({100*(off_perr-on_perr)/max(off_perr,1e-6):.0f}% lower), '
                    f'load dip {off_dip:.3f} -> {on_dip:.3f} m'))
    all_ok = all_ok and e_ok

    # F: closed-loop ATTACH. From a 3-drone hold, weld a 4th drone on mid-hover. The load
    # must stay aloft through the transient, the per-drone tension must DROP (the newcomer
    # picks up its share), and the newcomer must settle (small tracking error at the end).
    att, att_tick = run_attach(attach_t=6.0, sim_t=16.0)
    post = att['load_z'][att_tick:]
    f_load_ok = bool(post.min() > TARGET_Z - 0.15 and np.all(np.isfinite(att['load_z'])))
    t_before_att = np.nanmean(att['tension'][att_tick - 1])       # 3-drone share
    t_after_att = np.nanmean(att['tension'][-1])                  # 4-drone share
    f_tension_ok = bool(t_after_att < t_before_att)
    f_settle_ok = bool(att['perr3'][-1] < 0.10)
    f_ok = f_load_ok and f_tension_ok and f_settle_ok
    results.append(('F closed-loop attach', f_ok,
                    f'load min after attach {post.min():.2f} m (>{TARGET_Z-0.15:.2f}), '
                    f'tension/drone {t_before_att:.2f} -> {t_after_att:.2f} N (drops), '
                    f'newcomer perr {att["perr3"][-1]:.3f} m'))
    all_ok = all_ok and f_ok

    # G: UNEQUAL force sharing (moment-balanced). On an asymmetric attach set (3 at 120deg
    # + a 4th interstitial weld) equal sharing leaves a net moment and the rigid load tilts
    # (the reconfiguration blocker). The wrench solve must instead balance the moment (~0,
    # load stays LEVEL) and support full weight, with UNEQUAL positive tensions.
    Fb, Mb, rFb, rMb, tb, az_err, r_ref = run_balanced_asym()
    g_level_ok = bool(np.linalg.norm(rMb) < 0.06)
    g_lift_ok = bool(abs(rFb[2]) < 0.01 * Fb[2] and np.all(np.isfinite(tb)))
    g_unequal_ok = bool((tb.max() - tb.min()) > 0.02 * tb.mean() and np.all(tb >= 0.0))
    # the newcomer must settle at ITS OWN attach azimuth (no drift to an even ring) and every
    # node stays on the cone rim -- the fix for the sim drift/lever crash.
    g_fixed_ok = bool(az_err < 5.0 and max(r_ref) < 0.45 and min(r_ref) > 0.25)
    g_ok = g_level_ok and g_lift_ok and g_unequal_ok and g_fixed_ok
    results.append(('G unequal force sharing', g_ok,
                    f'asym attach: |moment| {np.linalg.norm(rMb):.3f} N.m (~0 -> level), '
                    f'Fz residual {rFb[2]:+.3f} N, tensions {np.round(tb, 2)} N (unequal), '
                    f'azimuth err {az_err:.1f}deg (pinned to weld), r_ref {np.round(r_ref, 2)}'))
    all_ok = all_ok and g_ok

    # H: CLOSED-LOOP off-centre balanced ATTACH on the RIGID-BODY payload -- the test that
    # reproduces the sim tilt/bunching failure (a level-by-construction plant could not). A
    # 4th drone welds at an off-centre RING point (gap centre, azimuth 180deg) and joins in
    # balanced (unequal-force) mode. With the formation built against the tilted body frame
    # the load settles heavily tilted (~14deg here, ~60deg in Gazebo); built against GRAVITY
    # (yaw-only, the fix) it settles near LEVEL. Assert the load stays aloft, ends near level,
    # and the newcomer carries real tension (it shares the load, not a decoration).
    R4 = ATTACH_RADIUS
    rho4 = [R4 * np.cos(np.radians(180.0)), R4 * np.sin(np.radians(180.0)), ATTACH_Z]
    loiter4 = [rho4[0], rho4[1], TARGET_Z + 0.45]          # hover over the off-centre weld
    hatt, h_tick = run_attach(attach_t=4.0, sim_t=20.0,
                              params=DissipativeParams(balanced_tensions=True),
                              rho4=rho4, loiter4=loiter4)
    h_post = hatt['load_z'][h_tick:]
    h_final_tilt = float(np.mean(hatt['tilt'][-10:]))      # settled tilt (last ~1 s)
    h_newcomer_t = float(hatt['tension'][-1][3])
    h_aloft_ok = bool(h_post.min() > TARGET_Z - 0.20 and np.all(np.isfinite(hatt['load_z'])))
    h_level_ok = bool(h_final_tilt < 8.0)                  # yaw-only ~4deg; full-R ~14deg
    h_share_ok = bool(h_newcomer_t > 0.3)                  # newcomer bears real load
    h_ok = h_aloft_ok and h_level_ok and h_share_ok
    results.append(('H off-centre attach level', h_ok,
                    f'gravity-framed balanced attach: settled tilt {h_final_tilt:.1f}deg '
                    f'(<8), load min {h_post.min():.2f} m, newcomer tension '
                    f'{h_newcomer_t:.2f} N (shares load)'))
    all_ok = all_ok and h_ok

    # I & J: SOFT HAND-OUT of an off-centre newcomer under a FEEDFORWARD-DOMINANT tracker
    # proxy (kp=5, kd=4 -- weaker position feedback, closer to the real acados tracker that
    # trusts the cable feedforward open-loop and has no integrator). Same gap weld as H, but
    # the newcomer now hangs TAUT at one cable length above its weld point (as a magnet on a
    # pendulum welds at natural length -- no rod-compression snap).
    #
    # HONEST SCOPE (see the memory note): this offline plant CANNOT reproduce the divergent
    # Gazebo runaway (tilt 12->90deg). Its PD proxy still corrects position error, so even an
    # INSTANT weld recovers here (~8 deg peak). The real MPC under-thrusts because it caps the
    # cable feedforward at 6 m/s^2 and never integrates the resulting droop -- behaviour a PD
    # proxy does not have. So these tests do NOT claim to reproduce the runaway; they verify
    # the two things the harness CAN show: (I) the hand-out keeps the newcomer's reference far
    # closer to equilibrium during the transit -- the very quantity (distance from equilibrium
    # -> actual force above the modelled feedforward) that drives the real under-thrust -- and
    # (J) the hand-out settles the load LEVEL and aloft with the newcomer a correct off-centre
    # ring member. The runaway fix itself is validated in Gazebo.
    R4 = ATTACH_RADIUS
    rho4 = [R4 * np.cos(np.radians(180.0)), R4 * np.sin(np.radians(180.0)), ATTACH_Z]
    taut4 = [rho4[0], rho4[1], TARGET_Z + ATTACH_Z + CABLE_LEN]   # hang taut, no snap
    ff_plant = dict(kp=5.0, kd=4.0)

    # I: the hand-out keeps the newcomer NEAR EQUILIBRIUM through the transit. Compare the
    # newcomer's peak reference tracking demand (|p_ref - drone|, ~how far the tracker is
    # asked to haul it, hence how far actual force runs ahead of the modelled feedforward)
    # for an INSTANT weld vs the soft hand-out. The hand-out must roughly halve it.
    iatt, i_tick = run_attach(attach_t=4.0, sim_t=18.0,
                              params=DissipativeParams(balanced_tensions=True),
                              rho4=rho4, loiter4=taut4, handout=False, plant_kwargs=ff_plant)
    jatt, j_tick = run_attach(attach_t=4.0, sim_t=26.0,
                              params=DissipativeParams(balanced_tensions=True, T_handout=12.0),
                              rho4=rho4, loiter4=taut4, handout=True, plant_kwargs=ff_plant)
    i_peak_err = float(np.nanmax(iatt['perr3'][i_tick:i_tick + 80]))
    j_peak_err = float(np.nanmax(jatt['perr3'][j_tick:j_tick + 80]))
    i_ok = bool(j_peak_err < 0.7 * i_peak_err
                and np.all(np.isfinite(iatt['load_z'])) and np.all(np.isfinite(jatt['load_z'])))
    results.append(('I hand-out stays near equilibrium', i_ok,
                    f'newcomer peak transit track-err: instant {i_peak_err:.3f} m -> '
                    f'hand-out {j_peak_err:.3f} m ({100*(1-j_peak_err/max(i_peak_err,1e-6)):.0f}% '
                    f'closer to equilibrium)'))
    all_ok = all_ok and i_ok

    # J: the soft hand-out settles the load LEVEL and aloft, the hand-out completes, and the
    # newcomer ends a real off-centre RING member (design elevation at the captured azimuth,
    # bearing real tension) -- a full, correct central->ring reconfiguration.
    j_post = jatt['load_z'][j_tick:]
    j_max_tilt = float(np.nanmax(jatt['tilt'][j_tick:]))
    j_final_tilt = float(np.mean(jatt['tilt'][-10:]))
    j_handout = float(jatt['handout3'][-1])
    j_newcomer_t = float(jatt['tension'][-1][3])
    j_elev = float(np.mean(jatt['elev3'][-10:]))
    j_az = float(np.mean(jatt['az3'][-10:]))
    j_az_err = min(abs(j_az - 180.0), 360.0 - abs(j_az - 180.0))
    j_aloft_ok = bool(j_post.min() > TARGET_Z - 0.20 and np.all(np.isfinite(jatt['load_z'])))
    j_level_ok = bool(j_max_tilt < 12.0 and j_final_tilt < 8.0)
    j_handout_ok = bool(j_handout > 0.999)
    j_ring_ok = bool(j_newcomer_t > 0.3 and 40.0 < j_elev < 55.0 and j_az_err < 20.0)
    j_ok = j_aloft_ok and j_level_ok and j_handout_ok and j_ring_ok
    results.append(('J soft hand-out settles level', j_ok,
                    f'gradual weld: max tilt {j_max_tilt:.1f}deg (<12), settled '
                    f'{j_final_tilt:.1f}deg (<8), hand-out {j_handout:.3f} complete, newcomer '
                    f'{j_newcomer_t:.2f} N @ elev {j_elev:.0f}deg az {j_az:.0f}deg '
                    f'(err {j_az_err:.0f}deg)'))
    all_ok = all_ok and j_ok

    # K: NON-BALANCED even respace -- the 3 tethered members open from their reduced-fleet even
    # splay to a full even 4-gon to make room for the newcomer, and (the fix) they do so
    # CONTINUOUSLY: the weighted slot spacing eases with the newcomer's hand-out instead of
    # stepping the azimuth targets 3-gon->4-gon in one tick. Contrast an INSTANT weld (targets
    # step, the existing members are yanked) with the eased hand-out, and check the eased
    # respace BOTH converges to an even 4-gon AND moves the existing members far more gently.
    def _existing_az_rate(h, tick):
        qaz = h['qaz_all'][tick:, :3]                     # commanded az of members 0,1,2
        rate = 0.0
        for c in range(3):
            a = np.unwrap(np.radians(qaz[:, c]))
            if a.size > 1:
                rate = max(rate, float(np.nanmax(np.abs(np.degrees(np.diff(a))))))
        return rate                                       # peak deg per control tick

    def _even_gaps(h):
        az = np.sort(h['qaz_all'][-1])                    # 4 final azimuths, ascending
        return np.diff(np.concatenate([az, [az[0] + 360.0]]))   # 4 adjacent gaps, sum 360

    k_inst, ki = run_attach(attach_t=4.0, sim_t=20.0, handout=False)
    k_soft, ks = run_attach(attach_t=4.0, sim_t=24.0,
                            params=DissipativeParams(T_handout=10.0), handout=True)
    k_inst_rate = _existing_az_rate(k_inst, ki)
    k_soft_rate = _existing_az_rate(k_soft, ks)
    k_gaps = _even_gaps(k_soft)
    k_post = k_soft['load_z'][ks:]
    k_even_ok = bool(np.all(k_gaps > 70.0) and np.all(k_gaps < 110.0))
    # the RATIO is the real claim (eased respace is far gentler than the stepped one); the
    # absolute bound is a loose sanity cap, not the discriminator.
    k_smooth_ok = bool(k_soft_rate < 0.5 * k_inst_rate and k_soft_rate < 4.0)
    k_aloft_ok = bool(k_post.min() > TARGET_Z - 0.20 and np.all(np.isfinite(k_soft['load_z'])))
    k_handout_ok = bool(k_soft['handout3'][-1] > 0.999)
    k_ok = k_even_ok and k_smooth_ok and k_aloft_ok and k_handout_ok
    results.append(('K even respace is smooth', k_ok,
                    f'settled gaps {np.array2string(k_gaps, precision=0)} deg (~90 even); '
                    f'existing-member peak az rate: instant {k_inst_rate:.2f} -> eased '
                    f'{k_soft_rate:.2f} deg/tick; load aloft, hand-out complete'))
    all_ok = all_ok and k_ok

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
