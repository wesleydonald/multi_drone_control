#!/usr/bin/env python3
"""
numerical_sim/simulate.py
=========================
Numerical simulation of a cable-suspended payload carried by N quadrotors.

Directly integrates the load-cable dynamic model from:
  Sun et al., "Agile and Cooperative Aerial Manipulation of a Cable-Suspended Load"
  arXiv:2501.18802v2  (Equations 1-3)

This is how TU Delft validated their work in simulation — not Gazebo, but a
numerical integration of the equations of motion with a simple point-mass model.

Physics:
  - Payload:  point mass (rotation ignored in this first version)
  - Drones:   point masses with direct force vectors (attitude abstracted away)
  - Cables:   tension-only spring-damper; high K approximates inextensible
  - Integrator: RK4 at 1 kHz, controller at 30 Hz

Controller implemented here: the quasi-static PD baseline (equivalent to the
"Geometric" controller that TU Delft used as a comparison baseline in Table 1).
Cables are assumed vertical for setpoint computation (quasi-static assumption).

Usage:
  python simulate.py                      # hover
  python simulate.py --traj slow          # figure-eight, v_max ~ 1 m/s
  python simulate.py --traj medium        # v_max ~ 2 m/s
  python simulate.py --traj medium_plus   # v_max ~ 2 m/s, a_max ~ 8 m/s^2 (baseline struggles)
  python simulate.py --traj fast          # v_max ~ 5 m/s  (baseline crashes)
  python simulate.py --traj all           # run all and compare (like Table 1)
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D projection)

# ── Physical parameters ────────────────────────────────────────────────────────
N      = 4          # quadrotors
M_Q    = 0.6        # kg  drone mass
M_L    = 0.4        # kg  payload mass
G      = 9.81       # m/s²
L_CAB  = 1.0        # m   cable natural length (paper uses 1 m)

# Cable spring-damper: high K  ≈  inextensible (tension-only).
# At hover, equilibrium extension ≈ M_L*G / (N * K * cos²θ) ≈ 0.2 mm — negligible.
# Choose slightly over-damped: D_crit = 2*sqrt(K * M_L / N) ≈ 89 Ns/m.
K_CAB  = 5000.0     # N/m
D_CAB  = 100.0      # Ns/m

# Drone max thrust (N) and minimum upward component.
F_MAX   = 3.0 * (M_Q + M_L / N) * G   # generous ceiling
F_ZMIN  = 0.05 * M_Q * G

# Formation: horizontal offsets of cables from payload CoM (square, symmetric).
# Using smaller radius so cable angle from vertical is ~15°, matching hover geometry.
R_FORM = 0.25       # m — half-diagonal of formation square
OFFSETS_XY = R_FORM / np.sqrt(2) * np.array([
    [ 1.,  1.],
    [-1.,  1.],
    [-1., -1.],
    [ 1., -1.],
])   # shape (N, 2)

# Pre-compute the "up" direction for each drone's cable in hover:
# raw offset from payload in 3D = [ox, oy, L_CAB]; normalise → sphere point.
_raw_dirs = np.column_stack([OFFSETS_XY,
                              np.full(N, L_CAB)])           # (N, 3)
_norms    = np.linalg.norm(_raw_dirs, axis=1, keepdims=True)
HOVER_DIRS = _raw_dirs / _norms                             # (N, 3) unit vectors

# ── Controller gains ───────────────────────────────────────────────────────────
KP = np.array([2.5, 2.5, 4.0])
KD = np.array([3.0, 3.0, 4.5])

# ── Target payload hover position ─────────────────────────────────────────────
Z_HOVER = 1.5       # m

# ── Figure-eight trajectory parameters ────────────────────────────────────────
T_HOVER  = 5.0      # s — hover before trajectory starts
AMP      = 2.0      # m — amplitude (matches paper Fig. 2 scale ~3 m)

# (omega, v_max, a_max) — matching Table 1 agility levels
TRAJS = {
    'hover':        (0.00, 0.0, 0.0),
    'slow':         (0.50, 1.0, 0.5),
    'medium':       (1.00, 2.0, 2.0),
    'medium_plus':  (1.50, 2.0, 8.0),
    'fast':         (2.50, 5.0, 8.0),
}

# ── Simulation timing ──────────────────────────────────────────────────────────
DT       = 0.001    # s   physics step (1 ms)
CTRL_HZ  = 30       # Hz  controller update rate (matches paper)
LOG_HZ   = 50       # Hz  data logging rate
T_TOTAL  = 35.0     # s   (matches paper Fig. 2 time axis)

CRASH_Z  = -0.5     # m   declare crash if any body falls below this


# ══════════════════════════════════════════════════════════════════════════════
#  Reference trajectory
# ══════════════════════════════════════════════════════════════════════════════

def payload_ref(t: float, omega: float):
    """
    Smooth Lissajous figure-eight for the payload CoM:
        x(τ) = A·sin(ωτ)
        y(τ) = A·sin(ωτ)·cos(ωτ)  =  A/2·sin(2ωτ)
        z    = Z_HOVER
    where τ = t − T_HOVER.

    Returns (p_ref, v_ref, a_ref), each shape (3,).
    During the initial hover phase (t < T_HOVER) or for omega=0 returns origin.
    """
    if omega == 0.0 or t < T_HOVER:
        return (np.array([0., 0., Z_HOVER]),
                np.zeros(3),
                np.zeros(3))

    tau = t - T_HOVER
    w   = omega
    s, c  = np.sin(w * tau),   np.cos(w * tau)
    s2, c2 = np.sin(2*w*tau),  np.cos(2*w*tau)

    p = np.array([AMP * s,       0.5 * AMP * s2,  Z_HOVER])
    v = np.array([AMP*w * c,     AMP*w * c2,       0.])
    a = np.array([-AMP*w**2 * s, -2*AMP*w**2 * s2, 0.])
    return p, v, a


# ══════════════════════════════════════════════════════════════════════════════
#  Drone setpoints  (quasi-static, sphere-projected)
# ══════════════════════════════════════════════════════════════════════════════

def drone_setpoints(p_ref, v_ref, a_ref):
    """
    Compute desired (position, velocity, acc) for each drone under the
    quasi-static cable assumption (paper's "Geometric" baseline):
      - Cables assumed vertical → drone target = payload target + [0,0,L_CAB]
        plus the horizontal formation offset.
      - Projected onto sphere |p_i - p_L| = L_CAB so the setpoint is always
        geometrically reachable even when the cable is not vertical.

    Returns p_des (N,3), v_des (N,3), a_des_ff (N,3).
    """
    directions = HOVER_DIRS                      # fixed (quasi-static)
    p_des  = p_ref[None, :] + L_CAB * directions   # (N, 3)
    v_des  = np.tile(v_ref,  (N, 1))
    a_ff   = np.tile(a_ref,  (N, 1))
    return p_des, v_des, a_ff


# ══════════════════════════════════════════════════════════════════════════════
#  Cable physics  (vectorised)
# ══════════════════════════════════════════════════════════════════════════════

def cable_forces(p_L, v_L, p_d, v_d):
    """
    Tension-only spring-damper cable model.

    Sign convention (matches paper Eq. 2):
      s_i  = unit vector FROM drone i TO payload  =  (p_L - p_d[i]) / dist
      Cable force on payload = -t_i * s_i  (toward drone  = upward when taut)
      Cable force on drone   = +t_i * s_i  (toward payload = downward)

    p_L : (3,)   payload position
    v_L : (3,)   payload velocity
    p_d : (N,3)  drone positions
    v_d : (N,3)  drone velocities

    Returns
      tensions  (N,)   scalar cable tensions [N]
      f_payload (3,)   net cable force on payload [N]
      f_drones  (N,3)  cable force on each drone [N]
    """
    diff  = p_L - p_d                                  # (N, 3)  drone→payload
    dist  = np.linalg.norm(diff, axis=1)               # (N,)

    valid = dist > 1e-9
    s     = np.where(valid[:, None], diff / np.maximum(dist[:, None], 1e-9), 0.)

    ext   = dist - L_CAB                               # (N,)
    rate  = np.einsum('ij,ij->i', s, v_L - v_d)       # (N,) stretch rate

    taut  = valid & (ext > 0.)
    t_raw = K_CAB * ext + D_CAB * rate
    t     = np.where(taut, np.maximum(0., t_raw), 0.)  # (N,) tensions

    f_payload = -np.einsum('i,ij->j', t, s)            # (3,)
    f_drones  =  t[:, None] * s                        # (N,3)
    return t, f_payload, f_drones


# ══════════════════════════════════════════════════════════════════════════════
#  PD controller  (Geometric baseline)
# ══════════════════════════════════════════════════════════════════════════════

def pd_control(t_sim, p_L, v_L, p_d, v_d, omega):
    """
    Per-drone PD position controller with cable tension feedforward.

    Equivalent to the "Geometric" baseline evaluated in the paper (Table 1).
    Each drone assumes cables hang vertically and accounts for its share of
    the payload weight as a downward external force.

    Returns F (N,3) — thrust vectors [N].
    """
    p_ref, v_ref, a_ref = payload_ref(t_sim, omega)
    p_des, v_des, a_ff  = drone_setpoints(p_ref, v_ref, a_ref)

    ff_z = M_L * G / N                                # cable tension share [N]

    ep  = p_des - p_d                                 # (N,3)
    ev  = v_des - v_d                                 # (N,3)
    a   = KP * ep + KD * ev + a_ff                    # (N,3) desired accel

    # Thrust = m*(a + g) + cable feedforward
    F   = M_Q * (a + np.array([0., 0., G])) + np.array([0., 0., ff_z])

    # Clamp magnitude and enforce minimum upward component
    mag = np.linalg.norm(F, axis=1, keepdims=True)
    F   = np.where(mag > F_MAX, F * (F_MAX / mag), F)
    F[:, 2] = np.maximum(F[:, 2], F_ZMIN)
    return F


# ══════════════════════════════════════════════════════════════════════════════
#  Equations of motion  (Eq. 2 simplified — point-mass payload)
# ══════════════════════════════════════════════════════════════════════════════

def eom(x, F):
    """
    State vector x = [p_L(3), v_L(3), p_0(3), v_0(3), ..., p_3(3), v_3(3)]
    Returns dx/dt with same layout.

    Payload dynamics (Eq. 2, point-mass, load rotation not yet included):
      p_L_dot = v_L
      v_L_dot = g - Σ(t_i * s_i) / m_L

    Drone dynamics (point mass, thrust F_i):
      p_i_dot = v_i
      v_i_dot = g + F_i/m_q + t_i*s_i/m_q   (cable pulls drone toward payload)
    """
    p_L = x[0:3];  v_L = x[3:6]
    p_d = x[6:].reshape(N, 6)[:, :3]
    v_d = x[6:].reshape(N, 6)[:, 3:]

    _, f_payload, f_drones = cable_forces(p_L, v_L, p_d, v_d)

    dx = np.empty_like(x)
    dx[0:3] = v_L
    dx[3:6] = np.array([0., 0., -G]) + f_payload / M_L

    drone_block = dx[6:].reshape(N, 6)
    drone_block[:, :3] = v_d
    drone_block[:, 3:] = (np.array([0., 0., -G])
                          + F / M_Q
                          + f_drones / M_Q)
    return dx


# ══════════════════════════════════════════════════════════════════════════════
#  RK4 integrator
# ══════════════════════════════════════════════════════════════════════════════

def rk4(x, F, dt):
    k1 = eom(x,           F)
    k2 = eom(x + dt/2*k1, F)
    k3 = eom(x + dt/2*k2, F)
    k4 = eom(x + dt*k3,   F)
    return x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.


# ══════════════════════════════════════════════════════════════════════════════
#  Initial state: drones exactly on the cable-length sphere around payload
# ══════════════════════════════════════════════════════════════════════════════

def initial_state():
    """
    Payload at Z_HOVER, each drone at L_CAB along its hover direction.
    Cables start with zero extension → zero initial tension → no impulse.
    """
    x = np.zeros(6 + N * 6)
    x[0:3] = [0., 0., Z_HOVER]
    for i in range(N):
        x[6 + i*6 : 9 + i*6] = x[0:3] + L_CAB * HOVER_DIRS[i]
    return x


# ══════════════════════════════════════════════════════════════════════════════
#  Main simulation loop
# ══════════════════════════════════════════════════════════════════════════════

def simulate(omega: float, traj_name: str):
    x = initial_state()
    t = 0.

    ctrl_dt   = 1. / CTRL_HZ
    log_dt    = 1. / LOG_HZ
    next_ctrl = 0.
    next_log  = 0.

    # Pre-compute initial control (avoids re-computing at first step)
    p_d = x[6:].reshape(N, 6)[:, :3].copy()
    v_d = x[6:].reshape(N, 6)[:, 3:].copy()
    F   = pd_control(t, x[0:3], x[3:6], p_d, v_d, omega)

    log_t, log_pL, log_pd, log_tens, log_ref = [], [], [], [], []
    crashed = False

    n_steps = int(T_TOTAL / DT)
    for _ in range(n_steps):

        if t >= next_ctrl:
            p_d = x[6:].reshape(N, 6)[:, :3].copy()
            v_d = x[6:].reshape(N, 6)[:, 3:].copy()
            F   = pd_control(t, x[0:3], x[3:6], p_d, v_d, omega)
            next_ctrl += ctrl_dt

        if t >= next_log:
            p_d  = x[6:].reshape(N, 6)[:, :3].copy()
            v_d  = x[6:].reshape(N, 6)[:, 3:].copy()
            tens, _, _ = cable_forces(x[0:3], x[3:6], p_d, v_d)
            p_r, _, _  = payload_ref(t, omega)
            log_t.append(t)
            log_pL.append(x[0:3].copy())
            log_pd.append(p_d.copy())
            log_tens.append(tens.copy())
            log_ref.append(p_r)
            next_log += log_dt

        # Crash check
        if x[2] < CRASH_Z or np.any(x[6:].reshape(N, 6)[:, 2] < CRASH_Z):
            print(f"  CRASH at t={t:.2f}s  (payload z={x[2]:.3f} m)")
            crashed = True
            break

        x = rk4(x, F, DT)
        t += DT

    if not crashed:
        print(f"  Completed  t={t:.2f}s")

    times = np.array(log_t)
    p_Ls  = np.array(log_pL)          # (T, 3)
    p_ds  = np.array(log_pd)          # (T, N, 3)
    tens  = np.array(log_tens)        # (T, N)
    refs  = np.array(log_ref)         # (T, 3)

    # RMSE over trajectory portion
    if omega > 0 and np.any(times > T_HOVER):
        mask = times > T_HOVER
        err  = np.linalg.norm(p_Ls[mask] - refs[mask], axis=1)
        rmse = float(np.sqrt(np.mean(err**2)))
        status = 'CRASH' if crashed else f'RMSE={rmse:.3f} m'
        print(f"  Trajectory RMSE: {rmse:.4f} m  [{status}]")
    else:
        err  = np.linalg.norm(p_Ls - refs, axis=1)
        rmse = float(np.mean(err[-int(2/log_dt):]))
        status = f'RMSE={rmse:.3f} m'
        print(f"  Hover steady-state: {rmse:.4f} m")

    return times, p_Ls, p_ds, tens, refs, crashed, rmse


# ══════════════════════════════════════════════════════════════════════════════
#  Plotting
# ══════════════════════════════════════════════════════════════════════════════

DRONE_COLORS = ['#e74c3c', '#2ecc71', '#3498db', '#9b59b6']

def plot_single(times, p_Ls, p_ds, tens, refs, crashed, rmse, traj_name, omega):
    err    = np.linalg.norm(p_Ls - refs, axis=1)
    status = 'CRASH' if crashed else f'RMSE = {rmse:.3f} m'

    fig = plt.figure(figsize=(15, 8))
    fig.suptitle(f"Cable-payload PD baseline  —  '{traj_name}'  [{status}]",
                 fontsize=12, fontweight='bold')

    # 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(refs[:, 0], refs[:, 1], refs[:, 2],
             'k--', lw=1, label='Reference')
    ax1.plot(p_Ls[:, 0], p_Ls[:, 1], p_Ls[:, 2],
             'b-', lw=1.5, label='Payload')
    for i in range(N):
        ax1.plot(p_ds[:, i, 0], p_ds[:, i, 1], p_ds[:, i, 2],
                 color=DRONE_COLORS[i], lw=0.5, alpha=0.4)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_title('3D trajectories')
    ax1.legend(fontsize=7)

    # XY top view
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(refs[:, 0], refs[:, 1], 'k--', lw=1, label='Ref.')
    ax2.plot(p_Ls[:, 0], p_Ls[:, 1], 'b-', lw=1.5, label='Payload')
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_title('Top view (XY)'); ax2.set_aspect('equal')
    ax2.legend(fontsize=7); ax2.grid(alpha=0.3)

    # Position error vs time
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(times, err, 'b-', lw=0.8)
    if omega > 0:
        ax3.axvline(T_HOVER, color='gray', ls=':', lw=0.8, label='Traj. start')
        ax3.legend(fontsize=7)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('|error| (m)')
    ax3.set_title('Payload position error'); ax3.grid(alpha=0.3)

    # Cable tensions
    ax4 = fig.add_subplot(2, 3, 4)
    for i in range(N):
        ax4.plot(times, tens[:, i], color=DRONE_COLORS[i], lw=0.7, label=f'Cable {i}')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Tension (N)')
    ax4.set_title('Cable tensions'); ax4.legend(fontsize=7); ax4.grid(alpha=0.3)

    # Altitude vs time
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(times, refs[:, 2], 'k--', lw=0.8, label='Ref. z')
    ax5.plot(times, p_Ls[:, 2], 'b-',  lw=1,   label='Payload z')
    for i in range(N):
        ax5.plot(times, p_ds[:, i, 2], color=DRONE_COLORS[i], lw=0.5, alpha=0.4)
    ax5.set_xlabel('Time (s)'); ax5.set_ylabel('z (m)')
    ax5.set_title('Altitude vs time'); ax5.legend(fontsize=7); ax5.grid(alpha=0.3)

    # Formation snapshot at t = T_HOVER + 3s  (mid-trajectory)
    ax6 = fig.add_subplot(2, 3, 6)
    snap_t = T_HOVER + 3.0
    snap_i = int(snap_t * LOG_HZ)
    snap_i = min(snap_i, len(times) - 1)
    for i in range(N):
        ax6.scatter(p_ds[snap_i, i, 0], p_ds[snap_i, i, 1],
                    color=DRONE_COLORS[i], s=80, zorder=3)
        ax6.plot([p_Ls[snap_i, 0], p_ds[snap_i, i, 0]],
                 [p_Ls[snap_i, 1], p_ds[snap_i, i, 1]],
                 color='gray', lw=0.8, alpha=0.6)
    ax6.scatter(p_Ls[snap_i, 0], p_Ls[snap_i, 1],
                c='blue', s=120, marker='s', zorder=4, label='Payload')
    ax6.scatter(refs[snap_i, 0], refs[snap_i, 1],
                c='black', s=60, marker='x', zorder=5, label='Ref.')
    ax6.set_xlabel('X (m)'); ax6.set_ylabel('Y (m)')
    ax6.set_title(f'Formation (t={times[snap_i]:.1f}s)')
    ax6.set_aspect('equal'); ax6.legend(fontsize=7); ax6.grid(alpha=0.3)

    plt.tight_layout()
    fname = f'numerical_sim/sim_{traj_name}.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    print(f"  Saved  {fname}")
    plt.close()
    return fname


def plot_table1_comparison(results):
    """
    Bar chart replicating the style of Table 1 from the paper.
    results: dict  traj_name → (crashed, rmse)
    """
    names  = [n for n in TRAJS if n != 'hover' and n in results]
    rmses  = []
    colors = []
    for n in names:
        crashed, rmse = results[n]
        rmses.append(np.nan if crashed else rmse)
        colors.append('#e74c3c' if crashed else '#3498db')

    fig, ax = plt.subplots(figsize=(8, 4))
    bars = ax.bar(names, rmses, color=colors, edgecolor='black', linewidth=0.6)
    for bar, (n, (crashed, rmse)) in zip(bars, [(n, results[n]) for n in names]):
        label = 'CRASH' if crashed else f'{rmse:.3f} m'
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.005 if not crashed else 0.02,
                label, ha='center', va='bottom', fontsize=9)

    # Reference: paper's Table 1 Geometric baseline values
    paper_geo = {'slow': 0.032, 'medium': 0.135, 'medium_plus': None, 'fast': None}
    ref_vals  = [paper_geo.get(n) for n in names]
    ax.scatter(range(len(names)),
               [v if v is not None else 0 for v in ref_vals],
               color='black', marker='_', s=200, linewidths=2,
               zorder=5, label='Paper (Geometric baseline)')

    ax.set_xlabel('Trajectory agility'); ax.set_ylabel('Payload position RMSE (m)')
    ax.set_title('Simulation results vs paper Table 1 (Geometric baseline)\n'
                 'Blue = completed, Red = crashed  |  — = paper reference')
    ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
    ax.set_ylim(bottom=0)

    fname = 'numerical_sim/sim_table1.png'
    plt.tight_layout()
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    print(f"  Saved  {fname}")
    plt.close()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Cable-suspended payload numerical simulation')
    parser.add_argument('--traj', default='hover',
                        choices=list(TRAJS.keys()) + ['all'],
                        help='Trajectory type (default: hover)')
    args = parser.parse_args()

    if args.traj == 'all':
        results = {}
        for name, (omega, vmax, amax) in TRAJS.items():
            if name == 'hover':
                continue
            label = f"{name}  (v_max={vmax} m/s, a_max={amax} m/s²)"
            print(f"\n{'='*60}")
            print(f"  {label}")
            print(f"{'='*60}")
            times, p_Ls, p_ds, tens, refs, crashed, rmse = simulate(omega, name)
            plot_single(times, p_Ls, p_ds, tens, refs, crashed, rmse, name, omega)
            results[name] = (crashed, rmse)

        print(f"\n{'='*60}")
        print("  Table 1 comparison summary")
        print(f"{'='*60}")
        print(f"  {'Trajectory':<15} {'RMSE':>10}  {'Status':>10}")
        print(f"  {'-'*40}")
        for name, (crashed, rmse) in results.items():
            status = 'CRASH' if crashed else 'OK'
            rmse_s = '  ---   ' if crashed else f'{rmse:.4f} m'
            print(f"  {name:<15} {rmse_s:>10}  {status:>10}")
        plot_table1_comparison(results)

    else:
        omega, vmax, amax = TRAJS[args.traj]
        print(f"\nTrajectory: {args.traj}")
        if vmax > 0:
            print(f"  v_max={vmax} m/s  a_max={amax} m/s²")
        times, p_Ls, p_ds, tens, refs, crashed, rmse = simulate(omega, args.traj)
        plot_single(times, p_Ls, p_ds, tens, refs, crashed, rmse, args.traj, omega)


if __name__ == '__main__':
    main()
