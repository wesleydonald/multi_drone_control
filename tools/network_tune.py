#!/usr/bin/env python3
"""tools/network_tune.py -- the offline harness of result C (card docs/experiments/2026-10-23_paper_method_compare.md).

The paper's method as the twin's network mode flies it -- the real DissipativeNetwork stepped at 10 Hz as
dissipative_node._network_plan steps it (no preview), feeding the real VelocityLoop at 50 Hz through the trackers'
affine-throttle plumbing -- on the SIL plant at C's geometry: 4 quads at 0/90/180/270 deg on the ring's 0.225 m
radius, 1 m rods at 60 deg, the 0.86 kg ring, 0.55 kg quads, the rig's thrust map with each twin quad's own offset
and a sagging pack, the mocap velocity filter at 120 Hz. Scored by the functions tools/c_metrics.py uses.

    python3 tools/network_tune.py --quick                # sanity pass: both forms at today's gains, hover + 2 lines
                                                         # (the claim's cycles: 4 at 0.2, 8 at 0.4)
    python3 tools/network_tune.py --search [--structure port|paper|both] [--workers 6]
                                                         # the card's pre-registered retune (grid, then descent)
    python3 tools/network_tune.py --gains diss_c=4.5,vel_kv=5 --structure paper    # one candidate
    python3 tools/network_tune.py --emit p1 --structure paper --gains ...          # its twin confirmation configs
    python3 tools/network_tune.py --bench                # compute per tick: the step, + a 21-node preview, the loop
    python3 tools/network_tune.py --search --structure quan --workers 4
                                                         # Quan et al. as printed (quan_law.py) on world C: the hub
                                                         # at the ring's CoM, rods at the drone centres (card
                                                         # docs/experiments/2026-10-23_quan_exact.md, its search)

What it cannot show (tools/velocity_probe.py, docs/design/sil_bench.md §7): contact, aerodynamics, mocap noise and
latency, the rods' mass, the 4 cm pivot (its rods hang from the drone centres), our OCP's lift (it starts settled on
the taut cone, the hand-over's state). It ranks candidates; the twin decides.
"""
import argparse
import datetime
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
for pkg in ('mpc_planner', 'dissipative_planner', 'tracker', 'drone_communication'):
    sys.path.insert(0, os.path.join(REPO, 'src', pkg))

import metrics as M  # noqa: E402
from dissipative_planner.dissipative_network import DissipativeNetwork, DissipativeParams  # noqa: E402
from dissipative_planner.quan_law import QuanLaw, QuanParams  # noqa: E402
from drone_communication.velocity_filter import VelocityFilter  # noqa: E402
from mpc_planner.geometry import attach_points, thrust_attitude  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from sil.plant import Link, PayloadParams, QuadParams, SilPlant, rot_to_quat  # noqa: E402
from tracker.velocity_loop import VelocityLoop  # noqa: E402

# C's geometry and the twin's plant (configs/experiments/c_*_L100.yaml)
N, L, ELEV_DEG = 4, 1.0, 60.0
ATTACH_R, ATTACH_Z = 0.225, 0.02
M_LOAD, M_QUAD = 0.86, 0.55
Z_TARGET, LINE_M = 1.0, 0.9
KT, OFFSET, OFFSET_SLOPE, V_REF, THR_MAX = 35.2, 0.185, 0.022, 23.5, 0.8
PLANT_OFFSETS = (0.181, 0.185, 0.187, 0.187)    # sim_thrust_offset: each twin quad's own
PACK_V0 = 23.8
NET_DT, CTRL_DT, MOCAP_DT, SIM_DT = 0.1, 0.02, 1.0 / 120.0, 0.001
HOLD_S, TAIL_S, HOVER_S, FIRST_S = 4.0, 2.0, 60.0, 5.0
# the ring starts this far off the formation in yaw: a perfectly symmetric start never excites the ring's yaw mode,
# which grows under the port at C's geometry (22 Oct: from 0.5 deg to a capsize in 15 s at today's gains, in this
# harness and in the design's own probe)
YAW_KICK_DEG = 0.5

# today's twin network mode (sim.yaml modes.network, common.yaml, the trackers' defaults)
DEFAULTS = dict(diss_c=6.0, diss_k_slot=18.0, diss_k_pay=40.0, diss_k_ring=20.0, vel_kp_pos=2.0, vel_kv=4.0,
                vel_k_att=8.0, diss_ki_load=1.0)
STRUCTURES = {'port': dict(net_damp_relative=False, net_traj_lean=False),
              'paper': dict(net_damp_relative=True, net_traj_lean=True)}
# the card's pre-registered search: a grid over the two knobs the ring's yaw mode turns on (stage 1), then coordinate
# descent over all of these, in this order, two passes from the grid's best (stage 2)
SPACE = [('diss_c', (3.0, 4.5, 6.0, 9.0, 12.0)), ('diss_k_slot', (9.0, 18.0, 36.0)),
         ('diss_k_pay', (20.0, 40.0, 80.0)), ('diss_k_ring', (10.0, 20.0, 40.0)),
         ('vel_kp_pos', (1.0, 1.5, 2.0, 2.5)), ('vel_kv', (3.0, 4.0, 5.0)), ('vel_k_att', (6.0, 8.0, 12.0, 16.0)),
         ('diss_ki_load', (0.5, 1.0, 2.0))]
GRID = ('diss_c', 'vel_kp_pos')
# Quan et al. as printed (card 2026-10-23_quan_exact): world C's attachment (the hub at the ring's centre of mass, the
# rods from the drone centres, as this plant hangs them), the law's coefficients and the stand-in autopilot's two gains
# (vel_kp_pos 0: a pure velocity command). One structure per reading of the node (R1) and the velocity command (R7)
GEOMETRY = {'rim': (0.225, 0.02), 'hubcom': (0.001, 0.0)}
QUAN_READINGS = {'quan': ('algebraic', 'measured'), 'quan_dyn': ('dynamic', 'measured'),
                 'quan_r7': ('algebraic', 'integrated'), 'quan_dyn_r7': ('dynamic', 'integrated'),
                 'quan_r7c': ('algebraic', 'feedforward')}
# the start point: init.m's coefficients (k 10 per edge, c 10, its friction gain 5) on its 1 kg nodes, scaled by our
# quad's 0.55 kg; init.m's lengths are ours (1 m cables), the node plane 3.41 m over the load (R4). The search: a grid
# over the springs and dampers among the nodes (QUAN_GRID), then one descent pass over QUAN_SPACE (x0.5, x1, x2)
QUAN_DEFAULTS = dict(quan_k_ij=5.5, quan_c_ij=5.5, quan_k_i=5.5, quan_c_i=5.5, quan_fc=2.75, quan_hc_m=3.41,
                     quan_node_mass=1.0, vel_kv=4.0, vel_k_att=8.0, quan_rod_mass=0.0, quan_aexp_zero=0.0,
                     quan_centre_dyn=0.0, quan_centre_mass=2.0, quan_centre_kp=1.0, quan_centre_kd=5.0)
# quan_rod_mass: the rod's load-end share in the pull's horizontal part (R6c; 0 exact on this plant's massless rods,
# 0.055 with twin_rods); quan_aexp_zero 1: no a_expected (R5 'zero'); quan_centre_dyn 1: the dynamic centre (R3
# 'dynamic') at quan_centre_mass/kp/kd. None is searched.
QUAN_LAND_VEL, QUAN_LAND_Z = 0.2, 0.06          # the network's LAND ramp (land_vel, net_land_z)
# world C's rods as the twin has them (make_carry_worlds.py): 0.040 kg along the rod and a 0.035 kg magnet at the
# payload end, inside drone_mass 0.55. With fly(twin_rods=True) the plant's quads are the 0.475 kg airframe plus half the
# rod and the payload carries the other half and the magnets: the twin's statics on these massless rods
TWIN_ROD_KG, TWIN_MAGNET_KG = 0.040, 0.035
QUAN_SPACE = [(k, tuple(f * QUAN_DEFAULTS[k] for f in (0.5, 1.0, 2.0)))
              for k in ('quan_k_ij', 'quan_c_ij', 'quan_k_i', 'quan_c_i', 'quan_fc')] + \
    [('vel_kv', (3.0, 4.0, 5.0)), ('vel_k_att', (8.0, 12.0, 16.0))]
QUAN_GRID = ('quan_k_ij', 'quan_c_ij')
# --quan-start low: the second start (card §3.3), the low point R7b held calm in its 20 s hover (discriminator c)
QUAN_LOW_START = dict(QUAN_DEFAULTS, quan_k_ij=0.25, quan_c_ij=0.25, quan_k_i=2.0, quan_c_i=1.0, quan_fc=0.25)
QUAN_CASES = (('hover', 0.0), ('line_x', 0.4))
QUAN_RANGE_M = 2.0                            # quan_law.py R8
BAND_M = 0.30                                 # world C's loss rule (c_metrics BAND_LOSS_M)
CASES = (('hover', 0.0), ('line_x', 0.2), ('line_x', 0.4), ('line_x', 0.6))
QUICK = (('hover', 0.0), ('line_x', 0.2), ('line_x', 0.4))
# the claim's whole cycles (the v600 neighbour's at 0.6), so the search sees the claim's window (the critic, M4)
CYCLES = {0.2: 4, 0.4: 8, 0.6: 8}
# guards (the card): a candidate failing any is out whatever its objective
GUARD_RING_TILT, GUARD_QUAD_TILT, GUARD_THR, GUARD_YAW = 15.0, 35.0, 0.78, 10.0
GUARD_YAW_GROWTH = 0.01           # deg/s: the ring yaw's envelope may not grow (yaw_growth), within the harness's floor
GUARD_MARGIN = 1.25               # still stable with kp_pos and kv both this much higher, on line_x 0.4
GUARD_HOVER_MM = 2.0              # hover no worse than the structure's start by more than this


def offset_now(v_meas):
    """The trackers' throttle offset at the measured pack voltage (thrust_offset + slope (V_REF - V))."""
    return OFFSET + OFFSET_SLOPE * (V_REF - v_meas)


def params_of(knobs, structure):
    k = {**DEFAULTS, **(knobs or {}), **STRUCTURES[structure]}
    return k, DissipativeParams(k_pay=k['diss_k_pay'], k_anchor=k['diss_k_pay'], k_ring=k['diss_k_ring'],
                                c=k['diss_c'], k_slot=k['diss_k_slot'], elev_deg=ELEV_DEG, ki_load=k['diss_ki_load'],
                                damp_relative=k['net_damp_relative'])


def quan_knobs(knobs=None):
    """The quan structures' knobs over QUAN_DEFAULTS, with the stand-in autopilot's position term off (R7)."""
    return {**QUAN_DEFAULTS, **(knobs or {}), 'vel_kp_pos': 0.0, 'net_traj_lean': False, 'net_damp_relative': False}


def quan_law(k, structure='quan'):
    nodes, vcmd = QUAN_READINGS[structure]
    return QuanLaw(N, M_QUAD, M_LOAD, QuanParams(k_ij=k['quan_k_ij'], c_ij=k['quan_c_ij'], k_i=k['quan_k_i'],
                                                 c_i=k['quan_c_i'], f_c=k['quan_fc'], hc_m=k['quan_hc_m'],
                                                 range_m=QUAN_RANGE_M, nodes=nodes, vcmd=vcmd,
                                                 node_mass=k['quan_node_mass'], rod_mass=k['quan_rod_mass'],
                                                 aexp='zero' if k['quan_aexp_zero'] else 'path',
                                                 centre='dynamic' if k['quan_centre_dyn'] else 'path',
                                                 centre_mass=k['quan_centre_mass'], centre_kp=k['quan_centre_kp'],
                                                 centre_kd=k['quan_centre_kd']))


def fly(case, speed, knobs=None, structure='port', vel_filter='ema', cycles=None, hover_s=HOVER_S, geometry=None,
        land_at=None, twin_rods=False):
    """One flight from the hand-over (t = 0), settled on the taut 60 deg cone at Z_TARGET: a hover of hover_s, or a
    HOLD_S hover then `cycles` whole cycles of the line (default: the claim's, CYCLES). Returns time series at 50 Hz,
    or {'diverged': t}. geometry rim (C's plates) or hubcom (world C, the quan structure's). land_at (quan, s): from
    then the held height ramps to the floor as the network's LAND does, the node plane with it. twin_rods: the
    twin's rod masses in the plant (TWIN_ROD_KG); the controllers keep drone_mass and load_mass."""
    quan = structure in QUAN_READINGS
    k, p = (quan_knobs(knobs), None) if quan else params_of(knobs, structure)
    cycles = CYCLES.get(round(float(speed), 3), 4) if cycles is None else cycles
    rho = attach_points(N, *GEOMETRY[geometry or ('hubcom' if quan else 'rim')])
    load0 = np.array([0.0, 0.0, Z_TARGET])
    e = math.radians(ELEV_DEG)
    pos0 = [load0 + r + L * np.r_[math.cos(e) * r[:2] / np.linalg.norm(r[:2]), math.sin(e)] for r in rho]
    if quan:
        law, h_c = quan_law(k, structure), Z_TARGET + k['quan_hc_m']
        law.start(np.array(pos0), np.zeros((N, 3)), load0, load0, h_c)
        refs = [(pos0[i], *law.command(i, np.array(pos0), np.zeros((N, 3)), load0, np.zeros(3), load0, (0.0, 0.0),
                                       np.zeros(3), h_c, 0.0)[:3]) for i in range(N)]
    else:
        net = DissipativeNetwork(N, rho, L, M_QUAD, M_LOAD, 9.81, p)
        net.seed(pos0)
        refs = [net.reference(i, np.array([1.0, 0, 0, 0]), load0) for i in range(N)]
    # each quad starts at the attitude its share of the ring needs, so t = 0 is the settled hover the OCP hands over
    q0 = [rot_to_quat(thrust_attitude(refs[i][2])) for i in range(N)]
    m_q, m_l = ((M_QUAD - TWIN_MAGNET_KG - 0.5 * TWIN_ROD_KG, M_LOAD + N * (TWIN_MAGNET_KG + 0.5 * TWIN_ROD_KG))
                if twin_rods else (M_QUAD, M_LOAD))
    quads = [QuadParams(mass=m_q, thrust_map='rig', thrust_offset=PLANT_OFFSETS[i]) for i in range(N)]
    plant = SilPlant(quads, [Link(rho[i], L, attached=True) for i in range(N)], PayloadParams(mass=m_l),
                     pack={'v0': PACK_V0})
    kick = math.radians(YAW_KICK_DEG)
    plant.reset(pos0, load0, drone_quat=q0, load_R=np.array([[math.cos(kick), -math.sin(kick), 0.0],
                                                              [math.sin(kick), math.cos(kick), 0.0], [0.0, 0.0, 1.0]]))
    plant.armed = [True] * N
    loops = [VelocityLoop(kp_pos=k['vel_kp_pos'], kv=k['vel_kv'], ki=0.0, k_att=k['vel_k_att']) for _ in range(N)]
    vf = [VelocityFilter(vel_filter) for _ in range(N)]
    vring = VelocityFilter('ema')
    line = case == 'line_x'
    traj = LoadTrajectory(case, speed if line else 0.2, LINE_M, 0.5)
    T_path = cycles * math.pi * LINE_M / speed if line else 0.0
    sim_t = (HOLD_S + T_path if line else hover_s) + TAIL_S
    v_meas = [np.zeros(3)] * N
    v_load = np.zeros(3)
    cmd = []
    for i in range(N):
        thr = float(np.linalg.norm(refs[i][2])) / KT
        cmd.append((0.0, 0.0, min(THR_MAX, thr + offset_now(plant.pack_v_measured(i))), 0.0))
    traj_t, p_des = 0.0, load0.copy()
    every = {'net': round(NET_DT / SIM_DT), 'ctrl': round(CTRL_DT / SIM_DT)}
    next_mocap = 0.0
    rows = []
    for step in range(int(round(sim_t / SIM_DT))):
        t = step * SIM_DT
        if t + 1e-9 >= next_mocap:
            next_mocap += MOCAP_DT
            for i in range(N):
                vi = vf[i].update(t, plant.p[i].copy())         # copies: the filter keeps the last sample, the plant
                v_meas[i] = v_meas[i] if vi is None else vi     # moves its arrays in place
            vl = vring.update(t, plant.xL.copy())
            v_load = v_load if vl is None else vl
        if step % every['net'] == 0:
            ps = plant.payload_state()
            # the node's clock: the tick after the hold's end flies traj_t 0.1, so the schedule is x(t - HOLD_S)
            ran = line and t >= HOLD_S + NET_DT - 1e-9 and traj_t < T_path - 1e-9
            if ran:
                traj_t = min(traj_t + NET_DT, T_path)
            dx, dy, vx, vy = traj.offset_at(traj_t)
            z_was = p_des[2]
            if land_at is not None and t >= land_at:
                z_cmd = max(QUAN_LAND_Z, z_was - QUAN_LAND_VEL * NET_DT)
            else:
                z_cmd = z_was
            p_des = load0 + np.array([dx, dy, z_cmd - Z_TARGET])
            a = None
            if k['net_traj_lean']:
                ax, ay = traj.accel_at(traj_t)
                a = np.array([ax, ay, 0.0]) if math.hypot(ax, ay) > 1e-9 else None
            if quan:
                # the law at the node's 10 Hz from each quad's view: positions, the filtered velocities, the ring's
                x, xd = np.array(plant.p), np.array(v_meas)
                ax, ay = traj.accel_at(traj_t) if ran else (0.0, 0.0)
                h_c, hc_dot = p_des[2] + k['quan_hc_m'], (p_des[2] - z_was) / NET_DT
                law.advance_nodes(x, xd, p_des, (vx, vy) if ran else (0.0, 0.0), h_c, NET_DT, hc_dot=hc_dot,
                                  x_L=ps[0:3], xd_L=v_load)
                refs = [(x[i], *law.command(i, x, xd, ps[0:3], v_load, p_des, (vx, vy) if ran else (0.0, 0.0),
                                            np.array([ax, ay, 0.0]), h_c, NET_DT, hc_dot=hc_dot)[:3])
                        for i in range(N)]
            else:
                v_des = np.array([vx if ran else 0.0, vy if ran else 0.0, 0.0]) if k['net_damp_relative'] else None
                net.step(ps[0:3], ps[3:7], v_load, p_des, NET_DT, load_accel=a, p_des_vel=v_des)
                refs = [net.reference(i, ps[3:7], p_des, taut_gate=1.0, load_accel=a) for i in range(N)]
        if step % every['ctrl'] == 0:
            for i in range(N):
                st = plant.drone_state(i)
                off = offset_now(plant.pack_v_measured(i))
                loops[i].thr_max = THR_MAX - off
                pr, vr, aff, _ = refs[i]
                roll, pitch, thr, yaw = loops[i].step(st[0:3], v_meas[i], st[3:7], pr, vr, aff, CTRL_DT, KT)
                cmd[i] = (roll, pitch, min(THR_MAX, thr + off), yaw)
            rows.append((t, *p_des, *plant.xL, plant.load_tilt_deg(), max(plant.drone_tilt_deg(i) for i in range(N)),
                         max(c[2] for c in cmd), math.degrees(math.atan2(plant.RL[1, 0], plant.RL[0, 0]))))
        for i in range(N):
            c = cmd[i]
            plant.set_command(i, c[0], c[1], c[2] * 2.0 - 1.0, c[3], True)
        plant.step(SIM_DT)
        # world C's tilt is free: there a fall, not a tilt, is the divergence
        if step % 50 == 0 and (not plant.is_finite() or (plant.xL[2] < p_des[2] - 0.8 if quan
                                                          else plant.load_tilt_deg() > 80.0)):
            return {'diverged': round(t, 2), **_series(rows)}
    return {**_series(rows), 'case': case, 'speed': speed, 'T_path': T_path, 'hover_s': hover_s}


def _series(rows):
    a = np.array(rows)
    return {'t': a[:, 0], 'p_des': a[:, 1:4], 'act': a[:, 4:7], 'tilt': a[:, 7], 'quad_tilt': a[:, 8], 'thr': a[:, 9],
            'yaw': np.degrees(np.unwrap(np.radians(a[:, 10])))}


def score(r):
    """The case's numbers (mm, s, deg) on the c_metrics windows: the hover from the hand-over (t = 0, first 5 s apart),
    the line over its whole cycles from the path's start (HOLD_S)."""
    if 'diverged' in r:
        return {'diverged': r['diverged']}
    t, act, des = r['t'], r['act'], r['p_des']
    if r['case'] == 'hover':
        win = t <= r['hover_s']
        e = np.linalg.norm(act - des, axis=1)
        out = dict(err_rms_mm=1000 * _rms(e[win]), err_first5_rms_mm=1000 * _rms(e[t <= FIRST_S]),
                   band_max_mm=1000 * float(np.max(e[win])))
    else:
        v, d = r['speed'], LINE_M
        T = math.pi * d / v
        t0, t1 = HOLD_S, HOLD_S + r['T_path']
        win = (t >= t0) & (t <= t1)
        x0 = float(des[0, 0])
        cf = M.segment_error(act, x0, x0 + d, des[:, 1], des[:, 2], past_end=False)
        reach = np.array([x[2] for x in M.turnaround_reach(t, act[:, 0], t0, T, x0, x0 + d, t_end=t1)])
        w = 2.0 * v / d
        sched = x0 + 0.5 * d * (1.0 - np.cos(w * np.clip(t - t0, 0.0, None)))
        sched = np.where(t <= t1, sched, x0)
        e_t = np.linalg.norm(act - np.c_[sched, des[:, 1], des[:, 2]], axis=1)
        out = dict(cf_rms_mm=1000 * _rms(cf[win]), cf_max_mm=1000 * float(np.max(cf[win])),
                   reach_rms_mm=1000 * _rms(reach), reach_mean_mm=1000 * float(np.mean(reach)),
                   lag_s=float(M.phase_lag_s(t[win], sched[win], act[win, 0])), rmse_t_mm=1000 * _rms(e_t[win]),
                   amp_ratio=float(M.radius_ratio(act[:, :2], np.c_[sched, des[:, 1]], win)))
        out['headline_mm'] = out['cf_rms_mm'] + out['reach_rms_mm']
        out['band_max_mm'] = 1000 * float(np.max(M.segment_error(act, x0, x0 + d, des[:, 1], des[:, 2])[t <= t1]))
    out.update(tilt_mean_deg=float(np.mean(r['tilt'][win])), tilt_peak_deg=float(np.max(r['tilt'])),
               quad_tilt_peak_deg=float(np.max(r['quad_tilt'])), thr_peak=float(np.max(r['thr'])),
               yaw_peak_deg=float(np.max(np.abs(r['yaw'] - r['yaw'][0]))),
               yaw_growth_deg_s=yaw_growth(t[win], r['yaw'][win] - r['yaw'][0]))
    return out


def yaw_growth(t, yaw):
    """How fast the ring yaw's envelope grows (deg/s): the largest |yaw| in the window's last quarter minus that in
    its second, over the time between them. Each quarter holds whole line cycles, so a periodic swing cancels; a mode
    growing too slowly to reach the 10 deg cap inside the window still reads > 0 (the card's critic, M4)."""
    t, y = np.asarray(t, float), np.abs(np.asarray(yaw, float))
    if t.size < 8:
        return math.nan
    q = t[0] + (t[-1] - t[0]) * np.array([0.25, 0.5, 0.75, 1.0])
    second, last = (t >= q[0]) & (t < q[1]), (t >= q[2]) & (t <= q[3])
    return float((np.max(y[last]) - np.max(y[second])) / (0.5 * (q[2] + q[3]) - 0.5 * (q[0] + q[1])))


def _rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else math.nan


def case_name(case, speed):
    return 'hover' if case == 'hover' else f'linex_v{1000 * speed:.0f}'


def guards(results, hover_ref_mm=None, tilt_free=False):
    """The card's guards over one candidate's scored cases (and its 1.25x margin flight): [] when it passes. tilt_free
    (world C, card 2026-10-23_quan_exact): the ring's tilt and yaw are free, so the ring staying within BAND_M of the
    path replaces their guards."""
    bad = []
    for name, s in results.items():
        if 'diverged' in s:
            bad.append(f'{name} diverged at {s["diverged"]:.1f} s')
            continue
        if tilt_free:
            if s['quad_tilt_peak_deg'] > GUARD_QUAD_TILT:
                bad.append(f'{name} quad tilt {s["quad_tilt_peak_deg"]:.1f} deg')
            if s['thr_peak'] > GUARD_THR:
                bad.append(f'{name} throttle {s["thr_peak"]:.3f}')
            if s.get('band_max_mm', 0.0) > 1000 * BAND_M:
                bad.append(f'{name} ring {s["band_max_mm"]:.0f} mm off the path')
            continue
        if s['tilt_peak_deg'] > GUARD_RING_TILT:
            bad.append(f'{name} ring tilt {s["tilt_peak_deg"]:.1f} deg')
        if s['quad_tilt_peak_deg'] > GUARD_QUAD_TILT:
            bad.append(f'{name} quad tilt {s["quad_tilt_peak_deg"]:.1f} deg')
        if s['thr_peak'] > GUARD_THR:
            bad.append(f'{name} throttle {s["thr_peak"]:.3f}')
        if s['yaw_peak_deg'] > GUARD_YAW:
            bad.append(f'{name} ring yaw {s["yaw_peak_deg"]:.1f} deg')
        if s.get('yaw_growth_deg_s', 0.0) > GUARD_YAW_GROWTH:
            bad.append(f'{name} ring yaw growing {s["yaw_growth_deg_s"]:.3f} deg/s')
    h = results.get('hover')
    if hover_ref_mm is not None and h and 'err_rms_mm' in h and h['err_rms_mm'] > hover_ref_mm + GUARD_HOVER_MM:
        bad.append(f'hover {h["err_rms_mm"]:.1f} mm > {hover_ref_mm:.1f} + {GUARD_HOVER_MM:.0f}')
    return bad


def objective(results):
    """J (the card): the clock-free path error plus the reach at the ends, RMS, on line_x 0.4 (mm)."""
    s = results.get('linex_v400', {})
    return s.get('headline_mm', math.inf) if 'diverged' not in s else math.inf


def evaluate(args):
    """One candidate: every case, plus line_x 0.4 at 1.25x kp_pos and kv (the margin guard)."""
    knobs, structure, vel_filter, cases = args
    out = {}
    for case, speed in cases:
        out[case_name(case, speed)] = score(fly(case, speed, knobs, structure, vel_filter))
    if ('line_x', 0.4) in cases:
        k = quan_knobs(knobs) if structure in QUAN_READINGS else {**DEFAULTS, **knobs}
        hot = {**knobs, 'vel_kv': GUARD_MARGIN * k['vel_kv']}
        if structure not in QUAN_READINGS:                      # the quan structure flies no position term to scale
            hot['vel_kp_pos'] = GUARD_MARGIN * k['vel_kp_pos']
        out['margin_linex_v400'] = score(fly('line_x', 0.4, hot, structure, vel_filter))
    return knobs, structure, out


def search(structure, vel_filter, workers, passes=2, log=print, start=None):
    """The card's retune of one structure. Stage 1: every (diss_c, vel_kp_pos) of SPACE, the rest at DEFAULTS. Stage 2:
    coordinate descent over SPACE from stage 1's best, two passes: per knob every value with the rest held, a candidate
    kept when it ranks better (rank). When no grid point passes every guard, stage 2 starts from the one with the
    fewest guard failures (then the lowest J, then the latest divergence) and walks toward feasibility on the same
    rank. The hover guard is against today's gains (or, if those fail, against the best feasible grid point).
    Returns (best knobs, its J, every candidate); J inf when nothing passes.

    structure quan (card 2026-10-23_quan_exact): QUAN_SPACE from QUAN_DEFAULTS on world C, the grid over QUAN_GRID,
    one descent pass, the cases QUAN_CASES (hover and line_x 0.4, then the margin flight), world C's guards and no
    hover guard (the law has no "today's" point)."""
    from multiprocessing import Pool
    seen = {}
    key = lambda k: json.dumps(k, sort_keys=True)  # noqa: E731
    quan = structure in QUAN_READINGS
    defaults, space, gr = (QUAN_DEFAULTS, QUAN_SPACE, QUAN_GRID) if quan else (DEFAULTS, SPACE, GRID)
    if quan and start == 'low':
        defaults = QUAN_LOW_START
        space = [(k, tuple(f * defaults[k] for f in (0.5, 1.0, 2.0))) if k.startswith('quan') else (k, v)
                 for k, v in space]
    cases, passes = (QUAN_CASES, 1) if quan else (CASES, passes)
    values = dict(space)

    def run(cands):
        todo = [k for k in cands if key(k) not in seen]
        for knobs, _, res in pool.imap(evaluate, [(k, structure, vel_filter, cases) for k in todo]):
            seen[key(knobs)] = res

    def feasible_j(k):
        res = seen[key(k)]
        return objective(res) if not guards(res, hover_ref, quan) else math.inf

    def rank(k):
        res = seen[key(k)]
        div = min((s['diverged'] for s in res.values() if 'diverged' in s), default=math.inf)
        if quan:
            # the quan critic (F2): a point that diverges must not outrank one that holds, however far off: first the
            # cases lost, then the guards broken, then how far the ring got (the worst case's band), then J
            n_div = sum('diverged' in s for s in res.values())
            band = max((s.get('band_max_mm', 0.0) for s in res.values() if 'diverged' not in s), default=math.inf)
            return n_div, len(guards(res, hover_ref, quan)), band, objective(res), -div
        return len(guards(res, hover_ref, quan)), objective(res), -div

    def nfail(k):
        return len(guards(seen[key(k)], hover_ref, quan))

    with Pool(workers, maxtasksperchild=4) as pool:
        run([dict(defaults)] + [{**defaults, gr[0]: a, gr[1]: b} for a in values[gr[0]] for b in values[gr[1]]])
        today = seen[key(defaults)]
        hover_ref = None if quan or guards(today) else today['hover']['err_rms_mm']
        grid = [k for k in (json.loads(s) for s in seen) if not guards(seen[key(k)], None, quan)]
        if hover_ref is None and grid and not quan:
            hover_ref = min(seen[key(k)]['hover']['err_rms_mm'] for k in grid)
        best = json.loads(min(seen, key=lambda s: rank(json.loads(s))))
        log(f'[{structure}] stage 1: start {guards(today, None, quan) or "passes"}; stage 2 starts from {best} '
            + (f'J {feasible_j(best):.1f}' if not nfail(best) else f'(no grid point passes; {nfail(best)} '
               'guard failures, the fewest)'))
        for p in range(passes):
            for name, vals in space:
                run([{**best, name: v} for v in vals])
                for v in vals:
                    k = {**best, name: v}
                    if rank(k) < rank(best):
                        best = k
                log(f'[{structure}] stage 2 pass {p + 1} {name}: {best[name]} J {feasible_j(best):.1f}, '
                    f'{nfail(best)} guard failures')
        best_j = feasible_j(best)
    return best, best_j, [dict(knobs=json.loads(s), results=v, J=objective(v), guards=guards(v, hover_ref, quan))
                          for s, v in seen.items()]


def bench(n=300):
    """Compute per tick on this machine: the network step, the step plus a 21-node preview, the velocity loop."""
    rho = attach_points(N, ATTACH_R, ATTACH_Z)
    _, p = params_of(None, 'port')
    net = DissipativeNetwork(N, rho, L, M_QUAD, M_LOAD, 9.81, p)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    p0 = np.array([0.0, 0.0, Z_TARGET])
    net.seed([net.cone_target(i, q, p0) for i in range(N)])
    ts, th = [], []
    for k in range(n):
        pd = p0 + np.array([0.001 * k, 0.0, 0.0])
        t0 = time.perf_counter()
        net.step(pd, q, np.zeros(3), pd, NET_DT)
        t1 = time.perf_counter()
        net.horizon_references(q, [pd + np.array([0.001 * j, 0.0, 0.0]) for j in range(21)], NET_DT)
        ts.append(1e3 * (t1 - t0))
        th.append(1e3 * (time.perf_counter() - t0))
    vl, tv = VelocityLoop(), []
    for _ in range(10 * n):
        t0 = time.perf_counter()
        vl.step(np.zeros(3), np.zeros(3), q, np.array([0.01, 0.0, 0.0]), np.zeros(3), np.array([0.0, 0.0, 9.81]),
                CTRL_DT, KT)
        tv.append(1e3 * (time.perf_counter() - t0))
    q50 = lambda x: [float(np.percentile(x, 50)), float(np.percentile(x, 95))]  # noqa: E731
    return {'net_step_ms': q50(ts), 'net_step_preview21_ms': q50(th), 'velocity_loop_ms': q50(tv)}


def _table(rows):
    keys = ('err_rms_mm', 'err_first5_rms_mm', 'cf_rms_mm', 'reach_rms_mm', 'reach_mean_mm', 'headline_mm', 'lag_s',
            'rmse_t_mm', 'tilt_peak_deg', 'quad_tilt_peak_deg', 'thr_peak', 'yaw_peak_deg', 'yaw_growth_deg_s')
    out = ['| form | case | ' + ' | '.join(keys) + ' |', '|---|---|' + '---|' * len(keys)]
    for form, res in rows:
        for case, s in res.items():
            if 'diverged' in s:
                out.append(f'| {form} | {case} | diverged at {s["diverged"]} s |')
                continue
            out.append(f'| {form} | {case} | ' + ' | '.join(
                '' if k not in s else f'{s[k]:+.3f}' if k in ('lag_s', 'yaw_growth_deg_s') else f'{s[k]:.1f}'
                if k.endswith('_mm')
                else f'{s[k]:.2f}' for k in keys) + ' |')
    return '\n'.join(out)


def emit(name, structure, knobs, root=REPO):
    """The twin confirmation of one candidate (card §3.3): configs/twin/c_net_<name>.yaml (the claim's params file plus
    the candidate's knobs) and its hover and line_x 0.4 configs, made from the form's claim configs. Returns paths."""
    import yaml
    base = 'c_net' if structure == 'port' else 'c_netp'
    params_src = os.path.join(root, 'configs', 'twin', 'c_compare.yaml' if structure == 'port' else
                              'c_compare_paper.yaml')
    with open(params_src) as fh:
        params = yaml.safe_load(fh)
    k = {**DEFAULTS, **knobs}
    params.update({'diss_c': k['diss_c'], 'diss_k_slot': k['diss_k_slot'], 'diss_k_pay': k['diss_k_pay'],
                   'diss_k_anchor': k['diss_k_pay'], 'diss_k_ring': k['diss_k_ring'], 'diss_ki_load': k['diss_ki_load'],
                   'vel_kp_pos': k['vel_kp_pos'], 'vel_kv': k['vel_kv'], 'vel_k_att': k['vel_k_att']})
    pfile = os.path.join(root, 'configs', 'twin', f'c_net_{name}.yaml')
    with open(pfile, 'w') as fh:
        fh.write(f'# Retune candidate {name} ({structure}), card docs/experiments/2026-10-23_paper_method_compare.md '
                 f'§3.3: {params_src.split("configs/")[-1]} plus its knobs (tools/network_tune.py --emit)\n')
        yaml.safe_dump(params, fh, default_flow_style=False, sort_keys=False)
    out = [pfile]
    hover_src = 'c_net_hover_L100.yaml'            # one law for both forms in hover
    for case, src in (('hover', hover_src), ('linex_v400', f'{base}_linex_v400_L100.yaml')):
        with open(os.path.join(root, 'configs', 'experiments', src)) as fh:
            text = fh.read()
        old = src[:-len('.yaml')]
        new = f'c_net_tune_{name}_{case}_L100'
        text = text.replace(f'name: {old}', f'name: {new}', 1)
        text = text.replace("description: '", f"description: 'Retune candidate {name} ({structure}, card §3.3): ", 1)
        text = text.replace("Claim config: flies on the frozen tree after the critic, 3 repeats, the card''s run-end "
                            "rule.", 'Exploratory, once.')
        text = text.replace(text.split('params_file: ')[1].split('\n')[0],
                            f'/home/wesley/multi_drone_control/configs/twin/c_net_{name}.yaml', 1)
        path = os.path.join(root, 'configs', 'experiments', f'{new}.yaml')
        with open(path, 'w') as fh:
            fh.write(text)
        out.append(path)
    return out


def _out_path(path, kind):
    return path or os.path.join(REPO, 'results', 'analysis',
                                f'network_tune_{kind}_{datetime.date.today().isoformat()}.json')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--quick', action='store_true', help='both forms at today\'s gains: hover, line_x 0.2 and 0.4')
    ap.add_argument('--search', action='store_true', help='the card\'s retune: the grid, then coordinate descent '
                                                         '(hours of CPU)')
    ap.add_argument('--bench', action='store_true')
    ap.add_argument('--emit', metavar='NAME', help='write the twin confirmation configs of --gains for --structure '
                                                  'port or paper (configs/twin/c_net_NAME.yaml + two experiments)')
    ap.add_argument('--structure', default='both', choices=('port', 'paper', 'both', *QUAN_READINGS),
                    help='quan: Quan et al. as printed on world C (card 2026-10-23_quan_exact)')
    ap.add_argument('--gains', default='', help='knob=value,... over DEFAULTS (quan: QUAN_DEFAULTS; one candidate)')
    ap.add_argument('--filter', default='ema', choices=('ema', 'slope', 'none'), help='the mocap velocity filter')
    ap.add_argument('--quan-start', default='init', choices=('init', 'low'),
                    help="quan searches: start at init.m's coefficients (default) or at the low point (card §3.3)")
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--out', help='JSON path (default results/analysis/network_tune_<kind>_<date>.json)')
    a = ap.parse_args(argv)
    quan = a.structure in QUAN_READINGS
    forms = ('port', 'paper') if a.structure == 'both' else (a.structure,)
    knobs = {k: float(v) for k, v in (kv.split('=') for kv in a.gains.split(',') if kv)}
    defaults = QUAN_DEFAULTS if quan else DEFAULTS
    unknown = sorted(set(knobs) - set(defaults))
    if unknown:
        ap.error(f'unknown knob(s) {", ".join(unknown)}; known: {", ".join(defaults)}')
    if a.bench:
        print(json.dumps(bench(), indent=1))
        return 0
    if a.emit:
        if a.structure == 'both' or quan:
            ap.error('--emit needs --structure port or paper')
        for p in emit(a.emit, a.structure, knobs):
            print('wrote', p)
        return 0
    if a.search:
        out = {}
        for s in forms:
            best, j, cands = search(s, a.filter, a.workers, start=a.quan_start)
            out[s] = dict(best=best, J=j, candidates=cands)
        path = _out_path(a.out, 'search')
    else:
        from multiprocessing import Pool
        cases = QUICK if a.quick or not a.gains else CASES
        jobs = [(knobs, s, a.filter, cases) for s in forms]
        with Pool(min(a.workers, len(jobs))) as pool:
            res = pool.map(evaluate, jobs)
        out = {s: dict(knobs={**defaults, **k}, results=r, J=objective(r), guards=guards(r, None, quan))
               for k, s, r in res}
        print(_table([(s, out[s]['results']) for s in forms]))
        for s in forms:
            print(f'{s}: J {out[s]["J"]:.1f} mm, guards {out[s]["guards"] or "pass"}')
        path = _out_path(a.out, 'quick' if a.quick or not a.gains else 'one')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as fh:
        ar, az = GEOMETRY['hubcom' if quan else 'rim']
        json.dump(dict(card='docs/experiments/2026-10-23_' + ('quan_exact.md' if quan else 'paper_method_compare.md'),
                       filter=a.filter,
                       geometry=dict(n=N, cable_len=L, elev_deg=ELEV_DEG, attach_radius=ar, attach_z=az, load_mass=M_LOAD,
                                     quad_mass=M_QUAD, line_m=LINE_M, thrust_ratio=KT, offsets=PLANT_OFFSETS),
                       results=out), fh, indent=1, default=float)
    print(f'-> {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
