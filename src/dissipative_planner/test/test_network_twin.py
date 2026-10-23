"""The network phase on the rig twin (card 2026-10-23_paper_method_compare), each switch off until the card's critic:
net_twin_geometry (references at the drone centre, nodes on the measured pivots and rods), net_hold_carry (the
pre-path hover hold the OCP began runs to its end), net_damp_relative's commanded target velocity. Off, the network
phase publishes what it did before."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from dissipative_planner.dissipative_network import DissipativeNetwork, DissipativeParams  # noqa: E402
from mpc_planner.geometry import attach_points, centre_from_pivot  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])
RODS = [0.99, 1.0, 1.01, 1.0]


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = debug = info


def _node(twin=False, carry=False, traj='hover', hold_left=0.0, pivot=-0.04, rel=False, preview=False):
    """A dissipative planner at the hand-over: 4 quads on 1 m rods at 60 deg over the ring at 1 m, each pivot the
    measured rod from its plate, the drone centre 4 cm above it."""
    f = D.__new__(D)
    rho = attach_points(4, 0.225, 0.02)
    load = np.array([0.0, 0.0, 1.0])
    f.n = f.n_net = f._n_carry0 = 4
    f.rho, f.slot2drone, f.reserved_attach = rho, [0, 1, 2, 3], 0
    f.attach_pending, f.attach_pos, f.detached = [], [], [False] * 4
    f.pivot_offset = np.array([0.0, 0.0, pivot])
    f.cable_len, f.cable_len_i = 1.0, list(RODS)
    f.diss = DissipativeParams(elev_deg=60.0, ki_load=1.0, damp_relative=rel)
    f.net = DissipativeNetwork(4, rho, 1.0, 0.55, 0.86, 9.81, f.diss)
    f.load_state = np.r_[load, LEVEL, np.zeros(6)]
    e = np.radians(60.0)
    f.drone_pos = [load + r + RODS[i] * np.array([r[0] / 0.225 * np.cos(e), r[1] / 0.225 * np.cos(e), np.sin(e)])
                   - f.pivot_offset for i, r in enumerate(rho)]
    f.drone_quat = {i: LEVEL.copy() for i in range(4)}
    f.phase, f._net_twin, f._net_hold_carry, f._net_preview, f._net_lean = 'planner', twin, carry, preview, False
    f.load_traj, f.traj = traj, LoadTrajectory(traj, 0.4, 0.9, 0.5)
    f.traj_t, f._traj_hold_s, f._traj_hold_left = 0.0, 5.0, hold_left
    f._reconfig_hold_left, f._settle_hold, f._net_landing, f._landed = 0.0, False, False, False
    f.land_vel, f._net_land_z, f.hover_xy = 0.2, 0.06, (0.0, 0.0)
    f.lift_z0, f.lift_progress, f.target_z, f.N, f.dt = 0.02, 0.98, 1.0, 20, 0.1
    f._weld_time, f._attach_ff_ramp_s, f._handover_blend_s = {}, 1.5, 0.0
    f._last_ocp_ac, f._ho_ac, f._ho_t0 = {}, {}, None
    f.log, f.refs, f.rows = _Log(), {}, []
    f.get_logger = lambda: f.log
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
    f.phase_pub = f.landed_pub = SimpleNamespace(publish=lambda m: None)
    f._set_ff_active = lambda on: None
    f._publish_load_desired = lambda target=None: None
    f._net_diag = lambda *a: None
    f._publish_ref = lambda d, nodes: f.refs.__setitem__(d, nodes)
    f._log_tick = lambda: f.rows.append(f.traj_t)
    return f


def _handover(f):
    D._enter_network_phase(f)
    D._network_plan(f)


def test_off_the_nodes_start_on_the_bodies_and_the_references_are_the_networks():
    f = _node()
    D._enter_network_phase(f)
    assert all(np.allclose(f.net.q[i], f.drone_pos[i]) for i in range(4))
    assert np.allclose(f.net.cable_len_i, 1.0)
    D._network_plan(f)
    for i in range(4):
        p_net = f.net.reference(i, LEVEL, f._net_p_des, taut_gate=1.0)[0]
        assert len(f.refs[i]) == f.N + 1 and np.allclose(f.refs[i][0][0], p_net)


def test_twin_geometry_seeds_the_pivots_and_the_measured_rods():
    f = _node(twin=True)
    D._enter_network_phase(f)
    for i in range(4):
        assert np.allclose(f.net.q[i], f.drone_pos[i] + f.pivot_offset)      # level: 4 cm below the centre
    assert list(f.net.cable_len_i) == RODS
    assert any('measured rods 0.990 1.000 1.010 1.000 m' in m for m in f.log.lines)


def test_twin_geometry_flies_the_drone_centre_above_the_networks_pivot():
    f = _node(twin=True)
    _handover(f)
    for i in range(4):
        p_net, _, a_ff, _ = f.net.reference(i, LEVEL, f._net_p_des, taut_gate=1.0)
        want = centre_from_pivot(p_net, a_ff, 0.0, f.pivot_offset)
        assert np.allclose(f.refs[i][0][0], want) and np.allclose(f.refs[i][-1][0], want)
        assert 0.03 < f.refs[i][0][0][2] - p_net[2] <= 0.04          # tilted out a little by the rod's pull
        assert np.allclose(f.refs[i][0][2], a_ff)                      # the feedforward itself is not moved


def test_twin_geometry_with_no_pivot_offset_changes_nothing():
    a, b = _node(pivot=0.0), _node(twin=True, pivot=0.0)
    b.cable_len_i = a.cable_len_i = [1.0] * 4
    _handover(a)
    _handover(b)
    for i in range(4):
        assert np.allclose(a.refs[i][0][0], b.refs[i][0][0])


def test_the_hold_carries_over_the_hand_over_and_says_when_the_path_starts():
    f = _node(carry=True, traj='line_x', hold_left=3.5)          # the OCP held 1.5 of its 5 s before the hand-over
    _handover(f)
    for _ in range(40):
        D._network_plan(f)
    done = [m for m in f.log.lines if 'hover hold done' in m]
    assert done == ['[planner] hover hold done: line_x starts']
    held = sum(1 for t in f.rows if t == 0.0)
    assert held in (35, 36)                                       # 3.5 s at 10 Hz, the line's tick included
    assert np.allclose(np.diff(f.rows[held:]), 0.1)


def test_without_the_carry_the_path_starts_at_the_hand_over():
    f = _node(traj='line_x', hold_left=3.5)
    _handover(f)
    D._network_plan(f)
    assert f.rows == [0.1, 0.2] and not any('hover hold done' in m for m in f.log.lines)


def test_a_hover_never_holds_and_every_network_tick_writes_one_row():
    f = _node(carry=True, traj='hover', hold_left=5.0)
    _handover(f)
    for _ in range(4):
        D._network_plan(f)
    assert len(f.rows) == 5 and f.rows[-1] > 0.0 and f._traj_hold_left == 5.0


def _vel_calls(f):
    calls = []
    step = f.net.step
    f.net.step = lambda *a, **k: (calls.append(k.get('p_des_vel')), step(*a, **k))[1]
    return calls


def test_relative_damping_gets_the_commanded_target_velocity():
    f = _node(rel=True, traj='line_x')
    calls = _vel_calls(f)
    _handover(f)
    for _ in range(9):
        D._network_plan(f)
    want = f.traj.offset_at(f.traj_t)[2:4]
    assert np.allclose(calls[-1], [want[0], want[1], 0.0]) and calls[-1][0] > 0.05
    f._reconfig_hold_left = 1.0                                   # a hold: the target is still
    D._network_plan(f)
    assert np.allclose(calls[-1], 0.0)
    f._reconfig_hold_left, f._net_landing = 0.0, True              # LAND: the target descends at land_vel
    D._network_plan(f)
    assert np.allclose(calls[-1], [0.0, 0.0, -0.2])


def test_without_relative_damping_the_network_gets_no_velocity():
    f = _node(traj='line_x')
    calls = _vel_calls(f)
    _handover(f)
    D._network_plan(f)
    assert calls == [None, None]
