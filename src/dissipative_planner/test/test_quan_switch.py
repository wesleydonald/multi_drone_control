"""net_law (card 2026-10-23_quan_exact): 'port' (the default) flies the network as before; 'quan' builds the paper's
law at the hand-over and publishes its velocity command and thrust feedforward on world C's geometry."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from dissipative_planner.dissipative_network import DissipativeNetwork, DissipativeParams  # noqa: E402
from dissipative_planner.quan_law import QuanParams  # noqa: E402
from mpc_planner.geometry import attach_points  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402

LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = debug = info


def _node(law=None, traj='hover', nodes='algebraic'):
    """A planner at the hand-over on world C: 4 quads on 1 m rods at 60 deg from a 1 mm hub at the ring's centre of
    mass, each rod hinged at its drone's centre. law None: a node built before the switch existed."""
    f = D.__new__(D)
    rho = attach_points(4, 0.001, 0.0)
    load = np.array([0.0, 0.0, 1.0])
    f.n = f.n_net = f._n_carry0 = 4
    f.rho, f.slot2drone, f.reserved_attach = rho, [0, 1, 2, 3], 0
    f.attach_pending, f.attach_pos, f.detached = [], [], [False] * 4
    f.pivot_offset = np.zeros(3)
    f.cable_len, f.cable_len_i = 1.0, [1.0] * 4
    f.diss = DissipativeParams(elev_deg=60.0)
    f.net = DissipativeNetwork(4, rho, 1.0, 0.55, 0.86, 9.81, f.diss)
    f.load_state = np.r_[load, LEVEL, np.zeros(6)]
    e = np.radians(60.0)
    f.drone_pos = [load + r + np.array([r[0] / 0.001 * np.cos(e), r[1] / 0.001 * np.cos(e), np.sin(e)]) for r in rho]
    f.drone_vel = [np.zeros(3)] * 4
    f.drone_quat = {i: LEVEL.copy() for i in range(4)}
    f.drone_mass, f.load_mass, f.dyn = 0.55, 0.86, SimpleNamespace(g=9.81)
    f.phase, f._net_twin, f._net_hold_carry, f._net_preview, f._net_lean = 'planner', False, False, False, False
    if law is not None:
        f._net_law, f._quan_params, f._quan = law, QuanParams(nodes=nodes), None
    f.load_traj, f.traj = traj, LoadTrajectory(traj, 0.4, 0.9, 0.5)
    f.traj_t, f._traj_hold_s, f._traj_hold_left = 0.0, 5.0, 0.0
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
    f._cable_taut_gate = lambda i: (1.0, 1.0)
    f._publish_ref = lambda d, nodes: f.refs.__setitem__(d, nodes)
    f._log_tick = lambda: f.rows.append(f.traj_t)
    return f


def _handover(f):
    D._enter_network_phase(f)
    D._network_plan(f)


@pytest.mark.parametrize('law', (None, 'port'))
def test_port_is_the_network_as_before(law):
    f, ref = _node(law), _node(None)
    _handover(f)
    _handover(ref)
    assert getattr(f, '_quan', None) is None
    for d in range(4):
        for a, b in zip(f.refs[d], ref.refs[d]):
            assert all(np.array_equal(np.asarray(u), np.asarray(v)) for u, v in zip(a, b))


@pytest.mark.parametrize('nodes', ('algebraic', 'dynamic'))
def test_quan_publishes_the_laws_velocity_command_and_thrust_feedforward(nodes):
    f = _node('quan', nodes=nodes)
    _handover(f)
    assert f._quan is not None and f._quan.started
    assert any('net_law quan' in m for m in f.log.lines)
    for d in range(4):
        nodes = f.refs[d]
        assert len(nodes) == f.N + 1
        p, v, a_ff, a_cable = (np.asarray(x) for x in nodes[0])
        assert np.allclose(p, f.drone_pos[d]) and np.allclose(v, 0.0, atol=1e-9)    # at rest: force-free
        assert np.allclose(a_ff, [0.0, 0.0, 9.81] - a_cable)
        assert a_cable[2] == pytest.approx(-0.86 * 9.81 / 4 / 0.55)                 # its share of the ring
    assert f.rows                                                                    # the tick log goes on


def test_quan_never_reads_the_rings_attitude():
    """The paper's load is a point: the same positions with the ring turned and tilted give the same commands."""
    f, g = _node('quan'), _node('quan')
    c, s = np.cos(0.3), np.sin(0.3)
    g.load_state[3:7] = [c, s * 0.6, 0.0, s * 0.8]
    _handover(f)
    _handover(g)
    for d in range(4):
        for a, b in zip(f.refs[d], g.refs[d]):
            assert all(np.array_equal(np.asarray(u), np.asarray(v)) for u, v in zip(a, b))


def test_quan_follows_the_path_point_after_the_hold():
    """R3, R5: on a line the formation centre is the commanded path point and a_expected its acceleration."""
    f = _node('quan', traj='line_x')
    D._enter_network_phase(f)
    for _ in range(5):
        D._network_plan(f)
    v = [np.asarray(f.refs[d][0][1]) for d in range(4)]
    assert all(vd[0] > 0.0 for vd in v)          # every quad's velocity command leads along +x


def test_quan_hands_over_on_the_pull_split():
    """R6c: the first vertical feed-forward is the ring's share alone (drone_mass holds the rod), whatever rod_mass
    adds horizontally; the hand-over line names the split."""
    f = _node('quan')
    f._quan_params = QuanParams(rod_mass=0.055)
    _handover(f)
    assert any('pull split rod 0.055 kg' in m for m in f.log.lines)
    for d in range(4):
        assert np.asarray(f.refs[d][0][2])[2] == pytest.approx(9.81 + (0.86 / 4) * 9.81 / 0.55)

