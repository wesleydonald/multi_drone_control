"""foldin_cap 'level' (card 2026-11-05_quad4_foldin_weld_cap; off by default, twin only): through QUAD4's fold-in blend
no node of its reference sits above where a level ring puts it on its rod (_foldin_cap_nodes), the level-ring half of
the rejoin's cap (_rejoin_cap, unchanged) without its weld-height ratchet ('weld', deleted 5 Nov after R3754). R2829:
folded onto the pair QUAD4 is the T's rider, and the plan followed the fold-in's thrust step up (9.9 deg).
"""
from types import MethodType

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import FOLDIN_BIND_M, DissipativeController as D  # noqa: E402
from mpc_planner.planner_node import LoadPlanner  # noqa: E402
from test_partner_contact import _fold_in_node  # noqa: E402

ROD = 0.59                     # _fold_in_node: QUAD4's rod, its body straight over plate 9 (+y) of the ring at 1 m
RHO9 = np.array([0.0, 0.225, 0.02])
ZERO = np.zeros(3)
METHODS = ('_foldin_cap_nodes', '_foldin_cap_start', '_foldin_cap_tick', '_plate_world')


def _folded(mode, tilt_deg=0.0, pivot=0.0, ring_z=1.0, **kw):
    """QUAD4 folded in by _attach_ocp with foldin_cap `mode` (None: a node from before the switch), its body straight
    over its plate on a ring at ring_z rolled tilt_deg about x (positive: plate 9 low), its rod's pivot `pivot` under
    the body (the arm body_to_tip = ROD - pivot); its blend running."""
    f = _fold_in_node()
    for name in METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    if mode is not None:
        f._foldin_cap = mode
    a = -np.radians(tilt_deg)
    f.load_state = np.r_[0.0, 0.0, ring_z, np.cos(a / 2), np.sin(a / 2), 0.0, 0.0, np.zeros(6)]
    f.pivot_offset = np.array([0.0, 0.0, pivot])
    f._attach_body_to_tip = ROD - pivot
    f.attach_pos[0] = f._plate_world({'rho': RHO9}) + [0.0, 0.0, ROD - pivot]
    f.__dict__.update(_rejoin_blend=None, descending=False, _traj_running=False, **kw)
    assert f._attach_ocp(3) is True
    f.active = [True]
    f.refs.blend_active = lambda: f.active[0]
    return f


def _rider(f, capped, push, ticks=100):
    """The T's rider through its blend (R2829, as test_rejoin_departed's E2 model): its rigid rod puts plate 9 wherever
    its body is, its plan starts from its body (node 0 pinned) and swings its rod out 30 deg about the plate, and its
    body goes where the published reference goes plus `push` up per tick for 1.5 s (the fold-in's thrust step).
    Returns the published node-0 heights and plate 9's lift over the level ring."""
    pz = float(f.pivot_offset[2])
    plate0 = f._plate_world({'rho': RHO9})
    u = np.array([0.0, 1.0])
    w = np.radians(30.0) / ticks
    body = np.asarray(f.attach_pos[0], float).copy()
    zs, lifts = [], []
    for j in range(ticks):
        h = float(np.hypot(*(body[:2] - plate0[:2])))
        plate = np.r_[plate0[:2], body[2] + pz - np.sqrt(ROD ** 2 - h * h)]
        lifts.append(plate[2] - plate0[2])
        el0 = np.arctan2(body[2] + pz - plate[2], h)
        nodes = [(np.r_[plate[:2] + ROD * np.cos(el0 - w * k) * u, plate[2] + ROD * np.sin(el0 - w * k) - pz], ZERO,
                  ZERO, ZERO) for k in range(21)]
        pub = f._foldin_cap_nodes(3, nodes) if capped else nodes
        zs.append(float(pub[0][0][2]))
        body = np.asarray(pub[1][0], float) + [0.0, 0.0, push if j < 15 else 0.0]
    return np.array(zs), np.array(lifts)


def test_off_folds_quad4_in_as_before_and_the_cap_changes_nothing_the_ocp_gets():
    """Off (a node from before the switch, or 'off'): no cap state and no line. On, the OCP's inputs are the same."""
    runs = {m: _folded(m) for m in (None, 'off', 'level')}
    for m in (None, 'off'):
        assert getattr(runs[m], '_foldin_cap_b', None) is None
        assert not any('FOLD-IN CAP' in s for s in runs[m].log.lines)
    on, off = runs['level'], runs['off']
    keys = {'d', 'rho', 'len', 'n', 'bind_n', 'bind_m', 'near_m', 'near_last', 'near_n'}
    assert on._foldin_cap_b['d'] == 3 and set(on._foldin_cap_b) == keys
    assert on.blends[-1] == off.blends[-1] and on.cable_len_i == off.cable_len_i
    assert all(np.allclose(r, q) for r, q in zip(on.rho, off.rho))


def test_a_start_weld_before_takeoff_is_not_capped():
    """The critic (5 Nov): an air or taut start weld reaches the blend too; its rod is laid out like a carrier's, not
    hung, so the cap leaves it alone. After takeoff the same fold-in is capped."""
    assert getattr(_folded('level', _start_attached=True, takeoff_seen=False), '_foldin_cap_b', None) is None
    assert _folded('level', _start_attached=True, takeoff_seen=True)._foldin_cap_b['d'] == 3


@pytest.mark.parametrize('pivot', [0.0, -0.04])
def test_onto_the_pair_quad4s_reference_never_rises_and_its_plate_stays_level(pivot):
    """R2829 (the pair 0/6, QUAD4 folded in on 9: the T's rider): the plan followed the fold-in's thrust step up 3.9 cm
    and the ring tilted 9.9 deg (R3743 on 5 Nov: 5.5 deg over the weld; capped, R3748: 2.6). Capped, the published
    height never passes the weld's on the level ring and plate 9 lifts at most one tick's push; uncapped the same model
    rises over 3 cm. The twin's pivot sits 4 cm under the body."""
    f = _folded('level', pivot=pivot)
    b = f._foldin_cap_b
    z_weld = float(f.attach_pos[0][2])
    assert b['len'] == pytest.approx(ROD) and np.allclose(b['rho'], RHO9)
    zs, lifts = _rider(f, True, push=0.006)
    assert zs.max() <= z_weld + 1e-9 and lifts.max() <= 0.006 + 1e-6
    assert b['near_n'] > 0 and b['near_m'] >= 0.006 - 1e-6        # the push binds where the tracker flies
    zs, lifts = _rider(_folded('off', pivot=pivot), False, push=0.006)
    assert zs.max() - zs[0] > 0.03 and lifts.max() > 0.03


def test_an_arm_off_the_ocps_rod_clips_the_caps_rod_and_says_so():
    """The cap's rod is the pivot-to-plate-point arm the capture gives (body_to_tip less the pivot, not a measurement),
    clipped to 1.5 cm of the OCP's rod: body_to_tip typed for no pivot, the pivot 4 cm under the body: 0.55 m."""
    assert _folded('level', pivot=-0.04)._foldin_cap_b['len'] == pytest.approx(ROD)
    g = _fold_in_node()
    for name in METHODS:
        setattr(g, name, MethodType(getattr(D, name), g))
    g.__dict__.update(_foldin_cap='level', pivot_offset=np.array([0.0, 0.0, -0.04]), descending=False,
                      _traj_running=False)
    assert g._attach_ocp(3) is True
    assert g._foldin_cap_b['len'] == pytest.approx(ROD - 0.015)
    line = next(s for s in g.log.lines if 'FOLD-IN CAP ON' in s)
    assert "rod 0.575 m (pivot to the captured plate point 0.550, the OCP's 0.590)" in line


def test_the_rejoiners_cap_is_untouched():
    """QUAD4's cap is its own state and method: the rejoiner's _rejoin_cap reads only _rejoin_blend, and with none it
    passes QUAD4's nodes through while QUAD4's cap is on."""
    f = _folded('level')
    f._rejoin_cap = MethodType(D._rejoin_cap, f)
    nodes = [(np.r_[0.0, 0.225, 1.70], ZERO, ZERO, ZERO)] * 3
    assert f._rejoin_cap(3, nodes) is nodes
    assert [p[2] for p, *_ in f._foldin_cap_nodes(3, nodes)] == pytest.approx([1.61] * 3)


def test_a_ring_leaning_down_on_its_plate_is_lifted_level():
    """R3712/R3714/R3715: the T leaned 11-15 deg with plate 9 low when QUAD4 welded, and the fold-in's plan raised QUAD4
    4.4 cm and levelled the ring in about 2 s. The level ring is 5.8 cm higher, so the cap leaves that rise alone (the
    deleted 'weld' ratchet held it: R3754, settle 5 -> 14 s)."""
    f = _folded('level', tilt_deg=14.7)
    b, z_weld = f._foldin_cap_b, float(f.attach_pos[0][2])
    off = float(f._plate_world({'rho': RHO9})[2] - 1.0 - RHO9[2])
    assert off == pytest.approx(-0.0578, abs=5e-4)
    line = next(s for s in f.log.lines if 'FOLD-IN CAP ON' in s)
    assert line.startswith('[dissipative] FOLD-IN CAP ON drone 4 (/drone_3) (level): rod 0.590 m')
    assert line.endswith(f'its plate {100.0 * off:+.1f} cm off level')
    rise = [(np.r_[0.0, 0.2227, z_weld + 0.044 * k / 20], ZERO, ZERO, ZERO) for k in range(21)]
    pub = np.array([p[2] for p, *_ in f._foldin_cap_nodes(3, rise)])
    assert np.allclose(pub, [p[2] for p, *_ in rise]) and b['bind_n'] == 0


def test_a_ring_sagging_under_the_plan_binds_far_nodes_only_under_the_2_mm_bar():
    """The critic (R3712): the measured ring centre sat at 0.994 against the plan's 1.000, so the level ring is 6 mm
    under the plan's and the cap clips the far nodes, which converge on the plan's ring, by ~6 mm. Nodes 0-1 (what the
    tracker flies now) sit within 2 mm of the measured ring: that is not a bind by the card's bar."""
    f = _folded('level', pivot=-0.04, ring_z=0.994)
    b = f._foldin_cap_b
    body = np.asarray(f.attach_pos[0], float)
    plan = [(body + [0.0, 0.0, 0.006 * k / 20], ZERO, ZERO, ZERO) for k in range(21)]
    pub = f._foldin_cap_nodes(3, plan)
    assert b['bind_n'] == 1 and b['bind_m'] == pytest.approx(0.006)
    assert b['near_m'] == pytest.approx(0.0003) and b['near_m'] < FOLDIN_BIND_M and b['near_n'] == 0
    assert pub[-1][0][2] == pytest.approx(body[2])


@pytest.mark.parametrize('event, why', [
    (lambda f: f.active.__setitem__(0, False), 'its blend is over'),
    (lambda f: f.detached.__setitem__(3, True), 'it left the ring'),
    (lambda f: setattr(f, 'n', 5), 'the fleet resized again'),
    (lambda f: setattr(f, '_dropped', 'tilt'), 'the ring dropped'),
    (lambda f: setattr(f, '_land_to_ground', True), 'LAND'),
    (lambda f: f.__dict__.update(_traj_running=True, _reconfig_hold_left=0.0), 'the path moves')])
def test_the_cap_lets_go_for_good_and_its_line_says_why_and_how_it_bound(event, why):
    f = _folded('level')
    for dz in (0.02, 0.05, -0.01):                 # the last one sits under the cap: no bind
        f._foldin_cap_nodes(3, [(f.attach_pos[0] + [0.0, 0.0, dz], ZERO, ZERO, ZERO)] * 3)
    f._traj_running = True                         # the path's clock ran, but the fold-in's hold holds it
    f._foldin_cap_tick()
    assert f._foldin_cap_b is not None
    f._traj_running = False
    event(f)
    f._foldin_cap_tick()
    assert f._foldin_cap_b is None
    assert f.log.lines[-1] == (f'[dissipative] FOLD-IN CAP OVER drone 4 (/drone_3) ({why}): the cap bound 2 ticks, '
                               f'the plan 5.0 cm above it at the last; nodes 0-1 at most 50.0 mm above it (2 ticks '
                               f'from 2 mm), 0.0 mm at the last (the step as it lets go)')
    g = _folded('level')
    g.active[0] = False
    g._foldin_cap_tick()
    assert g.log.lines[-1].endswith('(its blend is over): the cap never bound')


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)


def _bare(log):
    """A planner object for the publish wrapper (no __init__), QUAD4 welded on plate 9 of a level ring at 1 m."""
    f = D.__new__(D)
    f.__dict__.update(phase='planner', _rejoin_blend=None, _foldin_cap_b=None, _last_ocp_ac={}, _weld_time={},
                      _ocp_attached={}, _attach_velocity_newcomer=False, slot2drone=[0, 1, 2, 3], n=4,
                      detached=[False] * 4, load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
                      _dropped=None, pivot_offset=np.array([0.0, 0.0, -0.04]), _land_to_ground=False,
                      descending=False, _traj_running=False, _reconfig_hold_left=0.0)
    f.get_logger = lambda: log
    return f


def _cap_state():
    return {'d': 3, 'rho': RHO9.copy(), 'len': ROD, 'n': 4, 'bind_n': 0, 'bind_m': 0.0, 'near_m': 0.0,
            'near_last': 0.0, 'near_n': 0}


def test_the_publish_wrapper_caps_quad4_only(monkeypatch):
    """_publish_ref runs QUAD4's nodes through the cap while it is on; a carrier's, and every node with it off, pass as
    they came."""
    rec = []
    monkeypatch.setattr(LoadPlanner, '_publish_ref', lambda self, i, nodes: rec.append(nodes))
    f = _bare(_Log())
    high = [(np.r_[0.0, 0.225, 1.70], ZERO, ZERO, ZERO)] * 3
    D._publish_ref(f, 3, high)
    assert rec[-1] is high
    f._foldin_cap_b = _cap_state()
    D._publish_ref(f, 3, high)
    assert [p[2] for p, *_ in rec[-1]] == pytest.approx([1.0 + 0.02 + ROD + 0.04] * 3)
    D._publish_ref(f, 0, high)
    assert rec[-1] is high


def test_the_reference_steps_up_by_the_last_near_margin_when_the_cap_lets_go(monkeypatch):
    """The critic (5 Nov): when the blend ends the plan's nodes go out uncapped, so QUAD4's reference steps up by what
    the cap held off nodes 0-1 at its last tick (R2594/R2595 peaked ~1.3 s after the rejoin's cap let go). The line
    gives that step."""
    rec = []
    monkeypatch.setattr(LoadPlanner, '_publish_ref', lambda self, i, nodes: rec.append(nodes))
    log = _Log()
    f = _bare(log)
    active = [True]
    f.refs = type('R', (), {'blend_active': lambda self: active[0]})()
    f._foldin_cap_b = _cap_state()
    level = 1.0 + 0.02 + ROD + 0.04
    wants = [(np.r_[0.0, 0.225, level + (0.004 if k else 0.0)], ZERO, ZERO, ZERO) for k in range(21)]
    D._publish_ref(f, 3, wants)
    capped = rec[-1][1][0][2]
    active[0] = False
    D._foldin_cap_tick(f)
    D._publish_ref(f, 3, wants)
    assert rec[-1] is wants and wants[1][0][2] - capped == pytest.approx(0.004)
    assert log.lines[-1].endswith('nodes 0-1 at most 4.0 mm above it (1 ticks from 2 mm), 4.0 mm at the last (the step '
                                  'as it lets go)')
