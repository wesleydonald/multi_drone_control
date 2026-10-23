"""Drop and land (card 2026-10-08_drop_and_land): a loss that leaves fewer than min_survivors, a gap
>= 180 deg, a loss in LAND, or a tracker's /fleet/drop request releases every magnet, makes every
drone a departed drone that steps clear and lands, stops the OCP and latches /fleet/payload_dropped;
/fleet/landed follows once every drone is down."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (DEPARTED_CLEAR_R, DissipativeController as D,  # noqa: E402
                                                  _survivor_gap_deg)
from mpc_planner.geometry import attach_points  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data if hasattr(m, 'data') else m)


def _node(azimuths='30,90,150,270', detached=None, landing=False):
    rho = attach_points(4, 0.225, 0.0, azimuths)
    log = _Log()
    pos = [np.array([0.5 * np.cos(np.radians(a)), 0.5 * np.sin(np.radians(a)), 1.5])
           for a in (30, 90, 150, 270)]
    f = SimpleNamespace(
        slot2drone=[0, 1, 2, 3], detached=list(detached or [False] * 4), _min_survivors=3, rho=rho,
        _land_to_ground=landing, _dropped=None, _drop_t=None, _drop_centre=None, _drop_on_loss=True,
        phase='planner', takeoff_seen=True, _unload=None, load_state=np.r_[0.0, 0.0, 1.0, 1, 0, 0, 0,
                                                                          np.zeros(6)],
        drone_pos=pos, drone_vel=[np.zeros(3)] * 4, _departed_hold={}, _departed_step={},
        _departed_next={}, _departed_land=False, _landed=False,
        detach_pub=[_Pub() for _ in range(4)], _magnet_pubs=[_Pub() for _ in range(4)],
        _dropped_pub=_Pub(), landed_pub=_Pub(), reserved_attach=0, _magnet_cmd_pub=_Pub())
    f.get_logger = lambda: log
    f.log = log
    f.now = [100.0]                        # the node clock (sim time in the twin)
    f.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(f.now[0] * 1e9)))
    for m in ('_drop_due', '_drop_and_land', '_release_all_magnets', '_drop_tick',
              '_cancel_unload_quietly', '_drop_request_cb', '_on_unannounced'):
        setattr(f, m, (lambda name: lambda *a, **k: getattr(D, name)(f, *a, **k))(m))
    f._publish_departed_refs = lambda: None
    f.down = False
    f._departed_down = lambda: f.down
    return f


def test_drop_due_on_fewer_than_three_a_half_ring_or_in_land():
    assert _node()._drop_due(1) is None                                 # 30/150/270: CoG inside
    assert '240 deg gap' in _node()._drop_due(3)                        # 30/90/150: CoG outside
    assert 'would remain' in _node(detached=[False, True, False, False])._drop_due(0)
    assert _node(landing=True)._drop_due(1) == 'during LAND'


def test_drop_releases_every_magnet_departs_every_drone_and_latches():
    f = _node()
    f._drop_and_land('test')
    assert f._dropped == 'test' and f.detached == [True] * 4
    assert all(len(p.msgs) == 1 for p in f.detach_pub)                 # every twin joint once
    assert all(p.msgs == ['OFF'] for p in f._magnet_pubs)
    assert f._dropped_pub.msgs == [True] and f._departed_land
    for d in range(4):
        step = f._departed_step[d]
        assert abs(np.linalg.norm(step[:2]) - DEPARTED_CLEAR_R) < 1e-9 and step[2] == 1.5
    f._drop_and_land('again')
    assert f._dropped == 'test' and f._dropped_pub.msgs == [True]       # once


def test_a_drone_that_left_earlier_keeps_its_own_plan():
    f = _node(detached=[False, True, False, False])
    f._departed_hold[1] = np.array([0.0, 1.0, 1.5])
    f._departed_step[1] = np.array([0.0, 1.0, 1.5])
    f._drop_and_land('second loss')
    assert np.allclose(f._departed_step[1], [0.0, 1.0, 1.5])
    assert f.detach_pub[1].msgs == []                                  # its joint went before


def test_drop_tick_resends_off_and_announces_landed_once_all_down():
    f = _node()
    f._drop_and_land('test')
    f._drop_tick()
    assert all(p.msgs == ['OFF', 'OFF'] for p in f._magnet_pubs) and f.landed_pub.msgs == []
    f.down = True
    f._drop_tick()
    f._drop_tick()
    assert f.landed_pub.msgs == [True] and f._landed


def test_drop_tick_times_out_on_the_node_clock():
    f = _node()
    f._drop_and_land('test')
    f.now[0] += 29.0                       # 29 s of node time: still waiting, however long the wall took
    f._drop_tick()
    assert f.landed_pub.msgs == []
    f.now[0] += 2.0
    f._drop_tick()
    assert f.landed_pub.msgs == [True]


def test_drop_request_only_when_on_and_flying():
    f = _node()
    f._drop_on_loss = False
    f._drop_request_cb(SimpleNamespace(data='x'))
    assert f._dropped is None
    f._drop_on_loss, f.takeoff_seen = True, False
    f._drop_request_cb(SimpleNamespace(data='x'))
    assert f._dropped is None
    f.takeoff_seen = True
    f._drop_request_cb(SimpleNamespace(data='drone 2: ring tilt 31 deg'))
    assert f._dropped == 'drone 2: ring tilt 31 deg'


def test_detection_drops_instead_of_a_resize_when_due():
    f = _node()
    f.cable_len, f.cable_len_i, f._detect_m = 0.55, [0.55] * 4, 0.06
    f._reconfig_mode, f._detect_count, f._detect_ok, f._detect_refused = 'ocp', {}, {}, set()
    f._load_t, f._drone_t = time.monotonic(), {k: time.monotonic() for k in range(4)}
    f._load_down = lambda: False
    f.calls = []
    f._detach_ocp = lambda d: f.calls.append(d)
    dists = [0.55, 0.55, 0.55, 0.55]
    f._rim_dist = lambda i: dists[i]
    D._detect_tick(f)                      # the tick within the length
    dists[3] = 0.62
    D._detect_tick(f)
    dists[3] = 0.70
    D._detect_tick(f)                      # drone 4 (plate 3, at 270): survivors 30/90/150
    assert f.calls == [] and f._dropped.startswith('unannounced loss of drone 4')


def test_a_drop_switches_a_welded_newcomers_coil_off_at_most_once_a_second():
    f = _node()
    f.reserved_attach = 1
    f._drop_and_land('test')
    assert f._magnet_cmd_pub.msgs == ['OFF']
    f._release_all_magnets()
    assert f._magnet_cmd_pub.msgs == ['OFF']          # same second: not again
    f.now[0] += 1.0
    f._release_all_magnets()
    assert f._magnet_cmd_pub.msgs == ['OFF', 'OFF']
    assert _node()._magnet_cmd_pub.msgs == []         # no newcomer: the coil topic stays quiet


def test_drop_takes_a_pending_newcomer_back_only_when_switched_on():
    """Card 2026-10-16_drop_takes_back (RIG-1015-a2_2): with drop_takes_back on, the drop runs the take-back for a
    newcomer still pending (his stream flying it); off (the default), or after its attach, it does not."""
    for on, pending, want in ((False, True, []), (True, True, [(0, 'DROP')]), (True, False, [])):
        f = _node()
        f.reserved_attach, f.attach_pending, f._drop_takes_back = 1, [pending], on
        calls = []
        f._take_back = lambda j, why, calls=calls: calls.append((j, why))
        f._drop_and_land('test')
        assert calls == want, (on, pending)


def test_drop_tick_keeps_a_pending_newcomer_warm_and_waits_longer_for_its_take_back_only_when_switched_on():
    """Card 2026-10-16_drop_takes_back rev 2 (critic F1, F2): with the switch on, the drop tick keeps publishing the
    pending newcomer's warm reference (its tracker stays able to take it), and /fleet/landed waits
    DROP_TAKE_BACK_EXTRA_S longer while a take-back flies it; off, both as before."""
    from dissipative_planner.dissipative_node import DROP_LAND_WAIT_S, DROP_TAKE_BACK_EXTRA_S
    for on in (False, True):
        f = _node()
        f._drop_takes_back, f._partner_contact, f._taken_back = on, True, False
        warm = []
        f._publish_pending_attach_refs = lambda warm=warm: warm.append(1)
        f._drop_and_land('test')
        f._drop_tick()
        assert bool(warm) == on
        f._taken_back = True
        f.now[0] += DROP_LAND_WAIT_S + 1.0
        f._drop_tick()
        assert f._landed == (not on)                    # on: still waiting for the take-back
        f.now[0] += DROP_TAKE_BACK_EXTRA_S
        f._drop_tick()
        assert f._landed


def test_no_release_guard_after_a_drop():
    """Critic F3: after a drop every coil is OFF and the plate falls, so the rig's release guard does not arm."""
    st = {'hold': np.array([0.0, 0.2, 1.6])}
    f = SimpleNamespace(_release_guard=True, load_state=np.zeros(13), _dropped='test')
    D._guard_setup(f, 0, 3, st, True, [np.array([0.0, 1.2, 1.6])])
    assert 'guard' not in st


def _gap(f, deg=200.0, lifted=True):
    """drop_gap_deg, the lift state and the rule's helpers on a stub node."""
    f._drop_gap_deg = deg
    f.lift_z0, f.target_z, f.lift_progress = 0.02, 1.0, (0.98 if lifted else 0.4)
    f._gap_flies = lambda rho: D._gap_flies(f, rho)
    f._detach_plan = lambda d: D._detach_plan(f, d)
    return f


def test_the_survivor_gap_counts_a_pair_by_its_larger_arc():
    assert _survivor_gap_deg(attach_points(2, 0.225, 0.0, '0,180')) == pytest.approx(180.0)
    assert _survivor_gap_deg(attach_points(2, 0.225, 0.0, '0,150')) == pytest.approx(210.0)
    assert _survivor_gap_deg(attach_points(2, 0.225, 0.0, '350,170')) == pytest.approx(180.0)
    assert _survivor_gap_deg(attach_points(3, 0.225, 0.0, '0,90,180')) == pytest.approx(180.0)
    assert _survivor_gap_deg(attach_points(1, 0.225, 0.0, '0')) == 360.0


def test_drop_gap_deg_flies_the_t_and_the_opposite_pair_after_the_lift_and_nothing_else():
    """The 90 deg chain (card 2026-10-21_chain_release, Wesley's 200 deg rule): after the lift a loss keeps flying while
    the survivors' largest empty arc is at most 200 deg; anything wider, one quad, a loss in LAND or before the lift
    still drops, and with the knob off every decision is as before."""
    sq = '0,90,180,270'
    assert '180 deg gap' in _gap(_node(sq), deg=0.0)._drop_due(1)       # off: the square -> T drops, as today
    assert _gap(_node(sq))._drop_due(1) is None                         # the T 0/180/270
    assert '180 deg gap' in _gap(_node(sq), lifted=False)._drop_due(1)  # not before the lift is done
    assert _gap(_node('0,90,190,270'))._drop_due(1) is None             # survivors 0/190/270: 190 <= 200
    assert '205 deg gap' in _gap(_node('0,90,205,270'))._drop_due(1)    # 205 > 200
    t = _gap(_node(sq, detached=[False, True, False, False]), deg=0.0)
    assert 'would remain' in t._drop_due(3)                             # off, min_survivors 3: the pair drops
    t._min_survivors = 2
    assert '360 deg gap' in t._drop_due(3)                              # min_survivors 2 alone still drops
    t._drop_gap_deg = 200.0
    assert t._drop_due(3) is None                                       # T -> the opposite pair 0/180
    assert '270 deg gap' in t._drop_due(0)                              # T -> 180/270, 90 deg apart
    pair = _gap(_node(sq, detached=[False, True, False, True]))
    pair._min_survivors = 2
    assert 'would remain' in pair._drop_due(0)                          # the pair -> one quad
    land = _gap(_node(sq, landing=True))
    assert land._drop_due(1) == 'during LAND'


def test_the_announced_path_leaves_a_pair_only_within_drop_gap_deg():
    t = _gap(_node('0,90,180,270', detached=[False, True, False, False]), deg=0.0)
    t._min_survivors = 2
    assert t._detach_plan(3) is None                                    # knob off: no pair at all
    t._drop_gap_deg = 200.0
    assert t._detach_plan(3) is not None                                # the opposite pair 0/180
    assert t._detach_plan(0) is None                                    # 180/270 would remain
    assert t._detach_plan(2) is None                                    # 0/270 would remain
