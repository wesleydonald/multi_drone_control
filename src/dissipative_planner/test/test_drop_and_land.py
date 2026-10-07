"""Drop and land (card 2026-10-08_drop_and_land): a loss that leaves fewer than min_survivors, a gap
>= 180 deg, a loss in LAND, or a tracker's /fleet/drop request releases every magnet, makes every
drone a departed drone that steps clear and lands, stops the OCP and latches /fleet/payload_dropped;
/fleet/landed follows once every drone is down."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DEPARTED_CLEAR_R, DissipativeController as D  # noqa: E402
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
        _dropped_pub=_Pub(), landed_pub=_Pub())
    f.get_logger = lambda: log
    f.log = log
    for m in ('_drop_due', '_drop_and_land', '_release_all_magnets', '_drop_tick',
              '_cancel_unload_quietly', '_drop_request_cb'):
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


def test_drop_tick_times_out_into_landed():
    f = _node()
    f._drop_and_land('test')
    f._drop_t = time.monotonic() - 31.0
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
