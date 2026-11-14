"""carriers_down_alone (card 2026-11-05_carriers_down_alone): while the LAND waits for a departed drone (QUAD4 taken
back), each carrier on the load gets /drone_d/carrier_released True once the touchdown came by the stall, the ring is down,
every carrier and the ring have been still on fresh poses for CARRIERS_STILL_S, and no take-back can still pull the
ring; once per LAND. Off: nothing is published."""
import time
import types

import numpy as np

from mpc_planner.planner_node import (CARRIERS_STILL_MPS, CARRIERS_STILL_S, LAND_POSE_FRESH_S, PLANNER_HZ,
                                      LoadPlanner)

HOLD = int(round(CARRIERS_STILL_S * PLANNER_HZ))


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg.data)


def _fake(on=True, n=3, carriers=None, by_stall=True, ring_down=True, busy=None):
    carriers = list(range(n)) if carriers is None else carriers
    ids = max(carriers) + 1
    now = time.monotonic()
    created = {}
    f = types.SimpleNamespace(
        _carriers_alone=on, _td_by_stall=by_stall, n=len(carriers), slot2drone=list(carriers) + [9],
        _carriers_released=set(), _carrier_pubs={}, _load_t=now, _carriers_still_ct=0, _carriers_busy_said=False,
        load_state=np.zeros(13), drone_pos=[np.array([1.0, 0.0, 0.10]) for _ in range(ids)],
        drone_vel=[np.zeros(3) for _ in range(ids)], _drone_t={d: now for d in range(ids)}, lines=[], busy=busy)
    f._departed_pubs = {d: _Pub() for d in range(ids)}          # the detach mode's: never used for a carrier
    f._load_down = lambda: ring_down
    f._slot_vel = lambda: LoadPlanner._slot_vel(f)
    f._take_back_busy = lambda: f.busy
    f.get_logger = lambda: types.SimpleNamespace(info=lambda s: f.lines.append(s))

    def create_publisher(_type, topic, _qos):
        created[topic] = _Pub()
        return created[topic]
    f.create_publisher = create_publisher
    f._carrier_release_pub = lambda d: LoadPlanner._carrier_release_pub(f, d)
    f.created = created
    return f


def _tick(f, k=1):
    for _ in range(k):
        f._load_t = time.monotonic()
        for d in f._drone_t:
            f._drone_t[d] = f._load_t
        LoadPlanner._carriers_down_tick(f)


def _sent(f):
    return {t: p.msgs for t, p in f.created.items()}


def test_off_publishes_nothing():
    f = _fake(on=False)
    _tick(f, 20)
    assert _sent(f) == {} and f._carriers_released == set() and f.lines == []


def test_on_releases_every_carrier_once_after_the_still_hold():
    f = _fake()
    _tick(f, HOLD - 1)
    assert _sent(f) == {}                                   # one still tick is not enough (critic, 5 Nov)
    _tick(f)
    assert _sent(f) == {f'/drone_{d}/carrier_released': [True] for d in range(3)}
    _tick(f, 20)
    assert _sent(f) == {f'/drone_{d}/carrier_released': [True] for d in range(3)}
    assert f._carriers_released == {0, 1, 2} and len(f.lines) == 1


def test_not_on_the_max_drop_exit_nor_with_the_ring_up():
    for f in (_fake(by_stall=False), _fake(ring_down=False)):
        _tick(f, 20)
        assert _sent(f) == {}


def test_any_motion_restarts_the_hold():
    f = _fake()
    _tick(f, HOLD - 1)
    f.drone_vel[1] = np.array([0.0, 0.0, -(CARRIERS_STILL_MPS + 0.01)])   # one carrier still descending
    _tick(f)
    f.drone_vel[1] = np.zeros(3)
    _tick(f, HOLD - 1)
    f.load_state[7:10] = [CARRIERS_STILL_MPS + 0.01, 0.0, 0.0]          # the ring moving
    _tick(f)
    f.load_state[7:10] = 0.0
    _tick(f, HOLD - 1)
    assert _sent(f) == {}
    _tick(f)
    assert sorted(_sent(f)) == [f'/drone_{d}/carrier_released' for d in range(3)]


def test_a_stale_pose_reads_as_not_still():
    for stale in ('drone', 'ring', 'none'):
        f = _fake()
        for _ in range(3 * HOLD):
            now = time.monotonic()
            f._load_t = now - (2 * LAND_POSE_FRESH_S if stale == 'ring' else 0.0)
            f._drone_t = {d: now for d in f._drone_t}
            if stale == 'drone':
                f._drone_t[2] = now - 2 * LAND_POSE_FRESH_S
            if stale == 'none':
                f.drone_vel[0] = None
            LoadPlanner._carriers_down_tick(f)
        assert _sent(f) == {}, stale


def test_held_while_a_take_back_can_pull_the_ring():
    f = _fake(busy="drone 4 (/drone_3)'s take-back is in its guard_follow")
    _tick(f, 3 * HOLD)
    assert _sent(f) == {} and len(f.lines) == 1 and 'held armed' in f.lines[0]
    f.busy = None
    _tick(f, HOLD)
    assert sorted(_sent(f)) == [f'/drone_{d}/carrier_released' for d in range(3)]


def test_only_the_drones_on_the_load():
    """slot2drone past n (a departed drone) is not released; QUAD4 folded in (id 3, on the load) is."""
    f = _fake(carriers=[0, 2, 3])
    _tick(f, HOLD)
    assert sorted(_sent(f)) == ['/drone_0/carrier_released', '/drone_2/carrier_released', '/drone_3/carrier_released']


def test_its_own_topic_never_the_departed_flag():
    """A departed drone's lone landing (/drone_d/departed, its -3..+10 cm band) is not touched."""
    f = _fake()
    _tick(f, HOLD)
    assert sorted(f.created) == [f'/drone_{d}/carrier_released' for d in range(3)]
    assert all(p.msgs == [] for p in f._departed_pubs.values())


def _land(on):
    f = _fake(on=on)
    f._carriers_released = {0, 1, 2}                      # an earlier LAND's
    f._td_by_stall, f._carriers_still_ct, f._carriers_busy_said = True, 3, True
    f.phase, f.takeoff_seen, f.start_taut, f._land_to_ground, f._landed = 'planner', True, True, False, False
    f.descending = False
    f._touchdown = types.SimpleNamespace(reset=lambda: None)
    f.get_logger = lambda: types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    LoadPlanner._fleet_command_cb(f, types.SimpleNamespace(data='LAND'))
    return f


def test_land_starts_fresh_and_makes_the_publishers_before_the_touchdown():
    f = _land(on=True)
    assert f._carriers_released == set() and f._td_by_stall is False
    assert f._carriers_still_ct == 0 and f._carriers_busy_said is False
    # made at LAND, False until the release (it clears a last flight's latched True)
    assert {t: p.msgs for t, p in f.created.items()} == {f'/drone_{d}/carrier_released': [False] for d in range(3)}
    g = _land(on=False)
    assert g.created == {}


def test_the_plain_planner_has_no_take_back():
    assert LoadPlanner._take_back_busy(types.SimpleNamespace()) is None
