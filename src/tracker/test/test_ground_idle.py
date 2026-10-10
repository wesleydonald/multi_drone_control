"""ground_idle: latches to armed idle only after the drone has flown and is back on the floor with the
ring resting and its reference at the floor, for GROUND_IDLE_HOLD_S; never before takeoff."""
import types

import numpy as np

from tracker.tracker_node import Controller, FREQUENCY_HZ, GROUND_IDLE_HOLD_S


def _stub(z, ref_z, resting=True, airborne=False):
    s = types.SimpleNamespace(ground_idle=True, current_pose=np.array([0.0, 0.0, z]), _kt_spawn_z=0.10,
                              _gi_airborne=airborne, _gi_ticks=0, _gi_latched=False, payload_resting=resting,
                              _current_ref_pos=np.array([0.0, 0.0, ref_z]), drone_id=0,
                              get_logger=lambda: types.SimpleNamespace(warn=lambda *a, **k: None))
    return s


def _run(s, seconds):
    out = False
    for _ in range(int(seconds * FREQUENCY_HZ)):
        out = Controller._ground_idle_update(s)
    return out


def test_never_before_takeoff():
    s = _stub(z=0.10, ref_z=0.0)                # on the floor, never flew
    assert _run(s, 2.0) is False and not s._gi_latched


def test_latches_after_landing_and_holds():
    s = _stub(z=0.10, ref_z=0.0, airborne=True)
    assert _run(s, GROUND_IDLE_HOLD_S + 0.1) is True
    s.current_pose = np.array([0.0, 0.0, 0.5])  # latched: stays idle even if it bounces
    assert Controller._ground_idle_update(s) is True


def test_not_while_the_ring_flies_or_the_reference_is_up():
    assert _run(_stub(z=0.10, ref_z=0.0, resting=False, airborne=True), 1.0) is False
    assert _run(_stub(z=0.10, ref_z=0.4, airborne=True), 1.0) is False


def test_a_flight_sets_the_airborne_latch():
    s = _stub(z=0.6, ref_z=0.6)
    Controller._ground_idle_update(s)
    assert s._gi_airborne


# ── a departed drone landing by itself while the ring flies (card 2026-10-09_departed_ground_idle) ──

def test_a_departed_drone_idles_at_its_rest_while_the_ring_flies():
    s = _stub(z=0.11, ref_z=-0.15, resting=False, airborne=True)
    s._departed = True
    assert _run(s, GROUND_IDLE_HOLD_S + 0.1) is True


def test_departed_needs_its_own_rest_height_both_ways():
    for z in (0.24, 0.05):                     # hovering light on its rod / below its rest: not at rest
        s = _stub(z=z, ref_z=-0.15, resting=False, airborne=True)
        s._departed = True
        assert _run(s, 1.0) is False


def test_departed_without_the_flag_or_before_flying_does_nothing():
    assert _run(_stub(z=0.10, ref_z=-0.15, resting=False, airborne=True), 1.0) is False


def test_departed_idles_even_if_it_never_rose_4_cm():
    """Critic pass 2 N1: a newcomer taken out on the floor must idle too (the flag implies TAKEOFF)."""
    s = _stub(z=0.10, ref_z=-0.15, resting=False)
    s._departed = True
    assert _run(s, GROUND_IDLE_HOLD_S + 0.1) is True


def test_the_flag_counts_only_after_this_arm_and_while_flying():
    s = types.SimpleNamespace(armed=False, takeoff_requested=False, _armed_at=None, _departed=False)
    Controller._departed_cb(s, types.SimpleNamespace(data=True))    # a latched True from an earlier flight
    assert s._departed is False
    s.armed, s.takeoff_requested, s._armed_at = True, True, 1
    Controller._departed_cb(s, types.SimpleNamespace(data=True))
    assert s._departed is True
    Controller._departed_cb(s, types.SimpleNamespace(data=False))   # the planner's False at ARM/TAKEOFF
    assert s._departed is False


# ── ...and is then disarmed alone by the fleet manager (card 2026-10-09_departed_disarm_alone) ──

class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _with_pub(s):
    s._down_alone_pub = _Pub()
    s.get_logger = lambda: types.SimpleNamespace(warn=lambda *a, **k: None, info=lambda *a, **k: None)
    return s


def test_a_departed_drone_at_its_rest_asks_once_to_be_disarmed_alone():
    s = _with_pub(_stub(z=0.11, ref_z=-0.15, resting=False, airborne=True))
    s._departed = True
    _run(s, GROUND_IDLE_HOLD_S + 1.0)
    assert [m.data for m in s._down_alone_pub.msgs] == [True]


def test_a_carrier_landing_with_the_ring_does_not_ask():
    s = _with_pub(_stub(z=0.10, ref_z=0.0, airborne=True))
    assert _run(s, GROUND_IDLE_HOLD_S + 1.0) is True
    assert s._down_alone_pub.msgs == []


def test_a_departed_drone_above_its_rest_does_not_ask():
    s = _with_pub(_stub(z=0.24, ref_z=-0.15, resting=False, airborne=True))
    s._departed = True
    _run(s, 2.0)
    assert s._down_alone_pub.msgs == []
