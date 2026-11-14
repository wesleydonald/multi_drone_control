"""carriers_down_alone (card 2026-11-05_carriers_down_alone, rev 3): a carrier the planner released on the landed ring
(/drone_d/carrier_released) idles and asks once to be disarmed alone within its own band, -0.03..+carriers_down_band_m
(0.15 m, Wesley 5 Nov) of its resting height, its reference at the floor and its whole speed <= 0.05 m/s for
GROUND_IDLE_HOLD_S, whether or not it had already latched its idle. A departed drone's lone landing (/drone_d/departed,
-3..+10 cm) is exactly the committed code, checked tick by tick against a frozen copy of it."""
import types

import numpy as np
import pytest

from std_msgs.msg import Bool
from tracker.tracker_node import (AIRBORNE_MARGIN, CARRIERS_DOWN_BAND_M, DEPARTED_IDLE_MARGIN, FREQUENCY_HZ,
                                  GROUND_IDLE_HOLD_S, GROUND_IDLE_MARGIN, Controller)

REST = 0.10


def _drone(z=REST, ref_z=-0.3, latched=False, on=True, band=CARRIERS_DOWN_BAND_M):
    s = types.SimpleNamespace(
        ground_idle=True, current_pose=np.array([0.0, 0.0, z, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), _kt_spawn_z=REST,
        _gi_airborne=True, _gi_ticks=0, _gi_latched=latched, payload_resting=True, carriers_down_alone=on,
        carriers_down_band_m=band, _current_ref_pos=np.array([0.0, 0.0, ref_z]), drone_id=0, _departed=False,
        _carrier_released=False, _carrier_ticks=0, _carrier_asked=False, armed=True, takeoff_requested=True,
        _armed_at=1, keep_alive_on_disarm=False, sent=[], warned=[])
    s._down_alone_pub = types.SimpleNamespace(publish=lambda m: s.sent.append(m.data))
    s.get_logger = lambda: types.SimpleNamespace(warn=lambda m, **k: s.warned.append(m), info=lambda *a, **k: None)
    s._carrier_down_tick = lambda: Controller._carrier_down_tick(s)
    return s


def _run(s, seconds, update=Controller._ground_idle_update):
    out = None
    for _ in range(int(round(seconds * FREQUENCY_HZ))):
        out = update(s)
    return out


def _release(s):
    Controller._carrier_released_cb(s, types.SimpleNamespace(data=True))


@pytest.mark.parametrize('latched', [False, True])
def test_a_carrier_stopped_12_cm_up_is_idled_and_disarmed(latched):
    s = _drone(z=REST + 0.12, latched=latched)
    assert _run(s, 1.0) is latched and s.sent == []          # +12 cm is outside the ring-down idle's 3 cm
    _release(s)
    _run(s, GROUND_IDLE_HOLD_S - 1.0 / FREQUENCY_HZ)
    assert s.sent == []
    assert _run(s, 1.0 / FREQUENCY_HZ) is True
    assert s.sent == [True] and s.keep_alive_on_disarm and s._gi_latched
    _run(s, 2.0)
    assert s.sent == [True]                                 # once


@pytest.mark.parametrize('latched', [False, True])
def test_a_carrier_18_cm_up_stays_armed(latched):
    s = _drone(z=REST + 0.18, latched=latched)
    _release(s)
    assert _run(s, 3.0) is latched and s.sent == [] and not s.keep_alive_on_disarm


def test_the_band_edges():
    for dz, asks in ((CARRIERS_DOWN_BAND_M - 0.005, True), (CARRIERS_DOWN_BAND_M + 0.005, False),
                     (-GROUND_IDLE_MARGIN + 0.005, True), (-GROUND_IDLE_MARGIN - 0.005, False)):
        s = _drone(z=REST + dz)
        _release(s)
        _run(s, 1.0)
        assert (s.sent == [True]) is asks, dz


def test_not_moving_and_not_with_its_reference_up():
    for v, ref_z in (((0.0, 0.0, 0.08), -0.3), ((0.06, 0.0, 0.0), -0.3), ((0.0, 0.0, 0.0), REST + 0.20)):
        s = _drone(ref_z=ref_z)
        s.current_pose[7:10] = v
        _release(s)
        _run(s, 2.0)
        assert s.sent == [], (v, ref_z)


def test_the_hold_restarts_when_it_moves():
    s = _drone()
    _release(s)
    _run(s, GROUND_IDLE_HOLD_S / 2)
    s.current_pose[9] = 0.2
    _run(s, 1.0 / FREQUENCY_HZ)
    s.current_pose[9] = 0.0
    _run(s, GROUND_IDLE_HOLD_S - 1.0 / FREQUENCY_HZ)
    assert s.sent == []
    _run(s, 1.0 / FREQUENCY_HZ)
    assert s.sent == [True]


def test_no_release_no_ask_and_a_flag_before_this_arm_does_not_count():
    s = _drone(z=REST + 0.05)
    _run(s, 2.0)
    assert s.sent == []                                     # never released: as today
    t = _drone(z=REST + 0.05)
    t._armed_at = None                                      # a latched True from an earlier flight
    _release(t)
    _run(t, 2.0)
    assert not t._carrier_released and t.sent == []
    u = _drone()
    _release(u)
    Controller._carrier_released_cb(u, types.SimpleNamespace(data=False))
    _run(u, 2.0)
    assert u.sent == []


def test_a_typed_band():
    s = _drone(z=REST + 0.08, band=0.06)
    _release(s)
    _run(s, 1.0)
    assert s.sent == []


# ── QUAD4's departed band (-3..+10 cm) is exactly the committed code (week8 9f20f8b, frozen here) ─────────────────

def _departed_cb_before(self, msg):
    if not msg.data:
        self._departed = False
    elif self.armed and self.takeoff_requested and self._armed_at is not None:
        self._departed = True


def _ground_idle_update_before(self):
    if not self.ground_idle or self.current_pose is None or self._kt_spawn_z is None:
        return False
    if self._gi_latched:
        return True
    z = float(self.current_pose[2])
    if z >= self._kt_spawn_z + AIRBORNE_MARGIN:
        self._gi_airborne = True
    ref = self._current_ref_pos
    at_rest_ref = ref is not None and float(ref[2]) <= self._kt_spawn_z + GROUND_IDLE_MARGIN
    down = (self._gi_airborne and self.payload_resting
            and z <= self._kt_spawn_z + GROUND_IDLE_MARGIN and at_rest_ref)
    vz = float(self.current_pose[9]) if len(self.current_pose) > 9 else 0.0
    alone = (getattr(self, '_departed', False) and ref is not None
             and float(ref[2]) <= self._kt_spawn_z + DEPARTED_IDLE_MARGIN
             and -GROUND_IDLE_MARGIN <= z - self._kt_spawn_z <= DEPARTED_IDLE_MARGIN and abs(vz) <= 0.05)
    down = down or alone
    self._gi_ticks = self._gi_ticks + 1 if down else 0
    if self._gi_ticks >= GROUND_IDLE_HOLD_S * FREQUENCY_HZ:
        self._gi_latched = True
        self.get_logger().warn('on the floor after the flight')
        if alone and getattr(self, '_down_alone_pub', None) is not None:
            self.keep_alive_on_disarm = True
            self._down_alone_pub.publish(Bool(data=True))
    return self._gi_latched


def _trace(s, cb, update, flag_at, zs):
    """Per tick: (idle, sent, keep_alive, latched, departed) while z follows zs, the departed flag sent at flag_at."""
    out = []
    for k, z in enumerate(zs):
        if k == flag_at:
            cb(s, types.SimpleNamespace(data=True))
        s.current_pose[2] = z
        out.append((update(s), list(s.sent), s.keep_alive_on_disarm, s._gi_latched, s._departed))
    return out


@pytest.mark.parametrize('on', [False, True])
@pytest.mark.parametrize('latched, zs', [
    (True, [REST] * 60),                                    # idling on the floor when a departed flag comes
    (False, [REST + 0.12] * 60),                            # QUAD4 held 12 cm up: outside its band, never idles
    (False, [REST + 0.07] * 60),                            # 7 cm up (14 Oct): inside it
    (False, list(np.linspace(0.60, REST, 30)) + [REST] * 40),   # its lone landing, the flag in the air
    (False, [REST + 0.01] * 60)])
def test_the_departed_band_is_the_committed_code(on, latched, zs):
    for flag_at in (0, 5, 25, 45):
        new = _drone(z=zs[0], latched=latched, on=on)
        old = _drone(z=zs[0], latched=latched, on=on)
        a = _trace(new, Controller._departed_cb, Controller._ground_idle_update, flag_at, zs)
        b = _trace(old, _departed_cb_before, _ground_idle_update_before, flag_at, zs)
        assert a == b, (on, latched, flag_at)
    if zs[0] == REST + 0.12:
        assert a[-1][1] == []                               # QUAD4 at +12 cm: not disarmed alone (its 10 cm band)
