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
