"""ring_low_land_m (docs/analysis/x4_relay_fail2_2026-11-01.md defect 4, R3444): the ring far under its height
reference for ring_low_land_s after the lift lands the fleet once; off at 0, and never before the lift tops out or
during a LAND, a descent or a drop."""
from types import MethodType, SimpleNamespace

import numpy as np

from mpc_planner.planner_node import LoadPlanner


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.t * 1e9))


def _stub(m=0.20, s=3.0, z=1.0, **kw):
    sent, logs, clock = [], [], _Clock()
    st = SimpleNamespace(_ring_low_m=m, _ring_low_s=s, _ring_low_since=None, _ring_low_sent=False,
                         _ring_low_pub=SimpleNamespace(publish=lambda msg: sent.append(msg.data)),
                         phase='planner', lift_z0=0.02, target_z=1.0, lift_progress=0.98, takeoff_seen=True,
                         descending=False, _land_to_ground=False, load_state=np.r_[0.0, 0.0, z, 1.0, np.zeros(9)],
                         get_clock=lambda: clock, get_logger=lambda: SimpleNamespace(error=logs.append))
    for k, v in kw.items():
        setattr(st, k, v)
    st._ring_low_check = MethodType(LoadPlanner._ring_low_check, st)
    return st, sent, logs, clock


def _fly(st, clock, z, seconds, dt=0.05):
    for _ in range(int(round(seconds / dt))):
        st.load_state[2] = z
        st._ring_low_check()
        clock.t += dt


def test_lands_once_after_the_ring_stays_low_for_the_time():
    st, sent, logs, clock = _stub()
    _fly(st, clock, 0.75, 2.9)
    assert sent == []
    _fly(st, clock, 0.75, 0.3)
    assert sent == ['LAND'] and 'RING LOW' in logs[0]
    _fly(st, clock, 0.5, 5.0)
    assert sent == ['LAND']                      # once a flight


def test_a_dip_that_recovers_restarts_the_count():
    st, sent, _, clock = _stub()
    for _ in range(5):
        _fly(st, clock, 0.75, 2.0)               # a 25 cm dip for 2 s
        _fly(st, clock, 0.95, 0.5)               # back within 5 cm
    assert sent == []


def test_off_and_unarmed_cases_never_land():
    cases = [dict(m=0.0), dict(lift_progress=0.5), dict(descending=True), dict(_land_to_ground=True),
             dict(_dropped='tilt'), dict(phase='creep'), dict(takeoff_seen=False), dict(lift_z0=None)]
    for kw in cases:
        st, sent, _, clock = _stub(**kw)
        _fly(st, clock, 0.3, 6.0)
        assert sent == [], kw


def test_the_reference_is_the_lift_ramp_capped_at_the_target():
    st, sent, _, clock = _stub(lift_progress=1.5)  # past the top: the reference is target_z, not lift_z0 + progress
    _fly(st, clock, 0.85, 4.0)                     # 15 cm under 1.0: within 0.20
    assert sent == []
