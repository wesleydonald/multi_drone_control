"""ROS-independent M2C ground-settle gate regression tests."""

from tejen_mission.m2c_ground_readiness import ContinuousSettleGate


def test_m2c_ground_settle_gate_requires_full_continuous_dwell():
    gate = ContinuousSettleGate(dwell_s=1.0)
    assert not gate.update(condition=True, now_s=10.0)
    assert not gate.update(condition=True, now_s=10.99)
    assert gate.update(condition=True, now_s=11.0)


def test_m2c_ground_settle_gate_resets_on_any_interruption():
    gate = ContinuousSettleGate(dwell_s=1.0)
    assert not gate.update(condition=True, now_s=4.0)
    assert not gate.update(condition=True, now_s=4.8)
    assert not gate.update(condition=False, now_s=4.9)
    assert not gate.update(condition=True, now_s=5.0)
    assert not gate.update(condition=True, now_s=5.99)
    assert gate.update(condition=True, now_s=6.0)
