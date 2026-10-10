"""The contact attach's target and start gate: ring_bridge's full-pose plate point equals our
magnet manager's weld point on the rig-twin world (plate 3 = +y at r 0.225, contact 0.032 above the
ring origin), and the supervisor's pickup release gate opens only on a held 'detached'."""
import math

import numpy as np
import pytest

from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.fake_cooperative_transport_world import RigidBodyState
from tejen_mission.ring_bridge import RingBridge
from tejen_mission.simulation_test_supervisor import PickupReleaseGate

CONTACT_Z = 0.032


def _bridge(full_pose, q):
    bridge = object.__new__(RingBridge)
    params = {'bridge_attach_z_m': CONTACT_Z, 'full_pose_attachment': full_pose}
    bridge._param = lambda name, default: params.get(name, default)
    bridge.ring_geometry = RingNetGeometry(plate_count=12, plate_pitch_diameter_m=0.45,
                                           collision_outer_diameter_m=0.51)
    bridge.ring_attachment_plate_index = 3
    bridge._ring_q = q
    return bridge


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def _state(yaw):
    return RigidBodyState(x=0.4, y=-0.3, z=1.0, yaw=yaw, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)


@pytest.mark.parametrize('roll_deg, pitch_deg, yaw_deg', [(8.0, -5.0, 20.0), (-12.0, 3.0, -35.0), (0.0, 0.0, 0.0)])
def test_full_pose_plate_point_equals_our_weld_point(roll_deg, pitch_deg, yaw_deg):
    manager = pytest.importorskip('drone_magnet.magnet_attachment_manager')
    q = _quat(*np.radians([roll_deg, pitch_deg, yaw_deg]))
    state = _state(math.radians(yaw_deg))
    ours = np.array([state.x, state.y, state.z]) + manager._quat_rot(list(q), [0.0, 0.225, CONTACT_Z])
    his = RingBridge.compute_attachment_xyz(_bridge(True, q), state)
    np.testing.assert_allclose(his, ours, atol=1e-12)


def test_switch_off_keeps_the_yaw_only_plate_point():
    q = _quat(*np.radians([8.0, -5.0, 20.0]))
    state = _state(math.radians(20.0))
    yaw_only = RingBridge.compute_attachment_xyz(_bridge(False, q), state)
    a = math.radians(20.0 + 90.0)
    np.testing.assert_allclose(yaw_only, [0.4 + 0.225 * math.cos(a), -0.3 + 0.225 * math.sin(a), 1.0 + CONTACT_Z],
                               atol=1e-12)
    # tilt matters: 8 deg of roll moves the plate point by ~r*sin(8 deg) in z
    full = RingBridge.compute_attachment_xyz(_bridge(True, q), state)
    assert abs(full[2] - yaw_only[2]) > 0.02
    # no attitude yet: yaw-only even with the switch on
    np.testing.assert_allclose(RingBridge.compute_attachment_xyz(_bridge(True, None), state), yaw_only)


# ---- the pickup release gate --------------------------------------------------------------

def _run(gate, t0, t1, dt=0.1):
    sent = []
    t = t0
    while t < t1 - 1e-9:
        command = gate.step(t)
        if command:
            sent.append((round(t, 3), command))
        t += dt
    return sent


def test_release_repeats_detach_until_a_held_detached_echo():
    gate = PickupReleaseGate()
    sent = _run(gate, 0.0, 3.0)
    assert [c for _, c in sent] == ['detach'] * 3 and not gate.open
    gate.observe(3.0, 'detached')
    assert _run(gate, 3.0, 3.95) == [] and not gate.open
    gate.step(4.0)
    assert gate.open
    assert _run(gate, 4.0, 20.0) == []


def test_release_forces_an_echo_when_the_joint_never_answers():
    gate = PickupReleaseGate()
    sent = _run(gate, 0.0, 11.0)
    commands = [c for _, c in sent]
    assert commands[:5] == ['detach'] * 5 and commands[5] == 'attach'
    assert commands.count('attach') == 2      # one forced pair per echo timeout
    gate.observe(11.0, 'attached')
    assert {c for _, c in _run(gate, 11.0, 30.0)} == {'detach'}


def test_a_re_weld_right_after_the_detach_keeps_the_gate_shut():
    gate = PickupReleaseGate()
    gate.step(0.0)
    gate.observe(0.5, 'detached')
    gate.observe(0.6, 'attached')         # a queued attach re-welds it
    assert [c for _, c in _run(gate, 1.0, 3.0)] == ['detach', 'detach']
    assert not gate.open
    gate.observe(3.0, 'detached')
    gate.step(4.01)
    assert gate.open
