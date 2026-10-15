"""ring_bridge attach_z_lowpass_hz: off passes the plate height through; on, a 1.8 Hz bounce (a1, 15 Oct) is cut to
about a tenth while a steady height comes through unchanged; only the present, measured plate is filtered."""
import math
import types

import numpy as np

from tejen_mission.fake_cooperative_transport_world import RigidBodyState
from tejen_mission.ring_bridge import RingBridge

DT = 0.02          # the bridge's 50 Hz timer


def _stub(fc, ring=None):
    clock = {'t': 100.0}
    params = {'attach_z_lowpass_hz': fc, 'bridge_attach_z_m': 0.024, 'full_pose_attachment': True}
    s = types.SimpleNamespace(_zf=None, _zf_t=None, _ring=ring, _ring_q=(1.0, 0.0, 0.0, 0.0),
                              _param=lambda name, default: params.get(name, default),
                              _plate_xy=lambda: [0.013, 0.210],
                              get_clock=lambda: types.SimpleNamespace(
                                  now=lambda: types.SimpleNamespace(nanoseconds=int(clock['t'] * 1e9))))
    s._lowpass_z = lambda z: RingBridge._lowpass_z(s, z)
    return s, clock


def _run(fc, signal, seconds=20.0):
    s, clock = _stub(fc)
    out = []
    for k in range(int(seconds / DT)):
        clock['t'] += DT
        out.append(RingBridge._lowpass_z(s, signal(k * DT)))
    return np.array(out)


def test_off_is_raw():
    sig = lambda t: 1.0 + 0.01 * math.sin(2 * math.pi * 1.8 * t)  # noqa: E731
    out = _run(0.0, sig, 2.0)
    assert np.allclose(out, [sig(k * DT) for k in range(len(out))])


def test_bounce_cut_steady_kept():
    out = _run(0.6, lambda t: 1.0 + 0.01 * math.sin(2 * math.pi * 1.8 * t))
    tail = out[len(out) // 2:]
    assert abs(tail.mean() - 1.0) < 1e-3
    assert tail.std() < 0.15 * 0.01 / math.sqrt(2)


def test_only_the_present_plate_is_filtered():
    ring = RigidBodyState(x=0.0, y=0.0, z=1.0, yaw=0.0, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    s, clock = _stub(0.6, ring)
    RingBridge.compute_attachment_xyz(s, ring)                     # starts the filter at 1.024
    clock['t'] += DT
    ring.z = 1.05
    z_now = RingBridge.compute_attachment_xyz(s, ring)[2]
    ahead = RigidBodyState(x=0.0, y=0.0, z=1.05, yaw=0.0, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    z_ahead = RingBridge.compute_attachment_xyz(s, ahead)[2]
    assert z_now < 1.05 + 0.024 - 0.01                             # lagged
    assert abs(z_ahead - (1.05 + 0.024)) < 1e-9                    # the predicted path is not filtered


def test_zero_twist_only_when_typed():
    """attach_twist 'zero' (card 2026-10-16_plate_height_filter, critic F4): the bridge's attachment twist is zero;
    otherwise the base's finite difference."""
    from tejen_mission.fake_cooperative_transport_world import FakeCooperativeTransportWorld as Base
    state = RigidBodyState(x=0.0, y=0.0, z=1.0, yaw=0.0, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    for mode, zero in (('', False), ('zero', True)):
        s, _ = _stub(0.0)
        s._param = lambda name, default, mode=mode: mode if name == 'attach_twist' else default
        s.last_attachment_xyz, s.last_payload_state, s.last_time_sec = (0.0, 0.0, 1.0), state, 1.0
        got = RingBridge.finite_difference_attachment_twist(s, 1.02, state, (0.0, 0.0, 1.002))
        if zero:
            assert got == (0.0, 0.0, 0.0, 0.0)
        else:
            assert abs(got[2] - Base.finite_difference_attachment_twist(s, 1.02, state, (0.0, 0.0, 1.002))[2]) < 1e-12
            assert abs(got[2] - 0.1) < 1e-9


def test_zero_twist_never_while_the_ring_flies_a_trajectory():
    """Critic F3 (zero twist): a moving ring keeps its velocity (his references need it); zero only in hover."""
    state = RigidBodyState(x=0.0, y=0.0, z=1.0, yaw=0.0, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    s, _ = _stub(0.0)
    s._param = lambda name, default: 'zero' if name == 'attach_twist' else default
    s.last_attachment_xyz, s.last_payload_state, s.last_time_sec = (0.0, 0.0, 1.0), state, 1.0
    s._traj_state = {'running': True, 'kind': 'circle', 'speed': 0.125}
    assert abs(RingBridge.finite_difference_attachment_twist(s, 1.02, state, (0.0, 0.0, 1.002))[2] - 0.1) < 1e-9
    for hover in ({'running': False, 'kind': 'circle', 'speed': 0.125},
                  {'running': True, 'kind': 'hover', 'speed': 0.0}):      # a lifted hover: the trajectory clock runs
        s._traj_state = hover
        assert RingBridge.finite_difference_attachment_twist(s, 1.02, state, (0.0, 0.0, 1.002)) == (0.0, 0.0, 0.0, 0.0)


def test_lowpass_twist_keeps_a_slow_drift_and_cuts_the_bounce():
    """attach_twist 'lowpass' at 0.5 Hz (zero-twist critic F2): a 1.8 Hz velocity is cut to under a fifth, a 0.1 Hz
    drift velocity passes at over 0.9."""
    for f, lo, hi in ((1.8, 0.0, 0.2), (0.1, 0.9, 1.05)):
        s, _ = _stub(0.0)
        s._vf, s._vf_t = None, None
        s._param = lambda name, default: 'lowpass' if name == 'attach_twist' else default
        out = []
        for k in range(int(40.0 / DT)):
            t = 100.0 + k * DT
            v = math.sin(2 * math.pi * f * t)
            out.append(RingBridge._lowpass_twist(s, t, (v, 0.0, 0.0, 0.0))[0])
        tail = np.array(out[len(out) // 2:])
        amp = tail.std() * math.sqrt(2)
        assert lo <= amp <= hi, (f, amp)


def test_lowpass_twist_only_in_hover():
    """attach_twist 'lowpass' leaves the twist raw while the ring flies a trajectory."""
    from tejen_mission.fake_cooperative_transport_world import FakeCooperativeTransportWorld as Base
    raw = (0.1, -0.2, 0.05, 0.0)
    orig = Base.finite_difference_attachment_twist
    Base.finite_difference_attachment_twist = lambda self, t, p, a: raw
    try:
        s, _ = _stub(0.0)
        s._vf, s._vf_t = None, None
        s._param = lambda name, default: 'lowpass' if name == 'attach_twist' else default
        s._traj_state = {'running': True, 'kind': 'circle', 'speed': 0.125}
        for k in range(50):
            assert RingBridge.finite_difference_attachment_twist(s, 100.0 + k * DT, None, None) == raw
    finally:
        Base.finite_difference_attachment_twist = orig
