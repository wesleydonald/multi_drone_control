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
