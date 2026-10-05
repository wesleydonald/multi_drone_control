"""Yaw hold on a tilted floor start (rig yaw spins 2026-09-16, card docs/experiments/2026-09-29_rig_yaw_spin.md):
the yaw channel sends 0 from TAKEOFF until airborne or YAW_HOLD_MAX_S, only when the drone rested tilted past
YAW_HOLD_TILT_DEG; a level rest (every sim spawn) passes the solver's yaw through unchanged."""
import math
import types

import numpy as np
import pytest

cm = pytest.importorskip('tracker.tracker_node')


def _pose(tilt_deg, yaw_deg=168.7, z=0.07):
    """[x, y, z, qw, qx, qy, qz]: yaw about z, then a roll of tilt_deg (the rod lean)."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cr, sr = math.cos(math.radians(tilt_deg) / 2), math.sin(math.radians(tilt_deg) / 2)
    # q = q_yaw * q_roll
    qw, qx, qy, qz = cy * cr, cy * sr, sy * sr, sy * cr
    return np.array([0.0, 0.0, z, qw, qx, qy, qz])


def _node(tilt_deg):
    sent = []
    n = types.SimpleNamespace(
        takeoff_requested=True, takeoff_spool_s=0.0, _takeoff_step=None, drone_id=0,
        current_pose=_pose(tilt_deg), _kt_spawn_z=0.07, last_cmd_throttle=None,
        _yaw_hold=cm._rests_tilted(_pose(tilt_deg)), _yaw_hold_ticks=0,
        cb=types.SimpleNamespace(cmd_publisher_=types.SimpleNamespace(publish=sent.append)))
    n.get_logger = lambda: types.SimpleNamespace(debug=lambda *a, **k: None, info=lambda *a, **k: None)
    n._output_throttle = lambda thr, spool_frac=1.0: thr
    n._thrust_off = lambda: 0.0
    n._realised_model_throttle = lambda out: out
    cls = next(c for c in vars(cm).values() if isinstance(c, type) and hasattr(c, '_publish_channels'))
    n._kt_airborne = cls._kt_airborne.__get__(n)
    n.publish = lambda u: cls._publish_channels(n, np.asarray(u, float), np.zeros(4))
    return n, sent


@pytest.mark.parametrize('tilt, held', [(13.1, True), (11.8, True), (2.5, False), (0.0, False)])
def test_engages_only_on_a_tilted_rest(tilt, held):
    assert cm._rests_tilted(_pose(tilt)) is held


def test_tilted_rest_sends_yaw_zero_until_airborne_and_x0_sees_zero():
    n, sent = _node(13.1)
    for _ in range(10):
        n.publish([-0.3, -0.3, 0.45, -1.0])
    assert all(m.channel_3 == 0.0 for m in sent) and n._applied_u[3] == 0.0
    assert all(m.channel_0 == -0.3 for m in sent)                 # roll/pitch/throttle untouched
    n.current_pose = _pose(13.1, z=0.07 + cm.AIRBORNE_MARGIN + 0.01)
    n.publish([-0.3, -0.3, 0.45, -0.1])
    assert sent[-1].channel_3 == -0.1 and not n._yaw_hold          # released on the airborne gate
    n.current_pose = _pose(13.1, z=0.07)                           # back on the floor: not re-armed
    n.publish([0.0, 0.0, 0.3, -0.2])
    assert sent[-1].channel_3 == -0.2


def test_releases_after_the_timeout_below_the_gate():
    n, sent = _node(13.1)
    ticks = int(cm.YAW_HOLD_MAX_S * cm.FREQUENCY_HZ)
    for _ in range(ticks):
        n.publish([0.0, 0.0, 0.4, -0.5])
    assert sent[-1].channel_3 == 0.0
    n.publish([0.0, 0.0, 0.4, -0.5])
    assert sent[-1].channel_3 == -0.5 and not n._yaw_hold


def test_level_rest_is_unchanged():
    n, sent = _node(0.0)
    n.publish([0.1, -0.1, 0.45, -0.7])
    assert sent[-1].channel_3 == -0.7 and n._applied_u[3] == -0.7
