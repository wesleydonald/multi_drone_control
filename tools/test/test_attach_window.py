"""tools/attach_window.py on synthetic tracks: the tip error, the catch window, the swing and its forcing."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import attach_window as aw  # noqa: E402


def tracks(tip_xy, body_xy, dur=10.0, yaw=0.0, ring_xy=(0.5, -0.4), tilt_deg=0.0):
    """100 Hz tracks: tip_xy(t) and body_xy(t) world xy functions, the ring still at ring_xy with this yaw and tilt."""
    t = np.arange(0.0, dur, 0.01)
    h = math.radians(tilt_deg) / 2
    q = (math.sin(h) * math.cos(yaw / 2), 0.0, math.cos(h) * math.sin(yaw / 2), math.cos(h) * math.cos(yaw / 2))
    ring = np.array([(s, ring_xy[0], ring_xy[1], 1.0) + q for s in t])
    tip = np.array([(s, *tip_xy(s), 1.03) for s in t])
    body = np.array([(s, *body_xy(s), 1.60) for s in t])
    return {'tip': tip, 'ring': ring, 'body': body, 'phases': [(0.0, 'MATCH_VELOCITY'), (6.0, 'ATTACH_READY')]}


def plate_world(yaw, ring_xy=(0.5, -0.4), plate=aw.PLATE_XY):
    c, s = math.cos(yaw), math.sin(yaw)
    return ring_xy[0] + c * plate[0] - s * plate[1], ring_xy[1] + s * plate[0] + c * plate[1]


def test_a_still_tip_1_cm_off_the_plate_is_inside_and_slow():
    yaw = math.radians(30)
    px, py = plate_world(yaw)
    d = tracks(lambda s: (px + 0.01, py), lambda s: (px + 0.01, py), yaw=yaw)
    m = aw.window_metrics(d, 1.0, 9.0)
    assert abs(m['err_median_cm'] - 1.0) < 0.05 and abs(m['offset_x_cm'] - 1.0) < 0.05
    assert m['inside_pct'] == 100.0 and m['inside_slow_pct'] == 100.0
    assert m['swing_rms_cm'] < 0.05 and m['body_sd_cm'] < 0.05 and m['plate_sd_cm'] < 0.05


def test_a_swinging_tip_leaves_the_catch_window():
    px, py = plate_world(0.0)
    f = 0.65                                           # ~ a 0.57 m pendulum
    d = tracks(lambda s: (px + 0.04 * math.sin(2 * math.pi * f * s), py), lambda s: (px, py))
    m = aw.window_metrics(d, 1.0, 9.0)
    assert 2.7 < m['swing_rms_cm'] < 2.9                  # 4 cm amplitude: rms 2.83 cm
    assert m['inside_pct'] < 60.0 and m['inside_slow_pct'] < m['inside_pct']
    assert 0.55 < m['l_eff_m'] < 0.60 and m['body_acc_rms'] < 0.01


def test_tilt_and_phase_windows():
    px, py = plate_world(0.0)
    d = tracks(lambda s: (px, py), lambda s: (px, py), tilt_deg=6.0)
    m = aw.window_metrics(d, 1.0, 5.0)
    assert abs(m['tilt_mean_deg'] - 6.0) < 0.01
    w = aw.phase_windows([(0.0, 'TAKEOFF'), (2.0, 'MATCH_VELOCITY'), (9.0, 'DESCEND_TO_ATTACHMENT'),
                          (9.2, 'ATTACH_READY'), (12.0, 'TAKEN_BACK')], end_t=20.0)
    assert w == [(2.0, 9.0, 'MATCH_VELOCITY'), (9.2, 12.0, 'ATTACH_READY')]     # the 0.2 s descent is too short


def test_missing_streams_give_partial_rows():
    px, py = plate_world(0.0)
    d = tracks(lambda s: (px, py), lambda s: (px, py))
    d['body'] = np.zeros((0, 4))
    m = aw.window_metrics(d, 1.0, 5.0)
    assert 'err_median_cm' in m and 'swing_rms_cm' not in m
    d['tip'] = np.zeros((0, 4))
    assert set(aw.window_metrics(d, 1.0, 5.0)) == {'t0', 'dur_s', 'n_tip'}
