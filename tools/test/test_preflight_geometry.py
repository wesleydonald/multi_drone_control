"""
Unit tests for the pure geometry of tools/preflight.py (tools/preflight_geometry.py) and
the pose math of tools/fake_mocap.py. The preflight is the rig's last check before ARM:
a fit or bearing bug here says GO on a drone sitting on the wrong plate.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_mocap as FM  # noqa: E402
from preflight_geometry import (ang_diff_deg, bearings_deg, fit_circle, match_azimuths,  # noqa: E402
                                spacing_deg, typed_azimuths, yaw_deg_from_quat)


def ring(centre, r, az_deg):
    return [(centre[0] + r * math.cos(math.radians(a)), centre[1] + r * math.sin(math.radians(a)))
            for a in az_deg]


@pytest.mark.parametrize('az', [[30, 150, 270], [30, 90, 150, 270], [0, 72, 144, 216, 288]])
def test_fit_recovers_centre_and_radius_for_any_n(az):
    cx, cy, r, rms = fit_circle(ring((0.12, -0.34), 0.58, az))
    assert (cx, cy, r) == pytest.approx((0.12, -0.34, 0.58), abs=1e-9)
    assert rms < 1e-9


def test_three_points_give_the_circumcircle_like_the_old_exact_solve():
    P = [(1.0, 0.0), (0.0, 2.0), (-1.5, -0.5)]
    cx, cy, r, _ = fit_circle(P)
    d = [math.hypot(x - cx, y - cy) for x, y in P]
    assert max(d) - min(d) < 1e-9 and r == pytest.approx(d[0])


def test_least_squares_with_noise_stays_close_and_reports_residual():
    rng = np.random.default_rng(0)
    P = np.array(ring((0.0, 0.0), 0.6, [30, 90, 150, 270])) + rng.normal(0, 0.005, (4, 2))
    cx, cy, r, rms = fit_circle(P)
    assert math.hypot(cx, cy) < 0.01 and abs(r - 0.6) < 0.01 and 0 < rms < 0.01


def test_degenerate_inputs_return_none():
    assert fit_circle([(0, 0), (1, 1)]) is None
    assert fit_circle([(0, 0), (1, 1), (2, 2), (3, 3)]) is None


def test_bearings_are_in_the_load_frame():
    P = ring((0.5, 0.5), 0.6, [60, 180, 300])          # plates 1/5/9 on a ring yawed +30 deg
    assert bearings_deg(P, (0.5, 0.5), yaw_deg=30.0) == pytest.approx([30, 150, 270])


def test_spacing_sums_to_360():
    g = spacing_deg([30, 90, 150, 270])
    assert sorted(g) == pytest.approx([60, 60, 120, 120]) and sum(g) == pytest.approx(360)


def test_ang_diff_wraps():
    assert ang_diff_deg(5, 355) == pytest.approx(10)
    assert ang_diff_deg(355, 5) == pytest.approx(-10)


def test_match_is_order_free_and_reports_the_error():
    typed = [30, 90, 150, 270]
    meas = [152.0, 28.0, 275.0, 91.0]                   # drones placed in another order
    slot2drone, err = match_azimuths(meas, typed)
    assert slot2drone == [1, 3, 0, 2]
    assert err == pytest.approx([-2, 1, 2, 5])


def test_a_drone_on_the_wrong_plate_shows_a_large_error():
    typed = [30, 150, 270]                              # 1/5/9 typed
    meas = [30.0, 120.0, 270.0]                         # drone 1 put on plate 4
    _, err = match_azimuths(meas, typed)
    assert max(abs(e) for e in err) == pytest.approx(30)


def test_typed_azimuths_parse_and_even_ring():
    assert typed_azimuths('30,90,150,270', 4) == [30, 90, 150, 270]
    assert typed_azimuths('', 3) == [0, 120, 240]
    assert typed_azimuths('even', 4) == [0, 90, 180, 270]
    with pytest.raises(ValueError):
        typed_azimuths('30,150', 3)


def test_yaw_from_quat():
    y = math.radians(40)
    assert yaw_deg_from_quat(math.cos(y / 2), 0, 0, math.sin(y / 2)) == pytest.approx(40)


# ── fake_mocap ────────────────────────────────────────────────────────────────

def test_fake_mocap_default_is_the_original_even_cone():
    poses = FM.compute_poses(3)
    el = math.radians(45)
    for i in range(3):
        (x, y, z), q = poses[f'/drone_{i}/motion_capture_state']
        a = 2 * math.pi * i / 3
        r = 0.25 + 0.5 * math.cos(el)
        assert (x, y, z) == pytest.approx((r * math.cos(a), r * math.sin(a), 0.05 + 0.5 * math.sin(el)))
        assert q == (1.0, 0.0, 0.0, 0.0)
    assert poses['/payload/motion_capture_state'] == ((0.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0))
    assert '/drone_3/motion_capture_state' not in poses


def test_fake_mocap_rig_layout_passes_the_preflight_geometry():
    """Plates 1/3/5/9, payload yawed 20 deg, drones resting with near-flat rods: the fit
    must find the payload origin, the rod, and the typed azimuths."""
    az = [30, 90, 150, 270]
    poses = FM.compute_poses(4, cable_len=0.47, azimuths_deg='30,90,150,270', yaw_deg=20.0,
                             drone_z=0.08, load_z=0.05)
    P = [poses[f'/drone_{i}/motion_capture_state'][0][:2] for i in range(4)]
    cx, cy, r, _ = fit_circle(P)
    assert math.hypot(cx, cy) < 1e-9
    assert r - 0.25 == pytest.approx(math.sqrt(0.47 ** 2 - 0.03 ** 2))
    q = poses['/payload/motion_capture_state'][1]
    meas = bearings_deg(P, (cx, cy), yaw_deg_from_quat(*q))
    slot2drone, err = match_azimuths(meas, az)
    assert slot2drone == [0, 1, 2, 3] and max(abs(e) for e in err) < 1e-9
    for i in range(4):                                  # rod length kept at cable_len
        (x, y, z), _ = poses[f'/drone_{i}/motion_capture_state']
        a = math.radians(az[i] + 20)
        rim = (0.25 * math.cos(a), 0.25 * math.sin(a), 0.05)
        assert math.dist((x, y, z), rim) == pytest.approx(0.47)


def test_fake_mocap_rejects_a_mismatched_azimuth_list():
    with pytest.raises(ValueError):
        FM.compute_poses(3, azimuths_deg='30,90')


def test_fake_mocap_dry_run_prints_without_ros(capsys):
    FM.main(['--num-drones', '3', '--azimuths-deg', '30,150,270', '--drone-z', '0.08', '--dry-run'])
    out = capsys.readouterr().out
    assert out.count('/drone_') == 3 and '/payload/motion_capture_state' in out
