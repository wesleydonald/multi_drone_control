"""tools/fake_mocap.py --script: the combined M1 contact attach on a slack tether, for the desk test.
The plate point must be the magnet manager's (ring pose + R(q) @ offset), the contact must rest still on it,
the caught case must raise the plate with the tip only once the tether is taut, and the uncaught case must
lift the tip off by more than the proof's d_fail."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fake_mocap import BALL_TOPIC, ContactScript, main  # noqa: E402


def _manager_plate(center, q, offset):
    """drone_magnet.magnet_attachment_manager._update_object_pose: centre + R(q) @ offset."""
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return np.asarray(center) + r @ np.asarray(offset)


def _script(kind, yaw_deg=0.0, plate_dz=0.0):
    # plate 3 (load-frame +90 deg), the rig radius, the 8 Oct body-to-tip
    return ContactScript(kind, (0.2, -0.1, 1.0), math.radians(yaw_deg), 0.225, math.radians(90.0),
                         plate_dz, 0.58)


@pytest.mark.parametrize('yaw', [0.0, 35.0])
@pytest.mark.parametrize('kind', ['caught', 'uncaught'])
def test_the_tip_rests_on_the_managers_plate_point(kind, yaw):
    s = _script(kind, yaw, plate_dz=0.031)
    for t in np.arange(s.t_contact, s.t_lift, 0.05):
        body, tip, q = s.poses(t)
        assert np.allclose(tip, _manager_plate(s.center, q, (0.0, 0.225, 0.031)), atol=1e-9)
    body, tip, q = s.poses(s.t_lift - 0.01)
    assert body[2] - tip[2] == pytest.approx(0.58 - 0.01)            # slack by the overtravel


def test_hanging_and_descending_keep_the_tether_taut():
    s = _script('caught')
    for t in (-5.0, 0.0, 0.5, s.t_contact - 1e-6):
        body, tip, _ = s.poses(t)
        assert body[2] - tip[2] == pytest.approx(0.58)
    assert s.poses(-5.0)[1][2] == pytest.approx(1.0 + 0.10)


def test_caught_the_plate_rises_only_once_taut_and_stops_at_the_lift():
    s = _script('caught')
    slack_end = s.t_lift + s.overtravel / s.rise_rate
    for t in np.arange(s.t_lift, slack_end - 1e-6, 0.05):
        body, tip, q = s.poses(t)
        assert tip[2] == pytest.approx(1.0) and q == pytest.approx(s.q_yaw)
        assert body[2] - tip[2] < 0.58
    body, tip, q = s.poses(slack_end + 10.0)
    assert tip[2] - 1.0 == pytest.approx(0.005, abs=1e-6)
    assert body[2] - tip[2] == pytest.approx(0.58)
    assert np.allclose(tip, _manager_plate(s.center, q, (0.0, 0.225, 0.0)), atol=1e-9)
    assert math.degrees(2 * math.acos(min(1.0, q[0]))) == pytest.approx(math.degrees(math.asin(0.005 / 0.225)),
                                                                          abs=1e-6)


def test_uncaught_the_tip_lifts_off_past_d_fail_and_the_plate_stays():
    s = _script('uncaught')
    body, tip, q = s.poses(s.t_lift + 10.0)
    assert q == pytest.approx(s.q_yaw)
    assert tip[2] - 1.0 == pytest.approx(0.03 - 0.01) and tip[2] - 1.0 > 0.008
    assert body[2] - tip[2] == pytest.approx(0.58)


def test_cli_dry_run_and_refusals(capsys):
    main(['--rig', '--num-drones', '3', '--attach', '--tip', '--azimuths-deg', '270,30,150', '--load-z', '1.0',
          '--tip-plate', '3', '--script', 'caught', '--ball', '1.0,1.6', '--dry-run'])
    out = capsys.readouterr().out
    assert BALL_TOPIC in out and 'script caught' in out
    with pytest.raises(SystemExit):
        main(['--rig', '--attach', '--tip', '--script', 'caught', '--dry-run'])    # no plate
