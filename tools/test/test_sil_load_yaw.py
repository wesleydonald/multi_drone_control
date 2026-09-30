"""SIL scenario `initial: load_yaw_deg`: the ring spawns yawed with every drone on its rotated
slot and every rod at its length (the A4 quaternion check flies the ring at yaw 180)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sil.scenario import Scenario  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _scn(yaw):
    s = Scenario.from_yaml(os.path.join(REPO, 'configs/sil/carry_hover_n3.yaml'))
    s.load_yaw_deg = yaw
    return s


def test_zero_yaw_is_the_old_spawn():
    s = _scn(0.0)
    assert np.array_equal(s.load_R(), np.eye(3))


def test_yawed_spawn_keeps_the_rods_taut_on_rotated_slots():
    for yaw in (30.0, 179.0, 180.0):
        s = _scn(yaw)
        pos, load = s.initial_state()
        R = s.load_R()
        for p, rho in zip(pos, s.attach_rho()):
            assert abs(np.linalg.norm(p - (load + R @ rho)) - s.cable_len) < 1e-12
        p0, _ = _scn(0.0).initial_state()
        np.testing.assert_allclose(np.array(pos)[:, :2], np.array(p0)[:, :2] @ R[:2, :2].T, atol=1e-12)
