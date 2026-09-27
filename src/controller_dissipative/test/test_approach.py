import numpy as np
import pytest
from controller_dissipative.approach import ApproachProfile, carrot_step, APPROACH_VEL, DESCENT_VEL, TIP_STANDOFF_M


def test_carrot_is_rate_bounded():
    p, v = carrot_step([0, 0, 0], [10, 0, 0], 0.2, 0.1)
    assert np.isclose(p[0], 0.02) and np.isclose(v[0], 0.2)


def test_carrot_stops_at_goal():
    p, v = carrot_step([0.99, 0, 0], [1, 0, 0], 0.2, 0.1)
    assert np.allclose(p, [1, 0, 0]) and v[0] < 0.2


def test_profile_transits_then_descends_onto_target():
    prof = ApproachProfile(p_start=[0.0, -1.2, 0.12], arm_len=0.5, dt=0.1)
    target = np.array([0.0, -0.25, 0.65])
    phases = []
    for _ in range(600):
        p, v, ph = prof.step(target)
        phases.append(ph)
        assert np.linalg.norm(v) <= APPROACH_VEL + DESCENT_VEL + 1e-9
    assert phases[0] == 'climb' and 'transit' in phases and phases[-1] == 'descend'
    assert np.allclose(p, target + [0, 0, 0.5 + TIP_STANDOFF_M], atol=1e-6)   # tip just above the target
    assert phases.index('climb') < phases.index('transit') < phases.index('descend')


def test_profile_climbs_before_moving_sideways():
    prof = ApproachProfile(p_start=[0.0, -1.2, 0.12], arm_len=0.5, dt=0.1)
    target = np.array([0.0, -0.25, 0.65])
    while prof.phase == 'climb':
        p, v, _ = prof.step(target)
        assert abs(p[0]) < 1e-9 and abs(p[1] + 1.2) < 1e-9        # no xy motion yet
    assert p[2] > target[2] + 0.5 + 0.25 - 0.06                   # at the clearance altitude


def test_seek_lowers_the_reference_below_the_weld_height_bounded():
    target = np.array([0.0, 0.0, 0.5])
    body = target[2] + 0.49 + TIP_STANDOFF_M
    prof = ApproachProfile(np.array([0.0, 0.0, body + 0.3]), 0.49, 0.1, seek_m=0.05)
    prof.phase = 'descend'
    for _ in range(200):
        p, v, _ = prof.step(target)
    assert p[2] == pytest.approx(body - 0.05, abs=1e-6)


def test_no_seek_by_default():
    target = np.array([0.0, 0.0, 0.5])
    body = target[2] + 0.49 + TIP_STANDOFF_M
    prof = ApproachProfile(np.array([0.0, 0.0, body + 0.3]), 0.49, 0.1)
    prof.phase = 'descend'
    for _ in range(200):
        p, v, _ = prof.step(target)
    assert p[2] == pytest.approx(body, abs=1e-6)


def test_seek_engages_on_a_jittering_target():
    body0 = 0.5 + 0.49 + TIP_STANDOFF_M
    prof = ApproachProfile(np.array([0.0, 0.0, body0 + 0.3]), 0.49, 0.1, seek_m=0.05)
    prof.phase = 'descend'
    for k in range(300):
        p, v, _ = prof.step(np.array([0.0, 0.0, 0.5 + 0.002 * (-1) ** k]))
    assert p[2] < body0 - 0.045


def test_moving_target_is_tracked_with_its_velocity_fed_forward():
    v_t = np.array([0.1, -0.05, 0.0])
    target = np.array([0.0, 0.0, 0.5])
    body = 0.5 + 0.49 + TIP_STANDOFF_M
    prof = ApproachProfile(np.array([0.0, 0.0, body + 0.1]), 0.49, 0.1)
    prof.phase = 'descend'
    for _ in range(300):
        target = target + v_t * 0.1
        p, v, _ = prof.step(target, v_t)
    assert np.allclose(p[:2], target[:2], atol=0.02)
    assert np.allclose(v, v_t, atol=1e-6)
