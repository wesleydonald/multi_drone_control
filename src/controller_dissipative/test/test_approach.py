import numpy as np
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
