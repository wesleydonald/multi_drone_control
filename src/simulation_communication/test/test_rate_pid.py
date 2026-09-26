import numpy as np

from simulation_communication.rate_pid import RatePid


def test_defaults_are_the_old_p_only_loop():
    pid = RatePid()
    e = np.array([10.0, -4.0, 2.0])
    for _ in range(50):
        out = pid.step(e, 0.01)
    assert np.allclose(out, 0.5 * e)


def test_integral_accumulates_per_second_and_clamps():
    pid = RatePid(kp=0.0, ki=10.0, i_limit=50.0)
    for _ in range(100):                      # 1 s of a 1 deg/s error
        out = pid.step([1.0, 0.0, 0.0], 0.01)
    assert abs(out[0] - 10.0) < 1e-9
    for _ in range(1000):
        out = pid.step([1.0, 0.0, 0.0], 0.01)
    assert out[0] == 50.0


def test_integral_resets_when_inactive():
    pid = RatePid(kp=0.0, ki=10.0)
    for _ in range(100):
        pid.step([5.0, 5.0, 5.0], 0.01)
    assert np.allclose(pid.step([5.0, 5.0, 5.0], 0.01, active=False), 0.0)
