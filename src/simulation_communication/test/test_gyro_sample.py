import math

import numpy as np

from simulation_communication.rate_pid import RatePid, gyro_sample


def test_first_sample_has_no_dt_but_converts_to_deg():
    dt, w, t_prev = gyro_sample(10.0, None, (math.pi, -math.pi / 2, 0.0))
    assert dt is None and t_prev == 10.0
    assert np.allclose(w, [180.0, -90.0, 0.0])


def test_dt_is_the_stamp_difference():
    dt, w, t_prev = gyro_sample(10.002, 10.0, (0.0, 0.0, 1.0))
    assert abs(dt - 0.002) < 1e-12 and t_prev == 10.002
    assert abs(w[2] - math.degrees(1.0)) < 1e-12


def test_repeated_or_stale_stamp_is_skipped_and_t_prev_holds():
    assert gyro_sample(10.0, 10.0, (0.0, 0.0, 0.0))[::2] == (None, 10.0)
    assert gyro_sample(9.998, 10.0, (0.0, 0.0, 0.0))[::2] == (None, 10.0)


def test_out_of_order_sample_does_not_double_count_the_next_dt():
    _, _, t_prev = gyro_sample(10.002, 10.0, (0, 0, 0))
    _, _, t_prev = gyro_sample(10.0, t_prev, (0, 0, 0))     # late copy of the old sample
    dt, _, _ = gyro_sample(10.004, t_prev, (0, 0, 0))
    assert abs(dt - 0.002) < 1e-12


def test_clock_reset_restarts():
    dt, _, t_prev = gyro_sample(0.001, 120.0, (0, 0, 0))
    assert dt is None and t_prev == 0.001


def test_stream_with_duplicates_integrates_real_time_only():
    # a 1 deg/s error for 1 s at 500 Hz, every sample delivered twice
    pid = RatePid(kp=0.0, ki=10.0)
    t_prev = None
    for k in range(501):
        for _ in range(2):
            dt, _, t_prev = gyro_sample(k * 0.002, t_prev, (0.0, 0.0, 0.0))
            if dt is not None:
                out = pid.step([1.0, 0.0, 0.0], dt)
    assert abs(out[0] - 10.0) < 1e-9
