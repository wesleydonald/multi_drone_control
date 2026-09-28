"""Body rate for the MPC initial state from the stamped gyro (sim oracle).

Card docs/experiments/2026-09-28_mocap_diff_window.md, v3: with x0_rate_source 'imu' the
tracker's x0 body rate is the mean of the /drone_i/imu samples received since the previous
control tick instead of the mocap-differenced w. Both are body FLU rad/s.
"""
import numpy as np

X0_RATE_SOURCES = ('mocap', 'imu')
X0_RATE_MAX_AGE_S = 0.05


def x0_body_rate(w_sum, n, held, newest_stamp_s, now_s, mocap_w,
                 max_age_s=X0_RATE_MAX_AGE_S):
    """(w for x0, new held mean, fell back to mocap).

    w_sum, n: sum and count of the gyro samples received since the previous tick; held:
    the last mean (None before the first sample). The mean is held while the newest
    sample's stamp is at most max_age_s behind the node clock, else mocap w is used.
    """
    if n > 0:
        held = np.asarray(w_sum, float) / n
    if held is None or newest_stamp_s is None or now_s - newest_stamp_s > max_age_s:
        return np.asarray(mocap_w, float), held, True
    return held, held, False
