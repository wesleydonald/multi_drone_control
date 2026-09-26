import math

import numpy as np

from tejen_mpc.lateral_disturbance_observer import (
    LateralDisturbanceObserver,
    nominal_lateral_acceleration,
    parse_xy_bias_mode,
)


def test_xy_bias_modes_are_explicit_and_mutually_named():
    assert parse_xy_bias_mode('legacy_integral') == 'legacy_integral'
    assert parse_xy_bias_mode('LATERAL_DISTURBANCE') == 'lateral_disturbance'
    assert parse_xy_bias_mode('lateral_disturbance_shadow') == 'lateral_disturbance_shadow'
    assert parse_xy_bias_mode('none') == 'none'


def test_nominal_lateral_acceleration_matches_level_hover():
    accel = nominal_lateral_acceleration([1.0, 0.0, 0.0, 0.0], 0.45, 22.0)
    assert np.allclose(accel, [0.0, 0.0], atol=1e-12)


def test_nominal_lateral_acceleration_matches_small_pitch_sign():
    pitch = math.radians(5.0)
    q = [math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0]
    accel = nominal_lateral_acceleration(q, 9.81 / 22.0, 22.0)
    assert accel[0] > 0.0
    assert abs(accel[1]) < 1e-12


def test_constant_acceleration_mismatch_converges_without_velocity_differentiation():
    dt = 1.0 / 30.0
    observer = LateralDisturbanceObserver(
        bandwidth_rad_s=0.30,
        max_abs_disturbance_mps2=2.0,
        max_dt_s=0.15,
    )
    true_disturbance = np.array([-0.40, 0.20])
    velocity = np.zeros(2)
    observer.update(velocity, np.zeros(2), dt)

    for _ in range(int(30.0 / dt)):
        velocity = velocity + dt * true_disturbance
        result = observer.update(velocity, np.zeros(2), dt)

    assert result['updated']
    assert np.allclose(result['disturbance_hat'], true_disturbance, atol=0.03)


def test_observer_does_not_aggressively_follow_point_eight_hz_swing_disturbance():
    dt = 1.0 / 30.0
    observer = LateralDisturbanceObserver(
        bandwidth_rad_s=0.30,
        max_abs_disturbance_mps2=2.0,
        max_dt_s=0.15,
    )
    velocity = np.zeros(2)
    observer.update(velocity, np.zeros(2), dt)

    true_history = []
    estimate_history = []
    for k in range(int(30.0 / dt)):
        t = k * dt
        disturbance = np.array([0.4 * math.sin(2.0 * math.pi * 0.8 * t), 0.0])
        velocity = velocity + dt * disturbance
        result = observer.update(velocity, np.zeros(2), dt)
        if t > 10.0:
            true_history.append(disturbance[0])
            estimate_history.append(result['disturbance_hat'][0])

    true_rms = float(np.sqrt(np.mean(np.square(true_history))))
    estimate_rms = float(np.sqrt(np.mean(np.square(estimate_history))))
    assert estimate_rms < 0.15 * true_rms


def test_invalid_long_dt_resets_estimate():
    observer = LateralDisturbanceObserver(bandwidth_rad_s=0.30, max_dt_s=0.15)
    observer.update([0.0, 0.0], [0.0, 0.0], 1.0 / 30.0)
    result = observer.update([0.1, 0.0], [0.0, 0.0], 0.5)
    assert not result['updated']
    assert not result['initialized']
    assert result['status'] == 'invalid_dt_reset'
    assert np.allclose(result['disturbance_hat'], [0.0, 0.0])
