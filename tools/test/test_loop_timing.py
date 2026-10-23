"""tools/loop_timing.py and tools/pair_mode.py on synthetic data (card docs/experiments/2026-10-23_pair_mode_timing.md):
the delay fit recovers a known delay and lag, the system-momentum fit is not biased by the cables' internal pull, the
body rate of a constant spin, and the sign of the power flow."""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import loop_timing as lt  # noqa: E402
import pair_mode as pm  # noqa: E402


def noise(n, dt, hi_hz, seed):
    from scipy.signal import butter, sosfiltfilt
    x = np.random.default_rng(seed).normal(size=n)
    return sosfiltfilt(butter(4, hi_hz, fs=1.0 / dt, output='sos'), x)


def test_the_fit_recovers_a_37_ms_delay_and_a_15_ms_lag():
    dt = 0.001
    t = np.arange(0.0, 30.0, dt)
    x = noise(len(t), dt, 12.0, 1)
    y = lt.lpf1(np.r_[np.zeros(37), x[:-37]], 0.015, dt) * 1.3
    K, tau, T, r2 = lt.fit(t, x, t, y, 1.0, 29.0, (1.0, 10.0))
    assert abs(tau - 0.037) <= 0.002 and abs(T - 0.015) <= 0.002 and abs(K - 1.3) < 0.05 and r2 > 0.99


def test_a_held_command_at_50_hz_is_read_without_a_half_tick_bias():
    t50 = np.arange(0.0, 30.0, 0.02)
    cmd = noise(len(t50), 0.02, 6.0, 2)
    dt = 0.001
    tg = np.arange(0.0, 30.0, dt)
    y = lt.lpf1(lt.zoh(t50, cmd, tg - 0.025), 0.010, dt)            # the plant sees the held command 25 ms late
    _, tau, T, _ = lt.fit(t50, cmd, tg, y, 1.0, 29.0, (1.0, 10.0))
    assert abs(tau + T - 0.035) <= 0.003


def test_momentum_is_unbiased_by_an_internal_spring():
    """Two quads hold a ring on springs; each holds its command for a 50 Hz tick and its thrust acts 40 ms late
    through a 10 ms lag. The summed command against the system's momentum recovers it; the cables' pulls cancel."""
    dt, g, k, c = 0.001, 9.81, 400.0, 2.0
    t = np.arange(0.0, 40.0, dt)
    tick = slice(0, len(t), 20)
    F_cmd = [lt.M_Q * lt.KT * 0.55 + 1.5 * noise(len(t), dt, 4.0, s) for s in (3, 4)]
    F_cmd = [lt.zoh(t[tick], f[tick], t) for f in F_cmd]
    F_act = [lt.lpf1(np.r_[np.full(40, f[0]), f[:-40]], 0.010, dt) for f in F_cmd]
    z, v = np.array([1.0, 1.0, 0.5]), np.zeros(3)
    m = np.array([lt.M_Q, lt.M_Q, lt.M_RING])
    Z = np.empty((len(t), 3))
    for i in range(len(t)):
        pull = [k * (z[j] - z[2] - 0.5) + c * (v[j] - v[2]) for j in (0, 1)]
        a = np.array([F_act[0][i] - pull[0], F_act[1][i] - pull[1], pull[0] + pull[1]]) / m - g
        v += a * dt
        z += v * dt
        Z[i] = z
    ts = t[tick]
    off = 0.185
    logs = [pd.DataFrame({'sim_time': ts, 'pose_z': Z[tick, j], 'pose_qx': 0.0, 'pose_qy': 0.0,
                          'thr_out': off + F_cmd[j][tick] / (lt.M_Q * lt.KT), 'thr_off': off}) for j in (0, 1)]
    tg = np.arange(1.0, 39.0, 0.005)
    F, mom = lt.momentum_series(logs, tg, Z[tick, 2])
    ok = np.isfinite(mom)
    _, tau, T, r2 = lt.fit(tg, F, tg[ok], mom[ok], 2.0, 38.0, (1.0, 10.0), integrate=True)
    assert abs(tau + T - 0.050) <= 0.006 and r2 > 0.9


def test_body_rate_of_a_constant_spin():
    t = np.arange(0.0, 2.0, 0.01)
    w = np.array([0.3, -0.2, 1.1])
    ang = np.linalg.norm(w) * t
    axis = w / np.linalg.norm(w)
    q = np.c_[np.cos(ang / 2), np.sin(ang / 2)[:, None] * axis]
    q[50:] *= -1                                                     # a sign flip is the same attitude
    r = lt.body_rate(t, q)
    assert np.allclose(r[1:-1], w, atol=1e-3) and np.all(np.isnan(r[[0, -1]]))


def test_power_flow_sign():
    fs, f0 = 100.0, 1.5
    t = np.arange(0.0, 20.0, 1 / fs)
    V = np.sin(2 * np.pi * f0 * t)
    assert pm.damping(-2.0 * V, V, fs, f0)[0] > 1.9                  # a damper takes energy out
    assert pm.damping(+2.0 * V, V, fs, f0)[0] < -1.9                 # a push in phase feeds the swing
    assert abs(pm.damping(2.0 * np.cos(2 * np.pi * f0 * t), V, fs, f0)[0]) < 0.1   # in quadrature: neither
