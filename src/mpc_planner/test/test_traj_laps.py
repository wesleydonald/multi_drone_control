"""traj_laps (twin only; card 2026-10-23_scalability case 7): a lapped path (the figure-8, line_spin, circle, spin) flies
its lap that many times, each from rest at the start point back to it, before it completes; line_spin and spin turn once
more each lap. One lap (the default) is today's path bit for bit: fixtures/load_trajectory_one_lap.json was written
from load_trajectory.py as it was before traj_laps (23 Oct)."""
import json
import os

import numpy as np
import pytest

from mpc_planner.load_trajectory import KINDS, LAPPED, LoadTrajectory

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures', 'load_trajectory_one_lap.json')


def _eval(tr, t):
    return [*tr.offset_at(t), *tr.accel_at(t), *tr.yaw_at(t)]


def test_one_lap_is_todays_path_bit_for_bit():
    with open(FIXTURE) as fh:
        cases = json.load(fh)['cases']
    assert {c['kind'] for c in cases} == set(KINDS)
    for c in cases:
        for tr in (LoadTrajectory(c['kind'], c['speed'], c['distance'], c['radius']),
                   LoadTrajectory(c['kind'], c['speed'], c['distance'], c['radius'], laps=1)):
            assert tr.laps == 1
            for row in c['rows']:
                t = row[0]
                assert [float(v) for v in _eval(tr, t)] == row[1:-1], (c['kind'], t)
                assert bool(tr.complete(t)) is row[-1], (c['kind'], t)


@pytest.mark.parametrize('kind, speed, distance, radius', [('fig_8', 0.2, 1.0, 0.5), ('line_spin', 0.2, 0.75, 0.5),
                                                           ('circle', 0.2, 1.0, 0.5), ('spin', 0.125, 1.0, 0.5)])
def test_each_lap_is_the_one_lap_path_from_rest_to_rest_and_the_last_completes(kind, speed, distance, radius):
    one = LoadTrajectory(kind, speed, distance, radius)
    tr = LoadTrajectory(kind, speed, distance, radius, laps=3)
    T = tr.lap_s()
    assert T == pytest.approx(one.lap_s()) and one.complete(T) and not one.complete(T - 1e-6)
    for t in np.linspace(0.05, T - 0.05, 37):
        for k in (1, 2):
            assert np.allclose(_eval(tr, t + k * T)[:6], _eval(one, t)[:6], atol=1e-9)
    for k in (1, 2):                                        # each lap ends and starts at rest on the start point
        for t in (k * T - 1e-9, k * T, k * T + 1e-9):
            assert np.allclose(_eval(tr, t)[:6], 0.0, atol=1e-6)
        assert not tr.complete(k * T + 1.0)
    assert not tr.complete(3 * T - 1e-6) and tr.complete(3 * T)
    assert np.allclose(_eval(tr, 3 * T + 7.0)[:6], 0.0, atol=1e-12)          # held at rest after the last lap


@pytest.mark.parametrize('kind, speed', [('line_spin', 0.2), ('spin', 0.125)])
def test_the_load_turns_once_more_each_lap(kind, speed):
    one = LoadTrajectory(kind, speed, 1.0, 0.5)
    tr = LoadTrajectory(kind, speed, 1.0, 0.5, laps=3)
    T = tr.lap_s()
    for t in np.linspace(0.05, T - 0.05, 23):
        for k in (1, 2):
            yaw, rate = tr.yaw_at(t + k * T)
            assert yaw == pytest.approx(one.yaw_at(t)[0] + 2.0 * np.pi * k) and rate == pytest.approx(one.yaw_at(t)[1])
    ys = [tr.yaw_at(t)[0] for t in np.linspace(1e-3, 3 * T - 1e-3, 3001)]
    assert all(b >= a - 1e-9 for a, b in zip(ys, ys[1:]))                      # one way, through every lap end
    for k in (1, 2):
        assert tr.yaw_at(k * T - 1e-9)[0] == pytest.approx(2.0 * np.pi * k, abs=1e-6)
        assert tr.yaw_at(k * T + 1e-9)[0] == pytest.approx(2.0 * np.pi * k, abs=1e-6)
    if kind == 'line_spin':
        assert tr.yaw_at(3 * T + 5.0) == (pytest.approx(6.0 * np.pi), 0.0)    # its turns held after the last lap
    else:
        assert tr.yaw_at(3 * T + 5.0) == (0.0, 0.0)                            # spin levels after it, as after one


def test_laps_are_whole_and_only_for_a_path_that_completes():
    for kind in KINDS:
        assert LoadTrajectory(kind, 0.2, 1.0, 0.5, laps=1).laps == 1
    for kind in LAPPED:
        assert LoadTrajectory(kind, 0.2, 1.0, 0.5, laps=9.0).laps == 9
    for laps in (0, -1, 1.5):
        with pytest.raises(ValueError, match='traj_laps'):
            LoadTrajectory('fig_8', 0.2, 1.0, 0.5, laps=laps)
    for kind in ('hover', 'line_x', 'orbit'):                                  # they never complete
        with pytest.raises(ValueError, match='traj_laps'):
            LoadTrajectory(kind, 0.2, 1.0, 0.5, laps=2)
        assert LoadTrajectory(kind, 0.2, 1.0, 0.5).lap_s() is None


def test_from_params_reads_the_laps_a_flight_flew():
    prm = {'load_traj': 'fig_8', 'traj_speed': 0.2, 'traj_radius': 0.5, 'fig8_size': 0.5, 'traj_laps': 9}
    tr = LoadTrajectory.from_params(prm)
    assert tr.laps == 9 and tr.lap_s() == pytest.approx(19.24, abs=0.01)
    assert tr.complete(9 * tr.lap_s()) and not tr.complete(8.5 * tr.lap_s())
    for v in (None, ''):                                                       # a params.json from before 23 Oct
        assert LoadTrajectory.from_params({**prm, 'traj_laps': v}).laps == 1
    assert LoadTrajectory.from_params({k: v for k, v in prm.items() if k != 'traj_laps'}).laps == 1
