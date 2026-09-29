"""tools/thrust_check.py on synthetic free / hung-mass hovers of known airframes."""
import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
pytest.importorskip('controller_load_mpc.geometry')
import thrust_check  # noqa: E402


def _hover(folder, i, u):
    t = np.arange(0, 25, 0.02)
    ref = np.minimum(0.8, 0.1 + 0.15 * t)
    d = pd.DataFrame({'sim_time': t, 'u2': np.where(t > 0.5, u, 0.05), 'ref_z': ref, 'pose_z': ref - 0.01})
    p = os.path.join(folder, 'logs', 'controller_quad_load', f'planner_drone{i}_20260930_100000')
    os.makedirs(p)
    d.to_csv(os.path.join(p, 'log.csv'), index=False)


def test_recovers_mass_and_carry_throttle(tmp_path, capsys):
    m, dm, uf = 0.6, 0.15, 0.45
    for i in range(3):
        _hover(str(tmp_path / 'free'), i, uf)
        _hover(str(tmp_path / 'hung'), i, uf * (m + dm) / m)
    thrust_check.main(['--free', str(tmp_path / 'free'), '--loaded', str(tmp_path / 'hung'),
                       '--hung-kg', str(dm)])
    out = capsys.readouterr().out
    assert '0.600' in out and '21.8' in out            # implied mass, kT = g / 0.45
    assert 'three on 1/5/9' in out and 'OVER the cap' in out   # 0.6 kg carries at ~0.70


def test_hover_throttle_ignores_climb_and_takes_the_plateau(tmp_path):
    _hover(str(tmp_path), 0, 0.47)
    u, n, cap = thrust_check.hover_throttle(str(tmp_path / 'logs' / 'controller_quad_load' /
                                                 'planner_drone0_20260930_100000' / 'log.csv'))
    assert u == pytest.approx(0.47) and n > 500 and cap == 0.0
