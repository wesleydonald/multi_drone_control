"""tools/thrust_fit.py: the 13-point ladder fit (RIG-0930-ladder4) and the tethered rod-force balance."""
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import thrust_fit  # noqa: E402

SPEC = os.path.join(os.path.dirname(__file__), 'fixtures', 'rig_0930', 'ladder_points.yaml')


def test_reproduces_the_ladder4_fit():
    law = thrust_fit.fit(thrust_fit.read_spec(SPEC))
    assert law['n'] == 13
    assert law['a'] == pytest.approx([0.181, 0.185, 0.187, 0.187], abs=0.002)
    assert law['b'] == pytest.approx(0.507, abs=0.005)
    assert law['c'] == pytest.approx(-0.0219, abs=0.002)
    assert law['rms'] == pytest.approx(0.0032, abs=0.0003)
    assert law['max'] <= 0.0055


def _launch(folder, frac, affine, tilt_deg=12.0, stamp='20260930_120000'):
    """A planner log and four tracker logs whose throttles carry `frac` of the ring by the 0930 law."""
    law, g, md = thrust_fit.LAW_0930, thrust_fit.G, thrust_fit.DRONE_KG
    root = os.path.join(folder, 'logs', 'controller_quad_load')
    t = 1.79e9 + np.arange(0, 20, 0.02)
    ring = np.where(t - t[0] < 5, 0.05, 0.40)
    tp = t[::5]
    pl = pd.DataFrame({'sim_time': tp, 'phase': np.where(tp - t[0] < 5, 'creep', 'planner'),
                       'load_z': ring[::5], 'load_vz': 0.0})
    os.makedirs(os.path.join(root, f'load_planner_{stamp}'))
    pl.to_csv(os.path.join(root, f'load_planner_{stamp}', 'log.csv'), index=False)
    h = math.radians(tilt_deg) / 2
    r33 = math.cos(2 * h)
    for i in range(4):
        v = 24.2 - 0.05 * i - 0.04 * (t - t[0])
        m_sup = (md * g + frac * thrust_fit.RING_KG * g / 4) / (g * r33)
        real = law['a'][i] + law['b'] * m_sup + law['c'] * (v - thrust_fit.V_REF)
        u2 = real - (0.185 + 0.022 * (23.5 - v)) if affine else real
        d = pd.DataFrame({'sim_time': t + 0.003 * i, 'u2': u2, 'battery_v': np.round(v, 1),
                          'pose_qw': math.cos(h), 'pose_qx': math.sin(h), 'pose_qy': 0.0, 'pose_qz': 0.0,
                          'x0_vz': 0.0})
        p = os.path.join(root, f'planner_drone{i}_{stamp[:-2]}0{i}')
        os.makedirs(p)
        d.to_csv(os.path.join(p, 'log.csv'), index=False)
        with open(os.path.join(p, 'params.json'), 'w') as f:
            json.dump({'thrust_offset': 0.185, 'thrust_offset_v_slope': 0.022, 'thrust_v_ref': 23.5}
                      if affine else {'thrust_ratio': 21.7}, f)


@pytest.mark.parametrize('affine', [False, True])
def test_tethered_fraction_on_a_known_launch(tmp_path, affine):
    _launch(str(tmp_path), 0.80, affine)
    (p, trackers), = thrust_fit.launches(str(tmp_path))
    assert sorted(trackers) == [0, 1, 2, 3]
    df, rest = thrust_fit.rod_forces(p, trackers, thrust_fit.LAW_0930)
    s = thrust_fit.summarize(df, rest)
    assert rest == pytest.approx(0.05)
    assert s['secs'] == pytest.approx(15.0, abs=0.2)
    assert s['med'] == pytest.approx(0.80, abs=0.02)
    assert s['dyn'] == pytest.approx(0.80, abs=0.02)


def test_thr_out_column_wins_over_the_map():
    d = pd.DataFrame({'u2': [0.3], 'thr_out': [0.55], 'battery_v': [23.0]})
    assert thrust_fit.real_throttle(d, {'thrust_offset': 0.185})[0] == pytest.approx(0.55)
    d = d.drop(columns='thr_out')
    assert thrust_fit.real_throttle(d, {'thrust_offset': 0.185})[0] == pytest.approx(0.3 + 0.185 + 0.011)
    assert thrust_fit.real_throttle(d, {'thrust_ratio': 21.7})[0] == pytest.approx(0.3)
