"""The T6 release metrics (docs/rig/freeze_2026-10-14.md 1.1) as tools/release_bands.py computes them and
tools/release_transient.py times them, on a synthetic ring whose tilt vector is known in closed form."""
import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import release_bands as rb  # noqa: E402
import release_transient as rt  # noqa: E402

TE, TLAND, HZ = 10.0, 22.0, 50.0


def _quat(tx, ty):
    """Ring quaternion (w, x, y, z) whose body z leans to world (tx, ty) deg: rotation about (-ty, tx)."""
    ang = np.radians(np.hypot(tx, ty))
    if ang < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    ax = np.array([-ty, tx]) / np.hypot(tx, ty)
    return np.array([np.cos(ang / 2), ax[0] * np.sin(ang / 2), ax[1] * np.sin(ang / 2), 0.0])


def _theta(t):
    """Tilt vector (deg): 2 deg toward +x before the event; after it a 6 deg kick toward +y that decays in
    about 1 s, settling at 3 deg toward +x."""
    th = np.where(t < TE, 2.0, 3.0)[:, None] * np.array([1.0, 0.0])
    kick = np.where(t >= TE, 6.0 * np.exp(-((t - TE - 0.5) / 0.3) ** 2), 0.0)
    th[:, 1] += kick
    th[:, 0] -= np.where(t >= TE, np.exp(-(t - TE) / 0.3), 0.0)      # from 2 to 3 deg without a jump
    return th


def _run_dir(tmp_path, events):
    t = np.arange(0.0, 30.0, 1.0 / HZ)
    th = _theta(t)
    q = np.array([_quat(*v) for v in th])
    d = tmp_path / 'run'
    (d / 'logs').mkdir(parents=True)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    pd.DataFrame({'t': t, 'payload_x': 0.0, 'payload_y': 0.0, 'payload_z': 1.0, 'payload_qw': q[:, 0],
                  'payload_qx': q[:, 1], 'payload_qy': q[:, 2], 'payload_qz': q[:, 3], 'payload_tilt_deg': tilt,
                  'payload_ref_x': 0.0, 'payload_ref_y': 0.0, 'payload_ref_z': 1.0}).to_csv(d / 'logs' / 'run.csv',
                                                                                         index=False)
    pd.DataFrame(events, columns=['sim_time', 'event', 'arg']).to_csv(d / 'logs' / 'events.csv', index=False)
    return str(d), t, th


def test_tilt_vector_has_the_scalar_tilt_and_its_direction():
    th = rb.tilt_vector(np.array([_quat(3.0, -4.0), _quat(0.0, 0.0)]))
    assert np.allclose(th, [[3.0, -4.0], [0.0, 0.0]], atol=1e-9)


def test_dtheta_and_time_back_in_band_follow_the_frozen_rule(tmp_path):
    d, t, th = _run_dir(tmp_path, [(TE, 'RELEASE', 'drone 4'), (TLAND, 'LAND', '')])
    m = rb.Run(d).metrics()
    pre = th[(t >= TE - 5) & (t < TE)].mean(0)
    w = (t >= TE) & (t <= TLAND)
    dev = np.linalg.norm(th[w] - pre, axis=1)
    assert m['kind'] == 'unannounced' and m['steady']
    assert m['dtheta'] == pytest.approx(dev.max(), abs=1e-6)
    assert m['abs_peak'] == pytest.approx(np.linalg.norm(th[w], axis=1).max(), abs=1e-6)
    assert m['sigma_pre'] == pytest.approx(0.0, abs=1e-6)
    # band max(2 sigma_pre, 0.5) = 0.5 deg about the last 5 s before LAND (3 deg toward +x): back once the kick
    # and the 2 -> 3 deg drift are both within 0.5 deg
    inb = np.linalg.norm(th - [3.0, 0.0], axis=1) <= 0.5
    first = next(ti for ti in t[(t >= TE + m['t_peak']) & (t + 1 <= TLAND)]
                 if inb[(t >= ti) & (t < ti + 1)].all())
    assert m['t_back'] == pytest.approx(first - TE, abs=1e-9)


def test_a_flight_the_freeze_does_not_time_is_labelled(tmp_path):
    # an uncommanded release (a SLIP, no magnet OFF) in a flight that ends in a fleet abort with no LAND after it
    d, t, th = _run_dir(tmp_path, [(2.0, 'LAND', ''), (TE, 'SLIP', 'drone 2 (/drone_1) az -86.9'),
                                   (TE + 3.0, 'FLEET_ABORT', '')])
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    te, tend, rule, end = rt.rig_event(dict(kind='uncommanded', slip=1), ev)
    assert (te, tend) == (TE, TE + 3.0) and 'not a frozen' in rule and end.startswith('FLEET_ABORT')
    run = rb.Run(d, te=te, kind='uncommanded', tland=tend)
    m = run.metrics()
    assert m['window_s'] == pytest.approx(3.0) and m['post'] is None         # too short to score t_back
    assert rt.t_back_2s(run, m['sigma_pre'], m['t_peak']) is None            # nor the 2 s variant (< 4 s)
    with pytest.raises(SystemExit):                                          # the frozen path still refuses it
        rb.Run(d)
