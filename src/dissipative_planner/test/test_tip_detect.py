"""The magnet-marker release check, rev 2 after the critic (tip_detect_m; Wesley 21 Oct, card 2026-10-21_tip_detect):
each carrier's marker in the ring frame from its plate point. The attached offset is learnt once, over a still second
after the lift, and bounded; then TIP_DETECT_S of contiguous samples past it by tip_detect_m are a release. Samples
off the learnt body-to-marker length, non-finite samples, blips, gaps, a ring-pose jump, LAND and an unlearnt drone
are never releases."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (TIP_DETECT_S, TIP_LEARN_S,  # noqa: E402
                                                  DissipativeController as D)
from mpc_planner.geometry import attach_points  # noqa: E402

DT = 0.01                     # 100 Hz, as the rig's markers
OFF = np.array([0.0, 0.0, 0.03])   # the marker 3 cm above the plate point, as body 24 on the rig
BTT = 0.59                    # cable 0.55 from a pivot 4 cm below the body


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


def _node(tip_m=0.04):
    rho = attach_points(3, 0.225, 0.02, '0,270,180')                 # the T 0/3/6
    log = _Log()
    f = SimpleNamespace(slot2drone=[0, 1, 2], detached=[False] * 3, rho=rho, _tip_detect_m=tip_m, _tip_v0={},
                        _tip_learn={}, _tip_run={}, _tip_last={}, _tip_unarmed=set(), _detect_refused=set(),
                        phase='planner', _reconfig_mode='ocp', _land_to_ground=False, _unload=None,
                        takeoff_seen=True, load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
                        cable_len=0.55, cable_len_i=[0.55] * 3, pivot_offset=np.array([0.0, 0.0, -0.04]),
                        lift_z0=0.02, target_z=1.0, lift_progress=0.98, released=[], t=100.0)
    f.get_logger = lambda: log
    f.log = log
    f._on_unannounced = lambda d: f.released.append(d)
    f._tip_learn_step = lambda d, sl, v, b, now: D._tip_learn_step(f, d, sl, v, b, now)
    f._tip_check = lambda d, tip, now: D._tip_check(f, d, tip, now)
    f.drone_pos = [_tip(f, d) + BTT * _out(f, d) for d in range(3)]
    f._drone_t = {}
    return f


def _plate(f, d):
    return f.load_state[0:3] + np.asarray(f.rho[d], float)


def _out(f, d):
    """Unit vector from d's plate up and out to its drone, 60 deg up."""
    a = np.arctan2(f.rho[d][1], f.rho[d][0])
    return np.array([np.cos(a) * 0.5, np.sin(a) * 0.5, np.sqrt(3) / 2])


def _tip(f, d, extra=(0.0, 0.0, 0.0)):
    return _plate(f, d) + OFF + np.asarray(extra, float)


def _step(f, samples):
    """One 100 Hz frame: {drone: tip}; the ring and drone poses are fresh."""
    f.t += DT
    f._load_t = f.t
    for d, tip in samples.items():
        f._drone_t[d] = f.t
        f._tip_check(d, np.asarray(tip, float), f.t)


def _arm(f):
    for _ in range(int(TIP_LEARN_S / DT) + 3):
        _step(f, {d: _tip(f, d) for d in range(3)})
    assert set(f._tip_v0) == {0, 1, 2}


def test_it_arms_after_a_still_second_with_the_markers_offset():
    f = _node()
    _arm(f)
    assert np.allclose(f._tip_v0[1][0], OFF, atol=1e-9) and f._tip_v0[1][1] == pytest.approx(BTT)
    assert sum('armed' in m for m in f.log.lines) == 3


def test_it_does_not_learn_in_the_lift_or_while_the_ring_moves():
    f = _node()
    f.lift_progress = 0.5
    for _ in range(200):
        _step(f, {d: _tip(f, d) for d in range(3)})
    f.lift_progress, f.load_state[7] = 0.98, 0.2                    # lifted, but the ring moving at 0.2 m/s
    for _ in range(200):
        _step(f, {d: _tip(f, d) for d in range(3)})
    assert f._tip_v0 == {} and f.released == []


def test_a_wrong_offset_or_a_wrong_length_is_not_armed():
    f = _node()
    for _ in range(int(TIP_LEARN_S / DT) + 3):                       # drone 1's marker reads 10 cm off its plate
        _step(f, {0: _tip(f, 0), 1: _tip(f, 1, (0.1, 0.0, 0.0)), 2: _tip(f, 2)})
    assert 1 in f._tip_unarmed and 1 not in f._tip_v0 and any('NOT armed' in m for m in f.log.lines)
    for _ in range(20):
        _step(f, {1: _tip(f, 1, (0.0, -0.3, -0.1))})
    assert f.released == []


def test_a_magnet_swinging_off_its_plate_is_a_release_after_tip_detect_s():
    f = _node()
    _arm(f)
    hang = f.drone_pos[1] - np.array([0.0, 0.0, BTT])                # hanging below its quad: off the plate, in band
    n = int(np.ceil(TIP_DETECT_S / DT))
    for _ in range(n):
        _step(f, {0: _tip(f, 0), 1: hang, 2: _tip(f, 2)})
    assert f.released == []
    _step(f, {0: _tip(f, 0), 1: hang, 2: _tip(f, 2)})
    assert f.released == [1]
    assert any('UNANNOUNCED DETACH' in m and 'magnet' in m for m in f.log.lines)


def test_noise_a_blip_a_gap_and_bad_samples_are_not_releases():
    f, rng = _node(), np.random.default_rng(0)
    _arm(f)
    hang = f.drone_pos[1] - np.array([0.0, 0.0, BTT])
    for _ in range(300):                                             # 3 mm noise
        _step(f, {d: _tip(f, d) + rng.normal(0.0, 0.003, 3) for d in range(3)})
    _step(f, {1: hang})                                              # a 2-frame blip
    _step(f, {1: hang})
    _step(f, {1: _tip(f, 1)})
    _step(f, {1: hang})                                              # past, then a 0.1 s gap, then past again
    f.t += 0.1
    _step(f, {1: hang})
    _step(f, {1: [np.nan, 0.0, 0.0]})                                # non-finite
    for _ in range(10):
        _step(f, {1: [0.0, 0.0, 0.0]})                               # an untracked body: off the body band
    assert f.released == []


def test_a_ring_pose_jump_moves_every_marker_and_is_not_a_release():
    f = _node()
    _arm(f)
    tips = {d: _tip(f, d) for d in range(3)}                         # the magnets stay where they are...
    f.load_state[0] += 0.1                                           # ...while the ring's mocap pose jumps 10 cm
    for _ in range(10):
        _step(f, tips)
    assert f.released == []


def test_it_is_quiet_when_off_in_land_or_detached():
    f = _node(tip_m=0.0)
    for _ in range(300):
        _step(f, {1: _tip(f, 1, (0.0, -0.3, 0.0))})
    assert f.released == [] and f._tip_v0 == {}
    f = _node()
    _arm(f)
    hang = f.drone_pos[1] - np.array([0.0, 0.0, BTT])
    f._land_to_ground = True                                         # LAND, ring still up: the stretch check only
    for _ in range(20):
        _step(f, {1: hang})
    f._land_to_ground, f.detached[1] = False, True
    for _ in range(20):
        _step(f, {1: hang})
    assert f.released == []


def test_the_offline_replay_runs_the_same_check_over_recorded_samples():
    """tools/tip_replay.py (Wesley 21 Oct: Part C flies the check only if the A/B flights replay clean)."""
    import importlib.util
    import pathlib
    spec = importlib.util.spec_from_file_location(
        'tip_replay', pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'tip_replay.py')
    tr = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tr)
    g = _node()                                  # the T geometry, typed clockwise as the sheet does: 0,90,180
    q, v = np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3)
    ev, t = [], 0.0
    for k in range(400):                         # rest, lift to 1.0, hold
        t += DT
        z = min(1.0, 0.02 + 0.01 * k)
        ev.append((t, 'ring', -1, np.array([0.0, 0.0, z]), (q, v)))
        for d in range(3):
            tip = np.array([0.0, 0.0, z - 1.0]) + _tip(g, d)
            ev.append((t, 'drone', d, tip + BTT * _out(g, d), None))
            ev.append((t, 'tip', d, tip, None))
    f, fires, devmax = tr.replay(ev, [0.0, 90.0, 180.0], 0.02)
    assert set(f._tip_v0) == {0, 1, 2} and fires == [] and max(devmax.values()) < 0.01
    body2 = ev[-2][3]                            # drone 2 holds its place...
    hang = body2 - np.array([0.0, 0.0, BTT])     # ...while its magnet swings off the plate to hang below it
    for k in range(10):
        t += DT
        ev.append((t, 'ring', -1, np.array([0.0, 0.0, 1.0]), (q, v)))
        ev.append((t, 'drone', 2, body2, None))
        ev.append((t, 'tip', 2, hang, None))
    f, fires, devmax = tr.replay(ev, [0.0, 90.0, 180.0], 0.02)
    assert [d for _, d, _ in fires] == [2]
