"""The magnet-marker release check (tip_detect_m; Wesley 21 Oct, card 2026-10-21_tip_detect): a carrier's magnet marker
that leaves its plate point by tip_detect_m more than its attached offset, TIP_DETECT_N samples in a row, has let go,
which catches a quad carrying nothing (the T's opposite quad) whose cable never stretches. Noise, a slow drift, a
blip, every marker off at once (the ring's pose jumped) and a stale ring pose are not releases."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import TIP_DETECT_N, DissipativeController as D  # noqa: E402
from mpc_planner.geometry import attach_points  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


def _node(tip_m=0.04):
    rho = attach_points(3, 0.225, 0.02, '0,270,180')                   # the T 0/3/6
    log = _Log()
    f = SimpleNamespace(slot2drone=[0, 1, 2], detached=[False] * 3, rho=rho, _tip_detect_m=tip_m,
                        _tip_base={}, _tip_over={}, _tip_last={}, _detect_refused=set(), phase='planner',
                        _reconfig_mode='ocp', _land_to_ground=False, _drop_on_loss=True, _unload=None,
                        takeoff_seen=True, load_state=np.r_[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, np.zeros(6)],
                        released=[])
    f.get_logger = lambda: log
    f.log = log
    f._load_t = time.monotonic()
    f._load_down = lambda: False
    f._on_unannounced = lambda d: f.released.append(d)
    f._tip_check = lambda d, tip, now: D._tip_check(f, d, tip, now)
    return f


def _plate(f, d):
    return f.load_state[0:3] + np.asarray(f.rho[f.slot2drone.index(d)], float)


def _feed(f, d, offsets, rng=None, noise=0.0):
    """Tip samples at plate + offset (+ noise); returns the drones released so far."""
    for off in offsets:
        f._load_t = time.monotonic()
        n = rng.normal(0.0, noise, 3) if rng is not None else 0.0
        f._tip_check(d, _plate(f, d) + np.asarray(off, float) + n, time.monotonic())
    return f.released


def test_an_attached_marker_with_noise_never_reads_as_a_release():
    f, rng = _node(), np.random.default_rng(0)
    for d in range(3):
        _feed(f, d, [[0.0, 0.0, 0.03]] * 300, rng, noise=0.004)       # 3 cm marker offset, 4 mm noise
    assert f.released == []
    assert f._tip_base[0] == pytest.approx(0.03, abs=0.005)


def test_a_marker_leaving_its_plate_is_a_release_after_n_samples():
    f = _node()
    for d in range(3):
        _feed(f, d, [[0.0, 0.0, 0.03]] * 50)
    _feed(f, 1, [[0.0, -0.08, -0.03]] * (TIP_DETECT_N - 1))            # swinging off plate 3: 5.5 cm past its offset
    assert f.released == []
    _feed(f, 1, [[0.0, -0.08, -0.03]])
    assert f.released == [1]
    assert any('UNANNOUNCED DETACH' in m and 'magnet' in m for m in f.log.lines)


def test_a_blip_a_slow_drift_and_a_ring_pose_jump_are_not_releases():
    f = _node()
    for d in range(3):
        _feed(f, d, [[0.0, 0.0, 0.03]] * 50)
    _feed(f, 0, [[0.08, 0.0, 0.03]] * (TIP_DETECT_N - 1) + [[0.0, 0.0, 0.03]] * 5)   # a blip
    _feed(f, 0, [[0.0, 0.0, 0.03 + 0.0005 * k] for k in range(200)])               # 10 cm drift, 0.5 mm/sample
    assert f.released == []
    f = _node()
    for d in range(3):
        _feed(f, d, [[0.0, 0.0, 0.03]] * 50)
    for _ in range(TIP_DETECT_N + 2):                                 # every marker off at once (the samples
        for d in range(3):                                            # interleave as the stream does): the ring
            _feed(f, d, [[0.1, 0.0, 0.03]])                           # pose jumped, not every magnet
    assert f.released == []


def test_it_is_quiet_when_off_stale_detached_or_landing():
    f = _node(tip_m=0.0)
    assert _feed(f, 1, [[0.0, 0.0, 0.03]] * 5 + [[0.0, -0.2, 0.0]] * 10) == []
    f = _node()
    _feed(f, 1, [[0.0, 0.0, 0.03]] * 5)
    f._load_t = time.monotonic() - 1.0                                # a stale ring pose
    for _ in range(10):
        f._tip_check(1, _plate(f, 1) + [0.0, -0.2, 0.0], time.monotonic())
    assert f.released == []
    f.detached[1] = True
    assert _feed(f, 1, [[0.0, -0.2, 0.0]] * 10) == []
    f = _node()
    _feed(f, 1, [[0.0, 0.0, 0.03]] * 5)
    f._land_to_ground = True                                          # LAND with the ring already down
    f._load_down = lambda: True
    assert _feed(f, 1, [[0.0, -0.2, 0.0]] * 10) == []
