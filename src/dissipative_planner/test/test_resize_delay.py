"""resize_delay_s (twin only; card 2026-10-30_x1_release_ablation, critic waived by Wesley 30 Oct): an unannounced loss is
acted on (resize, or the drop rule) resize_delay_s of node-clock time after its detection, with the leaver kept in the
model and not re-detected meanwhile; 0 acts at detection as before; a delay longer than the flight never resizes (X1's
no-resize arm). Negative values and wall-clock nodes are refused."""
import time

import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D, resize_delay_value  # noqa: E402
from test_drop_and_land import _node  # noqa: E402


def _armed(delay):
    f = _node()
    f.cable_len, f.cable_len_i, f._detect_m = 0.55, [0.55] * 4, 0.06
    f._reconfig_mode, f._detect_count, f._detect_ok, f._detect_refused = 'ocp', {}, {}, set()
    f._load_t, f._drone_t = time.monotonic(), {k: time.monotonic() for k in range(4)}
    f._load_down = lambda: False
    f._resize_delay_s, f._resize_due = delay, {}
    f.calls = []

    def detach(d):
        f.calls.append((d, f.now[0]))
        f.detached[d] = True
    f._detach_ocp = detach
    for m in ('_resize_due_tick', '_act_on_loss'):
        setattr(f, m, (lambda name: lambda *a, **k: getattr(D, name)(f, *a, **k))(m))
    f.dists = [0.55] * 4
    f._rim_dist = lambda i: f.dists[i]
    return f


def _release(f, i=1):
    """drone i's cable stretches over two ticks (detected on the second), as a release does."""
    D._detect_tick(f)
    f.dists[i] = 0.62
    D._detect_tick(f)
    f.dists[i] = 0.70
    D._detect_tick(f)


def _tick(f, dt=0.125):          # exact in binary, so the due tick is the 8th of a second
    f.now[0] += dt
    D._resize_due_tick(f)
    D._detect_tick(f)


def test_no_delay_resizes_at_detection():
    f = _armed(0.0)
    _release(f)
    assert f.calls == [(1, 100.0)] and f._resize_due == {}


def test_a_delay_resizes_once_that_long_after_detection_and_the_leaver_is_not_redetected():
    f = _armed(1.0)
    _release(f)                                   # drone 2 (at 90): survivors 30/150/270, no drop due
    assert f.calls == [] and f._resize_due == {1: pytest.approx(101.0)}
    for _ in range(7):
        _tick(f)                                  # its cable stays stretched: no second detection, no resize yet
    assert f.calls == [] and 'LATE RESIZE' not in ' '.join(f.log.lines)
    _tick(f)
    assert f.calls == [(1, pytest.approx(101.0))] and f._resize_due == {}
    assert any('LATE RESIZE' in m for m in f.log.lines)
    for _ in range(20):
        _tick(f)
    assert len(f.calls) == 1


def test_a_delay_longer_than_the_flight_never_resizes():
    f = _armed(999.0)
    _release(f)
    for _ in range(480):                          # 60 s of ticks
        _tick(f)
    assert f.calls == [] and f._dropped is None and not f.detached[1] and 1 in f._resize_due


def test_the_drop_rule_is_applied_late_too():
    f = _armed(0.5)
    _release(f, i=3)                              # drone 4 (at 270): survivors 30/90/150, a drop is due
    assert f._dropped is None
    for _ in range(4):
        _tick(f)
    assert f.calls == [] and f._dropped.startswith('unannounced loss of drone 4')


def test_the_value_is_twin_only_and_not_negative():
    assert resize_delay_value(0.0, False) == 0.0 and resize_delay_value('0', True) == 0.0
    assert resize_delay_value(2.0, True) == 2.0
    with pytest.raises(ValueError, match='twin only'):
        resize_delay_value(1.0, False)
    with pytest.raises(ValueError, match='>= 0'):
        resize_delay_value(-1.0, True)
