"""Drop and land, the trackers' half (card 2026-10-08_drop_and_land): every ring sample over
drop_tilt_deg for drop_samples in a row asks for the drop once; a frame-to-frame attitude step no
ring can make resets the count; the payload checks stand down for DROP_CONFIRM_S, then come back
unless the planner dropped the ring."""
import math
from types import SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from tracker.tracker_node import DROP_CONFIRM_S, Controller as C, _quat_step_deg  # noqa: E402


def q_tilt(deg, axis=(1.0, 0.0, 0.0)):
    h = math.radians(deg) / 2.0
    return (math.cos(h), axis[0] * math.sin(h), axis[1] * math.sin(h), axis[2] * math.sin(h))


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return _T(self.t)


class _T:
    def __init__(self, t):
        self.t = t

    def __sub__(self, o):
        return SimpleNamespace(nanoseconds=int((self.t - o.t) * 1e9))


def _tracker(samples=2):
    sent, log = [], []
    clk = _Clock()
    f = SimpleNamespace(drop_on_loss=True, drop_tilt_deg=30.0, drop_samples=samples,
                        drop_glitch_step_deg=15.0, _drop_requested_t=None, _drop_count=0,
                        _drop_prev_q=None, _drop_fallback_said=False, _payload_dropped=False,
                        armed=True, takeoff_requested=True, drone_id=1, _safety_clock=clk,
                        _drop_pub=SimpleNamespace(publish=lambda m: sent.append(m.data)))
    f.get_logger = lambda: SimpleNamespace(error=log.append, warn=log.append, info=log.append)
    f.sent, f.clk = sent, clk
    return f


def test_quat_step():
    assert abs(_quat_step_deg(q_tilt(0), q_tilt(10)) - 10.0) < 1e-6
    assert _quat_step_deg(q_tilt(10), tuple(-v for v in q_tilt(10))) < 1e-4   # q and -q


def test_two_samples_over_30_ask_once():
    f = _tracker()
    for d in (20, 28, 31):
        C._drop_rule(f, q_tilt(d))
    assert f.sent == []
    C._drop_rule(f, q_tilt(33))
    assert len(f.sent) == 1 and 'ring tilt 33.0' in f.sent[0]
    C._drop_rule(f, q_tilt(40))
    assert len(f.sent) == 1


def test_a_glitch_step_resets_and_is_not_counted():
    f = _tracker()
    C._drop_rule(f, q_tilt(5))
    C._drop_rule(f, q_tilt(35))            # 30 deg in one frame: a pose glitch
    C._drop_rule(f, q_tilt(36))            # 1 deg step, over 30: count 1
    assert f.sent == []
    C._drop_rule(f, q_tilt(37))
    assert len(f.sent) == 1


def test_nothing_before_takeoff():
    f = _tracker()
    f.takeoff_requested = False
    for d in (31, 32, 33, 34):
        C._drop_rule(f, q_tilt(d))
    assert f.sent == []


def test_payload_checks_stand_down_then_come_back_without_a_drop():
    f = _tracker()
    assert not C._payload_checks_down(f)
    C._drop_rule(f, q_tilt(31))
    C._drop_rule(f, q_tilt(32))
    assert C._payload_checks_down(f)
    f.clk.t = DROP_CONFIRM_S + 0.01
    assert not C._payload_checks_down(f)
    f._payload_dropped = True
    assert C._payload_checks_down(f)
