"""The runner's event queue is a copy: the lift wait's shift of one run must not carry into the
next --repeats run (reviewer 8 Oct: every second repeat started ~10 s late)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_a_shift_in_one_run_leaves_the_config_alone():
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    from experiment.runner_node import schedule
    events = [Event(20.0, 'RELEASE', 1), Event(3.0, 'ARM')]
    run1 = schedule(events)
    assert [e.do for e in run1] == ['ARM', 'RELEASE']
    for e in run1:
        e.t += 10.0                      # what WAIT_LIFT does to the queued events
    run2 = schedule(events)
    assert [e.t for e in run2] == [3.0, 20.0] and [e.t for e in events] == [20.0, 3.0]


def _waiting_runner(events):
    """fire_due_events on a stand-in runner: a clock, a /rosout line list and a record of what fired."""
    from types import MethodType, SimpleNamespace
    from experiment.runner_node import ExperimentRunner, schedule
    log = SimpleNamespace(warn=lambda *a, **k: None, info=lambda *a, **k: None)
    r = SimpleNamespace(now=0.0, fired=[], event_log=[], _pending=schedule(events), _planner_lines=[],
                        _waiting_for_weld=False, _reweld_wait=None, _partner_release_wait=None,
                        _log_wait=None, _thrust_wait=None, _lift_wait=None, attached=False, payload=None,
                        t_partner_release=None, t_reweld=None)
    r._rel = lambda: r.now
    r.get_logger = lambda: log
    r._fire = lambda ev: r.fired.append((round(r.now, 2), ev.do))
    r._log_event = lambda name, arg=None: r.event_log.append((round(r.now, 2), name, arg))
    r.fire_due_events = MethodType(ExperimentRunner.fire_due_events, r)
    return r


def test_wait_log_holds_the_schedule_until_the_planner_line_then_shifts_it():
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(5.0, 'WAIT_LOG', '60 approach: drone 3 descend'), Event(8.0, 'LAND')])
    for t in (5.0, 6.0, 9.0):
        r.now = t
        r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_LOG']                 # LAND held back
    r._planner_lines.append('[dissipative] approach: drone 3 descend')
    r.now = 12.0
    r.fire_due_events()
    assert r.event_log[-1][1] == 'LOGGED'
    r.now = 14.9
    r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_LOG']
    r.now = 15.0                                                   # 8 s + the 7 s the wait took
    r.fire_due_events()
    assert r.fired[-1] == (15.0, 'LAND')


def test_wait_log_times_out_and_a_line_seen_earlier_counts():
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(1.0, 'WAIT_LOG', '2 never logged'), Event(2.0, 'LAND')])
    for t in (1.0, 2.5, 3.0, 4.0):
        r.now = t
        r.fire_due_events()
    assert ('LOG_TIMEOUT' in [n for _, n, _ in r.event_log]) and r.fired[-1] == (4.0, 'LAND')
    q = _waiting_runner([Event(1.0, 'WAIT_LOG', 'climb'), Event(2.0, 'LAND')])
    q._planner_lines.append('[dissipative] approach: drone 3 climb')
    q.now = 1.0
    q.fire_due_events()
    q.now = 1.0
    q.fire_due_events()
    q.now = 2.0
    q.fire_due_events()
    assert q.fired[-1] == (2.0, 'LAND')


def test_wait_log_arg_is_checked_when_the_config_loads():
    from experiment.config import Event, wait_log_arg
    pat, timeout = wait_log_arg('60 approach: drone 3 (descend|transit)')
    assert timeout == 60.0 and pat.search('approach: drone 3 transit')
    assert wait_log_arg('descend')[1] is None
    for bad in ('', '0 descend', '-1 x', '5'):
        with pytest.raises(ValueError):
            Event(1.0, 'WAIT_LOG', bad)


def test_mocap_loss_arg():
    from experiment.config import Event, mocap_loss_arg
    assert mocap_loss_arg('3') == (3, 600.0) and mocap_loss_arg('3 2.5') == (3, 2.5)
    for bad in (None, '', '3 0', '3 1 2'):
        with pytest.raises(ValueError):
            Event(1.0, 'MOCAP_LOSS', bad)
