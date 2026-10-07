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
