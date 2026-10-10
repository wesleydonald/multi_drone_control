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
    r.now = 12.0
    r._planner_lines.append((12.0, '[dissipative] approach: drone 3 descend'))
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
    q._planner_lines.append((0.4, '[dissipative] approach: drone 3 climb'))   # before the wait fired
    for t in (1.0, 1.1, 1.3):
        q.now = t
        q.fire_due_events()
    assert [d for _, d in q.fired] == ['WAIT_LOG']
    q.now = 1.4                                                    # 1.0 s after the line, as written
    q.fire_due_events()
    assert q.fired[-1] == (1.4, 'LAND')


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


def test_a_mocap_loss_the_run_injected_is_not_a_harness_failure():
    """Critic (fault-scope card) M1: the runner's own watchdog ended a MOCAP_LOSS run 2 s after the loss."""
    pytest.importorskip('interfaces.msg')
    from types import MethodType, SimpleNamespace
    from experiment.runner_node import ExperimentRunner
    def runner(cut):
        r = SimpleNamespace(cfg=SimpleNamespace(pose_timeout_s=2.0, criteria=None), sim_t=50.0, n=4,
                            pose_stamp=[50.0, 50.0, 50.0, 40.0], failures=[], aborts=[], _mocap_cut=set(cut))
        r._rel = lambda: 50.0
        r.check_health = MethodType(ExperimentRunner.check_health, r)
        return r
    assert runner([3]).check_health() is False                     # drone 3 silenced on purpose
    bad = runner([])
    assert bad.check_health() is True and 'drone 3 mocap stale' in bad.failures[0]
    other = runner([0])
    assert other.check_health() is True                            # only the drone that was cut


def test_wait_reweld_holds_until_the_reweld_then_shifts():
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(5.0, 'WAIT_REWELD'), Event(8.0, 'LAND')])
    for t in (5.0, 6.0, 200.0):                                   # no timeout: it waits
        r.now = t
        r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_REWELD']
    r.t_reweld = 200.0
    r.fire_due_events()
    r.now = 203.0                                                 # 8 s + the 195 s the wait took
    r.fire_due_events()
    assert r.fired[-1] == (203.0, 'LAND') and 'REWELD_TIMEOUT' not in [n for _, n, _ in r.event_log]


def test_wait_reweld_timeout_logs_and_the_schedule_goes_on_to_land():
    """Card 2026-10-10_combined_m1_contact: no attach -> the take-back, and the run still lands."""
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(20.0, 'WAIT_REWELD', 120), Event(28.0, 'LAND')])
    for t in (20.0, 60.0, 139.9):
        r.now = t
        r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_REWELD']
    r.now = 140.0
    r.fire_due_events()
    assert ('REWELD_TIMEOUT' in [n for _, n, _ in r.event_log]) and r.t_reweld is None
    r.now = 148.0                                                 # 28 s + the 120 s timeout
    r.fire_due_events()
    assert r.fired[-1] == (148.0, 'LAND')


def test_wait_weld_without_a_timeout_is_unchanged():
    """No arg: blocks until the first weld, then the overdue events fire as before (no shift)."""
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(20.0, 'WAIT_WELD'), Event(28.0, 'LAND')])
    for t in (20.0, 100.0):
        r.now = t
        r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_WELD']
    r.attached = True
    r.fire_due_events()
    assert r.fired[-1] == (100.0, 'LAND')


def test_wait_weld_timeout_times_the_rest_from_the_weld_or_the_timeout():
    """The combined M1 subtest: QUAD4 is never welded before, so WAIT_REWELD would never end on a success."""
    pytest.importorskip('interfaces.msg')
    from experiment.config import Event
    r = _waiting_runner([Event(20.0, 'WAIT_WELD', 120), Event(28.0, 'LAND')])
    r.now = 20.0
    r.fire_due_events()
    r.now, r.attached = 70.0, True
    r.fire_due_events()
    assert [d for _, d in r.fired] == ['WAIT_WELD']
    r.now = 78.0                                                  # 8 s after the weld
    r.fire_due_events()
    assert r.fired[-1] == (78.0, 'LAND') and 'WELD_TIMEOUT' not in [n for _, n, _ in r.event_log]
    r = _waiting_runner([Event(20.0, 'WAIT_WELD', 120), Event(28.0, 'LAND')])
    for t in (20.0, 139.9, 140.0):
        r.now = t
        r.fire_due_events()
    assert 'WELD_TIMEOUT' in [n for _, n, _ in r.event_log]
    r.now = 148.0
    r.fire_due_events()
    assert r.fired[-1] == (148.0, 'LAND')
    with pytest.raises(ValueError):
        Event(1.0, 'WAIT_WELD', '0')


def test_wait_reweld_arg_is_checked_when_the_config_loads():
    from experiment.config import Event, wait_reweld_arg
    assert wait_reweld_arg(None) is None and wait_reweld_arg(120) == 120.0 and wait_reweld_arg('90.5') == 90.5
    for bad in ('0', '-5', '60 9', 'soon'):
        with pytest.raises(ValueError):
            Event(1.0, 'WAIT_REWELD', bad)


def _contact_runner(contact=True):
    from types import MethodType, SimpleNamespace
    from experiment.runner_node import ExperimentRunner
    r = SimpleNamespace(n=4, event_log=[], _contact=False, _mux_state=None, _proof=None, _contact_run=contact,
                        PROOF_COLS=ExperimentRunner.PROOF_COLS)
    r._log_event = lambda name, arg=None: r.event_log.append((name, arg))
    for name in ('_contact_cb', '_give_up_cb', '_take_back_cb', '_mux_state_cb', '_proof_cb'):
        setattr(r, name, MethodType(getattr(ExperimentRunner, name), r))
    return r


def test_contact_signals_are_logged_as_events():
    pytest.importorskip('interfaces.msg')
    from std_msgs.msg import Bool, String
    r = _contact_runner()
    r._contact_cb(Bool(data=False))                               # the latched startup False: no event
    r._mux_state_cb(String(data='partner'))
    r._contact_cb(Bool(data=True))
    r._contact_cb(Bool(data=True))
    r._contact_cb(Bool(data=False))
    r._give_up_cb(Bool(data=True))
    r._take_back_cb(Bool(data=True))
    r._mux_state_cb(String(data='partner'))
    r._mux_state_cb(String(data='ours'))
    assert [n for n, _ in r.event_log] == ['MUX:partner', 'CONTACT', 'CONTACT_LOST', 'GIVE_UP', 'TAKE_BACK',
                                           'MUX:ours']


def _row_runner(contact):
    from types import MethodType, SimpleNamespace
    from experiment.runner_node import ExperimentRunner
    r = _contact_runner(contact)
    r.__dict__.update(payload=(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0), payload_ref=None, _last_log_t=None, wall0=0.0,
                      step_i=0, rows=[], _log_tip=False, pose=[None] * 4, vel=[None] * 4, cmd=[None] * 4,
                      ref=[None] * 4, attached=False, cfg=SimpleNamespace(ball_start=None), ball=None)
    r._rel = lambda: 1.0
    r.log_row = MethodType(ExperimentRunner.log_row, r)
    return r


def test_attach_proof_fields_ride_along_in_run_csv_columns():
    pytest.importorskip('interfaces.msg')
    from std_msgs.msg import String
    r = _row_runner(True)
    r.log_row()
    assert {k: r.rows[-1][k] for k in r.rows[-1] if k.startswith('proof_')} == {
        'proof_state': None, 'proof_drift_m': None, 'proof_plate_rise_m': None, 'proof_body_to_tip_m': None}
    r._proof_cb(String(data='not json'))
    r._proof_cb(String(data='{"state": "proving", "drift_m": 0.0012, "plate_rise_m": 0.004, '
                            '"body_to_tip_m": 0.593, "tries": 1}'))
    r._last_log_t = None
    r.log_row()
    assert {k: r.rows[-1][k] for k in r.rows[-1] if k.startswith('proof_')} == {
        'proof_state': 'proving', 'proof_drift_m': 0.0012, 'proof_plate_rise_m': 0.004,
        'proof_body_to_tip_m': 0.593}
    q = _row_runner(False)                                        # every other config: no new columns
    q.log_row()
    assert not [k for k in q.rows[-1] if k.startswith('proof_')]


def test_wait_log_can_wait_for_his_phase():
    """LAND during his transit (the take-back arm): his phase changes join the lines WAIT_LOG searches."""
    pytest.importorskip('interfaces.msg')
    from types import MethodType, SimpleNamespace
    from experiment.config import Event
    from experiment.runner_node import ExperimentRunner
    r = _waiting_runner([Event(20.0, 'WAIT_LOG', '400 PHASE:TRANSIT_TO_DROP_POINT'), Event(25.0, 'LAND')])
    r._phase = None
    r._phase_cb = MethodType(ExperimentRunner._phase_cb, r)
    r.now = 20.0
    r.fire_due_events()
    r.now = 90.0
    r._phase_cb(SimpleNamespace(data='TRANSIT_TO_DROP_POINT'))
    r.fire_due_events()
    r.now = 95.0                                                  # 5 s after his transit started
    r.fire_due_events()
    assert r.fired[-1] == (95.0, 'LAND')
