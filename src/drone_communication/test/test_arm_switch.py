import pytest

from drone_communication.arm_switch import ArmSwitch, fc_mode_armed, MAX_TRIES


def run(sw, t0, t1, want=True, mode='', mode_fresh=True, dt=0.01):
    """Tick from t0 to t1; returns the list of (t, high, state)."""
    out, t = [], t0
    while t < t1 - 1e-9:
        out.append((t,) + sw.update(want, t, mode, t if mode_fresh else None))
        t += dt
    return out


def test_mode_parse():
    assert fc_mode_armed('ACRO') is True
    assert fc_mode_armed('ACRO*') is False
    assert fc_mode_armed('!ERR*') is False
    assert fc_mode_armed('') is None
    assert fc_mode_armed('UNKNOWN') is None


def test_low_before_the_arm_edge_then_confirmed():
    sw = ArmSwitch()
    trace = run(sw, 0.0, 0.29, mode='ACRO*')
    assert all(not high for _, high, _ in trace)          # held low first
    high, state = sw.update(True, 0.31, 'ACRO*', 0.31)
    assert high and state == 'pending'
    high, state = sw.update(True, 0.5, 'ACRO', 0.5)
    assert high and state == 'armed'


def test_refused_arm_retries_with_a_fresh_edge_then_fails():
    sw = ArmSwitch()
    trace = run(sw, 0.0, 12.0, mode='!ERR*')
    highs = [h for _, h, _ in trace]
    rising = sum(1 for a, b in zip(highs, highs[1:]) if not a and b)
    assert rising == MAX_TRIES
    assert trace[-1][2].startswith(f"failed: FC says '!ERR*' after {MAX_TRIES} tries")


def test_second_try_succeeds():
    sw = ArmSwitch()
    run(sw, 0.0, 2.0, mode='ACRO*')                      # first edge refused
    trace = run(sw, 2.0, 3.0, mode='ACRO*')
    assert any(not h for _, h, _ in trace)               # dropped for the retry
    high, state = sw.update(True, 3.5, 'ACRO', 3.5)
    assert high and state == 'armed'


def test_no_telemetry_never_drops_the_switch():
    sw = ArmSwitch()
    trace = run(sw, 0.0, 5.0, mode='', mode_fresh=False)
    after_edge = [h for t, h, _ in trace if t >= 0.31]
    assert all(after_edge)
    assert trace[-1][2].startswith('unconfirmed')


def test_never_drops_after_confirmation():
    sw = ArmSwitch()
    run(sw, 0.0, 1.0, mode='ACRO')
    trace = run(sw, 1.0, 10.0, mode='ACRO*')             # telemetry says disarmed later
    assert all(h for _, h, _ in trace)
    assert trace[-1][2].startswith('failed: FC reports disarmed')


def test_disarm_command_drops_immediately():
    sw = ArmSwitch()
    run(sw, 0.0, 1.0, mode='ACRO')
    assert sw.update(False, 1.01, 'ACRO', 1.01) == (False, 'disarmed')


def test_sparse_telemetry_is_confirmed_by_a_late_frame():
    # QUAD1 (7 Oct): one flight-mode frame every 2-3 s; the first after the edge comes 2.2 s on
    sw = ArmSwitch()
    edge = 0.3
    trace = run(sw, 0.0, edge + 2.0, mode='AIR*', mode_fresh=False)   # only the pre-edge frame
    assert all(h for t, h, _ in trace if t >= edge + 0.01) and trace[-1][2] == 'pending'
    high, state = sw.update(True, edge + 2.2, 'AIR', edge + 2.2)
    assert high and state == 'armed'


def test_a_stale_pre_edge_frame_is_not_a_refusal():
    sw = ArmSwitch()
    sw.update(True, 0.0, 'AIR*', 0.0)
    for k in range(1, 300):                      # the same pre-edge frame keeps being passed in
        high, state = sw.update(True, 0.01 * k, 'AIR*', 0.0)
    assert high and state == 'pending'


def test_sparse_refusal_retries_then_fails_within_the_manager_window():
    from drone_communication.arm_switch import CONFIRM_MAX_S, PRE_LOW_S, RETRY_LOW_S
    sw = ArmSwitch()
    t, last_frame, states = 0.0, None, []
    while t < 10.0:
        if last_frame is None or t - last_frame >= 2.5:   # '!ERR*' every 2.5 s
            last_frame = t
        states.append(sw.update(True, t, '!ERR*', last_frame)[1])
        t += 0.01
    assert states[-1].startswith('failed') and MAX_TRIES * (PRE_LOW_S + CONFIRM_MAX_S) + RETRY_LOW_S < 10.0
