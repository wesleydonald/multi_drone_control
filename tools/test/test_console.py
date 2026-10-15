"""tools/console.py: the READY rules, the button rules and SPACE, without ROS (the GUI offscreen)."""
import json
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import console as con  # noqa: E402

NOW = 1000.0
ALL_NODES = ['/fleet_manager', '/dissipative_planner', '/fleet_viz', '/motion_capture_publisher',
             '/tejen/tejen_mpc', '/online_join_planner'] + [f'/tracker_{i}' for i in range(4)] + \
            [f'/drone_{i}/elrs_interface' for i in range(4)]


def health(mode='AIR*', volt=24.6, tilt=12.0, connected=True, mode_age=0.4):
    return json.dumps({'port': '/dev/QUAD1', 'connected': connected, 'reconnects': 0, 'cmd_rate_hz': 50, 'lq': 100,
                       'voltage': volt, 'voltage_age_s': None if volt is None else 0.5, 'mode': mode,
                       'mode_age_s': mode_age, 'fc_tilt_deg': tilt, 'attitude_age_s': 0.3})


def summary(z=(0.10, 0.10, 0.10, 0.05), tilt=(12, 12, 12, 2), age=0.01):
    b = {f'drone_{i}': {'z': z[i], 'tilt_deg': tilt[i], 'age_s': age, 'rate_hz': 100} for i in range(4)}
    b['payload'] = {'z': 0.02, 'tilt_deg': 0.5, 'age_s': age, 'rate_hz': 100}
    return json.dumps({'t': NOW, 'bodies': b})


def ready_state(now=NOW, **kw):
    s = con.ConsoleState(4, partner=3, **kw)
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1)
    s.on_summary(summary(), now)
    for i in range(4):
        s.on_health(i, health(), now)
        s.on_tracker_armed(i, False)
    return s


def test_fc_mode_strings():
    assert con.fc_mode_state('AIR*') == 'ready' and con.fc_mode_state('AIR') == 'armed'
    assert con.fc_mode_state('!ERR*') == 'blocked' and con.fc_mode_state('!FS!') == 'blocked'
    assert con.fc_mode_state('') is None and con.fc_mode_state('UNKNOWN') is None


def test_everything_fresh_is_ready():
    assert con.readiness(ready_state(), NOW) == []


def test_not_ready_reasons_say_what_to_do():
    s = ready_state()
    s.on_health(0, health(mode='!ERR*'), NOW - 25.0)        # blocked since 25 s ago
    s.on_health(0, health(mode='!ERR*'), NOW)
    s.on_health(1, health(volt=None), NOW)
    s.on_health(2, health(tilt=62.0), NOW)                  # QUAD1 on 14 Oct: 60-84 deg while at 12
    s.on_mux(3, 'latched')
    s.on_graph([n for n in ALL_NODES if n != '/tracker_2'], set(con.HIS_NODES), 2, 1)
    why = ' | '.join(con.readiness(s, NOW))
    assert "QUAD1: FC blocks arming ('!ERR*' for 25 s): replug its pack" in why
    assert 'QUAD2: no pack voltage' in why
    assert 'QUAD3: FC reads 62 deg of tilt > small_angle 25' in why
    assert 'QUAD4: mux latched' in why and 'not running: QUAD3 tracker' in why
    assert con.fc_check(ready_state(small_angle=180.0), 2, NOW)[0]


def test_short_block_after_a_boot_does_not_say_replug():
    s = ready_state()
    s.on_health(0, health(mode='!ERR*'), NOW - 3.0)
    s.on_health(0, health(mode='!ERR*'), NOW)
    ok, text = con.fc_check(s, 0, NOW)
    assert not ok and 'replug' not in text
    s.on_health(0, health(mode='AIR*'), NOW + 1.0)
    assert con.fc_check(s, 0, NOW + 1.0)[0] and s.err_since[0] is None


def test_stale_mocap_and_a_drone_on_its_side():
    s = ready_state()
    s.on_summary(summary(tilt=(12, 70, 12, 2)), NOW)
    assert any('QUAD2: on its side (70 deg' in w for w in con.readiness(s, NOW))
    assert 'no /fleet/mocap_summary (fleet_viz)' in con.readiness(s, NOW + 2.0)
    s.on_summary(summary(age=0.6), NOW)
    assert 'QUAD1: no fresh mocap' in con.readiness(s, NOW)


def test_twin_skips_the_radio_checks():
    s = con.ConsoleState(4, partner=3, twin=True)
    s.on_graph([n for n in ALL_NODES if 'elrs' not in n and 'motion_capture' not in n], set(con.HIS_NODES), 2, 1)
    s.on_summary(summary(), NOW)
    assert con.readiness(s, NOW) == []


def test_buttons_follow_the_flight_order():
    s = ready_state()
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['his_arm'] == (True, '') and b['arm'] == (False, 'his ARM first')
    assert b['takeoff'][0] is False and b['land'] == (False, 'nothing armed') and b['disarm'] == (True, '')
    s.on_health(3, health(mode='AIR'), NOW)                 # his ARM: QUAD4's FC armed
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['arm'] == (True, '') and b['his_arm'] == (False, 'QUAD4 armed')
    assert b['his_takeoff'] == (False, 'after the fleet ARM') and b['his_disarm'][0] is False
    for i in range(4):
        s.on_tracker_armed(i, True)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['takeoff'] == (True, '') and b['his_takeoff'] == (True, '') and b['arm'] == (False, 'armed')
    assert b['give_up'] == (True, '') and b['land'] == (True, '')
    s.on_graph(ALL_NODES, {con.HIS_NODES[0]}, 1, 1)         # his planner not subscribed yet
    assert con.button_states(s, NOW, [])['his_takeoff'] == (False, 'waiting for online_join_planner')
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 1, 1)         # our publisher has not matched both
    assert con.button_states(s, NOW, [])['his_takeoff'] == (False, 'matched 1 of 2 subscribers')


def test_a_disarm_closes_takeoff_and_opens_his_disarm():
    s = ready_state()
    for i in range(4):
        s.on_tracker_armed(i, True)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    s.on_manager('EMERGENCY STOP broadcast to 4 drones: DISARM pressed')
    assert s.arm_complete is False
    for i in range(4):
        s.on_tracker_armed(i, False)
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['takeoff'][0] is False and b['his_disarm'] == (True, '')


def test_airborne_is_height_above_the_lowest_seen():
    s = ready_state()
    s.on_summary(summary(z=(0.10, 0.10, 0.10, 0.30)), NOW + 1.0)
    assert [(i, round(z, 2)) for i, z in con.airborne(s, NOW + 1.0)] == [(3, 0.25)]


def test_proof_line_reads_the_manager_record():
    s = ready_state()
    s.on_contact(True)
    s.on_proof(json.dumps({'state': 'prove', 'tries': 1, 'xy_m': 0.0041, 'normal_m': -0.0013, 'drift_m': 0.0019,
                           'body_to_tip_m': 0.5907, 'l_hang_m': 0.5906, 'fails': 0}))
    line = con.proof_line(s)
    assert 'contact yes' in line and 'proof prove (try 1: xy 4.1 mm, normal -1.3 mm, drift 1.9 mm' in line


class FakeSend:
    def __init__(self):
        self.sent = []

    def fleet(self, cmd):
        self.sent.append(('fleet', cmd))

    def his(self, cmd):
        self.sent.append(('his', cmd))

    def give_up(self):
        self.sent.append(('give_up', True))


def test_gui_space_disarms_and_buttons_never_take_focus():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    from PyQt5 import QtCore
    from PyQt5.QtTest import QTest
    s = ready_state(now=time.time() + 30.0)                 # fresh for the whole test (the window reads the clock)
    s.on_log(40, 'fleet_manager', 'TAKEOFF REFUSED: drone(s) [1] not armed')
    send = FakeSend()
    seen = {}

    def drive(w, btns, space):
        seen['focus'] = {k: b.focusPolicy() for k, (b, _) in btns.items()}
        btns['arm'][0].click()                 # disabled (his ARM first): nothing goes out
        btns['his_arm'][0].click()
        w.activateWindow()
        QTest.qWait(100)                       # SPACE is an application shortcut: the window must be active
        btns['land'][0].setFocus()             # even a focused button must not take SPACE
        QTest.keyClick(w, QtCore.Qt.Key_Space)

    assert con.gui(s, send, exit_after_ms=900, on_ready=drive) == 0
    assert all(p == QtCore.Qt.NoFocus for p in seen['focus'].values())
    assert send.sent[0] == ('his', 'ARM') and ('fleet', 'ARM') not in send.sent
    assert send.sent[1:] == [('fleet', 'DISARM')] * con.DISARM_REPEATS


# ── a console started mid-flight (the live twin probe, 14 Oct): the trackers announce their arm only on a change ──

def test_started_mid_flight_it_still_knows_the_fleet_is_up():
    s = con.ConsoleState(4, partner=3)
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1)
    s.on_summary(summary(z=(1.0, 1.0, 1.0, 1.6)), NOW)                # first sample at hover: no rest learned
    for i in range(4):
        s.on_health(i, health(mode='AIR'), NOW)
    s.on_manager('TAKEOFF: flying.')
    assert [i for i, _ in con.airborne(s, NOW)] == [0, 1, 2, 3]
    assert not [k for k in s.rest_z if k.startswith('drone_')] and con.in_flight(s, NOW) and not con.ours_down(s, NOW)
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['land'] == (True, '') and b['give_up'] == (True, '')
    assert b['his_disarm'][0] is False and b['his_takeoff'] == (True, '')    # the fleet flies: his TAKEOFF allowed


def test_his_disarm_once_our_side_is_down_although_his_mpc_still_reads_armed():
    s = ready_state()
    s.on_his_armed(True)                                             # his MPC: still armed after our DISARM
    s.on_health(3, health(mode='AIR*'), NOW)                         # QUAD4's FC: disarmed by the mux
    s.on_manager('Disarm complete.')
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['his_disarm'] == (True, '') and con.ours_down(s, NOW)


def test_takeoff_does_not_wait_for_tracker_states_it_never_heard():
    s = ready_state()
    s.tracker_armed = [None] * 4                                     # started after the ARM: never heard them
    s.on_health(3, health(mode='AIR'), NOW)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    assert con.button_states(s, NOW, [])['takeoff'] == (True, '')
    s.on_tracker_armed(1, False)
    assert con.button_states(s, NOW, [])['takeoff'][0] is False


def test_the_twin_summary_is_fresh_for_longer():
    s = con.ConsoleState(4, partner=3, twin=True)
    s.on_summary(summary(), NOW)
    assert con.bodies(s, NOW + 2.0) is not None and con.bodies(s, NOW + 3.5) is None
    r = con.ConsoleState(4, partner=3)
    r.on_summary(summary(), NOW)
    assert con.bodies(r, NOW + 2.0) is None


def test_slot_gate_holds_ready_on_a_layout_off_its_plates():
    """--slot-gate (off by default): the planner's per-launch slot line over 30 deg holds READY (a3, 15 Oct)."""
    off = ('[planner] slot error (deg, + = clockwise): d2 +53.3 (plate 3), d1 +55.8 (plate 11), d3 +51.0 (plate 7)')
    ok = ('[planner] slot error (deg, + = clockwise): d3 +0.6 (plate 3), d2 +4.0 (plate 11), d1 -2.2 (plate 7)')
    s = ready_state()
    s.on_log(20, 'dissipative_planner', off)
    assert con.readiness(s, NOW) == []                       # the switch is off by default
    s.slot_gate = True
    assert any('56 deg off their plates' in w for w in con.readiness(s, NOW))
    s.on_log(20, 'dissipative_planner', ok)                  # the relaunch's line replaces it
    assert con.readiness(s, NOW) == []
