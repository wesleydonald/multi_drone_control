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


def test_his_land_needs_his_planner_listening_quad4_armed_and_a_phase_he_lands_from():
    s = ready_state()
    assert con.button_states(s, NOW, [])['his_land'] == (False, 'his planner not listening')
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1)
    assert con.button_states(s, NOW, [])['his_land'] == (False, 'QUAD4 not armed')
    s.on_health(3, health(mode='AIR'), NOW)                 # QUAD4's FC armed
    s.on_his_phase('MATCH_VELOCITY')
    assert con.button_states(s, NOW, [])['his_land'] == (True, '')
    s.on_his_phase('TAKEN_BACK')
    assert con.button_states(s, NOW, [])['his_land'] == (False, 'ours flies QUAD4: LAND')


def test_gui_his_land_sends_land_now():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    s = ready_state(now=time.time() + 30.0)
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1)
    s.on_health(3, health(mode='AIR'), time.time() + 30.0)
    s.on_his_phase('DESCEND_TO_ATTACHMENT')
    send = FakeSend()
    assert con.gui(s, send, exit_after_ms=500, on_ready=lambda w, btns, space: btns['his_land'][0].click()) == 0
    assert send.sent == [('his_land', True)]


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

    def his_land(self):
        self.sent.append(('his_land', True))

    def detach(self, i):
        self.sent.append(('detach', i))

    def force_drop(self):
        self.sent.append(('force_drop', True))

    def magnet_operator(self, cmd):
        self.sent.append(('magnet_operator', cmd))

    def his_to(self, d, cmd):
        self.sent.append(('his_to', d, cmd))

    def give_up_to(self, d):
        self.sent.append(('give_up_to', d))

    def his_land_to(self, d):
        self.sent.append(('his_land_to', d))

    def magnet_operator_to(self, d, cmd):
        self.sent.append(('magnet_operator_to', d, cmd))

    def detach_set(self, ids):
        self.sent.append(('detach_set', tuple(ids)))


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


def test_his_takeoff_is_held_until_our_release_only_while_quad4_is_ours():
    """5 Nov: a welded QUAD4 (mux 'ours') not yet released: his planner holds his TAKEOFF, so the 2 s ack waits."""
    s = con.ConsoleState(4, partner=3)
    s.mux[3] = 'ours'
    assert s.his_takeoff_held()
    s.on_release()
    assert not s.his_takeoff_held()
    s = con.ConsoleState(4, partner=3)
    s.mux[3] = 'partner'                                    # a floor start: his from boot
    assert not s.his_takeoff_held()


# ── Wesley 10 Nov: every button on the console (DETACH QUAD4, QUAD4 magnet OFF / ON) ──

def test_detach_once_per_flight_after_takeoff_with_a_listener():
    s = ready_state()
    assert con.button_states(s, NOW, [])['detach'] == (False, 'nothing listens on /fleet/detach')
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1, 1, 1)
    assert con.button_states(s, NOW, [])['detach'] == (False, 'after TAKEOFF')
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    s.on_manager('TAKEOFF: flying')
    assert con.button_states(s, NOW, [])['detach'] == (True, '')
    s.on_release()
    assert con.button_states(s, NOW, [])['detach'] == (False, 'QUAD4 already released this flight')
    s.on_manager('DISARMED')                                # whatever the reset text, a new ARM opens it again
    s.arm_complete = False
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    s.on_manager('TAKEOFF: flying')
    assert con.button_states(s, NOW, [])['detach'] == (True, '')


def test_magnet_operator_buttons_need_a_launch_that_listens():
    s = ready_state()
    b = con.button_states(s, NOW, [])
    assert b['q4_mag_off'] == (False, 'this launch has no magnet_operator_topic') and b['q4_mag_on'][0] is False
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1, 0, 1)
    b = con.button_states(s, NOW, [])
    assert b['q4_mag_off'] == (True, '') and b['q4_mag_on'] == (True, '')


def test_gui_detach_sends_quad4_and_magnet_off_needs_two_presses():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    s = ready_state(now=time.time() + 30.0)
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1, 1, 1)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    s.on_manager('TAKEOFF: flying')
    send = FakeSend()

    def drive(w, btns, space):
        btns['detach'][0].click()
        btns['q4_mag_off'][0].click()          # the first press only asks for a second
        btns['q4_mag_off'][0].click()
        btns['q4_mag_on'][0].click()

    assert con.gui(s, send, exit_after_ms=500, on_ready=drive) == 0
    assert send.sent == [('detach', 3), ('magnet_operator', 'OFF'), ('magnet_operator', 'ON')]
    assert s.detach_sent is True


def test_force_drop_only_in_his_drop_phases_with_his_planner_listening():
    s = ready_state()
    assert con.button_states(s, NOW, [])['force_drop'] == (False, 'his planner not listening')
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1, 1, 1, 1)
    s.on_his_phase('LIFT_OBJECT')
    assert con.button_states(s, NOW, [])['force_drop'][0] is False
    for ph in ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT', 'DESCEND_TO_DROP_HEIGHT'):
        s.on_his_phase(ph)
        assert con.button_states(s, NOW, [])['force_drop'] == (True, ''), ph
    s.on_his_phase('TRANSIT_TO_REATTACH')
    assert con.button_states(s, NOW, [])['force_drop'][0] is False


def test_gui_force_drop_publishes():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    s = ready_state(now=time.time() + 30.0)
    s.on_graph(ALL_NODES, set(con.HIS_NODES), 2, 1, 1, 1, 1, 1)
    s.on_his_phase('SETTLE_ABOVE_DROP_POINT')
    send = FakeSend()
    assert con.gui(s, send, exit_after_ms=500, on_ready=lambda w, btns, space: btns['force_drop'][0].click()) == 0
    assert send.sent == [('force_drop', True)]


def test_placement_warns_without_holding_ready():
    """QUAD4 or the object more than 1.3 m out in x or y: a warning on the ready line, never a reason (11 Nov)."""
    def with_xy(quad4_xy, puck_xy):
        doc = json.loads(summary())
        for i in range(4):
            doc['bodies'][f'drone_{i}'].update(x=0.0, y=0.0)
        doc['bodies']['drone_3'].update(x=quad4_xy[0], y=quad4_xy[1])
        if puck_xy is not None:
            doc['bodies']['puck'] = {'x': puck_xy[0], 'y': puck_xy[1], 'z': 0.03, 'tilt_deg': 0.0, 'age_s': 0.01}
        s = ready_state()
        s.on_summary(json.dumps(doc), NOW)
        return s
    s = with_xy((0.0, 1.12), (-0.6, 1.25))
    assert con.placement(s, NOW) == [] and con.readiness(s, NOW) == []
    s = with_xy((0.32, 1.60), (0.10, 1.83))                        # fm9hc_8's layout
    assert con.placement(s, NOW) == ['the object at x +0.10, y +1.83 m', 'QUAD4 at x +0.32, y +1.60 m']
    assert con.readiness(s, NOW) == []                             # still READY: it is a warning
    assert con.placement(ready_state(), NOW) == []                 # an older fleet_viz without x and y: silent


# ── 4 -> 2 -> 4 (--partners 3,2): a row per partner, DETACH BOTH and his TAKEOFF BOTH; off by default ──

D2_NODES = ['/tejen_d2/tejen_mpc', '/tejen_d2/online_join_planner']


def partners_state(now=NOW, partners=(3, 2), **kw):
    """Every node, link and listener up for QUAD4 (drone 3, today's topics) and QUAD3 (drone 2, /tejen_d2)."""
    s = con.ConsoleState(4, partners=list(partners), **kw)
    s.on_graph(ALL_NODES + D2_NODES, set(), 0, 0, detach_set_subs=1)
    for d in partners:
        s.on_partner_graph(d, con.partner_topics(d)['nodes'], 2, 1, 1, 1)
    s.on_summary(summary(), now)
    for i in range(4):
        s.on_health(i, health(), now)
        s.on_tracker_armed(i, False)
    return s


def test_partner_topics_follow_the_contract():
    """Partner 3 keeps every name it has today; any other d: ours under /drone_<d>, his under /tejen_d<d>."""
    assert con.partner_topics(3) == {
        'cmd': '/tejen/drone_command', 'give_up': '/join_planner/give_up', 'land': '/join_planner/land_now',
        'phase': '/join_planner/phase', 'armed': '/tejen/drone_arming_state_feedback', 'release': '/partner/release',
        'proof': '/magnet/attach_proof', 'contact': '/magnet/contact', 'attached': '/magnet/object_attached',
        'mag_op': '/drone_3/magnet_operator', 'nodes': ('/tejen/tejen_mpc', '/online_join_planner')}
    assert con.partner_topics(2) == {
        'cmd': '/tejen_d2/drone_command', 'give_up': '/tejen_d2/join_planner/give_up',
        'land': '/tejen_d2/join_planner/land_now', 'phase': '/tejen_d2/join_planner/phase',
        'armed': '/tejen_d2/drone_arming_state_feedback', 'release': '/drone_2/partner/release',
        'proof': '/drone_2/magnet/attach_proof', 'contact': '/drone_2/magnet/contact',
        'attached': '/drone_2/magnet/object_attached', 'mag_op': '/drone_2/magnet_operator',
        'nodes': ('/tejen_d2/tejen_mpc', '/tejen_d2/online_join_planner')}
    assert con.detach_set_text([3, 2]) == '2,3' and con.DETACH_SET == '/fleet/detach_set'


def test_command_line_partners_option():
    s, n = con.make_state([])
    assert (s.partner, s.partners, n) == (3, [], 4)
    s, _ = con.make_state(['--partners', '3'])                       # 3 alone: today's console exactly
    assert (s.partner, s.partners) == (3, [])
    s, _ = con.make_state(['--partners', '3,2'])
    assert (s.partner, s.partners, sorted(s.pp)) == (None, [3, 2], [2, 3])
    s, _ = con.make_state(['--no-partner'])
    assert (s.partner, s.partners) == (None, [])
    for bad in (['--partners', '2'], ['--partners', '3,3'], ['--partners', '3,4'], ['--partners', 'a,b'],
                ['--partners', '3,2', '--no-partner']):
        with pytest.raises(SystemExit):
            con.make_state(bad)


def test_partners_ready_needs_both_instances_and_allows_their_fcs_armed():
    s = partners_state()
    assert con.readiness(s, NOW) == []
    s.on_graph(ALL_NODES + D2_NODES[:1], set(), 0, 0, detach_set_subs=1)
    assert 'not running: his planner (QUAD3)' in con.readiness(s, NOW)
    s = partners_state()
    s.on_health(2, health(mode='AIR'), NOW)                         # his ARM of QUAD3: allowed before the fleet ARM
    s.on_health(1, health(mode='AIR'), NOW)                         # a carrier's FC armed: not
    assert con.fc_check(s, 2, NOW)[0] and not con.fc_check(s, 1, NOW)[0]


def test_partners_buttons_follow_the_flight_order():
    s = partners_state()
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['his_arm@3'] == (True, '') and b['his_arm@2'] == (True, '')
    assert b['arm'] == (False, 'his ARM first: QUAD4, QUAD3') and b['disarm'] == (True, '')
    assert 'detach' not in b and 'his_arm' not in b and 'force_drop' not in b      # no single-partner buttons
    s.on_health(3, health(mode='AIR'), NOW)
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['arm'] == (False, 'his ARM first: QUAD3') and b['his_arm@3'] == (False, 'QUAD4 armed')
    s.on_health(2, health(mode='AIR'), NOW)
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['arm'] == (True, '') and b['his_takeoff_both'] == (False, 'after the fleet ARM')
    for i in range(4):
        s.on_tracker_armed(i, True)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    b = con.button_states(s, NOW, con.readiness(s, NOW))
    assert b['takeoff'] == (True, '') and b['his_takeoff_both'] == (True, '') and b['his_takeoff@2'] == (True, '')
    assert b['detach_both'] == (False, 'after TAKEOFF') and b['give_up@2'] == (True, '')
    s.on_manager('TAKEOFF: flying')
    assert con.button_states(s, NOW, [])['detach_both'] == (True, '')
    s.on_partner(2, 'released', True)                               # one released (e.g. by the RViz DETACH)
    assert con.button_states(s, NOW, [])['detach_both'] == (False, 'QUAD3 already released this flight')
    s.on_manager('DISARMED')
    s.arm_complete = False
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')   # a new flight opens it again
    s.on_manager('TAKEOFF: flying')
    assert con.button_states(s, NOW, [])['detach_both'] == (True, '') and s.pp[2].released is False
    s.on_partner_graph(2, D2_NODES[:1], 1, 1, 1, 1)                 # QUAD3's planner gone: his TAKEOFF BOTH waits
    assert con.button_states(s, NOW, [])['his_takeoff_both'] == (False, 'QUAD3: waiting for online_join_planner')
    s.on_graph(ALL_NODES + D2_NODES, set(), 0, 0, detach_set_subs=0)
    assert con.button_states(s, NOW, [])['detach_both'] == (False, 'nothing listens on /fleet/detach_set')


def test_partners_his_land_and_magnet_buttons_are_per_partner():
    s = partners_state()
    s.on_health(2, health(mode='AIR'), NOW)
    s.on_partner(2, 'phase', 'DESCEND_TO_ATTACHMENT')
    s.on_partner(3, 'phase', 'TAKEN_BACK')
    s.on_health(3, health(mode='AIR'), NOW)
    b = con.button_states(s, NOW, [])
    assert b['his_land@2'] == (True, '') and b['his_land@3'] == (False, 'ours flies QUAD4: LAND')
    s.on_partner(2, 'phase', 'RING_ATTACHED')
    assert con.button_states(s, NOW, [])['his_land@2'] == (False, 'ours flies QUAD3: LAND')
    s.on_partner_graph(2, D2_NODES, 2, 1, 1, 0)                     # QUAD3's gate without an operator topic
    b = con.button_states(s, NOW, [])
    assert b['mag_off@2'] == (False, 'this launch has no magnet_operator_topic') and b['mag_on@3'] == (True, '')


def test_partners_line_and_takeoff_hold_per_partner():
    s = partners_state()
    s.mux[2] = s.mux[3] = 'ours'
    assert s.his_takeoff_held(2) and s.his_takeoff_held(3)
    s.on_partner(3, 'released', True)
    assert s.his_takeoff_held(2) and not s.his_takeoff_held(3)
    s.on_partner(2, 'proof', json.dumps({'state': 'prove', 'tries': 2, 'xy_m': 0.004}))
    s.on_partner(2, 'contact', True)
    s.on_partner(2, 'proof', 'not json')                             # ignored
    line = con.partner_line(s, 2, NOW)
    assert line.startswith('QUAD3 (drone 2): his phase --, not armed, release not yet. '
                           'Magnet: contact yes, proof prove')
    assert 'His link' not in line
    assert 'release sent' in con.partner_line(s, 3, NOW)


def test_gui_partners_buttons_publish_on_each_partners_topics():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    from PyQt5 import QtCore
    from PyQt5.QtTest import QTest
    t = time.time() + 30.0
    s = partners_state(now=t)
    for d in (2, 3):
        s.on_health(d, health(mode='AIR'), t)
    for i in range(4):
        s.on_tracker_armed(i, True)
    s.on_manager('ARM sequence complete: all 4 armed, ready for TAKEOFF.')
    s.on_manager('TAKEOFF: flying')
    s.on_partner(2, 'phase', 'DESCEND_TO_ATTACHMENT')
    send = FakeSend()
    seen = {}

    def drive(w, btns, space):
        seen['keys'] = list(btns)
        seen['labels'] = {k: lab for k, (_, lab) in btns.items()}
        btns['his_takeoff_both'][0].click()
        btns['detach_both'][0].click()
        btns['detach_both'][0].click()        # one set detach per flight: the second is not sent
        btns['give_up@2'][0].click()
        btns['his_land@2'][0].click()
        btns['mag_off@2'][0].click()          # the first press only asks for a second
        btns['mag_off@2'][0].click()
        btns['mag_on@3'][0].click()
        w.activateWindow()
        QTest.qWait(100)
        QTest.keyClick(w, QtCore.Qt.Key_Space)

    assert con.gui(s, send, exit_after_ms=900, on_ready=drive) == 0
    assert send.sent[:2] == [('his_to', 3, 'TAKEOFF'), ('his_to', 2, 'TAKEOFF')]      # both in one press
    assert send.sent[2:7] == [('detach_set', (3, 2)), ('give_up_to', 2), ('his_land_to', 2),
                              ('magnet_operator_to', 2, 'OFF'), ('magnet_operator_to', 3, 'ON')]
    assert send.sent[7:] == [('fleet', 'DISARM')] * con.DISARM_REPEATS                 # SPACE: the fleet DISARM
    assert s.detach_sent is True
    fleet, rows = con.partners_buttons([3, 2])
    assert seen['keys'] == [k for k, _, _ in fleet] + [k for d in (3, 2) for k, _, _ in rows[d]]
    assert seen['labels']['detach_both'] == 'DETACH BOTH (QUAD3+QUAD4)'
    assert seen['labels']['his_takeoff_both'] == 'his TAKEOFF BOTH (QUAD3+QUAD4)'
    assert seen['labels']['his_arm@2'] == 'QUAD3 his ARM' and seen['labels']['mag_on@3'] == 'QUAD4 magnet ON'


def test_gui_without_partners_keeps_todays_buttons():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    pytest.importorskip('PyQt5.QtWidgets')
    seen = {}

    def drive(w, btns, space):
        seen['buttons'] = [(k, lab) for k, (_, lab) in btns.items()]
    assert con.gui(ready_state(now=time.time() + 30.0), FakeSend(), exit_after_ms=300, on_ready=drive) == 0
    assert seen['buttons'] == [(k, lab) for k, lab, _ in con.BUTTONS]


# ── the ROS side without ROS: a stand-in rclpy records every topic the console opens ──

class _Msg:
    def __init__(self, data=None):
        self.data = data


class _Pub:
    def __init__(self, typ, topic):
        self.typ, self.topic, self.sent = typ, topic, []

    def publish(self, m):
        self.sent.append(m.data)

    def get_subscription_count(self):
        return 2


class _Node:
    def __init__(self):
        self.subs, self.pubs = {}, {}

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = (typ.__name__, cb)

    def create_publisher(self, typ, topic, qos):
        assert topic not in self.pubs, f'two publishers on {topic}'
        self.pubs[topic] = _Pub(typ.__name__, topic)
        return self.pubs[topic]

    def create_timer(self, period, cb):
        self.timer = cb

    def get_node_names_and_namespaces(self):
        return [('tejen_mpc', '/tejen_d2'), ('online_join_planner', '/tejen_d2'), ('fleet_manager', '/')]

    def get_subscriptions_info_by_topic(self, topic):
        from types import SimpleNamespace
        ns = {'/tejen_d2/drone_command': '/tejen_d2', '/tejen/drone_command': '/tejen'}.get(topic)
        if ns is None:
            return []
        planner_ns = '/' if ns == '/tejen' else ns
        return [SimpleNamespace(node_namespace=ns, node_name='tejen_mpc'),
                SimpleNamespace(node_namespace=planner_ns, node_name='online_join_planner')]

    def count_subscribers(self, topic):
        return 1


@pytest.fixture
def fake_ros(monkeypatch):
    import types
    node = _Node()
    mods = {'rclpy': types.SimpleNamespace(init=lambda: None, create_node=lambda name: node),
            'rclpy.qos': types.SimpleNamespace(DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                               ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
                                               QoSProfile=lambda **kw: kw),
            'rcl_interfaces.msg': types.SimpleNamespace(Log=type('Log', (_Msg,), {})),
            'std_msgs.msg': types.SimpleNamespace(Bool=type('Bool', (_Msg,), {}), Int32=type('Int32', (_Msg,), {}),
                                                  String=type('String', (_Msg,), {}))}
    for k, v in mods.items():
        monkeypatch.setitem(sys.modules, k, v)
    return node


LEGACY_SUBS = {**{f'/drone_{i}/{t}': typ for i in range(4)
                  for t, typ in (('arming_state_feedback', 'Bool'), ('radio_health', 'String'),
                                 ('mux_state', 'String'))},
               '/tejen/drone_arming_state_feedback': 'Bool', '/fleet/mocap_summary': 'String',
               '/fleet/manager_status': 'String', '/fleet/phase': 'String', '/join_planner/phase': 'String',
               '/partner/release': 'String', '/magnet/attach_proof': 'String', '/magnet/contact': 'Bool',
               '/magnet/object_attached': 'Bool', '/rosout': 'Log'}
LEGACY_PUBS = {'/fleet/command': 'String', '/tejen/drone_command': 'String', '/join_planner/give_up': 'Bool',
               '/join_planner/land_now': 'Bool', '/fleet/detach': 'Int32', '/join_planner/force_drop': 'Bool',
               '/drone_3/magnet_operator': 'String'}


def test_ros_topics_without_partners_are_todays(fake_ros):
    s = con.ConsoleState(4, partner=3)
    con.RosLink(s, 4)
    assert {t: typ for t, (typ, _) in fake_ros.subs.items()} == LEGACY_SUBS
    assert {t: p.typ for t, p in fake_ros.pubs.items()} == LEGACY_PUBS
    fake_ros.timer()
    assert con.his_link(s) == (True, '') and (s.give_up_subs, s.detach_subs, s.mag_op_subs) == (1, 2, 2)


def test_ros_topics_with_partners_and_the_set_detach(fake_ros):
    s = con.ConsoleState(4, partners=[3, 2])
    link = con.RosLink(s, 4)
    want_subs = {t: typ for t, typ in LEGACY_SUBS.items()}
    want_subs.update({'/tejen_d2/drone_arming_state_feedback': 'Bool', '/tejen_d2/join_planner/phase': 'String',
                      '/drone_2/partner/release': 'String', '/drone_2/magnet/attach_proof': 'String',
                      '/drone_2/magnet/contact': 'Bool', '/drone_2/magnet/object_attached': 'Bool'})
    assert {t: typ for t, (typ, _) in fake_ros.subs.items()} == want_subs
    assert {t: p.typ for t, p in fake_ros.pubs.items()} == {
        '/fleet/command': 'String', '/tejen/drone_command': 'String', '/join_planner/give_up': 'Bool',
        '/join_planner/land_now': 'Bool', '/drone_3/magnet_operator': 'String',
        '/tejen_d2/drone_command': 'String', '/tejen_d2/join_planner/give_up': 'Bool',
        '/tejen_d2/join_planner/land_now': 'Bool', '/drone_2/magnet_operator': 'String',
        '/fleet/detach_set': 'String'}
    link.detach_set([3, 2])
    link.his_to(2, 'TAKEOFF')
    link.his_to(3, 'TAKEOFF')
    link.give_up_to(2)
    link.magnet_operator_to(2, 'OFF')
    p = fake_ros.pubs
    assert p['/fleet/detach_set'].sent == ['2,3']
    assert p['/tejen_d2/drone_command'].sent == ['TAKEOFF'] and p['/tejen/drone_command'].sent == ['TAKEOFF']
    assert p['/tejen_d2/join_planner/give_up'].sent == [True] and p['/drone_2/magnet_operator'].sent == ['OFF']
    # each partner's topics feed its own row
    fake_ros.subs['/drone_2/partner/release'][1](_Msg('{"running": true}'))
    fake_ros.subs['/tejen_d2/join_planner/phase'][1](_Msg('MATCH_VELOCITY'))
    fake_ros.subs['/magnet/object_attached'][1](_Msg(True))
    assert s.pp[2].released and s.pp[2].phase == 'MATCH_VELOCITY' and not s.pp[3].released
    assert s.pp[3].attached is True and s.pp[2].attached is None
    fake_ros.timer()                                                # the graph poll
    assert con.his_link(s, 2) == (True, '') and con.his_link(s, 3) == (True, '')
    assert s.detach_set_subs == 2 and s.pp[2].give_up_subs == 1 and s.pp[2].mag_op_subs == 2


def test_detach_both_rearmed_after_unload_cancelled():
    """--partners: a cancelled set unload leaves both magnets ON, so DETACH BOTH is offered again; today's console keeps
    its one DETACH per flight."""
    s = con.ConsoleState(4, partners=[3, 2])
    s.detach_sent = True
    s.on_log(30, 'dissipative_planner', '[dissipative] UNLOAD CANCELLED for drone [2, 3]: tilt; tensions blend back '
                                        'over 3.0 s, magnets stay ON')
    assert s.detach_sent is False
    t = con.ConsoleState(4, partner=3)
    t.detach_sent = True
    t.on_log(30, 'dissipative_planner', '[dissipative] UNLOAD CANCELLED for drone 3: tilt; magnet stays ON')
    assert t.detach_sent is True
