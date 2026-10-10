"""Fleet manager under the tilt cut (card 2026-10-11_quad4_tilt_cut): a tilt fault of a drone the partner flies (its
mux forwards the partner or has cut it) lands the fleet, never the emergency stop; while ours flies it the fault
changes nothing (its own disarm decides, as today); a drone-3 disarm while its mux reads 'cut' is not escalated; ARM is
refused while a mux reads 'cut' (his DISARM, then relaunch T2); at LANDED a mux still forwarding the partner is cut
when that drone is down and still (under 0.25 m, under 0.1 m/s: critic round 2, L-C), otherwise the operator is told
to DISARM (critic round 1, H1); his stream's disarm after the drone was airborne lands the fleet (M-B); one LAND per
flight from either (M-C)."""
from types import SimpleNamespace

from std_msgs.msg import Bool, String

import fleet_manager.fleet_manager_node as m
from test_mux_abort_manager import _Rec, _fake, sync  # noqa: F401


def _tc(**kw):
    f = _fake(**kw)
    f.tilt_fault_land = f.landed_partner_cut = f.fc_disarm_land = True
    f._pose_z, f._rest_z, f._fc_flew, f._fc_landed_for = {}, {}, set(), set()
    f._pose_speed, f._fc_armed, f._fault_land_sent = {}, {}, False
    f._cut_pubs = {i: _Rec() for i in range(f.num_drones)}
    f.status_pub = _Rec()
    C = m.CentralController
    for name in ('_tilt_fault_callback', '_cut_partner_at_rest', '_landed_callback', '_fc_state_callback',
                 '_fault_land', '_tilt_pose_callback'):
        setattr(f, name, getattr(C, name).__get__(f))
    return f


def _lands(f):
    return [x.data for x in f.fleet_cmd_pub.msgs].count('LAND')


def test_a_tilt_fault_while_the_partner_flies_it_lands_the_fleet_once(sync):
    f = _tc(mux_state={3: 'partner'})
    f.flying = True
    f._tilt_fault_callback(Bool(data=True), 3)
    assert _lands(f) == 1 and 3 in f._excused and f.abort_pub.msgs == []
    f.landing = True
    f._tilt_fault_callback(Bool(data=True), 3)               # a repeat during LAND: no second LAND
    assert _lands(f) == 1


def test_its_tracker_disarm_after_the_cut_is_not_escalated(sync):
    f = _tc(mux_state={3: 'cut'})
    f.flying = True
    f.drone_armed[3] = True
    f._arming_feedback_callback(Bool(data=False), 3)
    assert f.abort_pub.msgs == [] and f.service_disarms == []


def test_while_ours_flies_it_its_own_disarm_decides(sync):
    """Ours flying QUAD4 (on the ring, our step-out or take-back), or QUAD4 already disarmed alone at its rest: the
    fault adds nothing (rig 7-8 Oct: a drone disarmed alone that topples at its rest must not land the fleet)."""
    for on_load in (False, True):
        f = _tc(mux_state={3: 'ours'}, on_load={3: on_load})
        f.flying = True
        f._tilt_fault_callback(Bool(data=True), 3)
        assert _lands(f) == 0 and 3 not in f._excused
    h = _tc(mux_state={3: 'partner'})
    h._tilt_fault_callback(Bool(data=True), 3)                # not flying: nothing
    assert _lands(h) == 0


def _pose(z, speed=0.0):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=z)),
                           twist=SimpleNamespace(linear=SimpleNamespace(x=speed, y=0.0, z=0.0)))


def test_arm_is_refused_while_a_mux_reads_cut(sync):
    f = _tc(mux_state={3: 'cut'})
    f._arm_fleet_thread()
    assert not f.fleet_armed and all(c.calls == 0 for c in f.arming_clients.values())
    assert any('his DISARM, then relaunch T2' in x.data for x in f.status_pub.msgs)


def test_at_landed_a_partner_stream_is_cut_only_when_down_and_still(sync):
    f = _tc(mux_state={3: 'partner', 0: 'ours'})
    f._tilt_pose_callback(_pose(0.081), 3)                    # R1503: QUAD4 lying tipped at 0.081
    f._tilt_pose_callback(_pose(0.10), 0)
    f.landing = True
    f._landed_callback(Bool(data=True))
    assert [x.data for x in f._cut_pubs[3].msgs] == [True] and f._cut_pubs[0].msgs == []
    for z, speed in ((1.80, 0.0), (0.60, 0.0), (0.15, 0.4)):  # up on his stream; a welded start's ARM height
        g = _tc(mux_state={3: 'partner'})                     # (L-C: its rest was ~0.6 m); still moving
        g._rest_z = {3: 0.60}
        g._tilt_pose_callback(_pose(z, speed), 3)
        g.landing = True
        g._landed_callback(Bool(data=True))
        assert g._cut_pubs[3].msgs == [] and any('press DISARM' in x.data for x in g.status_pub.msgs)


def test_off_by_default_landed_cuts_nothing(sync):
    f = _tc(mux_state={3: 'partner'})
    f.landed_partner_cut = False
    f._pose_z, f._pose_speed = {3: 0.08}, {3: 0.0}
    f.landing = True
    f._landed_callback(Bool(data=True))
    assert f._cut_pubs[3].msgs == []


def test_his_streams_disarm_after_it_was_airborne_lands_the_fleet_once(sync):
    """Rig-port critic F6: his MPC's self-disarm (stale pose) never reached us. Airborne first (M-B): his ARM and
    DISARM on the floor before his takeoff change nothing."""
    f = _tc(mux_state={3: 'partner'})
    f.flying = True
    f._rest_z = {3: 0.08}
    f._tilt_pose_callback(_pose(0.08), 3)
    f._fc_state_callback(String(data='disarmed [/dev/QUAD4]'), 3)   # before he armed it: nothing
    f._fc_state_callback(String(data='armed [/dev/QUAD4]'), 3)
    f._fc_state_callback(String(data='disarmed [/dev/QUAD4]'), 3)   # armed on the floor, then disarmed: nothing
    assert _lands(f) == 0 and 3 not in f._fc_flew
    f._fc_state_callback(String(data='armed [/dev/QUAD4]'), 3)
    f._tilt_pose_callback(_pose(0.35), 3)                     # his takeoff: airborne
    f._fc_state_callback(String(data='failed: FC reports disarmed (!ERR) [/dev/QUAD4]'), 3)
    f._fc_state_callback(String(data='disarmed [/dev/QUAD4]'), 3)
    assert _lands(f) == 1 and 3 in f._excused
    g = _tc(mux_state={3: 'ours'})                             # ours flies it: its tracker's disarm decides
    g.flying = True
    g._rest_z = {3: 0.08}
    g._fc_state_callback(String(data='armed'), 3)
    g._tilt_pose_callback(_pose(1.0), 3)
    g._fc_state_callback(String(data='disarmed'), 3)
    assert _lands(g) == 0


def test_the_tilt_fault_and_his_disarm_after_one_upset_send_one_land(sync):
    """Critic round 2, M-C (the likely default set): the tilt fault LANDs; his stream's disarm reported before the
    mux's 'cut' reaches the manager must not LAND again (the LAND reaches _land_fleet only through the topic)."""
    f = _tc(mux_state={3: 'partner'})
    f.flying = True
    f._rest_z = {3: 0.08}
    f._fc_state_callback(String(data='armed'), 3)
    f._tilt_pose_callback(_pose(2.4), 3)
    f._tilt_fault_callback(Bool(data=True), 3)
    f._fc_state_callback(String(data='disarmed'), 3)
    f.mux_state[3] = 'cut'
    f._tilt_fault_callback(Bool(data=True), 3)
    assert _lands(f) == 1
    g = _tc(mux_state={3: 'partner'})                          # the other order
    g.flying = True
    g._rest_z = {3: 0.08}
    g._fc_state_callback(String(data='armed'), 3)
    g._tilt_pose_callback(_pose(2.4), 3)
    g._fc_state_callback(String(data='disarmed'), 3)
    g._tilt_fault_callback(Bool(data=True), 3)
    assert _lands(g) == 1
