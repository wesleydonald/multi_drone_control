"""Fleet manager under the tilt cut (card 2026-10-11_quad4_tilt_cut): a tilt fault of a drone the partner flies (its
mux forwards the partner or has cut it) lands the fleet, never the emergency stop; while ours flies it the fault
changes nothing (its own disarm decides, as today); a drone-3 disarm while its mux reads 'cut' is not escalated; ARM is
refused while a mux reads 'cut'; at LANDED a mux still forwarding the partner is cut when that drone is at its rest,
otherwise the operator is told to DISARM (critic round 1, H1)."""
from std_msgs.msg import Bool

import fleet_manager.fleet_manager_node as m
from test_mux_abort_manager import _Rec, _fake, sync  # noqa: F401


def _tc(**kw):
    f = _fake(**kw)
    f.tilt_fault_land = f.landed_partner_cut = f.fc_disarm_land = True
    f._pose_z, f._rest_z, f._fc_flew, f._fc_landed_for = {}, {}, set(), set()
    f._cut_pubs = {i: _Rec() for i in range(f.num_drones)}
    f.status_pub = _Rec()
    C = m.CentralController
    for name in ('_tilt_fault_callback', '_cut_partner_at_rest', '_landed_callback', '_fc_state_callback'):
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


def test_arm_is_refused_while_a_mux_reads_cut(sync):
    f = _tc(mux_state={3: 'cut'})
    f._arm_fleet_thread()
    assert not f.fleet_armed and all(c.calls == 0 for c in f.arming_clients.values())


def test_at_landed_a_partner_stream_is_cut_only_at_its_rest(sync):
    f = _tc(mux_state={3: 'partner', 0: 'ours'})
    f._rest_z, f._pose_z = {3: 0.10, 0: 0.10}, {3: 0.081, 0: 0.10}   # R1503: QUAD4 lying tipped at 0.081
    f.landing = True
    f._landed_callback(Bool(data=True))
    assert [x.data for x in f._cut_pubs[3].msgs] == [True] and f._cut_pubs[0].msgs == []
    g = _tc(mux_state={3: 'partner'})
    g._rest_z, g._pose_z = {3: 0.10}, {3: 1.80}               # still up on his stream: told, not cut
    g.landing = True
    g._landed_callback(Bool(data=True))
    assert g._cut_pubs[3].msgs == [] and any('press DISARM' in x.data for x in g.status_pub.msgs)


def test_off_by_default_landed_cuts_nothing(sync):
    f = _tc(mux_state={3: 'partner'})
    f.landed_partner_cut = False
    f._rest_z, f._pose_z = {3: 0.10}, {3: 0.08}
    f.landing = True
    f._landed_callback(Bool(data=True))
    assert f._cut_pubs[3].msgs == []


def test_the_partners_own_disarm_after_it_flew_lands_the_fleet_once(sync):
    """Rig-port critic F6: his MPC's self-disarm (stale pose) or the FC's own disarm never reached us."""
    from std_msgs.msg import String
    f = _tc(mux_state={3: 'partner'})
    f.flying = True
    f._fc_state_callback(String(data='disarmed [/dev/QUAD4]'), 3)   # before he armed it: nothing
    assert _lands(f) == 0
    f._fc_state_callback(String(data='armed [/dev/QUAD4]'), 3)
    f._fc_state_callback(String(data='failed: FC reports disarmed (!ERR) [/dev/QUAD4]'), 3)
    f._fc_state_callback(String(data='disarmed [/dev/QUAD4]'), 3)
    assert _lands(f) == 1 and 3 in f._excused
    g = _tc(mux_state={3: 'ours'})                             # ours flies it: its tracker's disarm decides
    g.flying = True
    g._fc_state_callback(String(data='armed'), 3)
    g._fc_state_callback(String(data='disarmed'), 3)
    assert _lands(g) == 0
