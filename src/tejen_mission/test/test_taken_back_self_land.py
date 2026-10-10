"""taken_back_self_land (ours, card 2026-10-11_quad4_tilt_cut, rig-port critic F6): when the fleet's mux has not taken
the vehicle 1 s after TAKEN_BACK (its tracker cannot fly it), his planner flies out to 1.2 m from the ring centre, lands
and sends its landing disarm; its object magnet is TAKEN_BACK's own (m1-final d8c1b65, critic round 2 H-C): OFF after
the drop, no command while he carries the ball. Off (the default) TAKEN_BACK holds where it is, as before.
And the mux's 'cut' (critic round 2, M-E): his landing disarm once, the mission ended."""
from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import String

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import Recorder, planner, tick

RING = np.array([0.0, 0.0, 1.0])


def _taken_back(on=True, at=(0.1, 0.3, 1.6), mux='partner', phase=P.MATCH_VELOCITY):
    p = planner(phase=phase)
    p.taken_back_self_land = on
    p.taken_back_mux_wait_s, p.taken_back_clear_radius_m = 1.0, 1.2
    p.taken_back_step_speed, p.taken_back_land_z = 0.2, 0.20
    p.landing_descent_speed, p.landing_min_time_before_disarm_s = 0.25, 1.0
    p.landing_disarm_command = 'DISARM'
    p.drone_command_publisher = Recorder()
    p.payload_position = RING.copy()
    p.taken_back_t0 = p.taken_back_land = None
    p.cut_disarm_sent = False
    p.drone_position = np.array(at, float)
    p.last_reference = None
    p.mux_state = mux
    p.take_back_callback(SimpleNamespace(data=True))
    assert p.phase == P.TAKEN_BACK
    return p


def _run(p, dt):
    """One tick and his timer's magnet command, as his node does."""
    tick(p, dt)
    p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))


def _magnet(p):
    return [m.data for m in p.magnet_command_publisher.messages]


def _ref(p):
    return p.build_taken_back_land_reference().positions[0]


def test_no_switch_within_a_second_flies_out_lands_and_disarms_with_the_object_magnet_off_after_the_drop():
    p = _taken_back()                                          # MATCH_VELOCITY: his rejoin, after the drop
    _run(p, 0.5)
    assert p.taken_back_land is None                           # within the wait: holds
    _run(p, 0.6)
    land = p.taken_back_land
    assert land is not None
    assert np.isclose(np.hypot(*land['out'][:2]), 1.2) and land['out'][2] == 1.6
    assert np.allclose(land['out'][:2] / 1.2, np.array([0.1, 0.3]) / np.hypot(0.1, 0.3))   # radially out
    assert land['down'][2] == 0.20
    p.clock.t += 1.0
    assert _ref(p)[2] == 1.6 and np.hypot(*_ref(p)[:2]) > np.hypot(0.1, 0.3)             # stepping out first
    p.clock.t += 10.0
    assert _ref(p)[2] < 1.6                                                                 # then down
    p.drone_position = np.array([*land['out'][:2], 0.22])
    _run(p, 0.1)
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']
    _run(p, 0.1)
    assert len(p.drone_command_publisher.messages) == 1
    assert set(_magnet(p)) == {'OFF'}                          # TAKEN_BACK after the drop: OFF, held (H-C)


@pytest.mark.parametrize('phase', [P.LIFT_OBJECT, P.TRANSIT_TO_DROP_POINT, P.DESCEND_TO_DROP_HEIGHT])
def test_while_he_carries_the_ball_the_self_land_sends_no_object_magnet_command(phase):
    """F6a's falsifier (critic round 2, H-C): an OFF before the drop drops the ball wherever he is."""
    p = _taken_back(phase=phase)
    for _ in range(4):
        _run(p, 0.5)
    assert p.taken_back_land is not None
    p.drone_position = np.array([*p.taken_back_land['out'][:2], 0.22])
    p.clock.t += 20.0
    _run(p, 0.1)
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']
    assert _magnet(p) == [] and p.last_magnet_command == 'ON'


def test_a_mux_that_switched_to_ours_or_was_cut_keeps_the_hold():
    for mux in ('ours', 'cut', 'latched'):
        p = _taken_back(mux=mux)
        tick(p, 2.0)
        assert p.taken_back_land is None


def test_off_by_default_taken_back_holds():
    p = _taken_back(on=False)
    tick(p, 5.0)
    assert p.taken_back_land is None and p.drone_command_publisher.messages == []


def test_the_mux_cut_sends_his_landing_disarm_once_and_ends_the_mission():
    """Critic round 2, M-E: his MPC kept streaming armed after our cut, and a relaunched mux would forward it."""
    p = _taken_back(on=False, phase=P.TRANSIT_TO_DROP_POINT)
    p.phase = P.TRANSIT_TO_DROP_POINT
    p.mux_state_callback(String(data='partner'))
    assert p.drone_command_publisher.messages == []
    p.mux_state_callback(String(data='cut'))
    p.mux_state_callback(String(data='cut'))                   # latched, delivered again
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']
    assert p.phase == P.LANDED_DISARMED and p.mux_state == 'cut'


def test_the_partner_launch_passes_it_only_with_contact_attach():
    from test_partner_mission_geometry import _params
    assert 'taken_back_self_land' not in _params('rig_twin', contact_attach='true')['online_join_planner']
    assert _params('rig_twin', contact_attach='true', taken_back_self_land='true')[
        'online_join_planner']['taken_back_self_land'] is True
    assert 'taken_back_self_land' not in _params('rig_twin')['online_join_planner']
