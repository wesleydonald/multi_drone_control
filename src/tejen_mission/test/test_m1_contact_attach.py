"""contact_attach (combined M1 contact attach, docs/design/m1_contact_contract.md): his planner's
contact descent, LIFT_PROOF, the give-up, RING_ATTACHED and TAKEN_BACK, driven without ROS; and
that with the switch off his rejoin is unchanged."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import CONTACT_REJOIN_PHASES, OnlineJoinPlanner

CONTACT_Z = 0.0          # final_attach_offset_z under contact_attach (the launch sets it)
APPROACH_Z = 0.30
QUAD_FROM_TIP = np.array([0.0, 0.0, 0.59])
PLATE = np.array([0.0, 0.225, 1.032])


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def planner(contact_attach=True, phase=P.MATCH_VELOCITY):
    p = object.__new__(OnlineJoinPlanner)
    clock = SimpleNamespace(t=100.0)
    p.clock = clock
    p._physical_now_s = lambda: clock.t
    p._wall_now_s = lambda: clock.t
    log = SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None,
                          warn=lambda *a, **k: None, error=lambda *a, **k: None)
    p.get_logger = lambda: log
    p.mission = mission_for_mode('pickup_delivery', contact_attach=contact_attach)
    p.phase = phase
    p.phase_entry_time = clock.t
    p.condition_start_time = None
    p.publish_phase = lambda: None
    p.transfer_generator = SimpleNamespace(reset=lambda: None)
    p.is_m2b = False
    p.use_geometry_aware_pickup = False
    p.operator_advance_override_pending = False
    p.ground_ring_exit = None
    p.ground_ring_attach_trial = False
    p.c1f2_cpp_authority_enabled = False
    # rejoin geometry and gates as the partner launch sets them
    p.approach_offset_x = p.approach_offset_y = 0.0
    p.approach_offset_z = APPROACH_Z
    p.final_attach_offset_z = CONTACT_Z if contact_attach else 0.05
    p.descend_rate = 0.10
    p.match_xy_threshold, p.match_xy_velocity_threshold = 0.18, 0.20
    p.descend_abort_xy_threshold, p.descend_abort_xy_velocity_threshold = 0.25, 0.30
    p.dwell_time_s = 1.0
    p.reattach_transit_xy_threshold, p.approach_xy_threshold = 0.40, 0.25
    p.auto_descend = True
    p.enable_attach_ready_timeout_landing = False
    p.attach_ready_land_after_s = 0.0
    p.attach_ready_target_lead_time = 0.0 if contact_attach else 0.30
    # state
    p.drone_position = PLATE + QUAD_FROM_TIP
    p.drone_velocity = np.zeros(3)
    p.attachment_position = PLATE.copy()
    p.attachment_velocity = np.array([0.1, 0.0, 0.0])
    p.use_measured_magnet_tip = True
    p.measured_magnet_tip_position = PLATE.copy()
    p.magnet_drop_below_quad = 0.59
    p.quad_reference_offset_from_tip = lambda: QUAD_FROM_TIP.copy()
    p.errors = (0.0, 0.0, 0.0)
    p.tracking_errors = lambda: p.errors
    p.last_reference = SimpleNamespace(positions=np.array([[1.0, 2.0, 3.0]]))
    p.horizon_samples, p.dt = 5, 0.1
    p.min_reference_z, p.max_reference_speed = 0.0, 2.0
    p.external_landing_topic = '/join_planner/land_now'
    p.last_magnet_command = 'ON'
    p.magnet_command_publisher = Recorder()
    # the contact-mode parameters and state (the planner's defaults)
    p.contact_attach = contact_attach
    p.proof_rise_max, p.proof_rise_rate = 0.01, 0.02
    p.contact_max_failures, p.contact_give_up_s = 3, 30.0
    p.contact_match_xy_threshold, p.contact_match_xy_velocity_threshold = 0.025, 0.08
    p.contact_abort_xy_threshold, p.contact_abort_xy_velocity_threshold = 0.05, 0.15
    p.contact_reported = p.ring_attach_confirmed = p.take_back_requested = False
    p.mux_state = 'partner'
    p.contact_failures = 0
    p.contact_first_descend_s = None
    p.contact_gave_up = False
    p.contact_give_up_sent_s = None
    p.contact_descend_start_offset = None
    p.contact_hold_tip_offset = None
    p.contact_hold_position = None
    p.give_up_publisher = Recorder() if contact_attach else None
    return p


def tick(p, dt=0.0):
    p.clock.t += dt
    p.update_state_machine()


def descend(p):
    """MATCH_VELOCITY -> DESCEND_TO_ATTACHMENT through his own (contact) match gate."""
    p.phase = P.MATCH_VELOCITY
    tick(p)
    tick(p, 1.01)
    assert p.phase == P.DESCEND_TO_ATTACHMENT


# ---- DESCEND / ATTACH_READY -> LIFT_PROOF -------------------------------------------------

def test_contact_in_descend_starts_the_lift_proof_before_the_abort_guard():
    p = planner(phase=P.MATCH_VELOCITY)
    descend(p)
    p.errors = (0.20, 0.50, 0.0)          # his abort guard would send it back up
    p.contact_reported = True
    tick(p, 0.1)
    assert p.phase == P.LIFT_PROOF


def test_contact_in_attach_ready_starts_the_lift_proof_before_the_abort_guard():
    p = planner(phase=P.ATTACH_READY)
    p.errors = (0.20, 0.50, 0.0)
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF


def test_without_contact_the_contact_gates_hold_the_rejoin():
    p = planner(phase=P.ATTACH_READY)
    p.errors = (0.04, 0.10, 0.0)          # inside 0.05 m / 0.15 m/s
    tick(p)
    assert p.phase == P.ATTACH_READY
    p.errors = (0.06, 0.10, 0.0)
    tick(p)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    p = planner(phase=P.MATCH_VELOCITY)
    p.errors = (0.03, 0.0, 0.0)           # outside the 0.025 m match gate
    tick(p)
    tick(p, 2.0)
    assert p.phase == P.MATCH_VELOCITY


def test_attach_ready_raises_no_handoff_flag_and_lift_proof_keeps_the_magnet_on():
    m = mission_for_mode('pickup_delivery', contact_attach=True)
    assert not m.spec(P.ATTACH_READY).handoff_ready
    assert m.spec(P.LIFT_PROOF).magnet_command == 'ON'
    assert not any(spec.handoff_ready for spec in m.phases.values())


# ---- LIFT_PROOF ---------------------------------------------------------------------------

def test_lift_proof_rises_plate_relative_at_the_rate_capped_at_the_maximum():
    p = planner(phase=P.ATTACH_READY)
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF
    p.clock.t += 0.25
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, CONTACT_Z + 0.005])
    ref = p.build_reference()
    np.testing.assert_allclose(ref.positions[0], PLATE + [0, 0, CONTACT_Z + 0.005] + QUAD_FROM_TIP)
    # plate-relative with no lead: the ring's velocity plus the rise
    np.testing.assert_allclose(ref.velocities[0], [0.1, 0.0, 0.02])
    p.clock.t += 10.0
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, CONTACT_Z + 0.01])
    np.testing.assert_allclose(p.build_reference().velocities[0], [0.1, 0.0, 0.0])


def test_a_failed_proof_descends_again_from_the_lifted_offset_and_counts():
    p = planner(phase=P.ATTACH_READY)
    p.contact_first_descend_s = p.clock.t
    p.contact_reported = True
    tick(p)
    p.clock.t += 0.25                     # lifted 5 mm
    p.contact_reported = False
    tick(p)
    assert p.phase == P.DESCEND_TO_ATTACHMENT and p.contact_failures == 1
    # no step up to the approach height: the retry starts where the lift stopped
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, CONTACT_Z + 0.005])
    profile = p.rate_profile()
    np.testing.assert_allclose(profile.goal_tip_offset, [0, 0, CONTACT_Z])
    p.clock.t += profile.duration + 0.01
    tick(p)
    assert p.phase == P.ATTACH_READY
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF


def test_three_failed_proofs_give_up_and_hand_the_vehicle_back():
    p = planner(phase=P.MATCH_VELOCITY)
    descend(p)
    for failure in (1, 2, 3):
        p.contact_reported = True
        tick(p, 0.5)
        assert p.phase == P.LIFT_PROOF
        p.contact_reported = False
        tick(p, 0.5)
        assert p.contact_failures == failure
        if failure < 3:
            assert p.phase == P.DESCEND_TO_ATTACHMENT
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up
    assert [m.data for m in p.give_up_publisher.messages] == [True]
    np.testing.assert_allclose(p.contact_hold_position, [1.0, 2.0, 3.0])   # the last reference


def test_give_up_30_s_after_the_first_descent_start_whatever_the_phase():
    p = planner(phase=P.MATCH_VELOCITY)
    descend(p)
    first = p.contact_first_descend_s
    p.errors = (0.10, 0.0, 0.0)           # aborts back up: the timer keeps running
    tick(p, 5.0)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    p.phase = P.MATCH_VELOCITY
    p.errors = (0.0, 0.0, 0.0)
    tick(p)
    tick(p, 1.01)
    assert p.phase == P.DESCEND_TO_ATTACHMENT and p.contact_first_descend_s == first
    p.clock.t = first + 29.9
    tick(p)
    assert p.phase != P.TAKEN_BACK
    p.clock.t = first + 30.001
    tick(p)
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up
    # the give-up repeats once a second while it is latched
    p.publish_contact_give_up()
    p.clock.t += 1.0
    p.publish_contact_give_up()
    assert [m.data for m in p.give_up_publisher.messages] == [True, True]


def test_no_give_up_timer_before_the_first_descent():
    p = planner(phase=P.TRANSIT_TO_REATTACH)
    p.tracking_errors = lambda: (1.0, 0.0, 0.0)
    tick(p, 100.0)
    assert p.phase == P.TRANSIT_TO_REATTACH and not p.contact_gave_up


# ---- RING_ATTACHED ------------------------------------------------------------------------

@pytest.mark.parametrize('phase', sorted(CONTACT_REJOIN_PHASES, key=lambda ph: ph.value))
def test_ring_attached_from_every_rejoin_phase(phase):
    p = planner(phase=phase)
    p.ring_attach_confirmed = True
    p.errors = (1.0, 1.0, 0.0)
    tick(p)
    assert p.phase == P.RING_ATTACHED
    assert p.contact_hold_tip_offset is not None and p.contact_hold_position is None


def test_ring_attached_holds_the_lifted_offset_then_where_it_is_and_never_leaves():
    p = planner(phase=P.ATTACH_READY)
    p.contact_reported = True
    tick(p)
    p.clock.t += 0.4                      # lifted 8 mm
    p.ring_attach_confirmed = True
    tick(p)
    assert p.phase == P.RING_ATTACHED
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, CONTACT_Z + 0.008])
    ref = p.build_reference()             # plate-relative, no lead
    np.testing.assert_allclose(ref.positions[0], PLATE + [0, 0, CONTACT_Z + 0.008] + QUAD_FROM_TIP)
    np.testing.assert_allclose(ref.velocities[0], [0.1, 0.0, 0.0])
    p.mux_state = 'ours'
    p.drone_position = np.array([0.3, 0.4, 1.7])
    tick(p, 0.1)
    np.testing.assert_allclose(p.build_reference().positions, np.tile([0.3, 0.4, 1.7], (5, 1)))
    # our LAND, a take-back, contact loss, a lost alignment, the attach flag falling: no exit
    p.external_landing_callback(SimpleNamespace(data=True))
    p.take_back_callback(SimpleNamespace(data=True))
    p.contact_reported = p.ring_attach_confirmed = False
    p.errors = (2.0, 2.0, 2.0)
    for _ in range(5):
        tick(p, 60.0)
    assert p.phase == P.RING_ATTACHED
    assert not p.give_up_publisher.messages


def test_ring_attached_and_taken_back_send_no_magnet_command():
    p = planner(phase=P.LIFT_PROOF)
    p.m1_pickup_release = SimpleNamespace(stage='release')   # would otherwise force OFF
    for phase in (P.RING_ATTACHED, P.TAKEN_BACK):
        p.phase = phase
        command = p.pickup_magnet_command(p.current_phase_spec().magnet_command)
        p.publish_magnet_command(command)
    assert not p.magnet_command_publisher.messages
    assert p.last_magnet_command == 'ON'


# ---- TAKEN_BACK ---------------------------------------------------------------------------

@pytest.mark.parametrize('phase', [P.TRANSIT_TO_DROP_POINT, P.MATCH_VELOCITY, P.LIFT_PROOF, P.LANDING])
def test_take_back_ends_his_mission_from_any_phase(phase):
    p = planner(phase=phase)
    p.take_back_callback(SimpleNamespace(data=True))
    assert p.phase == P.TAKEN_BACK
    np.testing.assert_allclose(p.contact_hold_position, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(p.build_reference().positions, np.tile([1.0, 2.0, 3.0], (5, 1)))
    assert not p.contact_gave_up and not p.give_up_publisher.messages
    p.ring_attach_confirmed = True
    p.external_landing_callback(SimpleNamespace(data=True))
    tick(p, 60.0)
    assert p.phase == P.TAKEN_BACK


def test_take_back_seen_on_the_tick_too():
    p = planner(phase=P.DESCEND_TO_ATTACHMENT)
    p.take_back_requested = True
    tick(p)
    assert p.phase == P.TAKEN_BACK


# ---- the switch off -----------------------------------------------------------------------

def test_switch_off_keeps_his_mission_and_gates():
    legacy = mission_for_mode('pickup_delivery')
    assert legacy.spec(P.ATTACH_READY).handoff_ready
    assert not {P.LIFT_PROOF, P.RING_ATTACHED, P.TAKEN_BACK} & set(legacy.phases)
    assert len(legacy.phases) == 19
    with pytest.raises(ValueError):
        mission_for_mode('m2b', contact_attach=True)


def test_switch_off_ignores_the_contact_signals():
    p = planner(contact_attach=False, phase=P.ATTACH_READY)
    p.contact_reported = p.ring_attach_confirmed = p.take_back_requested = True
    p.errors = (0.20, 0.25, 0.0)          # inside his 0.25 / 0.30 abort gates
    tick(p, 100.0)
    assert p.phase == P.ATTACH_READY
    p.errors = (0.26, 0.0, 0.0)
    tick(p)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    p = planner(contact_attach=False, phase=P.DESCEND_TO_ATTACHMENT)
    p.contact_reported = True
    np.testing.assert_allclose(p.rate_profile().start_tip_offset, [0, 0, APPROACH_Z])
    tick(p, 100.0)
    assert p.phase == P.ATTACH_READY and p.contact_first_descend_s is None


def test_switch_off_match_gate_is_his():
    p = planner(contact_attach=False, phase=P.MATCH_VELOCITY)
    p.errors = (0.10, 0.10, 0.0)          # inside 0.18 / 0.20, outside the contact gates
    tick(p)
    tick(p, 1.01)
    assert p.phase == P.DESCEND_TO_ATTACHMENT
    np.testing.assert_allclose(p.rate_profile().start_tip_offset, [0, 0, APPROACH_Z])


# ---- multi_drone_control rig (critic F9): the drop ends on the measured object, a stuck one is a give-up ----

def _dropping(on_tip_offset, check=True, fresh=True):
    p = planner(phase=P.DROP_OBJECT)
    p.drop_check_puck, p.drop_stuck_radius_m = check, 0.08
    p.drop_wait_time_s, p.drop_start_time = 0.4, p.clock.t
    p.input_timeout_s = 0.25
    p.object_attached = False                        # his IRL flag clears with the OFF itself (inferred)
    p.measured_magnet_tip_position = PLATE + np.array([0.0, 0.0, 1.0])
    p.pickup_object_position = p.measured_magnet_tip_position + np.asarray(on_tip_offset, float)
    p.last_magnet_tip_pose_time = p.last_pickup_object_pose_time = p.clock.t if fresh else p.clock.t - 1.0
    return p


def test_drop_waits_the_release_dwell_even_with_the_flag_clear():
    p = _dropping([0.0, 0.0, -0.04])
    tick(p, 0.2)
    p.last_magnet_tip_pose_time = p.last_pickup_object_pose_time = p.clock.t
    assert p.phase == P.DROP_OBJECT


def test_drop_timeout_with_the_object_measured_on_the_magnet_is_a_give_up():
    p = _dropping([0.0, 0.0, -0.04])
    p.clock.t += 0.5
    p.last_magnet_tip_pose_time = p.last_pickup_object_pose_time = p.clock.t
    tick(p)
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up
    assert [m.data for m in p.give_up_publisher.messages] == [True]


def test_drop_clears_once_the_object_is_measured_away_or_not_measured():
    p = _dropping([0.0, 0.0, -0.6])
    p.clock.t += 0.5
    p.last_magnet_tip_pose_time = p.last_pickup_object_pose_time = p.clock.t
    tick(p)
    assert p.phase == P.CLEAR_DROP_ZONE
    p = _dropping([0.0, 0.0, -0.04], fresh=False)
    tick(p, 0.5)
    assert p.phase == P.CLEAR_DROP_ZONE and not p.contact_gave_up


def test_drop_without_the_switch_is_his():
    p = _dropping([0.0, 0.0, -0.04], check=False)
    tick(p)
    assert p.phase == P.CLEAR_DROP_ZONE               # his: the flag is clear, so the drop is done
