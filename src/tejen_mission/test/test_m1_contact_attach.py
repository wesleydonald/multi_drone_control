"""contact_attach (combined M1 contact attach, docs/design/m1_contact_contract.md): his planner's
contact descent, LIFT_PROOF, the give-up, RING_ATTACHED and TAKEN_BACK, driven without ROS; and
that with the switch off his rejoin is unchanged."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.m1_pickup_recovery import PickupProgressStall, PickupRelease
from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import CONTACT_REJOIN_PHASES, OnlineJoinPlanner, RateProfile
from tejen_mission.payload_geometry import PayloadGeometryProfile

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
    p.warnings, p.infos = [], []
    log = SimpleNamespace(info=lambda msg, *a, **k: p.infos.append(msg),
                          warning=lambda msg, *a, **k: p.warnings.append(msg),
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
    p.input_timeout_s = 0.5
    p.last_magnet_tip_pose_time = p.last_attachment_pose_time = p.last_drone_time = clock.t
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
    p.contact_proof_profile = None
    p.contact_proof_fallback_logged = False
    p.contact_taken_back_magnet = None
    p.give_up_publisher = Recorder() if contact_attach else None
    return p


def tick(p, dt=0.0):
    """One planner tick with fresh poses (the measured ones arrive every tick)."""
    p.clock.t += dt
    p.last_magnet_tip_pose_time = p.last_attachment_pose_time = p.last_drone_time = p.clock.t
    if hasattr(p, 'last_pickup_object_pose_time'):
        p.last_pickup_object_pose_time = p.clock.t
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


# ---- LIFT_PROOF from the measured contact height (R1498) --------------------------------------

LOW_Z = -0.01            # contact_dz -0.01: his target 1 cm below the plate point


def test_a_low_target_lifts_the_full_rise_above_the_measured_contact():
    p = planner(phase=P.ATTACH_READY)
    p.final_attach_offset_z = LOW_Z
    p.measured_magnet_tip_position = PLATE.copy()     # the tip held on the plate, 1 cm above its target
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, LOW_Z])          # from the commanded offset
    p.clock.t += 10.0
    np.testing.assert_allclose(p.desired_tip_offset(), [0, 0, 0.01])           # rise_max above the contact
    np.testing.assert_allclose(p.build_reference().positions[0], PLATE + [0, 0, 0.01] + QUAD_FROM_TIP)


def test_a_failed_proof_after_a_low_target_presses_again_from_the_lift_reference():
    p = planner(phase=P.ATTACH_READY)
    p.final_attach_offset_z = LOW_Z
    p.contact_first_descend_s = p.clock.t
    p.contact_reported = True
    tick(p)
    p.clock.t += 0.75                     # 1.5 cm of the 2 cm lift
    lifted = p.desired_tip_offset()
    np.testing.assert_allclose(lifted, [0, 0, 0.005])
    p.contact_reported = False
    tick(p)
    assert p.phase == P.DESCEND_TO_ATTACHMENT and p.contact_failures == 1
    np.testing.assert_allclose(p.desired_tip_offset(), lifted)                 # no step at the re-press
    np.testing.assert_allclose(p.rate_profile().goal_tip_offset, [0, 0, LOW_Z])
    # the next contact re-latches the tip where it is measured then
    p.clock.t += 0.1                      # pressed 1 cm back down
    p.measured_magnet_tip_position = PLATE + [0, 0, 0.002]
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF
    np.testing.assert_allclose(p.rate_profile().start_tip_offset, [0, 0, -0.005])
    np.testing.assert_allclose(p.rate_profile().goal_tip_offset, [0, 0, 0.012])


def test_a_tip_pressed_below_the_target_still_lifts_the_full_rise_above_the_target():
    """R1522: the ring sagged under the press and the tip read 5-6 mm below the target; a rise from the tip ended
    4-5 mm above the commanded offset, his MPC barely moved and the proofs timed out."""
    p = planner(phase=P.ATTACH_READY)
    p.measured_magnet_tip_position = PLATE + [0, 0, -0.006]
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF
    np.testing.assert_allclose(p.rate_profile().start_tip_offset, [0, 0, CONTACT_Z])
    np.testing.assert_allclose(p.rate_profile().goal_tip_offset, [0, 0, CONTACT_Z + 0.01])
    line = next(m for m in p.infos if m.startswith('LIFT_PROOF:'))
    assert 'tip measured -0.006' in line and 'target offset +0.000' in line and 'lift to +0.010' in line


@pytest.mark.parametrize('target_z, tip_z, goal_z', [
    (0.0, -0.006, 0.010),     # pressed below the target (R1522): the target + rise, as before
    (LOW_Z, 0.0, 0.010),      # resting above a low target (R1498): the measured contact + rise
    (0.0, 0.0, 0.010),        # nominal
    (0.0, 0.003, 0.013),      # held above the target: from the measured contact
    (LOW_Z, -0.012, 0.0),     # below a low target too: the target + rise
])
def test_the_lift_rises_from_the_higher_of_the_target_and_the_measured_contact(target_z, tip_z, goal_z):
    p = planner(phase=P.ATTACH_READY)
    p.final_attach_offset_z = target_z
    p.measured_magnet_tip_position = PLATE + [0, 0, tip_z]
    p.contact_reported = True
    tick(p)
    assert p.rate_profile().goal_tip_offset[2] == pytest.approx(goal_z)
    assert p.rate_profile().goal_tip_offset[2] >= target_z + 0.01 - 1e-12     # never less than before


@pytest.mark.parametrize('tip_dz', [-0.0008, 0.0, 0.0008])
def test_the_nominal_target_lifts_as_before_within_a_millimetre(tip_dz):
    p = planner(phase=P.ATTACH_READY)
    p.measured_magnet_tip_position = PLATE + [0, 0, tip_dz]
    p.contact_reported = True
    tick(p)
    before = RateProfile([0, 0, CONTACT_Z], [0, 0, CONTACT_Z + 0.01], 0.02)   # the lift from the target
    for elapsed in np.arange(0.0, 1.5, 0.05):
        assert np.abs(p.rate_profile().tip_offset_at(elapsed) - before.tip_offset_at(elapsed)).max() <= 1e-3


def test_a_contact_during_the_descent_starts_the_lift_without_a_reference_step():
    p = planner(phase=P.MATCH_VELOCITY)
    p.final_attach_offset_z = LOW_Z
    descend(p)
    p.clock.t += (APPROACH_Z + 0.004) / p.descend_rate     # commanded 4 mm below the plate point
    p.measured_magnet_tip_position = PLATE.copy()
    before = p.build_reference().positions[0]
    np.testing.assert_allclose(before, PLATE + [0, 0, -0.004] + QUAD_FROM_TIP, atol=1e-9)
    p.contact_reported = True
    tick(p)
    assert p.phase == P.LIFT_PROOF
    np.testing.assert_allclose(p.build_reference().positions[0], before, atol=1e-9)
    np.testing.assert_allclose(p.rate_profile().goal_tip_offset, [0, 0, 0.01])


def test_a_stale_measured_tip_lifts_from_the_target_as_before_and_says_so_once():
    p = planner(phase=P.ATTACH_READY)
    p.final_attach_offset_z = LOW_Z
    p.contact_first_descend_s = p.clock.t
    p.last_magnet_tip_pose_time = p.clock.t - 1.0         # older than input_timeout_s
    p.contact_reported = True
    p.update_state_machine()
    assert p.phase == P.LIFT_PROOF
    profile = p.rate_profile()
    np.testing.assert_allclose(profile.start_tip_offset, [0, 0, LOW_Z])
    np.testing.assert_allclose(profile.goal_tip_offset, [0, 0, LOW_Z + 0.01])
    assert len(p.warnings) == 1
    p.contact_reported = False
    p.update_state_machine()
    p.contact_reported = True
    p.update_state_machine()
    assert p.phase == P.LIFT_PROOF and len(p.warnings) == 1
    p.use_measured_magnet_tip = False                     # his switch off: the target, as before
    p.last_magnet_tip_pose_time = p.clock.t
    p.contact_reported = False
    p.update_state_machine()
    p.contact_reported = True
    p.update_state_machine()
    assert p.rate_profile().goal_tip_offset[2] == pytest.approx(LOW_Z + 0.01)


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


def magnet_ticks(p, n=3, dt=1.0):
    """His timer's magnet command for n ticks; the commands published."""
    for _ in range(n):
        tick(p, dt)
        p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    return [m.data for m in p.magnet_command_publisher.messages]


def test_a_give_up_after_the_drop_turns_his_object_magnet_off():
    """R1520: given up in the rejoin with his object magnet ON, our step-out brought the tip within his 0.10 m
    catch of the ball lying in the ring's net, and the weld dragged the net."""
    p = planner(phase=P.LIFT_PROOF)
    p.contact_first_descend_s = p.clock.t
    p.contact_failures = 2
    tick(p)                                   # the third failed proof
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up
    sent = magnet_ticks(p)
    assert sent and set(sent) == {'OFF'} and p.last_magnet_command == 'OFF'
    assert p.warnings.count('TAKEN_BACK after the drop: object magnet OFF') == 1


@pytest.mark.parametrize('phase', sorted(CONTACT_REJOIN_PHASES | {P.CLEAR_DROP_ZONE}, key=lambda ph: ph.value))
def test_our_take_back_after_the_drop_turns_his_object_magnet_off(phase):
    p = planner(phase=phase)
    p.take_back_callback(SimpleNamespace(data=True))
    assert p.phase == P.TAKEN_BACK
    assert set(magnet_ticks(p)) == {'OFF'}


@pytest.mark.parametrize('phase', [P.LIFT_OBJECT, P.TRANSIT_TO_DROP_POINT, P.SETTLE_ABOVE_DROP_POINT,
                                   P.DESCEND_TO_DROP_HEIGHT])
def test_a_take_back_while_he_carries_the_ball_sends_no_object_magnet_command(phase):
    """Before the drop the ball stays on (on the rig one coil holds it)."""
    p = planner(phase=phase)
    p.take_back_callback(SimpleNamespace(data=True))
    assert p.phase == P.TAKEN_BACK
    assert magnet_ticks(p) == [] and p.last_magnet_command == 'ON'
    assert not any('object magnet OFF' in w for w in p.warnings)


# ---- his pickup under contact_attach: the descent ends on the weld; the bounded retry (R1493, R1497, R1507) ----

BALL = np.array([1.0, 1.6, 0.08])        # the rig-twin ball's centre at its start (r 0.08)
PUSHED = np.array([1.03, 1.65, 0.08])    # where it lies after a push or a drop
BALL_R, TIP_R, CATCH_R = 0.08, 0.012, 0.10   # his sim magnet manager welds the ball within 0.10 m of its centre
BENT = np.array([0.04, 0.14, 0.0])       # the tip pushed aside by a buckled rod (R1507: tip (1.04, 1.74))


def pickup_planner(retry=True, hold=True, weld=True):
    """His pickup as the partner launch flies it: legacy alignment, the geometry-aware target from the
    spanner_8mm profile, his own tracking errors from the measured tip, the hold on the weld, the retry with the
    launch's 0.10 m clearance and his other defaults. weld: the twin's catch (ON within CATCH_R; an OFF drops it)."""
    p = planner(phase=P.SETTLE_ABOVE_PICKUP)
    del p.tracking_errors
    p.pickup_alignment_mode = 'legacy'
    p.pickup_retry_enabled = retry
    p.pickup_hold_on_attach = hold
    p.weld, p.bend = weld, np.zeros(3)
    p.m1_pickup_stall = PickupProgressStall(wait_s=10.0, improvement_m=0.005)
    p.m1_pickup_release = PickupRelease(clearance_m=0.10)
    p.m1_candidate_hold_reference = None
    p.pickup_tier_descent = None
    p.pickup_proof_enabled = False
    p.pickup_proof_tracker = SimpleNamespace(reset=lambda: None)
    p.object_attached = False
    p.pickup_xy_tolerance, p.pickup_z_tolerance, p.pickup_tip_speed_tolerance = 0.10, 0.20, 0.60
    p.pickup_settle_time_s = 1.0
    p.pickup_approach_clearance, p.pickup_attach_clearance, p.pickup_descent_speed = 0.35, 0.09, 0.04
    p.pickup_lift_height, p.pickup_lift_speed = 2.3, 0.12
    p.pickup_lift_xy_tolerance, p.pickup_lift_xy_dwell_s = 0.08, 0.30
    p.attach_wait_start_time, p.attach_wait_timeout_s = None, 8.0
    p.enable_pickup_timeout_landing, p.pickup_land_after_s, p.takeoff_detected_time = True, 20.0, 0.0
    p.pickup_alignment_waiting_reason = 'inactive'
    p.use_geometry_aware_pickup = True
    p.payload_profile = PayloadGeometryProfile(
        name='8mm_spanner', dimensions=[0.1, 0.02, 0.006], geometry_origin_from_pose=np.zeros(3),
        pickup_point=[0.025, 0.0, 0.003], magnet_marker_to_contact_face=[0.0, 0.0, -0.03])
    p.pickup_contact_gap, p.pickup_overtravel, p.pickup_max_overtravel = 0.0, 0.002, 0.005
    p.support_surface_z, p.pickup_support_surface_overtravel_allowance_m = 0.0, 0.0
    p.use_static_pickup_object, p.payload_pose_timeout_s = False, 0.5
    p.pickup_object_position, p.pickup_object_yaw, p.pickup_object_velocity = BALL.copy(), 0.0, np.zeros(3)
    p.last_pickup_object_pose_time = p.clock.t
    p.latched_pickup_position = p.latched_pickup_yaw = None
    p.clear_geometry_pickup_latch()
    p.last_magnet_command = 'OFF'
    p.drone_position = BALL + [0.025, 0.0, 0.40] + QUAD_FROM_TIP
    p.measured_magnet_tip_position = p.drone_position - QUAD_FROM_TIP
    fly(p)
    return p


def fly(p, dt=0.0):
    """One tick as his timer runs it (state machine, magnet, reference). The tip then follows the reference but
    rests on the ball where the ball is under it; the ball welds as his sim manager welds it."""
    tick(p, dt)
    if p.phase in {P.TAKEN_BACK, P.LIFT_OBJECT}:
        return
    p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    body = (p.transfer_body_target().position if p.phase == P.APPROACH_ABOVE_PICKUP
            else p.build_reference().positions[0])
    p.last_reference = SimpleNamespace(positions=np.array([body]))
    tip = body - QUAD_FROM_TIP
    ball = p.pickup_object_position
    if np.linalg.norm(tip[:2] - ball[:2]) < BALL_R:
        tip[2] = max(tip[2], ball[2] + BALL_R + TIP_R)
    if p.weld:
        if p.last_magnet_command == 'OFF':
            p.object_attached = False
        elif np.linalg.norm(tip - ball) <= CATCH_R:
            p.object_attached = True
    if p.object_attached:
        tip = tip + p.bend
    p.measured_magnet_tip_position = tip
    p.drone_position = tip + QUAD_FROM_TIP


def fly_until(p, phases, dt=0.1, limit=400):
    for _ in range(limit):
        if p.phase in phases:
            return
        fly(p, dt)
    raise AssertionError(f'still in {p.phase.value}')


def to_magnet_wait(p):
    """SETTLE/APPROACH -> DESCEND_TO_PICKUP (latched on the ball where it is) -> MAGNET_ATTACH_WAIT."""
    fly_until(p, {P.DESCEND_TO_PICKUP})
    np.testing.assert_allclose(p.latched_pickup_position, p.pickup_object_position)
    fly_until(p, {P.MAGNET_ATTACH_WAIT})


def stall_and_rise(p):
    """Not on the ball for his wait: magnet OFF (an attached ball drops) and a vertical rise from the stall
    reference. Returns that reference."""
    assert p.phase == P.MAGNET_ATTACH_WAIT
    for _ in range(120):
        if p.m1_pickup_release.stage != 'idle':
            break
        fly(p, 0.1)
    stall = p.m1_pickup_release.target.copy()
    risen = stall
    for _ in range(400):
        fly(p, 0.05)
        assert p.last_magnet_command == 'OFF' and not p.object_attached
        if p.phase != P.MAGNET_ATTACH_WAIT:
            break
        np.testing.assert_allclose(p.last_reference.positions[0][:2], stall[:2])   # vertical only
        risen = p.last_reference.positions[0].copy()
    if p.phase == P.TAKEN_BACK:
        np.testing.assert_allclose(p.contact_hold_position, risen)
    return stall


def face_clearance(p):
    return float(p.measured_magnet_tip_position[2] - 0.03 - (p.pickup_object_position[2] + 0.003))


# the descent ends on the weld (R1507: the weld at a tip height of 0.175 m, then ~2 s more descent)

def test_the_descent_ends_on_the_weld_and_holds_the_tip_where_it_caught():
    p = pickup_planner()
    fly_until(p, {P.DESCEND_TO_PICKUP})
    contact = p.latched_geometry_pickup_contact_target.copy()
    while not p.object_attached:
        fly(p, 0.1)
        assert p.phase == P.DESCEND_TO_PICKUP
    caught = p.measured_magnet_tip_position.copy()
    assert caught[2] - contact[2] > 0.05                       # the welded tip on the ball, well above the target
    fly(p, 0.1)
    assert p.phase == P.MAGNET_ATTACH_WAIT
    hold = p.m1_candidate_hold_reference.copy()
    assert hold[2] == pytest.approx(caught[2] + QUAD_FROM_TIP[2])        # no further descent
    for _ in range(2):                                         # inside his 0.3 s lift dwell
        fly(p, 0.1)
        np.testing.assert_allclose(p.last_reference.positions[0], hold)
    fly_until(p, {P.LIFT_OBJECT})
    np.testing.assert_allclose(p.build_reference().positions[0], hold, atol=1e-9)   # the lift starts there
    assert not p.m1_pickup_release.attempts and not p.give_up_publisher.messages


def test_without_the_hold_the_descent_pushes_on_past_the_weld():
    p = pickup_planner(hold=False)
    fly_until(p, {P.DESCEND_TO_PICKUP})
    contact = p.latched_geometry_pickup_contact_target
    fly_until(p, {P.MAGNET_ATTACH_WAIT})
    assert p.object_attached and p.m1_candidate_hold_reference is None
    assert p.last_transition_reason == 'pickup descent profile complete'
    np.testing.assert_allclose(p.last_reference.positions[0], contact + QUAD_FROM_TIP, atol=1e-9)


# the retry: attached but off target, or nothing caught

def test_attached_but_off_target_drops_the_ball_rises_and_catches_on_the_retry():
    p = pickup_planner()
    p.bend = BENT                                              # the catch bends the rod aside
    to_magnet_wait(p)
    assert p.object_attached
    for _ in range(50):
        fly(p, 0.1)
    assert p.phase == P.MAGNET_ATTACH_WAIT                     # attached, 0.15 m off: never aligned
    stall = stall_and_rise(p)
    assert p.phase == P.APPROACH_ABOVE_PICKUP and p.m1_pickup_release.attempts == 1
    assert p.last_transition_reason == 'fresh measured vertical separation; retry pickup'
    assert p.last_reference.positions[0][2] > stall[2]
    p.pickup_object_position = PUSHED.copy()                   # where the dropped ball lies
    p.bend = np.zeros(3)
    fly(p, 0.1)
    approach = p.transfer_body_target().position - QUAD_FROM_TIP
    np.testing.assert_allclose(approach[:2], PUSHED[:2] + [0.025, 0.0])     # re-centred on it
    to_magnet_wait(p)
    fly_until(p, {P.LIFT_OBJECT})
    assert p.m1_pickup_release.attempts == 1 and not p.contact_gave_up


def test_a_rod_bent_on_every_catch_gives_up_after_the_third_clear_of_the_ball():
    p = pickup_planner()
    p.bend = BENT
    for attempt in (1, 2, 3):
        to_magnet_wait(p)
        assert p.object_attached
        stall = stall_and_rise(p)
        assert p.m1_pickup_release.attempts == attempt
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up and not p.object_attached
    assert [m.data for m in p.give_up_publisher.messages] == [True]
    np.testing.assert_allclose(p.contact_hold_position[:2], stall[:2])
    assert face_clearance(p) >= 0.10 and p.last_magnet_command == 'OFF'


def test_nothing_caught_rises_recentres_on_the_moved_ball_and_descends_again():
    p = pickup_planner(weld=False)
    to_magnet_wait(p)
    first_target = p.latched_geometry_pickup_contact_target.copy()
    p.pickup_object_position = PUSHED.copy()           # the rod pushed the ball; no weld
    stall = stall_and_rise(p)
    assert p.phase == P.APPROACH_ABOVE_PICKUP and p.m1_pickup_release.attempts == 1
    assert face_clearance(p) >= 0.10
    assert p.last_reference.positions[0][2] - stall[2] >= 0.09                # ~0.10 m up
    approach = p.transfer_body_target().position - QUAD_FROM_TIP
    np.testing.assert_allclose(approach[:2], PUSHED[:2] + [0.025, 0.0])
    to_magnet_wait(p)
    np.testing.assert_allclose(p.latched_geometry_pickup_contact_target - first_target,
                               PUSHED - BALL, atol=1e-12)
    assert p.last_magnet_command == 'ON' and not p.give_up_publisher.messages


def test_a_pickup_caught_on_the_retry_lifts_the_ball():
    p = pickup_planner(weld=False)
    to_magnet_wait(p)
    stall_and_rise(p)
    p.weld = True
    to_magnet_wait(p)
    fly_until(p, {P.LIFT_OBJECT})
    assert p.m1_pickup_release.attempts == 1 and not p.contact_gave_up


def test_the_third_failed_pickup_gives_up_clear_of_the_ball_and_hands_the_vehicle_back():
    p = pickup_planner(weld=False)
    for attempt in (1, 2, 3):
        to_magnet_wait(p)
        stall = stall_and_rise(p)
        assert p.m1_pickup_release.attempts == attempt
        if attempt < 3:
            assert p.phase == P.APPROACH_ABOVE_PICKUP
    assert p.phase == P.TAKEN_BACK and p.contact_gave_up
    assert [m.data for m in p.give_up_publisher.messages] == [True]
    # held where the rise stopped, above the ball; his magnet stays OFF (TAKEN_BACK sends nothing)
    np.testing.assert_allclose(p.contact_hold_position[:2], stall[:2])
    assert p.contact_hold_position[2] - stall[2] >= 0.09 and face_clearance(p) >= 0.10
    sent = len(p.magnet_command_publisher.messages)
    for _ in range(5):
        fly(p, 1.0)
        p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    assert p.phase == P.TAKEN_BACK and p.last_magnet_command == 'OFF'
    assert len(p.magnet_command_publisher.messages) == sent
    np.testing.assert_allclose(p.build_reference().positions, np.tile(p.contact_hold_position, (5, 1)))


def test_without_the_retry_a_stalled_pickup_waits_as_before():
    p = pickup_planner(retry=False, weld=False)
    to_magnet_wait(p)
    for _ in range(12):
        fly(p, 10.0)
    assert p.phase == P.MAGNET_ATTACH_WAIT and p.m1_pickup_release.stage == 'idle'
    assert not p.give_up_publisher.messages


def test_without_contact_attach_an_attached_tip_off_target_is_his_wait():
    """His own retry (measured_tip_tiered) counts only a missing object as a contact-wait stall."""
    p = pickup_planner()
    p.bend = BENT
    to_magnet_wait(p)
    p.contact_attach = False
    for _ in range(150):
        fly(p, 0.1)
    assert p.phase == P.MAGNET_ATTACH_WAIT and p.m1_pickup_release.stage == 'idle'


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
