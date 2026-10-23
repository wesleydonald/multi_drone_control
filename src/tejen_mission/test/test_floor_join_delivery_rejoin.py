"""floor_join and delivery_rejoin (Wesley 14 Oct 2026): QUAD4 from its mat, WAIT_FOR_TAKEOFF -> TAKEOFF as in
pickup_delivery. floor_join climbs over the mat to the approach height and goes straight to the rejoin; delivery_rejoin
takes off holding the object (magnet ON from boot), climbs to where a loaded lift ends, prepares the C++ transit there
as LIFT_OBJECT does, and continues as pickup_delivery after its pickup. A delivery TAKEOFF without the object attached is
refused on the mat with his landing disarm (his MPC takes off on the same command); the object lost in the climb lands
the vehicle. Driven without ROS; pickup_delivery's takeoff is unchanged."""
import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.mission_definitions import PICKUP_PHASES, mission_for_mode
from tejen_mission.mission_types import MissionDefinition
from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import PLATE, QUAD_FROM_TIP, Recorder, _dropping, planner, tick

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'
MAT = np.array([1.5, -1.0, 0.05])           # QUAD4's body on its mat
ROD = float(QUAD_FROM_TIP[2])               # body to magnet marker (the harness's magnet_drop_below_quad)
TIP_ON_FLOOR_Z = 0.06                       # the marker lying on the floor (the rig's tether slack)
PUCK_BELOW_TIP = 0.039                      # the puck's centre below the marker (partner_backend_overrides_rig)
LIFT = 2.3                                  # the rig's pickup_lift_height
RING = np.array([0.0, 0.0, 1.0])

CONTACT = [P.LIFT_PROOF, P.RING_ATTACHED, P.TAKEN_BACK]
REJOIN = [P.APPROACH_ABOVE_TARGET, P.MATCH_VELOCITY, P.DESCEND_TO_ATTACHMENT, P.ATTACH_READY,
          P.LANDING, P.LANDED_DISARMED]
FLOOR_JOIN = [P.WAIT_FOR_TAKEOFF, P.TAKEOFF, *REJOIN]
DELIVERY_REJOIN = [P.WAIT_FOR_TAKEOFF, P.TAKEOFF, P.TRANSIT_TO_DROP_POINT, P.SETTLE_ABOVE_DROP_POINT,
                   P.DESCEND_TO_DROP_HEIGHT, P.DROP_OBJECT, P.CLEAR_DROP_ZONE, P.TRANSIT_TO_REATTACH, *REJOIN]


def floor_planner(mode, attached=True, c1f2=False):
    """His planner on the mat in WAIT_FOR_TAKEOFF as the rig launch configures it (contact_attach), the measured
    marker below the body and, for a delivery, the object on it."""
    p = planner(phase=P.WAIT_FOR_TAKEOFF)
    p.mission = mission_for_mode(mode, contact_attach=True)
    p.is_floor_join = p.mission.name == 'floor_join'
    p.is_delivery_rejoin = p.mission.name == 'delivery_rejoin'
    p.takeoff_climb_height = p.delivery_object_start_z = None
    p.takeoff_start_position = p.takeoff_command_time = p.takeoff_detected_time = p.initial_quad_z = None
    p.takeoff_height, p.takeoff_speed, p.takeoff_detection_height = 0.50, 0.15, 0.12
    p.takeoff_reference_acceleration_limit_mps2 = 0.50
    p.pickup_z_tolerance, p.loaded_lift_z_tolerance = 0.20, 0.05      # irl_commissioning.yaml
    p.pickup_lift_height, p.lift_complete_object_clearance = LIFT, 0.08
    p.landing_disarm_command, p.landing_disarm_sent = 'DISARM', False
    p.landing_disarm_height_above_start = 0.20
    p.landing_target_xy = p.landing_start_z = p.landing_disarm_z = None
    p.drone_command_publisher = Recorder()
    p.m2c_ground_reference_enabled = False
    p.visual_trajectory_acceleration_filter_tau_s = 0.15
    p.external_landing_pending = False
    p.last_magnet_command = None                                       # boot: nothing sent yet
    p.object_attached = attached
    p.pickup_object_velocity = np.zeros(3)
    place(p, MAT)
    # C1F.2 (c1f2b_cpp_authority.yaml, loaded on both launch paths); the C++ backend is not here
    p.c1f1_shadow_transfer_enabled = True
    p.c1f2_cpp_authority_enabled = c1f2
    p.c1f2_authority_publisher = Recorder()
    p.c1f1_shadow_enable_publisher = None
    p.reset_c1f2_for_pickup_retry()
    p.c1f2_prepare_count, p.c1f2_prepare_drift_m, p.c1f2_prepare_speed_mps = 0, 0.0, 0.0
    p.c1f2_authority_switch_count = 0
    p.c1f2_prepare_max_lift_drift_m, p.c1f2_prepare_settle_speed_mps = 0.15, 0.08
    p.c1f2_prepare_settle_acceleration_mps2, p.c1f2_prepare_settle_dwell_s = 0.50, 0.30
    return p


def place(p, body, velocity=(0.0, 0.0, 0.0)):
    """The vehicle at `body` with fresh poses: the marker below it (on the floor until the tether is taut) and the
    object on the marker, or where it was left (`left_object`)."""
    p.drone_position = np.asarray(body, float).copy()
    p.drone_velocity = np.asarray(velocity, float).copy()
    p.drone_acceleration = np.zeros(3)
    tip = p.drone_position - QUAD_FROM_TIP
    tip[2] = max(tip[2], TIP_ON_FLOOR_Z)
    p.measured_magnet_tip_position = tip
    left = getattr(p, 'left_object', None)
    p.pickup_object_position = (left.copy() if left is not None
                                else tip - [0.0, 0.0, PUCK_BELOW_TIP])
    p.last_drone_time = p.last_magnet_tip_pose_time = p.last_pickup_object_pose_time = p.clock.t
    p.last_attachment_pose_time = p.last_object_attached_time = p.clock.t


def command(p, word):
    p.drone_command_callback(SimpleNamespace(data=word))


def state(p, body):
    """One mocap sample through his drone_state_callback (the TAKEOFF detection exit)."""
    x, y, z = (float(v) for v in body)
    p.drone_state_callback(SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z), orientation=None),
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0))))


def fly(p, dt=0.1):
    """One tick, then the vehicle (and the object with it) where the reference puts it."""
    p.last_object_attached_time = p.clock.t + dt
    tick(p, dt)
    if p.phase not in {P.TAKEOFF}:
        return
    ref = p.build_reference()
    place(p, ref.positions[0])
    p.last_reference = ref


def climb(p, until, limit=600):
    for _ in range(limit):
        if until(p):
            return
        fly(p)
    raise AssertionError(f'still in {p.phase.value}')


# ---- the phase sequences ------------------------------------------------------------------

@pytest.mark.parametrize('contact', [False, True])
def test_the_phase_sequences(contact):
    extra = CONTACT if contact else []
    floor = mission_for_mode('floor_join', contact_attach=contact)
    delivery = mission_for_mode('delivery_rejoin', contact_attach=contact)
    assert list(floor.phases) == FLOOR_JOIN + extra
    assert list(delivery.phases) == DELIVERY_REJOIN + extra
    assert floor.initial_phase == delivery.initial_phase == P.WAIT_FOR_TAKEOFF
    assert (floor.name, delivery.name) == ('floor_join', 'delivery_rejoin')
    assert not set(PICKUP_PHASES) & (set(floor.phases) | set(delivery.phases))


@pytest.mark.parametrize('contact', [False, True])
def test_the_shared_phases_are_pickup_delivery_and_join_only_specs(contact):
    full = mission_for_mode('pickup_delivery', contact_attach=contact).phases
    join = mission_for_mode('join', contact_attach=contact).phases
    floor = mission_for_mode('floor_join', contact_attach=contact).phases
    delivery = mission_for_mode('delivery_rejoin', contact_attach=contact).phases
    for phase in (P.WAIT_FOR_TAKEOFF, P.TAKEOFF):
        assert floor[phase] == full[phase]
        assert delivery[phase].magnet_command == 'ON'
        assert delivery[phase].reference_type == full[phase].reference_type
        assert delivery[phase].target_source == full[phase].target_source
    for phase, spec in join.items():
        assert floor[phase] == spec
    for phase, spec in delivery.items():
        if phase not in (P.WAIT_FOR_TAKEOFF, P.TAKEOFF):
            assert spec == full[phase]


def test_mode_names_and_takeoff_exits():
    assert mission_for_mode('floor-join').name == 'floor_join'
    assert mission_for_mode(' Delivery-Rejoin ').name == 'delivery_rejoin'
    assert mission_for_mode('pickup_delivery').takeoff_exit == P.APPROACH_ABOVE_PICKUP
    assert mission_for_mode('floor_join').takeoff_exit == P.APPROACH_ABOVE_TARGET
    assert mission_for_mode('delivery_rejoin').takeoff_exit == P.TRANSIT_TO_DROP_POINT
    assert mission_for_mode('join').takeoff_exit is None
    with pytest.raises(ValueError):
        mission_for_mode('floorjoin')


def test_a_mission_with_takeoff_must_name_its_exit():
    full = mission_for_mode('pickup_delivery')
    with pytest.raises(ValueError):
        MissionDefinition(name='x', initial_phase=P.WAIT_FOR_TAKEOFF, phases=full.phases)
    with pytest.raises(ValueError):
        MissionDefinition(name='x', initial_phase=P.WAIT_FOR_TAKEOFF, phases=full.phases,
                          takeoff_exit=P.LIFT_PROOF)    # not one of its phases


def test_the_launch_choices_are_planner_modes():
    source = LAUNCH.read_text()
    choices = re.search(r"DeclareLaunchArgument\('mission_mode'.*?choices=\[([^\]]*)\]", source, re.S)
    names = re.findall(r"'([^']+)'", choices.group(1))
    assert set(names) == {'pickup_delivery', 'floor_join', 'delivery_rejoin'}
    for mode in names:
        assert mission_for_mode(mode, contact_attach=True).initial_phase == P.WAIT_FOR_TAKEOFF


# ---- the magnet ---------------------------------------------------------------------------

def _magnet_on_the_mat(mode):
    p = floor_planner(mode)
    sent = []
    for _ in range(2):
        tick(p, 1.1)
        p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    sent.append((p.phase, p.last_magnet_command))
    p.last_object_attached_time = p.clock.t
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF
    p.clock.t += 1.1
    p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    sent.append((p.phase, p.last_magnet_command))
    return sent, [m.data for m in p.magnet_command_publisher.messages]


def test_delivery_rejoin_holds_the_magnet_on_from_boot_through_takeoff():
    sent, published = _magnet_on_the_mat('delivery_rejoin')
    assert sent == [(P.WAIT_FOR_TAKEOFF, 'ON'), (P.TAKEOFF, 'ON')]
    assert published and set(published) == {'ON'}


@pytest.mark.parametrize('mode', ['pickup_delivery', 'floor_join'])
def test_the_other_floor_starts_hold_the_magnet_off(mode):
    sent, published = _magnet_on_the_mat(mode)
    assert sent == [(P.WAIT_FOR_TAKEOFF, 'OFF'), (P.TAKEOFF, 'OFF')]
    assert published and set(published) == {'OFF'}


def test_a_take_back_in_the_delivery_takeoff_sends_no_object_magnet_command():
    p = floor_planner('delivery_rejoin')
    command(p, 'TAKEOFF')
    p.publish_magnet_command('ON')
    before = len(p.magnet_command_publisher.messages)
    p.take_back_callback(SimpleNamespace(data=True))
    assert p.phase == P.TAKEN_BACK
    for _ in range(3):
        tick(p, 1.1)
        p.publish_magnet_command(p.pickup_magnet_command(p.current_phase_spec().magnet_command))
    assert len(p.magnet_command_publisher.messages) == before and p.last_magnet_command == 'ON'


# ---- TAKEOFF -> the next phase ------------------------------------------------------------

def test_pickup_delivery_leaves_takeoff_at_the_detection_height_as_before():
    p = floor_planner('pickup_delivery', attached=False)
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF and p.takeoff_climb_height is None
    assert p.drone_command_publisher.messages == []                    # no refusal: his pickup is ahead
    np.testing.assert_allclose(p.rate_profile().goal_tip_offset, [0.0, 0.0, 0.50])
    state(p, MAT + [0.0, 0.0, 0.11])
    assert p.phase == P.TAKEOFF
    state(p, MAT + [0.0, 0.0, 0.13])
    assert p.phase == P.APPROACH_ABOVE_PICKUP
    assert p.last_transition_reason == 'takeoff confirmed at 0.13 m above start'


def test_pickup_delivery_takeoff_height_reached_goes_to_the_pickup_as_before():
    p = floor_planner('pickup_delivery')
    command(p, 'TAKEOFF')
    p.clock.t += 10.0
    place(p, MAT + [0.0, 0.0, 0.50])
    p.update_state_machine()
    assert p.phase == P.APPROACH_ABOVE_PICKUP and p.last_transition_reason == 'Takeoff height reached'


@pytest.mark.parametrize('mode', ['floor_join', 'delivery_rejoin'])
def test_the_new_modes_do_not_leave_takeoff_at_the_detection_height(mode):
    p = floor_planner(mode)
    command(p, 'TAKEOFF')
    state(p, MAT + [0.0, 0.0, 0.30])
    assert p.phase == P.TAKEOFF


def test_floor_join_climbs_over_the_mat_to_the_approach_height_then_approaches():
    p = floor_planner('floor_join')
    command(p, 'TAKEOFF')
    approach_body_z = PLATE[2] + p.approach_offset_z + ROD
    assert p.takeoff_climb_height == pytest.approx(approach_body_z - MAT[2])
    climb(p, lambda q: q.phase != P.TAKEOFF)
    assert p.phase == P.APPROACH_ABOVE_TARGET and p.last_transition_reason == 'Takeoff height reached'
    np.testing.assert_allclose(p.drone_position[:2], MAT[:2], atol=1e-9)   # straight up
    assert abs(p.drone_position[2] - approach_body_z) < p.pickup_z_tolerance


def test_floor_join_climbs_at_least_the_takeoff_height_under_a_low_ring():
    p = floor_planner('floor_join')
    p.attachment_position = np.array([0.0, 0.225, -0.6])
    command(p, 'TAKEOFF')
    assert p.takeoff_climb_height == pytest.approx(0.50)


def test_delivery_rejoin_without_c1f2_goes_to_the_transit_once_the_loaded_climb_is_complete():
    p = floor_planner('delivery_rejoin')
    command(p, 'TAKEOFF')
    assert p.takeoff_climb_height == pytest.approx(ROD + LIFT)
    assert p.delivery_object_start_z == pytest.approx(TIP_ON_FLOOR_Z - PUCK_BELOW_TIP)
    climb(p, lambda q: q.phase != P.TAKEOFF)
    assert p.phase == P.TRANSIT_TO_DROP_POINT
    assert p.last_transition_reason == 'delivery_rejoin: loaded climb complete'
    assert p.pickup_object_position[2] - p.delivery_object_start_z >= p.lift_complete_object_clearance
    assert p.drone_position[2] == pytest.approx(MAT[2] + ROD + LIFT, abs=p.loaded_lift_z_tolerance)


def test_delivery_rejoin_waits_for_the_object_to_rise_before_the_transit():
    """The flag reads attached but the object's pose stays on the floor: the loaded climb is not complete."""
    p = floor_planner('delivery_rejoin')
    command(p, 'TAKEOFF')
    p.left_object = p.pickup_object_position + [0.0, 0.0, 0.05]
    climb(p, lambda q: q.rate_profile().complete(q.phase_elapsed())
          and abs(q.drone_position[2] - MAT[2] - ROD - LIFT) < 0.01)
    for _ in range(20):
        fly(p)
    assert p.phase == P.TAKEOFF
    p.left_object = None                                               # measured on the marker now
    place(p, p.drone_position)
    fly(p)
    assert p.phase == P.TRANSIT_TO_DROP_POINT


def test_delivery_rejoin_prepares_the_cpp_transit_in_the_loaded_climb_hold():
    """C1F.2 on (both launch paths): the climb end is latched and held, the hold settles, the C++ transit is prepared
    from it and only its certified handoff leaves TAKEOFF, as after LIFT_OBJECT."""
    p = floor_planner('delivery_rejoin', c1f2=True)
    p.drop_point_source, p.drop_offset, p.drop_approach_clearance = 'payload', np.zeros(3), 1.35
    p.payload_position, p.payload_yaw, p.payload_velocity = RING.copy(), 0.0, np.zeros(3)
    command(p, 'TAKEOFF')
    climb(p, lambda q: q.c1f2_prepare_active)
    hold = p.c1f2_prepare_hold_position.copy()
    assert p.phase == P.TAKEOFF and not p.c1f2_prepare_settled
    np.testing.assert_allclose(p.build_reference().positions[0], hold)   # held, not the climb profile
    for _ in range(5):
        fly(p)
    assert p.c1f2_prepare_settled and p.c1f2_prepare_phase_active()
    np.testing.assert_allclose(p.c1f2_transfer_target_state().position, RING + [0.0, 0.0, 1.35 + ROD])
    for _ in range(5):
        fly(p)
    assert p.phase == P.TAKEOFF                                        # no handoff from the backend yet
    # the backend's certified post-grant reference (_c1f2_try_finalize_authority_handoff)
    p.c1f2_handoff_ready = p.c1f2_authority_acknowledged = True
    p.c1f2_cached_cpp_reference = SimpleNamespace()
    fly(p)
    assert p.phase == P.TRANSIT_TO_DROP_POINT and p.c1f2_cpp_authority_active
    assert not p.c1f2_prepare_active
    assert 'loaded climb hold' in p.last_transition_reason


def test_without_the_handoff_the_transit_refuses_as_for_pickup_delivery():
    p = floor_planner('delivery_rejoin', c1f2=True)
    command(p, 'TAKEOFF')
    with pytest.raises(RuntimeError):
        p.transition_to(P.TRANSIT_TO_DROP_POINT, 'no handoff')


# ---- the object not attached --------------------------------------------------------------

@pytest.mark.parametrize('why', ['detached', 'stale flag', 'no object pose'])
def test_a_delivery_takeoff_without_the_object_is_refused_on_the_mat(why):
    p = floor_planner('delivery_rejoin', attached=why != 'detached')
    if why == 'stale flag':
        p.last_object_attached_time = p.clock.t - 1.0
    if why == 'no object pose':
        p.last_pickup_object_pose_time = p.clock.t - 1.0
    command(p, 'TAKEOFF')
    assert p.phase == P.LANDED_DISARMED
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']
    assert p.landing_disarm_sent and p.takeoff_start_position is None
    assert p.last_transition_reason.startswith('TAKEOFF refused on the mat')


def test_the_refusal_is_announced_in_wait_for_takeoff():
    p = floor_planner('delivery_rejoin', attached=False)
    tick(p, 0.1)
    assert p.phase == P.WAIT_FOR_TAKEOFF
    assert any('a TAKEOFF now would be refused: the object does not read attached' in w for w in p.warnings)
    q = floor_planner('delivery_rejoin')
    tick(q, 0.1)
    assert not any('refused' in w for w in q.warnings)


def test_floor_join_without_a_fresh_attachment_pose_is_refused_on_the_mat():
    p = floor_planner('floor_join', attached=False)
    p.last_attachment_pose_time = p.clock.t - 1.0
    command(p, 'TAKEOFF')
    assert p.phase == P.LANDED_DISARMED
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']


def test_floor_join_needs_no_object():
    p = floor_planner('floor_join', attached=False)
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF and p.drone_command_publisher.messages == []


@pytest.mark.parametrize('c1f2', [False, True])
def test_the_object_lost_in_the_delivery_climb_lands_over_the_mat(c1f2):
    p = floor_planner('delivery_rejoin', c1f2=c1f2)
    command(p, 'TAKEOFF')
    if c1f2:
        climb(p, lambda q: q.c1f2_prepare_active)
    else:
        for _ in range(20):
            fly(p)
    p.object_attached = False
    fly(p)
    assert p.phase == P.LANDING
    assert 'no longer reads attached' in p.last_transition_reason
    assert not p.c1f2_prepare_active and not p.c1f2_handoff_ready
    np.testing.assert_allclose(p.landing_target_xy, MAT[:2], atol=1e-9)


# ---- after the takeoff, delivery_rejoin is pickup_delivery ---------------------------------

def test_the_delivery_drop_check_is_pickup_deliverys():
    away = _dropping([0.0, 0.0, -0.6])
    away.mission = mission_for_mode('delivery_rejoin', contact_attach=True)
    away.clock.t += 0.5
    tick(away)
    assert away.phase == P.CLEAR_DROP_ZONE
    stuck = _dropping([0.0, 0.0, -0.04])
    stuck.mission = mission_for_mode('delivery_rejoin', contact_attach=True)
    stuck.clock.t += 0.5
    tick(stuck)
    assert stuck.phase == P.TAKEN_BACK and stuck.contact_gave_up


