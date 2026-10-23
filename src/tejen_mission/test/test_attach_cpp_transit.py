"""attach_cpp_transit (Wesley 21 Oct 2026): floor_join's leg to the plate on his C++ planner, so his collision
avoidance keeps QUAD4 and its tether off the carriers (RIG-1015-a1: the Python transfer swept the tether through
QUAD2). The climb ends at the transit height, the climb hold prepares the C++ transit as LIFT_OBJECT's does, the C++
planner flies TRANSIT_TO_REATTACH to above the plate and his approach (Python) takes over in its capture region.
The launch puts the transit height half the carriers' cable above them (carry_heights). Driven without ROS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.ring_bridge import RingBridge
from test_floor_join_delivery_rejoin import MAT, ROD, climb, command, floor_planner, fly, place
from test_m1_contact_attach import PLATE, tick

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'
CLEARANCE = 0.85                    # tip above the plate in the C++ transit (the launch computes it)


def cpp_floor_join():
    p = floor_planner('floor_join', c1f2=True)
    p.mission = mission_for_mode('floor_join', contact_attach=True, attach_cpp_transit=True)
    p.attach_cpp_transit = p.attach_cpp_transit_active = True
    p.attach_transit_clearance = CLEARANCE
    p.attachment_velocity = np.zeros(3)
    p.last_attachment_twist_time = p.clock.t
    p.c1f2_exit_switch_count = 0
    p.c1f2_cached_continuation_active = False
    p.c1f2_cached_cpp_reference_receive_time = None
    p.c1f2_nav_capture_xy_threshold_m, p.c1f2_nav_capture_z_threshold_m = 0.12, 0.15   # c1f2b_cpp_authority.yaml
    del p.tracking_errors                                 # the real errors: from the measured tip to the target
    return p


def fresh_attachment(p):
    p.last_attachment_pose_time = p.last_attachment_twist_time = p.clock.t


def transit_body_target(p):
    return PLATE + [p.approach_offset_x, p.approach_offset_y, CLEARANCE + ROD]


def test_the_mission_leaves_takeoff_for_the_cpp_transit_only_when_switched_on():
    on = mission_for_mode('floor_join', contact_attach=True, attach_cpp_transit=True)
    off = mission_for_mode('floor_join', contact_attach=True)
    assert on.takeoff_exit == P.TRANSIT_TO_REATTACH and P.TRANSIT_TO_REATTACH in on.phases
    assert off.takeoff_exit == P.APPROACH_ABOVE_TARGET and P.TRANSIT_TO_REATTACH not in off.phases
    # the delivery's rejoin is unchanged: after the drop the vehicle is over the ring's centre
    assert mission_for_mode('pickup_delivery', contact_attach=True, attach_cpp_transit=True).takeoff_exit == \
        mission_for_mode('pickup_delivery', contact_attach=True).takeoff_exit


def test_the_climb_ends_at_the_transit_height_and_prepares_the_cpp_transit_there():
    p = cpp_floor_join()
    command(p, 'TAKEOFF')
    assert p.takeoff_climb_height == pytest.approx(PLATE[2] + CLEARANCE + ROD - MAT[2])
    climb(p, lambda q: q.c1f2_prepare_active)
    hold = p.c1f2_prepare_hold_position.copy()
    assert p.phase == P.TAKEOFF
    np.testing.assert_allclose(p.drone_position[:2], MAT[:2], atol=1e-9)     # straight up over the mat
    np.testing.assert_allclose(p.build_reference().positions[0], hold)       # held, not the climb profile
    for _ in range(5):
        fresh_attachment(p)
        fly(p)
    assert p.c1f2_prepare_settled and p.c1f2_prepare_phase_active() and p.c1f2_cpp_phase_active()
    np.testing.assert_allclose(p.c1f2_transfer_target_state().position, transit_body_target(p))
    assert p.c1f2_drop_target_inputs_ready()
    p.last_attachment_twist_time = p.clock.t - 1.0                            # a stale ring twist: no target
    assert not p.c1f2_drop_target_inputs_ready()
    fresh_attachment(p)
    for _ in range(5):
        fly(p)
    assert p.phase == P.TAKEOFF                                               # no handoff from the backend yet
    p.c1f2_handoff_ready = p.c1f2_authority_acknowledged = True
    p.c1f2_cached_cpp_reference = SimpleNamespace()
    fly(p)
    assert p.phase == P.TRANSIT_TO_REATTACH and p.c1f2_cpp_authority_active
    assert p.c1f2_cpp_transit_authority_active() and not p.c1f2_prepare_active
    assert 'climb hold' in p.last_transition_reason


def test_without_the_handoff_the_cpp_transit_refuses():
    p = cpp_floor_join()
    command(p, 'TAKEOFF')
    with pytest.raises(RuntimeError):
        p.transition_to(P.TRANSIT_TO_REATTACH, 'no handoff')


def to_transit(p):
    command(p, 'TAKEOFF')
    climb(p, lambda q: q.c1f2_prepare_active)
    for _ in range(10):
        fresh_attachment(p)
        fly(p)
    p.c1f2_handoff_ready = p.c1f2_authority_acknowledged = True
    p.c1f2_cached_cpp_reference = SimpleNamespace()
    fly(p)
    assert p.phase == P.TRANSIT_TO_REATTACH


def test_the_transit_hands_back_to_his_approach_in_its_capture_region():
    p = cpp_floor_join()
    to_transit(p)
    np.testing.assert_allclose(p.desired_tip_offset(), [p.approach_offset_x, p.approach_offset_y, CLEARANCE])
    np.testing.assert_allclose(p.c1f2_transfer_target_state().position, transit_body_target(p))
    for _ in range(10):                                   # still on the way: C++ keeps it
        fresh_attachment(p)
        tick(p, 0.1)
    assert p.phase == P.TRANSIT_TO_REATTACH
    place(p, transit_body_target(p) + [0.05, 0.0, 0.05])  # inside 0.12 m / 0.15 m
    for _ in range(7):
        fresh_attachment(p)
        place(p, transit_body_target(p) + [0.05, 0.0, 0.05])
        tick(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET
    assert not p.c1f2_cpp_authority_active and not p.c1f2_cpp_transit_authority_active()
    assert p.c1f2_authority_publisher.messages[-1].data is False
    assert p.c1f2_cached_cpp_reference is None


def test_landing_from_the_transit_releases_the_cpp_authority():
    p = cpp_floor_join()
    to_transit(p)
    p.transition_to(P.LANDING, 'LAND')
    assert p.phase == P.LANDING and not p.c1f2_cpp_authority_active


def test_without_the_switch_transit_to_reattach_is_not_a_cpp_phase():
    p = floor_planner('pickup_delivery', c1f2=True)
    p.attach_cpp_transit, p.attach_cpp_transit_active = True, False   # the switch is floor_join's only
    assert P.TRANSIT_TO_REATTACH not in p.c1f2_cpp_transit_phases()
    p.phase = P.TRANSIT_TO_REATTACH
    assert not p.c1f2_cpp_phase_active()


# ---- the launch's transit heights and his planner's carrier model --------------------------

def launch_module():
    spec = importlib.util.spec_from_file_location('partner_mission_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('geometry, rod, attach_z', [('rig_twin', 0.59, 0.032), ('rig', 0.575, 0.03)])
def test_the_carried_object_and_the_magnet_pass_half_a_cable_above_the_carriers(geometry, rod, attach_z):
    m = launch_module()
    G = m.GEOMETRY[geometry]
    H = m.carry_heights(G, ring_z=1.0, rod=rod, attach_z=attach_z)
    carriers = 1.0 + 0.55 * np.sin(np.radians(60.0)) + 0.04
    tip = G['object_rest_z'] + H['pickup_lift_height']
    assert tip - G['object_bottom_below_tip'] == pytest.approx(carriers + 0.275)              # the object's bottom
    assert 1.0 + m.DROP_OFFSET_Z + H['drop_approach_clearance'] == pytest.approx(tip)          # one height to the ring
    assert 1.0 + attach_z + H['attach_transit_clearance'] - m.MAGNET_BOTTOM_BELOW_TIP == \
        pytest.approx(carriers + 0.275)                                                       # the magnet's bottom
    assert H['minimum_search_z_m'] == pytest.approx(tip + rod - 0.05)                         # the body just under
    assert m.carry_heights(G, ring_z=1.0, rod=rod, attach_z=attach_z, clearance=0.4)['pickup_lift_height'] == \
        pytest.approx(H['pickup_lift_height'] + 0.4 - 0.275)


def test_the_backend_gets_the_carriers_rods_and_quad4s_own_cable():
    m = launch_module()
    files = m._backend_overrides('base.yaml', m.GEOMETRY['rig_twin'], rod=0.59, floor_join=True,
                                 carrier_model='tether').split(',')
    assert files[0] == 'base.yaml'
    params = yaml.safe_load(Path(files[1]).read_text())['dynamic_planner_transfer_backend']['ros__parameters']
    assert params['suspended_cable_length_m'] == 0.59
    assert params['require_payload_attached_for_enable'] is False               # floor_join carries nothing
    assert params['cooperative_attached_tether_enabled'] is True
    assert params['cooperative_attached_plate_topics'] == [f'/fake_obstacles/drone_{i}/attached_plate'
                                                           for i in range(3)]
    assert params['cooperative_ring_plate_pitch_diameter_m'] == 0.45
    assert params['cooperative_body_half_z_m'] == 0.06                         # his box, not the tall column
    column = m._backend_overrides('base.yaml', m.GEOMETRY['rig_twin'], rod=0.59, floor_join=False,
                                  carrier_model='column').split(',')[1]
    params = yaml.safe_load(Path(column).read_text())['dynamic_planner_transfer_backend']['ros__parameters']
    assert 'cooperative_attached_tether_enabled' not in params and params['require_payload_attached_for_enable']


def test_ring_bridge_numbers_the_carriers_plates_as_his_planner_does():
    stub = SimpleNamespace(_carrier_azimuths=lambda: [270.0, 30.0, 150.0])   # rig_twin, counter-clockwise
    assert RingBridge.carrier_plates(stub) == [9, 1, 5]
