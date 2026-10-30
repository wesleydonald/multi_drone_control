"""pickup_cpp_reattach (Wesley 28 Oct 2026: "we need to fix the avoidance"; card 2026-10-28_attach_ambitious): after
the drop, pickup_delivery's flight to the plate goes on his C++ planner as floor_join's does, so his collision avoidance
keeps QUAD4 off the carriers; the hand-over is prepared in a hold at the end of CLEAR_DROP_ZONE (the drop leg's is
spent). Off by default: TRANSIT_TO_REATTACH stays his Python transfer. Driven without ROS."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase as P
from test_attach_cpp_transit import CLEARANCE, fresh_attachment, transit_body_target
from test_floor_join_delivery_rejoin import floor_planner


def pickup(on=True):
    p = floor_planner('pickup_delivery', c1f2=True)
    p.mission = mission_for_mode('pickup_delivery', contact_attach=True)
    p.attach_cpp_transit_active = False
    p.pickup_cpp_reattach = p.pickup_cpp_reattach_active = on
    p.pickup_reattach_prepare_started = False
    p.end_after_drop = False
    p.attach_transit_clearance = CLEARANCE
    p.attachment_velocity = np.zeros(3)
    p.last_attachment_twist_time = p.clock.t
    p.c1f2_exit_switch_count = 0
    p.c1f2_cached_continuation_active = False
    p.c1f2_cached_cpp_reference_receive_time = None
    return p


def test_off_by_default_the_reattach_stays_his_python_transfer():
    p = pickup(on=False)
    assert not p.cpp_reattach_active()
    assert P.TRANSIT_TO_REATTACH not in p.c1f2_cpp_transit_phases()
    p.phase = P.CLEAR_DROP_ZONE
    p.pickup_reattach_prepare_started = True
    assert not p.pickup_reattach_prepare_phase() and not p.c1f2_prepare_phase()


def test_on_the_reattach_is_a_cpp_phase_and_the_drop_zone_hold_prepares_it():
    p = pickup()
    assert p.cpp_reattach_active() and P.TRANSIT_TO_REATTACH in p.c1f2_cpp_transit_phases()
    p.phase = P.CLEAR_DROP_ZONE
    assert not p.c1f2_prepare_phase()                 # still climbing out of the drop zone: no hand-over yet
    p.pickup_reattach_prepare_started = True
    assert p.c1f2_prepare_phase() and p.c1f2_attach_transit_context()
    fresh_attachment(p)
    np.testing.assert_allclose(p.c1f2_transfer_target_state().position, transit_body_target(p))


def test_without_a_fresh_handoff_the_cpp_reattach_refuses():
    p = pickup()
    p.phase = P.CLEAR_DROP_ZONE
    p.pickup_reattach_prepare_started = True
    with pytest.raises(RuntimeError):
        p.transition_to(P.TRANSIT_TO_REATTACH, 'no handoff')


def test_the_prepared_handoff_leaves_the_drop_zone_for_the_cpp_reattach():
    p = pickup()
    p.phase = P.CLEAR_DROP_ZONE
    p.pickup_reattach_prepare_started = True
    p.c1f2_prepare_active = p.c1f2_prepare_settled = True
    p.c1f2_handoff_ready = p.c1f2_authority_acknowledged = True
    p.c1f2_cached_cpp_reference = SimpleNamespace()
    p.c1f2_climb_hold_handoff(True, 'drop-zone clearance', 'the drop-zone clearance hold',
                              exit_phase=P.TRANSIT_TO_REATTACH)
    assert p.phase == P.TRANSIT_TO_REATTACH and p.c1f2_cpp_authority_active
    np.testing.assert_allclose(p.desired_tip_offset(), [p.approach_offset_x, p.approach_offset_y, CLEARANCE])


def test_the_launch_mode_turns_it_on_though_the_mission_is_named_pickup_delivery_reattach():
    """R2855 and R2863: the activation tested the mission's name, which is pickup_delivery_reattach, so it never came
    on; the launch mode decides."""
    from tejen_mission.online_join_planner import pickup_cpp_reattach_applies
    assert mission_for_mode('pickup_delivery', contact_attach=True).name != 'pickup_delivery'
    assert pickup_cpp_reattach_applies('pickup_delivery', True)
    assert not pickup_cpp_reattach_applies('pickup_delivery', False)
    assert not pickup_cpp_reattach_applies('floor_join', True)


def test_floor_join_keeps_its_takeoff_exit():
    p = floor_planner('floor_join', c1f2=True)
    p.attach_cpp_transit_active = False
    p.pickup_cpp_reattach_active = False
    assert not p.cpp_reattach_active()


# ---- the launch (sim path) ----------------------------------------------------------------------------------------

def _launch_params(**overrides):
    from test_partner_mission_geometry import _params
    return _params('rig_twin_L100_p115', mission_mode='pickup_delivery', **overrides)


def _planner(params):
    return next(v for k, v in params.items() if 'join_planner' in k or 'online_join' in k)


def test_launch_off_by_default_passes_nothing_new():
    planner = _planner(_launch_params(transit='over'))
    assert planner['pickup_cpp_reattach'] is False and 'drop_clear_height' not in planner
    # the low route (the default since 29 Oct) climbs only 3 cm after the drop (R2898/R2899's unload overshoot)
    planner = _planner(_launch_params())
    assert planner['pickup_cpp_reattach'] is False
    assert planner['drop_clear_height'] == pytest.approx(0.17)


def test_launch_on_puts_the_handover_and_the_transit_end_above_his_search_floor():
    p = _launch_params(pickup_cpp_reattach='true', transit='over')
    planner = _planner(p)
    assert planner['pickup_cpp_reattach'] is True
    assert planner['drop_clear_height'] == pytest.approx(planner['drop_approach_clearance'])
    assert planner['attach_transit_clearance'] > 0.85     # raised to the loaded leg's floor
