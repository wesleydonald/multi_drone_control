"""return_after_drop (12 Nov 2026, the supervisor's drop-off: start -> ring -> start, no re-attach): once the drop zone
is clear his planner flies QUAD4 back to its start hover (TAKEOFF's start xy at the clearance height) and holds it there
(AT_START) until his LAND. 'cpp': on his C++ planner after a fresh hand-over in the drop-zone hold, as
pickup_cpp_reattach's; 'direct': on his Python transfer. Off by default. Driven without ROS."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase as P
from test_floor_join_delivery_rejoin import floor_planner, place

START = np.array([0.0, 1.9, 0.06])


def returning(mode='cpp'):
    p = floor_planner('delivery_rejoin', c1f2=True)
    p.mission = mission_for_mode('delivery_rejoin', contact_attach=True, return_to_start=bool(mode))
    p.attach_cpp_transit_active = p.pickup_cpp_reattach_active = False
    p.pickup_reattach_prepare_started = False
    p.end_after_drop = False
    p.return_after_drop = mode
    p.return_prepare_started = False
    p.return_hover = None
    p.c1f2_exit_switch_count = 0
    p.c1f2_cached_continuation_active = False
    p.c1f2_cached_cpp_reference_receive_time = None
    p.c1f2_nav_capture_xy_threshold_m, p.c1f2_nav_capture_z_threshold_m = 0.12, 0.15
    p.takeoff_start_position = START.copy()
    return p


def test_both_missions_carry_the_two_phases_only_when_asked_with_the_magnet_off():
    for mode in ('delivery_rejoin', 'pickup_delivery'):
        assert P.RETURN_TO_START not in mission_for_mode(mode, contact_attach=True).phases
        mission = mission_for_mode(mode, contact_attach=True, return_to_start=True)
        for phase in (P.RETURN_TO_START, P.AT_START):
            assert mission.phases[phase].magnet_command == 'OFF'


def test_off_by_default_nothing_changes():
    p = returning(mode='')
    assert P.RETURN_TO_START not in p.c1f2_cpp_transit_phases()
    p.phase = P.CLEAR_DROP_ZONE
    p.return_prepare_started = True
    assert not p.return_prepare_phase() and not p.return_context()


def test_cpp_the_return_is_a_cpp_phase_and_the_drop_zone_hold_prepares_it():
    p = returning()
    assert P.RETURN_TO_START in p.c1f2_cpp_transit_phases()
    p.phase = P.CLEAR_DROP_ZONE
    assert not p.c1f2_prepare_phase()            # still climbing out of the drop zone: no hand-over yet
    place(p, [0.1, 0.2, 2.9])
    p.latch_return_hover()
    p.return_prepare_started = True
    assert p.c1f2_prepare_phase() and p.return_context()
    assert p.c1f2_drop_target_inputs_ready()      # the start hover is static: no ring pose needed
    target = p.c1f2_transfer_target_state()
    np.testing.assert_allclose(target.position, [START[0], START[1], 2.9])
    np.testing.assert_allclose(target.velocity, 0.0)


def test_cpp_without_a_fresh_handoff_the_return_refuses():
    p = returning()
    p.phase = P.CLEAR_DROP_ZONE
    p.return_prepare_started = True
    with pytest.raises(RuntimeError):
        p.transition_to(P.RETURN_TO_START, 'no handoff')


def test_cpp_the_prepared_handoff_leaves_the_drop_zone_for_the_cpp_return():
    p = returning()
    p.phase = P.CLEAR_DROP_ZONE
    place(p, [0.1, 0.2, 2.9])
    p.latch_return_hover()
    p.return_prepare_started = True
    p.c1f2_prepare_active = p.c1f2_prepare_settled = True
    p.c1f2_handoff_ready = p.c1f2_authority_acknowledged = True
    p.c1f2_cached_cpp_reference = SimpleNamespace()
    p.c1f2_climb_hold_handoff(True, 'drop-zone clearance', 'the drop-zone clearance hold',
                              exit_phase=P.RETURN_TO_START)
    assert p.phase == P.RETURN_TO_START and p.c1f2_cpp_authority_active


def test_direct_is_not_a_cpp_phase_and_its_reference_flies_to_the_start_hover():
    p = returning('direct')
    assert P.RETURN_TO_START not in p.c1f2_cpp_transit_phases()
    place(p, [0.1, 0.2, 2.9])
    p.latch_return_hover()
    p.transition_to(P.RETURN_TO_START, 'direct return')   # no C++ hand-over needed
    calls = []
    p.transfer_generator = SimpleNamespace(generate=lambda **kw: calls.append(kw) or 'reference')
    assert p.build_reference() == 'reference'
    assert calls[0]['phase_key'] == 'RETURN_TO_START'
    np.testing.assert_allclose(calls[0]['target'].position, [START[0], START[1], 2.9])
    np.testing.assert_allclose(calls[0]['target'].velocity, 0.0)


def test_the_return_ends_at_the_start_hover_on_the_capture_rule():
    p = returning('direct')
    place(p, [0.1, 0.2, 2.9])
    p.latch_return_hover()
    p.phase = P.RETURN_TO_START
    place(p, [START[0] + 0.05, START[1] - 0.05, 2.95])
    p.condition_start_time = None
    p.update_state_machine()
    assert p.phase == P.RETURN_TO_START           # inside the capture region, not yet for 0.5 s
    p.clock.t += 0.6
    p.update_state_machine()
    assert p.phase == P.AT_START


def test_his_land_lands_from_the_start_hover():
    p = returning('direct')
    place(p, START + [0.0, 0.0, 2.84])
    p.latch_return_hover()
    p.phase = P.AT_START
    p.external_landing_callback(SimpleNamespace(data=True))
    assert p.phase == P.LANDING


# ---- the launch -----------------------------------------------------------------------------------------------------

def _sim(**overrides):
    """His planner's parameters on the twin path, with the typed set recorded as the launch records it."""
    from launch_ros.utilities import evaluate_parameters
    from test_partner_mission_geometry import _context
    typed = {'mission_mode': 'delivery_rejoin', 'minimum_search_z_m': '2.20', **overrides}
    mod, ctx = _context('rig_twin_L100', **typed)
    ctx.launch_configurations[mod.TYPED_KEY] = ','.join(typed)
    for node in mod._nodes(ctx):
        if getattr(node, '_Node__node_name', None) == 'online_join_planner':
            out = {}
            for p in evaluate_parameters(ctx, node._Node__parameters):
                if isinstance(p, dict):
                    out.update(p)
            return out
    raise AssertionError('no online_join_planner node')


def _rig_planner(**typed):
    from test_partner_mission_rig import _nodes
    typed = {'minimum_search_z_m': '2.20', 'magnet_drop_below_quad': '1.03', 'ring_cable_len': '1.0', 'ring_z': '1.2',
             'geometry': 'rig_m2a', 'max_body_z': '2.8', **typed}     # tomorrow's 1 m cables, search floor, ceiling
    return _nodes(mission_mode='delivery_rejoin', **typed)[0]['online_join_planner']['params']


def test_launch_typed_only_on_both_paths():
    assert 'return_after_drop' not in _sim()
    assert 'return_after_drop' not in _rig_planner()
    assert _sim(return_after_drop='cpp')['return_after_drop'] == 'cpp'
    assert _rig_planner(return_after_drop='direct')['return_after_drop'] == 'direct'


def test_launch_the_return_keeps_the_drop_zone_climb_and_the_rig_ceiling():
    """The rig's 2.8 m ceiling refuses a climb back to the transit height (2.96 m with the unload overshoot): the return
    leaves from the drop zone's usual climb, as without it."""
    assert _sim(return_after_drop='cpp')['drop_clear_height'] == pytest.approx(_sim()['drop_clear_height'])
    rig = _rig_planner(return_after_drop='cpp')
    assert rig['drop_clear_height'] == pytest.approx(_rig_planner()['drop_clear_height'])


def test_launch_refuses_a_search_floor_above_the_return_hand_over():
    with pytest.raises(RuntimeError, match='return hand-over'):
        _rig_planner(return_after_drop='cpp', minimum_search_z_m='2.60')


def test_launch_refuses_with_end_after_drop_and_cpp_without_his_planner():
    with pytest.raises(RuntimeError, match='exclude'):
        _rig_planner(return_after_drop='cpp', end_after_drop='true')
    with pytest.raises(RuntimeError, match='needs his C\\+\\+'):
        _rig_planner(return_after_drop='cpp', cpp_authority='false')


def test_launch_cpp_gives_his_backend_the_static_target_topic():
    """Codex review 13 Nov (Te-Jen's side): his committed_trajectory predictor moves any goal with the basket; the C++
    return's goal is a fixed point, so his backend predicts it still while the planner says so (cpp only)."""
    from test_attach_cpp_transit import _margins
    assert 'static_target_topic' not in _margins()
    assert 'static_target_topic' not in _margins(return_after_drop='direct')
    assert _margins(return_after_drop='cpp')['static_target_topic'] == '/dynamic_planner/static_target'


def test_launch_cpp_max_z_caps_his_search_box_only_when_typed():
    """13 Nov: his search box reached 1 m above start/goal (3.2-3.6 m flown in the twin); typed, its top is capped."""
    from test_attach_cpp_transit import _margins
    assert 'maximum_search_z_m' not in _margins()
    assert _margins(cpp_max_z='2.9')['maximum_search_z_m'] == 2.9


def test_launch_temporal_refinement_only_when_typed():
    from test_attach_cpp_transit import _margins
    assert 'temporal_refinement_enabled' not in _margins()
    assert _margins(temporal_refinement='true')['temporal_refinement_enabled'] is True


def test_launch_cpp_world_refresh_only_when_typed():
    from test_attach_cpp_transit import _margins
    assert 'cooperative_world_refresh_period_s' not in _margins()
    assert _margins(cpp_world_refresh='0.2')['cooperative_world_refresh_period_s'] == 0.2


def test_launch_cpp_splice_only_when_typed():
    from test_attach_cpp_transit import _margins
    assert 'fixed_splice_lookahead_s' not in _margins()
    assert _margins(cpp_splice='0.15')['fixed_splice_lookahead_s'] == 0.15
