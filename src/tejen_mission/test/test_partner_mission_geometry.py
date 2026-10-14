"""partner_mission.launch.py (ours): geometry:=legacy keeps every old-world value; geometry:=rig_twin gives his
mission the rig-twin attach world (the combined M1 subtest, Wesley 10 Oct)."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'


def _module():
    spec = importlib.util.spec_from_file_location('partner_mission_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _context(geometry, **overrides):
    mod = _module()
    ctx = LaunchContext()
    for a in mod._arguments():
        if isinstance(a, DeclareLaunchArgument):
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    if geometry is not None:
        ctx.launch_configurations['geometry'] = geometry
    ctx.launch_configurations.update(overrides)
    return mod, ctx


def _actions(geometry, **overrides):
    mod, ctx = _context(geometry, **overrides)
    return mod._nodes(ctx)


def _params(geometry, **overrides):
    """Each node's parameters, evaluated as the launch would pass them."""
    mod, ctx = _context(geometry, **overrides)
    out = {}
    for node in mod._nodes(ctx):
        name = getattr(node, '_Node__node_name', None)
        params = getattr(node, '_Node__parameters', None)
        if not name or not params:
            continue
        for p in evaluate_parameters(ctx, params):
            if isinstance(p, dict):
                out.setdefault(name, {}).update(
                    {k: list(v) if isinstance(v, tuple) else v for k, v in p.items()})
    return out


def test_default_is_the_old_world():
    p = _params(None)
    ring, mpc = p['fake_cooperative_transport_world'], p['tejen_mpc']
    assert (ring['bridge_attach_radius_m'], ring['bridge_cable_len_m'], ring['bridge_cable_elev_deg']) == (0.25, 0.5, 45.0)
    assert (ring['ring_plate_pitch_diameter_m'], ring['ring_collision_outer_diameter_m']) == (0.50, 0.56)
    assert p['online_join_planner']['magnet_drop_below_quad'] == 0.49
    assert (mpc['thrust_ratio'], mpc['enable_thrust_ratio_feedback'], mpc['cable_length'],
            mpc['object_vehicle_mass_kg']) == (82.7, False, 0.49, 0.643)


def test_rig_twin_values():
    p = _params('rig_twin')
    ring, mpc = p['fake_cooperative_transport_world'], p['tejen_mpc']
    assert (ring['bridge_attach_radius_m'], ring['bridge_cable_len_m'], ring['bridge_cable_elev_deg']) == (0.225, 0.55, 60.0)
    assert (ring['ring_plate_pitch_diameter_m'], ring['ring_collision_outer_diameter_m']) == (0.45, 0.51)
    assert p['online_join_planner']['magnet_drop_below_quad'] == 0.59
    assert (mpc['thrust_ratio'], mpc['enable_thrust_ratio_feedback'], mpc['cable_length'],
            mpc['object_vehicle_mass_kg']) == (22.0, True, 0.55, 0.55)


def test_unknown_geometry_is_refused():
    with pytest.raises(RuntimeError, match='geometry'):
        _params('rig')


# ---- the geometry owns his plate and carriers; the contact attach (combined M1, 10 Oct late) ----

CONTACT_KEYS = {'contact_attach', 'full_pose_attachment', 'bridge_attach_z_m', 'pickup_release_state_topic',
                'pickup_retry_enabled', 'pickup_retry_clearance_m', 'pickup_hold_on_attach'}


def _gz_release(actions):
    """The one-shot `gz topic -t /pickup/detach` publish."""
    def words(action):
        return [''.join(getattr(t, 'text', '') for t in part) for part in action.cmd]
    return [a for a in actions if hasattr(a, 'cmd') and not hasattr(a, '_Node__node_name')
            and words(a)[:3] == ['gz', 'topic', '-t'] and '/pickup/detach' in words(a)]


def _node_args(actions, name):
    node = next(a for a in actions if getattr(a, '_Node__node_name', None) == name)
    return [''.join(getattr(t, 'text', str(t)) for t in arg) if not isinstance(arg, str) else arg
            for arg in node._Node__arguments]


def test_the_geometry_owns_the_plate_and_carriers():
    legacy, rig = _params(None)['fake_cooperative_transport_world'], _params('rig_twin')['fake_cooperative_transport_world']
    assert (legacy['ring_attachment_plate_index'], legacy['bridge_carrier_azimuths_deg']) == (9, [330.0, 90.0, 210.0])
    assert (rig['ring_attachment_plate_index'], rig['bridge_carrier_azimuths_deg']) == (3, [270.0, 30.0, 150.0])
    # typed: our plate 3 (his 9), clockwise degrees (his 150/270/30 counter-clockwise)
    explicit = _params('rig_twin', plate='3', carrier_azimuths_deg='[210.0, 90.0, 330.0]')
    ring = explicit['fake_cooperative_transport_world']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (9, [150.0, 270.0, 30.0])


@pytest.mark.parametrize('geometry', [None, 'rig_twin'])
def test_typed_plate_and_carriers_are_ours_and_clockwise(geometry):
    """Wesley 14 Oct: plate is our plate number (clockwise from +x, 9 = +y) and carrier_azimuths_deg clockwise degrees;
    his ring bridge gets his counter-clockwise plate index, (12 - p) % 12, and azimuths. Typing the geometry's own
    values changes nothing."""
    ring = _params('rig_twin', plate='9', carrier_azimuths_deg='[90.0, 330.0, 210.0]')['fake_cooperative_transport_world']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (3, [270.0, 30.0, 150.0])
    own = {None: ('3', '[30.0, 270.0, 150.0]'), 'rig_twin': ('9', '[90.0, 330.0, 210.0]')}[geometry]
    assert _params(geometry, plate=own[0], carrier_azimuths_deg=own[1]) == _params(geometry)
    ring = _params(geometry, plate='0', carrier_azimuths_deg='[0.0, 120.0, 240.0]')['fake_cooperative_transport_world']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (0, [0.0, 240.0, 120.0])


@pytest.mark.parametrize('geometry', [None, 'rig_twin'])
def test_defaults_add_no_contact_attach_and_keep_the_one_shot_release(geometry):
    p = _params(geometry)
    assert not any(CONTACT_KEYS & set(params) for params in p.values())
    planner = p['online_join_planner']
    assert (planner['attach_ready_target_lead_time'], planner['match_xy_threshold']) == (0.30, 0.18)
    actions = _actions(geometry)
    assert len(_gz_release(actions)) == 1
    assert 'pickup_state_bridge' not in p


def test_contact_attach_on_the_rig_twin():
    p = _params('rig_twin', contact_attach='true')
    planner, ring = p['online_join_planner'], p['fake_cooperative_transport_world']
    assert (planner['contact_attach'], planner['final_attach_offset_z'], planner['attach_ready_target_lead_time']) \
        == (True, 0.0, 0.0)
    assert (ring['bridge_attach_z_m'], ring['full_pose_attachment']) == (0.032, True)
    # the rest of his mission is untouched (his drop gates included)
    base = _params('rig_twin')['online_join_planner']
    changed = {k for k in set(planner) | set(base) if planner.get(k) != base.get(k)}
    assert changed == {'contact_attach', 'final_attach_offset_z', 'attach_ready_target_lead_time', 'approach_offset_x',
                       'approach_offset_y', 'pickup_retry_enabled', 'pickup_retry_clearance_m', 'pickup_hold_on_attach'}
    assert planner['approach_offset_x'] == planner['approach_offset_y'] == 0.0
    # his bounded pickup retry (R1493, R1497): his wait and attempt budget, a ~0.10 m rise
    assert (planner['pickup_retry_enabled'], planner['pickup_retry_clearance_m']) == (True, 0.10)
    # his pickup descent ends on the ball weld (R1507), here where his object flag is the weld itself
    assert planner['pickup_hold_on_attach'] is True
    assert not {'pickup_retry_wait_s', 'pickup_retry_max_attempts', 'pickup_alignment_mode'} & set(planner)
    # the card's neighbours: one value each
    n = _params('rig_twin', contact_attach='true', contact_lead='0.3', contact_dz='-0.01', contact_dx='0.006')
    n = n['online_join_planner']
    assert (n['attach_ready_target_lead_time'], n['final_attach_offset_z'], n['approach_offset_x']) == (0.3, -0.01, 0.006)


def test_contact_attach_needs_a_contact_target():
    with pytest.raises(RuntimeError, match='contact'):
        _params(None, contact_attach='true')


def test_confirmed_pickup_release_bridges_the_joint_state_and_gates_the_arm():
    actions = _actions('rig_twin', confirm_pickup_release='true')
    assert not _gz_release(actions)
    assert _node_args(actions, 'pickup_state_bridge') == [
        '/pickup/detachable_joint_state@std_msgs/msg/String[gz.msgs.StringMsg']
    supervisor = _params('rig_twin', confirm_pickup_release='true')['simulation_test_supervisor']
    assert supervisor['pickup_release_state_topic'] == '/pickup/detachable_joint_state'
