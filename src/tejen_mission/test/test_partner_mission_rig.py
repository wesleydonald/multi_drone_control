"""partner_mission.launch.py real:=true (ours): his M1 on the rig around our real ring, for the combined M1 contact
attach (card 2026-10-10_combined_m1_contact). No Gazebo node, the wall clock, his IRL values with every topic
written out (never a root /magnet/* topic of ours: his IRL file's /magnet/object_attached is our attach signal),
his contact target on our calibrated weld point, his supervisor optional and landing instead of disarming."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'
# the topics only our nodes may write (combined-stack design, P3)
OURS_ONLY = {'/magnet/command', '/magnet/ELRSCommand', '/magnet_tip_pose', '/magnet/attachment_state',
             '/drone_3/magnet', '/drone_3/ELRSCommand', '/ELRSCommand', '/motion_capture_state'}


def _module():
    spec = importlib.util.spec_from_file_location('partner_mission_launch_rig', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _rig(**typed):
    """(module, context, entities) as `ros2 launch ... real:=true contact_attach:=true contact_target_z:=0.031`."""
    typed = {'real': 'true', 'contact_attach': 'true', 'contact_target_z': '0.031',
             'magnet_drop_below_quad': '0.575', 'mpc_thrust_ratio': '22.0', **typed}
    mod = _module()
    ctx = LaunchContext()
    ctx.launch_configurations.update(typed)
    mod._record_typed(ctx)
    for a in mod._arguments():
        if isinstance(a, DeclareLaunchArgument) and a.name not in ctx.launch_configurations:
            ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
    return mod, ctx, mod._nodes(ctx)


def _s(ctx, x):
    return x if isinstance(x, str) else ''.join(t.perform(ctx) for t in x)


def _nodes(**typed):
    _, ctx, ents = _rig(**typed)
    out = {}
    for e in ents:
        if not isinstance(e, Node):
            continue
        params = {}
        for p in evaluate_parameters(ctx, e._Node__parameters or []):
            if isinstance(p, dict):
                params.update({k: list(v) if isinstance(v, tuple) else v for k, v in p.items()})
        out[e._Node__node_name] = {'params': params, 'ns': e._Node__node_namespace,
                                   'executable': e._Node__node_executable,
                                   'remaps': {_s(ctx, k): _s(ctx, v) for k, v in (e._Node__remappings or [])}}
    return out, ents


def test_rig_node_set_has_no_gazebo_node_and_the_wall_clock():
    g, ents = _nodes()
    assert set(g) == {'fake_cooperative_transport_world', 'partner_rig_state', 'magnet_attachment_manager_irl',
                      'online_join_planner', 'tejen_mpc'}
    assert all(n['params'].get('use_sim_time') is False for n in g.values())
    assert sum(isinstance(e, IncludeLaunchDescription) for e in ents) == 1          # his C++ backend
    gz = [e for e in ents if isinstance(e, ExecuteProcess) and 'gz' in str(e.cmd)]
    assert not gz and not [n for n in g.values() if n['executable'] == 'parameter_bridge']
    # his mocap publisher and radio node are never started: our mocap node and our radio own Motive and QUAD4
    assert not {'motion_capture_publisher_irl', 'elrs_interface_irl'} & {n['executable'] for n in g.values()}


def test_every_topic_is_written_out_and_none_is_ours():
    g, _ = _nodes()
    reads = {('partner_rig_state', k) for k in ('body_topic', 'tip_topic', 'ball_topic')}   # our mocap, read
    for name, n in g.items():
        for k, v in n['params'].items():
            if k.endswith('_topic') and isinstance(v, str) and v and (name, k) not in reads:
                if k == 'ring_attached_topic':
                    assert v == '/magnet/object_attached'      # he READS our confirmation
                    continue
                assert v not in OURS_ONLY | {'/magnet/object_attached'}, f'{name}.{k} = {v}'
    p = g['online_join_planner']['params']
    assert (p['object_attached_topic'], p['magnet_command_topic'], p['magnet_tip_pose_topic'],
            p['pickup_object_pose_topic'], p['pickup_object_index']) == (
        '/tejen/object_attached', '/tejen/magnet/command', '/tejen/magnet_tip_pose', '/tejen/pickup_object/pose', 0)
    m = g['magnet_attachment_manager_irl']
    assert m['ns'] == 'tejen' and m['params']['enable_elrs_magnet_output'] is False
    assert (m['params']['object_attached_topic'], m['params']['magnet_command_topic']) == (
        '/tejen/object_attached', '/tejen/magnet/command')


def test_contact_target_is_our_weld_point_and_the_rig_values():
    g, _ = _nodes()
    ring = g['fake_cooperative_transport_world']['params']
    assert (ring['bridge_attach_z_m'], ring['full_pose_attachment'], ring['ring_attachment_plate_index']) == (
        0.031, True, 3)
    assert ring['bridge_carrier_azimuths_deg'] == [270.0, 30.0, 150.0] and ring['bridge_attach_radius_m'] == 0.225
    p = g['online_join_planner']['params']
    assert p['contact_attach'] is True and p['final_attach_offset_z'] == -0.01       # 1 cm below the plate
    assert (p['proof_rise_max'], p['attach_ready_target_lead_time'], p['attach_ready_land_after_s']) == (
        0.03, 0.0, 0.0)
    assert p['magnet_drop_below_quad'] == 0.575                                       # his IRL value
    assert (p['pickup_lift_height'], p['drop_approach_clearance']) == (2.3, 1.35)
    assert (p['min_reference_z'], p['pickup_alignment_mode']) == (0.58, 'measured_tip_tiered')
    mpc = g['tejen_mpc']
    assert (mpc['params']['thrust_ratio'], mpc['params']['cable_length'], mpc['params']['pose_timeout_s'],
            mpc['params']['enable_thrust_ratio_feedback'], mpc['params']['object_mass_kg']) == (
        22.0, 0.531, 0.25, True, 0.0)
    assert mpc['remaps']['ELRSCommand'] == '/drone_3/ELRSCommand_tejen'
    assert mpc['remaps']['motion_capture_state'] == '/drone_3/motion_capture_state'
    assert g['partner_rig_state']['params']['anchor_offset_z'] == -0.030


def test_typed_plate_and_carriers_are_ours_and_clockwise():
    """Wesley 14 Oct: plate:=9 is our plate 9 (+y), his 3; carrier_azimuths_deg typed clockwise reach his ring bridge
    counter-clockwise. The defaults are the same layout."""
    g, _ = _nodes(plate='9', carrier_azimuths_deg='[90.0, 330.0, 210.0]')
    ring = g['fake_cooperative_transport_world']['params']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (3, [270.0, 30.0, 150.0])
    assert ring == _nodes()[0]['fake_cooperative_transport_world']['params']
    g, _ = _nodes(plate='3', carrier_azimuths_deg='[270.0, 30.0, 150.0]')    # the old spelling: the mirror image
    ring = g['fake_cooperative_transport_world']['params']
    assert (ring['ring_attachment_plate_index'], ring['bridge_carrier_azimuths_deg']) == (9, [90.0, 330.0, 210.0])


def test_typed_values_win():
    g, _ = _nodes(contact_dz='-0.005', magnet_drop_below_quad='0.582', proof_rise_max='0.02',
                  pickup_lift_height='2.0', mpc_thrust_ratio='24.5')
    p = g['online_join_planner']['params']
    assert (p['final_attach_offset_z'], p['magnet_drop_below_quad'], p['proof_rise_max'],
            p['pickup_lift_height']) == (-0.005, 0.582, 0.02, 2.0)
    assert g['tejen_mpc']['params']['thrust_ratio'] == 24.5


def test_supervisor_is_off_and_refused_on_the_rig():
    """Critic F9: refused on the combined rig until its give-up stop has been through the critic."""
    assert 'simulation_test_supervisor' not in _nodes()[0]
    with pytest.raises(RuntimeError, match='supervisor'):
        _rig(supervisor='true')


def test_drop_ends_on_the_measured_object():
    assert _nodes()[0]['online_join_planner']['params']['drop_check_puck'] is True


@pytest.mark.parametrize('typed,match', [({'contact_attach': 'false'}, 'contact_attach'),
                                         ({'contact_target_z': ''}, 'contact_target_z'),
                                         ({'geometry': 'rig_twin'}, 'geometry'),
                                         ({'magnet_drop_below_quad': ''}, 'magnet_drop_below_quad'),
                                         ({'mpc_thrust_ratio': ''}, 'mpc_thrust_ratio')])
def test_refusals(typed, match):
    with pytest.raises(RuntimeError, match=match):
        _rig(**typed)


def test_rig_bottom_hold_trim_give_up_and_one_plate_point():
    """Plan 2026-10-14 WP3: on the rig his planner holds at the bottom, uses his measured-tip trim, gives up after
    45 s, and his bridge aims at our weld point when bringup passes it."""
    g, _ = _nodes()
    p = g['online_join_planner']['params']
    assert (p['contact_bottom_hold'], p['contact_give_up_s'], p['target_alignment_mode']) == (
        True, 45.0, 'measured_tip_trim')
    assert 'bridge_attach_xy_m' not in g['fake_cooperative_transport_world']['params']   # his nominal plate
    g, _ = _nodes(contact_target_xy='0.013,0.210', contact_give_up_s='60', target_alignment_mode='legacy')
    assert g['fake_cooperative_transport_world']['params']['bridge_attach_xy_m'] == [0.013, 0.210]
    p = g['online_join_planner']['params']
    assert (p['contact_give_up_s'], p['target_alignment_mode']) == (60.0, 'legacy')
    with pytest.raises(RuntimeError, match='contact_target_xy'):
        _nodes(contact_target_xy='0.013')


def test_trim_speed_gate_only_when_typed():
    """His trim's learning speed stays his (no parameter) unless the sheet types trim_speed_gate."""
    g, _ = _nodes()
    assert 'target_alignment_max_relative_speed_mps' not in g['online_join_planner']['params']
    g, _ = _nodes(trim_speed_gate='0.20')
    assert g['online_join_planner']['params']['target_alignment_max_relative_speed_mps'] == 0.20


def test_match_dwell_and_speed_only_when_typed():
    """His gates' hold time stays his 1.0 s and the contact match speed 0.15 unless typed (a3_2, 15 Oct)."""
    g, _ = _nodes()
    p = g['online_join_planner']['params']
    assert 'dwell_time_s' not in p and p['contact_match_xy_velocity_threshold'] == 0.15
    g, _ = _nodes(match_dwell='0.5', match_speed='0.2')
    p = g['online_join_planner']['params']
    assert p['dwell_time_s'] == 0.5 and p['contact_match_xy_velocity_threshold'] == 0.2


def test_attach_z_lowpass_only_when_typed():
    """The bridge's plate height stays raw (no parameter) unless attach_z_lowpass is typed (a1, 15 Oct)."""
    g, _ = _nodes()
    assert 'attach_z_lowpass_hz' not in g['fake_cooperative_transport_world']['params']
    g, _ = _nodes(attach_z_lowpass='0.8')
    assert g['fake_cooperative_transport_world']['params']['attach_z_lowpass_hz'] == 0.8


def test_typed_plate_and_carrier_angles_are_ours_clockwise_and_arrive_as_his():
    """Since 15 Oct (Wesley 14 Oct) plate is our number (clockwise, 9 = +y) and carrier_azimuths_deg clockwise
    degrees; his nodes count counter-clockwise: his plate 3, his azimuths 270/30/150."""
    g, _ = _nodes()
    b = g['fake_cooperative_transport_world']['params']
    assert b['bridge_carrier_azimuths_deg'] == pytest.approx([270.0, 30.0, 150.0])
    assert b['ring_attachment_plate_index'] == 3
    g, _ = _nodes(plate='9', carrier_azimuths_deg='[90.0, 330.0, 210.0]')
    b = g['fake_cooperative_transport_world']['params']
    assert b['bridge_carrier_azimuths_deg'] == pytest.approx([270.0, 30.0, 150.0])
    assert b['ring_attachment_plate_index'] == 3


def test_his_planner_gets_the_ring_pose_for_the_hold_plate_frame():
    """Tejen 15 Oct: the bottom hold's below-the-plate exit and its distances are measured in the plate frame."""
    g, _ = _nodes()
    assert g['online_join_planner']['params']['ring_pose_topic'] == '/payload/motion_capture_state'


def test_the_twin_bridge_takes_the_plate_height_filter_only_when_typed():
    """The sim graph's ring bridge gets attach_z_lowpass_hz as the rig's does (twin check of RIG-1015-a3's switch)."""
    def bridge(**typed):
        mod = _module()
        ctx = LaunchContext()
        ctx.launch_configurations.update({'geometry': 'rig_twin', 'contact_attach': 'true', **typed})
        mod._record_typed(ctx)
        for a in mod._arguments():
            if isinstance(a, DeclareLaunchArgument) and a.name not in ctx.launch_configurations:
                ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
        for e in mod._nodes(ctx):
            if isinstance(e, Node) and e._Node__node_name == 'fake_cooperative_transport_world':
                params = {}
                for p in evaluate_parameters(ctx, e._Node__parameters or []):
                    params.update(p if isinstance(p, dict) else {})
                return params
    assert 'attach_z_lowpass_hz' not in bridge()
    assert bridge(attach_z_lowpass='0.8')['attach_z_lowpass_hz'] == 0.8


def test_the_twin_takes_the_rigs_match_gate_only_when_typed():
    """N6 (plan 2026-10-15): match_xy / match_speed / match_dwell reach his twin planner only when typed."""
    def planner(**typed):
        mod = _module()
        ctx = LaunchContext()
        ctx.launch_configurations.update({'geometry': 'rig_twin', 'contact_attach': 'true', **typed})
        mod._record_typed(ctx)
        for a in mod._arguments():
            if isinstance(a, DeclareLaunchArgument) and a.name not in ctx.launch_configurations:
                ctx.launch_configurations[a.name] = a.default_value[0].text if a.default_value else ''
        for e in mod._nodes(ctx):
            if isinstance(e, Node) and e._Node__node_name == 'online_join_planner':
                params = {}
                for p in evaluate_parameters(ctx, e._Node__parameters or []):
                    params.update(p if isinstance(p, dict) else {})
                return params
    p = planner()
    assert 'dwell_time_s' not in p and 'contact_match_xy_threshold' not in p
    p = planner(match_xy='0.05', match_speed='0.15', match_dwell='0.5')
    assert (p['contact_match_xy_threshold'], p['contact_match_xy_velocity_threshold'], p['dwell_time_s']) == (0.05, 0.15, 0.5)


def test_zero_twist_reaches_both_bridges_only_when_typed():
    """attach_twist (critic F4 of card 2026-10-16_plate_height_filter): rig and twin bridges, typed only."""
    g, _ = _nodes()
    assert 'attach_twist' not in g['fake_cooperative_transport_world']['params']
    g, _ = _nodes(attach_twist='zero')
    assert g['fake_cooperative_transport_world']['params']['attach_twist'] == 'zero'
