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
    typed = {'real': 'true', 'contact_attach': 'true', 'contact_target_z': '0.031', **typed}
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
    g, _ = _nodes(supervisor='true')
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


def test_typed_values_win():
    g, _ = _nodes(contact_dz='-0.005', magnet_drop_below_quad='0.582', proof_rise_max='0.02',
                  pickup_lift_height='2.0', mpc_thrust_ratio='24.5')
    p = g['online_join_planner']['params']
    assert (p['final_attach_offset_z'], p['magnet_drop_below_quad'], p['proof_rise_max'],
            p['pickup_lift_height']) == (-0.005, 0.582, 0.02, 2.0)
    assert g['tejen_mpc']['params']['thrust_ratio'] == 24.5


def test_supervisor_is_optional_manual_and_lands():
    assert 'simulation_test_supervisor' not in _nodes()[0]
    s = _nodes(supervisor='true')[0]['simulation_test_supervisor']['params']
    assert (s['control_mode'], s['stop_action'], s['freshness_timeout_s']) == ('manual', 'land', 0.5)


@pytest.mark.parametrize('typed,match', [({'contact_attach': 'false'}, 'contact_attach'),
                                         ({'contact_target_z': ''}, 'contact_target_z'),
                                         ({'geometry': 'rig_twin'}, 'geometry')])
def test_refusals(typed, match):
    with pytest.raises(RuntimeError, match=match):
        _rig(**typed)
