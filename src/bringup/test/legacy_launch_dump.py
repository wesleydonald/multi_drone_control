"""A full dump of what the launch files start for a set of today's (single-partner) lines, for the 4 -> 2 -> 4 rule:
with partners_file empty every node, topic, parameter and launch output must be identical to the tree before it.

    python3 legacy_launch_dump.py <tree root> [<configs root>]  > dump.json

Run with PYTHONPATH on that tree's packages (test_partners_424.py runs it for the pristine tree and this one). Nothing
is started: each launch is evaluated offline, as test_real_mode_launches.py does, and every Node (name, namespace,
executable, parameters, remappings, arguments), LogInfo text, TimerAction period, include argument list and generated
RViz config is written out; a refused line records its error text.
"""
import contextlib
import importlib.util
import io
import json
import os
import re
import sys
import tempfile

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            PopEnvironment, PushEnvironment, ResetEnvironment, ResetLaunchConfigurations,
                            SetLaunchConfiguration, TimerAction)
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.push_launch_configurations import PushLaunchConfigurations
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.utilities import evaluate_parameters

QUAD4_NRTF = {'weld_radius': 0.025, 'proof_taut_len': 1.031, 'weld_point_z': 0.0491, 'attach_cable_len': 0.994,
              'attach_x_offset': 0.0, 'attach_y_offset': 0.2293}
HIS_NRTF = ('mpc_thrust_ratio:=22 mpc_cable_length:=1.0 trim_speed_gate:=0.20 carry_clearance:=0.25 plate:=9 '
            'carrier_azimuths_deg:=[90.0,330.0,210.0] commit_backdate:=0.04 tracking_check:=off '
            'attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20 approach_offset_z:=0.30 '
            'attach_twist:=zero match_xy:=0.05 match_dwell:=0.5 climb_speed:=0.3 match_speed:=0.20 '
            'descent_abort_speed:=0.35 descent_hold:=true geometry:=rig_m2a mission_mode:=floor_join '
            'descent_debounce:=1.0 descent_abort_xy:=0.05 contact_give_up_s:=120 contact_dz:=-0.03')
COMOTION = {'tip_skip_repeats': True, 'comotion_d_ok': 0.02, 'comotion_speed_window_s': 0.5,
            'comotion_taut_margin': 0.05, 'comotion_normal_min': -0.03, 'comotion_normal_max': 0.01,
            'comotion_normal_ok': 0.04, 'comotion_sep_normal': 0.04, 'comotion_speed': 0.06, 'comotion_speed_ok': 0.06,
            'comotion_proof_dwell_s': 0.5, 'comotion_inconclusive_s': 6.0, 'comotion_lost': 0.06,
            'comotion_sep_xy': 0.06}
PARTNER_ATTACHED_ORBIT = {
    'num_drones': 3, 'reserved_attach': 1, 'start_taut': True, 'load_traj': 'orbit',
    'traj_speed': 0.125, 'traj_radius': 0.5, 'cable_len': 0.5, 'load_mass': 0.86,
    'target_z': 0.6, 'attach_azimuths_deg': '210,90,330', 'attach_central': False,
    'diss_balanced_tensions': True, 'attach_handout': True, 'attach_t_handout': 12.0,
    'attach_x_offset': 0.0, 'attach_y_offset': 0.25, 'attach_elev_deg': 65.0,
    'attach_traj_hold_s': 10.0, 'attach_traj_hold_mode': 'timed',
    'control_mode': 'velocity_after_handover', 'vel_ki': 0.0, 'diss_ki_load': 1.0,
    'enable_obstacle_avoidance': False, 'kt_trim': True, 'z_ki': 0.4, 'reconfig_mode': 'ocp',
    'safety_ref_timeout_s': 4.0, 'pose_timeout_s': 3.0, 'attach_cable_len': 0.49,
    'partner': True, 'partner_attached': True}


def cases(configs):
    """(name, launch file, args, dds) of today's lines: the rig scripts of 13 Nov, the tests' twin and legacy lines."""
    rig_pf = os.path.join(configs, 'rig', 'attach_m1_1m_m2a_drop55_det15.yaml')
    fig8_pf = os.path.join(configs, 'rig', 'attach_m1_1m_m2a_fig8_det15.yaml')
    rig_line = {'mode': 'attach', 'target_z': 1.0, 'lift_ramp_vel': 0.15, 'throttle_max': 0.95,
                'enable_approach_mpc': False, 'mocap_magnet_tip_body_id': 24, 'attach_azimuths_deg': '90,330,210',
                'partner': True, 'partner_contact': True, 'partner_nodes': True, **QUAD4_NRTF, **COMOTION,
                'magnet_operator_topic': '/drone_3/magnet_operator', 'partner_attached': True,
                'detach_unload_s': 3.0}
    out = [
        ('rig attach plain', 'real_control_launch.py',
         {'mode': 'attach', 'enable_approach_mpc': False, 'mocap_magnet_tip_body_id': 24}, False),
        ('rig attach no approach', 'real_control_launch.py',
         {'mode': 'attach', 'enable_approach': False}, False),
        ('rig partner demo', 'real_control_launch.py',
         {'mode': 'attach', 'enable_approach_mpc': False, 'mocap_magnet_tip_body_id': 24, 'partner': True}, False),
        ('rig nrtf', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'partner_args': HIS_NRTF}, True),
        ('rig nr8', 'real_control_launch.py',
         {**rig_line, 'params_file': fig8_pf, 'load_traj': 'fig_8', 'fig8_size': 0.75, 'traj_speed': 0.125,
          'contact_rig_fig8': True, 'partner_args': HIS_NRTF}, True),
        ('rig nrtf no his nodes', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'partner_nodes': False}, True),
        ('rig nrtf every partner switch', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'partner_nodes': False, 'partner_tilt_cut': True,
          'partner_stuck_guard': True, 'fc_disarm_land': True, 'welded_on_load': True, 'dry_hold': True,
          'warm_fault_not_live': True}, True),
        ('rig contact not welded', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'partner_attached': False,
          'partner_args': HIS_NRTF}, True),
        ('rig reattach plate 1', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'reattach_plate': 1,
          'partner_args': HIS_NRTF.replace(' plate:=9', '')}, True),
        ('rig m2', 'real_control_launch.py', {'mode': 'm2', 'partner_m2': True}, False),
        ('rig dissipative', 'real_control_launch.py', {'mode': 'dissipative', 'num_drones': 3}, False),
        ('twin attach', 'sim_control_launch.py', {'mode': 'attach', 'legacy': False, 'num_drones': 3}, False),
        ('twin attach catch offset', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': False, 'num_drones': 3, 'catch_offset_t': 0.04}, False),
        ('twin partner', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': False, 'num_drones': 3, 'partner': True}, False),
        ('twin full m1', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': False, 'num_drones': 3, 'partner': True, 'partner_contact': True,
          'partner_attached': True, 'partner_tilt_cut': True, 'fc_disarm_land': True}, False),
        ('twin full m1 plate 1', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': False, 'num_drones': 3, 'partner': True, 'partner_contact': True,
          'partner_attached': True, 'reattach_plate': 1}, False),
        ('legacy attach', 'sim_control_launch.py', {'mode': 'attach', 'legacy': True, 'num_drones': 3}, False),
        ('legacy partner orbit', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': True, **PARTNER_ATTACHED_ORBIT}, False),
        ('sil attach', 'sim_control_launch.py',
         {'mode': 'attach', 'legacy': False, 'num_drones': 3, 'sil': True}, False),
        ('twin dissipative', 'sim_control_launch.py', {'mode': 'dissipative', 'num_drones': 3}, False),
        ('real io attach', 'real_io_launch.py', {'num_drones': 4, 'attach': True}, False),
        ('real io overrides', 'real_io_launch.py',
         {'num_drones': 4, 'attach': True, 'magnet_topic_overrides': '3:/drone_3/magnet_coil',
          'mocap_magnet_tip_body_id': 24, 'mocap_extra_bodies': '6:/pickup/motion_capture_state'}, False),
        ('sim io attach', 'sim_io_launch.py', {'num_drones': 3, 'attach': True}, False),
        ('sim io carry', 'sim_io_launch.py', {'num_drones': 3}, False),
        # refusals: their text is launch output too
        ('rig attach two carriers', 'real_control_launch.py',
         {'mode': 'attach', 'enable_approach_mpc': False, 'mocap_magnet_tip_body_id': 24, 'num_drones': 2}, False),
        ('rig nrtf no weld point', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'weld_point_z': 0.0}, True),
        ('rig nrtf rod off the taut length', 'real_control_launch.py',
         {**rig_line, 'params_file': rig_pf, 'load_traj': 'hover', 'attach_cable_len': 1.05}, True),
        ('rig attach no tip', 'real_control_launch.py', {'mode': 'attach', 'enable_approach_mpc': False}, False),
    ]
    return out


def _s(ctx, x):
    if x is None or isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx)
    return perform_substitutions(ctx, list(x))


def _param_file(path, name):
    data = yaml.safe_load(open(path)) or {}
    out = {}
    for key in ('/**', name, '/' + name):
        out.update((data.get(key) or {}).get('ros__parameters', {}))
    return out


def _node(ctx, e):
    params = {}
    for d in evaluate_parameters(ctx, e._Node__parameters or []):
        params.update(d if isinstance(d, dict) else _param_file(d, _s(ctx, e._Node__node_name)))
    glob = dict(ctx.launch_configurations.get('global_params', []))
    args = [_s(ctx, a) for a in (e._Node__arguments or [])]
    rec = {'node': (_s(ctx, e._Node__node_namespace) or '') + '/' + (_s(ctx, e._Node__node_name) or ''),
           'package': _s(ctx, e._Node__package), 'executable': _s(ctx, e._Node__node_executable),
           'params': params, 'global': {k: v for k, v in glob.items()},
           'remappings': [(_s(ctx, k), _s(ctx, v)) for k, v in (e._Node__remappings or [])],
           'arguments': args, 'prefix': _s(ctx, e.process_description.prefix)}
    if rec['executable'] == 'rviz2' and len(args) > 1 and os.path.isfile(args[1]):
        rec['rviz_config'] = open(args[1]).read()
    return rec


def _collect(entities, ctx, out):
    for e in entities:
        if getattr(e, 'condition', None) is not None and not e.condition.evaluate(ctx):
            continue
        if isinstance(e, (DeclareLaunchArgument, SetParameter, SetLaunchConfiguration, PushLaunchConfigurations,
                          PopLaunchConfigurations, PushEnvironment, PopEnvironment, ResetEnvironment,
                          ResetLaunchConfigurations)):
            e.execute(ctx)
        elif isinstance(e, Node):
            out.append(_node(ctx, e))
        elif isinstance(e, LogInfo):
            out.append({'log': ''.join(_s(ctx, m) for m in e.msg)})
        elif isinstance(e, LaunchDescription):
            _collect(e.entities, ctx, out)
        elif isinstance(e, TimerAction):
            out.append({'timer': _s(ctx, e.period) if not isinstance(e.period, (int, float)) else e.period})
            _collect(e.actions, ctx, out)
        elif isinstance(e, IncludeLaunchDescription):
            loc = perform_substitutions(ctx, e.launch_description_source._LaunchDescriptionSource__location)
            out.append({'include': os.path.basename(loc),
                        'arguments': [(_s(ctx, k), _s(ctx, v)) for k, v in e.launch_arguments]})
            # ours is evaluated; another package's (his partner_mission, the same file for both trees) is only
            # recorded: evaluating it writes log directories into the flown tree
            if FOLLOW[0] or os.path.dirname(os.path.abspath(loc)) == LAUNCH_DIR[0]:
                _collect(e.execute(ctx) or [], ctx, out)
        elif isinstance(e, (GroupAction, OpaqueFunction)):
            _collect(e.execute(ctx) or [], ctx, out)
        else:
            out.append({'entity': type(e).__name__})


LAUNCH_DIR = ['']
FOLLOW = [False]


def evaluate(launch_dir, launch_file, args, strict=False, follow=False):
    """The launch output of one line (a list of records); strict: a refusal raises instead of being recorded;
    follow: another package's include is evaluated too (his launch: the caller redirects its log directory)."""
    LAUNCH_DIR[0] = os.path.abspath(launch_dir)
    FOLLOW[0] = follow
    args = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    side = {'sim_control_launch.py': 'sim', 'real_control_launch.py': 'real'}.get(launch_file)
    tmp = None
    if side:
        from bringup.profiles import split
        args, params = split(side, args)
        if params:
            with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
                yaml.safe_dump(params, fh)
            tmp = args['params_file'] = fh.name
    spec = importlib.util.spec_from_file_location('launch_under_dump', os.path.join(launch_dir, launch_file))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update(args)
    out = []
    printed = io.StringIO()
    try:
        with contextlib.redirect_stdout(printed):
            _collect(mod.generate_launch_description().entities, ctx, out)
    except Exception as exc:          # a refusal is output too
        if strict:
            raise
        out.append({'error': f'{type(exc).__name__}: {exc}'})
    out.append({'printed': printed.getvalue()})
    text = json.dumps(out, sort_keys=True, default=repr)
    if tmp:
        text = text.replace(tmp, '<params_file>')
    # temporary files the launches write (tempfile names end in 8 random characters)
    return json.loads(re.sub(r'(/tmp/[A-Za-z_]*)[a-z0-9_]{8}\.yaml', r'\1<tmp>.yaml', text))


def dump(tree, configs=None):
    configs = configs or os.path.join(tree, 'configs')
    launch_dir = os.path.join(tree, 'src', 'bringup', 'launch')
    result = {}
    for name, launch_file, args, dds in cases(configs):
        saved = {k: os.environ.get(k) for k in ('ROS_LOCALHOST_ONLY', 'ROS_DOMAIN_ID')}
        if dds:
            os.environ.update({'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '77'})
        else:
            for k in saved:
                os.environ.pop(k, None)
        try:
            result[name] = evaluate(launch_dir, launch_file, args)
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
    return result


if __name__ == '__main__':
    os.environ.pop('MDC_PLANNER_CPUS', None)
    json.dump(dump(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None), sys.stdout, sort_keys=True, indent=1)
