#!/usr/bin/env python3
"""deskeval.py <launch file> name:=value ... -- evaluate a bringup launch offline (nothing started) and print a read-back.

The LaunchContext walk of src/bringup/test/test_real_mode_launches.py (it handles ResetEnvironment, unlike
tools/launch_dump.py). Prints the launch's own output (QUAD4's planned heights), the node list and the read-back keys.
Exit 1 on any exception (a refusal)."""
import importlib.util
import json
import os
import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, PopEnvironment, PushEnvironment, ResetEnvironment,
                            ResetLaunchConfigurations, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction)
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.push_launch_configurations import PushLaunchConfigurations
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.utilities import evaluate_parameters

LAUNCH_DIR = os.path.expanduser('~/multi_drone_control/src/bringup/launch')


def _s(ctx, x):
    if x is None or isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx)
    return perform_substitutions(ctx, list(x))


def _param_file(path, name):
    import yaml
    data = yaml.safe_load(open(path)) or {}
    out = {}
    for key in ('/**', name, '/' + name):
        out.update((data.get(key) or {}).get('ros__parameters', {}))
    return out


def _record(ctx, e):
    params = {}
    for d in evaluate_parameters(ctx, e._Node__parameters or []):
        params.update(d if isinstance(d, dict) else _param_file(d, _s(ctx, e._Node__node_name)))
    return {'package': _s(ctx, e._Node__package), 'executable': _s(ctx, e._Node__node_executable),
            'name': _s(ctx, e._Node__node_name), 'namespace': _s(ctx, e._Node__node_namespace), 'params': params}


def _collect(entities, ctx, out):
    for e in entities:
        if getattr(e, 'condition', None) is not None and not e.condition.evaluate(ctx):
            continue
        if isinstance(e, (DeclareLaunchArgument, SetParameter, SetLaunchConfiguration, PushLaunchConfigurations,
                          PopLaunchConfigurations, PushEnvironment, PopEnvironment, ResetEnvironment,
                          ResetLaunchConfigurations, SetEnvironmentVariable)):
            e.execute(ctx)
        elif isinstance(e, Node):
            out.append(_record(ctx, e))
        elif isinstance(e, ExecuteProcess):
            out.append({'package': 'process', 'executable': ' '.join(_s(ctx, c) for c in e.cmd)[:80],
                        'name': _s(ctx, e.name) if e.name else 'process', 'namespace': '', 'params': {}})
        elif isinstance(e, LogInfo):
            continue
        elif isinstance(e, LaunchDescription):
            _collect(e.entities, ctx, out)
        elif isinstance(e, TimerAction):
            _collect(e.actions, ctx, out)
        elif isinstance(e, (GroupAction, IncludeLaunchDescription, OpaqueFunction)):
            _collect(e.execute(ctx) or [], ctx, out)
        else:
            raise AssertionError(f'unhandled launch entity {type(e).__name__}')


# what to read back, per node (substring of namespace/name) -> keys
READ = {
    'planner': ['num_drones', 'load_mass', 'attach_radius', 'cable_len', 'target_z', 'load_traj', 'traj_speed',
                'traj_radius', 'traj_hold_s', 'drop_tilt_deg', 'detach_detect_m', 'tip_detect_m', 'attach_pause',
                'int_k_xy', 'land_descend_to_stall', 'land_slow_vel', 'land_slow_z', 'rod_len_reread',
                'partner_release_around', 'reattach_plate', 'foldin_cap', 'ring_low_land_m', 'attach_azimuths_deg'],
    'fleet_manager': ['num_drones', 'pos_limit_m'],
    'join_planner': ['match_xy', 'match_dwell', 'match_speed', 'contact_dz', 'takeoff_gate_topic', 'drop_height',
                     'drop_on_arrival', 'max_body_z', 'mission_mode', 'plate', 'line_up', 'lift_full_before_transit',
                     'pickup_contact_height', 'pickup_object_thickness', 'pickup_object_size'],
}


def main():
    launch = sys.argv[1]
    args = {}
    for it in sys.argv[2:]:
        k, v = it.split(':=', 1)
        args[k] = v
    path = launch if os.path.isabs(launch) else os.path.join(LAUNCH_DIR, launch)
    spec = importlib.util.spec_from_file_location('launch_under_desk', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update(args)
    out = []
    try:
        _collect(mod.generate_launch_description().entities, ctx, out)
    except Exception as exc:  # a refusal
        print(f'DESK: REFUSED / ERROR: {type(exc).__name__}: {exc}')
        return 1
    print(f'DESK: OK, {len(out)} nodes')
    if os.environ.get('DESK_DUMP'):
        json.dump(out, open(os.environ['DESK_DUMP'], 'w'), default=str, indent=1)
    for n in out:
        ns = (n['namespace'] or '') + '/' + (n['name'] or '')
        p = n['params']
        hits = {}
        for key, names in READ.items():
            if key in ns or key in (n['executable'] or ''):
                hits.update({k: p[k] for k in names if k in p})
        # trackers: the tilt drop, the loss check and QUAD4's mux topic
        if n['executable'] == 'tracker':
            hits.update({k: p[k] for k in ('drop_tilt_deg', 'max_payload_tilt_deg', 'mux_state_topic', 'throttle_max',
                                           'cable_notch_hz', 'pos_limit_m') if k in p})
        # any partner-side node with takeoff gate / match knobs
        for k in ('takeoff_gate_topic', 'match_dwell', 'match_xy', 'contact_dz', 'drop_height', 'max_body_z'):
            if k in p:
                hits[k] = p[k]
        if os.environ.get('DESK_FIND'):
            for k in os.environ['DESK_FIND'].split(','):
                for pk, pv in p.items():
                    if k in pk:
                        hits[pk] = pv
        if hits:
            print(f'  {ns} [{n["executable"]}]: ' + json.dumps(hits, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
