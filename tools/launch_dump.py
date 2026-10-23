#!/usr/bin/env python3
"""tools/launch_dump.py -- the node graph a launch would start, without starting anything.

    tools/launch_dump.py sim_control_launch.py mode:=mpc num_drones:=4 [--out g.json]
    tools/launch_dump.py --diff old.json new.json

Evaluates the launch file (a bringup/launch file name, or a path) in a bare LaunchContext
with the given arguments, follows includes, groups, timers and OpaqueFunctions, and
records every Node (package, executable, name, namespace, remappings, arguments and its
fully resolved parameters, each value with its type) and every ExecuteProcess command.
Nothing is started, no port or socket is opened. Two dumps that compare equal start the
same processes with the same parameters: the launch consolidation's equivalence gate
(plan noble-noodling-lollipop §3b).
"""
import argparse
import importlib.util
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(REPO, 'src', 'bringup', 'launch')


def _s(ctx, x):
    from launch.utilities import perform_substitutions
    if x is None or isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx)
    return perform_substitutions(ctx, list(x))


def _typed(v):
    """A parameter value with its ROS type, so 1 and 1.0 do not compare equal."""
    if isinstance(v, bool):
        return ['bool', v]
    if isinstance(v, int):
        return ['int', v]
    if isinstance(v, float):
        return ['double', v]
    if isinstance(v, str):
        return ['string', v]
    if isinstance(v, (list, tuple)):
        return ['array', [_typed(x) for x in v]]
    return [type(v).__name__, repr(v)]


def _node(ctx, e):
    from launch_ros.utilities import evaluate_parameters
    params = {}
    for d in evaluate_parameters(ctx, e._Node__parameters or []):
        if isinstance(d, dict):
            params.update(d)
        else:                                     # a parameter file: record its path
            params.setdefault('__params_files__', []).append(str(d))
    return {
        'kind': 'node',
        'package': _s(ctx, e._Node__package),
        'executable': _s(ctx, e._Node__node_executable),
        'name': _s(ctx, e._Node__node_name),
        'namespace': _s(ctx, e._Node__node_namespace),
        'remappings': sorted([_s(ctx, k), _s(ctx, v)] for k, v in (e._Node__remappings or [])),
        'arguments': [_s(ctx, a) for a in (e._Node__arguments or [])],
        'parameters': {k: _typed(v) for k, v in sorted(params.items())},
        'global_parameters': {k: _typed(v) for k, v in
                              sorted(dict(ctx.launch_configurations.get('global_params', [])).items())},
    }


def _collect(entities, ctx, out):
    from launch import LaunchDescription
    from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                                IncludeLaunchDescription, LogInfo, OpaqueFunction,
                                PopEnvironment, PushEnvironment, SetLaunchConfiguration,
                                TimerAction)
    from launch.actions.pop_launch_configurations import PopLaunchConfigurations
    from launch.actions.push_launch_configurations import PushLaunchConfigurations
    from launch_ros.actions import Node, SetParameter
    for e in entities:
        if getattr(e, 'condition', None) is not None and not e.condition.evaluate(ctx):
            continue
        if isinstance(e, (DeclareLaunchArgument, SetParameter, SetLaunchConfiguration,
                          PushLaunchConfigurations, PopLaunchConfigurations,
                          PushEnvironment, PopEnvironment)):
            e.execute(ctx)
        elif isinstance(e, Node):
            out.append(_node(ctx, e))
        elif isinstance(e, ExecuteProcess):
            out.append({'kind': 'process', 'name': _s(ctx, e.name) if e.name else None,
                        'cmd': [_s(ctx, c) for c in e.cmd]})
        elif isinstance(e, LogInfo):
            continue
        elif isinstance(e, LaunchDescription):
            _collect(e.entities, ctx, out)
        elif isinstance(e, TimerAction):
            _collect(e.actions, ctx, out)
        elif isinstance(e, (GroupAction, IncludeLaunchDescription, OpaqueFunction)):
            _collect(e.execute(ctx) or [], ctx, out)
        else:
            raise RuntimeError(f'launch_dump: unhandled launch entity {type(e).__name__}')


def dump(launch_file, args=None):
    """{'launch', 'args', 'entries'} for one launch file and its arguments."""
    from launch import LaunchContext
    path = launch_file if os.path.isabs(launch_file) or os.sep in launch_file \
        else os.path.join(LAUNCH_DIR, launch_file)
    spec = importlib.util.spec_from_file_location('launch_under_dump', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    args = {k: (str(v).lower() if isinstance(v, bool) else str(v)) for k, v in (args or {}).items()}
    ctx.launch_configurations.update(args)
    out = []
    _collect(mod.generate_launch_description().entities, ctx, out)
    key = lambda d: (d['kind'], d.get('namespace') or '', d.get('name') or '', d.get('executable') or '',
                     json.dumps(d.get('cmd')))
    return {'launch': os.path.basename(path), 'args': args, 'entries': sorted(out, key=key)}


def diff(a, b):
    """Human-readable differences between two dumps' entries ([] when equivalent)."""
    def index(d):
        return {(e['kind'], e.get('namespace') or '', e.get('name') or '',
                 json.dumps(e.get('cmd')) if e['kind'] == 'process' else ''): e for e in d['entries']}
    ia, ib = index(a), index(b)
    out = [f'only in A: {k}' for k in sorted(set(ia) - set(ib))]
    out += [f'only in B: {k}' for k in sorted(set(ib) - set(ia))]
    for k in sorted(set(ia) & set(ib)):
        ea, eb = ia[k], ib[k]
        for f in ('package', 'executable', 'remappings', 'arguments', 'global_parameters'):
            if ea.get(f) != eb.get(f):
                out.append(f'{k[2]}: {f} {ea.get(f)} != {eb.get(f)}')
        pa, pb = ea.get('parameters') or {}, eb.get('parameters') or {}
        for p in sorted(set(pa) | set(pb)):
            if pa.get(p) != pb.get(p):
                out.append(f'{k[2]}: {p} {pa.get(p)} != {pb.get(p)}')
    if len(ia) != len(a['entries']) or len(ib) != len(b['entries']):
        out.append(f'duplicate entries: A {len(a["entries"])}/{len(ia)}, B {len(b["entries"])}/{len(ib)}')
    return out


def parse_args(items):
    out = {}
    for it in items:
        if ':=' not in it:
            raise SystemExit(f'launch_dump: expected name:=value, got {it!r}')
        k, v = it.split(':=', 1)
        out[k] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('launch', nargs='?')
    ap.add_argument('args', nargs='*')
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2, metavar=('A', 'B'))
    a = ap.parse_args(argv)
    if a.diff:
        d = diff(json.load(open(a.diff[0])), json.load(open(a.diff[1])))
        print('\n'.join(d) if d else 'equivalent')
        return 1 if d else 0
    g = dump(a.launch, parse_args(a.args))
    text = json.dumps(g, indent=1, sort_keys=True)
    if a.out:
        open(a.out, 'w').write(text + '\n')
        print(f'{a.out}: {len(g["entries"])} entries')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
