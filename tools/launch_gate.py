#!/usr/bin/env python3
"""tools/launch_gate.py -- the launch consolidation's equivalence gate (plan noble-noodling-lollipop §3b).

    tools/launch_gate.py results/launch_gate_2026-10-03            # every frozen case
    tools/launch_gate.py results/launch_gate_2026-10-03 -k wed_     # cases whose id contains wed_

Each case is a pre-consolidation launch command (tools/launch_gate_cases.py froze its node
dump). It is mapped to the consolidated launches (old_to_new: the file and mode, the launch
arguments that remain, every other knob in a params_file) and dumped again; the two dumps
must be identical node for node and parameter for parameter, with each value's type. A case
whose old launch refused must still refuse. Writes <dir>/new/<id>.json and <dir>/report.txt.
"""
import argparse
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import launch_dump  # noqa: E402

TRUE = ('1', 'true', 'yes')
IO_RENAMES = {'rviz_quad_load_launch.py': 'sim_io_launch.py',
              'm2_bench_io_launch.py': 'sim_m2_bench_launch.py',
              'real_io_launch.py': 'real_io_launch.py'}


def old_to_new(launch, args):
    """(new launch file, launch args, params_file knobs) for a pre-consolidation command."""
    from bringup import profiles
    args = dict(args)
    if launch in IO_RENAMES:
        return IO_RENAMES[launch], args, {}
    real = str(args.pop('real', 'false')).lower() in TRUE
    if launch == 'mpc_quad_load_launch.py':
        side, mode = 'sim', 'free_hover' if args.pop('reference', 'planner') == 'free_hover' else 'mpc'
    elif launch == 'dissipative_launch.py':
        side, mode = ('real', 'm2') if real else ('sim', 'dissipative')
    elif launch == 'dissipative_only_launch.py':
        side, mode = 'sim', 'network'
    elif launch == 'three_attach_launch.py':
        side, mode = ('real', 'attach') if real else ('sim', 'attach')
    elif launch == 'real_control_launch.py':
        side, mode = 'real', 'mpc'
    elif launch == 'real_dissipative_launch.py':
        side, mode = 'real', 'dissipative'
    elif launch == 'real_hover_launch.py':
        side, mode = 'real', 'free_hover'
    else:
        raise ValueError(f'no mapping for {launch}')
    if side == 'sim':                         # the frozen cases predate the twin default
        args = {'legacy': True, **args}
    largs, params = profiles.split(side, {'mode': mode, **args})
    return f'{side}_control_launch.py', largs, params


def via_config(case, tmpdir):
    """(launch file, launch args) the harness builds from the converted config file."""
    from experiment.config import ExperimentConfig
    from sil.scenario import Scenario
    path = os.path.join(os.path.dirname(HERE), case['config'])
    pp = os.path.join(tmpdir, 'params_file.yaml')
    if case['kind'] == 'sil':
        s = Scenario.from_yaml(path)
        return s.launch_file, {'sil': 'true', **_argv(s.launch_argv(pp))}
    cfg = ExperimentConfig.from_yaml(path)
    if case['kind'] == 'io':
        return cfg.io_launch_file, _argv(cfg.io_launch_argv())
    return cfg.launch_file, _argv(cfg.launch_argv(pp))


def _argv(items):
    return dict(it.split(':=', 1) for it in items)


def run_via_config(case, outdir):
    with tempfile.TemporaryDirectory() as td:
        new_launch, largs = via_config(case, td)
        try:
            new = launch_dump.dump(new_launch, largs)
        except Exception as e:                              # noqa: BLE001
            new = {'error': f'{type(e).__name__}: {e}'}
    old = json.load(open(os.path.join(outdir, 'old', case['id'] + '.json')))
    if 'error' in old or 'error' in new:
        ok = 'error' in old and 'error' in new
        return ok, [] if ok else [f'old: {old.get("error", "ok")}', f'new: {new.get("error", "ok")}']
    d = launch_dump.diff(old, new)
    return not d, d


def run_case(case, outdir):
    new_launch, largs, params = old_to_new(case['launch'], case['args'])
    pf = None
    if params:
        pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        import yaml
        yaml.safe_dump(params, pf)
        pf.close()
        largs['params_file'] = pf.name
    try:
        new = launch_dump.dump(new_launch, largs)
    except Exception as e:                                  # noqa: BLE001
        new = {'error': f'{type(e).__name__}: {e}'}
    finally:
        if pf:
            os.unlink(pf.name)
    new['command'] = {'launch': new_launch, 'args': {k: v for k, v in largs.items() if k != 'params_file'},
                      'params_file': params}
    json.dump(new, open(os.path.join(outdir, 'new', case['id'] + '.json'), 'w'), indent=1, sort_keys=True)
    old = json.load(open(os.path.join(outdir, 'old', case['id'] + '.json')))
    if 'error' in old or 'error' in new:
        ok = 'error' in old and 'error' in new
        return ok, [] if ok else [f'old: {old.get("error", "ok")}', f'new: {new.get("error", "ok")}']
    d = launch_dump.diff(old, new)
    return not d, d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('dir')
    ap.add_argument('-k', default='', help='only case ids containing this')
    ap.add_argument('--via-configs', action='store_true',
                    help='config cases only, rebuilt by the harness from the converted configs')
    a = ap.parse_args(argv)
    os.makedirs(os.path.join(a.dir, 'new'), exist_ok=True)
    cases = [c for c in json.load(open(os.path.join(a.dir, 'cases.json'))) if a.k in c['id']
             and (not a.via_configs or 'config' in c)]
    lines, fails = [], 0
    for c in cases:
        ok, d = run_via_config(c, a.dir) if a.via_configs else run_case(c, a.dir)
        if not ok:
            fails += 1
            lines.append(f'FAIL {c["id"]} ({c["launch"]})')
            lines += [f'    {x}' for x in d[:12]] + ([f'    ... {len(d) - 12} more'] if len(d) > 12 else [])
    lines.append(f'{len(cases) - fails}/{len(cases)} cases equivalent')
    text = '\n'.join(lines)
    name = 'report' + ('_via_configs' if a.via_configs else '') + (f'_{a.k}' if a.k else '') + '.txt'
    open(os.path.join(a.dir, name), 'w').write(text + '\n')
    print(text)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
