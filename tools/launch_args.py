"""tools/launch_args.py -- a run config's launch arguments as the harnesses pass them.

A config sets every knob under `launch.args`. For sim_control_launch.py / real_control_launch.py
only the launch arguments (bringup.profiles.kept) go on the command line; the other knobs are
written to a params file in the run directory (provenance) and passed as params_file:=.
Used by tools/run_experiment.py and tools/sil_bench.py (and the equivalence gate).
"""
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL = {'sim_control_launch.py': 'sim', 'real_control_launch.py': 'real'}


def profiles():
    try:
        from bringup import profiles as p
    except ImportError:                       # not sourced: the source tree has it
        sys.path.insert(0, os.path.join(REPO, 'src', 'bringup'))
        from bringup import profiles as p
    return p


def _plain(v):
    # str(True) is 'True'; a launch wants 'true' (a bool arg would fall to its default)
    return ('true' if v else 'false') if isinstance(v, bool) else v


def split(launch_file, args):
    """(launch arguments, params_file knobs) of a config's launch.args."""
    args = {k: _plain(v) for k, v in (args or {}).items()}
    side = CONTROL.get(launch_file)
    if not side:
        return args, {}
    return profiles().split(side, args)


def unknown(launch_file, args):
    """Knobs of launch.args that the control launch knows neither as an argument nor a knob."""
    side = CONTROL.get(launch_file)
    if not side:
        return []
    p = profiles()
    known = set(p.kept(side)) | p.side_knobs(side) | {'mode', 'params_file'}
    return sorted(k for k in (args or {}) if k not in known and not p.PER_DRONE.match(k))


def argv(launch_file, args, params_path=None):
    """`name:=value` list; the knobs that are not launch arguments are written to params_path."""
    largs, params = split(launch_file, args)
    if params:
        if not params_path:
            raise ValueError(f'{launch_file}: {", ".join(sorted(params))} need a params file')
        # a typed params_file is merged in, not replaced: replacing it dropped cable_source mocap
        # from a spin run silently on 7 Oct (R1114). The config's own knobs win on a clash.
        typed = largs.get('params_file')
        if typed:
            with open(os.path.expanduser(str(typed))) as fh:
                base = yaml.safe_load(fh) or {}
            flat = {}
            for k, v in base.items():
                if isinstance(v, dict) and k != 'modes':
                    flat.update(v)
                elif k != 'modes':
                    flat[k] = v
            params = {**flat, **params}
        with open(params_path, 'w') as fh:
            yaml.safe_dump(params, fh, default_flow_style=False, sort_keys=True)
        largs['params_file'] = params_path
    return [f'{k}:={v}' for k, v in largs.items()]
