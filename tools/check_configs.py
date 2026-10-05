#!/usr/bin/env python3
"""tools/check_configs.py -- every run config names a launch that exists and arguments it declares.

    python3 tools/check_configs.py            # configs/experiments + configs/sil
    python3 tools/check_configs.py a.yaml ... # just these

Per config: the control launch (launch.package / launch.file), the I/O launch of a Gazebo
config and every LAUNCH event's launch must be a file under src/<package>/launch, and
every argument the config sets must be one that launch knows: declared by it (or one it
includes), or for sim_control/real_control_launch.py a launch argument or a profile knob
of its side with a valid mode (the harness passes the knobs in a params file). `ros2
launch` ignores an undeclared argument without a word. Nothing is started. Exit 1 on any
problem.
"""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import launch_args  # noqa: E402
import run_experiment as R  # noqa: E402
from experiment.config import ExperimentConfig  # noqa: E402
from sil.scenario import Scenario  # noqa: E402


def _exists(pkg, lf):
    return os.path.exists(os.path.join(REPO, 'src', pkg, 'launch', lf))


def _undeclared(pkg, lf, args):
    if pkg == 'bringup' and lf in launch_args.CONTROL:
        side = launch_args.CONTROL[lf]
        mode = str(args.get('mode', launch_args.profiles().DEFAULT_MODE))
        bad = launch_args.unknown(lf, args)
        if mode not in launch_args.profiles().MODES[side]:
            bad.append(f'mode {mode}')
        return bad
    names = R.declared_launch_args(pkg, lf)
    return [k for k in args if k not in names]


def check(path):
    """Problems with one config, as strings."""
    out = []
    if os.path.basename(os.path.dirname(path)) == 'sil':
        s = Scenario.from_yaml(path)
        launches = [(s.launch_package, s.launch_file, dict(s.launch_args))]
    else:
        c = ExperimentConfig.from_yaml(path)
        launches = [(c.launch_package, c.launch_file, dict(c.launch_args)),
                    ('bringup', c.io_launch_file, dict(a.split(':=', 1) for a in c.io_launch_argv()))]
        for e in c.events:
            if e.do == 'LAUNCH':
                pkg, lf, *largs = str(e.arg).split()
                launches.append((pkg, lf, dict(a.split(':=', 1) for a in largs)))
    for pkg, lf, argv in launches:
        if not _exists(pkg, lf):
            out.append(f'no launch src/{pkg}/launch/{lf}')
            continue
        bad = _undeclared(pkg, lf, argv)
        if bad:
            out.append(f'{lf}: undeclared {", ".join(bad)}')
    return out


def main(argv=None):
    paths = (argv if argv else sys.argv[1:]) or sorted(
        glob.glob(os.path.join(REPO, 'configs', 'experiments', '*.yaml'))
        + glob.glob(os.path.join(REPO, 'configs', 'sil', '*.yaml')))
    n_bad = 0
    for p in paths:
        try:
            probs = check(p)
        except Exception as e:                       # noqa: BLE001
            probs = [f'does not load: {type(e).__name__}: {e}']
        if probs:
            n_bad += 1
            print(f'{os.path.relpath(p, REPO)}: ' + '; '.join(probs))
    print(f'{len(paths)} configs, {n_bad} with problems')
    return 1 if n_bad else 0


if __name__ == '__main__':
    sys.exit(main())
