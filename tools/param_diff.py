#!/usr/bin/env python3
"""
param_diff.py — what actually differs between two launch configurations? (F9)

Answers "which parameters do I need to change between simulation and the real rig?"
without reading two configurations side by side and hoping you spotted everything.

The control launches take every default from bringup/config (common.yaml, sim.yaml,
real.yaml and their modes: entries); this reads those, so there is no second copy of the
configuration to drift. Any other launch file is read from its DeclareLaunchArgument
defaults.

    tools/param_diff.py sim/mpc real/mpc              # side/mode of the control launches
    tools/param_diff.py sim/dissipative real/m2
    tools/param_diff.py sim_io_launch.py real_io_launch.py
    tools/param_diff.py --sim-vs-real                  # the pairs that fly the same graph

  CHANGED       known to both, different defaults
  ONLY IN A/B   known to one only -- the other falls back to the NODE default, silently
                (a launch file), or the knob does not exist on that side (a profile).
"""
import argparse
import ast
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
LAUNCH_DIR = REPO / 'src/bringup/launch'

# The pairs worth comparing routinely: the sim mode and the rig mode that fly one graph.
SIM_VS_REAL = [('sim/mpc', 'real/mpc'), ('sim/dissipative', 'real/dissipative'),
               ('sim/attach', 'real/attach'), ('sim/dissipative', 'real/m2')]


def profile_args(spec):
    """{knob: default} of 'side/mode' (bringup/config), or None if spec is not one."""
    side, _, mode = str(spec).partition('/')
    if side not in ('sim', 'real') or not mode:
        return None
    sys.path.insert(0, str(REPO / 'src' / 'bringup'))
    from bringup.profiles import profile
    return profile(side, mode, cdir=str(REPO / 'src' / 'bringup' / 'config'))


def launch_args(path):
    """{name: default} from DeclareLaunchArgument, without executing the launch."""
    tree = ast.parse(pathlib.Path(path).read_text())
    out = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'
                and node.args):
            name = getattr(node.args[0], 'value', None)
            default = None
            for kw in node.keywords:
                if kw.arg == 'default_value':
                    default = getattr(kw.value, 'value', None)
            if isinstance(name, str):
                out[name] = default
    return out


def resolve(name):
    p = pathlib.Path(name)
    if p.exists():
        return p
    p = LAUNCH_DIR / name
    if p.exists():
        return p
    raise SystemExit(f"launch file not found: {name}")


def args_of(spec):
    """(name, {arg: default}) of a 'side/mode' profile or a launch file."""
    prof = profile_args(spec)
    if prof is not None:
        return str(spec), prof
    path = resolve(spec)
    return path.name, launch_args(path)


def compare(a_spec, b_spec, quiet=False):
    (an, a), (bn, b) = args_of(a_spec), args_of(b_spec)

    changed = {k: (a[k], b[k]) for k in sorted(a.keys() & b.keys()) if a[k] != b[k]}
    only_a = sorted(a.keys() - b.keys())
    only_b = sorted(b.keys() - a.keys())

    print(f"\n══ {an}  vs  {bn} ══")
    print(f"   {len(a)} vs {len(b)} known arguments\n")

    if changed:
        print(f"── CHANGED ({len(changed)}) — same argument, different default")
        w = max(len(k) for k in changed)
        for k, (va, vb) in changed.items():
            print(f"   {k:<{w}}  {str(va):>12}  ->  {str(vb)}")
    else:
        print("── CHANGED: none")

    for label, only, src, dst in (('ONLY IN ' + an, only_a, an, bn),
                                  ('ONLY IN ' + bn, only_b, bn, an)):
        if not only:
            continue
        print(f"\n── {label} ({len(only)}) — {dst} falls back to the NODE default")
        w = max(len(k) for k in only)
        vals = a if src == an else b
        for k in only:
            print(f"   {k:<{w}}  = {vals[k]}")

    return changed, only_a, only_b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('a', nargs='?')
    ap.add_argument('b', nargs='?')
    ap.add_argument('--sim-vs-real', action='store_true')
    args = ap.parse_args()

    if args.sim_vs_real:
        for a, b in SIM_VS_REAL:
            compare(a, b)
        return 0

    if not (args.a and args.b):
        ap.error('give two side/mode profiles or launch files, or --sim-vs-real')
    compare(args.a, args.b)
    return 0


if __name__ == '__main__':
    sys.exit(main())
