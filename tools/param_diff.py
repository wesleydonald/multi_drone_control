#!/usr/bin/env python3
"""
param_diff.py — what actually differs between two launch configurations? (F9)

Answers "which parameters do I need to change between simulation and the real rig?"
without reading two 400-line launch files side by side and hoping you spotted
everything.

It reads `DeclareLaunchArgument` defaults straight out of the launch files, so there
is no second copy of the configuration to drift. That matters: the whole finding this
tool closes is that sim/real divergence was implicit, and a YAML mirror of the same
values would just be one more thing to forget to update.

    tools/param_diff.py mpc_quad_load_launch.py real_control_launch.py
    tools/param_diff.py dissipative_launch.py real_dissipative_launch.py
    tools/param_diff.py --sim-vs-real        # the pairs that matter, all at once

Three categories are reported, and the third is the dangerous one:

  CHANGED       declared in both, different defaults
  ONLY IN A/B   declared in one only -- the other falls back to the NODE default,
                silently. This is how real_dissipative_launch.py ended up with no
                kT block at all while the sim launches carried nine kT parameters.
"""
import argparse
import ast
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
LAUNCH_DIR = REPO / 'src/controller_quad_load/launch'

# The pairs worth comparing routinely.
SIM_VS_REAL = [
    ('mpc_quad_load_launch.py', 'real_control_launch.py'),
    ('dissipative_launch.py', 'real_dissipative_launch.py'),
]


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


def compare(a_path, b_path, quiet=False):
    a, b = launch_args(a_path), launch_args(b_path)
    an, bn = pathlib.Path(a_path).name, pathlib.Path(b_path).name

    changed = {k: (a[k], b[k]) for k in sorted(a.keys() & b.keys()) if a[k] != b[k]}
    only_a = sorted(a.keys() - b.keys())
    only_b = sorted(b.keys() - a.keys())

    print(f"\n══ {an}  vs  {bn} ══")
    print(f"   {len(a)} vs {len(b)} declared arguments\n")

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
            pa, pb = LAUNCH_DIR / a, LAUNCH_DIR / b
            if pa.exists() and pb.exists():
                compare(pa, pb)
        print("\nRemember: 'ONLY IN' means the other side is running on whatever the")
        print("node declares. That is not necessarily wrong -- but it is not visible")
        print("in the launch file, so check it deliberately before a flight.")
        return 0

    if not (args.a and args.b):
        ap.error('give two launch files, or --sim-vs-real')
    compare(resolve(args.a), resolve(args.b))
    return 0


if __name__ == '__main__':
    sys.exit(main())
