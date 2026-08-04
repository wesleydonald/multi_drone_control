#!/usr/bin/env python3
"""
check_geometry.py — does the controller's geometry match the world it is flying? (F9)

The planner sizes every drone's tension feedforward from `cable_len`, `attach_radius`,
`load_mass` and `num_drones`. If any of them disagrees with the world, EVERY reference
is wrong, and it does not look like a parameter bug -- it looks like a controller that
cannot fly. This project has already lost a week to exactly that: the planner was
hard-coded for 0.6 m cables while the world had 1.0 m ones, which placed every drone
reference ~0.4 m inside the physical cable sphere.

Deliberately NOT a second config file. The launch files' `DeclareLaunchArgument`
defaults are the authority for the controller, and the world SDF is the authority for
the physics; this tool reads BOTH and compares them. Adding a third YAML to describe
them would just create a new thing to drift.

    tools/check_geometry.py simulation_assets/three_rigid_ground.sdf
    tools/check_geometry.py simulation_assets/three_rigid_ground.sdf \
        --launch src/controller_quad_load/launch/mpc_quad_load_launch.py
    tools/check_geometry.py --all      # every world, geometry summary

Exit code 1 if a comparison was requested and something disagrees.
"""
import argparse
import ast
import math
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

REPO = pathlib.Path(__file__).resolve().parent.parent
TOL = 1e-3


def _pose(elem):
    p = elem.find('pose')
    if p is None or not p.text:
        return [0.0] * 6
    vals = [float(v) for v in p.text.split()]
    return (vals + [0.0] * 6)[:6]


def _iter_models(elem):
    for m in elem.findall('model'):
        yield m
        yield from _iter_models(m)


def world_geometry(sdf_path):
    """Extract the geometry the controller cares about from a world SDF."""
    root = ET.parse(sdf_path).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f"{sdf_path}: no <world> element")

    g = {'world': pathlib.Path(sdf_path).name}

    # Drone count: the includes name models/x3_droneN.sdf
    text = pathlib.Path(sdf_path).read_text()
    drones = sorted(set(re.findall(r'models/(x3_drone\d+)(?:_magnet)?\.sdf', text)))
    g['num_drones'] = len(drones)

    # Payload mass, and the payload origin. The origin matters: `attach_z` is defined
    # as the attach height ABOVE THE LOAD CoG, while the stub_pay poses are expressed
    # in the parent lift_system frame. Forgetting to subtract the payload z makes an
    # elevated world (payload at 0.6) look like attach_z=0.625 and reports a mismatch
    # that is not there -- a validator that cries wolf gets ignored.
    payload_z = 0.0
    for m in _iter_models(world):
        if m.get('name') == 'payload':
            payload_z = _pose(m)[2]
            body = m.find("link[@name='body']")
            if body is not None:
                mass = body.find('inertial/mass')
                if mass is not None:
                    g['load_mass'] = float(mass.text)

    # Attach ring: stub_pay_N links carry the body-frame attach points
    radii, zs = [], []
    for m in _iter_models(world):
        for link in m.findall('link'):
            if (link.get('name') or '').startswith('stub_pay_'):
                x, y, z = _pose(link)[:3]
                radii.append(math.hypot(x, y))
                zs.append(z)
    if radii:
        g['attach_radius'] = sum(radii) / len(radii)
        g['attach_z'] = sum(zs) / len(zs) - payload_z   # above the load CoG
        g['attach_points'] = len(radii)
        if max(radii) - min(radii) > TOL:
            g['_warn_ring'] = f'attach radii are not uniform: {[round(r,4) for r in radii]}'

    # Cable length: the tether rod cylinders
    lengths = []
    for m in _iter_models(world):
        if (m.get('name') or '').startswith('tether'):
            for cyl in m.iter('cylinder'):
                ln = cyl.find('length')
                if ln is not None:
                    lengths.append(float(ln.text))
    if lengths:
        g['cable_len'] = sum(lengths) / len(lengths)
        if max(lengths) - min(lengths) > TOL:
            g['_warn_cable'] = f'rod lengths differ: {sorted(set(lengths))}'

    return g


def planner_defaults():
    """Module-level defaults in controller_load_mpc/params.py, read without importing
    ROS. These apply whenever a launch does not declare the parameter -- the silent
    case, and the one most likely to be stale."""
    src = REPO / 'src/controller_load_mpc/controller_load_mpc/params.py'
    out = {}
    if not src.exists():
        return out
    names = {'CABLE_LEN': 'cable_len', 'ATTACH_RADIUS': 'attach_radius',
             'ATTACH_Z': 'attach_z', 'LOAD_MASS': 'load_mass', 'N_DRONES': 'num_drones'}
    tree = ast.parse(src.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            t = getattr(node.targets[0], 'id', None)
            if t in names and isinstance(node.value, ast.Constant):
                out[names[t]] = float(node.value.value)
    return out


def launch_defaults(launch_path):
    """DeclareLaunchArgument defaults from a launch file, without executing it."""
    tree = ast.parse(pathlib.Path(launch_path).read_text())
    out = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'):
            if not node.args:
                continue
            name = getattr(node.args[0], 'value', None)
            default = None
            for kw in node.keywords:
                if kw.arg == 'default_value':
                    default = getattr(kw.value, 'value', None)
            if isinstance(name, str):
                out[name] = default
    return out


COMPARED = ['cable_len', 'attach_radius', 'load_mass', 'attach_z']
INFO_ONLY = ['num_drones']   # passed per run (num_drones:=3), not a fixed default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world', nargs='?')
    ap.add_argument('--launch')
    ap.add_argument('--all', action='store_true')
    args = ap.parse_args()

    if args.all:
        for sdf in sorted((REPO / 'simulation_assets').glob('*.sdf')):
            try:
                g = world_geometry(sdf)
            except Exception as e:
                print(f"{sdf.name}: could not parse ({e})")
                continue
            bits = [f"{k}={g[k]:.3f}" if isinstance(g[k], float) else f"{k}={g[k]}"
                    for k in COMPARED if k in g]
            print(f"{sdf.name:32s} " + "  ".join(bits))
            for w in [v for k, v in g.items() if k.startswith('_warn')]:
                print(f"{'':32s} WARN {w}")
        return 0

    if not args.world:
        ap.error('give a world .sdf, or --all')

    g = world_geometry(args.world)
    print(f"── world geometry: {g['world']}")
    for k in COMPARED:
        if k in g:
            v = g[k]
            print(f"   {k:16s} {v:.4f}" if isinstance(v, float) else f"   {k:16s} {v}")
    for w in [v for k, v in g.items() if k.startswith('_warn')]:
        print(f"   WARN {w}")

    if not args.launch:
        print("\n   (pass --launch <launch.py> to compare against the controller)")
        return 0

    d = launch_defaults(args.launch)
    nd = planner_defaults()
    print(f"\n── vs controller config: {pathlib.Path(args.launch).name}")
    bad = 0
    for k in COMPARED:
        if k not in g:
            continue
        wv = float(g[k])
        if k in d and d[k] is not None:
            try:
                cv, src = float(d[k]), 'launch'
            except (TypeError, ValueError):
                continue
        elif k in nd:
            cv, src = nd[k], 'params.py'      # the silent path: no launch arg declared
        else:
            print(f"   {k:16s} world={wv:<9.4f} controller=<unknown>")
            continue
        flag = '** MISMATCH **' if abs(cv - wv) > TOL else 'ok'
        bad += flag != 'ok'
        print(f"   {k:16s} world={wv:<9.4f} {src:>9s}={cv:<9.4f} {flag}")

    for k in INFO_ONLY:
        if k in g:
            dv = d.get(k) or nd.get(k)
            print(f"   {k:16s} world={g[k]:<9} (launch default {dv}; "
                  f"pass {k}:={g[k]} explicitly)")

    print()
    if bad:
        print(f"!! {bad} mismatch(es) — every drone's reference is sized off these.")
        return 1
    print("   geometry agrees.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
