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
        --launch src/bringup/launch/sim_control_launch.py --mode mpc
    tools/check_geometry.py --all      # every world, geometry summary
    tools/check_geometry.py --config configs/experiments/<name>.yaml   # world vs launch
                                       # defaults + the config's launch args

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

    # Drone count: the includes name models/x3_droneN.sdf (rig twin: x3_rig_droneN.sdf)
    text = pathlib.Path(sdf_path).read_text()
    drones = sorted(set(re.findall(r'models/(x3_(?:rig_)?drone\d+)(?:_magnet)?\.sdf', text)))
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
                # the OCP bakes the load inertia into its compiled solver, so a world
                # whose payload changed shape must change params.py too
                for tag, key in (('ixx', 'load_ixx'), ('izz', 'load_izz')):
                    el = body.find(f'inertial/inertia/{tag}')
                    if el is not None:
                        g[key] = float(el.text)

    # Attach ring: stub_pay_N links carry the body-frame attach points
    radii, zs, xs, ys = [], [], [], []
    for m in _iter_models(world):
        for link in m.findall('link'):
            if (link.get('name') or '').startswith('stub_pay_'):
                x, y, z = _pose(link)[:3]
                radii.append(math.hypot(x, y))
                zs.append(z)
                xs.append(x)
                ys.append(y)
    if radii:
        g['attach_azimuths'] = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 1)
                                      for x, y in zip(xs, ys))
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

    g.update(_drone_and_rod_geometry(sdf_path, world))
    return g


def _link_mass(link):
    m = link.find('inertial/mass')
    return float(m.text) if m is not None else 0.0


def _drone_and_rod_geometry(sdf_path, world):
    """Drone airframe mass (all links of each included model file), the rod mass, and
    the pivot: how far below the drone centre the rod's top end sits at spawn. The rod
    end is taken from the rod pose and length, so it holds whichever way the drone-end
    joint is posed; a detachable joint posed elsewhere is flagged."""
    out = {}
    base = pathlib.Path(sdf_path).parent
    lift = next((m for m in world.findall('model') if m.get('name') == 'lift_system'), None)
    if lift is None:
        return out
    drones, masses = {}, []
    for inc in lift.findall('include'):
        uri, name = inc.findtext('uri', ''), inc.findtext('name', '')
        mm = re.fullmatch(r'x3_drone(\d+)', name)
        if not mm:
            continue
        drones[int(mm.group(1))] = _pose(inc)
        mf = base / uri
        if mf.exists():
            model = ET.parse(mf).getroot().find('model')
            masses.append(sum(_link_mass(lk) for lk in model.iter('link')))
    if masses:
        out['drone_airframe_mass'] = sum(masses) / len(masses)
        if max(masses) - min(masses) > TOL:
            out['_warn_drone_mass'] = f'drone models differ in mass: {masses}'

    rods = {}
    for m in lift.findall('model'):
        mm = re.fullmatch(r'tether_(\d+)', m.get('name') or '')
        rod = m.find("link[@name='rod']")
        if mm and rod is not None:
            rods[int(mm.group(1))] = rod
    for lk in lift.findall('link'):
        mm = re.fullmatch(r'tether_(\d+)', lk.get('name') or '')
        if mm:
            rods[int(mm.group(1))] = lk
    pivots, rod_masses = [], []
    for i, rod in rods.items():
        cyl = rod.find('visual/geometry/cylinder/length')
        if cyl is None or i not in drones:
            continue
        L = float(cyl.text)
        x, y, z, _, pitch, yaw = _pose(rod)
        top = (x + 0.5 * L * math.sin(pitch) * math.cos(yaw),
               y + 0.5 * L * math.sin(pitch) * math.sin(yaw),
               z + 0.5 * L * math.cos(pitch))
        d = drones[i]
        if math.hypot(d[0] - top[0], d[1] - top[1]) > TOL:
            out['_warn_pivot'] = f'rod {i} top is not under drone {i}'
        pivots.append(d[2] - top[2] + 0.0)
        rod_masses.append(_link_mass(rod))
        j = lift.find(f"joint[@name='tether_{i}_to_drone{i}']")
        jp = j.find('pose') if j is not None else None
        if j is not None and (jp is None or jp.get('relative_to') is None):
            jz = _pose(j)[2]    # child (base_link) frame
            if abs(jz + pivots[-1]) > TOL:
                out['_warn_pivot'] = (f'tether_{i}_to_drone{i} is posed at z={jz:+.3f} in '
                                      f'base_link but the rod ends {pivots[-1]:.3f} below it')
    if pivots:
        out['pivot_dz'] = sum(pivots) / len(pivots)
        out['rod_mass'] = sum(rod_masses) / len(rod_masses)
        if max(pivots) - min(pivots) > TOL:
            out['_warn_pivot'] = f'pivot offsets differ: {[round(p, 4) for p in pivots]}'
        if 'drone_airframe_mass' in out:
            # as the planner counts it: airframe + rod + magnet (rig: 0.475 + 0.075)
            out['drone_mass'] = out['drone_airframe_mass'] + out['rod_mass']
    return out


def planner_defaults():
    """Module-level defaults in mpc_planner/params.py, read without importing
    ROS. These apply whenever a launch does not declare the parameter -- the silent
    case, and the one most likely to be stale."""
    src = REPO / 'src/mpc_planner/mpc_planner/params.py'
    out = {}
    if not src.exists():
        return out
    names = {'CABLE_LEN': 'cable_len', 'ATTACH_RADIUS': 'attach_radius',
             'ATTACH_Z': 'attach_z', 'LOAD_MASS': 'load_mass', 'N_DRONES': 'num_drones',
             'LOAD_IXX': 'load_ixx', 'LOAD_IZZ': 'load_izz'}
    tree = ast.parse(src.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            t = getattr(node.targets[0], 'id', None)
            if t in names and isinstance(node.value, ast.Constant):
                out[names[t]] = float(node.value.value)
    return out


# the consolidated control launches take their defaults from bringup/config (profiles.py)
CONTROL_LAUNCHES = {'sim_control_launch.py': 'sim', 'real_control_launch.py': 'real'}


def launch_defaults(launch_path, mode=None):
    """A launch file's argument defaults without executing it: the profile of `mode` for
    sim_control/real_control_launch.py, else its DeclareLaunchArgument defaults."""
    side = CONTROL_LAUNCHES.get(pathlib.Path(launch_path).name)
    if side:
        sys.path.insert(0, str(REPO / 'src' / 'bringup'))
        from bringup.profiles import DEFAULT_MODE, profile
        return profile(side, mode or DEFAULT_MODE, cdir=str(REPO / 'src' / 'bringup' / 'config'))
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


COMPARED = ['cable_len', 'attach_radius', 'load_mass', 'attach_z', 'load_ixx', 'load_izz']
INFO_ONLY = ['num_drones']   # passed per run (num_drones:=3), not a fixed default
# Compared only where the launch/config declares them. drone_mass is airframe + rod
# (the legacy worlds' 1 g rod sits inside its 5 g band); the launch's pivot_offset_z is
# body-frame z, so the world's pivot_dz 0.04 below the centre is -0.04 there.
DECLARED = {'drone_mass': ('drone_mass', 1.0, 0.005),
            'pivot_offset_z': ('pivot_dz', -1.0, TOL)}


def _tol(k, wv):
    # inertias are ~1e-2, so they get a relative tolerance instead of TOL
    return 0.05 * abs(wv) if k.startswith('load_i') else TOL


def config_mismatches(world, launch_path=None, overrides=None):
    """[(key, world value, controller value)] for every geometry value the controller
    would fly differently from `world`. Controller values: `overrides` (an experiment
    config's launch args) over the launch file's defaults over params.py. For
    run_experiment.py to refuse a config before it spends a run."""
    g = world if isinstance(world, dict) else world_geometry(world)
    d = dict(launch_defaults(launch_path, (overrides or {}).get('mode'))) if launch_path else {}
    d.update({k: v for k, v in (overrides or {}).items() if v is not None})
    nd = planner_defaults()
    bad = []
    for k in COMPARED:
        if k not in g:
            continue
        try:
            cv = float(d[k]) if d.get(k) is not None else nd.get(k)
        except (TypeError, ValueError):
            continue
        if cv is not None and abs(cv - g[k]) > _tol(k, g[k]):
            bad.append((k, g[k], cv))
    for k, (wk, sign, tol) in DECLARED.items():
        if wk not in g or d.get(k) is None:
            continue
        try:
            cv = float(d[k])
        except (TypeError, ValueError):
            continue
        if abs(cv - sign * g[wk]) > tol:
            bad.append((k, sign * g[wk] + 0.0, cv))
    return bad


def _load_config(path):
    """world path, launch path and launch args of a configs/experiments/*.yaml."""
    import yaml
    cfg = yaml.safe_load(pathlib.Path(path).read_text())
    world = pathlib.Path(cfg['world'])
    if not world.is_absolute():
        world = REPO / 'simulation_assets' / world
    launch = cfg.get('launch', {})
    lp = (REPO / 'src' / launch.get('package', 'bringup') / 'launch'
          / launch['file']) if launch.get('file') else None
    return world, lp, launch.get('args') or {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world', nargs='?')
    ap.add_argument('--launch')
    ap.add_argument('--mode', help='mode of sim_control/real_control_launch.py (default mpc)')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--config', help='experiment YAML: its world vs its launch + args')
    args = ap.parse_args()

    if args.config:
        world, lp, over = _load_config(args.config)
        bad = config_mismatches(world, lp, over)
        for k, wv, cv in bad:
            print(f"   {k:16s} world={wv:<9.4f} config={cv:<9.4f} ** MISMATCH **")
        print(f"{pathlib.Path(args.config).name} vs {world.name}: "
              + (f"{len(bad)} mismatch(es)" if bad else "geometry agrees"))
        return 1 if bad else 0

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
    for k in COMPARED + ['pivot_dz', 'rod_mass', 'drone_airframe_mass', 'drone_mass']:
        if k in g:
            v = g[k]
            print(f"   {k:16s} {v:.4f}" if isinstance(v, float) else f"   {k:16s} {v}")
    for w in [v for k, v in g.items() if k.startswith('_warn')]:
        print(f"   WARN {w}")

    if not args.launch:
        print("\n   (pass --launch <launch.py> to compare against the controller)")
        return 0

    d = launch_defaults(args.launch, args.mode)
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
        flag = '** MISMATCH **' if abs(cv - wv) > _tol(k, wv) else 'ok'
        bad += flag != 'ok'
        print(f"   {k:16s} world={wv:<9.4f} {src:>9s}={cv:<9.4f} {flag}")
    for k, (wk, sign, tol) in DECLARED.items():
        if wk not in g or d.get(k) is None:
            continue
        try:
            cv, wv = float(d[k]), sign * g[wk] + 0.0
        except (TypeError, ValueError):
            continue
        flag = '** MISMATCH **' if abs(cv - wv) > tol else 'ok'
        bad += flag != 'ok'
        print(f"   {k:16s} world={wv:<9.4f} {'launch':>9s}={cv:<9.4f} {flag}")

    # attach azimuths: the launch's attach_azimuths_deg ('' = even ring) vs the stubs
    if 'attach_azimuths' in g:
        spec = d.get('attach_azimuths_deg')
        n_w = len(g['attach_azimuths'])
        if spec:
            want = sorted(round(float(v) % 360.0, 1) for v in str(spec).split(',') if v.strip())
        else:
            want = sorted(round(360.0 * k / n_w, 1) for k in range(n_w))
        have = g['attach_azimuths']
        ok = (len(want) == len(have)
              and all(abs(((a - b + 180) % 360) - 180) <= 1.0 for a, b in zip(want, have)))
        flag = 'ok' if ok else '** MISMATCH **'
        bad += flag != 'ok'
        print(f"   {'attach_azimuths':16s} world={have} launch={want} {flag}")

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
