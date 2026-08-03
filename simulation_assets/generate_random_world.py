#!/usr/bin/env python3
"""
generate_random_world.py
------------------------
Generate a RANDOMLY PLACED N-drone rigid-cable lift world, for testing how the
stack copes with a rig that was not laid out perfectly world-aligned.

generate_rigid_world.py always spawns the payload at the origin with zero yaw,
drone i exactly on attach azimuth 2*pi*i/n, and every drone facing world +x.
That is the one configuration in which the planner's world-fixed reference ring
and the tracker's yaw-free attitude reference are correct by accident, so a
Gazebo run can never exercise them. This generator randomises:

    * payload YAW            (the big one -- the reference ring and the load
                              attitude reference are both world-fixed)
    * payload xy position    (the load reference latches hover_xy from mocap,
                              so this should be harmless -- worth confirming)
    * per-drone HEADING      (the tracker's attitude reference is a yaw-free
                              tilt, i.e. a commanded heading of world +x)
    * per-drone AZIMUTH      around its own attach point, so the drones are not
                              sitting on the nominal ring slots

What it deliberately does NOT randomise by default: the cable length. Every rod
is exactly --cable-len from its attach point to its drone, because the planner
models one shared cable_len and a mixed-length rig would confound the test.
--radius-jitter breaks that on purpose if you want it.

Drone i is always physically tethered to attach point i, so the correct
drone<->attach mapping is the identity no matter how the drones are placed --
which makes it easy to see when the planner's azimuth matching disagrees.

Usage:
    python3 generate_random_world.py --n 3 --seed 1 --ground-start --detachable \
        --out three_random_seed1.sdf

Every run prints (and writes alongside the SDF) a manifest of the exact
placement, so a run can be reproduced or replayed offline.
"""
import argparse
import json
import math
import os
import random
import sys

from generate_rigid_world import build_world, _fmt      # noqa: F401

# Optional: use the planner's own helpers to predict what it will make of the
# placement. Absent (e.g. running this outside the workspace) we just skip that
# part of the manifest rather than failing to generate a world.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'controller_load_mpc'))
try:
    from controller_load_mpc.geometry import azimuth_slot_assignment
except Exception:                                        # pragma: no cover
    azimuth_slot_assignment = None

DRONE_GROUND_Z = 0.10        # matches generate_rigid_world's --ground-start


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def sample_placement(rng, n, cable_len, elev_deg, attach_radius, attach_z,
                     payload_z, payload_yaw_deg, payload_xy, azimuth_jitter_deg,
                     heading_jitter_deg, radius_jitter, min_separation):
    """One random placement. Retries until the drones are at least
    min_separation apart, so a jittered ring cannot spawn two drones on top of
    each other."""
    for _ in range(500):
        pyaw = math.radians(payload_yaw_deg if payload_yaw_deg is not None
                            else rng.uniform(-180.0, 180.0))
        pr = rng.uniform(0.0, payload_xy)
        pa = rng.uniform(-math.pi, math.pi)
        px, py = pr * math.cos(pa), pr * math.sin(pa)

        attaches, drones, yaws, meta = [], [], [], []
        for i in range(n):
            # attach point i, rotated with the payload -- this is the physical
            # tie-down, and it moves with the load frame.
            th = 2.0 * math.pi * i / n + pyaw
            ax = px + attach_radius * math.cos(th)
            ay = py + attach_radius * math.sin(th)
            az = payload_z + attach_z
            # the drone sits out along its attach azimuth, jittered
            alpha = th + math.radians(rng.uniform(-azimuth_jitter_deg,
                                                  azimuth_jitter_deg))
            L = cable_len * (1.0 + rng.uniform(-radius_jitter, radius_jitter))
            # elevation is FIXED by the rod length and the drone's stand height
            # on a ground start; otherwise it is the requested elevation.
            if elev_deg is None:
                s = (DRONE_GROUND_Z - az) / L
                phi = math.asin(max(-1.0, min(1.0, s)))
            else:
                phi = math.radians(elev_deg)
            dx = ax + L * math.cos(phi) * math.cos(alpha)
            dy = ay + L * math.cos(phi) * math.sin(alpha)
            dz = az + L * math.sin(phi)
            attaches.append((ax, ay, az))
            drones.append((dx, dy, dz))
            yaws.append(math.radians(rng.uniform(-heading_jitter_deg,
                                                 heading_jitter_deg)))
            meta.append({'attach_azimuth_deg': math.degrees(wrap(th)),
                         'spawn_azimuth_deg': math.degrees(wrap(alpha)),
                         'azimuth_jitter_deg': math.degrees(wrap(alpha - th)),
                         'rod_length_m': L})
        ok = all(math.dist(drones[i], drones[j]) >= min_separation
                 for i in range(n) for j in range(i + 1, n))
        if ok:
            for i in range(n):
                meta[i]['heading_deg'] = math.degrees(yaws[i])
            return ({'payload': (px, py, payload_z, pyaw), 'attaches': attaches,
                     'drones': drones, 'drone_yaws': yaws}, meta)
    raise RuntimeError(f'could not place {n} drones at least {min_separation} m '
                       f'apart -- lower --azimuth-jitter or raise --cable-len')


def predict(placement, meta, n):
    """What the CURRENT stack will make of this placement. Purely informational;
    it changes nothing in the world file."""
    out = {}
    px, py, pz, pyaw = placement['payload']
    # The planner's reference ring and load-attitude reference are world-fixed,
    # so it wants the formation rotated onto the nearest world-aligned ring.
    out['payload_yaw_deg'] = math.degrees(pyaw)
    # The ring is n-fold symmetric, so only the payload yaw modulo one slot
    # pitch is actually demanded back (the rest is absorbed by relabelling).
    slot_pitch = 2.0 * math.pi / n
    out['formation_rotation_demanded_deg'] = abs(math.degrees(
        (pyaw + slot_pitch / 2) % slot_pitch - slot_pitch / 2))
    # Each drone's spawn heading is what the tracker will unwind to 0.
    out['heading_unwind_deg'] = [abs(m['heading_deg']) for m in meta]
    if azimuth_slot_assignment is not None:
        drones = [list(d) for d in placement['drones']]
        world = azimuth_slot_assignment(drones, [px, py], n)
        out['planner_slot_to_drone'] = [int(v) for v in world]
        out['correct_slot_to_drone'] = list(range(n))
        out['slot_mapping_wrong'] = out['planner_slot_to_drone'] != list(range(n))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--cable-len', type=float, default=0.5)
    ap.add_argument('--attach-radius', type=float, default=0.08)
    ap.add_argument('--attach-z', type=float, default=0.025)
    ap.add_argument('--payload-z', type=float, default=0.025)
    ap.add_argument('--elev', type=float, default=None,
                    help='cable elevation deg; omit with --ground-start to solve '
                         'it from the drone stand height')
    ap.add_argument('--ground-start', action='store_true',
                    help='drones on the floor (elevation solved per rod), the '
                         'ground-start layout dissipative_launch.py expects')
    ap.add_argument('--payload-yaw', type=float, default=None,
                    help='fix the payload yaw (deg) instead of randomising it')
    ap.add_argument('--payload-xy', type=float, default=0.3,
                    help='max payload offset from the origin (m); 0 pins it')
    ap.add_argument('--azimuth-jitter', type=float, default=25.0,
                    help='max per-drone azimuth offset from its attach ray (deg)')
    ap.add_argument('--heading-jitter', type=float, default=180.0,
                    help='max per-drone spawn heading (deg)')
    ap.add_argument('--radius-jitter', type=float, default=0.0,
                    help='fractional per-drone rod-length spread. NON-ZERO BREAKS '
                         "the planner's single cable_len -- off by default")
    ap.add_argument('--min-separation', type=float, default=0.35,
                    help='minimum drone-drone spawn distance (m)')
    ap.add_argument('--detachable', action='store_true',
                    help='releasable cables (/drone_k/detach), as the dissipative '
                         'stack expects')
    ap.add_argument('--out', type=str, default='three_random.sdf')
    a = ap.parse_args()

    if not a.ground_start and a.elev is None:
        a.elev = 45.0
    if a.ground_start:
        a.elev = None

    rng = random.Random(a.seed)
    placement, meta = sample_placement(
        rng, a.n, a.cable_len, a.elev, a.attach_radius, a.attach_z, a.payload_z,
        a.payload_yaw, a.payload_xy, a.azimuth_jitter, a.heading_jitter,
        a.radius_jitter, a.min_separation)

    sdf = build_world(a.n, placement, detachable=a.detachable)
    out = a.out if os.path.isabs(a.out) else os.path.join(_HERE, a.out)
    with open(out, 'w') as f:
        f.write(sdf)

    pred = predict(placement, meta, a.n)
    manifest = {'seed': a.seed, 'n': a.n, 'cable_len': a.cable_len,
                'ground_start': a.ground_start, 'detachable': a.detachable,
                'payload': {'x': placement['payload'][0],
                            'y': placement['payload'][1],
                            'z': placement['payload'][2],
                            'yaw_deg': math.degrees(placement['payload'][3])},
                'drones': [{'index': i,
                            'x': placement['drones'][i][0],
                            'y': placement['drones'][i][1],
                            'z': placement['drones'][i][2],
                            **meta[i]} for i in range(a.n)],
                'prediction': pred}
    with open(os.path.splitext(out)[0] + '.json', 'w') as f:
        json.dump(manifest, f, indent=2)

    px, py, _, pyaw = placement['payload']
    print(f"\nwrote {out}  (seed {a.seed}, n={a.n}, cable_len={a.cable_len}, "
          f"detachable={a.detachable})")
    print(f"  payload      xy=({px:+.3f}, {py:+.3f})  yaw={math.degrees(pyaw):+7.1f} deg")
    for i in range(a.n):
        dx, dy, dz = placement['drones'][i]
        m = meta[i]
        print(f"  drone {i}      xyz=({dx:+.3f}, {dy:+.3f}, {dz:.3f})  "
              f"heading={m['heading_deg']:+7.1f}  "
              f"azimuth={m['spawn_azimuth_deg']:+7.1f} "
              f"({m['azimuth_jitter_deg']:+.1f} off its attach ray)  "
              f"rod={m['rod_length_m']:.3f} m")
    print("\n  what the CURRENT stack will do with this placement:")
    print(f"    formation rotation demanded at takeoff : "
          f"{pred['formation_rotation_demanded_deg']:.1f} deg")
    print(f"    per-drone heading to unwind            : "
          + ', '.join(f'{h:.0f}' for h in pred['heading_unwind_deg']) + " deg")
    if 'planner_slot_to_drone' in pred:
        print(f"    planner slot->drone                    : "
              f"{pred['planner_slot_to_drone']}"
              + ("   <-- differs from the physical tie-down "
                 "(drone i is tied to attach i)"
                 if pred['slot_mapping_wrong'] else "   (matches the tie-down)"))
    print(f"\n  manifest: {os.path.splitext(out)[0] + '.json'}")


if __name__ == '__main__':
    main()
