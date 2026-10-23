#!/usr/bin/env python3
"""
tools/limit_replay.py -- the position limit's LAND check replayed over the rig flights, offline and read-only (card
docs/experiments/2026-10-22_position_limit.md, LIM-0; critic M4).

    tools/limit_replay.py                                       # the 21 and 22 Oct rig flights
    tools/limit_replay.py results/rig/2026-10-22 --limit 1.75 --reach 0.725

Each flight's quad and ring poses (<flight>_run/logs/run.csv, the bag on a 50 Hz grid) go through the fleet manager's
own test (utility_objects.position_limit.breach, POS_LIMIT_SAMPLES in a row, airborne: z over min(its height at ARM +
0.10, 0.25) m, the fleet flying: TAKEOFF to the first DISARM in events.csv). Twice: as flown, and with each quad's
horizontal offset from the ring scaled to the 1 m formation (x reach / the flight's reach, reach = attach_radius +
cable_len cos(cable_elev_deg) from its planner's params.json, defaults 0.225 m and 60 deg), the ring's path as flown.
The scaling stops at the flight's first release (RELEASE, DETECTED, DROP or DETACH in events.csv): a freed quad is
not on the formation, so from there on both columns are as flown. Prints per flight the largest |x| or |y| of any quad
(as flown and scaled), the scaled margin to the LAND line and to the reference clamp, and whether and when it would
have landed the fleet. Flights without a run.csv or poses are skipped. Exit 1 if any would have.

--path-scale k shrinks the ring's path about its hover point (the reference where it first reaches its height): the
ring's reference scaled by k about that point and the ring moved with it, its tracking error as flown, before the
first release (Wesley 22 Oct: the rig's figure-8 and lines at 0.75 m, from the 1.0 m flown). It scales every path
of the flights given, circles and spins too: read the rows of the paths it applies to.

On the 50 Hz grid three samples span 60 ms (the manager's three mocap poses: 25-35 ms), so a trip here is a trip
there; a shorter excursion shows in the max columns only.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'src', 'utility_objects'))
from utility_objects.position_limit import POS_LIMIT_REF_MARGIN_M, POS_LIMIT_SAMPLES, breach  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DAYS = [os.path.join(ROOT, 'results', 'rig', d) for d in ('2026-10-21', '2026-10-22')]
REACH_1M = 0.225 + 1.0 * math.cos(math.radians(60.0))     # the rig ring on 1 m cables at 60 deg: 0.725 m
FC_AIRBORNE_M, REST_Z_MAX_M = 0.10, 0.25                   # fleet_manager_node's


def reach_of(params):
    """attach_radius + cable_len cos(cable_elev_deg) of a planner's params (defaults 0.225 m, 60 deg)."""
    def num(k, default):
        try:
            return float(params.get(k))
        except (TypeError, ValueError):
            return default
    cable = num('cable_len', None)
    if cable is None:
        return None
    return num('attach_radius', 0.225) + cable * math.cos(math.radians(num('cable_elev_deg', 60.0)))


def planner_params(flight):
    """The flight's planner params.json beside its bag (<flight>_logs), {} when there is none."""
    for path in sorted(glob.glob(os.path.join(flight + '_logs', 'logs', '*', '*', 'params.json'))):
        if os.sep + 'tracker' + os.sep not in path:
            with open(path) as fh:
                return json.load(fh)
    return {}


def read_run(path):
    """(t, ring xy (T, 2), {drone: xyz (T, 3)}, ring reference xyz (T, 3)) from a run.csv; None when it has no ring or
    no quad poses."""
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows or 'payload_x' not in rows[0]:
        return None

    def col(k):
        return np.array([float(r[k]) if r.get(k) not in (None, '') else np.nan for r in rows])
    drones = sorted({int(k[1:k.index('_')]) for k in rows[0] if k.startswith('d') and k.endswith('_x')
                     and k[1:k.index('_')].isdigit()})
    poses = {d: np.column_stack([col(f'd{d}_{a}') for a in 'xyz']) for d in drones}
    poses = {d: p for d, p in poses.items() if np.isfinite(p).all(axis=1).any()}
    ring = np.column_stack([col('payload_x'), col('payload_y')])
    if not poses or not np.isfinite(ring).all(axis=1).any():
        return None
    ref = np.column_stack([col(f'payload_ref_{a}') if f'payload_ref_{a}' in rows[0] else col('nan') for a in 'xyz'])
    return col('t'), ring, poses, ref


RELEASES = ('RELEASE', 'DETECTED', 'DROP', 'DETACH')


def flying_window(events_path):
    """(t TAKEOFF, t first DISARM after it, t first release after it) from events.csv; the whole flight and no release
    when there is none."""
    t0, t1, rel = -math.inf, math.inf, math.inf
    if os.path.exists(events_path):
        with open(events_path) as fh:
            for r in csv.DictReader(fh):
                t, ev = float(r['sim_time']), r['event'].strip().upper()
                if ev == 'TAKEOFF' and t0 == -math.inf:
                    t0 = t
                elif t0 > -math.inf and t > t0 and ev in ('DISARM', 'FLEET_ABORT') and t1 == math.inf:
                    t1 = t
                elif t0 > -math.inf and ev in RELEASES and rel == math.inf:
                    rel = t
    return t0, t1, rel


def path_shift(ref, k):
    """(T, 2): how far the ring moves when its path is scaled by k about its hover point (zeros: k 1, no reference)."""
    shift = np.zeros((len(ref), 2))
    z = ref[:, 2]
    if k == 1.0 or not np.isfinite(z).any():
        return shift
    i = int(np.argmax(np.nan_to_num(z, nan=-1.0) >= np.nanmax(z) - 0.05))
    ok = np.isfinite(ref[:, :2]).all(axis=1)
    shift[ok] = (k - 1.0) * (ref[ok, :2] - ref[i, :2])
    return shift


def check(t, ring, poses, scale, limit, window, shift=None):
    """{drone: (largest |x| or |y| while airborne, the time it first trips or None)} with each quad's horizontal offset
    from the ring times scale and the ring moved by shift, until the first release (window[2]); the manager's test:
    POS_LIMIT_SAMPLES in a row, airborne, the fleet flying."""
    out = {}
    before = (t < window[2])[:, None]
    k_of_t = np.where(before, scale, 1.0)
    ring_s = ring + (np.where(before, shift, 0.0) if shift is not None else 0.0)
    for d, p in poses.items():
        ok = np.isfinite(p).all(axis=1) & np.isfinite(ring).all(axis=1)
        if not ok.any():
            continue
        xy = ring_s + k_of_t * (p[:, :2] - ring)
        rest = float(p[np.argmax(ok), 2])                 # its height at the first pose (ARM is at the start)
        floor = min(REST_Z_MAX_M, rest + FC_AIRBORNE_M)
        up = ok & (p[:, 2] > floor) & (t >= window[0]) & (t < window[1])
        peak = float(np.max(np.abs(xy[up]))) if up.any() else float('nan')
        k, trip = 0, None
        for i in np.flatnonzero(ok):
            k = k + 1 if up[i] and breach(xy[i], limit) else 0
            if k == POS_LIMIT_SAMPLES and trip is None:
                trip = float(t[i])
        out[d] = (peak, trip)
    return out


def flights(dirs):
    """[(flight path, run.csv or None)] of every flight (a <name>.log) in the given day folders."""
    out = []
    for day in dirs:
        names = sorted(f[:-4] for f in os.listdir(day) if f.endswith('.log'))
        for n in names:
            run = os.path.join(day, n + '_run', 'logs', 'run.csv')
            out.append((os.path.join(day, n), run if os.path.exists(run) else None))
    return out


def replay(flight, run, limit, reach_1m, path_scale=1.0):
    """One flight's row (dict), or None when it has no poses."""
    data = read_run(run)
    if data is None:
        return None
    t, ring, poses, ref = data
    reach = reach_of(planner_params(flight)) or reach_of({'cable_len': 0.55})
    window = flying_window(os.path.join(os.path.dirname(run), 'events.csv'))
    flown = check(t, ring, poses, 1.0, limit, window)
    shift = path_shift(ref, path_scale)
    scaled = check(t, ring, poses, reach_1m / reach, limit, window, shift)
    up = np.isfinite(ring).all(axis=1) & (t >= window[0]) & (t < window[1])
    ring = ring + np.where((t < window[2])[:, None], shift, 0.0)
    peak = max((v[0] for v in scaled.values() if np.isfinite(v[0])), default=float('nan'))
    who = max(scaled, key=lambda d: scaled[d][0] if np.isfinite(scaled[d][0]) else -1.0)

    def first(res):
        trips = [(v[1], d) for d, v in res.items() if v[1] is not None]
        return min(trips) if trips else None
    return {'flight': os.path.basename(flight), 'reach': reach, 'ring': float(np.max(np.abs(ring[up]))) if up.any()
            else float('nan'), 'flown': max((v[0] for v in flown.values() if np.isfinite(v[0])), default=float('nan')),
            'scaled': peak, 'who': who, 'trip_flown': first(flown), 'trip_scaled': first(scaled),
            'm_land': limit - peak, 'm_clamp': limit - POS_LIMIT_REF_MARGIN_M - peak, 'release': window[2]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('days', nargs='*', default=DAYS, help='rig day folders (default: 21 and 22 Oct)')
    ap.add_argument('--limit', type=float, default=1.75, help='pos_limit_m (m)')
    ap.add_argument('--reach', type=float, default=REACH_1M, help='the 1 m formation reach to scale to (m)')
    ap.add_argument('--path-scale', type=float, default=1.0, help='the ring path scaled about its hover point')
    a = ap.parse_args(argv)
    clamp = a.limit - POS_LIMIT_REF_MARGIN_M
    print(f'limit +-{a.limit:.2f} m (reference clamp {clamp:.2f}), {POS_LIMIT_SAMPLES} samples in a row, '
          f'offsets scaled to a reach of {a.reach:.3f} m'
          + (f', paths x {a.path_scale:g}' if a.path_scale != 1.0 else ''))
    print(f'{"flight":<26}{"reach":>6}{"ring":>6}{"flown":>7}{"1 m":>7}{"quad":>5}{"to LAND":>9}{"to clamp":>9}  '
          f'{"trips (flown)":<15}{"trips (1 m)":<15}release')
    skipped, tripped = [], 0
    for flight, run in flights(a.days):
        row = replay(flight, run, a.limit, a.reach, a.path_scale) if run else None
        if row is None:
            skipped.append(os.path.basename(flight))
            continue
        tf = 'no' if row['trip_flown'] is None else f'd{row["trip_flown"][1]} {row["trip_flown"][0]:.1f} s'
        ts = 'no' if row['trip_scaled'] is None else f'd{row["trip_scaled"][1]} {row["trip_scaled"][0]:.1f} s'
        tripped += row['trip_scaled'] is not None
        day = os.path.basename(os.path.dirname(flight))[5:]
        print(f'{day + " " + row["flight"]:<26}{row["reach"]:>6.3f}{row["ring"]:>6.2f}{row["flown"]:>7.3f}'
              f'{row["scaled"]:>7.3f}{"d" + str(row["who"]):>5}{row["m_land"]:>+9.3f}{row["m_clamp"]:>+9.3f}  '
              f'{tf:<15}{ts:<15}' + ('-' if row['release'] == math.inf else f'{row["release"]:.1f} s'))
    if skipped:
        print(f'skipped (no run.csv or no poses): {", ".join(skipped)}')
    return 1 if tripped else 0


if __name__ == '__main__':
    sys.exit(main())
