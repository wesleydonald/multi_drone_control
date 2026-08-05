#!/usr/bin/env python3
"""
tools/check_thresholds.py — hold a finished run to the gate's bars (§9.5)

    ./tools/check_thresholds.py results/2026-08-05/R0047_sil_carry_hover_n3 \
        --profile sil_smoke
    ./tools/check_thresholds.py R0047 --profile sil_smoke --list

Thresholds live in `configs/gate_thresholds.yaml`, set from a measured baseline rather
than guessed. The numbers checked come from `tools/metrics.py` -- this file only
compares, so a bar and the figure beside it can never be reading different quantities.

A metric a profile names but the run does not contain is a FAILURE, not a skip. The
harnesses omit a metric exactly when the source cannot observe it, so a vanished metric
means the run is not the kind of run the profile was written for -- which is precisely
what a gate should catch. Exit code is nonzero if any check fails.
"""
import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import metrics as M                    # noqa: E402
from run_dir import resolve            # noqa: E402

THRESHOLDS = os.path.join(REPO, 'configs', 'gate_thresholds.yaml')
AGG = {'max': max, 'min': min, 'mean': lambda v: sum(v) / len(v)}


def read_metric(summary, path):
    """Address summarise_run's output by dotted path, aggregating across drones.

    `per_drone.max.peak_track_err_m` -> the worst drone's peak error.
    Returns None if any step of the path is absent."""
    segs = path.split('.')
    agg = next((s for s in segs if s in AGG), None)
    if agg:
        segs.remove(agg)
    cur = summary
    for s in segs:
        try:
            cur = [c[s] for c in cur] if isinstance(cur, list) else cur[s]
        except (KeyError, TypeError, IndexError):
            return None
    if agg:
        vals = [float(v) for v in (cur if isinstance(cur, list) else [cur])
                if v is not None and math.isfinite(float(v))]
        if not vals:
            return None
        cur = AGG[agg](vals)
    try:
        v = float(cur)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def load_profile(name, path=THRESHOLDS):
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    profiles = doc.get('profiles') or {}
    if name not in profiles:
        raise SystemExit(f'no profile {name!r} in {path}; have: '
                         + ', '.join(sorted(profiles)))
    return profiles[name]


def check(run_dir, profile_name, quiet=False, thresholds=THRESHOLDS):
    """Returns (ok, [(metric, ok, detail)])."""
    profile = load_profile(profile_name, thresholds)
    summary = M.summarise_run(run_dir)
    if 'error' in summary:
        return False, [(profile_name, False, summary['error'])]

    results = []
    for c in profile.get('checks') or []:
        name = c['metric']
        v = read_metric(summary, name)
        lo, hi = c.get('min'), c.get('max')
        if v is None:
            results.append((name, False, 'not present in this run'))
            continue
        ok = (lo is None or v >= lo) and (hi is None or v <= hi)
        bar = ' and '.join(filter(None, [f'>={lo}' if lo is not None else '',
                                         f'<={hi}' if hi is not None else '']))
        results.append((name, ok, f'{v:.4g} ({bar})'))

    ok_all = all(r[1] for r in results) and bool(results)
    if not quiet:
        rid = summary.get('run_id') or os.path.basename(run_dir)
        print(f'    thresholds [{profile_name}] on {rid}:')
        for name, ok, detail in results:
            print(f'      [{"PASS" if ok else "FAIL"}] {name:42s} {detail}')
        if not results:
            print(f'      !! profile {profile_name!r} declares no checks')
    return ok_all, results


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', help='run id or run directory')
    ap.add_argument('--profile', required=True)
    ap.add_argument('--thresholds', default=THRESHOLDS)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()
    ok, _ = check(resolve(args.run), args.profile, quiet=args.quiet,
                  thresholds=args.thresholds)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
