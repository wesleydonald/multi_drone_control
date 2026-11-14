"""Compact per-run timing evidence for Te-Jen's Codex (entry 14): parameters, binary, authority/capture, replan and
installation timing split before/after authority, accepted vs installed, rejection kinds.
usage (from cpp_realtime_2026-11-13/): python3 full/evidence.py <run> [<run> ...]"""
import collections
import csv
import sys

import numpy as np
import yaml


def pct(a, q):
    return f'{np.percentile(a, q):.0f}' if len(a) else '-'


KEYS = ('fixed_splice_lookahead_s', 'cooperative_world_refresh_period_s', 'maximum_search_z_m', 'max_swing_angle_rad',
        'spline_time_factor', 'cooperative_tracking_half_x_m', 'swing_envelope_shape', 'target_time_fixed_point_iterations',
        'temporal_refinement_enabled')
for run in sys.argv[1:]:
    d = f'full/runs/{run}'
    cfg = yaml.safe_load(open(f'{d}/backend.yaml'))
    params = cfg.get('/**', cfg).get('ros__parameters', cfg) if isinstance(cfg, dict) else {}
    flat = {}
    def walk(x):
        if isinstance(x, dict):
            for k, v in x.items():
                if k in KEYS:
                    flat[k] = v
                walk(v)
    walk(cfg)
    binary = open(f'{d}/binary.txt').read().split('\n')[0][:30]
    drv = list(csv.DictReader(open(f'{d}/driver.csv')))
    t0 = float(drv[0]['t'])
    auth = next((float(r['t']) for r in drv if r['auth'] == '1'), None)
    cap = next((float(r['t']) for r in drv if r['captured'] == '1'), None)
    rows = list(csv.DictReader(open(f'{d}/replans.csv')))
    print(f'### {run}')
    print(f'binary mtime {binary} (per-run mtime only, no per-run hash recorded); '
          + ', '.join(f'{k} {flat.get(k, "not specified (binary default)")}' for k in KEYS))
    print('authority ' + (f'at +{auth - t0:.1f} s' if auth else 'NEVER granted') + '; captured at '
          + (f'+{cap - t0:.1f} s' if cap else 'never'))
    for name, sel in (('pre-authority', lambda r: auth is None or float(r['request_ros_time']) < auth),
                      ('after the first grant (approx.: request time; no epoch events)', lambda r: auth is not None and float(r['request_ros_time']) >= auth
                       and (cap is None or float(r['request_ros_time']) <= cap))):
        rs = [r for r in rows if sel(r)]
        if not rs:
            continue
        f = lambda k: np.array([float(r[k]) for r in rs if r.get(k) not in ('', 'nan', None)])
        acc = sum(r['accepted'] == '1' for r in rs)
        inst = sum(r['authority_installed'] == '1' for r in rs)
        late = sum(r['candidate_late'] == '1' for r in rs)
        rej = collections.Counter(r['installation_rejection'] for r in rs if r['installation_rejection'] not in ('', 'nan'))
        st = collections.Counter(r['status'] for r in rs).most_common(3)
        ok = lambda r, k: r.get(k) not in ('', 'nan', None)
        # Codex 15: the recorded margin is at the install ATTEMPT; the installed margin is splice time - authority_install_time_s
        m = np.array([1e3 * (float(r['installation_attempt_time_s']) + 1e-3 * float(r['installation_splice_margin_ms'])
                             - float(r['authority_install_time_s'])) for r in rs
                      if r['authority_installed'] == '1' and ok(r, 'installation_splice_margin_ms')
                      and ok(r, 'installation_attempt_time_s') and ok(r, 'authority_install_time_s')])
        span = float(rs[-1]['request_ros_time']) - float(rs[0]['request_ros_time'])
        wv = [int(float(r['planning_world_version'])) for r in rs if r['planning_world_version'] not in ('', 'nan')]
        print(f'- {name}: {len(rs)} replans over {span:.1f} s; accepted {acc}, INSTALLED {inst}, late {late}; '
              f'splice resolved ms p50 {pct(f("splice_lookahead_ms"), 50)}; replan runtime ms p50/p95/max '
              f'{pct(f("replan_runtime_ms"), 50)}/{pct(f("replan_runtime_ms"), 95)}/{pct(f("replan_runtime_ms"), 100)}; '
              f'completion-to-collection ms p95 {pct(f("completion_to_collection_ms"), 95)}; installed margin ms (splice - install time) '
              f'min/p5 {np.min(m) if len(m) else float("nan"):.1f}/{np.percentile(m, 5) if len(m) else float("nan"):.1f}; world versions/s {((wv[-1] - wv[0]) / span if span and wv else 0):.1f}; '
              f'installation outcomes {dict(rej) or "none"}; top statuses {st}')
    print()
