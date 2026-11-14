"""Shared set-up for the Q013 real-world tables (items 3 and 4): paper_rig's Flight (its log parsing, ring grid, losses
and windows) with the bag caches kept in this folder.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && nice -n 10 python3 results/analysis/rig_results_2026-11-13/<script>.py

Bags already read by paper_rig (~/mdc_overnight/joins_cache) are loaded from there read-only; new ones are cached in
cache/ beside this file. paper_rig's read_extra (every /drone_N/magnet topic) is skipped: it only finds operator
releases, and no drop flight has one.
"""
import csv
import os
import pickle
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.expanduser('~/multi_drone_control')
RIG = os.path.join(REPO, 'results', 'rig')
CACHE = os.path.join(HERE, 'cache')
OLD_JOINS = os.path.expanduser('~/mdc_overnight/joins_cache')
os.makedirs(CACHE, exist_ok=True)
os.environ['JOINS_CACHE'] = CACHE
os.environ['PAPER_CACHE'] = CACHE
sys.path.insert(0, os.path.join(REPO, 'results', 'analysis', 'paper_rig_2026-11'))
import paper_rig as PR  # noqa: E402
import joins  # noqa: E402  (paper_rig put its folder on the path)

_read_bag = joins.read_bag


def _read_bag_cached(folder):
    old = os.path.join(OLD_JOINS, folder.replace('/', '_') + '.pkl')
    if os.path.exists(old):
        with open(old, 'rb') as f:
            return pickle.load(f)
    return _read_bag(folder)


joins.read_bag = _read_bag_cached
PR.read_extra = lambda folder: {'magnet': {}, 'op': [], 'tipstate': np.zeros((0, 8))}

MOTION = {'hover': 'hover', 'orbit': 'circle', 'fig_8': 'figure-eight', None: 'hover'}


def flight(fid):
    """paper_rig's Flight for 'day/name'."""
    day = fid.split('/')[0]
    launch = {f"{L['day']}/{L['name']}": L for L in PR.launches(day)}
    return PR.Flight(fid, launch.get(fid, {}), PR.registry(), PR.conditions())


def log_lines(day, name, rx):
    """[(t, match)] for every stamped log line matching rx."""
    out = []
    with open(os.path.join(RIG, day, name + '.log'), errors='replace') as f:
        for ln in f:
            m = re.search(rx, ln)
            if m:
                s = PR.STAMP.search(ln)
                if s:
                    out.append((float(s.group(1)), m))
    return out


def write_csv(path, rows, cols=None):
    cols = list(cols or [])
    for r in rows:
        for k in r:
            if k not in cols:
                cols.append(k)

    def fmt(v):
        if isinstance(v, (float, np.floating)):
            return '' if not np.isfinite(v) else (f'{v:.3f}' if abs(v) < 100 else f'{v:.1f}')
        return '' if v is None else v
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, lineterminator='\n')
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(r.get(k)) for k in cols})
