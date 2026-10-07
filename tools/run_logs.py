"""tools/run_logs.py -- where a run's node logs are, in both folder layouts.

Until the 3 Oct 2026 rename every node logged under logs/controller_quad_load/:
    planner_drone<i>_<stamp>/          the per-drone tracker
    load_planner_<stamp>/              the load planner
    dissipative_controller_<stamp>/    the dissipative planner
Since then each node logs under its own name:
    logs/tracker/drone<i>_<stamp>/
    logs/mpc_planner/<stamp>/
    logs/dissipative_planner/<stamp>/

`path` may be a run directory (MDC_RUN_DIR, holding logs/), a rig `<flight>_logs`
folder, or the folder that holds the session dirs themselves. Every tool that reads a
tracker or planner log goes through here, so old runs and new runs read the same.
"""
import glob
import os
import re

OLD_FOLDER = 'controller_quad_load'
# kind -> (old session-dir prefix, new folder, new session-dir prefix)
LAYOUT = {
    'tracker': ('planner_drone', 'tracker', 'drone'),
    'mpc_planner': ('load_planner_', 'mpc_planner', ''),
    'dissipative_planner': ('dissipative_controller_', 'dissipative_planner', ''),
}
_STAMP = re.compile(r'(\d{8}_\d{6})$')
_DRONE = re.compile(r'^(?:planner_)?drone(\d+)_\d{8}_\d{6}$')


def node_dirs(path, kind):
    """Session dirs of one node kind under `path`: old layout first, then new, each sorted
    by name (for one drone or one planner that is start order)."""
    old_pre, new_folder, new_pre = LAYOUT[kind]
    roots = [(os.path.join(path, 'logs', OLD_FOLDER), old_pre), (path, old_pre),
             (os.path.join(path, 'logs', new_folder), new_pre),
             (os.path.join(path, new_folder), new_pre)]
    if os.path.basename(os.path.normpath(path)) == new_folder:
        roots.append((path, new_pre))
    out, seen = [], set()
    for root, pre in roots:
        if not os.path.isdir(root):
            continue
        for d in sorted(glob.glob(os.path.join(root, pre + '*'))):
            name = os.path.basename(d)
            ok = drone_of(d) is not None if kind == 'tracker' else (
                name.startswith(pre) and _STAMP.search(name) and
                (pre or _STAMP.fullmatch(name)))
            real = os.path.realpath(d)
            if ok and os.path.isdir(d) and real not in seen:
                seen.add(real)
                out.append(d)
    return out


def node_csvs(path, kind):
    """log.csv of each session dir of `kind` under `path` (node_dirs order)."""
    return [os.path.join(d, 'log.csv') for d in node_dirs(path, kind)
            if os.path.exists(os.path.join(d, 'log.csv'))]


def planner_csvs(path):
    """log.csv of every load-planner session under `path`: mpc_planner, then dissipative_planner
    (the same columns; the dissipative node subclasses the load planner)."""
    return node_csvs(path, 'mpc_planner') + node_csvs(path, 'dissipative_planner')


def drone_of(path):
    """Drone index of a tracker session dir (or its log.csv), else None."""
    p = os.path.normpath(path)
    if os.path.basename(p) == 'log.csv':
        p = os.path.dirname(p)
    m = _DRONE.match(os.path.basename(p))
    return int(m.group(1)) if m else None


def stamp_of(path):
    """'YYYYmmdd_HHMMSS' a session dir (or its log.csv) was started at, else None."""
    p = os.path.normpath(path)
    if os.path.basename(p) == 'log.csv':
        p = os.path.dirname(p)
    m = _STAMP.search(os.path.basename(p))
    return m.group(1) if m else None


def trackers(path):
    """{drone: log.csv} of the last tracker session of each drone under `path`."""
    out = {}
    for f in node_csvs(path, 'tracker'):
        out[drone_of(f)] = f
    return out


def run_root(csv_path):
    """The folder node_dirs() searches to find the siblings of one session's log.csv
    (the trackers beside a planner session, say), whichever layout it is in."""
    session = os.path.dirname(os.path.abspath(csv_path))
    parent = os.path.dirname(session)
    if os.path.basename(parent) in {v[1] for v in LAYOUT.values()}:
        return os.path.dirname(parent)
    return parent
