"""
tools/run_dir.py
----------------
Run-directory allocation and the manifest, shared by tools/sil_bench.py and
tools/run_experiment.py.

THESIS_PLAN §4.1: every run gets `results/<date>/R####_<slug>/` containing a manifest,
the parameter dump, logs, plots and metrics. §4.2 puts the allocator in
`utility_objects/run_context.py`; it is here instead because allocation and manifests
are HARNESS concerns -- no ROS node needs them, nodes only need `log_base_dir()`, which
run_context already provides and which this module reuses so there is still exactly one
answer to "where does data go".

Principle 6: a result directory that cannot tell you what produced it is not evidence.
So the manifest records the git SHA, whether the tree was dirty and how dirty, the
scenario/config file with its hash, sim-vs-real, the command line, and the exit reason.
"""
import getpass
import hashlib
import json
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'src', 'utility_objects'))
from utility_objects.run_context import repo_root, results_root  # noqa: E402


def _git(*args):
    try:
        return subprocess.check_output(['git', '-C', repo_root(), *args],
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ''


def git_state():
    """SHA, dirty file count, and a hash of the working diff.

    The diff hash matters: two runs from the same SHA with different uncommitted edits
    are different experiments, and without this they are indistinguishable in the
    results tree."""
    sha = _git('rev-parse', 'HEAD')
    status = _git('status', '--porcelain')
    diff = _git('diff', 'HEAD')
    return {
        'git_sha': sha,
        'git_branch': _git('rev-parse', '--abbrev-ref', 'HEAD'),
        'dirty_files': len([ln for ln in status.splitlines() if ln.strip()]),
        'diff_sha256': hashlib.sha256(diff.encode()).hexdigest() if diff else '',
    }


def file_sha256(path):
    try:
        with open(path, 'rb') as fh:
            return hashlib.sha256(fh.read()).hexdigest()
    except Exception:
        return ''


def next_run_id(root=None):
    """Monotonic R#### across the whole results tree.

    Scans rather than keeping a counter file, so a deleted or hand-copied run cannot
    desynchronise the sequence and reuse an id that a thesis figure already cites."""
    root = root or results_root()
    n = 0
    for dirpath, dirnames, _ in os.walk(root):
        for d in dirnames:
            if len(d) >= 5 and d[0] == 'R' and d[1:5].isdigit():
                n = max(n, int(d[1:5]))
        # only need two levels: results/<date>/R####_...
        if os.path.relpath(dirpath, root).count(os.sep) >= 1:
            dirnames[:] = []
    return n + 1


def allocate(slug, kind='sim', root=None):
    """Create and return `results/<YYYY-MM-DD>/R####_<kind>_<slug>/`.

    `kind` is 'sim' or 'real' and is in the directory NAME as well as the manifest --
    §4.1 is explicit that this must never be inferable only from context."""
    root = root or results_root()
    day = time.strftime('%Y-%m-%d')
    rid = next_run_id(root)
    name = f'R{rid:04d}_{kind}_{slug}'
    path = os.path.join(root, day, name)
    os.makedirs(os.path.join(path, 'logs'), exist_ok=True)
    os.makedirs(os.path.join(path, 'plots'), exist_ok=True)
    os.makedirs(os.path.join(path, 'params'), exist_ok=True)
    return path, f'R{rid:04d}'


def find_runs(root=None):
    """Every run directory under a results tree, newest date first."""
    root = root or results_root()
    out = []
    for dirpath, dirnames, _ in os.walk(root):
        for d in sorted(dirnames):
            if len(d) >= 5 and d[0] == 'R' and d[1:5].isdigit():
                out.append(os.path.join(dirpath, d))
        if os.path.relpath(dirpath, root).count(os.sep) >= 1:
            dirnames[:] = []
    return sorted(out, reverse=True)


def resolve(spec, roots=None):
    """A run id ('R0034'), a directory name or a path -> the run directory.

    Ids are how runs are cited in a caption, a commit message and a conversation, so
    every tool takes them. `results_archive/` is searched FIRST: once a run is promoted,
    the frozen copy is the one a figure should be built from, and silently preferring
    the mutable original would defeat the archive."""
    if os.path.isdir(spec) and os.path.exists(os.path.join(spec, 'manifest.json')):
        return os.path.abspath(spec)
    if roots is None:
        roots = [os.path.join(repo_root(), 'results_archive'), results_root()]
    key = os.path.basename(spec.rstrip('/'))
    for root in roots:
        if not os.path.isdir(root):
            continue
        for path in find_runs(root):
            name = os.path.basename(path)
            if name == key or name.startswith(key + '_') or name[:5] == key:
                return path
    raise SystemExit(f'no run matching {spec!r} under ' + ' or '.join(roots))


def write_manifest(run_path, **fields):
    """Write (or update) manifest.json. Safe to call twice -- the second call merges,
    so the runner can record the start immediately and the exit reason at the end even
    if the run crashes in between."""
    path = os.path.join(run_path, 'manifest.json')
    data = {}
    if os.path.exists(path):
        try:
            with open(path) as fh:
                data = json.load(fh)
        except Exception:
            data = {}
    if not data:
        data = {
            'run_dir': run_path,
            'host': socket.gethostname(),
            'operator': getpass.getuser(),
            'argv': sys.argv,
            'started_wall': time.strftime('%Y-%m-%dT%H:%M:%S'),
            **git_state(),
        }
    data.update(fields)
    with open(path, 'w') as fh:
        json.dump(data, fh, indent=2, sort_keys=True, default=str)
    return path
