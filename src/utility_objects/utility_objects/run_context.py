"""
run_context.py
--------------
Where things get written. One module that answers three questions the same way
for every node, instead of each node answering them from its own working
directory.

  repo_root()      the workspace root, found without hardcoding anyone's $HOME
  results_root()   where flight data goes            (default <repo>/results)
  acados_dir(name) where a solver's generated C goes (default <repo>/c_generated_code_<name>)
  run_dir()        the current run's directory, if a harness has declared one

WHY THIS EXISTS
---------------
Until 2026-08-04 `DataLogger` built its path from `os.getcwd()`, and two nodes
(`controller_mpc.py`, `thrust_ratio_node.py`) call `os.chdir()` into the acados
build directory at import time, because acados generates its C code relative to
the working directory. The consequence was that **237 MB of flight logs lived
inside `c_generated_code_quad_load/`** -- a gitignored build directory that is
deleted whenever a solver rebuild is forced. Flight data was one routine
troubleshooting step away from being gone, and identical runs landed in three
different trees depending on which node wrote them.

Separately, `ACADOS_DIR` was the literal string
`/home/wesley/multi_drone_control/c_generated_code_quad_load`, so the workspace
could not be checked out anywhere else, by anyone, including on a second machine
during a lab session.

COMPATIBILITY
-------------
`DataLogger` is called by several controllers in this workspace (including the
approach MPC, which is developed in parallel by a collaborator). Silently moving
where they all write would be a surprise, and a surprise in logging is how data
goes missing. So the `base_dir=` arguments added to `DataLogger` / `run_log_dir`
default to None, which reproduces the old `os.getcwd()` path exactly; nodes opt in
explicitly. Change them over deliberately, one at a time, if you want everything in
the results tree.

ENVIRONMENT OVERRIDES (all optional)
------------------------------------
  MDC_REPO_ROOT     workspace root, if auto-detection ever gets it wrong
  MDC_RESULTS_ROOT  flight data root
  MDC_ACADOS_ROOT   parent directory for generated solver code
  MDC_RUN_DIR       this run's directory, set by tools/run_experiment.py so that
                    every node of one launch writes into ONE run directory
"""
import os

_REPO_ROOT = None


def _detect_repo_root():
    """Walk up from this file looking for the workspace root.

    Works whether the package is symlink-installed (realpath lands in src/) or
    copy-installed (path lands in install/, still inside the workspace). The
    marker is a directory holding both `src/` and `.git/` -- a colcon workspace
    that is also a git checkout, which is what this repo is.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    d = here
    while True:
        if (os.path.isdir(os.path.join(d, 'src'))
                and os.path.isdir(os.path.join(d, '.git'))):
            return d
        parent = os.path.dirname(d)
        if parent == d:          # reached /
            return None
        d = parent


def repo_root():
    """Absolute workspace root. Raises rather than guessing wrong.

    A wrong root would silently scatter flight data again, which is the exact
    failure this module exists to end -- so if it cannot be determined, say so
    loudly at startup instead of writing somewhere surprising.
    """
    global _REPO_ROOT
    if _REPO_ROOT is not None:
        return _REPO_ROOT
    env = os.environ.get('MDC_REPO_ROOT')
    if env:
        _REPO_ROOT = os.path.abspath(os.path.expanduser(env))
        return _REPO_ROOT
    found = _detect_repo_root()
    if found is None:
        raise RuntimeError(
            "run_context: could not locate the workspace root (looked for a "
            "directory containing both src/ and .git/ above "
            f"{os.path.realpath(__file__)}). Set MDC_REPO_ROOT to the workspace "
            "root and relaunch.")
    _REPO_ROOT = found
    return _REPO_ROOT


def results_root(create=True):
    """Root for all flight data. Never inside a build directory."""
    env = os.environ.get('MDC_RESULTS_ROOT')
    path = (os.path.abspath(os.path.expanduser(env)) if env
            else os.path.join(repo_root(), 'results'))
    if create:
        os.makedirs(path, exist_ok=True)
    return path


def acados_dir(name, create=True):
    """Generated-C directory for a named solver, e.g. acados_dir('quad_load').

    Each solver keeps its own directory so trackers never clobber each other's
    generated code -- the reason `c_generated_code_quad_load` exists separately
    from `c_generated_code` in the first place.
    """
    env = os.environ.get('MDC_ACADOS_ROOT')
    parent = (os.path.abspath(os.path.expanduser(env)) if env else repo_root())
    path = os.path.join(parent, f'c_generated_code_{name}')
    if create:
        os.makedirs(path, exist_ok=True)
    return path


def run_dir(create=False):
    """This run's directory if a harness declared one via MDC_RUN_DIR, else None.

    tools/run_experiment.py sets MDC_RUN_DIR before launching, so all ~8 nodes of
    a single run write into one self-describing directory instead of each
    allocating its own timestamped folder that then has to be correlated by
    eyeballing timestamps.
    """
    env = os.environ.get('MDC_RUN_DIR')
    if not env:
        return None
    path = os.path.abspath(os.path.expanduser(env))
    if create:
        os.makedirs(path, exist_ok=True)
    return path


def log_base_dir():
    """Where a node should put its logs: the declared run directory if there is
    one, otherwise the results root. This is the value to pass as `base_dir=`."""
    return run_dir(create=True) or results_root()


def describe():
    """One-line summary for a node to log at startup, so every run records where
    it believed it was writing."""
    try:
        rd = run_dir()
        return (f"repo={repo_root()} results={results_root(create=False)} "
                f"run_dir={rd or '<none>'}")
    except RuntimeError as e:
        return f"UNRESOLVED ({e})"
