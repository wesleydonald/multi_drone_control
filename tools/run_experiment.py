#!/usr/bin/env python3
"""
tools/run_experiment.py — headless Gazebo experiment runner (THESIS_PLAN §9.1)

One command runs a Gazebo experiment and writes a self-describing result directory.

    ./tools/run_experiment.py configs/experiments/attach_ring_45_gz.yaml
    ./tools/run_experiment.py configs/experiments/attach_ring_45_gz.yaml --repeats 5
    ./tools/run_experiment.py <cfg> --gui        # watch it, for debugging only

Requirements, each earned by a past failure (§9.1):

  * HEADLESS (`gz sim -s -r`) so batches run overnight.
  * SIM-TIME GATED, never wall-clock. Unequal sweep durations have invalidated
    comparisons here before; see runner_node.py.
  * CLEAN SLATE every run via tools/clean_slate.sh, and it must SUCCEED -- a leftover
    planner publishing to the same tracker corrupts every number in the run and nothing
    in the logs says so.
  * MANIFEST + READ-BACK PARAMETER DUMP: what the nodes actually got, not what the YAML
    asked for. Launch args have silently failed to plumb through, and a run that cannot
    prove what it flew is not evidence.
  * AUTO-METRICS on completion, so a finished run is already readable.
  * FAILS LOUDLY on pose timeout, fleet abort, criteria violation, or a duration
    mismatch >10% across repeats.

Exit code is nonzero if any repeat fails or a criterion is violated.

RELATIONSHIP TO tools/sil_bench.py: same result layout, same log schema, same metrics.
That bench replaces Gazebo with a numpy plant to iterate in seconds; this one is the
real simulator and is what a bench result must eventually be checked against.
"""
import argparse
import csv
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from experiment.config import ExperimentConfig            # noqa: E402
from run_dir import allocate, file_sha256, write_manifest  # noqa: E402


# ── process management ───────────────────────────────────────────────────────

def clean_slate(strict=True):
    """Return the machine to a known-clean state, and REFUSE TO RUN if it will not go.

    sil_bench.py can be relaxed about this because it owns its own plant and topics.
    Here a stale bridge or planner from a previous Gazebo run is invisible and
    corrupting, which is the failure clean_slate.sh was written for."""
    script = os.path.join(REPO, 'tools', 'clean_slate.sh')
    if not os.path.exists(script):
        raise SystemExit('tools/clean_slate.sh is missing; refusing to run dirty.')
    r = subprocess.run([script], cwd=REPO, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT)
    out = r.stdout.decode(errors='replace')
    if r.returncode != 0 and strict:
        raise SystemExit(f'clean_slate.sh failed (exit {r.returncode}); the machine is '
                         f'not clean, so the run would be corrupt:\n{out}')
    return out


def start_gazebo(cfg, log_path, gui=False):
    """`gz sim -s -r <world>` -- server only, running immediately.

    `-s` (server, no GUI) is what makes a batch possible; `-r` starts the world
    unpaused, without which /clock never advances and every sim-time deadline in the
    runner waits forever."""
    if shutil.which('gz') is None:
        raise SystemExit('gz is not on PATH -- Gazebo is required for this runner.')
    world = cfg.world
    if not os.path.isabs(world):
        world = os.path.join(REPO, 'simulation_assets', world)
    if not os.path.exists(world):
        raise SystemExit(f'world not found: {world}')
    cmd = ['gz', 'sim', '-r', '-v', '2', world] if gui else \
          ['gz', 'sim', '-s', '-r', '-v', '2', world]
    fh = open(log_path, 'w')
    fh.write('$ ' + ' '.join(cmd) + '\n\n')
    fh.flush()
    # Gazebo resolves model:// against this.
    env = dict(os.environ)
    assets = os.path.join(REPO, 'simulation_assets')
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [assets, os.path.join(assets, 'models'),
         env.get('GZ_SIM_RESOURCE_PATH', '')]).strip(os.pathsep)
    p = subprocess.Popen(cmd, cwd=os.path.join(REPO, 'simulation_assets'),
                         stdout=fh, stderr=subprocess.STDOUT,
                         start_new_session=True, env=env)
    return p, fh


def start_launch(cfg, launch_file, argv, log_path, run_dir):
    """Launch a REAL launch file, so it stays the single authority for every
    controller parameter (same rule as the SIL bench)."""
    if shutil.which('ros2') is None:
        raise SystemExit(
            'ros2 is not on PATH. Source the underlay AND this workspace:\n'
            '    source ~/ros2_humble/install/setup.bash\n'
            '    source install/setup.bash')
    cmd = ['ros2', 'launch', cfg.launch_package, launch_file, *argv]
    fh = open(log_path, 'w')
    fh.write('$ ' + ' '.join(cmd) + '\n\n')
    fh.flush()
    env = dict(os.environ)
    # Every node of this launch writes into ONE run directory (run_context.run_dir).
    env['MDC_RUN_DIR'] = run_dir
    p = subprocess.Popen(cmd, cwd=REPO, stdout=fh, stderr=subprocess.STDOUT,
                         start_new_session=True, env=env)
    return p, fh


def wait_for_clock(timeout=30.0):
    """Block until /clock has a publisher, i.e. the sim-interface launch's clock bridge
    is actually up. Returns the seconds waited (or the timeout, having warned)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            out = subprocess.check_output(['ros2', 'topic', 'info', '/clock'],
                                          stderr=subprocess.DEVNULL,
                                          timeout=10).decode()
            for ln in out.splitlines():
                if 'Publisher count' in ln and int(ln.split(':')[1]) > 0:
                    return time.time() - t0
        except Exception:
            pass
        time.sleep(0.5)
    print(f'    !! /clock still has no publisher after {timeout:.0f}s — starting the '
          f'control launch anyway; expect a startup timeout.')
    return timeout


def stop(proc, fh, name='process', sig=signal.SIGINT, grace=15):
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), sig)
            proc.wait(timeout=grace)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    try:
        if fh:
            fh.close()
    except Exception:
        pass


# ── parameter read-back ──────────────────────────────────────────────────────

REQUIRED_NODE_RE = r'/(controller_\d+|dissipative\w*|central_controller)$'


def dump_params(run_dir, timeout=25, workers=8, only=None, skip=()):
    """Ask every live node what it ACTUALLY has, and store it.

    Not the launch file's declared defaults, and not the YAML: the values the running
    nodes hold. `thrust_ratio:=20` once provably changed nothing while the launch
    reported it fine, and only a read-back would have shown that.

    RUN IN PARALLEL. `ros2 param dump` spawns a fresh Python interpreter and redoes
    node discovery every call, ~5-7 s each, and this launch has ~30 nodes. Serially that
    was several MINUTES of dead wall time per run -- the single largest cost in the
    harness, larger than the simulation it was wrapping. The calls are independent and
    read-only, so a thread pool collapses it to roughly one call's latency.
    """
    from concurrent.futures import ThreadPoolExecutor
    out_dir = os.path.join(run_dir, 'params')
    os.makedirs(out_dir, exist_ok=True)
    try:
        nodes = subprocess.check_output(['ros2', 'node', 'list'],
                                        stderr=subprocess.DEVNULL,
                                        timeout=timeout).decode().split()
    except Exception as e:
        with open(os.path.join(out_dir, 'READBACK_FAILED.txt'), 'w') as fh:
            fh.write(f'ros2 node list failed: {e}\n')
        return {'dumped': [], 'missing_required': ['<node list failed>'], 'n_nodes': 0}
    if only is not None:
        nodes = [n for n in nodes if re.search(only, n)]
    if skip:
        nodes = [n for n in nodes if not re.search(skip, n)]

    def one(node):
        try:
            d = subprocess.check_output(['ros2', 'param', 'dump', node],
                                        stderr=subprocess.DEVNULL,
                                        timeout=timeout).decode()
        except Exception:
            return None
        if not d.strip():
            return None
        with open(os.path.join(out_dir, node.strip('/').replace('/', '__') + '.yaml'),
                  'w') as fh:
            fh.write(d)
        return node

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=workers) as ex:
        got = set(x for x in ex.map(one, nodes) if x)
    # Retry the stragglers serially. The busiest nodes are the ones that miss under
    # concurrency -- the acados trackers are servicing a 50 Hz control loop -- and they
    # are also the only ones whose parameters this file exists to prove.
    missed = [n for n in nodes if n not in got]
    for node in missed:
        if one(node):
            got.add(node)

    # A read-back that quietly omits the CONTROLLERS is worse than none: it looks like
    # evidence and is not. At 16 workers this dropped controller_1, controller_3 and
    # dissipative_controller while happily capturing 36 bridges.
    required = [n for n in nodes if re.search(REQUIRED_NODE_RE, n)]
    absent = [n for n in required if n not in got]
    label = 'param read-back' if only else 'param read-back (background)'
    print(f'    {label}: {len(got)}/{len(nodes)} nodes in '
          f'{time.time() - t0:.1f}s' + (f' ({len(missed)} needed a retry)' if missed
                                        else ''))
    if absent:
        print(f'    !! NO PARAMETER READ-BACK for {", ".join(absent)} — this run cannot '
              f'prove what those nodes actually flew.')
    return {'dumped': sorted(got), 'missing_required': absent,
            'n_nodes': len(nodes)}


# ── criteria ─────────────────────────────────────────────────────────────────

def evaluate(cfg, rows, t_weld, aborts):
    """Apply cfg.criteria. Returns (ok, [(name, ok, detail)])."""
    c = cfg.criteria
    if c is None:
        return True, [('(no criteria — exploratory run)', True, '')]
    if not rows:
        return False, [('no data recorded', False, 'the run produced no rows')]
    t = np.array([r['t'] for r in rows], float)
    t0 = 0.0
    if c.window_from == 'weld':
        if t_weld is None:
            return False, [('weld never happened', False,
                            'criteria are measured from the weld')]
        t0 = float(t_weld)
    t1 = t0 + float(c.window_s) if c.window_s else t[-1]
    m = (t >= t0) & (t <= t1)
    if not np.any(m):
        return False, [(f'no samples in [{t0:.1f}, {t1:.1f}]s', False,
                        f'run ends at {t[-1]:.1f}s')]

    def col(name):
        return np.array([r.get(name, math.nan) for r in rows], float)[m]

    out = []
    if c.forbid_abort:
        out.append(('no fleet abort', not aborts,
                    aborts[0][1] if aborts else 'none'))
    if c.require_weld:
        out.append(('weld occurred', t_weld is not None,
                    f'at t={t_weld:.2f}s' if t_weld is not None else 'never'))
    if c.max_payload_tilt_deg is not None:
        v = float(np.nanmax(col('payload_tilt_deg')))
        out.append(('payload tilt bounded', v < c.max_payload_tilt_deg,
                    f'peak {v:.1f} deg (<{c.max_payload_tilt_deg:.1f})'))
    if c.max_drone_tilt_deg is not None:
        v = max(float(np.nanmax(col(f'd{i}_tilt_deg'))) for i in range(cfg.n_total))
        out.append(('drone tilt bounded', v < c.max_drone_tilt_deg,
                    f'peak {v:.1f} deg (<{c.max_drone_tilt_deg:.1f})'))
    if c.max_track_err_m is not None:
        v = max(float(np.nanmax(col(f'd{i}_track_err'))) for i in range(cfg.n_total))
        out.append(('tracking error bounded', v < c.max_track_err_m,
                    f'peak {v:.2f} m (<{c.max_track_err_m:.2f})'))
    if c.min_payload_z is not None:
        v = float(np.nanmin(col('payload_z')))
        out.append(('payload stayed up', v > c.min_payload_z,
                    f'min {v:.3f} m (>{c.min_payload_z:.3f})'))
    if c.max_payload_z is not None:
        v = float(np.nanmax(col('payload_z')))
        out.append(('payload stayed down', v < c.max_payload_z,
                    f'max {v:.3f} m (<{c.max_payload_z:.3f})'))
    return all(o[1] for o in out), out


# ── output ───────────────────────────────────────────────────────────────────

def write_logs(run_dir, rows, events, aborts, t_weld):
    logs = os.path.join(run_dir, 'logs')
    os.makedirs(logs, exist_ok=True)
    csv_path = os.path.join(logs, 'run.csv')
    if rows:
        with open(csv_path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for r in rows:
                w.writerow(r)
    # One time origin with run.csv -- both are already relative to t_ready. Getting
    # this wrong is not hypothetical: the SIL bench shipped a 33.7 s offset between
    # these two files, which would have put every event-keyed metric in the wrong
    # phase of the run.
    with open(os.path.join(logs, 'events.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        for t, ev, arg in events:
            w.writerow([f'{t:.3f}', ev, arg])
        for t, reason in aborts:
            w.writerow([f'{t:.3f}', 'FLEET_ABORT', reason])
    return csv_path


def write_metrics_and_plots(run_dir):
    """metrics.json + the six figures, read back off the logs just written (§9.1/§9.4).

    Read back rather than computed here, and computed by tools/metrics.py rather than
    inline: §9.2 is that every number in the thesis comes from that one file, and a
    second implementation living in the runner is exactly how two numbers for the same
    quantity end up in the same document. Best-effort -- see plot_run.finish_run."""
    try:
        from plot_run import finish_run
    except Exception as e:                           # noqa: BLE001
        return {'error': f'analysis unavailable: {type(e).__name__}: {e}'}
    return finish_run(run_dir)


# ── one run ──────────────────────────────────────────────────────────────────

def run_once(cfg, cfg_path, gui=False, repeat=0):
    import rclpy
    from experiment.runner_node import ExperimentRunner

    run_dir, rid = allocate(f'gz_{cfg.name}', kind='sim')
    print(f'\n=== Gazebo experiment: {cfg.name}  ({rid}, repeat {repeat}) ===')
    print('   ' + cfg.summary().replace('\n', '\n   '))
    print(f'    out:    {run_dir}')

    write_manifest(run_dir, run_id=rid, kind='sim', harness='run_experiment.py',
                   scenario=os.path.abspath(cfg_path),
                   scenario_sha256=file_sha256(cfg_path),
                   world=cfg.world, launch_file=cfg.launch_file,
                   launch_args=cfg.launch_args, repeat=repeat,
                   headless=not gui, status='starting')

    clean_out = clean_slate()
    with open(os.path.join(run_dir, 'logs', 'clean_slate.log'), 'w') as fh:
        fh.write(clean_out)

    gz_proc = gz_fh = stack = stack_fh = io = io_fh = None
    node = None
    reason, failures = 'unknown', []
    try:
        gz_proc, gz_fh = start_gazebo(
            cfg, os.path.join(run_dir, 'logs', 'gazebo.log'), gui=gui)
        # ORDER MATTERS, and the docstring at the top of three_attach_launch.py says so:
        # the sim-interface launch owns the /clock bridge, the pose bridges and the
        # mocap emulators, and it must be publishing BEFORE any controller starts.
        io, io_fh = start_launch(
            cfg, cfg.io_launch_file, cfg.io_launch_argv(),
            os.path.join(run_dir, 'logs', 'io_launch.log'), run_dir)
        # Wait for the CLOCK BRIDGE specifically, rather than sleeping a fixed guess.
        # That bridge is the thing the control launch cannot start without, and it is up
        # in ~1-2 s -- a flat 5 s sleep was both slower than necessary and no guarantee.
        wait_for_clock(timeout=30.0)
        stack, stack_fh = start_launch(
            cfg, cfg.launch_file, cfg.launch_argv(),
            os.path.join(run_dir, 'logs', 'launch.log'), run_dir)

        rclpy.init()
        node = ExperimentRunner(cfg, run_dir)

        # ── wait for the stack (WALL clock: /clock may not exist yet) ────────
        t0 = time.time()
        while not node.stack_ready():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - t0 > cfg.ready_timeout_s:
                raise TimeoutError(
                    f'stack not ready after {cfg.ready_timeout_s:.0f}s wall. '
                    f'sim_t={node.sim_t} poses='
                    f'{sum(p is not None for p in node.pose)}/{node.n} cmds='
                    f'{sum(c is not None for c in node.cmd)}/{node.n} '
                    f'payload={"yes" if node.payload else "no"}. '
                    f'See logs/gazebo.log and logs/launch.log.')
            if gz_proc.poll() is not None:
                raise RuntimeError('Gazebo exited during startup; see logs/gazebo.log')
            if stack.poll() is not None:
                raise RuntimeError('the launch exited during startup; '
                                   'see logs/launch.log')
            if io.poll() is not None:
                raise RuntimeError('the sim-interface launch exited during startup; '
                                   'see logs/io_launch.log')
        # PARAMETER DUMP BEFORE THE RUN CLOCK STARTS. `ros2 param dump` shells out once
        # per node and blocks this thread for tens of seconds, while Gazebo -- started
        # with -r, i.e. free-running -- keeps advancing /clock. Doing it after
        # start_clock() latched t_ready at 5.45 s and returned at 83 s, so the schedule
        # fired ARM, TAKEOFF, MAGNET and LAND in the same instant and the run was
        # garbage. Here the fleet is still disarmed on its stands, so the lost sim time
        # costs nothing.
        # The nodes whose parameters ARE the evidence -- the trackers, the planner, the
        # fleet manager -- are captured now, before anything flies, and their absence is
        # fatal to the run's credibility. That is ~8 nodes and takes seconds.
        readback = dump_params(run_dir, only=REQUIRED_NODE_RE)
        # Everything else (bridges, mocap emulators, visualisation) is useful context
        # but not evidence, and there are ~37 of them at ~5-25 s each. Dumping those on
        # the critical path cost 90 s of dead wall time per run -- comparable to the
        # simulation itself. They run in a thread alongside the flight instead:
        # parameters do not change mid-run, and subprocess calls release the GIL so the
        # rclpy spin loop below is unaffected.
        bg = {}
        bg_thread = threading.Thread(
            target=lambda: bg.update(other=dump_params(run_dir, skip=REQUIRED_NODE_RE)),
            daemon=True)
        # Drain the mocap that arrived during the dump, so the watchdog starts from
        # fresh stamps rather than pre-dump ones.
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.02)
        node.start_clock()
        bg_thread.start()

        # ── the run itself (SIM clock) ──────────────────────────────────────
        last_report = time.time()
        wall_cap = 20.0 * cfg.duration_s + 300.0     # only a hang guard, not a deadline
        wall0 = time.time()
        while not node.finished():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.fire_due_events()
            node.resend_pending()
            node.log_row()
            if node.check_health():
                reason = 'health check failed'
                break
            if gz_proc.poll() is not None:
                failures.append('Gazebo exited mid-run')
                reason = 'gazebo died'
                break
            if stack.poll() is not None:
                failures.append('the controller stack exited mid-run')
                reason = 'stack exited'
                break
            if time.time() - wall0 > wall_cap:
                failures.append(
                    f'wall-clock hang guard tripped ({wall_cap:.0f}s) at sim '
                    f't={node._rel():.1f}/{cfg.duration_s:.1f}: sim time is not '
                    f'advancing')
                reason = 'sim time stalled'
                break
            if time.time() - last_report > 5.0:
                last_report = time.time()
                print(f'    t={node._rel():7.2f}s sim  '
                      f'payload_z={node.payload[2]:.3f}  '
                      f'tilt={node.rows[-1]["payload_tilt_deg"]:5.1f}deg'
                      if node.rows else '    (no rows yet)')
        else:
            reason = node.stop_reason or 'completed'
        failures += node.failures
    except Exception as e:
        failures.append(f'{type(e).__name__}: {e}')
        reason = 'exception'
    finally:
        rows = list(node.rows) if node else []
        events = list(node.event_log) if node else []
        aborts = list(node.aborts) if node else []
        t_weld = node.t_weld if node else None
        if node is not None:
            node.destroy_node()
        try:
            import rclpy as _r
            if _r.ok():
                _r.shutdown()
        except Exception:
            pass
        stop(stack, stack_fh, 'launch')
        stop(io, io_fh, 'io_launch')
        stop(gz_proc, gz_fh, 'gazebo')

    # The background read-back must finish before the manifest claims what was captured.
    try:
        bg_thread.join(timeout=90)
        if bg_thread.is_alive():
            print('    !! background param read-back did not finish; params/ is partial')
    except Exception:
        pass

    write_logs(run_dir, rows, events, aborts, t_weld)
    mtr = write_metrics_and_plots(run_dir)
    ok, checks = evaluate(cfg, rows, t_weld, aborts)
    ok = ok and not failures

    dur = rows[-1]['t'] - rows[0]['t'] if rows else 0.0
    print(f'\n    recorded {len(rows)} rows over {dur:.1f}s sim; exit: {reason}')
    if t_weld is not None:
        print(f'    weld at t={t_weld:.2f}s')
    for name, good, detail in checks:
        print(f'    [{"PASS" if good else "FAIL"}] {name:38s} {detail}')
    for f in failures:
        print(f'    !! {f}')

    write_manifest(run_dir, status='finished', exit_reason=reason,
                   param_readback=readback, param_readback_other=bg.get('other'),
                   stopped_early=bool(node is not None and node.stop_reason),
                   sim_duration_s=dur, n_rows=len(rows), weld_time_s=t_weld,
                   passed=bool(ok), failures=failures,
                   checks=[{'check': n, 'passed': bool(g), 'detail': d}
                           for n, g, d in checks],
                   metrics=mtr)
    return {'run_dir': run_dir, 'run_id': rid, 'ok': bool(ok),
            'duration_s': dur, 'failures': failures,
            'stopped_early': bool(node is not None and node.stop_reason)}


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', nargs='+', help='experiment YAML (configs/experiments/)')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--gui', action='store_true',
                    help='run Gazebo with its GUI (debugging; not for batches)')
    args = ap.parse_args()

    results = []
    for path in args.config:
        cfg = ExperimentConfig.from_yaml(path)
        for rep in range(args.repeats):
            results.append((cfg, rep, run_once(cfg, path, gui=args.gui, repeat=rep)))

    print('\n=== experiment summary ===')
    ok_all = True
    for cfg, rep, r in results:
        print(f'  [{"PASS" if r["ok"] else "FAIL"}] {cfg.name} repeat {rep}  '
              f'-> {r["run_dir"]}')
        ok_all = ok_all and r['ok']

    # Duration consistency across repeats (§9.1). Unequal durations make a repeat set
    # incomparable, and the difference is easy to miss in a directory listing.
    #
    # Runs that stopped EARLY are excluded: they end a fixed grace after the fleet
    # aborted, so their length varies with when the abort happened, which is a RESULT
    # rather than a defect. Flagging those would train everyone to ignore this warning,
    # which is the one thing a check like this must not do.
    by_cfg = {}
    for cfg, rep, r in results:
        if r.get('stopped_early'):
            continue
        by_cfg.setdefault(cfg.name, []).append(r['duration_s'])
    for name, ds in by_cfg.items():
        if len(ds) > 1 and max(ds) > 0:
            spread = (max(ds) - min(ds)) / max(ds)
            if spread > 0.10:
                print(f'  !! {name}: repeat durations differ by {100 * spread:.0f}% '
                      f'({min(ds):.1f}..{max(ds):.1f}s) — these repeats are NOT '
                      f'comparable')
                ok_all = False

    print(f'\n  RESULT: {"ALL PASSED" if ok_all else "FAILURES ABOVE"}')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
