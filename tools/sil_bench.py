#!/usr/bin/env python3
"""
tools/sil_bench.py — software-in-the-loop bench (THESIS_PLAN §9.3)

Closes the loop around the REAL controller nodes -- the acados trackers, the fleet
manager and the dissipative reference generator -- against a fast numerical plant, over
real ROS topics, with no Gazebo and no human at the RViz buttons.

    ./tools/sil_bench.py configs/sil/attach_ring_45.yaml
    ./tools/sil_bench.py configs/sil/attach_ring_45.yaml --repeats 3
    ./tools/sil_bench.py --acceptance          # the §9.3 acceptance pair, 45 vs 65

Design note: docs/design/sil_bench.md — read §7 (blind spots) before quoting a number
from this bench. Bench agreement is NECESSARY, NOT SUFFICIENT: it has no contact
physics, no aerodynamics, no mocap noise and no timing jitter, and nothing goes to the
rig on bench evidence alone.

Exit code is nonzero if the run fails or an acceptance criterion is not met.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from run_dir import allocate, file_sha256, write_manifest  # noqa: E402
from sil.scenario import Scenario  # noqa: E402

BLIND_SPOTS = """
  This bench has NO contact/collision physics, NO aerodynamics, NO motor mixing or ESC
  saturation, NO mocap noise/dropout/latency, NO rotor or magnet-arm inertia, and (by
  design, because it is lockstep) NO timing jitter. The body-rate time constant is
  SHARED with the tracker's own model, so a rate-loop mismatch failure is outside what
  it can show. Bench agreement is necessary, not sufficient -- see
  docs/design/sil_bench.md §7.

  ATTACH RUNS ARE A COUNTERFACTUAL. The shipped elrs_mux hands control to our tracker
  on the first /magnet/object_attached without checking that it has a reference, so the
  real newcomer gets 100-160 ms of ZERO throttle at the weld (measured, R0009/R0010).
  The bench holds it on the stand-in through that gap instead. Attach results here are
  therefore evidence about the CONTROL LAW, not about the shipped handoff, which has an
  open defect -- see bench_node._handed_over."""


# ── the controller stack ─────────────────────────────────────────────────────

def clean_slate():
    script = os.path.join(REPO, 'tools', 'clean_slate.sh')
    if not os.path.exists(script):
        return
    subprocess.run([script], cwd=REPO, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def start_stack(scn, console_path):
    """Launch the REAL controllers via the REAL launch file with sil:=true.

    Running the actual launch (rather than re-declaring the nodes here) is what keeps
    the launch file the single authority for every controller parameter -- see
    docs/design/sil_bench.md §3.2."""
    if shutil.which('ros2') is None:
        raise SystemExit(
            'ros2 is not on PATH. The bench runs the real launch file, so this shell '
            'needs the ROS underlay AND this workspace:\n'
            '    source ~/ros2_humble/install/setup.bash\n'
            '    source install/setup.bash')
    cmd = ['ros2', 'launch', scn.launch_package, scn.launch_file,
           'sil:=true', *scn.launch_argv()]
    console = open(console_path, 'w')
    console.write('$ ' + ' '.join(cmd) + '\n\n')
    console.flush()
    proc = subprocess.Popen(cmd, cwd=REPO, stdout=console, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, console


def stop_stack(proc, console):
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=15)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    try:
        console.close()
    except Exception:
        pass


# ── acceptance ───────────────────────────────────────────────────────────────

def evaluate(scn, rows, weld_t):
    """Apply the scenario's acceptance criteria. Returns (ok, list of (name, ok, detail))."""
    acc = scn.acceptance
    if acc is None:
        return True, [('(no acceptance criteria — exploratory run)', True, '')]
    if not rows:
        return False, [('no data logged', False, '')]
    # Rows carry absolute sim time; the first logged row is the instant the scenario
    # clock started (t_ready), and weld_t is already scenario-relative, so subtracting
    # the first row's time puts both on the same axis.
    rel_t = np.array([r['t'] for r in rows])
    rel_t = rel_t - rel_t[0]
    t0 = float(weld_t) if weld_t is not None else 0.0
    m = (rel_t >= t0) & (rel_t <= t0 + acc.window_s)
    if not np.any(m):
        return False, [(f'no samples in the {acc.window_s:.0f}s window after the weld',
                        False, f'weld at {t0:.2f}s, run ends at {rel_t[-1]:.2f}s')]
    d = acc.drone
    err = np.array([r[f'd{d}_track_err'] for r in rows])[m]
    acm = np.array([r[f'd{d}_acm'] for r in rows])[m]
    tilt = np.array([r['payload_tilt_deg'] for r in rows])[m]
    err = err[np.isfinite(err)]
    peak_err = float(np.max(err)) if err.size else float('nan')
    peak_acm = float(np.max(acm))
    peak_tilt = float(np.max(tilt))
    # "growing": compare the last quarter of the window against the first quarter
    q = max(1, len(err) // 4)
    growing = bool(err.size and np.mean(err[-q:]) > np.mean(err[:q]) + 0.05)

    out = []
    if acc.expect == 'diverge':
        out.append((f'drone {d} tracking error diverges',
                    bool(peak_err > acc.min_track_err_m and growing),
                    f'peak {peak_err:.2f} m (>{acc.min_track_err_m:.2f}), '
                    f'{"growing" if growing else "NOT growing"}'))
        out.append((f'drone {d} |aCm| exceeds control authority',
                    bool(peak_acm > acc.min_cable_accel),
                    f'peak {peak_acm:.1f} m/s^2 (>{acc.min_cable_accel:.1f})'))
        out.append(('payload capsizes',
                    bool(peak_tilt > acc.min_tilt_deg),
                    f'peak tilt {peak_tilt:.1f} deg (>{acc.min_tilt_deg:.1f})'))
    else:
        out.append((f'drone {d} holds its reference',
                    bool(np.isfinite(peak_err) and peak_err < acc.max_track_err_m),
                    f'peak {peak_err:.2f} m (<{acc.max_track_err_m:.2f})'))
        out.append((f'drone {d} |aCm| stays bounded',
                    bool(peak_acm < acc.max_cable_accel),
                    f'peak {peak_acm:.1f} m/s^2 (<{acc.max_cable_accel:.1f})'))
        out.append(('payload stays upright',
                    bool(peak_tilt < acc.max_tilt_deg),
                    f'peak tilt {peak_tilt:.1f} deg (<{acc.max_tilt_deg:.1f})'))
    return all(o[1] for o in out), out


# ── one run ──────────────────────────────────────────────────────────────────

def run_one(scn_path, keep_going=False, repeat=0):
    import rclpy
    from sil.bench_node import SilBench

    scn = Scenario.from_yaml(scn_path)
    slug = scn.name + (f'_r{repeat}' if repeat else '')
    run_path, run_id = allocate(slug, kind='sil')
    write_manifest(run_path, run_id=run_id, kind='sil', scenario=scn_path,
                   scenario_sha256=file_sha256(scn_path), scenario_name=scn.name,
                   launch=f'{scn.launch_package}/{scn.launch_file}',
                   launch_args=scn.launch_args, repeat=repeat,
                   blind_spots=BLIND_SPOTS.strip())

    print(f'\n=== SIL bench: {scn.name}  ({run_id}) ===')
    print(f'    {scn.description.strip()}')
    print(f'    launch: {scn.launch_file} sil:=true ' + ' '.join(scn.launch_argv()))
    print(f'    out:    {run_path}')

    clean_slate()
    proc, console = start_stack(scn, os.path.join(run_path, 'console.log'))

    rclpy.init()
    bench = SilBench(scn, run_path)
    t0 = time.monotonic()
    ok = False
    try:
        ok = bench.run()
    except KeyboardInterrupt:
        bench.finished_reason = 'interrupted'
    finally:
        wall = time.monotonic() - t0
        csv_path = bench.write_logs()
        weld_t = bench.weld_time()
        rows = bench.rows
        stalls, aborts = bench.stalls, list(bench.aborts)
        reason = bench.finished_reason
        rtf = (bench.sim_t / wall) if wall > 0 else 0.0
        bench.destroy_node()
        rclpy.shutdown()
        stop_stack(proc, console)

    acc_ok, results = evaluate(scn, rows, weld_t)
    print(f'\n    ran {len(rows)} steps in {wall:.1f} s wall '
          f'(x{rtf:.2f} realtime), {stalls} lockstep stalls')
    if aborts:
        print(f'    !! /fleet/abort fired: {aborts}')
    print(f'    exit: {reason}')
    for name, good, detail in results:
        print(f'    [{"PASS" if good else "FAIL"}] {name:44s} {detail}')

    write_manifest(run_path, exit_reason=reason, completed=bool(ok),
                   wall_seconds=round(wall, 2), realtime_factor=round(rtf, 3),
                   sim_seconds=round(bench_sim_seconds(rows), 2),
                   lockstep_stalls=stalls, fleet_aborts=aborts,
                   weld_sim_time=weld_t, acceptance_passed=bool(acc_ok),
                   acceptance=[{'check': n, 'passed': bool(g), 'detail': d}
                               for n, g, d in results],
                   log_csv=csv_path, ended_wall=time.strftime('%Y-%m-%dT%H:%M:%S'))
    return bool(ok and acc_ok), run_path


def bench_sim_seconds(rows):
    if not rows:
        return 0.0
    return float(rows[-1]['t'] - rows[0]['t'])


# ── CLI ──────────────────────────────────────────────────────────────────────

ACCEPTANCE_PAIR = ['configs/sil/attach_ring_45.yaml',
                   'configs/sil/attach_ring_65.yaml']


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario', nargs='*', help='scenario YAML (configs/sil/*.yaml)')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--acceptance', action='store_true',
                    help='run the §9.3 acceptance pair (45 deg must diverge, '
                         '65 deg must not)')
    args = ap.parse_args()

    paths = args.scenario or (ACCEPTANCE_PAIR if args.acceptance else [])
    if not paths:
        ap.error('give a scenario YAML, or --acceptance')
    paths = [p if os.path.isabs(p) else os.path.join(REPO, p) for p in paths]
    for p in paths:
        if not os.path.exists(p):
            ap.error(f'no such scenario: {p}')

    print(__doc__.split('Design note')[0].strip())
    print('\n  BLIND SPOTS' + BLIND_SPOTS)

    all_ok = True
    summary = []
    for p in paths:
        for r in range(args.repeats):
            ok, rd = run_one(p, repeat=r)
            summary.append((os.path.basename(p), r, ok, rd))
            all_ok = all_ok and ok

    print('\n=== SIL bench summary ===')
    for name, r, ok, rd in summary:
        print(f'  [{"PASS" if ok else "FAIL"}] {name} repeat {r}  -> {rd}')
    if args.acceptance:
        print('\n  The §9.3 acceptance claim is DISCRIMINATION: the 45 deg config must')
        print('  diverge and the 65 deg config must not. Both lines above must be PASS.')
    print(f'\n  RESULT: {"ALL PASSED" if all_ok else "FAILURES ABOVE"}\n')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
