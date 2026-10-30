#!/usr/bin/env python3
"""Codex's runtime checks of his MPC's send path in a twin run (DECISIONS_1355.md section 1).

    e2check.py <run dir> <his log dir (metadata.json + log.csv)>

Reads metadata.json (profile, command execution, stage duration) and his log: u0..u3 are the command logged each
tick (under partial_step the rounded command sent, which is also the history entry and the next tick's initial
command). The tick's sim interval comes from the run's (wall, t) pairs. Per tick after TAKEOFF: |du| against the oct10
derivative bounds (roll/pitch/yaw 1/s, throttle 0.5/s) times the interval, plus the radio rounding.
"""
import json
import os
import sys

import numpy as np
import pandas as pd

BOUND = np.array([1.0, 1.0, 0.5, 1.0])     # oct10 command-derivative bounds (Codex): roll, pitch, throttle, yaw per s
ROUND = np.array([0.0006, 0.0006, 0.0006, 0.0006])


def main(run, logdir):
    meta = json.load(open(os.path.join(logdir, 'metadata.json')))
    d = pd.read_csv(os.path.join(logdir, 'log.csv'))
    rc = pd.read_csv(os.path.join(run, 'logs', 'run.csv'))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from align import sim_time_of
    to_sim, _, resid = sim_time_of(rc, d)
    t = to_sim(d.timestamp.values)
    print(f'his log aligned to sim time (height residual {resid * 1000:.1f} mm)')
    print(f"metadata: profile {meta.get('mpc_profile')}, command_execution {meta.get('command_execution')}, "
          f"stage {meta.get('mpc_command_stage_duration_s')}, control {meta.get('control_frequency_hz')} Hz")
    zero = np.flatnonzero(d.step.values == 0)
    k0 = int(zero[-1]) if len(zero) else 0          # the TAKEOFF tick: the last row logged with step 0
    u = d[['u0', 'u1', 'u2', 'u3']].values
    a = slice(k0, len(d))
    dt = np.diff(t[a])
    du = np.abs(np.diff(u[a], axis=0))
    ok = np.isfinite(dt) & (dt > 0)
    allowed = BOUND[None, :] * dt[ok, None] + ROUND[None, :]
    over = du[ok] > allowed
    hover = 9.81 / d.mpc_thrust_ratio.values[k0]
    first = u[k0]
    print(f'TAKEOFF tick: row {k0} at sim {t[k0]:.2f} s; first command {np.round(first, 4)}; hover throttle {hover:.4f}; '
          f'|first - [0, 0, hover, 0]| = {np.round(np.abs(first - [0, 0, hover, 0]), 4)} '
          f'(one nominal interval allows {np.round(BOUND / 30 + ROUND, 4)})')
    print(f'ticks after TAKEOFF {ok.sum()}; sim interval median {np.median(dt[ok]) * 1e3:.1f} ms '
          f'(5-95 % {np.percentile(dt[ok], 5) * 1e3:.1f}-{np.percentile(dt[ok], 95) * 1e3:.1f}); wall period median '
          f'{np.median(d.control_loop_period_ms.values[a]):.1f} ms')
    names = ('roll', 'pitch', 'throttle', 'yaw')
    print(f'per tick |du| max {np.round(du.max(0), 4)}, p99 {np.round(np.percentile(du, 99, axis=0), 4)} '
          f'(one nominal 33.3 ms interval at the bounds: {np.round(BOUND / 30, 4)}; the 100 ms endpoint: {BOUND / 10})')
    for i, n in enumerate(names):
        r = du[ok, i] / dt[ok]
        print(f'  {n}: steps over the interval bound {over[:, i].sum()} of {ok.sum()} ({100 * over[:, i].mean():.1f} %); '
              f'|du|/dt median {np.median(r):.3f}, p99 {np.percentile(r, 99):.3f}, max {r.max():.3f} per s')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
