#!/usr/bin/env python3
"""tools/yaw_track.py -- the ring's yaw against its reference on a spin or spinny line, for rig or twin runs.

    python3 tools/yaw_track.py ls3c                   # rig run (today's folder first, else the latest day)
    python3 tools/yaw_track.py ls3 --old-law          # flown before 22 Oct: line_spin's yaw was 1 rad per radius
    python3 tools/yaw_track.py R2361 --png out.png    # twin run, with the figure

From the 22 Oct rig day (ls3, ls3c). The desired yaw is the planner's yaw datum (the ring's yaw at the path's start)
plus LoadTrajectory.yaw_at(traj_t) for the run's own load_traj / traj_speed / traj_distance / traj_radius
(params.json); traj_t comes from the planner's log, aligned to run.csv on the ring's lift-off. Printed (deg, s):
  error            ring minus desired over the path: mean, rms, max |.|
  delay            the time shift of the ring's yaw that best matches the desired (+ = the ring is late) and the
                   rms left after it
  wobble           the error less its 1 s moving mean over the path's middle (the path minus its first and last 2 s):
                   sd, p-p and its peak frequency
  turned           the ring's turn over the path against the desired
  --plan           the plan's delay behind the same desired (planner logs from 23 Oct): its yaw reference, the ring it
                   starts from (node 0) and its ring yaw at nodes 1 and 5, each placed at the time it plans for
Read-only.
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))
import rig_summary  # noqa: E402
import run_logs  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402

NODE_DT = 0.1                    # the planner's node step (PLAN_TF / PLAN_N)
PLAN_COLS = ('yaw_ref0', 'plan_yaw1', 'plan_yaw5')


class OldLineSpin(LoadTrajectory):
    """line_spin's yaw law until 22 Oct: one radian per radius travelled, held after the trip."""

    def yaw_at(self, t):
        if t <= 0.0 or self.kind != 'line_spin':
            return super().yaw_at(t)
        _, vx, _, arc, _ = self._line_spin(t)
        return arc / self.radius, abs(vx) / self.radius


def _lift(t, z):
    t, z = np.asarray(t, float), np.asarray(z, float)
    return float(t[np.argmax(z > np.median(z[:20]) + 0.03)])


def _yaw_deg(q):
    q = np.asarray(q, float)
    return np.degrees(np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                           1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))


def _delay(xs, ys, x, des, m):
    """The shift of ys(xs) that best matches des(x) over m (+ = late), and the rms left after it."""
    shifts = np.arange(-0.5, 1.5, 0.01)
    rms_at = [np.sqrt(np.mean((np.interp(x[m] + s, xs, ys) - des[m]) ** 2)) for s in shifts]
    k = int(np.argmin(rms_at))
    return float(shifts[k]), float(rms_at[k])


def plan_lag(L, t0, x, des, m):
    """{ref, node0, node1, node5: (delay s, rms deg after)} of the planner's log rows that carry a plan, against
    the ring's desired (x, des: time from the path start, deg); None when the log has no plan columns."""
    if any(c not in L for c in PLAN_COLS):
        return None
    P = L.dropna(subset=list(PLAN_COLS)).sort_values('run_t')
    if len(P) < 2:
        return None
    xp = P.run_t.to_numpy() - t0
    series = {'ref': (xp, np.degrees(np.unwrap(P.yaw_ref0.to_numpy(float)))),
              'node0': (xp, _yaw_deg(P[['qw', 'qx', 'qy', 'qz']].to_numpy())),
              'node1': (xp + NODE_DT, np.degrees(np.unwrap(P.plan_yaw1.to_numpy(float)))),
              'node5': (xp + 5 * NODE_DT, np.degrees(np.unwrap(P.plan_yaw5.to_numpy(float))))}
    out = {}
    for name, (xs, y) in series.items():
        y = y + 360.0 * np.round(np.median(np.interp(xs, x, des) - y) / 360.0)
        out[name] = _delay(xs, y, x, des, m)
    return out


def track(arg, old_law=False):
    run, logs, _, label, _ = rig_summary.resolve(arg)
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    csvs = sorted(run_logs.planner_csvs(logs), key=lambda p: -os.path.getsize(p))
    L = pd.read_csv(csvs[0])
    P = json.load(open(os.path.join(os.path.dirname(csvs[0]), 'params.json')))
    L['run_t'] = L.sim_time - (_lift(L.sim_time, L.load_z) - _lift(d.t, d.payload_z))
    cls = OldLineSpin if old_law else LoadTrajectory
    traj = cls.from_params(P)
    t = d.t.to_numpy()
    yaw = _yaw_deg(d[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy())
    on = L[L.traj_t > 0]
    if on.empty:
        raise SystemExit(f'{label}: the path never started')
    t0, T = float(on.run_t.min()), float(on.traj_t.max())
    tt = np.interp(t, L.run_t, L.traj_t)
    datum = float(np.interp(t0, t, yaw))
    des = np.array([np.degrees(traj.yaw_at(x)[0]) for x in tt]) + datum
    rate = np.array([np.degrees(traj.yaw_at(x)[1]) for x in tt])
    x = t - t0
    m = (x >= 0) & (x <= T)
    err = yaw - des
    delay, rms_after = _delay(x, yaw, x, des, m)
    dt = float(np.median(np.diff(t)))
    n1 = max(3, int(round(1.0 / dt)))
    hp = err - np.convolve(err, np.ones(n1) / n1, mode='same')
    mid = (x >= 2.0) & (x <= T - 2.0)
    seg = hp[mid]
    f = np.fft.rfftfreq(seg.size, dt)
    A = np.abs(np.fft.rfft(seg - seg.mean()))
    band = (f > 0.3) & (f < 5.0)
    return {'run': label, 'kind': P['load_traj'], 'path_s': T,
            'err_mean': float(np.mean(err[m])), 'err_rms': float(np.sqrt(np.mean(err[m] ** 2))),
            'err_max': float(np.max(np.abs(err[m]))), 'delay_s': delay, 'rms_after_delay': rms_after,
            'wobble_sd': float(np.std(seg)), 'wobble_pp': float(np.ptp(seg)),
            'wobble_hz': float(f[band][np.argmax(A[band])]) if band.any() else float('nan'),
            'turned': float(yaw[m][-1] - yaw[m][0]), 'turn_des': float(des[m][-1] - des[m][0]),
            'peak_rate_des': float(np.max(rate[m])), 'plan': plan_lag(L, t0, x, des, m),
            'series': (x, yaw, des, err, rate, T)}


def plot(r, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import plot_style as ps
    x, yaw, des, err, rate, T = r['series']
    on = (x >= -3) & (x <= T + 3)
    ps.use()
    fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, layout='constrained')
    ax[0].plot(x[on], des[on], color='#555555', ls='--', lw=1.2, label='desired')
    ax[0].plot(x[on], yaw[on], color=ps.PAYLOAD_LINE, lw=1.4, label='ring (mocap)')
    ax[0].set_ylabel('yaw (deg, anticlockwise\nseen from above)')
    ax[0].legend(loc='upper left')
    ax[1].plot(x[on], err[on], color='#e41a1c', lw=1.2)
    ax[1].axhline(0, color='#999999', lw=0.6)
    ax[1].set_ylabel('ring - desired (deg)')
    ax[1].set_xlabel('time from the path start (s)')
    for a in ax:
        a.axvline(0, color='#999999', lw=0.6, ls=':')
        a.axvline(T, color='#999999', lw=0.6, ls=':')
    fig.suptitle(f'{r["run"]}: yaw error mean {r["err_mean"]:+.1f}, rms {r["err_rms"]:.1f}, max {r["err_max"]:.1f} deg; '
                 f'delay {r["delay_s"]:+.2f} s; turned {r["turned"]:.0f} of {r["turn_des"]:.0f} deg', fontsize=10)
    fig.savefig(path, dpi=110)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--old-law', action='store_true', help='line_spin flown before 22 Oct (1 rad per radius)')
    ap.add_argument('--png', help='figure path (one run only)')
    ap.add_argument('--plan', action='store_true', help="the plan's delay at nodes 0/1/5 (planner logs from 23 Oct)")
    a = ap.parse_args(argv)
    for arg in a.runs:
        r = track(arg, a.old_law)
        print(f'{r["run"]} ({r["kind"]}, path {r["path_s"]:.1f} s): error mean {r["err_mean"]:+.1f} rms {r["err_rms"]:.1f} '
              f'max {r["err_max"]:.1f} deg | delay {r["delay_s"]:+.2f} s (rms {r["rms_after_delay"]:.1f} after) | '
              f'wobble sd {r["wobble_sd"]:.2f} p-p {r["wobble_pp"]:.1f} deg at {r["wobble_hz"]:.2f} Hz | '
              f'turned {r["turned"]:.0f} of {r["turn_des"]:.0f} deg (peak rate {r["peak_rate_des"]:.1f} deg/s)')
        if a.plan:
            pl = r['plan']
            print('  plan: no plan yaw columns in the planner log (logged from 23 Oct)' if pl is None else
                  '  plan delay s (rms deg after): ' + ' | '.join(
                      f'{k} {pl[k][0]:+.2f} ({pl[k][1]:.1f})' for k in ('ref', 'node0', 'node1', 'node5')))
        if a.png:
            plot(r, a.png)


if __name__ == '__main__':
    main()
