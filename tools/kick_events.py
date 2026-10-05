#!/usr/bin/env python3
"""
tools/kick_events.py -- output-side command kicks in the tracker logs, frozen from the
M-kicks analysis (registry row M-kicks, card docs/experiments/2026-09-28_mocap_diff_window.md).

    tools/kick_events.py R0702 R0703                     # run ids or run directories
    tools/kick_events.py R0702 -o kicks.csv --summary summary.csv

Definitions (exact; change them only with a new registry row, every past count depends on them):

  window      per drone: throttle u2 > 0.1 and solve_status == 0, minus the first 150 ticks after
              the first such row, from TAKEOFF + 3 s to LAND (events.csv), minus the first and last
              3 ticks. The drone named by DETACH is masked from DETACH to REWELD + 2 s (to the end of
              the log if it never re-welds).
  kick tick   one-tick |d u2| >= 0.02 or |d u3| >= 0.02 inside the window (> 99.9th pct of normal).
  event       kick ticks of one drone merged: a kick within 0.3 s of the previous kick tick
              (chained) belongs to the same event; the event sits on its first tick k.
  scheduled   no rate jump (max |dw| over k-1..k < 0.2) AND (a reference step at k or k-1,
              |d ref_p| > 0.015 m or |d qref| > 0.02, OR another drone has an event within 0.1 s).
              Scheduled events are fleet-wide steps of the mission, not kicks.
  normal tick a window tick that is not a kick tick.

Flags, each true if its condition holds at k or k-1 (stall: anywhere in k-5..k):
  rate_jump          |w[k] - w[k-1]| > 0.5 rad/s (logged x0 body rate)
  rate_inconsistent  logged w differs by > 0.5 rad/s from the body rate of the pose quaternion path
                     (central difference k-1..k+1, sim time), or a spike-and-return
                     (|dw| > 0.5 and |w[k+1] - w[k-1]| < 0.5 |dw|)
  real_rotation      rate_jump whose tick is not rate_inconsistent. Per tick, then k or k-1: the RETURN
                     tick of a one-tick glitch also counts, so rate_jump & ~rate_inconsistent is the
                     strict 'real rotation' set
  stale_pose         pose and rate columns bit-identical to the previous tick
  odd_tick_dt        tick dt > 30 ms or < 10 ms
  stall5             a tick dt > 30 ms within the last 6 ticks (k-5..k)
  pose_jump          |d v_pose| above max(0.3 m/s, 99.9th pct in the window), v_pose = d pose / dt
  ref_step           the reference step used by `scheduled`

Scored per airborne drone-second: unscheduled events / sum of tick dt over the window ticks of all
drones (reported per 100 drone-s), because aborted runs fly shorter.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import run_logs  # noqa: E402

THR_KICK = 0.02        # |d throttle| per tick
YAW_KICK = 0.02        # |d yaw command| per tick
MERGE_S = 0.3
SYNC_S = 0.1
RATE_JUMP = 0.5        # rad/s per tick
RATE_RESID = 0.5       # rad/s, logged w vs quaternion path
PLANNED_DW = 0.2
REF_DP, REF_DQ = 0.015, 0.02
DT_LONG, DT_SHORT = 0.03, 0.01
SKIP_TICKS = 150
TAKEOFF_SKIP_S = 3.0
REWELD_PAD_S = 2.0
EDGE_TICKS = 3
POSE_JUMP_MIN = 0.3

FLAGS = ['rate_jump', 'rate_inconsistent', 'real_rotation', 'stale_pose', 'odd_tick_dt',
         'stall5', 'pose_jump', 'ref_step']
POSE_COLS = ['pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
             'wx', 'wy', 'wz']


def quat_rate(q1, q2, dt):
    """Body-frame rate taking q1 to q2 (w,x,y,z rows) over dt, as the mocap emulator does:
    2 vec(q2 q1^-1) / dt, sign-fixed to the short way, rotated into the body frame of q1."""
    w1, x1, y1, z1 = q1.T
    w2, x2, y2, z2 = q2.T
    cw, cx, cy, cz = w1, -x1, -y1, -z1
    rw = w2 * cw - x2 * cx - y2 * cy - z2 * cz
    rx = w2 * cx + x2 * cw + y2 * cz - z2 * cy
    ry = w2 * cy - x2 * cz + y2 * cw + z2 * cx
    rz = w2 * cz + x2 * cy - y2 * cx + z2 * cw
    s = np.sign(rw)
    s[s == 0] = 1
    wv = 2 * np.stack([rx, ry, rz], 1) * s[:, None] / dt[:, None]
    w, x, y, z = q1.T
    R = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)
    return np.einsum('nji,nj->ni', R, wv)


def _at(flag):
    return flag | np.r_[False, flag[:-1]]


def mission_times(run):
    """TAKEOFF, LAND, REWELD times and (DETACH time, detached drone or None) from events.csv."""
    f = os.path.join(run, 'logs', 'events.csv')
    out = dict(takeoff=None, land=None, reweld=None, detach=None)
    if not os.path.exists(f):
        return out
    ev = pd.read_csv(f)
    for key, name in [('takeoff', 'TAKEOFF'), ('land', 'LAND'), ('reweld', 'REWELD')]:
        x = ev[ev.event == name].sim_time
        out[key] = float(x.iloc[0]) if len(x) else None
    e = ev[ev.event == 'DETACH']
    if len(e):
        a = str(e.arg.iloc[0])
        out['detach'] = (float(e.sim_time.iloc[0]),
                         int(float(a)) if a.replace('.', '').isdigit() else None)
    return out


def drone_ticks(d, drone, mt):
    """Per-tick frame for one tracker log: window, kick, event start, flags and helper columns."""
    d = d.reset_index(drop=True)
    t = d.sim_time.values
    dt = np.r_[np.nan, np.diff(t)]
    q = d[['pose_qw', 'pose_qx', 'pose_qy', 'pose_qz']].values
    W = d[['wx', 'wy', 'wz']].values

    wq = np.full_like(W, np.nan)
    dtc = t[2:] - t[:-2]
    ok = dtc > 0.01
    tmp = quat_rate(q[:-2], q[2:], np.where(ok, dtc, 1.0))
    tmp[~ok] = np.nan
    wq[1:-1] = tmp
    resid = np.linalg.norm(W - wq, axis=1)

    dWv = np.r_[[np.full(3, np.nan)], np.diff(W, axis=0)]
    dW = np.linalg.norm(dWv, axis=1)
    ret = np.r_[np.nan, np.linalg.norm(W[2:] - W[:-2], axis=1), np.nan]
    with np.errstate(invalid='ignore'):
        spike = (dW > RATE_JUMP) & (ret < 0.5 * dW)
    stale = np.r_[False, np.abs(np.diff(d[POSE_COLS].values, axis=0)).sum(1) == 0]

    dref = np.r_[np.nan, np.linalg.norm(np.diff(d[['ref_x', 'ref_y', 'ref_z']].values, axis=0), axis=1)]
    dqr = np.r_[np.nan, np.linalg.norm(np.diff(
        d[['qref_w', 'qref_x', 'qref_y', 'qref_z']].values, axis=0), axis=1)]
    P = d[['pose_x', 'pose_y', 'pose_z']].values
    vp = np.full_like(P, np.nan)
    g = dt > 0.005
    vp[g] = np.r_[[np.zeros(3)], np.diff(P, axis=0)][g] / dt[g, None]
    dvp = np.r_[np.nan, np.linalg.norm(np.diff(vp, axis=0), axis=1)]
    du2 = np.r_[np.nan, np.abs(np.diff(d.u2.values))]
    du3 = np.r_[np.nan, np.abs(np.diff(d.u3.values))]

    air = (d.u2.values > 0.1) & (d.solve_status.values == 0)
    air[:np.argmax(air) + SKIP_TICKS] = False
    if mt['takeoff'] is not None:
        air &= t >= mt['takeoff'] + TAKEOFF_SKIP_S
    if mt['land'] is not None:
        air &= t < mt['land']
    det = mt['detach']
    if det is not None and (det[1] == drone or det[1] is None):
        rew = mt['reweld']
        air &= ~((t >= det[0]) & ((t <= rew + REWELD_PAD_S) if rew else True))
    air[:EDGE_TICKS] = False
    air[-EDGE_TICKS:] = False

    thr_v = max(POSE_JUMP_MIN, np.nanpercentile(dvp[air], 99.9)) if air.any() else POSE_JUMP_MIN
    with np.errstate(invalid='ignore'):
        kick = air & ((du2 >= THR_KICK) | (du3 >= YAW_KICK))
        long_ = np.nan_to_num(dt > DT_LONG).astype(float)
        stall5 = np.convolve(long_, np.ones(6), 'full')[:len(t)] > 0
        art = (resid > RATE_RESID) | spike
        fl = dict(rate_jump=_at(dW > RATE_JUMP), rate_inconsistent=_at(art),
                  real_rotation=_at((dW > RATE_JUMP) & ~art), stale_pose=_at(stale),
                  odd_tick_dt=_at((dt > DT_LONG) | (dt < DT_SHORT)), stall5=stall5,
                  pose_jump=_at(dvp > thr_v), ref_step=_at((dref > REF_DP) | (dqr > REF_DQ)))

    start = np.zeros(len(t), bool)
    last = -1e9
    for k in np.where(kick)[0]:
        if t[k] - last >= MERGE_S:
            start[k] = True
        last = t[k]

    dWmax = np.r_[np.nan, np.fmax(dW[:-1], dW[1:])]       # max over k-1..k
    out = pd.DataFrame(dict(drone=drone, k=np.arange(len(t)), t=t, dt=dt, air=air, kick=kick,
                            event=start, du2=du2, du3=du3, dW=dW, dWmax=dWmax, resid=resid,
                            **{n: v for n, v in fl.items()}))
    out['resid_max'] = np.r_[np.nan, np.fmax(resid[:-1], resid[1:])]
    return out, d


def tracker_logs(run):
    return run_logs.trackers(run)


def run_ticks(run):
    """All drones of one run: (per-tick frame with an `event` start column and `scheduled` on event
    rows, {drone: raw log frame})."""
    mt = mission_times(run)
    frames, raw = [], {}
    for dr, f in tracker_logs(run).items():
        tk, d = drone_ticks(pd.read_csv(f), dr, mt)
        frames.append(tk)
        raw[dr] = d
    T = pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()
    if len(T):
        ev = T[T.event]
        sync = np.array([((ev.drone != r.drone) & ((ev.t - r.t).abs() < SYNC_S)).any()
                         for r in ev.itertuples()], bool)
        planned = (ev.dWmax.values < PLANNED_DW) & (ev.ref_step.values | sync)
        T['scheduled'] = False
        T.loc[ev.index, 'scheduled'] = planned
    return T, raw


def _cls(r):
    return ('REF' if r.ref_step else 'ART' if r.rate_inconsistent else 'STALE' if r.stale_pose
            else 'DT' if r.odd_tick_dt else 'REAL' if r.real_rotation else 'NONE')


def events_table(run_id, T):
    ev = T[T.event].copy()
    ev.insert(0, 'run', run_id)
    ev['cls'] = [_cls(r) for r in ev.itertuples()]
    cols = ['run', 'drone', 't', 'du2', 'du3', 'dWmax', 'resid_max', 'dt'] + FLAGS + ['scheduled', 'cls']
    ev = ev[cols].rename(columns={'dWmax': 'dW', 'dt': 'tick_dt'})
    for c in FLAGS + ['scheduled']:
        ev[c] = ev[c].astype(int)
    return ev.round(3)


def summary_row(run_id, T):
    air = T[T.air]
    normal = air[~air.kick]
    ev = T[T.event]
    u = ev[~ev.scheduled.astype(bool)]
    air_s = float(np.nansum(air.dt))
    r = dict(run=run_id, drones=int(T.drone.nunique()), air_drone_s=round(air_s, 1),
             events=len(ev), sched=int(ev.scheduled.sum()), unsched=len(u),
             per100ds=round(100 * len(u) / air_s, 2) if air_s else np.nan)
    rj = u[u.rate_jump]
    r['real_of_jump'] = f'{int(rj.real_rotation.sum())}/{len(rj)}'
    for c in ['rate_jump', 'rate_inconsistent', 'pose_jump', 'stale_pose', 'odd_tick_dt', 'stall5']:
        kp = f'{100 * u[c].mean():.0f}' if len(u) else '-'
        r[c] = f'{kp}/{100 * normal[c].mean():.1f}'
    return r


def resolve_run(spec):
    if os.path.isdir(spec):
        return os.path.abspath(spec)
    from run_dir import resolve
    return resolve(spec)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+', help='run ids (R0702) or run directories')
    ap.add_argument('-o', '--out', default='kick_events.csv', help='events CSV (all events, one row each)')
    ap.add_argument('--summary', help='optional per-run summary CSV')
    a = ap.parse_args(argv)
    events, rows = [], []
    for spec in a.runs:
        run = resolve_run(spec)
        rid = os.path.basename(run.rstrip('/'))[:5]
        T, _ = run_ticks(run)
        if not len(T):
            print(f'{rid}: no tracker logs', file=sys.stderr)
            continue
        events.append(events_table(rid, T))
        rows.append(summary_row(rid, T))
    if not rows:
        return 1
    S = pd.DataFrame(rows)
    pd.set_option('display.width', 250)
    print('flag columns: % of unscheduled events / % of normal window ticks; per100ds = unscheduled '
          'events per 100 airborne drone-seconds')
    print(S.to_string(index=False))
    tot_s, tot_u = S.air_drone_s.sum(), S.unsched.sum()
    print(f'ALL: {tot_u} unscheduled events in {tot_s:.0f} drone-s = {100 * tot_u / tot_s:.2f} per 100 drone-s')
    pd.concat(events, ignore_index=True).to_csv(a.out, index=False)
    if a.summary:
        S.to_csv(a.summary, index=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
