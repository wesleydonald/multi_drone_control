#!/usr/bin/env python3
"""tools/loop_timing.py -- the control loop's delays, rig and twin (card docs/experiments/2026-10-23_pair_mode_timing).

    python3 tools/loop_timing.py bag h2 h2slope              # rig bag: mocap, ELRS command, radio, fc_attitude
    python3 tools/loop_timing.py att h2 R2381 --split 24     # stick -> body rate (rig: --src bag (default) | log)
    python3 tools/loop_timing.py lift h3 R2381 --split 24    # throttle -> lift, from the whole system's momentum
    python3 tools/loop_timing.py filter h2 R2381             # what the mocap filters add to the trackers' x0

A run is a rig name (the latest day that has it, or day/name) or a twin run Rnnnn. Every delay is a fit of
y ~ K lag_T(x(t - tau)) on a 1 kHz grid, both signals zero-phase band-passed (--band, 1-10 Hz), by grid search over
tau (1 ms) and T (5 ms); printed as tau, T, the equivalent delay tau + T, the phase lag at 1.4 Hz and R2.
  att     the commanded body rate (100 deg/s x stick, the rig's rate profile) against the body rate from the
          quaternions (central difference); rig bag: the recorded ELRSCommand against the stamped mocap; log: the
          tracker's own command and the pose it was handed (rig and twin)
  lift    the summed commanded thrust (0.55 kg x 35.2 x (thr_out - thr_off) x R33 per quad) against the vertical
          momentum of the quads and the ring: the cables' pulls are internal, so the ring's swing drops out
  filter  x0's body rate and vertical velocity against the raw ones from the poses (the mocap filters' lag)
Windows: from --from s after the ring's lift-off (+5 cm) to the first LAND / DROP / DETECTED / FLEET_ABORT; --split T
fits before and after T (tracker clock: sim time in the twin). Read-only; --csv writes the rows.
"""
import argparse
import csv
import glob
import json
import os
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STOPS = ('LAND', 'DROP', 'DETECTED', 'FLEET_ABORT')
M_Q, M_RING, KT = 0.55, 0.86, 35.2
STICK_DPS = 100.0


# ── runs ─────────────────────────────────────────────────────────────────────

def rig_base(arg):
    """results/rig/<day>/<name> for 'name' (the latest day that has it) or 'day/name'."""
    if '/' in arg:
        day, name = arg.rstrip('/').split('/')[-2:]
        return os.path.join(REPO, 'results', 'rig', day, name)
    hits = sorted(glob.glob(os.path.join(REPO, 'results', 'rig', '*', f'{arg}_logs')))
    if not hits:
        raise SystemExit(f'loop_timing: no rig run {arg} (no results/rig/*/{arg}_logs)')
    return hits[-1][:-len('_logs')]


def resolve(arg):
    """kind, label, tracker log paths (by drone), stop time on the tracker clock (or None), bag dir, lab_env."""
    import pandas as pd
    if arg[:1] == 'R' and arg[1:].isdigit():
        hits = sorted(glob.glob(os.path.join(REPO, 'results', '*', f'{arg}_*')))
        if not hits:
            raise SystemExit(f'loop_timing: no twin run {arg}')
        run = hits[0]
        trackers = sorted(glob.glob(os.path.join(run, 'logs', 'tracker', 'drone*', 'log.csv')))
        ev_path, t0, bag = os.path.join(run, 'logs', 'events.csv'), 0.0, None
        man = os.path.join(run, 'manifest.json')
        env = (json.load(open(man)).get('lab_env') or {}) if os.path.exists(man) else {}
        kind, label = 'twin', arg
    else:
        base = rig_base(arg)
        trackers = sorted(glob.glob(base + '_logs/logs/tracker/drone*/log.csv'))
        ev_path, env, kind = base + '_run/logs/events.csv', {}, 'rig'
        try:
            t0 = json.load(open(base + '_run/manifest.json'))['t0_epoch']
        except (OSError, KeyError):
            t0 = None
        bag = base if os.path.isdir(base) else None
        label = '/'.join(base.split('/')[-2:])
    if not trackers:
        raise SystemExit(f'loop_timing: {arg} has no tracker logs')
    stop = None
    if t0 is not None and os.path.exists(ev_path):
        ev = pd.read_csv(ev_path)
        st = ev[ev.event.isin(STOPS)]
        stop = float(t0 + st.sim_time.min()) if len(st) else None
    return kind, label, trackers, stop, bag, env


def tracker(path):
    import pandas as pd
    T = pd.read_csv(path).drop_duplicates('sim_time').reset_index(drop=True)
    return T[T.pose_z.notna()].reset_index(drop=True)


def drone_of(path):
    return int(os.path.basename(os.path.dirname(path)).split('_')[0].replace('drone', ''))


def airborne(T, stop, t_from=1.0, t_to=None):
    """(lo, hi) on the tracker clock: the ring's lift-off (+5 cm) + t_from to its last airborne sample - 0.5 s, or
    stop."""
    t = T.sim_time.to_numpy()
    z = T.payload_z.to_numpy() if 'payload_z' in T and T.payload_z.notna().sum() > 50 else T.pose_z.to_numpy() - 0.05
    ok = np.isfinite(z)
    air = np.where(ok & (z > np.nanmedian(z[ok][:20]) + 0.05))[0]
    if not len(air):
        return None
    lo = t[air[0]] + t_from
    hi = t[air[-1]] - 0.5 if t_to is None else t[air[0]] + t_to
    if stop is not None:
        hi = min(hi, stop)
    return lo, hi


def spans(lo, hi, split):
    if split is None:
        return [(lo, hi, 'window')]
    return [(lo, min(hi, split - 0.5), 'before'), (max(lo, split + 0.5), hi, 'after')]


# ── signals and the fit ──────────────────────────────────────────────────────

def body_rate(t, q):
    """Body rate (rad/s, (n, 3)) from quaternions q (n, 4) wxyz by central difference; NaN at the ends."""
    a, b = q[:-2], q[2:].copy()
    b *= np.sign(np.sum(a * b, axis=1))[:, None]
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    r = np.c_[aw * bx - ax * bw - ay * bz + az * by, aw * by + ax * bz - ay * bw - az * bx,
              aw * bz - ax * by + ay * bx - az * bw]
    dt = t[2:] - t[:-2]
    w = np.full((len(t), 3), np.nan)
    ok = dt > 0
    w[1:-1][ok] = 2.0 * r[ok] / dt[ok, None]
    return w


def central(t, x):
    v = np.full(len(x), np.nan)
    v[1:-1] = (x[2:] - x[:-2]) / (t[2:] - t[:-2])
    return v


def zoh(t_src, x_src, tg):
    idx = np.clip(np.searchsorted(t_src, tg, side='right') - 1, 0, len(t_src) - 1)
    return np.asarray(x_src)[idx]


def lpf1(x, T, dt):
    from scipy.signal import lfilter
    if T <= 0:
        return x
    a = dt / (T + dt)
    return lfilter([a], [1.0, -(1.0 - a)], x, zi=[x[0] * (1.0 - a)])[0]


def fopdt(x, y, dt, taus=np.arange(0.0, 0.25, 0.001), Ts=np.arange(0.0, 0.151, 0.005)):
    """(K, tau, T, R2) of the best y ~ K lag_T(x(t - tau)) on a uniform grid of step dt, or None."""
    if np.var(x) <= 0 or np.var(y) <= 0:
        return None
    best = (np.inf, None)
    for T in Ts:
        xf = lpf1(x, T, dt)
        for tau in taus:
            n = int(round(tau / dt))
            xs, ys = (xf[:len(xf) - n], y[n:]) if n else (xf, y)
            K = (xs @ ys) / (xs @ xs)
            r = ys - K * xs
            sse = (r @ r) / len(r)
            if sse < best[0]:
                best = (sse, (float(K), float(tau), float(T), float(1.0 - sse / np.var(ys))))
    return best[1]


def fit(t_x, x, t_y, y, lo, hi, band, integrate=False, hold=True, step=0.001):
    """The fit of y against x over [lo, hi]: x held between its samples (a command) or, hold False, interpolated (a
    measurement); integrate: x's running integral."""
    from scipy.signal import butter, sosfiltfilt
    if hi - lo < 3.0:
        return None
    tg = np.arange(lo, hi, step)
    xg = zoh(t_x, x, tg) if hold else np.interp(tg, t_x, x)
    if integrate:
        xg = np.cumsum(xg - xg.mean()) * step
    ok = np.isfinite(y)
    if ok.sum() < 10:
        return None
    yg = np.interp(tg, np.asarray(t_y)[ok], np.asarray(y)[ok])
    sos = butter(2, band, btype='band', fs=1.0 / step, output='sos')
    return fopdt(sosfiltfilt(sos, xg), sosfiltfilt(sos, yg), step)


def row(label, drone, span, what, r, extra=''):
    out = {'run': label, 'drone': drone, 'window': span, 'signal': what, 'note': extra}
    if r is None:
        return dict(out, tau_ms='', T_ms='', eq_ms='', lag14_deg='', K='', R2='')
    K, tau, T, r2 = r
    lag = np.degrees(2 * np.pi * 1.4 * tau + np.arctan(2 * np.pi * 1.4 * T))
    return dict(out, tau_ms=f'{1000 * tau:.0f}', T_ms=f'{1000 * T:.0f}', eq_ms=f'{1000 * (tau + T):.0f}',
                lag14_deg=f'{lag:.0f}', K=f'{K:+.2f}', R2=f'{r2:.2f}')


# ── the rig's bag ────────────────────────────────────────────────────────────

def bag_arrays(bag, cache=None):
    """The bag's timing streams, cached as npz: mocap (t_rec, stamp, p xyz, q wxyz, v xyz, w xyz), ELRSCommand (t_rec,
    armed, ch0-3), fc_attitude (t_rec, roll, pitch, yaw), telemetry (t_rec, V); radio_health and /fleet/command as
    __t / __s pairs."""
    cache = cache or os.path.join(tempfile.gettempdir(), 'loop_timing')
    os.makedirs(cache, exist_ok=True)
    day, name = bag.rstrip('/').split('/')[-2:]
    npz = os.path.join(cache, f'{day}_{name}.npz')
    if os.path.exists(npz):
        return dict(np.load(npz))
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [k for k in types if k.endswith(('motion_capture_state', 'magnet_tip_state', 'ELRSCommand', 'fc_attitude',
                                            'radio_health', 'telemetry')) or k == '/fleet/command']
    cls = {k: get_message(types[k]) for k in want}
    rows = {k: [] for k in want}
    while r.has_next():
        topic, data, t_rec = r.read_next()
        if topic not in cls:
            continue
        m, tr, ty = deserialize_message(data, cls[topic]), t_rec * 1e-9, types[topic]
        if ty == 'interfaces/msg/MotionCaptureState':
            p, q, v, w = m.pose.position, m.pose.orientation, m.twist.linear, m.twist.angular
            rows[topic].append((tr, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z,
                                q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z))
        elif ty == 'interfaces/msg/ELRSCommand':
            rows[topic].append((tr, float(m.armed), m.channel_0, m.channel_1, m.channel_2, m.channel_3))
        elif topic.endswith('fc_attitude'):
            d = list(m.data) + [np.nan] * 3
            rows[topic].append((tr, d[0], d[1], d[2]))
        elif ty == 'interfaces/msg/Telemetry':
            rows[topic].append((tr, m.battery_voltage))
        else:
            rows[topic].append((tr, str(m.data)))
    out = {}
    for k, v in rows.items():
        key = k.strip('/').replace('/', '__')
        if k == '/fleet/command' or k.endswith('radio_health'):
            out[key + '__t'] = np.array([x[0] for x in v])
            out[key + '__s'] = np.array([x[1] for x in v])
        else:
            out[key] = np.array(v, dtype=float)
    np.savez(npz, **out)
    return out


def stamped(A):
    """Mocap rows in header-stamp order, repeated stamps out."""
    A = A[np.argsort(A[:, 1])]
    return A[np.r_[True, np.diff(A[:, 1]) > 1e-6]]


def bag_rows(label, B, lo, hi):
    rows = []
    for k in sorted(B):
        if k.endswith('motion_capture_state') and B[k].ndim == 2 and len(B[k]) > 100:
            A = B[k]
            m = (A[:, 1] >= lo) & (A[:, 1] <= hi)
            d = np.diff(np.sort(A[m, 1]))
            lat = 1000 * (A[m, 0] - A[m, 1])
            rows.append({'run': label, 'stream': k.split('__')[0] + ' mocap', 'rate_hz': f'{1 / np.median(d):.1f}',
                         'gap_p95_ms': f'{1000 * np.percentile(d, 95):.1f}', 'gap_max_ms': f'{1000 * d.max():.1f}',
                         'note': f'stamp to recorder {np.median(lat):.2f} ms (p95 {np.percentile(lat, 95):.2f})'})
    for k in sorted(B):
        if k.endswith(('ELRSCommand', 'fc_attitude')) and B[k].ndim == 2 and len(B[k]) > 10:
            A = B[k]
            t = np.sort(A[(A[:, 0] >= lo) & (A[:, 0] <= hi), 0])
            d = np.diff(t)
            if not len(d):
                continue
            rows.append({'run': label, 'stream': k.replace('__', ' '), 'rate_hz': f'{1 / np.median(d):.1f}',
                         'gap_p95_ms': f'{1000 * np.percentile(d, 95):.1f}', 'gap_max_ms': f'{1000 * d.max():.1f}',
                         'note': f'{100 * np.mean(d > 0.030):.1f} % of gaps > 30 ms; recorded over {t[-1] - t[0]:.1f} '
                                 f'of the {hi - lo:.1f} s airborne'})
    for k in sorted(B):
        if k.endswith('radio_health__t'):
            ts, ss = B[k], B[k[:-1] + 's']
            recs = []
            for s in ss[(ts >= lo) & (ts <= hi)]:
                try:
                    recs.append(json.loads(str(s)))
                except ValueError:
                    pass
            lq = [r['lq'] for r in recs if r.get('lq') is not None]
            rate = [r['cmd_rate_hz'] for r in recs if r.get('cmd_rate_hz') is not None]
            if recs:
                rows.append({'run': label, 'stream': k.split('__')[0] + ' radio', 'rate_hz':
                             f'{np.median(rate):.1f}' if rate else '', 'gap_p95_ms': '', 'gap_max_ms': '',
                             'note': f'LQ median {np.median(lq):.0f}, min {min(lq):.0f}' if lq else ''})
    return rows


def bag_window(B):
    P = stamped(B['payload__motion_capture_state'])
    air = np.where(P[:, 4] > np.median(P[:200, 4]) + 0.05)[0]
    return P[air[0], 1], P[air[-1], 1]


# ── the subcommands ──────────────────────────────────────────────────────────

def att_rows(arg, a):
    kind, label, trackers, stop, bag, _ = resolve(arg)
    src = 'log' if kind == 'twin' else a.src
    B = bag_arrays(bag, a.cache) if src == 'bag' else None
    if src == 'bag' and B is None:
        raise SystemExit(f'loop_timing: {arg} has no bag')
    rows = []
    for f in trackers:
        T, i = tracker(f), drone_of(f)
        w = airborne(T, stop, a.t_from, a.t_to)
        if w is None:
            continue
        for lo, hi, lab in spans(*w, a.split):
            if src == 'bag':
                C = B[f'drone_{i}__ELRSCommand']
                C = C[np.argsort(C[:, 0])]
                M = stamped(B[f'drone_{i}__motion_capture_state'])
                wb = body_rate(M[:, 1], M[:, 5:9])
                for ax, ch, k in (('roll', 2, 0), ('pitch', 3, 1)):
                    cmd = np.radians(STICK_DPS * np.clip(C[:, ch], -1, 1))
                    r = fit(C[:, 0], cmd, M[:, 1], wb[:, k], lo, hi, a.band)
                    rows.append(row(label, i, lab, f'att {ax} (bag)', r))
                continue
            t = T.sim_time.to_numpy()
            wb = body_rate(t, T[['pose_qw', 'pose_qx', 'pose_qy', 'pose_qz']].to_numpy())
            for ax, uc, k in (('roll', 'u0', 0), ('pitch', 'u1', 1)):
                cmd = np.radians(STICK_DPS * np.clip(T[uc].to_numpy(), -1, 1))
                rows.append(row(label, i, lab, f'att {ax} (log)', fit(t, cmd, t, wb[:, k], lo, hi, a.band)))
    return rows


def momentum_series(T_list, tg, ring_z):
    """(summed commanded vertical thrust, vertical momentum) on grid tg from the tracker logs."""
    t0 = T_list[0].sim_time.to_numpy()
    mom = M_RING * np.interp(tg, t0, central(t0, ring_z))
    F = np.zeros_like(tg)
    for T in T_list:
        t = T.sim_time.to_numpy()
        mom += M_Q * np.interp(tg, t, central(t, T.pose_z.to_numpy()))
        r33 = 1 - 2 * (T.pose_qx.to_numpy() ** 2 + T.pose_qy.to_numpy() ** 2)
        F += zoh(t, M_Q * KT * np.clip(T.thr_out.to_numpy() - T.thr_off.to_numpy(), 0, None) * r33, tg)
    return F, mom


def lift_rows(arg, a):
    kind, label, trackers, stop, bag, _ = resolve(arg)
    Ts = [tracker(f) for f in trackers]
    Ts = [T[T.thr_out.notna()].reset_index(drop=True) for T in Ts]
    w = airborne(Ts[0], stop, a.t_from, a.t_to)
    if w is None:
        return [row(label, 'all', 'never airborne', 'lift', None)]
    src = 'log' if kind == 'twin' or bag is None else a.src
    rows = []
    for lo, hi, lab in spans(*w, a.split):
        tg = np.arange(lo - 1.0, hi + 1.0, 0.005)
        if src == 'bag':
            B = bag_arrays(bag, a.cache)
            P = stamped(B['payload__motion_capture_state'])
            mom = M_RING * np.interp(tg, P[:, 1], central(P[:, 1], P[:, 4]))
            F = np.zeros_like(tg)
            for f, T in zip(trackers, Ts):
                i = drone_of(f)
                M = stamped(B[f'drone_{i}__motion_capture_state'])
                mom += M_Q * np.interp(tg, M[:, 1], central(M[:, 1], M[:, 4]))
                C = B[f'drone_{i}__ELRSCommand']
                C = C[np.argsort(C[:, 0])]
                off = np.interp(C[:, 0], T.sim_time.to_numpy(), T.thr_off.to_numpy())
                qx, qy = np.interp(C[:, 0], M[:, 1], M[:, 6]), np.interp(C[:, 0], M[:, 1], M[:, 7])
                F += zoh(C[:, 0], M_Q * KT * np.clip((C[:, 4] + 1) / 2 - off, 0, None) * (1 - 2 * (qx ** 2 + qy ** 2)),
                         tg)
        else:
            F, mom = momentum_series(Ts, tg, Ts[0].payload_z.to_numpy())
        ok = np.isfinite(mom)
        rows.append(row(label, f'all {len(Ts)}', lab, f'lift ({src})',
                        fit(tg, F, tg[ok], mom[ok], lo, hi, a.band, integrate=True)))
    return rows


def filter_rows(arg, a):
    _, label, trackers, stop, _, _ = resolve(arg)
    rows = []
    for f in trackers:
        T, i = tracker(f), drone_of(f)
        w = airborne(T, stop, a.t_from, a.t_to)
        if w is None:
            continue
        t = T.sim_time.to_numpy()
        wb = body_rate(t, T[['pose_qw', 'pose_qx', 'pose_qy', 'pose_qz']].to_numpy())
        for lo, hi, lab in spans(*w, a.split):
            for ax, k in (('x', 0), ('y', 1)):
                col = f'x0_w{ax}' if f'x0_w{ax}' in T else f'w{ax}'
                ok = np.isfinite(wb[:, k])
                rows.append(row(label, i, lab, f'body rate {ax}: raw -> x0', fit(
                    t[ok], wb[ok, k], t, T[col].to_numpy(), lo, hi, a.band, hold=False)))
            vz = central(t, T.pose_z.to_numpy())
            ok = np.isfinite(vz)
            r = fit(t[ok], vz[ok], t, T.x0_vz.to_numpy(), lo, hi, a.band, hold=False)
            rows.append(row(label, i, lab, 'vz: raw -> x0', r))
    return rows


# ── main ─────────────────────────────────────────────────────────────────────

COLUMNS = {'bag': ['run', 'stream', 'rate_hz', 'gap_p95_ms', 'gap_max_ms', 'note'],
           'fit': ['run', 'drone', 'window', 'signal', 'tau_ms', 'T_ms', 'eq_ms', 'lag14_deg', 'K', 'R2', 'note']}


def table(rows, cols):
    print('| ' + ' | '.join(cols) + ' |')
    print('|' + '---|' * len(cols))
    for r in rows:
        print('| ' + ' | '.join(str(r.get(c, '')) if r.get(c, '') != '' else '-' for c in cols) + ' |')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', choices=('bag', 'att', 'lift', 'filter'))
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--src', choices=('bag', 'log'), default='bag', help='rig source for att and lift')
    ap.add_argument('--band', type=float, nargs=2, default=(1.0, 10.0))
    ap.add_argument('--from', dest='t_from', type=float, default=1.0, help='s after the lift-off')
    ap.add_argument('--to', dest='t_to', type=float, default=None, help='s after the lift-off')
    ap.add_argument('--split', type=float, default=None, help='fit before / after this time (tracker clock)')
    ap.add_argument('--cache', default=None, help='folder for the bag npz caches (default: the temp dir)')
    ap.add_argument('--csv')
    a = ap.parse_args(argv)
    rows = []
    for arg in a.runs:
        if a.what == 'bag':
            _, label, _, _, bag, _ = resolve(arg)
            if bag is None:
                raise SystemExit(f'loop_timing: {arg} has no bag')
            B = bag_arrays(bag, a.cache)
            rows += bag_rows(label, B, *bag_window(B))
        else:
            rows += {'att': att_rows, 'lift': lift_rows, 'filter': filter_rows}[a.what](arg, a)
    cols = COLUMNS['bag' if a.what == 'bag' else 'fit']
    table(rows, cols)
    if a.csv:
        with open(a.csv, 'w', newline='') as fh:
            wr = csv.DictWriter(fh, fieldnames=cols, extrasaction='ignore', lineterminator='\n')
            wr.writeheader()
            wr.writerows(rows)
    return rows


if __name__ == '__main__':
    main()
