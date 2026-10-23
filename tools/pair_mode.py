#!/usr/bin/env python3
"""tools/pair_mode.py -- the 2-quad pair's 1.1-1.9 Hz mode, rig and twin on the same code
(card docs/experiments/2026-10-23_pair_mode_timing.md).

    python3 tools/pair_mode.py h2 h2slope R2381:28.5      # rig runs (latest day, or day/name); twin Rnnnn[:start s]
    python3 tools/pair_mode.py --tau-thr 0.06 h2          # the throttle -> thrust lag the power assumes

Per run, at the see-saw's peak frequency f0 (Welch, 4 s segments, 1-2 Hz), from the airborne window (or the twin's
start) to the first LAND / DROP / DETECTED / FLEET_ABORT:
  shape  amplitude per deg of see-saw, and phase against it (+ lags), of dz = (z0 - z1)/2, the pair's mean height, the
         ring along the pair's line, the quads' mean and anti-symmetric positions along it (run.csv). The headline:
         ring slide and dz in cm per deg (22 Oct: rig 0.17-0.26 and 0.15-0.18; the 60 ms twin 0.04 and 0.37)
  loop   dthr = (thr0 - thr1)/2 against -d(dz)/dt (0 deg damps, past 90 pumps); the reference trail along z and along
         the line (ms) with ref/act amplitude
  power  per quad, c = -Re(F/V) at f0 (N s/m; > 0 takes energy out, < 0 feeds the swing) for its vertical thrust and
         its thrust along the line (tilt from its attitude), 0.55 kg x 35.2 x (thr_out - thr_off) taken tau late
         (--tau-thr; default rig 45 ms, twin 15 ms plus the run's own throttle delay)
  links  rig only, from the bag's magnet markers: pivot-to-tip and tip-to-plate, mean, sd and 1.1-1.9 Hz band rms
Read-only; --csv writes one headline row per run.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import butter, csd, sosfiltfilt, welch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import loop_timing  # noqa: E402

STOPS = loop_timing.STOPS
M_Q, KT = loop_timing.M_Q, loop_timing.KT


def xspec(ref, sig, fs, f0):
    """(phase of sig behind ref in deg, coherence, sig's amplitude) at the bin nearest f0."""
    nper = min(len(ref), int(4 * fs))
    f, P = csd(ref, sig, fs=fs, nperseg=nper)
    _, Pr = welch(ref, fs=fs, nperseg=nper)
    _, Ps = welch(sig, fs=fs, nperseg=nper)
    k = int(np.argmin(np.abs(f - f0)))
    return -np.degrees(np.angle(P[k])), abs(P[k]) ** 2 / (Pr[k] * Ps[k]), np.sqrt(2 * Ps[k] * (f[1] - f[0]))


def damping(F, V, fs, f0):
    """c = -Re(F / V) at f0 (N s/m for a force against a velocity): > 0 takes energy out of the motion."""
    nper = min(len(V), int(4 * fs))
    f, Pvf = csd(V - V.mean(), F - F.mean(), fs=fs, nperseg=nper)
    _, Pvv = welch(V - V.mean(), fs=fs, nperseg=nper)
    k = int(np.argmin(np.abs(f - f0)))
    return float(-(Pvf[k] / Pvv[k]).real), float(np.sqrt(2 * Pvv[k] * (f[1] - f[0])))


def rot(q):
    """Rotation matrices (n, 3, 3) from quaternions (n, 4) wxyz."""
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def load(arg):
    """run.csv, events, tracker logs, start (twin) and the rig base path (or None)."""
    import rig_summary
    name, _, start = arg.partition(':')
    run, logs, _, label, _ = rig_summary.resolve(name)
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    trackers = sorted(glob.glob(os.path.join(logs, 'logs', 'tracker', 'drone*', 'log.csv')))
    twin = name[:1] == 'R' and name[1:].isdigit()
    man = os.path.join(run, 'manifest.json')
    env = (json.load(open(man)).get('lab_env') or {}) if twin and os.path.exists(man) else {}
    base = None if twin else loop_timing.rig_base(name)
    return label, d, ev, trackers, (float(start) if start else None), base, env


def window(d, ev, start):
    t, z = d.t.to_numpy(), d.payload_z.to_numpy()
    air = np.where(z > np.median(z[:50]) + 0.05)[0]
    st = ev[ev.event.isin(STOPS) & (ev.sim_time > t[air[0]])]
    t_end = float(st.sim_time.min()) if len(st) else float(t[air[-1]])
    return (t >= (t[air[0]] if start is None else start)) & (t <= t_end)


def shape(d):
    """f0, see-saw amplitude, and the rows of the mode shape and the loop (run.csv over the window)."""
    t = d.t.to_numpy()
    fs = 1 / np.median(np.diff(t))
    ex, ey = (d.d0_x - d.d1_x).to_numpy(), (d.d0_y - d.d1_y).to_numpy()
    n = np.hypot(ex, ey)
    ex, ey = np.median(ex / n), np.median(ey / n)
    w, x, y, z = d[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy().T
    see = np.degrees(np.arcsin(np.clip(-((2 * (x * z - w * y)) * ex + (2 * (y * z + w * x)) * ey), -1, 1)))
    see -= see.mean()
    f, P = welch(see, fs=fs, nperseg=min(len(see), int(4 * fs)))
    band = (f >= 1.0) & (f <= 2.0)
    f0 = float(f[band][np.argmax(P[band])])
    a_see = xspec(see, see, fs, f0)[2]

    def along(px, py):
        return 100 * (px * ex + py * ey).to_numpy()
    dz = 100 * (d.d0_z - d.d1_z).to_numpy() / 2
    u0, u1 = along(d.d0_x, d.d0_y), along(d.d1_x, d.d1_y)
    sig = {'dz': dz, 'mean z': 100 * (d.d0_z + d.d1_z).to_numpy() / 2, 'ring u': along(d.payload_x, d.payload_y),
           'quads u': (u0 + u1) / 2, 'anti u': (u0 - u1) / 2}
    out = {}
    for k, s in sig.items():
        ph, coh, amp = xspec(see, s - s.mean(), fs, f0)
        out[k] = (amp / a_see, ph, coh)
    dvz = np.gradient(dz / 100, t)
    dthr = (d.d0_thr - d.d1_thr).to_numpy() / 2
    dref = 100 * (d.d0_ref_z - d.d1_ref_z).to_numpy() / 2
    uref = along((d.d0_ref_x + d.d1_ref_x) / 2, (d.d0_ref_y + d.d1_ref_y) / 2)
    um = sig['quads u']
    loop = {'dthr vs -dvz': xspec(-dvz, dthr - dthr.mean(), fs, f0)[:2],
            'ref z trail': xspec(dz - dz.mean(), dref - dref.mean(), fs, f0),
            'ref u trail': xspec(um - um.mean(), uref - uref.mean(), fs, f0)}
    ratio = {'ref z trail': loop['ref z trail'][2] / max(xspec(dz - dz.mean(), dz - dz.mean(), fs, f0)[2], 1e-9),
             'ref u trail': loop['ref u trail'][2] / max(xspec(um - um.mean(), um - um.mean(), fs, f0)[2], 1e-9)}
    return f0, a_see, out, loop, ratio, (ex, ey)


def power(trackers, tau, f0, lo_hi):
    """Per quad (c_z, c_u) at f0 from its tracker log, the thrust taken tau late; lo_hi on the tracker clock."""
    Ts = [pd.read_csv(f).drop_duplicates('sim_time').reset_index(drop=True) for f in trackers]
    lo, hi = lo_hi
    tg = np.arange(lo, hi, 0.01)
    S = []
    for T in Ts:
        t = T.sim_time.to_numpy()
        c = {k: np.interp(tg, t, T[k].to_numpy())
             for k in ('pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz')}
        thr = loop_timing.zoh(t, (T.thr_out - T.thr_off).to_numpy(), tg - tau)
        S.append((c, thr))
    e = np.array([np.median(S[0][0]['pose_x'] - S[1][0]['pose_x']), np.median(S[0][0]['pose_y'] - S[1][0]['pose_y'])])
    e /= np.linalg.norm(e)
    out = []
    for c, thr in S:
        w, x, y, z = (c[k] for k in ('pose_qw', 'pose_qx', 'pose_qy', 'pose_qz'))
        F = M_Q * KT * np.clip(thr, 0, None)
        Fz = F * (1 - 2 * (x * x + y * y))
        Fu = F * (2 * (x * z + w * y) * e[0] + 2 * (y * z - w * x) * e[1])
        u = c['pose_x'] * e[0] + c['pose_y'] * e[1]
        out.append((damping(Fz, np.gradient(c['pose_z'], 0.01), 100.0, f0)[0],
                    damping(Fu, np.gradient(u, 0.01), 100.0, f0)[0]))
    return out


def tracker_window(trackers, stop, start):
    T = pd.read_csv(trackers[0]).drop_duplicates('sim_time').reset_index(drop=True)
    w = loop_timing.airborne(T, stop, 1.0)
    if w is None:
        return None
    return (w[0] if start is None else start), w[1] - 0.5


def links(base, cache=None):
    """Rig: per carrier (pivot-to-tip, tip-to-plate) as (mean mm, sd mm, 1.1-1.9 Hz band rms mm), frozen tips out."""
    if base is None or not os.path.isdir(base):
        return None
    B = loop_timing.bag_arrays(base, cache)
    P = loop_timing.stamped(B['payload__motion_capture_state'])
    air = np.where(P[:, 4] > np.median(P[:200, 4]) + 0.05)[0]
    stop = loop_timing.resolve('/'.join(base.split('/')[-2:]))[3]
    tg = np.arange(P[air[0], 1] + 1.0, min(P[air[-1], 1], stop or np.inf), 0.01)
    if len(tg) < 400:
        return None

    def ip(A, cols):
        return np.stack([np.interp(tg, A[:, 1], A[:, c]) for c in cols], -1)
    pr, qr = ip(P, (2, 3, 4)), ip(P, (5, 6, 7, 8))
    Rr = rot(qr / np.linalg.norm(qr, axis=1, keepdims=True))
    sos = butter(3, [1.1, 1.9], btype='band', fs=100.0, output='sos')
    out = []
    for i in range(2):
        tip_k = f'drone_{i}__magnet_tip_state'
        if tip_k not in B or len(B[tip_k]) < 100:
            out.append(None)
            continue
        Tk = loop_timing.stamped(B[tip_k])
        Tk = Tk[~np.r_[False, np.all(np.abs(np.diff(Tk[:, 2:5], axis=0)) < 1e-9, axis=1)]]
        D = loop_timing.stamped(B[f'drone_{i}__motion_capture_state'])
        qd = ip(D, (5, 6, 7, 8))
        piv = ip(D, (2, 3, 4)) + np.einsum('nij,j->ni', rot(qd / np.linalg.norm(qd, axis=1, keepdims=True)),
                                           np.array([0.0, 0.0, -0.04]))
        tip = ip(Tk, (2, 3, 4))
        rel = Rr[0].T @ (tip[0] - pr[0])
        a = np.arctan2(rel[1], rel[0])
        plate = pr + np.einsum('nij,j->ni', Rr, np.array([0.225 * np.cos(a), 0.225 * np.sin(a), 0.0175]))

        def stats(L):
            b = sosfiltfilt(sos, L - L.mean())
            return 1000 * L.mean(), 1000 * L.std(), 1000 * np.sqrt(np.mean(b ** 2))
        out.append((stats(np.linalg.norm(piv - tip, axis=1)), stats(np.linalg.norm(tip - plate, axis=1))))
    return out


def report(arg, tau_thr=None, cache=None):
    label, d, ev, trackers, start, base, env = load(arg)
    m = window(d, ev, start)
    d = d[m].reset_index(drop=True)
    f0, a_see, sh, loop, ratio, _ = shape(d)
    r = {'run': label + (f':{start:g}' if start is not None else ''), 'f0_hz': f0, 'seesaw_deg': a_see,
         'ring_slide_cm_per_deg': sh['ring u'][0], 'dz_cm_per_deg': sh['dz'][0]}
    print(f'== {r["run"]}: {d.t.iloc[-1] - d.t.iloc[0]:.1f} s, f0 {f0:.2f} Hz, see-saw {a_see:.2f} deg at f0')
    print('   shape (per deg of see-saw, phase behind it): ' + ' | '.join(
        f'{k} {v[0]:.3f} cm {v[1]:+.0f} deg (coh {v[2]:.2f})' for k, v in sh.items()))
    ph, coh = loop['dthr vs -dvz']
    print(f'   loop: dthr vs -dvz {ph:+.0f} deg = {1000 * ph / 360 / f0:+.0f} ms (coh {coh:.2f}) | ' + ' | '.join(
        f'{k} {loop[k][0]:+.0f} deg = {1000 * loop[k][0] / 360 / f0:+.0f} ms, ref/act {ratio[k]:.2f} '
        f'(coh {loop[k][1]:.2f})' for k in ('ref z trail', 'ref u trail')))
    if len(trackers) == 2:
        if tau_thr is None:
            tau_thr = 0.045 if base is not None else 0.015 + sum(
                float(env.get(k, 0) or 0) for k in ('SIM_CMD_DELAY_S', 'SIM_CMD_DELAY_THR_S'))
        stop = loop_timing.resolve(arg.partition(':')[0])[3]
        w = tracker_window(trackers, stop, start)
        if w is not None:
            cs = power(trackers, tau_thr, f0, w)
            print(f'   power (thrust {1000 * tau_thr:.0f} ms late): ' + ' | '.join(
                f'd{i} c_z {cz:+.2f} c_u {cu:+.2f} N s/m' for i, (cz, cu) in enumerate(cs)))
            r['c_z'] = ' '.join(f'{cz:+.2f}' for cz, _ in cs)
            r['c_u'] = ' '.join(f'{cu:+.2f}' for _, cu in cs)
    lk = links(base, cache)
    if lk:
        print('   links (mean / sd / band rms, mm): ' + ' | '.join(
            f'd{i} pivot-tip {p[0]:.1f} / {p[1]:.1f} / {p[2]:.1f}, tip-plate {q[0]:.1f} / {q[1]:.1f} / {q[2]:.1f}'
            for i, (p, q) in enumerate(lk) if p is not None or q is not None))
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--tau-thr', type=float, default=None, help='s from throttle to thrust (power)')
    ap.add_argument('--cache', default=None, help='folder for the bag npz caches (tools/loop_timing.py)')
    ap.add_argument('--csv')
    a = ap.parse_args(argv)
    rows = [report(arg, a.tau_thr, a.cache) for arg in a.runs]
    if a.csv:
        cols = ['run', 'f0_hz', 'seesaw_deg', 'ring_slide_cm_per_deg', 'dz_cm_per_deg', 'c_z', 'c_u']
        with open(a.csv, 'w', newline='') as fh:
            wr = csv.DictWriter(fh, fieldnames=cols, extrasaction='ignore', lineterminator='\n')
            wr.writeheader()
            wr.writerows(rows)
    return rows


if __name__ == '__main__':
    main()
