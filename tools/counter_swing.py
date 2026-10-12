#!/usr/bin/env python3
"""tools/counter_swing.py -- the rig-only thrust swing after a release (thesis plan P6, finding 6): what drives it.
    python3 tools/counter_swing.py [--runs 2026-10-08/r303_4 ... R1140 ...] [--carry 2026-10-07/r206e60] [--out DIR]
On the measured pull (tracker cable_source mocap) a survivor's thrust is its hover thrust plus the measured rod pull
at node 0 plus the position loop. For every survivor of every run, from the logs on disk (no bag): the swing of the
measured vertical pull fz = -m acz, of the commanded thrust m kT u2 and of their difference (the position loop's part),
in a window before the event and in the 5 s after the resized plan arrives; the ring's tilt-vector swing in the same
windows; the lag and correlation of the thrust against the measured pull; the thrust step across the plan's arrival
split into the measured pull's step and the rest; the reference change the resized plan brought; the swing's envelope
in 1 s bins; and a replay of a first-order low-pass on the measured offset (tau 0.3 and 0.5 s) to show what it would
remove. A carry flight with no event gives the same numbers for the steady orbit (rig: the mode exists without any
event). Times on the rig_to_run run clock (manifest t0_epoch); a twin run on the runner's clock (planner log aligned
on the ring's lift-off), as tools/resize_doublecount.py does."""
import argparse
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import release_bands as rb  # noqa: E402

RESULTS = os.environ.get('MDC_RESULTS_ROOT', '/home/wesley/multi_drone_control/results')
RUNS = ['2026-10-08/r303_4', '2026-10-08/r304_3', '2026-10-08/r305', '2026-10-08/r302_3',
        'R1140', 'R1145', 'R1139', 'R1141', 'R1118', 'R1121']
CARRY = ['2026-10-07/r206e60']
M, G, DT = 0.55, 9.81, 0.02


def spec(t, y, f_lo=0.3, f_hi=3.0):
    """Dominant frequency (Hz) in [f_lo, f_hi] and its amplitude, Hann window on a 50 Hz resample."""
    if len(t) < 20:
        return np.nan, np.nan
    tu = np.arange(t[0], t[-1], DT)
    yu = np.interp(tu, t, y)
    yu -= yu.mean()
    n = len(yu)
    win = np.hanning(n)
    Y = np.fft.rfft(yu * win)
    f = np.fft.rfftfreq(n, DT)
    k = (f >= f_lo) & (f <= f_hi)
    i = int(np.argmax(np.abs(Y[k])))
    return float(f[k][i]), float(2 * np.abs(Y[k][i]) / win.sum())


def xlag(ta, a, tb, b, maxlag=1.0):
    """Lag L (s) maximising corr(a(t + L), b(t)) and that correlation: L > 0 means a follows b."""
    tu = np.arange(max(ta[0], tb[0]), min(ta[-1], tb[-1]), DT)
    au, bu = np.interp(tu, ta, a), np.interp(tu, tb, b)
    au, bu = au - au.mean(), bu - bu.mean()
    best = (-2.0, 0.0)
    for L in np.arange(-maxlag, maxlag + 1e-9, DT):
        aa = np.interp(tu + L, tu, au, left=np.nan, right=np.nan)
        ok = np.isfinite(aa)
        if ok.sum() < 20:
            continue
        c = float(np.corrcoef(aa[ok], bu[ok])[0, 1])
        if c > best[0]:
            best = (c, float(L))
    return round(best[1], 2), round(best[0], 2)


def half_range(t, y, a, b, step=1.0):
    out = []
    for s in np.arange(a, b - 1e-9, step):
        w = (t >= s) & (t < s + step)
        if w.sum() > 10:
            out.append(round(float(np.ptp(y[w])) / 2, 2))
    return out


def lpf(t, y, tau):
    out = np.empty_like(y)
    out[0] = y[0]
    for i in range(1, len(y)):
        k = (t[i] - t[i - 1]) / (tau + (t[i] - t[i - 1]))
        out[i] = out[i - 1] + k * (y[i] - out[i - 1])
    return out


def liftoff(t, z, rise=0.03):
    t, z = np.asarray(t, float), np.asarray(z, float)
    return float(t[np.nonzero(z > np.nanmedian(z[:20]) + rise)[0][0]])


def load(tag):
    """(events dict, planner log with rt, run.csv, tracker dir, run-clock origin, source)."""
    if tag.startswith('R') and tag[1:5].isdigit():
        d = glob.glob(os.path.join(RESULTS, '2*', tag + '_*'))[0]
        logs = os.path.join(d, 'logs')
        ev = pd.read_csv(os.path.join(logs, 'events.csv'))
        pl = (glob.glob(os.path.join(logs, 'dissipative_planner', '*', 'log.csv'))
              or glob.glob(os.path.join(logs, 'mpc_planner', '*', 'log.csv')))
        P = pd.read_csv(pl[0])
        run = pd.read_csv(os.path.join(logs, 'run.csv')).drop_duplicates('t')
        t0 = liftoff(P.sim_time, P.load_z) - liftoff(run.t, run.payload_z)
        trackers, src = os.path.join(logs, 'tracker'), 'twin'
    else:
        date, name = tag.split('/')
        base = os.path.join(RESULTS, 'rig', date, name)
        t0 = json.load(open(base + '_run/manifest.json'))['t0_epoch']
        ev = pd.read_csv(base + '_run/logs/events.csv')
        run = pd.read_csv(base + '_run/logs/run.csv').drop_duplicates('t')
        pl = (glob.glob(base + '_logs/logs/dissipative_planner/*/log.csv')
              or glob.glob(base + '_logs/logs/mpc_planner/*/log.csv'))
        P = pd.read_csv(pl[0])
        trackers, src = base + '_logs/logs/tracker', 'rig'
    P['rt'] = P.sim_time - t0
    evd = {}
    for _, r in ev.iterrows():
        evd.setdefault(str(r.event), float(r.sim_time))
    return evd, P, run, trackers, t0, src


def tracker(trackers, d, t0):
    tr = glob.glob(os.path.join(trackers, f'drone{d}_*', 'log.csv'))[0]
    T = pd.read_csv(tr).drop_duplicates('sim_time')
    kt = float(json.load(open(os.path.join(os.path.dirname(tr), 'params.json'))).get('thrust_ratio', 35.2))
    T['rt'] = T.sim_time - t0
    T['thrust'] = M * kt * T.u2
    T['fz'] = -M * T.acz                 # the measured rod pull on the drone, vertical, N (down positive)
    T['resid'] = T.thrust - T.fz         # hover thrust plus the position loop's part
    return T


def ring_tilt(run):
    q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
    ok = np.all(np.isfinite(q), 1)
    return run.t.to_numpy(float)[ok], rb.tilt_vector(q[ok])


def analyse(tag, carry=False):
    evd, P, run, trackers, t0, src = load(tag)
    t_land = evd['LAND']
    tt, th = ring_tilt(run)
    rows = []
    if carry:
        th0 = evd.get('HANDOVER', evd['TAKEOFF'] + 15.0)
        wins = {'steady': (th0 + 8.0, min(th0 + 28.0, t_land - 2.0))}
        s2d = [int(v) for v in str(P.slot2drone.dropna().iloc[-1]).split()] if 'slot2drone' in P else list(range(4))
        t_arr = t_off = None
        res = None
    else:
        kind = 'unannounced' if 'RELEASE' in evd else 'announced'
        t_off = evd['RELEASE'] if kind == 'unannounced' else evd['DETACH']
        n0 = int(P.n.max())
        res = P[(P.n < n0) & (P.rt > t_off - 1.0)].iloc[0]
        t_arr = float(res.rt)
        wins = {'pre': (t_off - 7.0, t_off - 0.5), 'post': (t_arr, min(t_arr + 5.0, t_land))}
        s2d = [int(v) for v in str(res.slot2drone).split()]
        before = P[(P.rt >= t_off - 2.0) & (P.rt < t_off - 0.2) & (P.n == n0)]
        s2d_b = [int(v) for v in before.slot2drone.iloc[-1].split()]
        pre = (tt >= t_off - 5.0) & (tt < t_off)
        dev = np.linalg.norm(th - th[pre].mean(0), axis=1)
    for d in s2d:
        T = tracker(trackers, d, t0)
        t = T.rt.to_numpy(float)
        at = lambda tq, c: float(np.interp(tq, t, T[c]))                               # noqa: E731
        mean = lambda a, z, c: float(T[(T.rt >= a) & (T.rt < z)][c].mean())           # noqa: E731
        for wn, (a, b) in wins.items():
            S = T[(T.rt >= a) & (T.rt < b)]
            wt = (tt >= a) & (tt < b)
            if len(S) < 50 or wt.sum() < 50:
                continue
            f_fz, A_fz = spec(S.rt.values, S.fz.values)
            f_th, A_th = spec(S.rt.values, S.thrust.values)
            _, A_res = spec(S.rt.values, S.resid.values)
            f_tilt, A_tilt = spec(tt[wt], th[wt, 0])
            _, A_tilt_y = spec(tt[wt], th[wt, 1])
            row = dict(run=tag, source=src, kind='carry' if carry else kind, drone=d, window=wn,
                       t_a=round(a, 2), t_b=round(b, 2),
                       fz_mean_N=round(float(S.fz.mean()), 2), fz_f_Hz=round(f_fz, 2), fz_amp_N=round(A_fz, 2),
                       thrust_f_Hz=round(f_th, 2), thrust_amp_N=round(A_th, 2), resid_amp_N=round(A_res, 2),
                       ring_tilt_f_Hz=round(f_tilt, 2), ring_tilt_amp_deg=round(max(A_tilt, A_tilt_y), 2),
                       thrust_vs_fz_lag_s=None, thrust_vs_fz_corr=None)
            row['thrust_vs_fz_lag_s'], row['thrust_vs_fz_corr'] = xlag(S.rt.values, S.thrust.values, S.rt.values, S.fz.values)
            if not carry and wn == 'post':
                kb, ka = s2d_b.index(d), s2d.index(d)
                row.update(dict(
                    plan_vert_step_N=round(-float(res[f'tsz{ka}']) + float(before[f'tsz{kb}'].mean()), 2),
                    thrust_step_at_arrival_N=round(mean(t_arr + 0.08, t_arr + 0.12, 'thrust') - mean(t_arr - 0.06, t_arr, 'thrust'), 2),
                    fz_step_at_arrival_N=round(mean(t_arr + 0.08, t_arr + 0.12, 'fz') - mean(t_arr - 0.06, t_arr, 'fz'), 2),
                    resid_step_at_arrival_N=round(mean(t_arr + 0.08, t_arr + 0.12, 'resid') - mean(t_arr - 0.06, t_arr, 'resid'), 2),
                    dref_z_cm=round(100 * (at(t_arr + 0.5, 'ref_z') - at(t_arr - 0.05, 'ref_z')), 1),
                    dref_xy_cm=round(100 * float(np.hypot(at(t_arr + 0.5, 'ref_x') - at(t_arr - 0.05, 'ref_x'),
                                                          at(t_arr + 0.5, 'ref_y') - at(t_arr - 0.05, 'ref_y'))), 1),
                    fz_half_range_pre_N=half_range(t, T.fz.values, t_off - 6.0, t_off - 1.0, 5.0)[0],
                    fz_half_range_post_N=half_range(t, T.fz.values, t_arr, min(t_land, t_arr + 6.0)),
                    resid_half_range_post_N=half_range(t, T.resid.values, t_arr, min(t_land, t_arr + 6.0)),
                    ring_dev_max_deg_bins=[round(float(dev[(tt >= s) & (tt < s + 1)].max()), 1)
                                           for s in np.arange(t_arr, min(t_land, t_arr + 6.0) - 1e-9, 1.0)],
                    t_off=round(t_off, 2), t_arrival=round(t_arr, 2), hold_s=round(t_land - t_arr, 1)))
                for tau in (0.3, 0.5):
                    fzf = lpf(t, T.fz.to_numpy(float), tau)
                    w = (t >= t_arr) & (t < min(t_land, t_arr + 5.0))
                    row[f'fz_amp_lpf{tau}_N'] = round(spec(t[w], fzf[w])[1], 2)
                    # the filtered pull's share of the unfiltered step 1 s after the release
                    base_v = mean(t_off - 2.0, t_off - 0.2, 'fz')
                    step_raw = float(np.interp(t_off + 1.0, t, T.fz)) - base_v
                    step_f = float(np.interp(t_off + 1.0, t, fzf)) - base_v
                    row[f'step_share_lpf{tau}_at_1s'] = round(step_f / step_raw, 2) if abs(step_raw) > 0.1 else None
            rows.append(row)
    return rows


def md(df):
    """A markdown table without the tabulate dependency."""
    cols = list(df.columns)
    lines = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
    for _, r in df.iterrows():
        lines.append('| ' + ' | '.join('' if (isinstance(v, float) and np.isnan(v)) else str(v) for v in r.values) + ' |')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', nargs='+', default=RUNS)
    ap.add_argument('--carry', nargs='+', default=CARRY)
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'counter_swing_2026-10-12'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for tag in a.carry:
        rows += analyse(tag, carry=True)
    for tag in a.runs:
        rows += analyse(tag)
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(a.out, 'table.csv'), index=False)
    pd.set_option('display.width', 300)
    pd.set_option('display.max_columns', 60)
    cols = ['run', 'source', 'kind', 'drone', 'window', 'fz_f_Hz', 'fz_amp_N', 'thrust_amp_N', 'resid_amp_N',
            'ring_tilt_f_Hz', 'ring_tilt_amp_deg', 'thrust_vs_fz_lag_s', 'thrust_vs_fz_corr']
    with open(os.path.join(a.out, 'table.md'), 'w') as fh:
        fh.write('Spectra and correlations (made by tools/counter_swing.py; windows: pre = 7 to 0.5 s before the event, '
                 'post = the 5 s after the resized plan arrives, steady = 8-28 s into the orbit)\n\n')
        fh.write(md(df[cols]) + '\n\n')
        post = df[df.window == 'post']
        cols2 = ['run', 'drone', 'plan_vert_step_N', 'thrust_step_at_arrival_N', 'fz_step_at_arrival_N',
                 'resid_step_at_arrival_N', 'dref_z_cm', 'dref_xy_cm', 'fz_half_range_pre_N', 'fz_half_range_post_N',
                 'resid_half_range_post_N', 'ring_dev_max_deg_bins', 'fz_amp_lpf0.3_N', 'step_share_lpf0.3_at_1s',
                 'fz_amp_lpf0.5_N', 'step_share_lpf0.5_at_1s']
        fh.write('At the plan\'s arrival and after (post window only)\n\n' + md(post[cols2]) + '\n')
    print(df[cols].to_string(index=False))
    print(open(os.path.join(a.out, 'table.md')).read().split('At the plan')[1][:6000])
    print('->', a.out)


if __name__ == '__main__':
    main()
