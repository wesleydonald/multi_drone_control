#!/usr/bin/env python3
"""tools/pair_swing.py -- the ring's 1.1-1.9 Hz swing per run (card docs/experiments/2026-10-22_velocity_filter.md, M1).

    python3 tools/pair_swing.py h2 h2slope f2slope           # rig runs (today's folder first, else the latest day)
    python3 tools/pair_swing.py R2323 R2324                  # twin runs
    python3 tools/pair_swing.py --csv out.csv h2 R2323       # also write the rows

The 22 Oct rig pairs swung at 1.3-1.5 Hz along the pair's line. M1 is the reviewer's measure (22 Oct): the ring
normal's horizontal tip (both components), band-passed 1.1-1.9 Hz (3rd-order Butterworth, zero phase), rms
magnitude over (a) 0-6 s after lift-off and (b) from the top to the first of LAND / DROP / DETECTED / FLEET_ABORT / the start of
the planner's own descent after a path (its reference falling 2 cm below its top) / a frozen ring stream (50 ms of
identical poses with the ring off the floor: the twin's ring lies bit-still on the floor and repeats a sample or two in
flight; reviewer 22 Oct). Also printed:
  see-saw / roll   the tip along / across the pair's line (drone 0 to drone 1), p-p per 2 s, and their peak frequency;
                   only for two-quad runs
  vel lag          ms the drones' logged velocity trails the derivative of their position (run.csv, 50 Hz grid: a
                   20 ms step; the bags give it finer, see tools/tip_replay.py for the bag reader). Over the
                   window after the top: one from lift-off takes in the lift ramp and the descent and reads one
                   sample for both filters (reviewer 22 Oct)
Read-only.
"""
import argparse
import csv
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt, welch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rig_summary  # noqa: E402

STOPS = ('LAND', 'DROP', 'DETECTED', 'FLEET_ABORT')


FROZEN_S = 0.05     # s of identical ring poses, off the floor, that count as a frozen stream


def windows(d, ev):
    """(lift-off index, top index, end index, what ended it)."""
    t, z = d.t.to_numpy(), d.payload_z.to_numpy()
    z_rest = float(np.median(z[:50]))
    air = np.where(z > z_rest + 0.05)[0]
    a0 = int(air[0])
    stops = ev[(ev.event.isin(STOPS)) & (ev.sim_time > t[a0])]
    t_stop = float(stops.sim_time.min()) if len(stops) else float(t[air[-1]])
    why = str(stops.loc[stops.sim_time.idxmin(), 'event']) if len(stops) else 'end'
    e = int(np.searchsorted(t, t_stop))
    if 'payload_ref_z' in d:
        zr = d.payload_ref_z.to_numpy()
        i_top_ref = a0 + int(np.nanargmax(zr[a0:e])) if e > a0 else a0
        down = np.where(zr[i_top_ref:e] < np.nanmax(zr[a0:e]) - 0.02)[0] if e > a0 else []
        if len(down):
            e, why = i_top_ref + int(down[0]), 'the planner\'s descent'
    same = (np.abs(np.diff(d.payload_qx)) < 1e-12) & (np.abs(np.diff(d.payload_qy)) < 1e-12)
    n_frz = max(2, int(round(FROZEN_S / float(np.median(np.diff(t))))))
    run = np.convolve(same.astype(int), np.ones(n_frz, int), mode='valid') >= n_frz
    frz = np.where(run & (z[:len(run)] > z_rest + 0.05))[0]
    frz = frz[frz > a0 + 50]
    if frz.size and frz[0] < e:
        e, why = int(frz[0]), why + ' (ring frozen)'
    top = z[a0:e].max()
    top_i = a0 + int(np.argmax(z[a0:e] > min(top, 1.0) - 0.03))
    return a0, top_i, e, why


def band(s, fs, lo=1.1, hi=1.9):
    return sosfiltfilt(butter(3, [lo, hi], btype='band', fs=fs, output='sos'), s - s.mean())


def peak_hz(s, fs, lo=0.3, hi=3.0):
    f, P = welch(s - s.mean(), fs=fs, nperseg=min(len(s), int(8 * fs)))
    m = (f >= lo) & (f <= hi)
    return float(f[m][np.argmax(P[m])])


def vel_lag_ms(d, a0, e, n):
    t = d.t.to_numpy()[a0:e]
    out = []
    for i in range(n):
        lags = []
        for ax in ('x', 'z'):
            v = d[f'd{i}_v{ax}'].to_numpy()[a0:e]
            g = np.gradient(d[f'd{i}_{ax}'].to_numpy()[a0:e], t)
            cs = [np.corrcoef(v[k:] - v[k:].mean(), g[:len(g) - k] - g[:len(g) - k].mean())[0, 1] for k in range(8)]
            lags.append(int(np.argmax(cs)))
        out.append(round(1000 * float(np.median(np.diff(t))) * max(lags)))
    return out


def score(arg):
    run, _, _, label, _ = rig_summary.resolve(arg)
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    t = d.t.to_numpy()
    fs = 1.0 / float(np.median(np.diff(t)))
    n = len([c for c in d.columns if c.endswith('_ref_x') and c.startswith('d')])
    a0, top, e, why = windows(d, ev)
    w, x, y, z = d[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy().T
    nx, ny = 2 * (x * z - w * y), 2 * (y * z + w * x)
    seg = slice(a0, e)
    mag = np.degrees(np.hypot(band(nx[seg], fs), band(ny[seg], fs)))

    def rms(i0, i1):
        i0, i1 = max(i0 - a0, 0), min(i1 - a0, e - a0)
        return float(np.sqrt(np.mean(mag[i0:i1] ** 2))) if i1 - i0 > fs else float('nan')
    r = {'run': label, 'n': n, 'end': why, 'climb_s': t[top] - t[a0], 'hover_s': t[e - 1] - t[top],
         'm1_0_6s': rms(a0, a0 + int(6 * fs)), 'm1_hover': rms(top, e),
         'per2s': [rms(i, i + int(2 * fs)) for i in range(a0, e - int(2 * fs) + 1, int(2 * fs))],
         'vel_lag_ms': vel_lag_ms(d, a0, e, n)}
    if n == 2:
        az = np.unwrap(np.arctan2(d.d0_y - d.d1_y, d.d0_x - d.d1_x).to_numpy())
        k = 25
        az = np.convolve(np.pad(az, k, mode='edge'), np.ones(2 * k + 1) / (2 * k + 1), mode='valid')
        see = np.degrees(np.arcsin(np.clip(-(nx * np.cos(az) + ny * np.sin(az)), -1, 1)))
        rol = np.degrees(np.arcsin(np.clip(nx * np.sin(az) - ny * np.cos(az), -1, 1)))
        w2 = int(2 * fs)
        r['seesaw_pp'] = [float(np.ptp(see[i:i + w2])) for i in range(a0, e - w2 + 1, w2)]
        r['roll_pp'] = [float(np.ptp(rol[i:i + w2])) for i in range(a0, e - w2 + 1, w2)]
        r['seesaw_hz'], r['roll_hz'] = peak_hz(see[seg], fs), peak_hz(rol[seg], fs)
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--csv')
    a = ap.parse_args(argv)
    rows = []
    for arg in a.runs:
        r = score(arg)
        rows.append(r)
        print(f'== {r["run"]} ({r["n"]} quads): climb {r["climb_s"]:.1f} s, hover {r["hover_s"]:.1f} s, ended by {r["end"]}')
        print(f'   M1 1.1-1.9 Hz rms: 0-6 s {r["m1_0_6s"]:.2f} deg | hover {r["m1_hover"]:.2f} deg | per 2 s '
              + ' '.join(f'{v:.2f}' for v in r['per2s']))
        if 'seesaw_pp' in r:
            print(f'   see-saw p-p per 2 s ' + ' '.join(f'{v:.1f}' for v in r['seesaw_pp']) + f' [{r["seesaw_hz"]:.2f} Hz]'
                  + ' | roll ' + ' '.join(f'{v:.1f}' for v in r['roll_pp']) + f' [{r["roll_hz"]:.2f} Hz]')
        print(f'   drones\' velocity lag (run.csv): {r["vel_lag_ms"]} ms')
    if a.csv:
        with open(a.csv, 'w', newline='') as fh:
            wr = csv.writer(fh, lineterminator='\n')
            wr.writerow(['run', 'n', 'end', 'climb_s', 'hover_s', 'm1_0_6s', 'm1_hover', 'vel_lag_ms'])
            for r in rows:
                wr.writerow([r['run'], r['n'], r['end'], f'{r["climb_s"]:.1f}', f'{r["hover_s"]:.1f}',
                             f'{r["m1_0_6s"]:.3f}', f'{r["m1_hover"]:.3f}', ' '.join(map(str, r['vel_lag_ms']))])


if __name__ == '__main__':
    main()
