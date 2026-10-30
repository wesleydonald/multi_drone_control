#!/usr/bin/env python3
"""W2c: the ring's own motion on the rig (29 Oct), split by what QUAD4 was doing.

Per window, from /payload/motion_capture_state (Motive, ~95 Hz): xy rms about the window mean, z sd, tilt sd
(sqrt(var roll + var pitch) of the ring normal, i.e. the rocking about the window's mean tilt), the median tilt, and
the dominant frequency of the xy motion (Welch on the linearly detrended x and y, summed, 0.2-8 Hz,
10 s segments, one segment for a shorter window) and the xy rms in three bands (drift < 0.3 Hz, swing 0.3-2 Hz, > 2 Hz). QUAD4's median horizontal distance from the ring centre says how near it was.
Windows are seconds after our planner log's first row; his phase times come from his planner's transitions.

    python3 w2c_ring_motion.py
"""
import json
import os
import sys

import numpy as np
from scipy.signal import welch
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import OUT, planner, pose_array, read_bag  # noqa: E402

FS = 100.0
# (run, label, filter flown on the measured pull, QUAD4 state, window)
WINDOWS = [
    ('h4f_3', 'square 0/3/6/9, top of the lift', '2 Hz low-pass x1', 'carrier (on plate 9)', (65.5, 70.5)),
    ('ch1b_2', 'square 0/3/6/9, hover before the release', '2 Hz low-pass x2', 'carrier (on plate 9)', (71.0, 86.0)),
    ('sq2_4', 'triangle, upper lift and top', '2 Hz low-pass x2', 'on its mat (his TAKEOFF held back)', (41.0, 47.5)),
    ('sq2_5', 'triangle, his climb', 'notch 5.3 Hz', 'climbing at its mat', (71.0, 94.5)),
    ('sq2_5', 'triangle, his C++ transit', 'notch 5.3 Hz', 'transit, high', (94.5, 114.3)),
    ('sq2_5', 'triangle, his MATCH and descent', 'notch 5.3 Hz', 'over plate 9', (115.4, 139.1)),
    ('sq5_2', 'triangle, upper lift (to his TAKEOFF)', 'notch 5.3 Hz', 'on its mat', (43.5, 46.6)),
    ('sq5_2', 'triangle, his climb', 'notch 5.3 Hz', 'climbing at its mat', (46.6, 54.7)),
    ('sq5_2', 'triangle, his transfer to plate 9', 'notch 5.3 Hz', 'moving over the ring', (54.7, 61.5)),
    ('sq5_2', 'triangle, his MATCH and 6 descents', 'notch 5.3 Hz', 'over plate 9', (61.5, 86.5)),
    ('sq3', 'triangle, QUAD4 held at its climb end', 'notch 5.3 Hz', 'held high at its climb end', (150.0, 188.0)),
    ('sq4_5', 'triangle, QUAD4 held at the search floor', 'notch 5.3 Hz', 'held at the search floor', (112.0, 145.4)),
    ('k1_4', 'triangle, QUAD4 over the pickup', 'notch 5.3 Hz', 'over the pickup object', (112.2, 138.4)),
]


def stats(ring, q4, a, b):
    m = (ring[:, 0] >= a) & (ring[:, 0] <= b)
    r = ring[m]
    t = np.arange(r[0, 0], r[-1, 0], 1 / FS)
    xyz = np.column_stack([np.interp(t, r[:, 0], r[:, k]) for k in (1, 2, 3)])
    R = Rot.from_quat(r[:, 4:8]).as_matrix()
    nx, ny = np.degrees(np.arcsin(np.clip(R[:, 0, 2], -1, 1))), np.degrees(np.arcsin(np.clip(R[:, 1, 2], -1, 1)))
    tilt = np.degrees(np.arccos(np.clip(R[:, 2, 2], -1, 1)))
    d = xyz[:, :2] - xyz[:, :2].mean(axis=0)
    nper = int(min(len(t), 10 * FS))
    f, px = welch(xyz[:, 0], fs=FS, nperseg=nper, nfft=8192, detrend='linear')
    _, py = welch(xyz[:, 1], fs=FS, nperseg=nper, nfft=8192, detrend='linear')
    pxy, df = px + py, f[1] - f[0]
    band = (f >= 0.2) & (f <= 8.0)
    pk = float(f[band][np.argmax(pxy[band])])
    sw = (f >= 0.3) & (f <= 2.0)
    pk_swing = float(f[sw][np.argmax(pxy[sw])])
    def band_rms(lo, hi):
        sel = (f >= lo) & (f < hi)
        return 100 * float(np.sqrt(np.sum(pxy[sel]) * df))
    mq = (q4[:, 0] >= a) & (q4[:, 0] <= b)
    rq = np.column_stack([np.interp(q4[mq, 0], r[:, 0], r[:, k]) for k in (1, 2)])
    dq = float(np.median(np.linalg.norm(q4[mq, 1:3] - rq, axis=1))) if mq.any() else float('nan')
    return dict(duration_s=float(b - a), xy_rms_cm=100 * float(np.sqrt(np.mean(np.sum(d ** 2, axis=1)))),
                z_sd_cm=100 * float(np.std(xyz[:, 2])), tilt_sd_deg=float(np.sqrt(np.var(nx) + np.var(ny))),
                tilt_median_deg=float(np.median(tilt)), dom_freq_hz=pk, swing_peak_hz=pk_swing, xy_rms_below_0p3hz_cm=band_rms(0.0, 0.3),
                xy_rms_0p3_2hz_cm=band_rms(0.3, 2.0), xy_rms_above_2hz_cm=band_rms(2.0, 50.0), quad4_dist_m=dq,
                ring_z_m=float(np.median(xyz[:, 2])))


def main():
    out = []
    cache = {}
    for run, label, filt, q4s, (a, b) in WINDOWS:
        if run not in cache:
            bg = read_bag(run, ['/payload/motion_capture_state', '/drone_3/motion_capture_state'])
            cache[run] = (pose_array(bg['/payload/motion_capture_state']), pose_array(bg['/drone_3/motion_capture_state']),
                          float(planner(run).sim_time.iloc[0]))
        ring, q4, t0 = cache[run]
        s = stats(ring, q4, t0 + a, t0 + b)
        out.append(dict(run=run, window=label, filter=filt, quad4=q4s, t=(a, b), **s))
        print(f"{run:7s} {label:44s} {filt:17s} {s['duration_s']:5.1f} s  xy {s['xy_rms_cm']:4.1f} cm  z sd {s['z_sd_cm']:4.1f} cm  "
              f"tilt sd {s['tilt_sd_deg']:4.1f} (med {s['tilt_median_deg']:4.1f}) deg  "
              f"f {s['dom_freq_hz']:4.2f} Hz (0.3-2 Hz peak {s['swing_peak_hz']:.2f}) [<0.3 {s['xy_rms_below_0p3hz_cm']:.1f} | 0.3-2 {s['xy_rms_0p3_2hz_cm']:.1f} | >2 {s['xy_rms_above_2hz_cm']:.1f}]  "
              f"QUAD4 {s['quad4_dist_m']:4.2f} m out  ring z {s['ring_z_m']:.2f}")
    with open(os.path.join(OUT, 'w2c_ring_motion.json'), 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
