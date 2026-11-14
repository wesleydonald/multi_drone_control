"""Closest approaches of the virtual QUAD4 to the measured carriers during a replay (from authority on).
usage: python3 full/dist.py <run> <flight> <requested bag offset>   (run from rigcheck/)
QUAD4: body point, hanging line body -> body - (1.034 + 0.05 + 0.05) (cable, magnet, object; vertical, no swing).
Carrier i: measured body, tether from body - 0.04 m to its plate (ring + 0.225 m toward the carrier)."""
import bisect
import csv
import math
import sys

import numpy as np

run, flight, req = sys.argv[1], sys.argv[2], float(sys.argv[3])
d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
p = list(csv.DictReader(open(f'{flight}_poses.csv')))
pt = [float(r['t']) for r in p]


def at(t):
    i = min(max(bisect.bisect_left(pt, t), 0), len(pt) - 1)
    return p[i]


dt0 = float(d[0]['t'])
best = None
for off in [req - 4.0 + x * 0.02 for x in range(250)]:
    err = sum(math.hypot(float(r['ring_x']) - float(at(float(r['t']) - dt0 + off)['r_x']),
                         float(r['ring_y']) - float(at(float(r['t']) - dt0 + off)['r_y'])) for r in d[::20])
    if best is None or err < best[0]:
        best = (err, off)
off = best[1]


def seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def pt_seg(x, a, b):
    ab = b - a
    s = min(max(((x - a) @ ab) / (ab @ ab), 0.0), 1.0)
    return float(np.linalg.norm(x - (a + ab * s)))


mins = {'body-body': (9, 0), 'line-tether': (9, 0), 'body-tether': (9, 0), 'line-body': (9, 0)}
zmax = 0.0
strand = []   # (t, stationary, line-to-carrier min distance, captured)
for r in d:
    if r['auth'] != '1':
        continue
    q = at(float(r['t']) - dt0 + off)
    b = np.array([float(r['x']), float(r['y']), float(r['z'])])
    zmax = max(zmax, b[2])
    lo = b - np.array([0, 0, 1.034 + 0.10])
    ring = np.array([float(q['r_x']), float(q['r_y']), float(q['r_z'])])
    t = float(r['t']) - dt0
    dline = 9.0
    for i in range(3):
        c = np.array([float(q[f'd{i}_x']), float(q[f'd{i}_y']), float(q[f'd{i}_z'])])
        h = c[:2] - ring[:2]
        plate = ring + np.array([*(0.225 * h / np.linalg.norm(h)), 0.0])
        top = c - np.array([0, 0, 0.04])
        lt_, lb_ = seg_seg(b, lo, top, plate), pt_seg(c, b, lo)
        dline = min(dline, lt_, lb_)
        for k, v in (('body-body', float(np.linalg.norm(b - c))), ('line-tether', lt_),
                     ('body-tether', pt_seg(b, top, plate)), ('line-body', lb_)):
            if v < mins[k][0]:
                mins[k] = (v, t)
    sp = math.sqrt(float(r['vx']) ** 2 + float(r['vy']) ** 2 + float(r['vz']) ** 2)
    strand.append((t, sp < 0.05, dline, r['captured'] == '1'))


def longest(samples):
    best, cur = (0.0, None, 9.0), None
    for t, st, dl, _ in samples:
        if st and dl < 0.5:
            cur = (cur[0], t, min(cur[2], dl)) if cur else (t, t, dl)
            if cur[1] - cur[0] > best[0]:
                best = (cur[1] - cur[0], cur[0], cur[2])
        else:
            cur = None
    return best


sa = longest(strand)
sb = longest([x for x in strand if not x[3]])
STRAND = (f'stranding (QUAD4 still, < 0.05 m/s, with a carrier body/tether within 0.5 m of its hanging line): longest '
          f'{sa[0]:.1f} s' + (f' from +{sa[1]:.1f} s, min {sa[2]:.2f} m' if sa[1] is not None else '') +
          f'; before capture {sb[0]:.1f} s' + (f' from +{sb[1]:.1f} s, min {sb[2]:.2f} m' if sb[1] is not None else ''))
print(f'{run}: transit body z max {zmax:.2f}; closest (centre lines, m): ' +
      ', '.join(f'{k} {v:.2f} at +{t:.1f}s' for k, (v, t) in mins.items()))
print('   ' + STRAND)
