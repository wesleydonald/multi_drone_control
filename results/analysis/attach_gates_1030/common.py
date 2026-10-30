import json
import os

import numpy as np

S = os.path.dirname(os.path.abspath(__file__))
WELD = {'sq5_2': (-0.005, 0.197, 0.028), 'sq2_5': (-0.005, 0.197, 0.028), 'k1_4': (-0.005, 0.197, 0.028),
        't6h_4': (0.013, 0.210, 0.024), 'a1': (0.013, 0.210, 0.024)}


def quat_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load(tag):
    d = dict(np.load(os.path.join(S, f'{tag}.npz')))
    s = json.load(open(os.path.join(S, f'{tag}_strings.json')))
    return d, s


def hold_idx(t_src, tq):
    return np.clip(np.searchsorted(t_src, tq, side='right') - 1, 0, len(t_src) - 1)


def plate_frame(d, weld, tq=None):
    """(t, rel (n,3) tip minus weld point in the ring frame, plate point world (n,3), R list) at the tip samples."""
    tip, ring = d['tip'], d['ring']
    t = tip[:, 0] if tq is None else tq
    it = hold_idx(tip[:, 0], t)
    ir = hold_idx(ring[:, 0], t)
    rels = np.zeros((len(t), 3))
    plates = np.zeros((len(t), 3))
    Rs = np.zeros((len(t), 3, 3))
    for k in range(len(t)):
        R = quat_rot(ring[ir[k], 5:9])
        p = ring[ir[k], 2:5] + R @ np.asarray(weld)
        rels[k] = R.T @ (tip[it[k], 2:5] - p)
        plates[k] = p
        Rs[k] = R
    return t, rels, plates, Rs


def frozen_mask(d):
    """True where a tip sample repeats the previous one bit for bit (Motive's hold)."""
    tip = d['tip']
    same = np.all(tip[1:, 2:5] == tip[:-1, 2:5], axis=1)
    return np.concatenate([[False], same])


def phases(s):
    return [(t, v) for t, v in s.get('/join_planner/phase', [])]


def phase_spans(s):
    """[(t0, t1, phase)] from the latched phase stream (changes only)."""
    out = []
    last = None
    for t, v in phases(s):
        if v != last:
            if out:
                out[-1][1] = t
            out.append([t, None, v])
            last = v
    if out:
        out[-1][1] = out[-1][0] + 1e6
    return out
