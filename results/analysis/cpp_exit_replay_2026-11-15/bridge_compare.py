"""bridge_compare.py: ring_bridge's publication gaps and commitment validity, before (runs_disable/) against after
(runs_bridge_after/) the ring_bridge fix, from each run's independent recording (feeds/, system-clock receipt mapped to
ROS time by the recorded /clock), over the replay from its CLEAR_DROP_ZONE entry - 1 s to the recording's end.

Gaps: per topic max, p99, count > 0.15 s and > 0.25 s; the share of gaps > 0.15 s that end within 0.05 s of a
commitment refresh (the basket's committed trajectory receipt). Commitments: per refresh sequence, the three carriers'
and the basket's messages present with one sequence; sequences increasing by 1; each commitment's coverage after its
receipt (last valid_until - receipt) and start (first valid_from - receipt). Backend: the 1 Hz diagnostics' shared
fresh count and min future coverage, and any DISABLE_DIAG line. -> bridge_compare.csv and printed."""
import csv
import glob
import os
import re
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
STATES = ['/fake_payload/pose_array'] + [f'/fake_obstacles/drone_{i}/state' for i in range(3)]
COMMITS = [f'/fake_obstacles/drone_{i}/committed_trajectory' for i in range(3)] + ['/fake_payload/committed_trajectory']


def run_rows(d):
    db = glob.glob(os.path.join(d, 'feeds', '*.db3'))[0]
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {n: (i, t) for i, n, t in con.execute('select id, name, type from topics')}

    def msgs(name, decode=False):
        tid, ttype = topics[name]
        rows = con.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid,)).fetchall()
        w = np.array([r[0] * 1e-9 for r in rows])
        if not decode:
            return w, None
        cls = get_message(ttype)
        return w, [deserialize_message(r[1], cls) for r in rows]
    cw, cm = msgs('/clock', True)
    cr = np.array([m.clock.sec + 1e-9 * m.clock.nanosec for m in cm])
    to_ros = lambda w: np.interp(w, cw, cr)
    t0 = float(open(os.path.join(d, 'events.txt')).readline().split()[0])
    out = {'run': os.path.relpath(d, HERE)}
    cw_basket, _ = msgs('/fake_payload/committed_trajectory')
    refresh = to_ros(cw_basket)
    for name in STATES:
        w, _ = msgs(name)
        t = to_ros(w)
        t = t[t >= t0 - 1.0]
        g = np.diff(t)
        ends = t[1:]
        big = g > 0.15
        near = np.array([np.min(np.abs(refresh - e)) < 0.05 for e in ends[big]]) if big.any() else np.array([])
        key = 'pose' if 'pose_array' in name else name.split('/')[2]
        out[f'{key}_n'] = int(t.size)
        out[f'{key}_gap_max_s'] = round(float(g.max()), 3)
        out[f'{key}_gap_p99_s'] = round(float(np.percentile(g, 99)), 3)
        out[f'{key}_gaps_over_0.15'] = int(big.sum())
        out[f'{key}_gaps_over_0.25'] = int((g > 0.25).sum())
        out[f'{key}_big_gaps_at_refresh'] = f'{int(near.sum())}/{int(big.sum())}'
        k5 = np.argsort(g)[-5:]
        out[f'{key}_top5_at_refresh'] = sum(1 for k in k5 if np.min(np.abs(refresh - ends[k])) < 0.05)
    # commitments
    seqs = {}
    cover, start = [], []
    for name in COMMITS:
        w, ms = msgs(name, True)
        tr = to_ros(w)
        for ti, m in zip(tr, ms):
            if ti < t0 - 1.0:
                continue
            seqs.setdefault(int(m.sequence), set()).add(name)
            cover.append(m.pieces[-1].valid_until_s - ti)
            start.append(m.pieces[0].valid_from_s - ti)
    sq = sorted(seqs)
    out['commit_refreshes'] = len(sq)
    out['commit_incomplete'] = sum(1 for s in sq if len(seqs[s]) != 4)
    out['commit_seq_steps_not_1'] = int(np.sum(np.diff(sq) != 1)) if len(sq) > 1 else 0
    out['commit_coverage_min_s'] = round(float(min(cover)), 3)
    out['commit_start_after_receipt_max_s'] = round(float(max(start)), 3)
    # backend
    log = open(os.path.join(d, 'backend.log'), errors='replace').read()
    fr = [(int(a), float(b)) for a, b in re.findall(r'shared fresh: shared_trajectory / (\d)/3 min_future_s=([-\d.]+)', log)]
    after = fr[3:] if len(fr) > 3 else fr                  # the first lines precede the bag
    out['backend_shared_fresh_min'] = min(a for a, _ in after) if after else ''
    out['backend_min_future_s'] = min(b for _, b in after) if after else ''
    out['disable'] = ';'.join(re.findall(r'Backend disabled at ROS [0-9.]+: ([A-Z_]+)', log))
    out['disable_diag'] = ' || '.join(l.split('DISABLE_DIAG', 1)[1].strip() for l in log.splitlines() if 'DISABLE_DIAG' in l)
    out['outcome'] = 'AT_START' if 'phase RETURN_TO_START -> AT_START' in open(os.path.join(d, 'events.txt')).read() \
        else 'no AT_START'
    return out


def main():
    dirs = sorted(glob.glob(os.path.join(HERE, 'runs_disable', 'attempt[0-9]'))) + [os.path.join(HERE, 'runs_disable', 'idle1')]
    dirs += sorted(glob.glob(os.path.join(HERE, 'runs_bridge_after', 'attempt[0-9]'))) + \
        [os.path.join(HERE, 'runs_bridge_after', 'idle1')]
    rows = [run_rows(d) for d in dirs if os.path.isdir(d)]
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(HERE, 'bridge_compare.csv'), 'w', newline='\n') as fh:
        w = csv.DictWriter(fh, keys, lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)


if __name__ == '__main__':
    main()
