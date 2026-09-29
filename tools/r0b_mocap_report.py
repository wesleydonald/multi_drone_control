#!/usr/bin/env python3
"""tools/r0b_mocap_report.py -- the P9 numbers from the R0b rest bag (ladder P9, tests.txt R0b step 1).

    python3 tools/r0b_mocap_report.py results/rig/2026-09-30/r0b_rest [--window 5]

Every body sits still, so everything the publisher reports as motion is noise. Per body:
  * rate and dt jitter of the header stamps (the publisher stamps ARRIVAL time, so this is the
    jitter its velocity finite difference divides by);
  * the published twist at rest: std / p99 / max of |v| and |w|, and glitch counts (|w| > 0.5 rad/s,
    |v| > 0.10 m/s) per 100 s: the rig twin of the sim carrier kicks (M-kicks);
  * the P9 candidate for comparison: a least-squares slope of position over the last --window
    samples against the same stamps.
The decision stays Wesley's; this prints the numbers side by side."""
import argparse
import sys

import numpy as np

W_GLITCH = 0.5      # rad/s at rest
V_GLITCH = 0.10     # m/s at rest


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        if not topic.endswith('motion_capture_state'):
            continue
        m = deserialize_message(data, get_message(types[topic]))
        p, tw = m.pose.position, m.twist
        out.setdefault(topic, []).append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z,
             tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.x, tw.angular.y, tw.angular.z))
    return {k: np.array(v, float) for k, v in out.items()}


def lsq_velocity(t, pos, window):
    """Slope of each position axis over the trailing `window` samples (the P9 candidate)."""
    v = np.full_like(pos, np.nan)
    for k in range(window - 1, len(t)):
        tt = t[k - window + 1:k + 1] - t[k]
        A = np.column_stack([tt, np.ones_like(tt)])
        v[k] = np.linalg.lstsq(A, pos[k - window + 1:k + 1], rcond=None)[0][0]
    return v


def summarise(a, window):
    t = a[:, 0]
    dt = np.diff(t)
    span = t[-1] - t[0] if len(t) > 1 else float('nan')
    v = np.linalg.norm(a[:, 4:7], axis=1)
    w = np.linalg.norm(a[:, 7:10], axis=1)
    vl = np.linalg.norm(lsq_velocity(t, a[:, 1:4], window), axis=1)
    vl = vl[np.isfinite(vl)]
    per100 = 100.0 / span if span > 0 else float('nan')
    return {
        'n': len(t), 'rate_hz': (len(t) - 1) / span if span > 0 else float('nan'),
        'dt_ms_mean': 1e3 * dt.mean(), 'dt_ms_std': 1e3 * dt.std(), 'dt_ms_max': 1e3 * dt.max(),
        'dt_ms_min': 1e3 * dt.min(), 'pos_mm_std': 1e3 * np.linalg.norm(a[:, 1:4].std(axis=0)),
        'v_std': v.std(), 'v_p99': np.percentile(v, 99), 'v_max': v.max(),
        'w_std': w.std(), 'w_p99': np.percentile(w, 99), 'w_max': w.max(),
        'w_glitch_per100s': (w > W_GLITCH).sum() * per100,
        'v_glitch_per100s': (v > V_GLITCH).sum() * per100,
        'v_lsq_p99': np.percentile(vl, 99) if len(vl) else float('nan'),
        'v_lsq_max': vl.max() if len(vl) else float('nan'),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag')
    ap.add_argument('--window', type=int, default=5, help='samples in the LSQ slope (P9 candidate)')
    args = ap.parse_args(argv)
    bodies = read_bag(args.bag)
    if not bodies:
        print('no */motion_capture_state topics in the bag', file=sys.stderr)
        return 1
    cols = ['n', 'rate_hz', 'dt_ms_mean', 'dt_ms_std', 'dt_ms_min', 'dt_ms_max', 'pos_mm_std',
            'v_p99', 'v_max', 'v_lsq_p99', 'v_lsq_max', 'w_p99', 'w_max',
            'w_glitch_per100s', 'v_glitch_per100s']
    print('topic'.ljust(34) + ''.join(c.rjust(max(12, len(c) + 1)) for c in cols))
    for topic in sorted(bodies):
        s = summarise(bodies[topic], args.window)
        print(topic.ljust(34) + ''.join(f'{s[c]:{max(12, len(c) + 1)}.3f}' if c != 'n' else f'{s[c]:12d}' for c in cols))
    print(f'\nat rest every published v, w is noise; glitch = |w| > {W_GLITCH} rad/s or |v| > {V_GLITCH} m/s. '
          f'v_lsq = least-squares slope over {args.window} samples on the same stamps (P9 candidate).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
