#!/usr/bin/env python3
"""
tools/marker_swap_scan.py -- held magnet-marker swaps in our rig bags, against the check's body-to-marker band
(critic round 3, N1, card 2026-10-21_tip_detect: a swap held 0.1 s inside the band could fire the check).

    tools/marker_swap_scan.py results/rig/*/*/

For every airborne stream of /magnet_tip_pose (QUAD4's or QUAD1's marker; its body is /drone_3, or /drone_0 when it
flew alone) and /drone_<i>/magnet_tip_state: the stream is cut at every frame step over 3 cm (a magnet swings at most
~1 cm a sample). Each piece between two such steps is listed with its hold time, the step into it, the median body-to-marker
length against the stream's own (its median over the airborne samples, the hanging length), and how far its samples
sit off the line bridging the sample before it and the one after it. Real motion stays on that line, even with the
stream's repeat-then-catch-up jitter; a swap leaves it and comes back. An excursion over 3 cm, inside +-3 cm of the
hanging length and held 0.1 s or more, is what the check could take for a release.
"""
import argparse
import os
import re

import numpy as np

STEP_M = 0.03
BAND_M = 0.03
HOLD_S = 0.1
TRACK_S = 2.0


def read(bag):
    """The bag's magnet-marker streams and only the body poses they need: {topic: [(t, x, y, z)]}."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    tips = [t for t in types if t == '/magnet_tip_pose' or re.fullmatch(r'/drone_\d+/magnet_tip_state', t)]
    want = set(tips)
    for t in tips:
        m = re.match(r'/drone_(\d+)/', t)
        body = (f'/drone_{m.group(1)}/motion_capture_state' if m else
                '/drone_3/motion_capture_state' if '/drone_3/motion_capture_state' in types
                else '/drone_0/motion_capture_state')
        if body in types:
            want.add(body)
    if not tips:
        return {}
    r.set_filter(rosbag2_py.StorageFilter(topics=sorted(want)))
    cls = {t: get_message(types[t]) for t in want}
    out = {t: [] for t in want}
    while r.has_next():
        topic, data, ts = r.read_next()
        p = deserialize_message(data, cls[topic]).pose.position
        out[topic].append((ts * 1e-9, p.x, p.y, p.z))
    return {k: np.array(sorted(v)) for k, v in out.items() if len(v) > 50}


def pieces(t, p, body):
    """[(i0, i1, step into it)] between frame steps over STEP_M."""
    step = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1)]
    cuts = [0] + list(np.flatnonzero(step > STEP_M)) + [len(t)]
    return [(a, b, float(step[a])) for a, b in zip(cuts[:-1], cuts[1:]) if b > a]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    a = ap.parse_args()
    total, risky, n_streams, air_s = 0, [], 0, 0.0
    for bag in a.bags:
        bag = bag.rstrip('/')
        if not os.path.exists(os.path.join(bag, 'metadata.yaml')):
            continue
        try:
            D = read(bag)
        except Exception as ex:  # noqa: BLE001
            print(f'{bag}: unreadable ({ex})')
            continue
        streams = {k: v for k, v in D.items() if 'magnet_tip' in k}
        for topic, M in streams.items():
            m = re.match(r'/drone_(\d+)/', topic)
            body_topic = (f'/drone_{m.group(1)}/motion_capture_state' if m else
                          '/drone_3/motion_capture_state' if '/drone_3/motion_capture_state' in D
                          else '/drone_0/motion_capture_state')
            if body_topic not in D:
                continue
            B = D[body_topic]
            t, p = M[:, 0], M[:, 1:4]
            body = np.stack([np.interp(t, B[:, 0], B[:, k]) for k in (1, 2, 3)], 1)
            air = p[:, 2] > 0.3
            if air.sum() < 50:
                continue
            n_streams += 1
            dt = float(np.median(np.diff(t)))
            air_s += air.sum() * dt
            b = np.linalg.norm(p - body, axis=1)
            b_ref = float(np.median(b[air]))
            rows = []
            for i0, i1, s in pieces(t, p, body):
                if i0 == 0 or i1 >= len(t) or not air[i0:i1].any():
                    continue
                hold = (i1 - i0) * dt
                bm = float(np.median(b[i0:i1]))
                inband = abs(bm - b_ref) <= BAND_M
                # an excursion leaves the track and comes back: its samples sit off the line that bridges the
                # sample before it and the sample after it (real motion, even with the stream's repeat-and-catch-up
                # jitter, stays on that line)
                a_, z_ = p[i0 - 1], p[i1]
                f = (t[i0:i1] - t[i0 - 1]) / max(t[i1] - t[i0 - 1], 1e-9)
                off = float(np.median(np.linalg.norm(p[i0:i1] - (a_ + f[:, None] * (z_ - a_)), axis=1)))
                # not a swap: a piece held TRACK_S or more is the marker's own track resuming (a swap lands off the
                # band and is seen as its own piece), and a step right after repeated samples is the stream catching
                # up after a stall (Motive or the bridge repeating its last value)
                stall = bool(np.array_equal(p[i0 - 1], p[i0 - 2])) if i0 >= 2 else False
                if hold >= TRACK_S or stall:
                    off = 0.0
                total += 1
                rows.append((t[i0] - t[0], i1 - i0, hold, s, bm, inband, off))
                if inband and hold >= HOLD_S and off > STEP_M:
                    risky.append((os.path.relpath(bag), topic, t[i0] - t[0], i1 - i0, hold, s, bm, b_ref, off))
            n_in = sum(1 for r in rows if r[5] and r[2] >= HOLD_S and r[6] > STEP_M)
            print(f'{os.path.relpath(bag)} {topic} (body {body_topic}): airborne {air.sum() * dt:.0f} s, hanging length '
                  f'{b_ref:.3f} m; {len(rows)} steps over {100 * STEP_M:.0f} cm; in-band excursions held >= '
                  f'{1000 * HOLD_S:.0f} ms: {n_in}', flush=True)
            for r in rows[:12]:
                print(f'    +{r[0]:7.2f} s: {r[1]:5d} samples ({1000 * r[2]:6.0f} ms) after a {100 * r[3]:6.1f} cm step, '
                      f'body-to-marker {r[4]:.3f} m{" IN BAND" if r[5] else ""}, {100 * r[6]:.1f} cm off the bridging track', flush=True)
    print(f'\n{n_streams} airborne marker streams, {air_s / 60:.1f} min; {total} pieces after steps over '
          f'{100 * STEP_M:.0f} cm; excursions (off the bridging track by > {100 * STEP_M:.0f} cm) in band and held >= '
          f'{1000 * HOLD_S:.0f} ms: {len(risky)}')
    for r in risky:
        print(f'  {r[0]} {r[1]} +{r[2]:.2f} s: {r[3]} samples ({1000 * r[4]:.0f} ms) after a {100 * r[5]:.1f} cm '
              f'step, body-to-marker {r[6]:.3f} m against {r[7]:.3f}, {100 * r[8]:.1f} cm off the track')


if __name__ == '__main__':
    main()
