#!/usr/bin/env python3
"""tools/tip_calibration.py -- Wednesday's props-off tip calibration (card 2026-10-09_approach_tip_feedback).

    python3 tools/tip_calibration.py [--seconds 5] [--standoff 0.0]

Props off, QUAD4's tether hanging, its magnet face held ON plate 9. Reads /magnet_tip_pose (body 24) and
/payload/motion_capture_state (the ring, body 8) for --seconds and prints the tip marker in the ring frame
(mean, sd), then the two launch values for configs/rig/attach_m1.yaml:

    attach_tip_dz = (h + standoff) - 0.04      the approach's tip setpoint above today's (+0.04)
    weld_point_z  =  h + standoff              the weld check's point (the same height)

with h the marker's height above the ring-origin plane and standoff at most the measured pull-in gap minus
1.5 cm (the loop's deadband and overshoot). It also reports the record the card asks for: bit-identical repeats,
the precision seen, and the longest gap between tip frames (cover body 24 during a second run to see what
arrives when it is occluded). With QUAD4's body (14, /drone_3) streaming it also prints the body-to-tip distance:
with the tether hanging in the air, its taut length, the reference of the combined M1 lift proof
(card 2026-10-10_combined_m1_contact).
"""
import argparse
import sys

import numpy as np

SETPOINT_TODAY = 0.04          # m above the ring-origin plane: target +0.05, TIP_STANDOFF_M -0.01


def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def in_ring_frame(tip, ring_pos, ring_quat):
    """tip (world) in the ring frame; ring_quat (w, x, y, z)."""
    return quat_to_rot(*ring_quat).T @ (np.asarray(tip, float) - np.asarray(ring_pos, float))


def summarise(samples, standoff):
    """samples: [(t, tip_world, ring_pos, ring_quat)]. Returns a dict for the printout."""
    rel = np.array([in_ring_frame(tp, rp, rq) for _, tp, rp, rq in samples])
    tips = np.array([s[1] for s in samples])
    t = np.array([s[0] for s in samples])
    h = float(rel[:, 2].mean())
    repeats = int(np.sum(np.all(np.diff(tips, axis=0) == 0.0, axis=1))) if len(tips) > 1 else 0
    steps = np.abs(np.diff(tips, axis=0))
    nz = steps[steps > 0]
    return {'n': len(samples), 'rate_hz': (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0.0,
            'mean': rel.mean(axis=0), 'sd': rel.std(axis=0), 'h': h,
            'repeats': repeats, 'smallest_step_m': float(nz.min()) if nz.size else None,
            'longest_gap_s': float(np.max(np.diff(t))) if len(t) > 1 else None,
            'attach_tip_dz': h + standoff - SETPOINT_TODAY, 'weld_point_z': h + standoff}


def body_to_tip(pairs):
    """pairs: [(body_world, tip_world)]. Distance stats (m), or None without pairs."""
    if not pairs:
        return None
    d = np.array([np.linalg.norm(np.asarray(t, float) - np.asarray(b, float)) for b, t in pairs])
    return {'n': len(d), 'mean': float(d.mean()), 'sd': float(d.std()), 'min': float(d.min()), 'max': float(d.max())}


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--standoff', type=float, default=0.0,
                    help='m above the face-on-plate marker height (<= pull-in gap - 0.015)')
    a = ap.parse_args(argv)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from interfaces.msg import MotionCaptureState
    rclpy.init()
    node = rclpy.create_node('tip_calibration')
    ring = {}
    body = {}
    samples = []
    pairs = []

    def on_ring(m):
        p, q = m.pose.position, m.pose.orientation
        ring['pos'], ring['quat'] = (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)

    def on_body(m):
        p = m.pose.position
        body['pos'] = (p.x, p.y, p.z)

    def on_tip(m):
        p = m.pose.position
        if 'pos' in body:
            pairs.append((body['pos'], (p.x, p.y, p.z)))
        if 'pos' in ring:
            samples.append((node.get_clock().now().nanoseconds * 1e-9, (p.x, p.y, p.z), ring['pos'], ring['quat']))

    node.create_subscription(MotionCaptureState, '/payload/motion_capture_state', on_ring, 10)
    node.create_subscription(MotionCaptureState, '/drone_3/motion_capture_state', on_body, 10)
    node.create_subscription(PoseStamped, '/magnet_tip_pose', on_tip, 50)
    t_end = node.get_clock().now().nanoseconds * 1e-9 + a.seconds
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if len(samples) < 2:
        print('no tip frames with a ring pose: is mocap up with bodies 8 and 24?')
        return 1
    s = summarise(samples, a.standoff)
    print(f"tip in the ring frame over {s['n']} frames ({s['rate_hz']:.0f} Hz): "
          f"x {s['mean'][0]:+.4f}  y {s['mean'][1]:+.4f}  z {s['mean'][2]:+.4f} m  "
          f"(sd {s['sd'][0] * 1e3:.1f} / {s['sd'][1] * 1e3:.1f} / {s['sd'][2] * 1e3:.1f} mm)")
    print(f"record: bit-identical repeats {s['repeats']}, smallest step "
          f"{s['smallest_step_m'] if s['smallest_step_m'] is None else round(s['smallest_step_m'] * 1e3, 3)} mm, "
          f"longest gap {s['longest_gap_s']:.3f} s")
    print(f"configs/rig/attach_m1.yaml:  attach_tip_dz: {s['attach_tip_dz']:+.3f}   weld_point_z: {s['weld_point_z']:.3f}"
          f"   (standoff {a.standoff:.3f} m)")
    b = body_to_tip(pairs)
    print('body 14 to tip 24: no body frames (/drone_3/motion_capture_state)' if b is None else
          f"body 14 to tip 24 over {b['n']} frames: mean {b['mean']:.4f} m  sd {b['sd'] * 1e3:.1f} mm  "
          f"range {b['min']:.4f}-{b['max']:.4f} (tether hanging in the air: the taut length)")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
