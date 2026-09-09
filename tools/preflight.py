#!/usr/bin/env python3
"""
tools/preflight.py — the pre-arm checklist (THESIS_PLAN §5.4), run against the LIVE stack.

    ./tools/preflight.py                       # 4 s of listening, 3 drones
    ./tools/preflight.py --drones 3 --attach   # also expect the newcomer (drone 3)
    ./tools/preflight.py --real                # require telemetry + battery bars

What it checks, in the order things fail at the rig:
  1. mocap: every drone and the payload publishing, fresh, at a sane rate, no NaNs
  2. geometry from mocap: each tethered drone's distance to ITS rim attach point equals
     the cable length the controller was told (the 2026-08-03 slot-assignment bug and
     every geometry mismatch since present as "the controller cannot fly")
  3. controller parameters READ BACK from the running planner: cable_len, load_mass,
     attach_radius, attach_azimuths_deg, thrust_ratio -- launch args have silently failed
     to plumb through before; the read-back is the only version that cannot lie
  4. (--real) telemetry per drone: battery above the bar, link present
  5. the fleet has NOT been armed yet (the point of a pre-arm check)

Writes a flight-card section to results/preflight/<timestamp>.md and exits non-zero on
any FAIL, so it can gate a launch script.
"""
import argparse, datetime, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from interfaces.msg import MotionCaptureState, Telemetry

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


class Preflight(Node):
    def __init__(self, n, attach, real, listen_s):
        super().__init__('preflight')
        self.n, self.attach, self.real, self.listen_s = n, attach, real, listen_s
        ids = list(range(n + (1 if attach else 0)))
        self.pose = {i: [] for i in ids}
        self.payload = []
        self.tele = {i: None for i in ids}
        self.armed_seen = False
        for i in ids:
            self.create_subscription(MotionCaptureState, f'/drone_{i}/motion_capture_state',
                                     lambda m, k=i: self.pose[k].append((time.time(), m)), 20)
            self.create_subscription(Telemetry, f'/drone_{i}/telemetry',
                                     lambda m, k=i: self.tele.__setitem__(k, m), 5)
        self.create_subscription(MotionCaptureState, '/payload/motion_capture_state',
                                 lambda m: self.payload.append((time.time(), m)), 20)
        self.create_subscription(String, '/fleet/status', self._status_cb, 1)
        self.status = ''

    def _status_cb(self, msg):
        self.status = msg.data

    def read_params(self, node_name, names, timeout=3.0):
        """Read parameters straight off the node's get_parameters service (rclpy's
        parameter client is not in this Humble build)."""
        cli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            return None
        out = {}
        for name, pv in zip(names, fut.result().values):
            out[name] = (pv.string_value if pv.type == 4 else pv.double_value if pv.type == 3
                         else pv.integer_value if pv.type == 2 else pv.bool_value if pv.type == 1 else None)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drones', type=int, default=3)
    ap.add_argument('--attach', action='store_true', help='expect a newcomer drone too')
    ap.add_argument('--real', action='store_true', help='require telemetry and battery bars')
    ap.add_argument('--listen', type=float, default=4.0)
    ap.add_argument('--battery-min', type=float, default=15.2, help='V, 4S pack')
    ap.add_argument('--planner', default='dissipative_controller')
    a = ap.parse_args()
    rclpy.init()
    pf = Preflight(a.drones, a.attach, a.real, a.listen)
    t0 = time.time()
    while time.time() - t0 < a.listen:
        rclpy.spin_once(pf, timeout_sec=0.05)

    lines, ok_all = [], True
    def check(ok, what, detail=''):
        nonlocal ok_all
        ok_all = ok_all and ok
        lines.append(f"  [{'PASS' if ok else 'FAIL'}] {what:40s} {detail}")

    # 1. mocap freshness/rate
    now = time.time()
    for i, msgs in pf.pose.items():
        rate = len(msgs) / a.listen
        fresh = (now - msgs[-1][0]) < 0.25 if msgs else False
        nan = any(not math.isfinite(v) for _, m in msgs[-5:] for v in (m.pose.position.x, m.pose.position.y, m.pose.position.z)) if msgs else True
        check(bool(msgs) and fresh and rate > 20 and not nan, f'mocap drone {i}',
              f'{rate:.0f} Hz, {"fresh" if fresh else "STALE"}{", NaN" if nan else ""}')
    prate = len(pf.payload) / a.listen
    check(bool(pf.payload) and prate > 20, 'mocap payload', f'{prate:.0f} Hz')

    # 3. controller parameters, read back
    names = ['cable_len', 'load_mass', 'attach_radius', 'attach_azimuths_deg', 'thrust_ratio', 'attach_z']
    prm = pf.read_params(a.planner, names)
    if prm is None:
        check(False, 'planner parameters', f'node {a.planner!r} not reachable')
        cable_len, rho = None, None
    else:
        lines.append('  planner read-back: ' + ', '.join(f'{k}={v}' for k, v in prm.items()))
        cable_len = float(prm['cable_len'])
        from controller_load_mpc.geometry import attach_points
        rho = attach_points(a.drones, float(prm['attach_radius']), float(prm['attach_z']),
                            prm['attach_azimuths_deg'] or None)
        check(0.3 <= cable_len <= 1.5, 'cable_len plausible', f'{cable_len:.3f} m')
        check(float(prm['load_mass']) > 0.05, 'load_mass plausible', f'{prm["load_mass"]} kg')

    # 2. geometry from mocap: drone-to-rim-point distance vs cable_len (slot by nearest rim point)
    if pf.payload and rho is not None and cable_len is not None:
        _, pm = pf.payload[-1]
        pl = np.array([pm.pose.position.x, pm.pose.position.y, pm.pose.position.z])
        R = quat_to_rot([pm.pose.orientation.w, pm.pose.orientation.x, pm.pose.orientation.y, pm.pose.orientation.z])
        rim = [pl + R @ r for r in rho]
        used = set()
        for i in range(a.drones):
            if not pf.pose[i]:
                continue
            _, m = pf.pose[i][-1]
            d = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            dists = [np.linalg.norm(d - p) for p in rim]
            k = int(np.argmin([x if j not in used else 1e9 for j, x in enumerate(dists)]))
            used.add(k)
            err = dists[k] - cable_len
            check(abs(err) < 0.06, f'drone {i} rod length (rim point {k})',
                  f'{dists[k]:.3f} m vs {cable_len:.3f} ({err:+.3f})')
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        check(tilt < 25.0, 'payload resting level', f'{tilt:.1f} deg')

    # 4. telemetry
    if a.real:
        for i, tm in pf.tele.items():
            if tm is None:
                check(False, f'telemetry drone {i}', 'no message')
            else:
                check(tm.battery_voltage >= a.battery_min, f'battery drone {i}',
                      f'{tm.battery_voltage:.2f} V (bar {a.battery_min}), RSSI {tm.rssi} dBm')

    # 5. not armed
    check('ARM' not in pf.status.upper() or 'PLANNER' in pf.status.upper() or pf.status == '',
          'fleet not yet flying', f'status: {pf.status or "(none)"}')

    rclpy.shutdown()
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(REPO, 'results', 'preflight'); os.makedirs(out_dir, exist_ok=True)
    card = [f'# Pre-flight {stamp}', f'drones={a.drones} attach={a.attach} real={a.real}', ''] + lines + ['', f'RESULT: {"GO" if ok_all else "NO-GO"}']
    with open(os.path.join(out_dir, f'{stamp}.md'), 'w') as f:
        f.write('\n'.join(card) + '\n')
    print('\n'.join(card))
    print(f'\n  written to results/preflight/{stamp}.md')
    sys.exit(0 if ok_all else 1)


if __name__ == '__main__':
    main()
