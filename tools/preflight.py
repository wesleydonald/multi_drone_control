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
     to plumb through before; the read-back is the only version that cannot lie. Also the
     flight settings (start_taut, handover_elev_deg, creep_vel, z_ki, lift_ramp_vel,
     reconfig_mode, kt_trim, control_mode) and each radio's magnet_channel/magnet_initial
  2b. disc fit (any n >= 3): the drones' circle centre vs the payload origin, the fitted
     rod vs cable_len, and each drone's measured bearing vs the typed attach azimuths
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

from preflight_geometry import (bearings_deg, fit_circle, match_azimuths, spacing_deg,
                                typed_azimuths, yaw_deg_from_quat)

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
        if fut.result() is None or not fut.result().values:
            # rclpy answers an EMPTY list if any requested name is undeclared on that node
            return None
        out = {}
        for name, pv in zip(names, fut.result().values):
            out[name] = _pv(pv)
        return out

    def read_params_each(self, node_name, names, timeout=1.0):
        """Like read_params, one name per request, so a name the node does not declare
        reads as None instead of blanking the rest. None overall = node not reachable."""
        cli = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        out = {}
        for name in names:
            fut = cli.call_async(GetParameters.Request(names=[name]))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            vals = fut.result().values if fut.result() is not None else []
            out[name] = _pv(vals[0]) if vals else None
        return out


def _pv(pv):
    return (pv.string_value if pv.type == 4 else pv.double_value if pv.type == 3
            else pv.integer_value if pv.type == 2 else pv.bool_value if pv.type == 1 else None)


def _fmt(d):
    return ', '.join(f'{k}={"(undeclared)" if v is None else v}' for k, v in d.items())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drones', type=int, default=3)
    ap.add_argument('--attach', action='store_true', help='expect a newcomer drone too')
    ap.add_argument('--real', action='store_true', help='require telemetry and battery bars')
    ap.add_argument('--listen', type=float, default=4.0)
    ap.add_argument('--battery-min', type=float, default=22.8, help='V, 6S pack (3.8 V/cell)')
    ap.add_argument('--max-ground-z', type=float, default=0.30,
                    help='m; a drone above this counts as already flying (taut rods from the floor sit ~0.38)')
    ap.add_argument('--planner', default='dissipative_controller')
    ap.add_argument('--detach', action='store_true',
                    help='this flight DETACHES (R6/R7): require reconfig_mode ocp and detach_magnet true '
                         '(real_dissipative defaults to network/false; the network detach tilted the ring 26-48 deg)')
    ap.add_argument('--az-tol-deg', type=float, default=10.0,
                    help='deg; FAIL when a drone\'s measured bearing is further than this from its typed azimuth')
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
    # R1 free hover flies without the ring: its mocap and the ring geometry below are not required
    free = a.planner == 'free_hover'
    if free:
        lines.append(f'  mocap payload: {prate:.0f} Hz (free hover: not required; ring geometry checks skipped)')
    else:
        check(bool(pf.payload) and prate > 20, 'mocap payload', f'{prate:.0f} Hz')

    # 3. controller parameters, read back
    names = ['cable_len', 'load_mass', 'attach_radius', 'attach_azimuths_deg', 'attach_z']
    prm = None if free else pf.read_params(a.planner, names)
    if free:
        fh = pf.read_params_each(a.planner, ['hover_z', 'climb_vel', 'land_vel'])
        check(fh is not None, 'free_hover parameters', 'reachable' if fh else 'node not reachable')
        if fh is not None:
            lines.append('  free_hover read-back: ' + _fmt(fh))
        cable_len, rho = None, None
    elif prm is None:
        check(False, 'planner parameters', f'node {a.planner!r} not reachable or a name undeclared')
        cable_len, rho = None, None
    else:
        lines.append('  planner read-back: ' + ', '.join(f'{k}={v}' for k, v in prm.items()))
        fl = pf.read_params_each(a.planner, ['start_taut', 'handover_elev_deg', 'handover_settle_s',
                                             'creep_vel', 'target_z', 'lift_ramp_vel', 'z_ki',
                                             'drone_mass', 'load_traj', 'reconfig_mode'])
        if fl is not None:
            lines.append('  planner flight read-back: ' + _fmt(fl))
        if a.detach:
            dm = pf.read_params_each(a.planner, ['detach_magnet']) or {'detach_magnet': None}
            mode = (fl or {}).get('reconfig_mode')
            check(mode == 'ocp', 'detach: reconfig_mode ocp', f'{mode!r} (type reconfig_mode:=ocp)')
            check(dm['detach_magnet'] is True, 'detach: detach_magnet true',
                  f'{dm["detach_magnet"]!r} (type detach_magnet:=true, or the magnet stays ON)')
    # per-tracker read-back: kT and the control architecture actually running
    for i in range(a.drones + (1 if a.attach else 0)):
        tp = pf.read_params(f'controller_{i}', ['thrust_ratio', 'control_mode'])
        if tp is None:
            check(False, f'tracker {i} parameters', 'not reachable')
        else:
            kt = pf.read_params_each(f'controller_{i}', ['kt_trim']) or {'kt_trim': None}
            lines.append(f'  tracker {i} read-back: thrust_ratio={tp["thrust_ratio"]} control_mode={tp["control_mode"]} '
                         f'kt_trim={kt["kt_trim"]}')
            check(tp['thrust_ratio'] > 0, f'tracker {i} kT set', f'{tp["thrust_ratio"]}')
    # radios: the tether magnet channel and boot state, per drone (real_io_launch only)
    chans = {}
    for i in range(a.drones + (1 if a.attach else 0)):
        ep = pf.read_params_each(f'drone_{i}/elrs_interface', ['magnet_channel', 'magnet_initial'])
        if ep is None:
            if a.real:
                check(False, f'radio {i} parameters', 'elrs_interface not reachable')
            else:
                lines.append(f'  radio {i}: elrs_interface not running (sim or no real_io)')
            continue
        chans[i] = ep['magnet_channel']
        lines.append(f'  radio {i} read-back: magnet_channel={ep["magnet_channel"]} '
                     f'magnet_initial={ep["magnet_initial"]!r}')
    if len(chans) > 1:
        # all None = no radio answered (a node stuck in its serial reconnect loop): unverified, not identical
        check(len(set(chans.values())) == 1 and None not in chans.values(),
              'magnet_channel identical on every radio',
              ', '.join(f'd{i}={c}' for i, c in chans.items()))
    if prm is not None:
        cable_len = float(prm['cable_len'])
        from controller_load_mpc.geometry import attach_points
        rho = attach_points(a.drones, float(prm['attach_radius']), float(prm['attach_z']),
                            prm['attach_azimuths_deg'] or None)
        check(0.3 <= cable_len <= 1.5, 'cable_len plausible', f'{cable_len:.3f} m')
        check(float(prm['load_mass']) > 0.05, 'load_mass plausible', f'{prm["load_mass"]} kg')

    # 1b. each drone's RESTING attitude from mocap. A rigid body defined while the airframe
    # was tilted (or with Z not up) reports a large tilt on the ground; the tracker steers
    # with that same quaternion, and the envelope trips the instant it counts as airborne
    # (drone 1 read 123.6 deg on 2026-09-16). Fix the body in the mocap software, not here.
    for i in range(a.drones):
        if not pf.pose[i]:
            continue
        q = pf.pose[i][-1][1].pose.orientation
        Rd = quat_to_rot([q.w, q.x, q.y, q.z])
        tilt_d = math.degrees(math.acos(max(-1.0, min(1.0, Rd[2, 2]))))
        check(tilt_d < 15.0, f'drone {i} resting level',
              f'{tilt_d:.1f} deg (mocap body orientation; >15 = re-create the rigid body level)')

    # 1c. disc centre from the drones themselves. Three drones placed radially out from
    # their magnets with equal rods lie on a circle centred on the DISC CENTRE with radius
    # rim + rod. The 2026-09-16 rig logs put that centre 3-9 cm from the payload rigid
    # body's origin (its pivot is off the geometric centre), the fitted rod at 0.46 m against
    # the 0.50 told, and the magnets ~115/130/115 deg apart: every rim point the planner
    # used was wrong, which is what "two drones heading for one slot" was.
    if not free and a.drones >= 3 and all(pf.pose[i] for i in range(a.drones)) and pf.payload:
        P = np.array([[pf.pose[i][-1][1].pose.position.x, pf.pose[i][-1][1].pose.position.y]
                      for i in range(a.drones)])
        fit = fit_circle(P)
        if fit is not None:
            cx, cy, rad, rms = fit
            pm = pf.payload[-1][1].pose.position; off = float(math.hypot(cx - pm.x, cy - pm.y))
            gaps = spacing_deg(bearings_deg(P, (cx, cy)))
            rim = float(prm['attach_radius']) if prm is not None else 0.25
            lines.append(f'  disc fit from the {a.drones} drones: centre ({cx:+.3f},{cy:+.3f}), radius {rad:.3f} m '
                         f'(rms {rms * 100:.1f} cm) -> rod {rad - rim:.3f} m for rim {rim:.2f} (valid with the rods flat '
                         f'on the floor); drone spacing {[round(g) for g in gaps]} deg')
            check(off < 0.03, 'payload body origin at the disc centre',
                  f'{off * 100:.1f} cm off the drones\' circle centre (re-set the rigid body pivot in the mocap software)')
            if cable_len is not None:
                check(abs(rad - rim - cable_len) < 0.04, 'cable_len matches the fitted rod',
                      f'fitted {rad - rim:.3f} m vs told {cable_len:.3f} m')

    # 2c. typed attach azimuths vs where the drones actually are, in the LOAD frame (bearing
    # about the disc centre minus the payload yaw). Matched in any order, as the planner's auto
    # slot assignment does. A miss here is a drone on the wrong plate, a wrong azimuth list, or
    # the payload body's +x not on plate 0.
    if (a.drones >= 2 and prm is not None and pf.payload
            and all(pf.pose[i] for i in range(a.drones))):
        P = np.array([[pf.pose[i][-1][1].pose.position.x, pf.pose[i][-1][1].pose.position.y]
                      for i in range(a.drones)])
        pm = pf.payload[-1][1].pose
        fit = fit_circle(P) if a.drones >= 3 else None
        centre = (fit[0], fit[1]) if fit is not None else (pm.position.x, pm.position.y)
        yaw = yaw_deg_from_quat(pm.orientation.w, pm.orientation.x, pm.orientation.y, pm.orientation.z)
        try:
            typed = typed_azimuths(prm['attach_azimuths_deg'], a.drones)
        except ValueError as e:
            check(False, 'typed attach azimuths', str(e))
        else:
            meas = bearings_deg(P, centre, yaw)
            slot2drone, err = match_azimuths(meas, typed)
            lines.append(f'  payload yaw {yaw:+.1f} deg; slot->drone {slot2drone} (the planner\'s auto slot line '
                         f'should print the same)')
            for s_, d_ in enumerate(slot2drone):
                check(abs(err[s_]) <= a.az_tol_deg, f'drone {d_} on typed azimuth {typed[s_]:.0f}',
                      f'measured {meas[d_]:.1f} deg ({err[s_]:+.1f}; plate {meas[d_] / 30.0:.1f})')

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

    # 5. not flying yet: every drone on the ground and no network phase announced
    zs = [pf.pose[i][-1][1].pose.position.z for i in pf.pose if pf.pose[i]]
    on_ground = bool(zs) and max(zs) < a.max_ground_z
    check(on_ground and not pf.status.upper().startswith('NETWORK'),
          'fleet not yet flying', f'max drone z {max(zs):.2f} m; status: {pf.status or "(none)"}' if zs else 'no poses')

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
