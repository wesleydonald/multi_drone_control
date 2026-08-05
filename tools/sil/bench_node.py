"""
tools/sil/bench_node.py
-----------------------
The SIL bench ROS node: it IS the simulator. It owns /clock, publishes mocap and IMU,
consumes ELRSCommand from the real trackers, integrates the plant, and scripts the
mission (ARM / TAKEOFF / WELD / LAND) in sim time.

Design note: docs/design/sil_bench.md. Two things there are worth repeating here
because they are easy to break by accident:

  * LOCKSTEP. Sim time advances only once every live tracker has published a command
    for the current step. That makes the result independent of machine load BY
    CONSTRUCTION. controller_mpc.py:141 records two runs of the same trajectory at RTF
    0.40 and 0.60 giving payload radius errors of -16% and -34% -- "which made every sim
    A/B silently incomparable". Do not replace this with a wall-clock loop.

  * The bench does NOT use sim time itself. It is the clock source; a clock source that
    waits for its own clock deadlocks.
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Time as TimeMsg
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray, Int32, String

from interfaces.msg import ELRSCommand, MotionCaptureState

from .plant import (Link, PayloadParams, QuadParams, SilPlant, quat_to_rot,
                    rot_to_quat)
from .standin import ApproachStandin


def _time_msg(t):
    m = TimeMsg()
    m.sec = int(t)
    m.nanosec = int(round((t - int(t)) * 1e9))
    if m.nanosec >= 1_000_000_000:          # rounding can carry
        m.sec += 1
        m.nanosec -= 1_000_000_000
    return m


class SilBench(Node):

    def __init__(self, scn, run_dir):
        super().__init__('sil_bench')
        # This node is the clock SOURCE. If it followed sim time it would wait for
        # itself, and nothing would ever tick.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        self.scn = scn
        self.run_dir = run_dir
        self.n = scn.n_total
        self.n_teth = scn.n_tethered

        # ── plant ────────────────────────────────────────────────────────────
        rho = scn.attach_rho()
        pos, load = scn.initial_state()
        quads = [QuadParams(mass=scn.drone_mass, thrust_c=scn.thrust_c)
                 for _ in range(self.n)]
        if scn.stands:
            # A stand under each TETHERED drone at its spawn height. The newcomer is a
            # free flyer and never gets one. See Scenario.stands for why this is not
            # cosmetic.
            for i in range(self.n_teth):
                quads[i].stand_z = float(pos[i][2])
        links = [Link(rho[i], scn.cable_len, attached=True) for i in range(self.n_teth)]
        for _ in range(self.n_teth, self.n):
            # newcomer: unattached until the scripted weld
            links.append(Link(np.zeros(3), scn.magnet_arm_len, attached=False))
        self.plant = SilPlant(quads, links,
                              PayloadParams(mass=scn.load_mass))
        self.plant.reset(pos, load)
        self.standin = ApproachStandin(thrust_c=scn.thrust_c)
        self.welded = [False] * self.n
        # Pose the stand-in holds from the weld instant until the real tracker takes
        # over. Frozen at the weld rather than tracking the live payload, because
        # online_join_planner STOPS on /magnet/object_attached -- it holds its last
        # reference instead of chasing a load that is now moving under the weld.
        self.hold_ref = [None] * self.n
        # Which newcomers the real tracker has actually taken over. See _handed_over.
        self.handover_step = {}
        self.weld_step = None

        # ── clock / step bookkeeping ────────────────────────────────────────
        self.sim_t = 0.0
        self.step_i = 0
        self.t_ready = None
        self.alive = set()
        self.cmd = [dict(ch=(0.0, 0.0, -1.0, 0.0), armed=False) for _ in range(self.n)]
        self.got_cmd = [False] * self.n
        self.stalls = 0
        self.stall_steps = []
        self.silent = [0] * self.n       # consecutive stalled steps per drone
        self.gone = set()                # trackers that have exited (LAND -> disarm)
        self.aborts = []
        self.ref0 = [None] * self.n          # node-0 reference position per drone
        self.payload_des = None
        self.events_done = set()
        self.event_log = []
        self.rows = []
        self.finished_reason = None

        qos = QoSProfile(depth=10)
        self.clock_pub = self.create_publisher(Clock, '/clock', qos)
        self.mocap_pub = [self.create_publisher(
            MotionCaptureState, f'/drone_{i}/motion_capture_state', 5)
            for i in range(self.n)]
        self.imu_pub = [self.create_publisher(Imu, f'/drone_{i}/imu', 10)
                        for i in range(self.n)]
        self.payload_pub = self.create_publisher(
            MotionCaptureState, '/payload/motion_capture_state', 5)
        self.fleet_cmd_pub = self.create_publisher(String, '/fleet/command', 10)
        self.detach_pub = self.create_publisher(Int32, '/fleet/detach', 10)
        self.magnet_pub = self.create_publisher(Bool, '/magnet/object_attached', 10)

        for i in range(self.n):
            self.create_subscription(
                ELRSCommand, f'/drone_{i}/ELRSCommand',
                lambda msg, k=i: self._cmd_cb(msg, k), 1)
            self.create_subscription(
                Float64MultiArray, f'/drone_{i}/reference_trajectory',
                lambda msg, k=i: self._ref_cb(msg, k), 1)
        self.create_subscription(String, '/fleet/abort', self._abort_cb, 5)
        self.create_subscription(Float64MultiArray, '/payload/desired_position',
                                 self._payload_des_cb, 5)

    # ── subscriptions ────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: ELRSCommand, i):
        self.cmd[i] = dict(ch=(float(msg.channel_0), float(msg.channel_1),
                               float(msg.channel_2), float(msg.channel_3)),
                           armed=bool(msg.armed))
        self.got_cmd[i] = True
        self.alive.add(i)

    def _ref_cb(self, msg: Float64MultiArray, i):
        d = msg.data
        if len(d) < 2:
            return
        n_nodes = int(d[0])
        if n_nodes < 1:
            return
        fields = (len(d) - 2) // n_nodes
        if fields < 3:
            return
        self.ref0[i] = np.array(d[2:5], dtype=float)

    def _abort_cb(self, msg: String):
        self.aborts.append((self.sim_t, msg.data))
        self.get_logger().error(f'[sil] /fleet/abort at t={self.sim_t:.2f}: {msg.data}')

    def _payload_des_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.payload_des = np.array(msg.data[:3], dtype=float)

    # ── publishing the plant state ───────────────────────────────────────────

    def _publish_clock(self):
        m = Clock()
        m.clock = _time_msg(self.sim_t)
        self.clock_pub.publish(m)

    def _mocap_msg(self, p, q, v, w):
        m = MotionCaptureState()
        m.header.stamp = _time_msg(self.sim_t)
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = float(q[0])
        m.pose.orientation.x = float(q[1])
        m.pose.orientation.y = float(q[2])
        m.pose.orientation.z = float(q[3])
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v)
        # BODY-frame angular velocity, matching payload_mocap_emulator
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = map(float, w)
        return m

    def _publish_state(self):
        for i in range(self.n):
            s = self.plant.drone_state(i)
            self.mocap_pub[i].publish(
                self._mocap_msg(s[0:3], s[3:7], s[7:10], s[10:13]))
            a = self.plant.imu(i)
            msg = Imu()
            msg.header.stamp = _time_msg(self.sim_t)
            msg.header.frame_id = f'drone_{i}'
            msg.linear_acceleration.x = float(a[0])
            msg.linear_acceleration.y = float(a[1])
            msg.linear_acceleration.z = float(a[2])
            self.imu_pub[i].publish(msg)
        ps = self.plant.payload_state()
        self.payload_pub.publish(
            self._mocap_msg(ps[0:3], ps[3:7], ps[7:10], ps[10:13]))

    # ── mission events ───────────────────────────────────────────────────────

    def _fire(self, ev):
        self.event_log.append((self.sim_t, ev.do, ev.arg if ev.arg is not None else ''))
        self.get_logger().warn(f'[sil] t={self.sim_t:.2f} EVENT {ev.do} '
                               f'{ev.arg if ev.arg is not None else ""}')
        if ev.do in ('ARM', 'TAKEOFF', 'LAND', 'DISARM', 'ESTOP'):
            self.fleet_cmd_pub.publish(String(data=ev.do))
            # /fleet/command is how a human drives the real stack, so the bench uses
            # exactly that -- including for drone 3, whose /drone_3/command is remapped
            # to /fleet/command by the launch file.
        elif ev.do == 'DETACH':
            self.plant.release(int(ev.arg))
            self.detach_pub.publish(Int32(data=int(ev.arg)))
        elif ev.do == 'WELD':
            self._weld(int(ev.arg) if ev.arg is not None else self.n_teth)
        else:
            raise ValueError(f'unknown scenario event {ev.do!r}')

    def _weld(self, i):
        """The deterministic weld harness (decision D5).

        Two things happen in the SAME instant, exactly as the Gazebo stack does them:
        the DetachableJoint fixes the magnet tip to the payload (here: the plant's rod
        engages at the tip's current position), and the magnet manager announces
        /magnet/object_attached, which is the single signal
        dissipative_node._magnet_attached_cb acts on to fold the newcomer into the
        network and hand control authority to its real tracker."""
        rho = self.scn.weld_point_body(self.plant, i)
        self.plant.weld(i, rho, self.scn.magnet_arm_len)
        self.welded[i] = True
        self.hold_ref[i] = self._weld_hover_ref()
        self.weld_step = self.step_i
        self.magnet_pub.publish(Bool(data=True))
        self.get_logger().warn(
            f'[sil] WELD drone {i}: rho_body=({rho[0]:+.3f},{rho[1]:+.3f},{rho[2]:+.3f}) '
            f'rest={self.scn.magnet_arm_len:.2f} m; /magnet/object_attached -> True')

    # ── the loop ─────────────────────────────────────────────────────────────

    def _weld_hover_ref(self):
        """Where the newcomer holds before the weld: over the LIVE weld target, one
        magnet-arm above it, so its tip sits on the target.

        Tracking the live payload matters. The first acceptance run held a FIXED pre-weld
        pose while the OCP lifted the payload 0.45 -> 0.6 m underneath it; by weld time
        the tip was ~0.1 m BELOW the payload centre, the rod welded through the box, and
        both configurations snapped instantly at |aCm| ~110 m/s^2 -- a rod-compression
        artefact, not the documented runaway. The real stack does not have this problem
        because attach_target_publisher republishes the payload's live mocap plus the
        offset and online_join_planner follows it; the stand-in must do the same."""
        ps = self.plant.payload_state()
        R = quat_to_rot(ps[3:7])
        target = ps[0:3] + R @ np.array([self.scn.weld_x, self.scn.weld_y,
                                         self.scn.weld_z_offset])
        return target + np.array([0.0, 0.0, self.scn.magnet_arm_len])

    def _handed_over(self, i):
        """Has the real tracker taken authority over newcomer i?

        NOT simply "has it welded". A tracker with planner_ref_pos None publishes
        armed-idle -- channel_2 = -1.0, throttle ZERO ("hold armed-idle on the ground",
        controller_mpc.py) -- and the newcomer cannot have a reference at the weld
        instant, because dissipative_node clears attach_pending on the SAME
        /magnet/object_attached message and only publishes on its next 10 Hz plan tick.
        Measured here: 100-160 ms of zero throttle, in which the drone falls 4-9 cm
        while rigidly welded to the load at an 8 cm lever arm.

        READ THIS BEFORE TRUSTING A RESULT FROM THIS BENCH: elrs_mux does NOT do what
        this function models. It switches on the first /magnet/object_attached and never
        checks whether our tracker has a reference (elrs_mux._attached_cb), so the real
        stack HAS that dead window. Gating on the reference here is a deliberate
        COUNTERFACTUAL -- it answers "if the handoff gap were closed, would the bench
        discriminate 45 from 65?" without touching controller code. Any run made with
        this in place is evidence about the control law, NOT about the shipped handoff.
        """
        return self.welded[i] and self.ref0[i] is not None

    def _apply_commands(self):
        for i in range(self.n):
            if i >= self.n_teth and not self._handed_over(i):
                # The bench's stand-in approach controller flies the newcomer (D5):
                # before the weld it closes on the target, and from the weld until the
                # real tracker has a reference it HOLDS the pose it welded at -- which
                # is what online_join_planner does, since it stops on
                # /magnet/object_attached rather than continuing to command.
                s = self.plant.drone_state(i)
                ref = self.hold_ref[i] if self.welded[i] else self._weld_hover_ref()
                self.plant.set_command(
                    i, *self.standin.channels(s[0:3], s[7:10], s[3:7], ref), armed=True)
                continue
            if i >= self.n_teth and i not in self.handover_step:
                self.handover_step[i] = self.step_i
                gap = ((self.step_i - self.weld_step) * self.scn.dt * 1e3
                       if self.weld_step is not None else float('nan'))
                self.get_logger().warn(
                    f'[sil] HANDOVER drone {i}: real tracker has a reference '
                    f'{gap:.0f} ms after the weld (stand-in held it through the gap; '
                    f'the shipped elrs_mux would NOT have)')
            c = self.cmd[i]
            self.plant.set_command(i, *c['ch'], armed=c['armed'])

    def _wait_for_commands(self):
        """Lockstep barrier: block until every drone we have EVER heard from has
        published a command for this step. A drone that has not booted yet is not
        required (its tracker may still be compiling acados); once it has spoken once
        it is required forever, so a tracker that dies mid-run stalls the bench
        loudly instead of being silently dropped."""
        for i in range(self.n):
            self.got_cmd[i] = False
        deadline = time.monotonic() + self.scn.step_timeout_s
        ok = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.002)
            if all(self.got_cmd[i] for i in self.alive):
                ok = True
                break
        # A tracker that stops speaking has EXITED, not stalled -- the normal way that
        # happens is LAND -> /fleet/landed -> the manager disarms -> CallbackManagerMulti
        # .request_shutdown() kills the node. Waiting the full timeout for a dead process
        # on every remaining step turned a 2.5 s tail into 40 s of wall clock on the
        # first smoke run. So drop it after a short grace and say so.
        for i in sorted(self.alive):
            if self.got_cmd[i]:
                self.silent[i] = 0
                continue
            self.silent[i] += 1
            if self.silent[i] >= 5:
                self.alive.discard(i)
                self.gone.add(i)
                self.get_logger().warn(
                    f'[sil] drone {i} tracker stopped publishing at t={self.sim_t:.2f} '
                    f'- treating it as exited')
        return ok

    def _log_row(self):
        ps = self.plant.payload_state()
        row = {'t': self.sim_t, 'step': self.step_i,
               'payload_x': ps[0], 'payload_y': ps[1], 'payload_z': ps[2],
               'payload_qw': ps[3], 'payload_qx': ps[4], 'payload_qy': ps[5],
               'payload_qz': ps[6], 'payload_tilt_deg': self.plant.load_tilt_deg()}
        if self.payload_des is not None:
            row.update(payload_ref_x=self.payload_des[0],
                       payload_ref_y=self.payload_des[1],
                       payload_ref_z=self.payload_des[2])
        else:
            row.update(payload_ref_x=math.nan, payload_ref_y=math.nan,
                       payload_ref_z=math.nan)
        for i in range(self.n):
            s = self.plant.drone_state(i)
            ac = self.plant.cable_accel(i)
            r = self.ref0[i]
            err = (float(np.linalg.norm(r - s[0:3])) if r is not None else math.nan)
            row.update({
                f'd{i}_x': s[0], f'd{i}_y': s[1], f'd{i}_z': s[2],
                f'd{i}_vx': s[7], f'd{i}_vy': s[8], f'd{i}_vz': s[9],
                f'd{i}_tilt_deg': self.plant.drone_tilt_deg(i),
                f'd{i}_ref_x': r[0] if r is not None else math.nan,
                f'd{i}_ref_y': r[1] if r is not None else math.nan,
                f'd{i}_ref_z': r[2] if r is not None else math.nan,
                f'd{i}_track_err': err,
                f'd{i}_acm': float(np.linalg.norm(ac)),
                f'd{i}_thr': self.plant.u[i][2],
                f'd{i}_armed': int(self.plant.armed[i]),
                f'd{i}_tension': self.plant.rod_tension(i),
                f'd{i}_elev_deg': self.plant.cable_elev_deg(i),
                f'd{i}_attached': int(self.plant.links[i].attached),
            })
        self.rows.append(row)

    def run(self):
        scn = self.scn
        t_wall0 = time.monotonic()
        ready_deadline = t_wall0 + scn.ready_timeout_s
        self.get_logger().info(
            f'[sil] waiting for {self.n} trackers to come up '
            f'(acados compile can take a while)...')

        while rclpy.ok():
            self._publish_clock()
            self._publish_state()
            ok = self._wait_for_commands()

            if self.t_ready is None:
                if len(self.alive) >= self.n:
                    self.t_ready = self.sim_t
                    self.get_logger().warn(
                        f'[sil] all {self.n} trackers alive at sim t='
                        f'{self.sim_t:.2f} ({time.monotonic()-t_wall0:.1f} s wall); '
                        f'scenario clock starts now')
                elif time.monotonic() > ready_deadline:
                    self.finished_reason = (
                        f'timeout waiting for trackers: alive={sorted(self.alive)} '
                        f'of {self.n} after {scn.ready_timeout_s:.0f} s')
                    return False
            else:
                if not ok:
                    self.stalls += 1
                    self.stall_steps.append(self.step_i)
                rel = self.sim_t - self.t_ready
                for k, ev in enumerate(scn.events):
                    if k not in self.events_done and rel >= ev.t:
                        self.events_done.add(k)
                        self._fire(ev)

            # The world is PAUSED until every tracker is up. Sim time still advances --
            # the trackers need their timers to fire in order to boot at all -- but the
            # plant does not integrate, so a slow acados compile cannot drift the
            # formation before the scenario has started. This is the equivalent of
            # bringing Gazebo up without -r.
            if self.t_ready is not None:
                self._apply_commands()
                self.plant.advance(scn.dt, scn.substeps)
            self.sim_t += scn.dt
            self.step_i += 1

            if self.t_ready is not None:
                self._log_row()
                if not self.plant.is_finite():
                    self.finished_reason = 'plant state went non-finite (NaN/Inf)'
                    return False
                if self.sim_t - self.t_ready >= scn.duration_s:
                    self.finished_reason = 'completed'
                    return True
                if len(self.gone) >= self.n:
                    # Every tracker has exited. After a LAND this is the correct,
                    # expected end of the mission; before one it is a crash, and the
                    # reason says which so a failed run cannot read as a clean one.
                    landed = any(e[1] == 'LAND' for e in self.event_log)
                    self.finished_reason = (
                        'controller stack exited after LAND (fleet disarmed)' if landed
                        else 'controller stack exited UNEXPECTEDLY (no LAND was sent)')
                    return bool(landed)
            if self.step_i % 250 == 0:
                self._progress(t_wall0)

        self.finished_reason = 'rclpy shutdown'
        return False

    def _progress(self, t_wall0):
        wall = time.monotonic() - t_wall0
        rel = 0.0 if self.t_ready is None else self.sim_t - self.t_ready
        rtf = (self.sim_t / wall) if wall > 0 else 0.0
        self.get_logger().info(
            f'[sil] t={rel:7.2f}s  wall={wall:7.1f}s  x{rtf:.2f} realtime  '
            f'load_z={self.plant.xL[2]:.3f} tilt={self.plant.load_tilt_deg():5.1f}deg  '
            f'stalls={self.stalls}')

    # ── output ───────────────────────────────────────────────────────────────

    def write_logs(self):
        if not self.rows:
            return
        logs = os.path.join(self.run_dir, 'logs')
        os.makedirs(logs, exist_ok=True)
        # ONE TIME ORIGIN for both files. events.csv has always been written relative to
        # t_ready while sil.csv carried the raw /clock, which starts wherever the ROS
        # stack happened to come up -- 33.68 s into R0016. Every event-relative metric
        # (§9.2: "event time from events.csv") would then look up its event 33.68 s late,
        # i.e. in the wrong phase of the run entirely, and silently return a number. The
        # raw clock is kept as t_clock so a log can still be lined up against ROS bags.
        base = self.t_ready or 0.0
        keys = list(self.rows[0].keys())
        keys = keys[:1] + ['t_clock'] + keys[1:]
        with open(os.path.join(logs, 'sil.csv'), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in self.rows:
                r = dict(r)
                r['t_clock'] = r['t']
                r['t'] = r['t'] - base
                w.writerow(r)
        with open(os.path.join(logs, 'events.csv'), 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['sim_time', 'event', 'arg'])
            for t, ev, arg in self.event_log:
                w.writerow([f'{t - base:.3f}', ev, arg])
            for t, reason in self.aborts:
                w.writerow([f'{t - base:.3f}', 'FLEET_ABORT', reason])
        return os.path.join(logs, 'sil.csv')

    def weld_time(self):
        for t, ev, _ in self.event_log:
            if ev == 'WELD':
                return t - (self.t_ready or 0.0)
        return None
