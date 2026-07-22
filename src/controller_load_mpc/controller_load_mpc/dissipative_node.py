"""
dissipative_node.py
--------------------
Decentralized DISSIPATIVE reference generator for a cable-suspended load -- a drop-in
alternative to planner_node.py (the centralized load-cable OCP) whose native operating
mode is a drone detaching mid-flight. It publishes the SAME per-drone reference wire
format on /drone_{i}/reference_trajectory, so the per-drone cable-aware MPC tracker in
controller_quad_load is reused unchanged.

Flight sequence (mirrors planner_node.py's seam):
  phase 'creep'    -- reuse CreepController: arc-sweep the rigid rods up from the
                      ground to a liftable elevation, decide handover.
  phase 'network'  -- seed the DissipativeNetwork from the measured taut config
                      (bumpless), ramp the desired load height up to target_z, and each
                      tick step the spring-damper network and publish its references.
  /fleet/detach k  -- net.detach(k), fire /drone_k/detach to Gazebo, and switch drone k
                      to a free-flight fly-away reference. The remaining nodes re-settle
                      with the load still suspended; no solver switch, no hand-tuned
                      transient.

Reused verbatim from the OCP stack: PlannerConfig (params.py), CreepController
(creep_controller.py), the geometry helpers, LoadTrajectory, the /fleet/* integration
and the /drone_{i}/reference_trajectory wire format (see _publish_ref, canonical copy in
planner_node.py). PlannerSolver / ReferenceBuilder / planner_ocp are intentionally NOT
imported -- the network replaces them.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String, Bool, Empty
from interfaces.msg import MotionCaptureState

from .geometry import quat_to_rot_np, attach_points, azimuth_slot_assignment
from .load_trajectory import LoadTrajectory
from .params import PlannerConfig
from .creep_controller import CreepController
from .dissipative_network import DissipativeNetwork, DissipativeParams

DRONE_MASS = 0.6                 # matches planner_node.DRONE_MASS
PLANNER_HZ = 10.0
G = 9.81
N_HORIZON = 20                   # reference horizon nodes (matches the tracker's N)
DT_HORIZON = 0.05                # s per horizon node (matches the OCP dt used downstream)

# Cable tautness gate (see planner_node.CABLE_TAUT_*): scale the tension feedforward by
# how length-taut each cable measures right now. Rigid rods read ~1 immediately.
CABLE_TAUT_LO_FRAC = 0.85
CABLE_TAUT_HI_FRAC = 1.00
FF_EASE_S = 1.0                  # ease the tension FF in over this long after lift starts


class DissipativeController(Node):
    def __init__(self):
        super().__init__('dissipative_controller')

        cfg = PlannerConfig(self)
        cfg.log(self.get_logger())
        self.n = cfg.n
        self.cable_len = cfg.cable_len
        self.attach_radius = cfg.attach_radius
        self.attach_z = cfg.attach_z
        self.load_mass = cfg.load_mass
        self.handover_elev_deg = cfg.handover_elev_deg
        self.handover_settle_s = cfg.handover_settle_s
        self.start_taut = cfg.start_taut
        self.target_z = cfg.target_z
        self.lift_ramp_vel = cfg.lift_ramp_vel
        self.auto_slot_assign = cfg.auto_slot_assign
        self.load_traj = cfg.load_traj
        self.traj_speed = cfg.traj_speed
        self.traj_distance = cfg.traj_distance
        self.traj_radius = cfg.traj_radius
        self.land_vel = cfg.land_vel

        # dissipative-specific tuning (retunable per world without touching code).
        p = self.declare_parameter
        self.diss = DissipativeParams(
            k_station=float(p('diss_k_station', 8.0).value),
            k_cable=float(p('diss_k_cable', 40.0).value),
            c=float(p('diss_c', 6.0).value),
            c_ring=float(p('diss_c_ring', 2.0).value),
            node_mass=float(p('diss_node_mass', 0.5).value),
            substeps=int(p('diss_substeps', 10).value),
            elev_deg=float(p('diss_elev_deg', 45.0).value))
        # Pace the lift reference to the MEASURED load: the desired height leads the
        # actual lift by at most this, so the reference stays within rod reach and
        # cannot run away from the lagging load and destabilise (the open-loop ramp
        # otherwise outruns the load, over-stretches the config, and it flips). Raise
        # for a faster but harder pull.
        self.lift_lead = float(p('lift_lead', 0.10).value)

        self.rho = attach_points(self.n, self.attach_radius, self.attach_z)
        self.net = DissipativeNetwork(
            self.n, self.rho, self.cable_len, DRONE_MASS, self.load_mass, G, self.diss)
        self.traj = LoadTrajectory(self.load_traj, self.traj_speed,
                                   self.traj_distance, self.traj_radius)
        self.N = N_HORIZON
        self.dt = DT_HORIZON

        # Phase-1 takeoff, reused verbatim (its own creep state). Fed the live measured
        # state each tick; publishes creep refs through _publish_ref until handover.
        self.creep = CreepController(
            self.n, self.rho, self.cable_len, self.N, self.dt, G,
            self.handover_elev_deg, PLANNER_HZ, self._drone_at, self._publish_ref,
            self.get_logger())

        # state
        self.load_state = None                 # [p(3), q(4 wxyz), v(3), w(3)]
        self.drone_pos = [None] * self.n
        self.drone_vel = [None] * self.n
        self.slot2drone = list(range(self.n))
        self._slots_assigned = False
        self.hover_xy = None
        self.phase = 'creep'                   # 'creep' -> 'network'
        self.takeoff_seen = False
        self.lift_z0 = None                    # load z latched at handover
        self.lift_progress = 0.0               # ramped lift above lift_z0 (m)
        self._settle_left = 0.0
        self._ff_t = 0.0                       # cable-FF soft-start clock
        self.traj_t = 0.0
        self.descending = False
        self._land_to_ground = False
        self._landed = False
        self.detached = [False] * self.n       # per physical drone
        self._diag_ctr = 0

        # subs (identical to the planner's)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state', self._payload_cb, 5)
        self.create_subscription(
            Int32, '/fleet/step',
            lambda msg: setattr(self, 'takeoff_seen',
                                self.takeoff_seen or msg.data > 0), 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)
        self.create_subscription(
            String, '/fleet/command', self._fleet_command_cb, 10)
        # NEW: detach command (physical drone id). Drops the node, fires the Gazebo
        # detach, and flies that drone away.
        self.create_subscription(
            Int32, '/fleet/detach', self._fleet_detach_cb, 10)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]
        # Gazebo DetachableJoint release triggers (bridged to gz.msgs.Empty in the
        # launch file). std_msgs/Empty published once per detach.
        self.detach_pub = [self.create_publisher(Empty, f'/drone_{i}/detach', 1)
                           for i in range(self.n)]
        self.landed_pub = self.create_publisher(Bool, '/fleet/landed', 1)
        self.load_ref_pub = self.create_publisher(
            Float64MultiArray, '/payload/desired_position', 5)

        self.create_timer(1.0 / PLANNER_HZ, self._tick)
        self.get_logger().info('[dissipative] ready, waiting for mocap...')

    # ── mocap callbacks (identical to planner_node) ─────────────────────────
    def _payload_cb(self, msg: MotionCaptureState):
        p = msg.pose.position
        o = msg.pose.orientation
        lv = msg.twist.linear
        av = msg.twist.angular
        self.load_state = np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z, av.x, av.y, av.z])
        if self.hover_xy is None:
            self.hover_xy = (p.x, p.y)

    def _drone_cb(self, msg: MotionCaptureState, i):
        p = msg.pose.position
        lv = msg.twist.linear
        self.drone_pos[i] = np.array([p.x, p.y, p.z])
        self.drone_vel[i] = np.array([lv.x, lv.y, lv.z])

    def _drone_at(self, i):
        """Measured position of the physical drone in network slot i."""
        return self.drone_pos[self.slot2drone[i]]

    def _assign_slots(self):
        self.slot2drone = azimuth_slot_assignment(
            self.drone_pos, self.load_state[0:2], self.n)
        self._slots_assigned = True
        self.get_logger().info(
            f'[dissipative] auto slot assignment (slot->drone): {self.slot2drone}')

    # ── fleet commands ──────────────────────────────────────────────────────
    def _fleet_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(
            f'[dissipative] /fleet/command: {cmd!r} (phase={self.phase} '
            f'takeoff_seen={self.takeoff_seen})')
        if cmd != 'LAND':
            return
        if self.phase != 'network' or not self.takeoff_seen:
            self.get_logger().warn('[dissipative] LAND ignored - not flying yet')
            return
        self.descending = True
        self._land_to_ground = True
        self._landed = False
        self.get_logger().info('[dissipative] LAND - descending to the floor')

    def _fleet_detach_cb(self, msg: Int32):
        """Detach a physical drone: drop its network node, fire the Gazebo
        DetachableJoint, and switch it to a fly-away reference. Idempotent."""
        d = int(msg.data)
        if not (0 <= d < self.n):
            self.get_logger().warn(f'[dissipative] /fleet/detach {d} out of range')
            return
        if self.phase != 'network':
            self.get_logger().warn('[dissipative] detach ignored - not in network phase')
            return
        if self.detached[d]:
            return
        # physical drone d occupies network slot slot2drone.index(d)
        slot = self.slot2drone.index(d)
        self.net.detach(slot)
        self.detached[d] = True
        self.detach_pub[d].publish(Empty())
        self.get_logger().info(
            f'[dissipative] DETACH drone {d} (slot {slot}); '
            f'{self.net.n_attached()} drones remain on the load')

    # ── helpers ─────────────────────────────────────────────────────────────
    def _cable_taut_gate(self, i):
        """gate in [0,1]: how length-taut cable i measures now (reused idea from
        planner_node._cable_taut_gate). Rigid rods read ~1 immediately."""
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        dist = float(np.linalg.norm(attach - self._drone_at(i)))
        d_lo = CABLE_TAUT_LO_FRAC * self.cable_len
        d_hi = CABLE_TAUT_HI_FRAC * self.cable_len
        return float(np.clip((dist - d_lo) / max(d_hi - d_lo, 1e-6), 0.0, 1.0))

    def _p_des_load(self):
        """Desired load position [x,y,z]: captured hover xy + lateral trajectory
        offset, and the ramped lift height."""
        z = (self.lift_z0 + self.lift_progress) if self.lift_z0 is not None \
            else float(self.load_state[2])
        z = min(self.target_z, z) if not self._land_to_ground else z
        dx, dy, _, _ = self.traj.offset_at(self.traj_t)
        return np.array([self.hover_xy[0] + dx, self.hover_xy[1] + dy, z])

    def _publish_ref(self, i, nodes):
        """Publish one drone's reference. `nodes` is a sequence of (p, v, a, a_cable),
        one per horizon node. Wire format canonical copy lives in
        planner_node.LoadPlanner._publish_ref -- keep the two in sync."""
        data = [float(self.N + 1), float(self.dt)]
        for p, v, a, ac in nodes:
            data += [float(p[0]), float(p[1]), float(p[2]),
                     float(v[0]), float(v[1]), float(v[2]),
                     float(a[0]), float(a[1]), float(a[2]),
                     float(ac[0]), float(ac[1]), float(ac[2])]
        msg = Float64MultiArray()
        msg.data = data
        self.ref_pub[i].publish(msg)

    def _publish_load_desired(self):
        if self.hover_xy is None:
            return
        pd = self._p_des_load()
        msg = Float64MultiArray()
        msg.data = [float(pd[0]), float(pd[1]), float(pd[2])]
        self.load_ref_pub.publish(msg)

    # ── main tick ────────────────────────────────────────────────────────────
    def _tick(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return
        if self.auto_slot_assign and not self._slots_assigned:
            self._assign_slots()
        self._publish_load_desired()

        if self.phase == 'creep' and self.start_taut:
            self._enter_network_phase('start_taut')

        if self.phase == 'creep':
            gates = [(self._cable_taut_gate(i), 0.0) for i in range(self.n)]
            handover, reason = self.creep.step(self.load_state, self.drone_pos, gates)
            if handover:
                self._enter_network_phase(reason)
            return

        # ── network phase ────────────────────────────────────────────────
        # Post-handover hold before lifting (matches the planner's settle): keep the
        # reference frozen at the seeded config so the tracker unwinds its position
        # error before the load breaks ground.
        if self.takeoff_seen and self._settle_left > 0.0:
            self._settle_left -= 1.0 / PLANNER_HZ
        elif self.takeoff_seen:
            self._ff_t += 1.0 / PLANNER_HZ
            self._advance_lift()

        load_pos = self.load_state[0:3]
        load_quat = self.load_state[3:7]
        load_vel = self.load_state[7:10]
        p_des = self._p_des_load()
        # step the spring-damper network one control tick
        self.net.step(load_pos, load_quat, load_vel, p_des, 1.0 / PLANNER_HZ)

        ff = float(np.clip(self._ff_t / FF_EASE_S, 0.0, 1.0))
        self._publish_all_refs(load_pos, load_quat, p_des, ff)
        self._diag(load_pos, load_quat, ff)

    def _advance_lift(self):
        """Ramp the desired load height: up to target_z at lift_ramp_vel, or down to
        the floor on LAND. Linear ramp -- the network + tracker absorb the ends."""
        if self.descending:
            floor = -1.5 if self._land_to_ground else 0.0
            rate = self.land_vel if self._land_to_ground else self.lift_ramp_vel
            self.lift_progress = max(floor, self.lift_progress - rate / PLANNER_HZ)
            done = self.lift_progress <= floor + 1e-6
            if done and not self._landed:
                self._landed = True
                if self._land_to_ground:
                    self.landed_pub.publish(Bool(data=True))
                self.get_logger().info('[dissipative] descent complete')
            elif self._landed and self._land_to_ground:
                self.landed_pub.publish(Bool(data=True))
        else:
            total = self.target_z - self.lift_z0
            # Pace the ramp to the MEASURED load: lead the actual lift by at most
            # lift_lead so the reference stays within rod reach and can't run away from
            # the lagging load (which over-stretches the config and flips it). As the
            # load rises the cap rises with it.
            measured = float(self.load_state[2]) - self.lift_z0
            cap = min(total, measured + self.lift_lead)
            self.lift_progress = min(
                cap, self.lift_progress + self.lift_ramp_vel / PLANNER_HZ)
            lift_done = self.lift_progress >= total - 1e-6
            if self.load_traj != 'hover' and lift_done:
                self.traj_t += 1.0 / PLANNER_HZ
                if self.traj.complete(self.traj_t):
                    self.descending = True
                    self.get_logger().info(
                        '[dissipative] trajectory complete - descending')

    def _publish_all_refs(self, load_pos, load_quat, p_des, ff):
        for i in range(self.n):
            drone = self.slot2drone[i]
            if self.detached[drone]:
                # free-flight: rise and hold, no cable term (tracker handles a_cable=0).
                node = self.net.fly_away_reference(i)
                self._publish_ref(drone, [node] * (self.N + 1))
                continue
            gate = self._cable_taut_gate(i) * ff
            p_ref, v_ref, a_ff, a_cable = self.net.reference(
                i, load_pos, load_quat, p_des, taut_gate=gate)
            # constant-velocity horizon extrapolation of the node reference, matching
            # how the creep refs are shaped (the tracker tracks a receding horizon).
            nodes = [(p_ref + v_ref * self.dt * k, v_ref, a_ff, a_cable)
                     for k in range(self.N + 1)]
            self._publish_ref(drone, nodes)

    def _enter_network_phase(self, reason):
        """creep -> network: seed the network from the measured taut config (bumpless)
        and latch the lift-ramp start height."""
        self.phase = 'network'
        self.lift_z0 = float(self.load_state[2])
        self.lift_progress = 0.0
        self._settle_left = self.handover_settle_s
        self._ff_t = 0.0
        seed = [self._drone_at(i) for i in range(self.n)]
        self.net.seed(seed)
        self.get_logger().info(
            f'[dissipative] {reason} - network active (lift from z={self.lift_z0:.2f})')

    def _diag(self, load_pos, load_quat, ff):
        self._diag_ctr += 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) != 0:
            return
        R = quat_to_rot_np(load_quat)
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
        elevs = []
        for i in range(self.n):
            if self.detached[self.slot2drone[i]]:
                continue
            d = (load_pos + R @ self.rho[i]) - self._drone_at(i)
            nd = float(np.linalg.norm(d))
            elevs.append(np.degrees(np.arcsin(np.clip(-d[2] / max(nd, 1e-6), -1, 1))))
        e = ' '.join(f'{x:.0f}' for x in elevs)
        z_tgt = min(self.target_z, (self.lift_z0 or 0.0) + self.lift_progress)
        self.get_logger().info(
            f'[dissipative] load_z={load_pos[2]:.2f} z_tgt={z_tgt:.2f} ff={ff:.2f} '
            f'n_att={self.net.n_attached()} tilt={tilt:.1f}deg elev=[{e}]')


def main(args=None):
    rclpy.init(args=args)
    node = DissipativeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
