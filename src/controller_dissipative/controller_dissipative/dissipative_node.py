"""
dissipative_node.py
--------------------
OCP takeoff + DISSIPATIVE-network hold/detach for a cable-suspended load.

The pure decentralized network cannot break the load off the ground from the shallow
(~23 deg) creep handover -- the per-drone cable-aware MPC tracker stalls into a hover
rather than executing the analytic lift, the same wall that makes an open-loop analytic
takeoff unstable. So takeoff is delegated to the PROVEN centralized OCP planner
(controller_load_mpc.LoadPlanner): this node SUBCLASSES it and reuses the entire
fixed-topology flight -- creep, coupled-OCP lift, and stable hover -- UNCHANGED. The
centralized package is imported read-only; nothing in it is modified (we override _plan
here to DISPATCH on phase, rather than adding a hook to the parent).

At a /fleet/detach command the node hands the fleet over to the decentralized
DissipativeNetwork, whose NATIVE operating mode is a member leaving:
  * seed the network (bumpless) from the current airborne taut config,
  * drop the detaching drone's node, fire its Gazebo DetachableJoint, fly it away,
  * drive the remaining n-1 drones from the network -- no solver switch, no hand-tuned
    redistribution transient.
The dissipative network is only ever engaged AIRBORNE (a taut hover), where it is
verified stable (see verify_dissipative.py); it never attempts the ground break-off.

Phases: 'creep' -> 'planner'  (inherited: OCP takeoff + hover)
                 -> 'network'  (this class: dissipative hold of the reduced fleet,
                                entered at the first detach command).

Everything else -- the reference wire format, the fleet manager, the per-drone tracker --
is the shared stack, unchanged.
"""
import numpy as np
import rclpy
from std_msgs.msg import Int32, Empty

from controller_load_mpc.geometry import quat_to_rot_np
from controller_load_mpc.planner_node import LoadPlanner, DRONE_MASS, PLANNER_HZ

from controller_dissipative.dissipative_network import (
    DissipativeNetwork, DissipativeParams)


class DissipativeController(LoadPlanner):
    def __init__(self):
        super().__init__()          # builds the OCP planner (creep -> OCP -> hover)

        # dissipative-network tuning (retunable per world without touching code).
        p = self.declare_parameter
        self.diss = DissipativeParams(
            k_pay=float(p('diss_k_pay', 40.0).value),
            k_anchor=float(p('diss_k_anchor', 40.0).value),
            k_ring=float(p('diss_k_ring', 20.0).value),
            c=float(p('diss_c', 6.0).value),
            node_mass=float(p('diss_node_mass', 0.5).value),
            substeps=int(p('diss_substeps', 10).value),
            elev_deg=float(p('diss_elev_deg', 45.0).value))
        self.net = DissipativeNetwork(
            self.n, self.rho, self.cable_len, DRONE_MASS, self.load_mass,
            self.dyn.g, self.diss)
        self.detached = [False] * self.n       # per physical drone
        self._net_p_des = None                 # captured hover target held by the network
        self._net_diag_ctr = 0

        # detach command (physical drone id): hands the fleet to the network + releases.
        self.create_subscription(Int32, '/fleet/detach', self._fleet_detach_cb, 10)
        # Gazebo DetachableJoint release triggers (bridged to gz.msgs.Empty in launch).
        self.detach_pub = [self.create_publisher(Empty, f'/drone_{i}/detach', 1)
                           for i in range(self.n)]
        self.get_logger().info(
            '[dissipative] OCP-takeoff + dissipative-detach ready')

    # ── control tick: dispatch on phase (no parent modification) ─────────────
    def _plan(self):
        """The inherited timer calls this. In the network phase we run the spring-damper
        network; otherwise the inherited OCP planner flies (creep -> lift -> hover)."""
        if self.phase == 'network':
            self._network_plan()
        else:
            super()._plan()

    # ── detach: hand over to the dissipative network, then drop a drone ──────
    def _fleet_detach_cb(self, msg: Int32):
        d = int(msg.data)
        if not (0 <= d < self.n):
            self.get_logger().warn(f'[dissipative] /fleet/detach {d} out of range')
            return
        if self.phase not in ('planner', 'network'):
            self.get_logger().warn('[dissipative] detach ignored - not flying yet')
            return
        # first detach also performs the OCP -> network handover (seeds bumplessly).
        if self.phase == 'planner':
            self._enter_network_phase()
        if self.detached[d]:
            return
        slot = self.slot2drone.index(d)          # network slot of physical drone d
        self.net.detach(slot)
        self.detached[d] = True
        self.detach_pub[d].publish(Empty())
        self.get_logger().info(
            f'[dissipative] DETACH drone {d} (slot {slot}); '
            f'{self.net.n_attached()} drones remain on the load')

    def _enter_network_phase(self):
        """planner -> network: seed the spring-damper network from the current (airborne,
        taut) measured drone positions so the handover is bumpless, and capture the hover
        target the network will hold. The OCP is no longer solved after this."""
        self.phase = 'network'
        self.net.seed([self._drone_at(i) for i in range(self.n)])
        self._net_p_des = self._current_load_des()
        self.get_logger().info(
            '[dissipative] handover OCP -> dissipative network (airborne)')

    def _current_load_des(self):
        """The load position the network holds after handover: the OCP's current lift
        target (captured hover xy + trajectory offset + lifted height)."""
        z = float(self.load_state[2]) if self.lift_z0 is None else \
            min(self.target_z, self.lift_z0 + self.lift_progress)
        dx, dy, _, _ = self.traj.offset_at(self.traj_t)
        return np.array([self.hover_xy[0] + dx, self.hover_xy[1] + dy, z])

    # ── network flight: hold the reduced fleet + fly the detached drones away ─
    def _network_plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return
        self._publish_load_desired()             # keep the RViz/desired topic alive
        p_des = self._net_p_des
        load_pos = self.load_state[0:3]
        load_quat = self.load_state[3:7]
        load_vel = self.load_state[7:10]
        self.net.step(load_pos, load_quat, load_vel, p_des, 1.0 / PLANNER_HZ)
        for i in range(self.n):
            drone = self.slot2drone[i]
            if self.detached[drone]:
                node = self.net.fly_away_reference(i)
                self._publish_ref(drone, [node] * (self.N + 1))
                continue
            gate, _ = self._cable_taut_gate(i)
            p_ref, v_ref, a_ff, a_cable = self.net.reference(
                i, load_quat, p_des, taut_gate=gate)
            # hold p_ref constant across the horizon (one network target per tick).
            self._publish_ref(drone, [(p_ref, v_ref, a_ff, a_cable)] * (self.N + 1))
        self._net_diag(load_pos, load_quat, p_des)

    def _net_diag(self, load_pos, load_quat, p_des):
        self._net_diag_ctr += 1
        if self._net_diag_ctr % int(max(PLANNER_HZ, 1)) != 0:
            return
        R = quat_to_rot_np(load_quat)
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
        self.get_logger().info(
            f'[diss-net] load_z={load_pos[2]:.3f} z_tgt={p_des[2]:.2f} '
            f'tilt={tilt:.1f}deg n_att={self.net.n_attached()}')
        for i in range(self.n):
            drone = self.slot2drone[i]
            if self.detached[drone]:
                continue
            dvec = (load_pos + R @ self.rho[i]) - self._drone_at(i)
            nd = float(np.linalg.norm(dvec))
            dz = np.degrees(np.arcsin(np.clip(-dvec[2] / max(nd, 1e-6), -1, 1)))
            gate, _ = self._cable_taut_gate(i)
            p_ref, _, a_ff, a_cable = self.net.reference(
                i, load_quat, p_des, taut_gate=gate)
            perr = float(np.linalg.norm(p_ref - self._drone_at(i)))
            aff_t = np.degrees(np.arctan2(
                float(np.linalg.norm(a_ff[:2])), float(a_ff[2])))
            self.get_logger().info(
                f'   s{i}(d{drone}): elev={dz:.0f} perr={perr:.2f} '
                f'|aff|={float(np.linalg.norm(a_ff)):.1f} affT={aff_t:.0f} '
                f'|acab|={float(np.linalg.norm(a_cable)):.2f}')


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
