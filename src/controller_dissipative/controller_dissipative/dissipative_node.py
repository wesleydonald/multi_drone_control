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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Empty, Bool, Float64MultiArray, String

from interfaces.msg import MotionCaptureState
from controller_load_mpc.geometry import quat_to_rot_np, attach_points
from controller_load_mpc.planner_node import LoadPlanner, DRONE_MASS, PLANNER_HZ

from controller_dissipative.dissipative_network import (
    DissipativeNetwork, DissipativeParams)


def _largest_gap_deg(rho):
    """Largest angular gap between consecutive attach points, degrees.

    The load's centre of mass lies strictly inside the attach-point polygon -- the
    condition for it to be able to hang level -- exactly when this is < 180 deg. See
    _detach_ocp. tools/metrics.cog_margin is the reporting version of the same test."""
    a = np.sort(np.mod([np.arctan2(float(r[1]), float(r[0])) for r in rho],
                       2 * np.pi))
    if a.size < 3:
        return 360.0
    return float(np.degrees(np.max(np.diff(np.r_[a, a[0] + 2 * np.pi]))))


class DissipativeController(LoadPlanner):
    def __init__(self):
        super().__init__()          # builds the OCP planner (creep -> OCP -> hover)

        # dissipative-network tuning (retunable per world without touching code).
        p = self.declare_parameter
        self.diss = DissipativeParams(
            k_pay=float(p('diss_k_pay', 40.0).value),
            k_anchor=float(p('diss_k_anchor', 40.0).value),
            k_ring=float(p('diss_k_ring', 20.0).value),
            k_slot=float(p('diss_k_slot', 18.0).value),
            c=float(p('diss_c', 6.0).value),
            node_mass=float(p('diss_node_mass', 0.5).value),
            substeps=int(p('diss_substeps', 10).value),
            elev_deg=float(p('diss_elev_deg', 45.0).value),
            # UNEQUAL (moment-balanced) force sharing -- see DissipativeNetwork. Enables
            # the fleet to hold the load LEVEL at an asymmetric attach set (so an off-centre
            # mid-flight newcomer can be a load-bearing RING member and the fleet visibly
            # reconfigures). Mutually exclusive in spirit with attach_central (a centre weld
            # still wants the pure vertical lifter). Off by default.
            balanced_tensions=bool(p('diss_balanced_tensions', False).value),
            # SOFT HAND-OUT time constant: a welded newcomer joins as a central lifter and is
            # continuously handed out to its off-centre ring slot over this many seconds, so
            # its actual cable force tracks the modelled feedforward and the feedforward-
            # trusting, no-integrator tracker never under-thrusts (the mid-flight attach
            # runaway fix). Only used when attach_handout is on. See DissipativeParams.
            T_handout=float(p('diss_t_handout', 12.0).value))
        # Horizon PREVIEW for the published network references. ON (default) carries the
        # load trajectory's own future displacement along the horizon, the way the OCP
        # path does (reference_builder.yref_at); OFF repeats the network's single target
        # across all N+1 nodes, which was the long-standing behaviour.
        #
        # ON by default on LOGGED evidence, not a model: with it off the payload trailed
        # its reference by 3.11 s (0.172 m mean error), with it on 1.11 s (0.081 m). The
        # lag is in the tracker stage -- drone reference vs drone actual -- exactly where
        # a frozen 2 s lookahead would put it.
        self._net_preview = bool(p('net_horizon_preview', True).value)
        # Lean the reference cone onto effective gravity g_eff = g - a_traj, so the
        # formation LEADS the load by the geometry the commanded acceleration needs
        # (DissipativeNetwork._lean). Without it the cone is symmetric about the
        # vertical through p_des, its net horizontal pull on the load is zero, and the
        # load can only accelerate by first falling behind -- logged as an 18.5% radius
        # deficit (cutting the corner) on a circle at traj_speed 0.4, r 0.5.
        #
        # Default OFF: UNVALIDATED. It is the same flatness relation the OCP planner
        # uses and it is theoretically right, but mini_plant shows only a 1.6% radius
        # deficit where Gazebo shows 18.5%, so the offline harness cannot reproduce the
        # symptom and therefore cannot confirm the cure. Fly it as an A/B against the
        # logs (net_traj_lean:=true) before making it the default.
        self._net_lean = bool(p('net_traj_lean', False).value)
        # ATTACH capacity: the network is provisioned with reserved_attach EXTRA nodes
        # beyond the n tethered drones the OCP flies. The reserved slots start off-network
        # (detach()-ed, hence inert) and are welded in mid-flight by attach(). The parent
        # OCP/creep stays at self.n (it never references the reserved nodes), so takeoff is
        # unchanged; reserved_attach=0 (the default) reproduces the pure-detach node exactly.
        self.reserved_attach = int(p('reserved_attach', 0).value)
        self.n_net = self.n + self.reserved_attach
        # per-node cable rest for a welded newcomer (a swung-electromagnet pendulum may hang
        # a different length than the tethers); defaults to the shared cable_len.
        self._attach_cable_len = float(p('attach_cable_len', self.cable_len).value)
        # How to fold a welded newcomer into the network:
        #   False (default) -> RING member: it swings out to a cone slot and the fleet
        #     reconfigures. Needs a FLEXIBLE (ball-jointed) weld so it doesn't lever the load.
        #   True -> CENTRAL lifter: it holds straight up over the load centre (no reconfigure),
        #     tilt-free even with a rigid weld. Fallback if the ring version is unstable.
        self._attach_central = bool(p('attach_central', False).value)
        # SOFT HAND-OUT: fold a welded RING newcomer in gradually (central lifter -> ring
        # member over diss_t_handout seconds) instead of instantly. The robust fix for the
        # mid-flight attach runaway -- every intermediate is near-equilibrium so the tracker
        # never under-thrusts. On by default; ignored for a central-lifter weld (attach_central
        # already adds pure vertical lift with no reconfiguration to ease).
        self._attach_handout = bool(p('attach_handout', True).value)
        # FEEDFORWARD RAMP: at the instant of weld the newcomer's tracker would otherwise jump
        # from armed-idle (no reference) to the FULL cable-tension feedforward in one tick
        # (gate 0->1), and because the tracker is a no-integrator, feedforward-trusting
        # controller that step -- applied while the just-welded magnet arm is still swinging --
        # becomes a thrust transient. Instead ramp the newcomer's cable-FF gate 0->1 over this
        # many seconds so the feedforward fades in (mirrors the tethers' measured-tautness gate).
        # <=0 restores the instant gate=1.
        self._attach_ff_ramp_s = float(p('attach_ff_ramp_s', 1.5).value)
        self._weld_time = {}                    # physical drone id -> weld Time (for the FF ramp)
        # SETTLE elevation for a welded newcomer (deg). Steeper than the fleet's diss_elev_deg
        # keeps it close to its high near-vertical weld pose (small out-and-down transit ->
        # avoids the transit-driven runaway); default = the fleet elevation (full 45deg rim).
        self._attach_elev_deg = float(p('attach_elev_deg', float(self.diss.elev_deg)).value)
        # How close (m, drone body to load centre) a reserved drone must be before its
        # tracker is warmed with a hold reference -- see _publish_pending_attach_refs.
        self._attach_warm_radius = float(p('attach_warm_radius', 1.0).value)
        net_rho = attach_points(self.n_net, self.attach_radius, self.attach_z)
        self.net = DissipativeNetwork(
            self.n_net, net_rho, self.cable_len, DRONE_MASS, self.load_mass,
            self.dyn.g, self.diss)
        for k in range(self.n, self.n_net):
            self.net.detach(k)                 # reserved slots start off the load
        self.detached = [False] * self.n_net   # per physical drone (departed after detach)
        # The carrying fleet at construction. self.n becomes the OCP's fleet size once
        # resize_fleet runs, and self.detached spans n_net (which counts reserved
        # not-yet-welded drones as 'not detached'), so neither is the right thing to
        # count survivors over.
        self._n_carry0 = self.n
        # per reserved drone j (physical id self.n + j): True until its magnet welds on,
        # while Tejen's approach controller owns it and we publish no reference for it.
        self.attach_pending = [True] * self.reserved_attach
        # measured pose of each reserved drone (its own mocap; kept out of drone_pos so the
        # parent's "all mocap present" takeoff guard is not blocked waiting on it).
        self.attach_pos = [None] * self.reserved_attach
        self._net_p_des = None                 # current network target (xy from traj, z hold)
        self._net_hold_z = None                # held/descending target height
        self._net_landing = False              # LAND received in the network phase
        self._net_land_z = float(p('net_land_z', 0.06).value)  # floor for the held target
        self._net_diag_ctr = 0

        # AUTO HANDOVER (dissipative-only flight). Normally the network is entered only by a
        # detach/attach event, so a run with neither never leaves the OCP -- the fleet flies
        # centralized start to finish. With this on, the node hands over as soon as the OCP
        # lift tops out and has settled, so the dissipative network alone holds the load and
        # flies the trajectory for the rest of the flight, with the full fleet and no member
        # ever leaving. Takeoff still belongs to the OCP: the pure network cannot break the
        # load off the ground from the shallow creep handover (see the module docstring), and
        # it is only verified stable from an airborne taut config.
        # Off by default so every existing launch keeps its exact detach/attach behaviour.
        self._auto_handover = bool(p('auto_network_handover', False).value)
        # Seconds to hold at the top of the OCP lift before handing over. The network seeds
        # from MEASURED drone positions, so handing over mid-climb seeds it with the climb
        # transient still in the config and the hold starts off-equilibrium.
        self._auto_handover_settle_s = float(p('auto_handover_settle_s', 1.5).value)
        self._auto_handover_t = None           # settle timer, None until the lift tops out

        # ── How a fleet-size change is handled (THESIS_PLAN §12.2) ───────────
        # 'network' (default, verified): the dissipative network takes the fleet on a
        #     detach/attach, redistributes, and carries the trajectory on itself.
        # 'ocp': the OCP is RESIZED in place to the new fleet size and keeps flying.
        #     No network phase, no phase machine -- the cables do not move when a
        #     drone leaves, so the surviving attach points are simply a smaller ring
        #     and the OCP can solve for it directly. Only expressible because
        #     attachment geometry is a runtime parameter (LoadCableDynamics).
        self._reconfig_mode = str(p('reconfig_mode', 'network').value).lower()
        if self._reconfig_mode not in ('network', 'ocp'):
            raise ValueError("reconfig_mode must be 'network' or 'ocp', got "
                             f'{self._reconfig_mode!r}')
        # Seconds to freeze the trajectory clock across a resize. A plain timer, not a
        # settle detector: the settle detector this replaced timed out on a fleet that
        # was already still, because it tested "load reached its target" against a
        # documented 5-7 cm steady sag it can never reach.
        self._reconfig_hold_s = float(p('reconfig_hold_s', 1.5).value)
        self._reconfig_hold_left = 0.0
        # Where each departed drone was parked, so it keeps getting a reference after
        # an OCP resize (the resized OCP plans only for the drones still on the load,
        # and a tracker with no reference trips ref_stale and disarms the fleet).
        self._departed_hold = {}
        if self._reconfig_mode == 'ocp':
            # Every size this flight could end up at. Built now because an acados
            # build is 9-40 s and cannot happen mid-flight; warm cache makes it ~0.1 s
            # (tools/prebuild_planner.py).
            self.prebuild_solvers(range(max(2, self.n - self.reserved_attach - 2),
                                        self.n + self.reserved_attach + 1))

        # Which generator is flying the fleet, announced for anyone downstream that
        # needs to change behaviour at the handover -- currently the trackers'
        # control_mode:=velocity_after_handover (docs/design/velocity_loop.md §8).
        # TRANSIENT_LOCAL: a tracker that comes up after the handover must still learn
        # it happened, and this fires exactly once per flight.
        self.phase_pub = self.create_publisher(
            String, '/fleet/control_phase',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.phase_pub.publish(String(data='planner'))

        # detach command (physical drone id): hands the fleet to the network + releases.
        self.create_subscription(Int32, '/fleet/detach', self._fleet_detach_cb, 10)
        # Gazebo DetachableJoint release triggers (bridged to gz.msgs.Empty in launch).
        self.detach_pub = [self.create_publisher(Empty, f'/drone_{i}/detach', 1)
                           for i in range(self.n)]

        # ATTACH wiring for each reserved drone (physical id self.n + j): its own mocap in,
        # its reference out (appended so ref_pub[d] is valid for d>=self.n), plus the two
        # attach triggers. The magnet weld itself is done by the collaborator's manager,
        # which announces it on /magnet/object_attached (Bool); /fleet/attach (Int32 drone
        # id) is the manual/offline-symmetry trigger mirroring /fleet/detach.
        for j in range(self.reserved_attach):
            d = self.n + j
            self.create_subscription(
                MotionCaptureState, f'/drone_{d}/motion_capture_state',
                lambda msg, k=j: self._attach_drone_cb(msg, k), 5)
            self.ref_pub.append(self.create_publisher(
                Float64MultiArray, f'/drone_{d}/reference_trajectory', 5))
        if self.reserved_attach > 0:
            self.create_subscription(Int32, '/fleet/attach', self._fleet_attach_cb, 10)
            self.create_subscription(Bool, '/magnet/object_attached',
                                     self._magnet_attached_cb, 10)
        # Re-dump params.json now that this subclass's parameters exist too (the parent
        # wrote it with only the OCP planner's set). Same directory, overwritten -- so
        # the file always describes the node that actually flew, including
        # net_horizon_preview / net_traj_lean / the diss_* tuning.
        self._dump_run_params()
        self.get_logger().info(
            f'[dissipative] OCP-takeoff + dissipative-detach ready '
            f'(n={self.n}, reserved_attach={self.reserved_attach}, '
            f'preview={self._net_preview}, lean={self._net_lean}); '
            f'params.json -> {self._run_log_dir}')

    # ── attach-drone mocap ───────────────────────────────────────────────────
    def _attach_drone_cb(self, msg: MotionCaptureState, j):
        p = msg.pose.position
        self.attach_pos[j] = np.array([p.x, p.y, p.z])

    # ── control tick: dispatch on phase (no parent modification) ─────────────
    def _plan(self):
        """The inherited timer calls this. In the network phase we run the spring-damper
        network; otherwise the inherited OCP planner flies (creep -> lift -> hover)."""
        if self._departed_hold:
            self._publish_departed_refs()
        self._publish_pending_attach_refs()
        if self.phase == 'network':
            self._network_plan()
            return
        # Freeze the trajectory clock across a resize, so a fleet that is still
        # settling into its new ring is not also asked to chase a moving target.
        if self._reconfig_hold_left > 0.0:
            self._reconfig_hold_left -= 1.0 / PLANNER_HZ
            self.traj_t -= 1.0 / PLANNER_HZ      # cancel the advance super()._plan() makes
            if self._reconfig_hold_left <= 0.0:
                self.get_logger().info(
                    f'[dissipative] reconfiguration hold over — trajectory resumes at '
                    f't={self.traj_t:.2f} s')
        if self._auto_handover_due():
            self.get_logger().info(
                '[dissipative] auto handover: OCP lift complete and settled - the '
                'dissipative network now flies the full fleet (no detach involved)')
            self._enter_network_phase()
            self._network_plan()
            return
        super()._plan()

    def _publish_departed_refs(self):
        """Keep every drone that has left the fleet on a reference of its own.

        Mirrors the contract of the network's `_detached_reference`: hold position
        while the fleet flies, and DESCEND WITH THE FLEET ON LAND so the freed drones
        come down too. That LAND behaviour is verified and was briefly lost when an
        earlier version parked them at a fixed pose forever -- the fleet landed, the
        detached drone stayed up, and the tilt envelope aborted the run.

        Not optional either way: the resized OCP plans only for the drones still on
        the load, so without this a departed drone gets no reference at all, trips
        `ref_stale` and disarms the whole fleet."""
        g = np.array([0.0, 0.0, self.dyn.g])
        zero = np.zeros(3)
        for d, p_hold in self._departed_hold.items():
            if p_hold is None:
                continue
            if self._land_to_ground:
                # Follow the fleet down at the same rate the load is descending.
                p_hold[2] = max(0.0, float(p_hold[2]) - self.land_vel / PLANNER_HZ)
            self._publish_ref(d, [(p_hold, zero, g, zero)] * (self.N + 1))

    def _publish_pending_attach_refs(self):
        """Keep a reserved drone's tracker WARM while the approach controller still flies it.

        elrs_mux hands authority to our tracker on the first /magnet/object_attached, but
        `attach_pending` clears on that same message and the network only publishes on its
        next 10 Hz tick. A tracker with no reference publishes channel_2 = -1.0 -- motors
        OFF -- so the mux switched a flying drone onto a stream commanding zero throttle.
        Measured in Gazebo (R0031-R0046, 12/12 runs): throttle 0.36 -> 0.000 within 0.03 s
        of the weld, ~0.2 s of dead motors, tilt 6 -> 89 deg, ENVELOPE FAULT, fleet abort.

        A hold at the drone's MEASURED position is the bumpless choice: it costs the
        approach controller nothing (the mux drops this stream until the weld), it is what
        net.attach() seeds the newcomer's network reference with, and it means the first
        command after the switch is `stay exactly where you are`.

        Gated on PROXIMITY to the load, not on the whole approach. A tracker with a
        reference runs a full acados solve every cycle, and warming one from ARM added a
        fourth solver for ~35 s of approach: pose-timeout disarms went from 0 in 17 runs
        (2026-08-05) to 6 in 5 runs, every one of them ~1 s after ARM. The tracker only has
        to be live when the mux switches, and the weld needs 0.15 m proximity, so a 1 m
        gate leaves seconds of warm-up and costs a fraction of the solving."""
        if self.load_state is None:
            return
        load_p = np.asarray(self.load_state[0:3], float)
        g = np.array([0.0, 0.0, self.dyn.g])
        zero = np.zeros(3)
        for j in range(self.reserved_attach):
            if not self.attach_pending[j] or self.attach_pos[j] is None:
                continue
            p_hold = np.asarray(self.attach_pos[j], float).copy()
            if float(np.linalg.norm(p_hold - load_p)) > self._attach_warm_radius:
                continue
            self._publish_ref(self.n + j, [(p_hold, zero, g, zero)] * (self.N + 1))

    def _auto_handover_due(self):
        """True on the tick the OCP lift has topped out and held for the settle time.

        Deliberately conservative: it waits for a genuine steady hover, and it never fires
        during a descent/landing, because the network seeds from measured positions and a
        seed taken mid-transient starts the hold off-equilibrium."""
        if not self._auto_handover or self.phase != 'planner':
            return False
        if self.load_state is None or self.lift_z0 is None:
            return False
        if self.descending or self._landed or self._net_landing:
            return False
        if any(self.drone_pos[d] is None for d in range(self.n)):
            return False
        if self.lift_progress < (self.target_z - self.lift_z0) - 1e-6:
            self._auto_handover_t = None       # still climbing; restart the settle timer
            return False
        if self._auto_handover_t is None:
            self._auto_handover_t = 0.0
        self._auto_handover_t += 1.0 / PLANNER_HZ
        return self._auto_handover_t >= self._auto_handover_settle_s

    # ── detach: hand over to the dissipative network, then drop a drone ──────
    def _fleet_detach_cb(self, msg: Int32):
        d = int(msg.data)
        if not (0 <= d < len(self.detached)):
            self.get_logger().warn(f'[dissipative] /fleet/detach {d} out of range')
            return
        if self.phase not in ('planner', 'network'):
            self.get_logger().warn('[dissipative] detach ignored - not flying yet')
            return
        if self.detached[d]:
            return
        if self._reconfig_mode == 'ocp':
            self._detach_ocp(d)
            return
        # first detach also performs the OCP -> network handover (seeds bumplessly).
        if self.phase == 'planner':
            self._enter_network_phase()
        slot = self.slot2drone.index(d)          # network slot of physical drone d
        self.net.detach(slot)
        self.detached[d] = True
        self.detach_pub[d].publish(Empty())
        self.get_logger().info(
            f'[dissipative] DETACH drone {d} (slot {slot}); '
            f'{self.net.n_attached()} drones remain on the load')

    def _detach_ocp(self, d):
        """Detach WITHOUT handing the fleet to the network: resize the OCP in place.

        The cables do not move when a drone leaves -- they stay bolted where they were
        -- so the survivors are simply a smaller, generally UNEVEN ring, and the OCP
        can solve for that directly now that attachment geometry is a runtime
        parameter. No network phase, no phase machine, no settle detection.

        Whether the load can still hang LEVEL afterwards is geometry, not control: the
        load's centre of mass must stay inside the polygon of surviving attach points.
        Removing one cable from a symmetric n-ring leaves a largest angular gap of
        4*pi/n, so that holds only for n >= 5; n=4 sits exactly on the boundary. The
        resize is allowed either way and the margin is logged, because flying a
        marginal configuration is a result worth having, not an error."""
        survivors = [x for x in range(self._n_carry0)
                     if x != d and not self.detached[x]]
        if len(survivors) < 2:
            self.get_logger().error(
                f'[dissipative] refusing to detach {d}: {len(survivors)} would remain')
            return
        # Each survivor keeps its OWN attach point. Captured BEFORE the resize, while
        # self.rho and self.slot2drone still describe the current fleet.
        rho_of = {self.slot2drone[sl]: self.rho[sl]
                  for sl in range(len(self.slot2drone))}
        rho_new = [rho_of[x] for x in survivors]
        gap = _largest_gap_deg(rho_new)
        if not self.resize_fleet(len(survivors), survivors, rho=rho_new):
            self.get_logger().error(
                f'[dissipative] detach {d} ABORTED: no OCP for n={len(survivors)}')
            return
        # Warm-start the resized solver before it flies the fleet. Leaving last_X None
        # makes the first live solve a cold reconverge, which _prime_solver's own
        # docstring records as "a jump and a scrambled MPC path".
        for _ in range(2):
            self._prime_solver()
        if self.solver.last_X is None:
            self.get_logger().error(
                f'[dissipative] detach {d}: the n={self.n} OCP did not converge on the '
                f'current state. The fleet is resized but the first solves may jump.')
        self.detached[d] = True
        self.detach_pub[d].publish(Empty())
        self._departed_hold[d] = np.asarray(self.drone_pos[d], float).copy() \
            if self.drone_pos[d] is not None else None
        self._reconfig_hold_left = self._reconfig_hold_s
        self.get_logger().warn(
            f'[dissipative] DETACH drone {d} (OCP resize): n={self.n}, '
            f'trajectory held {self._reconfig_hold_s:.1f} s; surviving ring spans a '
            f'{gap:.0f} deg gap ({"CoG inside" if gap < 180.0 - 1e-6 else "CoG ON/OUTSIDE the hull — the load cannot hang level"})')

    # ── attach: weld a reserved drone in and fold it into the network ─────────
    def _fleet_attach_cb(self, msg: Int32):
        """Manual/offline-symmetry trigger: attach physical drone id msg.data."""
        self._do_attach(int(msg.data))

    def _magnet_attached_cb(self, msg: Bool):
        """The collaborator's manager announces a completed magnet weld here. On True,
        fold every still-pending reserved drone into the network (usually just one)."""
        if not msg.data:
            return
        for j in range(self.reserved_attach):
            if self.attach_pending[j]:
                self._do_attach(self.n + j)

    def _do_attach(self, d):
        """Add physical drone d to the dissipative network (mirror of _fleet_detach_cb).
        The first attach also performs the OCP -> network handover, so an attach works
        from a plain OCP hover as well as after an earlier detach."""
        if not (0 <= d < self.n_net):
            self.get_logger().warn(f'[dissipative] /fleet/attach {d} out of range')
            return
        if self.phase not in ('planner', 'network'):
            self.get_logger().warn('[dissipative] attach ignored - not flying yet')
            return
        if self.phase == 'planner':
            self._enter_network_phase()
        slot = self._drone_to_net_slot(d)
        if slot is None:
            self.get_logger().warn(f'[dissipative] attach: drone {d} has no network slot')
            return
        if self.net.attached[slot]:
            return                                # already on the load
        measured = self._net_pos(slot)
        if measured is None:
            self.get_logger().warn(f'[dissipative] attach drone {d}: no mocap yet')
            return
        # UNEQUAL force sharing: capture where the newcomer welded into rho[slot] so the
        # wrench-balance solve models the true asymmetric geometry. CAUTION: `measured` is
        # the DRONE BODY, which on a swung magnet arm can sit ~cable_len up-and-out from the
        # actual tip weld -- using it raw hands the solver a phantom moment arm that levers
        # the real (centre-welded) load over. So the horizontal offset is CLAMPED to the
        # attach ring radius: a genuinely off-centre weld is captured; a leaning drone over a
        # centre weld can no longer inject a large false lever. (A centre weld still cannot be
        # a load-bearing ring member at all -- use attach_central for that.)
        if self.diss.balanced_tensions and not self._attach_central \
                and self.load_state is not None:
            load_pos = np.asarray(self.load_state[0:3], float)
            R = quat_to_rot_np(self.load_state[3:7])
            rho_body = R.T @ (np.asarray(measured, float) - load_pos)
            r_xy = float(np.hypot(rho_body[0], rho_body[1]))
            if r_xy > self.attach_radius and r_xy > 1e-6:      # clamp the phantom lever
                rho_body[:2] *= self.attach_radius / r_xy
            self.net.set_attach_rho(slot, [float(rho_body[0]), float(rho_body[1]),
                                           self.attach_z])
            self.get_logger().info(
                f'[dissipative] weld offset captured for slot {slot}: '
                f'rho=({rho_body[0]:+.3f},{rho_body[1]:+.3f}) m (load frame, '
                f'clamped to r<={self.attach_radius:.2f})')
        # Fold the newcomer in as a ring member (reconfigures) or a central lifter (tilt-free
        # fallback), per the attach_central param. Ring mode relies on the flexible ball-jointed
        # weld so the drone hangs like a tether instead of levering the load.
        self.net.attach(slot, measured, cable_len_k=self._attach_cable_len,
                        central=self._attach_central, handout=self._attach_handout,
                        elev_deg_k=self._attach_elev_deg)
        if d >= self.n:
            self.attach_pending[d - self.n] = False
        self._weld_time[d] = self.get_clock().now()   # start the FF gate ramp for this newcomer
        self.detached[d] = False
        mode = ('central lifter' if self._attach_central
                else f'soft hand-out ({self.diss.T_handout:.0f}s)' if self._attach_handout
                else 'instant ring')
        self.get_logger().info(
            f'[dissipative] ATTACH drone {d} (slot {slot}) as {mode}; '
            f'{self.net.n_attached()} drones now on the load')

    def _net_slot2drone(self):
        """Network slot -> physical drone for ALL n_net nodes: the parent's (possibly
        azimuth-reassigned) tethered mapping, then identity for the reserved drones."""
        return list(self.slot2drone) + list(range(self.n, self.n_net))

    def _drone_to_net_slot(self, d):
        s2d = self._net_slot2drone()
        return s2d.index(d) if d in s2d else None

    def _net_pos(self, slot):
        """Measured position of the drone in network slot `slot`: drone_pos for a tethered
        slot, attach_pos for a reserved one. None if that mocap has not arrived yet."""
        d = self._net_slot2drone()[slot]
        return self.drone_pos[d] if d < self.n else self.attach_pos[d - self.n]

    def _enter_network_phase(self):
        """planner -> network: seed the spring-damper network from the current (airborne,
        taut) measured drone positions so the handover is bumpless, and capture the hover
        target the network will hold. The OCP is no longer solved after this."""
        self.phase = 'network'
        # seed all n_net nodes bumplessly. Reserved (not-yet-welded) nodes are inert, so a
        # finite placeholder (their mocap if present, else the load position) is enough --
        # attach() overwrites it with the measured pose at the weld.
        load_p = self.load_state[0:3] if self.load_state is not None else np.zeros(3)
        seeds = []
        for i in range(self.n_net):
            p = self._net_pos(i)
            seeds.append(load_p if p is None else p)
        self.net.seed(seeds)
        self._net_p_des = self._current_load_des()
        self._net_hold_z = float(self._net_p_des[2])   # hold this height; traj_t keeps running
        self.phase_pub.publish(String(data='network'))
        self.get_logger().info(
            '[dissipative] handover OCP -> dissipative network (airborne)')

    def _fleet_command_cb(self, msg):
        """In the network phase the inherited OCP LAND path is inert (its _plan is
        bypassed), so LAND is handled here: descend the held load target to the floor
        (see _network_plan) and announce /fleet/landed for the fleet manager to disarm.
        Every other command, and LAND before handover, falls through to the base handler."""
        if self.phase == 'network' and msg.data.strip().upper() == 'LAND':
            if self._landed:
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[dissipative] LAND repeated after touchdown - re-announcing /fleet/landed')
            elif not self._net_landing:
                self._net_landing = True
                self.get_logger().info(
                    '[dissipative] LAND (network) - descending the held load to the floor')
            else:
                self.get_logger().info('[dissipative] LAND (network) already in progress')
            return
        super()._fleet_command_cb(msg)

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

        # Keep any LATERAL trajectory running after handover: advance the trajectory clock
        # and take the target xy from it (frozen only during LAND). Without this the load
        # trajectory would stop the instant a drone detached.
        if not self._net_landing:
            self.traj_t += 1.0 / PLANNER_HZ
        dx, dy, _, _ = self.traj.offset_at(self.traj_t)
        # Commanded lateral acceleration of the load target -- the flatness term the
        # cone leans onto. Zero while hovering or landing. See net_traj_lean.
        def _a_at(tq):
            if self._net_landing or not self._net_lean:
                return None
            ax, ay = self.traj.accel_at(tq)
            a = np.array([float(ax), float(ay), 0.0])
            return a if float(np.linalg.norm(a)) > 1e-9 else None
        a_des = _a_at(self.traj_t)

        # z: hold the handover height, or ramp down to the floor on LAND. Touchdown (both
        # target and measured load near the floor) announces /fleet/landed to disarm.
        if self._net_landing:
            self._net_hold_z = max(self._net_land_z,
                                   self._net_hold_z - self.land_vel / PLANNER_HZ)
            if not self._landed and self._net_hold_z <= self._net_land_z + 1e-3 \
                    and float(self.load_state[2]) <= self._net_land_z + 0.10:
                self._landed = True
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[dissipative] load down - announcing /fleet/landed (fleet will disarm)')
        p_des = np.array([self.hover_xy[0] + dx, self.hover_xy[1] + dy, self._net_hold_z])
        self._net_p_des = p_des
        # Vertical rate the network is actually commanding, so the RViz horizon slopes
        # the right way. The inherited _lift_vel froze at whatever the OCP lift ramp
        # last set, which is stale from handover onward.
        self._lift_vel = -self.land_vel if self._net_landing else 0.0
        # Publish the desired/RViz topics with the target the NETWORK is actually
        # commanding. The inherited version recomputes z from the OCP lift schedule
        # (lift_z0 + lift_progress), which stops advancing at handover and so ignores
        # _net_hold_z entirely -- during a network-phase LAND it reported the load
        # sitting at hover height all the way to touchdown. Called here, after p_des
        # exists, rather than at the top of the tick.
        self._publish_load_desired(target=p_des)

        load_pos = self.load_state[0:3]
        load_quat = self.load_state[3:7]
        load_vel = self.load_state[7:10]
        self.net.step(load_pos, load_quat, load_vel, p_des, 1.0 / PLANNER_HZ,
                      load_accel=a_des)
        net_s2d = self._net_slot2drone()

        # HORIZON: roll the network forward along the load target's own future so every
        # node carries its own p/v/a_ff/a_cable, instead of repeating node 0. The repeat
        # is exact only while the formation's geometry relative to the load is constant;
        # on a curved path it is not (the nodes trail the moving cone and that trailing
        # direction rotates with the velocity), so a_cable swings by 68% of its magnitude
        # across one 2 s horizon at traj_speed 0.6 and the drone's offset from the load
        # drifts 0.35 m. With a constant target -- hover, LAND -- every node comes out
        # identical and this is exactly the old repeat.
        # One gate per NETWORK slot (n_net), not per tethered drone (n). The network is
        # built with n_net nodes, so horizon_references indexes gates[i] for every slot;
        # sizing this list to self.n made a ring attach raise IndexError on the first
        # plan tick after the weld, killing the whole planner process -- every drone lost
        # its reference, the newcomer went armed-idle and dropped onto the load, and the
        # payload capsized into an envelope fault. The non-preview path below already
        # guarded this with `if i < len(gates)`; the preview path (on by default) did not.
        gates = [self._attach_ff_gate(net_s2d[i]) if net_s2d[i] >= self.n
                 else self._cable_taut_gate(i)[0] for i in range(self.n_net)]
        if self._net_preview and not self._net_landing:
            p_seq, a_seq = [], []
            for k in range(self.N + 1):
                kx, ky, _, _ = self.traj.offset_at(self.traj_t + self.dt * k)
                p_seq.append(np.array([self.hover_xy[0] + kx,
                                       self.hover_xy[1] + ky, self._net_hold_z]))
                a_seq.append(_a_at(self.traj_t + self.dt * k))
            horizon = self.net.horizon_references(
                load_quat, p_seq, 1.0 / PLANNER_HZ, taut_gates=gates,
                load_accel_seq=a_seq)
        else:
            horizon = None
        for i in range(self.n_net):
            drone = net_s2d[i]
            # a reserved drone that has not yet welded on is owned by the collaborator's
            # approach controller -- publish nothing for it (the motor mux forwards their
            # stream until /magnet/object_attached flips us in).
            if drone >= self.n and self.attach_pending[drone - self.n]:
                continue
            if self.detached[drone]:
                node = self._detached_reference(i)
                self._publish_ref(drone, [node] * (self.N + 1))
                continue
            # tethered nodes gate the cable FF on the measured rod tautness; a welded
            # newcomer is rigidly attached (taut), but its FF is RAMPED 0->1 over
            # attach_ff_ramp_s from the weld so the no-integrator tracker is not slammed.
            if horizon is not None and i < len(horizon):
                self._publish_ref(drone, horizon[i])
            else:
                node = self.net.reference(i, load_quat, p_des,
                                          taut_gate=gates[i] if i < len(gates) else 1.0,
                                          load_accel=a_des)
                self._publish_ref(drone, [node] * (self.N + 1))
        self._net_diag(load_pos, load_quat, p_des)

    def _detached_reference(self, i):
        """Reference for a DETACHED drone: normally fly up and hold (fly_away_reference); on
        LAND, descend to the floor at its frozen xy so the freed drones come down with the
        fleet instead of hovering."""
        p_ref, v_ref, a_ff, a_cable = self.net.fly_away_reference(i)
        if self._net_landing:
            q = self.net.q[i]
            p_ref = np.array([q[0], q[1], self._net_hold_z])
            v_ref = np.zeros(3)
        return p_ref, v_ref, a_ff, a_cable

    def _attach_ff_gate(self, drone):
        """Cable-feedforward gate for a freshly welded newcomer: ramps 0->1 over
        attach_ff_ramp_s from the weld instant so the tracker's cable-tension feedforward
        fades in instead of stepping (the attach thrust-transient fix). Returns 1.0 once the
        ramp is done, if ramping is disabled (ramp_s<=0), or if the weld time is unknown."""
        t0 = self._weld_time.get(drone)
        if t0 is None or self._attach_ff_ramp_s <= 0.0:
            return 1.0
        elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
        return float(np.clip(elapsed / self._attach_ff_ramp_s, 0.0, 1.0))

    def _net_diag(self, load_pos, load_quat, p_des):
        self._net_diag_ctr += 1
        if self._net_diag_ctr % int(max(PLANNER_HZ, 1)) != 0:
            return
        R = quat_to_rot_np(load_quat)
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
        self.get_logger().info(
            f'[diss-net] load_z={load_pos[2]:.3f} z_tgt={p_des[2]:.2f} '
            f'tilt={tilt:.1f}deg n_att={self.net.n_attached()}')
        # Loop ALL network slots (n_net), so the welded newcomer shows up too. r_ref = horizontal
        # radius of the slot's reference target from the load centre: a center-welded drone that
        # the ring model pulls outward will show a large r_ref it cannot physically reach.
        net_s2d = self._net_slot2drone()
        for i in range(self.n_net):
            drone = net_s2d[i]
            if self.detached[drone]:
                continue
            if drone >= self.n and self.attach_pending[drone - self.n]:
                continue
            pos = self._net_pos(i)
            if pos is None:
                continue
            gate = 1.0 if drone >= self.n else self._cable_taut_gate(i)[0]
            p_ref, _, a_ff, a_cable = self.net.reference(
                i, load_quat, p_des, taut_gate=gate)
            perr = float(np.linalg.norm(p_ref - pos))
            r_ref = float(np.linalg.norm((p_ref - load_pos)[:2]))   # outward ring radius
            dz_ref = float(p_ref[2] - load_pos[2])                  # height above load
            self.get_logger().info(
                f'   s{i}(d{drone}): perr={perr:.2f} r_ref={r_ref:.2f} dz_ref={dz_ref:+.2f} '
                f'|aff|={float(np.linalg.norm(a_ff)):.1f} '
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
