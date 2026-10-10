"""
dissipative_node.py
--------------------
OCP takeoff + DISSIPATIVE-network hold/detach for a cable-suspended load.

The pure decentralized network cannot break the load off the ground from the shallow
(~23 deg) creep handover -- the per-drone cable-aware MPC tracker stalls into a hover
rather than executing the analytic lift, the same wall that makes an open-loop analytic
takeoff unstable. So takeoff is delegated to the PROVEN centralized OCP planner
(mpc_planner.LoadPlanner): this node SUBCLASSES it and reuses the entire
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
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Empty, Bool, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from dissipative_planner.approach import (ApproachProfile, TIP_FROZEN_N, TIP_STANDOFF_M, TipLoop, body_to_tip,
                                          carrot_step, tip_valid)

from interfaces.msg import MotionCaptureState
from mpc_planner.geometry import (quat_to_rot_np, attach_points, rot_z,
                                  apex_direction, balanced_tensions)
from mpc_planner.planner_node import LoadPlanner, DRONE_MASS, PLANNER_HZ, TouchdownDetector
from mpc_planner.reference_builder import unload_tensions

from dissipative_planner.dissipative_network import (
    DissipativeNetwork, DissipativeParams)


# a freed drone lands at least this far from the ring centre (ring 0.28 + cable 0.5 + margin)
DEPARTED_CLEAR_R = 1.2
DETACH_STEP_VEL = 0.15     # m/s, a freed drone's step out from the ring (detach_step_out_m)
# during a circle the formation sweeps a disc of radius r_orbit + 0.5 (+ props): a freed drone then
# also steps out from the PATH centre to r_orbit + this before it holds or lands
DEPARTED_PATH_CLEAR_M = 1.0
NEWCOMER_RELEASE_S = 0.5   # s an aborted newcomer holds after its magnet OFF (rig release < 0.2 s)
NEWCOMER_OFF_EVERY_S = 0.5  # s between the OFFs re-sent while a newcomer is out (a repeated ON must not weld it)
NEWCOMER_POSE_STALE_S = 0.5  # s: an older newcomer pose is not used (a frozen pose reads as a stall)
NEWCOMER_LIFTED_M = 0.05   # m above its height at ATTACH before an aborted newcomer counts as off the floor
DEPARTED_PRESS_M = 0.35    # m: a landing departed drone's reference goes this far below its rest, and never
                           # further below the drone itself (its rod and magnet on the floor make it light:
                           # it hovered 12 cm (twin R1252) and 0.19-0.22 m (rig r305, near the floor) above a
                           # floor reference)
DEPARTED_REST_TOL_M = 0.03  # m: back at its resting height (the tracker's GROUND_IDLE_MARGIN)
DEPARTED_AT_REST_S = 0.5   # s at its rest before its reference stops pressing (its tracker idles after 0.2 s)
DEPARTED_PRESS_LIMIT_S = 6.0  # s of press after it stopped above its rest; then held at its own height
DETECT_STALE_S = 0.1       # s, a ring or drone pose older than this is not used to detect a release
DROP_LAND_WAIT_S = 30.0    # s after a drop: /fleet/landed even if a drone never reads as down
# partner_contact (card 2026-10-10_combined_m1_contact): his phases from the rejoin on, in which our ring magnet span
# may begin (tejen_mission MissionPhase; LIFT_PROOF and RING_ATTACHED are the contact attach's own)
REJOIN_PHASES = ('TRANSIT_TO_REATTACH', 'APPROACH_ABOVE_TARGET', 'MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT',
                 'ATTACH_READY', 'LIFT_PROOF', 'RING_ATTACHED')
NOT_FLOWN_PHASES = ('', 'WAIT_FOR_TAKEOFF', 'LANDED_DISARMED')
CONTACT_SPAN_MIN_Z = 0.3   # m: QUAD4 above this before the span's ON (no ON on the floor, near the spawn weld)
CONTACT_SPAN_EVERY_S = 1.0  # s between the span's ON keepalives
RELEASE_SEEN_M = 0.05      # m the welded start's tip moves off the plate (ring frame) before its release counts as seen


def _ring_off_allowed(node):
    """partner_contact: the newcomer's ring magnet OFF only once our ON span has begun. On the rig one coil holds his
    ball before it, so an OFF there drops the ball over the ring (card 2026-10-10_combined_m1_contact, critic 6)."""
    return not getattr(node, '_partner_contact', False) or getattr(node, '_span_t', None) is not None


def _free_drone_ff(node, d, g):
    """(thrust feedforward, external pull) of drone d flying off the ring: its own weight, plus his ball's while
    QUAD4 carries it under partner_contact (/tejen/object_attached)."""
    j = d - getattr(node, '_n_carry0', d + 1)
    if not getattr(node, '_ball_on', False) or not 0 <= j < getattr(node, 'reserved_attach', 0):
        return g, np.zeros(3)
    b = node._ball_mass * float(g[2]) / node.drone_mass
    return g + np.array([0.0, 0.0, b]), np.array([0.0, 0.0, -b])


def _who(d):
    """A drone in operator lines: its label (1-4, as RViz, the fleet manager and the airframes)
    and its topic index."""
    return f'drone {d + 1} (/drone_{d})'


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
        super().__init__('dissipative_planner')   # builds the OCP planner (creep -> OCP -> hover)

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
            # COMMON-MODE LOAD TRIM (docs/design/velocity_loop.md §11): one z-only
            # integrator on the measured load error, added identically to every node's
            # a_ff, trimming the fixed-kT steady sag without the per-drone differential
            # bias that tilts the load. 0.0 = off = every verified config byte-unchanged.
            # SYMMETRIC HAND-OUT: ease the incumbents' tension solution with the
            # newcomer's hand-out scalar (see DissipativeParams). Balanced+hand-out only.
            handout_tension_blend=bool(p('diss_handout_tension_blend', True).value),
            # EXPERIMENT: wrench solve at the load's true attitude (see DissipativeParams)
            wrench_true_attitude=bool(p('diss_wrench_true_attitude', False).value),
            ki_load=float(p('diss_ki_load', 0.0).value),
            a_i_load_max=float(p('diss_a_i_load_max', 2.0).value),
            trim_share_weighted=bool(p('diss_trim_share_weighted', False).value),
            i_load_xyz=bool(p('diss_i_load_xyz', False).value),
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
        # the newcomer's rod hangs from a pivot below its body (pivot_offset, rig/twin -0.04 m), so
        # the tip is rod + pivot drop below the body the tracker flies; the OCP keeps the rod itself
        self._attach_body_to_tip = body_to_tip(self._attach_cable_len, self.pivot_offset[2])
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
        self._ocp_attached = {}                 # physical drone id -> True once an OCP-resize attach
        # SETTLE elevation for a welded newcomer (deg). Steeper than the fleet's diss_elev_deg
        # keeps it close to its high near-vertical weld pose (small out-and-down transit ->
        # avoids the transit-driven runaway); default = the fleet elevation (full 45deg rim).
        self._attach_elev_deg = float(p('attach_elev_deg', float(self.diss.elev_deg)).value)
        # How close (m, drone body to load centre) a reserved drone must be before its
        # tracker is warmed with a hold reference -- see _publish_pending_attach_refs.
        self._attach_warm_radius = float(p('attach_warm_radius', 1.0).value)
        # TRACKER-FLOWN APPROACH (2026-09-25): with no approach MPC in the loop, this
        # node flies the reserved drone to the live magnet-tip target on its own tracker
        # (see approach.py) from the ATTACH command until the weld.
        self._attach_approach = bool(p('attach_approach', False).value)
        # the newcomer's tracker runs the PD velocity loop (no cable model): fold the
        # OCP's cable term into its thrust feedforward, as the network does for every
        # velocity tracker (a_ff = g + a_des - a_cable)
        self._attach_velocity_newcomer = bool(p('attach_velocity_newcomer', False).value)
        self._attach_target = None
        self._approach = {}                     # reserved index j -> ApproachProfile
        # always: a partner handoff enables the approach mid-flight
        self._attach_target_vel = np.zeros(3)
        self.create_subscription(TwistStamped, '/attach_target/twist', self._attach_twist_cb, 10)
        self.create_subscription(PoseStamped, '/attach_target/pose',
                                 self._attach_target_cb, 5)
        # TRAJECTORY HOLD ON ATTACH (s): freeze the load target for this long after a weld
        # so the fleet reconfigures in place instead of chasing a moving target while the
        # newcomer hands out; the trajectory then resumes with the full fleet. 0 = off.
        self._attach_traj_hold_s = float(p('attach_traj_hold_s', 0.0).value)
        self._attach_seek_m = float(p('attach_seek_m', 0.0).value)
        # the approach corrects its body goal from the measured magnet tip (card 2026-10-09_approach_tip_feedback;
        # Wesley 9 Oct); approach_tip_log computes and logs the same hold and error with no correction (the
        # switch-off arms); attach_tip_dz moves the tip setpoint (the calibrated point; 0 = today's)
        self._tip_feedback = bool(p('approach_tip_feedback', False).value)
        self._tip_log = self._tip_feedback or bool(p('approach_tip_log', False).value)
        self._tip_dz = float(p('attach_tip_dz', 0.0).value)
        self._tip_frames = deque(maxlen=TIP_FROZEN_N + 2)
        self._tip_csv = None
        if self._tip_log:
            self.create_subscription(PoseStamped, '/magnet_tip_pose', self._tip_pose_cb, 10)
        self._attach_t_start_new = float(p('attach_t_start_new', 0.1).value)
        self._attach_blend_balanced = bool(p('attach_blend_balanced', True).value)
        self._attach_datum_shift = bool(p('attach_datum_shift', True).value)
        # opt-in: the trajectory keeps running through the rejoin (Tejen's request): no approach
        # hold, only the plain reconfiguration hold at the weld, the approach tracks the moving
        # plate with its velocity fed forward; the tension blend still spans attach_traj_hold_s
        self._attach_moving = bool(p('attach_moving', True).value)
        self._attach_approach_direct = bool(p('attach_approach_direct', True).value)
        # 'timed': the hold above. 'settle': the post-weld hold ends once the load tilt
        # has settled (below hold_resume_tilt_deg and quiet for hold_settle_s), capped at
        # hold_max_s -- a measured dwell instead of a tuned one (tools/hybrid_dwell.py:
        # the 10 s hold ended with 26-48 % of the weld's tilt excess still to decay).
        self._hold_mode = str(p('attach_traj_hold_mode', 'timed').value).lower()
        self._hold_resume_tilt = float(p('hold_resume_tilt_deg', 12.0).value)
        self._hold_resume_rate = float(p('hold_resume_rate_dps', 5.0).value)
        self._hold_settle_s = float(p('hold_settle_s', 1.5).value)
        self._hold_max_s = float(p('hold_max_s', 30.0).value)
        self._settle_hold = False           # a settle-mode post-weld hold is running
        self._settle_for = 0.0
        self._settle_elapsed = 0.0
        self._settle_prev_tilt = None
        # BUMPLESS TENSION HANDOVER (s). At OCP -> network the incumbents' cable
        # feedforward steps from the OCP's actual solution to the network's (which also
        # models the tilted three-drone hover as level); measured at the weld: the 12
        # o'clock drone dropped 17 cm in 0.5 s and the load tilted 25 -> 47 deg (R0203).
        # Blend each incumbent's a_cable from the last OCP value to the network's over
        # this long. 0 = off (the verified detach paths are unchanged).
        self._handover_blend_s = float(p('handover_blend_s', 0.0).value)
        self._last_ocp_ac = {}
        self._ho_ac = {}
        self._ho_t0 = None
        # operator status line for RViz (fleet_viz renders it above the payload)
        self.status_pub = self.create_publisher(String, '/fleet/status', 1)
        self.create_timer(0.5, self._publish_status)
        # The hold starts when the APPROACH starts (the operator's ATTACH = /magnet/command
        # ON), not at the weld: a payload that keeps moving during the descent makes the
        # tip weld wherever it happens to be inside the trigger radius (R0178 welded at
        # 11 o'clock, r=0.13, instead of the 6 o'clock rim). It is re-armed at the weld so
        # the reconfiguration also happens on a still target, then the trajectory resumes.
        # partner mode: the partner's mission also commands the magnet for its object pickup,
        # and it rejoins a MOVING ring, so no approach hold (the weld hold is kept)
        self._approach_hold_on = self._attach_traj_hold_s > 0.0 and bool(p('approach_hold', True).value)
        self.create_subscription(String, '/magnet/command', self._magnet_cmd_cb, 10)
        # detach of a magnet-attached newcomer releases the weld through the magnet manager
        self._magnet_cmd_pub = self.create_publisher(String, '/magnet/command', 1)
        # partner handover: the partner's mission hovers the newcomer ~0.18 m above the
        # plate (its ATTACH_READY is a handover point, R0623); on its handoff flag our
        # tracker flies the last descent (the validated approach, trajectory held) and
        # this node owns the ring magnet (ON, re-sent until the weld)
        self._partner_handed = False
        # partner demo start (Wesley 2026-09-26): the magnet drone starts WELDED as the
        # fourth carrier; after its detach our tracker steps it out and up, then it is
        # released to the partner's mission (/partner/release) and becomes a pending
        # rejoiner again (the handoff/weld below then work unchanged)
        self._start_attached = bool(p('start_attached', False).value)
        # M2 hand-over: the partner's fleet is already flying on the grounded ring
        self.creep.airborne_start = bool(p('airborne_start', False).value)
        self._partner_release = bool(p('partner_release', False).value)
        self._release_out = float(p('partner_release_out_m', 0.5).value)
        self._release_up = float(p('partner_release_up_m', 0.3).value)
        self._release_state = {}
        self._release_pub = self.create_publisher(String, '/partner/release', 1)
        # > 0: a detached drone steps this far radially out from the ring, then holds there,
        # clear of the survivors (Wesley 7 Oct); 0 = hold where it let go
        self._detach_step_out = float(p('detach_step_out_m', 0.0).value)
        self._departed_step = {}
        self._departed_next = {}     # d -> the path-clear waypoint after the ring step (circles)
        # > 0: soft detach. The leaver's tension is blended down to detach_unload_t (the
        # survivors take the balanced n-1 split) over this long in the n-drone OCP before its
        # magnet opens; 0 = release and resize in one tick (r0013 capsized on that step)
        self._unload_s = float(p('detach_unload_s', 0.0).value)
        self._unload_t = float(p('detach_unload_t', 0.5).value)   # N: light but taut (a slack cable snatches)
        self._unload_tilt = float(p('detach_unload_tilt_deg', 10.0).value)
        self._unload_wait = float(p('detach_unload_wait_s', 2.0).value)
        self._post_blend_s = float(p('detach_post_blend_s', 1.0).value)
        self._unload = None          # the detach being unloaded, until release or cancel
        # > 0: a drone whose cable measures this much longer than its length for two planner
        # ticks has let go unannounced (a magnet failing mid-mission): it is treated as
        # detached and the fleet resizes without being told (Wesley 7 Oct)
        self._detect_m = float(p('detach_detect_m', 0.0).value)
        self._detect_count = {}
        self._detect_ok = {}          # each drone's excess on its last tick within the threshold
        self._detect_refused = set()  # detected but refused (min_survivors): do not re-fire
        handoff_topic = str(p('partner_handoff_topic', '').value)
        # combined M1 contact attach (card 2026-10-10_combined_m1_contact): his mission flies QUAD4 until our check
        # confirms the attach; the newcomer lifecycle stays on (our take-back is the no-attach path)
        self._partner_contact = bool(p('partner_contact', False).value)
        # the newcomer's starting share at the fold-in: 'start' attach_t_start_new, 'ocp' its n+1 OCP share
        # ('ocp' sagged plate 9 2.5-3.5 cm and tilted the ring 7.7-8.6 deg after the switch, 4 of 4; 'start'
        # 1.8-2.6 deg, 3 of 3: R1479-R1490)
        self._contact_fold_seed = str(p('contact_fold_seed', 'start').value)
        if self._contact_fold_seed not in ('ocp', 'start'):
            raise ValueError(f"contact_fold_seed must be 'ocp' or 'start', got {self._contact_fold_seed!r}")
        self._partner_run = (bool(handoff_topic) or self._partner_release) and not self._partner_contact
        if handoff_topic:
            self.create_subscription(Bool, handoff_topic, self._partner_handoff_cb, 5)
        # The network's ring must be the PHYSICAL tether ring (the parent's rho, the n
        # points the world SDF and the OCP use), NOT attach_points(n_net): with one
        # reserved slot that put the three tethers at 0/90/180 deg while they sit at
        # 0/120/240 -- an error the 0.08 m ring hid and the 0.25 m rim turned into a
        # capsize within a second of every weld (R0166/R0168). Reserved slots get a
        # gap-centre placeholder that the weld capture in _do_attach overwrites.
        net_rho = [np.asarray(r, float) for r in self.rho]
        for j in range(self.reserved_attach):
            # placeholder at the centre of the LARGEST free gap of the current ring
            # (with 3/12/9 o'clock tethers that is 6 o'clock); the weld overwrites it
            azs = np.sort(np.mod([np.arctan2(r[1], r[0]) for r in net_rho], 2 * np.pi))
            gaps = np.diff(np.r_[azs, azs[0] + 2 * np.pi])
            k = int(np.argmax(gaps))
            az = azs[k] + 0.5 * gaps[k]
            net_rho.append(np.array([self.attach_radius * np.cos(az),
                                     self.attach_radius * np.sin(az), self.attach_z]))
        self.net = DissipativeNetwork(
            self.n_net, net_rho, self.cable_len, self.drone_mass, self.load_mass,
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
        # newcomer lifecycle (card 2026-10-09_newcomer_land_abort; Wesley 8 Oct, M1 rules a and b):
        # LAND or /magnet/command OFF before the weld takes the newcomer out of the flight, it moves
        # radially out to DEPARTED_CLEAR_R and lands by itself. Off = the behaviour before it.
        # Not in partner runs (the partner's mission flies that drone and shares its magnet).
        self._newcomer_lifecycle = bool(p('newcomer_lifecycle', False).value) and not self._partner_run
        self._newcomer_out = {}         # j -> why it will not join this flight ('LAND', 'abort')
        self._land_alone = set()        # departed drones that land after their step, departed_land or not
        self._departed_wait = {}        # d -> s still held after its magnet OFF
        self._attach_pos_t = [None] * self.reserved_attach
        self._off_resend_t = None
        # a departed drone landing by itself reaches its own resting height and idles there (card
        # 2026-10-09_departed_ground_idle): deeper floor reference, down only at rest, the tracker told
        self._departed_gi = bool(p('departed_ground_idle', False).value) and not self._partner_run
        if self._departed_gi and (self.start_taut or self.creep.airborne_start):
            # an air start records the stand height as its rest: the press and the gate would sit in the air
            self.get_logger().info('[dissipative] air start: departed_ground_idle off (it needs a floor start)')
            self._departed_gi = False
        self._at_rest_t = {}          # d -> s spent at its rest while landing by itself
        self._press_t = {}            # d -> s pressed since it stopped above its rest
        self._rest_z = {}
        self._departed_sent = set()
        self._departed_pubs = {}
        if self._departed_gi:
            latched_gi = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            for dd in range(self.n_net):
                self._departed_pubs[dd] = self.create_publisher(Bool, f'/drone_{dd}/departed', latched_gi)
                self._departed_pubs[dd].publish(Bool(data=False))
        # a fault on an unwelded newcomer lands the fleet instead of disarming it (card
        # 2026-10-09_newcomer_fault_scope; Wesley 8 Oct night: "if QUAD4 fails we just land everything"):
        # each reserved drone's latched /drone_d/on_load (False until it is folded in) tells the fleet
        # manager to LAND on its in-flight disarm. Off = the fleet-wide stop. Needs the lifecycle switch.
        # welded starts (R10b/R11) keep it off until a row covers a newcomer welded from t=0
        # welded_on_load (card 2026-10-11_quad4_tilt_cut, critic round 1 M4; off by default): a welded start gets the
        # flag too, True at its start weld and False only once its release is SEEN after the detach (the tip off the
        # plate), so an off-ring QUAD4's disarm lands the fleet after the detach as well
        self._welded_on_load = bool(p('welded_on_load', False).value) and self._start_attached
        self._release_seen = {}       # d -> the tip's ring-frame position at the detach, until the release is seen
        self._tip_now = None
        self._newcomer_fault_land = (bool(p('newcomer_fault_land', False).value) and self._newcomer_lifecycle
                                     and (not self._start_attached or self._welded_on_load))
        if self._welded_on_load and self._newcomer_fault_land:
            self.create_subscription(PoseStamped, '/magnet_tip_pose', self._tip_now_cb, 10)
        self._on_load_announced = False
        self._newcomer_armed = {}
        self._on_load_pubs = {}
        if self._newcomer_fault_land:
            on_load_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE)
            for j in range(self.reserved_attach):
                d = self._n_carry0 + j
                # no flag until the fleet leaves the floor creep (_announce_on_load): the planner ignores
                # LAND in the creep, so a fault there keeps the old stop, with the ring still on the floor
                self._on_load_pubs[d] = self.create_publisher(Bool, f'/drone_{d}/on_load', on_load_qos)
                self.create_subscription(Bool, f'/drone_{d}/arming_state_feedback',
                                         lambda msg, j=j: self._newcomer_arming_cb(msg, j), 5)
        self._his_phase = ''
        self._his_took_off = False
        self._mux_state = None
        self._joint_free = None       # the ring weld joint read free (None: no state yet)
        self._ball_on = False
        self._ball_mass = float(p('contact_ball_mass', 0.1).value)   # his pickup ball (tejen/payload_model.sdf)
        self._span_t = None           # our ring magnet span: the last ON sent (None: not begun)
        self._taken_back = False
        self._contact_released = False   # the welded start (partner_release): QUAD4 released in the air
        # card 2026-10-11_quad4_tilt_cut, each off by default:
        #  partner_tilt_cut: QUAD4's tilt fault while the partner flies it takes it out of the flight (no take-back,
        #    no weld, no references; the mux cuts it and the fleet manager lands the fleet);
        #  partner_stuck_guard: our drone-3 tracker latching while the partner flies it blocks the hand-over (the mux
        #    would wait for ever for a tracker that will not fly again; critic round 1, H1)
        self._tilt_cut = bool(p('partner_tilt_cut', False).value) and self.reserved_attach > 0
        self._stuck_guard = bool(p('partner_stuck_guard', False).value) and self.reserved_attach > 0
        self._tilt_faulted = set()
        self._handover_blocked = False
        self._tilt_armed = {}
        if self._tilt_cut or self._stuck_guard:
            d3 = self._n_carry0
            latched_tc = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            if not self._partner_contact:
                self.create_subscription(String, f'/drone_{d3}/mux_state', self._mux_state_cb, latched_tc)
        if self._tilt_cut:
            self.create_subscription(Bool, f'/drone_{d3}/tilt_fault', self._tilt_fault_cb,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.VOLATILE))
        if self._stuck_guard:
            self.create_subscription(Bool, f'/drone_{d3}/arming_state_feedback', self._tilt_tracker_cb, 5)
            self._handover_block_pub = self.create_publisher(Bool, f'/drone_{d3}/handover_blocked', latched_tc)
            self._handover_block_pub.publish(Bool(data=False))
        if self._partner_contact and self.reserved_attach > 0:
            latched_pc = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            d = self._n_carry0
            self._take_back_pub = self.create_publisher(Bool, f'/drone_{d}/take_back', latched_pc)
            self.create_subscription(String, f'/drone_{d}/mux_state', self._mux_state_cb, latched_pc)
            self.create_subscription(String, '/join_planner/phase', self._his_phase_cb, latched_pc)   # his is latched
            self.create_subscription(Bool, '/join_planner/give_up', self._give_up_cb, 10)
            self.create_subscription(Bool, '/tejen/object_attached', self._ball_cb, 10)
            self.create_subscription(Bool, '/magnet/contact', self._contact_cb, 10)
            # the gz state of the ring weld (sim): the span waits for 'detached' after spawn. Its bridge latches the
            # one-shot state for a late subscriber. '' (the rig): no joint to wait for
            joint_topic = str(p('contact_joint_state_topic', '').value)
            if joint_topic:
                self.create_subscription(String, joint_topic, self._joint_state_cb, latched_pc)
            else:
                self._joint_free = True
            self.get_logger().warn(
                f'[dissipative] partner_contact: drone {d} warm from his takeoff, ring magnet span from his rejoin, '
                f'take-back on his give-up or our LAND before the attach (joint gate: {joint_topic or "none"})')
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
        if getattr(self, '_z_ki_in_orbit', False) and self._reconfig_mode != 'ocp':
            # the network keeps the unshifted target height: an orbit-grown integral would
            # step it at network entry (card 2026-09-27_zki_orbit, critic)
            self._z_ki_in_orbit = False
            self.get_logger().warn('[dissipative] z_ki_in_orbit needs reconfig_mode ocp: off')
        # Seconds to freeze the trajectory clock across a resize. A plain timer, not a
        # settle detector: the settle detector this replaced timed out on a fleet that
        # was already still, because it tested "load reached its target" against a
        # documented 5-7 cm steady sag it can never reach.
        self._reconfig_hold_s = float(p('reconfig_hold_s', 1.5).value)
        self._reconfig_hold_left = 0.0
        # LAND gate: /fleet/landed disarms the whole fleet, so a drone that left the
        # fleet must be this low before the survivors' touchdown is announced
        self._departed_down_z = float(p('departed_down_z', 0.15).value)
        # a freed drone carries no load: it descends faster than the fleet on LAND so
        # it is down BEFORE the survivors stall (0.0 = twice land_vel)
        self._departed_land_vel = float(p('departed_land_vel', 0.0).value)
        # true: a freed drone lands right after its step-out and idles on the floor (armed)
        # while the fleet flies on, out of a trajectory's way (Wesley 7 Oct: detach mid-circle)
        self._departed_land = bool(p('departed_land', False).value)
        self._departed_landing = set()
        self._departed_td = {}       # per departed drone: its own TouchdownDetector on LAND
        # rig: release the tether magnet from here on a detach (aux channel via
        # elrs_interface), so the command and the physical release are one tick apart
        self._detach_magnet = bool(p('detach_magnet', False).value)
        # fewest drones that may stay on the load after a detach. 3 -> 2 hangs the ring
        # from two rim points (a pendulum about their chord; capsizes in SIL) and the
        # RViz DETACH spinbox auto-increments, so a double press must not get there.
        self._min_survivors = int(p('min_survivors', 3).value)
        # Where each departed drone was parked, so it keeps getting a reference after
        # an OCP resize (the resized OCP plans only for the drones still on the load,
        # and a tracker with no reference trips ref_stale and disarms the fleet).
        self._departed_hold = {}
        self._departed_clear = {}       # d -> True once stepped clear of the ring on LAND
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
        # the tethers at construction only: a reserved (magnet) drone's latch belongs to the
        # magnet manager (weld ON, release OFF via /magnet/command), one path per radio
        self._magnet_pubs = ([self.create_publisher(String, f'/drone_{i}/magnet', 1)
                              for i in range(self._n_carry0)] if self._detach_magnet else [])

        # Drop and land (card 2026-10-08_drop_and_land, Wesley 8 Oct): a loss that leaves fewer
        # than min_survivors or a gap >= 180 deg, any loss in LAND, or a tracker's ring-tilt
        # request on /fleet/drop -> every magnet OFF, the ring is dropped and every drone steps
        # clear and lands. Detach mode only; off by default.
        self._drop_on_loss = bool(self.declare_parameter('drop_on_loss', False).value)
        if self._drop_on_loss and self._reconfig_mode != 'ocp':
            self.get_logger().error('[dissipative] drop_on_loss needs reconfig_mode ocp: '
                                    'turned off')
            self._drop_on_loss = False
        self._dropped = None            # the reason, once dropped
        self._drop_centre = None        # ring xy at the drop: the clear-out centre
        self._drop_t = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._dropped_pub = self.create_publisher(Bool, '/fleet/payload_dropped', latched)
        self._dropped_pub.publish(Bool(data=False))
        self.create_subscription(String, '/fleet/drop', self._drop_request_cb, 10)

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
        lv = msg.twist.linear
        self.attach_pos[j] = np.array([p.x, p.y, p.z])
        self._attach_pos_t[j] = self.get_clock().now().nanoseconds * 1e-9
        d = self._n_carry0 + j
        if d < len(self.drone_pos) and self._ocp_attached.get(d, False):
            # an OCP-attached newcomer is a planner slot now: the parent reads it
            # from drone_pos like any tethered drone
            self.drone_pos[d] = self.attach_pos[j]
            self.drone_vel[d] = np.array([lv.x, lv.y, lv.z])
            o = msg.pose.orientation
            self.drone_quat[d] = np.array([o.w, o.x, o.y, o.z])     # _pivot_at tilts the pivot with it

    def _reserved_index(self, d):
        """j for a reserved (magnet) drone id d, else None. Reserved ids are counted
        from the fleet size at construction: self.n shrinks/grows with OCP resizes."""
        j = d - self._n_carry0
        return j if 0 <= j < self.reserved_attach else None

    def _set_on_load(self, d, on):
        pub = getattr(self, '_on_load_pubs', {}).get(d)
        # no False before the fleet has left the floor creep (the planner ignores LAND there, so the manager must
        # keep its old stop): the magnet manager's startup 'not attached' reached the manager in R1261
        if pub is not None and (on or getattr(self, '_on_load_announced', False)):
            pub.publish(Bool(data=bool(on)))

    def _weld_refused(self, j):
        """Why reserved drone j may not be folded in now, or None."""
        why = self._newcomer_out.get(j)
        if why == 'fault' or (self._newcomer_fault_land and getattr(self, 'takeoff_seen', False)
                              and self._newcomer_armed.get(j) is False):
            return 'it disarmed in flight'
        if self._newcomer_lifecycle and (why is not None or self._land_to_ground):
            return why or 'LAND in progress'
        return None

    def _pos(self, d):
        """Drone d's measured position: its planner slot, else a reserved drone's own mocap (an
        unwelded newcomer has no drone_pos slot)."""
        if d < len(self.drone_pos) and self.drone_pos[d] is not None:
            return self.drone_pos[d]
        j = self._reserved_index(d)
        if j is None:
            return None
        t = self._attach_pos_t[j] if j < len(getattr(self, '_attach_pos_t', [])) else None
        if t is not None and self.get_clock().now().nanoseconds * 1e-9 - t > NEWCOMER_POSE_STALE_S:
            return None
        return self.attach_pos[j]

    # ── control tick: dispatch on phase (no parent modification) ─────────────
    def _plan(self):
        """Timed wrapper: a planner tick over ~0.5 s is a reference-staleness fault in the
        making (the trackers disarm at 1.0 s). Log what was going on when it happens."""
        import time as _time
        _t0 = _time.perf_counter()
        try:
            return self._plan_inner()
        finally:
            _dt = _time.perf_counter() - _t0
            if _dt > 0.50:
                self.get_logger().warn(
                    f'[dissipative] SLOW TICK {_dt:.2f} s in phase {self.phase} '
                    f'(hold {self._reconfig_hold_left:.1f} s, n_att '
                    f'{self.net.n_attached() if hasattr(self, "net") else "?"})')

    def _plan_inner(self):
        """The inherited timer calls this. In the network phase we run the spring-damper
        network; otherwise the inherited OCP planner flies (creep -> lift -> hover)."""
        if self._dropped is not None:
            self._drop_tick()
            return
        self._contact_release_tick()
        self._release_seen_tick()
        if self._departed_hold:
            self._publish_departed_refs()
        self._newcomer_off_tick()
        self._announce_on_load()
        self._partner_release_tick()
        self._publish_pending_attach_refs()
        self._partner_magnet_keepalive()
        self._contact_span_tick()
        if self.phase == 'network':
            self._network_plan()
            return
        self._unload_tick()
        self._detect_tick()
        # Freeze the trajectory clock across a resize, so a fleet that is still
        # settling into its new ring is not also asked to chase a moving target.
        if self._reconfig_hold_left > 0.0:
            self._reconfig_hold_left -= 1.0 / PLANNER_HZ
            # cancel the advance super()._plan() makes; in hover it makes none (R0518 drifted
            # to -10 s), so never below zero
            self.traj_t = max(0.0, self.traj_t - 1.0 / PLANNER_HZ)
            if self._settle_hold:
                self._settle_dwell_tick()
            elif self._reconfig_hold_left <= 0.0:
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
        if getattr(self, '_resize_log_ticks', 0) > 0:
            self._resize_log_ticks -= 1
            try:
                import json
                r = lambda a: [round(float(v), 4) for v in np.asarray(a, float).ravel()]
                X = self.solver.last_X
                self.get_logger().info('[dissipative] RESIZE_TICK ' + json.dumps({
                    'traj_t': float(self.traj_t), 'hold': float(self._reconfig_hold_left),
                    'load': r(self.load_state[0:10]),
                    'yref0': r(self.refs.yref_at(0)[0:6]), 'yrefN': r(self.refs.yref_at(self.N)[0:6]),
                    'plan_load': None if X is None else [r(X[0:3, k]) for k in (0, 1, 3, 10, X.shape[1] - 1)]}))
            except Exception as e:
                self.get_logger().warn(f'[dissipative] resize tick log failed: {e}')

    def _release_magnet(self, d):
        """Rig release for a detached drone: latch its tether magnet OFF through the
        radio. Only reached after every refusal `return` of the detach paths, so a
        refused detach never drops a magnet."""
        if not self._detach_magnet or d >= len(self._magnet_pubs):
            return
        self._magnet_pubs[d].publish(String(data='OFF'))
        self.get_logger().warn(f'[dissipative] magnet OFF -> /drone_{d}/magnet')

    def _departed_down(self):
        """Every drone that left on an OCP-mode detach is down: it has STALLED on its
        own descent (the same touchdown test as the fleet, so no absolute height on
        the rig, where the resting mocap z is not 0.10) or it is below departed_down_z.
        A hold of None (no pose at the detach) counts as NOT down: that drone has no
        reference and must not be disarmed on an assumption."""
        for d, p_hold in self._departed_hold.items():
            if p_hold is None:
                return False
            pos = self._pos(d)
            if pos is None:
                return False
            rest = self._rest_z.get(d) if getattr(self, '_departed_gi', False) else None
            if rest is not None:
                if float(pos[2]) > rest + DEPARTED_REST_TOL_M:
                    return False
                continue
            td = self._departed_td.get(d)
            stalled = td is not None and td.stalled
            if not stalled and float(pos[2]) > self._departed_down_z:
                return False
        return True

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
        if not self._land_to_ground and not self._departed_land:
            self._departed_td = {d: v for d, v in self._departed_td.items() if d in self._land_alone}
            self._departed_clear = {d: v for d, v in self._departed_clear.items() if d in self._land_alone}
        rate = self._departed_land_vel if self._departed_land_vel > 0.0 else 2.0 * self.land_vel
        for d, p_hold in self._departed_hold.items():
            if p_hold is None:
                continue
            a_ff, a_c = _free_drone_ff(self, d, g)
            wait = getattr(self, '_departed_wait', {}).get(d, 0.0)
            if wait > 0.0:                      # an aborted newcomer: the magnet lets go first
                self._departed_wait[d] = wait - 1.0 / PLANNER_HZ
                self._publish_ref(d, [(p_hold, zero, a_ff, a_c)] * (self.N + 1))
                continue
            alone = d in self._land_alone
            if d in self._departed_step and (alone or not self._land_to_ground) \
                    and d not in self._departed_landing:
                p_hold[:], _ = carrot_step(p_hold, self._departed_step[d], DETACH_STEP_VEL,
                                           1.0 / PLANNER_HZ)
                if d in self._departed_next and \
                        float(np.linalg.norm(p_hold - self._departed_step[d])) < 0.02:
                    self._departed_step[d] = self._departed_next.pop(d)
            td_prev = self._departed_td.get(d)
            down = td_prev is not None and td_prev.stalled
            # departed_land: straight down once the step-out is done, the fleet still flying
            if (((self._departed_land and not self._land_to_ground) or alone) and
                    (d not in self._departed_step or
                     float(np.linalg.norm(p_hold - self._departed_step[d])) < 0.02)):
                if d not in self._departed_landing:
                    self.get_logger().info(f'[dissipative] departed {_who(d)} lands now (departed_land)')
                self._departed_landing.add(d)
            early = d in self._departed_landing and not down
            if self._land_to_ground and not alone and not down \
                    and not self._departed_clear.get(d, False) and self.load_state is not None:
                # Step OUT before descending: the freed drone still trails its released
                # cable, and straight down from a hover detach it lands 0.63 m from the
                # ring centre with the cable across the ring's landing spot; the ring
                # pinned it and tipped the drone at touchdown (R0567).
                c = np.asarray(self.load_state[0:2], float)
                v = np.asarray(p_hold[0:2], float) - c
                r = float(np.linalg.norm(v))
                if r >= DEPARTED_CLEAR_R - 0.03:
                    self._departed_clear[d] = True
                else:
                    u = v / r if r > 1e-6 else np.array([1.0, 0.0])
                    goal = np.array([*(c + u * DEPARTED_CLEAR_R), float(p_hold[2])])
                    p_new, _ = carrot_step(p_hold, goal, rate, 1.0 / PLANNER_HZ)
                    p_hold[:] = p_new
            if (self._land_to_ground and not alone and (down or self._departed_clear.get(d, False))) \
                    or early or ((self._departed_land or alone) and down):
                # Descend faster than the fleet (no load on this drone) so it is down
                # before the survivors stall, and run its own touchdown detector. Once
                # stalled the reference freezes at the floor (as the survivors' does): a
                # reference sinking into the floor pressed the drone onto its trailing
                # cable and tipped it while it waited for the fleet (R0570)
                td0 = self._departed_td.get(d)
                pos = self._pos(d)
                rest = self._rest_z.get(d) if getattr(self, '_departed_gi', False) else None
                if rest is not None and d not in self._departed_sent:
                    self._departed_sent.add(d)
                    self._departed_pubs[d].publish(Bool(data=True))     # its tracker may idle at its rest
                if td0 is None or not td0.stalled:
                    if rest is None:
                        p_hold[2] = max(0.0, float(p_hold[2]) - rate / PLANNER_HZ)
                    else:
                        floor = rest - DEPARTED_PRESS_M
                        if pos is not None:
                            floor = max(floor, float(pos[2]) - DEPARTED_PRESS_M)
                        p_hold[2] = max(floor, float(p_hold[2]) - rate / PLANNER_HZ)
                td = self._departed_td.get(d)
                if td is None:
                    td = self._departed_td[d] = TouchdownDetector(rate, PLANNER_HZ)
                    td.stalled = False
                if rest is not None and pos is not None and not td.stalled:
                    # at its rest for DEPARTED_AT_REST_S: down, the press ends (also a drone that never left the
                    # floor, which the stall test cannot see: it needs a real descent first)
                    near = abs(float(pos[2]) - rest) <= DEPARTED_REST_TOL_M
                    self._at_rest_t[d] = self._at_rest_t.get(d, 0.0) + 1.0 / PLANNER_HZ if near else 0.0
                    if self._at_rest_t[d] >= DEPARTED_AT_REST_S:
                        td.stalled = True
                        p_hold[2] = float(pos[2])
                        self.get_logger().info(f'[dissipative] departed {_who(d)} is down (at its rest)')
                    elif d in self._press_t and not near:     # at its rest the 0.5 s above decides (R1268)
                        self._press_t[d] += 1.0 / PLANNER_HZ
                        if self._press_t[d] >= DEPARTED_PRESS_LIMIT_S:
                            td.stalled = True                      # no endless press: held where it is
                            p_hold[2] = float(pos[2])
                            self.get_logger().warn(
                                f'[dissipative] departed {_who(d)} still {float(pos[2]) - rest:+.2f} m above its '
                                f'rest after {DEPARTED_PRESS_LIMIT_S:.0f} s of press: held there (not down)')
                if pos is not None and not td.stalled and td.update([float(pos[2])]):
                    if rest is not None and float(pos[2]) > rest + DEPARTED_REST_TOL_M:
                        # it stopped above its rest (light on its rod, or standing on it): not down, press on
                        td.reset()
                        self._press_t.setdefault(d, 0.0)
                        if d not in getattr(self, '_high_stall_said', set()):
                            self._high_stall_said = getattr(self, '_high_stall_said', set()) | {d}
                            self.get_logger().warn(
                                f'[dissipative] departed {_who(d)} stopped {float(pos[2]) - rest:+.2f} m above '
                                f'its rest: still pressing down')
                    elif rest is None or not self._departed_gi:
                        td.stalled = True
                        p_hold[2] = float(pos[2])
                        self.get_logger().info(f'[dissipative] departed {_who(d)} is down')
            self._publish_ref(d, [(p_hold, zero, a_ff, a_c)] * (self.N + 1))

    def _attach_target_cb(self, msg: PoseStamped):
        q = msg.pose.position
        self._attach_target = np.array([q.x, q.y, q.z])

    def _tip_pose_cb(self, msg: PoseStamped):
        # the reserved newcomer's measured tip, stamped at receipt with its body pose beside it (validity)
        body = self.attach_pos[0] if self.reserved_attach and self.attach_pos[0] is not None else None
        if body is None:
            return
        q = msg.pose.position
        self._tip_frames.append((self.get_clock().now().nanoseconds * 1e-9, np.array([q.x, q.y, q.z]),
                                 np.asarray(body, float).copy()))

    def _tip_log_row(self, j, prof, ok, why):
        if self._run_log_dir is None:
            return
        try:
            if self._tip_csv is None:
                import csv
                f = open(os.path.join(self._run_log_dir, 'tip.csv'), 'w', newline='')
                self._tip_csv = (f, csv.writer(f))
                self._tip_csv[1].writerow(['t', 'drone', 'phase', 'hold', 'reason', 'tip_z', 'setpoint_z', 'e_z',
                                           'e_f', 'c_z', 'valid', 'why'])
            f, w = self._tip_csv
            loop, fr = prof.tip, self._tip_frames
            t = self.get_clock().now().nanoseconds * 1e-9
            sp = (float(self._attach_target[2]) + TIP_STANDOFF_M + self._tip_dz
                  if self._attach_target is not None else float('nan'))
            w.writerow([f'{t:.3f}', self._n_carry0 + j, prof.phase, int(loop.hold), loop.reason,
                        f'{fr[-1][1][2]:.4f}' if fr else '', f'{sp:.4f}',
                        '' if loop.e is None else f'{loop.e:+.4f}', '' if loop.e_f is None else f'{loop.e_f:+.4f}',
                        f'{loop.c:+.4f}', int(ok), why])
            f.flush()
        except Exception as e:
            if not getattr(self, '_tip_log_warned', False):
                self._tip_log_warned = True
                self.get_logger().warn(f'[dissipative] tip log off: {e}')

    def _attach_twist_cb(self, msg: TwistStamped):
        v = msg.twist.linear
        self._attach_target_vel = np.array([v.x, v.y, v.z])

    def _start_approach(self):
        """ATTACH commanded: every pending reserved drone with a pose starts its approach
        from where it is (bumpless first reference)."""
        if not self._attach_approach:
            return
        for j in range(self.reserved_attach):
            if self.attach_pending[j] and self.attach_pos[j] is not None and j not in self._approach \
                    and j not in self._newcomer_out:
                self._approach_z0 = getattr(self, '_approach_z0', {})
                self._approach_z0[j] = float(self.attach_pos[j][2])     # its resting height (rig: not 0.10)
                self._approach[j] = ApproachProfile(self.attach_pos[j], self._attach_body_to_tip,
                                                    1.0 / PLANNER_HZ,
                                                    seek_m=self._attach_seek_m,
                                                    direct=self._attach_approach_direct,
                                                    tip_loop=(TipLoop(1.0 / PLANNER_HZ, apply=self._tip_feedback)
                                                              if self._tip_log else None),
                                                    tip_dz=self._tip_dz)
                self.get_logger().info(
                    f'[dissipative] approach: drone {self._n_carry0 + j} flies to the weld '
                    f'target on its own tracker from {np.round(self.attach_pos[j], 2)} '
                    f'(body-to-tip {self._attach_body_to_tip:.3f} m)')

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
            if (self._n_carry0 + j) in self._departed_hold:
                continue                  # released partner drone: its fixed hold flies it
            if getattr(self, '_partner_contact', False):
                self._contact_warm_ref(j, load_p, g)
                continue
            prof = self._approach.get(j)
            if prof is not None and self._attach_target is not None:
                ok, why, tip_z = False, 'off', None
                if prof.tip is not None:
                    fr = list(self._tip_frames)
                    ok, why = tip_valid(self.get_clock().now().nanoseconds * 1e-9, fr)
                    tip_z = float(fr[-1][1][2]) if fr else None
                p_ref, v_ff, phase = prof.step(
                    self._attach_target,
                    self._attach_target_vel if self._attach_moving else None, tip_z=tip_z, tip_ok=ok)
                if prof.tip is not None:
                    self._tip_log_row(j, prof, ok, why)
                    if prof.tip.hold and not getattr(prof, '_hold_said', False):
                        prof._hold_said = True
                        self.get_logger().info(f'[dissipative] approach: drone {self._n_carry0 + j} tip hold '
                                               f'(tip {-prof.tip.e if prof.tip.e is not None else float("nan"):+.3f} m '
                                               f'from its setpoint, + above)')
                if phase != getattr(prof, '_logged', None):
                    prof._logged = phase
                    self.get_logger().info(
                        f'[dissipative] approach: drone {self._n_carry0 + j} {phase}')
                # a horizon that advances with the feedforward velocity, as the creep and
                # landing references do (a hold-shaped horizon with v != 0 is inconsistent)
                nodes = [(p_ref + v_ff * (self.dt * k), v_ff, g, zero) for k in range(self.N + 1)]
                self._publish_ref(self._n_carry0 + j, nodes)
                continue
            if self._attach_approach:
                # the tracker-flown approach: no reference at all before the approach.
                # A tracker with no reference idles armed at zero throttle on the
                # floor; every pre-approach hold tried today failed (a gated hold
                # stops when the load lifts away and trips ref_stale, R0529; a hold
                # that follows the pose drifts 2 m, R0530; a fixed anchor at the spawn
                # height makes the drone hover against the floor and tip, R0531/R0532)
                continue
            p_hold = np.asarray(self.attach_pos[j], float).copy()
            # latched once warm: a partner mission comes close (the drop) and leaves again
            # before the rejoin, and a started tracker whose references stop faults on
            # ref_stale and disarms the fleet (R0618)
            warm = getattr(self, '_warm_latched', set())
            if j not in warm:
                if float(np.linalg.norm(p_hold - load_p)) > self._attach_warm_radius:
                    continue
                warm.add(j)
                self._warm_latched = warm
            self._publish_ref(self._n_carry0 + j, [(p_hold, zero, g, zero)] * (self.N + 1))

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
        if self._dropped is not None:
            self.get_logger().warn('[dissipative] detach ignored - the ring was dropped, landing')
            return
        if not (0 <= d < len(self.detached)):
            self.get_logger().warn(f'[dissipative] /fleet/detach {d} out of range')
            return
        if self.phase not in ('planner', 'network'):
            self.get_logger().warn('[dissipative] detach ignored - not flying yet')
            return
        if self._land_to_ground or getattr(self, '_net_landing', False):
            self.get_logger().warn('[dissipative] detach ignored - LAND in progress')
            return
        if self.detached[d]:
            return
        if self._reconfig_mode == 'ocp':
            if self._unload is not None:
                if self._unload['d'] != d:
                    self.get_logger().warn(
                        f'[dissipative] detach {d} refused: drone {self._unload["d"]} is unloading')
                return
            if self._unload_s > 0.0:
                self._start_unload(d)
            else:
                self._detach_ocp(d)
            return
        # first detach also performs the OCP -> network handover (seeds bumplessly).
        if self.phase == 'planner':
            self._enter_network_phase()
        # network slot of physical drone d -- via the n_net mapping, so a drone that
        # JOINED mid-flight (reserved id >= n) can leave again (R0223/R0224 crashed here).
        slot = self._drone_to_net_slot(d)
        if slot is None or not self.net.attached[slot]:
            self.get_logger().warn(f'[dissipative] detach: {_who(d)} is not on the load')
            return
        if self.net.n_attached() - 1 < max(2, self._min_survivors):
            self.get_logger().error(
                f'[dissipative] refusing to detach {d}: {self.net.n_attached() - 1} would '
                f'remain (min_survivors {self._min_survivors})')
            return
        self.net.detach(slot)
        self.detached[d] = True
        if d < self.n:
            self.detach_pub[d].publish(Empty())
            self._release_magnet(d)
        else:
            # a magnet-attached newcomer has no DetachableJoint trigger of its own: the
            # magnet manager releases the weld on OFF (detach_when_magnet_off).
            self._magnet_cmd_pub.publish(String(data='OFF'))
        self.get_logger().info(
            f'[dissipative] DETACH {_who(d)} (slot {slot}); '
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
        plan = self._detach_plan(d)
        if plan is not None:
            self._resize_and_release(d, *plan)

    def _detach_plan(self, d):
        """(survivors, their attach points, largest gap) for detaching d, or None after
        logging why it is refused."""
        if d not in self.slot2drone:
            self.get_logger().warn(f'[dissipative] detach: {_who(d)} is not on the load')
            return None
        survivors = [x for x in self.slot2drone if x != d and not self.detached[x]]
        if len(survivors) < max(2, self._min_survivors):
            self.get_logger().error(
                f'[dissipative] refusing to detach {d}: {len(survivors)} would remain '
                f'(min_survivors {self._min_survivors})')
            return None
        # Each survivor keeps its OWN attach point. Captured BEFORE the resize, while
        # self.rho and self.slot2drone still describe the current fleet.
        rho_of = {self.slot2drone[sl]: self.rho[sl]
                  for sl in range(len(self.slot2drone))}
        rho_new = [rho_of[x] for x in survivors]
        return survivors, rho_new, _largest_gap_deg(rho_new)

    def _resize_and_release(self, d, survivors, rho_new, gap, post_blend=None):
        """Resize the OCP to the survivors and open d's magnet. `post_blend` = (t_from,
        seconds): start the survivors' tension references there (the soft detach's
        unloaded split) instead of stepping to the new balance. False if no OCP."""
        lens_of = {self.slot2drone[sl]: self.cable_len_i[sl] for sl in range(len(self.slot2drone))
                   if sl < len(self.cable_len_i)}
        lens = [lens_of[x] for x in survivors] if all(x in lens_of for x in survivors) else None
        if not self.resize_fleet(len(survivors), survivors, rho=rho_new, cable_lengths=lens):
            self.get_logger().error(
                f'[dissipative] detach {d} ABORTED: no OCP for n={len(survivors)}')
            return False
        if post_blend is not None:
            self.refs.start_blend(list(self.refs._s_nom), post_blend[0], post_blend[1])
            self._resize_log_ticks = 3
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
        if d < len(self.detach_pub):
            self.detach_pub[d].publish(Empty())
            self._release_magnet(d)
        else:
            self._magnet_cmd_pub.publish(String(data='OFF'))   # magnet-welded newcomer
        if getattr(self, '_welded_on_load', False) and self._reserved_index(d) is not None:
            self._release_seen[d] = None                       # on_load False once the tip has left the plate
        else:
            self._set_on_load(d, False)                        # off the ring: a fault grounds it alone
        self._departed_hold[d] = np.asarray(self.drone_pos[d], float).copy() \
            if self.drone_pos[d] is not None else None
        if self._partner_release and self._reserved_index(d) is not None \
                and self._departed_hold[d] is not None and self.load_state is not None:
            hold = self._departed_hold[d]
            out = hold[:2] - np.asarray(self.load_state[0:2], float)
            out = out / max(float(np.linalg.norm(out)), 1e-6)
            target = hold + np.array([self._release_out * out[0], self._release_out * out[1],
                                      self._release_up])
            self._release_state[d] = {'target': target, 'released': False, 'quiet': 0.0,
                                      'sent': None}
        elif self._detach_step_out > 0.0 and self._departed_hold[d] is not None \
                and self.load_state is not None:
            hold = self._departed_hold[d]
            out = hold[:2] - np.asarray(self.load_state[0:2], float)
            out = out / max(float(np.linalg.norm(out)), 1e-6)
            self._departed_step[d] = hold + self._detach_step_out * np.array([out[0], out[1], 0.0])
            self.get_logger().info(f'[dissipative] departed {_who(d)} steps {self._detach_step_out:.2f} m '
                                   f'out from the ring at {DETACH_STEP_VEL:.2f} m/s, then holds')
            clear = self._path_clear_point(self._departed_step[d], out)
            if clear is not None:
                self._departed_next[d] = clear
                self.get_logger().info(
                    f'[dissipative] departed {_who(d)} then clears the {self.traj.kind}: to '
                    f'({clear[0]:.2f}, {clear[1]:.2f}), {self.traj.radius + DEPARTED_PATH_CLEAR_M:.2f} m '
                    f'from the path centre')
        self._reconfig_hold_left = self._reconfig_hold_s
        self.get_logger().warn(
            f'[dissipative] DETACH {_who(d)} (OCP resize): n={self.n}, '
            f'trajectory held {self._reconfig_hold_s:.1f} s; surviving ring spans a '
            f'{gap:.0f} deg gap ({"CoG inside" if gap < 180.0 - 1e-6 else "CoG ON/OUTSIDE the hull — the load cannot hang level"})')
        return True

    def _path_clear_point(self, p, out):
        """During a circular trajectory: the point r_orbit + DEPARTED_PATH_CLEAR_M from the path
        centre, radially out through p (along `out`, the ring step, when p is at the centre).
        None when p is already that far out or the trajectory is not a circle."""
        if self.traj.kind not in ('orbit', 'circle', 'spin') or self.hover_xy is None:
            return None
        # load_trajectory: offset (r sin th, r (1 - cos th)) from the captured hover point
        c = np.array([float(self.hover_xy[0]), float(self.hover_xy[1]) + float(self.traj.radius)])
        v = np.asarray(p[:2], float) - c
        n = float(np.linalg.norm(v))
        r_clear = float(self.traj.radius) + DEPARTED_PATH_CLEAR_M
        if n >= r_clear:
            return None
        u = v / n if n > 0.05 else np.asarray(out[:2], float) / max(float(np.linalg.norm(out[:2])), 1e-6)
        return np.array([*(c + u * r_clear), float(p[2])])

    def _drop_due(self, d):
        """Why losing d means dropping the ring, or None to resize as usual: in LAND, fewer than
        min_survivors left, or the survivors' largest gap >= 180 deg (CoG on or outside them)."""
        if self._land_to_ground:
            return 'during LAND'
        survivors = [x for x in self.slot2drone if x != d and not self.detached[x]]
        if len(survivors) < max(2, self._min_survivors):
            return f'{len(survivors)} would remain (min_survivors {self._min_survivors})'
        rho_of = {self.slot2drone[sl]: self.rho[sl] for sl in range(len(self.slot2drone))}
        gap = _largest_gap_deg([rho_of[x] for x in survivors])
        if gap >= 180.0 - 1e-6:
            return f'the survivors span a {gap:.0f} deg gap (CoG on or outside them)'
        return None

    def _drop_request_cb(self, msg):
        if not self._drop_on_loss:
            self.get_logger().warn(f'[dissipative] /fleet/drop ignored (drop_on_loss off): '
                                   f'{msg.data}')
            return
        if self.phase != 'planner' or not self.takeoff_seen:
            self.get_logger().warn(f'[dissipative] /fleet/drop ignored - not flying: {msg.data}')
            return
        self._drop_and_land(msg.data)

    def _drop_and_land(self, reason):
        """Every magnet OFF, the ring dropped, every drone a departed drone that steps clear of
        the ring (DEPARTED_CLEAR_R from where the ring was) and lands; the OCP stops."""
        if self._dropped is not None:
            return
        self._dropped = reason
        # node clock: sim time in the twin (a wall-clock timeout fired at ~8 s of sim and disarmed
        # drones still descending, R1182), wall time on the rig
        self._drop_t = self.get_clock().now().nanoseconds * 1e-9
        self._cancel_unload_quietly()
        self.get_logger().error(f'[dissipative] DROP AND LAND: {reason} - every magnet OFF, '
                                f'the ring is dropped, every drone steps clear and lands')
        self._drop_centre = (np.asarray(self.load_state[0:2], float).copy()
                             if self.load_state is not None else None)
        for d in range(len(self.detached)):
            if d < len(self.detach_pub) and not self.detached[d]:
                self.detach_pub[d].publish(Empty())          # the twin's joints
            self.detached[d] = True
            if d in self._departed_hold and self._departed_hold[d] is not None:
                continue                                     # left earlier: keeps its own plan
            pos = self.drone_pos[d] if d < len(self.drone_pos) else None
            if pos is None:
                self._departed_hold[d] = None
                continue
            vel = self.drone_vel[d] if d < len(self.drone_vel) and self.drone_vel[d] is not None \
                else np.zeros(3)
            # hold where the drone will have braked to, not where it is (a zero-velocity hold
            # at the current pose is a hard brake for a drone already moving)
            hold = np.asarray(pos, float) + 0.2 * np.clip(np.asarray(vel, float), -1.0, 1.0)
            self._departed_hold[d] = hold
            if self._drop_centre is not None:
                v = hold[:2] - self._drop_centre
                r = float(np.linalg.norm(v))
                if r < DEPARTED_CLEAR_R:
                    u = v / r if r > 1e-6 else np.array([1.0, 0.0])
                    self._departed_step[d] = np.array([*(self._drop_centre + u * DEPARTED_CLEAR_R),
                                                       float(hold[2])])
        self._departed_next = {}
        self._departed_land = True
        self._dropped_pub.publish(Bool(data=True))
        self._release_all_magnets()

    def _cancel_unload_quietly(self):
        if getattr(self, '_unload', None) is not None:
            self._unload = None

    def _release_all_magnets(self):
        """Every tether OFF; re-sent each tick of a drop (one lost radio OFF would leave a drone
        tied to a fallen ring)."""
        for pub in self._magnet_pubs:
            pub.publish(String(data='OFF'))

    def _drop_tick(self):
        self._release_all_magnets()
        self._publish_departed_refs()
        if self._landed:
            return
        late = self.get_clock().now().nanoseconds * 1e-9 - self._drop_t > DROP_LAND_WAIT_S
        if self._departed_down() or late:
            self._landed = True
            self.landed_pub.publish(Bool(data=True))
            self.get_logger().warn('[dissipative] after the drop every drone is down - announcing '
                                   '/fleet/landed' + (f' (timed out after {DROP_LAND_WAIT_S:.0f} s)'
                                                      if late else ''))

    def _start_unload(self, d):
        """Soft detach, phase 1: in the n-drone OCP blend the leaver's tension down to
        detach_unload_t and the survivors' up to their balanced n-1 split (directions held),
        with the trajectory clock held. _unload_tick opens the magnet once the blend is done
        and the gates pass, or cancels."""
        plan = self._detach_plan(d)
        if plan is None:
            return
        k = self.slot2drone.index(d)
        _, t_pre = self.refs._refs_at(0)
        target = unload_tensions(self.dyn.rho, self.refs._s_nom, self.dyn.m, k, self._unload_t,
                                 rod_mass=getattr(self.dyn, 'mr', 0.0),
                                 rod_lam=getattr(self.dyn, 'lam', 0.0))
        self.refs.balanced_blend = False
        self.refs.retarget(target, self._unload_s)
        self._unload = {'d': d, 'k': k, 'plan': plan, 't_pre': list(t_pre),
                        'target': target, 'waited': 0.0}
        self._reconfig_hold_left = self._unload_s + self._unload_wait + 1.0
        pct = ' '.join(f'd{self.slot2drone[i]} {t_pre[i]:.2f}->{target[i]:.2f} '
                       f'({100.0 * (target[i] / max(t_pre[i], 1e-6) - 1.0):+.0f}%)'
                       for i in range(len(target)))
        self.get_logger().warn(
            f'[dissipative] UNLOAD {_who(d)} over {self._unload_s:.1f} s before release: {pct} N; '
            f'survivors span a {plan[2]:.0f} deg gap')

    def _unload_tick(self):
        """Soft detach, phase 2 (every tick while unloading): once the blend is done,
        release when the leaver's planned tension and the ring tilt pass the gates, or
        cancel after detach_unload_wait_s."""
        u = self._unload
        if u is None or self.refs.blend_active():
            return
        u['waited'] += 1.0 / PLANNER_HZ
        X = self.solver.last_X
        t_leave = (float(X[13 + 14 * u['k'] + 12, min(1, X.shape[1] - 1)])
                   if X is not None else float('inf'))
        R = quat_to_rot_np(self.load_state[3:7])
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        if t_leave <= self._unload_t + 0.3 and tilt <= self._unload_tilt:
            self._unload = None
            t_from = [t for i, t in enumerate(u['target']) if i != u['k']]
            self.get_logger().warn(
                f'[dissipative] UNLOADED drone {u["d"]}: planned {t_leave:.2f} N, ring tilt '
                f'{tilt:.1f} deg - releasing')
            if not self._resize_and_release(u['d'], *u['plan'],
                                            post_blend=(t_from, self._post_blend_s)):
                self._unload = u
                self._cancel_unload('resize failed')
        elif u['waited'] >= self._unload_wait:
            self._cancel_unload(f'gates not met {self._unload_wait:.1f} s after the blend '
                                f'(leaver planned {t_leave:.2f} N, tilt {tilt:.1f} deg)')

    def _detect_tick(self):
        """Unannounced detach: a cable cannot measure longer than itself while its magnet
        holds, so two ticks over its length by detach_detect_m mean the drone is gone. Resize
        to the survivors as an announced detach would (its magnet OFF is sent too: harmless
        if already open, and it stops a half-held magnet from catching again)."""
        landing = self._land_to_ground and not (self._drop_on_loss and not self._load_down())
        if (self._detect_m <= 0.0 or self.phase != 'planner' or self._reconfig_mode != 'ocp'
                or landing or self._unload is not None or not self.takeoff_seen):
            return
        now = time.monotonic()
        load_t = getattr(self, '_load_t', None)
        if load_t is None or now - load_t > DETECT_STALE_S:
            self._detect_count = {}           # no ring pose: nothing to measure a cable against
            return
        over_of = {}
        for i, d in enumerate(list(self.slot2drone)):
            if self.detached[d] or d in self._detect_refused:
                continue
            t_d = getattr(self, '_drone_t', {}).get(d)
            if t_d is not None and now - t_d > DETECT_STALE_S:
                self._detect_count[d] = 0     # a frozen drone pose would read as a stretched cable
                continue
            try:
                dist = self._rim_dist(i)
            except (TypeError, IndexError, ValueError):      # a pose not in yet
                continue
            # against the longer of the typed and measured rod: a rod measured short at the
            # handover (r001: 0.48) reads over once taut, which is not a release
            over_of[(i, d)] = (dist, dist - max(float(self.cable_len), float(self.cable_len_i[i])))
        n_over = sum(1 for _, ov in over_of.values() if ov > self._detect_m)
        if len(over_of) >= 2 and n_over == len(over_of):
            # every cable over at once: the ring's pose jumped (marker swap, quaternion flip),
            # not every magnet at once
            self._detect_count = {}
            return
        for (i, d), (dist, over) in over_of.items():
            if over <= self._detect_m:
                self._detect_count[d] = 0
                self._detect_ok[d] = over
                continue
            self._detect_count[d] = self._detect_count.get(d, 0) + 1
            # and it must have JUMPED since the last tick within the threshold: a release
            # stretches 10+ cm in 0.1-0.2 s (r200006), a mocap bias does not move
            rise = over - self._detect_ok.get(d, over)
            if self._detect_count[d] >= 2 and rise >= 0.03:
                self.get_logger().warn(
                    f'[dissipative] UNANNOUNCED DETACH: {_who(d)} cable {dist:.3f} m, '
                    f'{100.0 * over:.0f} cm over its length (up {100.0 * rise:.0f} cm) - resizing')
                self._detect_count = {}
                why = self._drop_due(d) if self._drop_on_loss else None
                if why is not None:
                    self._drop_and_land(f'unannounced loss of {_who(d)}: {why}')
                    return
                self._detach_ocp(d)
                if not self.detached[d]:
                    self._detect_refused.add(d)
                return

    def _cancel_unload(self, why):
        """Abandon a soft detach: the tensions blend back to the full split and the magnet
        stays ON."""
        u, self._unload = self._unload, None
        if u is None:
            return
        back = max(self._unload_s, 0.5)      # as gently as the unload went
        self.refs.retarget(u['t_pre'], back)
        self._reconfig_hold_left = back
        self.get_logger().warn(
            f'[dissipative] UNLOAD CANCELLED for drone {u["d"]}: {why}; tensions blend back '
            f'over {back:.1f} s, magnet stays ON')

    def _log_resize_inputs(self, what, d, t_before, s_from, t_from):
        """One JSON line with what the resized OCP starts from (M1 weld yank vs the bench,
        GOALS Parked). Never raises."""
        try:
            import json
            r = lambda a: [round(float(v), 4) for v in np.asarray(a, float).ravel()]
            X = self.solver.last_X
            load_nodes = None if X is None else [r(X[0:3, k]) for k in (0, 5, 10, X.shape[1] - 1)]
            self.get_logger().info('[dissipative] RESIZE_INPUTS ' + json.dumps({
                'what': what, 'drone': int(d), 'n': int(self.n),
                'load': r(self.load_state), 'hover_xy': r(self.hover_xy), 'traj_t': float(self.traj_t),
                'drone_pos': [None if p is None else r(p) for p in self.drone_pos],
                'drone_vel': [None if v is None else r(v) for v in self.drone_vel],
                't_before': r(t_before), 't_from': r(t_from), 's_from': [r(v) for v in s_from],
                't_nom': r(self.refs._t_nom), 'load_nodes': load_nodes}))
        except Exception as e:
            self.get_logger().warn(f'[dissipative] resize input log failed: {e}')

    # ── attach: weld a reserved drone in and fold it into the network ─────────
    def _fleet_attach_cb(self, msg: Int32):
        """Manual/offline-symmetry trigger: attach physical drone id msg.data."""
        self._do_attach(int(msg.data))

    def _magnet_attached_cb(self, msg: Bool):
        """The collaborator's manager announces a completed magnet weld here. On True,
        fold every still-pending reserved drone into the network (usually just one)."""
        if not msg.data:
            for j in range(self.reserved_attach):
                if self.attach_pending[j]:
                    self._set_on_load(self._n_carry0 + j, False)   # a refused weld let go
            return
        for j in range(self.reserved_attach):
            if self.attach_pending[j]:
                why = self._weld_refused(j)
                if why is not None:
                    d = self._n_carry0 + j
                    self.get_logger().warn(f'[dissipative] weld of {_who(d)} refused: {why}')
                    if _ring_off_allowed(self):
                        self._magnet_cmd_pub.publish(String(data='OFF'))
                    if d in self._departed_hold:
                        self._departed_wait[d] = NEWCOMER_RELEASE_S   # let go before it moves on
                    continue
                self._do_attach(self._n_carry0 + j)

    def _partner_handoff_cb(self, msg):
        if not msg.data or self._partner_handed or not any(self.attach_pending):
            return
        self._partner_handed = True
        self._attach_approach = True
        for j in range(self.reserved_attach):
            self._departed_hold.pop(self._n_carry0 + j, None)
        self.get_logger().warn('[dissipative] PARTNER HANDOFF: our tracker flies the last '
                               'descent; ring magnet ON')
        # started here: a partner run has no approach hold, so the ON below returns before it
        # would start the approach, and with _attach_approach set a newcomer with no approach
        # gets no reference at all (ref stale 4 s after the hand-off, fleet abort: R1470)
        self._start_approach()
        self._magnet_cmd_cb(String(data='ON'))
        self._partner_magnet_sent = None

    def _partner_release_tick(self):
        """Step a detached partner drone out and up at 0.15 m/s, wait until it is still
        there, then release it (announced every 1 s: his supervisor and our mux)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        contact = getattr(self, '_partner_contact', False)
        for d, st in self._release_state.items():
            if self._reserved_index(d) in getattr(self, '_tilt_faulted', ()):
                continue                  # cut on its tilt fault: no release, no start for his mission
            if st['released']:
                if contact and self._taken_back:
                    continue              # taken back: his mission is not started after it
                if st['sent'] is None or now - st['sent'] >= 1.0:
                    self._release_pub.publish(String(data='{"running": true}'))
                    st['sent'] = now
                continue
            if contact and (self._land_to_ground or self._dropped is not None):
                # LAND before the release: no release, it lands with the fleet as a departed drone
                if not st.get('cancelled'):
                    st['cancelled'] = True
                    self.get_logger().warn(f'[dissipative] PARTNER RELEASE of {_who(d)} cancelled (LAND before '
                                           f'it): it lands as a departed drone')
                continue
            hold = self._departed_hold.get(d)
            if hold is None:
                continue
            step = 0.15 / PLANNER_HZ
            delta = st['target'] - hold
            dist = float(np.linalg.norm(delta))
            if dist > step:
                self._departed_hold[d] = hold + delta / dist * step
                continue
            self._departed_hold[d] = st['target'].copy()
            pos = self.drone_pos[d] if d < len(self.drone_pos) else None
            vel = self.drone_vel[d] if d < len(self.drone_vel) else None
            still = (pos is not None and vel is not None
                     and float(np.linalg.norm(np.asarray(pos) - st['target'])) < 0.12
                     and float(np.linalg.norm(vel)) < 0.10)
            st['quiet'] = st['quiet'] + 1.0 / PLANNER_HZ if still else 0.0
            if st['quiet'] >= 2.0:
                st['released'] = True
                j = self._reserved_index(d)
                self.attach_pending[j] = True
                if contact:
                    self._contact_released = True
                self._warm_latched = getattr(self, '_warm_latched', set()) | {j}
                self._partner_handed = False
                # the fixed step-out hold stays until the rejoin handoff: if the partner
                # never takes over, a hold that follows the measured pose drifts
                # (R0530, R0638 3.15 m/s)
                self.get_logger().warn(
                    f'[dissipative] PARTNER RELEASE: drone {d} stepped out to '
                    f'{np.round(st["target"], 2)}; its mission takes it from here')

    def _contact_release_tick(self):
        """partner_contact with the welded start (the full M1): once his stream flies the released QUAD4 (the mux
        reads partner, his mission past WAIT_FOR_TAKEOFF), its fixed step-out hold gives way to the warm reference
        on its measured body (critic round 1, finding 1), so the attach and a take-back find our tracker on QUAD4.
        Run before the departed references: one reference per tick for drone 3."""
        if not getattr(self, '_partner_contact', False) or self._taken_back or not self._his_took_off \
                or getattr(self, '_mux_state', None) != 'partner':
            return
        for d, st in self._release_state.items():
            if st['released'] and not st.get('warm') and d in self._departed_hold:
                st['warm'] = True
                self._drop_departed(d)
                self.get_logger().warn(f'[dissipative] {_who(d)} flies on his stream: its step-out hold gives way '
                                       f'to the warm reference on its measured body')

    # ── welded_on_load (card 2026-10-11_quad4_tilt_cut, M4) ───────────────────────────────────────────────────
    def _tip_now_cb(self, msg):
        q = msg.pose.position
        self._tip_now = (self.get_clock().now().nanoseconds * 1e-9, np.array([q.x, q.y, q.z]))

    def _release_seen_tick(self):
        """After a welded start's detach: on_load False once the magnet tip has moved RELEASE_SEEN_M from where it
        sat on the ring (ring frame). On the rig the tether is slack, so the body moving says nothing; the tip
        staying on the plate means the coil still holds."""
        if not self._release_seen or self.load_state is None or self._tip_now is None:
            return
        t, tip = self._tip_now
        if self.get_clock().now().nanoseconds * 1e-9 - t > NEWCOMER_POSE_STALE_S:
            return
        R = quat_to_rot_np(self.load_state[3:7])
        rel = R.T @ (tip - np.asarray(self.load_state[0:3], float))
        for d, rel0 in list(self._release_seen.items()):
            if rel0 is None:
                self._release_seen[d] = rel
                continue
            moved = float(np.linalg.norm(rel - rel0))
            if moved >= RELEASE_SEEN_M:
                del self._release_seen[d]
                self._set_on_load(d, False)
                self.get_logger().warn(f'[dissipative] {_who(d)} released (its tip {moved:.3f} m off the plate): '
                                       f'on_load False, a fault lands the fleet from here')

    # ── partner_tilt_cut (card 2026-10-11_quad4_tilt_cut) ─────────────────────────────────────────────────────
    def _off_the_load(self, j):
        d = self._n_carry0 + j
        return bool(self.attach_pending[j]) or bool(d < len(self.detached) and self.detached[d])

    def _tilt_fault_cb(self, msg):
        """QUAD4's tilt fault while the partner flies it (the mux cuts it): out of this flight, overriding an earlier
        take-back: no take-back, no weld, no ring magnet span, no references (the landed gate does not wait for it),
        the ring magnet OFF (the mux and the manager turn it OFF too: a cut drone falls with or without the coil).
        On the ring it is a carrier, whose own envelope stops the fleet."""
        if not msg.data:
            return
        if self._mux_state not in ('partner', 'cut'):
            # ours flies it, or it is down and disarmed alone: its tracker's disarm decides (_newcomer_arming_cb)
            self.get_logger().warn(f'[dissipative] tilt fault of {_who(self._n_carry0)} while ours flies it or it is '
                                   f'disarmed: its own disarm decides')
            return
        for j in range(self.reserved_attach):
            d = self._n_carry0 + j
            if j in self._tilt_faulted:
                continue
            if not self._off_the_load(j):
                self.get_logger().error(f'[dissipative] TILT FAULT of {_who(d)} on the ring: a carrier fault (its '
                                        f'envelope stops the fleet)')
                continue
            self._tilt_faulted.add(j)
            self._newcomer_out[j] = 'fault'
            self._approach.pop(j, None)
            self._approach_hold_armed = False
            for st in (self._departed_hold, self._departed_step, self._departed_next,
                       self._departed_td, self._departed_clear, self._departed_wait):
                st.pop(d, None)
            self._departed_landing.discard(d)
            self._land_alone.discard(d)
            self._release_seen.pop(d, None)
            self._magnet_cmd_pub.publish(String(data='OFF'))
            self.get_logger().error(
                f'[dissipative] TILT FAULT of {_who(d)} off the ring (mux {self._mux_state}): out of this flight - '
                f'no take-back, no weld, no references, ring magnet OFF')

    def _tilt_tracker_cb(self, msg):
        """Our drone-3 tracker latched (disarmed after TAKEOFF) while the partner flies QUAD4: the mux can never
        hand it to ours (its live-takeover rule waits for a tracker that will not fly again), so no attach is
        confirmed and the take-back cannot switch (critic round 1, H1)."""
        j = 0
        was = self._tilt_armed.get(j, False)
        self._tilt_armed[j] = bool(msg.data)
        if not was or msg.data or self._handover_blocked or not getattr(self, 'takeoff_seen', False) \
                or self._landed or self._mux_state != 'partner' or j in self._tilt_faulted:
            return
        self._handover_blocked = True
        self._handover_block_pub.publish(Bool(data=True))
        self.get_logger().error(
            f'[dissipative] {_who(self._n_carry0)}\'s tracker latched while the partner flies it: no take-back or '
            f'hand-over is possible, no attach will be confirmed; at LANDED its mux is cut if it is at its rest, '
            f'otherwise press DISARM')

    def _partner_magnet_keepalive(self):
        if not self._partner_handed or not any(self.attach_pending):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._partner_magnet_sent is None or now - self._partner_magnet_sent >= 1.0:
            self._magnet_cmd_pub.publish(String(data='ON'))
            self._partner_magnet_sent = now

    # ── partner_contact (card 2026-10-10_combined_m1_contact): his mission flies QUAD4 to the attach ──────
    def _his_phase_cb(self, msg):
        ph = str(msg.data).strip().upper()
        if ph != self._his_phase:
            self.get_logger().info(f'[dissipative] his phase {ph}')
        self._his_phase = ph
        if not self._his_took_off and ph not in NOT_FLOWN_PHASES:
            self._his_took_off = True
            self.get_logger().warn(f'[dissipative] his takeoff ({ph}): drone {self._n_carry0}\'s tracker warm on '
                                   f'its measured body from here')

    def _mux_state_cb(self, msg):
        self._mux_state = str(msg.data).strip().lower()

    def _ball_cb(self, msg):
        if bool(msg.data) != self._ball_on:
            self.get_logger().info(f'[dissipative] his ball {"on" if msg.data else "off"}: drone '
                                   f'{self._n_carry0}\'s model mass {"+" if msg.data else "-"}{self._ball_mass:.2f} kg')
        self._ball_on = bool(msg.data)

    def _contact_cb(self, msg):
        self.get_logger().info(f'[dissipative] /magnet/contact {bool(msg.data)}')

    def _joint_state_cb(self, msg):
        s = str(msg.data).strip().lower()
        if s not in ('attached', 'detached'):
            return
        if (s == 'detached') != self._joint_free:
            self.get_logger().info(f'[dissipative] ring weld joint {s}')
        self._joint_free = s == 'detached'

    def _give_up_cb(self, msg):
        if msg.data:
            for j in range(self.reserved_attach):
                self._take_back(j, 'his give-up')

    def _contact_warm_ref(self, j, load_p, g):
        """From his takeoff, drone j's tracker gets its measured body (not only near the ring, as the
        proximity-gated hold does) so it is warm wherever a take-back finds it; within the warm radius the ring
        velocity is fed forward along the horizon, the attach's hand-over point."""
        if not self._his_took_off or j in self._newcomer_out:
            return
        d = self._n_carry0 + j
        p = np.asarray(self.attach_pos[j], float).copy()
        v = np.zeros(3)
        if float(np.linalg.norm(p - load_p)) <= self._attach_warm_radius and len(self.load_state) >= 10:
            v = np.asarray(self.load_state[7:10], float).copy()
        a_ff, a_c = _free_drone_ff(self, d, g)
        self._publish_ref(d, [(p + v * (self.dt * k), v, a_ff, a_c) for k in range(self.N + 1)])

    def _contact_span_tick(self):
        """Our ring magnet span: ON from his rejoin (TRANSIT_TO_REATTACH on) once QUAD4 is above
        CONTACT_SPAN_MIN_Z and the ring weld joint reads free, re-sent every CONTACT_SPAN_EVERY_S until the attach
        or the take-back. Before it the one coil holds his ball on the rig."""
        if not self._partner_contact or self._taken_back or not self.reserved_attach \
                or not self.attach_pending[0] or 0 in self._newcomer_out:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._span_t is None:
            pos = self._pos(self._n_carry0)
            if (self._his_phase not in REJOIN_PHASES or pos is None or float(pos[2]) <= CONTACT_SPAN_MIN_Z
                    or self._joint_free is not True or self._land_to_ground or self._dropped is not None):
                return
            self.get_logger().warn(f'[dissipative] ring magnet span begins: ON (his {self._his_phase}, drone '
                                   f'{self._n_carry0} at z {float(pos[2]):.2f} m, the ring weld joint free)')
        elif now - self._span_t < CONTACT_SPAN_EVERY_S:
            return
        self._span_t = now
        self._magnet_cmd_pub.publish(String(data='ON'))

    def _reserved_plate_dir(self, j, c):
        """Unit xy from the ring centre c to reserved drone j's plate: the live attach target, else its slot."""
        t = self._attach_target
        if t is not None and float(np.linalg.norm(np.asarray(t[:2], float) - c)) > 1e-3:
            v = np.asarray(t[:2], float) - c
        else:
            v = (quat_to_rot_np(self.load_state[3:7]) @ self.net.rho[self._n_carry0 + j])[:2]
        return v / max(float(np.linalg.norm(v)), 1e-6)

    def _take_back(self, j, why):
        """No attach (his give-up, or our LAND before it): QUAD4 goes to our tracker (/drone_3/take_back) and holds
        where it is measured; then the departed path: within DEPARTED_CLEAR_R of the ring centre radially out at
        its height (over the middle, r < attach_radius, along its reserved plate's azimuth), else down where it is,
        landing and disarmed alone. The ring magnet goes OFF only if our span began. Before his takeoff it stays
        on its mat with no reference (a hold there fights the floor, R0531/R0532)."""
        if not self._partner_contact or self._taken_back or not (0 <= j < self.reserved_attach) \
                or not self.attach_pending[j]:
            return
        d = self._n_carry0 + j
        if j in getattr(self, '_tilt_faulted', ()):
            self.get_logger().warn(f'[dissipative] no take-back of {_who(d)} ({why}): it was cut on its tilt fault',
                                   throttle_duration_sec=2.0)
            return
        self._taken_back = True
        self._newcomer_out[j] = why
        self._take_back_pub.publish(Bool(data=True))
        off = self._span_t is not None
        if off:
            self._magnet_cmd_pub.publish(String(data='OFF'))
            self._off_resend_t = 0.0
        pos = self.attach_pos[j]
        # (the welded start releases QUAD4 in the air: from then on it is taken back where it flies)
        if (not self._his_took_off and not getattr(self, '_contact_released', False)) or pos is None:
            self.get_logger().warn(f'[dissipative] TAKE-BACK of {_who(d)} ({why}) before his takeoff: it stays '
                                   f'where it is, no reference')
            return
        hold = np.asarray(pos, float).copy()
        self._departed_hold[d] = hold
        self._land_alone.add(d)
        rest = self._rest_z.get(d)
        floor = self._departed_down_z if rest is None else rest + NEWCOMER_LIFTED_M
        if float(hold[2]) < floor:
            self.get_logger().warn(f'[dissipative] TAKE-BACK of {_who(d)} ({why}) on the floor: it lands where it is')
            return
        self._departed_wait[d] = NEWCOMER_RELEASE_S
        where = 'down where it is'
        # with his ball on, the ball lands first and QUAD4 comes down beside it along the rod: clear by the rod too
        # (R1482: taken back at r 1.3 m, it landed at r 1.03, 0.33 m from carrier 1)
        clear_r = DEPARTED_CLEAR_R + (self._attach_body_to_tip if self._ball_on else 0.0)
        if self.load_state is not None:
            c = np.asarray(self.load_state[0:2], float)
            v = hold[:2] - c
            r = float(np.linalg.norm(v))
            u = v / r if r >= self.attach_radius else self._reserved_plate_dir(j, c)
            if r < clear_r:
                self._departed_step[d] = np.array([*(c + u * clear_r), float(hold[2])])
                where = f'out to {clear_r:.2f} m from the ring at its height, then down'
            # during a circle, on out past the ring's path before it lands (as an aborted newcomer does)
            clear = self._path_clear_point(self._departed_step.get(d, hold), u)
            if clear is not None:
                if d in self._departed_step:
                    self._departed_next[d] = clear
                else:
                    self._departed_step[d] = clear
                where = 'out past the ring\'s path at its height, then down'
        self.get_logger().warn(f'[dissipative] TAKE-BACK of {_who(d)} ({why}): our tracker takes it, '
                               f'{"magnet OFF, " if off else ""}held at {np.round(hold, 2)}, then {where}; it '
                               f'lands alone')

    def _drop_departed(self, d):
        for st in (self._departed_hold, self._departed_step, self._departed_next,
                   self._departed_td, self._departed_clear, self._departed_wait):
            st.pop(d, None)
        self._departed_landing.discard(d)
        self._land_alone.discard(d)

    def _magnet_cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'OFF':
            # the operator's abort (Wesley 8 Oct, M1 rule b): an unwelded newcomer on its approach
            # moves clear and lands; the carriers fly on
            if self._newcomer_lifecycle and self.phase == 'planner':
                for j in range(self.reserved_attach):
                    if self.attach_pending[j] and j in self._approach:
                        self._abort_newcomer(j, 'abort')
            return
        if cmd == 'ON' and self._newcomer_lifecycle and (self._land_to_ground or self._dropped is not None
                            or any(self.attach_pending[j] and j in self._newcomer_out
                                   for j in range(self.reserved_attach))):
            # an ON repeated by the operator's tools (or the runner) gets an OFF each time, but one line
            # per 2 s (R1324: 2964 lines in 1.7 s)
            self.get_logger().warn('[dissipative] ATTACH refused: '
                                   + ('LAND in progress' if self._land_to_ground else
                                      'the ring was dropped' if self._dropped is not None else
                                      'the newcomer is out of this flight'), throttle_duration_sec=2.0)
            if any(self.attach_pending) and _ring_off_allowed(self):
                self._magnet_cmd_pub.publish(String(data='OFF'))   # never to a welded newcomer: OFF releases it
            return
        if getattr(self, '_partner_contact', False):
            return            # the ON is our own span's: no trajectory hold, no approach (critic round 3, H and 3)
        if not self._approach_hold_on:
            return
        # ON is republished by the operator tools; arm the hold once per approach so a
        # repeat cannot keep restarting the timer (the weld re-arms it explicitly).
        if cmd == 'ON' and self.phase in ('planner', 'network') \
                and not getattr(self, '_approach_hold_armed', False):
            self._approach_hold_armed = True
            if self._attach_moving:
                self.get_logger().info('[dissipative] ATTACH commanded: trajectory keeps running '
                                       '(attach_moving)')
                self._start_approach()
                return
            # Hold for as long as the approach takes (it is ~11 s and a timed hold that
            # expires first hands the descending tip a moving target: R0201/R0202). The
            # weld replaces this with the timed reconfiguration hold. Capped so a failed
            # approach cannot park the fleet forever.
            self._reconfig_hold_left = max(self._attach_traj_hold_s, 60.0)
            self.get_logger().info(
                f'[dissipative] ATTACH commanded: trajectory held {self._attach_traj_hold_s:.0f} s '
                f'for the approach (re-armed at the weld)')
            self._start_approach()

    def _abort_newcomer(self, j, why):
        """Take unwelded newcomer j out of this flight (Wesley 8 Oct, M1 rules a and b): its magnet
        OFF first, its approach stopped, and it becomes a departed drone that holds the approach's
        last reference for NEWCOMER_RELEASE_S (the magnet lets go), moves radially out from the ring
        centre to DEPARTED_CLEAR_R at that height and lands by itself; the landed gate waits for it.
        A newcomer whose approach never started stays where it is, with no reference (R0532)."""
        if not self._newcomer_lifecycle or not (0 <= j < self.reserved_attach) \
                or not self.attach_pending[j] or j in self._newcomer_out:
            return
        d = self._n_carry0 + j
        self._newcomer_out[j] = why
        self._magnet_cmd_pub.publish(String(data='OFF'))
        self._off_resend_t = 0.0
        prof = self._approach.pop(j, None)
        self._approach_hold_armed = False
        if not self._attach_moving and self._reconfig_hold_left > 0.0:
            self._reconfig_hold_left = 0.0                  # the approach's trajectory hold
        if prof is None:
            self.get_logger().warn(f'[dissipative] {_who(d)} out of this flight ({why}) before its '
                                   f'approach: it stays where it is')
            return
        pos = self._pos(d)
        z0 = getattr(self, '_approach_z0', {}).get(j)
        floor = self._departed_down_z if z0 is None else z0 + NEWCOMER_LIFTED_M
        if pos is not None and float(pos[2]) < floor:
            # its approach had started (its tracker has references) but it has not left the floor:
            # a departed drone that lands where it is, so the reference keeps coming and freezes at the stall
            self._departed_hold[d] = np.asarray(pos, float).copy()
            self._land_alone.add(d)
            self.get_logger().warn(f'[dissipative] {_who(d)} out of this flight ({why}) on the floor: it '
                                   f'stays down')
            return
        hold = np.asarray(prof.p_ref, float).copy()
        self._departed_hold[d] = hold
        self._departed_wait[d] = NEWCOMER_RELEASE_S
        if self.load_state is not None:
            c = np.asarray(self.load_state[0:2], float)
            v = hold[:2] - c
            r = float(np.linalg.norm(v))
            u = v / r if r > 1e-6 else np.array([1.0, 0.0])
            if r < DEPARTED_CLEAR_R:
                self._departed_step[d] = np.array([*(c + u * DEPARTED_CLEAR_R), float(hold[2])])
            # during a circle, on out past the ring's path before it lands (as a detached drone does)
            clear = self._path_clear_point(self._departed_step.get(d, hold), u)
            if clear is not None:
                if d in self._departed_step:
                    self._departed_next[d] = clear
                else:
                    self._departed_step[d] = clear
        self._land_alone.add(d)
        self.get_logger().warn(f'[dissipative] {_who(d)} out of this flight ({why}): magnet OFF, '
                               f'{NEWCOMER_RELEASE_S:.1f} s for the release, then radially out to '
                               f'{DEPARTED_CLEAR_R:.1f} m from the ring at its height and lands')

    def _announce_on_load(self):
        """Publish each unwelded newcomer's on_load False once the fleet has left the creep (LAND is acted on
        from the 'planner' phase on), so the fleet manager lands the fleet on its fault from then on."""
        if self._on_load_announced or not self._on_load_pubs or self.phase not in ('planner', 'network'):
            return
        self._on_load_announced = True
        for j in range(self.reserved_attach):
            if self.attach_pending[j]:
                self._set_on_load(self._n_carry0 + j, False)

    def _newcomer_off_tick(self):
        """While a newcomer is out and unwelded, keep its magnet OFF: the runner and the RViz panel
        repeat ON, and the magnet manager welds on its own geometry."""
        if not self._newcomer_out or self._off_resend_t is None:
            return
        if not any(self.attach_pending[j] for j in self._newcomer_out):
            return
        self._off_resend_t += 1.0 / PLANNER_HZ
        if self._off_resend_t >= NEWCOMER_OFF_EVERY_S:
            self._off_resend_t = 0.0
            if _ring_off_allowed(self):
                self._magnet_cmd_pub.publish(String(data='OFF'))

    def _newcomer_arming_cb(self, msg, j):
        """A reserved drone that is not on the ring disarms in flight (its tracker's own fault: tilt, speed,
        pose lost, reference stale). The fleet manager does not escalate it (its on_load is False); here it
        is out of this flight: magnet OFF, its approach and departed path dropped so the landed gate does
        not wait for a drone that has fallen, and no weld later."""
        was = self._newcomer_armed.get(j, False)
        self._newcomer_armed[j] = bool(msg.data)
        if not was or msg.data or not getattr(self, 'takeoff_seen', False) or self._landed:
            return
        if getattr(self, '_partner_contact', False) and getattr(self, '_mux_state', None) != 'ours':
            # his stream flies QUAD4: our idle tracker's disarm must not turn the magnet OFF or take QUAD4 out
            self.get_logger().warn(f'[dissipative] {_who(self._n_carry0 + j)}\'s tracker disarmed while his stream '
                                   f'flies it: ignored here')
            return
        d = self._n_carry0 + j
        if not self.attach_pending[j] and d not in self._departed_hold:
            return                                     # on the ring: a carrier like any other
        if self._newcomer_out.get(j) == 'fault':
            return
        rest, pos = self._rest_z.get(d), self._pos(d)
        at_rest = (d in self._departed_hold and rest is not None and pos is not None
                   and abs(float(pos[2]) - rest) <= DEPARTED_REST_TOL_M)
        self._newcomer_out[j] = 'fault'
        if self.attach_pending[j] and _ring_off_allowed(self):
            self._magnet_cmd_pub.publish(String(data='OFF'))
            self._off_resend_t = 0.0
        self._approach.pop(j, None)
        self._approach_hold_armed = False
        if not self._attach_moving and self._reconfig_hold_left > 0.0:
            self._reconfig_hold_left = 0.0                  # the approach's trajectory hold
        for st in (self._departed_hold, self._departed_step, self._departed_next,
                   self._departed_td, self._departed_clear, self._departed_wait):
            st.pop(d, None)
        self._departed_landing.discard(d)
        self._land_alone.discard(d)
        if at_rest:                                     # the fleet manager's disarm of a drone down alone
            self.get_logger().info(f'[dissipative] {_who(d)} disarmed at its rest off the ring: out of this flight')
            return
        self.get_logger().error(f'[dissipative] {_who(d)} disarmed in flight off the ring: out of this '
                                f'flight (its magnet OFF, the landed gate does not wait for it)')

    def _do_attach(self, d):
        """Add physical drone d to the dissipative network (mirror of _fleet_detach_cb).
        The first attach also performs the OCP -> network handover, so an attach works
        from a plain OCP hover as well as after an earlier detach."""
        if not (0 <= d < self.n_net):
            self.get_logger().warn(f'[dissipative] /fleet/attach {d} out of range')
            return
        j = self._reserved_index(d)
        why = self._weld_refused(j) if j is not None else None
        if why is not None:
            self.get_logger().warn(f'[dissipative] attach of {_who(d)} refused: {why}')
            return
        if self.phase == 'creep' and self._start_weld_joins_creep(d):
            self._attach_ocp(d, start_creep=True)
            return
        if self.phase not in ('planner', 'network'):
            # the manager repeats the weld at its timer rate (the props-off attach check: 551 lines in 20 s)
            self.get_logger().warn('[dissipative] attach ignored - not flying yet', throttle_duration_sec=2.0)
            return
        if self._reconfig_mode == 'ocp' and self.phase == 'planner':
            self._attach_ocp(d)
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
        # Captured in EVERY mode: the rim point is physical, and the central lifter's
        # vertical target and the ring modes' azimuth homes all hang off it.
        if self.load_state is not None:
            load_pos = np.asarray(self.load_state[0:3], float)
            R = quat_to_rot_np(self.load_state[3:7])
            # Project the magnet TIP, not the drone body: a reserved newcomer's arm hangs
            # one arm-length straight below it, and on a tilted load the body's load-frame
            # projection lands ~0.5*sin(tilt) inboard of the real weld (R0179/R0180: the 6
            # o'clock rim captured as r=0.03-0.07 with the disc at 27 deg, so the wrench
            # solve gave the newcomer no moment arm and the load never levelled).
            rho_body = self._capture_weld_rho(measured, d)
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
        if getattr(self, '_partner_contact', False):
            self._drop_departed(d)
        self._set_on_load(d, True)
        if self._reserved_index(d) is not None:
            self.attach_pending[self._reserved_index(d)] = False
            getattr(self, '_warm_latched', set()).discard(self._reserved_index(d))
            self._approach.pop(self._reserved_index(d), None)
        self._weld_time[d] = self.get_clock().now()   # start the FF gate ramp for this newcomer
        self.detached[d] = False
        mode = ('central lifter' if self._attach_central
                else f'soft hand-out ({self.diss.T_handout:.0f}s)' if self._attach_handout
                else 'instant ring')
        self.get_logger().info(
            f'[dissipative] ATTACH drone {d} (slot {slot}) as {mode}; '
            f'{self.net.n_attached()} drones now on the load')
        if self._attach_traj_hold_s > 0.0:
            self._approach_hold_armed = False                       # ready for another approach
            if self._hold_mode == 'settle':
                self._reconfig_hold_left = self._hold_max_s         # cap; settling ends it
                self._settle_hold, self._settle_for, self._settle_elapsed = True, 0.0, 0.0
                self._settle_prev_tilt = None
                self.get_logger().info(
                    f'[dissipative] trajectory held until the load tilt settles '
                    f'(< {self._hold_resume_tilt:.0f} deg, quiet {self._hold_settle_s:.1f} s; '
                    f'cap {self._hold_max_s:.0f} s), then resumes')
            else:
                self._reconfig_hold_left = self._attach_traj_hold_s     # replaces the approach hold
                self.get_logger().info(
                    f'[dissipative] trajectory held {self._attach_traj_hold_s:.1f} s for the '
                    f'reconfiguration, then resumes')

    def _capture_weld_rho(self, measured, d, clamp=True, keep_z=False):
        """Load-frame attach point of a drone that just welded: the magnet TIP (one arm
        length below the body for a reserved newcomer), projected into the load frame
        and clamped to the attach ring radius (a leaning body over a rim weld must not
        become a phantom lever, R0179/R0180)."""
        load_pos = np.asarray(self.load_state[0:3], float)
        R = quat_to_rot_np(self.load_state[3:7])
        weld_pt = np.asarray(measured, float)
        if d >= self._n_carry0:
            weld_pt = weld_pt - np.array([0.0, 0.0, float(self._attach_body_to_tip)])
        rho_body = R.T @ (weld_pt - load_pos)
        r_xy = float(np.hypot(rho_body[0], rho_body[1]))
        if clamp and r_xy > self.attach_radius and r_xy > 1e-6:
            rho_body[:2] *= self.attach_radius / r_xy
        if not keep_z:
            rho_body[2] = float(self.attach_z)
        return rho_body

    def _start_weld_joins_creep(self, d):
        """Floor start with a drone welded on at the start (Q9): it joins the creep as one
        more carrier, so the OCP and the creep are sized for it before TAKEOFF. The air
        start never gets here (its first tick leaves the creep phase)."""
        return (self._start_attached and not self.start_taut
                and self._reconfig_mode == 'ocp' and not self.takeoff_seen
                and self._reserved_index(d) is not None)

    def _attach_ocp(self, d, start_creep=False):
        """Attach WITHOUT handing the fleet to the network: grow the OCP in place, the
        mirror of _detach_ocp. The newcomer's weld point becomes one more attach point of
        the (generally uneven) ring, its magnet arm one more rod, and the pre-built n+1
        OCP takes over on the next tick with node 0 pinned to the measured state. The
        trajectory is held (timed or settle) exactly as the network attach holds it.

        start_creep: the floor-start weld folded in before TAKEOFF. The creep is resized
        to sweep the newcomer with the carriers, and there is no weld transient to slew
        or hold: at the hand-over its rod is at the creep elevation like the others."""
        if d in self.slot2drone:
            return                                            # already on the load
        if start_creep and self.creep.sweep_started():
            self.get_logger().warn(f'[dissipative] start weld of drone {d} not folded into '
                                   f'the creep: the sweep has already started')
            return
        j = self._reserved_index(d)
        measured = self.attach_pos[j] if j is not None else (
            self.drone_pos[d] if d < len(self.drone_pos) else None)
        if measured is None or self.load_state is None:
            self.get_logger().warn(f'[dissipative] attach drone {d}: no mocap yet')
            return
        # the OCP's attach point for the newcomer is where its rod actually pivots: the
        # welded TIP (r up to 0.10 outside the ring, its true height), not the rim point
        # (R0547: the 7 cm between them made the planned 45-deg swing unreachable and
        # the rod yanked the newcomer over 3 s after the weld)
        rho_new = self._capture_weld_rho(measured, d, clamp=False, keep_z=True)
        if self._start_attached and not getattr(self, 'takeoff_seen', False):
            # welded on the ground with its rod laid out like a carrier's: the body-minus-
            # arm estimate would put the weld 0.4 m under the ring. Use the plate.
            R = quat_to_rot_np(self.load_state[3:7])
            rel = R.T @ (np.asarray(measured, float) - np.asarray(self.load_state[0:3], float))
            a = float(np.arctan2(rel[1], rel[0]))
            rho_new = np.array([self.attach_radius * np.cos(a),
                                self.attach_radius * np.sin(a), float(self.attach_z)])
            self.get_logger().info(
                f'[dissipative] drone {d} welded at start: plate at {np.degrees(a):.0f} deg')
        r_xy = float(np.hypot(rho_new[0], rho_new[1]))
        if r_xy > self.attach_radius + 0.10:
            rho_new[:2] *= (self.attach_radius + 0.10) / r_xy
        rho_new = np.array([float(rho_new[0]), float(rho_new[1]), float(rho_new[2])])
        rho_all = [np.asarray(self.rho[sl], float) for sl in range(len(self.slot2drone))]
        rho_all.append(rho_new)
        lens = [float(v) for v in self.cable_len_i[:len(self.slot2drone)]]
        lens.append(float(self._attach_cable_len) if j is not None else float(self.cable_len))
        ids = list(self.slot2drone) + [int(d)]
        t_before = list(self.refs._t_nom)                    # incumbents' shares pre-resize
        if not self.resize_fleet(len(ids), ids, rho=rho_all, cable_lengths=lens):
            self.get_logger().error(
                f'[dissipative] attach {d} ABORTED: no OCP for n={len(ids)}; the newcomer '
                f'keeps its hold reference')
            return
        # the newcomer's nominal cable must point at the incumbents' common apex, or the
        # static balance gives it no load (see geometry.apex_direction)
        self._s_nom[-1] = apex_direction(self.rho[:-1], self._s_nom[:-1], self.rho[-1])
        self.refs._s_nom = self._s_nom
        self.refs._t_nom = balanced_tensions(self.dyn.rho, self._s_nom, self.dyn.m)
        # the parent reads every slot from drone_pos: give the newcomer a slot there
        while len(self.drone_pos) <= d:
            self.drone_pos.append(None)
            self.drone_vel.append(np.zeros(3))
        self.drone_pos[d] = np.asarray(measured, float).copy()
        self.drone_vel[d] = np.zeros(3)
        self._ocp_attached[d] = True
        if getattr(self, '_partner_contact', False):
            self._drop_departed(d)                            # the OCP is its one reference from now on
        self._set_on_load(d, True)
        if j is not None:
            self.attach_pending[j] = False
            getattr(self, '_warm_latched', set()).discard(j)
            self._approach.pop(j, None)
        self.detached[d] = False
        self._weld_time[d] = self.get_clock().now()          # cable-FF ramp for the newcomer
        if start_creep:
            self.creep.resize(self.n, self.rho, self.cable_len_i[:self.n])
            self.get_logger().warn(
                f'[dissipative] ATTACH drone {d} (start weld, joins the creep as carrier '
                f'{self.n}): n={self.n}, '
                f'plate at {np.degrees(np.arctan2(rho_new[1], rho_new[0])):.0f} deg, '
                f'rods {[round(v, 2) for v in lens]} m, '
                f'tensions {[round(v, 2) for v in self.refs._t_nom]}')
            return
        # slew the cable references from the weld state: the newcomer's rod as measured
        # (drone -> rim, ~vertical) carrying nothing, the incumbents at their pre-resize
        # shares, to the new balanced split over the hold (R0541: the immediate 45-deg
        # re-plan rolled the Gazebo newcomer over 2 s after the weld)
        blend_s = self._attach_traj_hold_s if self._attach_traj_hold_s > 0.0 else self._reconfig_hold_s
        R = quat_to_rot_np(self.load_state[3:7])
        rim = np.asarray(self.load_state[0:3], float) + R @ rho_new
        dvec = rim - np.asarray(measured, float)
        s_new = dvec / max(float(np.linalg.norm(dvec)), 1e-6)
        s_from = list(self._s_nom[:-1]) + [rot_z(-self.psi0) @ s_new]
        # the newcomer's start tension acts off the incumbents' apex: a moment on the ring
        t_new0 = self._attach_t_start_new
        if (getattr(self, '_partner_contact', False) and j is not None
                and getattr(self, '_contact_fold_seed', 'start') == 'ocp'):
            # his lift proof has QUAD4 carrying part of the ring already. Its measured pull exists only inside its
            # tracker (cable_source mocap), so it starts at the share the n+1 OCP gives it
            t_new0 = float(self.refs._t_nom[-1])
            self.get_logger().info(f'[dissipative] partner_contact: {_who(d)} starts at the OCP share '
                                   f'{t_new0:.2f} N (not {self._attach_t_start_new:.2f})')
        t_from = t_before + [t_new0]
        if self._attach_datum_shift and self.load_state is not None:
            # BUMPLESS in xy, as the network handover: the approach hold froze the orbit
            # reference while the ring coasted on (0.13 m off at the weld, R0669) and the
            # fresh n+1 plan pulled it back in 0.7 s (32 deg). Carry the offset instead.
            off = np.asarray(self.load_state[0:2], float) - self._current_load_des()[:2]
            if float(np.linalg.norm(off)) > 1e-3:
                self.hover_xy = (float(self.hover_xy[0] + off[0]), float(self.hover_xy[1] + off[1]))
                self.get_logger().info(
                    f'[dissipative] attach datum shifted ({off[0]:+.3f},{off[1]:+.3f}) m onto the measured load')
        self.refs.balanced_blend = self._attach_blend_balanced
        self.refs.start_blend(s_from, t_from, blend_s)
        for _ in range(2):
            self._prime_solver()
        self._log_resize_inputs('attach', d, t_before, s_from, t_from)
        self._resize_log_ticks = 3
        if self.solver.last_X is None:
            self.get_logger().error(
                f'[dissipative] attach {d}: the n={self.n} OCP did not converge on the '
                f'current state. The fleet is resized but the first solves may jump.')
        gap = _largest_gap_deg(self.rho)
        az = float(np.degrees(np.arctan2(rho_new[1], rho_new[0])))
        self._approach_hold_armed = False
        if self._attach_traj_hold_s > 0.0 and self._hold_mode == 'settle':
            self._reconfig_hold_left = self._hold_max_s
            self._settle_hold, self._settle_for, self._settle_elapsed = True, 0.0, 0.0
            self._settle_prev_tilt = None
            hold = f'held until the load settles (cap {self._hold_max_s:.0f} s)'
        else:
            self._reconfig_hold_left = (self._reconfig_hold_s if self._attach_moving
                                        else max(self._attach_traj_hold_s, self._reconfig_hold_s))
            hold = f'held {self._reconfig_hold_left:.1f} s'
        self.get_logger().warn(
            f'[dissipative] ATTACH drone {d} (OCP resize): n={self.n}, weld at azimuth '
            f'{az:.0f} deg (r={float(np.hypot(rho_new[0], rho_new[1])):.2f}), rod {lens[-1]:.2f} m, '
            f'tensions {[round(v, 2) for v in self.refs._t_nom]}; '
            f'trajectory {hold}; ring gap {gap:.0f} deg')

    def _net_slot2drone(self):
        """Network slot -> physical drone for ALL n_net nodes: the parent's (possibly
        azimuth-reassigned) tethered mapping, then identity for the reserved drones."""
        return list(self.slot2drone) + list(range(self._n_carry0, self.n_net))

    def _drone_to_net_slot(self, d):
        s2d = self._net_slot2drone()
        return s2d.index(d) if d in s2d else None

    def _net_pos(self, slot):
        """Measured position of the drone in network slot `slot`: drone_pos for a tethered
        slot, attach_pos for a reserved one. None if that mocap has not arrived yet."""
        d = self._net_slot2drone()[slot]
        return self.drone_pos[d] if d < self._n_carry0 else self.attach_pos[d - self._n_carry0]

    def _enter_network_phase(self):
        """planner -> network: seed the spring-damper network from the current (airborne,
        taut) measured drone positions so the handover is bumpless, and capture the hover
        target the network will hold. The OCP is no longer solved after this."""
        self.phase = 'network'
        # the network publishes its own cable term and never eases it: hand the tracker's
        # resting-ring gate back its old job (zero the pull once the ring is down)
        self._set_ff_active(False)
        # seed all n_net nodes bumplessly. Reserved (not-yet-welded) nodes are inert, so a
        # finite placeholder (their mocap if present, else the load position) is enough --
        # attach() overwrites it with the measured pose at the weld.
        load_p = self.load_state[0:3] if self.load_state is not None else np.zeros(3)
        seeds = []
        for i in range(self.n_net):
            p = self._net_pos(i)
            seeds.append(load_p if p is None else p)
        self.net.seed(seeds)
        self.net.reset_load_trim()   # fresh trim per flight; not reset on attach/detach
        self._ho_ac = {i: v.copy() for i, v in self._last_ocp_ac.items()}
        self._ho_t0 = self.get_clock().now()
        self._net_p_des = self._current_load_des()
        # BUMPLESS in xy: the OCP hover parks the load with a standing offset from its
        # target (19 cm at 3/12/9 o'clock, R0191), and handing the network the target
        # instead of the load yanks the whole fleet at the worst moment (weld: tilt
        # 24 -> 61 deg in 1.6 s). Shift the trajectory datum so the network's first
        # target IS the measured load; the offset is carried, not corrected in a step.
        if self.load_state is not None:
            off = np.asarray(self.load_state[0:2], float) - np.asarray(self._net_p_des[:2], float)
            if float(np.linalg.norm(off)) > 1e-3:
                self.hover_xy = (float(self.hover_xy[0] + off[0]), float(self.hover_xy[1] + off[1]))
                self._net_p_des = self._current_load_des()
                self.get_logger().info(
                    f'[dissipative] handover datum shifted ({off[0]:+.3f},{off[1]:+.3f}) m so the '
                    f'network starts on the measured load (bumpless)')
        self._net_hold_z = float(self._net_p_des[2])   # hold this height; traj_t keeps running
        self.phase_pub.publish(String(data='network'))
        self.get_logger().info(
            '[dissipative] handover OCP -> dissipative network (airborne)')

    def _fleet_command_cb(self, msg):
        """In the network phase the inherited OCP LAND path is inert (its _plan is
        bypassed), so LAND is handled here: descend the held load target to the floor
        (see _network_plan) and announce /fleet/landed for the fleet manager to disarm.
        Every other command, and LAND before handover, falls through to the base handler."""
        self._newcomers_out_on_land(msg.data)
        self._departed_flight_start(msg.data)
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
        cmd = msg.data.strip().upper()
        if self._dropped is not None and cmd == 'LAND':
            if self._landed:
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info('[dissipative] LAND after a drop - re-announcing '
                                       '/fleet/landed')
            else:
                self.get_logger().info('[dissipative] LAND during a drop: every drone is '
                                       'already landing')
            return
        if cmd == 'ARM':
            self._arm_magnets()
            if self._dropped is None:
                self._dropped_pub.publish(Bool(data=False))     # a re-ARM starts undropped
        self._unload_cancel_on(msg.data)
        super()._fleet_command_cb(msg)

    def _departed_flight_start(self, cmd):
        """ARM and TAKEOFF: every departed flag False; TAKEOFF records each drone's resting height."""
        if not self._departed_gi or cmd.strip().upper() not in ('ARM', 'TAKEOFF'):
            return
        for pub in self._departed_pubs.values():
            pub.publish(Bool(data=False))
        self._departed_sent = set()
        self._high_stall_said = set()
        self._at_rest_t, self._press_t = {}, {}
        if cmd.strip().upper() == 'ARM':
            self._rest_z = {}
            return
        missing = []
        for dd in range(self.n_net):
            # the last pose seen, not the staleness-filtered one: before TAKEOFF it sits on the floor, so an old
            # pose is still its rest (R1270: a planner busy at startup had a 0.5 s-stale pose and recorded none)
            j = self._reserved_index(dd)
            pos = (self.attach_pos[j] if j is not None
                   else (self.drone_pos[dd] if dd < len(self.drone_pos) else None))
            if pos is not None:
                self._rest_z[dd] = float(pos[2])
            else:
                missing.append(dd)
        self.get_logger().info('[dissipative] resting heights at TAKEOFF (m): ' + ', '.join(
            f'{_who(dd)} {z:.3f}' for dd, z in sorted(self._rest_z.items())))
        for dd in missing:
            self.get_logger().warn(f'[dissipative] no pose for {_who(dd)} at TAKEOFF: if it departs it lands as '
                                   f'before (no departed ground idle)')

    def _newcomers_out_on_land(self, cmd):
        """LAND lands every drone (Wesley 8 Oct, M1 rule a): before the fleet descends, every unwelded
        newcomer is taken out of the flight and lands by itself, clear of the ring."""
        if cmd.strip().upper() == 'ARM' and not getattr(self, 'takeoff_seen', False):
            self._newcomer_out.clear()          # a refused ARM and a re-ARM: a fresh flight
            return
        if not self._newcomer_lifecycle:
            return
        if cmd.strip().upper() != 'LAND' or not getattr(self, 'takeoff_seen', False) \
                or self.phase != 'planner' or self._land_to_ground or self._dropped is not None:
            return
        for j in range(self.reserved_attach):
            if self.attach_pending[j]:
                if getattr(self, '_partner_contact', False):
                    self._take_back(j, 'LAND')     # his stream flies it: our approach never started
                else:
                    self._abort_newcomer(j, 'LAND')

    def _unload_cancel_on(self, cmd):
        """A LAND, DISARM, ESTOP or ABORT during a soft detach cancels it (magnet stays ON)."""
        cmd = cmd.strip().upper()
        if self._unload is not None and cmd in ('LAND', 'DISARM', 'ESTOP', 'ABORT'):
            self._cancel_unload(f'{cmd} received')

    def _arm_magnets(self):
        """The radio latches the last magnet value across flights: re-arm every tether
        at ARM so a drone released last flight does not lift without the load. Tethers
        only (_magnet_pubs): a newcomer that must stay OFF is never turned on here."""
        if not self._detach_magnet:
            return
        for pub in self._magnet_pubs:
            pub.publish(String(data='ON'))
        self.get_logger().info(f'[dissipative] ARM: magnets ON on tethers '
                               f'0..{len(self._magnet_pubs) - 1}')

    def _current_load_des(self):
        """The load position the network holds after handover: the OCP's current lift
        target (captured hover xy + trajectory offset + lifted height)."""
        z = float(self.load_state[2]) if self.lift_z0 is None else \
            min(self.target_z, self.lift_z0 + self.lift_progress)
        dx, dy, _, _ = self.traj.offset_at(self.traj_t)
        return np.array([self.hover_xy[0] + dx, self.hover_xy[1] + dy, z])

    def _settle_dwell_tick(self):
        """Settle-mode hold: end it once the tilt is low and quiet for hold_settle_s."""
        dt = 1.0 / PLANNER_HZ
        self._settle_elapsed += dt
        R = quat_to_rot_np(self.load_state[3:7])
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        rate = (abs(tilt - self._settle_prev_tilt) / dt
                if self._settle_prev_tilt is not None else float('inf'))
        self._settle_prev_tilt = tilt
        quiet = tilt < self._hold_resume_tilt and rate < self._hold_resume_rate
        self._settle_for = self._settle_for + dt if quiet else 0.0
        done = self._settle_for >= self._hold_settle_s
        if done or self._reconfig_hold_left <= 0.0:
            self._reconfig_hold_left = 0.0
            self._settle_hold = False
            self.get_logger().info(
                f'[dissipative] settle hold over after {self._settle_elapsed:.1f} s '
                f'({"tilt settled at " + format(tilt, ".1f") + " deg" if done else "cap reached"}); '
                f'trajectory resumes at t={self.traj_t:.2f} s')

    # ── network flight: hold the reduced fleet + fly the detached drones away ─
    def _network_plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

        # Keep any LATERAL trajectory running after handover: advance the trajectory clock
        # and take the target xy from it (frozen only during LAND). Without this the load
        # trajectory would stop the instant a drone detached.
        if self._reconfig_hold_left > 0.0:
            # attach / resize hold: the target stays where it is (see _do_attach)
            self._reconfig_hold_left -= 1.0 / PLANNER_HZ
            if self._settle_hold:
                self._settle_dwell_tick()
        elif not self._net_landing:
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
        # Freeze (not zero) the common-mode load trim during LAND/touchdown: the target
        # is ramping to the floor there, and integrating the descent error would wind
        # the trim against a motion that is commanded, not a deficit.
        self.net.step(load_pos, load_quat, load_vel, p_des, 1.0 / PLANNER_HZ,
                      load_accel=a_des,
                      integrate_trim=not (self._net_landing or self._landed))
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
            if drone >= self._n_carry0 and self.attach_pending[drone - self._n_carry0]:
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

    def _publish_status(self):
        """One line the operator can read off RViz: phase, what is holding the trajectory,
        how many drones carry the load, and the load tilt -- the attach signature that
        used to be found in logs after the fact."""
        tilt = float('nan')
        if self.load_state is not None:
            R = quat_to_rot_np(self.load_state[3:7])
            tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        hold = ''
        if self._reconfig_hold_left > 0.0:
            hold = (' | HOLD (approach)' if getattr(self, '_approach_hold_armed', False)
                    else f' | HOLD {self._reconfig_hold_left:.0f}s')
        n_on = self.net.n_attached() if self.phase == 'network' else self.n
        self.status_pub.publish(String(
            data=f'{self.phase.upper()}{hold} | {n_on} on load | tilt {tilt:.0f} deg'))

    def _publish_ref(self, i, nodes):
        """Wraps the parent's publisher: remembers the OCP's last cable feedforward per
        drone, and blends an incumbent's network feedforward from it after the handover
        (see handover_blend_s). Wire format untouched."""
        if self.phase != 'network':
            self._last_ocp_ac[i] = np.asarray(nodes[0][3], float).copy()
            if i in self._weld_time and self._ocp_attached.get(i, False):
                gate = self._attach_ff_gate(i)
                if gate < 1.0:
                    nodes = [(p_, v_, a_, np.asarray(ac_, float) * gate)
                             for p_, v_, a_, ac_ in nodes]
                if self._attach_velocity_newcomer:
                    nodes = [(p_, v_, np.asarray(a_, float) - np.asarray(ac_, float), ac_)
                             for p_, v_, a_, ac_ in nodes]
        elif (self._handover_blend_s > 0.0 and self._ho_t0 is not None
              and i < self.n and i in self._ho_ac and not self.detached[i]):
            el = (self.get_clock().now() - self._ho_t0).nanoseconds * 1e-9
            sblend = float(np.clip(el / self._handover_blend_s, 0.0, 1.0))
            if sblend < 1.0:
                ac0 = self._ho_ac[i]
                blended = []
                for p_, v_, a_, ac_ in nodes:
                    ac_net = np.asarray(ac_, float)
                    ac_b = (1.0 - sblend) * ac0 + sblend * ac_net
                    # a_ff = g + a_des - a_cable, so shift a_ff by the same amount
                    a_b = np.asarray(a_, float) + (ac_net - ac_b)
                    blended.append((p_, v_, a_b, ac_b))
                nodes = blended
        super()._publish_ref(i, nodes)

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
            if drone >= self._n_carry0 and self.attach_pending[drone - self._n_carry0]:
                continue
            pos = self._net_pos(i)
            if pos is None:
                continue
            gate = 1.0 if drone >= self._n_carry0 else self._cable_taut_gate(i)[0]
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
