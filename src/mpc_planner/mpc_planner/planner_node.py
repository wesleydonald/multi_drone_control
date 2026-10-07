"""
planner_node.py
---------------
Centralized planner for a cable-suspended load. Runs at PLANNER_HZ and publishes
a reference trajectory per drone on /drone_{i}/reference_trajectory, consumed by
the per-drone trackers (package tracker).

Wire format (Float64MultiArray), 12 fields per node, world frame:
    [n_nodes, dt, px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, ...]
    a_i  required specific thrust acceleration (f_i/m_i): magnitude sets the
         tracker throttle, direction sets its desired yaw-free attitude.
    c_i  cable tension acceleration t_i*s_i/m_i, added to the tracker's
         prediction model so its feedback stops fighting the cable.

Solves the load-cable OCP (planner_ocp.py) online, pinning node 0 to the measured
state. This is the paper's method (Sun et al. 2025). x_init is built from mocap --
load pose/twist, cable directions s_i AND cable angular velocities r_i are all
measured; only the unobservable higher cable states (rd_i, rdd_i) and tensions
(t_i, td_i) are resampled from the previous solution (paper Fig 8). The fleet
creeps up from the ground (start_taut=false, handover_elev_deg>0), sweeping the
rigid rods up until the cables are taut, then the OCP takes over; an elevated
start_taut world skips the creep and hands over immediately.

Geometry must match the world SDF (see generate_rigid_world.py).
"""
import os
import re
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Float64MultiArray, Int32, String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interfaces.msg import ELRSCommand, MotionCaptureState, Telemetry

from .load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from .geometry import (quat_to_rot_np, attach_points, nominal_cable_dirs,
                       azimuth_slot_assignment, yaw_from_quat, slot_azimuth_errors,
                       slot_offset_warnings, rot_z)
from .load_trajectory import LoadTrajectory, ORBIT_RAMP_S
from .planner_solver import PlannerSolver, HorizonFallback, PLAN_N, PLAN_TF, rod_accel_at
from .params import PlannerConfig, load_inertia
from .creep_controller import CreepController
from .reference_builder import ReferenceBuilder
from .disturbance_estimator import DisturbanceEstimator
from .offset_free import OffsetFreeObserver
from .lumped_integral import LumpedForceIntegral
from utility_objects.data_logger import run_log_dir, write_params, node_params
from utility_objects.run_context import log_base_dir

# Physical constants baked into the load-cable model (not ROS params). Geometry and
# mode params (num_drones, cable_len, load_mass, ...) live in params.PlannerConfig.
# Sized for the original 0.4 kg payload. Inertia scales with mass for a body of
# fixed geometry, so it does NOT track the load_mass ROS param -- dropping
# load_mass to 0.1 without scaling these leaves the model ~4x over-stiff in
# rotation. Scale by (load_mass / 0.4) for a same-size lighter payload, or
# recompute from the real payload's dimensions.
from .params import DRONE_MASS   # sim airframe 0.64 kg; the node reads the drone_mass param
PLANNER_HZ    = 10.0
Z_KI_ORBIT_MAX_ACC = 0.05  # m/s^2 centripetal: M1 orbits at 0.031; 0.4 m/s circles sag 15 cm (R0484)
LAND_BLEND_S  = 1.0     # s, flight feedforward -> hover when the load touches down
LAND_FOLLOW_XY_Z = 0.25  # m, below this a landing drone's xy reference follows its measured xy

# logs/<node name>/<ts>/: logs/mpc_planner/<ts> or logs/dissipative_planner/<ts>, beside
# logs/tracker/drone<i>_<ts> (tools/run_logs.py reads both this and the pre-3 Oct layout).

# Lift-ramp easing: shape the rate 0 -> lift_ramp_vel -> 0 rather than stepping,
# since the velocity feedforward is passed straight through and a step there is
# taken by the drones as a jolt. For a 0.475 m climb at 0.20 m/s this cuts peak
# accel 2.00 -> 0.59 m/s^2 and arrives at ~0.01 m/s, costing ~1.5 s.
LIFT_SOFT_S   = 1.2           # s to ease in
LIFT_SOFT_D   = 0.08          # m to ease out over
LIFT_SOFT_MIN = 0.12          # floor on the shape factor, so the climb actually
                              # terminates instead of asymptoting at the target

# LAND ends on touchdown (drones stop following the descending reference), not at
# an absolute height: the fleet may be over a takeoff platform rather than open
# ground, making any fixed threshold unreachable. See _touchdown_stalled.
LAND_STALL_FRAC = 0.25        # descent slower than this fraction of commanded
                              # counts as stalled
LAND_STALL_S  = 1.0           # s the stall must persist
LAND_MIN_DESCENT = 0.05       # m the fleet must actually drop first
LAND_GRACE_S  = 1.5           # s to ignore the stall test after LAND, while the
                              # drones are still catching up to the new reference
LAND_MAX_DROP = 1.50          # m safety floor below the handover height

# Cable-FF soft-start. The payload sits on the GROUND through the handover settle,
# so stepping the full tension feedforward on at handover makes the drones lurch to
# absorb a pull that isn't real yet (and the OCP then tracks that thrash). Ease the
# published a_cable in over FF_EASE_S once the LIFT starts -- clocked on the
# planner's own lift schedule, not the measured load height, so unlike the old
# airborne height-gate it can never deadlock (the lift clock advances regardless).
FF_EASE_S = 1.0
PRETENSION_HOLD_S = 0.5         # full pull held this long before the height ramp starts
REFUSE_BELOW_DEG = 30.0         # creep timeout below this: rods too flat to carry the ring
# pretension breakaway: the ring is off the floor once it is this far above its rest height or
# rising this fast; the pull is then frozen at the fraction reached (rig 2026-09-30 lift f6: the
# ring broke free at ~2/3 of the planned pull and the ramp to full ran it up at 0.38 m/s)
BREAKAWAY_DZ = 0.01
BREAKAWAY_VZ = 0.03

LAND_DEPARTED_WAIT_S = 15.0   # s to wait for a departed (detached) drone to come
                              # down before /fleet/landed disarms the whole fleet


class ZBias:
    """Bounded load-height integral on the planner's height target (card
    2026-09-24_planner_offset). Node 0 of the OCP is re-pinned to the measured state
    every cycle, so the load height is held only by the trackers' weak position
    stiffness against the node-0 force budget: any static error (feedforward at the
    wrong elevation, rods, mass, gain) becomes a fixed height miss with DC gain 1 on the
    target (R0473-R0477). This integrates the measured miss into the reference height,
    only while the caller says the fleet is in a gated hover; frozen otherwise, zeroed
    when the planner phase is entered. `near_bound` is the fault-masking guard: a real
    fault rails it and the node warns."""

    def __init__(self, ki, i_max, hz):
        self.ki = float(ki)
        self.i_max = abs(float(i_max))
        self.hz = float(hz)
        self.reset()

    def reset(self):
        self.value = 0.0
        self.n_updates = 0

    @property
    def near_bound(self):
        return abs(self.value) > 0.7 * self.i_max

    def update(self, err, gated):
        """err = target_z - measured load z. Returns the bias to ADD to the reference."""
        if gated and self.ki > 0.0:
            self.value += self.ki * float(err) / self.hz
            self.value = float(np.clip(self.value, -self.i_max, self.i_max))
            self.n_updates += 1
        return self.value


class TouchdownDetector:
    """Touchdown by STALL of the drones a LAND descent is driving down.

    The reference descends at land_vel; a drone no longer following it has hit
    something solid, which is the only landing test that holds when the fleet is not
    above its takeoff points. Armed only after LAND_GRACE_S and a real descent of
    LAND_MIN_DESCENT (a tracker lagging longer than the grace would otherwise read
    "not moving yet" as landed at altitude); the stall must persist LAND_STALL_S so
    tracking lag or a swinging load does not read as touchdown. Pure so it can be unit
    tested; fed ONLY the drones still on the load (R0113/R0114: a detached drone still
    descending kept the max moving and the survivors were ground into the floor)."""

    def __init__(self, land_vel, hz):
        self.land_vel = float(land_vel)
        self.hz = float(hz)
        self.reset()

    def reset(self):
        self._start_z = None
        self._prev_max_z = None
        self._stall_ct = 0
        self._cycles = 0

    def update(self, zs):
        """One planner tick with the heights of the drones being landed. True once
        they have all stopped descending."""
        if not zs or any(z is None for z in zs):
            return False
        max_dz = max(float(z) for z in zs)
        if self._start_z is None:
            self._start_z = max_dz
        self._cycles += 1
        if (self._cycles < int(LAND_GRACE_S * self.hz)
                or max_dz > self._start_z - LAND_MIN_DESCENT):
            self._prev_max_z = max_dz
            return False
        prev = self._prev_max_z
        self._prev_max_z = max_dz
        if prev is None:
            return False
        expected = self.land_vel / self.hz      # drop per tick if tracking the reference
        if (prev - max_dz) < expected * LAND_STALL_FRAC:
            self._stall_ct += 1
        else:
            self._stall_ct = 0
        return self._stall_ct >= int(LAND_STALL_S * self.hz)


def land_arc_ref(attach, radial, rod, theta0, centre_off, s, v):
    """(drone-centre position, velocity) after `s` metres of LAND unwind at speed v: the rod
    pivot swings down its circle about the grounded attach point from elevation theta0 to
    level (the creep in reverse), then descends straight down; never below z 0."""
    attach = np.asarray(attach, float)
    radial = np.asarray(radial, float)
    arc = max(float(theta0), 0.0) * rod
    if s < arc:
        th = float(theta0) - s / rod
        pivot = attach + rod * (np.cos(th) * radial + np.array([0.0, 0.0, np.sin(th)]))
        vel = v * (np.sin(th) * radial - np.array([0.0, 0.0, np.cos(th)]))
    else:
        pivot = attach + rod * radial - np.array([0.0, 0.0, s - arc])
        vel = np.array([0.0, 0.0, -v])
    pos = pivot + np.asarray(centre_off, float)
    if pos[2] <= 0.0:
        pos[2], vel[2] = 0.0, 0.0
    return pos, vel


LIFT_STEP     = 0.04          # m max commanded climb above current load z. Kept
                              # small: a large lead builds climb speed and
                              # overshoots the taut transition, spiking tension
                              # past the drones' thrust authority.

# Cable tautness gate. Cables spawn slack, so feeding predicted tension while one
# is loose makes the tracker over-estimate the pull and lurch. Scale the published
# cable acceleration by how taut the cable measures right now (drone->attach
# distance vs cable length). Raise LO_FRAC toward 1.0 to feed tension in later.
CABLE_TAUT_LO_FRAC = 0.85
CABLE_TAUT_HI_FRAC = 1.00


def static_cable_z(planned_a_z, load_mass, drone_mass):
    """Per-drone STATIC vertical cable acceleration (m/s^2, negative = pull down) from the
    OCP's planned node-0 cable terms: keep the planned DISTRIBUTION across drones, but
    normalise the vertical sum to the load's weight. The planned magnitudes carry the
    planner's climb/descend intent (a load 3 cm high is planned to descend, so planned
    tension < weight); an estimator that took them as the true pull drifted (R0378:
    kt_hat 36.75 -> 36.29 while the load rose 0.61 -> 0.67 m, 2026-09-23). Statics do
    not. Returns None when the planned pull is not there (slack, grounded)."""
    a = [float(v) for v in planned_a_z]
    tot = sum(a)
    if not a or tot > -0.5:
        return None
    return [v / tot * (-float(load_mass) * 9.81 / float(drone_mass)) for v in a]


def tick_log_header(n_drones, n_slots):
    """Columns of the planner's log.csv. The first block is the original layout; later
    columns are only ever appended, so readers by name or by position keep working.
    Rod columns (t tsz gate len elev, then per rod) are per OCP SLOT (slot2drone maps
    them to drones); d<k>_* are per physical drone. t is the node-0 planned tension of
    the published horizon, len the pivot-to-attach distance, ref_age the time since the
    last successful solve. published is what an OCP tick sent (solved / shifted / none;
    blank on creep and hold ticks, whose solve_status is the priming solve), res_* the
    residuals of the last solve."""
    cols = (['sim_time', 'phase', 'n', 'load_x', 'load_y', 'load_z',
             'load_vz', 'tilt_deg', 'z_tgt', 'z_bias', 'lift_progress',
             'traj_t', 'land'] + [f'd{k}_z' for k in range(n_drones)])
    cols += ['solve_status', 'solve_ms', 'qp_iter', 'sqp_calls']
    for name in ('t', 'tsz', 'gate', 'len', 'elev'):
        cols += [f'{name}{i}' for i in range(n_slots)]
    cols += ['ff', 'ff_active', 'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz',
             'ref_age', 'fail_streak', 'slot2drone']
    cols += [f'd{k}_{a}' for k in range(n_drones) for a in ('x', 'y')]
    cols += [f'd{k}_{a}' for k in range(n_drones) for a in ('qw', 'qx', 'qy', 'qz')]
    cols += ['published', 'res_stat', 'res_eq']
    # int_mode model (card 2026-10-04_z_int_model): the integral, its force, and the published
    # plan's ring position at node 5 (what the plan predicted for 5 nodes on)
    cols += ['int_bx', 'int_by', 'int_bz', 'int_fx', 'int_fy', 'int_fz', 'plan5_x', 'plan5_y', 'plan5_z']
    return cols


def measured_rod_lengths(nominal, dists, tol_frac=0.15, spread_m=0.03):
    """Per-drone rod lengths from the measured drone-to-rim distances at the moment the
    rods are known taut (creep handover / taut air start). Returns (lengths, None) or
    (None, reason) when the measurement is not trusted: any rod outside +-tol_frac of the
    typed cable_len, or the rods disagreeing by more than spread_m (equal rods on this rig).
    Why: a typed length 4 cm off costs ~20 cm of hover (SIL R0286/R0287, 2026-09-23) --
    the tautness gate ramps the tension feedforward on the typed value and the OCP's
    references are placed at it. Measured, both errors disappear."""
    nominal = float(nominal); d = [float(v) for v in dists]
    if not d:
        return None, 'no drones'
    for i, v in enumerate(d):
        if abs(v - nominal) > tol_frac * nominal:
            return None, f'rod {i} measured {v:.3f} m vs typed {nominal:.3f} (outside +-{tol_frac * 100:.0f}%)'
    if max(d) - min(d) > spread_m:
        return None, f'rods disagree by {(max(d) - min(d)) * 100:.1f} cm (> {spread_m * 100:.0f} cm)'
    return d, None


class LoadPlanner(Node):
    def __init__(self, node_name='mpc_planner'):
        super().__init__(node_name)

        # ROS params (see params.PlannerConfig). Copied onto self so the rest of the
        # planner reads plain self.<name>.
        cfg = PlannerConfig(self)
        cfg.log(self.get_logger())
        self.n = cfg.n
        self.cable_len = cfg.cable_len
        # Per-drone rod lengths the OCP and the tautness gate use. Typed value until
        # measure_rod_len replaces them with the taut measurement at handover.
        self.cable_len_i = [float(cfg.cable_len)] * cfg.n
        self.measure_rod_len = bool(self.declare_parameter('measure_rod_len', False).value)
        self.declare_parameter('measured_rod_len', [0.0] * cfg.n)   # read-back of what was applied
        self.attach_radius = cfg.attach_radius
        self.attach_z = cfg.attach_z
        self.load_mass = cfg.load_mass
        self.load_inertia = cfg.load_inertia
        self.handover_elev_deg = cfg.handover_elev_deg
        self.handover_settle_s = cfg.handover_settle_s
        self.start_taut = cfg.start_taut
        self.target_z = cfg.target_z
        self.lift_ramp_vel = cfg.lift_ramp_vel
        self._zbias = ZBias(cfg.z_ki, cfg.z_i_max, PLANNER_HZ)
        self._z_taut_gate = cfg.z_taut_gate
        self.pivot_offset = np.asarray(cfg.pivot_offset, float)   # rod pivot, drone body frame
        self._solve_budget_s = cfg.solve_budget_s
        self._fallback = HorizonFallback(cfg.max_shift_publishes, PLAN_TF / PLAN_N)
        self._published = ''                   # this tick's refs: solved / shifted / none
        # opt-in: keep integrating through a constant-speed level orbit (M1 orbits from the
        # end of the lift, so the gated hover never happens and the ring flies 6 cm high, R0653)
        self._z_ki_in_orbit = bool(self.declare_parameter('z_ki_in_orbit', True).value)
        # > 0: hover this long at the target before a load trajectory starts, so the ring
        # integral learns a steady push at rest and carries it into the path (x and y integrate
        # only at rest, and a trajectory starts at the lift: twin 6 Oct, R1088 -> R1091)
        self._traj_hold_s = float(self.declare_parameter('traj_hold_s', 0.0).value)
        self._traj_hold_left = self._traj_hold_s
        # The breakaway freeze holds the pull where the ring broke free, but only through the
        # pretension: once the lift ramp starts the cap returns to 1 over ff_cap_release_s.
        # Held for the whole flight (0 here) it told the trackers the rods pull 7-27 % less
        # than they do and left the rig ring 20-30 cm low (1 Oct; twin R0817/R0818).
        self._ff_release_s = float(self.declare_parameter('ff_cap_release_s', 2.0).value)
        # TEST ONLY: cap the pull at this fraction when the pretension ends, as a rig breakaway
        # at 73-93 % does, so the twin can stand in for the rig. 0 = off.
        self._ff_cap_force = float(self.declare_parameter('ff_cap_force', 0.0).value)
        # LAND after the ring is down: each drone follows its rod's circle out to the floor
        # (the creep in reverse) instead of descending straight down, which the rigid rod
        # turns into an 18-26 cm outward shove (every twin landing; 2 tipped, R0793/R0816).
        self._land_unwind = bool(self.declare_parameter('land_unwind', False).value)
        self._land_ff_ramp = bool(self.declare_parameter('land_ff_ramp', False).value)
        self._zbias_warned = False
        self._load_t = None                    # monotonic time of the last payload pose
        self.auto_slot_assign = cfg.auto_slot_assign
        self.load_traj = cfg.load_traj
        self.traj_speed = cfg.traj_speed
        self.traj_distance = cfg.traj_distance
        self.traj_radius = cfg.traj_radius
        self.land_vel = cfg.land_vel

        # Flight-sequence runtime state (not params).
        self._settle_left = 0.0
        self.traj_t = 0.0            # elapsed lateral-trajectory time (post-hover)
        self.descending = False      # latched once the lateral trajectory closes
        self._land_to_ground = False # LAND command: descend all the way to ground
        self._landed = False         # descent finished, load back at start height
        self._lift_vel = 0.0         # signed vertical velocity of the lift target
        self._traj_t_drawn = 0.0     # traj_t at the last RViz horizon publish
        # touchdown detector (see TouchdownDetector / _touchdown_stalled)
        self._touchdown = TouchdownDetector(self.land_vel, PLANNER_HZ)
        self._touched_down = False   # survivors stalled on the floor; reference frozen
        self._land_wait_ct = 0       # ticks spent waiting for departed drones to land
        self._land_ff_off_said = False
        self._land_anchor = None     # per-slot (x, y, z) latched when the load is down on LAND
        self._last_air_ref = {}      # physical drone -> (thrust accel, cable accel) of its last flight reference
        self._land_drop = 0.0

        # Lateral load-reference trajectory generator (line_x / circle / fig_8 /
        # spin). The trajectory CLOCK traj_t stays here and is advanced in _plan.
        self.traj = LoadTrajectory(self.load_traj, self.traj_speed,
                                   self.traj_distance, self.traj_radius)

        # Attach ring on the payload body + nominal 45 deg cable directions (the
        # flatness s_i reference at hover, tilted per node by the load accel in
        # _yref_at). Both derived from the fleet size and attach geometry.
        self.attach_azimuths = cfg.attach_azimuths
        self.rho = attach_points(self.n, self.attach_radius, self.attach_z,
                                 self.attach_azimuths)
        # nominal cable elevation of the hover (rig default 60 since 7 Oct). Steeper = less
        # tension per cable and, for the partner's X3, less pivot torque (T0008)
        self.cable_elev_deg = float(self.declare_parameter('cable_elev_deg', 45.0).value)
        self._s_nom = nominal_cable_dirs(self.rho, self.cable_elev_deg)
        if self.cable_elev_deg != 45.0:
            self.get_logger().info(f'[planner] nominal cable elevation {self.cable_elev_deg:.1f} deg')

        self.drone_mass = cfg.drone_mass
        # Option F (2026-10-03): rods with mass in the model. The twin books the 0.075 kg rod
        # on the drone; the force replay put ~1.6 N of it on the ring instead (M-dist-split).
        # rod_mass 0 = the massless cable; rod_com = centre of mass from the load end, m.
        self.rod_mass = float(self.declare_parameter('rod_mass', 0.0).value)
        self.rod_lam = (float(self.declare_parameter('rod_com', 0.0).value) / self.cable_len
                        if self.rod_mass > 0.0 else 0.0)
        self.dyn = LoadCableDynamics(
            self.n, self.load_mass, self.load_inertia, [self.cable_len] * self.n,
            self.rho, self.drone_mass, rod_mass=self.rod_mass, rod_lam=self.rod_lam)
        # The OCP wrapper builds (or loads a cached) acados solver for this geometry
        # and owns the reference-extraction functions + warm-start state (last_X).
        # node 0 also pins the measured rod rates (card 2026-10-04_pin_cable_rates)
        self.pin_cable_rates = bool(self.declare_parameter('pin_cable_rates', False).value)
        self.solver = PlannerSolver(self.dyn, self.get_logger(), pin_rates=self.pin_cable_rates)
        self._configure_solver(self.solver)
        # Pre-built OCPs by fleet size, for handing back after a reconfiguration.
        # Populated by prebuild_solvers(); the current size is registered here so a
        # hand-back to the original n is a swap like any other.
        self._solvers = {self.n: (self.dyn, self.solver, self.rho)}
        self.N = self.solver.N
        self.dt = self.solver.dt
        # Phase-1 takeoff (soft/arc creep + handover decision), owns its own creep
        # state. Fed the live measured state each tick and publishes the creep refs
        # through _publish_ref (the trackers see these until the OCP takes over).
        self.creep = CreepController(
            self.n, self.rho, self.cable_len, self.N, self.dt, self.dyn.g,
            self.handover_elev_deg, PLANNER_HZ, self._pivot_at, self._publish_slot_ref,
            self.get_logger(), creep_vel=cfg.creep_vel,
            pivot_offset=self.pivot_offset, drone_yaw=self._slot_yaw)
        # Builds the per-node OCP tracking reference from the lift schedule + load
        # trajectory (fed the schedule via refs.update() before each planner solve).
        self.refs = ReferenceBuilder(self.dyn, self.n, self._s_nom, self.dt, self.traj)

        # state
        self.load_state = None                 # [p(3), q(4 wxyz), v(3), w(3)]
        self.drone_pos = [None] * self.n
        self.drone_vel = [None] * self.n       # world velocity from mocap twist
        self.drone_quat = {}                   # physical drone -> attitude [w, x, y, z]
        # OCP slot i -> physical drone slot2drone[i]. Identity until (optionally)
        # reassigned by azimuth on the first solve (see _assign_slots).
        self.slot2drone = list(range(self.n))
        self._slots_assigned = False
        # Load yaw the rig was placed at, latched on the first planner tick
        # (_latch_yaw_datum). 0.0 until then, which is the old world-aligned
        # behaviour and is correct for a payload that really is at yaw 0.
        self.psi0 = 0.0
        self._yaw_datum_latched = False
        # NB: the OCP warm-start state (last_X / recover) lives on self.solver.
        self.hover_xy = None                   # captured load x,y for the reference
        self._ff_t = 0.0                       # cable-FF soft-start clock (see _plan)
        self.phase = 'creep'                   # 'creep' (slow rise to taut) -> 'planner'
        self.takeoff_seen = False              # gate the lift ramp on TAKEOFF (see below)
        self.lift_z0 = None                    # load height latched at handover
        self.lift_progress = 0.0               # ramped lift above lift_z0 (m)
        self._lift_t = 0.0                     # s since the lift ramp started

        # subs
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        # /fleet/step is 0 before TAKEOFF and increments once flying, so step > 0
        # is the cue that it is safe to start the lift ramp (see _plan).
        self.create_subscription(
            Int32, '/fleet/step',
            lambda msg: setattr(self, 'takeoff_seen',
                                self.takeoff_seen or msg.data > 0), 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)
        # /fleet/command (ARM|TAKEOFF|DISARM|ESTOP|LAND). Only LAND is acted on:
        # stop the lateral trajectory and descend. The central controller
        # disarms once we report done.
        self.create_subscription(
            String, '/fleet/command', self._fleet_command_cb, 10)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]
        # True once the LAND descent finishes, so the central controller disarms.
        self.landed_pub = self.create_publisher(Bool, '/fleet/landed', 1)
        # the trackers apply the cable feedforward on a resting ring only while this is
        # true (the planner eases it; the tracker must not block or step it)
        self._ff_active_pub = self.create_publisher(Bool, '/fleet/cable_ff_active', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._ff_active = None
        self.pretension_s = 0.0 if cfg.start_taut else max(0.0, cfg.pretension_s)
        self._z_i_gate = cfg.z_i_gate
        self.rod_tol_frac = cfg.rod_tol_frac
        self.rod_spread_m = cfg.rod_spread_m
        self._pretension_said = False
        self._lift_refused = False
        self._refuse_anchor = None
        self._ff_cap = 1.0              # pull fraction frozen at the pretension breakaway
        self._ff_rel_step = None        # cap release per tick, set when the release starts
        self._ff_forced = False         # test-only ff_cap_force applied this lift
        # The phase in words for the RViz panel, latched and sent only on change
        self._phase_text = None
        self._phase_pub = self.create_publisher(String, '/fleet/phase', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # pre-flight warnings on the fleet manager's latched status line (RViz panel)
        self._status_pub = self.create_publisher(String, '/fleet/manager_status', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._slot_offsets_checked = False
        # Desired load position [x, y, z] at node 0. Logged by the drone-0
        # tracker so plot_run.py can overlay payload desired vs actual.
        self._traj_state_pub = self.create_publisher(String, '/payload/trajectory_state', 5)
        self.load_ref_pub = self.create_publisher(
            Float64MultiArray, '/payload/desired_position', 5)
        # Same reference as a full horizon Path, for RViz alongside each drone's
        # /drone_N/mpc_plan.
        self.load_plan_pub = self.create_publisher(
            Path, '/payload/mpc_plan', 5)

        # sim time only: the SIL bench holds its clock until this marker says the tick's
        # refs are out, so the planner is inside the lockstep like the trackers
        self._tick_pub = (self.create_publisher(Float64MultiArray, '/planner/tick', 10)
                          if self.get_parameter('use_sim_time').value else None)
        self.create_timer(1.0 / PLANNER_HZ, self._plan_timer)
        self.create_timer(0.2, self._publish_phase)
        # Record the run's configuration. Written here AND re-written at the end of a
        # subclass's __init__ (see _dump_run_params), so the file always reflects the
        # full parameter set of whichever node actually flew.
        self._init_disturbance()
        self._init_offset_free()
        self._init_lumped()
        self._run_log_dir = None
        self._dump_run_params()
        self.get_logger().info('[planner] ready, waiting for mocap...')

    def _dump_run_params(self):
        """Write logs/<pkg>/<node>_<ts>/params.json with every declared parameter, so a
        run says what it was configured with instead of having to be reverse-engineered
        from the flown trajectory. Safe to call more than once -- the directory is made
        on the first call and the file is overwritten after that, which is how a
        subclass folds in the parameters it declares after super().__init__()."""
        try:
            if self._run_log_dir is None:
                # Absolute results root (see utility_objects.run_context) so the
                # planner's params.json sits beside the trackers' CSVs instead of
                # wherever the launch happened to be started from.
                self._run_log_dir = run_log_dir(self.get_name(), None,
                                                base_dir=log_base_dir())
                for entry in self._solvers.values():
                    self._configure_solver(entry[1])
            write_params(self._run_log_dir, node_params(self, {
                'node': self.get_name(), 'planner_hz': PLANNER_HZ,
                'num_drones': self.n, 'horizon_N': self.N, 'node_dt': self.dt,
                'phase_at_start': self.phase}))
        except Exception as e:                      # never break the flight for a log
            self.get_logger().warn(f'[planner] could not write params.json: {e}')

    def _init_disturbance(self):
        """Option A (card 2026-10-03_offset_free_mpc): constant external forces on the load
        (d_L) and on every drone (d_D) from measured motion and delivered thrust. 'shadow'
        only logs them; 'ring' plans with d_L (OCP and reference cone); 'full' also hands d_D
        to the trackers through the published acceleration and external-force term."""
        p = self.declare_parameter
        self._dist_mode = str(p('dist_est', 'off').value).lower()
        self._dist = None
        self._dist_applied = (np.zeros(3), np.zeros(3))
        if self._dist_mode not in ('off', 'shadow', 'ring', 'full'):
            raise ValueError(f"dist_est must be off, shadow, ring or full, got {self._dist_mode!r}")
        if self._dist_mode == 'off':
            return
        self._dist_rod = (float(p('dist_rod_mass', 0.075).value), float(p('dist_rod_com', 0.275).value))
        # the trackers' own thrust map: the estimate uses the thrust they believe they deliver
        self._dist_map = {k: float(p(k, v).value) for k, v in (
            ('thrust_ratio', 35.2), ('thrust_offset', 0.185), ('thrust_offset_v_slope', 0.022),
            ('thrust_v_ref', 23.5))}
        self._dist = DisturbanceEstimator(self.n, tau=float(p('dist_tau', 0.6).value))
        self._dist_cmd, self._dist_volt = {}, {}
        self._dist_log = None
        self._dist_said = 0.0
        for d in range(self.n):
            self._dist_subscribe(d)
        self.get_logger().warn(f'[planner] disturbance estimate {self._dist_mode}: rod '
                               f'{self._dist_rod[0]:.3f} kg, centre {self._dist_rod[1]:.3f} m from the '
                               f'load end, map kT {self._dist_map["thrust_ratio"]:.1f} above '
                               f'{self._dist_map["thrust_offset"]:.3f}')

    def _dist_subscribe(self, d):
        if d in self._dist_cmd:
            return
        self._dist_cmd[d] = None
        self.create_subscription(
            ELRSCommand, f'/drone_{d}/ELRSCommand',
            lambda m, k=d: self._dist_cmd.__setitem__(k, ((m.channel_2 + 1.0) / 2.0, bool(m.armed))), 10)
        self.create_subscription(
            Telemetry, f'/drone_{d}/telemetry',
            lambda m, k=d: self._dist_volt.__setitem__(k, float(m.battery_voltage)), 10)

    def _dist_thrust(self, d, q):
        """Thrust vector drone d delivered on its last command, through its tracker's map."""
        cmd = self._dist_cmd.get(d)
        if cmd is None or not cmd[1] or q is None:
            return None
        mp, v = self._dist_map, self._dist_volt.get(d)
        off = 0.0
        if mp['thrust_offset'] > 0.0:
            dv = (mp['thrust_v_ref'] - v) if (v is not None and 18.0 < v < 27.0) else 0.0
            off = max(0.0, mp['thrust_offset'] + mp['thrust_offset_v_slope'] * dv)
        mag = self.dyn.mi[0] * mp['thrust_ratio'] * max(0.0, cmd[0] - off)
        return mag * quat_to_rot_np(q)[:, 2]

    def _update_disturbance(self):
        """One estimator tick, then what the plan uses: the estimate in 'ring'/'full', zero
        with the load down. Gated to an airborne load in the planner phase, not landing;
        frozen otherwise (a pause in the LAND reference reads as touchdown, R0852)."""
        if self._dist is None or self.load_state is None:
            return
        ls = self.load_state
        gated = (self.phase == 'planner' and self._settle_left <= 0.0 and not self._lift_refused
                 and not self.descending and not self._land_to_ground and self.lift_z0 is not None
                 and float(ls[2]) > self.lift_z0 + 0.05 and self._load_t is not None
                 and time.monotonic() - self._load_t < 0.2)
        R = quat_to_rot_np(ls[3:7])
        s_dirs, thrust, v_dr = [], [], []
        for i in range(self.n):
            d = self.slot2drone[i]
            piv = self._pivot_at(i)
            f = self._dist_thrust(d, self.drone_quat.get(d))
            if piv is None or f is None or self.drone_vel[d] is None:
                self._dist._prev = None              # a gap: no acceleration across it
                break
            s = np.asarray(ls[0:3], float) + R @ self.rho[i] - piv
            s_dirs.append(s / np.linalg.norm(s))
            thrust.append(f)
            v_dr.append(self.drone_vel[d])
        else:
            m_r, com = self._dist_rod
            self._dist.update(self.get_clock().now().nanoseconds * 1e-9, gated, s_dirs, thrust,
                              v_dr, ls[7:10], self.dyn.mi[0] - m_r, self.dyn.m, m_r,
                              com / max(float(self.cable_len_i[0]), 1e-6))
        d_L, d_D = self._dist.d_L, self._dist.d_D
        zero = np.zeros(3)
        if (self._land_to_ground and self._load_down()) or self._dist_mode == 'shadow':
            self._dist_applied = (zero, zero)
        else:
            self._dist_applied = (d_L.copy(), d_D.copy() if self._dist_mode == 'full' else zero)
        self._dist_log_row(gated)

    def _init_offset_free(self):
        """Offset-free MPC (card 2026-10-04_offset_free_innovation): an integrating force on
        the load from the planner's own one-step prediction error. 'shadow' only logs it; 'on'
        plans with it, added to option A's d_L when both are on."""
        p = self.declare_parameter
        self._of_mode = str(p('offset_free', 'off').value).lower()
        self._of = None
        self._of_applied = np.zeros(3)
        self._of_log = None
        self._of_said = 0.0
        if self._of_mode not in ('off', 'shadow', 'on'):
            raise ValueError(f"offset_free must be off, shadow or on, got {self._of_mode!r}")
        if self._of_mode == 'off':
            return
        self._of = OffsetFreeObserver(self.dyn.m, tau=float(p('of_tau', 2.0).value),
                                      window=float(p('of_window', 0.0).value))
        if self._of_mode == 'on' and self._zbias.ki > 0.0:
            raise ValueError('offset_free on with z_ki > 0: two integrals on one height (set z_ki 0)')
        if (self._of_mode == 'on' and self._dist_mode in ('ring', 'full')
                and self._of.tau < 6.0 * self._dist.tau):
            raise ValueError(f'offset_free with dist_est {self._dist_mode}: of_tau must be at least 3x '
                             f'option A\'s two {self._dist.tau:.2f} s stages ({6.0 * self._dist.tau:.1f} s)')
        self.get_logger().warn(f'[planner] offset-free observer {self._of_mode}: tau '
                               f'{self._of.tau:.2f} s, window {self._of.window:.2f} s, bound '
                               f'{self._of.bound:.1f} N')

    def _of_gated(self):
        """Steady planner flight: lift complete, the full pull fed forward (pretension ease and
        B2 cap released), not descending or landing, a fresh pose and every rod taut."""
        if (self.phase != 'planner' or self._settle_left > 0.0 or self._lift_refused
                or self.descending or self._land_to_ground or self.lift_z0 is None
                or self._load_t is None or time.monotonic() - self._load_t > 0.2):
            return False
        if self.lift_progress < (self.target_z - self.lift_z0) - 1e-6:
            return False
        if getattr(self, '_ff_last', 0.0) < 0.999:
            return False
        # as ZBias: not while the ring moves vertically (post-lift settle, a snag) or far off
        if (abs(float(self.load_state[9])) >= 0.05
                or abs(self.target_z - float(self.load_state[2])) >= self._z_i_gate):
            return False
        return all(self._drone_at(i) is not None and self._cable_taut_gate(i)[0] >= self._z_taut_gate
                   for i in range(self.n))

    def _update_offset_free(self):
        if self._of is None or self.load_state is None:
            return
        gated = self._of_gated()
        self._of.update(self.get_clock().now().nanoseconds * 1e-9, gated, self.load_state[7:10],
                        self.load_state[0:3])
        down = self._land_to_ground and self._load_down()
        self._of_applied = (np.zeros(3) if (down or self._of_mode == 'shadow')
                            else self._of.d.copy())
        self._of_log_row(gated)

    def _init_lumped(self):
        """int_mode (card 2026-10-04_z_int_model): 'reference' (default) adds the height integral
        to the target (ZBias); 'model' integrates the ring position error in x, y and z with ZBias's
        gain, bound and gates and plans with it as a force on the ring (int_k_xy, int_k_z N/m)."""
        p = self.declare_parameter
        mode = str(p('int_mode', 'reference').value).lower()
        k_z = float(p('int_k_z', 8.5).value)
        k_xy = float(p('int_k_xy', 5.4).value)
        # > 0: the ring-speed gate only for this long after the lift completes, then dropped so a
        # sag or a rebound is integrated while it happens (0: always, as ZBias)
        self._lint_settle_s = float(p('int_settle_s', 0.0).value)
        self._lint_lift_t = None
        self._lint = None
        self._lint_applied = np.zeros(3)
        self._lint_warned = np.zeros(3, dtype=bool)
        self._lint_said = 0.0
        if mode not in ('reference', 'model'):
            raise ValueError(f"int_mode must be reference or model, got {mode!r}")
        if mode == 'reference':
            return
        if self._zbias.ki <= 0.0:
            raise ValueError('int_mode model needs z_ki > 0 (it is the gain)')
        if self._of_mode == 'on':
            raise ValueError('int_mode model with offset_free on: two integrals on one ring force')
        if self._dist_mode in ('ring', 'full'):
            raise ValueError(f'int_mode model with dist_est {self._dist_mode}: both on the ring force (card critic 3)')
        self._lint = LumpedForceIntegral(self._zbias.ki, self._zbias.i_max, PLANNER_HZ, k_xy, k_z)
        fb = self._lint.force_bound
        self.get_logger().warn(f'[planner] int_mode model: ki {self._lint.ki:.2f}/s, bound '
                               f'{self._lint.i_max:.2f} m = {fb[0]:.2f} N xy, {fb[2]:.2f} N z')

    def _update_lumped(self):
        """One tick of the model-mode integral: the error to the ring's node-0 reference. z is
        gated as ZBias; x and y also need the reference at rest, the ring slow and the miss
        under z_i_gate."""
        ls = self.load_state
        err = np.asarray(self.refs.yref_at(0)[0:3], float) - np.asarray(ls[0:3], float)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if (self._lint_lift_t is None and self.lift_z0 is not None
                and self.lift_progress >= (self.target_z - self.lift_z0) - 1e-6):
            self._lint_lift_t = now_s
        speed_gate = not (self._lint_settle_s > 0.0 and self._lint_lift_t is not None
                          and now_s - self._lint_lift_t >= self._lint_settle_s)
        gz = self._zbias_gated(vz_gate=speed_gate)
        gxy = bool(gz and (self.traj_t <= 0.0 or self.traj.kind == 'hover')
                   and (not speed_gate or float(np.hypot(ls[7], ls[8])) < 0.05)
                   and float(np.hypot(err[0], err[1])) < self._z_i_gate)
        self._lint.update(err, gxy, gz)
        for a in np.flatnonzero(self._lint.near_bound & ~self._lint_warned):
            self._lint_warned[a] = True
            self.get_logger().warn(
                f'[planner] ring integral {"xyz"[a]} {self._lint.b[a]:+.3f} m is near its bound '
                f'(+-{self._lint.i_max:.2f} m, {self._lint.force_bound[a]:.2f} N): a real static '
                f'error is being covered; check feedforward / rods / mass / gain / air')
        now = time.monotonic()
        if now - self._lint_said >= 1.0:
            self._lint_said = now
            f = self._lint.force()
            self.get_logger().info(
                f'[planner int] b=({self._lint.b[0]:+.3f}, {self._lint.b[1]:+.3f}, {self._lint.b[2]:+.3f}) m  '
                f'F=({f[0]:+.2f}, {f[1]:+.2f}, {f[2]:+.2f}) N  gate xy {int(gxy)} z {int(gz)}')

    def _apply_load_force(self):
        """The constant load force the plan uses: option A's estimate, the observer's and the
        lumped integral's (int_mode model)."""
        if self._dist is None and self._of is None and getattr(self, '_lint', None) is None:
            return
        if getattr(self, '_lint', None) is not None:
            self._lint_applied = self._lint.force(bool(self._land_to_ground and self._load_down()))
        d_L = self._dist_applied[0] + self._of_applied + getattr(self, '_lint_applied', np.zeros(3))
        self.solver.set_disturbance(d_L)
        self.refs.d_L = d_L

    def _of_log_row(self, gated):
        """of_est.csv beside log.csv, every tick; a 1 Hz line. Never raises."""
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._of_log is None and self._run_log_dir is not None:
                import csv
                f = open(os.path.join(self._run_log_dir, 'of_est.csv'), 'w', newline='')
                w = csv.writer(f)
                w.writerow(['t', 'gated', 'railed', 'd_x', 'd_y', 'd_z', 'raw_x', 'raw_y', 'raw_z',
                            'app_x', 'app_y', 'app_z', 'vz_meas'])
                self._of_log = (f, w)
            raw = self._of.raw
            if self._of_log is not None:
                f, w = self._of_log
                w.writerow([f'{now:.3f}', int(gated), int(self._of.railed)]
                           + [f'{v:.4f}' for v in self._of.d]
                           + ([f'{v:.4f}' for v in raw] if raw is not None else [''] * 3)
                           + [f'{v:.4f}' for v in self._of_applied]
                           + [f'{float(self.load_state[9]):.4f}'])
                f.flush()
            if now - self._of_said >= 1.0:
                self._of_said = now
                self.get_logger().info(
                    '[planner of] d=(' + ', '.join(f'{x:+.2f}' for x in self._of.d) + ') N  '
                    f'applied={"yes" if np.any(self._of_applied) else "none"}'
                    f'{"  RAILED" if self._of.railed else ""}{"" if gated else "  (frozen)"}')
        except Exception as e:
            if not getattr(self, '_of_log_warned', False):
                self._of_log_warned = True
                self.get_logger().warn(f'[planner] offset-free log off: {e}')

    def _dist_log_row(self, gated):
        """dist_est.csv beside log.csv, every tick; a 1 Hz line. Never raises."""
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._dist_log is None and self._run_log_dir is not None:
                import csv
                f = open(os.path.join(self._run_log_dir, 'dist_est.csv'), 'w', newline='')
                w = csv.writer(f)
                w.writerow(['t', 'gated', 'railed', 'dL_x', 'dL_y', 'dL_z', 'dD_x', 'dD_y', 'dD_z',
                            'raw_dL_x', 'raw_dL_y', 'raw_dL_z', 'raw_dD_x', 'raw_dD_y', 'raw_dD_z',
                            'cond', 'app_dL_x', 'app_dL_y', 'app_dL_z', 'app_dD_x', 'app_dD_y',
                            'app_dD_z'] + [f't{i}' for i in range(self.n)])
                self._dist_log = (f, w)
            raw = self._dist.raw
            rawv = (list(raw[0]) + list(raw[1]) + [raw[3]]) if raw is not None else [''] * 7
            tens = list(raw[2]) if raw is not None else []
            if self._dist_log is not None:
                f, w = self._dist_log
                w.writerow([f'{now:.3f}', int(gated), int(self._dist.railed)]
                           + [f'{v:.4f}' for v in self._dist.d_L] + [f'{v:.4f}' for v in self._dist.d_D]
                           + [v if v == '' else f'{v:.4f}' for v in rawv]
                           + [f'{v:.4f}' for v in self._dist_applied[0]]
                           + [f'{v:.4f}' for v in self._dist_applied[1]]
                           + [f'{v:.3f}' for v in tens])
                f.flush()
            if now - self._dist_said >= 1.0:
                self._dist_said = now
                fmt = lambda v: '(' + ', '.join(f'{x:+.2f}' for x in v) + ')'
                self.get_logger().info(
                    f'[planner dist] d_L={fmt(self._dist.d_L)} N  d_D={fmt(self._dist.d_D)} N/drone  '
                    f'applied={self._dist_mode if any(np.any(a) for a in self._dist_applied) else "none"}'
                    f'{"  RAILED" if self._dist.railed else ""}{"" if gated else "  (frozen)"}')
        except Exception as e:
            if not getattr(self, '_dist_log_warned', False):
                self._dist_log_warned = True
                self.get_logger().warn(f'[planner] dist log off: {e}')

    def _publish_traj_state(self):
        """The load trajectory's parameters and clock (JSON on /payload/trajectory_state),
        so a partner's planner can predict the ring exactly (mission bridge,
        authority.md). hold_s: seconds the trajectory clock stays frozen (reconfig hold)."""
        if self.hover_xy is None:
            return
        import json
        z = (min(self.target_z, self.lift_z0 + self.lift_progress)
             if self.lift_z0 is not None else float(self.load_state[2]))
        msg = String()
        msg.data = json.dumps({
            'kind': self.traj.kind, 'speed': self.traj.speed, 'radius': self.traj.radius,
            'distance': self.traj.distance,
            'hover_x': float(self.hover_xy[0]), 'hover_y': float(self.hover_xy[1]),
            'z': float(z), 'traj_t': float(self.traj_t),
            'hold_s': float(max(0.0, getattr(self, '_reconfig_hold_left', 0.0))),
            'descending': bool(self.descending), 'landing': bool(self._land_to_ground),
            # the trajectory clock advances only once the lift has topped out
            'running': bool(self.phase == 'planner' and self.lift_z0 is not None
                            and self.lift_progress >= (self.target_z - self.lift_z0) - 1e-6
                            and not self.descending and not self._land_to_ground),
            'stamp': self.get_clock().now().nanoseconds * 1e-9})
        self._traj_state_pub.publish(msg)

    def _log_tick(self):
        """10 Hz log.csv beside params.json: load pose, tilt, the planner's height
        target and integral, phase, every physical drone's z, then (appended, 2026-10)
        the solve, per-rod and ring-attitude columns an offline replay needs (see
        tick_log_header). Interactive Gazebo runs (Wesley flies, the logs are read
        afterwards) had no payload record at all. Never raises."""
        try:
            if self._run_log_dir is None:
                return
            if getattr(self, '_tick_log', None) is None:
                import csv
                # widths latched here: a resize or a weld must not shift the columns
                self._log_nd = len(self.drone_pos) + len(getattr(self, 'attach_pos', None) or [])
                self._log_ns = max(self.n, self._log_nd)
                f = open(os.path.join(self._run_log_dir, 'log.csv'), 'w', newline='')
                w = csv.writer(f)
                w.writerow(tick_log_header(self._log_nd, self._log_ns))
                self._tick_log = (f, w)
            f, w = self._tick_log
            w.writerow(self._tick_log_row())
            f.flush()
        except Exception as e:
            if not getattr(self, '_tick_log_warned', False):
                self._tick_log_warned = True
                self.get_logger().warn(f'[planner] tick log off: {e}')

    def _tick_log_row(self):
        nd, ns = self._log_nd, self._log_ns
        fit = lambda xs, k: (list(xs) + [''] * k)[:k]
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        z_tgt = (min(self.target_z, self.lift_z0 + self.lift_progress)
                 if self.lift_z0 is not None else float('nan'))
        t = self.get_clock().now().nanoseconds * 1e-9
        drones = list(self.drone_pos)
        row = ([f'{t:.3f}', self.phase, self.n,
                f'{ls[0]:.4f}', f'{ls[1]:.4f}', f'{ls[2]:.4f}', f'{ls[9]:.4f}',
                f'{tilt:.2f}', f'{z_tgt:.4f}', f'{self._zbias.value:+.4f}',
                f'{self.lift_progress:.4f}', f'{self.traj_t:.2f}',
                int(bool(self._land_to_ground))]
               + fit([f'{d[2]:.4f}' if d is not None else '' for d in drones], nd))
        sv = self.solver
        st = getattr(sv, 'last_status', None)
        row += ['' if st is None else st, f'{getattr(sv, "last_solve_ms", float("nan")):.1f}',
                getattr(sv, 'last_qp_iter', -1), getattr(sv, 'last_iters', 0)]
        X = getattr(self, '_pub_X', None)
        per_rod = []
        for i in range(self.n):
            t_i = tsz = ''
            if X is not None and X.shape[0] >= LOAD_DIM + CABLE_DIM * (i + 1):
                b = LOAD_DIM + CABLE_DIM * i
                t_i = f'{float(X[b + 12, 0]):.3f}'
                tsz = f'{float(X[b + 12, 0]) * float(X[b + 2, 0]):.3f}'
            gate = dist = elev = ''
            pv = self._pivot_at(i)
            if pv is not None:
                d = ls[0:3] + R @ self.rho[i] - pv
                nrm = float(np.linalg.norm(d))
                dist = f'{nrm:.4f}'
                gate = f'{self._cable_taut_gate(i)[0]:.3f}'
                elev = f'{np.degrees(np.arcsin(np.clip(-d[2] / max(nrm, 1e-6), -1, 1))):.2f}'
            per_rod.append((t_i, tsz, gate, dist, elev))
        per_rod = fit(per_rod, ns)
        for k in range(5):
            row += [r[k] if r else '' for r in per_rod]
        row += [f'{getattr(self, "_ff_last", 0.0):.3f}', '' if self._ff_active is None else int(self._ff_active)]
        row += [f'{v:.5f}' for v in ls[3:7]] + [f'{v:.4f}' for v in ls[10:13]]
        age = self._fallback.age(t)
        row += [f'{age:.3f}', self._fallback.streak, ' '.join(str(d) for d in self.slot2drone)]
        for d in fit(drones, nd):
            row += [f'{d[0]:.4f}', f'{d[1]:.4f}'] if isinstance(d, np.ndarray) else ['', '']
        for k in range(nd):
            q = self.drone_quat.get(k)
            row += [f'{v:.5f}' for v in q] if q is not None else [''] * 4
        res = getattr(sv, 'last_res', None)
        row += [self._published] + ([f'{res[0]:.3e}', f'{res[1]:.3e}'] if res is not None else ['', ''])
        li = getattr(self, '_lint', None)
        row += ([f'{v:+.4f}' for v in li.b] + [f'{v:+.4f}' for v in self._lint_applied]
                if li is not None else [''] * 6)
        p5 = getattr(self, '_plan5', None)
        row += [f'{v:.4f}' for v in p5] if p5 is not None else [''] * 3
        return row

    # Mocap callbacks
    def _payload_cb(self, msg: MotionCaptureState):
        p = msg.pose.position
        o = msg.pose.orientation
        lv = msg.twist.linear
        av = msg.twist.angular
        self.load_state = np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z, av.x, av.y, av.z])
        self._load_t = time.monotonic()
        if self.hover_xy is None:
            self.hover_xy = (p.x, p.y)

    def _drone_cb(self, msg: MotionCaptureState, i):
        p = msg.pose.position
        lv = msg.twist.linear
        self.drone_pos[i] = np.array([p.x, p.y, p.z])
        self.drone_vel[i] = np.array([lv.x, lv.y, lv.z])
        o = msg.pose.orientation
        self.drone_quat[i] = np.array([o.w, o.x, o.y, o.z])

    def _fleet_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        # Log every command before filtering: LAND is the only one acted on, so
        # otherwise a non-matching subscription looks identical to a LAND that
        # was delivered and filtered out.
        self.get_logger().info(
            f'[planner] /fleet/command received: {cmd!r} '
            f'(phase={self.phase} takeoff_seen={self.takeoff_seen} '
            f'descending={self.descending} land_to_ground={self._land_to_ground})')
        if cmd != 'LAND':
            return
        if self.phase != 'planner' or not self.takeoff_seen:
            self.get_logger().warn('[planner] LAND ignored - not flying yet')
            return
        if self._land_to_ground:
            # A repeat LAND means the first didn't take, usually because the
            # central controller missed it while we heard it. If we already
            # touched down, re-announce: the one-shot /fleet/landed is long gone
            # and no retry could otherwise complete the handshake.
            if self._landed:
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[planner] LAND repeated after touchdown — re-announcing '
                    '/fleet/landed')
            else:
                self.get_logger().info('[planner] LAND already in progress')
            return
        # descending alone must not gate this: a closing lateral trajectory
        # latches it too, but that auto-descent stops at the handover height and
        # never sets _land_to_ground, so /fleet/landed is never published and the
        # fleet never disarms. LAND upgrades an auto-descent to a full one.
        self.descending = True
        self._land_to_ground = True
        self._landed = False
        # Arm the touchdown detector fresh: this may be upgrading a descent
        # already in progress, so stale stall state must not carry over.
        self._touchdown.reset()
        self._touched_down = False
        self._land_wait_ct = 0
        self._land_ff_off_said = False
        self._land_anchor = None
        self._land_drop = 0.0
        self.get_logger().info('[planner] LAND - descending to the floor')

    def _touchdown_stalled(self):
        """True once every drone STILL ON THE LOAD has stopped descending (see
        TouchdownDetector). Slots, not physical ids: after a resize the departed drones
        are not in slot2drone and must not hold the test open."""
        zs = []
        for i in range(self.n):
            d = self._drone_at(i)
            if d is None:
                return False
            zs.append(float(d[2]))
        return self._touchdown.update(zs)

    def _land_hold_refs(self):
        """LAND with the load already resting: the rods are slack and the OCP has no
        business planning. Its 45-degree cable references pull grounded drones inward
        (R0113: drone 0 slid and tipped; R0490: drone 2 touched down with 21 degrees of
        tilt and went over 0.3 s later, before any stall could be seen). Instead each
        survivor descends STRAIGHT DOWN from where it is, level, at hover thrust with no
        cable term, at land_vel, until the stall detector sees it on the floor. The
        creep in reverse. Frozen once the survivors have touched down."""
        if self._land_anchor is None:
            self._land_anchor = {}
            for i in range(self.n):
                p = self._drone_at(i)
                self._land_anchor[i] = (float(p[0]), float(p[1]), float(p[2]))
            self._land_arc = self._land_arcs() if self._land_unwind else None
            self._land_drop = 0.0
            self._land_blend_t = 0.0
            self.get_logger().info(
                '[planner] load down — survivors ' + (
                    'unwind along their rods to the floor' if self._land_arc
                    else 'descend straight down') + ', level, rods slack')
        # The arc never waits for the drones: a reference that stops descending reads as
        # touchdown to the stall detector, which disarmed the fleet 0.58 m up (R0852).
        if not self._touched_down:
            self._land_drop += self.land_vel / PLANNER_HZ
        # Blend each drone's thrust and cable feedforward from its last in-flight values
        # to plain hover over LAND_BLEND_S. Stepping them (18 deg lean and throttle 0.18 ->
        # level and 0.06 in one tick) dropped the gap's neighbours 0.3 m and slid them
        # 12 cm while they levelled at the flight controllers' 100 deg/s (R0588, R0590,
        # R0591: tipped on the floor).
        self._land_blend_t += 1.0 / PLANNER_HZ
        s = float(np.clip(self._land_blend_t / LAND_BLEND_S, 0.0, 1.0))
        g = self.dyn.g
        hover = np.array([0.0, 0.0, g])
        for i in range(self.n):
            drone = self.slot2drone[i]
            ax, ay, az = self._land_anchor[i]
            p = self._drone_at(i)
            if p is not None and float(p[2]) < LAND_FOLLOW_XY_Z:
                # near the floor, stop correcting sideways: a drone chasing its anchor
                # across the floor tips over (R0590)
                ax, ay = float(p[0]), float(p[1])
                self._land_anchor[i] = (ax, ay, az)
            a0, c0 = self._last_air_ref.get(drone, (hover, np.zeros(3)))
            a_ff = (1.0 - s) * a0 + s * hover
            c_ff = (1.0 - s) * c0
            nodes = []
            for k in range(self.N + 1):
                if self._land_arc is not None:
                    v = 0.0 if self._touched_down else self.land_vel
                    pos, vel = land_arc_ref(*self._land_arc[i], self._land_drop + v * self.dt * k, v)
                    if p is not None and float(p[2]) < LAND_FOLLOW_XY_Z:
                        pos[0:2], vel[0:2] = (ax, ay), 0.0
                    nodes.append((tuple(pos), tuple(vel), tuple(a_ff), tuple(c_ff)))
                    continue
                z = max(0.0, az - self._land_drop - self.land_vel * self.dt * k)
                vz = -self.land_vel if (z > 0.0 and not self._touched_down) else 0.0
                nodes.append(((ax, ay, z), (0.0, 0.0, vz), tuple(a_ff), tuple(c_ff)))
            self._publish_ref(drone, nodes)

    def _land_arcs(self):
        """Per slot (attach point, outward radial, rod length, start elevation, centre-minus-
        pivot offset) for the LAND unwind, from the ring pose and the measured pivots at load
        down. None if a pose is missing (the straight descent is used instead)."""
        if self.load_state is None:
            return None
        R = quat_to_rot_np(self.load_state[3:7])
        arcs = {}
        for i in range(self.n):
            piv = self._pivot_at(i)
            if piv is None:
                return None
            attach = np.asarray(self.load_state[0:3], float) + R @ self.rho[i]
            d = np.asarray(piv, float) - attach
            hn = float(np.hypot(d[0], d[1]))
            radial = np.array([d[0], d[1], 0.0]) / hn if hn > 1e-6 else np.array([1.0, 0.0, 0.0])
            centre_off = -(rot_z(self._slot_yaw(i)) @ self.pivot_offset)
            arcs[i] = (attach, radial, float(self.cable_len_i[i]),
                       float(np.arctan2(d[2], hn)), centre_off)
        return arcs

    def _load_down(self):
        """The load is back at its rest height (within 5 cm of where the lift started):
        the rods are slack from here on."""
        return (self.load_state is not None and self.lift_z0 is not None
                and float(self.load_state[2]) <= self.lift_z0 + 0.05)

    def _departed_down(self):
        """True when every drone that left the fleet mid-flight is on the floor, so
        /fleet/landed (a fleet-wide disarm) cannot catch one airborne. The plain
        planner has no departed drones; the dissipative node overrides this."""
        return True

    def _drone_at(self, i):
        """Measured position of the physical drone occupying OCP slot i (identity
        unless auto_slot_assign remapped it)."""
        return self.drone_pos[self.slot2drone[i]]

    def _slot_vel(self):
        """Measured velocity of the drone in each OCP slot (its rod pivot, the body rate
        term left out), None where unknown."""
        vel = getattr(self, 'drone_vel', None)
        if vel is None:
            return None
        return [vel[d] if d < len(vel) else None for d in self.slot2drone[:self.n]]

    def _pivot_at(self, i):
        """Measured rod pivot of the drone in OCP slot i: centre + R @ pivot_offset. The
        OCP's drone is the rod end, so every rod geometry the planner measures (x_init,
        rod lengths, tautness, elevation, creep) goes through here."""
        p = self._drone_at(i)
        if p is None or not self.pivot_offset.any():
            return p
        q = self.drone_quat.get(self.slot2drone[i])
        if q is None and not getattr(self, '_pivot_level_warned', False):
            self._pivot_level_warned = True
            self.get_logger().warn(f'[planner] drone {self.slot2drone[i] + 1} has no attitude: '
                                   f'its rod pivot assumes a level drone')
        R = quat_to_rot_np(q) if q is not None else np.eye(3)
        return p + R @ self.pivot_offset

    def _slot_yaw(self, i):
        q = self.drone_quat.get(self.slot2drone[i])
        return yaw_from_quat(q) if q is not None else 0.0

    def _rim_dist(self, i):
        """|pivot - attach point| of slot i: the rod length when the rod is taut."""
        ls = self.load_state
        attach = ls[0:3] + quat_to_rot_np(ls[3:7]) @ self.rho[i]
        return float(np.linalg.norm(attach - self._pivot_at(i)))

    def _configure_solver(self, solver):
        solver.solve_budget_s = self._solve_budget_s
        solver.pivot_offset = self.pivot_offset
        solver.fail_dump_dir = getattr(self, '_run_log_dir', None)

    def _solve_context(self):
        """Poses a failed solve is dumped with (planner_fail_<n>.npz)."""
        nan3, nan4 = [np.nan] * 3, [np.nan] * 4
        return {'load_state': self.load_state,
                'slot2drone': self.slot2drone,
                'drone_pos': [self._drone_at(i) if self._drone_at(i) is not None else nan3
                              for i in range(self.n)],
                'drone_quat': [self.drone_quat.get(self.slot2drone[i], nan4) for i in range(self.n)],
                'pivot_offset': self.pivot_offset,
                'sim_time': self.get_clock().now().nanoseconds * 1e-9}

    def _publish_slot_ref(self, i, nodes):
        self._publish_ref(self.slot2drone[i], nodes)

    def _assign_slots(self):
        """Match each physical drone to the nearest nominal azimuth slot around the
        load (see geometry.azimuth_slot_assignment), so the drones can be placed in
        the ring in any order. Relabels I/O only -- the OCP is unchanged.

        Matched in the LOAD frame: the slots ARE the attach points, which rotate with
        the payload. Matching in the world frame instead mismatched every drone to a
        neighbouring attach point as soon as the payload was placed past half a slot
        pitch, which made the first solve QP-infeasible (see the function's docstring)."""
        self.slot2drone = azimuth_slot_assignment(
            self.drone_pos, self.load_state[0:2], self.n, load_yaw=self.psi0,
            slot_az=[np.arctan2(r[1], r[0]) for r in self.rho])
        self._slots_assigned = True
        if getattr(self, 'creep', None) is not None:
            self.creep.label = lambda i: self.slot2drone[i] + 1
        self.get_logger().info(
            f'[planner] auto slot assignment (slot->drone, drones numbered from 1): {[d + 1 for d in self.slot2drone]} '
            f'(load yaw datum {np.degrees(self.psi0):+.1f} deg)')

    def _check_slot_offsets(self):
        """Warn once when a drone sits more than 10 deg from its modelled slot: rig model-f1
        had all four ~30 deg off (one plate), and every ring-quaternion flip then failed the solve."""
        self._slot_offsets_checked = True
        errs = slot_azimuth_errors(self.drone_pos, self.load_state[0:2], self.psi0, self.slot2drone,
                                   [np.arctan2(r[1], r[0]) for r in self.rho])
        self.get_logger().info('[planner] slot azimuth error (deg): '
                               + ', '.join(f'd{d + 1} {e:+.1f} (plate {p})' for d, e, p in errs))
        warns = slot_offset_warnings(errs)
        for text in warns:
            self.get_logger().warn(f'[planner] SLOT OFFSET: {text}')
        if warns:
            self._status_pub.publish(String(data='SLOT OFFSET: ' + '; '.join(warns)))

    def _latch_yaw_datum(self):
        """Latch the payload's measured yaw as the reference datum, once, before the
        first solve. Everything downstream -- the slot matching, the nominal cable
        ring in yref_at/hold_yref, and the load attitude reference q_ref -- is
        expressed about it, so the fleet holds the yaw the rig was PLACED at instead
        of rotating the payload onto world +x on takeoff."""
        self.psi0 = yaw_from_quat(self.load_state[3:7])
        self.refs.set_yaw_datum(self.psi0)
        self._yaw_datum_latched = True
        self.get_logger().info(
            f'[planner] load yaw datum latched at {np.degrees(self.psi0):+.1f} deg')

    # Plan step
    def phase_text(self):
        """What the planner is doing, in the operator's words."""
        if not self.takeoff_seen:
            return 'waiting for TAKEOFF'
        if self._land_to_ground or self.descending:
            return 'landing'
        if self.phase == 'creep':
            return 'creep to the hand-over'
        if self._settle_left > 0.0:
            return 'hand-over settle'
        if self._lift_refused:
            return 'creep timed out: LAND'
        if self._pretensioning():
            return 'pretension'
        if self.lift_ramp_vel <= 0.0:
            return 'holding (no lift)'
        if self.lift_z0 is not None and self.lift_progress < (self.target_z - self.lift_z0) - 1e-3:
            return 'lifting'
        return f'holding at {self.target_z:.2f} m'

    def _publish_phase(self):
        try:
            text = self.phase_text()
        except Exception:                    # never let the panel feed break the planner
            return
        if text != self._phase_text:
            self._phase_text = text
            self._phase_pub.publish(String(data=text))

    def _plan_timer(self):
        try:
            self._plan()
        finally:
            if self._tick_pub is not None:
                t = self.get_clock().now().nanoseconds * 1e-9
                self._tick_pub.publish(Float64MultiArray(data=[t, 1.0 / PLANNER_HZ]))

    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return
        self._published = ''
        try:
            self._plan_tick()
        finally:
            self._log_tick()                # after the solve, so a row carries this tick's solve

    def _plan_tick(self):
        # Latch the placement yaw datum first: the slot matching below is expressed
        # about it, as is every reference the builder produces.
        if not self._yaw_datum_latched:
            self._latch_yaw_datum()

        # Match drones to nominal slots once, now that every pose is in.
        if self.auto_slot_assign and not self._slots_assigned:
            self._assign_slots()
        if not self._slot_offsets_checked:
            self._check_slot_offsets()

        self._publish_load_desired()
        self._publish_traj_state()

        # Phase 1: creep takeoff. The planner assumes taut cables, but running the
        # lift through the ~0.4 m of slack makes the drones overshoot and snap the
        # cable taut, which the open-loop tracker cannot ride. Creep up slowly
        # instead so the slack->taut transition is gentle and tension never spikes
        # past the drones' thrust authority. start_taut worlds skip this.
        if self.phase == 'creep' and self.start_taut:
            self._enter_planner_phase('start_taut')

        gates = [self._cable_taut_gate(i) for i in range(self.n)]
        if self.phase == 'creep':
            # the creep works in slot order, like the OCP: with a non-identity slot map
            # (rig 2026-09-30, [2, 3, 0, 1]) physical order anchored each drone on the
            # opposite plate and flew the fleet into the middle of the ring
            handover, reason = self.creep.step(
                self.load_state, [self._pivot_at(i) for i in range(self.n)], gates,
                self.takeoff_seen)
            # Keep the OCP warm on the live measured config (discarding its horizon,
            # the trackers stay on the creep refs) so the handover has a warm start.
            self._prime_solver()
            if handover:
                self._enter_planner_phase(reason)
            return

        # Vertical phases: ascend to target_z, run the lateral trajectory, then
        # descend once it closes. The ramp must not advance before TAKEOFF: the
        # planner runs from first mocap while the drones sit idle, so
        # lift_progress would run metres above them and yank them up the instant
        # they arm. A position-based liftoff test can't substitute, since in the
        # elevated start_taut world the drones spawn airborne and never rise
        # above spawn to trip it.
        prev_progress = self.lift_progress
        if self.takeoff_seen and self._settle_left > 0.0:
            # post-handover hold: reference frozen at the latched config, load
            # still grounded so the FF gate stays at 0. Nothing accumulates.
            self._settle_left -= 1.0 / PLANNER_HZ
            if self._settle_left <= 0.0 and not self._lift_refused:
                self.get_logger().info(
                    '[planner] handover settle complete — '
                    + ('starting pretension' if self.pretension_s > 0.0 else 'starting lift ramp'))
        elif self.takeoff_seen and self._lift_refused and not self.descending:
            pass            # creep never reached the hand-over angle: hold, no pull, no lift
        elif self.takeoff_seen:
            # Cable-FF soft-start clock: advances only once the lift is active (NOT
            # during the grounded settle above), so the tension FF eases in as the
            # load breaks ground instead of stepping on. Applied in _publish_refs.
            was_pretensioning = self._pretensioning()
            self._ff_t += 1.0 / PLANNER_HZ
            if was_pretensioning and not self.descending:
                # floor start: every rod's pull ramps together, drones held, no height ramp
                if not self._pretension_said:
                    self._pretension_said = True
                    gates = [round(self._cable_taut_gate(i)[0], 2) for i in range(self.n)]
                    self.get_logger().info(
                        f'[planner] pretension: ramping every rod to its share over '
                        f'{self.pretension_s:.1f} s before the lift (length gates {gates}, '
                        f'overridden to 1: rods rigid after the hand-over)')
                rose_now = float(self.load_state[2]) - self.lift_z0
                if (self._ff_cap >= 1.0 and self._ff_t < self.pretension_s
                        and (rose_now > BREAKAWAY_DZ or float(self.load_state[9]) > BREAKAWAY_VZ)):
                    # the ring is off the floor: this fraction already carries it; freeze the
                    # pull here and end the pretension (the hold then runs as usual)
                    self._ff_cap = float(np.clip(self._ff_t / self.pretension_s, 0.0, 1.0))
                    self._ff_t = self.pretension_s
                    self.get_logger().info(
                        f'[planner] pretension: ring broke free at {self._ff_cap:.0%} of the planned '
                        f'pull (up {rose_now:+.3f} m, vz {float(self.load_state[9]):+.2f}); pull held there')
                if not self._pretensioning():
                    rose = float(self.load_state[2]) - self.lift_z0
                    self.get_logger().info(
                        f'[planner] pretension done (ring z {self.load_state[2]:.3f}, '
                        f'lifted {rose:+.3f} m) — starting lift ramp')
                    if rose > 0.0:
                        # ramp from where the ring is, never pull it back to the floor
                        self.lift_z0 = float(self.load_state[2])
            elif self.descending:
                # Auto-descent stops at the handover height (lift_progress -> 0).
                # LAND keeps driving the reference below that until every drone
                # is on the floor: the reference goes underground, the drones
                # just stop when they hit it. Only tracks cleanly if the drones
                # follow closely; a mistuned thrust_ratio floats the load ~1 m
                # above its reference and the descent goes unstable.
                floor = -LAND_MAX_DROP if self._land_to_ground else 0.0
                rate = self.land_vel if self._land_to_ground else self.lift_ramp_vel
                if not self._touched_down:
                    # frozen at the stall point once down: a reference that keeps
                    # sinking while /fleet/landed waits (for a departed drone) grinds
                    # the survivors into the floor and trips the tilt envelope
                    self.lift_progress = max(
                        floor, self.lift_progress - rate / PLANNER_HZ)
                if self._land_to_ground:
                    # Touchdown by stall, not absolute height: over a takeoff
                    # platform a fixed threshold is unreachable, so the descent
                    # would only end when lift_progress bottoms out, grinding the
                    # drones into the platform for ~10 s. Works either way.
                    if not self._touched_down:
                        self._touched_down = (self._touchdown_stalled()
                                              or self.lift_progress <= floor + 1e-6)
                        if self._touched_down and not self._departed_down():
                            self.get_logger().info(
                                '[planner] survivors down — reference held, waiting '
                                'for the departed drone(s) to land')
                    done = False
                    if self._touched_down:
                        self._land_wait_ct += 1
                        done = self._departed_down()
                        if not done and self._land_wait_ct >= int(LAND_DEPARTED_WAIT_S * PLANNER_HZ):
                            self.get_logger().warn(
                                f'[planner] departed drone(s) not down after '
                                f'{LAND_DEPARTED_WAIT_S:.0f} s; announcing landed anyway')
                            done = True
                else:
                    done = self.lift_progress <= 1e-6
                if done and not self._landed:
                    self._landed = True
                    if self._land_to_ground:
                        self.landed_pub.publish(Bool(data=True))
                    self.get_logger().info(
                        '[planner] descent complete — drones on the floor')
                elif self._landed and self._land_to_ground:
                    # Keep announcing until the fleet disarms. A single publish is
                    # lost if the controller missed the original LAND (its
                    # callback drops it while landing=False) and nothing would
                    # resend. Repeats are ignored once disarmed.
                    self.landed_pub.publish(Bool(data=True))
            else:
                # Eased lift ramp. At a constant rate the velocity reference
                # jumps 0 -> lift_ramp_vel the cycle TAKEOFF lands and back to 0
                # at the top; _lift_vel is fed forward directly, so both ends
                # were velocity steps the drones absorbed as the aggressive
                # liftoff. Ease in over LIFT_SOFT_S, out over LIFT_SOFT_D metres.
                total = self.target_z - self.lift_z0
                self._lift_t += 1.0 / PLANNER_HZ
                if (self._ff_cap_force > 0.0 and not self._ff_forced
                        and float(self.load_state[2]) - self.lift_z0 > BREAKAWAY_DZ):
                    # test stand-in for a rig breakaway at this fraction, applied the moment the
                    # ring is off the floor (the twin's ring leaves after the pretension, so its
                    # own freeze never fires), so the release below runs with the ring airborne
                    self._ff_forced = True
                    self._ff_cap = min(self._ff_cap, self._ff_cap_force)
                    self._ff_rel_step = None
                if self._ff_cap < 1.0 and self._ff_release_s > 0.0:
                    if self._ff_rel_step is None:     # cap -> 1 over ff_cap_release_s, from wherever it froze
                        self._ff_rel_step = (1.0 - self._ff_cap) / (PLANNER_HZ * self._ff_release_s)
                        self.get_logger().info(
                            f'[planner] cable pull cap {self._ff_cap:.0%} released to 100 % over '
                            f'{self._ff_release_s:.1f} s')
                    self._ff_cap = min(1.0, self._ff_cap + self._ff_rel_step)
                ease_in = 0.5 * (1.0 - np.cos(
                    np.pi * min(self._lift_t / max(LIFT_SOFT_S, 1e-6), 1.0)))
                remaining = max(total - self.lift_progress, 0.0)
                ease_out = 0.5 * (1.0 - np.cos(
                    np.pi * min(remaining / max(LIFT_SOFT_D, 1e-6), 1.0)))
                # floor the shape so the climb always finishes: a pure ease-out
                # approaches the target asymptotically and lift_done never fires.
                shape = max(min(ease_in, ease_out), LIFT_SOFT_MIN)
                self.lift_progress = min(
                    total, self.lift_progress + shape * self.lift_ramp_vel / PLANNER_HZ)
                # Lateral trajectory runs once the lift tops out: the coupled OCP
                # tracks the moving load reference via _yref_at. When it closes,
                # latch the descent.
                lift_done = (self.lift_progress
                             >= (self.target_z - self.lift_z0) - 1e-6)
                if self.load_traj != 'hover' and lift_done and self._traj_hold_left > 0.0:
                    if self._traj_hold_left >= self._traj_hold_s:
                        self.get_logger().info(
                            f'[planner] hover hold {self._traj_hold_s:.1f} s before the {self.load_traj}')
                    self._traj_hold_left -= 1.0 / PLANNER_HZ
                    if self._traj_hold_left <= 0.0:
                        self.get_logger().info(f'[planner] hover hold done: {self.load_traj} starts')
                elif self.load_traj != 'hover' and lift_done:
                    self.traj_t += 1.0 / PLANNER_HZ
                    if self.traj.complete(self.traj_t):
                        self.descending = True
                        self.get_logger().info(
                            '[planner] load trajectory complete — descending')
        # signed vertical velocity of the lift target for the FF (from the actual
        # change this cycle): +ascend, -descend, 0 hold.
        self._lift_vel = (self.lift_progress - prev_progress) * PLANNER_HZ

        # Solve the planner OCP against the per-node lift/trajectory reference and
        # publish the horizon. Reseed (hard reconverge from x_init) only when there
        # is no valid warm start -- the very first solve, or after a failed one;
        # otherwise warm-start from last_X. On a ground start the solver was kept
        # warm on the live config all through creep (_prime_solver), so last_X is
        # already populated here and this first post-handover solve is a warm
        # refinement, not a cold reconverge.
        self._update_disturbance()
        self._update_offset_free()
        self._apply_load_force()
        if self._land_to_ground and self._load_down():
            self._land_hold_refs()
            # the tracker zeroes the pull on the resting ring; with land_ff_ramp it first flies the
            # blend above (stepping it rocked a survivor to a tip, R1003/R0981)
            self._set_ff_active(bool(self._land_ff_ramp and self._land_blend_t < LAND_BLEND_S))
            return
        if self._lift_refused and not self.descending:
            self._refused_hold_refs()
            return
        self._update_zbias()
        self.refs.update(self.hover_xy, self.lift_z0, self.lift_progress,
                         self.target_z, self._lift_vel, self.traj_t,
                         z_bias=self._zbias.value)
        if self._lint is not None:
            self._update_lumped()
            self._apply_load_force()
        self._solve_and_publish()

    def _solve_and_publish(self):
        # stamped at the measurement, so the fallback's shift matches the horizon's node 0
        now = self.get_clock().now().nanoseconds * 1e-9
        drone_slot_pos = [self._pivot_at(i) for i in range(self.n)]
        x_init = (self.solver.build_x_init(self.load_state, drone_slot_pos, self._slot_vel())
                  if getattr(self, 'pin_cable_rates', False)
                  else self.solver.build_x_init(self.load_state, drone_slot_pos))
        X, status = self.solver.solve_horizon(
            self.refs.yref_at, self.refs.q_ref_at, x_init,
            reseed=self.solver.last_X is None or self.solver.recover,
            context=self._solve_context())
        if X is None:
            # Don't publish a degenerate solution and don't let it warm-start the
            # next cycle: drop the warm start (the next solve fully resets the iterate)
            # and bridge with the last good horizon shifted, for a bounded time. An
            # unconverged but finite iterate is continued from instead of reset.
            self.solver.last_X = self.solver.pending_X
            self.solver.recover = self.solver.last_X is None
            X_shift = self._fallback.failure(now)
            self.get_logger().warn(
                f'[planner] solve status {status} ({self.solver.last_solve_ms:.0f} ms) — '
                + (f'publishing the last horizon shifted ({self._fallback.age(now):.2f} s)'
                   if X_shift is not None else 'holding, nothing published')
                + ', reconverging from x_init next cycle')
            if X_shift is not None:
                self._published = 'shifted'
                self._publish_refs(X_shift)
            else:
                self._published = 'none'
            if getattr(self, '_of', None) is not None:
                self._of.drop_plan()
            return
        self.solver.recover = False
        self.solver.last_X = X
        self._fallback.success(X, now)
        self._published = 'solved'
        self._plan5 = np.array(X[0:3, min(5, X.shape[1] - 1)], float)
        self._publish_refs(X)
        if getattr(self, '_of', None) is not None:
            self._of.set_plan(now, self.dt, X[3:6, :].T, X[0:3, :].T)

    def _prime_solver(self):
        """Keep the OCP warm during the creep phase so the creep->planner switch has
        a valid warm start. Solves a HOLD reference (current load pose, nominal taut
        hover) with node 0 pinned to the live measured state, but does NOT publish
        the horizon — the trackers stay on the creep refs. By handover last_X is a
        converged solution matching the current geometry, so the first planner cycle
        is a warm refinement rather than a cold reconverge (which showed up as a jump
        and a scrambled MPC path for a moment right after handover)."""
        drone_slot_pos = [self._pivot_at(i) for i in range(self.n)]
        x_init = (self.solver.build_x_init(self.load_state, drone_slot_pos, self._slot_vel())
                  if getattr(self, 'pin_cable_rates', False)
                  else self.solver.build_x_init(self.load_state, drone_slot_pos))
        hold = self.refs.hold_yref(self.load_state)
        X, _status = self.solver.solve_horizon(
            lambda _k: hold, self.refs.q_ref_at, x_init,
            reseed=self.solver.last_X is None, context=self._solve_context())
        # None on a failed solve -> next tick reseeds; an unconverged one is continued
        self.solver.last_X = X if X is not None else self.solver.pending_X

    def _publish_load_desired(self, target=None):
        """Publish the desired LOAD position [x, y, z]: the captured hover xy and
        the ramped lift target. Before handover (lift_z0 unset) the load isn't
        being lifted, so the desired height is just its current height.

        `target` overrides the computed desired position, for a subclass whose flight
        phase owns a different one (the dissipative network's p_des) -- otherwise the
        published desired silently disagrees with what is actually being commanded,
        which is exactly what the plot_run.py error analysis reads."""
        if self.hover_xy is None:
            return
        if target is not None:
            x0, y0, z_des = (float(target[0]), float(target[1]), float(target[2]))
        else:
            if self.lift_z0 is not None:
                z_des = min(self.target_z, self.lift_z0 + self.lift_progress)
            else:
                z_des = float(self.load_state[2])
            dx, dy, _, _ = self.traj.offset_at(self.traj_t)
            x0 = float(self.hover_xy[0] + dx)
            y0 = float(self.hover_xy[1] + dy)
        msg = Float64MultiArray()
        msg.data = [x0, y0, float(z_des)]
        self.load_ref_pub.publish(msg)

        # Horizon path for RViz: where the load is heading over the next N+1 nodes.
        #
        # ANCHORED ON THE MEASURED LOAD, like each drone's /drone_N/mpc_plan (whose
        # node 0 is the pinned measurement), so the path visibly emanates from the
        # payload instead of floating at the desired point. The tracking error is NOT
        # lost by this -- it is the gap between /payload/desired_position and the
        # measured load, which is what plot_run.py overlays.
        #
        # The lateral shape comes from evaluating the TRAJECTORY at traj_t + dt*k, as
        # reference_builder.yref_at does. It used to extrapolate along the current
        # velocity, which draws a straight tangent line -- so a circle rendered as a
        # line shooting off the path (0.56 m off it by the end of a 2 s horizon at
        # traj_speed 0.4, radius 0.5).
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        z_cap = self.target_z if self.lift_z0 is not None else z_des
        p_now = self.load_state[0:3]
        dx0, dy0, _, _ = self.traj.offset_at(self.traj_t)
        # Lateral motion only while the trajectory clock is actually running, the same
        # gate reference_builder.yref_at applies to the OCP reference. During the lift
        # (traj_t still 0) and a trajectory hold the clock is frozen, and evaluating the
        # shape at traj_t + dt*k anyway drew the next 2 s of circle bending off a load
        # that is commanded to climb straight up.
        advancing = self.traj_t > self._traj_t_drawn
        self._traj_t_drawn = self.traj_t
        for k in range(self.N + 1):
            t_k = self.traj_t + self.dt * k if advancing else self.traj_t
            kx, ky, _, _ = self.traj.offset_at(t_k)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p_now[0]) + (kx - dx0)
            ps.pose.position.y = float(p_now[1]) + (ky - dy0)
            # z still shows the commanded climb/descent, capped at the target, but
            # measured from where the load actually is.
            ps.pose.position.z = min(z_cap,
                                     float(p_now[2]) + self._lift_vel * self.dt * k)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.load_plan_pub.publish(path)

    def prebuild_solvers(self, sizes):
        """Compile/load a planner OCP for each fleet size we might hand back to.

        Only n sets the OCP dimensions (attachment geometry is a runtime parameter --
        see LoadCableDynamics), so one solver per size covers any layout. They are
        built at STARTUP because an acados build is 9-40 s and cannot happen in
        flight; with a warm cache each is ~0.1 s. `tools/prebuild_planner.py` warms
        that cache after a model change.
        """
        for m in sorted({int(v) for v in sizes} | {self.n}):
            if m in self._solvers or m < 2:
                continue
            rho = attach_points(m, self.attach_radius, self.attach_z)
            dyn = LoadCableDynamics(m, self.load_mass, self.load_inertia,
                                    [self.cable_len] * m, rho, self.drone_mass,
                                    rod_mass=self.rod_mass, rod_lam=self.rod_lam)
            solver = PlannerSolver(dyn, self.get_logger(), pin_rates=self.pin_cable_rates)
            self._configure_solver(solver)
            self._solvers[m] = (dyn, solver, rho)
            self.get_logger().info(f'[planner] OCP ready for n={m}')

    def resize_fleet(self, new_n, drone_ids, rho=None, cable_lengths=None):
        """Re-point the planner at a fleet of `new_n` drones, listed in slot order.

        Swaps the OCP (and its dynamics, ring and reference builder) for the
        pre-built one of that size, and rebuilds the slot->drone map from the
        surviving physical ids. The solver's warm start is dropped: the previous
        solution describes a different fleet, and reusing it would seed the first
        solve of the new size with a state vector of the wrong meaning.

        `rho` IS THE IMPORTANT ARGUMENT. The pre-built solver for new_n carries a
        nominal EVEN ring, and after a detach that is physically wrong: the cables
        that remain are still bolted to their original attach points, so three
        survivors of a four-ring sit at 0/90/180 deg, not at 0/120/240. Handing the
        OCP an even 3-gon makes it solve for a payload whose cables are somewhere
        they are not, and the moment balance it computes tips the load over -- seen
        on R0093, payload tilt 62 deg about 9 s after the hand-back. Pass the
        surviving subset of the ORIGINAL ring instead. This is only expressible
        because attachment geometry is a runtime parameter (LoadCableDynamics).

        Returns False (and changes nothing) if no solver was pre-built for new_n.
        """
        entry = self._solvers.get(int(new_n))
        if entry is None:
            self.get_logger().error(
                f'[planner] cannot resize to n={new_n}: no OCP built for that size. '
                f'Add it to handback_sizes.')
            return False
        if len(drone_ids) != int(new_n):
            self.get_logger().error(
                f'[planner] resize to n={new_n} got {len(drone_ids)} drone ids')
            return False
        self.dyn, self.solver, self.rho = entry
        self.n = int(new_n)
        if rho is not None:
            self.rho = [np.asarray(r, float).reshape(3) for r in rho]
        if cable_lengths is not None:
            self.cable_len_i = [float(v) for v in cable_lengths]
        elif len(self.cable_len_i) < self.n:
            self.cable_len_i = self.cable_len_i + [float(self.cable_len)] * (self.n - len(self.cable_len_i))
        self.solver.set_geometry(rho=self.rho, cable_lengths=list(self.cable_len_i[:self.n]))
        self._s_nom = nominal_cable_dirs(self.rho, self.cable_elev_deg)
        self.refs = ReferenceBuilder(self.dyn, self.n, self._s_nom, self.dt,
                                     self.traj)
        # The new builder starts at yaw 0; keep the placement datum or a ring placed
        # rotated is pulled toward world yaw 0 at every detach and attach.
        if self._yaw_datum_latched:
            self.refs.set_yaw_datum(self.psi0)
        self.slot2drone = [int(d) for d in drone_ids]
        if getattr(self, '_of', None) is not None:
            self._of.drop_plan()             # held across the resize (a reset steps ~0.5 N mid-weld)
        if getattr(self, '_lint', None) is not None:
            self._lint.reset()               # refused with a resizable fleet; belt and braces
            self._lint_applied = np.zeros(3)
        if getattr(self, '_dist', None) is not None:
            self._dist.reset(self.n)         # a different fleet: re-estimate from zero
            for d in self.slot2drone:
                self._dist_subscribe(d)
        self.solver.last_X = None            # different fleet: no valid warm start
        if getattr(self, '_fallback', None) is not None:
            self._fallback.reset()           # nor a horizon to shift
        self.N = self.solver.N
        self.dt = self.solver.dt
        self.get_logger().warn(
            f'[planner] FLEET RESIZED to n={self.n}; slot->drone {self.slot2drone}; '
            f'attach azimuths '
            f'{[round(float(np.degrees(np.arctan2(r[1], r[0]))), 1) for r in self.rho]} deg')
        return True

    def _set_ff_active(self, active):
        if active != self._ff_active:
            self._ff_active = active
            self._ff_active_pub.publish(Bool(data=active))

    def _airborne_start(self):
        return bool(getattr(self.creep, 'airborne_start', False))

    def _refused_hold_refs(self):
        """Creep timed out short of the hand-over angle: hold every drone where it is,
        level, hover thrust, no cable term and no OCP horizon (an OCP horizon at the
        nominal 45 deg drags the rods up the arc and slid the fleet 0.6 m in hold-f1)."""
        self._set_ff_active(False)
        if self._refuse_anchor is None:
            self._refuse_anchor = {i: tuple(float(v) for v in self._drone_at(i))
                                   for i in range(self.n)}
        hover = (0.0, 0.0, self.dyn.g)
        for i in range(self.n):
            ax, ay, az = self._refuse_anchor[i]
            nodes = [((ax, ay, az), (0.0, 0.0, 0.0), hover, (0.0, 0.0, 0.0))
                     for _ in range(self.N + 1)]
            self._publish_ref(self.slot2drone[i], nodes)

    def _pretensioning(self):
        """Floor start, after the settle: the pull is still ramping (or being held full
        for PRETENSION_HOLD_S) and the height ramp has not started."""
        return (self.pretension_s > 0.0 and self.lift_ramp_vel > 0.0 and not self._airborne_start()
                and self.phase == 'planner' and self._settle_left <= 0.0
                and self._ff_t < self.pretension_s + PRETENSION_HOLD_S)

    def _enter_planner_phase(self, reason):
        """Transition creep -> coupled planner: latch the lift-ramp start height.
        No hard reconverge is forced here: the solver was kept warm on the live
        config through creep (_prime_solver), so the first planner solve warm-starts
        from that. For a start_taut air-start (no creep) last_X is still None at this
        point, so that first solve reseeds anyway."""
        self.phase = 'planner'
        self.lift_z0 = float(self.load_state[2])   # ramp lift from here
        self._zbias.reset()                         # fresh per flight
        self._zbias_warned = False
        if getattr(self, '_lint', None) is not None:
            self._lint.reset()
            self._lint_applied = np.zeros(3)
            self._lint_warned = np.zeros(3, dtype=bool)
            self._lint_lift_t = None
        self.lift_progress = 0.0
        self._lift_t = 0.0                         # restart the lift easing
        self._ff_t = 0.0                           # restart the cable-FF soft-start
        self._settle_left = self.handover_settle_s
        self._pretension_said = False
        self._ff_cap = 1.0
        self._ff_rel_step = None
        self._ff_forced = False
        self._traj_hold_left = self._traj_hold_s
        # a floor start that timed out short of the hand-over angle must not lift: from
        # 18-26 deg the rods need 2-3x the tension, mostly sideways (rig 2026-09-30)
        m = re.search(r'elevation timeout at ([-0-9.]+)', reason)
        self._lift_refused = (not self.start_taut and not self._airborne_start()
                              and m is not None and float(m.group(1)) < REFUSE_BELOW_DEG)
        self._refuse_anchor = None
        if self._lift_refused:
            self.get_logger().error(
                f'[planner] {reason}: NOT lifting (rods too flat to carry the ring). '
                f'Holding with no rod pull -- press LAND.')
        if self.measure_rod_len:
            self._apply_measured_rod_lengths()
        self.get_logger().info(
            f'[planner] {reason} — coupled planner active '
            f'(lift from z={self.lift_z0:.2f})')

    def _apply_measured_rod_lengths(self):
        """At handover the rods are taut, so |drone - rim| is the rod length. Replace the
        typed cable_len per drone (OCP geometry + tautness gate) when the guard passes."""
        from rclpy.parameter import Parameter
        dists = [self._rim_dist(i) for i in range(self.n)]
        lens, why = measured_rod_lengths(self.cable_len, dists, tol_frac=self.rod_tol_frac,
                                         spread_m=self.rod_spread_m)
        if lens is None:
            self.get_logger().warn(
                f'[planner] measure_rod_len: keeping typed cable_len {self.cable_len:.3f} -- {why} '
                f'(measured {[round(v, 3) for v in dists]})')
            return
        self.cable_len_i = lens
        self.solver.set_geometry(cable_lengths=lens)
        self.set_parameters([Parameter('measured_rod_len', Parameter.Type.DOUBLE_ARRAY, lens)])
        self.get_logger().info(
            f'[planner] measure_rod_len: rods {[round(v, 3) for v in lens]} m from mocap at '
            f'handover (typed {self.cable_len:.3f}); OCP geometry and tautness gate updated')

    def _publish_ref(self, i, nodes):
        """Publish one drone's reference trajectory. nodes is a sequence of
        (p, v, a, a_cable) with three components each, one entry per horizon
        node. Single definition of the wire format described in the module
        docstring, so the four callers cannot disagree on field order."""
        data = [float(self.N + 1), float(self.dt)]
        for p, v, a, ac in nodes:
            data += [float(p[0]), float(p[1]), float(p[2]),
                     float(v[0]), float(v[1]), float(v[2]),
                     float(a[0]), float(a[1]), float(a[2]),
                     float(ac[0]), float(ac[1]), float(ac[2])]
        msg = Float64MultiArray()
        msg.data = data
        self.ref_pub[i].publish(msg)

    def _zbias_gated(self, vz_gate=True):
        """Steady tethered hover, and nothing else moving the height: lift complete, no
        descent or LAND, no lateral trajectory (frozen through a circle by design),
        load vertical speed under 0.05 m/s, every rod taut (z_taut_gate, 0.99 in sim
        where rigid rods read 0.9999; ~0.9 on the rig when the rod length is typed, or
        the gate never opens), the miss under 0.25 m (else it is not a static offset) and the payload
        pose fresher than 0.2 s (a stale pose must not be integrated)."""
        if self.lift_z0 is None or self.load_state is None or self._load_t is None:
            return False
        lift_complete = self.lift_progress >= (self.target_z - self.lift_z0) - 1e-6
        if not lift_complete or self.descending or self._land_to_ground:
            return False
        if self.traj_t > 0.0 and not self._zbias_orbit_ok():
            return False
        if vz_gate and abs(float(self.load_state[9])) >= 0.05:
            return False
        if time.monotonic() - self._load_t > 0.2:
            return False
        if abs(self.target_z - float(self.load_state[2])) >= self._z_i_gate:
            return False
        for i in range(self.n):
            if self._drone_at(i) is None or self._cable_taut_gate(i)[0] < self._z_taut_gate:
                return False
        return True

    def _zbias_orbit_ok(self):
        """The opt-in orbit window (card 2026-09-27_zki_orbit): a level `orbit` past its
        spin-up, on ticks where its clock actually advanced (reconfiguration and approach
        holds freeze traj_t above the ramp while the ring sits off its reference), and slow
        enough that the centripetal sag is not integrated as a static offset."""
        advanced = self.traj_t > getattr(self, '_zbias_traj_t_prev', 0.0)
        self._zbias_traj_t_prev = self.traj_t
        if not (self._z_ki_in_orbit and self.traj.kind == 'orbit' and advanced
                and self.traj_t > ORBIT_RAMP_S):
            return False
        r = max(float(self.traj.radius), 1e-6)
        return float(self.traj.speed) ** 2 / r <= Z_KI_ORBIT_MAX_ACC

    def _update_zbias(self):
        if self._zbias.ki <= 0.0 or self.load_state is None or getattr(self, '_lint', None) is not None:
            return
        err = self.target_z - float(self.load_state[2])
        self._zbias.update(err, self._zbias_gated())
        if self._zbias.near_bound and not self._zbias_warned:
            self._zbias_warned = True
            self.get_logger().warn(
                f'[planner] height integral {self._zbias.value:+.3f} m is near its bound '
                f'(+-{self._zbias.i_max:.2f}): a real static error is being covered; check '
                f'feedforward / rods / mass / gain')

    def _cable_taut_gate(self, i):
        """(gate, dist): `gate` in [0, 1] scales the cable-tension feedforward we
        publish to the tracker by how LENGTH-taut the cable measures right now —
        0 while the cable is slack, 1 once straightened (the slack->taut
        transition). Rigid cables are taut from spawn, so this is ~1 immediately;
        soft cables ramp it in as they straighten."""
        dist = self._rim_dist(i)
        d_lo = CABLE_TAUT_LO_FRAC * self.cable_len_i[i]
        d_hi = CABLE_TAUT_HI_FRAC * self.cable_len_i[i]
        len_gate = float(np.clip((dist - d_lo) / max(d_hi - d_lo, 1e-6), 0.0, 1.0))
        return len_gate, dist

    def _publish_static_cable(self, X):
        """Static vertical cable share per physical drone (kt_trim's cable term)."""
        planned = [float(X[LOAD_DIM + CABLE_DIM * i + 12, 0]) * float(X[LOAD_DIM + CABLE_DIM * i + 2, 0]) / self.drone_mass
                   for i in range(self.n)]
        shares = static_cable_z(planned, self.load_mass, self.drone_mass)
        if shares is None:
            return
        if not hasattr(self, '_static_pub'):
            self._static_pub = {}
        for i, az in enumerate(shares):
            k = self.slot2drone[i]
            if k not in self._static_pub:
                self._static_pub[k] = self.create_publisher(Float64, f'/drone_{k}/cable_static_z', 1)
            self._static_pub[k].publish(Float64(data=float(az)))

    def _publish_refs(self, X):
        # Cable-FF soft-start: 0 through the grounded handover settle, easing to 1
        # over FF_EASE_S once the lift starts (clock advanced in _plan). Multiplied
        # into the published tension FF so it is never stepped onto the still-
        # grounded load -- that step made the drones lurch and scrambled the horizon.
        ease = self.pretension_s if self.pretension_s > 0.0 else FF_EASE_S
        ff = min(float(np.clip(self._ff_t / ease, 0.0, 1.0)), self._ff_cap)
        if self._lift_refused or self.lift_ramp_vel <= 0.0:
            ff = 0.0                   # refused, or a hold test: never pull on the ring
        if self._land_to_ground and self._load_down():
            # LAND with the load already resting: the rods are slack, but the OCP still
            # pins 45 deg cable directions and nominal tension, and that feedforward
            # (3.8 m/s^2 sideways per drone) is what slid R0113's drone 0 across the
            # floor until it tipped (critic, 2026-09-24_detach_land_fix). No pull to
            # feed forward once the load is down.
            ff = 0.0
            if not self._land_ff_off_said:
                self._land_ff_off_said = True
                self.get_logger().info(
                    '[planner] load down — cable feedforward off, rods slack')
        self._set_ff_active(ff > 0.0)
        self._ff_last = ff
        self._pub_X = X
        # floor start after a hand-over at >= 37 deg: the rods are rigid and straight, so
        # every rod gets the same fraction of its share (the length gate read 0.43 on a rig
        # rod whose resting distance was 3 cm short, and would step to 1 at breakaway)
        equal_pull = (self.pretension_s > 0.0 and not self._airborne_start()
                      and self.phase == 'planner' and self._settle_left <= 0.0)
        diag = []
        for i in range(self.n):
            gate, dist = self._cable_taut_gate(i)
            t_i = float(X[LOAD_DIM + CABLE_DIM * i + 12, 0])   # planned tension, node 0
            diag.append((i, dist, gate, t_i))
            nodes = []
            yaw = self._slot_yaw(i)
            d_D = self._dist_applied[1] / self.dyn.mi[i]
            for k in range(self.N + 1):
                pos, vel, acc, cable = self.solver.drone_kinematics(X[:, k], i, yaw)
                # a force on the drone (d_D, 'full'): the thrust must counter it, and the
                # tracker's model must contain it, as it contains the rod's pull
                nodes.append((pos, vel, acc - d_D, (1.0 if equal_pull else gate) * ff * cable + d_D
                              + rod_accel_at(self.solver, X[:, k], i)))
            # slot i's planned trajectory belongs to the physical drone occupying it
            self._publish_ref(self.slot2drone[i], nodes)
            # remembered for a smooth hand-over into the landing hold (_land_hold_refs)
            self._last_air_ref[self.slot2drone[i]] = (np.asarray(nodes[0][2], float),
                                                      np.asarray(nodes[0][3], float))
        self._publish_static_cable(X)

        # ~1 Hz: per-drone cable tautness so you can see when (and whether) the
        # cable term engages. dist -> CABLE_LEN means taut; gate is the applied
        # tension scale; t is the OCP's planned tension. If the drones fall while
        # gate stays 0, the cable never tautens before they lose it.
        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            s = '  '.join(f"d{self.slot2drone[i] + 1}:dist={d:.2f} gate={g:.2f} t={t:.2f}"
                          for (i, d, g, t) in diag)
            z_tgt = min(self.target_z, self.lift_z0 + self.lift_progress)
            # load tilt (angle of the load body-z off world-z) and each MEASURED
            # cable elevation, so an asymmetric divergence is visible before it
            # blows up: a rotating load / flattening cables drives the tension split.
            ls = self.load_state
            R = quat_to_rot_np(ls[3:7])
            tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
            elevs = []
            for i in range(self.n):
                d = (ls[0:3] + R @ self.rho[i]) - self._pivot_at(i)
                nd = np.linalg.norm(d)
                elevs.append(np.degrees(np.arcsin(np.clip(-d[2] / max(nd, 1e-6), -1, 1))))
            e = ' '.join(f"{x:.0f}" for x in elevs)
            # planned load height at the END of the horizon and drone 0's planned height
            # there: if these sit at the target while the measured load does not, the
            # trackers are not executing the horizon; if they sit at the measured load,
            # the OCP is not planning the move (experimentation, 2026-09-10).
            zN = float(X[2, -1])
            d0N = float(self.solver.drone_kinematics(X[:, -1], 0)[0][2])
            self.get_logger().debug(
                f"[planner cable] L={self.cable_len:.2f} load_z={self.load_state[2]:.2f} "
                f"z_tgt={z_tgt:.2f} zI={self._zbias.value:+.3f} zN={zN:.2f} d0N={d0N:.2f} ff={ff:.2f} tilt={tilt:.1f}deg "
                f"elev=[{e}]  {s}")


def main(args=None):
    rclpy.init(args=args)
    node = LoadPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
