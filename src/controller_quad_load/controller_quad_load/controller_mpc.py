import os
import fcntl

from utility_objects.run_context import acados_dir, log_base_dir

# Own acados dir so this solver doesn't clash with controller_mpc_multi's.
# Resolved from the workspace root (or MDC_ACADOS_ROOT) rather than a literal
# path, so the repo can be checked out anywhere -- including a second machine
# during a lab session.
ACADOS_DIR = acados_dir('quad_load')
# acados generates its C relative to the CWD, so this chdir is required. It is
# ALSO why logs must not be written to a CWD-relative path: see LOG_BASE_DIR.
os.chdir(ACADOS_DIR)

# Captured BEFORE any further chdir, and absolute, so flight data lands in
# results/ and never inside this build directory.
LOG_BASE_DIR = log_base_dir()

# file lock so the drone processes don't compile at the same time
_lock_path = os.path.join(ACADOS_DIR, '.compile.lock')
_lock_file = open(_lock_path, 'w')
fcntl.flock(_lock_file, fcntl.LOCK_EX)

import rclpy
import signal
import sys
import numpy as np
import copy
import math
import threading
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from . import acados as _acados_mod
from .acados import (generate_ocp_controller, set_initial_guess,
                     warm_start_from_previous_solution, set_planner_reference)
# The parameter UKF itself lives in thrust_ratio_node.py, a SEPARATE process --
# see the thrust_ratio_estimator parameter below for why it must not run here.
N_POSE = 13


# recompile the solver if acados.py/dynamics.py changed since the last build
_PKG_DIR = os.path.dirname(_acados_mod.__file__)
_SOLVER_SRC = [os.path.join(_PKG_DIR, 'acados.py'),
               os.path.join(_PKG_DIR, 'dynamics.py')]
# the .so lands in a nested c_generated_code dir, json goes in ACADOS_DIR
_SOLVER_SO = os.path.join(ACADOS_DIR, 'c_generated_code',
                          'libacados_ocp_solver_quad_load_dynamics.so')
_SOLVER_JSON = os.path.join(ACADOS_DIR, 'quad_load_dynamics_ocp.json')


def _is_fresh(so_path, json_path):
    """artifact exists and is newer than its source -> load it, don't recompile"""
    if not (os.path.exists(so_path) and os.path.exists(json_path)):
        return False
    so_mtime = os.path.getmtime(so_path)
    return all(os.path.exists(s) and os.path.getmtime(s) <= so_mtime
               for s in _SOLVER_SRC)


def _solver_is_fresh():
    return _is_fresh(_SOLVER_SO, _SOLVER_JSON)


from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger, node_params, write_params
from utility_objects.safety import EnvelopeChecker, EnvelopeLimits
from utility_objects.callback_manager_multi import CallbackManagerMulti
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from std_msgs.msg import Int32, Float64MultiArray, String
from sensor_msgs.msg import Imu


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True
# run at 50 Hz (up from 30). the taut cable is a stiff disturbance so the loop
# needs to be fast. if the "Solve time" log gets close to the control period,
# drop this.
FREQUENCY_HZ = 50.0
DT = 1.0 / FREQUENCY_HZ

# limits for the measured-f_ext path (cable_source=measured). feeding the raw
# imu cable force straight in makes an imu->model->command->imu loop that blows
# up at takeoff, so filter/clamp/ramp it.
CABLE_ACCEL_CAP = 6.0          # m/s^2: clamp measured cable-accel magnitude (~2x taut)
CABLE_SLEW = 25.0              # m/s^2 per second: max rate-of-change of applied cable
MAX_CONSEC_SOLVE_FAILS = 15    # tolerate this many bad solves (hold) before disarming

# Ceiling for the airborne kT schedule (a(u)=c*u^2 -> secant c*throttle). Sized
# for the ORIGINAL motorConstant 1.42e-06 (c=203, hover throttle ~0.24 -> ~49), so
# 55 capped momentary highs without exceeding the real plant. The floor is
# thrust_ratio, so takeoff can never be starved.
# WARNING: this ceiling only protects you if thrust_quad_c matches the SDF. With
# the current motorConstant 0.62e-06 the true c is 88.6; a launch still passing
# the stale 203 computes kT = 203*0.37 = 75, CLIPS to 55, and flies ~67% above the
# true ~33 -- i.e. the clip hides the stale constant instead of catching it. At the
# correct c=88.6 the schedule maxes at 88.6*0.6 = 53 and this ceiling never binds.
KT_SCHED_CEIL = 55.0
# The kT schedule only engages once the drone has climbed this far above its spawn
# height, i.e. off the stands. The takeoff-safe thrust_ratio (which slightly
# OVER-thrusts, giving the oomph to break off the stands) is kept until then; only
# the settled climb/hover gets the operating-point kT.
AIRBORNE_MARGIN = 0.04         # m above spawn z before scheduling kT. This is
                               # FUNCTIONAL, not just cosmetic: on a taut air-start the
                               # drones rest on stands that mask their weight-support
                               # need. Pinning kT to thrust_ratio (~45, ~15% below the
                               # true hover gain 203*throttle~52) makes the MPC
                               # over-throttle, which POPS the drones off the stands and
                               # tensions the cables -> the load lifts and stays taut.
                               # That pop IS the takeoff surge. Set the margin to 0 and
                               # kT snaps to the correct ~50 immediately: the drones hold
                               # exactly at spawn, never pop off, never tension the
                               # cables, the load droops to the floor and gets abandoned
                               # (payload_resting zeros the cable FF -> runaway sag). So
                               # the surge and the lift are the same event; the margin
                               # must be big enough to pop the drones off the stands, then
                               # the schedule corrects kT once airborne. Soften the surge
                               # via thrust_ratio (gentler over-thrust), not the margin.

# Takeoff spool floor. The applied throttle eases from FLOOR*u to u over
# takeoff_spool_s, NOT from 0. On a taut air-start the drones bear the load from
# the first cycle: spooling from 0 starves the lift, the cables go slack and the
# payload never leaves the ground. The floor gives near-hover thrust immediately
# (so the load lifts) while the cosine still eases the last bit on smoothly (so
# there is no lurch). 0.0 recovers the old spool-from-zero (ground-takeoff) shape.
TAKEOFF_SPOOL_FLOOR = 0.5

LOGGING_NAME = 'controller_quad_load'

# Formation offsets (x, y) for each drone relative to drone 0
DRONE_OFFSETS = {
    0: (0.0, 0.0),
    1: (1.0, 0.0),
    2: (0.0, 1.0),
    3: (1.0, 1.0),
}

class Controller(Node):
    def __init__(self):
        # use_sim_time is the LAUNCH's call, not ours. It used to be pinned False here
        # unconditionally, which is right on hardware (there is no /clock) but wrong in
        # simulation: the control loop then ran at FREQUENCY_HZ of WALL time while the
        # planner ran at PLANNER_HZ of SIM time, so the number of control updates per
        # simulated second was Gazebo's real-time factor -- i.e. a function of machine
        # load. Two runs of the same trajectory at RTF 0.40 and 0.60 gave payload radius
        # errors of -16% and -34%, which made every sim A/B silently incomparable.
        #
        # The sim launches set use_sim_time:=true (and rviz_quad_load_launch.py bridges
        # /clock); every real_*_launch.py sets it false, and False is also the ROS
        # default if nothing sets it -- so hardware behaviour is unchanged.
        super().__init__('controller')

        # ── Drone identity ────────────────────────────────────────────────
        self.declare_parameter("drone_id", 0)
        self.drone_id = self.get_parameter("drone_id").value
        self.offset_x, self.offset_y = DRONE_OFFSETS.get(self.drone_id, (0.0, 0.0))

        # A/B toggles for the two cable feedforward bits, to see which one
        # upsets the attitude loop:
        #   cable_ff_scale=0.0  ignore a_cable in the model
        #   attitude_ff=false   level attitude ref, keep throttle FF
        self.declare_parameter("cable_ff_scale", 1.0)
        self.cable_ff_scale = float(self.get_parameter("cable_ff_scale").value)
        self.declare_parameter("attitude_ff", True)
        self.attitude_ff = bool(self.get_parameter("attitude_ff").value)
        # cable accel source: 'model' = planner t*s/m, 'measured' = from the imu
        self.declare_parameter("cable_source", "model")
        self.cable_source = self.get_parameter("cable_source").value
        self.get_logger().warn(
            f"[Drone {self.drone_id}] tracker FF toggles: "
            f"cable_ff_scale={self.cable_ff_scale} attitude_ff={self.attitude_ff} "
            f"cable_source={self.cable_source}")

        self.planner_ref_pos = None      # (N+1, 3) world-frame position nodes
        self.planner_ref_vel = None      # (N+1, 3) world-frame velocity nodes
        self.planner_ref_acc = None      # (N+1, 3) required specific thrust accel
        self.planner_ref_cable = None    # (N+1, 3) cable tension accel t*s/m (world)
        self._current_ref_pos = None     # desired position now (node 0), for logging

        # imu measured cable force: accel reads (thrust+cable)/m in body frame,
        # subtract the modelled thrust to get the measured cable accel
        self.imu_lin_acc = None          # (3,) low-pass filtered body specific force
        self.last_cmd_throttle = None    # throttle actually applied to the FC
        # ema smoothing of the raw imu (alpha per sample). raw is too noisy to
        # feed back; 0.03 ~= 30 ms time constant
        self.imu_alpha = 0.03
        # slew-limited cable accel actually fed to the model + solve-fail counters
        self._applied_cable_vec = None   # (3,) last applied cable accel (for slew)
        self._last_good_msg = None       # last successfully-solved ELRS command
        self._solve_fail_ct = 0
        # channel values ACTUALLY sent to the FC last cycle [roll, pitch, thr,
        # yaw]. The MPC pins its u_state to this, so the model's actuator state
        # matches reality even while we're publishing idle (pre-TAKEOFF) or a
        # spooled-down throttle. Starts at the disarmed/idle value: all zero.
        self._applied_u = np.zeros(4)
        # the u_dot ACTUALLY applied last cycle. The parameter UKF propagates the
        # model with it, so it must be the published command, not the solver's latest.
        self._applied_u_rate = np.zeros(4)

        # get_clock() follows use_sim_time (sim clock in simulation, wall on hardware).
        # The pose-timeout watchdog deliberately stays on a separate WALL clock: it is
        # about real comms latency, and CallbackManagerMulti stamps last_pose_update_time
        # from its own wall clock, so both sides of that comparison must remain wall time.
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_pose_update_time = self._wall_clock.now()

        # ── Callbacks / subscriptions ─────────────────────────────────────
        self.cb = CallbackManagerMulti(self, drone_id=self.drone_id)

        # ── Wait for first mocap pose ─────────────────────────────────────
        self.current_pose = None
        self.get_logger().info(f"[Drone {self.drone_id}] Waiting for initial mocap pose...")
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"[Drone {self.drone_id}] Pose received.")
        # ── Visualizer ────────────────────────────────────────────────────
        # prefix the viz topics per drone (/drone_N/mpc_plan etc) so RViz can
        # show each drone's plan on its own display instead of N drones fighting
        # over one global /mpc_plan. Publishes the live MPC horizon + actual path
        # during flight (see control_loop); the reference is streamed by the planner.
        self.trajectory_visualizer = TrajectoryVisualizer(
            self, frame_id="map", prefix=f"drone_{self.drone_id}")

        # ── State ─────────────────────────────────────────────────────────
        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False
        self.step_counter = 0

        # fleet manager broadcasts the master step index, just follow it
        self.fleet_step_sub = self.create_subscription(
            Int32,
            '/fleet/step',
            self._fleet_step_callback,
            1)  # depth=1: always use the latest, never queue stale steps

        # ── Planner reference subscription ────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            f'/drone_{self.drone_id}/reference_trajectory',
            self._planner_ref_callback,
            1)
        self.get_logger().info(
            f"[Drone {self.drone_id}] Tracking external planner reference.")

        # ── IMU subscription (measured specific force -> cable force) ──────
        self.create_subscription(
            Imu, f'/drone_{self.drone_id}/imu', self._imu_callback, 10)

        # every drone watches the payload height so it knows if the load is
        # resting on the ground (cable slack, no tension) or suspended. drone 0
        # also logs the payload actual + desired.
        self.payload_pos = None
        self.payload_ref = None
        self.payload_resting = True       # assume grounded until told otherwise
        # payload counts as resting while its z is at/below this. set per world
        # (a bit above the payload's on-ground height).
        self.declare_parameter("payload_rest_z", 0.05)
        self.payload_rest_z = float(self.get_parameter("payload_rest_z").value)
        # takeoff spool-up: ramp the applied throttle from 0 up to the MPC value
        # over this many seconds so the drones ease off the platforms instead of
        # popping up the instant TAKEOFF fires. 0 = off (instant, old behaviour).
        self.declare_parameter("takeoff_spool_s", 2.0)
        self.takeoff_spool_s = float(self.get_parameter("takeoff_spool_s").value)
        # thrust_ratio (kT): accel per unit throttle the MPC assumes. if it's set
        # below the sim's real thrust the drones over-throttle and shoot up (worst
        # at takeoff). keep 24 for hardware; raise toward the sim value (~40) in
        # sim. overrides est_params[0] below.
        self.declare_parameter("thrust_ratio", 24.0)
        self.thrust_ratio = float(self.get_parameter("thrust_ratio").value)
        # Quadratic-plant coefficient c in a(u)=c*u^2. Once airborne the linear kT
        # is scheduled to the operating point, kT = clip(c*throttle, thrust_ratio,
        # KT_SCHED_CEIL), so the assumed thrust matches the true secant gain at
        # hover. 0 disables (fixed thrust_ratio everywhere). See _scheduled_kT.
        self.declare_parameter("thrust_quad_c", 0.0)
        self.thrust_quad_c = float(self.get_parameter("thrust_quad_c").value)
        # ── Adaptive kT ───────────────────────────────────────────────────
        # Measure kT in flight instead of assuming it. The open-loop thrust_quad_c
        # schedule above needs the plant's quadratic coefficient known in advance
        # (it comes from the SDF motor model) and has no hardware equivalent.
        #
        # Two backends:
        #   'ukf' (default) -- the controller_ukf method, run OUT OF LOOP by
        #       thrust_ratio_node.py (one process per drone). This tracker just
        #       publishes /drone_N/kt_input every cycle and reads /drone_N/kt_estimate;
        #       both are microseconds. The UKF itself is 39 acados propagations, which
        #       measured 25-150 ms against this 20 ms control period once three
        #       controllers, Gazebo and the planner were sharing cores -- run in this
        #       timer it stalled the control loop and degraded the tracker even in
        #       shadow mode, where its output was not used at all. Hence the split.
        #   'none' -- schedule/fixed kT only (thrust_quad_c, else thrust_ratio).
        self.declare_parameter("thrust_ratio_estimator", "ukf")
        self.thrust_ratio_estimator = str(
            self.get_parameter("thrust_ratio_estimator").value).strip().lower()
        if self.thrust_ratio_estimator not in ('ukf', 'none'):
            self.get_logger().warn(
                f"unknown thrust_ratio_estimator "
                f"{self.thrust_ratio_estimator!r}; using 'ukf'")
            self.thrust_ratio_estimator = 'ukf'
        # adaptive_thrust_feedback false = SHADOW mode: estimate and print, but the
        # MPC keeps flying on the scheduled/fixed kT -- run that first to check the
        # estimate is sane.
        # Defaults OFF at the node so the already-validated dissipative and attach
        # launches keep their exact tuned thrust behaviour; mpc_quad_load_launch.py
        # opts in explicitly.
        self.declare_parameter("adaptive_thrust_ratio", False)
        self.adaptive_thrust_ratio = bool(
            self.get_parameter("adaptive_thrust_ratio").value)
        self.declare_parameter("adaptive_thrust_feedback", True)
        self.adaptive_thrust_feedback = bool(
            self.get_parameter("adaptive_thrust_feedback").value)
        # Seconds between the one-line kT reports. 0 = silent.
        self.declare_parameter("kt_print_period_s", 1.0)
        self.kt_print_period_s = float(self.get_parameter("kt_print_period_s").value)
        # Hard bounds on the estimate. The floor matters: kT below ~12 is not a
        # physical airframe, it's a broken measurement, and acting on it would
        # command a huge throttle.
        self.declare_parameter("kt_min", 12.0)
        self.declare_parameter("kt_max", 60.0)
        # How fast the MPC's kT may follow the estimate (units/s). The tracker has
        # no integrator and trusts its feedforward, so kT must move gently.
        self.declare_parameter("kt_slew_per_s", 6.0)
        # Updates required before the estimate is allowed to drive the MPC.
        self.declare_parameter("kt_min_updates", 50)
        # LEARN-THEN-LOCK. Estimate kT for this many seconds after becoming airborne,
        # then FREEZE it for the rest of the flight.
        #
        # kT is a property of the airframe and the battery, not of the trajectory, so
        # there is nothing to track once it is known. The takeoff/lift is also the best
        # window to measure it: near-hover, well modelled, and the load is being lifted
        # straight up rather than swung. Freezing before the trajectory starts stops the
        # filter chasing exactly the disturbances it cannot explain -- cable tension
        # that differs from the planner's model, payload swing -- which is what made it
        # wander during the interesting part of the flight.
        #
        # Freezing also stops the tracker publishing kt_input, so the estimator nodes go
        # idle and give their cores back for the trajectory.
        # 0 = never freeze (keep estimating for the whole flight).
        self.declare_parameter("kt_freeze_after_s", 10.0)
        self.kt_freeze_after_s = float(self.get_parameter("kt_freeze_after_s").value)
        # ESTIMATOR SEED, decoupled from thrust_ratio. These two want different
        # values and cannot be the same number:
        #   thrust_ratio  is the TAKEOFF constant and wants to be a little LOW -- the
        #                 resulting over-thrust is the pop that breaks the drones off
        #                 their stands (see AIRBORNE_MARGIN).
        #   kt_seed       is where the estimator starts and the centre of its band,
        #                 and wants to be the drone's ACTUAL hover kT.
        # Tying them together meant a takeoff-safe 30 seeded the estimator 10% below
        # the true ~33; the filter is deliberately slow, so it stayed near 30 and the
        # MPC over-throttled (load overshot its target by ~0.18 m). <=0 means "use
        # thrust_ratio", which reproduces the old coupled behaviour.
        self.declare_parameter("kt_seed", 0.0)
        _seed = float(self.get_parameter("kt_seed").value)
        self.kt_seed = _seed if _seed > 0.0 else self.thrust_ratio
        # HARD BAND around kt_seed, as a fraction. The seed is a
        # measured property of the airframe (SDF motor model in sim, bench thrust
        # test on hardware), so it is already close; the estimator's job is a small
        # refinement, NOT a search. Bounding it to seed*(1 +/- this) means a
        # disturbance the filter cannot explain -- a tension spike, a swinging load,
        # a mocap dropout -- can at worst nudge kT, never walk it somewhere absurd.
        # Same idea as controller_mpc_payload's thrust_ratio_feedback_max_fractional_change.
        # 0 disables the band and falls back to the raw kt_min/kt_max limits.
        self.declare_parameter("kt_max_deviation", 0.15)
        self.kt_min = float(self.get_parameter("kt_min").value)
        self.kt_max = float(self.get_parameter("kt_max").value)
        # Intersect the seed-relative band with the absolute limits. Everything
        # downstream (both estimator backends and the MPC feedback) clamps to these,
        # so there is one place the estimate can live.
        dev = float(self.get_parameter("kt_max_deviation").value)
        if dev > 0.0:
            self.kt_min = max(self.kt_min, self.kt_seed * (1.0 - dev))
            self.kt_max = min(self.kt_max, self.kt_seed * (1.0 + dev))
            if self.kt_min >= self.kt_max:
                self.get_logger().warn(
                    f"[Drone {self.drone_id}] kt_max_deviation band is empty "
                    f"({self.kt_min:.2f}..{self.kt_max:.2f}); ignoring it.")
                self.kt_min = float(self.get_parameter("kt_min").value)
                self.kt_max = float(self.get_parameter("kt_max").value)
            else:
                self.get_logger().info(
                    f"[Drone {self.drone_id}] kT constrained to "
                    f"{self.kt_min:.2f}..{self.kt_max:.2f} "
                    f"(kt_seed {self.kt_seed:.2f} +/-{100*dev:.0f}%, "
                    f"takeoff thrust_ratio {self.thrust_ratio:.2f}).")
        self.kt_slew_per_s = float(self.get_parameter("kt_slew_per_s").value)
        self.kt_min_updates = int(self.get_parameter("kt_min_updates").value)

        # ── Parameter-UKF backend settings ────────────────────────────────
        # Estimate considered stale after this long with no message from the node
        # (it crashed, was never launched, or is starved). Stale -> fall back to the
        # scheduled/fixed kT rather than keep flying on a frozen number.
        self.declare_parameter("kt_estimate_timeout_s", 2.0)
        self.kt_estimate_timeout_s = float(
            self.get_parameter("kt_estimate_timeout_s").value)
        self._ukf_solve_ms = 0.0
        self._kt_node_value = None        # latest kT from /drone_N/kt_estimate
        self._kt_node_std = float('nan')
        self._kt_node_count = 0
        self._kt_node_time = None         # monotonic time that estimate arrived
        self._kt_input_pub = None
        if self.adaptive_thrust_ratio and self.thrust_ratio_estimator == 'ukf':
            # Out-of-loop: publish the estimator's inputs, consume its output. Both
            # are trivial; nothing acados-related runs in this process.
            self._kt_input_pub = self.create_publisher(
                Float64MultiArray, f'/drone_{self.drone_id}/kt_input', 5)
            self.create_subscription(
                Float64MultiArray, f'/drone_{self.drone_id}/kt_estimate',
                self._kt_estimate_cb, 1)
            self.get_logger().info(
                f"[Drone {self.drone_id}] kT from the external estimator node "
                f"(kt_seed {self.kt_seed:.2f}, band {self.kt_min:.2f}.."
                f"{self.kt_max:.2f}, feedback={self.adaptive_thrust_feedback}). "
                f"Launch controller_quad_load/kt_estimator for this drone, or no "
                f"estimate will arrive and the scheduled/fixed kT is used.")
        self._kt_status = 'waiting_for_takeoff'
        self._kt_frozen_value = None   # latched kT once the learn window closes
        self._kt_airborne_cycles = 0
        self._kt_was_driving = False   # latches the slew back to base
        self._kt_spawn_z = None           # resting z, re-latched until TAKEOFF
        self._kt_cable_world = None       # planner cable accel actually in effect
        self._kt_last_print_time = None
        # One-line ~2 Hz health diagnostic (off by default to keep the launch quiet). Enable per
        # drone to trace why one sinks/falls: z vs ref, xy error, throttle (0.6=saturated),
        # thrust/cable feed-forward accel. Set enable_diag_log:=true on the drone of interest.
        self.declare_parameter("enable_diag_log", False)
        self.enable_diag_log = bool(self.get_parameter("enable_diag_log").value)
        self._takeoff_step = None         # cycles since takeoff (None = not spooling)
        self._takeoff_z = None            # spawn z, latched to gate the kT schedule
        self._log_payload = (self.drone_id == 0)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_state_cb, 5)
        if self._log_payload:
            self.create_subscription(
                Float64MultiArray, '/payload/desired_position',
                self._payload_ref_cb, 5)

        # ── Flight envelope (finding F3) ──────────────────────────────────
        # Any FAULT disarms this drone; the fleet manager already propagates an
        # unexpected disarm to everyone else via /drone_N/arming_state_feedback.
        # See docs/design/fleet_safety.md.
        self._init_safety()

        # ── MPC ───────────────────────────────────────────────────────────
        self.N = 20
        # NB no skip_steps here (unlike controller_ukf / controller_mpc_payload):
        # the planner publishes at this MPC's own 0.1 s node spacing, so planner
        # node j maps directly to stage j. A `self.skip_steps = 3` used to sit here,
        # assigned and never read; it was removed 2026-08-04 after being wrongly
        # listed as a tracking-lag suspect in DISSIPATIVE_TRACKING_ISSUE.md.
        self.first_solve = True
        # Heading the tracker holds (rad). Re-latched from mocap while the drone
        # rests armed, then frozen at TAKEOFF -- see control_loop.
        self._heading_datum = 0.0
        fresh = _solver_is_fresh()
        if fresh:
            self.get_logger().info("[acados] loading cached quad_load_dynamics solver.")
        else:
            self.get_logger().info("[acados] compiling quad_load_dynamics solver (sources changed)...")
        self.ocp = generate_ocp_controller(generate=not fresh, build=not fresh)
        fcntl.flock(_lock_file, fcntl.LOCK_UN)
        self.est_params = np.array([self.thrust_ratio, 0.0, 0.12, 70.0, 670.0, 0.5])

        # ── Logging ───────────────────────────────────────────────────────
        log_headers = [
            'step', 'sim_time', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
            # desired reference position (node 0) - plot_run.py reads ref_* to
            # overlay desired vs actual trajectory.
            'ref_x', 'ref_y', 'ref_z',
            # adaptive thrust ratio: kT the MPC flew on, the filter's estimate and
            # spread, the raw per-sample measurement, and the open-loop schedule
            # for comparison.
            # kt_aux is backend-specific: UKF -> estimated drag_z, IMU -> raw kT sample
            'kt_mpc', 'kt_est', 'kt_est_std', 'kt_aux', 'kt_scheduled',
            'kt_update_count', 'kt_status',
        ]
        # drone 0 also logs the payload actual + desired so plot_run.py can
        # overlay the load track alongside the drones.
        if self._log_payload:
            log_headers += ['payload_x', 'payload_y', 'payload_z',
                            'payload_ref_x', 'payload_ref_y', 'payload_ref_z']
        self.data_logger = DataLogger(
            LOGGING_NAME, f"planner_drone{self.drone_id}", log_headers,
            base_dir=LOG_BASE_DIR)
        # Record what this run was actually configured with, next to the CSV.
        write_params(self.data_logger.log_dir, node_params(self, {
            'node': 'controller', 'frequency_hz': FREQUENCY_HZ,
            'horizon_N': self.N}))

        self.observed_state_history = []
        self.control_history = []
        self.estimated_state_history = []

        # ── Control timer ─────────────────────────────────────────────────
        self.timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info(
            f"[Drone {self.drone_id}] Controller ready. "
            f"Offset=({self.offset_x}, {self.offset_y}).")

    # ─────────────────────────────────────────────────────────────────────
    # Fleet step callback
    # ─────────────────────────────────────────────────────────────────────

    def _fleet_step_callback(self, msg: Int32):
        """Shadow the fleet manager's master step counter."""
        new_step = msg.data
        if new_step != self.step_counter:
            self.step_counter = new_step

    def _planner_ref_callback(self, msg: Float64MultiArray):
        """Parse [n_nodes, dt, px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, ...] from
        controller_load_mpc (12 fields/node: position, velocity, specific thrust
        acceleration feedforward, and cable tension acceleration t*s/m)."""
        data = msg.data
        if len(data) < 2:
            return
        n_nodes = int(data[0])
        if n_nodes < 1:
            return
        fields = (len(data) - 2) // n_nodes      # 12 (cable-aware) or 9 (legacy)
        if n_nodes < self.N + 1 or fields < 9 or len(data) < 2 + fields * n_nodes:
            return
        arr = np.array(data[2:2 + fields * n_nodes],
                       dtype=float).reshape(n_nodes, fields)
        self.planner_ref_pos = arr[:, 0:3]
        self.planner_ref_vel = arr[:, 3:6]
        self.planner_ref_acc = arr[:, 6:9]
        self.planner_ref_cable = arr[:, 9:12] if fields >= 12 else None
        # Wall clock, matching the pose watchdog: a stale reference must be
        # detected in real time even when sim time is running slow.
        self._last_ref_wall = self._wall_clock.now()

    def _imu_callback(self, msg: Imu):
        """Store the body-frame linear acceleration (specific force) from the
        drone IMU. At rest this reads [0,0,+9.81]; in flight it is
        (thrust + cable_force)/m expressed in the body frame."""
        a = np.array([msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z])
        if self.imu_lin_acc is None:
            self.imu_lin_acc = a
        else:
            self.imu_lin_acc += self.imu_alpha * (a - self.imu_lin_acc)

    def _payload_state_cb(self, msg: MotionCaptureState):
        p = msg.pose.position
        self.payload_pos = np.array([p.x, p.y, p.z])
        self.payload_resting = (p.z <= self.payload_rest_z + 0.05)
        # Payload attitude is the attach-failure signature (it ran past 90 deg in
        # the ring-attach runaway), so the envelope check needs it.
        q = msg.pose.orientation
        self.payload_quat = (q.w, q.x, q.y, q.z)

    # ── Flight envelope (F3) ─────────────────────────────────────────────────

    def _init_safety(self):
        """Declare envelope parameters and build the checker.

        There is deliberately NO geofence -- the operator holds the kill switch and
        is the out-of-bounds failsafe (Wesley, 2026-08-04). What remains is what a
        human cannot react to in time or cannot see at all: reference staleness,
        tilt and speed. See docs/design/fleet_safety.md."""
        p = self.declare_parameter
        self.safety_enabled = bool(p('safety_enabled', True).value)
        lim = EnvelopeLimits(
            max_tilt_deg=float(p('max_tilt_deg', 60.0).value),
            max_payload_tilt_deg=float(p('max_payload_tilt_deg', 60.0).value),
            warn_tilt_deg=float(p('warn_tilt_deg', 40.0).value),
            max_speed=float(p('max_speed', 3.0).value),
            ref_timeout_s=float(p('safety_ref_timeout_s', 1.0).value),
            warn_battery_v=float(p('warn_battery_v', 15.0).value),
        )
        self.envelope = EnvelopeChecker(lim)
        self.payload_quat = None
        self._last_ref_wall = None
        self._safety_warn_ctr = 0
        self._aborted = False
        self._was_armed = False
        # Fleet-wide abort: the manager broadcasts here so a healthy drone stops
        # even when its own envelope is fine.
        self.create_subscription(String, '/fleet/abort', self._fleet_abort_cb, 5)
        self.get_logger().info(
            f"[Drone {self.drone_id}] envelope: no geofence (operator failsafe); "
            f"tilt<{lim.max_tilt_deg:.0f}/{lim.max_payload_tilt_deg:.0f} deg "
            f"speed<{lim.max_speed:.1f} m/s "
            f"ref_stale>{lim.ref_timeout_s:.1f} s "
            f"({'ENABLED' if self.safety_enabled else 'DISABLED'})")

    def safety_preflight_block(self):
        """Pre-arm interlock, called by CallbackManagerMulti.handle_arming_service.
        Return a reason string to REFUSE arming, or None to allow it.

        Only conditions that are knowable on the ground and would make the flight
        unsafe from the first second. Deliberately narrow: an interlock that blocks
        arming for marginal reasons gets bypassed, and then protects nothing."""
        if not self.safety_enabled:
            return None
        if self.current_pose is None:
            return 'no mocap pose'
        # No position check here: there is no geofence by decision.
        # Stale mocap: the pose watchdog would fire within 0.25 s of arming anyway,
        # so refuse now rather than arm and immediately abort.
        age = (self._wall_clock.now()
               - self.last_pose_update_time).nanoseconds * 1e-9
        if age > POSE_TIMEOUT_THRESHOLD:
            return f'mocap pose is {age:.2f} s stale'
        return None

    def _fleet_abort_cb(self, msg: String):
        """Another drone (or the manager) declared a fault. Stop."""
        if self._aborted:
            return
        self.get_logger().error(
            f"[Drone {self.drone_id}] FLEET ABORT: {msg.data} - disarming.")
        self._do_safety_disarm()

    def _safety_check(self):
        """Run the envelope. Returns True if a fault fired (caller must return)."""
        # Re-arming clears a latched fault. Detected as a rising edge on `armed`
        # because arming is handled inside CallbackManagerMulti's service, so
        # there is no hook to hang this on. Must run before the _aborted guard,
        # or an aborted drone could never be re-armed.
        if self.armed and not self._was_armed:
            self.envelope.reset()
            self._aborted = False
            self.get_logger().info(
                f"[Drone {self.drone_id}] envelope armed and reset.")
        self._was_armed = self.armed

        if not self.safety_enabled or self._aborted:
            return False

        ref_age = None
        if self._last_ref_wall is not None:
            ref_age = (self._wall_clock.now()
                       - self._last_ref_wall).nanoseconds * 1e-9

        pose = self.current_pose
        verdict = self.envelope.check(
            quat=pose[3:7] if pose is not None else None,
            velocity=pose[7:10] if pose is not None else None,
            payload_quat=self.payload_quat,
            ref_age_s=ref_age,
            battery_v=None,          # warn-only, and only meaningful on hardware
            airborne=bool(self.armed and self.takeoff_requested),
        )

        if verdict.is_fault:
            self.get_logger().error(
                f"[Drone {self.drone_id}] ENVELOPE FAULT: {verdict.reason} "
                f"- disarming fleet.")
            self._do_safety_disarm(reason=verdict.reason)
            return True

        if verdict.level == 'WARN':
            self._safety_warn_ctr += 1
            if self._safety_warn_ctr % 25 == 1:      # ~2 Hz at 50 Hz control
                self.get_logger().warn(
                    f"[Drone {self.drone_id}] envelope warning: {verdict.reason}")
        else:
            self._safety_warn_ctr = 0
        return False

    def _do_safety_disarm(self, reason=''):
        """Disarm this drone. The fleet manager sees the arming-state feedback and
        disarms everyone else (main.py `_arming_feedback_callback`), which is the
        propagation path this reuses rather than duplicating."""
        self._aborted = True
        self.cb.disarm(ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                                   channel_2=-1.0, channel_3=0.0))

    def _payload_ref_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.payload_ref = np.array(msg.data[:3], dtype=float)

    def _scheduled_kT(self):
        """Operating-point thrust gain for the assumed linear model a=kT*throttle.

        The sim plant is quadratic a(u)=c*u^2, so the secant gain a/u = c*throttle
        varies with the operating point (~20 at takeoff, ~49 at hover). A single kT
        cannot serve both, AND the takeoff-safe thrust_ratio slightly over-thrusts
        on purpose (oomph to break off the stands). So keep thrust_ratio until the
        drone is clearly airborne, then schedule kT = clip(c*throttle, thrust_ratio,
        KT_SCHED_CEIL) to kill the residual hover over-thrust. Never drops below
        thrust_ratio, so scheduling can never starve takeoff. Disabled when c=0.
        """
        if self.thrust_quad_c <= 0.0 or self.last_cmd_throttle is None \
                or self.current_pose is None:
            return self.thrust_ratio
        z = float(self.current_pose[2])
        if self._takeoff_z is None:
            self._takeoff_z = z
        if z < self._takeoff_z + AIRBORNE_MARGIN:     # still on/near the stands
            return self.thrust_ratio
        kT = self.thrust_quad_c * float(self.last_cmd_throttle)
        return float(np.clip(kT, self.thrust_ratio, KT_SCHED_CEIL))

    def _effective_kT(self):
        """The kT the MPC actually flies on this cycle.

        Preference order: measured (adaptive) > scheduled (thrust_quad_c) > fixed
        thrust_ratio. The adaptive value is slew-limited because this tracker has
        no integrator -- it trusts its thrust feedforward, so a step change in kT
        is a step change in commanded throttle on a drone carrying load.
        """
        base = self._scheduled_kT()
        target, driving = base, False
        if self.adaptive_thrust_ratio and self.adaptive_thrust_feedback:
            estimate, count = self._kt_estimate()
            # Never let the estimate drive takeoff: pre-airborne the fixed
            # thrust_ratio deliberately over-thrusts to break the drones off the
            # stands (see AIRBORNE_MARGIN), and the estimator is not running yet.
            if (estimate is not None and count >= self.kt_min_updates
                    and self._kt_airborne()):
                target = float(np.clip(estimate, self.kt_min, self.kt_max))
                driving = True

        # Slew whenever the adaptive path is involved -- both while FOLLOWING the
        # estimate and while RETURNING to the schedule after it goes away (the node
        # died, the estimate went stale). Stepping back is the same thrust transient
        # as stepping to it: on this no-integrator tracker a 2-unit jump in kT is a
        # ~6% throttle jump on a drone carrying part of a payload.
        if driving or self._kt_was_driving:
            current = float(self.est_params[0])
            step = self.kt_slew_per_s * DT
            value = float(np.clip(target, current - step, current + step))
        else:
            value = target
        # Stay latched until the ramp back to base has actually finished, otherwise
        # the next cycle would jump the remaining distance.
        self._kt_was_driving = driving or abs(value - base) > 1e-9
        return value

    def _maybe_freeze_thrust_ratio(self):
        """Close the learning window once the drone has been airborne long enough and
        the estimate is actually usable. Returns True on the cycle it freezes.

        Deliberately requires kt_min_updates as well as the timer: freezing a
        half-converged estimate would lock in a worse number than the seed, and the
        estimator nodes can fall behind badly on a loaded machine (their update rate
        is not guaranteed), so elapsed time alone is not evidence of convergence."""
        if self.kt_freeze_after_s <= 0.0:
            return False
        if not (self.takeoff_requested and self._kt_airborne()):
            return False
        # Counted in CONTROL CYCLES, not wall seconds. This node runs use_sim_time
        # =False on a wall-clock 50 Hz timer while the drone's physics advance in sim
        # time, so on a loaded machine (Gazebo below real-time) wall seconds and
        # flight seconds diverge -- a wall timer would close the window early, part
        # way through the lift. Cycles are the tracker's own cadence and are what the
        # estimate actually accumulates against.
        self._kt_airborne_cycles += 1
        if self._kt_airborne_cycles < self.kt_freeze_after_s * FREQUENCY_HZ:
            return False
        estimate, count = self._kt_estimate()
        if estimate is None or count < self.kt_min_updates:
            # Window elapsed but there is nothing worth locking (node down, or not
            # enough updates yet). Keep learning -- locking a half-converged value
            # would be worse than the seed. Surfaced as "lock PENDING" in the report;
            # the status itself is left to the backend, which says WHY.
            return False
        self._kt_frozen_value = float(np.clip(estimate, self.kt_min, self.kt_max))
        self._kt_status = 'frozen'
        self.get_logger().info(
            f"[Drone {self.drone_id}] kT LOCKED at {self._kt_frozen_value:.2f} "
            f"after {self._kt_airborne_cycles} airborne control cycles "
            f"(~{self._kt_airborne_cycles / FREQUENCY_HZ:.1f}s, {count} updates, "
            f"seed {self.kt_seed:.2f}). Estimation stops here; the estimator node "
            f"goes idle for the rest of the flight.")
        return True

    def _kt_estimate(self):
        """(kT estimate, update count) from whichever backend is configured."""
        if self._kt_frozen_value is not None:
            # Frozen: report a count past kt_min_updates so the MPC keeps flying it,
            # and bypass the staleness check -- the node is meant to be silent now.
            return self._kt_frozen_value, max(self.kt_min_updates, 1)
        # Only ever act on a FRESH estimate: if the node dies the last value would
        # otherwise be flown indefinitely as though it were current.
        if self._kt_node_value is None or not self._kt_node_fresh():
            return None, 0
        return self._kt_node_value, self._kt_node_count

    def _kt_estimate_std(self):
        return self._kt_node_std

    def _kt_airborne(self):
        """True once the drone is clearly off its stand. Same AIRBORNE_MARGIN the
        kT schedule uses: on a taut air-start the stands carry the weight, so a
        thrust measurement taken there is not the free-flight thrust and would
        bias kT low. Deliberately uses its own spawn-z latch rather than
        _scheduled_kT's, which is latched lazily (first cycle with an applied
        throttle, i.e. already post-takeoff) and only when thrust_quad_c > 0."""
        if self.current_pose is None or self._kt_spawn_z is None:
            return False
        return float(self.current_pose[2]) >= self._kt_spawn_z + AIRBORNE_MARGIN

    def update_thrust_ratio_estimate(self):
        """One adaptive-kT step: hand this cycle's sample to the estimator node."""
        if not self.adaptive_thrust_ratio or self.thrust_ratio_estimator == 'none':
            self._kt_status = 'disabled'
            return
        if self._kt_frozen_value is not None:
            # Learn-then-lock: the window has closed. Nothing is estimated, nothing is
            # published, and the estimator node idles.
            self._kt_status = 'frozen'
            return
        if self._maybe_freeze_thrust_ratio():
            return
        self._update_thrust_ratio_ukf()

    def _kt_estimate_cb(self, msg: Float64MultiArray):
        """[kT, std, update_count, solve_ms, status_code] from the estimator node."""
        d = msg.data
        if len(d) < 5:
            return
        value = float(d[0])
        if not math.isfinite(value):
            return
        # Re-clamp on arrival: the band is the tracker's guarantee, so it does not
        # depend on the other process having been configured with the same seed.
        self._kt_node_value = float(np.clip(value, self.kt_min, self.kt_max))
        self._kt_node_std = float(d[1])
        self._kt_node_count = int(d[2])
        self._ukf_solve_ms = float(d[3])
        self._kt_node_time = time.monotonic()

    def _kt_node_fresh(self):
        return (self._kt_node_time is not None
                and (time.monotonic() - self._kt_node_time)
                <= self.kt_estimate_timeout_s)

    def _publish_kt_input(self):
        """Feed the estimator node one sample. Cheap: a 26-double message.

        Layout must match thrust_ratio_node.IN_LEN:
            [t, pose(13), u_state(4), u_rate(4), a_cable(3), airborne]
        The node brackets consecutive samples into one propagation interval, so what
        matters is that pose and the applied control here describe the SAME instant.
        """
        if self._kt_input_pub is None or self.current_pose is None:
            return
        a_cable = (self._kt_cable_world
                   if self._kt_cable_world is not None and not self.payload_resting
                   else np.zeros(3))
        msg = Float64MultiArray()
        msg.data = (
            [time.monotonic()]
            + [float(v) for v in self.current_pose[:N_POSE]]
            + [float(v) for v in np.asarray(self._applied_u, dtype=float)]
            + [float(v) for v in np.asarray(self._applied_u_rate, dtype=float)]
            + [float(v) for v in np.asarray(a_cable, dtype=float)]
            + [1.0 if (self.takeoff_requested and self._kt_airborne()) else 0.0])
        self._kt_input_pub.publish(msg)

    def _update_thrust_ratio_ukf(self):
        """Out-of-loop backend: publish this cycle's sample, then note freshness.

        No filtering happens here -- that is the whole point. The 39-propagation UKF
        runs in thrust_ratio_node.py so it can never stall this 50 Hz loop.
        """
        self._publish_kt_input()
        if not self.takeoff_requested:
            self._kt_status = 'waiting_for_takeoff'
        elif not self._kt_airborne():
            self._kt_status = 'waiting_for_airborne'
        elif self._kt_node_time is None:
            self._kt_status = 'no_estimator_node'
        elif not self._kt_node_fresh():
            self._kt_status = 'estimate_stale'
        else:
            self._kt_status = 'updated'

    def report_thrust_ratio(self):
        """~1 Hz one-liner so the estimate can be eyeballed against the plant."""
        if self.kt_print_period_s <= 0.0 or not self.adaptive_thrust_ratio:
            return
        now = time.monotonic()
        if (self._kt_last_print_time is not None
                and now - self._kt_last_print_time < self.kt_print_period_s):
            return
        self._kt_last_print_time = now

        estimate, count = self._kt_estimate()
        driving = (self.adaptive_thrust_feedback
                   and estimate is not None
                   and count >= self.kt_min_updates
                   and self._kt_airborne())
        thr = (float(self.last_cmd_throttle)
               if self.last_cmd_throttle is not None else float('nan'))
        shown = float('nan') if estimate is None else float(estimate)
        common = (f"MPC={float(self.est_params[0]):6.2f} "
                  f"{'ADAPTIVE' if driving else 'sched/fixed'} | "
                  f"sched={self._scheduled_kT():6.2f} "
                  f"fixed={self.thrust_ratio:5.2f} | thr={thr:.3f}")

        # The estimator node prints the parameter detail (drag/tau/innovation); here
        # we only report what this tracker is actually flying on.
        if self._kt_frozen_value is not None:
            self.get_logger().info(
                f"[kT d{self.drone_id}] LOCKED at {self._kt_frozen_value:6.2f} | "
                f"{common} | learn window closed, estimator idle "
                f"[{self._kt_status}]")
        else:
            age = ('  --' if self._kt_node_time is None
                   else f'{time.monotonic() - self._kt_node_time:4.2f}s')
            if self.kt_freeze_after_s <= 0.0:
                left = ''
            else:
                remain = (self.kt_freeze_after_s
                          - self._kt_airborne_cycles / FREQUENCY_HZ)
                left = (f' lock in {remain:4.1f}s |' if remain > 0.0
                        else ' lock PENDING (estimate not usable yet) |')
            self.get_logger().info(
                f"[kT d{self.drone_id}] node est={shown:6.2f} "
                f"+/-{self._kt_estimate_std():4.2f} | {common} | "
                f"band {self.kt_min:.1f}..{self.kt_max:.1f} |{left} age={age} "
                f"node={self._ukf_solve_ms:5.1f}ms | n={count} [{self._kt_status}]")

    def _measured_heading(self):
        """Current yaw (rad, world frame) from the mocap quaternion. This is the
        heading the tracker holds -- latched while resting, see control_loop."""
        if self.current_pose is None:
            return self._heading_datum
        w, x, y, z = (float(self.current_pose[3]), float(self.current_pose[4]),
                      float(self.current_pose[5]), float(self.current_pose[6]))
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def measured_cable_accel(self):
        """MEASURED cable acceleration in the WORLD frame from the IMU:
        a_cable = R * (imu_specific_force - [0,0, kT*throttle]).
        Returns None until an IMU sample and an applied throttle are available."""
        if self.imu_lin_acc is None or self.current_pose is None:
            return None
        thr = self.last_cmd_throttle
        if thr is None:
            return None
        q = self.current_pose[3:7]          # [qw, qx, qy, qz]
        Rmat = R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        kT = float(self.est_params[0])      # thrust_ratio (accel per unit throttle)
        thrust_body = np.array([0.0, 0.0, kT * float(thr)])
        return Rmat @ (self.imu_lin_acc - thrust_body)

    def _condition_measured_cable(self, ac):
        """Clamp + slew-limit the measured cable accel before it enters the MPC
        model, so the IMU->model->command->IMU feedback cannot spike or slam the
        solver. Updates and returns the running applied value."""
        m = float(np.linalg.norm(ac))
        if m > CABLE_ACCEL_CAP:
            ac = ac * (CABLE_ACCEL_CAP / m)
        prev = self._applied_cable_vec
        max_step = CABLE_SLEW * DT
        delta = ac - prev
        dn = float(np.linalg.norm(delta))
        if dn > max_step:
            ac = prev + delta * (max_step / dn)
        self._applied_cable_vec = ac
        return ac

    # ─────────────────────────────────────────────────────────────────────
    # Main control loop
    # ─────────────────────────────────────────────────────────────────────

    def control_loop(self):
        # ── Shutdown ──────────────────────────────────────────────────────
        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        # ── Pose timeout watchdog (wall time) ─────────────────────────────
        elapsed = (self._wall_clock.now() - self.last_pose_update_time).nanoseconds * 1e-9
        if self.armed and elapsed > POSE_TIMEOUT_THRESHOLD:
            self.get_logger().error(
                f"[Drone {self.drone_id}] Pose timeout ({elapsed:.2f}s) - disarming.")
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                              channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return

        # ── Armed + pose available ─────────────────────────────────────────
        if self.armed and self.current_pose is not None:

            # Flight envelope (F3). Runs before any control work: if the fleet is
            # outside its envelope there is nothing worth computing.
            if self._safety_check():
                return

            # Latch the resting height for the adaptive-kT airborne gate. Keep
            # re-latching while the fleet sits armed waiting for TAKEOFF, so this
            # ends up at the true stand height whenever takeoff actually fires.
            if not self.takeoff_requested:
                self._kt_spawn_z = float(self.current_pose[2])
                # Same for the HEADING the drone is resting at. The planner's
                # reference carries position/velocity/thrust but no yaw, and the
                # attitude feedforward is a yaw-free tilt (= a commanded heading of
                # world +x), so without this every drone spins to +x during takeoff
                # -- worst near 180 deg, where the sign-invariant attitude error is
                # degenerate and it can turn the long way round. Holding the resting
                # heading also keeps a mocap yaw offset from becoming a takeoff spin.
                self._heading_datum = self._measured_heading()

            # Assumed thrust gain for this cycle: the measured (adaptive) value
            # once the filter has converged, else the operating-point schedule,
            # else the fixed thrust_ratio. No-op during takeoff/resting.
            self.est_params[0] = self._effective_kT()

            # ── Set MPC reference (external planner) ──────────────────────
            # Track the streamed receding-horizon reference. There is no fixed
            # trajectory length / landing phase here: the fleet manager arms and
            # lands the fleet and the planner drives the timeline.
            if self.planner_ref_pos is None:
                # No reference streamed yet - hold armed-idle on the ground.
                self.cb.cmd_publisher_.publish(ELRSCommand(
                    armed=True, channel_0=0.0, channel_1=0.0,
                    channel_2=-1.0, channel_3=0.0))
                return
            # Diagnostic toggles: scale/zero the cable model term, and/or
            # replace the tilt attitude FF with a level one (vertical accel of
            # the same magnitude -> identity tilt quat, throttle FF preserved).
            ref_acc = self.planner_ref_acc
            if not self.attitude_ff and ref_acc is not None:
                ref_acc = np.stack([[0.0, 0.0, float(np.linalg.norm(a))]
                                    for a in self.planner_ref_acc])
            ref_cable = self.planner_ref_cable
            if ref_cable is not None and self.cable_ff_scale != 1.0:
                ref_cable = ref_cable * self.cable_ff_scale
            # Part 2c: optionally replace the open-loop model cable term with
            # the IMU-measured f_ext, held constant across the horizon. Falls
            # back to the model until a valid measurement is available.
            if self.cable_source == "measured":
                ac_meas = self.measured_cable_accel()
                if ac_meas is not None:
                    # Seed the slew from the model value on the first measured
                    # cycle so the model->measured switch ramps in gently
                    # instead of stepping (e.g. 3.08 -> 1.5) in one tick.
                    if self._applied_cable_vec is None:
                        self._applied_cable_vec = (
                            ref_cable[0].copy() if ref_cable is not None
                            else np.zeros(3))
                    ac_cond = self._condition_measured_cable(ac_meas)
                    rows = (self.planner_ref_pos.shape[0]
                            if self.planner_ref_pos is not None else self.N + 1)
                    ref_cable = np.tile(ac_cond, (rows, 1)) * self.cable_ff_scale
            # payload resting on the ground -> the cable carries ~no tension,
            # so zero the cable term. a_cable=0 in the same model IS the
            # resting (free-flight) dynamics, so no separate solver is needed;
            # it switches back on by itself once the load lifts off.
            if self.payload_resting and ref_cable is not None:
                #self.get_logger().info("PAYLOAD RESTING")
                ref_cable = np.zeros_like(ref_cable)
            #else:
                #self.get_logger().info("PAYLOAD LIFTED")
            # Remember what the model actually received, so the diag |aC|
            # reflects the applied value under either cable_source.
            self._applied_cable0 = (float(np.linalg.norm(ref_cable[0]))
                                    if ref_cable is not None else 0.0)
            # PHYSICAL cable pull for the kT estimator: the planner's own t*s/m,
            # unscaled by the cable_ff_scale A/B knob and never the measured
            # variant (which is itself derived from the IMU using the current kT).
            self._kt_cable_world = (
                np.asarray(self.planner_ref_cable[0], dtype=float)
                if self.planner_ref_cable is not None else None)
            set_planner_reference(
                self.ocp, self.planner_ref_pos, self.planner_ref_vel,
                ref_acc, self.N, self.est_params,
                ref_cable=ref_cable, heading=self._heading_datum)
            # desired reference position now (node 0) for the log / plot
            self._current_ref_pos = np.asarray(self.planner_ref_pos[0], float)

            estimated_state = copy.deepcopy(self.current_pose[:13])

            # u_state is a MODEL STATE: it represents where the four channels
            # physically are right now. So pin it to what was actually SENT to
            # the FC, not to what the solver last computed. Those differ whenever
            # we don't publish the solution — pre-TAKEOFF we publish idle
            # (0, 0, throttle 0, 0) while the solver keeps converging to the full
            # loaded-hover command. Pinning the solved value told the MPC the
            # actuators were already at hover throttle and ~14 deg of pitch while
            # they sat at zero, so the whole command was waiting at full value the
            # instant TAKEOFF flipped, and the drones leapt (measured: 0.77 m/s
            # climb against a reference moving at 0.00, overshooting 0.51 m).
            # Pinned to the applied value, u_state starts at zero and the solver
            # ramps it up under its own u_dot bounds (throttle 0.5/s), which is a
            # correct soft-start rather than a bolted-on one.
            estimated_state_with_control = np.concatenate(
                (estimated_state, self._applied_u))

            relaxation_factor = 0.025
            relaxed_lbx = estimated_state_with_control * (1 - relaxation_factor)
            relaxed_ubx = estimated_state_with_control * (1 + relaxation_factor)
            self.ocp.set(0, "lbx", relaxed_lbx)
            self.ocp.set(0, "ubx", relaxed_ubx)

            # ── Warm start ────────────────────────────────────────────────
            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            # ── Solve ─────────────────────────────────────────────────────
            t0 = time.perf_counter()
            status = self.ocp.solve()
            solve_ms = (time.perf_counter() - t0) * 1000
            #self.get_logger().info(f"Solve time: {solve_ms:.2f}ms")
            if status != 0:
                # NON-FATAL: a single bad solve should not kill the node. Hold the
                # last good command, re-seed the initial guess (a bad warm start
                # perpetuates failure), and only disarm after sustained failure.
                self._solve_fail_ct += 1
                self.get_logger().warn(
                    f"[Drone {self.drone_id}] acados status {status} "
                    f"(consecutive fail {self._solve_fail_ct}) - holding last command.")
                self.first_solve = True   # force set_initial_guess next cycle
                if self._solve_fail_ct > MAX_CONSEC_SOLVE_FAILS:
                    self.get_logger().error(
                        f"[Drone {self.drone_id}] {self._solve_fail_ct} consecutive "
                        f"solver failures - disarming for safety.")
                    self.cb.disarm(ELRSCommand(
                        armed=False, channel_0=0.0, channel_1=0.0,
                        channel_2=-1.0, channel_3=0.0))
                    return
                if self._last_good_msg is not None:
                    self.cb.cmd_publisher_.publish(self._last_good_msg)
                return
            self._solve_fail_ct = 0

            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")

            # ~2 Hz planner diagnostic. ez = z error, thr = throttle (0.6=sat),
            # |aT| = thrust-ff accel (>9.81), |aC| = cable accel (0 = gate shut)
            if self.takeoff_requested and self.enable_diag_log:
                self._diag_ctr = getattr(self, "_diag_ctr", 0) + 1
                if self._diag_ctr % 15 == 0:
                    zc = float(self.current_pose[2])
                    zr = float(self.planner_ref_pos[0][2])
                    aT = float(np.linalg.norm(self.planner_ref_acc[0]))
                    # applied cable accel (imu value if measured, else the model)
                    aC = getattr(self, "_applied_cable0", 0.0)
                    # horizontal tracking - the "fly outward" failure doesn't show
                    # in z. exy = xy error, rdrift = drone radius - ref radius
                    # (>0 = flying outward, <0 = pulled in)
                    cxy = np.array([float(self.current_pose[0]),
                                    float(self.current_pose[1])])
                    rxy = np.array([float(self.planner_ref_pos[0][0]),
                                    float(self.planner_ref_pos[0][1])])
                    exy = float(np.linalg.norm(rxy - cxy))
                    rdrift = float(np.linalg.norm(cxy) - np.linalg.norm(rxy))
                    # imu measured cable accel vs applied model: |aCm| = measured,
                    # dC = |measured - modelled| (small once they agree)
                    ac_meas = self.measured_cable_accel()
                    if ac_meas is not None:
                        ac_mod = (self.planner_ref_cable[0] * self.cable_ff_scale
                                  if self.planner_ref_cable is not None
                                  else np.zeros(3))
                        aCm = float(np.linalg.norm(ac_meas))
                        dC = float(np.linalg.norm(ac_meas - ac_mod))
                    else:
                        aCm = float('nan')
                        dC = float('nan')
                    thr = float(u[2])
                    # full REFERENCE (what the network/planner is asking of this drone) vs
                    # ACTUAL (measured pose) + per-axis error, so a divergence shows exactly
                    # where the drone goes vs where it is commanded. u = [roll, pitch, thr, yaw].
                    rp = self.planner_ref_pos[0]
                    rv = (self.planner_ref_vel[0] if self.planner_ref_vel is not None
                          else np.zeros(3))
                    cp = self.current_pose
                    ex, ey, ezz = (float(rp[0] - cp[0]), float(rp[1] - cp[1]),
                                   float(rp[2] - cp[2]))
                    self.get_logger().info(
                        f"[diag d{self.drone_id}] REF p=({rp[0]:+.2f},{rp[1]:+.2f},"
                        f"{rp[2]:+.2f}) v=({rv[0]:+.2f},{rv[1]:+.2f},{rv[2]:+.2f}) | "
                        f"ACT p=({cp[0]:+.2f},{cp[1]:+.2f},{cp[2]:+.2f}) | "
                        f"ERR=({ex:+.2f},{ey:+.2f},{ezz:+.2f}) |exy|={exy:.2f}")
                    self.get_logger().info(
                        f"[diag d{self.drone_id}] CMD roll={float(u[0]):+.2f} "
                        f"pitch={float(u[1]):+.2f} yaw={float(u[3]):+.2f} "
                        f"thr={thr:.3f}{' SAT' if thr >= 0.59 else ''} | "
                        f"|aT|={aT:.2f} |aC|={aC:.2f} |aCm|={aCm:.2f} dC={dC:.2f} "
                        f"rdrift={rdrift:+.2f}")

            # ── Adaptive kT ───────────────────────────────────────────────
            # Runs BEFORE this cycle's command is applied, so it pairs the
            # current IMU sample with the throttle that produced it (the one sent
            # last cycle). Cheap: a scalar KF update, no solver involved.
            self.update_thrust_ratio_estimate()
            self.report_thrust_ratio()

            # ── Publish command ───────────────────────────────────────────
            if self.takeoff_requested:
                thr = float(u[2])
                # takeoff spool-up: for the first takeoff_spool_s, scale the
                # throttle up from 0 to the commanded value so thrust rises
                # smoothly and the drones ease off the platforms. Raised-cosine
                # (not linear): the applied throttle jumps from idle-0 to the full
                # MPC hover value in one cycle when the spool is off/too short, and
                # a linear ramp still kinks the acceleration at both ends. The taut
                # air-start feels every bit of that as an initial lurch-and-bounce.
                # A cosine has zero slope at both ends, so thrust eases on and
                # settles onto hover without a step.
                if self.takeoff_spool_s > 0.0:
                    if self._takeoff_step is None:
                        self._takeoff_step = 0
                    spool_cycles = max(1.0, self.takeoff_spool_s * FREQUENCY_HZ)
                    if self._takeoff_step < spool_cycles:
                        ease = 0.5 * (1.0 - np.cos(
                            np.pi * self._takeoff_step / spool_cycles))
                        # ease from FLOOR*u up to u (not 0*u): keep enough thrust to
                        # hold the load taut from cycle 0, see TAKEOFF_SPOOL_FLOOR.
                        frac = TAKEOFF_SPOOL_FLOOR + (1.0 - TAKEOFF_SPOOL_FLOOR) * ease
                        thr *= frac
                        self._takeoff_step += 1
                # Record the throttle actually applied to the FC so the next IMU
                # sample can be decomposed into thrust + cable (measured_cable_accel)
                # and, inversely, into a kT measurement (update_thrust_ratio_estimate).
                self.last_cmd_throttle = thr
                msg = ELRSCommand(
                    armed=True,
                    channel_0=round(u[0], 3),
                    channel_1=round(u[1], 3),
                    channel_2=round((thr * 2) - 1, 3),
                    channel_3=round(u[3], 3))
                self._applied_u = np.array(
                    [float(u[0]), float(u[1]), float(thr), float(u[3])])
                self._applied_u_rate = np.asarray(u_rate, dtype=float).copy()
                self.get_logger().debug(
                    f"[Drone {self.drone_id}] r:{u[0]:.3f} p:{u[1]:.3f} "
                    f"t:{u[2]:.3f} y:{u[3]:.3f}")
            else:
                self._takeoff_step = None    # reset so the next takeoff spools again
                # Drop any in-flight kT estimate on LAND/re-arm: the next takeoff
                # must start from the takeoff-safe seed, not from a hover-tuned
                # value measured while the load was airborne.
                # The estimator node owns its own filter state; here just drop the
                # cached value so a stale pre-landing estimate cannot drive the next
                # takeoff before a fresh one arrives.
                self._kt_node_value = None
                self._kt_node_time = None
                self._kt_node_count = 0
                # Unlock: the next takeoff re-learns. A kT locked on the last flight
                # is not evidence about this one -- the battery is more discharged and
                # the payload may have changed.
                if self._kt_frozen_value is not None:
                    self.get_logger().info(
                        f"[Drone {self.drone_id}] kT unlocked "
                        f"(was {self._kt_frozen_value:.2f}); re-learning next takeoff.")
                self._kt_frozen_value = None
                self._kt_airborne_cycles = 0
                self.last_cmd_throttle = None
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                                  channel_2=-1.0, channel_3=0.0)
                # channel_2 = -1.0 maps to throttle 0, so every channel is at zero
                self._applied_u = np.zeros(4)
                self._applied_u_rate = np.zeros(4)
                # THROTTLED: this branch runs every cycle of the 50 Hz loop for
                # as long as the fleet sits armed waiting for TAKEOFF. Logging it
                # unthrottled put 50 lines/s PER DRONE through the launch stdout
                # pipe (200/s with four drones); when that pipe backs up the write
                # blocks the timer, which stalls the single-threaded executor, so
                # the node can't service the /drone_N/command subscription that
                # TAKEOFF arrives on. That showed up as TAKEOFF taking seconds to
                # be acted on while ARM was instant. Once every 2 s is plenty.
                self.get_logger().info(
                    f"[Drone {self.drone_id}] Armed - waiting for TAKEOFF command.",
                    throttle_duration_sec=2.0)

            self.cb.cmd_publisher_.publish(msg)
            # Remember this good command so a later failed solve can hold it.
            if self.takeoff_requested:
                self._last_good_msg = msg

            # ── Visualisation ─────────────────────────────────────────────
            mpc_trajectory = np.zeros((13, self.N))
            for i in range(self.N):
                x_i = self.ocp.get(i, "x")
                mpc_trajectory[:, i] = x_i[:13]
            mpc_trajectory[:, 0] = self.current_pose[:13]

            # Only publish the plan once we're actually flying it. Pre-TAKEOFF
            # u_state is pinned to the applied idle (throttle 0 -- see the
            # stage-0 pin above), so the model correctly predicts free fall and,
            # with u_dot capped at 0.5/s, cannot recover inside the 2 s horizon:
            # the plan dives ~4 m through the floor. That is an honest answer to
            # "what if the throttle stayed at zero", but it never happens (the
            # drones rest on platforms and the throttle ramps at TAKEOFF), so
            # drawing it in RViz is just misleading.
            if self.takeoff_requested:
                self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            # NOTE: the map -> drone_N_mocap TF is deliberately NOT broadcast
            # here. simulation_communication/fleet_viz owns it, so the RViz
            # stack can come up and show the fleet BEFORE any controller runs.
            # Broadcasting from both would just spam TF_REPEATED_DATA.
            self.trajectory_visualizer.publish_actual_path(self.current_pose)

            # ── Logging ───────────────────────────────────────────────────
            sim_time_sec = self.get_clock().now().nanoseconds * 1e-9
            ref = (self._current_ref_pos if self._current_ref_pos is not None
                   else np.full(3, np.nan))
            kt_est, kt_n = self._kt_estimate()
            log_row = [
                self.step_counter, sim_time_sec,
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]),
                float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]),
                float(self.current_pose[5]), float(self.current_pose[6]),
                float(ref[0]), float(ref[1]), float(ref[2]),
                float(self.est_params[0]),
                float(kt_est if kt_est is not None else np.nan),
                float(self._kt_estimate_std()),
                # the estimator node's update time in ms -- the cost that forced it
                # out of the control loop, worth keeping an eye on.
                float(self._ukf_solve_ms),
                float(self._scheduled_kT()),
                int(kt_n),
                str(self._kt_status),
            ]
            if self._log_payload:
                pa = (self.payload_pos if self.payload_pos is not None
                      else np.full(3, np.nan))
                pr = (self.payload_ref if self.payload_ref is not None
                      else np.full(3, np.nan))
                log_row += [float(pa[0]), float(pa[1]), float(pa[2]),
                            float(pr[0]), float(pr[1]), float(pr[2])]
            self.data_logger.append_row(log_row)

            self.control_history.append(np.concatenate((u, u_rate)).tolist())
            self.observed_state_history.append(self.current_pose[:13].tolist())
            self.estimated_state_history.append(estimated_state[:13].tolist())

        # ── Disarmed / no pose ─────────────────────────────────────────────
        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                              channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            # Do NOT reset step_counter here - fleet manager owns it

    # ─────────────────────────────────────────────────────────────────────
    # Shutdown helpers
    # ─────────────────────────────────────────────────────────────────────

    def signal_handler(self, sig, frame):
        print(f"[Drone {self.drone_id}] Interrupt received, shutting down...")
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if getattr(self, 'on_close_called', False):
            return
        self.on_close_called = True
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                          channel_2=-1.0, channel_3=0.0)
        self.cb.cmd_publisher_.publish(msg)
        self.data_logger.close()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print(f"[Drone {controller.drone_id}] Keyboard interrupt received.")
    except Exception as e:
        print(f"[Drone {controller.drone_id}] Exception: {e}")
    finally:
        controller.on_close()
        try:
            controller.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()