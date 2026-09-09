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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from . import acados as _acados_mod
from .acados import (generate_ocp_controller, set_initial_guess,
                     warm_start_from_previous_solution, set_planner_reference)
from .velocity_loop import VelocityLoop
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

# Height above spawn z at which the drone counts as AIRBORNE, which is when kT
# switches from takeoff_thrust_ratio to thrust_ratio.
#
# FUNCTIONAL, not cosmetic. On a taut air-start the drones rest on stands that mask
# their weight-support need. Assuming a kT BELOW the truth makes the MPC over-throttle,
# which POPS the drones off the stands and tensions the cables -> the load lifts and
# stays taut. That pop IS the takeoff surge. With a kT equal to the true hover gain the
# drones hold exactly at spawn, never pop off, never tension the cables, the load droops
# to the floor and gets abandoned (payload_resting zeros the cable FF -> runaway sag).
# So the surge and the lift are the same event: takeoff_thrust_ratio must be low enough
# to break the drones off their stands, and thrust_ratio takes over once they are up.
# Soften the surge by raising takeoff_thrust_ratio, not by changing this margin.
AIRBORNE_MARGIN = 0.04         # m above spawn z

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
        # ── Thrust gain kT ────────────────────────────────────────────────
        # thrust_ratio (kT): accel per unit throttle the MPC assumes, i.e. the linear
        # model a = kT * throttle. This is the ONE number the tracker flies on --
        # nothing estimates it, schedules it, or overrides it in flight.
        #
        #   HARDWARE: 24.0 -- the measured kT of these airframes on a healthy pack.
        #   SIM:      ~31  -- Gazebo's motor model is QUADRATIC, a = 88.6 * u^2, so
        #             the secant gain a/u that a LINEAR model must use is
        #             sqrt(88.6 * a_hover) ~ 31-33, not 24. Setting 24 in sim
        #             under-assumes thrust by ~25% and the drones shoot up.
        #
        # The two genuinely differ because the sim motor model and the real airframe
        # have different thrust curves; test_config_tools asserts they stay apart.
        self.declare_parameter("thrust_ratio", 24.0)
        self.thrust_ratio = float(self.get_parameter("thrust_ratio").value)
        # kT used BEFORE the drone is airborne (see AIRBORNE_MARGIN). Set it BELOW
        # thrust_ratio to buy a deliberate takeoff over-thrust: on a taut air-start
        # that over-thrust is what breaks the drones off their stands and tensions
        # the cables. <=0 means "same as thrust_ratio", i.e. one constant kT for the
        # whole flight -- which is correct for a ground takeoff (hardware) and the
        # right starting point anywhere the drones are not resting on stands.
        self.declare_parameter("takeoff_thrust_ratio", 0.0)
        _tk = float(self.get_parameter("takeoff_thrust_ratio").value)
        self.takeoff_thrust_ratio = _tk if _tk > 0.0 else self.thrust_ratio
        # ── Battery derate ────────────────────────────────────────────────
        # kT falls as the pack sags. Modelled as a straight line in measured pack
        # voltage between "full" and "empty":
        #
        #   kT        = thrust_ratio * (1 - kt_batt_sag_frac * depletion)
        #   depletion = clip((v_full - v) / (v_full - v_empty), 0, 1)
        #
        # OFF by default (kt_batt_sag_frac = 0). Shipping it on would change every
        # flight on the strength of a number nobody has measured on this rig. To
        # calibrate: hover the same load on a full pack and on a nearly-flat one and
        # compare hover throttle; their ratio is (1 - sag_frac * depletion).
        #
        # Voltage arrives on /drone_N/telemetry (ELRS on hardware, sim_telemetry's
        # placeholder in sim). With NO telemetry the derate is skipped and kT stays
        # at thrust_ratio -- the safe direction, since a missing battery reading can
        # then never make the tracker assume MORE thrust than it was configured with.
        self.declare_parameter("kt_batt_sag_frac", 0.0)
        self.kt_batt_sag_frac = float(self.get_parameter("kt_batt_sag_frac").value)
        self.declare_parameter("kt_batt_v_full", 16.8)     # 4S at 4.20 V/cell
        self.kt_batt_v_full = float(self.get_parameter("kt_batt_v_full").value)
        self.declare_parameter("kt_batt_v_empty", 14.0)    # 4S at 3.50 V/cell
        self.kt_batt_v_empty = float(self.get_parameter("kt_batt_v_empty").value)
        if (self.kt_batt_sag_frac > 0.0
                and self.kt_batt_v_full <= self.kt_batt_v_empty):
            self.get_logger().error(
                f"[Drone {self.drone_id}] kt_batt_v_full "
                f"({self.kt_batt_v_full:.2f}) must exceed kt_batt_v_empty "
                f"({self.kt_batt_v_empty:.2f}); battery derate DISABLED.")
            self.kt_batt_sag_frac = 0.0
        # Written by CallbackManagerMulti.telemetry_callback. Initialised here
        # because the derate reads it every cycle and telemetry may never arrive.
        self.battery_voltage = None
        # Seconds between the one-line kT reports. 0 = silent.
        self.declare_parameter("kt_print_period_s", 1.0)
        self.kt_print_period_s = float(self.get_parameter("kt_print_period_s").value)
        self.get_logger().info(
            f"[Drone {self.drone_id}] kT FIXED at {self.thrust_ratio:.2f} "
            f"(takeoff {self.takeoff_thrust_ratio:.2f}); battery derate "
            + (f"{100 * self.kt_batt_sag_frac:.0f}% across "
               f"{self.kt_batt_v_full:.1f}..{self.kt_batt_v_empty:.1f} V"
               if self.kt_batt_sag_frac > 0.0 else "OFF"))
        self._kt_spawn_z = None           # resting z, re-latched until TAKEOFF
        self._kt_last_print_time = None
        # One-line ~2 Hz health diagnostic (off by default to keep the launch quiet). Enable per
        # drone to trace why one sinks/falls: z vs ref, xy error, throttle (0.6=saturated),
        # thrust/cable feed-forward accel. Set enable_diag_log:=true on the drone of interest.
        self.declare_parameter("enable_diag_log", False)
        self.enable_diag_log = bool(self.get_parameter("enable_diag_log").value)
        self._takeoff_step = None         # cycles since takeoff (None = not spooling)
        self._log_payload = (self.drone_id == 0)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_state_cb, 5)
        if self._log_payload:
            self.create_subscription(
                Float64MultiArray, '/payload/desired_position',
                self._payload_ref_cb, 5)

        # ── Experiment T1: terminal velocity reference (finding F10) ──────
        # `set_planner_reference` has never set yref_N[3:6], so the terminal cost
        # asks for velocity ZERO at the end of every 2 s horizon while the stage
        # costs track ref_vel -- a standing "stop" command on any moving
        # trajectory, and a leading suspect for the 0.589 s tracker lag.
        # Defaults FALSE (historical behaviour) so this changes nothing until it
        # is measured; flip it to run the A/B:
        #   ros2 launch ... mpc_quad_load_launch.py load_traj:=circle terminal_vel_ref:=true
        self.declare_parameter('terminal_vel_ref', False)
        self.terminal_vel_ref = bool(
            self.get_parameter('terminal_vel_ref').value)
        if self.terminal_vel_ref:
            self.get_logger().warn(
                f"[Drone {self.drone_id}] T1 ACTIVE: terminal velocity reference "
                f"enabled (non-default) - this is an experiment, record it.")

        # ── Stage V: which controller turns the reference into channels ───
        # 'mpc'      (default) the acados position-tracking MPC, unchanged.
        # 'velocity' the paper's architecture -- the reference becomes a velocity
        #            command and a velocity -> attitude -> rate loop produces the
        #            channels directly, with no position MPC in between. Quan et
        #            al. Fig. 2 has no such MPC, and R0054-R0065 measure that
        #            inserted stage as +0.54 s of the +0.68 s total lag.
        # 'velocity_after_handover'
        #            MPC through creep and lift, velocity once the dissipative
        #            network takes the fleet over. This is the one that matches the
        #            architecture: the velocity loop has no cable term, so three
        #            independent position servos capsize the load through the lift
        #            (R0067-R0069) -- the same boundary dissipative_only_launch.py
        #            already documents for the network itself. The OCP owns the
        #            break-off; the paper's controller is a transport controller.
        # Design note: docs/design/velocity_loop.md. Defaults to 'mpc' so every
        # verified configuration is byte-unchanged until the flag is thrown.
        MODES = ('mpc', 'velocity', 'velocity_after_handover')
        self.declare_parameter('control_mode', 'mpc')
        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        if self.control_mode not in MODES:
            raise ValueError(
                f"control_mode must be one of {MODES}, got {self.control_mode!r}")
        self.velocity_loop = None
        self._prev_takeoff = False
        # True once the velocity loop is actually producing the channels. Immediate in
        # 'velocity'; set by the handover announcement in 'velocity_after_handover'.
        self._velocity_active = (self.control_mode == 'velocity')
        if self.control_mode != 'mpc':
            for name, default in (('vel_kp_pos', 2.0), ('vel_kv', 4.0),
                                  ('vel_ki', 1.0), ('vel_k_att', 8.0),
                                  ('vel_v_max', 2.0), ('vel_a_i_max', 2.0)):
                self.declare_parameter(name, default)
            gains = {n: float(self.get_parameter(f'vel_{n}').value) for n in
                     ('kp_pos', 'kv', 'ki', 'k_att', 'v_max', 'a_i_max')}
            self.velocity_loop = VelocityLoop(**gains)
            self.get_logger().warn(
                f"[Drone {self.drone_id}] STAGE V: control_mode={self.control_mode} "
                f"({gains}). This is an experiment, record it.")
        if self.control_mode == 'velocity_after_handover':
            # TRANSIENT_LOCAL to match the publisher: the handover fires once, and a
            # tracker that restarted after it must still learn the fleet has moved on.
            self.create_subscription(
                String, '/fleet/control_phase', self._control_phase_cb,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))

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
            # kT the MPC actually flew on this cycle, and the pack voltage it was
            # derated against (NaN until telemetry arrives). kT is fixed per flight
            # unless kt_batt_sag_frac is on, so kt_mpc is flat by design -- a step in
            # it means the takeoff/airborne switch, nothing else.
            'kt_mpc', 'battery_v',
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
        # Mocap watchdog budget. Measured on the WALL clock on purpose (it is about real
        # comms latency), which makes it wrong by construction in a simulator running
        # slower than real time: at Gazebo's ~0.3x realtime, 0.25 s of wall is under four
        # mocap periods, so ordinary scheduling jitter reads as a dead link and disarms a
        # healthy drone. It did, ~1 s after ARM, in 6 of today's runs. Sim launches raise
        # it; hardware keeps 0.25 s, where wall and sim time are the same thing.
        self.pose_timeout_s = float(p('pose_timeout_s', POSE_TIMEOUT_THRESHOLD).value)
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
        if age > self.pose_timeout_s:
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

    def _control_phase_cb(self, msg):
        """Follow the reference generator: MPC while the OCP owns the flight, velocity
        once the dissipative network has the fleet.

        The switch happens in steady airborne hover (dissipative_node only hands over
        after the lift tops out AND settles), which is the one moment the two
        controllers agree -- both are holding the same reference with the same thrust
        feedforward. The integrator starts from zero rather than inheriting anything."""
        want = (str(msg.data).strip().lower() == 'network')
        if want == self._velocity_active:
            return
        self._velocity_active = want
        if want:
            self.velocity_loop.reset()
        self.get_logger().warn(
            f"[Drone {self.drone_id}] control phase -> "
            f"{'VELOCITY LOOP' if want else 'MPC'} (/fleet/control_phase="
            f"{msg.data!r})")

    def _publish_channels(self, u, u_rate):
        """Send [roll, pitch, throttle, yaw] to the FC, or armed-idle before TAKEOFF.

        The single publish path, shared by the MPC and the velocity loop, so a mode
        change cannot also change the takeoff spool, the applied-throttle bookkeeping
        that measured_cable_accel depends on, or the last-good-command fallback."""
        if self.takeoff_requested:
            thr = float(u[2])
            # takeoff spool-up: for the first takeoff_spool_s, scale the throttle up
            # from 0 to the commanded value so thrust rises smoothly and the drones
            # ease off the platforms. Raised-cosine (not linear): the applied throttle
            # jumps from idle-0 to the full hover value in one cycle when the spool is
            # off/too short, and a linear ramp still kinks the acceleration at both
            # ends. The taut air-start feels every bit of that as a lurch-and-bounce.
            if self.takeoff_spool_s > 0.0:
                if self._takeoff_step is None:
                    self._takeoff_step = 0
                spool_cycles = max(1.0, self.takeoff_spool_s * FREQUENCY_HZ)
                if self._takeoff_step < spool_cycles:
                    ease = 0.5 * (1.0 - np.cos(
                        np.pi * self._takeoff_step / spool_cycles))
                    # ease from FLOOR*u up to u (not 0*u): keep enough thrust to hold
                    # the load taut from cycle 0, see TAKEOFF_SPOOL_FLOOR.
                    frac = TAKEOFF_SPOOL_FLOOR + (1.0 - TAKEOFF_SPOOL_FLOOR) * ease
                    thr *= frac
                    self._takeoff_step += 1
            # Record the throttle actually applied to the FC so the next IMU sample
            # can be decomposed into thrust + cable (measured_cable_accel).
            self.last_cmd_throttle = thr
            msg = ELRSCommand(
                armed=True,
                channel_0=round(float(u[0]), 3),
                channel_1=round(float(u[1]), 3),
                channel_2=round((thr * 2) - 1, 3),
                channel_3=round(float(u[3]), 3))
            self._applied_u = np.array(
                [float(u[0]), float(u[1]), float(thr), float(u[3])])
            self._applied_u_rate = np.asarray(u_rate, dtype=float).copy()
            self.get_logger().debug(
                f"[Drone {self.drone_id}] r:{u[0]:.3f} p:{u[1]:.3f} "
                f"t:{u[2]:.3f} y:{u[3]:.3f}")
        else:
            self._takeoff_step = None    # reset so the next takeoff spools again
            self.last_cmd_throttle = None
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                              channel_2=-1.0, channel_3=0.0)
            # channel_2 = -1.0 maps to throttle 0, so every channel is at zero
            self._applied_u = np.zeros(4)
            self._applied_u_rate = np.zeros(4)
            # THROTTLED: this branch runs every cycle of the 50 Hz loop for as long as
            # the fleet sits armed waiting for TAKEOFF. Logging it unthrottled put 50
            # lines/s PER DRONE through the launch stdout pipe (200/s with four
            # drones); when that pipe backs up the write blocks the timer, which stalls
            # the single-threaded executor, so the node can't service the
            # /drone_N/command subscription that TAKEOFF arrives on.
            self.get_logger().info(
                f"[Drone {self.drone_id}] Armed - waiting for TAKEOFF command.",
                throttle_duration_sec=2.0)

        self.cb.cmd_publisher_.publish(msg)
        # Remember this good command so a later failed solve can hold it.
        if self.takeoff_requested:
            self._last_good_msg = msg
        return msg

    def _velocity_step(self, ref_acc):
        """Stage V: produce the channels from the velocity loop instead of the MPC.

        Uses only node 0 of the reference -- that is the point. The paper's controller
        emits a velocity command from the current virtual-node state; the 2 s horizon
        exists for the MPC's benefit, not the architecture's."""
        p = np.asarray(self.current_pose[0:3], float)
        q = np.asarray(self.current_pose[3:7], float)
        v = np.asarray(self.current_pose[7:10], float)
        p_ref = np.asarray(self.planner_ref_pos[0], float)
        v_ref = (np.asarray(self.planner_ref_vel[0], float)
                 if self.planner_ref_vel is not None else np.zeros(3))
        a_ff = (np.asarray(ref_acc[0], float) if ref_acc is not None
                else np.array([0.0, 0.0, 9.81]))
        # Zero the integrator on the rising edge of TAKEOFF. Detected here rather than
        # in the callback manager, which the proven MPC path shares -- this keeps Stage
        # V out of it entirely.
        if self.takeoff_requested and not self._prev_takeoff:
            self.velocity_loop.reset()
        self._prev_takeoff = bool(self.takeoff_requested)
        # Integrate only while actually flying the reference. On the stand the drone
        # cannot move toward it and the integrator would wind up against the platform,
        # then dump that trim in as a lurch the moment thrust is applied.
        integrate = bool(self.takeoff_requested and not self.payload_resting)
        u = self.velocity_loop.step(
            p, v, q, p_ref, v_ref, a_ff, 1.0 / FREQUENCY_HZ,
            self._effective_kT(), heading=self._heading_datum,
            integrate=integrate)
        self._current_ref_pos = p_ref
        self._applied_cable0 = 0.0      # no cable model in this path
        self._publish_channels(u, np.zeros(4))

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

    def _battery_derate(self):
        """Multiplier on kT for pack sag. 1.0 when the derate is off or unusable.

        Linear in measured pack voltage between kt_batt_v_full and kt_batt_v_empty.
        A missing or implausible reading returns 1.0 rather than guessing: the
        tracker then flies the configured thrust_ratio, which is the conservative
        direction, because assuming LESS thrust than the airframe has makes it
        under-throttle, not over-throttle.
        """
        if self.kt_batt_sag_frac <= 0.0:
            return 1.0
        v = self.battery_voltage
        if v is None or not math.isfinite(float(v)) or float(v) <= 0.0:
            return 1.0
        depletion = ((self.kt_batt_v_full - float(v))
                     / (self.kt_batt_v_full - self.kt_batt_v_empty))
        return 1.0 - self.kt_batt_sag_frac * float(np.clip(depletion, 0.0, 1.0))

    def _effective_kT(self):
        """The kT the MPC flies on this cycle.

        Fixed by configuration, with exactly two modifiers, both explicit:
          * pre-airborne it is takeoff_thrust_ratio (see AIRBORNE_MARGIN);
          * it is scaled by the battery derate, which is OFF unless configured.

        There is deliberately no estimator and no throttle-dependent schedule here.
        Both existed and both were removed (2026-08-05): a kT that moves underneath
        the tracker is a moving thrust feedforward on a controller with NO
        integrator, and neither mechanism could be reasoned about from the one
        number the launch file set -- `thrust_ratio:=20` provably changed nothing,
        because the schedule overrode it the moment the drone left the ground.

        KNOWN CONSEQUENCE IN SIM (measured 2026-08-05, SIL R0015-R0023). Gazebo's
        motor model is quadratic, a = c*u^2 with c = 88.6, so a fixed kT can match
        either the equilibrium or the loop gain but not both: the secant c*u_hover =
        32.9 puts hover in the right place, while the gain the loop actually sees is
        the tangent 2*c*u_hover = 65.7. In the attach weld transient that factor of
        two capsizes the payload past the 60 deg tilt fault within 1.1 s -- at kT
        30.0, 32.9 and 35.0 alike -- and the fleet disarms. This is a SIM/plant
        mismatch, not a hardware one: the real airframe measures a linear kT ~ 24
        with the flight controller in the loop. Fixing it belongs in the simulator's
        command -> motor mapping, not here.
        """
        base = (self.thrust_ratio if self._kt_airborne()
                else self.takeoff_thrust_ratio)
        return base * self._battery_derate()

    def _kt_airborne(self):
        """True once the drone is clearly off its stand -- the switch from
        takeoff_thrust_ratio to thrust_ratio. On a taut air-start the stands carry
        the weight, so the takeoff value has to stay in force until the drone has
        actually broken free (see AIRBORNE_MARGIN)."""
        if self.current_pose is None or self._kt_spawn_z is None:
            return False
        return float(self.current_pose[2]) >= self._kt_spawn_z + AIRBORNE_MARGIN

    def report_thrust_ratio(self):
        """~1 Hz one-liner so the flown kT can be eyeballed against the throttle."""
        if self.kt_print_period_s <= 0.0:
            return
        now = time.monotonic()
        if (self._kt_last_print_time is not None
                and now - self._kt_last_print_time < self.kt_print_period_s):
            return
        self._kt_last_print_time = now
        thr = (float(self.last_cmd_throttle)
               if self.last_cmd_throttle is not None else float('nan'))
        v = self.battery_voltage
        batt = ('   -- ' if v is None else f'{float(v):5.2f}V')
        self.get_logger().info(
            f"[kT d{self.drone_id}] flying {float(self.est_params[0]):6.2f} "
            f"[{'airborne' if self._kt_airborne() else ' takeoff'}] | "
            f"fixed={self.thrust_ratio:5.2f} takeoff={self.takeoff_thrust_ratio:5.2f}"
            f" | batt {batt} x{self._battery_derate():.3f} | thr={thr:.3f}")

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
        if self.armed and elapsed > self.pose_timeout_s:
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

            # Latch the resting height for the kT airborne gate. Keep re-latching
            # while the fleet sits armed waiting for TAKEOFF, so this ends up at the
            # true stand height whenever takeoff actually fires.
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

            # Assumed thrust gain for this cycle: the configured thrust_ratio, or
            # takeoff_thrust_ratio while still on the stand, times the battery
            # derate. Constant for the whole flight unless the derate is enabled.
            self.est_params[0] = self._effective_kT()

            # ── Set MPC reference (external planner) ──────────────────────
            # Track the streamed receding-horizon reference. There is no fixed
            # trajectory length / landing phase here: the fleet manager arms and
            # lands the fleet and the planner drives the timeline.
            if self.planner_ref_pos is None:
                # On the ground, armed-idle (throttle 0) is right: the fleet sits waiting
                # for TAKEOFF. In the AIR it is a crash -- channel_2 = -1.0 cuts the
                # motors, and the mixer (throttle +/- rate offsets, clipped at 0) then has
                # no authority to hold attitude either. Hold the last solved command
                # instead and let ref_stale (1 s) decide; the attach handover reached this
                # branch for ~0.2 s and tumbled the newcomer out of the sky every run.
                if self.takeoff_requested and self._last_good_msg is not None:
                    self.cb.cmd_publisher_.publish(self._last_good_msg)
                    self.get_logger().warn(
                        f"[Drone {self.drone_id}] airborne with no reference - "
                        f"holding last command.", throttle_duration_sec=1.0)
                else:
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
            # Stage V: the velocity loop replaces the MPC entirely from here. It needs
            # only node 0 and the thrust feedforward, so it branches before the cable
            # model, the solver reference and the solve.
            if self._velocity_active:
                self._velocity_step(ref_acc)
                return
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
            set_planner_reference(
                self.ocp, self.planner_ref_pos, self.planner_ref_vel,
                ref_acc, self.N, self.est_params,
                ref_cable=ref_cable, heading=self._heading_datum,
                terminal_vel_ref=self.terminal_vel_ref)
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

            self.report_thrust_ratio()

            # ── Publish command ───────────────────────────────────────────
            self._publish_channels(u, u_rate)

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
            log_row = [
                self.step_counter, sim_time_sec,
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]),
                float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]),
                float(self.current_pose[5]), float(self.current_pose[6]),
                float(ref[0]), float(ref[1]), float(ref[2]),
                float(self.est_params[0]),
                float(self.battery_voltage
                      if self.battery_voltage is not None else np.nan),
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