# import os
# os.chdir('/home/wesley/multi_drone_control/c_generated_code')
import os
import fcntl

# own acados dir so this solver doesn't clash with controller_mpc_multi's
ACADOS_DIR = '/home/wesley/multi_drone_control/c_generated_code_quad_load'
os.makedirs(ACADOS_DIR, exist_ok=True)
os.chdir(ACADOS_DIR)

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
from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, set_planner_reference


# recompile the solver if acados.py/dynamics.py changed since the last build
_PKG_DIR = os.path.dirname(_acados_mod.__file__)
_SOLVER_SRC = [os.path.join(_PKG_DIR, 'acados.py'),
               os.path.join(_PKG_DIR, 'dynamics.py')]
# the .so lands in a nested c_generated_code dir, json goes in ACADOS_DIR
_SOLVER_SO = os.path.join(ACADOS_DIR, 'c_generated_code',
                          'libacados_ocp_solver_quad_load_dynamics.so')
_SOLVER_JSON = os.path.join(ACADOS_DIR, 'quad_load_dynamics_ocp.json')


def _solver_is_fresh():
    """solver exists and is newer than its source -> load it, don't recompile"""
    if not (os.path.exists(_SOLVER_SO) and os.path.exists(_SOLVER_JSON)):
        return False
    so_mtime = os.path.getmtime(_SOLVER_SO)
    return all(os.path.exists(s) and os.path.getmtime(s) <= so_mtime
               for s in _SOLVER_SRC)
from .trajectories import circle_trajectory, hover_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager_multi import CallbackManagerMulti
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from std_msgs.msg import Int32, Float64MultiArray
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

# Ceiling for the airborne kT schedule (a(u)=c*u^2 -> secant c*throttle). Hover
# throttle ~0.24 gives ~49, so 55 caps momentary highs without ever exceeding the
# real plant. The floor is thrust_ratio, so takeoff can never be starved.
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
        super().__init__('controller', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        # ── Drone identity ────────────────────────────────────────────────
        self.declare_parameter("drone_id", 0)
        self.drone_id = self.get_parameter("drone_id").value
        self.offset_x, self.offset_y = DRONE_OFFSETS.get(self.drone_id, (0.0, 0.0))

        # Reference source: 'internal' = own circle (default, unchanged);
        # 'planner' = track /drone_{id}/reference_trajectory from controller_load_mpc.
        self.declare_parameter("reference_source", "internal")
        self.reference_source = self.get_parameter("reference_source").value

        # 'circle' = fly a wide circle, 'hover' = go straight up and hold (the
        # cable test - climb till the cable goes taut and the load lifts with it)
        self.declare_parameter("trajectory", "circle")
        self.trajectory_type = self.get_parameter("trajectory").value

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

        # get_clock() is sim time (use_sim_time=True). keep a separate wall clock
        # for the pose-timeout watchdog since that's about real comms latency.
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
        init_pose = self.current_pose

        # each drone offsets the shared trajectory so they fly in formation
        self.traj, trajectory_name = self._build_offset_trajectory(DT, init_pose)

        # ── Visualizer ────────────────────────────────────────────────────
        # prefix the viz topics per drone (/drone_N/mpc_plan etc) so RViz can
        # show each drone's plan on its own display instead of N drones fighting
        # over one global /mpc_plan.
        self.trajectory_visualizer = TrajectoryVisualizer(
            self, frame_id="map", prefix=f"drone_{self.drone_id}")
        self.trajectory_visualizer.publish_all_visualizations(
            self.traj, pose_subsample=15, show_velocity=False,
            velocity_scale=0.3, color_by_time=True)

        # ── State ─────────────────────────────────────────────────────────
        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        # fleet manager broadcasts the master step index, just follow it
        self.fleet_step_sub = self.create_subscription(
            Int32,
            '/fleet/step',
            self._fleet_step_callback,
            1)  # depth=1: always use the latest, never queue stale steps

        # ── Planner reference subscription (planner mode only) ────────────
        if self.reference_source == "planner":
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

        # ── MPC ───────────────────────────────────────────────────────────
        self.N = 20
        self.skip_steps = 3
        self.first_solve = True
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
        ]
        # drone 0 also logs the payload actual + desired so plot_run.py can
        # overlay the load track alongside the drones.
        if self._log_payload:
            log_headers += ['payload_x', 'payload_y', 'payload_z',
                            'payload_ref_x', 'payload_ref_y', 'payload_ref_z']
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.observed_state_history = []
        self.control_history = []
        self.estimated_state_history = []

        # ── Control timer ─────────────────────────────────────────────────
        self.timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info(
            f"[Drone {self.drone_id}] Controller ready. "
            f"Offset=({self.offset_x}, {self.offset_y}). "
            f"Traj length={self.steps} steps.")

    # ─────────────────────────────────────────────────────────────────────
    # Trajectory helpers
    # ─────────────────────────────────────────────────────────────────────

    def _build_offset_trajectory(self, dt, init_pose):
        """Generate this drone's internal trajectory from its spawn pose.

        'hover' lifts straight up over the spawn point and holds (cable-carry
        test); 'circle' flies a wide circle (free-flight default).
        """
        if self.trajectory_type == "hover":
            traj, name = hover_trajectory(dt, init_pose=init_pose)
        else:
            traj, name = circle_trajectory(dt, init_pose=init_pose,
                                           center_offset_x=self.offset_x,
                                           center_offset_y=self.offset_y)
        return traj, f"{name}_drone{self.drone_id}"

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

            # Schedule the assumed thrust gain to the operating point (quadratic
            # plant). No-op during takeoff/resting and when disabled (c=0).
            self.est_params[0] = self._scheduled_kT()

            # ── Set MPC reference ─────────────────────────────────────────
            if self.reference_source == "planner":
                # External planner: track the streamed receding-horizon
                # reference (no fixed trajectory length / landing phase).
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
                set_planner_reference(
                    self.ocp, self.planner_ref_pos, self.planner_ref_vel,
                    ref_acc, self.N, self.est_params,
                    ref_cable=ref_cable)
                # desired reference position now (node 0) for the log / plot
                self._current_ref_pos = np.asarray(self.planner_ref_pos[0], float)
            else:
                # Internal circle: land + shutdown at end of trajectory.
                if self.step_counter + self.N * self.skip_steps > self.steps:
                    self.get_logger().info(
                        f"[Drone {self.drone_id}] Trajectory complete - disarming.")
                    msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                                      channel_2=-1.0, channel_3=0.0)
                    self.cb.disarm(msg)
                    self.cb.request_shutdown()
                    return
                set_trajectory_reference_aligned(
                    self.ocp, self.traj, self.N,
                    self.step_counter, self.skip_steps, self.est_params)
                # desired reference position now (aligned node 0) for the log / plot
                idx = min(self.step_counter, self.traj.shape[1] - 1)
                self._current_ref_pos = np.asarray(self.traj[0:3, idx], float)

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
            if self.reference_source == "planner" and self.takeoff_requested:
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
                    # self.get_logger().info(
                    #     f"[diag d{self.drone_id}] z={zc:.2f} ez={zr - zc:+.2f} "
                    #     f"exy={exy:.2f} rdrift={rdrift:+.2f} thr={float(u[2]):.3f} "
                    #     f"roll={float(u[0]):+.2f} pitch={float(u[1]):+.2f} "
                    #     f"|aT|={aT:.2f} |aC|={aC:.2f} |aCm|={aCm:.2f} dC={dC:.2f}")

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
                # sample can be decomposed into thrust + cable (measured_cable_accel).
                self.last_cmd_throttle = thr
                msg = ELRSCommand(
                    armed=True,
                    channel_0=round(u[0], 3),
                    channel_1=round(u[1], 3),
                    channel_2=round((thr * 2) - 1, 3),
                    channel_3=round(u[3], 3))
                self._applied_u = np.array(
                    [float(u[0]), float(u[1]), float(thr), float(u[3])])
                self.get_logger().debug(
                    f"[Drone {self.drone_id}] r:{u[0]:.3f} p:{u[1]:.3f} "
                    f"t:{u[2]:.3f} y:{u[3]:.3f}")
            else:
                self._takeoff_step = None    # reset so the next takeoff spools again
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                                  channel_2=-1.0, channel_3=0.0)
                # channel_2 = -1.0 maps to throttle 0, so every channel is at zero
                self._applied_u = np.zeros(4)
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
            log_row = [
                self.step_counter, sim_time_sec,
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]),
                float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]),
                float(self.current_pose[5]), float(self.current_pose[6]),
                float(ref[0]), float(ref[1]), float(ref[2]),
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