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
from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, set_planner_reference, update_ocp_parameters


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
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
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

        # only drone 0 logs the payload (actual from mocap, desired from planner)
        self.payload_pos = None
        self.payload_ref = None
        self._log_payload = (self.drone_id == 0)
        if self._log_payload:
            self.create_subscription(
                MotionCaptureState, '/payload/motion_capture_state',
                self._payload_state_cb, 5)
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
        self.est_params = np.array([24.0, 0.0, 0.12, 70.0, 670.0, 0.5])

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

    def _payload_ref_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self.payload_ref = np.array(msg.data[:3], dtype=float)

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

            if len(self.control_history) == 0:
                self.get_logger().warn(
                    f"[Drone {self.drone_id}] Control history empty - using zero initial control.")
                estimated_state_with_control = np.concatenate(
                    (estimated_state, np.array([0.0, 0.0, 0.0, 0.0])))
            else:
                estimated_state_with_control = np.concatenate(
                    (estimated_state, np.array(self.control_history[-1][0:4])))

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
                    self.get_logger().info(
                        f"[diag d{self.drone_id}] z={zc:.2f} ez={zr - zc:+.2f} "
                        f"exy={exy:.2f} rdrift={rdrift:+.2f} thr={float(u[2]):.3f} "
                        f"roll={float(u[0]):+.2f} pitch={float(u[1]):+.2f} "
                        f"|aT|={aT:.2f} |aC|={aC:.2f} |aCm|={aCm:.2f} dC={dC:.2f}")

            # ── Publish command ───────────────────────────────────────────
            if self.takeoff_requested:
                # Record the throttle actually applied to the FC so the next IMU
                # sample can be decomposed into thrust + cable (measured_cable_accel).
                self.last_cmd_throttle = float(u[2])
                msg = ELRSCommand(
                    armed=True,
                    channel_0=round(u[0], 3),
                    channel_1=round(u[1], 3),
                    channel_2=round((u[2] * 2) - 1, 3),
                    channel_3=round(u[3], 3))
                self.get_logger().debug(
                    f"[Drone {self.drone_id}] r:{u[0]:.3f} p:{u[1]:.3f} "
                    f"t:{u[2]:.3f} y:{u[3]:.3f}")
            else:
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                                  channel_2=-1.0, channel_3=0.0)
                self.get_logger().info(
                    f"[Drone {self.drone_id}] Armed - waiting for TAKEOFF command.")

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

            self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            self.trajectory_visualizer.publish_transform_frame(
                self.current_pose, f"drone_{self.drone_id}_mocap")
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