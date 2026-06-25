# import os
# os.chdir('/home/wesley/multi_drone_control/c_generated_code')
import os
import fcntl

# Separate export dir from controller_mpc_multi so the two solvers never clobber
# each other (this package's model is quad_load_dynamics, with the cable term).
ACADOS_DIR = '/home/wesley/multi_drone_control/c_generated_code_quad_load'
os.makedirs(ACADOS_DIR, exist_ok=True)
os.chdir(ACADOS_DIR)

# Serialize compilation across the 4 drone processes
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


# Solver-source files; if any is newer than the compiled .so, the solver is
# stale and must be regenerated/recompiled (otherwise we load the existing lib).
_PKG_DIR = os.path.dirname(_acados_mod.__file__)
_SOLVER_SRC = [os.path.join(_PKG_DIR, 'acados.py'),
               os.path.join(_PKG_DIR, 'dynamics.py')]
# cwd is ACADOS_DIR and acados' default code_export_directory is the relative
# 'c_generated_code', so the compiled solver actually lands in a NESTED dir
# (ACADOS_DIR/c_generated_code/...). The json is written to cwd (ACADOS_DIR).
_SOLVER_SO = os.path.join(ACADOS_DIR, 'c_generated_code',
                          'libacados_ocp_solver_quad_load_dynamics.so')
_SOLVER_JSON = os.path.join(ACADOS_DIR, 'quad_load_dynamics_ocp.json')


def _solver_is_fresh():
    """True if the compiled solver exists and is newer than its source files,
    so it can be loaded without recompiling."""
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


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True
# Cable-aware tracker runs faster than the cable-blind one (was 30 Hz): the
# tautening cable is a stiff disturbance and a low-rate loop has little phase
# margin against it (the paper runs the tracker at >=100 Hz). Watch the "Solve
# time" log — if it approaches the control period (1000/FREQUENCY_HZ ms), back off.
FREQUENCY_HZ = 50.0
DT = 1.0 / FREQUENCY_HZ

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

        # Internal trajectory shape: 'circle' (default, fly a wide circle) or
        # 'hover' (lift straight up over the spawn point and hold, then land).
        # 'hover' is the cable-carry test: with slack cables the drones climb
        # straight up, the cable goes taut, and the load lifts 1:1 — no wide
        # lateral motion that would fight the tethers.
        self.declare_parameter("trajectory", "circle")
        self.trajectory_type = self.get_parameter("trajectory").value
        self.planner_ref_pos = None      # (N+1, 3) world-frame position nodes
        self.planner_ref_vel = None      # (N+1, 3) world-frame velocity nodes
        self.planner_ref_acc = None      # (N+1, 3) required specific thrust accel
        self.planner_ref_cable = None    # (N+1, 3) cable tension accel t*s/m (world)

        # ── Sim-time clock ────────────────────────────────────────────────
        # self.get_clock() will return sim time because use_sim_time=True.
        # We also keep a wall-clock reference for the pose-timeout watchdog,
        # because that check is about real comms latency, not sim latency.
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

        # ── Build offset trajectory ───────────────────────────────────────
        # The fleet manager publishes the master step; each drone applies its
        # own spatial offset so all four fly congruent circles in formation.
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

        # ── Fleet step subscription ───────────────────────────────────────
        # The fleet manager owns the master clock and broadcasts the current
        # step index.  We simply shadow it here.
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
        self.est_params = np.array([38.0, 0.0, 0.12, 70.0, 670.0, 0.5])

        # ── Logging ───────────────────────────────────────────────────────
        log_headers = [
            'step', 'sim_time', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
        ]
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
                f"[Drone {self.drone_id}] Pose timeout ({elapsed:.2f}s) — disarming.")
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
                    # No reference streamed yet — hold armed-idle on the ground.
                    self.cb.cmd_publisher_.publish(ELRSCommand(
                        armed=True, channel_0=0.0, channel_1=0.0,
                        channel_2=-1.0, channel_3=0.0))
                    return
                set_planner_reference(
                    self.ocp, self.planner_ref_pos, self.planner_ref_vel,
                    self.planner_ref_acc, self.N, self.est_params,
                    ref_cable=self.planner_ref_cable)
            else:
                # Internal circle: land + shutdown at end of trajectory.
                if self.step_counter + self.N * self.skip_steps > self.steps:
                    self.get_logger().info(
                        f"[Drone {self.drone_id}] Trajectory complete — disarming.")
                    msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                                      channel_2=-1.0, channel_3=0.0)
                    self.cb.disarm(msg)
                    self.cb.request_shutdown()
                    return
                set_trajectory_reference_aligned(
                    self.ocp, self.traj, self.N,
                    self.step_counter, self.skip_steps, self.est_params)

            estimated_state = copy.deepcopy(self.current_pose[:13])

            if len(self.control_history) == 0:
                self.get_logger().warn(
                    f"[Drone {self.drone_id}] Control history empty — using zero initial control.")
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
                self.get_logger().error(
                    f"[Drone {self.drone_id}] acados returned status {status}.")
                raise RuntimeError(f"acados returned status {status}.")

            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")

            # ── Diagnostic (planner mode) ─────────────────────────────────
            # ~2 Hz: shows whether the planner feedforward is actually engaging.
            #   ez   = z tracking error (zref - z); large/growing => losing altitude
            #   thr  = commanded throttle u_state (0.05..0.6); 0.6 => saturating
            #   |aT| = thrust-ff accel magnitude (should be >9.81 and rising w/ load)
            #   |aC| = cable accel magnitude applied to the model (0 => gate shut /
            #          cable not engaging => effectively cable-blind)
            if self.reference_source == "planner" and self.takeoff_requested:
                self._diag_ctr = getattr(self, "_diag_ctr", 0) + 1
                if self._diag_ctr % 15 == 0:
                    zc = float(self.current_pose[2])
                    zr = float(self.planner_ref_pos[0][2])
                    aT = float(np.linalg.norm(self.planner_ref_acc[0]))
                    aC = (float(np.linalg.norm(self.planner_ref_cable[0]))
                          if self.planner_ref_cable is not None else 0.0)
                    self.get_logger().info(
                        f"[diag d{self.drone_id}] z={zc:.2f} zref={zr:.2f} "
                        f"ez={zr - zc:+.2f} thr={float(u[2]):.3f} "
                        f"roll={float(u[0]):+.2f} pitch={float(u[1]):+.2f} "
                        f"|aT|={aT:.2f} |aC|={aC:.2f}")

            # ── Publish command ───────────────────────────────────────────
            if self.takeoff_requested:
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
                    f"[Drone {self.drone_id}] Armed — waiting for TAKEOFF command.")

            self.cb.cmd_publisher_.publish(msg)

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
            log_row = [
                self.step_counter, sim_time_sec,
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]),
                float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]),
                float(self.current_pose[5]), float(self.current_pose[6]),
            ]
            self.data_logger.append_row(log_row)

            self.control_history.append(np.concatenate((u, u_rate)).tolist())
            self.observed_state_history.append(self.current_pose[:13].tolist())
            self.estimated_state_history.append(estimated_state[:13].tolist())

        # ── Disarmed / no pose ─────────────────────────────────────────────
        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                              channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            # Do NOT reset step_counter here — fleet manager owns it

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