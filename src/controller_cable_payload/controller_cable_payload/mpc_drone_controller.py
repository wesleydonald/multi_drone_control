"""
mpc_drone_controller.py — Per-drone acados MPC controller for the cable world
-----------------------------------------------------------------------------
A near-copy of controller_mpc_multi/controller_mpc.py, adapted for the
rigid-cable payload world (run_cables_world.sh).  Differences:

  * Reuses the SAME compiled acados solver and helpers from the
    controller_mpc_multi package (no duplicated OCP definition).
  * Self-owned step counter — there is no fleet manager broadcasting
    /fleet/step here, so each drone advances its own trajectory clock once
    TAKEOFF is requested.  The step is clamped near the end so the drone
    holds a steady hover instead of running off the trajectory.
  * A gentle "lift slightly" trajectory (LIFT_HEIGHT, default 0.3 m) instead
    of the 1.2 m takeoff, so the drones just rise a little and hover.

ARM / TAKEOFF / DISARM are handled exactly as in the MPC fleet controller,
via CallbackManagerMulti (/drone_{id}/arming_service + /drone_{id}/command).
The central planner in rigid_cables_launch.py relays /fleet/command to those
per-drone interfaces, so the existing Fleet Commands terminal still works.

Subscribes:  /drone_{id}/motion_capture_state   (via CallbackManagerMulti)
Publishes:   /drone_{id}/ELRSCommand            (via CallbackManagerMulti)
"""

import os
import fcntl

# The acados solver compiles into this shared directory (gitignored).  All four
# drone processes serialise compilation through a file lock, same as the MPC
# fleet controller.
ACADOS_DIR = '/home/wesley/multi_drone_control/c_generated_code'
os.chdir(ACADOS_DIR)

_lock_path = os.path.join(ACADOS_DIR, '.compile.lock')
_lock_file = open(_lock_path, 'w')
fcntl.flock(_lock_file, fcntl.LOCK_EX)

import sys
import copy
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

# Reuse the proven OCP + helpers from the MPC fleet package.
from controller_mpc_multi.acados import (
    generate_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_trajectory_reference_aligned,
)
from controller_mpc_multi.trajectories import takeoff_trajectory_with_goal
from utility_objects.callback_manager_multi import CallbackManagerMulti
from interfaces.msg import ELRSCommand


POSE_TIMEOUT_THRESHOLD = 0.25      # seconds
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ

# Same dynamics/Betaflight parameters the MPC fleet controller uses:
# [thrust_ratio, drag, arm_length, rate_d, rate_f, rate_g]
EST_PARAMS = np.array([38.0, 0.0, 0.12, 70.0, 670.0, 0.5])

# Optional XY goal per drone (same idea as controller_mpc.py's DRONE_GOALS).
# If a drone_id is absent, the controller defaults to a pure vertical takeoff
# over the drone's own start position (best for lifting the payload straight up).
DRONE_GOALS = {
    # Empty → each drone takes off straight up over its own start (x, y).
    # This is the right default for lifting the payload vertically.  Add
    # entries (drone_id: (x, y)) only if you want lateral moves, and make
    # sure they match the world's drone start layout.
}


class MpcDroneController(Node):
    def __init__(self):
        super().__init__('mpc_drone_controller', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        self.declare_parameter("drone_id", 0)
        self.drone_id = self.get_parameter("drone_id").value

        # Wall clock for the pose-timeout watchdog (real comms latency).
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_pose_update_time = self._wall_clock.now()

        # ── State flags (read/written by CallbackManagerMulti) ─────────────
        self.current_pose = None
        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False
        self.on_close_called = False

        self.cb = CallbackManagerMulti(self, drone_id=self.drone_id)

        # ── Wait for first mocap pose ──────────────────────────────────────
        self.get_logger().info(f"[Drone {self.drone_id}] Waiting for initial pose...")
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"[Drone {self.drone_id}] Pose received.")
        init_pose = self.current_pose

        self.est_params = EST_PARAMS
        self.N = 20
        self.skip_steps = 3

        # Build an initial trajectory from the spawn pose.  It is rebuilt from
        # the *current* pose the moment TAKEOFF is pressed (the drones settle to
        # the ground at spawn in the soft-cable world), so the lift always
        # starts from where the drone actually is.
        self._traj_anchored = False
        self._build_traj(init_pose)

        # ── MPC ────────────────────────────────────────────────────────────
        self.first_solve = True
        self.ocp, self.sim_integrator = generate_ocp_controller()
        fcntl.flock(_lock_file, fcntl.LOCK_UN)

        self.control_history = []

        self.timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info(
            f"[Drone {self.drone_id}] MPC controller ready. Send ARM -> TAKEOFF.")

    # ─────────────────────────────────────────────────────────────────────
    def _build_traj(self, init_pose):
        """Build the takeoff trajectory anchored at init_pose (vertical takeoff
        over the drone's own x,y unless DRONE_GOALS overrides)."""
        goal_x, goal_y = DRONE_GOALS.get(
            self.drone_id, (float(init_pose[0]), float(init_pose[1])))
        self.traj, _ = takeoff_trajectory_with_goal(
            DT, init_pose=init_pose, target_pose=np.array([goal_x, goal_y]))
        self.steps = self.traj.shape[1] - 1
        # Hold the final hover reference once we reach the end of the ramp.
        self.max_step = self.steps - self.N * self.skip_steps - 1
        self.step_counter = 0
        self.first_solve = True

    # ─────────────────────────────────────────────────────────────────────
    def control_loop(self):
        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        # Pose-timeout watchdog (wall time).
        elapsed = (self._wall_clock.now() - self.last_pose_update_time).nanoseconds * 1e-9
        if self.armed and elapsed > POSE_TIMEOUT_THRESHOLD:
            self.get_logger().error(
                f"[Drone {self.drone_id}] Pose timeout ({elapsed:.2f}s) — disarming.")
            self.cb.disarm(self._disarmed_msg())
            return

        if not (self.armed and self.current_pose is not None):
            self.cb.cmd_publisher_.publish(self._disarmed_msg())
            return

        # Re-anchor the trajectory to the current pose at the takeoff transition
        # so the lift starts from where the drone actually settled.
        if self.takeoff_requested and not self._traj_anchored:
            self._build_traj(self.current_pose)
            self._traj_anchored = True
            self.get_logger().info(
                f"[Drone {self.drone_id}] Trajectory anchored at "
                f"z={self.current_pose[2]:.2f}; lifting.")

        # ── Set MPC reference ──────────────────────────────────────────────
        set_trajectory_reference_aligned(
            self.ocp, self.traj, self.N,
            self.step_counter, self.skip_steps, self.est_params)

        estimated_state = copy.deepcopy(self.current_pose[:13])
        if len(self.control_history) == 0:
            est_with_u = np.concatenate((estimated_state, np.zeros(4)))
        else:
            est_with_u = np.concatenate(
                (estimated_state, np.array(self.control_history[-1][0:4])))

        relaxation = 0.025
        self.ocp.set(0, "lbx", est_with_u * (1 - relaxation))
        self.ocp.set(0, "ubx", est_with_u * (1 + relaxation))

        # ── Warm start ─────────────────────────────────────────────────────
        if self.first_solve:
            set_initial_guess(self.ocp, self.N)
            self.first_solve = False
        else:
            warm_start_from_previous_solution(self.ocp, self.N)

        # ── Solve ──────────────────────────────────────────────────────────
        status = self.ocp.solve()
        if status != 0:
            self.get_logger().error(
                f"[Drone {self.drone_id}] acados status {status} — disarming.")
            self.cb.disarm(self._disarmed_msg())
            return

        x = self.ocp.get(1, "x")
        u = x[-4:]
        u_rate = self.ocp.get(0, "u")

        # ── Publish command ────────────────────────────────────────────────
        if self.takeoff_requested:
            msg = ELRSCommand(
                armed=True,
                channel_0=round(float(u[0]), 3),
                channel_1=round(float(u[1]), 3),
                channel_2=round(float((u[2] * 2) - 1), 3),
                channel_3=round(float(u[3]), 3))
            # Advance our own trajectory clock; hold at the hover tail.
            if self.step_counter < self.max_step:
                self.step_counter += 1
        else:
            # Armed but waiting for TAKEOFF — hold props at idle.
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                              channel_2=-1.0, channel_3=0.0)

        self.cb.cmd_publisher_.publish(msg)
        self.control_history.append(np.concatenate((u, u_rate)).tolist())

        # ── Diagnostics: ref vs actual + commanded throttle ────────────────
        # Throttle channel maps to motor speed in the betaflight node as
        # (ch2 + 1)/2 * 4631.  Hover for a 0.6 kg drone is ~1018.
        p = self.current_pose
        ref_z = float(self.traj[2, self.step_counter])
        motor_eq = (float(msg.channel_2) + 1.0) / 2.0 * 4631.0
        self.get_logger().info(
            f"[D{self.drone_id}] step={self.step_counter} "
            f"z={p[2]:.2f}(ref {ref_z:.2f}) vz={p[9]:.2f} "
            f"roll={np.degrees(self._roll(p)):.1f} pitch={np.degrees(self._pitch(p)):.1f} "
            f"ch2={msg.channel_2:.2f} -> motor~{motor_eq:.0f} (hover~1018)")

    # ─────────────────────────────────────────────────────────────────────
    @staticmethod
    def _roll(p):
        """Roll (rad) from pose quaternion [qw,qx,qy,qz] at p[3:7]."""
        qw, qx, qy, qz = p[3], p[4], p[5], p[6]
        return np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))

    @staticmethod
    def _pitch(p):
        """Pitch (rad) from pose quaternion [qw,qx,qy,qz] at p[3:7]."""
        qw, qx, qy, qz = p[3], p[4], p[5], p[6]
        return np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0))

    @staticmethod
    def _disarmed_msg():
        return ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                           channel_2=-1.0, channel_3=0.0)

    def signal_handler(self, sig, frame):
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if self.on_close_called:
            return
        self.on_close_called = True
        self.cb.cmd_publisher_.publish(self._disarmed_msg())


def main(args=None):
    rclpy.init(args=args)
    controller = MpcDroneController()
    signal.signal(signal.SIGINT, controller.signal_handler)
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
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
