"""
drone_controller.py — Per-drone position PD controller with cable compensation
------------------------------------------------------------------------------
Implements the onboard trajectory tracking controller from the TU Delft paper
(Eq. 15), simplified for hover:

    F_des / m = Kp*(p_ref - p) + Kv*(v_ref - v) + v_ref_dot + f_ext/m

where f_ext (cable tension) is estimated as a constant vertical force
equal to the drone's share of the payload weight.

From F_des we compute:
  1. Thrust magnitude T = ||F_des||
  2. Desired body z-axis z_des = F_des / T  (thrust direction)
  3. Desired roll/pitch via geometric decomposition
  4. Attitude PD  →  angular rate commands
  5. Map rates to Betaflight ELRS channel_0/1/3 via linear approximation
     (valid for small corrections; channel ≈ rate_deg / RATE_CENTER)

Dynamics parameters match dynamics.py / est_params:
  THRUST_RATIO = 38.0   (thrust_ratio in MPC model)
  RATE_CENTER  = 70.0   (Betaflight centre rate, deg/s per unit)

State machine: same ARM / TAKEOFF / DISARM flow as other controllers,
driven by /drone_{id}/arming_service and /drone_{id}/command topics via
CallbackManagerMulti.

Subscribes:
  /drone_{id}/motion_capture_state
  /drone_{id}/setpoint            Float64MultiArray [x,y,z,vx,vy,vz]

Publishes:
  /drone_{id}/ELRSCommand
"""

import sys
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Float64MultiArray
from interfaces.msg import ELRSCommand, MotionCaptureState
from utility_objects.callback_manager_multi import CallbackManagerMulti
from tf_transformations import euler_from_quaternion

# ── Tunable parameters ────────────────────────────────────────────────────────

DRONE_MASS = 0.6       # kg
PAYLOAD_MASS = 0.4     # kg  (matches world_quad_payload.sdf)
N_DRONES = 4
G = 9.81               # m/s²
THRUST_RATIO = 38.0    # from dynamics.py est_params[0]
RATE_CENTER = 70.0     # Betaflight centre rate, deg/s per unit channel

# Position PD gains
KP = np.diag([3.0, 3.0, 5.0])
KV = np.diag([2.0, 2.0, 3.5])

# Attitude P gain (rad/s per rad error)
KA = 8.0

FREQUENCY_HZ = 30.0


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_pose_update_time = self._wall_clock.now()  # updated by CallbackManagerMulti
        self.current_pose = None
        self.setpoint = None
        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False
        self.on_close_called = False

        # CallbackManagerMulti handles arming service, command topic, pose sub
        self.cb = CallbackManagerMulti(self, drone_id=self.drone_id)

        # Setpoint subscription (from planner)
        self.create_subscription(
            Float64MultiArray,
            f'/drone_{self.drone_id}/setpoint',
            self._setpoint_cb, 1)

        # Control loop
        self.create_timer(1.0 / FREQUENCY_HZ, self._control_loop)

        self.get_logger().info(f'[Drone {self.drone_id}] Controller ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _setpoint_cb(self, msg: Float64MultiArray):
        self.setpoint = np.array(msg.data[:6])

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if not self.armed or self.current_pose is None or self.setpoint is None:
            self._publish_disarmed()
            return

        # Pose timeout watchdog
        elapsed = (self._wall_clock.now() - self.last_pose_update_time).nanoseconds * 1e-9
        if elapsed > 0.25:
            self.get_logger().error(
                f'[Drone {self.drone_id}] Pose timeout — disarming.')
            self._publish_disarmed()
            return

        p = self.current_pose[:3]
        q = self.current_pose[3:7]    # [qw, qx, qy, qz]
        v = self.current_pose[7:10]

        p_des = self.setpoint[:3]
        v_des = self.setpoint[3:6]

        if not self.takeoff_requested:
            self._publish_armed_idle()
            return

        # ── Position PD ───────────────────────────────────────────────────────
        pos_err = p_des - p
        vel_err = v_des - v
        a_des = KP @ pos_err + KV @ vel_err

        # ── Cable tension compensation (simplified: constant vertical force) ──
        # Each drone must support its share of the payload weight.
        f_cable_z = PAYLOAD_MASS * G / N_DRONES   # downward on drone

        # ── Desired force vector (world frame) ────────────────────────────────
        # F = m*(a_des + g_vec) + f_cable (both gravity and cable pull down)
        F_des = DRONE_MASS * (a_des + np.array([0.0, 0.0, G])) + np.array([0.0, 0.0, f_cable_z])
        F_des[2] = max(F_des[2], 0.5 * DRONE_MASS * G)   # safety floor

        T_mag = np.linalg.norm(F_des)
        z_des = F_des / T_mag       # desired body z-axis (thrust direction)

        # ── Throttle ──────────────────────────────────────────────────────────
        # From v_dynamics: a_z = thrust_ratio * u2 → u2 = T_mag / (m * thrust_ratio)
        u2 = T_mag / (DRONE_MASS * THRUST_RATIO)
        u2 = float(np.clip(u2, 0.05, 0.95))
        channel_2 = u2 * 2.0 - 1.0

        # ── Attitude: desired roll/pitch from thrust direction ─────────────────
        # scipy uses [qx, qy, qz, qw]; our state has [qw, qx, qy, qz]
        q_xyzw = [float(q[1]), float(q[2]), float(q[3]), float(q[0])]
        roll, pitch, yaw = euler_from_quaternion(q_xyzw)

        # Desired tilt angles (small-angle safe using atan2)
        pitch_des = float(np.arctan2(z_des[0], z_des[2]))
        roll_des = float(np.arctan2(-z_des[1], np.sqrt(z_des[0]**2 + z_des[2]**2)))

        # ── Attitude P → angular rate commands ────────────────────────────────
        roll_rate_rad = KA * (roll_des - roll)
        pitch_rate_rad = KA * (pitch_des - pitch)
        yaw_rate_rad = 0.0

        # Map to Betaflight channels (linear approximation for small rates)
        roll_rate_deg = float(np.degrees(roll_rate_rad))
        pitch_rate_deg = float(np.degrees(pitch_rate_rad))
        yaw_rate_deg = float(np.degrees(yaw_rate_rad))

        channel_0 = float(np.clip(roll_rate_deg / RATE_CENTER, -1.0, 1.0))
        channel_1 = float(np.clip(pitch_rate_deg / RATE_CENTER, -1.0, 1.0))
        channel_3 = float(np.clip(yaw_rate_deg / RATE_CENTER, -1.0, 1.0))

        self.cb.cmd_publisher_.publish(ELRSCommand(
            armed=True,
            channel_0=round(channel_0, 3),
            channel_1=round(channel_1, 3),
            channel_2=round(channel_2, 3),
            channel_3=round(channel_3, 3),
        ))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_disarmed(self):
        self.cb.cmd_publisher_.publish(
            ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0,
                        channel_2=-1.0, channel_3=0.0))

    def _publish_armed_idle(self):
        """Armed but waiting for TAKEOFF: spin props at minimum."""
        self.cb.cmd_publisher_.publish(
            ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0,
                        channel_2=-1.0, channel_3=0.0))

    def signal_handler(self, sig, frame):
        self.on_close()
        sys.exit(0)

    def on_close(self):
        if self.on_close_called:
            return
        self.on_close_called = True
        self._publish_disarmed()


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    signal.signal(signal.SIGINT, node.signal_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[Drone {node.drone_id}] Exception: {e}')
    finally:
        node.on_close()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
