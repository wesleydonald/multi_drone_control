"""
load_state_node.py
------------------
Stage 1 of the cable-load controller plan: the LOAD-STATE PIPELINE.

Consumes the mocap that is already bridged from Gazebo
(`/payload/motion_capture_state` and each `/drone_{i}/motion_capture_state`) and
republishes a single, compact load-state message on `/payload/state`:

    load position (world)        p   = [px, py, pz]
    load linear velocity (world) v   = [vx, vy, vz]
    N cable directions           s_i = (attach_i - p_i) / ||attach_i - p_i||

where the i-th attach point is attach_i = p + R(q)·rho_i (load-frame offset rho_i
rotated into the world by the load orientation q). This matches the cable-direction
convention the planner uses in `_build_x_init` (planner_node.py), so downstream
closed-loop code can consume `/payload/state` without re-deriving geometry.

This node adds NO estimation — it is a straight passthrough of the ground-truth
sim mocap plus the geometric s_i. It exists so the later closed-loop (geometric)
planner mode has one authoritative load-feedback topic to read.

Wire format — Float64MultiArray on `/payload/state`:
    [n_drones,
     px, py, pz,
     vx, vy, vz,
     s0x, s0y, s0z,  s1x, s1y, s1z,  ...  (3 per drone) ]
Length = 1 + 3 + 3 + 3*n_drones.

Parameters (ROS): n_drones, attach_radius, attach_z (must match the planner /
world geometry — see planner_node.py module defaults).

Published once per payload mocap update (i.e. at mocap rate), as soon as every
drone position has been seen at least once.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from interfaces.msg import MotionCaptureState

# geometry defaults — keep in sync with controller_load_mpc/planner_node.py
N_DRONES      = 3
ATTACH_RADIUS = 0.08
ATTACH_Z      = 0.025


def quat_to_rot_np(q):
    """Rotation matrix from quaternion q = [w, x, y, z] (numpy)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class LoadStatePublisher(Node):
    def __init__(self):
        super().__init__('load_state_publisher')

        self.n = int(self.declare_parameter('n_drones', N_DRONES).value)
        attach_radius = float(
            self.declare_parameter('attach_radius', ATTACH_RADIUS).value)
        attach_z = float(
            self.declare_parameter('attach_z', ATTACH_Z).value)
        # load-frame attach offsets rho_i, evenly spaced on a circle (same formula
        # as planner_node.py so s_i matches the planner's convention exactly).
        self.rho = [np.array([attach_radius * np.cos(2 * np.pi * k / self.n),
                              attach_radius * np.sin(2 * np.pi * k / self.n),
                              attach_z]) for k in range(self.n)]

        self.get_logger().info(
            f'[load_state] n_drones={self.n} attach_radius={attach_radius:.3f} '
            f'attach_z={attach_z:.3f}')

        self.drone_pos = [None] * self.n

        self.pub = self.create_publisher(Float64MultiArray, '/payload/state', 5)
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)

        self.get_logger().info('[load_state] ready, waiting for mocap...')

    def _drone_cb(self, msg: MotionCaptureState, i):
        p = msg.pose.position
        self.drone_pos[i] = np.array([p.x, p.y, p.z])

    def _payload_cb(self, msg: MotionCaptureState):
        # need every drone position to build the cable directions
        if any(d is None for d in self.drone_pos):
            return

        p = np.array([msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z])
        v = np.array([msg.twist.linear.x,
                      msg.twist.linear.y,
                      msg.twist.linear.z])
        o = msg.pose.orientation
        R = quat_to_rot_np([o.w, o.x, o.y, o.z])

        s = []
        for i in range(self.n):
            attach = p + R @ self.rho[i]
            d = attach - self.drone_pos[i]
            nrm = np.linalg.norm(d)
            if nrm > 1e-6:
                d = d / nrm
            s.append(d)

        data = [float(self.n)] + list(p) + list(v)
        for si in s:
            data += [float(si[0]), float(si[1]), float(si[2])]

        out = Float64MultiArray()
        out.layout.dim = [MultiArrayDimension(
            label='payload_state', size=len(data), stride=len(data))]
        out.data = [float(x) for x in data]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LoadStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
