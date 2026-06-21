"""
planner_node.py
---------------
Centralized cable-suspended load planner node (Sun et al. 2025).

Runs the load-cable OCP (planner_ocp.py) at PLANNER_HZ. Each cycle it:
  1. builds x_init from mocap (NO EKF — sim ground truth):
       - load pose/twist from /payload/motion_capture_state
       - cable directions s_i = (p + R(q) rho_i - p_i)/||.|| from drone positions
       - cable rates / tensions / higher derivatives resampled from the previous
         OCP solution (nominal on the first solve)
  2. sets the load-pose reference (hover at captured x,y and TARGET_Z for now)
  3. solves (SQP-RTI), extracts each drone's reference trajectory via the
     kinematic constraint p_i = p + R(q) rho_i - l_i s_i (+ velocity)
  4. publishes /drone_{i}/reference_trajectory (Float64MultiArray):
       [n_nodes, dt, px0,py0,pz0,vx0,vy0,vz0, px1,...]  (world frame)

These references are consumed by the per-drone tracker (controller_mpc_multi),
which replaces its hardcoded circle with this stream (tracker hookup = next step).

Geometry MUST match three_soft.sdf (see generate_soft_world.py defaults).
"""
import numpy as np
import casadi as ca
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from interfaces.msg import MotionCaptureState

from .load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from .planner_ocp import generate_load_ocp, nominal_hover_state

# ── world geometry / params (match three_soft.sdf) ─────────────────────────
N_DRONES      = 3
LOAD_MASS     = 0.4
LOAD_INERTIA  = [1.67e-3, 1.67e-3, 3.33e-3]
CABLE_LEN     = 0.423
ATTACH_RADIUS = 0.08
ATTACH_Z      = 0.025          # attach height above load CoG (load frame)
DRONE_MASS    = 0.6
PLANNER_HZ    = 10.0
TARGET_Z      = 0.6            # load hover height reference


def quat_to_rot_np(q):
    """Rotation matrix from quaternion q = [w, x, y, z] (numpy)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class LoadPlanner(Node):
    def __init__(self):
        super().__init__('load_planner')

        self.n = N_DRONES
        self.rho = [np.array([ATTACH_RADIUS * np.cos(2 * np.pi * k / self.n),
                              ATTACH_RADIUS * np.sin(2 * np.pi * k / self.n),
                              ATTACH_Z]) for k in range(self.n)]

        self.dyn = LoadCableDynamics(
            self.n, LOAD_MASS, LOAD_INERTIA, [CABLE_LEN] * self.n,
            self.rho, DRONE_MASS)
        self.get_logger().info(
            f'[planner] building OCP (nx={self.dyn.nx}, nu={self.dyn.nu})...')
        self.ocp, self.solver = generate_load_ocp(self.dyn)
        self.N = self.ocp.solver_options.N_horizon
        self.dt = self.ocp.solver_options.tf / self.N

        # numeric kinematic functions for reference extraction
        self.pos_fun = [ca.Function(f'p{i}', [self.dyn.x], [self.dyn.quad_position(i)])
                        for i in range(self.n)]
        self.vel_fun = [ca.Function(f'v{i}', [self.dyn.x], [self.dyn.quad_velocity(i)])
                        for i in range(self.n)]

        # state
        self.load_state = None                 # [p(3), q(4 wxyz), v(3), w(3)]
        self.drone_pos = [None] * self.n
        self.last_X = None                     # previous solution (nx, N+1)
        self.hover_xy = None                   # captured load x,y for the reference
        self.first_solve = True

        # subs
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]

        self.create_timer(1.0 / PLANNER_HZ, self._plan)
        self.get_logger().info('[planner] ready, waiting for mocap...')

    # ── mocap callbacks ─────────────────────────────────────────────────────
    def _payload_cb(self, msg: MotionCaptureState):
        p = msg.pose.position
        o = msg.pose.orientation
        lv = msg.twist.linear
        av = msg.twist.angular
        self.load_state = np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z, av.x, av.y, av.z])
        if self.hover_xy is None:
            self.hover_xy = (p.x, p.y)

    def _drone_cb(self, msg: MotionCaptureState, i):
        p = msg.pose.position
        self.drone_pos[i] = np.array([p.x, p.y, p.z])

    # ── x_init assembly ─────────────────────────────────────────────────────
    def _build_x_init(self):
        ls = self.load_state
        p, q = ls[0:3], ls[3:7]
        R = quat_to_rot_np(q)
        # base: resample previous solution (advance one node) or nominal hover
        if self.last_X is not None:
            x = self.last_X[:, 1].copy()
        else:
            x = nominal_hover_state(self.dyn, load_pos=tuple(p))
        # overwrite load block with fresh mocap
        x[0:3] = p
        x[3:6] = ls[7:10]      # v
        x[6:10] = q            # q (wxyz)
        x[10:13] = ls[10:13]   # w
        # overwrite each cable direction s_i from geometry (keep rates/tension)
        for i in range(self.n):
            attach = p + R @ self.rho[i]
            d = attach - self.drone_pos[i]
            nrm = np.linalg.norm(d)
            if nrm > 1e-6:
                b = LOAD_DIM + CABLE_DIM * i
                x[b:b + 3] = d / nrm
        return x

    def _yref(self):
        x0, y0 = self.hover_xy
        t_nom = self.dyn.m * 9.81 / (self.n * np.sin(np.deg2rad(45.0)))
        pose = [x0, y0, TARGET_Z, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # p, v, e_att, w
        y = np.array(pose + [t_nom] * self.n
                     + [0.0] * (3 * self.n)        # r_vec ref (no cable swing)
                     + [0.0] * self.dyn.nu)
        return y, y[:-self.dyn.nu]

    # ── plan step ───────────────────────────────────────────────────────────
    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

        x_init = self._build_x_init()
        yref, yref_e = self._yref()
        q_ref = np.array([1.0, 0.0, 0.0, 0.0])

        for k in range(self.N):
            self.solver.set(k, 'yref', yref)
            self.solver.set(k, 'p', q_ref)
        self.solver.set(self.N, 'yref', yref_e)
        self.solver.set(self.N, 'p', q_ref)

        if self.first_solve:                       # cold start: seed all nodes
            for k in range(self.N + 1):
                self.solver.set(k, 'x', x_init)
        self.solver.set(0, 'lbx', x_init)
        self.solver.set(0, 'ubx', x_init)

        iters = 15 if self.first_solve else 1      # converge once, then RTI
        status = 0
        for _ in range(iters):
            status = self.solver.solve()
        self.first_solve = False

        if status != 0:
            self.get_logger().warn(f'[planner] solve status {status}')
            return

        X = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)]).T
        self.last_X = X
        self._publish_refs(X)

    def _publish_refs(self, X):
        for i in range(self.n):
            data = [float(self.N + 1), float(self.dt)]
            for k in range(self.N + 1):
                xk = X[:, k]
                p_i = np.array(self.pos_fun[i](xk)).flatten()
                v_i = np.array(self.vel_fun[i](xk)).flatten()
                data += [*p_i, *v_i]
            msg = Float64MultiArray()
            msg.data = data
            self.ref_pub[i].publish(msg)


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
