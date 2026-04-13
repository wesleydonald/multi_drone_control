import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# Dynamics for linear MPC 
class QuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 2)  #  xy position
        #self.q = cs.MX.sym('a', 2)  # euler angles (roll and pitch)
        self.q = cs.MX.sym('q', 2)  # euler angles (roll and pitch)
        self.penPos = cs.MX.sym('penPos', 2) # a and b
        self.v = cs.MX.sym('v', 2)  # x and y velocity (previously rotated to global frame)
        #self.r = cs.MX.sym('r', 2)  # roll and pitch angular velocity
        self.penVel = cs.MX.sym('penVel', 2) #a_dot and b_dot 
        

        # State vector: position, quaternion, velocity, angular velocity
        #self.x = cs.vertcat(self.p, self.q, self.penPos, self.v, self.r, self.penVel)
        #self.x = cs.vertcat(self.p, self.q, self.penPos, self.v, self.penVel)


        self.x = cs.vertcat(self.p, self.penPos, self.q, self.v, self.penVel)
        #self.x = cs.vertcat(self.p, self.q, self.v)
        self.state_dim = 10

        # Control input: throttle, desired roll rate, pitch rate, yaw rate (Betaflight style)
        #throttle = cs.MX.sym('throttle')
        roll_rate_cmd = cs.MX.sym('roll_rate_cmd')
        pitch_rate_cmd = cs.MX.sym('pitch_rate_cmd')
        #yaw_rate_cmd = cs.MX.sym('yaw_rate_cmd')
        self.u = cs.vertcat(roll_rate_cmd, pitch_rate_cmd)

        self.mass = 0.45
        self.x_l = 0.173/2
        self.y_l = 0.146/2
        self.J = np.diag([0.001799424313, 0.001522934832, 0.002923509135])
        self.thrust_constant = 40
        self.tau_rate = 0.07
    
    


    def quad_dynamics(self):
        #x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(),self.penPos_dynamics(), self.v_dynamics(), self.w_dynamics(), self.penVel_dynamics())
        #x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(),self.penPos_dynamics(), self.v_dynamics(), self.penVel_dynamics())
        x_dot = cs.vertcat(self.p_dynamics(), self.penPos_dynamics(), self.q_dynamics(), self.v_dynamics(), self.penVel_dynamics())
        #x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics())
        return cs.Function('x_dot', [self.x, self.u], [x_dot], ['x', 'u'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        max_rate_deg = 100
        max_rate_rad = max_rate_deg * np.pi / 180.0
        r_cmd = cs.vertcat(
            self.u[0] * max_rate_rad,
            self.u[1] * max_rate_rad,
        )
        return r_cmd#self.q #1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        #f_thrust = self.thrust_constant * self.u[2]
        #f_thrust = 4 * 1.42e-06 * (self.u[2] *4631)**2

        
        return cs.vertcat(self.q[1]*9.81,self.q[0]*-9.81)
        

    '''def w_dynamics(self):
        max_rate_deg = 100
        max_rate_rad = max_rate_deg * np.pi / 180.0
        r_cmd = cs.vertcat(
            self.u[0] * max_rate_rad,
            self.u[1] * max_rate_rad,
        )
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot'''


    def penPos_dynamics(self):
        return self.penVel

    def penVel_dynamics(self):
    
        g = 9.81
        L = 0.5 #0.3
        a_ddot = self.penPos[0]*g/L - self.q[1]*g
        b_ddot = self.penPos[1]*g/L + self.q[0]*g
        return cs.vertcat(a_ddot, b_ddot)

    
    def skew_symmetric(self, v):
        if isinstance(v, np.ndarray):
            return np.array([[0, -v[0], -v[1], -v[2]],
                            [v[0], 0, v[2], -v[1]],
                            [v[1], -v[2], 0, v[0]],
                            [v[2], v[1], -v[0], 0]])

        return cs.vertcat(
            cs.horzcat(0, -v[0], -v[1], -v[2]),
            cs.horzcat(v[0], 0, v[2], -v[1]),
            cs.horzcat(v[1], -v[2], 0, v[0]),
            cs.horzcat(v[2], v[1], -v[0], 0))


'''class QuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 3)  # position
        self.q = cs.MX.sym('a', 4)  # angle quaternion (wxyz)
        self.penPos = cs.MX.sym('penPos', 2) #a and b
        self.v = cs.MX.sym('v', 3)  # velocity
        self.r = cs.MX.sym('r', 3)  # angular velocity
        self.penVel = cs.MX.sym('penVel', 2) #a_dot and b_dot 
        

        # State vector: position, quaternion, velocity, angular velocity
        self.x = cs.vertcat(self.p, self.q, self.penPos, self.v, self.r, self.penVel)
        self.state_dim = 13 + 4

        # Control input: throttle, desired roll rate, pitch rate, yaw rate (Betaflight style)
        throttle = cs.MX.sym('throttle')
        roll_rate_cmd = cs.MX.sym('roll_rate_cmd')
        pitch_rate_cmd = cs.MX.sym('pitch_rate_cmd')
        yaw_rate_cmd = cs.MX.sym('yaw_rate_cmd')
        self.u = cs.vertcat(throttle, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd)

        self.mass = 0.7
        self.x_l = 0.173/2
        self.y_l = 0.146/2
        self.J = np.diag([0.001799424313, 0.001522934832, 0.002923509135])
        self.thrust_constant = 40
        self.tau_rate = 0.07
    
    def q_to_rot_mat(self, q):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        if isinstance(q, np.ndarray):
            rot_mat = np.array([
                [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]])

        else:
            rot_mat = cs.vertcat(
                cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
                cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
                cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

        return rot_mat

    def v_dot_q(self, v, q):
        rot_mat = self.q_to_rot_mat(q)
        if isinstance(q, np.ndarray):
            return rot_mat.dot(v)

        return cs.mtimes(rot_mat, v)

    def skew_symmetric(self, v):
        if isinstance(v, np.ndarray):
            return np.array([[0, -v[0], -v[1], -v[2]],
                             [v[0], 0, v[2], -v[1]],
                             [v[1], -v[2], 0, v[0]],
                             [v[2], v[1], -v[0], 0]])

        return cs.vertcat(
            cs.horzcat(0, -v[0], -v[1], -v[2]),
            cs.horzcat(v[0], 0, v[2], -v[1]),
            cs.horzcat(v[1], -v[2], 0, v[0]),
            cs.horzcat(v[2], v[1], -v[0], 0))

    def quad_dynamics(self):
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(),self.penPos_dynamics(), self.v_dynamics(), self.w_dynamics(), self.penVel_dynamics())
        return cs.Function('x_dot', [self.x, self.u], [x_dot], ['x', 'u'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        f_thrust = self.thrust_constant * self.u[2]
        #f_thrust = 4 * 1.42e-06 * (self.u[2] *4631)**2


        a_thrust = cs.vertcat(0.0, 0.0, f_thrust) / self.mass
        g = cs.vertcat(0.0, 0.0, 9.81)
        v_dynamics = self.v_dot_q(a_thrust, self.q) - g
        return v_dynamics

    def w_dynamics(self):
        max_rate_deg = 100
        max_rate_rad = max_rate_deg * np.pi / 180.0
        r_cmd = cs.vertcat(
            self.u[0] * max_rate_rad,
            self.u[1] * max_rate_rad,
            -self.u[3] * max_rate_rad 
        )
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot


    def penPos_dynamics(self):
        return self.penVel

    def penVel_dynamics(self):
        state = self.q
        roll, pitch, yaw = self.quaternion_to_euler(state[0], state[1], state[2], state[3])
        g = 9.81
        L = 0.3/2
        a_ddot = self.penPos[0]*g/L - pitch*g
        b_ddot = self.penPos[1]*g/L + roll*g
        return cs.vertcat(a_ddot, b_ddot)

    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = cs.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        #t2 = +1.0 if t2 > +1.0 else t2
        #t2 = -1.0 if t2 < -1.0 else t2
        t2 = cs.if_else(t2>1.0, 1.0, t2)
        t2 = cs.if_else(t2<-1.0, -1.0, t2)
        
        pitch = cs.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = cs.atan2(t3, t4)

        return roll, pitch, yaw
    '''


