import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
        
class QuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 3)
        self.q = cs.MX.sym('a', 4)
        self.v = cs.MX.sym('v', 3) 
        self.r = cs.MX.sym('r', 3) 
        self.u = cs.MX.sym('u', 4)

        self.x = cs.vertcat(self.p, self.q, self.v, self.r,self.u)
        self.state_dim = 17

        self.u_dot = cs.MX.sym('u_dot', 4)


        self.thrust_ratio = cs.MX.sym('kT', 1)
        self.tau_rate = cs.MX.sym('tau', 1)
        self.drag_coeff_z = cs.MX.sym('drag_coeff_z', 1)
        self.centre_rate_deg = cs.MX.sym('centre_rate_deg', 1)
        self.max_rate_deg = cs.MX.sym('max_rate_deg', 1)
        self.rate_expo = cs.MX.sym('rate_expo', 1)
        # Betaflight rates parameters
        
        # Parameter values for evaluation
        self.p_param = cs.vertcat(self.thrust_ratio,self.drag_coeff_z, self.tau_rate,self.centre_rate_deg, self.max_rate_deg ,self.rate_expo)  # Parameter vector

        self.g = 9.81
    
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
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics(), self.u_dynamics())
        return cs.Function('x_dot', [self.x, self.u_dot, self.p_param], [x_dot], ['x', 'u', 'p'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)
    
    def v_dynamics(self):
        a_thrust = cs.vertcat(0.0, 0.0, self.thrust_ratio * self.u[2]) 
        g = cs.vertcat(0.0, 0.0, 9.81)

                # Linear drag force - proportional to velocity
        # Only vertical drag for now since Z-axis is the focus
        drag_force = cs.vertcat(
            0.0,  # No X drag
            0.0,  # No Y drag  
            -self.drag_coeff_z * self.v[2]  # Vertical drag proportional to Z velocity
        )
        
        # Gravity vector
        g = cs.vertcat(0.0, 0.0, 9.81)
        
        v_dynamics = self.v_dot_q(a_thrust, self.q) - g + drag_force

        return v_dynamics

    def betaflight_rates(self, x):
        """
        Betaflight rates formula:
        h = x * (x^5 * g + x * (1-g))
        j = (d * x) + ((f-d) * h) for x in [-1, 1]
        The mapping from -1 to 0 is the inverted version of 0 to 1
        """
        # Ensure x is clamped between -1 and 1
        #x_clamped = cs.fmax(-1.0, cs.fmin(1.0, x))

        ax  = cs.sqrt(x*x + 1e-6) 
        
        # Work with absolute value for the curve calculation
        sgn = x / ax   
        
        # Calculate h = x * (x^5 * g + x * (1-g)) using absolute value
        h_abs = ax * (cs.power(ax, 5) * self.rate_expo  + ax * (1.0 - self.rate_expo ))
        
        # Calculate j = (d * |x|) + ((f-d) * h) for the positive curve
        j_abs = self.centre_rate_deg * ax + (self.max_rate_deg  - self.centre_rate_deg) * h_abs
        
        # Apply the sign to get symmetric behavior around zero
        j = sgn * j_abs
        
        return j

    
    def w_dynamics(self):
        """
        Angular rate dynamics using Betaflight rates formula
        """
        # Apply Betaflight rates to each control input
        r_cmd = cs.vertcat(
            (self.betaflight_rates(self.u[0]) * np.pi) / 180.0,
            (self.betaflight_rates(self.u[1]) * np.pi) / 180.0,
            (self.betaflight_rates(-self.u[3]) * np.pi) / 180.0 
        )
        
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot
    
    '''
    # Original linear mapping implementation (commented out)
    def w_dynamics(self):
        max_rate_rad = self.max_rate_deg * np.pi / 180.0

        r_cmd = cs.vertcat(
            self.u[0] * max_rate_rad,
            self.u[1] * max_rate_rad,
            -self.u[3] * max_rate_rad 
        )
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)

        return r_dot'''
    
    def u_dynamics(self):
        return self.u_dot

class QuadPayloadDynamics(QuadDynamics):
    def __init__(self):
        super().__init__()

        # payload-quad vector RPY parameters. no cable yaw considered
        self.phi = cs.MX.sym('phi', 1)
        self.theta = cs.MX.sym('theta', 1)
        self.phi_dot = cs.MX.sym('phi_dot', 1)
        self.theta_dot = cs.MX.sym('theta_dot', 1)

        self.x = cs.vertcat(
            self.p,
            self.q,
            self.v,
            self.r,
            self.u,
            self.phi,
            self.theta,
            self.phi_dot,
            self.theta_dot
        )

        self.state_dim = 21

        # symbolic variable of cable_length but we know what the lenght is
        self.cable_length = cs.MX.sym('cable_length', 1)

        self.p_param = cs.vertcat(
            self.thrust_ratio,
            self.drag_coeff_z,
            self.tau_rate,
            self.centre_rate_deg,
            self.max_rate_deg,
            self.rate_expo,
            self.cable_length
        )

    def payload_dynamics(self):
        a_quad = self.v_dynamics()

        L = self.cable_length
        g = self.g

        phi_ddot = -(g / L) * self.phi - a_quad[0] / L
        theta_ddot = -(g / L) * self.theta - a_quad[1] / L

        return cs.vertcat(
            self.phi_dot,
            self.theta_dot,
            phi_ddot,
            theta_ddot
        )

    def quad_dynamics(self):
        x_dot = cs.vertcat(
            self.p_dynamics(),
            self.q_dynamics(),
            self.v_dynamics(),
            self.w_dynamics(),
            self.u_dynamics(),
            self.payload_dynamics()
        )

        return cs.Function(
            'x_dot',
            [self.x, self.u_dot, self.p_param],
            [x_dot],
            ['x', 'u', 'p'],
            ['x_dot']
        )

class QuadLoadTautDynamics(QuadDynamics):
    """
    Sreenath-inspired taut-cable quadrotor/load model.

    State layout:
        x[0:3]    = pQ, quad position
        x[3:7]    = qQ, quad attitude quaternion [qw, qx, qy, qz]
        x[7:10]   = vQ, quad velocity
        x[10:13]  = omega, body rates
        x[13:17]  = control state [roll, pitch, throttle, yaw]
        x[17:20]  = pL, payload/load world position
        x[20:23]  = vL, payload/load world velocity

    Parameter layout:
        p[0] = thrust_ratio
        p[1] = drag_coeff_z
        p[2] = tau_rate
        p[3] = centre_rate_deg
        p[4] = max_rate_deg
        p[5] = rate_expo
        p[6] = cable_length
        p[7] = quad_mass
        p[8] = payload_mass
    """
    def __init__(self):
        super().__init__()

        self.p_load = cs.MX.sym('p_load', 3)
        self.v_load = cs.MX.sym('v_load', 3)

        self.x = cs.vertcat(
            self.p,
            self.q,
            self.v,
            self.r,
            self.u,
            self.p_load,
            self.v_load
        )

        self.state_dim = 23

        self.cable_length = cs.MX.sym('cable_length', 1)
        self.quad_mass = cs.MX.sym('quad_mass', 1)
        self.payload_mass = cs.MX.sym('payload_mass', 1)

        self.p_param = cs.vertcat(
            self.thrust_ratio,
            self.drag_coeff_z,
            self.tau_rate,
            self.centre_rate_deg,
            self.max_rate_deg,
            self.rate_expo,
            self.cable_length,
            self.quad_mass,
            self.payload_mass
        )

    def cable_kinematics_and_tension(self):
        e3 = cs.vertcat(0.0, 0.0, 1.0)
        g_vec = self.g * e3

        r_cable = self.p - self.p_load
        r_norm = cs.sqrt(cs.dot(r_cable, r_cable) + 1e-9)
        q_cable = r_cable / r_norm

        projection = cs.MX.eye(3) - cs.mtimes(q_cable, q_cable.T)
        q_dot = cs.mtimes(projection, self.v - self.v_load) / self.cable_length

        a_quad_no_cable = QuadDynamics.v_dynamics(self)

        tension_raw = (
            self.payload_mass * cs.dot(q_cable, a_quad_no_cable + g_vec)
            + self.payload_mass * self.cable_length * cs.dot(q_dot, q_dot)
        )
        tension = cs.fmax(tension_raw, 0.0)

        return q_cable, q_dot, tension, a_quad_no_cable

    def v_dynamics(self):
        q_cable, _, tension, a_quad_no_cable = self.cable_kinematics_and_tension()
        return a_quad_no_cable - (tension / self.quad_mass) * q_cable

    def load_dynamics(self):
        q_cable, _, tension, _ = self.cable_kinematics_and_tension()
        gravity = cs.vertcat(0.0, 0.0, self.g)
        load_accel = (tension / self.payload_mass) * q_cable - gravity

        return cs.vertcat(
            self.v_load,
            load_accel
        )

    def quad_dynamics(self):
        x_dot = cs.vertcat(
            self.p_dynamics(),
            self.q_dynamics(),
            self.v_dynamics(),
            self.w_dynamics(),
            self.u_dynamics(),
            self.load_dynamics()
        )

        return cs.Function(
            'x_dot',
            [self.x, self.u_dot, self.p_param],
            [x_dot],
            ['x', 'u', 'p'],
            ['x_dot']
        )
