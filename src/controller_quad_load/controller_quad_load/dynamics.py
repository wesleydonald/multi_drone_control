import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


class QuadLoadDynamics:
    """Quadrotor model with an external cable-tension acceleration term.

    Identical to controller_mpc_multi.QuadDynamics but v_dynamics adds a world-frame
    cable acceleration a_cable = t_i s_i / m_i, supplied per stage as a parameter by
    the cable-suspended-load planner."""

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

        # Cable tension acceleration on this drone, world frame (a_cable = t*s/m),
        # supplied per stage by the planner so the model accounts for the cable pull.
        self.cable_accel = cs.MX.sym('a_cable', 3)

        # Parameter values for evaluation: 6 dyn params + 3 cable-accel params
        self.p_param = cs.vertcat(self.thrust_ratio, self.drag_coeff_z, self.tau_rate, self.centre_rate_deg, self.max_rate_deg, self.rate_expo, self.cable_accel)  # Parameter vector

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

        # Cable tension acceleration (world frame), supplied by the planner. Pulls the
        # drone toward the load; making the model aware of it stops the tracker's
        # feedback from fighting the cable once it is taut.
        v_dynamics = self.v_dot_q(a_thrust, self.q) - g + drag_force + self.cable_accel

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


if __name__ == "__main__":
    # self-test: a nonzero a_cable parameter must shift v_dot by exactly that vector
    dyn = QuadLoadDynamics()
    f = dyn.quad_dynamics()
    x0 = np.zeros(17); x0[3] = 1.0          # identity quaternion (level)
    p_base = np.array([24.0, 0.0, 0.12, 70.0, 670.0, 0.5, 0.0, 0.0, 0.0])
    a_cable = np.array([0.3, -0.4, -2.5])   # inward/downward cable pull
    p_cab = p_base.copy(); p_cab[6:9] = a_cable
    u0 = np.zeros(4)
    xd0 = np.array(f(x0, u0, p_base)).flatten()
    xd1 = np.array(f(x0, u0, p_cab)).flatten()
    dv = xd1[7:10] - xd0[7:10]              # v_dot indices [7:10]
    print("p_param length =", dyn.p_param.shape[0], "(expect 9)")
    print("v_dot delta    =", np.round(dv, 6), " expect", np.round(a_cable, 6))
    assert np.allclose(dv, a_cable), "cable accel not added correctly"
    print("OK: cable acceleration enters v_dynamics exactly.")