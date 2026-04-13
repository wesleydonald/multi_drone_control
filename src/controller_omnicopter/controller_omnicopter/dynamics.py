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
        self.p = cs.MX.sym('p', 3)  # position
        self.q = cs.MX.sym('a', 4)  # angle quaternion (wxyz)
        self.v = cs.MX.sym('v', 3)  # velocity
        self.r = cs.MX.sym('r', 3)  # angular velocity
        
        # Actuator states (actual motor control values)
        self.actuators = cs.MX.sym('actuators', 8)  # 8 actual actuator states
        
        # Desired actuator states (what we want the actuators to be)
        self.u_desired = cs.MX.sym('u_desired', 8)  # 8 desired actuator states

        # State vector: position, quaternion, velocity, angular velocity, actual actuators, desired actuators
        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.actuators, self.u_desired)
        self.state_dim = 29  # 13 + 8 actual actuators + 8 desired actuators

        # Control input: rate of change of desired actuator values (d(u_desired)/dt)
        u_dot0 = cs.MX.sym('u_dot0')  # rate for desired actuator 0
        u_dot1 = cs.MX.sym('u_dot1')  # rate for desired actuator 1
        u_dot2 = cs.MX.sym('u_dot2')  # rate for desired actuator 2
        u_dot3 = cs.MX.sym('u_dot3')  # rate for desired actuator 3
        u_dot4 = cs.MX.sym('u_dot4')  # rate for desired actuator 4
        u_dot5 = cs.MX.sym('u_dot5')  # rate for desired actuator 5
        u_dot6 = cs.MX.sym('u_dot6')  # rate for desired actuator 6
        u_dot7 = cs.MX.sym('u_dot7')  # rate for desired actuator 7
        
        self.u_dot = cs.vertcat(u_dot0, u_dot1, u_dot2, u_dot3, u_dot4, u_dot5, u_dot6, u_dot7)
        
        # Adaptive estimator parameters for offset mass moments (passed as parameters)
        self.theta_roll = cs.MX.sym('theta_roll')   # Estimated constant moment in roll (x-axis)
        self.theta_pitch = cs.MX.sym('theta_pitch') # Estimated constant moment in pitch (y-axis)
        self.theta_yaw = cs.MX.sym('theta_yaw')     # Estimated constant moment in yaw (z-axis)
        self.thrust_base = cs.MX.sym('thrust_base') # Base thrust coefficient (around 7.42)
        self.motor_time_constant = cs.MX.sym('motor_time_constant')  # Motor time constant (default 0.12)
        self.ixx = cs.MX.sym('ixx')  # Inertia around x-axis
        self.iyy = cs.MX.sym('iyy')  # Inertia around y-axis
        self.izz = cs.MX.sym('izz')  # Inertia around z-axis

        self.params = cs.vertcat(self.theta_roll, self.theta_pitch, self.theta_yaw, self.thrust_base, self.motor_time_constant)
        
        # Fixed motor distance (not adaptive)
        self.motor_distance_fixed = 0.17  # Fixed distance from center to motor (m)


        # Fixed scale factor for thrust constant
        self.thrust_scale = 1e-7  # Fixed e-07 multiplier
        
        # Known mass constant (not adaptive)
        self.mass_constant = 1.00  # kg - known/measured mass

        #init_params = [0.0,0.0,0.0,1.1,7.42678162e-07,0.12,0.07]
        


        self.J = np.array([0.025, 0.025, 0.025])
        self.max_rpm = 4631.0

        # Define motor positions as fixed values (will be scaled by motor_distance parameter in dynamics)
        self.mot_pos_vec_base = 0.17 * np.array([[1, -1, 1],
                                    [-1, -1, 1], 
                                    [1, -1, -1],
                                    [-1, -1, -1],
                                    [1, 1, -1],
                                    [-1, 1, -1],
                                    [1, 1, 1],
                                    [-1, 1, 1]])

        self.mot_rot_vec = np.array([[-0.211325,-0.788675,  -0.57735],  
                                    [0.788675, -0.211325 ,  0.57735],       
                                    [0.211325, 0.788675, -0.57735],    
                                    [-0.788675,  0.211325,   0.57735],     
                                    [0.788675, -0.211325, 0.57735],        
                                    [-0.211325, -0.788675,  -0.57735],     
                                    [-0.788675,  0.211325,   0.57735],       
                                    [0.211325, 0.788675, -0.57735]])     



    
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
        x_dot = cs.vertcat(
            self.p_dynamics(), 
            self.q_dynamics(), 
            self.v_dynamics(), 
            self.w_dynamics(),
            self.actuator_dynamics(),
            self.u_dynamics()
        )
        return cs.Function('x_dot', [self.x, self.u_dot, self.params], [x_dot], ['x', 'u_dot', 'params'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        # Calculate thrust forces from actuator states
        motor_thrusts = self.thrust_base * self.thrust_scale * self.max_rpm * self.max_rpm * (self.actuators)
        
        # Sum all motor thrust forces in body frame (each motor contributes thrust in its direction)
        f_thrust_body = cs.mtimes(self.mot_rot_vec.T, motor_thrusts)
        
        # Rotate body thrust forces to world frame
        f_thrust_world = self.v_dot_q(f_thrust_body, self.q)
        

        g = cs.vertcat(0.0, 0.0, -9.81) 

        v_dynamics = f_thrust_world / self.mass_constant + g

        return v_dynamics

    def w_dynamics(self):
        thrusts =  self.thrust_base * self.thrust_scale * self.max_rpm * self.max_rpm * (self.actuators)

        thrust_torque = cs.MX.zeros(3)
        for i in range(8):
            motor_pos = self.motor_distance_fixed * self.mot_pos_vec_base[i, :]
            thrust_torque += thrusts[i] * cs.cross(motor_pos, self.mot_rot_vec[i, :])


        adaptive_moments = cs.vertcat(self.theta_roll, self.theta_pitch, self.theta_yaw)
        total_torque = thrust_torque + adaptive_moments

        J_inv = cs.diag(1 / self.J)
        angular_acceleration = cs.mtimes(J_inv, total_torque - cs.cross(self.r, cs.mtimes(cs.diag(self.J), self.r)))

        return angular_acceleration
    
    def actuator_dynamics(self):
        """
        First-order actuator dynamics: d(actuator)/dt = (1/tau) * (u_desired - actuator_current)
        where tau is the adaptive motor time constant and u_desired is the desired actuator state
        """
        return (1.0 / self.motor_time_constant) * (self.u_desired - self.actuators)
    
    def u_dynamics(self):
        """
        Desired actuator dynamics: d(u_desired)/dt = u_dot
        where u_dot is the control input (rate of change of desired actuator states)
        """
        return self.u_dot
