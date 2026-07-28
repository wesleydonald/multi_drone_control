"""
mini_plant.py
-------------
A small closed-loop multibody plant for OFFLINE verification of the dissipative
reference generator -- the piece the first harness was missing. It models what Gazebo
actually does at the level that matters for "does the load lift, stay level, and not
collapse":

  * n drones as point masses with a thrust-limited PD+feedforward proxy for the real MPC
    tracker (the tracker itself is off-limits; this is a deliberately simple stand-in),
  * a payload as a RIGID BODY (mass + diagonal inertia) -- it translates AND rotates,
  * RIGID RODS between each drone and its (body-fixed, so rotating) payload attach point
    (stiff two-way spring + damper -- rods push and pull, unlike cables),
  * GROUND CONTACT under the payload (one-way penalty), so the load must be lifted OFF
    the floor by cable tension, exactly the break-off the sim was failing.

The payload ATTITUDE is now simulated (small-angle rigid-body dynamics, torque = sum of
r_i x rod_reaction_i about the COM). This is what lets the harness reproduce the
post-attach TILT / drone-bunching failures that a level-by-construction payload could not
show. Integrated with semi-implicit Euler at a small fixed step; stable for the stiff
rod/ground springs.
"""
import numpy as np


def _skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


class MiniPlant:
    def __init__(self, n, rho, cable_len, drone_mass, load_mass, g, ground_z,
                 k_rod=3000.0, c_rod=25.0, k_ground=6000.0, c_ground=60.0,
                 kp=30.0, kd=8.0, accel_max=22.0, load_inertia=0.004):
        self.n = n
        self.rho = [np.asarray(r, float) for r in rho]
        self.L = float(cable_len)
        self.m_d = float(drone_mass)
        self.m_L = float(load_mass)
        self.g = float(g)
        self.ground_z = float(ground_z)
        self.k_rod, self.c_rod = k_rod, c_rod
        self.k_g, self.c_g = k_ground, c_ground
        self.kp, self.kd = kp, kd
        self.f_max = self.m_d * accel_max      # thrust magnitude cap (N)
        # payload rigid-body inertia (diagonal, body frame) -- a small transported box.
        self.I = np.diag([float(load_inertia)] * 3)
        self.Iinv = np.linalg.inv(self.I)
        self.q = np.zeros((n, 3)); self.vd = np.zeros((n, 3))
        self.xL = np.zeros(3); self.vL = np.zeros(3)
        self.RL = np.eye(3); self.wL = np.zeros(3)   # payload orientation + body rate

    def reset(self, drone_pos, load_pos, load_R=None):
        self.q = np.array([np.asarray(p, float) for p in drone_pos])
        self.vd[:] = 0.0
        self.xL = np.asarray(load_pos, float).copy()
        self.vL[:] = 0.0
        self.RL = np.eye(3) if load_R is None else np.asarray(load_R, float).copy()
        self.wL[:] = 0.0

    def attach(self, i):
        """World position of drone i's payload attach point (rotates with the body)."""
        return self.xL + self.RL @ self.rho[i]

    def _attach_vel(self, i):
        """World velocity of that attach point: COM velocity + rigid-body rotation term."""
        return self.vL + np.cross(self.wL, self.RL @ self.rho[i])

    def step(self, p_ref, v_ref, a_ff, attached, dt):
        """One integration substep. p_ref/v_ref/a_ff: (n,3) held references (the drone's
        commanded state + specific-thrust feedforward). attached: length-n bool. dt: the
        substep length."""
        f_load = np.array([0.0, 0.0, -self.m_L * self.g])
        tau_load = np.zeros(3)                  # torque about the payload COM (world frame)
        f_drone = np.zeros((self.n, 3))
        for i in range(self.n):
            f_drone[i] += np.array([0.0, 0.0, -self.m_d * self.g])
            # thrust proxy: feedforward + PD toward the reference, magnitude-limited.
            a_cmd = (np.asarray(a_ff[i], float)
                     + self.kp * (np.asarray(p_ref[i], float) - self.q[i])
                     + self.kd * (np.asarray(v_ref[i], float) - self.vd[i]))
            f_thr = self.m_d * a_cmd
            fn = float(np.linalg.norm(f_thr))
            if fn > self.f_max:
                f_thr *= self.f_max / fn
            f_drone[i] += f_thr
            if not attached[i]:
                continue
            # rigid rod (two-way stiff spring + damper) between drone i and its (rotating)
            # attach point. The reaction force AND its moment arm act on the payload body.
            r_world = self.RL @ self.rho[i]
            d = self.q[i] - (self.xL + r_world)
            dist = float(np.linalg.norm(d))
            if dist < 1e-9:
                continue
            u = d / dist
            vrel = self.vd[i] - self._attach_vel(i)
            f_rod = -(self.k_rod * (dist - self.L) + self.c_rod * float(vrel @ u)) * u
            f_drone[i] += f_rod
            f_load += -f_rod                    # reaction on the payload COM
            tau_load += np.cross(r_world, -f_rod)   # ... and its torque about the COM
        # ground contact under the payload (one-way, central -- no torque).
        pen = self.ground_z - self.xL[2]
        if pen > 0.0:
            n_force = self.k_g * pen
            if self.vL[2] < 0.0:
                n_force += -self.c_g * self.vL[2]
            f_load[2] += n_force
        # semi-implicit Euler: drones, then payload translation, then payload attitude.
        for i in range(self.n):
            self.vd[i] += (f_drone[i] / self.m_d) * dt
            self.q[i] += self.vd[i] * dt
        self.vL += (f_load / self.m_L) * dt
        self.xL += self.vL * dt
        # rigid-body rotation: I w_dot + w x (I w) = tau  (world-frame diagonal inertia is
        # a fair approximation for a near-symmetric small box and keeps this cheap).
        w_dot = self.Iinv @ (tau_load - np.cross(self.wL, self.I @ self.wL))
        self.wL += w_dot * dt
        self.RL = (np.eye(3) + _skew(self.wL) * dt) @ self.RL
        # re-orthonormalise (Gram-Schmidt) so the small-angle update doesn't drift.
        x = self.RL[:, 0] / max(np.linalg.norm(self.RL[:, 0]), 1e-12)
        y = self.RL[:, 1] - (x @ self.RL[:, 1]) * x
        y = y / max(np.linalg.norm(y), 1e-12)
        z = np.cross(x, y)
        self.RL = np.column_stack([x, y, z])

    def load_quat(self):
        """Payload orientation as a [w,x,y,z] quaternion (from RL) -- the wire format the
        dissipative reference generator consumes as `load_quat`."""
        R = self.RL
        t = np.trace(R)
        if t > 0.0:
            s = np.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
            if i == 0:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
                w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
                w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
        q = np.array([w, x, y, z])
        return q / max(np.linalg.norm(q), 1e-12)

    def load_tilt_deg(self):
        """Payload tilt: angle between its body z-axis and world vertical (deg)."""
        return float(np.degrees(np.arccos(np.clip(self.RL[2, 2], -1.0, 1.0))))

    def rod_tension(self, i):
        """Current rod tension magnitude (N), positive = pulling the drone in / the load
        up. Zero if the rod is compressed."""
        d = self.q[i] - self.attach(i)
        dist = float(np.linalg.norm(d))
        stretch = dist - self.L
        return max(self.k_rod * stretch, 0.0)

    def cable_elev(self, i):
        """Measured cable elevation above horizontal (deg)."""
        d = self.q[i] - self.attach(i)
        nd = float(np.linalg.norm(d))
        return float(np.degrees(np.arcsin(np.clip(d[2] / max(nd, 1e-6), -1, 1))))
