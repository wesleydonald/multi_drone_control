"""
mini_plant.py
-------------
A small closed-loop multibody plant for OFFLINE verification of the dissipative
reference generator -- the piece the first harness was missing. It models what Gazebo
actually does at the level that matters for "does the load lift and not collapse":

  * n drones as point masses with a thrust-limited PD+feedforward proxy for the real MPC
    tracker (the tracker itself is off-limits; this is a deliberately simple stand-in),
  * a payload point mass,
  * RIGID RODS between each drone and its payload attach point (stiff two-way spring +
    damper -- rods push and pull, unlike cables),
  * GROUND CONTACT under the payload (one-way penalty), so the load must be lifted OFF
    the floor by cable tension, exactly the break-off the sim was failing.

The payload is kept level (no rotation) -- this harness verifies vertical lift and
inward/outward lean of the references, not load attitude. Integrated with semi-implicit
Euler at a small fixed step; stable for the stiff rod/ground springs.
"""
import numpy as np


class MiniPlant:
    def __init__(self, n, rho, cable_len, drone_mass, load_mass, g, ground_z,
                 k_rod=3000.0, c_rod=25.0, k_ground=6000.0, c_ground=60.0,
                 kp=30.0, kd=8.0, accel_max=22.0):
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
        self.q = np.zeros((n, 3)); self.vd = np.zeros((n, 3))
        self.xL = np.zeros(3); self.vL = np.zeros(3)

    def reset(self, drone_pos, load_pos):
        self.q = np.array([np.asarray(p, float) for p in drone_pos])
        self.vd[:] = 0.0
        self.xL = np.asarray(load_pos, float).copy()
        self.vL[:] = 0.0

    def attach(self, i):
        return self.xL + self.rho[i]           # level load

    def step(self, p_ref, v_ref, a_ff, attached, dt):
        """One integration substep. p_ref/v_ref/a_ff: (n,3) held references (the drone's
        commanded state + specific-thrust feedforward). attached: length-n bool. dt: the
        substep length."""
        f_load = np.array([0.0, 0.0, -self.m_L * self.g])
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
            # rigid rod (two-way stiff spring + damper) between drone i and its attach.
            d = self.q[i] - self.attach(i)
            dist = float(np.linalg.norm(d))
            if dist < 1e-9:
                continue
            u = d / dist
            vrel = self.vd[i] - self.vL
            f_rod = -(self.k_rod * (dist - self.L) + self.c_rod * float(vrel @ u)) * u
            f_drone[i] += f_rod
            f_load += -f_rod                    # reaction on the payload
        # ground contact under the payload (one-way).
        pen = self.ground_z - self.xL[2]
        if pen > 0.0:
            n_force = self.k_g * pen
            if self.vL[2] < 0.0:
                n_force += -self.c_g * self.vL[2]
            f_load[2] += n_force
        # semi-implicit Euler.
        for i in range(self.n):
            self.vd[i] += (f_drone[i] / self.m_d) * dt
            self.q[i] += self.vd[i] * dt
        self.vL += (f_load / self.m_L) * dt
        self.xL += self.vL * dt

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
