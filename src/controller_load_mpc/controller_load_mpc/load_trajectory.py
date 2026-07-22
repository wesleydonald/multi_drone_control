"""
load_trajectory.py
------------------
The lateral LOAD reference trajectory, factored out of the planner node. Once the
lift tops out the whole taut formation is translated along this path, so the load
follows it. A LoadTrajectory is a pure function of trajectory time t and the four
shape params (kind, speed, distance, radius) -- it holds no node/solver state (the
trajectory CLOCK traj_t stays in the node) so it is testable in isolation.

Supported kinds: 'hover' (no motion), 'line_x' (sinusoidal shuttle along +x),
'circle', 'fig_8' (Gerono lemniscate), 'spin' (circle + one full load yaw). All use
an eased (smootherstep) angle sweep so the maneuver starts and ends at rest -- no
velocity/accel step that a non-fed-forward payload would take as a pendulum kick.
"""
import numpy as np


class LoadTrajectory:
    def __init__(self, kind, speed, distance, radius):
        self.kind = str(kind)
        self.speed = float(speed)            # m/s peak lateral speed
        self.distance = float(distance)      # m (line_x)
        self.radius = float(radius)          # m (circle / fig_8 / spin)

    @staticmethod
    def _eased_sweep(t, T):
        """Smootherstep angle sweep. theta goes 0 -> 2*pi over duration T via
        6u^5-15u^4+10u^3, whose 1st AND 2nd derivatives vanish at both ends, so the
        maneuver spins up from rest and winds down to rest (no velocity step /
        pendulum kick at either end). Peak d(theta)/dt = 2*pi/T * 1.875 at u=0.5.
        Returns (theta, dtheta/dt, d2theta/dt2)."""
        u = min(max(t / T, 0.0), 1.0)
        s   = u * u * u * (u * (6.0 * u - 15.0) + 10.0)   # 6u^5-15u^4+10u^3
        ds  = 30.0 * u * u * (u - 1.0) * (u - 1.0)        # 30u^2(u-1)^2
        dds = 60.0 * u * (2.0 * u - 1.0) * (u - 1.0)      # 60u(2u-1)(u-1)
        two_pi = 2.0 * np.pi
        return two_pi * s, two_pi * ds / T, two_pi * dds / (T * T)

    def _circle_theta(self, t):
        """Eased circle/spin angle (see _eased_sweep). Duration is stretched so the
        PEAK tangential speed equals speed (peak speed = r * dtheta_max).
        Returns (theta, dtheta/dt, d2theta/dt2, T)."""
        r = max(self.radius, 1e-6)
        T = 1.875 * 2.0 * np.pi * r / max(self.speed, 1e-6)
        th, dth, ddth = self._eased_sweep(t, T)
        return th, dth, ddth, T

    def _fig8_theta(self, t):
        """Eased figure-eight angle (see _eased_sweep). The lemniscate's peak speed
        is a*sqrt(2)*dtheta_max (at the centre crossing), so T is stretched by the
        extra sqrt(2) to keep the peak tangential speed at speed. Returns
        (theta, dtheta/dt, d2theta/dt2, T)."""
        a = max(self.radius, 1e-6)
        T = 1.875 * 2.0 * np.pi * a * np.sqrt(2.0) / max(self.speed, 1e-6)
        th, dth, ddth = self._eased_sweep(t, T)
        return th, dth, ddth, T

    def complete(self, traj_t):
        """True once the lateral trajectory has finished, so the descent can begin.
        'hover' and 'line_x' never self-complete (hold / shuttle indefinitely, end
        the run with LAND); circle/spin/fig_8 finish after one eased sweep."""
        if self.kind == 'line_x':
            # Shuttles continuously - like 'hover' it never self-completes, so
            # there is no auto-descent. End the run with a LAND command.
            return False
        if self.kind in ('circle', 'spin'):
            # eased-circle duration (see _circle_theta); T is independent of t.
            return traj_t >= self._circle_theta(0.0)[3]
        if self.kind == 'fig_8':
            return traj_t >= self._fig8_theta(0.0)[3]
        return False

    def offset_at(self, t):
        """Lateral (x, y) offset + velocity of the LOAD reference at absolute
        trajectory time t. The whole taut formation is translated by this, so the
        load follows it. Returns (dx, dy, vx, vy); zero at/before t=0."""
        # At t=0 the velocity terms are not zero (vx = speed*cos(0)), so the
        # drones were handed a full-speed reference at startup while the position
        # reference sat still. Hold at zero until the trajectory starts.
        if t <= 0.0:
            return 0.0, 0.0, 0.0, 0.0
        if self.kind == 'line_x':
            # Shuttle along +x, 0 -> distance -> 0, repeating until LAND.
            # Sinusoidal rather than a triangle wave: constant speed reverses
            # velocity instantly at each end, and with lateral accel not fed
            # forward the payload takes that step as a pendulum kick. The
            # half-cosine has continuous velocity and accel and starts from rest.
            # w gives peak speed speed; period = pi*distance/speed.
            d = max(self.distance, 1e-6)
            w = 2.0 * self.speed / d
            dx = 0.5 * d * (1.0 - np.cos(w * t))
            vx = 0.5 * d * w * np.sin(w * t)
            return dx, 0.0, vx, 0.0
        if self.kind in ('circle', 'spin'):
            # 'spin' has the SAME circular load path as 'circle'; it additionally
            # yaws the load (formation rotates about the payload, see yaw_at).
            r = max(self.radius, 1e-6)
            th, dth, _, T = self._circle_theta(t)
            if t >= T:
                # completed one revolution -> back at the start point, hold there.
                return 0.0, 0.0, 0.0, 0.0
            dx = r * np.sin(th)                        # starts at (0,0), heads +x
            dy = r * (1.0 - np.cos(th))               # then curves +y
            vx = r * np.cos(th) * dth
            vy = r * np.sin(th) * dth
            return dx, dy, vx, vy
        if self.kind == 'fig_8':
            # Gerono lemniscate through the origin: x = a*sin(th), y = a/2*sin(2th),
            # th swept 0 -> 2*pi (eased). Traces the right lobe then the left,
            # crossing the start point at th=pi; starts and ends at rest at (0,0).
            a = max(self.radius, 1e-6)
            th, dth, _, T = self._fig8_theta(t)
            if t >= T:
                return 0.0, 0.0, 0.0, 0.0
            dx = a * np.sin(th)
            dy = 0.5 * a * np.sin(2.0 * th)
            vx = a * np.cos(th) * dth
            vy = a * np.cos(2.0 * th) * dth
            return dx, dy, vx, vy
        return 0.0, 0.0, 0.0, 0.0

    def accel_at(self, t):
        """Lateral (ax, ay) ACCELERATION of the LOAD reference at trajectory time t.
        Used for the flatness-based cable references. Analytic 2nd derivative of the
        offsets above; zero at/before t=0."""
        if t <= 0.0:
            return 0.0, 0.0
        if self.kind == 'line_x':
            d = max(self.distance, 1e-6)
            w = 2.0 * self.speed / d
            return 0.5 * d * w * w * np.cos(w * t), 0.0
        if self.kind in ('circle', 'spin'):
            r = max(self.radius, 1e-6)
            th, dth, ddth, T = self._circle_theta(t)
            if t >= T:
                return 0.0, 0.0
            ax = r * (-np.sin(th) * dth * dth + np.cos(th) * ddth)
            ay = r * (np.cos(th) * dth * dth + np.sin(th) * ddth)
            return ax, ay
        if self.kind == 'fig_8':
            a = max(self.radius, 1e-6)
            th, dth, ddth, T = self._fig8_theta(t)
            if t >= T:
                return 0.0, 0.0
            # dx = a sin th; dy = a/2 sin 2th
            ax = a * (-np.sin(th) * dth * dth + np.cos(th) * ddth)
            ay = a * (-2.0 * np.sin(2.0 * th) * dth * dth + np.cos(2.0 * th) * ddth)
            return ax, ay
        return 0.0, 0.0

    def yaw_at(self, t):
        """Load YAW angle + rate at trajectory time t. Nonzero only for 'spin': the
        load rotates one full turn about its vertical axis, synchronised with the
        circular path (same eased sweep), so the drone formation orbits the payload
        while the payload also circles. Zero (level) for every other trajectory.
        Returns (yaw, yaw_rate)."""
        if t <= 0.0 or self.kind != 'spin':
            return 0.0, 0.0
        th, dth, _, T = self._circle_theta(t)
        if t >= T:
            return 0.0, 0.0                            # closed a full turn, hold level
        return th, dth
