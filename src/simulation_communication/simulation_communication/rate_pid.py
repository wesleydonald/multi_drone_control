"""Rate loop of the sim Betaflight stand-in, shared by the sim bridges.

The historical loop is P only (0.5 motor rad/s per deg/s of rate error): a steady torque on
the body can then be held only by a steady rate error, and the stick range ends at 100 deg/s.
A tether pivoting 0.04 m below the body loaded at 45 deg needs ~0.08 N m, i.e. ~260 deg/s of
error, so the drone spins (T0008). A real Betaflight holds that with its I-term. rate_ki adds
one (per second; clamped to 200 motor rad/s ~ 0.13 N m, reset below ~75 % of hover throttle so it
cannot wind up on the floor, T0011).
Class defaults reproduce the old loop exactly; every bridge defaults rate_ki to SIM_RATE_KI or 5: 10 sits
on the 3.2 Hz PI zero (ki/kp = 20 rad/s) and limit-cycled our X3 (R0647) and rocked Tejen's ring (T0017)."""
import numpy as np


def integrate_active(armed, u, u_min):
    """The I-term runs only armed and above u_min throttle: armed on the floor the drone
    cannot rotate, the error persists and the integral winds up (T0011)."""
    return bool(armed) and u > u_min


class RatePid:
    def __init__(self, kp=0.5, ki=0.0, kd=0.0, i_limit=200.0):
        self.kp, self.ki, self.kd, self.i_limit = float(kp), float(ki), float(kd), float(i_limit)
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)

    def step(self, error, dt, active=True):
        error = np.asarray(error, dtype=float)
        if not active or self.ki == 0.0:
            self.integral[:] = 0.0
        elif dt is not None and dt > 0.0:
            self.integral = np.clip(self.integral + self.ki * error * dt,
                                    -self.i_limit, self.i_limit)
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error
        return self.kp * error + self.integral + derivative
