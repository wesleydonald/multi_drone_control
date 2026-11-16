"""
rig_thrust.py
-------------
The rig's measured thrust law and battery pack, as an optional sim plant (thrust_map 'rig').

Law (RIG-0930-ladder4, tools/thrust_fit.py): a hovering drone of mass M needs
u = a_i + b M + c (V - V_REF), so a throttle u gives

    T = (G / b) * max(0, u - a_i - c (V - V_REF))      [N, per drone]

with c negative (a full pack needs less throttle). Pure python, no ROS: the Gazebo bridge,
sim_telemetry and the SIL plant share it.
"""
import math

G = 9.81
B = 0.507           # throttle per kg
C = -0.0219         # throttle per volt
V_REF = 23.5
K = G / B           # N per unit throttle above the offset
A_DEFAULT = 0.185

# Gazebo x3 rotor: thrust = motorConstant * omega^2 per rotor (x3_drone*.sdf).
MOTOR_CONSTANT = 0.62e-06
MAX_ROT_VEL = 4631.0

THRUST_MAPS = ('linear', 'rig')


def rig_thrust(u, v, a=A_DEFAULT):
    """Total thrust [N] of one drone at throttle u (0..1) and pack voltage v."""
    return K * max(0.0, float(u) - float(a) - C * (float(v) - V_REF))


def linear_thrust(u, motor_constant=MOTOR_CONSTANT, max_rot_vel=MAX_ROT_VEL):
    """The linear sim map: rotor speed sqrt(u) * max, so thrust 4 mc max^2 u."""
    return 4.0 * motor_constant * max_rot_vel ** 2 * max(0.0, min(1.0, float(u)))


def rotor_speed(thrust, motor_constant=MOTOR_CONSTANT, max_rot_vel=MAX_ROT_VEL):
    """Common rotor speed whose four rotors give `thrust` in Gazebo, clipped at max."""
    return min(math.sqrt(max(0.0, float(thrust)) / (4.0 * motor_constant)), max_rot_vel)


class PackModel:
    """Fitted pack sag: V = V_ocv - R u through a tau low-pass, V_ocv draining with the
    integrated throttle. The fit is ill-conditioned in tau (1-20 s all fit), so voltage
    conclusions should come from the 22.8 / 24.6 V neighbours, not from this model."""

    def __init__(self, v0=24.4, r_sag=1.0, drain=0.022, tau=5.0, quant=0.1):
        self.v0, self.r_sag, self.drain = float(v0), float(r_sag), float(drain)
        self.tau, self.quant = float(tau), float(quant)
        self.reset()

    def reset(self):
        self.used = 0.0             # throttle-seconds
        self.v = self.v0

    def ocv(self):
        return self.v0 - self.drain * self.used

    def step(self, u, dt):
        u = max(0.0, min(1.0, float(u)))
        dt = max(0.0, float(dt))
        self.used += u * dt
        target = self.ocv() - self.r_sag * u
        alpha = 1.0 if self.tau <= 0.0 else 1.0 - math.exp(-dt / self.tau)
        self.v += alpha * (target - self.v)
        return self.v

    def measured(self):
        """What the telemetry link reports: quantised like the flight controller."""
        if self.quant <= 0.0:
            return self.v
        return round(round(self.v / self.quant) * self.quant, 6)
