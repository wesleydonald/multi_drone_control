"""The sim thrust ratio (kT) as an OPERATING POINT, derived instead of hand-tuned.

The tracker flies a linear thrust model a = kT * u. Gazebo's motor model is quadratic,
a = c * u^2 with c = 4 * motorConstant * maxRotVelocity^2 / mass (88.6 for the x3
airframe), so the kT a linear model must use is the SECANT gain at the hover throttle,
kT* = c * u_hover -- and u_hover moves with everything that changes the thrust a drone
needs at hover: its load share, and the cable angle (the horizontal cable pull has to
be cancelled too). The 32.9 the launches carried was measured at load_mass 0.4 with
three drones; at 0.6 kg the same number floated the load 18 cm above target (SIL
R0230 vs R0228, 2026-09-10) with identical throttle, because a wrong kT lands in
POSITION on an integrator-free tracker, never in throttle.

Hardware kT is a measured airframe property (24 at full battery) and is NOT derived
here -- the real thrust curve is not the sim's parabola.
"""
import math

G = 9.81
# x3 drone model (simulation_assets/models/x3_drone*.sdf): test_thrust_model ties these
# to the SDF so an airframe edit cannot silently leave the derived kT behind.
SIM_DRONE_MASS = 0.6
SIM_MOTOR_CONSTANT = 0.62e-06
SIM_MAX_ROT_VELOCITY = 4631.0
# takeoff_thrust_ratio sits below kT on purpose (the over-thrust pops the drones off
# their stands on a taut air-start); 30.0/32.9 was the working ratio at 0.4 kg.
TAKEOFF_POP_FRAC = 30.0 / 32.9


def thrust_c(motor_constant=SIM_MOTOR_CONSTANT, max_rot_velocity=SIM_MAX_ROT_VELOCITY,
             mass=SIM_DRONE_MASS):
    """Quadratic thrust coefficient: a = thrust_c * u^2  [m/s^2 per unit throttle^2]."""
    return 4.0 * motor_constant * max_rot_velocity ** 2 / mass


def hover_thrust_accel(load_mass, n_drones, drone_mass=SIM_DRONE_MASS, elev_deg=45.0):
    """Thrust acceleration one drone needs at a level hover: its own weight plus its
    load share vertically, plus the horizontal component of the cable pull at the
    cable elevation (the cables are not vertical, so the tension exceeds the share)."""
    share = load_mass * G / max(n_drones, 1)
    horiz = share / math.tan(math.radians(elev_deg)) if elev_deg < 90.0 else 0.0
    return math.hypot(drone_mass * G + share, horiz) / drone_mass


def hover_throttle(load_mass, n_drones, drone_mass=SIM_DRONE_MASS, c=None, elev_deg=45.0):
    c = thrust_c() if c is None else c
    return math.sqrt(hover_thrust_accel(load_mass, n_drones, drone_mass, elev_deg) / c)


def secant_kt(load_mass, n_drones, drone_mass=SIM_DRONE_MASS, c=None, elev_deg=45.0):
    """kT* = c * u_hover: the linear gain that matches the parabola at the hover point."""
    c = thrust_c() if c is None else c
    return c * hover_throttle(load_mass, n_drones, drone_mass, c, elev_deg)


def resolve_thrust_ratio(thrust_ratio, takeoff_thrust_ratio, load_mass, n_drones,
                         elev_deg=45.0):
    """Turn the launch's thrust_ratio / takeoff_thrust_ratio specs into numbers.
    Either may be 'auto' (derive from the operating point) or a number (explicit,
    kept verbatim). Returns (kT, takeoff_kT, note)."""
    spec = str(thrust_ratio).strip().lower()
    if spec == 'auto':
        kt = secant_kt(float(load_mass), int(n_drones), elev_deg=elev_deg)
        note = (f'kT auto {kt:.2f} = secant of a=c*u^2 at load {float(load_mass):.2f} kg '
                f'/ {int(n_drones)} drones, cable elev {elev_deg:.0f} deg')
    else:
        kt = float(spec)
        note = f'kT explicit {kt:.2f}'
    tspec = str(takeoff_thrust_ratio).strip().lower()
    kt_to = kt * TAKEOFF_POP_FRAC if tspec == 'auto' else float(tspec)
    return kt, kt_to, note
