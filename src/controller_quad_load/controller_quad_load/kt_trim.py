"""
kt_trim.py -- per-drone thrust-gain trim (hover-thrust estimation).

The tracker flies throttle -> acceleration through one gain kT. On the rig that gain
moves with the pack, the airframe and the props, and a tracker with no integrator turns
any gain error into a steady height error (2026-09-23: 0.86 kg never lifted at kT 23.5;
SIL: 10 % typed error = 10-15 cm of hover). This estimates the gain from what the drone
is actually flying, from the tracker's own model

    v_dot_z = kT * u * R33 - g + a_cable_z        (a_cable_z < 0 for a pull)

so at any tick

    kT_inst = (a_meas_z + g - a_cable_z) / (u * R33)

which at a steady hover reduces to "the gain that makes the throttle I am flying hold
the force I am holding". First-order filtered (tau), bounded to typed*(1 +- max), and
only updated when the caller says the drone is in steady airborne flight; otherwise the
last value is held. One scalar per drone, no coupling to the cable model's state: this
is not the removed UKF (CURRENT_STATE §4.5), it is what PX4's hover-thrust estimator does.
"""
import numpy as np

GRAV = 9.81


def hover_thrust_gain(a_meas_z, a_cable_z, u, r33):
    """Instantaneous gain estimate, or None when the throttle/attitude make it
    ill-conditioned (throttle below 0.1 or thrust axis more than 60 deg off vertical)."""
    if u < 0.1 or r33 < 0.5:
        return None
    return (float(a_meas_z) + GRAV - float(a_cable_z)) / (float(u) * float(r33))


class KtTrim:
    def __init__(self, kt_typed, max_frac=0.25, tau_s=1.5, acc_tau_s=0.3, dt=0.02):
        self.kt_typed = float(kt_typed)
        self.lo = self.kt_typed * (1.0 - float(max_frac))
        self.hi = self.kt_typed * (1.0 + float(max_frac))
        self.tau = max(float(tau_s), dt)
        self.acc_tau = max(float(acc_tau_s), dt)
        self.dt = float(dt)
        self.reset()

    def reset(self):
        self.kt_hat = self.kt_typed
        self._vz_prev = None
        self.a_meas_z = 0.0
        self.n_updates = 0

    @property
    def railed(self):
        return self.kt_hat <= self.lo + 1e-9 or self.kt_hat >= self.hi - 1e-9

    def measure_accel(self, vz):
        """Vertical acceleration from the mocap vertical velocity, first-order filtered.
        Call every tick with a pose, whether or not the gain is updated."""
        vz = float(vz)
        if self._vz_prev is not None:
            raw = (vz - self._vz_prev) / self.dt
            self.a_meas_z += (self.dt / self.acc_tau) * (raw - self.a_meas_z)
        self._vz_prev = vz
        return self.a_meas_z

    def in_contact(self, a_cable_z, u, r33, lo=-2.0, hi=5.0):
        """Something other than thrust and cable is carrying force. Judged with the TYPED
        gain (a railed estimate makes the floor look like a hover): the typed model's
        prediction kT_typed u R33 - g + a_cable against the measured acceleration. A drone
        on the floor at 0.28 throttle predicts -3 m/s^2 and measures 0 (the floor holds it;
        Gazebo 2026-09-24: the estimate railed to +25 % and held the fleet down for 14 s);
        on a stand at takeoff throttle it predicts a climb that does not happen. A sagging
        hover under a 25 % typed-high gain predicts +3.6 and measures 0; a typed-low gain
        of 14 % predicts -2.0: both still update."""
        pred = self.kt_typed * float(u) * float(r33) - GRAV + float(a_cable_z)
        d = pred - self.a_meas_z
        return d < lo or d > hi

    def leak_to_typed(self, tau_s=3.0):
        """While in contact, forget toward the typed gain so a railed estimate cannot
        hold the fleet on the ground (an estimate that says "strong motors" flies too
        little throttle to ever leave the floor)."""
        self.kt_hat += (self.dt / max(tau_s, self.dt)) * (self.kt_typed - self.kt_hat)
        return self.kt_hat

    def update(self, a_cable_z, u, r33, steady):
        """One tick. `steady`: the caller's gate (armed, airborne, feedforward fully on,
        no transient). Returns the current kt_hat either way."""
        if not steady:
            return self.kt_hat
        if self.in_contact(a_cable_z, u, r33):
            return self.leak_to_typed()
        inst = hover_thrust_gain(self.a_meas_z, a_cable_z, u, r33)
        if inst is None:
            return self.kt_hat
        self.kt_hat += (self.dt / self.tau) * (inst - self.kt_hat)
        self.kt_hat = float(np.clip(self.kt_hat, self.lo, self.hi))
        self.n_updates += 1
        return self.kt_hat
