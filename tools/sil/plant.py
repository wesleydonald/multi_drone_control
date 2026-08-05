"""
tools/sil/plant.py
------------------
The SIL bench plant: a fast numerical stand-in for Gazebo, pure numpy, no ROS.

Design note: docs/design/sil_bench.md. Read §2 and §7 there before trusting a number
that comes out of this file.

What it models, and why each piece is here:

  * n quadrotors as 6-DOF rigid bodies -- position, velocity, ATTITUDE and body rate.
    The attitude is the point. `mini_plant`'s drone is a point mass with an
    omnidirectional force, so it can push sideways for free; a real drone must TILT to
    push sideways, which costs it vertical thrust, and its rate loop lags. A drone
    being dragged outward by a cable therefore SINKS, and that sink is the documented
    attach-runaway mechanism (2026-07-28: "ERR y grows REF 0.30 -> ACT 1.06, |aCm| ->
    20, tilt 10 -> 33 -> 72 deg"). A point mass cannot express it.

  * QUADRATIC thrust a = c*u^2 (c = 88.6 for the sim airframe: 4 rotors,
    motorConstant 0.62e-06, maxRotVelocity 4631, mass 0.6). The tracker's model assumes
    a LINEAR a = kT*u, so the mismatch away from the operating point is real and is the
    reason _scheduled_kT / AIRBORNE_MARGIN / the kT estimator all exist. A linear plant
    would delete that error term and flatter the tracker.

  * The betaflight rate curve and the armed 5% throttle floor, decoded exactly as
    simulation_communication/payload_betaflight_comm.py decodes them, so the bench
    consumes the same ELRSCommand the sim does.

  * The payload as a RIGID BODY (mass + diagonal inertia from the world SDF), rigid
    two-way rods with ball ends, and one-way ground contact. Same structure as
    mini_plant, which is gate-proven -- kept deliberately close to it.

NOT modelled (see docs/design/sil_bench.md §7): contact/collision geometry,
aerodynamics, motor mixing and ESC saturation, mocap noise/latency, rotor inertia,
magnet-arm swing inertia. The rate time constant is shared with the tracker's model,
so the bench cannot show a failure caused by rate-loop mismatch.
"""
from dataclasses import dataclass, field

import numpy as np

G = 9.81


# ── quaternion helpers (w, x, y, z) ──────────────────────────────────────────

def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_integrate(q, w_body, dt):
    """q_dot = 1/2 * q (x) [0, w_body], integrated one step and renormalised."""
    w, x, y, z = q
    p, qq, r = w_body
    dq = 0.5 * np.array([
        -x * p - y * qq - z * r,
        w * p + y * r - z * qq,
        w * qq - x * r + z * p,
        w * r + x * qq - y * p,
    ])
    out = np.asarray(q, float) + dq * dt
    return out / max(float(np.linalg.norm(out)), 1e-12)


def rot_to_quat(R):
    t = float(np.trace(R))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                          (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
        elif i == 1:
            s = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
            q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                          0.25 * s, (R[1, 2] + R[2, 1]) / s])
        else:
            s = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
            q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                          (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    # Canonicalise to w >= 0. q and -q are the same rotation, but the mocap emulator
    # publishes the w >= 0 branch (payload_mocap_emulator._normalize_quat), so anything
    # downstream comparing quaternions componentwise sees one convention, not two.
    return -q if q[0] < 0.0 else q


def _skew(w):
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def betaflight_rates(x, d=70.0, f=670.0, g=0.5):
    """Betaflight rate curve, deg/s, for a stick input in [-1, 1].

    Byte-for-byte the same expression as payload_betaflight_comm._betaflight_rates and
    dynamics.QuadLoadDynamics.betaflight_rates -- so plant, sim inner loop and MPC model
    agree on the actuator mapping. test_sil_plant asserts that agreement.
    """
    x = float(np.clip(x, -1.0, 1.0))
    ax = np.sqrt(x * x + 1e-6)
    sgn = x / ax
    h = ax * (ax ** 5 * g + ax * (1.0 - g))
    return float(sgn * (d * ax + (f - d) * h))


# ── configuration ────────────────────────────────────────────────────────────

@dataclass
class QuadParams:
    """One airframe. Defaults are the sim x3 drone (three_attach.sdf / x3_drone3_magnet.sdf)."""
    mass: float = 0.6
    # a = thrust_c * throttle^2  [m/s^2]. 4 * motorConstant * (u*maxRotVel)^2 / mass
    # = 4 * 0.62e-6 * (u*4631)^2 / 0.6 = 88.6 * u^2. Same number as the launches'
    # number the sim launches' thrust_ratio is derived FROM: the tracker's linear model
    # has to use the secant gain sqrt(thrust_c * a_hover) ~ 31 at the hover operating
    # point. Both come from the SDF motor model, which is not a coincidence.
    thrust_c: float = 88.6
    # Betaflight rate curve (rates_d / rates_f / rates_g in payload_betaflight_comm,
    # centre_rate_deg / max_rate_deg / rate_expo in dynamics.py).
    rates_d: float = 70.0
    rates_f: float = 670.0
    rates_g: float = 0.5
    # First-order body-rate response. 0.12 s is the tracker model's tau_rate. The real
    # gz chain is a rate-P controller into rigid-body inertia; this is a lumped stand-in
    # and it is SHARED with the tracker's model, so a rate-loop mismatch failure is
    # outside what this bench can show (docs/design/sil_bench.md §7).
    rate_tau: float = 0.12
    # Armed idle: payload_betaflight_comm clamps throttle up to 5% whenever armed, so a
    # disarmed-but-armed drone is never at literally zero thrust.
    idle_throttle: float = 0.05
    drag_z: float = 0.0
    # One-way support under the drone (its take-off stand). None = no stand.
    stand_z: float | None = None


@dataclass
class PayloadParams:
    """Defaults are three_attach.sdf's payload link: 0.4 kg, 0.2x0.2x0.05 box."""
    mass: float = 0.4
    inertia: tuple = (1.67e-3, 1.67e-3, 3.33e-3)
    ground_z: float = 0.025


@dataclass
class RodParams:
    """Rigid rod with ball ends. Stiff two-way spring+damper: rods push AND pull, which
    is what the SDF tethers (rod cylinder + ball joints at both ends) actually do."""
    k: float = 3000.0
    c: float = 25.0


@dataclass
class Link:
    """One drone->payload connection. `rho` is the attach point in the PAYLOAD BODY
    frame; `length` its rest length. Tethers use the attach ring and cable_len; a welded
    newcomer uses its captured weld point and the magnet arm length (0.5 m).

    After the 2026-07-28 ball-joint fix the magnet chain is
        payload =weld(fixed)= tip =BALL= arm =BALL= drone,
    i.e. structurally a rod with ball ends -- the same element as a tether, with a
    different anchor and length. That is why no new joint type is needed here.
    """
    rho: np.ndarray
    length: float
    attached: bool = True


# ── the plant ────────────────────────────────────────────────────────────────

class SilPlant:
    """n quadrotors + a rigid payload + rods + ground, integrated with semi-implicit
    Euler at a small fixed step (stable for the stiff rod/ground springs).

    Units are SI throughout; quaternions are [w, x, y, z]; body rates are BODY frame
    (matching what payload_mocap_emulator publishes in twist.angular).
    """

    def __init__(self, quads, links, payload=None, rod=None, gravity=G):
        self.n = len(quads)
        self.quads = list(quads)
        self.links = list(links)
        assert len(self.links) == self.n, "one Link per drone (attached=False if free)"
        self.pay = payload or PayloadParams()
        self.rod = rod or RodParams()
        self.g = float(gravity)
        self.I = np.diag(np.asarray(self.pay.inertia, float))
        self.Iinv = np.linalg.inv(self.I)

        self.p = np.zeros((self.n, 3))
        self.v = np.zeros((self.n, 3))
        self.q = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (self.n, 1))
        self.w = np.zeros((self.n, 3))
        self.u = np.zeros((self.n, 4))          # decoded [roll, pitch, throttle, yaw]
        self.armed = [False] * self.n

        self.xL = np.zeros(3)
        self.vL = np.zeros(3)
        self.RL = np.eye(3)
        self.wL = np.zeros(3)

        # Per-drone accelerations from the last substep, kept so the IMU and the
        # diagnostics can be read out without recomputing anything.
        self._a_thrust = np.zeros((self.n, 3))   # world frame, thrust only
        self._a_cable = np.zeros((self.n, 3))    # world frame, rod reaction only

    # ── setup ────────────────────────────────────────────────────────────────

    def reset(self, drone_pos, load_pos, drone_quat=None, load_R=None):
        self.p = np.array([np.asarray(x, float) for x in drone_pos])
        self.v[:] = 0.0
        self.w[:] = 0.0
        if drone_quat is None:
            self.q = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (self.n, 1))
        else:
            self.q = np.array([np.asarray(x, float) for x in drone_quat])
        self.xL = np.asarray(load_pos, float).copy()
        self.vL[:] = 0.0
        self.RL = np.eye(3) if load_R is None else np.asarray(load_R, float).copy()
        self.wL[:] = 0.0
        self._a_thrust[:] = 0.0
        self._a_cable[:] = 0.0

    def weld(self, i, rho_body, length):
        """Engage drone i's link at `rho_body` (payload frame) with rest `length`.

        Called by the bench's deterministic weld harness at the scripted attach time,
        at the same instant it publishes /magnet/object_attached (design note §4)."""
        self.links[i] = Link(np.asarray(rho_body, float), float(length), True)

    def release(self, i):
        self.links[i].attached = False

    # ── command decoding ─────────────────────────────────────────────────────

    def set_command(self, i, ch0, ch1, ch2, ch3, armed):
        """Decode one ELRSCommand exactly as payload_betaflight_comm does.

        channel_2 is the throttle mapped to [-1, 1] by the tracker
        (`round((thr * 2) - 1, 3)`), so it comes back as (ch2 + 1) / 2. The armed idle
        floor is the sim inner loop's, not ours: it clamps motor speed up to 5% of
        maxRotVelocity whenever `armed` is set.
        """
        qp = self.quads[i]
        thr = (float(ch2) + 1.0) * 0.5
        thr = float(np.clip(thr, 0.0, 1.0))
        if armed:
            thr = max(thr, qp.idle_throttle)
        else:
            thr = 0.0
        self.u[i] = [float(ch0), float(ch1), thr, float(ch3)]
        self.armed[i] = bool(armed)

    # ── integration ──────────────────────────────────────────────────────────

    def step(self, dt):
        """One integration substep of length dt."""
        f_load = np.array([0.0, 0.0, -self.pay.mass * self.g])
        tau_load = np.zeros(3)
        f_drone = np.zeros((self.n, 3))

        for i in range(self.n):
            qp = self.quads[i]
            R = quat_to_rot(self.q[i])

            # ── attitude: betaflight rate command -> first-order body-rate response ──
            u = self.u[i]
            w_cmd = np.radians([
                betaflight_rates(u[0], qp.rates_d, qp.rates_f, qp.rates_g),
                betaflight_rates(u[1], qp.rates_d, qp.rates_f, qp.rates_g),
                betaflight_rates(-u[3], qp.rates_d, qp.rates_f, qp.rates_g),
            ])
            self.w[i] += (w_cmd - self.w[i]) / qp.rate_tau * dt
            self.q[i] = quat_integrate(self.q[i], self.w[i], dt)

            # ── thrust: body +z only, quadratic in throttle ──────────────────
            a_thrust = R @ np.array([0.0, 0.0, qp.thrust_c * u[2] * u[2]])
            self._a_thrust[i] = a_thrust
            f_drone[i] += qp.mass * a_thrust
            f_drone[i] += np.array([0.0, 0.0, -qp.mass * self.g])
            f_drone[i][2] += -qp.mass * qp.drag_z * self.v[i][2]

            # ── rod ──────────────────────────────────────────────────────────
            link = self.links[i]
            if not link.attached:
                self._a_cable[i] = np.zeros(3)
            else:
                r_world = self.RL @ link.rho
                d = self.p[i] - (self.xL + r_world)
                dist = float(np.linalg.norm(d))
                if dist < 1e-9:
                    self._a_cable[i] = np.zeros(3)
                else:
                    n = d / dist
                    v_att = self.vL + np.cross(self.wL, r_world)
                    vrel = self.v[i] - v_att
                    f_rod = -(self.rod.k * (dist - link.length)
                              + self.rod.c * float(vrel @ n)) * n
                    f_drone[i] += f_rod
                    self._a_cable[i] = f_rod / qp.mass
                    f_load += -f_rod
                    tau_load += np.cross(r_world, -f_rod)

            # ── take-off stand: one-way support under the drone ──────────────
            if qp.stand_z is not None:
                pen = qp.stand_z - self.p[i][2]
                if pen > 0.0:
                    fn = 6000.0 * pen
                    if self.v[i][2] < 0.0:
                        fn += -60.0 * self.v[i][2]
                    f_drone[i][2] += fn

        # ground contact under the payload (one-way, central -- no torque)
        pen = self.pay.ground_z - self.xL[2]
        if pen > 0.0:
            fn = 6000.0 * pen
            if self.vL[2] < 0.0:
                fn += -60.0 * self.vL[2]
            f_load[2] += fn

        # semi-implicit Euler
        for i in range(self.n):
            self.v[i] += (f_drone[i] / self.quads[i].mass) * dt
            self.p[i] += self.v[i] * dt
        self.vL += (f_load / self.pay.mass) * dt
        self.xL += self.vL * dt

        w_dot = self.Iinv @ (tau_load - np.cross(self.wL, self.I @ self.wL))
        self.wL += w_dot * dt
        self.RL = (np.eye(3) + _skew(self.wL) * dt) @ self.RL
        # re-orthonormalise (Gram-Schmidt) so the small-angle update does not drift
        x = self.RL[:, 0] / max(np.linalg.norm(self.RL[:, 0]), 1e-12)
        y = self.RL[:, 1] - (x @ self.RL[:, 1]) * x
        y = y / max(np.linalg.norm(y), 1e-12)
        self.RL = np.column_stack([x, y, np.cross(x, y)])

    def advance(self, dt, substeps):
        for _ in range(int(substeps)):
            self.step(dt / substeps)

    # ── readouts ─────────────────────────────────────────────────────────────

    def drone_state(self, i):
        """[p(3), q(4 wxyz), v(3), w_body(3)] -- the 13-vector the mocap wire format
        carries and CallbackManagerMulti unpacks into `current_pose`."""
        return np.concatenate([self.p[i], self.q[i], self.v[i], self.w[i]])

    def payload_state(self):
        return np.concatenate([self.xL, rot_to_quat(self.RL), self.vL,
                               self.RL.T @ self.wL])

    def imu(self, i):
        """Body-frame specific force, the quantity a real IMU reports.

        f = R^T (a_thrust_world + a_cable_world). At rest this is [0, 0, +9.81] and in
        free fall it is zero -- the contract controller_mpc._imu_callback documents, and
        the input to measured_cable_accel() whose magnitude |aCm| is half the bench's
        acceptance criterion."""
        R = quat_to_rot(self.q[i])
        return R.T @ (self._a_thrust[i] + self._a_cable[i])

    def cable_accel(self, i):
        """TRUE world-frame cable acceleration on drone i (rod force / mass). The
        ground truth |aCm| is measured against."""
        return self._a_cable[i].copy()

    def rod_tension(self, i):
        link = self.links[i]
        if not link.attached:
            return 0.0
        d = self.p[i] - (self.xL + self.RL @ link.rho)
        return max(self.rod.k * (float(np.linalg.norm(d)) - link.length), 0.0)

    def cable_elev_deg(self, i):
        link = self.links[i]
        d = self.p[i] - (self.xL + self.RL @ link.rho)
        nd = float(np.linalg.norm(d))
        return float(np.degrees(np.arcsin(np.clip(d[2] / max(nd, 1e-6), -1.0, 1.0))))

    def load_tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(self.RL[2, 2], -1.0, 1.0))))

    def drone_tilt_deg(self, i):
        R = quat_to_rot(self.q[i])
        return float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))

    def is_finite(self):
        return bool(np.all(np.isfinite(self.p)) and np.all(np.isfinite(self.q))
                    and np.all(np.isfinite(self.xL)) and np.all(np.isfinite(self.RL)))
