"""
Unit tests for the SIL bench plant (tools/sil/plant.py).

Each test names the property it protects in plain language, per THESIS_PLAN §8.2.
These are the tests that stand between "the bench printed a number" and "the number
means something": if the plant's thrust, actuator mapping, tilt coupling or IMU
convention is wrong, every scenario result is wrong in a way no scenario would reveal.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sil.plant import (G, Link, PayloadParams, QuadParams, SilPlant,  # noqa: E402
                       betaflight_rates, quat_to_rot, rot_to_quat)


def _free_drone(**kw):
    """One drone, no payload connection, starting level at the origin."""
    qp = QuadParams(**kw)
    plant = SilPlant([qp], [Link(np.zeros(3), 0.5, attached=False)])
    plant.reset([[0.0, 0.0, 1.0]], [0.0, 0.0, -10.0])   # payload far below, unlinked
    return plant, qp


def _hover_throttle(qp):
    """Throttle that exactly cancels gravity for this airframe: c*u^2 = g."""
    return float(np.sqrt(G / qp.thrust_c))


def _channel_2(throttle):
    """The tracker publishes channel_2 = throttle*2 - 1."""
    return throttle * 2.0 - 1.0


# ── 1. thrust/gravity balance ────────────────────────────────────────────────

def test_level_drone_at_hover_throttle_holds_altitude():
    """A level drone commanded exactly hover throttle neither climbs nor sinks.

    If this fails the thrust coefficient or the mass is wrong, and every altitude in
    every scenario is wrong with it."""
    plant, qp = _free_drone()
    u = _hover_throttle(qp)
    plant.set_command(0, 0.0, 0.0, _channel_2(u), 0.0, armed=True)
    for _ in range(5000):
        plant.step(1e-3)
    assert abs(plant.p[0][2] - 1.0) < 1e-3
    assert abs(plant.v[0][2]) < 1e-3


def test_thrust_is_quadratic_in_throttle():
    """Doubling throttle quadruples thrust acceleration -- the plant is a = c*u^2, not
    the linear a = kT*u the tracker's model assumes. That mismatch is deliberate: it is
    why _scheduled_kT and the kT estimator exist."""
    plant, qp = _free_drone()
    plant.set_command(0, 0.0, 0.0, _channel_2(0.2), 0.0, armed=True)
    plant.step(1e-6)
    a1 = plant._a_thrust[0][2]
    plant.set_command(0, 0.0, 0.0, _channel_2(0.4), 0.0, armed=True)
    plant.step(1e-6)
    a2 = plant._a_thrust[0][2]
    assert a2 == pytest.approx(4.0 * a1, rel=1e-6)
    assert a1 == pytest.approx(qp.thrust_c * 0.04, rel=1e-6)


# ── 2. actuator mapping agrees with the sim inner loop AND the MPC model ─────

def test_rate_curve_matches_the_sim_inner_loop_and_the_mpc_model():
    """The plant, payload_betaflight_comm and dynamics.QuadLoadDynamics must agree on
    what a stick deflection means, or the bench flies a different aircraft than Gazebo
    and than the model the MPC optimises against."""
    try:
        import casadi as cs
        from controller_quad_load.dynamics import QuadLoadDynamics
    except Exception as exc:                       # pragma: no cover - env dependent
        pytest.skip(f"MPC model not importable here: {exc}")

    dyn = QuadLoadDynamics()
    x = cs.MX.sym('x', 1)
    f = cs.Function('bf', [x, dyn.centre_rate_deg, dyn.max_rate_deg, dyn.rate_expo],
                    [dyn.betaflight_rates(x)])
    for stick in (-1.0, -0.6, -0.15, 0.0, 0.15, 0.6, 1.0):
        mine = betaflight_rates(stick)
        theirs = float(f(stick, 70.0, 670.0, 0.5))
        assert mine == pytest.approx(theirs, abs=1e-9), f"stick={stick}"


def test_armed_idle_floor_matches_the_sim_inner_loop():
    """payload_betaflight_comm clamps throttle up to 5% whenever armed, so an armed
    drone commanded zero throttle is not at zero thrust. A bench without the floor would
    let an armed-idle drone free-fall where the sim holds it slightly."""
    plant, _ = _free_drone()
    plant.set_command(0, 0.0, 0.0, -1.0, 0.0, armed=True)
    assert plant.u[0][2] == pytest.approx(0.05)
    plant.set_command(0, 0.0, 0.0, -1.0, 0.0, armed=False)
    assert plant.u[0][2] == pytest.approx(0.0)


# ── 3. the tilt coupling that mini_plant does not have ──────────────────────

def test_a_tilted_drone_at_hover_throttle_sinks():
    """THE property the whole bench exists for. Thrust acts along body z, so a drone
    that tilts to push sideways loses vertical thrust and descends unless it adds
    throttle. mini_plant's point-mass drone pushes sideways for free and therefore
    cannot reproduce the attach runaway (docs/design/sil_bench.md §2)."""
    plant, qp = _free_drone()
    tilt = np.radians(30.0)
    # roll by `tilt` about world x: q = [cos(t/2), sin(t/2), 0, 0]
    plant.q[0] = np.array([np.cos(tilt / 2), np.sin(tilt / 2), 0.0, 0.0])
    plant.set_command(0, 0.0, 0.0, _channel_2(_hover_throttle(qp)), 0.0, armed=True)
    # freeze the attitude loop so this isolates the thrust projection
    plant.quads[0].rate_tau = 1e9
    for _ in range(1000):
        plant.step(1e-3)
    assert plant.v[0][2] < -0.5, "a 30 deg tilted drone at hover throttle must sink"
    # Rolling by t about +x points body z at world [0, -sin t, cos t], so after 1 s the
    # sideways speed is exactly the horizontal component of an unchanged hover thrust.
    assert plant.v[0][1] == pytest.approx(-G * np.sin(tilt), rel=0.02)
    assert plant.v[0][2] == pytest.approx(G * (np.cos(tilt) - 1.0), rel=0.02)


def test_attitude_lags_the_command():
    """The rate loop is first order with tau, so attitude cannot change instantly. A
    drone being dragged cannot re-point its thrust immediately -- the lag is part of the
    runaway mechanism."""
    plant, qp = _free_drone()
    plant.set_command(0, 0.5, 0.0, _channel_2(0.3), 0.0, armed=True)
    w_cmd = np.radians(betaflight_rates(0.5))
    plant.step(qp.rate_tau)                        # exactly one time constant
    assert plant.w[0][0] == pytest.approx(w_cmd, rel=0.02), \
        "one full tau of a single Euler step reaches the command; check rate_tau usage"
    plant2, qp2 = _free_drone()
    plant2.set_command(0, 0.5, 0.0, _channel_2(0.3), 0.0, armed=True)
    for _ in range(int(0.5 * qp2.rate_tau / 1e-3)):
        plant2.step(1e-3)
    assert plant2.w[0][0] < 0.7 * w_cmd, "rate must still be short of the command"


# ── 4. IMU convention ────────────────────────────────────────────────────────

def test_imu_reads_one_g_up_at_rest_and_zero_in_free_fall():
    """controller_mpc._imu_callback documents: at rest [0,0,+9.81]; in flight
    (thrust + cable)/m in the body frame. measured_cable_accel() subtracts the modelled
    thrust from this, and |aCm| is half the bench's acceptance criterion -- a sign or
    frame error here would silently invent or hide the runaway signature."""
    plant, qp = _free_drone()
    plant.set_command(0, 0.0, 0.0, _channel_2(_hover_throttle(qp)), 0.0, armed=True)
    plant.step(1e-3)
    assert plant.imu(0) == pytest.approx([0.0, 0.0, G], abs=1e-6)

    plant.set_command(0, 0.0, 0.0, -1.0, 0.0, armed=False)
    plant.step(1e-3)
    assert plant.imu(0) == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)


def test_imu_is_expressed_in_the_body_frame():
    """A tilted drone's IMU still reads its thrust along its OWN z, not world z."""
    plant, qp = _free_drone()
    tilt = np.radians(20.0)
    plant.q[0] = np.array([np.cos(tilt / 2), np.sin(tilt / 2), 0.0, 0.0])
    plant.quads[0].rate_tau = 1e9
    plant.set_command(0, 0.0, 0.0, _channel_2(_hover_throttle(qp)), 0.0, armed=True)
    plant.step(1e-3)
    assert plant.imu(0) == pytest.approx([0.0, 0.0, G], abs=1e-6)


# ── 5. static rod tension ────────────────────────────────────────────────────

def test_three_taut_rods_each_carry_a_third_of_the_payload_weight():
    """Static load sharing. If this is wrong the tension-share metric and every
    reconfiguration claim built on it are wrong."""
    n, cable_len, radius, attach_z = 3, 0.5, 0.08, 0.025
    pay = PayloadParams()
    quads, links, pos = [], [], []
    load = np.array([0.0, 0.0, 0.6])
    elev = np.radians(45.0)
    for k in range(n):
        th = 2 * np.pi * k / n
        rho = np.array([radius * np.cos(th), radius * np.sin(th), attach_z])
        d = np.array([np.cos(th) * np.cos(elev), np.sin(th) * np.cos(elev), np.sin(elev)])
        quads.append(QuadParams())
        links.append(Link(rho, cable_len))
        pos.append(load + rho + cable_len * d)
    plant = SilPlant(quads, links, pay)
    plant.reset(pos, load)
    # Hold every drone at its start pose and let the payload settle: the rods start at
    # exactly rest length (zero tension), so the load sags the ~0.6 mm that generates
    # the static tension. 2 s is many settling times at zeta ~= 0.44.
    for _ in range(20000):
        plant.p = np.array(pos)
        plant.v[:] = 0.0
        plant.step(1e-4)
    total = sum(plant.rod_tension(i) * np.sin(elev) for i in range(n))
    assert total == pytest.approx(pay.mass * G, rel=0.02), \
        "vertical components of the three rod tensions must carry the payload weight"
    tens = [plant.rod_tension(i) for i in range(n)]
    assert max(tens) - min(tens) < 0.02 * np.mean(tens), "symmetric config, equal share"


# ── 6. integrator sanity ─────────────────────────────────────────────────────

def test_undriven_payload_energy_does_not_grow():
    """A hanging payload with no thrust and no damping must not gain energy over
    10 s of 1 ms steps. Semi-implicit Euler on a stiff rod spring is stable but not
    unconditionally so; this pins the step size that is actually safe."""
    n, cable_len, radius = 3, 0.5, 0.08
    pay = PayloadParams(ground_z=-100.0)           # no ground contact in this test
    quads, links, pos = [], [], []
    load = np.array([0.0, 0.0, 0.6])
    elev = np.radians(45.0)
    for k in range(n):
        th = 2 * np.pi * k / n
        rho = np.array([radius * np.cos(th), radius * np.sin(th), 0.025])
        d = np.array([np.cos(th) * np.cos(elev), np.sin(th) * np.cos(elev), np.sin(elev)])
        quads.append(QuadParams())
        links.append(Link(rho, cable_len))
        pos.append(load + rho + cable_len * d)
    plant = SilPlant(quads, links, pay, rod=None)
    # start the load 3 cm low so the rods are stretched and there is energy to conserve
    plant.reset(pos, load - np.array([0.0, 0.0, 0.03]))

    def energy():
        ke = 0.5 * pay.mass * float(plant.vL @ plant.vL)
        ke += 0.5 * float(plant.wL @ (plant.I @ plant.wL))
        pe = pay.mass * G * float(plant.xL[2])
        for i in range(n):
            d = plant.p[i] - (plant.xL + plant.RL @ plant.links[i].rho)
            pe += 0.5 * plant.rod.k * (float(np.linalg.norm(d)) - cable_len) ** 2
        return ke + pe

    e0 = energy()
    for _ in range(10000):
        plant.p = np.array(pos)        # anchors held: the payload is the only free body
        plant.v[:] = 0.0
        plant.step(1e-3)
    assert plant.is_finite()
    assert energy() < e0 + 0.05 * abs(e0) + 0.05, "energy must not grow (damped rods)"


# ── quaternion round trip ────────────────────────────────────────────────────

def test_quaternion_and_rotation_round_trip():
    """rot_to_quat/quat_to_rot are used for every published payload attitude; a sign
    error here presents as an inexplicable tilt reading."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        if q[0] < 0:
            q = -q
        R = quat_to_rot(q)
        assert rot_to_quat(R) == pytest.approx(q, abs=1e-9)
        assert R.T @ R == pytest.approx(np.eye(3), abs=1e-9)
