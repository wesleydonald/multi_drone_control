"""
Tests for the phase-1 creep takeoff (creep_controller.py).

These exist because of a DEADLOCK measured in Gazebo on 2026-08-05. Across 8 identical
runs of the 45 deg attach config (R0026-R0033) the fleet never left the ground in 2, and
only escaped after ~40 s in 3 more. The cause was not the tracker -- it followed its
reference faithfully every time -- it was that the reference never moved:

  * `arc_theta` (the swept elevation reference) only advances once `lifted_off` is set;
  * `lifted_off` needs a drone more than LIFTOFF_MARGIN = 0.05 m above its spawn z;
  * on a rigid 0.5 m rod pivoting about a grounded attach point, +0.05 m of z means
    reaching ~11.5 deg of elevation;
  * but the reference held BEFORE liftoff is a pure vertical lead (CREEP_LEAD), which is
    precisely what `_arc_creep`'s own docstring says "cannot rotate the rod at all".

The drones topped out at 9-10 deg, never crossed the margin, and the phase had no
timeout -- so `ref_done` stayed false, the handover timeout below it never even armed,
and the fleet sat at its spawn angle until the run was killed.

Run:  python3 -m pytest src/mpc_planner/test/test_creep_controller.py -v
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))), 'src', 'mpc_planner'))

from mpc_planner.creep_controller import (      # noqa: E402
    CreepController, LIFTOFF_MARGIN, LIFTOFF_TIMEOUT_S)


class _Log:
    def __init__(self):
        self.warns, self.infos = [], []

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        self.infos.append(m)


HZ = 10.0
CABLE = 0.5
SPAWN_ELEV = np.deg2rad(5.7)          # the three_attach.sdf ground start


def build(n=3, elev_deg=45.0):
    """A rigid-rod ground start: payload on the floor, drones out at 5.7 deg."""
    rho = [np.array([0.08 * np.cos(a), 0.08 * np.sin(a), 0.025])
           for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    state = {'pos': None}

    def drone_at(i):
        return state['pos'][i]

    log = _Log()
    c = CreepController(n=n, rho=rho, cable_len=CABLE, N=5, dt=0.1, g=9.81,
                        handover_elev_deg=elev_deg, hz=HZ,
                        drone_at=drone_at, publish_ref=lambda i, nodes: None,
                        logger=log)
    return c, state, log, rho


def load_state():
    return np.array([0.0, 0.0, 0.025, 1.0, 0.0, 0.0, 0.0])


def positions(rho, rise=0.0, n=3):
    """Drones on the rod at the spawn elevation, optionally raised by `rise` m of pure
    z -- which is what the rigid rod resists, so `rise` stays small in reality."""
    ls = load_state()
    out = []
    for i in range(n):
        attach = ls[0:3] + rho[i]
        horiz = np.array([attach[0], attach[1], 0.0])
        hn = np.linalg.norm(horiz)
        radial = horiz / hn if hn > 1e-6 else np.array([1.0, 0.0, 0.0])
        p = attach + CABLE * (np.cos(SPAWN_ELEV) * radial
                              + np.array([0.0, 0.0, np.sin(SPAWN_ELEV)]))
        out.append(p + np.array([0.0, 0.0, rise]))
    return out


def run(c, state, rho, seconds, rise, takeoff_seen=True):
    """Step the controller for `seconds`, with the drones sitting `rise` m above spawn.

    The FIRST tick is always at rise=0: `_arc_creep` latches arc_anchor/arc_theta0 from
    whatever pose it first sees, so feeding it an already-raised pose would define that
    raised height AS the spawn height and no rise could ever be detected. Real runs latch
    while the drones are parked, which is what this reproduces.
    """
    gates = [(0.0, 0.0)] * 3
    handover = False
    state['pos'] = positions(rho, rise=0.0)
    h, _r = c.step(load_state(), state['pos'], gates, takeoff_seen)
    handover = handover or h
    for _ in range(int(seconds * HZ)):
        state['pos'] = positions(rho, rise=rise)
        h, _r = c.step(load_state(), state['pos'], gates, takeoff_seen)
        handover = handover or h
    return handover


def test_a_drone_that_clears_the_margin_starts_the_sweep_immediately():
    """The normal path must keep working: a real liftoff starts the arc sweep at once,
    with no timeout involved."""
    c, state, log, rho = build()
    run(c, state, rho, seconds=1.0, rise=LIFTOFF_MARGIN + 0.02)
    assert c.lifted_off is True
    assert c.arc_theta > SPAWN_ELEV, 'sweep did not advance after a genuine liftoff'
    assert not log.warns, f'unexpected warning on the normal path: {log.warns}'


def test_the_rigid_rod_deadlock_does_not_strand_the_fleet():
    """THE REGRESSION. Drones stuck just under the margin -- the measured Gazebo case,
    9-10 deg of elevation against a 11.5 deg requirement -- must still get their sweep
    started, via the timeout, rather than sitting at the spawn angle forever."""
    c, state, log, rho = build()
    stuck = LIFTOFF_MARGIN - 0.02                  # ~0.03 m, what was measured
    run(c, state, rho, seconds=LIFTOFF_TIMEOUT_S + 1.0, rise=stuck)
    assert c.lifted_off is True, 'liftoff gate deadlocked: the fleet is stranded'
    assert c.arc_theta > SPAWN_ELEV, 'sweep still not advancing after the timeout'
    assert any('liftoff gate timed out' in w for w in log.warns), (
        'the fleet was released without saying so — this must be loud, because a run '
        f'that needed the timeout is not a clean takeoff. warns={log.warns}')


def test_the_timeout_does_not_fire_before_takeoff_is_commanded():
    """The planner runs from first mocap, while the drones are still disarmed on the
    ground. Not rising is CORRECT then, and sweeping the reference away from parked
    drones would yank them the instant they arm."""
    c, state, log, rho = build()
    run(c, state, rho, seconds=LIFTOFF_TIMEOUT_S + 3.0, rise=0.0, takeoff_seen=False)
    assert c.lifted_off is False, 'sweep started before TAKEOFF was commanded'
    assert c.arc_theta == pytest.approx(SPAWN_ELEV, abs=1e-6)
    assert not log.warns


def test_the_sweep_reaches_the_target_and_then_hands_over():
    """End to end: once sweeping, the reference must actually arrive at the target
    elevation, since `ref_done` is what arms the handover check at all."""
    c, state, log, rho = build(elev_deg=45.0)
    handover = run(c, state, rho, seconds=40.0, rise=LIFTOFF_MARGIN + 0.02)
    assert c.arc_theta == pytest.approx(np.deg2rad(45.0), abs=1e-6), (
        f'sweep stopped at {np.degrees(c.arc_theta):.1f} deg')
    assert handover, 'sweep completed but the phase never handed over'


def _at_elev(rho, elevs_deg):
    ls = load_state()
    out = []
    for i, e in enumerate(elevs_deg):
        attach = ls[0:3] + rho[i]
        radial = np.array([attach[0], attach[1], 0.0])
        radial /= np.linalg.norm(radial)
        a = np.deg2rad(e)
        out.append(attach + CABLE * (np.cos(a) * radial + np.array([0.0, 0.0, np.sin(a)])))
    return out


def test_airborne_start_holds_live_until_takeoff_then_sweeps_down_to_target():
    """M2 hand-over (T0007): drones already flying near-vertical on the grounded ring.
    Before TAKEOFF the reference is the live pose (no anchor latched); after it each rod
    sweeps from its own elevation to the target, from above."""
    refs = {}
    rho = [np.array([0.25 * np.cos(a), 0.25 * np.sin(a), 0.0])
           for a in np.linspace(0, 2 * np.pi, 3, endpoint=False)]
    state = {'pos': None}
    c = CreepController(n=3, rho=rho, cable_len=CABLE, N=5, dt=0.1, g=9.81,
                        handover_elev_deg=45.0, hz=HZ, drone_at=lambda i: state['pos'][i],
                        publish_ref=lambda i, nodes: refs.__setitem__(i, nodes),
                        logger=_Log())
    c.airborne_start = True
    state['pos'] = _at_elev(rho, [80.0, 70.0, 75.0])
    for _ in range(20):                               # his controller moves them
        state['pos'] = [p + np.array([0.01, 0.0, 0.0]) for p in state['pos']]
        assert c.step(load_state(), state['pos'], [(1.0, 0.0)] * 3, False) == (False, None)
    assert c.arc_anchor is None
    assert np.allclose(refs[0][0][0], state['pos'][0])
    assert np.allclose(refs[0][-1][1], 0.0)

    state['pos'] = _at_elev(rho, [80.0, 70.0, 75.0])
    c.step(load_state(), state['pos'], [(1.0, 0.0)] * 3, True)
    th = [np.degrees(t) for t in c._arc_th]
    assert 78.5 < th[0] < 80.0 and 68.5 < th[1] < 70.0     # moved down, from its own
    for _ in range(200):
        handover, _ = c.step(load_state(), _at_elev(rho, [45.0] * 3),
                             [(1.0, 0.0)] * 3, True)
        if handover:
            break
    assert handover and c._arc_done
    p = refs[1][-1][0]
    attach = load_state()[0:3] + rho[1]
    assert abs(np.degrees(np.arcsin((p - attach)[2] / CABLE)) - 45.0) < 1e-6


def test_airborne_start_off_keeps_the_ground_sweep():
    c, state, log, rho = build()
    assert c.airborne_start is False


# Q9 (floor-start M1): a drone welded on at the start joins the creep as a 4th carrier.
def _four_on_plates(rods, elevs_deg, azs_deg=(150.0, 270.0, 30.0, 90.0), r=0.25):
    rho = [np.array([r * np.cos(np.deg2rad(a)), r * np.sin(np.deg2rad(a)), 0.025])
           for a in azs_deg]
    ls = load_state()
    pos = []
    for i, e in enumerate(elevs_deg):
        attach = ls[0:3] + rho[i]
        radial = np.array([attach[0], attach[1], 0.0]) / np.hypot(attach[0], attach[1])
        a = np.deg2rad(e)
        pos.append(attach + rods[i] * (np.cos(a) * radial + np.array([0.0, 0.0, np.sin(a)])))
    return rho, pos


def test_resize_to_four_relatches_every_drone_with_its_own_rod_and_rest_elevation():
    rods = [CABLE, CABLE, CABLE, 0.49]
    rho4, rest = _four_on_plates(rods, [7.5, 7.6, 7.5, 9.0])
    state = {'pos': rest[:3]}
    refs = {}
    c = CreepController(n=3, rho=rho4[:3], cable_len=CABLE, N=5, dt=0.1, g=9.81,
                        handover_elev_deg=45.0, hz=HZ, drone_at=lambda i: state['pos'][i],
                        publish_ref=lambda i, nodes: refs.__setitem__(i, nodes), logger=_Log())
    c.step(load_state(), state['pos'], [(0.0, 0.0)] * 3, False)
    assert c.arc_anchor is not None and len(c.arc_anchor) == 3

    assert c.resize(4, rho4, rods)
    assert c.n == 4 and c.arc_anchor is None
    state['pos'] = rest
    c.step(load_state(), state['pos'], [(0.0, 0.0)] * 4, False)
    assert len(c.arc_anchor) == 4
    assert np.degrees(c.arc_theta0[3]) == pytest.approx(9.0, abs=1e-6)
    assert np.degrees(c.arc_theta) == pytest.approx(np.mean([7.5, 7.6, 7.5, 9.0]), abs=1e-6)
    # drone 3's reference lies on ITS rod's arc about ITS plate (plus the grounded lead)
    p3 = refs[3][0][0] - np.array([0.0, 0.0, 0.10])
    assert np.linalg.norm(p3 - c.arc_anchor[3][0]) == pytest.approx(0.49, abs=1e-9)


def test_four_carriers_sweep_to_target_and_the_handover_waits_for_the_fourth():
    rods = [CABLE, CABLE, CABLE, 0.49]
    rho4, rest = _four_on_plates(rods, [7.5] * 4)
    state = {'pos': rest}
    refs = {}
    c = CreepController(n=3, rho=rho4[:3], cable_len=CABLE, N=5, dt=0.1, g=9.81,
                        handover_elev_deg=45.0, hz=HZ, drone_at=lambda i: state['pos'][i],
                        publish_ref=lambda i, nodes: refs.__setitem__(i, nodes), logger=_Log())
    assert c.resize(4, rho4, rods)
    c.step(load_state(), rest, [(0.0, 0.0)] * 4, True)
    _, up = _four_on_plates(rods, [7.5 + 5.0] * 4)          # a real liftoff
    state['pos'] = up
    for _ in range(int(40 * HZ)):
        c.step(load_state(), up, [(0.0, 0.0)] * 4, True)
    assert c.arc_theta == pytest.approx(np.deg2rad(45.0), abs=1e-6)
    for i in range(4):
        p = refs[i][-1][0]
        attach = c.arc_anchor[i][0]
        assert np.degrees(np.arcsin((p - attach)[2] / rods[i])) == pytest.approx(45.0, abs=1e-6)

    # three at 45, the fourth lagging at 25: no hold accumulates, no hand-over
    _, lag = _four_on_plates(rods, [45.0, 45.0, 45.0, 25.0])
    state['pos'] = lag
    c._arc_wait = 0.0
    for _ in range(int(2 * HZ)):
        h, _ = c.step(load_state(), lag, [(1.0, 0.0)] * 4, True)
        assert not h and c._arc_hold == 0.0
    _, at = _four_on_plates(rods, [45.0] * 4)
    state['pos'] = at
    c._arc_wait = 0.0
    handover = False
    for _ in range(int(2 * HZ)):
        handover, reason = c.step(load_state(), at, [(1.0, 0.0)] * 4, True)
        if handover:
            break
    assert handover and 'reached' in reason


def test_resize_is_refused_once_the_sweep_has_started():
    c, state, log, rho = build()
    run(c, state, rho, seconds=1.0, rise=LIFTOFF_MARGIN + 0.02)
    assert c.lifted_off
    assert not c.resize(4, rho + [rho[0]], [CABLE] * 4)
    assert c.n == 3


def test_three_drone_creep_is_unchanged_without_a_resize():
    c, state, log, rho = build()
    assert c.cable_lens is None and all(c._rod(i) == CABLE for i in range(3))
