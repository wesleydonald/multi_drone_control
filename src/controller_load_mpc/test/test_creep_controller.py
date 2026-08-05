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

Run:  python3 -m pytest src/controller_load_mpc/test/test_creep_controller.py -v
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))), 'src', 'controller_load_mpc'))

from controller_load_mpc.creep_controller import (      # noqa: E402
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
