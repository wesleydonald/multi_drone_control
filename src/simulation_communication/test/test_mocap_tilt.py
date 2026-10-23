"""The runner's MOCAP_TILT in the mocap emulator (card 2026-10-11_quad4_tilt_cut): one step (a mis-solve, which the
tilt cut's glitch guard holds off) or a ramp (a body tipping, which it counts)."""
from types import SimpleNamespace

import numpy as np

from simulation_communication.payload_mocap_emulator import PayloadMocapEmulator as E
from utility_objects.tilt_cut import TiltCutRule


def _deg(q):
    return np.degrees(2.0 * np.arctan2(q[0], q[3]))


def test_a_step_is_whole_at_once_a_ramp_is_linear_then_held():
    step = SimpleNamespace(_tilt=(10.0, 90.0, 0.0))
    assert np.isclose(_deg(E._tilt_q(step, 10.0)), 90.0)
    ramp = SimpleNamespace(_tilt=(10.0, 90.0, 0.3))
    assert np.isclose(_deg(E._tilt_q(ramp, 10.1)), 30.0)
    assert np.isclose(_deg(E._tilt_q(ramp, 10.2)), 60.0)
    assert np.isclose(_deg(E._tilt_q(ramp, 11.0)), 90.0)


def test_at_120_hz_the_ramp_is_counted_and_the_step_is_held_off():
    def first_fault(ramp):
        rule = TiltCutRule()
        rule.takeoff()
        rule.step(0.0, (1.0, 0.0, 0.0, 0.0))     # level until the injection
        ns = SimpleNamespace(_tilt=(0.0, 90.0, ramp))
        for k in range(1, 240):
            t = k / 120.0
            x, y, z, w = E._tilt_q(ns, t)
            if rule.step(t, (w, x, y, z)):
                return t
        return None
    assert 0.2 < first_fault(0.3) < 0.25          # crosses 60 deg at 0.2 s, three poses later
    assert first_fault(0.0) > 0.5                 # the one-pose step: held off 0.5 s, then counted
