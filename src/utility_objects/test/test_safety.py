"""Tests for the flight envelope checker (finding F3).

Written to be READ as much as run: each test names the property it protects, because
this is the code that decides whether four drones and a payload stay in the air.

    python3 -m pytest src/utility_objects/test/test_safety.py -v
"""
import math
import pytest

from utility_objects.safety import (
    EnvelopeChecker, EnvelopeLimits, Verdict, tilt_deg_from_quat, OK, WARN, FAULT)


LEVEL = EnvelopeLimits()          # defaults under test


def airborne(checker, **kw):
    """Run a check with the drone flying, defaulting everything to healthy."""
    args = dict(velocity=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0),
                ref_age_s=0.02, airborne=True)
    args.update(kw)
    return checker.check(**args)


def fault_after_debounce(checker, **kw):
    """A fault only latches after `fault_debounce` consecutive bad samples, so a
    single corrupt mocap frame cannot drop the fleet. Drive it past that."""
    v = None
    for _ in range(LEVEL.fault_debounce):
        v = airborne(checker, **kw)
    return v


# ── tilt maths ───────────────────────────────────────────────────────────────

def test_tilt_is_zero_when_level():
    assert tilt_deg_from_quat((1.0, 0.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-9)


def test_tilt_matches_a_known_rotation():
    # 30 deg about x: q = (cos15, sin15, 0, 0)
    a = math.radians(30.0) / 2.0
    assert tilt_deg_from_quat((math.cos(a), math.sin(a), 0.0, 0.0)) == pytest.approx(30.0)


def test_tilt_is_invariant_to_quaternion_sign():
    """Mocap may hand over either q or -q for the same attitude. If tilt flipped
    with the sign, the fleet would abort at random."""
    a = math.radians(50.0) / 2.0
    q = (math.cos(a), math.sin(a), 0.0, 0.0)
    assert tilt_deg_from_quat(q) == pytest.approx(tilt_deg_from_quat(tuple(-c for c in q)))


def test_tilt_of_degenerate_quaternion_is_nan_not_a_crash():
    assert math.isnan(tilt_deg_from_quat((0.0, 0.0, 0.0, 0.0)))


# ── healthy flight ───────────────────────────────────────────────────────────

def test_healthy_flight_is_ok():
    assert airborne(EnvelopeChecker()).level == OK


def test_missing_signals_are_not_faults():
    """Signals arrive at different times during startup. A checker that faulted on
    absent data would make every launch an emergency."""
    c = EnvelopeChecker()
    v = c.check(velocity=None, quat=None, ref_age_s=None, airborne=True)
    assert v.level == OK


# ── attitude ─────────────────────────────────────────────────────────────────

def test_payload_tilt_beyond_limit_faults():
    a = math.radians(75.0) / 2.0
    v = fault_after_debounce(EnvelopeChecker(),
                             payload_quat=(math.cos(a), math.sin(a), 0.0, 0.0))
    assert v.is_fault and v.detail['who'] == 'payload'


def test_attach_experiment_tilt_is_allowed():
    """Ring-attach legitimately reaches 10-30 deg payload tilt. The limit is 60 deg
    precisely so these experiments are not aborted."""
    a = math.radians(30.0) / 2.0
    c = EnvelopeChecker()
    for _ in range(10):
        v = airborne(c, payload_quat=(math.cos(a), math.sin(a), 0.0, 0.0))
    assert not v.is_fault


# ── speed and reference staleness ────────────────────────────────────────────

def test_excessive_speed_faults():
    v = fault_after_debounce(EnvelopeChecker(), velocity=(4.0, 0.0, 0.0))
    assert v.is_fault and v.detail['check'] == 'speed'


def test_stale_reference_faults():
    """If the planner stops publishing, a tracker will happily fly a stale
    reference forever."""
    v = fault_after_debounce(EnvelopeChecker(), ref_age_s=2.0)
    assert v.is_fault and v.detail['check'] == 'ref_stale'


# ── battery is warn-only, by decision ────────────────────────────────────────

def test_low_battery_warns_but_never_faults():
    """Voltage sags under thrust transients; a fault here would fire mid-manoeuvre.
    The operator is standing next to the cage and makes the call."""
    c = EnvelopeChecker()
    for _ in range(50):
        v = airborne(c, battery_v=13.0)
        assert not v.is_fault
    assert v.level == WARN


# ── ground behaviour and latching ────────────────────────────────────────────

def test_nothing_faults_while_on_the_ground():
    """A drone on its stand may sit tilted and has no reference yet."""
    c = EnvelopeChecker()
    for _ in range(10):
        v = c.check(quat=(0.9, 0.4, 0.0, 0.0),
                    ref_age_s=99.0, airborne=False)
    assert not v.is_fault


def test_a_fault_latches_until_reset():
    """The condition going away must not silently re-arm the fleet mid-flight."""
    c = EnvelopeChecker()
    fault_after_debounce(c, velocity=(9.0, 0.0, 0.0))
    assert airborne(c).is_fault              # healthy sample, still faulted
    c.reset()
    assert airborne(c).level == OK


def test_reset_is_what_re_arming_uses():
    c = EnvelopeChecker()
    fault_after_debounce(c, velocity=(9.0, 0.0, 0.0))
    c.reset()
    assert c.check(velocity=(0.0, 0.0, 0.0), airborne=True).level == OK


# ── pose-layout contract ─────────────────────────────────────────────────────

def test_current_pose_layout_contract():
    """controller_mpc slices `current_pose` as [3:7] quaternion (w,x,y,z) and
    [7:10] linear velocity, matching what CallbackManagerMulti.pose_callback assembles
    ([0:3] position, [10:13] angular velocity). If that layout ever changes, the speed
    check would silently start reading angular velocity. This test fails instead.
    """
    # Exactly the vector pose_callback builds, with distinguishable values.
    a = math.radians(70.0) / 2.0          # 70 deg tilt -> must fault at 60
    current_pose = [
        1.0, 2.0, 1.0,                    # position (not consumed: no geofence)
        math.cos(a), math.sin(a), 0.0, 0.0,   # quaternion (w, x, y, z)
        0.1, 0.2, 0.3,                    # linear velocity
        9.0, 9.0, 9.0,                    # angular velocity -- must NOT be read as speed
    ]
    c = EnvelopeChecker()
    v = None
    for _ in range(LEVEL.fault_debounce):
        v = c.check(quat=current_pose[3:7],
                    velocity=current_pose[7:10],
                    airborne=True)

    # Tilt is read from [3:7] and trips; angular velocity at [10:13] is huge but is
    # NOT the speed signal, so the fault must be tilt, not speed.
    assert v.is_fault
    assert v.detail['check'] == 'tilt', (
        f"expected the tilt check to fire, got {v.detail} -- pose slicing is wrong")
    assert v.detail['value'] == pytest.approx(70.0, abs=0.5)
