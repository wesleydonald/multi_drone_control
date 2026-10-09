"""The measured-tip correction of the approach (card 2026-10-09_approach_tip_feedback rev 3): z only, in a
still hold only, bounded, frozen on contact, in the deadband and on an invalid tip."""
import numpy as np

from dissipative_planner.approach import (
    ApproachProfile, TIP_C_MAX, TIP_C_MIN, TIP_STANDOFF_M, TipLoop, tip_valid)

DT = 0.1
TARGET = np.array([0.0, 0.25, 1.05])
TRUE_ARM = 0.59


def _fly(assumed_arm, seconds=60.0, apply=True, tip_dz=0.0, freeze=None, contact_z=None, moving=False):
    """Approach from above with a perfect body tracker; the tip hangs TRUE_ARM below the body.
    freeze: (t0, t1) when the tip reading is stale; contact_z: the tip cannot go below this."""
    loop = TipLoop(DT, apply=apply)
    prof = ApproachProfile(TARGET + [0.0, 0.0, assumed_arm + 0.3], assumed_arm, DT,
                           tip_loop=loop, tip_dz=tip_dz)
    prof.phase = 'descend'
    out = []
    for k in range(int(seconds / DT)):
        t = k * DT
        tgt = TARGET + ([0.1 * t, 0.0, 0.0] if moving else [0.0, 0.0, 0.0])
        tip_z = float(prof.p_ref[2]) - TRUE_ARM
        if contact_z is not None:
            tip_z = max(tip_z, contact_z)
        ok = not (freeze and freeze[0] <= t < freeze[1])
        prof.step(tgt, None, tip_z=tip_z, tip_ok=ok)
        out.append((t, tip_z, loop.c, loop.hold, loop.reason))
    return out, loop


def _setpoint(tip_dz=0.0):
    return float(TARGET[2]) + TIP_STANDOFF_M + tip_dz


def test_both_signs_of_a_wrong_rod_converge_within_a_centimetre():
    for assumed in (0.55, 0.63):              # E- (tip ends 4 cm low), E+ (4 cm high)
        out, loop = _fly(assumed)
        assert abs(out[-1][1] - _setpoint()) <= 0.01
        assert TIP_C_MIN < loop.c < TIP_C_MAX


def test_no_error_no_correction():
    out, loop = _fly(TRUE_ARM)
    assert abs(loop.c) < 0.005 and abs(out[-1][1] - _setpoint()) < 0.005


def test_switch_off_holds_and_logs_but_never_corrects():
    out, loop = _fly(0.63, apply=False)
    assert loop.c == 0.0 and any(h for _, _, _, h, _ in out)
    assert abs(out[-1][1] - _setpoint() - 0.04) < 0.005    # the 4 cm error stays


def test_no_windup_on_the_planned_descent():
    out, _ = _fly(0.63)
    for t, tip_z, c, hold, _ in out:
        if not hold:
            assert c == 0.0                                    # frozen until the still hold


def test_contact_freezes_and_latches():
    # E+ wants 4 cm down, but a plate stops the tip 1.5 cm above the setpoint: c reaches the plate at -0.025
    out, loop = _fly(0.63, contact_z=_setpoint() + 0.015)
    assert loop.contact
    assert -0.025 - loop.c <= 0.025                           # pushed at most 2.5 cm past contact
    tail = [c for t, _, c, _, _ in out if t > 45.0]
    assert max(tail) - min(tail) < 1e-9                       # latched


def test_an_invalid_tip_freezes_c_and_does_not_reset_it():
    out, loop = _fly(0.63, freeze=(25.0, 27.0))
    seg = [c for t, _, c, _, _ in out if 25.0 <= t < 27.0]
    assert max(seg) - min(seg) < 1e-9
    assert any(r == 'tip invalid' for t, *_, r in out if 25.0 <= t < 27.0)


def test_a_moving_target_never_enters_the_hold():
    out, loop = _fly(0.63, moving=True, seconds=20.0)
    assert not any(h for _, _, _, h, _ in out) and loop.c == 0.0


def test_the_calibrated_setpoint_moves_the_goal():
    out, _ = _fly(TRUE_ARM, tip_dz=0.02)
    assert abs(out[-1][1] - _setpoint(0.02)) < 0.01


def test_deadband_holds_c_still_once_converged():
    out, loop = _fly(0.63, seconds=90.0)
    tail = [c for t, _, c, _, _ in out if t > 70.0]
    assert max(tail) - min(tail) < 1e-9 and loop.reason == 'deadband'


# ── tip validity (critic rev 1.3) ──

def _frames(n=12, tip=(0.0, 0.25, 1.06), body=(0.0, 0.25, 1.65), body_step=0.0, t0=10.0):
    return [(t0 + 0.01 * i, np.array(tip), np.array(body) + [body_step * i, 0.0, 0.0]) for i in range(n)]


def test_a_good_tip_passes():
    fr = _frames(n=12)
    fr = [(f[0], f[1] + [0.0, 0.0, 1e-4 * i], f[2]) for i, f in enumerate(fr)]
    assert tip_valid(fr[-1][0] + 0.01, fr) == (True, '')


def test_stale_distance_lean_step_and_frozen_fail():
    fr = _frames()
    assert tip_valid(fr[-1][0] + 0.2, fr)[1] == 'stale'
    assert tip_valid(10.2, _frames(tip=(0.0, 0.25, 0.9)))[1] == 'distance'
    assert tip_valid(10.2, _frames(tip=(0.2, 0.25, 1.11)))[1] == 'lean'
    fr = _frames()
    fr[-1] = (fr[-1][0], fr[-1][1] + [0.0, 0.0, 0.04], fr[-1][2])
    assert tip_valid(10.2, fr)[1] in ('step', 'distance')
    fr = _frames(n=30)                                     # bit-identical for 0.29 s
    assert tip_valid(fr[-1][0] + 0.01, fr)[1] == 'frozen'


def test_a_frozen_tip_is_caught_even_in_a_still_hold():
    # R1340: the rev-3 rule needed the body to move, so a freeze in a still hold passed as valid
    fr = _frames(n=30, body_step=0.0)
    assert tip_valid(fr[-1][0] + 0.01, fr)[1] == 'frozen'


def test_a_noisy_still_tip_is_valid():
    rng = np.random.default_rng(0)
    fr = [(10.0 + 0.01 * i, np.array([0.0, 0.25, 1.06]) + rng.normal(0.0, 2e-4, 3), np.array([0.0, 0.25, 1.65]))
          for i in range(40)]
    assert tip_valid(fr[-1][0] + 0.01, fr) == (True, '')
