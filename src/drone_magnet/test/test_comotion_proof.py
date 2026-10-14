"""Tejen's attach confirmation for the airborne ring (comotion_proof.py), on synthetic 100 Hz tracks shaped like the
14 Oct rig flights: the t6h_4 catch (tip within ~1 mm of the plate while ring and QUAD4 bobbed +-25 mm, tether at
0.5694 m against a free hang of 0.5698 m) must be proven; a resting or lifted-away tip must not."""
import math

import numpy as np
import pytest

from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from drone_magnet.contact_proof import ATTACHED, PROVE, WATCH, ContactProof, ProofConfig

DT = 0.01
L_HANG = 0.5698
PLATE0 = np.array([0.0, -0.19, 1.0])


def _rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


class Rig:
    """Plate (the weld point) on a ring that bobs and tilts; the tip given in the plate frame; QUAD4's body above."""

    def __init__(self, proof, tilt_deg=0.0, bob=0.025, period=2.5, seed=0):
        self.p, self.t = proof, 100.0
        self.R = _rot_x(tilt_deg)
        self.bob, self.period = bob, period
        self.rng = np.random.default_rng(seed)
        self.events = []

    def plate(self):
        return PLATE0 + np.array([0.0, 0.0, self.bob * math.sin(2 * math.pi * (self.t - 100.0) / self.period)])

    def run(self, seconds, rel, btt, lift=False, noise=0.0003, fresh=True):
        """rel: tip in the plate frame (a vector, or a function of time); btt: body-to-tip (float or function)."""
        out = None
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            r = np.asarray(rel(self.t) if callable(rel) else rel, float) + self.rng.normal(0.0, noise, 3)
            plate = self.plate()
            tip = plate + self.R @ r
            length = btt(self.t) if callable(btt) else btt
            body = tip + np.array([0.0, 0.0, length])
            ev = self.p.step(self.t, tip, plate, body=body, fresh=fresh, rotation=self.R, lift_done=lift)
            ev = ev or self.p.tick(self.t)
            if ev:
                self.events.append((round(self.t, 2), ev))
                out = out or ev
        return out


def _proof(**kw):
    p = ComotionProof(ComotionConfig(lift_fallback_s=0.0, **kw))
    p.magnet(True)
    return p


def _hang(rig, seconds=3.0):
    """Free hang 0.3 m above the plate, the tether at its own length."""
    return rig.run(seconds, [0.0, 0.0, 0.30], L_HANG, noise=0.0)


CAUGHT = np.array([-0.0233, 0.005, 0.0026])      # t6h_4: the contact 23.8 mm off the weld point, 2.6 mm high


def test_the_t6h_4_catch_is_proven_a_second_after_the_lift():
    rig = Rig(_proof(), tilt_deg=6.0)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.003) == 'contact'
    assert rig.p.l_hang == pytest.approx(L_HANG, abs=0.0005)
    assert rig.run(1.0, CAUGHT, 0.5694) is None                      # the lift still running: no verdict
    t_lift = rig.t
    assert rig.run(1.5, CAUGHT, 0.5694, lift=True) == 'proven'
    assert rig.p.state == ATTACHED
    assert rig.events[-1][0] - t_lift == pytest.approx(1.0, abs=0.05)


def test_the_old_rules_reproduce_the_14_oct_rejection():
    old = ContactProof(ProofConfig(radius=0.024, speed=0.05, contact_dwell_s=0.15, d_ok=0.003, d_fail=0.008,
                                   h_plate=0.003, proof_dwell_s=0.3, fail_dwell_s=0.12, timeout_s=3.0,
                                   taut_len=0.572))
    old.magnet(True)
    rig = Rig(old, tilt_deg=0.0)
    out = None
    for _ in range(600):
        rig.t += DT
        plate = rig.plate()
        tip = plate + CAUGHT
        ev = old.step(rig.t, tip, plate, body=tip + np.array([0.0, 0.0, 0.5694])) or old.tick(rig.t)
        out = out or (ev if ev in ('proven', 'failed') else None)
    assert out == 'failed' and old.last_fail['reason'] == 'timeout'


def test_a_tip_the_lift_raises_off_the_plate_is_separated():
    rig = Rig(_proof(), bob=0.0)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.010) == 'contact'         # resting, 10 mm of slack
    t0 = rig.t
    rise = lambda t: CAUGHT + np.array([0.0, 0.0, min(0.020, 0.02 * (t - t0) / 0.5)])
    out = rig.run(1.0, rise, L_HANG, lift=True)
    assert out == 'failed' and rig.p.last_fail['reason'] == 'separated' and rig.p.state == WATCH


def test_a_resting_tip_on_a_slack_tether_is_never_proven():
    rig = Rig(_proof())
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.010) == 'contact'
    out = rig.run(3.5, CAUGHT, L_HANG - 0.010, lift=True)            # stays with the bobbing plate, never taut
    assert out == 'failed' and rig.p.last_fail['reason'] == 'inconclusive'


def test_an_edge_catch_inside_the_gate_is_proven():
    rig = Rig(_proof())
    _hang(rig)
    edge = np.array([0.032, 0.0, 0.002])
    assert rig.run(0.5, edge, L_HANG - 0.002) == 'contact'
    assert rig.run(1.5, edge, L_HANG - 0.001, lift=True) == 'proven'


def test_a_tip_brushing_past_makes_no_contact():
    rig = Rig(_proof(), bob=0.0)
    _hang(rig)
    t0 = rig.t
    sweep = lambda t: np.array([-0.04 + 0.10 * (t - t0), 0.0, 0.002])
    assert rig.run(0.8, sweep, L_HANG - 0.002) is None and rig.p.state == WATCH


def test_contact_needs_the_tip_on_the_plate_not_above_it():
    rig = Rig(_proof(), bob=0.0)
    assert rig.run(1.0, [0.0, 0.0, 0.020], L_HANG) is None           # 2 cm above, still: no contact


def test_a_two_sample_marker_glitch_neither_fails_nor_breaks_the_proof():
    rig = Rig(_proof())
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    glitch_t = [rig.t + 0.3]
    glitch = lambda t: CAUGHT + (np.array([0.05, 0.0, 0.0]) if glitch_t[0] <= t < glitch_t[0] + 0.02 else 0.0)
    out = rig.run(2.5, glitch, L_HANG - 0.002, lift=True)
    assert out == 'proven'


def test_a_data_gap_while_proving_is_stale():
    rig = Rig(_proof())
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    out = rig.run(0.6, CAUGHT, L_HANG - 0.002, fresh=False)
    assert out == 'failed' and rig.p.last_fail['reason'] == 'stale'


def test_his_planner_leaving_the_proof_voids_the_contact():
    rig = Rig(_proof())
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.abort('aborted', rig.t) == 'failed' and rig.p.state == WATCH
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002, lift=True) == 'contact'   # a new contact can follow


def test_the_hang_length_comes_from_a_swinging_free_hang():
    rig = Rig(_proof(), bob=0.0)
    swing = lambda t: np.array([0.03 * math.sin(2 * math.pi * 0.66 * t), 0.0, 0.30])
    rig.run(3.0, swing, lambda t: L_HANG + 0.0004 * math.sin(7.0 * t), noise=0.0)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.l_hang == pytest.approx(L_HANG, abs=0.0005)


def test_without_a_lift_signal_the_fallback_times_the_lift():
    rig = Rig(ComotionProof(ComotionConfig(lift_fallback_s=1.0)))
    rig.p.magnet(True)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    t_contact = rig.p.t_contact
    assert rig.run(2.2, CAUGHT, L_HANG - 0.002) == 'proven'
    assert rig.events[-1][0] - t_contact == pytest.approx(2.0, abs=0.06)   # lift at +1.0, proof dwell 1.0


def test_no_hang_seen_and_no_typed_length_skips_the_taut_check_like_the_twin():
    rig = Rig(_proof(), bob=0.004)
    assert rig.run(0.5, [0.0, 0.0, 0.0], 0.49) == 'contact'           # the twin's rigid rod: no free hang above
    assert rig.p.l_hang is None
    assert rig.run(1.5, [0.0, 0.0, 0.0], 0.49, lift=True) == 'proven'


def test_the_plate_frame_reads_a_tilted_catch_as_on_the_plate():
    rig = Rig(_proof(), tilt_deg=8.0, bob=0.0)
    _hang(rig)
    assert rig.run(0.5, [0.0, 0.03, 0.0], L_HANG - 0.002) == 'contact'
    assert abs(rig.p.normal) < 0.002                                    # in the world frame it reads 4 mm off


def test_off_during_the_proof_resets_and_off_after_the_attach_is_refused():
    rig = Rig(_proof())
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.magnet(False) == 'reset' and rig.p.state == 'off'
    rig.p.magnet(True)
    _hang(rig)
    rig.run(0.5, CAUGHT, L_HANG - 0.002)
    assert rig.run(1.5, CAUGHT, L_HANG - 0.002, lift=True) == 'proven'
    assert rig.p.magnet(False) == 'refused' and rig.p.state == ATTACHED


def test_report_is_plain_json():
    import json
    rig = Rig(_proof())
    _hang(rig)
    rig.run(0.5, CAUGHT, L_HANG - 0.002)
    rec = rig.p.report(rig.t)
    json.dumps(rec)
    assert rec['rules'] == 'comotion' and rec['state'] == PROVE and rec['l_hang_m'] == pytest.approx(L_HANG, abs=5e-4)


def test_config_is_checked():
    with pytest.raises(ValueError):
        ComotionConfig(normal_min=0.01)
    with pytest.raises(ValueError):
        ComotionConfig(proof_dwell_s=0.0)


# ── critic 15 Oct: the free hang through a long bottom hold and retries (F1), a plate moving under a still tip and
# a gap in the tip stream (F7); the twin's still plate ──

def test_a_long_bottom_hold_keeps_the_free_hang():
    rig = Rig(_proof(taut_fallback=0.580), bob=0.0)
    _hang(rig)
    rig.run(10.0, [0.040, 0.0, 0.004], L_HANG - 0.004)              # 10 s at the bottom, just outside the gate
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.l_hang == pytest.approx(L_HANG, abs=0.0005) and rig.p.l_hang_source == 'free hang'


def test_a_failed_proof_keeps_the_free_hang_for_the_retry():
    rig = Rig(_proof(taut_fallback=0.580), bob=0.0)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.010) == 'contact'
    t0 = rig.t
    rise = lambda t: CAUGHT + np.array([0.0, 0.0, min(0.020, 0.02 * (t - t0) / 0.5)])
    assert rig.run(1.0, rise, L_HANG, lift=True) == 'failed'
    rig.run(2.0, [0.040, 0.0, 0.004], L_HANG - 0.004)                # his retry, back at the bottom
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.l_hang == pytest.approx(L_HANG, abs=0.0005) and rig.p.l_hang_source == 'free hang'


def test_without_a_free_hang_the_typed_length_is_the_named_fallback():
    rig = Rig(_proof(taut_fallback=0.575), bob=0.0)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert rig.p.l_hang_source == 'fallback' and rig.p.l_hang == pytest.approx(0.575 - 0.005 + 0.003)


def test_a_plate_moving_under_a_still_tip_is_not_proven():
    """The tip hangs still in the world on a straight tether while the plate drops 5.5 mm under it: drift and plate
    motion are the same size, so the tip does not follow the plate."""
    rig = Rig(_proof(), bob=0.0)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG) == 'contact'
    t0 = rig.t
    drop = lambda t: min(0.0055, 0.0055 * (t - t0) / 2.0)
    rig.plate = lambda: PLATE0 - np.array([0.0, 0.0, drop(rig.t)])
    out = rig.run(3.5, lambda t: CAUGHT + np.array([0.0, 0.0, drop(t)]), L_HANG, lift=True)
    assert out == 'failed' and 'proven' not in [e for _, e in rig.events]
    assert rig.p.fails == 1 and rig.p.last_fail['reason'] == 'inconclusive'


def test_the_twin_proves_a_welded_tip_on_a_still_plate_without_plate_motion():
    assert Rig(_proof(), bob=0.0).run(0.0, CAUGHT, L_HANG) is None
    rig = Rig(_proof(require_motion=0.0), bob=0.0)
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.001) == 'contact'
    assert rig.run(1.5, CAUGHT, L_HANG - 0.001, lift=True) == 'proven'
    rig = Rig(_proof(), bob=0.0)                                     # the rig's rules: a still plate is no proof
    _hang(rig)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.001) == 'contact'
    assert rig.run(3.5, CAUGHT, L_HANG - 0.001, lift=True) == 'failed'


def test_a_gap_in_the_tip_stream_restarts_the_contact_dwell():
    rig = Rig(_proof(), bob=0.0)
    _hang(rig)
    assert rig.run(0.25, CAUGHT, L_HANG - 0.002) is None             # 0.25 s of the 0.30 s dwell
    rig.t += 0.4                                                     # a mocap dropout: no tip samples for 0.4 s
    assert rig.run(0.05, CAUGHT, L_HANG - 0.002) is None             # not a contact on the first samples after it
    assert rig.p.gap_restarts == 1
    assert rig.run(0.3, CAUGHT, L_HANG - 0.002) == 'contact'


def test_a_short_executor_stall_keeps_the_dwell():
    """Laptop stalls of 0.16-0.25 s (14 Oct) must not restart the proof's windows (critic round 2, finding 8)."""
    rig = Rig(_proof(), bob=0.0)
    _hang(rig)
    assert rig.run(0.20, CAUGHT, L_HANG - 0.002) is None
    rig.t += 0.2
    assert rig.run(0.02, CAUGHT, L_HANG - 0.002) == 'contact' and rig.p.gap_restarts == 0


def test_the_free_hang_is_measured_with_the_coil_off_too():
    """The span is OFF while he climbs back and matches (critic F3): the tether length is still measured there."""
    p = ComotionProof(ComotionConfig(lift_fallback_s=0.0, taut_fallback=0.580))
    rig = Rig(p, bob=0.0)
    _hang(rig)                                                       # coil OFF: no contact logic, but the free hang
    p.magnet(True)
    assert rig.run(0.5, CAUGHT, L_HANG - 0.002) == 'contact'
    assert p.l_hang == pytest.approx(L_HANG, abs=0.0005) and p.l_hang_source == 'free hang'
