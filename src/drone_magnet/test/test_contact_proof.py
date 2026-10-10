"""The contact-and-lift proof (card 2026-10-10_combined_m1_contact, Goal 2), fed synthetic 120 Hz
traces: the tip rests on plate 9 from t=0, the body lifts from t=0.3 s at 0.02 m/s up to 1 cm. A
caught tip carries the plate up with it; an uncaught tip rises alone."""
import json
import math

import numpy as np
import pytest

from drone_magnet.contact_proof import (ATTACHED, OFF, PROVE, WATCH, ContactProof, ProofConfig,
                                        slope_speed)

HZ = 120.0
PLATE = np.array([0.0, 0.225, 1.032])
BODY_TO_TIP = 0.59


def _trace(kind='caught', *, t0=0.0, t_end=4.0, t_lift=0.3, rate=0.02, cap=0.01, rise=0.005,
           c=(0.0, 0.0, 0.0), noise=0.0, seed=0, body_to_tip=BODY_TO_TIP):
    """(t, tip, plate, body) samples. caught: tip = plate + c, the plate rising with the lift up to
    `rise` (the rigid twin rod holds the rest); uncaught: the tip rises with the body, the plate
    stays; sunk: the tip 3 mm below the plate point, nothing rises."""
    rng = np.random.default_rng(seed)
    out = []
    for k in range(int(round((t_end - t0) * HZ)) + 1):
        t = t0 + k / HZ
        up = min(max(t - t_lift, 0.0) * rate, cap)
        if kind == 'caught':
            plate = PLATE + [0.0, 0.0, min(up, rise)]
            tip = plate + c
        elif kind == 'uncaught':
            plate = PLATE.copy()
            tip = PLATE + c + [0.0, 0.0, up]
        else:
            plate = PLATE.copy()
            tip = PLATE + [0.0, 0.0, -0.003]
        body = tip + [0.0, 0.0, body_to_tip]
        out.append((t, tip + rng.normal(0.0, noise, 3), plate, body))
    return out


def _run(p, trace, fresh=lambda t: True, body=True):
    events = []
    for t, tip, plate, b in trace:
        ev = p.step(t, tip, plate, body=b if body else None, fresh=fresh(t))
        if ev:
            events.append((t, ev, p.reason if ev == 'failed' else ''))
    return events


def _on(config=None):
    p = ContactProof(config or ProofConfig())
    assert p.magnet(True) == 'on' and p.state == WATCH
    return p


def _names(events):
    return [e[1] for e in events]


def test_caught_catch_is_proven():
    p = _on()
    ev = _run(p, _trace('caught'))
    assert _names(ev) == ['contact', 'proven']
    t_contact, t_proven = ev[0][0], ev[1][0]
    assert t_contact == pytest.approx(0.15, abs=1.5 / HZ)
    assert 0.6 < t_proven and t_proven - t_contact < 3.0
    assert p.state == ATTACHED and p.tries == 1 and p.fails == 0
    assert p.rise >= 0.003 and p.drift <= 0.003


def test_near_miss_fails_on_drift_and_is_never_proven():
    p = _on()
    ev = _run(p, _trace('uncaught'))
    assert _names(ev)[:2] == ['contact', 'failed'] and ev[1][2] == 'drift'
    assert 'proven' not in _names(ev)
    # the lift at 0.02 m/s: drift 8 mm 0.4 s into it, then the 0.12 s fail dwell
    assert ev[1][0] == pytest.approx(0.3 + 0.4 + 0.05 + 0.12, abs=0.03)
    assert p.last_fail['reason'] == 'drift' and p.last_fail['plate_rise_m'] < 0.001


def test_off_centre_catch_is_proven_not_rejected():
    p = _on()
    ev = _run(p, _trace('caught', c=(0.006, 0.0, 0.0)))
    assert _names(ev) == ['contact', 'proven']
    assert p.c0 == pytest.approx([0.006, 0.0, 0.0], abs=1e-9)


def test_sunk_tip_is_not_proven_and_fails_by_timeout():
    p = _on()
    ev = _run(p, _trace('sunk'))
    assert _names(ev)[:2] == ['contact', 'failed'] and ev[1][2] == 'timeout'
    assert ev[1][0] - ev[0][0] == pytest.approx(3.0, abs=1.5 / HZ)
    assert 'proven' not in _names(ev)
    # the tip is still on the plate: watching again, a new contact after a fresh dwell
    assert _names(ev)[2:] == ['contact'] and ev[2][0] - ev[1][0] == pytest.approx(0.15, abs=1.5 / HZ)


def test_slow_catch_is_proven_within_the_timeout():
    p = _on()
    ev = _run(p, _trace('caught', rate=0.002))
    assert _names(ev) == ['contact', 'proven']
    assert 1.5 < ev[1][0] - ev[0][0] < 3.0


def test_a_catch_slower_than_the_timeout_fails():
    p = _on()
    ev = _run(p, _trace('caught', rate=0.001))
    assert _names(ev)[:2] == ['contact', 'failed'] and ev[1][2] == 'timeout'


def test_a_loss_after_the_proof_keeps_the_attach():
    p = _on()
    _run(p, _trace('caught', t_end=2.0))
    assert p.state == ATTACHED
    later = [(t, tip + [0.0, 0.0, 0.02], plate, body)
             for t, tip, plate, body in _trace('caught', t0=2.0 + 1 / HZ, t_end=3.0)]
    assert _run(p, later) == []
    assert p.state == ATTACHED and p.drift > 0.008            # logged, not acted on
    assert p.magnet(False) == 'refused' and p.state == ATTACHED


@pytest.mark.parametrize('seed', range(20))
def test_tip_noise_gives_no_false_fail_and_no_false_proof(seed):
    p = _on()
    assert _names(_run(p, _trace('caught', noise=0.0005, seed=seed))) == ['contact', 'proven']
    p = _on()
    ev = _names(_run(p, _trace('uncaught', noise=0.0005, seed=seed)))
    assert ev[0] == 'contact' and 'proven' not in ev
    p = _on()
    ev = _names(_run(p, _trace('sunk', noise=0.0005, seed=seed)))
    assert ev[0] == 'contact' and 'proven' not in ev


def test_contact_needs_the_tip_slow_for_the_dwell():
    p = _on()
    # through the 1 cm radius at 0.06 m/s: inside for 0.33 s, but never slow
    trace = [(k / HZ, PLATE + [0.0, -0.01 + 0.06 * k / HZ, 0.0], PLATE, None) for k in range(40)]
    assert _run(p, trace) == []
    assert p.rel_speed == pytest.approx(0.06, rel=1e-6)


def test_contact_needs_the_tip_within_the_radius():
    p = _on()
    ev = _run(p, [(t, plate + [0.0, 0.0, 0.0101], plate, b) for t, _, plate, b in _trace('sunk', t_end=1.0)])
    assert ev == [] and p.state == WATCH


def test_stale_plate_or_gaps_earn_no_dwell():
    p = _on()
    # every 0.1 s one stale sample: the 0.15 s dwell never completes
    ev = _run(p, _trace('caught', t_end=1.0, t_lift=5.0), fresh=lambda t: round(t * HZ) % 12 != 0)
    assert ev == []
    p = _on()
    # one sample every 0.125 s: a gap longer than proof_ma_s restarts the dwell each time
    ev = _run(p, [s for i, s in enumerate(_trace('caught', t_end=1.0, t_lift=5.0)) if i % 15 == 0])
    assert ev == []


def test_the_timeout_runs_without_samples():
    p = _on()
    _run(p, _trace('sunk', t_end=0.2))
    assert p.state == PROVE
    t_contact = p.t_contact
    assert p.tick(t_contact + 2.9) is None
    assert p.tick(t_contact + 3.0) == 'failed' and p.reason == 'timeout' and p.state == WATCH


def test_rig_taut_check():
    cfg = ProofConfig(taut_len=0.587)
    p = _on(cfg)
    assert _names(_run(p, _trace('caught'))) == ['contact', 'proven']
    p = _on(cfg)                                              # slack: the tether not yet taut
    ev = _run(p, _trace('caught', body_to_tip=0.585))
    assert _names(ev)[:2] == ['contact', 'failed'] and ev[1][2] == 'timeout'
    p = _on(cfg)                                              # no body pose: never proven
    assert 'proven' not in _names(_run(p, _trace('caught'), body=False))


# ── the arbiter ──────────────────────────────────────────────────────────────

def test_nothing_happens_before_the_magnet_is_on():
    p = ContactProof(ProofConfig())
    assert p.state == OFF and _run(p, _trace('caught')) == []
    assert p.magnet(False) is None


def test_off_during_the_proof_resets_until_a_new_on():
    p = _on()
    trace = _trace('caught', t_end=2.0)
    _run(p, trace[:30])
    assert p.state == PROVE
    assert p.magnet(False) == 'reset' and p.state == OFF and p.c0 is None
    assert _run(p, trace[30:]) == []                          # a perfect catch, but the magnet is OFF
    assert p.magnet(True) == 'on'
    again = _trace('caught', t0=2.0 + 1 / HZ, t_end=6.0, t_lift=2.5)
    ev = _run(p, again)
    assert _names(ev) == ['contact', 'proven']
    assert ev[0][0] - again[0][0] == pytest.approx(0.15, abs=1.5 / HZ)    # a fresh dwell
    assert p.tries == 2


def test_on_keepalive_during_the_proof_changes_nothing():
    p = _on()
    trace = _trace('caught')
    ev = _run(p, trace[:40])
    assert p.magnet(True) is None and p.state == PROVE
    ev += _run(p, trace[40:])
    assert _names(ev) == ['contact', 'proven'] and p.tries == 1


def test_off_after_the_attach_is_refused():
    p = _on()
    _run(p, _trace('caught'))
    assert p.state == ATTACHED
    assert p.magnet(False) == 'refused' and p.state == ATTACHED
    assert p.magnet(True) is None and p.state == ATTACHED


def test_off_while_watching():
    p = _on()
    assert p.magnet(False) == 'off' and p.state == OFF


# ── the record and the config ────────────────────────────────────────────────

def test_report_is_plain_json():
    p = _on()
    keys = {'state', 'drift_m', 'plate_rise_m', 'body_to_tip_m', 'tries', 'contact_age_s'}
    r = p.report(0.0)
    assert keys <= set(r) and r['drift_m'] is None
    trace = _trace('caught')
    _run(p, trace[:60])
    r = json.loads(json.dumps(p.report(trace[59][0]), allow_nan=False))
    assert r['state'] == PROVE and r['tries'] == 1
    assert r['body_to_tip_m'] == pytest.approx(BODY_TO_TIP)
    assert r['contact_age_s'] == pytest.approx(trace[59][0] - p.t_contact, abs=1e-5)


def test_config_is_checked():
    with pytest.raises(ValueError):
        ProofConfig(d_ok=0.008, d_fail=0.008)
    with pytest.raises(ValueError):
        ProofConfig(timeout_s=math.nan)
    with pytest.raises(ValueError):
        ProofConfig(ma_s=0.0)


def test_slope_speed():
    t = np.arange(10) / HZ
    r = np.outer(t, [0.03, 0.0, -0.04]) + [1.0, 2.0, 3.0]
    assert slope_speed(t, r) == pytest.approx(0.05)
    assert math.isnan(slope_speed(t[:1], r[:1]))
