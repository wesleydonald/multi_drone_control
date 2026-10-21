"""The magnet-marker release check, rev 3 after critic round 2 (tip_detect_m; Wesley 21 Oct, card 2026-10-21_tip_detect):
each carrier's marker in the ring frame from its plate point. The attached offset is learnt once after the lift, when
a 1 s window holds still, and bounded; then TIP_DETECT_S (and TIP_DETECT_N samples) past it by tip_detect_m, that also
moved the marker's distance from the ring's centre, are a release. Not releases: noise, blips, gaps, bad or repeated
samples, a ring-pose jump or attitude glitch (held off, or the distance unchanged), a lone armed marker (logged as a
would-fire), LAND and an unlearnt drone."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import (TIP_DETECT_S, TIP_HOLD_S, TIP_LEARN_S,  # noqa: E402
                                                  TIP_LEARN_WARN_S, DissipativeController as D)
from mpc_planner.geometry import attach_points, quat_to_rot_np  # noqa: E402

DT = 0.01                     # 100 Hz, as the rig's markers
OFF = np.array([0.0, 0.0, 0.03])   # the marker 3 cm above the plate point, as body 24 on the rig
BTT = 0.59                    # cable 0.55 from a pivot 4 cm below the body
Q0 = np.array([1.0, 0.0, 0.0, 0.0])
_RNG = np.random.default_rng(42)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


def _node(tip_m=0.04):
    rho = attach_points(3, 0.225, 0.02, '0,270,180')                 # the T 0/3/6: plates at +x, -y, -x
    log = _Log()
    f = SimpleNamespace(slot2drone=[0, 1, 2], detached=[False] * 3, rho=rho, _tip_detect_m=tip_m, _tip_v0={},
                        _tip_learn={}, _tip_learn_warned=set(), _tip_run={}, _tip_last={}, _tip_raw={},
                        _tip_lone={}, _tip_unarmed=set(), _tip_hold_until=-1e18, _tip_ring_same=0,
                        _detect_refused=set(), phase='planner', _reconfig_mode='ocp', _land_to_ground=False,
                        _unload=None, takeoff_seen=True, load_state=np.r_[0.0, 0.0, 1.0, Q0, np.zeros(6)],
                        cable_len=0.55, cable_len_i=[0.55] * 3, pivot_offset=np.array([0.0, 0.0, -0.04]),
                        lift_z0=0.02, target_z=1.0, lift_progress=0.98, released=[], t=100.0, _k=0)
    f.get_logger = lambda: log
    f.log = log
    f._on_unannounced = lambda d: f.released.append(d)
    for name in ('_tip_learn_step', '_tip_check', '_tip_ring_sample', '_tip_forget', '_tip_rod'):
        setattr(f, name, (lambda fn: lambda *a: fn(f, *a))(getattr(D, name)))
    f.drone_pos = [_tip(f, d) + BTT * _out(f, d) for d in range(3)]
    f._drone_t = {}
    return f


def _plate(f, d):
    return f.load_state[0:3] + np.asarray(f.rho[d], float)


def _out(f, d):
    """Unit vector from d's plate up and out to its drone, 60 deg up."""
    a = np.arctan2(f.rho[d][1], f.rho[d][0])
    return np.array([np.cos(a) * 0.5, np.sin(a) * 0.5, np.sqrt(3) / 2])


def _tip(f, d, extra=(0.0, 0.0, 0.0)):
    """Where d's marker truly is: on its plate (the ring level at its position), plus extra."""
    return _plate(f, d) + OFF + np.asarray(extra, float)


def _hang(f, d):
    """d's released magnet hanging below its quad: off the plate, in the body-to-marker band."""
    return f.drone_pos[d] - np.array([0.0, 0.0, BTT])


def _s(x, sd=0.0005):
    """One marker sample: Motive's sub-mm jitter (a sample repeated bit for bit is a lost marker)."""
    return np.asarray(x, float) + _RNG.normal(0.0, sd, 3)


def _step(f, samples, ring=None):
    """One 100 Hz frame: a ring sample (the same pose, jittered, unless given) and {drone: tip}; the drone poses are
    fresh."""
    f.t += DT
    f._k += 1
    if ring is None:
        ring = f.load_state.copy()
        ring[2] += 1e-6 * (1 if f._k % 2 else -1)
    prev = f.load_state[0:7].copy()
    f.load_state = np.asarray(ring, float)
    f._tip_ring_sample(prev, f.load_state[0:7], f.t)
    f._load_t = f.t
    for d, tip in samples.items():
        f._drone_t[d] = f.t
        f._tip_check(d, np.asarray(tip, float), f.t)


def _on(f, d):
    return {x: _s(_hang(f, x)) if x == d else _s(_tip(f, x)) for x in range(3)}


def _arm(f):
    for _ in range(int(TIP_LEARN_S / DT) + 3):
        _step(f, {d: _s(_tip(f, d)) for d in range(3)})
    assert set(f._tip_v0) == {0, 1, 2}


def _qy(deg):
    h = 0.5 * np.radians(deg)
    return np.array([np.cos(h), 0.0, np.sin(h), 0.0])                 # about the ring's y: the 3-9 axis


def test_it_arms_after_a_still_second_with_the_markers_offset():
    f = _node()
    _arm(f)
    v0, b0, r0 = f._tip_v0[1]
    assert np.allclose(v0, OFF, atol=2e-3) and b0 == pytest.approx(BTT, abs=2e-3)
    assert r0 == pytest.approx(np.hypot(0.225, 0.02 + 0.03), abs=2e-3)
    assert sum('armed' in m for m in f.log.lines) == 3


def test_it_does_not_learn_in_the_lift_or_while_the_marker_moves():
    f = _node()
    f.lift_progress = 0.5
    for _ in range(200):
        _step(f, {d: _s(_tip(f, d)) for d in range(3)})
    assert f._tip_v0 == {}
    f.lift_progress = 0.98                                           # lifted, but the markers rock 2 cm on their plates
    for k in range(int(TIP_LEARN_WARN_S / DT) + 20):
        w = 0.02 * np.sin(2 * np.pi * 1.5 * k * DT)
        _step(f, {d: _s(_tip(f, d, (w, 0.0, 0.0))) for d in range(3)})
    assert f._tip_v0 == {} and f.released == []
    assert sum('still learning' in m for m in f.log.lines) == 3


def test_a_wrong_offset_or_a_wrong_length_is_not_armed():
    f = _node()
    for _ in range(int(TIP_LEARN_S / DT) + 3):                       # drone 1's marker reads 10 cm off its plate
        _step(f, {0: _s(_tip(f, 0)), 1: _s(_tip(f, 1, (0.1, 0.0, 0.0))), 2: _s(_tip(f, 2))})
    assert 1 in f._tip_unarmed and 1 not in f._tip_v0 and any('NOT armed' in m for m in f.log.lines)
    for _ in range(30):
        _step(f, _on(f, 1))
    assert f.released == []


def test_a_magnet_swinging_off_its_plate_is_a_release_after_tip_detect_s():
    f = _node()
    _arm(f)
    for _ in range(int(round(TIP_DETECT_S / DT))):                   # 0.09 s of samples past: not yet
        _step(f, _on(f, 1))
    assert f.released == []
    for _ in range(2):
        _step(f, _on(f, 1))
    assert f.released == [1]
    assert any('UNANNOUNCED DETACH' in m and 'magnet' in m for m in f.log.lines)


def test_a_slack_cable_arms_and_its_release_still_fires():
    """The T's unloaded quad on the rig may hang a few cm slack: it arms, and once its magnet lets go the cable goes
    taut (body-to-marker = the rod), still inside the band."""
    f = _node()
    f.drone_pos[1] = _tip(f, 1) + (BTT - 0.06) * _out(f, 1)         # 6 cm closer than the rod: slack
    _arm(f)
    assert f._tip_v0[1][1] == pytest.approx(BTT - 0.06, abs=2e-3)
    for _ in range(12):
        _step(f, _on(f, 1))
    assert f.released == [1]


def test_noise_blips_a_gap_and_bad_samples_are_not_releases():
    f = _node()
    _arm(f)
    for _ in range(300):                                             # 3 mm noise
        _step(f, {d: _s(_tip(f, d), 0.003) for d in range(3)})
    for _ in range(9):                                               # a 0.08 s blip off the plate
        _step(f, _on(f, 1))
    _step(f, {d: _s(_tip(f, d)) for d in range(3)})
    for _ in range(6):                                               # past, a 0.1 s gap, then past again
        _step(f, _on(f, 1))
    f.t += 0.1
    for _ in range(6):
        _step(f, _on(f, 1))
    _step(f, {1: [np.nan, 0.0, 0.0]})                                # non-finite
    for _ in range(20):                                              # an untracked body: off the band
        _step(f, {0: _s(_tip(f, 0)), 1: _s([0.0, 0.0, 0.0]), 2: _s(_tip(f, 2))})
    assert f.released == []


def test_a_frozen_marker_is_no_evidence():
    """A lost Motive body repeats its last position bit for bit: one sample of evidence, not a run."""
    f = _node()
    _arm(f)
    stuck = _s(_hang(f, 1))
    for _ in range(30):
        _step(f, {0: _s(_tip(f, 0)), 1: stuck, 2: _s(_tip(f, 2))})
    assert f.released == []


def test_a_ring_pose_jump_moves_every_marker_and_is_not_a_release():
    f = _node()
    _arm(f)
    tips = {d: _tip(f, d) for d in range(3)}                         # the magnets stay where they are...
    ring = f.load_state.copy()
    ring[0] += 0.1                                                   # ...while the ring's mocap pose jumps 10 cm
    _step(f, {d: _s(tips[d]) for d in range(3)}, ring=ring)
    for _ in range(40):
        _step(f, {d: _s(tips[d]) for d in range(3)})
    assert f.released == []


@pytest.mark.parametrize('hold_s', [0.15, 1.0])
def test_a_ring_attitude_glitch_about_the_3_9_axis_fires_nothing(hold_s):
    """Critic round 2, M1: Motive mis-solves the ring's attitude 12 deg about the 3-9 axis. In the ring frame markers
    0 and 6 move ~4.8 cm and marker 3 ~1 cm, so the every-marker guard cannot see it, and a fire on 0 or 6 would leave
    a 270 deg gap and drop the ring. The step holds the check off, and after that the markers' distance from the
    ring's centre has not moved."""
    f = _node()
    _arm(f)
    tips = {d: _tip(f, d) for d in range(3)}
    good, bad = f.load_state.copy(), f.load_state.copy()
    bad[3:7] = _qy(12.0)
    dev0 = np.linalg.norm(quat_to_rot_np(bad[3:7]).T @ (tips[0] - good[0:3]) - np.asarray(f.rho[0], float)
                          - f._tip_v0[0][0])
    assert dev0 > 0.045                                              # past the threshold for as long as it lasts
    _step(f, {d: _s(tips[d]) for d in range(3)}, ring=bad)
    for _ in range(int(round(hold_s / DT))):
        _step(f, {d: _s(tips[d]) for d in range(3)})
    _step(f, {d: _s(tips[d]) for d in range(3)}, ring=good)
    for _ in range(30):
        _step(f, {d: _s(tips[d]) for d in range(3)})
    assert f.released == []
    assert any('held' in m for m in f.log.lines)


def test_a_slow_ring_attitude_glitch_without_a_step_fires_nothing():
    """The same 12 deg mis-solve creeping in at 1 deg a sample (no step to hold on) and held 1 s: only the distance
    from the ring's centre keeps it from firing."""
    f = _node()
    _arm(f)
    tips = {d: _tip(f, d) for d in range(3)}
    ring = f.load_state.copy()
    for k in range(1, 13):
        ring = ring.copy()
        ring[3:7] = _qy(float(k))
        _step(f, {d: _s(tips[d]) for d in range(3)}, ring=ring)
    for _ in range(100):
        _step(f, {d: _s(tips[d]) for d in range(3)})
    assert f.released == [] and not any('held' in m for m in f.log.lines)


def test_a_ring_pose_that_repeats_holds_the_check_off():
    f = _node()
    _arm(f)
    frozen = f.load_state.copy()
    for _ in range(3):                                               # the ring body lost: its last pose, bit for bit
        _step(f, {d: _s(_tip(f, d)) for d in range(3)}, ring=frozen.copy())
    assert f._tip_hold_until > f.t
    for _ in range(15):                                              # a release while it is frozen is not acted on...
        _step(f, _on(f, 1), ring=frozen.copy())
    assert f.released == []
    moving = frozen.copy()
    for _ in range(40):                                              # ...and once the ring moves again, it is
        moving = moving.copy()
        moving[0] += 1e-4
        _step(f, _on(f, 1), ring=moving)
    assert f.released == [1]
    assert sum('repeats' in m for m in f.log.lines) == 1


def test_a_lone_armed_marker_is_a_logged_would_fire_not_a_release():
    """Critic round 2, M3: with no other armed marker fresh there is nothing to tell a glitch from a release by."""
    f = _node()
    _arm(f)
    f.t += 1.0                                                       # markers 0 and 2 stop streaming
    for _ in range(30):
        _step(f, {1: _s(_hang(f, 1))})
    assert f.released == []
    assert sum('WOULD-FIRE' in m for m in f.log.lines) == 1


def test_it_is_quiet_when_off_in_land_or_detached():
    f = _node(tip_m=0.0)
    for _ in range(300):
        _step(f, {1: _s(_tip(f, 1, (0.0, -0.3, 0.0)))})
    assert f.released == [] and f._tip_v0 == {}
    f = _node()
    _arm(f)
    f._land_to_ground = True                                         # LAND, ring still up: the stretch check only
    for _ in range(20):
        _step(f, _on(f, 1))
    f._land_to_ground, f.detached[1] = False, True
    for _ in range(20):
        _step(f, _on(f, 1))
    assert f.released == []


def test_attach_and_detach_forget_that_drone_only():
    f = _node()
    _arm(f)
    f._tip_forget(1)
    assert set(f._tip_v0) == {0, 2} and 1 not in f._tip_last and f._tip_run == {}


def test_the_offline_replay_runs_the_same_check_over_recorded_samples():
    """tools/tip_replay.py (Wesley 21 Oct: Part C flies the check only if the A/B flights replay clean)."""
    import importlib.util
    import pathlib
    spec = importlib.util.spec_from_file_location(
        'tip_replay', pathlib.Path(__file__).resolve().parents[3] / 'tools' / 'tip_replay.py')
    tr = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tr)
    g = _node()                                  # the T geometry, typed clockwise as the sheet does: 0,90,180
    v = np.zeros(3)
    ev, t = [], 0.0
    bodies = {}
    for k in range(400):                         # rest, lift to 1.0, hold
        t += DT
        z = min(1.0, 0.02 + 0.01 * k)
        ev.append((t, 'ring', -1, np.array([0.0, 0.0, z + 1e-6 * (k % 2)]), (Q0, v)))
        for d in range(3):
            tip = np.array([0.0, 0.0, z - 1.0]) + _tip(g, d)
            bodies[d] = tip + BTT * _out(g, d)
            ev.append((t, 'drone', d, bodies[d], None))
            ev.append((t, 'tip', d, _s(tip), None))
    f, fires, devmax = tr.replay(ev, [0.0, 90.0, 180.0], 0.02)
    assert set(f._tip_v0) == {0, 1, 2} and fires == [] and max(devmax.values()) < 0.01
    for k in range(15):                          # drone 2 holds its place while its magnet swings off its plate
        t += DT
        ev.append((t, 'ring', -1, np.array([0.0, 0.0, 1.0 + 1e-6 * (k % 2)]), (Q0, v)))
        for d in range(3):
            ev.append((t, 'drone', d, bodies[d], None))
            ev.append((t, 'tip', d, _s(bodies[2] - np.array([0.0, 0.0, BTT])) if d == 2 else _s(_tip(g, d)), None))
    f, fires, devmax = tr.replay(ev, [0.0, 90.0, 180.0], 0.02)
    assert [d for _, d, _ in fires] == [2]
