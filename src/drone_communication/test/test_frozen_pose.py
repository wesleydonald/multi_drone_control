"""The frozen-pose filter of the mocap node (card 2026-11-11_frozen_pose_guard)."""
import numpy as np
import pytest

from drone_communication.frozen_pose import FrozenPoseFilter


def test_a_changing_pose_always_passes_and_a_short_repeat_too():
    f = FrozenPoseFilter(0.5)
    assert f.allow(14, ('0.1,0.2,0.3', '0,0,0,1'), 0.0)
    assert f.allow(14, ('0.1,0.2,0.3', '0,0,0,1'), 0.3)        # unchanged 0.3 s: still published
    assert f.allow(14, ('0.1,0.2,0.3001', '0,0,0,1'), 0.31)
    assert not f.withheld and not f.events


def test_unchanged_past_the_window_is_withheld_until_it_changes():
    f = FrozenPoseFilter(0.5)
    pose = ('0.29,2.34,2.89', '0,0,0,1')
    assert f.allow(14, pose, 0.0)
    assert f.allow(14, pose, 0.5)
    assert not f.allow(14, pose, 0.51) and not f.allow(14, pose, 4.0)
    assert f.withheld == {14} and f.events == [(14, 'frozen', 0.51)]
    assert f.allow(14, ('0.30,2.34,2.80', '0,0,0,1'), 4.1)      # it moves: published again at once
    assert not f.withheld and f.events[-1][:2] == (14, 'live')


def test_each_body_has_its_own_clock_and_zero_turns_it_off():
    f = FrozenPoseFilter(0.5)
    still, t = ('1,1,1', '0,0,0,1'), 0.0
    for k in range(100):
        t = k * 0.01
        assert f.allow(11, (f'{k},0,1', '0,0,0,1'), t)          # a live carrier beside a frozen QUAD4
        f.allow(14, still, t)
    assert f.withheld == {14}
    off = FrozenPoseFilter(0.0)
    assert all(off.allow(14, still, 10.0 * k) for k in range(5)) and not off.withheld


# the node side (the card's critic, F4): after withheld poses the parser must not read the jump across the gap as speed
@pytest.mark.parametrize('mode', ['ema', 'slope'])
def test_first_pose_after_a_gap_restarts_the_velocity(monkeypatch, mode):
    pytest.importorskip('scipy')
    node = pytest.importorskip('drone_communication.motion_capture_publisher_node')
    from drone_communication.velocity_filter import VelocityFilter
    clock = [100.0]
    monkeypatch.setattr(node.time, 'time', lambda: clock[0])
    parser = node.ParseData()
    if mode != 'ema':
        parser.vel_filter = VelocityFilter(mode, frames=4)

    def feed(x):
        clock[0] += 0.011
        return parser.parse_packet(f'14 | {x:.6f},0.5,1.2 | 0.0,0.0,0.0,1.0')

    for k in range(80):                                   # tracked and still: jitter only (the start-up transient of the ema decays first)
        out = feed(1e-5 * (k % 3))
    assert np.linalg.norm(out.velocity) < 0.01
    clock[0] += 2.0                                        # lost: nothing reaches the parser
    parser.reseed()
    assert feed(1.0) is None                               # the first pose back, 1 m away: not published
    out = feed(1.0 + 1e-5)
    assert out is not None and np.linalg.norm(out.velocity) < 0.1 and np.linalg.norm(out.angular_velocity) < 0.1
    for k in range(10):
        out = feed(1.0 + 1e-5 * (k % 3))
    assert np.linalg.norm(out.velocity) < 0.01


def test_without_the_restart_the_jump_reads_as_speed(monkeypatch):
    """What reseed() prevents: the same gap parsed straight through."""
    pytest.importorskip('scipy')
    node = pytest.importorskip('drone_communication.motion_capture_publisher_node')
    clock = [100.0]
    monkeypatch.setattr(node.time, 'time', lambda: clock[0])
    parser = node.ParseData()
    for x in (0.0, 0.0, 0.0):
        clock[0] += 0.011
        parser.parse_packet(f'14 | {x:.6f},0.5,1.2 | 0.0,0.0,0.0,1.0')
    clock[0] += 0.011
    out = parser.parse_packet('14 | 1.000000,0.5,1.2 | 0.0,0.0,0.0,1.0')
    assert np.linalg.norm(out.velocity) > 3.0
