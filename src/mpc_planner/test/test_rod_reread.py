"""The rod reading in the pretension's hold (card 2026-10-31_rod_len_reread): the median of the hold, a ramp from the lengths flown, once;
refuses a change outside its limits; a cancel holds the lengths."""
import pytest

from mpc_planner.rod_reread import RodReread


def test_reads_the_hold_and_ramps():
    r = RodReread(ramp_s=0.5)
    for d in ([0.978, 1.000], [0.980, 1.002], [0.979, 1.001]):      # the middle sample is each rod's median
        r.sample(d)
    assert r.step(0.0) is None                                   # nothing moves before the hold ends
    r.finish(10.0, [0.980, 0.986])
    assert r.state == 'ramp' and r.target == pytest.approx([0.979, 1.001])
    assert r.step(10.0) == pytest.approx([0.980, 0.986])
    assert r.step(10.25) == pytest.approx([0.9795, 0.9935])
    assert r.step(10.5) == pytest.approx([0.979, 1.001]) and r.state == 'done' and r.done
    assert r.step(11.0) is None
    r.sample([0.5, 0.5])                                         # once
    r.finish(12.0, [0.979, 1.001])
    assert r.state == 'done' and r.target == pytest.approx([0.979, 1.001])


@pytest.mark.parametrize('read, ok', [(1.019, True), (1.021, False), (0.993, True), (0.991, False)])
def test_limits(read, ok):
    r = RodReread()
    r.sample([1.0, read])
    r.finish(0.0, [1.0, 1.0])
    assert (r.state == 'ramp') == ok
    if not ok:
        assert r.state == 'refused' and r.done and 'kept' in r.why and r.step(1.0) is None


def test_a_cancel_holds_the_lengths():
    r = RodReread()
    r.sample([1.01])
    r.finish(0.0, [1.0])
    assert r.step(0.1) == pytest.approx([1.002])
    r.cancel('a resize')
    assert r.state == 'cancelled' and r.why == 'a resize' and r.step(0.2) is None


def test_no_reading_is_no_change():
    r = RodReread()
    r.finish(0.0, [1.0])
    assert r.state == 'cancelled' and r.step(0.1) is None


def test_a_glitch_in_the_hold_does_not_move_a_rod():
    r = RodReread()
    for d in ([1.01], [1.01], [1.012], [1.011], [1.01]):
        r.sample(d)
    r.finish(0.0, [1.0])
    assert r.target == pytest.approx([1.01])
    r = RodReread()
    for d in ([1.01], [1.07], [1.01], [1.01], [1.01]):            # one 60 mm glitch
        r.sample(d)
    r.finish(0.0, [1.0])
    assert r.state == 'refused' and 'spread' in r.why
