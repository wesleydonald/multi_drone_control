"""The planner in the SIL lockstep: every due step waits for the tick marker, the scenario
clock starts on a tick, and a planner that stops ticking is let go (R0775 vs R0777 split
because the planner's solves landed on whichever bench step was current)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sil.lockstep import PlannerLockstep  # noqa: E402

DT = 0.02
PERIOD = 0.1


def _bench(first_tick_step, n_steps, ready_step, planner_alive=lambda t: True):
    """Bench steps against a fake 10 Hz planner whose timer was created at a step of its own.
    Returns (steps that waited for the planner, steps the planner ticked, start step)."""
    ls = PlannerLockstep()
    sim_t, waited, ticked, start = 0.0, [], [], None
    next_tick = first_tick_step * DT
    for k in range(n_steps):
        if start is None and k >= ready_step and ls.can_start(sim_t):
            start = k
        due = ls.due(sim_t)
        if sim_t >= next_tick - 1e-9 and planner_alive(sim_t):
            ls.on_marker(round(sim_t / DT) * DT, PERIOD)    # the stamp is the /clock value
            ticked.append(k)
            next_tick += PERIOD
        if due:
            waited.append(k)
            ls.step_done(sim_t)
        sim_t += DT
    return waited, ticked, start, ls


def test_every_tick_after_the_first_is_waited_for():
    for phase in range(5):
        waited, ticked, _, _ = _bench(7 + phase, 60000, 0)
        assert waited == ticked[1:], phase


def test_start_is_on_a_tick_whatever_the_timer_phase():
    for phase in range(5):
        _, ticked, start, _ = _bench(3 + phase, 200, 41)
        assert start in ticked and start >= 41
        # same phase of the scenario clock against the planner every run
        assert [k - start for k in ticked if k >= start][:3] == [0, 5, 10]


def test_no_marker_does_not_block_the_start():
    ls = PlannerLockstep()
    assert ls.can_start(1.23) and not ls.due(1.23)


def test_a_planner_that_stops_ticking_is_let_go():
    waited, ticked, _, ls = _bench(0, 400, 0, planner_alive=lambda t: t < 3.0)
    assert ls.gone
    assert len([k for k in waited if k > ticked[-1]]) == ls.give_up_after
