"""
tools/sil/lockstep.py
---------------------
The planner's side of the SIL lockstep (bench_node). The planner ticks at 10 Hz of sim
time on its own timer and publishes /planner/tick [t, period] once a tick's references
are out. A bench step on which a tick is due waits for that marker, and the scenario
clock starts on a tick, so the planner's phase against the events is the same every run.
"""
EPS = 1e-6


class PlannerLockstep:

    def __init__(self, give_up_after=5):
        self.give_up_after = int(give_up_after)
        self.last = None             # sim time of the last finished tick
        self.next = None             # when the next one is due
        self.silent = 0              # consecutive due steps without a marker
        self.gone = False

    def on_marker(self, t, period):
        self.last = t if self.last is None else max(self.last, t)
        self.next = self.last + period

    def due(self, sim_t):
        return self.next is not None and not self.gone and sim_t >= self.next - EPS

    def ticked_at(self, sim_t):
        return self.last is not None and abs(self.last - sim_t) < EPS

    def can_start(self, sim_t):
        """Start the scenario clock only on a step the planner ticks (or with no marker at all)."""
        return self.next is None or abs(self.next - sim_t) < EPS

    def step_done(self, sim_t):
        """After a due step's wait. True when the planner has just been given up on."""
        self.silent = 0 if self.ticked_at(sim_t) else self.silent + 1
        if self.silent >= self.give_up_after and not self.gone:
            self.gone = True
            return True
        return False
