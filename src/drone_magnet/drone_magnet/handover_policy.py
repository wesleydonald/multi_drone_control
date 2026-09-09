"""
handover_policy.py
------------------
Who flies the approach drone: the collaborator's approach MPC, or our dissipative
tracker. Pure (no ROS) so the rule can be tested without a running stack -- the rule is
the safety-critical part of `elrs_mux`, and the bug it now prevents killed the drone in
12 of 12 Gazebo attach runs (R0031-R0046).

THE RULE

  Hand over when the weld has happened AND our tracker is actually commanding thrust.

The second half is not belt-and-braces. A tracker with no reference publishes armed-idle
-- `channel_2 = -1.0`, motors off (`controller_mpc.control_loop`) -- and the newcomer
cannot have a reference at the weld instant unless something publishes one for it first,
because `dissipative_node` clears `attach_pending` on the same /magnet/object_attached
message the mux switches on and only publishes on its next 10 Hz plan tick. Switching
into that window cut the motors 0.03 s after every weld; with throttle at zero the
betaflight mixer (throttle +/- rate offsets, clipped at 0) also loses attitude authority,
so the drone tumbled to 89 deg within 0.35 s and tripped the fleet envelope.

The weld itself still LATCHES, so this only ever delays the handover to the first flying
command -- it cannot hand a welded drone back to the approach controller.
"""

# channel_2 = -1.0 exactly is the trackers' armed-idle throttle, so anything above it is
# a real commanded thrust. Comparing against a value just above -1.0 rather than -1.0
# itself keeps float round-trips through the message from reading as "flying".
IDLE_THROTTLE = -0.99


class HandoverPolicy:
    """Decides which input stream `elrs_mux` forwards.

    Times are seconds from any monotonic source; the caller owns the clock.
    """

    def __init__(self, latch=True, require_live=True, live_timeout_s=0.5):
        self.latch = bool(latch)
        self.require_live = bool(require_live)
        self.live_timeout_s = float(live_timeout_s)
        self.welded = False
        self._last_live_t = None

    def weld(self, attached, now=None):
        """Feed /magnet/object_attached."""
        if attached:
            self.welded = True
        elif not self.latch:
            self.welded = False
        return self.attached(now)

    def tracker_command(self, armed, throttle, now):
        """Feed one command from our tracker's pre-mux topic."""
        if armed and throttle > IDLE_THROTTLE:
            self._last_live_t = float(now)
        return self.attached(now)

    def tracker_live(self, now=None):
        if not self.require_live:
            return True
        if self._last_live_t is None:
            return False
        if now is None:
            return True
        return (float(now) - self._last_live_t) <= self.live_timeout_s

    def attached(self, now=None):
        """True when our tracker should be flying the drone."""
        return self.welded and self.tracker_live(now)
