"""Arm switch with flight-controller confirmation (pure, no ROS; driven by elrs_interface).

Betaflight latches ARMING_DISABLED_ARM_SWITCH when the arm switch goes high while any
arming-disabled flag is set, and then needs a low -> high edge before it will arm. The
rig showed it as "QUAD1/QUAD2 do not arm on the first press; a power cycle fixes it"
(2026-10-01). So the switch is held low briefly before every arm edge, the FC's CRSF
flight-mode string confirms the arm (Betaflight appends '*' while disarmed and reads
'!ERR*' while arming is disabled), and a refused arm is retried with a fresh edge.

The switch is only ever dropped for a retry while the FC itself says it is disarmed,
and never after the arm was confirmed, so a telemetry glitch cannot disarm a flying
drone. With no flight-mode telemetry at all the arm cannot be confirmed: the switch
stays high and the state reads 'unconfirmed'.

Each attempt is judged only by frames that arrived after its own arm edge: QUAD1 sends a
flight-mode frame only every 2-3 s (7 Oct bags), so a fixed 1 s freshness window turned 28 of
its 29 arms into 'unconfirmed' and a later '!ERR*' never triggered a retry.
"""

PRE_LOW_S = 0.3        # switch low before each arm edge
CONFIRM_S = 1.5        # an after-edge frame saying disarmed is acted on from here
CONFIRM_MAX_S = 4.0    # no after-edge frame by then: 'unconfirmed' (a sparse sender's gap)
EDGE_SETTLE_S = 0.2    # a frame must arrive this long after the edge to speak for it
RETRY_LOW_S = 0.5      # low time between attempts
MAX_TRIES = 2          # 2 x (0.3 + 4.0) + 0.5 s fits the fleet manager's 10 s arm window
MODE_FRESH_S = 1.0     # once armed: a flight-mode frame older than this is no evidence


def fc_mode_armed(mode):
    """True/False from a Betaflight CRSF flight-mode string, None when it says nothing."""
    mode = (mode or '').strip()
    if not mode or mode == 'UNKNOWN':
        return None
    return not mode.endswith('*')


class ArmSwitch:
    def __init__(self):
        self._phase = 'off'      # off | pre_low | high | retry_low | armed | failed | unconfirmed
        self._t = 0.0
        self._tries = 0
        self._last_mode = ''

    def update(self, want_armed, now, mode='', mode_t=None):
        """One tick. Returns (switch_high, state) with state one of 'disarmed', 'pending',
        'armed', 'unconfirmed: ...', 'failed: ...'."""
        fresh = mode_t is not None and now - mode_t <= MODE_FRESH_S
        says = fc_mode_armed(mode) if fresh else None
        if fresh and mode:
            self._last_mode = mode
        # what the FC said about THIS attempt: only a frame that arrived after its edge
        # (an 'armed' frame counts from the edge on: the switch was low before it; a 'disarmed'
        # frame only after EDGE_SETTLE_S, once the FC has had the edge)
        edge = self._t if self._phase in ('high', 'unconfirmed', 'failed') else None
        said = fc_mode_armed(mode) if mode_t is not None and edge is not None and mode_t > edge else None
        says_edge = (said if said is True or (said is False and mode_t >= edge + EDGE_SETTLE_S)
                     else None)
        if says_edge is not None and mode:
            self._last_mode = mode
        if not want_armed:
            self._phase, self._tries = 'off', 0
            return False, 'disarmed'
        if self._phase == 'off':
            self._phase, self._t = 'pre_low', now
        if self._phase == 'pre_low':
            if now - self._t < PRE_LOW_S:
                return False, 'pending'
            self._phase, self._t, self._tries = 'high', now, 1
        if self._phase == 'retry_low':
            if now - self._t < RETRY_LOW_S:
                return False, 'pending'
            self._phase, self._t = 'high', now
            self._tries += 1
        if self._phase in ('high', 'unconfirmed'):
            if says_edge is True:
                self._phase = 'armed'
            elif says_edge is False and now - self._t >= CONFIRM_S:
                if self._tries < MAX_TRIES:
                    self._phase, self._t = 'retry_low', now
                    return False, 'pending'
                self._phase = 'failed'
            elif now - self._t >= CONFIRM_MAX_S:
                self._phase = 'unconfirmed'
                return True, 'unconfirmed: no flight-mode frame since the arm edge'
            else:
                return True, 'pending'
        if self._phase == 'armed':
            if says is False:
                return True, f'failed: FC reports disarmed ({mode})'
            return True, 'armed'
        # failed: keep the switch high (a late arm still shows) until the command drops
        if says_edge is True:
            self._phase = 'armed'
            return True, 'armed'
        return True, f"failed: FC says '{self._last_mode}' after {self._tries} tries"
