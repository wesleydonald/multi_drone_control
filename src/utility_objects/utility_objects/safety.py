"""
safety.py — flight envelope checks (finding F3).

A pure class: no ROS, no I/O, no clock. It is handed numbers and returns a verdict,
which is what makes it unit-testable and usable inside the offline harnesses. The ROS
wiring lives in the tracker; see docs/design/fleet_safety.md for the design and the
reasoning behind the policy.

POLICY (Wesley, 2026-08-04): **any FAULT disarms the whole fleet, immediately.** There
is no descent manoeuvre. The tracker calls its existing `cb.disarm()`, which publishes
arming feedback, which the fleet manager already propagates to every drone. This reuses
a path that is proven in flight rather than adding a parallel one.

NO GEOFENCE, BY DECISION (Wesley, 2026-08-04)
---------------------------------------------
Position/altitude bounds checking was implemented and then **deliberately removed**: the
operator is at the cage with a kill switch throughout testing, and a geofence cannot
catch anything a ready human cannot. Do not re-add one without asking -- it was not an
oversight.

What is kept is what a human CANNOT react to in time, or cannot see at all:

  * reference staleness -- if the planner dies, the drones keep flying the last
    reference and drift with nothing on screen to indicate why;
  * tilt and speed -- the attach-runaway signature, which develops in ~1 s.

WARN vs FAULT
-------------
WARN is logged and shown, and changes nothing. FAULT disarms. Battery is deliberately
WARN-only: voltage sags hard under thrust transients, so a naive threshold produces
false positives mid-manoeuvre, and a safety layer that cries wolf gets switched off.
The operator is standing next to the cage and can call it.
"""
from dataclasses import dataclass, field
import math

OK = 'OK'
WARN = 'WARN'
FAULT = 'FAULT'


@dataclass
class EnvelopeLimits:
    """Every threshold. Values come from the parameter profile, never from code --
    the lab volume is not the sim volume."""

    # No geofence -- see the module docstring. The operator is the out-of-bounds
    # failsafe.

    # Attitude, degrees. 60 for the payload because the ring-attach experiments
    # legitimately reach 10-30 deg and the boundary sweep goes higher; a real
    # runaway goes past 90.
    max_tilt_deg: float = 60.0
    max_payload_tilt_deg: float = 60.0
    warn_tilt_deg: float = 40.0

    max_speed: float = 3.0                # m/s
    ref_timeout_s: float = 1.0            # reference staleness before it is a fault
    ref_warn_s: float = 0.5

    warn_battery_v: float = 15.0          # WARN ONLY -- never faults

    # Faults are only meaningful once the drone is actually flying: on the stands a
    # drone can sit tilted, and the reference is legitimately absent before takeoff.
    require_airborne: bool = True

    # Consecutive violating samples before a fault latches. At 50 Hz, 3 samples =
    # 60 ms. One bad mocap frame should not drop the fleet out of the sky.
    fault_debounce: int = 3


@dataclass
class Verdict:
    level: str = OK
    reason: str = ''
    detail: dict = field(default_factory=dict)

    @property
    def is_fault(self):
        return self.level == FAULT


def tilt_deg_from_quat(q):
    """Angle between body-z and world-z, degrees. q = (w, x, y, z).

    Body z-axis in world coords is the third column of the rotation matrix:
        R[:,2] = (2(xz + wy), 2(yz - wx), 1 - 2(x^2 + y^2))
    The tilt is the angle of that vector from vertical, so only its z-component is
    needed. Sign-invariant in q, which matters because mocap can hand over either
    of the two quaternions representing the same attitude.
    """
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return float('nan')
    w, x, y, z = w / n, x / n, y / n, z / n
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


class EnvelopeChecker:
    """Stateful only in its debounce counter. `check()` is otherwise a pure function
    of its arguments."""

    def __init__(self, limits: EnvelopeLimits = None):
        self.limits = limits or EnvelopeLimits()
        self._violations = 0
        self._latched = None      # once faulted, stay faulted until reset()

    def reset(self):
        """Call on re-arm. A latched fault must not survive into the next flight
        silently, and must not be cleared by the condition merely going away."""
        self._violations = 0
        self._latched = None

    def check(self, *, velocity=None, quat=None, payload_quat=None,
              ref_age_s=None, battery_v=None, airborne=False):
        """Return a Verdict. Any argument may be None (signal not available yet),
        in which case the checks that need it are skipped -- a missing signal is not
        by itself a fault, because that would make every startup a fault.

        Staleness of a signal that SHOULD be present is caught elsewhere (the pose
        timeout watchdog for mocap, ref_age_s for the planner)."""
        if self._latched is not None:
            return self._latched

        lim = self.limits
        warn = None

        if lim.require_airborne and not airborne:
            # On the ground: report warnings, never fault.
            v = self._survey(velocity, quat, payload_quat,
                             ref_age_s, battery_v, ground=True)
            return v if v.level == WARN else Verdict(OK)

        v = self._survey(velocity, quat, payload_quat,
                         ref_age_s, battery_v, ground=False)

        if v.level == FAULT:
            self._violations += 1
            if self._violations >= lim.fault_debounce:
                self._latched = v
                return v
            # Not yet debounced -- surface it as a warning so it appears in the log
            # before it latches, which makes post-hoc diagnosis much easier.
            return Verdict(WARN, f'(debouncing {self._violations}/'
                                 f'{lim.fault_debounce}) {v.reason}', v.detail)
        self._violations = 0
        return v if v.level == WARN else (warn or Verdict(OK))

    # ── individual checks ────────────────────────────────────────────────────
    def _survey(self, velocity, quat, payload_quat, ref_age_s, battery_v, ground):
        lim = self.limits
        warn = None

        for name, q, cap in (('drone', quat, lim.max_tilt_deg),
                             ('payload', payload_quat, lim.max_payload_tilt_deg)):
            if q is None:
                continue
            tilt = tilt_deg_from_quat(q)
            if math.isnan(tilt):
                continue
            if tilt > cap and not ground:
                return Verdict(FAULT, f'{name} tilt {tilt:.1f} deg > {cap:.1f}',
                               {'check': 'tilt', 'who': name, 'value': tilt})
            if tilt > lim.warn_tilt_deg and warn is None:
                warn = Verdict(WARN, f'{name} tilt {tilt:.1f} deg',
                               {'check': 'tilt_warn', 'who': name, 'value': tilt})

        if velocity is not None:
            speed = math.sqrt(sum(float(c) ** 2 for c in velocity))
            if speed > lim.max_speed and not ground:
                return Verdict(FAULT, f'speed {speed:.2f} m/s > {lim.max_speed:.2f}',
                               {'check': 'speed', 'value': speed})

        if ref_age_s is not None:
            if ref_age_s > lim.ref_timeout_s and not ground:
                return Verdict(FAULT, f'reference stale by {ref_age_s:.2f} s',
                               {'check': 'ref_stale', 'value': ref_age_s})
            if ref_age_s > lim.ref_warn_s and warn is None:
                warn = Verdict(WARN, f'reference {ref_age_s:.2f} s old',
                               {'check': 'ref_warn', 'value': ref_age_s})

        # Battery: WARN ONLY, by decision. Never returns FAULT.
        if battery_v is not None and 0.0 < battery_v < lim.warn_battery_v:
            if warn is None:
                warn = Verdict(WARN, f'battery {battery_v:.2f} V',
                               {'check': 'battery', 'value': battery_v})

        return warn or Verdict(OK)
