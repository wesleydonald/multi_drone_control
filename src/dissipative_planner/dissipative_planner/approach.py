"""Tracker-flown approach for a rejoining drone (2026-09-25).

The newcomer's own OCP tracker flies it to the weld point on a reference that the
dissipative node publishes: a vertical climb to the clearance altitude first (the magnet
arm hangs 0.5 m below the body; a straight line from the floor swept it through the ring
and the rods, R0523), then a level transit over the live magnet-tip target, then straight
down until the tip sits on the target, then a hold that follows the target.
A rate-bounded carrot keeps every step bumpless for the tracker; the attach speed is the
locked 0.2 m/s.
"""
from collections import deque

import numpy as np

APPROACH_VEL = 0.20      # m/s toward the clearance point (locked: attach at 0.2 m/s)
DESCENT_VEL = 0.10       # m/s on the final vertical leg
CLEARANCE_M = 0.25       # m above the tip-on-target height before the descent
CAPTURE_XY_M = 0.05      # m: switch to the descent once the carrot is over the target
TIP_STANDOFF_M = -0.01   # m, relative to /attach_target/pose (rim point + 0.05 z): the
                         # magnet manager measures the tip to the rim point ITSELF (no z
                         # offset) with an 0.08 m weld radius, so the tip must end within
                         # ~0.05 of the ring plane: here 0.04 above it, 0.02 clear of the
                         # plate top (R0540: a 0.04 standoff on the +0.05 point left the tip
                         # 0.12 from the manager's point, no weld)
DIRECT_XY_M = 0.10       # m: 'already over the plate' for a direct descent
SEEK_VEL = 0.02          # m/s: below-target seek once the carrot sits on the target


# The measured-tip correction (card 2026-10-09_approach_tip_feedback rev 3; Wesley 9 Oct: calibrate + a continuous
# loop): z only, integrating only in a still hold, bounded, frozen on contact and on an invalid tip.
TIP_K = 0.5                     # 1/s on the filtered error: crossover ~0.42 rad/s, 10x below the rod (4.2 rad/s)
TIP_TAU_S = 1.5                 # s low-pass on the error, run only in the hold
TIP_RATE = 0.02                 # m/s on the correction
TIP_C_MIN, TIP_C_MAX = -0.06, 0.08    # m: at most 6 cm down (contact windup), 8 cm up
TIP_DB_IN, TIP_DB_IN_S, TIP_DB_OUT = 0.005, 2.0, 0.015   # deadband: in below 5 mm for 2 s, out above 15 mm
TIP_CONTACT_S, TIP_CONTACT_DC, TIP_CONTACT_RATIO = 2.0, 0.003, 0.3
HOLD_AT_GOAL_M, HOLD_AT_GOAL_S = 0.005, 1.0
HOLD_SPEED_IN, HOLD_SPEED_OUT = 0.05, 0.08   # m/s, the target's 1 s mean (rig 8 Oct: hover p50 0.02-0.03, circle 0.12)
TIP_BODY_MIN, TIP_BODY_MAX = 0.55, 0.63      # m, |tip - body| of a believable tip
TIP_XY_MAX, TIP_STEP_MAX, TIP_FRESH_S = 0.15, 0.03, 0.1
TIP_FROZEN_S = 0.2              # s: a bit-identical tip this long is frozen (still mocap noise never repeats so);
TIP_FROZEN_N = 64               # frames kept for the checks (~0.5 s at 120 Hz)


def tip_valid(now, frames):
    """frames: the last tip samples as (t, tip xyz, body xyz), oldest first. The tip passes as a measurement:
    fresh, at a believable distance and lean from the body, no jump, and not frozen (bit-identical for
    TIP_FROZEN_S: the body-moved test of rev 3 could not see a freeze in a still hold, R1340). Returns (ok, why)."""
    if not frames:
        return False, 'no tip'
    t, tip, body = frames[-1]
    tip, body = np.asarray(tip, float), np.asarray(body, float)
    if now - t > TIP_FRESH_S:
        return False, 'stale'
    d = tip - body
    if not TIP_BODY_MIN <= float(np.linalg.norm(d)) <= TIP_BODY_MAX:
        return False, 'distance'
    if float(np.hypot(d[0], d[1])) > TIP_XY_MAX:
        return False, 'lean'
    tips = [np.asarray(f[1], float) for f in frames]
    if any(float(np.linalg.norm(b - a)) > TIP_STEP_MAX for a, b in zip(tips, tips[1:])):
        return False, 'step'
    recent = [f for f in frames if t - f[0] <= TIP_FROZEN_S]
    if len(recent) >= 2 and t - frames[0][0] >= TIP_FROZEN_S \
            and all(np.array_equal(np.asarray(f[1], float), tip) for f in recent):
        return False, 'frozen'
    return True, ''


class TipLoop:
    """The slow z correction of the body goal from the measured tip. `apply` False: the hold and the error are
    still computed and logged (the switch-off arms), the correction stays 0."""

    def __init__(self, dt, apply=True):
        self.dt = float(dt)
        self.apply = bool(apply)
        self.c = 0.0
        self.e = self.e_f = None
        self.hold = self.contact = False
        self.reason = 'no hold'
        self._at_goal = 0.0
        self._targets = deque(maxlen=int(round(1.0 / self.dt)) + 1)
        self._e_hist = deque(maxlen=max(1, int(round(0.5 / self.dt))))
        n = int(round(TIP_CONTACT_S / self.dt)) + 1
        self._c_hist, self._tip_hist = deque(maxlen=n), deque(maxlen=n)
        self._db, self._db_t = False, 0.0

    def _speed(self):
        if len(self._targets) < self._targets.maxlen:
            return None
        return float(np.linalg.norm(self._targets[-1] - self._targets[0])) / ((len(self._targets) - 1) * self.dt)

    def update(self, at_goal, target, tip_z, setpoint_z, tip_ok):
        """One planner tick: at_goal (the carrot within HOLD_AT_GOAL_M of its descent goal), the live target
        (its 1 s mean speed gates the hold), the measured tip z and the setpoint z, the validity verdict.
        Returns c (m, added to the body goal's z)."""
        self._targets.append(np.asarray(target, float))
        speed = self._speed()
        self._at_goal = self._at_goal + self.dt if at_goal else 0.0
        self.e = None if tip_z is None else float(setpoint_z) - float(tip_z)
        if self.e is not None and tip_ok:
            self._e_hist.append(self.e)
        if not self.hold:
            if (self._at_goal >= HOLD_AT_GOAL_S and speed is not None and speed < HOLD_SPEED_IN
                    and tip_ok and self._e_hist):
                self.hold = True
                self.e_f = float(np.mean(self._e_hist))      # no filter lag carried in from the descent
                self._c_hist.clear()
                self._tip_hist.clear()
        elif speed is not None and speed > HOLD_SPEED_OUT:
            self.hold, self.e_f, self._db, self._db_t = False, None, False, 0.0
        if not self.hold:
            self.reason = 'no hold'
            return self.c
        if not tip_ok or self.e is None:
            self.reason = 'tip invalid'
            return self.c
        self.e_f += self.dt / (TIP_TAU_S + self.dt) * (self.e - self.e_f)
        if self._db:
            if abs(self.e_f) > TIP_DB_OUT:
                self._db, self._db_t = False, 0.0
        else:
            self._db_t = self._db_t + self.dt if abs(self.e_f) < TIP_DB_IN else 0.0
            self._db = self._db_t >= TIP_DB_IN_S
        self._c_hist.append(self.c)
        self._tip_hist.append(float(tip_z))
        if not self.contact and len(self._c_hist) == self._c_hist.maxlen:
            dc = self._c_hist[-1] - self._c_hist[0]
            if abs(dc) > TIP_CONTACT_DC and abs(self._tip_hist[-1] - self._tip_hist[0]) < TIP_CONTACT_RATIO * abs(dc):
                self.contact = True         # the tip stops following c: contact, a catch or a slack tether
        if self.contact:
            self.reason = 'contact'
        elif self._db:
            self.reason = 'deadband'
        elif not self.apply:
            self.reason = 'off'
        else:
            step = float(np.clip(TIP_K * self.e_f * self.dt, -TIP_RATE * self.dt, TIP_RATE * self.dt))
            self.c = float(np.clip(self.c + step, TIP_C_MIN, TIP_C_MAX))
            self.reason = 'integrating'
        return self.c


def body_to_tip(rod_len, pivot_offset_z):
    """Distance from the drone body (what the tracker flies) down to the magnet tip: the rod hangs
    from a pivot pivot_offset_z below the body (negative on the rig and twin, -0.04; 0 on legacy)."""
    return float(rod_len) - float(pivot_offset_z)


def carrot_step(p_ref, goal, v_max, dt):
    """Move p_ref toward goal by at most v_max*dt. Returns (p_new, v_ff)."""
    p_ref = np.asarray(p_ref, float)
    d = np.asarray(goal, float) - p_ref
    dist = float(np.linalg.norm(d))
    if dist < 1e-9:
        return p_ref.copy(), np.zeros(3)
    step = min(dist, v_max * dt)
    v = d / dist * (step / dt if dt > 0 else 0.0)
    return p_ref + d / dist * step, v


class ApproachProfile:
    """State of one approach: 'climb' (to the clearance altitude), 'transit' (level, over
    the target), then 'descend'."""

    def __init__(self, p_start, arm_len, dt, clearance=CLEARANCE_M, seek_m=0.0, direct=False,
                 tip_loop=None, tip_dz=0.0):
        self.p_ref = np.asarray(p_start, float).copy()
        self.arm_len = float(arm_len)
        self.dt = float(dt)
        self.clearance = float(clearance)
        # a drone hovering a few cm above its reference parks the tip just outside the
        # weld radius (R0653: 2.8 cm high for 135 s); seek_m lets the reference creep
        # this far below the tip-on-target height until the weld ends the approach
        self.seek_m = max(0.0, float(seek_m))
        self._seek = 0.0
        self.phase = 'climb'
        # a partner hands the drone over hovering above the plate (ATTACH_READY): 'climb' to
        # the clearance height first kicked its reference up 7.5 cm (R0687, Tejen). direct=True
        # descends from where it is when it is already over the plate, within the clearance.
        self._direct = bool(direct)
        self.tip = tip_loop                 # TipLoop or None
        self.tip_dz = float(tip_dz)         # the calibrated setpoint above today's (m)

    def step(self, target, target_vel=None, tip_z=None, tip_ok=False):
        """target: live magnet-tip target (world); target_vel: its velocity (a moving ring),
        carried by the reference and fed forward after the climb. Returns (p_ref, v_ff, phase)."""
        t = np.asarray(target, float)
        v_t = np.zeros(3) if target_vel is None else np.asarray(target_vel, float)
        if self.phase != 'climb':
            self.p_ref = self.p_ref + v_t * self.dt
        body_on_target = t + np.array([0.0, 0.0, self.arm_len + TIP_STANDOFF_M + self.tip_dz])
        z_clear = float(body_on_target[2] + self.clearance)
        if self._direct:
            self._direct = False            # decided on the first target only
            above = float(self.p_ref[2] - body_on_target[2])
            if (float(np.linalg.norm((self.p_ref - body_on_target)[:2])) < DIRECT_XY_M
                    and 0.0 <= above <= self.clearance + 0.05):
                self.phase = 'descend'
        if self.phase == 'climb':
            goal = np.array([self.p_ref[0], self.p_ref[1], z_clear])
            self.p_ref, v = carrot_step(self.p_ref, goal, APPROACH_VEL, self.dt)
            if abs(float(self.p_ref[2] - z_clear)) < CAPTURE_XY_M:
                self.phase = 'transit'
        elif self.phase == 'transit':
            goal = np.array([body_on_target[0], body_on_target[1], z_clear])
            self.p_ref, v = carrot_step(self.p_ref, goal, APPROACH_VEL, self.dt)
            if float(np.linalg.norm((self.p_ref - goal)[:2])) < CAPTURE_XY_M:
                self.phase = 'descend'
        else:
            # keep the xy on the moving target at transit speed, descend at the slow rate
            goal_xy = body_on_target.copy()
            goal_xy[2] = self.p_ref[2]
            self.p_ref, v_xy = carrot_step(self.p_ref, goal_xy, APPROACH_VEL, self.dt)
            goal = body_on_target.copy()
            if self.tip is not None:
                at_goal = float(np.linalg.norm(self.p_ref - goal - [0.0, 0.0, self.tip.c])) < HOLD_AT_GOAL_M
                goal[2] += self.tip.update(at_goal, t, tip_z, float(t[2]) + TIP_STANDOFF_M + self.tip_dz, tip_ok)
            if self.seek_m > 0.0 and float(self.p_ref[2]) - (goal[2] - self._seek) < 0.005:
                self._seek = min(self.seek_m, self._seek + SEEK_VEL * self.dt)
            goal[2] -= self._seek
            self.p_ref, v_z = carrot_step(self.p_ref, goal, DESCENT_VEL, self.dt)
            v = v_xy + v_z
        if self.phase != 'climb':
            v = v + v_t
        return self.p_ref.copy(), v, self.phase
