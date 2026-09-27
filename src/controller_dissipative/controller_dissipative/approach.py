"""Tracker-flown approach for a rejoining drone (2026-09-25).

The newcomer's own OCP tracker flies it to the weld point on a reference that the
dissipative node publishes: a vertical climb to the clearance altitude first (the magnet
arm hangs 0.5 m below the body; a straight line from the floor swept it through the ring
and the rods, R0523), then a level transit over the live magnet-tip target, then straight
down until the tip sits on the target, then a hold that follows the target.
A rate-bounded carrot keeps every step bumpless for the tracker; the attach speed is the
locked 0.2 m/s.
"""
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

    def __init__(self, p_start, arm_len, dt, clearance=CLEARANCE_M, seek_m=0.0, direct=False):
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

    def step(self, target, target_vel=None):
        """target: live magnet-tip target (world); target_vel: its velocity (a moving ring),
        carried by the reference and fed forward after the climb. Returns (p_ref, v_ff, phase)."""
        t = np.asarray(target, float)
        v_t = np.zeros(3) if target_vel is None else np.asarray(target_vel, float)
        if self.phase != 'climb':
            self.p_ref = self.p_ref + v_t * self.dt
        body_on_target = t + np.array([0.0, 0.0, self.arm_len + TIP_STANDOFF_M])
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
            if self.seek_m > 0.0 and float(self.p_ref[2]) - (goal[2] - self._seek) < 0.005:
                self._seek = min(self.seek_m, self._seek + SEEK_VEL * self.dt)
            goal[2] -= self._seek
            self.p_ref, v_z = carrot_step(self.p_ref, goal, DESCENT_VEL, self.dt)
            v = v_xy + v_z
        if self.phase != 'climb':
            v = v + v_t
        return self.p_ref.copy(), v, self.phase
