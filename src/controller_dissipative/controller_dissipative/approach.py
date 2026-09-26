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

    def __init__(self, p_start, arm_len, dt, clearance=CLEARANCE_M):
        self.p_ref = np.asarray(p_start, float).copy()
        self.arm_len = float(arm_len)
        self.dt = float(dt)
        self.clearance = float(clearance)
        self.phase = 'climb'

    def step(self, target):
        """target: live magnet-tip target (world). Returns (p_ref, v_ff, phase)."""
        t = np.asarray(target, float)
        body_on_target = t + np.array([0.0, 0.0, self.arm_len + TIP_STANDOFF_M])
        z_clear = float(body_on_target[2] + self.clearance)
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
            self.p_ref, v_z = carrot_step(self.p_ref, body_on_target, DESCENT_VEL, self.dt)
            v = v_xy + v_z
        return self.p_ref.copy(), v, self.phase
