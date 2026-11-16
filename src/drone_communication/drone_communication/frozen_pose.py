"""A lost rigid body keeps arriving with its last pose (the mocap PC's forwarder has no validity flag), so the pose
watchdogs, which read arrival times, never fire: RIG-1111-fm9c_6, QUAD4 flew 4.9 s on a pose frozen where motion
capture lost it. A tracked body never repeats exactly, at rest included (11 Nov recordings: 0.0 % exact repeats). So a
pose unchanged for longer than the window is withheld until it changes, and the watchdogs see the loss they already
handle. Card: docs/experiments/2026-11-11_frozen_pose_guard.md."""


class FrozenPoseFilter:
    """allow(body, pose, now) -> publish this packet or not. window_s <= 0 turns it off."""

    def __init__(self, window_s):
        self.window_s = float(window_s)
        self._pose = {}
        self._since = {}
        self.withheld = set()
        self.events = []          # (body, 'frozen' | 'live', seconds unchanged), drained by the node for its log

    def allow(self, body, pose, now):
        if self.window_s <= 0.0:
            return True
        if self._pose.get(body) != pose:
            if body in self.withheld:
                self.withheld.discard(body)
                self.events.append((body, 'live', now - self._since[body]))
            self._pose[body] = pose
            self._since[body] = now
            return True
        if now - self._since[body] <= self.window_s:
            return True
        if body not in self.withheld:
            self.withheld.add(body)
            self.events.append((body, 'frozen', now - self._since[body]))
        return False
