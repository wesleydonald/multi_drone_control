"""Pure opt-in M2 contact/proof helpers, not a second mission state machine."""
from dataclasses import dataclass
import math
import numpy as np

POLICY = "measured_contact_15deg"
LEGACY = "legacy"

class QuietMissTracker:
    """One-shot accumulated eligible-time timer; caller owns recovery."""

    def __init__(self, *, wait_s=10., improvement_m=.005):
        if not all(math.isfinite(v) and v > 0 for v in (wait_s, improvement_m)):
            raise ValueError("quiet miss wait and improvement must be finite and positive")
        self.wait_s = wait_s
        self.improvement_m = improvement_m
        self.reset()

    def reset(self):
        self.key = self.since = self.baseline = None
        self.elapsed_s = 0.
        self.fired = False

    def pause(self):
        """Retain earned time/baseline, but never credit the skipped interval."""
        self.since = None

    def step(self, *, now, xy_error_m, eligible, key):
        if key != self.key:
            self.reset()
        self.key = key
        if not eligible or not math.isfinite(now) or not math.isfinite(xy_error_m):
            self.pause()
            return False
        if self.since is not None and now < self.since:
            self.reset()
            self.key = key
        if self.fired:
            return False
        if self.baseline is None or self.baseline - xy_error_m >= self.improvement_m:
            self.baseline = xy_error_m
            self.elapsed_s = 0.
        elif self.since is not None:
            self.elapsed_s += max(0., now - self.since)
        self.since = now
        self.fired = self.elapsed_s >= self.wait_s
        return self.fired


def validate_policy(value):
    if value not in (LEGACY, POLICY):
        raise ValueError("attachment_policy must be legacy or measured_contact_15deg")
    return value

@dataclass(frozen=True)
class ContactProofConfig:
    xy_m: float = .025
    normal_m: float = .015
    speed_mps: float = .03
    contact_dwell_s: float = .30
    proof_dwell_s: float = .50
    drift_m: float = .008
    angle_deg: float = 15.
    angle_tolerance_deg: float = 2.
    loss_xy_m: float = .045
    loss_normal_m: float = .045
    loss_dwell_s: float = .12
    initial_contact_max_normal_m: float = 0.0
    proof_xy_m: float | None = None

class ContactProofEvidence:
    """Freshness pauses evidence; only fresh geometric separation establishes loss."""
    def __init__(self, config=None):
        self.config = config or ContactProofConfig()
        self.initial_contact = self.confirmed = self.lost = False
        self.contact_seed = self.anchor_seed = None
        self.contact_since = self.proof_since = self.loss_since = None
        self.candidate_condition = self.proof_condition = False
        self.candidate_dwell = self.proof_dwell = self.loss_dwell = 0.
        self.drift = 0.
        self.angle_error_deg = float("nan")

    def update(self, *, now, contact, anchor, speed, fresh, magnet_on,
               proving=False, motion_complete=False):
        c=self.config
        contact=np.asarray(contact, dtype=float)
        anchor=np.asarray(anchor, dtype=float)
        self.candidate_condition = self.proof_condition = False
        self.candidate_dwell = self.proof_dwell = self.loss_dwell = 0.
        if not fresh or not np.all(np.isfinite(contact)) or not np.all(np.isfinite(anchor)):
            self.contact_since = self.proof_since = self.loss_since = None
            return self
        xy=float(np.linalg.norm(contact[:2])); normal=float(contact[2])
        separated=xy>c.loss_xy_m or abs(normal)>c.loss_normal_m
        # History is deliberately not erased by OFF. Release/reset is the planner's job.
        if self.initial_contact and separated:
            if self.loss_since is None: self.loss_since=now
            self.loss_dwell=max(0.,now-self.loss_since)
            if self.loss_dwell>=c.loss_dwell_s:
                self.lost=True
                self.confirmed=False
        else:
            self.loss_since=None
        self.candidate_condition=bool(magnet_on and not self.lost and xy<=c.xy_m
            and -c.normal_m<=normal<=c.initial_contact_max_normal_m
            and math.isfinite(speed) and speed<=c.speed_mps)
        if self.candidate_condition:
            if self.contact_since is None: self.contact_since=now
            self.candidate_dwell=max(0.,now-self.contact_since)
            if not self.initial_contact and self.candidate_dwell>=c.contact_dwell_s:
                self.initial_contact=True
                self.contact_seed=contact.copy()
                self.anchor_seed=anchor.copy()
        else:
            self.contact_since=None
        if self.initial_contact:
            self.drift=float(np.linalg.norm(contact-self.contact_seed))
            vector=anchor-self.contact_seed
            length=float(np.linalg.norm(vector))
            desired=np.array([math.sin(math.radians(c.angle_deg)),0.,math.cos(math.radians(c.angle_deg))])
            self.angle_error_deg=math.degrees(math.acos(float(np.clip(np.dot(vector,desired)/length,-1,1)))) if length>1e-9 else float("nan")
            self.proof_condition=bool(proving and magnet_on and not self.lost
                and xy<=(c.xy_m if c.proof_xy_m is None else c.proof_xy_m) and abs(normal)<=c.normal_m and speed<=c.speed_mps
                and self.drift<=c.drift_m and math.isfinite(self.angle_error_deg)
                and self.angle_error_deg<=c.angle_tolerance_deg and motion_complete)
        if self.proof_condition:
            if self.proof_since is None: self.proof_since=now
            self.proof_dwell=max(0.,now-self.proof_since)
            if self.proof_dwell>=c.proof_dwell_s: self.confirmed=True
        else:
            self.proof_since=None
        return self

class ProofArc:
    """Plate-frame anchor arc, with a continuous zero-velocity entry bridge."""
    def __init__(self, contact, anchor, reference_anchor, *, angle_deg=15.,
                 duration_s=10., speed_mps=.03):
        self.contact=np.asarray(contact,dtype=float).copy()
        self.anchor=np.asarray(anchor,dtype=float).copy()
        self.reference_anchor=np.asarray(reference_anchor,dtype=float).copy()
        self.radius=float(np.linalg.norm(self.anchor-self.contact))
        if not all(np.all(np.isfinite(x)) for x in (self.contact,self.anchor,self.reference_anchor)) or self.radius<=1e-6:
            raise ValueError("proof arc requires finite nonzero contact-to-anchor geometry")
        if (not all(math.isfinite(v) for v in (speed_mps, duration_s, angle_deg))
                or speed_mps<=0 or duration_s<=0 or not 0<angle_deg<90):
            raise ValueError("proof arc time and speed must be positive")
        self.start=(self.anchor-self.contact)/self.radius
        self.goal=np.array([math.sin(math.radians(angle_deg)),0.,math.cos(math.radians(angle_deg))])
        self.angle=math.acos(float(np.clip(np.dot(self.start,self.goal),-1,1)))
        self.bridge_duration=max(1.,1.875*float(np.linalg.norm(self.anchor-self.reference_anchor))/speed_mps)
        self.arc_duration=max(duration_s,1.875*self.radius*self.angle/speed_mps)
        self.duration=self.bridge_duration+self.arc_duration

    @staticmethod
    def blend(u):
        u=float(np.clip(u,0.,1.))
        return u*u*u*(10.+u*(-15.+6.*u))

    def position(self, elapsed):
        if elapsed<=self.bridge_duration:
            s=self.blend(elapsed/self.bridge_duration)
            return (1.-s)*self.reference_anchor+s*self.anchor
        s=self.blend((elapsed-self.bridge_duration)/self.arc_duration)
        if self.angle<1e-8: direction=self.goal
        else: direction=(math.sin((1.-s)*self.angle)*self.start+math.sin(s*self.angle)*self.goal)/math.sin(self.angle)
        return self.contact+self.radius*direction

class RetryBudget:
    """Total attempt counter; manual requests never erase the history."""
    def __init__(self, max_attempts=3):
        self.attempt=1
        self.max_attempts=int(max_attempts)
        self.awaiting_operator=False

    def fail(self):
        if self.attempt>=self.max_attempts:
            self.awaiting_operator=True
            return False
        self.attempt+=1
        return True

    def manual_retry(self):
        self.attempt+=1
        self.awaiting_operator=False


@dataclass(frozen=True)
class ParkingConfig:
    max_radius_m: float = .50
    slack_margin_m: float = .02
    disarm_height_above_rest_m: float = .02
    speed_mps: float = .10
    final_speed_mps: float = .04
    final_height_band_m: float = .15

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0 or (name != 'slack_margin_m' and value == 0):
                raise ValueError(f'parking.{name} must be finite and positive (slack may be zero)')
        if self.final_speed_mps > self.speed_mps:
            raise ValueError('parking final speed must not exceed cruise speed')


class ParkingArc:
    """Frozen-world shrinking arc; body reference, not a physical slack-cable model.

    The anchor rotates by spherical interpolation while radius decreases linearly
    in a quintic progress variable. Analytic P/V/A share exactly that progress.
    A conservative derivative bound sets duration for both speed caps.
    """
    def __init__(self, contact, start_body, body_to_anchor, outward, *,
                 measured_reach_m, resting_body_z, config=None):
        self.config = config or ParkingConfig()
        self.contact, self.start_body, self.body_to_anchor = (
            np.asarray(x, dtype=float).copy() for x in (contact, start_body, body_to_anchor))
        direction = np.asarray(outward, dtype=float).copy()
        values = (self.contact, self.start_body, self.body_to_anchor, direction)
        if any(x.shape != (3,) or not np.all(np.isfinite(x)) for x in values):
            raise ValueError('parking requires finite 3D geometry')
        if not math.isfinite(measured_reach_m) or not math.isfinite(resting_body_z):
            raise ValueError('parking requires finite measured reach and resting height')
        direction[2] = 0.
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            raise ValueError('parking outward direction must have horizontal extent')
        direction /= norm
        start = self.start_body + self.body_to_anchor - self.contact
        self.start_radius = float(np.linalg.norm(start))
        self.end_radius = min(self.config.max_radius_m,
                              measured_reach_m-self.config.slack_margin_m, self.start_radius)
        height = resting_body_z+self.body_to_anchor[2]-self.contact[2]
        if self.end_radius <= abs(height) or self.start_body[2] <= resting_body_z:
            raise ValueError('parking endpoint is outside reach or not below entry')
        end = math.sqrt(self.end_radius**2-height**2)*direction + np.array([0., 0., height])
        self.d0 = start/self.start_radius
        df = end/self.end_radius
        self.angle = math.acos(float(np.clip(self.d0 @ df, -1., 1.)))
        if not 1e-8 < self.angle < math.pi-1e-8:
            raise ValueError('parking arc directions must define a nondegenerate sweep')
        self.tangent = (df-math.cos(self.angle)*self.d0)/math.sin(self.angle)
        self.delta_radius = self.end_radius-self.start_radius
        self.cutoff_z = resting_body_z+self.config.disarm_height_above_rest_m
        self.duration = 1.
        # A sampled geometric sanity check, not collision certification.
        for q in np.linspace(0., 1., 257):
            if self._geometry(q)[1][2] > 1e-9:
                raise ValueError('parking geometry must descend monotonically')
        # |dp/dq| <= |dr/dq| + r0*angle, and max dq/du = 1.875.
        bound = abs(self.delta_radius)+self.start_radius*self.angle
        self.duration = max(1., 1.875*bound/self.config.speed_mps)
        low_z = self.cutoff_z+self.config.final_height_band_m
        lo, hi = 0., 1.
        for _ in range(50):
            mid = (lo+hi)/2.
            if self._geometry(ProofArc.blend(mid))[0][2] > low_z:
                lo = mid
            else:
                hi = mid
        u = max(.5, lo)  # quintic speed decreases monotonically after u=.5
        self.duration = max(self.duration, 30*u*u*(1-u)**2*bound/self.config.final_speed_mps)

    def _geometry(self, q):
        theta = self.angle*q
        d = math.cos(theta)*self.d0 + math.sin(theta)*self.tangent
        dq = self.angle*(-math.sin(theta)*self.d0 + math.cos(theta)*self.tangent)
        radius = self.start_radius+self.delta_radius*q
        p = self.contact+radius*d-self.body_to_anchor
        first = self.delta_radius*d+radius*dq
        second = 2*self.delta_radius*dq-radius*self.angle**2*d
        return p, first, second

    def sample(self, elapsed):
        u = float(np.clip(elapsed/self.duration, 0., 1.))
        p, first, second = self._geometry(ProofArc.blend(u))
        qdot = 30*u*u*(1-u)**2/self.duration
        qddot = (60*u-180*u*u+120*u**3)/self.duration**2
        return p, first*qdot, second*qdot*qdot+first*qddot
