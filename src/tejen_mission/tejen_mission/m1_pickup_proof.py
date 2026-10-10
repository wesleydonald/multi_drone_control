"""Measured co-motion proof for M1 pickup, independent of ROS and attachment guesses."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PickupProofConfig:
    minimum_body_rise_m: float = 0.06
    minimum_object_rise_m: float = 0.02
    maximum_contact_drift_m: float = 0.015
    relative_speed_mps: float = 0.03
    proof_dwell_s: float = 0.50
    loss_contact_drift_m: float = 0.045
    loss_dwell_s: float = 0.12

    def __post_init__(self) -> None:
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("pickup proof values must be finite and non-negative")
        if self.loss_contact_drift_m <= self.maximum_contact_drift_m:
            raise ValueError("loss drift must exceed proof contact drift")
        if self.minimum_body_rise_m <= 0.0 or self.minimum_object_rise_m <= 0.0:
            raise ValueError("pickup proof requires positive measured movement")


@dataclass(frozen=True)
class PickupProofDecision:
    proven: bool
    failed: bool
    reason: str
    body_rise_m: float = 0.0
    object_rise_m: float = 0.0
    contact_drift_m: float = 0.0
    relative_speed_mps: float = 0.0
    proof_dwell_s: float = 0.0
    loss_dwell_s: float = 0.0


class PickupProofTracker:
    """Latch a real pickup only after an upward, low-relative-motion object lift.

    The first portion of the existing LIFT_OBJECT reference supplies the proof
    manoeuvre. Proximity is only a candidate; no timer alone proves attachment.
    A separated object fails after geometry-backed dwell, not mere uncertainty.
    """

    def __init__(self, config: PickupProofConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.start_body_z: Optional[float] = None
        self.start_object_z: Optional[float] = None
        self.start_relative: Optional[np.ndarray] = None
        self.last_relative: Optional[np.ndarray] = None
        self.last_sample_time_s: Optional[float] = None
        self.filtered_relative_velocity = np.zeros(3, dtype=float)
        self.proof_since_s: Optional[float] = None
        self.loss_since_s: Optional[float] = None
        self.proven = False
        self.failed = False
        self.last_decision = PickupProofDecision(False, False, "not_started")

    def step(
        self,
        *,
        now_s: float,
        body_position,
        magnet_position,
        object_position,
        fresh: bool,
        magnet_on: bool,
        candidate: bool,
    ) -> PickupProofDecision:
        if not magnet_on or not candidate:
            self.reset()
            self.last_decision = PickupProofDecision(False, False, "no_magnetic_candidate")
            return self.last_decision
        if not fresh:
            # Keep the contact baseline, but never earn dwell while data is stale.
            self.last_relative = None
            self.last_sample_time_s = None
            self.proof_since_s = None
            self.loss_since_s = None
            self.last_decision = PickupProofDecision(self.proven, self.failed, "stale_evidence")
            return self.last_decision

        now_s = float(now_s)
        body = np.asarray(body_position, dtype=float).reshape(3)
        magnet = np.asarray(magnet_position, dtype=float).reshape(3)
        obj = np.asarray(object_position, dtype=float).reshape(3)
        if not math.isfinite(now_s) or not all(np.all(np.isfinite(x)) for x in (body, magnet, obj)):
            raise ValueError("pickup proof requires finite timestamp and poses")
        relative = magnet - obj
        if self.start_body_z is None:
            self.start_body_z = float(body[2])
            self.start_object_z = float(obj[2])
            self.start_relative = relative.copy()
        if self.last_sample_time_s is not None and now_s <= self.last_sample_time_s:
            return self.last_decision
        if self.last_relative is not None and self.last_sample_time_s is not None:
            dt = now_s - self.last_sample_time_s
            self.filtered_relative_velocity = (
                0.35 * (relative - self.last_relative) / dt
                + 0.65 * self.filtered_relative_velocity
            )
        self.last_relative = relative.copy()
        self.last_sample_time_s = now_s

        body_rise = float(body[2] - self.start_body_z)
        object_rise = float(obj[2] - self.start_object_z)
        drift = float(np.linalg.norm(relative - self.start_relative))
        relative_speed = float(np.linalg.norm(self.filtered_relative_velocity))
        config = self.config
        proof_condition = (
            body_rise >= config.minimum_object_rise_m
            and object_rise >= config.minimum_object_rise_m
            and drift <= config.maximum_contact_drift_m
            and relative_speed <= config.relative_speed_mps
        )
        loss_condition = (
            body_rise >= config.minimum_body_rise_m
            and drift >= config.loss_contact_drift_m
        )
        self.proof_since_s = (
            now_s if proof_condition and self.proof_since_s is None
            else self.proof_since_s if proof_condition else None
        )
        self.loss_since_s = (
            now_s if loss_condition and self.loss_since_s is None
            else self.loss_since_s if loss_condition else None
        )
        proof_dwell = 0.0 if self.proof_since_s is None else now_s - self.proof_since_s
        loss_dwell = 0.0 if self.loss_since_s is None else now_s - self.loss_since_s
        if loss_dwell >= config.loss_dwell_s and loss_condition:
            self.failed = True
        if (
            body_rise >= config.minimum_body_rise_m
            and proof_condition
            and proof_dwell >= config.proof_dwell_s
            and not self.failed
        ):
            self.proven = True
        reason = (
            "measured_separation" if self.failed else
            "proven_co_motion" if self.proven else
            "object_not_airborne" if object_rise < config.minimum_object_rise_m else
            "relative_motion" if relative_speed > config.relative_speed_mps else
            "contact_drift" if drift > config.maximum_contact_drift_m else
            "proof_dwell"
        )
        self.last_decision = PickupProofDecision(
            self.proven, self.failed, reason, body_rise, object_rise, drift,
            relative_speed, proof_dwell, loss_dwell,
        )
        return self.last_decision
