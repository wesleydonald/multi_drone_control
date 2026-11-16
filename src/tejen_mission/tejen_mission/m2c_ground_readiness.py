"""Small ROS-independent readiness helpers for M2C commissioning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class ContinuousSettleGate:
    """Require one condition to remain true continuously for a dwell interval."""

    dwell_s: float
    _true_since_s: Optional[float] = None

    def __post_init__(self) -> None:
        self.dwell_s = max(0.0, float(self.dwell_s))

    @property
    def true_since_s(self) -> Optional[float]:
        return self._true_since_s

    def reset(self) -> None:
        self._true_since_s = None

    def update(self, *, condition: bool, now_s: float) -> bool:
        now = float(now_s)
        if not condition:
            self._true_since_s = None
            return False
        if self._true_since_s is None or now < self._true_since_s:
            self._true_since_s = now
        return now - self._true_since_s >= self.dwell_s

    def elapsed_s(self, now_s: float) -> float:
        if self._true_since_s is None:
            return 0.0
        return max(0.0, float(now_s) - self._true_since_s)
