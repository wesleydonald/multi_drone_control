"""Pure hardware identity and rate-limit helpers for the two-drone M2 IRL rig."""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping


DEFAULT_MOCAP_BIND_ADDRESS = "0.0.0.0"
DEFAULT_MOCAP_BIND_PORT = 1511
RING_RIGID_BODY_ID = 8


@dataclass(frozen=True)
class IRLVehicleHardware:
    vehicle_id: str
    physical_quad: int
    body_id: int
    magnet_id: int
    ftdi_serial: str
    elrs_device: str
    magnet_channel: int = 6
    props_off_verified: bool = False


def default_two_drone_hardware() -> Mapping[str, IRLVehicleHardware]:
    return MappingProxyType(
        {
            "drone_0": IRLVehicleHardware(
                "drone_0", 2, 12, 22, "FTF0AROT", "/dev/QUAD2"
            ),
            "drone_1": IRLVehicleHardware(
                "drone_1", 4, 14, 24, "FTF09R63", "/dev/QUAD4"
            ),
        }
    )


def required_rigid_body_ids(
    mapping: Mapping[str, IRLVehicleHardware] | None = None,
) -> tuple[int, ...]:
    hardware = default_two_drone_hardware() if mapping is None else mapping
    result = [RING_RIGID_BODY_ID]
    for vehicle_id in hardware:
        result.extend((hardware[vehicle_id].body_id, hardware[vehicle_id].magnet_id))
    return tuple(result)


class PerRigidBodyThrottle:
    """Independent pre-parser rate gate for each OptiTrack rigid-body stream."""

    def __init__(self, *, max_rate_hz: float = 120.0) -> None:
        rate = float(max_rate_hz)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("max_rate_hz must be finite and positive")
        self.minimum_period_s = 1.0 / rate
        self._last_accepted_s: dict[int, float] = {}

    def accept(self, rigid_body_id: int, now_s: float) -> bool:
        body_id = int(rigid_body_id)
        now = float(now_s)
        if body_id < 0 or not math.isfinite(now):
            raise ValueError("rigid_body_id and now_s must be valid")
        previous = self._last_accepted_s.get(body_id)
        if previous is not None and now < previous:
            return False
        if previous is not None and now - previous < self.minimum_period_s:
            return False
        self._last_accepted_s[body_id] = now
        return True
