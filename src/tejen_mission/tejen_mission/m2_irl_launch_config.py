"""Fail-closed identity and mission validation for the two-drone M2 IRL rig."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml

from .m2_irl_hardware import IRLVehicleHardware

EXPECTED_VEHICLES = ("drone_0", "drone_1")


@dataclass(frozen=True)
class TwoDroneIdentity:
    ring_id: int
    vehicles: Mapping[str, IRLVehicleHardware]

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicles", MappingProxyType(dict(self.vehicles)))


def load_hardware_config(path: str | Path) -> dict:
    config_path = Path(path).expanduser()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"M2 IRL config is not a mapping: {config_path}")
    return payload


def _positive_id(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def parse_two_drone_identity(payload: Mapping) -> TwoDroneIdentity:
    try:
        root = payload["m2_irl"]
        mocap = root["mocap"]
        vehicles = root["vehicles"]
        ring_id = _positive_id(mocap["ring_rigid_body_id"], "ring_rigid_body_id")
    except (KeyError, TypeError) as exc:
        raise ValueError("M2 IRL config requires m2_irl.mocap and vehicles") from exc
    if not isinstance(vehicles, Mapping) or set(vehicles) != set(EXPECTED_VEHICLES):
        raise ValueError("M2 IRL vehicles must be exactly drone_0 and drone_1")

    parsed: dict[str, IRLVehicleHardware] = {}
    for vehicle_id in EXPECTED_VEHICLES:
        values = vehicles[vehicle_id]
        if not isinstance(values, Mapping):
            raise ValueError(f"{vehicle_id} hardware config must be a mapping")
        try:
            quad = _positive_id(values["physical_quad"], f"{vehicle_id}.physical_quad")
            body = _positive_id(values["body_id"], f"{vehicle_id}.body_id")
            magnet = _positive_id(values["magnet_id"], f"{vehicle_id}.magnet_id")
            if not isinstance(values["ftdi_serial"], str):
                raise ValueError(f"{vehicle_id}.ftdi_serial must be a serial token")
            ftdi = values["ftdi_serial"].strip()
            port = str(values["serial_port"]).strip()
            channel = _positive_id(values["magnet_channel"], f"{vehicle_id}.magnet_channel")
        except KeyError as exc:
            raise ValueError(f"{vehicle_id} missing required identity {exc.args[0]}") from exc
        if not ftdi or not ftdi.isalnum():
            raise ValueError(f"{vehicle_id}.ftdi_serial must be a nonempty serial token")
        if port != f"/dev/QUAD{quad}":
            raise ValueError(f"{vehicle_id}.serial_port must match physical_quad alias /dev/QUAD{quad}")
        if channel < 5 or channel > 10:
            raise ValueError(f"{vehicle_id}.magnet_channel must be in 5..10")
        verified = values.get("props_off_verified", False)
        if not isinstance(verified, bool):
            raise ValueError(f"{vehicle_id}.props_off_verified must be boolean")
        parsed[vehicle_id] = IRLVehicleHardware(
            vehicle_id, quad, body, magnet, ftdi, port, channel, verified
        )

    if len({ring_id, *(item.body_id for item in parsed.values()), *(item.magnet_id for item in parsed.values())}) != 5:
        raise ValueError("ring, body and magnet rigid-body IDs must be distinct")
    for label, values in (
        ("physical_quad", [item.physical_quad for item in parsed.values()]),
        ("ftdi_serial", [item.ftdi_serial for item in parsed.values()]),
        ("serial_port", [item.elrs_device for item in parsed.values()]),
    ):
        if len(set(values)) != 2:
            raise ValueError(f"{label} values must be distinct")
    return TwoDroneIdentity(ring_id, parsed)


def identity_ros_parameters(identity: TwoDroneIdentity) -> dict[str, str]:
    data = {
        "m2_irl": {
            "mocap": {"ring_rigid_body_id": identity.ring_id},
            "vehicles": {
                key: {
                    "physical_quad": item.physical_quad,
                    "body_id": item.body_id,
                    "magnet_id": item.magnet_id,
                    "ftdi_serial": item.ftdi_serial,
                    "serial_port": item.elrs_device,
                    "magnet_channel": item.magnet_channel,
                    "props_off_verified": item.props_off_verified,
                }
                for key, item in identity.vehicles.items()
            },
        }
    }
    return {"hardware_identity_json": json.dumps(data, sort_keys=True)}


def parse_identity_json(encoded: str) -> TwoDroneIdentity:
    if not encoded:
        raise ValueError("hardware_identity_json is required from the M2 IRL launch")
    try:
        return parse_two_drone_identity(json.loads(encoded))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("hardware_identity_json is invalid") from exc


def validate_mission_hardware_config(payload: Mapping) -> dict[str, tuple[str, int]]:
    identity = parse_two_drone_identity(payload)
    result: dict[str, tuple[str, int]] = {}
    for vehicle_id, item in identity.vehicles.items():
        if not item.props_off_verified:
            raise ValueError(
                f"{vehicle_id} magnet AUX channel requires props-off verification"
            )
        result[vehicle_id] = (item.elrs_device, item.magnet_channel)
    return result
