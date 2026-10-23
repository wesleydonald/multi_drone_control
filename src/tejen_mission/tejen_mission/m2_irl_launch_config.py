"""Fail-closed identity and mission validation for two/four-drone M2 IRL rigs."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml

from .m2_irl_hardware import IRLVehicleHardware
from .m2b_attachment_mission import M2BIRLTipAlignmentConfig

EXPECTED_VEHICLES = ("drone_0", "drone_1")


@dataclass(frozen=True)
class IRLFleetIdentity:
    ring_id: int
    vehicles: Mapping[str, IRLVehicleHardware]

    def __post_init__(self) -> None:
        object.__setattr__(self, "vehicles", MappingProxyType(dict(self.vehicles)))


# Preserve callers of the commissioned two-drone interface.
TwoDroneIdentity = IRLFleetIdentity


def parking_ros_parameters(payload: Mapping, completion_mode='airborne_hold') -> dict:
    """Parking profile is opt-in; old configurations remain valid when disabled."""
    from .m2_irl_attachment_policy import ParkingConfig
    if completion_mode not in ('airborne_hold', 'attach_park'):
        raise ValueError('completion_mode must be airborne_hold or attach_park')
    result = {'m2b_completion_mode': completion_mode}
    if completion_mode == 'airborne_hold':
        return result
    if payload['m2_irl'].get('attachment_policy') != 'measured_contact_15deg':
        raise ValueError('attach_park requires measured_contact_15deg')
    try:
        values = payload['m2_irl']['parking']
        config = ParkingConfig(**{name: float(values[name]) for name in ParkingConfig.__dataclass_fields__})
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'invalid M2 IRL parking profile: {exc}') from exc
    result.update({'parking_'+name: value for name, value in asdict(config).items()})
    return result


@dataclass(frozen=True)
class IRLRingGeometry:
    """Measured physical annulus; simulation's RingNetGeometry defaults stay fixed."""

    inner_diameter_m: float
    outer_diameter_m: float
    plate_diameter_m: float

    @property
    def radial_width_m(self) -> float:
        return 0.5 * (self.outer_diameter_m - self.inner_diameter_m)

    @property
    def plate_pitch_diameter_m(self) -> float:
        return 0.5 * (self.inner_diameter_m + self.outer_diameter_m)

    @property
    def plate_pitch_radius_m(self) -> float:
        return 0.5 * self.plate_pitch_diameter_m


def parse_irl_ring_geometry(payload: Mapping) -> IRLRingGeometry:
    """Validate dimensions once; derive all IRL plate and collision radii."""
    try:
        geometry = payload["m2_irl"]["geometry"]
        values = (
            float(geometry["ring_inner_diameter_m"]),
            float(geometry["ring_outer_diameter_m"]),
            float(geometry["ring_plate_diameter_m"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("M2 IRL ring geometry requires inner, outer and plate diameters") from exc
    inner, outer, plate = values
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("M2 IRL ring diameters must be finite and positive")
    if outer <= inner:
        raise ValueError("M2 IRL ring outer diameter must exceed inner diameter")
    result = IRLRingGeometry(inner, outer, plate)
    if plate > result.radial_width_m:
        raise ValueError("M2 IRL ring plate diameter exceeds measured annulus width")
    return result


@dataclass(frozen=True)
class IRLQuietMissProfile:
    wait_s: float
    improvement_m: float
    relative_speed_mps: float


@dataclass(frozen=True)
class IRLAttachmentSearchProfile:
    contact_overtravel_m: float
    max_contact_correction_m: float
    contact_correction_rate_mps: float
    capture_normal_m: float
    candidate_xy_m: float
    candidate_normal_m: float
    proof_normal_m: float
    proof_relative_speed_mps: float
    proof_max_contact_drift_m: float



def progressive_ros_parameters(payload: Mapping) -> dict:
    """One validated profile shared by hardware and opt-in simulation adapters."""
    from .m2_irl_attachment_policy import validate_policy, POLICY
    policy = validate_policy(str(payload["m2_irl"].get("attachment_policy", "legacy")))
    if policy != POLICY:
        return {"planner": {}, "observer": {}, "supervisor": {}}
    profiles = parse_gate_profiles(payload)
    search = profiles.attachment_search
    values = payload["m2_irl"]["gate_profiles"].get("progressive_attachment")
    if not isinstance(values, Mapping):
        raise ValueError("M2 IRL config requires gate_profiles.progressive_attachment")
    names = ("contact_dwell_s", "proof_dwell_s", "proof_angle_deg",
             "proof_angle_tolerance_deg", "proof_duration_s", "proof_speed_mps",
             "release_dwell_s", "release_speed_mps", "loss_xy_m", "loss_normal_m", "loss_dwell_s")
    v = {n: _gate_value(values, n, "progressive_attachment."+n, positive=True) for n in names}
    attachment = payload["m2_irl"]["gate_profiles"]["attachment_search"]
    proof_xy_m = _gate_value(attachment, "proof_xy_m", "attachment_search.proof_xy_m", positive=True) if "proof_xy_m" in attachment else search.candidate_xy_m
    if not 0 < v["proof_angle_tolerance_deg"] < v["proof_angle_deg"] < 90:
        raise ValueError("proof angle/tolerance must be ordered within (0,90)")
    if (proof_xy_m < search.candidate_xy_m or v["loss_xy_m"] < proof_xy_m
            or v["loss_normal_m"] < search.proof_normal_m):
        raise ValueError("loss geometry must exceed contact geometry")
    planner = dict(attachment_policy=policy,
        progressive_arc_duration_s=v["proof_duration_s"], progressive_arc_speed_mps=v["proof_speed_mps"],
        progressive_loss_dwell_s=v["loss_dwell_s"], progressive_release_dwell_s=v["release_dwell_s"],
        progressive_release_speed_mps=v["release_speed_mps"], m2b_attached_hold_angle_deg=v["proof_angle_deg"],
        m2b_irl_capture_search_enabled=True, m2b_irl_tip_alignment_enabled=True,
        m2b_contact_overtravel_m=search.contact_overtravel_m,
        m2b_irl_max_contact_correction_m=search.max_contact_correction_m,
        m2b_irl_contact_correction_rate_mps=search.contact_correction_rate_mps,
        m2b_capture_hold_normal_m=search.capture_normal_m,
        m2b_evidence_timeout_s=profiles.mocap_timeout_s)
    observer = dict(attachment_policy=policy,
        candidate_xy_m=search.candidate_xy_m, candidate_normal_m=search.candidate_normal_m,
        candidate_speed_mps=search.proof_relative_speed_mps, candidate_dwell_s=v["contact_dwell_s"],
        proof_xy_m=proof_xy_m, proof_normal_m=search.proof_normal_m,
        proof_speed_mps=search.proof_relative_speed_mps, proof_dwell_s=v["proof_dwell_s"],
        proof_max_contact_drift_m=search.proof_max_contact_drift_m,
        proof_angle_deg=v["proof_angle_deg"], proof_angle_tolerance_deg=v["proof_angle_tolerance_deg"],
        loss_xy_m=v["loss_xy_m"], loss_normal_m=v["loss_normal_m"], loss_dwell_s=v["loss_dwell_s"],
        pose_timeout_s=profiles.mocap_timeout_s, magnet_off_counts_as_loss=False)
    return {"planner": planner, "observer": observer, "supervisor": {"attachment_policy": policy}}

def load_hardware_config(path: str | Path) -> dict:
    config_path = Path(path).expanduser()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"M2 IRL config is not a mapping: {config_path}")
    return payload



def _gate_value(section: Mapping, name: str, label: str, *, positive: bool) -> float:
    try:
        value = float(section[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"M2 IRL gate profile requires {label}") from exc
    if not math.isfinite(value) or (value <= 0.0 if positive else value < 0.0):
        qualifier = "positive" if positive else "non-negative"
        raise ValueError(f"{label} must be finite and {qualifier}")
    return value


def parse_gate_profiles(payload: Mapping) -> IRLGateProfiles:
    """Parse the one fail-closed M2 IRL gate-profile source of truth."""
    try:
        profiles = payload["m2_irl"]["gate_profiles"]
        freshness = profiles["freshness"]
        ground = profiles["ground"]
        airborne = profiles["airborne"]
        c1f2 = profiles["c1f2_physical_recheck"]
        tip = profiles["tip_alignment"]
        attachment = profiles["attachment_search"]
        quiet_miss = profiles["quiet_miss"]
    except (KeyError, TypeError) as exc:
        raise ValueError("M2 IRL config requires complete m2_irl.gate_profiles") from exc
    if not all(isinstance(value, Mapping) for value in (freshness, ground, airborne, c1f2, tip, attachment, quiet_miss)):
        raise ValueError("M2 IRL gate profile sections must be mappings")
    result = IRLGateProfiles(
        mocap_timeout_s=_gate_value(freshness, "mocap_timeout_s", "freshness.mocap_timeout_s", positive=True),
        telemetry_timeout_s=_gate_value(freshness, "telemetry_timeout_s", "freshness.telemetry_timeout_s", positive=True),
        ring_timeout_s=_gate_value(freshness, "ring_timeout_s", "freshness.ring_timeout_s", positive=True),
        status_timeout_s=_gate_value(freshness, "status_timeout_s", "freshness.status_timeout_s", positive=True),
        link_statistics_timeout_s=_gate_value(freshness, "link_statistics_timeout_s", "freshness.link_statistics_timeout_s", positive=True),
        cooperative_shared_start_skew_s=_gate_value(freshness, "cooperative_shared_start_skew_s", "freshness.cooperative_shared_start_skew_s", positive=False),
        ground_speed_mps=_gate_value(ground, "speed_mps", "ground.speed_mps", positive=False),
        ground_dwell_s=_gate_value(ground, "dwell_s", "ground.dwell_s", positive=False),
        airborne_position_tolerance_m=_gate_value(airborne, "position_tolerance_m", "airborne.position_tolerance_m", positive=True),
        airborne_speed_mps=_gate_value(airborne, "speed_mps", "airborne.speed_mps", positive=True),
        airborne_swing_deg=_gate_value(airborne, "swing_deg", "airborne.swing_deg", positive=True),
        takeoff_swing_deg=_gate_value(airborne, "takeoff_swing_deg", "airborne.takeoff_swing_deg", positive=True),
        airborne_dwell_s=_gate_value(airborne, "dwell_s", "airborne.dwell_s", positive=False),
        quiet_miss=IRLQuietMissProfile(
            wait_s=_gate_value(quiet_miss, "wait_s", "quiet_miss.wait_s", positive=True),
            improvement_m=_gate_value(quiet_miss, "improvement_m", "quiet_miss.improvement_m", positive=True),
            relative_speed_mps=_gate_value(quiet_miss, "relative_speed_mps", "quiet_miss.relative_speed_mps", positive=True),
        ),
        c1f2_max_lift_drift_m=_gate_value(c1f2, "max_lift_drift_m", "c1f2_physical_recheck.max_lift_drift_m", positive=True),
        c1f2_speed_mps=_gate_value(c1f2, "speed_mps", "c1f2_physical_recheck.speed_mps", positive=True),
        c1f2_acceleration_mps2=_gate_value(c1f2, "acceleration_mps2", "c1f2_physical_recheck.acceleration_mps2", positive=True),
        c1f2_dwell_s=_gate_value(c1f2, "dwell_s", "c1f2_physical_recheck.dwell_s", positive=False),
        tip_alignment=M2BIRLTipAlignmentConfig(
            gain_per_s=_gate_value(tip, "gain_per_s", "tip_alignment.gain_per_s", positive=True),
            filter_tau_s=_gate_value(tip, "filter_tau_s", "tip_alignment.filter_tau_s", positive=True),
            max_rate_mps=_gate_value(tip, "max_rate_mps", "tip_alignment.max_rate_mps", positive=True),
            max_offset_m=_gate_value(tip, "max_offset_m", "tip_alignment.max_offset_m", positive=True),
            min_normal_m=_gate_value(tip, "min_normal_m", "tip_alignment.min_normal_m", positive=True),
            pre_low_normal_m=_gate_value(tip, "pre_low_normal_m", "tip_alignment.pre_low_normal_m", positive=True),
            capture_xy_m=_gate_value(tip, "capture_xy_m", "tip_alignment.capture_xy_m", positive=True),
            capture_dwell_s=_gate_value(tip, "capture_dwell_s", "tip_alignment.capture_dwell_s", positive=True),
            low_xy_m=_gate_value(tip, "low_xy_m", "tip_alignment.low_xy_m", positive=True),
            low_dwell_s=_gate_value(tip, "low_dwell_s", "tip_alignment.low_dwell_s", positive=True),
            max_relative_speed_mps=_gate_value(tip, "max_relative_speed_mps", "tip_alignment.max_relative_speed_mps", positive=True),
        ),
        attachment_search=IRLAttachmentSearchProfile(
            contact_overtravel_m=_gate_value(attachment, "contact_overtravel_m", "attachment_search.contact_overtravel_m", positive=False),
            max_contact_correction_m=_gate_value(attachment, "max_contact_correction_m", "attachment_search.max_contact_correction_m", positive=False),
            contact_correction_rate_mps=_gate_value(attachment, "contact_correction_rate_mps", "attachment_search.contact_correction_rate_mps", positive=True),
            capture_normal_m=_gate_value(attachment, "capture_normal_m", "attachment_search.capture_normal_m", positive=True),
            candidate_xy_m=_gate_value(attachment, "candidate_xy_m", "attachment_search.candidate_xy_m", positive=True),
            candidate_normal_m=_gate_value(attachment, "candidate_normal_m", "attachment_search.candidate_normal_m", positive=True),
            proof_normal_m=_gate_value(attachment, "proof_normal_m", "attachment_search.proof_normal_m", positive=True),
            proof_relative_speed_mps=_gate_value(attachment, "proof_relative_speed_mps", "attachment_search.proof_relative_speed_mps", positive=True),
            proof_max_contact_drift_m=_gate_value(attachment, "proof_max_contact_drift_m", "attachment_search.proof_max_contact_drift_m", positive=True),
        ),
    )
    if result.tip_alignment.min_normal_m > result.tip_alignment.pre_low_normal_m:
        raise ValueError("tip alignment minimum clearance exceeds pre-LOW staging clearance")
    search = result.attachment_search
    if search.contact_overtravel_m > 0.050:
        raise ValueError("IRL contact overtravel must be <= 0.050 m")
    if search.max_contact_correction_m > 0.030:
        raise ValueError("IRL measured contact correction must be <= 0.030 m")
    if search.contact_correction_rate_mps > 0.010:
        raise ValueError("IRL measured contact correction rate must be <= 0.010 m/s")
    if search.candidate_normal_m > search.capture_normal_m or search.proof_normal_m > search.candidate_normal_m:
        raise ValueError("IRL proof/contact normal bands must be no wider than capture search")
    if search.capture_normal_m > 0.04:
        raise ValueError("IRL contact band must remain within magnet enable height")
    if search.proof_max_contact_drift_m > 0.040:
        raise ValueError("IRL proof contact drift must be <= 0.040 m")

    if result.tip_alignment.low_xy_m > result.tip_alignment.capture_xy_m:
        raise ValueError("tip alignment LOW XY gate cannot exceed capture XY gate")
    if result.link_statistics_timeout_s < result.mocap_timeout_s:
        raise ValueError("link-statistics freshness must not be tighter than mocap freshness")
    if result.cooperative_shared_start_skew_s > min(0.05, result.mocap_timeout_s):
        raise ValueError("cooperative shared start skew must be at most 0.05 s and within mocap freshness")
    return result


def gate_profile_ros_parameters(profiles: IRLGateProfiles) -> dict[str, str]:
    """Carry the audited profile snapshot with the YAML-derived identity."""
    return {"m2_irl_gate_profiles_json": json.dumps(asdict(profiles), sort_keys=True)}

def _positive_id(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def parse_fleet_identity(payload: Mapping) -> IRLFleetIdentity:
    try:
        root = payload["m2_irl"]
        mocap = root["mocap"]
        vehicles = root["vehicles"]
        ring_id = _positive_id(mocap["ring_rigid_body_id"], "ring_rigid_body_id")
    except (KeyError, TypeError) as exc:
        raise ValueError("M2 IRL config requires m2_irl.mocap and vehicles") from exc
    if not isinstance(vehicles, Mapping) or len(vehicles) not in (2, 4):
        raise ValueError("M2 IRL requires two or four vehicles")
    vehicle_ids = tuple(f"drone_{i}" for i in range(len(vehicles)))
    if set(vehicles) != set(vehicle_ids):
        raise ValueError("M2 IRL vehicles must be contiguous drone_0..drone_N")

    parsed: dict[str, IRLVehicleHardware] = {}
    for vehicle_id in vehicle_ids:
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

    if len({ring_id, *(item.body_id for item in parsed.values()), *(item.magnet_id for item in parsed.values())}) != 1 + 2 * len(parsed):
        raise ValueError("ring, body and magnet rigid-body IDs must be distinct")
    for label, values in (
        ("physical_quad", [item.physical_quad for item in parsed.values()]),
        ("ftdi_serial", [item.ftdi_serial for item in parsed.values()]),
        ("serial_port", [item.elrs_device for item in parsed.values()]),
    ):
        if len(set(values)) != len(parsed):
            raise ValueError(f"{label} values must be distinct")
    return IRLFleetIdentity(ring_id, parsed)


def parse_two_drone_identity(payload: Mapping) -> TwoDroneIdentity:
    identity = parse_fleet_identity(payload)
    if tuple(identity.vehicles) != EXPECTED_VEHICLES:
        raise ValueError("two-drone M2 IRL vehicles must be exactly drone_0 and drone_1")
    return identity


def parse_flight_order(payload: Mapping) -> tuple[str, ...]:
    """Operator sequence is independent of identity and assignment ordering."""
    vehicle_ids = tuple(parse_fleet_identity(payload).vehicles)
    order = payload["m2_irl"].get("flight_order", list(vehicle_ids))
    if (not isinstance(order, list) or not all(isinstance(v, str) for v in order)
            or len(order) != len(vehicle_ids) or set(order) != set(vehicle_ids)):
        raise ValueError("flight_order must contain each configured vehicle exactly once")
    return tuple(order)


def identity_ros_parameters(identity: IRLFleetIdentity) -> dict[str, str]:
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


def parse_identity_json(encoded: str) -> IRLFleetIdentity:
    if not encoded:
        raise ValueError("hardware_identity_json is required from the M2 IRL launch")
    try:
        return parse_fleet_identity(json.loads(encoded))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("hardware_identity_json is invalid") from exc


def validate_mission_hardware_config(payload: Mapping) -> dict[str, tuple[str, int]]:
    identity = parse_fleet_identity(payload)
    result: dict[str, tuple[str, int]] = {}
    for vehicle_id, item in identity.vehicles.items():
        if not item.props_off_verified:
            raise ValueError(
                f"{vehicle_id} magnet AUX channel requires props-off verification"
            )
        result[vehicle_id] = (item.elrs_device, item.magnet_channel)
    return result
@dataclass(frozen=True)
class IRLGateProfiles:
    mocap_timeout_s: float
    telemetry_timeout_s: float
    ring_timeout_s: float
    status_timeout_s: float
    link_statistics_timeout_s: float
    cooperative_shared_start_skew_s: float
    ground_speed_mps: float
    ground_dwell_s: float
    airborne_position_tolerance_m: float
    airborne_speed_mps: float
    airborne_swing_deg: float
    takeoff_swing_deg: float
    airborne_dwell_s: float
    c1f2_max_lift_drift_m: float
    c1f2_speed_mps: float
    c1f2_acceleration_mps2: float
    c1f2_dwell_s: float
    tip_alignment: M2BIRLTipAlignmentConfig
    attachment_search: IRLAttachmentSearchProfile
    quiet_miss: IRLQuietMissProfile
