"""Offline replay for recorded M2A attachment observations."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Optional

import numpy as np

from .attachment_detection import AttachmentDetector
from .cooperative_trajectory import RingNetGeometry
from .m2a_attachment_runtime import (
    calibration_from_dict,
    default_calibration,
    default_detector_config,
    detector_config_from_dict,
    observation_from_csv_row,
    parse_bool,
)


def _load_configuration(metadata_path: Optional[Path]):
    if metadata_path is None or not metadata_path.exists():
        return 0, default_calibration(), default_detector_config(), "rigid_calibrated", 0.025
    payload = json.loads(metadata_path.read_text())
    plate = int(payload.get("assigned_plate_id", 0))
    calibration = (
        calibration_from_dict(payload["calibration"])
        if "calibration" in payload
        else default_calibration()
    )
    config = (
        detector_config_from_dict(payload["detector_config"])
        if "detector_config" in payload
        else default_detector_config()
    )
    contact_observation_model = str(payload.get("contact_observation_model", "rigid_calibrated"))
    sphere_radius_m = float(payload.get("sphere_radius_m", 0.025))
    return plate, calibration, config, contact_observation_model, sphere_radius_m


def replay_attachment_csv(*, input_csv: Path, output_csv: Path, metadata_path: Optional[Path] = None) -> dict:
    assigned_plate_id, calibration, config, contact_observation_model, sphere_radius_m = _load_configuration(metadata_path)
    detector = AttachmentDetector(
        geometry=RingNetGeometry(),
        assigned_plate_id=assigned_plate_id,
        calibration=calibration,
        config=config,
        contact_observation_model=contact_observation_model,
        sphere_radius_m=sphere_radius_m,
    )

    with Path(input_csv).open(newline="") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError("input attachment CSV contains no rows")

    output_fields = list(rows[0].keys()) + [
        "replay_state",
        "replay_fresh",
        "replay_xy_error_m",
        "replay_normal_error_m",
        "replay_relative_speed_mps",
        "replay_candidate_condition",
        "replay_candidate_dwell_s",
        "replay_proof_condition",
        "replay_proof_dwell_s",
        "replay_proof_excitation_m",
        "replay_confirmed",
        "replay_lost_condition",
        "replay_loss_dwell_s",
        "replay_lost",
    ]

    ever_confirmed = False
    truth_rows = 0
    sample_tp = sample_tn = sample_fp = sample_fn = 0
    with Path(output_csv).open("w", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=output_fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            observation = observation_from_csv_row(row)
            result = detector.update(observation)
            ever_confirmed = ever_confirmed or result.confirmed
            replay = dict(row)
            replay.update(
                {
                    "replay_state": result.state.value,
                    "replay_fresh": str(result.fresh).lower(),
                    "replay_xy_error_m": result.xy_error_m,
                    "replay_normal_error_m": result.normal_error_m,
                    "replay_relative_speed_mps": result.relative_speed_mps,
                    "replay_candidate_condition": str(result.candidate_condition).lower(),
                    "replay_candidate_dwell_s": result.candidate_dwell_s,
                    "replay_proof_condition": str(result.proof_condition).lower(),
                    "replay_proof_dwell_s": result.proof_dwell_s,
                    "replay_proof_excitation_m": result.proof_excitation_m,
                    "replay_confirmed": str(result.confirmed).lower(),
                    "replay_lost_condition": str(result.lost_condition).lower(),
                    "replay_loss_dwell_s": result.loss_dwell_s,
                    "replay_lost": str(result.lost).lower(),
                }
            )
            writer.writerow(replay)

            truth_text = str(row.get("joint_detached_truth", "")).strip()
            if truth_text:
                truth_rows += 1
                attached_truth = not parse_bool(truth_text)
                predicted = bool(result.confirmed)
                if predicted and attached_truth:
                    sample_tp += 1
                elif predicted and not attached_truth:
                    sample_fp += 1
                elif not predicted and attached_truth:
                    sample_fn += 1
                else:
                    sample_tn += 1

    return {
        "rows": len(rows),
        "ever_confirmed": ever_confirmed,
        "truth_rows": truth_rows,
        "sample_tp": sample_tp,
        "sample_tn": sample_tn,
        "sample_fp": sample_fp,
        "sample_fn": sample_fn,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--metadata", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or args.input_csv.with_name(args.input_csv.stem + "_replay.csv")
    metadata = args.metadata
    if metadata is None:
        attachment_candidate = args.input_csv.with_name("attachment_metadata.json")
        generic_candidate = args.input_csv.with_name("metadata.json")
        if attachment_candidate.exists():
            metadata = attachment_candidate
        elif generic_candidate.exists():
            metadata = generic_candidate
    summary = replay_attachment_csv(
        input_csv=args.input_csv,
        output_csv=output,
        metadata_path=metadata,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"Replay CSV: {output}")


if __name__ == "__main__":
    main()
