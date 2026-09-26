#!/usr/bin/env python3
"""Initialize the real X3's M2B grounded tether configuration while Gazebo is paused.

The production X3 SDF is not edited or replaced. This helper only sets the initial
poses of the existing x3 model, tether_rod link, and magnet_tip_link before the
first physics step. It exits non-zero if Gazebo rejects any pose command.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Iterable

import numpy as np

from tejen_mission.m2b_ground_start import GroundStartConfig, compute_ground_start_geometry, yaw_quaternion_xyzw


SET_POSE_SERVICE = "/world/quadcopter/set_pose"


def _pose_request(name: str, position: Iterable[float], quaternion_xyzw: Iterable[float]) -> str:
    p = np.asarray(tuple(position), dtype=float).reshape(3)
    q = np.asarray(tuple(quaternion_xyzw), dtype=float).reshape(4)
    return (
        f'name: "{name}" '
        f'position {{ x: {p[0]:.9f} y: {p[1]:.9f} z: {p[2]:.9f} }} '
        f'orientation {{ x: {q[0]:.12f} y: {q[1]:.12f} z: {q[2]:.12f} w: {q[3]:.12f} }}'
    )


def _set_pose(name: str, position: np.ndarray, quaternion_xyzw: np.ndarray, *, timeout_s: float) -> None:
    request = _pose_request(name, position, quaternion_xyzw)
    try:
        result = subprocess.run(
            [
                "gz", "service", "-s", SET_POSE_SERVICE,
                "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                "--timeout", str(max(1, int(round(timeout_s * 1000.0)))),
                "--req", request,
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s + 1.0,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"Timed out setting Gazebo pose for {name}") from exc
    output = result.stdout or ""
    if result.returncode != 0 or re.search(r"\bdata\s*:\s*true\b", output, flags=re.IGNORECASE) is None:
        raise RuntimeError(
            f"Gazebo rejected set_pose for {name}: rc={result.returncode}, output={output.strip()}"
        )


def _set_pose_with_retry(name: str, position: np.ndarray, quaternion_xyzw: np.ndarray, *, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            _set_pose(name, position, quaternion_xyzw, timeout_s=min(1.0, max(0.2, deadline - time.monotonic())))
            return
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.10)
    raise RuntimeError(f"Unable to initialize {name} before timeout: {last_error}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assigned-plate-id", type=int, default=0)
    parser.add_argument("--ring-z", type=float, default=0.035)
    parser.add_argument("--body-z", type=float, default=0.105)
    parser.add_argument("--body-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--output-json", type=Path, default=None)
    args = parser.parse_args(argv)

    cfg = GroundStartConfig(
        ring_z_m=args.ring_z,
        body_z_m=args.body_z,
        initial_body_yaw_rad=args.body_yaw,
    )
    geometry = compute_ground_start_geometry(cfg, assigned_plate_id=args.assigned_plate_id)

    body_q = yaw_quaternion_xyzw(geometry.body_yaw_rad)
    # Model first, then child links. All commands happen while simulation time is paused.
    _set_pose_with_retry("x3", geometry.body_position_world, body_q, timeout_s=args.timeout)
    _set_pose_with_retry(
        "x3::tether_rod",
        geometry.tether_origin_world,
        geometry.tether_quaternion_xyzw,
        timeout_s=args.timeout,
    )
    _set_pose_with_retry(
        "x3::magnet_tip_link",
        geometry.magnet_center_world,
        geometry.magnet_quaternion_xyzw,
        timeout_s=args.timeout,
    )

    cable = geometry.magnet_center_world - geometry.tether_origin_world
    horizontal = float(np.linalg.norm(cable[:2]))
    angle_deg = float(np.degrees(np.arctan2(abs(cable[2]), horizontal)))
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "assigned_plate_id": int(args.assigned_plate_id),
            "body_yaw_rad": float(geometry.body_yaw_rad),
            "body_position_world_m": geometry.body_position_world.tolist(),
            "tether_origin_world_m": geometry.tether_origin_world.tolist(),
            "joint_anchor_world_m": geometry.joint_anchor_world.tolist(),
            "tether_quaternion_xyzw": geometry.tether_quaternion_xyzw.tolist(),
            "magnet_center_world_m": geometry.magnet_center_world.tolist(),
            "magnet_quaternion_xyzw": geometry.magnet_quaternion_xyzw.tolist(),
            "plate_position_world_m": geometry.plate_position_world.tolist(),
            "cable_angle_from_horizontal_deg": angle_deg,
        }
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    print(
        "M2B ground initialization complete: "
        f"plate={args.assigned_plate_id}, "
        f"body=({geometry.body_position_world[0]:.3f},"
        f"{geometry.body_position_world[1]:.3f},{geometry.body_position_world[2]:.3f}), "
        f"magnet=({geometry.magnet_center_world[0]:.3f},"
        f"{geometry.magnet_center_world[1]:.3f},{geometry.magnet_center_world[2]:.3f}), "
        f"cable_angle_from_horizontal={angle_deg:.2f} deg"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
