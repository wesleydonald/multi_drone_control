#!/usr/bin/env python3
"""Capture Gazebo WorldStatistics into a compact CSV for M2C performance analysis.

This is diagnostic-only instrumentation. It subscribes to Gazebo's native world stats
stream and records one row per reported real_time_factor sample, preserving the
corresponding simulation / real times and wall-clock receipt time. It does not alter
simulation state or ROS timing.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time
from typing import Dict, Iterable, Iterator, Optional


_FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def _seconds(parts: Dict[str, float]) -> Optional[float]:
    if "sec" not in parts and "nsec" not in parts:
        return None
    return float(parts.get("sec", 0.0)) + float(parts.get("nsec", 0.0)) * 1e-9


def parse_world_stats_lines(lines: Iterable[str]) -> Iterator[Dict[str, object]]:
    """Yield parsed samples from text output of ``gz topic -e``.

    ``WorldStatistics`` is rendered as a multi-line protobuf. ``real_time_factor``
    occurs before ``step_size`` in the current message, so a sample is committed
    when the next ``sim_time`` block starts (or at EOF). This keeps the complete
    message together while preserving wall receipt time at the RTF field itself.
    """

    section: Optional[str] = None
    nested: Dict[str, Dict[str, float]] = {
        "sim_time": {},
        "real_time": {},
        "step_size": {},
    }
    scalar: Dict[str, object] = {}
    factor: Optional[float] = None
    factor_wall_time: Optional[float] = None

    def build_sample() -> Dict[str, object]:
        assert factor is not None
        assert factor_wall_time is not None
        return {
            "wall_epoch_s": factor_wall_time,
            "wall_iso_utc": datetime.fromtimestamp(
                factor_wall_time, tz=timezone.utc
            ).isoformat(),
            "sim_time_s": _seconds(nested["sim_time"]),
            "real_time_s": _seconds(nested["real_time"]),
            "real_time_factor": factor,
            "iterations": scalar.get("iterations"),
            "paused": scalar.get("paused"),
            "step_size_s": _seconds(nested["step_size"]),
        }

    def reset_message() -> None:
        nonlocal section, nested, scalar, factor, factor_wall_time
        section = None
        nested = {"sim_time": {}, "real_time": {}, "step_size": {}}
        scalar = {}
        factor = None
        factor_wall_time = None

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        opening = re.match(r"^(sim_time|real_time|step_size)\s*\{$", line)
        if opening:
            opened = opening.group(1)
            if opened == "sim_time" and factor is not None:
                yield build_sample()
                reset_message()
            section = opened
            nested[section] = {}
            continue
        if line == "}":
            section = None
            continue

        if section is not None:
            match = re.match(r"^(sec|nsec):\s*(-?\d+)$", line)
            if match:
                nested[section][match.group(1)] = float(match.group(2))
            continue

        match = re.match(r"^paused:\s*(true|false)$", line, flags=re.IGNORECASE)
        if match:
            scalar["paused"] = match.group(1).lower() == "true"
            continue

        match = re.match(r"^iterations:\s*(\d+)$", line)
        if match:
            scalar["iterations"] = int(match.group(1))
            continue

        match = re.match(rf"^real_time_factor:\s*({_FLOAT_RE})$", line)
        if match:
            factor = float(match.group(1))
            factor_wall_time = time.time()
            continue

    if factor is not None:
        yield build_sample()


def capture(topic: str, output: Path) -> int:
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["gz", "topic", "-e", "-t", topic]
    # gz topic is normally interactive; when stdout is piped through Python, ask
    # coreutils stdbuf for line buffering so each 5 Hz WorldStatistics message is
    # written promptly instead of arriving in large delayed blocks.
    if shutil.which("stdbuf"):
        cmd = ["stdbuf", "-oL", "-eL", *cmd]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    stop_requested = False

    def _stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True
        if proc.poll() is None:
            proc.terminate()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    fieldnames = [
        "wall_epoch_s",
        "wall_iso_utc",
        "sim_time_s",
        "real_time_s",
        "real_time_factor",
        "iterations",
        "paused",
        "step_size_s",
    ]

    try:
        assert proc.stdout is not None
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            handle.flush()
            for sample in parse_world_stats_lines(proc.stdout):
                writer.writerow(sample)
                handle.flush()
                if stop_requested:
                    break
    finally:
        if proc.poll() is None:
            proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2.0)

    if stop_requested:
        return 0
    if proc.returncode in (0, -signal.SIGTERM, -signal.SIGINT):
        return 0

    stderr = ""
    if proc.stderr is not None:
        stderr = proc.stderr.read().strip()
    if stderr:
        print(stderr, file=sys.stderr)
    return int(proc.returncode or 1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/world/quadcopter/stats")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    return capture(args.topic, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
