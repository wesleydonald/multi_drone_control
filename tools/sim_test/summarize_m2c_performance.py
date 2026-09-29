#!/usr/bin/env python3
"""Summarize M2C stage timing and Gazebo RTF diagnostic logs."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import statistics
from typing import Dict, List, Tuple


def load_stages(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle, delimiter="\t"))


def load_rtf(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def stage_times(stages: List[Dict[str, str]]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for row in stages:
        try:
            result[row["stage"]] = float(row["wall_epoch_s"])
        except (KeyError, TypeError, ValueError):
            continue
    return result


def interval_seconds(times: Dict[str, float], start: str, end: str):
    if start not in times or end not in times:
        return None
    return max(0.0, times[end] - times[start])


def samples_between(rows: List[Dict[str, str]], start: float, end: float) -> List[float]:
    values: List[float] = []
    for row in rows:
        try:
            wall = float(row["wall_epoch_s"])
            factor = float(row["real_time_factor"])
        except (KeyError, TypeError, ValueError):
            continue
        if start <= wall <= end:
            values.append(factor)
    return values


def summarize(stage_path: Path, rtf_path: Path) -> str:
    stages = load_stages(stage_path)
    times = stage_times(stages)
    rtf_rows = load_rtf(rtf_path)
    lines = ["M2C PERFORMANCE SUMMARY", ""]

    timing_pairs: List[Tuple[str, str, str]] = [
        ("total_to_pass", "runner_ready", "pass"),
        ("base_bringup", "base_launch_started", "base_plumbing_ready"),
        ("startup_release", "world_paused", "startup_releases_complete"),
        ("post_release_to_assignment", "world_running", "assignment_ready"),
    ]
    for i in range(4):
        timing_pairs.append(
            (f"controller_{i}_startup", f"controller_{i}_start", f"controller_{i}_ready")
        )
    timing_pairs.append(("rviz_startup", "rviz_start", "rviz_ready"))

    lines.append("Stage durations [wall seconds]")
    for label, start, end in timing_pairs:
        duration = interval_seconds(times, start, end)
        if duration is not None:
            lines.append(f"  {label}: {duration:.3f}")
    lines.append("")

    window_labels = ["base_no_controllers"] + [f"after_controller_{i}" for i in range(4)] + ["after_rviz"]
    lines.append("Stable RTF windows")
    any_window = False
    for label in window_labels:
        start_key = f"rtf_window_{label}_start"
        end_key = f"rtf_window_{label}_end"
        if start_key not in times or end_key not in times:
            continue
        values = samples_between(rtf_rows, times[start_key], times[end_key])
        if not values:
            lines.append(f"  {label}: no samples")
            any_window = True
            continue
        lines.append(
            f"  {label}: n={len(values)} mean={statistics.fmean(values):.4f} "
            f"median={statistics.median(values):.4f} min={min(values):.4f} max={max(values):.4f}"
        )
        any_window = True
    if not any_window:
        lines.append("  no completed measurement windows")
    lines.append("")

    lines.append(f"RTF samples captured: {len(rtf_rows)}")
    lines.append("Diagnostic note: measurement dwell windows intentionally add wall time; use per-stage durations, not total_to_pass alone, when assessing controller construction cost.")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stages", type=Path, required=True)
    parser.add_argument("--rtf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    text = summarize(args.stages, args.rtf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
