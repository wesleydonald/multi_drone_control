#!/usr/bin/env python3
"""Capture existing C1F experiment logs into one timestamped evidence bundle.

This tool deliberately does not redirect or modify any runtime logger. ``start``
records a filesystem snapshot and experiment metadata. ``stop`` finds relevant
files that are new or changed since that snapshot, copies them into one bundle,
and leaves the canonical logs untouched.

Run from the drone_cage_control repository root through the shell wrappers:

    tools/c1f_experiment/start_experiment.sh c1f3a sim
    ... run the normal experiment terminals ...
    tools/c1f_experiment/stop_experiment.sh
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


EXPERIMENT_ROOT = Path("logs/c1f_experiments")
ACTIVE_STATE = EXPERIMENT_ROOT / ".active_run.json"

# Only existing runtime products are collected. No logger is redirected.
SOURCE_PATTERNS: dict[str, tuple[str, ...]] = {
    "planner": (
        "logs/tejen_dynamic_planner/**/backend.csv",
        "logs/tejen_dynamic_planner/**/replans.csv",
    ),
    "mpc": (
        "logs/tejen_mpc/**/log.csv",
        "logs/tejen_mpc/**/metadata.json",
    ),
    "mocap": (
        "logs/motion_capture_udp/motion_capture_udp_*.csv",
    ),
    "join_planner": (
        "logs/join_planner/*.csv",
    ),
}

CONFIG_SNAPSHOTS = (
    Path("src/tejen_mission/config/c1f2b_cpp_authority.yaml"),
    Path("src/tejen_dynamic_planner/config/c1f3a_static_sim.yaml"),
    Path("src/tejen_dynamic_planner/config/c1f3b_cooperative_hover.yaml"),
    Path("src/tejen_dynamic_planner/config/c1f6_moving_rendezvous.yaml"),
    Path("src/tejen_dynamic_planner/config/c1f_irl_clear.yaml"),
    Path("src/tejen_dynamic_planner/config/c1f_irl_virtual_static.yaml"),
)

COMMAND_SHEET_GLOBS = (
    "commandsFor*C1F*",
    "commandsForPayloadIRL*.txt",
)


def repo_root() -> Path:
    root = Path.cwd().resolve()
    if not (root / "src").is_dir():
        raise RuntimeError(
            "Run this command from the drone_cage_control repository root "
            f"(current directory: {root})"
        )
    return root


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def safe_label(value: str) -> str:
    value = value.strip().lower().replace(" ", "_")
    allowed = set("abcdefghijklmnopqrstuvwxyz0123456789_-.")
    cleaned = "".join(ch for ch in value if ch in allowed)
    return cleaned.strip("_.-") or "run"


def git_value(root: Path, args: list[str], default: str = "unavailable") -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=3,
        )
        return result.stdout.strip() or default
    except (OSError, subprocess.SubprocessError):
        return default


def git_metadata(root: Path) -> dict[str, object]:
    commit = git_value(root, ["rev-parse", "HEAD"])
    branch = git_value(root, ["rev-parse", "--abbrev-ref", "HEAD"])
    status = git_value(root, ["status", "--porcelain"], default="")
    return {
        "commit": commit,
        "branch": branch,
        "dirty": bool(status),
        "status_porcelain": status.splitlines(),
    }


def iter_sources(root: Path) -> Iterable[tuple[str, Path]]:
    seen: set[Path] = set()
    for subsystem, patterns in SOURCE_PATTERNS.items():
        for pattern in patterns:
            for path in sorted(root.glob(pattern)):
                if not path.is_file():
                    continue
                resolved = path.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                yield subsystem, path


def snapshot(root: Path) -> dict[str, dict[str, int]]:
    result: dict[str, dict[str, int]] = {}
    for _, path in iter_sources(root):
        stat = path.stat()
        rel = str(path.relative_to(root))
        result[rel] = {
            "size": int(stat.st_size),
            "mtime_ns": int(stat.st_mtime_ns),
        }
    return result


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def snapshot_configuration(
    root: Path, run_dir: Path, extra_paths: Iterable[str] = ()
) -> list[str]:
    copied: list[str] = []
    config_dir = run_dir / "configuration"
    requested = list(CONFIG_SNAPSHOTS)
    requested.extend(Path(value) for value in extra_paths)
    for source in dict.fromkeys(requested):
        full = root / source
        if not full.is_file():
            continue
        try:
            source = full.resolve().relative_to(root)
        except ValueError as exc:
            raise ValueError(f"configuration snapshot escapes repository: {source}") from exc
        full = root / source
        destination = config_dir / source
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(full, destination)
        copied.append(str(source))

    for pattern in COMMAND_SHEET_GLOBS:
        for full in sorted(root.glob(pattern)):
            if not full.is_file():
                continue
            rel = full.relative_to(root)
            destination = config_dir / rel
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(full, destination)
            copied.append(str(rel))
    return copied


def start(args: argparse.Namespace) -> int:
    root = repo_root()
    EXPERIMENT_ROOT.mkdir(parents=True, exist_ok=True)

    if ACTIVE_STATE.exists() and not args.force:
        try:
            active = json.loads(ACTIVE_STATE.read_text())
            active_id = active.get("run_id", "unknown")
        except Exception:
            active_id = "unknown"
        raise RuntimeError(
            f"An experiment is already marked active ({active_id}). "
            "Run stop_experiment.sh first, or rerun start with --force only if "
            "you intentionally want to abandon that marker."
        )

    if ACTIVE_STATE.exists() and args.force:
        ACTIVE_STATE.unlink()

    scenario = safe_label(args.scenario)
    mode = safe_label(args.mode)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_id = f"{scenario}_{mode}_{timestamp}"
    run_dir = EXPERIMENT_ROOT / run_id
    suffix = 1
    while run_dir.exists():
        run_id = f"{scenario}_{mode}_{timestamp}_{suffix}"
        run_dir = EXPERIMENT_ROOT / run_id
        suffix += 1
    run_dir.mkdir(parents=True)

    start_inventory = snapshot(root)
    state = {
        "schema_version": 1,
        "run_id": run_id,
        "scenario": scenario,
        "mode": mode,
        "repo_root": str(root),
        "run_dir": str(run_dir),
        "start_time_unix": time.time(),
        "start_time_iso": now_iso(),
        "start_inventory": start_inventory,
        "git": git_metadata(root),
        "notes": args.notes or "",
    }
    copied_configs = snapshot_configuration(root, run_dir, args.extra_config)
    state["configuration_snapshots"] = copied_configs

    write_json(run_dir / "manifest" / "start_state.json", state)
    write_json(ACTIVE_STATE, state)

    (run_dir / "README.txt").write_text(
        "C1F experiment evidence bundle\n"
        "==============================\n"
        f"Run ID: {run_id}\n"
        f"Scenario: {scenario}\n"
        f"Mode: {mode}\n"
        f"Started: {state['start_time_iso']}\n\n"
        "Runtime loggers are NOT redirected into this directory. After the normal\n"
        "experiment terminals have been stopped, stop_experiment.sh copies only\n"
        "new/changed canonical logs into raw/. The originals remain untouched.\n"
    )

    print(f"Started experiment capture: {run_id}")
    print(f"Bundle: {run_dir}")
    print("Now start the normal experiment terminals.")
    return 0


def changed_since_start(
    rel: str,
    path: Path,
    start_inventory: dict[str, dict[str, int]],
) -> bool:
    stat = path.stat()
    old = start_inventory.get(rel)
    if old is None:
        return True
    return int(stat.st_size) != int(old.get("size", -1)) or int(stat.st_mtime_ns) != int(
        old.get("mtime_ns", -1)
    )


def destination_for(subsystem: str, source: Path, root: Path, run_dir: Path) -> Path:
    if subsystem in {"planner", "mpc"}:
        # Preserve the source run-directory name to keep backend/replans and
        # log/metadata pairs together if more than one node was started.
        return run_dir / "raw" / subsystem / source.parent.name / source.name
    return run_dir / "raw" / subsystem / source.name


def stop(args: argparse.Namespace) -> int:
    root = repo_root()
    if not ACTIVE_STATE.exists():
        raise RuntimeError("No active experiment marker found. Run start_experiment.sh first.")

    state = json.loads(ACTIVE_STATE.read_text())
    expected_root = Path(state.get("repo_root", "")).resolve()
    if expected_root != root:
        raise RuntimeError(
            f"Active experiment belongs to {expected_root}, not the current repository {root}."
        )

    run_dir = Path(state["run_dir"])
    start_inventory = state.get("start_inventory", {})
    collected: list[dict[str, object]] = []
    counts = {key: 0 for key in SOURCE_PATTERNS}

    for subsystem, source in iter_sources(root):
        rel = str(source.relative_to(root))
        if not changed_since_start(rel, source, start_inventory):
            continue
        destination = destination_for(subsystem, source, root, run_dir)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        stat = source.stat()
        counts[subsystem] += 1
        collected.append(
            {
                "subsystem": subsystem,
                "source": rel,
                "destination": str(destination.relative_to(run_dir)),
                "size_bytes": int(stat.st_size),
                "mtime_unix": float(stat.st_mtime),
            }
        )

    end_time = time.time()
    final_state = {
        **state,
        "end_time_unix": end_time,
        "end_time_iso": now_iso(),
        "duration_s": end_time - float(state["start_time_unix"]),
        "collected_counts": counts,
        "collected_file_count": len(collected),
    }
    final_state.pop("start_inventory", None)

    write_json(run_dir / "manifest" / "run_manifest.json", final_state)
    write_json(run_dir / "manifest.json", final_state)
    write_json(run_dir / "manifest" / "collected_files.json", collected)
    with (run_dir / "manifest" / "collected_files.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["subsystem", "source", "destination", "size_bytes", "mtime_unix"],
        )
        writer.writeheader()
        writer.writerows(collected)

    lines = [
        f"Run ID: {state['run_id']}",
        f"Started: {state['start_time_iso']}",
        f"Finished: {final_state['end_time_iso']}",
        f"Duration: {final_state['duration_s']:.1f} s",
        "",
        "Collected canonical logs:",
    ]
    for subsystem in SOURCE_PATTERNS:
        lines.append(f"  {subsystem}: {counts[subsystem]}")
    missing = [name for name, count in counts.items() if count == 0]
    if missing:
        lines.extend(
            [
                "",
                "No new/changed files were found for: " + ", ".join(missing),
                "This can be normal for a scenario that does not run every subsystem.",
            ]
        )
    lines.extend(
        [
            "",
            "The source files under logs/ were copied, not moved.",
            "Run the C1F.3 analyser on this bundle when evidence plots are wanted.",
        ]
    )
    (run_dir / "manifest" / "collection_summary.txt").write_text("\n".join(lines) + "\n")

    ACTIVE_STATE.unlink()

    print(f"Finished experiment capture: {state['run_id']}")
    print(f"Bundle: {run_dir}")
    for subsystem in SOURCE_PATTERNS:
        print(f"  {subsystem}: {counts[subsystem]} file(s)")
    if missing:
        print("Note: no new/changed files for " + ", ".join(missing))
    return 0


def status(_: argparse.Namespace) -> int:
    root = repo_root()
    if not ACTIVE_STATE.exists():
        print("No experiment is marked active.")
        return 0
    state = json.loads(ACTIVE_STATE.read_text())
    print(f"Active run: {state.get('run_id', 'unknown')}")
    print(f"Started: {state.get('start_time_iso', 'unknown')}")
    print(f"Bundle: {state.get('run_dir', 'unknown')}")
    print(f"Repository: {root}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_start = sub.add_parser("start", help="mark the start of one experiment")
    p_start.add_argument("scenario", help="short scenario label, e.g. c1f3a, c1f3b, irl_clear")
    p_start.add_argument(
        "mode", choices=("sim", "irl", "auto", "manual"), help="experiment mode"
    )
    p_start.add_argument("--notes", default="", help="optional short note stored in the manifest")
    p_start.add_argument(
        "--extra-config",
        action="append",
        default=[],
        help="additional repository-relative configuration file to snapshot",
    )
    p_start.add_argument(
        "--force",
        action="store_true",
        help="discard an existing active marker before starting this run",
    )
    p_start.set_defaults(func=start)

    p_stop = sub.add_parser("stop", help="collect logs changed since start")
    p_stop.set_defaults(func=stop)

    p_status = sub.add_parser("status", help="show the current capture marker")
    p_status.set_defaults(func=status)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"experiment_capture.py: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
