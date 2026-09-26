#!/usr/bin/env python3
"""Plot R6.3B/B.3 receding-horizon missions in native Matplotlib 3D.

Supports the existing static receding_horizon_demo outputs and the B.3
cooperative_mission_demo other-drone CSV. By default this opens Matplotlib's
interactive desktop figure window so the scene can be rotated, panned and
zoomed. PNG saving is optional.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]
    physical_half: tuple[float, float, float]


@dataclass(frozen=True)
class Scene:
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    obstacles: tuple[Box, ...]


# Exact scene definitions from apps/receding_horizon_demo.cpp in the R6.3B.2 baseline.
EGO_HALF = np.array([0.105, 0.105, 0.060], dtype=float)
SCENES: dict[str, Scene] = {
    "straight": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "crossing": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "same_direction": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "two_drones": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "crossing_stress": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "same_direction_stress": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "two_drones_stress": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.0, 0.0, 1.5),
        obstacles=(),
    ),
    "diagonal": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.2, 2.2, 1.5),
        obstacles=(),
    ),
    "long_chicane": Scene(
        start=(0.0, 0.0, 1.5),
        goal=(3.2, 2.4, 1.5),
        obstacles=(
            Box("chicane-L", (0.78, 0.88, 1.50), (0.18, 0.34, 0.24)),
            Box("chicane-R", (1.48, 1.10, 1.50), (0.22, 0.34, 0.24)),
        ),
    ),
}

TRAJECTORY_REQUIRED = ("time", "x", "y", "z")
REPLANS_REQUIRED = (
    "index",
    "now",
    "status",
    "accepted",
    "splice_time",
    "Ax",
    "Ay",
    "Az",
    "Gx",
    "Gy",
    "Gz",
)

OTHER_DRONES_REQUIRED = (
    "time", "name", "x", "y", "z",
    "physical_half_x", "physical_half_y", "physical_half_z",
    "tracking_half_x", "tracking_half_y", "tracking_half_z",
)


def fail(message: str) -> "NoReturn":
    raise ValueError(message)


def read_csv_rows(path: Path, required: Sequence[str], label: str) -> list[dict[str, str]]:
    if not path.is_file():
        fail(f"{label} CSV does not exist: {path}")

    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            fail(f"{label} CSV has no header: {path}")
        fields = [f.strip() for f in reader.fieldnames]
        missing = [name for name in required if name not in fields]
        if missing:
            fail(
                f"{label} CSV is missing required column(s): {', '.join(missing)}\n"
                f"  file: {path}\n"
                f"  found: {', '.join(fields)}"
            )
        rows = list(reader)

    if not rows:
        fail(f"{label} CSV contains no data rows: {path}")
    return rows


def floats(rows: Sequence[dict[str, str]], columns: Sequence[str], label: str) -> np.ndarray:
    out = np.empty((len(rows), len(columns)), dtype=float)
    for i, row in enumerate(rows, start=2):
        for j, col in enumerate(columns):
            raw = (row.get(col) or "").strip()
            try:
                value = float(raw)
            except ValueError as exc:
                fail(f"{label} CSV row {i}: column '{col}' is not numeric: {raw!r}")
            if not math.isfinite(value):
                fail(f"{label} CSV row {i}: column '{col}' is not finite: {raw!r}")
            out[i - 2, j] = value
    return out


def accepted_value(raw: str) -> bool:
    value = raw.strip().lower()
    if value in {"1", "true", "t", "yes", "y"}:
        return True
    if value in {"0", "false", "f", "no", "n"}:
        return False
    fail(f"replans CSV has unrecognised boolean in 'accepted': {raw!r}")


def evenly_spaced_indices(count: int, max_count: int) -> np.ndarray:
    if count <= 0 or max_count <= 0:
        return np.array([], dtype=int)
    if count <= max_count:
        return np.arange(count, dtype=int)
    # Unique after rounding, always includes first and last.
    return np.unique(np.rint(np.linspace(0, count - 1, max_count)).astype(int))


def box_vertices(center: np.ndarray, half: np.ndarray) -> np.ndarray:
    return np.array(
        [
            center + np.array([sx * half[0], sy * half[1], sz * half[2]])
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=float,
    )


def box_faces(vertices: np.ndarray) -> list[list[np.ndarray]]:
    # vertex index = x_bit*4 + y_bit*2 + z_bit
    quads = [
        (0, 1, 3, 2),
        (4, 5, 7, 6),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (0, 2, 6, 4),
        (1, 3, 7, 5),
    ]
    return [[vertices[i] for i in q] for q in quads]


def box_edges() -> tuple[tuple[int, int], ...]:
    return (
        (0, 1), (0, 2), (0, 4),
        (1, 3), (1, 5),
        (2, 3), (2, 6),
        (3, 7),
        (4, 5), (4, 6),
        (5, 7), (6, 7),
    )


def add_obstacles(ax, scene: Scene, show_inflated: bool) -> list[np.ndarray]:
    all_vertices: list[np.ndarray] = []
    physical_label_used = False
    inflated_label_used = False

    for obstacle in scene.obstacles:
        center = np.array(obstacle.center, dtype=float)
        physical_half = np.array(obstacle.physical_half, dtype=float)
        physical_vertices = box_vertices(center, physical_half)
        all_vertices.append(physical_vertices)

        poly = Poly3DCollection(
            box_faces(physical_vertices),
            alpha=0.18,
            linewidths=0.8,
            edgecolors="0.35",
        )
        # Do not attach a legend label directly to Poly3DCollection. Some
        # Matplotlib releases cannot construct 3D collection legend handles and
        # raise AttributeError on _facecolors2d/_edgecolors2d. A 2D Patch proxy
        # is added to the legend below instead.
        if not physical_label_used:
            physical_label_used = True
        ax.add_collection3d(poly)

        if show_inflated:
            inflated_vertices = box_vertices(center, physical_half + EGO_HALF)
            all_vertices.append(inflated_vertices)
            for edge_index, (i, j) in enumerate(box_edges()):
                label = None
                if not inflated_label_used and edge_index == 0:
                    label = "planner-inflated obstacle"
                    inflated_label_used = True
                ax.plot(
                    [inflated_vertices[i, 0], inflated_vertices[j, 0]],
                    [inflated_vertices[i, 1], inflated_vertices[j, 1]],
                    [inflated_vertices[i, 2], inflated_vertices[j, 2]],
                    linestyle="--",
                    linewidth=0.9,
                    alpha=0.7,
                    color="0.35",
                    label=label,
                )
    return all_vertices


def set_equal_3d_limits(ax, point_sets: Iterable[np.ndarray], pad_fraction: float = 0.08) -> None:
    arrays = [np.asarray(points, dtype=float).reshape(-1, 3) for points in point_sets if np.size(points)]
    if not arrays:
        return
    points = np.vstack(arrays)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = np.maximum(maxs - mins, 1e-6)
    half = 0.5 * float(span.max()) * (1.0 + 2.0 * pad_fraction)
    # Avoid an excessively thin plot for the straight scene while keeping equal physical scale.
    half = max(half, 0.55)
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot R6.3B receding-horizon trajectory/replan CSVs in 3D."
    )
    parser.add_argument("--trajectory", required=True, type=Path, help="*_trajectory.csv")
    parser.add_argument("--replans", required=True, type=Path, help="*_replans.csv")
    parser.add_argument(
        "--other-drones",
        type=Path,
        help="optional B.3 *_other_drones.csv containing fixed shared trajectories",
    )
    parser.add_argument("--scene", required=True, choices=sorted(SCENES), help="built-in demo scene")
    parser.add_argument(
        "--output",
        type=Path,
        help="optional PNG output path; the interactive Matplotlib window still opens by default",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open the native Matplotlib window (useful for SSH/headless runs)",
    )
    parser.add_argument(
        "--max-local-lines",
        type=int,
        default=16,
        help="maximum accepted splice A->local-goal lines to draw (default: 16)",
    )
    parser.add_argument(
        "--time-markers",
        type=int,
        default=0,
        help="draw this many evenly spaced executed-path time markers (default: 0)",
    )
    parser.add_argument(
        "--include-rejected",
        action="store_true",
        help="also draw rejected replan splice/local-goal points and local-goal lines",
    )
    parser.add_argument(
        "--no-inflated-obstacles",
        action="store_true",
        help="hide the planner's ego-inflated C-space obstacle wireframes",
    )
    parser.add_argument(
        "--other-boxes",
        action="store_true",
        help=(
            "at selected --time-markers, draw effective other-drone C-space "
            "wireframes (planning body + other body + tracking tube)"
        ),
    )
    parser.add_argument("--elev", type=float, default=24.0, help="camera elevation in degrees")
    parser.add_argument("--azim", type=float, default=-58.0, help="camera azimuth in degrees")
    parser.add_argument("--dpi", type=int, default=180, help="PNG resolution (default: 180)")
    args = parser.parse_args()
    if args.max_local_lines < 0:
        parser.error("--max-local-lines must be >= 0")
    if args.time_markers < 0:
        parser.error("--time-markers must be >= 0")
    if args.dpi <= 0:
        parser.error("--dpi must be > 0")
    return args


def main() -> int:
    args = parse_args()
    scene = SCENES[args.scene]

    trajectory_rows = read_csv_rows(args.trajectory, TRAJECTORY_REQUIRED, "trajectory")
    replan_rows = read_csv_rows(args.replans, REPLANS_REQUIRED, "replans")
    other_rows = (
        read_csv_rows(args.other_drones, OTHER_DRONES_REQUIRED, "other-drones")
        if args.other_drones is not None
        else []
    )

    trajectory_xyz = floats(trajectory_rows, ("x", "y", "z"), "trajectory")
    trajectory_time = floats(trajectory_rows, ("time",), "trajectory")[:, 0]
    replan_a = floats(replan_rows, ("Ax", "Ay", "Az"), "replans")
    replan_g = floats(replan_rows, ("Gx", "Gy", "Gz"), "replans")
    accepted_mask = np.array([accepted_value(row["accepted"]) for row in replan_rows], dtype=bool)

    other_groups: dict[str, dict[str, np.ndarray]] = {}
    if other_rows:
        names = sorted({(row.get("name") or "").strip() for row in other_rows})
        if not names or any(not name for name in names):
            fail("other-drones CSV contains an empty 'name'")
        for name in names:
            rows = [row for row in other_rows if (row.get("name") or "").strip() == name]
            times = floats(rows, ("time",), "other-drones")[:, 0]
            xyz = floats(rows, ("x", "y", "z"), "other-drones")
            physical = floats(
                rows, ("physical_half_x", "physical_half_y", "physical_half_z"), "other-drones"
            )
            tracking = floats(
                rows, ("tracking_half_x", "tracking_half_y", "tracking_half_z"), "other-drones"
            )
            if not np.all(np.diff(times) >= -1e-12):
                fail(f"other-drones CSV times are not monotonic for {name!r}")
            if np.any(physical < 0.0) or np.any(tracking < 0.0):
                fail(f"other-drones CSV contains negative half-extents for {name!r}")
            other_groups[name] = {
                "time": times,
                "xyz": xyz,
                "physical": physical,
                "tracking": tracking,
            }

    if not np.all(np.diff(trajectory_time) >= -1e-12):
        fail("trajectory CSV 'time' column is not monotonically nondecreasing")

    visible_mask = np.ones(len(replan_rows), dtype=bool) if args.include_rejected else accepted_mask
    visible_indices = np.flatnonzero(visible_mask)
    local_line_pick = evenly_spaced_indices(len(visible_indices), args.max_local_lines)
    local_line_indices = visible_indices[local_line_pick] if len(local_line_pick) else np.array([], dtype=int)

    output = args.output
    if output is not None:
        if output.suffix.lower() not in {".png", ".pdf", ".svg"}:
            fail("--output must use .png, .pdf, or .svg for the native Matplotlib plotter")
        output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(10.5, 8.0))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        trajectory_xyz[:, 0],
        trajectory_xyz[:, 1],
        trajectory_xyz[:, 2],
        linewidth=2.2,
        label="planning-drone executed path",
    )

    for name, group in other_groups.items():
        xyz = group["xyz"]
        ax.plot(
            xyz[:, 0], xyz[:, 1], xyz[:, 2],
            linewidth=1.7, linestyle="--", alpha=0.8,
            label=f"{name} shared trajectory",
        )

    start = np.array(scene.start, dtype=float)
    goal = np.array(scene.goal, dtype=float)
    ax.scatter(*start, marker="o", s=70, label="start", depthshade=False)
    ax.scatter(*goal, marker="*", s=150, label="global goal", depthshade=False)

    if len(visible_indices):
        vis_a = replan_a[visible_indices]
        vis_g = replan_g[visible_indices]
        ax.scatter(vis_a[:, 0], vis_a[:, 1], vis_a[:, 2], s=12, alpha=0.45, label="splice A")
        ax.scatter(vis_g[:, 0], vis_g[:, 1], vis_g[:, 2], s=15, alpha=0.45, marker="x", label="local goal G")

    for n, idx in enumerate(local_line_indices):
        a = replan_a[idx]
        g = replan_g[idx]
        ax.plot(
            [a[0], g[0]], [a[1], g[1]], [a[2], g[2]],
            linewidth=0.8,
            alpha=0.35,
            linestyle=":",
            label="selected A to local-goal" if n == 0 else None,
        )

    other_box_vertex_sets: list[np.ndarray] = []
    if args.time_markers > 0:
        marker_indices = evenly_spaced_indices(len(trajectory_xyz), args.time_markers)
        pts = trajectory_xyz[marker_indices]
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=24, marker="|", label="planning-drone time progression")
        for idx in marker_indices:
            p = trajectory_xyz[idx]
            t = trajectory_time[idx]
            ax.text(p[0], p[1], p[2], f" {t:.1f}s", fontsize=7)

        other_marker_label_used = False
        other_box_label_used = False
        for name, group in other_groups.items():
            times = group["time"]
            xyz = group["xyz"]
            for idx in marker_indices:
                t = trajectory_time[idx]
                j = int(np.argmin(np.abs(times - t)))
                q = xyz[j]
                ax.scatter(
                    q[0], q[1], q[2], s=24, marker="^", alpha=0.75,
                    label="time-aligned other-drone position" if not other_marker_label_used else None,
                )
                other_marker_label_used = True
                if args.other_boxes:
                    half = EGO_HALF + group["physical"][j] + group["tracking"][j]
                    vertices = box_vertices(q, half)
                    other_box_vertex_sets.append(vertices)
                    for edge_index, (a, b) in enumerate(box_edges()):
                        label = None
                        if not other_box_label_used and edge_index == 0:
                            label = "time-aligned other-drone C-space tube"
                            other_box_label_used = True
                        ax.plot(
                            [vertices[a, 0], vertices[b, 0]],
                            [vertices[a, 1], vertices[b, 1]],
                            [vertices[a, 2], vertices[b, 2]],
                            linestyle=":", linewidth=0.7, alpha=0.35, color="0.50", label=label,
                        )

    obstacle_vertex_sets = add_obstacles(ax, scene, show_inflated=not args.no_inflated_obstacles)

    # Keep rejected points visible when requested, without letting them dominate the normal view.
    if args.include_rejected and np.any(~accepted_mask):
        rejected = np.flatnonzero(~accepted_mask)
        pts = replan_a[rejected]
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], marker="X", s=34, alpha=0.8, label="rejected splice A")

    accepted_count = int(accepted_mask.sum())
    rejected_count = int((~accepted_mask).sum())
    duration = trajectory_time[-1] - trajectory_time[0]
    milestone = "R6.3B.3 cooperative mission" if other_groups else "R6.3B receding-horizon trajectory"
    ax.set_title(
        f"{milestone} - {args.scene}\n"
        f"{duration:.2f} s executed, {accepted_count} accepted replans, {rejected_count} rejected"
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=args.elev, azim=args.azim)
    ax.grid(True, alpha=0.3)

    point_sets: list[np.ndarray] = [trajectory_xyz, start[None, :], goal[None, :], replan_a, replan_g]
    point_sets.extend(group["xyz"] for group in other_groups.values())
    point_sets.extend(obstacle_vertex_sets)
    point_sets.extend(other_box_vertex_sets)
    set_equal_3d_limits(ax, point_sets)

    # Deduplicate labels because each obstacle contributes several primitives.
    handles, labels = ax.get_legend_handles_labels()
    unique: dict[str, object] = {}
    for handle, label in zip(handles, labels):
        if label and label not in unique:
            unique[label] = handle

    # Matplotlib 3D Poly3DCollection legend handling varies by release. Use a
    # simple 2D proxy patch for the physical obstacle so the same script works
    # on Ubuntu/system Matplotlib as well as newer environments.
    if scene.obstacles:
        unique.setdefault(
            "physical obstacle",
            Patch(facecolor="0.8", edgecolor="0.35", alpha=0.35),
        )

    ax.legend(unique.values(), unique.keys(), loc="upper left", fontsize=8)

    fig.tight_layout()

    if output is not None:
        fig.savefig(output, dpi=args.dpi, bbox_inches="tight")
        print(f"Saved 3D plot: {output}")

    other_summary = (
        f", other-drone rows={len(other_rows)}, other drones={len(other_groups)}"
        if other_rows else ""
    )
    print(
        f"Rows: trajectory={len(trajectory_rows)}, replans={len(replan_rows)}"
        f"{other_summary}; accepted={accepted_count}, rejected={rejected_count}; "
        f"local-goal lines drawn={len(local_line_indices)}"
    )

    if not args.no_show:
        backend = matplotlib.get_backend()
        print(f"Opening interactive Matplotlib 3D window using backend: {backend}")
        print("Drag in the 3D axes to rotate; use the toolbar for pan/zoom/reset.")
        plt.show()

    plt.close(fig)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"plot_receding_horizon_3d: ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
