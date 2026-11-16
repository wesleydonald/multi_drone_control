"""Standalone R3 visual diagnostics for the permanent convex separator.

Examples, from ``src/tejen_mission``::

    PYTHONPATH=. python3 -m tejen_mission.visualize_separator --scene basics
    PYTHONPATH=. python3 -m tejen_mission.visualize_separator --scene mixed

The mixed scene contains static obstacles, the R2 sinusoidal moving obstacle,
and a second constant-velocity obstacle.  Moving hulls use one shared time
colormap.  Separator status is encoded with planes/annotations, not by changing
the time colours.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np

from .cooperative_trajectory import SinusoidalLineTrajectory
from .convex_geometry import (
    box_vertices_from_center,
    plane_patch_vertices,
    rotate_vertices,
    rotation_z,
    swept_box_vertices_linear,
    triangular_faces,
    unique_hull_edges,
)
from .separator import SeparatingPlane, glpk_available, solve_separator
from .time_indexed_hulls import AxisAlignedEnvelope, build_sinusoidal_hulls, uniform_intervals


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("basics", "mixed"), default="mixed")
    parser.add_argument("--backend", choices=("auto", "glpk", "scipy"), default="auto")
    parser.add_argument("--inspect-interval", type=int, default=1, help="Selected moving-obstacle interval in the mixed scene")
    parser.add_argument("--num-intervals", type=int, default=4)
    parser.add_argument("--horizon", type=float, default=7.291666666666667)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def _add_polytope(ax, vertices: np.ndarray, *, facecolor, edgecolor, alpha: float, linewidth: float = 1.0, label: str | None = None, linestyle: str = "-") -> None:
    faces = triangular_faces(vertices)
    polygons = [[vertices[index] for index in face] for face in faces]
    poly = Poly3DCollection(polygons, facecolors=[facecolor], edgecolors=[edgecolor], alpha=alpha, linewidths=0.45)
    ax.add_collection3d(poly)
    for a, b in unique_hull_edges(vertices):
        p, q = vertices[a], vertices[b]
        ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], color=edgecolor, linewidth=linewidth, linestyle=linestyle)
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], s=10, color=edgecolor, alpha=min(1.0, alpha + 0.35), label=label)


def _add_plane(ax, plane: SeparatingPlane, reference_points: np.ndarray, *, facecolor, label: str, show_margin_planes: bool = True) -> None:
    patch = plane_patch_vertices(plane, reference_points, scale=1.35, level=0.0)
    ax.add_collection3d(Poly3DCollection([patch], facecolors=[facecolor], edgecolors=[facecolor], alpha=0.18, linewidths=1.0))

    if show_margin_planes:
        for level in (-1.0, 1.0):
            margin_patch = plane_patch_vertices(plane, reference_points, scale=1.22, level=level)
            loop = np.vstack((margin_patch, margin_patch[0]))
            ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color=facecolor, linewidth=0.8, linestyle="--", alpha=0.75)

    anchor = np.mean(patch, axis=0)
    n = plane.unit_normal
    span = max(float(np.ptp(reference_points[:, 0])), float(np.ptp(reference_points[:, 1])), float(np.ptp(reference_points[:, 2])), 0.5)
    ax.quiver(anchor[0], anchor[1], anchor[2], n[0], n[1], n[2], length=0.28 * span, normalize=True, color=facecolor, linewidth=1.5)
    ax.text(anchor[0], anchor[1], anchor[2], label, fontsize=7)


def _equal_axes(ax, points: np.ndarray) -> None:
    lo = np.min(points, axis=0)
    hi = np.max(points, axis=0)
    centre = 0.5 * (lo + hi)
    span = max(float(np.max(hi - lo)), 1.0)
    half = 0.58 * span
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(max(0.0, centre[2] - half), centre[2] + half)


def _annotate_result(ax, result, centre: np.ndarray, prefix: str) -> None:
    if result.feasible:
        text = f"{prefix}: SEPARABLE\n{result.backend}\ngap={result.geometric_gap_m:.3f} m"
    else:
        text = f"{prefix}: NO SEPARATOR\n{result.backend}"
    ax.text(centre[0], centre[1], centre[2] + 0.10, text, fontsize=8)


def _run_basic_cases(backend: str):
    fig = plt.figure(figsize=(12, 10))
    cases = []
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.25, 0.22, 0.18]))
    cases.append(("Clearly disjoint", first, box_vertices_from_center(np.array([1.15, 0.12, 1.05]), np.array([0.25, 0.22, 0.18]))))
    cases.append(("Near-touching gap", first, box_vertices_from_center(np.array([0.54, 0.0, 1.0]), np.array([0.25, 0.22, 0.18]))))
    cases.append(("Touching", first, box_vertices_from_center(np.array([0.50, 0.0, 1.0]), np.array([0.25, 0.22, 0.18]))))
    rotated = rotate_vertices(box_vertices_from_center(np.array([0.35, 0.06, 1.0]), np.array([0.28, 0.18, 0.16])), rotation_z(math.radians(28.0)))
    cases.append(("Overlapping rotated", first, rotated))

    for index, (title, a, b) in enumerate(cases, start=1):
        ax = fig.add_subplot(2, 2, index, projection="3d")
        result = solve_separator(a, b, backend=backend)
        _add_polytope(ax, a, facecolor="tab:blue", edgecolor="tab:blue", alpha=0.14, linewidth=1.2, label="convex set A")
        _add_polytope(ax, b, facecolor="tab:orange", edgecolor="tab:orange", alpha=0.14, linewidth=1.2, label="convex set B")
        reference = np.vstack((a, b))
        if result.feasible and result.plane is not None:
            _add_plane(ax, result.plane, reference, facecolor="tab:green", label="n")
        else:
            ax.text(np.mean(reference[:, 0]), np.mean(reference[:, 1]), np.max(reference[:, 2]) + 0.10, "NO SEPARATOR", fontsize=9, weight="bold")
        _equal_axes(ax, reference)
        ax.set_title(f"{title}\n{'FEASIBLE' if result.feasible else 'INFEASIBLE'}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        if index == 1:
            ax.legend(loc="upper left", fontsize=7)
        print(f"{title:22s}: {'PASS' if result.feasible else 'FAIL'} via {result.backend}")
    fig.suptitle("R3 separator fundamentals: hulls, central plane, ±1 constraint planes", fontsize=13)
    fig.tight_layout()
    return fig


def _constant_velocity_hulls(interval_edges: np.ndarray, *, p0: np.ndarray, velocity: np.ndarray, half: np.ndarray):
    hulls = []
    for index, (t0, t1) in enumerate(zip(interval_edges[:-1], interval_edges[1:])):
        c0 = p0 + velocity * float(t0)
        c1 = p0 + velocity * float(t1)
        hulls.append((index, float(t0), float(t1), swept_box_vertices_linear(c0, c1, half)))
    return hulls


def _run_mixed_scene(backend: str, inspect_interval: int, num_intervals: int, horizon: float):
    if num_intervals <= 0:
        raise SystemExit("--num-intervals must be positive")
    if horizon <= 0.0:
        raise SystemExit("--horizon must be positive")
    if not 0 <= inspect_interval < num_intervals:
        raise SystemExit(f"--inspect-interval must be in [0, {num_intervals - 1}]")

    edges = uniform_intervals(0.0, horizon, num_intervals)
    cmap = plt.get_cmap("viridis")
    norm = colors.Normalize(vmin=0.0, vmax=horizon)

    # Generic convex query object used only by this permanent separator demo.
    # It is intentionally not an ego trajectory approximation.
    query = rotate_vertices(
        box_vertices_from_center(np.array([1.05, 1.18, 1.50]), np.array([0.20, 0.16, 0.13])),
        rotation_z(math.radians(18.0)),
    )

    static_obstacles = [
        ("static-A", box_vertices_from_center(np.array([0.32, 1.55, 1.35]), np.array([0.18, 0.26, 0.20]))),
        ("static-B", rotate_vertices(box_vertices_from_center(np.array([1.75, 0.72, 1.62]), np.array([0.24, 0.16, 0.18])), rotation_z(math.radians(-25.0)))),
    ]

    sinusoid = SinusoidalLineTrajectory()
    envelope = AxisAlignedEnvelope(np.array([0.105, 0.105, 0.060]), tracking_error_m=0.0)
    sinusoid_hulls = build_sinusoidal_hulls(sinusoid, envelope, edges)

    # Second moving obstacle: deterministic constant-velocity motion.  Its
    # selected interval is intentionally separable from the query so the mixed
    # inspector shows both a valid plane and an infeasible collision at once.
    cv_p0 = np.array([0.45, 2.10, 1.28])
    cv_velocity = np.array([0.18, -0.025, 0.035])
    cv_hulls = _constant_velocity_hulls(
        edges,
        p0=cv_p0,
        velocity=cv_velocity,
        half=np.array([0.12, 0.10, 0.075]),
    )

    fig = plt.figure(figsize=(15, 7.8))
    ax_context = fig.add_subplot(121, projection="3d")
    ax_inspect = fig.add_subplot(122, projection="3d")
    all_points = [query]

    # ----------------------------- context panel -----------------------------
    _add_polytope(ax_context, query, facecolor="tab:blue", edgecolor="tab:blue", alpha=0.15, linewidth=1.5, label="generic query hull")
    for name, vertices in static_obstacles:
        _add_polytope(ax_context, vertices, facecolor="0.72", edgecolor="0.38", alpha=0.08, linewidth=0.85, label=name)
        all_points.append(vertices)
    for hull in sinusoid_hulls:
        color = cmap(norm(hull.t_mid))
        selected = hull.interval_index == inspect_interval
        _add_polytope(ax_context, hull.vertices, facecolor=color, edgecolor=color, alpha=0.14 if selected else 0.025, linewidth=1.35 if selected else 0.45, label="sinusoidal hulls" if hull.interval_index == 0 else None)
        all_points.append(hull.vertices)
    for index, t0, t1, vertices in cv_hulls:
        color = cmap(norm(0.5 * (t0 + t1)))
        selected = index == inspect_interval
        _add_polytope(ax_context, vertices, facecolor=color, edgecolor=color, alpha=0.14 if selected else 0.025, linewidth=1.35 if selected else 0.45, linestyle="--", label="constant-velocity hulls" if index == 0 else None)
        all_points.append(vertices)

    dense_t = np.linspace(0.0, horizon, 301)
    sin_points = np.vstack([sinusoid.state(float(t)).position for t in dense_t])
    sin_segments = np.stack((sin_points[:-1], sin_points[1:]), axis=1)
    sin_line = Line3DCollection(sin_segments, cmap=cmap, norm=norm, linewidth=2.2)
    sin_line.set_array(0.5 * (dense_t[:-1] + dense_t[1:]))
    ax_context.add_collection3d(sin_line)
    cv_points = cv_p0[None, :] + dense_t[:, None] * cv_velocity[None, :]
    cv_segments = np.stack((cv_points[:-1], cv_points[1:]), axis=1)
    cv_line = Line3DCollection(cv_segments, cmap=cmap, norm=norm, linewidth=2.0, linestyles="dashed")
    cv_line.set_array(0.5 * (dense_t[:-1] + dense_t[1:]))
    ax_context.add_collection3d(cv_line)
    all_points.extend((sin_points, cv_points))

    _equal_axes(ax_context, np.vstack(all_points))
    ax_context.set_xlabel("x [m]")
    ax_context.set_ylabel("y [m]")
    ax_context.set_zlabel("z [m]")
    ax_context.set_title("Time-indexed context\nselected interval shown more strongly")
    ax_context.legend(loc="upper left", fontsize=7)

    # ---------------------------- inspector panel ----------------------------
    _add_polytope(ax_inspect, query, facecolor="tab:blue", edgecolor="tab:blue", alpha=0.18, linewidth=1.65, label="query hull")
    inspect_points = [query]
    result_rows = []

    # Static obstacles are solved and reported here, but kept out of the right-hand
    # inspector because the basics scene already visualizes static separator planes.
    # This keeps the selected-time moving-obstacle geometry legible.
    for name, vertices in static_obstacles:
        result = solve_separator(query, vertices, backend=backend)
        result_rows.append((name, "static", result))

    selected_sinusoid = sinusoid_hulls[inspect_interval]
    sin_color = cmap(norm(selected_sinusoid.t_mid))
    _add_polytope(ax_inspect, selected_sinusoid.vertices, facecolor=sin_color, edgecolor=sin_color, alpha=0.22, linewidth=1.65, label=f"sinusoid I{inspect_interval}")
    sin_result = solve_separator(query, selected_sinusoid.vertices, backend=backend)
    result_rows.append(("sinusoid", f"I{inspect_interval}", sin_result))
    if sin_result.feasible and sin_result.plane is not None:
        _add_plane(ax_inspect, sin_result.plane, np.vstack((query, selected_sinusoid.vertices)), facecolor=sin_color, label="sinusoid plane")
    else:
        _annotate_result(ax_inspect, sin_result, np.mean(selected_sinusoid.vertices, axis=0), "sinusoid")
    inspect_points.append(selected_sinusoid.vertices)

    _, cv_t0, cv_t1, selected_cv = cv_hulls[inspect_interval]
    cv_color = cmap(norm(0.5 * (cv_t0 + cv_t1)))
    _add_polytope(ax_inspect, selected_cv, facecolor=cv_color, edgecolor=cv_color, alpha=0.20, linewidth=1.65, linestyle="--", label=f"CV I{inspect_interval}")
    cv_result = solve_separator(query, selected_cv, backend=backend)
    result_rows.append(("constant-velocity", f"I{inspect_interval}", cv_result))
    if cv_result.feasible and cv_result.plane is not None:
        _add_plane(ax_inspect, cv_result.plane, np.vstack((query, selected_cv)), facecolor=cv_color, label="CV plane")
    else:
        _annotate_result(ax_inspect, cv_result, np.mean(selected_cv, axis=0), "CV")
    inspect_points.append(selected_cv)

    _equal_axes(ax_inspect, np.vstack(inspect_points))
    ax_inspect.set_xlabel("x [m]")
    ax_inspect.set_ylabel("y [m]")
    ax_inspect.set_zlabel("z [m]")
    ax_inspect.set_title("Selected-interval separator inspector\nplanes only where the LP is feasible")
    ax_inspect.legend(loc="upper left", fontsize=7)

    fig.suptitle(
        f"R3 mixed separator scene, I{inspect_interval} = "
        f"[{edges[inspect_interval]:.2f}, {edges[inspect_interval + 1]:.2f}] s\n"
        "Moving hull colour = time; static obstacles stay neutral",
        fontsize=13,
    )
    scalar = cm.ScalarMappable(norm=norm, cmap=cmap)
    scalar.set_array([])
    cbar = fig.colorbar(scalar, ax=[ax_context, ax_inspect], pad=0.07, shrink=0.72)
    cbar.set_label("Time [s]")
    fig.subplots_adjust(left=0.02, right=0.91, bottom=0.05, top=0.86, wspace=0.05)

    print("R3 mixed separator scene")
    print(f"  selected interval: I{inspect_interval} [{edges[inspect_interval]:.3f}, {edges[inspect_interval + 1]:.3f}] s")
    print(f"  GLPK available   : {glpk_available()}")
    for name, interval, result in result_rows:
        state = "SEPARABLE" if result.feasible else "NO SEPARATOR"
        gap = "" if result.geometric_gap_m is None else f", gap={result.geometric_gap_m:.4f} m"
        print(f"  {name:18s} {interval:8s}: {state:12s} via {result.backend}{gap}")
    return fig

def main() -> None:
    args = parse_args()
    if args.scene == "basics":
        fig = _run_basic_cases(args.backend)
    else:
        fig = _run_mixed_scene(args.backend, args.inspect_interval, args.num_intervals, args.horizon)

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=180, bbox_inches="tight")
        print(f"  saved figure      : {args.save}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
