"""Standalone R2 visualization for cooperative dynamic planning.

Run from ``src/tejen_mission`` with::

    PYTHONPATH=. python3 -m tejen_mission.standalone_dynamic_planner

This is intentionally not yet Octopus/MINVO/optimization.  It validates the
planner's time-parametric world model and rendezvous timing in isolation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np

from .cooperative_trajectory import (
    AttachmentPointTrajectory,
    OffsetPointTrajectory,
    SinusoidalLineTrajectory,
    StaticRigidBodyTrajectory,
    TrajectoryState,
    basket_attachment_positions,
)
from .rendezvous import estimate_rendezvous_time
from .time_indexed_hulls import (
    AxisAlignedEnvelope,
    build_sinusoidal_hulls,
    maximum_centreline_containment_violation,
    uniform_intervals,
)


BOX_FACES = (
    (0, 1, 3, 2),
    (4, 5, 7, 6),
    (0, 1, 5, 4),
    (2, 3, 7, 6),
    (0, 2, 6, 4),
    (1, 3, 7, 5),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attachment-id", type=int, default=0)
    parser.add_argument("--num-intervals", type=int, default=4)
    parser.add_argument("--basket-x", type=float, default=2.0)
    parser.add_argument("--basket-y", type=float, default=2.0)
    parser.add_argument("--basket-z", type=float, default=1.2)
    parser.add_argument("--approach-height", type=float, default=0.30)
    parser.add_argument("--v-max", type=float, default=1.0)
    parser.add_argument("--a-max", type=float, default=1.5)
    parser.add_argument("--tracking-error", type=float, default=0.0)
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional image path, e.g. /tmp/r2_dynamic_world.png",
    )
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def time_coloured_line(ax, points: np.ndarray, times: np.ndarray, cmap, norm, *, linewidth: float, alpha: float = 1.0):
    segments = np.stack((points[:-1], points[1:]), axis=1)
    collection = Line3DCollection(segments, cmap=cmap, norm=norm, linewidth=linewidth, alpha=alpha)
    collection.set_array(0.5 * (times[:-1] + times[1:]))
    ax.add_collection3d(collection)
    return collection


def add_time_labels(ax, points: np.ndarray, times: np.ndarray, *, spacing_s: float = 1.0) -> None:
    if times[-1] <= 0.0:
        return
    label_times = np.arange(spacing_s, times[-1] + 1e-9, spacing_s)
    for target in label_times:
        index = int(np.argmin(np.abs(times - target)))
        p = points[index]
        ax.text(p[0], p[1], p[2] + 0.04, f"{times[index]:.1f}s", fontsize=7)


def draw_hull(ax, hull, cmap, norm) -> None:
    face_vertices = [[hull.vertices[index] for index in face] for face in BOX_FACES]
    color = cmap(norm(hull.t_mid))
    poly = Poly3DCollection(face_vertices, facecolors=[color], edgecolors=[color], alpha=0.11, linewidths=0.8)
    ax.add_collection3d(poly)


def set_equal_3d_axes(ax, points: np.ndarray) -> None:
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    centre = 0.5 * (lower + upper)
    span = max(float(np.max(upper - lower)), 1.0)
    half = 0.55 * span
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(max(0.0, centre[2] - half), centre[2] + half)


def main() -> None:
    args = parse_args()
    if not 0 <= args.attachment_id < 12:
        raise SystemExit("--attachment-id must be in [0, 11]")

    # R2 initial test geometry.  The start/approach altitude is 1.5 m so the
    # x=1 sinusoidal obstacle crosses the same vertical plane.
    start_state = TrajectoryState(
        position=np.array([0.0, 0.0, 1.5]),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
    )
    basket = StaticRigidBodyTrajectory(
        position=np.array([args.basket_x, args.basket_y, args.basket_z]),
        yaw=0.0,
    )
    attachment = AttachmentPointTrajectory(
        basket=basket,
        attachment_id=args.attachment_id,
        attachment_count=12,
        basket_radius_m=0.25,
    )
    approach = OffsetPointTrajectory(
        base=attachment,
        offset=np.array([0.0, 0.0, args.approach_height]),
    )

    limits_v = np.full(3, args.v_max, dtype=float)
    limits_a = np.full(3, args.a_max, dtype=float)
    rendezvous = estimate_rendezvous_time(
        start_state,
        approach,
        v_max=limits_v,
        a_max=limits_a,
        factor_alloc=1.0,
        factor_alloc_close=2.5,
        dist_factor_alloc_close_m=5.0,
    )
    if not rendezvous.converged:
        raise RuntimeError("Rendezvous timing fixed-point iteration did not converge")
    tf = rendezvous.duration_s

    obstacle = SinusoidalLineTrajectory()
    # The CineLog35 family frame is about 203 mm square including guards.  R2
    # keeps a slightly conservative provisional AABB and, importantly, keeps
    # tracking error separate.  Measure the actual V2 hardware before flight use.
    obstacle_envelope = AxisAlignedEnvelope(
        physical_half_extents_m=np.array([0.105, 0.105, 0.060]),
        tracking_error_m=args.tracking_error,
    )
    interval_edges = uniform_intervals(0.0, tf, args.num_intervals)
    hulls = build_sinusoidal_hulls(obstacle, obstacle_envelope, interval_edges)
    containment_violation = maximum_centreline_containment_violation(obstacle, hulls)

    times = np.linspace(0.0, tf, 401)
    obstacle_points = np.vstack([obstacle.state(float(t)).position for t in times])

    # Visualization-only nominal timing line.  This is not a collision-free
    # planner result.  Octopus will replace it in R4.  Its only purpose here is
    # to make same-time colour interpretation obvious in the R2 plot.
    s = times / tf if tf > 0.0 else np.zeros_like(times)
    smooth = 3.0 * s**2 - 2.0 * s**3
    nominal_points = (
        start_state.position[None, :]
        + smooth[:, None] * (rendezvous.goal_state.position - start_state.position)[None, :]
    )

    basket_state = basket.state(0.0)
    basket_points = basket_attachment_positions(
        basket_state,
        attachment_count=12,
        basket_radius_m=0.25,
    )
    theta = np.linspace(0.0, 2.0 * np.pi, 181)
    basket_ring = np.column_stack(
        (
            basket_state.position[0] + 0.25 * np.cos(theta),
            basket_state.position[1] + 0.25 * np.sin(theta),
            np.full_like(theta, basket_state.position[2]),
        )
    )

    cmap = plt.get_cmap("viridis")
    norm = colors.Normalize(vmin=0.0, vmax=max(tf, 1e-9))
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    time_coloured_line(ax, obstacle_points, times, cmap, norm, linewidth=3.0)
    time_coloured_line(ax, nominal_points, times, cmap, norm, linewidth=4.0, alpha=0.90)
    for hull in hulls:
        draw_hull(ax, hull, cmap, norm)

    ax.plot(basket_ring[:, 0], basket_ring[:, 1], basket_ring[:, 2], linestyle="--", linewidth=1.2, label="50 cm basket rim")
    ax.scatter(basket_points[:, 0], basket_points[:, 1], basket_points[:, 2], s=24, marker="o", label="12 attachment points")
    selected = attachment.state(tf).position
    goal = rendezvous.goal_state.position
    ax.scatter(*start_state.position, s=80, marker="^", label="pickup drone start")
    ax.scatter(*selected, s=95, marker="s", label=f"attachment_{args.attachment_id}")
    ax.scatter(*goal, s=115, marker="*", label="0.30 m approach state")
    ax.plot([selected[0], goal[0]], [selected[1], goal[1]], [selected[2], goal[2]], linestyle=":", linewidth=1.5)

    add_time_labels(ax, nominal_points, times, spacing_s=1.0)
    add_time_labels(ax, obstacle_points, times, spacing_s=1.0)

    all_points = np.vstack(
        [nominal_points, obstacle_points, basket_points, np.vstack([hull.vertices for hull in hulls])]
    )
    set_equal_3d_axes(ax, all_points)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("R2 standalone cooperative dynamic world\nSame colour = same planner time")
    existing_handles, existing_labels = ax.get_legend_handles_labels()
    time_proxies = [
        Line2D([0], [0], linewidth=4.0, label="time-coloured nominal transit (visualization only)"),
        Line2D([0], [0], linewidth=3.0, label="time-coloured moving obstacle centre"),
    ]
    ax.legend(handles=time_proxies + existing_handles, loc="upper left", fontsize=8)
    scalar = cm.ScalarMappable(norm=norm, cmap=cmap)
    scalar.set_array([])
    cbar = fig.colorbar(scalar, ax=ax, pad=0.10, shrink=0.72)
    cbar.set_label("Time [s]")
    fig.tight_layout()

    print("R2 standalone cooperative dynamic world")
    print(f"  basket centre [m]       : {basket_state.position}")
    print(f"  selected attachment     : attachment_{args.attachment_id}")
    print(f"  attachment position [m] : {selected}")
    print(f"  approach position [m]   : {goal}")
    print(f"  approach height [m]     : {args.approach_height:.3f}")
    print(f"  RMADER allocation factor: {rendezvous.allocation_factor:.3f}")
    print(f"  rendezvous duration Tf  : {tf:.6f} s")
    print(f"  obstacle trajectory     : x=1.0, y=0.70+1.40*sin(0.20*t), z=1.50")
    print(f"  obstacle half extents   : {obstacle_envelope.physical_half_extents_m}")
    print(f"  tracking error margin   : {args.tracking_error:.3f} m")
    print(f"  time-indexed hulls      : {len(hulls)}")
    print(f"  containment violation   : {containment_violation:.12f} m")
    print("  hull containment        : PASS" if containment_violation <= 1e-10 else "  hull containment        : FAIL")
    print("  NOTE: coloured ego line is timing visualization only, not an avoidance plan.")

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=180, bbox_inches="tight")
        print(f"  saved figure            : {args.save}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
