"""ROS-independent Bernstein/Bezier smoothing inside a convex safe-flight corridor.

Each degree-7 Bezier segment is assigned to one convex cell ``A @ x <= b``.
Constraining every control point to its owning cell guarantees containment of the
whole polynomial segment by the Bezier convex-hull property.  The fixed-time QP
minimises integrated squared snap with exact p/v/a endpoint conditions and C3
inter-segment continuity.

Velocity and acceleration feasibility follows the Bernstein hodograph approach
used by Gao et al. / Btraj: each Cartesian component of every derivative control
point is bounded by linear inequalities.  If the initial timing probe exceeds the
tighter planning limits, segment times are stretched once and the QP is solved
once more with those hard dynamic constraints.  The final dynamic gate uses a
configurable headroom ratio.  Geometry receives no analogous headroom.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Optional, Sequence, Tuple

import numpy as np

from .convex_corridor import ConvexPolyhedron


_DEGREE = 7
_NCTRL = 8

# Numerical solver acceptance only, not physical safety margins.
_POS_TOL_M = 2e-4
_VEL_TOL_MPS = 1e-2
_ACC_TOL_MPS2 = 5e-2
_JERK_JOIN_TOL_MPS3 = 5.0
_CORRIDOR_TOL_M = 1e-4


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    value = np.asarray(value, dtype=float).reshape(-1)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return value.copy()


@dataclass(frozen=True)
class ConvexTrajectoryConfig:
    enabled: bool = True

    # Extra whole-body C-space clearance applied before A*/SFC generation.
    planning_clearance_margin_m: float = 0.05

    # Gao/Btraj-style Cartesian derivative limits.  These are per-axis
    # Bernstein hodograph bounds, not Euclidean vector-norm limits.
    # Planning limits are deliberately tighter than final validation limits.
    max_speed_mps: float = 1.0
    max_acceleration_mps2: float = 1.5
    validation_headroom_ratio: float = 1.10

    nominal_speed_mps: float = 0.50
    minimum_segment_time_s: float = 0.18
    maximum_segment_time_s: float = 6.0

    # One initial solve + at most one deterministic retiming solve.
    max_time_scaling_retries: int = 1
    time_stretch_safety_factor: float = 1.05

    solver_time_s: float = 0.75
    solver_iterations: int = 3000

    # Conservative initial default. Can be enabled once the estimate is useful.
    use_target_acceleration: bool = False

    def __post_init__(self) -> None:
        clearance = float(self.planning_clearance_margin_m)

        if not math.isfinite(clearance) or clearance < 0.0:
            raise ValueError(
                "planning_clearance_margin_m must be finite and non-negative"
            )

        for name in (
            "max_speed_mps",
            "max_acceleration_mps2",
            "validation_headroom_ratio",
            "nominal_speed_mps",
            "minimum_segment_time_s",
            "maximum_segment_time_s",
            "time_stretch_safety_factor",
            "solver_time_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.validation_headroom_ratio < 1.0:
            raise ValueError("validation_headroom_ratio must be >= 1")
        if self.maximum_segment_time_s < self.minimum_segment_time_s:
            raise ValueError("maximum_segment_time_s must be >= minimum_segment_time_s")
        if self.time_stretch_safety_factor < 1.0:
            raise ValueError("time_stretch_safety_factor must be >= 1")
        retries = int(self.max_time_scaling_retries)
        if retries not in (0, 1):
            raise ValueError("max_time_scaling_retries must be 0 or 1")
        object.__setattr__(
            self,
            "enabled",
            bool(self.enabled),
        )

        object.__setattr__(
            self,
            "planning_clearance_margin_m",
            clearance,
        )
        object.__setattr__(self, "max_time_scaling_retries", retries)
        object.__setattr__(self, "solver_iterations", max(20, int(self.solver_iterations)))
        object.__setattr__(self, "use_target_acceleration", bool(self.use_target_acceleration))


@dataclass(frozen=True)
class ConvexTrajectoryResult:
    success: bool
    status: str
    message: str
    control_points: np.ndarray = field(default_factory=lambda: np.empty((0, 8, 3)))
    segment_times: np.ndarray = field(default_factory=lambda: np.empty(0))
    positions: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    velocities: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    accelerations: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    jerks: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    solve_time_ms: float = 0.0
    qp_solve_count: int = 0
    solver_iterations: int = 0
    solver_message: str = ""
    peak_speed_mps: float = 0.0
    peak_acceleration_mps2: float = 0.0
    velocity_control_bound_mps: float = 0.0
    acceleration_control_bound_mps2: float = 0.0
    validation_speed_limit_mps: float = 0.0
    validation_acceleration_limit_mps2: float = 0.0
    corridor_violation_m: float = 0.0
    boundary_position_error_m: float = 0.0
    boundary_velocity_error_mps: float = 0.0
    boundary_acceleration_error_mps2: float = 0.0
    join_position_error_m: float = 0.0
    join_velocity_error_mps: float = 0.0
    join_acceleration_error_mps2: float = 0.0
    join_jerk_error_mps3: float = 0.0


def resolve_target_acceleration(
    config: ConvexTrajectoryConfig,
    target_acceleration: Optional[Sequence[float]],
) -> np.ndarray:
    if not config.use_target_acceleration or target_acceleration is None:
        return np.zeros(3, dtype=float)
    return _vec3(target_acceleration, "target_acceleration")


def _bernstein_gram_degree3() -> np.ndarray:
    gram = np.empty((4, 4), dtype=float)
    for i in range(4):
        for j in range(4):
            gram[i, j] = (
                math.comb(3, i)
                * math.comb(3, j)
                * math.factorial(i + j)
                * math.factorial(6 - i - j)
                / math.factorial(7)
            )
    return gram


def _snap_hessian_degree7() -> np.ndarray:
    difference4 = np.diff(np.eye(8), n=4, axis=0)
    derivative = (7.0 * 6.0 * 5.0 * 4.0) * difference4
    return derivative.T @ _bernstein_gram_degree3() @ derivative


_SNAP_HESSIAN = _snap_hessian_degree7()


def _idx(segment: int, control: int, axis: int) -> int:
    return 3 * (8 * segment + control) + axis


def _initial_segment_times(
    corridor: Tuple[ConvexPolyhedron, ...],
    config: ConvexTrajectoryConfig,
) -> np.ndarray:
    times = []
    vmax = config.max_speed_mps
    amax = config.max_acceleration_mps2
    switch_distance = vmax * vmax / amax
    for cell in corridor:
        distance = float(np.linalg.norm(cell.segment_end - cell.segment_start))
        if distance <= 1e-12:
            raise ValueError("corridor contains a zero-length owning segment")
        if distance <= switch_distance:
            dynamic_time = 2.0 * math.sqrt(distance / amax)
        else:
            dynamic_time = 2.0 * vmax / amax + (distance - switch_distance) / vmax
        times.append(
            np.clip(
                max(distance / config.nominal_speed_mps, dynamic_time),
                config.minimum_segment_time_s,
                config.maximum_segment_time_s,
            )
        )
    return np.asarray(times, dtype=float)


def _first_boundary_exit_time(
    quadratic: float,
    linear: float,
    constant: float,
    maximum_time: float,
) -> float:
    """First time q(T) <= 0 leaves the feasible branch containing T=0."""

    scale = max(
        1.0,
        abs(quadratic),
        abs(linear),
        abs(constant),
    )

    eps = (
        1e-11
        * scale
    )

    if constant > eps:
        return 0.0

    roots: list[float] = [
        0.0,
        float(maximum_time),
    ]

    if abs(quadratic) <= eps:

        if abs(linear) > eps:

            root = (
                -constant
                / linear
            )

            if (
                0.0
                <= root
                <= maximum_time
            ):
                roots.append(
                    float(root)
                )

    else:

        discriminant = (
            linear * linear
            - 4.0
            * quadratic
            * constant
        )

        if discriminant >= -eps:

            discriminant = max(
                0.0,
                discriminant,
            )

            sqrt_disc = math.sqrt(
                discriminant
            )

            for root in (
                (
                    -linear
                    - sqrt_disc
                )
                / (
                    2.0
                    * quadratic
                ),
                (
                    -linear
                    + sqrt_disc
                )
                / (
                    2.0
                    * quadratic
                ),
            ):
                if (
                    0.0
                    <= root
                    <= maximum_time
                ):
                    roots.append(
                        float(root)
                    )

    roots = sorted(
        set(
            round(
                root,
                12,
            )
            for root in roots
        )
    )

    # The roots divide the duration axis into regions in which
    # the quadratic has constant sign.
    for left, right in zip(
        roots[:-1],
        roots[1:],
    ):

        if (
            right - left
            <= 1e-10
        ):
            continue

        midpoint = (
            0.5
            * (
                left
                + right
            )
        )

        value = (
            quadratic
            * midpoint
            * midpoint
            + linear
            * midpoint
            + constant
        )

        if value > eps:
            return float(left)

    return float(
        maximum_time
    )


def _boundary_time_upper_bound(
    cell: ConvexPolyhedron,
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    at_start: bool,
    maximum_time: float,
) -> float:
    """Maximum T on the boundary-feasible branch connected to T=0."""

    if at_start:

        # P1(T)
        # P2(T)
        control_polynomials = (
            (
                position,
                velocity / 7.0,
                np.zeros(3),
            ),
            (
                position,
                2.0
                * velocity
                / 7.0,
                acceleration
                / 42.0,
            ),
        )

    else:

        # P6(T)
        # P5(T)
        control_polynomials = (
            (
                position,
                -velocity / 7.0,
                np.zeros(3),
            ),
            (
                position,
                -2.0
                * velocity
                / 7.0,
                acceleration
                / 42.0,
            ),
        )

    upper = float(
        maximum_time
    )

    for (
        constant_vec,
        linear_vec,
        quadratic_vec,
    ) in control_polynomials:

        for normal, offset in zip(
            cell.A,
            cell.b,
        ):

            upper = min(
                upper,
                _first_boundary_exit_time(
                    float(
                        normal
                        @ quadratic_vec
                    ),
                    float(
                        normal
                        @ linear_vec
                    ),
                    float(
                        normal
                        @ constant_vec
                        - offset
                    ),
                    maximum_time,
                ),
            )

    return upper


def _clip_time_to_boundary_upper(
    preferred: float,
    upper: float,
    minimum: float,
) -> Optional[float]:

    if (
        upper
        < minimum - 1e-9
    ):
        return None

    chosen = min(
        float(preferred),
        float(upper),
    )

    if chosen < minimum:
        chosen = float(
            minimum
        )

    # If clipping occurred exactly at a plane crossing, remain
    # microscopically inside if there is room.
    if (
        preferred > upper
        and upper
        > minimum + 1e-6
    ):
        chosen = max(
            float(minimum),
            float(upper) - 1e-6,
        )

    return chosen


def _boundary_aware_segment_times(
    corridor: Tuple[ConvexPolyhedron, ...],
    start_p: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    goal_p: np.ndarray,
    goal_v: np.ndarray,
    goal_a: np.ndarray,
    config: ConvexTrajectoryConfig,
) -> Tuple[
    Optional[np.ndarray],
    str,
]:
    """Clip first/last heuristic durations to exact boundary-control limits."""

    times = _initial_segment_times(
        corridor,
        config,
    )

    minimum = float(
        config.minimum_segment_time_s
    )

    maximum = float(
        config.maximum_segment_time_s
    )

    start_upper = (
        _boundary_time_upper_bound(
            corridor[0],
            start_p,
            start_v,
            start_a,
            True,
            maximum,
        )
    )

    goal_upper = (
        _boundary_time_upper_bound(
            corridor[-1],
            goal_p,
            goal_v,
            goal_a,
            False,
            maximum,
        )
    )

    if len(corridor) == 1:

        upper = min(
            start_upper,
            goal_upper,
        )

        chosen = (
            _clip_time_to_boundary_upper(
                times[0],
                upper,
                minimum,
            )
        )

        if chosen is None:

            return (
                None,
                (
                    f"no positive segment duration >= "
                    f"{minimum:.3f}s keeps both boundary "
                    "Bezier control points inside the corridor"
                ),
            )

        times[0] = chosen

    else:

        chosen_start = (
            _clip_time_to_boundary_upper(
                times[0],
                start_upper,
                minimum,
            )
        )

        chosen_goal = (
            _clip_time_to_boundary_upper(
                times[-1],
                goal_upper,
                minimum,
            )
        )

        if chosen_start is None:
            return (
                None,
                (
                    "no admissible first-segment duration "
                    "for exact start p/v/a"
                ),
            )

        if chosen_goal is None:
            return (
                None,
                (
                    "no admissible final-segment duration "
                    "for exact goal p/v/a"
                ),
            )

        times[0] = chosen_start
        times[-1] = chosen_goal

    return (
        times,
        (
            f"boundary-aware times selected; "
            f"T0={times[0]:.3f}s "
            f"(max {start_upper:.3f}s), "
            f"Tf={times[-1]:.3f}s "
            f"(max {goal_upper:.3f}s)"
        ),
    )


def _project_segment_times_to_boundary_feasible(
    proposed_times: np.ndarray,
    corridor: Tuple[ConvexPolyhedron, ...],
    start_p: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    goal_p: np.ndarray,
    goal_v: np.ndarray,
    goal_a: np.ndarray,
    config: ConvexTrajectoryConfig,
) -> Optional[np.ndarray]:

    times = np.clip(
        np.asarray(
            proposed_times,
            dtype=float,
        ).copy(),
        config.minimum_segment_time_s,
        config.maximum_segment_time_s,
    )

    minimum = float(
        config.minimum_segment_time_s
    )

    maximum = float(
        config.maximum_segment_time_s
    )

    start_upper = (
        _boundary_time_upper_bound(
            corridor[0],
            start_p,
            start_v,
            start_a,
            True,
            maximum,
        )
    )

    goal_upper = (
        _boundary_time_upper_bound(
            corridor[-1],
            goal_p,
            goal_v,
            goal_a,
            False,
            maximum,
        )
    )

    if len(corridor) == 1:

        chosen = (
            _clip_time_to_boundary_upper(
                times[0],
                min(
                    start_upper,
                    goal_upper,
                ),
                minimum,
            )
        )

        if chosen is None:
            return None

        times[0] = chosen

        return times

    chosen_start = (
        _clip_time_to_boundary_upper(
            times[0],
            start_upper,
            minimum,
        )
    )

    chosen_goal = (
        _clip_time_to_boundary_upper(
            times[-1],
            goal_upper,
            minimum,
        )
    )

    if (
        chosen_start is None
        or chosen_goal is None
    ):
        return None

    times[0] = chosen_start
    times[-1] = chosen_goal

    return times

def _boundary_control_diagnostics(
    corridor,
    times,
    start_p,
    start_v,
    start_a,
    goal_p,
    goal_v,
    goal_a,
) -> str:
    """Report fixed boundary Bezier control-point corridor violations."""

    t0 = float(times[0])
    tf = float(times[-1])

    p0 = start_p
    p1 = p0 + start_v * t0 / 7.0
    p2 = (
        2.0 * p1
        - p0
        + start_a * t0**2 / 42.0
    )

    p7 = goal_p
    p6 = p7 - goal_v * tf / 7.0
    p5 = (
        2.0 * p6
        - p7
        + goal_a * tf**2 / 42.0
    )

    checks = (
        ("start_P0", p0, corridor[0]),
        ("start_P1", p1, corridor[0]),
        ("start_P2", p2, corridor[0]),
        ("goal_P5", p5, corridor[-1]),
        ("goal_P6", p6, corridor[-1]),
        ("goal_P7", p7, corridor[-1]),
    )

    parts = []

    for name, point, cell in checks:
        violation = max(
            0.0,
            float(
                np.max(
                    cell.A @ point - cell.b
                )
            ),
        )

        parts.append(
            f"{name}={violation:.4f}m"
        )

    return (
        "boundary CP violations: "
        + ", ".join(parts)
        + f"; |v0|={np.linalg.norm(start_v):.3f}m/s"
        + f", |a0|={np.linalg.norm(start_a):.3f}m/s^2"
        + f", T0={t0:.3f}s"
        + f", Tf={tf:.3f}s"
    )

def _objective(segment_times: np.ndarray) -> np.ndarray:
    segment_times = np.asarray(segment_times, dtype=float).reshape(-1)
    nvar = 24 * len(segment_times)
    hessian = np.zeros((nvar, nvar), dtype=float)
    for segment, duration in enumerate(segment_times):
        block = 2.0 * _SNAP_HESSIAN / max(1e-9, float(duration)) ** 7
        for axis in range(3):
            indices = [_idx(segment, control, axis) for control in range(8)]
            hessian[np.ix_(indices, indices)] += block
    return 0.5 * (hessian + hessian.T)


def _equalities(
    times: np.ndarray,
    start_p: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    goal_p: np.ndarray,
    goal_v: np.ndarray,
    goal_a: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    segment_count = len(times)
    nvar = 24 * segment_count
    rows: list[np.ndarray] = []
    rhs: list[float] = []

    def add(entries, value) -> None:
        row = np.zeros(nvar, dtype=float)
        for index, coefficient in entries:
            row[index] += coefficient
        rows.append(row)
        rhs.append(float(value))

    t0 = max(1e-9, float(times[0]))
    for axis in range(3):
        p0, p1, p2 = (_idx(0, k, axis) for k in (0, 1, 2))
        add(((p0, 1.0),), start_p[axis])
        add(((p1, 1.0), (p0, -1.0)), start_v[axis] * t0 / 7.0)
        add(((p2, 1.0), (p1, -2.0), (p0, 1.0)), start_a[axis] * t0**2 / 42.0)

    for segment in range(segment_count - 1):
        tl, tr = max(1e-9, float(times[segment])), max(1e-9, float(times[segment + 1]))
        for axis in range(3):
            l4, l5, l6, l7 = (_idx(segment, k, axis) for k in (4, 5, 6, 7))
            r0, r1, r2, r3 = (_idx(segment + 1, k, axis) for k in (0, 1, 2, 3))
            add(((l7, 1.0), (r0, -1.0)), 0.0)
            add(((l7, 1/tl), (l6, -1/tl), (r1, -1/tr), (r0, 1/tr)), 0.0)
            add(
                ((l7, 1/tl**2), (l6, -2/tl**2), (l5, 1/tl**2),
                 (r2, -1/tr**2), (r1, 2/tr**2), (r0, -1/tr**2)),
                0.0,
            )
            add(
                ((l7, 1/tl**3), (l6, -3/tl**3), (l5, 3/tl**3), (l4, -1/tl**3),
                 (r3, -1/tr**3), (r2, 3/tr**3), (r1, -3/tr**3), (r0, 1/tr**3)),
                0.0,
            )

    last = segment_count - 1
    tf = max(1e-9, float(times[-1]))
    for axis in range(3):
        p5, p6, p7 = (_idx(last, k, axis) for k in (5, 6, 7))
        add(((p7, 1.0),), goal_p[axis])
        add(((p7, 1.0), (p6, -1.0)), goal_v[axis] * tf / 7.0)
        add(((p7, 1.0), (p6, -2.0), (p5, 1.0)), goal_a[axis] * tf**2 / 42.0)

    return np.vstack(rows), np.asarray(rhs)


def _corridor_inequalities(
    corridor: Tuple[ConvexPolyhedron, ...],
) -> Tuple[np.ndarray, np.ndarray]:
    rows, rhs = [], []
    nvar = 24 * len(corridor)
    for segment, cell in enumerate(corridor):
        for control in range(8):
            for normal, offset in zip(cell.A, cell.b):
                row = np.zeros(nvar, dtype=float)
                for axis in range(3):
                    row[_idx(segment, control, axis)] = normal[axis]
                rows.append(row)
                rhs.append(float(offset))
    return np.vstack(rows), np.asarray(rhs)


def _dynamic_inequalities(
    corridor: Tuple[ConvexPolyhedron, ...],
    times: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    config: ConvexTrajectoryConfig,
) -> Tuple[np.ndarray, np.ndarray]:
    """Gao/Btraj-style Cartesian Bernstein hodograph constraints.

    For fixed segment time T, the derivative control points are affine in the
    position control points:

        V_j = 7 / T * (P_{j+1} - P_j)
        A_j = 42 / T^2 * (P_{j+2} - 2 P_{j+1} + P_j)

    Btraj bounds each x/y/z component independently.  Because a Bezier curve is
    contained in the convex hull of its control points, bounding every derivative
    control point component is a continuous sufficient bound for that derivative
    component over the complete segment.

    Online replanning modification: if a measured start component is already
    outside the normal planning box, that one fixed derivative component is not
    constrained retroactively.  All subsequent derivative control points remain
    constrained, so the new command must recover into the planning envelope.
    """

    rows: list[np.ndarray] = []
    rhs: list[float] = []
    nvar = 24 * len(corridor)

    start_v = np.asarray(start_v, dtype=float).reshape(3)
    start_a = np.asarray(start_a, dtype=float).reshape(3)

    inherited_v_excess = (
        np.abs(start_v) > config.max_speed_mps + 1e-9
    )
    inherited_a_excess = (
        np.abs(start_a) > config.max_acceleration_mps2 + 1e-9
    )

    for segment, duration in enumerate(times):
        t = max(1e-9, float(duration))
        velocity_factor = 7.0 / t
        acceleration_factor = 42.0 / (t * t)

        # Degree-6 velocity hodograph control points.
        for control in range(7):
            for axis in range(3):
                if (
                    segment == 0
                    and control == 0
                    and inherited_v_excess[axis]
                ):
                    continue

                row = np.zeros(nvar, dtype=float)
                row[_idx(segment, control + 1, axis)] = velocity_factor
                row[_idx(segment, control, axis)] = -velocity_factor

                # -v_max <= V_j,axis <= v_max
                rows.append(row)
                rhs.append(float(config.max_speed_mps))
                rows.append(-row)
                rhs.append(float(config.max_speed_mps))

        # Degree-5 acceleration hodograph control points.
        for control in range(6):
            for axis in range(3):
                if (
                    segment == 0
                    and control == 0
                    and inherited_a_excess[axis]
                ):
                    continue

                row = np.zeros(nvar, dtype=float)
                row[_idx(segment, control + 2, axis)] = acceleration_factor
                row[_idx(segment, control + 1, axis)] = -2.0 * acceleration_factor
                row[_idx(segment, control, axis)] = acceleration_factor

                # -a_max <= A_j,axis <= a_max
                rows.append(row)
                rhs.append(float(config.max_acceleration_mps2))
                rows.append(-row)
                rhs.append(float(config.max_acceleration_mps2))

    if not rows:
        return (
            np.empty((0, nvar), dtype=float),
            np.empty(0, dtype=float),
        )

    return np.vstack(rows), np.asarray(rhs, dtype=float)

def _solve_fixed_time_qp(
    corridor: Tuple[ConvexPolyhedron, ...],
    times: np.ndarray,
    start_p: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    goal_p: np.ndarray,
    goal_v: np.ndarray,
    goal_a: np.ndarray,
    config: ConvexTrajectoryConfig,
    deadline: float,
    enforce_dynamic: bool,
) -> Tuple[Optional[np.ndarray], str, str, int]:
    hessian = _objective(times)
    equality, beq = _equalities(
        times,
        start_p,
        start_v,
        start_a,
        goal_p,
        goal_v,
        goal_a,
    )
    corridor_inequality, corridor_bub = _corridor_inequalities(corridor)

    if enforce_dynamic:
        dynamic_inequality, dynamic_bub = _dynamic_inequalities(
            corridor,
            times,
            start_v,
            start_a,
            config,
        )
    else:
        dynamic_inequality = np.empty((0, hessian.shape[0]), dtype=float)
        dynamic_bub = np.empty(0, dtype=float)

    inequality = np.vstack((corridor_inequality, dynamic_inequality))
    bub = np.r_[corridor_bub, dynamic_bub]
    nvar = hessian.shape[0]

    try:
        import casadi as ca
        from scipy import sparse

        if not ca.has_conic("osqp"):
            return (
                None,
                "SOLVER_UNAVAILABLE",
                "CasADi OSQP QP plugin unavailable",
                0,
            )
    except Exception as exc:
        return (
            None,
            "SOLVER_UNAVAILABLE",
            f"CasADi/SciPy unavailable: {exc}",
            0,
        )

    # Normalize equality rows before giving them to the QP.  Position and
    # derivative equalities naturally contain very different coefficient scales.
    norms = np.maximum(np.linalg.norm(equality, axis=1), 1e-12)
    equality_n = equality / norms[:, None]
    beq_n = beq / norms

    # Assembly above is dense for readability, but the actual solver matrices are
    # sparse CSC matrices.  Do not regress this to dense CasADi sparsities.
    equality_sp = sparse.csc_matrix(equality_n)
    inequality_sp = sparse.csc_matrix(inequality)
    constraint_sp = sparse.vstack(
        (equality_sp, inequality_sp),
        format="csc",
    )

    lower = np.r_[
        beq_n,
        np.full(len(bub), -np.inf),
    ]
    upper = np.r_[beq_n, bub]

    scale = max(1.0, float(np.max(np.abs(hessian))))
    pmat_sp = sparse.csc_matrix(
        0.5 * (hessian + hessian.T) / scale
    )

    # Integrated snap has a polynomial nullspace.  CasADi's conic interface
    # expects a positive-definite Hessian, so retain a tiny numerical
    # regularisation.  It is solver conditioning, not a trajectory cost term of
    # practical magnitude.
    pmat_sp = pmat_sp + 1e-10 * sparse.eye(nvar, format="csc")

    pmat_dm = ca.DM(pmat_sp)
    constraint_dm = ca.DM(constraint_sp)

    if time.perf_counter() >= deadline:
        return (
            None,
            "SOLVER_TIME_LIMIT",
            "deadline expired before QP solve",
            0,
        )

    try:
        solver = ca.conic(
            "convex_corridor_bezier_osqp",
            "osqp",
            {
                "h": pmat_dm.sparsity(),
                "a": constraint_dm.sparsity(),
            },
            {
                "error_on_fail": False,
                "print_time": False,
                "verbose": False,
                "warm_start_primal": False,
                "warm_start_dual": False,
                "osqp": {
                    "verbose": False,
                    "max_iter": config.solver_iterations,
                    "eps_abs": 1e-6,
                    "eps_rel": 1e-6,
                    "polish": True,
                },
            },
        )

        if time.perf_counter() >= deadline:
            return (
                None,
                "SOLVER_TIME_LIMIT",
                "deadline expired during QP setup",
                0,
            )

        solution = solver(
            h=pmat_dm,
            g=ca.DM.zeros(nvar, 1),
            a=constraint_dm,
            lba=ca.DM(lower),
            uba=ca.DM(upper),
        )
        stats = solver.stats()

    except Exception as exc:
        return (
            None,
            "SOLVER_FAILURE",
            f"CasADi/OSQP exception: {exc}",
            0,
        )

    iterations = max(0, int(stats.get("iter_count", -1)))
    solver_status = str(stats.get("return_status", "unknown"))

    if not stats.get("success", False):
        lower_status = solver_status.lower()
        unified_status = str(stats.get("unified_return_status", ""))

        if "infeasible" in lower_status:
            status = "SOLVER_INFEASIBLE"
        elif (
            "time" in lower_status
            or unified_status == "SOLVER_RET_LIMITED"
        ):
            status = "SOLVER_TIME_LIMIT"
        elif (
            "maximum iterations" in lower_status
            or "max_iter" in lower_status
            or "iteration" in lower_status
        ):
            status = "SOLVER_NOT_CONVERGED"
        else:
            status = "SOLVER_FAILURE"

        return (
            None,
            status,
            f"OSQP QP failed: {solver_status}",
            iterations,
        )

    # CasADi 3.7.2's bundled OSQP does not expose OSQP's newer time_limit
    # setting.  Keep the whole-smoother wall-clock deadline as a final gate.
    if time.perf_counter() > deadline:
        return (
            None,
            "SOLVER_TIME_LIMIT",
            "OSQP QP returned after the configured whole-smoother deadline",
            iterations,
        )

    x = np.asarray(solution["x"], dtype=float).reshape(-1)
    if not np.all(np.isfinite(x)):
        return (
            None,
            "SOLVER_INVALID_SOLUTION",
            "OSQP returned non-finite variables",
            iterations,
        )

    eq_error = float(np.max(np.abs(equality @ x - beq)))
    corridor_error = max(
        float(np.max(corridor_inequality @ x - corridor_bub)),
        0.0,
    )
    dynamic_error = (
        max(
            float(np.max(dynamic_inequality @ x - dynamic_bub)),
            0.0,
        )
        if dynamic_inequality.size
        else 0.0
    )

    return (
        x.reshape(len(corridor), 8, 3),
        "SUCCESS",
        (
            f"OSQP QP {solver_status}; "
            f"raw eq={eq_error:.2e}, "
            f"corridor={corridor_error:.2e}, "
            f"dynamic={dynamic_error:.2e}"
        ),
        iterations,
    )

def _endpoint(control: np.ndarray, duration: float, start: bool):
    p, t = control, max(1e-9, float(duration))
    if start:
        return (
            p[0],
            7 * (p[1] - p[0]) / t,
            42 * (p[2] - 2*p[1] + p[0]) / t**2,
            210 * (p[3] - 3*p[2] + 3*p[1] - p[0]) / t**3,
        )
    return (
        p[7],
        7 * (p[7] - p[6]) / t,
        42 * (p[7] - 2*p[6] + p[5]) / t**2,
        210 * (p[7] - 3*p[6] + 3*p[5] - p[4]) / t**3,
    )


def _residuals(
    controls: np.ndarray,
    times: np.ndarray,
    corridor: Tuple[ConvexPolyhedron, ...],
    start_p: np.ndarray,
    start_v: np.ndarray,
    start_a: np.ndarray,
    goal_p: np.ndarray,
    goal_v: np.ndarray,
    goal_a: np.ndarray,
) -> dict[str, float]:
    first = _endpoint(controls[0], times[0], True)
    last = _endpoint(controls[-1], times[-1], False)
    join = np.zeros(4)
    for i in range(len(times) - 1):
        left, right = _endpoint(controls[i], times[i], False), _endpoint(controls[i + 1], times[i + 1], True)
        for order in range(4):
            join[order] = max(join[order], float(np.max(np.abs(left[order] - right[order]))))
    corridor_violation = max(
        0.0,
        *(float(np.max(cp @ cell.A.T - cell.b[None, :])) for cp, cell in zip(controls, corridor)),
    )
    return {
        "bp": max(float(np.max(np.abs(first[0] - start_p))), float(np.max(np.abs(last[0] - goal_p)))),
        "bv": max(float(np.max(np.abs(first[1] - start_v))), float(np.max(np.abs(last[1] - goal_v)))),
        "ba": max(float(np.max(np.abs(first[2] - start_a))), float(np.max(np.abs(last[2] - goal_a)))),
        "jp": join[0], "jv": join[1], "ja": join[2], "jj": join[3], "corr": corridor_violation,
    }


def _residual_check(residuals: dict[str, float]) -> Tuple[bool, str]:
    tolerances = {
        "bp": _POS_TOL_M, "bv": _VEL_TOL_MPS, "ba": _ACC_TOL_MPS2,
        "jp": _POS_TOL_M, "jv": _VEL_TOL_MPS, "ja": _ACC_TOL_MPS2,
        "jj": _JERK_JOIN_TOL_MPS3, "corr": _CORRIDOR_TOL_M,
    }
    failed = [name for name, tolerance in tolerances.items() if residuals[name] > tolerance]
    message = (
        f"physical residuals bp={residuals['bp']:.2e}m, bv={residuals['bv']:.2e}m/s, "
        f"ba={residuals['ba']:.2e}m/s^2, jp={residuals['jp']:.2e}m, "
        f"jv={residuals['jv']:.2e}m/s, ja={residuals['ja']:.2e}m/s^2, "
        f"jj={residuals['jj']:.2e}m/s^3, corr={residuals['corr']:.2e}m"
    )
    return not failed, message + ("; exceeded=" + ",".join(failed) if failed else "")


def _dynamic_bounds(
    controls: np.ndarray,
    times: np.ndarray,
    config: Optional[ConvexTrajectoryConfig] = None,
    start_v: Optional[np.ndarray] = None,
    start_a: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Per-axis Bernstein hodograph bounds used by QP and retiming.

    The returned value for each segment is the maximum absolute Cartesian
    derivative-control-point component.  This deliberately matches the
    Gao/Btraj box constraints rather than a Euclidean vector norm.
    """

    velocity = np.zeros(len(times))
    acceleration = np.zeros(len(times))

    inherited_v_excess = np.zeros(3, dtype=bool)
    inherited_a_excess = np.zeros(3, dtype=bool)

    if config is not None and start_v is not None:
        inherited_v_excess = (
            np.abs(np.asarray(start_v, dtype=float).reshape(3))
            > config.max_speed_mps + 1e-9
        )

    if config is not None and start_a is not None:
        inherited_a_excess = (
            np.abs(np.asarray(start_a, dtype=float).reshape(3))
            > config.max_acceleration_mps2 + 1e-9
        )

    for i, (control, duration) in enumerate(zip(controls, times)):
        velocity_cp = 7.0 * np.diff(control, axis=0) / duration
        acceleration_cp = 42.0 * np.diff(control, n=2, axis=0) / duration**2

        velocity_components = np.abs(velocity_cp)
        acceleration_components = np.abs(acceleration_cp)

        if i == 0 and velocity_components.size:
            velocity_components = velocity_components.copy()
            velocity_components[0, inherited_v_excess] = 0.0

        if i == 0 and acceleration_components.size:
            acceleration_components = acceleration_components.copy()
            acceleration_components[0, inherited_a_excess] = 0.0

        velocity[i] = float(np.max(velocity_components))
        acceleration[i] = float(np.max(acceleration_components))

    return velocity, acceleration

def _bernstein(degree: int, tau: np.ndarray) -> np.ndarray:
    tau = np.asarray(tau, dtype=float).reshape(-1)
    return np.column_stack([
        math.comb(degree, k) * tau**k * (1.0 - tau)**(degree-k)
        for k in range(degree + 1)
    ])


def _sample(controls: np.ndarray, times: np.ndarray, dt: float):
    if not math.isfinite(float(dt)) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    outputs = [[], [], [], []]
    for i, (control, duration) in enumerate(zip(controls, times)):
        local = np.linspace(0.0, duration, max(1, int(math.ceil(duration / dt))) + 1)
        if i:
            local = local[1:]
        tau = np.clip(local / duration, 0.0, 1.0)
        derivative_controls = (
            control,
            7 * np.diff(control, axis=0) / duration,
            42 * np.diff(control, n=2, axis=0) / duration**2,
            210 * np.diff(control, n=3, axis=0) / duration**3,
        )
        for order, cp in enumerate(derivative_controls):
            outputs[order].append(_bernstein(7-order, tau) @ cp)
    return tuple(np.vstack(value) for value in outputs)


def _result_from_residuals(result: ConvexTrajectoryResult, residuals: dict[str, float]):
    return ConvexTrajectoryResult(
        **{**result.__dict__,
           "corridor_violation_m": residuals["corr"],
           "boundary_position_error_m": residuals["bp"],
           "boundary_velocity_error_mps": residuals["bv"],
           "boundary_acceleration_error_mps2": residuals["ba"],
           "join_position_error_m": residuals["jp"],
           "join_velocity_error_mps": residuals["jv"],
           "join_acceleration_error_mps2": residuals["ja"],
           "join_jerk_error_mps3": residuals["jj"]}
    )


def solve_convex_corridor_trajectory(
    corridor: Sequence[ConvexPolyhedron],
    start_position: Sequence[float],
    start_velocity: Sequence[float],
    start_acceleration: Sequence[float],
    goal_position: Sequence[float],
    goal_velocity: Sequence[float],
    goal_acceleration: Sequence[float],
    config: ConvexTrajectoryConfig,
    dt: float,
) -> ConvexTrajectoryResult:
    """Generate one C3 degree-7 minimum-snap trajectory inside the supplied SFC."""

    started = time.perf_counter()
    cells = tuple(corridor)
    if not cells:
        return ConvexTrajectoryResult(False, "NO_CORRIDOR", "no convex corridor cells available")
    if not all(isinstance(cell, ConvexPolyhedron) for cell in cells):
        raise TypeError("corridor must contain ConvexPolyhedron objects")

    start_p, start_v, start_a = map(_vec3, (start_position, start_velocity, start_acceleration),
                                     ("start_position", "start_velocity", "start_acceleration"))
    goal_p, goal_v, goal_a = map(_vec3, (goal_position, goal_velocity, goal_acceleration),
                                  ("goal_position", "goal_velocity", "goal_acceleration"))
    if not cells[0].contains(start_p, 1e-8):
        return ConvexTrajectoryResult(False, "BOUNDARY_OUTSIDE_CORRIDOR", "start is outside first corridor cell")
    if not cells[-1].contains(goal_p, 1e-8):
        return ConvexTrajectoryResult(False, "BOUNDARY_OUTSIDE_CORRIDOR", "goal is outside final corridor cell")

    times, time_message = (
        _boundary_aware_segment_times(
            cells,
            start_p,
            start_v,
            start_a,
            goal_p,
            goal_v,
            goal_a,
            config,
        )
    )

    if times is None:

        return ConvexTrajectoryResult(
            False,
            "BOUNDARY_TIME_INFEASIBLE",
            time_message,
            solve_time_ms=(
                1000
                * (
                    time.perf_counter()
                    - started
                )
            ),
        )
    deadline = started + config.solver_time_s
    messages = [
        time_message
    ]

    total_iterations = 0
    controls = None
    velocity_bounds = acceleration_bounds = np.full(len(cells), np.inf)

    for attempt in range(config.max_time_scaling_retries + 1):
        controls, status, message, iterations = (
            _solve_fixed_time_qp(
                cells,
                times,
                start_p,
                start_v,
                start_a,
                goal_p,
                goal_v,
                goal_a,
                config,
                deadline,

                # First solve is the timing probe.
                # Second solve, if required, has hard dynamics.
                enforce_dynamic=(
                    attempt > 0
                ),
            )
        )
        total_iterations += iterations
        messages.append(f"solve{attempt + 1}:{status} ({message})")
        if controls is None:
            if status == "SOLVER_INFEASIBLE":
                message += (
                    "; "
                    + _boundary_control_diagnostics(
                        cells,
                        times,
                        start_p,
                        start_v,
                        start_a,
                        goal_p,
                        goal_v,
                        goal_a,
                    )
                )
            return ConvexTrajectoryResult(
                False, status, message, segment_times=times.copy(),
                solve_time_ms=1000 * (time.perf_counter() - started),
                qp_solve_count=attempt + 1, solver_iterations=total_iterations,
                solver_message="; ".join(messages)[-3000:],
            )

        residuals = _residuals(controls, times, cells, start_p, start_v, start_a, goal_p, goal_v, goal_a)
        residual_ok, residual_message = _residual_check(residuals)
        messages.append(residual_message)
        if not residual_ok:
            result = ConvexTrajectoryResult(
                False,
                "CORRIDOR_VIOLATION" if residuals["corr"] > _CORRIDOR_TOL_M else "NUMERICAL_VALIDATION_FAILURE",
                residual_message, control_points=controls.copy(), segment_times=times.copy(),
                solve_time_ms=1000 * (time.perf_counter() - started),
                qp_solve_count=attempt + 1, solver_iterations=total_iterations,
                solver_message="; ".join(messages)[-3000:],
            )
            return _result_from_residuals(result, residuals)

        velocity_bounds, acceleration_bounds = (
            _dynamic_bounds(
                controls,
                times,
                config,
                start_v,
                start_a,
            )
        )
        required_scale = np.maximum(
            velocity_bounds / config.max_speed_mps,
            np.sqrt(acceleration_bounds / config.max_acceleration_mps2),
        )
        if np.all(required_scale <= 1.0 + 1e-9) or attempt >= config.max_time_scaling_retries:
            break
        proposed_times = np.minimum(
            config.maximum_segment_time_s,
            times
            * np.maximum(
                1.0,
                required_scale,
            )
            * config.time_stretch_safety_factor,
        )

        projected_times = (
            _project_segment_times_to_boundary_feasible(
                proposed_times,
                cells,
                start_p,
                start_v,
                start_a,
                goal_p,
                goal_v,
                goal_a,
                config,
            )
        )

        if projected_times is None:

            return ConvexTrajectoryResult(
                False,
                "BOUNDARY_TIME_INFEASIBLE",
                (
                    "dynamic retiming has no duration "
                    "compatible with the exact boundary "
                    "control points"
                ),
                control_points=controls.copy(),
                segment_times=times.copy(),
                solve_time_ms=(
                    1000
                    * (
                        time.perf_counter()
                        - started
                    )
                ),
                qp_solve_count=(
                    attempt + 1
                ),
                solver_iterations=(
                    total_iterations
                ),
                solver_message=(
                    "; ".join(messages)[-3000:]
                ),
            )

        times = projected_times

    assert controls is not None
    validation_v = config.validation_headroom_ratio * config.max_speed_mps
    validation_a = config.validation_headroom_ratio * config.max_acceleration_mps2
    max_v_bound, max_a_bound = float(np.max(velocity_bounds)), float(np.max(acceleration_bounds))
    if max_v_bound > validation_v + 1e-9 or max_a_bound > validation_a + 1e-9:
        solve_count = len(
            [
                message
                for message in messages
                if message.startswith(
                    "solve"
                )
            ]
        )
        return ConvexTrajectoryResult(
            False, "DYNAMIC_LIMIT_FAILURE",
            f"continuous Cartesian derivative bounds "
            f"v_axis={max_v_bound:.3f}/{validation_v:.3f} m/s, "
            f"a_axis={max_a_bound:.3f}/{validation_a:.3f} m/s^2; "
            f"|v0|={np.linalg.norm(start_v):.3f} m/s, "
            f"|a0|={np.linalg.norm(start_a):.3f} m/s^2, "
            f"worst_acc_segment={int(np.argmax(acceleration_bounds))}, "
            f"T={times[int(np.argmax(acceleration_bounds))]:.3f} s",
            control_points=controls.copy(), segment_times=times.copy(),
            solve_time_ms=1000 * (time.perf_counter() - started),
            # qp_solve_count=config.max_time_scaling_retries + 1,
            qp_solve_count=solve_count,
            solver_iterations=total_iterations, solver_message="; ".join(messages)[-3000:],
            velocity_control_bound_mps=max_v_bound,
            acceleration_control_bound_mps2=max_a_bound,
            validation_speed_limit_mps=validation_v,
            validation_acceleration_limit_mps2=validation_a,
        )

    positions, velocities, accelerations, jerks = _sample(controls, times, dt)
    residuals = _residuals(controls, times, cells, start_p, start_v, start_a, goal_p, goal_v, goal_a)

    solve_count = len(
        [
            message
            for message in messages
            if message.startswith(
                "solve"
            )
        ]
    )

    result = ConvexTrajectoryResult(
        True, "SUCCESS",
        (
            "convex-corridor minimum-snap Bezier trajectory generated; "
            f"QP solves={solve_count}"
        ),
        control_points=controls.copy(), segment_times=times.copy(),
        positions=positions, velocities=velocities, accelerations=accelerations, jerks=jerks,
        solve_time_ms=1000 * (time.perf_counter() - started),
        # qp_solve_count=len([m for m in messages if m.startswith("solve")]),
        qp_solve_count=solve_count,
        solver_iterations=total_iterations, solver_message="; ".join(messages)[-3000:],
        peak_speed_mps=float(np.max(np.linalg.norm(velocities, axis=1))),
        peak_acceleration_mps2=float(np.max(np.linalg.norm(accelerations, axis=1))),
        velocity_control_bound_mps=max_v_bound,
        acceleration_control_bound_mps2=max_a_bound,
        validation_speed_limit_mps=validation_v,
        validation_acceleration_limit_mps2=validation_a,
    )
    return _result_from_residuals(result, residuals)
