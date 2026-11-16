"""Permanent convex-set separator primitive for dynamic planning.

The formulation follows the MADER separator convention.  Given two finite sets
of 3-D points A and B, solve for a plane ``n.T x + d = 0`` satisfying

    n.T a_i + d <= -1   for all a_i in A
    n.T b_j + d >= +1   for all b_j in B

Feasibility means the two convex hulls are strictly linearly separable.
Infeasibility means their convex hulls overlap or touch.

GLPK is the preferred backend, matching the MIT ACL separator library.  A
SciPy/HiGHS backend is intentionally retained as a portable fallback.  Both
backends consume exactly the same LP matrices; there is only one mathematical
formulation in this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import ctypes.util
from typing import Literal

import numpy as np


BackendName = Literal["auto", "glpk", "scipy"]


@dataclass(frozen=True)
class SeparatingPlane:
    """Plane ``normal.T x + offset = 0`` in the raw LP scaling."""

    normal: np.ndarray
    offset: float

    def __post_init__(self) -> None:
        normal = np.asarray(self.normal, dtype=float).reshape(3)
        if not np.all(np.isfinite(normal)) or not np.isfinite(self.offset):
            raise ValueError("Plane coefficients must be finite")
        if np.linalg.norm(normal) <= 1e-12:
            raise ValueError("Plane normal must be non-zero")
        object.__setattr__(self, "normal", normal.copy())
        object.__setattr__(self, "offset", float(self.offset))

    @property
    def unit_normal(self) -> np.ndarray:
        return self.normal / np.linalg.norm(self.normal)

    @property
    def unit_offset(self) -> float:
        return self.offset / float(np.linalg.norm(self.normal))

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        points = _validate_points(points, "points")
        return (points @ self.normal + self.offset) / np.linalg.norm(self.normal)

    def point_on_plane(self) -> np.ndarray:
        denom = float(np.dot(self.normal, self.normal))
        return -self.offset * self.normal / denom


@dataclass(frozen=True)
class SeparationResult:
    feasible: bool
    plane: SeparatingPlane | None
    backend: str
    status: str
    first_max_value: float | None = None
    second_min_value: float | None = None
    geometric_gap_m: float | None = None


@dataclass(frozen=True)
class SeparationLP:
    """Canonical LP representation shared by every backend."""

    a_ub: np.ndarray
    b_ub: np.ndarray


def _validate_points(points: np.ndarray, name: str) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        raise ValueError(f"{name} must have shape (N, 3) with N >= 1")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} must contain only finite values")
    return points


def build_separation_lp(first_vertices: np.ndarray, second_vertices: np.ndarray) -> SeparationLP:
    """Build ``A_ub x <= b_ub`` for x = [nx, ny, nz, d]."""

    first = _validate_points(first_vertices, "first_vertices")
    second = _validate_points(second_vertices, "second_vertices")

    first_rows = np.column_stack((first, np.ones(first.shape[0])))
    # n.T b + d >= +1  <=>  -(n.T b + d) <= -1
    second_rows = -np.column_stack((second, np.ones(second.shape[0])))
    a_ub = np.vstack((first_rows, second_rows)).astype(float, copy=False)
    b_ub = -np.ones(a_ub.shape[0], dtype=float)
    return SeparationLP(a_ub=a_ub, b_ub=b_ub)


def _result_from_solution(
    solution: np.ndarray,
    first_vertices: np.ndarray,
    second_vertices: np.ndarray,
    *,
    backend: str,
    status: str,
    tolerance: float,
) -> SeparationResult:
    solution = np.asarray(solution, dtype=float).reshape(4)
    plane = SeparatingPlane(solution[:3], solution[3])
    first = _validate_points(first_vertices, "first_vertices")
    second = _validate_points(second_vertices, "second_vertices")

    first_values = first @ plane.normal + plane.offset
    second_values = second @ plane.normal + plane.offset
    first_max = float(np.max(first_values))
    second_min = float(np.min(second_values))
    normal_norm = float(np.linalg.norm(plane.normal))
    geometric_gap = (second_min - first_max) / normal_norm

    if first_max > -1.0 + tolerance or second_min < 1.0 - tolerance:
        raise RuntimeError(
            "Separator backend returned a solution that violates the canonical "
            f"constraints: max(first)={first_max:.6g}, min(second)={second_min:.6g}"
        )

    return SeparationResult(
        feasible=True,
        plane=plane,
        backend=backend,
        status=status,
        first_max_value=first_max,
        second_min_value=second_min,
        geometric_gap_m=float(geometric_gap),
    )


class _GlpkApi:
    """Minimal ctypes wrapper around the GLPK C API used by this LP."""

    GLP_MIN = 1
    GLP_FR = 1
    GLP_LO = 2
    GLP_UP = 3
    GLP_FEAS = 2
    GLP_INFEAS = 3
    GLP_NOFEAS = 4
    GLP_OPT = 5
    GLP_UNBND = 6
    GLP_OFF = 0

    def __init__(self, library_path: str | None = None) -> None:
        path = library_path or ctypes.util.find_library("glpk")
        if not path:
            raise RuntimeError(
                "GLPK shared library not found. On Ubuntu install libglpk40, "
                "or use backend='scipy' / backend='auto'."
            )
        self.lib = ctypes.CDLL(path)
        self._declare_signatures()

    def _declare_signatures(self) -> None:
        lib = self.lib
        lib.glp_create_prob.argtypes = []
        lib.glp_create_prob.restype = ctypes.c_void_p
        lib.glp_delete_prob.argtypes = [ctypes.c_void_p]
        lib.glp_delete_prob.restype = None
        lib.glp_set_obj_dir.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.glp_set_obj_dir.restype = None
        lib.glp_add_rows.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.glp_add_rows.restype = ctypes.c_int
        lib.glp_add_cols.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.glp_add_cols.restype = ctypes.c_int
        lib.glp_set_row_bnds.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
        ]
        lib.glp_set_row_bnds.restype = None
        lib.glp_set_col_bnds.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
        ]
        lib.glp_set_col_bnds.restype = None
        lib.glp_set_obj_coef.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double]
        lib.glp_set_obj_coef.restype = None
        lib.glp_load_matrix.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_double),
        ]
        lib.glp_load_matrix.restype = None
        lib.glp_simplex.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.glp_simplex.restype = ctypes.c_int
        lib.glp_get_status.argtypes = [ctypes.c_void_p]
        lib.glp_get_status.restype = ctypes.c_int
        lib.glp_get_col_prim.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.glp_get_col_prim.restype = ctypes.c_double
        if hasattr(lib, "glp_term_out"):
            lib.glp_term_out.argtypes = [ctypes.c_int]
            lib.glp_term_out.restype = ctypes.c_int

    def solve(self, lp: SeparationLP) -> tuple[bool, np.ndarray | None, str]:
        problem = self.lib.glp_create_prob()
        if not problem:
            raise RuntimeError("glp_create_prob returned null")
        try:
            if hasattr(self.lib, "glp_term_out"):
                self.lib.glp_term_out(self.GLP_OFF)
            self.lib.glp_set_obj_dir(problem, self.GLP_MIN)
            rows = int(lp.a_ub.shape[0])
            cols = 4
            self.lib.glp_add_rows(problem, rows)
            self.lib.glp_add_cols(problem, cols)

            for row in range(1, rows + 1):
                # All canonical constraints are upper bounds A_i x <= -1.
                self.lib.glp_set_row_bnds(problem, row, self.GLP_UP, 0.0, float(lp.b_ub[row - 1]))
            for col in range(1, cols + 1):
                self.lib.glp_set_col_bnds(problem, col, self.GLP_FR, 0.0, 0.0)
                self.lib.glp_set_obj_coef(problem, col, 0.0)

            ne = rows * cols
            ia = (ctypes.c_int * (ne + 1))()
            ja = (ctypes.c_int * (ne + 1))()
            ar = (ctypes.c_double * (ne + 1))()
            k = 1
            for i in range(rows):
                for j in range(cols):
                    ia[k] = i + 1
                    ja[k] = j + 1
                    ar[k] = float(lp.a_ub[i, j])
                    k += 1
            self.lib.glp_load_matrix(problem, ne, ia, ja, ar)

            return_code = int(self.lib.glp_simplex(problem, None))
            if return_code != 0:
                return False, None, f"glp_simplex return code {return_code}"

            status = int(self.lib.glp_get_status(problem))
            if status in (self.GLP_OPT, self.GLP_FEAS):
                solution = np.array(
                    [self.lib.glp_get_col_prim(problem, j) for j in range(1, cols + 1)],
                    dtype=float,
                )
                return True, solution, f"GLPK status {status}"
            if status in (self.GLP_INFEAS, self.GLP_NOFEAS):
                return False, None, f"GLPK infeasible status {status}"
            if status == self.GLP_UNBND:
                return False, None, "GLPK returned unbounded status for feasibility LP"
            return False, None, f"GLPK status {status}"
        finally:
            self.lib.glp_delete_prob(problem)


def glpk_available() -> bool:
    return ctypes.util.find_library("glpk") is not None


def _solve_with_scipy(lp: SeparationLP) -> tuple[bool, np.ndarray | None, str]:
    try:
        from scipy.optimize import linprog
    except ImportError as exc:  # pragma: no cover - package already depends on SciPy elsewhere
        raise RuntimeError("SciPy is required for the HiGHS separator fallback") from exc

    result = linprog(
        c=np.zeros(4, dtype=float),
        A_ub=lp.a_ub,
        b_ub=lp.b_ub,
        bounds=[(None, None)] * 4,
        method="highs",
    )
    if result.success:
        return True, np.asarray(result.x, dtype=float), f"HiGHS status {result.status}: {result.message}"
    # HiGHS status 2 is infeasible.  Other statuses are surfaced rather than
    # silently being reclassified as collision.
    if int(result.status) == 2:
        return False, None, f"HiGHS infeasible: {result.message}"
    raise RuntimeError(f"HiGHS separator solve failed: status={result.status}, {result.message}")


def solve_separator(
    first_vertices: np.ndarray,
    second_vertices: np.ndarray,
    *,
    backend: BackendName = "auto",
    validation_tolerance: float = 1e-7,
) -> SeparationResult:
    """Return a separating plane, or an infeasible result if none exists.

    ``backend='auto'`` prefers native GLPK and falls back to SciPy/HiGHS when
    GLPK is unavailable.  The fallback is intentional and permanent: both
    backends solve the exact same canonical LP built by :func:`build_separation_lp`.
    """

    first = _validate_points(first_vertices, "first_vertices")
    second = _validate_points(second_vertices, "second_vertices")
    if validation_tolerance <= 0.0 or not np.isfinite(validation_tolerance):
        raise ValueError("validation_tolerance must be finite and positive")
    if backend not in ("auto", "glpk", "scipy"):
        raise ValueError("backend must be one of: auto, glpk, scipy")

    lp = build_separation_lp(first, second)
    selected = backend
    if selected == "auto":
        selected = "glpk" if glpk_available() else "scipy"

    if selected == "glpk":
        feasible, solution, status = _GlpkApi().solve(lp)
        backend_name = "glpk"
    else:
        feasible, solution, status = _solve_with_scipy(lp)
        backend_name = "scipy-highs"

    if not feasible:
        return SeparationResult(
            feasible=False,
            plane=None,
            backend=backend_name,
            status=status,
        )
    if solution is None:
        raise RuntimeError("Separator backend reported feasible without a solution")
    return _result_from_solution(
        solution,
        first,
        second,
        backend=backend_name,
        status=status,
        tolerance=validation_tolerance,
    )
