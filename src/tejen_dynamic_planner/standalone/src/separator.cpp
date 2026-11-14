#include "dynamic_planner/separator.hpp"

#include <glpk.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace dynamic_planner {
namespace {

void validatePoints(const Eigen::MatrixXd& points, const char* name) {
    if (points.rows() <= 0 || points.cols() != 3) {
        throw std::invalid_argument(std::string(name) + " must have shape (N,3) with N>=1");
    }
    if (!points.allFinite()) {
        throw std::invalid_argument(std::string(name) + " must contain only finite values");
    }
}

std::string glpkStatusString(int status) {
    switch (status) {
        case GLP_OPT: return "GLP_OPT";
        case GLP_FEAS: return "GLP_FEAS";
        case GLP_INFEAS: return "GLP_INFEAS";
        case GLP_NOFEAS: return "GLP_NOFEAS";
        case GLP_UNBND: return "GLP_UNBND";
        case GLP_UNDEF: return "GLP_UNDEF";
        default: return "GLP_STATUS_" + std::to_string(status);
    }
}

class GlpkProblem {
public:
    GlpkProblem() : problem_(glp_create_prob()) {
        if (problem_ == nullptr) {
            throw std::runtime_error("glp_create_prob returned null");
        }
    }

    ~GlpkProblem() {
        if (problem_ != nullptr) {
            glp_delete_prob(problem_);
        }
    }

    GlpkProblem(const GlpkProblem&) = delete;
    GlpkProblem& operator=(const GlpkProblem&) = delete;

    glp_prob* get() noexcept { return problem_; }

private:
    glp_prob* problem_;
};

}  // namespace

Eigen::Vector3d SeparatingPlane::unitNormal() const {
    const double norm = normal.norm();
    if (!(norm > 1e-12) || !std::isfinite(norm)) {
        throw std::runtime_error("SeparatingPlane normal must be finite and non-zero");
    }
    return normal / norm;
}

double SeparatingPlane::unitOffset() const {
    const double norm = normal.norm();
    if (!(norm > 1e-12) || !std::isfinite(norm)) {
        throw std::runtime_error("SeparatingPlane normal must be finite and non-zero");
    }
    return offset / norm;
}

Eigen::VectorXd SeparatingPlane::signedDistances(const Eigen::MatrixXd& points) const {
    validatePoints(points, "points");
    const double norm = normal.norm();
    if (!(norm > 1e-12) || !std::isfinite(norm)) {
        throw std::runtime_error("SeparatingPlane normal must be finite and non-zero");
    }
    return (points * normal + Eigen::VectorXd::Constant(points.rows(), offset)) / norm;
}

Eigen::Vector3d SeparatingPlane::pointOnPlane() const {
    const double denom = normal.squaredNorm();
    if (!(denom > 1e-24) || !std::isfinite(denom)) {
        throw std::runtime_error("SeparatingPlane normal must be finite and non-zero");
    }
    return -offset * normal / denom;
}

SeparationLP buildSeparationLP(const Eigen::MatrixXd& first_vertices,
                               const Eigen::MatrixXd& second_vertices) {
    validatePoints(first_vertices, "first_vertices");
    validatePoints(second_vertices, "second_vertices");

    const Eigen::Index n_first = first_vertices.rows();
    const Eigen::Index n_second = second_vertices.rows();

    SeparationLP lp;
    lp.a_ub.resize(n_first + n_second, 4);
    lp.b_ub = Eigen::VectorXd::Constant(n_first + n_second, -1.0);

    lp.a_ub.topLeftCorner(n_first, 3) = first_vertices;
    lp.a_ub.topRightCorner(n_first, 1).setOnes();

    lp.a_ub.bottomLeftCorner(n_second, 3) = -second_vertices;
    lp.a_ub.bottomRightCorner(n_second, 1).setConstant(-1.0);

    return lp;
}

SeparationResult certifyProjectedSeparation(
    const Eigen::MatrixXd& first_vertices,
    const Eigen::MatrixXd& second_vertices,
    const Eigen::Vector3d& direction,
    double minimum_gap_m,
    double validation_tolerance) {
    validatePoints(first_vertices, "first_vertices");
    validatePoints(second_vertices, "second_vertices");
    if (!(minimum_gap_m >= 0.0) || !std::isfinite(minimum_gap_m) ||
        !(validation_tolerance > 0.0) || !std::isfinite(validation_tolerance)) {
        throw std::invalid_argument("invalid projection gap or validation tolerance");
    }
    SeparationResult result;
    result.backend = "projection";
    result.status = "DIRECTION_NOT_CERTIFIED";
    const double direction_norm = direction.norm();
    if (!direction.allFinite() || !(direction_norm > 1e-12) ||
        !std::isfinite(direction_norm)) return result;

    const Eigen::Vector3d unit = direction / direction_norm;
    const double first_support = (first_vertices * unit).maxCoeff();
    const double second_support = (second_vertices * unit).minCoeff();
    const double gap = second_support - first_support;
    if (!(gap > 0.0) || gap < minimum_gap_m || !std::isfinite(gap)) return result;

    SeparatingPlane plane;
    plane.normal = (2.0 / gap) * unit;
    plane.offset = -(first_support + 0.5 * gap) * (2.0 / gap);
    const double norm = plane.normal.norm();
    if (!plane.normal.allFinite() || !std::isfinite(plane.offset) ||
        !(norm > 1e-12) || !std::isfinite(norm)) return result;

    // Do not trust support arithmetic alone: the actual returned coefficients
    // must satisfy the same independent row residual check as the LP output.
    const double first_max = (first_vertices * plane.normal).maxCoeff() + plane.offset;
    const double second_min = (second_vertices * plane.normal).minCoeff() + plane.offset;
    const double actual_gap = (second_min - first_max) / norm;
    if (!std::isfinite(first_max) || !std::isfinite(second_min) ||
        first_max > -1.0 + validation_tolerance || second_min < 1.0 - validation_tolerance ||
        !(actual_gap > 0.0) || actual_gap < minimum_gap_m) return result;

    result.feasible = true;
    result.plane = plane;
    result.first_max_value = first_max;
    result.second_min_value = second_min;
    result.geometric_gap_m = actual_gap;
    result.status = "VALIDATED_PROJECTION";
    return result;
}

Separator::Separator(double validation_tolerance)
    : validation_tolerance_(validation_tolerance) {
    if (!(validation_tolerance_ > 0.0) || !std::isfinite(validation_tolerance_)) {
        throw std::invalid_argument("validation_tolerance must be finite and positive");
    }
}

SeparationResult Separator::solveWithFastPaths(
    const Eigen::MatrixXd& first_vertices,
    const Eigen::MatrixXd& second_vertices,
    const std::optional<Eigen::Vector3d>& hint_direction) {
    validatePoints(first_vertices, "first_vertices");
    validatePoints(second_vertices, "second_vertices");
    const auto certify = [&](const Eigen::Vector3d& direction) {
        return certifyProjectedSeparation(first_vertices, second_vertices, direction,
                                          0.0, validation_tolerance_);
    };
    if (hint_direction) {
        auto result = certify(*hint_direction);
        if (result.feasible) return result;
    }
    const Eigen::Vector3d first_lo = first_vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d first_hi = first_vertices.colwise().maxCoeff().transpose();
    const Eigen::Vector3d second_lo = second_vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d second_hi = second_vertices.colwise().maxCoeff().transpose();
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d direction = Eigen::Vector3d::Zero();
        if (first_hi(axis) < second_lo(axis)) direction(axis) = 1.0;
        else if (second_hi(axis) < first_lo(axis)) direction(axis) = -1.0;
        else continue;
        auto result = certify(direction);
        if (result.feasible) return result;
    }
    auto result = certify((second_vertices.colwise().mean() -
                           first_vertices.colwise().mean()).transpose());
    if (result.feasible) return result;
    return solve(first_vertices, second_vertices);
}

SeparationResult Separator::solveWithMinimumGap(
    const Eigen::MatrixXd& first_vertices,
    const Eigen::MatrixXd& second_vertices,
    double minimum_gap_m,
    const std::optional<Eigen::Vector3d>& hint_direction) {
    if (!std::isfinite(minimum_gap_m) || minimum_gap_m < 0.)
        throw std::invalid_argument("minimum separator gap must be finite and nonnegative");
    const auto before = num_lps_run_;
    const auto initial = solveWithFastPaths(first_vertices, second_vertices, hint_direction);
    if (minimum_gap_m == 0. || !initial.feasible) return initial;
    const auto checked = [&](const Eigen::Vector3d& direction) {
        return certifyProjectedSeparation(first_vertices, second_vertices, direction,
                                          minimum_gap_m, validation_tolerance_);
    };
    if (initial.plane) {
        auto result = checked(initial.plane->normal);
        if (result.feasible) return result;
    }
    // A valid axis separator may be arbitrarily tight at a rotated corner.
    // Alternate directions are hints only: current supports certify each one.
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) {
                if (x == 0 && y == 0 && z == 0) continue;
                auto result = checked(Eigen::Vector3d(x, y, z));
                if (result.feasible) return result;
            }
        }
    }
    if (num_lps_run_ == before) {
        const auto fallback = solve(first_vertices, second_vertices);
        if (fallback.feasible && fallback.plane) {
            auto result = checked(fallback.plane->normal);
            if (result.feasible) return result;
        }
    }
    SeparationResult rejected;
    rejected.status = "NO_MINIMUM_GAP_CERTIFICATE";
    return rejected;
}

SeparationResult Separator::solve(const Eigen::MatrixXd& first_vertices,
                                  const Eigen::MatrixXd& second_vertices) {
    validatePoints(first_vertices, "first_vertices");
    validatePoints(second_vertices, "second_vertices");
    ++num_lps_run_;

    const SeparationLP lp = buildSeparationLP(first_vertices, second_vertices);
    const int rows = static_cast<int>(lp.a_ub.rows());
    constexpr int cols = 4;

    GlpkProblem holder;
    glp_prob* problem = holder.get();
    glp_set_obj_dir(problem, GLP_MIN);
    glp_add_rows(problem, rows);
    glp_add_cols(problem, cols);

    for (int i = 1; i <= rows; ++i) {
        glp_set_row_bnds(problem, i, GLP_UP, 0.0, lp.b_ub(i - 1));
    }
    for (int j = 1; j <= cols; ++j) {
        glp_set_col_bnds(problem, j, GLP_FR, 0.0, 0.0);
        glp_set_obj_coef(problem, j, 0.0);
    }

    // GLPK uses 1-based sparse arrays. Skip exact zeros rather than loading them.
    std::vector<int> ia(1, 0);
    std::vector<int> ja(1, 0);
    std::vector<double> ar(1, 0.0);
    ia.reserve(static_cast<std::size_t>(rows * cols + 1));
    ja.reserve(static_cast<std::size_t>(rows * cols + 1));
    ar.reserve(static_cast<std::size_t>(rows * cols + 1));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            const double value = lp.a_ub(i, j);
            if (value == 0.0) {
                continue;
            }
            ia.push_back(i + 1);
            ja.push_back(j + 1);
            ar.push_back(value);
        }
    }

    const int ne = static_cast<int>(ar.size()) - 1;
    glp_load_matrix(problem, ne, ia.data(), ja.data(), ar.data());

    glp_smcp params;
    glp_init_smcp(&params);
    params.msg_lev = GLP_MSG_OFF;
    // Match the MIT ACL separator used by MADER/RMADER and the validated
    // Python R3/R6 reference: use the GLPK simplex defaults after
    // glp_init_smcp(), i.e. do not enable presolve here.  With this tiny
    // zero-objective feasibility LP, explicitly enabling presolve can report
    // GLP_OPT while glp_get_col_prim() does not satisfy the original canonical
    // rows for some Octopus hull configurations.  The residual validation below
    // is deliberately retained as an independent safety check.
    params.presolve = GLP_OFF;

    const int return_code = glp_simplex(problem, &params);

    SeparationResult result;
    result.backend = "glpk";

    // With GLP_ON presolve, GLPK can report a certified primal-infeasible
    // feasibility LP directly from glp_simplex() as GLP_ENOPFS (10), rather
    // than returning 0 and exposing GLP_NOFEAS through glp_get_status().
    // This is an expected collision/non-separability result, not a solver error.
    if (return_code == GLP_ENOPFS) {
        result.status = "GLP_ENOPFS";
        result.feasible = false;
        return result;
    }

    if (return_code != 0) {
        std::ostringstream oss;
        oss << "glp_simplex failed with return code " << return_code;
        throw std::runtime_error(oss.str());
    }

    const int status = glp_get_status(problem);
    result.status = glpkStatusString(status);

    if (status == GLP_INFEAS || status == GLP_NOFEAS) {
        result.feasible = false;
        return result;
    }
    if (status == GLP_UNBND) {
        throw std::runtime_error("GLPK returned unbounded status for separator feasibility LP");
    }
    if (status != GLP_OPT && status != GLP_FEAS) {
        throw std::runtime_error("GLPK returned unexpected separator status: " + result.status);
    }

    Eigen::Vector4d solution;
    for (int j = 1; j <= cols; ++j) {
        solution(j - 1) = glp_get_col_prim(problem, j);
    }
    if (!solution.allFinite()) {
        throw std::runtime_error("GLPK separator solution contains NaN or Inf");
    }

    SeparatingPlane plane;
    plane.normal = solution.head<3>();
    plane.offset = solution(3);
    const double normal_norm = plane.normal.norm();
    if (!(normal_norm > 1e-12) || !std::isfinite(normal_norm)) {
        throw std::runtime_error("GLPK returned a zero/invalid separator normal");
    }

    const Eigen::VectorXd first_values =
        first_vertices * plane.normal + Eigen::VectorXd::Constant(first_vertices.rows(), plane.offset);
    const Eigen::VectorXd second_values =
        second_vertices * plane.normal + Eigen::VectorXd::Constant(second_vertices.rows(), plane.offset);

    const double first_max = first_values.maxCoeff();
    const double second_min = second_values.minCoeff();

    if (first_max > -1.0 + validation_tolerance_ ||
        second_min < 1.0 - validation_tolerance_) {
        std::ostringstream oss;
        oss << "GLPK returned a separator violating canonical constraints: max(first)="
            << first_max << ", min(second)=" << second_min;
        throw std::runtime_error(oss.str());
    }

    result.feasible = true;
    result.plane = plane;
    result.first_max_value = first_max;
    result.second_min_value = second_min;
    result.geometric_gap_m = (second_min - first_max) / normal_norm;
    return result;
}

}  // namespace dynamic_planner
