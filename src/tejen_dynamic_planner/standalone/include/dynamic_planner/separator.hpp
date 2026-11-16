#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <string>

namespace dynamic_planner {

struct SeparatingPlane {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double offset = 0.0;

    Eigen::Vector3d unitNormal() const;
    double unitOffset() const;
    Eigen::VectorXd signedDistances(const Eigen::MatrixXd& points) const;
    Eigen::Vector3d pointOnPlane() const;
};

struct SeparationResult {
    bool feasible = false;
    std::optional<SeparatingPlane> plane;
    std::string backend = "glpk";
    std::string status;
    std::optional<double> first_max_value;
    std::optional<double> second_min_value;
    std::optional<double> geometric_gap_m;
};

struct SeparationLP {
    Eigen::MatrixXd a_ub;
    Eigen::VectorXd b_ub;
};

// Build the canonical MADER-style feasibility LP for x=[nx,ny,nz,d]:
//   n^T a_i + d <= -1
//   n^T b_j + d >= +1  <=> -(n^T b_j + d) <= -1.
SeparationLP buildSeparationLP(const Eigen::MatrixXd& first_vertices,
                               const Eigen::MatrixXd& second_vertices);

// Try a direction, not a cached collision verdict. Recompute both current-hull
// supports, enforce the physical gap, then independently validate a canonical
// <=-1/>=+1 plane. Failure means only that this direction did not certify.
SeparationResult certifyProjectedSeparation(
    const Eigen::MatrixXd& first_vertices,
    const Eigen::MatrixXd& second_vertices,
    const Eigen::Vector3d& direction,
    double minimum_gap_m = 0.0,
    double validation_tolerance = 1e-7);

class Separator {
public:
    explicit Separator(double validation_tolerance = 1e-7);

    SeparationResult solve(const Eigen::MatrixXd& first_vertices,
                           const Eigen::MatrixXd& second_vertices);

    // Materialize a checked plane using a hint, disjoint AABB axis or centroid
    // direction; fall back to the unchanged LP if none certifies separation.
    SeparationResult solveWithFastPaths(
        const Eigen::MatrixXd& first_vertices,
        const Eigen::MatrixXd& second_vertices,
        const std::optional<Eigen::Vector3d>& hint_direction = std::nullopt);

    // Repair-only path: verify the required gap from current hull supports.
    // Bounded alternate directions, at most one LP; failure is not proof that
    // no other separating direction exists. Zero preserves the legacy path.
    SeparationResult solveWithMinimumGap(
        const Eigen::MatrixXd& first_vertices,
        const Eigen::MatrixXd& second_vertices,
        double minimum_gap_m,
        const std::optional<Eigen::Vector3d>& hint_direction = std::nullopt);

    std::size_t numLPsRun() const noexcept { return num_lps_run_; }

private:
    double validation_tolerance_;
    std::size_t num_lps_run_ = 0;
};

}  // namespace dynamic_planner
