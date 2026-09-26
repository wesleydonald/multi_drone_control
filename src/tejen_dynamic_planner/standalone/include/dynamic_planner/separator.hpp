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

class Separator {
public:
    explicit Separator(double validation_tolerance = 1e-7);

    SeparationResult solve(const Eigen::MatrixXd& first_vertices,
                           const Eigen::MatrixXd& second_vertices);

    std::size_t numLPsRun() const noexcept { return num_lps_run_; }

private:
    double validation_tolerance_;
    std::size_t num_lps_run_ = 0;
};

}  // namespace dynamic_planner
