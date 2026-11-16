#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/minvo.hpp"
#include "dynamic_planner/separator.hpp"

namespace dynamic_planner {

// Bounded certificate configuration.  This changes only collision-certificate
// partitioning; it never changes trajectory decision variables, limits, or
// planner/execution timing.  A failed leaf is deliberately fail-closed.
struct TemporalRefinementConfig {
    int max_depth = 3;
    std::size_t max_child_tests = 16;
    // Diagnostics only: continue after a rejected branch to enumerate the
    // complete bounded refinement tree. This never changes the certificate
    // verdict and is disabled for normal planner execution.
    bool collect_full_tree_diagnostics = false;
    // Integration opt-in. Direct certificate calls always perform their explicit
    // request; native planner/checker defaults retain the parent-only path.
    bool enabled = false;
};

// Diagnostic only: dense equal-time sampling is never used for certificate
// acceptance and is not a continuous-time safety proof.
struct TemporalClearanceDiagnostic {
    double minimum_sampled_clearance_m = std::numeric_limits<double>::quiet_NaN();
    double minimum_sample_time_s = std::numeric_limits<double>::quiet_NaN();
    std::size_t sampled_points = 0;
};

struct TemporalLeafDiagnostic {
    std::string stage;
    std::string candidate_identity;
    std::string primitive_identity;
    int parent_interval = -1;
    double start_s = 0.0;
    double end_s = 0.0;
    int depth = 0;
    std::string limit_reason;
    TemporalClearanceDiagnostic equal_time_clearance;
};

struct TemporalRefinementStats {
    std::size_t parent_tests = 0;
    std::size_t parent_fast_path_accepts = 0;
    std::size_t child_tests = 0;
    int max_depth_reached = 0;
    std::size_t accepted_leaves = 0;
    std::size_t depth_or_budget_rejections = 0;
    std::size_t separator_lp_calls = 0;
    std::size_t aabb_separation_skips = 0;
    double wall_time_s = 0.0;
    std::vector<TemporalLeafDiagnostic> unresolved_leaves;
};

void accumulateTemporalRefinementStats(TemporalRefinementStats* destination,
                                       const TemporalRefinementStats& source);

// Returns a conservative convex hull for the exact physical interval. It must
// not perform point sampling or merge the requested child interval with a
// neighbouring temporal slice.
using TemporalHullBuilder = std::function<Eigen::MatrixXd(double start_s, double end_s)>;
using TemporalEgoPosition = std::function<Vec3(double time_s)>;
using TemporalClearanceBuilder = std::function<TemporalClearanceDiagnostic(
    const TemporalEgoPosition& ego_position, double start_s, double end_s)>;

struct TemporalSeparationLeaf {
    double start_s = 0.0;
    double end_s = 0.0;
    int depth = 0;
    Eigen::MatrixXd first_vertices;
    Eigen::MatrixXd second_vertices;
    SeparationResult separation;
};

using TemporalLeafDiagnosticBuilder = std::function<TemporalLeafDiagnostic(
    const TemporalSeparationLeaf& leaf)>;

struct AdaptiveTemporalCertificateResult {
    bool certified = false;
    std::string status;
    std::vector<TemporalSeparationLeaf> accepted_leaves;
    std::optional<TemporalSeparationLeaf> rejected_leaf;
    TemporalRefinementStats stats;
};

// Parent-first MADER-style certificate. The parent hulls retain the caller's
// existing representation (MINVO in Octopus); only a nonseparable parent is
// split into common physical-time child intervals. Child builders must provide
// exact restricted polynomial hulls. When materialize_planes is false,
// AABB-disjoint leaves need not invoke GLPK because no QP plane is required.
AdaptiveTemporalCertificateResult certifyAdaptiveTemporalSeparation(
    double parent_start_s,
    double parent_end_s,
    const Eigen::MatrixXd& parent_first_vertices,
    const Eigen::MatrixXd& parent_second_vertices,
    const TemporalHullBuilder& first_child_hull,
    const TemporalHullBuilder& second_child_hull,
    std::vector<double> mandatory_breakpoints_s,
    const TemporalRefinementConfig& config,
    double separator_validation_tolerance,
    bool materialize_planes,
    TemporalLeafDiagnosticBuilder unresolved_leaf_builder = {});

// Exact de Casteljau restriction of a cubic Bezier control hull. The returned
// four vertices enclose precisely the source curve on [child_start, child_end].
FourPoints restrictCubicBezierVertices(const FourPoints& parent_vertices,
                                       double parent_start_s,
                                       double parent_end_s,
                                       double child_start_s,
                                       double child_end_s);

// Linear maps from the four parent B-spline control points for one cubic span
// to exact restricted Bezier/MINVO child control vertices. The latter is the
// same fixed Bernstein-to-MINVO basis transform applied after de Casteljau
// restriction, so child QP constraints remain affine in the original controls.
Matrix4 cubicBSplineIntervalBezierMap(const std::vector<double>& knots,
                                      int num_control_points,
                                      int parent_interval);

Matrix4 restrictCubicBezierControlMap(const Matrix4& parent_bezier_map,
                                      double parent_start_s,
                                      double parent_end_s,
                                      double child_start_s,
                                      double child_end_s);

Matrix4 cubicBSplineIntervalMinvoMap(const std::vector<double>& knots,
                                     int num_control_points,
                                     int parent_interval,
                                     int num_segments,
                                     double child_start_s,
                                     double child_end_s);

}  // namespace dynamic_planner
