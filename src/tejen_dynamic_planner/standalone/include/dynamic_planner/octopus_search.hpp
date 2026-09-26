#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/minvo.hpp"
#include "dynamic_planner/separator.hpp"
#include "dynamic_planner/types.hpp"

namespace dynamic_planner {

using OctopusStopPredicate = std::function<bool()>;

struct TimeIndexedObstacle {
    std::string name;
    std::vector<Eigen::MatrixXd> interval_vertices;
};

struct OctopusConfig {
    Vec3 v_max = Vec3::Ones();
    Vec3 a_max = Vec3::Constant(1.5);
    std::array<int, 3> samples_per_axis{{7, 7, 7}};
    double alpha_shrink = 0.9;
    double voxel_fraction = 0.10;
    double heuristic_bias = 1.0;
    double goal_tolerance_m = 0.05;
    double max_runtime_s = 8.0;
    Vec3 xyz_min = Vec3(-0.75, -0.75, 0.40);
    Vec3 xyz_max = Vec3(3.00, 3.00, 2.60);
    double planning_radius_m = 30.0;
    std::uint32_t random_seed = 1;
};

struct IntervalSeparator {
    std::string obstacle_name;
    int interval_index = -1;
    SeparationResult result;
};

struct OctopusResult {
    bool success = false;
    bool reached_goal = false;
    std::string status;
    std::optional<ControlPoints> control_points;
    // Diagnostics-only copy of the completed candidate selected for the final
    // exact separator check. It is populated even when that candidate is
    // rejected, so ROS/RViz can show the path that actually failed without
    // weakening or bypassing certification.
    std::optional<ControlPoints> diagnostic_control_points;
    std::vector<IntervalSeparator> separators;
    std::size_t expanded_nodes = 0;
    std::size_t popped_nodes = 0;
    std::size_t separator_lp_calls = 0;
    std::size_t aabb_separation_skips = 0;
    double search_time_s = 0.0;
    double voxel_size_m = 0.0;
    std::optional<double> goal_distance_m;

    // Why the queue exploration ended. This is deliberately separate from
    // status, which records which candidate/fallback was finally selected.
    std::string termination_reason;
    bool complete_available_at_termination = false;

    // Diagnostics only. These expose when the search first found any complete
    // final-index candidate, before later queue exploration improved it or reached
    // the goal. They do not alter selection, pruning, timing, or safety behavior.
    std::optional<double> first_complete_time_s;
    std::optional<double> first_complete_goal_distance_m;
    std::size_t first_complete_expanded_nodes = 0;
    std::size_t first_complete_popped_nodes = 0;
    std::size_t closest_complete_improvements = 0;
    // C1F.5: complete terminal nodes that would be swept by an obstacle during
    // the post-arrival hold guard are rejected as infeasible, not merely ranked lower.
    std::size_t terminal_hold_rejections = 0;
    std::size_t terminal_hold_separator_lp_calls = 0;

    // M2D hard-geometry fallback diagnostics. If no complete candidate exists,
    // Octopus retains up to eight closest expanded partial histories and tries
    // their existing stopped completion/certification in goal-distance order.
    // selected_rank is 1-based; zero means no partial fallback certified.
    std::size_t partial_fallback_candidates_retained = 0;
    std::size_t partial_fallback_candidates_tested = 0;
    std::size_t partial_fallback_selected_rank = 0;

    // C1F.5p2 restart-contract diagnostics. These record the authoritative
    // B-spline prefix presented to Octopus and the transaction search domain.
    // The initial q2 gate accepts only tiny numerical boundary residue; it does
    // not enlarge the normal search workspace for subsequently generated nodes.
    Vec3 initial_q0 = Vec3::Zero();
    Vec3 initial_q1 = Vec3::Zero();
    Vec3 initial_q2 = Vec3::Zero();
    Vec3 search_xyz_min = Vec3::Zero();
    Vec3 search_xyz_max = Vec3::Zero();
    double initial_q2_radius_m = 0.0;
    bool initial_q2_box_valid = false;
    bool initial_q2_radius_valid = false;
};

class OctopusSearch {
public:
    OctopusSearch(std::vector<double> knots,
                  ControlPoints initial_control_points,
                  Vec3 goal,
                  std::vector<TimeIndexedObstacle> obstacles,
                  OctopusConfig config,
                  OctopusStopPredicate stop_requested = {},
                  std::string stop_reason = "EXTERNAL_STOP_REQUESTED");

    OctopusSearch(std::vector<double> knots,
                  ControlPoints initial_control_points,
                  Vec3 goal,
                  std::vector<TimeIndexedObstacle> obstacles,
                  std::vector<TimeIndexedObstacle> terminal_hold_obstacles,
                  OctopusConfig config,
                  OctopusStopPredicate stop_requested = {},
                  std::string stop_reason = "EXTERNAL_STOP_REQUESTED");

    OctopusSearch(std::vector<double> knots,
                  ControlPoints initial_control_points,
                  Vec3 goal,
                  std::vector<TimeIndexedObstacle> obstacles,
                  std::vector<TimeIndexedObstacle> terminal_hold_obstacles,
                  TerminalBoundary terminal_boundary,
                  OctopusConfig config,
                  OctopusStopPredicate stop_requested = {},
                  std::string stop_reason = "EXTERNAL_STOP_REQUESTED");

    OctopusResult search();

    int numSegments() const noexcept { return num_segments_; }
    int numControlPoints() const noexcept { return num_control_points_; }
    int finalSearchIndex() const noexcept { return final_search_index_; }
    double voxelSize() const noexcept { return voxel_size_; }

    std::optional<std::pair<Vec3, Vec3>> velocityBounds(
        int index,
        const Vec3& qi_m2,
        const Vec3& qi_m1,
        const Vec3& qi) const;

private:
    struct Node;
    struct VoxelKey;
    struct VoxelKeyHash;
    struct IntervalAabb {
        Vec3 lower = Vec3::Zero();
        Vec3 upper = Vec3::Zero();
    };

    Vec3 velocityBetween(const Vec3& q_prev,
                         const Vec3& q_curr,
                         int control_index_prev) const;
    Vec3 nextQ(const Vec3& qi, int index, const Vec3& velocity) const;
    ControlPoints completedControlPoints(const Node* node) const;
    double computeVoxelSize() const;
    VoxelKey voxelKey(const Vec3& q) const;
    std::vector<Vec3> pathToNode(const Node* node) const;
    bool controlHullSeparable(const FourPoints& control_points, int interval);
    bool intervalSeparable(const Node* node);
    std::pair<bool, std::vector<IntervalSeparator>> finalCheck(
        const ControlPoints& control_points);
    bool finalDynamicsFeasible(const ControlPoints& control_points) const;
    bool terminalHoldViable(const Vec3& terminal_position);
    bool withinSearchLimits(const Vec3& q) const;
    void validateInputs() const;

    std::vector<double> knots_;
    ControlPoints q012_;
    Vec3 goal_;
    std::vector<TimeIndexedObstacle> obstacles_;
    std::vector<TimeIndexedObstacle> terminal_hold_obstacles_;
    TerminalBoundary terminal_boundary_;
    std::vector<std::vector<IntervalAabb>> obstacle_aabbs_;
    std::vector<std::vector<IntervalAabb>> terminal_hold_aabbs_;
    OctopusConfig config_;
    int num_segments_ = 0;
    int num_control_points_ = 0;
    int final_search_index_ = 0;
    std::vector<std::tuple<int, int, int>> sample_combinations_;
    std::size_t separator_calls_ = 0;
    std::size_t aabb_separation_skips_ = 0;
    std::size_t terminal_hold_rejections_ = 0;
    std::size_t terminal_hold_separator_calls_ = 0;
    double voxel_size_ = 0.0;
    Vec3 voxel_origin_ = Vec3::Zero();
    Separator separator_;
    OctopusStopPredicate stop_requested_;
    std::string stop_reason_;
};

}  // namespace dynamic_planner
