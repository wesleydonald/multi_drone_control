#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dynamic_planner/bspline.hpp"
#include "dynamic_planner/octopus_search.hpp"
#include "dynamic_planner/trajectory_refinement.hpp"

namespace dynamic_planner {

struct LocalPlannerConfig {
    int num_segments = 4;
    OctopusConfig octopus;
    RefinementConfig refinement;
};

struct LocalPlanRequest {
    State initial_state;
    Vec3 local_goal = Vec3::Zero();
    double t_start = 0.0;
    double duration_s = 0.0;
    std::vector<TimeIndexedObstacle> obstacles;
    std::vector<TimeIndexedObstacle> terminal_hold_obstacles;
    TerminalBoundary terminal_boundary;
    OctopusStopPredicate octopus_stop_requested;
    std::string octopus_stop_reason = "EXTERNAL_STOP_REQUESTED";
};

struct LocalPlanResult {
    bool success = false;
    bool partial = false;
    std::string status;
    std::string message;
    // Immutable diagnostic: the terminal contract requested by the receding-horizon
    // layer before Octopus/refinement may substitute a stopped safe partial.
    TerminalBoundary requested_terminal_boundary;
    TerminalBoundary effective_terminal_boundary;
    std::vector<double> knots;
    OctopusResult search;
    std::optional<RefinementResult> refinement;
    std::optional<ControlPoints> control_points;
    double total_solve_time_s = 0.0;
};

class LocalPlanner {
public:
    explicit LocalPlanner(LocalPlannerConfig config);

    LocalPlanResult plan(const LocalPlanRequest& request) const;
    const LocalPlannerConfig& config() const noexcept { return config_; }

private:
    LocalPlannerConfig config_;
};

}  // namespace dynamic_planner
