#pragma once

#include <cstddef>
#include <string>

#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/world_snapshot.hpp"

namespace dynamic_planner {

struct TrajectorySafetyResult {
    bool safe = false;
    std::string status;
    double checked_from_s = 0.0;
    double checked_until_s = 0.0;
    std::size_t separator_lp_calls = 0;
    // Exact broad-phase skips.  AABB-disjoint convex hulls are provably
    // separated, so the expensive LP separator is unnecessary in these cases.
    std::size_t aabb_separation_skips = 0;
    std::string obstacle_name;
    double unsafe_interval_start_s = 0.0;
    double unsafe_interval_end_s = 0.0;
};

class TrajectorySafetyChecker {
public:
    explicit TrajectorySafetyChecker(double separator_validation_tolerance = 1e-7)
        : separator_validation_tolerance_(separator_validation_tolerance) {}

    TrajectorySafetyResult checkPiece(
        const TrajectoryPiece& ego,
        double from_s,
        double until_s,
        const WorldSnapshot& world) const;

    TrajectorySafetyResult checkCommitted(
        const CommittedTrajectory& ego,
        double from_s,
        double until_s,
        const WorldSnapshot& world) const;

private:
    double separator_validation_tolerance_ = 1e-7;
};

}  // namespace dynamic_planner
