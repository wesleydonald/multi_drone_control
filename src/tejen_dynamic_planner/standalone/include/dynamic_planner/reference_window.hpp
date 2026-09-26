#pragma once

#include <cstddef>
#include <vector>

#include "dynamic_planner/committed_trajectory.hpp"

namespace dynamic_planner {

// R6.3C.1a planner->MPC sampling contract.
//
// The current payload MPC runs its control/reference loop at 30 Hz, uses a
// 20-stage OCP, and takes every third trajectory sample. Therefore a complete
// rolling reference window contains 20*3 + 1 = 61 samples and spans 2.0 s.
// Sample 0 always means the authoritative committed reference at "now".
struct ReferenceWindowConfig {
    double reference_rate_hz = 30.0;
    std::size_t mpc_horizon_stages = 20U;
    std::size_t mpc_skip_steps = 3U;

    void validate() const;
    double samplePeriodS() const;
    std::size_t requiredSampleCount() const;
    double horizonDurationS() const;
};

struct ReferenceSample {
    double absolute_time_s = 0.0;
    double time_from_start_s = 0.0;
    State state;
};

struct ReferenceWindow {
    double sampled_from_s = 0.0;
    double sample_period_s = 0.0;
    std::vector<ReferenceSample> samples;
};

// Evaluate the authoritative CommittedTrajectory directly. This function does
// not fit, smooth, interpolate, or otherwise alter the committed B-spline.
ReferenceWindow sampleCommittedReferenceWindow(
    const CommittedTrajectory& trajectory,
    double now_s,
    const ReferenceWindowConfig& config = {});


}  // namespace dynamic_planner
