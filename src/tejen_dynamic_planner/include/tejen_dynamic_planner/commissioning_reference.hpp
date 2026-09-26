#pragma once

#include "dynamic_planner/reference_window.hpp"
#include "dynamic_planner/types.hpp"

namespace tejen_dynamic_planner {

struct SeventhOrderStep {
    double position_fraction = 0.0;
    double velocity_fraction_per_s = 0.0;
    double acceleration_fraction_per_s2 = 0.0;
    double jerk_fraction_per_s3 = 0.0;
};

// Seventh-order rest-to-rest step with zero endpoint velocity, acceleration,
// and jerk. elapsed_s is clamped to [0, duration_s].
SeventhOrderStep seventhOrderRestToRest(double elapsed_s, double duration_s);

// Generate the exact 61-sample (for the current default MPC contract) rolling
// reference used during the C.1d commissioning takeoff. X/Y remain fixed and Z
// follows the seventh-order rest-to-rest step from start.z() to target_z.
dynamic_planner::ReferenceWindow makeVerticalTakeoffReferenceWindow(
    const dynamic_planner::Vec3& start,
    double target_z,
    double takeoff_start_s,
    double now_s,
    double duration_s,
    const dynamic_planner::ReferenceWindowConfig& config = {});

// Generate a stationary rolling reference directly, without constructing a
// planner or changing any committed-trajectory state.
dynamic_planner::ReferenceWindow makeStationaryReferenceWindow(
    const dynamic_planner::Vec3& position,
    double now_s,
    const dynamic_planner::ReferenceWindowConfig& config = {});

}  // namespace tejen_dynamic_planner
