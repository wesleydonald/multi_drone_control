#pragma once

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "dynamic_planner/reference_window.hpp"

namespace tejen_dynamic_planner {

// Encode a pre-sampled authoritative rolling window without changing its
// geometry or dynamics. Position, velocity and acceleration are copied exactly
// into the public MultiDOFJointTrajectory fields. The current MPC consumes
// index 0 as "now" and every third sample thereafter.
trajectory_msgs::msg::MultiDOFJointTrajectory makeReferenceMessage(
    const dynamic_planner::ReferenceWindow& window,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id,
    const std::string& vehicle_id);

}  // namespace tejen_dynamic_planner
