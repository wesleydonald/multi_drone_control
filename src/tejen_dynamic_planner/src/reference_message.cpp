#include "tejen_dynamic_planner/reference_message.hpp"

#include <cmath>
#include <cstddef>
#include <builtin_interfaces/msg/duration.hpp>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

namespace tejen_dynamic_planner {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

builtin_interfaces::msg::Duration durationFromSeconds(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        throw std::invalid_argument("trajectory time_from_start must be finite and nonnegative");
    }

    const long double nanoseconds_ld =
        static_cast<long double>(seconds) * static_cast<long double>(kNanosecondsPerSecond);
    if (nanoseconds_ld >
        static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::overflow_error("trajectory time_from_start exceeds int64 nanoseconds");
    }
    const std::int64_t nanoseconds = static_cast<std::int64_t>(std::llround(nanoseconds_ld));
    const std::int64_t seconds_whole = nanoseconds / kNanosecondsPerSecond;
    if (seconds_whole > std::numeric_limits<std::int32_t>::max()) {
        throw std::overflow_error("trajectory time_from_start exceeds ROS Duration seconds range");
    }

    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<std::int32_t>(seconds_whole);
    duration.nanosec = static_cast<std::uint32_t>(
        nanoseconds % kNanosecondsPerSecond);
    return duration;
}

void validateState(const dynamic_planner::State& state) {
    if (!state.position.allFinite() || !state.velocity.allFinite() ||
        !state.acceleration.allFinite()) {
        throw std::invalid_argument("reference window contains non-finite state");
    }
}

}  // namespace

trajectory_msgs::msg::MultiDOFJointTrajectory makeReferenceMessage(
    const dynamic_planner::ReferenceWindow& window,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id,
    const std::string& vehicle_id) {
    if (window.samples.empty()) {
        throw std::invalid_argument("cannot encode an empty reference window");
    }
    if (!std::isfinite(window.sampled_from_s) ||
        !std::isfinite(window.sample_period_s) || !(window.sample_period_s > 0.0)) {
        throw std::invalid_argument("reference window has invalid timing metadata");
    }
    if (frame_id.empty()) {
        throw std::invalid_argument("reference frame_id must not be empty");
    }
    if (vehicle_id.empty()) {
        throw std::invalid_argument("reference vehicle_id must not be empty");
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.joint_names = {vehicle_id};
    message.points.reserve(window.samples.size());

    double previous_offset_s = -1.0;
    for (std::size_t k = 0U; k < window.samples.size(); ++k) {
        const auto& sample = window.samples[k];
        validateState(sample.state);
        if (!std::isfinite(sample.absolute_time_s) ||
            !std::isfinite(sample.time_from_start_s) ||
            sample.time_from_start_s < 0.0) {
            throw std::invalid_argument("reference sample contains invalid time");
        }
        const double expected_offset_s = static_cast<double>(k) * window.sample_period_s;
        if (std::abs(sample.time_from_start_s - expected_offset_s) > 1e-9 ||
            std::abs(sample.absolute_time_s -
                     (window.sampled_from_s + sample.time_from_start_s)) > 1e-9) {
            throw std::invalid_argument(
                "reference sample timing does not match rolling-window metadata");
        }
        if (k == 0U && std::abs(sample.time_from_start_s) > 1e-12) {
            throw std::invalid_argument("reference sample zero must mean now");
        }
        if (k > 0U && sample.time_from_start_s <= previous_offset_s) {
            throw std::invalid_argument(
                "reference samples must have strictly increasing offsets");
        }

        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

        geometry_msgs::msg::Transform transform;
        transform.translation.x = sample.state.position.x();
        transform.translation.y = sample.state.position.y();
        transform.translation.z = sample.state.position.z();
        transform.rotation.w = 1.0;
        transform.rotation.x = 0.0;
        transform.rotation.y = 0.0;
        transform.rotation.z = 0.0;
        point.transforms.push_back(transform);

        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = sample.state.velocity.x();
        velocity.linear.y = sample.state.velocity.y();
        velocity.linear.z = sample.state.velocity.z();
        point.velocities.push_back(velocity);

        geometry_msgs::msg::Twist acceleration;
        acceleration.linear.x = sample.state.acceleration.x();
        acceleration.linear.y = sample.state.acceleration.y();
        acceleration.linear.z = sample.state.acceleration.z();
        point.accelerations.push_back(acceleration);

        point.time_from_start = durationFromSeconds(sample.time_from_start_s);
        message.points.push_back(std::move(point));
        previous_offset_s = sample.time_from_start_s;
    }

    return message;
}

}  // namespace tejen_dynamic_planner
