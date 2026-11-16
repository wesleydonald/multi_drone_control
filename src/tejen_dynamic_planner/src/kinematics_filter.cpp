#include "tejen_dynamic_planner/kinematics_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace tejen_dynamic_planner {

KinematicsFilter::KinematicsFilter(double acceleration_tau_s)
    : acceleration_tau_s_(acceleration_tau_s) {
    if (!std::isfinite(acceleration_tau_s_) || !(acceleration_tau_s_ > 0.0)) {
        throw std::invalid_argument("acceleration filter tau must be finite and positive");
    }
}

void KinematicsFilter::reset() {
    previous_velocity_.reset();
    previous_time_s_.reset();
    filtered_acceleration_.setZero();
}

dynamic_planner::State KinematicsFilter::update(
    const dynamic_planner::Vec3& position,
    const dynamic_planner::Vec3& velocity,
    double sample_time_s) {
    if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(sample_time_s)) {
        throw std::invalid_argument("kinematics sample must be finite");
    }

    dynamic_planner::State state;
    state.position = position;
    state.velocity = velocity;

    if (!previous_velocity_.has_value() || !previous_time_s_.has_value()) {
        filtered_acceleration_.setZero();
    } else {
        const double dt = sample_time_s - *previous_time_s_;
        if (!(dt > 1e-6)) {
            // Duplicate/out-of-order timestamps cannot support a derivative.
            // Reset the differentiator rather than injecting an acceleration spike.
            previous_velocity_ = velocity;
            previous_time_s_ = sample_time_s;
            filtered_acceleration_.setZero();
            state.acceleration = filtered_acceleration_;
            return state;
        }
        const dynamic_planner::Vec3 raw_acceleration =
            (velocity - *previous_velocity_) / dt;
        const double alpha = 1.0 - std::exp(-dt / acceleration_tau_s_);
        filtered_acceleration_ += alpha * (raw_acceleration - filtered_acceleration_);
    }

    previous_velocity_ = velocity;
    previous_time_s_ = sample_time_s;
    state.acceleration = filtered_acceleration_;
    return state;
}

}  // namespace tejen_dynamic_planner
