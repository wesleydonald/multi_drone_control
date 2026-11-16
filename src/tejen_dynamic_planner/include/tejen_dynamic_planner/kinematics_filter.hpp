#pragma once

#include <optional>

#include "dynamic_planner/types.hpp"

namespace tejen_dynamic_planner {

class KinematicsFilter {
public:
    explicit KinematicsFilter(double acceleration_tau_s = 0.15);

    dynamic_planner::State update(
        const dynamic_planner::Vec3& position,
        const dynamic_planner::Vec3& velocity,
        double sample_time_s);

    void reset();
    bool initialized() const noexcept { return previous_velocity_.has_value(); }
    double accelerationTauS() const noexcept { return acceleration_tau_s_; }

private:
    double acceleration_tau_s_ = 0.15;
    std::optional<dynamic_planner::Vec3> previous_velocity_;
    std::optional<double> previous_time_s_;
    dynamic_planner::Vec3 filtered_acceleration_ = dynamic_planner::Vec3::Zero();
};

}  // namespace tejen_dynamic_planner
