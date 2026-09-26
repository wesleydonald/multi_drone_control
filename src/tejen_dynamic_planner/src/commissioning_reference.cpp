#include "tejen_dynamic_planner/commissioning_reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tejen_dynamic_planner {

SeventhOrderStep seventhOrderRestToRest(double elapsed_s, double duration_s) {
    if (!std::isfinite(elapsed_s) || !std::isfinite(duration_s) || !(duration_s > 0.0)) {
        throw std::invalid_argument("seventh-order step requires finite elapsed time and positive duration");
    }

    const double tau = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;

    SeventhOrderStep out;
    out.position_fraction = 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;

    const double ds_dtau = 140.0 * t3 - 420.0 * t4 + 420.0 * t5 - 140.0 * t6;
    const double d2s_dtau2 = 420.0 * t2 - 1680.0 * t3 + 2100.0 * t4 - 840.0 * t5;
    const double d3s_dtau3 = 840.0 * tau - 5040.0 * t2 + 8400.0 * t3 - 4200.0 * t4;

    out.velocity_fraction_per_s = ds_dtau / duration_s;
    out.acceleration_fraction_per_s2 = d2s_dtau2 / (duration_s * duration_s);
    out.jerk_fraction_per_s3 = d3s_dtau3 / (duration_s * duration_s * duration_s);
    return out;
}

dynamic_planner::ReferenceWindow makeVerticalTakeoffReferenceWindow(
    const dynamic_planner::Vec3& start,
    double target_z,
    double takeoff_start_s,
    double now_s,
    double duration_s,
    const dynamic_planner::ReferenceWindowConfig& config) {
    config.validate();
    if (!start.allFinite() || !std::isfinite(target_z) ||
        !std::isfinite(takeoff_start_s) || !std::isfinite(now_s) ||
        !std::isfinite(duration_s) || !(duration_s > 0.0)) {
        throw std::invalid_argument("invalid C.1d takeoff-reference arguments");
    }

    dynamic_planner::ReferenceWindow window;
    window.sampled_from_s = now_s;
    window.sample_period_s = config.samplePeriodS();
    window.samples.reserve(config.requiredSampleCount());

    const double dz = target_z - start.z();
    for (std::size_t k = 0U; k < config.requiredSampleCount(); ++k) {
        const double offset_s = static_cast<double>(k) * window.sample_period_s;
        const double absolute_time_s = now_s + offset_s;
        const SeventhOrderStep step = seventhOrderRestToRest(
            absolute_time_s - takeoff_start_s, duration_s);

        dynamic_planner::ReferenceSample sample;
        sample.absolute_time_s = absolute_time_s;
        sample.time_from_start_s = offset_s;
        sample.state.position = start;
        sample.state.position.z() = start.z() + dz * step.position_fraction;
        sample.state.velocity.setZero();
        sample.state.velocity.z() = dz * step.velocity_fraction_per_s;
        sample.state.acceleration.setZero();
        sample.state.acceleration.z() = dz * step.acceleration_fraction_per_s2;
        window.samples.push_back(std::move(sample));
    }
    return window;
}

dynamic_planner::ReferenceWindow makeStationaryReferenceWindow(
    const dynamic_planner::Vec3& position,
    double now_s,
    const dynamic_planner::ReferenceWindowConfig& config) {
    config.validate();
    if (!position.allFinite() || !std::isfinite(now_s)) {
        throw std::invalid_argument("invalid stationary-reference arguments");
    }

    dynamic_planner::ReferenceWindow window;
    window.sampled_from_s = now_s;
    window.sample_period_s = config.samplePeriodS();
    window.samples.reserve(config.requiredSampleCount());

    for (std::size_t k = 0U; k < config.requiredSampleCount(); ++k) {
        const double offset_s = static_cast<double>(k) * window.sample_period_s;
        dynamic_planner::ReferenceSample sample;
        sample.absolute_time_s = now_s + offset_s;
        sample.time_from_start_s = offset_s;
        sample.state.position = position;
        sample.state.velocity.setZero();
        sample.state.acceleration.setZero();
        window.samples.push_back(std::move(sample));
    }
    return window;
}

}  // namespace tejen_dynamic_planner
