#include "dynamic_planner/reference_window.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace dynamic_planner {

void ReferenceWindowConfig::validate() const {
    if (!std::isfinite(reference_rate_hz) || !(reference_rate_hz > 0.0)) {
        throw std::invalid_argument("reference_rate_hz must be finite and positive");
    }
    if (mpc_horizon_stages == 0U) {
        throw std::invalid_argument("mpc_horizon_stages must be positive");
    }
    if (mpc_skip_steps == 0U) {
        throw std::invalid_argument("mpc_skip_steps must be positive");
    }
    if (mpc_horizon_stages >
        (std::numeric_limits<std::size_t>::max() - 1U) / mpc_skip_steps) {
        throw std::overflow_error("reference-window sample count overflows size_t");
    }
}

double ReferenceWindowConfig::samplePeriodS() const {
    validate();
    return 1.0 / reference_rate_hz;
}

std::size_t ReferenceWindowConfig::requiredSampleCount() const {
    validate();
    return mpc_horizon_stages * mpc_skip_steps + 1U;
}

double ReferenceWindowConfig::horizonDurationS() const {
    validate();
    return static_cast<double>(mpc_horizon_stages * mpc_skip_steps) /
           reference_rate_hz;
}

ReferenceWindow sampleCommittedReferenceWindow(
    const CommittedTrajectory& trajectory,
    double now_s,
    const ReferenceWindowConfig& config) {
    config.validate();
    if (trajectory.empty()) {
        throw std::invalid_argument("cannot sample an empty committed trajectory");
    }
    if (!std::isfinite(now_s)) {
        throw std::invalid_argument("reference-window now_s must be finite");
    }
    if (now_s < trajectory.startTime() - kTrajectoryTimeToleranceS) {
        throw std::out_of_range("reference-window start lies before committed trajectory");
    }

    ReferenceWindow result;
    result.sampled_from_s = now_s;
    result.sample_period_s = config.samplePeriodS();
    result.samples.reserve(config.requiredSampleCount());

    const std::size_t count = config.requiredSampleCount();
    for (std::size_t k = 0U; k < count; ++k) {
        const double offset_s = static_cast<double>(k) / config.reference_rate_hz;
        const double absolute_time_s = now_s + offset_s;

        ReferenceSample sample;
        sample.absolute_time_s = absolute_time_s;
        sample.time_from_start_s = offset_s;
        sample.state = trajectory.evaluate(absolute_time_s);
        result.samples.push_back(std::move(sample));
    }

    return result;
}


}  // namespace dynamic_planner
