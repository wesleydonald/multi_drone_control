#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dynamic_planner {

// Timestamp representation precision, not additional lead or collision margin.
inline double timestampUlp(double value) {
    if (!std::isfinite(value)) return std::numeric_limits<double>::infinity();
    const double infinity = std::numeric_limits<double>::infinity();
    return std::max(std::abs(std::nextafter(value, infinity) - value),
                    std::abs(value - std::nextafter(value, -infinity)));
}

inline bool strictlyBeforeSplice(double now, double splice) {
    return std::isfinite(now) && std::isfinite(splice) &&
        splice - now > std::max(1e-12, timestampUlp(now) + timestampUlp(splice));
}

// Existing equation; extraction does not enable adaptive timing in IRL.
inline double adaptiveSpliceLookahead(double elapsed, double dt, double alpha,
                                     double minimum, double maximum) {
    if (!std::isfinite(elapsed) || elapsed < 0 || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(alpha) || alpha <= 0 || !std::isfinite(minimum) ||
        !std::isfinite(maximum) || minimum < 0 || maximum < minimum)
        throw std::invalid_argument("invalid adaptive timing inputs");
    return std::clamp(alpha * std::max(1.0, std::ceil(elapsed / dt)) * dt,
                      minimum, maximum);
}

}  // namespace dynamic_planner
