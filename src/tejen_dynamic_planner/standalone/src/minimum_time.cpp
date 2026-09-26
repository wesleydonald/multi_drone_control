#include "dynamic_planner/minimum_time.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {
namespace {

int signum(double value) {
    return (0.0 < value) - (value < 0.0);
}

double safeSqrt(double value) {
    if (value < -1e-10) {
        throw std::runtime_error("negative radicand in double-integrator minimum-time calculation");
    }
    return std::sqrt(std::max(0.0, value));
}

}  // namespace

double minimumTimeDoubleIntegrator1D(double p0, double v0,
                                     double pf, double vf,
                                     double v_max, double a_max) {
    if (!std::isfinite(p0) || !std::isfinite(v0) ||
        !std::isfinite(pf) || !std::isfinite(vf) ||
        !std::isfinite(v_max) || !std::isfinite(a_max) ||
        !(v_max > 0.0) || !(a_max > 0.0)) {
        throw std::invalid_argument("minimum-time inputs must be finite and limits positive");
    }

    // Keep the notation and branch structure of RMADER utils.cpp exactly.
    const double x1 = v0;
    const double x2 = p0;
    const double x1r = vf;
    const double x2r = pf;
    const double k1 = a_max;
    const double k2 = 1.0;
    const double x1_bar = v_max;

    const double B = (k2 / (2.0 * k1)) * static_cast<double>(signum(-x1 + x1r))
                   * (x1 * x1 - x1r * x1r) + x2r;
    const double C = (k2 / (2.0 * k1)) * (x1 * x1 + x1r * x1r)
                   - (k2 / k1) * x1_bar * x1_bar + x2r;
    const double D = (-k2 / (2.0 * k1)) * (x1 * x1 + x1r * x1r)
                   + (k2 / k1) * x1_bar * x1_bar + x2r;

    double time = 0.0;
    if ((x2 <= B) && (x2 >= C)) {
        const double radicand = k2 * k2 * x1 * x1
            - k1 * k2 * ((k2 / (2.0 * k1)) * (x1 * x1 - x1r * x1r) + x2 - x2r);
        time = (-k2 * (x1 + x1r) + 2.0 * safeSqrt(radicand)) / (k1 * k2);
    } else if ((x2 <= B) && (x2 < C)) {
        time = (x1_bar - x1 - x1r) / k1
             + (x1 * x1 + x1r * x1r) / (2.0 * k1 * x1_bar)
             + (x2r - x2) / (k2 * x1_bar);
    } else if ((x2 > B) && (x2 <= D)) {
        const double radicand = k2 * k2 * x1 * x1
            + k1 * k2 * ((k2 / (2.0 * k1)) * (-x1 * x1 + x1r * x1r) + x2 - x2r);
        time = (k2 * (x1 + x1r) + 2.0 * safeSqrt(radicand)) / (k1 * k2);
    } else {
        time = (x1_bar + x1 + x1r) / k1
             + (x1 * x1 + x1r * x1r) / (2.0 * k1 * x1_bar)
             + (-x2r + x2) / (k2 * x1_bar);
    }

    if (!std::isfinite(time) || time < -1e-10) {
        throw std::runtime_error("invalid minimum-time result");
    }
    return std::max(0.0, time);
}

double minimumTimeDoubleIntegrator3D(const Vec3& p0, const Vec3& v0,
                                     const Vec3& pf, const Vec3& vf,
                                     const Vec3& v_max, const Vec3& a_max) {
    if (!p0.allFinite() || !v0.allFinite() || !pf.allFinite() || !vf.allFinite() ||
        !v_max.allFinite() || !a_max.allFinite() ||
        !(v_max.array() > 0.0).all() || !(a_max.array() > 0.0).all()) {
        throw std::invalid_argument("3D minimum-time inputs must be finite and limits positive");
    }
    const double tx = minimumTimeDoubleIntegrator1D(
        p0.x(), v0.x(), pf.x(), vf.x(), v_max.x(), a_max.x());
    const double ty = minimumTimeDoubleIntegrator1D(
        p0.y(), v0.y(), pf.y(), vf.y(), v_max.y(), a_max.y());
    const double tz = minimumTimeDoubleIntegrator1D(
        p0.z(), v0.z(), pf.z(), vf.z(), v_max.z(), a_max.z());
    return std::max({tx, ty, tz});
}

}  // namespace dynamic_planner
