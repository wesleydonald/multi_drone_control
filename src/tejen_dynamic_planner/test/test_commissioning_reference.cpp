#include "tejen_dynamic_planner/commissioning_reference.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool near(double a, double b, double tol = 1e-10) {
    return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
    try {
        const double T = 4.0;
        const auto a = tejen_dynamic_planner::seventhOrderRestToRest(0.0, T);
        const auto b = tejen_dynamic_planner::seventhOrderRestToRest(T, T);
        require(near(a.position_fraction, 0.0), "start position fraction");
        require(near(a.velocity_fraction_per_s, 0.0), "start velocity");
        require(near(a.acceleration_fraction_per_s2, 0.0), "start acceleration");
        require(near(a.jerk_fraction_per_s3, 0.0), "start jerk");
        require(near(b.position_fraction, 1.0), "end position fraction");
        require(near(b.velocity_fraction_per_s, 0.0), "end velocity");
        require(near(b.acceleration_fraction_per_s2, 0.0), "end acceleration");
        require(near(b.jerk_fraction_per_s3, 0.0), "end jerk");

        dynamic_planner::ReferenceWindowConfig cfg;
        cfg.reference_rate_hz = 30.0;
        cfg.mpc_horizon_stages = 20U;
        cfg.mpc_skip_steps = 3U;
        const dynamic_planner::Vec3 start(0.2, -0.1, 0.05);
        const auto window = tejen_dynamic_planner::makeVerticalTakeoffReferenceWindow(
            start, 1.20, 10.0, 10.0, T, cfg);
        require(window.samples.size() == 61U, "takeoff window must contain 61 samples");
        require((window.samples.front().state.position - start).norm() <= 1e-12,
                "takeoff sample zero must begin at start");
        require(near(window.samples.front().time_from_start_s, 0.0),
                "sample zero must mean now");

        const auto hold = tejen_dynamic_planner::makeStationaryReferenceWindow(
            dynamic_planner::Vec3(1.0, 2.0, 3.0), 20.0, cfg);
        require(hold.samples.size() == 61U, "stationary window must contain 61 samples");
        for (const auto& sample : hold.samples) {
            require((sample.state.position - dynamic_planner::Vec3(1.0, 2.0, 3.0)).norm() <= 1e-12,
                    "stationary position changed");
            require(sample.state.velocity.norm() <= 1e-12, "stationary velocity nonzero");
            require(sample.state.acceleration.norm() <= 1e-12, "stationary acceleration nonzero");
        }

        // The rolling horizon straddles the end of ascent. Samples after T must
        // remain exactly at the endpoint with v=a=0 rather than overshoot.
        const auto end_window = tejen_dynamic_planner::makeVerticalTakeoffReferenceWindow(
            start, 1.20, 10.0, 13.9, T, cfg);
        require(end_window.samples.back().state.position.z() <= 1.20 + 1e-12,
                "takeoff reference overshot target z");
        require(near(end_window.samples.back().state.position.z(), 1.20, 1e-12),
                "takeoff reference did not clamp to target z");
        require(end_window.samples.back().state.velocity.norm() <= 1e-12,
                "post-ascent velocity nonzero");
        require(end_window.samples.back().state.acceleration.norm() <= 1e-12,
                "post-ascent acceleration nonzero");

        std::cout << "test_commissioning_reference: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_commissioning_reference: FAIL: " << exc.what() << '\n';
        return 1;
    }
}
