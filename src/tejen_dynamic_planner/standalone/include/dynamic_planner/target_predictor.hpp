#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "dynamic_planner/committed_trajectory.hpp"
#include "dynamic_planner/types.hpp"

namespace dynamic_planner {

// Time is an absolute trajectory/ROS-clock time.  A prediction is explicit
// about validity so callers can fail closed instead of silently reverting to
// an unbounded constant-velocity extrapolation.
struct TargetPrediction {
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    bool valid = false;
    std::string detail;
};

class TargetPredictor {
public:
    virtual ~TargetPredictor() = default;
    virtual TargetPrediction evaluate(double future_time_s) const = 0;
    virtual const char* type() const noexcept = 0;
    virtual bool highConfidenceFuture() const noexcept = 0;
};

class ConstantVelocityTargetPredictor final : public TargetPredictor {
public:
    ConstantVelocityTargetPredictor(Vec3 position, Vec3 velocity, double reference_time_s)
        : position_(std::move(position)), velocity_(std::move(velocity)),
          reference_time_s_(reference_time_s) {}

    TargetPrediction evaluate(double future_time_s) const override {
        if (!position_.allFinite() || !velocity_.allFinite() ||
            !std::isfinite(reference_time_s_) || !std::isfinite(future_time_s)) {
            return {{}, {}, false, "NONFINITE_CONSTANT_VELOCITY_INPUT"};
        }
        return {position_ + (future_time_s - reference_time_s_) * velocity_, velocity_, true,
                "CONSTANT_VELOCITY"};
    }
    const char* type() const noexcept override { return "constant_velocity"; }
    bool highConfidenceFuture() const noexcept override { return false; }

private:
    Vec3 position_;
    Vec3 velocity_;
    double reference_time_s_;
};

// Deterministic simulation provider. The measured target position fixes the
// circle phase at reference_time_s; subsequent evaluations use the configured
// scripted dynamics, rather than simulator-only future-state access.
class ScriptedCircleTargetPredictor final : public TargetPredictor {
public:
    ScriptedCircleTargetPredictor(Vec3 centre, double radius_m, double omega_rad_s,
                                  Vec3 observed_position, double reference_time_s)
        : centre_(std::move(centre)), radius_m_(radius_m), omega_rad_s_(omega_rad_s),
          reference_time_s_(reference_time_s) {
        if (!centre_.allFinite() || !observed_position.allFinite() ||
            !std::isfinite(radius_m_) || !(radius_m_ > 0.0) ||
            !std::isfinite(omega_rad_s_) || !std::isfinite(reference_time_s_)) {
            throw std::invalid_argument("invalid scripted-circle target predictor configuration");
        }
        const Vec3 relative = observed_position - centre_;
        const double observed_radius = std::hypot(relative.x(), relative.y());
        if (!std::isfinite(observed_radius) || std::abs(observed_radius - radius_m_) > 0.05) {
            throw std::invalid_argument("observed target is inconsistent with scripted circle");
        }
        phase_at_reference_rad_ = std::atan2(relative.y(), relative.x());
    }

    TargetPrediction evaluate(double future_time_s) const override {
        if (!std::isfinite(future_time_s)) {
            return {{}, {}, false, "NONFINITE_PREDICTION_TIME"};
        }
        const double phase = phase_at_reference_rad_ +
            omega_rad_s_ * (future_time_s - reference_time_s_);
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const Vec3 position = centre_ + Vec3(radius_m_ * c, radius_m_ * s, 0.0);
        const Vec3 velocity(-radius_m_ * omega_rad_s_ * s,
                            radius_m_ * omega_rad_s_ * c, 0.0);
        return {position, velocity, position.allFinite() && velocity.allFinite(), "SCRIPTED_CIRCLE"};
    }
    const char* type() const noexcept override { return "scripted_circle"; }
    bool highConfidenceFuture() const noexcept override { return true; }

private:
    Vec3 centre_;
    double radius_m_;
    double omega_rad_s_;
    double reference_time_s_;
    double phase_at_reference_rad_ = 0.0;
};

// High-confidence provider backed by the exact advertised committed trajectory
// used elsewhere for dynamic collision geometry. The constant position offset
// maps the moving body's advertised centre to the mission's desired quad-body
// rendezvous point, so avoidance and rendezvous share one future source.
class CommittedTrajectoryTargetPredictor final : public TargetPredictor {
public:
    CommittedTrajectoryTargetPredictor(CommittedTrajectory trajectory, Vec3 position_offset)
        : trajectory_(std::move(trajectory)), position_offset_(std::move(position_offset)) {
        if (trajectory_.empty() || !position_offset_.allFinite()) {
            throw std::invalid_argument("invalid committed-trajectory target predictor");
        }
    }

    TargetPrediction evaluate(double future_time_s) const override {
        if (!std::isfinite(future_time_s)) {
            return {{}, {}, false, "NONFINITE_PREDICTION_TIME"};
        }
        try {
            const State state = trajectory_.evaluate(future_time_s);
            const Vec3 position = state.position + position_offset_;
            return {position, state.velocity, position.allFinite() && state.velocity.allFinite(),
                    "COMMITTED_TRAJECTORY"};
        } catch (const std::exception& exc) {
            return {{}, {}, false, std::string("COMMITTED_TRAJECTORY_OUT_OF_RANGE:") + exc.what()};
        }
    }
    const char* type() const noexcept override { return "committed_trajectory"; }
    bool highConfidenceFuture() const noexcept override { return true; }

private:
    CommittedTrajectory trajectory_;
    Vec3 position_offset_ = Vec3::Zero();
};

using TargetPredictorPtr = std::unique_ptr<TargetPredictor>;

}  // namespace dynamic_planner
