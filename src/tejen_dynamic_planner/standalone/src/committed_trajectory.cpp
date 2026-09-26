#include "dynamic_planner/committed_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {
namespace {

TrajectoryPiece stationaryHoldPiece(const State& stopped_state,
                                    double valid_from,
                                    double valid_until) {
    if (!std::isfinite(valid_from) || !std::isfinite(valid_until) ||
        !(valid_until > valid_from)) {
        throw std::invalid_argument("invalid stationary hold interval");
    }
    TrajectoryPiece hold;
    hold.knots = openUniformKnots(valid_from, valid_until, 1);
    hold.control_points = ControlPoints::Zero(4, 3);
    for (Eigen::Index row = 0; row < hold.control_points.rows(); ++row) {
        hold.control_points.row(row) = stopped_state.position.transpose();
    }
    hold.valid_from = valid_from;
    hold.valid_until = valid_until;
    return hold;
}

State exactStationaryHoldState(const State& endpoint) {
    State hold = endpoint;
    hold.velocity.setZero();
    hold.acceleration.setZero();
    return hold;
}

}  // namespace

void TrajectoryPiece::validate() const {
    if (control_points.cols() != 3 || control_points.rows() < 4 || !control_points.allFinite()) {
        throw std::invalid_argument("trajectory piece has invalid control points");
    }
    if (knots.size() != static_cast<std::size_t>(control_points.rows() + kCubicDegree + 1)) {
        throw std::invalid_argument("trajectory piece knot/control-point size mismatch");
    }
    if (!std::isfinite(valid_from) || !std::isfinite(valid_until) || valid_until < valid_from) {
        throw std::invalid_argument("trajectory piece has invalid logical time range");
    }
    const double spline_start = knots.at(static_cast<std::size_t>(kCubicDegree));
    const double spline_end = knots.at(static_cast<std::size_t>(control_points.rows()));
    if (valid_from < spline_start - kTrajectoryTimeToleranceS || valid_until > spline_end + kTrajectoryTimeToleranceS) {
        throw std::invalid_argument("logical time range lies outside spline domain");
    }
}

State TrajectoryPiece::evaluate(double t) const {
    validate();
    if (!std::isfinite(t) || t < valid_from - kTrajectoryTimeToleranceS || t > valid_until + kTrajectoryTimeToleranceS) {
        throw std::out_of_range("trajectory-piece evaluation outside logical validity range");
    }
    return evaluateCubicState(control_points, knots, std::clamp(t, valid_from, valid_until));
}

double CommittedTrajectory::startTime() const {
    if (pieces_.empty()) {
        throw std::runtime_error("committed trajectory is empty");
    }
    return pieces_.front().valid_from;
}

double CommittedTrajectory::endTime() const {
    if (pieces_.empty()) {
        throw std::runtime_error("committed trajectory is empty");
    }
    return pieces_.back().valid_until;
}

bool CommittedTrajectory::endsInStoppedHold(double tolerance) const {
    if (pieces_.empty() || !(tolerance >= 0.0) || !std::isfinite(tolerance)) {
        return false;
    }
    const auto& last = pieces_.back();
    const State endpoint = evaluateCubicState(last.control_points, last.knots, last.valid_until);
    return endpoint.position.allFinite() && endpoint.velocity.allFinite() &&
           endpoint.acceleration.allFinite() &&
           endpoint.velocity.cwiseAbs().maxCoeff() <= tolerance &&
           endpoint.acceleration.cwiseAbs().maxCoeff() <= tolerance;
}

State CommittedTrajectory::evaluate(double t) const {
    if (pieces_.empty()) {
        throw std::runtime_error("committed trajectory is empty");
    }
    if (!std::isfinite(t) || t < startTime() - kTrajectoryTimeToleranceS) {
        throw std::out_of_range("committed-trajectory evaluation outside range");
    }

    // At a splice boundary prefer the newer suffix. C2 continuity means either
    // side is mathematically identical through acceleration.
    for (auto it = pieces_.rbegin(); it != pieces_.rend(); ++it) {
        if (t >= it->valid_from - kTrajectoryTimeToleranceS && t <= it->valid_until + kTrajectoryTimeToleranceS) {
            return evaluateCubicState(
                it->control_points, it->knots,
                std::clamp(t, it->valid_from, it->valid_until));
        }
    }

    // B.1 terminal-hold contract: a certified stopping endpoint remains a
    // valid stationary command after the explicit spline finishes. This makes
    // the terminal stop a true fallback state rather than a dead end. A later
    // replan may choose its future splice A inside this hold.
    if (t > endTime() + kTrajectoryTimeToleranceS && endsInStoppedHold()) {
        const auto& last = pieces_.back();
        const State endpoint = evaluateCubicState(
            last.control_points, last.knots, last.valid_until);
        return exactStationaryHoldState(endpoint);
    }

    throw std::runtime_error("no committed piece covers requested time");
}

State CommittedTrajectory::endState() const {
    return evaluate(endTime());
}

CommittedTrajectory CommittedTrajectory::shiftedToStartTime(double new_start_time) const {
    if (pieces_.empty()) {
        throw std::runtime_error("cannot time-shift an empty committed trajectory");
    }
    if (!std::isfinite(new_start_time)) {
        throw std::invalid_argument("new committed-trajectory start time must be finite");
    }
    const double delta = new_start_time - startTime();
    CommittedTrajectory shifted = *this;
    for (auto& piece : shifted.pieces_) {
        piece.valid_from += delta;
        piece.valid_until += delta;
        for (double& knot : piece.knots) {
            knot += delta;
        }
        piece.validate();
    }
    return shifted;
}

SpliceDiagnostics CommittedTrajectory::replaceSuffix(
    double splice_time,
    const TrajectoryPiece& candidate,
    double continuity_tolerance) {
    candidate.validate();
    if (!std::isfinite(splice_time) || !(continuity_tolerance >= 0.0) ||
        std::abs(candidate.valid_from - splice_time) > kTrajectoryTimeToleranceS) {
        throw std::invalid_argument("invalid splice time/candidate start/tolerance");
    }

    SpliceDiagnostics diagnostics;
    if (pieces_.empty()) {
        pieces_.push_back(candidate);
        diagnostics.accepted = true;
        return diagnostics;
    }
    if (splice_time < startTime() - kTrajectoryTimeToleranceS) {
        throw std::out_of_range("splice time lies before committed trajectory");
    }
    const double old_explicit_end = endTime();
    const bool splice_in_terminal_hold = splice_time > old_explicit_end + kTrajectoryTimeToleranceS;
    if (splice_in_terminal_hold && !endsInStoppedHold(continuity_tolerance)) {
        throw std::out_of_range(
            "splice time lies after a committed trajectory that is not safely stopped");
    }

    const State old_state = evaluate(splice_time);
    const State new_state = evaluateCubicState(candidate.control_points, candidate.knots, splice_time);
    diagnostics.position_error = (old_state.position - new_state.position).norm();
    diagnostics.velocity_error = (old_state.velocity - new_state.velocity).norm();
    diagnostics.acceleration_error = (old_state.acceleration - new_state.acceleration).norm();
    if (diagnostics.position_error > continuity_tolerance ||
        diagnostics.velocity_error > continuity_tolerance ||
        diagnostics.acceleration_error > continuity_tolerance) {
        return diagnostics;  // atomic rejection: pieces_ is untouched
    }

    std::vector<TrajectoryPiece> updated;
    updated.reserve(pieces_.size() + 2U);

    if (splice_in_terminal_hold) {
        // Preserve every old spline exactly, then materialize only the finite
        // hold interval that will actually execute before the new candidate.
        updated = pieces_;
        const State endpoint = exactStationaryHoldState(endState());
        updated.push_back(stationaryHoldPiece(endpoint, old_explicit_end, splice_time));
    } else {
        for (const auto& piece : pieces_) {
            if (piece.valid_from >= splice_time - kTrajectoryTimeToleranceS) {
                break;
            }
            TrajectoryPiece kept = piece;
            if (kept.valid_until > splice_time) {
                kept.valid_until = splice_time;
            }
            if (kept.valid_until > kept.valid_from + 1e-12) {
                updated.push_back(std::move(kept));
            }
            if (piece.valid_until >= splice_time - kTrajectoryTimeToleranceS) {
                break;
            }
        }
    }

    updated.push_back(candidate);
    pieces_ = std::move(updated);
    diagnostics.accepted = true;
    return diagnostics;
}

SpliceDiagnostics CommittedTrajectory::appendVelocityContinuousBackup(
    const TrajectoryPiece& backup,
    double continuity_tolerance) {
    backup.validate();
    if (pieces_.empty() || !(continuity_tolerance >= 0.0) ||
        !std::isfinite(continuity_tolerance) ||
        std::abs(backup.valid_from - endTime()) > kTrajectoryTimeToleranceS) {
        throw std::invalid_argument("invalid velocity-continuous backup append request");
    }

    const State old_state = endState();
    const State new_state = evaluateCubicState(
        backup.control_points, backup.knots, backup.valid_from);
    SpliceDiagnostics diagnostics;
    diagnostics.position_error = (old_state.position - new_state.position).norm();
    diagnostics.velocity_error = (old_state.velocity - new_state.velocity).norm();
    diagnostics.acceleration_error = (old_state.acceleration - new_state.acceleration).norm();
    if (diagnostics.position_error > continuity_tolerance ||
        diagnostics.velocity_error > continuity_tolerance) {
        return diagnostics;
    }

    pieces_.push_back(backup);
    diagnostics.accepted = true;
    return diagnostics;
}


CommittedTrajectory makeStationaryHoverTrajectory(
    const Vec3& position,
    double start_time_s,
    double seed_duration_s) {
    if (!position.allFinite()) {
        throw std::invalid_argument("stationary-hover position must be finite");
    }
    if (!std::isfinite(start_time_s)) {
        throw std::invalid_argument("stationary-hover start time must be finite");
    }
    if (!std::isfinite(seed_duration_s) || !(seed_duration_s > 0.0)) {
        throw std::invalid_argument("stationary-hover seed duration must be positive");
    }

    TrajectoryPiece hold;
    hold.valid_from = start_time_s;
    hold.valid_until = start_time_s + seed_duration_s;
    hold.knots = openUniformKnots(hold.valid_from, hold.valid_until, 1);
    hold.control_points = ControlPoints::Zero(4, 3);
    for (Eigen::Index row = 0; row < hold.control_points.rows(); ++row) {
        hold.control_points.row(row) = position.transpose();
    }

    CommittedTrajectory committed;
    const SpliceDiagnostics diagnostics = committed.replaceSuffix(
        start_time_s, hold, 1e-12);
    if (!diagnostics.accepted || !committed.endsInStoppedHold()) {
        throw std::runtime_error("failed to initialise stationary committed hover");
    }
    return committed;
}

CommittedTrajectory makeBrakingHoverTrajectory(
    const State& measured_state,
    double start_time_s,
    const Vec3& brake_accel_limit,
    double minimum_duration_s) {
    if (!measured_state.position.allFinite() || !measured_state.velocity.allFinite() ||
        !std::isfinite(start_time_s) || !brake_accel_limit.allFinite() ||
        (brake_accel_limit.array() <= 0.0).any() ||
        !std::isfinite(minimum_duration_s) || !(minimum_duration_s > 0.0)) {
        throw std::invalid_argument("invalid braking-hover request");
    }

    double duration_s = minimum_duration_s;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        // With q1=q2=q3, a clamped cubic has
        //   v(0)=3(q1-q0)/T, a(0)=-2 v(0)/T, v(T)=a(T)=0.
        // Choosing T >= 2|v0|/a_limit therefore bounds every acceleration
        // component while preserving position/velocity continuity at sample 0.
        duration_s = std::max(
            duration_s,
            2.0 * std::abs(measured_state.velocity(axis)) / brake_accel_limit(axis));
    }

    const Vec3 stop_position =
        measured_state.position + measured_state.velocity * (duration_s / 3.0);

    TrajectoryPiece brake;
    brake.valid_from = start_time_s;
    brake.valid_until = start_time_s + duration_s;
    brake.knots = openUniformKnots(brake.valid_from, brake.valid_until, 1);
    brake.control_points = ControlPoints::Zero(4, 3);
    brake.control_points.row(0) = measured_state.position.transpose();
    brake.control_points.row(1) = stop_position.transpose();
    brake.control_points.row(2) = stop_position.transpose();
    brake.control_points.row(3) = stop_position.transpose();

    CommittedTrajectory committed;
    const SpliceDiagnostics diagnostics = committed.replaceSuffix(
        start_time_s, brake, 1e-10);
    if (!diagnostics.accepted || !committed.endsInStoppedHold()) {
        throw std::runtime_error("failed to initialise braking committed hover");
    }
    return committed;
}


std::vector<State> CommittedTrajectory::sample(double t0, double t1, double dt) const {
    if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(dt) ||
        !(dt > 0.0) || t1 < t0 || t0 < startTime() - kTrajectoryTimeToleranceS) {
        throw std::invalid_argument("invalid committed-trajectory sampling range");
    }
    if (t1 > endTime() + kTrajectoryTimeToleranceS && !endsInStoppedHold()) {
        throw std::invalid_argument(
            "sampling beyond explicit end requires a certified stopped terminal hold");
    }
    std::vector<State> states;
    for (double t = t0; t < t1 - 0.5 * dt; t += dt) {
        states.push_back(evaluate(t));
    }
    states.push_back(evaluate(t1));
    return states;
}

}  // namespace dynamic_planner
