#pragma once

#include "dynamic_planner/committed_trajectory.hpp"
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>

namespace tejen_dynamic_planner {

enum class RecoveryAction { Ignore, Reset, Seed, Grant, Cancel, SeedTrajectory };
enum class SeedKind { Stationary, Prefix };
enum class PrefixBinding { Ignore, Resend, New };

struct RecoveryRequest {
    std::string id;
    RecoveryAction action = RecoveryAction::Ignore;
    dynamic_planner::Vec3 position = dynamic_planner::Vec3::Zero();
    std::int64_t stamp_ns = 0;
    // `<id> seedtraj exit <sequence> <stamp_ns> <deadline_ns>`: stamp_ns is the bound prefix's header stamp.
    bool exit_context = false;
    std::uint64_t prefix_sequence = 0U;
    std::int64_t deadline_ns = 0;
};

namespace detail {
// Digits only: istream extraction would wrap "-1" into a huge unsigned value.
inline bool parsePositiveDecimal(std::istream& input, std::uint64_t maximum, std::uint64_t* value) {
    std::string token;
    if (!(input >> token) || token.size() > 19U ||
        token.find_first_not_of("0123456789") != std::string::npos) return false;
    *value = std::stoull(token);
    return *value > 0U && *value <= maximum;
}
}  // namespace detail

inline std::optional<RecoveryRequest> parseRecoveryRequest(const std::string& text) {
    std::istringstream input(text);
    RecoveryRequest request;
    std::string action, extra;
    if (!(input >> request.id >> action) || request.id.size() != 32 ||
        request.id.find_first_not_of("0123456789abcdef") != std::string::npos) return std::nullopt;
    if (action == "reset") request.action = RecoveryAction::Reset;
    else if (action == "grant") {
        request.action = RecoveryAction::Grant;
        if (!(input >> request.stamp_ns) || request.stamp_ns <= 0) return std::nullopt;
    }
    else if (action == "cancel") request.action = RecoveryAction::Cancel;
    else if (action == "seed") {
        request.action = RecoveryAction::Seed;
        if (!(input >> request.position.x() >> request.position.y() >> request.position.z()
                    >> request.stamp_ns) || !request.position.allFinite() || request.stamp_ns <= 0)
            return std::nullopt;
    } else if (action == "seedtraj") {
        constexpr auto kMaxNs = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
        std::string kind;
        std::uint64_t stamp = 0U, deadline = 0U;
        if (!(input >> kind) || kind != "exit" ||
            !detail::parsePositiveDecimal(input, std::numeric_limits<std::uint64_t>::max(),
                                          &request.prefix_sequence) ||
            !detail::parsePositiveDecimal(input, kMaxNs, &stamp) ||
            !detail::parsePositiveDecimal(input, kMaxNs, &deadline)) return std::nullopt;
        request.action = RecoveryAction::SeedTrajectory;
        request.exit_context = true;
        request.stamp_ns = static_cast<std::int64_t>(stamp);
        request.deadline_ns = static_cast<std::int64_t>(deadline);
    } else return std::nullopt;
    if (input >> extra) return std::nullopt;
    return request;
}

// Transaction identity and the immutable common seed or bound moving prefix
// only. Mission/replanning decisions, collision certification and measured-state
// guards remain in the node.
struct ReferenceRecoveryContext {
    std::string request_id;
    std::optional<dynamic_planner::Vec3> seed;
    std::int64_t seed_stamp_ns = 0;
    bool grant_requested = false;
    bool cancelled = false;
    std::set<std::string> seen_requests;
    // seedtraj: the deadline is fixed by the first binding; refreshes only advance
    // the sequence. The node fills the coverage and protected-window ends.
    SeedKind seed_kind = SeedKind::Stationary;
    bool exit_context = false;
    std::uint64_t prefix_sequence = 0U;
    std::int64_t prefix_stamp_ns = 0;
    std::int64_t prefix_coverage_end_ns = 0;
    std::int64_t protected_window_end_ns = 0;
    std::int64_t deadline_ns = 0;

    bool prefixBound() const { return prefix_sequence != 0U; }
    bool hasSeed() const { return seed.has_value() || prefixBound(); }

    PrefixBinding prefixBinding(const RecoveryRequest& request) const {
        if (request.action != RecoveryAction::SeedTrajectory || request.id != request_id ||
            cancelled || seed) return PrefixBinding::Ignore;
        if (!prefixBound()) return PrefixBinding::New;
        if (request.deadline_ns != deadline_ns || request.prefix_sequence < prefix_sequence)
            return PrefixBinding::Ignore;
        if (request.prefix_sequence == prefix_sequence)
            return request.stamp_ns == prefix_stamp_ns ? PrefixBinding::Resend : PrefixBinding::Ignore;
        return request.stamp_ns > prefix_stamp_ns ? PrefixBinding::New : PrefixBinding::Ignore;
    }

    RecoveryAction apply(const RecoveryRequest& request) {
        if (request.action == RecoveryAction::Reset) {
            if (!seen_requests.insert(request.id).second) return RecoveryAction::Ignore;
            request_id = request.id;
            seed.reset();
            seed_stamp_ns = 0;
            grant_requested = false;
            cancelled = false;
            seed_kind = SeedKind::Stationary;
            exit_context = false;
            deadline_ns = 0;
            clearPrefix();
            return RecoveryAction::Reset;
        }
        if (request.action == RecoveryAction::SeedTrajectory) {
            const PrefixBinding binding = prefixBinding(request);
            if (binding == PrefixBinding::Ignore) return RecoveryAction::Ignore;
            if (binding == PrefixBinding::New) {
                seed_kind = SeedKind::Prefix;
                exit_context = request.exit_context;
                prefix_sequence = request.prefix_sequence;
                prefix_stamp_ns = request.stamp_ns;
                deadline_ns = request.deadline_ns;
            }
            return RecoveryAction::SeedTrajectory;
        }
        if (request.id != request_id || cancelled) return RecoveryAction::Ignore;
        if (request.action == RecoveryAction::Seed) {
            if (prefixBound()) return RecoveryAction::Ignore;
            if (seed && (*seed != request.position || request.stamp_ns < seed_stamp_ns))
                return RecoveryAction::Ignore;
            seed = request.position;
            seed_stamp_ns = request.stamp_ns;
        } else if (request.action == RecoveryAction::Grant) {
            if (prefixBound()) {
                // The grant refers to the bound prefix, never to an older sequence.
                if (request.stamp_ns < prefix_stamp_ns) return RecoveryAction::Ignore;
                grant_requested = true;
                return request.action;
            }
            if (!seed || request.stamp_ns < seed_stamp_ns) return RecoveryAction::Ignore;
            grant_requested = true;
            seed_stamp_ns = request.stamp_ns;  // refresh the same commanded hold, not its origin
        } else if (request.action == RecoveryAction::Cancel) {
            seed.reset();
            grant_requested = false;
            cancelled = true;
            clearPrefix();
        }
        return request.action;
    }

private:
    void clearPrefix() {
        prefix_sequence = 0U;
        prefix_stamp_ns = 0;
        prefix_coverage_end_ns = 0;
        protected_window_end_ns = 0;
    }
};

// A prefix seed needs the splice plus this much coverage, at seedtraj acceptance and at the grant.
inline constexpr double kExitPrefixCoverageMarginS = 2.0;

inline bool prefixCoverageSufficient(const dynamic_planner::CommittedTrajectory& prefix,
                                     double now_s, double lookahead_s) {
    return !prefix.empty() && std::isfinite(now_s) && std::isfinite(lookahead_s) &&
        prefix.endTime() >= now_s + lookahead_s + kExitPrefixCoverageMarginS;
}

// Relative readiness: the measured state against the reference at the same instant,
// within the existing tracking half-extents and activation tolerances. '' = ready.
inline std::string activationKinematicsBlocker(
    const dynamic_planner::State& measured, const dynamic_planner::State& reference,
    const dynamic_planner::Vec3& position_half_extents, double speed_tolerance_mps,
    double accel_tolerance_mps2, bool check_motion) {
    if (((measured.position - reference.position).cwiseAbs().array() >
         position_half_extents.array()).any()) return "position";
    if (check_motion && (measured.velocity - reference.velocity).norm() > speed_tolerance_mps)
        return "velocity";
    if (check_motion && (measured.acceleration - reference.acceleration).norm() > accel_tolerance_mps2)
        return "acceleration";
    return "";
}

// Refresh: candidates survive only where the refreshed prefix equals the bound one
// in p, v and a at `samples` uniform times over [from_s, until_s], both ends included.
inline bool prefixesIdenticalOver(const dynamic_planner::CommittedTrajectory& bound,
                                  const dynamic_planner::CommittedTrajectory& refreshed,
                                  double from_s, double until_s,
                                  int samples = 21, double tolerance = 1e-9) {
    if (!std::isfinite(from_s) || !std::isfinite(until_s) || !(until_s > from_s) || samples < 2)
        return false;
    try {
        for (int k = 0; k < samples; ++k) {
            const double t = k + 1 == samples ? until_s
                : from_s + (until_s - from_s) * static_cast<double>(k) / static_cast<double>(samples - 1);
            const auto a = bound.evaluate(t);
            const auto b = refreshed.evaluate(t);
            if ((a.position - b.position).norm() > tolerance ||
                (a.velocity - b.velocity).norm() > tolerance ||
                (a.acceleration - b.acceleration).norm() > tolerance) return false;
        }
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

// Admission of a candidate spliced onto the bound prefix. '' = admit.
inline std::string prefixAdmissionRejection(double splice_s, double coverage_start_s,
                                            double coverage_end_s, double deadline_s) {
    if (!std::isfinite(splice_s) ||
        splice_s < coverage_start_s - dynamic_planner::kTrajectoryTimeToleranceS ||
        splice_s > coverage_end_s + dynamic_planner::kTrajectoryTimeToleranceS)
        return "exit_splice_outside_prefix_coverage";
    if (!(splice_s < deadline_s)) return "exit_splice_after_deadline";
    return "";
}

}  // namespace tejen_dynamic_planner
