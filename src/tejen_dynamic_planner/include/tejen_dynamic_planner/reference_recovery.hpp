#pragma once

#include "dynamic_planner/committed_trajectory.hpp"
#include <cstdint>
#include <optional>
#include <set>
#include <sstream>
#include <string>

namespace tejen_dynamic_planner {

enum class RecoveryAction { Ignore, Reset, Seed, Grant, Cancel };

struct RecoveryRequest {
    std::string id;
    RecoveryAction action = RecoveryAction::Ignore;
    dynamic_planner::Vec3 position = dynamic_planner::Vec3::Zero();
    std::int64_t stamp_ns = 0;
};

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
    } else return std::nullopt;
    if (input >> extra) return std::nullopt;
    return request;
}

// Transaction identity and immutable common seed only. Mission/replanning
// decisions, collision certification and measured-state guards remain in the node.
struct ReferenceRecoveryContext {
    std::string request_id;
    std::optional<dynamic_planner::Vec3> seed;
    std::int64_t seed_stamp_ns = 0;
    bool grant_requested = false;
    bool cancelled = false;
    std::set<std::string> seen_requests;

    RecoveryAction apply(const RecoveryRequest& request) {
        if (request.action == RecoveryAction::Reset) {
            if (!seen_requests.insert(request.id).second) return RecoveryAction::Ignore;
            request_id = request.id;
            seed.reset();
            seed_stamp_ns = 0;
            grant_requested = false;
            cancelled = false;
            return RecoveryAction::Reset;
        }
        if (request.id != request_id || cancelled) return RecoveryAction::Ignore;
        if (request.action == RecoveryAction::Seed) {
            if (seed && (*seed != request.position || request.stamp_ns < seed_stamp_ns))
                return RecoveryAction::Ignore;
            seed = request.position;
            seed_stamp_ns = request.stamp_ns;
        } else if (request.action == RecoveryAction::Grant) {
            if (!seed || request.stamp_ns < seed_stamp_ns) return RecoveryAction::Ignore;
            grant_requested = true;
            seed_stamp_ns = request.stamp_ns;  // refresh the same commanded hold, not its origin
        } else if (request.action == RecoveryAction::Cancel) {
            seed.reset();
            grant_requested = false;
            cancelled = true;
        }
        return request.action;
    }
};

}  // namespace tejen_dynamic_planner
