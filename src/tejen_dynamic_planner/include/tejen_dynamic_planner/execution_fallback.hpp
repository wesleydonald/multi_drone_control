#pragma once

#include "dynamic_planner/committed_trajectory.hpp"

#include <exception>
#include <string>

namespace tejen_dynamic_planner {

enum class ExecutionFallbackOutcome {
    InstalledCertified,
    InstalledUncertified,
    NotInstalled,
};

struct PreparedExecutionFallback {
    dynamic_planner::CommittedTrajectory trajectory;
    bool certified = false;
    std::string mode;
    std::string error;
};

struct ExecutionFallbackAttempt {
    ExecutionFallbackOutcome outcome = ExecutionFallbackOutcome::NotInstalled;
    std::string mode = "INSTALL_FAILED";
    std::string error;
};

// Keep construction, checking and installation distinct. An unavailable check
// must not discard a valid bounded stop. The installer prepares all throwing
// objects before swapping live state, and performs a non-throwing final commit.
template <typename Build, typename Check, typename Install>
ExecutionFallbackAttempt attemptExecutionFallback(Build build, Check check, Install install) {
    try {
        PreparedExecutionFallback prepared{
            build(), false, "UNCERTIFIED_SMOOTH_BRAKE_CHECK_ERROR", ""};
        try {
            prepared = check(prepared.trajectory);
        } catch (const std::exception& exc) {
            prepared.error = exc.what();
        }
        // Prepare the return value before installation: allocation failures
        // cannot label an already-swapped live commitment as NotInstalled.
        ExecutionFallbackAttempt result{
            prepared.certified ? ExecutionFallbackOutcome::InstalledCertified
                               : ExecutionFallbackOutcome::InstalledUncertified,
            prepared.mode, prepared.error};
        install(prepared);
        return result;
    } catch (const std::exception& exc) {
        return {ExecutionFallbackOutcome::NotInstalled, "INSTALL_FAILED", exc.what()};
    }
}

}  // namespace tejen_dynamic_planner
