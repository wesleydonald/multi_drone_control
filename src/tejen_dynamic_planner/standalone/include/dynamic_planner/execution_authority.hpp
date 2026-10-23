#pragma once

#include "dynamic_planner/execution_timing.hpp"
#include "dynamic_planner/receding_horizon_planner.hpp"

namespace dynamic_planner {

// Native acceptance is not caller installation. Prepare all throwing objects
// before entering this transaction; install must only swap and must not relock
// the world. Epoch belongs to the caller's single ROS authority thread.
// There is deliberately no new wall-runtime ceiling or ongoing expiry trigger.
inline std::string installCertifiedReplacement(
    const WorldSnapshotSource& world, const ReplanResult& result,
    std::uint64_t solve_epoch, const std::uint64_t& current_epoch,
    double available, double now, double request, bool live_execution,
    const std::function<void()>& install,
    const TrajectoryTimeSource& installation_clock = {}, double* installed_at = nullptr) {
    if (solve_epoch != current_epoch) return "obsolete_authority_epoch";
    if (!std::isfinite(available) || !std::isfinite(now) || !std::isfinite(request) ||
        available < request || now < available) return "invalid_timestamp";
    if (!result.accepted || !result.certified_world_version)
        return "core_rejection_incumbent_retained";
    if (result.incumbent_prefix_unsafe || result.status.rfind("UNCERTIFIED_", 0) == 0)
        return "uncertified_execution";
    if (live_execution && !strictlyBeforeSplice(now, result.splice_time_s))
        return "obsolete_splice_discarded";
    std::string decision = "install";
    const bool current = world.runIfVersionCurrent(*result.certified_world_version, [&] {
        if (solve_epoch != current_epoch) { decision = "obsolete_authority_epoch"; return; }
        double actual = now;
        try { if (installation_clock) actual = installation_clock(); }
        catch (...) { decision = "invalid_timestamp"; return; }
        if (!std::isfinite(actual) || actual < now) { decision = "invalid_timestamp"; return; }
        if (live_execution && !strictlyBeforeSplice(actual, result.splice_time_s)) {
            decision = "obsolete_splice_discarded"; return;
        }
        install();
        if (installed_at) *installed_at = actual;
    });
    return current ? decision : "world_version_changed";
}

}  // namespace dynamic_planner
