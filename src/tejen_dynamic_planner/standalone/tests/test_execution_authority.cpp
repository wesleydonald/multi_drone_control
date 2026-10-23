#include "dynamic_planner/execution_authority.hpp"
#include "test_common.hpp"

using namespace dynamic_planner;
class AdmissionWorld final : public WorldSnapshotSource {
public:
    std::uint64_t version = 17;
    std::function<void()> before_install;
    WorldSnapshot snapshot(double t) const override { WorldSnapshot s; s.version = version; s.captured_at_s = t; return s; }
    std::uint64_t currentVersion() const override { return version; }
    bool runIfVersionCurrent(std::uint64_t expected, const std::function<void()>& action) const override {
        if (version != expected) return false;
        if (before_install) before_install();
        action(); return true;
    }
};
int main() {
    AdmissionWorld world;
    ReplanResult candidate;
    candidate.accepted = true;
    candidate.certified_world_version = 17;
    candidate.splice_time_s = .25;
    std::uint64_t epoch = 7;
    int installations = 0;
    auto admit = [&](double now, bool live = true) {
        return installCertifiedReplacement(world, candidate, 7, epoch, .09, now, 0., live,
            [&] { ++installations; });
    };
    requireTrue(admit(.1) == "install" && installations == 1, "fresh native candidate not installed");
    requireTrue(admit(.3) == "obsolete_splice_discarded" && installations == 1, "expired splice installed");
    candidate.splice_time_s = .5;
    requireTrue(admit(.3) == "install", "unapproved 200 ms ceiling introduced");
    epoch = 8;
    requireTrue(admit(.1) == "obsolete_authority_epoch", "cancelled worker installed");
    epoch = 7; world.version = 18;
    requireTrue(admit(.1) == "world_version_changed", "new world bypassed certification");
    world.version = 17;
    requireTrue(installCertifiedReplacement(world, candidate, 7, epoch, .09, .1, 0., true,
        [&] { ++installations; }, [] { return .6; }) == "obsolete_splice_discarded", "lock-wait clock not rechecked");
    world.before_install = [&] { epoch = 8; };
    requireTrue(admit(.1) == "obsolete_authority_epoch", "atomic epoch not rechecked");
    world.before_install = {}; epoch = 7;
    requireTrue(admit(.6, false) == "install", "frozen preparation semantics changed");
    candidate.accepted = false;
    requireTrue(admit(.1) == "core_rejection_incumbent_retained", "native rejection installed");
    candidate.accepted = true; candidate.certified_world_version.reset();
    requireTrue(admit(.1) == "core_rejection_incumbent_retained", "uncertified candidate installed");

    State state; state.position = Vec3(0,0,1.5);
    RecedingHorizonConfig config; config.fixed_splice_lookahead_s = .1;
    LocalPlannerConfig local;
    RecedingHorizonPlanner retained(Vec3(3,0,1.5), config, local);
    retained.initializeCommittedTrajectory(makeStationaryHoverTrajectory(state.position, 0., .1));
    const auto before = retained.committedTrajectory();
    retained.setPreviousSpliceTiming(.09);
    requireNear(retained.previousSpliceTiming(), .09, 1e-12, "rejected-solve history lost");
    requireMatrixNear(retained.committedTrajectory().endState().position, before.endState().position, 0., "timing changed authority");
    bool bad_history = false;
    try { retained.setPreviousSpliceTiming(NAN); } catch (const std::invalid_argument&) { bad_history = true; }
    requireTrue(bad_history, "invalid timing history accepted");

    // Exercise a real native solve, not just admission bookkeeping. A discarded
    // shadow's new commitment and goal latch must not leak into the live planner.
    local.octopus.xyz_min = Vec3(-1., -2., .4);
    local.octopus.xyz_max = Vec3(4., 2., 2.6);
    local.refinement.xyz_min = local.octopus.xyz_min;
    local.refinement.xyz_max = local.octopus.xyz_max;
    local.refinement.j_max = Vec3::Constant(4.);
    local.refinement.max_working_set_recalculations = 500;
    config.factor_alloc = 2.5;
    // Synthetic goal-latch fixture only; no commissioned tolerance changes.
    config.goal_tolerance_m = .2;
    RecedingHorizonPlanner live(Vec3(1.,0.,1.5), config, local);
    live.initializeCommittedTrajectory(before);
    auto shadow = live;
    VersionedWorld clear;
    const auto solved = shadow.replan(0., state, clear, [] { return .02; });
    requireTrue(solved.accepted && solved.certified_world_version.has_value(), "native fixture failed: " + solved.status);
    requireTrue(shadow.goalSeen() && !live.goalSeen(), "goal-latch fixture not discriminating: " + solved.status +
        " distance=" + std::to_string(solved.committed_endpoint_distance_m));
    auto executable = live.committedTrajectory();
    const auto proposed = shadow.committedTrajectory();
    auto install = [&] { live = shadow; executable = proposed; };
    requireTrue(installCertifiedReplacement(clear, solved, 7, epoch, .02, .2, 0., true,
        install) == "obsolete_splice_discarded", "stale real solve installed");
    live.setPreviousSpliceTiming(solved.trajectory_elapsed_s);
    requireTrue(!live.goalSeen(), "discarded goal latch leaked");
    requireMatrixNear(executable.endState().position, state.position, 0., "discarded trajectory leaked");
    requireTrue(installCertifiedReplacement(clear, solved, 7, epoch, .02, .03, 0., true,
        install) == "install", "fresh real solve rejected");
    requireTrue(live.goalSeen(), "installed goal latch missing");
    requireMatrixNear(executable.endState().position, proposed.endState().position, 0., "installed PVA changed");
}
