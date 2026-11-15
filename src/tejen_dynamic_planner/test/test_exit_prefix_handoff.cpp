// C++ exit hand-off, backend half (multi_drone_control 15 Nov): the seedtraj transaction, the moving-prefix
// seed through the real planner core, relative readiness, refresh windows, admission and the exit workspace.
#include "tejen_dynamic_planner/experiment_workspace.hpp"
#include "tejen_dynamic_planner/reference_recovery.hpp"

#include "dynamic_planner/receding_horizon_planner.hpp"
#include "dynamic_planner/world_snapshot.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using dynamic_planner::CommittedTrajectory;
using dynamic_planner::State;
using dynamic_planner::TrajectoryPiece;
using dynamic_planner::Vec3;
using namespace tejen_dynamic_planner;

namespace {

void requireTrue(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected));
}

constexpr double kT0 = 1763200000.0;  // ROS-scale absolute time, as the mission sends it
constexpr double kKnotS = 0.25;
constexpr int kSegments = 20;          // 5 s of coverage

Vec3 ringFollowing(double t) {
    const double s = t - kT0;
    return Vec3(-0.4 + 0.30 * s, 0.6 + 0.20 * std::sin(0.8 * s), 2.9 - 0.02 * s);
}

// One uniform cubic B-spline split into two pieces at a knot, as the mission's fitted pieces are joined C2.
// Control points from `from_index` on are moved by `tail_shift` (a refresh's new pieces).
CommittedTrajectory prefix(double start_s, int from_index = 1 << 20, const Vec3& tail_shift = Vec3::Zero()) {
    const int rows = kSegments + 3;
    TrajectoryPiece piece;
    for (int k = 0; k < rows + 4; ++k) piece.knots.push_back(start_s + (k - 3) * kKnotS);
    piece.control_points.resize(rows, 3);
    for (int i = 0; i < rows; ++i) {
        Vec3 p = ringFollowing(start_s + (i - 1) * kKnotS);
        if (i >= from_index) p += tail_shift;
        piece.control_points.row(i) = p.transpose();
    }
    const double split_s = start_s + 10 * kKnotS;
    CommittedTrajectory trajectory;
    TrajectoryPiece first = piece, second = piece;
    first.valid_from = start_s;
    first.valid_until = split_s;
    second.valid_from = split_s;
    second.valid_until = start_s + kSegments * kKnotS;
    requireTrue(trajectory.replaceSuffix(first.valid_from, first).accepted &&
                trajectory.replaceSuffix(second.valid_from, second).accepted, "prefix fixture not C2");
    return trajectory;
}

void testParse() {
    const std::string id(32, 'a');
    const auto valid = parseRecoveryRequest(id + " seedtraj exit 3 1763200000123456789 1763200007000000000");
    requireTrue(valid && valid->action == RecoveryAction::SeedTrajectory && valid->exit_context &&
                valid->prefix_sequence == 3U && valid->stamp_ns == 1763200000123456789LL &&
                valid->deadline_ns == 1763200007000000000LL, "valid seedtraj not parsed exactly");
    for (const std::string bad : {
             " seedtraj", " seedtraj exit", " seedtraj exit 1 2", " seedtraj enter 1 2 3",
             " seedtraj EXIT 1 2 3", " seedtraj exit 0 2 3", " seedtraj exit -1 2 3",
             " seedtraj exit 1 -2 3", " seedtraj exit 1 2 0", " seedtraj exit 1 2.5 3",
             " seedtraj exit 1 2e9 3", " seedtraj exit 1 2 3 4", " seedtraj exit x 2 3",
             " seedtraj exit 1 9223372036854775808 3", " seedtraj exit 1 2 99999999999999999999"}) {
        requireTrue(!parseRecoveryRequest(id + bad).has_value(), "malformed seedtraj accepted:" + bad);
    }
    requireTrue(!parseRecoveryRequest(std::string(31, 'a') + " seedtraj exit 1 2 3"), "short id accepted");
    // The existing actions parse exactly as before.
    requireTrue(parseRecoveryRequest(id + " seed 0.4 -0.3 0.9 1000000000")->action == RecoveryAction::Seed &&
                parseRecoveryRequest(id + " grant 10")->action == RecoveryAction::Grant &&
                !parseRecoveryRequest(id + " seed nan 0 1 1000000000"), "existing actions changed");
}

void testTransactionContext() {
    const std::string id(32, 'b');
    const auto request = [&](const std::string& text) { return *parseRecoveryRequest(id + " " + text); };
    ReferenceRecoveryContext context;
    requireTrue(context.apply(request("reset")) == RecoveryAction::Reset, "reset");
    requireTrue(!context.hasSeed() && context.seed_kind == SeedKind::Stationary, "fresh transaction has a seed");
    requireTrue(context.apply(request("seedtraj exit 1 1000 9000")) == RecoveryAction::SeedTrajectory &&
                context.seed_kind == SeedKind::Prefix && context.exit_context && context.prefixBound() &&
                context.prefix_sequence == 1U && context.prefix_stamp_ns == 1000 && context.deadline_ns == 9000,
                "first seedtraj not bound");
    requireTrue(!context.grant_requested, "a prefix seed grants authority");
    requireTrue(context.prefixBinding(request("seedtraj exit 1 1000 9000")) == PrefixBinding::Resend &&
                context.apply(request("seedtraj exit 1 1000 9000")) == RecoveryAction::SeedTrajectory &&
                context.prefix_sequence == 1U, "heartbeat resend is not idempotent");
    requireTrue(context.apply(request("seedtraj exit 1 1001 9000")) == RecoveryAction::Ignore,
                "same sequence with another stamp accepted");
    requireTrue(context.apply(request("seedtraj exit 2 2000 9500")) == RecoveryAction::Ignore &&
                context.deadline_ns == 9000, "refresh moved the fixed deadline");
    requireTrue(context.apply(request("seedtraj exit 2 900 9000")) == RecoveryAction::Ignore,
                "refresh with an older stamp accepted");
    requireTrue(context.apply(request("seed 0 0 1 1500")) == RecoveryAction::Ignore && !context.seed,
                "stationary seed mixed into a prefix transaction");
    requireTrue(context.apply(request("grant 999")) == RecoveryAction::Ignore && !context.grant_requested,
                "grant older than the bound prefix accepted");
    requireTrue(context.apply(request("seedtraj exit 2 2000 9000")) == RecoveryAction::SeedTrajectory &&
                context.prefix_sequence == 2U && context.prefix_stamp_ns == 2000 && context.deadline_ns == 9000,
                "refresh not bound");
    requireTrue(context.apply(request("seedtraj exit 1 1000 9000")) == RecoveryAction::Ignore &&
                context.prefix_sequence == 2U, "stale sequence rebound");
    requireTrue(context.apply(request("grant 1999")) == RecoveryAction::Ignore,
                "grant refers to the superseded prefix");
    requireTrue(context.apply(request("grant 2000")) == RecoveryAction::Grant && context.grant_requested &&
                context.prefix_stamp_ns == 2000, "grant on the bound prefix not recorded, or it moved the stamp");
    ReferenceRecoveryContext other = context;
    requireTrue(other.apply(*parseRecoveryRequest(std::string(32, 'c') + " seedtraj exit 3 3000 9000")) ==
                RecoveryAction::Ignore, "foreign transaction bound");
    requireTrue(context.apply(request("cancel")) == RecoveryAction::Cancel && context.cancelled &&
                !context.hasSeed() && !context.grant_requested, "cancel leaves the prefix bound");
    requireTrue(context.apply(request("seedtraj exit 3 3000 9000")) == RecoveryAction::Ignore,
                "late seedtraj undoes the cancel");
    const std::string next(32, 'd');
    requireTrue(context.apply(*parseRecoveryRequest(next + " reset")) == RecoveryAction::Reset &&
                context.seed_kind == SeedKind::Stationary && !context.exit_context && context.deadline_ns == 0 &&
                !context.prefixBound(), "reset with a new id keeps the old exit transaction");
    // A stationary transaction never takes a prefix.
    requireTrue(context.apply(*parseRecoveryRequest(next + " seed 0 0 1 100")) == RecoveryAction::Seed &&
                context.apply(*parseRecoveryRequest(next + " seedtraj exit 1 200 9000")) == RecoveryAction::Ignore,
                "prefix mixed into a stationary transaction");
}

void testRelativeReadiness() {
    const auto reference = prefix(kT0).evaluate(kT0 + 1.0);
    requireTrue(reference.velocity.norm() > 0.25, "fixture is not moving");
    const Vec3 half = Vec3::Constant(0.05);
    // The stationary gate (|v| <= 0.10) refuses this state; relative readiness accepts it.
    requireTrue(reference.velocity.norm() > 0.10, "stationary gate would pass anyway");
    requireTrue(activationKinematicsBlocker(reference, reference, half, 0.10, 0.50, true).empty(),
                "measured == reference not ready");
    State measured = reference;
    measured.position += Vec3(0.049, -0.049, 0.049);
    measured.velocity += Vec3(0.05, 0.05, 0.0);
    measured.acceleration += Vec3(0.3, 0.3, 0.0);
    requireTrue(activationKinematicsBlocker(measured, reference, half, 0.10, 0.50, true).empty(),
                "inside every tolerance but refused");
    measured.position.x() = reference.position.x() + 0.051;
    requireTrue(activationKinematicsBlocker(measured, reference, half, 0.10, 0.50, true) == "position",
                "position outside the tracking half-extent accepted");
    measured = reference;
    measured.velocity += Vec3(0.08, 0.08, 0.0);
    requireTrue(activationKinematicsBlocker(measured, reference, half, 0.10, 0.50, true) == "velocity",
                "relative speed over tolerance accepted");
    requireTrue(activationKinematicsBlocker(measured, reference, half, 0.10, 0.50, false).empty(),
                "motion checked with require_stationary_activation off");
    measured = reference;
    measured.acceleration += Vec3(0.4, 0.4, 0.0);
    requireTrue(activationKinematicsBlocker(measured, reference, half, 0.10, 0.50, true) == "acceleration",
                "relative acceleration over tolerance accepted");
}

void testRefreshWindowAndAdmission() {
    const auto bound = prefix(kT0);
    const double t_r = kT0 + 1.0;
    const double protected_s = 1.5;
    // Refreshed prefix: identical through t_R + Pw (control points whose support starts there or later moved).
    const int first_free = static_cast<int>(std::lround((t_r + protected_s - kT0) / kKnotS)) + 3;
    const auto same_window = prefix(kT0, first_free, Vec3(0.3, -0.2, 0.1));
    requireTrue(prefixesIdenticalOver(bound, same_window, t_r, t_r + protected_s),
                "verbatim protected window judged different");
    requireTrue(!prefixesIdenticalOver(bound, same_window, t_r, t_r + protected_s + 0.3),
                "difference after the window not seen when the window is longer");
    const auto inside_change = prefix(kT0, first_free - 2, Vec3(0.0, 0.0, 2e-9));
    requireTrue(!prefixesIdenticalOver(bound, inside_change, t_r, t_r + protected_s),
                "a 2e-9 m change inside the window passed the 1e-9 test");
    requireTrue(!prefixesIdenticalOver(bound, same_window, t_r, t_r), "an empty window is identical");
    requireTrue(!prefixesIdenticalOver(bound, same_window, kT0 + 4.0, kT0 + 6.0),
                "a window beyond coverage is identical");

    requireTrue(prefixAdmissionRejection(t_r + 0.3, kT0, kT0 + 5.0, kT0 + 7.0).empty(), "valid splice dropped");
    requireTrue(prefixAdmissionRejection(kT0 - 0.01, kT0, kT0 + 5.0, kT0 + 7.0) ==
                "exit_splice_outside_prefix_coverage", "splice before coverage installed");
    requireTrue(prefixAdmissionRejection(kT0 + 5.01, kT0, kT0 + 5.0, kT0 + 7.0) ==
                "exit_splice_outside_prefix_coverage", "splice after coverage installed");
    requireTrue(prefixAdmissionRejection(kT0 + 4.0, kT0, kT0 + 5.0, kT0 + 4.0) == "exit_splice_after_deadline",
                "splice at the deadline installed");
    requireTrue(prefixCoverageSufficient(bound, kT0 + 2.69, 0.3) &&
                !prefixCoverageSufficient(bound, kT0 + 2.71, 0.3), "coverage end >= splice + 2 s not applied");
}

// The core seeded with a moving prefix: the committed path is the prefix before the splice (absolute time
// kept), the splice is at now + lookahead on the prefix, and p/v/a are continuous there.
void testPrefixSeedThroughCore() {
    const auto incumbent = prefix(kT0);
    dynamic_planner::RecedingHorizonConfig config;
    config.fixed_splice_lookahead_s = 0.30;
    config.continuity_tolerance = 1e-7;
    dynamic_planner::LocalPlannerConfig local;
    local.num_segments = 4;
    local.octopus.samples_per_axis = {7, 7, 7};
    local.octopus.max_runtime_s = 2.0;
    local.octopus.xyz_min = Vec3(-2.0, -1.0, 2.0);
    local.octopus.xyz_max = Vec3(2.0, 3.0, 3.2);
    local.refinement.xyz_min = local.octopus.xyz_min;
    local.refinement.xyz_max = local.octopus.xyz_max;
    local.refinement.j_max = Vec3::Constant(4.0);
    const Vec3 goal(0.0, 1.55, 2.5);  // AT_START over the reserved mat
    dynamic_planner::RecedingHorizonPlanner planner(goal, config, local);
    planner.initializeCommittedTrajectory(incumbent);
    dynamic_planner::VersionedWorld world;
    const double now_s = kT0 + 0.5;
    const State measured = incumbent.evaluate(now_s);
    const auto result = planner.replan(now_s, measured, world, [now_s]() { return now_s + 0.01; });
    requireTrue(result.accepted, "free-space replan from the moving prefix failed: " + result.status);
    requireNear(result.splice_time_s, now_s + 0.30, 1e-6, "splice not at now + lookahead in absolute time");
    const auto& committed = planner.committedTrajectory();
    requireNear(committed.startTime(), kT0, 1e-9, "the prefix clock was shifted");
    for (double t = now_s; t < result.splice_time_s - 1e-6; t += 0.01) {
        const State a = committed.evaluate(t), b = incumbent.evaluate(t);
        requireTrue((a.position - b.position).norm() <= 1e-12 && (a.velocity - b.velocity).norm() <= 1e-12 &&
                    (a.acceleration - b.acceleration).norm() <= 1e-12,
                    "committed path differs from the prefix before the splice");
    }
    const State joined = committed.evaluate(result.splice_time_s);
    const State expected = incumbent.evaluate(result.splice_time_s);
    requireTrue((joined.position - expected.position).norm() <= 1e-7 &&
                (joined.velocity - expected.velocity).norm() <= 1e-7 &&
                (joined.acceleration - expected.acceleration).norm() <= 1e-7,
                "p/v/a discontinuous at the splice");
    requireTrue(expected.velocity.norm() > 0.25, "splice state was not moving (a stationary seed?)");
}

// v3 section 3: the exit workspace and the centre bounds his erosion leaves (5 deg unloaded, 1.034 m cable).
void testExitWorkspaceErosion() {
    dynamic_planner::SuspendedGeometry geometry;
    geometry.enabled = true;
    geometry.cable_length_m = 1.034;
    geometry.cable_radius_m = 0.0025;
    geometry.max_swing_angle_rad = 5.0 * std::acos(-1.0) / 180.0;
    geometry.magnet_half_extents = Vec3(0.05, 0.05, 0.025);
    geometry.magnet_center_below_cable_end_m = 0.025;
    geometry.payload_attached = false;
    for (const auto envelope : {dynamic_planner::SwingEnvelope::LegacySquare,
                                dynamic_planner::SwingEnvelope::OuterOctagon}) {
        geometry.swing_envelope = envelope;
        const auto bounds = assemblyWorkspaceBounds(Vec3(-1.8, -1.8, 0.0), Vec3(1.8, 1.8, 3.26),
            Vec3(0.105, 0.105, 0.06), geometry, Vec3::Constant(0.05));
        std::cout << "exit centre bounds (" << (envelope == dynamic_planner::SwingEnvelope::OuterOctagon
                         ? "outer_octagon" : "legacy_square") << "): x/y [" << bounds.first.x() << ", "
                  << bounds.second.x() << "] / [" << bounds.first.y() << ", " << bounds.second.y()
                  << "] z [" << bounds.first.z() << ", " << bounds.second.z() << "]\n";
        requireNear(bounds.second.x(), 1.61, 0.005, "x centre bound differs from v3's 1.61 m");
        requireNear(bounds.second.y(), 1.61, 0.005, "y centre bound differs from v3's 1.61 m");
        requireNear(bounds.second.z(), 3.15, 1e-9, "z top differs from v3's 3.15 m");
        const Vec3 mat(0.0, 1.55, 2.5);
        requireTrue((mat.array() > bounds.first.array()).all() && (mat.array() < bounds.second.array()).all(),
                    "AT_START over the reserved mat lies outside the exit centre bounds");
        requireTrue(bounds.second.y() + 0.05 < 1.7, "body (centre + tracking) can reach |y| 1.7 m");
    }
}

}  // namespace

int main() {
    try {
        testParse();
        testTransactionContext();
        testRelativeReadiness();
        testRefreshWindowAndAdmission();
        testPrefixSeedThroughCore();
        testExitWorkspaceErosion();
        std::cout << "test_exit_prefix_handoff: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_exit_prefix_handoff: FAIL: " << error.what() << '\n';
        return 1;
    }
}
