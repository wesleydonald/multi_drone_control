"""C++ exit hand-off, backend wiring (multi_drone_control 15 Nov): source contracts of the node. Behaviour is in
test_exit_prefix_handoff.cpp (ctest) and exit_prefix_transport_smoke.py (the real binary, explicit run)."""
import re
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / 'src/transfer_backend_node.cpp').read_text()


def body(start, end):
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def block(text, opener):
    """The brace block that starts at the first `opener` (which must end with '{')."""
    start = text.index(opener) + len(opener)
    depth = 1
    for i in range(start, len(text)):
        depth += {'{': 1, '}': -1}.get(text[i], 0)
        if depth == 0:
            return text[start:i]
    raise AssertionError('unbalanced block ' + opener)


def test_parameters_default_off_or_neutral():
    for declaration in ('declare_parameter<std::string>("moving_prefix_topic", "")',
                        'declare_parameter<double>("exit_max_swing_angle_rad", 0.0)',
                        '"exit_workspace_min", std::vector<double>{}',
                        '"exit_workspace_max", std::vector<double>{}',
                        'declare_parameter<double>("exit_max_search_z_m", 0.0)',
                        'declare_parameter<double>("exit_protected_s", 1.5)'):
        assert declaration in SOURCE
    assert 'bool exitFeatureEnabled() const { return !moving_prefix_topic_.empty(); }' in SOURCE


def test_feature_off_ignores_seedtraj_exactly_like_an_unparsed_request():
    callback = body('    void recoveryRequestCallback(', '    bool recoverySeedReady(')
    off = 'if (request->action == RecoveryAction::SeedTrajectory && !exitFeatureEnabled()) return;'
    assert callback.index('if (!request) return;') < callback.index(off) < callback.index('publishRecoveryStatus();')
    # The prefix subscription and the new status keys exist only when the feature is on.
    constructor = SOURCE.split('    ~TransferBackendNode() override', 1)[0]
    assert 'moving_prefix_sub_ = create_subscription' in block(constructor, 'if (exitFeatureEnabled()) {')
    status = body('    void publishRecoveryStatus(', '    void recoveryRequestCallback(')
    keys = block(status, 'if (exitFeatureEnabled()) {')
    for key in ('seed_kind', 'exit_context', 'prefix_sequence', 'prefix_coverage_end_ns', 'deadline_ns',
                'splice_time_ns', 'exit_reject_reason', 'candidates_attempted', 'candidates_accepted',
                'candidates_late', 'candidates_invalidated'):
        assert '",\\"' + key + '\\":"' in keys, key
    diagnostics = body('    void diagnosticsTimer()', '    KinematicsFilter kinematics_;')
    assert 'exit context/swing_deg/seq' in block(diagnostics, 'if (exitFeatureEnabled()) {')


def test_every_exit_branch_is_reachable_only_from_an_accepted_seedtraj():
    # The context is installed in one place, reached only from the seedtraj handler; the handler is called only
    # after the feature-off return and from the prefix subscription (which exists only when the feature is on).
    assert len(re.findall(r'\binstallExitContext\(\);', SOURCE)) == 1
    assert 'if (!exitContextActive()) installExitContext();' in body('    void handleSeedTrajectory(',
                                                                   '    void rejectExit(')
    calls = re.findall(r'\bhandleSeedTrajectory\((\*request|pending)\);', SOURCE)
    assert sorted(calls) == ['*request', 'pending']
    assert ('return exitFeatureEnabled() && recovery_.prefixBound() && exit_prefix_.has_value();'
            in SOURCE)
    assert 'bool exitContextActive() const { return exit_loaded_geometry_.has_value(); }' in SOURCE
    assert 'exit_loaded_geometry_ = ' in body('    void installExitContext()', '    void endExitContext()')


def test_payload_attached_checks_skipped_for_the_exit_context_only():
    raw = 'payload_collision_enabled_ && require_payload_attached_for_enable_'
    assert SOURCE.count(raw) == 1
    assert (raw + ' && !exitContextActive()') in SOURCE
    for start, end in (('    bool tryActivate()', '    void pollPlannerFuture()'),
                       ('    void authorityCallback(', '    void disableBackend('),
                       ('    void replanTimer()', '    void referenceTimer()')):
        assert 'if (payloadAttachedChecksApply()) {' in body(start, end)
    attached = body('    void objectAttachedCallback(', '    void cooperativeStateCallback(')
    assert attached.index('if (exitContextActive()) return;') < attached.index('PAYLOAD_DETACHED_DISABLED')
    assert attached.index('last_object_attached_receive_ros_s_ = now_s;') < attached.index(
        'if (exitContextActive()) return;')


def test_relative_readiness_replaces_both_stationary_gates_for_the_prefix_only():
    for start, end, status in (
            ('    bool tryActivate()', '    void pollPlannerFuture()', 'WAITING_STATIONARY_ACTIVATION'),
            ('    void authorityCallback(', '    void disableBackend(', 'AUTHORITY_GRANT_REJECTED_NOT_STATIONARY')):
        text = body(start, end)
        prefix = text.index('if (prefix_seed) {')
        assert prefix < text.index('activationKinematicsReady(now_s, measured,')
        assert text.index('activationKinematicsReady(now_s, measured,') < text.index(
            '} else if (require_stationary_activation_ &&') < text.index(status)
    ready = body('    bool activationKinematicsReady(', '    void movingPrefixCallback(')
    assert 'exit_prefix_->evaluate(now_s)' in ready
    for bound in ('ego_tracking_error_half_extents_', 'activation_speed_tolerance_mps_',
                  'activation_accel_tolerance_mps2_', 'require_stationary_activation_'):
        assert bound in ready


def test_prefix_seed_never_stationary_and_never_clock_shifted():
    activation = body('    bool tryActivate()', '    void pollPlannerFuture()')
    seed = block(activation[activation.index('const bool extended_pre_authority_seed'):], 'if (prefix_seed) {')
    assert 'planner_->initializeCommittedTrajectory(*exit_prefix_);' in seed
    assert 'shadow_commit_ = *exit_prefix_;' in seed
    authority = body('    void authorityCallback(', '    void disableBackend(')
    fresh = authority[authority.index('auto fresh_planner = std::make_unique<RecedingHorizonPlanner>'):]
    grant_seed = block(fresh, 'if (prefix_seed) {')
    assert 'fresh_planner->initializeCommittedTrajectory(*exit_prefix_);' in grant_seed
    assert 'shadow_commit_ = *exit_prefix_;' in grant_seed
    rebind = body('    void rebindExitPlanner()', '    void installExitContext()')
    assert 'planner->initializeCommittedTrajectory(*exit_prefix_);' in rebind
    handler = body('    void handleSeedTrajectory(', '    void installExitContext()')
    for text in (seed, grant_seed, rebind, handler):
        assert 'makeStationaryHoverTrajectory' not in text and 'shiftedToStartTime' not in text
    # The rebased-prepared path stays a non-recovery path; the grant splices on the prefix at now + lookahead.
    assert 'if (reuse_prepared_commit_on_authority_ && recovery_.request_id.empty())' in authority
    assert 'grant_origin = exit_prefix_->evaluate(now_s + lookahead_s).position;' in authority
    origin = block(activation, '} else if (prefix_seed) {')
    assert 'activation_origin = exit_prefix_->evaluate(now_s + lookahead_s).position;' in origin
    # Prefix certification over [now, now + lookahead] replaces the hold certification.
    certification = block(activation[activation.index('RECOVERY_COMMON_HOLD_NOT_CERTIFIED:'):],
                          '} else if (prefix_seed) {')
    assert 'checker.checkCommitted(*exit_prefix_, now_s,' in certification
    assert 'now_s + activation_planner->chooseSpliceLookahead()' in certification


def test_context_opens_before_activation_on_validated_evidence():
    handler = body('    void handleSeedTrajectory(', '    void rejectExit(')
    new_binding = handler[handler.index('const auto received = std::find_if('):]
    order = ['exitBindingBlocker(*received', 'keepCandidatesAcrossRefresh(prefix',
             'recovery_.apply(request);', 'installExitContext();', 'tryActivate();']
    assert [new_binding.index(step) for step in order] == sorted(new_binding.index(step) for step in order)
    blocker = body('    std::string exitBindingBlocker(', '    void handleSeedTrajectory(')
    for reason in ('deadline_passed', 'prefix_invalid', 'prefix_coverage_short', 'object_attached_or_stale'):
        assert '"' + reason + '"' in blocker
    assert 'currentObjectAttached(now_s, &attached, &age_s) || attached' in blocker
    assert '"prefix_missing"' in handler
    # A heartbeat resend never opens the context or raises a reject reason.
    resend = block(handler, 'if (binding == PrefixBinding::Resend) {')
    assert 'installExitContext' not in resend and 'rejectExit' not in resend
    prefix = body('    void movingPrefixCallback(', '    std::string exitBindingBlocker(')
    for check in ('msg->vehicle_id != vehicle_id_', 'msg->header.frame_id != frame_id_',
                  'msg->terminal_hold', 'trajectoryFromMessage(*msg)'):
        assert check in prefix


def test_every_cleanup_path_restores_the_loaded_context():
    disable = body('    void disableBackend(', '    void publishRecoveryStatus(')
    assert 'endExitContext();' in disable
    end = body('    void endExitContext()', '    void resetExitTransaction()')
    assert 'world_->setEgoSuspendedGeometry(suspended_geometry_);' in end
    assert 'exit_loaded_geometry_.reset();' in end
    callback = body('    void recoveryRequestCallback(', '    bool recoverySeedReady(')
    assert 'resetExitTransaction();\n            disableBackend("RECOVERY_WAITING_COMMON_STOPPED_REFERENCE");' in callback
    assert 'disableBackend("RECOVERY_CANCELLED");' in callback
    enable = body('    void enableCallback(', '    void authorityCallback(')
    assert 'disableBackend("DISABLED_BY_COMMAND");' in enable
    deadline = body('    bool checkExitDeadline()', '    std::string exitPrefixAdmissionRejection(')
    assert '"deadline_passed"' in deadline and 'disableBackend("EXIT_DEADLINE_PASSED");' in deadline
    assert '!exitContextActive() || authority_granted_' in deadline
    for start, end in (('    void referenceTimer()', '    void publishMarkers()'),
                       ('    void replanTimer()', '    void referenceTimer()'),
                       ('    void diagnosticsTimer()', '    KinematicsFilter kinematics_;'),
                       ('    void authorityCallback(', '    void disableBackend(')):
        assert 'checkExitDeadline()' in body(start, end)
    reference = body('    void referenceTimer()', '    void publishMarkers()')
    assert reference.index('checkExitDeadline();') < reference.index('if (!enabled_ || !shadow_commit_.has_value())')


def test_admission_precedes_installation_and_refresh_invalidates_before_rebinding():
    poll = body('    void pollPlannerFuture()', '    void replanTimer()')
    admitted = poll.index('outcome.installation_rejection = exitPrefixAdmissionRejection(outcome);')
    assert admitted < poll.index('if (outcome.installation_rejection.empty()) {') < poll.index(
        'installCertifiedReplacement(')
    admission = body('    std::string exitPrefixAdmissionRejection(', '    void countExitOutcome(')
    assert 'prefixAdmissionRejection(splice_s, exit_prefix_->startTime(), exit_prefix_->endTime(),' in admission
    assert 'recovery_.deadline_ns' in admission and '"exit_prefix_refreshed"' in admission
    refresh = body('    bool keepCandidatesAcrossRefresh(', '    void rebindExitPlanner()')
    assert 'prefixesIdenticalOver(*exit_prefix_, refreshed, from_s, until_s)' in refresh
    assert 'refreshed.startTime() + exit_protected_s_' in refresh
    invalidated = refresh[refresh.index('return true;'):]
    assert '++epoch_;' in invalidated and 'prepared_candidate_ready_ = false;' in invalidated
    timer = body('    void replanTimer()', '    void referenceTimer()')
    assert timer.index('if (exit_rebind_pending_) rebindExitPlanner();') < timer.index(
        'if (!planner_ || !active_target_.has_value()) {')
    assert 'outcome.exit_prefix_sequence = exit_prefix_sequence;' in timer
