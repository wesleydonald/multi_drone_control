# C++ exit — implementation handoff, 15 November 2026

For Wesley and Claude, from Te-Jen and Codex. Send with
`plan_cpp_exit_handoff_2026-11-15_v3.md`; v2 still supplies details not replaced by
v3. This supersedes the previous review-only/no-code instruction for the scoped
implementation and offline validation below. **It does not authorise rig flights.**

## Direction

Proceed with v3 in **Wesley's current integrated fork**, after checkpointing the
working state. Its moving-prefix, continuity and transaction machinery is
accepted as a reasonable implementation direction, not rejected as “slop.”
Judge it by demonstrated operation and regression containment, not patch size.
Do not start another broad redesign or remove necessary mechanisms just to save
lines. No planner-algorithm changes, MPC tuning or unrelated cleanup.

The objective is to replace the Python post-drop exit reference with the existing
C++ planner, then use the existing LAND sequence at a reserved destination.
The same MPC remains the flight controller. Keep the new exit opt-in and retain
the old frozen paper configuration separately and unchanged.

## Settled settings and scope

- General body-origin height limit **3.2 m** in the new trial configuration,
  including the approach; do not command that height unnecessarily.
- Exit trial swing assumption **5 degrees**, not a universal containment claim.
- Start with exit hold **2.9 m**, blend **1 s**, refresh **1 s**, prefix horizon
  **5 s**, protected window **1.5 s**, AT_START dwell **2 s**. Validate these;
  do not silently tune/promote replacements.
- No independent no-overflight rule. Ordinary 3D collision checks remain.
- Use the proposed **exit-only XY assembly workspace +/-1.80 m**, with the
  general height policy resolved consistently. Verify the actual erosion and
  resulting centre bounds, including the reserved mat at **(0,+1.55)**. Do not
  double-subtract margins. Preserve unrelated loaded-leg geometry/checks.
- Reserve the ego mat; ego lands before fleet LAND except emergency intervention.
- Initial handoff timeout **7 s**, not a limit on the complete return.

## Required implementation contracts

1. **One exact timed reference.** Python continues ring-following motion during
   preparation. Relative-motion readiness does not demand zero world velocity.
   Both sides evaluate the same fitted cubic coefficients at the same absolute
   times. Initial control points encode the correct P/V/A; activation/grant must
   not replace the prefix with a stationary seed or shift the curve's clock.

2. **Refresh without invalidating hidden assumptions.** Preserve the protected
   curve exactly or invalidate dependent candidates/worker results before
   rebinding. Admit candidates only inside actual shared coverage and deadlines.
   A p95 latency is not a worst-case bound; 1.5 s is a trial window, not a timing
   guarantee. Check the actual fitted curves' continuity and dynamic bounds.

3. **Unloaded context before activation.** Bind it to the validated post-drop
   transaction during preparation, not only after grant. Install its geometry
   and bounds before certification; execution authority remains separate. Restore
   normal context on cancellation/exit. A static goal alone grants no exception.

4. **No obstructive new detachment qualification.** Reuse existing release and
   fresh attachment-state evidence, with no new proof manoeuvre, speed/position
   gate or dwell. Show that normal release introduces no extra waiting phase.
   Once the existing clearance stage has completed and exit preparation is
   requested, start the 7 s budget **before** new consistency checks or the first
   successful prefix fit. Fits, rejections and refreshes must not reset it.
   Inconsistent/stale evidence is reported within that bounded failure policy;
   do not silently fabricate detachment or weaken the loaded approach.

5. **Timeout and manual authority.** Timeout reports failure and cancels back to
   continuous live ring-following; no silent Python return or world-fixed stop.
   Manual LAND cancels C++ and explicitly invokes the manual Python escape/
   return-and-land. Use real Cancel/disable semantics, local selection first,
   stale-result/cache invalidation and no acknowledgement wait. DISARM is unchanged.
   The direct recovery remains manually authorised, not collision-certified.

6. **Correct takeover timestamp.** Evaluate the already published reference
   horizon at the actual switch/publication time. Do not relabel an older
   sample-zero P/V/A as “now,” creating a reference rewind.

7. **Defined failure branches.** Specify what is published if a refresh/cancel/
   manual blend cannot be built or both planned and live ring evidence are
   unavailable. Reuse existing manual/input-loss contracts; no indefinite
   refitting, expired-prefix execution or blocked manual override. If an existing
   contract cannot cover a branch, return one concrete question before inventing
   a new physical recovery behaviour.

## Implementation and evidence milestones

- Keep separate reversible commits for backend/handoff support and mission/launch
  integration where practical; update CHANGES_FROM_TEJEN.md. Keep one active plan.
- Write focused tests for P/V/A/time agreement, prefix refresh/coverage, obsolete
  results, context cleanup, timeout without resets, detachment fast path and
  failure, manual LAND between publications, failed blends, automatic landing
  and feature-off equivalence. Preserve the existing loaded-approach contracts.
- **First end-to-end milestone:** use the actual new backend/handoff in the
  recorded moving-ring replay under the agreed geometry and limits. Demonstrate
  that the search can find and install an exit, rather than only checking a
  hand-drawn free route. If it cannot, report the blocker before expanding recovery.
- Run the bounded v2/v3 replay selection, idle then CPU-loaded, and the proposed
  headless twin wiring/manual-override checks. No replacement-success campaign
  or automatic tuning. Perfect-tracking/recorded-error replay is not flight proof;
  the slow twin does not validate real-time deadlines.
- Return exact commits/diff, resolved parameters, test/build results and retained
  replay/twin artifacts. Report handoff/exit time, attempted/accepted/installed
  plans, late candidates, failures, reference joins, geometry checks and landing.
  Do not count a manual Python recovery as a C++ exit success.

Codex will review the **actual implementation and evidence** next. No additional
general planning essay is requested. Physical commissioning requires a subsequent
operator go-ahead: one hover-first flight, analysis, then a targeted retry if
promising or retirement if not. No rig flight is authorised by this document.

Only this handoff document was created in Te-Jen's repository; production code
and flight configuration here remain unchanged.
