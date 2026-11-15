# C++ exit: review of handoff plan v2 — 15 November 2026

For Wesley and Claude, from Te-Jen and Codex.

This reviews `plan_cpp_exit_handoff_2026-11-15_v2.md` and supplements
`20261115-cpp-exit-plan-reply.md`.

**No code yet.** Please resolve the specific contract issues below in a compact
reply/amended plan before implementation approval. No flight/configuration
changes or simulator launches are authorized by this review. No new planning
framework, mission phases or restrictive readiness gates are requested.

## 1. Establish exit context before activation, not after grant

V2 section 3 activates the unloaded exit context “from its grant.” However, the
backend checks payload attachment during activation and again during grant.
After delivery this would prevent reaching the grant that enables the exception.

Bind the exit context to the validated exit-prefix transaction **during
preparation**, before those checks and before prefix certification. Execution
authority remains separate and is granted only through the existing handoff.
Install the appropriate unloaded geometry and exit bounds before checking or
planning the prefix. Preserve loaded-leg checks outside that transaction and
define cleanup on cancellation/exit. A static target alone grants no exception.

## 2. Define prefix refresh and the exact executed curve

V2 describes both binding the newest prefix and fixing its sequence per
transaction. Specify one consistent lifecycle:

- Do not silently replace the prefix an in-flight candidate was planned against.
- A refresh must preserve the protected prefix that candidate depends on, or
  invalidate the old transaction/worker results before rebinding to the new one.
- Preserve the original 7-second preparation deadline across refreshes/retries.
  Keep the stated coverage/freshness and timeout-to-live-continuity requirements.
- Both Python and C++ must evaluate **the same fitted cubic coefficients at the
  same absolute times**. Do not execute the analytic quintic expression in Python
  while C++ receives an approximate cubic fit.
- Validate actual fitted-cubic P/V/A continuity and velocity/acceleration/jerk
  bounds, including refreshes and cancellation. Reporting fit error alone does
  not establish those properties.

Keep the first three control points derived from the shared reference's P/V/A at
the appropriate joining time; no stationary reseed or clock shifting.

## 3. Reconcile body-centre limits with assembly workspace erosion

V2 calls +/-1.65 m a body-XY workspace and proposes landing at (0,+1.55).
The supplied backend's existing workspace interface is an **assembly workspace**:
it erodes the bounds using the collision assembly and tracking allowance.

If +/-1.65 is passed directly to that interface, even an identity body proxy
alone at Y=1.55 requires:

    1.55 + 0.105 body half-width + 0.05 tracking = 1.705 m

That exceeds1.65 before considering the cable/magnet. The proposed mat could be
unreachable by construction.

Show separately the physical/body-origin policy limits, the actual assembly
workspace bounds supplied to the backend, and the resulting centre bounds.
Avoid manually shrinking for margins and then applying the same erosion again.
Verify the landing approach and an exit route fit the resolved bounds; do not
silently reduce margins or enlarge the permitted physical area to make them fit.

## 4. Manual LAND must use actual cancellation semantics

In the supplied backend, `authority=false` is intentionally ignored: it is not
revocation. Existing recovery Cancel disables the backend, and enable=false also
disables it. Use the actual cancellation/disable path, invalidate obsolete
results/cached authority, and make the local reference selector choose Python
before its first escape sample.

Do not claim publishing a Boolean makes two ROS processes switch atomically.
Manual authority must win locally without adding an indefinite acknowledgement
wait. Preserve P/V/A continuity into the agreed manual recovery path.

## 5. Decisions already settled / reasonable trial values

- **3.2 m is the general body-height limit for the new trial configuration**, not
  an exit-only ceiling. Preserve the old frozen paper configuration separately;
  historical runs are not changed. Do not reopen that as an unanswered choice.
- **5 degrees** is the exit trial assumption, not guaranteed swing containment.
- **7 seconds** is the initial handoff timeout, not the complete return duration.
  Expiry reports failure/cancels back to continuous ring-following; it does not
  silently invoke a Python return or command a world-fixed stop.
- No special no-overflight prohibition; ordinary 3D collision checks remain.
- The proposed2.9m exit target,1s blend,1s refresh and2s AT_START dwell are
  reasonable starting values for validation, not values already commissioned.
- Reserving QUAD4's mat and landing QUAD4 before the fleet is a sensible ordering,
  subject to the actual bounds and landing sequence being checked.
- The current direct Python return remains a **manually authorised recovery**,
  not a collision-certified route. Its reported model intersections remain a
  qualification; do not label that backup checked/safe or substitute it silently.
- One flight -> analysis -> targeted retry or retirement, rather than automatic
  abandonment or an open-ended sweep.

Please reply with the corrected context lifecycle, prefix/refresh ownership,
resolved bounds and manual-cancellation sequence. Identify any genuinely
remaining human choice. The rest of v2 need not be rewritten into a larger plan.
Implementation and replay/twin/rig validation require subsequent approval.

This handoff changes documentation only; no production code/configuration changed.
