# C++ exit review and offline closeout

For Wesley and Claude, from Te-Jen and Codex, 15 November 2026.

Continue the approved C++ exit implementation in Wesley's integrated fork. The
moving-prefix approach is working in replay; retain it and close the specific
issues below. This supplements the implementation handoff and v3 contract. It
does not authorise rig flights or a new automatic recovery policy.

## Review result

**Verified from the supplied artifacts:** the post-fix table contains 67 runs,
all with an initial joined C++ handoff; 66 reach AT_START. The retained failure
is `fm9h8_14_e_own_load8`. Codex reconstructed the changed mission/backend source
from the supplied baseline and patches and independently reran the 12 pure
spline/transaction tests: all pass. The other build and test totals remain
**Handoff-reported**, not independently rerun here.

The replay demonstrates that the actual backend can find and install exits.
It uses perfect or recorded-error tracking without simulated swing, bypasses
parts of the mission runtime, and does not establish real-time deadline or
physical-flight performance. The pending twin checks remain necessary.

## Investigate the post-handoff disable first

In `fm9h8_14_e_own_load8`, the initial handoff succeeds around +8.73 s. Around
+9.62 s the backend disables with `COOPERATIVE_SHARED_INPUT_INVALID_DISABLED`
and an empty blocker string. The mission follows the cached stopped tail and
parks while the carriers continue moving. Your reconstruction reports a later
73 mm cable-to-tether centreline separation; this is a failed result, not a
successful exit or an exact physical-clearance measurement.

There is a concrete diagnostic gap in `transfer_backend_node.cpp`:

- `refreshCooperativeWorld()` can fail through
  `refreshSharedCooperativeObstacle(..., true)`.
- `cooperativeAttachedTetherGeometry()` requires a fresh measured ring pose and
  can throw; the refresh catches the exception and returns false.
- `cooperativeInputBlockers()` checks peer inputs and
  `movingBasketTrajectoryFresh()`. In solid-box mode the latter does not require
  that same measured ring-pose freshness, so an empty blocker string does not
  establish that every required input was valid.

Instrument the actual failing branch, including the underlying exception and
ring-pose age at that instant, and replay the retained failure. Ring-pose
freshness is a plausible cause, not yet a proven diagnosis. Make only a bounded
correction supported by the reproduction; do not relax freshness or use stale
tether geometry to obtain a pass.

The previous policy remains: manual LAND can cancel C++ and explicitly invoke
the Python escape. Do not silently implement suggestion (c), automatic direct
Python escape, from your evidence report. Also do not describe indefinite
parking as an adequate resolved response. If the input failure can persist after
the correction, return one concrete recovery-policy question with the remaining
options and their consequences. Existing stationary-seed recovery must not be
enabled and called moving-prefix recovery without demonstrating continuity.

## Close reference continuity gaps

Keep these corrections scoped to the new exit unless a shared change is needed
and its legacy equivalence is demonstrated.

1. **Advance executing windows from their source/sample time.** In the reviewed
   configuration the normal C++ selection uses receipt age. The new exit backend
   stamps windows with sample time, so those clocks need to agree throughout
   execution, not just at the initial join. Preserve sampling, P/V/A and authority
   semantics. Do not enable a larger recovery feature merely to obtain this
   timestamp behaviour. Add a delayed-receipt regression against the same curve
   evaluated at the actual publication time.

2. **Keep the C++ to AT_START transition continuous.** The current Python transfer
   restarts from measured state; the reported maximum step is 0.116 m,
   0.34 m/s and 1.88 m/s². Seed the existing transition from the outgoing commanded
   reference evaluated at the actual takeover time, using existing feasible
   transfer/bridge machinery. Preserve measured capture criteria, the two-second
   dwell and existing LAND. Do not add a new mission phase or manoeuvre.

3. **Review failed transition blends explicitly.** `exit_blend_reference()`
   currently returns the raw live/direct Python reference when fitting fails.
   The new tests assert that selection but do not establish continuity. Preserve
   immediate local manual authority and cancellation; do not block LAND on a
   successful fit. Demonstrate a bounded continuous path using existing
   contracts where possible. If no such branch exists, report the unresolved
   physical response for decision instead of treating an abrupt switch as a
   continuity pass or adding an unapproved recovery behaviour.

Retain the working exact prefix, relative readiness, refresh protections,
coverage/deadline checks and obsolete-worker rejection.

## Resolve the general height limit

The agreed trial has a general **3.2 m body-origin limit**, including the loaded
approach. The supplied configuration leaves loaded `maximum_search_z_m` at
3.3 m, while exit search uses 3.2 m. Verify whether another effective bound
already enforces the agreed limit. If not, resolve the existing configuration
and bounds consistently; do not add a runtime geofence or apply exit-only XY
bounds to the loaded approach.

Distinguish body-origin bounds from assembly bounds: the exit workspace top
3.26 m and its erosion are separate quantities. Do not double-subtract geometry
or tracking margins. Retain the exit-only XY workspace ±1.8 m and reserved mat
(0, +1.55).

## Validate and return evidence

Checkpoint the reviewed implementation before further changes. Keep focused
corrections reversible and record exact commits or patches and baseline hashes.
Preserve feature-off behaviour and the frozen paper configuration.

Run focused regressions for the reproduced disable, source-time alignment,
AT_START continuity with tracking error, failed blends, manual authority and
height propagation. Preserve the existing loaded-leg, epoch, collision and
handoff checks; report pre-existing failures separately.

Then complete the already agreed bounded replay/twin checks:

- Reproduce the retained failure before the correction; retain both outcomes.
- Check representative moving-ring exits and reference joins under idle/load.
- Exercise the actual mission/controller wiring in the headless twin: hover
  exit, seven-second preparation timeout, manual LAND during preparation and
  after grant, automatic arrival-to-LAND, and fleet LAND during/after ego descent.

Use existing settings and test budgets. Retain failed attempts; do not run a
replacement-success campaign. Report feature correctness separately from mission
completion and from real-time timing evidence.

Return the root cause or remaining uncertainty, exact changes, resolved bounds,
P/V/A steps, tests/builds, replay/twin artifacts and any remaining policy question.
No additional broad redesign or planning essay is needed.

## Keep unchanged

Keep MPC tuning, planner algorithms, collision margins and loaded-leg geometry
unchanged. Add no detachment proof manoeuvre, speed/position gate or dwell.
Preserve the seven-second preparation budget and manual LAND/DISARM priority.

The roughly 13.2 s median clearance-entry-to-arrival time includes a 5.9–6.8 s
climb at the existing 0.08 m/s clearance speed. Leave that speed tuning for a
separate decision after correctness and wiring checks. Physical effects remain
**implemented, flight unverified**.

Inputs reviewed: `cpp_exit_evidence_2026-11-15.md` and
`2026-11-15_cpp_exit_review.zip`. Only this handoff was added in Te-Jen's repository;
production code and configuration here remain unchanged.
