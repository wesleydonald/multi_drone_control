# C++ exit v2 review and remaining work

For Wesley and Claude, from Te-Jen and Codex, 15 November 2026.

Continue in Wesley's integrated fork from the corrected v2 tree. Preserve the
working moving-prefix implementation and its continuity/height corrections.
This requests the bounded offline work below; it does not authorise rig flights
or automatic Python escape.

## Accepted progress

Codex checked the supplied patches, eight corrected replay results and selected
raw twin logs. All eight replays reach AT_START and then LANDING. Source-time
execution and C++ to AT_START continuity are substantially improved. The twin
demonstrates hover exit, manual LAND before/after grant, and fleet takeover
during descent. The loaded-search configuration now respects the 3.2 m limit.

Keep these changes. The full test-suite totals remain **Handoff-reported** here;
the replay and slow twin do not establish physical or real-time performance.

## Apply the diagnostic logging

Apply `2026-11-15_cpp_exit_disable_diag.patch` in your fork, after checking it
against the current corrected tree. The previous handoff already requested this
instrumentation; its location in the imported backend is not an additional
approval requirement. Preserve behaviour and throttle any recurring diagnostics.

Report the actual failing branch, exception and input ages if the retained
post-grant disable recurs. Keep the diagnosis qualified: the five reruns did
not reproduce it, and the measured 0.23–0.33 s bridge gaps do not themselves
demonstrate a breach of the 0.5 s ring-pose limit. The empty blocker string is
incomplete evidence, not proof that every required input was valid.

## Remove bridge refresh work from the publication path

The measured publication stalls justify a focused correction to
`fake_cooperative_transport_world.py`: keep expensive commitment construction
from blocking the existing pose/state publication callbacks. Inspect the current
implementation and choose the smallest workable isolation; do not add a new
coordinator or redesign trajectory generation.

Preserve a consistent input snapshot and complete, consistent commitment
publication. If work becomes asynchronous, prevent obsolete results from
overwriting newer state/configuration using the existing sequence/lifecycle
semantics. Do not hold a shared lock through the expensive fit and thereby
recreate the publisher stall. Merely selecting a MultiThreadedExecutor does not
establish isolation: inspect callback groups, shared data and Python execution
contention.

Keep future trajectories, geometry, refresh/publication rates, time bases and
freshness thresholds unchanged. Compare publication gaps and commitment validity
before/after under the same bounded idle/CPU-load replay conditions. Report any
remaining stalls rather than increasing timeouts.

Separately record that stamping republished held mocap as "now" does not prove
measurement freshness. Distinguish measured, predicted and republished outputs.
Do not silently change the upstream timestamp/validity contract in this patch.

## Inspect the circle twin rejection

R4535 reportedly opens the exit context but accepts zero of 33 candidates before
the seven-second timeout. Its detailed artifacts are absent from the supplied
ZIP. Collect the backend replan CSV, status, resolved configuration and relevant
reference/prefix records for that attempt; rerun one instrumented circle check
if those records were not captured.

Identify whether rejection comes from geometry/coverage, search or refinement,
checking, authority/version invalidation, or timing. Compare the actual twin
state and limits with the successful circle replay. Do not infer a planner defect
or change budgets, gates or geometry from the zero-acceptance count alone.
Report the cause and smallest supported next action. No new tuning sweep.

## Complete a valid timeout wiring check

R4533 demonstrates that the deadline fires, but supervisor-triggered disarm
invalidates the intended continuous timeout-recovery check.

Prefer withholding the grant through the twin test harness while keeping backend
status and required inputs alive. Do not add a production mission switch solely
for this test. Keep the actual seven-second budget and cancellation behaviour
unchanged. If the existing harness cannot isolate the grant, a clearly documented
per-run supervisor timeout override is an alternative for the SIGSTOP test;
restore it afterward and retain bounded cleanup. Neither method validates
real-time deadlines in a slow twin.

Verify that the mission cancels at seven seconds, follows the live ring reference
through the existing blend, and still accepts manual LAND. Retain the invalid
R4533 attempt alongside the valid result.

## Recovery policy remains open

Do not introduce automatic direct Python escape or moving-prefix re-seeding
after post-grant input loss in this increment. Manual LAND remains the explicit
escape, with immediate local authority and real backend cancellation; DISARM
remains unchanged.

Indefinite parking over a moving ring is not a resolved response either. Return
the remaining genuine-input-loss case as one concrete question for Te-Jen,
including the practical consequences of the available options. Successful bridge
repair must not be presented as eliminating all possible input loss.

Retain these limitations explicitly:

- Failed cancel blends can still switch discontinuously to the live reference.
- Failed escape/arrival blends preserve position and velocity, with a possible
  acceleration step; they are not fully C2-continuous fallbacks.
- Existing AT_START to LANDING and the manual escape's own arrival still have
  the reported legacy steps. Leave those paths and clearance-climb speed alone
  in this round.

## Checkpoints and return package

Checkpoint the corrected v2 tree before further edits. Keep logging and bridge
changes separately reviewable, with exact commits or patches and baseline hashes.
Preserve feature-off behaviour, frozen flight configurations, MPC tuning,
planner algorithms, margins and the agreed manual-authority contracts. Add no
mission phases or qualification gates.

Run focused publication/concurrency tests, relevant existing regressions and
scoped builds. Use bounded replay/twin checks for the specific outstanding cases;
retain failures and do not replace them with an undisclosed success campaign.

Return publication-gap comparisons, actual disable diagnostics if reproduced,
circle rejection evidence, the valid timeout result and the unresolved policy
question. Mark physical effects **implemented, flight unverified**.

Reviewed inputs: `cpp_exit_evidence_2026-11-15_v2.md` and
`2026-11-15_cpp_exit_review_v2.zip`. Only this handoff was created in Te-Jen's
repository; no production code or configuration was changed here.
