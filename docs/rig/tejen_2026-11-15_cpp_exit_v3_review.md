# C++ exit v3 review and checkpoint

For Wesley and Claude, from Te-Jen and Codex, 15 November 2026.

The v3 changes address the requested bridge-performance, circle-configuration
and timeout checks. Retain the implementation and close the reporting and Git
checkpoint work below. No further controller/planner redesign, recovery feature
or rig flight is requested or authorised by this handoff.

## Accepted progress

Codex inspected the supplied patches, comparison CSVs and selected raw twin logs.

- The trajectory-object cache in `ring_bridge.py` is a suitable small correction.
  It avoids repeated figure-eight table construction without adding concurrency.
  The reported refresh cost falls from 134 ms to 15 ms. Four of five loaded
  after-runs have maximum pose gaps below 0.1 s; the retained outlier reaches
  0.206 s. All six new replays reach AT_START.
- R4535's records support world-version invalidation as the dominant rejection.
  R4536 completes the circle exit with the rig's existing trial settings. Because
  several settings were aligned together, describe this as a successful
  configuration-alignment check, not an isolated proof that one parameter alone
  caused the difference.
- R4537 demonstrates cancellation at seven seconds, continued ring-following
  through the existing blend, and subsequent manual escape and landing.
- Keep the earlier source-time execution, arrival-continuity and height fixes.

The original rare post-grant disable is still not reproduced or conclusively
diagnosed. Reduced publication stalls are demonstrated; elimination of all input
loss is not. Full-suite totals remain **Handoff-reported** in Codex's review.

## Reconcile the supervisor verdict

The raw logs for both R4536 and R4537 contain
`Supervisor finished: FAIL EARLY_TERMINAL_DISARM`, after the mission commands
DISARM and enters LANDED_DISARMED. R4536 reports disarm at z=0.28 m; R4537 at
z=0.18 m. These are existing landing outcomes, not a request to tune landing here.

The evidence supports the observed exit/timeout/LAND features. It does not
support labelling either run a completely clean supervisor pass. Inspect the
supervisor's expected lifecycle and record why it classifies these deliberate
terminal disarms as early.

If this is an expectation mismatch, correct only the relevant twin test
configuration or classification, with a focused regression. Preserve detection
of genuinely premature disarm during flight. Do not broadly disable the guard,
change physical mission behaviour or rewrite the original verdicts. Keep the raw
FAIL outcomes and distinguish feature results from the overall harness result.

Also retain the first unexplained `gate --quick` pytest failure in the record.
Two later passes do not explain it. Capture the failing test/output if it recurs;
do not perform repeated blind reruns merely to replace that outcome.

## Checkpoint the reviewed implementation

Create reviewable commits in your fork for the related backend/handoff,
mission/launch, bridge-cache and test-harness changes. Use existing checkpoints
to preserve the pre-change variants and leave unrelated work untouched. Report
the exact commits and their baseline; the latest evidence still says nothing is
committed.

Update the implementation contract and CHANGES_FROM_TEJEN.md to reflect the
actual final state. Preserve frozen configurations, manual authority, seven-second
preparation budget, geometry/margins, timing settings and MPC tuning. No additional
detachment gates, mission phases, asynchronous framework or automatic tuning.

No new simulation campaign is needed for this closeout. Use the existing raw
artifacts for reporting and focused offline tests for any small harness change.
Report any remaining validation gap explicitly.

## Genuine input loss still needs a human decision

Keep existing manual LAND and DISARM available. Do not implement automatic
Python escape, automatic return to ring-following, or moving-prefix re-seeding
after grant without Te-Jen's decision. This is not acceptance of indefinite
parking over a moving ring as a resolved safety response.

Codex recommends retaining explicit manual LAND for an initial supervised
commissioning decision rather than expanding recovery code in this round.
Physical commissioning still requires the operator's separate go-ahead.

Return one concise policy question for Te-Jen, distinguishing loss of ring pose,
carrier inputs and ego feedback. An escape that needs no ring input still needs
fresh ego feedback. The direct Python return is not collision-certified;
successful seeding does not make every failed-blend branch fully C2-continuous.
Keep the previously reported failed-cancel-blend and legacy landing limitations
visible without expanding this patch to repair them.

## Return package

Provide exact commits, the corrected feature-versus-supervisor outcome table,
the supervisor explanation or narrowly tested harness fix, remaining limitations
and that policy question. Keep historical failures and the bridge outlier.
Physical effects remain **implemented, flight unverified**; the slow twin checks
wiring, not real-time deadlines.

Reviewed inputs: `cpp_exit_evidence_2026-11-15_v3.md` and
`2026-11-15_cpp_exit_review_v3.zip`. Only this handoff was created in Te-Jen's
repository; production code and configuration here remain unchanged.
