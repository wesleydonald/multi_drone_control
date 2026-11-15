# C++ exit: the v3 review closeout, evidence v4 (15 Nov 2026, night), for Te-Jen and Codex

This answers `tejen_2026-11-15_cpp_exit_v3_review.md`.
- Physical effects are **implemented, flight unverified**.
- The slow twin checks wiring, not real-time deadlines.
- No new simulation was flown for this closeout: it uses the existing raw artifacts and focused offline tests.

## 1. The supervisor verdict
**What the raw logs show.** Every twin check that ended with QUAD4 landed at its start has `Supervisor finished: FAIL
EARLY_TERMINAL_DISARM`. That is R4528-R4532, R4536 and R4537, and the same applies to every earlier return-after-drop
twin run.

**Why.** `simulation_test_supervisor` stops on the first terminal condition while RUNNING, and every stop requests a
disarm. The twin configures it with:
- `success_phase` `NONE_PARTNER_WELD_OWNS_SUCCESS`: never reached. Our ring weld owns success, because a supervisor
  stop before the weld would disarm QUAD4 in the air.
- `failure_phase` `LANDED_DISARMED`: on a re-attach line, QUAD4 landing means it gave up.

A `return_after_drop` line has no re-attach. Its intended end is AT_START, then LANDING, then `LANDED_DISARMED` at the
start. The supervisor's lifecycle had no success for that line, so the deliberate terminal disarm was classed as early.
This is an expectation mismatch in the twin test configuration, not a premature disarm. QUAD4 was already disarmed on
the floor at z 0.18-0.30 m, the mission's own landing.

**The narrow correction (twin test configuration only).**
- **Supervisor:** `success_after_phase` (default '' = off, so every existing configuration is unchanged). The success
  phase counts only once that phase has been seen while running.
- **Twin launch, `return_after_drop` lines only:** `success_phase` `LANDED_DISARMED`, `success_after_phase` `AT_START`,
  `require_handoff_ready` false. Every other twin line, and the rig's supervisor, are unchanged.
- **Guards kept:**
  - a landing before AT_START is still `EARLY_TERMINAL_DISARM`;
  - a disarm in flight is still `PREMATURE_DISARM`;
  - a stale required input is still `REQUIRED_INPUT_STALE`.
- **Regressions:** `test_simulation_test_supervisor.py` has 4 new cases: the pass, the early landing, the disarm in
  flight, and the default still failing the same landing. `test_cpp_exit_launch.py` covers the return line's keys and
  the other lines unchanged. The pristine-tree dump test excludes only these supervisor keys.
- **Not changed:** the guard, physical mission behaviour, landing, and the recorded verdicts. The runs were not
  re-flown.

**Feature results against the recorded supervisor outcome** (the twin checks):

| run | check | feature result | mission | supervisor (raw, as recorded) |
|---|---|---|---|---|
| R4527 | hover, start 1.9 m out | join and switch correct | FAIL: goal outside the exit bounds, held at the edge | none (run ended) |
| R4528 | LAND while preparing | cancel, escape, LANDING | complete | FAIL EARLY_TERMINAL_DISARM |
| R4529 | hover | end to end | complete | FAIL EARLY_TERMINAL_DISARM |
| R4530 | hover, corrected tree | end to end, arrival seeded | complete | FAIL EARLY_TERMINAL_DISARM |
| R4531 | LAND while preparing | escape seeded | complete | FAIL EARLY_TERMINAL_DISARM |
| R4532 | LAND after the grant | escape seeded | complete | FAIL EARLY_TERMINAL_DISARM |
| R4533 | timeout, backend frozen | deadline at 7.0 s, cancel | VOID: supervisor disarmed QUAD4 in the air | FAIL REQUIRED_INPUT_STALE (a real disarm in flight) |
| R4534 | figure-eight; fleet LAND during the descent | end to end, take-back from LANDING | complete | none (run ended) |
| R4535 | circle, no world refresh | timeout and cancel correct | FAIL: no exit (world-version invalidation) | none (run ended) |
| R4536 | circle, the trial's values | end to end | complete | FAIL EARLY_TERMINAL_DISARM |
| R4537 | timeout, supervisor limit 120 s | 7.0 s, cancel to live, LAND escape | complete | FAIL EARLY_TERMINAL_DISARM |

With the correction, the seven EARLY_TERMINAL_DISARM endings above (all landed after AT_START) are expected to classify
as PASS. That expectation comes from the regression on the same phase sequence, not from a re-flight.
- The success needs `LANDED_DISARMED` held 2 s on the supervisor's wall clock (`time.monotonic`).
- In these runs our fleet LAND took QUAD4 back 1.1-8.0 s (simulated) after it disarmed. At the twin's 0.13-0.16 times
  real time, that is about 7 s of wall time or more, longer than the 2 s hold.
- On a faster clock, a take-back inside 2 s would end the hold, and the later disarm would read PREMATURE_DISARM.

R4533's verdict was a real disarm in flight and stays a failure under the correction.

## 2. The gate failure stays unexplained
- One `tools/gate.sh --quick` run (~21:13) failed in pytest, and its output was not captured.
- Two runs passed after it (2331 passed). A third, with its output kept
  (`results/analysis/cpp_exit_twin_2026-11-15/gate_quick_2146.txt`), passed after this round's changes.
- The first failure is not explained by these passes. No blind reruns were made to replace it.

## 3. Builds and tests (this round)
- **Mission suite:** 1390 passed, with the same 26 older failures (identical set).
- **Backend:** ctest 22/22 (no backend change in this round).
- **Gate:** `gate --quick` PASSED, output kept.

## 4. Commits (branch week8, Wesley's fork; baseline 8f0afc79; not pushed)
| commit | content |
|---|---|
| 096eea44 | 1/6 his backend: the C++ exit on a moving prefix (= the v2 base's backend) |
| 0b80e0ae | 2/6 his backend: the disable diagnosis, logging only (DIAG) |
| 803e6a03 | 3/6 twin test harness: the supervisor's `success_after_phase`; the runner's SIGNAL event and `HIS:` lines |
| 98d068f3 | 4/6 mission and launch: the exit, the review corrections, the height bound, the twin supervisor settings; tests; CHANGES_FROM_TEJEN.md |
| 778aedb9 | 5/6 ring_bridge: one trajectory per refresh; test |
| (this) | 6/6 the C++ exit's docs, twin check configs and analysis scripts |

- **The pre-change variants** are kept in `~/mdc_checkpoints/`:
  - `2026-11-15_pre_cpp_exit` (before any change);
  - `2026-11-15_cpp_exit` (the first evidence);
  - `2026-11-15_cpp_exit_review_base`, `_corrections`, `_v2_base` and `_closeout` (the later rounds).
- **Left uncommitted, as unrelated:** the endless figure-eight's lines in `tools/experiment/config.py`, the registry,
  the session and decision logs, and today's other work.

## 5. What remains as it was (limitations)
- **The original post-grant disable** is not reproduced or conclusively diagnosed. Reduced publication stalls are
  shown; the elimination of input loss is not. The bridge outlier (one loaded run, 0.206 s) is kept.
- **R4536** is a successful configuration-alignment check: five trial-line values were aligned together. It is not
  proof that one parameter alone made the difference.
- **Blends:**
  - a failed cancel blend can switch discontinuously to the live reference;
  - failed escape and arrival blends keep p and v with a possible a step, so they are not fully C2-continuous;
  - seeding does not make every failed-blend branch C2.
- **Legacy steps:** AT_START -> LANDING (4-73 mm, 0.25 m/s) and the manual escape's own arrival at AT_START (15 mm) keep
  their legacy steps.
- **The C++ return has no timeout once granted.** The operator's LAND is the way out.
- **The direct Python return** is not collision-certified.

## 6. The question for Te-Jen: input loss after the switch to C++
After the grant, a genuine input loss disables the backend. What is QUAD4's response for each kind of loss?

| lost input | what still works | the options |
|---|---|---|
| **the ring pose** (> 0.5 s) | ego feedback; the carriers' commitments (predicted) | (a) park on the last window until the operator's LAND (today); (b) the automatic direct return to the start, then LANDING: needs fresh ego feedback only, not collision-certified, passes near the moving ring on an unchecked path |
| **carrier inputs** (states > 1.0 s or commitments) | ego feedback; the ring pose | (a) park (today); (b) the automatic direct return; (c) the existing cancel blend back to live ring-following over the ring: needs the ring pose, flies back over the carriers on an unchecked path |
| **ego feedback** (QUAD4's own pose) | nothing that can steer QUAD4 safely | no reference choice helps: the existing mocap-loss behaviour applies (the operator's DISARM or LAND) |

Codex recommends keeping the explicit manual LAND for the first supervised commissioning, with no new recovery code in
this round. Which response do you want for the ring-pose loss and for the carrier-input loss: (a), (b) or, for
carriers, (c)?
