# C++ exit: reply to the review of v2 (v3, 15 Nov 2026): context lifecycle, prefix ownership, bounds, manual cancel

For Te-Jen and Codex, from Wesley and the lab chat. **Plan only: no code, configuration change or launch.**
- The settled items in section 5 of the review are taken as given and not reopened:
  - 3.2 m as the general body-height limit of the new trial configuration;
  - 5 deg as the exit trial;
  - 7 s handoff timeout;
  - collision checks only;
  - 2.9 m / 1 s / 1 s / 2 s as starting values;
  - the mat reserved, and QUAD4 landing first;
  - the direct Python return as a manually authorised recovery, not a certified route;
  - one flight, then analysis.
- Everything in v2 not touched here stands.

## 1. Exit context: bound during preparation, before activation and grant checks
- **Opening.** The mission opens the exit transaction at the latch (preparation start) with
  `<id> seedtraj exit <seq> <stamp_ns>`. It sends it only from the post-drop branch, after its own release decision.
- **What the backend does on accepting it,** before any activation or grant check and before certifying the prefix:
  1. checks the transaction is well formed (vehicle, frame, sequence) and that the object reads detached on a fresh
     `/tejen/object_attached` (proposed belt-and-braces; see section 5);
  2. sets `exit_context` for this transaction id only;
  3. installs the unloaded geometry (payload off, 5 deg swing) and the exit bounds (section 3) through the existing
     geometry rebuild;
  4. only then certifies the prefix and runs activation, with the payload-attached checks skipped **for this
     transaction**.
- **Execution authority** stays separate: only the existing grant gives it.
- **Cleanup.** `exit_context` ends, and the loaded-leg geometry, bounds and checks are restored, on any of:
  - cancel;
  - the 7 s deadline;
  - LAND;
  - AT_START reached;
  - a new transaction id;
  - backend disable.

  Outside a live exit transaction every loaded-leg check applies as today. `static_target` alone grants nothing.

## 2. Prefix lifecycle: one owner of the curve, the same coefficients on both sides
- **One curve.** What both sides execute is the fitted cubic B-spline in the CommittedTrajectory message, evaluated at
  the same absolute times.
  - Python publishes QUAD4's reference by evaluating those coefficients, never the analytic blend.
  - The quintic of v2 is used only to produce the fit targets.
  - The first three control points come from the shared reference's p/v/a at the start of coverage, by the existing
    boundary equations; no stationary reseed and no clock shift.
- **Refresh, preserving the protected prefix.**
  - Prefix k+1 copies prefix k's pieces **unchanged** (the same knots and coefficients) over a protected window
    [t_R, t_R + Pw]. It continues after that window with new pieces, joined C2 by the boundary equations at the
    junction.
  - Pw >= the splice lookahead (0.30 s) + the worst replan latency (p95 0.35 s on 13 Nov) + margin; start at 1.5 s.
  - A candidate planned against prefix k, splicing at <= now + lookahead + latency, splices inside the copied window, so
    it stays valid against prefix k+1 and is not silently replaced.
  - The backend drops any candidate whose splice time falls outside the window it shares with the newest prefix
    (sequence and protected window carried in the transaction).
- **Deadline and coverage.**
  - The 7 s deadline is set once, at the first latch; refreshes and retries never reset it.
  - Coverage end >= splice + 2 s is required for a grant.
  - Python never evaluates past coverage end. If a refresh cannot be built in time (ring commitment older than its
    freshness limit), Python switches to the cancel curve below before coverage ends.
- **Bounds on every curve QUAD4 may execute.** Each fitted curve (prefix, refresh, cancel and the manual-escape
  start) is checked before it is published:
  - P/V/A continuity at its start and at every junction, numerically, to 1e-6;
  - |v|, |a|, |j| under the planner's own limits over its whole span.

  If a fit violates them, the blend time is lengthened and the curve refitted. If it still fails, no exit transaction
  opens on that tick (QUAD4 stays on the live ring-following reference) and the reason is logged. Fit error is
  reported but is not the acceptance test.
- **Cancel back to live** (timeout, refresh failure, rejection after the deadline) uses the same machinery: a fitted
  cubic from the executed curve's p/v/a at the switch time to the live ring-following reference, bounds-checked as
  above. There is no world-fixed stop.

## 3. Bounds: body-origin policy, the assembly workspace supplied, and the centre bounds that result
Computed with his own `assemblyCentreBounds` (built libraries, the rig line's values: body half 0.105/0.105/0.06,
tracking 0.05, cable 1.034, magnet 0.05/0.025):
- The assembly's horizontal extent is +-0.140 m unloaded at 5 deg (the swung magnet).
- At 3 deg it is +-0.105 m, loaded or unloaded (the body).

| Layer | x, y | z | Source |
|---|---|---|---|
| Body-origin policy (physical) | +-1.8 m (the fleet position limit); QUAD4 also |y| < 1.7 m (motion capture lost it beyond, 11 Nov) | <= 3.2 m (the general limit) | Settled policy |
| **Assembly workspace supplied to the backend** | **+-1.80 m** | **0 to 3.26 m** (the body's top face when its origin is at 3.2 m) | The policy, with nothing pre-shrunk |
| Resulting centre bounds (his erosion: minus the assembly extent and tracking) | **+-1.61 m** at 5 deg unloaded (+-1.645 at 3 deg) | **<= 3.15 m** (3.26 - 0.06 - 0.05) | Computed |
| What the body can reach (centre + tracking) | <= 1.66 m: inside 1.8 and 1.7 | <= 3.20 m | Check |

- **Fits.**
  - The exit goal, AT_START over the mat at (0.00, +1.55, ~2.5), lies inside the centre bounds with 0.06 m to spare in
    y.
  - The candidate exit route (|x| <= 0.73, |y| <= 1.31, hold 2.9 m) lies inside them.
  - The hold plus the launch's overshoot allowance is 2.9 + 0.20 = 3.10 m, under 3.2.
  - `cpp_max_z` is set to 3.2 so that the eroded workspace (3.15) is the binding top. The two never double-shrink: his
    code takes the minimum.
- **The landing descent** below AT_START is the existing Python LANDING, not C++. Its column was clear by his margins
  in all 8 drops (v2).
- **Whether the bounds should cover the whole trial flight.** If the xy workspace is applied to the whole trial
  configuration, the loaded approach from the 1.55 m mat also fits (centre bound +-1.645 m at 3 deg). The failure of
  12 Nov (R4366-R4373) came from starts 1.65-1.9 m out.

## 4. Manual LAND: the real cancellation path, local selection first
- The backend ignores `authority=false`, so it is not used. In the LAND callback the mission, which is the only
  publisher of QUAD4's reference to his MPC, does in this order within the one callback:
  1. **Local selection first.**
     - Clear `c1f2_cpp_authority_active` and its prepare/grant flags.
     - End the exit context locally.
     - Set the reference selector to Python.
     - Record the current backend epoch and sequence, so any later C++ reference or status from the old transaction
       is ignored.
  2. **The backend's real off path,** fire-and-forget with no acknowledgement wait:
     - the recovery **Cancel** for the transaction;
     - **enable=false**.

     Both disable the backend; obsolete worker results and cached authority are invalidated by the epoch bump on its
     side and by step 1 on ours.
  3. **The first escape sample** comes from the last reference sample the mission itself published. That is known
     exactly whether it came from C++, the prefix or the live reference. The escape starts with a bounds-checked
     fitted cubic from that sample's p/v/a into the manual recovery path: the direct return to AT_START, then LANDING.
- **What is and is not claimed.** The switch is atomic only in the sense that QUAD4's reference changes owner between
  two consecutive publications of the one publisher. Nothing is claimed about the two processes' internal states
  switching together.
- Immediate DISARM and the existing manual authority are unchanged.
- The direct return keeps its qualification: it intersected his model on 13 Nov, so it is a manually authorised
  recovery, not certified.

## 5. Remaining human choices
1. **The xy assembly workspace (+-1.8):** for the exit transaction only (the lab chat's suggestion, as v1 asked), or for
   the whole trial configuration (it fits the approach from the 1.55 m mat).
2. **Belt-and-braces object check:** open the exit context only if a fresh `/tejen/object_attached` reads detached
   (suggested: yes).
3. **The protected window Pw:** 1.5 s as a starting value, set from the replay's latencies.
