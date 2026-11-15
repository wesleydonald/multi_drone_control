# C++ exit: decisions and requested plan revision — 15 November 2026

For Wesley and Claude, from Te-Jen and Codex.

**Review and revise the plan only. Do not implement, change flight configuration,
or launch simulations/flights yet. We want at least one more exchange before
coding approval.** This replies to `plan_cpp_exit_handoff_2026-11-15.md`.

## Decisions from Te-Jen

- **General height limit: 3.2 m.** This is the combined mission's general
  drone-body height limit, not an exit-only exception. It does not mean command
  the drone to 3.2 m: retain the lowest workable exit height. Show how the goal,
  search/workspace bounds and overshoot allowance fit the limit; distinguish
  body-origin limits from full-assembly workspace bounds.
- **Exit swing assumption: trial 5 degrees.** Treat this as an experimental
  modeling choice, not a claim that it contains all observed swing. Evaluate it
  with the higher permitted height; do not change loaded-leg geometry silently.
- **No separate no-overflight rule.** The planner may choose a route above a
  carrier if its ordinary 3D collision checks permit it. Remove “never over a
  carrier” as an additional pass/fail criterion. Do not remove collision checks.
- **Landing point:** preferably the original takeoff XY, but not if the ring or
  carriers will also land there. Inspect Wesley's actual flight/landing sequence
  and propose one fixed, reserved ego landing point and ordering. No autonomous
  landing-site selector is needed. Automatic landing after the return remains
  the intended end state, using the existing LAND sequence.
- **Initial handoff timeout: 7 seconds.** This is not a maximum duration for the
  entire return. On timeout report the failed handoff; do not silently substitute
  a Python return or introduce a world-fixed stop. Manual LAND must remain able
  to cancel C++ and invoke the agreed Python escape/return-and-land path.
  Describe its behavior before and after C++ takes authority. Immediate DISARM
  and existing manual authority remain unchanged.
- **Flight iteration:** one flight, then analysis. If the failure has a credible
  bounded remedy, discuss a targeted retry. If analysis is unpromising, retire
  the feature. There is no automatic one-failure abandonment or open-ended sweep.
- Preserve the frozen paper configuration and keep this new behavior opt-in.

## Agreed moving-reference handoff

Python continues the ring-following clearance reference while C++ prepares.
Readiness concerns motion relative to that reference, not zero world velocity.
The first three cubic control points must encode its world-frame position,
velocity and acceleration at the **future splice time** using the existing
B-spline boundary equations. They are not three recent measured positions.

Both owners must share the same absolute-time prefix. Neither activation nor
grant may replace it with a stationary seed or shift a prepared curve's clock.
Keep existing collision checking, epoch/version/deadline checks and one reference
owner. No planner-algorithm or MPC-tuning redesign is requested.

## Points to resolve in the next reply

1. **Live-reference to prefix continuity.** A translation of the ring spline
   matches position only; it need not match velocity or acceleration. The stated
   ~0.1 m/s step is not inherently required by planning. Explain how to construct
   a continuous latch/transition without another restrictive readiness gate.
2. **Coverage, delays and cancellation.** The draft's 5-second prefix needs
   reconciling with the 7-second preparation timeout, elapsed-time coverage and
   freshness checks. Specify the timer origin/reset policy and what reference
   executes during rejected/late attempts. Do not execute past prefix coverage.
   Cancelling back to live tracking or manual Python takeover must also avoid a
   position/velocity/acceleration jump.
3. **Exit-only context.** Bind unloaded operation and exit-specific workspace
   handling to the authorized return/prefix transaction. A generic static-target
   flag should not alone authorize removal of loaded-leg attachment checks.
   Reuse the transaction rather than creating another independent mode framework.
4. **Resolved geometry.** Show the proposed exit goal and effective bounds with
   5 degrees and the general 3.2 m body-height limit. Distinguish a geometrically
   free candidate route from a route the real-time search has demonstrated it
   can find. The exploratory eight-flight geometry results remain reported, not
   independently reproduced by this review.
5. **Landing and manual override.** Identify the fixed mat relative to the ring's
   eventual landing area. Describe AT_START -> LANDING and how Wesley's fleet
   take-back handles an ego already landing/disarmed. Confirm that manual LAND
   atomically relinquishes C++ authority before Python commands the escape.

Return a compact amended file-level plan, the few remaining human choices, and
the proposed replay/twin checks. Keep ordinary low-speed motion separate from a
failed handoff; do not add a low-speed-only automatic takeover trigger. Backup
path changes require explicit review; a radial path is not automatically safe.

**No code yet.** Once this revision is agreed, implementation and validation can
be authorized separately. No production files were changed for this handoff.
