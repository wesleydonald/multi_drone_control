R6.3C.1b/C.1c CURRENT INTEGRATION NOTE - 31 AUG 2026
========================================================
This cumulative tree is now version 0.11.0. Read C1BC_INTEGRATION_NOTE.txt and
the package-level INSTALL_AND_TEST_C1BC.txt before running the passive Gazebo
commissioning. B.4 planner/search/collision settings remain frozen.

R6.3B.4.2 SUSPENDED-SYSTEM CANDIDATE - 31 AUG 2026
=======================================================
Start with B4_IMPLEMENTATION_NOTES.txt, B4_1_OCTOPUS_9_RUNTIME.txt,
B4_2_REPLAN_10HZ.txt and INSTALL_AND_TEST_COMMANDS_B4.txt. B.4 adds component-wise body/cable/magnet/
optional-payload collision geometry for the planning drone and fixed/shared other
drones. B.4.1 keeps that geometry unchanged, makes 9x9x9 the suspended-mode demo/
regression default after both approved witnesses succeeded natively at 9x9x9, and
adds runtime instrumentation. B.4.2 keeps the 30 Hz control/reference cadence but
requests suspended replans at a provisional 10 Hz, coalescing requests if a solve is
still running. The frozen B.3/body-only setting remains 7x7x7.

R6.3B.3 - cooperative commissioning + R_a=2.0 m finalization candidate
========================================================================

CURRENT CHECKPOINT - 31 AUG 2026
--------------------------------
The B.3 moving-other-drone stress missions have been run natively and demonstrated
safe/lively conflict resolution under the fixed-shared-trajectory assumption. A planning-
radius sweep then showed that R_a=1.0 m was comparatively reactive, R_a=1.5 m could
lose local feasibility in the two-drone stress geometry, and R_a=3.0 m effectively became
a one-shot full-mission plan for the 3 m commissioning mission.

Engineering choice for the next baseline:
  default planning radius R_a = 2.0 m

The cooperative_mission_demo keeps --planning-radius as an experimental override.
The core RecedingHorizonConfig default and receding_horizon_demo default are also 2.0 m.
No Octopus, MINVO, separator, qpOASES objective, Check/Recheck, safety-checker, dynamic
limit, spline-degree, or segment-count logic is changed by this finalization.

Run the complete 13-test native suite after installing this package. The new 2.0 m
default must not be called frozen/validated until that native suite passes.

The next substantial topic after that check is B.4 suspended cable/electromagnet/attached-
object whole-body C-space planning, beginning with math/code/questions before coding.

-----------------------------------------------------------------------

R6.3B.2 - native C++ newest-world Check/Recheck + atomic commit gate
===================================================================

Status entering this package
----------------------------
R6.3A is validated/frozen.
R6.3B.1 is validated/frozen on the user's native GLPK + qpOASES machine:
- 8/8 tests passed
- 3 m straight mission reached the 5 cm goal tolerance
- 186 accepted replans, 0 failed replans
- max native C2 splice residue approximately:
    p = 2.23e-9 m
    v = 3.39e-9 m/s
    a = 9.63e-9 m/s^2
- numerical C2 acceptance tolerance remains 1e-7

What B.2 adds
-------------
1. Immutable/versioned planning snapshots.
2. Cooperative obstacle trajectories represented as:
       shared committed center trajectory
       + physical body AABB
       + configurable tracking-error AABB
       + ego AABB
   The initial tracking-error commissioning value is +/-0.05 m per axis.
3. Exact continuous cubic convex-hull checking for arbitrary time intervals.
   Partial spline spans are converted exactly to cubic Bezier control points.
4. Post-solve newest-world safety recheck of:
       incumbent prefix [candidate_finish, A]
       candidate suffix [A, candidate_end]
5. Atomic version-gated authority change. replaceSuffix() runs only while the
   world version is locked/current.
6. One bounded recheck retry if the world changes between the first newest-world
   check and commit. A second version race is rejected; there is no retry loop.
7. Distinct unsafe-prefix semantics:
       INCUMBENT_PREFIX_UNSAFE
   This does NOT fall through to a claim that continuing the incumbent is safe.

Important literature/scope statement
------------------------------------
B.2 is MADER/RMADER-inspired Check/Recheck, but it is NOT full RMADER Delay
Check. The current thesis assumption is asymmetric cooperation:
- pickup drone is the active replanner
- carrier/basket drones expose committed shared trajectories
- those trajectories are treated as fixed commitments over the horizon used
  by the pickup drone

If multiple agents later replan simultaneously and can change their advertised
future trajectories under communication delay, full RMADER-style publication /
Delay Check must be reconsidered.

The +/-0.05 m tracking tube is a project commissioning value, not an RMADER
parameter or experimentally proven bound. It should later be replaced/tightened
from measured tracking-error statistics.

Not in B.2
----------
- no ROS2 integration
- no MPC authority integration
- no full RMADER Delay Check
- no background/global runtime safety monitor after a candidate is committed
- no emergency controller implementation for INCUMBENT_PREFIX_UNSAFE
- no moving-rendezvous terminal-condition redesign

Run INSTALL_AND_TEST_COMMANDS_B2.txt.
