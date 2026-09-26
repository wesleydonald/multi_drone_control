R6.3B.3 - cooperative moving-drone commissioning
=================================================
31 AUG 2026

PURPOSE
-------
B.3 is a commissioning/validation increment on top of the validated B.2 planner.
It deliberately does NOT change the core planner formulation or tune path optimality.

The planning pipeline remains:
  Octopus Search discrete kinodynamic B-spline seed
    -> qpOASES continuous spline refinement
    -> continuous convex-hull safety Check/Recheck
    -> atomic committed-trajectory suffix replacement

B.3 asks whether that existing pipeline can safely and live-ly plan against other
drones whose future trajectories are fixed shared commitments.

APPROVED B.3 ASSUMPTIONS
------------------------
- Other-drone trajectories remain fixed shared commitments during each run.
- Planning-drone and other-drone physical half-extents:
    [0.105, 0.105, 0.060] m
- Other-drone tracking-error half-extents:
    [0.05, 0.05, 0.05] m
- Safety + liveness are pass/fail criteria.
- Path length and runtime are diagnostics, not optimality pass/fail gates.
- No cable/electromagnet/picked-up-object collision geometry yet. That is B.4.


PLANNING-RADIUS FINALIZATION
----------------------------
After native stress commissioning, the user swept R_a = 1.0, 1.5, 2.0 and 3.0 m
on crossing_stress and two_drones_stress. The sweep showed strong scene dependence:
- 1.0 m remained feasible but produced comparatively late/reactive avoidance;
- 1.5 m failed the initial two_drones_stress local plan with
  PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED;
- 2.0 m remained feasible in the tested crossing/two-drone stress cases and produced
  a broader anticipatory maneuver while retaining receding-horizon operation;
- 3.0 m covered essentially the whole 3 m commissioning mission, producing one accepted
  full-mission plan rather than a representative rolling-horizon case.

Engineering decision: freeze the next baseline default at R_a = 2.0 m. This is a
pragmatic project value, not a claim of universal geometric optimality. The B.3 demo
keeps --planning-radius for later sensitivity experiments.

SCENARIOS
---------
1. crossing
   Planning drone: (0,0,1.5) -> (3,0,1.5)
   Other drone:    (1.5,-1.1,1.5) -> (1.5,1.1,1.5), rest-to-rest over 9.3 s.

2. same_direction
   Planning drone: (0,0,1.5) -> (3,0,1.5)
   Other drone:    (0.85,0,1.5) -> (3.70,0,1.5), rest-to-rest over 12.0 s.

3. two_drones
   Planning drone: (0,0,1.5) -> (3,0,1.5)
   Other drone 1:  (1.05,-1.1,1.5) -> (1.05,1.1,1.5), 7.0 s.
   Other drone 2:  (2.05,1.1,1.5) -> (2.05,-1.1,1.5), 13.0 s.

STRESS SCENARIOS
----------------
The first three cases are retained because they demonstrate that time-aware planning
does not conservatively avoid every point in another drone's full spatial swept path.
However, their first native plots showed that the planning-drone path could remain nearly
straight. B.3 v2 therefore adds three deliberately stronger cases:

4. crossing_stress
   Other drone: (1.50,-0.40,1.5) -> (1.50,1.00,1.5), 16.0 s.
   With the approved 0.26 m effective y half-extent, its C-space tube covers the
   direct y=0 centreline at x=1.5 for approximately t=4.50..7.80 s.

5. same_direction_stress
   Other drone: (0.55,0,1.5) -> (3.65,0,1.5), 16.0 s.
   It starts only 0.55 m ahead and remains on the direct centreline while moving
   substantially more slowly, so an unobstructed plan should catch its tube.

6. two_drones_stress
   Other drone 1: (1.00,-0.30,1.5) -> (1.00,1.20,1.5), 18.0 s.
   Other drone 2: (2.00, 0.30,1.5) -> (2.00,-1.20,1.5), 18.0 s.
   Both direct-path intersections are covered by their C-space tubes for roughly
   t=3.26..7.97 s, creating overlapping conflict windows.

STRESS-WITNESS CONTRACT
-----------------------
Every stress run first executes the SAME receding-horizon planner in an empty world
and samples that unobstructed trajectory against the configured other-drone tubes.
The stress scenario is rejected as invalid unless:

  unobstructed-reference minimum simultaneous clearance < 0

Therefore a passing stress test cannot merely be a conveniently mistimed encounter:
the nominal obstacle-free mission is explicitly shown to collide, after which the
obstacle-aware mission must still satisfy the normal B.3 safety/liveness contract.
This witness is a commissioning diagnostic, not the executed-path safety proof.

OTHER-DRONE TRAJECTORY MODEL
----------------------------
Each fixed shared trajectory is a three-span clamped cubic B-spline with repeated
start/end control points. This gives exact rest-to-rest boundary conditions:
  v(0)=a(0)=0, v(T)=a(T)=0.

The helper checks its derivative control points against the same commissioning
limits used by the planning drone:
  |v_axis| <= 1.0 m/s
  |a_axis| <= 1.5 m/s^2

B.2 stopped-endpoint hold semantics keep each other drone stationary after its
explicit committed motion ends.

COLLISION MODEL
---------------
For other drone j, each local planning interval uses the exact cubic convex hull
of its shared centre trajectory, inflated by:
  planning-drone body AABB
  + other-drone physical AABB
  + other-drone tracking-error AABB.

The final mission safety gate remains the existing continuous B.2
TrajectorySafetyChecker. Sampled clearance diagnostics do NOT replace it.

DIAGNOSTICS
-----------
cooperative_mission_demo logs:
- trajectory and replan CSVs
- other-drone trajectory CSV
- goal reached / goal seen
- continuous final safety certificate
- accepted / failed / late replans
- Check/Recheck retry count
- maximum C2 p/v/a splice residue
- mean / median / p95 / max total replan runtime
- mean / median / p95 / max Octopus runtime
- mean / median / p95 / max qpOASES runtime
- executed path length and straight-distance ratio
- sampled same-time AABB clearance
- sampled time-ignored swept-path AABB clearance

The time-ignored diagnostic intentionally discards timing. A useful crossing result
can therefore have positive same-time clearance but negative time-ignored clearance,
showing that the drones safely used overlapping physical space at different times.

REGRESSION CONTRACT
-------------------
CTest adds two end-to-end tests:
  test_cooperative_mission
  test_cooperative_stress

The first preserves the original three temporal-awareness scenarios at their historical
commissioning radius R_a=1.0 m. This is intentional regression preservation: these synthetic
scenes were authored/timed around the original B.3 configuration and are not used to select
the new default radius. The second runs all three stress scenarios at the current default
R_a=2.0 m and additionally requires the negative-clearance unobstructed conflict witness
described above. Path shape is deliberately NOT fixed.
Each scenario must instead satisfy physical/authority invariants:
- reach the global goal
- final continuous executed-trajectory safety = SAFE
- every authoritative accepted replacement has safe prefix + safe candidate checks
- every authoritative accepted splice satisfies the existing C2 tolerance
- no accepted candidate is late
- no INCUMBENT_PREFIX_UNSAFE condition occurs
- fixed world remains version 0 and requires no Check/Recheck retry

A rejected/late replan attempt by itself is not a regression failure if the safe
committed trajectory remains valid and the mission still reaches the goal.

FROZEN CORE PLANNER SETTINGS
----------------------------
B.3 does not change:
- 4 local spline segments
- 7x7x7 Octopus samples by default
- alpha_shrink 0.9
- voxel_fraction 0.10
- heuristic_bias 1.0
- Octopus max runtime 2.0 s
- qpOASES refinement / max WSR 500
- jerk limit 4.0 per axis
- planning radius 2.0 m (engineering default; CLI-overridable in cooperative_mission_demo)
  - legacy test_cooperative_mission explicitly retains R_a=1.0 m for historical regression
  - test_cooperative_stress exercises the current 2.0 m default
- alpha 2.5
- splice lookahead 0.05..1.0 s
- factor_alloc 1.0
- factor_alloc_close 2.5
- spline_time_factor 2.5
- close-to-goal 0.20 m
- goal tolerance 0.05 m
- C2 tolerance 1e-7
- separator validation tolerance 1e-7
- v_max 1.0 m/s per axis
- a_max 1.5 m/s^2 per axis

SANDBOX VALIDATION LIMITATION
-----------------------------
The assistant environment does not contain the user's real GLPK + qpOASES native
installation. The new B.3 translation unit and CMake/test wiring were checked here,
but final end-to-end solver execution must be performed on the user's Ubuntu machine.
The prior R_a=2.0 candidate showed test_cooperative_stress passing natively but exposed that
the legacy crossing scenario is not feasible at R_a=2.0. This v2 package therefore separates
legacy-regression radius from current-default validation as described above. Final 13/13 native
execution of this exact package is still required before freezing B.3.
