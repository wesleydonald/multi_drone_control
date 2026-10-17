# Card: the 16 cm residual at LAND (rev 2 after the critic: diagnose first, then a no-floor descent)

Written Sat 17 Oct 2026, ~14:00 (Wesley ~13:55: "Keep A, card the residual", docs/decisions.md 17 Oct). **Rev 2
~14:10 after the critic (below).** Bars frozen before any run. Parent: `docs/experiments/2026-10-17_touchdown_height_gate.md`
(results R2088-R2108).

## Why

The parent card's arm A (re-arm the stall detector at ring touchdown) removes the 0.4 m disarm, but a drone stronger
than its thrust map still disarms in the air.

| arm (twin, A on) | runs | highest drone above rest at LANDED (cm) | drone tilt after LANDED (deg) |
|---|---|---|---|
| drone 1 offset 0.155 against its map's 0.185 | R2088-R2091, R2099-R2102 | 15.7-15.8 (all 8) | 23.6-29.9 |
| normal maps | R2092, R2103 | 4.6-4.7 (drone 0, every normal landing) | 2.3-2.7 |

## Step 0: what holds the drone up (offline, no code, before anything is built)

Two readings fit R2088. They need different fixes:
- **(a) A steady-state position error.** Once the ring rests, the tracker drops its cable term, and no integrator runs
  while grounded (`tracker_node.py:1365`; `kt_trim` off). The LAND reference stops at z = 0, so a drone stronger than
  its map hovers above it.
- **(b) The twin's rigid rod props it.** Every drone at LANDED sits about one rod length (0.55 m) from its plate.
  Drone 1 stops at 0.258 m, 8 mm above `LAND_FOLLOW_XY_Z` (0.25), at r 0.746 against the unwind's 0.775, with
  throttle just under its true hover. The rig's tethers go slack, so this would be a twin artefact.

Checks, from the existing logs (R2088-R2108, R2023/R2035):
- each drone's pivot-to-plate distance at LANDED, against the rod length;
- its xy against the reference, and the throttle against its true hover (map hover minus its sim offset error);
- drone 0's 4.7 cm in every normal landing (offset 0.181 against 0.185: almost no map error);
- a rod-free neighbour from the logs if one exists: a departed drone landing alone.

If (b) explains it, the twin cannot show the rig's residual. The card then goes back to Wesley with the rig estimate,
the steady-state error of (a) for a free drone, before any code.

**Step 0 result (14:01, R2088/R2091/R2092/R2023/R2096/R2105, the last 0.5 s before LANDED):**

| run | drone 0: z (m) / pivot-plate (m) / throttle | drone 1 | a drone idle on the floor |
|---|---|---|---|
| R2088 strong map | 0.147 / 0.553 / 0.454 | **0.258 / 0.557 / 0.427** | 0.100 / 0.551 / 0 |
| R2091 strong map | 0.147 / 0.553 / 0.453 | 0.256 / 0.557 / 0.427 | 0.100 / 0.551 / 0 |
| R2092, R2023, R2096 normal | 0.146-0.147 / 0.553 / 0.454 | 0.100-0.101 / 0.551 / 0 | |
| R2105 rig path (no unwind) | 0.100 / 0.551 / 0 | 0.101 / 0.550 / 0 | |

- **Drone 1 is reading (a).** Its rod is the most stretched (0.557 against the 0.551 a resting rod reads): the rod is in
  tension and holds it down, it does not prop it. The drone sits 0.26 m above its target on its own height error; the
  rig would show the same without the rod.
- **Drone 0's 4.7 cm is reading (b), a twin artefact.** Below `LAND_FOLLOW_XY_Z` its xy is frozen at r 0.771. A 0.553
  m rigid rod from a grounded plate cannot reach lower than 0.02 + sqrt(0.553^2 - 0.546^2) + 0.04 = 0.147 m there, which
  is where it sits. Without the unwind (R2105) it lands at rest. Harmless: it falls 4.7 cm onto its rod.
- So the change goes ahead. F1's per-drone bar keeps drone 0's 4.6-4.7 cm as its normal. A deeper reference could make
  drone 0 press on its pinned rod, so F3 watches it.

## The change (if Step 0 says (a)): one switch, default off (today's behaviour bit for bit)

`land_descend_to_stall` (bool, default false). After ring touchdown (`_land_hold_refs` only), the reference keeps
descending at `land_vel` with **no floor** until the stall detector freezes it (`_touched_down`). This is the critic's
simpler and stronger fix:
- no rest records are needed;
- it cannot stop in the air the way a rest-relative floor could (QUAD4's record is its in-air `attach_pos`);
- a strong drone follows with a constant lag and does not stall in the air, whatever its map error;
- a normal drone sees about 0.2 m of extra depth (1 s stall window x 0.2 m/s) before the freeze;
- the backstop is the existing max-drop exit (`lift_progress <= -LAND_MAX_DROP`).

**Both paths:**
- straight: drop `max(0.0, ...)`, and `vz` stays `-land_vel` until touchdown;
- unwind: `land_arc_ref` takes `floor=0.0` (the default keeps every existing call) and gets `-inf` from the switch,
  with the position and the velocity together.

**Logged:** each drone's reference z when `_touched_down` latches.

**Code:**
- `planner_node.py`;
- `bringup` (`real_mode.PLANNER_OPTIONAL`, `common.yaml`);
- no tracker change.

## Tests (unit, before any twin run)

- off: `land_arc_ref` and the straight descent give today's nodes exactly (fixed inputs, node by node);
- on: the reference keeps descending below z = 0 with `vz = -land_vel`, then freezes at `_touched_down`;
- unwind path: the arc, then straight down, below 0;
- a QUAD4-like record at 0.8 m has no effect (no rest is used).

## Twin runs (laptop, one at a time; A on in every arm, as it becomes the default)

| arm | config | n |
|---|---|---|
| T1 strong map, target shift (offset error 0.03) | `app_mapneg_ref` + A + switch | 2 |
| T1b dose: offset error 0.045 (sim offset 0.140 on drone 1) | new `app_mapneg45_ref` + A, and + A + switch | 1 + 2 |
| T2 strong map, model integral | `app_mapneg_model` + A + switch | 2 |
| T3 normal hover landing | `app_x0_hold_eps0` + A + switch | 2 |
| T4 floor-start lift and LAND on a path | `traj3_orbit_v125` + A + switch | 2 |
| T5 the rig LAND path (`land_unwind` false) | `app_x0_hold_eps0` + land_unwind false + A + switch | 2 |
| neighbour: weak map (offset 0.215) | `app_mappos_ref` + A + switch | 1 |
| neighbour: two survivors after a detach | `ta_a1_r1` + A + switch | 1 |
| neighbour: LAND in the creep | `twin_land_e1_creep` + A + switch | 1 |

Bases: the same arms with A only, R2088-R2108, plus the new T1b A-only run. F3 is also scored against the switch-off
bases (R2023/R2035, R2105).

## Bars (frozen now, rev 2)

| # | bar | pass | fail |
|---|---|---|---|
| F1 | the residual is gone | T1, T1b, T2: each drone's height above rest at LANDED within 2 cm of the same drone's in the normal landings (drone 0 4.6-4.7 cm, drones 1-2 at most 1.6); drone tilt after LANDED at most 12.2 deg (the parent's H1 bar), every run | any drone higher or any tilt larger |
| F2 | normal landings unchanged | T3, T4, T5, the neighbours: LANDED within 1.0 s of the A-only base's time after ring touchdown; the highest drone at LANDED within 2 cm of the base | outside either |
| F3 | no pressing on the floor | every arm: drone tilt while on the floor (ring down to LANDED, drone within 3 cm of rest) at most the A-only and switch-off bases' + 2 deg; no drone slides more than 3 cm in xy | larger |
| F4 | the fleet never hangs armed; no new faults | LANDED within 10 s of ring touchdown; no FLEET ABORT, ENVELOPE FAULT, DROP AND LAND or UNANNOUNCED DETACH the base did not have | any |

**Falsifier:** F1 fails on T1 or T1b (the strong drone still disarms high, or tips), or F2/F3/F4 fails.

**What it cannot say:**
- the twin's rods are rigid and the rig's tethers go slack, so the tilt after a drop is a twin number;
- two map errors on one drone.

## Results (17 Oct 14:45-17:37, laptop, tree 0a77fa1; R2138-R2147, R2158-R2163; bases R2088-R2108, R2133-R2137, R2139, R2164)

Scored per drone (`scratchpad f_score.py`: height above rest at LANDED; floor tilt and xy slide from ring down to LANDED
while the drone is within 3 cm of rest) and with `tools/land_touchdown.py` (tilt after LANDED).

| arm | base (A only): runs / LANDED - ring down (s) / per-drone height (cm) / tilt after (deg) | switch on: runs / s / cm / deg | ref depth (m) |
|---|---|---|---|
| T1 error 0.03 | R2088, R2099 / 4.38, 4.44 / 4.7, 15.8, 0.0 / 25.6, 26.3 | R2138, R2158 / 4.94, 5.04 / 0, 0, 0 / 0.1, 0.1 | 1.08 |
| T1b error 0.045 | R2139 / 4.32 / 4.7, 22.3, 0.0 / 90.4 | R2140, R2159 / 5.41, 5.42 / 0, 0, 0 / 0.0, 0.1 | 1.16 |
| T2 model integral | R2090, R2101 / 4.27, 4.41 / 4.8, 15.8, 0.0 / 25.2, 27.8 | R2141, R2160 / 5.13, 4.99 / 0, 0, 0 / 0.1, 0.0 | 1.08 |
| T3 hover | R2092, R2103, R2164 / 4.36, 4.35, 4.42 / 4.6-4.7, 0, 0 / 2.7, 2.3, 1.7 | R2142, R2161 / 4.55, 4.51 / 0, 0, 0 / 0.1, 0.1 | 0.94 |
| T4 orbit 0.125 | R2093, R2104 / 4.28, 4.31 / 4.8-4.9, 0, 0 / 1.9, 2.5 | R2143, R2162 / 4.47, 4.50 / 0, 0, 0 / 0.0, 0.1 | 0.94 |
| T5 rig LAND path | A R2095, R2106, R2134, R2136, R2137 / 4.54-5.14 / 0.9-4.6 / 3.1-11.2; off R2105, R2133 / 5.33, 5.05 / 0.1 / 1.5, 2.3 | R2144, R2163 / 4.15, 4.26 / 0, 0, 0 / 0.4, 1.0 | 0.86, 0.88 |
| weak map | R2096, R2107 / 4.45, 4.33 / 4.7, 0, 0 / 0.9, 1.9 | R2145 / 4.50 / 0, 0, 0 / 0.2 | 0.94 |
| detach | R2097, R2108 / 4.49, 4.36 / 4.3-4.8 / 1.7, 2.3 | R2146 / 4.49 / 0 x4 / 0.2 | 0.94 |
| creep | R2098, R2165 / 3.45, 3.38 / 4.5-4.7 / 0.8, 4.1 | R2147 / 3.48 / 0 x4 / 0.2 | 0.68 |

Floor tilt max / slide max (switch on): 0.4-1.1 deg / 0.2-0.3 cm off the rig path; 6.2, 6.9 deg / 0.9, 1.4 cm on it
(bases there: off 7.2-8.6 deg / 0.6-1.1 cm, A 4.3-11.6 deg / 0.5-1.3 cm).

| bar | result |
|---|---|
| F1 | **pass.** T1, T1b, T2 (6 runs): every drone at rest at LANDED; tilt after 0.0-0.1 deg |
| F2 | **fails as written** on the height clause: timing passes (T3, T4 +0.2 s; T5 0.3-0.9 s sooner than A; the neighbours within 0.2 s; T1/T1b/T2, not under F2, take 0.6-1.1 s longer), but drone 0 lands at rest (0.0) where the base has 4.3-4.9 cm, which is outside the two-sided 2 cm. The reference no longer stops at z 0, so drone 0 reaches the floor (r 0.775, z 0.100) before its xy freezes, instead of hanging on its rigid rod at 0.147 m (Step 0 (b)). T5 passes (A bases 0.9-4.6) |
| F3 | **pass.** Floor tilt within the bases + 2 deg on every arm, slide at most 1.4 cm |
| F4 | **pass.** LANDED 3.5-5.4 s after ring touchdown; no fault the base did not have (the detach's UNANNOUNCED DETACH is the scenario). The runner FAILs on the map arms are its in-flight ring-tilt check (9.0-12.7 deg), as in the A bases |

**Verdict against the frozen bars: the falsifier fires (F2).** It fires because drone 0 now lands lower, at its rest.
Re-scoring with a one-sided height bar would break the rule (bars frozen, runs seen), so a default needs Wesley's word on
the bar and fresh runs scored against it.

Notes:
- the logged reference z ends at -0.34..-0.36 m (T1) and -0.42..-0.44 m (T1b), up to 0.53 m below a drone's rest
  (0.10). Every drone's throttle was already 0 there (the tracker's ground idle), so nothing pressed and the R0997
  tip-over did not appear; in the twin ground idle is instant, so this is not a rig result;
- **contact speed (no bar covers it):** a drone meets the floor at 0.17-0.23 m/s with the switch against 0.04-0.14 in
  the bases, 0.29-0.35 m/s on the rig LAND path (R2144, R2163);
- the dose arm: A alone leaves drone 1 at 15.8 cm (error 0.03) and 22.3 cm (0.045), and it tips over at 0.045; with the
  switch both are 0.

## Reviewer (17 Oct ~18:00): (a) supported, (b) not supported

1. Every number reproduces (repeats within 0.1 cm / 0.1 deg); the reference depth was wrong (fixed above).
2. (a) drone 1 at 0.0 cm in all six T1/T1b/T2 runs against 15.8 / 22.3 cm; tilt after 0.0-0.1 against 25.2-90.4; each
   config differs from its A-only one in the switch only.
3. F2 fires as written. F1's pass text is two-sided per drone too, so drone 0 at 0.0 also misses it; F1 passes only on its
   one-sided fail column. T5's final-tree A bases disagree (R2134 1.0 cm, R2136/R2137 4.6).
4. Controls: the T3, T4, weak-map, detach and creep bases are A+B on b7ab074; T4, weak map and detach have no A-only base
   on the final tree (switch off, the flight code differs only by B's removal).
5. Floor: no pressing only because ground idle cuts the throttle; contact speed rises to 0.17-0.23 m/s (0.29-0.35 on the
   rig path), uncovered by any bar. Must not say: drone 0 reaching rest, or "no R0997 tip", as rig results.

## Rig

Only after the twin, the reviewer and Wesley's word. It goes in through a `configs/rig/` params file, with A.

## Critic

Critic, 17 Oct ~13:58 (rev 1). Verdict: **rewrite the card.** Rev 2 above:
- must-fix 1 (never raise the floor): the floor is gone; there is no rest record to misuse;
- must-fix 2 (vz with the floor): both paths change position and velocity together;
- must-fix 3: dose arm T1b;
- must-fix 4 (drone 0's 4.7 cm): Step 0;
- must-fix 5 (QUAD4 record): test;
- should-fix 1-2: F1 per drone, F3 against the switch-off base too;
- should-fix 3: logged reference at the freeze;
- should-fix 4 (the rigid rod at 0.258 m, 8 mm above LAND_FOLLOW_XY_Z): Step 0 (b);
- should-fix 5 (no-record fallback): moot.

The critic's text, as written:

1. **Tried before?** Not as such, but the card sits between two known failures.
   - R0852: a LAND reference that stops above where the drone can get to reads as a stall. A fixed floor is that same
     failure, made deeper.
   - R0997 (decisions 2026-10-04): a drone tracking 10 cm below the floor tipped over. That is why `ground_idle`
     exists.
2. **Simplest explanation not addressed.** It is a steady-state offset. It is not the tracker clipping the reference
   (u2 0.22 is above the 0.05 floor), and not the planner's integral (`_land_hold_refs` has no integral term).
   - The tracker pulls throttle toward the map's hover through its cost reference (acados.py:340).
   - No integrator acts while grounded (tracker_node.py:1365); `kt_trim` is off.
   - The real defect is that the reference stops. Any fixed depth is tuned to one map error: 0.30 against the 0.26
     needed leaves 4 cm of margin.
   - Drone 0 has a correct map yet sits 4.7 cm above rest in every normal landing; unexplained, so the 0.26 m per
     0.03 ratio is not established.
   - **Simpler and stronger fix:** no floor after ring touchdown; keep descending at land_vel until the stall freezes
     the reference; the max-drop exit is the backstop.
3. **Bars:** one map error at one depth has no dose-response. F1's 5 cm is 3 mm above drone 0's normal residual. F3's
   A-only base already ends T5 at 10-11 deg against 1.5 with A off.
4. **What it breaks:**
   - QUAD4: its rest record is `attach_pos` z (dissipative_node.py:2844-2848). An in-air attach point above 0.30 m
     puts `rest - 0.30` above 0: R0852 on the rig. A raised mat above 0.30 the same.
   - `mpc_planner` never sets `_rest_z`; the subclass sets it only with `departed_ground_idle` on.
   - Departed drones are untouched. `ground_idle`: drone 0 at 4.7 cm never meets its 3 cm margin today.
5. **Asked for:** yes.

Must-fix:
1. never raise the floor, or fly the no-floor variant;
2. line 1103's `vz` and `land_arc_ref` 209-210 use the floor;
3. a dose arm and a no-floor arm;
4. explain drone 0's 4.7 cm before using the 0.26/0.03 ratio;
5. a test for a QUAD4 record at 0.8 m.

Should-fix:
1. F1 per drone, against the normal residual;
2. F3 against the switch-off base;
3. log each drone's floor; name each arm's planner node and rest records;
4. drone 1 at 0.258, 8 mm above LAND_FOLLOW_XY_Z 0.25: check its xy at the stall (the rigid rod can carry load);
5. the no-record fallback depends on the mocap origin on the rig.
