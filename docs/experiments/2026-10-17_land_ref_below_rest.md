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

## Reviewer (17 Oct ~17:45): (a) supported, (b) not supported

1. Every number reproduces (repeats within 0.1 cm / 0.1 deg); the reference depth was wrong (fixed above).
2. (a) drone 1 at 0.0 cm in all six T1/T1b/T2 runs against 15.8 / 22.3 cm; tilt after 0.0-0.1 against 25.2-90.4; each
   config differs from its A-only one in the switch only.
3. F2 fires as written. F1's pass text is two-sided per drone too, so drone 0 at 0.0 also misses it; F1 passes only on its
   one-sided fail column. T5's final-tree A bases disagree (R2134 1.0 cm, R2136/R2137 4.6).
4. Controls: the T3, T4, weak-map, detach and creep bases are A+B on b7ab074; T4, weak map and detach have no A-only base
   on the final tree (switch off, the flight code differs only by B's removal).
5. Floor: no pressing only because ground idle cuts the throttle; contact speed rises to 0.17-0.23 m/s (0.29-0.35 on the
   rig path), uncovered by any bar. Must not say: drone 0 reaching rest, or "no R0997 tip", as rig results.

## Rev 3 (Wesley 17 Oct ~19:45: "re-card and re-fly"; written ~20:00, bars frozen before the rev-3 runs)

Rev 2's runs (R2138-R2163) stay scored against rev 2's bars (F2 fired). These bars apply only to new runs.
No code change: the switch is the same. What changed: the height bars are one-sided (a drone landing LOWER, at its
rest, is the aim), a contact-speed bar is added (the reviewer: no bar covered it), the bases are A-only on the same
tree (A is now the twin default), and QUAD4 folded in gets an arm (the A critic: with this switch the reference sinks
1.5 m below the anchors if the stall never fires).

**Contact speed, the rig's own range:** each drone's mean descent speed over the 0.2 s before it comes within 3 cm of
its rest, after LAND. Rig landings 14 Oct: t5_4 0.14-0.18, t5_5 0.17-0.29, t6f 0.10-0.38 m/s. Twin, switch off:
0.02-0.13 (0.79 for the strong drone that dropped, R2088). Rev 2 with the switch: 0.18-0.20, rig LAND path 0.29-0.32.

| # | bar (rev 3) | pass | fail |
|---|---|---|---|
| F1 | residual gone | T1, T1b, T2: each drone's height above rest at LANDED at most its normal-landing value + 2 cm (drone 0 6.7, drones 1-2 3.6 cm); tilt after LANDED at most 12.2 deg | any drone higher, any tilt larger |
| F2 | normal landings unchanged | every other arm: LANDED within 1.0 s of its same-tree A-only base's time after ring touchdown; the highest drone at LANDED at most the base's + 2 cm | slower by more than 1.0 s, faster by more than 1.0 s, or higher |
| F3 | no pressing | as rev 2 (floor tilt at most the bases' + 2 deg; slide at most 3 cm) | as rev 2 |
| F4 | never hangs, no new faults | as rev 2, plus: the exit is the stall in every arm (never the max drop) | any |
| F5 | contact speed | every drone at most 0.40 m/s (the rig's landings reach 0.38) | any drone faster |

**Runs (laptop, one at a time; same tree; switch run then its A-only base, ABAB where n = 2):**

| arm | switch config | A-only base | n |
|---|---|---|---|
| T1 error 0.03 | `td_nf_mapneg_ref` | `td_a_mapneg_ref` | 2 + 1 |
| T1b error 0.045 | `td_nf_mapneg45_ref` | `td_a_mapneg45_ref` | 2 + 1 |
| T2 model integral | `td_nf_mapneg_model` | `td_a_mapneg_model` | 2 + 1 |
| T3 hover | `td_nf_hold` | `td_a_hold` | 2 + 1 |
| T4 orbit 0.125 | `td_nf_orbit_v125` | `traj3_orbit_v125` (A by default) | 2 + 1 |
| T5 rig LAND path | `td_nf_hold_unwind0` | `td_a_hold_unwind0` | 2 + 1 |
| weak map | `td_nf_mappos_ref` | `td_a_mappos_ref` (new) | 1 + 1 |
| detach | `td_nf_detach` | `ta_a1_r1` | 1 + 1 |
| creep | `td_nf_creep` | `td_a_creep` | 1 + 1 |
| QUAD4 folded in (combined M1) | `td_nf_partner_attached_orbit` (new) | `partner_attached_orbit` | 2 + 2 (the A critic's re-fly) |

**Falsifier:** F1 fails on T1 or T1b, or any of F2-F5 fails.

## Rev 3 results (17 Oct 21:04-23:36, laptop, flight code 9d30665; R2174-R2200)

Scored per drone with `f_score.py` / `contact.py` (scratchpad) and `tools/land_touchdown.py`, as rev 2. Bases are the
A-only arms on the same tree.

| arm | base (A only): run / LANDED - ring down (s) / height at LANDED (cm) / tilt after (deg) / contact (m/s) | switch: runs / s / cm / tilt after / contact | exit |
|---|---|---|---|---|
| T1 error 0.03 | R2175 / 4.33 / 4.7, 15.8, 0.0 / 29.2 / 0.17, 0.74, 0.09 | R2174, R2194 / 5.03, 5.05 / 0, 0, 0 / 0.1, 0.1 / 0.18-0.21 | stall |
| T1b error 0.045 | R2177 / 4.29 / 4.7, 22.3, 0.0 / 90.0 (drone 1 tips over) / 0.00, 14.9 (falls), 0.07 | R2176, R2195 / 5.37, 5.42 / 0, 0, 0 / 0.1, 0.1 / 0.17-0.20 | stall |
| T2 model integral | R2179 / 4.27 / 4.7, 15.9, 0.0 / 28.4 / 0.14, 0.70, 0.08 | R2178, R2196 / 4.98, 5.00 / 0, 0, 0 / 0.0, 0.0 / 0.18-0.21 | stall |
| T3 hover | R2181 / 4.31 / 4.7, 0, 0 / 1.9 / 0.01-0.12 | R2180, R2197 / 4.48, 4.54 / 0, 0, 0 / 0.0, 0.1 / 0.18-0.20 | stall |
| T4 orbit 0.125 | R2183 / 4.42 / 4.8, 0, 0 / 2.3 / 0.02-0.11 | R2182, R2198 / 4.52, 4.56 / 0, 0, 0 / 0.0, 0.0 / 0.18-0.20 | stall |
| T5 rig LAND path | R2185 / 4.59 / 4.5, 0, 0.1 / 4.5 / 0.03-0.11 | R2184, R2199 / 4.20, 4.45 / 0, 0, 0 / 0.7, 0.4 / 0.28-0.31 | stall |
| weak map | R2187 / 4.31 / 4.7, 0, 0 / 2.0 / 0.08-0.22 | R2186 / 4.49 / 0, 0, 0 / 0.1 / 0.18-0.21 | stall |
| detach | R2189 / 4.39 / 5.1, 0, 0, 0.1 / 2.4 / 0.06-0.12 | R2188 / 4.54 / 0 x4 / 0.3 / 0.17-0.18 | stall |
| creep | R2191 / 3.47 / 4.7, 0, 0, 0 / 1.2 / 0.04-0.09 | R2190 / 3.39 / 0 x4 / 0.2 / 0.18-0.20 | stall |
| QUAD4 folded in (after his rejoin) | R2193 / 2.58 / 0, 0, 0, QUAD4 4.4 / 11.8 / 0.17-0.18 | R2192, R2200 / 2.40, 2.63 / 0, 0, 0, QUAD4 3.7, 2.8 / 16.1, 10.0 / 0.17-0.27 | stall |

Floor tilt max / xy slide max with the switch: 0.3-1.4 deg / 0.1-0.5 cm off the rig path; rig path R2184 5.3 deg / 0.9
cm, R2199 7.1 deg (drone 0) / 1.1 cm. Bases there: A-only R2185 4.5 deg (drone 0 never comes within 3 cm of its rest,
so it has no floor-tilt reading); switch-off R2105, R2133 7.2-8.6 deg.

| bar | result |
|---|---|
| F1 | **pass.** T1, T1b, T2 (6 runs): every drone at its rest at LANDED; tilt after 0.0-0.1 deg (bases: drone 1 at 15.8-22.3 cm, 28-90 deg) |
| F2 | **pass.** Every other arm lands within 0.4 s of its base (-0.39..+0.23 s; T5 0.14-0.39 s sooner); the highest drone at LANDED 0 cm (bases 4.5-5.1), QUAD4 3.7 / 2.8 cm against 4.4 |
| F3 | **FAILS as written** (reviewer): the bar is the A-only AND switch-off bases + 2 deg, and rev 3's A-only base is the same-tree run R2185 (4.5 deg). R2199's drone 0 at 7.1 deg is over 6.5 (R2184's 5.2/5.3 passes). It is under the switch-off bases + 2 (R2105/R2133 7.2-8.6), and in R2185 drone 0 never reached the floor, so the comparison is drone 0 on the floor against drones 1-2. Slide at most 1.1 cm. QUAD4: no reading and no base (in R2192 and R2193 it never comes within 3 cm of its rest); only R2200 reads, 10.0 deg |
| F4 | **pass.** Exit = stall in all 17 switch runs; LANDED 2.4-5.4 s after ring touchdown; no fault the base lacked (the detach arm's UNANNOUNCED DETACH is the scenario). The runner's FAILs on the map arms are its in-flight ring-tilt check (9.2-9.4 deg on T1/T2, 12.3-12.9 on T1b, > 5), as in the bases; R2194 also flags drone 0's mocap stale 2.23 s at t 70.79, after LANDED (68.51) |
| F5 | **pass for the carriers:** every carrier at most 0.31 m/s (T5 0.28-0.31); QUAD4 has one reading (R2200 0.27; R2192 none). Under the bar, but 0.17-0.21 m/s against 0.01-0.12 in the bases off the rig path and 3-9x the base on it (0.28-0.31 against 0.03-0.11) |

**Verdict against the frozen rev 3 bars: the falsifier fires (F3, R2199 on the rig LAND path, 0.6 deg over).** F1 (the
residual) passes in all 6 map runs; F2, F4 and the carriers' F5 pass. R2193's exit line is missing from its launch log
(not flushed), so the A-only QUAD4 base has no exit reading.

Not rig results (the twin): nothing presses on the floor only because the twin's ground idle cuts the throttle at once;
"drone 0 reaches its rest" and "no R0997 tip" are twin numbers; drone 0's 7-9 deg once on the floor (switch and switch-off
runs alike) is the rigid rod's.

## Reviewer (18 Oct ~06:28): not supported

F3 fails as written on R2199 (same-tree A base, the bar's AND); the earlier reading dropped the A clause. QUAD4's F3/F5
are no reading / no base, not passes. Corrected above: the 17-run count, the runner tilt peaks, R2194's post-LANDED mocap
flag, the contact-speed rise, and the ground-idle caveat carried over from rev 2. Correction rows R2192c, R2199c, R2200c.

**Also (18 Oct ~06:30):** the QUAD4 arm (R2192, R2200 vs R2193) flew the legacy profile (an old-worlds SDF), where A is
not set: A was OFF in all three, so that arm compared the switch alone against no switches, not against A. Rows R2192c2,
R2193c, R2200c2. The stall still fired with QUAD4 folded in in both switch runs.
The runs are not re-scored. A default needs Wesley's word: a new bar (drone 0 on the floor has no A-only reading) and
fresh runs.

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

## Rev 4 (5 Nov; written ~13:05 before the runs; simulation bars written first, not frozen: Wesley 2 Nov)

Rev 3's runs stay scored against rev 3's bars (F3 fired; the switch stays off). Rev 4 adds one variable on top of the
switch and re-flies the baselines on today's tree: every rev-3 run flew the 0.55 m rods and the old twin link; the
twin's links changed on 4 Nov ~23:15 (rod 0.002 + magnet 0.038 kg at the plate, airframe 0.510) and fly 1 m.

**Why now (rig, 5 Nov, RIG-1105-k1d).** After LAND the three carriers stopped 15-23 cm above z 0 (their rests are
4.4-6.7 cm, so 10-17 cm above rest) and hovered for ~6 s (each carrier 0.48-0.61 m in xy from ring touchdown to the end of the log), until QUAD4 (taken back, landed
alone) was down. Their LAND reference stopped at z 0 (57.2 s) while they were 0.17-0.23 m up and lagging it by 0.13-0.20
m; ground effect and the tracker's weak position stiffness held them there. Rev 3's switch removes that stop, but its
drones met the floor at the reference's own speed (0.17-0.21 m/s, 0.28-0.31 on the rig LAND path, against 0.01-0.12
off): the reason it stayed off.

**The variable: a slow final descent under the switch.** After ring touchdown (`_land_hold_refs`), the reference
moves at `land_vel` until it passes `land_slow_z` (world z of the reference), then at `land_slow_vel`, down to the
stall as before. Both paths: the straight descent (the rig's) and the unwind (the arc is slowed along its own path,
position and velocity together, from where it passes `land_slow_z`). Values from the data:
- `land_slow_vel` **0.10 m/s**: with the switch the drones meet the floor at the reference speed (rev 3: 0.17-0.21 at
  0.20), so 0.10 aims at the bases' upper end (0.01-0.12) and the rig's softest landings of 14 Oct (0.10-0.38). Half
  of `land_vel` also keeps today's stall fraction (0.25) twice away even without the fix below.
- `land_slow_z` **0.40 m**: the drones lag the reference by 0.20-0.24 m at 0.20 m/s on the twin's rig LAND path (R2184,
  R2185, 1.5-3 s after ring touchdown; 0.10-0.13 on the unwind, R2180) and by 0.13-0.20 m on the rig (k1d). The slowing
  must start that lag plus ~0.1 m of settling above the rest: 0.30 m above the twin's 0.10 m rest, 0.33-0.36 m above
  the rig's. Cost: the last 0.30 m at half speed, +1.5 s per LAND.

**Two things the variable must not break (built with it):**
- **The stall test (the trap).** `TouchdownDetector` expects each tick's drop at `land_vel`; a reference slowed under
  `LAND_STALL_FRAC` x `land_vel` reads as a stall in the air. It now takes the speed commanded (the slowest slot's
  node-0 speed; None = `land_vel`, as before).
- **The max-drop backstop.** It is a clock on `lift_progress` at `land_vel`: on 1 m links the carriers start ~0.9 m
  up at ring touchdown, and the slowed tail would put the stall after it (LAND to max drop ~12.4 s from a 1 m hover).
  While slowed, that clock runs at the commanded speed, so it stays a depth.

**Switches** (planner; `common.yaml`, `real_mode.PLANNER_OPTIONAL`): `land_slow_vel` (m/s, default 0 = off: today's
nodes exactly), `land_slow_z` (m, default 0.40; used only when `land_slow_vel` > 0). Refused (after the critic, at
the launch and by the planner): `land_slow_vel` above `land_vel`, below 0.05 m/s (at 0.01 the stall test's drop per
tick is 0.25 mm, inside the mocap noise, and the backstop would run ~150 s), and without `land_descend_to_stall` (never
flown). Logged: each drone's reference z at the touchdown line. Tests `src/mpc_planner/test/test_land_slow.py` (12)
and `test_real_mode_launches.py::test_land_slow_vel_reaches_the_planner_only_under_descend_to_stall`: off = the
committed function node for node (a frozen copy of the pre-switch `_land_hold_refs` and `TouchdownDetector.update` in
the test; both paths, with and without floor, through the xy follow and the touchdown freeze); fast then slow
below the zone; with the floor it stops at 0; inside the zone from the start; the slowest slot sets the commanded speed;
frozen at touchdown; the arc slowed on its circle; the trap (today's detector fires in the air at 0.04 m/s; with the
commanded speed only on the floor); `_touchdown_stalled` passes it.

**Arms and scenarios** (twin, lab PC, today's tree and worlds, `land_rearm_touchdown` on as the twin default in every
arm; exploratory: one run per arm and scenario, Wesley 30 Oct):

| scenario | base config (today's world) | A: today (switch off) | B: switch on (rev 3) | C: switch + slow (0.10 below 0.40) |
|---|---|---|---|---|
| S1 hover LAND, the rig LAND path (`land_unwind` false) | `b3_hover_L100_slope` | `lr4a_hold_u0` | `lr4b_hold_u0` | `lr4c_hold_u0` |
| S2 QUAD4 folded in: released on 9, his re-attach, LAND 10 s after the re-weld; rig LAND path | `nr_hover_L100_p9` | `lr4a_nr9_u0` | `lr4b_nr9_u0` | `lr4c_nr9_u0` |
| S3 orbit 0.125 m/s, LAND on the twin's unwind | `traj3_L100_orbit_v125` | `lr4a_orbit` | `lr4b_orbit` | `lr4c_orbit` |
| added ~13:20, after S1/S2 (below): S1u the hover on the twin's unwind | `b3_hover_L100_slope` | `lr4a_hold` | `lr4b_hold` | `lr4c_hold` |
| added ~13:20: S4 drone 1 stronger than its map by 0.03 (sim offset 0.155 against 0.185), rig LAND path | `b3_hover_L100_slope` | `lr4a_strong_u0` | `lr4b_strong_u0` | `lr4c_strong_u0` |
| added ~13:20: S4u the same on the unwind | `b3_hover_L100_slope` | `lr4a_strong` | `lr4b_strong` | `lr4c_strong` |

**Scored per drone** (rest = its median z before ARM; ring touchdown = first sample after LAND within 5 mm of the
ring's lowest): height above rest at the stall line and at LANDED; contact speed (mean descent speed over the 0.2 s
before it first comes within 3 cm of its rest, after LAND); floor tilt (ring down to LANDED, within 3 cm of rest); xy
drift from ring touchdown to LANDED; ring touchdown to stall and to LANDED; the exit (stall / max drop); any tip (tilt
after LANDED); faults.

**Bars (rev 4; written before the runs; anything judged after the runs is marked so):**

| # | bar | pass | fail |
|---|---|---|---|
| G1 | no touchdown in the air (one-sided height) | B, C: every drone at most 3 cm above its rest at the stall line and at LANDED (lower is the aim) | any drone higher |
| G2 | height not worse than today | B, C: each drone's height above rest at LANDED at most A's same drone + 2 cm | higher |
| G3 | contact speed | C: every drone at most 0.13 m/s (0.10 + 0.03); B and A: at most 0.40 (rev 3's F5) | faster |
| G4 | time | B: LANDED - ring touchdown within 1.0 s of A's (rev 3's F2); C: at most A's + 2.5 s (the slowing costs ~1.5 s by design) and never more than 1.0 s faster | outside |
| G5 | floor | B, C: floor tilt at most A's + 2 deg; xy slide on the floor at most 3 cm (rev 3's F3, against the same-scenario A run) | larger |
| G6 | exit and faults | B, C: exit = stall (never the max drop); LANDED within 10 s of ring touchdown; no FLEET ABORT, ENVELOPE FAULT, DROP AND LAND or UNANNOUNCED DETACH that A lacks; no tip | any |

**Falsifier (C):** G1, G3 or G6 fails on any scenario, or C lands a drone higher than B.

**What it cannot say:**
- the twin has no ground effect, so A's rig hover cannot appear in it: the twin shows that B and C reach the floor
  and how fast; whether C's shallower freeze (`land_slow_vel` x 1 s + its lag, ~0.2 m below the drone against ~0.4 m
  for B) still pushes a carrier out of ground effect is a rig question;
- the twin's links are rigid rods (on the rig LAND path a rod props or pushes a drone; rev 3's F3 failure was drone 0
  on its rod), and its ground idle cuts the throttle at once;
- xy drift after ring touchdown (the rig's 0.3-0.6 m) and the planner's wait for a taken-back QUAD4
  (`LAND_DEPARTED_WAIT_S`) are not this variable; they are scored, not judged.

### Rev 4 results

5 Nov 13:05-13:32, lab PC (4 slots x 6 cpus), week8 9f20f8b + uncommitted (the switches above); R3716-R3724 synced
13:05, R3725-R3733 synced 13:20 (the main chat's runner edits of 13:05-13:20 went with them; the three arms of each
scenario always flew together). Scorer: scratchpad `lr4_score.py` (definitions under Scored above; QUAD4's contact
against where it ends, 2-3 cm over its start). Every runner verdict PASS; no FLEET ABORT, ENVELOPE FAULT, DROP AND LAND or
UNANNOUNCED DETACH in any run. Numbers judged after the runs.

| scenario | arm | run | exit | LAND to stall (s) | ring down to LANDED (s) | height at LANDED, per drone (cm) | contact (m/s) | floor tilt max (deg) / slide max (cm) | ref z at stall (m) |
|---|---|---|---|---|---|---|---|---|---|
| S1 hover, rig path | A | R3719 | stall | 11.8 | 6.83 | 1.2, 0.1, 0.1 | 0.24-0.32 | 14.1 / 2.0 | 0.00 |
| | B | R3720 | stall | 11.1 | 6.13 | 0, 0.1, 0.1 | 0.37-0.40 | 7.3 / 2.0 | -0.28 |
| | C | R3721 | stall | 13.4 | 8.41 | 0, 0.1, 0 | 0.30-0.34 | 11.9 / 3.1 | -0.17 |
| S2 QUAD4 folded in, rig path | A | R3716 | stall | 11.5 | 6.51 | 0.2, 0.1, 0.1, Q4 2.4 | 0.28-0.30, Q4 0.31 | 12.4 / 8.5 (Q4) | 0.00 |
| | B | R3717 | stall | 11.0 | 6.02 | 0.1, 0, 0, Q4 2.9 | 0.38-0.39, Q4 0.60 | 11.9 / 1.5 | -0.27..-0.22 |
| | C | R3718 | stall | 13.0 | 8.00 | 0.1 x3, Q4 2.9 | 0.27-0.31, Q4 0.62 | 12.4 / 1.6 | -0.13..-0.11 |
| S4 strong drone 1, rig path | A | R3725 | stall | 11.0 | 6.05 | 1.2, **26.6**, 0.1 | 0.29-0.31; drone 1 falls 1.25 at disarm, **tips (90 deg)** | 14.6 / 3.0 | 0.00 |
| | B | R3726 | stall | 12.0 | 7.04 | 0, 0.1, 0.1 | 0.38-0.40 | 7.0 / 2.7 | -0.46 |
| | C | R3727 | stall | 14.5 | 9.53 | 0, 0, 0 | 0.29-0.31 | 11.3 / 2.2 | -0.28 |
| S3 orbit, unwind | A | R3722 | stall | 11.9 | 6.89 | 0, 0, 0 | 0.12-0.17 | 0.5 / 0.1 | 0.00 |
| | B | R3723 | stall | 11.7 | 6.69 | 0, 0, 0 | 0.20 | 0.7 / 0.2 | -0.21 |
| | C | R3724 | stall | 13.7 | 8.69 | 0, 0, 0 | **0.10** | 0.4 / 0.2 | -0.10 |
| S1u hover, unwind | A | R3731 | stall | 11.9 | 6.93 | 0, 0, 0 | 0.12-0.18 | 0.4 / 0.1 | 0.00 |
| | B | R3732 | stall | 11.7 | 6.69 | 0, 0, 0 | 0.20 | 0.2 / 0.1 | -0.21 |
| | C | R3733 | stall | 13.7 | 8.70 | 0, 0, 0 | **0.10** | 0.4 / 0.1 | -0.10 |
| S4u strong drone 1, unwind | A | R3728 | stall | 11.8 | 6.75 | 0, **12.4**, 0 | 0.12-0.18; drone 1 falls 0.58 at disarm | 0.5 / 0.1 | 0.00 |
| | B | R3729 | **max drop** | 12.2 | 7.16 | 0, 0, 0 | 0.19-0.20 | 0.6 / 0.1 | -0.31 |
| | C | R3730 | stall | 14.8 | 9.77 | 0, 0, 0 | **0.10** | 0.3 / 0.1 | -0.21 |

xy from ring touchdown to LANDED: 45-65 cm in every run and arm. On the unwind it is the arc by design; on the rig path
the rigid rods push the drones out the same way (r 0.74 -> 1.22 m). Neither is the rig's drift (slack cables, inward).

| bar | result (judged after the runs) |
|---|---|
| G1 | **pass**, B and C, all 6 scenarios: every carrier within 0.1 cm of its rest at the stall and at LANDED, QUAD4 2.9 cm over its start (A 2.4). A fails it where a drone is stronger than its map (S4 26.6, S4u 12.4 cm) |
| G2 | **pass**, B and C everywhere |
| G3 | **C: pass on the unwind (0.10 m/s in all 9 carrier landings), FAILS on the rig path** (S1 0.30-0.34, S2 0.27-0.31 and QUAD4 0.62, S4 0.29-0.31). **B fails on S2** (QUAD4 0.60 > 0.40). A fails on S4 (drone 1 falls 1.25 m/s at its disarm in the air) |
| G4 | B passes (-0.70..+0.99 s against A). C passes on S1, S2, S3, S1u (+1.49..+1.80 s); **fails as written on S4 and S4u** (+3.48, +3.02 s), where A's LANDED came from a touchdown declared with drone 1 still 12-27 cm up (C against B there: +2.49, +2.61) |
| G5 | pass on the run maxima (rev 3's reading). Per drone, S1 C's drone 1 (11.9 deg) is over A's drone 1 + 2 (11.1), and its slide 3.1 cm **fails** the 3 cm by 1 mm (rig path) |
| G6 | C: **pass** (stall exit in all 6, LANDED 8.0-9.8 s after ring touchdown, no fault, no tip). **B fails on S4u: the max drop fired before the stall** (R3729, 12.2 s after LAND); in R3726 its max-drop clock ended 0.03 m short of the floor |

**Verdict: the falsifier fires as written (G3 on the rig-path scenarios).** Corrected after the critic (point 1): on the
twin's rig LAND path each carrier is **propped**, not dropped by its rod alone. Above `LAND_FOLLOW_XY_Z` (0.25 m) its xy
reference is still the anchor, 0.45-0.48 m inboard; the rigid rod pushed against that inward hold leans it 8.6-10 deg
inward and holds it 0.21-0.26 m over its z reference (0.30-0.38 for the strong drone). At 0.25 m the xy follow engages
(planner_node.py, `_land_hold_refs`), the drone levels, the prop goes, and it falls the last ~0.15 m. The contact speed
is then the speed it had at the crossing plus that fall:

| arm (rig path, R3716-R3721, R3725-R3727; carriers) | z over its reference at 0.25 m (m) | xy off its reference / lean | vz at 0.25 m (m/s) | contact (m/s) | 0.25 m to contact (s) |
|---|---|---|---|---|---|
| A | 0.23-0.25 | 0.45-0.48 m / 8.6-9.4 deg | -0.09..-0.18 | 0.24-0.32 | 0.5-0.6 |
| B | 0.22-0.26 (strong drone 0.38) | 0.45-0.48 m / 8.8-10.0 deg | -0.22..-0.24 | 0.37-0.40 | 0.3-0.4 |
| C | 0.21-0.24 (strong drone 0.37) | 0.45-0.48 m / 8.6-9.8 deg | -0.11..-0.12 | 0.28-0.34 | 0.5-0.6 |
| on the unwind, all arms (R3722-R3724, R3728-R3733) | 0.08-0.12 (strong 0.22-0.23) | 0.01-0.02 m / 0-0.3 deg | the arm's speed | A 0.12-0.18, B 0.19-0.20, C 0.10 | A/B 0.6-0.7, C 1.2 |

So the gap at the crossing is the same in every arm (the prop) and C lowers the contact by its lower incoming speed
(0.30 against B's 0.39), not to 0.10. The rig's cables go slack near the floor: at their 0.25 m crossing on 5 Nov the
carriers were 0.01-0.15 m off their xy reference and leaning 0.2-4.6 deg, so no prop. Where the reference sets the
descent (the unwind: S3, S1u, S4u) C does what it was built for: 0.10 m/s at the floor in all 9 carrier landings (B
0.19-0.20, A 0.12-0.18), every drone at its rest, the stall exit every time, for +1.8 s (+3.0 s with the strong drone).

**Findings for the rig (not this variable alone):**
- **A carrier stronger than its map (S4), corrected after the critic (point 2).** A on the rig path stops drone 1 26.6 cm
  over its rest, disarms it in the air and it tips over (R3725), but most of that is the prop: at 62.80 s it sits at z
  0.368, above the follow height, 0.44 m outboard of its anchor, leaning 9.2 deg inward. The map error alone is S4u's
  12.4 cm (R3728, no prop), inside the 9-16 cm Wesley accepted at LANDED on 10 Oct (decisions.md, "Landing drop"). On the
  rig (k1d) the carriers rode 0.13-0.20 m over their reference already in the air (53.9 s: z 0.83 against 0.67) and
  0.15-0.23 m over it once it stopped at z 0. What was new on the rig was not the height but **6.5-8.6 s of hover
  below 0.25 m with 0.2-1.0 m of xy drift** while the planner waited for QUAD4 (next section). B and C land every drone
  at its rest in S4 and S4u.
- **The max-drop backstop is tight on 1 m links.** It is a clock: ~12.4 s from a 1 m hover at 0.20 m/s. Today's stall
  comes 11.0-11.9 s after LAND (`lift_progress` ends at -1.27..-1.43 of -1.50). B with a strong drone reached it once
  (R3729) and missed it by 0.03 m once (R3726). C's clock runs at the commanded speed and ended 0.09-0.40 m short.
- QUAD4 drops 0.60-0.62 m/s on the rig path under B and C (A 0.31). That is the twin's prop again, but it is the drone
  with the shortest record.

### The follow zone: xy drift below 0.25 m (critic point 2; offline, from the same runs and the 5 Nov rig logs)

Per carrier, from its first crossing of 0.25 m after ring touchdown (scratchpad `lr4_follow.py`):

| | time in the zone, 0.25 m to contact (to LANDED) | xy drift in the zone, to contact (to LANDED) |
|---|---|---|
| twin A (all 6 scenarios; not the strong drone A disarmed in the air) | 0.5-0.7 s (1.1-2.2) | 1.3-2.9 cm (1.0-3.6) |
| twin B | 0.3-0.6 s (1.4-2.4) | 1.3-2.4 cm (1.4-3.8) |
| twin C | 0.5-1.2 s; 1.2 on the unwind (1.5-3.9) | 1.3-2.8 cm (1.4-3.8) |
| twin QUAD4 on the rig path, A / B / C (contact against its start height; it ends 2-3 cm higher) | 0.5 / 0.2 / 1.5 s | 16.7 / 10.1 / 21.2 cm (25.0 / 10.6 / 22.1) |
| rig 5 Nov, A, flights that waited for a taken-back or departed QUAD4 (k1d, nr9m, nr1r_3, nr9o_2, k1o_2, fm9y_2) | - (6.5-8.6 s to LANDED) | 17-102 cm to LANDED (k1d 47-66) |
| rig 5 Nov, A, flights without the wait (fm9_4, fm9y; fm9_2, fm9s, k1ob_2 ended in a ring drop) | - (0.3-2.6 s to LANDED or the log end) | 0.8-14 cm |

- The twin has no drift source near the floor (1-4 cm in every arm), so it cannot rank the arms on drift. It only shows
  how long each spends airborne in the zone: C doubles it on the unwind (1.2 s against 0.6).
- On the rig the drift grows with the time airborne in the zone, ~6-13 cm/s. k1d's 47-66 cm is ~7 s of that: the
  reference stopped at z 0 with the carriers 0.15-0.23 m over it, and LANDED waited for QUAD4.
- Estimate for the rig, not measured: under B or C a carrier reaches the floor once its reference has gone its offset
  (~0.15-0.2 m) further, ~1 s after the crossing under B, ~2 s under C: ~6-13 cm (B) and ~12-26 cm (C) of drift at the
  rig's rate. C can miss a 10 cm drift bar that B meets.

### The rig: what flies first (after the critic, points 5-7; Wesley's word)

- **Twin first, A off (critic point 5).** Every run above flew `land_rearm_touchdown` on (the twin default); the rig
  flies it off, and with it off the twin's rig path had false touchdowns in 2 of 4 (17 Oct, 0.55 m rods). To launch
  (main chat), rig LAND path, 1 m links, A off: `lr4b_hold_u0_norearm`, `lr4c_hold_u0_norearm`, and their base
  `lr4a_hold_u0_norearm` (today's rig: every switch off).
- **First rig flight: a plain three-carrier hover LAND (critic point 6).** The 15 s wait for a taken-back QUAD4
  (`LAND_DEPARTED_WAIT_S`) with the carriers' reference frozen 0.1-0.5 m below the floor is untested. The trackers' ground
  idle latches only within 3 cm of each drone's spawn height with its reference at the floor (tracker_node.py,
  `_ground_idle_update`); a carrier resting higher than that (on its magnet, its cable, the mat) keeps pressing on that
  reference for up to 15 s (R0997's tip-over). The departed-wait case flies only after the first flight shows every
  carrier's ground-idle line.
- **Rig bars, frozen before that flight (critic point 7):**

| # | bar | pass |
|---|---|---|
| R1 | contact speed | every carrier at most 0.15 m/s (mean over the 0.2 s before it is within 3 cm of its rest), with an A or B flight on the same battery set as the base |
| R2 | ground idle | every carrier's "on the floor after the flight: armed idle" line within 0.5 s of its contact |
| R3 | height | every carrier within 3 cm of its rest at the stall line and at LANDED |
| R4 | floor tilt while armed | every carrier at most 10 deg from contact to disarm |
| R5 | drift | every carrier's xy drift from its 0.25 m crossing to contact at most 10 cm; no carrier on the ring or a cable |
| R6 | exit | the stall, with `lift_progress` at least 0.10 m above its -1.50 floor at the stall (logged: the margin) |
| R7 | no tip | every carrier's tilt at most 30 deg from contact to 2 s after its disarm |

## Critic (rev 4)

Run by the main chat, 5 Nov ~14:10. Verdict: **rewrite the card** (fly with changes). Its points as the main chat relayed
them (the full text is with the main chat):
1. (medium) "The rigid rod drops it the last 15 cm" is half right: R3721 drone 0 at 63.87 s is at z 0.255 with its xy
   reference still the anchor 0.47 m inboard, leaning 8.7 deg inward, its z reference 0.24 m below it, vz -0.12; at
   64.12 s the xy follow engages, it levels and falls (-0.28, then -0.45 m/s). The rod pushed against the inward xy hold
   props it while the reference runs ~0.22-0.25 m below; the follow removes the prop. B (bigger gap) 0.37-0.40 against
   C 0.30-0.34. Settle offline: each drone's z gap at the 0.25 m crossing against its contact speed.
2. (medium) S4 "reproduces the rig hover" is mostly the rod (R3725 at 62.80 s: drone 1 at z 0.368, 0.44 m outboard of
   its anchor, leaning 9.2 deg inward); the map error alone is S4u's 12.4 cm. Wesley accepted carriers 9-16 cm up at
   LANDED (10 Oct). New on the rig (k1d) was 6 s of hover with 0.48-0.61 m of xy drift below 0.25 m: the xy follow, not
   the reference stop, and C spends ~2x longer in the follow zone. Quantify the follow-zone drift per arm.
3. (low) Refuse `land_slow_vel` below 0.05 (at 0.01 the stall threshold is 0.25 mm per tick, mocap noise resets it, the
   backstop runs 150 s) and without `land_descend_to_stall` (never flown), in the launch.
4. (low) `test_off_is_todays_nodes_exactly` compares the new code with itself: compare against the old behaviour.
5. (high) Every C run flew with `land_rearm_touchdown` on; on the rig it is off, and with it off the twin's rig path had
   false touchdowns in 2 of 4. Prepare C and B with it off on the rig path.
6. (high, for a second flight) The 15 s wait for a taken-back QUAD4 with the reference frozen below the floor is untested;
   ground idle latches only within 3 cm of the spawn height. A plain three-carrier hover LAND first; the departed-wait
   case only after that flight shows each carrier's ground-idle line.
7. (medium) Rig bars: contact <= 0.15 m/s with an A or B flight on the same battery as the base; each carrier's
   ground-idle line within 0.5 s of contact; floor tilt while armed; xy drift below 0.25 m <= 10 cm with no carrier on
   the ring or a cable; the max-drop margin at the stall; "no tip" as a number.
9. Its "not approved" point is wrong: Wesley approved this work ("2) Yes", 5 Nov).

## Response (5 Nov ~14:15)

1. Agreed and settled offline: the table under the verdict. The gap at the crossing is the prop and the same in every
   arm (0.21-0.26 m); contact follows the incoming speed plus the fall. The verdict text is corrected; registry
   correction rows R3716c-R3721c, R3725c, R3727c.
2. Agreed: corrected under Findings; the follow-zone drift per arm is its own section above. The twin cannot rank the
   arms on drift (1-4 cm in all); on the rig the drift is ~6-13 cm/s while airborne in the zone, and C doubles that time.
3. Done: refused at the launch (`real_mode.planner_options`) and by the planner; launch test added.
4. Done: the off test compares against a frozen copy of the committed `_land_hold_refs` and `TouchdownDetector.update`;
   a check that the comparison fails with the switch on was run once.
5. Done: `lr4{a,b,c}_hold_u0_norearm` (rig path, A off) prepared and checked (`--check` ok), not launched.
6. Noted in the rig section: a plain three-carrier hover LAND first, the departed-wait case only after it.
7. Done: R1-R7 in the rig section, to be frozen before the flight.

What the change does and does not address (the honest reading): the hover Wesley saw is the reference stopping at z 0
with the carriers riding 0.15-0.23 m over it, held there for ~7 s by the wait for QUAD4. **B** (the reference keeps
going) removes the stop, so the carriers would reach the floor ~1 s after the 0.25 m crossing instead of hovering, and
most of the drift window goes with it. **C** adds a softer contact (0.10 against 0.20 m/s where the reference sets it)
that nobody asked for on the rig, at the price of twice the time airborne in the follow zone: for the drift, C is worse
than B. Neither touches the drift's own cause (no xy correction below 0.25 m while still airborne) or the 15 s wait;
with B or C the wait happens on the floor, harmless only if ground idle latches. The direct fixes are in the follow
zone (follow xy only once a drone is within a few cm of its rest, or keep a soft hold until contact; R0590's tip-over
is why the follow exists) and in the wait (the carriers idle and disarm alone while QUAD4 lands). Each is a variable of
its own.

## Decision (Wesley 5 Nov ~15:05)

- **B is the rig default.** `land_descend_to_stall: true` in `src/bringup/config/real.yaml`, so every rig planner gets
  it; a params_file with `land_descend_to_stall: false` turns it off (it is not a launch argument). The twin keeps it
  off for now: the weekend claim batch flies the frozen tree, and Wesley may extend it later. `land_rearm_touchdown` is
  unchanged (twin on, rig off). Launch test `test_rig_lands_with_descend_to_stall_by_default`.
- **The slow final descent is deleted** (`land_slow_vel`, `land_slow_z`, the commanded-speed stall test, the slowed
  max-drop clock, their refusals, tests and the arm C configs): it did not beat plain `land_descend_to_stall` on the
  rig's problem. Its runs stay recorded: R3716-R3733 and R3746 (registry; each run dir keeps its config). The LAND path
  is the committed one (9f20f8b) again; `test_land_no_floor.py::test_references_are_the_reviewed_ones` pins its
  references against a frozen copy.
- **The first rig flights test it.** No separate barred flight (his call); R1-R7 above are not flown as a gate.
- **Open risk: the departed-wait case.** With a taken-back QUAD4 the carriers' reference stays frozen below the floor
  for up to 15 s (`LAND_DEPARTED_WAIT_S`). The trackers' ground idle latches only within 3 cm of each drone's spawn
  height, so a carrier resting higher keeps pressing on that reference. Read each carrier's ground-idle line in the
  first flights' logs.
