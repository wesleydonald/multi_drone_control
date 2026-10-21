# Card: an unannounced release into the T or the opposite pair keeps flying (21 Oct, rev 2 after the critic)

Chain step 1 (plan 2026-10-21 §1.6 and §4). It covers the drop rule only. The release transient, the tether-mass split
and the rig's `attach_z` are the next cards. **Bars FROZEN 21 Oct 19:33**, before the baseline `c4_release9_base`
flies (batch 1, last run) and before any arm.

## 1. Question
Wesley (21 Oct): after an unannounced release from the 90 deg square "we need to be able to accept and minimise the
tilt". This card asks two things:
- Does the planner keep flying when a release leaves the ring's centre of mass on the survivors' edge (the T 0/3/6,
  or the opposite pair 0/6), instead of dropping the ring, and only then?
- How large is the square-to-T transient on today's code? That is your "see what the tilt is".

## 2. Hypothesis
1. **The rule.** With `opposite_pair_ok` on, and only after the lift, a loss that leaves a T (largest gap 180 +- 1
   deg) or an opposite pair resizes and flies on. Every other loss drops as today. With the switch off, nothing
   changes.
2. **The square to the T is not fixed by the rule.** During the 0.22-0.34 s detection delay, the quad opposite the
   gap keeps its old ~2.4 N share (2.44 N x 0.225 m in the T, against 2.45 N x 0.195 m in G180).
   - R1901/R1906 (G180, today's tree, drop off) resized at +0.22 s and still rolled to the 60 deg fault at +0.63-0.66
     s; R1919 (drop on) was dropped at +0.32 s.
   - **Prediction:** `c4_release9_base` reaches the 60 deg fault, and `c4_release9_pair` ends in the trackers' 30 deg
     drop ~0.3-0.4 s after the release.
   - The fix (react within ~0.1 s, the tether-mass split) is the next card.
3. **The T to the pair.** The plate-3 quad carries ~0.1 N in the T, so its release may never stretch its cable the
   6 cm detection needs.
   - If it is detected, the pair resizes and hangs level by gravity.
   - If not, the rule never acts and the arm is inconclusive.

## 3. Variables
The arms differ from the baseline in three things: `drop_on_loss` on (which also arms the trackers' 30 deg drop and
detection during LAND), `opposite_pair_ok`, and `min_survivors` 2 (`configs/rig/chain_pair.yaml`).
- `c4_release9_base` measures the unprotected transient, as G180 did. **It is not the switch's control.**
- The switch's control is `t3_pair_release_off`: the switch off and `min_survivors` 2, with everything else as arm 2.

Configs:
- `c4_release9_base` (batch 1);
- arm 1 `c4_release9_pair`;
- arm 2 `t3_pair_release`: the T world from the floor, 3 drones, RELEASE of drone 1 (plate 3) at 30 s, LAND at 50 s;
- neighbours `t3_pair_release_off` and `t3_pair_release_az0` (the planner's `attach_z` 0.0 in the 0.02 world, the
  rig's model gap).

## 4. Baseline
- R1901/R1906: G180, today's tree, drop off; resize +0.22 s, 60 deg fault at +0.63-0.66 s.
- R1919: drop on; dropped at +0.32 s at 24 deg.
- The legacy R0561/R0562 are an ANNOUNCED detach on the legacy plant (0.5 m rods), so they are not this release.
- `c4_release9_base` tonight.

## 5. Pass / fail numbers (frozen 19:33; scorer: tools/experiment metrics plus `score_1021.py` on the run's run.csv
and events.csv; roll = about the survivors' line, see-saw = about the perpendicular)

| Run | Confirms | Falsifies | Inconclusive |
|---|---|---|---|
| Unit tests (before any run) | switch off: every decision as today; with the switch, after the lift, only the T and the opposite pair fly on; a pair 90 deg apart, a gap > 181 deg, one quad, a loss in LAND or before the lift all drop; the announced path leaves only an opposite pair under the switch | any of these fails | - |
| Arm 2 `t3_pair_release` x2, **whole flight from TAKEOFF** | no drop or abort; tilt peak < 15 deg (the T lift included); "UNANNOUNCED DETACH" and the resize to n=2 within 1 s of the RELEASE; the pair's roll < 5 deg peak-to-peak and lean < 5 deg over its last 10 s; every quad lands | a drop or abort, tilt >= 15, or the pair over 5 / 5 in either repeat | no detection (the rule never acted) |
| `t3_pair_release_off` x1 | drops at detection (the switch is what keeps arm 2 flying) | it flies on | no detection: the detector cannot see a near-zero-share leaver |
| `t3_pair_release_az0` x1 | informational: the pair's roll and lean with the rig's model gap, against arm 2 | - | - |
| Arm 1 `c4_release9_pair` x1 | informational, scored against the prediction (30 deg drop at +0.3-0.4 s); staying under 30 deg would mean the symmetric square is milder than G180 | - | - |

**Rig go/no-go for Part C tomorrow** (arm 1 is not part of it):
1. the unit tests pass;
2. arm 2 passes with the pair formed;
3. tomorrow's B1 pair hover flies clean;
4. Wesley's word that the pair flies on the rig's `attach_z` 0.0.

Then on the sheet: QUAD2 not in the pair, `int_k_xy` 0, no SLOT OFFSET line, `opposite_pair_ok` and `min_survivors`
read back, and the procedure for an undetected release (critic item 8).

## 6. Repeats
Arm 2 x2; arm 1 and the neighbours once.

## 7. Cost
c4 base 1 (batch 1) + arm 1 1 + arm 2 2 + neighbours 2 = 6 laptop runs, each recorded.

## 8. New code?
Yes, in `dissipative_node.py`:
- `opposite_pair_ok` (default off);
- `_opposite_pair` and `_on_edge_ok` (the T or an opposite pair, only after the lift);
- `_drop_due` and `_detach_plan` use them; the announced path leaves only an opposite pair, under the switch;
- under the switch a failed resize drops the ring;
- `_largest_gap_deg`, the logs and `metrics.cog_margin` are unchanged;
- the knob is added in `common.yaml` and `real_mode.PLANNER_OPTIONAL`;
- unit tests in `test_drop_and_land.py`.

If arm 2 fails, the switch stays off and the code goes (the keep-only-if-better rule).

## Critic (21 Oct ~19:30, verbatim summary of the must-fix items; full report in the session)
1. **The prior.** It comes from the wrong plant and the wrong release. Replace it with R1901/R1906 and write down
   the expected c4 result (the 30 deg drop at ~0.3-0.4 s).
2. **The rig gate.** Separate it from c4 and tie it to what decides tomorrow: the unit tests, t3 with the pair
   formed, the B1 pair hover, and Wesley's word on `attach_z` 0.0.
3. **t3 must prove the switch acted.** Detection plus the resize to n=2 within ~1 s, and a neighbour with the
   switch off and `min_survivors` 2.
4. **Close the announced path.** `_detach_plan` checks only the count, so `min_survivors` 2 would fly any pair.
5. **QUAD2 is in the pair on the sheet.** Put it on plate 3, or leave it out.
6. **Score t3 from TAKEOFF.** The T from the floor has never flown, and it is mechanically the 21 Oct pair lift.
7. **The t3 config typed `num_drones` 4.** As written the run was void.
8. **Sheet: an undetected release followed by LAND can drop the ring from height.** Detection runs during LAND, and
   `_drop_due` returns 'during LAND' first.

Should-fix:
9. freeze stamps, a named scorer and windows, RELEASE at 30 s or later;
10. the pair bars at 5 deg, a lean bar, and a run with the planner's `attach_z` 0.0;
11. "one variable" is three;
12. a failed resize under the switch keeps flying;
13. apply the switch only after the lift;
14. no SLOT OFFSET warning;
15. logs and metrics change with the two-point gap;
16. Part C lower, with `int_k_xy` 0, the drop at 30 vs 55 deg, and the read-back;
17. neighbours;
18. CLAUDE.md if this becomes the rig config.

Verdict: rewrite the card.

## Response (rev 2)

| Items | What changed |
|---|---|
| 1, 11 | §2-§4 rewritten, with c4 predicted from G180 |
| 2 | The rig gate is above; c4 is out of it |
| 3, 17 | The detection bar; neighbours `_off` and `_az0`. The other neighbours (plate 0 from the T, a pair member, a release in LAND, `min_survivors` 3) are unit tests |
| 4, 12, 13, 15 | The rev 2 code. `_largest_gap_deg` is no longer touched, so logs and metrics are as before |
| 5, 8, 14, 16 | The sheet. A lower `target_z` for Part C needs Wesley's word |
| 6, 9, 10 | §5 |
| 7 | Fixed: `num_drones` 3 in launch and io |
| 18 | If Part C becomes the rig config, CLAUDE.md's locked drop line is amended the same session |

## Results (21 Oct evening, tree d440664; registry R2249-R2258; scored against the bars frozen 19:33)

| Run | Result | Verdict |
|---|---|---|
| Unit tests (`test_drop_and_land.py`, rev 3, Wesley's 200 deg rule) | 14 pass; gate --quick passes (1495) | pass |
| `c4_release9_base` R2249 | detected and resized at +0.21 s (tilt 16.8 deg then); peak 57.1 deg at +0.6 s; under 10 deg by +1.5 s; T settles at 6.7 deg | as predicted from G180 |
| Arm 1 `c4_release9_pair` R2254 | detected at +0.2 s; ring 45.7 deg at +0.5 s; the trackers' 45 deg drop: DROP AND LAND | as predicted: the rule keeps the T but not the transient |
| Arm 2 `t3_pair_release` R2250 / R2251 | whole-flight tilt peak 8.6 / 9.6; T 6.8 / 6.8 deg; detected and resized to n=2 at +0.82 / +0.83 s; pair roll 8.4 / 7.8 deg p-p, lean 2.4 / 2.2 deg | **FALSIFIED on the pair's rocking (bar 5 deg p-p)**; the rule acted |
| `t3_pair_release_off` R2252 | detected, DROP AND LAND (360 deg gap) | confirms the rule is what keeps arm 2 flying |
| `t3_pair_release_az0` R2253 | T 27.3 deg; tilt peak 41.4; pair roll 81.8 deg p-p, lean 24.4 | the rig's attach_z gap wrecks the T |
| (extra) `t3_pair_release_rodmass` R2258 | T 4.4 deg (rider planned 0.43 N); release NOT detected; pair 6.6 deg p-p | the tether weight halves the T's hang; detection of a zero-share leaver is unreliable |

**Rig go/no-go for Part C: NO-GO** (arm 2 failed its frozen bar). It moved to Wed 28.

**Next card (chain step 2):**
- solve the reference balance for force vectors with the tether mass, so the rider holds its own tether and the T is
  level;
- react to a release within ~0.1 s: the opposite quad sheds its old share as soon as the release is seen;
- a release cue that sees an unloaded quad: the magnet marker, card 2026-10-21_tip_detect rev 2, built on the rig's
  recordings;
- the rig ring pose moved to its centre of mass, with attach_z typed.
