# Tier A, the crossover: where the survivors learn of the change (thesis plan P11, freeze 1.5 T2/T3/T4)

Card written 12 Oct 2026 (13:30 from `date`), before any config and before any run; rev 2 at 13:45 after the
critic's round 1 (under "## Critic": the challenge verbatim, then what rev 2 changed and what is left for Wesley). Twin only (the lab PC, when free;
not the laptop before Wednesday's rig day). Predictions and bars are the ones signed on 11 Oct in
`docs/freeze_2026-10-14.md` (1.2 the resolution rule, 1.5 the 1-DOF signs); nothing here re-opens them. Labels per
freeze 1.3: T2, T3 and T4 are **new predictions** (no arm of either pair had flown at the signature).

## 1. Question

Does the pull source the survivors fly on decide how much the ring tilts when one UAV leaves, in opposite directions
for an announced ramped release and for an unannounced one: the planned pull better when the planner knows first, the
measured pull better when it does not?

## 2. Hypothesis

The 1-DOF model (freeze 1.5): a survivor's feedforward pull is F_plan + k (F_meas - F_plan), k = 0 on the planned pull
(`cable_source model`), k = 1 on the measured pull (`cable_source mocap`). The ring tilts by an amount that grows with
the size and the duration of the mismatch between the pull the survivors apply and the pull the ring needs.

- **Announced, ramped (soft v2):** the plan steps to the new shares as the leaver unloads over 3 s; the model is right
  (the twin's plant is the planner's). k = 0 applies the right pull at once; k = 1 applies it late by the measurement lag
  and reports the ring's own rocking back into the thrust (`docs/counter_swing_2026-10-12.md`). **T2: planned <
  measured.**
- **Unannounced:** the planner learns at detection, about 0.3 s after the magnet opens (0.27-0.42 s, 8 Oct twin and
  rig); k = 0 flies the old shares for that long and then steps; k = 1 follows the true pull from the first 80 ms. **T3:
  measured < planned.**
- **T4, the crossover:** (measured - planned) > 0 on the announced ramped pair and < 0 on the unannounced pair.

How much, from what exists (not on the frozen metric and not on this tree, so evidence, not baseline): announced plain,
planned 3.3 deg against measured 8.4 deg (R1128, R1118, 7 Oct tree, peak tilt); the ramp took the planned plain 3.3 to
2.5-2.6 (R1134, R1137). Unannounced measured on the W4 base: Delta-theta 5.74-6.67 (R1459, R1460, R1626). Unannounced
planned has never flown; the model says it sits above the measured arm by the load change held wrong for ~0.3 s plus
the step at the resize, so more than the 1 deg floor of the rule and likely more than twice the arm's range. Twin
planned-pull repeats have agreed within 0.1-0.2 deg (R1134, R1137); the measured arms spread more (W4 set range 0.93
deg; announced plain measured on the model integral R1217/R1218 3.4 and 7.0 deg). Every magnitude above except the W4
set comes from the reference integral; the arms here fly the model integral (the rig's locked default), whose learned
force is frozen through the resize, an input the 1-DOF model has no term for.

## 3. The one variable

Two pairs, each one variable, on one base config and one tree. Base: `configs/experiments/w4_da_release_hover.yaml`
(the W4 line: 120 deg layout 30/90/150/270, leaver at 90, hover at 1.0 m, lift 0.15, `drop_and_land.yaml`, detection
0.06 m, departed_land, departed_ground_idle, model integral, the rig thrust map). Four arms, all from that base:

| arm | pull source (tracker `cable_source`) | event | config | status |
|---|---|---|---|---|
| A1 | mocap (measured) | RELEASE (runner magnet OFF, detection on) | `w4_da_release_hover.yaml` as is | the W4 set exists on a25c40d; re-flown here for one tree |
| A2 | model (planned) | RELEASE, detection on | new: A1 + `cable_source: model` | never flown |
| A3 | mocap (measured) | DETACH with `detach_unload_s: 3.0` (soft v2) | new: A1 with the DETACH event + `detach_unload_s: 3.0` | never flown anywhere: the 7 Oct card's measured soft arm was dropped after its critic's must-fix 1 (below); D4 at 150 deg was the planned pull, R1133 a void of it |
| A4 | model (planned) | DETACH, soft v2 | new: A3 + `cable_source: model` | R1134/R1137 flew its 7 Oct form (`w4_3915_softdetach_model`) |

- Pair T2 = A4 against A3 (announced soft v2; the one variable is the pull source).
- Pair T3 = A2 against A1 (unannounced; the one variable is the pull source).
- T4 reads the two pairs' signs together. Nothing else moves: `detach_unload_t 0.5`, `detach_unload_wait_s 2.0`,
  `detach_detect_m 0.06`, `cable_elev_deg 60`, the step-out 0.5 m, LAND 20 s after the event as in the base.
- The configs are written after this card is accepted; each one's `description` names this card. The release instant
  shifts between repeats: the base has RELEASE at runner time 20.0 s (12 s after WAIT_LIFT at 8.0); the three repeats of
  an arm put the event at 20.0, 20.5 and 21.0 s and LAND 20 s after it (40.0, 40.5, 41.0), so the three runs meet the
  ring at three phases of its settling, as the plan asks. The dither adds range to each arm on purpose; it is in the
  frozen rule's bar.
- **A3 is not a ramped release unless the leaver really unloads.** On the measured pull the leaver's node 0 is its
  measured rod pull (`tracker_node.py:1770-1774`), so the planner's 3 s unload lowers the planned tension the leaver's
  tracker does not fly, and the release gate reads the planner's own tension (`dissipative_node.py:1461-1465`) and
  passes by construction. The 7 Oct soft-detach critic found this (must-fix 1) and the measured soft arms were dropped
  then. Here A3 flies as specified and is **labelled by what the leaver carried when its magnet opened** (section 5);
  making the leaver fly the planned, unloading term during an unload would be a flight-code change and needs Wesley's
  word (open question in the hand-back).
- Tree: the week5 candidate at the time of the batch; its SHA goes at the top of the result table and every run's
  registry row. All twelve runs on that SHA (one code tree per claim table).

## 4. Baseline

Each pair is its own comparison; there is no outside baseline on the frozen metric and this tree. What is known:

| arm | nearest existing runs | metric there | value |
|---|---|---|---|
| A1 | R1459, R1460, R1626 (W4 set, a25c40d / c2279cf) | Delta-theta cut at 15 s (freeze) | 6.12, 5.74, 6.67 deg |
| A2 | none | | |
| A3 | none (no measured-pull soft arm has flown); nearest: announced plain measured on the model integral, R1217, R1218 | Delta-theta | 3.4, 7.0 deg |
| A4 | R1134, R1137 (7 Oct tree, reference integral, 1/3/5/9) | peak tilt (pre-freeze) | 2.5, 2.6 deg |

The 8 Oct rig pair r302_3 (announced plain, measured) against r0012 (announced plain, planned at 45 deg) differs in
five settings and is not a baseline for T2 (plan 4.2 B).

## 5. Pass / fail numbers (frozen; freeze 1.1 and 1.2)

**Metric:** Delta-theta per run (the tilt vector's largest departure from its 5 s pre-window mean), window from the
event to event + 15 s (Q1, the W4/W5 horizon; every arm holds 20 s). Event times: A1, A2 the runner's RELEASE; A3, A4
the **ramp start**, the first planner tick after DETACH at which the leaver's planned tension falls (freeze 1.1, "soft").
Beside it, reported per run, not scored: the absolute peak, the dip, time back in band (the hold is 20 s, so it is
scorable), sigma_pre and the Q12 flag, the landing lean, the survivors' largest planned step; **the leaver's actual rod
pull when its magnet opened** (A3, A4; `tools/detach_tensions.py` as on 7 Oct) and its pre-unload pull; **Delta-theta
split at the magnet opening** for A3 and A4 (the ramp-start value stays the score); **the detection delay** for A1 and
A2 (it may depend on the pull source: a leaver still flying its planned pull leaves faster); and, if the counter-swing
branch is merged by then, the measured-pull swing after the arrival (`tools/counter_swing.py`; the twin reads about 0).

**The ramped-release rule, written now:** if A3's leaver carries more than 50 % of its pre-unload pull when the
magnet opens, A3 is "not a ramped release" and T2 and T4 are **reported, not confirmed**, whatever the signs. The same
rule is applied to A4 (expected to pass: R1134/R1137 released at about 0.5 N).

**Resolution (freeze 1.2), per pair, arms of n = 3:** resolved only if (1) sign(mean B - mean A) is the frozen sign,
(2) the ranges do not overlap, (3) |mean B - mean A| >= max(1 deg, 2 x the larger range). Otherwise unresolved.

| | confirmed | falsified | unresolved |
|---|---|---|---|
| T2 (A4 vs A3) | resolved with planned < measured | resolved with measured < planned | neither |
| T3 (A2 vs A1) | resolved with measured < planned | resolved with planned < measured | neither |
| T4 (crossover) | T2 and T3 both confirmed | T2 or T3 falsified | any other outcome |

An unresolved pair is reported as unresolved with its six values; it does not become a claim and is not re-flown to
make it one (the plan's "levers stay twin-unresolved" is the stated risk). Exceptions written now: an A2 run that
drops the ring (tilt over 30 deg under `drop_on_loss`) is **T3 support**, scored as its Delta-theta at the drop with a
">=" mark, not a void; an A3 unload cancel (the planner's own tilt or gate) is a **result**, reported, not re-flown. A
solver fault or a twin artefact (a tip-over in the LAND unwind) is re-flown once. At the twin's spreads the rule is
unlikely to resolve T2: A1's range of 0.93 deg means A2 must average about 8.1 deg with no overlap (reachable if planned
repeats stay within 0.2 deg), while a measured spread like R1217/R1218's 3.6 deg sets T2's bar at 7.2 deg against an A4
near 2.5 deg. Unresolved is then the honest outcome and is written as such.

**What falsifies the mechanism, not only the sign** (agreeing with the table: unresolved is neither): T3
**falsified** (A2 resolved below A1) says the "planner learns 0.3 s late" mismatch does not drive the twin's tilt, and
the plan's C2-B sentence ("the measured pull wins when the change is unannounced") and the abstract's bracket change
their wording; it does not move the measured-pull default, which rests on rig carry evidence. T2 **falsified** (A3
resolved below A4) says the measurement lag is not what costs the measured pull on an announced change, and the planned
soft-v2 rig flight of P14 loses its reason. T2 **confirmed with A3 "not a ramped release"** is the critic's trap: a
three-times-larger release step, not the lag; the rule above keeps it out of the claim.

## 6. Repeats

3 per arm (the plan P11's x3; Wesley's plan), 12 runs. Order: A1, A2, A3, A4, then the second and third of each in the
same order, so a tree or lab fault part-way leaves one complete 2x2 at n = 1 or n = 2. P13's claim re-fly on the frozen
tree happens whatever this batch shows (the plan); this batch is exploratory and its label stays so.

## 7. Cost

12 Gazebo runs at about 4 min each: about 50 min on one slot; on the lab PC at most 4 slots, evenings only (the
working rules, 12 Oct), about 15 min of wall time, only when nobody else is on it (`who`, load, containers first), with
the slot count in every registry row (the 12 Oct wave-1 confound R1638-R1641 showed slot load moves results). It waits
behind the lab evenings Wesley has already ordered (decision 178's four items, then 182's week5 W4/W5 checks); P11 says
from Sat 17. Not on the laptop before Wednesday. Plus `tools/prebuild_planner.py` once on the tree.

## 8. New code?

**Flight code: no.** Every arm is a config of parameters that exist (`cable_source`, `detach_unload_s`, the runner's
RELEASE/DETACH events).

**Analysis code: yes, small, two items; the critic's challenge is below.**
1. The soft-detach event time. `tools/release_bands.py` finds RELEASE and DETACH only; the freeze defines the soft
   event as the ramp start. The planner prints `[dissipative] UNLOAD <drone> over 3.0 s before release` at that tick;
   the runner (and `tools/rig_to_run.py`) would log it as an `UNLOAD` event and `release_bands.Run` would take `UNLOAD`
   over `DETACH` when both exist. About 15 lines, no flight behaviour. `release_bands.py` scored the frozen W4/W5 bands
   and `rig_to_run.py` converts Wednesday's bags: the edit lands only if `metrics R1459 R1460 R1626 R1457 R1627` and one
   8 Oct conversion come out byte-for-byte the same, or else after Wednesday's W4/W5 ledger rows are scored. If it is
   not in by the batch, A3 and A4 are scored from DETACH (the command, one tick before the ramp start) and the card says
   so.
2. Nothing else: the metrics, the bands and the swing numbers come from `release_bands.py`, `release_transient.py` and
   `counter_swing.py` as they are.
Removal plan: the UNLOAD event is a log line and a lookup; if the card is dropped, the two edits are reverted in one
commit.

**What the result cannot say.** The twin has no magnet hold, no slack tether, no mocap noise and none of the rig's
1.4-1.7 Hz rocking (`docs/counter_swing_2026-10-12.md`); the measured pull's costs on the rig (the swing fed forward)
are not in these runs. A twin crossover is a statement about the mechanism in the planner's own plant; the rig rows of
the plan's P14 (28-29 Oct) decide whether it carries.

## Critic

### Round 1 (rev 1, 12 Oct 13:34-13:44): the challenge, verbatim

1. **Has it been tried?** A4 has flown before as R1134/R1137 (peak 2.5/2.6 deg, 7 Oct tree, reference integral). A1 is the W4 set R1459/R1460/R1626 (Delta-theta 6.12/5.74/6.67). A2 really has never flown in the twin: the registry has no planned-pull RELEASE row. The only unannounced losses on the planned pull are rig magnet slips (the 30 Sep lifts, no bag; r0013's second survivor at +0.155 s, inside a capsize), and none of them can be scored. A3's mechanism was weighed on 7 Oct and dropped. The soft-detach critic's must-fix 1 found that on the measured pull the leaver is not unloaded: node 0 is measured, `tracker_node.py:1770-1774`. Wesley then removed the measured soft arms. So the card's "D4 flew it at 150 deg" and "R1133?" are wrong: D4m was the planned pull, R1133 is a void of it, and no measured-pull soft arm has ever flown.

2. **Simplest explanation the card does not address.** For T2 it is the load the leaver still carries when its magnet opens. On A4 that is about 0.5 N (R1134/R1137). On A3 it may be the 1.3-1.7 N of the plain measured R1118, because the release gate reads the planner's own tension (`dissipative_node.py:1461-1465`) and passes by construction. "Planned < measured" would then come from a three-times-larger release step, not the measurement lag. The rocking half of the card's mechanism is rig-only (counter_swing: the twin's survivors read 0.00-0.01 N). P6 item 3 also shows the twin's measured pull is not pure k = 1: survivors took 59-76 % of the announced plan's step when it arrived (R1118/R1121, 6 of 6). For T3, the go-between is the detection delay, which may depend on the pull source (a leaver still flying its planned pull leaves faster), and the card does not report it. The model integral is the right condition: it is the rig's locked one. But every magnitude in section 2 comes from the reference integral (T14: the integral mode alone halves the tilt). The integral's learned force is frozen through the resize, an input the 1-DOF model has no term for.

3. **What would falsify it, and does the bar test it?** T3 is falsified by A2 resolved below A1; T2 by A3 resolved below A4. The frozen rule tests the signs, but at the twin's spreads it is unlikely to give a confirmation that tells the explanations apart.
   - T3: A1's range is 0.93 deg, so A2 must average about 8.1 deg or more with no overlap. That is reachable if planned-pull repeats stay as tight as on 7 Oct (0.1-0.2 deg).
   - T2: the only neighbour of A3 on the model integral (announced plain measured, R1217/R1218) read 3.4/7.0 deg. That range of 3.6 sets a bar of 7.2 deg, so against A4 at about 2.5 deg, T2 is very likely to come out unresolved.
   - P6's 4.5 deg MDE is a rig number and does not apply here. Its n = 3 range penalty does, and the 0.5 s release dither adds range on purpose (the ring still drifts about 1.5 deg across a pre-window, freeze 2.1).
   - The ramp-start window is fair within T2, and T4 only compares signs. But it puts the 3 s unload inside A3's Delta-theta, so a tilt from a mismatched measured-pull unload would count as T2 support (v1 showed 2.0 -> 4.7 deg from exactly that, R1120).

4. **What it breaks if it works.** It touches no flight code, no locked decision, no gate and not the network. The risks are elsewhere:
   - `tools/release_bands.py` scored the frozen W4/W5 bands, and `tools/rig_to_run.py` converts Wednesday's bags. The UNLOAD edit must re-score R1459/R1460/R1626/R1457/R1627 and re-convert one 8 Oct bag byte-for-byte the same, or wait until Wednesday's W4/W5 ledger rows are scored.
   - "Up to 6 slots" breaks the working rules' cap of 4 lab slots (12 Oct), and the 12 Oct wave-1 confound (R1638-R1641) shows slot load moves results.

5. **Is it asked for?** Yes: P11 of the adopted plan (decision 158), and the T2-T4 signs Wesley signed on 11 Oct at 17:24. But P11 says "lab from Sat 17", calls it exploratory and puts the claim re-fly on the frozen tree at P13. Tonight's lab evening is already ordered by Wesley (decision 178's four items, then decision 182's week5 W4/W5 checks), so this waits behind those. The card's "falsified" outcome needs a stated action:
   - T2 falsified would reasonably drop the planned soft-v2 rig flight from P14.
   - T3 falsified would only change thesis wording (C2-B, the abstract's bracket), not the measured-pull default, which rests on rig carry evidence.

**Changes needed (analysis and text only, no new runs):**
- Report the leaver's actual pull at release on A3 and A4 (`tools/detach_tensions.py`, as on 7 Oct). Write the rule now: if A3's leaver carries more than 50 % of its pre-unload pull, A3 is "not a ramped release" and T2/T4 are reported, not confirmed.
- Report Delta-theta split at the magnet opening for A3/A4 (the frozen ramp-start value stays the score), and the detection delay per run for A1/A2.
- A2 dropping the ring (over 30 deg) is scored as T3 support, not a void. An A3 unload cancel is a result, not re-flown.
- Make the section 5 "falsifies the mechanism" paragraph agree with the table: unresolved means neither.
- Fix the A3 history line.
- At most 4 slots, with the slot count in every row.
- `tools/counter_swing.py` exists only in `mdc_wt/counter-swing`, not on week5. Merge it or drop that column; the twin reads about 0 anyway.
- Say whether 20.0/20.5/21.0 are config times (the base has RELEASE at t 20, which is 12 s after WAIT_LIFT) and that LAND moves with them.
- State that P13's re-fly on the frozen tree happens whatever this batch shows.
- Remove the rig-MDE remark about the ramp's 0.8 deg: twin planned-pull repeats agreed within 0.1-0.2 deg.

CRITIC: rewrite the card. As written, a T2 "confirmed" cannot be told apart from a measured-pull "soft" detach that never unloads the leaver (7 Oct must-fix 1, a gate that passes on the planner's own tension), because the card does not measure the leaver's pull at release or split the ramp from the release. The fixes are analysis only.

### Rev 2 (13:45): what changed, and what is Wesley's

Taken into the card: the A3 history (no measured soft arm has flown; D4 was planned; R1133 a void); the ramped-release
rule on the leaver's pull at the magnet opening (50 %); Delta-theta split at the opening and the detection delay per
run as reported values; A2's drop as T3 support, an A3 unload cancel as a result; the "falsifies" paragraph agreeing
with the table; the config times and LAND moving with the dither; 4 lab slots, evenings, slot count per row, behind
decisions 178 and 182; the counter-swing column only if that branch is merged; the UNLOAD edit's byte-for-byte guard;
P13's re-fly whatever the outcome; the rig-MDE remark removed and the twin's own spreads put in its place.

Left for Wesley (the card does not decide them):
1. **A3 as a true ramped release** needs the leaver's tracker to fly the planned, unloading term during an unload
   (flight code, `tracker_node.py` node 0 on `mocap`): his word. Without it, A3 is "announced with a 3 s warning" and
   T2/T4 can only be reported.
2. Whether the batch is worth its 12 runs when T2 is likely to come out unresolved at the twin's measured-arm spread:
   the alternative is T3 alone (A1, A2: 6 runs), which is the pair that can resolve and the one the C2-B sentence rests
   on.
3. When: after decisions 178 and 182's lab evenings, from Sat 17 (P11).
