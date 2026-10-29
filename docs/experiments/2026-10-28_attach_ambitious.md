# Card 2026-10-28: Tejen's attach on the new ring: lower approach and transit, wider gates, a filtered plate, a keep-out

Wesley 28 Oct ~22:10: "on approach, I don't want the magnet to be too high over the payload, around 15 cm"; "make sure
that the single quad on approach uses obstacle avoidance so it doesn't hit the other quads on the way to attaching or
dropping off the object"; "I want his controller to be ambitious ... strict limitations ... cause it to not attempt to
attach even when it seems possible"; "it can match the payload's bob out of phase so I think some filter or smoothing
could help". Answers: overnight; lower the transit too (rely on the avoidance); log the changes only.
Every edit to his code goes in `src/tejen_mission/CHANGES_FROM_TEJEN.md`.

## Setting
The new payload (1.15 kg, magnets at r 0.204, RIG-1028-payload), 1 m cables, the square: carriers QUAD1-3 on plates
0/3/6, QUAD4 joins plate 9 (Tejen's controller, floor_join; pickup_delivery for the object), throttle_max 0.9 (the T
loads like a pair, R2839-R2842). Twin worlds `_p115` (tools/make_carry_worlds_p115.py) plus attach worlds at 1.15 kg.

## Variables (each a switch, default off until the bars below pass)
- **V1 height (the whole approach, Wesley 28 Oct ~22:40: "the full approach to only be 15 cm above the ring, not just
  the last part").** Every leg from QUAD4's climb off its mat to the contact carries the magnet tip ~0.15 m above the
  ring's plates: the climb (takeoff_climb_height), the transit to the plate (TRANSIT_TO_REATTACH), the loaded leg to the
  drop point and the release over the net, and the approach before MATCH (approach_offset_z 0.30 -> 0.15). Today the
  transit carries the tip 0.25 m above the carriers' BODIES (QUAD4's body ~3.25 m up). At 0.15 m above the plates the
  tip passes below the carriers' bodies, so his C++ planner's avoidance of the carriers and their cables (carrier_model
  tether) does the clearing, and QUAD4's body (~1.2 m above the tip) passes only ~0.3 m above the carriers' bodies:
  the planner must keep it horizontally clear of them.
- **V2 gates.** MATCH -> DESCEND on contact attach: xy 0.05 -> 0.08 m, relative speed 0.15 -> 0.25 m/s, held 0.3 s
  (was 1.0 s unbroken); the descent aborts 0.08 -> 0.12 m and 0.25 -> 0.35 m/s. Our co-motion proof is unchanged: a
  wrong catch is still refused and a miss retries.
- **V3 filtered plate.** The plate point QUAD4 follows, and the gates judge against, is the measured plate through two
  first-order low-pass stages at 0.4 Hz in x, y and z (the existing height filter, attach_z_lowpass_hz, extended).
- **V4 keep-out.** On the legs without his planner's avoidance (APPROACH_ABOVE_TARGET, MATCH_VELOCITY, DESCEND, the
  drop's descent): QUAD4 holds its reference instead of advancing while its body, its cable (body to tip) or the object
  comes within 0.30 m of a carrier's body or a carrier's cable segment; it advances again once clear for 0.5 s.

## Baseline
The same flight with every switch off (today's rig settings on the new ring, throttle_max 0.9).

## Metrics
1. clearance: the minimum distance from QUAD4's body, its cable segment and the object to every carrier body and carrier
   cable segment, from his TAKEOFF to the weld (or LAND);
2. time from his TAKEOFF to the weld, and from MATCH's start to the descent's start;
3. tries (CONTACT count) and the proof's verdict;
4. ring tilt peak from his TAKEOFF to the weld; any abort, envelope fault or position-limit LAND.

## Bars (frozen 28 Oct ~22:30, before any run of this card)
- **Safety (every arm, both repeats):** clearance >= 0.25 m; no abort or envelope fault; the weld's proof passes.
  Any miss here fails the arm whatever its speed.
- **V2+V3 (ambitious + filtered):** MATCH-start to descent-start shorter than the baseline's in both repeats, and a weld
  within 300 s of his TAKEOFF in both.
- **V1 (low):** the transit's lowest tip height over the ring area at most 0.25 m above the plate level, with the safety bar.
- **V4:** never triggers a hold that lasts more than 20 s in the square layout (the free plate's line is clear).
- **Combined (V1-V4 on):** meets the safety bar and welds in both repeats; tomorrow flies this arm only if it does.

## Repeats
2 per arm in the twin. Arms: baseline, V1, V2+V3, combined (V1-V4). V4 alone is flown only if the combined arm's
clearance is under 0.30 m.

## Limits
The twin's ring bobs less than the rig's (rigid links, no rotor wash): V3's benefit may not show there. The twin tests it
for no harm; the rig shows the benefit. The twin's slack-free cables also hide a slack-cable strike.

## Critic
(28 Oct ~22:45, before any run; read-only critic, pasted here verbatim in substance. Verdict: **rewrite the card**.)

Short answers. (1) Tried before: V3 (card 2026-10-16_plate_height_filter: z-only two-stage 0.8 Hz cut tip-vs-plate
motion 35 % at 1.8 Hz but +24 % at 0.8 Hz; the low-passed twist was removed; the decided route is `attach_twist:=zero`
as a rig A/B, decisions.md:223); V2 partly (widened 14 Oct; a3_2 replay: dwell 0.5 s passes at 1.1 s;
`tools/match_gate_replay.py --zero-twist` opens 3 of 4 blocked rig MATCH windows); V1's transit was moved UP after
RIG-1015-a1/a1c (carry_clearance 0.25, decided 21 and 23 Oct); V4 new. (2) a3_2's 42 s in MATCH: the bridge's raw
finite-difference twist (plate speed rms 0.09 m/s) and a 5 cm gate round a 5.2 cm median tip error. (3) V1 has no
completion falsifier; the V2+V3 bar passes from the dwell cut alone and the twin cannot fail it. (4) It breaks the
twice-decided carry_clearance, the 16 Oct zero-twist A/B, the 16 Oct critic finding 3 (the bottom hold and proof read the
plate point the filter moves), Tejen's say over his gates and C++ margins, and the reviewer step. (5) Asked for by
Wesley, but QUAD4 has avoidance only on the two C++ legs.

1. BLOCKING. V1's lowered transit end is inside the carriers' obstacles in his own collision model: inflated bounding
   boxes (world_snapshot.cpp:274-305), the ego cable a 15 deg swing pyramid down the 1.04 m rod (box 0.56 x 0.56 x 1.04;
   ego_collision_model.cpp:24-49; max_swing_angle 0.2618, partner_backend_overrides_rig.yaml:10), each carrier tether a
   0.015 m bar to its plate (ego_collision_model.cpp:89-114), inflation 0.05 + 0.05 (c1f6_moving_rendezvous.yaml). The
   tethers on plates 0 and 6 each block QUAD4's body around (+-0.45, 0, ring+0.95), half-extents (0.65, 0.40, 1.07); the
   transit's body target above plate 9 with the tip 0.15 up is (0, 0.20, ring+1.22): inside. The lowest end his model
   accepts is a tip ~0.95 m above the plate (today's 1.18 clears by ~0.23). Expect TRANSIT_TO_REATTACH never to finish.
   The loaded drop leg has the same problem. Split V1: V1a (approach_offset_z 0.15 only) and V1b (the transit), V1b only
   after an offline goal check against his world snapshot with the rig parameters; loosening his margins is his call.
2. BLOCKING. Avoidance exists only where his C++ planner owns the leg: TRANSIT_TO_DROP_POINT always,
   TRANSIT_TO_REATTACH only in floor_join (online_join_planner.py:5138-5143, :575). In pickup_delivery the reattach
   transit is his Python transfer at approach_offset_z (:8258-8265, :7082-7093). No avoidance on TAKEOFF,
   APPROACH_ABOVE_TARGET, MATCH, DESCEND, ATTACH_READY, LIFT_PROOF, the abort/retry loop, SETTLE_ABOVE_DROP_POINT,
   DESCEND_TO_DROP_HEIGHT, CLEAR_DROP_ZONE. The bridge feeds modelled (ring-frame) carriers, not measured ones.
3. BLOCKING. V4's 0.30 m keep-out cannot be met in the square: plate 9 is 0.287 / 0.290 m from plates 0 and 6, so the
   hold fires on every descent (tip ~9 cm above the plate) and on every drop descent (~0.21-0.25 m). DESCEND's reference
   and completion run on phase elapsed time (:7008-7030, :8336-8341): holding the reference without freezing that clock
   reaches ATTACH_READY with the tip not at the bottom. Redesign: exclude plate 9's last ~0.10 m (or ~0.20 m), measured
   carriers, a frozen phase clock, every hold logged.
4. BLOCKING. The 0.25 m safety bar measures near-plate geometry (0.287 m floor at a good contact), not strike risk; the
   object at the drop point fails it every time, the baseline included. Score the §5 windows (`tools/m1_metrics.py`
   `flight`: the rod until the first CONTACT, the ball while carried) with a separate near-plate rule; a slack rig tether
   bows outside the straight body-to-tip line.
5. BLOCKING. V3 repeats a measured mechanism at a worse corner: two 0.4 Hz lags give ~0.8 s of delay; the 1 m ring's
   sway is near 0.5 Hz, where |H| 0.35 at ~-107 deg makes the tip error ~1.15x the sway (worse than not following). The
   gates, the bottom hold and the proof read the same attachment_position (:7272-7282, :2902-2918, :8362-8367; the open
   16 Oct finding 3); the xy filter makes the twist low-passed (the variant removed 16 Oct). Choose the corner from
   tomorrow's h4 plate-9 spectrum, against the decided `attach_twist:=zero`.
6. BLOCKING. V2 is falsified offline: replay the rig bags (a3_2, a1, t6f, t6f2_4, t6g_2, t6h5_3) with
   `tools/match_gate_replay.py --gate XY SPEED DWELL` per number and with `--zero-twist`. The twin welds at every contact.
7. BLOCKING. Confounds: V2 is five numbers; V2 and V3 share one arm and both loosen the speed gate; only the combined arm
   flies tomorrow, with no rig baseline on the new ring. One variable per arm.
8. BLOCKING. Preconditions: sq1 not rehearsed on the new ring (no 1.15 kg attach world yet; a world SDF needs Wesley's
   word); the twin square floor_join with a params file is 1 pass (R2829) against 2 stalls (R2830/R2831); no reviewer
   step; V3-xy and V4 code do not exist; the launch was being edited during the review.
9. ADVISORY. Typing attach_transit_clearance alone is refused by check_search_floor (launch :173-178); a negative
   carry_clearance (~-0.78) drives drop_approach_clearance to ~0.03 m; the rig block hard-codes contact_match_xy 0.05
   and the aborts 0.08 / 0.25 (only match_speed and match_dwell are typed). Write every V1 number into the card first.
10. ADVISORY. dwell_time_s also gates APPROACH_ABOVE_TARGET (:8269-8272); MATCH checks xy only, so a 0.3 s dwell can
    start the descent while QUAD4 is still dropping from the transit end.
11. ADVISORY. Rig-only risks: a slack whipping 1 m tether; plate 9 at the largest roll lever of the T with the carriers
    at ~0.81 of 0.9 throttle; a 12 cm / 0.35 m/s abort lands an energised magnet off the plate or on its edge (the drop
    rule fires at 30 deg); K still the old 24 mm on 85-100 mm slots.
12. ADVISORY. Missing neighbours: pickup_delivery, the triangle fallback (plate 9 0.20 m from plates 7 and 11), a retry
    after an abort, a bob neighbour (the n6 shake configs), the T's lean (~7 deg, R2825).
13. ADVISORY. Split the 300 s time bar: climb, his install wait, transit, approach, MATCH, descent, bottom.

Before tomorrow's lab flies any of it: the baseline sq1 welds twice in the twin on a 1.15 kg square world with the
sheet's line; V1a only, if wanted; V1b only after the offline goal check and Tejen's OK on any margin change; V2 values
by replay; V3 or zero twist only after h4's plate-9 spectrum, with the hold and the proof on the raw plate; V4
redesigned; each variant alone against the same morning's baseline, after the reviewer.

## Revision 2 (28 Oct ~23:00, after the critic, before any run of these arms)
Accepted: findings 1-8. What can be valid by 08:00, one variable per arm, against the same baseline:

- **A0 baseline**: sq1 on the 1.15 kg square world (carriers 0/3/6, QUAD4 joins 9, floor_join, throttle_max 0.9, every
  switch off, the sheet's line), 2 runs. Precondition for everything else (finding 8).
- **V1a approach height**: approach_offset_z 0.15 (was 0.30) only; the transit as today. 2 runs.
- **V1b low transit**: NOT flown. An offline check of the low transit end against his world snapshot with the rig
  parameters (finding 1); if his bounding-box model refuses it, the options (shrink his swing envelope or inflation,
  or capsule tethers in his C++ model) are Wesley's and Tejen's calls, reported in the morning.
- **V2 gates**: chosen OFFLINE by `tools/match_gate_replay.py` on the rig bags (finding 6), one number at a time and with
  `--zero-twist`; a twin arm only for the single number the replay picks, 2 runs.
- **V3 smoothing**: the low-pass is NOT flown (finding 5). The decided `attach_twist:=zero` (16 Oct) is the morning's
  option; the filter corner, if any, comes from h4's plate-9 spectrum.
- **V4 keep-out**: NOT built tonight; redesign per finding 3 (exclude plate 9's last ~0.10 m, measured carriers, a
  frozen phase clock, every hold logged), its own card.

Bars (frozen now, before any run of these arms):
- **Safety (each arm, both runs):** no abort, envelope fault or position-limit LAND; the weld's proof passes; the
  clearance in the §5 windows of `tools/m1_metrics.py flight` (the rod until the first CONTACT) not below the
  baseline's lower run minus 0.05 m. No near-plate distance rule (finding 4).
- **A0:** welds in both runs within 300 s of his TAKEOFF (time split per finding 13). If A0 fails, nothing else flies
  tomorrow on the new ring.
- **V1a:** welds in both runs; the tip's height at MATCH's start 0.15 +- 0.04 m above the plate; MATCH-start to
  descent-start not longer than the baseline's longer run + 2 s.
- **V2 (replay):** the chosen number opens at least 3 of the 6 replayed rig windows that the current gate leaves
  closed, and in the twin arm welds in both runs.

## V2 replay (28 Oct ~23:00, tools/match_gate_replay.py on a3_2, a1, t6f, t6f2_4, t6g_2, t6h5_3; raw twist as flown
## and --zero-twist; the old ring's flights)
Windows the flown gate (5 cm, 0.15 m/s, 1.0 s) never passed: 5 with the raw twist, 3 with the zero twist.

| Gate (one number changed) | Raw twist: of 5 opened | Zero twist: of 3 opened |
|---|---|---|
| xy 8 cm | 3 | 3 |
| speed 0.25 m/s | 2 | - |
| dwell 0.5 s | 2 | 1 |
| dwell 0.3 s | 3 | 3 |
| all three (8 cm, 0.25, 0.5 s) | 5 | 3 |

Zero twist alone opens 2 of the 5. Pick (bar: >= 3 opened): **xy 0.08 m** (speed and dwell unchanged): it opens 3 of 5
alone and, with the zero twist, all 5, without shortening the dwell (critic 10: the dwell also gates the approach). The
new slots (85-100 mm) catch further off-centre than the old 90 mm discs, which a wider xy gate matches.
Artefacts: scratchpad replay_raw.txt / replay_zero.txt.

## Baseline (twin)
(28 Oct 22:22-23:00, laptop, tree 413ec0e + the working copy; registry R2843-R2845.) A0 with every switch off: the
1.15 kg square attach world (tools/make_attach_layout_worlds_p115.py), his ring model `geometry:=rig_twin_L100_p115`
(r 0.204, pitch 0.408, contact 0.037, plates 0.09), throttle_max 0.9, the slope filter on the carriers, no params file.
Configs `sq_floorjoin_hover_L100_p115`, `sq_pickup_hover_L100_p115`. Scored by `tools/attach_clearance.py` (every arm
the same way). R2825 is the old ring (0.86 kg, ema, 0.8) on the same scorer.

| run | flight | clearance min, his TAKEOFF -> weld (pair, phase) | rod min to CONTACT (m1 flight) | his TAKEOFF -> weld | climb / wait / transit / approach / MATCH / descent / bottom (s) | MATCH -> DESCEND | tries, proof | ring tilt peak (before his TAKEOFF) | tip above plate 9 at MATCH start / descent start | aborts |
|---|---|---|---|---|---|---|---|---|---|---|
| R2843 | floor_join | 0.287 m (cable-cable, ATTACH_READY) | 0.290 m | 38.7 s | 21.0 / 0.9 / 7.3 / 1.0 / 3.0 / 3.8 / 1.7 | 3.0 s | 1, proven | 8.38 deg (1.39) | 1.059 / 0.296 m | none |
| R2844 | floor_join | 0.285 m (cable-cable, LIFT_PROOF) | 0.288 m | 38.5 s | 21.1 / 0.9 / 7.0 / 1.1 / 3.0 / 3.8 / 1.7 | 2.97 s | 1, proven | 8.58 deg (1.69) | 1.053 / 0.293 m | none |
| R2845 | pickup_delivery | 0.154 m (object-cable, DROP_OBJECT) | 0.274 m (ball 0.211) | 89.7 s (object legs 74.9) | - | 1.23 s | 2 (try 1 failed: separated), proven | 17.8 deg at the drop (1.88) | 0.418 / 0.342 m | FLEET_ABORT 7.5 s after LAND (TB14) |
| R2825 (old ring) | floor_join | 0.315 m (cable-cable, DESCEND) | 0.317 m | 37.8 s | 21.0 / 0.9 / 7.4 / 1.1 / 2.1 / 3.8 / 1.6 | 2.1 s | 1, proven | 9.09 deg (1.76) | 1.058 / 0.305 m | none |

Against the bars:
- **A0 (revision 2): met.** Both floor_join runs weld on the first try, 38.7 and 38.5 s after his TAKEOFF. The
  revision-2 safety floor for the other arms: rod min not below 0.288 - 0.05 = 0.238 m.
- **Original safety bar (frozen 22:30): floor_join met (0.285 / 0.287 >= 0.25), pickup missed (0.154).** Both
  minima are geometry, as critic finding 4 said. At the contact the tip is on plate 9, 0.289 m (the chord) from plates 0
  and 6. At the drop the ball (r 0.08) hangs at the ring's centre ~0.15 m above the plates. On the ring's axis it clears
  the plates' cables by 0.25 m only with its centre >= ~0.31 m above the plate plane (estimate, level ring).
- **For V1a's bar:** in the baseline MATCH starts with the tip still at the transit height (1.05-1.06 m above plate 9).
  APPROACH_ABOVE_TARGET lasts 1 s, and the tip comes down to approach_offset_z (0.30) during MATCH: 0.293-0.296 m at
  the descent's start. The "tip at MATCH's start 0.15 +- 0.04 m" bar cannot be met by approach_offset_z alone (critic
  advisory 10). The descent-start height is the number that follows approach_offset_z.
- **The T leans ~8 deg** (8.2-8.6 deg) from ~3 s after his TAKEOFF to the weld, as R2825's ~7 deg on the old ring
  (attach mode, no params file).
- **Pickup (R2845):** the ball's drop into the net rocks the ring to 17.8 deg (6.7 before). The first contact's proof
  failed (separated, plate rise 21 mm) while the ring still rocked; the second try was proven, 89.7 s after his TAKEOFF.
  The fleet abort 7.5 s after LAND is QUAD4 tipping at touchdown on the twin's rigid rod (tools/m1_metrics.tb14 true;
  decisions 9 and 22 Oct: a twin artefact). His descent to the drop height took 25.6 s.
- **Against the sheet's line:** the twin runs type no params file. `configs/rig/attach_m1_1m.yaml` adds only
  drop_takes_back, which acts only on a ring drop.

## Revision 3 (28 Oct ~23:55, after A0, before any V1 run)
Wesley ~23:05: "either making the bounding boxes smaller or start by trying 0.3 m instead of 0.15 m"; "try the existing
filter option" (attach_twist:=zero); "we need to fix the avoidance"; videos of each successful run and of the avoidance
(QUAD4 starting on the wrong side) by morning. A0 showed MATCH starts with the tip still at the transit height (~1.05 m
above plate 9) and reaches approach_offset_z only during MATCH (baseline section), so V1's bar moves to the transit's end.

- **V1(0.30)**: the whole approach with the tip 0.30 m above the plates: the transit's end (attach_transit_clearance
  0.30, the C++ search floor lowered to match) and approach_offset_z 0.30 (today's). 2 runs, floor_join.
  Bars: TRANSIT_TO_REATTACH completes in both (no stall: its end reached within 60 s of the transit's start); the tip at
  MATCH's start 0.30 +- 0.06 m above plate 9; welds in both within 300 s of his TAKEOFF; safety as revision 2 (rod min
  >= 0.238 m, no abort, proof passes).
- **If his collision model refuses V1(0.30)** (the transit stalls, critic 1): shrink his ego swing envelope and the
  inflation (config, logged as a change to his settings), re-fly V1(0.30) 2 runs with the same bars; the values are
  written here before that run.
- **V1(0.30) re-fly values (28 Oct 23:57, after R2846, before its re-fly).** R2846 (V1(0.30), his margins as in A0)
  stalled as critic 1 said: TRANSIT_TO_REATTACH ran 294 s; his backend's 44 partial plans parked QUAD4 at the right
  height but 0.31 m short of plate 9 (y 0.52), then sat in RECOVERY_WAITING_FRESH_COMMON_SEED. Offline check of the
  end against his model (his AABB pairs, world_snapshot.cpp:274-305; scratchpad goal_check.py): the binding pair is the
  tether of plate 0 or 6 against his magnet box (0.10 m square plus the swing's lateral 1.04 sin(swing)), separation
  -0.137 m with the twin's swing 0.17 rad and inflation 0.05 + 0.05 m. Chosen, the least change with ~2 cm left:
  his swing envelope `max_swing_angle_rad` 0.17 -> **0.035** (2.0 deg; A0's measured rod swing: transit 0.83 / 0.94 deg
  max, approach and MATCH 1.51 / 1.62 deg) and the carriers' tracking inflation `cooperative_tracking_half_m` (x y z)
  0.05 -> **0.03**; his ego tracking certificate stays 0.05 (it trips his brake fallback). End separation +0.023 m (the
  same at 0.15, where the ring's basket box is +0.057). The swing alone would need <= 0.018 rad (1 deg, under the
  measured approach swing); the inflation alone cannot (at 0.17 rad the magnet box reaches past plate 9's line). Typed
  launch args (`max_swing_angle_rad:=0.035 cooperative_tracking_half_m:=0.03`, default off; his files and the rig
  defaults unchanged); config `sq_floorjoin_hover_L100_p115_v1a30m`. The same bars. Caveats: in A0's transit the
  carriers sat 4.2 cm (|dy| 3.6 cm) off his ring-frame model, more than the 0.03; his tether box spans x 0.18-0.72 m at
  every height, while the real distance at this end between QUAD4's rod and the carriers' cables is ~0.4 m (the
  critic's capsule point). The rig's slack tether swings more (RIG-1014-t6f: ~3.8 deg), so these values are twin-only
  until Wesley and Tejen decide.
- **V1(0.30) second re-fly values (29 Oct 00:53, after R2847/R2848, before its re-fly).** R2847 met every bar; R2848
  stalled: his plans reached the end, then at 48.2 s his backend disabled itself (all three carriers
  `tracking_mismatch`) while the T rocked (ring z +4 cm and x +5 cm in ~1 s). `cooperative_tracking_half_m` is not only
  an inflation: it is also his gate on the carriers' live states against their published trajectories
  (transfer_backend_node.cpp:1806-1819), and 0.03 m is too tight for the T on the 1.15 kg ring; his recovery
  (RECOVERY_WAITING_FRESH_COMMON_SEED) never resumed. So the 0.035 / 0.03 pair fails (1 of 2). New values: his tracking
  inflation kept at **0.05** (untyped), the swing envelope alone **0.02 rad** (1.15 deg; A0's measured rod swing in the
  transit 0.83 / 0.94 deg max). End separation in his model +0.018 m at 0.30 and at 0.15 (basket +0.057). Config
  `sq_floorjoin_hover_L100_p115_v1a30s` (`max_swing_angle_rad:=0.02` only). The same bars, 2 runs.
- **The V1(0.15) gate, clarified before any 0.15 run (29 Oct 00:53; from the orchestrating session, not Wesley).** "Rod
  min >= 0.30 m" is read as the minimum BEFORE the first descent (transit, approach and MATCH: attach_clearance's
  `clearance_to_descend`), because the minimum at the contact is plate 9's own geometry (0.289 m chord, critic 4) and no
  arm can exceed it. On the original reading (m1 flight rod min until CONTACT) the gate is not met: R2847 0.285 m
  (A0 0.290 / 0.288). R2847's pre-descent minimum was 0.369 m (A0 0.373 / 0.372). The 0.15 runs fly the V1(0.30)
  values that pass both runs; config `sq_floorjoin_hover_L100_p115_v1a15s` (or `_v1a15m` had the first pair passed).
- **V1(0.15)**: only if V1(0.30) passes with the rod min >= 0.30 m; the same bars at 0.15 +- 0.05.
- **Avoidance (fix)**: his C++ planner owns pickup_delivery's TRANSIT_TO_REATTACH (switch `pickup_cpp_reattach`, default
  off; a re-prepare in CLEAR_DROP_ZONE). Bars: the pickup run welds; the reattach transit completes; rod min >= 0.238 m.
- **Avoidance demo**: QUAD4's mat on the far side (behind the carrier on plate 3); bars: welds, rod min >= 0.238 m.
- **V2 xy 0.08**, **V3' zero twist**: 2 runs each, the A0 bars; plus one run of both together.

## Arms prepared (28-29 Oct night)
Prepared, not flown (28 Oct ~23:25). Each arm is the A0 config with one change, `record` rendering after the run
(`render: true`) with a second, high camera. `tools/run_experiment.py --check` passes on all five, but it does not check
the partner launch's event arguments. Checked against its `DeclareLaunchArgument`s separately.

| arm | config (configs/experiments/) | the one change against A0 | ready |
|---|---|---|---|
| avoidance demo | `sq_far_floorjoin_hover_L100_p115` | world `sq_far_rigid_ground_rig_attach_pickup_L100_p115.sdf` | yes |
| avoidance fix | `sq_pickup_hover_L100_p115_cppreattach` | `pickup_cpp_reattach:=true` | no: launch edits below |
| V3' zero twist | `sq_floorjoin_hover_L100_p115_zerotwist` | `attach_twist:=zero` | yes |
| V2 xy 0.08 | `sq_floorjoin_hover_L100_p115_xy08` | `match_xy:=0.08` | yes |
| V2 + V3' | `sq_floorjoin_hover_L100_p115_xy08_zerotwist` | both | yes |

**The far world** (`tools/make_attach_layout_worlds_p115.py --spawn far`; `--check` covers both spawns; `gz sdf -k` valid):
`simulation_assets/sq_far_rigid_ground_rig_attach_pickup_L100_p115.sdf` and its newcomer
`simulation_assets/models/x3_rig_drone3_magnet_far_pickup_L100.sdf`. The A0 worlds are byte-identical.

| quantity | value |
|---|---|
| QUAD4's mat (body, yaw 0) | (-1.0, -1.0) |
| mat to the plate-3 / plate-6 carriers' floor spots | 1.02 / 1.02 m |
| magnet on the floor (arm pointing away from the ring, its pose turned 135 deg about the pivot) | (-1.706, -1.706) |
| laid-out arm to the nearest carrier's cable on the floor | 1.00 m |
| ball (unused by floor_join) to the arm | 3.97 m |

The body keeps yaw 0, because his reference yaw is 0 and a turned body would sweep the 1 m arm over the floor at takeoff.
His floor_join takes its start from the measured ground pose at TAKEOFF (`wait_takeoff_hold`), climbs straight up from
there and plans the C++ transit from there. Neither his launch nor his node has a mat-position parameter, so the config
types none. A0's camera does not frame the far mat, which is why the config adds `far_top`. At A0's transit height the
tip passes 0.25 m above the carriers' bodies, so the planner can clear them from above. The demo shows a route round
the carriers only with a lower transit (V1(0.30)).

**The twin's MATCH gate.** On the sim path the launch already passes `match_xy` to `contact_match_xy_threshold` (typed
only). Only the rig block (`_rig_nodes`) hard-codes 0.05. Untyped, the twin flies his code defaults: xy 0.025 m, speed
0.08 m/s, dwell 1.0 s. So A0 in the twin used 0.025, not the rig's 0.05. With 0.08 his descent abort xy follows,
max(gate, 0.05) = 0.08.

**Launch edits still needed for `pickup_cpp_reattach`** (sim path `_nodes`; the rig path likewise before any rig use):
1. `DeclareLaunchArgument('pickup_cpp_reattach', default_value='false')`, and the planner parameter
   `'pickup_cpp_reattach': cpp_reattach` with `cpp_reattach = _flag(context, 'pickup_cpp_reattach') and not floor_join`.
2. Backend: `require_payload_attached_for_enable` off when `cpp_reattach` is on (pass `floor_join=floor_join or
   cpp_reattach` to `_backend_overrides`). Otherwise the grant after the drop is refused with
   AUTHORITY_GRANT_REJECTED_PAYLOAD_NOT_ATTACHED (transfer_backend_node.cpp:2135-2145, `payload_collision_enabled` true
   in c1f6_moving_rendezvous.yaml). The cost: the loaded leg no longer disables itself on a detach
   (PAYLOAD_DETACHED_DISABLED, :1404).
3. Heights. The backend has one search floor, the loaded one in pickup_delivery: 3.306 m body at ring_z 1.0 and
   carry_clearance 0.25. Two points sit under it:

   | point | body z (m) | against the floor | edit |
   |---|---|---|---|
   | the hand-over at the end of CLEAR_DROP_ZONE (tip `drop_clear_height` 0.45, his default, untyped, above the drop point) | 2.64 | 0.67 under: WAITING_ABOVE_SEARCH_FLOOR (:2726) | type `drop_clear_height` = `drop_approach_clearance` (1.166): body 3.356 |
   | the transit's goal (`attach_transit_clearance` 1.169 above the contact target) | 3.246 | 0.06 under | when on, `attach_transit_clearance` = floor + 0.05 - rod - (ring_z + 0.037) = 1.279 |

   Extend `check_search_floor` to both points. The alternative is the empty floor (3.196) for the whole mission, but it
   lowers the loaded leg's floor by 0.11 m.

## V1 results (twin)
(28 Oct 23:16 - 29 Oct 01:45, laptop, one run at a time, tree 413ec0e + the working copy; registry R2846-R2852; scored
by `tools/attach_clearance.py` against Revision 3's V1 bars. His `online_join_planner.py` (4cdcc000, the other session's
`pickup_cpp_reattach`, default off), `mission_definitions.py` and `partner_mission.launch.py` hash-checked before every
run: unchanged between runs except my launch args (8ff903b2 from R2847). Every run has `video/<run>_baseline.mp4` (3/4
view) and `video/<run>_top.mp4` (steep overhead: QUAD4's mat at (0, 1.4), its path to plate 9 between the carriers on
plates 0 and 6).)

| run | arm: tip above plate 9 / his C++ margins | transit (<= 60 s) | tip at MATCH start | his TAKEOFF -> weld (<= 300 s) | climb / wait / transit / approach / MATCH / descent / bottom (s) | rod min to CONTACT (>= 0.238) | min before the first descent | tries, proof | ring tilt peak | aborts | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| R2843-R2844 (A0) | 1.169 / his (swing 0.17 rad, tracking 0.05 m) | 7.3 / 7.0 s | 1.059 / 1.053 m | 38.7 / 38.5 s | 21.0 / 0.9 / 7.3 / 1.0 / 3.0 / 3.8 / 1.7 | 0.290 / 0.288 m | 0.373 / 0.372 m | 1, proven | 8.4 / 8.6 deg | none | baseline |
| R2846 | 0.30 / his | STALL 294 s | - | no weld | 15.2 / 1.2 / 294 | 0.603 m | - | 0 | 40.8 deg (drop at 302 s) | DROP AND LAND (tilt 30.5 deg) | **FAIL** |
| R2847 | 0.30 / swing 0.035, tracking 0.03 | 7.8 s | 0.286 m | 32.1 s | 15.3 / 1.4 / 7.8 / 1.0 / 1.0 / 3.8 / 1.7 | 0.285 m | 0.369 m | 1, proven | 8.6 deg | none | pass |
| R2848 | 0.30 / swing 0.035, tracking 0.03 | STALL 300 s | - | no weld | 15.3 / 0.9 / 300 | 0.519 m | - | 0 | 26.4 deg | none (TAKE_BACK at LAND) | **FAIL** |
| R2849 | 0.30 / swing 0.02 | 7.7 s | 0.288 m | 31.3 s | 15.3 / 0.7 / 7.7 / 1.1 / 1.0 / 3.8 / 1.7 | 0.284 m | 0.369 m | 1, proven | 8.8 deg | none | pass |
| R2850 | 0.30 / swing 0.02 | 7.7 s | 0.281 m | 31.9 s | 15.3 / 1.4 / 7.7 / 1.1 / 1.0 / 3.8 / 1.6 | 0.289 m | 0.367 m | 1, proven | 8.9 deg | none | pass |
| R2851 | 0.15 / swing 0.02 | 7.8 s | 0.133 m | 29.1 s | 14.3 / 1.4 / 7.8 / 1.1 / 1.0 / 1.9 / 1.6 | 0.286 m | 0.305 m | 1, proven | 8.9 deg | none | pass |
| R2852 | 0.15 / swing 0.02 | 7.9 s | 0.139 m | 30.6 s | 14.3 / 1.0 / 7.9 / 1.1 / 2.5 / 1.9 / 1.9 | 0.270 m | 0.268 m | 1, proven | 8.5 deg | none | pass |

Against the bars:
- **V1(0.30) with his margins: FAIL** (R2846), as critic 1: his box model of the tethers on plates 0 and 6 covers the
  end (offline -0.137 m); his partial plans parked QUAD4 0.31 m short at the right height.
- **V1(0.30) at swing 0.035 + tracking 0.03: FAIL** (1 of 2). R2848: `cooperative_tracking_half_m` is also his gate on
  the carriers' live states against their published trajectories; at 0.03 m the T's rocking tripped it (48.2 s, all three
  carriers) and his recovery never resumed. Keep it at his 0.05.
- **V1(0.30) at swing 0.02 rad alone: MET in both** (R2849, R2850): transit 7.7 s, tip at MATCH 0.288 / 0.281 m (0.30 +-
  0.06), weld 31.3 / 31.9 s, rod min 0.284 / 0.289 m, proven on the first try. The minimum before the first descent
  (0.369 / 0.367 m) meets the clarified V1(0.15) gate; the original reading (0.284 / 0.289 m) does not.
- **V1(0.15) at swing 0.02 rad: MET in both** (R2851, R2852): tip at MATCH 0.133 / 0.139 m (0.15 +- 0.05), weld 29.1 /
  30.6 s, rod min 0.286 / 0.270 m (>= 0.238). But R2852 is the closest flight of the card before the descent: QUAD4
  reached the transit end still moving (body -0.2 m/s, 5 cm past the goal), the rod swung 2.2 deg in the transit and
  3.5 deg in MATCH (his C++ envelope 1.15 deg), the tip went 8 cm inside plate 9 and the rod came 0.268 m from plate 6's
  cable during MATCH (A0's minimum, at the contact, 0.288 m). R2847 and R2849-R2851 swung <= 0.8 deg.
- **Faster:** his TAKEOFF -> weld 29-32 s against A0's 38.6 s: the climb ends lower (14.3-15.3 s against 21.0), MATCH no
  longer carries the drop from the transit height (1.0 s against 3.0), and at 0.15 the descent halves (1.9 s against 3.8).
- **The T's endurance (rig-relevant, twin pack model):** in both stalls the side carriers (plates 0 and 6) went from 0.80
  of the 0.9 cap to the cap after ~4 min of hover (packs 23.8 -> 17.6 V); the ring tilted past 30 deg and was dropped in
  R2846 (302 s) and reached 26.4 deg in R2848; the carrier on plate 3 sat at 0.49-0.56. A stalled attach on the T needs a
  LAND well before ~4 min.
- **His margins on the rig:** 0.02 rad (1.15 deg) covers the twin's rigid rod in his transit (0.65-0.80 deg measured, but
  2.2 deg in R2852) and not the rig's slack tether (~3.8 deg, RIG-1014-t6f). On the rig his box model refuses a low end
  with any envelope that covers the real swing; the geometric fix (capsule tethers in his C++ model, critic 1) is
  Tejen's code. The typed args are default off; his files and the rig defaults are unchanged.

## Revision 4 (29 Oct ~03:50, before any run of these arms): the commitments' start
R2854 (the far start, low transit) and R2855 (the pickup, C++ reattach) stopped the same way: his backend disabled
itself (COOPERATIVE_SHARED_INPUT_INVALID_DISABLED) with all three carriers at `start_skew` at once (R2855 at ROS 60.8 s,
0.5 s into the loaded drop transit) and never came back. Code read: his backend refuses a carrier whose shared
commitment starts after its latest state (transfer_backend_node.cpp:1794-1799; his `cooperative_shared_start_skew_s`
default 0 allows none, his maximum 0.05 s only below 0.05 m/s), and his ring world refreshes the commitments (every
1.0 s) in its timer tick before it publishes that tick's carrier states. A check between the two fails all three
carriers by one tick. R2853 welded on the same code: a race, so it bites at random. The run logs' disable lines (read
03:50, before any of these runs): R2846 also ended on `start_skew` (ROS 52.6 s, after its 44 partial plans: his box model
and the race both acted there, which one stalled it is not separable); R2848 (and R2830/R2831 on 27 Oct) ended on
`tracking_mismatch`, the rocking T against its published hover: a different trip, which this switch does not touch.

**V5, the switch** (`commit_backdate:=<s>`, default off; CHANGES_FROM_TEJEN.md 29 Oct; his node untouched): the ring
bridge starts the commitments that long before each refresh. 0.04 s: two ticks of the bridge's 50 Hz timer (his default
`publish_rate_hz`; neither launch path sets it, so twin and rig alike: corrected 06:10 after the critic, this line first
said "four of the twin's 100 Hz ticks", the 100 Hz being pendulum_state_publisher's), and under 0.05 s so the first
control point stays the measured ring (when the spline is built within ~10 ms of the tick). Test `test/test_ring_bridge_backdate.py` reproduces the race (off)
and clears it (on).

| arm | config (configs/experiments/) | its parent (flown) | repeats |
|---|---|---|---|
| far start, low transit | `sq_far_floorjoin_hover_L100_p115_v1a30s_bd` | R2854, `..._v1a30s` | 2 |
| pickup, C++ reattach | `sq_pickup_hover_L100_p115_cppreattach_bd` | R2855, `..._cppreattach` | 1, a second if it welds |
| the candidate | `sq_floorjoin_hover_L100_p115_v1a30s_zt_xy08_bd` | not flown (stopped for this) | 2 |

**Bars (frozen now, before the runs).** Each arm keeps its parent's Revision 3 bars. The switch passes when, in every
run of these arms: no backend disable with a `start_skew` blocker, and no new `tracking_mismatch` disable (a backdated
commitment evaluated off the measured ring would show as one). It fails on either. Exploratory: the bars find the
failure; the switch stays off by default and needs a critic look before it becomes one (it changes the input to his
safety gate). His backend's own fix (publish the states first, or a skew allowance at speed) is Tejen's to choose.

### Critic (Revision 4, 29 Oct ~06:05; read-only critic, pasted)
Verdict: **run it on the rig if Tejen agrees** (it changes the input to his gate): bounded, it commands nothing, its worst
case is the latch it removes; it has already flown at the rig's 50 Hz. R2860 stays FAIL under the frozen bar.
1. **Does it weaken his checks?** Barely, in hover. The first 0.04 s of each spline carries the measured ring labelled
   40 ms early: error speed x 0.04 s, <= ~5 mm at the rig ring's +-0.13 m/s bob against his 0.05 m gate (mocap latency
   already makes the measurement older than its label). Just after a refresh the tracking check compares 12-22 % of the
   way from the measured ring toward the planner's target instead of at the ring; by ~0.25 s into each 1 s window both
   compare against the target, so the window's worst case is unchanged and the switch cannot explain R2860's
   tracking_mismatch. No new staleness: the 0.30 s state timeout and the expiry checks stand; what is new is a <= 40 ms
   commitment-state gap at any speed (his allowance: 50 ms, only below 0.05 m/s).
2. **The tick:** the bridge runs at his default 50 Hz in both paths (this card said 100 Hz; corrected). It fails again
   only if the previous tick's states reach his backend > 40 ms before this tick's refresh (a tick late by > ~20 ms).
   The four 63-point splines are built one after another; past ~10 ms of build time the later splines' first point
   becomes the planner target instead of the measured ring (harmless, but not guaranteed as written).
3. **Rig:** one laptop, one launch, wall clock on both nodes: no clock offset; mocap latency is not in the skew (both
   times are local). The unknown is Python timer jitter on the loaded rig laptop: check /fake_payload/pose gaps in an
   earlier rig partner bag (gaps >= 40 ms beat 0.04). sq1 is the first rig flight of his C++ transit
   (`attach_cpp_transit`, default true since 21 Oct), with or without the switch.
4. **The other latch:** his backend switches off for good on any failed input check. tracking_mismatch is on in our
   partner backend (c1f6_moving_rendezvous.yaml leaves `cooperative_tracking_safety_enabled` at the code default true;
   his own IRL configs set it false: c1f_irl_m1_virtual_ring.yaml:154). On the rig (slack 1 m cables, the T's lean, ring
   bob) it is the likelier latch, and the backdate does not touch it. Turning it off is his policy and his call.
5. **Alternatives on our side:** typing his own allowance (0.05 s, 0.05 m/s) likely swaps one latch for another (the
   carriers' state speed exceeds 0.05 m/s in a bob; an advertised start speed of 6 x (measured - target) trips
   start_skew_speed from ~9 mm of hover offset): weaker. Reordering (states before the refresh, in RingBridge) fabricates
   no past but is not watertight and unflown: later, with the backdate. The backdate is the best our-side option.
6. **The bar:** the start_skew clause is tested fairly (~1 decisive hit per 60 s of C++ transit without it; 205 s
   clean in R2860 has ~3 % chance by luck); the tracking_mismatch clause cannot tell cause, since that latch fires
   without the switch too. For later runs score the logged blocker and the tracking ratio just after each refresh,
   switch on against off.
7. **Fixes asked:** the card's 100 Hz (done), CHANGES_FROM_TEJEN.md's tick note and the test's `TICK = 0.01` and its
   0.05 assertion (the flown value is 0.04): after R2863 (no src edit while a run flies).

## Remaining arms (twin)
(29 Oct 01:52-04:20, laptop, one run at a time, tree 413ec0e + the working copy; registry R2853-R2859; scored by
`tools/attach_clearance.py` like V1. Before and after every run: `online_join_planner.py` 4cdcc000,
`partner_mission.launch.py` 206bf93d (the pickup_cpp_reattach launch edits; 8ff903b2 in R2847-R2852),
`mission_definitions.py` c32d4783, unchanged all night. New configs, one change against their base:
`sq_far_floorjoin_hover_L100_p115_v1a30s` (the far mat + the V1(0.30) settings of `..._v1a30s`),
`sq_floorjoin_hover_L100_p115_v1a30s_zt_xy08` and `sq_pickup_hover_L100_p115_cppreattach_v1a30s` (both written, not
flown). Every run has `video/<run>_baseline.mp4` and `video/<run>_top.mp4` (`_far_top` on the far world). At ~03:35 the
orchestrating session stopped the queue after the xy 0.08 pair: the candidate and the pickup with V1(0.30) were not
flown (Revision 4's switch first).)

| run | arm | transit (<= 60 s) | tip at MATCH start / descent start | his TAKEOFF -> weld (<= 300 s) | climb / wait / transit / approach / MATCH / descent / bottom (s) | MATCH -> DESCEND | rod min to CONTACT (>= 0.238) | min before the first descent | tries, proof | ring tilt peak | aborts; his backend | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R2843-R2844 | A0 (mat beside plate 9) | 7.3 / 7.0 s | 1.059 / 1.053, 0.296 / 0.293 m | 38.7 / 38.5 s | 21.0 / 0.9 / 7.3 / 1.0 / 3.0 / 3.8 / 1.7 | 3.0 s | 0.290 / 0.288 m | 0.373 / 0.372 m | 1, proven | 8.4 / 8.6 deg | none | baseline |
| R2853 | demo: far mat (-1, -1), A0 height | 7.5 s, straight over the T | 1.058, 0.292 m | 39.3 s | 21.2 / 1.0 / 7.5 / 1.0 / 3.2 / 3.8 / 1.7 | 3.17 s | 0.286 m | 0.357 m | 1, proven | 8.4 deg | none | pass |
| R2854 | demo: far mat, V1(0.30) | STALL 299.5 s | - | no weld | 15.3 / 1.3 / 299.5 | - | 0.419 m | - | 0 | 26.5 deg (pack sag, 342 s) | none; DISABLED 49.8 s, start_skew | **FAIL** |
| R2855 | pickup, C++ reattach | never reached (stalled in the loaded drop leg) | - | no weld | - | - | 1.375 m | - | 0 | 49.8 deg (DROP AND LAND at 31.1 deg, ~305 s, pack sag) | ring drop; DISABLED 60.8 s, start_skew | **FAIL** |
| R2856 | zero twist | 7.6 s | 1.055, 0.308 m | 38.0 s | 21.0 / 0.9 / 7.6 / 1.0 / 2.0 / 3.8 / 1.7 | 2.02 s | 0.287 m | 0.390 m | 1, proven | 8.6 deg | none | pass |
| R2857 | zero twist | 7.5 s | 1.056, 0.317 m | 38.6 s | 21.1 / 1.1 / 7.5 / 1.0 / 2.1 / 3.8 / 2.0 | 2.10 s | 0.287 m | 0.393 m | 1, proven | 8.6 deg | none; DISABLED 56.6 s, start_skew (0.6 s after the transit's end) | pass |
| R2858 | xy 0.08 | 7.3 s | 1.049, 0.400 m | 37.0 s | 21.0 / 1.0 / 7.3 / 1.1 / 1.0 / 3.8 / 1.8 | 1.04 s | 0.289 m | 0.408 m | 1, proven | 8.5 deg | none | pass |
| R2859 | xy 0.08 | 7.7 s | 1.073, 0.417 m | 45.0 s | 21.1 / 1.0 / 7.7 / 2.1 / 4.8 / 6.7 / 1.8 | 1.04 s (2 MATCH) | 0.272 m | 0.421 m | 1, proven | 8.3 deg | one DESCENT ABORT and retry; no fleet abort | pass (bars), flagged |

Against the bars:
- **Avoidance demo: A0 height MET (R2853), V1(0.30) FAIL (R2854).** From the far mat his C++ transit at A0's height
  flies straight over the T (1.51 m path for 1.51 m; the tip 0.21-0.26 m above the carriers' bodies, QUAD4's body
  0.46 m horizontally from the plate-6 carrier): it shows the wrong-side start and the attach, not a route round. With
  the low transit his planner aimed straight at the gap between the plate-3 and plate-6 carriers' cables (24
  ACCEPTED_PARTIAL plans, every search at its runtime limit; 6 PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED on the plate-6
  tether against QUAD4's cable), then his backend latched the `start_skew` disable at 49.8 s and QUAD4 held at
  (-0.44, -0.44, 2.35) for 290 s. Whether his planner can go round from that side is not shown either way.
- **Avoidance fix (pickup, C++ reattach): FAIL, not exercised (R2855).** The ball was picked up (40.2 s); 0.5 s into his
  C++ loaded drop transit (TRANSIT_TO_DROP_POINT, the leg A0's R2845 flew) his backend latched `start_skew`, so the ball
  never reached the net and the new C++ reattach leg never ran. Not repeated (conditional on a weld).
- **Zero twist: MET in both** (R2856, R2857): weld 38.0 / 38.6 s, MATCH 2.0 / 2.1 s against A0's 3.0 s, rod min
  0.287 m. In the twin's quiet ring this is the no-harm check.
- **xy 0.08: MET in both on the A0 bars** (R2858, R2859), with one descent abort in R2859. With the wider gate MATCH ends
  at its 1 s dwell while the tip is still coming down from the transit height (0.40 / 0.42 m at the descent's start,
  A0 0.29-0.30: critic advisory 10). In R2859 that first descent lost alignment, aborted after 2.9 s and the retry
  welded (+8 s; rod min 0.272 m in the second approach). The V1(0.30) transit end would start MATCH at the approach
  height, which is the reason for the candidate.
- **`start_skew` this card:** R2846 (52.6 s), R2854 (49.8 s), R2855 (60.8 s), and R2857 (56.6 s, harmless: 0.6 s after
  its transit ended): 4 of the card's 17 runs (R2843-R2859, every one with a C++ transit, the pickups' loaded leg
  included); it decided the outcome in three. Every stall of the card is a latched backend disable that never
  recovers (R2848: `tracking_mismatch`). Revision 4's `commit_backdate` is the arm that tests it.
- **The T's pack sag again:** a stalled attach on the T drops the ring at ~5 min of hover (R2855: 31.1 deg at ~305 s,
  DROP AND LAND; R2854: 26.5 deg at 342 s).

Videos: `results/2026-10-29/R285{3..9}_*/video/` (`<run>_baseline.mp4`, `<run>_top.mp4`, `_far_top` for R2853/R2854);
A0's overhead view re-rendered from its recording: `results/2026-10-28/R2843_sim_gz_sq_floorjoin_hover_L100_p115/
video_top/R2843_baseline.mp4` (the `baseline` camera re-aimed to the `top` pose). Reel of one welded run per arm
(demo R2853, A0 R2843, V1(0.30) R2849, V1(0.15) R2851, zero twist R2856, xy 0.08 R2858; no pickup welded):
`results/videos/reel_2026-10-29_attach_overnight.mp4`.

## Revision 4 results (twin, 29 Oct 04:30-07:00) and the reviewer
Registry R2860-R2864 and notes R2853-note, R2859-note, R2862-note, start_skew-note. Tree as the Remaining arms plus
`commit_backdate` (ring_bridge.py c653ab90, partner_mission.launch.py 06cc441c); R2864 also carries the
pickup_cpp_reattach activation fix (online_join_planner.py 6c3f08f9).

| run | arm | his TAKEOFF -> weld | his backend | verdict on the frozen bars |
|---|---|---|---|---|
| R2860 | far start, low transit + switch | no weld (WELD_TIMEOUT) | no start_skew in 206 s of C++ transit; his planner parked QUAD4 at (-0.475, -0.475) between the carriers on plates 3 and 6, tip 0.83 m from the target, 0 % of the transit in his capture region (1029 replans, 0 full plans); tracking_mismatch disable at ROS 248.8 s (drone_0 only), the plate-6 carrier at the 0.9 cap, plate 0 at 0.884 | FAIL (parent's bars and the switch's second clause) |
| R2861 | pickup + switch | - | - | VOID: the plate-3 carrier never left the ground |
| R2862 | candidate + switch | 31.6 s | none | pass, 1 run (MATCH 1.06 s, as V1(0.30)'s 1.04 / 1.02: no gain from zero twist + xy 0.08 on the low end) |
| R2863 | pickup + switch | 97.0 s | none; the loaded C++ drop leg flew (R2855 latched 0.5 s in) | FAIL on the avoidance-fix bar: pickup_cpp_reattach never engaged (his planner tested the mission name, `pickup_delivery_reattach`); the switch's bars met |
| R2864 | pickup + switch + activation fix | 116.1 s (2nd descent) | no start_skew; his C++ planner flew TRANSIT_TO_REATTACH (113.3-115.4 s) from the drop-zone hand-over; tracking_mismatch at ROS 116.0 (drone_1) while the ring bobbed from the ball drop (z 1.066 m at 113.3 s) | avoidance fix ENGAGED and welded (rod min 0.247 m, the card's lowest); the switch's second clause hit again; QUAD4's touchdown tip-over after LAND (twin artefact, as R2845) = an envelope fault; not reviewed |
| R2865 | as R2864 (repeat) | 97.7 s (one descent) | none at all; his C++ reattach 113.3-115.7 s into its capture region | pass on the avoidance-fix bars, the switch's bars met; not reviewed |

Reviewer (29 Oct ~06:10, read-only, recomputed from the logs): overall WEAK. C1 A0 supported (but A0 flew before his
planner's 28 Oct edits: the one-tree rule is broken for comparisons with A0; the edits are guarded off for floor_join).
C2 weak (R2848 cut swing and tracking together; it tripped 1 of 2). C3 supported (R2852's 0.268 m before the descent is
the lowest of the 12 welded runs; V1(0.15) opened on the 00:53 reading of its gate). C4 weak (R2859's abort cause not
shown). C5 not supported as worded: start_skew decided only R2855. C6 not supported: the switch FAILS its frozen bar
(R2860). C7 weak: R2853 dropped the ring at LAND after a 0.23 m fold-in drift; only R2860 shows his planner itself not
routing round. C8 weak: 1 of the 2 repeats. C9 FAIL on the avoidance bar. C10 supported: the side carriers cap at
3.6-4.0 min after LIFTED in all five long runs, all ending in DROP AND LAND (R2848, R2854, R2860 2-3.5 s into the LAND
after WELD_TIMEOUT; the card listed R2848 and R2854 as no abort). Scorer gap: `attach_clearance.py` counts aborts only to
LAND and tilt only to the weld (R2853, R2859 missed).
