# Newcomer approach and weld capture use body-to-tip (rod + pivot drop)

1. **Question.** Does the newcomer's approach put its magnet tip at the plate target, and does the weld capture the
   plate, when the rod hangs from a pivot below the body (rig and twin: 4 cm)?
2. **Hypothesis.** attach_cable_len served both as the body-to-tip length (approach, weld capture) and the rod
   pivot-to-tip (OCP rod, detection). With the pivot 0.04 m below the body the approach parked the tip 4 cm low
   and the weld capture put the tip 4 cm high. body_to_tip = attach_cable_len - pivot_offset_z for the two
   body-referenced uses puts the tip at the target; the OCP keeps the rod (|pivot - rho_new| is then 0.55 against
   the 0.55 rod, it was 0.51: the kind of mismatch behind the R0547 yank).
3. **The one variable.** approach.body_to_tip(rod, pivot_z), used by ApproachProfile and _capture_weld_rho only
   (twin-attach 08d9712 and the helper commit). Legacy (pivot 0): unchanged.
4. **Baseline.** The same twin dry approach with attach_cable_len typed 0.51, so body_to_tip is 0.55 = the
   pre-change approach arm (pivot_offset is fleet-wide and stays): one run, named as the baseline.
5. **Pass / fail** (critic: falsify on geometry, not the total tip error, which mixes in the newcomer's own 2-3 cm
   hover offset, R0653). The runner now records /magnet_tip_pose and /attach_target/pose (tip_*, tip_target_*).
   Report three numbers over the 5 s hold: body - body reference (tracker), measured body - tip (geometry), tip -
   (target - 0.01) (total). Supports if the measured body-to-tip is 0.59 +/- 0.01 m along z and the tip ends within
   0.01 m of the target in z beyond the tracker's own offset; falsified if the geometry term is off by more than
   0.01 m. Plumbing: params.json pivot (0, 0, -0.04) and the approach log line shows body-to-tip 0.590.
   Weld capture: one twin T-R8b (weld_radius 0.08): rho_new z vs the tip at the weld, the fold-in rod length, and
   the newcomer's tilt after the weld (R0548 history).
6. **Repeats.** T-R8a x2 (+1 baseline), T-R8b x1 here (its own gate has x2).
7. **Cost.** Rides on the gate runs (+1 baseline).
8. **New code?** Yes, small, attach reference. Removal: revert the two commits. **Rig: needs Wesley's word**
   (it moves the rig newcomer's parking height 4 cm up; attach geometry).

## Critic
Verdict: code OK, rewrite the card (8 Oct night). Source (fleet pivot_offset) and sign right, legacy unchanged,
no missed body-referenced use in the flight path. Must-fix: measure the real tip (runner records the tip and
target), falsify on the measured body-to-tip (the 3 cm total bar could not separate the 4 cm fix from the hover
offset), name the pre-change baseline, fly a weld run (a dry approach never calls the capture), drop "no ring
contact" from the twin bars (the twin tip cannot touch the ring), Wesley's word before the rig. Should-fix
(open): measure QUAD4 body-to-tip on the rig in R8a0 (0.58 decisions / 0.59 code / 0.575 Tejen's QUAD2); the rig
weld standoff from the capture gap, not the legacy TIP_STANDOFF (the target is 0.05 above the ring origin: 3 cm
above the twin plate, maybe 5 cm on the rig); the partner direct-descent window is 4 cm tighter; capture from the
measured tip instead of body - length (alternative); s_new built from the body (nit).

## Result (8 Oct night, twin dry approach; rig-prep 5130c70, R1247 ce17d4e: a docs-only difference)

| Run | Arm the approach uses | body - body ref (m) | body - tip, measured (m) | tip - (target - 0.01) (m) |
|---|---|---|---|---|
| R1247 (baseline) | 0.550 (attach_cable_len 0.51) | 0.0001 | 0.590 | -0.0401 |
| R1245 | 0.590 (body-to-tip) | 0.0001 | 0.590 | -0.00003 (mean abs 0.34 mm) |
| R1246 | 0.590 | 0.0000 | 0.590 | +0.00001 (mean abs 0.26 mm) |

SUPPORTS for the approach, 2 of 2: the tip ends at the target within 0.4 mm; the old arm parks it 4.0 cm low, as
predicted. The measured body-to-tip (0.590 m) is the twin's physical rod + pivot in every arm, so the fix shows only
in the total term. Reviewer (9 Oct 00:45): supported, approach half only; the weld-capture half (rho_new at the weld,
fold-in rod length, newcomer tilt after the weld) needs the twin T-R8b weld run (batch 6), and the rig QUAD4's own
body-to-tip is still unmeasured (Wednesday's capture calibration), so the twin's sub-millimetre result does not carry
over to the rig by itself. Also seen (outside the scoring window, the land/abort card's bug): at /fleet/landed QUAD4
was still at 0.60-0.64 m, the fleet disarm dropped it and it came to rest inverted (173-179 deg) in all three runs.
