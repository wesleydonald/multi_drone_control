# approach_tip_feedback (9 Oct 2026): the newcomer steers its measured magnet onto the target

## 1. Question

Wesley, 9 Oct: "The drone that comes in to attach can use the feedback from its actual magnet position vs
where it is supposed to go to have a better chance of attaching (the magnet is defined in mocap as well).
I saw Tejen had lots of issues with this where the magnet would be directly over but not lower down enough."

## 2. Hypothesis

Today the approach flies the BODY to the target plus a fixed body-to-tip length (approach.py); the measured
tip (`/magnet_tip_pose`, mocap body 24 on the rig) is used only by the weld check. Any error in the assumed
rod (length, lean, marker) or in the drone's own hover (sitting above its reference, R0653) leaves the tip
off the target by that much: 4.0 cm low with a 4 cm shorter assumed rod (R1247). A slow correction from the
measured tip error removes it: the tip settles within 1 cm of the target in every injected-error arm,
without exciting the rod swing (~0.7 Hz for the 0.55 m rod).

## 3. The one variable

`approach_tip_feedback` (planner parameter, default off). When on, in ApproachProfile:
- error e = (target + [0, 0, TIP_STANDOFF_M]) - measured tip, from `/magnet_tip_pose` while it is fresh
  (< 0.1 s old); first-order low-pass, tau 1.5 s (the swing averages out);
- correction c integrates k * e_filtered (k 0.8 1/s): xy from the transit's last 0.15 m on, z in the
  descent and hold; |c| <= 0.08 m per axis, rate <= 0.03 m/s; added to the body goal;
- stale or missing tip: c is frozen (not integrated, not reset): the approach falls back to body + rod;
- the z-only below-target seek (`attach_seek_m`) is off while this is on (it is the crude form of the
  same thing).

Everything else is pinned: configs `twin_r8a_dry` (dry approach to plate 9, weld 0, hover) with the
injected errors below; code tree rig-prep + this change.

| arm | injected error | config |
|---|---|---|
| E- | assumed rod 4 cm short (`attach_cable_len 0.51`): tip ends low | `twin_r8a_dry_b051` (+ switch) |
| E+ | assumed rod 4 cm long (`attach_cable_len 0.59`): tip ends HIGH (Tejen's case) | new copy |
| H | true rod, the newcomer's thrust offset typed 0.01 low (it hovers below its reference) | new copy |
| H+ | the other sign: typed 0.01 high (hovers above, R0653's case) | new copy |
| 0 | no error (the switch must not hurt the good case) | `twin_r8a_dry` (+ switch) |
| W | weld on (`twin_r8b`), E+ error: does it weld | new copy |

## 4. Baseline

The same arms with the switch off: R1245/R1246 (arm 0: tip at target - 0.01 within 0.1 mm), R1247 (E-:
-0.0401 m). E+, H, H+, W baselines are flown once each with the switch off, before the switch-on arms.

## 5. Pass / fail numbers

Over the hold (from 3 s after the descent ends to LAND; tools/attach_metrics.py tip columns):

| metric | supports | falsifies |
|---|---|---|
| mean tip - goal distance (3-D), every arm | <= 1.0 cm | > 2.0 cm in any arm |
| arm 0 vs its baseline | within 0.5 cm | worse by > 1.0 cm |
| rod swing: sd of the horizontal tip - body offset in the hold | <= 1.2 x the switch-off arm | > 1.5 x |
| correction c at the end of the hold | under its 0.08 m bound in every arm | at the bound (the error is bigger than it can fix) |
| W: weld | welds; ring tilt peak within 2 deg of R1266 (2.7 deg) | no weld in 30 s of hold |
| envelope faults, aborts | none | any |

## 6. Repeats

2 per switch-on arm after one neighbour pass (all arms once). Baselines once (new) or existing.

## 7. Cost

4 new baselines + 6 arms x 2 = 16 Gazebo runs (lab PC slots when idle).

## 8. New code?

Yes: approach.py (the correction), dissipative_node.py (subscribe `/magnet_tip_pose`, pass it to step,
the switch), bringup (the switch through graph_attach and the rig profile), unit tests on the correction
(bounds, rate, freeze on stale, swing rejection on a synthetic 0.7 Hz tip). If it fails: registry rows
kept, code removed.

## Critic

9 Oct, critic on rev 1 (design review, before any code): **REWRITE**. Its challenge, condensed faithfully:
- Already tried: `attach_seek_m` (R0655-R0661): the blocker was the 0.05 m/s weld speed gate waiting ~80 s for
  the rod swing, not height (R0660); R0653, the premise, was a legacy world with a stray pickup joint (R0659).
  Tejen's tree has two measured-tip trims not ported (906ee3fb/0138b9e1 rate-limited HIGH/LOW trim, "fixes a
  repeated 5-6 cm tip offset"; 11a0683f `target_alignment_mode`, flown nowhere).
- Tejen's "over but not low enough" has a documented cause: his attach target was published 0.12 m high
  (marker aliasing fef23390; 7 Oct rig: face 106 mm above the plate): a setpoint bug in his planner. On our rig
  the live risk is also the setpoint: TIP_STANDOFF -0.01 on a target +0.05 above body 8 puts the tip MARKER
  4 cm above the ring-origin plane, attach_z 0.0, the face 2.25 cm below the marker (Tejen's calibration):
  the face converges ~1.75 cm above body 8's origin plane, unmeasured against the plate top. Feedback makes
  that gap exact; it does not correct it.
- The twin tip is ground truth and the injected errors constant: any stable integrator removes them, so the
  bars test the implementation, not "feedback helps the magnet attach"; on the rig the tip-goal bar is circular.
- It moves where QUAD4 parks: attach geometry, Wesley's word.
- Twin vs rig: rod lean/swing yes in both (magnet_tip_link is a real link on two damped joints, R1266 tip 2.7 cm
  radially in); marker-to-face no in the twin; tip occlusion/frozen pose no in the twin; tip-plate contact no in
  the twin; weld speed gate 0.35 m/s in the twin vs 0.05 m/s + dwell 0.15 on the rig.
- Pendulum coupling is not the risk at these gains (outer loop 0.73 rad/s, zeta 0.46, ~20 % overshoot, 2 % in
  ~12 s; 5.7x below the rod's 4.2 rad/s).
Must-fix: (1) the error winds up on the planned descent: integrate only in a defined hold (carrot at its goal
>= 1 s, target speed < 0.02 m/s, tip valid), frozen elsewhere; (2) one calibrated setpoint for the feedback and
the weld check, from a props-off calibration (body 24 in body 8's frame with the face on plate 9), plus a rig
bar not based on body 24 (a physical catch); (3) tip validity beyond message age (|tip - body| in ~[0.55, 0.63]
m, xy < 0.15 m, no 3 cm step, not frozen over N frames), a frozen-tip twin arm and a props-off occlusion check;
(4) contact/catch anti-windup (asymmetric bound, at most 2-3 cm down; freeze when the tip stops following c;
freeze within ~1 cm so the reference is still while the gate counts); (5) W cannot discriminate (twin gate
0.35 m/s welds on first entry; E+ baseline sits exactly at the 0.08 m radius): fly W with the rig gate;
(6) the swinging-rod arm Wesley asked for (push-world wrench at the hold start), an absolute swing floor, and
freeze c when |target_vel| > 0.02 m/s (attach_moving, R9); (7) z-only, or test xy; (8) re-fly baselines on the
final tree, score 3-D, write the hold metric (attach_metrics' tip columns are rim distances), log c, hold >= 15 s
past convergence; (9) cap the twin tip at 120 Hz (500 Hz into a 10 Hz planner); (10) section 8: Wesley's word.
Should-fix: drop H/H+ (same z signal as E+-) for the swing and frozen-tip arms; W's tilt reflects the capture
(`_capture_weld_rho`), not the feedback; a simpler alternative, a one-shot trim (2 s still-hold mean error,
rate-limited, re-measure, at most 2 iterations, frozen before the weld; close to Tejen's 906ee3fb); unit tests
for c's resets (weld pop, step-out, partner handoff); seek already defaults to 0.

Status: rev 2 waits on Wesley (setpoint calibration first vs feedback; continuous loop vs one-shot trim).

