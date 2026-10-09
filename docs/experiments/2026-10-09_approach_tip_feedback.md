# approach_tip_feedback (9 Oct 2026): the newcomer steers its measured magnet onto the target

## 1. Question

Wesley, 9 Oct: "The drone that comes in to attach can use the feedback from its actual magnet position vs
where it is supposed to go to have a better chance of attaching (the magnet is defined in mocap as well).
I saw Tejen had lots of issues with this where the magnet would be directly over but not lower down enough."

## 2. Hypothesis (rev 2, after the critic's REWRITE; Wesley 9 Oct: calibrate + continuous loop)

Two parts, one card:
- **Setpoint.** The target is calibrated, not assumed: on Wednesday 09:00 (props off) QUAD4's magnet face is
  set on plate 9 and the tip marker (body 24) is read in the ring frame (body 8). That point, plus a named
  standoff, is the ONE target for both the approach and the weld check (today they differ: the approach aims
  at rim + 0.05 - 0.01 while the manager measures to the rim point; Tejen's calibration puts the face 2.25 cm
  below the marker). Feedback cannot fix a wrong setpoint; this does.
- **Loop.** With that setpoint, a slow z-only loop on the measured tip removes what the body-to-tip model
  gets wrong (rod length, lean, the drone's own hover offset) and holds the tip there while the rod swings:
  in the twin, with the assumed rod 4 cm off either way, the tip settles within 1 cm of the setpoint in z,
  where the switch-off arm stays 4 cm off; with a swinging rod and a frozen tip pose it does not run away.

## 3. The one variable (rev 3: the critic's rev-2 numbers)

`approach_tip_feedback` (planner parameter, default off). When on, in ApproachProfile:
- **hold**: the carrot is at its descent goal (within 5 mm) for >= 1 s, the target's 1 s mean speed is
  < 0.05 m/s (exit > 0.08 m/s; rig 8 Oct: hover p50 0.022-0.029 m/s, the r305 circle p50 0.117 m/s) and the
  tip is valid. Only in the hold does the loop run; everywhere else c is frozen (no windup on the planned
  0.25 m descent; a circling ring, R9, freezes it);
- **filter**: e_z = setpoint_z - tip_z; the low-pass (tau 1.5 s) runs only inside the hold and starts at the
  0.5 s mean of e_z at hold entry (a filter carried through the descent lags the ramp by ~8 cm, critic rev 2.3);
- **integrator**: c_z += k * e_f * dt, k 0.5 1/s (crossover ~0.42 rad/s, ~58 deg margin; 10x below the rod),
  rate <= 0.02 m/s, c_z in [-0.06, +0.08] m (E+ needs -0.04: rev 2's -0.03 would have saturated it);
- **deadband with hysteresis**: frozen once |e_f| < 5 mm for 2 s; released at |e_f| > 15 mm;
- **contact / catch freeze (latched until the weld or the end of the approach)**: over the last 2 s, |dc| > 3 mm
  and the tip moved < 0.3 |dc| (the tip stops following c: contact, a catch, or a slack tether);
- **tip valid**: fresh (< 0.1 s); |tip - body| in [0.55, 0.63] m; horizontal tip - body < 0.15 m; no step
  > 0.03 m between frames; not frozen (identical over 10 frames while the body moved > 5 mm: a still drone's
  tip may legitimately repeat). Invalid: c held; the body goal stays body + rod + c; never reset mid-approach;
- c resets per approach; dropped at the weld and at a newcomer step-out; starts at 0 after a partner hand-off;
- **setpoint**: `attach_tip_target_z` (m above the ring-origin plane at the plate, default = today's twin point
  rim + 0.05 - 0.01 = +0.04). The weld check's point gets its own parameter `weld_point_z` (default 0.0 =
  today's ring origin + R(x, y, 0), so every other attach run is unchanged); arm W sets it to the setpoint.
  Rig numbers from Wednesday's props-off calibration (rod/tether hanging vertical, face on plate 9): the
  setpoint standoff <= the measured pull-in gap minus 1.5 cm (the deadband/overshoot band); Wesley signs
  them before any rig attach with the switch on;
- **tip rate**: `magnet_tip_publisher` gets `max_rate_hz` (default 0 = unthrottled, today); every arm here,
  baselines included, sets 120 (the rig's mocap cap) so the planner is not fed 500 Hz;
- **logging**: tip_c_z, tip_e_z, tip_e_f, tip_valid, tip_hold, tip_frozen_reason in the planner log, in the
  switch-off runs too (the same hold reference for both arms);
- xy is not corrected; seek stays 0 (its default).

QUAD4's arm on the rig is a tether that goes slack (Wesley 9 Oct); in tension it behaves as the twin's rod, and
the contact freeze is what catches a slack tether (the tip stops following c down).

Pinned: `twin_r8a_dry` and copies; LAND 35 s after the descent starts; one code tree for the table.

| arm | what | config |
|---|---|---|
| 0 | no injected error | twin_tf_0 |
| E- | assumed rod 4 cm short (`attach_cable_len 0.51`): tip ends low | twin_tf_em |
| E+ | assumed rod 4 cm long (`attach_cable_len 0.59`): tip ends HIGH | twin_tf_ep |
| S | E+ plus a swing: a horizontal radial push on QUAD4's body at hold entry + 1 s, sized in one exploratory run to a 3-8 cm horizontal tip - body swing (under the 0.15 m band; not the fault-scope push) | twin_tf_swing |
| F | E+ plus `/magnet_tip_pose` frozen (runner TIP_FREEZE, inert by default) for 2 s from hold entry + 0.5 s, while c moves; frozen samples excluded from tip scoring | twin_tf_freeze |
| W | E+ with the weld at the setpoint (`weld_point_z` +0.04) and `weld_radius` 0.025, rig gate (0.05 m/s, dwell 0.15, rig weld_vel_filter_s): switch-off cannot weld | twin_tf_weld |

## 4. Baseline

Every arm once with the switch off on the same tree (R1245-R1247 are on an older tree). Expected (R1247): E-
ends ~4 cm low, E+ ~4 cm high; W switch-off: no weld.

## 5. Pass / fail numbers

Scoring window = the last 5 s before LAND; convergence scored separately. tools/attach_metrics.py gains tip_goal_z
(tip z minus the setpoint z) and reads tip_c_z and tip_hold from the planner log.

| metric | supports | falsifies |
|---|---|---|
| mean |tip_goal_z| in the window, arms E-, E+, S, F | <= 0.01 m (switch-off ~0.04 m) | > 0.02 m in any |
| arm 0 vs its baseline | within 0.005 m | worse by > 0.01 m |
| time from hold entry to |tip_goal_z| < 0.01 m, arms E-, E+ | <= 15 s | never |
| arm S: c excursion while the rod swings | <= 5 mm | > 15 mm |
| arm S: share of the hold with the tip invalid | reported | - |
| arm F: c over the 2 s freeze | changes < 5 mm | runs > 20 mm |
| tip_c_z at the end | inside (-0.06, +0.08) | at a bound |
| arm W | switch-on welds within 15 s of hold entry | no weld in 30 s |
| envelope faults, aborts | none | any |

Rig (Wednesday): the calibration (above); a props-off record of `/magnet_tip_pose` for 10 s still and with body
24 covered (repeats, precision, what arrives when occluded); a bar that does not depend on body 24: with the
magnet ON the ring follows a small lift (a physical catch), or a side photo shows the face on the plate.

## 6. Repeats

Neighbour pass: every arm once (switch on) after the switch-off baselines; then E-, E+, S x2.

## 7. Cost

6 baselines + 6 arms + 3 repeats + 1 push-sizing run = 16 Gazebo runs (lab PC when idle).

## 8. New code?

Yes: approach.py (hold phase, z loop, validity, freezes), dissipative_node.py (`/magnet_tip_pose` at 120 Hz,
body pose to the validity check, the switch, log columns), the setpoint parameter (`attach_tip_target`: the
calibrated tip point in the ring frame; until Wednesday the twin keeps today's rim + 0.05 - 0.01) used by both
the approach and the weld check, the runner's TIP_FREEZE event (magnet_tip_publisher holds its output),
tools/attach_metrics.py (tip_goal_z, c), bringup plumbing (graph_attach, rig profile). Unit tests: bounds,
rate, hold-only integration (no windup on a synthetic descent), freeze on stale/frozen/stepped/out-of-band
tips, the contact freeze, deadband, resets (weld pop, step-out, partner hand-off). If it fails: rows kept,
code removed. Attach geometry: the setpoint is Wesley's word (given for this approach, 9 Oct; the numbers
come from Wednesday's calibration).

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

Rev 2 (above) takes every must-fix: 1 hold-only integration; 2 one calibrated setpoint for the approach and the weld
check + a rig bar without body 24; 3 validity checks + arm F + the props-off occlusion check; 4 asymmetric bound,
contact freeze, deadband; 5 W on the rig gate, scored on time; 6 arm S, absolute swing floor, moving-target freeze;
7 z only; 8 baselines on one tree, z-scored hold metric, c logged, 35 s hold; 9 tip at 120 Hz; 10 section 8.
Should-fix: H/H+ dropped; the capture (`_capture_weld_rho`) stays its own card; the one-shot trim was offered
to Wesley and he chose the continuous loop; reset tests listed; seek 0 stated. Rev 2 goes back to the critic.

9 Oct, critic on rev 2: **GO WITH CHANGES** (numbers and definitions only; the design stands). Items: (1) E+ needs
c -0.04 against rev 2's -0.03 bound -> down bound -0.06; (2) the contact freeze could not see slow windup -> ratio
test, latched; (3) the filter carried through the descent lags ~8 cm and drives E- the wrong way -> filter only in
the hold, started at the 0.5 s mean; (4) the deadband froze the ~6 mm overshoot and equalled the bar -> enter
< 5 mm for 2 s, exit 15 mm; (5) W could not discriminate (the rig gate welds before the hold if the tip is inside
the radius) -> weld at the setpoint, radius 0.025, switch-off cannot weld; (6) setpoint and weld-point numbers
named, the weld point behind its own parameter with today's default, Wesley signs the rig numbers; (7) decimate
at the publisher, every arm; (8) S scored on z (c excursion, invalid share), push sized and specified; (9) F timed
while c moves, frozen samples excluded; (10) rig thresholds: the moving-target gate re-sized from the 8 Oct rig
hover/circle logs (1 s mean, 0.05/0.08 hysteresis); tip precision/repeats unknown (no rig run logged the tip):
frozen = identical while the body moved, plus Wednesday's props-off record. Should-fix taken: tip_hold logged
switch-off; calibrate hanging vertical; TIP_FREEZE inert by default with a unit test.

