# 2026-09-25 Attach by OCP resize (n → n+1), the mirror of the OCP-resize detach

## 1. Question
The end goal (Wesley, 2026-09-24): a drone leaves the ring during the circle, does a job,
and REJOINS while the circle continues. The rejoin is the attach path. Today every attach
hands the fleet to the dissipative network at the weld (`_do_attach` → `_enter_network_phase`
→ `net.attach`, no OCP mode exists), and the network cannot carry this 0.86 kg ring's
reconfiguration transient: the detach study (card 2026-09-25_detach_layout_mode) aborted
both network arms at 53–61° while the OCP resize settled at 1–2°. The attach demo
(R0234/R0243, 39–47° weld peaks) was last flown on the 0.6 kg disc and the quadratic plant.
Does folding the welded newcomer into the OCP as an n+1 fleet, in place, keep the ring
level through the weld in hover and mid-circle, and beat the network path on the current
tree?

## 2. The change (built 2026-09-25 morning, Wesley remote)
`dissipative_node.py`: `_attach_ocp(d)` runs when `reconfig_mode: ocp` and the fleet is in
the planner phase (the network path is untouched otherwise):
1. weld point = magnet tip (body − `attach_cable_len` ẑ for a reserved drone) in the load
   frame, clamped to the ring radius (`_capture_weld_rho`, shared with the network path);
2. `resize_fleet(n+1, slot2drone + [d], rho = current ring + weld point,
   cable_lengths = current + arm length)` — the pre-built n+1 OCP with the true uneven
   geometry (`resize_fleet` now takes per-slot cable lengths and keeps `cable_len_i`);
3. the newcomer becomes a planner slot: its mocap feeds `drone_pos`/`drone_vel`;
4. solver primed twice; trajectory held (`attach_traj_hold_s`, timed or settle — the
   settle tick now also runs in the planner phase);
5. its cable feedforward ramps 0→1 over `attach_ff_ramp_s` (the existing gate, applied in
   `_publish_ref` for an OCP-attached drone);
6. reserved ids are counted from the fleet size at construction (`_n_carry0`), so an attach
   after an OCP detach addresses the right drone (they were `self.n + j`, which shifts with
   every resize); `_detach_ocp` takes survivors from the live slot map, so an OCP-attached
   newcomer can leave again (magnet OFF release, no DetachableJoint of its own).
Nothing changes for `reconfig_mode: network` (the default) or for a fleet without
`reserved_attach`.

## 3. Variables
A. Mode at the weld in hover: OCP resize vs network, SIL, current tree (0.86 kg ring,
   linear plant, trim on, z_ki 0.4, 330/90/210 + weld at 270, elev 65, hold 10 s timed).
B. Same during the 0.4 m/s circle (weld at 14 s, hold, resume with four).
C. Gazebo: the demo config `attach_circle_n3_settle` with `reconfig_mode: ocp` vs as is.
Falsifiers (any arm): abort; settled tilt > 25° after the hold; load off 0.60 by > 5 cm
settled; newcomer tracking error > 0.3 m after the hold; the circle not resumed.

## 4. Baseline
R0243 (network, settle hold, 0.6 kg disc, quadratic plant: resume tilt 1.2°, weld peak
~40°). The current-tree network arm is the live control.

## 5. Pass / fail
| arm | supports | falsifies |
|---|---|---|
| A ocp vs net, hover | ocp peak tilt < net peak and settles < 5° within the hold | ocp peak > 25° or abort |
| B ocp vs net, circle | as A and the circle resumes with four within 5 s of the hold end | same |
| C Gazebo ×2 | as B on the physical weld (DetachableJoint, mux hand-over) | same |

## 6. Repeats
SIL one per arm. Gazebo 2 for the chosen mode.

## 7. Cost
SIL 4 runs (free). Gazebo 2 (of today's 6, 4 spent by 08:00).

## 8. New code
dissipative_node.py (+~110), planner_node.py resize_fleet (+6), 4 SIL configs. Removal:
delete `_attach_ocp`, `_capture_weld_rho` (inline back), `_reserved_index`, `_ocp_attached`;
keep the `_n_carry0` id fix and the resize cable-length argument (bug fixes).

## Critic
Not run: the change reuses the OCP-resize machinery the detach already flies (no new law
or estimator in the loop; the hold and ramp are the existing attach gates). Wesley can ask
for it. Reviewer on the final table before the rig.

## 9. Outcome, SIL (2026-09-25, current tree, R0518–R0521)
Three bench faults first (R0513–R0517, all void): the stand-in approach still inverted the
quadratic throttle on the linear plant (drone 3 flew to 2.5 m, out of the 1 m warm-reference
radius, its tracker faulted at TAKEOFF); the bench started its scenario clock before the
dissipative node had loaded its solver (the WELD fired into a node that was not listening);
and the weld flag was published once, where the Gazebo magnet manager latches it. All three
fixed in `tools/sil/` (stand-in linear, wait for `/fleet/control_phase`, latched flag).
| arm | mode | weld peak | +8 s | last 8 s tilt | load z | newcomer err | run |
|---|---|---|---|---|---|---|---|
| A hover | **OCP resize** | 10.3° | 2° | 5.4° (growing, see below) | 0.598 | 0.018 m | R0518 |
| A hover | network | 72.6° | abort | — | — | 4 m | R0519 |
| B circle | **OCP resize** | 10.0° | 3° | 5.6° (growing) | 0.513 (circle sag) | 0.06 m | R0520 |
| B circle | network | 74.2° | abort | — | — | 3.9 m | R0521 |
Mode: decided by four arms. The network attach that carried the 0.6 kg disc (R0243) capsizes
the 0.86 kg ring 8 s after every weld; the OCP resize holds it at a 10° peak, in hover and
mid-circle, and the circle resumes with four.
Finding: after the OCP attach the four-ring 330/90/210/270 has a lightly UNSTABLE rocking
mode, ~1.5 Hz about the 90–270 line (the newcomer at 270 carries 2.0 N, the drone opposite
at 90 carries 4.8 N in the balanced split; the even triple carries 2.8 each): amplitude 4°
→ 10° over 30 s in hover (R0518), 4.8° → 8.9° over 40 s in the circle (R0520). The
three-ring's own 1.4 Hz mode holds at 2° (R0505). Trims are frozen, every drone sits on its
reference (err ≤ 0.02 m), tensions are constant: the planner's references carry the
oscillation. Not seen in the Gazebo four-ring hover on 30/90/150/270 (R0510, 0.1–0.9° for
20 s), which is the same ring rotated by 180° with a real rod instead of a magnet arm — so
either the SIL magnet-arm link, the 2.5 cm attach_z mismatch of the weld point (planner
0.025, plant weld 0.05 above the ring plane), or the 10 s hold is the difference. Gazebo
arm C decides whether this is real.

## 10. Gazebo arm C, first run (R0522): the approach, not the attach, failed
`attach_circle_n3_settle` + `reconfig_mode: ocp` on the current tree: the fleet flew the
0.2 m/s circle at 0.662 m with < 1° tilt and landed clean, but the newcomer never welded.
The collaborator's approach MPC (`controller_mpc_payload`, `mpc_thrust_ratio` = the solo
gain, now 83.1 on the linear plant instead of ~30 on the quadratic one) climbed 0.12 → 0.6 m
in 1.5 s, pinned its throttle at the 0.05 floor and rolled into a growing ~2 Hz rate
oscillation (tilt 5, 12, 6, 10, 22, 30, 27, 52, 100° over 1.5 s); drone 3 tilt fault 7.7 s
after MAGNET ON. Its model is linear (a = kT·u) so the number is right; its weights and
rate model were tuned on the quadratic plant and it has not flown since the linearisation.
Not retuned here (the collaborator's package; the end-goal mission will be another
student's controller anyway). OPEN in decisions.md.
Built instead (same morning): the newcomer's OWN tracker flies the approach on a reference
from the dissipative node (`controller_dissipative/approach.py`: rate-bounded carrot to a
clearance point 0.25 m above the tip-on-target height at the locked 0.2 m/s, then straight
down at 0.1 m/s, then a hold that follows the live `/attach_target/pose`); the mux forwards
our stream from the start (`approach_stream: false`) when `enable_approach_mpc:=false`;
the weld is still the magnet manager's proximity test and the attach is the OCP resize.
Three unit tests on the profile. Gazebo run 6 (R0523) flies it.
R0523 (run 6): the tracker flew the carrot to within 0.1 m for 13 s, then the straight
floor-to-clearance line put the body at the ring's altitude 0.36 m from the load centre
with the 0.5 m arm hanging below; the arm swept the ring/rods, the drone dropped 0.4 m and
tipped (drone 3 fault, fleet unaffected: circle at 0.662, tilt < 1°, clean LAND). Profile
changed the same hour: vertical climb to the clearance altitude first, level transit, then
the descent (5 unit tests). Headless budget for the session spent at 6: the re-fly is
Wesley's call (a 7th headless run, or his own interactive run).

## 11. The post-attach rock is the SIL plant, not the attach (R0524/R0525)
A four-ring on 330/90/210/270 started as a four-ring in SIL (no attach) rocks about the
90–270 line at 1.4 Hz, 13° → 17° over 35 s; the even four-ring holds 0.4°. Gazebo flew
the same ring shape (30/90/150/270 = this one rotated 180°) at < 1° for 20 s of hover
(R0510) and 15 s of circle (R0512) the same morning. The mode is an uneven-ring
pendular mode that the SIL plant leaves undamped and Gazebo's joints damp. Gazebo is the
arbiter for uneven rings; SIL stays valid for the weld transient itself (the first 8 s
of R0518/R0520 match the Gazebo detach transients in size). OPEN in decisions.md.

## 12. Runs 7 and 8 (R0526, R0527): the newcomer's own tracker is roll-unstable in free flight
Climb-first profile: the newcomer rolls into a growing ~1.25 Hz oscillation during the pure
vertical climb at (0, −1.2) and tips at z ≈ 0.95, 6 s of sim after the climb starts, both
runs identically. Its lateral acceleration equals its own roll (residual < 0.1 m/s²), so
no external force, no snag, no phantom cable; the 3 g magnet arm is not a pendulum. The
node-0 box inversion (`x0_relax_symmetric`) is not the cause (R0527 identical) and that
flag tipped a tethered drone on LAND, so it stays off. Three things differ for the
standalone drone and are untested: its mocap/pose bridge is the standalone
`/model/x3_drone3/pose` (rate and latency of the sim rate loop's feedback), its tracker
runs `takeoff_spool_s` 0, and no static cable share is published for it (kt_trim input).
The collaborator's approach MPC failed on the same drone in R0522 with a similar roll
oscillation, which points at the standalone drone's sim chain rather than either
controller. Time box reached: stop, Wesley's call.
Checked and cleared the same hour: the newcomer's tracker runs at 50 Hz (not starved); the
standalone pose array's last entry is the model pose, as for the nested drones (so the sim
rate loop and the mocap emulator read the body, not the swinging tip); the magnet arm
is 3 g; no external force. Remaining suspects for the free-flight roll instability, in
order: the tracker's MPC on a drone with NO cable term and a hold-shaped reference (all
N+1 nodes at one point with a 0.2 m/s velocity feedforward) far from the origin, versus
the creep refs the tethered drones fly free on; `takeoff_spool_s` 0 on the newcomer. The
isolating run is drone 3 climbing alone with the fleet on the floor, then the same with
a proper horizon reference. Two runs on this fix spent (R0526, R0527): stop.

## 13. Root cause of every free-drone tip: the sim rate estimate under the throttled clock (R0535)
Runs 9–15 (R0528–R0534) narrowed it: not the reference shape, not the node-0 box, not the
distance from the origin, not the magnet arm (a plain x3 tipped too, R0533), not the outer
controller (MPC, velocity loop and the collaborator's approach MPC all tipped at the same
~1.4 Hz lateral mode). R0535 flew the plain drone through the whole approach and a 60 s
hold at 1–3 cm error with one change: `clock_hz: 0` on the RViz launch. The sim Betaflight
node (`payload_betaflight_comm`, also the two older nodes) estimates body rates by
differencing consecutive poses against the ROS clock; since 2026-09-23 that clock is
throttled to 100 Hz (default `clock_hz`) while poses arrive at 500 Hz, so four of five
pairs have dt = 0 and are dropped and the surviving estimate is ~5× low. Tethered drones
are damped by their cables and flew every 09-24/25 result on that weak rate loop; a free
drone is not damped. Fix (2026-09-25, all three sim Betaflight nodes): dt from the pose
message stamp (Gazebo time from the bridge), clock fallback. Every tethered Gazebo result
since 09-23 was flown with the weak inner loop; the fix restores the pre-09-23 inner loop
under the throttled clock. Regression to check: creep n3 hover (R0510-class) unchanged.
Pre-approach hold: none (the newcomer idles with no reference until ATTACH; every hold
variant failed, R0529–R0532).

## 14. Gazebo, afternoon: the chain fires; the post-weld transient is the last problem
| run | change | approach | weld | after the weld |
|---|---|---|---|---|
| R0537 | rate timing fixed, clock 100 | flown, 0.4 m wobble | no (tip bounced on the plate) | — |
| R0538 / R0539 | plain drone, clock 100 / 500 | 0.53 m / 0.011 m hold error | — | — |
| R0540 | clock 500, tip 0.04 above the +0.05 point | 2 cm hold | no (0.12 from the manager's rim point) | — |
| R0541 | tip 0.04 above the rim point | 2 cm hold | **yes**, 36.4 s, d 0.070 | resize + OCP attach fired; newcomer yanked to its 45° slot at 1 m/s, flipped in 2 s, ring 30° |
| R0546 (SIL) | post-weld reference slew | — | — | hover attach peak 2.1° (was 10.3), settled 0.7° |
| R0547 | slew | 2 cm | yes | calm 2 s, then rod slack → yank, newcomer flipped at +3 s, ring 33° |
| R0548 | + true tip as the attach point, 1 N start | 1 cm | yes | same: flip at +3 s, ring 32° |
Mechanism in R0547/R0548: the planned newcomer height drops 0.5 m in 1 s while its planned
velocity says +0.9 m/s (an inconsistent horizon), the tautness gate falls to 0.46 because
the body measures 0.46–0.48 from its pivot against a 0.50 rod, and the rod yanks (measured
cable pull 1.6 → 6 m/s²). The Gazebo-only feature left is the arm itself: ball joints at
both ends with 0.1 N·m·s/rad damping, which on a 0.0017 kg·m² body turns any arm swing
into tens of rad/s² of body torque, and a base pivot 0.05 m below the body centre that
the OCP's centre-attached cable does not model. R0549 tests the damping with a soft-joint
model copy. Decision Wesley owns either way: the newcomer's arm model (SDF).

## 15. Handoff (2026-09-25 afternoon)
R0549 (soft joints): identical flip → damping cleared. R0550: attach_cable_len 0.55 was
wrong (the sim tip is 0.49 below the body centre); 0.49 from here. R0551 void (pose
watchdog in the lift). R0552 in flight: pivot at the body centre. Decision tree and the
owed runs are in the plan file (sparkling-sprouting-possum.md, HANDOFF section).

**R0552 (pivot at the body centre): SUPPORTED.** Weld 36.6 s, resize, peak 11.6°, 3.6° at +5–10 s, 0.8° from +10 s, circle resumed with four, newcomer 0.5–2 cm, clean LAND. The post-weld flip was the 5 cm pivot lever. Wesley: no hardware change; the lever must be modelled (decisions.md OPEN).

## 16. Why only the newcomer: the sim tethered drones pivot at the body centre
`tether_i_to_drone_i` (ball joint, rod → `x3_drone_i::base_link`) carries no `<pose>`, so it
sits at the base_link origin: every tethered drone in every world hangs its rod from its
body centre. Only the newcomer's `base_to_magnet_arm` pivots 0.05 m below it. The sim was
inconsistent with itself, and R0552 (newcomer pivot moved to the centre) is the consistent
model. What matters for the rig is where each airframe's cable actually hooks relative to
its centre of mass; that is a measurement, not a modelling choice.

## 17. The rejoin claim, canonical model (Wesley 2026-09-25: newcomer arm pivot at the body centre, like the tethered drones; rejoin straight to 45°)
Attach during the 0.2 m/s circle: tracker-flown approach (climb, transit, descent, tip 0.04 above
the rim point), magnet-manager weld, OCP resize n=3 → 4 with the true tip as the attach point and
the 10 s reference slew to the balanced split, circle resumed with four. Clock 500, fixed sim
inner loop, attach_cable_len 0.49.
| run | weld | 0–5 s after | 5–10 s | 10–20 s (circle resumed) | newcomer error | LAND |
|---|---|---|---|---|---|---|
| R0575 | 36.4 s | max 10.3°, mean 5.6° | mean 3.3° | mean 0.8° | 1.6–3.3 cm | clean |
| R0577 | 36.5 s | max 10.5°, mean 5.7° | mean 3.3° | mean 0.9° | 1.5–3.0 cm | clean |
Earlier on the same geometry (test copy): R0552 11.6° peak, 0.8° settled. Void: R0576 (sim stall).
The network attach on this ring capsizes it (SIL R0519/R0521). A passenger stage (rod vertical)
is statically unable to share load with 45° incumbents and was worse in SIL (R0573/R0574):
dropped. For the rig: where each airframe's cable hooks relative to its centre of mass decides
whether the 45° rod puts a lever on the body; the sim now assumes the centre for all four.

## 18. Reviewer on §17: not supported → fixed and re-flown
The reviewer found the §17 newcomer carried NO load (throttle 0.118 → 0.12; planned tension at the
0.1 N floor) and was left armed after LAND. Causes: (1) the newcomer's nominal cable was "45° from its
own attach point"; welded at r 0.27–0.28 its line of action missed the apex where the three
incumbents' 45° lines meet, and a static balance with fixed directions can only load a cable through
that apex (geometry.apex_direction now gives it that direction; unit test); (2) the fleet manager in
the attach launch managed only the three tethered drones (now n + reserved_attach). The mux's
"HOLDING approach authority" line was spurious in the tracker-flown mode (silenced).
| run | weld | planned tensions | throttle pre → post (d0, d1, d2, newcomer) | tilt 0–5 s | 5–10 s | 10–20 s, circle | LAND |
|---|---|---|---|---|---|---|---|
| R0580 | 36.0 s | 2.65–2.72 / 4.45–4.57 / 2.51–2.62 / 1.74–1.83 (10–20 s) | 0.18 ×3, 0.118 → 0.161, 0.190, 0.160, 0.142 | max 10.7, mean 6.4 | 2.5 | 2.6 (max 3.7) | all four disarmed |
| R0581 | 36.6 s | 2.62 / 4.63 / 2.68 / 1.99 | 0.18 ×3, 0.119 → 0.159, 0.191, 0.159, 0.146 | max 11.9, mean 7.0 | 4.6 | 0.9 (max 2.2) | all four disarmed |
SIL with the same fixes: R0578 hover newcomer 2.12 N, peak 8.0°, settled 0.7°; R0579 circle 2.15 N.
The load rides 4–7 cm high in the circle (the height integral is gated off during a trajectory).

Reviewer, second pass (2026-09-25): WEAK. Numbers confirmed; the newcomer carries load (+0.024/+0.028
throttle over its free-flight 0.118, incumbents −0.031/−0.033 together) but the SMALLEST share (drone 1
≈ 2.4×), as the balanced split for 330/90/210 + 270 prescribes. R0580's planned tensions corrected to
the 10–20 s window above. The repeats differ in the newcomer's rod: R0580 elevation 57–61°, 3 cm short
of taut (gate 0.67–0.70) and 2.65° settled; R0581 47–51°, taut, 0.88°. Two variables changed from R0577
(apex direction + fleet manager count n → n+1); the count only touches arm/disarm/abort bookkeeping
(pre-weld throttles identical), but a single-variable run was not flown. Disarm of all four at LAND and
controller_3 flying the newcomer throughout: confirmed.
