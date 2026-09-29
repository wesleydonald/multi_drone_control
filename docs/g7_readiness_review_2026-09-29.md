> **Corrections from Wesley (2026-09-29), read with this review.** Tethered flights were flown on the rig (the claim below that
> no rig flight put load on a tether is wrong; one drone may have detached itself after a fast climb). DISARM has been used on
> the rig many times (it changed on 28 Sep: it now also latches the muxes). Drone 0 sometimes disarmed at takeoff (cause
> unknown; the pre-TAKEOFF gate now grounds the fleet instead). Motive ids: quads 11-14, ring 8, pickup object 6. Airframes
> about 1.2 kg. The combined stack is for later. Yaw spin: fixed in code, card docs/experiments/2026-09-29_rig_yaw_spin.md.
> Short version: docs/rig_readiness_short_2026-09-29.md.

# Real-world readiness report: from free hover to M1 and M2 (review of 29 Sep 2026)

Nothing was edited or launched for this review. Anything marked **[inferred]** comes from arithmetic or a model and has not been measured. Everything else was read in code, logs, the registry or the docs. Numbers are from those sources.

---

## 1. Where things stand

- **The sim is proven for the full chain, at the sim's own gain.**
  - Carry hover: R0721, R0558.
  - Four-drone R4 twin: R0729, ring tilt peak 1.7°.
  - Detach: R0571/R0572 about 3°, R0722 3.04°.
  - Rejoin: SIL R0578 8.0°. Gazebo R0575/R0577 are 10.3–10.5°, just over the ≤ 10° first-weld bar.
  - Orbit rejoin: R0690 2.1°, R0756 3.61°.
  - Full M1: R0733 passes every G1 bar.
  - M2: T0030 and T0031 pass; T0035 failed at 10.25° at the lift, so 2 of 3 are under the 5° bar.
  - The known sim weak spot is "carrier kicks" from rates differenced off mocap. R0757 was a one-off 17° upset at detach.
- **The rig has proven free hover of three airframes, and nothing else.**
  - That was 09-16. Median throttle was 0.451 / 0.446 / 0.459 (d0/d1/d2), sitting 15–20 cm low. That puts kT at about 22 against the typed 24.
  - The fourth airframe (quad4, our drone 3, the M1 newcomer) has never flown on our stack.
- **No rig flight has ever put load on a tether.** This is established for drone 1 from geometry, and it is the fact that matters most.
  - In the loaded launches of 09-16 16:11 and 16:38 and 09-23 13:36 and 13:47, drone 1 reached z 0.59–0.65 m on a 0.47 m rod while the ring stayed on the floor.
  - All drones flew at free-hover throttle (0.435–0.479).
  - The earlier non-lifts therefore say nothing about whether the fleet can lift the ring. The loaded fleet has never been measured.
- **The sim runs with about 4× the throttle authority of the rig.**
  - Sim carriers hold the ring at u ≈ 0.18 (R0726: 0.181), because the sim kT is 83.1.
  - The tracker caps throttle at 0.6 (acados.py:133-136; feedforward clip at :272). The rig free-hovers at about 0.45.
  - No sim run has ever reached the cap. Whether three real drones can hold the 0.86 kg ring under it is the top open question (§4, R1).
- **Several safety paths have never been exercised on hardware:**
  - the DISARM kill switch;
  - the Betaflight failsafe (not recorded anywhere);
  - the mocap forwarder's behaviour on occlusion;
  - magnet hold under a real load.
- **Two rig faults are unexplained:**
  - drone 0 did not move in 4 of 11 loaded launches, with its pose frozen to the millimetre;
  - 2 of 4 creep launches spun, by 180.5° and 330.6°. The creep is now the rig default.
- **The combined stack (Tejen's plus ours) has no hardware launch.**
  - There is no real mode for partner_mission, no agreed owner for the magnets, and no router that feeds both stacks.
  - Tejen's hardware M2 is two-drone only, and its single run on 09-25 stalled.
  - M1 and M2 on the rig depend on pieces that do not exist yet.
- **Time is short.**
  - Six lab Wednesdays remain (30 Sep; 7, 14, 21, 28 Oct; 4 Nov). The logs show one lab day a week so far.
  - The results freeze date conflicts between two documents: THESIS_PLAN:953 says W13 (27–31 Oct), CURRENT_STATE:668 says 7 Nov.

---

## 2. The test ladder to M1 and M2

"NEW" marks a rung inserted because the step before it was too big. **[W]** needs Wesley's word; **[T]** needs Tejen.

| Rung | Proves | Key prerequisite | Pass bar | Time |
|---|---|---|---|---|
| R0a (home) | The exact Wednesday lines start and read back what was typed; the pre-TAKEOFF gate refuses; the between-flight reset brings everything back | Sheet fixes (§6) | Read-back = typed; "TAKEOFF REFUSED" when one pose is stopped; mocap, bag and RViz return after the reset | 1–1.5 h |
| R0a' (home, NEW, optional) | What the stack does in the air when the 0.6 cap binds | A new `configs/sil/carry_hover_n3` variant with plant `thrust_c` 22.4 | Exploratory row. Falsifier: at 0.64 kg the ring holds 0.55 m for 40 s with no tracker at 0.6 for more than 1 s | 30 min |
| R0b (lab, props off) | Motive bodies, identity, radios, magnet channel, preflight GO, abort path | Q3 answered (ring body 8 or 9, tip bodies, forwarder) | Each magnet clicks on the airframe Motive calls body 11+i; preflight GO; every drone disarms in step 5 | 75–90 min |
| R0b' (lab, NEW) | Weights, the headroom prediction, Betaflight config and failsafe, kill-path timing, arm state, CPU and DDS | A scale; a Betaflight `diff all` [T] | Every kill path stops the motors in under about 1 s; failsafe known; each flight controller reports armed; tick max well under 0.25 s | 60 min |
| R2 (props off) | Magnets hold, release and couple; capture gap; pivot without peel | Spring scale; T2 down | Hold ≥ 2× the share, axially and at 45°; no peel through a 0–50° sweep; the rim rises when the drone is lifted by hand; release < 0.2 s | 30 min |
| R1 | All four airframes fly our tracker; kT per airframe with rods on; body frames | R0b, R0b', R2 pass | Tilt < 3°, xy < 0.12 m, yaw within ±15°, clean LAND | 30–45 min |
| R1' (NEW, optional) [W] | A second point on the thrust curve (a mass hung from the rod tip) | Wesley's word | Loaded kT within 10 % of free kT | 20 min |
| R3a | Creep to 45° and HOLD on 1/5/9; tethers coupled; **first measured loaded throttle** | Headroom go (§3); `payload_rest_z` measured; one set of bars | Hand-over < 15 s; rods 45 ± 5°; payload z < 0.15; drone tilt < 25°; coupling shown; HOLD throttle below Wesley's stop value | 2 flights, 30–40 min |
| R3e (NEW, conditional) | First lift on an even four-drone ring (0/90/180/270), for the most headroom | One Gazebo re-fly of `ocp_hover_ground_creep_n4` (R0468 was falsified on the old plant); drone 3 through R1/R2 | As R3b | 30 min |
| R3b0 (split out) | First lift ever; tethered kT per drone | R3a coupled, and settled HOLD throttle under the stop value | Ring reaches its target; no drone at 0.6 for more than 1 s; "[kT dN] measured in hover" prints on every drone; ring tilt peak < 15° | 20 min |
| R3b (card) | Three-carrier claim | R3b0; typed kT = measured | z over the last 10 s within 0.55–0.66; ring tilt peak < 15°, settled < 5°; drone tilt < 25°; lift < 40 s; no fault | 2 flights, 40 min |
| R4 (card) | Four on 1/3/5/9 (the M1 layout); the most-loaded drone in the whole ladder | R3b; plate-9 drone predicted under the stop value | As R3b; settled tilt ≤ 5°; no growing 1–2 Hz ring rock | 45 min |
| R5 | Four-drone orbit, 0.125 m/s, radius 0.5 | R4; orbit centre ≥ 1.5 m from the nets; airborne time ≤ 60 s or the sag model on | Tilt mean ≤ 3° | 30 min |
| R6a0 (NEW) | R4 flown on the detach graph, with no DETACH | attach_z convention [W]; `reconfig_mode:=ocp detach_magnet:=true` typed | As R4; desk DETACH releases the right magnet | 30 min |
| R6 | First in-flight release, 4 → 3 in hover | R6a0 | Peak ≤ 8°, < 2° within 5 s; survivors dip ≤ 0.05 m; freed drone holds within 0.15 m | 45 min |
| R7 | Detach during the orbit | R5, R6; a 0.125 m/s twin (only 0.4 m/s exists) | Peak ≤ 8°; orbit resumes within 5 s | 30 min |
| R8a0 (NEW) | The M1 graph carries on three while drone 3 hovers free outside the ring | Tip bodies (Q3); attach_z on three_attach; settle 1.0 or 2.0 [W] | As R3b; drone 3 holds within 0.1 m; no weld line | 40 min |
| R8a | Newcomer approach, no weld, magnet OFF; downwash | R8a0 | Tip error < 0.03 m over the plate for 5 s; ring tilt < 10° with drone 3 overhead | 45 min |
| R8b | Rejoin weld in hover | weld_radius ≤ R2 capture gap; weld-confirmation decision [W] | First weld peak ≤ 10°; claim runs ≤ 5° | 45–60 min |
| R9 | Rejoin during the orbit | R8b, R7 | G1 bars | 45 min |
| R0c (lab, NEW, props off) | Combined graph: one mocap owner, one writer per radio, magnet ownership, DISARM on real radios | A real mode for partner_mission; P12 router; P14 remap; one identity file; fork merge [W][T] | One publisher per command, weld and tip topic; one process per port; DISARM on every stream within 0.5 s; drone 3 arms at TAKEOFF; 10-min CPU soak | 2 h |
| R10a (Tejen) | His hardware M1 alone, on quad4 | serial_port pinned; decided code tree | His pickup, lift, drop, land | his |
| R10a0 (NEW) | Hand-over on drone 3 in free air, both directions, no ring | R0c | Dip < 5 cm at each switch; no arm flicker; mux switches < 1 s | 30 min |
| R10b | His approach and hand-off onto our weld | R8b, R10a, R10a0; mocap coverage mapped to 1.8 m | R8b bars; abort on a drone-3 pose timeout while on his stream | 45 min |
| R11 | Full M1 from a floor start | R10b, R9; ball, net and stand decided | G1 bars together | 60 min |
| M2a (Tejen) | His two-drone hardware M2 | d9c69415 in the tree he flies; cable 0.531 vs 0.515 settled | His success hold | his |
| M2b | Our real M2 graph flies R4 on a hand-placed fleet | R4; P12; P14; magnet ownership; spool 0 and airborne_start read back | As R4 | 45 min |
| M2c | Two-drone hand-over, hold only | M2a, M2b; one drone-id map; azimuths generated by a tool | Mux switch < 1 s; dip ≤ 0.05 m; abort on dip > 0.1 m or tilt > 35° | 45 min |
| M2d0 (NEW) | Tejen's four-drone join alone | His four-drone stack (does not exist) | His success hold | his |
| M2d | Full M2 | M2c, M2d0, P9 | Ring 0.60 ± 0.05; tilt ≤ 5°; drone tilt ≤ 25° | 60 min |

### Desk (R0a, R0a', R0b, R0b', R2)

- **R0a** is run at home against `fake_mocap --drone-z`, with no radio plugged in and our mocap node not running. It is the first run against the 28 Sep preflight. The newest preflight card is still 20260923_134509.
- **R0a' is optional and limited.**
  - The Gazebo config `ocp_hover_ground_creep_defaults` cannot be used, because SIL runs only `configs/sil/*` from a taut air start (tools/sil/scenario.py:187-194). It needs a new SIL config.
  - Even then it cannot show the R3a HOLD or the floor lift, which is where the cap binds first. It only shows the airborne carry at the cap.
  - It must never run with the rig stack up.
- **R0b amendments to the sheet:**
  - Motive ring origin at the ring centre, +x toward plate 0.
  - Read the resting ring z and tilt, and type the z as `payload_rest_z` (the default is 0.05; the resting gate is rest + 0.05).
  - Tilt and rotate each body by hand to check for flips.
  - Align the heading: tilt each airframe nose-down, then right-down, and compare the flight controller's ATTITUDE line in T1 (elrs_interface.py:252-256) with mocap.
  - Record the P9 bag with the full graph armed, props off, plus 30 s carried by hand.
  - Cover the ring's markers as well as a drone's.
- **R0b' is new** because none of its facts are on the sheet yet. After `fc_probe.py cli`, power-cycle the flight controller before flying (fc_probe.py:15-16).
- **R2 must not run in parallel with the powered R0b steps.**
  - Step 3 toggles the magnets, step 4 needs the drones on their plates, and step 5 spins the motors.
  - Run R2 between steps 3 and 4 with T2 down.
  - Add three things: a hand-lift coupling check, a pivot sweep to 50° with the magnet wire in place, and hold after the coil has been on for the usual pre-flight time.

### Single drone (R1, R1')

- Fly R1 with rods and magnets on, because the carry flies with them. That also settles the open item at tests.txt:99.
- R1 is the first landing with a dangling 0.47 m rod: watch for the rod tip tipping the airframe.
- R1 is **not** a spin check. Both spins happened on tethered creep flights; R3a is where a spin would show.
- R1' (a hung mass) is the only way to get a second point on the thrust curve without the ring. It is optional and needs Wesley's word.

### Carry (R3a, R3e, R3b0, R3b, R4)

- **Correction to the drafts: R3a HOLD is already a loaded-throttle test.**
  - The resting gate zeroes only the cable term in the tracker's prediction model (controller_mpc.py:1172-1174).
  - The throttle feedforward is thrust_vec/m_i, which includes the tension (planner_solver.py:128-131; planner_node.py:10-11).
  - R0726 held u 0.181, the loaded value, with the ring floating about 2 cm (peak 0.065 m).
  - So **the 0.6 cap binds first in R3a**, and the go/no-go from the weights has to come before R3a, not before R3b0.
- **Headroom prediction [inferred].**
  - Predicted loaded throttle is u = u_free·√((1+r)² + r²), with r = (ring share × 0.86)/m_drone.
  - The formula reproduces R0726 (0.179 predicted, 0.181 logged) and R0729.
  - At u_free 0.45, staying at or under 0.60 needs airframes of at least:
    - 0.96 kg for three drones on 1/5/9;
    - 1.12 kg for the plate-9 drone of 1/3/5/9, which carries 39 %;
    - 0.72 kg for an even four-drone ring.
  - The table assumes u_free was measured with rod and magnet on, and a linear thrust map.
  - Tejen's UKF adapted kT to 18–20 while carrying 0.06 kg with 22 typed (experiment-history.md:702-713). That was possibly a different airframe (body 7). If it reflects the airframes, loaded throttle is about 15 % higher than the table. A convex thrust curve would push it the other way. Only R3a measures it.

| Airframe with pack, rod and magnet | 3 on 1/5/9 | 1/3/5/9, plate-9 drone | Even four |
|---|---|---|---|
| 0.64 kg (the sim number) | 0.67–0.70 | 0.71–0.74 | 0.61–0.63 |
| 0.80 kg | 0.62–0.65 | 0.65–0.68 | 0.57–0.60 |
| 1.00 kg | 0.58–0.61 | 0.61–0.63 | 0.54–0.57 |
| 1.20 kg | 0.56–0.58 | 0.58–0.60 | 0.53–0.55 |

- **If three drones are marginal, the better fallback is an even four-drone ring (R3e), not R4.** R4 on 1/3/5/9 is the hardest carry in the ladder. The even ring also avoids the uneven-ring rock seen in SIL (decisions.md:30). It needs a Gazebo twin first.
- **Keep R3b0 as its own exploratory rung:** trim off, `lift_ramp_vel` 0.15. Type the tethered kT it prints for the R3b claim flights. A gap over 10 % from R1's free kT means the thrust map is non-linear.
- **From R4 on, put quad4 (drone 3) on plate 3**, so that R6–R8 exercise the airframe and magnet M1 will use.

### Orbit and detach (R5, R6a0, R6, R7)

- **Airborne time.** The trim freezes once converged, and the battery-sag derate is off. Keep flights to about 60 s or less until `kt_batt_sag_frac` has been fitted from the R1/R3b logs [W].
- **R6a0 separates two firsts:** the first flight of the detach graph and the first in-flight release. The real_dissipative defaults are dangerous:
  - `reconfig_mode` defaults to network (:177). The network detach tilts the ring 26–48° in sim.
  - `detach_magnet` defaults to false (:183).
  - There is no `attach_z` argument, so it flies 0.025 against the sheet's 0.0.
- **R7 has no twin at 0.125 m/s.** Only 0.4 m/s exists (R0554/R0556).

### Rejoin (R8a0, R8a, R8b, R9)

- **R8a0 exists** so that the first `three_attach real:=true` flight, the first free drone next to a carried ring and the first tip body do not all happen in one flight.
- **R8a gathers two sets of data:**
  - Downwash, which the sim does not model: drone 3 over plate 3 at 2–3 heights, magnet OFF.
  - Noise on the weld relative speed, which sets `weld_vel_filter_s` (currently 0.0).
- **R8b needs two decisions before it flies:**
  - weld_radius ≤ the R2 capture gap;
  - whether the magnet may be ON only inside weld_radius, so a physical catch cannot come before the declared weld [W].
- **Added abort:** a catch with no weld line within 0.5 s. A payload-staleness fault is recommended before any fold-in (critic plus [W]).

### M1 with Tejen (R0c, R10a, R10a0, R10b, R11)

- **The rig has one magnet on drone 3, and both stacks think they own it.** Nothing combined flies until R0c passes.
  - Sim gives Tejen's nodes their own /tejen namespace and a separate magnet joint.
  - On the rig his nodes use our global `/magnet/command`, `/magnet/object_attached` and `/magnet_tip_pose`.
  - His LANDING phase commands the magnet OFF (mission_definitions.py:259; online_join_planner.py:5730-5739).
- **R10a0** proves both hand-over directions on our airframe with nothing tethered.
- **R11 needs the physical pieces decided:** the steel ball, the net on the ring (which changes load_mass, inertia and marker occlusion), and the pickup stand.
  - A real object has no passing sim twin: G1b stalls in R0681.
  - Wesley decides whether R11 uses a virtual drop, as Tejen's R10a does.

### M2 (M2a to M2d)

- **M2b flies R4 through Tejen's router.** Fly R4 on our mocap node first, so that any change is the router's.
  - The rig command in the dissipative_launch.py:48-50 docstring omits the M2 driver's `takeoff_spool_s 0`, `airborne_start true`, `cable_len`, `attach_z` and azimuths.
  - With the spool default (0.5 s from 0.5·u), thrust is halved at the switch.
  - Read these back with `ros2 param get` before any hand-over.
- **In M2 the panel button follows Tejen's arming feedback,** because ours is remapped to /ours. The spacebar always disarms (arm_panel.cpp:237-241).
- **M2d0 is his first four-drone flight.** Without it, M2d would be that first flight.

---

## 3. Wednesday 30 Sep: the realistic plan

**Tuesday, at home:**
1. Fix the sheet (§6). About 45 min.
2. Ask Tejen for:
   - airframe weights with pack, rod and magnet;
   - a `diff all` per quad;
   - the Motive forwarder program, and whether it can send frame timestamps;
   - the ring body id (8 or 9);
   - the tip bodies.

   If the weights come back, fill in the headroom table that night.
3. Run R0a. Optionally run R0a' (two exploratory registry rows).
4. Count and label the packs, and count the chargers. Quarantine the pack that fell to 16.9 V on 09-23; nobody recorded which one it was. Budget about 19 pack-uses for R1 to R3b, and every flight needs a pack at ≥ 24.0 V at rest.
5. Back up the logs and docs off the laptop. `MDC_BACKUP_DEST` is unset, and the last backup, on 08-05, went to the same disk.
6. Wesley's decisions: the stop value for HOLD throttle, and whether to raise the cap (§5).

**Wednesday (about 5 h):**

| Time | Step | Go/no-go at the end |
|---|---|---|
| 0:00–0:45 | Laptop on a fixed table outside the cage; TX modules and hub strain-relieved. Motive: check calibration, ring origin at the centre, measure rest z and tilt → `payload_rest_z`, hand flip test and heading check. Weigh the ring and each airframe; tape-measure the rods and hook offsets → wed.env; prebuild the planner; compute predicted throttle per drone. | **Tethered flight today?** Predicted HOLD u ≤ the stop value (the drafts propose 0.52–0.57; Wesley picks) → R3a and R3b0 possible. Predicted 0.57–0.65 → R3a only, as a measurement, with LAND if u sits at 0.6 for more than 2 s; no R3b0. Predicted above 0.65 → no three-drone tethered flight; plan R3e or the cap decision. |
| 0:45–2:30 | R0b steps 1–3 (T1 up, `ls -l /dev/QUAD*`, magnet identity). Then R2 with T2 down (hold, peel sweep, hand-lift coupling, capture gap). Then R0b steps 4–5, fixed, and R0b' (failsafe and rxfail on the magnet channel; time every kill path, armed with props off; Telemetry.mode after ARM; `ros2 node list`; 3–5 min armed soak; the P9 bag). | **No props on unless:** every kill path stops every motor in under about 1 s; the failsafe is known; every tether lifts its rim by hand; every flight controller reports armed; the forwarder does not repeat a frozen pose when markers are covered. |
| 2:30–3:15 | R1: four airframes, rods on, one at a time or ≥ 1 m apart. | Tilt < 3° and xy < 0.12 m on each airframe, or its body frame is wrong: fix it before it goes near the ring. Record kT and pack V before and after, per airframe. |
| 3:15–3:30 | Charge and swap packs; the freshest go on the drones predicted to be most loaded. | — |
| 3:30–4:15 | R3a, two flights. | **R3b0 today only if:** coupling is shown (HOLD throttle clearly above R1, z not above reference, drone-to-plate ≤ rod + 3 cm), no spin, and settled HOLD u under the stop value. Two flights without coupling means stop and report (working rules: time box). No retuning on the rig. |
| 4:15–5:00 | R3b0 if the gates above hold, then R3b flights 2–3 if packs and time allow. Otherwise use the time for R1' or unfinished R0b' items. | One exploratory registry row per flight. |

**Realistic target:** R0b, R0b', R2, R1 and R3a. R3b0 is a stretch that depends on the weights. The R3b claims and R4 (or R3e) go to 7 Oct.

**Suggested visit map [inferred; Wesley decides, with descope triggers]:**

| Visit | Rungs | Descope trigger |
|---|---|---|
| 30 Sep | R0b, R0b', R2, R1, R3a (R3b0 stretch) | — |
| 7 Oct | R3b0/R3b, then R4 or R3e | No R3b pass by here → the floor becomes a hardware carry only |
| 14 Oct | R6a0, R6, and R5 if time | R6 here reaches the CURRENT_STATE descope floor ("hardware carry and detach") |
| 21 Oct | R8a0, R8a, R8b | — |
| 28 Oct | R0c, R10a0 (and R9 or R7) | Only if the Tejen-side pieces exist |
| 4 Nov | R10b/R11 or M2b/M2c | Only if 4 Nov is before the freeze |

---

## 4. What could go wrong because real life differs

Ranking is likelihood × impact. Established facts and **[inferred]** are marked.

| # | Risk | Why the sim hid it | Consequence | How you'd see it | What to do | Retired at |
|---|---|---|---|---|---|---|
| 1 | **Throttle headroom.** Loaded throttle at or above the 0.6 cap. | Sim kT 83.1 puts the load at u ≈ 0.18. The rig free-hovers at about 0.45; no sim run ever hit the cap. | No lift, or a one-sided lift with the ring tilted. kt_trim cannot learn above 0.6 (controller_mpc.py:989), so no kT line prints. | u2 ≥ 0.599 in the tracker log or `[diag]`; drone z below its reference; no "measured in hover" at LAND | Weigh first and compute per drone. Stop value on HOLD u. Freshest pack on the most-loaded drone. Consider R3e. Raising the cap is a solver constant plus a rebuild [W], only after throttle_limit and motor_output_limit are read. | R3a (measured), R3b0 |
| 2 | **Kill path untested.** Betaflight failsafe unknown; single-drone loss not escalated; the sheet's reset kills T1. | Sim has no radios and no failsafe; one-shot commands were lost in sim (R0602, R0612 and others). | A late or missed stop; one drone drops while the others keep carrying. The R0b ESTOP check passes without testing anything. | R0b' timings; `diff all` failsafe and rxfail; after clean_slate, `/drone_0/motion_capture_state` goes silent | Time every kill path, armed, props off. One named person on the spacebar with RViz focused, one backup on T2 Ctrl-C. Fix the reset procedure (§6). Rule: ESTOP if any drone drops. Heartbeat or link-loss escalation needs critic + [W]. | R0b' |
| 3 | **People next to live, radio-linked drones.** | Not a sim concept. The magnet latch needs T1 up, so drones are placed on the plates with the link live. | Injury | — | Safe-to-enter state: T2 down, or DISARMED on the panel and in telemetry. Packs unplugged before handling tethered drones. Props on last. ARM called aloud with the door shut. Never hand-catch. Eye protection. | R0a (sheet), from R1 on |
| 4 | **Tethers not coupled.** | The Gazebo weld is perfect. | Another non-lift with no usable data. A partial coupling gives a one-sided lift while an uncoupled drone climbs. | Drone-to-plate distance > rod + 3 cm; HOLD u not above R1's | R2 hand-lift check. `magnet_initial:=ON` read back 'ON' per radio. Bag `/drone_{0..3}/magnet`. Channel 6 confirmed on every quad (so far only drone 0). Coupling is a LAND rule in R3a. | R2, R3a |
| 5 | **A drone is commanded but does not fly.** Seen in 4 of 11 loaded launches, drone 0, pose frozen to the millimetre, u2 0.37–0.54 for 1.3–11 s. | Sim drones always respond. | One-sided lift while a tethered drone sits on the floor. The logs cannot tell a disarmed flight controller from a frozen pose. | Telemetry.mode (parsed but never read, elrs_interface.py:139); motors spinning after ARM with props off | Check motors and mode per drone at R0b'. Read the arming flags and runaway_takeoff. Keep the 5 s manual rule. Video plus a Motive .tak take to diagnose. | R0b', R3a |
| 6 | **Yaw spin on the creep floor start.** 2 of 4 creep launches spun (180.5°, 330.6°) with the yaw command saturated; not a mocap glitch. | Sim momentConstant is 0.2 vs 0.01–0.0168 stock (x3_drone0.sdf:273) [inferred: about 10× more yaw authority]; the creep is simplified. | No hand-over, a spin, an abort. The cause is open. | Yaw > 45° (ESTOP); `yaw_excursion.py`; "arc creep timed out" | R3a before any lift; a ±30° yaw step per airframe at R1; `creep_vel` 0.1 if the arc lags. A Gazebo copy with momentConstant 0.016 plus a tether [W]. | R3a |
| 7 | **Mocap dropout or frozen pose; the ring pose has no watchdog.** | The emulator never drops or freezes. | Fleet disarm from height, or flying blind on a repeated pose [inferred]. A drone-3 timeout while on Tejen's stream is only warned (main.py:213-221). | Which line fires when markers are covered at R0b; bag dt gaps or identical consecutive poses | Cover both a drone and the ring at R0b. **No flight if the forwarder repeats frozen poses.** Operator ESTOP on a frozen ring. A payload-age fault needs critic + [W]. | R0b |
| 8 | **Two stacks on one airframe.** Identity maps differ; two writers per radio. | Sim namespaces separate everything. | His router publishes `/drone_0/motion_capture_state` for body 12, ours for body 11. His relative `ELRSCommand` under /drone_0 reaches our QUAD1 radio. **One topic, two airframes.** Neither radio node opens its port exclusively. | `ros2 topic info -v` shows more than one publisher; `lsof /dev/QUAD*`; flapping "Armed:" lines | One identity file read by every stack. P14 remap. One radio node per port. Pin serial_port in his M1 config (it opens the first ttyUSB). Change the real_io default to QUAD(i+1) [W]. | R0c |
| 9 | **One magnet, two owners.** | Sim has two DetachableJoints and a /tejen namespace. | False fold-in; the pickup never grabs; his auto-land drops drone 3 off the ring. In M2 our radios latch magnets ON from boot. His MPC drives channels 5–10 high when armed. | One publisher per weld topic; a props-off magnet sequence against a steel plate | His nodes under /tejen with `attach_ready_land_after_s 0`; an authority-gated relay (critic + [W]); magnet OFF at boot in partner_m2 [W]; agree it with Tejen. | R0c, M2b |
| 10 | **One-point, fleet-wide kT.** | Sim is linear by construction (decisions.md:20). | Height offset; about 5–8° of ring tilt from a 3–4 % spread between airframes before the trim converges [inferred]. The loaded kT may differ either way (§2 carry notes). | Tethered kT at R3b0 vs R1's free kT > 10 % apart; per-drone z − ref spread > 3 cm | kT per airframe at R1 with rods on. Type the tethered value for the claims. Per-drone thrust_ratio [W]. | R1, R3b0 |
| 11 | **Pack sag and pack state.** 09-23 packs started at 21.8–22.4 V, below both the 22.8 V bar and the 24.0 V bar; drone 1 ended at 16.9 V. | Sim gain is constant. | Fleet sinks late in a flight; the weakest drone tilts the ring; a damaged pack fails. | u2 creeping up at constant z after "converged"; battery_v slope | ≥ 24.0 V, matched within about 0.2 V; pack IDs logged per flight; ≤ 60 s airborne; fit `kt_batt_sag_frac` before R5 [W]. | R1, R3b |
| 12 | **Motive body definitions.** Drone 1's body flipped on 09-16: 39 samples above 90° (121–172°) over three launches. Body heading has never been aligned. | The emulator is ideal. | Envelope fault from height; a heading error rotates every roll and pitch command [inferred]; a symmetric ring can solve rotated. | Hand tilt and rotate in Motive; flight controller ATTITUDE vs mocap; R1 tilt < 3° | Asymmetric marker sets. Re-create a body only with the airframe shimmed level. Check heading. Orientation-jump reject needs critic + [W]. | R0b, R1 |
| 13 | **Rod and magnet mechanics.** Real magnets let go under load: Tejen lost attachment in 4 of 5 loaded lifts on 09-18. | Rods of 1–3 g, frictionless joints, an unbreakable weld. The pivot was moved to the CoM in sim; the hardware was not. | A rod releases or peels at 45°. The magnet wire stiffens the joint. Tejen's config has the anchor 0.04 m below the body origin, which is in the sim flip range (decisions.md:33-36). Rod-tip landings; snags. | R2 sweep and side pull; a roll/pitch bias at the hold; a growing 1–2 Hz rock | R2 as amended. Measure hook offset, rod mass and swing decay. R4 abort on a growing rock. A Gazebo twin with the measured lever before R8b [W]. | R2, R4 |
| 14 | **Launch-argument fallbacks.** | Sim configs type everything. | Network detach (26–48° in sim); a 2.5 cm attach_z error; half thrust at the M2 switch; wrong plates. | Preflight read-back; `ros2 param get` | One attach_z convention [W]. Preflight FAIL on `reconfig_mode` ≠ ocp for detach rungs (tool). Copy the full M2 driver argument list into the sheet. Generate azimuths with a tool. | R0a, R6a0, M2b |
| 15 | **Weld detection is geometric only.** Tip within 0.08 m, relative speed ≤ 0.05 m/s unfiltered, 0.15 s dwell; no magnet feedback. | Sim geometry is exact. | False fold-in, or a physical catch that is never declared, and the ring gets yanked. The 0.05 m/s gate sits near Motive noise [inferred]. | R8a relative-speed noise; tip-to-plate distance in the ring frame | weld_radius ≤ the capture gap; filter set from R8a; abort on "caught, no weld within 0.5 s"; post-weld confirmation (card, critic). | R8a, R8b |

**The rest, one line each:**
- **P9 mocap velocity estimate.**
  - The rig adds an EMA (about 42–67 ms at the effective 60–95 Hz), w rounding and processing-time differencing to what caused the sim kicks.
  - The preflight Hz figure is a lower bound.
  - Decide from the R0b bag with the graph armed plus a hand-carried segment. Any estimator change needs critic + [W].
- **Betaflight config and latency.**
  - No `diff all` exists. The rates changed to 100/100 after the only passing hover.
  - About 60 ms lag was fitted from the 09-16 logs [inferred].
  - TPA or airmode could matter at 0.55–0.6 throttle. Dump everything at R0b'.
- **Geometry and mass.**
  - Rod fits range 0.43–0.53 m; the ring origin is 4.3–4.6 cm off centre; masses and ring CoG are unmeasured.
  - In sim, a 4 cm rod error cost 20–22 cm of height (R0286, R0310).
  - Weigh, measure, and re-run preflight until the rods agree within 3 cm.
- **Partner code tree.**
  - d9c69415 is not in our fork. His repo still has the mocap `return None` bug and lacks the stall fix; 7 of his hardware tests fail in our fork.
  - Decide which tree flies each rung.
- **Downwash onto the ring and the carriers** [inferred]. Measure at R8a; approach radially.
- **Combined-stack prerequisites do not exist:** a real mode for partner_mission, the P12 router, a four-drone M2. These block R0c and everything after it.
- **Laptop and DDS.**
  - Pose watchdogs run on the wall clock at 0.25 s; max tick gaps on 09-16 were 63–65 ms.
  - The rig is on DDS domain 0.
  - Soak the graph at R0b'. A private ROS_DOMAIN_ID needs [W] and Tejen to match.
- **kT typo.** Only kT > 0 is checked (real_mode.py, preflight.py:168). A range FAIL in preflight is a tool change [W for the range].
- **TX and laptop physically disturbed.**
  - Tejen's drone fell after someone nudged the laptop.
  - Our radio node blocks in a reconnect loop on a serial error (elrs_interface.py:175-194, :333-337).
  - Strain-relieve the TX modules and hub; keep hands off the laptop in flight.
- **Tejen's udev install overwrites 99-elrs-quad.rules**, leaving only QUAD2 and QUAD4 (notes_for_tejen.md:17). Check `ls -l /dev/QUAD*` every visit.
- **No record of the flown code.**
  - Rig log directories hold no git SHA.
  - tools/, configs/ and docs/ are untracked and local-only.
  - Log HEAD, `git status --short`, and a tarball of tools/ and configs/ per flight.
- **Rig flights cannot be scored by metrics.py** (it reads only sil.csv or run.csv, metrics.py:649-681). A converter is needed, plus the missing bag topics.
- **No video, blackbox or Motive take.** Two faults (the drone-1 jump and drone 0 on the floor) are undiagnosed from logs alone. Blackbox would also show motor saturation hidden behind the 0.6 command cap.
- **Documents conflict** (§6): a wrong line could get flown, or an R3a pass could be judged a fail.

---

## 5. Decisions and inputs needed from Wesley

**Hardware facts (lab or Tejen):**
- Mass of each airframe with pack, rod and magnet. The ring's mass and CoG, weighed on three scales.
- Rod lengths, and the hook offset below each airframe's CoG. Tejen's config says 0.04 m, which is in the sim flip range.
- Pack count, charger count, pack labels.
- Per quad, a Betaflight `diff all`:
  - failsafe_procedure and failsafe_delay;
  - rxfail on throttle and the magnet channel (does the magnet drop on failsafe?);
  - throttle_limit and motor_output_limit;
  - TPA, thrust_linear, airmode;
  - arming flags and runaway_takeoff;
  - blackbox flash.
- Who owns the flight-controller tune.
- Motive:
  - which program forwards to UDP 1511, and can it send frame timestamps;
  - the ring body id (the code default is 8; tests.txt:48 types 9);
  - tip bodies 21–24;
  - the calibration result and camera count.
- Floor mats or not, decided **before** `payload_rest_z` is measured.
- Equipment: mass scale, spring scale (still a TODO), tape, feeler gauge, spare props, rods, magnets and markers, phone mounts.

**Parameters to type (measured on the day):** `payload_rest_z`, kT per airframe (R1, then tethered at R3b0), DM3/DM4 and RING in wed.env, weld_radius from the R2 capture gap.

**Decisions:**

| # | Decision | Default the drafts propose |
|---|---|---|
| 1 | Stop value on the R3a HOLD throttle for going to R3b0 | 0.55 (the drafts ranged 0.52–0.57) |
| 2 | Raise the 0.6 cap? It is a tracker constant (acados.py:135, :272; velocity_loop.py:93), a kt_trim band (controller_mpc.py:989), and a solver rebuild | Only after the flight-controller limits are read and R0a' has run; pair it with a kT plausibility FAIL (e.g. 18–30) |
| 3 | Even four-drone ring (R3e) as the fallback first lift | Yes, if three drones predict above the stop value; needs one Gazebo n4 re-fly |
| 4 | attach_z convention for R6 onward | Add the argument to real_dissipative and three_attach, or put the Motive origin at the CoM and type 0.025 everywhere |
| 5 | settle 1.0 or 2.0 on three_attach `real:=true` | Match R11's twin (1.0) |
| 6 | DDS domain for the rig | Private domain, with Tejen matching |
| 7 | Payload-staleness fault, drone-loss escalation, orientation-jump reject | Critic plus [W]; before any fold-in |
| 8 | Magnet ON only inside weld_radius; a post-weld confirmation | Card at R8b |
| 9 | partner_m2 rig defaults: spool 0, airborne_start true, magnet OFF at boot plus relay | Refuse to launch without them |
| 10 | Change the real_io default to QUAD(i+1) (a launch default) | Yes |
| 11 | Per-drone thrust_ratio; `kt_batt_sag_frac` | Decide from the R1 and R3b logs |
| 12 | Two or three claim flights? THESIS_PLAN:483 and metrics.py:18 say 3; card §6 says 2 | Wesley |
| 13 | Freeze date (27–31 Oct or 7 Nov) and the descope floor vs G7 | Wesley |
| 14 | Which code tree Tejen flies; his tests or his configs as the truth | Merge a7524bae..d9c69415, send him our two fixes |
| 15 | R11 with a real ball or a virtual drop | Wesley |
| 16 | First lift target_z 0.35 instead of 0.60 (needs one twin); the R1' hung mass | Optional |

**What the loop can prepare before Wednesday without the rig** (sheet and tool edits on Wesley's OK; nothing committed without his word):
- The sheet fixes in §6, plus:
  - a cage-safety block;
  - a LAND-vs-ESTOP rule (LAND for slow problems: pinned throttle, drift, sag, ring above 0.7 m);
  - a drop and crash recovery checklist: wait for DISARMED, unplug packs, free the rods, inspect, re-check the Motive bodies, one free hover before the next tethered flight;
  - a pack log;
  - a code-state line per flight.
- R0a at home against fake_mocap.
- R0a' in SIL: a new `configs/sil/carry_hover_n3` variant with `thrust_c` 22.4 and two arms at drone_mass 0.64 and 1.0; exploratory rows.
- A headroom calculator: weights and layout in, predicted u per drone out, in both kT cases.
- One Gazebo re-fly of `ocp_hover_ground_creep_n4` as an even-ring R3e twin. It counts against the 6-run headless limit.
- Tools:
  - a `clean_slate` rig option that spares T1 and T4;
  - preflight FAILs on a kT range and on `reconfig_mode` ≠ ocp for detach rungs;
  - a rig-log-plus-bag → run-directory converter for metrics.py;
  - `kick_events.py` adapted to rig logs.
- A draft identity file (ours, Tejen's M1, Tejen's M2), and a message to Tejen covering:
  - weights, `diff all`, the forwarder;
  - the udev overwrite;
  - magnet ownership;
  - P14;
  - serial_port;
  - the two fixes for his tree.
- The backup path, which needs a destination off the laptop from Wesley.

---

## 6. Stale items in the current sheet and configs, with the fix

**tests.txt**

| Where | Stale | Fix |
|---|---|---|
| :13, :21-22 | "Ctrl-C T2, clean_slate, relaunch T2. T1 stays up." clean_slate kill -9s the mocap and radio nodes, RViz (the DISARM panel) and the bag (clean_slate.sh:42-65), and deletes the Fast-DDS shared memory (:101). | After clean_slate: relaunch T1, then T4, then T2, then preflight `--real` again. Or add a rig option to clean_slate. |
| :87-89 (R0b step 5) | Tests the pose timeout, then ESTOP after clean_slate on drones that were never armed. The DISARM kill switch is never tested and the ring markers are never covered. | Require "Armed - waiting for TAKEOFF" on every tracker first. Record which line fired. Cover the ring for 2 s. Test the spacebar and the panel button. Timings go to R0b'. |
| :15-16 | `ros2 topic pub --once` for ARM, TAKEOFF and ESTOP; one-shots were lost in sim. | The primary kill is the spacebar with RViz focused. A CLI ESTOP publishes repeatedly (`-t`/`-r`). |
| :17-19 | Every global abort is ESTOP. | Add LAND rules for slow problems. Add "ESTOP if any drone drops". |
| :46 | "Re-create each body with the airframe LEVEL." The resting tilt is physical; the bodies hover at 0.7–2.4°. | Keep the bodies unless the airframe is shimmed level. Add the heading check. Treat the rest-tilt line as information. |
| :48 | Example `mocap_payload_body_id:=9`; the code default is 8. | Confirm in Motive (Q3) and type it. |
| :55-57 | DM3/DM4 0.64 are the sim X3 airframe. | Weighed values before the prebuild. |
| :68 | The bag lacks `/drone_{0..3}/magnet`, `/rosout`, `/magnet/object_attached`, `/payload/desired_position`; T1 is not tee'd. | Add them; tee T1 to `results/rig/2026-09-30/t1.log`. |
| :71-76 | The P9 rest bag is recorded with T2 down. | Record armed, props off, plus 30 s carried by hand. |
| :91 | R2 "in parallel with R0b" clashes with steps 3–5. | Run R2 between steps 3 and 4 with T2 down. Add the hand-lift coupling check, the pivot sweep, side and 45° pulls, and hold after the coil has been on for the pre-flight time. |
| :99 | "TODO(lab) rods on or off" | Rods on. |
| :128 vs ladder:223, :482 vs card :103 | R3a bars disagree: payload z < 0.15 / < 0.07 / ≤ 0.10; drone tilt < 25 / < 15. The card calls "ring lifts in HOLD" a FAIL. | Keep the sheet's bars (z < 0.15, tilt < 25; the twin R0726 floats about 2 cm, peak 0.065, drone tilt 18.8°). Align the ladder and the card. Add the coupling check and the HOLD-u stop value. |
| R3a/R3b T2 lines | `payload_rest_z` is not typed (default 0.05). | Type the value measured after the Motive fix. |
| :141-142 (and :219-221) | "Known hole: typed > 14 % low" is recorded as CLOSED by v2 (decisions.md:19). | Keep the "ring above 0.7 m: LAND" abort; drop the claim that the hole is live. |
| :7-8 | "Tejen's runners use the same /dev/QUAD2" | His M1 radio opens the first `/dev/ttyUSB*`; pin serial_port. |
| :170-386 | HARDWARE SETUP, SIM and older rig notes are obsolete: magnet channel_10, the network detach, two-drone carry lines, `rio num_drones:=2` with `rctl num_drones:=3`. | Move them out of the file; only the WED section goes to the lab. |

**Other files**

| Where | Stale | Fix |
|---|---|---|
| Card 2026-09-30_rig_carry_hover.md :4, :103 | "hand kill TODO Q4" (answered on 09-28); "ring lifts in HOLD = FAIL" | Update both. Add a pack-ID column and an observations column. |
| Ladder §1/§4/§6, :125 | P1, P3, P6, P7, P8, P10, P13 and P15 shown open (done). Q2, Q4 and Q5 open (answered). The R4 command uses real_dissipative, which has no attach_z. The pre-TAKEOFF gate is missing. | Refresh. Add the rungs from §2. |
| docs/experimentation/real_attach_gap.md | Presents real_attach_launch.py, which now refuses to start (:155) | Mark it superseded. |
| CURRENT_STATE.md §9 (:698-717), §7 | Bodies 10/20/30 (now 11–14); channel_10; QUAD1/QUAD2 sharing a serial (udev now has four unique serials); load_mass 0.1. §7 says a pre-takeoff disarm is unhandled (fixed in 189b6c6). | Update. |
| real_io_launch.py:3, :155-158; udev header | Defaults drone i to /dev/QUAD{i}; real_mode maps it to QUAD(i+1) (:139-142). | Change the default [W]. Until then, keep typing the serials. |
| real_control_launch.py:40 | Docstring says load_mass defaults to 0.1. | Say 0.86. |
| real_dissipative_launch.py :45-46, :49-50, :52-53 | Docstrings say "FIXED" kT (kt_trim is on), load_mass 0.1 (actual 0.86 at :72), handover_elev 0.0 (actual 45). | Correct the docstrings. |
| real_dissipative_launch.py :77, :177, :183 | settle 1.0, `reconfig_mode` network, `detach_magnet` false. A trap for R6 via `rdiss`. | Type `reconfig_mode:=ocp detach_magnet:=true` in the sheet; preflight FAIL on network; change the defaults on Wesley's word. |
| dissipative_launch.py:48-50 | The rig M2 command omits spool 0, airborne_start true, cable_len, attach_z and the azimuths. | Copy the drive_m2_handover.py:131-146 argument list. |
| THESIS_PLAN:953 vs CURRENT_STATE:668 | Results freeze in W13 (27–31 Oct) vs 7 Nov | Wesley picks one. |
| THESIS_PLAN:483, metrics.py:18 vs card §6 | 3 vs 2 real repeats | Wesley picks one before the R3b claims. |