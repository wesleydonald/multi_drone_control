# Card: does the rig-timing twin swing QUAD4's magnet like the rig? Then Tejen's timing fix (29 Oct 2026)

Owner Wesley (decisions.md, 29 Oct ~15:10: "first add the rig's delays to the twin ... so it reproduces the rig's 2-5 deg
in-flight swing and Tejen's timing fix can be judged offline"; this plan approved ~15:50, the triangle world included).
Exploratory rows under this card; no default changes. Written 29 Oct ~16:00 AEDT, before any run (the first run started 16:03).

## 1. Question
Does the twin given the rig's delays reproduce QUAD4's in-flight magnet swing and its magnet-to-plate error on the
triangle attach that the rig flew today (RIG-1029-sq5_2)? If it does, does Tejen's timing fix
(`mpc_command_execution:=partial_step`) change them?

## 2. Hypothesis
- E0 (the twin as it is) swings ~10x less than the rig: the overnight square runs read 0.2-0.6 deg against the rig's 2.1.
- E1 (the delayed twin) brings the swing into the rig's range: a delay in QUAD4's loop that his MPC does not model
  takes damping out of his active swing control.
- Prior against it: a 50 ms delay is only 9-16 deg of phase at the rig's 0.5-0.9 Hz. The rig swings 2.2 deg rms in a
  26 s hover (k1_4), which looks like forcing (QUAD4's own wash on the cable and magnet, mocap noise) more than lost
  damping. The twin has neither.
- E2: the fix sends the 33 ms point of the first stage instead of its 100 ms endpoint. That removes a lead of ~67 ms.
  On the delayed twin this can go either way; it is an outcome, not a prediction.

## 3. The one variable
| Arm | Config | Change |
|---|---|---|
| E0 | `configs/experiments/tri_floorjoin_hover_L100_p115_sq52.yaml` | none (the baseline) |
| E1 | `..._sq52_cd050.yaml` | `SIM_CMD_DELAY_S` 0.050 from `SIM_CMD_DELAY_FROM_S` 25.0 s |
| E2 | `..._sq52_cd050_ps.yaml` | E1 + `mpc_command_execution:=partial_step` in partner_args |

**What E0 pins (sq5_2 as the sim path allows):**
- **World:** the triangle, carriers on plates 3/11/7 (`attach_azimuths_deg 90,330,210`). The 1.15 kg ring in the new
  world `three_rigid_ground_rig_attach_pickup_L100_p115.sdf` (`tools/make_attach_layout_worlds_p115.py --layout tri`:
  the square tool with the triangle's carriers; `gz sdf -k` valid; `--check` clean). QUAD4's mat is at (0, 1.4) on
  plate 9's line.
- **Carriers:** `throttle_max` 0.95, the slope filter (QUAD4 ema), and `cable_notch_hz` 5.3. The notch is the rig
  default (real.yaml); the twin's default is 0, so it is typed here for parity.
- **partner_args:** sq5_2's: `attach_cpp_transit:=false attach_transit_clearance:=0.30 minimum_search_z_m:=2.20
  approach_offset_z:=0.30 attach_twist:=zero match_xy:=0.08 match_dwell:=0.5 climb_speed:=0.3 commit_backdate:=0.04
  tracking_check:=off carry_clearance:=0.25`. Also `mission_mode:=floor_join contact_attach:=true
  confirm_pickup_release:=true`, `geometry:=rig_twin_L100_p115 plate:=9 carrier_azimuths_deg:=[90.0,330.0,210.0]`.
  - The search floor 2.20 is the rig's typed value. It is 0.18 m under the twin's climb end, and the launch's check passes.
  - Added: `match_speed:=0.15`. The rig path has it built in; the sim path's default is 0.08.
  - Left as the sim path has them: the descent abort is 0.08 m on both paths (max(match_xy, 0.05) in the twin). The
    abort speed is 0.15 m/s in the twin against 0.25 on the rig. The rig's bottom hold and measured-tip trim are
    rig-path only.
- **Timing:** WAIT_WELD 150 s, then LAND. The rig gave up about 40 s after his TAKEOFF.

**The delay (E1):** the 22 Oct decided delayed twin (decisions.md: "~50 ms extra command delay, calibrated on
R2397-R2404 against rig f3h / ls3c").
- It is the only rig-timing setting still decided. The 23 Oct per-channel twin (throttle +35 ms, rate_kff, body-rate
  EMA) did not reproduce the rig pair, and its extra parts were deleted.
- It goes on every simulated Betaflight bridge. QUAD4's commands go through `bf_comm_3`
  (`graph_attach.py`, the same `payload_betaflight_comm` node), so it reaches him too.
- From 25.0 s of sim time: after the ring's lift (LIFTED at ~20.3 s, the top ~25 s) and before his TAKEOFF (~26.2 s in
  every earlier attach run). The drones flipped in the takeoff creep with delays from the start (R1100).
- No mocap delay:
  - Motive's latency is not measured (`docs/rig/betaflight_2026-10-22.md`).
  - The 50 ms was calibrated as the whole loop's delay.
  - The twin's magnet-tip publisher has no delay knob, so a delayed body pose against an undelayed tip would create a
    false swing angle.

## 4. Baseline
**Rig reference** (computed by this card's `swing.py` from the bags; it reproduces the 13:30 cable write-up window
for window, and its 6.15 cm is the "6.2 cm"):

| Run | Window | Swing rms per window (total) | Magnet to plate 9 |
|---|---|---|---|
| RIG-1029-sq5_2 | MATCH and descent windows >= 1 s (7) | 3.48, 1.69, 2.11, 2.83, 2.13, 3.75, 1.23 deg; median **2.13** | median **6.15 cm** over every MATCH and descent sample |
| RIG-1029-sq5_2 | approaches >= 1 s (2) | 2.63, 5.63 deg | |
| RIG-1029-k1_4 | 26.3 s over the object | 2.20 deg (1.75 / 1.33 per axis) | 4.1 cm to the object (rig review) |

- Dominant frequency in the MATCH and descent windows: 0.36-0.88 Hz.
- Mean lean: median 0.76 deg.
- Phases: MATCH 1.7 s first; 6 MATCH, 6 descents, 6 aborts; no contact.

**Twin before this card** (square, no delays; `swing.py` on the overnight runs):

| Runs | Swing S (median of MATCH/descent windows) | E (magnet to plate 9) | Phases |
|---|---|---|---|
| R2843, R2849, R2850, R2858, R2862 | 0.50, 0.19, 0.17, 0.62, 0.17 deg | 0.65, 0.54, 0.47, 1.45, 0.36 cm | 1 MATCH, 1 descent, welded first try |

## 5. Pass / fail numbers (FROZEN 29 Oct 16:00 AEDT, before any run)
**Metrics** (`swing.py`, scratchpad copy `twin_timing/swing.py`, to be saved with the results):
- **S:** the median, over the run's MATCH_VELOCITY and DESCEND_TO_ATTACHMENT windows of 1 s or more, of the total
  swing rms sqrt(std_phi^2 + std_theta^2).
  - phi = atan2(r_x, -r_z) and theta = atan2(r_y, -r_z), with r = magnet - (body - 0.03 z).
  - The twin reads run.csv (d3_*, tip_*) at 30 Hz, sampled and held.
- **E:** the median magnet-to-plate-9 xy distance over every MATCH and descent sample.
  - Plate 9 is the ring pose plus R(q)(0, 0.204) in the twin, (-0.005, 0.197) on the rig.
  - Cross-check, not the metric: the twin's logged `tip_target`.
- **Validity:**
  - E1/E2: the launch log shows `[BF3] command delay thr 0.050 s att 0.050 s from t 25.0 s`, and `[BF0]`-`[BF2]` alike.
  - E0: no such line.
  - Every run: QUAD4 reaches MATCH, with at least one MATCH or descent window of 1 s. A run without it is void for S
    and E, and is reported.

**E1, "the delayed twin reproduces the rig's swing":**
1. S within 1.2-3.8 deg in both repeats (the rig's per-window range, 1.23-3.75).
2. E within 4.0-9.0 cm in both repeats (the rig's per-window medians over windows of 1 s or more: 4.0-9.0).
- **Confirms** if 1 and 2 hold in both repeats.
- **Falsifies** if S < 1.0 deg in both repeats: the swing stays under half the rig's median, so the delay alone does not
  reproduce it.
- Otherwise reported, no claim (partial).
- Reported against the rig, no bar: the approach swing (rig 2.6-5.6), mean lean, dominant frequency, MATCH times,
  descents and aborts, contact and weld.

**E0:** the baseline, no bar. Its S and E are recorded against the rig and the overnight square runs.

**E2** (flown only if E1 confirms):
- **Validity** (Codex's runtime checks, DECISIONS_1355.md section 1). Otherwise E2 is void:
  - his MPC's `metadata.json` says `mpc_profile` oct10 and `command_execution` `first_stage_partial_step`, with
    `mpc_command_stage_duration_s` 0.1;
  - the launch log shows the `MPC command execution partial_step` warning;
  - in his log, the sent command never steps further per tick than its rate bound (throttle 0.5/s, attitude 1/s,
    times the logged interval, plus the radio's rounding);
  - the first command after TAKEOFF starts from the idle command.
- **Confirms** "the fix lowers the swing in the rig-timing twin": both E2 runs have S <= 0.7x the lower E1 run's, with
  no new abort or fault.
- **Falsifies:** the mean E2 S >= the mean E1 S.
- Otherwise reported, no claim. E, the MATCH time, the descents and aborts, and the weld are reported against E1.

**If E1 misses** (partial or falsified): at most ONE round of calibration variants, then stop and report. Each variant
is named here with its own bars, written and frozen before its runs.

**E1c, the calibration round: FROZEN 29 Oct 16:30 AEDT, after E1 missed (R2867/R2869), before its runs.** One
variant: `configs/experiments/tri_floorjoin_hover_L100_p115_sq52_cd060.yaml`, with `SIM_CMD_DELAY_S` 0.060 instead of
0.050 and everything else as E1.
- **Why 60 ms:** the top of the 22 Oct calibration bracket. cd045 sat near the rig on f3h / ls3c and cd060 above it
  (R2397-R2404).
- **Why not more:** at 80 ms the 1 m pair already see-sawed 19-41 deg p-p (R2371/R2372). Any larger delay would swing
  the ring and force QUAD4 through it, which would confound the answer.
- **Question:** within the rig-calibrated range, can the command delay alone bring QUAD4's swing to the rig's?
- **Validity:** as E1, with the `[BF0]`-`[BF3]` lines reading `thr 0.060 s att 0.060 s from t 25.0 s`.
- **Bars:** the same as E1, with 2 repeats.
  - Confirms if S is within 1.2-3.8 deg and E within 4.0-9.0 cm in both runs.
  - Falsifies if S < 1.0 deg in both. Then the delay within its calibrated range does not reproduce the swing, and the
    card stops.
  - Otherwise partial, no claim.
- **Reported, no bar:** S per run against E1's 0.69 / 0.68, to say whether the swing scales with the delay.

**E2c, Tejen's fix on the 60 ms twin: FROZEN 29 Oct 16:46 AEDT, before its runs.** Wesley (16:45) chose to fly E2 on
the 60 ms twin although E1 missed and E1c is partial; this waives the "only if E1 confirms" gate above.
- **Config:** `configs/experiments/tri_floorjoin_hover_L100_p115_sq52_cd060_ps.yaml`: E1c plus
  `mpc_command_execution:=partial_step` in partner_args, nothing else.
- **Baseline:** E1c, R2870 / R2871 (S 1.60 / 1.80 deg, E 3.75 / 3.67 cm), same tree.
- **Validity:** E2's list above, with the `[BF0]`-`[BF3]` lines reading `thr 0.060 s att 0.060 s from t 25.0 s`, and
  at least one MATCH or descent window of 1 s. Reported with it (no bar): the first command step after his TAKEOFF
  (endpoint mode in E0: 0.050 above hover; partial_step should give 0.017 or less).
- **Confirms** "the fix lowers the swing in the 60 ms twin": both runs have S <= 1.12 deg (0.7 x the lower E1c run's
  1.60), with no new abort or fault in the attach window.
- **Falsifies:** the mean E2c S >= 1.70 deg (the mean E1c S).
- Otherwise reported, no claim.
- **Reported against E1c, no bar:** E, the MATCH time, descents and aborts, contact and weld. The carriers' instability
  after a take-back at 60 ms (E1c's DROP AND LAND) is outside the scored window and is reported, not scored.

## 6. Repeats
2 per arm.

## 7. Cost
- Lab PC (`drones`, Wesley 22 Oct: whenever it is free; at most 4 slots).
- Wave 1: E0 x2 and E1 x2 (4 slots).
- Wave 2: E2 x2, or the calibration round x2. Optional: an E1 pickup pair (k1_4's settings as the sim path allows).
- About 6-8 runs, ~1.5 h of lab time. No laptop Gazebo.

## 8. New code?
No flight-loop code from this card:
- The delay knobs exist (`payload_betaflight_comm.py`, default off).
- `mpc_command_execution` was written at 13:35 and code-read against Codex's checklist at 14:05
  (CHANGES_FROM_TEJEN.md).
- New: the triangle layout in `tools/make_attach_layout_worlds_p115.py` (a world generator; the square output is
  unchanged, `--check` clean) and the two triangle `_p115` worlds (approved with this plan).
- Critic: not run from this card. The fix is Tejen's code, reviewed by his Codex. A critic on partial_step before the
  rig rehearsal is the orchestrator's call.

## Results
**Wave 1 (lab PC, 16:03-16:26, R2866-R2869; scored with `swing.py`).**
- **Validity:** every E1 launch log carries the `[BF0]`-`[BF3]` `command delay thr 0.050 s att 0.050 s from t 25.0 s`
  lines. QUAD4's bridge `bf_comm_3` is delayed too.
- **Alignment:** his MPC log was pulled from the lab slot and aligned to sim time (height residual 0.6-1.0 mm). Its
  phi/theta rms agree with the geometric ones to <= 0.05 deg.

| Arm | Runs | S (deg) | E (cm) | Approach swing (deg) | MATCH | Descents / aborts | Contact / weld |
|---|---|---|---|---|---|---|---|
| Rig sq5_2 | RIG-1029-sq5_2 | 2.13 (windows 1.23-3.75) | 6.15 | 2.63 / 5.63 | 1.67 s first, 6 in all | 6 / 6 | none |
| E0, no delay | R2866 / R2868 | 0.51 / 0.51 | 0.90 / 0.93 | 0.96 / 1.00 | 0.57 / 0.57 s | 1 / 0 each | welded 46.4 / 46.3 s |
| E1, 50 ms | R2867 / R2869 | 0.69 / 0.68 | 1.86 / 2.07 | 1.00 / 1.03 | 0.53 / 0.56 s | 1 / 0 each | no contact; give-up 71.2 / 71.3 s, taken back |

- **The windows:** with MATCH under 1 s (the 0.5 s dwell), S rests on each run's one descent window (3.77 s).
- **Frequency** of that window: E0 0.26 Hz in both, E1 0.79 Hz in both (rig 0.36-0.88).
- **Mean lean:** E0 0.20 / 0.21 deg, E1 0.17 / 0.13 deg (rig 0.76).

**E1 FALSIFIES "the delayed twin reproduces the rig's swing"** (frozen bars): S 0.69 / 0.68 < 1.0 deg in both runs.
- E (1.86 / 2.07 cm) is also under the 4.0 cm floor.
- The delay raised S by ~35 % and E ~2x over E0, but the swing stays ~3x under the rig's.
- **The attach failed on the twin's contact gate:** in both E1 runs QUAD4 reached the bottom (ATTACH_READY) and held
  26 s without a contact; his give-up and our take-back followed.
  - The tip was centred on average but oscillated radially: radius 5-95 % 0.185-0.222 m about the plate's 0.204, a
    median 1.9 / 2.2 cm off, moving at 0.04 m/s (median).
  - The twin's comotion contact gate is xy <= 0.010 m and <= 0.030 m/s for 0.30 s. It held for 8 % / 3 % of the hold,
    never 0.3 s unbroken. E0's tip sat 0.5 cm off at 0.017 m/s, 77 % inside.
  - The tip also sat 3.3 cm (median; 5-95 % 2.8-4.0) under the contact target, outside the gate's normal band
    (-1.5 cm); E0 sat 1.3 cm under. The cause is not read here. One candidate: the twin's rim ends 9 mm inside the plate
    centre (r 0.195).
  - The rig's magnet catches anywhere on the ~100 x 85 mm slot, so this failure is the twin's gate and rim, not a rig
    prediction.
- **The runner FAILs are after the scored window:** a carrier tipped on the floor after the ring's touchdown (envelope
  fault, tilt 74-80 deg, ~11 s after LAND). That is the delayed twin's known landing fault (R2368, R2372, R2449,
  R2453).
- Per the card, E2 is not flown. One calibration round follows (E1c above).

**Wave 2, E1c at 60 ms (lab PC, 16:30-16:41, R2870/R2871).**
- Valid: every `[BF0]`-`[BF3]` line reads `thr 0.060 s att 0.060 s from t 25.0 s`.

| Arm | Runs | S (deg) | E (cm) | Approach swing (deg) | MATCH (s) | Descents / aborts | Contact / weld |
|---|---|---|---|---|---|---|---|
| Rig sq5_2 | RIG-1029-sq5_2 | 2.13 | 6.15 | 2.63 / 5.63 | 1.67, 1.13, 2.57, 4.67, 0.56, 0.80 | 6 / 6 | none |
| E1, 50 ms | R2867 / R2869 | 0.69 / 0.68 | 1.86 / 2.07 | 1.00 / 1.03 | 0.53 / 0.56 | 1 / 0 each | none (bottom hold, give-up) |
| E1c, 60 ms | R2870 / R2871 | **1.60 / 1.80** | 3.75 / 3.67 | 1.66 / 1.87 | 0.90, 0.57, 0.63, 4.74 / 1.00, 0.54, 1.03, 0.70, 2.57 | 3 / 2 and 4 / 3 | none: one 15-16 s bottom hold, then the aborts, give-up at 71.7 / 71.9 s |

- **The per-window swing:**
  - R2870: 1.21-2.13 deg at 0.53-0.84 Hz.
  - R2871: 1.02-2.38 deg at 0.39-1.0 Hz.
  - Rig: 1.23-3.75 deg at 0.36-0.88 Hz.
- **Mean lean:** 0.16 / 0.25 deg (rig 0.76).
- **The ring in the attach window** (approach to give-up or weld):

| | Tilt sd about its mean (deg) | xy rms (cm) |
|---|---|---|
| E0 | 0.05 | 0.1 |
| E1 | 0.27 / 0.27 | 1.0 / 0.8 |
| E1c | 0.52 / 0.40 | 1.1 / 0.9 |
| Rig sq5_2 (bag) | 2.22 | 4.6 |
| Rig k1_4 hover | 1.60 | 3.3 |

  The rig's ring moved 4-5x more than the twin's in every arm.

**E1c is PARTIAL (frozen bars; no claim):**
- S is inside the rig's band in both runs (1.60 / 1.80, bar 1.2-3.8).
- E is under its 4.0 cm floor in both (3.75 / 3.67).
- What changed between 50 and 60 ms:
  - The swing more than doubled.
  - The rig's failure mode appeared: MATCH-descent cycles, descents aborted on his 8 cm check (2 of 2 and 3 of 3 after
    the first), give-up.
  - So the twin crosses a threshold between 50 and 60 ms of command delay, as the 0.55 m pair did on the throttle
    between 35 and 50 ms (2026-10-23_pair_mode_timing round B, `_rt050`).
- **Caveat: 60 ms is past the carriers' margin in this twin.** After the take-back, in both runs:
  - the delayed carriers' throttle swung tick to tick (0.5-0.85);
  - the ring sank from 1.0 m and tilted past 30 deg about 6 s later;
  - the fleet did DROP AND LAND;
  - R2871 then took a carrier envelope fault in the drop landing (the runner FAIL).
  - The scored attach window ends before this, but during it the ring already moved twice as much as at 50 ms.
- **Card stops here** (one calibration round). E2 is not flown: E1 did not meet its bars.
- **Send path, before the fix (endpoint mode):** E0's log shows the first command after his TAKEOFF already 0.050
  above hover throttle. That is the full 100 ms stage. The same check under partial_step should read <= 0.017.


**E2c, Tejen's fix on the 60 ms twin (lab PC, 16:47-16:58, R2872/R2873; scored against the frozen E2c block).**

| Arm | Runs | Validity: send path | Validity: MATCH/descent window >= 1 s | S (deg) | E (cm) | QUAD4 |
|---|---|---|---|---|---|---|
| E1c, 60 ms, endpoint send | R2870 / R2871 | (endpoint) first command +0.048 roll / +0.050 throttle; per-tick steps up to 0.10 / 0.05 | yes | 1.60 / 1.80 | 3.75 / 3.67 | climb and approach normal; aborts, give-up |
| E2c, 60 ms, partial_step | R2872 / R2873 | pass / pass | **no / no** | void | void | **lost control in its climb, upside down on the floor at ~33-35 s** |

**Send-path checks, E2's list (all pass in both runs):**
- `metadata.json`: `mpc_profile` oct10, `command_execution` `first_stage_partial_step`, `mpc_command_stage_duration_s`
  0.1.
- The launch log shows `MPC command execution partial_step (mpc_command_execution), the oct10 weights and bounds
  unchanged`.
- `[BF0]`-`[BF3]` read `thr 0.060 s att 0.060 s from t 25.0 s`.
- The first command after his TAKEOFF is one interval from his start state [0, 0, hover, 0]: +0.016 roll and +0.0166
  throttle. The endpoint runs read +0.048 / +0.050; the reported check wanted <= 0.017.
- Per-tick steps: at most 0.038 / 0.041 roll and 0.018 / 0.0205 throttle. That is the oct10 bound times an elapsed
  sim interval of up to ~41 ms (5-95 % 21.5-41.2 ms). The endpoint runs step 0.10 / 0.05, the whole 100 ms stage, every
  tick.
- Read with care: 2-7 % of ticks exceed the bound times my estimated sim interval. That estimate comes from matching
  his wall-clock log to run.csv; his log's own interval field is wall time (~196 ms).

**E2c is VOID for its swing bar** (the frozen validity: no MATCH or descent window; QUAD4 never reached
APPROACH_ABOVE_TARGET). It is a **new fault** against E1c, whose climb was normal in 2 of 2.
- **The climb:** from his TAKEOFF (26.3 s) the body bounced vertically as the rod lifted off the floor:
  - R2872: 0.10 -> 0.59 -> 0.19 -> 0.82 m;
  - R2873: 0.10 -> 0.66 -> 0.43 -> 1.0 m;
  - his throttle swung 0.29-0.57 at ~1.2 Hz. E1c rose smoothly: 0.10 -> 0.42 -> 0.53 m, then a steady climb.
- **Then a lateral divergence:** tilt 21 / 39 deg, and in R2872 our tracker's envelope speed 3.17 m/s. QUAD4 ended
  upside down on the floor (tilt 128 / 119 deg) at ~35 / ~33 s.
- **A reading, not tested:** the endpoint send leads the plant by ~67 ms, and on this twin that lead offsets most of
  the 60 ms delay. partial_step removes the lead, so his MPC's unmodelled loop delay goes from ~0 to ~60 ms.
- **QUAD4's crash did not land the fleet.** Our tracker's envelope fault on drone 4 was not escalated ("tracker
  disarmed while its mux forwards the partner - not escalating"). QUAD4 stayed armed, upside down, at idle 0.05 for
  ~50 s. `newcomer_fault_land` did not act.
- **Correction to E1c's caveat (no earlier text edited; registry R2870-note):** the ring's sinking at 60 ms does not
  come from the take-back.
  - In all four 60 ms runs the carriers' tick-to-tick throttle change grows from ~60 s: rms 0.002, then 0.008-0.020
    by 70-80 s. This includes R2872/R2873, where QUAD4 lay crashed and nothing was taken back.
  - The ring goes below 0.9 m at 74.7 / 77.1 / 78.4 / 80.8 s, about 50 s after the delay starts. At 50 ms it never
    did before LAND (R2867).
  - E1c's first-attempt windows (40-46 s) predate the growth and already read S 1.21-2.38 deg.
- **Runner FAILs:** after the drop, a carrier's envelope fault in the drop landing (FLEET_ABORT at 86.3 / 90.2 s).

**Neighbours of E2c (exploratory, one run each, no bars; lab PC 17:00-17:16; R2874/R2875).** I flew these
unprompted to find out whether the climb failure belongs to the fix or to the delay (working rules: fly the
neighbours).

| Send path | 0 ms | 50 ms | 60 ms |
|---|---|---|---|
| endpoint (his current) | climb normal, welded (R2866 / R2868) | climb normal (R2867 / R2869) | climb normal (R2870 / R2871) |
| partial_step (the fix) | climb normal, welded 46.7 s, S 0.43, E 0.76 cm (R2874) | **lost control in the climb** (R2875) | **lost control in the climb** (R2872 / R2873) |

- **R2875's climb:** the body bounced 0.10 -> 0.53 -> 0.26 -> 0.68 -> 0.51 -> 0.74 m (throttle 0.36-0.60). Then it
  diverged laterally (tilt 59 deg at 34.2 s) and lay upside down on the floor from 34.7 s.
- **Send path in both neighbours:** as E2c. The first command is +0.016 / +0.0166; per-tick steps are at most 0.041 /
  0.018.
- **In this twin the fix is safe without the added delay and unsafe with 50 or 60 ms of it.** The endpoint send flew
  the same delays without trouble.
- **R2875 also:** the 50 ms carriers sank later (ring below 0.9 m at 147.3 s). It ended in DROP AND LAND and a carrier
  envelope fault in the drop landing. R2867/R2869 had held the ring until LAND at 192 s.
- **Not shown here:**
  - which channel carries the climb failure. The twin's attitude path is already ~30 ms slower than the rig's, so a
    uniform delay overstates the rig's attitude lag; its throttle path at +50 ms is close to the rig's 44-67 ms;
  - whether the rig has the margin the twin lacks.

**Channel split (exploratory; Wesley 17:20; one run each; lab PC 17:22-17:46; R2876/R2877).** Two copies of
`_cd060_ps`, each with the 60 ms delay on one channel group only. The launch logs read `thr 0.060 s att 0.000 s` and
`thr 0.000 s att 0.060 s` from t 25.0 s on `[BF0]`-`[BF3]`. Falsifier (frozen in the request): the throttle delay
alone flips QUAD4 in its climb (tilt > 60 deg before APPROACH_ABOVE_TARGET) and the rate delay alone does not.

| Run | Delay (partial_step in both) | Climb: height | Throttle p-p (first 3.5 s) | Max tilt before APPROACH | Reached APPROACH | Then | Send path: first step / per-tick max |
|---|---|---|---|---|---|---|---|
| R2872 / R2873 (E2c) | 60 ms, all channels | bounces 0.1 -> 0.6 -> 0.2 -> 0.8-0.9 m | 0.30 / 0.31 | 178 / 162 deg (flipped) | no | | +0.016 / +0.0166; 0.038-0.041 / 0.018-0.0205 |
| R2876 | 60 ms, throttle only | bounces 0.10 -> 0.60 -> 0.16 -> 0.99 -> 0.08 m | 0.54 | 142 deg at 29.8 s (flipped) | no | upside down on the floor | +0.016 / +0.0166; 0.037 / 0.0185 |
| R2877 | 60 ms, rates only | clean: 0.10 -> 0.46 -> 0.41 m, then steady | 0.14 | 6.5 deg | yes, 34.3 s | the swing mode diverged at ~0.7 Hz; flipped at 45.8 s | +0.016 / +0.0166; 0.045 / 0.018 |
| R2870 (E1c, his current send) | 60 ms, all channels | clean (0.42 -> 0.37 m dip) | 0.20 | 5.9 deg | yes | | (endpoint) +0.048 / +0.050; 0.10 / 0.05 |

- **The falsifier holds as written:** the throttle delay alone flips QUAD4 in its climb, and the rate delay alone does
  not.
- **But the rate delay is not harmless:** R2877 flipped later, in the approach.
  - Tilt per 1 s block: 6.5, 7.1, 5.1, 6.8, 7.6, 16.1, 41.3 deg.
  - The body swung to ±1.2 m/s and the magnet to ±1.5 m in y. The body fell from 2.1 to 0.1 m by 41.6 s, flew off,
    and lay upside down at 45.8 s.
- **So under partial_step, 60 ms on either channel group destabilises QUAD4, in different phases:**
  - the throttle path gives the vertical bounce as the cable lifts off;
  - the rates give a growing swing in the approach.
  - With his current send and the same 60 ms on all channels, both phases were clean (R2870/R2871).
- **Correction to the E2c part above** (that text stands): its "~1.2 Hz" throttle swing should read ~0.6-0.7 Hz. The
  height peaks are 1.4-1.6 s apart, and Welch on his throttle gives 0.7-0.93 Hz (`climb.py`).
- **Both splits held the ring until LAND** (R2876 z >= 0.998 m; R2877 tilt <= 5.1 deg as QUAD4 passed close). The
  sinking seen in the four all-channel 60 ms runs did not appear with one channel group delayed.
- **The usual faults:** QUAD4's envelope fault was again not escalated in both runs. R2877's runner FAIL is a carrier's
  envelope fault in the landing after LAND.
- No further variants flown.
