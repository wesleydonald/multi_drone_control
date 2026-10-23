# Overnight 3-4 Oct 2026: internal and external model errors (laptop, Wesley's /loop)

Task 1: remove the ring's height and xy errors (internal: twin +5 cm hold with z_ki 0, thrust-map
error, one drone's map offset; external: wind, push) with a literature-backed control-law change.
Order: offline replay -> SIL -> headless Gazebo. One card per method, critic before flying.
Rules: new knobs default off; no launch defaults / params.py / gate / existing SDF changes; losing
methods stay behind off knobs; no commits; one registry row per run; questions to the list below.
Task 2 after: W4 rehearsals (even ring 0/3/6/9), W5 rig sheet + prebuild n=2/3/4 + desk test,
guide refs + PDF.

## State (update every iteration)

- R0868 (option F, rod model): hold +5.0 cm vs R0841 +4.8; elevation 42.8 deg vs 49.7 (planned 45).
  Drones on their references (0.0 cm), so the offset is in the planned references.
- tools/l1_replay.py: predictor used A x_hat (should be A x) and aT = 1 at the 10 Hz tick
  (estimate biased by e^-aT); fix before judging C.

## Log

### 1. Where the +5 cm comes from (no flight)
- `tools/hold_replay.py R0868` (new): at the hold tick t 57.0 (ring 1.045 m) the planner plans the
  ring to 1.001 m within 1.5 s (45.0 deg, 4.88 N). Closed on its own model it settles at 1.0000 m.
  The planner is unbiased.
- SIL R0861 / R0473 (matched plant: 0.64 kg both sides, map 83.1 both sides, massless rods, no
  pivot): 0.665 m vs 0.600 target, rods 42.5 deg, planned tension 3.97 N vs plant 4.16 N.
- So the offset is the cascade not executing the plan's first move. It is not a mass, map or rod
  error, and a force balance (option A) cannot see it. Online map/mass adaptation would not fix
  it either (the SIL map and masses are exact).
- R0868 (option F) fixed the elevation booking (49.7 -> 42.8 deg) but not the height (+5.0 cm).
### 2. Method 1: offset-free MPC (card docs/experiments/2026-10-04_offset_free_innovation.md)
- `mpc_planner/offset_free.py` + 4 tests; planner knob `offset_free` off|shadow|on, `of_tau`.
- Gate: lift complete, full pull (ff 1), not descending/landing, fresh pose, rods taut, a solved
  previous horizon.
- Option A's d_L and the observer's are summed. Quick gate PASSED (426 tests).
- Critic launched. SIL `carry_hover_n3_long_of` running.
- R0869 SIL long hover with the observer on: 0.663 -> 0.592 m (target 0.600). Converged
  (innovation 0) with d_z +0.59 N; no oscillation. The -0.8 cm residual: the plan's first move
  for a small offset is in the cable rates, not the ring velocity, so a velocity innovation reads
  zero there.
- L1 replay fixed (predictor on the measured state). At aT 0.1 it passes the critic's must-fix 1
  falsifiers: R0856 step -0.90 N, R0862 -0.96 N, R0832 lap p-p 0.198 N, R0841 z steady. At aT 1.0
  it fails (the estimate reads e^-aT of a step). Registry M-l1-replay.
- Critic on the observer card: NOT READY (9 must-fixes). Main points:
  - It is integral action through the planner's feedforward.
  - It repeats the 23 Sep load-force observer (R0347/R0348 -2 cm, drifting).
  - No arm told it from z_ki 0.4.
- Fixed:
  - Gate as ZBias (|vz| < 0.05, miss < z_i_gate), first tick after a freeze skipped, held across a
    resize.
  - Refused with z_ki > 0, with kt_trim on, and with A unless of_tau >= 3x A.
  - Card rewritten (history, integral framing, discriminating arms, one baseline per arm).
  - decisions.md line for the overnight instruction (sim only).
- R0870 map-offset baseline (drone 2 plant offset 0.215 vs 0.185): z -3.0 cm, xy 3.8 cm, tilt 6.7 deg.
- Queue (sequential, SIL and Gazebo share topics and clean_slate):
  - z_ki 0.4 hold.
  - SIL observer arms: hover, FF 0.9, kT +25 %, circle, n=2, with re-flown baselines.
  - Gazebo observer arms: hold, push, map, orbit, wind.
  - z_ki / kt_trim / A comparators.
  - Observer + A (of_tau 4).

## Questions for Wesley (none of these were acted on)

1. The offset-free observer is integral action through the planner model (critic). If it wins in
   sim, may it go to the rig, against decisions.md 2026-09-30 (the integral is a safety net) and
   2026-10-03 (observer only, no new integral)?
2. Option F (rod mass in the planner model, R0868): it fixes the elevation booking (49.7 -> 42.8 deg,
   planned 45) but not the height. Keep it (behind `rod_mass`) or delete it?
3. INDI: the measured-force throttle was removed on 23 Sep. Tonight's test uses the MPC tracker's
   other path (`cable_source: measured`) with a cap knob (default unchanged). Keep or delete after
   the result?
4. Which ring-error approach the thesis adopts (sim evidence tonight; claims need 2 Gazebo repeats
   and the reviewer):
   - (a) A (external force balance) + z_ki 0.4 (the existing rig safety net): best on push, wind and
     height. Does not fix a per-drone map error (A books it as a force and tilts the ring 9 deg).
   - (b) The offset-free observer (one estimator): helps everywhere but slower, with residuals in
     Gazebo (hold +1.6, wind 7.8).
   - (c) INDI + z_ki: best on the per-drone map error, but it tipped a drone in LAND in 4 of 6 runs.
5. Keep or delete tonight's off-knob code:
   - `offset_free` / `of_tau` / `of_window`
   - `pin_cable_rates` (falsified offline)
   - `w_thr_ref` (falsified offline)
   - the INDI fixes and `cable_accel_cap`
   - `rod_mass` / `rod_com` (F)

   Plus the new tools (hold_replay, method_metrics, method_table, l1_replay).
6. Wednesday: fly A in `shadow` (log only) alongside z_ki 0.4 to get the rig's force estimate?
7. The guide's path:line references (21 of 83 moved by tonight's code) and the PDF: update after
   question 5.
- SIL arms, one-step observer:

  | arm | without | with |
  |---|---|---|
  | hover | +6.3 | -0.7 (R0872) |
  | FF 0.9 | -4.0 (R0874) | -0.9 (R0873) |
  | n=2 | +4.2 (R0879) | -2.9 (R0878) |
  | circle | +5.6 (R0877) | +4.3 (R0876) |
  | kT +25 % | — | -12.5, no warning (R0875) |

- Gazebo hold, one-step observer (R0880): +7.8 cm, worse than +4.8. It converged with zero
  innovation on d_z -0.19 N. The one-step velocity innovation does not pin the fixed point to the
  target: the plan's first move is in the cable states.
- New: `of_window` (default 0 = one-step as flown). With window W the innovation is the position
  the horizon solved W s earlier predicted for now, less the estimate change since; converged, the
  plan predicts no motion over W. Test added (6 pass).
- INDI card critic: NOT READY (unsynchronised throttle, floor path open, hold has nothing to fix).
  Fixed in the tracker's measured path:
  - throttle low-passed with the IMU
  - planned term until airborne and while the ring is grounded (slew reset)
  - z clamped <= 0
  - `cable_accel_cap` knob (default 6.0 unchanged), 1 Hz diag, no-IMU warning

  Arms added: map+, map- (0.155), orbit, hold, hold + z_ki 0.4; SIL hover first.
- Rod-rate pin (card 2026-10-04_pin_cable_rates):
  - Built from the R0887 replay: free node-0 rod rates imply 0.19 m/s drone velocities on drones at
    rest.
  - Critic NOT READY. Its offline test falsified it (M-pin-replay): the jump reverses sign between
    R0841 (50 deg) and R0868 (42.8 deg) at the same +5 cm. Not flown; knob off.
- Window observer on the Gazebo hold:

  | window | hold | run |
  |---|---|---|
  | 1.0 s | +2.35 cm | R0890 |
  | 1.5 s | +2.62 cm | R0892 |

  Both were still integrating at LAND (the residual per cm falls as 1/T^2). Next: window 1.0,
  tau 0.5.
- The pgrep self-match trap hit the queue monitor; now a pidfile.
- Fast-window observer (window 1.0 s, tau 0.5):
  - SIL hover R0893 -0.01 cm, monotone, no oscillation, but ~24 s into 1 cm (z_ki 0.4: 6.2 s).
  - SIL n=2 R0894 -0.5 cm.
  - Gazebo hold next.
- Task 2 started:
  - W5 rig sheet `docs/rig_2026-10-07_commands.md` drafted: four drones, block A on the even ring
    (F1, F2, F5, F6, F7), block B on 1/3/5/9 (B1, F8 detach); F3/F4 dropped (W3 failed, knobs
    deleted); land_unwind off.
  - Prebuild n=2/3/4 at rig masses done (n=4 81 s).
  - W4 rehearsal configs queued: `b2_cap075_fix` x2, `w4_even_orbit`, `w4_even_fig8`,
    `w4_3915_hover`, `w4_3915_detach`.
  - Desk test with fake_mocap waits until the queue is done (same topics).
  - Guide refs: at the end of the night.
- Ancillary tracker (card 2026-10-04_ancillary_tracker): critic NOT READY. Its offline must-fix
  falsified it (M-ancillary-gap): the planned node-0 thrust gap flips sign between R0841 (+0.63) and
  R0868 (-0.59) at the same +5 cm. Not flown; `w_thr_ref` default.
- So three causal mechanisms for the internal +5 cm have now been falsified tonight:
  - rod booking (F)
  - the node-0 rate jump
  - the unflown thrust cut

  The offset is the force-balance point of the re-pinned cascade (critic). Only integral action
  closes it: z_ki 0.4, or the observer.
- Observer push R0897: 2.6 cm (R0856 18.0), overshoot 12 cm after release.
- INDI map R0896: tilt 0.8 deg (R0870 6.7), z +3.8, abort in LAND. The measured term is biased by
  the airframe/rod mass split.
- Gazebo results (one run each unless noted; registry rows R0897-R0923).
  - Observer, window 1.0 s, tau 0.5:

    | arm | result | baseline |
    |---|---|---|
    | push | 2.6 cm, 12 cm release overshoot (R0897) | |
    | wind | 7.8 cm (R0898) | A 3.9 |
    | orbit | 2.6 / +2.5 (R0902) | |
    | map+ | +1.0 / 0.2 / 8.0 deg (R0903-R0905, three agree) | |
    | map- | +2.4 / 0.09 / 7.5 (R0907) | R0919 +12.8 / 3.5 / 6.1 |
    | long hold | +1.6 (R0906, converged short) | |

  - z_ki 0.4:

    | arm | result |
    |---|---|
    | hold | +0.1 |
    | push xy | 17.9 |
    | map+ | 0.0 / 3.8 / 6.7 |
    | orbit | 2.8 / +1.1 |
    | wind | 18.0 |
    | map- | 0.0 / 3.5 / 6.0 |

  - A:

    | arm | result |
    |---|---|
    | push | 0.3 (release overshoot 15) |
    | map+ | +3.9 / 1.3 / 9.0 |
    | orbit | 2.6 / +3.7 |
    | hold | +3.8 |

  - kt_trim:

    | arm | result |
    |---|---|
    | map+ | +4.1 / 1.3 / 2.3 |
    | hold | +3.7 |

  - INDI:

    | arm | result |
    |---|---|
    | map- | +4.5 / 0.58 / 0.32 |
    | hold | +4.2 |
    | orbit | 3.8 / +4.3 |

    LAND aborts in 2 of 3 runs.
- A + z_ki 0.4:

  | arm | result | run |
  |---|---|---|
  | push | 0.4 cm, z +0.1 | R0928 |
  | wind | 3.9 / 6.8 cm, z +0.9 | R0929 |
  | map+ | 0.0 / 1.3 / 9.0 deg | R0930 |

  The best height and external-force combination. It does not fix a per-drone map error: A books
  it as a force and tilts the ring.
- INDI + z_ki 0.4:

  | arm | result | run |
  |---|---|---|
  | hold | 0.0, tilt 0.18 | R0924, LAND abort |
  | map+ | 0.0 / 3.1 / 0.8 | R0926, landed |
  | map- | 0.0 / 0.57 / 0.31 | R0927, LAND abort |

  Best on the per-drone map error, but 4 of 6 INDI runs tipped a drone in LAND.
- Guide refs: 21 of 83 moved, all from tonight's off-knob code. To be updated once Wesley decides
  what stays (the deletions move them again).
- W4 rehearsals now flying (four drones).
- W4 rehearsals (four-drone rig twin at 1.0 m, B2):

  | rehearsal | result | run | bar |
  |---|---|---|---|
  | even hover pair, z_ki 0 | +3.2 cm, tilt 1.21 deg (reproduces R0834/R0836 on the renamed code) | R0931/R0932 | |
  | even orbit | 2.9 / 5.0 cm xy, tilt 1.20 | R0933 | PASS F6 |
  | even figure-8 a 1.0 at 0.25 | the whole eight, xy 2.2 / 5.4 cm, tilt 1.13 | R0935 | PASS F7 |
  | 1/3/5/9 hover | +2.5 cm, tilt 1.38 | R0936 | PASS F1 |
  | 1/3/5/9 detach 4 -> 3 | peak 3.2 deg, < 2 deg after 2.6 s, dip 2.7 cm | R0939 | PASS F8 |

  Voids: R0934, R0937, R0938.
- W5 desk test (fake_mocap, real_control mode mpc, four drones, even ring), all PASS:
  - four trackers ready on the rig map (kT 35.2, cap 0.8, offset 0.185 + V term)
  - planner geometry n=4 / 0.55 / 0.225 / attach_z 0 / 0.86 / pivot -0.04
  - nothing compiled
  - preflight GO (results/preflight/20261004_040059.md)

  The props-off dry launch with the radios stays for Wesley on the rig.
- Final batch: A + z_ki hold and orbit; repeats of A + z_ki push and wind; repeats of observer push
  and wind.

## Report draft (end of night, 4 Oct ~05:15)

Three-drone rig twin, Gazebo, 1.0 m, floor start; cm; map = one drone's plant offset 0.215 (+) / 0.155 (-) against 0.185 in its tracker; ABORT = a LAND abort after the hold (the hold numbers stand).

| method | hold z | map+ (z / xy / tilt) | map- (z / xy / tilt) | orbit xy / z | push xy | wind circle xy |
|---|---|---|---|---|---|---|
| z_ki 0 (none) | +4.8 (R0841) | -3.0 / 3.8 / 6.7 (R0870) | +12.8 / 3.5 / 6.0 (R0919) | 2.8 / +4.8 (R0832) | 18.0 (R0856) | 17.9 (R0864) |
| z_ki 0.4 | +0.1 (R0871) | +0.0 / 3.8 / 6.7 (R0909) | -0.0 / 3.5 / 6.0 (R0921) | 2.8 / +1.1 (R0910) | 17.9 (R0908) | 18.0 (R0911) |
| A (force balance) | +3.8 (R0915) | +3.9 / 1.3 / 9.0 (R0913) |  | 2.6 / +3.7 (R0914) | 0.3 (R0912) | 3.9 (R0867) |
| INDI f_ext | +4.2 ABORT (R0922) | +3.8 / 3.1 / 0.8 ABORT (R0896) | +4.5 / 0.6 / 0.3 (R0920) | 3.8 / +4.3 (R0923) |  |  |
| A + z_ki 0.4 | +0.1 (R0940)<br>+0.1 (R0953) | -0.0 / 1.3 / 9.0 (R0930)<br>-0.0 / 1.3 / 9.0 (R0955) |  | 2.6 / +0.9 (R0941) | 0.4 (R0928)<br>0.4 (R0942) | 3.9 (R0929)<br>4.0 (R0947) |
| offset-free observer (1.0 s window) | +1.6 (R0906) | +1.0 / 0.2 / 8.0 (R0903) | +2.4 / 0.1 / 7.5 (R0907) | 2.6 / +2.5 (R0902) | 2.6 (R0897)<br>2.6 (R0944) | 7.8 (R0898)<br>7.8 (R0945) |
| kt_trim (thrust adaptation) | +3.7 (R0918) | +4.1 / 1.3 / 2.3 (R0917) |  |  |  |  |
| INDI + z_ki 0.4 | -0.0 ABORT (R0924) | -0.0 / 3.1 / 0.8 (R0926) | +0.0 / 0.6 / 0.3 ABORT (R0927) |  |  |  |
| rod model (F) | +5.0 (R0868) |  |  |  |  |  |

Four drones, even ring, 1 N push: none R0949 15.0 cm / z +3.2; A + z_ki 0.4 R0950 0.1 cm / z +0.2.

SIL (matched plant, z err cm): long hover none +6.3 (R0473), observer one-step -0.7 (R0872), window 1.0 tau 0.5 -0.01 (R0893); n=2 +4.2 (R0879) / -0.5 (R0894).

**Reviewer on A + z_ki 0.4 (4 Oct, 05:20): PARTLY.**
- Every steady-state number recomputes, and the repeats agree within 0.05 cm.
- Missing from the claim: the push transients. The onset peak is 14.5 cm; the release overshoot is
  +14.8 cm, above 3 cm for 3.4 s, with a tilt peak of 7.1 deg (R0856 without A: back under 3 cm in
  1.3 s, tilt peak 3.2 deg).
- The orbit (R0941) and the four-drone push (R0950) are single runs.
- The push metric is the ring against the pre-push reference (R0856 reads 18.5 cm against the
  pre-push ring).
- All of it is sim; nothing here supports a rig claim.
