# Card: the pair's 1.4 Hz swing — which loop carries it, and a twin with the rig's timing (23 Oct 2026)

Owner Wesley. Origin: the pair-swing design (`results/analysis/push_1022_designs.json`, package pair-swing), the
critic's integration notes (`docs/analysis/push_1022_plan.md`: C12, G8, G24 and its 22 Oct 17:11 note) and the
Betaflight readout (`docs/rig/betaflight_2026-10-22.md`). Follows card 2026-10-22_velocity_filter (stage 1b: the
uniform 60 ms twin swings with ema, not with slope). Written 23 Oct ~00:35; **bars PROPOSED**: the integrator freezes
them (with the time) before the first run of each round. Rounds A and B are exploratory rows under this card (one
registry row each, card name in the `card` column). No twin default changes here (plan C12 / Q6).

## 1. Question
Which command path carries the twin's 1.4 Hz pair swing, the throttle or the rates? And does a twin given the rig's
measured per-channel timing (throttle slower, rates faster, the body-rate EMA) swing like the rig pair: the same
filter dependence, frequency, mode shape and power flow, so that it can test fixes?

## 2. Hypothesis
**The timing, rig against twin** (`tools/loop_timing.py`, 22 Oct flights and runs; equivalent delay = tau + T):

| Path | Rig | Twin today | Subcommand |
|---|---|---|---|
| Stick to body rate, tracker log | 25-35 ms (pairs h2, h2slope, h2rts, f2slope, h2t9p); 31-39 (h3, f3h) | roll 56-62, pitch 48-50 ms (R2381/84/85/88 after 24 s, minus their 60 ms) | `att --src log` |
| Stick to body rate, bag | 14-28 ms | no bag | `att` |
| Throttle to lift (system momentum) | 44 ms (h3, f3h), 67 ms (h2slope) | 17-29 ms (R2381 77, R2385 89, minus 60) | `lift` |
| Body rate in the trackers' x0 (raw to x0) | 36-48 ms (h2, h2slope, f2slope, h3: the alpha-0.2 EMA at ~89 Hz) | 4-5 ms (plain difference) | `filter` |

The design quoted 28-33 ms for the body-rate EMA (from x0's command fit minus the quaternions'); the direct raw-to-x0
fit reads 36-48 ms, as an alpha-0.2 EMA at 89 Hz should (4 frames, ~45 ms). The Betaflight readout explains the first
two rows: a setpoint feedforward (F 120-125) on an 8 kHz rate loop, and no loop on collective thrust.

**The mode at f0** (`tools/pair_mode.py`; per deg of see-saw):

| | ring slide (cm) | dz (cm) | c_u, along-line thrust (N s/m) |
|---|---|---|---|
| Rig ema (h2, h2t9p, h2rts) | 0.25 / 0.18 / 0.27 | 0.17 / 0.20 / 0.29 | -0.4 to -1.1 (feeds) |
| Rig slope (h2slope, f2slope) | 0.22 / 0.24 | 0.13 / 0.13 | +0.4 to +1.1 (damps) |
| Twin cd060 (R2381, R2384, R2385, R2388) | 0.04 | 0.36 | +1.3 to +2.2 |

**Round A:** the uniform 60 ms twin's swing rides on the throttle path. 60 ms on the throttle only swings like 60 ms
on all four channels; 60 ms on the rates only stays calm. Basis: with two quads the see-saw is damped only through the
quads' vertical loop (velocity-filter card section 2), and the delayed twin's mode is vertical (dz equals the plates'
motion).

**Round B:** the rig-timing twin swings with ema and not with slope, at the rig's f0, in the rig's mode (the ring
slides along the line, the quads' heights move less than the plates), with the along-line thrust feeding the swing.
- The twin is set to: throttle +35 ms, `rate_kff` 0.012, and the body-rate EMA.
- Prior against it: cd040 (all four channels +40 ms) stayed calm (R2365 M1(b) 0.15 deg; at 1 m, R2369 0.39). The
  rig-timing twin adds only 35 ms, on the throttle alone.
- So it swings only if the faster attitude path or the body-rate EMA matters. The neighbours split the two.

## 3. The one variable
**Round A:** which channel group carries the 60 ms delay from 24.0 s: the throttle, the rates, or all four (cd060).
Everything else is `b2_hover_vf_ema_cd060.yaml` (0.55 m pair hover, ema).
- Arms: `b2_hover_vf_ema_thr060.yaml`, `b2_hover_vf_ema_att060.yaml`, and the neighbour `b2_hover_vf_ema_thr040.yaml`.
- Baseline re-flown on this tree: `b2_hover_vf_ema_cd060.yaml` (G24: R2381-R2388 are on an older tree).
- **Change from the design:** att060 replaces att040. cd040 never swung (R2365 0.15 deg), so "att040 >= 0.5x cd040"
  would have been met by noise alone.

**Round B:** within the rig-timing twin, the carriers' velocity filter (ema against slope), at 0.55 m and 1 m.
- Configs: `b2_hover_vf_{ema,slope}_rt.yaml` (rig-timing from 24.0 s) and `b2_hover_L100_{ema,slope}_rt.yaml` (from
  26.5 s).
- lab_env:
  - `SIM_CMD_DELAY_THR_S` 0.035;
  - `SIM_RATE_KFF` 0.012 (from `SIM_CMD_DELAY_FROM_S`);
  - `SIM_MOCAP_RATE_FILTER` ema (from the start, as on the rig; its from-time is the mocap one, which would also gate
    the velocity filter).
- Neighbours, one switch each against `b2_hover_vf_ema_rt`:
  - `_rt_norate`: no body-rate EMA;
  - `_rt050`: throttle +50 ms;
  - `_rt_nokff`: `rate_kff` 0, the design's "attitude +0" (added for G24).
- `mocap_hz` stays at 120: the twin's ema velocity lag already reads 42 ms (R2381) against the rig's 35-60 ms, so the
  twin's effective mocap rate is close to the rig's 88-91 Hz.

**Round C** (Wesley 23 Oct, decisions.md DECIDED 2026-10-23 (3)): within the rig-timing twin (`b2_hover_vf_ema_rt`'s
lab_env as is) on the tip-hinge world `two_rigid_ground_rig_tip.sdf` (the hinge at the magnet tip, 5.75 cm above the
ring's centre of mass, 4 cm above the magnet face by Wesley's 22 Oct measurement; 0.53 m pivot to tip), what the planner
is typed:
- `b2_hover_tip_typed0175_ema_rt.yaml`: as the rig is (attach_z 0.0175, cable_len 0.55, payload_rest_z 0.0275), a
  deliberate mismatch named under `geometry_exempt` (attach_z, cable_len);
- `b2_hover_tip_typed0575_ema_rt.yaml`: to the hinge (attach_z 0.0575, cable_len 0.53, payload_rest_z 0.02, the world's
  ring at rest). The design's `typed050` at Wesley's 5.75 cm.
- `measure_rod_len` stays on in both, as on the rig: at handover it replaces the typed cable with the measured
  drone-to-typed-rim distance, so typed0175's remaining error is mostly the hinge height.

## 4. Baseline
| Cell | Runs | Numbers |
|---|---|---|
| Twin cd060, 0.55 m, ema | R2381, R2384 (older tree) | M1(b) 6.97 / 7.09 deg; f0 1.5 Hz; slide 0.04, dz 0.36 cm/deg |
| Twin cd060, 0.55 m, slope | R2382, R2383 | M1(b) 0.15 / 0.11 deg |
| Twin cd060, 1 m, ema / slope | R2385, R2388 / R2386, R2387 | 6.76 / 6.68 deg; 0.10 / 0.11 deg |
| Twin cd040, 0.55 m / 1 m, ema | R2365 / R2369 | M1(b) 0.15 / 0.39 deg |
| Rig pair 0.55 m, ema | RIG-1022-h2, -h2t9p, -h2rts | M1(a) 3.72 / 2.60 / 2.99 deg (each hovered < 1 s: no M1(b)) |
| Rig pair 0.55 m, slope | RIG-1022-h2slope, -f2slope | M1(a) 1.46 / 1.35, M1(b) 0.79 / 0.74 deg |

Round A's bars use the cd060 re-fly on this tree; R2381/R2384 stay as its history.

## 5. Pass / fail numbers (FROZEN 23 Oct 00:42 by the integrator, as written, before any run; overnight
authority, Wesley asleep: he may revise them for later runs only)
**Metrics:**
- M1(b): the velocity-filter card's definition, by `tools/pair_swing.py`.
- f0, ring slide, dz and c_u: `tools/pair_mode.py`, twin runs read from FROM + 4.5 s (`Rnnnn:28.5` at 0.55 m,
  `Rnnnn:31` at 1 m) to LAND.
- Every run's launch log must show the `[BF*] command delay thr ... att ...` line, and for round B the
  `[MOCAP*] body-rate filter ema` line, with the configured values. Otherwise the run is void.

**Round A:**
- **Testbed check:** the cd060 re-fly has M1(b) >= 1.0 deg. Otherwise round A is void on this tree: stop and report.
- **Confirms** "the throttle path carries it": both thr060 runs have M1(b) >= 0.5x the re-fly's, and both att060 runs
  < 0.3x it.
- **Falsifies:** either att060 run >= 0.5x the re-fly's, or either thr060 run < 0.3x it.
- Otherwise both paths contribute: reported, no claim.
- **thr040**, a neighbour with no bar: read against R2365 (0.15 deg). A swing >= 1 deg would say that an undelayed
  attitude path makes a 40 ms throttle delay worse.

**Round B, calibration check** (the first `b2_hover_vf_ema_rt` run, flown alone before the rest; not a bar on the
hypothesis):
- `loop_timing.py att <run> --split 24`, window after: the mean of the roll and pitch equivalent delays is 25-36 ms
  (the rig's log readings). The offline model predicts roll ~35, pitch ~25.
- `loop_timing.py filter <run> --split 24`: the body rate raw-to-x0 reads 30-50 ms (rig 36-48).
- If either is outside its band: re-set `SIM_RATE_KFF` with `loop_timing.py kff <run> --split 24` (or set
  `mocap_hz`), re-fly that run, then fly the rest. Record the change in the registry row.

**Round B, "the rig-timing twin reproduces the rig pair"** (0.55 m, both repeats of each arm):
1. ema M1(b) >= 1.0 deg;
2. slope M1(b) <= 0.5x the lower ema run's;
3. ema f0 in 1.25-1.5 Hz;
4. ema mode: ring slide >= 0.15 and dz <= 0.25 cm per deg of see-saw;
5. ema c_u < 0 on both quads (thrust taken 50 ms late: the twin's 15 ms plus the 35).

**Confirms** if 1-5 all hold. **Falsifies** if both ema runs have M1(b) < 1 deg, or both have ring slide < 0.1 cm/deg;
then round C (the hinge) is next. In between, the table says which of 1-5 hold.
- **The 1 m arms** have no bar (no 1 m rig pair has flown yet): their five numbers are recorded as the twin's prediction
  for Wed 28's 1 m pair, before that rig day.
- **Neighbours**, each read against `b2_hover_vf_ema_rt`, no bars:
  - `_rt_norate`: if rt swings (M1(b) >= 1 deg) and norate is <= 0.5x it in both repeats, the body-rate EMA is a cause
    (for fix F3 / D4);
  - `_rt050`: brackets the throttle delay (the rig's pairs read 52-67 ms);
  - `_rt_nokff`: a factor-2 difference from rt, either way, says the attitude path's speed matters.

**Checks reported, not bars:** on the after-window of every B run, `loop_timing.py lift` (the rig reads 44-67 ms) and
`att` (the rig reads 25-35 ms).

**Round C (FROZEN 23 Oct 08:05 by the integrator, as written 07:54, before the first run; the integrator freezes them before the first
round-C run).** Metrics and read windows as round B (0.55 m: `Rnnnn:28.5` to LAND). Prediction: the typed-vs-world
hinge mismatch adds the rig's ring slide and lowers the margin; typing the planner to the hinge restores it.
- **Validity:** each run's launch log shows round B's `[BF*]` and `[MOCAP*] body-rate filter ema` lines with the
  configured values, and the planner's `measure_rod_len` line (its measured rods go in the registry row). Otherwise void.
1. **Testbed:** typed0175's M1(b) >= 1.0 deg in both runs. If not, both arms are calm at the rig's timing: the five
   numbers are reported and no margin claim is made.
2. **Mode:** typed0175's ring slide >= 0.15 cm per deg of see-saw in both runs (the rig's 0.18-0.27) and dz <= 0.25.
3. **Confirms** "the mismatch carries the rig's swing": 1 and 2 hold, and both typed0575 runs have M1(b) <= 0.5x the
   lower typed0175 run's.
4. **Falsifies:** typed0575's M1(b) >= 0.8x typed0175's (each arm's mean over its two runs).
- Otherwise (between 0.5x and 0.8x, or 1 without 2): reported, no claim.
- f0 and c_u are recorded against the rig (1.25-1.5 Hz; ema c_u < 0), no bar.

**Round D, reduced (Wesley 23 Oct, after round C came back calm; FROZEN 23 Oct 11:14, before any round-D run).**
Testbed thr060 (§7: B did not reproduce the rig, A pinned the throttle), no-fix arm R2450/R2451 (M1(b) 7.39/7.36,
mean 7.38). Only the two config-only rig candidates, 2 runs each, read as round A (M1(b) by `pair_swing.py`, f0, slide,
dz and c_u by `pair_mode.py` from 28.5 s):
- D1 `b2_hover_vf_slope_thr060` (velocity filter slope); D2 `b2_hover_vf_ema_thr060_cablemodel` (`cable_source model`).
- **Validity:** the `[BF*] command delay thr 0.060` line, the filter's `[MOCAP*]` line, and for D2 the trackers'
  cable_source model in the run's params. Otherwise void.
- **Confirms** (per arm): both runs have M1(b) <= 0.5x the no-fix mean (3.69 deg) with no new fault.
- **Falsifies** (per arm): the arm's mean M1(b) >= 0.8x the no-fix mean (5.90 deg).
- Otherwise reported, no claim. The testbed swings in the twin's vertical mode (c_u > 0), not the rig's (c_u < 0):
  a confirmation says the fix damps a throttle-delay swing in the twin, not that it fixes the rig's; Wed 28's 1 m rig
  pair is the test. D3-D5 are not flown.

## 6. Repeats
- 2 per arm.
- The cd060 re-fly once: it is the check, and R2381/R2384 are its history.

## 7. Cost
- **Round A:** 7 runs (thr060 x2, att060 x2, thr040 x2, cd060 x1), 2 waves on the lab PC.
- **Round B:** 14 runs (4 rig-timing cells x2, 3 neighbours x2): the calibration run alone first, then 4 waves.
- About 21 runs, ~1.5 h of lab wall time. No laptop runs. Lab rules: check who, the load and the containers first, and
  yield.
- **Round C (approved 23 Oct, DECIDED 2026-10-23 (3); BUILT 23 Oct):** the tip-hinge world
  `two_rigid_ground_rig_tip.sdf` (hinge at the magnet tip, 5.75 cm above the ring's centre of mass; cable 0.53 m).
  - `make_carry_worlds.py`: WORLDS `['--n', '2', '--attach-z', '0.0575']`, the 0.53 m cable in `WORLD_OVERRIDES` (G23),
    the rods' pose at 60 Hz as its 0.55 m twin; `world_expect` reads `--attach-z`; `test_rig_worlds.py` checks the
    hinge height and the pivot-to-tip length; `test_config_checks.py` checks typed0175's named exemption.
  - Arms `b2_hover_tip_{typed0175,typed0575}_ema_rt`, 2 repeats each, 4 runs (bars in section 5). Falsifier:
    typed0575's M1(b) >= 0.8x typed0175's.
- **Round D (plan only):** fixes in the best testbed.
  - Testbed: B if it reproduces the rig; else thr060 if A pins the throttle; else cd060.
  - Candidates:
    - D1 slope (exists);
    - D2 `cable_source model` (exists);
    - D3 `w_vel_scale` 2 (a new tracker constant: twin pass and bars);
    - D4 body-rate filter none (= `_rt_norate`; on the rig, fix F3, which needs a critic);
    - D5 a per-channel Smith predictor (only if A pins the throttle; critic).
  - Each against its no-fix arm, 2 runs. Confirms: M1(b) <= 0.5x the no-fix arm's in both repeats with no new fault.
    Falsifies: >= 0.8x.
  - Then the 3- and 4-quad "not worse" cells on the fix that passes. About 16 runs.
  - D's bars go into this card before D's runs.

## 8. New code?
**Yes.** Twin-only and default off: each default leaves today's twin unchanged, and a test asserts it.
- `simulation_communication/payload_betaflight_comm.py`:
  - per-channel delay `cmd_delay_thr_s` / `cmd_delay_att_s` (env `SIM_CMD_DELAY_THR_S` / `_ATT_S`), added to
    `cmd_delay_s`, both from `cmd_delay_from_s`; `cmd_delay_s` alone behaves as on 22 Oct;
  - `thr_tau_s` (`SIM_THR_TAU_S`): a first-order lag on the throttle's rotor speed;
  - `rate_kff` (`SIM_RATE_KFF`, `rate_kff_tau_s` 5 ms): a feedforward on the rate set point's derivative in the
    emulated rate loop, like Betaflight's F term (`rate_pid.RateFeedForward`);
  - the queue pop is a pure `due()`;
  - one WARN line with every value.
- `simulation_communication/payload_mocap_emulator.py`: `rate_filter` none | ema (`SIM_MOCAP_RATE_FILTER`), the rig's
  alpha-0.2 EMA on every body's angular velocity, gated like the velocity filter.
- `tools/experiment/config.py`: the five names in LAB_ENV; numeric switches must be finite and >= 0.
- `tools/loop_timing.py` and `tools/pair_mode.py`: read-only analysis.
- Tests:
  - `src/simulation_communication/test/test_cmd_delay_split.py`, `test_rate_filter.py`, `test_rate_pid.py` (the
    feedforward);
  - `tools/test/test_loop_timing.py`, `tools/test/test_experiment_config.py`;
  - the three simulation_communication files are in `gate.sh` step 2 (G8).

**rate_kff, how 0.012 was set:**
- `loop_timing.py kff` drives the bridge's own RatePid and RateFeedForward on one axis of the X3 rig model (arms 0.083
  / 0.0765 m, base and rotor inertia, 20 ms motors, the cable's pull at the 4 cm pivot), with the run's logged commands
  and its 60 ms delay. It reads the result as `att` reads the twin.
- At kff 0 it reproduces R2381, R2384 and R2385 within -5 to +2 ms (roll 116-117 against 116-122; pitch 106-107
  against 108-110).
- At 0.012 it reads roll 94-98 and pitch 84-85 ms, i.e. a base of roll 34-38 and pitch 24-25 ms. The mean, ~30 ms, is
  the middle of the rig's log readings (25-35).
- Roll stays ~10 ms slower than pitch at every kff. The twin's roll axis has 12 % less authority per unit inertia;
  the rig reads the two axes alike. One scalar matches the mean only.
- The first round-B run checks it (section 5).

**If it fails:** if round B falsifies, `rate_kff`, `thr_tau_s` and the rate filter are recorded in the registry and
deleted. The per-channel delay stays only as round A's instrument, and only with Wesley's word. Any twin-wide timing
default needs its own card and reviewer (plan section 5).
**Deleted 23 Oct 11:19 AEDT** (Wesley 23 Oct, "Delete 3, keep delay"; rounds B and C null): `rate_kff`
(`RateFeedForward`), `thr_tau_s`, the body-rate filter, `loop_timing.py kff`, the round-C tip world and the nine configs
that used them (`b2_hover_*_rt*`, `b2_hover_tip_*`; each run folder keeps its own in `git_diff.patch`). The per-channel
delay stays, default off. Backup: `/tmp/claude-1000/-home-wesley-multi-drone-control/f2a54328-3214-4ac3-989e-f5a0c5208168/scratchpad/del_rt/backup/`.

## Critic
(pending)

## Results (23 Oct, overnight; registry R2449-R2456)
| Runs | Arm | M1(b) deg | Reading |
|---|---|---|---|
| R2449 | cd060 re-fly (testbed) | 6.95 | testbed valid (>= 1.0) |
| R2450 / R2451 | thr060 (60 ms on the throttle only) | 7.39 / 7.36 | >= 0.5x the re-fly |
| R2452 / R2453 | att060 (60 ms on the rates only) | 0.56 / 0.81 | < 0.3x the re-fly |
| R2454 / R2455 | thr040 | 0.12 / 0.10 | calm, like R2365 |
| R2456 | round B calibration (rt, rate_kff 0.012) | 0.08 | pitch delay 46.5 ms (band 25-36; poor fits, R^2 0.50-0.63; roll unreadable, the pair never rolls) |

**Round A CONFIRMS: the throttle path carries the 1.4 Hz swing** (frozen bars). Two notes: att060 grows a slower
1.0-1.1 Hz mode up to LAND whose shape is the rig's (ring slide 0.64-0.67 cm/deg, along-line thrust feeding it,
c_u < 0); and the two runner FAILs (R2449, R2453, the arms with 60 ms on the rates) are a drone's envelope fault at
touchdown (77.9 deg), after the scored window. **Calibration outside its band:** per section 5, rate_kff re-set to
0.027 (the run's pitch was 8-11 ms slower than the offline model's 0.016 for 30 ms) in every rt config but _rt_nokff;
the calibration run is re-flown alone before the rest of round B. The rig-timing twin at 0.012 damped (c_u +2.4/+1.8).

**Round B (R2465, R2470-R2482; rate_kff 0.027, lab PC)**
| Runs | Arm | M1(b) deg | f0 Hz | slide / dz cm/deg | c_u d0/d1 |
|---|---|---|---|---|---|
| R2465 / R2470 | rt, 0.55 m, ema | 0.066 / 0.081 | 1.50 / 1.50 | 0.029, 0.344 / 0.043, 0.357 | +0.24/-0.99, -0.65/+0.81 |
| R2471 / R2472 | rt, 0.55 m, slope | 0.029 / 0.029 | 1.25 / 1.50 | 0.200, 0.271 / 0.041, 0.332 | -0.27/-0.02, -0.60/+0.31 |
| R2473 / R2474 | rt, 1 m, ema | 0.069 / 0.065 | 1.50 / 1.50 | 0.052, 0.366 / 0.055, 0.368 | -0.49/+0.35, +0.85/+0.59 |
| R2475 / R2476 | rt, 1 m, slope | 0.029 / 0.029 | 1.00 / 1.00 | 0.328, 0.255 / 0.294, 0.254 | +0.00/+0.24, +0.47/+0.01 |
| R2477 / R2478 | _rt_norate | 0.082 / 0.077 | 1.50 / 1.50 | 0.036, 0.352 / 0.039, 0.358 | +0.76/+0.29, +1.41/+0.27 |
| R2479 / R2480 | _rt050 | 4.07 / 5.14 | 1.50 / 1.50 | 0.037, 0.353 / 0.037, 0.353 | +1.11/+0.74, +1.23/+1.34 |
| R2481 / R2482 | _rt_nokff | 0.070 / 0.089 | 1.50 / 1.50 | 0.051, 0.360 / 0.042, 0.356 | +0.97/-0.69, +1.25/+1.82 |

**Round B FALSIFIES "the rig-timing twin reproduces the rig pair"** (frozen bars): both ema runs are under 1 deg and
both ring slides under 0.1 cm/deg (items 1, 4, 5 fail; 2 holds only at the noise floor; 3 at 1.50 Hz). The 1 m
prediction for Wed 28 (from a twin that missed the 0.55 m rig pair): calm with either filter. Neighbours: norate has no
reading (rt does not swing); rt050 swings (4-5 deg, growing to ~19 deg p-p) in the twin's vertical mode (c_u > 0), so the
twin starts to swing between +35 and +50 ms of throttle delay; nokff ~1.1x rt (the attitude path's speed does not matter
here; its pitch delay 73-86 ms against rt's 35-38). Reported: att pitch 31-39 ms (rig 25-35); lift unreadable in calm
runs. **Next by the card:** round C (the hinge world: a world SDF, Wesley's word). §8's deletion of rate_kff, thr_tau_s
and the rate filter is held for Wesley's morning decision, since round C was designed on the rig-timing twin.

**Round C (R2598-R2601; the tip-hinge world, scored 23 Oct ~10:00):**
| Runs | Arm | M1(b) deg | f0 Hz | slide / dz cm/deg | c_u d0/d1 |
|---|---|---|---|---|---|
| R2598 / R2600 | typed0175 (planner typed as today, world hinge at the tip) | 0.09 / 0.08 | 1.50 / 1.50 | 0.054, 0.276 / 0.050, 0.261 | +0.52/+1.05, -0.04/+0.17 |
| R2599 / R2601 | typed0575 (planner typed to the hinge) | 0.08 / 0.08 | 1.50 / 1.50 | 0.046, 0.298 / 0.034, 0.289 | +0.80/+0.52, -0.14/+0.68 |

**Round C is null** (frozen bars): the testbed bar fails (typed0175 M1(b) 0.09 / 0.08 < 1.0 deg; ring slide 0.054 /
0.050 < 0.15), so no margin claim is made; the tip hinge with the mismatched typing stays calm at the rig's timing.
Then (Wesley 23 Oct): round D reduced to the config-only rig candidates; rate_kff, thr_tau_s, the rate filter and the
tip world deleted (§8 line above).

**Round D, reduced (R2691-R2694; lab batch 17; bars frozen 11:14; scored 13:59):**
| Runs | Arm (on thr060) | M1(b) deg | f0 Hz | dz cm/deg | c_u d0/d1 |
|---|---|---|---|---|---|
| R2450 / R2451 | no fix (ema, cable_source mocap) | 7.39 / 7.36 | 1.50 | - | - |
| R2691 / R2692 | D1 slope | 0.07 / 0.06 | 1.50 / 1.50 | 0.360 / 0.366 | +1.38/+0.17, +2.64/+1.05 |
| R2693 / R2694 | D2 cable_source model | 0.01 / 0.01 | 1.00 / 1.00 (no 1.5 Hz mode) | 0.492 / 0.349 | +0.48/+0.59, +0.41/+0.14 |

**Both arms CONFIRM** (both runs <= 3.69 deg, no new fault): each removes the twin's throttle-delay swing. As frozen,
this says the fix damps a throttle-delay swing in the twin, whose mode is the twin's (c_u > 0), not that it fixes the
rig's (c_u < 0). D1 (slope) is already Wed 28's pair A/B. D2 says the swing runs through the trackers' measured cable
pull: without it the 1.5 Hz mode is gone. The rig's measured pull also carries Tier A's gains (R1706-R1711), so D2 is a
candidate for the pair only, after Wed's slope A/B, not a default. D3-D5 not flown.
