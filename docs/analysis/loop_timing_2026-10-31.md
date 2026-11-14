# Loop timing: the control loop's delays, real and simulated (31 Oct 2026)

Request Q005 (`docs/lab_requests.md`), modelled on the timing table of the supervisors' CEP paper (Table 1 and its delay
paragraph, p8). Offline and exploratory: nothing was flown or simulated, no card, no reviewer. It reads existing bench
files, bags, tracker and planner logs and blackbox logs. Nothing but this file was written in the repo.

## 1. Verdict

| # | Finding | Size |
|---|---|---|
| 1 | Real-world finding, by accident: at 4 carriers a planner tick can start on a stale ring pose | 9.5 % over 30 ms |
| 2 | No flight has flown the reduced body set, and its mocap delay is unmeasured (first chance 4 Nov) | not measured |
| 3 | The slope filter's lag is smaller than the ~20 ms quoted, which came from 50 Hz sampling | 16-17 ms |
| 4 | The flight controller's own rate response, new here, one quad | 11-13 ms |
| 5 | Throttle to thrust was never measured directly. The in-flight reading at the FC is weak | 9-19 ms |
| 6 | The simulation's rate path is far slower than the real one on the same manoeuvre | 83-100 vs 34-39 ms |
| 7 | Simulated time does not wait for the planner. The plan arrives late in simulated time too | 22-39 vs 74 ms |

**Finding 1.** At 4 carriers, 9.5 % of the planner's ticks started on a ring pose older than 30 ms and 4.2 % on one
older than 90 ms (max 121.5 ms, 1372 ticks, 6 flights of 29 Oct). 85 % of those ticks came within 4 ms of the previous
tick's end. So when a tick overruns, the next one fires before the pose callbacks have run, and it pins node 0 to a pose
from before the overrun. A reading, not tested: the single-threaded executor serves the due timer first. At 2 and 3
carriers it is at most 0.2 %. The drones' poses come through the same callbacks and were not checked. Worth knowing
before the next 4-carrier flight (h4m1, 4 Nov).

## 2. The table

Values are ms. Solve times are median / p95. Keys M1-M8 are in section 3 and flags A-E in section 5.

| Row | Delay | Real | How | When | Flag | Evidence |
|---|---|---|---|---|---|---|
| 1a | Mocap delivery, 1 body | 7.5 | M1 | 30 Oct | D, E | RIG-1030-bench_mocap |
| 1b | Mocap delivery, 10 bodies | 95.0 | M1 | 30 Oct | D, E | RIG-1030-bench_mocap |
| 1c | Mocap delivery, the flight's set | not measured | - | - | - | section 6 |
| 2a | Velocity lag, carriers (slope) | 16-17 | M2 | 28-30 Oct | A | RIG-1022-h2slopec, section 4.2 |
| 2b | Velocity lag, ring and QUAD4 (ema) | 45-51 | M2 | 28-30 Oct | A | section 4.2 |
| 3a | Planner solve, 2 carriers | 16.1 / 34.0 | M3 | 28 Oct | A, C | RIG-1028-hE1 to -ls2c |
| 3b | Planner solve, 3 carriers | 32.9 / 58.5 | M3 | 30 Oct | B | RIG-1030-h3t, -h3tp |
| 3c | Planner solve, 3 carriers, QUAD4 up | 34.4 / 44.4 | M3 | 29 Oct | A | RIG-1029-k1_4 to -sq5_2 |
| 3d | Planner solve, 4 carriers | 54.1 / 71.8 | M3 | 29 Oct | A | RIG-1029-h4, -h4f_3, -f4, -ch1* |
| 4a | Tracker solve, 2 carriers | 0.82 / 2.37 | M4 | 28 Oct | A, C | as 3a |
| 4b | Tracker solve, 3 carriers | 0.98 / 2.97 | M4 | 30 Oct | B | as 3b |
| 4c | Tracker solve, 3 carriers, QUAD4 up | 1.16 / 2.95 | M4 | 29 Oct | A | as 3c |
| 4d | Tracker solve, 4 carriers | 1.02 / 2.12 | M4 | 29 Oct | A | as 3d |
| 5 | Radio, our write to the FC | 6.5 | M5 | 30 Oct | D, E | RIG-1030-bench_radio |
| 6a | Throttle to thrust, direct | not measured | - | - | - | section 6 |
| 6b | Motor command to lift, at the FC | 17-19 free, 9 loaded | M6 | 15, 30 Oct | A, B, D | section 4.6 |
| 6c | Throttle to lift, whole loop, 3 carriers | 44 | M7 | 22 Oct | A, C | RIG-1022-h3, -f3h |
| 6d | Throttle to lift, whole loop, 4 carriers | 56-65 | M7 | 29 Oct | A | RIG-1029-h4, -h4f_3, -f4 |
| 6e | Throttle to lift, whole loop, 2 carriers | 50-112 | M7 | 22, 28 Oct | A, C | section 4.6 |
| 7a | Rate command to rate, at the FC | 11-13 | M6 | 30 Oct | B, D | RIG-1030-h4n |
| 7b | Rate, whole loop, 4 carriers | 35.5-39 | M8 | 29 Oct | A | RIG-1029-h4, -h4f_3, -f4 |
| 7c | Rate, whole loop, 3 carriers, QUAD4 up | 33-36 | M8 | 29 Oct | A | RIG-1029-sq2_5 and 4 more |
| 7d | Rate, whole loop, 2 carriers | 25.5-32 | M8 | 28 Oct | A, C | RIG-1028-hE1 and 8 more |
| 8a | End to end: body moves, FC answers in rate | 40-46 | sum | - | all | section 4.8 |
| 8b | End to end: body moves, FC answers in thrust | 46-52 | sum | - | all | section 4.8 |
| 8c | End to end: ring moves, plan at tracker, 2 / 3 / 4 | 42 / 68 / 89 | sum | - | all | section 4.8 |
| 9 | Tracker's rate model, tau | 120, assumed | - | - | - | `tracker_node.py:688` |

Not measured: rows 1c and 6a. Row 9 is a model constant, not a measurement. The tracker's rate model (tau 0.12 s, 100
deg/s) is not sourced in the thesis, by Wesley's decision of 30 Oct (`docs/decisions.md`, 16:52). 100 deg/s is the FC's
configured rate (QUAD3's CLI on 22 Oct, QUAD4's blackbox headers). The rate response of row 7a is much faster than a
0.12 s lag, as `docs/analysis/p6_offline_2026-10-12.md` said from older data.

## 3. How each was measured

| Key | Method |
|---|---|
| M1 | `tools/bench_latency.py`, props off, QUAD1 on USB, twisted by hand 40 s. Lag of the peak correlation between |
|  | FC gyro speed (MSP over USB, 50 Hz) and mocap speed (raw UDP), net of half the MSP round trip of 10.0 ms |
| M2 | `velocity_filter.py` replayed on each bag's stamps and poses, which matches the published twist. By |
|  | construction: 1.5 frames of the node's real spacing. Fit: twist against a central difference of poses, 1-10 Hz |
| M3 | Planner `log.csv`: `solve_ms` (`solve_horizon`, residual inside), `res_ms`, `tick_ms`. Phase `planner`, land 0, |
|  | status 0 |
| M4 | Tracker `log.csv`: `solve_ms` (`ocp.solve()`, one SQP_RTI step, 20 nodes). Rows with the quad 0.15 m over rest |
| M5 | `tools/bench_latency.py`, props off, drives the TX at 100 Hz with random roll steps. Lag of the peak |
|  | correlation between the roll written and the roll the FC reports over MSP, net of half the round trip |
| M6 | QUAD4's blackbox at 2 kHz, one clock, no radio or mocap. Step by Wiener deconvolution over 0.5 s segments. |
|  | Read at 50 % of the step |
| M7 | `tools/loop_timing.py lift --src bag`: summed commanded thrust against the vertical momentum of quads and ring, |
|  | closed loop, fit of delay and lag, 1-10 Hz. Masses set per flight (0.55 / 0.86, or 0.583 / 1.15 kg) |
| M8 | `tools/loop_timing.py att --src log`: the tracker's stick against the body rate of the pose it was handed, |
|  | 1-10 Hz |

"Whole loop" means the command path, the airframe and the mocap path together. Rows 6c-6e and 7b-7d already contain rows
1a and 5, so never add them. The ground computer is this laptop: Intel Core 5 210H, 8 cores, 12 threads, up to 4.8 GHz,
with the whole rig stack and the bag recorder on it. The planner ran pinned to CPUs 0-7 through `taskset`
(`MDC_PLANNER_CPUS=0-7`, seen in 29 Oct's `h4.log`).

## 4. Row by row

### 4.1 Mocap delivery (rows 1a-1c)

Re-read from the saved files with the tool's own analysis. The values reproduce.

| File (`results/rig/2026-10-30/`) | Bodies | Per body | Delay | Per 10 s window | Correlation |
|---|---|---|---|---|---|
| `bench_latency_113009.npz` | 1 | 239.7 Hz | 7.5 | 7.5, 6.0, 5.0 | 0.97 |
| `bench_latency_113538.npz` | 10 | 225.0 Hz | 95.0 | 100, 91.5, 96 | 0.92 |

- **Inside the number:** Motive (smoothing 10, RIG-1030-solo1), the forwarder, the network and a Python socket read. Our
  mocap node is not in it. From its stamp to the recorder adds 0.3-0.5 ms (bags of 29-30 Oct). The FC's gyro filters
  (about 1-2 ms, an estimate in `docs/rig/betaflight_2026-10-22.md`) sit on the other side, so the true value is that
  much larger.
- **The flight's own set is not measured.** Only 1 and 10 bodies have flown this bench. The 4 Nov sheet's sets are 7
  bodies each and its P2 line runs this bench with the set. Flights stream at most 8 from now (1,920 packets/s against
  the forwarder's ~2,250). The 9-body T1 (`tools/t1_four_markers.sh`, 2,160 packets/s) is still OPEN in
  `docs/decisions.md`.
- **In-flight check, indirect:** 29 Oct's 4-carrier flights routed 9 bodies (8, 11-14, 21-24). How many were streamed is
  not recorded. Their stick-to-pose loop (M8 with `--src bag`) read 23.5-27.5 ms against 98-133.5 ms on 30 Oct. So there
  was no queue on 29 Oct, and the bench's 87.5 ms difference shows in flight (h4 to h4n: +82.5 to +85.5 ms).
- **In-flight check, the sum:** command path plus mocap delay, QUAD4's blackbox against our bag, re-done from the raw
  files because the scripts behind the registry row are gone. RIG-1030-h4n_bb2's method, signed roll and pitch rates,
  4 s windows.

| Flight | Windows used | Command path + mocap | RIG-1030-h4n_bb2 | Bench, for scale |
|---|---|---|---|---|
| RIG-1015-q4solo | 7 of 11 | 17 (8 s windows: 13) | 16 | 7.5 + 6.5 + hold 5 = 19 |
| RIG-1015-q4circle_2 | 19 of 21 | 21 (8 s windows: 20) | 15 | 19 |
| RIG-1030-h4n | 7 of 7 | 114 | 109 | 95 + 6.5 + 5 = 106.5 |

### 4.2 The velocity estimate (rows 2a, 2b)

The ~20 ms is a rounded reading. RIG-1022-h2slopec (reviewer, 22 Oct) already said the "20 / 40 ms" came from run.csv's
50 Hz sampling and that the bags read 16-17 against 45-47. The 28 Oct decision and the review repeat 20. Measured again
on the latest flights, on the node's own stamps:

| Body, filter | Frames at the node | By construction | Fit | R2 |
|---|---|---|---|---|
| Carriers, slope over 4 frames | 91-94 Hz | 15.8-16.7 | 16-19 | 0.98-1.00 |
| Ring and QUAD4, ema alpha 0.2 | 89-94 Hz | 4.5 frames, ~50 | 45-51 | 0.94-1.00 |

- Flights: hS1, hS2, f2a-c, ls2a, ls2b_4, ls2c (28 Oct), h4, h4f_3, f4, sq2_5, sq5_2, k1_8 (29 Oct), h3tp (30 Oct). One
  outlier, left out above: h4's ring (ema), 29 ms, K 0.41, R2 0.58, too calm to fit.
- **Why ~91 Hz, not 240 or 120:** the node caps each body at 120 Hz (`motion_capture_publisher_node.py:50`), so on a 240
  Hz stream it passes every 2nd or 3rd frame. The cap replayed on the bench's raw arrivals gives 91.1 Hz and 16.5 ms at
  240 Hz, and 92.2 Hz and 16.3 ms at 225 Hz. So the reduced set should keep 16-17 ms, by construction, not yet flown.
- **The tracker-log reading** (`tools/loop_timing.py filter`) gives 16-20 ms on 29 Oct's 4-carrier flights (R2
  0.89-0.95) and 25-50 ms on calmer flights (R2 0.51-0.90). Those higher values are a noisy calm signal. The bag reading
  stands.
- **Body rate in the tracker's state, same path, not asked for:** still the alpha 0.2 ema on every body. Raw to x0 reads
  a median of 43-48 ms per day (range 32-55, 28-30 Oct, K over 0.6), about 2.5-3x the velocity's lag.

### 4.3 The planner on the ground computer (rows 3a-3d)

All at `plan_sqp_iters` 0, meaning 5 warm iterations, with the real budget of 60 ms (checked after each iteration, so a
slow tick drops iterations). The residual is inside `solve_ms` (`plan_res_every` 1, `plan_fast_refs` off). `tick_ms`
runs from the timer's start to the log row, so it holds the reference extraction and the publishing. "Plan out" is the
tick's start to the plan's arrival at a tracker (planner row time minus `tick_ms`, against tracker row time minus
`ref_age`), over all trackers. That is the delay the trackers see.

| Setup | Flights | Ticks | Solve | No residual | Whole tick | Plan out | 5 iters | Over 60 |
|---|---|---|---|---|---|---|---|---|
| 2, 28 Oct | 10 | 3359 | 16.1 / 34.0 | 14.7 / 31.4 | 30.0 / 54.6 | 28.8 / 56.2 | 100 % | 0 % |
| 3, 30 Oct (h3t, h3tp) | 2 | 310 | 32.9 / 58.5 | 30.1 / 55.3 | 51.9 / 82.5 | 51.4 / 79.4 | 100 % | 4.5 % |
| 3, QUAD4 up, 29 Oct | 11 | 3973 | 34.4 / 44.4 | 30.8 / 40.4 | 62.0 / 74.9 | 54.3 / 76.3 | 100 % | 0.1 % |
| 3, QUAD4 up, 30 Oct | 4 | 241 | 31.6 / 37.1 | 28.3 / 33.2 | 56.3 / 65.5 | - | 100 % | 0 % |
| 4, 29 Oct | 6 | 1372 | 54.1 / 71.8 | 47.7 / 65.0 | 79.8 / 99.3 | 74.0 / 93.5 | 95.6 % | 25.4 % |
| 4, 30 Oct (h4n) | 1 | 76 | 57.8 / 72.7 | 51.5 / 66.5 | 82.9 / 100.2 | - | 93.4 % | 39.5 % |

- "QUAD4 up" means QUAD4, its tracker and Tejen's stack run on the same laptop. The first column is the carriers.
- **Exactly 5 iterations at 4 carriers:** 53.7 / 69.8 (29 Oct), 56.6 / 72.2 (h4n). 4.3 % of 29 Oct's ticks ran over the
  100 ms period (finding 1). The flight range of the medians is 52.7-56.4 and of the p95s 65.3-73.4.
- **Plan out** used hS1 to ls2c, k1_4 to sq5_2, h3t, h3tp, and h4 to ch1b_2.
- **The older stack, for reference:** 22 Oct (0.86 kg, 0.55 m, no `res_ms` or `tick_ms`, not pinned): 2 carriers 17.4 /
  39.5, 3 carriers 38.5 / 70.7 with 6.6 % of ticks under 5 iterations. The pin is the likely reason 3 carriers read 34.4
  / 44.4 on 29 Oct, not tested (`docs/experiments/2026-10-23_scalability.md`, B calibration).
- **The same solve, idle** (S-BENCH-1022, residual inside, node-by-node extraction, 200 warm ticks): 11.1 / 11.3, 20.3 /
  20.6, 38.2 / 38.8 at 2, 3, 4 carriers, whole tick 15.9, 27.5, 48.2. In flight the median is 1.4-1.7x and the p95
  1.9-3.0x.

### 4.4 The tracker on the ground computer (rows 4a-4d)

The solve is the only part of a tracker's tick the logs time, so the whole tick cannot be separated. Tick spacing is the
log rows' time difference. Attach flights leave out QUAD4's tracker. h4n for the record: 1.14 / 2.33, spacing 31.3 at
p95.

| Setup | Trackers | Ticks | Solve | p99 | Max | Tick spacing p50 / p95 / p99 |
|---|---|---|---|---|---|---|
| 2, 28 Oct | 20 | 45006 | 0.82 / 2.37 | 3.32 | 8.33 | 20.0 / 29.5 / 36.2 |
| 3, 30 Oct (h3t, h3tp) | 6 | 8559 | 0.98 / 2.97 | 3.56 | 6.21 | 20.1 / 30.9 / 39.6 |
| 3, QUAD4 up, 29 Oct | 27 | 81522 | 1.16 / 2.95 | 4.51 | 11.43 | 19.9 / 33.0 / 39.1 |
| 4, 29 Oct | 24 | 46450 | 1.02 / 2.12 | 2.82 | 10.14 | 20.0 / 30.8 / 35.9 |

Pose age at the tracker: the log row's pose matched to the bag's message at 1 mm, against the row's time, after the
solve. Median per flight 10-14 ms, p95 22-40 ms, 17 flights, 28-30 Oct. That reproduces the "11-14 ms" noted on 30 Oct
at 09:21. It also accounts for most of the 11-16 ms gap between M8's two sources (`--src log` against `--src bag`).

### 4.5 The radio (row 5)

| File | Delay | Per 10 s window | Correlation | Within 0.005 of the peak |
|---|---|---|---|---|
| `bench_latency_102620.npz` | 6.5 | 9.5, 6.5, 4.0, 5.0 | 0.94 | 1.5-12.0 |

- QUAD1 only, disarmed, MSP sampled at 50 Hz, so the resolution is coarse. No ROS nodes in it.
- **Not in it:** `elrs_interface.py` holds the latest frame for 0-10 ms (100 Hz writer, mean 5 by construction).
- Whether MSP's RC reading sits before the FC's RC smoothing was not checked. RC smoothing (cutoff 94 Hz, blackbox
  headers) is about 2-3 ms, an estimate in `docs/rig/betaflight_2026-10-22.md`, not measured.
- Radio frames reach QUAD4's FC every 4006-4007 us (250 Hz, blackbox headers, 15 and 30 Oct). The note's 333 Hz and F500
  are inferences from the link's mode number.

### 4.6 Throttle to lift (rows 6a-6e)

**Direct:** not measured. No thrust stand or load-cell test exists. `tools/thrust_fit.py` and `thrust_check.py` are
static.

**At the FC, from blackbox.** Throttle after RC smoothing to the motors: 0 ms (R2 0.97-1.00), which reproduces
RIG-1030-h4n_bb. Motors to body z specific force is the rest. The FC low-passes that accelerometer at 10 Hz
(`acc_lpf_hz` 10, second order, as its spectrum shows, 23.0 ms to 50 % by calculation). The excess over that is rotor
plus airframe.

| Flight (QUAD4) | Motors to az, 50 % | Excess over the filter | 90 % interval | Note |
|---|---|---|---|---|
| RIG-1015-q4solo | 40-42.5 | 17-19.5 | 6-26 | free hover, cable hanging |
| RIG-1015-q4circle_2 | 40.5-42 | 17.5-19 | 9-26 | free circle, cable hanging |
| RIG-1030-h4n | 32-32.5 | 9-9.5 | 5-15 | in the heave, the cable's pull is in the accelerometer |

Weak evidence: small signals, one quad, the filter removed by calculation. The direct 1-10 Hz fits are poorer still (R2
0.08-0.27 free). The registry's 36 vs 16 ms were marked not trusted (RIG-1030-h4n_bb2). This is the better reading, not
a final one.

**Whole loop**, `tools/loop_timing.py lift --src bag`. It includes hold, radio, FC, rotor and mocap delivery. Re-run, it
reproduces the card's 44 (h3, f3h) and 67 (h2slope). The review's "44-67" is only those three flights.

| Set | Flights: delay (R2) |
|---|---|
| 3 carriers, 22 Oct | h3 44 (0.89), f3h 44 (0.77) |
| 4 carriers, 29 Oct | h4 56 (0.72), h4f_3 63 (0.62), f4 65 (0.67) |
| 2 carriers, 22 Oct | h2 50 (0.69), h2t9p 53 (0.82), f2slope 67 (0.66), h2slope 67 (0.37), h2rts 82 (0.60) |
| 2 carriers, 28 Oct, R2 >= 0.6 | f2c 60 (0.64), hE1 63 (0.81), f2a 82 (0.71), ls2b_4 86 (0.65), f2b 112 (0.67) |
| 2 carriers, 28 Oct, R2 < 0.6 | hS2 34 (0.43), hS1 64 (0.54), ls2a 92 (0.45), ls2c 105 (0.20) |
| 3 carriers, 30 Oct | h3tp 113 (0.60). h3t and h4n: too short to fit |

Attach flights are left out, because QUAD4's thrust is not ours. Pairs scatter too widely to quote one number. Row 6e's
50-112 is the R2 >= 0.6 range.

### 4.7 Rate command to rate (rows 7a-7d)

**At the FC**, setpoint (after RC smoothing) to filtered gyro, 90 % bootstrap interval over segments in brackets.

| Flight (QUAD4) | Roll, 50 % | Pitch, 50 % | Step gain | Fit 1-10 Hz, eq (K, R2) |
|---|---|---|---|---|
| RIG-1030-h4n (well excited) | 10.5-11 [9.5-12] | 12-13 [11-13.5] | 1.0-1.3 | 2-4 (1.2, 0.82-0.88) |
| RIG-1015-q4circle_2 | 12-13 [11-14.5] | 15-17.5 [14-19] | 1.6-2.2 | 9-15 (2.7, 0.71-0.73) |
| RIG-1015-q4solo | 13 [10.5-15] | 17 [14-19] | 2.2, 2.7 | 18-19 (3.5-4, 0.59-0.69) |

- Only h4n has a gain near 1, so row 7a takes h4n. In the calm flights the setpoint is under 3 deg/s and the tracker's
  answer to the motion dominates the correlation.
- These are closed-loop data, and no dedicated step test exists. The 1-10 Hz fit reads less than the 50 % step (2-4
  against 11-13), probably because the FC's feedforward leads at low frequency. They say different things. The loop's
  phase at the ring's frequencies follows the low-band one.

**Whole loop**, `tools/loop_timing.py att`, per-flight axis medians in ms:

| Set | `--src log` (M8, to the pose at the tracker) | `--src bag` (sent to pose stamped) | K | R2 |
|---|---|---|---|---|
| 2 carriers, 22 Oct | 28-32.5 (per drone 25-35, the card's) | 16.5-20.5 | 0.4-1.4 | 0.36-0.74 |
| 3 carriers, 22 Oct | 33-38 (per drone 31-39, the card's) | 18-24 | 1.0-1.4 | 0.60-0.72 |
| 2 carriers, 28 Oct | 25.5-32 | 13-19.5 | 0.5-1.6 | 0.48-0.72 |
| 3 carriers, QUAD4 up, 29 Oct | 33-36 | 17-21 | 1.0-1.5 | 0.57-0.77 |
| 4 carriers, 29 Oct | 35.5-39 | 23.5-27.5 | 1.0-2.6 | 0.55-0.93 |
| 30 Oct, 10 bodies, all sets | 109-148.5 | 98-133.5 | 0.6-2.6 | 0.60-0.86 |

P6 of 12 Oct (`tools/rate_latency.py`, flights of 7-8 Oct) read 20-24, an older stack and method. The review's "25-35"
is the 22 Oct pairs. For the current setup use row 7b.

### 4.8 End to end

A sum, not a measurement. Nothing has measured the whole path on the reduced stream.

**Tracker loop:**

| Step | ms | Kind |
|---|---|---|
| Mocap delivery | 7.5 | bench, one body, row 1a |
| Pose age at the tracker, solve included | 10-14 | flights, median per flight, section 4.4 |
| Radio writer's hold | 5 | by construction, 0-10 |
| Radio | 6.5 | bench, row 5 |
| FC, 50 % of a rate step | 11-13 | blackbox, row 7a |
| **Sum, FC answers in rate** | **40-46** | row 8a |
| Same, with the rotors (17-19) in place of the rate step | **46-52** | row 8b |
| Add for velocity feedback (slope) | +16-17 | row 2a |

**Planner loop** (medians, p95 in brackets):

| Step | 2 carriers | 3 carriers | 4 carriers |
|---|---|---|---|
| Mocap delivery | 7.5 | 7.5 | 7.5 |
| Ring pose age at the tick's start | 6.1 (11.5) | 6.3 (12.8) | 7.3 (52.2) |
| Tick's start to the plan at a tracker | 28.8 (56.2) | 54.3 (76.3) | 74.0 (93.5) |
| **Sum, ring moves, plan at a tracker** | **42** | **68** | **89** |
| Then the plan is held, `ref_age` at a tracker tick | 55.6 (103.5) | 55.5 (100.9) | 54.6 (101.6) |

**Caveats:**
- A sum of medians is not the median of a sum. The tails are much longer (p95 22-40 ms for the pose age alone).
- The 7.5 is one body on a bench, the 6.5 is QUAD1, the FC values are QUAD4. None is on the reduced flight stream.
- Not counted: RC smoothing (~2-3, an estimate) and half the 50 Hz command period (10, sampled data). Not counted the
  other way: the tracker sends node 1 of its plan, 0.1 s ahead, a lead of unknown net size
  (`docs/experiments/2026-10-14_tracker_timing.md`).
- The ring's velocity in the planner's state is +45-51 ms and the body rate in the tracker's state +43-48 ms.

**Cross-checks against the direct readings:**

| Path | The parts add to | Measured | Reading |
|---|---|---|---|
| Stick to pose at the tracker, FC 2-13 | 31-46 | 25.5-39 | agrees on the 3 and 4 carrier flights |
| Stick to pose stamped, FC 2-13 | 21-32 | 13-27.5 | the pairs read under the sum, a closed-loop fit is not additive |
| Command path + mocap, QUAD4 | 19 | 13-21 | agrees |
| Throttle to momentum, rotor 17-19 | 36-38 | 44, 56-65 | 6-29 ms not accounted for, cause open |

### 4.9 The simulation beside the real system

The simulation's claim tables fly with no added delay (`docs/decisions.md`, 22 Oct ~15:30). The delay switches below
exist and are default off. Runs that use them are exploratory or a named arm.

| Stage | Real | Simulation | Kind, in simulation |
|---|---|---|---|
| Mocap delivery | 7.5 | 0 | not modelled (`SIM_MOCAP_DELAY_S`, off, R2909-R2914) |
| Pose stream per body | 91-94 Hz | ~100 Hz | by construction: 500 Hz poses, 2 ms clock, 120 Hz cap |
| Velocity lag, carriers | 16-17 | ~15 | same filter, by construction, too calm to fit (R2835) |
| Body rate in the tracker's state | 43-48 | 5-6 | plain difference (R2835, R2 0.86-0.96) |
| Radio and its hold | 6.5 + 5 | 0 | not modelled (`SIM_CMD_DELAY_S`, off) |
| RC smoothing | ~2-3 (estimate) | 0 | not modelled |
| FC rate response, 50 % of a step | 11-13 | 47.5-51.5 | calculated: PI, kp 0.5, ki 5, no feedforward, 4 carriers |
| Rate, whole loop, 4 carriers, figure-8 | 34-39 (f4) | 83-100 (R2835) | same tool, same path, R2 0.60-0.86 |
| Rotor, 50 % of a step | 9-19 | 13.9 | an assumed constant: 20 ms time constant in the SDF |
| Throttle, whole loop | 44-65 | 17-29 | R2381, R2385 flown with +60 ms, minus 60 |
| Solve budget | 60 | off | `params.py:47` |
| Plan out, 4 carriers | 74.0 / 93.5 | 22-24 / 30-31 | simulated ms, lab PC (R2915, R2916) |
|  |  | 38-39 / 76-78 | simulated ms, laptop (R2833, R2835) |
| Tracker tick spacing, p95 | 29.5-33 | 22-23 | R2835, R2915, R2833 |

- **The rate path.** The review's and the draft's "48-62" is 108-119 from runs with +60 ms, minus 60 (re-read here, it
  reproduces). Those fits are not additive. Read directly on an undelayed run the same tool gives 83-100, and the
  calculation from the constants gives 77-88 in its 1-10 Hz fit. So the 4-carrier figure-8 compares 34-39 real (f4)
  against 83-100 simulated, about 2.3-2.6x on the axis medians. Calm undelayed hovers (R2833, R2915) cannot be read (R2
  under 0.6).
- **The throttle path.** Undelayed runs cannot be read: R2833 9 ms (R2 0.16), R2835 13 ms (R2 0.26). The 17-29 stands
  only as a delayed-run subtraction (re-read: 17 and 27). Rotor against rotor the lags are close, so at least 19 ms of
  the real system's longer reading is the path the simulation leaves out (hold, radio, mocap), not the motors. That
  corrects the reading in `docs/rig/betaflight_2026-10-22.md` ("the motors' spin-up shows in full"). A reading on weak
  data.
- **Simulated time and the planner.** Gazebo has no lockstep (`tools/sil/lockstep.py` is the SIL bench only). It keeps
  running at RTF 0.15-0.28 while the planner solves, so the plan arrives at about wall tick x RTF: R2915 24 simulated ms
  (108.5 x 0.241 = 26), R2866 16 (105.9 x 0.152 = 16). The simulation has a shorter planner delay than the real one, not
  none, and a slower run hides more of the solve.

## 5. Flags on dates and stacks

| Flag | Meaning |
|---|---|
| A | Flown before the body-set change of 30 Oct 11:41. No flight has flown the reduced set. How many bodies were |
|   | streamed on 15, 22, 28 and 29 Oct is not recorded. The 29 Oct bags show at most 9 routed and no queue (4.1) |
| B | Flown on 30 Oct with 10 bodies, poses ~95 ms late, every ring lift heaving. Solve times stand as CPU numbers |
| C | The earlier setup: 0.86 kg M2A ring, 0.55 m cables |
| D | One airframe: QUAD1 on the bench, QUAD4 in blackbox. QUAD3's flash is not detected, QUAD1 and QUAD2 not read |
| E | A bench without our ROS nodes |

**The code against the current tree.** `git diff 284ffb4 HEAD` is empty for `src/mpc_planner`, `src/tracker` and
`src/drone_communication`, and the 30 Oct flights' uncommitted changes do not touch them. `src/mpc_planner`, which holds
the solve, is unchanged since 413ec0e, the 28-29 Oct flights' base. Their tracker and mocap node differ at most by the
pull filters and the slope default. `src/dissipative_planner`, whose node wraps the planner's tick, has changed since
(X1's `resize_delay_s`, X4's relay, both behind switches, default off). Its effect on `tick_ms` is not measured. No
flight has flown the current tree.

## 6. Not measured, and what would measure it

| Item | Status | What would measure it |
|---|---|---|
| Mocap delay, the flight's set | not measured | P2 of `docs/rig/wed_2026-11-04.md`: `bench_latency.py` with that set |
| Throttle to thrust, directly | not measured | A load cell or thrust stand, throttle steps at hover, FC blackbox |
| Throttle to lift, clean, in flight | weak data | `acc_lpf_hz` raised for one flight. FC change: supervisor's OK |
| Rate step, open loop | closed-loop data | A scripted roll and pitch step or chirp in free hover, QUAD4's blackbox on |
| QUAD1-3's FC lags | not read | Blackbox on them, if they have working flash (QUAD3's is not detected) |
| The tracker's whole tick | not logged | A `tick_ms` column as the planner has (code, not offline) |
| Motive's share of the 7.5 | not separated | The sender on the Motive PC adds its frame time to the packet |
| RC smoothing's delay | estimate only | Betaflight's RC smoothing debug mode (an FC change) |

## 7. For the lab chat to carry

**Text in the draft that this changes** (the thesis chat's to edit, `system_overview.tex`):
- Line 151 (i): "25-35 against 48-62" is the 0.55 m pairs and a delayed-run subtraction. Same setup and path: 34-39
  against 83-100 (RIG-1029-f4, R2835).
- Line 151 (i): the throttle's 44-67 is three flights. Latest setup 56-65 (RIG-1029-h4, -h4f_3, -f4).
- Line 151 (ii): "simulated time waits for the planner" is not so. The plan arrives 22-39 simulated ms after its tick at
  4 carriers against 74.0 ms real.
- The review's "velocity estimate ~20 ms" should read 16-17 ms. The 28 Oct decision's "40 against 20" was run.csv's 50
  Hz sampling.

**Registry rows owed** (exploratory, not written here, `results/` was off limits):
- The stale ring pose at 4 carriers. Falsifier: a tick following an overrun reads a pose as fresh as any other.
- QUAD4's FC rate and thrust lags from blackbox. Falsifier: the reading moves with the step gain across flights.

Nothing here changes code, a default or a bar. Whether finding 1 needs a look before 4 Nov is Wesley's.

## 8. Scripts

The fits use the repo's `tools/loop_timing.py` and `tools/bench_latency.py` unchanged. The rest is one-off and sits in
`results/analysis/loop_timing_2026-10-31`:
`bbl_decode.py` (blackbox decoder, no dependency, 0 bad frames on all three logs, reproduces rcCommand to motors 0 ms),
`bench_reread.py`, `solve_stats.py`, `planner_pool.py`, `lt_run.py`, `fc_step.py`, `fc_thrust.py`, `fc_lags.py`,
`uplink_mocap2.py`, `vel_replay.py`, `pose_age.py`, `planner_age.py`, `planner_stale.py`, `plan_arrival2.py`,
`twin_lt.py`, `twin_rate_calc.py`. The scripts behind RIG-1030-h4n_bb were lost with their scratchpad, so copy these if
the table goes in the thesis. The idle bench's file is `results/analysis/solve_scaling_tuf_2026-10-22.json`.
