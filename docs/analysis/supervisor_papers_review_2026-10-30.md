# The supervisors' papers: what the thesis and the experiments should borrow (30 Oct 2026)

Five papers by Mitchell Torok and Mohammad Deghat (UNSW), read in full with their figures (rendered pages), against
the draft thesis (`~/thesis_full_draft`, the 30 Oct full-draft copy) and our measured facts. Wesley: "they are high
quality ones written by my supervisors so you can take inspiration, also the testing environment and quadcopter model
will be the same for the most part (we are using 4 Cinewhoops)".

Who reads this: the thesis chat (sections 3 and 4, the citations, the tables to add) and the lab chat (sections 1 and 2,
the repeats, the checks on the next lab days). Nothing here edits code, configs, results or the thesis.

## The five papers

| Tag | Reference | In one line |
|---|---|---|
| PEND | E. Paul, M. Torok, M. Deghat, "Balancing a Flying Inverted Pendulum with an Unknown Length Using Model Predictive Control and a Genetic Algorithm Estimator", arXiv:2608.17256 [eess.SY], 18 Aug 2026, 6 pp. | A linear MPC (acados, 120 Hz) balances a 1 m pendulum on a 3.5 in Cinewhoop, and a genetic algorithm finds the unknown length online in 4 to 15 s. |
| WIND | M. Torok, M. C. M. Chan, D. Sui, M. Deghat, "Online Wind Mapping for Coupled Path Planning and Contouring Control of Quadrotors", Sensors 26(16), 5250, 2026, doi:10.3390/s26165250, 15 pp. | A disturbance observer estimates wind from mocap, a map shares it between a contouring MPC (feedforward) and an A* planner, and a 2 x 2 ablation on a CineLog35 cuts tilt RMS by up to 38 %. |
| LSTM | M. Torok, M. Deghat, Y. Song, "Bearing-Only Tracking and Circumnavigation of a Fast Time-Varied Velocity Target Utilising an LSTM", 23rd European Control Conference (ECC), Thessaloniki, 2025, pp. 1815-1821. | An LSTM estimates a moving target's position and velocity from bearings, in simulation only, against two classical estimators. |
| GDAG | M. Torok, M. Deghat, Y. Song, "Greedy-DAgger - A Student Rollout Efficient Imitation Learning Algorithm", IEEE Robotics and Automation Letters 10(3), 2878-2885, March 2025, doi:10.1109/LRA.2025.3536297. | A small network learns to imitate a sampling MPC expert with up to ten times fewer student rollouts, then flies a Crazyflie zero-shot. |
| CEP | M. Torok, M. Deghat, Y. Song, J. Katupitiya, "Adaptive quadcopter model predictive control using remote monocular vision", Control Engineering Practice 177, 107235, 2026, doi:10.1016/j.conengprac.2026.107235, 11 pp. | Off-board MPC on four hobby quads (Cinewhoop among them) from FPV video alone, with an augmented UKF that learns the throttle gain and the flight controller's mounting offsets online. |

Page numbers below are the printed ones (WIND "p. 8 of 15", CEP "p. 8", GDAG and LSTM journal pages). PEND has no
printed numbers, so its pages count from 1.

## 1. The test environment and the quadcopter as they describe them

### 1.1 Side by side

| Item | PEND | WIND | CEP | GDAG / LSTM | Ours |
|---|---|---|---|---|---|
| Vehicle | 3.5 in Cinewhoop, 2004 1800 kV, 45 A F7 AIO, 6S 1300 mAh (s3, p4) | CineLog35 V2, 2004 1750 KV, 3.5 in ducted, F7 45A AIO, 6S 1350 mAh (Table 1, p8) | four platforms, Cinewhoop 3.5 in, 6S 1300 mAh (Fig. 2, p7) | Crazyflie (GDAG). LSTM is simulation only | four Cinewhoops. Frame, motors, props, FC and pack capacity are not in the compiled draft |
| Mass | 0.569 kg (Table 1, p4) | 459 g all-up with battery (Table 1) | 459 g (Fig. 2) | - | 0.475 kg airframe and pack, 0.55 kg with the 0.55 m cable and magnet, 0.583 kg with the 1 m cable (scaled, not weighed) |
| Firmware and mode | Betaflight, linear angular-rate response, max 100 deg/s (s3, Table 1) | Betaflight rate mode, linear rate curve, r_max 100 deg/s, tau_r 0.12 s (Table 2, p9) | Betaflight angle mode, alpha_max 55 deg, yaw linear capped at 100 deg/s, tau_a = tau_psi = 0.08 s identified from mocap (p8) | - | Betaflight rate mode. Tracker model: tau 0.12 s, centre and max rate 100 deg/s (`tracker_node.py:688`, `velocity_loop.py:39-41`), so linear like WIND |
| Command link | ExpressLRS, throttle and rate commands from a remote computer (s3) | ExpressLRS, voltage and current returned as telemetry (p7-8) | Jumper T-Lite ELRS module on a USB-serial adapter, a ROS 2 node writes the packets, 100 Hz to keep the link alive (p7, Fig. 3) | - | one ELRS transmitter per UAV on USB serial, 100 Hz, arm and magnet on AUX, voltage back. Radio delay 6.5 ms (RIG-1030-bench_radio) |
| Motion capture | ten OptiTrack cameras, 240 Hz, velocities differentiated and low-pass filtered (s3, p4) | 6 x 6 x 5 m^3 arena, ten OptiTrack PrimeX 13, Motive 2.0, 240 Hz (s3.1, p7, Fig. 2) | ground truth only: 21 trajectories, SLAM against mocap 8.0 +- 3.4 mm (p8) | OptiTrack with the Crazyflie | 6 x 6 x 4 m cage, 240 Hz, a forwarder of ~2,250 packets/s, at most 8 bodies. One body 7.5 ms, ten bodies 95 ms (RIG-1030-bench_mocap). Velocity: slope of the last 4 frames, ~20 ms behind |
| Ground computer | laptop, "Intel Core i7-240H x 16 CPU", 32 GB (p5) | i9-13900H, 32 GB, Ubuntu 22.04, ROS 2 (p7) | i9-13900H, 32 GB, Ubuntu 22.04, ROS 2 Humble, CPU only (p7-8) | i7-10700 local, remote RTX 4080 (GDAG p2883) | this laptop: Intel Core 5 210H, 12 threads, ~16 GB, Ubuntu 22.04.5, ROS 2 Humble from source (if it is the flight laptop, as the working rules imply). Not stated in the draft |
| Controller and rates | linearised MPC, RK4, acados, partial-condensing HPIPM, 120 Hz, N = 60 over 6 s, no terminal constraint (s2.2, p3, Table 2). Altitude and yaw by PID | MPCC, RTI warm start, 30 Hz, N = 20, dt 0.1 s (Table 2) | NMPC, acados, 15 Hz, N = 15 over 3 s, solve 4.65 +- 2.38 ms (p8, Table 1) | MPPI expert 83.3 +- 2.97 ms per action on CPU, student 0.17 +- 0.06 ms (GDAG p2883) | planner 10 Hz, N = 20 over 2 s, up to 5 SQP iterations, 60 ms budget. Tracker 50 Hz, N = 20 over 2 s, RTI. Commands 100 Hz |
| Thrust model | not stated | proportional, k_T = 24.0 m/s^2 identified offline, linear drag 0.45, 0.45, 0.5 s^-1 (Table 2) | proportional k_t estimated online by a UKF. Thrust tests: k_t,max ~27 m/s^2 at full charge, ~23 m/s^2 at 3.5 V per cell (p9, Fig. 2) | - | affine: u = 0.185 + 0.507 M - 0.022 (V - 23.5), 35.2 m/s^2 above the offset at 0.55 kg, cap 0.8. Drag 0 in the tracker |
| Delays | not stated | not stated | ~200 ms video feedback delay, about three control intervals, measured with high-speed photography. Forward-propagation compensation tried and dropped (p8) | GDAG: a remote expert that cannot fly the vehicle directly because of latency (p2879) | radio 6.5 ms, mocap 7.5 ms (one body) to 95 ms (ten), velocity estimate ~20 ms, throttle to lift 44-67 ms (simulation 17-29), rate command 25-35 ms (simulation 48-62) |
| Battery protocol | not stated | every trial on a fresh 6S at 4.0 V per cell, 24.0 V (p8) | conditions interleaved to cancel battery depletion, cell voltage read after the trials (p9) | - | preflight check above 23.6 V (uncompiled `hardware.tex`). The map has a voltage term |
| Safety | - | - | disarm if no pose arrives within a set time (p8) | - | mocap timeout 0.25 s, fleet LAND at +-1.75 m |

### 1.2 Numbers that should match ours, and the ones that do not

| # | Quantity | Theirs | Ours | Verdict |
|---|---|---|---|---|
| 1 | Thrust per unit throttle at hover (secant through zero) | WIND: 24 m/s^2 x 0.459 kg = 11.0 N. CEP: 23-27 m/s^2 x 0.459 = 10.6-12.4 N | 0.55 kg x 9.81 / 0.464 = 11.6 N (21.1 m/s^2 at 0.55 kg, 23.5 V). At 0.583 kg, 11.9 N | Match within 5-10 %. Same motors and props class |
| 2 | Effect of pack voltage | CEP: k_t ~27 to ~23 m/s^2 from 4.0 to 3.5 V per cell, -15 % over 3 V on 6S (p9) | 0.022 throttle per V, so 3 V costs 0.066 of a 0.464 hover throttle, 14 % | Match. Our voltage term is the same physics |
| 3 | Thrust at full throttle | CEP thrust test: ~12.4 N for the 459 g Cinewhoop (Fig. 2) | the affine map extrapolated to u = 1: 19.36 N x 0.815 = 15.8 N | Disagree by ~27 %. Our map was fitted only between u = 0.46 and 0.56 (hung masses 0-196 g). See 1.3 |
| 4 | Rate-loop model | WIND: tau_r 0.12 s, r_max 100 deg/s, linear | tau 0.12 s, 100 / 100 deg/s | Identical. Say where ours came from (identified, or WIND's value) |
| 5 | All-up mass | 459 g (WIND, CEP), 0.569 kg with the pendulum seat (PEND) | 0.475 kg airframe and pack | 16 g heavier than WIND's. Name what it is (marker mounts, magnet board) |
| 6 | Arena | 6 x 6 x 5 m^3 (WIND p7) | 6 x 6 x 4 m (draft `system_overview.tex`, working rules) | Contradiction. Confirm the height |
| 7 | Cameras | ten (PEND), ten PrimeX 13 and Motive 2.0 (WIND) | count, model and Motive version not stated | State them |
| 8 | Ground computer | i9-13900H, 32 GB | Core 5 210H, ~16 GB | Ours is slower. Every solve time (and the 3-4 carrier real-time limit) is on it. State it beside the tick table |
| 9 | Command rate | 100 Hz, needed to keep the ELRS link (CEP p7) | 100 Hz | Match. Borrow CEP's reason sentence |
| 10 | Velocity estimate | differentiated and low-pass filtered (PEND) | least-squares slope over 4 frames | Same kind. Already stated |
| 11 | Lateral drag | 0.45 s^-1 (WIND) | 0 in the tracker | Fine at 0.2 m/s (0.09 m/s^2), but the measured pull then absorbs it. One sentence |
| 12 | Delay tolerated | CEP flew with ~200 ms at 15 Hz on slow, smooth paths | 95 ms of mocap delay heaved every ring lift on 30 Oct | Contrast worth a sentence: a load on cables and the measured-pull loop make our system far more delay-sensitive than one free vehicle |

### 1.3 What this means for the real world (computed for this review)

Thrust budget from our map, cables at the planned 60 deg elevation, the 1.15 kg ring shared evenly, the cable's mass in
the UAV's 0.583 kg. A computation, not a measurement.

| Carriers | Thrust per UAV | Throttle at 23.5 V (3.92 V/cell) | at 22.5 V (3.75 V/cell) | Cap 0.8 |
|---|---|---|---|---|
| 2 (opposite) | 11.8 N | 0.796 | 0.818 | at or over the cap: matches "two carriers cannot lift the 1.15 kg ring" |
| 3 (even triangle) | 9.7 N | 0.688 | 0.710 | 0.09-0.11 margin by the map |
| 4 (square) | 8.7 N | 0.634 | 0.656 | 0.14-0.17 margin |

A quick read of three flights' tracker logs (`thr_out`, `battery_v`, `acm_z` in `results/rig/2026-10-2*/<name>_logs`,
steady window with the ring up, median per carrier) says the map is optimistic at these throttles. Exploratory, one
pass, not reviewed:

| Flight | Carriers | Median throttle | Pack | Summed vertical measured pull | Ring weight | Excess |
|---|---|---|---|---|---|---|
| RIG-1029-sq2_5 (triangle hover, notch, throttle_max 0.95 typed) | 3 | 0.73-0.76 (p95 up to 0.83) | 22.4-22.6 V | 14.3 N | 11.3 N | +27 % (~1.0 N per carrier) |
| RIG-1029-f4 (figure-eight) | 4 | 0.60-0.70 | 22.3-22.8 V | 12.3 N | 11.3 N | +9 % |
| RIG-1028-hS2 (pair hover, 0.86 kg ring, 3 s window) | 2 | 0.76-0.78 | 22.2-22.5 V | 9.8 N | 8.4 N | +16 % |

In steady hover the carriers' vertical pulls should add up to the ring's weight. They read 9-27 % more. The measured pull
is the residual of acceleration minus the map's thrust, so a map that over-predicts thrust at high throttle shows up
exactly like this, and the tracker then adds throttle (0.73-0.76 flown against the map's 0.71). CEP's thrust test (row 3
above) points the same way. Either the map is ~0.3-1.0 N per carrier optimistic between u = 0.6 and 0.8, or the pull
estimate is biased. The triangle on the 1.15 kg ring then has less margin than the map says, and pack voltage at takeoff
decides it.

## 2. Methods we could use or should cite

| Method | Paper | What it is | Relevance to us | Before 20 Nov? |
|---|---|---|---|---|
| Nonlinear disturbance observer on the translational dynamics | WIND s2.2, eqs. 7-8, p5. Gain l = 5 per axis (Table 2). Validated against an anemometer: bias +0.003 m/s, RMSE 0.72 m/s (Table 3, p10) | estimates an external force from mocap velocity and the commanded thrust without differentiating velocity | our measured pull is the same residual (acceleration minus modelled thrust, gravity, drag), formed by differentiating (80 ms slope, 50 ms low-pass, 5.3 Hz notch). An NDOB is that residual through l/(s+l) | **Cite** and name the measured pull a disturbance observer. **Do not** swap it into the flight loop: a new estimator needs the critic, and l = 5 is ~200 ms of lag, the thing the 1.1 Hz slide taught us to avoid. Optional offline replay on logs for the discussion |
| Estimator validated against ground truth | WIND s3.2 and Table 3 (bias, SD, RMSE per trial). CEP p8 (SLAM against mocap, 21 trajectories) | the estimate the controller relies on is checked against an independent measurement | the measured pull has no such check in the thesis. The ring's weight is a free ground truth in every steady hover (section 1.3) | **Do now**, offline from existing logs: per flight, summed vertical pull against the ring's weight, bias, SD, RMSE. Half a day, lab chat |
| Augmented-state UKF for throttle gain and mounting offsets | CEP s3, eqs. 15-26, pp5-6. Converges from k_t,0 in {20, 24, 28, 32} (Fig. 5, p7). Offsets cut tracking RMSE from 0.54 to 0.30-0.34 m (Table 3, p9) | joint state and parameter estimation feeding an adaptive MPC | CEP attributes every residual to static parameters and assumes no external force (p2, p5). On our carriers the cable's pull is a large, time-varying external force on the same residual, so the throttle gain and the pull are not separately observable from position. Our own adaptive UKF was deleted earlier by decision. Tejen's QUAD4 controller has a thrust-ratio UKF (`src/tejen_mpc/.../thrust_ratio_full_model_ukf.py`), the CEP pattern on a free vehicle | **Cite** with that reason in the tracker section. Nothing to build |
| Genetic-algorithm estimator of an unknown length | PEND s2.3, pp3-4. Sliding window of 55 samples, KF innovation as fitness, 0.2 Hz, converges in 4-15 s (Table 3, p6). An estimate that jumps by more than 0.8 m made the MPC unstable (p6) | online estimation of a pendulum parameter on the same airframe | our cable lengths are measured at the hand-over and the ring is weighed. An unknown object in the basket goes to the model integral. PEND's warning (a model parameter changed in one step unsettles the MPC, change it gradually) supports our 10 s fold-in blend and the slow, bounded integral | **Cite** for both points. Nothing to build |
| Delay compensation by forward propagation | CEP p8 | propagate the delayed state through the applied commands | CEP dropped it: the noise it added to the MPC's initial state cost more than the 200 ms it removed | **Cite**, do not build. Remove the delay at its source (at most 8 bodies, or Motive at 200 Hz if a flight needs 9) |
| MPC formulation details | PEND p3 (linearised model, no terminal constraint to keep feasibility under disturbance and parameter jumps). CEP p6 (input increments penalised, recursive feasibility because only inputs are constrained, stability "not formally established" said plainly). WIND eq. 13-14, p7 (contour error and lag error, per-stage feedforward of a disturbance along the horizon, soft tilt bound) | - | our tracker's stick states with their rates as inputs is CEP's increment form. Our planned pull along the horizon is WIND's per-stage feedforward. Our "clock-free error" and "lag" are WIND's contour and lag errors. CEP's honest stability paragraph is the model for ours | **Cite and borrow the wording**: "contour error" and "lag error", one paragraph on what is guaranteed (feasibility from softened bounds) and what is not (stability) |
| Learned speed-up of an MPC | GDAG: a 2 x 32 network trained against an MPC expert, expert rollouts in parallel on a remote GPU (100 x 250 steps in 512 ms), student inference 0.17 ms against 83 ms for the expert. Expert-level control within 8 s of wall time. Zero-shot on a Crazyflie: RMSE 0.63 / 0.48 against 0.23 / 0.15 in simulation (p2884) | replaces an expensive controller by a cheap learned student | our planner's p95 tick is 256 ms at eight carriers (idle bench, 5 iterations) and the real-time limit is 3-4 carriers on this laptop. A student of the planner could fly eight. But its output is a full horizon of 13 + 14N states that changes with N, the reconfiguration transients are the off-distribution states, and there is no training pipeline | **Cite only**, as the future work for eight or nine carriers in real time. Not before 20 Nov |
| Wind map, wind-aware A* | WIND s2.2-2.3 | spatial memory of a disturbance, routing around it | indoor, no wind | **Not relevant** beyond the observer |
| LSTM bearing-only estimator | LSTM s2 | learned estimator of a moving target | none for the method | **Not relevant**. Borrow its evaluation design only (section 3.3) |
| 2 x 2 ablation | WIND s3.3, Table 4, p13 | each component on and off, and both | X1 is a one-variable ladder (ours, no resize, late 0.5/1/2 s, planned pull) with its bars frozen. A fourth cell (no resize with the planned pull) would make the resize and the measured pull a 2 x 2 | **Present X1 in WIND's table format.** The extra cell is a new arm, not a re-score, and needs Wesley's word |

## 3. How they write and present

### 3.1 Structure and length

| Paper | Venue | Pages | Figures | Tables | Sections |
|---|---|---|---|---|---|
| PEND | arXiv (IFAC-style) | 6 | 6 | 3 | Introduction (MPC, estimation, GA, contributions), Dynamics and control, Experimental setup, Experimental results, Conclusion |
| WIND | Sensors (MDPI) | 15 | 7 | 4 | Introduction with 3 contribution bullets carrying the headline numbers, Methodology, Experiments (setup, estimator validation, ablation), Conclusions and future work |
| LSTM | ECC | 7 | 12 | 3 | Introduction, Methodology, Evaluation (approach, then one subsection per scenario), Conclusion |
| GDAG | RA-L | 8 | 5 + 1 algorithm | 3 | Introduction, Methodology, Experiments (setup, student, expert, one subsection per environment, hardware), Conclusion |
| CEP | Control Eng. Practice | 11 | 8 | 4 | Introduction, Related work (italic run-in topics), Methodology, Implementation, Experiments, Conclusions, Limitations and future work |

The pattern: problem, prior work grouped by theme, the gap, contributions as 3-4 bullets, the method, then a separate
implementation or setup section, then one subsection per experiment, then a conclusion and a separate limitations
section (CEP s7, p10).

### 3.2 How the experimental setup reads

WIND s3.1 (p7-8) is the model to copy. In order: the arena and its size, the cameras and their software, what the pose
is used for, the platform with a photo (Fig. 3) and a parameter table (Table 1: frame, mass, motors, props, FC,
firmware, link, battery), the computer and OS, the link and its telemetry, the battery protocol, the disturbance source,
and the one restriction chosen (a fixed-height plane) with its reason. Then a second table of identified model
parameters and settings by subsystem, with symbol and value (Table 2, p9). Each number appears once.

CEP s4 (pp6-8) adds three things worth copying: a photo of the ground-station hardware (Fig. 3), a timing table of each
component (mean and SD, Table 1), and a parameter table with a "Determination" column saying how each value is known
(offline calibration, frame specification, configured, identified offline, recovered at start-up, estimated online,
Table 2). It also states the measured end-to-end delay, how it was measured, and what was tried against it (p8).

PEND s3 (p4) is the short form: one paragraph, two tables, one photo.

### 3.3 How results are reported

- Each experiment opens with one sentence of purpose, then the protocol (trials, conditions, order), the metric and its
  window, a table, one figure of a representative trial, and one paragraph of reading.
- Repeats and spread: CEP three trials per cell, mean +- SD (Tables 3-4, pp9-10). WIND five trials per condition in the
  main configuration and three in the replications, mean +- 1 SD pooled over legs (Table 4). GDAG ten complete training
  cycles, mean +- SD (Tables I-III). LSTM fifteen evenly spaced trials per sweep and 1000 random trials shown as
  histograms (Figs. 4, 6, 8, Tables II-III).
- Windows are stated: CEP's RMSE starts 6 s into the flight (p9), LSTM averages the last 5 s after a 10-15 s settle
  (Fig. 4, 6 captions).
- The best value per metric in bold (WIND Table 4, CEP Table 3, GDAG Tables I-III). A reference row (GDAG's "Expert").
- Percent change against the named baseline, in the abstract and the conclusion ("tilt RMS by up to 38 % and its 95th
  percentile by 28 %", WIND).
- Robust statistics for attitude: tilt RMS and tilt p95 (WIND), never only the peak. Effort costs reported beside the
  gain (path length, energy per round trip from battery telemetry, WIND Table 4).
- Battery control: fresh packs at one voltage (WIND p8), or interleaved arms with the voltage read after (CEP p9).
- Estimators validated against ground truth before they are used (WIND s3.2, CEP s5).
- Failures kept and explained: "The flight is retained in the statistics as a genuine failure mode" (WIND p11). A
  baseline that could not be made to work is named and why (GDAG p2881, LSTM p1817).
- Fairness stated: same constants for all methods, implemented per their papers, run concurrently (LSTM p1817, GDAG
  p2881).

### 3.4 Figures

| Feature | Where | Note for us |
|---|---|---|
| Full box, light grid, legend below the axes and framed | CEP Figs. 5, 7, 8. LSTM Figs. 3-6. GDAG Fig. 3. WIND Figs. 6-7 (unframed) | already Wesley's style of 30 Oct |
| Subplots with a bold title "(a) Altitude Tracking Performance" | CEP Figs. 5, 7, 8 | we use "(a)" only. Either is fine, be consistent |
| One colour per condition, reference dashed or grey | all | as ours |
| Estimate converging over time beside the tracking it improves | CEP Fig. 5 (k_t), Fig. 7 (offsets), Fig. 8c. PEND Figs. 5-6 | the model integral d over time beside the ring's height |
| Error against a swept parameter, one line per method | LSTM Figs. 4, 6, 11 | the tilt rise against the survivors' gap (F4), the catch offset sweep |
| All conditions overlaid on one map | WIND Fig. 6, Fig. 7 (coloured by leg index) | X1 arms on one tilt plot. Rejoins coloured by order |
| Multi-exposure composite photo of a flight | WIND Fig. 1, CEP Fig. 4, PEND Fig. 1 | three of five papers open with one. We have none |
| Trajectory traced on a photo of the room | CEP Fig. 6 | optional |
| Platform photos on a plain background with key numbers under each | CEP Fig. 2. Single vehicle with markers: WIND Fig. 3, PEND Fig. 2 | a single UAV with its markers, cable mount and magnet |
| Ground-station hardware photo | CEP Fig. 3 | the four ELRS modules and the laptop |
| System diagram with the rate in each block, "Ground station" and "Flight arena" boxes | WIND Fig. 2, CEP Fig. 1 | our `fig:owners` has some rates. Add mocap 240 Hz, radio 100 Hz and the two boxes |
| What to avoid | PEND: MATLAB figures with very small fonts, and two text-table mismatches (14.48 s in the text against 14.68 s in Table 3, pp5-6, a 0.8 m circle in s4.2 against 0.4 m in s4.3) | our one-source tables (generated) already guard this |

### 3.5 Notation, captions, prose

- Notation (CEP s3, WIND s2.1): x = [p q v w] in R^13, u = [u_T u_phi u_theta u_psi] in R^4 normalised (u_T in [0, 1],
  the rest in [-1, 1]), frames W and B, R(q) from body to world, k_T the throttle gain in m/s^2, tau_r and r_max for the
  rate loop, f_RK4 for the discretised model, the attitude error 2 vec(q_ref (x) q*) (CEP eq. 28, the same as our e_R).
  Every vector is given with its dimension. Ours is close (we inherited much of it). Our stick vector
  c = [c_phi, c_theta, u_T, c_psi] orders the throttle third, theirs first.
- No nomenclature lists (journal format). The thesis template needs one, so no conflict.
- Captions: figure captions say what each panel shows and what to notice (CEP Fig. 7: "as the angle offsets converge the
  tracking performance increases"). Table captions say n, the statistic and what bold means (WIND Table 4, CEP Tables
  3-4, GDAG Tables I-III). Wesley prefers short captions. Push-back: keep them short, but put n and the statistic in
  every table caption ("mean +- SD over three flights"). The papers never leave a reader guessing what a cell is.
- Prose: short declarative sentences, numbers with units, a closing sentence per experiment ("This experiment validates
  the system's ability to ...", CEP p9). They do not tag gaps inside results sentences.
- Front matter: a video link (PEND footnote, CEP video abstract) and a code and data statement (CEP footnote, WIND data
  availability).

## 4. Gaps between the draft and the papers, section by section

The draft read: `main.tex` as compiled on 30 Oct (Introduction, Literature review with `contributions_intro`,
Methodology with `system_overview`, `model`, `controller`, `reconfiguration`, `simulation`, `test_cases`, `metrics`,
`sim_studies`, the Results draft `drafts_1030/results_discussion.tex`, a blank conclusion, the appendices).

| Section (file) | What the draft has | What the papers do | Change |
|---|---|---|---|
| References (`references*.bib`) | none of the five papers, no Torok or Deghat entry | - | Add WIND, CEP, PEND, GDAG. LSTM only if the review mentions learned estimators |
| Literature review, execution layer (`lit_execution.tex`) | "The vehicles of this thesis run unmodified Betaflight ... Nothing can be added onboard, so the execution layer is a tracking MPC per vehicle on the ground" with no precedent cited | CEP, WIND and PEND fly exactly this: off-board MPC to stock Betaflight over ELRS from mocap or video, in the same lab | Cite them as the single-vehicle base. Cite WIND's observer beside Tagliabue's for force estimation |
| Gaps (`lit_gap.tex`, G4) | "No execution layer reviewed serves closed flight controllers" | - | True only because "execution layer" means a carrying team's. Say so: the off-board MPC on closed flight controllers is the lab's (CEP, WIND), and this thesis extends it to a cable's pull, planned or measured |
| Platform (`system_overview.tex` 2.1.1) | the mocap stream limit, the two setups table, the ring and magnets, photos of the team and the ring, the owners figure | WIND Table 1, CEP Table 1 and 2, arena, cameras, computer, battery protocol, single-vehicle photo | Add a platform table (frame, motors and KV, props, FC, firmware and rate profile, ELRS module and packet rate, pack, all-up mass). State the cameras (count, model, Motive version), the arena (4 or 5 m), the ground computer, the pack voltage rule. Add a single-UAV photo and a ground-station photo. Keep the stream limit as a setup fact (decided 30 Oct), but tell the 95 ms day once, in the results, where it is already told a second time |
| Timing | delays scattered: radio not stated, mocap 7.5/95 ms in 2.1.1, loop latency in 2.1.2, solve times in results | CEP Table 1 and the delay paragraph (p8) | One timing table: mocap delivery, velocity estimate lag, planner solve (median, p95), tracker solve, radio, throttle to lift, rate command to rate. Each with how it was measured |
| Tracker and thrust map (`controller.tex` 2.3) | "Thrust is affine in throttle ... identified on the hardware", no numbers in the text (they are in the appendix table and the uncompiled `hardware.tex`, where M reads "the mass carried" though it is the total mass held: 0.507 = 9.81 / (35.2 x 0.55)). k_T means the gain above the offset, unlike WIND's and CEP's k_T. tau_omega and the rate curve not given | WIND Table 2 lists k_T, drag, tau_r, r_max. CEP explains k_t as the acceleration at full throttle | State the map in N per unit throttle (19.4 N above u_0 = 0.185 at 23.5 V), give the hover secant (21.1 m/s^2 at 0.55 kg) beside WIND's 24 m/s^2 and CEP's 23-27 m/s^2, and the voltage term beside CEP's -15 %. Give tau = 0.12 s and 100 deg/s with their source. Add a CEP-style "Determination" table (thrust map identified offline from hung masses, tau from WIND or identified, rate curve configured, cable length measured at the hand-over, ring mass weighed, inertia estimated as an annulus, d estimated online, measured pull estimated online). Rename symbols to theirs where cheap (u_T, u_phi, u_theta, u_psi, tau_r, r_max, W, B) |
| Measured pull | described in detail, no validation | WIND validates its observer against an anemometer before using it | Add the validation (section 1.3, done properly by the lab chat) and call the measured pull a disturbance observer (WIND) |
| Model integral (`controller.tex` 2.3.2) | honest: "This assumes stability but does not give it" | CEP p6 says the same thing about its loop and adds why feasibility holds | Keep. Add the feasibility sentence (thrust bounds softened with slack) in CEP's form |
| Simulation (`system_overview.tex` 2.1.2) | seven omitted effects listed | CEP's limitations on its own model (p5) | Fine. Point to the timing table once it exists |
| Protocol and metrics (`simulation.tex`, `metrics.tex`) | frozen bars, cards, the lever-pair rule: stricter than any of the papers | mean +- SD, tilt RMS and p95 | State the spread rule: mean +- 1 SD for n >= 3, the range for n = 2, "(1)" for one run. Add tilt RMS and p95 beside the peak. Rename "clock-free error" and "lag" to contour error and lag error (WIND eq. 13) or say they are the same |
| Results, fixed team (`results_discussion.tex` 4.1) | the real-world earlier-setup cells are one flight each in hover (Table `tab:res_hover`) and mostly one in the figure-eight. Means without spread. The later setup's f4 and R2835 use two different path metrics (noted in a comment) | CEP and WIND: three to five trials, mean +- SD, bold best | Three flights per real-world baseline cell on 4-12 Nov, arms interleaved, fresh packs. One metric per column. Bold the best |
| Results prose | "Against G4, the ground tracking MPC for closed flight controllers (G4) kept ..." (line 193), "For G2 this is the leave (G2) is shown ..." (line 302) | one closing sentence per experiment, no gap tags | Fix those two sentences. Keep the gap mapping in `tab:res_matrix` and close each subsection in plain words |
| Results, losing carriers (4.2) | one real-world flight per row (`tab:res_release`). X1 pending | WIND's ablation table with every arm as a column | X1 in WIND's format: rows the metrics, columns the arms, mean +- SD, best in bold, the percent change against no resize in the text |
| Results, composite and video | none | WIND Fig. 1, CEP Fig. 4, PEND Fig. 1, video links | A multi-exposure composite of a real-world release or join (a fixed camera on the next lab days) and a video link |
| Results, solve time (4.4) | the tick table on "the idle ground computer" | CEP names the CPU beside its timings | Name the CPU in the caption or the text. Note that the supervisors' laptop (i9-13900H) is faster |
| Conclusion (`Conclusion_blank.tex`) | blank | CEP s6 and a separate s7 "Limitations and future work" | Write both. Future work: a learned student of the planner (GDAG) for eight or nine carriers, online parameter estimation where no external force competes (CEP, PEND), an observer form of the measured pull (WIND), delay compensation only if the stream cannot be made fast (CEP) |
| Appendix (`Appendices.tex`) | "Simulation and Rig Parameters" | - | "rig" is against the thesis wording rule. Merge into the determination table if it moves to the methods |
| Length | methodology ~21 pp against the school's 10-15 (`drafts_1030/index.md`) | setup sections of one to two pages | The platform section reads as a history (what changed when). Keep the setups table, move dated detail to an appendix |

## Borrow now (ranked)

| Rank | What | Why | Where | Effort |
|---|---|---|---|---|
| 1 | Cite WIND, CEP, PEND (and GDAG in future work), and reword G4 as an extension of the lab's off-board MPC on stock Betaflight | the thesis cites none of its supervisors' papers, and G4 reads as if their architecture were new | `references.bib`, `lit_execution.tex`, `lit_gap.tex` (G4), `controller.tex` (tracker model, measured pull) | 1-2 h, thesis chat |
| 2 | Check the thrust margin before the next lab day: the triangle's real throttle and the summed measured pull against the ring's weight (section 1.3), then decide whether the 1.15 kg ring flies on three only with fresh packs, or as four | the quick read says the map over-predicts thrust by up to ~1 N per carrier at u 0.6-0.8, so the triangle sits nearer its cap than the map says, and the packs sag 1 V in a flight | lab chat: an exploratory registry row per flight read, the rig sheet's pack rule | 1-2 h offline |
| 3 | Validate the measured pull against ground truth: per steady hover, summed vertical pull against the ring's weight (bias, SD, RMSE), both setups, beside the planned pull | the controller's key estimate has no independent check, and WIND shows the form (Table 3) | lab chat analysis, then a short table in the methods or the fixed-team results | half a day, existing logs |
| 4 | Three real-world flights per baseline cell on 4-12 Nov, arms interleaved, every pack started at a stated voltage, the start voltage in the registry. Report mean +- 1 SD | most real-world cells are one flight. The papers never report a hardware cell from one trial | rig sheets, `metrics.tex` (spread rule), every results table | lab time, 1 h of table edits |
| 5 | State the thrust map in their terms: 19.4 N per unit throttle above 0.185, the hover secant 21.1 m/s^2 against k_T 24 (WIND) and 23-27 (CEP), the voltage term against CEP's -15 %, and the thrust budget per team size | a supervisor reading "k_T = 35.2 m/s^2" will compare it with his 24 and think one is wrong. The budget explains why a pair cannot lift the 1.15 kg ring | `controller.tex` 2.3.1, `tab:setups` | 1 h, numbers in section 1 |
| 6 | Platform table, determination table, timing table, and the missing facts (cameras, Motive version, arena height, ground computer, pack capacity, frame and motors) | WIND Tables 1-2 and CEP Tables 1-2 are what their readers expect. Wesley to confirm the hardware facts | `system_overview.tex` 2.1.1, `controller.tex` | 2-3 h, plus Wesley's answers |
| 7 | Tilt RMS and p95 (and command-rate RMS) as reported metrics beside the peak | the peak is one sample. WIND's headline is RMS and p95. Descriptive only: no frozen bar is re-scored | scorers (`tools/`, lab chat), results tables | 2-3 h |
| 8 | A multi-exposure composite photo of a real-world release or join and a video link | three of the five papers open with one. It is the single most readable figure of a reconfiguration | a fixed tripod camera on the 4-12 Nov lab days (add to the rig sheet), composited after | 30 min on the day, 2-3 h after |
| 9 | Photos: one UAV with markers, cable mount and magnet, and the ground station with its four ELRS modules | WIND Fig. 3, PEND Fig. 2, CEP Fig. 3 | 2.1.1 | 30 min on a lab day |
| 10 | Conclusion with a separate "Limitations and future work" | CEP s7. The draft's conclusion is blank | `Conclusion_blank.tex` | 1-2 h |
| 11 | X1 in WIND's ablation-table format (arms as columns, mean +- SD, best in bold, percent change against no resize) | the headline result should read like the supervisors' headline result | Results 4.2, after the runs | 1 h. The fourth 2 x 2 cell only with Wesley's word |
| 12 | Wording and symbols: contour error and lag error, u_T u_phi u_theta u_psi, tau_r, r_max, frames W and B | the thesis then reads in the lab's language | `metrics.tex`, `controller.tex`, nomenclature | 1 h |
| 13 | System diagram in WIND Fig. 2's form: rates in every block, "Ground computer" and "Flight area" boxes | the reader sees the loop and its rates at once | `fig:owners` in `system_overview.tex` | 1 h |

Only worth citing, not doing before 20 Nov: the online UKF of CEP (with the reason above), the genetic-algorithm
estimator of PEND, an NDOB in place of the measured pull (WIND), delay compensation (CEP), a learned planner (GDAG), the
wind map and planner (WIND), the LSTM estimator.

### For Wesley to confirm

1. The arena height: 6 x 6 x 5 m (WIND p7) or 6 x 6 x 4 m (our draft and the working rules).
2. The UAV build: frame (CineLog35 V2?), motors (2004 at 1750 or 1800 KV?), props, FC, pack capacity (1300 or 1350 mAh),
   and what the 16 g over WIND's 459 g is.
3. The cameras (ten PrimeX 13?) and the Motive version, and whether the ground computer of every flight is this laptop.
4. Where tau = 0.12 s and the 100 deg/s rate curve came from: identified on our quads, or taken from WIND's values.
5. Whether to ask Mitchell how his pose stream reaches his laptop (his papers stream one or two bodies, so the forwarder
   limit never bit him). He may know a faster path than the forwarder.

**Wesley's answers (30 Oct 16:52):** 1. the arena is 6 x 6 x 5 m. 2. QUAD1-2 CineLog35 V2, QUAD3-4 CineLog35 V3, 1350 mAh
packs, masses as measured (the packs are newer). 3. Ten PrimeX 13; the Motive version from the paper or omitted; every
real flight from this laptop. 4. tau and the rate curve: not sourced in the thesis. 5. Not asking about the stream path.
