# Card: mocap velocity/rate over a minimum time window (sim emulator first; rig publisher P9 later)

Date: 2026-09-28. Type: estimator in the (sim) flight loop -> card + critic (working rules). Status: REWRITTEN v2 2026-09-28 (Wesley Q10: pursue now with the sim oracle test; 1-2 clock-0 runs approved); critic pass 2 NOT READY (text/logging fixes) -> v3 below applies them.

## Problem
The tracker's MPC starts every solve from the mocap state x0 = [p, q, v, w]. Both the sim emulator
(src/simulation_communication/simulation_communication/payload_mocap_emulator.py:130-174) and the rig publisher
(src/drone_communication/drone_communication/motion_capture_publisher_node.py:239-265) compute v and w by differencing
CONSECUTIVE poses over the ARRIVAL clock (node clock / time.time()). A pose that arrives late then early, or two poses in
one callback burst, gives a ratio error of 0.5-2x on that sample. M-kicks (registry): 62-70 % of M1 carrier kicks follow
an x0 body-rate jump at the same or previous tick (0.1-2.3 % of normal ticks); about half of those rates disagree with the
attitude path of the logged poses; both rise ~8x with the unthrottled clock (R0702/R0703) together with the kicks.

## Change (one variable, opt-in, default off)
Emulator param `diff_min_dt_s` (default 0 = today's consecutive differencing). When > 0: keep a short ring buffer of
(t_arrival, pose); difference the current pose against the newest buffered pose at least `diff_min_dt_s` older
(20 ms proposed: 10 samples at 500 Hz poses, ~2.4 at the emulator's ~100 Hz output), for both v and w (w from the relative
quaternion over that span). Lag added ~ half the window (~10 ms).

## Why this and not stamps
The bridged PoseArray has no stamp (R0703c); a gz stamped pose topic would be the clean fix but needs a bridge/world change.
A window keeps the arrival clock but shrinks the relative timing error ~ jitter/window instead of jitter/period.

## Experiment
- Arm A (baseline, exists): R0702 / R0703 (clock_hz 0 stress arm: 42 / 64 kick events).
- Arm B: the same config (partner_attached_orbit_clock0.yaml) with `diff_min_dt_s: 0.02` on the emulators, gyro default
  now on (note: baselines were pose-rate bridges; one extra B0 run at clock 0 on the current default without the window is
  needed as the true baseline, since the bridge change could itself move the kicks).
- Metric: unscheduled output-side kicks (M-kicks detector, scratchpad kicks/events3.py; |dthrottle| or |dyaw| >= 0.02 per
  tick, fleet-wide scheduled steps excluded), NOT carrier_rate_kicks (smoothing would lower that trivially).
- Falsifier: B's unscheduled kicks >= 0.7x B0's while rate-inconsistent ticks drop below 0.3 %  -> the x0 glitches are not
  the cause. Supported if kicks drop >= 2x with no new failure (M1 stage bars, detach/rejoin peaks within +1 deg).
- Runs: 2 exploratory (B0, B), then decide.

## Rig transfer (P9, not this card)
If supported, the same window goes into the rig publisher behind a param, decided after the R0b bag gives the real arrival
jitter.

## Critic
VERDICT: NOT READY (critic 2026-09-28, pasted by the loop). Hypothesis not refuted, card too broad and too weak.
- Noise: R0702 and R0703 are repeats of one arm (37 vs 61 unscheduled events; the card's 42/64 counted scheduled ones) -> a
  1.65x spread any one-run comparison must beat; both aborted mid-mission, so score per airborne drone-second.
- Sharper mechanism: rclpy's /clock subscription is depth-1 best-effort (always newest), the emulator's PoseArray queue is
  depth-10 FIFO (payload_mocap_emulator.py:72): after an emulator stall it pairs the OLDEST queued pose with the NEWEST clock,
  then the next fresh pose differences 20+ ms of motion over ~8 ms -> spike-and-return. A 20 ms window only attenuates it
  (~2.4x). The tracker's own pose queue is depth-5 FIFO (callback_manager_multi.py:37). The node-1 command (0.1 s ahead,
  re-pinned every 20 ms) amplifies any x0 change up to 5x the modelled slew.
- Clean causal arm: w in x0 from the header-stamped gyro (/drone_N/imu, bridged since the gyro default) as a sim oracle; or
  pose subscription depth 1 (targets the stale queue, no lag).
- Scope: w only, carrier drone streams only; never the payload stream (planner, network, magnet manager, Tejen's ring_bridge),
  not drone 3 (Tejen's MPC), not vz (kt_trim measure_accel, floor/descent logic = a takeoff/floor change); keep header.stamp.
- Before any run: free analyses from R0685-R0732 (2x2 kicks given glitch-no-stall vs stall-no-glitch; kicks split by the sign
  of the jumped rate component, since the node-0 box inverts only for negative states, controller_mpc.py:1183); freeze the
  detector in tools/; fly B0 first on the current default; two runs per arm; neighbours once each (canonical M1 at 500 Hz,
  one loaded 500 Hz CPU-hog run as R0709).
- Not asked for by Wesley; kicks are parked within every bar; clock-0 runs need his OK (CURRENT_STATE: never run M1 unthrottled).

## Free analyses (2026-09-28, registry M-kicks2)
Rate glitch favoured (78 vs 1.45 per 1000 ticks for glitch-only vs stall-only; 0.16 base); inverted node-0 box ruled out as a
main source. Detector: tools/kick_events.py. Revised next arm (if pursued): tracker x0 body rate from the stamped gyro on
the carriers (sim oracle) on the clock-0 arm; needs a rewritten card, critic, and Wesley's word (GOALS question 10).


## v2 (2026-09-28): gyro-oracle arm (Wesley Q10)
Hypothesis (M-kicks2): M1 carrier kicks are driven by glitches in the body rate w in the tracker's MPC initial state x0,
produced by the mocap emulator differencing consecutive unstamped poses on the arrival clock (stale-queue pairing after a
stall; critic 2). Free analyses: kicks per 1000 ticks 78 (glitch, no stall) vs 1.45 (stall, no glitch) vs 0.16 (neither).

Variable (one): tracker param `x0_rate_source` ('mocap' default | 'imu'). With 'imu' the tracker takes w for x0 from the
header-stamped simulated gyro /drone_<i>/imu (already bridged since the gyro default; body FLU rad/s, same frame as the
MotionCaptureState angular_velocity - verify the frame) instead of the mocap state; p, q, v unchanged. Sim-only oracle:
on the rig the same idea would need the flight controller's gyro telemetry, not in scope. Scope: the three CARRIERS only
(drones 0-2) via a per-drone launch arg; not drone 3 (the partner/newcomer), not the payload stream, not v (kt_trim,
floor/descent untouched). Stale-gyro guard: if no gyro sample within 50 ms, fall back to the mocap w and count it.

Arms (score with tools/kick_events.py, unscheduled kicks per 100 airborne drone-s; R0702/R0703 repeat spread 1.43x):
- B0 = configs/experiments/partner_attached_orbit_clock0.yaml on the current default (gyro bridges since 961eb03), x0 w
  from mocap (the true baseline; R0702/R0703 flew pose bridges). One run.
- B1 = the same with x0_rate_source imu on drones 0-2. One run (Wesley approved 1-2 clock-0 runs; B0 + B1 = 2).
Falsifier: B1's unscheduled kicks per 100 drone-s >= 0.7x B0's while B1's rate-inconsistent ticks fall below 0.3 %
-> the x0 rate glitches are not the cause. Supported: B1 <= 0.5x B0 AND rate-inconsistent ticks < 0.3 %. Between 0.5x and
0.7x = inconclusive (stop, time box). Both runs aborted mid-mission at clock 0 before (R0702/R0703): exposure is scored per
drone-second, stage bars only if reached.
Neighbour after a supported result (once, not a repeat): canonical M1 at 500 Hz with B1's setting vs R0724 (stage peaks,
kicks per drone-second).


## Critic pass 2 (2026-09-28, condensed; pasted by the loop)
NOT READY, text/logging fixes only: (1) keep the logged wx/wy/wz = mocap w so tools/kick_events.py reads the same thing in
both arms; append x0_wx/wy/wz (the w actually used) and a gyro-fallback flag as new last columns; (2) decide on the within-run
FINGERPRINT (do carrier kicks still follow mocap-w glitches once that w is no longer used?), not on the count ratio (1.43x
spread); (3) the support ratio if glitch-driven kicks vanished is only ~0.5x, inside v2's inconclusive band; (4) B0 count
floor (>= ~20 unscheduled carrier events) or stop after B0; (5) score drones 0-2 only; drone 3 as control; (6) v glitches
come with w glitches (same pose pair): split scoring by channel (throttle du2 vs yaw du3) and check vz on B0 via x0_v*;
(7) stale guard on the sim clock vs the IMU header stamp; hold the last gyro <= 50 ms, then fall back and count; VOID if
fallbacks > 1 % of carrier window ticks; use the MEAN of the gyro samples since the last tick; VOID if < ~60 airborne
carrier drone-s; confirm the carriers stay on the MPC path (control_phase never 'network'); (8) real:=true must refuse
x0_rate_source imu; delete the code after the test unless Wesley keeps it. Frame/sign checks all pass (both body FLU rad/s).

## v3 (2026-09-28): what is built and how it is decided
Build: tracker param `x0_rate_source` ('mocap' default | 'imu'); with 'imu' the MPC x0 body rate = mean of the /drone_i/imu
angular_velocity samples received since the previous tick (already subscribed at 1 kHz), stale guard on the sim clock vs the
IMU header stamp (hold last <= 50 ms, then mocap w, counted); log columns appended: x0_wx, x0_wy, x0_wz, x0_w_fallback;
wx/wy/wz stay mocap. Launch arg per drone on three_attach_launch (drones 0-2 only in B1); refused under real:=true.
Runs (Wesley's approved 2): B0 = partner_attached_orbit_clock0.yaml, current default; B1 = the same + x0_rate_source imu on
drones 0-2. Stop after B0 if it has < 20 unscheduled carrier events (report B0 as the finding: the gyro-default plant already
changed the kick rate).
Decision (carriers 0-2, unscheduled kicks, tools/kick_events.py; baseline share from R0702/R0703: 46-51 % of unscheduled
events follow an attitude-inconsistent mocap-w glitch, 70 % any mocap-w jump):
- PRIMARY fingerprint in B1: share of unscheduled carrier kicks preceded (k or k-1) by a rate_inconsistent mocap-w glitch.
  Supported if <= 20 % (towards the 1.5 % normal-tick rate) while B0 stays >= 40 %; refuted if B1 >= 40 % (kicks still follow
  a glitch the tracker no longer uses -> the glitch is a symptom of the shared pose pair / stall, not the channel);
  20-40 % inconclusive.
- Manipulation check: x0_w == gyro on >= 99 % of carrier window ticks; fallbacks <= 1 %; x0-w jumps on normal ticks <= B0's.
- Secondary: per-100-drone-s ratio B1/B0 (expected ~0.5x if supported), split by channel (throttle vs yaw); B0's throttle
  kicks vs x0_vz jumps (the v channel, untouched here).
- VOID if < 60 airborne carrier drone-s or if the carriers ever leave the MPC path.


## Result 2026-09-28 (B0 R0734, B1 R0735)
| Arm | x0 body rate | Carrier unscheduled kicks | per 100 drone-s (all) | throttle / yaw | share after a mocap-w glitch |
|---|---|---|---|---|---|
| B0 R0734 | mocap | 47 | 14.21 | 23 / 43 | 32 % |
| B1 R0735 | gyro (carriers) | 12 | 3.43 | 4 / 9 | 42 % (5/12) |
Manipulation check passed (fallback <= 0.08 %, swap live on 91-92 % of ticks, MPC path throughout, no abort).
Verdict: the count drops 0.26x (beyond the 1.43x repeat spread): the mocap-derived body rate in x0 is the main channel of
the carrier kicks. The written primary fingerprint rule reads 'refuted' (B1 42 % >= 40 %) but its precondition failed
(B0 32 % < 40 %) and n = 12; the residual kicks still coincide with the same pose-pair glitches, consistent with the
velocity channel (v is differenced from the same pose pair and was not changed). Exploratory; not a claim.
Rig transfer: the rig tracker has no gyro; the rig fix is the mocap rate/velocity estimator itself (P9, from the R0b bag).
Code: x0_rate_source (sim-only oracle) kept pending Wesley's word (working-rules default: delete).
