# Card: tracker command timing and mocap velocity lag (14 Oct 2026, rev 2 after the critic)

Owner Wesley. Tree: week5 00e7d61 + the 14 Oct uncommitted changes (see the session state). Origin: Tejen's review of
his M2 oscillations, checked against our side (docs/analysis/rig_load_2026-10-14.md, last section). Wesley asked for
the code and a critic (14 Oct ~15:05).

## Hypothesis

Two timing and model inconsistencies cut the vertical damping margin of our carriers. They show up only when something
else makes the loop late, as the overloaded laptop did in RIG-1014-t5_2.

**Status: a margin hypothesis, not a shown cause.** liftc_3 flew clean under the combined stack with neither change
(throttle sd 0.025-0.039, 50 Hz). Slew above the model's bound appears only inside the t5_2 bob, so on the rig it is a
symptom, not yet a cause.

**A: command timing.**
- The tracker sends `ocp.get(1, "x")[-4:]`. On the uniform 20-node, 2.0 s grid (acados.py:57-58) that is node 1,
  0.1 s ahead, sent every 20 ms tick. Node 0 is pinned to the sent value (`x0_relax_frac` 0).
- So the throttle may move 0.05 per tick, 2.5/s, against the model's 0.5/s (acados.py:140-141, index 2). The same
  holds for roll, pitch and yaw.
- Rig throttle slew, model units (radio units are 2x), % of ticks above 0.5/s: lift3_3 0-2 %, liftc_3 0-1 %, the t5_2
  bob 19-34 % (p95 0.92-1.14/s, max 2.2/s).
- The direction is not certain. The 5x rate may also offset unmodelled latency: ~20-25 ms from command to body rate,
  plus the mocap lag.

**B: velocity lag.** The rig mocap node differentiates on receipt time and low-passes with alpha 0.2. The twin does
not filter.

Offline replay on the rig bags, carriers' raw z, the node's header stamps, against a zero-phase 6 Hz reference:

| Flight | `ema` lag | `ema` deviation | `slope` (4 frames) lag | `slope` deviation |
|---|---|---|---|---|
| lift3_3 | 4 frames (44-45 ms) | 0.8-1.8 cm/s | 1-2 frames (11-22 ms) | 0.4-0.7 cm/s |
| liftc_3 | 4 frames (44-46 ms) | 1.0-1.4 cm/s | 2 frames (22-23 ms) | 1.0-1.1 cm/s |
| t5_2 | 3-4 frames | - | 2 frames | - |

- The replayed `ema` equals the bag's own twist.
- The mocap is very quiet (0.02 mm position sd in the still bag), so on this rig lag, not noise, dominates `slope`'s
  error. The critic's white-noise argument (slope sd 2.1x) holds for noisy positions only.

## Changes

All are switches, off by default. Rig use only through an explicit opt-in.

**A: `cmd_next_tick`** (tracker parameter, default from env `TRACKER_CMD_NEXT_TICK`; `1` = on, anything but 0, 1 or
empty warns and stays off; a startup WARN line when on).
- Once airborne (`takeoff_requested` and `_kt_airborne()`), the THROTTLE sent is node 0's u_state[2] plus the
  first-stage rate x DT. That is exact for the u_state integrator.
- Roll, pitch, yaw and the takeoff spool keep node 1's command.
- While live, the warm start is not shifted: 20 ms passed, not a 0.1 s stage.
- QUAD4's tracker pins it off (graph_attach.py). The newcomer is never in the experiment.

**B: `drone_communication/velocity_filter.py`**, modes `ema` (the old path, byte-for-byte), `slope` and `none`.
- Rig node: env `MDC_MOCAP_VEL_FILTER` (default `ema`, the old path) applies only to the carrier drone bodies in
  `MDC_MOCAP_VEL_DRONES` (default 0,1,2), with a WARN line. The ring, tip, ball and QUAD4 keep `ema` (they feed his
  stack, our planner and the cable estimate).
- Twin emulator: parameter `velocity_filter`, default from env `SIM_MOCAP_VEL_FILTER` (default `none`, the old plain
  difference). When set, the ring gets `ema` and the carriers the chosen mode, as on the rig.

**The rig is protected.** `tools/rig_flight.sh` unsets `TRACKER_CMD_NEXT_TICK`, `MDC_MOCAP_VEL_*` unless
`RIG_TIMING_SWITCHES=1`, so a stray export cannot reach the rig.

**Tests:**
- test_velocity_filter.py (4)
- test_next_tick_command.py (3: throttle only, airborne only)
- test_real_mode_launches.py (QUAD4 pinned off)
- gate --quick PASS 15:14 (before the critic fixes; rerun before the batch)

## Twin, tonight on the lab PC (exploratory card runs)

**Stage 1, the trigger.** `delay_sweep_lift3_d000/d020/d040` (SIM_CMD_DELAY_S 0 / 0.02 / 0.04), 2 repeats each.
- The hypothesis needs a late loop.
- If d040 shows no bob, the twin cannot test A or B, and stage 2 is reported as uninformative.
- Bob: carrier z band amplitude (below) >= 2x d000's in both repeats.

**Stage 2, at +40 ms.** 2 repeats each, one of each arm per wave of 4 slots. One variable per neighbour:

| Arm | SIM_MOCAP_VEL_FILTER | TRACKER_CMD_NEXT_TICK | Against |
|---|---|---|---|
| t0a | none | 1 | d040 (A alone, no rig lag) |
| t1 | ema | 0 | d040 (the rig-like lag) |
| t2 | ema | 1 | t1 (A under the rig lag) |
| t3 | slope | 0 | t1 (B) |

Configs: configs/experiments/timing_lift3_*_d040.yaml. Their `lab_env` reaches the containers through
~/handoff_2026-10-14/delay_batch.py; run_experiment.py ignores it. The batch needs `ALLOW_DIRTY_SRC=1` (the tree is
uncommitted).

## Metrics

Bars are frozen here. The scorer, ~/handoff_2026-10-14/score_delay.py, is extended to these before scoring and is
copied into the run notes. Windows:
- **R:** the first 6 s of the lift ramp.
- **H:** the last 8 s of the hover before LAND.

| Metric | Window | Definition |
|---|---|---|
| **bob** | R | Each carrier's z minus its 0.3 s centred moving average, band 0.5-2 Hz (zero-phase); the RMS, max over the three carriers |
| tilt | R and H | Ring tilt mean and max |
| track | R | Ramp-mean carrier z error to the planner reference, and the time from ramp start to ring z 0.95 m |
| hold | H | Hover z sd |
| slew | R | Model-unit throttle slew: p50 and % of ticks above 0.5/s (descriptive, no bar) |

**Checks.** Each run's launch log shows the expected switch lines ("command delay", "velocity filter …",
"next-tick throttle ON") or the run is void.

**Repeat-spread rule.** An arm beats its neighbour only if BOTH its repeats beat BOTH of the neighbour's.

**A passes** (t2 against t1), all of:
- (a) bob lower by >= 20 %;
- (b) tilt max not higher;
- (c) ramp-mean z error within 1 cm of t1's, and time to 0.95 m within 0.5 s;
- (d) hold not worse by more than 0.3 cm.

**A falsified:** bob not lower, or (c) fails. t0a against d040 says whether A needs the lag to matter.

**B passes** (t3 against t1), all of:
- (a) bob lower by >= 20 %;
- (b) tilt max not higher;
- (c) slew p50 <= 1.5x t1's.

**B falsified:** bob not lower by 20 %, or slew p50 > 1.5x t1's.

**Twin fidelity.** If t1's bob is not >= 15 % above d040's (both repeats), the twin does not see B's lag. B is then
judged on the rig only, after its own frozen bars.

## Rig plan

Not before the twin passes, and each switch only with Wesley's word.
- Lift-only flights, our stack first, then combined, each switch alone, with switch-off runs interleaved on the same
  day and packs.
- Bars written into this card before those flights.
- Abort on a growing bob (`fleet land`); drop and land is armed.

## Risks

- **A:** a slower throttle recovery after a pull, and a possible sag in the lift. The spool is excluded.
- **B:** noise is low on this rig, but untested in flight.
- Both change the plant the planner's feedforward was tuned against.

## Critic

Critic agent, 14 Oct ~15:20, on rev 1. **Verdict: do not run as written; run after the fixes; twin first; the rig waits
for the offline B replay and its own bars.**

| # | Challenge | Response (rev 2) |
|---|---|---|
| 1 | No arm contains the late loop the hypothesis needs, so the twin cannot falsify it | Stage 1 (the delay sweep) is the trigger; stage 2 runs at +40 ms; the t0a neighbour is added |
| 2 | A changes all four channels | Throttle only |
| 3 | A slows the takeoff spool (the pin stores the realised throttle) | Live only after TAKEOFF and off the stand |
| 4 | The warm start shifts 0.1 s per 20 ms tick | Not shifted while A is live |
| 5 | A has no log line and stays off silently on a bad value | WARN line; any value but 0, 1 or empty warns |
| 6 | A by global env would also reach QUAD4 and the rig | QUAD4 pinned off; rig_flight unsets the switches unless RIG_TIMING_SWITCHES=1 |
| 7 | B by env reaches every body (his inputs, our planner, the cable estimate) | B on the carrier drones only (MDC_MOCAP_VEL_DRONES); the ring, tip, ball and QUAD4 stay ema |
| 8 | The twin has no position noise, so B's chatter bar cannot fail; replay offline first | Done (table above): slope halves the lag, and its deviation from the reference is lower on these quiet bags; a chatter bar is kept (slew p50) |
| 9 | A's slew bar passes by construction, and the throttle sd bar is biased | Bars judged on motion: bob, tilt, track, hold |
| 10 | B's falsifier has no number | B falsified if bob not lower by 20 % or slew p50 > 1.5x |
| 11 | The 15 %/10 % margins sit inside the repeat spread | Both-repeats-beyond-both rule; margins 20 % |
| 12 | Height at +6.5 s depends on the bob's phase | Ramp-mean error and time to 0.95 m |
| 13 | Units, scorer location, env path to the lab | Stated above |
| 14 | "Cuts the damping margin" is not shown: slew is a symptom; liftc_3 was clean without the changes | Agreed: the status line above |
| 15 | Rig plan with no frozen bars, n = 1 baselines | Interleaved on/off runs; bars written before any rig flight |
| 16 | No record that Wesley asked for the code | He asked in the session (~15:05); recorded in decisions.md 14 Oct |
| 17 | Note: alternatives not tried (alpha 0.4-0.5; a model throttle bound of 2.5/s) | Noted as later arms if A or B fail |
