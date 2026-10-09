# departed_disarm_alone (9 Oct 2026): a drone down at its rest off the ring is disarmed alone

## 1. Question

Wesley, 9 Oct (W4/W5, the freed drone landing by itself): "Push down, then disarm it alone": no armed
time at zero throttle on the floor after a flight, where Betaflight's airmode is still active.

## 2. Hypothesis

With `departed_ground_idle` on, a drone that has left the ring (a freed carrier, an aborted QUAD4, every
drone after a drop) and is back at its own resting height is disarmed alone within 0.5 s of its tracker's
idle latch. Nothing else changes: no fleet abort, no LAND, the ring and the other drones fly on, and a
drone still on the ring is never disarmed this way.

## 3. The one variable

The disarm-alone path (code, behind `departed_ground_idle`, default off):
- tracker: when its idle latch comes from the departed branch (planner flag + within 0.03 m of its rest,
  0.2 s), it publishes `/drone_i/down_alone` once;
- fleet manager: disarms that one drone through its arming service, only if the planner's
  `/drone_i/departed` arrived during this flight, the fleet is flying, the drone is not on the
  partner's stream and its tracker answers; its arming feedback False is then expected (no fault path);
- planner: an aborted newcomer disarmed at its rest logs as such (not as a fault).

Configs (all existing except the W4 copy): `w4_gi_g2_release_orbit` (W5 twin), `w4_da_release_hover`
(W4 twin = `w4_gi_release_drop` with `departed_land: true`), `w4_gi_release_drop`, `twin_all_a1`,
`twin_all_b1`, `w4_gi_drop_forced`. Code tree: rig-prep + this change.

## 4. Baseline

The same configs on the armed-idle code (rev 1 of this card wrongly said every departed drone idled in
each). From run.csv's armed edge (`tools/departed_metrics.py`: idle_t, disarm_t, idle_armed_s = seconds
armed at throttle 0 on the floor, before_landed):

| run | config | departed drone | idle_armed_s | disarmed |
|---|---|---|---|---|
| R1288 | w4_gi_g2_release_orbit (W5 twin) | drone 1 (freed carrier) | 21.06 | by the fleet disarm |
| R1299-R1302 | twin_all_b1 (rod tip, 6 s press) | drone 3 (QUAD4) | 20.93 / 21.53 / 20.49 / 20.76 | by the fleet disarm |
| R1294 | twin_all_a1 (abort, LAND) | drone 3 | none: no idle before LANDED | by the fleet disarm |
| R1289 | w4_gi_drop_forced | drone 3 (others reached rest at LANDED) | 0.22 | by the fleet disarm |

(Carriers latched the ordinary ground idle 3.6-3.8 s before the fleet disarm in R1294: not this card.)

## 5. Pass / fail numbers

Per run, `tools/departed_metrics.py` on run.csv (sim seconds; the armed edge the plant receives, not the fleet
manager's log line), clipped to run.csv's last row. Expected per arm: a departed drone whose idle latch comes
before LANDED is disarmed alone; one that reaches its rest only at LANDED is disarmed by the fleet ("fleet
first": not applicable, not a P1 fail).

| # | metric | supports | falsifies |
|---|---|---|---|
| P1 | departed drone with idle_t before LANDED: disarm_t - idle_t (its armed edge) | <= 0.5 s, before_landed, disarm_dz within +-0.03 m | > 1.0 s, or still armed at LANDED |
| P2 | aborts, emergency stops, LANDs not in the config's schedule | none; the run reaches LANDED | any |
| P3 | the drone after its disarm edge | z within its rest +-0.03 m to the end of run.csv | moves > 0.05 m |
| P4 | regression count: disarm edges before LANDED of drones the planner never flagged departed (every carrier) | 0 | any |

P4's guard is one source, stated honestly: the planner's `/drone_i/departed` (received this flight) plus
the tracker's two-sided 0.03 m band around its own rest. No arm can put a wrongly flagged drone near its
rest; P4 is a regression count, not a test of that guard.

Grey, not this card's question: a departed drone that never reaches its rest (the 6 s press limit holds it
above) is not disarmed alone and hovers until the fleet disarm, as on 8 Oct.

Rig bar (W4/W5, Wednesday): the freed drone's elrs_interface logs its disarm (and `fc_arm_state` reads
disarmed) within ~1 s of its tracker's "down at its rest off the ring" line while the others stay armed.
The rig is safe even if the last message is lost: elrs_interface disarms after 0.5 s without a command.

## 6. Repeats

Neighbours first, once each: the freed drone landing with the fleet at LAND (`w4_gi_release_drop`), QUAD4
aborted (`twin_all_a1`), QUAD4 on its rod tip (`twin_all_b1`), every drone departed after a forced drop
(`w4_gi_drop_forced`), and the early disarm: QUAD4 taken out while still parked on the floor (`twin_land_e2_lift`,
LAND mid-lift; the floor-start case `test_departed_idles_even_if_it_never_rose_4_cm` allows). Then W5 twin
(`w4_gi_g2_release_orbit`) x2 and W4 twin (`w4_da_release_hover`) x2.

## 7. Cost

9 Gazebo runs (lab PC slots when idle; else the laptop, ~6-7 min each).

## 8. New code?

Yes: tracker (`_ground_idle_update` publishes `/drone_i/down_alone`), fleet manager (`_departed_callback`,
`_down_alone_callback` with a disarm straight to that drone's radio after the service reply, the expected-disarm
return in `_arming_feedback_callback`, the fleet disarm skipping it), planner (`_newcomer_arming_cb` log),
tools/departed_metrics.py (disarm_t, idle_armed_s, before_landed). Unit tests: fleet_manager test_down_alone.py
(9), tracker test_ground_idle.py (+3). If it fails: the registry rows stay, the code is removed and the departed
drone keeps armed idle.

Switch coupling (critic must-fix 4, option b): no separate switch. `departed_ground_idle` now means pushed to
its rest, then disarmed alone (Wesley 9 Oct: "Replaces armed idle for a departed drone"). The armed-idle rows
of card 2026-10-09_departed_ground_idle are superseded for the idle half (their push-down half stands); the LAND
matrix configs (`*_land_e*`, all with the switch on) fly after this change on the same tree.

Lock wording: when the switch becomes a default, the working rules' lock line gets this third exemption from
"any disarm in flight -> emergency stop" (after newcomer_fault_land and the partner mux), and that `_down_alone`
silences that drone's feedback until the next ARM (fine only because its tracker has closed).

## Critic

9 Oct, critic on rev 1: **GO WITH CHANGES** (the code can fly as written; bars, baseline and scope rewritten
before any run). Must-fix and what was done:
1. Score P1 on the armed edge the plant gets (run.csv `d{i}_armed`), not the fleet manager's line; the tracker's
   last disarm is one message from `on_close()` and the twin bridge holds the last command. -> P1 rewritten on
   `tools/departed_metrics.py` (disarm_t, idle_armed_s, before_landed added); plus (note b) a disarm straight to
   the drone's ELRSCommand after the service reply.
2. Baseline numbers in the table; the rev-1 sentence was wrong for R1289 and A1. -> table above; per-arm
   expectation and "fleet first = not applicable".
3. P4 cannot fail as written; add a floor-start early-disarm neighbour and fly neighbours before repeats. ->
   P4 is a regression count with its guard stated; `twin_land_e2_lift` added; order fixed in section 6.
4. Switch coupling. -> option b, section 8.
5. Lock wording at the merge. -> section 8.
6. The fleet disarm logs "tracker not answering" for a drone already disarmed alone (a false alarm on the
   rig). -> `_disarm_fleet_thread` skips `_down_alone` drones (test added).
7. A rig bar. -> section 5.
Notes taken: (a) nothing in the planner, runner or launch depends on the departed drone's tracker process
after it closes; (c) the combined M1 stack keeps the feature off (`_departed_gi` is off in partner runs), so
QUAD4 landing under Tejen's LAND keeps the old landing there; if it is ever turned on, latch that drone's mux
on the disarm; (d) the disarm-alone / LANDED race in A1 and the drop is harmless either way.

## Results (9 Oct; rows R1304-R1310, R1324-R1326)

| run | arm | departed drone | first at rest (s) | armed edge - at rest (s) | before LANDED | disarm z - rest (m) | P2 | P4 |
|---|---|---|---|---|---|---|---|---|
| R1309 / R1326 | W5 twin (circle) | freed carrier | 48.21 / 48.41 | 0.20 / 0.19 | yes / yes | 0.000 / 0.001 | none | 0 |
| R1310 / R1325 | W4 twin (hover) | freed carrier | 37.70 / 37.87 | 0.19 / 0.22 | yes / yes | 0.002 / 0.000 | none | 0 |
| R1304 | freed carrier, departed_land off | freed carrier | 54.79 | 0.24 | yes | 0.002 | none | 0 |
| R1306 | QUAD4 on its rod tip | QUAD4 | 65.80 | 0.20 | yes | 0.000 | none | 0 |
| R1307 | forced drop | QUAD4 (others: fleet first) | 43.28 | 0.22 | yes | 0.001 | none | 0 |
| R1324 | QUAD4 taken out parked (early disarm) | QUAD4 | 30.87 (hand) | 0.185 (hand) | yes | 0.000 | none | 0 |
| R1305 | QUAD4 aborted (A1) | QUAD4 | at LANDED | - (fleet first, n/a) | - | 0.006 | none | 0 |
| R1308 | LAND mid-lift, no ATTACH | none (QUAD4 never out) | - | n/a | - | - | none | 0 |

Baseline armed time at throttle 0 for the departed drone: 21.06 s (R1288), 20.49-21.53 s (R1299-R1302).
P3 (reviewer): the largest |z - rest| from the disarm edge to the end of run.csv is 0.0024 m (R1304).

**Verdict:** supports. A drone that has left the ring and is back at its rest is disarmed alone ~0.2 s after
reaching it (the tracker's 0.2 s dwell), before LANDED, with no fault path and no carrier disarmed alone.

## Reviewer (9 Oct): SUPPORTED, with corrections (taken)

1. The column scored is "armed edge - first back at rest", an UPPER BOUND on P1's frozen metric (disarm_t -
   idle_t): the idle latch needs 0.2 s inside the same band, so it cannot precede at_rest. run.csv shows an
   armed throttle-0 sample only in R1304 and R1325 (0.02 s); elsewhere the throttle drops from the press
   (0.38-0.42) to 0 on the same row as the armed edge; the tracker logs agree. P1 holds either way.
2. R1324 is hand-scored: `tools/departed_metrics.py` skips QUAD4 there (it rose 0.080 m, under the tool's
   0.2 m "flew" gate). Definition used: the first sample after LAND within 0.03 m of rest (30.87 s) to the armed
   edge (31.055 s). Read literally with the tool's idle_t (the parked idle from 3.03 s) it would fail P1; that
   idle is before any flight (airmode not active), not this card's case.
3. The W4 twin configs (`w4_da_release_hover`: R1310, R1325; `w4_gi_release_drop`: R1304) have no armed-idle
   baseline run; R1288 (cited in the registry) is the W5 config. The baselines R1288/R1289/R1294 also predate
   the 6 s press limit (6e0c825), which did not bind there (press <= 0.87 s).
4. Two code trees: cb09b39 (7 runs) and 734f0f4 (R1324-R1326, adds 0400066, whose branches were never
   reached: every LAND came with the fleet flying); HEAD e434a11 not flown, its diff off by default; parameter
   read-backs identical between the repeats.
5. Section 6 named `twin_land_e2_lift` as the early-disarm arm; R1308 showed it is not (no ATTACH), and R1324
   (`twin_land_e3_parked`) replaced it after R1308 was seen, on the later tree.
6. Nothing contradicts "expected disarm, no fault path": no EMERGENCY / FAULT_LAND / "not answering" lines; the
   planner logs the disarm at INFO (it still marks the newcomer out internally, harmless: R1306's next LAND was
   the scheduled one). Each disarm-alone is followed by the tracker process exiting (-15), as designed.
7. Not shown: elrs_interface / fc_arm_state / Betaflight airmode (rig bar, section 5); a lost disarm message;
   QUAD4 has one run per arm (R1306, R1324); untouched armed time at throttle 0: QUAD4 parked before ATTACH
   (27-40 s, before any flight) and carriers before the fleet disarm (0.6-3.4 s); no bar on the ring (R1326's
   6.9 deg peak started before its disarm). The feature is off in air starts, partner runs, and for a drone
   with no pose at TAKEOFF (R1308: a slow planner start; logged, fails safe to the fleet disarm).
Found outside this card: R1324's magnet command flapped ON/OFF (the runner re-sends ON for 2 s; each refused ON
gets an OFF, as designed); the refusal's log line is now throttled to one per 2 s.

## After the review: a failure found by the LAND matrix (9 Oct evening)

R1378 (card 2026-10-09_land_every_phase, LAND in QUAD4's climb): QUAD4 was disarmed alone at its rest while the
carriers still flew; its tracker process exited ~0.1 s later (a disarm after a flight shut the controller down);
the graph stalled 0.54 s sim (2.3 s wall) and two carriers timed out on pose: fleet EMERGENCY STOP at 0.8-0.9 m.
Over the day 1 of ~20 mid-flight disarm-alones (the others: 0.02-0.05 s gaps). This is the P2 failure ("an abort
caused by the expected disarm") that this card's 10 rows did not show. Fix f330bd2: the tracker stays up after a
disarm alone (one-shot keep-alive, unit-tested; it keeps sending disarmed commands; the fleet disarm after LANDED
closes it). The verdict above stands for its rows only; the rows are re-flown on f330bd2 with the LAND table.

