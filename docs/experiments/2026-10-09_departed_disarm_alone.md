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
