# Card X4: the endurance relay (30 Oct 2026)

Owner Wesley (decisions.md 30 Oct ~12:30: X4 is a headline added-value result; ~12:35: the relay rota approved as
twin-only code behind switches, default off, critic waived; B = 5 min, swaps announced (~13:40: B = 10 min, Q1); the twin's ring for claims the
0.86 kg class plus one 1.15 kg cell). Plan row X4 of `docs/plan_rescope_2026-10-30.md`. Written 30 Oct 13:15 AEDT.
**Bars frozen before any run (section 5); B = 600 s from Wesley's answer to Q1 (30 Oct ~13:40, before any run).** The
short run is exploratory and may fly now.

## 1. Question
Can the carriers take turns to leave the ring, to recharge or to run an errand, and come back, so the ring flies far
longer than any one pack? ("a ring that outlives every pack, carriers leaving on errands and coming back")

## 2. Hypothesis
- 8 quads fly the lapped figure-8 for ~30 min of simulated time (1781 s projected, TAKEOFF to touchdown): 26 announced
  soft detaches, one per lap end, each followed by the same quad's rejoin in the next lap-end hold; no drop.
- Every release and every fold-in stays under 5 deg of ring tilt (8-quad evidence on an older tree: releases 1.84-3.18
  deg, fold-ins 1.05-2.00 deg, R2776/R2779).
- The path error does not grow lap by lap (the last lap within 1.25 x the first) and the ring's height does not drift
  (within 10 mm first lap to last): nothing accumulates over 26 resizes up and 26 down.
- The fixed team must land at B; the relay flies ~3.0 B (B = 10 min; `--check`: 1782 s = 2.97 B, longest pack 581 s).

## 3. The one variable
The relay (the `relay:` section: leaves and their rota). Everything else identical in both files (a test pins it:
`tools/test/test_relay_rota.py::test_the_fixed_team_and_the_relay_differ_only_in_the_relay`): the 8-quad R 0.5 m /
0.9 kg ring (`eight_rigid_ground_rig_R50_L100.sdf`), 1 m cables, the size-0.5 figure-8 at 0.2 m/s lap by lap (19.2 s
laps, 3 s lap-end holds, `traj_laps`), 5 SQP iterations, `s_chain_pair.yaml`, `rejoin_departed`/`rejoin_task`/`relay_task`
on (inert without a leave), `detach_unload_s 3` (the announced soft detach), the default (slope) velocity filter.

| Config | Arm | Runs |
|---|---|---|
| `configs/experiments/x4_relay_30min.yaml` | the relay: 26 leaves, every 9th an errand, 1 final lap at 8 (28 laps) | 2 |
| `configs/experiments/x4_fixed_B.yaml` | the fixed team: no leave, the whole laps that land it by B (24 laps at B = 600 s; `--check` 578 s = 0.96 B) | 2 |
| `configs/experiments/x4_relay_30min_m115.yaml` | the 1.15 kg cell: the relay on the same ring at 1.15 kg (`eight_rigid_ground_rig_R50_m115_L100.sdf`, new) | 1 |
| `configs/experiments/x4_relay_short.yaml` | exploratory first check: 3 leaves (pad, errand, pad), 5 laps | 1 |

How a swap flies (`relay_task`, twin only; `tools/experiment/relay.py` writes the events): at `lap k done` the runner
sends TASK (the leaver's route, queued), ATTACH (the last leaver, parked at its pad) and DETACH (the leaver). The parked
quad flies back to its staging point with the path held and rejoins (approach, weld, 10 s fold-in blend); only then
does the leaver unload and release (a leave waits for a full ring: 8 -> 7, never 6); it flies out with the path held
until it is clear of the lap's keep-out (its staging radius, 1.28 m, about every point of the lap), the path runs, it
reaches its pad (8 s dwell = the recharge) and parks. The leaver is the quad with the oldest pack; every 9th leave
sends the freshest quad on an errand instead (its pad point, pickup (1.0, 1.56) 3 s, delivery (-1.0, 1.56) 3 s; no
recharge). Pads (one per plate, the nearest point clear of the lap by 5 cm, 2 cm inside the +-1.57 m staging bound):
0 (1.55, -0.95), 1 (1.18, 1.32), 2 (0, 1.54), 3 (-1.18, 1.32), 4 (-1.55, -0.95), 5 (-1.18, -1.32), 6 (0, -1.54),
7 (1.22, -1.29). The pack clock runs from TAKEOFF, resets at the end of a pad dwell, and ends at touchdown. B is a
budget, not a battery model.

## 4. Baseline
- The fixed team `x4_fixed_B` (not flown; flown on the same tree as the claim): mission / B <= 1 by construction; its
  laps are the per-lap path error and height references.
- Laps without events on an older tree: R2778 (9 laps, ema filter, week7 63b504f): clock-free path error 16.49 mm, tilt
  peak 1.28 deg, 8 lap ends held 3 s, the clock never ran back. Swaps at lap ends, older tree: R2779 (four rejoins at
  lap ends: fold-ins 1.05-1.91 deg, ATTACH to weld 19.6-24.1 s, P 19.69 mm). Cited for scale only; not the comparator.
- **The projection** (`python3 tools/run_experiment.py --check configs/experiments/x4_relay_30min.yaml`): each swap
  holds the path ~46 s (the way back 2-6 s, ATTACH to weld ~21 s, the 10 s blend, the 3 s unload and its release, the
  way out of the keep-out 4-8 s), so a lap with a swap takes ~65 s. With at least 7 of 8 on the ring only one quad is
  off at a time, so the 8 recharge one after another: a pack must last 8 swaps (9 with the errand) = ~580 s (9.7 min).
  **B = 5 min cannot be kept by any rota at 8 quads with this rejoin**: the holds alone take 8 x 46 s = 6.1 min, even
  in hover and with no errand. Q1 below.

## 5. Pass / fail numbers (frozen 30 Oct 13:15, before any run; B = 600 s per Q1)
Scored by `tools/x4_metrics.py` (laps: the rows where the ring's reference moves between `lap k-1 done` and
`lap k done`; clock-free error = RMS of the distance to the figure-8 and the height error; event tilt = the tilt
vector's peak departure from its 5 s pre-event mean, over 10 s or to the next event) and `tools/s_metrics.py` (late
ticks, the planner's n).

| # | Metric (each relay run) | Confirms | Falsifies |
|---|---|---|---|
| R1 | the rota flown: leaves tried, swaps done (released and welded back) | 26 / 26, mission (TAKEOFF -> touchdown) >= 1700 s | a drop or abort, a swap not done, a REJOIN ABORT / REFUSED, TASK REFUSED / ABORT, RELAY LEAVE CANCELLED |
| R2 | carriers | >= 7 in every lap and no planner resize below n = 7 | a lap or a resize with 6 |
| R3 | ring tilt at every release and fold-in | every peak <= 5.0 deg | one over 5.0 deg |
| R4 | per-lap clock-free path error | last lap <= 1.25 x the first; every lap <= max(1.25 x, + 3 mm) the fixed team's mean lap (its 2 runs) | last lap > 1.25 x the first |
| R5 | height drift (last lap's mean height error less the first's) | abs <= 10 mm | abs > 10 mm |
| R6 | the path never stopped mid-lap | no RELAY HOLD line | a RELAY HOLD |
| R7 | every pack interval | <= B (600 s) | one over B |
| F1 | the fixed team (each run) | lands with mission <= B, no drop | a drop, or mission > B |

Headline (reported, not barred): mission / B, relay and fixed, and the longest pack. Void (re-fly, not scored): more
than 1 % late planner ticks, a runner failure (no subscriber, a pose timeout), a LOG_TIMEOUT or TRAJ_TIMEOUT caused by
the harness, real-time factor under 0.15. Both repeats must confirm; one falsified repeat falsifies the claim. The
1.15 kg cell is scored on the same bars and reported beside the claim (1 run, not a repeat). The short run is
exploratory: its registry row names its falsifier (the R1, R3, R6 lines) and nothing is claimed from it.

**Q1 (B, Wesley)**: at B = 300 s, R7 is projected to fail on all 8 quads (~580 s). Options: (a) B = 10 min (600 s,
recommended: one swap per lap end as approved, the projected packs inside it, mission 30 min = 3.0 B; the fixed team
then flies 24 laps; one number in three files plus the fixed team's `traj_laps`/`duration_s`); (b) keep 5 min and report
the pack the rota needs instead of R7 (the claim becomes "a 30 min mission on ~10 min packs"); (c) cut the swap cost (a
shorter fold-in blend is X7's variable; still over 5 min in hover). The value chosen goes into this table before the
first claim run.
**Answer (Wesley 30 Oct ~13:40): (a), B = 600 s.** Set in the four configs' `budget_s` and the fixed team's `traj_laps`
24 / `duration_s` 780 (30 Oct 15:41, every `--check` passes).

## 6. Repeats
2 (relay, fixed team); 1 for the 1.15 kg cell (Wesley 30 Oct: one cell per table); 1 exploratory short run.

## 7. Cost
6 Gazebo runs on the lab PC, 2 slots of 12 cores (8-quad RTF 0.22-0.25, R2776-R2779): the 30 min relay ~1781 s sim
= ~2.3 h wall each (3 runs), the fixed team ~290 s sim = ~25 min each (2), the short run ~295 s sim = ~25 min. About
8.5 slot-hours, ~4.5-5 h wall on 2 slots. `tools/remote/lab_batch.py` needs `--wall-cap-min 180` for the 30 min runs
(its default 30 min would stop them). At B = 600 s the fixed team takes ~50 min each.

## 8. New code?
Yes, twin only, every part off unless typed; critic waived by Wesley 30 Oct (decisions (a)); the reviewer reads the
final table.
- `relay_task` (dissipative_node, launch knob in common.yaml, refused on the rig and without `rejoin_task`): a carrier's
  TASK waits for its leave and starts from its staging point; in relay mode an announced leave waits until no other
  carrier is off the ring; the path is held while a relay drone is inside the lap's keep-out (its route is refused
  unless it leaves the keep-out once and stays out, every waypoint clear of the whole lap, and its way back keeps the
  staging radius of the held ring); the task parks at its last waypoint; an ATTACH (refused while the path runs)
  flies it back through its pass points (dwell 0) to its staging point with the path held, then its rejoin starts; an
  ATTACH during a dwell waits for it (the dwell is never cut). Tests `src/dissipative_planner/test/test_relay_task.py`
  (12, incl. a full swap on the real lap clock: the path never runs with a relay drone inside, never runs back, never
  two off); `src/bringup/test/test_relay_task_launch.py` (1: the switch reaches only the twin planner).
- The runner: WAIT_LOG `#n` (the n-th matching line), the relay's planner lines as events; `tools/experiment/relay.py`
  (the rota, pads, events, projection; `relay:` in a config, refused when its laps, switches or duration do not fit);
  `tools/x4_metrics.py` (the scorer). Tests `tools/test/test_relay_rota.py` (23), `tools/test/test_x4_metrics.py` (1).
- The 1.15 kg 8-quad world (`tools/make_carry_worlds.py` entry, `simulation_assets/eight_rigid_ground_rig_R50_m115_L100.sdf`:
  the 0.9 kg world with mass 1.15 and inertia scaled; a world SDF, Wesley's word before it flies).
- If X4 is cut or fails: the result is recorded; the code stays off by default until Wesley decides; the default is to
  delete it (relay_task, relay.py, x4_metrics.py, the configs, the world).

Known limits, beside the claim: the rejoin is the twin's (a weld wherever the rod's end is, rigid hinged rods); the
pads hover at the staging height (no landing, no pack model); the errand's pickup and delivery carry no object; the
staging bound (+-1.57 m) leaves the errand's top-edge leg only 3-4 cm outside the lap's keep-out; 8 quads run below
real time.
