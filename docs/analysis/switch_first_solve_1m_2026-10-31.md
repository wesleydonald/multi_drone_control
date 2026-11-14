# Q001: the switch of solvers on the 1 m runs (31 Oct 2026)

Offline log analysis for request Q001 (`docs/lab_requests.md`), thesis section 4.2.3. Nothing was flown.

**Verdict.** The count survives the move to 1 m: 20 switches, no first solve failed, no solve after it failed, and
none failed in the second after it. The 0.55 m figures reproduce exactly. The "24-28 ms" does not belong to the
simulated switches: it is the real-world flights of 7-8 Oct, and it stands as it is. Simulated solve times are load
on the lab PC, not solver speed, so the proposed sentence quotes none.

**Three things the thesis chat should know:**
1. "First solve" means the first solve the planner published. Two priming solves ran before it on the same tick, and
   their status and time are not logged (section "Not logged").
2. The text says each first solve had "a residual in the range of the larger problem's last solve". The simulated
   runs meet that. Two real-world flights, RIG-1007-r0012 and -r0013, do not (section "Reproduction").
3. Two later real-world switches exist that the text does not count, on 1 m cables with the 1.15 kg ring. They are
   slower than 24-28 ms and inside the budget (section "Switches outside the count").

## Summary

| set, n | first fail | next fail | fail in 1 s | first ms, min/med/max | stat first | stat last | eq first | eq last |
|---|---|---|---|---|---|---|---|---|
| 1 m, 20 | 0 | 0 | 0 of 186 | 15.7/24.4/93.5 | 1.0e-4..7.5e-4 | 1.2e-4..8.3e-4 | 8.0e-9..2.4e-7 | 1.4e-7..7.3e-7 |
| ta+tb, 16 | 0 | 0 | 0 of 146 | 16.3/49.2/93.5 | 1.0e-4..7.5e-4 | 1.2e-4..8.3e-4 | 8.0e-9..6.0e-8 | 1.4e-7..7.3e-7 |
| 0.55 m, 20 | 0 | 0 | 0 of 185 | 15.6/21.9/88.0 | 7.2e-5..3.9e-4 | 1.2e-4..4.4e-4 | 6.8e-9..2.3e-7 | 1.8e-8..3.5e-7 |

"stat" is the KKT stationarity residual (`res_stat`) and "eq" the dynamics residual (`res_eq`). "first" is the first
solve of the smaller problem, "last" the larger problem's last solve, one tick before. Ranges run over the switches.
The ta+tb row is the 1 m runs without D1 and W5, because F007 keeps `dl_d1` and `w4_gi` as the earlier setup. Their
1 m versions flew too, so all 20 can be 1 m.

## Which runs

All 33 registry rows are valid: none is void, no later row voids one, and every manifest says finished.

| runs | arm | switches |
|---|---|---|
| R2486, R2495, R2504 | A1, measured pull, 120 deg | 3 |
| R2487, R2496, R2505 | A2, planned pull, 120 deg | 3 |
| R2492, R2501 | W5, circle, unannounced release | 2 |
| R2493, R2502 | D1, second loss (the first loss resizes, the second drops the ring) | 2 |
| R2604, R2610, R2605, R2611 | G135 and G150, measured pull | 4 |
| R2606, R2612 | G150p, planned pull | 2 |
| R2607, R2613 | G165, measured pull (the trackers drop the ring just after the switch) | 2 |
| R2608, R2614 | G180, measured pull, drop rule off (the fleet latches just after the switch) | 2 |
| R2488, R2497, R2506, R2489, R2498, R2507 | A5, A6, detection off | 0 |
| R2490, R2499, R2491, R2500 | A7, A8, no event | 0 |
| R2494, R2503, R2609 | D4, G180d, the planner drops the ring at detection instead of resizing | 0 |

The same split as 0.55 m: the same arms switch. Each switching run has one fall of `n`, from 4 to 3.

## Per switch (1 m)

| run | arm | pull | slot | st 1 | st 2 | fail 1 s | ms 1 | tick | QP | stat 1 | stat L | ratio | eq 1 | eq L | ratio |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R2486 | A1 r1 | meas | 1 | 0 | 0 | 0 of 10 | 80.4 | 219 | 10 | 2.7e-4 | 4.5e-4 | 0.60 | 4.7e-8 | 1.4e-7 | 0.32 |
| R2495 | A1 r2 | meas | 4 | 0 | 0 | 0 of 10 | 19.4 | 216 | 10 | 1.9e-4 | 2.9e-4 | 0.67 | 2.9e-8 | 2.3e-7 | 0.13 |
| R2504 | A1 r3 | meas | 4 | 0 | 0 | 0 of 10 | 79.2 | 307 | 10 | 2.1e-4 | 3.2e-4 | 0.66 | 3.3e-8 | 2.4e-7 | 0.14 |
| R2487 | A2 r1 | plan | 3 | 0 | 0 | 0 of 10 | 18.1 | 216 | 10 | 1.7e-4 | 2.3e-4 | 0.73 | 6.0e-8 | 3.1e-7 | 0.19 |
| R2496 | A2 r2 | plan | 3 | 0 | 0 | 0 of 10 | 22.6 | 267 | 10 | 1.7e-4 | 2.0e-4 | 0.85 | 6.0e-8 | 2.6e-7 | 0.23 |
| R2505 | A2 r3 | plan | 2 | 0 | 0 | 0 of 10 | 20.2 | 238 | 10 | 1.7e-4 | 1.5e-4 | 1.13 | 5.6e-8 | 2.0e-7 | 0.28 |
| R2492 | W5 r1 | meas | 3 | 0 | 0 | 0 of 10 | 19.0 | 216 | 11 | 4.3e-4 | 3.5e-4 | 1.22 | 2.1e-7 | 2.2e-7 | 0.96 |
| R2501 | W5 r2 | meas | 2 | 0 | 0 | 0 of 10 | 85.9 | 293 | 12 | 5.2e-4 | 5.7e-4 | 0.90 | 2.4e-7 | 2.3e-7 | 1.03 |
| R2493 | D1 r1 | meas | 4 | 0 | 0 | 0 of 10 | 16.4 | 267 | 10 | 2.9e-4 | 5.3e-4 | 0.54 | 4.9e-8 | 1.8e-7 | 0.27 |
| R2502 | D1 r2 | meas | 3 | 0 | 0 | 0 of 10 | 15.7 | 288 | 10 | 2.0e-4 | 2.8e-4 | 0.71 | 3.0e-8 | 2.1e-7 | 0.14 |
| R2604 | G135 r1 | meas | 3 | 0 | 0 | 0 of 10 | 93.5 | 313 | 10 | 1.7e-4 | 3.7e-4 | 0.45 | 2.0e-8 | 2.5e-7 | 0.08 |
| R2610 | G135 r2 | meas | 3 | 0 | 0 | 0 of 10 | 23.0 | 286 | 10 | 1.5e-4 | 3.3e-4 | 0.47 | 1.7e-8 | 2.1e-7 | 0.08 |
| R2605 | G150 r1 | meas | 4 | 0 | 0 | 0 of 10 | 25.7 | 274 | 10 | 1.1e-4 | 4.0e-4 | 0.26 | 9.1e-9 | 3.1e-7 | 0.03 |
| R2611 | G150 r2 | meas | 4 | 0 | 0 | 0 of 10 | 82.4 | 216 | 10 | 1.0e-4 | 3.0e-4 | 0.34 | 8.0e-9 | 2.7e-7 | 0.03 |
| R2606 | G150p r1 | plan | 2 | 0 | 0 | 0 of 10 | 16.3 | 288 | 10 | 1.2e-4 | 1.4e-4 | 0.85 | 2.5e-8 | 1.9e-7 | 0.13 |
| R2612 | G150p r2 | plan | 2 | 0 | 0 | 0 of 10 | 17.7 | 212 | 10 | 1.1e-4 | 1.2e-4 | 0.95 | 2.2e-8 | 1.9e-7 | 0.12 |
| R2607 | G165 r1 | meas | 3 | 0 | 0 | 0 of 3 | 73.2 | 275 | 11 | 1.6e-4 | 3.3e-4 | 0.48 | 2.4e-8 | 3.5e-7 | 0.07 |
| R2613 | G165 r2 | meas | 1 | 0 | 0 | 0 of 2 | 72.7 | 212 | 11 | 1.7e-4 | 3.7e-4 | 0.46 | 2.6e-8 | 3.4e-7 | 0.08 |
| R2608 | G180 r1 | meas | 4 | 0 | 0 | 0 of 10 | 79.3 | 298 | 35 | 4.7e-4 | 8.3e-4 | 0.57 | 4.4e-8 | 7.3e-7 | 0.06 |
| R2614 | G180 r2 | meas | 4 | 0 | 0 | 0 of 11 | 84.7 | 302 | 29 | 7.5e-4 | 4.5e-4 | 1.68 | 4.3e-8 | 5.3e-7 | 0.08 |

- st 1, st 2: solver status of the first solve and of the next one. 0 means all five SQP-RTI steps returned without
  error. Convergence is read from the residuals, not from the status.
- fail 1 s: failed solves among the solves in the second from the switch. G165 shows 2 or 3 because the ring drops
  about 0.2 s after the switch and the planner stops solving.
- ms 1: `solve_ms` of the first solve, wall time on a loaded machine (section "Machine and timing").
- tick: `tick_ms` of the switch tick in ms: detection, resize, both priming solves, the first solve, the publish.
- QP: QP iterations over the five SQP steps of the first solve. All 20 used five SQP steps.
- stat L, eq L: the larger problem's last solve, one tick before. Ratio is first over last, above 1 means worse.
- A1, A2, W5, D1 are at a 120 deg gap. G135 to G180 give theirs in the name.
- Configs: `r1m_ta_a1_rN`, `r1m_ta_a2_rN`, `r1m_w4_gi_g2_release_orbit` (W5), `r1m_dl_d1_second_loss` (D1),
  `r1m_tb_g<gap>_rN`.

## Residual against the larger problem's last solve

First solves above their own run's last solve, and the largest ratio:

| set | stat | largest | eq | largest |
|---|---|---|---|---|
| 1 m | 3 of 20 (R2492, R2505, R2614) | 1.68 | 1 of 20 (R2501) | 1.03 |
| 0.55 m | 4 of 20 (R1895, R1906, R1913, R1916) | 1.28 | 4 of 20 (R1895, R1913, R1914, R1916) | 1.76 |

Largest values, stat / eq, over the switches of each set:

| set | steady flight before | last solve | first solve | in 1 s, no G180 | in 1 s, G180 |
|---|---|---|---|---|---|
| 1 m | 2.3e-5 / 9.5e-10 | 8.3e-4 / 7.3e-7 | 7.5e-4 / 2.4e-7 | 3.0e-3 / 6.3e-7 | 1.3e-1 / 2.3e-4 |
| 1 m, no G180 | 2.3e-5 / 9.5e-10 | 5.7e-4 / 3.5e-7 | 5.2e-4 / 2.4e-7 | same | - |
| 0.55 m | 4.7e-5 / 2.4e-8 | 4.4e-4 / 3.5e-7 | 3.9e-4 / 2.3e-7 | 5.6e-4 / 2.3e-7 | 1.4e-1 / 2.3e-4 |

Steady flight is the window 6 s to 1 s before the switch. The last solve is already after the release, which has lifted
it above steady flight, so "in the range of the last solve" does not mean as low as steady flight. G180 is the ring
falling to the latch, with status 0 throughout. Its first solves also take far more QP iterations, as at 0.55 m.

## Reproduction

The same method on all 20 simulated and all 6 real-world switches of T10, not only two or three. Every cell matches
`docs/figures/tables/T10_switch.tex` and `docs/analysis/thesis_tables_T10_T13_2026-10-16.md`.

| T10 row | as printed: fail in 1 s, stat / eq, ms | recomputed |
|---|---|---|
| simulation, measured, gaps 120-165 (13) | 0 of 130, 3.8e-4 / 2.3e-7 | 0 of 130, 3.8e-4 / 2.3e-7 |
| simulation, measured, gap 180 (2) | 0 of 21, 3.9e-4 / 1.7e-8 | 0 of 21, 3.9e-4 / 1.7e-8 |
| simulation, planned, gaps 120-150 (5) | 0 of 34, 9.9e-5 / 2.4e-8 | 0 of 34, 9.9e-5 / 2.4e-8 |
| real world, announced, planned (2) | 0 of 22, 1.1e-3 / 1.1e-7, 24-28 | 0 of 22, 1.1e-3 / 1.1e-7, 23.7, 27.6 |
| real world, announced, measured (1) | 0 of 11, 6.5e-5 / 2.4e-8, 26 | 0 of 11, 6.5e-5 / 2.4e-8, 26.5 |
| real world, unannounced, measured (3) | 0 of 33, 3.3e-4 / 2.9e-7, 25-26 | 0 of 33, 3.3e-4 / 2.9e-7, 25.5, 25.2, 25.7 |
| last solve, all 26, stat / eq | 4.4e-4 / 5.0e-7 | 4.4e-4 / 5.0e-7 |
| simulation, first solve ms (the note only) | 15.6-88 | 15.6-88.0 |
| simulation, 4-carrier p95 ms (the note only) | 106-177 | 106-177 |

**Where "24-28 ms" comes from.** It is T10's range for the two 7 Oct flights, and the other four real-world flights
lie inside it. T10 never reported a simulated solve time, and the thesis draft of 31 Oct already says "in the real
world". Q001's wording folds it into the simulated count. The 1 m re-fly neither replaces nor changes it.

**One clause does not hold in the real world.** The text says each first solve "returned from the warm start with a
residual in the range of the larger problem's last solve". Per flight:

| flight | stat first | stat last | ratio | steady before, max stat | eq first | eq last | ratio |
|---|---|---|---|---|---|---|---|
| RIG-1007-r0012 | 9.0e-4 | 1.3e-4 | 6.81 | 8.8e-4 | 5.6e-8 | 2.3e-8 | 2.46 |
| RIG-1007-r0013 | 1.1e-3 | 3.5e-4 | 3.08 | 6.1e-4 | 1.1e-7 | 1.8e-7 | 0.60 |
| RIG-1008-r302_3 | 6.5e-5 | 4.1e-5 | 1.61 | 2.0e-4 | 2.4e-8 | 1.2e-8 | 1.96 |
| RIG-1008-r303_4 | 6.9e-5 | 2.1e-4 | 0.33 | 1.9e-4 | 2.1e-8 | 2.5e-7 | 0.08 |
| RIG-1008-r304_3 | 9.8e-5 | 3.5e-4 | 0.28 | 1.6e-4 | 3.2e-8 | 3.3e-7 | 0.10 |
| RIG-1008-r305 | 3.3e-4 | 3.8e-4 | 0.87 | 6.5e-4 | 2.9e-7 | 5.0e-7 | 0.58 |

The two 7 Oct flights are above their own last solve, above the top of the range for all 26, and above their own
steady flight (r0012 only just). They are the announced flights with the rods planned at 45 deg. T10's table shows
the number, the sentence does not. RIG-1008-r302_3 is above its own last solve too, but inside the range.

## Not logged

- **The two priming solves, status and time.** Not logged, because the first published solve overwrites them on the
  same tick. What the logs do show: no `planner_fail_*` dump and no "did not converge" line in any of the 40 switching
  runs or the 8 real-world flights, so neither failed hard. An unconverged priming solve (status 20) would leave no
  trace. Time is only a sum, in the table below, and it also holds detection and the resize. To log it: a row or log
  line after each `_prime_solver()` in `_resize_and_release`, with `last_status`, `last_solve_ms`, `last_iters`,
  `last_res`.
- **Inequality and complementarity residuals.** Not logged. `last_res` holds all four and the log writes two. To log
  them: two more columns in `_tick_log_row`.
- **The gap to an offline SQP solve.** Not computed, as on 16 Oct. The QP is dumped only on a failed solve, and none
  failed. To get it: a dump at the resize tick, or `tools/planner_replay.py` on the logged states.
- **Solver time free of waiting for CPU.** Not logged. `solve_ms` is `perf_counter` wall time round the whole solve.
  To log it: a column from acados's `get_stats('time_tot')`.
- **`tick_ms` at 0.55 m and on 7-8 Oct.** Not logged. The column came with the 23 Oct tree.
- **Other users' load on the lab PC, per run.** Not logged. `lab_batch.py` checks once, at the start of a batch.

| 1 m, switch tick, ms | min | median | max |
|---|---|---|---|
| `tick_ms` | 212 | 271 | 313 |
| `tick_ms` minus `solve_ms` | 133 | 218 | 272 |

The priming is in the code of every tree used: 8f4fb2b and b811d90 (real world, 7-8 Oct), 8c468bd (0.55 m), be34f95
(1 m, and neither patch changes it). The swap empties `last_X`, so the first priming solve is cold (up to 15 SQP
steps) and the second warm. The logged first solve is the third on this solver, warm. So T10's "no cold reseed" is
true of the published solve only. The methods text already says "primed twice", so the two agree. Dump files do reach
run folders: other runs under `results/` have them.

## Machine and timing

| set | runs | machine | slots x cpus | Gazebo nice | runs at once | mean RTF | tree |
|---|---|---|---|---|---|---|---|
| 1 m, batch 7 | R2486-R2507 | lab PC | 4 x 4 | 10 | 3 or 4 | 0.22-0.27 | be34f95 + patch 6d980933 |
| 1 m, batch 16 | R2604-R2614 | lab PC | 4 x 4 | 10 | 4 | 0.23-0.28 | be34f95 + patch cf1ab9ad |
| 0.55 m, batch 15 | R1887-R1919 | lab PC | 4 x 4 | 10 | 1, 3 or 4 | 0.23-0.29 | 8c468bd, clean |
| real world | 7-8 Oct flights | rig laptop `tuf` | - | no Gazebo | one flight | real time | 8f4fb2b+, b811d90 |

8f4fb2b+ means with uncommitted changes (the 16 Oct note). Lab PC: `drones@drones`, 24 cores, Docker containers
`wesley_slot1` to `wesley_slot4`. Slots and cpus come from each
run's `manifest.json` and the first line of its `lab_console.log`. "Runs at once" counts lab runs at the moment of the
switch, from the manifests' wall times, for the 20 switching runs of each set. Both 1 m batches flew before another
user took the PC at about 14:00 on 23 Oct (decisions.md). The two 1 m patches differ only in rejoin fixes behind a
switch that is off here (registry). `plan_res_every` is 1 and `solve_budget_s` is 0, so every solve logs its residual
and none is cut short.

| set | first, fast group, ms | first, slow group, ms | later 3-carrier: < 40 / 40-60 / > 60 ms | 4-carrier p50 / p95 |
|---|---|---|---|---|
| 1 m | 11: 15.7-25.7 | 9: 72.7-93.5 | 48 / 2 / 50 % of 4227 | 100 / 123 |
| 0.55 m | 12: 15.6-28.0 | 8: 77.9-88.0 | 47 / 1 / 53 % of 4127 | 101 / 161 |

The first solve took less time than the larger problem's last solve in 20 of 20 at 1 m and 19 of 20 at 0.55 m.

**Reading.** First solves fall into two groups with nothing between them, and the later solves of the same size in the
same runs fall into the same two groups. So the switch does not slow the first solve, and the spread is the machine.
Both groups occur with four runs at once. `docs/experiments/2026-10-16_corner_cutting.md` found the same split in the
stack and put it down to CPU contention with Gazebo and the other nodes. These logs cannot name the cause. Do not
quote simulated milliseconds as solver speed. T10 left them out for this reason.

## Switches outside the count

| switch | setup | st 1 | st 2 | fail 1 s | ms 1 | tick | QP | stat 1 | stat L | ratio | eq 1 | eq L | ratio |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R2483 | W4, lab PC | 0 | 0 | 0 of 10 | 76.0 | 222 | 10 | 2.0e-4 | 2.8e-4 | 0.72 | 3.1e-8 | 2.1e-7 | 0.15 |
| R2484 | W4, lab PC | 0 | 0 | 0 of 10 | 71.6 | 215 | 10 | 2.7e-4 | 4.9e-4 | 0.55 | 4.6e-8 | 1.6e-7 | 0.28 |
| R2485 | W4, lab PC | 0 | 0 | 0 of 10 | 24.7 | 240 | 10 | 2.0e-4 | 2.9e-4 | 0.70 | 3.2e-8 | 2.1e-7 | 0.15 |
| RIG-1029-ch1b | T, 1.15 kg | 0 | 0 | 0 of 6 | 41.4 | 161 | 40 | 5.7e-4 | 1.1e-4 | 5.02 | 4.3e-7 | 3.6e-8 | 12.15 |
| RIG-1029-ch1b_2 | T, 1.15 kg | 0 | 0 | 0 of 11 | 42.5 | 157 | 38 | 8.0e-4 | 7.6e-5 | 10.53 | 5.9e-7 | 6.1e-8 | 9.75 |

R2483-R2485 are valid 1 m runs just below the requested ids, in T10's W4 role. They stay out of the 20 and change
nothing. The two real-world flights are the only other falls of `n` in the rig logs (175 logs, 7-30 Oct). Both first
solves are inside the 60 ms budget, on a T after QUAD4 leaves, with far more QP iterations. Their residuals sit well
above the last solve, as on 7 Oct, so the same caveat applies. Their whole switch tick overran the planner's 100 ms
period, and whether the 7-8 Oct ticks did is unknown. Both flights ended badly for other reasons (registry).

## Method

1. `switch()` of `tools/thesis_freeze_tables.py`, unchanged (T10's generator): the first row where the planner log's
   `n` falls is the first solve, the row before is the larger problem's last solve, residuals are `res_stat`, `res_eq`.
2. Only rows with `published` set count as solves (other rows repeat the last one). "Next" is the next such row, and
   "fail in 1 s" counts status not 0 from the switch to 1 s on. Checked on the raw log: all those rows are `solved`.
3. Run by setting `FREEZE_TWIN` and calling `twin_switches()`, never `main()`, which overwrites the T10 table.

Rows with a repeated time, which `planner_log()` drops, occur in R2605, R2613, R1908 and R2485, none near the switch.

## Proposed sentence

> Across 20 simulated switches with the 1\,m cables, the first published solve of the smaller problem never failed,
> nor did the solve after it or any other in the second after it, and its stationarity residual reached at most
> $7.5\times10^{-4}$, against $8.3\times10^{-4}$ for the larger problem's last solve.

The real-world sentence on 24 to 28 ms can stay beside it, labelled the earlier setup's cables. The residual clause
should then leave the real-world flights, or carry the 7 Oct exception. These numbers are on the 23 Oct tree.

## For the lab chat

- **Re-run after X11.** `configs/batches/x11_A.txt` re-flies `r1m_ta_a1_*` and `r1m_tb_*`, which covers 13 of the 20.
  Not listed: `r1m_ta_a2_*` (3), `r1m_w4_gi_g2_release_orbit` (2), `r1m_dl_d1_second_loss` (2). A final-tree count of
  20 needs those added, and the planned-pull row needs `r1m_ta_a2_*` most.
- **The tool crashes on a dirty tree.** `tools/thesis_freeze_tables.py` line 218 takes `len()` of `dirty_files`, which
  `tools/run_dir.py` writes as a count. Here `manifest()` was wrapped from outside, metadata only, and the tool is
  unedited. A clean tree writes 0, which is why 16 Oct never hit it.
