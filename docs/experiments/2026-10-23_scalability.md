# Card: scalability S, the fleet our planner flies in real time at fewer solver iterations, and the 8-quad chain with the return (23 Oct 2026)

Owner Wesley. DRAFT, written 22 Oct ~17:15, before any bench, replay or twin run of this card. Every bar below is
PROPOSED; the bars freeze when Wesley approves the card, before its first run. Merged into the main tree 22 Oct
(evening) with the review's fixes (scorer windows, the return configs, the lab warm-up). Revised 22 Oct (late) after
its critic: B calibrated on the rig's own flights and barred on the whole tick, the lift, the residual and a release
cell added to C and N, N given a comparator (## Response (22 Oct)). The calibration ran the bench at n = 3 only
(22 Oct ~23:40); no row above n = 3 and no Q, C or N run of this card has been made. One number was seen after Q6
was written and before its bars freeze: a 20-tick smoke of the flags at n = 3 (Q6 below).
Origin: the supervisor's results spec (decisions.md, DECIDED 2026-10-21 (6): "scalability to 9 quads at fewer solver
iterations if quality holds ... the payload bigger, not much heavier"). Push answers 22 Oct: (2) S is 8 quads WITH the
return, 8 -> 4 (-> 2) -> 8, and a 9-quad steady hover only as an extra; (18) the ring R 0.5 m, 0.9 kg, and the
'not heavily worse' bars as proposed; (19) two 12-core lab slots; (20) the speed-ups default in the twin after a smoke
run and identity tests, on the rig after the reviewer. Design: results/analysis/push_1022_designs.json (sim-S),
docs/analysis/push_1022_plan.md.

## 1. Question
How many quads can our planner fly in real time on the rig laptop, at how few SQP iterations a tick, without the plan
getting heavily worse; and can that fleet carry the ring down a chain of unannounced releases and back
(8 -> 4 -> 8)?

## 2. Hypothesis
- **Bench.** About half of a big fleet's tick is overhead, not the solve (profile, 22 Oct: at 9 quads and 1 iteration the
  solve is 55 ms, the logged KKT residual 43 ms, the reference extraction 26 ms). With the two speed-ups (planner option
  `plan_fast_refs`: each drone's references from one mapped call, the residual after they went out) the idle laptop's
  latency path fits 60 ms (p95) at 8 quads with 1 iteration (projected ~48 ms p50 / ~53 p95), 6 at 2 and 4 at 5
  (today 6 / 5 / 4): idle solver throughput.
- **The rig laptop in flight is slower than that bench** (B calibration, §4: the same solve 2.0x at p50 and 3.4x at
  p95, on two flights). Scaled by it, the whole tick (with the residual) fits the 100 ms period for about 3 / 4 / 4
  quads at 5 / 2 / 1 iterations (a projection from the 22 Oct sweep, not a run): 8 quads in real time on the rig
  laptop is expected to FAIL bar B.
- **Quality.** One warm iteration a tick is not heavily worse than five: against the plan converged from the same
  iterate, the cost is within a few per cent (kinematic preview: J/J_conv 1.021 p50 / 1.053 p95 at 4 quads) and the
  references within 1-2 mm; the 4-quad twin flying 1 iteration stays inside its b22 bands and near the 5-iteration
  runs, through the lift and an unannounced release too (where one iteration was once rejected, `planner_solver.py:35`),
  and no tick it publishes has a dynamics residual over the gate's 1e-2.
- **Chain.** At 1 iteration (the count Q and C pick) 8 quads carry the 0.9 kg R 0.5 m ring in the twin and survive four
  unannounced releases down to the square (the survivors' largest gap is 90 deg after every release, the CoG 0.35 m
  inside them) with small tilt peaks, and the four freed quads rejoin to 8 (code design B, `rejoin_departed`, its own
  card and critic).
- **Expected to fail (the critic's physics, exploratory `s8_chain2back` only):** the pair's roll about its own line has
  only the 2 cm `attach_z` lever (both rod ends are ball joints, ~0.2 Hz, undamped in DART); on R 0.5 m a plate moves
  8.7 mm per degree, so more than ~1.7 deg of roll breaks the rejoin's weld window (1.5 cm, 2 cm/s). Prediction: the
  first rejoin onto the pair (onto plate 90 deg) times out. The run records it as `s_metrics`' rejoin lines (REJOIN
  START, then ABORT with its reason, not WELD), the pair stage's tilt peak, and the carriers' throttle p95 there.
- **Not claimed.** Real time on the rig for 8 quads (no 8-quad rig exists, and B expects it to fail): the real-time
  claim is the calibrated bench's rig N_max. The twin's 8-quad worlds run at a real-time cap of 0.4, so the twin plans
  with almost no compute latency: the chain is a twin result about the plan and the physics at 1 iteration, not about
  real time. N -> 2 by unannounced releases (chain step 2b, out of this push); the -> 2 here is soft and announced,
  exploratory.

## 3. The one variable
Three parts, one variable each; everything else as each part's baseline.
- **Bench (B):** fleet size n = 2..10 at 5, 2 and 1 warm iterations, read on the rig-scaled whole tick (§4's
  calibration). A measurement, no arm.
- **Quality (Q, C):** `plan_sqp_iters` 5 -> 1 (2 reported).
  - Replay (Q): `tools/planner_replay.py` arms `base`, `i2`, `i1` on the frozen flights (§5), each tick of each arm
    against the plan converged from the same iterate (`--quality`, tools/plan_quality.py).
  - Twin closed loop (C): `configs/experiments/s4_hover_L100_it5` / `_it1` and `s4_fig8big_h5_v200_L100_it5` / `_it1`:
    the b4 baselines with `plan_fast_refs` on; only `plan_sqp_iters` differs. The release cell (the critic's M2):
    `s4_release9_3915_L100_it5` / `_it1`, c4_release9_L100_tipoff on the M1 geometry (carriers typed 330,210,90,
    QUAD4's plate 9 at 270, world four_rigid_ground_rig_3915_L100): an unannounced RELEASE of plate 9 leaves the
    120 deg Y. The square's release (c4_release9_L100_tipoff itself) leaves a T that drops at 5 iterations already
    (R2319: 76.7 deg, dropped at 46.0 deg), so its it5 / it1 pair would compare two drops.
- **Chain (N):** 8 quads on the R 0.5 m ring at the iteration count Q and C pick (expected 1; B no longer picks it,
  §5); bars against `s4_hover_L100_it1` where one exists (N1), else absolute (no 8-quad baseline exists).
  Claims: `s8_hover_R50_L100_it1`, `s8_chain4back_R50_L100_it1` (the return, through `rejoin_departed`: its own card
  2026-10-23_rejoin_departed and critic; departed_land false, io `tether_tips` for the weld, hover only, each ATTACH
  20 s after the previous rejoin's weld through a `WAIT_LOG` on `REJOIN WELD`, LAND 30 s after the last weld).
  The freed quads stage on their plates' bearings at the formation's reach + 0.3 m (1.3 m on this ring), inside
  +-1.6 m (the 1.75 m limit less 0.15); under LAND or a drop they step out to 1.475 m (`_departed_clear_r`).
  First pass and neighbours (exploratory): `s8_chain4_R50_L100_it1` (8 -> 4, no return), `s8_fig8_h5_v200_R50_L100_it1`
  (a path, size 0.5: at 0.9 or 1.0 the quads 1 m out would pass the +-1.75 m limit), `s6_chain4_R50_L100_it1` (one
  pair fewer; the fallback), `s8_chain2back_R50_L100_it1` (soft -> 2 and back, rejoined as the return). Extra:
  `s9_hover_R53_L100_it1` (9 quads on R 0.53 m).
- **Not a variable:** the speed-ups. They give the references and the logged residuals bit for bit
  (test_horizon_kinematics, test_res_defer; replays R2351 and rig f3: plans, every drone's references, statuses and
  residuals identical per tick with the switch on and off, at 5 and 1 iterations), so every s* config flies them on.
  The values are identical, not the timing: with the switch on the references go out about 12 ms (wall) earlier at 4
  quads (the extraction 10.1 -> ~2 ms, the 4 ms residual after the publish), so the trackers see a younger plan.
  `plan_fast_refs` therefore stays typed in every s* config (on, per config), and the twin's default (sim.yaml) flips
  only between claim tables, never inside one (one code tree per claim table, Wesley's (7)): a table started with
  the default off is finished with it off.
  Planner logs: with `plan_fast_refs` on, `solve_ms` excludes the residual and the new `res_ms` column carries it;
  every table says which compute number it uses.
- **Params file:** the s6/s8/s9 configs and the release cell read `configs/twin/s_chain_pair.yaml`, S's own copy of
  `configs/rig/chain_pair.yaml` (sha1 3ede0b11 on 22 Oct), so a rig edit cannot move a frozen claim. The s4 hover and
  figure-8 cells read `configs/rig/drop_and_land.yaml` / `drop_and_land_hold5.yaml` as their b4 baselines do; their
  registry rows record that file's sha1.

## 4. Baseline
| Part | Baseline | Numbers |
|---|---|---|
| B | results/analysis/solve_scaling_tuf_2026-10-22.json (before the speed-ups: residual inside, node-by-node extraction, no pivot, figure-8 size 0.5; registry row S-BENCH-1022, to append) | idle N_max 4 / 5 / 6 at 5 / 2 / 1 iterations; 8 quads at 1 iteration p95 103.0 ms; 9 at 1 iteration 134.8 ms; no failed solve |
| B (the rig factor) | the B calibration below (registry row S-CALIB-1022, to append) | the rig laptop in flight over the bench: 2.0x at p50, 3.4x at p95 (the card uses 2.1 / 3.5) |
| Q | each flight's `base` arm (5 iterations, as flown) | in the replay's own table |
| C | b4_hover_L100_ema R2348 / R2351 | path RMSE 1.3 / 1.4 mm, tilt peak 0.21 / 0.21 deg; b22 band <= 22.0 mm, <= 3.21 deg |
| C | b4_fig8big_h5_v200_L100_ema R2352 / R2355 | clock-free RMSE 12.4 / 11.9 mm, tilt peak 1.07 / 1.05 deg; band <= 33.0 mm, <= 4.06 deg |
| C (the claim's own base) | s4_*_it5, re-flown on the S tree, 2 each (the release cell's too) | (to fly) |
| C (release, for scale) | Tier B G120 (0.55 m cables, 4 quads, the same Y) | release tilt peaks 4.6-6.2 deg |
| N1 | `s4_hover_L100_it1`, C's two runs on the same tree | (to fly): N1's comparator |
| N | Tier B card (0.55 m, 4 quads, twin) | release tilt peaks 4.6-6.2 deg at a 120 deg gap, 17 at 150, >= 60 at 180 |
| N | the carry card's values (T7t) | 25 mm, 6 deg |

### B calibration (22 Oct ~23:40, the critic's B1; no new flights)
The rig flights RIG-1022-h3 (hover) and -f3h (figure-8 size 1.0), 3 quads, 5 iterations, flown with the residual
inside `solve_ms` and the rig's 60 ms solve budget, replayed on the rig laptop (tuf) idle: `tools/planner_replay.py
--flight <h3>,<f3h> --arms base,i1,i1+v --q-source auto --flown-pivot` (per tick against the logged `solve_ms`: the
replay's new timing table), and the bench at n = 3 as the rig flew it (`solve_scaling.py --n 3 --iters 5 2 1
--pivot-dz 0.04 --traj-radius 1.0 --ring-inertia annulus`, no `--fast`). Warm planner ticks only; the solve is the same
call in all three (`PlannerSolver.solve_horizon`).

| Flight | warm ticks | rig in flight p50 / p95 (ms) | the same ticks replayed idle | per tick rig / replay p50 / p95 | rig / bench p50 / p95 |
|---|---|---|---|---|---|
| h3 | 264 | 36.7 / 67.8 | 18.5 / 19.4 | 1.97 / 3.65 | 1.86 / 3.37 |
| f3h | 573 | 40.4 / 69.4 | 19.2 / 20.6 | 2.10 / 3.59 | 2.04 / 3.45 |
| pooled | 837 | 38.8 / 69.0 | 19.0 / 20.4 | | 1.96 / 3.43 |
| bench n = 3, 5 iterations, residual inside | 200 | | 19.8 / 20.1 | | |

- The flights' own states cost the solver nothing extra (replay / bench 0.94-1.02): the factor is the laptop in
  flight (the rig stack, 3 trackers, mocap, the bag, RViz), not the states. On the ticks the budget cut did not touch
  (253 / 536) the per-tick ratio is 1.95 / 3.41 and 2.02 / 3.38.
- **Rig factor, frozen with the card:** k95 = 3.5 and k50 = 2.1 (the larger flight's rig / bench ratio, rounded up).
  It is a lower bound: the rig's budget cut held 11 (h3) and 37 (f3h) slow ticks to 3-4 iterations, and an 8-quad rig
  would run 8 trackers, not 3.
- **Where it comes from (a lead, untested on the rig):** the laptop has 8 P-core threads (4.8 GHz, CPUs 0-7) and 4
  E-cores (3.6 GHz, 8-11). h3 replayed pinned to one CPU: P-core 18.4 / 18.9 ms, E-core 47.7 / 48.9 ms (2.6x). In
  flight no tick came under 20 ms and h3's mode was 40-45 ms: the planner probably ran partly on E-cores.
  `results/analysis/s_calib_replay_cores_2026-10-22.json`.
- **Cross-check on the whole tick:** scaled by 3.5, the bench's whole tick as the rig flew it at n = 3 (residual
  inside, node-by-node extraction with the pivot: p95 32.1 ms) is 112 ms. The rig's own tick spacing (row stamp minus
  `solve_ms`, i.e. near each tick's start) had p95 116 / 117 ms, 10-12 % of ticks over 110 ms and none over 150 ms; the
  10 Hz loop lost no ticks (h3: -0.07 s drift over 352). So the rig at 3 quads and 5 iterations already runs about at
  its period, as the scaled bench says.
- Files: `results/analysis/s_calib_replay_2026-10-22.json` (per flight and arm, `timing` and the pooled
  `calibration`), `s_calib_bench_n3_resin_2026-10-22.json`, `s_calib_bench_n3_fast_2026-10-22.json` (the card's bench
  flags at n = 3: whole tick p95 21.5 / 12.3 / 8.9 ms at 5 / 2 / 1).

## 5. Pass / fail numbers (FROZEN 23 Oct ~07:25 on Wesley's approval, decisions.md DECIDED 2026-10-23 (5): bars as written; 8 quads reported "twin only, not real time". Stamped 11:18; no claim cell had flown: R2464, R2587, R2588, R2597 are exploratory)
**B, the bench.**
- Command: `python3 tools/solve_scaling.py --n 2 3 4 5 6 7 8 9 10 --iters 5 2 1 --fast --pivot-dz 0.04
  --traj-radius 1.0 --ring-inertia annulus --rig-factor 3.5 --tick-budget 100` (200 warm ticks after 20; figure-8 size
  1.0 at 0.2 m/s; 1 m cables at 60 deg; 0.7 m between quads; 0.86 kg; `--res-every 1`, the residual every tick as
  Wesley set `plan_res_every`; no `--quality`, no `--noise`).
- The bench's ring is sized by that spacing (R 0.391 m at n = 8, 0.503 m at n = 9, at least 0.225 m) and weighs
  0.86 kg with the annulus inertia of that radius: not the flown S ring (R 0.5 m, 0.9 kg, Ixx/Izz 0.113/0.226).
  Geometry and mass are runtime parameters or constants of the same OCP, so the solve time is not expected to
  depend on them; the table says which ring it timed.
- On the rig laptop (tuf), idle: no Gazebo, no lab sync, no other job, `uptime` load under 0.5 before it starts. A
  lab-container copy is reported, not barred.
- Latency = the solve plus the mapped extraction with the 4 cm pivot (when the references go out); whole tick
  (`tick_ms`) = latency plus the residual (`finish_tick` runs in the same timer callback on the node's single-threaded
  executor, `planner_node.py` `_plan`, so a tick past the 100 ms period delays the next).
- **N_max(it), the bar:** the largest n with no failed solve whose rig whole tick, 3.5 x `tick_ms` p95 (the rig factor,
  §4), is <= 100 ms. The tool prints and stores it (`n_max.rig_tick`).
- Reported beside it, not barred: the idle N_max on latency p95 <= 60 ms (the card's first bar: idle solver
  throughput) and on the idle whole tick <= 100 ms, 2.1 x the p50s, and 3.5 x the latency p95 against the rig's 60 ms
  solve budget.
- Expected (a projection from the 22 Oct sweep plus the speed-ups' measured n = 3 deltas): rig N_max 3 / 4 / 4 at
  5 / 2 / 1 iterations (5 at 1 iteration only if its whole tick p95 comes in under 28.6 ms).
- B no longer picks the chain's iteration count: the chain flies at 1 iteration if Q and C hold, else 2, else 5.
- **Falsifies the 8-quad real-time claim:** rig N_max(1) < 8 (expected). S's real-time result is then the rig N_max
  at each count ("our planner flies N_max quads in real time on the rig laptop at k iterations"), and the 8-quad
  chain stays a twin result, labelled not real time. (This replaces "fly the chain with 6 quads at the count whose
  N_max >= 6", which no count is expected to reach: needs Wesley.)
- The factor is defensible (two flights within 3 %, and the replay matches the bench), so the claim keeps its rig
  wording rather than becoming "idle solver throughput". A later rig flight's factor is reported beside it; the runs
  are not re-scored.

**Q, the replay quality.**
- Flights, chosen from the design before any replay looked at them; the tool was debugged on R2351 and rig f3, outside
  this list:
  - twin R2348 (b4 hover, ema), R2352 (b4 figure-8 size 1.0), R2356 (b4 spinny line), R2343 (b3 figure-8), R2360 (b2
    figure-8);
  - rig RIG-1022-h3, -f3h, -ls3c, -f2slope (results/rig/2026-10-22/<run>_logs).
- Command: `python3 tools/planner_replay.py --flight <the nine> --arms base,i2,i1 --quality --q-source auto
  --flown-pivot --summary-json results/analysis/s_quality_replay_<date>.json`. Warm planner ticks only; a tick whose
  converged plan failed is left out and counted.
- Bars, per flight, for `i1` (`i2` reported against the same bars):
  - Q1 cost, paired with Q2 tick by tick: J/J_conv p50 <= 1.10 and p95 <= 1.50, where a tick whose arm residual is
    over Q2's 1e-3 counts as a miss whatever its ratio (`get_cost` on an iterate that breaks the dynamics can come out
    below J_conv; tools/plan_quality.py `cost_ratio_paired`, the replay table's 'paired' column). The kinematic preview
    predicted 1.021 / 1.053; the bars stay as Wesley approved them ((18)), the predicted values reported beside;
  - Q2 the arm's dynamics residual: p95 <= 1e-3;
  - Q3 references (the drone position the tracker flies, max over drones): node 1 p95 <= 5 mm, node 5 p95 <= 10 mm;
  - Q4 node-0 thrust feed-forward: p95 <= 0.3 m/s^2;
  - Q5 failed solves (status other than 0 and the held-back 20): no more than the `base` arm's on that flight.
- Q6, n = 8 (every Q flight has n <= 4): `python3 tools/solve_scaling.py --n 8 --iters 5 2 1 --fast --pivot-dz 0.04
  --traj-radius 1.0 --ring-inertia annulus --quality --noise 0.01
  --out results/analysis/s_quality_bench_n8_<date>.json`, untimed (the converged solves run between its ticks; its
  times are not read). The noise moves each measured state 1 cm (ring position) and 10 cm/s (its velocity) off the
  plan's node 1, the order of the rig's ring error (h3: 2.4 cm 3-D RMSE), so every warm start begins off the plan.
  Bars: `i1` inside Q1-Q4 (`i2` reported). Seen before the bars freeze: a 20-tick smoke of these flags at n = 3,
  `i1` (testing the tool, 22 Oct), had res_eq p50 9.2e-4 / p95 5.2e-3 (9 of 20 ticks over 1e-3), node-1 reference
  p95 6.6 mm and feed-forward p95 0.63 m/s^2, i.e. outside Q2-Q4; the noise level was chosen before it and is left as
  it was (Wesley's to change, for the bar as a whole).
- **Confirms:** Q1-Q5 on all nine flights and Q6. **Falsifies:** any bar missed on any flight or in Q6; then `i2` is
  the candidate if it passes them all, else 5 iterations, and the chain and the rig N_max are read at that count.

**C, the twin closed loop** (2 each of the six s4 configs: hover, figure-8 and release, at `it5` and `it1`,
interleaved; scored with `tools/n2_bands.py --set b22`, clock-free path error in the cell's window, and
`tools/s_metrics.py` for the planner, the lift and the release):
- C1 every `it1` run inside its b22 band (hover <= 22.0 mm and <= 3.21 deg; figure-8 <= 33.0 mm and <= 4.06 deg);
- C2 every `it1` run's path error <= max(1.25 x the two `it5` runs' mean, that mean + 3 mm);
- C3 every `it1` run's tilt peak <= the two `it5` runs' mean + 1 deg;
- C4 no abort, drop, envelope fault or failed planner solve; late planner or tracker ticks <= 1 % (a tick more than
  1.5 periods after the last), else the run is void and flown once more.
- C5 no published planner tick (`published` 'solved') with a logged res_eq > 1e-2, and none published without one
  (`s_metrics` `published_res_over`, `published_res_missing`): at 1 iteration the budget cut never fires, so the
  publish gate never reads a warm tick's residual and this is read from the log afterwards.
- C6 the lift (the window C1-C3 leave out: from the ring's reference 1 cm above its start to 2 s after the top,
  `s_metrics` `lift`): every `it1` run's tilt peak <= the two `it5` runs' mean + 1 deg (its position error reported).
- C7 the release cell (`s4_release9_3915_L100_*`; C1 and C2 do not apply): every `it1` run with no drop, the planner at
  3 quads within 2.0 s of the RELEASE, a release tilt peak (from the 5 s mean before it) <= the two `it5` runs' mean
  + 1 deg, and a settled tilt (the 5 s before LAND) <= their mean + 1 deg.
- **Confirms:** C1-C7 in both repeats of every cell. **Falsifies:** any of C1-C3 or C5-C7 missed in a valid run.
- The `it5` pair is also `plan_fast_refs`' twin smoke run: inside its b22 bands with C4 held, the twin's default may
  flip (sim.yaml, Wesley 22 Oct (20)), but only between claim tables, never inside one (§3, M3); the rig's waits for
  the reviewer.

**N, the 8-quad chain** (`s8_hover` x2 and `s8_chain4back` x2 on the final tree; `tools/s_metrics.py`):
- N1 steady 8-quad hover (`s8_hover`, and the stage before the first release), from 2 s after the top (the ring
  reference within 1 mm of its highest, as the b22 bands' hover window) to the first event or LAND; the lift is not in
  it. Against `s4_hover_L100_it1` (C's two runs on the same tree, scored by `s_metrics`' n1, the same window): ring
  position RMSE <= 2 x their mean + 3 mm, and tilt peak (the tilt vector's magnitude) <= 2 x their mean + 1 deg; and
  never above the first draft's absolute 25 mm / 6 deg;
- N2 each unannounced release: the planner at the new size within 2.0 s; tilt peak (the tilt vector's departure from
  its mean over the 5 s before) <= the release cell's `it1` runs' mean + 1 deg (C7: four quads, a 120 deg gap; the
  chain's releases leave 90 deg gaps and more survivors) and <= 15 deg; the settled tilt (mean over the 5 s before the
  next event) <= 3 deg; no drop;
- N3 each rejoin: the planner one quad bigger within 20 s of the ATTACH (the next ATTACH comes 20 s after its weld);
  tilt peak <= 10 deg; settled <= 3 deg;
- N4 back at 8: the planner at 8 after the last rejoin, and from 2 s after its resize to 8 until LAND within N1's bars;
  **REVISED 23 Oct 14:32 (Wesley; before any 8-quad claim run flew; applies to the claim runs only):** N4's window opens 2 s after the last REJOIN_BLEND_OVER (the rejoin's 10 s blend, added on 23 Oct after this card was written, put the last fold-in inside the original window; the fold-in is scored by N3 and the rejoin card's bars). The original window (2 s after the resize to 8) is reported beside it (`tools/s_metrics.py`: `n4`, `n4_resize`). Exploratory R2597 (seen before the revision): revised 6.3 mm / 0.89 deg, original 19.3 mm / 1.65 deg, against <= 5.72 mm / <= 1.44 deg.
- N5 validity: late planner and tracker ticks <= 1 % (else void, flown once more); no failed planner solve.
- N6 no published planner tick with res_eq > 1e-2, and none published without one (as C5).
- Reported with N, not barred (no 8-quad 5-iteration run to compare with): the lift window's tilt peak and position
  error (as C6) beside `s4_hover_L100_it1`'s; per stage, the carriers' throttle p95 and the fraction of samples at the
  0.8 cap (`s_metrics` `thr_p95`, `thr_cap_frac`; the pair stage of `s8_chain2back` is the one the critic flagged: the
  rig pair at 0.43 kg sat at the cap for 7-8 % of its lift, RIG-1022-h2); the achieved RTF and the late-tick fractions
  first, before any other number of an 8-quad run.
- **Confirms:** N1-N6 in both repeats. **Falsifies:** a drop, an abort, or any bar missed in a valid run.
- **Re-fly on any N miss (one variable):** that cell once more at `plan_sqp_iters` 5, with the world's real-time cap
  lowered so the planner's tick takes the same sim time as in the `it1` run (cap x the bench's `tick_ms` p50 at 1
  over 5 iterations at that n: about 0.4 -> 0.14 at 8 quads). The plan's age in sim time is then held and the
  iteration count is the one thing that changes: a miss that goes away is the one iteration's; one that stays is the
  fleet, the ring or the chain's. The lower-cap world is a new twin world SDF (needs Wesley's word when it comes).
- `rejoin_departed`'s own card and critic set the bars of its mechanism; N3 is the chain's outcome only.
- The neighbours (`s8_chain4`, `s8_fig8`, `s6_chain4`, `s8_chain2back`, `s9_hover`) are exploratory rows, one each, a
  falsifier in each row: an abort, a drop, a failed solve, more than 1 % late ticks, or a release over N2's tilt bar.

## 6. Repeats
2 in the twin for every claim cell (C, N); the bench and the replays are deterministic offline runs, one each.

## 7. Cost
- Offline, laptop idle: the bench (n = 2..10 x 3; the first run builds each size, n = 10 ~15 min) and the replays
  (9 flights x 3 arms; each warm tick also solved to convergence, so tens of minutes).
- Offline: Q6, the n = 8 quality pass (3 counts x 220 ticks, each warm tick also converged: ~20 min).
- Lab PC, 4 slots: C, 12 runs (~3 h; the release cell's 4 added).
- Lab PC, two 12-core slots (`tools/remote/lab_batch.py --slots 2 --cpus 12 --wall-cap-min 60 --warm-from-config`,
  sizes 2..8 prebuilt in each slot first, ~12 min cold): the exploratory first pass (`s8_hover`, `s8_chain4`,
  `s8_fig8`, `s6_chain4`), then the claims (`s8_hover` x2, `s8_chain4back` x2), `s8_chain2back` and `s9_hover` if the
  evening allows: about 10 runs of 8-25 min. The 8-quad runs give way first if the lab is short.
- Laptop runs (one at a time) only as the lab's backup, with `MDC_ACADOS_ROOT=generated/s_ring` so the 0.9 kg builds
  never evict the shared cache.
- A lab slot's cache holds one planner solver per fleet size, so S's 0.9 kg builds of n = 2..8 replace the slot's
  0.86 kg ones. `lab_batch.py` therefore warms every size a config's planner resizes through (`prebuild_planner.py
  --from-config`) for any config whose planner resizes, with or without `--warm-from-config`: a later 4-quad detach or
  chain batch rebuilds its 2 and 3 in the warm-up (9-40 s a size), not in the run's ready window.

## 8. New code?
Added after the critic (22 Oct, late), all offline or config: `planner_replay.py`'s timing table (each tick's
`iters`, the logged `solve_ms` against the replayed time); `solve_scaling.py --rig-factor --tick-budget` (N_max on the
rig-scaled whole tick, stored as `n_max`); `plan_quality.py` `cost_ratio_paired` (Q1 with Q2); `s_metrics.py`'s lift
window, the published ticks over the residual bar, the carriers' throttle per stage and the rejoin lines (tests:
tools/test/test_s_metrics.py, tools/test/test_s_offline.py); `configs/twin/s_chain_pair.yaml`; the release cell's two
configs.

Yes, all of it offline except the switch, which is in the flight loop and off by default.
- **Flight loop, behind `plan_fast_refs` (default off everywhere):** `planner_solver.kinematics_funcs` /
  `horizon_map` / `PlannerSolver.horizon_kinematics` (the existing per-node functions, mapped), the residual after the
  publish (`defer_res`, `finish_tick`), `geometry.centre_from_pivot_many`. `plan_res_every` (default 1, as before:
  per-tick residuals kept, the push plan's Q22, Wesley's answer (20)).
  Gated as the plan's section 5 says, no critic: bit-identity tests (2, 3, 8 quads, rod model on and off, pivot 0 and
  -0.04, |a| ~ 0 and thrust along the heading), the residual's state-machine tests, identical replays of a twin and a
  rig flight, the twin smoke run (C's `it5` pair), the reviewer before the rig. The bit identity rests on numpy /
  OpenBLAS rounding measured on the rig laptop only: `test_horizon_kinematics.py` and one `--ref-hash` replay run
  inside a lab container before the switch counts as 'not a variable' there or the twin default flips.
- **Also in the flight loop, behaviour unchanged for every config flown so far:** the ring inertia as parameters
  (`load_ixx` / `load_izz`, 0 = as before), the prebuilt sizes reaching `min_survivors` (identical for 4 quads or
  fewer), the freed drone's landing clearance growing past the 1 m rig ring's reach (1.2 m for the rig ring, 1.475 m
  for S's ring; the 1m-defaults helper `_departed_clear_r`, which S's own copy was merged onto).
- **Twin only:** the R 0.5 m (6, 8 quads) and R 0.53 m (9 quads) worlds, drone models 4-8, `--plates` and
  `--real-time-factor` in the world generator (defaults byte for byte).
- **Offline:** solve_scaling's flags, planner_replay's quality mode, `tools/plan_quality.py`, `tools/s_metrics.py`,
  `prebuild_planner.py --from-config`, lab_batch's `--warm-from-config` and 12-core slots.
- **If it fails:** a speed-up that is not bit-identical on any machine, or a smoke run outside its bands, keeps the
  switch off and the code is recorded and deleted (the working rules). The S result then uses the slower bench and
  says so.

## 9. Risks
- **RTF and the release detector (the critic's minor).** Eight trackers (~0.9 core each disarmed, reference §10) and
  the planner share a 12-core slot. The detector's freshness gate `DETECT_STALE_S` (0.1 s, `dissipative_node.py:100`,
  used from `:1799`) compares wall-clock (`time.monotonic`) ages, while the drones' poses come at 60 Hz in sim time
  (the worlds' pose publishers): below an achieved RTF of about 0.17 a pose interval passes 0.1 s wall, the detector
  discards samples, and an unannounced release goes unseen (a drop that is the twin's load, not the plan's). Every
  8-quad run reports its achieved RTF (`metrics.json` mean_rtf) and late-tick fractions first; a run under 0.25 is
  flagged, its detections checked against its RELEASE events, and a release missed under a low RTF voids the run (flown
  once more on a lighter slot), not the claim.
- **The rig factor.** It is measured at 3 quads with 3 trackers; an 8-quad rig would run more processes, and the rig's
  budget cut held its slowest ticks down, so the factor is a lower bound and the rig N_max an upper one.
- **End-stamped planner rows.** The planner's log row is stamped after the tick (`_log_tick`), so jitter in the solve
  shows as 'late' rows: on the rig at n = 3, 2.8 % (h3) and 6.0 % (f3h) of planner steps were over 0.15 s while the
  start-corrected spacing (stamp minus `solve_ms`) had none. In the twin a wall-clock jitter is scaled by the RTF in sim
  time (x 0.4), so the 1 % late bar is not expected to trip on jitter; if it does, the start-corrected spacing is
  reported beside it before a run is voided.

## Critic
*The project's critic, 22 Oct 2026 (read-only review of the card and the merged code, before any run).*

**1. Tried before?** Two warm iterations were flown in the twin. R1983 and R1985 halved the solve time, and the path error stayed inside the base spread. Combined with the leads, the two-iteration runs R2151 and R2169 failed N5. One iteration has never been flown: the code records it as rejected for the slack-to-taut lift (`planner_solver.py:35`, `:321`; corner-cutting card line 173).

**2. Simplest unaddressed explanation.** The real cost is CPU contention plus warm starts from real, noisy states on a flying laptop, not idle overhead. RIG-1022-h3 logged a solve time of p50 41.5 / p95 68.7 ms at n=3 with 5 iterations. The bench gives 20.3 / 20.6 ms for the same case (`solve_scaling_tuf_2026-10-22.json`). The speed-ups trim idle overhead and leave that factor of 2 to 3. The generator's own comment agrees: an 8-quad tick takes "100-220 ms wall in the twin" (`make_carry_worlds.py:91-92`).

**3. Falsifier vs bars.** The thing that would falsify real time is the rig's in-flight tick at n=8 running past the 100 ms period. Bar B cannot see that, for three reasons: it is measured idle, its plant is the plan's own node 1 with no noise (`solve_scaling.py:109`), and the whole tick is only reported, not barred. Bars Q, C and N leave out the lift and the releases, which are exactly where one iteration was rejected.

**4. What it breaks if it works.**
- **Twin default flip:** flipping `sim.yaml` mid-push splits the R/U/C tables across a timing change, against Wesley's (7) one frozen tree. The twin would also fly a planner timing the rig does not, against the locked rule that the rig twin is the simulation.
- **One iteration on the rig:** the publish gate would be blind on warm ticks (see M2).
- **Untouched while the switch is off:** the network, the gate and the hardware path.

**5. Asked for?** Yes: 21 Oct (6), and 22 Oct (2) and (18)-(20).

### Findings
**BLOCKER B1: the bench is not the rig laptop.** N_max comes from an idle, noise-free kinematic bench, and the rig is 2 to 3.3 times slower at n=3. The projected 53 ms p95 at 8 quads and 1 iteration has 7 ms of margin, which cannot absorb that. Fix with a calibration step before N_max is read. Replay h3 and f3h (already in Q) on the idle laptop and compare their per-tick `ms` with the logged `solve_ms` and with the bench at n=3. Then either scale N_max by the measured factor, or rename the claim "idle solver throughput" and drop "real time on the rig laptop".

**M1: the whole tick is unbarred.** `finish_tick` runs inside the same timer callback (`planner_node.py:1381-1384`), and the executor is single-threaded (`rclpy.spin`, line 2126). At 8 quads and 1 iteration the latency (~53 ms) plus the residual (~35-43 ms) comes to about 90-100 ms on an idle machine, so on the rig the 10 Hz timer would overrun. Either require whole-tick p95 ≤ 100 ms in N_max, or fly S with `plan_res_every` 0 or 10. The second option needs Wesley's word against his (20)/Q22.

**M2: one iteration is never tested where it matters.**
- C and N score from the top + 2 s (`s_metrics.py:122`), so the lift is not in any bar.
- `planner_replay` runs at a fixed n (`:341-475`), so Q cannot replay a release.
- At 1 iteration the budget cut can never fire (`:339`), so the publish gate never reads a warm tick's residual (`:346`), and a bad step goes out unchecked.
- Fix: add to C4/N5 "no published tick with res_eq > 1e-2" (already logged). Add the lift-window tilt peak, it1 against it5. Add one 4-quad release cell (e.g. `c4_release9_L100_tipoff`) at it5 and it1.

**M3: "not a variable" holds for the values only.** The references go out about 12 ms wall earlier at 4 quads: the extraction drops from 10.1 to about 2 ms, and the 4 ms residual moves after the publish. That changes the plan age the trackers see. Keep the switch in the s* configs, and flip the twin default only between claim tables.

**M4: N has no comparator.**
- N1's 25 mm is about 20 times the 1.3 mm the 4-quad twin hovers at.
- N2's 15 deg is 2.5 times Tier B's 4.6-6.2 deg, at a wider gap.
- So N cannot falsify "small tilt peaks" or "not heavily worse".
- Fix: make `s4_hover_it1` N1's comparator (for example, at most 2 times its value + 3 mm). On any N miss, re-fly that cell once at it5 with a lower RTF cap, so the miss can be pinned on one variable.

**Minor**
- Q1's 1.10 / 1.50 is 5 to 10 times looser than the predicted 1.021 / 1.053, so it tests a much weaker claim. Also, `get_cost` on an iterate that breaks the dynamics can come out below J_conv (`plan_quality.py:93`). Pair Q1 with Q2 tick by tick.
- Every Q flight has n ≤ 4. Add an untimed `solve_scaling --quality --noise` pass at n=8.
- The s* configs read `configs/rig/chain_pair.yaml` (line 49), so a rig edit would change a frozen claim. Copy the file or record its hash in each row.
- The B baseline JSON has no registry row.
- 8 trackers (~0.9 core each when disarmed, reference §10) share a 12-core slot. If the RTF falls well below the 0.4 cap, the wall-clock gate `DETECT_STALE_S` 0.1 s (`dissipative_node.py:100`, used at `:3368`) starts dropping samples.

### Can it physically fly?
- **Thrust:** each quad carries 0.11 kg of ring at 8 quads, 0.225 kg in the square (about b4) and 0.45 kg in the pair. The rig pair at 0.43 kg hit the 0.8 cap for 7-8 % of its lift (RIG-1022-h2). S's pair is hover only, but read throttle p95 there.
- **Spacing:** pivots sit 0.765 m apart at r 1.0 m, and the plates are 0.383 m apart. The staging points are 0.92 m from the neighbours. The figure-8 at size 0.5 reaches 1.5 m, inside the 1.75 m limit.
- **The pair's roll (s8_chain2back):** both ends of each rod are ball joints (`generate_rigid_world.py:174`, `:185`). Roll about the pair's line therefore has only the 2 cm `attach_z` lever, which gives about 0.2 Hz, and DART does not damp it.
  - On R 0.5 m a plate moves 8.7 mm per degree, so more than 1.7 deg of roll breaks `REJOIN_WELD_M` (1.5 cm) and `REJOIN_SLOW_MPS` (2 cm/s) (`dissipative_node.py:88-92`).
  - Prediction: the first rejoin onto the pair, onto plate 90 deg, times out.

### Runs that expose each risk
| Risk | What exposes it |
|---|---|
| B1 | Q's h3/f3h base replays, timing only. No new runs |
| M1 | The bench's `tick_ms` column |
| M2 | Lift metrics from the existing `s4_*` floor starts, plus one c4 release pair |
| CPU / RTF | `s8_hover`'s first pass: report the achieved RTF and the late-tick fractions first |
| Pair roll | `s8_chain2back`: planner tilt during the pair stage and its REJOIN lines |

CRITIC: rewrite the card | the 8-quad bar is measured on an idle bench, and the rig's own in-flight log shows that bench is 2-3 times too optimistic. The whole tick and the lift and release transients are not barred.

## Response (22 Oct)
*22 Oct, late, before any run of this card. Every bar is still PROPOSED.*

| Finding | Fate | What changed |
|---|---|---|
| B1 the bench is not the rig laptop | fixed | Calibrated without new flights (§4, B calibration): h3 and f3h replayed idle on the rig laptop and the bench run at n = 3 as the rig flew it. The rig in flight is 1.86 / 2.04x the bench at p50 and 3.37 / 3.45x at p95 (pooled 1.96 / 3.43); the replay matches the bench (0.94-1.02), so the factor is the laptop in flight, not the states. Bar B now reads N_max on 3.5 x the whole tick p95 (§5); the idle numbers are reported as idle solver throughput. Expected: rig N_max 3 / 4 / 4 at 5 / 2 / 1, so the 8-quad real-time claim is expected to fail. |
| M1 the whole tick is unbarred | fixed | N_max is barred on the whole tick (`tick_ms`: solve + mapped extraction + residual) <= 100 ms, rig-scaled; `plan_res_every` stays 1 as Wesley set it (the bench runs `--res-every 1`). |
| M2 one iteration untested where it matters | fixed | C5 / N6: no published tick with a logged res_eq > 1e-2 (and none without one). C6: the lift window's tilt peak, it1 against it5 (reported for N). C7: a 4-quad release cell at it5 and it1, `s4_release9_3915_L100_*` (both pass `--check`): c4_release9_L100_tipoff on the M1 geometry, because the square's release drops at 5 iterations already (R2319). `s_metrics.py` gained the lift window and the residual check (tests). |
| M3 'not a variable' holds for the values only | accepted | §3 says it: the references go out ~12 ms earlier at 4 quads; `plan_fast_refs` stays typed in every s* config, and the twin default flips only between claim tables, never inside one. |
| M4 N has no comparator | fixed | N1 against `s4_hover_L100_it1` (position RMSE <= 2 x its mean + 3 mm; tilt peak <= 2 x its mean + 1 deg, the same form, proposed); N2's tilt peak against the release cell's it1 mean + 1 deg (proposed). One-variable re-fly on any N miss: the cell at 5 iterations with the real-time cap lowered so the planner's tick keeps its sim time (the cap-lowered world needs Wesley's word when it comes). |
| Minor: Q1 loose; get_cost below J_conv | fixed (pairing) / needs Wesley (values) | Q1 is paired with Q2 tick by tick (`cost_ratio_paired`: a tick over 1e-3 residual is a miss). The 1.10 / 1.50 values stay as Wesley approved them ((18)); tightening toward the predicted 1.021 / 1.053 is his call. |
| Minor: every Q flight has n <= 4 | fixed | Q6: an untimed `solve_scaling --quality --noise 0.01` pass at n = 8 (the flags existed), barred with Q1-Q4. A 20-tick smoke at n = 3 already sits outside Q2-Q4 at that noise (§5, disclosed): one warm iteration from a state 1 cm / 10 cm/s off the plan leaves a residual. |
| Minor: the s* configs read the rig's chain_pair.yaml | fixed | `configs/twin/s_chain_pair.yaml` (a copy); the s6/s8/s9 configs and the release cell read it. The s4 hover / figure-8 cells keep the rig's drop_and_land files their b4 baselines read; their rows record the sha1. |
| Minor: the B baseline JSON has no registry row | accepted | Rows S-BENCH-1022 (the baseline) and S-CALIB-1022 (the calibration) written for appending; not appended here. |
| Minor: RTF and DETECT_STALE_S | accepted | §9: the achieved RTF reported first; under 0.25 the run is flagged and a release missed under a low RTF voids the run, not the claim. |
| Twin default flip mid-push | accepted | As M3. |
| Physics: thrust in the pair | accepted | The carriers' throttle p95 and the fraction at the 0.8 cap per stage (`s_metrics` `thr_p95`, `thr_cap_frac`), the pair stage of `s8_chain2back` reported. |
| Physics: the pair's roll | accepted | §2 states the prediction as an expected failure of the exploratory `s8_chain2back`: the first rejoin onto the pair (plate 90 deg) times out. The run records its rejoin lines (`s_metrics` `rejoins`: START then ABORT with its reason), the pair stage's tilt peak and the carriers' throttle there. |

**Needs Wesley:**
- The chain's claim once B fails at 8 (expected): keep the 8-quad chain as a twin result labelled not real time, with
  the rig N_max as S's real-time result (recommended), or fly the chain at the rig N_max (about 4 quads: no chain).
- The rig laptop's E-cores (§4): h3 replayed on an E-core takes 2.6x a P-core, and the rig's in-flight times sit
  between the two. Pinning the planner to the P-cores on the rig (`taskset`) could shrink the factor; it is a rig
  launch change, untested.
- Whether Q1's values tighten toward the predicted ones, and Q6's noise level (1 cm / 10 cm/s, chosen before a
  20-tick n = 3 smoke that missed Q2-Q4 at 1 iteration).

## Results, C (23 Oct 14:12, lab batch 17: R2700-R2711)
*Scored against §5 C as frozen (~07:25, stamped 11:18); nothing re-set. C1-C3 use `tools/n2_bands.py`'s b22 metric
(`path_metrics_clock_free`), which reproduces §4's baselines to the digit (R2348 / R2351 1.34 / 1.38 mm, R2352 / R2355
12.38 / 11.89 mm). C4-C7 use `tools/s_metrics.py`'s `score()`, run with the planner and tracker logs moved onto the
runner's clock and clipped at run.csv's last row (the two scorer faults below). Registry rows R2700-R2711, plus R2703c,
which corrects R2703's note (no reviewer yet). The numbers, aligned and as the scorer prints them:
`results/analysis/s_c_twin_2026-10-23.json`. The wrapper is the scratchpad's `s_c_score/score_c.py`.*

**Validity: all 12 runs valid, none void.**
- One code tree: be34f95 + patch 537551ba. One host: the lab PC, 4 slots, RTF 0.22-0.27.
- Runner PASS in all 12. No abort, drop, envelope fault, or failed or unconverged solve. The param read-back misses
  nothing required.
- Each run flew its arm:
  - the planner's read-back shows `plan_sqp_iters` 5 or 1, `plan_fast_refs` true and `plan_res_every` 1;
  - the launch log says "n=4: k SQP iterations per tick";
  - `sqp_calls` is k on every planner tick (the twin never cut the budget);
  - `SIM_MOCAP_VEL_FILTER` is ema ([MOCAP0-3] lines).
- Params files (sha1; each run's copy matches): `drop_and_land.yaml` 4803b0f0, `drop_and_land_hold5.yaml` a26f3836,
  `s_chain_pair.yaml` 94df7942.
- Each cell's it5 pair flew first and its it1 pair next, overlapping on the 4 slots. So the arms are interleaved by
  cell, not run by run.

**Two faults in `tools/s_metrics.py` as it stands.** These affect measurement only; no bar was touched.
1. *Clocks.* The scorer reads the planner log's `sim_time` (/clock) against run.csv's and events.csv's `t`. Those
   start at the manifest's `t_ready_sim`, so every planner time it compares with an event is off by t_ready. That
   shifts the resize after an event, the stages' n, tensions and throttle, and N4's window. The lift-off alignment
   the other twin scorers use (detach_tensions, drop_metrics, release_transient) agrees with t_ready to 0.05 s.
2. *Clip.* The scorer reads the node logs to their end, past run.csv's last row, into the shutdown after LANDED.
   Every late step that pushes a run over 1 % lies outside run.csv's range: after its last row, except one in R2703
   that comes before its first.
- The fix is not made here: shift the node logs by `t_ready_sim`, clip them to run.csv's range, and add a test with a
  non-zero offset. N must not be scored until it is in, because N2's 2.0 s, N3's 20 s and N4's window all read the
  planner's n against events.csv.

| run | arm | t_ready_sim (s) | late planner, as it stands / run window (%) | late tracker, as it stands / run window (%) | resize after RELEASE, as it stands / runner clock / FLEET RESIZED line (s) |
|---|---|---|---|---|---|
| R2700 | it5 | 6.60 | 0.42 / 0.15 | 0.00 / 0.00 | - |
| R2701 | it5 | 3.27 | 0.11 / 0.00 | 0.00 / 0.00 | - |
| R2702 | it1 | 8.45 | **1.31** / 0.00 | 0.00 / 0.00 | - |
| R2703 | it1 | 8.89 | **1.45** / 0.00 | 0.00 / 0.00 | - |
| R2704 | it5 | 3.27 | 0.37 / 0.00 | 0.00 / 0.00 | - |
| R2705 | it5 | 7.49 | 0.21 / 0.00 | 0.00 / 0.00 | - |
| R2706 | it1 | 6.97 | 0.07 / 0.00 | 0.02 / 0.02 | - |
| R2707 | it1 | 9.38 | 0.57 / 0.00 | 0.00 / 0.00 | - |
| R2708 | it5 | 11.99 | 0.00 / 0.00 | 0.03 / 0.03 | **12.27** / 0.29 / 0.29 |
| R2709 | it5 | 8.65 | 0.11 / 0.00 | 0.00 / 0.00 | **8.93** / 0.28 / 0.28 |
| R2710 | it1 | 22.55 | **1.01** / 0.00 | 0.05 / 0.05 | **none** / 0.32 / 0.32 |
| R2711 | it1 | 22.57 | **2.15** / 0.14 | 0.00 / 0.00 | **none** / 0.31 / 0.32 |

**it5 against it1, per cell (repeats a / b).** Path error and tilt peak come from n2_bands b22: the hover window (2 s
after the top to LAND) or the figure-8's clock-free window. The lift columns come from s_metrics' lift window.

| cell | arm | runs | path error (mm) | tilt peak (deg) | lift tilt peak (deg) | lift RMSE (mm) | late planner (%) | late tracker (%) | published ticks with res_eq > 1e-2 | max published res_eq |
|---|---|---|---|---|---|---|---|---|---|---|
| hover | it5 | R2700 / R2701 | 1.34 / 1.32 | 0.25 / 0.20 | 1.77 / 1.87 | 19.0 / 20.0 | 0.15 / 0.00 | 0.00 / 0.00 | 0 / 0 of 440 | 1.1e-8 / 1.1e-8 |
| hover | it1 | R2702 / R2703 | 1.35 / 1.37 | 0.24 / 0.21 | 1.82 / 1.87 | 16.6 / 15.9 | 0.00 / 0.00 | 0.00 / 0.00 | 0 / 0 of 440 | 1.0e-4 / 1.5e-4 |
| figure-8 | it5 | R2704 / R2705 | 11.21 / 11.43 | 1.05 / 1.05 | 1.80 / 1.78 | 18.8 / 18.7 | 0.00 / 0.00 | 0.00 / 0.00 | 0 / 0 of 902 | 1.2e-8 / 1.2e-8 |
| figure-8 | it1 | R2706 / R2707 | 9.66 / 9.84 | 0.89 / 0.93 | 1.82 / 1.75 | 16.5 / 16.6 | 0.00 / 0.00 | 0.02 / 0.00 | 0 / 0 of 902 | 4.2e-4 / 3.5e-4 |
| release | it5 | R2708 / R2709 | - | 6.75 / 5.79 | 2.57 / 2.54 | 48.6 / 49.2 | 0.00 / 0.00 | 0.03 / 0.00 | 0 / 0 of 491 | 2.5e-7 / 1.9e-7 |
| release | it1 | R2710 / R2711 | - | 6.46 / 6.96 | 2.63 / 2.66 | 47.3 / 47.9 | 0.00 / 0.14 | 0.05 / 0.00 | 0 / 0 of 490 | 1.9e-3 / 2.3e-3 |

| release cell | runs | release tilt peak, from the 5 s mean before (deg) | settled, 5 s before LAND (deg) | planner at n = 3 after the RELEASE (s) | drop |
|---|---|---|---|---|---|
| it5 | R2708 / R2709 | 8.25 / 7.40 | 0.085 / 0.083 | 0.29 / 0.28 | no / no |
| it1 | R2710 / R2711 | 8.00 / 8.46 | 0.103 / 0.106 | 0.32 / 0.31 | no / no |

**The bars.** Each bar is computed from the it5 pair's mean, and each it1 run is held against it.

| bar | hover | figure-8 | release |
|---|---|---|---|
| C1 inside the b22 band | <= 22.0 mm and <= 3.21 deg: holds | <= 33.0 mm and <= 4.06 deg: holds | does not apply |
| C2 path <= max(1.25 x it5 mean, it5 mean + 3 mm) | <= 4.33 mm: holds | <= 14.32 mm: holds | does not apply |
| C3 tilt peak <= it5 mean + 1 deg | <= 1.23 deg: holds | <= 2.05 deg: holds | <= 7.27 deg: holds (R2711 6.96 is the closest) |
| C4 validity | 4 of 4 valid | 4 of 4 valid | 4 of 4 valid |
| C5 no published tick with res_eq > 1e-2 or none logged | holds | holds | holds |
| C6 lift tilt peak <= it5 mean + 1 deg | <= 2.82 deg: holds | <= 2.79 deg: holds | <= 3.55 deg: holds |
| C7 no drop; n = 3 within 2.0 s; release peak and settled tilt <= it5 mean + 1 deg | does not apply | does not apply | no drop; 0.31-0.32 s; peak <= 8.82 deg (R2711 8.46 is the closest); settled <= 1.08 deg: holds |

**Verdict: C CONFIRMS.** C1-C7 hold in both repeats of every cell, and no valid run misses a bar. The closest margins
are both in the release cell, both R2711: C3 by 0.31 deg and C7's release peak by 0.37 deg.

**Observations (no bar):**
- *it1 tracks the figure-8 closer and lifts with a smaller position error, in both repeats.* A likely reason, not
  tested: in the lab containers the it5 solve takes about 10x the it1 solve in wall time. On published ticks the
  median is 84-98 ms at it5 against 7-12 ms at it1. At RTF ~0.25, the it1 plan therefore reaches the trackers about
  20 ms of sim time younger (M3's effect, larger here). On the rig that difference would be in wall time.
- *One iteration leaves a dynamics residual that the publish gate never reads.* The published res_eq (table above)
  peaks at the release, with 5 published ticks over 1e-3 in each it1 release run. That is under C5's 1e-2 but over
  Q2's 1e-3.
- *The it5 pairs are also `plan_fast_refs`' twin smoke run.* Both pairs are inside their b22 bands with C4 held; §4
  has the b4 baselines without the switch.
  - Wesley's (20) lets the twin default flip after the smoke run and the identity tests.
  - §8 also asks for `test_horizon_kinematics.py` and one `--ref-hash` replay inside a lab container. That was not
    checked here.
  - The flip happens only between claim tables.
- *The release cell's four hover tilted before the release, in both arms.* That comes from the M1 layout's uneven
  planned tensions (s_metrics stages) and settles once the three are a Y. It is not an iteration effect.

**N's comparators from these runs.** §5 N's formulas are evaluated here, not re-set. They count as "on the same tree"
for N because src/ is unchanged since batch 17 started at 12:27; only `tools/experiment/runner_node.py` and its test
changed after that.

| N bar | from | value |
|---|---|---|
| N1 ring position RMSE | 2 x mean(s_metrics n1 on R2702 / R2703: 1.35 / 1.37 mm) + 3 mm | <= 5.72 mm (and <= 25 mm) |
| N1 tilt peak | 2 x mean(0.235 / 0.209 deg) + 1 deg | <= 1.44 deg (and <= 6 deg) |
| N2 release tilt peak | mean(R2710 / R2711: 8.00 / 8.46 deg) + 1 deg | <= 9.23 deg (and <= 15 deg) |
| N2, REVISED 23 Oct 15:12 (Wesley; before any claim run; the claims fly at 5 iterations) | mean(R2708 / R2709, the release cell at 5 iterations: 8.25 / 7.40 deg) + 1 deg | **<= 8.83 deg** (and <= 15 deg); 9.23 reported beside it |

**What C means for the chain's iteration count.** §5 B's rule is "1 iteration if Q and C hold, else 2, else 5".
- C holds at 1 iteration.
- Q has not been run: there is no `results/analysis/s_quality_replay_*` or `s_quality_bench_n8_*`. Both missing pieces
  are offline (§7: tens of minutes and ~20 min):
  1. The nine-flight replay (Q1-Q5): `tools/planner_replay.py --flight` R2348, R2352, R2356, R2343, R2360,
     RIG-1022-h3, -f3h, -ls3c, -f2slope `--arms base,i2,i1 --quality --q-source auto --flown-pivot --summary-json
     results/analysis/s_quality_replay_<date>.json`. Every input is on disk.
  2. Q6, the untimed n = 8 pass: `tools/solve_scaling.py --n 8 --iters 5 2 1 --fast --pivot-dz 0.04 --traj-radius 1.0
     --ring-inertia annulus --quality --noise 0.01 --out results/analysis/s_quality_bench_n8_<date>.json`.
- Until Q holds at 1 iteration, the card has not picked the count. `s8_hover_R50_L100_it1` and
  `s8_chain4back_R50_L100_it1` flown before then count as claims only if Q then holds at i1.
- On a Q miss at i1 the chain re-flies at 2 iterations. N1 would then need a comparator at that count, and the card
  names none.
- A miss is a live possibility: §5 discloses a 20-tick smoke of Q6's flags at n = 3 that sat outside Q2-Q4 at i1.

## Results, Q6 (23 Oct, scored 14:35; `results/analysis/s_quality_bench_n8_2026-10-23.json`, n = 8, noise 0.01, untimed)
| Bar | i5 | i2 | i1 |
|---|---|---|---|
| Q1 cost ratio, paired: p50 / p95 (<= 1.10 / 1.50; a tick over Q2's 1e-3 is a miss) | 1.000 / 1.001 | 1.003 / 1.010 | **miss** (170 of 200 ticks over the residual bar) |
| Q2 dynamics residual p95 (<= 1e-3) | 3.2e-7 | 8.8e-5 | **9.3e-3** |
| Q3 references p95: node 1 / node 5 (<= 5 / 10 mm) | 0.9 / 0.9 mm | 3.2 / 5.1 mm | **7.3 / 14.5 mm** |
| Q4 node-0 feed-forward p95 (<= 0.3 m/s^2) | 0.12 | **0.316** | **0.60** |
| failed solves | 0 | 0 | 0 |

**Q6 falsifies i1** (Q1-Q4) **and i2** (Q4 by 0.016 m/s^2). By §5's rule (any bar missed on any flight or in Q6; then
i2 if it passes them all, else 5 iterations) the 8-quad chain and the rig N_max are read at **5 iterations**, whatever
the nine-flight replay (Q1-Q5, still running) shows. Solve time at n = 8 on the laptop, idle: i5 p50 227 ms, i1 within
the 60 ms budget (Q6 is untimed; B, the bench, is the timed reading).

## Results, Q1-Q5 (23 Oct, scored 14:58; `results/analysis/s_quality_replay_2026-10-23.json`, the nine flights, warm ticks)
| Flight | i2: Q1 paired p50/p95, Q2 res p95, Q3 ref1/ref5 p95 (mm), Q4 ff0 p95 | i1: the same | i1 misses |
|---|---|---|---|
| twin b4 hover (R2348) | 1.000/1.000, 1.6e-7, 0.2/0.2, 0.023 | 1.000/1.000, 1.9e-5, 0.3/0.5, 0.036 | none |
| twin b4 figure-8 (R2352) | 1.000/1.000, 1.3e-7, 0.2/0.3, 0.030 | 1.000/1.000, 1.3e-5, 0.4/0.7, 0.045 | none |
| twin b4 spinny line (R2356) | 1.000/1.000, 6.1e-6, 0.5/0.7, 0.062 | 1.000/inf (117 over), 2.3e-3, 1.2/2.6, 0.107 | Q1, Q2 |
| twin b3 figure-8 (R2343) | 1.000/1.000, 2.9e-7, 0.3/0.3, 0.033 | 1.000/1.000, 2.0e-5, 0.4/0.7, 0.050 | none |
| twin b2 figure-8 (R2360) | 1.000/1.000, 3.1e-7, 0.3/0.4, 0.041 | 1.000/1.000, 4.3e-5, 0.6/0.9, 0.063 | none |
| rig h3 | 1.000/1.002, 6.3e-5, 0.8/2.2, 0.098 | 1.001/inf (49), 3.0e-3, 1.6/4.1, 0.154 | Q1, Q2 |
| rig f3h | 1.001/1.003, 5.5e-5, 1.3/3.0, 0.144 | 1.002/inf (148), 2.3e-3, 2.3/5.6, 0.227 | Q1, Q2 |
| rig ls3c | 1.001/1.003, 1.3e-4, 1.5/3.3, 0.165 | inf/inf (232), 6.6e-3, 3.1/6.8, 0.279 | Q1, Q2 |
| rig f2slope | 1.001/1.006 (4), 8.6e-5, 2.2/4.6, 0.247 | inf/inf (294), 7.4e-3, 3.7/10.0, 0.378 | Q1, Q2, Q4 |

Q5 (failed solves) holds for every arm on every flight (0, as the base). **i1 is falsified on five of nine flights**
(every rig flight and the twin's spinny line: the per-tick dynamics residual 2-7 times its bar), **i2 passes Q1-Q5 on
all nine** and misses only Q6's feed-forward at n = 8 (0.316 vs 0.3). By §5 the chain and the rig N_max are read at
5 iterations (Wesley, 23 Oct). Reported beside it: for fleets up to 4, the rig's included, two iterations give
converged plans by every Q bar.

## More complex cases (proposed 23 Oct, for Wesley)
*PROPOSED, written 23 Oct ~15:05 after Wesley's ask (decisions.md, DECIDED 2026-10-23 S (2): "come up with more complex
cases e.g trajectories and quads doing tasks in between etc"). Nothing here has flown and no bar is frozen: Wesley picks
the cases and their order, and a case's bars freeze when he approves it, before its first run. Every config below flies
5 SQP iterations (the claims' count, DECIDED 2026-10-23 S's iteration count), passes `tools/run_experiment.py --check`,
records the wide camera as the other s8 configs do, and is twin only, not real time.*

### What the code allows today
- **Paths.** `load_trajectory.py` flies one lap of fig_8, line_spin, circle or spin, then the planner descends
  (`complete()`, :177); `orbit` (a circle at constant speed) and `line_x` run until LAND. `traj_laps` exists nowhere and
  the runner has no trajectory wait (no `WAIT_TRAJ` in `tools/experiment/config.py` EVENT_KINDS, :28). The planner does
  publish its clock (`/payload/trajectory_state`, `planner_node.py:896-918`), which a runner wait could read.
  *(23 Oct ~16:55: `traj_laps`, the lap-end hold and WAIT_TRAJ are now built for case 7, and the task mode for case 6,
  all behind switches that are off by default: see ### 6 and ### 7. The line numbers in this section are from before
  that build.)*
- **Path holds.** A rejoin is refused while the path runs (`_rejoin_refused`, `dissipative_node.py:3391`: the clock
  advanced on the last tick). Only the planner stops the clock: 1.5 s after every resize (`reconfig_hold_s`, :1615),
  through a rejoin's approach (renewed every tick, :3422, :3478) and through its 10 s fold-in blend (:3228). No event
  pauses the path from outside, so with no new code a rejoin on a path must start inside one of those holds (case 1).
- **A freed quad's reference is the planner's** (`_rejoin_tick`, :3339, publishes it every tick, staged or rejoining):
  the runner cannot send it anywhere. A task while away is planner code (case 6).
- **Each tether re-welds only to its own stub** (`DetachableJoint` parent `stub_pay_i`, child `tether_i`,
  `eight_rigid_ground_rig_R50_L100.sdf:617-693`): a rejoin at another plate needs a world.
- **The release detector takes one loss per tick** and restarts its count after it (`_detect_tick`, :1858-1860): two
  quads let go together are resized one after the other, a few ticks apart.

### Ranked
Value is what a case adds to S's claim (8 quads carry the ring through releases and rejoins, twin only); cost is code,
runs and lab time. Slot time per run is the sim time at RTF 0.26 plus ~5 min to start (R2597: 282 s of sim in 18 min of
run). At 5 iterations an 8-quad tick solves in ~227 ms on the idle laptop (Q6) plus the ~45 ms residual (R2464), so the
planner keeps its 0.1 s sim period only below RTF ~0.35; the claim runs of batch 18 measure that first, and every case
here inherits their answer.

| Rank | Case | What it adds | Configs (new, `configs/experiments/`) | New code | First pass: runs, slot time | Main risk |
|---|---|---|---|---|---|---|
| 1 | The chain on the move | the releases at 0.2 m/s on a circle, the return with the path held, back at 8 on the move; its hover twin also flies each approach while the last rejoiner folds in | `s8_orbit_r30_v200_R50_L100_it5`, `s8_chain4b2b_R50_L100_it5`, `s8_chain4b2b_orbit_r30_v200_R50_L100_it5` | none | 3; ~47 min | the first ATTACH must land in the last release's 1.5 s hold; a ~70 s hold mid-circle has never flown |
| 2 | Two releases on the figure-8 | the supervisor's trajectory, released at both lobe tips, the freed quads riding the lap | `s8_fig8_h5_v200_R50_L100_it5`, `s8_fig8rel2_h5_v200_R50_L100_it5` | none | 2; ~26 min | staging on a path has flown with a 4-quad T only (E4) |
| 3 | Two quads lost at once | opposite pairs let go together (8 -> 6 -> 4), then the return; an adjacent pair as the neighbour | `s8_chain4back_pairs_R50_L100_it5`, `s8_adjpair_R50_L100_it5` | none (scorer: merge same-time releases) | 2; ~33 min | the adjacent pair's 135 deg gap |
| 4 | A loss during a fold-in | the rejoin card's critic M2 exposure, never flown: a neighbour released inside the last rejoiner's blend | `s8_chain4back_relblend_R50_L100_it5` | none | 1; ~23 min | the cut blend steps the rejoiner to its share at once (the residual the rejoin card accepted) |
| 5 | Relay | the freed quads leave and return in turns, never fewer than 6 carrying: a battery-swap rota with today's machinery | `s8_relay_R50_L100_it5` | none | 1; ~24 min | low (every step keeps a 90 deg gap); a quad away still only stages |
| 6 | A task before the rejoin | a freed quad visits a station (waypoints, a dwell) and comes back: the real "task in between" | `s8_chain4task_R50_L100_it5` | built 23 Oct: the task mode for staged quads (`rejoin_task`, default off) and a runner TASK event; critic waived by Wesley 23 Oct | 2-3; ~1 h | its clearance from the formation is geometry only (no collisions in the twin) |
| 7 | The chain lap by lap | the plan's design B on the figure-8: one release per lap at the same point, rejoins in a hold at lap ends; shared with N | `s8_fig8laps_h5_v200_R50_L100_it5` (base), `s8_chain4back_fig8laps_R50_L100_it5` | built 23 Oct: `traj_laps` and a lap-end hold (`traj_lap_hold_s`), both default off; runner: WAIT_TRAJ; critic waived by Wesley 23 Oct | 2; ~1.5 h | two clock holds overlapping run the clock backwards (handled: one counter, tested) |

Recommended order: rank order, with case 1's two neighbours (`s8_chain4b2b`, `s8_orbit`) before its orbit chain. The
first passes of 1-5 are ~2.6 h of slot time (~1.3 h on two 12-core slots); a claim adds a second run per cell.

### Shared bars (draft, PROPOSED)
- **V, validity** (as N5/N6): the achieved RTF and late-tick fractions first (§9: under 0.25 flagged); late planner or
  tracker ticks <= 1 % (else void, flown once more); no failed planner solve; no published tick with res_eq > 1e-2.
- **L, each release** (N2 at 5 iterations): the planner at the new size within 2.0 s; tilt peak (from the 5 s mean
  before) <= the release cell's it5 mean + 1 deg (R2708 / R2709: 8.25 / 7.40, so 8.83 deg) and <= 15 deg; settled (mean
  over the 5 s before the next event) <= 3 deg; no drop. §5's N2 names the it1 runs (9.23 deg), written before the
  claims moved to 5 iterations: Wesley's call.
- **J, each rejoin** (N3 with the rejoin card's stages): REJOIN_ON_PLATE within 30 s of REJOIN_START; JOINT_ATTACHED
  within 0.5 s of it; the planner one bigger within 20 s of the ATTACH; fold-in tilt peak (weld to +15 s) <= 5 deg; the
  rod end within 2 cm of its plate point through the blend; settled <= 3 deg.
- **N4, back at 8**: from 2 s after the last REJOIN_BLEND_OVER to LAND, inside N1's bars at 5 iterations (comparator
  `s4_hover_L100_it5`, R2700 / R2701).
- **P, on a path** (§5 C2's form, against the same fleet on the same path with no event): clock-free path error (n2_bands'
  b22 metric) <= max(1.25 x the base's, the base's + 3 mm) in the window the case names; where that window holds no
  event, tilt peak <= the base's + 1 deg.

### 1. The chain on the move
- **Shows** S's chain while the ring moves. The ring flies an orbit (a circle at constant speed until LAND) of radius
  0.3 m at 0.2 m/s: its centre stays within x +-0.3, y 0..0.6 m, so the quads 1 m out stay inside +-1.6 m; 0.133 m/s^2
  centripetal; 9.4 s a lap. The four releases come at speed (the first ~7 s after the circle reaches it), each followed
  by the planner's 1.5 s hold, and the fleet flies on at 7, 6 and 5 (~1.4 laps each). The return flies with the path
  held; the circle resumes at 8 after the last blend.
- **Why held, and how without code:** a rejoin is refused while the path runs, so the first ATTACH comes 0.5 s after
  the last release's resize line (a WAIT_LOG on it; the hold lasts 1.5 s) and each next ATTACH 1 s after the previous
  REJOIN WELD, inside that rejoin's 10 s blend hold. The path stays held from the last release to the last blend's end.
  The hover twin does the same, so it is also (c)'s "two rejoins close together" as far as the code allows: each
  rejoiner flies its climb and transit while the one before it folds in (the weld itself waits for the blend, :3507).

| Config | Against | The one variable |
|---|---|---|
| `s8_orbit_r30_v200_R50_L100_it5` ("orbit") | `s8_hover_R50_L100_it5` | the circle (orbit 0.3 m at 0.2 m/s after the 5 s hover hold; LAND at 60, not 50, for ~4 laps at speed) |
| `s8_chain4b2b_R50_L100_it5` ("b2b") | `s8_chain4back_R50_L100_it5` | the return's schedule: from the last release (there 20 s after it), each next ATTACH 1 s after the last weld (there 20 s), first out first back (/drone_1, 5, 3, 7; there 7, 3, 5, 1) |
| `s8_chain4b2b_orbit_r30_v200_R50_L100_it5` ("b2b orbit") | b2b | the circle (as orbit) |

- **Bars (draft).** orbit: V; its clock-free path error and tilt from 2 s after the circle reaches speed to LAND
  reported; falsifier: either over the 4-quad figure-8's b22 band (33.0 mm, 4.06 deg). b2b: V; L per release (as the
  claim); J per rejoin; and the overlap's own bar: each fold-in tilt peak <= the claim runs' (`s8_chain4back_R50_L100_it5`)
  mean fold-in peak + 1 deg; N4. Reported: the return's length (last release to last BLEND_OVER) beside the claim's, and
  the closest approach (body to body) of each rejoiner to the quad folding in. b2b orbit: as b2b with the releases at
  speed; P back at 8 on the move (2 s after the last BLEND_OVER to LAND) against orbit; per stage at speed (8, 7, 6, 5)
  path error and tilt reported beside orbit; every quad's largest |x| and |y| reported (the twin's limit is off). A
  REJOIN REFUSED "the path is running" in b2b orbit means the harness missed the hold: void, not a falsification.
- **Runs:** one each first (~47 min of slot time); for a claim, 2 each of b2b and b2b orbit and 2 of orbit (P's base).
- **Risks.** The ATTACH has 1.0 s of sim (~4 s of wall at RTF 0.26) inside the hold. A ~70 s hold mid-circle has never
  flown: the OCP's horizon keeps previewing the circle from the frozen clock (`reference_builder.yref_at`, :211), and
  each fold-in moves the path's datum onto the measured ring (`attach_datum_shift`, on; QUAD4's one held-orbit weld on
  this planner moved it 6 mm, R2193). The clock steps back to 0.2 m/s at the resume, as after every reconfiguration
  hold. R2597's slowest rejoin took 19.4 s against J's 20 s, and here each approach starts during the last blend.

### 2. Two releases on the figure-8
- **Shows** releases while the ring flies the supervisor's trajectory: the figure-8 at size 0.5 (size 0.9-1.0 would put
  the quads past +-1.75 m), 0.2 m/s, a 19.2 s lap. /drone_1 is released at the right lobe tip (path time 5.81 s: x +0.5,
  moving -y at 0.2 m/s, 0.08 m/s^2 toward the lobe's inside), /drone_5 at the left one (path 13.43 s, plus the first
  release's 1.5 s hold). Both are timed from the path's start line (WAIT_LOG on "hover hold done", the runner's
  PATH_START), so each lands on its point within a tick (2 cm). The six finish the lap and the planner descends; LAND at
  about the base's time. The freed quads stage in the ring frame and ride the lap, as E4's did, instead of landing
  under the path.
- **Configs.** `s8_fig8_h5_v200_R50_L100_it5` against `s8_fig8_h5_v200_R50_L100_it1`: plan_sqp_iters 1 -> 5 (the it5
  twin of batch 18's exploratory figure-8). `s8_fig8rel2_h5_v200_R50_L100_it5` against `s8_fig8_h5_v200_R50_L100_it5`:
  the two releases, with the staging switches they need (rejoin_departed, departed_land false, io tether_tips; inert
  without a release).
- **Bars (draft):** V; L per release (the first one's settled window is mid-lap, so it holds the path's own tilt); P on
  the lap (the path's start to the descent) against `s8_fig8_h5_v200_R50_L100_it5`; each staged quad inside +-1.6 m and
  >= 0.8 m from every carrier body through the lap (E4's bars).
- **Runs:** 2 (~26 min); a claim, 2 each.
- **Risks:** staging on a path has flown with a 4-quad T (E4, R2591), not 8 quads and two staged; each release stops the
  clock 1.5 s mid-lobe, as R2313 / R2316 did.

### 3. Two quads lost at once
- **Shows** a double loss at scale. `s8_chain4back_pairs_R50_L100_it5` against `s8_chain4back_R50_L100_it5`: its
  releases in opposite pairs at once, /drone_1 and 5 together at 45 s, /drone_3 and 7 together at 75 s (there 15 s
  apart; the survivors' largest gap stays 90 deg), then the same return. Neighbour `s8_adjpair_R50_L100_it5` against
  `s8_hover_R50_L100_it5`: /drone_1 and 2 together (8 -> 6, a 135 deg gap, the CoG inside), the six hover to LAND.
- **Expected:** the detector takes the two a few ticks apart (above). An opposite pair is symmetric, so the ring mostly
  heaves (each survivor's share up a third at once); the adjacent pair tilts it toward its gap.
- **Bars (draft).** Pairs: V; per pair both resizes (n - 2) within 2.0 s of the RELEASE rows, the pair's tilt peak (from
  the 5 s mean before) <= 8.83 deg (L) and <= 15 deg, settled <= 3 deg, no drop; the carriers' throttle p95 and time at
  the 0.8 cap reported; then J and N4 as the claim; each pair's peak reported beside the claim's two single releases of
  the same quads. Adjacent pair (a falsifier only): an abort, a drop, the planner not at 6 within 2.0 s, a tilt peak over
  15 deg or settled over 3 deg; reported beside Tier B's G135 at 1 m (R2604 / R2610: 10.42 / 11.02 deg, 4 quads).
- **Scorer (offline, not built):** `s_metrics` scores each RELEASE row as its own event, so the first of a pair gets an
  empty window; the pair needs one merged event (a wrapper or an option; s_metrics.py is not edited here).
- **Runs:** 2 (~33 min).

### 4. A loss during a fold-in
- **Shows** the residual the rejoin card accepted (critic M2), at scale. `s8_chain4back_relblend_R50_L100_it5` against
  `s8_chain4back_R50_L100_it5`: one added RELEASE, /drone_2, unannounced, 3 s after the last rejoin's weld (/drone_1,
  its neighbour on plate 45 deg) and inside /drone_1's 10 s blend. The drop rule counts the rejoiner absent (survivors
  0/3/4/5/6/7, largest gap 135 deg: it flies on); the resize keeps the rejoiner and ends its blend, so its tension
  steps to its 7-quad share at once.
- **Bars (draft):** V; no drop; the planner at 7 within 2.0 s (the rejoiner kept); tilt peak from the RELEASE to +15 s
  <= 10 deg (J's N3 bar: the release and the cut fold-in overlap) and <= 15 deg from the 5 s mean before; settled (the
  5 s before LAND) <= 3 deg; every quad lands. Reported: the rejoiner's planned tension before and after the step, and
  its rod end's distance from its plate.
- **Runs:** 1 (~23 min).
- **Risks:** R0541's kind of yank on a rigid rod. A drop here is a twin finding that goes to Wesley before anything
  changes (the rejoin card's rule).

### 5. Relay
- **Shows** a carry that keeps going while its members rotate. `s8_relay_R50_L100_it5` against
  `s8_chain4back_R50_L100_it5`: the same four quads leave and come back in turns (out 1, 5; in 1; out 3; in 5; out 7; in
  3; in 7), so never fewer than 6 carry and the largest gap is 90 deg at every step; each ATTACH 20 s after the last
  release, each next release 20 s after the last weld, LAND 30 s after the last weld. One variable: the schedule. A quad
  away still only stages: case 6 gives it something to do.
- **Bars (draft):** V; L per release; J per rejoin; N4. Reported: release peaks with 6-7 carrying beside the claim's.
- **Runs:** 1 (~24 min).

### 6. A task before the rejoin (code; built 23 Oct behind `rejoin_task`, default off; critic waived by Wesley 23 Oct)
- **Shows** a freed quad doing something and coming back. The S case: the claim's return with each freed quad visiting a
  station first, e.g. the corner on its own diagonal, (+-1.45, +-1.45) m at its staging height, with a 15 s dwell (a
  battery-swap stand-in in the air), then back to staging and its ATTACH. With the formation 1 m out, only the corners
  of the +-1.6 m box are free. A station beyond +-1.75 m flies in the twin (its limit is off) but breaks S's own
  convention: Wesley's call.
- **Planner** (`dissipative_node.py`, behind a new `rejoin_task`, default off, twin only, needs `rejoin_departed`):
  - the switch beside `rejoin_departed` (:853); a `/fleet/task` subscription (String "d x,y,z,dwell; ...", world m)
    beside `/fleet/attach` (:940);
  - `_start_task`, refused as `_rejoin_refused` refuses (:3375: not staged, LAND or the descent, dropped, the ring not
    up), and when a waypoint is outside `_stage_lim()` (:3274) or a leg comes within the formation's reach + 0.3 m of
    the planned ring point (on a path, of every ring point the path reaches while the task runs);
  - a "task" branch in `_rejoin_tick` (:3339, beside the retreat's): `take_back_ramp` at APPROACH_VEL to each waypoint,
    held inside the bound by `_stage_inside` (:3279), reached by the reference (TAKE_BACK_REACHED_M), its dwell, then
    back to "stage"; lines TASK START / TASK AT k / TASK DONE / TASK REFUSED;
  - `_rejoin_busy` (:3330) leaves a task out (the quad is away from the ring); `_rejoin_out` (:3629) lands a quad on a
    task from where it is (no climb over a plate). An ATTACH during a task is refused already (:3379).
- **Launch:** `common.yaml:110` gets `rejoin_task: ''`; `real_mode.py` PLANNER_OPTIONAL (:119) and `planner_options`
  (:153-175) refuse it on the rig and without `rejoin_departed`.
- **Runner:** `config.py` EVENT_KINDS (:28) gets TASK with a parser and its checks (waypoints inside +-1.57 m, z at least
  the rod + 0.35 m so the rod's end clears the floor), and `_rejoin_problems` (:500) requires the switch;
  `runner_node.py` gets a `/fleet/task` publisher beside `/fleet/attach` (:196), a `_fire` branch (:681) and the TASK
  rows in `_ROSOUT` (:359).
- **Tests:** `src/dissipative_planner/test/test_rejoin_task.py` in `test_rejoin_departed.py`'s stub style (the
  waypoints, the bound, the keep-out refusal, LAND and a drop mid-task, an ATTACH refused during it and accepted after
  TASK DONE); `tools/test/test_task_event.py`. Critic: yes (a new reference for a flying quad in the flight loop, and its
  keep-out is a gate). **Waived by Wesley 23 Oct**; the registry rows say so when it flies.
- **Bars (draft, for when it is built):** each waypoint reached (reference within 2 cm, body within 5 cm) and its dwell
  held; the quad never within 0.5 m of a carrier's body and inside +-1.6 m; TASK DONE, then J; the fleet meanwhile
  inside N1's bars at its size. **Runs:** 2-3 after the code (~1 h).
- **Built 23 Oct (~16:55; switch `rejoin_task`, default off, twin mode:=dissipative only, needs `rejoin_departed`;
  critic waived by Wesley 23 Oct).** As designed, in `dissipative_node.py`: `parse_task` (:325); the switch and its
  refusal without `rejoin_departed` (:893, :907); `/fleet/task` (:988); the task branch of `_rejoin_tick` (:3429,
  :3445); `_rejoin_busy` (:3399) and the rejoin's one-at-a-time rule both leave a task out; `_rejoin_out` (:3703); the
  task itself (`_fleet_task_cb` to `_task_step`, :3746-3845). Details the design left open, as built:
  - TASK DONE comes once its reference is back on its staging point (the way back is part of the task), so an ATTACH
    after it starts from staging, as in the claim;
  - the planner also refuses a waypoint whose rod end would be under 0.3 m (TASK_TIP_FLOOR_M), beside the runner's
    rod + 0.35 m;
  - the keep-out is the quad's own staging radius (reach + 0.3 m) less 2 cm, because its route starts on that circle.
    It is taken horizontally, from the route (its reference, the waypoints, back to its staging point) to the planned
    ring points. On a path every ring point of a whole lap counts (orbit and line_x: one period). That is conservative:
    on the size-0.5 figure-8 the lobes come 0.84 m from a staging point, so every task on that path is refused. Case 6
    flies in hover;
  - LAND or a drop mid-task logs TASK ABORT; the quad goes out along its own bearing from where it is (to the clear
    radius, if it is nearer than that) at its own height, then lands alone;
  - a quad on its task holds up neither another quad's rejoin nor a DETACH, because it is outside its staging radius.
    The one-rejoin-at-a-time rule still holds.
  Launch: `common.yaml:113`, `real_mode.py:137` and :177-179 (refused on the rig, outside the twin's dissipative
  graph and without `rejoin_departed`). Runner: TASK (`config.py:91`, `task_arg` :265, the checks in
  `_rejoin_problems` :553); `runner_node.py` publisher (:212), `_fire` (:744) and the rows TASK_START / TASK_AT /
  TASK_DONE / TASK_ABORT / TASK_REFUSED (:399-403). Tests: `src/dissipative_planner/test/test_rejoin_task.py` (19;
  with the switch off a TASK is refused and the quad holds), `tools/test/test_task_event.py` (11) and the launch test
  in `src/bringup/test/test_real_mode_launches.py`. Gate --quick PASSED 16:55 (1927 passed).
- **Config (first pass, `--check` ok; bars not frozen):** `s8_chain4task_R50_L100_it5` against
  `s8_chain4back_R50_L100_it5`. One variable, the tasks: the same four releases, then for /drone_7, 3, 5 and 1 in turn
  a TASK to the corner on its own diagonal ((+-1.45, +-1.45) m at 1.93 m, 15 s dwell; each route is radial, at least
  0.9 m from any carrier); its ATTACH 2 s after its TASK DONE, the next TASK 10 s after the weld, and LAND 30 s after the
  last weld (~350 s of sim).

### 7. The chain lap by lap (code; built 23 Oct behind `traj_laps` and `traj_lap_hold_s`, default off; critic waived by Wesley 23 Oct)
- **Shows** the plan's design B at scale: on the figure-8 (size 0.5) a lap at 8, one release per lap at the same point
  (mid-lap) down to 4, one rejoin per lap end with the path held, a last lap at 8 (`s8_chain4back_fig8laps_R50_L100_it5`,
  9 laps of ~19 s plus the holds). The same code gives N its 4 -> 3 -> 2 -> 3 -> 4 on the figure-8 and the spinny line.
- **Trajectory:** `traj_laps` in `load_trajectory.py`, default 1 (today's single lap, bit for bit): `__init__` /
  `from_params` (:42, :49); the lapped kinds evaluated at the lap's own time (`offset_at` :195, `accel_at` :252,
  `yaw_at` :288, line_spin's turn adding 2 pi a lap); `complete()` (:177) after the last lap. `_publish_traj_state`
  (`planner_node.py:896`) adds the lap and its period; `ring_bridge.py:155` and the tools that rebuild a flight's path
  read it (`from_params` callers: `tb5_replay.py:111`, `lag_metrics.py:77`, `yaw_track.py:98`; `planner_replay.py:382`
  builds its own).
- **The lap-end hold:** at each lap end but the last, the dissipative node holds the clock `traj_lap_hold_s` through
  `_reconfig_hold_left` and logs "lap k done" (a line a WAIT_LOG can wait on); a rejoin started in it keeps the path
  held as today. Not through the parent's pre-path hold (`planner_node.py:1610`): the hold's cancel
  (`dissipative_node.py:1055-1057`) takes a tick off the clock whether or not the parent advanced it, so two holds
  overlapping run the clock backwards (as R0518 drifted in hover).
- **Runner:** WAIT_TRAJ ("tau [timeout]" or "lap x [timeout]"): `config.py` EVENT_KINDS (:28) with a parser beside
  `wait_log_arg` (:223); `runner_node.py` subscribes to `/payload/trajectory_state`, blocks the schedule until the
  planner's `traj_t` reaches it and times later events from that tick, as WAIT_LOG does (:528-541, :605-608); tests in
  `tools/test/test_runner_schedule.py`'s style. Runner-only and self-contained (built with `traj_laps`, below).
- **Critic:** yes (the trajectory and a hold that gates the path are in the flight loop). **Waived by Wesley 23 Oct**;
  the registry rows say so when it flies. The plan's design A (a smooth pause, `PausableTrajectory`) would replace case
  1's borrowed holds and the step stop of every hold. **Runs:** 2 after the code (~1.5 h).
- **Built 23 Oct (~16:55; switches `traj_laps`, default 1, and `traj_lap_hold_s`, default 0 = off; twin only; critic
  waived by Wesley 23 Oct).**
  - `load_trajectory.py`: `LAPPED` (:33: the figure-8, line_spin, circle and spin; laps must be whole, and above 1
    only for those); `laps` in `__init__` / `from_params` (:47-64); `lap_s()` (:69); `_lap()` (:76). Each lap is the
    one-lap path at the lap's own time; line_spin and spin turn 2 pi more each lap; `complete()` (:203) comes after the
    last lap. One lap is today's path bit for bit, checked against a snapshot of today's values
    (`src/mpc_planner/test/fixtures/load_trajectory_one_lap.json`).
  - `planner_node.py`: `traj_laps` declared (:406); `/payload/trajectory_state` carries `laps` and `lap_s` (:914).
    `ring_bridge.py:155` passes the laps on, and `planner_replay.py` does so only when the flight was lapped (its
    `--src` may be older). The other tools read them through `from_params`.
  - The lap-end hold: `dissipative_node.py` `_lap_end_tick` (:1138), called after the parent's tick (:1123). When the
    clock passes a lap end other than the last on a tick (it began that tick before the lap end), it sets
    `_reconfig_hold_left = max(itself, traj_lap_hold_s)` and logs `lap k done at t=...`, once per lap end and never
    in the descent or LAND. The resize's, the rejoin's and the lap-end holds share that one counter, so the clock never
    runs back. A rejoin started in the hold renews it, and its weld's 10 s blend hold follows. The clock stands under
    one tick past the lap end, where the ring is at rest (under 0.1 mm and 0.1 mm/s). With the switch on, the hold's
    cancel (:1101-1107) also skips the descent, where the parent makes no advance, so a LAND inside a hold no longer
    runs the clock back; with it off, that is unchanged (R0518's mechanism). It needs reconfig_mode ocp; the node and
    the launch refuse it otherwise.
  - Runner: WAIT_TRAJ, 't_s [timeout]' or 'lap x [timeout]' (`config.py:93`, `wait_traj_arg` :251). `_traj_problems`
    (:583) checks that the path's clock runs, that lap targets are within traj_laps, and that the lap knobs are on a
    lapped path in the twin's dissipative mode with ocp. `runner_node.py` subscribes to `/payload/trajectory_state`
    only in runs that have a WAIT_TRAJ. The wait (:576) ends on the first sample whose flown clock reaches the target
    (`traj_clock`, :77): the state goes out at the start of the planner's tick, so the flown clock is traj_t + one tick
    while the path runs. In a lap-end hold the published traj_t reads one tick short of the lap end. The planner's
    'lap k done' lines become LAP_DONE rows (:404).
  - Launch: `common.yaml:61-62`, `real_mode.py:137` and :180-185 (traj_laps refused on the rig; the lap-end hold
    refused outside the twin's dissipative graph with ocp). Tests: `src/mpc_planner/test/test_traj_laps.py` (9,
    added to the gate's list) and `src/dissipative_planner/test/test_lap_hold.py` (5). The lap-hold tests run the real
    `_plan_inner` with the parent's clock:
    - with the switch off, no hold at any lap end;
    - with it on, a 3 s hold at each lap end but the last, once each;
    - **the overlap the design warns of:** a rejoin starts 0.5 s into the lap-end hold, then flies its approach, weld,
      fold-in and blend; the clock stands at the lap end through all of it and never runs back;
    - LAND inside a lap-end hold;
    - the hazard itself: the same lap end held on the parent's pre-path hold runs the clock back 1.0 s in 10 ticks.

    WAIT_TRAJ is tested in `tools/test/test_runner_schedule.py`, and the switches in the launch test. Gate --quick
    PASSED 16:55 (1927 passed).
- **Configs (first pass, `--check` ok; bars not frozen):**
  - `s8_fig8laps_h5_v200_R50_L100_it5`, P's base: 9 laps at 8 quads with the 3 s lap-end holds, LAND 2 s after the
    last lap (~230 s of sim). Against `s8_fig8_h5_v200_R50_L100_it5`; one variable, the laps.
  - `s8_chain4back_fig8laps_R50_L100_it5`, against that base; one variable, the chain. The releases of /drone_1, 5, 3
    and 7 come at WAIT_TRAJ lap 1.5, 2.5, 3.5 and 4.5 (mid-lap: the centre crossing at speed). The ATTACHes of /drone_7,
    3, 5 and 1 come 0.5 s after 'lap 5 done' to 'lap 8 done' (WAIT_LOG); each next lap-end wait starts after the last
    weld. LAND 2 s after lap 9 (~355 s of sim).

### Considered, not proposed now
| Idea | Why not now |
|---|---|
| Land on a pad and take off again (a battery swap) | the twin's rods have no collision, so below ~1.05 m a freed quad's rod passes through the floor; a landed freed quad is disarmed alone and nothing re-arms one quad mid-flight: tracker, fleet manager and planner code plus a floor start, for a twin that cannot show it honestly |
| Pick up an object while away | a world with an object and a joint on a rod's end (Wesley's word), case 6's task mode and an approach to the object; the rig's K (QUAD4, Tejen's controller) is the real version |
| Rejoin at another plate | each tether welds only to its own stub (the world, :617-693): 8 x 8 joints or a re-slotting planner and a new world; the rig's magnet takes any plate, the twin cannot |
| Two rejoins at once | the one-at-a-time refusal (:3394) is a gate (critic), and the welds already queue on the blend (:3507); case 1's hover twin overlaps an approach with the last fold-in, as close as the code allows |
| A rejoin while the path moves | refused by design (the rejoin card's critic M3: the ATTACH would stop the ring in one tick), and the supervisor pauses the path for every rig attach |
| The chain on the spinny line | staged quads ride the ring frame; with 8 quads on a 0.75 m line the spin turns it at ~0.63 rad/s, so a staging point 1.3 m out accelerates ~0.5 m/s^2 against the staging ramp's 0.25 (TAKE_BACK_ACCEL): the staged quad would lag across the formation |

### For Wesley
1. Which cases fly, and in what order (recommended: rank order; in case 1 `s8_chain4b2b` and `s8_orbit` first).
2. L's comparator at 5 iterations: the release cell's it5 runs (8.83 deg, used above) or §5's it1 runs (9.23 deg).
3. Case 1 borrows the planner's holds for its return: fly it as is (exploratory first), or wait for a clean pause (case
   7's design A) before it can be a claim.
4. Whether `s8_fig8_h5_v200_R50_L100_it5` replaces batch 18's exploratory figure-8 at 1 iteration.
5. Cases 6 and 7 to the critic now (code after its answer and yours), or after 1-5 have flown. *Answered 23 Oct:
   critic waived, built behind switches (default off, twin only), first-pass configs ready (### 6, ### 7).*
6. (From the 6-7 build, 23 Oct.)
   - A task on a path: the keep-out counts every ring point of a lap, so a task on the size-0.5 figure-8 is refused
     (its staging points are 0.84 m from the lobes). Keep that, or ask for a check timed along the task.
   - The keep-out (staging radius) against the draft bar (0.5 m from a carrier's body): a route along the staging
     circle passes 0.3 m from a carrier. Case 6's corner routes are radial, at least 0.9 m from any carrier. Raise the
     keep-out to the bar's 0.5 m?
   - `traj_lap_hold_s` also stops a hold running the clock back in the descent. That rides with the switch: one
     variable, or split it out?
   - The lap-end hold is 3 s, a window for the ATTACH; the rejoin then holds the path itself. A release inside a
     lap-end hold cuts it to the resize's 1.5 s, as every resize hold assigns today.
