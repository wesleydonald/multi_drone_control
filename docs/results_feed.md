# Results feed: from the lab chat to the thesis chat (started 30 Oct 2026)

The lab and results chat (`/lab`) appends an entry here the moment something the thesis depends on changes. The thesis
chat (`/thesis`) reads every entry after the last one it processed (recorded in `docs/THESIS_STATE.md`), acts on it,
and records the newest one. Append only: a later entry corrects an earlier one, nothing is rewritten.

**Entry format**
```
### F<nnn> <date> <time> <KIND>: <one-line title>
- Section: <the thesis section(s) it touches>
- What changed: <plain words, the numbers>
- Text should now say: <the claim, as the writer should put it; or "keep the planned box">
- Evidence: <card / analysis / figure path; run ids for source comments only>
```
KIND is one of: RESULT (final, ticked in `handin_plan.md`), INTERIM (a first pass: do not write it up yet), CORRECTION,
STALE (a cited result is now in doubt), FIGURE (ready in a review folder), DECISION (Wesley's, changes the text), FACT
(about the setup).

---

### F001 2026-10-30 16:52 FACT: the flight arena is 6 x 6 x 5 m
- Section: System overview (`system_overview.tex` line 12 says 6 x 6 x 4 m).
- What changed: Wesley confirmed the supervisors' wind-mapping paper's figure, 6 x 6 x 5 m.
- Text should now say: 6 x 6 x 5 m.
- Evidence: `docs/decisions.md` 30 Oct.

### F002 2026-10-30 16:52 FACT: the hardware for the setup table
- Section: the experimental platform (the UAV and motion-capture tables).
- What changed: QUAD1-2 are CineLog35 V2, QUAD3-4 CineLog35 V3; 1350 mAh packs; masses as measured (0.55 kg, 0.583 kg
  with the 1 m cable and magnet); ten OptiTrack PrimeX 13 cameras at 240 Hz; every real flight ran from the same laptop
  (the ground computer). The Motive version: from the supervisors' paper, or omitted. The rate model's constants (tau
  0.12 s, 100 deg/s) are not sourced in the text.
- Text should now say: a UAV table and a "how each value was found" table in the supervisors' format.
- Evidence: `docs/decisions.md` 30 Oct; `docs/analysis/supervisor_papers_review_2026-10-30.md` sections 1 and 3.

### F003 2026-10-30 16:15 FIGURE: F1, the ring's bending and the band-stop
- Section: Results, the later setup (fig:res_bend).
- What changed: `fig_res_bend` is in `docs/figures/review_2026-10-30/` and both Images/results folders, not in any .tex.
- Text should now say: with four carriers the ring bends at about 5.3 Hz; two 2 Hz low-pass stages HALVED it in the lift
  (not removed it); their lag let the triangle slide sideways at 1.1 Hz; the band-stop has flown only with three
  carriers, where the bending never showed, so the text must not claim the band-stop removes it (its first four-carrier
  flight is 4 Nov).
- Evidence: `docs/figures/review_2026-10-30/index.md` (fig_res_bend); real-world flights of 29 Oct.

### F004 2026-10-30 16:15 CORRECTION: the low-pass bending numbers
- Section: wherever the draft quotes the two-stage low-pass bending amplitude.
- What changed: the registry's "two stages 0.07-0.09 m/s^2" was measured at the top of the flight, against the others
  in the lift. Over the same lift window two stages give 0.25-0.33 m/s^2 (no filter 0.57-0.62, one stage 0.40-0.47).
- Text should now say: the like-for-like numbers above.
- Evidence: `docs/figures/review_2026-10-30/index.md`.

### F005 2026-10-30 16:20 INTERIM: X1, what the resize is worth (first pass, not for write-up)
- Section: Results, the release ablation (X1's planned box).
- What changed: 63 runs. In releases that leave an even formation (four quads hover and figure-eight, eight quads, the
  1.15 kg ring) a late resize or no resize flies the same as ours: the measured cable pull holds the ring through the
  release. The planned pull is 2.4-4.4 times worse. In the square-to-T release our planner settled the T 7-13 deg off
  level where no resize settled at 2.2 deg: a defect, being fixed before the code freeze (F006).
- Text should now say: keep the planned box. Wesley's framing: X1 is about the resize; measured against planned pull is
  not a thesis comparison (the measured pull is the method).
- Evidence: `docs/experiments/2026-10-30_x1_release_ablation.md`, `results/analysis/x1_2026-10-30.json`.

### F006 2026-10-30 17:00 STALE: any T formed by an unannounced release, in simulation
- Section: X1's square-to-T cell, the chain (X2), the comparison with Quan et al. through a loss (X8), and any draft text
  on our planner's T after a release.
- What changed: the cause of the T's tilt is found (the planner's 0.1 N minimum pull on the T's middle carrier, the
  sideways model integral, and the simulation's rigid rods with mass); a fix is being tested tonight. Real-world T's fly
  with the sideways integral off and are not affected.
- Text should now say: nothing new about the simulated T until the fix's result is posted here.
- Evidence: `docs/analysis/t_offset_forensics_2026-10-30.md`, `t_offset_model_2026-10-30.md`,
  `t_offset_attach_2026-10-30.md`.

### F007 2026-10-30 16:45 DECISION: what the final re-fly keeps
- Section: The Switch of Solvers; 4.1's later setup; the delay lesson.
- What changed: the 16 Oct 0.55 m runs with a 1 m version (ta_*, tb_*) are dropped in favour of their 1 m re-fly;
  dl_d1 and w4_gi are cited as the earlier setup, as flown; the 30 Oct delay runs, the 1.15 kg triangle joins and two
  network smokes stay cited as exploratory; the 1.15 kg fixed-team cells and the simulated joins are re-flown.
- Text should now say: cite the 1 m rows in The Switch of Solvers; label the exploratory rows as such.
- Evidence: `docs/analysis/x11_refly_manifest.md`, `docs/decisions.md` 30 Oct.

### F008 2026-10-30 16:26 DECISION: X8, the paper's method through a loss
- Section: the comparison with Quan et al.
- What changed: X8 compares our planner with our port of their network, told of the loss 0.35 s after it (the authors'
  own failure model); the exact published law's loss row is left out (it cannot meet a loss as built). It flies after
  the T fix. The exact law's hover row is re-flown on the kept force reading (R6c), which gave the same result.
- Text should now say: keep the planned box for X8.
- Evidence: `docs/experiments/2026-10-30_x8_network_loss.md`.

### F009 2026-10-30 15:37 DECISION: cite the supervisors and reword G4
- Section: the literature review (the gaps, 2.7-2.8) and the experimental setup.
- What changed: the supervisors' five papers (`docs/papers/supervisors/`) already fly off-board MPC to stock Betaflight
  over ExpressLRS in this lab; the thesis cites none of them.
- Text should now say: G4 as "extends off-board MPC on stock Betaflight to a cable's pull"; cite them where the setup,
  the thrust model and the MPC are described.
- Evidence: `docs/analysis/supervisor_papers_review_2026-10-30.md` ("Borrow now").

### F010 2026-10-30 17:17 INTERIM: the simulated T's tilt, cause found, fix being chosen (not for write-up yet)
- Section: X1's square-to-T cell, the chain (X2), X8's rim cell; any text on the simulated T after a release or a join.
- What changed: the cause is the planner's 0.1 N minimum pull on the T's middle carrier together with the sideways
  model integral (on by default in simulation), which holds the ring on its point against the simulated rods' weight.
  With the integral off the resize beats flying on without it (settled 0.58 deg against 2.19); with a zero minimum pull
  for a carrier whose level share is zero the T settles at 1.06 deg but slowly. A join into a T tilts the same way as a
  release into one (6.91 deg with the integral on, 0.83 off): it is the formation, not the direction. Real-world T's
  fly with the integral off and are not affected.
- Text should now say: nothing yet; F006 stands until the fix's default is chosen and its claim runs are posted here.
- Evidence: registry rows R2997-R3003 (exploratory), the three analysis files of F006, the card to come
  (`docs/experiments/2026-10-30_t_settle_fix.md`).

### F011 2026-10-31 10:40 DECISION: the simulated T's tilt stays open, Wesley asks why the integral is needed at all
- Section: X1's square-to-T cell, the chain (X2), X8's rim cell; any text on the simulated T after a release or a join.
- What changed: no fix is chosen yet. Wesley: first find out why the sideways integral is needed at all, in case the
  planner or the trackers are wrong; if nothing is found, the likely fix is a rule that turns the sideways integral off
  when a formation has a carrier with zero share. New exploratory runs: the zero minimum pull with the integral off
  settles worse than either alone (3.33 deg) and the zero minimum pull is worse on the figure-eight (3.07 deg against
  1.57), so it is out. With the integral off, about 0.6 deg in hover and 1.69 deg on the figure-eight, but 64 mm off its
  point in hover. Settling time is reported as a number with no bar. The first 3 s after the release are worse with the
  resize than without it (p95 tilt 21.3 deg against 12.5, integral off), even though it ends more level and dips less.
  The 1.15 kg runs of this cell hit the 0.8 throttle cap and do not count; they re-fly at 0.95.
- Text should now say: nothing yet. F006 stands. Do not write that the T is fixed or that a fix is chosen.
- Evidence: `docs/decisions.md` 31 Oct; registry rows R3201-R3216 (exploratory).

### F012 2026-10-31 10:40 FACT: the Switch of Solvers on the 1 m runs (Q001)
- Section: 4.2.3, The Switch of Solvers.
- What changed: the count survives the move to 1 m: 20 simulated switches, no first solve of the smaller problem failed,
  nor the solve after it, nor any of the 186 solves in the second after it. The first solve's stationarity residual
  reached at most 7.5e-4, against 8.3e-4 for the larger problem's last solve (both peaks from one arm; without it 5.2e-4
  and 5.7e-4). "First solve" means the first solve the planner published: two priming solves run before it on the same
  tick and are not logged. Simulated first-solve times (15.7-93.5 ms) reflect load on the lab computer, so do not quote
  them as solver speed. These are the 23 Oct tree until the final re-fly; 7 of the 20 (r1m_ta_a2 x3, r1m_w4_gi_g2 x2,
  r1m_dl_d1 x2) are being added to it.
- Text should now say: "Across 20 simulated switches with the 1 m cables, the first published solve of the smaller
  problem never failed, nor did the solve after it or any other in the second after it, and its stationarity residual
  reached at most 7.5e-4, against 8.3e-4 for the larger problem's last solve."
- Evidence: `docs/analysis/switch_first_solve_1m_2026-10-31.md` (per-switch table, the 0.55 m figures reproduced).

### F013 2026-10-31 10:40 CORRECTION: two claims in The Switch of Solvers (Q001)
- Section: 4.2.3, The Switch of Solvers.
- What changed: (1) "24-28 ms" is the real-world first solve only: 23.7 and 27.6 ms on the two 7 Oct flights, the other
  four inside that range. It was never a simulated number. It stands for those flights. (2) "A residual in the range of
  the larger problem's last solve" holds in simulation but fails for two real flights of 7 Oct: 9.0e-4 against 1.3e-4
  (ratio 6.8), and 1.1e-3 against 3.5e-4 (ratio 3.1). (3) Two real switches are not in the count: 29 Oct, 1 m cables,
  1.15 kg ring, on a T. Both solved: status 0 first and next, first solves 41.4 and 42.5 ms against the 60 ms budget,
  residuals 5 and 10.5 times the last solve, and the whole switch tick took 157-161 ms against the 100 ms period.
- Text should now say: state 24-28 ms for the real flights of 7-8 Oct only; drop or qualify the residual clause for the
  real flights; add the two 29 Oct switches, or say why the count excludes them.
- Evidence: `docs/analysis/switch_first_solve_1m_2026-10-31.md`, sections "Reproduction" and "Switches outside the count".

### F014 2026-10-31 10:40 FACT: the real and simulated figure-eight in one metric (Q003)
- Section: Results, the later setup (one paragraph).
- What changed: both scored by the same function over the lap: real 21.7 mm sideways, 33.8 mm height, 40.2 mm combined;
  simulation 12.8, 2.0, 12.9 mm. The real ring's height error is nearly all a constant offset (-32 mm mean, 10.7 mm SD).
  One flight against one run, and the flight flew the 2 Hz low-pass the simulation did not (since replaced by the
  band-stop, with no 1 m figure-eight on it yet).
- Text should now say: those numbers, with the constant offset named. Exploratory, one flight.
- Evidence: `docs/analysis/thrust_margin_2026-10-31.md` section 8.

### F015 2026-10-31 10:40 FACT: the measured pull against the ring's weight (Q006)
- Section: the experimental setup or the real-world results (the validation, as in the wind-mapping paper).
- What changed: 54 steady real hovers of at least 3 s, summed vertical pull against the ring's weight, median per group:

  | Setup | Carriers | Hovers | Bias, N | % of ring | SD, N | RMSE, N |
  |---|---|---|---|---|---|---|
  | 1 m, 1.15 kg | 3 | 6 | -0.04 | -0.4 | 0.62 | 0.86 |
  | 1 m, 1.15 kg | 4 | 3 | -1.04 | -9.3 | 0.32 | 1.12 |
  | 0.55 m, 0.86 kg | 2 | 10 | +0.75 | +8.8 | 0.20 | 0.77 |
  | 0.55 m, 0.86 kg | 3 | 22 | +0.22 | +2.6 | 0.30 | 0.45 |
  | 0.55 m, 0.86 kg | 4 | 10 | +0.36 | +4.3 | 0.36 | 0.55 |

  It validates the mean only (the thrust map, the masses and the lean together), not the dynamics or the split between
  carriers. The earlier read of 9-27 % over the weight is refuted as a thrust-map error: it multiplied an acceleration
  built on 0.55 kg by 0.583 kg, and one window had the newcomer over the ring. The airframe's mass as flown is the largest
  open number (0.97-1.29 N, it sets the sign of the 1 m rows), to be settled by weighing on 4 Nov.
- Text should now say: the table above, with what it does and does not validate, marked exploratory until the weighing.
- Evidence: `docs/analysis/thrust_margin_2026-10-31.md` sections 1-3.

### F016 2026-10-31 10:40 FACT: the rejoin table stands; the eight-quad chain is outside the T defect (Q002)
- Section: 4.3, tab:res_rejoin and the eight-quad chain.
- What changed: (a) the re-fly of rj3_pairrelease_rejoin_L100_{tip,lean,leanneg} keeps `int_k_xy` 0 through its own
  params file. Neither remaining fix changes these rows: the zero minimum pull is out (F011), and a rule that turns the
  integral off in a T matches what they already fly. (b) No formation of the eight-quad chain has a carrier under the
  planner's minimum pull: the smallest level share is 1.02 N, so R2766/R2767 are outside the defect, and their settled
  0.04-1.21 deg comes from their uneven 7- and 5-carrier shares.
- Text should now say: the rejoin table and the eight-quad chain stand as they are, until the final re-fly.
- Evidence: `configs/rig/chain_pair_tip.yaml`, the statics in `src/mpc_planner/mpc_planner/geometry.py`
  (`balanced_tensions`), registry rows R2766/R2767.

### F017 2026-10-31 10:40 FACT: X1's and X4's scores in the supervisors' format (Q004)
- Section: X1 (the release ablation) and X4 (the endurance relay).
- What changed: `tools/x1_metrics.py` now reports tilt RMS and p95 beside the peak over the release to LAND, plus T6's last
  exit, the worst tilt after 6 s and the ring's offset from its point. `--table` prints one table per cell with arms as
  rows, mean +- SD for n >= 3, both values for n = 2, and percent against the no-resize arm. `tools/x4_metrics.py` reports
  tilt RMS and p95 per lap and over the laps. Descriptive only: no frozen bar is re-scored.
- Text should now say: keep the planned boxes; the final tables arrive in this format.
- Evidence: `tools/x1_metrics.py`, `tools/x4_metrics.py`, `tools/test/test_x1_metrics.py`.

### F018 2026-10-31 12:29 FACT: one timing table for the methods (Q005)
- Section: the methods, timing (the review's section 4, "Timing").
- What changed: the table is written, each value with how, where and when it was measured. Real-world values in ms, solves
  as median / p95: motion capture 7.5 (one body; the flight's own set is not measured yet, 4 Nov will); velocity estimate
  16-17; planner solve 16.1 / 34.0 (2 carriers), 34.4 / 44.4 (3, the newcomer up), 54.1 / 71.8 (4); tracker solve about 1
  / 2-3; radio 6.5; rate command to rate at the flight controller 11-13, around the whole loop 34-39; motors to lift at the
  flight controller 17-19 free, 9 loaded (weak); throttle to lift around the whole loop 44-65. Direct throttle to thrust is
  not measured. Rows that go around the whole loop already contain the mocap and radio delays, so do not add them. The
  tracker's rate model (0.12 s, 100 deg/s) is assumed, not measured. Every flight value predates the 30 Oct body-set change.
- Text should now say: the table; see F019 for what changes in `system_overview.tex`.
- Evidence: `docs/analysis/loop_timing_2026-10-31.md`.

### F019 2026-10-31 12:29 CORRECTION: timing text in `system_overview.tex` line 151 (Q005)
- Section: System overview, the delays.
- What changed: (1) "25-35 against 48-62" mixes the 0.55 m pairs with a delayed-run subtraction; on one setup and path the
  rate loop reads 34-39 ms real against 83-100 simulated. (2) The throttle's "44-67" is three flights; the latest setup reads
  56-65. (3) "Simulated time waits for the planner" is not so: Gazebo keeps running and at 4 carriers the plan arrives 22-39
  simulated ms after its tick, against 74.0 ms real. (4) The velocity estimate's lag is 16-17 ms, not ~20.
- Text should now say: those numbers, from the table of F018.
- Evidence: `docs/analysis/loop_timing_2026-10-31.md` section 7.

### F020 2026-10-31 12:29 INTERIM: why the model integral is needed (not for write-up yet)
- Section: the planner's integral, the ring's height and tilt in the real-world results, the simulated T (F006, F011).
- What changed: two read-only investigations, exploratory, not reviewed and not yet flown. Real flights: the sideways force
  stands in for cable lengths read short on the floor (they read 3-16 mm longer in flight) and for a reading of about 1 deg
  off vertical (Motive's frame, room air or the ring's centre of mass, undecided). Short lengths also explain the real ring
  riding about 3 cm low. Simulation: a rod with mass pulls its plate outward by about 0.31 N, which cancels on even layouts
  and not on a T, and that is what the integral learns there. The T that looks level with the integral off is a cancellation
  of two errors, not a level plan, so do not describe it as such. Both agree the height integral has almost no authority
  when the trackers fly the measured pull.
- Text should now say: nothing yet. Do not explain the integral's role or the ring's offset beyond the current text; a checked
  entry follows after the test runs and Wesley's decision.
- Evidence: `docs/analysis/why_integral_real_2026-10-31.md`, `docs/analysis/why_integral_code_2026-10-31.md`.

### F021 2026-10-31 16:07 INTERIM: X3b, eight and nine carriers at one planner iteration (not for write-up yet)
- Section: the planner's tick and the eight-quad set (4.4).
- What changed: at eight carriers, one iteration per tick gives the same verdicts as five on hover, figure-eight and the chain on
  the runs flown (hover 1.46-1.51 mm, 0.067-0.085 deg; figure-eight 9.47-10.39 mm against 15.12-15.43 at five; every release and join
  inside its bar, and the chain's position misses its bar at both settings). One five-iteration comparator owes a re-fly. Nine carriers
  are not confirmed: the hover holds (1.56-1.60 mm), but both chains are void. In 3 of 6 chain runs the release detector missed one
  quad (its cable rose 2.6-2.8 cm against a 3 cm test), which a card rule voids. The ground computer's timing runs have not flown, so
  no real-time claim yet.
- Text should now say: keep the planned box. Do not write "nine carriers work" or "eight run in real time" yet.
- Evidence: `docs/experiments/2026-10-30_x3b_realtime_8_9.md`, "Results (31 Oct, first pass)"; not reviewed.

### F022 2026-10-31 17:16 DECISION: why the integral is needed, and what is being fixed (not for write-up yet)
- Section: the planner's integral; the ring's height and tilt in the real-world results; the simulated T (F006, F011, F020).
- What changed: exploratory runs now back F020. In simulation, with no integral force at all, an even hover stays within 1 mm and the even
  figure-eight is 11.8 mm against 12.9 with it. With the rods' mass gone, no floor and no integral, the T is level and on its point (-0.19 deg, 0.5 mm).
  The default T is worse than X1's 20 s showed: left flying it climbs to 16.9 deg by 45 s. A T with the integral off is level at 60 deg cables only (+4.58 deg
  at 70, -2.86 at 50), so never describe that as a level plan. Wesley's decisions: simulation: the planner's rod model is being corrected (a switch, not yet
  flown), with the integral off in T cells as the fallback. Real flights: a second reading of the cable lengths just before the lift (a switch, not yet flown,
  first flight no earlier than Thu 5 Nov).
- Text should now say: nothing yet. No claim that the integral is unnecessary until the switch flies and a reviewer has seen the table.
- Evidence: `docs/decisions.md` 31 Oct, `docs/experiments/2026-10-31_rod_len_reread.md`; registry R3232-R3242, R3292, R3400-R3411.

### F023 2026-10-31 17:38 INTERIM: X2-full, the 4 -> 3 -> 2 -> 3 -> 4 chain on the figure-eight in one run (not for write-up yet)
- Section: X2, the supervisor's chain.
- What changed: the whole chain flew and landed in all four chain runs, with no drop or abort, but both variants miss a frozen bar on this first pass. Announced: one
  fold-in reached 5.19 deg against 5 for 0.11 s, on one repeat. Unannounced: T6 35.5 s against 5, because a 0.5 deg band cannot read a recovery on a moving path (0.84-0.97 deg
  at 5 s after the release). Every other bar holds, with lap 6's path 8.7-10.0 mm. The T laps read 30-41 mm against 9-10 mm for the square: the T hangs about 60 mm off its point, and the
  last fold-in re-zeroes the path there. The last hold sits on the 0.8 throttle cap with the simulated pack near 19 V. It flew before the T fix, so it re-flies on the final tree.
- Text should now say: keep the planned box; do not quote these.
- Evidence: `docs/experiments/2026-10-30_x2_full_chain_fig8.md`, "Results (31 Oct, first pass)"; not reviewed.

### F024 2026-10-31 20:04 DECISION: the simulation's claims will fly the rod in the plan (not for write-up yet)
- Section: the simulation's setup and every simulated table; the T (F006, F011, F020, F022).
- What changed: Wesley 31 Oct: all simulation claims fly with the planner told the simulated rods' true mass, and references that balance it. Real flights keep it off. Exploratory, one run each: the T hangs within 0.3 deg and under 2 mm at 50, 60 and 70 deg cables, the integral learns almost nothing, and the resize beats no resize on settled tilt, point and dip. One limit: a T with drone 0 as the rider hangs 3.37 deg off. X4's 30-minute relay has not completed yet (a kick at a fold-in, cause being found). The final-tree re-fly stays the source of every number, after a reviewer.
- Text should now say: nothing yet. When it lands: the simulated links are rods with mass that the planner models, real cables are light and the real planner leaves this off, so nothing here transfers to real cables as a claim.
- Evidence: `docs/experiments/2026-10-31_rod_in_plan.md` section 8, `docs/decisions.md` 31 Oct.

### F025 2026-10-31 21:05 DECISION: the sideways integral goes off in the simulation's claims (not for write-up yet)
- Section: the planner's integral, the simulation's setup; every simulated table when it re-flies.
- What changed: with the rod in the plan, the 8 -> 4 -> 8 chain with no sideways integral comes back to 0.4 mm and 0.07 deg, against 6.36 mm and 0.99 deg as flown on 23 Oct, whose position missed its bar. Wesley 31 Oct: simulation claims fly with the rod in the plan and `int_k_xy` 0, after a reviewer. Real flights keep their settings: there the integral still has a real room-fixed 0.2 N to hold, and the cable lengths are a separate fix (F022). One run, not reviewed.
- Text should now say: nothing yet. When it lands: in simulation the sideways integral was standing in for the rods' mass, and the height integral is unchanged.
- Evidence: `docs/experiments/2026-10-31_rod_in_plan.md`, registry R3428, `docs/decisions.md` 31 Oct.

### F026 2026-11-01 03:21 INTERIM: X3b's timing and X4's first pass (not for write-up yet)
- Section: 4.4 the planner's tick; X4 the endurance relay; the solve-time figure.
- What changed: the solver fits the 100 ms period on the ground computer at one iteration, idle: 51.0 ms p95 at 8 carriers, 64.0 at 9. The closed loop with Gazebo sharing the same computer does not: 260.9 ms p95 at 8, and 9 never lifted, so "real time" may only be said of the solver on the bench. 9 -> 5 -> 9 now completes with the magnet-marker check. X4's 30-minute relay has not yet passed: two simulation faults found and fixed in turn (a re-weld kick, then a simulated pack that never recharges), a third is the home point moving 1 cm per fold-in. Nothing here is a claim.
- Text should now say: keep the planned boxes for X3b's timing and X4.
- Evidence: `docs/experiments/2026-10-30_x3b_realtime_8_9.md`, `docs/analysis/x4_weld_kick_2026-10-31.md`, `docs/analysis/x4_relay_fail2_2026-11-01.md`.

### F027 2026-11-01 03:36 CORRECTION: the rod in the plan did not pass its reviewer (F024, F025 stand as decisions, not results)
- Section: the simulation's setup, the T, the eight-quad chain.
- What changed: the reviewer failed the first table for a default. Fair so far: in simulation, with hinged rods, in hover, a T with rider 3 or 1 hangs level at 50-70 deg cables and does not run away at 5.26, one run each. Not shown: that the 8 -> 4 -> 8 improvement is the rod (that run changed two variables), real cables, rider 0, disturbances. Dip rises in all seven hard T pairs, +5.6 to +15.3 mm. F025's "the sideways integral was standing in for the rods' mass" and its 0.4 mm are not yet established as cause.
- Text should now say: nothing. One-variable arms and repeats are queued.
- Evidence: `docs/experiments/2026-10-31_rod_in_plan.md` section 9.

### F028 2026-11-02 23:36 INTERIM: the X4 relay flies its whole rota on both rings (not for write-up yet)
- Section: X4, the endurance relay; 4.x the planner's setup in simulation.
- What changed: with the plate's universal hinge, the rod in the plan, no sideways integral, a pack that doesn't sag in simulation and a home point that no longer moves at a fold-in (Wesley adopted them 2 Nov), the relay flies 26 of 26 swaps on the 0.9 kg and on the 1.15 kg ring. The mission is 2.78 B, 1666 and 1671 s, against 0.97 B for the fixed team. Event tilt at most 1.84 and 1.24 deg, height drift about 0, longest pack 549 and 546 s, 7 or 8 carriers throughout. The card's written mission of 1700 s is missed by about 30 s, because the plan projects 1782 s and the rota flies 0.94 of that. On the 1.15 kg run the last lap over the first is 1.40 against 1.25, also missed. One run each, not the final tree, so these are not the claim. Correction to what I said earlier: the 0.9 kg run is 26 of 26, not 25, as one log line was lost in the run's event file.
- Text should now say: keep the planned box for X4. When it lands, the mission length may change: it is Wesley's call.
- Evidence: `docs/experiments/2026-10-30_x4_endurance_relay.md` section 9, `docs/analysis/x4_relay_fail2_2026-11-01.md`, `docs/analysis/x4_weld_kick_2026-10-31.md`.

### F029 2026-11-02 23:39 CORRECTION: the rod in the plan passes its reviewer with corrections; the "it is the rod" comparison does not stand (not for write-up yet)
- Section: the simulation's setup, the T, the eight-quad chain, the attach.
- What changed: the reviewer's second look. Holds on repeats, in simulation with hinged rods, rider 3, in hover: the T settles at 0.01-0.07 deg and 1.0 mm against 8.3-10.5 deg, and the dip rises 5.5 to 9.0 mm. With the rod the chain comes back at 8 to 0.37-0.50 mm and 0.07 deg. Wrong: the no-rod chain arms, as a release was never detected in them, so the 40 mm figure and the rod's share of the chain are not established. The attach's arms changed the newcomer's velocity filter too, so the rod alone on the attach is not established either. The resize's dip gain is 1.5-6.1 mm, not 6.8, and at 1.15 kg 10.9, not 18.5. Re-flies are queued. The rod is not flipped.
- Text should now say: nothing yet. When it lands, nothing about real cables, rider 0 or the rod alone on the attach.
- Evidence: `docs/experiments/2026-10-31_rod_in_plan.md` section 10.

### F030 2026-11-03 04:16 INTERIM: the eight-quad chain and the attach on clean arms (not for write-up yet)
- Section: the simulation's setup, the eight-quad chain, the attach. Replaces F029's open points.
- What changed: the chain re-flown with no release lost, one variable each. Back at 8: today's planner 6.4 mm and 0.99 deg, the integral off alone 1.1 mm and 0.07 deg, the rod alone 0.5 mm and 0.07 deg. So either change is enough there, and only the rod levels the odd stages, 0.03-0.04 deg against about 1 deg. The attach on one newcomer filter, tilt after the weld: today's planner 1.26 deg, the integral off alone 2.20, the rod alone 1.70 (0.95 in another run), both 0.76 (0.32-0.39 in others). So the two together are lowest and the rod alone is within scatter of today's. Today's planner did not weld twice in simulation on an unfiltered speed gate, and it welds with a 0.1 s filter: a simulated gate only, the real attach uses the co-motion check. One run each, judged after the run, not reviewed. Nothing is flipped.
- Text should now say: nothing yet. Do not write "the rod alone" for the attach.
- Evidence: `docs/experiments/2026-10-31_rod_in_plan.md` section 11.

### F031 2026-11-03 19:56 RESULT (current tree, not yet the final one): Q007 items 1 and 2, the fixed team and the one release flown in both
- Section: 4.1 Simulation (the fixed team) and 4.2 Real World (what was flown in both).
- What changed: the runs Q007 asked for, all 20 complete, no drop, no abort. Two runs per cell, shown as a / b. 0.86 kg ring, 1 m cables, the slope estimate. Scored with the same tools as the real flights: best_data.score (hover: distance to the hold point; paths: clock-free distance to the path, and the lag reported apart) and tools/yaw_track.py.

  | Carriers | Hover error (mm) | Figure-eight path error (mm), lag (s) | Spinny line path error (mm), yaw error RMS (deg), yaw delay (s) | Tilt peak, hover / figure-eight / spinny line (deg) |
  |---|---|---|---|---|
  | 2 | 1.1 / 1.1 | 8.7 / 8.0, 0.01 / 0.00 | 8.7 / 7.6, 5.6 / 5.9, 0.19 / 0.20 | 0.16 / 0.18, 2.17 / 2.55, 0.94 / 1.00 |
  | 3 | 1.1 / 1.0 | 11.1 / 10.3, 0.06 / 0.06 | 6.1 / 5.4, 6.6 / 6.8, 0.23 / 0.23 | 0.21 / 0.21, 1.40 / 1.38, 1.15 / 1.25 |
  | 4 | 1.4 / 1.3 | 12.9 / 12.8, 0.11 / 0.10 | 4.1 / 4.9, 4.1 / 4.4, 0.14 / 0.15 | 0.18 / 0.17, 1.25 / 1.29, 1.03 / 1.05 |

  Tilt mean: hover 0.04-0.06 deg, figure-eight 0.80-0.88, spinny line 0.45-0.53. On the figure-eight almost all the error is across the path (height RMS 1.7-2.1 mm). On the spinny line it is mostly height (3.6-8.0 mm).

  **The 4 -> 3 unannounced release in hover, 120 deg gap (simulation, two runs):** detection 0.35 / 0.34 s; tilt rise 6.59 / 7.18 deg; absolute tilt peak 5.57 / 5.93 deg; back in band 3.53 / 3.63 s; dip 30.6 / 32.2 mm; settled 0.03 deg.
  **The real 8 Oct releases, the tilt rise the thesis was missing** (the same tool, tools/release_bands.py; "tilt rise" is how far the tilt moved from its mean over the 5 s before the event, as in simulation):

  | Real flight | Kind | Tilt rise (deg) | Absolute peak (deg) | Back in band (s) |
  |---|---|---|---|---|
  | r303_4 | unannounced, hover | 5.29 | 6.50 | - (log ends 7.6 s after) |
  | r304_3 | unannounced, 150 deg gap | 16.88 | 17.82 | - (log ends 7.7 s after) |
  | r305 | unannounced | 8.02 | 8.09 | 3.93 |
  | r302_3 | announced detach, not a release | 11.21 | 15.35 | - (log ends 6.6 s after) |

- Text should now say: 4.1's fixed-team table can use these simulation numbers. For the release flown in both, compare tilt rise with tilt rise: simulation 6.6-7.2 deg against r303_4's 5.3 and r305's 8.0. **Caveats for the text:** the real 8 Oct releases flew 0.55 m links with no velocity filter, and the simulation flies 1 m cables with the slope estimate. They are the same manoeuvre, not the same setup. Every number here is on the current tree. If the tree changes before Friday's freeze (the rod-in-the-plan flip is Wesley's open call), they re-fly, and I will post a correction.
- Evidence: registry R3489-R3508; `docs/lab_requests.md` Q007. Item 3 (X1, X2-full, X3, X4, X8, X3b's bench, the join figure run) stays in `docs/sim_runs_todo.md` as the claim runs after the freeze.

### F032 2026-11-03 19:56 CORRECTION to F031: the real releases' velocity estimate
- Section: as F031.
- What changed: F031 says the real 8 Oct releases flew "no velocity filter". That isn't established. They flew that day's estimate, which predates the slope filter (the filter switch came on 14 Oct, slope became the default on 28 Oct). The 0.55 m links are confirmed (the planner's log: cable_len 0.550).
- Text should now say: "the real releases flew 0.55 m links and an earlier velocity estimate", not "no velocity filter".
- Evidence: `results/rig/2026-10-08/r303_4.log` (geometry line); `git log -S MDC_MOCAP_VEL_FILTER`.

### F033 2026-11-03 19:58 ANSWER Q008: which "back in band" stands for the releases
- Section: 4.2 Real World, Losing Carriers (tab:res_release), and the release flown in both.
- What changed: the two readings use two definitions. The thesis table's numbers don't come from tools/release_bands.py. They come from `docs/figures/review_2026-10-23_best/fig_reaction.py`.
  - **The table's definition (fig_reaction.py), which stands:** the band is the highest ring tilt in the 5 s before the release plus 0.5 deg. "Back in band" is the last moment after the release that the tilt was above that band, looking until LAND or 15 s, whichever comes first. It reads every real flight, including those that landed 6.6-7.7 s after the release.
  - **release_bands' definition (F031's "-" and 3.93 s):** a band of max(2 x the pre-release spread, 0.5 deg) around the tilt the ring settles to (the last 5 s before LAND), and back in band when the tilt stays inside it for 1 s. It needs LAND at least 10 s after the release, so it can't read r302_3, r303_4 or r304_3.
- Numbers on the standing definition, real and simulated alike:

  | Flight | Back in band (s) | Tilt peak (deg) | Window (s) |
  |---|---|---|---|
  | r302_3, announced detach (real) | 1.95 | 15.35 | 6.6 (to LAND) |
  | r303_4, unannounced (real) | 1.84 | 6.50 | 7.6 (to LAND) |
  | r304_3, unannounced, 150 deg gap (real) | 3.74 | 17.82 | 7.7 (to LAND) |
  | r305, unannounced (real) | 3.79 | 8.09 | 14.0 (to LAND) |
  | R3507, R3508, simulation, 1 m | 3.34, 3.23 | 5.57, 5.93 | 15 |

  The script reproduces the table's real values (1.95 / 1.8 / 3.7 / 3.8) and the review's simulated R2486 (3.36 against its 3.4).
- Text should now say: keep the table's real values. For the release flown in both, compare 1.84 s (r303_4) and 3.79 s (r305) with 3.23-3.34 s in simulation, not with F031's 3.53 / 3.63 (that is release_bands' definition, which the real flights can't be scored on). Say in one line what the band is: the highest tilt before the release plus 0.5 deg.
- Evidence: `docs/figures/review_2026-10-23_best/fig_reaction.py` (reaction()), `tools/release_bands.py` (metrics, constants PRE_S 5, HOLD_S 1, BAND_MIN_DEG 0.5, POST_MIN_S 10); runs R3507, R3508, R2486.

### F034 2026-11-03 21:26 RESULT (current tree): X3b's solver bench, Q007 item 3's first part
- Section: 4.4, the planner's tick; the solve-time figure's simulation panel.
- What changed: the idle bench on the ground computer, rerun on the current tree. At one SQP iteration the whole planner tick (p95) is 11.0 ms with 4 carriers, 51.0 ms with 8, 64.0 with 9 and 79.2 with 10, with no failed solve at any count. These are identical to 31 Oct, so the 8- and 9-carrier ticks fit the 100 ms period with room.
- Text should now say: the 31 Oct numbers stand for the current tree. One caveat: the laptop's load was 1.8 when the bench started, not the 0.5 the card asks for. The numbers matched anyway.
- Evidence: `results/analysis/x3b_bench_res0_2026-11-03.json`, registry BENCH-1103-x3b, card `docs/experiments/2026-10-30_x3b_realtime_8_9.md` (R8a).

### F035 2026-11-04 01:07 RESULT (current tree): X1, the release ablation without the planned-pull arms (Q007 item 3)
- Section: X1, the release ablation (the headline), 4.1 Simulation.
- What changed: 44 runs, all complete, no drop, no failed solve (R3520-R3563). Five arms per cell, x2: ours (the resize at detection), no resize, and the resize 0.5 / 1 / 2 s late. Plus the 1.15 kg cell for ours and no resize. Ranges are over the two runs.

  | Cell | Arm | Tilt rise (deg) | Dip (mm) | Settled (deg) | Tilt after 6 s (deg) |
  |---|---|---|---|---|---|
  | 4 quads, hover | ours | 6.80-6.96 | 32.0-32.7 | 0.06 | 0.50-0.51 |
  | 4 quads, hover | no resize | 6.45-7.03 | 31.0-32.7 | 0.21-0.22 | 0.58-0.61 |
  | 4 quads, figure-eight | ours | 7.03-7.07 | 29.9-30.3 | 3.66-3.68 (the path's own tilt) | 5.30-5.43 |
  | 4 quads, figure-eight | no resize | 6.73-6.93 | 29.8-30.1 | 3.67 | 5.10-5.19 |
  | 8 quads, hover | ours | 1.57-1.59 | 13.2-15.6 | 1.06 | 1.37-1.38 |
  | 8 quads, hover | no resize | 1.61-1.64 | 15.2-15.6 | 1.19 | 1.60-1.62 |
  | 4 quads, 1.15 kg, hover | ours | 9.43-9.68 | 44.7-45.4 | 0.07-0.08 | 0.25-0.26 |
  | 4 quads, 1.15 kg, hover | no resize | 9.24-9.49 | 45.7-45.8 | 0.21-0.22 | 0.47-0.48 |
  | square to T (uneven) | ours | 56.3-57.1 | 96.5-98.2 | **9.77-10.10** | 10.00-10.46 |
  | square to T (uneven) | no resize | 57.3-57.5 | 107.9-108.4 | 2.17-2.18 | 3.20-3.37 |

  The late resizes (0.5, 1, 2 s) fly within a few per cent of the on-time one on peak and dip in every even cell. They take longer to level: T6 5.0-5.6 s against 3.9 s in 4-quad hover.
- What it says: the measured pull carries the ring through the release whatever the planner does. The peak and the dip are set by the pull, not the resize. The resize's worth is the level afterwards: about 3x closer to level in hover (0.06 against 0.21 deg at 0.86 kg, 0.07 against 0.21 at 1.15 kg). **In the uneven T the resize makes it worse, 10 deg settled against 2.2.** That is the known T defect of today's planner in simulation, which the rod in the plan fixes (0.01-0.07 deg, rod card). If Wesley flips the rod before the freeze, the T cell and the cells it changes re-fly and this table is replaced.
- Text should now say: X1 as above for the even cells, with the T row held until the rod decision. Not final: current tree.
- Evidence: `results/analysis/x1_2026-11-04.json` (tools/x1_metrics.py --table), registry R3520-R3563, card `docs/experiments/2026-10-30_x1_release_ablation.md`.

### F036 2026-11-04 05:00 RESULT (current tree): X4, the endurance relay's claim runs (Q007 item 3)
- Section: X4, the endurance relay (the headline).
- What changed: the claim runs: two relays at 0.9 kg, one at 1.15 kg and two fixed teams, all complete, no drop, no failed solve.

  | Run | Swaps | Mission (s), over B | Event tilt max (deg) | Longest pack (s) | Path per lap, first to last (mm) |
  |---|---|---|---|---|---|
  | Relay, 0.9 kg | 26 / 26 | 1681, 2.80 B | 1.72 | 552 | 18.3 to 14.7 |
  | Relay, 0.9 kg | 26 / 26 | 1663, 2.77 B | 1.79 | 546 | 13.6 to 11.9 |
  | Relay, 1.15 kg | 26 / 26 | 1669, 2.78 B | 1.21 | 547 | 14.9 to 13.6 |
  | Fixed team, 0.9 kg | - | 570, 0.95 B | - | 570 | 19.3 to 12.3 |
  | Fixed team, 0.9 kg | - | 569, 0.95 B | - | 569 | 13.3 to 14.3 |

  The relay keeps the ring flying for 2.77-2.80 pack budgets (B = 600 s), against 0.95 for the fixed team. Every swap is done, and the tilt at any release or fold-in stays at or under 1.79 deg. The path error does not grow over 28 laps (last over first 0.81-0.91), the height does not drift, and every pack stays inside B. 7 or 8 carriers fly throughout. All of the card's bars hold on both repeats and the 1.15 kg cell **except one: the mission is 1663-1681 s against the 1700 s written.** 26 leaves cannot reach it (card section 9).
- Text should now say: X4 can be written from these numbers. The mission length is still Wesley's open call: either keep 26 leaves and report 2.8 B with the written miss, or fly 29 leaves (about 1860 s, 3.1 B), which re-flies these five runs. Current tree, re-fly if it changes before the freeze.
- Evidence: `docs/experiments/2026-10-30_x4_endurance_relay.md` section 10, `results/analysis/x4_2026-11-04.json`, registry R3519, R3564-R3567.

### F037 2026-11-04 07:24 RESULT (current tree): X3b, eight and nine carriers at the real-time setting (Q007 item 3)
- Section: 4.4, the planner's tick and the 8- and 9-carrier results.
- What changed: 18 runs, all complete, no drop, abort or failed solve. One SQP iteration per tick ("real time") against five, two runs each. The chains use the magnet-marker release check.

  | Cell | 1 iteration | 5 iterations |
  |---|---|---|
  | 8 carriers, hover: position RMSE, tilt peak | 1.4 / 1.4 mm, 0.09 / 0.08 deg | 1.4 / 1.0 mm, 0.07 / 0.05 deg |
  | 8 carriers, figure-eight: path error, tilt peak | 10.6 / 10.4 mm, 1.07 / 1.16 deg | 15.6 / 15.3 mm, 1.21 / 1.27 deg |
  | 8 -> 4 -> 8: release peaks, fold-in peaks | 1.50-2.53 deg, 1.25-1.39 deg | 1.55-2.39 deg, 1.24-1.44 deg |
  | 8 -> 4 -> 8: back at 8, position, tilt | 6.3 / 6.2 mm, 0.92 / 0.96 deg | 6.3 / 6.5 mm, 0.98 deg |
  | 9 carriers, hover | 1.8 / 1.8 mm, 0.07 / 0.05 deg | - |
  | 9 -> 5 -> 9: release peaks, fold-ins, back at 9 | 1.52-2.00 deg, 1.11-1.32 deg, 6.0 / 6.2 mm | - |

  The planner's solve p95 in these lab runs is 60-68 ms at 8 carriers and 82-93 ms at 9 at one iteration, against 234-261 ms at five.
- What it says: one iteration gives the same verdict as five on every bar, and on the figure-eight it tracks closer. The card's C8 holds. Every fold-in stays under 1.5 deg, and nine carriers fly the same chain as eight. Back at 8 or 9 both settings sit 6.0-6.5 mm off the point. That is the same 6.4 mm as today's planner shows without the rod (F030), so it is the integral's offset, not the iteration count. It misses N4's 5.7 mm position bar at both settings, as the card said 5 iterations did before.
- Text should now say: X3b's claim: one iteration per tick is enough at 8 and 9 carriers. The idle tick fits 100 ms (F034). Note the back-at-8 offset beside it, and that it re-flies if the rod flips.
- Evidence: registry R3568-R3585; card `docs/experiments/2026-10-30_x3b_realtime_8_9.md` (C8).

### F038 2026-11-04 07:25 DECISION: X4 keeps 26 leaves; the 1700 s mission is not a concern (Wesley)
- Section: X4, the endurance relay.
- What changed: Wesley 4 Nov: "don't worry about the time limitation for the 1700 s thing". The relay's numbers in F036 are the claim's: 26 of 26 swaps, 1663-1681 s, 2.77-2.80 B, against 0.95 B for the fixed team. No re-fly at 29 leaves.
- Text should now say: write the mission as flown, about 1670 s (2.8 pack budgets). Don't call it a 30-minute mission: it is about 28 minutes. Leave out the card's 1700 s, as the thesis does not report objectives set before the runs (decisions 3 Nov). Still owed before it is final: the final tree's re-fly if X4's code path changes by Friday, and the reviewer on the table.
- Evidence: `docs/decisions.md` (4 Nov), `docs/experiments/2026-10-30_x4_endurance_relay.md` section 10.

### F039 2026-11-04 08:49 RESULT (real world, first look): the first real join
- Section: 4.2 Real World, the join; the abstract and conclusions if Wesley wants it there.
- What changed: on 4 Nov QUAD4, flown by Te-Jen's controller, attached to the ring that three of our quads carried, and our planner folded it in as the fourth carrier: contact, a failed first proof, a second contact, the proof, the resize to four. On the 0.86 kg M2A ring with Te-Jen's relaxed gates (the descent held once at 10 cm and resumed). The same attempt on the 1.15 kg ring before it ended in the 30 deg drop during QUAD4's descent (the ring sank and tilted as QUAD4 came low over it).
- Text should now say: nothing final yet. More attempts follow today. Keep the planned box until the repeats are in.
- Evidence: registry RIG-1104-sq6m, RIG-1104-sq6b; `results/rig/2026-11-04/sq6m.log`.

### F040 2026-11-04 11:20 RESULT (real world, the day's end): the release works, the re-attach and the pickup did not finish
- Section: 4.2 Real World (the join, the release, the pickup); the limitations paragraph.
- What changed: the rest of Wed 4 Nov on the 0.86 kg M2A ring, 13 flights after sq6m (registry RIG-1104-nr9 to -k1m_4).
  - **Release, every time it was asked:** on six flights four quads lifted the ring with QUAD4 welded on plate 9, our planner unloaded it (ring tilt 2.9-5.2 deg at the release), resized to three, stepped QUAD4 out and handed it to Te-Jen's controller, and the three carried on. On LAND or his give-up our planner took QUAD4 back clear of the ring every time (it landed alone at its rest; two flights ended with an operator DISARM).
  - **Re-attach: not yet.** On plate 1 (nr1_2) the magnet caught within 4 mm of the target and the plate rose 18-19 mm, but our confirmation rejected it on its 3 mm tether margin. On plate 9 the descents ended 24-59 mm off the target (the plates are 27.5 mm in radius), so the contacts slid off and the confirmation lost them (nr9m_4, nr9m_5). His takeoff after the release now starts from the air (no dip, no stall), which fixed nr9m_2's stall 25 cm low.
  - **Pickup: picked up and carried, never dropped.** QUAD4 picked the object up and carried it over the ring on three flights, but the object swung on the 1 m cable, its magnet 0.23-0.26 m (median) from the ring's centre, so his arrival check (0.18 m, held 0.5 s) passed at most once in 13-28 s and the drop never started. In k1m_3 his reference then sank 0.4 m with the object at the ring's height, and LAND's take-back lifted it with the ring: tilt 30 deg, our automatic drop (the ring fell, nothing broke).
- Text should now say: the release on the real system can be written up (six flights, the tilt range above, the take-back). The re-attach and the pickup stay planned: neither finished on 4 Nov. Thursday 5 Nov tries again.
- Evidence: registry RIG-1104-nr9 to RIG-1104-k1m_4 (FAIL/VOID rows give the reasons); simulation R3633-R3635 (the route round the outside and the takeoff from the air); `results/rig/2026-11-04/*.log`.

### F041 2026-11-04 15:11 CORRECTION (real world): F040's k1m_3 sentence was wrong; QUAD4's throttle hit a cap
- Section: 4.2 Real World (the pickup); the limitations paragraph.
- What changed: F040 said Te-Jen's height reference sank 0.4 m in k1m_3. It did not. His reference stayed at 2.43 m and
  QUAD4 sank under it, because his controller caps the throttle at 0.60 and, carrying the object, QUAD4 needed that and
  more (pinned at the cap 17 s in k1m_3). Without the object it never reached the cap. Found by Te-Jen in the bags,
  checked here. The rest of F040 stands.
- Text should now say: if the thesis mentions the pickup's failure, the cause is the partner controller's throttle limit
  with the object on board, not its reference.
- Evidence: registry RIG-1104-k1m_3c.

### F042 2026-11-04 17:04 RESULT (current tree, Q007 item 3 complete): X2-full, X8 and the eight-carrier chains
- Section: 4.1 Simulation (X2 the full chain, X8 losing a carrier under a network law, the scalability chains).
- What changed: the rest of Q007 item 3 flew on the lab PC (R3586-R3593 on the 3 Nov tree, R3636-R3674 on the 4 Nov tree;
  the flight code differs only by switches that are off). Scored against the cards; simulation bars judged after the runs
  where the card had none (Wesley 2 Nov).
  - **X2-full (R3586-R3591):** arm A holds in both repeats (square-to-T peak 29.1 / 27.8 deg against 30; fold-ins 2.2-3.3
    deg against 5); arm B holds (peak 58.2 / 59.2 deg against 80; the 31 Oct later-runs bar 3.1 / 2.7 deg against 5, whose
    value still waits for Wesley). Lap 6 path 8.9-9.9 mm against 12.3, the base 9.3 mm.
  - **X8 (R3592-R3593, R3636-R3647), bars frozen 30 Oct:** L1 (hub) holds: the port's E 365 / 370 mm against our 312-315
    mm (+20 mm bar 335). L2 (rim) holds: the port's 446-448 mm against our 236-244 mm, and the port loses the ring (tilt over
    80 deg) where ours does not. L3 holds: no "ours" run loses the ring, the hub settles in 5.6-5.7 s against 10. The exact
    law never told (R3640/41) aborts the fleet at +1.3 s, as the card predicted.
  - **The eight-carrier chains (R3648-R3673, two each; the scalability card's draft bars, never frozen):** every release
    held (peaks at most 3.7 deg). N4 position misses where a chain ends back at eight (6.3-7.1 mm against 5.7). The chain to
    two (chain2back) fails in both: the 8-quad T peaks 28.3 deg and its next release is cancelled by its own unload gate.
    These runs flew the ema filter, their comparators slope.
  - R3674 (the join figure run): one descent, proof on try 1, welded at 46.3 s.
- Text should now say: X2-full and X8 can be written from these numbers, marked as the current tree (re-fly after the
  freeze if their code changes). The reviewer has not seen these tables yet.
- Evidence: registry R3586-R3593, R3636-R3674.

### F043 2026-11-04 17:31 FACT (real world): lab photos for the setup figure
- Section: 3 Experimental setup (the payload, the magnet, the pickup object).
- What changed: Wesley's photos of 4 Nov are in `results/rig/2026-11-04/photos/` (index README.md there): QUAD4's
  electromagnet over the small pickup object (54 mm steel disc in a printed holder, 60 g), the M2A ring from above on a plain
  background (04, the one for a figure; 07 has a brand logo on the mat behind), side on (05: the net's depth) and angled.
- Text should now say: nothing yet; use them when the setup figure is made. The magnet close-ups have a hand in shot and no
  ruler; Wesley may retake those.
- Evidence: the photo files.

### F044 2026-11-04 21:03 FACT (real world): more lab photos, and the setup's measured values
- Section: 3 Experimental setup (the payload, the magnet, the pickup object, the quads); 4.2 if the pickup is reported.
- What changed:
  - **15 more photos** in `results/rig/2026-11-04/photos/` (README.md indexes all 22). Best candidates for a setup figure:
    22 (the M2A ring from directly above, plain, centred), 15 (the ring side on: the truss and the net), 16 (QUAD4's magnet
    caught on a ring plate), 08 and 09 (the small object and the magnet with a ruler), 19 (the electromagnet face from
    below), 13/14 (QUAD1 side on and from above, markers). 17 and 20 are out of focus.
  - **Measured on 4 Nov (props off):** QUAD4's electromagnet face 20 mm across; the M2A ring's steel plates 55 mm; the net's
    lowest point about 5 cm below the ring's bottom when loaded; the small pickup object a 54 mm steel disc in a printed holder,
    70 mm overall, 22 mm thick, 60 g; QUAD4's tether (body to magnet marker, hanging free) 1.034 m.
  - **Results since F043:** none new for the thesis. The day's simulation runs (R3675-R3681) only tested Thursday's
    pickup switches (drop on arrival, the full lift, the higher drop, the throttle limit), not thesis claims.
- Text should now say: the setup section can use these numbers and photos; nothing else changes.
- Evidence: the photo folder; decisions.md 4 Nov (the measurements).

### F045 2026-11-04 22:30 RESULT + DECISION (simulation): the reviewer on X2-full and X8; the rod flip; X2's bar value
- Section: 4.x X2 (the full chain on the figure-8), X8 (the network's loss), and every simulation claim's method.
- What changed:
  - **Reviewer, X2-full (R3586-R3591): supported, with corrections** (card 2026-10-30_x2_full_chain_fig8, section
    "Reviewer"). Every barred number reproduces within 0.1. Corrections for the text: (1) B passes the bar that replaced
    T6 before the re-fly (the largest tilt from +6 s, at most **5 deg: Wesley set the value 4 Nov ~22:20, after the runs**;
    readings 3.10, 2.66); the original T6 bar (5.0 s) is missed in both (35.46, 35.40 s) and stays recorded as missed.
    (2) The T stage runs one carrier at its throttle cap for 6.7-10.4 s before the last weld, and the ring sits 7-14 mm
    low: the fold-in pass (3.00, 3.30 deg) is not independent of throttle headroom. (3) A1's 29.1 / 27.8 deg is the rise
    from the 5 s mean (the bar's quantity); the absolute peaks are 29.7 / 28.4. (4) Lap 6 flies a datum moved 53-58 mm
    (36.6-39.7 mm RMS from lap 1's reference against the base's 10.5).
  - **Reviewer, X8 (R3592-R3593, R3636-R3647): supported, with corrections** (card 2026-10-30_x8_network_loss, section
    "Reviewer"). L1 holds on E (ours 315 / 312 mm, bar 335; port 365 / 370), L2 holds (ours 244 / 236, bar 264; port
    448 / 446), L3 holds. Corrections: (1) the hub pair spans two code trees (the difference is off and bit-identical):
    say so. (2) Do not claim the port loses the ring on the hub (over 300 mm for 0.99 / 1.02 s against a 1 s rule). (3) On
    the rim write "passes the 80 deg drop limit", not "drops the ring" (the ring stayed up, hanging at 45-74 deg). (4) Ours
    on the hub is not "barely moving": the ring swings 0.31 m. (5) Ours on the rim does not settle level in both repeats
    (one grows to 16 deg by LAND, one stays near 7).
  - **The rod in the plan becomes the simulation's default** (Wesley ~22:20; decisions 4 Nov): the planner models the
    rods' mass, and the sideways integral is off wherever it does. SIL and the paper's method (network mode) keep their
    settings; real flights cannot use it. Also the twin's weld gate gets a 0.1 s speed filter by default. Every simulation
    claim re-flies on the final tree after the freeze, so the numbers above are the current tree's, not the final ones.
- Text should now say: X2 and X8 as above with the corrections; the method section can say the planner includes the
  rods' weight in simulation (the real-world planner does not); final numbers come from the post-freeze re-fly.
- Evidence: registry R3586-R3593, R3636-R3647; decisions.md 4 Nov ~22:20; the two cards' Reviewer sections.

### F046 2026-11-04 23:25 DECISION (simulation) + CORRECTION of F045: the plan stays massless; the twin's links changed
- Section: the method (the plan's cable model), the simulation setup (the twin's links), every simulation claim; X1's T.
- What changed:
  - **F045's third bullet is withdrawn.** The rod in the plan is NOT the simulation default: Wesley reversed it at ~22:50
    and chose at ~23:00 to keep the plan massless everywhere (simulation and real world), as the method describes.
  - **Why the T leans** (the physics, corrected): each quad's magnet (about 95 % of magnet + cable, ~0.040 kg in all) sits on
    the plate and hangs on the cable, while the plan counts it as part of the quad. On an uneven formation (the T) that
    leaves a small sideways pull on the ring that the plan does not expect (~0.23 N per plate at 60 deg on the real cables),
    and the height-and-position integral leans the ring to meet it. This is true of the real system as well as the twin.
    Reported as a limitation.
  - **The twin's links now match the hardware:** each link is a 0.002 kg cable plus a 0.038 kg magnet at the plate (was a
    0.040 kg rod plus a 0.035 kg magnet), each quad's airframe 0.510 kg, so each quad with its cable is still 0.55 kg. Every
    simulation result before 4 Nov 23:15 flew the old link. The T's lean should shrink by about a third; the first runs on
    the new links are queued tonight.
- Text should now say: the planner models taut massless cables in both simulation and experiment; the simulated links are
  a light cable with the magnet's mass at the plate (masses above). Do not describe a "rod mass in the plan".
- Evidence: decisions.md 4 Nov ~22:50, ~23:00 (CORRECTION, DECIDED, BUILT); tools/make_carry_worlds.py.

### F047 2026-11-05 00:45 CORRECTION of F046 (simulation): the T leans more on the new links, not less
- Section: X1's uneven cell (the square to the T), the limitation text of F046.
- What changed: F046 said the T's lean "should shrink by about a third" on the new links. It did the opposite. With the
  sideways integral on (the default) the T settles at 17.83 and 16.50 deg (R3699, R3700) against 8.30 and 10.52 deg on the
  old links (R2965, R2972, an older code tree). With the sideways integral off it settles at 1.77 deg, 47 mm off its point
  (R3701). Cause: a second planner effect (a 0.1 N minimum planned pull on the carrier with no share of the load) runs away
  with the sideways integral; the old heavy rod partly offset it. The detach-pickup-drop-reattach holds on the new links
  (R3702).
- Text should now say: nothing new yet for the thesis; do not use F046's "shrinks by a third". X1's T numbers are not final
  (every claim re-flies after the freeze).
- Evidence: registry R3699-R3702; decisions.md 5 Nov ~00:45 CORRECTION.

### F048 2026-11-05 11:20 RESULT (real world, exploratory) + FACT: the lab day of 5 Nov
- Section: the real-world chapter (the combined attach, the pickup), the setup (motion capture), limitations; the simulation
  method (replays).
- What changed (registry RIG-1105-*, decisions.md 5 Nov FINDING):
  - **The first real detach and re-attach** (RIG-1105-sq8_2): four quads on a square (plates 11, 2, 5 + QUAD4 on 8);
    QUAD4 detached, flew out, came back and caught its plate on the first try (17 mm off), was confirmed and folded back in
    14.4 s after the release. While it was away the three carriers held the ring as a T leaning 21-28 deg (the simulation
    predicted ~8.5 deg). The ring was then dropped during the landing by a false loss detection (a timing artefact), so
    the flight has no clean landing.
  - **The pickup and drop into the flying ring worked on the rig** (RIG-1105-k1d): QUAD4 alone picked up the small object
    (60 g) first try, lifted it straight up 1.39 m and dropped it into the ring's net while the other three hovered.
    With the ring moving on a 0.3 m circle the bigger object also landed in the net, 4 cm from the centre (RIG-1105-k1ob_2),
    but the ring then jerked to over 54 deg and the flight ended in an automatic drop (not explained yet).
  - **The circle-pause on the rig** (RIG-1105-nr9o_2): the circle stopped when QUAD4 started its re-attach, the ring stayed
    still, and the circle resumed afterwards. The re-attach itself did not catch (the magnet came down 3.5-8 cm off the
    plate, the catch needs 2.5 cm); the same on the triangle in hover (nr9m).
  - **Setup fact:** a recording running in the motion-capture software doubled the command-to-response delay on every
    quad (45-65 ms -> 75-135 ms) and made every lift bounce at ~1 Hz (two automatic ring drops); with it stopped the
    lifts were clean. Worth one line in the setup or limitations: the loop is sensitive to mocap latency.
  - **Simulation replays deleted** (disk full): the Gazebo replay recordings of every simulation run so far are gone;
    run.csv, logs, metrics and plots remain. A video of a simulation run needs that run flown again.
- Text should now say: the real-world chapter can report a detach and re-attach and a pickup-and-drop into the flying ring
  as demonstrated once each (exploratory, single flights, no frozen bars), with the T's real lean as a measured limitation.
  Not claims: no repeats, no card.
- Evidence: registry RIG-1105-k1d, -sq8_2, -nr9o_2, -k1ob_2; results/rig/2026-11-05/ (logs, bags, launches.csv, scripts/).

### F049 2026-11-05 12:30 CORRECTION of F048 (simulation number) + FACT (real world): the T's simulated lean; the false drop
- Section: the real-world chapter (the detach and re-attach on the square), limitations.
- What changed:
  - **The simulated T lean is a range, not ~8.5 deg.** Three runs of the same square line on today's code lean 11.2-12.0
    deg (median, while QUAD4 is away; registry R3712, R3714, R3715) against 8.5 in R3708 (01:30). The cause of the
    difference is not found (same settings; not the CPUs; not today's fixes). Use "about 8-12 deg in simulation" against
    the real 21-28 deg (RIG-1105-sq8_2).
  - **sq8_2's drop at landing was a false loss from late position data:** in that flight the planner received the three
    carriers' positions ~0.6 s late (the ring's ~0.15 s); when the landing moved the ring down, a carrier's cable read
    9 cm too long and the loss check dropped the ring. Every other flight that day had all positions 60-150 ms old.
    Wesley decided (5 Nov) not to add a fix for it; the planner now logs the timestamp gap of every position.
- Text should now say: the simulated lean of the T as about 8-12 deg; the landing drop of sq8_2 as a false detection
  caused by late position data, not a magnet release.
- Evidence: registry R3708, R3712-R3715, RIG-1105-sq8_2; decisions.md BUILT and FINDING 5 Nov.

### F050 2026-11-05 17:54 DECISION (simulation + real world): what the weekend re-fly and the next lab days fly
- Section: the simulation method and results (4.1-4.3, X1-X4, X8, X11), the real-world method (landing), limitations.
- What changed (decisions.md 5 Nov; registry R3712-R3770):
  - **Every simulation claim re-flies on one frozen tree** this weekend (Sat 7 - Sun 8; 255 runs): no claim run had flown the
    simulation's new links of 4 Nov. Until then the claim numbers in F035-F047 are interim.
  - **Every simulated cell that leaves a T flies with the sideways integral off** (Wesley's 31 Oct fallback, since the
    rod-in-plan fix was reversed): X1's T cell, X2's three-carrier stage, X3's chain2back, X8's rim and the three 4.3 joins
    that lift a T. The text states this cancellation once. The gap table re-flies with the integral as flown.
  - **No more 1.15 kg cells:** only X1 and X4 have one; the text says which tables do.
  - **The 4.3 pickup-then-re-attach cell flies QUAD4 over the carriers** (~3.25 m), the one exception to "QUAD4 never flies
    over the carriers": on the low route the partner's route planner stops 0.7 m short of the drop point. It re-attached and
    the object landed in the net 3 mm from the centre (R3768).
  - **The partner's route planner checks each carrier against its plan to 0.20 m** (was 0.05 m; its collision margin stays
    0.05 m): with the integral off, the T hovers 5 cm off its plan and the old bound switched the planner off.
  - **The real ring's LAND (from the next lab day):** the height reference keeps descending until each quad stops on the
    floor; the stall test restarts when the ring touches down; while the LAND waits for a taken-back QUAD4 each carrier on
    the landed ring is disarmed on its own once still within 15 cm of its rest. Why: on 5 Nov the carriers hovered
    15-23 cm up and drifted 17-102 cm while waiting. Simulation check: R3755-R3770.
- Text should now say: the method describes these settings; the results wait for the weekend's numbers (a new feed entry
  per claim when scored).
- Evidence: decisions.md 5 Nov; docs/claim_batch_2026-11-07.md; registry R3712-R3770.

### F051 2026-11-05 18:35 FACT (real world): why the ring dropped after the bigger object's drop (F048 said "not explained yet")
- Section: the real-world chapter (the pickup and drop into the flying ring), limitations.
- What changed: RIG-1105-k1ob_2 is explained (registry RIG-1105-k1ob_2c). The bigger object (~0.15 kg) fell 0.53 m into the
  net and hit it 14 cm off centre at ~2.8 m/s; the impact pulled two carriers' magnets off their plates (about 16 N against
  the magnets' ~10 N hold), the ring swung down and the third magnet let go; the automatic drop came after. Not a control or
  motion-capture fault. The 60 g object, dropped the same way (k1d), hit 11 cm off centre and every magnet held. Two numbers
  in F048 were wrong: the object hit the net 14 cm from the centre (the 4 cm was measured mid-swing), and the 93 deg was a
  frozen pose after motion capture lost the ring.
- Text should now say: a heavier object's impact can exceed the magnets' hold (a limitation of magnetic attachment for drop
  delivery); the small object's drop is the result.
- Evidence: registry RIG-1105-k1ob_2, -k1ob_2c, -k1d; decisions.md FINDING 5 Nov.
