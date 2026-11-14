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

### F052 2026-11-05 20:35 RESULT (real world, exploratory) + CORRECTION of F048: every attach, re-attach and pickup descent of 4-5 Nov
- Section: the real-world chapter (the attach and re-attach, the pickup and drop), discussion of the attach reliability,
  limitations.
- What changed: one table of every descent of QUAD4's magnet toward a plate or the object on 4-5 Nov (64: 46 plate
  descents, 15 pickups, 3 drops into the net), read from the bags (registry RIG-1105-joins). The miss is the magnet's
  horizontal distance from the target centre when it first reached the target's height; the plates are 5.5 cm across.

  | kind | carriers | ring | descents | at target height | contacts | caught + proven | miss at height, cm: median (range) |
  |---|---|---|---|---|---|---|---|
  | attach from the mat | triangle | hover | 3 | 2 | 2 | 1 | 2.4 (1.9-2.8) |
  | re-attach | triangle | hover | 37 | 31 | 11 | 0 | 6.2 (0.3-11.0) |
  | re-attach | triangle | moving (paused) | 4 | 2 | 0 | 0 | 8.6 (8.3-8.9) |
  | re-attach | square | hover | 2 | 1 | 1 | 1 | 1.6 |
  | pickup | (object on the floor) | hover | 10 | 10 | 8 | 5 | 3.4 (1.7-12.9) |
  | pickup | (object on the floor) | moving | 5 | 5 | 4 | 1 | 6.1 (2.0-10.2) |

  All three drops landed in the net, 6.3-13.3 cm from its centre. Why the re-attach missed (31 misses, all on the triangle):
  mostly radial (21 of 31), outward and inward about equally; Te-Jen's reference was within 3.7 cm of the plate, QUAD4's body
  0.2-6.2 cm off his reference, and the magnet swung a few cm under the body; in 27 of 31 misses the body's offset and the
  swing were on the same side and added, while in the one square catch they cancelled. QUAD4 itself was slow (0.05 m/s).
  The triangle's 4 on-plate descents were all on 4 Nov; on 5 Nov it put 0 of 19 on the plate.
- Corrections: F048's "the magnet came down 3.5-8 cm off the plate" for the moving ring: 2 of 4 descents reached the plate's
  height, 8.3-8.9 cm off; the other two stopped 13-14 cm above it (RIG-1105-nr9o_2c). RIG-1105-fm9y_2's re-attach misses were
  5-11 cm sideways, not ~2 cm high (RIG-1105-fm9y_2c). The square's catch came with the body 2-4 cm outward and the magnet
  swinging 4-6 cm inward (the notes for Te-Jen had it the other way round; fixed).
- Text should now say: on the rig the plate attach and the pickup worked (one re-attach caught, on the square; pickups 6 of
  15 proven), but the re-attach on the triangle never caught: the magnet came down a median 6 cm off a 5.5 cm plate, mostly
  radially, from QUAD4's own position error and the magnet's swing adding up. One square catch is too few to credit the
  layout. Exploratory: single flights, no card, no bars.
- Evidence: registry RIG-1105-joins, -nr9o_2c, -fm9y_2c, RIG-1104-*, RIG-1105-*; results/analysis/rig_joins_2026-11-04_05/
  (joins.csv, summary.md: per flight, per descent, pickups, drops, caveats; the 4 Nov plate height is uncertain by ~1 cm).

### F053 2026-11-05 22:59 FACT (real world): which ring the 1 m flights fly (no earlier entry said it)
- Section: the real-world method (the setups), every real-world 1 m table and figure.
- What changed: since 4 Nov every real flight except sq6b (4 Nov) flies the 0.86 kg M2A ring on 1 m cables, and the Wed 11
  / Thu 12 Nov sheets do too (decisions.md DECIDED 5 Nov ~14:40 (1); the 1.15 kg ring flew 29-30 Oct and sq6b). The
  thesis's setups are "Setup 1" (0.86 kg, 0.55 m cables) and "Setup 2" (1.15 kg, 1 m cables) (decisions.md 2 Nov 23:03): the
  4-5 Nov flights (feed F048, F052) and the coming 1 m baselines, release and re-attach fit neither. How the thesis names
  this third combination is Wesley's call (asked by the lab chat, 5 Nov night).
- Text should now say: each real-world 1 m row names its ring; nothing from 4 Nov on is "Setup 2" unless Wesley says so.
- Evidence: decisions.md 5 Nov ~14:40; registry RIG-1104-*, RIG-1105-* (the notes column: "M2A ring 0.86 kg, 1 m cables").

### F054 2026-11-06 00:11 RESULT (simulation, claim) + STALE: X4 the endurance relay on the frozen tree
- Section: the results (the endurance relay, tab:res_x4, fig:res_x4), the abstract, the conclusion, limitations.
- What changed: X4 re-flew on the frozen tree (bf52a8a; the twin's hardware-matched links and the massless plan); every
  number in F036/F038 and in the draft (1670 s, 2.8 budgets, "no quadrotor flew longer than its budget", tilt under
  1.8 deg) is replaced. Wesley ~21:55 5 Nov: report it as flown (decisions.md).

  | Run | Swaps | Flight time (s) | Over B (600 s) | Longest pack (s) | Largest tilt at an event (deg) | Lap error first, last, worst (mm) |
  |---|---|---|---|---|---|---|
  | Relay, 0.86 kg (R3772) | 26 | 2118 | 3.53 | 707 | 2.52 | 15.2, 14.8, 27.9 |
  | Relay, 0.86 kg (R3773) | 26 | 2109 | 3.51 | 703 | 2.61 | 15.4, 14.9, 27.9 |
  | Relay, 1.15 kg (R3774) | 26 | 2105 | 3.51 | 706 | 3.01 | 15.6, 16.8, 26.0 |
  | Fixed team, 0.86 kg (R3775) | - | 568 | 0.95 | 568 | - | 17.2, 15.1, 17.2 |
  | Fixed team, 0.86 kg (R3776) | - | 568 | 0.95 | 568 | - | 15.1, 15.3, 16.4 |

  The card's bars (written before the runs): R1 (26/26 swaps, mission >= 1700 s), R2 (>= 7 carriers), R3 (event tilt <= 5
  deg), R5, R6 and F1 hold. R7 MISSES: every quadrotor flew 695-707 s against its 600 s budget. R4 is not confirmed: the
  last lap is no worse than the first (x0.97-1.08), but 26 of 28 laps (those flown with 7 carriers, 21-28 mm) are over its
  every-lap bound of 19.4 mm (1.25 x the fixed team's mean lap, 15.5 mm). Why both moved: a departed quadrotor's return
  (from the start of its rejoin to its cable end on the plate) takes 35 s median against 18 s on 3 Nov: the twin's
  magnet mass now sits at the cable's end (as on the hardware), so its swing settles more slowly before the weld. The
  path pauses for each return, so each lap and each pack interval grew (laps 79 s against 62 s).
- Text should now say: the relay kept the ring flying 2105-2118 s, 3.5 times one budget and 3.7 times the fixed team, with
  all 26 departures and returns and at least seven carriers throughout; but each quadrotor flew about 700 s, so with 600 s
  packs this rota would not close (it needs faster returns or ~12 min packs), and the laps flown one carrier short track
  worse (21-28 mm) than the full fixed team (15.5 mm). Remove "no quadrotor flew longer than its budget". Mission over the
  longest pack is about 3.0.
- Evidence: registry R3772-R3776, R3772c; results/analysis/x4_2026-11-05.json (tools/x4_metrics.py); decisions.md 5 Nov
  ~21:55; the rejoin comparison R3519/R3564 vs R3772/R3773 and R3483 vs R3741 (both in the registry notes).

### F055 2026-11-06 08:19 RESULT (simulation, claim) + STALE: X3 (8-quad events) and X3b (8 and 9 carriers in real time) on the frozen tree
- Section: the results (scalability, eight quadrotors, the solve-time figure's simulation panel), limitations.
- What changed: X3 (24 runs, R3779-R3802; R3793 void, re-flown after the batch) and X3b (18, R3803-R3820) re-flew on the
  frozen tree. Every release was still detected in time (L holds in every case); the returns got slower, as in X4 (F054).
  - **X3, 5 iterations:** J (each return: on its plate within 30 s, the planner one bigger within 20 s of the ATTACH,
    fold-in <= 5 deg) now misses in EVERY case with returns (pairs, relay, the task and relblend held it on 4 Nov): on the
    plate 13-40 s after the start of the return and the planner one bigger after 13-40 s. chain2back now reaches the pair
    (8 -> 2 -> 8; it failed on 4 Nov), but one return's fold-in peaks at 7.3-7.5 deg (bar 5). P (path against the same
    path with no event) misses on fig8rel2 (21.3-21.5 > 19.0 mm, as 4 Nov), chain4b2b_orbit (28.0-28.4 > 21.3 mm, 2 returns
    refused, the hover point walked 0.82 m; it held on 4 Nov) and one of the two fig8laps chains (19.6 > 18.1 mm). The tasks'
    ring error at n = 5-7 is 10.2-10.7 mm against N1's 5.16. adjpair and the orbit hold.
  - **X3b, 1 iteration:** 8 carriers: C8 CONFIRMED in hover and on the figure-8 (solve p95 59-64 ms; the 5-iteration
    comparators 231-252 ms); the chain is FALSIFIED by one P item (R3807's /drone_5 fold-in 3.97 deg against 2.95; R3808
    0.93). 9 carriers: N9 CONFIRMED in hover (solve p95 84-89 ms); the 9 -> 5 -> 9 chain FALSIFIED by J (fold-ins up to
    5.2-7.0 deg against 5).
  - Validity: RTF ~0.22 (flagged under 0.25, no release missed), late ticks under 0.2 %, no failed solve.
- Text should now say: at eight quadrotors (and nine at one iteration) the planner holds the ring through every release,
  and one iteration per tick matches five in hover and on the figure-8; the returns are slower than the card's 30 s / 20 s
  on the final simulation (the cable's magnet mass, as F054), and the nine-quadrotor chain's fold-ins reach 7 deg.
- Evidence: registry R3779-R3820; results/analysis/x3_2026-11-06.json, x3b_2026-11-06.json (tools/x3_metrics.py,
  x3b_metrics.py); the cards docs/experiments/2026-10-23_scalability.md (J, N, P) and 2026-10-30_x3b_realtime_8_9.md (bars
  frozen 30 Oct).

### F056 2026-11-06 10:02 CORRECTION of F054 (and of F055's cause): X4 after its reviewer
- Section: the results (the endurance relay; scalability), limitations.
- What changed (the reviewer recomputed every F054 number from the raw logs: all within 3 %; one tree; verdict SUPPORTED
  WITH CORRECTIONS; registry R3772c2):
  - **The ring:** the relays fly the 0.9 kg ring (as the draft's table says), not 0.86 kg.
  - **The cause, as far as it is known:** the slower returns (34-35 s against 18-19 s on 3 Nov) are measured, and the weld
    waits on the cable end's own motion with the ring and the drone still. They come from the change of the simulated
    cable links on 4 Nov (by elimination: nothing else in the return changed). F054's and F055's mechanism ("the magnet's
    mass now at the cable end", "~3x the swing inertia") is WRONG: the old link already had a 0.035 kg magnet at that end,
    and its swing inertia went down, not up. Write it as an observation of the final simulation, without a mechanism.
  - **R4 and the tilts are a different cause:** the 7-carrier laps (20-28 mm) and the release tilts (2.5-3.0 deg) follow
    the massless plan (Wesley 4 Nov: the plan no longer counts the cable's mass), not the slower returns: a 2 Nov relay with
    the massless plan and the old links already had 21-28 mm and 2.2 deg; with the cable in the plan it was 9.5-14.4 mm.
  - **Two overstatements to avoid:** "3.5 times one budget" (no quadrotor kept to its budget; the mission over the longest
    pack is 3.0), and R1's 1700 s now passes only because the swaps slowed (the same 26 leaves). R3772's slowest return
    reached its plate at 45.0 s, the planner's limit for a return: the 26 of 26 had no margin.
- Text should now say: the relay kept the 0.9 kg ring flying about 2110 s with all 26 returns, about 3.0 times the longest
  time any quadrotor flew and 3.7 times the fixed team; each quadrotor flew about 700 s, over a 600 s budget; returns took
  about 35 s in the final simulation; the laps flown one carrier short track 20-28 mm, the cost of the massless plan.
- Evidence: registry R3772-R3776, R3772c, R3772c2; decisions.md CORRECTION 2026-11-06; the reviewer's scripts in the lab
  chat's scratchpad (review_x4/).

### F057 2026-11-06 10:04 CORRECTION of F055: X3 and X3b after their reviewer
- Section: the results (scalability, eight and nine quadrotors), limitations.
- What changed (the reviewer re-ran both scorers and reproduced them exactly; one tree; registry R3779c):
  - **Returns that failed:** three returns never welded (each gave up at the planner's 45 s limit: fig8laps R3781 ended at
    7 quadrotors; the moving-ring chain4b2b_orbit R3795/R3796 then refused two more returns and ended at 5). F055 did not say
    so. The on-plate times span 10.4-40.4 s. By the rescope's rule every X3 case with returns is FALSIFIED.
  - **chain2back** (8 -> 2 -> 8, the T flown with the sideways integral off): two fold-ins miss (5.5-7.5 deg), and the
    square-to-T detach tips the ring 28.8-31.8 deg. Also missed and not in F055: fig8rel2's spacing (E4, 0.69 m) and
    chain4b2b's N4 (5.50 mm, one valid run). The claim chain's N4 holds (4.93-4.96 mm against 5.16).
  - **X3b:** C8 is FALSIFIED as a whole by the card's own fold-in rule (one fold-in 3.97 deg against 2.95, within the repeat
    spread); "confirmed in hover and on the figure-8" is a per-cell reading, not the card's verdict. One iteration's whole
    tick p95 is 104-118 ms, so "real time" is not shown in the simulation. N9 is FALSIFIED (both 9-quadrotor chains miss J:
    a return at 36.2 s, the cable end 2.5-2.8 cm off, fold-ins to 7 deg; 3 Nov's flew at most 1.32 deg), and N6 misses
    in both (a published solve residual of 0.015 at a fold-in). Late planner ticks reach 0.40 %.
  - **The cause of the slower returns is NOT established** (this also corrects F056's "by elimination"): the 8- and
    9-quadrotor worlds already had the magnet at the cable end, and the planner and tracker code changed between the trees.
    All the extra time is in the last hold over the plate. The lab chat is looking into it.
- Text should now say: at eight quadrotors every release still holds the ring (L, the adjacent pair, the orbit), and one
  iteration per tick flies hover and the figure-8 like five; the returns are the weak point in the final simulation (slow,
  three gave up), and the nine-quadrotor chain is not shown. Do not write "real time" for the simulation.
- Evidence: registry R3779-R3820, R3779c; results/analysis/x3_2026-11-06.json, x3b_2026-11-06.json; the reviewer's re-runs in
  the lab chat's scratchpad (review_x3/).

### F058 2026-11-06 10:24 FACT (simulation) + CORRECTION of F056/F057: why the returns got slower
- Section: the simulation method (the cable model), the results with returns (X2, X3, X4, the rejoin), limitations.
- What changed: an A/B on the frozen code, one run each (exploratory; registry R3907, R3908): the same 3-quadrotor return
  (rj3_pairrelease_rejoin_L100_tip) took 37.0 s with the final simulated quadrotors and cables, and 16.6 s with the ones of
  before 4 Nov (as on the old tree: 16.6-17.7 s). So the slower return comes from the model change of 4 Nov (each drone
  still 0.55 kg, but the mass moved from the cable into the airframe: cable 0.075 -> 0.040 kg, airframe 0.475 -> 0.510 kg,
  matched to the hardware), NOT from a code change. The extra time is in the last hold over the plate: the cable end
  closes on its plate point slowly (7.8 -> 2.7 cm in ~14 s against 5.9 -> 1.1 cm). The mechanism is not identified (no
  controller setting assumes the old cable mass). F057's "the code changed too" is ruled out as the cause.
- Text should now say: with the quadrotor and cable masses matched to the hardware, a returning quadrotor needs about 35 s
  to settle its cable end on the plate (about 17 s with the earlier model); the release side is unaffected. A limitation of
  the return manoeuvre, not of the planner's resize.
- Evidence: registry R3907, R3908 (the old models outside the repo: ~/mdc_overnight/oldlinks/, from commit 9f20f8b).

### F059 2026-11-06 11:23 RESULT (simulation, claim): X1 the release ablation on the frozen tree
- Section: the results (the unannounced release: what the resize is worth), the conclusion.
- What changed: X1 re-flew on the frozen tree (43 runs, R3821-R3822, R3865-R3906; R3903 void, re-flown after the batch;
  final quadrotor masses: X1 has no returns). Scored with tools/x1_metrics.py (results/analysis/x1_2026-11-06.json). Arms per
  cell: the proposed planner (resize at detection), no resize, and the resize delayed 0.5 / 1 / 2 s.

  | Cell | Tilt rise, resize vs none (deg) | Settled tilt (deg) | Settle time T6 (s) | Card verdicts |
  |---|---|---|---|---|
  | 4 quads hover (h4) | 6.4-7.0 vs 7.0 | 0.03 vs 0.20-0.23 | 3.8-3.9 vs 5.8 | B1, B2 FALSIFIED; B4 holds |
  | 8 quads hover (h8) | 1.6-1.8 vs 1.6-1.7 | 0.80 vs 0.91-0.93 | 4.4-4.5 vs 3.1 | B1, B2 FALSIFIED; B4 holds |
  | 4 quads figure-8 (f4) | 7.1-7.2 vs 7.1-7.5 | 2.5-2.6 vs 2.6 | 12.7-12.9 vs 12.7-13.2 | reported |
  | 4 quads, 1.15 kg (h4p115) | 10.6 vs 10.1-10.5 | 0.05 vs 0.22-0.23 | 4.7 vs 3.4-4.4 | reported |
  | square to T (sqTnoxy) | 59.2-61.6 vs 58.2-62.3 | 1.74-1.76 vs 1.78-1.80 | 4.3-4.7 vs 6.3 | U1, U2 INCONCLUSIVE; U5 holds |

  Every release in every arm was caught (detection 0.29-0.40 s) and nothing dropped. The delayed resizes (0.5-2 s) sit
  between the two with no order. As on the first pass (30 Oct): the resize does not lower the release's tilt peak or dip;
  its effect is on the settled level after it (4 quads: 0.03 against 0.2 deg) and, for 4 quads and the square-to-T, a
  shorter settle; at 8 quads none is measurable. The square-to-T release tips the ring ~60 deg in every arm (56-58 deg on
  30 Oct) and the T now settles to 1.7 deg (6.8-13 deg on 30 Oct, with the sideways integral on).
- Text should now say: the release is caught and held in every case; the resize at detection is what leaves the ring
  level afterwards (small, a fraction of a degree), not what limits the release's own transient. X1's claim that the resize
  keeps the ring up is falsified as written (B1, B2); report the cells as measured. Reviewer next.
- Evidence: registry R3821-R3906; results/analysis/x1_2026-11-06.json; card docs/experiments/2026-10-30_x1_release_ablation.md.

### F060 2026-11-06 11:30 CORRECTION of F059: X1 after its reviewer
- Section: the results (the unannounced release), the conclusion.
- What changed (the reviewer recomputed every number from the logs: they match; one tree; SUPPORTED WITH CORRECTIONS;
  registry R3821c):
  - **What the resize does, as measured:** it leaves the release's own transient unchanged (tilt rise and dip within the
    repeat spread) and leaves the ring 0.1-0.2 deg more level afterwards (4 quads 0.03 vs 0.2 deg, 8 quads 0.81 vs 0.92 deg);
    in the square-to-T release it lowers the dip by ~8 mm (97-102 vs 104-112 mm). F059's "at 8 quads none is measurable" and
    "a shorter settle" are WRONG: the resize does not settle faster (4 quads: last exit from the band 6.8 vs 5.8 s).
  - B2 is a card bar in the figure-8 cell too (falsified there). The delayed resizes do not sit in order between the two.
  - **The square-to-T cell depends on the simulated masses or the tree:** on the earlier masses and the 30 Oct tree the T
    settled 0.6 deg with the resize against 2.2 deg without (R2998/R2999, R3202/R3203); now 1.74 vs 1.80 deg.
- Text should now say: every release was caught in 0.29-0.40 s with no loss; the resize at detection does not change the
  release's transient, and leaves the ring a fraction of a degree more level afterwards. Do not claim that it keeps the ring
  up, that it settles faster, or any order of the delayed resizes. The square-to-T result waits on Wesley's call on its masses.
- Evidence: registry R3821-R3906, R3821c; results/analysis/x1_2026-11-06.json.

### F061 2026-11-06 14:31 DECISION (thesis framing, setups, catch radius; Wesley ~14:30)
- Section: the abstract, the results' framing, the methods (setups), the real-world results.
- What changed (decisions.md DECIDED 2026-11-06 ~14:30):
  - **Structure stays:** simulation results first, then the real world.
  - **Framing within it:** X1 (the release ablation) is a supporting mechanism result, not a headline: every release is
    caught and held, and the resize leaves the ring a fraction of a degree more level (F059/F060). The real-world detach,
    re-attach and pickup are demonstrations with their counts (feed F052: one re-attach caught of 43 plate descents, one
    attach from the mat, 6 of 15 pickups proven; all three drops in the net). The abstract's "quadrotors ... re-attached"
    becomes singular, with its conditions (the square layout, hover).
  - **Setup 3:** the 0.86 kg ring on 1 m cables (every real flight since 4 Nov except sq6b, and the coming sessions; F053).
  - **Catch radius as flown:** 25 mm for the real attach's confirmation, 10 mm for the simulation's weld (not 45 mm).
- Text should now say: as above; X4 and X3's numbers wait for the old-mass re-fly (running; F058 and decisions 6 Nov).
- Evidence: decisions.md 6 Nov; feed F052, F053, F059, F060.

### F062 2026-11-06 16:13 RESULT (simulation, claim): X4 the endurance relay, FINAL (the earlier quadrotor masses; replaces F054/F056's numbers)
- Section: the results (the endurance relay, tab:res_x4, fig:res_x4), the abstract, the conclusion.
- What changed: X4 re-flew on the quadrotor masses of before 4 Nov (Wesley 6 Nov: the claims with returns fly them; the
  thesis does not explain the two mass splits). These are the numbers the thesis uses (R3910-R3914; F054's R3772-R3776 stay
  in the registry as a recorded comparison only).

  | Run | Swaps | Flight time (s) | Over B (600 s) | Longest pack (s) | Largest tilt at an event (deg) | Worst lap (mm) |
  |---|---|---|---|---|---|---|
  | Relay, 0.9 kg ring | 26 | 1764 | 2.94 | 581 | 2.55 | 34.0 |
  | Relay, 0.9 kg ring | 26 | 1761 | 2.94 | 580 | 2.56 | 31.4 |
  | Relay, 1.15 kg ring | 26 | 1751 | 2.92 | 575 | 3.01 | 31.6 |
  | Fixed team, 0.9 kg ring | - | 568 | 0.95 | 568 | - | 16.5 |
  | Fixed team, 0.9 kg ring | - | 568 | 0.95 | 568 | - | 16.2 |

  The card's bars (written before the runs): R1 (26 of 26 swaps, flight time >= 1700 s), R2 (>= 7 carriers), R3 (event
  tilt <= 5 deg), R5, R6, R7 (every quadrotor inside its 600 s budget: longest 575-581 s) and F1 hold. R4 is not
  confirmed: the last lap is no worse than the first (x0.93-1.04), but the laps flown one carrier short track 31-34 mm
  against its 19.2 mm every-lap bound (1.25 x the fixed team's 15.4 mm): the cost of the massless plan (F056). Returns
  took 20.8-21.5 s (median). Two relays' event lists miss their last weld (LAND came right after it); the planner logs
  show all 26 (the scorer's "25" is that gap).
- Text should now say: the relay kept the ring flying 1751-1764 s, 2.9 times one 600 s budget and 3.1 times the fixed
  team, with all 26 departures and returns, at least seven carriers throughout and no quadrotor over its budget; the
  largest tilt at a release or return was 3.0 deg; the laps flown one carrier short tracked worse (31-34 mm) than the full
  team (15-17 mm).
- Evidence: registry R3910-R3914; results/analysis/x4_oldlinks_2026-11-06.json (tools/x4_metrics.py); decisions 6 Nov.

### F063 2026-11-06 16:20 CORRECTION of F062: X4 final, after its reviewer
- Section: the results (the endurance relay).
- What changed (the reviewer rebuilt every number from the logs: all within 1 %; one tree; no void; SUPPORTED WITH
  CORRECTIONS; registry R3910c): the table and the verdicts of F062 stand. Corrections to its text:
  - All three relays did 26 of 26 (each planner log has 26 welds; the last lap flew with eight). The two missing event lines
    are a logging gap, not LAND coming right after the weld (LAND came ~30 s later).
  - "31-34 mm" is each run's worst lap. The laps flown one carrier short span 21-34 mm (median 28-30 mm): about 21 mm at the
    second lap, rising to about 30 mm by the ninth, then level. The full team's laps are 15-17 mm.
  - Do not give a cause for the worse laps: the massless plan fits, but it was never tested on one code tree.
  - The flight time clears 1700 s partly because each return takes ~3 s longer than on 3 Nov with the same masses
    (21 s against 18.5 s). With the 8 s pad dwell counted, the longest pack is 583-589 s, still under 600 s.
- Text should now say: as F062, with the laps one carrier short "21-34 mm, rising over the first laps" and no cause given.
- Evidence: registry R3910-R3914, R3910c.

### F064 2026-11-06 18:02 FINDING (real world, exploratory): one-quad tests of funnel v4 and the pickup (Fri lab, 17:18-17:50)
- What: QUAD4 alone (Te-Jen's controller), the M2A ring resting on the floor with funnel v4 (TPU) on plate 9; then
  QUAD4 alone picking up the small object. No other quad flying. Exploratory, judged after the flights, no bars.
- Funnel: 2 of 2 flights seated the magnet on plate 9 on the first descent. The magnet started its descent 3.9 and
  7.2 cm off the plate and ended 0.8 and 0.3 cm from its centre (a 7 cm miss never touched the bare plate on 5 Nov).
  Whether the magnet latched was judged by eye (this setup has no attach proof).
- Pickup: 1 of 1 caught on the first try at 0.9 cm (with the carriers flying on 4-5 Nov: 7 catches in 11 tries at
  1.6-3.5 cm). One flight: not yet a comparison.
- Also: the first launch crashed (two stacks drove QUAD4's radio at once; operator commands reached only one of them);
  fixed in the lab scripts, nothing for the text.
- Text: nothing to write yet. The paper's attach numbers come from the funnel flights with all four quads (Wed 11 Nov).
- Evidence: registry RIG-1106-q4f_5, -q4f_6, -q4p (and the void RIG-1106-q4f, -q4f_2, -q4f_c).

### F065 2026-11-07 08:13 RESULT (simulation, claim): X3 the 8-quadrotor events, FINAL (the earlier quadrotor masses; replaces F055/F057's X3 numbers)
- Section: the results (scalability, eight quadrotors), limitations. X3b (8 and 9 at one iteration) follows separately.
- What changed: X3 re-flew on the quadrotor masses of before 4 Nov (Wesley 6 Nov: the claims with returns fly them):
  32 runs, 16 cases x2, one code tree, no void, no failed solve, no late-tick problem (RTF ~0.22, flagged, no release
  missed). These are the numbers the thesis uses (R3922-R3927, R3954-R3979; F055/F057's R3779-R3820 stay in the
  registry as a recorded comparison only). Bars as written on the card (N1/N4 frozen 23 Oct; the harder cases' draft
  bars judged after the runs, as before).

  | Case (x2) | Returns welded | On the plate (s) | Planner one bigger (s) | Fold-in peak (deg) | Verdict |
  |---|---|---|---|---|---|
  | The claim chain 8 -> 4 -> 8 (chain4back) | 8 of 8 | 15.5-18.4 | 15.6-18.5 | 1.2-1.8 | J holds; N4 MISS (6.45-6.56 mm against 5.65) |
  | The task (four leave, work, return) | 8 of 8 | 13.7-16.2 | 13.8-16.3 | 1.1-1.8 | J holds; N4 MISS (6.5 mm); ring 12-14 mm at n = 5-7 |
  | Relay | 8 of 8 | 15.4-19.1 | 15.5-19.2 | 1.1-2.0 | J holds; N4 MISS (6.4 mm) |
  | Pairs | 8 of 8 | 15.4-18.2 | 15.5-18.3 | 1.2-1.8 | J holds; N4 MISS (6.4 mm) |
  | Release during a return (relblend) | 8 of 8 | 15.4-19.1 | 15.5-19.2 | 1.1-2.1 | J MISS (rod end 2.2-2.7 cm); N4 MISS (8.2 mm) |
  | 8 -> 2 -> 8, the T (chain2back) | 12 of 12 | 16.6-23.8 | 16.7-23.9 | 0.9-4.4 | J MISS (one bigger > 20 s, 3 of 6); the DETACH tips 27-28 deg |
  | Chain on the figure-8 laps | 8 of 8 | 9.7-21.6 | 9.8-21.7 | 1.1-1.9 | J MISS (one bigger 20.6-21.7 s); P MISS (19.4 mm against 17.9) |
  | Back-to-back (chain4b2b) | 8 of 8 | 17.3-23.6 | 17.4-23.7 | 2.7-3.3 | J MISS (23.7 s); overlap MISS (> 2.50 deg); N4 MISS (7.1-7.3 mm) |
  | Back-to-back on the orbit | 8 of 8 | 15.7-26.5 | 15.8-26.7 | 2.6-3.5 | J MISS (> 20 s, 3 of 4); overlap MISS; P holds (16.3 mm against 21.2) |
  | Two releases on the figure-8 (fig8rel2) | - | - | - | - | P MISS (25.3-25.5 mm against 20.4); E4 MISS (0.67 m) |
  | Adjacent pair, orbit, the bases | - | - | - | - | hold (the pair's peak 6.7-7.0 deg) |

  Against the final masses (F055/F057): every return now welds (the final masses had three give up at 45 s) and is on
  its plate in 10-27 s (10-40 s); four cases hold J where none did; chain2back's fold-ins drop from 7.5 to 4.4 deg; the
  orbit's P now holds (28 -> 16 mm). But N4 (the ring back at eight after the last return, against the 4-quadrotor
  hover comparator 2 x mean + 3 mm = 5.65 mm) misses by 0.8-1.6 mm in every case with returns, where the final masses held
  it on the claim chain (4.93-4.96 against 5.16). L holds in every case: every release detected, the planner one smaller
  in 0.25-0.60 s.
- Verdicts as the card is written: the claim chain (N) is FALSIFIED by N4's position bar (tilt holds); every harder case
  with returns misses at least one item (J's 20 s "one bigger", the rod end, the b2b overlap, N4, or P); L and the
  adjacent pair hold.
- Text should now say: at eight quadrotors the planner holds the ring through every release and brings every quadrotor
  back (each on its plate in 10-27 s); the ring then tracks 6.4-7.3 mm in hover at eight after the returns, about 1 mm
  worse than the comparator's bound, and some returns take the planner 20-27 s to count (the card's 20 s).
- Evidence: registry R3922-R3927, R3954-R3979 (R3928/R3929 void, R3930-R3953 not run); results/analysis/
  x3_oldlinks_2026-11-07.json (tools/x3_metrics.py); one-tree hash 5e7b1d3bd133. Reviewer done: see F066 (corrections).

### F066 2026-11-07 08:28 CORRECTION of F065: X3 on the earlier masses, after its reviewer
- The reviewer (7 Nov) rebuilt the numbers from the raw logs: every number in F065 stands; one code tree
  (5e7b1d3bd133) on all 32 runs; no void; every return counted (events.csv agrees with the planner log in all 26 runs
  with returns). Four statements change:
  1. N4 does NOT miss in every case with returns. chain2back holds it: 1.37 / 1.30 mm against 5.65 mm, tilt 0.07 deg
     (it flies without the sideways integral, int_k_xy 0). The five cases that miss (chain4back, the task, relay, pairs,
     b2b) miss by 0.7-1.6 mm because N4's window opens with the ring still 28-31 mm off after the last fold-in. The ring
     is back under 3 mm by +2.7-3.0 s. It is not a steady hover error. The claim chain stays FALSIFIED by N4 as the card
     is written: "N4 holds from +5 s" was ruled out as a bar judged after the run.
  2. The b2b orbit: the hover point walked 1.45 m (each fold-in moves the held reference onto the ring, ~0.25 m per
     weld). The ring reached |y| 1.45-1.49 m and airborne quadrotors |x| 1.90-1.95 m, |y| 2.45-2.49 m (the simulation has
     no position limit; the card asks for these extents). Its P (16.3 mm) is measured on that walked path, so "P now
     holds (28 -> 16 mm)" is not like for like: the final-mass runs ended at 5 carriers and walked 0.82 m.
  3. relblend: its own draft bars all hold (planner at 7 in 0.40-0.41 s with the rejoiner kept; tilt 2.1 deg in the 15 s
     after the release, peak 2.3 deg; settled 1.1 deg; every quadrotor landed). The rod end 2.2-2.7 cm is reported, not a
     bar for this case, and N4 does not apply (it ends at seven). So "every harder case with returns misses at least one
     item" is false for relblend.
  4. The task: two of its own draft bars miss, missing from F065. The fleet during the tasks is 12.4-13.8 mm at n = 5-7
     against 5.65 mm, and one task reference is 20.3 mm off against 2 cm (R3926).
- Text should now say: at eight quadrotors the planner holds the ring through every release and brings every quadrotor
  back, each on its plate in 10-27 s; some returns take the planner 20-27 s to count against the card's 20 s. After the
  last return the ring starts 28-31 mm off and is back within 3 mm in about 3 s. That opening puts five cases 0.7-1.6 mm
  over the comparator's bound; the case flown without the sideways integral holds it. The orbit's back-to-back case
  walks the ring 1.45 m.
- Evidence: registry R3922c; F065's runs.

### F067 2026-11-07 08:42 RESULT (simulation, claim): X3b's chains at one iteration per tick, on the earlier quadrotor masses (replaces F055/F057's chain numbers)
- Section: the results (scalability, eight and nine quadrotors at one planner iteration per tick).
- What changed: X3b's two chain cells re-flew on the quadrotor masses of before 4 Nov (Wesley 6 Nov: the claims with
  returns fly them), 2 runs each, one code tree with X3 (5e7b1d3bd133), no void. The 8-carrier chain is compared with
  its five-iteration pair on the same masses (X3's R3972/R3973). X3b's hover and figure-8 cells have no returns and
  flew only on the final masses (R3803-R3820, F055/F057). Each cell's one-vs-five comparison is on one model, but the
  cells of C8 now span two quadrotor models. Bars as written on the card (frozen 30 Oct).

  | Cell (x2) | Returns welded | On the plate (s) | Planner one bigger (s) | Fold-in peak (deg) | N4 (mm, bar 5.65) | Verdict |
  |---|---|---|---|---|---|---|
  | 8 carriers, 8 -> 4 -> 8, 1 iteration (R3984, R3985) | 8 of 8 | 16.3-18.3 | 16.4-18.4 | 1.08-1.76 | 6.15-6.20 | C8's chain cell CONFIRMED: the same verdicts as its 5-iteration pair (L holds, J holds, N4 position MISS in both arms, tilt holds) and every close-to-5-iterations item holds |
  | 9 carriers, 9 -> 5 -> 9, 1 iteration (R3980, R3981) | 8 of 8 | 16.3-19.3 | 16.4-19.4 | 1.05-1.95 | 5.98-6.04 | N9's chain: L and J hold in both repeats; N4 (reported) position MISS, tilt holds |

  - Releases: every one detected, the planner one smaller in 0.25-0.36 s, peaks 1.5-2.5 deg, settled <= 1.5 deg.
  - Returns: the rod end within 1.4-1.6 cm of its plate point (bar 2 cm).
  - N4: the window opens with the ring 26-28 mm off after the last fold-in, under 3 mm by +2.7-2.9 s, as in X3 (F066)
    and in the five-iteration pair (27.7-28.1 mm).
  - Validity: RTF 0.185-0.227 (flagged under 0.25, no release missed), late planner ticks 0.11-0.18 %, no failed
    solve, N6 holds (published residual at most 0.0036).
  - Timing in these simulation runs: whole planner tick p95 117-122 ms at eight and 152-158 ms at nine (solve p95
    60-62 / 80-83 ms), over the 100 ms period, as in F057.
- Against the final masses (F057): J now holds at nine as well as eight. The final masses had a return at 36.2 s, the
  rod end 2.5-2.8 cm and fold-ins to 7 deg; here they are at most 19.4 s, 1.6 cm and 1.95 deg. N6 now holds at nine
  (0.015 before). N4's position, which held on the final masses (4.4-4.7 against 5.16), now misses by 0.3-0.6 mm, as
  the five-iteration chain does.
- Verdicts as the card is written: C8's chain cell is confirmed on the earlier masses (its hover and figure-8 cells were
  confirmed per cell on the final masses, F057). N9's chain meets its L and J bars, but N9 as a whole is not confirmed:
  R9b cannot fly on this laptop (R3452, R3778 void), and R9a's idle bench on the frozen tree is due tonight.
- Text should now say: at one planner iteration per tick, eight and nine quadrotors fly the release-and-return chain
  with every release caught and every quadrotor back on its plate in 16-19 s. Eight quadrotors give the same
  verdicts as five iterations. The ring's error after the last return is about 6 mm against a 5.65 mm bound in both,
  from the first 3 s after the last fold-in. Do not write "real time" for the simulation: the whole tick is 117-158 ms.
- Evidence: registry R3980, R3981, R3984, R3985 (R3982/R3983 not run); comparators R3972, R3973, R3978, R3979;
  results/analysis/x3b_oldlinks_2026-11-07.json (tools/x3b_metrics.py). Reviewer done: see F068 (corrections).

### F068 2026-11-07 08:51 CORRECTION of F067: X3b's chains on the earlier masses, after its reviewer
- The reviewer (7 Nov) rebuilt the numbers from the raw logs. All but item 1 reproduce within 10 %. One tree; no void (the
  card only flags RTF under 0.25; every one of the 16 releases resized in 0.25-0.36 s). Corrections:
  1. Solve p95 at full size is 70.1-71.7 ms at eight and 89.9-94.2 ms at nine. F067's 60-62 / 80-83 ms covered all fleet
     sizes. Also "settled <= 1.5 deg" is 1.52 deg, and "4.4-4.7" is 4.38-4.75.
  2. "Real time": the conclusion stands (not shown), but not for F067's reason. The 117-158 ms tick includes the residual
     computed every tick for N6 (31-41 ms), which the real-time setting turns off: 85-88 ms at eight and 109-111 ms at nine
     without it. These runs ticked on simulated time at RTF 0.19-0.23, so they test nothing about real time. The only
     closed-loop timing run (R8b, 31 Oct) was FALSIFIED and R9b is void; R9a's idle bench is due tonight.
  3. "J now holds at nine as well as eight" is wrong. J missed at BOTH sizes on the final masses (first return on its
     plate at 37.2 s against 30 s, R3805-R3808) and holds at both now.
  4. On the final masses this chain cell FAILED C8 (R3807's /drone_5 fold-in 3.97 deg against 2.95; F057). That stays
     on record: the chain cell fails on the final masses and holds on the earlier ones.
  5. C8 as a whole is NOT confirmed. Its hover and figure-8 cells flew only on the final masses and its chain on the
     earlier ones, so only per-cell verdicts are fair. N9 is split the same way (hover on the final masses). The text
     cannot carry a two-model caveat (Wesley 6 Nov), so the lab chat queued x3b_s8_hover_rt, x3b_s8_fig8_rt and
     x3b_s9_hover_rt on the earlier masses (6 runs, after X1, ~Sat 20:00; the it5 arms already flew on them, R3974-R3977).
     C8 and N9 are then judged on one model.
- Text should now say: nothing about C8 or N9 yet. Per cell: at nine the chain holds L and J, each return on its plate in
  16-19 s; N4's position misses by 0.3-0.6 mm, for the reason given in F066. Real time is not shown in simulation.
- Evidence: registry R3980c; the runs of F067.

### F069 2026-11-07 09:14 RESULT (simulation, claim): X2 the full chain 4 -> 3 -> 2 -> 3 -> 4 on the figure-8, FINAL (the earlier quadrotor masses; replaces F042/F045's X2 numbers)
- Section: 4.x X2, the supervisor's chain in one run (card 2026-10-30_x2_full_chain_fig8).
- What changed: X2 re-flew on the final tree with the quadrotor masses of before 4 Nov (Wesley 6 Nov: the claims with
  returns fly them): 6 runs (A announced, B unannounced, the base; x2 each), one code tree (5e7b1d3bd133), no void
  (late ticks at most 0.05 %, no failed solve, read-backs as the card, every event fired, both returns counted in
  events.csv and the planner log). These are the numbers the thesis uses (R3986-R3991). R3586-R3591 (3 Nov tree, the
  same masses) stay in the registry as a recorded comparison. Bars as frozen 30 Oct; B1's later-runs bar (largest tilt
  from +6 s) frozen 31 Oct, its 5 deg value set 4 Nov ~22:20 after the 3 Nov runs.

  | Item | Bar | A R3986 | A R3987 | B R3988 | B R3989 |
  |---|---|---|---|---|---|
  | Step 1 square -> T: A rise from the 5 s mean / B abs peak (deg) | 30 / 80 | **30.45** (abs 31.03) | 29.39 (abs 29.94) | 58.61 | 58.02 |
  | A planner at n = 3 / B largest tilt from +6 s (s / deg) | 6.0 s / 5 deg | 3.10 s | 3.10 s | 2.72 | 2.97 |
  | Step 2 T -> pair: rise, pair lean (deg) | < 15, < 5 | 5.38, 0.78 | 5.62, 0.70 | 5.18, 0.72 | 5.49, 0.85 |
  | Step 3 pair -> T: on plate (s), fold-in tilt (deg) | 30, 5 | 18.5, 2.24 | 17.5, 2.08 | 17.6, 2.23 | 19.5, 2.13 |
  | Step 4 T -> square: on plate (s), fold-in tilt (deg) | 30, 5 | 22.9, **5.36** | 22.6, 4.13 | 20.7, 2.53 | 20.7, 3.00 |
  | Lap 6 path (mm), tilt peak (deg) | 13.11, 2.15 | 9.66, 1.21 | 9.69, 1.30 | 9.35, 1.16 | 10.23, 1.21 |
  | Whole run: drop, abort, landed, staging | none, all, E4 | holds | holds | holds | holds |

  Reported, not barred: the original T6 for B (37.64 / 35.49 s against 5.0 s) stays recorded as missed; the rod end
  through the blends 1.3-1.8 cm (bar 2); the planned tension at each blend's end within 1 % of the share; lap 6 flies a
  datum moved 53-61 mm (ring to lap 1's reference 36.4-42.7 mm RMS against the base's 10.9 / 11.8 mm); in the second T
  drone 2 sits on the 0.8 throttle cap for 5.6-12.3 s before the last weld (the base never reaches it, p95 0.65-0.66) and
  the ring is 7.5-18.8 mm low over its last 5 s.
- Verdicts as the card is written:
  - A (announced): FALSIFIED in R3986 by two items. Its square-to-T rise is 30.45 deg against 30 (over 30 deg for 0.21 s),
    and its last fold-in is 5.36 deg against 5. R3987 holds every bar. The card predicted the 30 deg bar "may fall either
    way" (its hover peaks sat within ~0.5 deg of it). R3986's fold-in miss comes with the longest stretch on the throttle
    cap (12.3 s): the first pass (31 Oct, R3296) missed the same item at 5.19 deg the same way.
  - B (unannounced, the drop at 80 deg): CONFIRMED in both repeats under the 31 Oct bar. No loss, abs peak 58.0-58.6 deg
    (over the rig's 30 and 45 deg rules, as predicted), back under 5 deg by +6 s.
  - Against the 3 Nov runs (same masses, earlier tree, F042/F045): A held there (29.1 / 27.8 deg; fold-ins 3.00 / 3.30).
    Here A1 is 1.3-1.6 deg higher and one last fold-in is over. B is unchanged in substance (58.2 / 59.2 deg then).
- Text should now say: the whole chain flies on the figure-8 in one run, every step in both repeats of both variants,
  with no drop or loss. The announced square-to-T step peaks at 29.4-30.5 deg, at the 30 deg bound, and misses it once.
  The last fold-in, with one T carrier on its throttle limit, reaches 4.1-5.4 deg against 5 and misses once. Unannounced,
  the same step rolls the ring to about 58 deg and settles under 5 deg within 6 s.
- Evidence: registry R3986-R3991; results/analysis/x2_oldlinks_2026-11-07.json (tools/x2_metrics.py). Reviewer done: see F070 (corrections).

### F070 2026-11-07 09:21 CORRECTION of F069: X2 on the earlier masses, after its reviewer
- The reviewer (7 Nov) recomputed every barred number from run.csv / events.csv: all within 0.01 of F069. A FALSIFIED
  and B CONFIRMED follow the card's rules (a miss in one valid repeat falsifies). Corrections to the wording:
  1. A1's 29.4-30.5 deg are rises from the 5 s mean (the bar's quantity); the absolute peaks are 29.9-31.0 deg. The rise
     is over 30 deg for 0.15 s (0.21 s is the absolute tilt).
  2. The last fold-in spans 2.5-5.4 deg over all four chain runs (A 4.1-5.4, B 2.5-3.0).
  3. The throttle cap: drone 2 is continuously on the 0.8 cap for 10.5 / 9.2 / 5.6 / 6.4 s (F069's 5.6-12.3 s counted
     gaps of up to 1 s). The cap is 0.8 in the twin and on the rig, but what it leaves depends on the twin's modelled
     pack (22.75 V -> 19.0 V). The link to the fold-in is a reading of four runs, not a shown cause.
  4. A1 1.3-1.6 deg higher than on 3 Nov is about the spread between repeats (1.1-1.3 deg).
  5. Missing from F069: lap 6 flies a datum moved 53-61 mm (the ring 36-43 mm RMS from lap 1's reference against the
     base's 11-12). And "B settles under 5 deg within 6 s" should read "B stays under 3 deg from 6 s on": the card's
     own settle measure (T6, 35.5-37.6 s) stays recorded as missed.
- The card now records this re-fly (section "The claim re-fly on the earlier quadrotor masses"), with the B bar's
  provenance: its 5 deg was set 4 Nov ~22:20, after the runs. One tree: diff_sha256 f9ef4ab959d3 on all six.
- Text should now say: in the announced case the ring's rise to the T stays near 30 deg (29.4 and 30.5 deg in two
  repeats, the second just over the card's bar) and the last fold-in reaches 4.1-5.4 deg; in the unannounced case the
  ring stays under 3 deg from 6 s after the loss; every return welds, on the plate in 18-23 s.
- Evidence: registry R3986c; the runs of F069.

### F071 2026-11-07 09:30 RESULT (simulation, claim): 4.3 a departed carrier rejoins (tab:res_rejoin), FINAL (the earlier quadrotor masses)
- Section: 4.3 the rejoin table (tab:res_rejoin). Card docs/experiments/2026-10-23_rejoin_departed.md.
- What changed: the four rejoin cases re-flew x2 on the quadrotor masses of before 4 Nov (Wesley 6 Nov: the claims with
  returns fly them) on the frozen tree: 8 runs, one code tree (5e7b1d3bd133; diff_sha256 e4817e2048c0), no void (late
  ticks <= 0.08 %, no failed solve, RTF 0.70-0.91), one REJOIN WELD line and one REJOIN_WELD row per run. These are the
  numbers the thesis uses (R3992-R3999). Every bar was frozen before its case first flew (23 Oct: the square E1 00:42, the
  T E2 05:13, the leaning T E5 07:57); none is a draft bar.

  | Case (x2) | On the plate (s, bar 30) | Joint (s, 0.5) | Fold-in tilt peak (deg, 5) | End off (cm, 2) | Tension vs share (10 %) | Lean before LAND (deg, 2) | Verdict |
  |---|---|---|---|---|---|---|---|
  | Square (E1) | 21.9-23.8 | 0.03-0.04 | 1.48-1.56 | 1.4-1.5 | +2.8 to +3.0 % | 0.08-0.10 | all hold |
  | T (E2) | 17.5 | 0.015 | 1.49-1.60 | 1.3-1.4 | -2 to -3 % | 0.73 | all hold |
  | T leaning toward the plate (E5) | 16.5-16.6 | 0.02 | 3.02-3.51 | 1.2-1.5 | -1 % | 0.48-0.49 | all hold |
  | T leaning away (E5 mirror) | 17.7-18.6 | 0.01-0.02 | 3.10-3.33 | 1.5 | -1 to -2 % | 1.92 | all hold |

  The fold-in tilt is the ring's absolute tilt (its normal from vertical), peak from the weld to +15 s; it never went over
  5 deg. In the leaning cases it includes the pair's own lean, held by an applied torque: +3.21/+3.25 deg toward plate 3
  and -2.61/-3.05 deg away in the 0.5 s before the weld. Both are over the 2 deg under which E5 does not test the lean.
  The lean before LAND in the mirror (1.92 deg against 2) is the closest margin, as on 23 Oct, with the -0.011 N m
  torque still on until LAND. Reported, not barred: the weld-height cap bound 101 ticks on the T (59-61 in the square),
  the plan 2.0-2.7 cm above it at the last.
- Against the 23 Oct runs (the same masses, an earlier tree): the same verdicts, except E5's R2712, which missed the
  5 deg fold-in bar (5.28 deg for 0.19 s) and stays recorded as missed; now 3.02-3.51 deg in both repeats. On the final
  masses this table never flew; the one exploratory run there (R3907, the T) took 37.0 s to reach its plate against the
  30 s bar (the slow return of the 4 Nov model change, F058). That stays on record.
- Verdicts as the card is written: every frozen bar holds in all eight runs (E1, E2 and E5 both ways).
- Text should now say: a departed carrier rejoins its own plate in every case, the square and the T, with the pair
  level or leaning about 3 deg either way. It reaches its plate in 16.5-23.8 s, the joint closes within 0.04 s, and the
  ring's tilt stays at or under 3.5 deg through the fold-in (1.5-1.6 deg when the pair is level). The rejoiner carries
  its share within 3 %, and the ring settles within 2 deg of level before landing.
- Evidence: registry R3992-R3999; results/analysis/rejoin_oldlinks_2026-11-07.json (tools/rejoin_metrics.py). Reviewer done: see F072 (corrections).

### F072 2026-11-07 09:38 CORRECTION of F071: the rejoin table on the earlier masses, after its reviewer
- The reviewer (7 Nov) rebuilt every number from run.csv / events.csv / the planner log: all as F071 says; one tree
  (5e7b1d3bd133; diff_sha256 e4817e2048c0); no void; the fold-in bar is on the absolute tilt, as F071 uses it. Corrections:
  1. Who set the bars: the lab chat set E1, E2 and E5 overnight on 23 Oct, each minutes before its case first flew
     (E1 00:42 / R2457 01:03, E2 05:13 / R2584 05:15, E5 07:57 / R2602 08:13). Wesley approved flying the leaning pair,
     not the bar values. The card's header still reads "bars PROPOSED: Wesley freezes them". So: bars set before the runs
     by the lab chat, not frozen by Wesley.
  2. The mirror case: "settles within 2 deg of level before landing" overstates it. The bar is a 10 s mean (1.92 deg);
     inside that window the ring keeps swinging up to 3.14 / 3.47 deg (more than its fold-in peak) with its torque on until
     LAND. And the leaning cases' 3.51 deg is "at or under 3.6", not 3.5.
  3. "Carries its share within 3 %" is the planned tension, not a measured one (run.csv's tension columns are empty). On
     the T the share is the planner's 0.1 N floor logged to 1 mN, so the -3 % could be -2.5 to -3.5 %. Only the square
     tests a real share: 2.508 / 2.503 N against 2.435 N.
  4. The leaning cases' tilt is not only the pre-weld lean: when the blend ends (+11.2-11.5 s) the ring swings through
     level, a 4.3-4.6 deg change toward the plate (R3996: -3.3 to +1.3 deg); the mirror's peaks (3.1-3.3 deg at +11.2 s)
     are this swing.
  5. "Same verdicts as 23 Oct" holds only for the runs after the 23 Oct fixes. The earlier misses stay on the card:
     R2457 / R2458 5.83 / 5.32 deg, R2466 / R2467 5.82 / 5.75, R2584 10.61, R2712 5.28.
- Also on record (the text follows Wesley 6 Nov: the mass split is not explained): this table flew the pre-4-Nov split;
  on the hardware-matched masses the one exploratory run (R3907) took 37.0 s to its plate, over the 30 s bar.
- Text should now say: a departed carrier rejoins in all four cases (square, T, the pair leaning either way): on its
  plate in 16.5-23.8 s (bar 30 s), the joint closed within 0.04 s, the ring's tilt at most 3.5 deg in the 15 s after the
  weld and its lean before landing 1.9 deg or less on average; the leaning cases swing the ring through level when the
  blend ends.
- Evidence: registry R3992c; the runs of F071.

### F073 2026-11-07 10:00 RESULT (simulation, claim): X8 the paper's method through a loss, FINAL (the final, hardware-matched quadrotor masses; replaces F042's X8 numbers)
- What: 4 quadrotors hover the 0.86 kg ring on 1 m rods; /drone_1 is released unannounced 25 s after the lift; LAND
  30 s later. Ours (our planner, the detach mode) against the network as we ported it, told 0.35 s after the release
  (the baseline), on two cells: world C (every cable on a hub at the ring's centre, the paper's geometry) and our rim.
  Neighbours without a bar: the port never told, and the exact law never told (a smoke on world C). 14 runs, 7 configs
  x2, one code tree (code hash 5e7b1d3bd133, diff_sha256 f6426db0b67a, bf52a8a), no void (late ticks <= 0.11 %, no
  failed solve, every RELEASE let go, ours resized and the port's DETACH line at +0.35 s as the card requires). X8 has
  no returns, so it flies the final masses (Wesley 6 Nov). Bars L1-L3 and the loss rules FROZEN 30 Oct 16:26 by Wesley
  before any run; no draft bars. The rim's ours arm flies int_k_xy 0 (the 31 Oct fallback for every T cell, Wesley
  31 Oct 17:16); the hub's ours arm keeps the integral as flown (taken on the lab chat's recommendation 5 Nov; Wesley
  may override).

  | Cell | Arm (x2) | E, largest excursion in 20 s (mm) | Dip (mm) | Settle t_s (s) | Loss by the card's rule | Survivors' gap at the end |
  |---|---|---|---|---|---|---|
  | World C (hub) | ours | 286.2 / 290.4 | 65.5 / 66.0 | 5.33 / 5.37 | none | 171 deg (the T) |
  | World C (hub) | port, told | 373.3 / 371.9 | 144.1 / 142.5 | 7.72 / 7.69 | "over 300 mm for 1.00 / 1.02 s" (on the 1 s rule: not claimed) | 120 deg (re-spaced) |
  | World C (hub) | port, never told | 616.2 / 615.5 | 430 | censored | over 300 mm for 29 s | 144 deg |
  | World C (hub) | exact law, never told | - | - | - | fleet abort +1.22 / +1.21 s (the leaver at 3.40 / 3.30 m/s > 3) | - |
  | Rim | ours | 220.6 / 218.7 | 94.7 / 93.2 | 4.42 / 4.39 | none | 177 deg (the T) |
  | Rim | port, told | 455.1 / 459.9 | 153.5 / 159.7 | 27.77 / censored | the offline 80 deg rule only (0.17-0.19 s continuous); the ring stayed up at 47-48 deg | 137 deg |
  | Rim | port, never told | 679.6 / 678.4 | 406.5 | censored | over 80 deg (0.30 s continuous) | 154 deg |

- Verdicts (bars as written): **L1 (world C) CONFIRMED** on E alone: both port E (373.3, 371.9) > ours_max + 20 =
  310.4 mm. **L2 (rim) CONFIRMED**: both port E (455.1, 459.9) > 240.6 mm. **L3 HOLDS**: no ours run loses the ring;
  on world C t_s 5.33 / 5.37 s <= 10. The exact law's smoke: the card's code reading stands (the released quad passes
  3 m/s and the trackers' speed fault disarms the fleet; the ring is down by +1.55 s).
- Reported: ours resizes 0.39-0.42 s after the release (UNANNOUNCED DETACH), the port's DETACH at +0.35 s. On world C
  ours' ring swings to 286-290 mm at +1.25 s and its free tilt (the hub leaves it free) peaks at 95-97 deg, still 45-46
  deg in the last 5 s before LAND (the port's 107). On the rim ours' T ends level: 1.8 deg in the last 5 s before LAND
  (both repeats); the port's ring hangs at 47-48 deg. The leaver flies 1.5-1.6 m out (ours), 2.0-2.1 m (port), 3.1-3.2 m
  (never told).
- Against F042 (3-4 Nov, R3592-R3593, R3636-R3647): not like for like (an earlier tree, the earlier quadrotor masses,
  and the rim's ours arm then kept the integral): the same verdicts; E then ours 315 / 312 (hub), 244 / 236 (rim), port
  365 / 370 (hub), 448 / 446 (rim). The rim's ours T then leaned to 16 deg by LAND in one repeat (R3642); now it ends at
  1.8 deg in both, a reading, not a shown cause (three things changed).
- Text should say: through an unannounced loss in hover our planner keeps the ring closer to where it was than the
  network told at the same moment: on the paper's hub geometry by 84 mm on average (largest excursion 286-290 against 372-373
  mm), on our rim by 238 mm on average (219-221 against 455-460 mm), where the network's re-spacing tips the ring past 80 deg
  for under 0.2 s and leaves it hanging at 47-48 deg; our ring settles within 5.4 s on the hub. The exact law, never told,
  loses the fleet within 1.3 s. (No hub loss is claimed for the port: its 300 mm rule sits on the boundary.)
- Evidence: registry R4000-R4013; results/analysis/x8_2026-11-07.json (tools/x8_metrics.py). Reviewer done: see F074 (corrections).

### F074 2026-11-07 10:07 CORRECTION of F073: X8 on the final masses, after its reviewer
- The reviewer (7 Nov) rebuilt the numbers from the raw logs (within 0.02 s and 0.1 mm); one tree (5e7b1d3bd133;
  diff_sha256 f6426db0b67a); no void; bars frozen 30 Oct 16:26 before the first X8 run (R3439, 31 Oct). L1, L2, L3 hold
  as written. Corrections:
  1. The hub's 300 mm rule: each port run is over 300 mm in one unbroken stretch of 0.98-1.02 s (1.01 s by interpolated
     crossings). F073 said "not claimed" while the scorer and the json count both as losses (>= 1.0 s). Read: "on the 1 s
     boundary within one sample". L1 holds either way.
  2. Landings: R4005 (the port told, a claim run) never landed: for 80 s, to the 160 s limit, the ring sat at
     z 0.18-0.20 m tilted 41-46 deg with the survivors hovering at 1.1-1.3 m; R4000 / R4001 likewise (0.20-0.22 m,
     47-54 deg); R4001 and R4005 end with pose timeouts and a FLEET ABORT at shutdown. R4004 landed at +7.1 s. After LAND,
     so no verdict changes, but it is part of the result.
  3. "The network's re-spacing tips the ring past 80 deg" is not shown: the port never told passes 80 deg at the same
     moment (+0.58-0.60 s). Read: "the network, told or not, tips the ring past 80 deg about 0.6 s after the release"
     (over 80 deg for 0.15-0.19 s).
  4. "Hanging at 47-48 deg" is a 5 s mean (the ring swings 23-75 deg in it). "Our ring settles within 5.4 s on the hub"
     is the ring's centre only: the free ring stays over 80 deg from +11.6 s for 11.0-11.3 s, peaks at 96-97 deg at
     +17.3 s and swings 26-66 deg in the last 5 s (R4008 / R4009). The text must give the tilt.
  5. "Keeps the ring closer to where it was" holds for the largest excursion only: on the rim ours ends 47 mm from its
     pre-release mean, the port 3 mm (last 5 s before LAND).
  6. Not "told at the same moment": the port is told at +0.35 s, ours resizes at +0.39-0.42 s (the port knows
     0.04-0.07 s earlier, in its favour).
  7. The integral: the rim's ours arm flies int_k_xy 0 under Wesley's 5 Nov ~14:00 rule for every T cell (the 31 Oct
     17:16 fallback); the hub's ours arm keeps int_k_xy 5.26 ("X8's hub as flown", on the lab chat's recommendation),
     although its survivors leave a 171 deg gap. The two ours arms fly different integral settings: Wesley's call
     (asked 7 Nov).
  8. R4013's leaver peaks at 3.16 m/s in run.csv (3.30 m/s is the tracker's fault-line reading).
- Text should now say: on the rim and world C the port of the paper's method moves the ring 372-460 mm at its worst
  against ours' 219-290 mm; told or not, the network tips the ring past 80 deg about 0.6 s after the release and some
  of its runs never land the ring; our planner keeps the ring up in every run, its centre settled within 5.4 s on the
  hub, but the free ring there swings past 80 deg for about 11 s later in the run.
- Evidence: registry R4000c; the runs of F073.

### F075 2026-11-07 10:10 DECISION (Wesley): X3 accepted after the run; X3b's six runs and an X8 hub arm queued
- X3 on the earlier masses (F065 / F066): Wesley judged the results fine and the bars arbitrary (his rule of 2 / 5 Nov: no
  bars unless compared with another paper). The misses stay recorded in the registry and on the card; the verdict is
  ACCEPTED, judged after the run. The text gives the numbers (F066's "text should say"), not bars or falsifications.
- Queued after X1 (~19:45-20:00): X3b's real-time hover and figure-8 cells on the earlier masses (6 runs; F068 item 5),
  and X8's hub with our arm's sideways integral off (2 runs; F074 item 7). Their results follow in the feed.
- Evidence: decisions.md 7 Nov.

### F076 2026-11-07 10:32 RESULT (simulation, claim): 4.1 the fixed team and 4.2 the 4 -> 3 release, FINAL (the final quadrotor masses; replaces F031's simulation numbers)
- Section: 4.1 Simulation (the fixed team) and 4.2 (the release flown in both, tab:res_release's simulation row).
- What changed: chunk 7 of the claim batch flew Q007's cells on the final tree with the final (hardware-matched) masses:
  20 runs, R4014-R4033, two per cell. One tree (code hash 5e7b1d3bd133; diff_sha256 2863a8be2293; bf52a8a), no void
  (planner late ticks 0.000 %, tracker at most 0.098 %, no failed solve, RTF 0.71-0.96, no drop or abort; the SLOW TICK
  lines in R4021 / R4025 come 26-33 s after LAND, after the run's end). Scored as before (tools/s41_metrics.py: the real
  flights' scorer, best_data.score and tools/yaw_track.py; the release with tools/x1_metrics and F033's band). No card
  gives 4.1's cells bars (Q007, 3 Nov: judged after the run); numbers only.

  | Carriers | Hover error (mm) | Figure-eight path error (mm), lag (s) | Spinny line path error (mm), yaw error RMS (deg), yaw delay (s) | Tilt peak, hover / figure-eight / spinny line (deg) |
  |---|---|---|---|---|
  | 2 | 1.1 / 1.0 | 9.1 / 8.2, 0.01 / 0.01 | 6.5 / 6.7, 5.7 / 6.2, 0.19 / 0.21 | 0.16 / 0.16, 2.37 / 2.24, 1.02 / 0.87 |
  | 3 | 1.1 / 1.1 | 9.6 / 10.2, 0.05 / 0.06 | 5.5 / 4.1, 4.1 / 4.4, 0.14 / 0.15 | 0.25 / 0.24, 1.34 / 1.39, 1.12 / 0.98 |
  | 4 | 1.1 / 1.1 | 11.8 / 12.7, 0.09 / 0.10 | 3.7 / 3.8, 4.8 / 4.6, 0.16 / 0.16 | 0.16 / 0.20, 1.34 / 1.29, 1.02 / 1.11 |

  Tilt mean: hover 0.04-0.06 deg, figure-eight 0.77-0.89, spinny line 0.42-0.51. On the figure-eight the error is across
  the path (height RMS 1.6-1.9 mm); on the spinny line height RMS 2.4-6.0 mm. Tilt peak is the absolute tilt; the
  path errors are clock-free, the lag reported apart.

  **The 4 -> 3 unannounced release in hover, 120 deg gap (x11_r1m_ta_a1_r1 / _r2 = R4033 / R4032):** detection 0.36 /
  0.37 s; tilt rise from the 5 s mean 6.15 / 6.45 deg; absolute tilt peak 5.48 / 5.59 deg; back in band, the thesis
  table's definition (F033) 2.63 / 2.63 s (release_bands' T6, a 1 s hold, 3.59 / 3.70 s); dip 30.1 / 30.3 mm; settled
  0.01 / 0.03 deg; no loss; the freed drone disarmed alone before LANDED in both. The 1 m re-fly card's smoke falsifier
  ("no detection of the release, or the freed drone not disarmed alone"; set by the lab chat, frozen 23 Oct 02:15)
  holds.
- Against F031 (R3489-R3508, 3 Nov): NOT like for like. Those flew the earlier tree, the earlier quadrotor masses (every
  twin run before 4 Nov ~23:15 flew 0.040 + 0.035 on 0.475, CLAUDE.md), and the release runs flew ema (their manifest:
  SIM_MOCAP_VEL_FILTER ema; their registry notes said slope); today's fly the slope estimate. The numbers move little
  (hover 1.0-1.1 against 1.0-1.4 mm; figure-eight 8.2-12.7 against 8.0-12.9 mm; spinny line 3.7-6.7 against 4.1-8.7 mm;
  the release's tilt rise 6.15-6.45 against 6.59-7.18 deg, back in band on F033's definition 2.63 against 3.34 / 3.23 s),
  but which change moved them is not shown.
- Text should now say: 4.1's fixed-team table uses these numbers (one line per carrier count, two runs each). The
  release flown in both, in simulation: detected 0.36-0.37 s after it, the tilt rising 6.2-6.5 deg from its mean (5.5-5.6
  deg absolute), back in band 2.6 s after it, the ring dipping 30 mm and settling level. The caveats of F031 / F032 for
  the comparison with the 8 Oct real releases stand (0.55 m links and an earlier velocity estimate on the rig).
- Evidence: registry R4014-R4033; results/analysis/s41_s42_2026-11-07.json (tools/s41_metrics.py). Reviewer done: see F077 (corrections).

### F077 2026-11-07 10:42 CORRECTION of F076: 4.1 / 4.2 on the final masses, after its reviewer
- The reviewer (7 Nov) recomputed from run.csv: one tree (5e7b1d3bd133; diff_sha256 2863a8be2293), no void; hover,
  figure-8 path error (clock-free) and lag, 4.2's detection, dip and back-in-band all match. Corrections:
  1. The spinny line's yaw columns are wrong. tools/yaw_track.py aligns the planner log by detecting lift-off, which
     lands 0.012-0.079 s off the exact clock. Re-run on the exact clock, every cell is 3.7-4.0 deg yaw error RMS with a
     0.12-0.13 s delay (b2: 3.98 / 4.02 deg, 0.13 s, against F076's 5.7 / 6.2 deg, 0.19 / 0.21 s; b3: 3.73 / 3.73 against
     4.1 / 4.4; b4: 3.93 / 3.86 against 4.8 / 4.6). The 2-carrier row's larger error came from the alignment, not the
     flight. Earlier yaw numbers from the same tool (3 Nov, F031) may carry the same offset; the tool gets fixed after the
     batch (tools/ is in the one-tree hash) and those numbers re-read then.
  2. 4.2's "tilt rising 6.2-6.5 deg" overstates: that is the distance between tilt vectors from the 5 s mean (the ring
     hung 0.7-0.9 deg toward -y, then swung to 5.5-5.6 deg toward +y). Read: "the tilt moved 6.2-6.5 deg from its
     pre-release attitude (peak 5.5-5.6 deg)". Give the band too: the highest tilt before the release + 0.5 deg,
     1.79-1.81 deg. The detection time (0.36-0.37 s) is good to the planner log's 0.1 s.
  3. In R4033 the freed quadrotor tipped onto its side right after it was disarmed alone (tilt 10.7 deg at the disarm,
     90 deg by +0.4 s, body 0.10 -> 0.03 m); in R4032 it stayed at 1 deg. The card's section-5 bar for that event
     (ledger_w4, stays within 0.05 m after) would fail R4033. The smoke falsifier "disarmed alone before LANDED" holds in
     both (set by the lab chat in the card's smoke list, not one of section 5's frozen bars).
  4. The figure-8 lag of the 2-carrier cell (0.01 s) is within one 0.02 s log step: about zero.
- Text should now say: the fixed team holds a hover to about 1 mm, follows the figure-8 within 8-13 mm (lag under 0.1 s)
  and the spinning line within 4-7 mm, its yaw within 3.7-4.0 deg RMS at a 0.12-0.13 s delay; after the 4 -> 3 release
  the ring is back in its band in 2.6 s, its attitude moved 6.2-6.5 deg (peak 5.5-5.6), dipping 30 mm; the freed
  quadrotor lands alone (in one repeat it tipped over after disarming).
- Evidence: registry R4014c; the runs of F076.

### F078 2026-11-07 13:38 RESULT (simulation, claim): U, the magnet caught off its slot, on the final masses (replaces the 23 Oct grid's numbers)
- What: the U grid re-flown on the final tree and the hardware-matched masses (chunk 8a): two arms (the fold-in at the
  measured catch point, as the rig does it; or the slot assumed), the catch offset along the rim (clockwise, toward the
  plate-11 carrier) or radially in, 0-16 / 0-20 cm, 2 repeats each, 60 runs (R4034-R4093). One tree (section-5 code
  hash 5e7b1d3bd133; diff_sha256 11643a387a4c); RTF 0.63-0.70; late planner ticks at most 0.2 %; no failed solve; no
  VOID (no late weld); every run welded; plumbing (catch to aim) at most 2.5 cm, the trigger radius. The bars are §5's,
  frozen 23 Oct 07:56 on Wesley's answers (decisions.md 23 Oct (7)); the scorer (tools/catch_offset.py) works on the run
  clock from events.csv, no log alignment by detection.
- The fold-in (the claim's subject): the measured arm's fold-in tilt peak is 0.6-2.8 deg at every offset in both
  directions (1.5-2.2 deg along the rim at 11.6-16 cm; 0.6-1.6 deg radially in at 12-20 cm). The slot arm stays within
  3.9 deg to 4 cm radially in, then 3.1-19.9 deg from 6 cm (all over 5 deg but one 8 cm repeat at 3.1), with the
  clock-free path error 118-315 mm; along the rim it stays within 4.5 deg to 11.6 cm, 3.4-6.8 deg at 12-16 cm.

  | Arm | Along the rim: completes, both repeats, to | Radially in: to | Separation kept to (rim / in) |
  |---|---|---|---|
  | measured | 2 cm (the landing aborts below) | 20 cm | 12 / 20 cm (16 cm along the rim: 0.22 m, both) |
  | slot | 11.6 cm | 4 cm (the fold-in over 5 deg from 6 cm) | 16 / 20 cm |

- The landing: 10 runs end in a fleet abort during LAND (runner FAIL, the card's failure 3): x11_u_meas_t04, t06, t08
  (one repeat each: R4071, R4075, R4079), t116, t12, t16 (both: R4082, R4083, R4086, R4087, R4090, R4091) and
  x11_u_slot_t16 (one: R4093). In each the figure-8 had completed and LAND was given; 6.8-7.6 s later, with the ring
  already on the floor (z 0.02-0.03 m) and the carriers still 0.5-0.7 m up, the newcomer, its body 0.20-0.27 m up, tilted
  to 64-71 deg and the tracker's 60 deg envelope disarmed the fleet. A result of the runs, not a void (nothing crashed,
  no event missed). The newcomer tips in every landing of the grid, 26-71 deg: 26-58 deg radially in (both arms), 44-53 deg on
  the slot, 26-67 deg in the slot arm along the rim, and 55-71 deg in the measured arm along the rim, rising with the
  offset (55 at 2 cm, 57-68 at 4-8 cm, 64-71 at 11.6-16 cm). R4078 (meas t08, completed) reached 62.7 deg in run.csv without an abort. That the off-slot catch makes
  the newcomer tip is a reading of these runs, not a shown cause.
- Against the card's bars (frozen 23 Oct): falsified as written, by the landing aborts alone. d_task(measured, along the
  rim) is 2 cm (< 8: falsifier 1 fires through the aborts at 4, 6 and 8 cm) and d_tol(slot) 11.6 > d_tol(measured) 2
  along the rim (falsifier 3). Not fired: the measured fold-in over 5 deg at d <= 4 (max 2.69). Confirm items: radially
  in d_task(measured) 20 >= 16 met; d_tol(measured) >= d_tol(slot) radially (20 vs 4) met, along the rim not met; the
  mechanism shown in both directions (along the rim the fold-in growth +1.48 / +2.92 deg at 12 / 16 cm, against the
  d = 0 spread 1.18; radially in +17.3 / +14.8 at 16 / 20). Separation's prediction (reported): d_sep(measured, rim)
  12 cm, as §2 predicted.
- Against 23 Oct (R2629-R2732): not like for like (an older tree, the earlier masses, ema). On record from then: the
  measured arm's fold-in over the bar at 11.6 / 12 / 16 cm along the rim (7.57 / 7.62, 5.82 / 5.14, 7.96 / 8.87 deg),
  d_tol(measured, rim) 6-8 cm; R2636's no-weld and R2672's late weld in the slot arm. No landing abort was recorded then.
- Text should say: when the magnet catches off its slot and the planner folds the newcomer in at the measured catch
  point, the ring's tilt at the fold-in stays within 2.8 deg up to 16 cm along the rim and 20 cm radially in;
  assuming the slot instead tips it up to 20 deg once the catch is 6 cm or more inside the rim. The newcomer comes
  within 0.30 m of a carrier past 12 cm along the rim. In the landing the newcomer tips over (26-71 deg); past 4 cm
  along the rim this disarmed the fleet once the ring was on the floor.
- Evidence: registry R4034-R4093; results/analysis/x11_u_2026-11-07.json (tools/catch_offset.py, plus the lab chat's
  landing reading under "landing_reading"). Reviewer done: see F079 (corrections).

### F079 2026-11-07 13:48 CORRECTION of F078: catching off the plate (U) on the final masses, after its reviewer
- The reviewer (7 Nov) recomputed from the raw logs: every number matches (path error within 10 %); one tree
  (5e7b1d3bd133; diff_sha256 11643a387a4c); every run welded; the 10 FAILs are results (the card's failures 3 and 6), not
  voids; falsifiers 1 and 3 fire and 2 does not, as the frozen wording reads. Corrections:
  1. Catch-to-aim is at most 2.52 cm (R4081), 0.2 mm over the card's 2.5 cm check, which as written stops the grid at one
     run over. Judged after the run (F075): a 0.2 mm overshoot of an arbitrary check; reported, not acted on.
  2. The landing aborts rest on a touchdown spike against the 60 deg envelope: at 4, 6 and 8 cm along the rim only one
     repeat of two aborts (the others peaked at 56.6 / 58.9 / 62.7 deg; R4078 was over 60 deg for ~0.05 s without a trip).
     So d_tol(measured, rim) = 2 cm and both falsifiers that fire rest on that threshold.
  3. "Tips over (26-71 deg)" overstates: 26-71 deg is the touchdown peak; afterwards the newcomer rests at 26-56 deg in 46
     of the 50 completed runs and level in 4 (R4072, R4076, R4077, R4081). It tips with no offset too: 44-53 deg on the
     slot (R4034-R4037).
  4. The aborts start at 4 cm along the rim (R4071): one run in two at 4, 6 and 8 cm, both at 11.6-16 cm (measured arm),
     and one in two at 16 cm in the slot arm (R4093). The three carriers were disarmed 0.48-0.72 m up.
  5. Wording: the card's freeze (section 5) words offsets past ~2.5 cm as a tolerance of attach-point model error, not as
     catches; points 6-20 cm radially in are off the ring's rim (t116 is the only real catch). The 0.30 m crowding is the
     measured arm's only (the slot arm keeps >= 0.54 m).
  6. The rim ordering REVERSED since 23 Oct: then at 11.6-16 cm along the rim the measured arm folded in at 5.1-8.9 deg
     and the slot arm 3.5-7.1; now 1.5-2.2 and 2.75-6.79. The 23 Oct grid differs in tree, carrier tethers and filter, so
     which change moved it is not shown. **The thesis draft's sentence is stale**: results_discussion.tex line 201
     ("folding in at the plate's centre tilted the ring less", "up to 8 cm along the rim").
- Text should now say: with the measured aim the fold-in stays under 2.8 deg for attach-point errors up to 16 cm along the
  rim and 20 cm inward; aiming at the slot does worse from 6 cm inward (up to 20 deg). In the landings after the task the
  newcomer tips 26-71 deg at touchdown (also with no offset); in 10 of 60 runs that crossed the 60 deg safety envelope and
  disarmed the fleet with the carriers 0.5-0.7 m up.
- Evidence: registry R4034c; the runs of F078.

### F080 2026-11-07 14:16 RESULT (simulation, claim): 4.3 the newcomer's join (7 cells x2), FINAL (the final quadrotor masses)
- What: QUAD4 joins the flying 1.15 kg ring under Te-Jen's planner (his contact attach, our co-motion proof, our fold-in),
  then the fleet lands. 14 runs R4094-R4107, chunk 8b of docs/claim_batch_2026-11-07.md. One code tree (code hash
  5e7b1d3bd133; diff_sha256 4d10fad5b123; bf52a8a); no void (late planner ticks <= 0.14 %, trackers <= 0.06 %, no failed
  solve, RTF 0.34-0.41). Scored with tools/attach_clearance.py (the named scorer); the fold-in tilt and the landing are
  the lab chat's own reading from run.csv (results/analysis/x11_joins_2026-11-07.json).

  | Cell (x2) | his TAKEOFF -> weld (s) | MATCH -> descent (s) | tries, proof | rod min to contact (m) | ring tilt after the weld, peak (mean) (deg) | landing |
  |---|---|---|---|---|---|---|
  | Square floor join, low route (A0) | 31.8 / 31.9 | 1.03 / 1.04 | 1, proven | 0.286-0.287 | 2.2 (1.4) | FLEET ABORT both, 7.1-7.3 s after LAND |
  | Square floor join, V5 (V1(0.30) + zero twist + xy 0.08) | 31.9 / 31.7 | 1.07 / 1.03 | 1, proven | 0.287 | 2.2 (1.5) | FLEET ABORT both, 7.0-8.0 s after LAND |
  | Square pickup, drop, re-attach (over route) | 96.7 / 99.1 | 1.03 / 3.23 | 1, proven | 0.284-0.285 (object 0.238-0.243) | 2.4-2.5 (1.7) | landed |
  | Triangle sq6b (rig line) | 19.9 / 20.1 | 0.53 / 0.60 | 1, proven | 0.201-0.202 | 1.8 (1.0) | landed |
  | sq6b + proof fixes G | 20.0 / 20.0 | 0.57 / 0.53 | 1, proven | 0.200-0.201 | 1.8 (1.0) | landed |
  | sq6b + the rig's frozen tip marker | 20.0 / 21.4 | 0.53 / 0.57 | 1, proven | 0.197-0.198 | 1.7-1.8 (1.0) | landed |
  | sq6b + G + frozen marker | 33.7 / 29.1 | 0.57 / 0.57 | 1 / 3, proven | 0.196-0.198 | 1.8 (1.0) | landed |

  Tilt after the weld: the ring's absolute tilt from the weld to LAND (or +15 s). The triangle's 0.20 m rod minimum is
  plate 9's own geometry at the contact (plates 7 and 11 at 60 deg), the square's 0.29 m likewise (90 deg). QUAD4's body
  stays under 2.45 m (2.36 m on the square's low route) except the pickup cell, which flies over the carriers at 3.3 m
  (Wesley 5 Nov, its exception). With the frozen marker and G the proof needed a 15.4 s bottom hold (R4100) and 3 tries
  (R4101); without G it held on try 1.
- The four FLEET ABORTs (both floor-join cells, both repeats): each comes after a successful join, during the fleet's
  LAND, 1.9-2.8 s after the ring is on the floor (z 0.013 m, level). QUAD4's magnet stays welded on plate 9 (tip z
  0.04-0.05 m); under the LAND's unwind arc its body moves out on its 1 m rod from 0.69 to 1.01-1.06 m from the ring's
  centre while it descends; then within ~0.5 s our tracker's pitch command for QUAD4 climbs to its +1.00 limit (yaw
  command -0.71), QUAD4 drifts 3-5 cm sideways and tilts from ~9 deg to 74-92 deg; its 60 deg envelope disarms the
  fleet with the carriers 0.61-0.77 m up. In the other ten runs QUAD4 follows the same unwind arc out to ~1.1 m and its
  tilt falls to 0.3 deg. The rod pinned on the grounded plate is the same in all 14 runs, so it is the setting, not what
  separates the four; what does (same world, same our-side settings and heading as the pickup cell) is not shown. Not a
  void: the join completes in every run, and the abort lies outside the cards' window (his TAKEOFF to the weld).
  History, the same configs: R3765 (5 Nov, final masses) aborted the same way, R3766 landed; so 5 of 6 floor-join
  landings with int_k_xy 0 on the final masses aborted, against 0 of 3 pickup-cell and 0 of 10 triangle landings.
  The 9 Oct ruling ("a tip-over on a LAND within ~1 s of a weld is a known twin artefact") does not cover these: LAND
  comes 10 s after the weld. The 29 Oct reviewer already flagged that the scorer counts aborts only up to LAND.
- Against the cards' bars (frozen 28 Oct ~22:30 / revision 2 ~23:00 / revision 3 ~23:55 by the lab chat on Wesley's
  goals of 28 Oct ~22:10 and ~23:05; he set the goals, not the values):
  - A0 and V5 (attach_ambitious): every bar holds in the card's window (welds in both within 300 s; rod >= 0.238 m;
    proof passes; V5's transit ends in 8.0 s and its tip is 0.285-0.286 m above plate 9 at MATCH, inside 0.30 +- 0.06);
    the runs as a whole end in the fleet abort above.
  - The pickup (avoidance fix): holds (welds; his C++ re-attach transit completes in 2.1-2.3 s; rod 0.284-0.285 m).
    The card's original 0.25 m bar stays missed at the drop (object-cable 0.197-0.200 m; 0.154 on 28 Oct).
  - The sq6b cells: no card bars (exploratory sheet previews); R2897's falsifier holds in all eight (QUAD4 under 2.6 m,
    no abort, no ring drop).
  F075's rule: the bars are the lab chat's, judged after the run by Wesley.
- Like for like (the same configs on the final masses, 5 Nov, an earlier tree): R3765 / R3766 welded at 31.7 / 32.0 s,
  R3768 at 95.9 s, R3751 at 20.0 s: today's within 1 s (pickup 3 s). The 28-30 Oct runs (R2843-R2908) flew the earlier
  masses and tree with the integral on: not like for like (their records stay: A0 38.7 / 38.5 s, V5 31.6 s, sq6b 46.3 s
  event time).
- Text should say: QUAD4 joins on its first contact in every case, proven by our co-motion check, 20 s after its own
  take-off on the triangle and 32 s on the square (97-99 s with a pickup and a drop first); the ring tilts at most
  2.5 deg after the weld. On the square floor join with the sideways integral off, the fleet's landing tipped QUAD4 over
  in both repeats of both cells (a fleet abort after touchdown); the other cells landed.
- Evidence: registry R4094-R4107; results/analysis/x11_joins_2026-11-07.json. Reviewer done: see F081 (corrections).

### F081 2026-11-07 14:27 CORRECTION of F080: the newcomer's join on the final masses, after its reviewer
- The reviewer (7 Nov) rebuilt the join numbers from the raw logs: all hold (weld 31.7-31.9 s square floor join,
  96.7 / 99.1 s pickup, 19.9-20.1 s plain triangle after his TAKEOFF; ring tilt after the weld 1.7-2.5 deg; object-cable
  0.197 / 0.200 m); one tree (5e7b1d3bd133; diff_sha256 4d10fad5b123); repeats agree. Corrections:
  1. The four floor-join landing aborts are classified TB14 by the named scorer (tools/m1_metrics.py tb14(): 60 deg
     0.15-0.33 s after the last reading under 10 deg, 2.1-3.0 s after the tip is down), as R3765 on 5 Nov. decisions.md
     (11 Oct, 22 Oct) records TB14 as a twin effect: no twin fix; on the rig the tether goes slack. R3765c's "the known
     twin artefact" stands.
  2. The safety bar as the card writes it ("no abort or envelope fault", no time window; metric 4 "any abort") MISSES in
     all four floor-join runs (tracker_3 logs an ENVELOPE FAULT in each); "the card's window (TAKEOFF to the weld)" was
     set after the run, and the scorer's PASS comes only from excluding after LAND. Judged after the run (F075).
  3. The mechanics, corrected: QUAD4's yaw starts to diverge right after touchdown in all five aborts (2.6-18 deg by +2 s;
     at most 0.33 deg in the 13 clean landings), then its pitch command climbs over 1.0-1.6 s to the +-1.00 limit (+1 in
     R4096, -1 in R4097-R4099 and R3765), the tilt diverging from 0.4-2.1 deg about 1.1 s after touchdown; drift 1.7-7.5 cm
     at 30 deg (up to 11.8 cm at the abort); the abort 2.0-2.9 s after the ring is down; the fault lines read 75.0 / 93.4 /
     106.1 / 87.8 deg. R3766 (5 Nov) also tipped past 30 deg (32.8): the floor join tips past 30 deg in 6 of 6 int_k_xy-0
     landings with QUAD4 welded, past 60 deg in 5. The pickup on the same square, also int_k_xy 0, lands cleanly in 3 of 3
     (the one state difference: its object lies in the ring's net). What separates them is not shown.
  4. "0 of 10 triangle landings" matches no set of runs (9 on the final masses, or 11 with R3752 / R3753), and the
     triangle flies int_k_xy 5.26, so it is not an int_k_xy-0 comparison.
  5. Not "first contact in every case": R4101 took 3 contacts; R4100 waited 12.8 s at the plate. "20 s on the triangle"
     holds for the plain, G-only and frozen-marker cells (19.9-21.4 s); G + frozen marker took 29.1 / 33.7 s.
  6. Provenance: the bar values are the lab chat's (28 Oct), the V1 centre heights Wesley's own (15 cm ~22:10 and 22:40,
     0.30 m ~23:05). The card's metric 4 is the ring tilt from his TAKEOFF to the weld (12.2-13.0 deg in the pickup,
     from the ball drop); F080's table gave the tilt after the weld instead.
- Text should now say: the newcomer joins the flying ring from the floor in about 32 s on the square and 20-34 s on the
  triangle (one cell needing 3 contacts), and after a pickup and drop in 97-99 s; the ring tilts at most 2.5 deg after
  the weld. In the simulation the welded newcomer can tip over in the fleet's landing (a known effect of the twin's
  rigid links; the rig's tether goes slack).
- Evidence: registry R4094c; the runs of F080.

### F082 2026-11-07 15:17 RESULT (simulation, claim): 4.5 comparison with Quan et al., FINAL (final tree and masses; X11 re-fly of 23-24 Oct's C, hub and world-C cells)
- What: our planner against the network (our port of Quan et al. at its search-best gains, and the paper's damping
  form at its offline-best gains) on the rim ring, on the hub (every cable on a 1 mm hub, the paper's own geometry) and,
  as one row, the law as printed on world C. 32 runs R4108-R4139, x2 per config, one code tree (section-5 hash
  5e7b1d3bd133; diff_sha256 319c020f6359; bf52a8a), every config the 23-24 Oct one with the slope filter (nothing else
  changed). Scored by tools/c_metrics.py on each world apart (the scorer pools by arm and case, not world). No void:
  every run valid by the card's VOID rule; every run not lost landed.
- The 6 FAILs (R4126-R4131, the paper's form, both repeats of hover, line 0.2, line 0.4) are LOSSES, the result itself
  (the card: "the method failed: scored, never re-flown"): 20-25 s after the hand-over the ring tipped (over 30 deg at
  +23.3 / +24.5 s in hover, +19.8 / +20.6 s on line 0.4, window peaks 37.6-70.9 deg; on line 0.2 the abort came at
  +23.9 / +25.4 s, after the last whole cycle scored) and a tracker's ENVELOPE FAULT (payload tilt 63.1-74.5 deg > 60)
  disarmed the fleet before LAND. The same as 23 Oct (R2744-R2749: all 6 lost).

  | World, case (tested number, mm) | Our planner | The port | The paper's form |
  |---|---|---|---|
  | Rim, hover (3-D error from the hand-over) | 1.1 [1.1-1.1] | 11.8 [11.8-11.9] | lost 2 of 2 |
  | Rim, line 0.2 (clock-free + reach) | 15.1 [14.8-15.3] | 38.9 [38.0-39.8] | lost 2 of 2 |
  | Rim, line 0.4 | 41.7 [41.4-42.0] | 96.4 [94.4-98.4] | lost 2 of 2 |
  | Hub, hover | 1.3 [1.2-1.3] | 10.8 [10.8-10.9] | not flown |
  | Hub, line 0.2 | 28.7 [28.5-28.9] | 37.5 [37.4-37.7] | not flown |
  | Hub, line 0.4 | 69.3 [68.8-69.9] | 99.5 [99.4-99.5] | not flown |

  Beside it, not tested: the port's lag 1.14-1.26 s against ours 0.07-0.21 s; reach (rim line 0.4: ours 40 mm short,
  port 88 mm); ring yaw, largest change: ours <= 0.3 deg everywhere, the port 2.7-7.7 deg on the rim and 20.6-35.9 deg
  on the hub's 0.4 line; ring tilt peak ours 0.2-5.8 deg, port 0.5-11.6 deg; planner tick median ours 58-60 ms, the
  network 4.5 ms.
- World C, the law as printed (R6c reading, vel_k_att 8; R4138 / R4139): carries the ring and lands; 146.7 / 141.9 mm
  3-D RMS from the target, the ring 66.6 / 66.9 mm ABOVE it on average (first 5 s: 81 mm at most from 1.0 m); ring off
  the quads' centroid 4.2 mm, quad-quad distances within 0.69 % (std / mean), estimated tensions within 2.8 %. Its
  exploratory falsifier of 24 Oct (ring > 0.10 m off 1.0 m) does not fire. Not like for like with R2805 / R2812 (the R6b reading) and R2813 (R6c)
  (110.0 / 118.3 / 109.1 mm, height -3.6 / -3.7 / -4.8 mm): the tree, the filter and world C's mass split changed (its
  rods 0.075 -> 0.040 kg, the airframe base 0.435 -> 0.470 kg; the law types quan_rod_mass 0.055); which moved the
  height is not shown. Our planner on world C did not fly in this batch (last R2791, 23 Oct, another tree), so the world-C
  arms comparison (A, O) is not made here.
- Verdicts against the cards' bars, with their provenance:
  - Rim (C's section-5 claim rule: written by the lab chat 22 Oct; the headline and windows Wesley's, 22 Oct (22);
    never frozen by Wesley, who said 23 Oct "report C as is; the bars matter less than that it works"): "our planner
    at least as close" CONFIRMED against the port in all three cells (every run, ranges apart by 10.7 / 22.7 / 52.4 mm
    against a minimum effect of 1.0 / 1.5 / 4.2 mm) and against the paper's form in all three (its runs lost rank
    behind). At n = 2 per arm: the rule asks 3 valid runs, so short of its n, as on 23 Oct.
  - Hub (G, P, A, O FROZEN by Wesley 23 Oct 16:22, before the hub runs): against the frozen rim pair (R2685-R2690:
    17.4 / 41.8 / 101.9 mm): hover G not met (10.8 > 8.7), P not met (< 13.9): "partly the geometry"; lines P met
    (37.5 >= 33.4; 99.5 >= 81.5): "the port". Read on this tree's rim pair (11.8 / 38.9 / 96.4): P met in all three ("the
    port"), as 23 Oct's same-tree re-read. A: "our planner at least as close on the paper's own geometry" in all three
    (gaps 9.5 / 8.5 / 29.5 mm). O: holds (all six of ours valid). The scorer prints the paper form's hub hover as
    "confirms (the port's hover cell)": no paper-form run flew on the hub; disregard.
  - World C (Q1, Q3, Q4, A, O: PROPOSED 23-24 Oct, never frozen; Wesley 30 Oct: the exact law is one row beside our port):
    reported, not judged.
  - Against Quan et al.'s own numbers: nothing is tested (their accuracy is stated beside, card Q5 dropped).
- Against 23-24 Oct (tree, masses, filter all changed; not like for like): rim ours 1.3-1.4 / 17.0-17.6 / 44.3-46.2, the
  port 16.1-18.7 / 41.5-42.1 / 99.3-104.5 (same-tree re-read 14.2 / 42.6 / 102.6); hub ours 1.4 / 31.1 / 72.5, the port
  12.2 / 40.0 / 102.4; the paper's form lost all six then and now. Recorded misses stay: R2685 tipped past 30 deg at
  +63.9 s, after its window; R2793 and R2809 (the law as printed, other readings) lost.
- Our planner carries the hub worse than the rim on the lines (28.7 vs 15.1, 69.3 vs 41.7 mm, mostly reach: 27 / 68 mm
  short), as on 23 Oct: cause not tested.
- Text should say: on the rim ring our planner holds the hover to about 1 mm and the 0.2 / 0.4 m/s line to 15 / 42 mm
  (path plus reach), against 12 mm and 39 / 96 mm for our port of Quan et al.'s method, which lags the path by about
  1.2 s; the paper's damping form at its best offline gains loses the ring in every run, 20-25 s after the hand-over.
  With every cable on a central hub (the paper's own geometry) the port's ring stops turning in hover and on the slow line
  but its error barely changes (11 / 38 / 100 mm against our 1 / 29 / 69 mm). The law as printed, on world C, carries
  the ring about 0.14 m from its target, its estimated rod tensions within 3 % of each other.
- Evidence: registry R4108-R4139; results/analysis/x11_quan_2026-11-07.json (rim, hub, world_c apart). Reviewer done: see F083 (corrections).

### F083 2026-11-07 15:27 CORRECTION of F082: the comparison with Quan et al. on the final masses, after its reviewer
- The reviewer (7 Nov) recomputed from the raw logs: one tree (5e7b1d3bd133; diff_sha256 319c020f6359), no void, no late
  loss; the hover cells exact (3-D RMS to the held reference over 60 s), the line cells the card's path error (clock-free
  part within 0.3 mm, reach within 7 %); the six paper-form losses as F082 says and as on 23 Oct; the hub freeze
  (23 Oct ~16:22) before its runs (18:56-19:16); world C "reported, not judged" right (its bars still PROPOSED); nothing in
  the thesis draft compares with Quan et al.'s own published numbers. Corrections:
  1. The rim's bars: Wesley ACCEPTED the card's seven C values on 23 Oct ~07:25 (decisions.md: "C: reported as is ...;
     its seven card values accepted"), before every claim run; which seven is not recorded, and the card header was
     never marked FROZEN. ("Not so important as long as it works" was about S's bars.) The verdict does not change.
  2. World C did not fly the same law as 24 Oct: R2805 / R2812 flew c_net_quan_const_katt8.yaml; R4138 / R4139 fly
     c_net_quan_const_katt8_r6c.yaml (Wesley's choice of 30 Oct 16:26, the params of R2813).
  3. Loss timing: the ring first passes 30 deg at +19.8 to +24.7 s after the hand-over (when the tracker's DROP REQUEST
     fires, ignored in network mode) and the fleet aborts at +23.9 to +29.7 s (envelope fault 63-75 deg). "Loses the ring
     20-25 s after" fits the tip, not the abort.
  4. The hub hover's "partly the geometry" holds only against the frozen 23 Oct rim pair (another tree, masses and filter);
     against this tree's own rim pair, the like-for-like reading, P is met in all three cells: "the port".
  5. Missing from the text: on the rim the port's ring now turns only 2.7-5.4 deg in hover and 6.2-7.5 deg on line 0.2
     (23 Oct: 22-51 deg); on the hub's 0.4 line 20.6-35.9 deg against 6.2-7.7 on the rim. **The thesis draft is stale**:
     results_discussion.tex line 325 ("14 to 51 deg", "the hover error fell by 30 %": now 11.8 -> 10.8 mm, 8 %). The hub is
     not "the paper's own geometry" (it sits 2 cm above the centre of mass; world C is the paper's attachment).
  6. Our planner's tick median is 56.3-60.0 ms.
- Text should now say: against the port of the paper's method our planner tracks closer in every cell (rim hover 1.1 vs
  11.8 mm, line 0.4 41.7 vs 96.4 mm; hub 1.3-69 vs 10.8-99.5 mm); the paper's own damping form loses the ring 20-25 s
  after the hand-over in every case; the hub geometry does not by itself close the gap; the paper's attachment at the
  centre of mass (world C) holds the ring 142-147 mm from its target.
- Evidence: registry R4108c; the runs of F082.

### F084 2026-11-07 15:38 RESULT (simulation, claim): 4.2 the gap table (tab:res_gap), FINAL on the final tree and masses (replaces the 23 Oct numbers the draft cites)
- What: one of four carriers let go without telling the planner, the largest gap the three survivors leave set by the
  layout (120-180 deg; support margin 112-0 mm); final tree, final quadrotor masses, slope on the carriers, the
  sideways integral as flown (Wesley 5 Nov ~14:00). 10 runs R4140-R4149 (chunk 9) plus 4.2's R4033 / R4032 (chunk 7,
  F076 / F077) as the 120 deg arm's first two repeats. One tree (code hash 5e7b1d3bd133; diff_sha256 8d39976a843a,
  chunk 7's 2863a8be2293, bf52a8a), no void, 0 failed solves, RTF 0.71-0.78.
- Scorer: tools/release_bands.py metrics (the section-10 scorer: dtheta, the tilt vector's largest change from its
  5 s pre-release mean, event to +15 s). The drop and fault arms have no LAND event, so they are cut at the planner's
  DROP AND LAND or the first ENVELOPE FAULT line on the ARM-stamp clock (the 23 Oct rows' rule, as drop_metrics maps it)
  and scored ">=". Detection is the planner's first smaller problem after the RELEASE (planner log, 10 Hz, on the exact
  clock t_ready_sim: the load position matches run.csv within 0.03 mm, so no detection-based alignment).

  | Largest gap (margin) | Runs | Detection (s) | dtheta (deg) | Absolute peak (deg) | Back in band, T6 (s) | Outcome |
  |---|---|---|---|---|---|---|
  | 120 deg (112 mm) | R4033, R4032, R4149 | 0.36-0.37 | 6.15, 6.45, 6.35 | 5.3-5.6 | 3.6-3.8 | held, landed |
  | 135 deg (86 mm) | R4140, R4141 | 0.27, 0.37 | 10.12, 10.50 | 9.6-9.8 | 3.9-4.0 | held, landed |
  | 150 deg (58 mm) | R4142, R4143 | 0.27, 0.29 | 15.81, 16.25 | 15.5-15.8 | 4.2, 6.1 | held, landed |
  | 165 deg (29 mm) | R4144, R4145 | 0.30, 0.28 | 30.00; >= 30.31 | 29.84; 30.05 | 8.0; - | R4144 held (peak just under the 30 deg drop request); R4145 dropped at +0.58 s, everyone down in 8.2 s |
  | 180 deg, drop rule off (0) | R4146, R4147 | 0.29, 0.28 | >= 60.77, >= 62.12 | 60.8, 62.2 | - | 60 deg envelope fault at +0.71 / +0.62 s, the fleet disarmed in the air (the arm's expected end) |
  | 180 deg, drop rule on (optional, n = 1) | R4148 | dropped at detection | >= 21.31 | 21.3 | - | the planner's gap rule dropped the ring at +0.29 s, everyone down in 8.1 s |

  The freed quadrotor is disarmed alone, 0.28-0.31 s after it is at rest and before LANDED, in every run from 120 to
  165 deg except R4145 (the drop's LANDED disarmed it with the fleet) and in R4148; R4033's tipped onto its side after
  its lone disarm (F077), every other freed quadrotor stayed level.
- Against the card's bars (Tier B card section 5, frozen by the lab chat 16 Oct 08:41 before the 0.55 m runs, "set while
  Wesley is at work"; applied at 1 m by the re-fly card, frozen by the lab chat 23 Oct 02:15 under overnight authority;
  judged after the run, F075): B1 CONFIRMED (G120 6.32 -> G150 16.03, +9.71 against the 1.00 needed; G150 -> G180
  >= 61.45, +45.42 against 2.70); B2 CONFIRMED (every neighbour pair resolved with the right sign: +3.99, +5.72,
  >= +14.13, >= +31.30); B3 not flown (the planned-pull arm, dropped by Wesley 2 Nov); B4 CONFIRMED (G120-G150 absolute
  peaks 5.3-15.8 deg, under 30); B5 not confirmed and not falsified (no abort or fault latch from 120 to 165 deg, but
  R4145's freed quadrotor was not disarmed alone, as R2607 / R2613 on 23 Oct).
- Readings, not judged: R4146's planner ran 1.70 % of its run-span ticks over 100 ms (7 of 411, max 189 ms, five of them
  in the 6.3-3.6 s before the release; the span ends 8 s after the fault): over the batch's general 1 % late-tick rule,
  not the Tier B card's (solver fault or TB14 re-flown); its repeat R4147 (0.24 %) ends the same way. Every other run
  0-0.74 %, the only late tick after a release the resize tick (+0.27-0.37 s, 101-185 ms).
- Against 23 Oct (R2486 / R2495 / R2504, R2604-R2614): not like for like (tree be34f95, earlier masses, ema). Then:
  120 deg 6.9-7.5, 135 10.4-11.0, 150 17.6-18.2, both 165 deg runs dropped (>= 29.8-30.5), 180 >= 61.9-62.5,
  detection 0.25-0.34 s. Now 120 deg is 0.5-1.3 deg and 150 deg 1.3-2.4 deg lower, and one 165 deg run of two held.
  Which change moved them is not shown.
- **The thesis draft is stale**: results_discussion.tex lines 79-96 (tab:res_gap and its paragraph) cite the 23 Oct
  numbers: "about 7 deg at 120 to 18 deg at 150", "at 165 the ring was dropped", "detection took 0.25 to 0.34 s", and
  the table's 6.9-7.5 / 10.4-11.0 / 17.6-18.2 / "Over 29.8, ring dropped" / "Over 61.9" / 0.25-0.34 s.
- Text should now say: the tilt rise grows with the gap, about 6 deg at 120 deg, 10 deg at 135 deg and 16 deg at 150 deg;
  at 165 deg the ring reaches the drop rule's 30 deg (one run of two dropped, the other peaked at 29.8 deg); at 180 deg
  with the drop rule off it tips past 60 deg and the fleet disarms in the air, and with the rule on it is dropped at
  detection. Detection took 0.27-0.37 s.
- Evidence: registry R4140-R4149 (and R4032 / R4033); results/analysis/x11_gap_2026-11-07.json. Reviewer done: see F085 (corrections).

### F085 2026-11-07 15:46 CORRECTION of F084: the gap table on the final masses, after its reviewer
- The reviewer (7 Nov) recomputed every number from run.csv (dtheta, peaks, detection, cut times within 0.01 deg / 3 %);
  one tree (5e7b1d3bd133; diff_sha256 8d39976a843a, chunk 7's 2863a8be2293); each config differs only in azimuth and
  world (and drop_on_loss false at 180 deg, declared). Corrections:
  1. R4146 is VALID. Neither card has a late-tick rule; the 1.70 % counted wall-clock compute over 100 ms (tick_ms), not
     the batch's late-tick rule (a sim-time step over 1.5 x 0.1 s, tools/s_metrics.py), by which R4146 reads planner
     0.00 %, trackers 0.40 %. One slow tick (the +0.29 s resize, every run has it) falls in the scored window.
  2. The 165 deg run that held (R4144): dtheta 30.00 deg (the tilt vector's change from a 0.29 deg mean) while the drop rule
     tests the ABSOLUTE tilt (over 30 deg on 2 samples), which peaked at 29.84 deg: no drop. A knife edge (on 23 Oct R2613
     dropped at 29.57 in run.csv while the tracker read 30.6). Text: "one run of two crossed the line and was dropped at
     +0.58 s; the other peaked at 29.8 deg".
  3. Bar provenance: Tier B's bars were set by the working session on 16 Oct (card header "FROZEN Fri 16 Oct 08:41 ...
     set while Wesley is at work"; decisions.md records only the list of gaps) and applied at 1 m on 23 Oct; the re-fly
     card contradicts itself (section 5 "FROZEN 23 Oct 02:15 by the integrator", its header still DRAFT / PROPOSED). Never
     approved by Wesley; judged after the run (F075).
  4. Caveats beside the verdicts: B1's 150 -> 180 and B2's 165 -> 180 pairs also differ in drop_on_loss, so their +45.4 and
     +31.3 deg measure the gap between the 30 deg drop rule and the 60 deg fault (quote signs only); with the drop rule on
     (n = 1) 165 -> 180 reads 30.15 -> >= 21.31. Not counting R4145 / R4148's drop and R4146 / R4147's fault as S misses
     is a reading after the run. R4033's freed quadrotor tipped over after its disarm (TB14, the named re-fly case, not
     re-flown); it changes neither dtheta nor B5.
  5. "With the rule on it is dropped at detection" is one run (R4148, n = 1); its freed quadrotor disarmed 0.19-0.20 s
     after rest.
  6. **Thesis draft stale**: results_discussion.tex lines 79 and 91-95 cite the 23 Oct numbers (not like for like), and
     line 79's "whether the ring survives a detachment is therefore decided by the layout before the flight" is ruled out
     by Tier B's own "cannot say" (rigid rods; not a real-world safety margin).
- Text should now say: the wider the gap the survivors leave, the more the ring tilts at the release: 6.2-6.5 deg at
  120 deg, 10.1-10.5 at 135, 15.8-16.3 at 150; at 165 deg one run of two crossed the 30 deg drop line and was dropped,
  the other peaked at 29.8 deg; at 180 deg without the drop rule the ring passes 60 deg within 0.7 s (with it, one run,
  dropped at detection). Detection 0.27-0.37 s.
- Evidence: registry R4140c; the runs of F084.

### F086 2026-11-07 16:52 DECISION (Wesley): every simulation claim stated as is; compare with other papers where possible
- Wesley: "the bars are arbitrary so it is fine to state results as is and compare to other papers where possible". This
  covers every claim of the 5-7 Nov batch, not only X3 (F075). The registry and cards keep each miss as scored; the text
  gives the numbers (each entry's "text should say", as corrected by its reviewer) without bars or "falsified".
- Where a result can sit beside another paper's published numbers (e.g. the comparison with Quan et al., which so far
  compares with our port of their method only), the text may compare; the lab chat can pull the numbers on request
  (docs/lab_requests.md).
- Final and reviewed today: F065/F066 X3, F067/F068 X3b chains, F069/F070 X2, F071/F072 rejoin, F073/F074 X8, F076/F077
  4.1 / 4.2, F078/F079 X11 U, F080/F081 joins, F082/F083 Quan, F084/F085 the gap table. Still to come: X1 (flying), the
  six X3b real-time cells and X8's hub integral-off arm (tonight), X3b's idle bench (Sunday morning).
- Stale in the draft (results_discussion.tex): lines 79-96 (the gap table, F085), 201 (catching off the plate, F079), 325
  (the port's ring and hover error, F083).

### F087 2026-11-07 17:35 RESULT (simulation, claim): X1 the release ablation, FINAL on the earlier quadrotor masses (replaces F059/F060's numbers); the switch of solvers recounted
- Section: the results (the unannounced release: what the resize is worth; 4.2.3 the switch of solvers), the conclusion.
- What changed: X1 re-flew on the masses of before 4 Nov (Wesley 6 Nov ~11:35): 44 runs R4150-R4193, the five arms (the
  proposed planner resizing at detection, no resize, the resize 0.5 / 1 / 2 s late) x2 in h8, h4, f4, sqTnoxy, and ours /
  no resize x2 in h4p115. One tree (section-5 hash 057e999eadce; diff_sha256 cc4c1a017289): bf52a8a plus the old-mass
  worlds and models, tools/make_oldlink_worlds.py, the batch tools (lab_batch.py, lab_sync.sh) and an unused X8 twin config;
  no src/ change. Against the final-mass X1 (R3821-R3906, plain bf52a8a) only the worlds differ for the flight. No void:
  every release detected (0.27-0.38 s), no drop, loss or abort, late ticks at most 0.40 % (planner) / 0.23 % (trackers),
  0 failed solves, RTF 0.22-0.81 (h8 0.22, as before). R4178's planner read-back file is missing; its run times show the
  2 s delay (detect 0.36, resize 2.36 s).

  | Cell (x2) | Arm | Tilt rise (deg) | Dip (mm) | Settled tilt (deg) | Back in band, T6 (s) | Tilt from +6 s (deg) |
  |---|---|---|---|---|---|---|
  | 4 quads, hover (h4) | resize at detection | 7.04-7.06 | 32.7-33.8 | 0.03-0.06 | 3.9-4.0 | 0.50-0.51 |
  |  | no resize | 6.77-7.10 | 32.0-33.5 | 0.21 | 3.5-3.6 | 0.55-0.58 |
  | 8 quads, hover (h8) | resize at detection | 1.61 | 14.8-15.2 | 1.06 | 4.5 | 1.40 |
  |  | no resize | 1.58-1.66 | 15.2-15.5 | 1.18 | 6.2 | 1.58-1.63 |
  | 4 quads, figure-8 (f4) | resize at detection | 6.95-7.32 | 30.8-31.3 | 3.64-3.66 | 12.7-12.9 | 5.25-5.29 |
  |  | no resize | 6.83-7.01 | 30.4-31.1 | 3.62-3.65 | 12.8-12.9 | 5.10-5.14 |
  | 4 quads, 1.15 kg ring (h4p115) | resize at detection | 9.66-9.76 | 44.7-45.8 | 0.06 | 5.4-5.5 | 0.24-0.25 |
  |  | no resize | 9.46-9.74 | 44.9-48.3 | 0.20-0.22 | 4.2-4.4 | 0.54-0.55 |
  | Square to a T (sqTnoxy) | resize at detection | 56.9-58.1 | 97.5-99.3 | 0.59-0.63 | 4.8-5.3 | 1.28-1.30 |
  |  | no resize | 57.2-57.7 | 107.1-108.0 | 2.20-2.21 | 6.1-6.2 | 3.09-3.10 |

  Tilt rise is release_bands' delta-theta (the tilt vector's largest departure from its 5 s pre-release mean); settled is
  the mean tilt magnitude over the last 5 s before LAND; T6 the first entry into the band held 1 s (its last exit is in
  the json: h4 6.6-6.8 vs 5.5 s, h8 4.5 vs 3.5 s, sqT 14.1-14.3 vs 15.2 s). The late resizes (0.5-2 s) end at the
  resized arm's settled level (h4 0.01-0.08 deg, sqT 0.56-0.65 deg) with no order between them in the rise.
- Against the card's bars (set by the lab chat before any X1 run, 30 Oct ~12:50, the sqT addendum ~15:20, after Wesley's
  plan of ~12:30; not approved by him as values; judged after the run, F086): h4, h8 B1 and B2 FALSIFIED, B4 holds; f4
  (reported) B1, B2 FALSIFIED, B4 holds; h4p115 B1 FALSIFIED, B4 MISSES by T6 > 5 s (5.4-5.5; the frozen t_level rule
  holds); sqTnoxy U1 and U2 CONFIRMED (U1 by the dip, +9.2 mm, and the settled tilt, +1.59 deg, without the resize), U5
  MISSES in R4181 (T6 5.3 s > 5). The scorer counts no void.
- Against the final masses (F059/F060, not like for like: the masses changed): the release transient is the same (h4
  rise 6.4-7.0 then); the square-to-T cell changes: the resize now leaves the T at 0.6 deg against 2.2 deg without it,
  as on the earlier masses on 30 Oct (R2998/R2999, R3202/R3203), where the final masses gave 1.74 vs 1.80 deg. F060's
  records stay: the resize does not settle faster in h4 (last exit 6.6-6.8 vs 5.5 s now too).
- The switch of solvers (4.2.3; docs/analysis/switch_first_solve_1m_2026-10-31.md's method, switch() of
  tools/thesis_freeze_tables.py on the planner logs; results/analysis/x1_oldlinks_switches_2026-11-07.json): 34 switches,
  one per resizing run (4 -> 3 in h4, f4, h4p115, sqTnoxy; 8 -> 7 in h8; the 10 no-resize runs do not switch). The first
  published solve of the smaller problem never failed (status 0, 5 SQP steps, "solved"), nor the solve after it, nor any
  of the 342 solves in the second after it. Its stationarity residual reached at most 1.5e-3 (sqT; 9.5e-5 at eight) against
  4.1e-3 for the larger problem's last solve, but per switch the first solve's residual is the larger one in 24 of 34
  (ratio up to 18; at eight 9.5e-5 vs 3.2e-5). Simulated solve times (14-167 ms) are load on the lab computer: not quoted.
- Text should now say: every release was caught in 0.27-0.38 s and nothing dropped. The resize at detection leaves the
  release's own transient unchanged (tilt rise and dip within the repeat spread) and the ring more level afterwards: by
  0.12-0.18 deg with four or eight quadrotors (0.03-0.06 against 0.21 deg at four), and by 1.6 deg when a square becomes a
  T (0.6 against 2.2 deg), where it also lowers the dip by 8-10 mm. It does not settle faster: at four the no-resize arm
  enters and last leaves the band sooner (3.5-3.6 against 3.9-4.0 s; 5.5 against 6.6-6.8 s); at eight the two measures
  disagree (first entry 4.5 against 6.2 s, last exit 4.5 against 3.5 s). Across
  34 switches the first solve of the smaller problem never failed, nor any solve in the second after it; its residual
  stayed within the range of the larger problem's solves (at most 1.5e-3 against 4.1e-3), though often above the very
  last one before the switch.
- Stale in the draft (results_discussion.tex): the X1 table and paragraph (lines 101-132) cite F035 / R3520-R3563 (another
  tree and the final masses); the square-to-T placeholder (line 134) can now be filled; line 138's "20 resizes ... at most
  7.5e-4, similar to the 8.3e-4 of the last solve" becomes 34 switches, 1.5e-3 against 4.1e-3, and "similar" holds only for
  the range (first above last in 24 of 34).
- Evidence: registry R4150-R4193; results/analysis/x1_oldlinks_2026-11-07.json (+ _bars.json), x1_oldlinks_switches_2026-11-07.json;
  card docs/experiments/2026-10-30_x1_release_ablation.md. Reviewer done: see F088 (corrections).

### F088 2026-11-07 17:44 CORRECTION of F087: X1 on the earlier masses, after its reviewer
- The reviewer (7 Nov) reproduced every number from the raw logs (within 0.01 deg / 0.1 mm / 0.05 s); one tree
  (057e999eadce; against X3's 5e7b1d3bd133 only added worlds, a tool and a config; src/ identical to the final-mass X1);
  each arm differs from its pair only in resize_delay_s; no void; R4178's missing read-back hides nothing (its other 42
  read-backs and the planner's own params match R4179). The rise is the vector departure (absolute 3.9-4.0 deg at h4).
  Corrections:
  1. Settling: at EIGHT quadrotors too, no resize settles sooner. F087's "the two measures disagree" is a scoring
     artefact: the no-resize ring drifts to a new level at +6.2 s (1.58-1.66 deg) and T6's search starts at that late
     peak; it is within 0.15-0.17 deg of its settled tilt by +4 s and last leaves the band at 3.45-3.49 s (ours 4.46-4.52).
  2. The resize adds a larger second swing: absolute tilt at +2.1 s 4.19-4.29 vs 3.20-3.27 deg (h4), 3.70-3.74 vs
     3.02-3.06 (h4p115); at h8 ours' rise is that swing (1.61 vs 1.32-1.39 deg). "Unchanged transient" holds for the first
     peak and the dip only. "More level after" holds in hover only: on the figure-8 (f4) settled 3.64-3.66 vs 3.62-3.65 deg,
     after +6 s 5.25-5.29 vs 5.10-5.14.
  3. The square-to-T result (0.6 vs 2.2 deg; dip means differ by 9.2 mm) holds under two conditions: the sideways integral
     off (with it, R2965 / R2972: ours 8.3 / 10.5 vs 2.2 deg) and the earlier masses. On the hardware-matched masses it was
     1.74 vs 1.80 deg (F059 / F060). X1 has no returns; its move to the earlier masses was decided after that result
     (decisions.md 6 Nov ~11:35, on the lab chat's recommendation). The text must give both conditions. Wesley is asked
     which X1 the thesis reports (7 Nov).
  4. Switches: "often" understates the per-cell picture. Within a cell the largest first-solve residual is above every
     last-solve residual at h8 (9.5e-5 vs <= 3.2e-5) and f4 (5.9e-4 vs <= 4.8e-4); 21 of the 24 are in the late arms, 3 of
     10 in the resize-at-detection arm (ratios 1.06-1.27); in sqT ours the residual reached 1.6e-2 to 2.2e-2 at +0.1 s
     (status 0; R4180 / R4181). No solve failed (0 of 342).
  5. Bars: no X1 bar values were approved by Wesley; B4's T6 rule was revised ~13:10 31 Oct, after the three X1 smokes;
     U1-U5 were frozen for sqT, not sqTnoxy, and decisions.md 31 Oct ~10:38 (2) reports the new T runs' settling with no
     bar, so "U5 misses in R4181" applies a bar that decision lifted.
- Stale in the draft (results_discussion.tex): lines 101-132 (the X1 table, R3520-R3563), 134 (placeholder), 138 (20
  resizes, 7.5e-4 vs 8.3e-4). Line 101's "second swing of about 4 deg, 2 s after" still holds.
- Text should now say (pending Wesley's choice of masses): resizing at detection does not change the first peak or the
  dip, adds a larger second swing about 2 s after, leaves the ring more level in hover (h4 0.03-0.06 vs 0.21 deg, h8 1.06
  vs 1.18) but not on the figure-8, and does not settle sooner. Its switches never fail a solve (0 of 342), though the
  first solve after a switch is often less converged than the last before it.
- Evidence: registry R4150c; the runs of F087.

### F089 2026-11-07 17:46 DECISION (Wesley): X1 reported on both quadrotor models, side by side
- X1 (the release ablation) goes in the thesis on both models with the conditions stated: the hardware-matched masses
  (R3821-R3906, F059 / F060) and the earlier split (R4150-R4193, F087 / F088). For this table the text explains the two
  mass splits (an exception to "no explanation" of 6 Nov, which covered the claims with returns).
- What differs between them: the square-to-T benefit appears only on the earlier model (0.6 vs 2.2 deg, with the
  sideways integral off; 1.74 vs 1.80 deg on the final masses). On both, the resize leaves the ring more level in hover,
  not on the figure-8, and does not settle sooner (F088).
- The lab chat can give the side-by-side table on request (docs/lab_requests.md).
- Evidence: decisions.md 7 Nov; F059, F060, F087, F088.

### F090 2026-11-07 18:24 RESULT (simulation, claim): X3b's C8 and N9 on one model (the earlier quadrotor masses)
- What: the six real-time cells queued after F068 (Wesley 7 Nov), x3b_s9_hover_rt (R4194, R4195), x3b_s8_fig8_rt (R4196,
  R4197), x3b_s8_hover_rt (R4198, R4199), flown on the earlier masses, so C8 (eight carriers at one iteration against five,
  three cells) and N9 (nine carriers, hover and chain) each sit on one model. Same-model comparators: the chains R3980 /
  R3981 (9) and R3984 / R3985 (8) (F067 / F068); X3's it5 arms R3974 / R3975 (hover), R3976 / R3977 (figure-8), R3972 /
  R3973 (chain), and the 4-carrier hover R3978 / R3979 (N1's bars 5.65 mm, 1.35 deg). Bars FROZEN 30 Oct ~15:46 by Wesley
  before any X3b run (decisions.md; card section 5); read as results stated as is (F086).
- One tree for the six (section-5 hash 057e999eadce, diff_sha256 45436aa0c6fc, bf52a8a, as X1): against the chains'
  5e7b1d3bd133 only the X1 worlds, tools/make_oldlink_worlds.py and configs/twin/x8_hub_ours_noxy.yaml are added; no
  flight code. Each config differs from its final-mass copy only in world. No void: late planner ticks <= 0.27 %, failed
  solves 0, every published tick at one SQP call, max res_eq 3.4e-5 to 9.6e-5 (N6 holds); RTF 0.19-0.25 (flagged; these
  cells have no release).

  | Cell (x2) | 1 iteration (rt) | 5 iterations (it5) | Verdict |
  |---|---|---|---|
  | 8, hover | N1 1.48-1.62 mm, 0.07 deg | 1.0-1.4 mm | same verdicts; every close-to-it5 item holds (bar 4.19 mm) |
  | 8, figure-8 | path 10.24 mm, tilt 1.08-1.10 deg | path 15.3-17.3 mm, 1.30-1.40 deg | same verdicts; every close-to-it5 item holds |
  | 8, chain (F067 / F068) | N4 6.15-6.20 mm; L, J hold | N4 6.45-6.56 mm | same verdicts (N4 position missed by both) |
  | 9, hover | N1 1.63-1.79 mm, 0.06-0.07 deg | - | N1 holds |
  | 9, chain (F067 / F068) | L, J hold; N4 5.98-6.04 mm | - | L, J hold |

- Verdicts (against the card's bars, read as results as is): C8 CONFIRMED on one model (all three 8-carrier cells give
  the same verdicts as five iterations and every close-to-it5 item holds); N9's simulation part CONFIRMED (N1, L, J hold
  in both repeats; N4 reported). The final-mass record stays: there the chain cell FAILED C8 (R3807's fold-in 3.97 deg
  against 2.95; F057, F068 item 4).
- Readings, not shown causes: on the figure-8 one iteration tracks the path closer than five (10.2 against 15.3-17.3 mm);
  in hover slightly wider (1.5-1.6 against 1.0-1.4 mm).
- Timing (sim-time runs at RTF 0.19-0.25: they test nothing about real time): at full size (constant n in these cells)
  solve p95 60-67 ms at eight and 85-86 ms at nine; the whole tick p50 89-92 / 121 ms, p95 104-115 / 146-148 ms, of which
  the per-tick N6 residual (logging, off in the real-time setting) p50 29-30 / 39-41 ms. R8 (closed-loop tick) stays not
  shown: R8b FALSIFIED 31 Oct (R3451), R9b void; R9a's idle bench runs tonight on the laptop (00:30-06:30).
- Text should say: at eight carriers one planner iteration per tick gives the same results as five in hover, on the
  figure-8 and through the chain of releases and returns, at about a quarter of the solve time (solve p95 60-67 ms against
  240-256 ms); at nine carriers the ring holds in hover and through the chain. The simulation does not show real-time
  operation (the ground computer's bench follows).
- Evidence: registry R4194-R4199; results/analysis/x3b_oneModel_2026-11-07.json (tools/x3b_metrics.py). Reviewer done: see F093 (corrections).

### F091 2026-11-07 18:26 RESULT X8's hub with our integral off (simulation, claim arm; final masses); and the X1 two-model table
- What: X8's hub cell (world C) re-flown with our arm's sideways integral off (int_k_xy 0), so both X8 cells follow the
  5 Nov T rule (Wesley 7 Nov ~10:10; F074 item 7). R4200 / R4201 (x8_hub_ours_noxy). The read-back differs from the
  integral-on arm R4008 / R4009 only in int_k_xy (0.0 vs 5.26). Flown on X1's tree (code hash 057e999eadce; diff_sha256
  45436aa0c6fc), which adds to X8's 5e7b1d3bd133 only configs/twin/x8_hub_ours_noxy.yaml and the two X1 _oldlinks worlds
  and changes tools/make_oldlink_worlds.py: no flight code. Both valid (late planner ticks <= 0.11 %, 0 failed solves,
  RTF 0.68-0.84), both landed, no loss by the card's rule. Our arm resized at +0.41 / +0.43 s (the port is told at
  +0.35 s, F074).

  | Hub, our arm | E (mm) | dip (mm) | centre's end shift (mm) | t_s (s) | free ring over 80 deg | ring tilt last 5 s before LAND |
  |---|---|---|---|---|---|---|
  | integral on (R4008 / R4009, F073 / F074) | 286.2 / 290.4 | 65.5 / 66.0 | 1.4 / 1.2 | 5.33 / 5.37 | 11.0 / 11.3 s from +11.6 s, peak 96.3 / 97.2 deg | 26-66 deg |
  | integral off (R4200 / R4201) | 287.3 / 288.3 | 66.7 / 66.6 | 33.9 / 33.2 | 5.54 / 5.61 | 17.0 / 17.3 s from +12.7-12.9 s to LAND, peak 100.4 / 103.0 deg | 89-102 deg |

- Against the card's bars (frozen 30 Oct 16:26 by Wesley before any run), with this arm as ours: L1 (hub) CONFIRMED on E
  alone (bar ours_max + 20 = 308.3 mm; port 373.3 / 371.9, R4010 / R4011; the port's 300 mm stretch is on the 1 s
  boundary within one sample, F074); L3 holds. The same verdicts as with the integral on. Results as is (F086).
- What changes with the integral off: the ring's centre ends 33-34 mm off its point (a steady sideways offset no longer
  cancelled), and the free ring (its cables meet at its centre of mass, so its attitude is not controlled) stays past
  80 deg from about +13 s to LAND and ends at 89-102 deg, against swinging back to 26-66 deg with the integral on. Why
  the integral changes the free ring's attitude is not shown (a reading).
- Survivors' largest gap over the last 5 s: 176 deg (171 with the integral on).
- Text should say: on the hub our planner keeps the ring's centre within 287-288 mm at its worst, against the port's
  372-373 mm, and settles it in 5.5-5.6 s; with the sideways integral off it ends 33-34 mm off its point, and the free
  ring turns past 80 deg about 13 s after the release and stays there until the landing.
- X1 on both models (Wesley 7 Nov, F089): the side-by-side table is results/analysis/x1_both_models_2026-11-07.md
  (per cell and arm, both models: the release rise, the second swing, the settled tilt, the last band exit, the dip,
  and the solver switches; measures and conditions stated). It replaces the draft's results_discussion.tex lines
  101-138. In short: on both models the resize at detection leaves the hovering ring more level (4 quadrotors 0.03-0.06
  vs 0.20-0.23 deg; 8 quadrotors 0.80-0.81 vs 0.91-0.93 final, 1.06 vs 1.18 earlier), adds a larger second swing about
  2 s after the release (4 quadrotors 3.47-3.49 vs 2.30-2.45 deg final, 4.19-4.29 vs 3.20-3.27 earlier), and does not
  settle sooner; the square-to-T benefit appears on the earlier masses only (0.56-0.65 vs 2.20-2.21 deg; final 1.64-1.80
  vs 1.78-1.80). The second-swing column is my reading of run.csv (largest absolute tilt from +1 to +4 s), not a scorer
  output.
- Evidence: registry R4200, R4201; results/analysis/x8_hub_noxy_2026-11-07.json (tools/x8_metrics.py);
  results/analysis/x1_both_models_2026-11-07.md. Reviewer done: see F094 (corrections).

### F092 2026-11-07 18:34 FACT: the October simulation results are archived; extract a run before a figure needs it
- To free the laptop's disk (7 Nov) every October simulation day is one compressed file, results/archive/<day>.tar.zst
  (nothing deleted; results/archive/README.md). The final claims are all November runs. But figure and analysis tools
  name October runs (tools/thesis_runsets.py, thesis_rig_twin_figures.py, thesis_rig_figures.py, release_bands.py): a
  figure built from them needs those runs extracted first, e.g.
  tar -I zstd -xf results/archive/2026-10-16.tar.zst -C results 2026-10-16/R1887_sim_gz_ta_a1_r1
  Or put a request in docs/lab_requests.md and the lab chat extracts them.

### F093 2026-11-07 18:40 CORRECTION of F090: X3b's C8 and N9 on one model, after its reviewer
- The reviewer (7 Nov) recomputed from run.csv and the planner logs: hover N1 1.48-1.62 mm (1 iteration) vs 0.97-1.41
  (5), figure-8 path 10.1-10.2 vs 15.2-17.2 mm, N9 hover 1.63-1.79 mm; rt vs it5 differ only in plan_sqp_iters; no src/
  file differs (tools/make_oldlink_worlds.py is changed, not added); C8's bars FROZEN by Wesley 30 Oct ~15:46; R3807's
  final-mass miss kept. C8 CONFIRMED stands. Corrections:
  1. N9 is NOT confirmed: the card confirms it only with N1, L, J AND R9a / R9b. R9b (a Gazebo run) is void twice
     (R3452, R3778) and R9a (tonight's idle bench) has not run on this tree. Read: "N9's lab-PC flight items (N1, L, J)
     hold; N9 is not confirmed (R9a pending, R9b void)". (The registry row R4194's "N9 CONFIRMED" is corrected by R4194c.)
  2. Timing: at eight across the three cells the full-size solve p95 is 60-72 ms against 238-257 ms at five iterations (a
     ratio 0.24-0.29; the chain 70.1-71.7, F068); at nine 85-94 ms. The tick without the per-tick residual is still p95
     102 ms at nine (chains 109-111). These are lab-PC wall times under Gazebo load: not real-time evidence.
  3. "Same results as five" means the same verdicts, not the same numbers: give them (hover 1.48-1.62 vs 0.97-1.41 mm;
     figure-8 10.2 vs 15.3-17.3 mm). The figure-8 cell's only bar is P. The card (section 2) requires "a closed-loop claim
     only, and the thesis says so".
  4. "At nine carriers the ring holds" is bar language: give N1 1.63-1.79 mm, the chain's release peaks 1.5-1.8 deg,
     fold-ins at most 1.95 deg, and 5.98-6.04 mm after the last return.
- Text should now say: one planner iteration per tick gives the same results as five at eight carriers on hover (1.5-1.6
  vs 1.0-1.4 mm), the figure-8 (10 vs 15-17 mm) and the chain (F067), with the solve 3.5-4 times faster (60-72 vs
  238-257 ms on the lab computer under the simulation's load); at nine the ring hovers within 1.6-1.8 mm and the chain's
  releases and returns tilt it at most 1.95 deg. These are closed-loop simulation results only; whether the planner keeps
  real time is not shown in simulation (the laptop bench follows).
- Evidence: registry R4194c; the runs of F090.

### F094 2026-11-07 18:40 CORRECTION of F091: X8's hub with our integral off, and the X1 two-model table, after their reviewer
- X8 hub, integral off (R4200 / R4201): every number recomputes from run.csv; the read-backs differ from R4008 / R4009
  only in int_k_xy (0.0 vs 5.26); no src/ change (the tree also adds X1's configs and changes seven docs/figures scripts,
  not listed in F091; still no flight code). Corrections:
  1. The resize times are swapped: R4200 at +0.426 s, R4201 at +0.408 s.
  2. Say plainly that this arm is WORSE than the integral-on arm: the ring's centre ends 33.2-33.9 mm off its point
     (vs 1.2-1.4); the free ring stays past 80 deg from +12.7-13.0 s right through to LAND and is at 89-102 deg at the
     end (vs 11.0-11.3 s over 80 and 26-66 deg); the centre settles slightly later (5.54-5.61 vs 5.33-5.37 s). L1 and L3
     hold either way (ours 287-288 mm against a bar of 308.3; the port 372-373). Why the integral changes this is not
     shown.
- The X1 two-model table (results/analysis/x1_both_models_2026-11-07.md): all 348 cells match the two scoring files and
  the run logs; the two models fly the same windows and arms (only the world differs). Four statements corrected in the
  file: (1) the band is max(2 sigma before, 0.5 deg), not 1 deg (the order holds either way); (2) the square-to-T's
  smaller dip at detection shows on BOTH models (final 96.6-101.9 vs 106.0-109.3 mm, means -8.4 mm; earlier -9.2 mm), only
  the settled-tilt benefit is the earlier model's; (3) the second swing comes about 2 s after the release with 4
  quadrotors, about 1.2-1.3 s with 8, and at h8 on the earlier masses the rise column holds that swing and a drift; (4) the
  later band exit holds for the resize at detection, not the 2 s-late arm.
- Text should now say (X8 hub): with our sideways integral off the hub cell keeps its verdicts against the port (287-288
  vs 372-373 mm) but its free ring stays tipped past 80 deg to the landing and its centre ends 33-34 mm off, where with the
  integral on it ended within 1.4 mm.
- Evidence: registry R4200c; the runs of F091; the table file.

### F095 2026-11-08 00:34 RESULT (simulation, bench): X3b's R9a, the planner's idle tick at 1 iteration on the ground computer
- What: the planner's whole tick at one SQP iteration (res_every 0), n = 2-10 carriers, no Gazebo, 200 ticks each, on
  the laptop (tuf, the ground computer), idle (load 0.02 at the start); the current tree (src/mpc_planner unchanged since
  bf52a8a). It ran Sun 8 Nov 00:30-00:31 (results/analysis/x3b_bench_res0_2026-11-06.json: the date in the name is the
  script's).

  | Carriers | 4 | 6 | 7 | 8 | 9 | 10 |
  |---|---|---|---|---|---|---|
  | Solve p95 (ms) | 8.4 | 25.3 | 34.9 | 45.7 | 57.7 | 71.6 |
  | Whole tick p95 (ms) | 10.4 | 28.5 | 38.8 | 50.4 | 63.1 | 77.7 |
  | Whole tick max (ms) | 10.5 | 39.2 | 39.8 | 51.4 | 68.0 | 78.9 |

  No failed solve (0 of 200 at every n). Within 2 % of 31 Oct and 3 Nov (tick p95 51.0 at 8, 64.0 at 9, 79.2-79.5 at
  10): same command, same computer, the planner's code unchanged.
- Against the card's bars (frozen by Wesley 30 Oct ~15:46): R8a holds (50.4 ms at 8, bar 100 ms); R9a holds (63.1 ms at 9,
  predicted ~63). The rig projection (3.5 x the idle tick p95 <= 100 ms) gives 6 carriers.
- What it shows and what it does not: the solver alone fits the 100 ms planner period on the ground computer, idle, up to
  10 carriers. It is an open-loop bench: not the closed loop. The closed-loop check at 8 (R8b, 31 Oct) was over the bar
  on the same computer with Gazebo and eight trackers sharing it (tick p95 260.9 ms at RTF 0.088), and at 9 (R9b) it
  could not be flown (void twice: R3452 never lifted, R3778 published no pose). Nothing here was flown on the rig at 8 or 9.
- N9 overall (results as is, F086): its flight items hold on one model (hover N1 1.63-1.79 mm; the chain's L and J;
  N4 6.0 mm reported) and R9a holds; R9b is void, so N9 stays not confirmed as the card writes it. The card's sentence
  "nine carriers at the real-time setting in the twin" needs R9 met and cannot be used.
- Text should say: on the ground computer the planner's tick at one iteration per step takes 50 ms at eight carriers and
  63 ms at nine (idle, no simulation running), inside the 100 ms period; in a closed loop with the simulation on the same
  computer it does not keep up, so real-time operation at eight or nine is not shown.
- Evidence: registry BENCH-1108-x3b; R8a-1031, BENCH-1103-x3b; results/analysis/x3b_bench_res0_2026-11-06.json;
  ~/mdc_overnight/x3b_bench_night2.log. Reviewer done: see F096 (corrections).

### F096 2026-11-08 00:38 CORRECTION of F095: X3b's R9a idle bench, after its reviewer
- The reviewer (8 Nov) recomputed from the bench file: whole-tick p95 50.4 / 63.1 / 77.7 ms at 8 / 9 / 10 carriers (the
  card's quantity: tick_ms, p95, <= 100 ms, no failed solve), solve p95 45.7 / 57.7 ms, 0 failed; start load 0.02; no
  file on the bench's path differs from bf52a8a; the same compiled solvers as 31 Oct / 3 Nov; tuf is the rig's ground
  computer. R8a and R9a hold. Corrections:
  1. "Within 2 % of 31 Oct and 3 Nov" holds at 8 and 9 only (-1.2 %, -1.5 %); at 10 -2.3 %, and at 3-6 the new bench is
     2.4-6.2 % faster. Both earlier benches broke the card's load rule (1.5 and 1.8), so 8 Nov is the first R8a / R9a run
     that meets it.
  2. Not "the planner's code unchanged": the same compiled solver; its Python (reference_builder, geometry, planner_node)
     changed since 3 Nov only by switches that are off by default.
  3. The rig projection (3.5 x the idle tick <= 100 ms) gives 5-6 carriers (99.8 ms at 6; 31 Oct and 3 Nov gave 5): a
     projection from S-CALIB-1022's lower-bound factor (3-carrier rig flights at 5 iterations), for a 1-iteration setting
     the rig has never flown. X3b does not score it.
  4. R8 as frozen is FALSIFIED: the card confirms it only if R8a and R8b both hold, and R8b missed (R3451: tick p95
     260.9 ms, 45.1 % of ticks over 150 ms). R8a holding does not change that.
  5. Nine carriers in a closed loop was never flown (R3452 never lifted, R3778 published no pose): "did not keep up at
     eight; nine could not be flown on that computer".
  6. The text must name the 50 / 63 ms as p95 and add the in-flight caveat: the same laptop in flight ran the solve 3.4
     times slower at p95 (S-CALIB-1022), projecting ~176 ms at 8 and ~221 ms at 9.
- Text should now say: on the idle ground computer one planner iteration per tick takes 50 ms (p95) with eight carriers and
  63 ms with nine, inside the 100 ms period, with no failed solve. This is the solver alone: in the closed loop with the
  simulation on the same computer the tick did not keep up at eight (261 ms p95), nine could not be flown there, and in
  flight the same computer has run the solve about 3.4 times slower. So real time at eight or nine carriers is not shown.
- Evidence: registry BENCH-1108-x3bc; F095.

### F097 2026-11-09 23:58 RESULT (simulation, claim): the newcomer's join (4.3) on Setup 3's 0.86 kg ring (answers Q011)
- What changed (Wesley 9 Nov, Q011 and its addendum): the 7 join cells of chunk 8b re-flew x2 (R4214-R4227) on the
  0.86 kg M2A ring instead of the 1.15 kg ring. Plate worlds made from the M2A worlds with the 1.15 kg worlds' plate object
  (tools/make_m2a_plate_world.py; Wesley's word for the new worlds), his geometry rig_twin_L100_plate, the planner's ring
  as every 0.86 kg claim (radius 0.225, attach and rest 0.02, 0.86 kg, inertia untyped), QUAD4's weld point 0.225 / 0.032,
  and LAND 30 s after the weld (was 10 s) so Figure 15 shows the steady state. One code tree (5f527e4a631f; diff_sha256
  1e3b555824a0): it adds QUAD4's tracker drop-rule fix to the tree of R4094-R4107 (its tracker reads drop_on_loss true;
  no drop was requested in any run). No void; late planner ticks at most 0.11 %, no failed solve.

  | Cell (x2) | Runs | Takeoff to weld (s) | Contacts | Ring tilt after the weld, peak (deg) | Landing | 1.15 kg ring (R4094-R4107) |
  |---|---|---|---|---|---|---|
  | Pickup, drop, re-attach (square) | R4214 / R4215 | 97.3 / 97.7 | 1 / 1 | 1.79 / 2.09 | landed | 96.7 / 99.1 s, 2.46 / 2.35 deg |
  | Square floor join, low route | R4216 / R4217 | 31.5 / 31.5 | 1 / 1 | 2.62 / 2.63 | QUAD4 past 30 deg / TB14 abort | 31.8 / 31.9 s, 2.20 / 2.21 deg, both TB14 |
  | Square floor join, V1 0.30 m | R4218 / R4219 | 31.9 / 31.6 | 1 / 1 | 2.50 / 2.70 | QUAD4 past 30 deg / TB14 abort | 31.9 / 31.7 s, 2.19 / 2.23 deg, both TB14 |
  | Triangle, G + frozen marker | R4220 / R4221 | 20.0 / 20.8 | 1 / 1 | 1.71 / 1.67 | landed | 33.7 / 29.1 s (R4101: 3 contacts), 1.80 / 1.79 deg |
  | Triangle, frozen marker | R4222 / R4223 | 21.4 / 19.9 | 1 / 1 | 1.57 / 1.69 | landed | 20.0 / 21.4 s, 1.76 / 1.70 deg |
  | Triangle, G | R4224 / R4225 | 19.9 / 20.0 | 1 / 1 | 1.60 / 1.66 | landed | 20.0 / 20.0 s, 1.75 / 1.76 deg |
  | Triangle, plain (layout (d)) | R4226 / R4227 | 19.9 / 19.8 | 1 / 1 | 1.68 / 1.62 | landed | 19.9 / 20.1 s, 1.76 / 1.77 deg |

  Takeoff to weld is from his TAKEOFF (tools/attach_clearance.py). Contacts are CONTACT events (no contact was lost in any
  run). The ring tilt is the absolute tilt's peak from the weld to +15 s; it is the same over the weld to +10 s, the window
  the 1.15 kg runs had because they landed at +10 s. The pickup's object-cable clearance is 0.216 / 0.221 m (0.197 / 0.200
  on the 1.15 kg ring).
- Results as is (F086): every join proven on the first contact. On the triangle the join takes 20-21 s from his takeoff
  in every cell, including the G + frozen-marker cell that took 29-34 s with 3 contacts on the 1.15 kg ring. The ring tilts
  at most 1.7 deg after the weld on the triangle and 2.5-2.7 deg on the square (slightly higher than on the 1.15 kg ring,
  2.2 deg). The square floor-join landings are the twin's known TB14 effect (the welded rigid link at touchdown;
  decisions 11 / 22 Oct; card line 199): 2 of 4 abort 6.9-7.3 s after LAND (R4217, R4219) and the other 2 land with
  QUAD4 past 30 deg (47.9-49.3 deg). On the 1.15 kg ring all 4 aborted. Results, not voids.
- Figure 15 (fig:res_join_sim): R4227 replaces R4106. Over the 20 s before LAND its four measured tensions (the figure
  script's measure) are steady: carriers 3.59 / 2.76 / 2.71 N (sd 0.008-0.039 N, drift at most 0.07 N), the new agent
  1.11 N (sd 0.035 N). R4226's are noisier (sd up to 0.10 N, drift up to 0.18 N) and it has 0.108 % late planner ticks,
  against 0 in R4227. Run docs/figures/review_2026-11-05/fig_join_sim.py with MDC_JOIN_RUN=R4227.
- Text should say: on Setup 3's ring the new agent joins the flying ring on its first contact in every case. That takes
  about 20 s from its takeoff on the triangle, 32 s on the square, and 97-98 s after a pickup and drop. The ring tilts at
  most 2.7 deg after the attachment. Thirty seconds after it, the four cable tensions are steady.
- Evidence: registry R4214-R4227; results/analysis/x11_joins_m2a_2026-11-09.json. Reviewer done: see F098 (corrections).

### F098 2026-11-10 00:06 CORRECTION of F097: Q011, the joins on the 0.86 kg ring, after its reviewer
- The reviewer (10 Nov) recomputed from the raw logs: takeoff to weld (triangle 19.8-21.4 s, square 31.5-31.9, pickup
  97.3 / 97.7), one contact and "proven (try 1)" in all 14, the tilt peak after the weld, the pickup clearance
  (0.216 / 0.221 m), TB14 for R4217 / R4219, R4227's tensions: all hold. Each config differs from its original only in
  the ring, QUAD4's weld point and LAND at +30 s; the plate worlds only in the object. The tree adds the drop-rule fix
  (QUAD4's tracker drop_on_loss true); no run requested a drop (the ring never passed 2.7 deg after a weld), so it changed
  no result. Corrections:
  1. The square comparison was the wrong way round. On the 0.86 kg ring the square's 2.50-2.70 deg peak comes 0.01-0.23 s
     after the weld: it is the tilt the ring already had going in (2.1-2.3 deg over the 2 s before); from 1 s after the
     weld the peak is 1.47-1.59 deg. On the 1.15 kg ring the peak came ~6 s after the weld (2.19-2.23). So the fold-in
     tilts the lighter ring LESS, not more. The pickup likewise (2.09 deg at the weld, 1.88 from +1 s).
  2. The landing column is not like for like: both the ring and the hold changed (LAND +30 s instead of +10 s, which
     landed as the blend ended). QUAD4 still passed 30 deg in 4 of 4 square floor-join landings (2 aborted as TB14, 2
     recovered at 48-49 deg); keep F081's sentence on the landing tip-over (a twin effect of the rigid link).
  3. Steady tensions hold for R4227 only: 3.59 / 2.76 / 2.71 N (carriers), 1.11 N (QUAD4), sd 0.008-0.039 N over the 20 s
     before LAND ("drift" = the difference of the two 10 s half-means, at most 0.07 N; end to end up to 0.12 N). In R4226
     QUAD4 falls 1.05 -> 0.70 N in that window (sd 0.10 N). R4227 is the cleaner run for the figure.
  4. The tension is each tracker's applied cable feed-forward from motion capture (0.55 kg x |a_cable|, 0.2 s mean;
     notch 5.3 Hz), the same quantity docs/figures/review_2026-11-05/fig_join_sim.py plots; not a force from Gazebo
     (run.csv's tension columns are empty).
  5. The planner's ring model: on the 0.86 kg ring it flies the untyped inertia 0.02733 / 0.05452 against the world ring's
     0.02227 / 0.04431 (23 % high), as every 0.86 kg claim; on the 1.15 kg ring it matched the world.
  6. R4222 lacks /tracker_3's parameter read-back (its node's own params.json shows the same values).
  7. F097's "29-34 s with 3 contacts" on the 1.15 kg ring fits R4101 only (R4100 had one contact).
- Text should now say (fold-in tilt corrected by F099: at most 1.9 deg overall): on Setup 3's 0.86 kg ring the newcomer joins on its first contact in every cell: 20-21 s from its
  takeoff to the weld on the triangle, 32 s on the square, 97 s after a pickup and drop; the fold-in tilts the ring at
  most 1.6 deg (from 1 s after the weld; the square starts the join already tilted 2.1-2.3 deg). In the simulation the
  welded newcomer can tip over in the landing (the twin's rigid link). Figure 15 from R4227 (steady tensions over the
  20 s before LAND).
- Evidence: registry R4214c; F097; results/analysis/x11_joins_m2a_2026-11-09.json.

### F099 2026-11-10 00:10 CORRECTION of F098 (from the thesis chat, checked by the lab chat): the fold-in tilt on the 0.86 kg ring
- F098's "the fold-in tilts the ring at most 1.6 deg (from 1 s after the weld)" holds for the square floor joins only.
  The absolute tilt peak from weld + 1 s to weld + 15 s, per run (run.csv): square floor joins R4216-R4219 1.51 / 1.47 /
  1.54 / 1.59; triangle R4220-R4227 1.71 / 1.67 / 1.57 / 1.69 / 1.60 / 1.66 / 1.68 / 1.62 (they peak after +1 s, so the
  same as from the weld); pickup R4214 / R4215 1.79 / 1.88. Overall: at most 1.9 deg. Rechecked here for R4214, R4215,
  R4216 and R4220 (1.79, 1.88, 1.51, 1.71). F098's point stands: the lighter ring tilts less than the 1.15 kg ring.
- Text should now say: "at most 1.9 deg" (Table 14's per-row means 1.5 / 1.6 / 1.7 / 1.8; docs/figures/review_2026-11-05/
  tab_averages.py, join()), as the thesis already does.
- Evidence: F097, F098; registry R4214c.

### F100 2026-11-10 23:44 DECISION: the real-world plan for the four sessions left, and the freeze moves to Thu 19 Nov
- Wesley 10 Nov, after the supervisor's message ("results are king": at least a full run once and 3-5 detach and
  re-attach flights; it may take 10 attempts for one success): one list, flown as far as each session gets
  (docs/rig/wed_2026-11-11.md). (1) detach and re-attach: nrtf x5 (hover, plate 9, funnel v4), nr1f x3 (re-attach on
  plate 1), nro x3 (on the circle, the ring stopped for the re-attach); (2) the drop into the circling ring (k1zo x3);
  (3) if pickups keep failing, the held object into the circling ring then the re-attach (fm9hc); then THE FULL MISSION,
  which Wesley defines as on the circle: detach, pickup, drop into the circling ring, the ring stopping by itself for the
  re-attach, four circling on (fm9c; fallback fm9 in hover).
- Dropped: the unannounced releases (rel, relo): the paper's planned "detach performance" rows are now read from the
  planned detaches at the start of every nrtf / nro / fm9c flight. The 1 m baselines stay dropped (6 Nov).
- The thesis's real-world freeze moves from Thu 12 to Thu 19 Nov (Wesley 10 Nov).
- New in the method (thesis text when these fly): the attach pause can start at the re-attach leg instead of at the
  returning quadrotor's takeoff (attach_pause_at 'reattach'); the funnel on the plate; the descent bands with the funnel
  (results/analysis/attach_band_2026-11-10/summary.md: where the descent starts does not predict where the magnet lands;
  the band that pauses the descent is the one that aims, ~2.5 cm at best).
- Evidence: decisions DECIDED 2026-11-10 23:10, 23:27, 23:42; registry RIG-1110-bands.

### F101 2026-11-11 11:23 REAL WORLD (exploratory, lab session 11 Nov): the first triangle re-attaches, the drop into the circling ring, the pickup is the blocker
- Re-attach in hover on the triangle with funnel v4 on plate 9 and the cone bands (MATCH 5 cm / 0.5 s, descent pause 5 cm):
  RIG-1111-nrtfc_2 caught on the first descent, 4.9 mm from the centre, folded in 19.0 s after the release, held through
  the hover and the LAND (ring tilt peak 8.1 deg); RIG-1111-nrtfc_3 caught first try (13.8 mm), folded in 16.7 s after
  the release, then the magnet came off 4 s later (a loose magnet lead, found and fixed by Wesley). On 5 Nov without the
  funnel the triangle caught 0 of 19 descents. Counted strictly: 1 clean detach-and-re-attach, 1 caught and lost.
- The drop into the CIRCLING ring works: RIG-1111-k1zo (his settle drop over the moving drop point, release 14 cm over the
  drop point) put the object in the net 3-8 cm from the centre, but the impact tipped the ring off a carrier's magnet and
  the automatic drop released it; with the release lowered to 3 cm (RIG-1111-fm9hc_8) the object went into the net 3 cm from
  the centre with the ring at 7.8 deg peak and no detachment.
- The full mission did not complete: every pickup that arrived within ~2 cm of the object lifted at once (4 of 4); every
  one 4.7-9.7 cm off failed (7 of 7, fm9c / fm9c_3 / fm9c_4), with the circle running or at rest.
- Not yet for the text: all exploratory; the numbers are in the registry rows RIG-1111-*.

### F102 2026-11-11 16:09 CORRECTION of F101 and the paper's real-world tables re-run with 11 Nov (exploratory)
- F101 undercounted: the hover re-attach on the triangle with a funnel and the 5 cm bands caught on the FIRST descent in
  3 of 3 flights (contact 4.9, 13.8, 7.9 mm from the plate centre; release to fold-in 19.0, 16.7, 21.9 s): nrtfc_2
  (funnel v4) held to the landing, nrtfc_3 (v4) let go 3.8 s after the fold-in with no command (a loose magnet lead,
  Wesley), nrtfc_5 (the supervisor's 105 mm funnel) held. Counted as "proven, folded in and still on its plate until LAND
  or 15 s": 2 of 3. Before the funnel (4-5 Nov): 0 of 5 flights, 6 contacts in 23 descents, median miss 6.4 cm.
- The funnel per flight on 11 Nov: v4 for nrtfc_2, nrtfc_3, k1zo, k1zo_3; the supervisor's from nrtfc_5 on (every
  fm9hc and fm9c flight). Registry RIG-1111-funnel_c.
- The paper's tables (results/analysis/paper_rig_2026-11/tables.md, .tex) now hold the 11 Nov flights; the attach
  success rule gained "the magnet marker never 2 cm from where it sat on the plate" (paper_rig.py), which is what makes
  nrtfc_3 a miss. Planned detaches: 11 of 11 static, 5 of 5 moving (tilt before 4.3 deg, swing 6.0 (3.9-11.6) deg static).
- Sentences in the combined paper (main.tex) that no longer match the data, for the thesis chat and Wesley:
  "the re-attachment onto the triangle never caught" and tab:descents (add the funnel row: 3 descents, 3 contacts, 3
  proven, 2 held); the abstract's and conclusion's unannounced detection (0.42 s and 0.44 s both appear; the flights are
  8 Oct on 0.55 m cables while the text says 1 m); "In six flights" and "2.9-5.2 deg" for the announced detach (2.9-5.2
  is the tilt before, not the swing; the table now has 11 static and 5 moving); the parameters table's detach margin
  0.06 m and drop rule 30 deg (every 1 m attach flight since 5 Nov flew 0.15 m and 55 deg); the hardware section has
  no funnel; fig "Attach single run" still draws the square flight sq8_2 (nrtfc_2 or nrtfc_5 are the triangle + funnel
  candidates); "6 of 15" pickups (11 Nov adds 4 of 11: all 4 that arrived within ~2 cm lifted, all 7 at 4.7-9.7 cm failed).
- No full mission yet (0 of 6 reached the re-attach on 11 Nov). Thu 12 Nov's order: hover re-attach to 5 clean, then the
  full mission in hover, then the circle (decisions DECIDED 2026-11-11 15:56).
- Evidence: registry RIG-1111-nrtfc_5, -nrtfc_3c, -funnel_c, -fm9c_6c; results/analysis/paper_rig_2026-11/events.csv.

### F103 2026-11-12 09:12 REAL WORLD: the first complete full mission (RIG-1112-fm9_5)
- Hover at 1.2 m, triangle on plates 3/11/7, QUAD4 welded on plate 9 (supervisor's funnel on plate 9 and on the small
  object): detach, release, Te-Jen's pickup (3rd descent; the first attempt flew with body 6 mis-defined, Wesley),
  lift 1.58 m, settle drop into the net (the object at rest 9.2 cm from the ring centre, 5 cm under its plane, kept to
  the landing), his re-attach on plate 9 (contact 15.0 mm, proof failed once, re-contact 17.7 mm, proof, folded in
  16.0 s after the drop), four carried the ring, LAND, every quad down. Ring tilt peak 10.1 deg.
- This is the supervisor's "full run once" for the combined paper. Exploratory row (no card); the paper's tables and
  figures follow once the session ends. Evidence: registry RIG-1112-fm9_5; bag results/rig/2026-11-12/fm9_5/.

### F104 2026-11-12 11:12 REAL WORLD: the 12 Nov session closed; the drop into the CIRCLING ring 3 of 3 with the re-attach
- fm9hc_2, _4, _5 (QUAD4 starts on its mat holding the object; ring at 1.2 m on the 0.3 m circle at 0.125 m/s): every
  flight dropped the object into the net with the ring moving at 0.13-0.14 m/s (object at rest 8.7-11.1 cm from the
  ring centre, 5-6 cm under its plane, kept to the landing); the circle then slowed to rest by itself, his re-attach on
  plate 9 was confirmed 35-61 s after the drop (1st, 2nd and 4th descent), and four carried the ring on the circle to
  LAND. 3 of 3 complete. This is the moving-ring half of the mission; the pickup on the circle did not succeed today.
- The pickup: hover 1 of 2 flights (fm9_5, F103); circle 0 of 2 (fm9c_3: his attach manager's 12 cm radius latched a
  LOST before the magnet seated; fm9c_5 with 4 cm: attached at 1.5 cm but no grip, the object never rose).
- The re-attach with the ring still moving (nro, no pause): 0 of 1, ~14 descents in 96 s, one contact whose proof failed.
  As in simulation on 1 m cables (0 of 12).
- Exploratory rows, no card, no bars. Evidence: registry RIG-1112-fm9hc_2/_4/_5, -fm9c, -fm9c_3, -fm9c_5, -nro,
  -launches; bags results/rig/2026-11-12/.

### F105 2026-11-12 23:52 DECISION + SIMULATION: the supervisor's experiment plan; Te-Jen's C++ planner works in the twin
- **The supervisor's plan for the combined paper** (decisions DECIDED 2026-11-12 22:52), replacing the 9-trial plan (three
  starts per case: "the exact same case", no repeatability). 1-3 identical runs per case:
  (1) Te-Jen's drop-off: start -> ring -> start, his planner both ways, the ring moving, no pickup, no re-attach; hover,
      circle, figure-eight. Metrics: path, distance to the other drones, drop position from the ring centre, time.
  (2) Wesley's detach/re-attach: detach while moving (hover, circle, figure-eight), the ring hovers for the re-attach.
      Metrics: time, re-attach accuracy, ring tilt, QUAD4's path from 3 to 4.
  (3) 4 -> 2 -> 4 (next week, a limit test).
  (4) The hero for the video: detach, pickup (object raised on a stand), drop, re-attach.
  Current draw per drone with 4 and 3 carrying: next week.
- **Wording that changes:** every real-world drop-off so far (fm9h, fm9hc_2/_4/_5) flew "the direct Python transfer,
  obstacle avoidance bypassed" (Te-Jen); none used his obstacle-avoiding C++ planner. The paper must not describe them
  as validating his planner.
- **His C++ planner in simulation (exploratory, judged after the runs; one run per case):**
  - Over a hovering ring it stalled 0.5-0.67 m short in 15 of 15 runs. The cause: his default collision shapes
    (`swing_envelope_shape legacy_square`) box each carrier's slanted cable, and over the ring the boxes overlap the drop
    point. His convex shapes (`outer_octagon`) fix it: hover 4 of 4 full missions (R4383-R4385, R4392).
  - On a moving ring, his rendezvous-time fixed point needed 30 iterations instead of 6 (a parameter). Full missions on
    the circle from 1 and 5 o'clock (R4389, R4390) and on the figure-eight from 9 o'clock (R4391). The circle from 9
    o'clock stalled (R4388), and a carrier's cable came within 1 cm (centre lines) of the waiting QUAD4.
  - He climbs to 3.2-3.6 m on a moving ring; the rig caps QUAD4 at 2.8 m planned.
  - Evidence: registry R4374-R4392 (R4383c corrects the baseline count to 0 of 15), decisions FINDING 2026-11-12 22:26.

### F106 2026-11-13 03:42 FACT CHANGED + SIMULATION: Te-Jen's C++ planner on a moving ring does not hold at real time
- **What changes for the text:** F105's simulation results for his C++ planner on a MOVING ring (full missions on the
  circle from 1 and 5 o'clock, R4389/R4390; the figure-eight from 9 o'clock, R4391; 3.2-3.6 m) were flown by a twin
  running at 0.13-0.16 of real time. In real time his planner discards almost every plan (the collision world changes
  ~214 times a second and his final check near the cables takes 60-90 ms), so those results must not be described as what
  the planner does in a real flight. They are slow-time results.
- **Real-time replays of 12 Nov's real flights** (open loop: the real ring and carriers, a virtual QUAD4 that tracks his
  reference perfectly; exploratory, one replay per row, judged after the runs; evidence results/analysis/
  cpp_realtime_2026-11-13/README.md):
  - Over a STILL ring, with two timing settings changed (his world refresh coalesced to 0.2 s, his splice lookahead
    0.20 s) and his convex collision shapes: he reached the drop point 8 of 8, at most 2.9 m, also under heavy CPU load.
  - Over a MOVING ring (circling at 0.125 m/s): with the same settings he reached it in 26-41 s at spline 4, but he has no
    escape when no plan is found: he stops, and a stopped quadrotor beside a moving ring is swept by the carriers' cables
    (a carrier cable crossed QUAD4's cable, centre lines 0.00 m, in 2 of 3 replays at spline 3). Not flown on the rig.
- **The twin's paused drop-offs of 13 Nov** (R4423-R4434) tested the drop-off and our three LAND aborts over a still ring
  only (his TAKEOFF came before the path started, R4423c); the stop of a running figure-eight is being re-flown (batch 9).
- **For the rig tomorrow** (Wesley to confirm): his C++ planner only over a ring paused at his TAKEOFF; the moving drop-off
  on the direct route (his Python transfer, obstacle avoidance bypassed), as on 12 Nov.

### F107 2026-11-13 04:44 SIMULATION: the stopped-ring drop-off in the rig's order (batch 9), and the moving ring once more
- Exploratory, one run per row, judged after the runs, twin at 0.11-0.14 of real time. Settings: his convex collision
  shapes, swing bound 5 deg, spline 4, carrier padding 0.06 m, 30 rendezvous-time steps, world refresh 0.2 s, splice
  0.20 s, height cap 2.9 m; return on his Python transfer.
- Figure-eight running at 0.12 m/s, then his TAKEOFF (as the rig will fly it): the ring at rest 3 s later; his C++
  transit started 11 s after his TAKEOFF and took 18-20 s; QUAD4 at most 2.90 m; at least 0.33 m from the carriers'
  cables in his transit; object in the net; back at the start; the ring still until he had landed (R4435, R4436). Circle
  from 9 and 5 o'clock: the same (R4439, R4437; 0.30 and 0.29 m).
- Failures: one stall 0.5 m short over the still circle from 5 o'clock (R4433); one QUAD4 crash in its loaded climb from
  a pendulum swing of the twin's rigid cable, before his C++ part (R4438); his C++ into the MOVING figure-eight never
  reached the drop and passed 0.04 m from a carrier's cable (R4440).
- Our LAND in his transit, at the drop, on the way back: everyone landed, the ring still (R4429-R4431, over a ring that
  had not started its path, R4423c).
- For the text: his planner's drop-off is a STOPPED-ring result; the moving drop-off is the direct route (his Python
  transfer, obstacle avoidance bypassed).

### F108 2026-11-13 05:33 SIMULATION: our LAND during the stopped-ring drop-off, in the rig's order
- Completes F107: with the figure-eight running at 0.12 m/s and then his TAKEOFF, our LAND in his C++ transit (twice)
  and in his return (twice; the second LAND was sent at the drop zone but he had already entered the return): 4 of 4, QUAD4 taken back at once, the ring held still, everyone landed, no fleet
  abort (R4441-R4444; exploratory, one run each, twin at ~0.13 of real time). One QUAD4 toppled at its own touchdown on the
  twin's rigid cable (R4441), a simulation artefact.
- Interface limit (decisions OPEN 05:16): from the pause request his planner predicts the ring already stopped while it
  slows for 3 s; the drop-off relies on his climb keeping his C++ part ~8 s later. Do not describe the pause as a
  verified hand-off condition.

### F109 2026-11-13 17:06 ANSWER Q010: the harder detachment's tilt while away, and sq8_2's re-attachment time
- Section: 4.2.1 (tab:res_both, layout (e)) and 4.2.3 (the first real re-attachment).
- Nothing changes in the thesis's numbers. An independent recomputation gives the same values:
  - the real flight read from its bag, the simulated runs from their pose quaternions;
  - tilt is the angle of the ring's body z axis from vertical;
  - the window runs from the magnet off to the re-attachment: the real flight's FLEET RESIZED n=4, the simulated runs' RELEASED to REWELD. The two definitions match within 0.06 s.

  | Run | Window (s) | Mean tilt (deg) | Median (deg) |
  |---|---|---|---|
  | RIG-1105-sq8_2 (real) | 21.5 | 15.6 | 13.2 |
  | R3708 / R3712 / R3714 / R3715 | 19.3-19.4 | 9.13 / 11.93 / 11.55 / 12.33 | 8.49 / 11.43 / 11.19 / 12.02 |
  | Mean of the four simulated runs | | 11.2 | |

- The real re-attachment came 14.4 s after the partner release (14.37 s) and 21.5 s after the magnet off (21.53 s): both stand.
  - Counted from the radio switch to his controller, 0.14 s after the release, it was 14.2 s.
  - The simulated runs took 12.3-12.4 s from the release, each with a single descent.
  - The real one needed two descents: the returning quadrotor climbed back from the first.
- Small correction: the real median is 13.2 deg, not 13.3 (13.247 by tab_averages.py). This matters only if the median is quoted.
- The window's ends matter on the real flight, because its tilt rose during the returning quadrotor's descents:
  - 6-11 deg in the first 10 s, 21-30 deg in the last 10 s;
  - moving either end by 1 s changes the mean by 0.4-0.6 deg;
  - ending at the first contact gives 14.1 deg;
  - over reasonable windows the real mean is 14-17 deg;
  - the simulated means move by at most 0.5 deg over the same windows.
- The real ring leans more than the simulated one under every window: from the partner release, 19.9 deg against 8.4-13.1.
- Text: keep 15.6 against 11.2 deg, and 14.4 s. If the text says the real tilt rose from about 7 to 29 deg, say it rose
  during the returning quadrotor's descents over the free plate, not steadily. The cause is not established.
- The four simulated run folders moved to the lab PC on 13 Nov (drones:~/mdc_archive/results/2026-11-05/), so
  tab_averages.py's tilt_away_sim() no longer runs on the laptop.
- Evidence: results/analysis/q010_tilt_while_away/ (q010_tilt.py, tilt_while_away.csv, window_sensitivity.csv,
  tilt_profile_1s.csv); RIG-1105-sq8_2, R3708, R3712, R3714, R3715.

### F110 2026-11-13 17:06 ANSWER Q009: X1's second swing in layout (e), read from the end of the first swing
- Section: 4.1.2 Losing Quadrotors in Simulation (tab:res_x1), the harder detachment of layout (e) (sqTnoxy,
  R3893-R3902, final masses).
- What was checked: the reading was recomputed independently from run.csv, with the tilt taken from the ring's
  quaternion.
  - t = 0 is the RELEASE event.
  - The first swing ends at its first minimum after the peak in [0, 1.5] s (the lowest sample within +-0.1 s).
  - The second swing is the largest tilt from there to +4.0 s.
  - The run folders are on the lab PC (drones:~/mdc_archive/results/2026-11-06/). The run.csv checksums match.
- The thesis numbers stand. All 10 runs and all 5 means reproduce (largest difference 0.005 deg):

  | Arm | Runs | First swing ends (s) | Tilt there (deg) | Second swing (deg) | at (s) | Mean (deg) |
  |---|---|---|---|---|---|---|
  | Baseline | R3895 / R3896 | 1.39 / 1.40 | 0.19 / 0.14 | 9.45 / 10.34 | 2.63 / 2.56 | 9.9 |
  | Proposed | R3893 / R3894 | 1.50 / 1.55 | 12.94 / 12.47 | 23.44 / 21.27 | 2.16 / 2.19 | 22.4 |
  | 0.5 s late | R3897 / R3898 | 1.46 / 1.58 | 0.13 / 0.08 | 14.60 / 14.11 | 2.70 / 2.56 | 14.4 |
  | 1 s late | R3899 / R3900 | 1.40 / 1.40 | 0.02 / 0.43 | 15.68 / 15.13 | 2.77 / 2.74 | 15.4 |
  | 2 s late | R3901 / R3902 | 1.41 / 1.42 | 0.30 / 0.34 | 10.31 / 10.50 | 2.73 / 2.70 | 10.4 |

  - The first swing ends at +1.39-1.58 s, so "+1.4-1.6 s" holds.
  - The second swing peaks at +2.16-2.19 s with the resize at detection and +2.56-2.63 s without. The comment's
    +2.2 / +2.6 s holds.
  - Ending the window 4 s after detection (+4.30-4.32 s) changes no value.
- The h4 and f4 claim holds: the reading equals F091's +1 to +4 s window in 20 of 20 runs.
  - Their first swing ends at +0.74-0.81 s.
  - The table's h4 and f4 second-swing column reproduces: 2.4/3.5/3.6/2.9/2.4 and 3.4/3.4/3.6/3.5/3.3.
- Two notes for the text:
  1. The "(35-44 deg at +1.0 s)" in the comment at results_discussion.tex line 138 covers both models. On these runs the
     tilt at +1.0 s is 41.3-44.1 deg. 35-42 deg is the earlier masses (R4180-R4189). Write 41-44 deg.
  2. With the resize at detection the ring does not return to level after the first swing. It comes back only to
     12.5-12.9 deg, on the same side, then tips further to 21-23 deg.
     - In the other four arms it passes through level, leans up to 8 deg the other way, and swings back.
     - Measured from the end of the first swing, the second swing rises 8.8-10.5 deg with the resize and 9.3-10.2 deg
       for the baseline.
     - So 22.4 against 9.9 is how far the ring tips again, not a larger oscillation. The text "added a larger second
       swing" is right as the largest tilt. It should not say the swing doubles.
- Text could say: in layout (e), with resizing at detection the ring did not return to level after the first swing
  (12.5-12.9 deg at its lowest). It tipped again to 22.4 deg on average, against 9.9 deg with the baseline.
- Side notes, not Q009:
  - h4's "2 s late no larger than the baseline" is 2.40 against 2.38 deg on the means, so "about the same" is exact.
  - On the figure-eight, three arms' column values fall at +3.4-3.6 s, not at the +2.2 s swing, and the baseline's tilt
    is still rising at +4 s. The arms differ by at most 0.3 deg.
- Evidence: results/analysis/q009_x1_second_swing/ (second_swing.py, per_run.csv: 30 runs with the end of the first
  swing, the signed tilt, the F091 window, the tilt at +1.0 s and each run.csv's sha256). An analysis check, no new
  runs and no registry rows. Not reviewed. Closes Q009.

### F111 2026-11-13 17:15 RESULT (real world): Te-Jen's C++ planner flies the drop-off into the MOVING ring; the hero on the circle
- The combined paper's drop-off (decisions DECIDED 2026-11-12 22:52, case 1). His obstacle-avoiding C++ planner flew QUAD4 in to the moving
  ring. His Python transfer did the settle, the descent and the way back to the start. Word it as **"C++ approach, Python return"**: his
  C++ return over a moving ring never starts (the hand-over needs the quadrotor still).
- 14 flights used his C++ planner: 7 PASS, 1 PARTIAL, 5 FAIL, 1 ABORT before any hand-over. The settings were fixed at 10:40 after
  six changes made at the rig (below). On those final settings: **4 of 4**. Exploratory, one or two runs per case, judged after the
  flights, no bars.

  | Flight | Ring | C++ transit (s) | Ring speed (m/s) | Closest, C++ transit: cable / object (m) | Closest, Python descent: cable / object (m) | Object from ring centre (m) | Return to start (s) |
  |---|---|---|---|---|---|---|---|
  | fm9h8_13 | figure-eight | 6.4 | 0.123 | 0.43 / 0.37 | 0.16 / 0.11 | 0.076 | 6.5 |
  | fm9hc_10 | circle | 5.3 | 0.121 | 0.43 / 0.37 | 0.23 / 0.18 | 0.063 | 7.3 |
  | fm9hc_11 | circle | 4.0 | 0.124 | 0.50 / 0.43 | 0.14 / 0.09 | 0.015 | 8.5 |
  | fm9h_2 | hover | 11.5 | 0.027 | 0.38 / 0.31 | 0.16 / 0.10 | 0.094 | 6.7 |

  - Clearances are centre-line distances from QUAD4's cable (and from the object) to the nearest carrier cable, with no radii.
  - "Object from ring centre" is the horizontal distance 3 s after the release. The object lay 0.05-0.06 m below the ring marker's
    plane: in the net, read from motion capture.
  - The hover flew his swing 5 deg / spline 4; the moving ring flew 3 deg / 2.
- Also passed: fm9h8_5 and fm9hc_8 (before the freeze), and fm9h8_14 (figure-eight at the old 2.75 m goal).
- Captured against failed transits:
  - every capture kept QUAD4's cable 0.21-0.50 m from every carrier cable;
  - every failure came to 0.00-0.09 m; four of the five installed 32-63 emergency hovers (UNCERTIFIED_HARD_HOVER) beside
    the moving ring, and the fifth (fm9h8_3) had its planner disabled by stale inputs;
  - the final settings installed none;
  - in every drop the closest approach came in the Python descent, not in the C++ transit.
- The six changes at the rig (decisions 13 Nov), each from a failed flight:
  - carrier-state timeout 1.0 s (stale inputs disabled his planner);
  - plan lead 0.30 s (late plans);
  - search limit 0.10 s;
  - tracking-error brakes OFF (the supervisor);
  - the emergency hover held at its first point (it had ratcheted QUAD4 down toward the cables);
  - goal over the drop point at 3.0 m.
- On the final settings QUAD4 flew to 3.04-3.08 m.
- **The hero on the circle (fm9c_2):**
  - detach; pickup from the stand on the 3rd try; the drop into the circling ring, in the net 0.04 m from its centre;
  - the re-attach missed: our contact check fired with the magnet 0.8 cm above plate 9, his lift proof started before QUAD4
    reached its reference, and the magnet never touched;
  - a fix is open (decisions OPEN 13 Nov). Not a full mission.
- No detach-and-re-attach flight today. The paper's real re-attachments are still RIG-1111-nrtfc_2 and -nrtfc_5.
- Text: the first real-world drop-offs by his obstacle-avoiding planner, into a moving ring. Earlier drop-offs (F105) used the direct
  Python transfer and must not be cited as his planner.
- Evidence: registry RIG-1113-* (with corrections RIG-1113-*c); results/analysis/cpp_rig_2026-11-13/ (table.md, metrics.csv,
  figures: top view + clearance per final-settings flight, summary_clearance_transit); results/analysis/hero_2026-11-13/. Not reviewed.

### F112 2026-11-13 18:56 ANSWER Q012: the X11 U ring paths for Figure 18, as one analysis file
- File: `results/analysis/x11_u_paths_2026-11-07.csv.gz` (1.2 MB). It has columns run, t, payload_x, payload_y,
  payload_ref_x, payload_ref_y:
  - one row per logged sample of each run's logs/run.csv, deduplicated on t;
  - from its PATH_START to its LOGGED event (events.csv sim_time);
  - all 60 runs R4034-R4093 (the list in x11_u_2026-11-07.json), 97,157 rows.
- Source: the run folders in the lab PC archive (drones:~/mdc_archive/results/2026-11-07/), only run.csv and events.csv
  fetched. The window starts match the json's t_path (R4034: 100.986 s).
- No thesis number changes. Note for the script: x11_u_2026-11-07.json's path_err_m is the clock-free distance to the
  path (R4034 0.0133 m). A time-matched xy RMS of the same rows is larger (R4034 0.0183 m). Use the same method as
  before when printing the RMS per case.
- Nothing new flown or scored.

### F113 2026-11-13 19:38 ANSWER Q013 item 3: the drop into the net as a disturbance of the ring (NOT REVIEWED yet)
- Table: `results/analysis/rig_results_2026-11-13/drop_disturbance.csv`, one row per drop on Setup 3: 18 drops on
  5, 11, 12 and 13 Nov, plus k1zo_3 listed with no release. Judged after the flights, no bars.
- Window: the release (his DROP_OBJECT) to +10 s, cut at QUAD4's next descent, LAND, a loss or a resize. Baseline: the
  5 s before. Rules: drops.py docstring.
- 13 drops ended in the net, all on 11-13 Nov, released 12-19 cm above the ring's plane:

  | | before | after |
  |---|---|---|
  | tilt (deg), median (range) | max 7.7 (6.1-9.3) | peak 6.6 (5.0-13.9) |
  | dip (mm) | 15 (8-22) | 23 (13-45) |
  | horizontal error (cm) | max 4.6 (1.6-7.1) | peak 5.1 (3.2-9.1) |

- Object in the net: 8.4 cm from the ring centre (1.5-11.9). Ring speed at the release: 0.10 m/s (0.01-0.14).
- Recovered (tilt, height and horizontal error back in their own pre-release bands, held 1 s) in 11 of 13; median 1.5 s
  (0.3-9.9), with height usually last. The other two were 12 Nov circles where the ring drifted while the orbit slowed
  for the re-attach.
- The bands are wide because the ring already moves 6-9 deg with QUAD4 hovering over it. "Recovered" therefore means
  back inside its own pre-release motion.
- Two releases from 0.53 m and 0.23 m (k1ob_2, 5 Nov; k1zo, 11 Nov) pulled carrier magnets off the plates, and the ring
  dropped automatically within 0.7 s. None of the 14 drops since the release was lowered to 0.03 m lost the ring.
- fm9hc (13 Nov, C++) missed the net: the object hit the ring edge 24 cm out and fell, giving a 15.9 deg tilt peak.
- Mixed in this table: 5 Nov drops on arrival from ~0.5 m (object pose lost in the net); 13 Nov drops by his C++
  approach. Credit the drop-off to his controller and the ring's response to our planner (Q013's framing).
- A reviewer is running; a follow-up entry will say what it confirms or corrects.

### F114 2026-11-13 19:38 ANSWER Q013 item 3 (pickups): pickup counts per flight (NOT REVIEWED yet)
- Table: `results/analysis/rig_results_2026-11-13/pickups.csv`, 23 Setup 3 flights with a pickup descent (from
  pickup_seat_2026-11-13/tries.csv).

  | | flights | tries | lifts started | held |
  |---|---|---|---|---|
  | carriers flying | 19 | 40 | 19 | 13 |
  | one quad (ring on the floor) | 4 | 5 | 5 | 4 |

- With the carriers flying, by object (tries / lifts / held):
  - big: 11 / 7 / 7 (one fell 7.5 s into the lift, fm9c_6);
  - bare small: 15 / 5 / 4;
  - small with the funnel: 14 / 7 / 2.
- The cause of the failed lifts (13 Nov analysis, F111 context): the magnet tilted on its edge on the object; its tilt
  sorts 26 of the 27 lifts. QUAD4's body sat 3-5 cm above its reference at the bottom, so the cable never went slack.

### F115 2026-11-13 19:38 ANSWER Q013 item 4: the full mission fm9_5, and the moving-ring half of fm9hc_2/_4/_5 (NOT REVIEWED yet)
- Tables: `results/analysis/rig_results_2026-11-13/full_mission_fm9_5.csv` and `moving_half_fm9hc.csv` (one row
  per phase; the log lines used are in `mission_events.csv`).
- fm9_5, in s after TAKEOFF:
  - the detach starts in the lift: unload at +17.8, our release at +20.9;
  - QUAD4 handed to his planner at +27.1;
  - pickup tries at +38.3, +66.3 and +92.4 (the third attached);
  - lift +109.8, drop +154.5;
  - contact +163.5, failed proof, re-contact +167.4, proven +170.9;
  - fold-in +171.0 (back to four), LAND +186.7, disarmed +199.7.
- Our planner flew four (QUAD4 welded) up to the release, three until the fold-in, then four.
- fm9hc_2/_4/_5:
  - drop into the circling ring at 0.10-0.12 m/s; the orbit slows to rest over 3 s, starting 0.9 s after the drop;
  - the re-attach took 4 / 1 / 2 descents (fm9hc_5 had one failed proof);
  - drop to fold-in 60.9 / 34.6 / 39.8 s; the circle resumed on four.
- fm9_5's whole flight:
  - run.csv: the ring and the four quads, 50 Hz, ARM to DISARM;
  - new: `results/analysis/rig_results_2026-11-13/fm9_5_full/`, the tip, object, QUAD4, his reference and the flags
    from the whole bag;
  - the 12 Nov `fm9_5_poses.csv` stops 3.8 s after the drop.

### F116 2026-11-13 19:51 REVIEW of F113-F115 (Q013 items 3-4): SUPPORTED WITH CORRECTIONS
- Method: the reviewer recomputed from the bags with its own reader, sharing no code with drops.py or paper_rig.
- **F113 (drops):** every tilt, dip, horizontal error, object position, speed and recovery matches within ~2%, and so do
  the medians and counts. No drop is missing or double-counted (all 19 Setup 3 logs with DROP_OBJECT). Corrections:
  - k1zo's automatic drop came at +0.72 s (k1ob_2 +0.64 s): write "within 0.75 s", not 0.7.
  - fm9h8_13 is labelled figure-eight, but the ring was AT REST at its release (lap end; reference 0.003 m/s). So 2 of
    the 3 figure-eight drops were into a moving ring.
  - fm9h8_13's baseline is only 1.43 s, which inflates its dip. With a 5 s baseline it is 35 mm, and the dip range top
    becomes 36 mm (fm9h8_14), not 45.
  - Window bias: "after" runs 10 s and "before" 5 s. The 12 Nov circles start slowing to rest 0.9 s after the drop, and
    several peaks come late (+7-10 s) from that slowing, not from the drop. With 0-5 s for "after":

    | | median (range), 0-5 s after |
    |---|---|
    | horizontal error | 4.2 cm (2.7-8.3), BELOW the before median 4.6 |
    | dip | 21 mm (10-36) |
    | tilt | unchanged, 6.6 deg |

  - In fm9_5 and fm9hc_4 the swing peak comes before the object leaves the magnet.
  - Height decides "recovered" in most flights: the tilt band is already met from impact in 7 of 13.
- **F114 (pickups):** every count is right. Fix the cause paragraph:
  - "26 of 27" includes three 14 Oct lifts outside this table. On Setup 3's 24 lifts the tilt sorts 23 of 24 (k1d held
    at 5.6 deg; fm9c_5 try 1 failed at 5.35).
  - QUAD4's body sat 0.5-4.1 cm (median 3.2) above its reference at the seat, not 3-5 cm.
  - The cable stayed taut in 23 of 24 lifts, not "never" slack: k1ob_2 had 7.7 mm of slack.
  - The tilt is a strong correlate (AUC 0.99), not a tested cause.
  - Some failures are setup issues, not the method: k1o_2 (body 6 redefined), 12 Nov fm9c_3 (his 12 cm attach latch),
    fm9_5 try 1 (body 6 redefined 25 mm lower).
- **F115 (full mission):** every fm9_5 time matches the log. Corrections:
  - Our magnet OFF reached the coil at +20.99 (+20.9 is the fleet resize).
  - The +199.7 disarm was the operator's DISARM.
  - fm9hc_5's descents were 4 / 1 / 3 (three DESCEND_TO_ATTACHMENT), not 4 / 1 / 2.
  - Ring speed at the drop: the registry's 0.128 / 0.140 / 0.134 m/s against this table's 1 s chord 0.099 / 0.119 /
    0.117. Use one definition in the thesis.
- **The thesis must NOT claim:**
  - that the drop raised the ring's tilt (after <= before max in 10 of 13) or its horizontal error (lower than before
    at the median with equal windows);
  - a drop success rate "13 of 18" (k1d and fm9y_2: the object's pose was lost; k1ob_2 and k1zo: the object was in the
    net when the ring fell);
  - that the lower release alone ended the ring losses (both losses were on the circle; k1ob_2 carried the 0.15 kg
    object; the 0.5 m hover releases of the 60 g object kept the ring);
  - three drops into the moving figure-eight;
  - a per-try pickup success rate as a property of the method.
- Give the date with fm9hc_8 (11 and 13 Nov) and fm9c_3 (11 and 12 Nov): the names repeat.
- With these corrections F113-F115 are reviewed and may be cited.

### F117 2026-11-13 19:53 ADDITION to F113/F116: the equal 0-5 s window, per flight (Q013 item 3, the thesis chat's follow-up)
- `results/analysis/rig_results_2026-11-13/drop_disturbance.csv` gains five columns: window_0to5_s,
  tilt_peak_0to5_deg, dip_0to5_mm, xy_err_peak_0to5_cm and recover_0to5.
  - The window is the release to +5 s, with the same cuts as before (QUAD4's next descent, LAND, a loss, a resize).
  - recover_0to5 is "yes" when the three signals were back in their pre-release bands and held 1 s by +5 s.
  - No existing column changed: the table was recomputed and compared.
- Over the 13 drops in the net:
  - medians tilt 6.6 deg, dip 21 mm, horizontal error 4.2 cm (the reviewer's numbers in F116);
  - recovered within 5 s in 8 of 13.
- Short windows: fm9_5 3.57 s and fm9c_2 3.37 s (cut by QUAD4's re-attach descent).
- fm9h8_13's dip 45 mm stands on a 1.43 s baseline (lap end). F116: 35 mm on a 5 s baseline. Footnote it, or drop it
  from a mean.
- Ring speed: the reviewer has no preference; it asks for one definition. ring_speed_mps (the measured 1 s chord before
  the release) is fine. Say so, and note that the commanded path speed is in ref_speed_mps (0.114-0.127 m/s on the
  moving flights); the chord swings about +-0.03 m/s around it.

### F118 2026-11-13 19:58 ADDITION (Q013 addendum): the ring's response around every drop, aligned at the release
- File: `results/analysis/rig_results_2026-11-13/drop_series.csv.gz` (script drop_series.py), 16 drops x 750 rows.
  - Columns: flight, day, t_from_release_s, ring_tilt_deg, ring_dz_mm (height minus the 5 s pre-release mean),
    xy_err_cm (drops.py's clock-free distance to the reference's path), ref_speed_mps.
  - The drops: the 13 in the net, 13 Nov fm9hc (missed), k1ob_2 and k1zo (ring losses).
  - The grid: -5 to +10 s around his DROP_OBJECT, on drops.py's 50 Hz grid. It has the same release time, so it
    aligns with drop_disturbance.csv without TAKEOFF.
- Checked against the table: the peaks over each row's window equal tilt_peak_deg, dip_mm and xy_err_peak_cm
  (fm9hc_11, fm9h_2, fm9hc_4, exactly).
- Not cut at QUAD4's next descent: fm9_5, fm9c_2 and fm9hc_5 include it from +3.4-5.9 s (q4_next_descent_s in the
  table). k1ob_2 and k1zo show the ring falling from +0.64 / +0.72 s.
- Two flights share a name: use day with flight (fm9hc_8: 11 and 13 Nov).

### F119 2026-11-13 20:01 ANSWER Q013 item 1: the planned detachments on Setup 3, one row per flight (NOT REVIEWED yet)
- Table: `results/analysis/rig_results_2026-11-13/detach_planned.csv` (detach_planned.py). It has 25 planned soft
  detaches of QUAD4 on Setup 3 (4, 5, 11, 12, 13 Nov), plus the three Setup 1 unannounced rows of
  tab:detach_context_0p55 copied unchanged. paper_rig's rules; judged after the flights, no bars.
- Every one held: nothing was lost in the 15 s after. Medians (range):

  | | Setup 3 planned (n) |
  |---|---|
  | UNLOAD to resize (s) | 3.18 (3.04-3.46), 25 |
  | magnet off its plate after the OFF (s) | 0.21 (0.14-0.45), 24 |
  | swing (deg) | 5.1 (3.1-23.8), 21 |
  | tilt peak (deg) | 6.9 (3.8-24.5), 25 |
  | settled at the new tilt (s) | 5.0 (0.7-12.0), 20 |
  | dip (mm) | 12 (-1-50), 21 |
  | horizontal offset (cm) | 8.3 (3.1-18.0), 25 |

- The ring was AT REST at every Setup 3 OFF (0.00-0.02 m/s). The unload holds the path:
  - nr9o_2 and nro began the unload on the circle, and the circle resumed 1.7 s after the OFF;
  - the fm9c* detaches came in the hover hold before the circle.

  So paper_rig's "Planned, moving" column is the launch's label, not the ring's motion. The only moving detach is
  Setup 1's r305 (unannounced, 0.13 m/s). There is no figure-eight detach (nr8 on 13 Nov never armed).
- Outliers:
  - sq8_2: the square, then the T leaned (swing 23.8 deg, peak 24.5);
  - 4 Nov nr9m: peak 15.3 deg, dip 50 mm;
  - nr9 and nr1_2 flew the planner built for the 1.15 kg ring.
- Four flights detached right at the lift's end, with a baseline under 1 s: their swing, settle and dip are empty. The
  dip depends on its baseline (nrtfc_2: +12 mm on 3.1 s, -10 mm on a plain 5 s): state the rule with any dip.
- Settling back to the OLD tilt happened in only 3 of 21: on three quadrotors the ring settles at a new tilt.
- Setup 1:
  - detected 0.33-0.42 s after the OFF;
  - swing 5.3 / 8.0 / 16.9 deg (hover, circle, the 150 deg gap);
  - the table holds three rows, not two (the 120 deg pair is r303_4 and r305).

### F120 2026-11-13 20:01 ANSWER Q013 item 2: the re-attachments on Setup 3, before and after the funnel, and the resizing (NOT REVIEWED yet)
- Tables in `results/analysis/rig_results_2026-11-13/` (reattach.py, paper_rig's rules; judged after the flights,
  no bars):
  - `reattach_attempts.csv`: one row per attempt, 26 including 3 excluded;
  - `reattach_resize.csv`: one row per proven attachment, 9;
  - `reattach_tensions.csv`: the four planned tensions every 0.5 s, from the fold-in -2 s to +15 s.
- Counts, excluding the VOID nr9m_3, nrtfc and nrtfc_4, and with sq6m (from the mat) apart:

  | funnel | attempts | descents | contacts | proven | held |
  |---|---|---|---|---|---|
  | none, 4-5 Nov | 13 | 43 | 12 | 1 (sq8_2, the square) | 1 |
  | v4, 11 Nov | 2 | 2 | 2 | 2 | 1 |
  | supervisor's, 11-13 Nov | 7 | 25 | 9 | 5 | 5 |

- With a funnel, the ring at rest:
  - 6 of 7 were proven on the 1st or 2nd contact;
  - 5 held for 15 s or until LAND. nrtfc_3's magnet let go 3.8 s after the fold-in with no command (the loose lead).
- On the circling ring (nro, no pause): 12 descents, 1 contact, no proof.
- The resize at each proof (9 proven):
  - contact to proof 2.0-3.5 s; proof to fold-in under 0.1 s;
  - the path is held 1.5 s and resumes at +1.7-1.8 s;
  - the planned tensions blend over 10 s from (3 x 3.25 N, QUAD4 0.10 N) to about 3.6-3.8 N on plate 3, 2.1-2.4 N
    on plates 7 and 11, and 1.6-1.8 N for QUAD4;
  - nrtfc_2 and nrtfc_3 landed before the blend ended (+8.6 s).
- The ring from the fold-in to +15 s (or LAND), triangle flights: tilt peak 4.8-8.5 deg, dip -0.2 to 18 mm,
  horizontal offset 4.3-11.1 cm.
- sq8_2 (the square) levelled from the T's 26.7 deg lean: peak 30.7 deg at +0.2 s, 4.2 deg at the end.
- Labels:
  - paper_rig's "Re-attach moving" nr9o_2 had the circle paused: the ring was at rest.
  - fm9hc_2/_4/_5 are attachments after a drop (QUAD4 started on its mat holding the object), not re-attaches after
    our detach.
- The funnel for 12-13 Nov is inferred from the sheets and registry: paper_rig's conditions.csv has no lines for those
  days.
- Credit the attachment itself to his single-quadrotor controller, and the resize to our planner (Q013's framing).

### F121 2026-11-13 20:01 ANSWER Q013 "Also": the time series of nrtfc_2, nrtfc_5 and fm9_5
- Files in `results/analysis/rig_results_2026-11-13/`: `nrtfc_2_poses.csv`, `nrtfc_5_poses.csv` and
  `fm9_5_poses.csv`.
  - 20 Hz, the whole flight to 10 s after Landed (fm9_5: -14 s to +209.7 s after TAKEOFF).
  - The 12 Nov fm9_5_poses.csv columns, plus t_epoch, t_from_takeoff, n_ours, d3_mux, Tplan_d0..3 (our planned
    tensions), Tpull_d0..3 (the trackers' estimated pull, ARM to DISARM) and event (CONTACT, proven, FLEET RESIZED,
    UNLOAD, DETACH, LAND, ...).
- In nrtfc_2 and nrtfc_5 the object columns are a body standing still on the floor: no object flew in them.
- The 12 Nov cpp_realtime fm9_5_poses.csv stops at +157 s, so use this one.

### F122 2026-11-13 20:15 REVIEW of F119-F121 (Q013 items 1-2, time series): SUPPORTED WITH CORRECTIONS; the tension question answered
- Method: the reviewer checked with its own parsers and bag reads. It found no missing or double-counted detach or
  attempt.
- **F119 (detachments):** the 25 detaches are confirmed, and 9 recomputed match within 2%. Corrections:
  - Swing median 5.0 deg, not 5.1.
  - The horizontal offset "8.3 (3.1-18.0)" mixes two rules: the 14 hover rows use distance from the mean position
    (median 10.1, 8.1-18.0 cm), the rest use path distance (5.4, 3.1-7.0). With one rule for all 25 (xy_err_peak_cm):
    5.7 (3.1-15.6) cm.
  - The medians mix configurations. Without nr9 and nr1_2 (the 1.15 kg planner) and sq8_2 (the T), 22 rows:
    swing 4.7 (3.1-11.6) deg, settle 4.7 (0.7-8.0) s, tilt peak 7.1 deg, dip 12 mm.
  - 24 of 25 baselines are under 5 s, and the dip depends on the baseline (nrtfc_5: 15.7 mm on 3.3 s, 0.6 mm on 5 s).
- **F120 (re-attachments):** every count and time reproduces. Contact to proof 1.98-3.54 s, proof to fold-in under
  0.08 s, the path resumes at +1.68-1.81 s. Corrections:
  - The "none" row has 12 attempts, not 13 (RIG-1105-sq8 never detached).
  - With a funnel and the ring at rest there are 8 attempts, 7 proven (5 on the 1st contact, 2 on the 2nd), 6 held.
    F120's "6 of 7, 5 held" was wrong.
  - 3 of the supervisor's-funnel proofs were deliveries from the mat (12 Nov fm9hc_2/_4/_5). Re-attach after our detach
    only, supervisor's funnel: 4 attempts, 17 descents, 5 contacts, 2 proven, 2 held (ring at rest: 3 / 5 / 4 / 2 / 2).
  - Before vs after the funnel is NOT one variable: the descent bands (0.08/0.10 -> 0.05/0.05), the tree and the contact
    gates (45 / 60 / 25 mm) changed with it.
  - Tension endpoints (the reference split): plates 7/11 1.90-2.43 N, not 2.1-2.4. Plate 3 3.61-3.84 N and QUAD4
    1.56-1.81 N stand. sq8_2 (the T) goes 4.87 / 0.10 / 4.87 / 0.10 -> 2.30 / 2.24 / 2.54 / 2.57 N.
  - nrtfc_3's 8.5 deg / 16.3 mm came AFTER its magnet let go (+3.84 s). Cut there: 4.4 deg / 8.3 mm. The triangle
    ranges become tilt peak 4.4-6.9 deg and dip -0.2 to 18.4 mm.
  - fm9hc_4's plate_d3 = 10 is the planner's captured weld point (287 deg); the magnet marker sat at plate 9 (273 deg).
- **F121 (time series):** coverage, columns, event alignment and the still object are confirmed. Corrections:
  - The sampling is ~19.4 Hz sample-and-hold, with single gaps of 0.38 s (nrtfc_2) and 0.34 s (fm9_5).
  - Tpull starts at TAKEOFF, not ARM.
  - Tpull_d3 is FROZEN while his stream flies QUAD4 (d3_mux = partner): the tracker skips the update outside its
    authority.
  - The old fm9_5_poses.csv ends at +158.1 s.
  - The dN columns are drone ids, and a drone id is not the same plate on every day: on 11 Nov QUAD2 (d1) is on plate 3,
    on 12 Nov QUAD3 (d2) is.
- **The thesis chat's tension question (nrtfc_5):**
  1. They are different quantities.
     - F120 / reattach_resize give the planner's REFERENCE split (RESIZE_INPUTS t_from -> t_nom, by slot, no
       integral). It sums to 9.74-9.77 N (0.86 kg x g / sin 60 deg).
     - Tplan in the poses files (and reattach_tensions.csv) is the OCP's node-0 tension by drone: the reference plus the
       model integral's vertical force plus transients. In nrtfc_5 the integral is -1.6 to -1.8 N, so node-0 sums to
       11.65-11.83 N; nrtfc_2 and fm9_5 are similar, fm9hc_2/_4 sit within 0.1-0.25 N of the reference.
     - With attach_blend_balanced on, only QUAD4's reference ramps linearly; the incumbents are re-solved every step.
  2. The mapping is right. nrtfc_5 has d1 on plate 3, d0 on 11 and d2 on 7. The thesis chat's +61-63.5 s readings
     (plate 3 at 4.67-4.70 N, plates 11/7 at 2.77-2.87 / 2.52-2.69 N) are consistent.
  3. Which to use:
     - the REFERENCE endpoints to describe the blend;
     - the node-0 series as what the planner commanded (0.5-0.9 N above the reference per carrier in nrtfc_2/_5 and
       fm9_5). At +10 s over the 6 triangle blends that ended before LAND: plate 3 3.77-4.64 N, plates 7/11 1.99-2.91 N,
       QUAD4 1.32-2.14 N.
  4. **Tpull is NOT a measured cable tension.** It is 0.55 kg x |mocap acceleration - thrust map + g| with a fixed mass.
     - Four on the ring it sums to 7.8-8.0 N where 9.7 N is needed.
     - QUAD4 reads 3.25 N in nrtfc_5 and 0.84 N in fm9_5 against plans of 1.74 / 2.27 N.
     - The gap is per-quad thrust-map and mass error plus acceleration noise. Do not cite it.
- **The thesis must NOT claim:**
  - a planned detach on a moving ring (all 25 at 0.00-0.02 m/s);
  - a hover-vs-circle comparison of horizontal offset (two rules);
  - that the funnel alone improved re-attachment;
  - "5 of 5" for the supervisor's funnel as re-attach after detach;
  - an attach on a moving ring (nro: 12 descents, 1 contact, no proof);
  - the reference endpoints as commanded tensions, or any Tpull value as a cable tension;
  - nrtfc_3 as held, or its 8.5 deg / 16.3 mm as a resize response;
  - sq8_2's 30.7 deg as a resize disturbance (the T's 26.7 deg lean levelling to 4.1 deg).
- With these corrections F119-F121 are reviewed and may be cited. Q013 is complete.

### F123 2026-11-14 11:43 ANSWER Q014: the run folders of four figures are back on the laptop (relabel only)
- results/2026-11-06/: R3972 (fig_s8_chain), R3924 (fig_s8_laps), R3910, R3911, R3913, R3914 (fig_x4_relay). Whole
  folders, restored from the lab PC archive (drones:~/mdc_archive/results/2026-11-06/), 1.9 GB.
- results/2026-10-23/: R2764-R2779 (fig_solve_time), 16 whole folders including logs/dissipative_planner, extracted
  from results/archive/2026-10-23.tar.zst, 1.4 GB.
- No numbers change. Laptop disk: 17 GB free. Say when they can go again: the 6 Nov folders stay on the lab PC, and the
  23 Oct ones in the archive.
- F123 follow-up 11:44: the thesis chat re-rendered the four figures; the restored copies are removed again (originals:
  drones:~/mdc_archive/results/2026-11-06/ and results/archive/2026-10-23.tar.zst). Q014 closed.
- Q015 12:24 (relabel only, no numbers change): run folders back for the panel-title re-render. results/2026-11-06/: R3972,
  R3973, R3910, R3911, R3913, R3914; results/2026-11-07/: R4112, R4124, R4125 (1.9 GB, from the lab PC archive). They
  will be removed again after the re-render.
- Q015 12:26: re-rendered by the thesis chat; the restored copies removed again (originals in the lab PC archive). Q015 closed.

### F124 2026-11-14 14:51 DECISION (Wesley): the credit wording for the collaborator's controller
- Write it without specifics: a controller from a collaborating student, for a single quadrotor with a swung payload,
  makes the attachment (and flies the drop-off). The proposed planner is credited with the payload's response and the
  resizing (Q013's framing). The "C++ approach, Python return" detail (F111) is not needed in the text.
- Q016 15:13 (text colour and capitals only, no numbers change): run folders back: results/2026-11-06/ R3972, R3973, R3910, R3911, R3913, R3914 (lab PC archive); results/2026-10-23/ R2764-R2779 (archive tar). Removed again after the re-render.
- Q016 16:23: Wesley approved the re-rendered figures (thesis 8c81c7d); the restored copies removed again (originals in the lab PC archive and the 23 Oct tar). Q016 closed.
