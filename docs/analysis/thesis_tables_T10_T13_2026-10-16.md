# T10 and T13 from data in hand, and figures F10b and F12 (16 Oct 2026)

Desk work, no runs: this note gives the numbers, the runs and the caveats. Wesley asked at about 12:30 on 16 Oct.

- **Generators:**
  - `tools/thesis_freeze_tables.py`: writes `docs/figures/tables/T10_switch.tex` and `T13_parity.tex`. Each is a
    complete table float (generated caption, label `tab:T10` / `tab:T13`) to `\input` in place of the skeleton tables
    in `skel_leave.tex` and `skel_ledger.tex`. It also prints the per-run listing quoted below.
  - `tools/thesis_lever_figures.py`: writes `docs/figures/F10b_crossover/` and `docs/figures/F12_levers/`. Captions,
    citing sentences and must-not-say lists are in each folder's `caption.md`.
- **Provenance:** `configs/thesis_figures.yaml` and `docs/figures/PROVENANCE.md` (16 Oct section). No run ids appear
  in any caption or `.tex` file.
- **Tree:** every twin number comes from the detach freeze batch (lab batch 15, R1887-R1919). That is day-1016 8c468bd,
  clean (checked in every manifest), whose flight code is week5 4b3560e's.
- **Rig numbers:** they come from the 7-8 Oct flights (rig trees 8f4fb2b with uncommitted changes on 7 Oct, b811d90 on 8 Oct). The 14-15 Oct flights flew
  no release and no switch.

## T13: the twin against the rig, on Delta-theta

The scorer is `tools/release_bands.py` with the signed defaults: event to +15 s for W4/W5, and the rig flight's hold
for the 8 Oct pairs.

| cell | twin, freeze tree (deg) | band set (deg), 11 Oct | band (deg) | twin inside | rig |
|---|---|---|---|---|---|
| W4 hover, unannounced | R1887 6.05 (n = 1) | R1459 6.12, R1460 5.74, R1626 6.67 | [1.75, 14.51] | yes | not flown |
| W5 circle, unannounced | R1913 6.70, R1914 7.11 | R1457 4.87, R1627 5.91 | [1.00, 13.20] | yes | not flown |

| 8 Oct pair (agreement only) | twin runs (deg) | hold (s) | rig (deg) | rig / twin mean |
|---|---|---|---|---|
| r302_3 hover, announced | R1207 5.95, R1212 7.13 | 6.6 | 11.21 | 1.71 |
| r303_4 hover, unannounced | R1140 6.00, R1145 6.44 | 7.6 | 5.29 | 0.85 |
| r304_3 hover, unannounced, 150 deg | R1206 14.63, R1211 17.39 | 7.7 | 16.88 | 1.05 |
| r305 circle, unannounced | R1208 5.41, R1213 4.00 | 14.0 | 8.02 | 1.70 |

The unannounced class gives f_lo 0.851 and f_hi 1.704 (3 pairs), and sigma_rig is 1.568 deg. The bands recompute to
the frozen values from their own inputs, with the signed rule.

Caveats:
- **W4 twin column is R1887 only (n = 1).** This follows the departed_disarm_alone reviewer. R1891 and R1895 moved the
  release by +0.5 and +1.0 s; they read 7.68 and 5.46, also inside the band. The thesis must not say "W4 n = 3 on the
  freeze tree".
- **The freeze-tree runs are not band inputs.** They only show that the band stands on the freeze tree (F1 confirmed).
  The band is never recomputed from them.
- **No rig rows.** Neither W4 nor W5 has flown on the rig (14-15 Oct flew neither), so the scored rows read "not yet
  flown". When they fly: add `rig:<date>/<run>` to `W4_RIG` / `W5_RIG` and re-run the script.
- **R1140/R1145 carry uncommitted diffs.** They were flown on week4 0e91d86 with 14 dirty files, and the two runs'
  diffs differ (the P6 reviewer). They are fine as agreement, and they are labelled in PROVENANCE.
- **"Rate fix" column.** Before or after the rig's body rates went to full precision (decisions.md, 8 Oct evening). It
  describes the rig flight, so W4/W5 read "after".

## T10: the switch itself

The switch is the planner's resize when a carrier leaves: the planner log's `n` falls from 4 to 3. For each switch,
the script reads three things:
- at the first solve of the smaller problem: the status, the logged KKT residuals (`res_stat`, `res_eq`; the
  inequality and complementarity residuals are not logged), the QP iterations and SQP calls, and the solve time;
- in the 1 s after that solve: the failed solves, the largest residuals and the longest solve;
- the survivors' throttle step: m1_metrics' hand-over definition moved to the switch. It is the largest |thr_out minus
  its mean over the 0.1 s before| in the 0.3 s after, over the hover throttle (the mean 3 s to 1 s before), taken over
  the survivors. Beside it sits a null: the same statistic at every planner tick 6 s to 1 s before.

**Unique ticks.** `log.csv` repeats the last solve on every creep and hold row and after a path ends. So only rows with
`published` set (solved, shifted or none) count as solves. De-duplicating on `traj_t` does not work for the hover
releases, because `traj_t` stays 0 for the whole run. Rows with `published` set never repeat `solve_ms` in a hover run.
A circle run has 8 equal consecutive values among 540, which are coincidences at 0.1 ms resolution.

| switch | data (n) | status at the first solve; failed in 1 s | KKT stat / eq, first solve (max over runs) | rig solve time: first; max in 1 s (ms) | throttle step % (null max) |
|---|---|---|---|---|---|
| leave, announced | rig, planner's pull, rods at 45 deg, 7 Oct (2) | 0; 0 of 22 | 1.1e-3 / 1.1e-7 | 24-28; 58-60 | 12.6-15.8 (1.2-3.9) |
| leave, announced | rig, measured pull, 8 Oct (1) | 0; 0 of 11 | 6.5e-5 / 2.4e-8 | 26; 35 | 15.9 (7.3) |
| leave, unannounced | twin, measured pull, gaps 120-165 deg (13) | 0; 0 of 130 | 3.8e-4 / 2.3e-7 | (not reported) | 1.9-5.7 (0.4-0.9) |
| leave, unannounced | twin, measured pull, 180 deg, drop rule off (2) | 0; 0 of 21 | 3.9e-4 / 1.7e-8 | (not reported) | 11.5-12.1 (0.4-0.5) |
| leave, unannounced | twin, planner's pull, 120-150 deg (5) | 0; 0 of 34 | 9.9e-5 / 2.4e-8 | (not reported) | 11.4-16.8 (0.4-0.5) |
| leave, unannounced | rig, measured pull, 8 Oct (3) | 0; 0 of 33 | 3.3e-4 / 2.9e-7 | 25-26; 32-62 | 9.2-13.3 (6.8-9.4) |
| join (hand-over) | the join tree | pending | | | |

What it says:
- **No solve failed at any switch.** That holds for 26 switches (20 twin, 6 rig): every first solve has status 0, and
  no solve failed in the second after it. Every first solve took the warm path: 5 SQP calls and 10-11 QP iterations,
  no cold reseed.
- **G180 is the exception on QP iterations.** It took 30 QP iterations at its first solve. Its residuals in the 1 s
  after reached 0.14 / 2e-4 as the ring fell to the latch, and the solves still returned status 0.
- **The first solve is as converged as the one before it.** The first three-UAV solve has residuals in the range of the
  last four-UAV solve one tick earlier: stat up to 4.4e-4 and eq up to 5.0e-7 across all rows.
- **Steady hover differs between twin and rig.** In the twin it reads 1e-6 to 5e-5 (stat). On the rig it reads up to
  8.8e-4. The release transient, not the switch, lifts the twin's stationarity residual to about 1e-4.
- **The rig solves the smaller problem quickly.** The first three-UAV solve took 24-28 ms. The four-UAV problem took
  53-56 ms (p50) and 73-79 ms (p95) before the switch. In the second after the switch, 2 of 66 solves passed the 60 ms
  budget (60.0, 61.9 ms), and none was budget-cut (minimum SQP calls 5).
- **Throttle step on the measured pull: the release, not the switch.** The twin's detection-off runs never switch. In
  the same window (release + 0.33 s) they read 1.9 / 2.6 / 2.5 %, against 2.1 / 2.6 / 2.2 % for the arm that switches.
- **Throttle step on the planner's pull: the switch itself.** It is the planned step arriving: 11.4-11.9 % against
  2.3-3.3 % without a switch. This is the "planned physical step, a lever on the planned pull" of the thesis plan.
- **The rig's throttle is noisier.** The null max is 6.8-9.4 % on 8 Oct, so the rig's steps of 9-16 % are 1-2 times
  its null.

Caveats:
- **Twin solve times are not reported.** On the lab PC with 4 slots, the first three-UAV solve took 15.6-88 ms and the
  four-UAV p95 was 106-177 ms, which is load, not the solver. This matches the 16 Oct corner-cutting note: the stack is
  2-4 times slower than the solver alone.
- **The rig rows come from older trees.** They are the 7-8 Oct flights (8f4fb2b with uncommitted changes on 7 Oct, b811d90 on 8 Oct), not the
  freeze tree. The 14-15 Oct rig flights never changed size: 47 planner logs, n = 3 throughout. Their planner-phase
  solve times (31-46 ms p50, 46-65 ms p95; corner-cutting card) are the current rig's steady numbers, without a switch.
- **The gap to an offline SQP solve is not computed.** The planner dumps a solve's QP only when the solve fails
  (`MAX_QP_DUMPS`), and no solve failed. Two ways to get it, both Wesley's call:
  - a logging switch that dumps the QP at the resize tick (flight-adjacent code, default off);
  - `tools/planner_replay.py` on the logged states. That needs a solver build of a few minutes of CPU, and it
    reconstructs the unlogged ring velocity and rates, so it is approximate.
- **G150p and G180.** In G150p (2 of the 5 planner's-pull switches) the trackers' 30 deg drop came 0.18-0.20 s after
  the switch, and the throttle window ends there. In G180 (drop rule off) the fleet latched 0.41-0.44 s after the
  switch, outside the 0.3 s window, so its 11.5-12.1 % is the ring falling.
- **Thirteen runs of the batch have no switch.** They are A5/A6 (detection off), A7/A8 (no event), and D4 and G180d
  (the planner dropped the ring at detection instead of resizing).
- **The join row** waits for the join tree (after the 21-22 Oct rig day). No rig join has folded a fourth UAV in yet
  (14-15 Oct: n stayed 3).

Per-run listing: `python3 tools/thesis_freeze_tables.py t10` prints one row per switch (run id, status, residuals at the
switch, in 1 s, before and one tick before, QP/SQP, solve times, the throttle step per survivor and the null).

## F10b and F12

| figure | path | what it shows | numbers (from the script) |
|---|---|---|---|
| F10b | `docs/figures/F10b_crossover/crossover.pdf` (6.27 x 2.45 in) | (a) tilt departure against time from the release, A1 (measured) against A2 (planner's), three runs each, the detection band; (b) Delta-theta per run, the arm means and the gap the frozen rule needs | A1 5.46 / 6.05 / 7.68, peaks at 0.40-0.41 s; A2 16.64 / 16.68 / 17.08 at 0.51-0.54 s; detection 0.28-0.37 s; gap 10.40 against 4.44 needed: resolved |
| F12 | `docs/figures/F12_levers/levers.pdf` (6.27 x 3.0 in) | (a) one-variable pairs: pull source at 120 and 150 deg, detection off on each pull source; (b) Delta-theta against the support margin, with the rig points | (a) T3 6.40 against 16.80, resolved; P150 16.82 against >= 29.54, resolved; T9 6.40 against 5.58, no resolvable difference; T8 16.80 against 28.18, resolved. (b) G120 5.5-7.7, G135 7.8-9.1, G150 16.7-17.0, G165 27.9-28.1; G180 >= 60.7 / 61.7 (latch, drop rule off); G180d >= 24.1 (gap rule at detection); G150p >= 28.5 / 30.6 (trackers' 30 deg drop); rig r303_4 5.29, r304_3 16.88 (to LAND); r0013 >= 68.3 (P6 table) |

Every lower bound recomputes to the card's values: G150p 28.46 / 30.61, G180 60.71 / 61.72, G180d 24.08. Each is cut
at the DROP AND LAND or ENVELOPE FAULT stamp, mapped to run time as `tools/drop_metrics.py` does.

What the figures respect:
- **Lower bounds:** every ">=" value is drawn as a lower bound (arrow or triangle).
- **G180:** it appears only labelled "drop rule off". G180d, on the defaults, has its own marker.
- **B4:** not drawn as a bar.
- **K1:** not drawn.
- **T9:** reads "no resolvable difference".
- **T15:** left out.
- **F10b:** has no rig point, because no rig flight of this cell exists. `RIG_F10B` is empty until W4 flies.
- **F12b rig points:** they are the ones the Tier B card names: r303_4 and r304_3, plus r0013 off scale as the thesis
  plan asks.
  - r0013's value comes from the P6 release table (`results/analysis/p6_2026-10-12/1_releases/table.csv`), because its
    bag conversion exists only in the thesis worktree's results.

The figure loop:
- [rounds, audit and the critic's verdict: filled in below]
