# Session state (read after every compaction; keep it short and current)

Updated: Wed 21 Oct 2026, 19:00 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## Where things are (12 Oct tidy)
- **One tree:** `~/multi_drone_control` on **week5**: code, docs, the registry and the working rules. Built, prebuilt,
  gate --quick passes (609 tests). rig-prep is retired (merged). week5 is the testing tree for the rig and the twin.
- **Layout (12 Oct):** `docs/` top = state, decisions, the manual, the plans; `docs/rig/`, `docs/tejen/`, `docs/reports/`,
  `docs/analysis/`, `docs/experiments/` (cards), `docs/figures/`, `docs/design/`, `docs/archive/` (everything finished;
  a registry row's `docs/X.md` is `docs/archive/X.md`). `configs/experiments/` = the 106 configs current cards, tonight's
  batch or tests use; `configs/experiments/old/` the rest. `tools/old/` = one-off study scripts. `generated/` = every acados solver and
  the harness PNGs (was the repo root; MDC_ACADOS_ROOT on the lab).
- **Other worktrees:**
  - `mdc_wt/thesis` (thesis-rig): the assembled thesis, 122 pp. Ch2/Ch3 rewritten; the Ch4 skeleton has F8/F10/F11/F14,
    and T7/T8 are filled after a reviewer.
  - `mdc_lag` (time-cascade) and `mdc_safety` (safety-defaults) are Wesley's.
- **Backups:** every branch and tag is on GitHub under `backup-2026-10-12/*` (verified), and in
  `~/backups/multi_drone_control_2026-10-12.bundle`. Flight data (`results/`, 34 GB), `docs/papers` and his twin logs
  are local only.
- **Archive tags:** archive/m1-tiltcut, archive/m1-e0, archive/m1-full, archive/combined-m1, archive/m1-video.

## Now (Fri 16 Oct, 08:20): day branch `day-1016`, Wesley at work until the evening
- **Merged:** overnight-1015 into week5 (fast-forward to 4b3560e, Wesley ~07:40). Today's work commits on `day-1016`
  (from week5; Wesley merges tonight). Overnight in short: `docs/reports/morning_report_2026-10-16.md`.
- **Decisions this morning** (docs/decisions.md, 16 Oct, two DECIDED lines): drop_takes_back ON at the rig via
  params_file (CLAUDE.md's locked "Open" line closed) plus a console **his LAND** button (his planner's land_now =
  straight down where QUAD4 is, magnet OFF; his ground-ring exit back to his mat is NOT on in our launch, so over a
  fallen ring it lands on it; Wesley kept it with that label); zero twist: hardware first, then A/B if Tejen agrees;
  `--slot-gate` on at the rig; backup of results/ by Wesley tonight. Inventory: carry and detach freeze on week5 now
  (4b3560e), join after the 21-22 Oct rig day; carry card bars frozen as drafted; 0.55 m baseline, 1 m design row;
  appendix re-flown and regenerated; T11/F14 on co-motion (join tree); N2 1 m bands before the rig day; T12 hover
  under a card, circle exploratory (OPEN line closed); Tier B 120/150/180 + 135/165; Tier A no-code cells now; L5
  twin column R1712/R1713 (against the recommendation: different integral from the rig flight, caption says so);
  push-abort bar = QUAD4's own clearance (new runs only).
- **Lab PC: DO NOT USE** (Wesley ~11:20: someone else wants it; batch 17 stopped, my containers removed, nothing of mine left). Wait for his word. Twin runs go on the laptop, one at a time, idle for the first ~3 min.
- **Commits on day-1016:** 5375333 (decisions, his LAND button, HIS_LAND harness event, his-LAND configs), 83a7088
  (detach freeze cards + configs + 3 Tier B worlds, bars frozen 08:41 while Wesley is at work; N2 1 m configs and
  band draft), e836cf3 (sheet). Registry rows to R1856 (uncommitted).
- **Results:** his LAND after a drop (R1844-R1846): straight down onto the fallen ring; after a drop at the plate QUAD4
  stays armed on top of it: use it only clear of the ring. **Carry claim done** (R1847-R1886): 39/40 pass; spin at
  0.3 m/s fails the tilt bar once (R1875 6.31 deg): outside the envelope; 1 m arm passes; reviewer SUPPORTED WITH
  CORRECTIONS (applied: payload_rmse_m is takeoff+5 s to LAND, the tilt peak is whole-flight; must-not-say list).
- **Detach freeze scored** (R1887-R1919, 2c1985a): T3, T8, T9 (no sign), T15, B1-B3, B5, W4/W5 F1+P1-P4, D1/D4 all
  confirmed; K1 planned unresolved (K1 measured confirmed on the tool's reading, which with detection off is the freed
  drone's lean); B4 holds on the measured pull (G150p reached 30.9 before its drop); A6 (planned, detection off) holds
  the ring at 28 deg and aborts at landing 3/3 (scored as the arm's result). Reviewer running on the four cards.
- **Corner cutting (card 2026-10-16_corner_cutting.md; fix plan approved ~17:20: `~/.claude/plans/kind-snacking-pinwheel.md`):**
  the frozen path RMSE scores against the 10 Hz HELD desired, so an on-time ring pays 17 mm at 0.3 m/s. Wesley chose the
  ruler: clock-free distance to the path + lag in seconds (decisions.md 16 Oct). On it the timing combination removes the
  cut (figure-8 bends +20..+31 -> +3..+7 mm) and turns the lag into a 0.04-0.06 s lead on the circle/orbit (R1986c-R1991c).
  Tools (6970467): `metrics.contour_lag` (new metrics.json keys payload_contour_*, payload_lag_s, payload_rmse_sched_m),
  `tools/corner_metrics.py R1847-R1991 [--chain] [--D]`, lag_metrics A0 on the planner's own clock. Timing fix back in play
  in the twin (rig needs his word: reopens the 8 Oct lag decision).
  **G2 done (R1992-R2005, 391bd9a + this commit):** with ref_time_shift + 2 iterations, the lead that centres the
  ring is 0.075-0.078 on the orbit and circle (= 0.1 - D: pure delay; cut gone, across rms 11-13 vs 17-19 mm) but
  ~0.05 on the figure-8 (at 0.075 it runs 0.033-0.038 s late). Offline bench `tools/latency_bench.py` FAILED G1
  (catch-up 6x weaker than the twin): code paths only. Reviewer on the afternoon claims running; then /report and
  Wesley's decision on the next build (recommended: a measured lead, plan_ref_lag_s from the measured plan age, with
  ref_time_shift; then the figure-8 residual).
- **Evening queue done (20:25-01:42, laptop, c3bf628, R2006-R2076, 71 runs):**
  - **N2 voids re-flown** (R2006-R2015), all landed, no faults. 7 lab "passes" were starved too (RTF 0.28-0.29,
    RMSE 48-68 mm, tilt 10-15 deg): correction rows R1921c..R1936c VOID; re-fly of those 7 queued after the appendix scoring.
  - **Figure-8 repeats** (R2016-R2022) are in the card. On the cut both leads remove most of the bend cut. The 0.075 arm
    scatters across sessions (+15.5/+9.6 then -8.8/-2.4). The figure-8 lag is too noisy to rank leads, so
    "figure-8 needs 0.03 s less" is NOT supported.
  - **Appendix scored** (R2023-R2076, rows appended; docs/analysis/appendix_refly_2026-10-17.md; generator edits in
    tools/thesis_sim_tables.py / thesis_sim_figures.py, regenerated to the scratchpad only):
    - S: 54/54 pass, no voids.
    - A: 29/60 cells agree. C: 10 confirmed, 3 falsified, 2 unresolved.
    - Falsified C-D2/C-D3/C-D5: on the freeze stack the vertical push and the mass errors leave ~0 cm with no integral
      (measured pull, cable_source mocap; checked R2066 load_mass 1.03 reached the planner).
    - Relaxed box hover +13 cm (was +4.8); rods +5 not measured +17 cm (was +8).
    - Radius +2 cm: int_fz winds to +0.76 N while the height error stays at +0.85 cm (R2061/R2076): look before the rig day.
    - Reviewer next, then Wesley: the thesis text for the vertical claims.
  - **N2 re-fly of the 7 starved runs done** (R2077-R2083, 01:58-02:33): all landed, no in-flight faults. R2080's FAIL
    happened after landing. spin_v300 tilt 6.16 deg, over the carry bar, as at 0.55 m (R1875). Every N2 cell is now clean;
    the band rule still needs Wesley.
  - **Reviewer on the appendix: weak.**
    - False touchdown: R2032, R2044, R2045 disarmed 0.46-0.53 m up and the drones fell; R2033 0.26 m. The stall
      detector reads a drone that is stronger than its map as stalled. RIG FINDING; a fix needs card + critic.
    - Correction rows R2032c/R2033c/R2044c/R2045c. C-L1 not supported.
    - Vertical push absorbed by the measured pull (0.61 N within ~1 s).
    - Morning report: docs/reports/morning_report_2026-10-17.md (decisions: touchdown fix, cable_source model pair,
      measured lead, band rule, merge).
  - Lab PC checked 20:22: someone else's 12-sim batch, not used.
- **Laptop rebooted ~15:59: `/tmp` wiped** (scratchpad scorers, figures, batch logs, registry.bak17-28). Scorers restored
  into tools/ (above); the old figure scripts are in the scratchpad `restored/` folder only.
- **For Wesley tonight:** the 3 Tier B world files (new, via make_carry_worlds.py); the 1 m band rule; T15's layout
  (even ring chosen); the bars set in his absence (he may change them for later runs).

## Now (Sat 17 Oct, ~10:45): Wesley's answers to the morning report (decisions.md 17 Oct)
- Figures page for him: https://claude.ai/artifact/2MuafaZQuWTusdeLxJLukT (script: scratchpad/make_figs.py).
- **(1) Touchdown:** card rev 2 (`docs/experiments/2026-10-17_touchdown_height_gate.md`).
  - Code committed b1be021, configs b7ab074: arm A `land_rearm_touchdown`, arm B `land_height_gate`; 167 planner
    tests and the gate pass.
  - `tools/land_touchdown.py` now reads past LANDED (landed_dz_cm, post/end tilt, exit); committed 8009445.
- **(2) Vertical claims:** cause confirmed. With cable_source model the push leaves -7.95 cm and +20 % mass +22.2 cm
  (R2084-R2087; measured pull ~0). Next: rewrite the thesis text around the measured pull.
- **(3) Corner cutting:** Wesley ~11:00, sweeps first (decisions.md).
  - Offline: plan tick -> tracker arrival is 0.018 s (2 iterations) / 0.026 s (5) in sim time, which plus pose age is
    the 0.025 s leftover. Only the tracker can measure it, which favours the stamped shift.
- **Batch bppon3jkw done 13:40 (R2088-R2124), scored (results in the touchdown card):**
  - touchdown: A fixes the 0.5 m case (also R2094: the rig LAND path, switches off, normal maps, disarmed 0.42 m
    up). The strong-map drone still stops 15.8 cm up (steady-state height error once the ring rests) and tips
    24-30 deg: H1 FAILS. B never acted. Decision for Wesley.
  - orbit 0.125 sweep: lag crosses zero at lead ~0.064 (0.078 at 0.3); 0.075 centres both speeds within 0.011 s.
  - delayed twin R2117-R2124: all flipped in the takeoff creep (FAIL setup). Twin switch SIM_MOCAP_DELAY_FROM_S
    added (8009445).
- **Wesley ~13:55:** keep A, delete B; A default (rig too) once the reviewer supports it; card the residual.
  Card `docs/experiments/2026-10-17_land_ref_below_rest.md` rev 2 after the critic: Step 0 done (drone 1 = steady-state
  height error, its rod in tension; drone 0's 4.7 cm = rigid rod + xy freeze, twin only). Switch
  `land_descend_to_stall` (no floor after ring touchdown) and B deleted: coded, UNCOMMITTED, tests written but NOT RUN
  (laptop busy). Configs td_nf_* + td_a_mapneg45_ref; td_ab_* moved to configs/experiments/old/.
  Next after the batch: pytest + gate --quick + prebuild, reviewer on A, then fly the card's arms.
- Delayed twin R2125-R2132 scored: lead = delay falsified (crossing 0.078 -> ~0.081 with +0.035 s delay).
  **Wesley ~14:29: fixed 0.075, no new code** -> card `2026-10-17_lead075_default.md` (neighbour bars frozen).
- Code committed 3edeb04 (land_descend_to_stall, B deleted; 20 + 316 tests, gate --quick pass), 0a77fa1 (configs).
- Reviewer on A: **weak** (rig path 1 of 2 base; A alone not flown there; 10 deg end tilt on the rig path).
  A stays off until a re-fly supports it.
- **Batch b3k2h4x1c done 17:37 (R2133-R2167, tree 0a77fa1), scored, rows appended (bak39), cards updated:**
  - A re-fly: rig LAND path off fires the false touchdown 2 of 4 (R2094, R2135), A 0 of 5; A ends drone 0 at
    10-11 deg in 3 of 5. A on hover/creep/lift lands as its base.
  - land_descend_to_stall: strong-map residual gone (F1 pass, also at error 0.045), F3/F4 pass; F2 fails as written
    (drone 0 lands at rest, 4.7 cm lower than the base: two-sided bar). With A on the rig path the end tilt is 0.4/1.0.
  - lead075 neighbours: N5 fails on spin (lag -0.045); the rest pass. Stays exploratory.
  - Reviewers: A supported (twin default; rig needs critic + Wesley); no-floor (a) supported, (b) not supported (F2, contact speed 0.17-0.35 m/s); lead075 weak (spin N5 one lap); spin repeat R2168-R2171: fails N5 in 2 of 3 pairs, falsifier stands, exploratory.
- **Wesley ~19:45 (decisions.md):** A on in the twin (sim.yaml, gate --quick passed); rig after a critic
  look at the wall-clock pose check. No-floor: re-card (one-sided height bar + contact-speed bar) and re-fly. Corner
  cutting CLOSED (defaults stay, lag reported). N2 bands FROZEN ~19:54 (tools/n2_bands.py, card 2026-10-16_bands_1m). Rig sheet updated (43cfd13). Critic on A: go with must-fixes; welded-QUAD4 stamp fixed (9d30665); QUAD4-folded-in LAND x2 flying (task b3ko1376i, scratchpad/laptop_q4.log). Then the no-floor rev 3 batch (scratchpad/laptop_r3.sh, 27 runs; card rev 3, bars frozen ~20:00).
- **Overnight (Wesley ~20:10):** task b8uawb0oz runs scratchpad/laptop_night.sh after the QUAD4 LAND pair (b3ko1376i):
  no-floor rev 3 (27), the 0.55 m band cells (15, one per cell to make 3; freeze the 0.55 m bands after),
  radius +2 cm wind-up (4: base, cable_source model x2, a 160 s hold), the join re-fly on today's tree (18,
  exploratory; the thesis joins still fly after the rig day). Log scratchpad/laptop_night.log. Then score each,
  registry, reviewers, morning report docs/reports/morning_report_2026-10-18.md.
  QUAD4 rejoin LAND pair R2172/R2173 never reached LAND (his mission stalled / QUAD4 fault in his descent); the A
  critic re-fly is now q4_welded_land (+ no-floor), 4 runs chained after the night batch (task b3c2thf2x).
  Thesis writing goes to a separate session: brief docs/thesis_rewrite_brief_2026-10-17.md (Opus 5.5, high).
- **(4) Band rule signed** (f = 1 +- 2 sigma_rig). Compute and freeze the bands (sigma_rig from RIG-1014-lift3).

## Now (Wed 21 Oct, 19:00): NEW PLAN approved, docs/plan_thesis_results_2026-10-21.md (read it first)
- Branch **day-1021** (from day-1016 at 410f0f1, today's rig files committed). tejen-sync2 committed at 6483d90 in
  ~/mdc_wt/tejen_sync2 (not merged yet; merge before the basket runs). Decisions: docs/decisions.md DECIDED 2026-10-21.
- Tonight (plan section 1), for Thu 22 8-11 am (QUAD2/3/4, 0.55 m): disk delete list (Wesley approves; 8.8 GB free),
  line_spin (code design A), worlds 2q 0/6 and T 0/3/6, twin runs with video (record, render after the batch, reel),
  the T on today's code (0/3/6/9 RELEASE 9, unannounced), the chain fix card + critic + code (drop rule, tether-mass
  split, attach_z), sheet docs/rig/thu_2026-10-22.md. Never edit src/ of the main tree during a batch: code the chain
  fix in a worktree while runs fly.
- Releases are UNANNOUNCED only (Wesley): no unloading before a release.

## Now (Wed 21 Oct, 16:55): the ring body pivot is fixed in Motive (Wesley); tilt next
- 4-angle bench (RIG-1021-ringbench): ring mocap centre was (-0.99, +1.39) cm off in the ring frame (fit rms 0.6 mm);
  body frame tilted 1.3 deg; floor or Motive ground plane 0.9 deg (normal toward -20 deg world).
- Wesley moved the pivot +9.9 mm toward plate 0, +13.9 mm toward plate 3 (RIG-1021-ringpivot): a 180 deg turn now
  moves the reading 0.3 mm (was 3.4 cm). EVERY rig flight from now on uses the new pivot; Te-Jen's stack reads it too.
- Budget of the 5.5-6.6 cm no-hold offset (scratchpad budget.py, planner 4.4-4.6 N/m): ring centre 4.6-5.0 cm (fixed),
  ring body tilt 1.3 deg 0.9-1.0 cm (sign confirmed by QUAD3/QUAD4 rod lengths), QUAD2 0-0.5 cm, left +-0.5 cm, if the
  0.9 deg is the floor. If it is Motive's ground plane: +2.1-2.2 cm more, then QUAD2 must cancel 1.3-1.6 cm; the flights
  cannot tell the two apart, the spirit level can.
- Open: spirit level on the floor (level floor -> Motive ground plane 0.9 deg off = 0.13 N sideways in flight; sloped
  floor -> shim the ring level and reset the ring body orientation); QUAD2 link ~2 cm short (bench: rod, magnet, Motive pivot).

## Now (Wed 21 Oct, 16:24): offset cause fitted (desk, exploratory, no reviewer): ring mocap centre + QUAD2
- Geometry fit over 16 flights (scratchpad geomfit.py / geomfit_run.py, session 3e643efb): ring mocap centre fitted
  (-1.23, +1.33) cm ring frame = bench (-1.15, +1.30); QUAD2 link ~2.2-2.6 cm short vertically (pivot or magnet).
  Together they remove the no-hold false pull (0.25-0.29 -> 0.02-0.05 N). Hold flights keep 0.12-0.32 N (open).
  Registry RIG-1021-offsetc (bak52); addendum in docs/reports/rig_2026-10-21.md.

## Now (Wed 21 Oct, 16:12): why the no-hold offset happens (desk, exploratory, no reviewer yet)
- No-hold flights (c125_2, c200_2, f125): the quads sit within ~0.5 cm of their refs; the planner plans the ring 2.2-2.6 cm
  back per 0.5 s and shifts the quads 1.3-1.5 cm toward the path (0.28-0.33 N pull in its model); the ring does not move.
  From the measured pivots alone (rigid 0.55 m rods, free hanging), mocap puts the ring 1.2-1.4 cm from where it must hang
  (ring frame 109-128 deg, sd 0.3). The planner is ~5 N/m soft against ~18 N/m of rod geometry, so the offset settles at
  ~3.5-4x the measurement error. This supersedes the report's "re-centring fixes about a third" (it assumed 1:1).
  Rod-length pattern does not fit a ring-centre shift alone (QUAD2 also suspect). Hold flights do not fit free hanging
  (ring tilts about half the free-hanging tilt). Scripts: scratchpad offset_chain.py, hang.py (session 3e643efb).

## Now (Wed 21 Oct, 15:20): Tejen sync 2, avoidance on the transit and floor_join, QUAD1 pickup sheet
- **Worktree `~/mdc_wt/tejen_sync2`, branch `tejen-sync2`** (from 1db5e0d, nothing committed; Wesley's word needed):
  his `SwungPayload2026_ThesisCExperiments` (fd495db0, 60 commits since sync 1) merged into our fork
  (`src/tejen_mission/CHANGES_FROM_TEJEN.md`, 21 Oct sections). Built; `gate.sh --quick` PASSES (1493);
  his mission tests 1102 pass / 24 fail (12 as before + his files we do not carry); C++ 20/20.
- **Avoidance (Wesley 21 Oct):** `carry_heights` in partner_mission (the object's bottom half the carriers' cable above
  them, typed values win), `carrier_model:=tether` (his rod model, fed by ring_bridge), `attach_cpp_transit`
  (floor_join's leg to the plate on his C++ planner). Unit-tested only; twin configs ready in the worktree:
  `basket_hover`, `basket_circle`, `floorjoin_cpp_hover` (not run yet). line_x with his mission is refused by our
  partner_contact guard (needs Wesley's word). Old configs that type 2.7 m with floor_join now refuse at launch.
- **QUAD1 pickup (Wesley, rig now):** `m1_irl_pickup.launch.py` (his IRL stack + an overlay; hold after the lift),
  `tools/pickup_geometry.py`, sheet `docs/rig/wed_2026-10-21_quad1_pickup.md` (all in the worktree).
- Registry: RIG-1015-a1c (the tether struck QUAD2, from the bag).

## Now (Wed 21 Oct, 11:21): rig day done (08:23-11:05), scored, reviewed; report docs/reports/rig_2026-10-21.md
- 24 launches, 17 flown (QUAD1 out: three quads QUAD2/3/4 on plates 0/8/4, then 4 two-quad attempts). Registry
  RIG-1021-* + reviewer corrections (bak46-48 in the scratchpad, cmp-checked). Nothing committed yet (asks Wesley).
- No hold: the integral never runs -> 5.5-6.6 cm constant offset. 10 s hold: offset 0.8-1.8 cm but the ring leans
  4.4-6.1 deg (the integral's ~0.8 N sideways force). The bias keeps one direction relative to the ring/carrier set
  (weak; the ring mocap centre 1.7 cm off explains about a third).
- QUAD2's magnet let go 6 times (lifts); 8 of 13 ARM refusals QUAD2 FC !ERR*. Two quads: no circle completed.
- New, uncommitted: docs/rig/wed_2026-10-21.md, configs/rig/two_quad_{drop55,noxy,nohold}.yaml, tools/rig_summary.py
  (title says "detach mode"), docs/reports/rig_2026-10-21.md. Thesis: Wesley edits v3_full; question sheet
  ~/mdc_wt/thesis/docs/thesis_drafts/INPUT_intro_to_method.md (uncommitted, protocol paragraph removed).

## Now (Sun 18 Oct, 06:35): overnight done, scored; morning report docs/reports/morning_report_2026-10-18.md
- Batches b8uawb0oz (night, R2174-R2236) and b3c2thf2x (welded QUAD4, R2237-R2240) finished 06:30; nothing running.
- Registry to R2240 + corrections (bak42-45 in the scratchpad, each cmp-checked).
- 0.55 m bands FROZEN (card 2026-10-16_bands_1m, `tools/n2_bands.py --rod 0.55`, results/analysis/n2_bands_055_2026-10-18.json).
- No-floor rev 3: F3 fails once (R2199, 0.6 deg); reviewer not supported; stays off. Decision for Wesley (re-card F3 or park).
- A critic must-fix 1 met: stall exit with QUAD4 folded in, 7/7 rig-twin joins. Legacy-world QUAD4 configs fly
  `legacy:=true` and sim_legacy.yaml has no A (A off in R2172/R2173/R2192/R2193/R2200/R2237-R2240): decision (launch default).
- Radius wind-up: the integral saturates at 2.14 N (~99 s), ring unchanged; the planned pull keeps it at 0.
- Joins (exploratory): twin false proof R2228 (twin waives plate motion); orbit joins drop the ring 3/4 (cause open);
  TB14 x2 (R2235/R2236). m1c_hover_2.yaml does not exist (17 of 18 flew).
- Decisions waiting (morning report): A on the rig after the should-fixes; no-floor F3; A in sim_legacy.yaml; orbit drops; merge.

## 14 Oct rig day in short (detail: registry RIG-1014-*, `git log -p` of this file)
- 46 launches: 8 passed, 11 failed in flight, 11 arm refusals (QUAD1/QUAD2 FC attitude drift), the rest void.
- t5_2 crashed (three-carrier lift bobbing with starved loops; drop_on_loss now on in attach). lt_13 F1 PASS after the
  load cuts and the lean rclpy switch.
- His pickup failed on contact time (t5_4, t5_5, t6); floor_join flown t6f..t6h5_3. t6h_4: the magnet caught plate 9
  and our old proof's taut check rejected it (fixed by WP3).
- Two operator DISARMs with QUAD4 ~0.3 m up on its own landing (fixed by WP8: it now disarms alone a few cm up).
- Notes for Tejen: docs/tejen/notes_2026-10-14.md; his logs zipped at ~/tejen_attach_logs_2026-10-14.zip.

## Rig sheets and twin evidence
- 14 Oct sheet: `docs/rig/wed_2026-10-14.md`. Thursday's sheet is written from the plan's status on Thursday (anything
  unfinished keeps 14 Oct names and plate numbering). Every terminal: `w5` (main tree, ROS_LOCALHOST_ONLY=1,
  ROS_DOMAIN_ID=23).
- Twin evidence for the rig: the claim batch on rig-wed (R1638-R1681) and the 12 Oct batch on week5 (R1682-R1713,
  docs/reports/morning_report_2026-10-13.md). Lab PC free; scripts for a lab batch in `~/handoff_2026-10-12/`.

## Open for Wesley
- After Wednesday: the T6 amendments A1-A4 (explained again with Wednesday's numbers; decisions.md 12 Oct).
- After tonight: Tier A's announced half (the measured soft release does not unload the leaver).
- From the offline analyses (decisions.md OPEN 12 Oct): the rig null, the stretch threshold, the rig ramp pair.
- The counter-swing filter: not now (docs/analysis/counter_swing_2026-10-12.md).

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, evenings only, one tree at a time. Check idle first (`who`, load, containers). Each batch
  runs from a script in a fresh `env -i` shell sourcing ONE install; start it detached; kill exact PIDs only.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt|tb14 RUN --rev N [--check]` (week5). It prints one LF row:
  back up the registry, append the row yourself, then diff the old lines. Wrapper: `$CLAUDE_JOB_DIR/tmp/sc.sh`.
- **Permissions:** deletions (branches, worktrees), git push and switching the main tree are refused for the
  assistant. Write a script for Wesley.
- **Untracked local files:** `.claude/*` and `docs/claude-setup-proposal/`. Edit them in place; never `git add` them.
- **Timestamps:** read `date` before writing any clock time.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.
