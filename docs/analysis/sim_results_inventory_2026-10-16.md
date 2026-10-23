# Simulation results: what is ready to present, what is stale, what to fly (plan B2)

Written Thu 15 Oct, 23:20 AEDT, on `overnight-1015` (HEAD bad76e3). Plan row B2 of
`docs/plan_after_rig_2026-10-15.md`. Desk work only: no runs. One recomputation as a spot check. Tonight's
traj3 batches are placed in section 5 and not scored here.

Sources: the registry (1692 rows), 828 run manifests from R0900 on, the experiment cards, `docs/decisions.md`, the
morning reports, `docs/analysis/`, `docs/figures/PROVENANCE.md`, `tools/thesis_sim_tables.py`, and the thesis
worktree `mdc_wt/thesis` (branch thesis-rig, 5919279, 12 Oct, 122 pp).

## 0. Verdict

- **Ready to show** (to the supervisor now; for the thesis after a cheap re-fly on the same tree): Tier A T3, the
  W4/W5 bands with their checks on the same tree, the 8 Oct twin pairs (as agreement), drop-and-land, and
  disarm-alone.
- **Stale in configuration, not just in code:** every table and figure that `thesis_sim_tables.py` and
  `thesis_sim_figures.py` make. They were flown on 4-5 Oct (week4, 413-419 uncommitted files) with the planned pull,
  a 45° hand-over and the old carry planner. The rig no longer flies that stack.
- **Stale in mechanism:** every join result from before 14 Oct (T11, F14, the full M1). The attach confirmation was
  rewritten on 14 Oct (co-motion proof, bottom hold, Tejen's review).
- **Never flown:** Tier B (F12b), T12 under a card, and bands for the 1 m cables. T10 has never been computed, though
  its columns are already logged.
- **Cost to make the twin side of Chapter 4 thesis-grade:** about two lab evenings at 4 slots (section 7).

## 1. Found while checking (rig-relevant first)

| # | Finding | Why it matters | Suggested action |
|---|---|---|---|
| 1 | The frozen W4/W5 bands come from 0.55 m rods with four quads. Next week's rig has 1 m cables and maybe bigger discs. Only three-carrier carry and attach worlds exist with 1 m cables. | A release flown on the new hardware has no band: it becomes an unscored "after" row in the ledger. | Freeze bands on worlds with 1 m cables before the sheet flies them (N2). |
| 2 | T8b's twin column for L5 was to switch to R1712/R1713 (13 Oct report). Those runs flew `int_mode model`. The rig's L5 (r303_4) and R1140/R1145 flew `reference`. | Different cell; the numbers agree anyway (6.04-6.34° against 6.0-6.4°). | Keep R1140/R1145 for L5. File R1712/R1713 under the W4 cell. |
| 3 | T11 and F14 describe the contact-and-lift proof. Since eacab58 (14 Oct) the co-motion proof is the default everywhere. | The thesis would document a proof the rig no longer uses. | Re-fly T11 on the co-motion tree and rebuild F14 from it (decision 2). |
| 4 | No rig row for K3/K4/K7 (carry) or L8/L9 (W4/W5 releases) flew on 14-15 Oct. | T7, T8 and T13 depend on rig days that are now scarce. The twin tables carry more of the thesis. | Section 8. |
| 5 | `results/` (34 GB) exists only on the laptop (SESSION_STATE). Every run cited here lives there. | One disk failure loses every table. | Back it up before the freeze batch (decision 12). |
| 6 | Spot check: `release_bands.py metrics` on R1712, R1707 and R1686 reproduces the registry rows exactly. All 75 run directories cited here exist with `metrics.json`. | The registry numbers below can be trusted as recorded. | None. |

## 2. How staleness was judged

For each result I took the tree from the manifest (`git_sha`, `dirty_files`) and diffed the flight code from that
SHA to HEAD: `src/mpc_planner`, `tracker`, `dissipative_planner`, `fleet_manager`, `bringup/config`,
`simulation_communication`, `drone_magnet`, `tejen_*`. Where a default might have moved, I compared the planner and
tracker parameter dumps.

| Label | Meaning |
|---|---|
| CURRENT | Tree at or after week5 70d525f. Since then, only logging or switches that default off have changed on the path under test. |
| DRIFT | Code on the path changed but the mechanism did not. A re-fly on the same tree should keep the numbers. |
| STALE-CFG | A default the result rests on has changed since: pull source, rod plan, integral or planner mode. |
| STALE-MECH | The mechanism under test was rewritten. |
| LEGACY | Flown on the earlier plant, before the rig twin (5 Oct). Appendix only, and labelled as such. |
| UNCOMMITTED | The manifest records 10 or more dirty files, so the tree cannot be rebuilt from its SHA. |

Code events behind the labels:

| Date | Commit(s) | Change | What it makes stale |
|---|---|---|---|
| 5 Oct | (decision) | The rig twin becomes the simulation; the old worlds move to `old_worlds/`. | Everything earlier becomes LEGACY. |
| 7 Oct | common.yaml | Measured pull (`cable_source mocap`); rods planned and handed over at 60°. | Carry and detach runs before 7 Oct. |
| 8 Oct | 57912f4, 896aff2, c142662 | Our planner (detach mode) by default, detection 0.06, drop_on_loss, the model integral on every run. | Runs before 8 Oct. |
| 9 Oct | bfbd181, cb09b39 | Newcomer defaults; the freed drone is disarmed alone. | Attach and LAND runs before 9 Oct. |
| 10-11 Oct | b7bb2cb .. c53ae0c | Combined M1 (`partner_contact`), contact_still, the co-moving take-back, the release guard. | Attach runs before 11 Oct. |
| 12 Oct | 29b2d7b | Take-back race fixed (week5 70d525f). | Tilt-cut rows on rig-wed. |
| 14 Oct | eacab58, f035625, 0e47f9b, 76180f8 | Attach confirmed by co-motion (the default; contact_lift stays selectable); bottom hold (rig); Tejen's review points. | Every join result (STALE-MECH). |
| 14 Oct | 3297746, dade70d | A departed drone stalled at most 10 cm above its rest counts as down; voltage fallback; his takeoff hold; arming. | Disarm-alone rows (DRIFT). |
| 15 Oct | 1057c7e, ab6fd18, 8d2036c, bad76e3 | attach_z_lowpass, his match dwell, drop_takes_back: all typed-only or off. | None. |
| 15 Oct | cca8803, 6856cf8 | 1 m cable worlds (new files). | None. |

Read-back of the 4-5 Oct era (R1013, R1033, R1038, R1068): mode mpc, hand-over 45°, `cable_source model`, z_ki 0.
Read-back of today's default (R1723): our planner, 60°, `cable_source mocap`, `int_mode model`.

Since 70d525f the carry and detach path has changed only in these places:
- the typed attach angles (now clockwise);
- the departed-drone "down" rule;
- a drop now turns a newcomer's coil off;
- the visualisation runs every fifth tick;
- the next-tick throttle (off).

So carry and detach results on 70d525f count as CURRENT or DRIFT.

## 3. Inventory by topic

Reviewer and card refer to `docs/experiments/<card>.md`. Numbers are copied from the registry or the card. Ranges are
over the runs listed.

### 3.1 Detach (RQ2, Section 4.2)

| # | Result (one line) | Runs (n) | Card; reviewer | Tree | Label | Thesis place | Missing for thesis-grade |
|---|---|---|---|---|---|---|---|
| D1 | Unannounced hover release with a 120° gap: Δθ is 5.4-6.2° on the measured pull against 17.0-17.2° on the planned pull (Tier A T3). | R1706-R1711 (3+3) | tier_a_crossover rev 2; SUPPORTED with corrections | week5 70d525f, clean | CURRENT | F10b, T9, §4.2 text | The card calls the batch exploratory: re-fly 3+3 on the freeze tree; observe the card's "must not say" list (one twin cell, no rig-sized claim). |
| D2 | W4/W5 cells (unannounced, model integral, freed UAV disarmed alone): twin Δθ 5.74-6.67° hover and 4.87-5.91° circle. Bands [1.75, 14.51] and [1.00, 13.20]. | R1459, R1460, R1626; R1457, R1627 | freeze_2026-10-14 (signed 11 Oct) | rig-prep c2279cf, week4 a25c40d, clean | DRIFT. Week5 checks R1686 (4.61°) and R1687 (7.36°) are inside their bands. | tab:ledger_bands (done), T13, F16 | A rig row (none flew). The bands do not cover 1 m cables (finding 1). |
| D3 | The 8 Oct twin pairs for L4-L7: twin Δθ 6.0-7.1 / 6.0-6.4 / 14.6-17.4 / 4.0-5.4°; rig over twin 1.71, 0.85, 1.05, 1.70. | R1207/R1212, R1140/R1145, R1206/R1211, R1208/R1213 | drop_and_land, unannounced_detach; the P6 reviewer on T8 | drop-port 700a77b, clean; R1140/R1145 on week4 0e91d86 (14 dirty files, with different diffs) | Matched to that rig day's code by design: fine as agreement. | T8b twin column, fig:twin_rig_parity, fig:rig_detach, fig:rig_circle_detach, T13 agreement row | Nothing for agreement. Label R1140/R1145's diffs; see finding 2. |
| D4 | Drop and land: after a second loss or a 240° gap, the trackers' tilt request drops the ring 0.21-0.25 s after the loss. Every drone is down in under 9 s, drone tilt ≤ 15°, climb ≤ 0.18 m. One release on 1/3/6/9 does not drop. | R1191-R1198 (D1, D2, D4, D5, 2 each); R1181/R1186 (D3); R1155/R1156 (B0) | drop_and_land v2; SUPPORTED 8 Oct | safety-defaults 7ed29c9 (D3 on ca6457f), clean | DRIFT: the drop now also covers attach; the carry trigger is unchanged. | §4.3 fault coverage, App K | Re-fly D1 and D4, 2 each, on the freeze tree. Keep the reviewer's caveats: B0 is not one-variable; "down" means z < 0.15 m. |
| D5 | The freed UAV is disarmed alone, about 0.2 s after reaching its rest and before LANDED. The registry has 16 supporting rows and 1 n/a; the skeleton's "17 of 17" needs a recount. | R1304-R1459 (17 rows) | departed_disarm_alone; SUPPORTED 9 Oct | rig-prep cb09b39 .. c2279cf | DRIFT (3297746 changed the "down" rule) | fig:disarm_lanes, fig:departed_landing | Comes with the W4-cell re-fly; the rig rows. |
| D6 | App tab:detach: the resize with ε 0 peaks at 2.8-4.0° and is back in band in 0.9-1.0 s; ε 0.025 gives 3.2° and 2.6 s. | R0939; R0997, R1007, R1012, R1029, R1034 | thesis_figure_plan (no card) | week4 5efe16b, 418-419 dirty | STALE-CFG, UNCOMMITTED | App tab:detach, fig:detach (R1012) | Replace with the W4 cell and Tier A on the freeze tree, or cut. |
| D7 | Announced soft release (ramp v1): peak 2.5-2.6° against 3.3° plain on 1/3/5/9; 3.3-3.5° on 1/3/6/9. | R1134/R1137 vs R1128; R1131/R1136 | soft_detach v2 (critic: rewrite); no reviewer | week4 0e91d86, 11-12 dirty | STALE-CFG | F12a ramp pair | Tier A A3/A4 waits for your word: the measured soft release does not unload the leaver. |
| D8 | The twin does not reproduce the r0013 magnet slip: 5.5° in the twin where the rig capsized. | R1125 (1) | soft_detach | week4 0e91d86 | Limitation (mechanism absent) | §4.4 pattern, F11 | None. |
| D9 | OCP resize against the network hand-off, 4 to 3 in a circle: settled tilt about 1° against 26-48°. In SIL the network aborts at 53-61°. | R0108/R0109, R0111-R0114; SIL R0496-R0505 | September cards | September trees | LEGACY | App F, tab:detach_ab | Label it as the earlier plant. No re-fly: the network is leaving week5. |
| D10 | Layout study, 4 to 3: even ring against 1/3/5/9 against 1/3/6/9. | R0559-R0572, R0554/R0556, R0582/R0584; twin R1076 | detach_layout_mode; SUPPORTED with corrections (25 Sep) | legacy | LEGACY | sim_detach \redo box | Superseded by Tier B (N1). |

### 3.2 Attach and rejoin in the twin (RQ3, Section 4.3)

| # | Result (one line) | Runs (n) | Card; reviewer | Tree | Label | Thesis place | Missing for thesis-grade |
|---|---|---|---|---|---|---|---|
| A1 | Combined M1, the hover lines: attach with contact-to-proof 0.72-1.08 s, throttle step 0.0-0.6 %, ring tilt 1.83-2.41°, rod clearance ≥ 0.213 m; take-back tilt 1.66-2.50°; no-catch rejects every contact; LAND during the proof. | R1644, R1645, R1650-R1652; R1642, R1643, R1648, R1649, R1653 | combined_m1_contact rev 10 §6.3a: hover lines SUPPORTED, overall NOT SUPPORTED. You claimed the hover lines on 12 Oct. | rig-wed ba40bd0, clean | STALE-MECH (contact_lift proof, mat start with pickup) | T11 | Re-fly on the co-motion tree, 2-5 per line, then the reviewer. Or keep it labelled as the earlier proof (decision 2). |
| A2 | The failed lines, re-flown on week5: no-catch circle and LAND take-back circle pass 4 of 4. Push abort and re-attach work 4 of 4, but rod clearance is 8.0-9.5 cm against the 0.10 m bar (the bar measures the push). | R1694, R1695, R1698, R1699; R1688, R1689, R1692, R1693 | same card, exploratory | week5 70d525f | STALE-MECH | T11 "re-fly" rows | A new table on the freeze tree. The push-abort bar is still open (decision 11). |
| A3 | Co-motion confirmation in the twin: contact, his lift signal, proof, fold-in at plate 9, a clean landing. | R1718 PASS, R1719 FAIL (led to rev 2), R1720, R1721, R1722 PASS | attach_comotion_rig (rig card); twin smoke tests | eacab58 .. cd54f29, 1-266 dirty | CURRENT; one run per tree | T11, F14 | A twin card and a claim batch (≥ 2 per line). The rigid twin tether cannot go slack, so the rig thresholds stay on the rig side. |
| A4 | Full M1 (release, pickup, drop, rejoin): hover end to end 2 of 2. Circle reattached 7 of 11 across trees (6 of 6 on m1-*, 0 of 1 on rig-wed, 1 of 4 on week5). | R1655, R1685; R1523, R1541, R1564, R1579, R1631, R1633, R1656, R1690, R1691, R1700, R1701 | no card (T12 still to come); OPEN line, 12 Oct | m1-*, rig-wed, week5 | STALE-MECH | T12 | A card, then 2 hover + 2 circle on the freeze tree. The circle misses are his gate on a moving plate (in gate 66-82 % of the time). |
| A5 | Tilt cut on a newcomer under his controller: the fault is caught 0.23-0.25 s after the injected upset and QUAD4 is disarmed within 0.022 s; with the switch off it stays armed. | rig-wed R1660-R1678 (NOT SUPPORTED: the take-back race); week5 R1702-R1705 (4 of 4, no take-back) | quad4_tilt_cut rev 4 | ba40bd0; 70d525f | DRIFT (magnet_gate.py changed for co-motion) | §4.3 fault coverage, App K | The reviewer on the week5 rows; 2 per arm on the freeze tree. The switch stays off on the rig. |
| A6 | Our own approach (twin and thesis only): on a held plate it supports 2 of 2, on a moving plate it fails 2 of 2. The measured-tip loop brings a 4 cm rod error within 1 cm in 1.2-2.1 s, and to 0.1-6.3 mm at 3.6-11.7 s (the reviewer's split of the card's "1-6 mm in under 2 s"). | R1468-R1472; R1330-R1454 (figure: R1394/R1395, R1400/R1406, R1401/R1407) | twin_r9; approach_tip_feedback rev 5, SUPPORTED with corrections | rig-prep 9d52656, 1a4b328; 1bbe1f8 | DRIFT: approach.py is unchanged since 1bbe1f8; never flown on the rig | fig:tip_feedback, App | As is, labelled twin-only. |
| A7 | LAND in every phase lands and disarms every drone, including the freed drone and the newcomer; a newcomer fault lands the fleet. | R1413-R1446 (2 per phase; R1438 tipped); R1248-R1456 | land_every_phase (re-fly SUPPORTED with corrections); newcomer_land_abort (merge-gate reviewer SUPPORTED with corrections, 10 Oct); newcomer_fault_scope (reviewer WEAK 9 Oct, corrections taken) | rig-prep 8588745, aa010f0, clean | DRIFT for the carry and detach phases; STALE-MECH for the attach phases (his controller flies them now) | fig:land_phases, App K | Optional: a combined-M1 LAND re-fly of about 40 runs (thesis plan §6). |
| A8 | Legacy rejoin: M1 rejoin and M1 full with the partner mission (SUPPORTED), the M2 ground join, and the storyboard on the 0.6 kg ring. | R0685/R0686, R0696/R0697, T0018/T0019, R0209 | September cards | September trees | LEGACY | App F (fig:attach), \redo boxes | Cut, or label as history. |

### 3.3 Carry and robustness (RQ1, Section 4.1, and the appendices)

| # | Result (one line) | Runs (n) | Card; reviewer | Tree | Label | Thesis place | Missing for thesis-grade |
|---|---|---|---|---|---|---|---|
| C1 | Trajectories with 3 and 4 UAVs (hover, line, circle, figure-8, spin): mean error 0.5-0.9 cm, max 0.7-2.7 cm, tilt 1.2-1.6°. | R1013/R1023, R1018/R1032, R0974/R0979/R1016/R1027, R1033/R1037, R1035/R1036, R1068/R1070, R1069/R1071 | thesis_figure_plan (no card) | week4 5efe16b, 419 dirty | STALE-CFG, UNCOMMITTED | App tab:traj, fig:traj (R1033/R1035), fig:flight (R1013) | Replace with the traj3 claim (3 carriers). Re-fly or drop the 4-UAV rows. |
| C2 | Steady errors: pinning node 0 takes the hover offset from +4.8 to +0.9 cm. The model integral rejects a 0.6 N downward and a 0.4 N sideways push (steady 0.1-0.2 cm against 7.6-8.4 cm). | R0841/R0844, R0960/R0978, R0832, R0974/R0979; R0961-R0986 | offset_free_innovation (PARTLY), z_int_model (weak, corrections applied) | week4 5efe16b, 413-418 dirty | STALE-CFG, UNCOMMITTED | fig:tracker_x0, fig:disturbance, tab:disturbance; App tab:x0, tab:gain | Re-fly on the freeze tree (N3). |
| C3 | Robustness to a wrong model: mass ±10/20 %, rods +5 cm (measured or not), radius +2 cm. Mass +20 % misses by 25.8 cm (the integral gate, a stated limit). | R1038-R1067, R1013/R1023 | thesis_figure_plan; reviewer applied | week4 5efe16b, 419 dirty | STALE-CFG, UNCOMMITTED | App tab:robust | Re-fly (N3). |
| C4 | Landing: unwind + ramp + idle tips 0 of 18, against 2 of 21 with a straight descent. | R0987-R1034 | land_matrix, land_fix | week4 5efe16b | STALE-CFG, UNCOMMITTED; a rigid-rod artefact | App tab:land, fig:landing | Low priority: keep it as a twin-artefact appendix, or cut. |
| C5 | Measured pull in the twin: no abort or tip-over; landing lean 1.6-2.4°. | R1104-R1116 | cable_mocap (critic: rewrite) | week4 8f4fb2b, 171-173 dirty | UNCOMMITTED (the config is now the default) | none | Superseded by traj3. |
| C6 | The lag mechanism: the unstamped plan makes the lag. Adding 0.035 s to every reference reproduces the rig's extra lag (H1); a shared time base removes it (rung b). Parked: the decision is to report the lag. | R1172/R1173; R1165/R1166, R1176/R1177 | lag_h1, time_cascade; SUPPORTED (H1, rung b, figure-8) | time-cascade 1df4330/5bc43c9; week4 8ef5871 | Side branch, not on week5 | §4.5a, §4.4 pattern | Label it as a side branch. N4 measures latency on the current tree. |
| C7 | The supervisor's half-speed circle: at half speed the along-path error stays (0.91 to 0.87 cm) and the lag doubles (A0 0.069 to 0.134 s). | R1228/R1229, R1230/R1231 | half_speed_circle (falsified in the twin) | rig-prep 8b10be2, clean, current defaults | DRIFT | The RQ1 lag paragraph | traj3's three speeds extend it. The rig half (W1/W2) never flew. |
| C8 | Steady wind on the circle: the model integral takes the error from 17.9 to 3.9-4.0 cm. | R0864, R0929, R0947 | load_ext_force (critic: not ready); PARTLY | week4 5efe16b, 408-413 dirty (3-4 Oct) | STALE-CFG, UNCOMMITTED | none | Fly the wind world on the current stack (N3). |
| C9 | A command delay of ≥ 0.05 s makes the takeoff unstable (tilt 40-85°). | R1097, R1098, R1100, R1102 | none (exploratory, 7 Oct) | week4 8f4fb2b, 169-171 dirty | STALE-CFG | none | N4. |
| C10 | Tonight: traj3 (3 carriers × 5 shapes × 3 speeds × 2) and traj3_L100. | R1723-R1752, then L100 | exploratory (a falsifier in each config) | d4f8ce8, clean | CURRENT | Section 5 | Not scored here. |

### 3.4 Rig twin fidelity (Section 4.4)

| # | Result | Runs | Status | Thesis place | Missing |
|---|---|---|---|---|---|
| F1 | 8 Oct parity: rig over twin Δθ 0.85-1.71. The dip is 1.4-2.5 times the twin's in 3 of 4 pairs. The horizontal error before the release is 1.9-3.1 times the twin's. | D3 | Figure done; agreement only | fig:twin_rig_parity, T13 agreement row | Fill T13's agreement row now (the data exists). |
| F2 | Frozen W4/W5 bands, inside on their week5 checks. | D2 | Frozen; no rig row | tab:ledger_bands (done), T13, F16 | A rig release; new bands for 1 m (N2). |
| F3 | Hover, twin against the 1 Oct rig: the cap reproduces the rig's height shortfall. | R0834/R0836, R0838/R0839 (3 Oct) | STALE: from before the tracker fix (the caption says so) | res_carry tab:sim_vs_rig | Cut it, or replace with T13 carry rows (rig K2/K6 against the twin on the same config). |
| F4 | Twin limitations, each measured: no magnet holding limit (D8); the rigid rod tips QUAD4 on landing (TB14, 12 of 48 landings, R1479-R1578); no counter-swing in the twin (rig 9 of 9 survivors; `docs/analysis/counter_swing_2026-10-12.md`); horizontal error before a release 5.3-6.8 cm on the rig against 1.7-3.3 cm in the twin (the 8 Oct pairs). | as listed | Presentable as is | §4.4 pattern paragraph | None. |
| F5 | The twin flies the rig's affine thrust map with per-drone offsets 0.181-0.187. | all twin runs | A method | §3, tab:params | None. |

### 3.5 Methods and thesis status (thesis-rig 5919279, 12 Oct)

- **Chapter 4 as assembled:**
  - res_carry, res_steady and skel_carry (RQ1);
  - sim_detach and skel_leave (RQ2);
  - sim_attach and skel_join (RQ3);
  - ledger and skel_ledger (§4.4);
  - discussion.

  Placeholders:
  - 7 `\pendingfig`: F9, F12, F15, F16, rig_rejoin, rig_departed, rig_photo;
  - empty tables: T10, T12, T13;
  - rig rows still to come: T7 K3/K4/K7 and T8 L8/L9.

  There are 21 `\redo` boxes (12 in Chapter 4, 7 in the appendices, 2 in the conclusion), and each names a legacy
  result.
- **Twin-built figures in the thesis:**
  - current enough: twin_rig_parity, rig_detach, rig_circle_detach, departed_landing, disarm_lanes, land_phases,
    tip_feedback;
  - stale: flight, detach, tracker_x0, disturbance, trajectories, landing (all 4-5 Oct), proof_timeline
    (contact_lift).
- **Generators:**
  - `tools/thesis_sim_tables.py` holds hard-coded run lists from 4-5 Oct (TRAJ, ROBUST, DETACH, STRAIGHT). Update
    the lists and re-run after the freeze batch.
  - `thesis_join_figures.py` and `thesis_leave_figures.py` exist only on thesis-rig; merge them before building F10b,
    F12 and F14.
- **T10 needs no runs:** the planner's `log.csv` already logs `solve_status`, `res_stat`, `res_eq` and `solve_ms` at
  every tick. The gap to an offline SQP solve needs the QP dumps (`MAX_QP_DUMPS` 5 per run).

## 4. Readiness, ranked

| Rank | Item | Readiness | What it takes | Lab cost |
|---|---|---|---|---|
| 1 | D1 Tier A T3 (F10b) | Needs a re-fly on the final tree (cheap) | 3+3 runs, reviewer on the table | 30 min |
| 2 | D2/F2 W4/W5 bands, D3/F1 8 Oct pairs | Ready | T13 agreement row from existing data | none |
| 3 | A1-A3 T11 on the co-motion proof | Needs a re-fly | Twin card, about 19 runs, reviewer | about 75 min |
| 4 | C10 → T7t, the twin carry table (traj3 claim) | Needs more runs (claim) | Card with bars, 2 per cell on the freeze tree, reviewer | about 30 min |
| 5 | D4 drop and land, D5 disarm-alone, A5 tilt cut | Needs a re-fly on the final tree (cheap) | 2 per arm, reviewer on the week5 tilt rows | about 40 min |
| 6 | Tier B → F12b, T9 | Needs runs (never flown) | Card, worlds, about 12 runs | about 20 min |
| 7 | C2, C3, C8 steady errors and robustness | Needs a re-fly (STALE-CFG) | N3, about 36 runs, then regenerate four tables | about 35 min |
| 8 | A4 T12 full M1 | Needs more runs (card) | 2 hover + 2 circle | about 20 min |
| 9 | C7, C6 lag | Ready as exploratory or side branch | Label the tree; traj3 extends it | none |
| 10 | A6, A7, D9, F4 | Ready for the appendix, labelled | None | none |
| 11 | C1, D6, C4 appendix tables and figures | Not presentable as current | Replaced by 4, 1 and 7, or cut | — |
| 12 | A8, C5, C9, D10, F3 | Not presentable, except as history | Cut, or label | — |

## 5. Where tonight's batches fit (not scored here)

| Batch | Runs | Fits | Becomes thesis-grade after |
|---|---|---|---|
| traj3_{circle,spin,fig8,linex,orbit}_v{125,200,300} ×2 | R1723-R1752, tree d4f8ce8 (clean, CURRENT) | (1) Replaces App tab:traj and fig:traj (the 3-UAV rows) with today's stack. (2) A new twin carry table and figure for §4.1 (T7t, F8t): the rig flew one speed per shape, the twin gives the speed envelope. (3) Extends the half-speed answer (C7) to lag against speed for five shapes. (4) Frozen as bands before a three-carrier rig carry, it gives the ledger a carry row. The 0.3 m/s falsifier sets the rig's speed cap. | A carry card with bars written after the exploratory read; 2 per cell on the freeze tree; the reviewer. Tonight's runs stay exploratory. |
| traj3_L100 (circle v125/v200; fig8, linex, orbit at v200) | after R1752 | B3: a design comparison for the supervisor and §4.5 (cable length), and the baseline for N2. One variable: the world (cable_len 1.0). | The same card as traj3, with L100 as one arm. Caveat in every caption: the twin's links are rigid and the rig's tethers go slack. |

## 6. Chapter 4 figures and tables: what to make, from which runs

Every figure goes through the thesis-figure skill: 6.27 in, Computer Modern, look rounds, a PDF audit and a blind
critic. Run ids stay out of the text; provenance goes in `configs/thesis_figures.yaml` and
`docs/figures/PROVENANCE.md`.

**Figures**

| ID | Section | What it shows | Twin source | Status | Next | Order |
|---|---|---|---|---|---|---|
| F10b | 4.2 | Crossover: Δθ, measured against planned pull, with rig dots as they come | R1706-R1711 (then the re-fly) | DATA | Build the panel now; swap in the re-fly runs | 1 |
| F14 | 4.3 | Proof timeline | R1644/R1645 (contact_lift) | STALE | Rebuild from the T11 co-motion re-fly | 2 |
| F8t (new) | 4.1 | Twin carry envelope: path error, lag and ring tilt against speed for each shape, with a 1 m row | traj3 claim | FLY | Card, re-fly, build | 3 |
| F12 | 4.2 | (a) one-variable pairs: pull source, ramp, detection off, integral mode; (b) Δθ against support margin | Tier A, Tier B, D7 | FLY | N1, N5 | 4 |
| fig:detach | 4.2 / App | Planned tensions, tilt and height through a resize | R1012 (stale) | REBUILD | From an A1-arm run of the W4 cell on the freeze tree | 5 |
| fig:flight | 3 / 4.1 | One flight from the floor | R1013 (stale) | REBUILD | From a traj3 claim hover run | 6 |
| fig:tracker_x0, fig:disturbance | 4.1 steady | x0 box; pushes | 4 Oct (stale) | REBUILD | After N3 | 7 |
| fig:traj | App | Plan view of the circle and the figure-8 | R1033/R1035 (stale) | REBUILD | From the traj3 claim | 8 |
| fig:landing | App | Landing push and lean | R0960/R1013 (stale) | CUT or label | — | — |
| fig:twin_rig_parity, rig_detach, rig_circle_detach | 4.2 / 4.4 | Rig against twin, 8 Oct | D3 | DONE | — | — |
| F10a, F11 | 4.2 | Releases; magnet anatomy | D3 ranges; D8 | DONE | Add L8/L9 when they fly | — |
| fig:departed_landing, disarm_lanes | 4.2 / App K | Freed-UAV landing | D5 | DONE (rig rows pending) | — | — |
| fig:land_phases, tip_feedback | App K, App | LAND matrix; tip loop | A7, A6 | DONE (DRIFT) | Label the trees | — |
| F9, F15, F16, F7, F8 | 4.1-4.4 | Rig figures | — | RIG | Not simulation | — |

**Tables**

| ID | What | Twin source | Status | Next |
|---|---|---|---|---|
| T13 | Parity on Δθ | 8 Oct agreement rows: D3; W4/W5: rig | Partly fillable now | Fill the agreement row now |
| T10 | Switch numerics: status, KKT, solve time, throttle step | Planner `log.csv` of the freeze-tree runs, plus the rig logs | No runs needed | Offline script |
| T11 | Join claim | A1-A3 | REFLY | Freeze batch, join tree |
| T7t (new) | Twin carry: shape × speed, error, lag, tilt, n | traj3 claim (+ L100 row) | FLY | Card |
| T9 (App J) | Levers | Tier A, Tier B, D4, D7 | FLY | N1, N5 |
| T12 | Full M1 | A4 | FLY | Card; hover 2, circle 2 |
| T8b twin column | L4-L7 | D3 | DONE | Finding 2 |
| tab:ledger_bands | W4/W5 | D2 | DONE | Bands for 1 m if N2 runs |
| App tab:traj, x0, gain, disturbance, robust, land, detach | Appendix tables | 4-5 Oct | STALE | Re-fly (traj3, N3, W4 cell) and re-run `thesis_sim_tables.py` with new lists |
| tab:sim_vs_rig | Hover, twin against rig | F3 | STALE | Cut, or move to T13 |
| tab:detach_ab, tab:attach_layout | Network | D9 | LEGACY | Keep in App F, labelled |

## 7. The freeze batch (lab, 4 slots; wall times from tonight and 12 Oct)

| Block | Runs | Per run (wall) | Lab time |
|---|---|---|---|
| Carry claim (traj3 cells ×2, L100 arm) | about 40 | 2.5-4 min | about 35 min |
| Steady errors and robustness (N3) | about 36 | about 4 min | about 35 min |
| Detach: Tier A 3+3, W4/W5 cells 2+2, D1/D4 2+2, Tier B about 12 | about 26 | about 4.4 min | about 30 min |
| Join: T11 about 19, T12 4, tilt cut about 10 | about 33 | 8.5-18 min | about 120 min |
| Voids and re-runs (pose timeouts, about 20 %) | | | about 45 min |

That is about 4.5 h: two evenings. Carry and detach code has not changed in substance since 12 Oct, so those blocks
can fly on one tree now. Join code changed on 14-15 Oct and will change again with next week's discs and cables, so
the join block flies on its own tree after the 21-22 Oct rig day. Each table stays on one tree, as the working rules
require.

## 8. New simulation experiments (ranked; about 5 weeks left, rig time scarce)

| # | Experiment | Serves | Design (one variable, n) | Code | Lab |
|---|---|---|---|---|---|
| N1 | **Tier B, the support margin.** | F12b, T9, the N-1 design rule (RQ2) | Largest gap after the release: 120/150/180° on rig plates, plus 135/165° at twin-only azimuths for the curve's shape. Measured pull, unannounced hover release, 2 each; planned pull at 150°, 2. drop_on_loss off at 180° (the gap rule drops), or score the run as a drop. | Configs and 2-3 worlds (`make_carry_worlds.py`); no flight code; card, no critic. | about 20 min |
| N2 | **Bands for next week's hardware.** Time-critical: before the 21 Oct sheet. | §4.4: the only way the next rig day yields scored ledger rows | The cells the sheet lists, flown on the 1 m worlds, 3 each. A four-drone 1 m release world is needed for W4/W5. A carry band needs a carry band rule, which is your word. | World generator only | about 20 min |
| N3 | **Steady errors and robustness on today's stack.** | Replaces four stale appendix tables (C2, C3, C8) with one tree | 0.6 N down and 0.4 N sideways push; mass ±10/20 %; rods +5 cm (measured or not); radius +2 cm; the wind world on the circle. Each with the model integral (default) and with it off, 2 each. | None (the configs exist in `configs/experiments/old/`) | about 35 min |
| N4 | **Latency margin.** | §4.4 "the twin lacks latency", RQ1's lag | `SIM_CMD_DELAY_S` 0 / 0.015 / 0.03 / 0.045 s (≥ 0.05 tipped at takeoff on 7 Oct), on the three-carrier circle at 0.125 and the W4 release, 2 each. Read at the rig latency from P6 item 4. | None (the env vars exist) | about 15 min |
| N5 | **Tier A, the cells that need no new code.** | "What knowing about the release buys", F12a | Detection off on each pull source, 3 each; the 60° circle with measured against planned pull, 2 each: the twin version of the rig falsifier K4. The announced pair waits for your A3/A4 call. | None | about 15 min |
| N6 | **The join under rig-like plate motion.** | RQ3; checks A4 (match dwell) and A5 (z filter) before they fly | A train of WRENCH steps on the attach push world bounces the ring about 0.8 cm at 1.8 Hz during his MATCH/DESCEND. Arms: attach_z_lowpass 0 against 0.8 Hz, and match dwell 1.0 against 0.5 s, 2 each. Measured: time to the first descent, contacts, contact-to-proof time, false proofs. It tests our gates, not the catch: the twin magnet and tether stay ideal. | Config only (the WRENCH steps exist) | about 60 min |

Not recommended in the time left: a twin with slack tethers or a magnet holding limit (the locked decision is
rigid links, and it is a large change); a re-fly of the network comparison (legacy, leaving week5).

## Decisions for Wesley

1. Claim trees: fly carry and detach on one freeze tree now (overnight-1015 once merged to week5), and join on a second tree after the 21-22 Oct rig day. Recommended; the alternative is one freeze for everything around 25 Oct.
2. T11 and F14: re-fly on the co-motion proof. Recommended; or keep the 12 Oct contact-and-lift table, labelled as the earlier proof.
3. The 4-5 Oct appendix tables and figures: re-fly on the freeze tree and regenerate (recommended, about 1 h of lab), or cut them to a dated "earlier stack" note.
4. T8b, L5 twin column: keep R1140/R1145 (the same integral as the rig flight) and file R1712/R1713 under the W4 cell. Recommended.
5. Twin baseline cable length for the thesis tables: 0.55 m (every rig flight so far), with 1 m as a design row. Recommended.
6. Next week's hardware: freeze twin bands on the 1 m worlds for every cell the next sheet lists, before it flies (recommended). Otherwise those flights are unscored "after" rows.
7. traj3: write the carry card with frozen bars after tonight's exploratory read, then fly the claim twice on the freeze tree. Recommended; tonight's runs are not re-scored as the claim.
8. Tier B: fly it at 120/150/180° on rig plates plus 135/165° at twin-only azimuths (recommended), or drop F12b.
9. T12: two hover runs under a card, with the circle reported as exploratory (7 of 11 across trees, 1 of 4 on week5). Recommended over more lab time on the circle.
10. Tier A's remaining cells: fly detection-off on both pull sources now (no new code), and hold the announced pair until you pick its leaver rule. Recommended.
11. Push-abort clearance bar (open since 13 Oct): for the new table, reword it to QUAD4's own clearance during the abort (recommended), or keep 0.10 m and report the fail.
12. Back up `results/` (34 GB, laptop only) before the freeze batch. Recommended.
