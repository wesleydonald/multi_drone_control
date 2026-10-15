# Appendix re-fly: the 4-5 Oct appendix tables and figures on the freeze stack (inventory C1-C4, D6, N3)

Card written Fri 16 Oct 2026, 09:05 AEDT (from `date`), before any run; configs prepared with it (no runs).
Wesley's decision 3 (decisions.md, DECIDED 2026-10-16 thesis inventory line): the 4-5 Oct appendix tables and figures
are re-flown on the freeze tree (week5 4b3560e's flight code, flown from day-1016) and regenerated. Twin only, lab PC.
Background: `docs/analysis/sim_results_inventory_2026-10-16.md` sections 3.3, 4 (ranks 7 and 11), 6 ("Tables" row
"App tab:traj, x0, gain, disturbance, robust, land, detach") and 8 (N3).
**Bars in section 5: FROZEN Fri 16 Oct 09:05, before the runs (set while Wesley is at work, with the recommended answers to section 11 items 1-4; he may change them for later runs only). The optional block (T4, DT-A) waits for his word.**

## 0. What is re-flown, replaced or cut

Generators: `tools/thesis_sim_tables.py` (tables), `tools/thesis_sim_figures.py` (figures). Thesis places from
`mdc_wt/thesis` (thesis-rig 5919279). Every run they read is from 1-5 Oct, week4 5efe16b with 413-419 dirty files
(UNCOMMITTED), mode mpc, hand-over 45 deg, planned pull (`cable_source model`), reference integral.

| Item | Thesis place | Built from (4-5 Oct) | Decision | New source |
|---|---|---|---|---|
| tab:x0 | App, Further tests | R0841/R0844, R0832 (old box); `b2_cap075_fix_n3_x0b0`, `_orbit_x0b0` | RE-FLY | arms X0-H0/H1, X0-O0/O1 |
| fig:tracker_x0 | Ch4 res_steady | R0841/R0960, R0832/R0974 | RE-FLY | r1 of the same arms |
| tab:disturbance | Ch4 res_steady | `b2_cap075_fix_n3_{pushz,push04,mapneg}_x0b0*` | RE-FLY | arms D-* |
| fig:disturbance | Ch4 res_steady | R0962/R0968/R0969, R0961/R0971 | RE-FLY | r1 of D-Z0/ZR/ZM, D-X0/XM |
| tab:gain | App | R0969/R0982, R0976, R0977 | RE-FLY | D-ZM, G-K8, G-S5 |
| tab:robust | App, Wrong model values | R1013/R1023, `b2_cap075_fix_n3_rob_*` | RE-FLY | arms R-* |
| fig:flight | Ch4 res_carry | R1013 | RE-FLY | R-none-M r1 (default controller) |
| fig:landing | App | R0960 (straight), R1013 | RE-FLY | L-S r1 against X0-H0 r1 |
| tab:land | App | 21 straight (R0960-R0986), land_matrix (18), land_fix (18) | RE-FLY (one row; open question 1) | every freeze-tree 0.55 m three-carrier landing |
| tab:traj, 3-UAV rows | App | `b2_cap075_fix_n3_x0b0_lf`, `..._orbit_x0b0{,_lu,_lf}` | REPLACE | traj3 claim (card 2026-10-16_carry_speed_envelope) + R-none-M hover |
| tab:traj, 4-UAV rows | App | `w4_3915_hover_lf`, `w4_even_{line,orbit,fig8,spin}_lf` | CUT (open question 2) | optional arms T4-* |
| fig:traj | App | R0933/R1033, R0935/R1035 | REPLACE | traj3 claim orbit and figure-8 at 0.125 m/s, r1 |
| tab:detach | App | R0939; R0997, R1007, R1012, R1029, R1034 | REPLACE (open question 3) | Tier A A1/A2 (card 2026-10-12_tier_a_crossover rev 3) |
| fig:detach | Ch4 sim_detach | R1012 | REPLACE | Tier A A1 r1 (`ta_a1_r1`) |
| tab:four | none (no `\input`) | R0931-R0939 | CUT | none: drop it from the generator |
| tab:params | (method) | common.yaml + real.yaml | no runs | regenerates as is |

Not in the generators, so not here: the wind circle (inventory C8, N3's last item), the latency sweep (N4).

## 1. Question

Do the appendix results flown on 4-5 Oct (tracker box, push rejection, integral gain and gate, wrong-model robustness,
landing) still hold on the freeze stack, so that each table and figure can be regenerated from one tree?

## 2. Hypothesis

Each table's claim (from the thesis text that reads it) and what the stack change could move:

| Item | Claim as written | What the freeze stack changes under it |
|---|---|---|
| tab:x0, fig:tracker_x0 | Pinning the tracker's node 0 (epsilon 0) removes the hover offset and most of the orbit error of the relaxed box (0.025). | The pull is measured, not planned; hand-over and planned cable angle 60 deg, not 45. The box is a tracker property, so the sign should hold. |
| tab:disturbance, fig:disturbance | Without an integral a constant ring force leaves a steady offset (the cascade stiffness). The integral removes it; the model integral learns a force equal to the applied one and also removes the horizontal error that the target shift leaves under a thrust-map error. | Ring stiffness at 60 deg and on the measured pull differs from 4 Oct: the no-integral offsets and the settling times move. The learned force should still equal the applied one: at equilibrium it does not depend on `int_k_xy`/`int_k_z`. |
| tab:gain | Gain 0.8 lowers the peak but rings longer; the gate held for 5 s after the lift trims 7-8 % of peak and overshoot. | Small differences (n = 1 before). The settling time scales with `int_k` over the real stiffness, and the `int_k` defaults (5.26 / 7.13 N/m) were identified at 45 deg on the planned pull. |
| tab:robust | A mass error moves the hover by about 1.25 cm per percent; the integral removes it inside its 0.25 m gate (+20 % is over it). Measuring the rods absorbs a 5 cm rod error; unmeasured, the tautness gate keeps the integral off. A 2 cm radius error does nothing. All 28 landed. | The measured pull may shrink the mass-error offset; the integral column changes from the target shift (reference) to the model integral, today's default. |
| fig:flight | Creep, pretension, lift to 1.0 m, hold, LAND, unwind, on the default controller. | Hand-over at 60 deg (the figure draws a 45 deg line). |
| fig:landing, tab:land | A straight descent pushes the UAVs outward, and they rock until they land; unwind + ramp + idle removes this (0 of 18 tipped against 2 of 21). | Measured pull and our planner's landing. A tip rate needs about 20 runs per arm, so the straight descent is flown for the figure only (open question 1). |

Prediction: no run aborts, drops or self-detects a release; each claim keeps its sign; each cell stays within a factor
of 2 of its 4-5 Oct value or within the floor of section 5.

## 3. The one variable

**The stack change, common to every arm** (the only lines changed from the 4-5 Oct config; the descriptions name the
source config):
- `mode: mpc` becomes `mode: dissipative` (our planner, detach mode: the freeze tree's carry, as the traj3 claim).
- `handover_elev_deg: 45.0` deleted: default 60, matching the planned cable angle `cable_elev_deg` 60. That one was
  never typed, so the old line flown today would be a 45/60 hybrid that never flew.
- `z_i_max: 0.15` deleted: default 0.3 (Wesley 4 Oct). Each robustness pair then differs only in the integral; the
  old pairs also differed in this line.
- Arms meant as the reference integral (target shift) gain `int_mode: reference`. Without it, `int_mode auto` flies
  z_ki 0.4 as the model integral today.

**Inherited from today's defaults, no line changed:** the measured pull (`cable_source mocap`), detection 0.06 m,
drop_on_loss, departed_land, departed_ground_idle, land_unwind + land_ff_ramp + ground_idle, z_i_max 0.3.

**Kept as on 4-5 Oct:**
- every event time, world and criteria block;
- lift ramp 0.1 m/s (the sim and rig default; the traj3 and Tier A configs type 0.15; the lift is outside every
  window);
- `ff_cap_force: 0.75`: typed, but detach mode never passes it to the planner (section 10, item 2);
- z_ki 0 in the x0 arms. As on 4 Oct, an integral would remove the hover offset the box causes.

Compared on the same tree, arm against arm (one variable each); the 4-5 Oct runs are the named baseline for agreement
only, since the stack changed under them:

| Arm | Config | Against | The one variable | n |
|---|---|---|---|---|
| X0-H0 | `app_x0_hold_eps0` | (base) | hover 1.0 m, three carriers, z_ki 0, epsilon 0 | 2 |
| X0-H1 | `app_x0_hold_eps025` | X0-H0 | `x0_relax_frac: 0.025` (the 1-3 Oct baseline configs never typed it: flown today they would be epsilon 0) | 2 |
| X0-O0 | `app_x0_orbit_eps0` | (base) | orbit r 0.5 m at 0.125 m/s, z_ki 0, epsilon 0 | 2 |
| X0-O1 | `app_x0_orbit_eps025` | X0-O0 | `x0_relax_frac: 0.025` | 2 |
| D-Z0 | `app_pushz_none` | (base) | 0.6 N down on the ring, 15 s; z_ki 0 | 2 |
| D-ZR | `app_pushz_ref` | D-Z0 | reference integral (z_ki 0.4, `int_mode: reference`) | 2 |
| D-ZM | `app_pushz_model` | D-Z0 | model integral (z_ki 0.4, `int_mode: model`) | 2 |
| D-X0 | `app_push04_none` | (base) | 0.4 N along -x, 15 s; z_ki 0 | 2 |
| D-XM | `app_push04_model` | D-X0 | model integral | 2 |
| D-MR | `app_mapneg_ref` | (base) | drone 1's plant offset 0.155 against its map's 0.185; reference integral | 2 |
| D-MM | `app_mapneg_model` | D-MR | model integral instead of reference | 2 |
| G-K8 | `app_pushz_model_ki08` | D-ZM | z_ki 0.8 | 2 |
| G-S5 | `app_pushz_model_settle` | D-ZM | `int_settle_s: 5.0` | 2 |
| R-x-0 (6) | `app_rob_{m+10,m+20,m-20,rod+5,rod+5nomeas,r+2}_z0` | X0-H0 | one wrong typed value (`mis_seed: true`), z_ki 0 | 2 each |
| R-none-M | `app_rob_none_int` | X0-H0 | model integral (`int_mode: model` typed so the planner refuses rather than falls back) | 2 |
| R-x-M (6) | `app_rob_{...}_int` | its R-x-0 | model integral | 2 each |
| L-S | `app_land_straight` | X0-H0 | the landing package as on 4 Oct: `land_unwind`, `land_ff_ramp`, `ground_idle` false (three switches, one package; figure only) | 2 |
| T4-* (optional) | `app_traj4_{hover3915,orbit,fig8,linex,spin}` | traj3 cells | four carriers; z_ki 0 pin also removed (the default controller, as the traj3 claim) | 2 each |
| DT-A (optional) | `app_w4_3915_detach` | Tier A A1 | announced DETACH of drone 1 (plate 9); the 8 Oct pins (detection 0, drop_on_loss off, departed_land off) removed | 2 |

Shared runs:
- X0-H0 is also the ROBUST "none, no integral" cell, the fig:landing "after" panel, and part of the tab:land row.
- R-none-M is also the tab:traj three-carrier hover row and fig:flight.
- D-ZM is also tab:gain's k_i 0.4 row.

`run_experiment.py --check` passes on all 33 configs; the only output is the usual params.py inertia warning.

## 4. Baseline (4-5 Oct; values as `thesis_sim_tables.py` computes them today, which reproduces the thesis tables)

| Table | Cell | 4-5 Oct runs | Value |
|---|---|---|---|
| x0 | hover height error, epsilon 0.025 / 0 (cm) | R0841, R0844 / R1013, R1023 (R0960, R0978 the same config) | +4.8 / +0.9 |
| x0 | orbit horizontal error mean, 0.025 / 0 (cm) | R0832 (n = 1, 1 Oct) / R0974, R0979 | 2.8 / 0.8 |
| x0 | orbit height error, 0.025 / 0 (cm) | as above | +4.8 / +0.8 |
| disturbance | 0.6 N down: steady / peak / overshoot (cm), none, ref, model | R0962/R0980, R0968/R0981, R0969/R0982 | 8.4/8.9/0.4; 0.3/6.6/7.1; 0.2/6.5/7.0 |
| disturbance | 0.6 N down: time to 1 cm after onset (s), ref / model; learned force (N) | as above | 6.0 / 6.1; 0.58 |
| disturbance | 0.4 N -x: steady / peak / overshoot (cm), none, model; onset settle (s); learned force (N) | R0961/R0983, R0971/R0984 | 7.6/7.7/0.1; 0.1/7.2/7.0; 5.5; 0.42 |
| disturbance | thrust-map error: height / horizontal error (cm), tilt (deg), ref / model | R0967/R0985, R0963/R0986 | 0.0/3.5, 6.4 / 0.0/0.0, 7.9 |
| gain | peak / overshoot (cm) / settling (s): k_i 0.4; 0.8; gate 5 s | R0969/R0982; R0976; R0977 | 6.5/7.0/4.7; 5.6/6.0/7.2; 6.0/6.4/4.5 |
| robust | height error no integral / integral (cm), shift (cm): none | R1013/R1023; R1066/R1067 | +0.9 / 0.0, -0.8 |
| robust | mass +10 % | R1061/R1064; R1063/R1065 | +13.5 / 0.0, -13.4 |
| robust | mass +20 % | R1038/R1043; R1039/R1044 | +25.8 / +25.8 (a), 0.0 |
| robust | mass -20 % | R1040/R1045; R1041/R1046 | -23.5 / 0.0, +23.3 |
| robust | rods +5 cm, measured | R1047/R1054; R1048/R1055 | +0.9 / +0.1, -0.9 |
| robust | rods +5 cm, not measured | R1049/R1056; R1050/R1057 | +8.2 / +8.2 (b), 0.0 |
| robust | radius +2 cm | R1051/R1059; R1042/R1053 | +0.9 / 0.0, -0.9 |
| robust | tilt (deg); landed | all 28 | 1.0-1.3; 28/28 |
| land | straight; unwind; unwind + ramp + idle: runs, tipped (aborted), pushed out (cm), tilt after 1.2 s (deg) | R0960-R0986; land_matrix R0987-R1008; land_fix R1009-R1034 | 21, 2 (1), 5.4-18.2, 2.2-5.7; 18, 2 (2), 0.5-4.1, 2.4-5.0; 18, 0 (0), 0.5-3.0, 1.8-4.0 |
| traj (replaced) | hover 3 / circle 3: mean / max error (cm), height (cm), tilt (deg) | R1013/R1023; R0974/R0979/R1016/R1027/R0992/R1004 | 0.7/0.7, +0.9, 1.2; 0.8/1.5, +0.8, 1.2 |
| traj (cut or T4) | 4 UAVs: hover 1/3/5/9 (old numbering); line; circle; figure-8; spin | R1018/R1032; R1068/R1070; R1033/R1037; R1035/R1036; R1069/R1071 | 0.6/0.7, -2.2, 1.6; 0.9/2.7, -1.7, 1.3; 0.9/1.9, -1.7, 1.3; 0.8/2.4, -1.7, 1.3; 0.5/1.4, -1.7, 1.2 |
| detach (replaced) | peak tilt (deg) / back in band (s) / height change (cm) | R0939; R0997, R1007, R1012, R1029, R1034 | 3.2/2.6/2.7; 2.8-4.0/0.9-1.0/3.4-3.6 |

## 5. Pass / fail numbers (frozen 16 Oct 09:05, before the runs)

What a miss means:
- **S miss:** the run is investigated. An infrastructure void or a twin artefact (a TB14-type tip on landing) is
  re-flown once and recorded; anything else keeps the cell out of the table.
- **C miss:** the claim is falsified on the freeze tree and is removed or reworded in the thesis. It is not re-flown.
- **A miss:** the cell is flagged "changed with the stack", and any text quoting the 4-5 Oct number is rewritten from
  the new runs. It is not re-flown to chase agreement.

**S, every run** (sources: events.csv, the harness checks, launch.log):
- S1: no FLEET_ABORT, ENVELOPE FAULT or EMERGENCY line; no "DROP AND LAND" and no "UNANNOUNCED DETACH" line; LANDED
  reached, and every drone disarmed after LANDED.
- S2: planner solve failures at or under the configs' own `max_planner_solve_fail: 1`.
- The harness criteria (`max_hold_z_err_m` and the like) are not bars. They fail by design on R-m+20-0, R-m-20-0 and
  R-rod+5nomeas-0, and possibly on X0-H1.

**A, agreement with 4-5 Oct, every cell of section 4:**
- A cell agrees if the new mean is within [0.5, 2] times the old, or within a floor of the old.
- Floors: 1.0 cm for displacements and errors, 0.5 deg for tilt, 1.0 s for times, 0.1 N for forces.
- Why: four known changes sit under every cell: the measured pull, the 60 deg hand-over, detach mode, and z_i_max.
  The 4-5 Oct repeats agreed to 0.1-0.2 cm (R0969/R0982 peak 6.51/6.52), so the twin's noise is not the issue. A
  factor of 2 still catches a mechanism that stopped working: an integral that never engages leaves 8 cm where 0.2 cm
  was, a factor of 40. The floors keep a 0.1 to 0.6 cm change from counting as a factor of 6.

**C, each table's claim.** "Resolved" means the two arms' run ranges do not overlap and the means differ by at least
max(floor, 2 x the larger within-arm range): Tier A's freeze 1.2 rule with the floors above. An unresolved pair is
reported as unresolved, and the text then says "no difference resolved".

| # | Claim | Confirms | Falsifies | Why this bar |
|---|---|---|---|---|
| C-X1 | the relaxed box raises the hover | X0-H1 minus X0-H0 height error resolved, at least +2 cm | resolved below +2 cm or the other sign | 4-5 Oct: 3.9 cm (+4.8 against +0.9) |
| C-X2 | pinning cuts the orbit error | X0-O0 horizontal mean at most 0.5 x X0-O1, resolved | not resolved, or ratio over 0.5 | 4-5 Oct: 0.29 (0.8 against 2.8, n = 1) |
| C-D1 | the model integral removes a constant push | D-ZM and D-XM steady displacement at most 1.0 cm, every run | any run over 1.0 cm | 4-5 Oct 0.2 and 0.1 |
| C-D2 | the push is visible without it | D-Z0 and D-X0 steady at least 2 cm | under 2 cm (the table then shows nothing) | 4-5 Oct 8.4 and 7.6 |
| C-D3 | it learns the applied force | change of the integral force within 25 % of the applied: 0.45-0.75 N (z), 0.30-0.50 N (x), every run | outside | 4-5 Oct 0.58 and 0.42 (3 % and 5 %) |
| C-D4 | the target shift also removes the vertical push | D-ZR steady at most 1.0 cm | over 1.0 cm | 4-5 Oct 0.3 |
| C-D5 | the model integral also removes the horizontal error of a thrust-map error | D-MM horizontal error below D-MR's, resolved | not resolved, or the other sign | 4-5 Oct 0.0 against 3.5 |
| C-G1 | gain 0.8 lowers the peak | G-K8 peak below D-ZM, resolved | the other sign, resolved | 4-5 Oct 5.6 against 6.5 (n = 1) |
| C-G2 | the 5 s gate lowers overshoot | G-S5 overshoot below D-ZM, resolved | the other sign, resolved | 4-5 Oct 6.4 against 7.0 (n = 1) |
| C-R1 | inside the gate the integral removes the error | R-x-M height error within 1.0 cm for none, m+10, m-20, rod+5, r+2 | any over 1.0 cm | 4-5 Oct 0.0-0.1 |
| C-R2 | footnotes a and b (gate; tautness gate) | R-m+20-M and R-rod+5nomeas-M within 1.0 cm of their R-x-0 | the integral engages (footnote rewritten from the new runs) | 4-5 Oct equal to 0.1 cm |
| C-R3 | measuring the rods absorbs a 5 cm error | R-rod+5-0 within 1.0 cm of X0-H0 | over 1.0 cm | 4-5 Oct +0.9 against +0.9 |
| C-R4 | every flight landed | 26 of 26 R-* runs plus X0-H0 land without a tip past 45 deg | any tip or abort | 4-5 Oct 28/28 |
| C-L1 | the default landing does not tip | 0 tips past 45 deg over every freeze-tree 0.55 m three-carrier landing (app_* without L-S, and the traj3 claim cells); pushed out at most 6.0 cm; tilt after 1.2 s at most 8.0 deg | any tip (the claim becomes "k of N"), or over the two limits | 4-5 Oct 0 of 18, 3.0 cm, 4.0 deg (limits = 2 x) |
| C-L2 | a straight descent pushes the UAVs outward | L-S peak outward push above X0-H0's, both runs | not | 4-5 Oct 18.2 against 3.0 cm |
| T4 (optional) | four carriers on the default controller | the carry card's bars 1-4 per run (RMSE at most 25 mm; peak tilt at most 6 deg; radius ratio at least 0.95 where it applies) | any | carry card, frozen 16 Oct |
| DT-A (optional) | the announced resize | S, and peak tilt at most 8.0 deg | over | 4-5 Oct 2.8-4.0 (2 x the top) |

fig:tracker_x0, fig:disturbance, fig:flight, fig:landing and fig:detach come from runs that pass S. Their numbers are
the tables', so no bars of their own.

**What the result cannot say:**
- The twin's rods are rigid, it has no downwash, and the push is an ideal wrench.
- The robustness rows are about the controller's model, not the rig's.
- The landing tip count is a twin rigid-rod result (TB14) and is labelled as such.

## 6. Repeats

- 2 per arm, exactly: `table_robust` asserts 2 runs per cell.
- Voids (infrastructure) are re-flown until each arm has 2 non-void runs.
- An in-flight abort is a result, not a void (section 9, table_robust).
- The 4-5 Oct single-run cells (orbit epsilon 0.025, k_i 0.8, gate 5 s) get their second run here.

## 7. Cost and the lab batch

27 configs x 2 = 54 runs, plus 12 optional.

| Rate | Assumption |
|---|---|
| per run | about 4 min of lab wall per slot (three carriers; the 15-16 Oct traj3 runs measured 157-219 s plus start-up) |
| per run, four carriers | about 4.5 min |
| batch, 4 slots | about 55 min |
| with voids | about 65 min (about 20 % pose time-outs, inventory section 7) |
| with sync and build | about 70 min |
| optional | about 17 min more |

Fly after the carry claim and the detach freeze (fig:detach and tab:detach read Tier A). The order puts the Chapter 4
items first and alternates r1 and r2, so a batch cut short leaves whole pairs:

1. r1, Chapter 4 (12): app_x0_hold_eps0, app_x0_hold_eps025, app_x0_orbit_eps0, app_x0_orbit_eps025,
   app_pushz_none, app_pushz_ref, app_pushz_model, app_push04_none, app_push04_model, app_mapneg_ref,
   app_mapneg_model, app_rob_none_int
2. r2: the same 12
3. r1, appendix (15): app_pushz_model_ki08, app_pushz_model_settle, app_land_straight, app_rob_m+10_z0,
   app_rob_m+20_z0, app_rob_m-20_z0, app_rob_rod+5_z0, app_rob_rod+5nomeas_z0, app_rob_r+2_z0, app_rob_m+10_int,
   app_rob_m+20_int, app_rob_m-20_int, app_rob_rod+5_int, app_rob_rod+5nomeas_int, app_rob_r+2_int
4. r2: the same 15
5. Optional, at Wesley's word (r1 then r2): app_traj4_hover3915, app_traj4_orbit, app_traj4_fig8,
   app_traj4_linex, app_traj4_spin, app_w4_3915_detach

`tools/remote/lab_batch.py` takes the paths in this order (each listed twice). Check first that
`git diff 4b3560e -- src/` is still empty: it is at a7cb254.

## 8. New code?

- No flight code: 33 configs only (`configs/experiments/app_*.yaml`), each a line-for-line copy of its 4-5 Oct
  config with section 3's lines changed. No critic: no control law, estimator or gate changes.
- The generator edits (section 9) are made only once the runs exist.
- Reviewer: on the regenerated tables, before the thesis.
- Removal: delete the app_ configs.

## 9. Generator edits once the runs exist (not made)

`tools/thesis_sim_tables.py`:
- **table_x0:** hold `{'old': 'app_x0_hold_eps025', 'new': 'app_x0_hold_eps0'}`, orbit
  `{'old': 'app_x0_orbit_eps025', 'new': 'app_x0_orbit_eps0'}`. Caption: the epsilon 0.025 circle is no longer a
  single run on an earlier tree.
- **table_disturbance:**
  - Z = `app_pushz_none` / `app_pushz_ref` / `app_pushz_model`;
  - X = `app_push04_none` / `app_push04_model`;
  - M = `app_mapneg_ref` / `app_mapneg_model`.
- **table_gain:** the three arms become `app_pushz_model`, `app_pushz_model_ki08` and `app_pushz_model_settle`
  through `ids()`, not explicit run lists.
- **table_land:**
  - `from land_metrics import land` fails today (ImportError). land_metrics was rewritten on 9 Oct (0400066):
    restore the old function as its own module (`git show 0400066^:tools/land_metrics.py`) and import from it.
  - One row, "unwind, feedforward ramp, ground idle (freeze tree)": every non-void run of the three-carrier app_
    configs except `app_land_straight`, plus the 15 `traj3_<shape>_v<speed>` claim cells (0.55 m; not L100, not
    traj3d).
  - The straight-descent and unwind rows: open question 1.
- **table_robust:**
  - ROBUST names become `app_x0_hold_eps0` (none, no integral), `app_rob_none_int` and `app_rob_{arm}_{z0|int}`.
  - The "Target shift" column becomes the model integral's learned vertical force at LAND (`int_fz`, N), since
    `z_bias` is the reference integral's.
  - The same `land` import fix.
  - Count an in-flight abort in the 2-per-cell assertion: `runs_of` drops aborted runs today, so an aborting cell would
    fail the assertion instead of showing in "Landed".
- **table_traj:**
  - TRAJ becomes the traj3 claim: a hover row (`app_rob_none_int`, 3 carriers, 'hover'), then the five shapes at
    0.125 m/s (`traj3_{circle,orbit,fig8,linex,spin}_v125`), or all 15 cells if the appendix carries T7t's detail.
  - The 'lap' window's 27.2 s is the 0.125 m/s orbit. Other speeds need their own lap (2 pi r / v plus the ramps).
  - `spin_yaw_err` hard-codes 0.125 m/s.
  - Caption: the default controller (model integral, measured pull), not "epsilon 0, no integral".
  - The 4-UAV rows go, or become `app_traj4_*`.
- **table_detach:**
  - DETACH becomes Tier A rows on the RELEASE event: measured pull (`ta_a1_r1..r3`), planned pull
    (`ta_a2_r1..r3`), and optionally detection off (`ta_a5_r1..r3`).
  - `detach_stats` reads the first DETACH or RELEASE event.
  - Caption: unannounced release, plates 11/9/7/3, the leaver on plate 9.
  - Or `app_w4_3915_detach` if open question 3 picks the announced resize.
- **`__main__`:** drop table_four.

`tools/thesis_sim_figures.py`:
- X0_RUNS: hold (`app_x0_hold_eps025` r1, `app_x0_hold_eps0` r1); orbit (`app_x0_orbit_eps025` r1,
  `app_x0_orbit_eps0` r1).
- DIST_RUNS: z (`app_pushz_none`, `app_pushz_ref`, `app_pushz_model`, r1 each); x (`app_push04_none`,
  `app_push04_model`, r1 each).
- LAND_RUNS: before = `app_land_straight` r1; after = `app_x0_hold_eps0` r1.
- DETACH_RUN: `ta_a1_r1`'s run.
  - The event becomes RELEASE.
  - DETACH_PLATES = {0: 11, 1: 9, 2: 7, 3: 3}. The old {0: 1, 1: 3, 2: 5, 3: 9} is the counter-clockwise numbering
    from before 8 Oct; the configs type 330,270,210,90 clockwise.
  - Docstring: resize at detection, not at an announced DETACH.
- TRAJ_RUNS: circle = `traj3_orbit_v125` r1 (the same reference kind as the old panel); eight = `traj3_fig8_v125` r1
  (0.125, the rig's speed; the old figure-8 was 0.25).
  - Drop the epsilon 0.025 traces: the box comparison is fig:tracker_x0's.
  - Caption: three carriers.
- FLIGHT_RUN: `app_rob_none_int` r1. The hard-coded `axhline(45.0)` and the docstring's "45 deg" become 60, or the
  value is read from the run's planner parameters.
- Run ids go to `configs/thesis_figures.yaml` and `docs/figures/PROVENANCE.md`, not the text.

## 10. Stale or wrong, found while preparing

1. **table_land and table_robust cannot run on today's tree.** They import `land` from `tools/land_metrics.py`, which
   was rewritten on 9 Oct (0400066). The Appendices comment already says the table was made "as of 0400066^".
2. **`ff_cap_force` is a no-op in detach mode.**
   - `graph_detach` never passes it, and the planner reads 0.0: R1853's dump against R0960's 0.75 in mode mpc.
   - That holds for the traj3 claim, Tier A, Tier B, D1/D4 and these configs: all type 0.75.
   - Harmless for every window here (the breakaway stand-in acts during the lift only). But the 4-5 Oct carry runs
     flew it and today's do not.
3. **tab:detach mislabels its second row.** "epsilon 0, no integral": R0997, R1007, R1012 and R1029 flew z_ki 0.4
   with the reference integral (planner dumps; `w4_3915_detach_lu/_lf` type it). Both rows had the integral on.
4. **The old configs, flown as they are today, fly a hybrid** (`--check` passes on all of them, so nothing warns):
   - mode mpc;
   - hand-over 45 against a planned cable angle of 60;
   - z_i_max 0.15;
   - the `_zki` arms on the model integral (int_mode auto);
   - the old-box baselines `b2_cap075_fix_n3` and `_orbit` at epsilon 0 (no `x0_relax_frac` line).
   The inventory's N3 "configs exist in old/" needs the section 3 edits.
5. **Plate names.**
   - "Plates 1/3/5/9" in tab:traj, tab:detach's caption and fig:detach use the counter-clockwise numbering from before
     8 Oct.
   - In today's numbering the W4 layout is 11/9/7/3, the leaver on plate 9.
   - The freeze cards still say "1/3/5/9" in places.
6. **fig:flight draws the hand-over at 45 deg; it is 60 since 7 Oct.**
7. **The model integral's `int_k_xy`/`int_k_z` defaults were identified at 45 deg on the planned pull.**
   - D-Z0 and D-X0 re-measure the stiffness (applied force over steady displacement).
   - A mismatch moves the settling times, not the learned force.
   - Re-identifying the defaults would be a launch-default change: Wesley's word.

## 11. Open for Wesley

1. **tab:land.**
   - Recommended: keep one row, the default landing over every freeze-tree 0.55 m three-carrier landing (about 82).
     The straight-descent contrast stays in fig:landing (L-S, 2 runs).
   - A straight-descent tip rate on the freeze tree would need about 20 runs per arm. Mixing in the 4-5 Oct rows
     breaks "one code tree per claim table": keep them only as a dated "earlier stack" note.
2. **tab:traj's four-carrier rows.**
   - Recommended: cut. The rig carries with three; the four-carrier phases are covered by the detach and join tables.
   - The alternative is the optional T4 block (10 runs, about 17 min).
   - Also: does the appendix keep all 15 traj3 cells (T7t's detail) or only the 0.125 m/s row?
3. **tab:detach.**
   - Recommended: Tier A A1/A2 with the transient measures (no runs; they complement T9's Delta-theta).
   - Or the announced resize DT-A (2 runs).
   - Or cut, since T9 and F10b cover the release.
4. **tab:robust's integral column.**
   - Recommended: the model integral (today's default, inventory N3). The "Target shift" column becomes the learned
     force.
   - Or keep the reference integral (`int_mode: reference` on the R-x-M arms) to keep the old column.
5. **Freeze the section 5 bars before the batch**, or change them. Done 09:05 with the recommended answers to 1-4 (main block only); Wesley may change any of 1-4 tonight, before the runs that depend on it are scored.

## Critic

Not required: configs only. No control law, estimator, or gate that can hold the fleet down is changed.
