# Appendix re-fly: scored against the card's frozen bars

Scored Sat 17 Oct 2026, 01:55 AEDT (from `date`). Card: `docs/experiments/2026-10-16_appendix_refly.md`; section 5 bars
frozen 16 Oct 09:05, applied as written. No bar was moved.

Runs: R2023-R2076 (54), laptop, one at a time, 16 Oct 21:37 to 17 Oct 01:41, mean RTF 0.44-0.45. Order as card
section 7: Chapter 4 block r1 R2023-R2034, r2 R2035-R2046; appendix block r1 R2047-R2061, r2 R2062-R2076. T4 and DT-A
were not flown and are not scored.

Tree: every manifest gives c3bf628 (day-1016) with one dirty file; `git diff c3bf628 -- src/` is empty. The card's
check "`git diff 4b3560e -- src/` is empty" does not hold literally: 4b3560e..c3bf628 adds the corner-cutting
switches (8 files under `src/`: `plan_w_pos_scale`, `plan_w_s_scale`, `plan_w_r_scale`, `plan_rate_ff`,
`plan_ref_lag_s`, `plan_sqp_iters`), all off by default and typed by no app_ config. 50 of the 54 planner parameter
dumps show them off (scales 1.0, rate feedforward false, lag 0.0, iterations 0 = the default). R2046, R2051, R2068 and
R2073 have no planner dump (the parameter readback was incomplete); their launch arguments in the manifest match their
configs. The flight code is the freeze's with those switches off.

## Verdict

| Bar | Scored | Pass | Miss | Other |
|---|---|---|---|---|
| S (every run) | 54 runs | 54 | 0 | 0 void |
| A, re-flown cells (x0, disturbance, gain, robust, land) | 60 cells | 29 agree | 31 | |
| A, replaced rows (traj hover and circle, detach) | 11 cells | 8 agree | 3 | |
| C (C-X1..C-L2) | 15 claims | 10 CONFIRMED | 3 FALSIFIED | 2 UNRESOLVED |

Every run is usable: nothing to re-fly. The tracker-box, horizontal-push, robustness and landing claims hold. The
vertical claims do not: on the freeze stack a 0.6 N downward push leaves no steady offset even without an integral,
and a typed mass error of ±20 % no longer moves the hover. The integral therefore has almost nothing left to remove
vertically: it learns 0.10 N of the 0.6 N push, and gain 0.8 and the 5 s gate show no resolved difference. The likely
cause is the measured pull (`cable_source mocap`), one of the four changes under every cell; this re-fly does not
separate it from the other three.

## S per run

S1: no FLEET_ABORT, ENVELOPE FAULT, EMERGENCY, DROP AND LAND or UNANNOUNCED DETACH in events.csv or launch.log; LANDED
reached; all three drones disarmed after LANDED (run.csv `d*_armed` drops at or after LANDED in every run, and
launch.log has three "Disarmed via service" lines after "Landed - disarming the fleet"). S2: planner solve failures 0
in every run (limit 1). The runner's FAILs are harness criteria, not bars (card section 5): checked against S1 and S2,
none is an S miss.
- X0-H1, X0-O1 and both R-rod+5nomeas arms fail the harness hold-z check (+0.13 m and +0.17 m): the offsets these
  arms exist to show.
- The four mapneg runs fail the harness peak-tilt check: 9.2-9.4 deg at t 15.5 s with the ring at 0.07 m, during
  lift-off with the weak drone. The hover tilt is 0.1 deg.
- R2057: "drone 0 mocap stale at t=70.09" is 2.4 s after LANDED (67.68), with the ring at 0.02 m. All three drones
  disarmed at 67.76-68.42, after LANDED. It is a shutdown artefact, not a void.

The card expected harness FAILs on R-m+20-0 and R-m-20-0. They passed (see A, robust).

| Run | Config | Arm | Rep | S1 | S2 (solve fails) | Runner | RTF | Verdict |
|---|---|---|---|---|---|---|---|---|
| R2023 | app_x0_hold_eps0 | X0-H0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2024 | app_x0_hold_eps025 | X0-H1 | r1 | pass | 0 | FAIL: hold z error +0.131 m (limit 0.050), harness only | 0.44 | S PASS |
| R2025 | app_x0_orbit_eps0 | X0-O0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2026 | app_x0_orbit_eps025 | X0-O1 | r1 | pass | 0 | FAIL: hold z error +0.130 m (limit 0.050), harness only | 0.44 | S PASS |
| R2027 | app_pushz_none | D-Z0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2028 | app_pushz_ref | D-ZR | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2029 | app_pushz_model | D-ZM | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2030 | app_push04_none | D-X0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2031 | app_push04_model | D-XM | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2032 | app_mapneg_ref | D-MR | r1 | pass | 0 | FAIL: peak tilt 9.4 deg (limit 5.0) at ring lift-off, harness only | 0.44 | S PASS |
| R2033 | app_mapneg_model | D-MM | r1 | pass | 0 | FAIL: peak tilt 9.2 deg (limit 5.0) at ring lift-off, harness only | 0.44 | S PASS |
| R2034 | app_rob_none_int | R-none-M | r1 | pass | 0 | PASS | 0.45 | S PASS |
| R2035 | app_x0_hold_eps0 | X0-H0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2036 | app_x0_hold_eps025 | X0-H1 | r2 | pass | 0 | FAIL: hold z error +0.131 m (limit 0.050), harness only | 0.44 | S PASS |
| R2037 | app_x0_orbit_eps0 | X0-O0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2038 | app_x0_orbit_eps025 | X0-O1 | r2 | pass | 0 | FAIL: hold z error +0.129 m (limit 0.050), harness only | 0.44 | S PASS |
| R2039 | app_pushz_none | D-Z0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2040 | app_pushz_ref | D-ZR | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2041 | app_pushz_model | D-ZM | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2042 | app_push04_none | D-X0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2043 | app_push04_model | D-XM | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2044 | app_mapneg_ref | D-MR | r2 | pass | 0 | FAIL: peak tilt 9.3 deg (limit 5.0) at ring lift-off, harness only | 0.44 | S PASS |
| R2045 | app_mapneg_model | D-MM | r2 | pass | 0 | FAIL: peak tilt 9.3 deg (limit 5.0) at ring lift-off, harness only | 0.45 | S PASS |
| R2046 | app_rob_none_int | R-none-M | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2047 | app_pushz_model_ki08 | G-K8 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2048 | app_pushz_model_settle | G-S5 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2049 | app_land_straight | L-S | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2050 | app_rob_m+10_z0 | R-m+10-0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2051 | app_rob_m+20_z0 | R-m+20-0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2052 | app_rob_m-20_z0 | R-m-20-0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2053 | app_rob_rod+5_z0 | R-rod+5-0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2054 | app_rob_rod+5nomeas_z0 | R-rod+5nomeas-0 | r1 | pass | 0 | FAIL: hold z error +0.171 m (limit 0.050), harness only | 0.44 | S PASS |
| R2055 | app_rob_r+2_z0 | R-r+2-0 | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2056 | app_rob_m+10_int | R-m+10-M | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2057 | app_rob_m+20_int | R-m+20-M | r1 | pass | 0 | FAIL: health check after LANDED (drone 0 mocap stale at t=70.09) | 0.45 | S PASS |
| R2058 | app_rob_m-20_int | R-m-20-M | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2059 | app_rob_rod+5_int | R-rod+5-M | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2060 | app_rob_rod+5nomeas_int | R-rod+5nomeas-M | r1 | pass | 0 | FAIL: hold z error +0.171 m (limit 0.050), harness only | 0.44 | S PASS |
| R2061 | app_rob_r+2_int | R-r+2-M | r1 | pass | 0 | PASS | 0.44 | S PASS |
| R2062 | app_pushz_model_ki08 | G-K8 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2063 | app_pushz_model_settle | G-S5 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2064 | app_land_straight | L-S | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2065 | app_rob_m+10_z0 | R-m+10-0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2066 | app_rob_m+20_z0 | R-m+20-0 | r2 | pass | 0 | PASS | 0.45 | S PASS |
| R2067 | app_rob_m-20_z0 | R-m-20-0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2068 | app_rob_rod+5_z0 | R-rod+5-0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2069 | app_rob_rod+5nomeas_z0 | R-rod+5nomeas-0 | r2 | pass | 0 | FAIL: hold z error +0.171 m (limit 0.050), harness only | 0.44 | S PASS |
| R2070 | app_rob_r+2_z0 | R-r+2-0 | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2071 | app_rob_m+10_int | R-m+10-M | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2072 | app_rob_m+20_int | R-m+20-M | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2073 | app_rob_m-20_int | R-m-20-M | r2 | pass | 0 | PASS | 0.45 | S PASS |
| R2074 | app_rob_rod+5_int | R-rod+5-M | r2 | pass | 0 | PASS | 0.44 | S PASS |
| R2075 | app_rob_rod+5nomeas_int | R-rod+5nomeas-M | r2 | pass | 0 | FAIL: hold z error +0.171 m (limit 0.050), harness only | 0.44 | S PASS |
| R2076 | app_rob_r+2_int | R-r+2-M | r2 | pass | 0 | PASS | 0.44 | S PASS |

## A per cell

Rule (card section 5): a cell agrees if the new mean is within [0.5, 2] times the old, or within the floor of the old
(1.0 cm, 0.5 deg, 1.0 s, 0.1 N). Old values are section 4's. Where section 4 gives a range over many runs (land,
detach, robust tilt), the old mean was recomputed from the same runs with the same tool (`land_touchdown.land`,
`detach_stats`, `method_metrics`), which reproduces section 4's ranges exactly; the new mean is compared with it.
Values are as `tools/thesis_sim_tables.py` computes them. Signed cells keep their sign.

| Table | Cell | Old (4-5 Oct) | New r1 / r2 | New mean | Verdict |
|---|---|---|---|---|---|
| x0 | hover height error, eps 0.025 (cm) | +4.80 | +13.07 / +13.07 | +13.07 | MISS |
| x0 | hover height error, eps 0 (cm) | +0.90 | -0.06 / -0.06 | -0.06 | agree |
| x0 | orbit horizontal error mean, eps 0.025 (cm) | +2.80 | +3.87 / +3.87 | +3.87 | agree |
| x0 | orbit horizontal error mean, eps 0 (cm) | +0.80 | +0.74 / +0.74 | +0.74 | agree |
| x0 | orbit height error, eps 0.025 (cm) | +4.80 | +13.04 / +12.94 | +12.99 | MISS |
| x0 | orbit height error, eps 0 (cm) | +0.80 | -0.05 / -0.11 | -0.08 | agree |
| disturbance | 0.6 N down, none: steady (cm) | +8.40 | +0.03 / +0.01 | +0.02 | MISS |
| disturbance | 0.6 N down, none: peak (cm) | +8.90 | +1.43 / +1.34 | +1.38 | MISS |
| disturbance | 0.6 N down, none: overshoot (cm) | +0.40 | +1.38 / +1.46 | +1.42 | MISS |
| disturbance | 0.6 N down, ref: steady (cm) | +0.30 | 0.00 / +0.02 | +0.01 | agree |
| disturbance | 0.6 N down, ref: peak (cm) | +6.60 | +1.31 / +1.34 | +1.32 | MISS |
| disturbance | 0.6 N down, ref: overshoot (cm) | +7.10 | +1.52 / +1.44 | +1.48 | MISS |
| disturbance | 0.6 N down, model: steady (cm) | +0.20 | +0.01 / +0.02 | +0.01 | agree |
| disturbance | 0.6 N down, model: peak (cm) | +6.50 | +1.25 / +1.41 | +1.33 | MISS |
| disturbance | 0.6 N down, model: overshoot (cm) | +7.00 | +1.39 / +1.39 | +1.39 | MISS |
| disturbance | 0.6 N down, ref: time to 1 cm after onset (s) | +6.00 | +1.03 / +1.05 | +1.04 | MISS |
| disturbance | 0.6 N down, model: time to 1 cm after onset (s) | +6.10 | +0.97 / +1.13 | +1.05 | MISS |
| disturbance | 0.6 N down, model: learned force (N) | +0.58 | +0.10 / +0.10 | +0.10 | MISS |
| disturbance | 0.4 N -x, none: steady (cm) | +7.60 | +4.82 / +4.81 | +4.81 | agree |
| disturbance | 0.4 N -x, none: peak (cm) | +7.70 | +5.44 / +5.35 | +5.40 | agree |
| disturbance | 0.4 N -x, none: overshoot (cm) | +0.10 | +0.75 / +0.72 | +0.73 | agree |
| disturbance | 0.4 N -x, model: steady (cm) | +0.10 | -0.01 / 0.00 | 0.00 | agree |
| disturbance | 0.4 N -x, model: peak (cm) | +7.20 | +5.34 / +5.63 | +5.49 | agree |
| disturbance | 0.4 N -x, model: overshoot (cm) | +7.00 | +5.20 / +5.35 | +5.27 | agree |
| disturbance | 0.4 N -x, model: time to 1 cm after onset (s) | +5.50 | +4.96 / +4.78 | +4.87 | agree |
| disturbance | 0.4 N -x, model: learned force (N) | +0.42 | +0.33 / +0.32 | +0.32 | agree |
| disturbance | thrust-map error, ref: height error (cm) | 0.00 | 0.00 / 0.00 | 0.00 | agree |
| disturbance | thrust-map error, ref: horizontal error (cm) | +3.50 | 0.00 / +0.01 | +0.01 | MISS |
| disturbance | thrust-map error, ref: ring tilt (deg) | +6.40 | +0.09 / +0.08 | +0.08 | MISS |
| disturbance | thrust-map error, model: height error (cm) | 0.00 | -0.07 / -0.07 | -0.07 | agree |
| disturbance | thrust-map error, model: horizontal error (cm) | 0.00 | +0.01 / +0.01 | +0.01 | agree |
| disturbance | thrust-map error, model: ring tilt (deg) | +7.90 | +0.09 / +0.10 | +0.09 | MISS |
| gain | k_i 0.4: peak (cm) | +6.50 | +1.25 / +1.41 | +1.33 | MISS |
| gain | k_i 0.4: overshoot (cm) | +7.00 | +1.39 / +1.39 | +1.39 | MISS |
| gain | k_i 0.4: settling (s) | +4.70 | +1.22 / +1.18 | +1.20 | MISS |
| gain | k_i 0.8: peak (cm) | +5.60 | +1.26 / +1.44 | +1.35 | MISS |
| gain | k_i 0.8: overshoot (cm) | +6.00 | +1.32 / +1.37 | +1.35 | MISS |
| gain | k_i 0.8: settling (s) | +7.20 | +1.13 / +1.15 | +1.14 | MISS |
| gain | gate 5 s: peak (cm) | +6.00 | +1.32 / +1.31 | +1.31 | MISS |
| gain | gate 5 s: overshoot (cm) | +6.40 | +1.33 / +1.31 | +1.32 | MISS |
| gain | gate 5 s: settling (s) | +4.50 | +1.08 / +1.15 | +1.11 | MISS |
| robust | none: height error, no integral (cm) | +0.90 | -0.06 / -0.06 | -0.06 | agree |
| robust | none: height error, integral (cm) | 0.00 | -0.05 / -0.05 | -0.05 | agree |
| robust | mass +10 %: height error, no integral (cm) | +13.50 | -0.05 / -0.04 | -0.04 | MISS |
| robust | mass +10 %: height error, integral (cm) | 0.00 | -0.04 / -0.03 | -0.03 | agree |
| robust | mass +20 %: height error, no integral (cm) | +25.80 | 0.00 / +0.03 | +0.01 | MISS |
| robust | mass +20 %: height error, integral (cm) | +25.80 | 0.00 / 0.00 | 0.00 | MISS |
| robust | mass -20 %: height error, no integral (cm) | -23.50 | -0.11 / -0.11 | -0.11 | MISS |
| robust | mass -20 %: height error, integral (cm) | 0.00 | -0.11 / -0.10 | -0.11 | agree |
| robust | rods +5 cm, measured: height error, no integral (cm) | +0.90 | -0.06 / -0.06 | -0.06 | agree |
| robust | rods +5 cm, measured: height error, integral (cm) | +0.10 | -0.06 / -0.05 | -0.05 | agree |
| robust | rods +5 cm, not measured: height error, no integral (cm) | +8.20 | +17.09 / +17.09 | +17.09 | MISS |
| robust | rods +5 cm, not measured: height error, integral (cm) | +8.20 | +17.09 / +17.09 | +17.09 | MISS |
| robust | radius +2 cm: height error, no integral (cm) | +0.90 | +0.89 / +0.90 | +0.90 | agree |
| robust | radius +2 cm: height error, integral (cm) | 0.00 | +0.87 / +0.83 | +0.85 | agree |
| robust | tilt, all rows (deg; old 1.0-1.3 per row) | +1.17 | 28 runs, +0.05 to +0.14 | +0.07 | MISS |
| land | straight descent: pushed outward (cm; old 21 runs, mean, range 5.4-18.2) | +16.45 | +23.52 / +23.55 | +23.54 | agree |
| land | straight descent: tilt after 1.2 s (deg; old mean, range 2.2-5.7) | +4.96 | +12.43 / +6.76 | +9.60 | agree |
| land | unwind + ramp + idle: pushed outward (cm; old 18 runs, mean, range 0.5-3.0) | +0.93 | 82 runs, -2.24 to +0.65 | +0.46 | agree |
| land | unwind + ramp + idle: tilt after 1.2 s (deg; old mean, range 1.8-4.0) | +2.39 | 82 runs, +1.46 to +4.70 | +2.36 | agree |
| traj | hover 3: mean error (cm) | +0.70 | +0.04 / +0.04 | +0.04 | agree |
| traj | hover 3: max error (cm) | +0.70 | +0.08 / +0.09 | +0.09 | agree |
| traj | hover 3: height (cm) | +0.90 | -0.05 / -0.05 | -0.05 | agree |
| traj | hover 3: tilt (deg) | +1.20 | +0.06 / +0.06 | +0.06 | MISS |
| traj | circle 3: mean error (cm) [traj3_orbit_v125] | +0.80 | +1.03 / +0.82 | +0.93 | agree |
| traj | circle 3: max error (cm) [traj3_orbit_v125] | +1.50 | +1.94 / +1.47 | +1.70 | agree |
| traj | circle 3: height (cm) [traj3_orbit_v125] | +0.80 | -0.09 / -0.05 | -0.07 | agree |
| traj | circle 3: tilt (deg) [traj3_orbit_v125] | +1.20 | +0.54 / +0.56 | +0.55 | MISS |
| detach | peak tilt (deg; old 5 runs mean, range 2.8-4.0) [Tier A A1] | +3.48 | +5.64 / +7.06 / +4.62 | +5.77 | agree |
| detach | back in band (s; old mean, range 0.9-1.0) [Tier A A1] | +0.95 | +3.16 / +3.20 / +3.12 | +3.16 | MISS |
| detach | height change (cm; old mean, range 3.4-3.6) [Tier A A1] | +3.50 | +3.21 / +3.83 / +2.70 | +3.25 | agree |

Sources of the new values: x0 R2023-R2026 / R2035-R2038; disturbance R2027-R2033 / R2039-R2045; gain D-ZM, G-K8
(R2047/R2062), G-S5 (R2048/R2063); robust X0-H0, R-none-M (R2034/R2046), R2050-R2061 / R2065-R2076; land L-S
(R2049/R2064) and the 82 freeze-tree landings of C-L1; traj hover R-none-M (15 s before LAND), traj circle
`traj3_orbit_v125` R1853/R1854 (first lap, carry claim, flight code 4b3560e); detach Tier A A1 R1887/R1891/R1895
(three runs, so three values).

Not scored:
- land, runs and tipped counts: counts, not means; the tip count is C-L1's.
- land, "unwind" row (land_matrix): not flown (open question 1, recommended answer: one row).
- robust, "Target shift" column: replaced by the model integral's learned force (section 9); no old value.
- robust, landed 28/28: C-R4's.
- traj, four-carrier rows: T4, not flown.
- detach, first old row (R0939, epsilon 0.025): no freeze-tree counterpart. The second old row (the default controller
  then) is compared with A1 (the default now). A different event: unannounced release against announced DETACH.
- detach with the planned pull (Tier A A2, R1888/R1892/R1896), for information: peak tilt 17.0-17.5 deg, back in band
  1.9-2.0 s, height change 4.5-5.2 cm.

## C per claim

"Resolved": the two arms' run ranges do not overlap and the means differ by at least max(floor, 2 x the larger
within-arm range). Values r1 / r2.

| # | Claim | Numbers | Runs | Verdict |
|---|---|---|---|---|
| C-X1 | the relaxed box raises the hover | X0-H1 +13.07 / +13.07 cm against X0-H0 -0.06 / -0.06 cm: +13.13 cm, ranges apart, needed 1.0; at least +2 | R2024, R2036; R2023, R2035 | CONFIRMED |
| C-X2 | pinning cuts the orbit error | X0-O0 0.74 / 0.74 cm against X0-O1 3.87 / 3.87 cm: ratio 0.19 (at most 0.5), 3.13 cm apart, needed 1.0 | R2025, R2037; R2026, R2038 | CONFIRMED |
| C-D1 | the model integral removes a constant push | steady: D-ZM 0.01 / 0.02 cm, D-XM -0.01 / 0.00 cm (limit 1.0 every run) | R2029, R2041; R2031, R2043 | CONFIRMED |
| C-D2 | the push is visible without it | steady: D-Z0 0.03 / 0.01 cm (under 2), D-X0 4.82 / 4.81 cm | R2027, R2039; R2030, R2042 | FALSIFIED (vertical) |
| C-D3 | it learns the applied force | z 0.099 / 0.102 N (band 0.45-0.75); x 0.325 / 0.325 N (band 0.30-0.50) | R2029, R2041; R2031, R2043 | FALSIFIED (vertical) |
| C-D4 | the target shift also removes the vertical push | D-ZR steady 0.00 / 0.02 cm (limit 1.0) | R2028, R2040 | CONFIRMED |
| C-D5 | the model integral also removes the thrust-map horizontal error | D-MM 0.007 / 0.005 cm against D-MR 0.003 / 0.010 cm: ranges overlap, 0.000 cm apart | R2033, R2045; R2032, R2044 | FALSIFIED (not resolved) |
| C-G1 | gain 0.8 lowers the peak | G-K8 1.26 / 1.44 cm against D-ZM 1.25 / 1.41 cm: ranges overlap, +0.02 cm | R2047, R2062; R2029, R2041 | UNRESOLVED |
| C-G2 | the 5 s gate lowers overshoot | G-S5 1.33 / 1.31 cm against D-ZM 1.39 / 1.39 cm: ranges apart, but -0.07 cm against the 1.0 cm floor | R2048, R2063; R2029, R2041 | UNRESOLVED |
| C-R1 | inside the gate the integral removes the error | R-x-M height error (cm): none -0.05 / -0.05, m+10 -0.04 / -0.03, m-20 -0.11 / -0.10, rod+5 -0.06 / -0.05, r+2 +0.87 / +0.83 (limit 1.0) | R2034, R2046; R2056, R2071; R2058, R2073; R2059, R2074; R2061, R2076 | CONFIRMED |
| C-R2 | footnotes a and b: the integral stays off | m+20: M 0.00 / 0.00 against 0 0.00 / +0.03 cm (0.01 apart); rod+5nomeas: M +17.09 / +17.09 against 0 +17.09 / +17.09 cm (0.00 apart); learned force at LAND at most 0.006 N | R2057, R2072; R2051, R2066; R2060, R2075; R2054, R2069 | CONFIRMED |
| C-R3 | measuring the rods absorbs a 5 cm error | R-rod+5-0 -0.06 / -0.06 cm against X0-H0 -0.06 / -0.06 cm (limit 1.0) | R2053, R2068; R2023, R2035 | CONFIRMED |
| C-R4 | every flight landed | 28 of 28 (26 R-* runs and X0-H0's 2) landed, no abort, worst drone tilt after 1.2 s 3.8 deg | R2023, R2035, R2034, R2046, R2050-R2061, R2065-R2076 | CONFIRMED |
| C-L1 | the default landing does not tip | 82 landings: 0 tips past 45 deg, 0 aborts; pushed outward at most 0.65 cm (limit 6.0); tilt after 1.2 s at most 4.70 deg (limit 8.0) | 52 app_ runs (all but L-S); carry claim R1847-R1876 (30) | CONFIRMED |
| C-L2 | a straight descent pushes the UAVs outward | L-S 23.52 / 23.55 cm against X0-H0 0.52 / 0.53 cm | R2049, R2064; R2023, R2035 | CONFIRMED |

C-L1 by source: the 52 app_ runs, pushed outward -2.24 to 0.65 cm, tilt after 1.2 s 1.84-4.70 deg; the 30 carry-claim
runs, 0.45 to 0.63 cm and 1.46-2.13 deg.

## Generator edits (section 9), made 17 Oct

`tools/thesis_sim_tables.py`:
- `runs_of(config, aborted=False)`: with `aborted=True` an in-flight abort is kept, so table_robust's 2-per-cell
  assertion counts it and "Landed" shows it.
- table_x0: hold `app_x0_hold_eps025` / `app_x0_hold_eps0`, orbit `app_x0_orbit_eps025` / `app_x0_orbit_eps0`.
- table_disturbance: Z, X and M arms become the app_ configs of section 9.
- table_gain: `app_pushz_model`, `app_pushz_model_ki08`, `app_pushz_model_settle` through `ids()`.
- table_land: one row, "unwind, feedforward ramp, ground idle (freeze tree)": every non-void run of the three-carrier
  app_ configs except `app_land_straight` (aborts kept), plus the carry claim's 0.55 m runs (`claim_runs()`: registry
  rows of card 2026-10-16_carry_speed_envelope, not L100, not traj3d). STRAIGHT and the two 4-5 Oct rows removed. The
  import was already fixed (`land_touchdown`, 8c468bd).
- table_robust: ROBUST "none" is `app_x0_hold_eps0` / `app_rob_none_int`, the others `app_rob_{arm}_{z0|int}`; the
  "Target shift (cm)" column becomes "Learned force (N)": `learned_fz()`, the planner log's `int_fz` at the last tick
  before LAND.
- table_traj: TRAJ is the hover row (`app_rob_none_int`) and the five traj3 shapes at 0.125 m/s from the carry claim's
  runs (orbit on the 27.2 s 'lap' window, the others 'all'); the four-carrier rows removed (open question 2); the spin
  footnote keys on the row name.
- table_detach: DETACH is Tier A on the freeze tree: measured pull R1887/R1891/R1895 (A1), planned pull
  R1888/R1892/R1896 (A2); `detach_stats` reads the first DETACH or RELEASE; first column header "Rod pull".
- table_four, `registry()` and `trajectory_error()` removed (only table_four used them); `__main__` drops table_four
  and takes an optional output directory.

`tools/thesis_sim_figures.py`:
- X0_RUNS hold (R2024, R2023), orbit (R2026, R2025); DIST_RUNS z (R2027, R2028, R2029), x (R2030, R2031);
  LAND_RUNS before R2049, after R2023; FLIGHT_RUN R2034, its hand-over line and docstring at 60 deg.
- DETACH_RUN R1887 (ta_a1_r1), on RELEASE; DETACH_PLATES {0: 11, 1: 9, 2: 7, 3: 3}; docstring, labels and the tilt
  axis (0-8 deg, the A1 peak is 5.6 deg).
- TRAJ_RUNS circle R1853 (`traj3_orbit_v125` r1), eight R1849 (`traj3_fig8_v125` r1); the epsilon 0.025 traces and
  legend removed; docstring three carriers.

The tables were regenerated into the scratchpad, not into `thesis/` or `mdc_wt/thesis`. The figures were rendered
into the scratchpad as a check, not into `docs/figures/`. `configs/thesis_figures.yaml` and `docs/figures/PROVENANCE.md`
are not updated yet.

## What the thesis text must change

Only A misses and C falsifications.

| Item | Cell or claim | Old | New | Change |
|---|---|---|---|---|
| tab:x0, fig:tracker_x0 | hover height error, epsilon 0.025 (cm) | +4.8 | +13.1 | rewrite the number; the claim holds (C-X1). The hold panel's y-axis (-1 to 7 cm) clips the new trace |
| tab:x0 | orbit height error, epsilon 0.025 (cm) | +4.8 | +13.0 | rewrite the number |
| tab:disturbance, fig:disturbance | 0.6 N down, no integral: steady / peak / overshoot (cm) | 8.4 / 8.9 / 0.4 | 0.0 / 1.4 / 1.4 | C-D2 falsified: a vertical push leaves no steady offset without an integral on the freeze stack. Remove or reword "without an integral a constant force leaves an offset" for the vertical case; it still holds horizontally (4.8 cm) |
| tab:disturbance | 0.6 N down, target shift: peak / overshoot (cm) | 6.6 / 7.1 | 1.3 / 1.5 | rewrite |
| tab:disturbance | 0.6 N down, model: peak / overshoot (cm) | 6.5 / 7.0 | 1.3 / 1.4 | rewrite |
| tab:disturbance | 0.6 N down, time to 1 cm after onset, ref / model (s) | 6.0 / 6.1 | 1.0 / 1.1 | rewrite |
| tab:disturbance, fig:disturbance | 0.6 N down, learned force (N) | 0.58 | 0.10 | C-D3 falsified vertically: the integral learns a sixth of the applied force (the rest needs no integral). Keep "learns the applied force" for the horizontal push only (0.32 of 0.40 N, inside the 25 % band) |
| tab:disturbance | thrust-map error, target shift: horizontal error (cm) / tilt (deg) | 3.5 / 6.4 | 0.0 / 0.1 | rewrite |
| tab:disturbance | thrust-map error, model: tilt (deg) | 7.9 | 0.1 | rewrite |
| tab:disturbance | C-D5, the model integral also removes the horizontal error of a thrust-map error | 0.0 against 3.5 | 0.0 against 0.0 | falsified (no difference): neither integral leaves a horizontal error now. Remove the claim |
| tab:gain | k_i 0.4: peak / overshoot (cm) / settling (s) | 6.5 / 7.0 / 4.7 | 1.3 / 1.4 / 1.2 | rewrite |
| tab:gain | k_i 0.8 | 5.6 / 6.0 / 7.2 | 1.3 / 1.3 / 1.1 | rewrite; C-G1 unresolved: "no difference resolved" |
| tab:gain | gate 5 s | 6.0 / 6.4 / 4.5 | 1.3 / 1.3 / 1.1 | rewrite; C-G2 unresolved: "no difference resolved" |
| tab:robust | mass +10 %, no integral (cm) | +13.5 | 0.0 | the "about 1.25 cm per percent" sentence goes: a typed mass error no longer moves the hover |
| tab:robust | mass +20 %, no integral / integral (cm) | +25.8 / +25.8 | 0.0 / 0.0 | footnote a (the error is over the integral's 0.25 m gate) has no error left to describe: rewrite or drop it |
| tab:robust | mass -20 %, no integral (cm) | -23.5 | -0.1 | as mass +10 % |
| tab:robust | rods +5 cm not measured, both columns (cm) | +8.2 / +8.2 | +17.1 / +17.1 | rewrite the number; footnote b holds (C-R2) |
| tab:robust | tilt (deg) | 1.0-1.3 | 0.1 | rewrite |
| tab:traj | hover, three carriers: tilt (deg) | 1.2 | 0.1 | rewrite (row now from the default controller) |
| tab:traj, fig:traj | circle, three carriers: tilt (deg) | 1.2 | 0.6 | rewrite (row now the carry claim's orbit) |
| tab:detach | back in band (s) | 0.9-1.0 | 3.1-3.2 | rewrite (row now Tier A A1, an unannounced release) |

## Noted while scoring (not bars)

- The radius +2 cm arm's model integral winds up. In R2061 and R2076 `int_fz` climbs steadily from 0 to +0.76 N
  over the 40 s hold while the height error stays at +0.85 cm, about the same as without the integral (+0.90 cm).
  The integral's force is not moving the ring height. In a longer hold it would keep climbing toward its bound. A
  radius error is plausible on the rig, so this is worth a look before the rig. Not tested here.
- What absorbs the vertical push and the mass error is not identified. The measured pull is the obvious suspect; a
  pair with `cable_source model` on D-Z0 and R-m+20-0 would separate it (one variable each).
- The land table's "Pushed outward" range now starts below zero (-2.2 cm: in three of the four mapneg runs the worst drone never
  left its reference, and LANDED came 1.3 s after touchdown). The caption needs "+ = outside the reference".
- The figures need the thesis-figure pass before use: the fig:tracker_x0 y-axis, fig:detach's new axis, and fig:traj
  without its epsilon 0.025 traces.

## Reviewer (17 Oct, 02:40): weak, with corrections that override the sections above

**1. False touchdown in the thrust-map arms. C-L1 is NOT supported.**
- In R2032, R2044 and R2045 LANDED came 1.3 s after the ring touched down. All three drones were 0.46-0.53 m up
  (rest 0.10 m), the fleet disarmed, and every drone ended on its side (90 deg).
- In R2033 drone 1 was disarmed 0.26 m up and ended at 25 deg.
- None of the other 80 landings disarmed a drone more than 0.05 m above rest.
- **Cause, from R2032's trace:**
  - The planner's `TouchdownDetector` reads "stalled" when the highest drone descends slower than
    LAND_STALL_FRAC x land_vel for LAND_STALL_S.
  - Drone 1 flies a sim thrust offset of 0.155 against its map's 0.185, so it is stronger than the map.
  - After the ring is down it sinks only ~0.1 m/s at hover thrust.
- `tools/land_touchdown.py` stops at LANDED, so the scoring never saw the fall.
- Correction rows R2032c, R2033c, R2044c, R2045c (S MISS). They are not re-flown: the fault repeats in 3 of 4 runs.
- **Rig relevance:** a drone whose thrust map is off (a pack or prop change) could trigger the same disarm at 0.4 m.
  A fix to the detector is a change to a gate that lands the fleet, so it needs a card and the critic. Not built.

**2. The vertical push is applied and the tracker's measured pull absorbs it.**
- The push is WRENCH 0 0 -0.6 over 35.7-50.7 s (R2027), and the steady window sits inside it.
- On drone 0 the measured cable acceleration goes from -4.966 to -5.337 m/s². That is 0.61 N in total, taken up in
  ~1 s, with no tracker integral and `kt_trim` off.
- The same path explains the mass arms: the measured pull is the same at 0.86 and 1.03 kg typed.
- It also explains the thrust-map arm: drone 1's measured pull is lower, so the horizontal error is gone.

**3. The vertical model integral integrates with no effect. The "learned force" is not a learned force.**
- The push value doubles with the gain: 0.10 N at k_i 0.4, 0.21-0.22 N at k_i 0.8.
- R2034 drifts -0.05 N with no push.
- On the radius +2 cm arm the integral winds to 0.76 N (R2061, R2076) while the height error stays at 0.78-0.98 cm
  (0.89/0.90 without it).
- The robust table's "learned force" column must not present 0.76 N as learned.

**4. The rods +5 cm not measured result (+17.1 cm) stands as a number, but its geometry is unexplained.** A rigid
estimate gives +2 to +6 cm.

**5. C-L1's 82 landings span three trees** (c3bf628, 5375333, e836cf3). Their `src/` differs only by the
corner-cutting switches, which are off by default.

**The thesis must not say:**
- "0 of 82 tipped" or "the default landing does not tip" without the thrust-map exception;
- that a vertical force or a mass error leaves an offset without the integral on this stack, or quote 8.4 cm;
- that the integral learns the push, or a share of it;
- that the integral removes the mass, rod or radius error;
- that 0.76 N is a learned force;
- that the model integral fails on the thrust-map error;
- that a typed mass error does not matter, without naming the measured pull and the rigid-rod twin;
- a geometric explanation of the +17.1 cm;
- that the epsilon 0.025 hover offset is tracking error (the references themselves are 10.6 cm higher);
- that gain 0.8 or the 5 s gate changes the peak or the overshoot.

**Generator open items:**
- `table_land` must count the post-LANDED fall: read until the drones stop, not until LANDED.
- The robust learned-force column needs relabelling or removing.
