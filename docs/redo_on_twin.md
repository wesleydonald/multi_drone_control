# To fly again on the rig twin (5 Oct 2026)

The rig twin is the simulation (docs/decisions.md, 5 Oct 2026). Every thesis result that was flown
on the legacy worlds (`simulation_assets/old_worlds/`, `legacy:=true`) or on the SIL bench is a
`\redo{...}` box in the thesis, with the old text kept under it as comments. This file is the list
behind those boxes: one row per box, 38 rows for 38 boxes.

Runs: Gazebo arms are flown twice, SIL arms once (CLAUDE.md). Before repeating an arm, fly its
neighbours once. Priority: A = a thesis claim rests on it, B = supports a claim, C = restated or
re-confirmed only where cited.

## Prerequisites (nothing in the attach, network or SIL rows can fly before these)

| id | what | where | unblocks |
|---|---|---|---|
| P1 | A rig-twin attach world and newcomer model: a free fourth UAV with a magnet arm on the twin's geometry (0.55 m rod, pivot 4 cm below centre, 0.55 kg), built by `make_carry_worlds.py` / `generate_rigid_world.py`; `make_weld_variants.py` parametrised to build the weld ladder from it | `simulation_assets/`, `tools/` | every attach row |
| P2 | `graph_attach` (sim branch) and `graph_detach.sim_network` pass `real_mode.map_and_geometry` (the rig thrust map, pivot, pretension, cap release), as carry and dissipative already do; then `profiles.LEGACY_ONLY` is emptied and `sim.yaml` gains `network` and `attach` modes | `src/bringup/bringup/` | every network and attach row |
| P3 | The SIL plant gains the pivot offset, the rod mass and the affine thrust map with pack sag; `tools/sil/scenario.py` defaults, the 96 `configs/sil` and the gate's SIL smoke move to the twin and drop `legacy:=true` | `tools/sil/` | every SIL row |

## The list (box numbers follow the thesis order)

| # | thesis place | what to fly on the twin | needs | runs | pri | flown before as |
|---|---|---|---|---|---|---|
| 1 | abstract (`title_page.tex`) | none of its own: restated from 21 (rejoin under 2 deg) and 12 (OCP resize against network) | 12, 21 | 0 | C | |
| 2 | `contributions.tex` (OCP resize against network) | none of its own: restated from 12 and 19/20 | 12, 20 | 0 | C | |
| 3 | `contributions.tex` (full sequence) | none of its own: restated from 21 | 21 | 0 | C | |
| 4 | `contributions.tex` (the comparison) | none of its own: restated from 29 | 29 | 0 | C | |
| 5 | `reconfiguration.tex` (hand-back to an even ring) | detach 4 to 3 from the even ring, survivors at 0/90/180 deg with the planner told 0/120/240: payload tilt | P2 | 2 | B | R0093 |
| 6 | `reconfiguration.tex` (payload speed at the approach) | weld onto a payload moving at 0.4 m/s against 0.2 m/s | P1, P2 | 4 | B | R0267 |
| 7 | `reconfiguration.tex` (mux authority at the weld) | hand-over with the mux switching on an armed but idle stream against the armed-and-above-idle rule: do the newcomer's motors cut | P1, P2 | 2 | B | R0663-R0666 |
| 8 | `reconfiguration.tex` (newcomer's starting tension) | fold-in from 1 N against 0.1 N: tilt at the weld | P1, P2 | 4 | B | R0663-R0666 |
| 9 | `dissipative.tex` (cone origin) | offline network test: cone drawn from the payload centre against from each rim attachment point, on the twin's 0.55 m rods and r 0.225 | none (offline harness, `dissipative_verify`) | 2 offline | C | offline harness, 0.5 m ring |
| 10 | `sim_results.tex` (initial condition, SIL) | SIL: relaxed box against node 0 pinned, three UAVs, hover; the x0 SIL row of `tab:x0` and panel of `fig:tracker_x0` come back with it | P3 | 2 SIL | B | R0956/R0957 |
| 11 | `sim_results.tex` (integral forms, SIL) | SIL: reference-shift against model integral with the feedforward at 0.9: plan-ahead error, rod elevation, share of the bound used | P3 | 3 SIL | B | R0964-R0966 |
| 12 | `sim_detach.tex` (detach A/B, `tab:detach_ab`) | OCP resize against network hand-off, 4 to 3, during a circle and in hover; with it the network's hand-back with and without the two fixes | P2 | 8 + 4 | A | R0111-R0114, R0108/R0109 |
| 13 | `sim_detach.tex` (SIL mode study) | SIL: the same A/B in hover and circle | P2, P3 | 4 SIL | B | R0496-R0505 |
| 14 | `sim_detach.tex` (layout study, `tab:detach_layout`) | 4 to 3 in hover, plate 3 released: even ring 0/3/6/9 against 1/3/5/9; 1/3/5/9 mid-circle; 1/3/6/9. The 1/3/5/9 hover case exists on the twin (R1076 and `twin_default_detach_3915`); the even and 1/3/6/9 rings need worlds from `make_carry_worlds.py` | two new twin worlds | 6 | A | R0559-R0572, R0554/R0556, R0582/R0584 |
| 15 | `sim_detach.tex` (fewer than three) | detach 3 to 2: the ring capsizes on two rim points (`min_survivors` lowered for the run) | none (Gazebo) or P3 (SIL) | 1 | C | SIL |
| 16 | `sim_attach.tex` (network attach, `fig:attach`) | network attach during a circle at 0.2 m/s, three UAVs at 3/12/9 o'clock, fourth welds at 6; the storyboard figure from the run (`tools/attach_storyboard.py`) | P1, P2 | 2 | A | R0196-R0209 |
| 17 | `sim_attach.tex` (layout, `tab:attach_layout`) | network attach on 3/12/9 against 4/12/8 with the newcomer at 6: hover tilt, weld transient, circle tilt, load share at 12. Settles the OPEN demo-layout line in decisions.md | P1, P2 | 4 | A | R0234, R0266-R0272 |
| 18 | `sim_attach.tex` (weld ladder, `tab:weld`) | ball joint, rod with inertia, rod with a mid-span joint, rigid weld: flight time and tilt 2 s and 8 s after the weld | P1 (weld variants), P2 | 8 | B | R0126-R0141 |
| 19 | `sim_attach.tex` (attach A/B, SIL) | SIL: network attach against OCP resize at the same weld, hover and mid-circle | P2, P3 | 4 SIL | B | R0518-R0521 |
| 20 | `sim_attach.tex` (attach by OCP resize, Gazebo) | full sequence in hover; rejoin during the circle; the newcomer's reference along the line to the cable apex. The newcomer pivot study is not redone: the twin has the low pivot on every UAV | P1, P2 | 2 + 2 + 2 | A | R0541-R0552, R0575/R0577, R0580/R0581 |
| 21 | `sim_attach.tex` (rejoin mission M1, `tab:m1`) | plates 1/3/5/9 on the orbit, plate 3 detaches, picks up and drops, rejoins, fleet lands: peak tilt at detach and rejoin, orbit tilt, height dip, duration. Also settles the OPEN `attach_traj_hold_mode=settle` line | P1, P2, the partner's mission planner on the twin plant | 2 | A | R0646, R0685/R0686, R0663-R0672, R0684-R0687 |
| 22 | `sim_attach.tex` (harder rejoin) | payload keeps moving during the rejoin and the newcomer drops 0.1 kg into the ring | 21 | 2 | B | R0696/R0697 |
| 23 | `sim_attach.tex` (ground join M2) | the partner's controller joins four UAVs on the floor and hands them over; lift, hover, land; with it the hand-over bench under CPU load, gyro against differenced poses | a twin M2 bench world (`tools/sim_test/make_m2_bench_world.py`, `tejen/`), P2 | 2 + 2 | A | T0015-T0029, R0707-R0720 |
| 24 | `sim_architecture.tex` (network on a circle) | the network flying a circle with the MPC tracker, three UAVs, r 0.5 m: radius ratio and lag by stage | P2 | 2 | A | R0054-R0077 |
| 25 | `sim_architecture.tex` (tracking architecture, `tab:arch`) | under the network on the same circle: MPC tracker, velocity loop, velocity loop with the common-mode trim; an integrator in each UAV's velocity loop | P2 | 8 | A | R0054-R0077 |
| 26 | `sim_architecture.tex` (planner against network) | none of its own: restated from 12 and 16/20 | 12, 20 | 0 | C | |
| 27 | `sim_architecture.tex` (measured-force throttle, `tab:indi`) | network attach during a circle: model-based, measured force at gain 1, at 0.5, at 0.5 with anti-swing. The code was removed on 2026-09-23; restore it for the run or cut the table (Wesley) | P1, P2, the removed loop | 8 | C | R0234-R0257 |
| 28 | `sim_architecture.tex` (closing the loop) | network variants on the measured payload state (tensions at the measured attitude, horizontal leash, cone on effective gravity, admittance on rod tension), each against the open-loop network | P2 | 10 | C | registry, Sep |
| 29 | `sim_architecture.tex` (verdict) | none of its own: rewritten from 12, 16-20, 24, 25 | those | 0 | C | |
| 30 | `sim_robustness.tex` (wrong geometry, was `tab:rodlen`) | hover with rods typed short, radius typed 2 cm short, and the combination with the mass typed 20 % low, with and without the rod measurement (T9 has the long cases) | none (twin carry, Gazebo) | 12 | A | SIL R0308-R0327 |
| 31 | `sim_robustness.tex` (attach robustness, E10) | attach with the payload mass wrong by +/-25 % and the rod length by +/-10 %: completes or not, height after the weld | P1, P2 | 8 | B | R0215-R0222 |
| 32 | `discussion.tex` (RQ1, attach half) | none of its own: restated from 21 | 21 | 0 | C | |
| 33 | `discussion.tex` (hand-over faults) | none of its own: restated from 7, 8, 21 | 7, 8, 21 | 0 | C | |
| 34 | `discussion.tex` (RQ2) | none of its own: rewritten from 29 | 29 | 0 | C | |
| 35 | `discussion.tex` (geometry cases) | none of its own: restated from 14, 15, 17 | 14, 15, 17 | 0 | C | |
| 36 | `Conclusion.tex` (reconfiguration finding) | none of its own: restated from 12 and 21 | 12, 21 | 0 | C | |
| 37 | `Conclusion.tex` (comparison finding) | none of its own: restated from 29 | 29 | 0 | C | |
| 38 | `Appendices.tex` (negatives table, 12 rows) | re-confirm only the negatives still cited in the text, one arm against its baseline each: terminal velocity reference, per-UAV integrators (25), cone on effective gravity / measured attitude / leash / admittance (28), weld compliance (18), measured-force throttle (27), even-ring hand-back (5), load-force Kalman filter, steeper rods at the hand-over, rate-loop integral gain 10 | per row | up to 12 | C | registry, Aug-Sep |

Totals: 38 boxes; 13 need no flight of their own (1-4, 26, 29, 32-37, and 9 is offline);
25 need runs, about 110 Gazebo runs and 13 SIL runs at the repeats above. Runnable today without a
prerequisite: 14 (after two worlds), 15 and 30.

## Cut, not redone (marked `% CUT 5 Oct 2026` in the thesis, no box)

| thesis place | why |
|---|---|
| `sim_robustness.tex`, thrust-gain trim (`kt_trim`) paragraph | `kt_trim` is off on the rig and the twin; a legacy-only feature |
| `sim_robustness.tex`, first height-integral test (R0557/R0558, R0564-R0568) | superseded by T9 and the disturbance table on the twin |

## Outside the thesis

- `tools/gate.sh` step 6 (SIL smoke) and all 96 `configs/sil` fly the legacy plant until P3.
- The 104 legacy Gazebo configs (`world: old_worlds/...` or `tejen/...`) stay flyable; the runner
  passes `legacy:=true` and `sim_thrust_map:=linear` for them.
- `params.py` still holds the legacy geometry and ring inertia (the twin's values are typed by
  `sim.yaml`; the inertia has no launch argument, so the twin flies ixx 0.0273 against the
  world's 0.0223). Needs Wesley's word.
