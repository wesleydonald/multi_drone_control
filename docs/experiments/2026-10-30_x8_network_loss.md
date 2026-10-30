# Card X8: the paper's method through a loss (30 Oct 2026)

Owner Wesley. Plan row X8 of `docs/plan_rescope_2026-10-30.md` (approved 30 Oct: P1, a one-day box, 15 runs; risk
line "the network cannot take a loss as built: X8 to P3"; cut order "X8 (if it needs code)"). Written 30 Oct ~16:45
AEDT on week8 5a7ccd0. Nothing flown. **Status: bars FROZEN 30 Oct 16:26 on Wesley's word, as proposed, before any run.** Scope (Wesley): ours against the
port now (no code); the exact law's loss row to P3. The port arm is told 0.35 s after the release, the never-told port
beside it. Flies after the square-to-T fix (its rim cell is that release; decisions 30 Oct).

**The check (section 8) in one line.** Our planner and the port fly X8 with no code: the runner releases a carrier in
the network mode, and the port drops the node when it is told (the runner's DETACH, as the authors' code drops one).
The exact law's arm needs code: unaware, our stand-in of the law flies the released quad off on its modelled pull and
trips the trackers' 3 m/s speed fault (a fleet abort); told, the quad never leaves the law's 2 m perception range, so
the paper's own mechanism never runs. Per the rescope X8 then moves to P3 unless Wesley says otherwise. Recommended
(decision 1): fly ours against the port now (no code, the 15 runs below) and leave only the exact law's loss row to P3.

## 1. Question
Quan et al. claim their method copes with losing a robot ("Once a failed robot exits the perception range of the others,
the system autonomously readjusts", p.16: four robots carry 4 kg, robot 2 disengages at 40 s and flies away, the
tensions rebalance in 13 s). Through one unannounced loss in hover, does the paper's method keep the ring as close to
where it was as our planner does: on the paper's geometry (world C, every cable on a hub at the ring's centre of mass)
and on our rim?

## 2. Hypothesis
- **World C.** Our planner detects the release (X1: 0.28-0.38 s), resizes to the T (no moment arm on the hub) and
  the ring barely moves. The port, told at the same moment, drops the node, re-spaces the three survivors to an even
  triangle and makes up the larger share with its bounded trim: its excursion exceeds ours by more than 20 mm.
- **Rim.** Our planner peaks at ~57 deg with ~100 mm of dip (X1's square-to-T, R2965/R2972). The port's re-spacing
  pulls the survivors off the plates they are bolted to: a larger excursion and tilt than ours, or a loss.
- **The port never told** (neighbour): worse than told. The node stays in the network, every survivor keeps a quarter
  share until the trim catches up, and the leaver keeps its modelled pull and drifts ~1.5 m out of its slot.
- **The exact law never told** (smoke): the released quad keeps its modelled pull, leaves the survivors' 2 m range in
  0.9-1.4 s, its own share then becomes the whole ring and it passes 3 m/s 1.0-1.6 s after the release (offline model of
  the law and the velocity loop, 45-60 deg cables): the trackers' speed fault disarms the fleet.

## 3. The one variable
The method that flies the fleet through the loss. As in the C table, a cell's arms differ only in `mode` and the
method's params file (each the C table's file plus the same loss rules); the told port adds one DETACH event.

| Arm | What flies after the release | Config | Kind |
|---|---|---|---|
| ours | our planner (the detach mode): its detector (a cable 6 cm over its length for 2 ticks), the problem resized to the T, the leaver stepped out 0.5 m and landed | `x8_<cell>_ours` | claim |
| port | the network, our port (the compared arm, decisions 30 Oct): RELEASE, then `/fleet/detach 1` 0.35 s later (when our planner would know), so the node's rows of the network are zeroed as in the authors' init.m:74-79; the survivors re-space, the leaver rises 0.6 m and holds | `x8_<cell>_port` | claim |
| port never told | the port with no DETACH: the node stays in the network | `x8_<cell>_portu` | neighbour, no bar |
| exact law never told | `net_law quan` at its current reading (R6c), RELEASE only | `x8_hub_quanu` (world C only) | smoke, no bar |

Cells, 4 quads typed 0,270,180,90 (plates 0/9/6/3), 0.86 kg ring, 1 m rods, hover at 1.0 m. Unannounced RELEASE of
`/drone_1` (plate 9; "drone 2" in the logs) 25 s after the ring passes 0.95 m (~22 s after the network's hand-over, R2805: 2.8 s after
0.95 m; the exact law's offset settles by +15 s), LAND 30 s after the release (the paper's rebalance took 13 s).

| Cell | World | Base configs (the C table's hover cells, as flown) | Loss rules (`configs/twin/x8_<cell>_<arm>.yaml`) |
|---|---|---|---|
| `hub` | `four_rigid_ground_rig_hubcom_L100.sdf` (world C) | `c_pl_hubcom_hover_L100`, `c_net_port_hubcom_hover_L100`, `c_net_quan_split_hubcom_hover_L100` | C's (tilt drop 180, ring envelope off) + `drop_gap_deg 200` |
| `rim` | `four_rigid_ground_rig_L100.sdf` (plates at 0.225 m) | `c_pl_hover_L100`, `c_net_tune_portfinal_hover_L100` | C's + `drop_gap_deg 200`, `drop_tilt_deg 80`, `payload_tilt_envelope false` (X1's sqT cell) |

`drop_gap_deg 200` lets our planner fly on with the T (its largest gap 180 deg; on the hub there is no moment arm) and
is inert in the network phase, as are the planner's other loss rules. On the rim the trackers' 80 deg drop only acts
through our planner (the network phase ignores `/fleet/drop`), so the scorer applies it to every arm offline.

Pinned in every run: one tree (whatever the batch flies, recorded in each manifest's git_diff.patch), the `slope`
velocity filter (the default since 28 Oct and the final-tree re-fly's; the C table flew `ema`), the C table's gains
and launch arguments, 5 SQP iterations (the default), no added delay.

## 4. Baseline
- **Named baseline: the port, told** (plan row: "the network"; decisions 30 Oct: "our port is the compared arm, the
  law as published one row"), per cell, flown in the same batch on the same tree. Ours is the treatment.
- For scale, fixed team, not compared (C table, `ema`, 3-D error from the hand-over, RMS): world C ours 1.0 mm
  (R2791), port 12.3 mm (R2792), the exact law 110.0 / 118.3 mm with R6b (R2805/R2812) and 109.1 mm with R6c
  (R2813); rim ours 1.4 / 1.3 mm (R2733/R2734), port 18.7 / 16.1 mm with a 51 / 22 deg yaw swing (R2685/R2686).
  Ours through the same rim release (X1 sqT, 5 iterations, `slope`): abs peak 57.1 / 57.0 deg, dip 100.2 / 99.5 mm,
  path error over 10 s 100.5 / 100.2 mm, resized 0.31 s after the release, no loss (R2965/R2972).

## 5. Pass / fail numbers (FROZEN 30 Oct 16:26, Wesley, as proposed, before any run)
Metrics (a scorer `tools/x8_metrics.py`, offline, from run.csv, events.csv and the logs; t_e = RELEASE):
- **Excursion E** (mm): the ring centre's largest 3-D distance from its mean over the 5 s before t_e, t_e .. t_e + 20 s
  (measured from where it was, so the exact law's fixed-team offset is not counted).
- **Dip** (mm): as X1. **Shift** (mm): the ring's horizontal mean over the last 5 s before LAND against the 5 s before t_e.
- **Settle t_s** (s): from t_e to the first time after which the ring centre stays within max(20 mm, 3 sigma_pre) of its
  mean over the last 5 s before LAND, through LAND; censored at LAND.
- **Quad tilt** (deg): the survivors' peak attitude, t_e .. t_e + 10 s (the leaver's apart). **Ring tilt** (rim):
  X1's tilt rise and abs peak; on world C reported only (the hub leaves it free).
- **Loss**: a fleet abort or envelope fault, our planner's DROP AND LAND, the ring under 0.3 m before LAND, on the rim
  ring tilt over 80 deg on 2 samples, on world C the ring over 300 mm from its pre-release mean for 1 s (C's band rule,
  about the pre-release mean instead of the target). The same rule for every arm. A loss ranks behind every run of
  the other arm without one.
- Reported: our planner's detection and resize times; the port's DETACH line; the survivors' largest azimuth gap over
  the last 5 s (the port re-spaces, ours keeps the T); the leaver's peak speed, farthest distance from the ring and
  closest approach to a survivor.

Bars (ours_max = the larger of ours' two repeats; means over two):
- **L1 (the headline; the plan's falsifier), world C.** CONFIRMED if in both port repeats E > ours_max + 20 mm, or the
  port loses the ring where ours does not. **FALSIFIED if the port's mean E <= ours' mean E and the port loses
  nothing** ("network excursion <= ours on world C"). Otherwise inconclusive, said so.
- **L2 (the rim).** The same rule on the rim, a claim of its own cell.
- **L3 (ours is safe).** Every ours run: no loss; on world C t_s <= 10 s. The rim's settling is reported (it is X1
  sqT's subject).
- **Validity (a void run is re-flown once).** RELEASE in events.csv and the joint let go (the leaver's pivot over
  1.06 m from its plate within 1 s of t_e, from run.csv); ours: UNANNOUNCED DETACH within 0.5 s and the planner at
  n-1; the network arms: NET_HANDOVER at least 15 s before t_e; port: the planner's "DETACH drone 2 (/drone_1) ... 3 drones remain on the load"
  within 0.6 s of t_e; never told: no DETACH line; no abort, drop or failed solve before t_e; late planner and tracker
  ticks <= 1 %; read-back `mode`, `net_law`, `drop_gap_deg`, `drop_tilt_deg` as typed; >= 95 % of the rows. A loss
  after t_e is an outcome, not void.
- The smoke `x8_hub_quanu` sets no bar. It falsifies this card's code reading if the released quad stays under 3 m/s
  and the ring stays up to LAND: the exact law would then meet an unannounced loss as built, and its arm flies unaware
  (a revision of this card before any claim run).

Reading caveat, stated before the runs: the twin's links are rigid rods hinged at both ends and the twin has no added
delay; the paper's robots fly 1 m away on slack-able cables.

## 6. Repeats
2 per claim arm and cell. Before them one smoke each of `x8_hub_ours` (our detector has never met a release on the
hub), `x8_hub_port` (the network's node drop after a release, first on this tree) and `x8_hub_quanu` (exploratory rows,
not in the table). An ill-defined arm is fixed before the claim runs; the bars do not change.

## 7. Cost
Lab PC, 4 slots x 4 cpus, ~9 min wall a run (R2805: 99 s of sim at RTF 0.25): 3 smokes + 8 claim runs (2 cells x 2
arms x 2) + 4 neighbour runs (2 cells x 2) = **15 runs, ~45 min**, then scoring. Laptop: none. With the exact law's
arm (code below): +2 claim runs on world C and 1 smoke, +3 on the rim if the law first hovers the rim ring (it never
has).

## 8. New code?
**None for the 15 runs.** Tooling only: `tools/x8_metrics.py` (offline, reusing `x1_metrics` and `c_metrics`
loaders), written before the claim runs. Written and checked (`run_experiment.py --check`: ok, with the usual
inertia warning of every 0.86 kg config; each params file loads through `bringup.profiles`):
`configs/experiments/x8_hub_{ours,port,portu,quanu}.yaml`, `x8_rim_{ours,port,portu}.yaml`;
`configs/twin/x8_hub_{ours,port,quan}.yaml`, `x8_rim_{ours,port}.yaml`.

### The check: how the network meets a loss (code at week8 5a7ccd0)
| Question | Answer | Where |
|---|---|---|
| Can the runner release a carrier in the network mode? | Yes. RELEASE publishes `/drone_<k>/detach` (Empty) and nothing else; the network graph bridges it to the world's joint like the detach mode (`network` only adds the velocity loop's and the network's knobs); world C and the rim world have a DetachableJoint per rod | `tools/experiment/config.py:40`, `runner_node.py:217, 757-761`; `bringup/graph_detach.py:48, 72-89, 171-178`; `four_rigid_ground_rig_hubcom_L100.sdf:424-463` |
| Does anything notice it? | No. The network phase returns before the detector, and the detector runs only before the hand-over with `reconfig_mode ocp` (the network mode sets `network`); the trackers' `/fleet/drop` is ignored after the hand-over | `dissipative_node.py:1160-1165, 1960, 1804-1811`; `bringup/config/sim.yaml:57-66` |
| Can it be told? | Yes. `/fleet/detach k` in the network phase zeroes the node (`net.detach`), marks the quad detached, opens its joint (harmless if open) and refuses below `min_survivors` (3 here: 4 -> 3 passes). The runner's DETACH event sends it | `dissipative_node.py:1568-1633`; `dissipative_network.py:215-221`; `runner_node.py:749-751` |
| The port with n-1? | Fixed topology: the ring rest, the even-azimuth slots and each node's share (m_L g / (n_attached sin phi)) follow `n_attached`, which only `net.detach` changes. Never told, the node stays: the survivors keep a quarter share each (the common-mode trim, `diss_ki_load` 0.5, bounded at 2 m/s^2, makes up the rest) and the leaver keeps its slot and its modelled pull, because the pull's gate reads only the rod's measured length (a released rod reads >= 1 m, gate 1) | `dissipative_network.py:269-300, 700, 723`; `dissipative_node.py:4408` (gates); `mpc_planner/planner_node.py:2046-2056` |
| The exact law with n-1? | n is fixed at the hand-over (rest lengths l_ij(0) for all four); a robot leaves another's sums only when it is over `quan_range_m` (2 m) away (Eq. 5), and each robot's modelled pull is m_L / n_i with n_i the robots it sees. The released quad's own pull is gated only by its distance from the ring (>= 0.85 L: gate 1), so it keeps commanding a pull its cable no longer gives | `quan_law.py:138-169, 174-177, 273-282, 284-300`; `dissipative_node.py:4480-4492` |
| What flies the released quad? | The trackers' velocity loop (the network mode, after the hand-over) has no measured pull: the planner's modelled pull is the feed-forward. The quan arm has no position term (`vel_kp_pos` 0, R7b). Any quad over 3 m/s is an envelope fault: the fleet disarms. A detached quad's reference is 0.6 m above its hand-over node, a position | `tracker_node.py:1391-1423`; `velocity_loop.py:226`; `utility_objects/safety.py:58, 173-174`; `dissipative_node.py:4483-4485, 4551-4560`; `dissipative_network.py:795-806` |

So: the port flies a loss as built (told: its own node drop; never told: the neighbour). The exact law does not:
never told, its released quad flies off on the modelled pull (offline: out of range at 0.9-1.4 s, over 3 m/s at
1.0-1.6 s, a fleet abort); told, the quad's velocity reference is zero and its loop has no position term, so it stops
near where it let go (it never reaches its 0.6 m point), well inside the survivors' 2 m range (0.7-1.4 m at 45-60 deg
cables): they never drop it and keep a quarter share each (estimate, not flown: the ring sinks until the robot-node
springs, k_i 1 N/m, carry the missing ~0.7 N per survivor). Neither is the paper's experiment, where robot 2 knows it
left and flies out of the others' range.

### Code needed (the exact law's arm only)
- **What.** Under `net_law quan`, a quad the planner is told has left (the runner's DETACH 0.35 s after the release, as
  the port) flies out of every survivor's perception range: a velocity reference straight up (<= 1 m/s, no modelled
  pull) until it is `quan_range_m` + 0.5 m from every survivor, then a hold; LAND brings it down as today. The
  survivors' law is untouched: they drop it from their sums (Eq. 5) when it passes 2 m and their shares rise from
  m_L/4 to m_L/3, which is the paper's own mechanism.
- **Where.** `dissipative_node.py` `_quan_plan`, the detached branch (5a7ccd0: 4483-4485), behind a twin-only switch
  (e.g. `quan_leave`, default off: today's 0.6 m point, bit for bit), declared in `common.yaml` and
  `graph_detach.py`'s `QUAN_KNOBS` (the network mode is twin only, `graph_detach.py:54-55`).
- **How big.** ~20 lines + 2 planner tests (off unchanged; on: the leaver climbs at <= 1 m/s with no pull until out of
  every survivor's range, then holds) + the launch test; pytest, `tools/gate.sh --quick`, one smoke on world C, then 2
  claim runs. Before the rim's exact-law arm, one fixed-team smoke: the law has never hovered the rim ring.
- **Critic.** It is flight-loop code (a departed quad's reference), not the law: decisions 30 Oct ~12:35 give X8's
  network loss handling the critic "if it touches the law", which this does not, so it may be waived (Wesley's word,
  noted in the registry row). Not recommended instead: flying the exact law unaware on a measured pull (the paper's
  robots cancel the true tension) changes R6c, the reading behind the thesis row, so it would need the critic and a
  re-fly of that row.
- **If it fails** (the leaver's climb disturbs the survivors, or the law loses the ring for reasons of our stand-in):
  the switch and its tests come out in one commit, and the thesis states that the exact law's loss was not tested.

## Critic
Not run: no flight code in the recommended plan (the arms are configs, the scorer is offline tooling). The exact law's
arm gets one unless Wesley waives it (Code needed).

## Decisions for Wesley (recommendation first)
1. **Scope.** (a) Fly ours against the port now, 15 runs, no code; the exact law's loss row to P3 (the thesis says it
   was not tested through a loss, and why). (b) Build the leaver's fly-out above (~20 lines, critic waived?) and add
   the exact law, ~+3 runs on world C. (c) All of X8 to P3, as the rescope's risk line says. Recommend (a).
2. **The port's arm: told 0.35 s after the release** (the authors' own failure model, init.m's zeroed rows, with the
   information our planner has at the same moment), the never-told port beside it. Push-back on the plan's literal
   "not told": never told, the port never drops the node at all; that is X1's no-resize for the network, not the
   paper's mechanism. Recommend told.
3. **The rim's tilt rules** as X1's sqT cell (the trackers' 80 deg drop, the 60 deg ring envelope off), and
   `drop_gap_deg 200` in both cells so our planner flies the T. Recommend yes (at 30/60 deg our planner drops the ring
   by rule on the rim and nothing is compared).
4. **Freeze the bars** L1-L3 as written (20 mm margin; settle 10 s on world C).
