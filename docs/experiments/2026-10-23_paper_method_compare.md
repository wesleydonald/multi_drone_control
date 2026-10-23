# Card: thesis result C, the paper's method against our planner, in the twin (23 Oct 2026)

Owner Wesley. Drafted Thu 22 Oct ~17:00 in the sim-C worktree (nothing flown). Origin: the 21 Oct plan
(docs/plan_thesis_results_2026-10-21.md, result C), the push design (docs/analysis/push_1022_plan.md, sim-C; full design
results/analysis/push_1022_designs.json). Decisions: docs/decisions.md, DECIDED 2026-10-22 push answers (21) both
network forms, (22) the headline and the hover window, (23) the twin network mode's values, (7) one frozen tree.
**Status: DRAFT; every bar PROPOSED.** The critic's challenge (22 Oct) is below the card, answered in "Response (22
Oct)"; the card is rewritten to it (main tree, 22 Oct late). The bars in §5 freeze when Wesley approves the card,
before the first C twin run. The three new switches (§8) stay off until then; the as-ported smoke runs may go first.

## 1. Question
How closely does the paper's method (Quan et al. 2025: a spring-damper network driving each quad through a velocity
loop; here our port of it, in two forms) carry the 1 m ring compared with our planner, in hover and on a line at 0.2
and 0.4 m/s, in the twin? (Wesley 22 Oct: fly both network forms; the headline is the clock-free path error plus a
reach term at the line's ends, with lag and the time-indexed error beside it; the hover is scored from the hand-over,
its first 5 s apart.)

## 2. Hypothesis
- Our planner follows the path at least as closely as either network form in every cell: a lower headline on both
  lines and a lower 3-D error in the hover.
- The port (damping against nodes it treats as still) runs late on a line and stops short of the ends; the paper's
  form is about on time and runs past the ends.
- Expected numbers:

| Arm | Hover, 3-D error from the hand-over | Line 0.4 m/s: clock-free / reach / lag | Source |
|---|---|---|---|
| Our planner | 1-2 mm, ring tilt peak 0.2 deg | ~6 / ~20-30 mm short / ~0.05 s | twin, 1 m: 4-quad hover R2348, R2351; 3-quad line_x at 0.3 R2079 (6.0 / 21.1 mm short / 0.05 s) and 0.125 R2013, R2077 (1.8 / 5.3 / 0.10 s), scored by tools/c_metrics.py (22 Oct late, with the critic's fixes) |
| Our port | 9 mm | 7 / 145 mm short / 1.0 s | offline, exploratory gains (§3.4); the design's probe: 6 mm / ~0 / 0.47 s at today's gains, but unkicked (§3.4) |
| The paper's form | 9 mm | 20 / 54 mm past the ends / -0.02 s | offline, exploratory gains (§3.4) |

- **The risk, found offline (§3.4):** at today's gains both forms capsize the ring in the harness 12-16 s after the
  hand-over: the ring's yaw grows (x1.6 per second from a 0.5 deg kick), because the formation turns with the ring's
  measured yaw. The twin smoke runs (§3.5) are the first check. If the twin does the same, the retune (§3.3) has to
  find gains that pass its guards, or C reports the port as unable to hold the ring at this geometry.
- **Tried before:** the network has failed on the 0.86 kg ring (R0498, R0500 aborted; R0517, R0519, R0521 capsized 8 s
  after a weld) and has never flown the rig-twin plant. The yaw capsize comes from the port's yaw-following formation
  (the locked yaw-only design), so a failure to hold the ring describes our port, never Quan et al.'s method.
- **The harness may be wrong:** its SIL plant has shown a growing 4-ring mode Gazebo did not (decisions 2026-09-25,
  R0510: Gazebo decides), and it hangs the rods from the drone centres (no 4 cm pivot lever). The smoke hover's reading
  is pre-registered in §3.3.

## 3. The one variable: the method after the lift
The method after the lift, set by the launch mode: the reference generator and the loop under it together (our OCP on
the MPC trackers with the model integral, against the network on the trackers' velocity loop with no per-quad
integral). The table compares the two methods as flown; it cannot pin a difference on the network alone. Everything
before the hand-over is our planner in every arm (the creep, our OCP's lift, its 5 s hover hold).

| Arm | Mode | After the lift | Configs (configs/experiments/) |
|---|---|---|---|
| Our planner | `dissipative` (the detach mode B flies; nothing detaches) | our OCP and the MPC trackers | `c_pl_hover_L100`, `c_pl_linex_v200_L100`, `c_pl_linex_v400_L100` |
| Our port of Quan et al. | `network` | the network on the trackers' velocity loop | `c_net_hover_L100`, `c_net_linex_v200_L100`, `c_net_linex_v400_L100` |
| The paper's form | `network` + Eq. 9 damping + Eq. 10 acceleration | as the port | (`c_netp_hover_L100`), `c_netp_linex_v200_L100`, `c_netp_linex_v400_L100` |

- The two network forms are one law in hover (both terms act only while the target moves), but only at the same gains,
  and each form is retuned on its own (§3.3). If the two forms freeze different gains, `c_netp_hover_L100` (the paper's
  params file) flies 3 runs as its own cell. If they freeze identical gains it is not flown: `c_net_hover_L100` stands
  for both forms' hover cell, and the table says so.
- **Pinned in every arm:** the B 4-quad configs (`b4_*_L100_ema`, 22 Oct):
  - world `four_rigid_ground_rig_L100.sdf`, quads on plates 0/3/6/9, 1 m cables, target 1.0 m;
  - the twin plant: the rig map, offsets 0.181/0.185/0.187/0.187, pack 23.8 V;
  - the carriers' velocity filter: `ema` now; the claim types the rig's winner after Wed 28's A/B, every carrier
    listed in `MDC_MOCAP_VEL_DRONES`;
  - the params file `configs/twin/c_compare.yaml`: B's detection, step-out, drop and land and 5 s hold, plus the
    network's two plumbing switches, which the dissipative graph does not pass;
  - the configs of one case differ only in `mode` (and the paper's form in its params file,
    `configs/twin/c_compare_paper.yaml`);
  - `drone_mass` 0.55 (the twin's model; 0.583 is the rig's). A host's planner cache can hold the rig's 0.583 solvers,
    so before the first C batch on a host: `python3 tools/prebuild_planner.py --from-config
    configs/experiments/c_pl_hover_L100.yaml` (and `c_net_hover_n3_L100.yaml` for the 3-quad neighbour).
- **B's loss detector** (`detach_detect_m` 0.06, `drop_on_loss`) comes with the params file but acts in our arm only:
  the network node turns drop and land off under `reconfig_mode network`. A trip in our arm is a loss (§5), named in
  the row, so a false one shows; `c_pl_linex_v400_L100` flies early (§3.5) because our planner has never flown the 0.4
  line at 1 m.
- **The line:** `line_x` 0.9 m out from the start and back (sinusoidal, at rest at both ends), whole cycles: 4 at
  0.2 m/s and 8 at 0.4 m/s, 56.55 s each. Why 0.9 m: Wesley's +-1.75 m position limit (22 Oct). A 1 m line from the
  origin puts the outboard quad at x 1.725 m; at 0.9 m it peaks at 1.625 m, as the rig's figure-8 does.

### 3.1 Our port's deviations from the paper (Quan et al., arXiv 2509.03563, Eq. 3, 7-10)
The thesis calls the arm "our port of Quan et al.'s method" and lists these:
- **Nodes:** integrated virtual nodes, sprung to a payload node pinned at the target and an anchor above it (the
  authors' simulation code). The paper's nodes are the kinematic projection of Eq. 3.
- **Damping:** against the pinned nodes as if they were still. Eq. 9's friction is relative to the formation centre.
  The paper's form switches it (`net_damp_relative`, the target's commanded velocity, open loop).
- **The command:** our trackers' velocity loop stands in for their autopilot. The thrust feedforward carries a
  modelled cable tension (m g / (n sin phi)), not their measured f_i. The expected acceleration of Eq. 10 is fed
  forward only in the paper's form (`net_traj_lean`, which also leans the cone onto it). `net_traj_lean` is a
  measured negative on the old plant (reference.md §6: no radius benefit, contraction worse, Aug 2026 A/B); the paper's
  form turns it on because Eq. 10 is the paper's, not as an improvement.
- **Rod lengths:** `net_twin_geometry` gives the network the rods the planner measured at the hand-over. Measured rods
  as the network's length are a measured negative too (reference.md §6, R0261-R0263: no benefit; that 0.53 m reading
  was drone-to-rim with the tip stand-off). The switch has its own bar (§5).
- **Yaw:** the formation turns with the ring's measured yaw. The paper's load is a point with no yaw.
- **Take-off:** our OCP creeps and lifts. The network takes over 1.5 s after the lift tops out, seeded at the measured
  quads, its datum shifted onto the measured ring.
- **Integrators:** one common-mode height trim (`diss_ki_load` 1.0) and none per quad (`vel_ki` 0).
- **Plant:** rigid rods (the twin).

### 3.2 The network mode on the twin (twin only; the rig never hands over)
- `sim.yaml` `modes.network`, Wesley's approved values: the automatic hand-over, the velocity loop after it,
  `vel_ki` 0, `diss_ki_load` 1.0, `diss_elev_deg` 60 (= the planned cable angle), no horizon preview (the loop reads
  node 0 only; the preview costs ~45 ms a tick), `handover_blend_s` 0, `reconfig_mode network`.
- **New switches, each off until Wesley approves the card's answer to its critic:**
  - `net_twin_geometry` (approved on): the references at the drone centre, the nodes on the measured pivots and rods;
  - `net_hold_carry`: the 5 s pre-path hold carried across the hand-over, so both arms start the path at the same
    time;
  - `net_damp_relative`: the paper's Eq. 9.
- **Drop and land:** off in the network arm (the node needs `reconfig_mode ocp` and says so in its log). The scorer
  applies the trackers' 30 deg ring-tilt rule to both arms the same way (§5).

### 3.3 The retune (pre-registered; the network forms only)
Our planner is not retuned: it flies the frozen tree's defaults, as B does. Each network form is retuned on its own.
- **When:** after the card is approved and the smoke runs, before the claim.
- **Time box:**
  - one offline search: an evening on the lab PC, or several hours on the laptop with no Gazebo run live (the cases
    now fly the claim's cycles: one candidate, both forms, took 2.2 min on 2 laptop cores);
  - then at most 11 twin runs: the twin confirmation below (2 candidates x 2 forms x 2 cases = 8) and today's gains
    (the hover once, one law at the same gains, and `line_x` 0.4 once per form = 3);
  - plus at most 6 more if the offline and twin rankings disagree.
- **Freezing:** the values are frozen with Wesley's word. Values that come after the code freeze go in a params file,
  so the tree does not change.
- **Rankings disagree** when, for a form, the twin's better candidate (guards first, then J) is not the offline better
  one, or neither passes the twin's guards. Then that form's next offline candidates fly hover and `line_x` 0.4 once
  each: 3 candidates (6 runs) at most over both forms.
- **The smoke hover's reading, pre-registered:** if the twin smoke hover at today's gains holds (the ring's yaw within
  10 deg over the 60 s and `yaw_growth_deg_s` at most 0.01), the harness's yaw finding is a SIL artefact (as the 4-ring
  rock was, decisions 2026-09-25). Then stage 1 is dropped, stage 2 starts from today's gains, and the twin ranks the
  candidates; the offline J only picks which fly.
- **Offline:** `tools/network_tune.py --search`.
  - The plant: the SIL plant at C's geometry (the rig map with each twin quad's own offset, the 23.8 V pack, the `ema`
    filter at 120 Hz).
  - The network and the loop: the real network at 10 Hz and the real velocity loop at 50 Hz.
  - The start: settled on the taut 60 deg cone, the ring 0.5 deg off in yaw.
- **Cases** (the claim's windows, so a mode that grows over 12-25 s cannot hide in a short case):
  - hover, 60 s;
  - `line_x` 0.9 m at 0.2, 0.4 and 0.6 m/s, the claim's whole cycles: 4, 8 and 8 (56.55, 56.55 and 37.70 s);
  - the margin flight: `line_x` 0.4, 8 cycles, with `vel_kp_pos` and `vel_kv` both x1.25.
- **Search space:**

| Knob | Values | Today |
|---|---|---|
| `diss_c` | 3, 4.5, 6, 9, 12 | 6 |
| `diss_k_slot` | 9, 18, 36 | 18 |
| `diss_k_pay` = `diss_k_anchor` | 20, 40, 80 | 40 |
| `diss_k_ring` | 10, 20, 40 | 20 |
| `vel_kp_pos` | 1, 1.5, 2, 2.5 | 2 |
| `vel_kv` | 3, 4, 5 | 4 |
| `vel_k_att` | 6, 8, 12, 16 | 8 |
| `diss_ki_load` | 0.5, 1, 2 | 1 |

- **Stage 1:** every (`diss_c`, `vel_kp_pos`) pair with the rest at today's values: the two knobs the yaw mode turned
  on (§3.4).
- **Stage 2:** coordinate descent over all eight knobs, in the table's order, two passes, from stage 1's best. Per
  knob, every value with the rest held; a candidate is kept when it ranks better: fewer guard failures first, then the
  lower J, then the later divergence. Among candidates that pass every guard that is the lowest J.
- **Where stage 2 starts:** stage 1's best point that passes every guard. If no grid point passes, the grid point with
  the fewest guard failures (ties: the lower J, then the later divergence); the descent then walks toward feasibility
  on the same ranking. If nothing passes at the end, the two candidates with the fewest guard failures per form fly the
  twin confirmation anyway: the twin decides.
- **Objective J:** the clock-free path error RMS plus the reach RMS on `line_x` 0.4 (mm), as §5's headline. Lag and
  the time-indexed error are reported, not optimised.
- **Guards** (every case and the margin flight; a candidate failing any is out, whatever its J):
  - no divergence;
  - ring tilt peak at most 15 deg;
  - quad tilt at most 35 deg;
  - throttle at most 0.78;
  - the ring's yaw within 10 deg of where it started;
  - the ring's yaw envelope not growing: `yaw_growth_deg_s` (the largest |yaw| in the window's last quarter minus that
    in its second quarter, over the time between them; each quarter holds whole line cycles) at most 0.01 deg/s;
  - the hover's 3-D error at most today's + 2 mm (if today's gains fail a guard: stage 1's best feasible point's).
- **Twin confirmation:** the 2 best offline candidates per form, and today's gains as the reference, fly hover and
  `line_x` 0.4 once each (`c_net*_tune_<cand>` configs with a params file per candidate). They are scored by
  `tools/c_metrics.py` on J and the guards; in the twin the ring's yaw change is `yaw_excursion_deg`. The best form-wise
  passing candidate is frozen. If the two forms freeze identical gains, the hover cell is shared (§3).
- **If nothing passes in the twin:** that form flies the claim at today's gains, and the table reports how its runs
  failed (§5). No structural change and no knob outside the table without a new card.

### 3.4 The offline sanity pass (exploratory, 22 Oct; `tools/network_tune.py`; not a result)
The harness as first built, at the cases of `--quick`: hover 60 s, `line_x` 0.2 and 0.4 (2 cycles then; now the
claim's), the margin flight. The ring starts 0.5 deg off in yaw.

Line cells: clock-free RMS / reach RMS (its mean: + short, - past the ends) / lag.

| Gains | Form | Hover | Line 0.2 | Line 0.4 | Margin flight |
|---|---|---|---|---|---|
| today's | port | capsized at 15.4 s | capsized at 12.1 s | capsized at 12.1 s | capsized at 7.3 s |
| today's | paper's form | capsized at 15.4 s | capsized at 16.1 s | capsized at 11.9 s | capsized at 7.6 s |
| `diss_c` 12, `vel_kp_pos` 1 | port | 8.7 mm, tilt 2.2 deg, yaw 1.0 deg | 4.9 / 59.5 (+59) mm / +1.30 s | 7.4 / 145 (+145) mm / +1.04 s | capsized at 17.4 s |
| `diss_c` 12, `vel_kp_pos` 1 | paper's form | as the port | 5.3 / 11.0 (-9) mm / -0.02 s | 19.5 / 54.0 (-54) mm / -0.02 s | capsized at 19.9 s |

- **The mechanism:** the ring's yaw. Kicked 0.5 deg it grows x1.5-1.6 a second at today's gains, then the ring
  spins and capsizes. It does the same in the design's own probe once that ring is kicked. The probe's stable numbers
  came from a perfectly symmetric start, which never excites the mode.
- **What it turns on** (30 s hovers, one knob or pair changed):
  - holding the formation at the hand-over's yaw slows it (x1.1-1.3 a second) but does not stop it;
  - `diss_k_slot` 9 or 36, or `diss_k_pay` 80 with `diss_k_ring` 40, do not stop it;
  - `diss_c` 12 alone delays the capsize to 25 s;
  - `diss_c` 12 with `vel_kp_pos` 1 stops it (the yaw stays within 1 deg). Hence stage 1 of §3.3.
- **Not a candidate:** `diss_c` 12 with `vel_kp_pos` 1 fails the margin guard in both forms.
- **At the claim's cycles** (22 Oct late, the critic's run 3; 4 cycles at 0.2, 8 at 0.4 and 0.6, the clock-free error
  without the along-line term; `results/analysis/network_tune_one_2026-10-22_c12kp1_claimcycles.json`). The point holds
  for the whole windows and its yaw does not grow, but it is still not a candidate: the margin flight diverges, and at
  0.6 m/s the ring tilts past the 15 deg guard.

| `diss_c` 12, `vel_kp_pos` 1 | Hover | Line 0.2 | Line 0.4 | Line 0.6, ring tilt peak | Margin flight | Ring yaw peak / growth |
|---|---|---|---|---|---|---|
| port | 8.7 mm | 4.7 / 61.0 (+60.6) mm / +1.30 s | 7.2 / 144.4 (+144.3) mm / +1.01 s | 15.3 deg | diverged at 17.4 s | 0.96 deg / -0.001 deg/s |
| paper's form | 8.7 mm | 5.0 / 10.3 (-7.7) mm / -0.02 s | 5.4 / 50.9 (-50.4) mm / -0.02 s | 20.0 deg | diverged at 19.9 s | 0.96 deg / -0.000 deg/s |

- **The network alone** (`verify_dissipative --geometry c`: tests A-D on the point-mass proxy plant at 1 m and
  60 deg, symmetric start, no yaw kick): it holds and lifts the ring, keeps the 4->3 detach aloft and stays outward.
- **Compute** (`--bench`, this laptop under load): the network's step 2.0 ms, with the 21-node preview 45 ms, the
  velocity loop 0.07 ms. Our planner's tick at n 4 and 5 iterations: 48 ms (solve 38 + extraction 10;
  results/analysis/solve_scaling_tuf_2026-10-22.json).

### 3.5 Order and hosts
1. The critic on §8: done 22 Oct, answered below the card. Until Wesley approves the card, the three switches stay
   off.
2. **Smoke, 2 runs, laptop** (as ported, `configs/rig/drop_and_land_hold5.yaml`, no new switch):
   `c_net_smoke_hover_L100`, then `c_net_smoke_linex_v200_L100`. The line's path starts at the hand-over (no hold
   carried); its LAND is timed from the hand-over line. Read the hover as §3.3 pre-registers.
   Then `c_net_smoke_hover_twingeo_L100` once (`net_twin_geometry` on, `configs/twin/c_smoke_twin_geometry.yaml`): the
   switch's own bar (§5) against the smoke hover.
3. The retune (§3.3). Then `c_netr_linex_v400_L100` once at the port's frozen gains
   (`configs/twin/c_compare_rel.yaml`): `net_damp_relative`'s own bar (§5).
4. **Neighbours of the frozen network arm, once each, exploratory:**
   - `c_net_linex_v600_L100` (faster);
   - `c_net_hover_n3_L100` (one fewer quad);
   - `c_net_linex_v400_L100_slope` (the other filter).
   And `c_pl_linex_v400_L100` once early: our planner has never flown `line_x` at 0.4 at 1 m.
5. **The claim, lab PC** (4 slots, `who` / load / containers checked first): 8 configs x 3 runs = 24 (27 with
   `c_netp_hover_L100`, §3), arms and cases interleaved in every 4-slot wave, on one frozen tree with R, U and S.
6. `tools/c_metrics.py --table <runs>`, then `/report`, then the reviewer.

## 4. Baseline
- **Our planner's arm (`c_pl_*`) is the baseline.** It flies in the same batches, on the same tree.
- **Expectation for it** (scored by `tools/c_metrics.py`, read-only):
  - 4-quad hover R2348, R2351: 1.3 / 1.4 mm (their LAND at 40 s cut the 60 s window);
  - 3-quad 1 m `line_x`:
    - R2079 at 0.3 m/s: clock-free 6.0 mm, reach 21.1 mm (15.7 short on average), lag +0.05 s, ring tilt peak
      3.8 deg;
    - R2013 and R2077 at 0.125 m/s: 1.8 / 1.8 mm, reach 5.3 mm, lag +0.08 / +0.12 s.
  - re-scored 22 Oct late with the critic's fixes (the clock-free error without the along-line term, the target from
    before the path); R2079's clock-free read 5.3 mm before.
- **What C replaces:** the legacy network numbers of thesis Appendix F: the circle 18 % small and 0.68 s late, and
  tab:detach_ab, both on the old plant. Inventory row D9 (LEGACY).

## 5. Pass / fail numbers (PROPOSED; frozen when Wesley approves the card, before any C run)
**Windows** (`tools/c_metrics.py`; events on the run clock):
- **Hover:** from the hand-over (`NET_HANDOVER`) for 60 s, never past LAND. Our planner hands over to nothing, so its
  window starts where the network arm's would: the reference's top + `auto_handover_settle_s` (1.5 s). The first 5 s
  are reported apart. Every network run's `NET_HANDOVER` must fall within 0.15 s of the top + 1.5 s, else a note.
- **Line:** from `PATH_START` (the planner's "hover hold done" line, in both arms with the hold carried) over the
  whole cycles that fit before LAND (4 at 0.2 m/s, 8 at 0.4).

**Metrics** (mm unless stated):
- **The target:** both arms are scored against the target from just before the hand-over (the reference in the second
  before `NET_HANDOVER`; our planner's at the top + 1.5 s; on a line whose path started first, before the path). The network moves its datum onto the measured ring at the
  hand-over and our planner does not, so this counts each arm's standing offset the same way. Each arm's error to its
  own published target is reported beside it (`err_own_rms_mm`, `headline_own_mm`), never tested.
- **Headline, line:** clock-free path error RMS + reach RMS.
  - The clock-free error: each sample's distance to the line the reference traces (sideways and height). An
    overshoot past an end is not counted here, because the reach counts it.
  - The reach: per commanded reversal inside the window, the end minus the ring's furthest x toward it within half a
    period of the reversal (to the window's end). + = short, - = past. Half a period, so a ring up to T/2 late still
    turns inside the window (a quarter is only 1.18 s at 0.6 m/s).
  - The reach is needed because the clock-free error cannot see a line flown short (offline: 6.8 cm short read
    10 mm).
- **Beside it, line:** the lag (s, the ring's x against the continuous schedule through the published points); the 3-D
  RMSE against that schedule; the amplitude ratio.
- **Hover:** the 3-D error to the target, RMS, over the window and over its first 5 s; the height error, mean.
- **Both:**
  - ring tilt, mean and peak (deg);
  - the ring's largest yaw change in the window (deg);
  - quad tilt peak (deg) and throttle peak;
  - the network's datum shift at the hand-over, and the ring's dip and tilt peak in the 5 s after it;
  - the path's start after the top (`path_start_s`, PATH_START minus the top);
  - the planner's `tick_ms` and the trackers' `solve_ms` (p50, p95; the velocity loop logs no rows) beside the
    offline bench (§3.4). The in-run numbers carry the lab's 4-slot contention.

**A lost run** (the method failed: scored, never re-flown; checked by the scorer, the same for every arm). Before the
window ends:
- a fleet abort;
- ring tilt over 30 deg on 2 consecutive samples (the trackers' drop rule; only our planner's arm acts on it);
- a quad over 60 deg;
- a "DROP AND LAND", "ENVELOPE FAULT", "UNANNOUNCED DETACH" (our arm's loss detector, named in the row) or "FLEET
  ABORT" line in the log before the planner heard LAND.

A lost run stays in its cell: it counts in the cell's runs and failures, ranks behind every run of the other arm that
was not lost, and its numbers are left out of the cell's mean and range.

**A VOID run** (plumbing only; re-flown once): fewer than 95 % of the window's rows with no loss, or a network run with
no hand-over (no `NET_HANDOVER`). A cell short of 3 valid runs after its re-flies is reported with its n.

**Where a run ends (the network's LAND):**
- Scoring ends at LAND at the latest.
- The network lands by lowering its target to the floor; at the floor `/fleet/landed` disarms the quads, still on
  the cone ~0.9 m up, and they drop.
- Anything after LAND (aborts, quad tilt, the runner's own FAIL) is a note, never a VOID.
- The landings are reported apart, not compared.

**The claim** (per cell, each network form against our planner, 3 valid runs each; `--table` prints it as `claim`):
- **The tested number is the headline alone:** the line's clock-free RMS + reach RMS; the hover's 3-D error.
- **Confirms "our planner follows the path at least as closely":** in every cell, every one of our planner's runs has
  a lower tested number than every run of that form, by at least the minimum effect.
- **Falsifies it:** any cell where every run of a form has a lower tested number than every one of ours, by at least
  the minimum effect.
- **Otherwise:** "no difference shown" for that cell.
- **Reported beside it, never tested:** the clock-free RMS and the reach RMS apart, the lag, the time-indexed error,
  the own-target numbers, the tilt and the compute.
- **Minimum effect:** the gap between the two arms' ranges (the worse arm's best run against the better arm's worst)
  at least the larger of 1 mm and 10 % of the better arm's mean. On a near-deterministic twin, separated ranges alone
  can be a 0.1 mm difference.

**The switches' own bars** (each switch alone, one run each; a switch that fails its bar is deleted, §8):
- **`net_twin_geometry`:** `c_net_smoke_hover_twingeo_L100` (on) against `c_net_smoke_hover_L100` (off), today's
  gains. It passes if, with it on:
  - the ring's dip in the 5 s after the hand-over is at most off's + 2 mm;
  - the ring's tilt peak in those 5 s is at most off's + 0.5 deg;
  - the size of the hover's mean height error is at most off's + 2 mm.

  A loss with it on and not off fails it.
- **`net_damp_relative`:** `c_netr_linex_v400_L100` (on, without `net_traj_lean`) against `c_net_linex_v400_L100`
  (off), the port's frozen gains. It passes if, with it on:
  - the lag is at most half the port's (the term's purpose: the formation moving with the target);
  - the headline is at most the port's + 2 mm;
  - the ring's tilt peak is at most the port's + 0.5 deg.

  A loss with it on and not off fails it.
- **`net_hold_carry`:** on every network line run (the retune's twin confirmation and the claim), `path_start_s`
  within 0.1 s of our planner's arm in the same case (its runs' mean). If a run falls outside, the switch fails.
  Without the switch the network's path starts at the hand-over, and c_metrics takes the line from the reference's
  first move.

**Smoke (exploratory, one run each):**
- Falsified (the port does not fly the twin as ported) by any of: no hand-over; an abort or drop before LAND; ring tilt
  over 30 deg; the ring's yaw changing more than 10 deg in the window.
- The harness expects the yaw to fail (§3.4).
- Checked as well: the trackers switch to the velocity loop, the network phase's log rows, the hand-over 1.5 s after
  the top.

**Neighbours (exploratory, once each):** falsified by an abort, drop or envelope fault before LAND, or ring tilt over
30 deg.

## 6. Repeats
- 3 per claim cell (the 21 Oct plan's number).
- 1 for the smoke runs, the switches' bars and the neighbours.
- One re-fly per VOID run; a lost run is a result and is not re-flown.

## 7. Cost (twin; no cap on headless runs)
| Step | Runs | Host |
|---|---|---|
| Smoke | 2 | laptop |
| `net_twin_geometry`'s bar | 1 | laptop |
| Retune | 0 offline (no Gazebo), at most 11 + 6 twin | lab PC |
| `net_damp_relative`'s bar | 1 | lab PC |
| Neighbours, and `c_pl_linex_v400` once | 4 | lab PC |
| Claim | 24 (8 configs x 3); 27 with `c_netp_hover_L100` | lab PC, ~1.5 h |
| Optional C2 (an announced 4->3 detach, both arms; after R, if room) | 12 | lab PC |

## 8. New code?
**Yes.** All of it twin or offline; the rig never enters the network phase. What the rig's tree gets: two appended
log columns (the planner's `tick_ms`, the trackers' `solve_ms`), blank OCP columns in network rows, the runner's two
event names.

**Built (sim-C worktree, 22 Oct):**
- the twin network mode: `graph_detach.dissipative(network=True)`, `sim_network` legacy or twin, `LEGACY_ONLY` empty,
  `sim.yaml modes.network`, three switches in `common.yaml`;
- the planner's log rows through the network phase;
- `PATH_START` and `NET_HANDOVER` events, and the runner waits for references in mode network too;
- `metrics.segment_error` and `metrics.turnaround_reach`;
- `tools/c_metrics.py`, `tools/network_tune.py`, the thesis table (`thesis_sim_tables.table_compare`);
- `verify_dissipative --geometry c` (tests A-D at 1 m and 60 deg; the default run, the gate's step 5, is unchanged);
- the configs (`configs/experiments/c_*`, `configs/twin/c_compare*.yaml`), and the tests.

**After the critic (main tree, 22 Oct late; "Response (22 Oct)" below):**
- `tools/c_metrics.py`:
  - both arms against the target from before the hand-over, with the own-target numbers beside (M1);
  - the clock-free error without the along-line term (M2);
  - LOSS against VOID (M3), with the loss detector named (m3);
  - the claim on the headline alone (B1), with the minimum effect (m4);
  - the registry's baseline column;
  - `path_start_s` and the hand-over's tilt peak for the switches' bars (M5).
- `metrics`: `segment_error(past_end=False)`; `turnaround_reach` looks half a period around each reversal.
- `tools/network_tune.py`: the claim's cycles, the yaw-growth guard, and stage 2's start when no grid point passes
  (M4).
- `run_experiment.py --check` refuses a typed params_file that does not exist.
- Configs: `c_netp_hover_L100` (B2), `c_net_smoke_hover_twingeo_L100` and `c_netr_linex_v400_L100` with
  `configs/twin/c_smoke_twin_geometry.yaml` and `c_compare_rel.yaml` (M5).
- Merge with the main tree:
  - the network graph refuses `rejoin_departed` at launch (it does not bridge the joints);
  - the planner log's four C columns come after S's `res_ms`, which a network row leaves blank;
  - the gate's twin network pair is `four_rigid_ground_rig_L100.sdf:network`.

**The three switches the critic is asked to challenge** (control-law and reference changes in the flight loop):
1. **`net_damp_relative`** (`dissipative_network.py` `step`): the node-to-pinned-node damping becomes
   `-c (qd_i - v_d) 2`, `v_d` the target's commanded velocity (zero in holds, the descent on LAND). Off, the network
   is the one flown before, bit for bit (a golden test).
2. **`net_twin_geometry`** (`dissipative_node.py`):
   - each tethered node's reference moves from the rod's drone end to the drone centre: `centre_from_pivot` at the
     attitude its own thrust feedforward asks for, as the OCP's `drone_kinematics`;
   - at the hand-over the nodes are seeded at the measured pivots and take the measured rod lengths.
3. **`net_hold_carry`** (`dissipative_node.py`): the pre-path hover hold the OCP began (`traj_hold_s`) runs to its end
   in the network phase, with our planner's "hover hold done" line, so the path starts when it would under the OCP.

**Questions for the critic beyond the three:** the yaw mode (§3.4); scoring the 30 deg rule on an arm that does not
act on it; the network LAND as the run's end.

**If one fails:** a switch that fails its critic or its bar (§5) is deleted with its branch and its tests. The wiring,
logging, scorer and harness are not control laws and stay.

## Critic
*The project's critic, 22 Oct 2026 (read-only review of the card and the code as built, before any run).*

1. **Tried before?** Partly. The network has already failed on the 0.86 kg ring: R0498 and R0500 aborted, and R0517, R0519 and R0521 capsized 8 s after the weld. Two parts of this card are measured negatives in reference.md §6. `net_traj_lean` is one (Aug A/B: no radius benefit, contraction worse), and the paper's form turns it on. Measured rod length as the network length is the other (R0261–R0263), and `net_twin_geometry` brings it back (dissipative_node.py:3030-3032). Neither is cited. The network has never flown on the rig-twin plant.
2. **Simpler explanation:** the ring-yaw capsize that shapes the whole retune comes from the SIL plant. That plant has shown growing 4-ring modes before that Gazebo did not (decisions 2026-09-25, R0510: "Gazebo decides"). The harness also leaves out the 4 cm pivot lever (network_tune.py:18), which T0010 showed turns any position mismatch into torque. The card does not say what happens if the twin smoke hover holds.
3. **Falsifier:** a form beating ours on the headline would falsify the hypothesis. §5 also falsifies on clock-free error alone, which cannot see a line flown short, as §5 itself says. The same cell can then confirm and falsify at once. A capsize also counts as VOID, so the arm's own failures drop out of the table.
4. **What it breaks:** I found no leak into our planner's default graph:
   - the velocity-loop and network knobs pass only with `network=True` (graph_detach.py:72-80);
   - `net_damp_relative` off is bit-identical to before (the golden test);
   - the blank log columns depend on phase `network` (planner_node.py:925);
   - sim.yaml:35-37 keeps `reconfig_mode ocp` with no hand-over.

   The capsize comes from the locked "yaw-only" design, so a verdict that the network "cannot hold the ring" describes our port, not Quan et al.
5. **Asked for:** yes: the 21 Oct plan's result C and Wesley's answers (21)-(23). The yaw-driven stage-1 grid and the yaw guard are the implementer's additions, inside (23).

**Blockers (fix before the bars freeze)**
- **B1. The bars contradict each other.** §5 falsifies on "clock-free RMS alone", which a late, short port wins by construction. Offline at 0.4 the port scores 7.4 mm on clock-free while 145 mm short; ours is about 6 mm. One cell can then pass "confirm" on the headline and "falsify" on clock-free alone. Falsify on the headline only, and report the parts beside it.
- **B2. The shared hover cell.** §3.3 retunes each form separately, but `c_net_hover_L100` stands in for both forms. The two forms fly the same law in hover only if both freeze the same gains. Add `c_netp_hover_L100` if they differ.

**Major**
- **M1. Each arm is scored against a different target.** c_metrics scores each arm against its own published target: c_metrics.py:176,202 for hover, and x0, y, z at :223-225 for the line. The network moves its target onto the measured ring at the hand-over and never corrects it (dissipative_node.py:3045-3049). Its standing xy offset is therefore free, while ours counts. Score both arms against the target from just before the hand-over, with each arm's own-target number beside it. This bias favours the network: a confirmation survives it, a falsification does not.
- **M2. Overshoot is counted twice.** `segment_error` adds the distance past an end (metrics.py:305), and the reach counts it again; a shortfall counts once. That favours short arms (ours about 16 mm short, the port) over the paper's form (54 mm past). It is the one bias in favour of the hypothesis. Drop the along-line term from the clock-free error when the reach is added.
- **M3. A capsize is scored as VOID** (c_metrics.py:270-275, and §5's re-fly). A method failing is a result, not an invalid run, and a successful re-fly erases it. Keep VOID for plumbing faults (coverage, no hand-over). A capsize or abort before the window ends should score as a loss.
- **M4. The retune cannot see the claim's window.**
  - The offline line cases are 2 cycles (network_tune.py:74), which is 18 s at 0.4 m/s. The capsizes came at 12-25 s, and the claim window is 56.55 s.
  - A 10 deg yaw cap lets a slowly growing mode pass.
  - Fix: fly the claim's cycle counts offline, and guard on the yaw's growth rate (the slope of its envelope at most zero), not its excursion.
  - The verifier's open gap also remains: nothing says where stage 2 starts when no stage-1 point passes.
- **M5. No switch has a bar of its own,** though §8 deletes "a switch that fails its bar".
  - `net_twin_geometry` first flies inside the retune.
  - `net_damp_relative` never flies apart from `net_traj_lean`.
  - Proposed bars: twin geometry on against off, with the hover's hand-over dip and tilt and its mean height error no worse. Hold carry: PATH_START minus the top within 0.1 s of the planner arm's.

**Minor**
- **m1. "One variable" is really two:** the reference generator, and the loop under it (MPC tracker replaced by the velocity loop, with no model integral). The table cannot pin a loss on the network itself, so the card should say "the method after the lift".
- **m2. The twin budget does not add up.** 2 candidates × 2 forms × 2 cases, plus 3 runs at today's gains, is 11 runs against a cap of 10. "Rankings disagree" is not defined.
- **m3. Only our arm runs B's loss detector** (`detach_detect_m` 0.06), on a 0.4 m/s line it has never flown. A false loss would VOID our runs only.
- **m4. There is no minimum effect size.** On a nearly deterministic twin, "every run beats every run" can trigger on a 0.1 mm difference.

**Runs that expose these**
1. Smoke hover at today's gains (already planned). Pre-register the reading: if the ring's yaw stays under 10 deg for 60 s in the twin, the SIL finding is an artefact. Then drop stage 1 and rank candidates in the twin.
2. One hover with `net_twin_geometry` on, compared with run 1, for M5's bar.
3. Offline, no Gazebo: the stage-1 best flown for 8 cycles at 0.4 and 4 at 0.2 (M4).
4. Re-score run 1 against the target from just before the hand-over (M1). This needs no new run.
5. `c_pl_linex_v400_L100` early (already planned): it shows m3 and our reach's sign for M2.

CRITIC: rewrite the card. The bars can confirm and falsify the same cell, and each arm is scored against a different target; fix §5 and the hover cell before they freeze.

## Response (22 Oct)
*The integrator, in the main tree (day-1021, uncommitted), 22 Oct late.*
- **Fixed:** in the card and the tools now, with tests.
- **Accepted:** taken as the critic said; no code needed.
- **Needs Wesley:** his word when he approves the card.

Every bar stays PROPOSED.

| Finding | Fate | What changed |
|---|---|---|
| 1. Tried before | fixed | §2 cites the network's failures on the 0.86 kg ring (R0498, R0500, R0517, R0519, R0521). §3.1 cites both negatives (`net_traj_lean`, Aug A/B; measured rods, R0261-R0263). `net_twin_geometry` gets its own bar (M5). |
| 2. Simpler explanation | fixed | §2 names the SIL plant's history and the missing pivot lever. §3.3 pre-registers the smoke hover's reading: if it holds, the finding is a SIL artefact, stage 1 is dropped, and the twin ranks the candidates. |
| 3. Falsifier | fixed | See B1 and M3. |
| 4. Our port, not Quan et al. | accepted | §2: a failure to hold the ring describes our port's yaw-following formation, never Quan et al.'s method. |
| B1 | fixed | §5 tests the headline alone: the line's clock-free + reach, the hover's 3-D error. The parts are reported beside it, never tested. `c_metrics.claim` reads only that key (test). |
| B2 | fixed; needs Wesley | `c_netp_hover_L100` is written (the paper's params file). It flies 3 runs only if the forms freeze different gains; with identical gains the table says the cell is shared. `claim` and `latex` fall back to the port's hover only when the paper's was not flown. The 3 extra runs need his word. |
| M1 | fixed | `c_metrics` scores both arms against the target from just before the hand-over: the hover's 3-D error and height, and the line's ends, line and schedule. Each arm's own-target numbers sit beside it (`err_own_rms_mm`, `headline_own_mm`). A test gives a 12 mm standing offset to both arms: each scores 12 mm, and only our planner's own-target number shows it. The critic's run 4 is now the default scoring. |
| M2 | fixed | The headline's clock-free error drops the along-line term (`segment_error(past_end=False)`) in `c_metrics` and `network_tune`. A 36 mm overshoot now counts once, in the reach (test). |
| M3 | fixed | A failure before the window ends (abort, ring tilt over 30 deg, a quad over 60 deg, a drop, loss or envelope line) is a LOSS. It stays in the cell's count of runs and losses and ranks behind every run of the other arm that did not fail. VOID covers plumbing only: under 95 % of the rows with no loss, or no hand-over. The registry row reads LOSS or VOID. §5 and §6 are rewritten (tests). |
| M4 | fixed; the tolerance needs Wesley | `network_tune` flies the claim's cycles (4 / 8 / 8). A new guard, `yaw_growth_deg_s` (the envelope of the last quarter against the second, each over whole cycles), must stay at most 0.01 deg/s; the 10 deg cap stays. §3.3 now says where stage 2 starts when no grid point passes, and `search()` does it (tests). The critic's run 3 was flown offline (§3.4): the stage-1 point holds the claim's windows with its yaw not growing, but still fails the margin flight and, at 0.6 m/s, the tilt guard. |
| M5 | fixed; the tolerances need Wesley | §5 gives each switch its own bar, one run each. `net_twin_geometry`: on against off on the smoke hover (dip, tilt and height error). `net_damp_relative`: alone against the port on the 0.4 line (lag at most half, headline and tilt no worse). `net_hold_carry`: `path_start_s` within 0.1 s of our arm's. New configs: `c_net_smoke_hover_twingeo_L100` and `c_netr_linex_v400_L100`, with `configs/twin/c_smoke_twin_geometry.yaml` and `c_compare_rel.yaml`. |
| m1 | fixed | §3 is now "the method after the lift": the generator and the loop under it. The table cannot pin a difference on the network alone. |
| m2 | fixed; needs Wesley | The budget now adds up: 8 candidate runs plus 3 at today's gains makes 11, not 10, plus at most 6 more. §3.3 defines "rankings disagree". Raising the cap from 10 to 11 needs his word. |
| m3 | fixed in the scorer; needs Wesley | A trip of the loss detector is named in the row ("the loss detector"), so a false one shows, and `c_pl_linex_v400_L100` flies early. Proposed: our arm keeps B's detector on (it is our system as flown, the rig's too), and a trip counts as our loss. Whether the claim turns it off in our arm is his call. |
| m4 | fixed; the numbers need Wesley | Minimum effect: the gap between the arms' ranges must be at least the larger of 1 mm and 10 % of the better arm's mean. It is applied in `c_metrics` (`MIN_EFFECT_MM`, `MIN_EFFECT_FRAC`; test) and §5. |
| Runs that expose these | as listed | Run 1: §3.3's pre-registered reading. Run 2: `c_net_smoke_hover_twingeo_L100`. Run 3: flown offline (§3.4). Run 4: the default scoring (M1). Run 5: already in §3.5. |

## Results (23 Oct, scored 14:01 with `tools/c_metrics.py`; exploratory until our planner's cells and the claim repeats fly)
| Runs | Arm / point | Case | Headline (mm) | Ring yaw, largest change (deg) | Ring tilt peak (deg) | Verdict |
|---|---|---|---|---|---|---|
| R2446 | network as ported (today's gains) | hover | 60.9 (3-D RMS) | 124.7 | 44.81 | LOSS (tilt > 30 deg, fleet abort) |
| R2447 | network as ported | line 0.2 | 69.0 | 473.7 | 57.67 | LOSS (tilt > 30 deg, fleet abort) |
| R2468 | stage-1 point (diss_c 12, vel_kp_pos 1, the rest DEFAULTS) | hover | 12.0 | 58.3 | 2.22 | valid |
| R2469 | stage-1 point | line 0.4 | 156.0 | 60.5 | 10.81 | valid |
| R2685 / R2686 | **search's best** (c_net_portfinal) | hover | 18.7 / 16.1 | 51.1 / 22.3 | 12.85 / 8.42 | valid / valid |
| R2687 / R2688 | search's best | line 0.2 | 41.5 / 42.1 (lag 1.25 s) | 28.8 / 48.2 | 5.01 / 8.17 | valid / valid |
| R2689 / R2690 | search's best | line 0.4 | 104.5 / 99.3 (lag 1.13 s) | 13.9 / 14.3 | 11.17 / 16.39 | valid / valid |
| R2448 | our planner | line 0.4 | 46.2 (lag 0.03 s) | 0.2 | 3.52 | valid |
| R2733 / R2734 | our planner (batch 18u) | hover | 1.4 / 1.3 (3-D RMS) | 0.0 / 0.1 | 0.25 / 0.25 | valid / valid |
| R2735 / R2736 | our planner (batch 18u) | line 0.2 | 17.6 / 17.0 (lag 0.10 / 0.09 s) | 0.1 / 0.1 | 1.18 / 1.16 | valid / valid |
| R2737 | our planner (batch 18u) | line 0.4 | 44.3 (lag 0.03 s) | 0.2 | 3.64 | valid |

- **Batch 18u (scored 23 Oct 17:45):** every case now has both arms at n = 2 (line 0.4: R2448 + R2737). In each case
  every run of our planner beats every run of the search's best on the headline and the ring tilt peak: hover 1.3-1.4
  vs 16.1-18.7 mm; line 0.2 17.0-17.6 vs 41.5-42.1 mm; line 0.4 44.3-46.2 vs 99.3-104.5 mm. The tool's §5 claim
  check prints "confirms" in all three cases. Our planner's line 0.4 headline is almost all reach (42.7 mm RMS, short at
  the ends; clock-free 1.7 mm). Caveat: three patches on be34f95 in this table (R2448 aad7c59a, R2685-R2690 537551ba,
  R2733-R2737 cae5cf26); the claim needs one tree and the reviewer. `python3 tools/c_metrics.py --table R2685-R2690
  R2448 R2733-R2737` reproduces it.

- **Correction (23 Oct ~10:00):** R2468/R2469 are not the search's best. Its emit typed only `diss_c` and
  `vel_kp_pos`; the other knobs fell back to DEFAULTS (k_pay = k_anchor 40, k_ring 20, ki_load 1.0, vel_kv 4). The
  morning report's "the best point still capsizes" was wrong and is corrected there.
- **The real best** keeps the ring's yaw within ~14 deg on the 0.4 m/s line (61 deg at the stage-1 point) but still
  lets it swing 22-51 deg in hover; on the 0.4 m/s line its headline is 99-105 mm against our planner's 46 mm (2.2x),
  with ~1.1 s of lag against 0.03 s. Headline = clock-free path error + reach RMS (Wesley 22 Oct: score distance to
  the path, lag apart).
- **Pending (flown in batch 18u, R2733-R2737, rows above):** our planner's hover x2, line 0.2 x2 and line 0.4 x1, so every case
  has both arms at n = 2. Wesley (23 Oct morning): report C as is; the bars matter less than that it works.

## The paper's own geometry (Wesley 23 Oct)
**Bars FROZEN 23 Oct 16:22 (Wesley: G, P, A, O as written; the hub above the centre of mass as built; also the network as first ported on the hub, c_net_smoke_hub_{hover,linex_v200}_L100, x1 each).** (Drafted Fri 23 Oct ~16:10, nothing flown.) The world and the attach geometry are approved for this test
(decisions 2026-10-23, network yaw); the bars freeze with Wesley's word, before the first run. No flight-loop code, so
no critic; the reviewer on the final table, as C's.

**Question.** Does the network hold the ring when every cable hangs from its centre, as Quan et al. model it ("The
connection points of the flying robots and the load are located at the center of mass"; their Eq. 1 carries the load's
position only), and how closely does each method carry it there? On C's ring the rods hang from four rim plates
(r 0.225 m): the cables torque the ring, which gives it a yaw mode the paper has no term for, and our port turns the
formation with the ring's measured yaw (the locked yaw-only design), which feeds it (Results: 22-51 deg in hover at the
search's best gains; as ported, R2446/R2447 lost).

**The one variable: the attachment geometry, in both arms.** Every rod's payload end on a 1 mm hub at the ring's
centre, against C's four plates at r 0.225 m. The rest is C's cells: the quads' azimuths 0/90/180/270, 1 m rods from
the 4 cm pivot, the ring's mass and inertia, the twin plant, the `ema` filter, the params files, windows and events.
A hub config differs from its rim config in `world` and `attach_radius` (0.001) only, name and description aside.
- **The world**, `four_rigid_ground_rig_hub_L100.sdf` (`tools/make_carry_worlds.py`, generator `--hub-r 0.001`; its
  test in `test_rig_worlds.py`): the four rod joints 1 mm from the ring's axis on the plates' plane (the top face, 2 cm
  above the CoG); a hub and four spokes drawn, visual only; the plates still drawn on the rim. Ring mass, inertia and
  collision, rods, pivots and quads unchanged. The quads sit 0.50 m from the axis at 60 deg (rim 0.725 m), neighbours
  0.71 m apart (1.03 m).
- **Why 1 mm, not 0:** both controllers take each quad's azimuth from its attach point. At 0 the planner's cable
  references fall on two azimuths (0 and 180 deg) and every network slot on +x. At 1 mm only the lever changes: the
  cables' yaw arm is 1/225 of the rim's.
- **Why the top face, not the CoG:** the ring hangs 2 cm below the hub, as their payloads hang from S-shaped carabiner
  hooks, and `attach_z` stays 0.02, so the radius is the only change. At the CoG nothing torques the ring: whatever tilt
  rate it leaves the floor with stays, and our arm's 30 deg drop and the scorer's 30 deg loss rule would see it. Wesley's
  call: the CoG is `--attach-z 0` in the world and `attach_z: 0` in the six configs.
- **What the hub does to the ring:** nothing holds its yaw, and its tilt is a pendulum on the 2 cm drop (0.43 Hz; it
  leans atan(a/g), ~2 deg at the 0.4 line's peak acceleration). Its yaw and tilt here say little about either method.
- **How each arm takes it:** from `attach_radius` and `attach_z`. Our planner's attach points (planner_node.py:413) are
  a runtime parameter of the OCP: no rebuild (`prebuild_planner.py --dry-run` builds what C's configs build; the
  laptop's cache is fresh for n 2-4). The network takes the planner's points (dissipative_node.py:630-635); with
  `net_twin_geometry` its nodes are seeded at the measured pivots with the rods measured to them (:3696-3705). Its
  formation still turns with the ring's measured yaw, now with nothing to feed. Offline, its tests A-D
  (`verify_dissipative --geometry c` with the radius at 1 mm: lift and hold, outward hold, a 4->3 detach) pass.
- **Zero radius would need code** (not built; flight loop): each slot's azimuth read from `attach_azimuths_deg` (or the
  even ring) where it is now `atan2` of the attach point: geometry.py:182, planner_ocp.py:50, planner_node.py:310, 1300,
  1314, 1869, dissipative_network.py:317-322 (and the attach, tip and rejoin paths C does not fly).

**Our planner at the hub, offline** (exploratory, not a result). `geometry.balanced_tensions` gives the equal split
(2.435 N each) at 1 mm as on the rim: the moment rows fold onto the horizontal-force rows, rank unchanged. The OCP
against its own model (solve_scaling's loop: the next state is the plan's node 1), 4 quads, 1 m, 60 deg, no failed
solve in any case, hover exact:

| line_x, 0.2 / 0.4 m/s | Time-indexed RMS (mm) | Worst neighbour gap off 90 deg (deg) | Tension spread (N) | Ring tilt peak (deg) |
|---|---|---|---|---|
| Rim, r 0.225 | 14.1 / 27.3 | 0.8 / 3.0 | 0.03 / 0.13 | 0.5 / 1.6 |
| Hub, 1 mm on the top face | 14.0 / 25.8 | 1.1 / 4.0 | 0.06 / 0.22 | 1.2 / 4.8 (the pendulum) |

In latency_bench's plant (kinematic quads; its G1 failed, code paths only) the hub diverged where the rim holds: the
hover after ~14 s with the bench's default rods carrying their ring-end mass, the 0.4 line within ~3 s behind the
bench's 200 rad/s follower (at 1 mm, 1 cm and 5 cm alike). Stiffer or softer rods (hover) and a 30 rad/s follower (line,
bounded) removed them; switching off the OCP's attitude rows or telling it the plant's mass did not. They follow the
plant, not the planner: four position-held pivots on stiff rods meeting at one point carry tensions no plan can set.
The twin's trackers fly the planned thrust and pull, so the twin decides (the smoke pair below).

**Configs** (configs/experiments/; `run_experiment.py --check` ok for all six, with C's params.py inertia warning):

| Cell | Network: our port at the search's best gains (`c_net_portfinal.yaml`) | Our planner (`c_compare.yaml`) | Rim cell (C) |
|---|---|---|---|
| Hover | `c_net_hub_hover_L100` | `c_pl_hub_hover_L100` | `c_net_tune_portfinal_hover_L100` (R2685, R2686); `c_pl_hover_L100` |
| Line 0.2 m/s | `c_net_hub_linex_v200_L100` | `c_pl_hub_linex_v200_L100` | `c_net_tune_portfinal_linex_v200_L100` (R2687, R2688); `c_pl_linex_v200_L100` |
| Line 0.4 m/s | `c_net_hub_linex_v400_L100` | `c_pl_hub_linex_v400_L100` | `c_net_tune_portfinal_linex_v400_L100` (R2689, R2690); `c_pl_linex_v400_L100` (R2448) |

**Order (twin):**
1. Smoke, exploratory, once each: `c_pl_hub_hover_L100`, `c_net_hub_hover_L100`. Falsified (the hub does not fly as
   built) by no lift, no hand-over in the network arm, a loss before LAND, or a VOID.
2. The six configs x2 (12 runs; x3 if the hub cells join C's claim table), arms interleaved, on one tree with the rim
   cells: our planner's pending rim runs in the same batch, and the network's rim pairs again if flight code changed
   since batch 17.
3. `tools/c_metrics.py --table` on the hub runs alone (it groups runs by mode and case, not by world; its registry
   baseline names the rim config, so the rows name `c_pl_hub_*`), then `/report`, the reviewer.

**What each outcome means** (per cell, the network's hub pair against its rim pair):
- **The yaw problem is the geometry:** the network holds the centre-hooked ring and its headline falls to half its rim
  one or less. C's rim cells then compare the methods on a load the paper does not model, and the thesis says so beside
  them.
- **It is the port:** a hub run lost, or the headline still at least 80 % of the rim's. Without the yaw mode the network
  fails, or flies no better: look at the port itself (the velocity loop, the damping against still nodes, the gains, the
  hand-over).
- **Between the two:** "partly the geometry", with the numbers. A mixed result is likely: the hover gaining (its error
  came with the yaw swings), the lines not (their ~1.1 s lag is the port's damping, §3.1).
- **Our planner** should carry the hub as it does the rim: its formation holds the yaw datum, not the ring's yaw. A loss
  in our arm would say its model needs the cables' moment arms (the bench's warning above): a finding about our planner.

**Bars (PROPOSED).** Scored by `tools/c_metrics.py` exactly as C's cells (§5: windows, the target from before the
hand-over, the headline, LOSS and VOID); n = 2 per config; minimum effect as §5 (the larger of 1 mm and 10 % of the
better mean).

| Cell (the tested number, §5) | Network's rim pair (mean) | G: hub mean at most | P: hub mean at least |
|---|---|---|---|
| Hover (3-D error) | R2685, R2686 (17.4 mm) | 8.7 mm | 13.9 mm |
| Line 0.2 m/s (headline) | R2687, R2688 (41.8 mm) | 20.9 mm | 33.4 mm |
| Line 0.4 m/s (headline) | R2689, R2690 (101.9 mm) | 51.0 mm | 81.5 mm |

- **G, the geometry (per cell):** both network hub runs valid, their mean at most half the rim pair's (the table), and
  the two pairs' ranges apart by the minimum effect.
- **P, the port (per cell):** a network hub run lost, or its mean at least 80 % of the rim pair's. Neither G nor P:
  "partly the geometry".
- **A, the arms on the hub (per cell, §5's claim rule):** "our planner at least as close on the paper's own geometry"
  if both our runs are below both network runs by the minimum effect; "the network closer" the other way round;
  otherwise "no difference shown".
- **O, our planner flies the hub (per cell):** both runs valid; a loss falsifies it.
- **Reported, never tested:** the ring's yaw change (nothing holds it here) and tilt (a pendulum), the clock-free and
  reach parts, lag, the hand-over's dip and tilt, quad tilt, throttle, compute.

**The paper's own geometry, smoke pair (R2725, R2726; batch 18u; scored 16:59):** both valid (lift, the network's
hand-over, no loss, LANDED); the smoke falsifier did not fire, so the 12 cells and the as-ported pair fly (batch 18h).
Hover on the hub: our planner 1.4 mm, ring tilt peak 0.19 deg, yaw change 0.0 deg; the network at its best gains
11.8 mm (first 5 s 22.0), tilt peak 0.32 deg, **yaw change 0.1 deg** (on the rim at the same gains: 22-51 deg,
R2685/R2686). One run each, exploratory: the first sign that the rim attachment, not the port's code, drives the yaw.
