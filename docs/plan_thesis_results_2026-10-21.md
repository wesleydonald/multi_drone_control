# Plan: thesis results to the supervisor's spec (21 Oct 2026)

Approved by Wesley 21 Oct ~18:50. This is the current plan (CLAUDE.md). Status per work package:

| Section | Status | Commits / runs |
|---|---|---|
| Housekeeping (commits, branch day-1021) | done 21 Oct | 410f0f1 (day-1016), 6483d90 (tejen-sync2) |
| 1. Tonight for Thu 22 | done 20:30: line_spin, 2q/T/1 m worlds, drop_gap_deg (chain card, critic, arm 2 FAILED its rocking bar). Sheet docs/rig/thu_2026-10-22.md; Part C kept on it (Wesley ~23:00). Magnet detector rev 3 after critics 2 and 3 on branch tip-detect; its twin comparison flies overnight 22 Oct | 5fbdaba, d440664, 33723c2; tip-detect c56ad44; R2242-R2258 |
| 2. Thu 22 rig | done (review `docs/reports/rig_review_2026-10-22.md`): 3 quads fly well with the 5 s hold and the size-1.0 figure-8; the pair swings at 1.4 Hz unless the quads' velocity filter is `slope` (halved and decayed, full pair figure-8; reviewer weak, Wed A/B); the rig T hangs 2.6 deg; Part C's release failed on operations (RViz button next) | be34f95 + uncommitted spin law; RIG-1022-* |
| 3. 1 m switch and the re-fly (R) | 1 m defaults DONE 22-23 Oct (merged, gate green; rig paths 0.75 m, limit on in real.yaml, own rig solver folder). R re-fly DONE 23 Oct: **19 tables hold at 1 m (B4 marginal; C-D1 vertical and C-D4 empty), 5 do not, 2 unresolved, 1 not confirmed (B5), 0 skipped** (the five new 1 m worlds' Tier B, gain and push tables added 11:33, R2604-R2628, reviewer 2 [weak], corrections applied 11:47: at 1 m G165 crosses the 30 deg drop line 2/2, an S miss; G150p holds by 0.19 deg; first pass: 13 hold, 3 do not (T7t orbit_v300 mostly from the newer tree; C-L1, C-D5 as at 0.55 m), reviewer [weak] with corrections applied, table docs/analysis/refly_1m_table_2026-10-23.md; the five missing 1 m worlds built 23 Oct morning (Wesley), their 25 runs flying in batch 16 | R2483-R2583 (re-fly), R2417-R2422 (smoke); uncommitted |
| 4. Chain in the twin (N) | step 1 done (drop_gap_deg). The magnet-marker check (rev 3, merged as a switch) catches the zero-share T-to-pair release at 0.55 m and 1 m and mid figure-8; the stretch check misses it. The loaded square-to-T release still drops the ring (57-84 deg) whatever the cue; the soft detach does not (22.9/25.9 deg). Step 2a DROPPED 22 Oct (the rig T hangs 2.6 deg). Next: 2b (reaction); the rig T-to-pair on Wed 28 after the pair A/B | d440664, c881ef0, 367c98f; R2249-R2258, R2292-R2321 |
| 5. Rig Nr and K | twin: his basket mission works in hover at 0.55 m and 1 m (R2306, R2322); his 1 m floor join is clean (R2308); his 1 m circle rejoin stalls (R2310). 23 Oct: tejen-sync2 merged into the tree (uncommitted, gate green), his launch on our 1 m cables, configs/rig/attach_m1_1m.yaml, Thu 29 sheet docs/rig/thu_2026-10-29.md (his attach go/no-go j1-j3, Nr hover n1-n3 on our announced DETACH; carry_clearance 0.25, Wesley). Twin rehearsal (floor join x2, Nr x3 for the band) in lab batch 17. 28-29 Oct night (new 1.15 kg ring; card 2026-10-28_attach_ambitious; morning report 29 Oct): his attach welds first try with his settings (R2843/R2844); the low approach welds faster in the twin only (swing allowance 0.02 rad); the pickup's reattach now flies on his C++ planner (R2864/R2865, after an activation fix); a bridge timing race (commit_backdate, off) and his tracking check can switch his planner off mid-transit; reviewer: weak. Thu 29 sheet: sq1 / k1 | R2306, R2308, R2310, R2322; batch 17; R2843-R2865 |
| 6. Sim U (magnet off its slot) | grid scored (R2629-R2684, bars frozen 07:56): between, neither confirmed nor falsified. Along the rim the measured catch point's limit is <= 8 cm (bar 12: confirm fails), the slot arm's 2-4 cm; radially in both reach 16 cm (mechanism shown radially, NOT shown along the rim, Wesley). The slot arm folds in more gently along the rim. Owed: 2 re-flies (R2637, R2634 VOID) + the 20 cm radial extension x2 per arm, on today's tree (Wesley); stopped 14:03 because the lab was busy, relaunch when free | R2459-R2463, R2629-R2684; docs/experiments/2026-10-23_catch_offset.md |
| 7. Sim S (scalability) | rejoin_departed built, merged, critic answered and fixed: E1 x3, E2, E3, E4 PASS (R2578-R2596). S: 8 quads hover, chain 8 -> 4, and the full 8 -> 4 -> 8 with four rejoins fly in the twin (R2464, R2587, R2588, R2597; tilt <= 3.5 deg); the rig-calibrated bench gives ~4 quads in real time, so 8 quads are reported 'twin only, not real time' (Wesley 23 Oct). Claim cells next (bars accepted) 23 Oct 14:15: the 4-quad claim cells (C) CONFIRM at 1 iteration (R2700-R2711, C1-C7 in both repeats); the offline quality check Q6 at 8 quads fails 1 and 2 iterations, so the 8-quad claims fly at 5 (Wesley 14:37; configs s8_*_it5, queued for the lab); N4 opens after the last rejoin blend (Wesley). Rejoin E5 (leaning pair): 3 of 4 pass, R2712 missed by 0.28 deg (reported as is). 23 Oct 15:05: harder S cases PROPOSED for Wesley (the S card's "More complex cases": 7 ranked, the chain on a circle first; 5 need no code, their 9 it5 configs pass --check; a station task and the laps are designs for a critic; nothing flown). | R2464-R2597; uncommitted |
| 8. Sim C (paper's method) | the search's real best point flown (R2685-R2690, c_net_portfinal): on the 0.4 m/s line 99-105 mm vs our planner's 46 mm (the stage-1 point R2469: 156), ring yaw 14 deg on the line but 22-51 deg in hover; line 0.2: 42 mm, lag 1.25 s. Our planner's hover and line 0.2 cells (c_pl_*) wait for the lab. R2468/R2469 were the stage-1 point (my emit error, corrected) | R2446-R2448, R2468-R2469, R2685-R2690; C card Results |
| 9. Videos | overnight highlights rendered (R2597, R2464, R2594, R2578, R2446, R2427, R2433) + reel results/videos/reel_2026-10-23_overnight_highlights.mp4; every new config records | |
| 9b. Pair swing (rig 1.4 Hz) | round A: the throttle path carries the twin's swing; round B: the rig-timing twin stays calm; round C (tip hinge): null. Reduced round D CONFIRMS both rig candidates on the thr060 testbed: slope 0.07/0.06 deg, cable_source model 0.01/0.01 deg, against 7.39/7.36 (R2691-R2694); the testbed's mode is the twin's, so Wed 28's 1 m rig pair (slope A/B) is the test; cable_source model is a pair-only candidate after it. rate_kff / thr_tau_s / the rate filter / the tip world deleted. Betaflight read: radio + FC ~10-15 ms | R2449-R2482, R2598-R2601, R2691-R2694; docs/experiments/2026-10-23_pair_mode_timing.md |
| 10. Thesis | drafts 23 Oct (unreviewed, uncommitted): R, S, rejoin, C, pair swing in ~/mdc_wt/thesis .../src/drafts_1023/ (index DRAFTS_2026-10-23.md); system_overview latency and reconfiguration drop rule corrected to the measurements |  |

## Context

Supervisor (21 Oct), as relayed by Wesley:
- **Baseline, 9 configs:** hover, figure-8 and a new "spinny line", each with 2, 3 and 4 quads, at 0.2 m/s.
- **Novelty, 3 repeats each:** the same three trajectories with 4->3->2->3->4 (or 4->3->4). The releases are at 90 deg,
  with as little ring tilt as possible.
- **Rig:** 4->3->4 where the single quad puts the object in the basket in between.
- **Sim:**
  - how far off its slot the magnet can catch before the task fails;
  - scalability: find the limit, then fly N -> 2 -> N around a bigger payload;
  - the paper's dissipative velocity method against our MPC on static, linear and linear-fast.
- **Side notes:** pause the trajectory for every rig attach. Every link becomes a ~1 m slack cable, at the length
  measured on the day.

Wesley's answers (21 Oct evening):

| Topic | Decision |
|---|---|
| Links | Slack cables on every quad, the same kind as the 0.55 m ones |
| Baseline layouts | 2 quads 0/6, 3 quads 0/4/8 (even), 4 quads 0/3/6/9 |
| Rig baseline | One flight per config |
| Spinny line | 1 m out and back; **from 22 Oct the ring turns once (360 deg) at a constant yaw rate (~24 deg/s at 0.2 m/s, 4 s rate ramps), through the turnaround** (was 1 rad per 0.5 m, ~229 deg). At the end the planner lands the ring by itself, as for the figure-8. |
| Chain timing | One lap per stage: 5 laps at 4, 3, 2, 3 and 4 quads. Releases come unannounced at a fixed point mid-lap, while moving; re-attaches pause the path between laps. |
| **Releases** | **Unannounced only: no load shifted before a release.** Accept the tilt and minimise it after detection. |
| Drop rule | A loss that leaves a T or an opposite pair keeps flying (switch, card, critic). A non-opposite pair, one quad or tilt > 30 deg still drops the ring. |
| Rig 4->3->4 | Tejen's controller re-attaches QUAD4. Start in the M1 layout (carriers 3/7/11, QUAD4 on 9: its leaving leaves an even triple), then the square 0/3/6/9 once the twin fix works. Widen to more re-attaches as they work. |
| Basket runs | The baseline three trajectories |
| Offset sweep | The magnet catching off its slot. Two arms: the measured catch point (as built) and the slot assumed. |
| Scalability | Reach 9 quads (fewer solver iterations) if solve quality doesn't drop heavily. The payload is bigger, not much heavier. If time runs short, the 9-quad Gazebo run gives way first. |
| Comparison | Twin only. 4 quads 0/3/6/9; hover and a line at 0.2 and 0.4 m/s; 3 repeats. The paper's method is retuned fairly, time-boxed to ~2 days. |
| Old tables | **Re-fly every 0.55 m twin table at 1 m** |
| Videos | Every twin run gets one fixed wide-view mp4 (720p) at real speed, rendered after the batch, plus a reel per evening (title card per run). State logs are deleted once their video is checked. Space comes from freeing the laptop disk (95% full) after your approval of a delete list. |
| Figure-8 and hold (22 Oct) | The figure-8 is size 1.0 (x +-1.0, y +-0.5 m; 7 Oct's), and every path flies a 5 s hover hold first (the integral removes the rig's -x push) |
| Velocity filter (22 Oct) | The twin copies the rig's mocap velocity filter; bands for both filters; Wed opens with the pair A/B |
| Tomorrow | If time runs short: the offset check and the 3-quad baselines first |
| Lab | Every Wed/Thu 8-11 am (noise); a couple of Fridays if needed. Results freeze Sat 7 Nov; submission 20 Nov. |

## What the thesis gets

| Id | Result | Where | Layout / cases | Runs |
|---|---|---|---|---|
| B | Baseline: hover, figure-8, line_spin at 0.2 m/s | rig (1 m) and twin | 2 quads 0/6, 3 quads 0/4/8, 4 quads 0/3/6/9 | rig 9; twin 18 |
| N | Chain 4->3->2->3->4, unannounced releases | twin | 0/3/6/9 -> T -> pair 0/6 -> back | 9 (3 trajectories x 3) |
| Nr | 4->3->4, QUAD4 leaves and Tejen re-attaches it (path paused) | rig | M1 layout, then the square | 9 (3 x 3) |
| K | 4->3->4 with QUAD4's pickup and basket drop | rig | as Nr | 9 (3 x 3) |
| U | Magnet caught off its slot, swept until failure | twin | 3->4 attach, then the figure-8 | ~28 per arm |
| S | Solve time against N (bench), then N_max -> 2 -> N_max | bench, twin (lab PC) | bigger ring | bench + 2 |
| C | Paper's method against our planner | twin | 4 quads; hover, line 0.2 and 0.4 | 18 |
| R | Every existing 0.55 m twin table re-flown at 1 m | twin (lab PC) | as each table | from the inventory |

## What the exploration found (and the design pass corrected)

1. **The 90 deg T. The real problem is the release transient; the settled tilt comes from a model gap.**
   - Old Gazebo T runs peaked at 25.6-28.5 deg at the release, then settled at 6.7-6.8 deg (R0561/R0562). Today's twin
     drops the ring at detection (gap rule, R1919) or faults at 60 deg with the drop off (R1901).
   - After a release from 0/3/6/9, the ring's centre of mass lies on the line between the remaining opposite pair
     (0/6). The quad opposite the gap (plate 3) then carries no share of the ring. **It still has to hold up its own
     tether** (~0.7 N in the twin: magnet plus part of the rod).
   - The planner's default massless-rod model gives it 0 N, floored to 0.1 N. The tether's weight then pulls plate 3
     down: that's the ~6.7 deg.
   - With the tether-mass model (exists, off by default) and a balance solved for force vectors, the T split is
     5.73 / 5.73 / 0.70 N, level, with quads 0 and 6 leaning ~1 deg.
   - The transient: until the re-plan lands, the plate-3 quad keeps its old 4-quad share and lifts its side. The fix
     is to react faster and closer to the new split; the measured-pull feed-forward already helped a lot (Tier A,
     R1706-R1711).
   - **The pair:** its roll has no actuation except a weak sideways lever through the attach height. Gravity levels
     it (h ~2 cm gives ~0.17 N m/rad, rocking ~0.4-0.5 Hz). The planner only models this if `attach_z` is the real
     height above the ring's centre of mass: 0.02 in the twin (right), 0.0 on the rig (wrong; the rig's mocap origin
     sits ~3 cm above the centre of mass). That gap can explain the 11-15 deg peak-to-peak rocking of 21 Oct.
   - **Rig and twin differ:** the twin's rods are rigid, can push, and have no collision geometry, so twin T, rider
     and rejoin results will look better than the rig's.
2. **No rig attach has completed yet** (15 Oct). Every Nr and K run waits on his attach working on 1 m cables.
3. **Compute:**
   - 4 quads take 53-56 ms median (73-79 ms p95) on the rig at 5 SQP iterations, against the 60 ms cap.
   - TU Delft's 9 quads at 10 Hz (Sun et al. 2025, Fig. 7B) used 1 iteration.
   - Ours at 3 quads takes 7.4 ms at 1 iteration and 20.6 ms at 5.
4. **Already built:**
   - stop, attach, continue (`attach_moving:=false`), though a frozen clock lets the ring coast 0.13 m (R0669);
   - the measured catch point (`_attach_ocp`);
   - per-cable measured lengths;
   - 1 m worlds and bands for three carriers;
   - unannounced-release detection;
   - the tether-mass model (`rod_mass` 0.075, `rod_com` 0.147);
   - solver sizes 2-4 for a 4-quad run;
   - video recording (`record:`, `tools/render_video.py`).
5. **Missing:**
   - 2-quad and T worlds;
   - re-attaching a quad that left;
   - anything above 4 quads;
   - the paper's controller in the rig twin;
   - `line_spin`;
   - a smooth path pause;
   - yaw in ring_bridge;
   - partner_contact beyond hover/orbit;
   - the evening reel.

## Lab days

| Day (8-11 am) | Cables | Goal (what has passed its gates by then flies; the rest waits) |
|---|---|---|
| Thu 22 Oct | 0.55 m, 3 quads | offset check; 3-quad then 2-quad baselines; T -> pair if its fix passed tonight |
| Wed 28 Oct | 1 m | in order (overfull is fine): props-off bench; the first 1 m lift low; the pair A/B (old filter / slope, interleaved, + P4); Part C (T to pair, RViz release); the 3-quad cells once per filter; the 2-quad and 4-quad cells (4 quads approved) |
| Thu 29 Oct | 1 m | his QUAD4 attach on a hovering ring, M1 layout (go/no-go); Nr hover x 3 |
| Fri 30 Oct (if added) | 1 m | Nr figure-8 and line_spin x 3 |
| Wed 4 Nov | 1 m | K: the basket on the three trajectories x 3 |
| Thu 5 Nov | 1 m | re-flies; Nr from the square if the twin fix passed; 4->3->2 (our releases) |
| Fri 6 Nov (if added) | 1 m | spare |
| Sat 7 Nov | - | results freeze |

## Order of work

### 1. Tonight, for Thu 22 (laptop twin, 0.55 m, one run at a time, idle 3 min per run)
1. **Disk:** a list of old run folders to delete or move (state logs, plots, voided runs) for your approval, until
   ~15 GB is free. Nothing is deleted without your word.
2. **`line_spin`** (code design A): code, tests, `gate.sh --quick`, prebuild `--drone-mass 0.55`.
3. **Worlds:** 2 quads on 0/6 and the T on 0/3/6, as `make_carry_worlds.py` entries checked by gate step 4.
4. **Baseline twin runs** (tomorrow's first priority), recorded, exploratory rows with falsifiers:
   - line_spin with 3 quads (2 runs);
   - with 2 quads: hover (1), figure-8 (2), line_spin (2).
5. **The T on today's code** (your "see what the tilt is"): 0/3/6/9 hover, RELEASE 9 (unannounced), with the gap drop
   off (2 runs). It may fault at 60 deg; it is the baseline the fix is judged against.
6. **The chain fix** (code design C):
   - the card `docs/experiments/2026-10-21_chain_release.md`, then the critic;
   - code behind switches: the drop rule, the tether-mass split, and the measured `attach_z`;
   - twin: the RELEASE 9 run again (2), and the T 0/3/6 from the floor with an unannounced RELEASE of the plate-3
     quad down to the pair (2).
7. **After the batch:** render the videos and the reel (`tools/make_reel.py`, new), then check them.
8. **Sheet `docs/rig/thu_2026-10-22.md`**, plus `configs/rig/chain_pair.yaml`.
   - The T -> pair step goes on it only if step 6 passed its bars.
   - Stop rule: pair rocking over 15 deg peak-to-peak -> LAND.

### 2. Thu 22 rig (QUAD2/3/4, 0.55 m)
Props off:
- QUAD2's cable, magnet seating and Motive pivot;
- a spirit level on the floor;
- each cable's hanging length;
- the ring's free-rocking period on two opposite plates (gives h for `attach_z`).

Then:
1. 3 quads 0/4/8, hover, no hold. Expect the offset at 0.5-2 cm; over 2 cm means the pivot explanation is wrong.
2. Hover with the 10 s hold: the lean should be under ~1-2 deg. **You choose hold or no hold for every baseline.**
3. 3 quads: figure-8, then line_spin, at 0.2.
4. 2 quads 0/6: hover, figure-8, line_spin. Use the two best magnets.
5. The T 0/3/6 -> you turn the plate-3 quad's magnet OFF from the console (unannounced) -> pair hover -> LAND. Only if
   tonight's step 6 passed.

Every flight gets a registry row and a log read. The new cells have no band yet, so they are exploratory.

### 3. The 1 m switch and the re-fly
- **Defaults:**
  - `cable_len` 1.0 (`src/bringup/config/sim.yaml:9`, `real.yaml:8`); the rig types each quad's measured length;
  - the L100 worlds as the twin's set, with new ones for 2 quads, the T, 4 quads (0/3/6/9) and the M1 layout;
  - the partner launch's 'rig' GEOMETRY for QUAD4's 1 m tether (1.033 m body to marker today, ~0.99 m from the pivot).
- **Short-cable constants:**
  - `approach.py:41`;
  - `DEPARTED_CLEAR_R` / `DEPARTED_PATH_CLEAR_M` (`dissipative_node.py:54-58`);
  - QUAD4's guards (`graph_attach.py:99-131`);
  - first, settle `attach_cable_len` (the code reads rod-from-pivot, the configs type body-to-tip);
  - tools: `n2_bands`, `fake_mocap`, `latency_bench`, `planner_replay`, `slip_replay`, `thesis_sim_tables`.
- **Rig ring:** add a centre-of-mass offset where the ring pose is read (or move the Motive pivot in z). Type
  `attach_z` from the measured h.
- **Takeoff:** with 1 m slack cables the creep climbs ~0.45 m further, so check the creep timeouts. The first rig
  flight goes low.
- **Twin baseline at 1 m:** 9 configs x 2 runs, then the bands (the signed N2 rule) frozen for every rig cell before
  Wed 28.
- **Re-fly (R):**
  - list every 0.55 m twin table the thesis uses (`docs/analysis/sim_results_inventory_2026-10-16.md`, the thesis
    `CHANGES.md`), and write each one's 1 m configs;
  - fly them all on the frozen tree, lab PC 4 slots in the evenings plus the laptop on non-rig days;
  - one reviewer per table.
- **Tree freeze:** once the chain fix and line_spin have passed their gates. Every claim table and R flies on it.

### 4. The chain in the twin (code design B and C)
1. **Card:** `2026-10-21_chain_release.md`.
   - Releases are unannounced (RELEASE events).
   - One lap per stage (4, 3, 2, 3, 4 quads), so every fleet size flies an identical lap.
   - Each release fires at a fixed mid-lap point; each re-attach pauses the path smoothly between laps, like for like
     with the rig.
2. **Code:** the drop rule, the tether-mass split and `attach_z` (C), plus `rejoin_departed` (B).
3. **Runs:**
   - hover first;
   - the neighbours once: the leaver at each of the four 90 deg positions, a trajectory, a floor start;
   - then the figure-8 and line_spin;
   - the table: 3 repeats per trajectory at 1 m on the frozen tree.
4. **Per stage:** tilt peak and settled, the transient at each release and attach, the opposite quad's tension, path
   error.
5. **If the pair rocks on the figure-8:** a speed scale for the 2-quad stage (card first). Only if the numbers call
   for it.

### 5. Rig Nr and K (Tejen's controller re-attaches QUAD4)
Prerequisites:
- the sync2 worktree committed and merged;
- his launch on the 1 m tether;
- the smooth pause (A) with `contact_traj_pause`, allowing the figure-8 and line_spin under partner_contact (card and
  critic, since it is a gate);
- ring_bridge predicting yaw.

The same laps apply on the figure-8 and line_spin:
1. a lap with 4 quads;
2. QUAD4 leaves (his release, unannounced to our planner);
3. laps with 3 quads while he flies (pickup and drop in K);
4. the path pauses for his attach;
5. a final lap with 4.

Order:
1. M1 layout first (QUAD4 leaving leaves an even triple, so the ring stays level for his pickup, drop and attach).
2. The square once the twin fix has passed.
3. 4->3->2 (our releases) when a Thursday has room.
4. More re-attaches as his attach becomes reliable.

### 6. Sim U: the magnet caught off its slot
- **How the twin catches:** the weld locks wherever the tip is when it fires (a DetachableJoint at the current pose).
- **The knob:** a twin-only one that aims the newcomer d off its slot, tangentially and radially, with `weld_radius`
  >= d.
- **Arms:**
  - the measured catch point (as built);
  - the slot assumed.
- **Sweep:** d = 0, 2, 4, 6, 8, 12, 16 cm until it fails (no fold-in, a fault or a drop). 2 repeats. The attach
  happens in hover, then the figure-8 at 0.2.
- **Metrics:** completion, fold-in tilt peak, the steady offset after, path error after.
- **Figure:** error against offset, one line per arm.

### 7. Sim S: scalability
1. **Bench, wall clock:**
   - `tools/latency_bench.py --n --radius --load`, plus inertia flags;
   - n = 2..10 on the rig laptop and the lab PC, at 5, 2 and 1 SQP iterations (shorter horizons if 9 still misses);
   - the limit: the largest n with p95 tick (solve plus reference extraction) under the 60 ms cap.
2. **Quality** at the reduced setting against 5 iterations:
   - plan cost, tension and constraint violations on logged replays (`tools/planner_replay.py`);
   - a twin run at 4 quads;
   - the "not heavily worse" bar is written in the card first.
3. **Payload:**
   - spacing 2 pi (R + 0.5) / N >= 0.7 m with 1 m cables, so R ~0.5 m for 9;
   - mass ~0.9-1.0 kg (two quads must lift it: throttle ~0.75 against the 0.80 cap; checked against the 21 Oct 2-quad
     logs);
   - inertia override, so every N rebuilds.
4. **Layout:** the chain must end on an opposite pair, so N is even (8 or 10), or 9 on a near-even layout that includes
   one opposite pair.
5. **Run:** unannounced releases one at a time, keeping the survivors' largest gap smallest (for 6: 5, 2, 4, 1), down
   to the pair, then re-attach in reverse. Hover, lab PC, 2 runs. **This gives way first if time runs short.**

### 8. Sim C: the paper's method against our planner
- **Wiring:**
  - the network into the rig twin (`sim_network` through `map_and_geometry`, `auto_network_handover` in the sim
    detach graph, a twin `network` mode);
  - retuned for 1 m cables at 60 deg (`diss_elev_deg` 45 -> 60), offline with `verify_dissipative` first, time-boxed
    to ~2 days;
  - the paper's velocity loop as its tracker.
- **Arms:** 4 quads 0/3/6/9 with the same lift, scored from the hand-over: hover 60 s, `line_x` at 0.2 and 0.4 m/s, 3
  repeats each.
- **Metrics:** RMSE and max, lag (s), tilt mean and peak, thrust peak, compute per tick.
- It replaces Appendix F. The network code stays.

### 9. Videos (every twin run)
- **During the run:**
  - `record:` in every new config, with one fixed `wide` camera at 1280x720;
  - `period_s` 1/30 (a third of today's state-log size);
  - `render: false`, so nothing renders while runs are live (laptop or lab PC).
- **Cameras:** presets per scene in `tools/experiment/record.py`: baseline paths (2/3/4 quads, figure-8 +-0.5 m, line
  1 m), the chain (with the departed quads' staging points), and the 9-quad ring. `--view` re-aims after the fact
  without a re-fly.
- **After each batch:**
  - `tools/render_video.py` per run;
  - a new `tools/make_reel.py` joins the evening's mp4s with a 2 s title card per run (run id, config, pass or fail
    from the registry row) into `results/videos/reel_<date>.mp4`;
  - each video's frame count is checked (`render.json`), and the state log is deleted after the check;
  - lab PC runs render there and the mp4s are copied back.

### 10. Thesis
- **Cards** for B, N, K, U, S, C and R, each with its falsifier and bars frozen before its runs.
- **One reviewer per table.**
- **Figures** through the thesis-figure skill.
- **Mapping:** B goes to RQ1 (with the 21 Oct geometry finding); N, Nr and K go to RQ2/RQ3; U, S and C get their own
  sections.
- `docs/thesis_plan_2026-10-11.md` is updated to match.

## Needs your word (approving the plan gives it; flight code stays behind switches)
1. New twin worlds (2 quads, the T, 4 quads and the M1 layout at 1 m, the 9-quad ring) and drone models beyond 4.
2. The 1 m defaults, the short-cable constants, and the rig ring's centre-of-mass offset and `attach_z`.
3. The drop-rule change and the tether-mass split for the chain (switches; card and critic before any rig use).
4. Partner_contact on the figure-8 and line_spin with the smooth pause (card and critic).
5. A twin `network` mode for the comparison.
6. The delete list for disk space (each folder shown first).
7. Commits: today's rig files, the sync2 worktree, and this work on `day-1021`.

## Extra tests worth adding (my suggestions)
1. **Twin against rig for the 9 baselines.** Free, and it makes the sim-only sections believable.
2. **An unannounced loss on each trajectory with both controllers** in table C. Resilience is the paper's own claim.
3. **Attach with the path paused against moving** (twin, 3 each). It backs your rig rule with numbers.
4. **A cable-length error sweep beside U.** On 21 Oct a 2 cm geometry error became ~6 cm.
5. **For the paper with Tejen:** the basket mission's success rate over every attempt, plus one continuous video.

## Housekeeping on approval
- This plan becomes `docs/plan_thesis_results_2026-10-21.md` with a status table, and CLAUDE.md's "current plan" line
  points to it.
- **CLAUDE.md locked lines that change:** the drop rule (after its critic), and the partner_contact hover-only line
  (after its critic and reviewer).
- `docs/decisions.md` gets the answers above as DECIDED 2026-10-21 lines, superseding 16 Oct decision 5.
- `docs/SESSION_STATE.md` gets updated.
- The new branch `day-1021` starts after today's files are committed with your word.

## Verification
- pytest per new test, `tools/gate.sh --quick` before every twin batch and commit, and prebuild
  `--drone-mass 0.55`.
- **Twin:**
  - one registry row per run (LF, `cmp`);
  - exploratory rows carry falsifiers until a card exists;
  - every run's video is in the evening reel;
  - `/report` per matrix, and a reviewer per table.
- **Rig:** the sheet's stop rules, a log read for every flight, and RIG-* rows.

## Code design (from the design pass)

### A. `line_spin` and the smooth pause
**`src/mpc_planner/mpc_planner/load_trajectory.py`:**
- Two smootherstep legs of 1.875 d/v each (18.75 s at 0.2 m/s; the file's rule of zero speed and acceleration at the
  start, the turnaround and the end).
- Yaw = arc length / 0.5 m, counter-clockwise. Widen the `yaw_at` gate (:221) to `('spin', 'line_spin')`.
- After completion, `yaw_at` returns (2d/0.5, 0). It must not snap back to 0 as spin does.

**Yaw-aware fixes:**
- `hold_yref` (`reference_builder.py:274`) and `_prime_solver` (`planner_node.py:1576`) use `psi0 + yaw_at(traj_t)`;
- `_attach_ocp`'s `rot_z(-psi0)` (`dissipative_node.py:2691`) likewise;
- `_path_clear_point` (:1345) learns `fig_8` and the line kinds.

**`PausableTrajectory`** (in `load_trajectory.py`, installed at `planner_node.py:364`):
- the rate ramps smoothly 1 -> 0 -> 1 (smootherstep), with a closed-form warped clock;
- velocity sigma v, acceleration sigma^2 a + sigma-dot v, yaw rate sigma omega;
- `_publish_traj_state` (:841-863) publishes the warped clock and a hold flag;
- pause on his first rejoin phase (`_his_phase_cb`, `dissipative_node.py:1806`, behind `contact_traj_pause`), resume
  when the post-weld hold ends or a take-back clears;
- a take-back on a yawing path pauses first.

**Partner bridge and launch:**
- `ring_bridge.py:121-135` predicts yaw (measured + yaw_at(t) - yaw_at(now));
- `ring_bridge.py:168-170` treats a hold as "not moving";
- `graph_attach.py:76-89` allows `fig_8` / `line_spin` under partner_contact only with `contact_traj_pause:=true`
  (typed-only on the rig, like `contact_rig_orbit`).

**Call sites:** `tb5_replay.py:109`, `lag_metrics.py:83/93`, `latency_bench.py:217/324`, `thesis_sim_tables.py:361`,
`rig_summary.py:47/100/307` (needed tomorrow), `n2_bands.py:30`.

**Tests:**
- `test_line_spin_trajectory.py` and `test_pausable_trajectory.py`;
- add line_spin to `test_rate_ff.py`;
- a yaw case for `hold_yref` in `test_resize_yaw_datum.py`;
- a bridge yaw test;
- pause cases in `test_partner_contact.py`.

### B. `rejoin_departed` (twin only)
Not by re-arming a departed quad as a reserved newcomer (that state is built for one partner newcomer).

Changes:
- bridge `/drone_i/attach` in `graph_detach.py:149-152`;
- subscribe to `/fleet/attach` (`dissipative_node.py:711`);
- remember the leaver's attach point and measured length in `_detach_plan` / `_resize_and_release` (:1256-1294);
- departed quads hold at a ring-frame staging point (~1.0 m out, their own bearing) instead of landing;
- `/fleet/attach` starts an `ApproachProfile` rejoin: a new `_rejoin_tick`, welding at <= 1.5 cm and slow for 0.3 s,
  publishing `/drone_d/attach`, then `_attach_ocp(d)` with the remembered point (not `_capture_weld_rho`, which would
  weld a carrier at its body);
- `_drop_departed` is always called, and the loss detector is reset for d;
- fix the latent bug: with the switch off, `/fleet/attach` of a departed carrier folds it in at its body.

Laps:
- a `traj_laps` parameter: fig_8 and line_spin repeat k laps before completing, and the planner then lands the ring
  as usual;
- the runner gets a wait on the published trajectory clock (e.g. `WAIT_TRAJ lap fraction`). Releases then fire at the
  same point of their lap, and re-attaches start at a lap's end with the path paused.

Runner (as `app_w4_3915_detach.yaml`):
- azimuths `0,270,180,90`, `traj_laps: 5`, `min_survivors: 2`, `rejoin_departed: true`, `departed_land: false`,
  `rod_mass: 0.075`;
- events: WAIT_TRAJ 0.5 -> RELEASE 1 -> WAIT_TRAJ 1.5 -> RELEASE 3 -> WAIT_TRAJ 3.0 -> ATTACH 3 (path paused) ->
  WAIT_TRAJ 4.0 -> ATTACH 1 (paused) -> the planner lands at the end of lap 5.

Test: `test_rejoin_departed.py`, in the `test_soft_detach.py` stub style.

### C. Unannounced release into a T or a pair
- **Drop rule:** `_drop_due` (`dissipative_node.py:1357-1369`) and `_largest_gap_deg` (:190) become pair-aware. A T or
  an opposite pair keeps flying; min_survivors 2 by params; behind a switch.
- **Split:**
  - the tether-mass model on (`rod_mass` 0.075, `rod_com` 0.147; it changes the compiled OCP, so it gets its own
    cache and prebuild);
  - the reference balance solved for force vectors (`geometry.balanced_tensions`, `geometry.py:165-194`; also
    `dissipative_node.py:2656`, which drops the rod terms);
  - a per-quad floor settable at run time (`solver.set(k, 'lbx', ...)`, `planner_solver.py:292`; reset on every
    resize).
- **Transient:** measured tonight on today's code. Candidates:
  - the survivors' trackers take the new split as soon as the release is detected, before the re-plan lands;
  - a tighter detection.
  The card picks one after the measurement.
- **`attach_z`:** twin 0.02 (already right). Rig: the ring pose moved to its centre of mass, and h from the free
  rocking period (h = I w^2 / (m g), I ~0.0226 kg m^2).
- **Falsifier:** with the tether-mass split on, the twin T still settles over ~2 deg, or the release peak isn't lower
  than tonight's baseline.

### D. Above 4 quads
- drone models for i >= 4, derived from drone 0 (model and IMU names, IMU topic, the four `robotNamespace` tags);
- `make_carry_worlds.py:108` loops over N;
- a `--plates` flag (`generate_rigid_world.py:40/252/316`);
- per-world expectations in `test_rig_worlds.py:49-61`;
- prebuild sizes `max(2, min(n - reserved - 2, min_survivors))` (`dissipative_node.py:654`);
- an inertia override in the node and in `tools/prebuild_planner.py` (the cache signature includes it);
- `latency_bench.py` flags (:62, :75, :197, :324, :350);
- the slot matcher past 6 quads uses its cyclic shifts (fine).

### Risks
- The twin flatters the T, the rider and the rejoin (rigid rods, no collisions); the rig is the real test.
- The chain's laps change the per-stage timing whenever an attach takes longer than expected (the path waits). The
  table therefore compares laps, not clock times.
- Release transients must stay under the trackers' 30 deg tilt drop.
- The re-fly (R) is large: it needs the tree frozen early, and it competes with U, C and S for the lab PC. The 9-quad
  run gives way first.
- Tonight is long. What isn't through its gates by morning waits for Wed 28.
