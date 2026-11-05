# Simulation runs still to do (the checklist; started 30 Oct 2026)

Every simulation run the thesis still needs, so the lab PC can take the next block whenever it is free. Tick a line the
moment its runs are scored, with the run ids. Order: top to bottom within a priority. The approved rescope
(`docs/plan_rescope_2026-10-30.md`) and `docs/decisions.md` (30 Oct) are the source.

**Status:** `[ ]` to do, `[~]` running, `[x]` done (run ids), `[-]` dropped (why).
**Rules:**
- Lab PC only, through `tools/remote/lab_batch.py`.
- A card before any claim run. Simulation results may be judged after the run (Wesley 2 Nov 23:14); a card's frozen bars stay as written.
- 2 repeats.
- The 0.86 kg ring for claims, plus one 1.15 kg cell per table.
- Claim tables fly on one code tree, the final one (freeze Fri 6 Nov), with the slope velocity estimate.
- The plan is massless (Wesley 4 Nov ~23:00); a cell that forms a T flies int_k_xy 0 and says so (fallback of 31 Oct 17:16).
- **The final-tree batch (Sat 7 - Sun 8 Nov, 245 runs + 10 on his word, ~22 h): `docs/claim_batch_2026-11-07.md`**
  (ordered chunk lists `configs/batches/claim1107_*.txt`, the commands, the scorers, the open questions).
- A registry row per run.

## P1: the headline results and the claims

- [~] **X1 release ablation** (card `docs/experiments/2026-10-30_x1_release_ablation.md`, configs `configs/experiments/x1_*`).
  - [x] First pass 30 Oct: 63 runs R2915-R2978 (R2952 void, re-flown R2964), scored `results/analysis/x1_2026-10-30.json`.
        The measured pull holds the ring through a release (late or no resize fly the same: B1, B2 falsified); the
        planned pull is 2.4-4.4x worse (B3); in the T our planner settles 7-13 deg off level against 2.2 without the
        resize (U5 missed): the T fix comes first (Wesley 30 Oct). Claims re-fly on the final tree.
  - Arms: ours, no resize, resize 0.5 / 1 / 2 s late. Planned pull and no resize + planned pull are out (Wesley 2 Nov 23:03:
    no planned pull anywhere), so B3, U3 and U4 are not scored.
  - Cells: 4 quads hover and figure-8, 8 quads hover, the square to a T (sqT), plus the 1.15 kg cell (h4p115, ours and no resize).
  - [x] Re-fly on the current tree (3 Nov, not the final tree): R3520-R3563, 44 runs, scored `results/analysis/x1_2026-11-04.json`.
  - The T: the rod in the plan was flipped on 4 Nov ~22:20 and reversed ~23:00 (the plan is massless everywhere). The sqT cell
    flies int_k_xy 0 instead (Wesley's fallback of 31 Oct 17:16) as `x1_sqTnoxy_<arm>`. On the new links the T leans
    16.5-17.8 deg with the integral on and 1.77 deg with it off (R3699, R3700, R3701).
  - [ ] **Claim re-fly on the final tree:** 44 runs, chunk 4 of `docs/claim_batch_2026-11-07.md` (h8, h4, f4, sqTnoxy x 5 arms, h4p115 x 2, each x2).
  - The paper's hand-over arm moved to X8 (no loss handling for it in the code yet).
  - [x] **Uneven cell (Wesley 30 Oct), flown 30 Oct R2965-R2978:** the square 0/3/6/9 to a T, one unannounced release in hover, the same seven arms x2
        (14 runs), `drop_tilt_deg: 80` in simulation only (the square-to-T release peaks 57-61 deg on 1 m cables; at 30 every
        arm drops). Compares recovery: peak tilt, dip, time back to level. Queued with the X1 agent after its batch.
- [~] **X4 endurance relay** (card `docs/experiments/2026-10-30_x4_endurance_relay.md`, sections 9, 10; setting `x4_*_v3`, adopted by Wesley 2 Nov).
  - Pack budget B = 10 min. 26 leaves (Wesley 4 Nov ~07:45: the 1700 s miss stays written, not a reason to re-fly).
  - [x] `x4_relay_short_v3`, exploratory: R3454, 3 of 3.
  - [x] First passes R3455-R3457 and the claim runs on the current tree R3519, R3564-R3567 (card section 10). They flew
        `plan_rod_level` true, against Wesley's 4 Nov ~23:00 decision (the plan is massless), so none is the claim.
  - [ ] **Claim re-fly on the final tree with the rod out of the plan** (`x4_*_v3_norod`; int_k_xy 0 stays typed in
        `x4_relay_nowalk.yaml`): relay_30min x2, fixed_B x2, relay_30min_m115 x1 = 5 runs, ~5 h, chunk 1 of
        `docs/claim_batch_2026-11-07.md` (2 slots x 12 cpus, wall cap 200 min).
- [ ] **X2 the supervisor's chain** 4 -> 3 -> 2 -> 3 -> 4, a lap per stage, hover / figure-8 / spinny line. 11 runs.
  - [ ] **Claim re-fly on the final tree:** `x2_full_fig8_{soft,unann,base}` x2 = 6 runs, chunk 5 of `docs/claim_batch_2026-11-07.md`
        (no config change: `x2_chain_tip80.yaml` already flies slope and int_k_xy 0 for the whole run). Its 1.15 kg cell (A on
        `four_rigid_ground_rig_L100_p115.sdf` at throttle_max 0.9, x1) was never written: Wesley's call. The hover and spinny-line
        cells of the 11-run plan were never written either.
  - [x] **The full chain on the figure-eight in one run (Wesley 30 Oct):** R3586-R3591 (current tree; feed F042: A and B hold) square 4 -> 3 -> 2 -> 3 -> 4, a lap per stage,
        two variants differing only at the square-to-T step: (A) that step announced (soft detach, 25-29 deg seen in
        R2840/R2842), (B) that step unannounced with `drop_tilt_deg: 80` (simulation only). Every other step
        unannounced. x2 each. Configs from c4_softdetach9_fig8_L100 / c4_detach9_fig8_L100 and the chain laps.
  - The square-to-T step stays unannounced: the drop rule puts the ring down there, reported as the rule's boundary.
  - The rest (T -> pair -> T -> square) flies from a fresh start in the T layout (decided 30 Oct).
  - Configs to write (the laps exist since 23 Oct).
  - [x] The full chain's card 30 Oct (`docs/experiments/2026-10-30_x2_full_chain_fig8.md`), bars FROZEN ~15:46 (Wesley);
        configs `x2_full_fig8_{soft,unann,base}` + `configs/twin/x2_chain_tip80.yaml` (pass --check). 6 runs ~1 h.
- [~] **X3 the 8-quad set as claims:** R2768-R2779's nine cases re-flown as claims, plus the never-flown 8 -> 2 -> 8
  (`s8_chain2back`).
  - [x] Current tree, 4 Nov: R3648-R3673 (26 runs; feed F042; chain2back fails in both). They flew the OLD `ema` filter (the
        s8 configs type it) while their N1/N4 comparators R3584/R3585 flew slope, so they are not the claim.
  - [ ] **Claim re-fly on the final tree, on slope** (the `x3_` copies), chunk 2 of `docs/claim_batch_2026-11-07.md`, 24 runs:
        orbit, adjpair, chain4b2b, chain4b2b_orbit, chain4back_fig8laps, chain4back_pairs, chain4back_relblend,
        chain4task, fig8rel2, relay (`x3_s8_<case>_it5[_tip]`); chain2back as `x3_s8_chain2back_it5_tip_noxy` (its T 0/4/6:
        int_k_xy 0, the fallback of 31 Oct 17:16); P's base `x3_s8_fig8laps_it5` (it never flew on one tree with its chain);
        each x2. The hover and figure-8 baselines (`x3_s8_hover_it5`, `x3_s8_fig8_it5`) fly in X3b's chunk.
  - Scorer: `tools/s_metrics.py` per run. The per-case wrapper of 4 Nov (L/J/N4/P per case, adjpair's same-time releases)
    was in a session scratchpad and is gone: it must be rewritten before scoring.
- [~] **X3b eight and nine carriers in real time** (first pass 1 Nov: C8 confirmed at 8 on the runs flown; R8a holds, 51.0 ms p95 idle at 8, 64.0 at 9; R8b falsified on this laptop, R3451, 260.9 ms p95 with Gazebo on the same CPU; R3453 the first complete 9 -> 5 -> 9 with the marker check; claim runs owed on the final tree) (Wesley 30 Oct: "show that running the controller at sim time for 8 quads
  still gives the same results ... the reference paper said this method could go up to 9 ... the solve time figure to
  show that 8 is possible"). One SQP iteration per tick (`plan_sqp_iters: 1`), the residual only where the publish gate
  needs it (`plan_res_every: 0`) and `plan_fast_refs` on; existing switches, no new code. Ground computer, idle bench
  24 Oct (results/analysis/s_bench_B_2026-10-24.json, host tuf): tick at 1 iteration 82.5 ms with 8 (max 90), 106 ms
  with 9, of which the residual 33 / 43 ms.
  - [x] Card 30 Oct (`docs/experiments/2026-10-30_x3b_realtime_8_9.md`), bars FROZEN ~15:46 (Wesley; lab runs res_every 1):
        C8 (it1 + res 0 + fast refs = the same verdicts as it5 on N1, L, J, N4, P) and R8 (closed-loop tick p95 <= 100 ms,
        <= 1 % over 150 ms). Configs `x3b_s8_{hover,fig8,chain4back}_rt`, `x3b_s9_{hover,chain4back}_rt` and it5
        comparators `x3_s8_*_it5`, `x3_s4_hover_it5`; tuf timing runs `x3b_s{8,9}_chain4back_rt0` (res 0). All pass --check.
        18 lab runs ~2.1 h.
  - [x] Lab PC, current tree (3 Nov): R3568-R3585, the card's 18 runs on slope (the chains with the marker check, `_tip`).
        Not the final tree.
  - [-] The one-iteration extras R3686-R3693 (4 Nov, `s8_*_it1` / `s9_hover_R53_L100_it1`, ema, not scored): not re-flown.
        They are the card's `x3b_*_rt` cells with the old filter.
  - [ ] **Claim re-fly on the final tree:** the same 9 configs x2 = 18 runs (no config change), chunk 3 of
        `docs/claim_batch_2026-11-07.md`. Scorer `results/analysis/x3b_2026-10-31/score_x3b.py` (point its CELLS at the new ids).
  - [ ] Ground computer (this laptop, headless, the rig stack down, laptop idle): one closed-loop 8-quad and one 9-quad it1
        run, logging the planner's tick, as the conservative in-loop timing (Gazebo shares the CPU).
  - [ ] Optional: re-run the solver bench (`tools/sil_bench.py`) for 2-10 with the two switches, idle, on this laptop.
  - [ ] Figure: replace the solve-time figure's simulation panel with the ground computer's tick at 1 iteration for
        2-9 carriers (bench + the closed-loop points) against the 100 ms period, the real flights (2-4, 5 iterations) kept.
  - Earlier exploratory it1 runs (23 Oct): R2464 (8 hover: tilt max 0.08 deg; solve 44 / tick 79 ms p50, p95 101),
    R2587 (8 -> 4), R2588 / R2597 (8 -> 4 -> 8, tilt max 3.3 deg).
- [~] **X8 the paper's method through a loss** (card `docs/experiments/2026-10-30_x8_network_loss.md`, bars frozen 30 Oct; ours vs the
  port told 0.35 s after, + the never-told port; the exact law never told as a smoke) (one-day box): 4 quads hover, one unannounced loss, world C (their centre
  hook) and the rim. Gets a critic if it touches the network's law.
  - [x] Current tree: R3592-R3593, R3636-R3647 (14 runs; feed F042: L1-L3 hold; reviewer 4 Nov: supported with corrections). Not the final tree.
  - [ ] **Claim re-fly on the final tree:** 7 configs x2 = 14 runs, chunk 6 of `docs/claim_batch_2026-11-07.md`. The rim's
        ours arm flies `x8_rim_ours_noxy` (its T: int_k_xy 0 in its own params file, the fallback of 31 Oct 17:16). The hub
        cell's ours arm keeps the integral as flown (Wesley's call: the hub's ring tilt is free).
  - Scorer: `tools/x8_metrics.py` was never written; the reviewer's `results/analysis/review_x2_x8_2026-11-04/rev_x8/x8_check.py`
    (edit its RUNS and its `results/2026-11-0[34]` glob).

## P1: the final-tree re-fly of every table the thesis cites (X11)

After the code-tree freeze (Fri 6 Nov), with the slope estimate, each config x2: the batch of Sat 7 - Sun 8 Nov,
`docs/claim_batch_2026-11-07.md` (chunks 7-9; its section 7 maps every item below to the restructured draft).
The 30 Oct manifest (`docs/analysis/x11_refly_manifest.md`, `configs/batches/x11_{A,B,C}.txt`, 143 configs) was made
from the 30 Oct draft. It is **stale**: the Results were restructured on 3 Nov and are headings only since 4 Nov. A
read-only scan of the full `results_discussion.tex` (5 Nov) cites 114 configs: none new, and the families marked [-]
below gone. Wesley 30 Oct 16:45: B re-flies only where a table claims it (the 4.3 joins). The exact law
(`c_net_quan_const_katt8_hubcom_hover_L100`, R2805/R2812) flies R6c (decided 30 Oct 16:26). The copies of the configs
that type ema are `x11_<source>` (the `r1m_*` and `u_*` names are reserved by their generators' --check).

- [~] **4.1 fixed team, simulation (Q007, Wesley 3 Nov: run now, on the current tree):** b2/b3/b4 x hover, fig8s075 (a 0.75 m,
      0.2 m/s), linespinc075, `*_L100_slope`, 0.86 kg, x2 = 18 runs: R3489-R3506 (current tree, not the final one).
      Scored with best_data.score and tools/yaw_track.py as the real flights. The circle, line and spin are out (Wesley 3 Nov).
      [ ] **Re-fly on the final tree** (no config change), chunk 7. This block replaces the old `b*_fig8big` / `b*_linespinc`
      cells (R2330-R2363) the draft still cites, once F031 is applied in the thesis.
- [-] **4.1 later setup** (2): b2_hover_L100_slope_p115, b4_fig8s075_h5_v200_L100_slope_p115: not needed while 4.1.4 is out (Q007).
- [~] **4.2 the 4 -> 3 release:** r1m_ta_a1_r1, r1m_ta_a1_r2 (120 deg gap): R3507, R3508 (current tree; they flew **ema**,
      the generated configs type it, though their registry notes say slope). Scored as RIG-1008's releases (Q007).
      [ ] **Re-fly on the final tree on slope** as `x11_r1m_ta_a1_r1`, `_r2` (x1 each: they are the repeats), chunk 7.
- [ ] **4.2 the gap table** (`tab:res_gap`): `x11_r1m_ta_a1_r3`, `x11_r1m_tb_{g135,g150,g165,g180}_r{1,2}`,
      `x11_r1m_tb_g180drop_r1`, 10 runs, chunk 9 (prepared, flies only on Wesley's word). Every planned-pull arm
      (r1m_tb_g150p and others) is dropped (Wesley 2 Nov).
- [ ] **4.3 the newcomer's join** (7 x2 = 14), chunk 8b: fri30_sq6b, fri30_sq6b_g, the two freeze variants (as they are), and the
      three that lift a T (carriers 0/3/6) as `_noxy` copies: sq_floorjoin_hover_L100_p115, its v1a30s_zt_xy08_bd variant and
      sq_pickup ... cppreattach_bd (int_k_xy 0 by the T rule: Wesley's call, the claim batch section 8).
- [ ] **4.3 a departed carrier rejoins** (4 x2 = 8), chunk 8c: rj3_pairrelease_rejoin_L100 lean / leanneg / tip,
      rj4_softdetach9_rejoin_L100 (no change: slope by default, int_k_xy 0 through the rig chain params).
      s8_chain4back_R50_L100_it5 flies as `x3_s8_chain4back_it5_tip` in X3b's chunk.
- [ ] **4.3 catching off the plate (U)** (30 x2 = 60), chunk 8a: `x11_u_meas_*` and `x11_u_slot_*` at d00, ri02-ri20, t02-t16.
- [ ] **4.4 the planner's tick, the harder cases:** the s8 set flies in X3 (chunk 2); count once.
- [ ] **4.5 comparison with Quan et al.** (16 x2 = 32), chunk 8d: `x11_c_pl_*` (hover, linex v200 / v400, hub x 3),
      `x11_c_net_tune_portfinal_*`, `x11_c_net_tune_paperfinal_*`, `x11_c_net_hub_*`, `x11_c_net_quan_const_katt8_hubcom_hover_L100`.
- [ ] **The switch of solvers' count (F012, 31 Oct):** seven of its 20 switches came from runs no longer re-flown (r1m_ta_a2
      planned pull x3, r1m_w4_gi x2, r1m_dl_d1 x2; not prepared). Proposed: count the switches from X1's resized runs on
      the final tree (34 switches) with `docs/analysis/switch_first_solve_1m_2026-10-31.md`'s method. Wesley's call.
- [-] No longer cited in the restructured draft: `lim_a_*` / `lim_e_*` (the position-limit paragraph is cut), `r1m_traj3_*`
      (appendix only, commented out), group C (`ta_*`, `tb_*`, `dl_d1`, `w4_gi`: 0.55 m), `b3/b4_*_p115`.
- [-] Methods' simulation section (`b2_hover_*_ema_cd060`, `tri_floorjoin_*`, `fri30_h3t_*`) and the network smokes:
      exploratory, cited as flown (their filter and delays are the point); no re-fly unless Wesley says otherwise.

X11 in the batch: 114 runs (+10 on his word), ~6 h of the ~22 h batch.

## P2: if time allows (in this order)

- [ ] **X5 pickup and delivery:** QUAD4 picks up the plate, drops it in the net, rejoins. 5 runs (R2898-R2900, R2905 exist).
- [ ] **X6 random failures:** seeded carrier and time on the figure-8, 4 / 6 / 8 quads x10, + 5 no-resize each. 45 runs,
      needs the seeded event generator and 6-quad configs.
- [ ] **X7 join ablations:** blend vs step (`rejoin_blend_s` ~0), attach paused vs moving. 10 runs.
- [-] X9 delay tolerance: dropped by Wesley 30 Oct.
- [ ] X0 a join buys margin: analysis of flown runs only, no new runs.

## Figures that need simulation data

- [ ] The simulated join (contact, lift, confirmation, switch, fold-in): from a 4.3 join run after the re-fly
      (`figures_needed.md` in the drafts folder).
- [ ] X1's "what the resize is worth", X4's "the ring outlives every pack", X2's chain on three paths, X6 if run.
