# Simulation runs still to do (the checklist; started 30 Oct 2026)

Every simulation run the thesis still needs, so the lab PC can take the next block whenever it is free. Tick a line the
moment its runs are scored, with the run ids. Order: top to bottom within a priority. The approved rescope
(`docs/plan_rescope_2026-10-30.md`) and `docs/decisions.md` (30 Oct) are the source.

**Status:** `[ ]` to do, `[~]` running, `[x]` done (run ids), `[-]` dropped (why).
**Rules:**
- Lab PC only, through `tools/remote/lab_batch.py`.
- Card and frozen bars before any claim run.
- 2 repeats.
- The 0.86 kg ring for claims, plus one 1.15 kg cell per table.
- Claim tables fly on one code tree, the final one (freeze Fri 6 Nov), with the slope velocity estimate.
- A registry row per run.

## P1: the headline results and the claims

- [~] **X1 release ablation** (card `docs/experiments/2026-10-30_x1_release_ablation.md`, configs `configs/experiments/x1_*`).
  - [x] First pass 30 Oct: 63 runs R2915-R2978 (R2952 void, re-flown R2964), scored `results/analysis/x1_2026-10-30.json`.
        The measured pull holds the ring through a release (late or no resize fly the same: B1, B2 falsified); the
        planned pull is 2.4-4.4x worse (B3); in the T our planner settles 7-13 deg off level against 2.2 without the
        resize (U5 missed): the T fix comes first (Wesley 30 Oct). Claims re-fly on the final tree.
  - Arms: ours, no resize, resize 0.5 / 1 / 2 s late, planned pull, no resize + planned pull.
  - Cells: 4 quads hover and figure-8, 8 quads hover, plus the 1.15 kg cell.
  - Running on the lab PC since 30 Oct ~12:40.
  - The paper's hand-over arm moved to X8 (no loss handling for it in the code yet).
  - [x] **Uneven cell (Wesley 30 Oct), flown 30 Oct R2965-R2978:** the square 0/3/6/9 to a T, one unannounced release in hover, the same seven arms x2
        (14 runs), `drop_tilt_deg: 80` in simulation only (the square-to-T release peaks 57-61 deg on 1 m cables; at 30 every
        arm drops). Compares recovery: peak tilt, dip, time back to level. Queued with the X1 agent after its batch.
- [ ] **X4 endurance relay** (card `docs/experiments/2026-10-30_x4_endurance_relay.md`; merged into week8 30 Oct 16:12, its
  37 tests pass; B = 600 s in its configs, every --check passes).
  - Pack budget B = 10 min. Command: `lab_batch.py --slots 2 --cpus 12 --wall-cap-min 180 --warm-from-config`.
  - [ ] `x4_relay_short` (3 swaps, exploratory first check, ~25 min)
  - [ ] `x4_relay_30min` x2 (~2.3 h each)
  - [ ] `x4_fixed_B` x2 (the fixed-team baseline)
  - [ ] `x4_relay_30min_m115` x1 (the 1.15 kg cell, new world approved 30 Oct)
- [ ] **X2 the supervisor's chain** 4 -> 3 -> 2 -> 3 -> 4, a lap per stage, hover / figure-8 / spinny line. 11 runs.
  - [ ] **The full chain on the figure-eight in one run (Wesley 30 Oct):** square 4 -> 3 -> 2 -> 3 -> 4, a lap per stage,
        two variants differing only at the square-to-T step: (A) that step announced (soft detach, 25-29 deg seen in
        R2840/R2842), (B) that step unannounced with `drop_tilt_deg: 80` (simulation only). Every other step
        unannounced. x2 each. Configs from c4_softdetach9_fig8_L100 / c4_detach9_fig8_L100 and the chain laps.
  - The square-to-T step stays unannounced: the drop rule puts the ring down there, reported as the rule's boundary.
  - The rest (T -> pair -> T -> square) flies from a fresh start in the T layout (decided 30 Oct).
  - Configs to write (the laps exist since 23 Oct).
  - [x] The full chain's card 30 Oct (`docs/experiments/2026-10-30_x2_full_chain_fig8.md`), bars FROZEN ~15:46 (Wesley);
        configs `x2_full_fig8_{soft,unann,base}` + `configs/twin/x2_chain_tip80.yaml` (pass --check). 6 runs ~1 h.
- [ ] **X3 the 8-quad set as claims:** R2768-R2779's nine cases re-flown as claims, plus the never-flown 8 -> 2 -> 8
  (`s8_chain2back`). 20 runs, needs an it5 config copy and the scorer merge for same-time releases.
  - [ ] s8 hover / figure-8 / orbit (baselines)
  - [ ] s8_adjpair, s8_chain4b2b, s8_chain4b2b_orbit, s8_chain4back_fig8laps, s8_chain4back_pairs,
        s8_chain4back_relblend, s8_chain4task, s8_fig8rel2, s8_relay
  - [ ] s8_chain2back (new)
- [ ] **X3b eight and nine carriers in real time** (Wesley 30 Oct: "show that running the controller at sim time for 8 quads
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
  - [ ] Lab PC, x2 each: s8_hover / s8_chain4back / s8_fig8 at it1 (configs exist: `*_it1.yaml`), plus the two switches;
        s9_hover_R53_L100_it1 (exists, never flown) and a new s9 chain (9 -> 5 -> 9) on nine_rigid_ground_rig_R53_L100.
  - [ ] Ground computer (this laptop, headless, the rig stack down, laptop idle): one closed-loop 8-quad and one 9-quad it1
        run, logging the planner's tick, as the conservative in-loop timing (Gazebo shares the CPU).
  - [ ] Optional: re-run the solver bench (`tools/sil_bench.py`) for 2-10 with the two switches, idle, on this laptop.
  - [ ] Figure: replace the solve-time figure's simulation panel with the ground computer's tick at 1 iteration for
        2-9 carriers (bench + the closed-loop points) against the 100 ms period, the real flights (2-4, 5 iterations) kept.
  - Earlier exploratory it1 runs (23 Oct): R2464 (8 hover: tilt max 0.08 deg; solve 44 / tick 79 ms p50, p95 101),
    R2587 (8 -> 4), R2588 / R2597 (8 -> 4 -> 8, tilt max 3.3 deg).
- [ ] **X8 the paper's method through a loss** (one-day box): 4 quads hover, one unannounced loss, world C (their centre
  hook) and the rim; ours, our port, the exact law. 15 runs. Gets a critic if it touches the network's law.

## P1: the final-tree re-fly of every table the thesis cites (X11)

After the code-tree freeze (Fri 6 Nov), with the slope estimate. Each config x2, plus one 1.15 kg cell per table.
Generated from the run ids the full draft cites (Results and discussion, 30 Oct); the left column is the draft's section.
**Manifest (30 Oct 16:01): `docs/analysis/x11_refly_manifest.md`, batch lists `configs/batches/x11_{A,B,C}.txt`, made by
`tools/x11_manifest.py` from every file the full draft inputs.** 143 configs: A 86 cited claims (re-fly x2), B 39 cited
exploratory runs (the s8 set flies in X3, fri30_sq6b/sq_floorjoin in 4.3; the rest need a call: re-fly or keep as
exploratory), C 18 from before 21 Oct ("The Switch of Solvers": ta_/tb_ R1887-R1908, dl_d1, w4_gi; the older tree and
cables: re-fly, cite as the earlier setup, or drop). One config is missing: `c_net_quan_const_katt8_hubcom_hover_L100`
(the exact published law, R2805/R2812), to rebuild from R2805's params_file.yaml.

- [ ] **4.1 fixed team, simulation columns** (12 configs): b2/b3/b4 hover, fig8big, linespinc (`*_L100_slope`; b3 hover
      flew ema), r1m_traj3 circle / linex / spin
- [ ] **4.1 later setup** (2): b2_hover_L100_slope_p115, b4_fig8s075_h5_v200_L100_slope_p115
- [ ] **4.2 the gap and the measured pull** (14): r1m_ta_a1_r1-r3, r1m_tb_g135 / g150 / g150p / g165 / g180 (r1, r2),
      r1m_tb_g180drop_r1
- [ ] **4.3 the newcomer's join** (7): fri30_sq6b, fri30_sq6b_g, the two freeze variants, sq_floorjoin_hover_L100_p115,
      sq_floorjoin ... v1a30s_zt_xy08_bd, sq_pickup ... cppreattach_bd
- [ ] **4.3 a departed carrier rejoins** (5): rj3_pairrelease_rejoin_L100 lean / leanneg / tip, rj4_softdetach9_rejoin_L100,
      s8_chain4back_R50_L100_it5
- [ ] **4.3 catching off the plate (U)** (30): u_meas_* and u_slot_* at d00, ri02-ri20, t02-t16
- [ ] **4.4 the planner's tick** (1): s8_chain4back_fig8laps_R50_L100_it5 (with X3)
- [ ] **4.4 harder cases** (12): the s8 set of X3; count once
- [ ] **4.5 comparison with Quan et al.** (18): c_pl_* (hover, linex v200 / v400, hub), c_net_tune_portfinal_* (our port),
      c_net_tune_paperfinal_* (the damping form), c_net_quan_const_katt8_hubcom_hover_L100 (the exact law), c_net_hub_*
- [-] Discussion's delay runs fri30_h3t_* (R2909-R2914): exploratory, cited as the delay lesson, no re-fly

About 100 configs, ~200-260 runs, ~6 h on the lab PC's slots.

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
