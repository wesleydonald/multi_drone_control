# The final-tree claim batch, Sat 7 - Sun 8 Nov 2026

Every simulation claim re-flies on the one tree frozen Fri 6 Nov (the code freeze), on the lab PC, before the
simulation freeze Mon 9 Nov (`docs/handin_plan.md`). Prepared Thu 5 Nov ~13:20 AEDT; **nothing is launched**.

Status of the check (5 Nov ~13:05, on the tree of that hour): all 134 configs pass `tools/run_experiment.py --check`
(the only warnings are the square L100 ring's inertia, as every L100 square config has). `lab_batch.py --dry-run` passed
its config check for every chunk; its slot plan did not print, because the lab PC was full (the landing agent's four
slots: "no free slot within the core budget"). The planner warm-up resolves for all 134 configs, with no rod in any
plan (`prebuild_planner.config_builds`). Run both checks again on the frozen tree (section 5).

## 1. The settings every claim config flies, and where they come from

| Rule | Source | How it is typed |
|---|---|---|
| The `slope` velocity filter on every carrier | CLAUDE.md (Gazebo); decisions.md DECIDED 2026-10-28 (lines 604, 610) | `lab_env: SIM_MOCAP_VEL_FILTER: slope`, or nothing (the emulator's default is slope on every carrier). Attach configs list their carriers in `MDC_MOCAP_VEL_DRONES`, so the newcomer keeps ema |
| The plan is massless, in simulation and on the rig | decisions.md DECIDED 2026-11-04 ~22:50 and ~23:00 (lines 790, 792; feed F046) | `plan_rod_level` false (sim.yaml); no batch config types the rod |
| A cell that forms a T flies `int_k_xy 0`, and the text says so | decisions.md DECIDED 2026-10-31 17:16 (line 751): the fallback if the rod in the plan was not clean by Wed 4 Nov. The rod was reversed 4 Nov ~23:00 (line 792). On the new links the T leans 16.5-17.8 deg with the integral on and 1.77 deg with it off (CORRECTION 2026-11-05 ~00:45; R3699, R3700, R3701) | The cell's params file or a launch arg. **The statement for the table:** the sideways integral is off for that cell's whole flight, so a steady sideways offset is not cancelled (R3701: level within 1.8 deg, 47 mm off its point) |
| One tree per claim table, 2 repeats | CLAUDE.md (Exploration vs claims) | one sync and build, then every chunk `--no-sync --no-build` |
| Simulation results are judged after the run; a card's frozen bars stay as written | decisions.md DECIDED 2026-11-02 23:14 (line 759) | no bar moves |
| No planned-pull arm anywhere | decisions.md DECIDED 2026-11-02 23:03 (line 758) | X1 flies 5 arms (no `_plan`, `_nplan`); X1's B3 and U3/U4 cannot be scored |
| X4 flies 26 leaves | decisions.md DECIDED 2026-11-04 ~07:45 (line 766) | as configured |
| X11's scope; the exact law flies R6c | decisions.md DECIDED 2026-10-30 16:45 and 16:26 (lines 746, 745) | section 7 |

Launch defaults, world SDFs and `params.py` were not touched.

## 2. What changed: new configs (every source is kept, because registry rows cite it)

Naming: `x3_` and `x11_` are slope copies (as `x3_s8_hover_it5`). The `r1m_*` and `u_*` names are reserved, because
`tools/make_refly_configs.py --check` (a gate test) and `tools/make_catch_offset_configs.py --check` flag any file of
those names they did not generate. `_noxy` means int_k_xy 0 and `_norod` means plan_rod_level false. Every copy starts
with a header line that names its source and the change. Its description reads "Final-tree claim copy of <source>
...: <why>. Was: <the old description>". A script checked each copy against its source: the loaded YAML differs only in
the name, the description and the key listed below.

| Claim | New configs | The one change | Why |
|---|---|---|---|
| X1 sqT | `x1_sqTnoxy_d05`, `_d10`, `_d20` (`x1_sqTnoxy_ours`, `_nors` already existed with the same setting; their descriptions are still the 30-31 Oct exploratory ones) | params `configs/twin/x1_sqT.yaml` -> `x1_sqT_noxy.yaml` (int_k_xy 0) | the square-to-T release leaves a T |
| X3 | `x3_s8_orbit_it5`, `x3_s8_fig8laps_it5` (P's base, never flown on one tree with its chain), `x3_s8_{adjpair, chain4b2b, chain4b2b_orbit, chain4back_fig8laps, chain4back_pairs, chain4back_relblend, chain4task, fig8rel2, relay}_it5_tip` | `SIM_MOCAP_VEL_FILTER` ema -> slope | R3648-R3673 flew ema |
| X3 | `x3_s8_chain2back_it5_tip_noxy` | slope, and params `s_chain_pair_tip.yaml` -> `s_chain_pair_noxy_tip.yaml` (the same + int_k_xy 0) | the 8 -> 2 -> 8 passes the T 0/4/6 (R3672/R3673: the T sat at ~14 deg and the T -> pair unload never ran) |
| X4 | `x4_relay_30min_v3_norod`, `x4_fixed_B_v3_norod`, `x4_relay_30min_m115_v3_norod` | `plan_rod_level` true -> false | the massless plan. int_k_xy 0 and `attach_datum_shift` false stay typed in `x4_relay_nowalk.yaml` (the adopted v3 setting, Wesley 2 Nov); as `x4_relay_short_v3_norod` (R3483) |
| X8 rim | `x8_rim_ours_noxy` + `configs/twin/x8_rim_ours_noxy.yaml` (`x8_rim_ours.yaml` + int_k_xy 0) | the params file | our arm flies the T. It sits in the method's own params file, so the arms still differ only in mode and that file. The port arms are unchanged: after the hand-over the network flies, not our planner's integral |
| 4.2 | `x11_r1m_ta_a1_r1`, `x11_r1m_ta_a1_r2` | ema -> slope | the generated r1m configs type ema. **R3507/R3508 flew ema** (manifests), though their registry notes say slope |
| X11 | `x11_c_*` (16), `x11_u_*` (30), `x11_r1m_ta_a1_r3`, `x11_r1m_tb_*` (9) | ema -> slope | the C, U and gap tables flew ema (23-27 Oct) |
| X11 4.3 | `sq_floorjoin_hover_L100_p115_noxy_low` (5 Nov: was `_noxy`), `sq_floorjoin_hover_L100_p115_v1a30s_zt_xy08_bd_noxy`, `sq_pickup_hover_L100_p115_cppreattach_bd_noxy_over` (5 Nov: was `_noxy`) | `int_k_xy: 0.0` launch arg; the base cell also the low route; the pickup cell flies OVER the carriers (Wesley 5 Nov ~17:40: on the low route his C++ planner stalls 0.7 m short of the drop point, R3767, R2904) | the carriers lift a T (plates 0/3/6) until QUAD4 joins on 9. **Wesley's call** (section 8, item 1): the rule's own example is a T left by a release |

**X11 4.3, 5 Nov ~16:00 (Wesley).** (1) His carriers' tracking check is 0.20 m (was 0.05): with int_k_xy 0 the T hovers 0.050 m off the planner's hover point, ring_bridge publishes the carriers' paths from that point, and his 0.05 m check latched his C++ backend off for good 2 s into TRANSIT_TO_REATTACH, so `sq_floorjoin_hover_L100_p115_noxy` never welded (R3738, R3749, R3750). The check stays on; its bound is a new parameter `cooperative_tracking_check_half_*` 0.20 in `c1f6_moving_rendezvous.yaml`, and the carriers' collision inflation stays 0.05 (at 0.20 their tether boxes cover the low transit end; `CHANGES_FROM_TEJEN.md` 5 Nov). It is C++: the lab tree must be rebuilt before these runs (no `--no-build`). (2) The base square floor-join flies the low route, QUAD4 never over the carriers (his rule of 29 Oct): `sq_floorjoin_hover_L100_p115_noxy_low` types the route as `_v1a30s` does (attach_transit_clearance 0.30, minimum_search_z_m 2.327, approach_offset_z 0.30, max_swing_angle_rad 0.02); the old cell flew it over at 3.20-3.25 m. Verify first, one run each: `configs/batches/verify1105_x11_joins.txt`.

Unchanged and already right: X1 h4, f4, h8, h4p115 (a Y or 7 of 8, slope); X2 (`x2_chain_tip80.yaml`: slope and int_k_xy 0
for the whole run, base included); X3b (slope; `x3b_s9_chain4back_rt_tip` types no lab_env, so slope by default); the X8 hub
and port arms; 4.1's `b*_slope`; X11's `rj3_*` and `rj4_*` (no filter typed, so slope; int_k_xy 0 through
`configs/rig/chain_pair_tip.yaml` / `chain_pair_noxy.yaml`, unchanged since 22 Oct); `fri30_sq6b*` (slope; the triangle,
no T).

## 3. The batch, in order (longest runs first)

The lists are `configs/batches/claim1107_<chunk>.txt`: one path per run, so a repeat appears twice, longest first. Every
chunk runs on 2 slots x 12 cpus. Minutes per run are each config's batch-log wall at 2 x 12 on 3-4 Nov (the warm-up
included). The X11 minutes are estimates (marked *): their last runs flew 4 slots x 4 cpus or on the laptop, so they are
scaled to RTF ~0.6, plus the ~2.5 min warm-up. The chunk wall comes from packing that list onto 2 slots in order.

| # | Chunk file | Claim | Configs x repeats | Runs | Min per run | Chunk wall | `--wall-cap-min` |
|---|---|---|---|---|---|---|---|
| 1 | `claim1107_1_x4` | X4 endurance relay | relay_30min x2, relay_30min_m115 x1, fixed_B x2 (`*_v3_norod`) | 5 | relay 130-157, m115 159, fixed 47-74 | 5.0 h (5.3 at the slowest seen) | 200 |
| 2 | `claim1107_2_x3` | X3 the 8-quad set | chain2back_noxy, chain4back_fig8laps, chain4task, fig8laps (base), relay, pairs, relblend, chain4b2b, chain4b2b_orbit, fig8rel2, orbit, adjpair; each x2 | 24 | 9-37 (fig8laps base 21*) | 4.6 h | 60 |
| 3 | `claim1107_3_x3b` | X3b 8 and 9 in real time, + X3's hover / figure-8 baselines and N1's comparator | x3b_s9_chain4back_rt_tip, x3_s8_chain4back_it5_tip, x3b_s8_chain4back_rt_tip, x3b_s9_hover_rt, x3b_s8_fig8_rt, x3_s8_hover_it5, x3_s8_fig8_it5, x3b_s8_hover_rt, x3_s4_hover_it5; each x2 | 18 | 4-29 | 2.6 h | 60 |
| 4 | `claim1107_4_x1` | X1 release ablation | h8, h4, f4, sqTnoxy: ours, nors, d05, d10, d20; h4p115: ours, nors; each x2 | 44 | h8 9-15, 4 quads 4-5 | 2.1 h | 30 |
| 5 | `claim1107_5_x2` | X2 full chain, figure-8 | soft, unann, base; each x2 | 6 | 8.5-9.5 | 0.5 h | 40 |
| 6 | `claim1107_6_x8` | X8 the paper's method through a loss | rim: portu, ours_noxy, port; hub: portu, ours, port, quanu; each x2 | 14 | 3-5.4 | 0.5 h | 30 |
| 7 | `claim1107_7_s41_42` | 4.1 fixed team, 4.2 the 4 -> 3 release | b2/b3/b4 x hover, fig8s075, linespinc075 (`*_L100_slope`) x2; `x11_r1m_ta_a1_r1`, `_r2` x1 (they are the repeats) | 20 | 2.7-4.7 | 0.6 h | 30 |
| 8a | `claim1107_8a_x11_u` | X11 catching off the plate (U) | `x11_u_{meas,slot}_{d00, ri02..ri20, t02..t16}` x2 | 60 | 6.7* | 3.3 h | 30 |
| 8b | `claim1107_8b_x11_joins` | X11 the newcomer's join (4.3) | sq_pickup_cppreattach_bd_noxy, sq_floorjoin_noxy_low, sq_floorjoin_v1a30s_bd_noxy, fri30_sq6b_g_frz, fri30_sq6b_frz, fri30_sq6b_g, fri30_sq6b; each x2 | 14 | 4.4-10* | 0.7 h | 45 |
| 8c | `claim1107_8c_x11_rejoin` | X11 a departed carrier rejoins | rj4_softdetach9_rejoin_L100, rj3_pairrelease_rejoin_L100_{tip, lean, leanneg}; each x2 | 8 | 6-6.5* | 0.4 h | 30 |
| 8d | `claim1107_8d_x11_quan` | X11 comparison with Quan et al. | `x11_c_pl_*` (6), `x11_c_net_tune_{port,paper}final_*` (6), `x11_c_net_hub_*` (3), `x11_c_net_quan_const_katt8_hubcom_hover_L100` (R6c); each x2 | 32 | 5-5.5* | 1.4 h | 30 |
| 9 | `claim1107_9_x11_gap_onword` | X11 the gap table (Wesley 5 Nov ~14:00: yes, the integral as flown) | `x11_r1m_tb_{g135,g150,g165,g180}_r{1,2}`, `x11_r1m_tb_g180drop_r1`, `x11_r1m_ta_a1_r3`; x1 each (the `_rN` are the repeats) | 10 | 3.5-4.5* | 0.3 h | 30 |

**Total, with chunk 9: 255 runs, ~2600 run-minutes, ~22 h wall. Add the sync and build (~15 min) and each chunk's
start (~2 min), so about 22-23 h.** Started Sat ~08:00, it ends Sun ~06:00-07:00. That leaves Sunday for the voids'
re-flies, the scoring and the reviewer. Disk: ~10 GB of results (X4 ~0.6 GB a relay, an 8-quad chain ~0.13 GB, a 4-quad
run 15-50 MB); the laptop had 21 GB free on 5 Nov.

## 4. The slot plan

2 slots x 12 cpus for every chunk:
- It is the setting every claim run of 3-4 Nov flew (RTF 0.62-0.73 with 4 quads, 0.21-0.23 with 8).
- X3b's timing bars (R8) were measured at 12 cpus a slot. X4 and the 8-quad chains sit near their RTF void floors at 12
  cpus already.
- At 4 cpus a 4-quad run fell to RTF ~0.25 and its run took 2.5 times as long (x1_h4_ours 4.6 against 1.8 min,
  x2_full_fig8_soft 23 against 7.8). Throughput per core is no better.

The chunks run one after another on one synced tree. A failed run does not stop its chunk, and the wall cap stops a hung
run, so a crash or hang ends at most one chunk. The script goes on to the next chunk either way.

Not chosen:
- Two lanes at once (the relays on one 12-cpu slot while the short runs share the other): the same ~22 h total, but two
  `lab_batch.py` processes would reserve run ids at the same moment, which has never been tried.
- 4 slots x 6 cpus: the 8-quad RTF at 6 cpus was never measured.

The cost of chunk 1 alone on 2 slots is ~40 min in which one slot idles while the 1.15 kg relay finishes.

## 5. Commands

Pre-flight, on the frozen tree (Fri 6 or Sat 7 Nov):
```bash
source ~/ros2_humble/install/setup.bash && cd ~/multi_drone_control && source install/setup.bash
date; git log --oneline -1; git status --short | wc -l             # the frozen tree, as Wesley left it
./tools/gate.sh --quick                                              # must pass on this tree
python3 tools/run_experiment.py --check $(cat configs/batches/claim1107_*.txt | sort -u) | grep -c '^ok '   # 134
ssh -o BatchMode=yes drones@drones 'who; uptime; docker ps --format "{{.Names}}"'   # nobody else, no wesley_slot*
df -h /home | tail -1                                                # >= 15 GB free
python3 tools/remote/lab_batch.py --dry-run --slots 2 --cpus 12 --warm-from-config --no-sync --no-build \
  $(cat configs/batches/claim1107_1_x4.txt)                          # now prints "2 slot(s) [1, 2] at 12 cpus" and the jobs
```

The script, `~/mdc_overnight/claim1107.sh` (copy it there; never edit it while it runs):
```bash
#!/usr/bin/env bash
# The final-tree claim batch of 7-8 Nov (docs/claim_batch_2026-11-07.md): one sync + build, then every chunk on that tree.
source ~/ros2_humble/install/setup.bash >/dev/null 2>&1
cd ~/multi_drone_control && source install/setup.bash >/dev/null 2>&1
B=configs/batches
echo "start $(date '+%a %d %b %H:%M') $(git rev-parse --short HEAD), $(git status --short | wc -l) dirty"
up=$(ssh -o BatchMode=yes drones@drones "docker ps --format '{{.Names}}' | grep -c '^wesley_slot'")
[ "$up" = "0" ] || { echo "REFUSED: $up wesley_slot containers up $(date +%H:%M)"; exit 1; }
tools/remote/lab_build.sh --ws || { echo "SYNC/BUILD FAILED $(date +%H:%M): nothing ran"; exit 1; }
run() {   # chunk, wall cap (min); a failed chunk does not stop the next
  echo "== $1 start $(date '+%a %H:%M'), $(wc -l < $B/$1.txt) runs"
  python3 tools/remote/lab_batch.py $(cat $B/$1.txt) --slots 2 --cpus 12 --wall-cap-min $2 \
    --warm-from-config --no-sync --no-build
  echo "== $1 EXIT $? $(date '+%a %H:%M')"
}
run claim1107_1_x4 200
run claim1107_2_x3 60
run claim1107_3_x3b 60
run claim1107_4_x1 30
run claim1107_5_x2 40
run claim1107_6_x8 30
run claim1107_7_s41_42 30
run claim1107_8a_x11_u 30
run claim1107_8b_x11_joins 45
run claim1107_8c_x11_rejoin 30
run claim1107_8d_x11_quan 30
run claim1107_9_x11_gap_onword 30         # Wesley 5 Nov ~14:00: yes (section 8, item 3)
echo "DONE $(date '+%a %d %b %H:%M')"
```

Launch, and stop:
```bash
setsid nohup bash ~/mdc_overnight/claim1107.sh > ~/mdc_overnight/claim1107.log 2>&1 &
echo $! > ~/mdc_overnight/claim1107.pid                             # wait on this PID (never a pgrep loop)
# stop: the script's own PID first (no next chunk), then the running lab_batch (it removes its own containers)
kill $(cat ~/mdc_overnight/claim1107.pid); pgrep -af 'tools/remote/lab_batch.py'   # then: kill -TERM <that pid>
```

To fly one chunk alone (e.g. a re-fly, or chunk 9 on his word), on the tree already synced:
`python3 tools/remote/lab_batch.py $(cat configs/batches/<chunk>.txt) --slots 2 --cpus 12 --wall-cap-min <cap> --warm-from-config --no-sync --no-build`.

From the sync to the last chunk's start, edit nothing on the laptop under `src/`, `tools/`, `configs/` or
`simulation_assets/`. The lab runs the synced tree, but each chunk's manifests record the laptop's diff at that chunk's
start. A void re-fly config can be added only once its chunk has started.

After each chunk:
1. One registry row per run (append with LF; never rewrite the file).
2. The one-tree check: one hash for the whole batch. Docs edits change `diff_sha256` but not this one.
   ```bash
   python3 - R#### R#### ... <<'EOF'
   import glob, hashlib, re, sys
   KEEP = ('src/', 'tools/', 'simulation_assets/', 'configs/twin/', 'configs/rig/')
   for rid in sys.argv[1:]:
       txt = open(glob.glob(f'results/20*/{rid}_*')[0] + '/git_diff.patch', errors='replace').read()
       code = [b for b in re.split(r'(?m)^(?=diff --git )', txt) if re.match(r'diff --git a/(\S+)', b)
               and re.match(r'diff --git a/(\S+)', b).group(1).startswith(KEEP)]
       print(rid, hashlib.sha256(''.join(sorted(code)).encode()).hexdigest()[:12])
   EOF
   ```
   It reads the same hash for R3520, R3563 and R3586 (3 Nov 21:23) and another for R3648 and R3660 (4 Nov).
3. A void run (its card's validity rule: late ticks over 1 %, the RTF floor, a missed event) is flown once more.
4. Score the chunk (section 6), then post it in `docs/results_feed.md` and tick `docs/sim_runs_todo.md`.

## 6. Who scores each claim

| Claim | Scorer | Before it can run on the new ids |
|---|---|---|
| X1 | `tools/x1_metrics.py R.... --table --bars --json results/analysis/x1_<date>.json` (bars B1, B2, B4; sqTnoxy by U1, U2, U5; the validity items) | nothing: it reads cell and arm from the name (`x1_<cell>_<arm>`), so `sqTnoxy` is its own cell |
| X2 | `tools/x2_metrics.py R.... --json ...` (5 Nov: the 31 Oct `score_x2.py` + `lap_tools.py`, ids as arguments, variant from the name; prints the card's every-bar table, B1 by the later runs' bar) | nothing |
| X3 | `tools/x3_metrics.py R.... --json ...` (5 Nov: the per-case wrapper rewritten over s_metrics; L/J/N4/P per case, adjpair's and the pairs' same-time releases merged, P against the bases given) | nothing; give chunk 3 with chunk 2 (the bases, the N1 comparator, the claim chain) |
| X3b | `tools/x3b_metrics.py R.... --json ...` (5 Nov: the 31 Oct `score_x3b.py`, ids as arguments, cell from the name, `_tip` dropped; C8 and N9 verdicts) | nothing |
| X4 | `tools/x4_metrics.py R.... --bars --json results/analysis/x4_<date>.json` (R1-R7, F1, the void items) | nothing (`--config` only if a manifest's scenario is not this tree's) |
| X8 | `tools/x8_metrics.py R.... --json ...` (5 Nov: the reviewer's `x8_check.py` generalised: cell and arm from the name, `ours_noxy` is ours; L1-L3 and the validity items) | nothing |
| 4.1 | `tools/s41_metrics.py R.... --json ...` (5 Nov: `best_data.py`'s `score()` + `tools/yaw_track.py`, as the real flights (Q007), in F031's table) | nothing |
| 4.2 | `tools/s41_metrics.py` (the same command: x1_metrics' detection, tilt rise, peak, T6, dip and settled, and the thesis table's back in band, F033) | nothing |
| X11 U | `tools/catch_offset.py` | nothing (arm from the params, config from the run dir) |
| X11 Quan | `tools/c_metrics.py --table R....-R.... --out results/analysis/c_compare_<date>.json` | nothing (arm and case from the params) |
| X11 rejoin | `tools/rejoin_metrics.py R.... --json ...` (5 Nov: the rejoin card's stages, x2_metrics' rejoin(); prints `tab:res_rejoin`'s form) | nothing |
| X11 joins | `tools/attach_clearance.py`, `tools/m1_metrics.py` (flight), `tools/n2_bands.py` (R2843, R2894's metrics) | nothing |
| X11 gap (chunk 9) | `tools/release_bands.py metrics` (dtheta, T6, as R2604). `tools/refly_score.py` finds runs by their `r1m_` names, so give it ids, not tables | nothing |

Commands for the weekend's ids: section 10.

## 7. X11 against the restructured draft

The 30 Oct manifest (`docs/analysis/x11_refly_manifest.md`, `configs/batches/x11_{A,B,C}.txt`, 143 configs) came from
the 30 Oct draft. On 3 Nov the thesis was restructured (THESIS_STATE), and since 4 Nov its Results are headings only.
`tools/x11_manifest.py` would read only those headings and would overwrite the 30 Oct files, so it was not re-run. A
read-only copy of its scan (5 Nov ~13:00) read the full `results_discussion.tex` (with `res_scale_8.tex`) in place of the
headings stub, appendices out (Wesley commented them out in 09efb4c). It finds 114 cited configs (A 79, B 35, C 0):
nothing new, and these gone:

| 30 Oct item | Now | In this batch |
|---|---|---|
| `b*_fig8big`, `b*_linespinc` (the old fixed-team table, R2330-R2363) | still cited, but F031 (pending in the thesis) replaces them with the 4.1 block's `fig8s075` / `linespinc075` (R3489-R3506) | no: chunk 7 flies the 4.1 block |
| `lim_a_*`, `lim_e_*` (the position limit, The Experimental Platform) | not cited (his cc5a994 cut the paragraph) | dropped |
| `r1m_traj3_{circle,linex,spin}` (appendix_other_paths) | appendix only, commented out; the circle, line and spin are out (Wesley 3 Nov) | dropped |
| group C: `ta_*`, `tb_*`, `dl_d1_second_loss`, `w4_gi_g2_release_orbit` (0.55 m, before 21 Oct) | not cited | dropped (the switch count: section 8, item 4) |
| `b3/b4_*_p115` (Setup 2's 1.15 kg cells) | not cited; `b2_hover_L100_slope_p115` only in the commented Setup 2 subsection | dropped (Q007: not needed while 4.1.4 is out) |
| Methods' simulation section: `b2_hover_L100_ema_cd060`, `b2_hover_vf_ema_cd060`, `tri_floorjoin_*` (6), `fri30_h3t_*` (6) | cited in `system_overview.tex` as the delay and filter checks | no: section 8, item 6 |
| `c_net_smoke_hub_*` | cited as smokes | no (decided 30 Oct 16:45: stay exploratory) |
| `s4_hover_L100_it5`, `s4_release9_3915_L100_it5`, `s8_hover_R50_L100_it5`, `s8_chain4back_R50_L100_it5` (res_scale_8) | cited | by `x3_s4_hover_it5`, `x1_h4_ours`, `x3_s8_hover_it5`, `x3_s8_chain4back_it5_tip` (the marker check on, as every chain cell since 3 Nov; R2766/R2767 flew without it) |
| the s8 harder cases (B, R2768-R2779) | cited | X3 (chunk 2) |

What X11 flies (chunks 8a-8d): U 60, the 4.3 joins 14 (decided 30 Oct 16:45: "the simulated joins of 4.3"), the rejoins
8, the Quan comparison 32 (the exact law on R6c included). That is 114 runs, 10 more on his word (chunk 9). The 30 Oct
estimate of 200-260 runs shrank with the dropped families.

## 8. Open questions for Wesley (recommended option first)

**Answered 5 Nov ~14:00 (Wesley): 1 int_k_xy 0 (as prepared); 3 yes, the integral as flown (chunk 9 on); 5 no more 1.15 kg cells, the text says which tables have one (X1, X4). Taken on the recommendation (the main chat; Wesley may override): 2 as flown; 4 the switches from X1's resized runs; 6 cited as flown; 7 as prepared, with one short relay in the pre-freeze smoke; 8 not re-flown; 9 dropped.**

1. **The 4.3 joins that lift a T** (`sq_floorjoin` x2, `sq_pickup`): fly them with int_k_xy 0 (prepared, in chunk 8b),
   or as flown, with the integral on? Recommended int_k_xy 0: it is the same T with the same zero-share carrier. To fly
   them as flown, put the three source names back in `claim1107_8b_x11_joins.txt`.
2. **X8's hub cell** also leaves a T, but on the hub the ring's tilt is free and the card says it is no capsize there.
   Our arm keeps the integral on as flown (in the batch), or it gets int_k_xy 0 too (a one-line params copy)?
   Recommended: as flown.
3. **The gap table** (`tab:res_gap`, chunk 9, prepared and commented out): re-fly it on the final tree? Recommended yes:
   it is a claim table, 10 runs, ~20 min. Its 180 deg rows leave a T, but keep the integral as flown there: the ring
   tips or drops within ~1 s, before the integral acts, and int_k_xy 0 in those rows alone would be a second variable
   in the gap sweep.
4. **The switch of solvers' 20 switches** (4.1.2's text): seven came from runs that are no longer re-flown (`r1m_ta_a2`
   planned pull x3, `w4_gi` x2, `dl_d1` x2; not prepared). Recommended: count the switches from X1's resized runs on the
   final tree instead (h4, f4, h8, sqTnoxy, h4p115 with ours and the late arms: 34 switches on one tree, no planned pull).
5. **The 1.15 kg cells:** X2's (A on `four_rigid_ground_rig_L100_p115.sdf` at throttle_max 0.9, x1; the card's call,
   never written). X3, X3b and X8 have none, and the todo's rule says one per table. Which tables still want one?
6. **The methods' simulation runs** (`*_ema_cd060`, `tri_floorjoin_*`, `fri30_h3t_*`): keep them cited as exploratory, as
   flown? Recommended yes: the ema filter and the added delays are what they show.
7. **X4** keeps int_k_xy 0 without the rod, as typed in its v3 params. Only R3483 (the short relay) flew that setting:
   it held, slightly worse than with the rod. The integral on without the rod has never flown. OK as prepared?
8. **X3b's one-iteration extras R3686-R3693** (ema) are not re-flown: they are the X3b cells with the old filter.
9. X2's other cells of its 11-run plan (hover and spinny line from a T start) were never written: drop them?

For the lab chat (not a decision): R3507/R3508's registry notes say slope, but their manifests show ema (the generated
r1m configs type it). A correction row is owed.

## 9. Not in this batch
- X3b's laptop timing runs R8b/R9b (`x3b_s{8,9}_chain4back_rt0`, tuf, headless, rig stack down, on a quiet night).
- P2's X5-X7, and every exploratory and rig-prep run (the landing fix, QUAD4's fold-in cap): their own cards.
- If a feature that passes before the freeze becomes a simulation default, the batch flies it with no config change
  (the tree is the claim). A config that types a switch off would need a copy.

## 10. Scoring (added Thu 5 Nov ~14:00)

Every claim has a scorer under `tools/` that takes run ids (or run directories) as arguments. Each was proven on the
old runs: it reproduces the registered numbers (the table below). Run them from the repo root on the laptop after each
chunk's read-back (`source ~/ros2_humble/install/setup.bash && source install/setup.bash` first: x4_metrics loads the
run's config). A run without `logs/run.csv` is skipped with a note. A run that is void by its card's rules (late ticks,
the read-back, a missed event) is flagged in the output and left out of the bars where the card says so. Add a void
run's re-fly id to the list; the void id can stay.

The ids are reserved when each chunk starts. Fill each chunk's first and last id from its batch log (`lab_batch.py`
prints them); `seq` builds the list. If a chunk's ids are not contiguous (a re-fly, or another batch in between), list
them by hand instead.

```bash
cd ~/multi_drone_control && source ~/ros2_humble/install/setup.bash && source install/setup.bash
D=2026-11-08                                    # the date in the output names
ids() { seq -f 'R%04g' "$1" "$2"; }             # ids FIRST LAST -> R#### ... (each chunk's, from its batch log)
C1=$(ids <c1_first> <c1_last>)                  # 1  X4         5 runs
C2=$(ids <c2_first> <c2_last>)                  # 2  X3        24
C3=$(ids <c3_first> <c3_last>)                  # 3  X3b       18 (+ X3's hover / figure-8 bases, N1's comparator, the claim chain)
C4=$(ids <c4_first> <c4_last>)                  # 4  X1        44
C5=$(ids <c5_first> <c5_last>)                  # 5  X2         6
C6=$(ids <c6_first> <c6_last>)                  # 6  X8        14
C7=$(ids <c7_first> <c7_last>)                  # 7  4.1, 4.2  20
C8A=$(ids <c8a_first> <c8a_last>)               # 8a X11 U     60
C8B=$(ids <c8b_first> <c8b_last>)               # 8b X11 joins 14
C8C=$(ids <c8c_first> <c8c_last>)               # 8c X11 rejoin 8
C8D=$(ids <c8d_first> <c8d_last>)               # 8d X11 Quan  32
C9=$(ids <c9_first> <c9_last>)                  # 9  X11 gap   10

python3 tools/x4_metrics.py  $C1 --bars --json results/analysis/x4_$D.json          # R1-R7 per relay, F1 per fixed team
python3 tools/x3_metrics.py  $C2 $C3 --json results/analysis/x3_$D.json            # chunk 3 brings the bases; its x3b_* runs are left out
python3 tools/x3b_metrics.py $C3 --json results/analysis/x3b_$D.json               # V, C8, N9
python3 tools/x1_metrics.py  $C4 --table --bars --json results/analysis/x1_$D.json  # B1, B2, B4; sqTnoxy U1, U2, U5
python3 tools/x2_metrics.py  $C5 --json results/analysis/x2_$D.json                # A1-A6, B1-B6 (B1 by the 31 Oct bar)
python3 tools/x8_metrics.py  $C6 --json results/analysis/x8_$D.json                # L1, L2, L3
python3 tools/s41_metrics.py $C7 --json results/analysis/s41_$D.json               # F031's table, the release (F033)
python3 tools/catch_offset.py $C8A --table --json results/analysis/catch_offset_$D.json
python3 tools/attach_clearance.py $C8B                                             # tab:res_join: his TAKEOFF -> weld, tries, proof
python3 tools/rejoin_metrics.py $C8C --json results/analysis/rejoin_$D.json        # tab:res_rejoin
python3 tools/c_metrics.py --table $C8D --out results/analysis/c_compare_$D.json
python3 tools/release_bands.py metrics $C9                                         # tab:res_gap (dtheta, T6)
```

`results/analysis/` needs `git add -f` (CLAUDE.md). With `--bars`, x1 and x4 also write `<json>_bars.json`.

**Proven on the old runs (5 Nov).** Each scorer was run on the registered runs. "Reproduced" means every barred number
matches the registry and the feed at the registered rounding, and where an old JSON exists it is identical. The only
exceptions are the keys the new scorers add.

| Claim | Scorer | Old runs | Reproduced | Differences, and why |
|---|---|---|---|---|
| X1 | `x1_metrics.py --bars` | R3520-R3563; bars also on R2918-R2978 | yes: `x1_2026-11-04.json` identical; on the 30 Oct runs every verdict of the card's Results (B1 falsified, B2 falsified, B3 confirmed, B4 holds frozen / T6 misses at 1.15 kg, U1-U4 confirmed, U5 missed) | none. On the 4 Nov runs f4's B2 is falsified by a 0.52 deg inversion (bar 0.5); F035 states no bar verdicts |
| X2 | `x2_metrics.py` | R3586-R3591 | yes: the reviewer's `x2_refly.json` identical | B1's step now passes by the later runs' bar (31 Oct, 5 deg: 3.10, 2.66); the original T6 (35.46, 35.40 s) is printed as a recorded miss. R3590's missing /tracker_3 read-back is read from the tracker's own params.json, so R3590 is valid, as the reviewer found by hand |
| X3 | `x3_metrics.py` | R3648-R3673 + R3582-R3585 | yes, every number in the 26 registry rows | R3661's /drone_5 on plate is 20.654 s, which prints 20.7 (registry 20.6). The relblend rejoiner's "after" tension (registry 1.93 N) is its lowest planned tension in the second after the resize; the first tick at n=7 reads 1.98. chain4task's /drone_5 reference 20.4 mm is printed as missing the 2 cm draft bar; the registry judged it held after the run. Three definitions were found by matching the registry: the airborne box from LIFTED, P on the move from `n2_bands`' `t_from`, and the settled window ending at the path's end (the planner's clock at its largest) when it comes before LAND |
| X3b | `x3b_metrics.py` | R3568-R3585; also R2979-R3231 | yes: `x3b_2026-10-31.json` identical (side analyses left out); every registry number for R3568-R3585 | **the registry's and F037's "fold-ins" (1.25-1.39 deg) are the ATTACH's rise from its 5 s mean, not J's fold-in peak** (the tilt magnitude, weld to +15 s: 1.04-2.00 deg). Both are printed. Both pass J's 5 deg; P's per-rejoin row uses J's quantity. The filter check reads the mocaps' read-back, because the s9 chain types no lab_env |
| X4 | `x4_metrics.py --bars` | R3519, R3564-R3567 | yes: `x4_2026-11-04.json` identical; card section 10's table | R2 reads the planner's n (run.csv's `d*_attached` columns are 0 in these runs). Card section 10 gives R3519's RTF as 0.21; its manifest says 0.226 |
| X8 | `x8_metrics.py` | R3592-R3593, R3636-R3647 | yes: the reviewer's `x8_rows.json` identical (its alternative `ts_xyz` dropped) | L1 confirmed on E; by the frozen rule the scorer also lists R3637's hub "over 300 mm for 1.02 s" as a loss. The reviewer said not to claim it in the text |
| X11 rejoin | `rejoin_metrics.py` | R2594-R2596, R2602, R2603, R2712, R2713; R2457, R2584 | yes, every cell of `tab:res_rejoin`; R2457 5.83, R2584 10.61 deg | R2712's time over 5 deg prints 0.21 s (the sum of samples) against the registry's 0.19 s (first to last sample). The T's share is the planner's RESIZE_INPUTS (0.100 N); the registry used the rider's share before the release (0.099 N) |
| 4.1, 4.2 | `s41_metrics.py` | R3489-R3508 | yes, every number in F031's table, F031's release line and F033's 3.34 / 3.23 s | none |
| X11 joins | `attach_clearance.py` | R2843, R2844, R2862 | yes: 38.7, 38.5, 31.6 s, proven on try 1 | not re-checked: the frozen tip (R2907/R2908) and the pickup (R2865) |
| X11 U, Quan, gap | `catch_offset.py`, `c_metrics.py`, `release_bands.py` | - | not re-checked on 5 Nov | committed tools that take ids, as before |

Tests: `tools/test/test_claim_scorers.py` (names to cells, arms and cases; X3's comparator bars; X3b's filter read-back;
X8's bars; on this laptop, R3592, R2596 and R3507 are reproduced read-only), and the bar tests added to
`test_x1_metrics.py` and `test_x4_metrics.py`.

## 11. The claims with returns on the earlier masses (Wesley 6 Nov ~10:30)
After the reviewers of X3/X4 and the A/B R3907/R3908 (the frozen code returns in 37.0 s with the final quadrotor masses and
16.6 s with the earlier ones), the claims with returns re-fly on the earlier masses (decisions.md DECIDED 2026-11-06):
`configs/batches/oldlinks1106_{1_x4,2_x3,3_x3b_chains,5_x2,8c_rejoin}.txt` (55 runs; configs in
`configs/experiments_oldlinks/`, worlds `simulation_assets/*_oldlinks.sdf`, `tools/make_oldlink_worlds.py --check`). They
replace chunks 1, 2, 5 and 8c and X3b's chain cells; X3b's hover and figure-8 cells (R3809-R3818, final masses) stand.
Script `~/mdc_overnight/claim1106_old.sh`: after X1's resumed chunk, one sync + build, the five old-mass sets, then chunks
6, 7, 8a, 8b, 8d and 9 on the final masses. Scoring as section 10, with the new ids.
- 6 Nov ~14:20 (Wesley): `tools/remote/lab_batch.py` no longer fetches `gz_record/` (disk). Chunks started after the edit
  carry it in their recorded laptop diff, so their one-tree hash differs from the earlier chunks' by that file only; the
  synced tree that flies on the lab PC is unchanged.
- 6 Nov ~15:20: `tools/remote/lab_sync.sh` excludes `/.claude/` (agent worktrees with build outputs, 1.6-1.9 GB each, were
  being copied to the lab PC; the lab never uses them). Orchestration only, as the gz_record exclude above.
