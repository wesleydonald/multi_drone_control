# Session state (read after every compaction; keep it short and current)

Updated: Fri 23 Oct 2026, 11:18 AEDT (from `date`). Only what a new session needs to act. History: `git log -p docs/SESSION_STATE.md`,
`docs/decisions.md`, `results/registry.csv`, and `docs/archive/`.

## HANDOVER (Fri 23 Oct 17:41) -> NEW CHAT ON BRANCH week7: READ THIS FIRST
- **Branch:** `week7`, made from day-1021 (be34f95). Its first commit (5f8bd51) holds everything up to 23 Oct evening: the 21-23 Oct
  work, the tejen-sync2 merge, figures, both papers and these handover files. NOT pushed (working rules); Wesley pushes
  `week7` and merges to main himself.
- **Today in one page:** `docs/reports/day_report_2026-10-23.md`. Then the 23 Oct blocks below, and every
  `DECIDED 2026-10-23` entry in `docs/decisions.md`.
- **Lab (check it first):**
  - Batch 18u (25 runs) was launched at 16:37 by the old chat (lab_batch PID 340122 on the laptop; log in the old
    scratchpad). If that chat closed, lab_batch may have died and removed its containers.
  - Check with `ssh drones@drones 'docker ps | grep wesley_'` and `ls results/2026-10-23/R27*`. Re-fly whatever is
    missing from `docs/handover/2026-10-23/batch18u_list.txt` (`--wall-cap-min 60`).
  - Then `batch18h_list.txt`: the centre-hook test, 14 runs. Its smoke pair is already valid.
  - Then `batch18_list.txt`: S's 8-quad claims at 5 iterations plus the 7 harder cases, 16 runs;
    `--slots 2 --cpus 12 --wall-cap-min 150 --warm-from-config`.
  - Lab only when nobody else is on it: `lab_free_watch.py` waits for that.
- **Batch 18u FINISHED 18:25** (the old chat's lab_batch is done; the lab is free): 25 runs. The paper's damping form at its
  offline-best gains LOST the ring in all 6 runs (R2744-R2749: the ring tilts past 30 deg 20-30 s after the hand-over;
  rows written). The Thu 29 rehearsal and the clearance A/B (R2738-R2743) all PASSED the runner and are NOT scored yet
  (score with install_tally.py). Next: batch 18h, then 18.
- **Scoring owed:**
  - whatever of 18u the scoring agent did not finish (U's re-flies, our planner's C cells, the paper's damping form,
    the Thu 29 rehearsal and its clearance A/B);
  - 18h against the frozen G/P/A/O bars (C card);
  - 18 against S's frozen bars: N2 is 8.83 deg; N4 opens 2 s after the last blend (`tools/s_metrics.py`);
  - cases 6 and 7: their rows carry "critic waived by Wesley 23 Oct".
- **Offline owed:** S's bench bar B, which needs an idle laptop and no lab sync.
- **Rig:**
  - Wed 28: `docs/rig/wed_2026-10-28.md` (15/15 lines resolve on the final tree).
  - Thu 29: `docs/rig/thu_2026-10-29.md` (5/5 resolve; the 60 s GIVE UP rule for Tejen's install gate;
    carry_clearance 0.25).
  - Wesley sends Tejen `docs/rig/note_tejen_install_gate_2026-10-23.md`.
- **Papers (IEEE RA-L):**
  - `docs/paper/ral_2026-10-23/`: v1, with the comparison.
  - `docs/paper/ral_2026-10-23_reconfig/`: v2, reconfiguration focus, reviewer-polished; `REVIEW.md` lists what's
    left. Pending results are red `\placeholder{}`s.
- **Figures:** `docs/figures/review_2026-10-23/` and `review_2026-10-23_best/` (Wesley's style: memory
  thesis-figure-preferences).
- **Tools copied from the old scratchpad:** `docs/handover/2026-10-23/`. `append_rows.py` backs the registry up beside
  itself, so COPY it into the new scratchpad before using it. Also `check_sheet.py`, `check_thu_eval.py`,
  `install_tally.py`, `e5.py`, `lab_free_watch.py`.
- **For Wesley:**
  - send Tejen's note;
  - 2 SQP iterations on the rig (a launch default: twin closed-loop check plus reviewer first);
  - case 6's path keep-out and clearance;
  - merging `week7` into main.
- **Thesis worktree** `~/mdc_wt/thesis` (branch thesis-rig): today's drafts and the two corrected chapters, committed
  there (dd6aeac).

## ULTRACODE PUSH (from Thu 22 Oct ~14:20) -- READ THIS FIRST
**23 Oct morning (07:27): Wesley's answers to every open decision are in decisions.md (DECIDED 2026-10-23). DISK 99 %
(1.9 GB free): nothing heavy (renders, lab batches, worlds) until Wesley decides on space; his rule is delete nothing.
Queue from his answers: Wed sheet MDC_PLANNER_CPUS=0-7; pair-swing round C world + 4 runs; S claim cells; U grid (bars
frozen with his four answers); R's five 1 m worlds + ~36 runs; rejoin leaning-pair test; highlight videos + reel.**
**07:47 status:** Wesley freed the DUPLICATE scratch only (the backup's wave1/verify + check, 3 GB; refresh.sh now
excludes them): 4.7 GB free. Wed sheet: MDC_PLANNER_CPUS=0-7 in w5 (done). Highlight videos rendered (R2597, R2464,
R2594, R2578, R2446, R2427, R2433; frames checked) + reel results/videos/reel_2026-10-23_overnight_highlights.mp4.
Running: agent 1 = round C tip-hinge world (attach_z 0.0575) + R's five 1 m worlds + their configs; agent 2 = the
rejoin leaning-pair config (config-only) + U's bars frozen on his answers + the U grid list. Then lab batches: round
C x2 per arm, the lean test, R's new tables, the U grid (~3 GB of results: S's 8-quad claim cells wait for storage).
**08:09:** Wesley freed ~/.ros/log older than 7 days (9.4 GB) and the scratchpad's reviewer scratch (2.9 GB): 18 GB
free. Built (gate green): round C tip world two_rigid_ground_rig_tip (hinge 0.0575, cable 0.53) + configs typed0175 /
typed0575 (bars FROZEN 08:05); R's five 1 m worlds (Tier B 3969/gap135/gap165/gap180, push) + 18 r1m configs; the 3-quad
T push world + rj3_..._lean / _leanneg (+3 / -3 deg; leanneg -0.0110 N m); U bars FROZEN 07:56 on his answers (56-run
list); --check now refuses a WRENCH on a world without the push plugin. BATCH 16 LAUNCHED (87 runs, ~4.5 h): round C x2
per arm, lean, leanneg, R's 25, U's 56.
**09:29:** Wed sheet checked: all 15 launch lines resolve (tools/launch_dump; fig8 0.75, dist 0.75, cable 1.0, limit
1.75); plan status table updated. Wesley picked all four next items: (1) Thu 29 readiness agent RUNNING (merge
tejen-sync2 into main, QUAD4 at 1 m, twin rehearsal configs, sheet docs/rig/thu_2026-10-29.md); (2) thesis drafts agent
RUNNING (~/mdc_wt/thesis, new files only; index DRAFTS_2026-10-23.md); then (3) pair-swing round D (card + critic) and
(4) S's claim cells on the lab after batch 16.
**11:18:** Drafts agent DONE: five results sections in ~/mdc_wt/thesis (.../Source/src/drafts_1023/, one \input at
Results_and_Discussion.tex:25, index ~/mdc_wt/thesis/DRAFTS_2026-10-23.md; builds, pp. 69-86). It caught MY C error: the
twin's "retuned" R2468/69 flew the search's stage-1 point (emit got only diss_c, vel_kp_pos); the real best never flew
(morning report corrected). Wesley: fly the best (configs/twin/c_net_portfinal.yaml + c_net_tune_portfinal_* x2 each);
correct system_overview latency + reconfiguration drop rule (done, thesis worktree; backups scratchpad/*.tex.bak).
Round C NULL (R2598-R2601 rows written: M1(b) 0.08-0.09 < 1.0 testbed bar). Wesley: reduced round D (slope x2,
cable_source model x2 on thr060; bars FROZEN 11:14 in the card) and delete rate_kff / thr_tau_s / rate filter / tip
world (agent RUNNING; keep the per-channel delay). Thu 29 agent DONE (sync2 merged, uncommitted; backups
scratchpad/sync2_merge/backup/; new configs/rig/attach_m1_1m.yaml; sheet docs/rig/thu_2026-10-29.md; gate green 1887).
Wesley: carry_clearance 0.25 (in $H and the thu29 twin configs), Nr on our announced DETACH. Scoring agent RUNNING:
R2602-R2628 (lean pair + R's new worlds). BATCH 17 READY (scratchpad/batch17_list.txt, 15 runs: C portfinal x6, round D
x4, thu29 floorjoin x2 + Nr x3 for the Nr band, frozen before Thu 08:00): launch when batch 16 ends (~12:20) AND the
deletion agent's gate is green; it now has 27 runs (+ S's s4 C cells x2: hover, fig8big, release9 at it5/it1). Then
BATCH 18 (scratchpad/batch18_list.txt, `--slots 2 --cpus 12 --wall-cap-min 60 --warm-from-config`): s8_hover x2,
s8_chain4back x2 (claims), s8_fig8, s6_chain4, s8_chain2back, s9_hover (x1). S's bars stamped FROZEN (Wesley ~07:25).
S bench bar B (laptop, idle, offline) after the U scoring agent: the drafts' 3/3/4 limit predates the speed-ups.
U's 56 (batch 16) get a scoring agent when the batch ends.
**11:31:** Deletion agent DONE (backups scratchpad/del_rt/backup/, 25 files; 9 rt/tip configs + the tip world +
test_rate_filter deleted; per-channel delay kept; gate --quick PASSED 1874; my re-run of the touched tests 103 pass).
Checks on the merged tree: Wed 28 sheet 15/15 lines resolve (fig8 0.75, dist 0.75, cable 1.0, lim 1.75); Thu 29 sheet
5/5 lines resolve with bench stand-ins (W 0.0285, L 1.033) via test_real_mode_launches.evaluate (launch_dump lacks
ResetEnvironment); the typed W=0/L=0 and a 0.55 m-era QUAD4 record are refused as designed; carry_clearance 0.25
lowers his transit heights by 0.25 (minimum_search_z 3.44 -> 3.19 m). Thesis builds clean with my two edits (135 pp,
0 undefined refs; scratch copy scratchpad/thesis_build). S bench bar B needs the laptop idle with NO lab sync: after
batches 17/18 (tonight). Note: his launch makes an empty log folder in logs/tejen_dynamic_planner/ each time it is
merely evaluated (gate tests too: 130 today); harmless, nothing deleted.
**11:33:** R scoring agent DONE: R2602-R2628 rows written (registry checked: 1 header, LF end). R now 19 hold / 6 do not
/ 2 unresolved / 0 skipped (table doc updated). Tier B at 1 m: G165 crosses the 30 deg drop line 2/2 (28 deg at 0.55 m);
G150p no longer drops (peak 29.81); B5 "freed drone disarmed alone" fails as written (G165: every quad disarmed at
z 0.10 m with the fleet, on the ground: bookkeeping, not a hazard); G180's in-air fleet disarm (1.85-1.99 m) is the
designed drop-off envelope fault. tab:faults' tilt-over-30 row: cite G165 at 1 m (my call under his re-fly-at-1-m
decision; he can keep the 0.55 m history instead). E5 lean pair: R2602 +3.26 deg PASS, R2603 -2.67 deg PASS (lean before
LAND 1.88 vs the 2 deg bar): each gets its 2nd run in batch 17 (now 29 runs). Reviewer for R's new rows RUNNING.
**11:47:** R reviewer 2 [weak]: every number reproduces (0.01), all 25 runs valid, no bar re-set; corrections applied to
docs/analysis/refly_1m_table_2026-10-23.md (backup scratchpad/refly_1m_table.before_reviewer2.md): B4 a MARGINAL hold
(0.19 deg, repeats 2.50 apart; the tier_b must-not-say stands), B5 NOT CONFIRMED (not "does not hold"), G165's drop is an
S MISS 2/2 (its post-hoc ">=" exemption withdrawn), C-D1 vertical and C-D4 EMPTY holds. Tally 19 / 5 / 2 / 1 not
confirmed. 11:45: at Wesley's request, the 23 Oct highlights reel emailed to wesleydonaldnz@gmail.com (his Chrome Gmail
account) in 3 lossless parts (Chrome's upload limit 10 MB per file; parts in scratchpad/email_reel/); confirmed SENT.
make_reel's title cards clipped long lines (run id cut off at the left): FIXED in tools/make_reel.py (verdict wrapped to <= 3 lines of 56 chars; checked on R2446 and R2603 cards). Today's emailed reel keeps the old cards.
**12:23:** BATCH 16 DONE (87/87 ran; one runner FAIL, R2634 u_slot_t04; exit 1 from that). U scoring agent RUNNING
(R2629-R2684 against the 07:56 bars). gate --quick re-running before batch 17 (log scratchpad/gate_pre17.log); batch 17
(29 runs) launches when it passes.
**12:27:** gate PASSED; BATCH 17 LAUNCHED (task bgpo2gnxe, log scratchpad/lab_batch17.log, 4 slots, lab idle): C
portfinal x6, round D x4, thu29 floorjoin x2 + Nr x3, S s4 cells x12, E5 lean + leanneg x1 (~1.5 h). Then batch 18 (S
8-quad, 12-core slots).
**14:05:** BATCH 17 DONE 13:55 (29 runs, 79 min; 27 PASS). Scored: round D CONFIRMS both arms (slope 0.07/0.06, cable
model 0.01/0.01 vs 7.39/7.36 deg; R2691-R2694; card updated with rounds C and D); C's real best (R2685-R2690): line 0.4
99-105 mm vs our planner's 46 (stage-1 156), yaw 14 deg on the line, 22-51 deg in hover (C card has a Results section
now); E5 2nd runs: R2712 FAIL (5.28 deg, the cap's dip), R2713 PASS; reported as is (Wesley). U answers (Wesley): 6
owed runs on today's tree; rim mechanism NOT SHOWN. Runner fixed: no mocap-staleness check after /fleet/landed (R2634's
race; test added; gate PASSED). Thu 29 rehearsal: R2696/R2697 killed by the 30-min wall cap while his planner's C++
transfer search kept failing its final collision checks (time to TRANSIT 84 to >284 s; pre-merge 22-84 s): the merged
C1F backend and/or carry_clearance 0.25: INVESTIGATION AGENT RUNNING. S s4 claim scoring AGENT RUNNING (its C verdict
decides whether S's 8-quad claims fly at 1 iteration). LAB BUSY (another user, 23 of 24 cores): batch 18u stopped at
14:03 (R2714-R2724 VOID); watcher b6a3vcas0 (scratchpad/lab_free_watch.py) exits when the lab is free -> relaunch
scratchpad/batch18u_list.txt (11 runs), then batch18_list.txt (S 8-quad, --slots 2 --cpus 12 --wall-cap-min 60).
**14:15:** S's C CONFIRMS (R2700-R2711: C1-C7 hold in both repeats of every cell; it1 as good as it5, the figure-8 1.6 mm
closer; closest margins R2711's release cell, 0.31 / 0.37 deg). But the card picks 1 iteration only if Q holds too, and
Q had never run: Q1-Q5 (planner_replay over the nine flights, task bgxt5ipp0, log scratchpad/s_q_replay.log) and Q6
(solve_scaling n 8 --quality, task b6kvbx05i, log scratchpad/s_q6.log) RUNNING on the laptop (untimed; they rebuild
solvers in <repo>/generated: prebuild_planner.py before the next LAPTOP twin batch; ~/mdc_rig_generated untouched).
The 8-quad claims (at it1) wait for Q and the lab. tools/s_metrics.py: clocks unaligned (planner /clock vs runner, 3-23
s) and node logs read past run.csv's end (false VOIDs): FIX AGENT RUNNING (the scorer's wrapper scratchpad/s_c_score/
is the reference). Thu 29 investigation agent still RUNNING.
**14:28:** Thu 29 investigation DONE: the stall is Tejen's merged install gate (execution_authority.hpp runIfVersionCurrent;
every carrier/ring sample bumps his world version): 3,533 of 3,541 plans discarded (R2695-R2699, verified with
scratchpad/c1f_diag/install_tally.py); not the clearance (18 final-check failures in 3,689). Wesley: Tejen's fix (note
docs/rig/note_tejen_install_gate_2026-10-23.md, Wesley sends it); sheet: 60 s GIVE UP stop rule + the hold note;
carry_clearance stays 0.25. Rows R2695-R2699 written (R2696/R2697 VOID, wall cap). batch18u_list.txt now 17 runs (+ the
two re-flies and the cc050 A/B x2): launch with --wall-cap-min 60 when the lab is free. lab_batch does not bring his
logs back (logs/tejen_dynamic_planner on the lab): the A/B's replans.csv must be fetched (install_tally.py does).
**14:33:** s_metrics FIXED (agent: run clock via the manifest's t_ready_sim, node logs cut to run.csv; test; gate PASSED;
R2700-R2711 reproduce the wrapper exactly). N4 REVISED (Wesley) for the claim runs: opens 2 s after the last
REJOIN_BLEND_OVER (s_metrics n4; n4_resize = the original; test added); R2597 6.3 mm / 0.89 deg revised, 19.3 / 1.65
original. Design agent RUNNING: harder S cases (trajectories, tasks while away), a PROPOSED addendum in the S card.
**14:37:** S Q6 DONE (n = 8): i1 misses Q1-Q4, i2 misses Q4 by 0.016, i5 passes (card Results, Q6). Wesley: the 8-quad claims
fly at 5 iterations -> new configs s8_hover_R50_L100_it5, s8_chain4back_R50_L100_it5 (--check ok); batch18_list.txt is
now those 4 runs (the it1 list kept as batch18_list_it1_superseded.txt); launch AFTER batch18u with --slots 2 --cpus 12
--wall-cap-min 150 --warm-from-config. The nine-flight replay (Q1-Q5, task bgxt5ipp0) still running.
**14:52:** While the lab is busy, Wesley picked: (1) thesis figures -> FIGURES AGENT RUNNING (his style brief in memory
thesis-figure-preferences: simple, one message, minimal words, red/blue, supervisors' papers as reference; output
docs/figures/review_2026-10-23/ + contact_sheet.pdf; NOT into the thesis until he has looked); (2) update the five
drafts with today's results + write U's section -> agent when the design agent finishes (max 2); (3) today's videos +
reel -> RENDERING 12 runs on the laptop (task bwpe9f9ts, scratchpad/videos_1023/): R2695 R2699 (Thu 29 rehearsal),
R2450 R2691 R2693 (pair swing: no fix, slope, cable model), R2448 R2689 R2685 (C: ours vs the network's best),
R2639 R2684 (U), R2712 (leaning rejoin), R2710 (S release at 1 iteration); then make_reel.py ->
results/videos/reel_2026-10-23_afternoon.mp4. No laptop twin runs (not picked).
**14:59:** S Q1-Q5 DONE (nine flights): i1 misses Q1/Q2 on all four rig flights and the twin spinny line (f2slope Q4
too); i2 passes Q1-Q5 on all nine (misses only Q6's feed-forward at n = 8). Card Results updated. The 8-quad claims stay
at 5 iterations (Wesley). Open idea for Wesley (not asked yet): 2 iterations on the rig (n <= 4: converged by every Q
bar, ~half the solve time; the rig planner runs near its tick at 5): needs a twin closed-loop check at it2 + reviewer,
and his word (a launch default).
**15:12:** Design agent DONE: "More complex cases (proposed)" addendum at the end of the S card (7 ranked; 9 configs, --check
ok, gate 1876). Wesley: all five no-code cases fly as exploratory first passes in rank order after the claims ->
batch18_list.txt is now 13 runs (4 it5 claims + 9 cases; --slots 2 --cpus 12 --wall-cap-min 150 --warm-from-config);
N2 comparator 8.83 deg (card revised); cases 6-7: critic WAIVED, build behind switches when a slot frees (QUEUED:
next agent after figures/drafts). DRAFTS AGENT RUNNING (today's results + U's section, ~/mdc_wt/thesis drafts_1023/).
FIGURES AGENT RUNNING. Videos rendering (bwpe9f9ts).
**15:26:** FIGURES DONE: 13 in docs/figures/review_2026-10-23/ (+ index.md, contact_sheet.pdf rebuilt; sent to Wesley);
ring now black (plot_style.PAYLOAD_LINE), pair-rig like for like, "t after". VIDEOS DONE: 12 rendered (frames all ok),
reel results/videos/reel_2026-10-23_afternoon.mp4 (24 min; sent in 2 lossless parts, scratchpad/email_reel/). Wesley
asked why the network's yaw grows: explained (point-load model vs our rim-hung ring + our yaw-following); he chose the
paper's own geometry test: PAPER-GEOMETRY AGENT RUNNING (centre-hooked world + both arms' configs, PROPOSED C section).
Drafts agent still RUNNING. Queued for a slot: cases 6-7 build (critic waived).
**15:44:** DRAFTS DONE (5 updated + U's new section res_catch_offset.tex; index rewritten; before-copies scratchpad/
drafts_1023_before/). Wesley: fly the paper's FORM at its best gains (c_net_tune_paperfinal_* x2, in batch18u, now 23
runs); C draft cuts done; tab:faults + system_overview timing fixed; thesis builds 148 pp. BEST-RESULTS FIGURES AGENT
RUNNING (Wesley: best 2/3/4-quad hover/trajectory/detach, rig first, one figure per component: trajectory, error
components, throttle, tension, reaction time... -> docs/figures/review_2026-10-23_best/). Paper-geometry agent RUNNING.
Queued for a slot: cases 6-7 build. Lab still busy (watcher b6a3vcas0).
**16:22:** BEST-RESULTS FIGURES DONE (docs/figures/review_2026-10-23_best/, 9 figures, contact sheet; sent); Wesley: r302_3,
spin in, ls3c, more colour -> the agent is REVISING (resumed). PAPER-GEOMETRY DONE: hub world + 6 configs (gate 1878);
Wesley froze G/P/A/O, hub as built, + the as-ported net on the hub -> smoke pair at the front of batch18u (now 25), the
rest in batch18h_list.txt (14). CASES 6-7 BUILD AGENT RUNNING (critic waived; switches default off): DO NOT launch a
lab batch until it reports gate green (the batch syncs the tree). Lab order when free: 18u (25, --wall-cap-min 60) ->
check the hub smoke pair -> 18h (14) -> 18 (S 8-quad, 13 runs, 2x12 cores, cap 150).
**16:29:** Best-results set REVISED with colour (fleet: 2 orange #f79143, 3 rose #cc4778, 4 purple #6a00a8, plasma;
red/blue for planned vs measured, survivors vs released; grey references); sent. A re-render after Wed 28 needs only new
run ids in docs/figures/review_2026-10-23_best/best_data.py. The rig's "spin" is a 0.5 m circle with one yaw turn.
**16:37:** LAB FREE (16:31). Cases 6-7 agent paused src edits at a coherent point (its switches default off, 570 tests);
BATCH 18u LAUNCHED 16:37 (task bkxctkz0a, log scratchpad/lab_batch18u.log, 4 slots, wall cap 60): the hub smoke pair
first, U's 6, C ours x5, the paper's form x6, Thu 29 re-flies x2 + cc050 A/B x4. Next: read the hub smoke pair as soon
as it lands; after 18u: 18h (hub 14) then 18 (S 8-quad 13, 2x12 cores, cap 150; after the cases 6-7 agent's gate).
**16:55:** Cases 6-7 CODE FINAL, gate green (1927; switches default off; no prebuild needed). Configs s8_chain4task (case
6), s8_fig8laps (case 7 base), s8_chain4back_fig8laps (case 7) --check ok, appended to batch18_list.txt (now 16 runs).
Their registry rows must carry "critic waived by Wesley 23 Oct".
**17:29:** JOURNAL DRAFT BUILT (Wesley: IEEE RA-L, in-flight reconfiguration story, authors Donald/Torok/Deghat, results:
release + rejoin first, then scalability and the comparison): docs/paper/ral_2026-10-23/ (main.tex + sections/,
figures/, refs.bib), main.pdf 7 pp, 0 undefined; sent. Written by 4 parallel agents (Wesley lifted the 2-agent cap for
this, medium effort). Red placeholders: affiliations; the rig newcomer attach (Thu 29); the real-time bench; the
8-carrier claims at 5 iterations; the full hub test and the paper's damping form; our planner's hover/0.2 C cells;
the release table's resize times for all three rig releases (only R303_4's 0.51 s is measured). Check: the method's
description of the weld-height cap (the agent's reading of the rejoin card).
**17:34:** VERSION 2 (Wesley: full focus on reconfiguration, no mention of the other method, scalability kept):
docs/paper/ral_2026-10-23_reconfig/ main.pdf 8 pp, 0 undefined, no trace of Quan/the network; new subsections: what sets
the release transient (Tier B gap curve at 1 m and 0.55 m; measured pull 0.63x the planned at 150 deg, not "halves")
and how far off its plate a newcomer can catch (U: radially 16 cm measured vs 8 cm slot; rim 6-8 vs 2-4 cm); sent.
Version 1 (with the comparison) unchanged in docs/paper/ral_2026-10-23/.
**16:59:** HUB SMOKE PAIR VALID (R2725 ours 1.4 mm, yaw 0.0; R2726 network 11.8 mm, YAW 0.1 deg vs 22-51 on the rim): rows
written; 18h flies after 18u. Cases 6-7 agent DONE; open notes (not asked, case 6's first pass is hover): (1) on a path
the task keep-out counts every ring point of a lap, so every task on the 0.5 figure-8 is refused; (2) the task keep-out
passes 0.3 m from carriers vs the draft 0.5 m bar (reported on the first pass); (3) the descent guard rides on
traj_lap_hold_s (kept bundled, my call); (4) a release inside a lap-end hold cuts it to 1.5 s (resize holds overwrite the
counter; documented, not changed: the counter is shared with every run). Backup refresh loop ended 11:0x; its
restart was blocked by the permission classifier (Wesley may restart ~/mdc_backup/2026-10-22/refresh.sh).


**OVERNIGHT LOOP ENDED 23 Oct 07:09: report docs/reports/morning_report_2026-10-23.md; the decisions for Wesley are its
section 'Decisions for you'. Nothing committed. Lab PC idle.** Was: OVERNIGHT LOOP 22->23 Oct (Wesley ~23:40, asleep until 09:00): NO QUESTIONS; take the best option, note why, ask in
the morning report. At most 2 agents at once. Use the lab PC (4 slots, or 2 x 12-core for S's 8-quad runs; check
who/load first, yield to anyone). His 5-hour usage limit resets ~01:50 AEDT: if work stops, the 01:55 cron restarts the
loop. Morning report `docs/reports/morning_report_2026-10-23.md` by 08:45 (rig first; table, one-line verdict,
decisions), then stop the loop. No commits. Overnight queue, in order (update the status inline):**
0. After the sim-C merge: bringup suite + gate --quick (pos_limit_m 1.75 now in real.yaml); check QUAD4 is exempt from
   our limit while Tejen flies it (his pickup ball sat at (1.6, 2.0) in the 1 m twin world); fix if not (switch-free
   safety fix, tested). tools/prebuild_rig.sh again as the LAST step of the night (merges touched the planner).
1. Lab smoke runs of the merged packages, falsifiers / bars frozen by the integrator before each run (note the time in
   the card): rejoin E0 (re-weld smoke), then E1 x2; C's smoke hover at today's gains (c_net_hover_L100) and
   c_pl_linex_v400_L100; S's s8_hover first pass (a 12-core slot; report RTF and late ticks).
2. Wave 2, one agent at a time (designs in results/analysis/push_1022_designs.json; the critic's integration notes in
   docs/analysis/push_1022_plan.md): pair-swing (loop_timing.py, pair_mode.py, per-channel delay, emulator rate filter,
   rate_kff per the Betaflight note, LAB_ENV names, card 2026-10-23_pair_mode_timing with frozen bars for rounds A-C;
   no tip-hinge world: world SDFs need Wesley's word) -> rounds A and B on the lab PC.
3. U (catch offset): build, card, its runs. 4. R (the 1 m re-fly): build make_refly_configs etc., then R batches on
   the lab PC through the night. 5. The yaw-lag log columns (the plan's yaw and cable azimuths per tick).
6. Score every finished batch (registry rows via scratchpad/append_rows.py), keep this list's status current.
   STATUS 00:10: sim-C MERGED (report scratchpad/wave1/sim-C_merge_report.md; card Response; 7 items need Wesley, in
   its Response). Item 0 DONE: the limit's launch test updated for the rig default; QUAD4 exempt from our limit while
   the partner flies it (mux 'partner'/'cut'; test added); gate --quick PASSED. Lab smoke batch R2445-R2448 running
   (E0 rj0_reweld_floor, C smoke hover + line v200, our planner line v400; E0's bars frozen 00:08). Next: pair-swing
   agent once the batch has synced. 00:25: smoke batch SCORED (rows R2445-R2448): E0 FAIL on tilt (the floor re-weld
   locked off-plate: ring 5.6 deg toward the re-welded quad; joint mechanics fine); the network as ported CAPSIZES in
   yaw in the twin too (R2446 230 deg, R2447 602 deg: the C retune is needed); our planner at 0.4 m/s clean (R2448).
   Next lab batch (after both agents finish editing): E1 x2 (bars to freeze first), S's s8_hover exploratory pass
   (12-core slot), pair-swing rounds A/B once frozen. 00:28: C's offline retune search started on the laptop
   (nice 10, 6 workers, both forms, ema; out results/analysis/network_tune_search_2026-10-23.json, log
   scratchpad/network_tune_search.log): exploratory, re-scorable if Wesley changes the card's tolerances.
   00:31: sim-U BUILT (card 2026-10-23_catch_offset.md, 33 configs u_*, suites green; for Wesley: the measured arm's
   claim cut to 12 cm along the rim (16 cm puts the newcomer 0.23 m from a carrier < his 0.30 m), traj_hold_s 75, three
   new sim.yaml attach knobs at today's defaults). TODO after pair-swing ends: add
   src/dissipative_planner/test/test_attach_point_source.py to gate.sh step 2 (+ make_catch_offset_configs --check in
   step 4). U critic DONE (pasted; REWRITE: B1 the rim comparison is decided by the 0.30 m rule, M1 VOID hides FAIL,
   M2 the slot arm also changes z, M3 off-plate welds are not catches, M4 no mechanism bar); U response agent running.
   No U runs until pair-swing has landed (one frozen tree).
   00:45: pair-swing BUILT (gate PASSED 1807; rate_kff 0.012 calibrated offline to ~30 ms mean attitude; rounds A 7 runs,
   B 14 runs; card bars FROZEN 00:42; att040 swapped for att060; body-rate filter lag fit 36-48 ms). E1 bars FROZEN.
   Batch 4 queued (scratchpad/batch4_list.txt: round A x7, round B's calibration run, E1 x2) for when the U response
   agent stops editing.
   00:58: U critic ANSWERED (B1 d_task = min(d_tol, d_sep); M1 FAIL before VOID; M2 slot xy + captured z; M3 wording
   + t116 cells; M4 mechanism bar; 35 configs; U's grid bars wait for Wesley: 4 questions in its Response). Gate
   --quick PASSED on the full tree. Batch 4 LAUNCHED (R2449-R2463: round A x7 incl. the cd060 re-fly, round B's
   calibration run R2456, E1 x2 R2457/58, U's E-runs R2459-63). 00:59: batch 4 synced; R build agent and a small
   agent (planner yaw-plan log columns + yaw_track --plan; MDC_PLANNER_CPUS P-core pin switch, default off) running.
   01:51 (after the usage-limit reset at 01:50; both agents had died at the limit, work saved): batch 4 DONE
   (R2449-R2463; 2 runner FAILs in round A); scoring agent running; R build agent RESUMED; the yaw/P-core agent paused
   (its planner columns and MDC_PLANNER_CPUS are in and pass tests; yaw_track --plan and the gate left). S's s8_hover
   R2464 on a 12-core slot. C's offline retune DONE (results/analysis/network_tune_search_2026-10-23.json): the port
   form's best (diss_c 12, vel_kp_pos 1) passes every offline guard, J 115.1; the paper form has no passing point.
   Twin confirmation configs written: c_net_tune_portbest_{hover,linex_v400}_L100 (next batch).
   01:59: batch 4 SCORED, rows R2449-R2463 in (bak103). Round A CONFIRMS: the throttle path carries the swing (thr060
   7.4 deg, att060 0.6-0.8; att060 grows a slower rig-shaped mode). R2456 calibration outside band -> rate_kff 0.027 in
   the rt configs (not nokff). E1 R2457/58 FAIL on fold-in tilt 5.8/5.3 deg (0.2-0.4 s over) BUT the configs never set
   attach_traj_hold_s: blend 1.5 s, not 10: configs fixed (10.0), E1b re-fly queued. U E-runs: plumbing clean; at
   16 cm along the rim the slot arm beat the measured arm (fold-in 4.6 vs 8.7 deg; measured separation 0.16 m);
   radially the slot arm 15 deg vs measured 0.8. Batch 6 queued (scratchpad/batch6_list.txt: calibration re-fly,
   E1b x2, C portbest hover + line v400) once S's R2464 frees the lab.
   02:04: S's R2464 DONE (8 quads lift and hover level, tilt <= 0.08 deg; RTF 0.28; whole tick p50 79 / p95 101 ms,
   the residual 44-51 ms doubles the solve: critic M1 holds); row in. Batch 6 LAUNCHED (lab_batch PID 147299, log
   scratchpad/lab_batch6.log; ids from R2465).
   02:11: the yaw/P-core agent DONE: planner log columns yaw_ref0, plan_yaw1, plan_yaw5, az1_k, az0_k (log only; the
   next rig spin shows where its lag enters: tools/yaw_track.py <run> --plan); MDC_PLANNER_CPUS (unset = today) gives
   the planner nodes `taskset -c <cpus>` on every graph: Wesley decides (e.g. 0-7 on the rig laptop's P-cores). Suites
   and gate --quick green.
   02:15: R BUILT (53 r1m_* configs, 95 runs, ~3 h at 4 slots; tools make_refly_configs / thesis_runsets / refly_score,
   release_bands --factors; card 2026-10-23_refly_1m.md, bars FROZEN 02:15). Skipped (no 1 m world; ask Wesley): Tier B
   (3969 / gap135 / gap165 / gap180 L100 worlds, 11 configs) and the push rows (push_L100, 7 configs). Batch list
   scratchpad/refly/batch_list.txt; launch after batch 6 (with round B's rest if the calibration re-fly is in band).
   02:18: batch 6 DONE (R2465-R2469, all runner PASS; scoring agent running). Calibration re-fly R2465 in band at the
   edge (pitch 35/37 ms; filter unreadable in a calm hover; M1 0.07: the rig-timing twin still damps). BATCH 7
   LAUNCHED (task btqt4sity, log scratchpad/lab_batch7.log): round B's other 13 runs, then the 95-run R re-fly (~4 h,
   ids from R2470).
   02:25: batch 6 SCORED (rows in, bak105). E1b R2466/67 VOID as E1b (attach_traj_hold_s never reached the planner:
   the detach graph does not pass it) and FAIL as E1 repeats (5.82/5.75 deg): FIXED with a planner param
   rejoin_blend_s (default 10 s, rejoin only; test; dissipative suite 317 green); E1b re-fly after batch 7. C's
   retuned port (R2468/69): no capsize but the yaw guard fails (grows x1.14/s to +-60 deg; offline it held <1 deg:
   twin and offline disagree, so per section 3.3 the next offline candidates fly in the twin); on the 0.4 m/s line its
   headline is 156 mm vs our planner's 46 (R2448). Note: the network's LAND tips its quads (174-176 deg) after LAND.
   03:00: round B SCORED (rows R2470-R2482, bak106): FALSIFIES "the rig-timing twin reproduces the rig pair" (calm
   with either filter at the rig's timing; +50 ms throttle swings in the twin's own vertical mode). Round C (hinge
   world) needs Wesley; the card's delete of rate_kff / thr_tau_s / rate filter is held for his call.
   04:08: R part 1 SCORED (55 rows R2483-R2537 appended, bak108): ledger W4 / W5, Tier A T3 / T8 / T9, T15 (narrowly),
   drop D1 / D4 all HOLD at 1 m; T7t does NOT: orbit_v300 is just outside at 1 m (path 25.7 / 25.9 mm vs bar 25, radius
   ratio 0.9499 / 0.9495 vs 0.95); the other 14 T7t cells inside; spin_v300 went from outside to inside. R2511 = a lab
   runner stall after LANDED (flight complete), scored valid (Wesley may overrule). REGISTRY REPAIR: the 03:00 round B
   append had written 13 header lines (the rows were dicts): restored byte-exact to bak106, re-appended as lists
   (bak107); append_rows.py now refuses anything but lists of 12 strings.
   04:51: batch 7 DONE (108 runs; runner FAILs in x0 / C-D5 mapneg / one robust arm, being classified by the R part 2
   scorer). Gate --quick running, then batch 8 = E1b x2 on the rejoin_blend_s fix. C: the next passing port
   candidates differ only in ki_load / k_att at diss_c 12, kp_pos 1, so they would likely repeat R2468's yaw growth: not
   flown. The offline harness lacks the 4 cm pivot lever (critic, network_tune.py:18), the likeliest reason it misses the
   twin's yaw mode: a harness fix is for Wesley (C is time-boxed).
   04:59: R part 2 SCORED (40 rows R2538-R2577 appended): R tally 14 HOLD (W4, W5, T3, T8, T9, T15, drop, C-X1, C-X2,
   C-R1..R4, C-L2), 3 DO NOT (T7t orbit_v300 = the only claim changed by 1 m; C-L1 and C-D5 failed the same way at
   0.55 m), 0 void, 3 tables skipped (tier_b, gain, push rows: no 1 m world). Every runner FAIL is the harness check its
   0.55 m original also failed. New at 1 m: the L-S landing stalls 0.14-0.17 m up on the rigid 1 m rods (twin artefact):
   Wesley keeps it as a labelled result or drops the cell. Full table docs/analysis/refly_1m_table_2026-10-23.md.
   05:07: E1b R2578/R2579 flown on the rejoin_blend_s fix: blend 10.1 s, fold-in tilt 1.27 / 1.30 deg (bar 5; the 1.5 s
   blend gave 5.3-5.8), weld 24.9 s after REJOIN_START, no abort, landed; full E1 scoring + rows by an agent. R
   REVIEWER: [weak] overall: verdicts recompute, but C-L2 is OUT (L-S misses S: 13 holds, not 14), the L-S stall is not
   attributed (cable + land_rearm_touchdown both changed), C-D5 relabelled, T7t orbit_v300 not attributed to the cable,
   and 5 of 15 T7t cells rose > 1.5x; R2485's read-back lacks /tracker_2. Correction rows by the same agent.
   Attribution re-flies LAUNCHED (task b97meqbr4): traj3_orbit_v300 x2 and app_land_straight x2 at 0.55 m on this tree.
   05:13: E1b SCORED: PASS every frozen E1 bar (rows R2578/79 + 4 correction rows, bak110). E2-E4 bars FROZEN 05:13;
   batch 10 (E2 rj3_pairrelease, E3 rj4_..._land, E4 rj4_..._fig8stage) queued after the attribution re-flies.
   05:15: attribution re-flies SCORED (rows R2580-R2583, bak111): T7t orbit_v300's slip is mostly the TREE (0.55 m on
   this tree 22.7-23.7 mm / 0.954-0.956 vs 15.2 / 0.975 on the old tree; the cable adds ~2.6 mm); the L-S landing
   stall IS the 1 m cable (passes at 0.55 m on this tree). R table updated (docs/analysis/refly_1m_table_2026-10-23.md):
   13 hold, 3 do not, 3 skipped. Batch 10 (E2-E4) RUNNING (PID 180538, waiter armed).
   05:25: batch 10 DONE (R2584-R2586 runner PASS; E2-E4 scoring agent running). Batch 11 LAUNCHED (task b6ci006xq, 2 x 12-core
   slots): S's exploratory s8_chain4 (8 -> 4) and s8_chain4back (8 -> 4 -> 8 with rejoin_departed).
   05:33: E2-E4 SCORED (rows R2584-R2586, bak112): each FAILS one frozen bar: E2 tilt 10.6 deg (the blend raised the
   rejoiner's planned height 4.6 cm on the T), E3 landed 1.179 m vs 1.2 (reference stopped short), E4 staged at x
   1.709 vs +-1.6 (ramp overshoots its clamp). Fix agent running; re-fly E2-E4 after batch 11 if time allows before
   the morning report.
   05:55: batch 11 SCORED (rows R2587/R2588): the 8-quad chain 8 -> 4 flies (tilt max 3.48) and the FULL 8 -> 4 -> 8
   with four rejoins flies (each on plate in 16-18 s, 10 s blends, tilt max 3.25 deg). Exploratory; S's claim waits for
   Wesley's N_max / real-time decision.
   05:59: the rejoin fixes DONE (E2: _rejoin_cap holds the rejoiner's reference at or below its weld height through the
   blend; E3: exact out point + REJOIN_OUT_MARGIN_M 5 cm; E4: the staging ramp brakes on the bound and every node is held
   inside, bound 1.57 m = 3 cm inside the card's 1.6 for tracking). Suites + gate green. For Wesley: _rejoin_cap changes
   a carrier's reference in the flight loop (twin-only switch): critic or not. Batch 12 LAUNCHED: E2, E3, E4 + E1 x1.
   06:25: batch 12 SCORED (rows R2589-R2592): E2, E3, E4 and E1 all PASS on the fixes. R2593 (twin 0.75 m spin with the
   plan columns): the plan itself trails (node0 0.14 s, node5 0.09 s). Rig prebuild re-checked (fresh). Critic on the
   rejoin fixes: "run it", no blocker; M2 (hold the path for the 10 s blend) being fixed now, then re-fly S 8->4->8 and
   E2 x2 (+E1) on that tree before the report is final. Morning report drafted: docs/reports/morning_report_2026-10-23.md.
   06:33: M2 FIXED (a rejoin holds the path for rejoin_blend_s; the cap's binding logged at REJOIN BLEND OVER; suite 323,
   gate green). Batches 14 (E2 x2, E1) and 15 (s8_chain4back, 12-core) LAUNCHED in sequence (task br9esrx23, one shell).
   07:08: batch 14 SCORED (E2 x2 PASS 1.45 / 2.25 deg, E1 PASS 1.55; rows R2594-R2596) and S's R2597 (8 -> 4 -> 8 on the
   final tree: tilt max 3.35, the cap barely binds) logged. Morning report FINAL; loop stopped.
   00:15: pair-swing agent AND sim-U agent building in the main tree (U told not
   to touch gate.sh / config.py / simulation_communication; U's gate tests to add by hand after).
   STATUS 23:58: S critic ANSWERED (card Response; bar B now 3.5 x whole-tick p95 <= 100 ms -> N_max ~4: the 8-quad
   real-time claim should fail; Wesley picks twin-only "not real time" (recommended) or N_max chain). RIG FINDING
   (logs h3/f3h): the planner at 3 quads, 5 iterations runs near its 100 ms period on the rig (tick spacing p95
   116 ms, 10-12 % over 110 ms; in flight 2-3.4x the bench); pinned to an E-core it is 2.6x slower than on a P-core
   (laptop: CPUs 0-7 P-cores 4.8 GHz, 8-11 E-cores 3.6 GHz). Queue item 7: a default-off switch pinning the planner
   to CPUs 0-7 (launch prefix), tested offline; Wesley's word before the rig. Rows S-BENCH-1022, S-CALIB-1022 in.

**SAFETY / RESUME (17:25; Wesley ~17:24: near the session limit, slow down, lose nothing):**
- Backup on disk: `~/mdc_backup/2026-10-22/` = the whole scratchpad (wave-1 patches and reports, design caches and
  scripts, registry backups), the main tree's patch vs be34f95 plus its untracked files (tgz), and each workflow
  worktree's patch plus untracked files. Nothing is committed (Wesley's rule).
- **Wave 1 DONE ~21:20**; reviews in `scratchpad/wave1/verify_results.json`.
- **Fix A DONE ~21:55** (wf_0946430b-ce9): every 1m-defaults finding fixed in the main tree and checked (report
  `scratchpad/wave1/1m-defaults_fix_report.md`, check in `fixA_result.json`): the solver-cache CODE_DIR bug (sim-S's
  hunks), the main cache rebuilt at 0.55 for n=2-4 (n=5-10 signatures removed), legacy/0.55 m clearances unchanged
  (scaling only by max(0, cable_len - 0.55) cos(elev)), the body-to-tip arm check (40 exemptions gone), lazy imports,
  tests. Suites and gate --quick green. Open: TB-1m must fly before the 1 m clearance scaling stands; the rig's 0.583
  prebuild before Wed 28.
- **Critics DONE** (pasted under ## Critic): position limit, rejoin_departed (worktree -4 card), paper comparison
  (worktree -2 card). All three say REWRITE THE CARD before runs. Position limit: M1 a freed quad's clamped step-out can
  land it across its own tether (R0567 kind); M2 the rig cannot type pos_limit_m on a Wed line (one params_file, no
  CLI) -> each rig file or real.yaml (Wesley's word); M3 5 cm between clamp and LAND line; add LIM-e (forced breach)
  and LIM-0 (replay the rig logs through the check). Rejoin: B1 the weld gate may never pass in the twin (undamped
  ball joint, open-loop end height), B2 the fold-in does not wait for the joint. Compare: B1 bars can confirm and
  falsify one cell, B2 shared hover cell, M1 each arm scored against its own target.
- **Smoke runs R2417-R2422 SCORED, registry rows appended (bak99):** V1-V5 pass (V1 = R2409/12 within 0.2 mm;
  V3 fold-in tilt 1.63 vs R2308 1.66; V4 no distance rejections; V5 full pickup). TB-1m R2422 FAIL on its frozen
  falsifier: clearances hold (QUAD4 1.39 m, ball 0.63 m from any carrier) but QUAD4 tipped at touchdown (the ball hit
  the floor with QUAD4 1.14 m up; the twin's rigid 1 m rod levered it over, 67 deg, disarm). TB14 mechanism; the
  rig's tether goes slack. Wesley ~22:40: the clearances stand as they are, the tip recorded as a twin effect, no
  twin fix (decisions.md). Was (task bf0tnjkuq, log `scratchpad/lab_v_batch.log`): V1 b3_fig8s09_defaults_L100,
  V2 traj3_L100_fig8_v200, V3 m1c_floorjoin_hover_L100_rod, V4 twin_r9_held_L100, V5 m1_full_hover_L100,
  TB-1m m1c_land_transit_L100. Once its sync is done (task bj4hy7jw1), the main tree may be edited again.
- **Evening answers (~22:05, decisions.md):** rig paths 0.75 m (real.yaml fig8_size 0.75, traj_distance 0.75; twin
  keeps 0.9 / 1.0); the rig's own solver folder ~/mdc_rig_generated (rig_flight.sh, rctl/rdiss; tools/prebuild_rig.sh,
  started 22:15); QUAD4 under Tejen's LAND accepted; at most 2 agents. Position-limit agent DONE 22:20 (gate green; card Response + bars FROZEN 22:25; LIM-0 replay: at 1 m reach the
  1.0 m paths would trip, the 0.75 m ones peak 1.655 m) (critic fixes M1/m1,
  tools/limit_replay.py = LIM-0, card Response, lim_*.yaml on the size-0.75 figure-8). Band configs for the 0.75 m
  cells written: b{2,3,4}_{fig8s075,linespinc075}_h5_v200_L100_{ema,slope} (12, --check ok): lab batch once the V batch
  AND the position-limit agent are done (never sync a half-edited tree). Then the Wed sheet: 0.75 m lines (drop
  traj_radius:=0.9), the new bands, prebuild_rig.sh in the bench list.
- **Lab batch 2 DONE ~23:00** (19 runner-PASS; R2438 FAIL: the 60 ms delayed 2-quad pair on the 0.75 figure-8 with
  ema swung at 1.38 Hz until a drone disarmed (see-saw 50, roll 58 deg p-p; not the limit); R2439/R2440 FAIL: LIM-e
  tripped at takeoff, the quads start ~1.225 m out past the 1.2 m test limit (config error). Scoring agent running:
  bands -> n2_bands + sheet, LIM verdicts vs frozen bars, LIM-e2 config (WRENCH push past 1.75), rows to
  scratchpad/batch2/rows.json.) Was: **Lab batch 2 R2423-R2444 running from 22:26** (task bokqw9h9o, log `scratchpad/lab_batch2.log`): the 12 band runs
  for the 0.75 m paths (R2423-R2434) and LIM-a (R2435-R2438), LIM-e x2 (R2439-40), LIM-c on x2 / off x2 (R2441-44).
- **Wed sheet updated 22:27** for the 0.75 m paths (no traj_radius typed; spin ~32 deg/s; prebuild_rig.sh as bench
  item 0); the path bands are PENDING until R2423-R2434 are scored (then add the fig8s075/linespinc075 cells to
  tools/n2_bands.py --set b22 and fill the table). Rejoin-departed merge agent running in the main tree (with its
  review and critic fixes); scoring agent on R2417-R2422 running.
- **rejoin_departed MERGED ~23:00** into main (report `scratchpad/wave1/rejoin-departed_merge_report.md`; pre-merge
  copies in `wave1/rejoin_merge/backup/`): every review and critic finding answered (B2 fold-in waits for the joint,
  M1 landed wait +30 s only while a rejoiner lands, M2 rejoiner absent until its blend ends, M3 no ATTACH while a path
  runs, rod re-measured, closed loop on the end's z). B1 numbers: a released rod's end still swings ~3.6 cm at 10 s
  (R2312, tau 3.9 s; riders 13-17 s): if E0/E1 stall on swing, Wesley picks x/y damping or a damped joint. Card
  rewritten (Response, staged bars PROPOSED), E0-E4 configs pass --check. Tests and gate green. QUAD4's combined-M1
  fold-in bit-identical.
- **Batch 2 SCORED ~23:15, rows R2423-R2444 appended (bak100).** 0.75 m bands in `--set b22`, the Wed sheet table
  and `results/analysis/bands_075_2026-10-22.json` (b2/b3/b4 figure-8 ema 31.4/34.0/35.2 mm, slope 28.9/31.2/32.2;
  spinny line ema 33.4/31.0/28.9, slope 28.9/26.7/25.9; tilt 3.8-5.2 deg). LIM-a R2435-37 PASS (reach <= 1.469 m);
  R2438 FAIL (delayed pair swing, not the limit); LIM-e R2439/40 FAIL (config error: tripped at takeoff, carrier landed
  1.226 > 1.17); LIM-c R2441-44 FAIL on clearance in BOTH arms: the plate-0 detach at 1 m tilted the ring past 45 deg
  in 0.5 s (full drop); dropped quads step 1.2 m from where the ring WAS, the ring lands 0.2-0.3 m toward one, which
  then comes down 0.86-1.14 m from it (the R0567 layout; not the limit). The limit put the freed quad at 1.63/1.64 m
  (off arm 1.89/1.91). Wesley ~23:20: drop step-out not important (quads may land on the payload); no 1 m push world
  (LIM-e2 dropped, config removed). Open for Wesley: how the rig turns the limit on (real.yaml recommended).
- **sim-S MERGED ~23:20** (report `scratchpad/wave1/sim-S_merge_report.md`, backups `wave1/simS_merge/backup`): main's
  _departed_clear_r kept, res_ms logged after plan5_*, s8 return configs fixed for the rejoin checks (420 / 600 s),
  s_metrics N1/N4 as carded, NaN branch matched, lab_batch prebuilds every resize size; identity tests, all suites,
  verify_dissipative and gate --quick green; card bars PROPOSED, Critic pending.
- **S critic DONE ~23:35** (pasted into the card): REWRITE. B1 the 8-quad bar is an idle bench, the rig's h3 solve
  ran 2-3x slower; M1 the whole tick is unbarred; M2 one iteration untested at the lift and releases; M4 no comparator.
  An agent is answering it (calibration replay of h3/f3h, tick bar, lift/release cells, Response).
- **Now:** sim-C merge agent (with C's critic fixes and a Response) + the S critic-response agent. Then: paste
  S's challenge, an S Response/fix pass, a critic re-look at C's rewritten card only if Wesley wants it, then wave 2
  (pair-swing tools and switches, U, R) in the agreed order, two agents at most.
- Was next: merge rejoin-departed, then sim-S, then sim-C into main, ONE agent at a time (conflicts in the reviews), each
  followed by suites + gate; score R2417-R2422 with registry rows.
- Was: **Wave 1 RESUMED 20:55** after the reboot (task wpcrjr4ej, same run id; scratchpad restored from the backup, trees
  matched it; backup loop task b2qpz3qgk). Was: **Wave 1 PAUSED 17:30** (TaskStop, Wesley's word: near his usage limit; restart when it resets). All four
  implementations are DONE: patches and reports in `scratchpad/wave1/` (1m-defaults is applied in the main tree;
  sim-C, sim-S and rejoin-departed are in worktrees `.claude/worktrees/wf_d62e7db2-b9f-{2,3,4}` and their `.patch`).
  Only the four verifiers were stopped mid-way (they may have left partial edits: compare against the 17:30 backup
  patches before trusting a tree).
- **Restart, same session:** `Workflow({scriptPath: "~/.claude/projects/-home-wesley-multi-drone-control/f2a54328-3214-4ac3-989e-f5a0c5208168/workflows/scripts/push-1022-wave1-wf_d62e7db2-b9f.js", resumeFromRunId: "wf_d62e7db2-b9f"})`:
  the four implementations come back from the cache, the four verifiers re-run. **New session:** resume is impossible;
  run the four verifications directly on the main tree and the three worktrees (script and journal copied to the backup).
- Backup refresh loop stopped 17:38 (the backup is final and checked: every tgz tests clean, patches complete). Re-run
  `~/mdc_backup/2026-10-22/refresh.sh` in the background if work resumes.
- **After a reboot** /tmp is cleared: first restore the scratchpad with `rsync -a ~/mdc_backup/2026-10-22/scratchpad/
  /tmp/claude-1000/-home-wesley-multi-drone-control/f2a54328-3214-4ac3-989e-f5a0c5208168/scratchpad/` (the paths above and the
  workflow's prompts point there), then restart the workflow as above.
- Next (after the pause), in order: integrate the worktree patches; critics (position limit card `docs/experiments/2026-10-22_position_limit.md`,
  rejoin_departed, C's law changes); V1-V5 + TB-1m smoke runs (1m-defaults report's table); prebuild `--drone-mass 0.583`
  after the last laptop twin run before Wed; n2_bands fig8s09 cells; Thu-29 band runs; Wed sheet (tip_replay exit 2 =
  NOT A PASS; figure-8 lines type `fig8_size`); the pair-swing card (with the Betaflight rate_kff note); the yaw-lag log
  columns.

**Authority (decisions.md, DECIDED 2026-10-22 ultracode push + push answers 1-24):** Wesley said "use ultracode to work
through as much as possible". Scope: Wednesday readiness (bands, sheet, RViz; the 1 m defaults; the pair-swing cause)
and the thesis sims U, S, C, R. Code may go as far as needed, defaults included; report what fails and delete it,
report every improvement. NO commits until Wesley says (do not even ask). The lab PC may be used whenever free (check
who / load / containers first, at most 4 slots, or 2 x 12-core for S's 8-quad runs; yield to anyone). No time
constraints: work through the order, one thing finished well, then the next. Disk: delete nothing (Wesley is getting
400 GB of storage; watch free space, ~17 GB on 22 Oct).

**The plan:** `docs/analysis/push_1022_plan.md` (six package designs and the critic's integration order, gaps and run
schedule; full designs `results/analysis/push_1022_designs.json`). Day review: `docs/reports/rig_review_2026-10-22.md`.
Wed sheet (draft): `docs/rig/wed_2026-10-28.md`. Card: `docs/experiments/2026-10-22_velocity_filter.md`.

**Decided today (values a session must not re-derive):**
- Twin = rig's mocap velocity filter: payload_mocap_emulator defaults SIM_MOCAP_VEL_FILTER `ema`; configs set the twin's
  switches with `lab_env:` (tools/experiment/config.py LAB_ENV; run_experiment applies it per run, laptop and lab).
- Claim tables (R, U, C, S) fly the UNDELAYED twin; the delayed twin (~50 ms extra command delay, calibrated on
  R2397-R2404) is a named rig-timing arm once its landing stops faulting (it faults with the delay on).
- Rig filter: Wed 28 opens with the pair A/B (ema / slope interleaved x2 + P4); the claims type the winner.
- Position limit: any quad's x or y beyond +-1.75 m makes the fleet LAND (envelope LAND, not disarm); nothing is
  commanded or landed outside it (freed/dropped quads' landing points clamped). Replaces the locked "no geofence".
- Figure-8: the RIG flies size 0.9 at 1 m (outboard quad at +-1.625 m); twin tables may keep size 1.0 (R2329-R2364
  stand). Bands for the rig at 0.9: twin runs b{2,3,4}_fig8s09_h5_v200_L100_{ema,slope} (running 15:45).
- Bands: path flights score CLOCK-FREE (distance to the path + height), path-only window (first move to the planner's
  descent); lag reported, not banded (tools/n2_bands.py --set b22). Hover: 2 s after the top to LAND.
- Every path flies a 5 s hover hold first (configs/rig/traj_hold.yaml; twin drop_and_land_hold5.yaml).
- line_spin = one full turn at a constant yaw rate (load_trajectory.py, uncommitted; RIG-1022-ls3c).
- 1 m: drone_mass 0.583 (rig + prebuilt solvers); the ring's CoM as rig default (attach_z 0.0175, payload_rest_z 0.0275;
  attach/M2 forced rest 0.0275); QUAD4 attach_cable_len = rod from its pivot (configs that disagree are refused; old
  ones exempt); DEPARTED_CLEAR_R stays 1.2 m (scaled only for S's ring); path-clear/take-back scale with reach.
- The cable's real hinge is ~5.75 cm above the ring's CoM (4 cm above the magnet face, ~8.5 cm above the floor at rest;
  Wesley 22 Oct); the rig types 1.75 cm; tip-hinge twin world approved, rig typing later as its own A/B.
- S: 8 quads with the return (8 -> 4 (-> 2) -> 8): build rejoin_departed (code design B, card + critic); ring R 0.5 m,
  0.9 kg; S's bars as designed; two 12-core lab slots allowed. C: both network forms; clock-free + reach headline;
  network-mode defaults approved (may be tuned). U as designed. R as designed. C2 after R if room.
- Agents may implement in up to 3 parallel git worktrees (apply today's uncommitted diff first; I integrate into the
  main tree between runs, never during a laptop run; no commits).

**Findings today (registry has the rows):**
- Rig pair swings at ~1.4 Hz on the old filter (6 pair lifts 21-22 Oct); slope halved and damped it (h2slope, f2slope:
  first full 2-quad figure-8); reviewer [weak] on the rig evidence. The twin reproduces it only with ~50-60 ms extra
  command delay: stage 1b (R2381-R2388) ema 6.7-7.1 deg vs slope 0.10-0.15 (bars hold; reviewer [weak]: 60 ms picked
  from the sweep). Under 60 ms, 3/4 quads on paths also swing with ema (tilt 16-28 deg) and stay calm with slope
  (R2389-R2396). The Smith predictor (TRACKER_DELAY_COMP_S) and the ring on slope do NOT fix it; cmd_next_tick from t=0
  flipped a creep (void). The rig's yaw lag (0.49 s) and spin path error (36 mm) match no delay: another cause, open.
- Twin 1 m baselines (R2329-R2364): the filter changes little in the undelayed twin; 3/4-quad 'not worse' passes.
- Spin yaw lag (17:24, read-only, scratchpad/yawlag/): rig ls3c 12 deg / 0.49 s against the twin 4.4 deg / 0.15-0.19 s at
  every command delay (R2401-R2404). The error builds in the 4 s spin-up (rig -9.2, twin -3.9 deg) and then holds
  through the 10 s at constant rate on both: the planner barely pulls a yaw error back. The ring follows the quads
  within 0.5 deg and the quads lead their references, so the lag is in the planner's plans (refs trail the desired
  0.61 s rig, 0.25 s twin). Ruled out: a yaw-rate measurement bias (both within 1-3 % over the constant phase),
  the trajectory clock, solve failures or late plans, parameter differences (attach_z, the 60 ms budget, landing
  switches only). Next (after wave 1): log the plan's yaw and cable azimuths per tick; suspects are the plan's
  slow correction through the pinned cable chain and the rig's real inertia/hinge against the model.
- The rig T hangs 2.6 deg (t3p_2): chain step 2a dropped. Part C failed on operations (OFF never reached; then the
  wrong quad): RViz RELEASE RIDER built (planner /fleet/carrier_map; two clicks; shown with the magnet rows).

**Work queue (in order; update the status as it moves):**
1. DONE 16:20: size-0.9 figure-8 bands (R2405-R2416, all clean): results/analysis/bands_fig8s09_2026-10-22.json, in the
   Wed sheet (lines at traj_radius:=0.9). n2_bands.py --set b22 still lacks the fig8s09 cells: add them after wave 1.
2. 1m-defaults package (design in push_1022_plan.md): fig8_size knob (rig default 0.9), cable_len 1.0, drone_mass
   0.583, the CoM defaults, the +-1.75 m LAND envelope (new), tip_replay's cable length (else gate 3 passes vacuously at
   1 m), the 1 m attach worlds + M1 layout, attach_cable_len check, reach-scaled constants, one 1 m twin take-back test,
   prebuild --drone-mass 0.583. Then V1-V5 twin smoke runs. Then the Wed sheet.
3. Thu 29 band: 3 twin runs of QUAD4 leaving the M1 layout at 1 m, as soon as the M1 1 m world exists.
4. Pair-swing package: loop_timing / pair_mode tools, per-channel rig-timing twin, the tip-hinge world (5.75 cm),
   rounds A-D under card 2026-10-23_pair_mode_timing (bars before runs). Betaflight DONE 17:11: QUAD3 read-only
   (docs/rig/betaflight_2026-10-22.md; Wesley: the others are the same; live radio check dropped, the module cannot
   plug into the laptop). Rate mode, linear 100 deg/s, feedforward on, ELRS 333 Hz (QUAD4 500 Hz): radio + FC
   ~10-15 ms. The rig is slower on the throttle (motor spin-up, no FC loop) and faster on the attitude than the
   twin, so round B delays the throttle only; the body-rate EMA is ours, not Betaflight. No FC change needed.
5. U (catch offset), R (re-fly at 1 m), C (network vs ours, critic for its law changes), S (+ rejoin_departed; the
   speed-ups) -- C, S and R's tools in worktrees; cards with frozen bars before each claim; reviewer per table.

**Running now:** wave 1 implementation workflow wf_d62e7db2-b9f (task w3zylhcnd): 1m-defaults in the MAIN tree (do not
  edit src/ or tools/ there until it ends; no lab batch either, it would ship a half-edited tree), sim-C, sim-S and
  rejoin-departed in worktrees (patches to scratchpad/wave1/<pkg>.patch), each followed by a verifier. Lab PC idle.

**Agent rules:** at most 2 agents running at once (Wesley 22 Oct, token control); registry append-only with LF via scratchpad/append_rows.py (backups registry.bakNN); never edit src
during a laptop batch (symlink install); laptop idle the first 3 min of a run; never pgrep "gz sim"; stop batches by
the script's own PID; run `date` before writing any time.

## Where things are (21 Oct)
- **Main tree** `~/multi_drone_control` on **day-1021** (cf556ff; from day-1016 at 410f0f1). day-1016 is NOT merged
  into week5 (4b3560e) yet: Wesley merges. Commits by explicit path, authored by Wesley, no trailers, never push.
- **Current plan:** `docs/plan_thesis_results_2026-10-21.md` (approved 21 Oct ~18:50; status table at the top).
  Decisions: `docs/decisions.md` DECIDED 2026-10-21.
- **Worktrees:**
  - `~/mdc_wt/tejen_sync2` (tejen-sync2 6483d90): Tejen's fd495db0 merged into our fork, avoidance on the transit and
    floor_join, `m1_irl_pickup.launch.py`. Not merged into day-1021; merge before the basket runs (plan s5).
  - `~/mdc_wt/tipdetect` (tip-detect b4356aa): the magnet-marker release check **rev 3** (critic round 2's guards:
    ring-glitch hold-off, the distance-from-centre test, 0.1 s on stamps, lone-marker would-fire, frozen markers,
    learning without the ring speed, slack-cable band, forget on attach/detach), the twin ring tilt fault
    (`MOCAP_TILT -1`), `tools/tip_replay.py`, `tools/mocap_glitch_scan.py`, the tracker's raw-pull columns, the rod-pose
    publishers in the existing worlds (Wesley ~23:30). Unit tests 16/16, planner suite 234. **Critic round 3 running**
    (background agent, its verdict goes into the card). Bars frozen 23:45 (card, rev 3 section).
  - `~/mdc_wt/thesis` (thesis-rig): the thesis.
- **Lab PC:** full all evening (Ryan's 11-12 Gazebo sims, load ~107-129 on 24 cores). Everything ran on the laptop.

## Now (Thu 22 Oct, 06:13): rig 08:00-11:00; overnight done (morning report `docs/reports/morning_report_2026-10-22.md`)
- **Rig tree:** `day-1021`, flight code as of 367c98f:
  - the magnet-marker check rev 3, a switch (`tip_detect_m`, default off; Part C's `chain_pair_tip.yaml`);
  - the 8 N pull cap;
  - the tracker's raw-pull columns.
  Gate passed at badb4a5. Note `git log --oneline -1` at the rig.
- **Sheet** `docs/rig/thu_2026-10-22.md`:
  - bench item 0 (the ring's centre of mass) first;
  - A then B (markers bagged);
  - Part C last. Status line: your call, R3on recommended if the A/B replay after Part B is clean (gate 3: run
    `tools/tip_replay.py` on each A/B bag). R3off will most likely end in "no detection: LAND" (the twin's stretch
    check missed 4 of 4 zero-share releases).
- **Overnight results** (registry R2259-R2322; plan status table updated):
  - the 1 m baselines;
  - the comparison (one frozen bar missed: the pair's rocking against an arm that never resized);
  - A/B with the check on, all clean;
  - the neighbours;
  - the basket: the mission works in hover at 0.55 m and 1 m, the landing hits TB14; the 1 m circle rejoin stalls;
  - the 1 m detaches: hard drops, soft does not;
  - the solve sweep re-timed: 4 / 5 / 6 quads at 5 / 2 / 1 iterations within 60 ms.
- **Videos:** `scratchpad/render22.sh` running until ~07:05. Reel `results/videos/reel_2026-10-22_overnight.mp4`.
- **The loop** (cron 73b7592d) stops after the report's push.
- **Rig so far (09:12):** bench 0 (pivot to 2.76 cm), markers, A1 h3n, A2 h3, A3 f3 (rows RIG-1022-*).
  - A3's corners (Wesley not happy): the cut is in the plan, and the rig lags ~0.13 s more than the twin. Today's
    size-0.5 figure-8 has corners of 10.4 cm radius (0.38 m/s^2 at 0.2); 7 Oct's r2008e60 flew traj_radius 1.0. Two
    things changed since 21 Oct f200 (corners +31 -> +44 mm): attach_z, and the integral, which never ran without the
    hold. Correction row RIG-1022-f3c.
  - f3big (7 Oct's size 1.0): cuts about as much as 7 Oct at matched bend demand, but the lap sat (-4.7, -1.3) cm
    off: every flight lifts 2-5 cm toward -x and no hold means no integral. QUAD2 ARM refused twice (FC !ERR*).
  - 09:20 Wesley: the 5 s hold back on every path flight, the figure-8 at size 1.0 (decisions.md); sheet updated
    (A3' f3h, A4 and B with the hold, configs/rig/two_quad_hold5.yaml new). After the rig: the figure-8-only size
    default, the twin B configs and the size-1.0 bands; find the -x push's source.
  - f3h (A3': size 1.0, 5 s hold): corners as good as 7 Oct (mean +13 mm, 7 Oct +9; f3big +35), lap centre
    (-0.8, -1.1) cm, tilt peak 5.0 deg, 3-D RMSE 4.0 cm; the hold learned fx -0.23 fy -0.15 N. QUAD2 ARM refused a
    3rd time today (FC !ERR*). Next: A4 ls3 with the hold.
  - ls3 (A4): clean; yaw 0.43 s behind the desired (max 11.5 deg), turned 225/229; figure ls3_yaw.png. Wesley: the
    spinny line becomes a constant-rate full turn, replacing the old law (decisions.md).
  - ~09:40 built at his request (load_trajectory.py + its test, uncommitted: he says when; diff in
    scratchpad/spin_constant_rate.patch); offline planner check clean; gate --quick running; then A4' ls3c on the rig
    (twin run waived for now, goes in tonight's batch).
  - ls3c flown (09:40): full turn 359/360, steady -12 deg (delay 0.49 s), no turnaround dip; the ~1.5 Hz yaw wobble
    grew (sd 1.1 vs 0.7 deg). The delay is the error; a yaw lead is a card + critic after the rig. Next: Part B.
  - Part B: h2 (QUAD3 magnet let go at the top), h2t9p (cap 0.9 + plates 11/5: grew, LAND), h2m (planned pull: slides
    along the line, LAND): all a 1.3-1.5 Hz swing, as 21 Oct's pairs (only c2q calm: drained packs 22.2 V). Mode: the ring
    drives the quads; their commanded positions trail their motion ~90-100 ms (45-55 deg), so they chase it late; the
    twin's pair is calm (R2244/R2255). 10:13 Wesley wants it fixed now: sheet "Part B, delay tests" P1 ref_time_shift
    (configs/rig/two_quad_rts.yaml), P2 slope velocity filter (T1 env), P3 both, P4 drained packs.
  - 10:50: P1 h2rts no help; P2 h2slope CALM (velocity lag 40 -> 20 ms, see-saw 8 -> 2 deg p-p). Repeat + pair figure-8
    lines on the sheet. Twin + reviewer before the slope filter becomes a default (it is a T1 env switch, off).
  - 10:55 f2slope: the FIRST full two-quad figure-8 on the rig (slope filter): 3-D RMSE 3.7 cm, tilt peak 14.3 deg
    (mean 7.3), no 1.4 Hz swing, path lag 0.10 s. OPEN line in decisions.md for the default.
  - 11:00-11:20 Part C (plates 11/2/5, R3on, slope on all three): t3p_2 the OFF never reached (T4 env); t3p_3 the OFF
    went to /drone_0 (on plate 11; the rider was /drone_1): 270 deg gap -> drop and land as designed. Rig day over.
    Rig day over.
- **Afternoon (11:30-):** rig-day review `docs/reports/rig_review_2026-10-22.md` (reviewer on the pair fix: [weak],
  corrections in RIG-1022-h2slopec/-h2t9pc); Wesley's answers in decisions.md (DECIDED 22 Oct review answers: Wed on
  slope if twin + reviewer agree, twin = rig filter, ring slope twin-first, 4 quads approved, RViz release button,
  Part C Wed after the pair A/B, overfull Wed sheet, step 2a dropped, bands for both filters). The plan is the table at
  the end of the review doc. 12:10 twin stage 1 running (scratchpad/vf_stage1.sh: b2_hover_vf_{ema,slope,none} x2,
  0.55 m; task b9831jf33). No src edits until it ends (symlink install).
  - 12:26 stage 1 done (R2323-R2328): the twin does NOT swing with the rig's filter (ema 0.03-0.05 deg, rig 2.6-3.7):
    the twin cannot test the filter; the rig decides (Wed A/B). tools/pair_swing.py reproduces the reviewer exactly.
  - Code (gate --quick 1514 pass; mpc_planner 180 pass; drone_visualisation built): run_experiment applies a config's
    lab_env (tools/experiment/config.py LAB_ENV); the twin's default filter = ema (payload_mocap_emulator); ring
    filter option (SIM_MOCAP_VEL_RING / MDC_MOCAP_VEL_RING, default ema); SIM_CMD_DELAY_FROM_S; the planner's
    latched /fleet/carrier_map; the RViz RELEASE RIDER button (two clicks, OFF at 10 Hz for 1 s, one per flight).
  - 12:37 batch running (task betkmx4t3, scratchpad/b_l100_vf.sh): the nine 1 m baselines x {ema, slope} x 2
    (configs b{2,3,4}_{hover,fig8big_h5_v200,linespinc_h5_v200}_L100_{ema,slope}; params configs/rig/drop_and_land_hold5.yaml).
    No src edits until it ends. Next: the Wed 28 sheet (docs), the band tool for these cells, step 2b card.
  - 13:01 split (Wesley: "go, use the lab PC"; idle, load 0.2/24, no containers): the laptop did R2329-R2335 (b2
    hover x4, fig8big x3); its script was stopped and b2_rest.sh (task bcmr3qspc) flies the last 5 b2 runs (R2360+);
    the lab PC (task b4dexzoci, lab_batch --slots 4, log scratchpad/lab_b34.log) flies the 24 b3/b4 runs R2336-R2359.
  - Done since: the Wed 28 sheet draft (docs/rig/wed_2026-10-28.md), tools/n2_bands.py --set b22 (hover window: top +2 s
    to LAND), configs/rig/drop_and_land_hold5.yaml.
  - 13:48 all 36 done (lab 45 min; R2344/R2350/R2359 'FAIL' = the runner's pose check ~2.1 s after /fleet/landed, at
    teardown: counted). Registry R2329-R2364. Twin: the filter changes nothing that matters (swing/tilt jitter at noise,
    0.02-0.05 deg; corners 3-4 mm less with slope; fig-8 lag +0.005 s (3q) / +0.01-0.02 s (4q) with slope; spin yaw lag
    0.14-0.22 s either way). Card's "not worse" (3 and 4 quads): passes every cell. Bands frozen:
    results/analysis/bands_b22_2026-10-22.json, in the Wed sheet. Reviewer on the twin table running (background).
    New tools: tools/yaw_track.py (reproduces the rig ls3/ls3c numbers), pair_swing.py, n2_bands --set b22.
  - 14:05 reviewer on the twin table: [weak] (pair_swing windows cut short; lag pass marginal; slope raises the spin's yaw
    wobble 24-47 %; the 3 lab FAILs = a runner stall after LANDED). pair_swing fixed, re-scored (verdict holds),
    correction rows R2329-R2364c, R2344c.
  - 14:15 Wesley: band path error clock-free, path-only window (decisions.md); n2_bands --set b22 uses it; bands
    results/analysis/bands_b22_clockfree_2026-10-22.json in the Wed sheet (+ expectations: the pair's tilt will MISS).
    RViz RELEASE RIDER shows with the magnet rows (rebuilt). Delayed-twin study on the lab PC (task bo7btjorw, R2365-).
  - 14:20 ULTRACODE PUSH (Wesley): scope + code authority in decisions.md (Wed readiness, 1 m + fig-8 defaults, pair
    swing cause, sims U/S/C/R; lab PC whenever free; report and delete what fails, report improvements; no commits).
    Design workflow wf_5bad4277-125 (task wptqrjn89): 6 designers + critic, read-only. Next: implement per the critique.
  - Delayed twin (SIM_CMD_DELAY_S after the climb; valid to LAND, landings fault with the delay on): R2365-R2372 (40/80 ms)
    and round 2 R2373-R2380. 60 ms reproduces the rig: ema swings to ~25 deg p-p (R2373), slope calm (R2374). The tracker's
    Smith predictor (TRACKER_DELAY_COMP_S) and the ring on slope do NOT fix it; cmd_next_tick from t=0 flipped a creep
    (void). Card stage 1b added (bars written 14:27, before its runs); R2381-R2388: testbed and confirm bars hold
    (ema 6.7-7.1 deg, slope 0.10-0.15); reviewer [weak] (60 ms in sample at 1 m; throttle-capped; the rig's mode and
    direction, not its flight; the delay came on 1.6-3.8 s before the top) -> correction row R2365-R2388c.
  - 3/4 quads under 60 ms (R2389-R2396, one each): ema swings on paths (3q fig-8 tilt 16 deg, 4q fig-8 28 deg, M1 5-12)
    while slope stays calm (tilt ~2 deg, M1 ~0.15); 60 ms looks harsher than the rig for 3/4 quads. Calibration of the
    delay against rig f3h / ls3c (3q, 0.55 m, ema, 0/30/45/60 ms) running on the lab PC (task b2w0iysqu).
- **Open for after the rig:**
  - the rig lag at tight bends (A3 corners +44 mm against the twin's +18);
  - chain step 2a (draft card) and 2b (the reaction to a loaded release);
  - the scalability quality card;
  - Tejen's 1 m moving-ring rejoin;
  - merge day-1021;
  - the 1 m bands and defaults;
  - the re-fly R.

## Earlier today (21 Oct)
- **Rig day 08:23-11:05** (`docs/reports/rig_2026-10-21.md`, RIG-1021-*):
  - QUAD2's magnet let go 6 times, and its link reads ~2 cm short (bench check on the sheet);
  - the no-hold offset of 5.5-6.6 cm was mostly the ring's mocap centre, 1.7 cm off. Wesley moved the ring pivot in
    xy (RIG-1021-ringpivot); expected next no-hold hover 0.5-2 cm (sheet A1);
  - ring body tilt 1.3 deg; the floor or Motive's ground plane 0.9 deg (spirit level on the sheet).
- **Te-Jen:** QUAD1 pickup on a 1 m cable with the new 0.15 kg object (RIG-1021-q1pick: picked up first try; the lift
  lagged with his kT feedback; LAND drops the object). Draft message to his Codex written in chat.
- RIG-1015-a1c: on 15 Oct the tether struck QUAD2.

## Earlier (16-18 Oct)
- Morning reports `docs/reports/morning_report_2026-10-1{6,7,8}.md`; decisions 16-17 Oct in decisions.md.
- The 0.55 m and 1 m bands are frozen (card 2026-10-16_bands_1m). Touchdown A is on in the twin; the rig waits on a
  critic look. Corner cutting is closed (lag reported).

## 14 Oct rig day in short (detail: registry RIG-1014-*, `git log -p` of this file)
- 46 launches: 8 passed, 11 failed in flight, 11 arm refusals (QUAD1/QUAD2 FC attitude drift), the rest void.
- t5_2 crashed (three-carrier lift bobbing with starved loops; drop_on_loss now on in attach). lt_13 F1 PASS after the
  load cuts and the lean rclpy switch.
- His pickup failed on contact time (t5_4, t5_5, t6); floor_join flown t6f..t6h5_3. t6h_4: the magnet caught plate 9
  and our old proof's taut check rejected it (fixed by WP3).
- Two operator DISARMs with QUAD4 ~0.3 m up on its own landing (fixed by WP8: it now disarms alone a few cm up).
- Notes for Tejen: docs/tejen/notes_2026-10-14.md; his logs zipped at ~/tejen_attach_logs_2026-10-14.zip.

## Rig sheets and twin evidence
- Tomorrow: `docs/rig/thu_2026-10-22.md`. Earlier sheets: `docs/rig/wed_2026-10-21.md`, `docs/rig/wed_2026-10-14.md`.
  Every terminal: the setup line on the sheet (main tree, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=23).

## Open for Wesley
- Merge day-1016/day-1021 into week5; merge tejen-sync2 before the basket runs.
- Tonight's report decisions: the chain's step-2 card (after the rig); the CoM fix (done on the rig before Part C).
- Older: A on the rig after its critic; the no-floor F3 bar; the orbit-join drops (morning report 18 Oct).

## Working constraints (beyond CLAUDE.md)
- **Helpers:** one general-purpose helper agent at a time (Wesley).
- **Lab:** at most 4 slots, evenings only (Wesley 21 Oct: use it at the same time as the laptop when free), one tree at a time. Check idle first (`who`, load, containers). Each batch
  runs from a script in a fresh `env -i` shell sourcing ONE install; start it detached; kill exact PIDs only.
- **Scorers:** `tools/m1_score.py claim|full|takeback|tilt|tb14 RUN --rev N [--check]` (week5). It prints one LF row:
  back up the registry, append the row yourself, then diff the old lines. Wrapper: `$CLAUDE_JOB_DIR/tmp/sc.sh`.
- **Permissions:** deletions (branches, worktrees), git push and switching the main tree are refused for the
  assistant. Write a script for Wesley.
- **Untracked local files:** `.claude/*` and `docs/claude-setup-proposal/`. Edit them in place; never `git add` them.
- **Timestamps:** read `date` before writing any clock time.
- **Branch rule:** delete a branch once its work is on week5 or the attempt failed. Archive-tag any unmerged SHA the
  registry cites.
