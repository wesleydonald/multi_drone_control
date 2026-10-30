# Fact-check: supervisor pack of 31 Oct 2026

Pack: `~/mdc_wt/thesis/docs/thesis_drafts/supervisor_pack_2026-10-31/` (PDF = `build/main.pdf`, 30 pp., clean build, 0
undefined references). Locations are `Source/<file>:<line>` in the pack. Checked 30 Oct ~13:30 against
`results/registry.csv`, `docs/decisions.md`, the cards, `configs/`, `src/`, the review-figure index and the run logs.
Read-only: nothing in the pack or the evidence was changed.

## Summary

1. **Counts:** 11 WRONG (4 of them trivial), 11 UNSUPPORTED (6 are the orange physical values), 7 STALE, 6 OVERSTATED. The rest of the numbers check out (listed as OK per section).
2. **Worst 1, the planner's iterations:** `controller.tex:11` says "one Gauss–Newton SQP step per tick". The planner runs up to 5 warm (15 cold) SQP-RTI iterations a tick (`planner_solver.py:35-36, 322-334`), and the results use "five iterations per tick" (`res_scale_8.tex:39`).
3. **Worst 2, the tick figure:** `fig:res_tick` plots the idle bench at 1 iteration with no 3.5x in-flight factor (`fig_s_tick.py:23-24`). Eight carriers sit at 83 ms, under the 100 ms line. The text cites it as showing that the ground computer cannot run eight (`res_scale_8.tex:39`).
4. **Worst 3, Quan et al.:** "the method's law as published lost the ring" (`contributions_results_plan.tex:34`). The runs that lost the ring were the paper's damping form on our rim world (R2744-R2749). The exact law, committed as "as published" (413ec0e), held the ring 110-118 mm off in hover and landed (R2805, R2812, R2813), and it carried the ring on the lines (R2808, R2814, R2815). Decision 30 Oct (4) uses the same words, so this is Wesley's call.
5. **Orange items only Wesley can settle:** the 0.583 kg UAV (weigh it); the old plates' ~60 mm and the new plate-face height (~25 mm) and centre-of-mass height (measure); both ring inertias (measure, or keep "model"); the magnet hold of 9–10 N and its dependence on angle, plus the new magnets' hold (spring scale); the catch radius, 24 or 35 mm (OPEN since 30 Oct 02:15); the velocity-filter bracket (OPEN: the final-tree filter); the join's "[real world]" and "[paused path]" scope; the catch-offset result after its re-fly; the appendix pointer.

Verdicts: OK, WRONG, UNSUPPORTED (no source, or only an estimate), STALE (earlier setup or superseded value given as
current), OVERSTATED (claim larger than the evidence), INCONSISTENT (two places in the pack disagree).

---

## Cover note (`cover_note.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| :5 | "the methodology chapter brought up to date with the current hardware" | `fig:geometry` is still the earlier setup. `tab:constants` gives the 1.6 N share for the earlier ring and the old plates' 24 mm catch radius. The 9–10 N hold and the latency figures are the earlier setup's. | OVERSTATED (say "largely") |
| :10 | methodology "about 21 pages", school guide 10–15 | PDF pp. 4–24 = 21 pp. `thesis_audit_2026-10-30.md` gives the 10–15 guide. | OK |
| :12 | real-world plan 4–12 Nov: first join, 1 m / 1.15 kg baselines, 4→3→4 in hover | `plan_rescope_2026-10-30.md` §4 (S1–S4, freeze Thu 12 Nov 11:00) | OK |
| :5 | "Section numbers follow the thesis" | The contributions print as Section 1, but the thesis has no contributions section yet (audit §7 Q4). | minor |

## 1 Contributions and the results plan (`contributions_results_plan.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| :34 | "the method's law as published lost the ring" | R2744-R2749 = "the paper's form (Eq. 9 damping, Eq. 10 acceleration)", rim world, LOSS 6/6. The exact law (`net_law quan`, commit 413ec0e "Quan et al.'s law as published", world C): hover 110.0 / 118.3 / 109.1 mm, LANDED (R2805, R2812, R2813), lines 201.7 / 244.9 mm, LANDED (R2814, R2815). Decision 30 Oct (4) says "it loses the ring, R2744-R2749". | OVERSTATED. Name the damping form, or say that the exact law holds the ring ~0.11 m off. |
| :34 | "the planner runs in real time for three or four carriers" | Bench B projects 3 / 4 / 4 carriers at 5 / 2 / 1 iterations, and only with `plan_fast_refs`, which is off in the real world (card 2026-10-23_scalability, Results B). Four carriers flew at 5 iterations only under the 60 ms budget cut (RIG-1007-r0012, RIG-1008-r305). With three carriers the tick already runs at about its period (p95 116 ms, card §4). | OVERSTATED (mild). Say "projected 3 at 5 iterations; 4 flown with the 60 ms cut". |
| :38 | "In the real world one catch on the plate was refused by the confirmation" | RIG-1014-t6h_4 was refused by the 10 Oct `contact_lift` rules (taut check, `proof_taut_len` 0.572 m). `common.yaml:228` says so. Section 3.4.1 describes the co-motion confirmation that replaced those rules. | OVERSTATED / misleading. Say "by the earlier confirmation rule". |
| :36 | "the release chain from four carriers to two and back … whose unannounced step from the square to the T marks the boundary of the drop rule" | Decision 30 Oct (c): the unannounced square-to-T step drops the ring. R2254 was dropped at 45° 0.5 s after the release. R2249 peaked at 57°, R2823 at 61°. As written, the chain is promised "to two and back", but its first step is expected to drop the ring. | OVERSTATED / unclear |
| :34 | "flew the smaller team within 0.4 s of a detected release" | n−1 came 0.27–0.38 s after the RELEASE (magnet off), detection included (R2708/R2709 0.29/0.28 s; R2766/R2767 0.27–0.38 s) | WRONG wording (it is "of the release"). The number is OK. |
| :42 | later setup "with three and four" carriers, planned | RIG-1029-f4 already flew a 4-carrier figure-eight (with the 2 Hz low-pass, not the notch) | STALE (minor) |
| :42 | earlier setup: two, three and four carriers on hover, figure-eight and spinny line "as flown" | No four-carrier spinny line on the earlier setup. The registry has only RIG-1022-ls3/ls3c and RIG-1028-ls2*. The 4-carrier figure-eight is RIG-1007-r2008e60 (a = 1.0 m, 0.25 m/s). | OK with "as flown". Say which cells are missing. |
| :40 | "an errand before each rejoin" | R2777: a waypoint task (corner at 1.93 m, 15 s dwell), no object | OK. "a task" is more accurate. |
| :2 (comment) | "contributions C1-C4 still OPEN in decisions 30 Oct" | Decision 30 Oct ~13:10: "I approve everything else: contributions C1-C4" | STALE (comment only) |
| :36 | earlier setup: 3 unannounced releases, detected 0.33–0.42 s, tilt 6.5–17.8°, no drop | RIG-1008-r303_4 0.42 s / 6.5°, -r304_3 0.33 / 17.8°, -r305 0.34 / 8.1°, all "landed + disarmed" | OK |
| :36 | later setup: a magnet let go 4 times on the same plate; detected; drop rule put the ring down and **landed the fleet** (orange) | h4 "every drone landed"; ch1b "landed"; ch1b_2 "landed"; ch1 has no row text, but `results/rig/2026-10-29/ch1.log:310-311` reads "after the drop every drone is down … Landed - disarming the fleet" | OK (orange resolved) |
| :36 | at a 150° gap the measured pull cut the rise to about two thirds of the planned | G150 17.80 / 17.03 (arm mean 17.88) against G150p 29.81 / 27.31 (mean 28.36) = 0.63 (R2605/R2611, R2606/R2612) | OK |
| :38 | departed carriers rejoined, "all eight times" in the 8-carrier chain; the collaborator's newcomer confirmed and folded in in simulation | R2766/R2767: 4 rejoins each, all welded. R2843/R2844, R2862, R2865, R2906, R2908 proven and welded. | OK |
| :38 | no join has completed in the real world | RIG-1029-sq5_2, -k1_4 FAIL; no weld through 30 Oct | OK |
| :40 | 8→4→8 in simulation; harder cases once each, no drop or abort; chain on a moving path walked ~1 m | R2766/R2767; R2768-R2779 "no falsifier tripped"; R2770 (card: the held return ratchets the datum ~1 m; ring to \|y\| 1.41 m, UAVs to 2.41 m) | OK |
| :40 | endurance relay, ~30 min, 5-min pack budget; the ablation arms; Quan claim with the port compared and the published law as one row | Decisions 30 Oct ~12:30 (1), (4), ~12:35 (b); X1 card §3 | OK |

## 3.1 Platform and simulation (`methodology/system_overview.tex`, `fig_geometry.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| system_overview:146 | "The simulation runs at 0.3–0.45 of real time" | That was true for R1600-R2399. Since R2400 (1 m, lab PC) the RTF is 0.14–0.28, median 0.22. The cited runs: R2700/R2701 0.22/0.27, R2708/R2709 0.24/0.27, R2764-R2767 0.217–0.225. | STALE. It also contradicts res_scale_8:39 (0.22). |
| system_overview:34 | bending: "opposite plates in phase and neighbouring plates in anti-phase" | Decision 29 Oct (RIG-1029-h4) is about the four carriers 90° apart. On a 12-plate ring, neighbouring plates (30°) move nearly in phase in a saddle mode. | WRONG. Say "carriers 90° apart in anti-phase". |
| system_overview:148 | sim throttle-to-lift "**about 15–25 ms**" (orange); real 44–67 ms; body rate 25–35 against 48–62 ms | The measured sim value is 17–29 ms (R2381/R2385; `docs/experiments/2026-10-23_pair_mode_timing.md:22`). 15–25 is the Betaflight note's design estimate (`docs/rig/betaflight_2026-10-22.md:51`). The real-world numbers are from 0.55 m flights (h3, f3h, h2slope). | STALE. Use 17–29 ms (orange resolved). |
| system_overview:40 | magnets "release under a jerk rather than under a steady load" | That is Wesley's 7 Oct note for the earlier magnets (RIG-1007-r200006). On the later setup plate 0 let go at 2.8 N (h4), ~5.2 N (ch1b) and ~8 N (ch1b_2) with no jerk recorded. Contributions:36 reports these releases. | UNSUPPORTED for the later setup, and INCONSISTENT with C2 |
| system_overview:40 | earlier magnets held "**about 9–10 N**" (orange) | This is Wesley's statement (r200006 row; `2026-10-07_cable_mocap.md:190`). The 30 Sep card says "rated 10-12 N" (`2026-09-30_pretension.md:10`). The spring-scale test of 14 Oct (`docs/rig/wed_2026-10-14.md:118`) has no registry row. | UNSUPPORTED. Open for Wesley. |
| system_overview:16 | "every flight streams only the bodies it uses, at most eight" | OPEN 30 Oct: the 4-UAV T1 streams 9 bodies (8, 11-14, 21-24), 2,160 packets/s | UNSUPPORTED (open) |
| system_overview:16 | "the flights of that day are not reported" | The pack's own comment says the coordinator decided this, to confirm with Wesley. There is no decision line. | UNSUPPORTED (Wesley's call) |
| system_overview:59 | "Plate faces above the centre of mass: 17.5 mm / **about 25 mm**" | Later: `real.yaml:17` attach_z 0.025 is above the Motive **pivot** (markers +2.78 cm less a 2.25 mm offset, an estimate; RIG-1029-bench). The pivot is at 1.17–1.31 cm and the centre of mass at "~1 cm" (Wesley, RIG-1028-payload), so faces are ~2.5–2.8 cm above it. Earlier: the centre-of-mass height of 2.75 cm was also Wesley's estimate (RIG-1022-ringpivot), not a measurement. | UNSUPPORTED (estimate; the row label mixes pivot and centre of mass). Open for Wesley. |
| system_overview:54 | UAV with cable and magnet "**0.583 kg**" (orange) | `real.yaml:10`: the 0.55 m cable's 0.040 kg scaled to ~0.073 at 1 m (Wesley 22 Oct). Not weighed. | UNSUPPORTED. Open: weigh it. |
| system_overview:58 | earlier plate "**about 60 mm**" (orange) | Only source: decision 30 Oct 02:15 (2), "0.024 is the old 60 mm plates'" | UNSUPPORTED. Open: measure. |
| system_overview:60 | inertia "**0.027, 0.055**" / "**0.025, 0.049**" (orange) | `configs/rig/ring_m2a.yaml` 0.02733 / 0.05452; `real.yaml:13-15` 0.0246 / 0.049 (a flat annulus, not measured) | OK as model values (the row says "model"). Open only if Wesley wants them measured. |
| system_overview:27 | "**twelve** plates" (orange) | The M2A ring: 12 plates every 30° (memory payload-disc-spec, Tejen's fixture). Later ring: `Images/platform_closeup.jpg` shows 12 metal segments, and the flights use plates 0/3/6/7/9/11. Nothing written down. | UNSUPPORTED (likely OK). Wesley to confirm. |
| system_overview:34 | pivot "about 1 cm above the floor, near its centre of mass" | rest z 1.30–1.31 cm (RIG-1028-payload), 1.17 cm (RIG-1029-bench). Centre of mass "~1 cm": Wesley's estimate. | "near its centre of mass" UNSUPPORTED (estimate) |
| system_overview:14 | commands "over its own ExpressLRS radio link at 100 Hz" | The tracker computes at 50 Hz, `elrs_interface` re-sends every 10 ms, and the ELRS frames reach the FC every 4.0 ms (250 Hz; RIG-1030-h4n_bb) | minor. Say "sent at 100 Hz". |
| system_overview:92 | photo shows "the newcomer's magnet tip" | The photos are the 28 Oct square (four carriers on 0/3/6/9). The markers are the four carriers' magnet markers. There is no newcomer. | minor (wording) |
| system_overview:56 / fig_geometry:13 | earlier ring 0.51 m | The figure draws an outer radius of 0.25 (0.50 m) and its header says "500 mm" | minor INCONSISTENT |
| system_overview:16 | 240 Hz, forwarder ~2,250 packets/s, 95 ms with ten bodies against 7.5 ms with one | RIG-1030-bench_mocap; decision 30 Oct ~11:40 | OK |
| system_overview:27, table | 0.86 kg / 0.51 m / r 0.225 / 0.55 m; 1.15 kg / 510 mm / slots ~100×85 / r 0.204 / 1 m metal cables; flown until 28 Oct / from 29 Oct | decisions 28 Oct ~19:00; RIG-1028-payload (radii 0.201–0.209, mean 0.204); registry (RIG-1028-ls2c last 0.55 m; RIG-1029-h4 first 1 m) | OK |
| table :61-66 | 5.3 Hz; band-stop 5.3 Hz Q 1 (later), off (earlier); cap 12 then 14.5; ema then slope; position limit last flights / on; margins 112 / 102 mm | `real.yaml:67-69`; decisions 21 Oct (3), 28 Oct, 22 Oct; r cos 60° = 112.5 / 102.0 mm | OK (112.5 is rounded down to 112) |
| :83 | plates 3/7/11 + 9 for the join; fixed teams opposite pair / even triangle / 0,3,6,9; 60° elevation; δ 4 cm | CLAUDE.md; `common.yaml` cable_elev_deg 60; RIG-1029-h4 typed 0,90,180,270 | OK |
| :95 | ±1.75 m limit lands the fleet, never disarms; on in every later flight; off in simulation | `real.yaml:45`; `common.yaml` pos_limit_m ''; decision 22 Oct | OK |
| :107 | planner 10 Hz, trackers 50 Hz, mux latches | code; fig:owners | OK |
| :146 | rigid links with ball joints; floor start; emulator 120 Hz; per-UAV offset; sagging pack | CLAUDE.md; `common.yaml` mocap_hz 120; s8 config sim_thrust_offset / sim_pack_v0 | OK |

## 3.2 Model and planner (`methodology/model.tex`, `controller.tex:8-35`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| controller:11 | "real-time iteration … one Gauss–Newton SQP step per tick" (both levels) | Planner: `STEADY_ITERS = 5`, `COLD_ITERS = 15`, loop at `planner_solver.py:322-334`. The launch logs print "n=…: 5 SQP iterations per tick" (R2766 notes), and the rig kept 5 (decision 24 Oct (1)). Tracker: one solve per tick (`tracker_node.py:1963`). The 60 ms "starts no new iteration" in the same paragraph only makes sense with several iterations. | WRONG for the planner. INCONSISTENT with res_scale_8:39 and sim_studies:43. |
| controller:11 | N_h = 20 over 2 s; HPIPM partial condensing (planner), full (tracker); 60 ms of 100 ms | `planner_ocp.py:61,192-193`; `tracker/acados.py:57-58,117-118`; `real.yaml:43` solve_budget_s 0.06 | OK |
| controller:25 | C_0 pins p_L, v_L, q_L, ω_L, q_i; tensions free at node 0 | `planner_solver.py:179-181` (observed_state_indices) | OK |
| model.tex | state 13 + 14N; δ = −0.04 e3; thrust equation | p, v, q, ω = 13; per cable 3+3+3+3+1+1 = 14; signs checked | OK |
| controller:27-29 | references "by flatness" | no citation | supervisor question (cite) |

## 3.3 Tracker, thrust map, measured pull, model integral (`controller.tex:37-109`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| controller:62 | "**[The simulation results flew the estimate of their time: frame differences before 22 Oct, the exponential average to 28 Oct and the slope since.]**" (orange) | `payload_mocap_emulator.py:56-59` ('none' = plain difference before 22 Oct, 'ema' 22–28 Oct, 'slope' since); R2764-R2767 typed ema | OK as a fact (orange resolved). Keeping it depends on the OPEN final-tree filter. |
| controller:62 | "The ring, the newcomer and its magnet tip keep the exponential average, since they also feed the collaborator's controller" | The newcomer's reason is recorded (CLAUDE.md, attach mode). The ring's ema default comes from card 2026-10-22_velocity_filter (ring arm), with no reason given. | UNSUPPORTED for the ring |
| controller:62 | slope through the last four positions, ~20 ms against ~40 ms; the pair's swing cut by half or more; default since the last earlier-setup flights | `velocity_filter.py:17` (frames 4); decision 28 Oct (2.69 vs 1.06°, 3.98 vs 0.74°; 40 vs 20 ms), 22 Oct (1.35–1.46 vs 2.60–3.72°) | OK (one flight per arm: say so if asked) |
| controller:52 | hung masses 0, 98, 196 g over 22.7–24.6 V | `docs/archive/rig_2026-09-30_review.md:37` | OK |
| controller:54 | slope over 80 ms, 50 ms low-pass, thrust through the same filter, vertical part downward, cap 14.5 m/s² ≈ 8 N, slew 25 m/s²/s; cap 12 before 22 Oct | `tracker_node.py:1656-1712`, `CABLE_SLEW` :105; `common.yaml` cable_accel_cap 14.5; 14.5×0.55 = 8.0 N (8.5 at 0.583) | OK |
| controller:71 | band-stop ahead of the cap, −3 dB band 3.4–8.1 Hz, ~30 ms at 1.1 Hz; two-stage 2 Hz low-pass lag ~120 ms, 1.1 Hz slide | Recomputed from `notch_coeffs` at 50 Hz: 3.35–8.12 Hz, 29.7 ms. Low-pass: 120 ms measured (RIG-1029-sq2_2), 141 ms computed. RIG-1029-ch1b, -sq2_4. | OK |
| controller:88 | throttle ≥ 0.05, throttle rate 0.5 s⁻¹, sticks ±1, node 0 = measurement | `tracker/acados.py:134-141` (by state order the 3rd input is the throttle rate; the code comment mislabels it); `x0_relax_frac` 0 | OK |
| controller:101-109 | gate: < 0.05 m/s, miss < 0.25 m; 1/k_i = 2.5 s; b̄ 0.3 m, K b̄ 2.1 N, 0.1 kg; warning at 70 % | `planner_node.py:805,135`; `common.yaml` z_ki 0.4, z_i_max 0.3, int_k_z 7.13, z_i_gate 0.25 | OK |
| controller:101 | "K … is set to the cascade's measured closed-loop stiffness" | The values (5.26 / 7.13 N/m, card 2026-10-04_z_int_model) are not in the text | supervisor question |

## 3.4 Reconfiguration (`methodology/reconfiguration.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| :115, sim_studies:27 | catch radius "**24 mm**" (orange) | `weld_radius` is typed per sheet (`graph_attach.py:234`). 29 Oct: K 0.024 (RIG-1029-bench). 30 Oct sheet: K=0.035 (`docs/rig/fri_2026-10-30.md:10`, flown in RIG-1030-sq6b_2). OPEN decision 30 Oct 02:15 (2): "0.024 is the old 60 mm plates'". | STALE / OPEN. Wesley to decide. |
| :117 | confirmation "tip within 6 mm" | The code default is 0.006 (`comotion_proof.py:45`). The 30 Oct rig lines type G = `comotion_d_ok` 0.008 (OPEN 30 Oct (1); R2906). | STALE / OPEN (same decision) |
| :121 | seed against a balanced share of "**1.6 N** on plate 9 of the earlier ring" (orange) | Recomputed: the minimum-norm vertical shares on 3/7/9/11 give plate 9 W/6 = 1.41 N, and 1.41 / sin 60° = 1.62 N tension. For the 1.15 kg ring it is 2.17 N. | OK for the earlier ring (orange resolved). STALE in a table captioned "real-world values". |
| :86 | "On both rings the plates sit about **2 cm** above the centre of mass" (orange) | 17.5 mm (earlier, estimate) and ~25 mm above the pivot, ~2.5–2.8 cm above the centre of mass (later, estimates); the table gives 17.5 / 25 | UNSUPPORTED / INCONSISTENT with tab:setups. Say "2 to 3 cm (estimates)". Recomputing for radial cables at one elevation shows the shares stay exact, so "within a few per cent" is safe. |
| :116 | "still for 36 motion-capture samples" in 0.3 s | That is 120 Hz, the publisher's cap (`MOCAP_MAX_PUBLISH_HZ` 120, `motion_capture_publisher_node.py:50`). 3.1.1 says 240 Hz. | INCONSISTENT. State the 120 Hz cap in 3.1.1. |
| :48 (figure) | exit "Drop and land: second loss, or tilt > 30°" | The text (:66) gives fewer than three, a largest gap ≥ 180°, or tilt > 30° in two samples. A first loss of plate 7 or 11 already drops. | WRONG (figure). Say "gap ≥ 180°, or tilt > 30°". |
| :49 (figure) | exit "Tilt cut: newcomer's motors off" | `partner_tilt_cut` is false in common.yaml and not set in real.yaml | minor. Say it is behind a switch. |
| :59, :61, :66 | 0.06 m for two ticks; 1.0 s after the lift; 15 vs 10 mm in simulation; solvers primed twice; refusal below three; drop rules; relaxed 200° / two / 45° | `common.yaml` detach_detect_m 0.06, min_survivors 3, drop defaults; `dissipative_node.py:1669,3341` (two primes); `configs/twin/s_chain_pair.yaml`; decision 21 Oct (4); `graph_attach.py` comotion_normal_ok 0.015 | OK |
| :66 | relaxed rule at 200° | Comment: "Why 200 and not 180: NOT RECORDED". At 200° the centre of mass is 35–39 mm outside the polygon, which contradicts eq:margin's own argument. | supervisor question (give the reason) |
| :93 | margins 112/86/58/29/0 and 102/78/53/27/0 mm; four carriers cannot tolerate any loss; five can; 3/7/9/11 admits only 9 | Recomputed: r cos(g/2). Gaps 120/60/60/120: losing 7 or 11 gives 180°, losing 3 gives 240° (−112 mm). | OK ("as far outside" is unclear: say "112 mm outside") |
| :113-124 | face −15..+12 mm, 0.03 m/s, 0.3 s; 10 mm; plate range 5 / rise 3 mm; taut 3 mm; 45 mm for 0.12 s; gap 0.1 s; take-back 0.25 m/s² → 1.46°, 0.34 m/s in 1.36 s | `comotion_proof.py:33-61`; `dissipative_node.py:147` | OK |
| :102 | the collaborator does every real-world attach down to contact; our approach is simulation only | CLAUDE.md locked decisions (10 Oct) | OK |

## 3.5 Experimental method (`Methodology.tex`, `simulation.tex`, `test_cases.tex`, `metrics.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| test_cases:23 | "the earlier simulation tables fly the figure-eight at a = 0.5 m and a 1 m line at 0.125, 0.2 and 0.3 m/s" | The catch offset flies a = 1.0 (`u_meas_t08.yaml` fig8_size 1.0), as do the 4-UAV S figure-eight (`s4_fig8big_*` 1.0) and the bench. The Quan comparison flies a 0.9 m line at 0.2 and 0.4 m/s (`c_pl_linex_v200_L100.yaml`, `c_net_*_v400`). | WRONG / incomplete |
| test_cases:36, :38 | real-world pair on plates "0, 6"; triangle "3, 7, 11" | The decided real-world pair rows are 28 Oct on plates 11/5 (decision 30 Oct ~13:10; RIG-1028-f2a…ls2c). The earlier setup's triangle flew 0/4/8 (RIG-1021/1022). | WRONG for the reported real-world rows |
| test_cases:36 | "real world **[ring]**" (orange) | 0.86 kg M2A ring on 0.55 m cables (decision 30 Oct ~13:10) | resolved |
| test_cases:23 | real-world paths a = 0.75, d = 0.75, r = 0.5; fixed teams at 0.2 m/s, orbit at 0.125 m/s | True for the later setup and the 28 Oct pair (`real.yaml:29-31`; sheets type traj_speed 0.2). The earlier 3- and 4-carrier flights that are reported "as flown" used a = 1.0 at 0.2 m/s (RIG-1022-f3h) and a = 1.0 at 0.25 m/s (RIG-1007-r2008e60), and the pair flew a 1.5 m line (RIG-1028-ls2b_4, -ls2c). | INCONSISTENT with contributions:42. Add the earlier sizes. |
| test_cases:19 | lift "**[at 0.15 m/s]**" (orange) | Every flight line types `lift_ramp_vel:=0.15` (RIG-1029-h4, RIG-1030-h3t, the s8/s4 configs). The `real.yaml:22` default is 0.1. | OK as flown (orange resolved) |
| test_cases:46 | chain "simulation **[scope]**" (orange) | X2 is a simulation P1 claim; the real-world 4→3→2 chain is on the cut list (rescope §4, approved 13:10) | resolved: simulation |
| test_cases:44 | join "**[real world]**", "**[and on a paused path]**" (orange) | Not flown; planned S1 (first join), Nr on the figure-8 in S4, X7 "attach paused vs moving" P2 | open (scope, Wesley) |
| simulation:10 | "its simulation pair, flown beforehand on the same code tree, is its prediction" | No frozen simulation band exists for any 1 m / 1.15 kg flight. Every 29–30 Oct row is exploratory (audit §5 G5). | OVERSTATED. Say "where one was frozen". |
| test_cases:21 | spinny line ~36°/s at d 0.75, 0.2 m/s; smootherstep 1.875 d/v; circle T | Recomputed: 360 / (2·7.03 − 4) = 35.8°/s; the same formula gives the 24.4°/s of RIG-1022-ls3c | OK |
| metrics:24 | on-schedule ring pays v·0.1/√3 = 17 mm at 0.3 m/s | 17.3 mm | OK |
| Methodology:22 | parameters "in an appendix **[not in this draft]**" | The appendices do not compile (audit §1) | open (Wesley) |

## 3.6 Studies in simulation only (`methodology/sim_studies.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| :21 | measured catch point "as built and as flown on the hardware" | No real-world fold-in has flown (no weld; contributions:38 says so) | WRONG. Say "as built for the hardware". |
| :43 | in flight the ticks took "about 3.4 times as long as the same ticks replayed on the idle computer" | Per-tick against the replay: p95 3.65 (h3) and 3.59 (f3h). Against the bench: 3.37 / 3.45 (pooled 3.43). Scalability card §4 table. | WRONG (minor). Say "as the idle bench". |
| :41 | Sun et al. ran nine UAVs at 10 Hz with one iteration "**[check against their Fig. 7B]**" (orange) | `docs/papers/sun2025_*.pdf` p. 17: "the planner supported nine units at 10 Hz" (Fig. 7B, **simulation**; their real world flew 3, then 4). The solver is SQP in an RTI scheme (Materials and Methods), which means one iteration. | OK. Add "in simulation" (orange resolved). |
| :34 | staging point "kept inside the position limit" | 3.1.1 says the limit is off in simulation. The staging clamp uses 1.75 m when no limit is set (`dissipative_node.py:91`, card rejoin). In R2770 the UAVs were airborne out to 2.41 m. | INCONSISTENT wording |
| :23-25, :36-38 | 0.86 kg ring on 1 m cables; plates 3/7/11; 25 mm, 0.05 m/s, 0.15 s; d_c lists, 11.6 cm = plate 10; 5° and 0.30 m bars; staging 0.3 m, 0.2/0.1 m/s, 1.5 cm for 0.3 s, 2 cm/s, 45 s, 10 s blend, 1.5 s ramp | `u_meas_t08.yaml`; card 2026-10-23_catch_offset §3-5; `dissipative_node.py:88-99`; card rejoin_departed | OK |
| :43-45 | bench n 2–10, 5/2/1 iterations, 200 ticks; ×3.5; ring R 0.5 m, 0.9 kg, 45°, five iterations; 3 s lap-end hold; about a quarter of real time | `s_bench_B_2026-10-24.json` args; card §5; `s8_*` configs; R2778 traj_lap_hold_s 3 | OK |
| :23 | the studies fly the 0.86 kg ring while the hardware moved to 1.15 kg | decision 30 Oct (d): 0.86 kg plus one 1.15 kg cell per table | OK, but the text gives no reason (supervisor question) |

## 4.3 Scalability (`results/results.tex`, `results/res_scale_8.tex`)

| Where | Claim | Source says | Verdict |
|---|---|---|---|
| res_scale_8:39 + results:19 | eight in real time is something "the ground computer cannot do for eight (Figure fig:res_tick)" | `fig_s_tick.py:23-24` plots one iteration count, unlabelled, with no rig projection (Wesley 27 Oct): the idle tick at **1 iteration** (n = 8 83.1 ms, n = 9 106.4 ms). As drawn, eight carriers sit under the 100 ms line. The claim runs fly 5 iterations (idle 256 ms at n = 8), and the real-time limit needs ×3.5 (sim_studies:43). The caption ("The planner's tick against the number of carriers") names neither. | WRONG (the figure contradicts the text). Plot 5 iterations or the ×3.5 line, and say "idle, 1 iteration" in the caption. |
| res_scale_8:83 | "The ring never tilted more than 3.3°" | 3.28 / 3.31° (review index) | WRONG (trivial). Say "about 3.3°". |
| res_scale_8:97 | fold-in moved the ring "3.7 cm when it grew to eight" | 36.5 / 36.3 mm (mean 36.4) | WRONG (trivial). Say 3.6 cm. |
| res_scale_8:83 | eight held the ring "at about 1.3 mm rms" before the first release | 1.31 / 0.93 (chain), 1.32 / 1.26 (hover) | minor: "0.9 to 1.3 mm" |
| table :50-75 | RTF, late ticks, N1 1.32/1.26 & 1.31/0.93 mm, 0.06°; N2 0.38/0.37 s, 2.60/2.90°, 1.21/1.20°; N3 17.7/19.4 s, 1.42/1.44°, 1.27/1.26°; N4 6.36/6.35 mm (bold), 0.99/0.95°; original window 18.9/19.0, 1.56/1.58; bars 5.66, 1.45, 8.83, 2.0, 3, 20, 10 | R2764-R2767 value and verdict columns; bars from R2700/R2701 (2×1.33 + 3; 2×0.225 + 1) and R2708/R2709 (7.825 + 1) | OK |
| res_scale_8:39 | hover at 1.0 m, five iterations, releases 15 s apart, first rejoin 20 s after the last release, each next 20 s after the weld, flown twice, RTF 0.22 | `s8_chain4back_R50_L100_it5.yaml` events (RELEASE 30/45/60/75, ATTACH 95, WAIT_LOG +20); registry | OK |
| res_scale_8:97 | release offsets ~6 cm (8, 7 carrying) / 8 cm (6, 5), peak ~1.1 s, back within 5 mm 5.2–5.7 s; fold-in 4.6 cm; back ~4 s after the blend; runs within 3 mm | review index (62.0/62.0, 59.9/59.6, 80.5/83.1, 80.9/82.6; 45.5/45.8; 14.0–14.7 s after weld, blend 10.1–10.2 s); R2766 notes (1.15 s) | OK |
| res_scale_8:108 | N4 missed by 0.7 mm; 36 mm at the blend's end, 28 mm at the window; steady return to ~2.5 mm with no overshoot | R2766c; same-tree bar 5.69 (R2788/R2789) gives 0.66 mm: still "0.7" | OK |
| res_scale_8:115, results:31 | lap-by-lap chain once, exploratory, no drop or abort, rejoins took longer | R2779 (one bigger 20.09–24.58 s against 16.17–19.40 in hover) | OK |
| res_scale_8:39 | "The bars were set before the runs" | Bars frozen 23 Oct 07:25, revised at 14:32–15:12, before the claim runs (batch 18, ~22:40) | OK (the revision is stated in the table) |
| contributions:32 vs this section | "every simulation table is flown again on the final code before it is reviewed" | This table was reviewed on 24 Oct (strong) on 63b504f and flew ema | minor INCONSISTENT. Say "re-flown before the thesis". |
| res_scale_8:39 | the 8-UAV chain flew the relaxed drop rule (`s_chain_pair.yaml`: 200°, two, 45°) | not stated; the peaks stayed far from it | supervisor question (one clause) |

## Orange items: resolution

| Where | Orange text | Resolution |
|---|---|---|
| contributions:36 | landed the fleet | **OK**: ch1.log:310-311, plus the h4 / ch1b / ch1b_2 rows |
| contributions:38 | [result after its re-fly] | open: U's verdict is "between" (R2629-R2732); re-fly on the final tree |
| system_overview:27 | twelve | likely OK (photo shows 12 segments); **Wesley confirms** |
| system_overview:34 | [both to be measured] (inertia, face height) | **Wesley** |
| system_overview:40 | about 9–10 N; [hold against angle; later magnets] | **Wesley** (spring-scale test never logged; 30 Sep note says rated 10–12 N) |
| system_overview:54 | 0.583 kg | **Wesley**: weigh |
| system_overview:58 | about 60 mm | **Wesley**: measure |
| system_overview:59 | about 25 mm | **Wesley**: measure the marker-to-face offset and the centre-of-mass height; fix the row label (pivot vs centre of mass) |
| system_overview:60 | inertias | model values verified against the configs; measuring them is **Wesley's** choice |
| system_overview:148 | about 15–25 ms | **replace with 17–29 ms** (pair_mode_timing card :22) |
| controller:62 | sim velocity filter by date | **OK** factually; wording waits on the OPEN final-tree filter (**Wesley**) |
| reconfiguration:86 | 2 cm | change to "2 to 3 cm (estimates)" |
| reconfiguration:115, sim_studies:27 | 24 mm | **Wesley**: OPEN K decision (0.035 flown 30 Oct) |
| reconfiguration:121 | 1.6 N | **OK** (recomputed 1.62 N); add "2.2 N on the later ring" or label the row |
| sim_studies:41 | [check Fig. 7B] | **OK**; add "in simulation" |
| test_cases:19 | [at 0.15 m/s] | **OK** as flown |
| test_cases:36 | [ring] | 0.86 kg on 0.55 m, plates 11/5 |
| test_cases:44 | [and on a paused path], [real world] | **Wesley** (scope) |
| test_cases:46 | [scope] | simulation (rescope approved) |
| Methodology:22 | [not in this draft] | **Wesley** (compile the appendix or drop the pointer) |

## Style (Wesley's rules)

- **"rig" / "twin":** none in the printed text (they appear in comments and in the label `fig:rig_photo` only). OK.
- **Semicolons in prose:** none. The two at controller.tex:18 and :83 are inside display math. OK.
- **Floats [H]:** every figure and table is `[H]` (the `\pendingfig` macro too). OK.
- **Bold paragraph leads:** none. Bold appears only in Table 5's over-bar cells and the title. OK.
- **One source line per paragraph:** two hard-wrapped paragraphs, controller.tex:27-29 ("The payload reference …") and controller.tex:42-44 ("Each UAV $i$ runs a tracking MPC …"). Table cells are also wrapped (reconfiguration.tex:113-124, metrics.tex:21-39); those are cells, not paragraphs.
- **Short captions:** all are 24 words or fewer except `fig:res_s8` (results.tex:31, 31 words). Move the panel list into the text if it needs trimming.

## What a supervisor would ask

- **Notation used twice:** **F** (dynamics at controller:18, and the constant force at :107); f_i (the thrust vector in model.tex, and the vertical share in eq:statics); r (ring radius, and **r**_i the cable rate); m (support margin, and m_L, m_q); θ (the tilt vector, and the path angle); **q** (cable direction, and the UAV quaternion in the tracker); d (the force **d**, the line length d, and the catch offset d_c); g (gravity, the gap g_max, and the gates g_xy, g_z).
- **Symbols with no definition or value:** x̂(t), W, W_N, Q, Q_N, Ω(·), c_z, λ_min/max, f_min/max, γ_max, η_max, u_T,max, and the values of K_xy and K_z.
- **No citation:** "by flatness" (controller:27-29); "the collaborator's controller" is never named or cited.
- **Why 200° in the relaxed drop rule** when eq:margin puts the centre of mass outside the polygon there (reconfiguration:66).
- **The X4 relay's feasibility:** 30 min with at least 7 of 8 on the ring and a 5-min pack budget means near-instant recharging. State that B is a budget and say what "charging" means (plan X4: "B is a budget, not a battery model").
- **"Real time"** is used both for the bench projection and for the flights. The tick figure (above) will draw the first question.
- **The sim studies use the 0.86 kg ring and 1 m cables** while the hardware moved to 1.15 kg. Give decision 30 Oct (d) as one sentence.
- **"spinny line"** is informal. Define it at first use (it is defined at test_cases:21) or rename it.
- **Section 2.1 of the thesis** (model.tex) points to a chapter the pack does not include.
