# Card X2-full: the supervisor's chain 4 -> 3 -> 2 -> 3 -> 4 on the figure-eight in one run (30 Oct 2026)

Owner Wesley. Row X2 of `docs/sim_runs_todo.md` and `docs/plan_rescope_2026-10-30.md` (X2: "a drop, or a release peak
> 30 deg" falsifies); decisions.md DECIDED 2026-10-30 ~15:10 (2): the full square chain on the figure-eight in one
continuous run, two variants differing at the square-to-T step, announced (soft detach) and unannounced with the drop
limit raised in simulation, every other step unannounced; the morning's restart-from-the-T version stays the boundary
case (~14:05). Written 30 Oct ~15:40 AEDT, before any run of it. **Bars FROZEN 30 Oct ~15:46 on Wesley's word, as
proposed (both choices below taken), before any X2-full run.** No new code (section 8).

**Two choices made here for Wesley to confirm before the bars freeze:**
1. **Both variants fly the same params, the drop limit at 80 deg and the 60 deg ring envelope off** (as X1's sqT
   cell), so they differ in the square-to-T event alone (DETACH vs RELEASE of plate 9). A is still scored against the
   rig's rule: its peak must stay at or under 30 deg (the rescope's falsifier and the 4-quad tilt drop), so a peak the
   rig would have dropped is a miss, not a silent pass; the higher limit only lets the rest of A's chain be scored
   after such a miss. The alternative (A under the chain's 45 deg file, `configs/rig/chain_pair_tip.yaml`) is one
   line in `x2_full_fig8_soft.yaml` and makes the variants differ in two things.
2. **Releases mid-lap, rejoins in the lap-end holds** (S's case 7 pattern, X1's `f4` release point: the centre crossing
   at speed), so every stage flies at least one full lap: square 1.5 laps, T 1, pair 1.5, T 1, square 1 (6 laps).
   A rejoin cannot start while the path runs (planner: "REJOIN REFUSED ... the path is running"; the rescope's stated
   limit), so the rejoins have to sit in a hold.

## 1. Question
Wesley (30 Oct): fly the supervisor's whole chain, the square 0/3/6/9 to the T 0/3/6, to the pair 0/6, back to the T
and back to the square, a lap per stage, on the figure-eight in one continuous run. Does it fly with the square-to-T
step announced; and with it unannounced, what does the drop rule's boundary look like when the limit is raised?

## 2. Hypothesis
- **A (square-to-T announced, soft detach 3 s).** Every step flies: the soft release peaks 23-30 deg (R2312 hover
  22.9, R2313 mid figure-8 25.9, R2821/R2822 hover with the marker-check params 29.6/29.0 deg; R2840/R2842 on the
  1.15 kg ring 24.9/26.6), the T's rider lets go with a peak of ~2-4 deg (R2821/R2822 2.2, R2314 3.5) and is caught by
  its magnet marker in ~0.1 s, the two rejoins fold in under 5 deg (hover: T to square 1.27-1.55 deg, R2578/R2579/
  R2596; pair to T 1.45-3.02 deg, R2589/R2594/R2595), and the square's last lap is within P of the no-event square.
  **The 30 deg bar sits within ~0.5 deg of the marker-check hover peaks: it may fall either way.**
- **B (square-to-T unannounced, the drop at 80 deg).** The square-to-T release rolls the ring to 55-65 deg (hover
  57-61, R2249/R2823; the announced hard detach mid figure-8 62.7 deg, R2311) and is back within 5 s; nothing is
  lost at 80 deg; the rest of the chain then flies as in A.

## 3. The one variable
The square-to-T step: an announced DETACH of plate 9 (`drone 1`, soft: `detach_unload_s` 3.0) in A, an unannounced
RELEASE of it in B. Everything else is one config pair, and the no-event base flies the same configs with no event.
- World `four_rigid_ground_rig_L100.sdf` (0.86 kg ring, 1 m rods), typed 0,270,180,90 = plates 0/9/6/3 (drones 0-3).
- Path: the figure-8 at size 1.0 (x +-1.0, y +-0.5 m; X1's `f4` and the thesis's `fig8big`), 0.2 m/s, a 5 s hover hold
  first, `traj_laps` 6 (lap ~34.5 s), `traj_lap_hold_s` 3.0 at each lap end (S's case 7 machinery, 23 Oct).
- Planner: 5 SQP iterations, `plan_fast_refs` on, the `slope` filter; `rejoin_departed` on, `departed_land` off (the
  freed quads stage on their plates' bearings, ~1.03 m from the ring centre, inside the +-1.57 m staging bound at every
  point of this path), io `tether_tips` (the rod ends: the marker check and the weld); `payload_tilt_envelope` false.
- Params `configs/twin/x2_chain_tip80.yaml` (new, twin only): `configs/rig/chain_pair_tip.yaml`'s values (the T and the
  pair fly under `drop_gap_deg` 200 with `min_survivors` 2; `int_k_xy` 0; `tip_detect_m` 0.04, the marker check that
  sees the T's ~0.1 N rider let go; the stretch check alone missed it, R2258) with `drop_tilt_deg` 45 -> 80.
- Events (both variants; WAIT events re-time the ones after them):
  ARM, TAKEOFF, WAIT_LIFT; WAIT_TRAJ lap 1.5 -> **DETACH 1 (A) / RELEASE 1 (B)** (square -> T);
  WAIT_TRAJ lap 2.5 -> RELEASE 3 (the T's rider, plate 3: T -> pair);
  WAIT_LOG "lap 4 done" -> +0.5 s ATTACH 3 -> WAIT_LOG "REJOIN WELD drone 4" (pair -> T);
  WAIT_LOG "lap 5 done" -> +0.5 s ATTACH 1 -> WAIT_LOG "REJOIN WELD drone 2" (T -> square);
  WAIT_TRAJ lap 6 -> +2 s LAND.
- Configs (`configs/experiments/`): `x2_full_fig8_soft` (A), `x2_full_fig8_unann` (B), `x2_full_fig8_base` (the
  square on the same lapped path, no event: P's base and the per-lap reference). The three differ only in `name`,
  `description`, the events and the base's `criteria`; all pass `tools/run_experiment.py --check` (30 Oct 15:39; the
  inertia warning is every L100 square config's).

## 4. Baseline
- **The no-event square on the same path**, `x2_full_fig8_base` x2 on the same tree (new: no lapped 4-quad figure-8
  has flown). P and the per-lap path errors read it. For scale: the single-lap square at size 1.0 in S's C, 11.32 mm
  and 1.05 deg (the it5 mean, s4_fig8big, ema).
- **The steps flown alone (hover unless said; exploratory rows):** soft square-to-T R2312 22.9 deg, R2313 (mid
  figure-8, size 0.5) 25.9; the chain square -> T -> pair in one flight R2821/R2822 (soft 29.6/29.0 deg, the rider's
  release 2.2 deg, caught in 100-118 ms, the pair 0.5-0.6 deg); unannounced square-to-T R2249 57.1 deg, R2823; X1's
  sqT cell (today, hover, the same 80 deg limit) is the twin's own reading of B's step; rejoins: T -> square E1b
  R2578/R2579 (ON_PLATE 24.9 s after START, fold-in 1.27/1.30 deg), pair -> T E2 R2594/R2595 (1.45/2.25 deg).

## 5. Pass / fail numbers (FROZEN 30 Oct ~15:46, Wesley, as proposed, before any X2-full run)
Bars are taken from the cards whose steps these are, unchanged: the chain card (`2026-10-21_chain_release.md`, arm 2,
the T to the pair), the rejoin card (`2026-10-23_rejoin_departed.md`, E1 T -> square, E2 pair -> T, E4 staging on a
path), S's P (`2026-10-23_scalability.md`), X1's sqT U5 (`2026-10-30_x1_release_ablation.md`, B's step), and the
rescope's X2 falsifier (30 deg). Scorers: `tools/s_metrics.py` (per event and per stage, rejoins, late ticks, drops),
`tools/x1_metrics.py` (B's step: abs peak, T6 settle), `tools/n2_bands.py`'s clock-free path error on lap windows
(section 8: the lap wrapper is missing), `tools/pair_swing.py` (the pair's roll and lean). Lap k = the planner's
trajectory clock from (k-1) to k laps.

**A, the claim: the full chain flies with the square-to-T step announced.**
- **A1, square -> T (DETACH 1, soft, at lap 1.5).** UNLOAD then the planner at n = 3 within 6.0 s of the DETACH
  (3 s unload, 2 s gate wait, 1 s), no unload cancelled; ring tilt peak (the tilt vector's largest departure from its
  mean over the 5 s before the DETACH, to the next event) <= 30 deg; no drop or loss. Reported: abs peak, time over
  15 deg, T6 settle.
- **A2, T -> pair (RELEASE 3, unannounced, at lap 2.5; chain card arm 2):** UNANNOUNCED DETACH and the planner at
  n = 2 within 1.0 s of the RELEASE; tilt peak < 15 deg from the RELEASE to the ATTACH; the pair's lean < 5 deg (mean
  over its last 10 s before the ATTACH); no drop. Reported: the pair's roll peak-to-peak (arm 2's 5 deg was a hover
  bar; on the path it is not one), S's L (<= 8.83 deg), detection by marker or stretch.
- **A3, pair -> T (ATTACH 3 at lap 4's end; rejoin card E2) and A4, T -> square (ATTACH 1 at lap 5's end; E1):**
  (a) REJOIN_ON_PLATE within 30 s of REJOIN_START; (b) JOINT_ATTACHED within 0.5 s of it, then REJOIN_WELD and
  FOLD_IN; (c) ring tilt <= 5 deg from REJOIN_WELD to +15 s; the rod end within 2 cm of its plate point through the
  blend; the rejoiner's planned tension at the blend's end within 10 % of the balanced share; no REJOIN_REFUSED or
  REJOIN_ABORT. A4 also: the square's lean <= 2 deg over the last 10 s before LAND. Reported: S's J (the planner one
  bigger within 20 s of the ATTACH; E1's hover rejoin reached its plate 24.9 s after START, so J's 20 s is expected to
  miss here and is not this card's bar).
- **A5, the square's last lap (lap 6, no event; S's P):** clock-free path error <= max(1.25 x the base's lap 6 mean,
  that mean + 3 mm), and tilt peak <= the base's lap 6 mean + 1 deg.
- **A6, the whole run:** no drop, abort or envelope fault; every quad lands; through the laps flown while staged,
  every staged drone inside +-1.6 m, at least 0.6 m from the ring centre and 0.8 m from every carrier body (E4).
- **Confirms:** A1-A6 in both valid repeats. **Falsifies:** a drop or loss; A1's peak over 30 deg; any of A2-A6
  missed in a valid repeat.

**B, the boundary: the square-to-T step unannounced, the drop at 80 deg (simulation only).**
- **B1, square -> T (RELEASE 1 at lap 1.5; X1's sqT U5, frozen 15:20):** no loss (the 80 deg drop, the planner's
  drop, or a quad's own 60 deg attitude envelope); abs peak <= 80 deg; T6 settle (release_bands' time back within
  max(2 sigma_pre, 0.5 deg) of the post-release steady tilt, held 1 s) <= 5.0 s. Reported: the peak against the rig's
  30 and 45 deg rules (predicted over both: that is the boundary), detection time, the resize time.
- **B2-B6:** A2-A6 unchanged.
- **Confirms** (the chain completes past an unannounced square-to-T once the limit is lifted): B1-B6 in both valid
  repeats. **Falsifies:** a loss at 80 deg, T6 over 5.0 s, or any of B2-B6 missed. A loss in B is reported as "lost
  even at 80 deg"; the restart-from-the-T version (decided 30 Oct ~14:05) stays the way the rest of the chain is shown.

**Validity (a void run is flown once more):** late planner or tracker ticks <= 1 %; no failed planner solve;
read-back `traj_laps` 6, `traj_lap_hold_s` 3, `detach_unload_s` 3, `rejoin_departed` true, `drop_tilt_deg` 80,
`tip_detect_m` 0.04, `drop_gap_deg` 200, `min_survivors` 2, the filter slope; every event fired (no TRAJ_TIMEOUT or
LOG_TIMEOUT); a REJOIN_REFUSED "the path is running" means a lap-end hold was missed: void (as S's case 7), not a
falsification; a gap drop of the T or the pair is a config error: void. Lap 1 has no event in any run, so each run's
lap 1 must sit within P's form of the base's lap 1; a miss means something else differs, and the run is looked into
before it is scored.

## 6. Repeats
2 each: A, B and the base (6 runs).

## 7. Cost
Lab PC, 4 slots x 4 cores (4-quad runs flew RTF 0.21-0.27 there today): each run ~345 s of sim time (lift ~25 s,
6 laps 207 s, lap holds 15 s, two rejoins with their 10 s blends ~65 s, the unload and resize holds ~8 s, landing
~20 s), ~30 min wall; the base ~25 min. `python3 tools/remote/lab_batch.py --slots 4 --cpus 4 --wall-cap-min 60
--warm-from-config configs/experiments/x2_full_fig8_{soft,unann,base}.yaml` with each config listed twice
(interleaved). **~1 h of lab PC** (4 + 2). Laptop: none. The table's 1.15 kg cell is not written: the pair cannot hold
1.15 kg at throttle_max 0.8 (R2834 put the ring on the floor) and held it at 0.9 (R2840/R2842), so that cell needs a
second variable (Wesley's call: A on `four_rigid_ground_rig_L100_p115.sdf` at throttle_max 0.9, x1).

## 8. New code?
No change to `src/`. One new twin params file, `configs/twin/x2_chain_tip80.yaml`. Everything the chain needs exists
and has flown in parts: DETACH with `detach_unload_s` on a path (R2313), unannounced releases with the marker check
(R2821), staging and rejoins in hover and staging on a path (rejoin card E1-E5), lapped paths with lap-end holds,
WAIT_TRAJ and rejoins in the holds (S case 7, R2779). Not a new law or gate, so no critic.
What the runner and tools cannot do yet (none blocks the runs; all are scoring or design limits):
- A rejoin cannot start while the path runs (REJOIN REFUSED): rejoins only in a lap-end hold (the rescope's limit).
- A release inside a lap-end hold would cut the hold to `reconfig_hold_s` (1.5 s): the resize sets the hold, it does
  not extend it. Not used here (releases mid-lap); it matters if a later variant releases at a lap end.
- `detach_unload_s` applies to every announced DETACH of a run: one soft step per run is fine, a mixed soft / hard
  announced chain is not expressible.
- No committed scorer cuts the clock-free path error and tilt by trajectory-clock lap (S's case 7 used a scratchpad
  wrapper): ~30 lines over `x1_metrics._clock_free_err` and the planner log's `traj_t`, in `tools/`.
- `payload_tilt_envelope false` sets the trackers' ring envelope to 180 deg; there is no knob for another value (X1).
