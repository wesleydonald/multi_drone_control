# Lane 5, 12 October 2026: hand-back

Four tasks from the brief, in order. Times are from `date`. Nothing merged, nothing pushed; commits are authored by
Wesley with no trailers; the main tree, week5, the rig sheets, the lab PC, the registry and `~/thesis` were not
touched (reading only).

## Branches and worktrees

| branch | worktree | from | what |
|---|---|---|---|
| `wed-review` | `/home/wesley/mdc_wt/wed-review` | rig-prep 9c9dcc2 | task 2: `docs/rig_2026-10-14_review.md` (commit 7df6834, 11:27) |
| `thesis-figs` | `/home/wesley/mdc_wt/thesis-figs` | thesis-rig a493c7d | task 3: the figure tooling from rig-prep (da45990), then the four figures, the registry, the provenance and this note |
| `thesis-cites` | `/home/wesley/mdc_wt/thesis-cites` | thesis-rig a493c7d | task 4: the seventeen citations into the bibliography, the placeholders replaced (one commit) |

## 1. The supervisor deck (done 11:13)

https://claude.ai/artifact/TwjqA7ebTDrkgDK6nwutiR, version 7. Three slides changed, in plain words: the status table
(the combined claim's hover lines supported, the circle take-back and the push abort not; the tilt cut's timing bars,
its 1-of-2 table and the take-back race found and fixed; Chapters 2 and 3 drafted), the full-mission slide (6 of 7
circle missions reattached in the twin; the hover lines all pass), and the plan slide (front-loaded: Wednesday tries
everything, Thursday is for retries and fixes, the last weeks for repeats and data). The deck is still private.

## 2. Wednesday red-teamed (done 11:27, branch wed-review)

`docs/rig_2026-10-14_review.md`: 13 findings ranked, each with a fix; the pack budget (about 19 min of carrier
flight and 15 min for the newcomer, no pack count on the sheet); what each flight must record for the thesis; six
cheap extra tests; a walk of every sheet item. The three that matter most:
1. The 09:00 I/O line starts the mocap node without the tip body or the puck, so W and L cannot be measured and the
   puck check has nothing to read. One line change.
2. P7's attach check runs with the weld point at zero, so a face on the plate reads about 3 cm from the plate point
   and records no catch. Type `weld_point_z:=$W` on the P7 line after P3.
3. R9's chain matches the twin's step for step in the code, but two links have never run on the rig; both can be
   rehearsed props-off during the load test.

## 3. Thesis figures from the data that exists (done 12:00, branch thesis-figs)

### Inventory of the plan's 14 main figures

| figure | status | where |
|---|---|---|
| F1 storyboard | not made: six 3-D frames need a frozen-tree full mission and the frame renderer (the technique is in `tools/attach_storyboard.py`); the tree freezes with Friday's claim | |
| F2 rig and geometry | the to-scale geometry exists (`fig:geometry`); the rig photo waits for 14 Oct (`fig:rig_photo` pending) | system_overview.tex |
| F3 free-body diagram | exists (`fig:payload_system`) | model.tex |
| F5 cascade and owners | exists (`fig:owners`, 12 Oct) | authority.tex |
| F6 state machine | exists (`fig:reconfig_states`, 12 Oct) | reconfiguration.tex |
| F7 thrust map | exists (`docs/figures/F_thrust_map`, 9 Oct) | res_carry.tex |
| **F8 carry circle** | **made today**: `docs/figures/F_rig_carry`, `fig:F8` | for 4.1 |
| F9 circle-release storyboard | exists (`F_rig_circle_detach`, on the signed metric since 11 Oct) | sim_detach.tex |
| **F10 releases on the rig** | **the rig half made today**: `docs/figures/F_releases`, `fig:F10` (panel (a)); the twin crossover panel comes with Tier A | for 4.2 |
| **F11 magnet anatomy** | **made today**: `docs/figures/F_magnet_anatomy`, `fig:F11` | for 4.2 |
| F12 levers and margin | not made: only two margin levels exist until Tier B | |
| **F14 proof timeline** | **made today** on the twin claim runs: `docs/figures/F_proof_timeline`, `fig:F14`; the rig's plate-noise band waits for the proof replay | for 4.3 |
| F15 rig attach | waits for Wednesday | |
| F16 ledger | waits for Wednesday | |

### How the four were made

- Scripts: `tools/thesis_leave_figures.py` (carry, releases, magnet) and `tools/thesis_join_figures.py` (proof).
  Every run id comes from `configs/thesis_figures.yaml` through the new `tools/figure_runs.py`; a script holds none.
  Wednesday's flights go into the yaml (a row each under `F_releases`, and the 7 Oct conversions' command is in the
  yaml's comment) and the script is re-run.
- The rig flights are read from their `rig_to_run.py` conversions. The 8 Oct ones exist on the main tree; the 7 Oct
  ones (r006, r0013, r200006) were converted here into `results/rig/2026-10-07/<name>_run` (untracked, 3-14 s each,
  the command is in the yaml). The main tree can make the same three with that command.
- The thesis-figure loop ran for each: render, audit (`figure_audit.py`, all four PASS), look, one blind critic each,
  the accepted fixes, re-render, re-audit. Captions, sections and citing sentences are in each
  `docs/figures/<ID>/caption.md`; the labels are `fig:F8`, `fig:F10`, `fig:F11`, `fig:F14`;
  `docs/figures/PROVENANCE.md` has the rows. `Results_and_Discussion.tex` is untouched, as asked.
- The figure tooling (`thesis_rig_figures.py`, `thesis_rig_twin_figures.py`, `release_bands.py`,
  `release_transient.py`, `attach_metrics.py`, `rig_to_run.py`, `drop_trigger_replay.py`, the registry, the
  provenance and the figure outputs) was brought onto thesis-figs from rig-prep in one commit, so the thesis branch
  regenerates every figure it includes.

### Numbers the figures print (so the chapter text can cite them)

| figure | numbers |
|---|---|
| F8 | measured pull at 60 deg: 0.17 s behind the reference on average (mean along-track -2.1 cm, worst -5.5), height sd 0.5 cm, tilt mean 1.7 deg; planner's pull at 45 deg: 0.35 s (-4.4 cm, worst -17), 5.0 cm, 2.7 deg; one lap each (26 s) |
| F10a | tilt rises: 8 Oct 11.2 (announced), 5.3, 16.9 (150 deg gap), 8.0 (circle) against twin ranges 6.0-7.1, 6.0-6.4, 14.6-17.4, 4.0-5.4; 7 Oct r0012 6.1, r0013 68 (capsized), r200006 31 (recovered) and 63 (capsized, in the landing) |
| F11 | r0013: the 30 deg rod off 0.17 s after the release at 3.1 deg of tilt (the replay card, on the bag at 91 Hz, says 0.155 s and 2.6 deg), the 180 deg rod at 0.39 s and 47 deg, the latch at 0.47 s; the planned step 2.80 to 4.63 N (+66 %) and 2.29 to 3.82 N (+67 %) on the two survivors beside the gap, 3.62 to 2.37 N (-35 %) opposite. r200006: the 270 deg rod off at 0.9 deg, the ring to 31 deg and back, LAND 0.58 s after; the 180 deg rod off 2.80 s later at 7.5 deg, the latch at 3.21 s; 3.0 N per rod before |
| F14 | contact to proof 1.08 and 0.72 s; plate rise at the proof 3.5 and 5.8 mm (the manager logged 3.3 and 5.7); tip drift at the proof 0.7 and 1.2 mm; throttle step -0.2 and -0.5 % of hover; ring tilt rise 1.8 and 2.1 deg |

The F11 instants come from the 50 Hz run table with the pivot 4 cm below the body; the card's come from the bag at
91 Hz with the body's attitude. They agree to about 0.02 s; the caption quotes the figure's own numbers.

### Critic suggestions rejected, with the reason

- F8: a cross-track or distance-norm panel (the caption now speaks in along-track and height terms instead).
- F14: colouring the two proof lines by run is done; plotting the "gated quantity" is not (the dwell is stated in
  the caption instead, 0.3 s, which is what the manager holds).
- F10a: cutting the shape key (kept: the caption refers to it); a mark for "no twin" on the capsize rows (they say
  "capsized"; only the planner's-pull hover row says "no twin pair").
- F11: trimming the planned tensions at the latch is done; the "gated quantity" and a second y axis were not asked.

## 4. The seventeen citations (done 12:20, branch thesis-cites)

Every entry of the approved batch is in `references_extra.bib` with its DOI, venue, volume, pages and year checked
against Crossref or OpenAlex (the preprint against its arXiv listing), and every bracketed placeholder in the
Chapter 2 files and in Section 3.4 is replaced by the citation. The thesis builds with no undefined references or
citations (110 pages). Two things to know:
- Loianno and Kumar 2018 is cited for cooperative transport with no sensor on the payload (each quadrotor's camera and
  inertial unit), not for "a sensed grasp": the abstract was not reachable to confirm a gripper sensor, so the
  sentence does not claim one. Shankar 2020 carries the sensed-grasp point on its own.
- The "Farina, Zhang and Scattolini 2020 (TCST)" of the plan is their hierarchical MPC with plug-and-play
  capabilities, TCST 28(2), 2020 (online 2018); Gros et al. is International Journal of Control 93(1), 2020 (online
  2016); RotorTM is T-RO 40, 2024 (online 2023). Print years are used.

T3's cells from the full texts (plan C2-A: Bacheti, Liang, Saiella, Farras, Hajieghrary) are not done: the full texts
are not on this machine (only Quan's is under docs/papers), and the lane did not fetch PDFs.

## Open questions for Wesley

1. The 7 October releases in F10a: decision 168 keeps r0012 and r0013 out of the thesis *tables*; the plan's F10a
   lists every rig release event. The figure shows them as a second group, labelled by date and rod plan. Keep them
   in the figure, or drop the 7 October group (one yaml edit)? Recommended: keep them; the magnet story needs them.
2. F11's instants: quote the figure's 50 Hz numbers (0.17 s, 3.1 deg) or the replay card's bag numbers (0.155 s,
   2.6 deg)? Recommended: the figure's, and say in the text that the bag replay gives 0.155 s.
3. The pack count for Wednesday (review finding 4).

## Script for Wesley (deletions are refused here)

```bash
# after the branches are merged or declined
git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/wed-review
git -C /home/wesley/multi_drone_control branch -D wed-review
git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/thesis-figs
git -C /home/wesley/multi_drone_control branch -D thesis-figs
git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/thesis-cites
git -C /home/wesley/multi_drone_control branch -D thesis-cites
# the 7 Oct conversions in the thesis-figs worktree go with it (untracked)
```
