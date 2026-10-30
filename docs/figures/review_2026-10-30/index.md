# Review figures, 30 Oct 2026 (not in the thesis yet)

Built Fri 30 Oct 2026 from recorded run data, read-only (no Gazebo run, nothing in src/ or configs/ touched, nothing
committed). Each figure is a PDF (for LaTeX, `\includegraphics[width=\textwidth]`) and a 300 dpi PNG at print size
(6.27 in, Computer Modern), drawn by the script of the same name beside it. `review30_style.py` reuses the 23 Oct
review look (`../review_2026-10-23/review_style.py`). Both PDFs pass the skill's audit (size, embedded fonts, nothing
under 7 pt, no text collisions); grey and deuteranopia previews checked.

**Colour.** The ring's data in the thesis blue (`#2166ac`, as `fig_s_tick` in the same section); grey only for events
and bands. In `fig_s8_events` the blue's shade is the team size after the event (darker = fewer carriers), keyed by a
framed legend below the panels.

**Style of 30 Oct (Wesley).** Re-rendered the same day in the thesis style: a full box round every axes, and any key as
one framed legend below the figure, in columns (`plot_style.legend_below`), never on the plot.

## Why these two

The scalability claim (S, card `docs/experiments/2026-10-23_scalability.md`, N) is the one finished simulation result
the thesis needs that has no figure from its claim runs: the Overleaf results show the tick time (`fig_s_tick`) and four
Gazebo screenshots of an exploratory lap-by-lap chain, and the earlier `review_2026-10-23/fig_s_chain` drew the
exploratory one-iteration run R2597. The draft `res_scale.tex` leaves `fig:scale_chain` pending.

## The figures

| # | Figure | One message | Data (run ids stay out of the thesis) |
|---|---|---|---|
| 1 | `fig_s8_chain` | Eight carriers lose four of their number one at a time, unannounced, take the four back one at a time, and the ring stays within 3.3 deg of level and 81 mm of its hover point. | R2766 (`s8_chain4back_R50_L100_it5`), from the top of the lift to LAND. run.csv (tilt, offset = 3-D distance to the reference), the planner log (n, moved onto the run clock by `t_ready_sim` as `tools/s_metrics.py` does), events.csv (RELEASE; REJOIN_WELD to REJOIN_BLEND_OVER). Its repeat R2767 is drawn in figure 2. |
| 2 | `fig_s8_events` | The fewer carriers share the ring, the further one event moves it: a release about 6 cm with eight or seven carrying and 8 cm with six or five; a fold-in 4.6 cm into five down to 3.7 cm into eight. The ring is back within 5 mm about 5 s after a release and 4 s after a fold-in ends. | R2766 and R2767, aligned at each RELEASE (a) and each REJOIN_WELD (b); offset as in figure 1. |

Both runs: lab batch 18, 23 Oct, tree week7 `63b504f` (diff: docs and the registry only), 5 SQP iterations, 8 quads at
45 deg, 0.9 kg ring of radius 0.5 m, 1 m rods, hover at 1 m, `SIM_MOCAP_VEL_FILTER=ema`, RTF 0.217-0.218 (twin only,
not real time). Reviewer 24 Oct: supported (strong); N4 missed by 0.7 mm in both runs, reported as missed (Wesley 24 Oct).

Numbers drawn, from the scripts' printout (R2766 / R2767):
- tilt peak 3.28 / 3.31 deg (at the fourth release); offset peak 80.9 / 83.1 mm (third or fourth release);
- release offset peaks: 8 to 7: 62.0 / 62.0 mm; 7 to 6: 59.9 / 59.6; 6 to 5: 80.5 / 83.1; 5 to 4: 80.9 / 82.6;
- fold-in offset peaks: 4 to 5: 45.5 / 45.8 mm; 5 to 6: 42.6 / 42.0; 6 to 7: 40.2 / 40.2; 7 to 8: 36.5 / 36.3;
- back under 5 mm: 5.2-5.7 s after a release; 14.0-14.7 s after a weld (the blend ends 10.1-10.2 s after it).

## Draft captions (short; the conditions and numbers go in the paragraph)

1. **fig_s8_chain.** Eight carriers release four of their number one at a time, unannounced, and take them back one at
   a time, in simulation (one of two runs; $t = 0$ when the ring reaches its hover height): (a) the number of carriers,
   (b) the ring's tilt, (c) the ring's distance from its hover point. Grey lines mark the releases, grey bands the
   fold-ins. With seven or five carriers the layout is uneven and the ring hangs about 1$^\circ$ off level.
2. **fig_s8_events.** How far the ring moves after each release (a) and each fold-in (b) of Figure 1, both runs (they
   agree within 3\,mm). Darker blue: fewer carriers after the event.

Citing sentence (draft): "Eight UAVs released four of their number and took them back with the ring within
3.3$^\circ$ of level (Figure 1); each release moved it 6 to 8\,cm and each fold-in under 5\,cm, less the more UAVs
carried, and it was back within 5\,mm about 5\,s later (Figure 2)."

## The loop

- fig_s8_chain: round 1 three panels; round 2 one-line y label ("Ring offset"), 3.2 in tall.
- fig_s8_events: round 1 labels crossed the lines; round 2 a key in the lines' own colours, top right.
- Blind critic (PNG and caption only), fixes taken: "peak" on the peak labels; darker label text (0.3 grey); the
  caption says one of two runs, defines $t = 0$, uses "carriers" as the axis does, and names the odd-team tilt; in
  figure 2 the offset axis 0-100 mm as in figure 1, both windows from -2 s, a wider blue range (Blues 1.0 to 0.52),
  "fold-in" instead of "weld" on the axis, and the caption gives the runs' agreement (at most 2.8 mm on any event).
  Rejected: (1) the tilt's drops to zero at two releases are not missing data (no NaN, samples every 21-33 ms): the
  tilt vector swings through level as the ring tips the other way (e.g. (0.73, 0.77) deg before, (-0.07, -0.07) after);
  (2) one label per release pair in figure 2(a): the two events of a pair differ by under 3 mm, so one label is what
  the data supports; (3) a label on the release line: the axis says it.

## Not drawn, and why

- **C, the paper's method against our planner (R2685-R2690 and baselines): not final.** On 23 Oct Wesley made Quan et
  al. as published the comparison arm; its bars freeze only after he has seen videos and accuracy figures (24 Oct),
  and none of its cells has flown as a claim. The port's rows stand as flown but span three patch trees (be34f95 +
  aad7c59a / 537551ba / cae5cf26). The review figures `fig_c_line`, `fig_c_yaw` (23 Oct) and `fig_quan_lines` (24 Oct)
  already show them.
- **The 1 m re-fly (R):** tables, and its figures exist (`review_2026-10-23/fig_refly_orbit`, `fig_flight_1m`,
  `fig_traj_1m`, `fig_detach_1m`). They fly the 0.86 kg ring, while the hardware now flies the 1.15 kg one.
- **S at four carriers, one against five iterations (R2700-R2711, confirms):** two arms whose dots overlap; the draft's
  table says it better. `fig_s_quality_n8` covers eight.
- `review_2026-10-23/fig_s_chain` (the exploratory one-iteration run R2597) is superseded by `fig_s8_chain`.

## Before these go into the thesis

- The S runs flew `SIM_MOCAP_VEL_FILTER=ema` (typed in the lab env; the twin's default until 28 Oct, `slope` since).
  Every flight-loop change after 63b504f is behind a switch that is off in these runs, so the runs are not stale. But
  the final-tree re-fly the rules ask for will fly `slope` unless ema is typed.
- Wesley's 29 Oct decision (every twin link becomes a cable like the hardware's; not built yet) will put every twin
  result, these included, up for a re-fly. The scripts read their run ids from the top of the file, so a re-render is
  one line.

---

## Added 30 Oct: `fig_solve_time` (the planner's solve time per tick, by team size, as flown)

**One message.** The solve grows with the team. In real-world flight every solve finished inside the 100 ms period:
median 16, 34 and 56 ms with two, three and four carriers, and with four, 32 % of the solves ran past the 60 ms budget.
In simulation (a different computer, no budget, slower than real time) the median grows to 210 ms with eight.

**What is drawn.** (a) Real world, blue; (b) simulation, red. One violin per team size over all its solves (shortest
to longest; Gaussian kernel density, every violin scaled to the same widest half-width), a bar and the number at its
median. Dashed grey: in (a) the 60 ms budget and the 100 ms period, in (b) the period only. Full box, no legend (the
two panels are named).

**Where the two lines come from.** Period: `PLANNER_HZ = 10.0` (`src/mpc_planner/mpc_planner/planner_node.py` l. 64;
the timer at l. 580). Budget: `solve_budget_s: 0.06` (`src/bringup/config/real.yaml` l. 44; code default 0, i.e. off,
in `params.py` l. 47 and `planner_solver.py` l. 37-39, which the simulation keeps). The budget is checked after each
of the first four iterations (`planner_solver.py` l. 339-341): past 60 ms no new iteration starts, so a fifth started
before 60 ms can still end after it. Five iterations: `plan_sqp_iters ''` = 5 (`common.yaml` l. 51) ->
`STEADY_ITERS = 5` (`planner_solver.py` l. 35); every log's params.json reads `plan_sqp_iters 0` (rig, = 5) or 5 (sim).

**Data** (planner log `logs/dissipative_planner/*/log.csv`; rows with phase `planner` AND `published == solved`;
`solve_ms` as logged, grouped by the log's own `n`):

| Team | Source | Flights / runs (ticks) |
|---|---|---|
| 2 | real world, 28 Oct, earlier setup (0.86 kg ring, 0.55 m cables) | RIG-1028-f2a (498), -ls2a (371), -ls2b_4 (513), -f2b (498), -f2c (500) |
| 3 | real world, 29 Oct, later setup (1.15 kg ring, 1 m cables), the triangle | RIG-1029-sq2_5 (855), -sq3 (558), -sq4_5 (510), -sq5_2 (565), -k1_4 (461) |
| 4 | real world, 29 Oct, later setup | RIG-1029-h4 (131), -h4f_3 (225), -f4 (500) |
| 4-8 | simulation, lab PC batch 18 (two runs at a time, RTF 0.22-0.25) | R2764-R2779 (all 16; the chains give 4-8, the hovers and laps 8) |

Real world: one planner tree, 413ec0e (the working copy of 28-29 Oct touches no file in `src/mpc_planner`; checked in
each flight's `_code.txt`); the ground computer ('tuf'); `plan_fast_refs` false, so `solve_ms` includes the plan's
residual (`res_ms` median 1.1 / 3.6 / 6.0 ms at 2 / 3 / 4). Simulation: tree week7 63b504f, `plan_fast_refs` true
(the residual after the publish, not in `solve_ms`), `solve_budget_s` 0. Every listed flight lifted the ring to
0.98-1.08 m.

**Numbers** (printed by the script):

| Team | Where | Ticks | Median (ms) | p95 (ms) | Max (ms) | Over 60 ms | Cut short (< 5 iterations) | Whole tick median / p95 (ms) | Tick over 100 ms |
|---|---|---|---|---|---|---|---|---|---|
| 2 | real world | 2380 | 15.8 | 32.0 | 51.2 | 0 % | 0 % | 29.6 / 51.0 | 0 % |
| 3 | real world | 2949 | 34.2 | 41.2 | 61.7 | 0.1 % | 0 % | 62.0 / 73.4 | 0 % |
| 4 | real world | 856 | 55.5 | 72.6 | 77.2 | 32.5 % | 6.9 % | 81.1 / 100.3 | 5.1 % |
| 4 | simulation | 2738 | 45.5 | 62.5 | 108.8 | 7.7 % | 0 % (no budget) | 61.6 / 80.1 | 0.7 % |
| 5 | simulation | 4275 | 76.5 | 96.4 | 179.6 | 97.6 % | 0 % | 97.3 / 120.5 | 39.4 % |
| 6 | simulation | 6466 | 113.1 | 137.8 | 263.1 | 100 % | 0 % | 140.0 / 167.4 | 100 % |
| 7 | simulation | 5631 | 159.5 | 189.3 | 290.1 | 100 % | 0 % | 194.5 / 228.1 | 100 % |
| 8 | simulation | 11286 | 210.1 | 245.8 | 495.2 | 100 % | 0 % | 251.2 / 290.1 | 100 % |

(Over 60 ms: share of solves longer than 60 ms. The simulation's times are wall clock on the lab PC while its planner
ticked every 100 ms of simulated time, about 0.45 s of wall time at RTF 0.22, so none of its solves made the plan late.)

**Things the text should know.**
- **Not every real-world tick ran five iterations.** With four carriers the budget cut 6.9 % of the ticks to three or
  four (`sqp_calls`); two and three carriers ran five on every tick.
- **The whole tick is longer than the solve.** The rest of the tick (references, publishing, logging) adds about 14 ms
  at two carriers and 26-28 ms at three and four. With four, the whole tick passed the 100 ms period on 5.1 % of the
  ticks (longest 113 ms); the solves themselves never did (longest 77 ms).
- **The draft's "median of 27 ms with two carriers" (results_discussion.tex, the tick paragraph, and its comment
  "RIG-1028-f2b median 27.2") is wrong:** f2b's planner phase runs on for 122 s after touchdown with no solve, and
  those rows repeat the last solve time (27.2 ms). On solved ticks f2b's median is 16.7 ms, the pair's 15.8. The same
  filter moves sq5_2 from 32.6 to 34.6 and f4 from 55.9 to 56.3 ms (27.7 -> 33.0 % over 60 ms).
- **Left out: the 22 Oct flights** (three carriers: h3n, h3, f3, f3big, f3h, ls3, ls3c; trees 15a9c14 / e120f41 /
  964796f / be34f95, 0.55 m cables). One code tree per figure, and their tail is different: median 38.7, p95 70.7,
  max 88.5 ms, 16.4 % over 60 ms, 6.5 % cut short, against 34.2 / 41.2 / 61.7 / 0.1 % / 0 % on 29 Oct. There were no
  four-carrier flights on 22 Oct. The 28 Oct pair hovers (hE1, hS1, hS2) and lines (ls2b_5, ls2c) are not drawn either
  (not in the list); their medians are 15.7-18.3 ms.
- **No two- or three-carrier simulation** in batch 18; other batches are other trees and other slot loads.
- The simulation at four (46 ms) is faster than the real world at four (56 ms): different computers, and in flight
  the ground computer also runs the trackers, motion capture and the recording. The idle-bench numbers (24 Oct, the
  ground computer, five iterations) are 9.7 / 17.3 / 31.8 ms at 2 / 3 / 4 and 215 ms at 8.

**Draft caption.** The planner's solve time per tick against the number of carriers, at five iterations per tick, as
flown: (a) on the ground computer in real-world flights (two carriers on the earlier setup, three and four on the
later one), (b) in simulation, on a different computer, which ran at about 0.22 of real time with no budget. Each
violin holds every solve of that team size, and the bar and the number mark its median (ms). Dashed: the 60\,ms
budget, after which the planner starts no new iteration, and the 100\,ms period. With four carriers, 32\,\% of the
real-world solves ran past the budget, and none reached the period.

Citing sentence (draft): "As flown, at five iterations per tick, the median solve took 16, 34 and 56\,ms with two,
three and four carriers, and every solve finished inside the 100\,ms period; with four, 32\,\% ran past the 60\,ms
budget, which cut 7\,\% of the ticks to three or four iterations (Figure~X)."

**The loop.** Round 1: two panels, violins over the middle 99 %, both lines in both panels. Round 2: median labels
moved clear of the bodies; (b) widened so "210" clears the box. Blind critic (PNG and caption only), taken: no budget
line in (b) (the simulation has none) and the clock of its period in the caption; violins over all solves (no flat
caps; the tip is the longest solve, so "none reached the period" is read off (a)); "32 % over budget" written above
the four-carrier violin; a y label on (b); "Number of carriers". Rejected: (1) drop the top and right spines (Wesley's
full box, 30 Oct); (2) n under each tick (in this index; minimal words); (3) a hatched two-carrier violin for the
earlier setup (same problem structure, the caption says it); (4) line labels on one side in both panels (each side is
where the data leaves room). Audit PASS (6.27 x 2.5 in, CMR only, nothing under 8 pt, no collisions).

**Files.** `fig_solve_time.{py,pdf,png}`; the PDF copied to
`~/mdc_wt/thesis/docs/thesis_drafts/full_draft_2026-10-30/Images/results/fig_solve_time.pdf` (no .tex edited).

---

## Added 30 Oct: `fig_rejoin` (redesign of Figure 17, fig:res_rejoin; same file name)

**One message.** A carrier released from the T flies back to its own plate, its cable's end is lowered onto the plate
and caught, it is folded in, and the ring stays level (under the 5 deg bar) throughout.

**What is drawn.** (a) A side view from one fixed camera (from +x, 10 deg up: the rejoiner flies in the y-z plane),
the rejoiner in red at five numbered, named moments (1 released, 2 staged, 3 over its plate, 4 caught, 5 folded in;
1 and 5 share the slot), its path with one arrowhead per leg (dashed: out to the staging point), its cable pale with
an open circle at the cable's end, its plate red; the two carriers that stay in grey and the ring at the catch; a
"from above" inset showing the T and the trip, turned to match. (b) the distance from the cable's end to its plate
point, (c) the number of carriers (the planner's n: 3 -> 2 -> 3, not 3 -> 4), (d) the ring's tilt against the 5 deg
bar; grey lines and badges at the five moments.

**Data.** R2594 (`rj3_pairrelease_rejoin_L100_tip`, the T rejoin of Table "A departed carrier rejoins", tree (iii);
0.86 kg ring, 1 m links, three UAVs at 0/90/180 deg). run.csv, events.csv, the planner log (n, moved onto the run clock
by `t_ready_sim`), and `gz_record/state.tlog`: the rod's logged pose gives the cable's end (decoded by
`tools/snapshot_render.py`; the state log's clock is run.csv's t + t_ready_sim, checked: the UAV's logged position
matches to 1 mm rms). Moment 3 = the first sample with the UAV within 2 cm (horizontally) of its plate.

**Numbers** (script printout): moments at 0, 20.0, 25.5, 37.8, 47.9 s after the release; the weld 17.76 s after the
rejoin command; the cable's end 1.19 cm from the plate point at the catch; tilt peak 3.46 deg after the release (the
ring rocks as the pair takes over) and 1.45 deg in the fold-in (weld to +15 s).

**One thing to know (surprising).** The rod's logged end was within 2 cm of the plate point from 28.0 s, ten seconds
before the catch at 37.8 s. The planner's own reading of the end (the twin's marker, `/drone_1/magnet_tip_state`, its
launch-log lines) said 3.3 -> 2.8 -> 2.4 -> 1.8 -> 1.4 -> 1.1 cm over those ten seconds, while the rod's end in the
state log read 0.3-0.9 cm; the two agree at the catch (1.1 / 1.2 cm). The catch waits for the planner's reading (within
1.5 cm, still for 0.3 s), so a 1-3 cm offset between the marker and the rod's end cost about 10 s here. Not chased
(no Gazebo); worth a look at how the twin's tip marker is placed.

**Draft caption.** A carrier released from the T flies back to its own plate and is folded in, in simulation (three
UAVs, 0.86\,kg ring, 1\,m links). (a) Side view: the returning UAV (red) at five moments, 1 released, 2 staged (the
rejoin command), 3 over its plate, 4 caught, 5 folded in, with its cable and path (dashed: out to its staging point);
the two carriers that stay in grey; inset, the T from above. (b) Distance from its cable's end to its plate, (c) the
number of carriers, (d) the ring's tilt against the 5$^\circ$ bar. The end was caught 1.2\,cm from its plate 17.8\,s
after the rejoin command, and the tilt peaked at 3.5$^\circ$ after the release and 1.5$^\circ$ in the fold-in. While
free, the 1\,m cable swings as a pendulum.

**The loop.** Round 1: a 3-D oblique view with role colours (carrying blue, departed grey, rejoining red): moments 1, 2
and 4 crowded, labels collided. Round 2: one subject colour (red), the near side view, named moments with halos.
Blind critic, taken: arrowheads per leg; pale cables with an open circle at each end; a "from above" inset for the T and
its own plate; "1.2 cm" written at the catch in (b); (c) a thin strip; the bar label moved off moment 5's line; darker
names; the scale bar off the cable's end. Rejected: drop the box and the top/right spines (Wesley's full box); "bar" ->
"limit" (the thesis says bar throughout); a light-to-dark red ramp (one colour plus arrowheads is simpler); an inset
zoom of (b) (the number is written on the catch).

**Files.** `fig_rejoin.{py,pdf,png}`; the PDF copied over the draft's `Images/results/fig_rejoin.pdf` (same name, the
.tex needs no change; the old one stays in `../review_2026-10-23/`).

---

## Added 30 Oct: `fig_s8_laps` (replaces the four Gazebo screenshots of Figure 22, fig:res_s8)

**One message.** The team shrinks from eight to four by releases on the move and grows back to eight, the ring on its
path.

**What is drawn.** Four panels, one fixed orthographic camera (azimuth -62 deg, 32 deg up), white: the ring black (far
half grey), its plates as dots, each UAV a disc coloured by its role (carrying blue, departed a hollow grey disc on a dashed cable; the roles come
from events.csv's JOINT_ATTACHED / JOINT_DETACHED, since run.csv's `d<i>_attached` is all zero in this run), each cable
as the state log has it (the departed ones hang free), the ring's reference path (figure-eight, hover height) light
grey, a plate hollow once its carrier has left, a 1 m scale bar; the panel label gives the count, the lap and the run
time. One framed legend below.

**Data.** R2779 (`s8_chain4back_fig8laps_R50_L100_it5`, exploratory, lab batch 18, week7 63b504f; the run the draft's
comment names). Times (run clock): 44 s (lap 1, 8 carrying), 100 s (lap 4, 6 carrying, 2 departed), 140 s (lap 5, 4
carrying, 4 departed), 340 s (lap 9, 8 carrying after the four rejoins). The screenshots' own moments are unknown (no
script made them); these are mid-lap moments in the same four team sizes.

**Draft caption.** Eight carriers on the figure-eight in simulation, lap by lap, one exploratory run, rebuilt from the
logged poses: (a) eight carrying, (b) six after two releases, (c) four after four releases, (d) eight again after four
rejoins. One camera for all four panels: carriers blue, departed UAVs hollow grey on their free cables, a hollow plate
where a carrier has left, the ring black (its far half grey), the ring's reference path light grey.

**For the text agent.** The figure is one PDF: replace the four-minipage block (results_discussion.tex, the
`shot_s8_fig8laps_1-4.png` includes) with `\includegraphics[width=\textwidth]{Images/results/fig_s8_laps.pdf}`, label
`fig:res_s8` unchanged. Also in `Source/Results.tex` (not compiled in the 30 Oct main.tex).

**The loop and the critic.** Round 1: the tool's first render (every UAV a disc, grey for departed). Blind critic
(PNG and caption only), taken: a departed UAV now a hollow grey disc on a dashed cable (grey had meant four things), a
plate hollow once its carrier has left (and in the legend), the lap in each panel's name; the one departed UAV that
sits over the ring in (c) is 1.3 m from its centre, an overlap of the view only, and reads as off the ring once
hollow. Rejected: a floor grid and shadows (the brief asked for clean white), the ring's actual path in colour (the
figure shows the team, not tracking; fig_traj does that), dropping the scale bar (the view is orthographic, so a
horizontal bar is true; the ring is 1.0 m across), dropping the legend frame (Wesley's style). The path shows two or
three loops because the held path shifts at each weld (the "walked about 1 m" of the harder cases).

**The tool.** `tools/snapshot_render.py`: `Run(run_dir)` (run.csv, events, params.json, and the state log's rods when
recorded), `Camera(azim, elev)` (orthographic), `draw_snapshot(ax, run, t, cam)`, `roles_at`, `extent`,
`reference_path`, `scale_bar`; CLI `python3 tools/snapshot_render.py <run_dir> --times 44,100 --out <stem>`. Without a
state log a carrying cable is drawn to its plate and a free one straight down. `fig_rejoin` uses it too.

**Screenshot-type images in the 30 Oct draft** (every raster `\includegraphics` reached from main.tex):
`Images/results/shot_s8_fig8laps_1-4.png` (Gazebo, replaced here); `Images/platform_full.jpg`,
`Images/platform_closeup.jpg` (photos of the real platform, left alone); `Images/UNSW.jpg`, `Images/signature.png`
(title page). Not compiled (Project_preparations.tex is not input): `Images/Screenshot 2026-04-20 at 11.35.37 AM.png`,
`Images/formation_error.png`, `Images/world_cables.png`; `Images/trajectories_3d.png` is unused. No other simulation
screenshot is in the draft, so nothing else to render.

---

## Added 30 Oct: `fig_summary` (redesign of Figure 12, fig:res_summary; same file name)

**One message.** The real world tracks several times worse than simulation on every trajectory (hover 15-26 times,
figure-eight 1.7-4.2 times, spinny line 8.2-8.5 times), and the team size changes little, apart from the pair on the paths.

**What is drawn.** Three panels, one per main trajectory: (a) hover, the position error; (b) figure-eight and (c)
spinny line, the clock-free path error. Grouped bars for 2, 3, 4 carriers: real world, earlier setup (red) beside
simulation, the same ring (blue); each bar's value on it; linear y, each panel its own scale; a grey dash where no
real-world four-carrier spinny line flew; one framed legend below. The circle, line and spin go to the appendix.

**Numbers** (computed by the script with `best_data.score`, a cell the mean over its runs; they match the 31 Oct
tables tab:res_hover, tab:res_fig8, tab:res_spin exactly):

| mm | hover 2 / 3 / 4 | figure-eight 2 / 3 / 4 | spinny line 2 / 3 / 4 |
|---|---|---|---|
| real world, earlier | 25.6 / 21.8 / 21.5 | 42.1 / 19.0 / 17.4* | 56.9 / 36.4 / -- |
| simulation, 0.86 kg | 1.0 / 1.0 / 1.4 | 10.0 / 9.1 / 10.1 | 6.9 / 4.3 / 4.1 |

Runs: real world hover RIG-1028-hS2, RIG-1022-h3, RIG-1007-r20004; figure-eight RIG-1028-f2a/-f2b/-f2c, RIG-1022-f3h,
RIG-1007-r2008e60 (*half speed, 7 Oct tree); spinny line RIG-1028-ls2a/-ls2b_4, RIG-1022-ls3c. Simulation R2330/31,
R2337/38, R2349/50; R2334/35, R2341/42, R2353/54; R2362/63, R2345/46, R2357/58.

**Draft caption.** Path error by trajectory and team size, the earlier real-world setup (red; 0.55\,m cables,
0.86\,kg ring) beside simulation (blue; 1\,m links, the same ring): (a) hover, the position error; (b) the figure-eight
and (c) the spinny line, the clock-free path error. Each bar is the mean over its runs. *At half the speed, on an
earlier code tree. --: no four-carrier spinny line flew in the real world.

**The loop and the critic.** Round 1: grouped bars, value labels, linear y per panel. Blind critic (PNG and caption
only), taken: each run as a dot on its bar (the bars are means over 1-3 runs), the half-speed earlier-tree cell in a
light red, "not flown" written in the empty slot, the cable lengths in the legend (0.55 m real against 1 m
simulation: the comparison is not one variable). Also its warning on the message, kept in the wording: the team size
changes little in simulation and for three and four real carriers, but the real pair is the worst on both paths
(42 and 57 mm), and on the figure-eight with three and four carriers the real world is only 1.7-2.1 times the
simulation. Rejected: one shared log axis or a ratio plot (Wesley asked for linear bars per panel), y labels on (b) and
(c) (the panel names give the metric, the unit is the same), dropping the legend frame and the top/right spines
(Wesley's style).

**Files.** `fig_summary.{py,pdf,png}`; the PDF copied over the draft's `Images/results/fig_summary.pdf` (same name and
label; the old two designs stay in `../review_2026-10-23_best/`).
