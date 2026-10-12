# Lane 6, 12 October 2026: hand-back

Five tasks from the 12:57 brief, in order. Times are from `date`. Nothing merged, nothing pushed; commits are authored
by Wesley with no trailers; the main tree, week5, the rig sheets, the lab PC, the registry, the experiment cards of
others and `~/thesis` were not touched (reading only). No Gazebo, SIL or rig stack ran.

## Branches and worktrees

| branch | worktree | from | what |
|---|---|---|---|
| `thesis-rig` | `/home/wesley/mdc_wt/thesis` | (the thesis branch) | task 1: merges a612d22 (thesis-figs), ea4036b (thesis-cites), f887835 (thesis-ch4); c176b3d the four figures in the skeleton; task 2: 71a9fdd T7, T8a, T8b filled; this note |
| `wed-pipeline` | `/home/wesley/mdc_wt/wed-pipeline` | week5 4573cd4 | task 3: `tools/rig_night.py` (e5d3f59) |
| `counter-swing` | `/home/wesley/mdc_wt/counter-swing` | week5 4573cd4 | task 4: `tools/counter_swing.py`, `docs/counter_swing_2026-10-12.md`, `results/analysis/counter_swing_2026-10-12/` (2e836f6) |
| `tier-a-card` | `/home/wesley/mdc_wt/tier-a-card` | week5 4573cd4 | task 5b: `docs/experiments/2026-10-12_tier_a_crossover.md` (b8bb26d rev 1; ff56be7 rev 2 with the critic's challenge) |

## 1. Thesis assembled (done 13:02)

thesis-figs, thesis-cites and thesis-ch4 merged into thesis-rig with no conflicts. The skeleton's `\pendingfig`
placeholders for F8, F10, F11 and F14 are the finished figures, captions from each `docs/figures/<ID>/caption.md`,
labels `fig:F8`, `fig:F10`, `fig:F11`, `fig:F14`, none defined twice. The four PDFs are copied into
`docs/thesis_drafts/thesis_v2/Images/` and committed (the directory is git-ignored: `git add -f`, as the working rules
say for docs/). Build: 0 errors, 0 undefined references or citations.

| build | pages |
|---|---|
| after the figures (c176b3d) | 121 |
| after the tables (71a9fdd) | 122 |

## 2. The reviewer on the P6 numbers, and the tables (done 13:28)

The reviewer (reviewer agent, read-only, recomputed with `release_bands.py` and with its own code) returned **weak**
on the table as first drafted. Every T8 cell for L1, L4-L7 and the twin pairs reproduces exactly; the corrections are
what changed the draft:

| cell or claim | as drafted | what the reviewer found | what the skeleton now says |
|---|---|---|---|
| L2 (r0013) and L3b (r200006 slip 2) Delta-theta and peak | 68.3 / 68.6 and 63.2 / 65.5 as values | the log ends at the fleet abort 0.43-0.47 s after the event with the tilt still rising: lower bounds | $\geq$68.3, $\geq$68.6, $\geq$63.2, $\geq$65.5, caption says why |
| "every carried-through release lies above the hover floor except L5 and L1" | proposed sentence | not supported: L7 sits inside the path null (8.02 against p95 8.32); L1 is above the raw hover p95 but inside once scaled | the floor paragraph after T8b: L4 and L6 above every window; L5 inside the raw hover null, L7 inside the path null; scaled, L1 and L7 do not separate, L5 does |
| L1 (r0012) dip -2.7 cm | a dip | the ring rose 15 cm, never below its pre-mean | "none" |
| L1 peak 5.5 | the detach's peak | came 0.1 s before LAND; the first second peaked at 4.6 | 5.5 in the cell, both in the paragraph |
| L5 twin pair R1140, R1145 | n = 2 | the two runs carry different uncommitted diffs on 0e91d86, 62 commits behind the other pairs' tree | kept with a comment; needs a footnote if quoted as a pair |
| T7 K2 (r206e60) 2.91 cm / 0.20 s | registry | more than 10 % off on the window the card used (+4 s to LAND): 2.63 cm / 0.17 s | 2.6 / 0.17 |
| T7 lag column | registry values of three different kinds | the registry's 0.45 and the figure's 0.35 (r006) are different windows and definitions | one definition for all four rows: xy error mean and max from +4 s into the path to LAND, lag the best time shift of the reference |
| T7 K1 vs K2, K5 vs K6 | the pull source | six parameters differ (pull source, planned and hand-over elevation, the pull cap 6 to 12, land_unwind, delay_comp_s) | said in the caption: two configurations |

T7 now (one definition, one run each; the K3, K4, K7 cells wait for Wednesday):

| row | flight | xy error mean / max (cm) | lag (s) | height sd (cm) |
|---|---|---|---|---|
| K1 | r006, circle, planned 45 | 7.9 / 16.9 | 0.47 | 4.9 |
| K2 | r206e60, circle, measured 60 | 2.6 / 5.5 | 0.17 | 0.5 |
| K5 | r0007, figure-eight, planned 45 | 8.2 / 19.9 | 0.37 | 4.8 |
| K6 | r2008e60, figure-eight, measured 60 | 2.9 / 6.6 | 0.19 | 0.6 |

The R3 falsifier stays frozen on 2.91 cm (the card's reading); the skeleton says the same flight reads 2.6 on the
table's window, inside the 1 cm tolerance.

T8b now (L1-L7; the columns are detection, Delta-theta, peak, dip, back in band, landing lean, survivor step, twin):

| row | flight | det (s) | Delta-theta | peak | dip (cm) | band (s) | lean | step (N) | twin range, n |
|---|---|---|---|---|---|---|---|---|---|
| L1 | r0012 | -- | 6.1 | 5.5 | none | n/s | 3.5 | +1.2 | -- |
| L2 | r0013 | -- | >=68.3 | >=68.6 | 21.7 | -- | capsized | +2.8 | -- |
| L3a | r200006 slip 1 | -- | 30.9 | 30.6 | 12.5 | -- | -- | none (3.0 per rod) | -- |
| L3b | r200006 slip 2 | -- | >=63.2 | >=65.5 | 68.1 | -- | capsized | none | -- |
| L4 | r302_3 | -- | 11.2 | 15.4 | 6.9 | n/s | 2.5 | +1.2 | 6.0-7.1, 2 |
| L5 | r303_4 | 0.42 | 5.3 | 6.5 | 3.1 | n/s | 3.0 | +1.2 | 6.0-6.4, 2 |
| L6 | r304_3 | 0.33 | 16.9 | 17.8 | 5.3 | n/s | 3.3 | +1.7 | 14.6-17.4, 2 |
| L7 | r305 | 0.34 | 8.0 | 8.1 | 3.7 | 3.9 | 4.0 | +1.1 | 4.0-5.4, 2 |

Run ids are in LaTeX comments only. Routine calls made here (say if you want them otherwise): L3 split into L3a and
L3b; "Survivor pull (N)" renamed "Survivor step (N)" (the largest planned tension step handed to a survivor in the
first second of the resized plan, P6's column) because that is what the data has; layouts in today's clockwise plate
numbering; "n/s" for back-in-band rows LAND cut short.

What the reviewer could not verify: the slip onset times (from the bags; it took the P6 rows), the 8 Oct flights' tree
(no SHA in the manifests), and the card script behind the registry's 8.25 / 2.91. It also noted that the rig run
directories `r006_run`, `r0013_run`, `r200006_run` lived only in the thesis-figs and p6-offline worktrees: the five
7 Oct flights the figures and T7 cite (r006, r0007, r2008e60, r0013, r200006) were converted with their bags into the
thesis worktree's `results/rig/2026-10-07/` at 13:36 (untracked, `results/` is ignored; 3-5 s each, ROS sourced from
the week5 install, no node started), so F8, F10, F11 and the T7 recomputation regenerate from that tree.

## 3. The Wednesday-night pipeline (done 13:25, branch wed-pipeline)

`tools/rig_night.py`: one command per flight, the sheet's labels R1-R9 and the flight's name under
`results/rig/<date>/`.

```bash
cd ~/mdc_wt/week5   # or wherever wed-pipeline is merged; ROS sourced for the bag
python3 tools/rig_night.py --date 2026-10-14 R1:r1 R2:r2 R3:r3 R4:r4 R5:r5 R6:r6 R7:r7 R8:r8 R9:r9
```

Per flight it converts the bag with `rig_to_run.py` when `<name>_run` is missing, then by label:

| label | thesis target | what it computes |
|---|---|---|
| R1 | T7 K7 | hover xy error mean / max, height sd, tilt, pack voltage per drone (first, last, min) |
| R2, R3 | T7 K3, K4 | per lap xy error mean / max, along-track lag, height sd, tilt; the lap-to-lap sd; the T7 window (+4 s to LAND, lag by best shift); R3's frozen falsifier against 2.91 cm with max(1 cm, R2's lap-to-lap sd) |
| R4, R5 | T8 L8, L9; T13; F16 | T6 metrics to LAND, cut at 7 s and 15 s (freeze 1.1 via `release_bands.py`), detection, dip, landing lean (survivors, 1.2 s after touchdown), the survivors' planned pull before and after, Delta-theta against the no-event floor (P6 5a), and the ledger: hit / width / log ratio against the frozen W4 or W5 band and the two naive predictors ("rig = twin", "rig = previous rig flight") |
| R6-R8 | F15 | the console timeline (his manager's `ATTACH:` / `KINEMATIC ATTACH`, our `ATTACH commanded`, `TAKE-BACK`, `mux:`, `RELEASE GUARD`, `disarmed alone`), every message on the bag's attach topics (`/magnet/contact`, `/magnet/attach_proof`, `/join_planner/*`, `/drone_3/mux_state`, `/partner/release`, `/tejen/*`), and tip / ring / plate 9 / QUAD4 traces at 50 Hz with the tip-to-plate distance |
| R9 | T8 (announced), F15 | both of the above |

Outputs: `results/analysis/rig_<date>/<LABEL>_<name>/{summary.md, metrics.json, timeline.csv, laps.csv | ledger.csv |
trace.csv | bag_events.csv}` and `rows.md` with the LaTeX row text for T7 / T8b / T13 (run ids in comments). Tested
on the 8 Oct bags and the 7 Oct circle (outputs in the scratchpad only, nothing written under results/):

| test | pipeline | P6 / reviewer |
|---|---|---|
| r303_4 as R4: Delta-theta, detection, dip, lean | 5.29, 0.418, 3.1, 3.0 | 5.29, 0.418, 3.1, 3.0 |
| r305 as R5: Delta-theta, detection, dip, back in band, lean | 8.02, 0.345, 3.7, 3.93, 4.0 | 8.02, 0.345, 3.7, 3.93, 4.0 |
| r302_3 as R9 (announced path): Delta-theta, dip, lean | 11.2, 6.9, 2.5 | 11.21, 6.9, 2.5 |
| r206e60 as R2: xy mean / max, lag, height sd (+4 s window) | 2.6 / 5.5, 0.17, 0.5 | 2.63 / 5.5, 0.17, 0.53 |

Known limits: the attach traces need ROS sourced and the bag (else the console timeline only, with a note); the
ledger marks a flight "NOT scored" unless its date is 14 Oct or later; the no-event floor and the frozen bands are
the 11-12 Oct numbers written into the script with their sources (they are frozen data, not tunables); the R3 verdict
uses R2's lap-to-lap sd when R2's output exists in the same output folder, else R3's own, and says which. It needs
`<name>.log` and `<name>_logs` to exist (as `rig_flight.sh` writes them).

## 4. The rig-only counter-swing (done 13:20, branch counter-swing)

`docs/counter_swing_2026-10-12.md` has the full note; `tools/counter_swing.py` makes its tables. In short:

| question | finding |
|---|---|
| what swings | the commanded thrust follows the measured vertical rod pull one tick later (lag 0-0.02 s, correlation 0.95-0.99 in every window); what is left (hover thrust + position loop) swings 2-4 times less |
| where it comes from | the rig's own ring-and-rods rocking: 1.45-1.65 Hz at 0.05-0.09 N and 0.5 deg of ring tilt in the steady 7 Oct circle with no event; 0.3-1.7 Hz before each 8 Oct release; the twin reads 0.00-0.01 N and 0.04-0.10 deg in the same windows |
| what the release does | excites that mode (ring tilt swing 2.3-3.7 deg, pull swing 0.10-0.38 N after the plan arrives) and it decays over 2-5 s; nothing grows |
| the "against the step" at the arrival | the measurement's phase at that instant (the ring still falling toward its dip): the pull's own step explains 0.6-0.9 of the thrust step, the reference change 0.2-0.6 N; in the twin the same reference change comes with a quiet measurement |
| not it | the double count (P6 3 stands), the resize timing, the reference drop alone (the twin has it without the swing), the rate latency (attitude loop) |

Proposed fix, Wesley's call, no code written: a first-order low-pass on the measured offset `(ac_cond - ref_cable[0])`
in the tracker, tau about 0.3 s, as one new parameter default off. The offline replay says it removes most of the
swing and passes 61-82 % of a release step within 1 s. The twin cannot test it (no rocking); the test is a rig
one-variable pair on the carry circle (R2's line, filter on against off) before any detach pair. Second option: a
slew on the offset instead of a band. Third: leave the planner alone (it is not the driver).

## 5a. T6 amendments: a proposal (not applied; later runs only, the freeze's own rule)

Four texts for signature, each the recommended option first. None re-scores a run already seen; the 8 Oct rows keep
their frozen readings.

| # | what | amended text (recommended) | alternative | why |
|---|---|---|---|---|
| A1 | time back in band on a short hold | when LAND comes less than 10 s after the event, theta_post is the mean over [LAND - 2 s, LAND), the band max(2 sigma_pre, 0.5 deg), the hold 0.5 s, and the cell is marked "short hold"; the 10 s rule stays for the unmarked value | leave "n/s" (today) | 7 of 8 rig rows read n/s; the sheet's 15-20 s holds fix it for Wednesday, the record before it stays empty otherwise |
| A2 | event time of an uncommanded release | the slip-detector onset in the bag (`drop_trigger_replay.release_marks`: 3 mm off the 1 s baseline on 6 samples, confirmed by rod error > 3 cm within 1.5 s); window end LAND, else the fleet abort or the last ring pose, the cell marked ">=" when the tilt is still rising there | report the row with no Delta-theta | the record needs the capsizes in the same column, labelled |
| A3 | Delta-theta against the no-event floor | beside Delta-theta report Delta-theta / sigma_pre; a release "separates from the floor" when Delta-theta exceeds the null's 95th percentile for its window (hover 7 s 6.0 deg; path 15 s 8.3 deg) and Delta-theta / sigma_pre exceeds the null ratio's 95th percentile (4.06 hover, 3.87 path); the words "separates" / "does not separate" are the only claim made for a single run | the mean floor (4.8 / 6.3) as a line in F16 | the floor is three times the rocking because the statistic is a 2-D maximum; a 5 deg rig value is not an event by itself |
| A4 | the resolution rule's spread at n = 3 | condition 3 becomes abs(mean B - mean A) >= max(1 deg, 2 x the larger sample sd); condition 2 (ranges apart) stays | keep the range and fly n = 2 | with the range the bar grows from n = 2 to 3 (MDE 3.7 to 4.6 deg on the hover null); with the sd it does not; condition 2 still guards against overlap |

A1-A3 touch only `release_bands.py` and `release_transient.py` (the scorers); A4 touches the rule text and the
pair scorer. The hover null behind A3 rests on two 7 Oct carry flights; a 3-min rig hover (sheet R1) would give it
25 more windows at 7 s and the first ones at 15 s, if R1 is converted with its bag.

## 5b. The Tier A card (branch tier-a-card)

`docs/experiments/2026-10-12_tier_a_crossover.md`: the crossover 2x2 at 120 deg, {measured, planned} x {announced soft
v2, unannounced}, four arms from the W4 base config on one tree, n = 3, 12 lab runs; the frozen signs T2, T3, T4 and
the resolution rule as the bars, written before any config; no flight code; one analysis item (the soft-detach event
time, the planner's UNLOAD line as an event, not coded yet). The critic's challenge is under "## Critic" in the card.

**Critic, round 1: rewrite the card.** Its point: on the measured pull the leaver's tracker flies its measured rod pull
at node 0, so the planner's 3 s unload does not unload the leaver and the release gate passes on the planner's own
tension (the 7 Oct soft-detach card's must-fix 1, after which the measured soft arms were dropped; no measured soft arm
has ever flown). A T2 "confirmed" could then be a three-times-larger release step, not the measurement lag. Rev 2 took
every text and analysis fix: the leaver's pull at the magnet opening is reported and a 50 % rule labels A3 "not a
ramped release" (T2 and T4 then reported, not confirmed); Delta-theta split at the opening and the detection delay per
run; an A2 drop counts as T3 support; the config times and LAND; 4 lab slots, evenings, behind the batches you ordered
(decisions 178, 182), from Sat 17 (P11); the UNLOAD analysis edit guarded byte-for-byte against the frozen W4/W5
scores; P13's re-fly whatever the outcome. The critic also expects T2 to come out unresolved at the twin's measured-arm
spread (R1217/R1218 read 3.4 and 7.0 deg; the bar would be 7.2 deg against an A4 near 2.5), while T3 can resolve.

## Open questions for Wesley (recommended first)

1. **T7's definition.** Keep the one-definition rows (xy mean / max from +4 s into the path to LAND, lag by best time
   shift; K2 reads 2.6 / 0.17) or restore the registry's mixed numbers (K2 2.91 / 0.20)? Recommended: keep one
   definition; the falsifier stays frozen on 2.91 and the text says so.
2. **The L3a / L3b rows and the ">=" rows in T8.** Keep the uncommanded releases in T8 with their labels (the plan's
   choice), or move the capsizes to a separate "lost" table? Recommended: keep them, labelled as now.
3. **The counter-swing filter.** Approve a rig carry pair (filter on against off) for the next free rig slot after
   Wednesday's lines? Recommended: yes, after R1-R5 are in; it is a tracker parameter default-off, so it needs your word
   as a flight-loop change and the critic before it flies.
4. **T6 amendments A1-A4.** Sign any of them for runs after Wednesday? Recommended: A2 and A3 now (they change no
   score, only labels and a second column), A1 and A4 after Wednesday's holds are seen.
5. **The L5 twin pair footnote** (R1140 and R1145 on different uncommitted diffs): accept as the pair with a footnote,
   or re-fly the pair on week5 (2 runs)? Recommended: re-fly with the Tier A batch (same tree), 2 more runs.
6. **The pipeline's home.** Merge wed-pipeline into week5 before Wednesday night so the rows come out of the flight
   tree? Recommended: yes; it adds one file and touches nothing that flies.
7. **Tier A, A3.** A true ramped release on the measured pull needs the leaver's tracker to fly the planned, unloading
   term during an unload (flight code, node 0 on `mocap`): your word, and the critic again if so. Options: (a) fly the
   batch as rev 2 says, A3 labelled by the leaver's pull at the opening; (b) fly T3 alone first (A1, A2: 6 runs), the
   pair that can resolve and the one the C2-B sentence rests on; (c) make the code change first. Recommended: (b), then
   decide A3 from what the 7 Oct must-fix needs.

## Decisions for the record (routine calls made here)

- T8: "Survivor pull" became "Survivor step" (P6's largest planned step in the first second); L3 split into L3a / L3b.
- T7: one definition for the four 7 Oct rows (the reviewer's recommendation), the registry values in comments.
- rig_night.py: Delta-theta for the ledger is cut at event + 15 s (freeze Q1); the R3 verdict uses R2's lap sd.

## Twin runs wanted (asked, not run)

- Tier A crossover: 12 runs on the week5 candidate (the card), when the lab PC is free; plus 2 to replace the L5 pair.

## Script for Wesley (deletions are refused here)

```bash
# lane 5 and 6 branches whose work is merged (thesis-figs, thesis-cites, thesis-ch4 are in thesis-rig; wed-review is in week5)
for b in thesis-figs thesis-cites thesis-ch4 wed-review thesis-work3; do
  git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/$b 2>/dev/null
  git -C /home/wesley/multi_drone_control branch -D $b
done
# after merging or declining today's three: wed-pipeline -> week5, counter-swing -> week5, tier-a-card -> week5
for b in wed-pipeline counter-swing tier-a-card; do
  git -C /home/wesley/multi_drone_control worktree remove /home/wesley/mdc_wt/$b
  git -C /home/wesley/multi_drone_control branch -D $b
done
# the 7 Oct rig conversions in the thesis-figs and p6-offline worktrees are untracked and go with them; the thesis
# worktree (mdc_wt/thesis) now holds its own copies of r006, r0007, r2008e60, r0013 and r200006 (also untracked)
```
