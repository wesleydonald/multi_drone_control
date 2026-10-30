# Morning report, Sat 24 Oct 2026 (the overnight loop, 23 Oct 22:45 to 24 Oct 10:00)

**Verdict:** Quan et al.'s law, as their text gives it, now carries the ring in the twin (hover at the right height,
reproducibly; lines with an end overshoot), after two twin faults of ours were found and removed. The rig planner stays at
5 iterations for Thursday: the 2-iteration case did not survive its reviewer (below). Nothing is committed; the Quan code sits uncommitted in
the main working copy, every switch off by default, gate green.

## Rig (Wed 28 / Thu 29)
| What | Result | Why it matters |
|---|---|---|
| Planner real-time limit (bench B) | A PROJECTION (idle bench x 3.5), an upper bound: 3 quads with every tick at 5 full iterations, 4 at 2, 4 at 1. Row S-BENCHB-1024 (+ correction) | Not a flight limit: the rig flew 4 quads at 5 iterations on 7-8 Oct under the 60 ms budget cut (solve p95 51-78 ms, at most 1.4 % of ticks late). No change needed for Thursday |
| 2 iterations in the twin | The S card's C bars, written for 1 iteration, all hold at 2 in both repeats of hover, figure-8 and release (R2796-R2801). Reviewer: WEAK as rig evidence | Bench B ran with plan_fast_refs on (off on the rig; without it 4 quads at 2 iterations projects to 104.7 ms, over the bar), the twin at RTF ~0.25 never hit the 5-iteration budget cut, and no 3 -> 4 fold-in has flown at 2. Kept as a twin result only |
| Thu 29 rehearsal (from the evening) | R2738 no weld in 300 s; R2739 re-weld 1.1 s late; the clearance A/B says 0.25 is innocent | The wait is Tejen's install gate (his note); the sheet's 60 s GIVE UP rule covers it |

## Thesis sims
| Result | Runs | Verdict |
|---|---|---|
| **Quan et al. exactly (the paper's text law)** | offline; twin R2791-R2812 | Found the paper's full supplementary (S1-S9) hidden in its arXiv source. Built the law from the text behind `net_law quan` (critic x3). On the CoM-hub world: as first flown the fleet **climbed 0.45 m** (the rod's mass counted twice: the twin's drone_mass already holds it) and **wobbled at 2.6 Hz** (attitude gain 16). With both fixed it **holds the ring in hover at the right height, twice (110 / 118 mm, a settled sideways offset)**, and carries it along the lines with little lag but **overshoots the ends** (202 / 253 mm; at 0.4 m/s past our 300 mm band). Our planner on the same world: 1.0 / 24 / 65 mm; our port: 12 / 41 / 100 mm. Card: docs/experiments/2026-10-23_quan_exact.md "Results so far" |
| The paper's damping form (earlier losses) | R2744-R2749, R2780-R2781 | All flown at attitude gain 16: the wobble found tonight is a confound for those losses (say so beside them) |
| C, the network's rim pairs on the hub tree | R2782-R2786, R2790 | Against same-tree rim pairs every cell reads "the port" (hover rim 14.2 vs hub 12.2 mm; the frozen verdict, against 17.4, was "partly the geometry"; both reported) |
| S, 8 quads in the twin (5 it) | R2764-R2779, R2788-R2789 | Hover claim holds; the chain 8 -> 4 -> 8 misses back-at-8 position in both runs (6.36 / 6.35 vs 5.69 mm: the ring returns smoothly from 36 mm after the last fold-in but is still ~28 mm off when the window opens), reported as missed; the same-tree baseline reproduces. Reviewer: supported (strong) |
| S, real time | S-BENCHB-1024 | The 8-quad real-time claim is **falsified, as the card expected**: the projection (an upper bound) is 3 / 4 / 4 quads with every tick at 5 / 2 / 1 full iterations; 8 quads is twin only, not real time. Reviewer: supported (strong), wording corrected |
| Papers and thesis | (drafts) | Both RA-L drafts and the thesis's scalability and rejoin sections now cite the 8-quad claim runs (chain misses back-at-8 by 0.7 mm; releases one at a time), N_max as a projection; the chain figure is still the earlier 1-iteration run's (re-render owed). Paper v1 8 pp, v2 8 pp, thesis 152 pp, 0 undefined |

## Housekeeping
Backup round 1 is on OneDrive (15 archives, sizes checked) and its 408 uncited Aug-Sep sim runs are deleted; MATLAB and
three finished worktrees are removed (archive tags kept). Disk: 28 GB free (was 2.6 GB). Round 2 (1-14 Oct) is packed and
waits for your upload.

## Decisions for you
1. **2 SQP iterations on the rig: not for Thursday** (recommended). The reviewer found the case weak (it also needs plan_fast_refs on the rig; no 3 -> 4 fold-in at 2 iterations; the rig already flies 4 quads at 5 with the budget cut). If you want it later: the rig's measured tick at n = 4 on the current tree, then a twin 3 -> 4 fold-in at 2 with fast refs.
2. **Quan, the readings to keep** (each a §3.1 row, your word): (a) `vel_k_att` 8 for the law's arm (the stand-in autopilot's gain, not the law); (b) the vertical: rod 0.055 with the seeded constant (flown, works) or the critic's one-line split of the modelled pull (same physics, no seed; not built); (c) R6b's integral: not flown, its critic says fix first and it was not asked for. Recommended: (a) yes, (b) the split, (c) record and delete.
3. **Quan, the bars** (§5): freeze, then the cells x2. The smoke runs did not set them.
4. **Quan, the line overshoot** (amplitude 1.3-1.4): leave it as the result, or test its cause (R5's expected acceleration, R3's leader, the gains)?
5. **Quan, R4's conflict**: init.m puts each robot 1.0 m from the payload (h_c 3.41 cable lengths); Fig. 5 / S6 call 2.02 m the mean cable. Which do we cite?
6. **The Quan code** is in the main working copy (uncommitted, switches off, gate 1953 green): your commit, or back to its branch.
7. **Backup round 2**: upload the 12 `results_2026-10-*.tar.zst` from `~/mdc_onedrive_upload`; then the delete script (~15 GB).
8. **S's chain:** report it as missed as written (what the card, papers and thesis now say; recommended), or set a later N4 window for future runs only.
9. Still open from yesterday: Tejen's note; case 6's keep-out and clearance; merging `week7`.

Details: `docs/SESSION_STATE.md` (overnight block), the cards' Results sections, `results/registry.csv` R2738-R2812.
