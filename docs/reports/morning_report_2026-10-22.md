# Morning report, Thu 22 Oct 2026 (overnight 21-22 Oct, laptop)

Runs R2259-R2322 (void ones marked in the registry). Everything is on `day-1021`, flight code as of 367c98f, nothing
pushed. The reel is `results/videos/reel_2026-10-22_overnight.mp4` (41 runs; done 06:55). The basket and floor-join
configs (R2306, R2308, R2310, R2322) carry no recording block, so they have no video: to add before the next ones.

## For the rig today (read first)

| What | Result | What you do |
|---|---|---|
| **The rig tree.** `day-1021` now carries the magnet-marker check rev 3 (a switch, off by default), the trackers' cable-pull cap at 8 N (`cable_accel_cap` 14.5, was 6.6 N), and the trackers' raw measured pull in their logs. | `tools/gate.sh --quick` passed after the merge and the cap (1512 tests). Later fixes touched only the twin's marker emulation and the check's frozen-ring rule; the planner suite passes. | Note `git log --oneline -1` on the sheet. |
| **Part C (the T to the pair): which release check.** Twin comparison on the rig's own Part C files, R2292-R2299. | **Check ON:** caught both T-to-pair releases ~0.25 s after the OFF and resized to the pair, no drop. No false fire in hover, through three 12 deg ring-attitude glitches, on the A/B paths, or on 1 m cables; no extra planner load. **Stretch check alone (R3off):** missed every such release (4 of 4 tonight). The planner flew the T model to LAND, so expect "no detection: LAND". **One frozen bar missed:** after the resize the pair rocked 3.6-3.7 deg p-p against 3.0-3.3 in the off arm, which never resized. | **Your call** (sheet, Part C status line). Recommended: **R3on**, if the A/B replay after Part B is clean (gate 3: `WOULD-FIRES: 0`, `RUNS OVER 50 ms: 0`, every drone armed, each magnet's offset within 1 cm across flights). |
| **What Part C should look like with R3on** (twin, int_k_xy 0). | The T hangs ~2.3 deg (bench item 0 matters: with the rig's attach_z gap it was 27). The release is caught in ~0.25 s in the twin; expect ~0.3 s or more on the rig, since the coils open 0.16-0.20 s after OFF. The pair then rocks ~4 deg p-p. | Watch for the "armed" lines: QUAD2's marked "rider", each offset within 1 cm of the A/B replay's. |
| **Critic round 3's must-fixes** (N1-N4). | **N1:** our rig bags (17 airborne marker streams) hold no marker swap that stays inside the body-length band for 0.1 s (two of its rules were set after seeing the data; card). **N2:** built (the replay's run and band report). **N3:** done (the rig prediction). **N4:** done (the sheet's ON procedure). | - |

## Twin, overnight

| What | Runs | Result | Verdict |
|---|---|---|---|
| **The nine baselines on 1 m cables** (thesis B, the twin half) | R2259-R2270, R2282-R2286 | Paths: 2 quads 15-18 mm, 3 quads 16-18 mm, 4 quads 21-23 mm (hover 9-12 mm). Tilt peaks 2.3-5.7 deg; no rocking grows; measured pulls at most 5.4 N. | pass (exploratory) |
| **A/B in the twin with the check on** | R2300-R2305 | every marker armed; nothing detected or would-fire | the twin side of the replay gate is clean |
| **The check's neighbours** | R2314-R2321 | **On 1 m cables:** T to the pair in hover and on the figure-8 mid-lap (the chain's thesis step, first flight): caught with the check, the pair finished the lap; missed without it. **Square to T (a loaded quad leaves):** caught as fast by either check (0.21 s) and dropped at 45 deg either way. **Plate-0 release (a 270 deg gap):** drops as designed; with the check at detection, without it only at 51 deg. | the check adds the zero-share release; the loaded release's transient is the open problem (step 2b) |
| **Detach on 1 m cables** (square to T) | R2309, R2311 (hard), R2312, R2313 (soft) | **Hard** (resize and release at once): dropped at ~46 deg, in hover and mid figure-8. **Soft** (`detach_unload_s` 3, the leaver's tension ramped off first): no drop, peaks 22.9 deg (hover) and 25.9 deg (figure-8), and the T finishes the lap. | the announced soft detach is safe at 1 m |
| **Tejen's basket: pickup, the drop into the ring's net, his rejoin** | R2306 (0.55 m, hover), R2322 (1 m, hover), R2310 (1 m, circle) | **Hover:** the whole mission works at both cable lengths. 1 m: pickup 39.9 s, drop 112.5 s, contact 126.3 s, weld 128.3 s, fold-in. Both landings then ended in the twin's welded-QUAD4 tip-over (TB14), an envelope fault ~7 s into LAND. **1 m circle:** pickup and drop work, but his rejoin sat in MATCH_VELOCITY for 330 s and never made contact. Our take-back landed QUAD4. His rod passed within 6 mm of a carrier's rod. | mission works in hover; the moving-ring rejoin at 1 m needs Tejen's work |
| **His floor join on 1 m cables** (attach) | R2308 | contact, weld, fold-in and landing as on 15 Oct | pass |
| **Solve time against fleet size** (the scalability study) | `results/analysis/solve_scaling_tuf_2026-10-22.json` | Tick p95 within the rig's 60 ms: up to 4 quads at 5 SQP iterations (48.8 ms), 5 at 2 (55.2), 6 at 1 (59.9). 9 quads take 135 ms at 1 iteration. The first sweep was invalid: it never kept its warm start; fixed and re-run. | 9 quads at 10 Hz needs ~2.2x faster solves, or 5 Hz |

**Harness fixes and my errors tonight:**
- The twin's magnet markers read the rod pose relative to its own model and never armed. Fixed to the world pose,
  plus a crash fixed in the same code.
- The twin's ring stream repeats a sample or two. The frozen-ring rule now needs 50 ms of repeats.
- The 1 m basket configs typed the 0.55 m ball spot.
- I stopped a batch by its wrapper's PID, so two Gazebo stacks ran at once. R2277-R2281 are void; there is a memory
  note on it.

## Decisions for you

1. **Part C's check:** R3on (recommended, if the A/B replay is clean) or R3off.
2. **Chain step 2a** (draft card `docs/experiments/2026-10-22_chain_step2a_vector_split.md`): a force-vector split
   so the T hangs level. Recommended: review after the rig; the critic before any code.
3. **Step 2b, the loaded release's transient:** an unannounced loss of a loaded quad rolls the ring past 45 deg
   whatever the cue (57-84 deg peaks). The soft detach survives because it ramps the leaver's share off, which an
   unannounced release cannot. So the survivors must take the new split faster. Recommended: card it next.
4. **For Tejen:**
   - the 1 m rejoin on a moving ring (stuck in MATCH_VELOCITY);
   - his rod's 6 mm pass by a carrier's rod;
   - the drop and rejoin on a hovering 1 m ring work.
5. **TB14 on the basket landings:** a twin rigid-rod effect, but watch QUAD4's landing on the rig.
6. **Scalability:** next, the plan quality at 1-2 iterations against 5. That needs a card before it means anything.
7. **Merge `day-1021`** when you are ready.
