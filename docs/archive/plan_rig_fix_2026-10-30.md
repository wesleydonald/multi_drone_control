# Plan: fix what went wrong on the 29 Oct rig day, fly for results on Fri 30 Oct

**Status 19:35: queue done** (W1-W5 done, W4b added and done, W6 the morning report; the scorer gaps deferred).
Results: docs/reports/morning_report_2026-10-30.md. Written Thu 29 Oct 18:37 for Wesley (away until ~23:30). Worked by
the loop tonight; status per item in
`docs/SESSION_STATE.md` (block "FRI 30 OCT PREP"). Sits under the current plan (`docs/plan_thesis_results_2026-10-21.md`):
its rig results B (baselines), Nr (4->3->4 with Tejen's re-attach) and K (pickup and basket). Results freeze 7 Nov, so
after tomorrow only ~2-3 lab days remain.

## What went wrong on 29 Oct, and the fix
| # | What went wrong | Cause | Fix | Status |
|---|---|---|---|---|
| 1 | Plate 0 released (h4, ch1, ch1b) | QUAD1's weaker magnet (Wesley) | replacement magnet; until then QUAD1 carries the least load | hardware |
| 2 | The ring jittered at 5.3 Hz | the ring bends; the trackers fed it | 5.3 Hz notch, rig default | flown on 3 quads only: **fly h4 with it first** |
| 3 | The T could not lift the ring (sq1) | the T's loaded pair at the throttle cap on 1.15 kg | the triangle 3/11/7 + throttle_max 0.95 | works for 3; **2-quad and chain layouts: W1** |
| 4 | Commands typed wrong or missing (the 0.30 m approach, the floor, refused args) | the sheet | every sheet line desk-checked tonight | **W5** |
| 5 | His hand-over never granted (sq3), waited under my floor (sq4_5) | stock bounds; no margin | rig bounds 0.12/0.30; the launch refuses a floor within 0.10 m | done 29 Oct |
| 6 | 6 of 6 descents aborted (sq5_2) | the magnet swings ~4 cm round its target; his 8 cm abort trips on one tick | Codex's 0.3 s debounce (behind a switch) | **W2, W3, W4** |
| 7 | The pickup never went down (k1_4) | capture 2.5 cm / 0.07 m/s | typed 6 cm / 0.15 m/s (Codex: a fair provisional trial) | typed, never flown: **W2 replay, W4 twin** |
| 8 | His timing fix | flips QUAD4 in the twin with the rig's delay | off the rig | done (decisions 17:20) |
| 9 | The ring moved 4.6 cm rms during his attach | our carriers' hover, or QUAD4 near the ring | find which first | **W2** |
| 10 | The ring rides ~3 cm low | height integral / thrust map | sheet margins; no fix tonight | noted |

## Work queue tonight (in order; each item finished well before the next)
- **W1. Formation thrust table** (offline, analysis agent). Each carrier's predicted hover throttle per layout (square
  0/3/6/9, triangle, even 0/4/8, the T, the pair 0/6) for the 1.15 kg and the 0.86 kg rings on 1 m cables, using the rig
  thrust map, checked against 29 Oct's measured throttles. Says which baselines and chain stages can fly tomorrow.
- **W2. Rig-log replays** (offline, the same agent):
  - (a) sq5_2's descents under a 0.3 s debounce: which aborts remain, and how long a descent needs to reach the plate;
  - (b) k1_4's hover under the 6 cm / 0.15 m/s / 0.5 s capture, xy and speed and dwell together (Codex);
  - (c) the ring's motion in the triangle before QUAD4 takes off against during its approach (sq2_5, sq5_2), and in the
    square hover (h4f).
- **W3. Descent debounce in his planner**, behind a switch, default off (Codex's spec, DECISIONS_1355 section 2):
  - 0.30 s of continuous fresh violation of the 8 cm position abort;
  - reset on recovery, a new phase or attempt, or stale evidence; the speed abort unchanged;
  - tests; logged in CHANGES_FROM_TEJEN.md; a typed launch argument.
- **W4. Twin check** (lab PC, exploratory, one or two runs each):
  - the debounce on the 60 ms twin (E1c's settings), where the rig's abort cycle shows: do the descents reach the bottom?
  - the pickup with the rig's capture values typed: does it go down?
- **W5. Tomorrow's sheet** `docs/rig/fri_2026-10-30.md`. Every command copy-paste ready and desk-checked:
  `RIG_FLIGHT_DRY_RUN=1`, then the launch brought up against `tools/fake_mocap.py` with no radios and its parameters read
  back (`tools/preflight.py`), so no refused argument or missing switch costs a battery. Ordered for results (below).
- **W6. If time remains:** the scorer gaps (`tools/attach_clearance.py`), the morning report.

## Tomorrow's order (results first; a line that fails moves on, it is not retried twice)
1. **h4n:** 4 quads, hover with the notch, 20 s. It gates the 4-quad lines.
2. **B, 4 quads:** ls4 (line_spin; never flown at 1 m), f4n (figure-8 with the notch; f4 flew the low-pass).
3. **B, 3 quads (even 0/4/8):** h3, f3, ls3 at 1 m on the 1.15 kg ring.
4. **Nr/K, Tejen's attach in the triangle:** sq6 (sq5_2's line plus the debounce if W4 holds); then the pickup k1_5
   (capture 6 cm / 0.15).
5. **B, 2 quads:** only if W1 says the pair is under the cap (else Wesley's call: the 0.86 kg ring, or skip).

## Decisions for Wesley (asked in the morning, not before)
- Typing the debounce on the rig (a gate in Tejen's code; Codex proposed it, Tejen has not approved it).
- The 2-quad baselines if the pair is at the cap: the 0.86 kg ring, or skip them.
- Send Tejen the twin note (`~/Downloads/twin-timing-20261029.md`).
- Commit the work since 284ffb4.

## Rules kept
- No launch default, world, `params.py` or gate threshold changed: new behaviour is a typed switch, off by default.
- At most 2 agents. Gazebo on the lab PC only (laptop idle). Bars before claims; W4 is exploratory.
- Every edit to Tejen's code is logged.
- Nothing is committed or pushed.
