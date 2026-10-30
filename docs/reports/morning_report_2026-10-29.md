# Morning report, Thu 29 Oct 2026 (the overnight loop, 28 Oct 22:30 to 29 Oct 07:30)

**Verdict:** Tejen's attach on the new ring works in simulation with his settings (first try, 38.6 s), and faster with
the whole approach low (0.30 m: 31-32 s), but the low approach needs his swing allowance cut to a value that does not
cover a real slack cable. The reviewer rated the night's case **weak**. The avoidance fix (the pickup's reattach on
his C++ planner) works since a bug fix at 06:12 (R2864, R2865: both welded, R2865 cleanly; not reviewed). The timing fix removes the shutdowns it
targets but fails its frozen bar on his other check, which also tripped in R2864. The far-side "route round" video was not achieved. Nothing committed.

## For today's lab (08:00-11:00), first
| What | Result | Why it matters | Sheet |
|---|---|---|---|
| sq1 is the first real-world flight of Tejen's C++ transit | His C++ transit has been on by default since 21 Oct and has not flown on the rig | Expect surprises in the transit, not only at MATCH | as written |
| His C++ planner can switch itself off for good | Two checks do it. (1) A timing race in our ring bridge (`start_skew`): latched in 4 of 17 runs, decided 1 (R2855). (2) His carrier-tracking check (`tracking_mismatch`, 5 cm): R2848, R2860, R2864 (the ring bobbing 6.6 cm after the ball dropped in) and 27 Oct's R2830/R2831. His own real-world configs turn (2) off; ours has it on. The critic thinks (2) is the likelier one on the rig (1 m slack cables, the T's lean) | If QUAD4 stops mid-transit and never moves on, his T2 line `Backend disabled ... blockers=` says which | read the blocker (new note in section 4) |
| The fix for (1), `commit_backdate:=0.04` | No `start_skew` in R2860, R2862, R2863 (none in 206 s of C++ transit in R2860). The critic: bounded, commands nothing, fly it if Tejen agrees (it feeds his gate). But it FAILS its frozen bar: R2860 had a (2) at the throttle cap. Reviewer: not supported | Without it, sq1 can lose the transit to the race (the critic's estimate: ~1 in 8 transits at the simulation's rate) | your word + Tejen's |
| A wider MATCH gate (xy 0.08) with a zero plate velocity | Rig replay: opens all 5 stalled match windows of six recorded attempts. Simulation: each alone welds twice; R2859 had one descent abort and a retry (cause not identified, reviewer) | Only if sq1 stalls in MATCH | your word (a gate) |
| The low approach (0.15-0.30 m above the ring) | Simulation only: needs his swing allowance at 0.02 rad (1.15 deg); the rig's slack cable swings ~3.8 deg | Not for the rig until his tether model changes (Tejen's code) | not today |
| A stalled attach on the T | The side carriers reach the 0.9 cap ~4 min after the lift; the ring then tips past 26-30 deg and drops (all 5 long simulation runs) | The sheet's GIVE UP at ~2.5 min stays | as written |
| LAND too soon after a fold-in | R2853 (far start): the fold-in dragged the ring 0.23 m and the LAND 10 s after the weld tipped it past 30 deg (dropped). Other runs: 3-4 cm, clean LAND. One run, cause unknown | On sq1, LAND only once the ring has settled on four | note |
| Sheet fix | Its "revert to the old ring" line typed `geometry:=rig_m2a` in partner_args, which the launch refused; fixed (graph_attach.py) with a test | Only matters if you revert | fixed |

## Overnight simulation runs (card docs/experiments/2026-10-28_attach_ambitious.md; reviewer verdicts in brackets)
| Arm | Runs | His TAKEOFF -> weld (s) | Rod min (m, >= 0.238) | Result |
|---|---|---|---|---|
| A0: his settings (baseline) | R2843 / R2844 | 38.7 / 38.5 | 0.288 / 0.286 | first try both [supported] |
| Low approach 0.30, his margins | R2846 | stall | - | his box model blocks the end [weak] |
| 0.30, swing 0.035 + tracking 0.03 | R2847 / R2848 | 32.1 / stall | 0.285 / - | R2848: his tracking check trips (1 of 2) |
| 0.30, swing 0.02 only | R2849 / R2850 | 31.3 / 31.9 | 0.282 / 0.286 | every bar [weak: see C2] |
| 0.15, swing 0.02 | R2851 / R2852 | 29.2 / 30.6 | 0.284 / 0.268 | every bar; R2852 overshoots [supported] |
| Zero plate velocity | R2856 / R2857 | 38.0 / 38.6 | 0.285 / 0.285 | no harm [weak] |
| MATCH xy 0.08 | R2858 / R2859 | 37.0 / 45.0 | 0.289 / 0.272 | R2859 one abort + retry [weak] |
| Far start, A0 height | R2853 | 39.3 | 0.286 | flies over the T; ring dropped at LAND [weak] |
| Far start, low | R2854 / R2860 (+ timing fix) | stall / stall | - | his planner parks QUAD4 between two carriers, 0.8 m short [weak] |
| Candidate: 0.30 + swing 0.02 + zero velocity + xy 0.08 + timing fix | R2862 | 31.6 | 0.284 | every bar, 1 run [weak: 1 of 2] |
| Pickup, C++ reattach | R2855 / R2861 / R2863 (+ timing fix) | stall / VOID / 97.0 | - / - / 0.287 | R2863's reattach flew his Python path (bug) [FAIL on the avoidance bar] |
| Pickup, C++ reattach, bug fixed | R2864 / R2865 | 116.1 / 97.7 | 0.247 / 0.282 | his C++ planner flew the reattach (avoidance on); weld on the 2nd descent; his tracking check tripped as the ring bobbed from the ball drop; QUAD4 tipped over at touchdown after LAND (the twin's rigid rod, as R2845); R2865 clean: one descent, no backend trip, every runner check passed [not reviewed] |

## Videos
- **Reel** (one welded run per arm, a card before each; top views, 16 min): `results/videos/reel_2026-10-29_attach_overnight_v2.mp4`
  (a 10 MB copy, `..._v2_small.mp4`, sent to your phone). The agent's first reel, `..._overnight.mp4`, labels R2853 a
  pass: it is not, see above. The pickup with the avoidance fix is R2865.
- Each run has `video/<run>_baseline.mp4` and `video/<run>_top.mp4` (`_far_top` on the far world) in
  `results/2026-10-29/R28xx_*/`. Worth a look: R2853 (the wrong-side start over the T, then the drop at LAND), R2860
  (the far low route parked between two carriers), R2862 (the candidate), R2863 / R2864 / R2865 (pickup and basket).
- No video of QUAD4 going round the carriers: his planner did not find that route (R2854, R2860).

## Changes to Tejen's code tonight (each logged in src/tejen_mission/CHANGES_FROM_TEJEN.md, each default off)
| Switch | What it does | State |
|---|---|---|
| `approach_offset_z`, `max_swing_angle_rad`, `cooperative_tracking_half_m` (launch, typed only) | the low approach and his collision margins | twin only |
| `pickup_cpp_reattach` (his planner + launch, sim path) | the pickup's reattach leg on his C++ planner (avoidance) | its activation was broken until 06:12 (R2855, R2863 flew without it); R2864 and R2865 flew it and welded; not on the rig path |
| `match_xy` on the rig path | his MATCH xy gate | typed only |
| `commit_backdate` (our ring bridge + launch) | starts the shared paths 0.04 s early | default off; critic: OK for the rig with Tejen's yes |
| `geometry` in partner_args (ours, bringup) | lets the sheet's revert to the old ring through | fixed tonight |
His package's test suite: 1126 pass, 24 fail, and the same 24 fail with tonight's edits swapped out (his M2 real-world
contracts, RViz configs, lab YAMLs). Gate `--quick` passed (04:29, 06:17).

## Decisions for you
1. **sq1 today: type `commit_backdate:=0.04`?** Recommended yes, if Tejen agrees at 08:00 (it feeds his gate; the critic
   says bounded and safe; it failed its frozen bar on a different trip). Otherwise fly sq1 as written.
2. **His tracking check on the rig.** Ours runs it on, his real-world configs run it off, and it is the likelier
   mid-transit stop on the rig. Recommended: ask Tejen; if he wants it off, I add a typed switch (not built).
3. **The low approach.** Recommended: high transit on the rig; ask Tejen for capsule-shaped tethers in his C++ model
   (his boxes block the low end unless the swing allowance is cut below what a slack cable needs). Twin and thesis:
   0.30 m, not 0.15 (R2852 came closest).
4. **The wrong-side demo.** His planner does not route round from behind the T at the low height; at the normal height
   it flies over the T, and R2853's ring then dropped at LAND. Recommended: ask Tejen about his search (partial plans
   only, 0 full plans in 1029); I look at R2853's fold-in drift.
5. **What to keep.** Recommended: keep every switch above, off by default; `commit_backdate` cannot become a default on
   tonight's evidence (its bar cannot tell the cause of a tracking trip: a new card); `pickup_cpp_reattach` (2 of 2 since the fix) kept and offered to Tejen.
6. **A ruling for the card:** does a descent abort with a successful retry (R2859) count as an "abort", and a ring drop at
   the LAND after a weld (R2853)? Recommended: the drop yes, the designed retry no.
7. **Commit:** nothing is committed; the night's code and docs are in the `week7` working copy.
8. Still open: the thesis text describes the old ring (photos of the new one are in).
