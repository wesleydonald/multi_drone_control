# Morning report, Fri 16 Oct (overnight 15-16 Oct; branch `overnight-1015`, not merged, not pushed)

**Verdict:** the next rig day can be prepared now. Three findings point at the attach's real problems:
- the tip falls off the edge of the 60 mm plates, so bigger discs matter most;
- the bridge's raw plate velocity keeps his match check shut;
- the ring's bounce reaches QUAD4 a quarter of a second late.

The twin has a safe answer for a ring drop during his flight. Nothing tonight changes a default; every change is a
typed switch or a new file. The full gate passes on the branch.

## Rig findings first (yesterday's bags, re-read overnight)

| Finding | Number | Why it matters |
|---|---|---|
| The tip "sliding toward the ring centre" at the bottom was the tip falling off the plate's edge | the three slides (a1 twice, t6h5_3) reached plate height 3.3-4.3 cm along the ring from the plate point, past the 60 mm plate's 30 mm edge; the one hold that led to a proof (t6h_4) arrived 2.2 cm off | a 90 mm disc would have had all three on the plate |
| Disc size, 8 rig attach flights | magnet inside the catch at the bottom 22 % (24 mm) -> 49 % (45 mm, a 90 mm disc); inside and slow 14 % -> 28 % | the bigger discs roughly double the chance |
| Longer cables (geometry) | QUAD4's room from the nearest carrier 0.45 -> ~0.7 m with either 1 m cable; a 1 m QUAD4 tether puts QUAD4 1.06 m above the ring (0.64 today) | room on the way in; less downwash on the ring (14 Oct: 5-8 deg ring tilt with QUAD4 overhead, 1-3 without) |
| What held a3_2 in MATCH for 42 s | his match check (tip within 5 cm and speed within 0.15 m/s of the plate, both unbroken for 1.0 s) held at best 1.0 s; the plate's sideways speed as the bridge reports it, 0.09 m/s rms, was the limit (`tools/match_gate_replay.py` reproduces a1's descents) | the twin reproduced it tonight (below), and a zero plate velocity removes it |
| QUAD2 let go of plate 7 (a2_2) | rod 0.57 -> 0.76 m in 0.1 s, radio fine, pack 21.9 V; the coils stay ON from each launch and nothing records their current or temperature | a props-off pull test (cold and warm, full and low pack) separates the two causes |

## Overnight twin results (lab PC; every row in the registry)

| What | Runs | Result |
|---|---|---|
| Three-carrier carry, 5 shapes x 3 speeds (your 15 Oct rig plan) | R1723-R1752 | 30/30 clean; path error 7-19 mm; tilt under 3.1 deg except the spin at 0.3 m/s (5.1); `docs/analysis/traj3_2026-10-15.md` |
| The same with 1 m cables | R1769-R1778 | 10/10 clean, the same numbers as 0.55 m |
| Latency margin at 0.3 m/s | R1794-R1799 | 15 and 30 ms extra delay: as 0 ms; 45 ms: a carrier tips at takeoff (the limit is the takeoff, as on 7 Oct) |
| Combined M1 attach with 1 m cables (carriers and QUAD4) | R1760, R1761 | both: contact, proof, fold-in, landed |
| A ring drop during his flight, `drop_takes_back` (off by default) | 8 on, 4 off (R1753-R1759, R1779-R1781, R1815-R1816) | **Switch off:** it reproduces a2_2 every time: our tracker for QUAD4 faults 1 s after the drop, his planner flies on (down onto the fallen ring), the landing waits the 30 s timeout. GIVE UP after a drop cannot hand QUAD4 over either, so the operator rule on the sheets does not work. **Switch on:** QUAD4 is taken back on the drop tick, at least 0.5 m from every carrier, and down at its rest 14-17 s after the drop, with no fault. Every case passes (end of MATCH, early MATCH, DESCEND, a carrier release, at the plate), except one bar that did not fit a drop starting at the plate (R1759, scored a fail). Critic: fix first, done. Reviewer: supported with corrections (twin only). |
| The ring shaken at the rig's bounce (0.85 cm at 1.8 Hz), his attach | R1800-R1811 (4 per arm, bars frozen before the runs) | Tip's motion against the plate before the first contact: baseline 1.40 cm, **plate-height filter 0.91 cm (-35 %)**, **zero plate velocity 1.02 cm (-27 %)**; all 8 runs below the baseline; -39 % / -29 % on a common window (reviewer). QUAD4's 1.8 Hz height 0.6 -> 0.1 cm with either. As frozen, the filter passed every bar; zero twist failed a confounded bar (the 3 cm, 0.8 Hz neighbour). Weld counts withdrawn (the twin welds at the instant of contact). |
| Zero plate velocity on a still ring | R1824-R1826 | attach as the baseline (0.55 m x2, 1 m x1): contact, weld, fold-in; no regression |
| Zero plate velocity under a slow sideways drift (0.25 Hz) | R1827-R1830 | at a 15 cm swing no worse; at a rig-like 3 cm it trails the plate a little (tip error median 1.1 vs 0.9 cm, p90 2.2 vs 1.6): the lag its critic expected; a low-passed twist would keep the slow motion |
| A slow bounce (0.8 Hz), after the fact | R1812-R1814 (3 cm), R1818-R1823 (rig amplitude) | at 3 cm the baseline and the filter sat in MATCH ~36 s (plate 0.094 m/s sideways: a3_2 again), zero twist descended at once (one run). At the rig's amplitude (post-hoc metric, ~one effective run per arm): the filter worse (+17 to +32 %), zero twist no different (0 to -7 %). Reviewer: supported with corrections, weak at 0.8 Hz |
| The filter with no shake | R1762-R1765, R1786-R1789 | attach unchanged; the QUAD4 landing tip-over after the fold-in hits both arms (twin artefact) |

## Decisions for you

1. **Merge `overnight-1015` into week5?** Nothing changes a default; the full gate passes. *Recommended: yes, after a
   look at the plan (`docs/plan_after_rig_2026-10-15.md`).*
2. **`drop_takes_back` for the rig** (closes the locked rules' "Open" line, which is yours to close). *Recommended: on
   for the next attach day, through `params_file:=configs/rig/drop_takes_back.yaml`, after a props-off check. Change
   the sheets' "GIVE UP after a drop" line in any case: without the switch it does not work.*
3. **The bounce and the match check on the rig** (one change at a time, alternating A/B with today's bridge; the critic:
   the draft sheet stacked six changes). Your call; what the evidence says:
   - **Zero plate velocity** (`attach_twist:=zero`): replayed on the rig bags with his exact gate, it opens 3 of the 4
     rig MATCH windows that never descended (a3_2: first pass at 4.9 s instead of never). Twin: the tip's bounce
     against the plate -27 % at 1.8 Hz, no worse at 0.8 Hz, no regression on a still ring (0.55 m and 1 m). Its critic
     said fix first: done (never zeroed while the ring flies a trajectory, the argument checked, the bridge logs its
     mode, the rig bag records the twist). Still open: it turns his relative-velocity guards into QUAD4-speed guards (a
     moving ring is then caught only by the xy limits), and it changes what his gates measure: **Tejen should agree**.
     A draft A/B with a hardware go/no-go is in the card for you to freeze.
   - **Plate-height filter:** -35 % at 1.8 Hz and passed its frozen bars, but worse below 1 Hz, and it needs his bottom
     hold and proof lift moved to the raw plate first (Tejen's code). Not for the next lab day.
   *Recommended: put the hardware (discs, 1 m tether) first; then an alternating zero-twist A/B if the go/no-go holds and
   Tejen agrees.*
4. **Hardware with Mitch:** 90 mm discs first; 1 m tethers at least on QUAD4; a carrier magnet pull test. Draft:
   `docs/reports/supervisor_brief_2026-10-16.md`.
5. **Console `--slot-gate`** (READY refuses a layout more than 30 deg off its plates; a3 flew two plates off).
   *Recommended: on at the rig.*
6. **His match hold** (`match_dwell`, typed only). *Recommended: leave at his 1.0 s.* Zero plate velocity addresses
   the same block without touching his gate; keep it as a fallback.
7. **The thesis inventory's 12 decisions** (`docs/analysis/sim_results_inventory_2026-10-16.md`, end of the file).
   The first is the freeze tree for the carry and detach tables. The draft carry card
   (`docs/experiments/2026-10-16_carry_speed_envelope.md`) needs your bars.
8. **Back up `results/`** (34 GB, laptop only), before the freeze batch.

## What is on the branch (from 48ccad6; about 50 commits, the main ones)

| Commit(s) | What |
|---|---|
| 1057c7e | 15 Oct rig work: free_hover circle, plate-height filter, console 22.6 V rule, QUAD4 FC backup, the day's registry rows |
| 33334ea, 53dcca3, ba6c97d | Plan after 15 Oct: blockers ranked from the data, Mitch's numbers, the queue |
| 35d2b86 | Console `--slot-gate` (off by default) |
| ab6fd18, 66a701c | His match hold and speed as typed args (rig and twin); `tools/match_gate_replay.py` |
| cca8803, 6856cf8 | Twin worlds and configs with 1 m cables (new files only), his geometry `rig_twin_L100` |
| bad76e3, 0038c1a, 42d81fe, 0a3ac30, 8694f95 | `drop_takes_back` (off by default) and its card (critic, three revisions, reviewer) |
| 8d2036c, e9ff80a, efb9b5d | The plate-height filter and the zero plate velocity, typed into his twin and rig launch; after the critic: never zero while the ring flies, the bridge logs its mode, the rig bag records the twist; `tools/n6_mechanism.py` |
| 83a2c75, 8ded848, 6f26708, 040341b, 13e6ee2 | The filter card: revs 2 and 3, the reviewer, the zero-twist critic and the drift check |
| d4f8ce8, e96ae68, b786e8f, 9ac6232 | The trajectory, 1 m and latency matrices and their notes |
| 26ac24f | The thesis sim-results inventory |
| d34c421, 9ca9b28, 44cf72c, 7af0260, 217c5d7 | Drafts: the next lab day's sheet, the carry claim card, the brief for Mitch |
