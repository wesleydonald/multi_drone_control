# Morning report, Fri 16 Oct (overnight 15-16 Oct; branch `overnight-1015`, not merged, not pushed)

**Verdict:** DRAFT (filled in through the night).

## Rig findings first (yesterday's bags, re-read overnight)

| Finding | Number | Why it matters |
|---|---|---|
| The "slide toward the ring centre" at the bottom is the tip falling off the plate's edge | the three slides (a1 twice, t6h5_3) reached plate height 3.3-4.3 cm along the ring from the plate point, past the 60 mm plate's 30 mm edge; the one hold that went on to a proof (t6h_4) arrived 2.2 cm off | a 90 mm disc (45 mm radius) would have had all three on the plate |
| Bigger disc, 8 rig attach flights | magnet inside the catch at the bottom: 22 % (24 mm) -> 49 % (45 mm); inside and slow 14 % -> 28 % | Mitch's bigger discs roughly double the chance |
| Longer cables (geometry) | QUAD4 to the nearest carrier 0.45 -> ~0.7 m with either 1 m cable; a 1 m QUAD4 tether puts QUAD4 1.06 m above the ring (0.64 today) | room on the way in; less downwash on the ring (14 Oct: 5-8 deg tilt with QUAD4 overhead, 1-3 without) |
| His match gate is what held a3_2 (42 s, no descent) | both conditions true 36 % of the time, longest unbroken 1.0 s (replay of his exact gate, `tools/match_gate_replay.py`, reproduces a1's descents) | a 0.5 s hold would have descended at 1.1 s (6 chances in 42 s) |
| QUAD2 let go of plate 7 in a2_2 | rod 0.57 -> 0.76 m in 0.1 s, radio fine, pack 21.9 V | not explained; the carrier magnets' hold has never been measured |

## Overnight twin results (lab PC, every row in the registry)

| What | Runs | Result |
|---|---|---|
| Three-carrier carry, 5 shapes x 3 speeds (your 15 Oct rig plan) | R1723-R1752 | 30/30 clean; path error 7-19 mm, tilt under 3.1 deg except the spin at 0.3 m/s (5.1); `docs/analysis/traj3_2026-10-15.md` |
| The same with 1 m cables | R1769-R1778 | 10/10 clean, the same numbers as 0.55 m |
| Combined M1 attach with 1 m cables (carriers and QUAD4) | R1760, R1761 | both: contact, proof, fold-in, landed (free hang read 1.0405 m) |
| A drop during his flight (`drop_takes_back`, off by default) | R1753-R1759, R1779-R1781 | switch off reproduces a2_2 every time; switch on takes QUAD4 back and lands it in 14-17 s; one bar failed as frozen (R1759, explained); reviewer: PENDING |
| The ring shaken like the rig (1.8 Hz) | R1766-R1768, R1782-R1785 | a 1 cm bounce stops the twin attach (no contact, his give-up); at 0.7 cm with the filter it attaches (2/2); baseline at 0.7 cm: PENDING |
| The plate-height filter, no shake | R1762-R1765, R1786-R1789 | attach unchanged; the twin's known QUAD4 landing tip-over hit: PENDING |
| Latency margin (15-45 ms extra) at 0.3 m/s | PENDING | |

## Decisions for you

PENDING

## What is on the branch

PENDING
