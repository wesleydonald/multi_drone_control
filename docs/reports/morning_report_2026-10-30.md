# Morning report, Fri 30 Oct: the lab sheet is ready

**Verdict:** today's sheet `docs/rig/fri_2026-10-30.md` is in your order and every line came up at the desk on the final
tree. The order: measurements first, then Tejen's attach (sq6b), the drop-off (k1_5 / k1_6 / k1_7), the 3-quad
baselines, a re-hang, the 4-quad baselines, then 0/3/6 (sq036, and sq036m on the old ring).
Today's three failures are fixed in software and checked: QUAD4 too high, never setting off for plate 9, never going
down for the object. The likeliest new blocker of a first attach is our own attach check on QUAD4's magnet marker. It
is fixed behind switches that need your word (decision 1).

## Rig findings first (from 29 Oct's logs; the twin cannot show them)
| What | Evidence | What it means today |
|---|---|---|
| **Our co-motion proof would reject a real catch** on QUAD4's magnet marker (body 24) | f4 flew 35 s with QUAD4's magnet caught on plate 9: as configured, the proof passes at 0 of 108 start times. Body 24 repeats 6.5 % of its samples even while caught (the other magnet markers 0 %). With G (repeats dropped, 0.5 s speed window, 8 mm drift): 107 of 108 | decision 1; the new markers (A2) may also fix it |
| **Our contact radius K 0.024 is the old 60 mm plates'** | in sq5_2's descents the magnet sat a median 4.2 cm off our weld point, within 24 mm only 12 % of the time. A catch 24-35 mm off gets neither our contact nor his quiet miss, so it sits until his give-up | K from the bench catch spread (A5b), else 0.035 |
| **His give-up (45 s) fits ~4 attempts** | sq5_2's cycles: 7-12 s per attempt with full descents | `contact_give_up_s:=120` (decision 1) |
| QUAD4's magnet marker froze ~0.2 s at a time in every attach flight | sq5_2 13 %, sq2_5 18 % of samples; 3 of sq5_2's 6 descent restarts came on the jump after a freeze | the descent debounce ignores frozen samples; A2: the new markers, then check with `tools/mocap_freeze_scan.py` |
| The ring's slow drift (1.4-3.8 cm, 0.07-0.23 Hz) is the whole formation following our planner's references | the carriers hold their references to under 1 cm; the references' centroid wanders 2.0 cm, the ring 2.4 cm (sq2_5, correlation 0.95-0.97) | no change on the attach; optional h3tw (decision 3) |

## What was done overnight
| Item | Result | Where |
|---|---|---|
| QUAD4 never over the carriers | the low route is the drop-off's default; the rig launch refuses any planned QUAD4 height over 2.6 m, counting the 0.2 m overshoot after a drop. Climb after the drop: 3 cm (it overshot to 2.62 m before) | twin k1_5 R2900: max 2.545 m |
| Tejen's attach in the triangle | sq6b (10 cm, 1.0 s debounce, G, K 0.035, his 120 s give-up): welded first descent in the twin | R2897, R2906 |
| 0/3/6 | the loaded pair at 0.80 throttle (1.15 kg) or 0.72 (0.86 kg), welded; the T leans up to 8.2 deg toward plate 9 before the join (the stop rule is 10 deg) | R2901, R2902 |
| The drop-off | the plate picked, carried low, dropped into the net, 2.545 m max | R2900 |
| Its fallback: QUAD4 starts with the object on its magnet (k1_7) | his `delivery_rejoin` with `attach_magnet_initial:=ON`; at the desk his manager reads ASSUMED ATTACHED with the object at the magnet. Twin: dropped at 43.2 s, 2.547 m max | R2905 |
| His contact point on the real object | typed `pickup_contact_height` (A3 measures it); without it the combined launch aims at his puck's geometry, ~1.7 cm high | desk: -0.0045 m in his planner and manager |
| The pickup object | the 100 x 100 x 25 mm 0.15 kg plate in the twin and RViz | |
| His C++ planner on the low route from outside plate 9's gap | stalls: authority granted, then his collision check fails 100+ times on one carrier's tether model, with 0.70-0.75 m of real clearance. So the direct transfer and a start in plate 9's gap stay | R2903, R2904; note for Tejen |
| The frozen marker in the twin | our proof still proved first try with and without G (the twin's freeze does not reach his MPC) | R2907, R2908 |
| The desk sweep | 15 lines came up on the final tree, 02:31 | the sheet's last section |
| Videos | every line, isometric: `results/videos/fri30_preview/<line>.mp4`, reel 21 min | |

## Decisions for you
1. **The attach's gates (sq6b):** G (our proof on body 24), K = the bench catch spread + 5 mm (else 0.035), and his
   `contact_give_up_s:=120`. Recommended: yes. sq6b0 is the line without them.
2. **Tell Tejen:** his descent restart at 10 cm / 1.0 s (your choice last night), the 120 s give-up, and the C++ stall.
   Notes ready: `~/Downloads/tejen-overnight-20261030.md` (plus last night's twin-timing and frozen-marker notes).
3. **h3tw (optional):** the planner's ring-position weight x2 in a triangle hover, against h3t, bar on the sheet.
4. **Commit** the work since 284ffb4 (nothing is committed).

## Limits
- The twin welds easily: its magnet does not swing like the rig's and its catch is a joint at contact. The rig evidence
  for decision 1 is a replay of 29 Oct's recordings, not a flight.
- The plate's 25 mm thickness, the catch spread and the contact height are measured this morning (section A).
