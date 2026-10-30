# Attach gates after the descent: what can still stop the first attach (sq6b, Fri 30 Oct)

Written Fri 30 Oct 00:41 for the rig sheet docs/rig/fri_2026-10-30.md section 3 (sq6b). Exploratory analysis on the rig
logs and bags only, no claim. Scripts: `results/analysis/attach_gates_1030/` (`extract.py` and `extract2.py` build the caches
from the bags, `p*.py` the tables). Code read at 284ffb4 plus the working copy.

**Verdict:** if sq6b's descents now reach the plate, the most likely blocker is our own co-motion proof. On a magnet
truly caught on plate 9 in flight (QUAD4's marker, body 24, on plate 9 through f4's 35 s figure-8) it proves 0 of 108
start times as configured, because body 24 is noisy and freezes. Second is our contact radius K 0.024, sized for the
old 60 mm plates, against a magnet that sits a median 4.2 cm off our weld point in the descents. Third is his give-up
budget (45 s, 3 failed proofs), which the first two use up.

## The line as typed (sq6b)

| What | Value on sq6b | Where it lands |
|---|---|---|
| W, L, K, X, Y | 0.028, 1.041, 0.024, -0.005, 0.197 (29 Oct bench, RIG-1029-bench, K not measured: the 14 Oct value) | weld_point_z, taut length, weld_radius, weld point in the ring frame |
| P = L - 0.003, Q = L - 0.04 | 1.038, 1.001 | proof_taut_len (proof fallback 1.041), attach_cable_len (rod from the pivot) |
| proof_rules | comotion (common.yaml default) | contact xy = K, his lift-done signal only (no timed fallback), taut fallback P + 0.003 |
| his descent | approach 0.30 m, 0.08 m/s to 1 cm under W (3.9 s), restart at 10 cm held 1.0 s on fresh tip samples, or QUAD4 body xy speed over 0.25 m/s at once | `descent_abort_xy:=0.10 descent_debounce:=1.0` in partner_args |
| his bottom hold | on (rig path): gross 12 cm or 0.35 m/s for 0.3 s, face 2 cm under the plate for 0.12 s, quiet miss (tip at most 0.03 m/s, over 3.5 cm off, no 5 mm progress for 4.0 s), 3 misses give up | partner_mission.launch.py, his defaults |
| his proof lift | 0.03 m above the contact height at 0.02 m/s (2.0 s from the -1 cm bottom), then /join_planner/proof_lift_complete | RIG proof_rise_max 0.03 |
| his give-up | 45 s after the first descent, or 3 failed proofs, or 3 quiet misses | `contact_give_up_s` (his rig launch, typable), his defaults |
| our proof | contact: xy at most K, normal -15..+12 mm, at most 0.03 m/s for 0.30 s. Proven: 1.0 s after his lift-done with drift at most 6 mm, rel. speed at most 0.03 m/s (0.2 s slope), normal within 10 mm, taut, plate moved 5 mm. Failed: lost or separated at 45 mm, inconclusive 3.0 s after the lift, stale 0.5 s | comotion_proof.py defaults, graph_attach |

## Gates, most likely to block first

| # | Gate (owner) | Parameter, value on sq6b | What the rig data says the magnet does there | Verdict | Narrowest change (owner) |
|---|---|---|---|---|---|
| 1 | Co-motion proof on body 24 (ours) | proof dwell 1.0 s, drift 6 mm, rel. speed 0.03 m/s over a 0.2 s slope, gap 0.1 s, inconclusive 3.0 s. A repeated tip sample counts as fresh | Body 24 caught on plate 9 in flight, 29 Oct square: f4 proves 0 of 108 start times (all inconclusive), h4f_3 7 of 36. The carriers' markers on plates 0 and 3 in the same f4: 54 of 54. Body 24 in flight: z sd 3.8 mm (bodies 21-22: 0.9), slope speed median 0.029 m/s, drift p90 6.7 mm, 7-15 % of samples frozen on the plate and 32-34 % while QUAD4 flies over plate 9 (sq5_2, sq2_5). Synthetic catch on sq5_2/sq2_5's plate motion with their holds and f4's noise: first proof 2-3 %, within 3 tries 8-11 % | **FAIL** | A switch in our magnet manager (default off, Wesley's word: proof thresholds): drop a tip sample that repeats the last one bit for bit, run the proof's dwells on fresh-sample time only (Tejen's rule: stale evidence earns no dwell), comotion_speed_window_s 0.5, comotion_d_ok 0.008 (manager parameters already, graph_attach does not pass them yet). Replays: f4 107 of 108 first try, synthetic 100 % first try (median 4.3 s), the 14 Oct catch (t6h_4) still proven, 0 contacts on 7 no-catch flights. His lift moves an uncaught tip 30-40 mm (separation at 10 mm), 5x the 8 mm drift. No-code route: the new body 24 markers, if its in-flight sd and holds fall to bodies 21-22's |
| 2 | Contact radius K (ours) | weld_radius 0.024 = comotion_contact_xy (old 60 mm plates: furthest hand catch 19.2 mm + 5, RIG-1014-t3) | sq5_2's descents, fresh tip samples: median 41.5 mm off our weld point, within 24 mm 12 %, 35 mm 28 %, 45 mm 57 %, mean 25 mm radially inward. The only rig catch ever (t6h_4) sat at 23.6 mm, 0.4 mm inside K. New slots ~100 x 85 mm, catch area never measured. A catch at 24-35 mm gets neither our contact nor his quiet miss and sits until his 45 s give-up | **FAIL** for most catches | Typed (ours, Wesley's word): measure the furthest catch at the bench (props off, coil ON, ~10 hand catches round the slot's edges, ring frame, as 14 Oct t3), K = furthest + 5 mm. Without the bench: K 0.035, which closes the 24-35 mm dead zone and stays 10 mm inside the 45 mm loss rule. K over 0.040 also needs comotion_lost and comotion_sep_xy moved to K + 0.021 (code, graph_attach). Replay: K 0.045 (loss 0.065) adds 0 contacts on 7 no-catch flights, and a synthetic catch 4 cm off proves like one at 1 cm |
| 3 | Give-up budget (his) | contact_give_up_s 45 s after the first descent, contact_max_failures 3, contact_max_misses 3 | sq5_2: first descent 16.6 s after his TAKEOFF, 6 cycles in 22.4 s (re-approach 0.5-3.2 s plus MATCH 0.6-4.7 s per cycle). With full descents (3.9 s) and bottom holds (15 Oct: 1.9-2.2 s) a cycle is ~7-12 s, so 45 s is 4-6 attempts. With row 1 as configured, 3 failed proofs end the flight ~17 s after the first contact. The sheet says LAND after ~2.5 min | **UNKNOWN** (binding if rows 1-2 hold) | Typed in partner_args (his gate, Wesley's and Tejen's word): `contact_give_up_s:=120`. W1's fade (+0.066 per min) takes the triangle from 0.71 to ~0.88 after 2.5 min of hover, under 0.95. Nothing for the failures until row 1 is fixed |
| 4 | Quiet miss in the bottom hold (his) | contact_quiet_wait_s 4.0, improvement 5 mm, min xy 35 mm, tip 0.03 m/s | Synthetic catch at 45 mm on sq5_2/sq2_5's motion: leaves the hold after 4.0 s without holds, 7.5-9.0 s with 29 Oct's holds, then APPROACH_ABOVE_TARGET, our span OFF, coil OFF: a real catch released on purpose. Catches at 10 and 30 mm: never fires | **FAIL** with K 0.024, **PASS** once K reaches 35 mm (our contact fires at 0.31 s) | None in his code. Row 2 |
| 5 | His lift-done signal (his, read by ours) | /join_planner/proof_lift_complete, on change and at 2 Hz. comotion_lift_fallback_s 0 (no timed fallback) | Flows on the rig but only ever False: 235 / 373 / 319 / 728 / 484 messages in sq5_2 / sq2_5 / k1_4 / a1 / a3_2, none True. No LIFT_PROOF on the rig since 14 Oct (before the signal). His lift base uses the measured tip, which a hold can put off by the plate's motion (median 16 mm in 0.14 s) | **UNKNOWN** (passes in the twin) | None. Watch for our "no lift-done signal" warning 6 s after a contact |
| 6 | Taut check and plate height: P, Q, W (ours) | P 1.038 (fallback 1.041), Q 1.001, W 0.028, normal gate -15..+12 mm, proof normal 10 mm | Free hang in sq5_2's flight, fresh samples: body-to-tip median 1.0388 m, sd 1.4 mm (threshold 1.0358). With holds included the 2 s free-hang median ranged 1.029-1.047 m in sq2_5: a high one fails a real catch. Loaded plates' markers in flight held their rest height within -0.6..+2.1 mm (f4, h4f_3), body 24 on plate 9 read +2.1 mm. t6h_4's catch +0.7..+3.1 mm. Plate 9 is unloaded mid-span in the triangle: never measured there | **PASS** with row 1's repeat drop, at risk without it | Row 1. W, X, Y, L re-measured on the redefined body 24 (sheet step 0.5) |
| 7 | Bottom hold gross and depth (his) | contact_gross_xy 0.12 / speed 0.35 m/s for 0.3 s, contact_face_below_m 0.020 for 0.12 s | Synthetic catches at 10, 30 and 45 mm with 29 Oct's holds: 0 of 196 left on gross or depth. 15 Oct (old ring, a1): both bottoms left on depth, the tip slid 6-8 cm off the 60 mm plate | **PASS** for a catch | None |
| 8 | Descent restart before the bottom (his) | descent_abort_xy 0.10, descent_debounce 1.0, speed 0.25 m/s at once | W2a: 85 % of full descents survive (rig_1029_replays.md). Holds over 0.1 s restart the debounce count, 1.44-1.64 per s over plate 9, and only 4-5 % of 1.0 s windows have none, so the 10 cm restart rarely completes (W2a did not model this: 85 % is a floor). QUAD4 body xy speed over 0.25 m/s: 2.5 % (sq5_2), 5.4 % (sq2_5) of samples | **PASS** likely | None. The gross 12 cm rule in the hold is the real position guard now |
| 9 | Contact reported to his planner (ours to his) | /magnet/contact from our manager (transient local, on change and at 1 Hz), his contact_reported, LIFT_PROOF from DESCEND or ATTACH_READY | t6h_4: our PROVE at 39.42 s, his LIFT_PROOF at 39.41 s (the same tick), 3 of 3 contacts | **PASS** | None |
| 10 | Magnet gate and coil (ours) | planner span ON at the first APPROACH_ABOVE_TARGET (QUAD4 over 0.3 m), OFF in APPROACH/MATCH after a descent, ON at each DESCEND_TO_ATTACHMENT. Manager -> /drone_3/magnet_ours -> magnet_gate_3 -> /drone_3/magnet_coil | sq5_2: 6 ON and 6 OFF, the coil 3-11 ms after the span, our proof in WATCH within 0.05 s of each ON | **PASS** | None |
| 11 | Hand-over and fold-in (ours) | mux: welded and our tracker live. Planner fold-in refused only if QUAD4 is out, disarmed or in LAND | Our drone-3 tracker was warm through sq5_2's attach (armed, thr_out median 0.47, reference 4 mm from the body, age 0.05 s). The same mux switch flew every take-back (14, 15, 29 Oct). The fold-in itself is twin only (R2886-R2895). Plate 3's carrier after it: 0.73-0.76 (W1) | **UNKNOWN** (low risk) | None |

## The evidence behind rows 1 and 2

| Real caught magnet on plate 9 (29 Oct square, body 24) | Proven, first try | Proven within 3 tries |
|---|---|---|
| f4, as configured | 0 of 108 | 0 of 108 |
| f4, speed window 0.5 s only | 56 of 108 | 90 of 108 |
| f4, repeats dropped, gap 0.35 s, window 0.5 s | 67 of 108 | 101 of 108 |
| f4, repeats dropped, fresh-time dwell, window 0.5 s, drift 8 mm | 107 of 108 | 108 of 108 |
| f4, bodies 21 and 22 (plates 0 and 3), as configured | 54 of 54 each | 54 of 54 each |
| h4f_3 (hover, 7.9 s in the air), as configured / proposed | 7 of 36 / 29 of 36 (7 ran out of record) | - |

| Synthetic catch (1 cm off, plate motion and holds of the flight, f4's body 24 noise) | sq5_2 first try | sq5_2 within 3 | sq2_5 first try | sq2_5 within 3 |
|---|---|---|---|---|
| as configured | 2 % | 11 % | 3 % | 8 % |
| no holds, no noise | 100 % | 100 % | 100 % | 100 % |
| repeats dropped, hold time credited (gap 0.35 s), window 0.5 s, drift 8 mm | 90 % | 100 % | 94 % | 100 % |
| row 1 as proposed (repeats dropped, fresh-time dwell, window 0.5 s, drift 8 mm) | 100 % | 100 % | 100 % | 100 % |
| row 1 as proposed, K 0.045, loss 0.065, catch 4 cm off | 100 % | 100 % | 100 % | 100 % |

Median time from the touch to ATTACH with row 1 as proposed: 4.3 s (0.3 s contact, 2.0 s lift, 1.0 s of fresh proof,
plus the holds).

| Real streams through the proof (contacts / proofs) | Rig as typed | Proposed | Proposed, K 0.045 |
|---|---|---|---|
| t6h_4 (14 Oct, the one real catch) | 1 / 1 | 1 / 1 | 1 / 1 |
| t6f2_4, t6h5_3, t6g_2, a1, a3_2, sq5_2, sq2_5 (no catch) | 0 / 0 | 0 / 0 | 0 / 0 |

| sq5_2 descents, fresh tip samples (n 432) | Value |
|---|---|
| xy from our weld point, median | 41.5 mm |
| within 24 / 35 / 40 / 45 / 60 mm | 12 / 28 / 46 / 57 / 78 % |
| share of touches inside a 40 / 45 mm catch area that our K 24 sees | 27 / 22 % |
| mean offset in the ring frame (x, y) | (-2.5, -25.2) mm, y = radially inward at plate 9 |

| Body 24 holds (repeated pose, stamps advancing) | sq5_2 | sq2_5 | k1_4 | f4 (on plate 9) | h4f_3 (on plate 9) |
|---|---|---|---|---|---|
| samples frozen, QUAD4 over plate 9 / on it | 32.5 % | 34.3 % | 14.8 % (over the object) | 7.0 % | 14.6 % |
| holds per s | 2.84 | 2.82 | 0.80 | 0.37 | 1.14 |
| median hold | 0.14 s | 0.20 s | 0.20 s | 0.21 s | 0.20 s |
| 1.0 s windows with no frozen sample | 0 % | 1 % | 33 % | 64 % | 21 % |
| before takeoff (props off) | - | - | - | 0.0 % | 0.1 % |

The holds come with flight, not with the plate: bodies 21-23 never froze, body 24 did not freeze on the floor before
takeoff, and it froze 12-75 % across his climb, transit and MATCH. So the props-off bench (step 0.5) cannot show them.

## What to watch in T2 during sq6b

- **Coil:** `[magnet_gate] coil ON (our command)` after each `[planner] ring magnet span: ON (his DESCEND_TO_ATTACHMENT)`.
- **Descent:** `Planner phase: DESCEND_TO_ATTACHMENT -> ATTACH_READY. Reason: terminal attachment profile complete` = the
  bottom reached. `... -> APPROACH_ABOVE_TARGET. Reason: attachment descent aborted after loss of relative alignment` =
  a restart (row 8).
- **Bottom hold:** `bottom hold left: moving or far off the plate`, `... the tip sunk below the plate (off its edge)`,
  `... quiet miss n/3: no 5 mm progress in 4 s, NN mm off` (row 4: a tip still for 4 s at plate height, NN mm off, is
  most likely a catch outside K, released by the coil OFF that follows).
- **Live gate:** in a spare terminal `ros2 topic echo /magnet/attach_proof` (10 Hz JSON: `state`, `xy_m`, `normal_m`,
  `rel_speed_mps`, `l_hang_m`, `gap_restarts`). A tip sitting still at plate height (`normal_m` near 0) with `xy_m`
  over K is row 2.
- **Contact:** `CONTACT (try n): xy .. mm, normal .. mm, rel_speed=.., free-hang tether .. m (free hang)` then his
  `-> LIFT_PROOF. Reason: contact reported; proving the catch by a small lift` and `LIFT_PROOF: tip measured ...`.
- **Proof:** `ATTACH: proven (try n) ...` is the goal. `PROOF FAILED (try n, inconclusive)` with drift under 10 mm is
  row 1. `PROOF FAILED (..., separated)` or `lost` is a real separation or a hold. `PROOF (try n): no lift-done signal
  from his planner` is row 5.
- **Give-up:** `Contact attach: giving up (no confirmed attach 45 s after the first descent | 3 failed proofs | 3 quiet
  misses at the bottom)` then `TAKE-BACK of drone 4 (/drone_3) (his give-up)` (row 3).
- **Fold-in:** `[elrs_mux]` hand-over, `[Drone 4] mux -> ours: solver re-seeded`, his `RING_ATTACHED: mux reads ours`.
  `weld of drone 4 (/drone_3) refused: ...` would be row 11.
- **After each attach flight:** `python3 tools/mocap_freeze_scan.py results/rig/2026-10-30/sq6b` and
  `python3 tools/proof_replay.py results/rig/2026-10-30/sq6b --weld X Y W --contact-xy K --taut-fallback 1.041`.

## Limits

- No magnet has touched plate 9 of the new ring in flight. Row 1's real-catch evidence is body 24 on plate 9 while QUAD4
  carried in the square (cable at 60 deg, 7-15 % frozen), not QUAD4 hovering over its own catch (32-34 % frozen while
  over the plate). Its taut check was switched off there: at 60 deg the marker-to-body geometry reads ~5 mm short.
- The synthetic catch moves the tip exactly with the plate. Real slip, the plate pulled up by his lift, and the magnet's
  hold force are not in it. White noise stands in for body 24's in-flight noise.
- The false-proof check is weak: on the no-catch flights his planner never lifted, so the separation rule after a lift
  is untested on the rig. The checks show only that the wider settings add no contact candidates there.
- The catch area of the ~100 x 85 mm slot, the magnet face size, whether the ring's metal sheet outside the slot also
  catches, and which magnet QUAD4 carries after 29 Oct's swap are unknown. K's fractions assume a round catch area.
- Wesley's new body 24 markers may remove rows 1 and 6 without code, or move the pivot and every typed W, X, Y, L.
- His give-up cycle times are estimated from sq5_2's restarts and 15 Oct's old-ring bottoms. No full bottom hold has
  flown on the new ring.
