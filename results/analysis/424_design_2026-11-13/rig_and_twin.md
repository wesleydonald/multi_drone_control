# 4 -> 2 -> 4 on the real quads: layout, hardware, pair evidence, twin plan, sheet rows (13 Nov 2026)

Read-only design study for the supervisor's item 3 of 12 Nov (decisions DECIDED 2026-11-12 22:52; Wesley 13 Nov: both
returns flown by Te-Jen's planner, attach commands simultaneous if possible). Written Fri 13 Nov ~17:30 from the registry,
decisions, the rig scripts of 13 Nov, the twin configs and the code on week8 (uncommitted tree). Nothing was flown or
edited. The software side (two of his instances, our planner with two newcomers) is in `ours.md` and `tejen_stack.md`
beside this file. No bars: everything here is read as flown.

## 0. Summary

| Item | Recommendation | Why, in one line |
|---|---|---|
| Layout | The square: QUAD1 on plate 0, QUAD2 on 6 (the pair), QUAD3 on 3 and QUAD4 on 9 (the returners) | QUAD4 keeps its plate, funnel and tuned settings, the two returns come in from opposite sides, the twin flew this square (R2829, R3743) |
| Typed | `attach_azimuths_deg:=0,180,90`, his `carrier_azimuths_deg:=[0.0,180.0,90.0]`, QUAD4 `plate:=9` as today | angles are clockwise, 30 x plate |
| Rules | A new params file: today's attach file + `drop_gap_deg 200`, `min_survivors 2`, `int_k_xy 0` | the pair needs the gap rule; the twin's T is level with the sideways integral off (1.9 against 8.5-14 deg) |
| Detach | In succession first (flies on today's code), together once built and twin-checked | together skips the T, which leaned 21-28 deg on the rig (sq8_2) |
| Returns | QUAD4 first, QUAD3 second; both at once only after a caught succession | QUAD4 has the catch record; at the end the two bodies are only 0.45 m apart |
| Order of flights | Rung B (4 -> 2 -> 3) on today's graph, then rung C (4 -> 2 -> 4) when the two-instance stack is built | rung B needs only a params file and a script; rung C needs new code on both sides |
| Biggest rig risk | The pair has no control over the ring's roll about its own line | a contact push or a returner's downwash on plate 3 or 9 rolls the ring freely; the twin cannot show downwash |

**What the rig has shown for a pair:** two quads have carried only the 0.86 kg M2A ring on 0.55 m links (21-22 Oct and
28 Oct). With the slope filter, 9 of 9 pair flights on 28 Oct held the ring (hover tilt mean 2.0-2.1 deg, peak 4.4-5.4;
throttle peak 0.57-0.69). **No pair has ever carried a ring on 1 m cables on the rig**: the 22 Oct and 29 Oct chain lines
ended before the pair (a wrong quad released, the plate-0 magnet letting go, the T at its throttle cap on the 1.15 kg ring).

## 1. Layout

### 1.1 Options

Plates count clockwise from the ring's +x seen from above (`docs/rig/plate_numbering.md`): plate 0 on +x, 3 on -y, 6 on
-x, 9 on +y. The ring radius to the plates is 0.225 m.

| | A. Square 0/3/6/9 (recommended) | B. Today's layout 3/7/9/11, the pair 3/9 | C. Square, returners 0 and 6 |
|---|---|---|---|
| Pair (carries throughout) | QUAD1 on 0, QUAD2 on 6 | QUAD1 on 3, QUAD4 on 9 | QUAD4 on 9, one on 3 |
| Returners | QUAD3 on 3, QUAD4 on 9 | QUAD2 on 11, QUAD3 on 7 | two non-QUAD4 quads on 0 and 6 |
| Re-hang | 3 magnets moved (~10-12 min) | none | as A |
| His planner flies QUAD4 onto plate 9 | yes, as every attach line | no (both returners new to his MPC) | no |
| Funnels | the one on 9 stays, a second on 3 | two new on 11 and 7 | two new on 0 and 6 |
| Returners' end positions (bodies over the plates) | 0.45 m apart, opposite sides | 0.39 m apart, both on the +y half | 0.45 m apart |
| A returner's corridor | 90 deg clear either side (today 60) | 60 deg to QUAD4's cable on one side | 90 deg |
| Twin precedent | R2829, R3743, R3748 (4 -> 3 -> 2 -> 3, his re-attach on 9 onto the pair 0/6), X2 | none | none |

A is recommended. B saves the re-hang but makes both returners quads his MPC has never flown, onto plates his planner has
never targeted, and puts QUAD4 (the partner drone in every launch: `graph_attach.ATTACH_DRONE_ID = 3`) on the pair. C has
the same problem.

Which quad is the second returner: QUAD3 keeps the default id map (drone_2 = QUAD3 = Motive body 13, magnet marker 23,
`/dev/QUAD3`), so a generalised attach graph can list the newcomers as drone_2 and drone_3 without retyping radios or
bodies. Its thrust offset is QUAD4's (0.187 in the twin's per-quad map `0.181,0.185,0.187,0.187`), and the pair then gets
QUAD1 and QUAD2, the two strongest by the same map. **Check first:** QUAD1's magnet let go three times on the 1.15 kg
ring's plate 0 at ~4-5.2 N (RIG-1029-h4, -ch1, -ch1b; memory: "QUAD1's magnet a bit weaker; a replacement magnet is
coming", not recorded as replaced). No QUAD1 release on the M2A ring since 4 Nov. If the bench pull (section 2) shows
QUAD1's magnet weakest, swap QUAD1's and QUAD3's magnet-and-cable assemblies (they swap like QUAD1's and QUAD4's did on
29 Oct, RIG-1029-ch1b_2); the markers 21 and 23 move with them, so the returner's tip body is typed from where it hangs.

### 1.2 Geometry and the position limit

| Body | Plate | Ground start (laid out on the plate's line) | In flight (60 deg cable plan) | After the detach step-out |
|---|---|---|---|---|
| QUAD1 (pair) | 0 | (+1.25, 0) | (+0.73, 0), 0.87 m above the plate | stays |
| QUAD2 (pair) | 6 | (-1.25, 0) | (-0.73, 0) | stays |
| QUAD3 (returner B) | 3, second funnel | (0, -1.25) | (0, -0.73) | (0, -1.2) then his planner |
| QUAD4 (returner A) | 9, funnel | (0, +1.25) | (0, +0.73) | (0, +1.2) then his planner |

- Every point is inside +-1.8 m (pos_limit_m 1.8) with ~0.55 m to spare, and every start is inside the console's 1.3 m
  READY rule. The ring at the cage centre, plate 0 toward +x (13 Nov fm9h8_4: a ring set down 0.73 m off centre took a
  carrier to the 1.8 m limit, on the figure-eight).
- Spacing: neighbours in the square fly 1.03 m apart (today's triangle puts QUAD4 0.73 m from the carriers on 7 and 11).
  A returner over its plate is 0.76 m from each pair quad. **The two returners over plates 3 and 9 are 0.45 m apart**:
  the closest any two quads come in this test, and only when both are on their final descent at once.
- No object, no stand: the -y side stays clear for QUAD3's take-back or give-up landing (a freed drone lands >= 1.2 m
  out on its plate's bearing, `DEPARTED_CLEAR_R`).
- QUAD4's drop-off mat at (0, +1.55) is not used: both returners start welded on their plates, as the nrtf lines.
- QUAD3, like QUAD4, comes in low between the pair, 0.30 m over the plates (`attach_transit_clearance 0.30`,
  `minimum_search_z_m 2.20`, `attach_cpp_transit:=false`), never over a carrier, under `max_body_z` 2.6 at the ring's
  1.0 m.

### 1.3 The gap rule, the balance and the loads

**drop_gap_deg 200 with min_survivors 2** (configs/rig/chain_pair*.yaml; common.yaml defaults min_survivors 3 and the gap
rule off, so the attach line must type both, as R2827 found in the twin: refused at min_survivors 3):

| Fleet | Largest empty arc | Flies under the rule? |
|---|---|---|
| Square 0/3/6/9 | 90 | yes |
| T 0/6/9 or 0/3/6 (succession, either way) | 180 | yes |
| Pair 0/6 | 180 (its larger arc) | yes; an announced detach to a pair is refused without drop_gap_deg (`_detach_plan`) |
| A pair quad lost from the T (e.g. 6/9 left) | 270 | drop and land, as designed (RIG-1022-t3p_3 passed this case on the rig) |
| A pair quad lost from the pair | 1 survivor < 2 | drop and land |

**Balance.** The square with one quad per plate is symmetric: each cable takes a quarter of the ring. The pair takes half
each. From the ring's 0.86 kg, the 60 deg cable plan and the identified thrust map (throttle = 0.185 + 0.507 x mass -
0.022 (V - 23.5)):

| Fleet | Vertical share per cable (N) | Pull along the cable (N) | Throttle, static (23.5 V / 22.5 V) |
|---|---|---|---|
| Square | 2.11 | 2.44 | 0.57 / 0.60 |
| Today's triangle (QUAD4 away) | 2.81 | 3.25 | 0.61 / 0.63 |
| Pair, or the pair inside a T | 4.22 | 4.87 | 0.68-0.70 / 0.70-0.72 |

The pair has room under the 0.95 cap (the 28 Oct pair peaked at 0.57-0.69 on the same ring). The magnets are the thin
margin: ~10 N held in a straight pull (Wesley 30 Oct), but on 21-22 Oct pair magnets let go at 6.4-6.5 N peaks during
lifts that rocked (RIG-1021-c2q_p11p5_2, RIG-1022-h2). The pair's 4.9 N static is ~75 % of that. Here the four lift the
ring and the pair only hovers, and the soft detach (3 s unload) ramps the pair's share up. An unannounced double release
would step it up at once: do not fly that on the rig (section 4 probes it in the twin).

**The pair cannot hold the ring level about its own line.** Both cable ends lie on the 0-6 line, so no cable force has a
moment about it. Only gravity holds the roll, through the height h of the cable ends above the ring's centre of mass
(h ~ 2-5 cm: attach_z 0.0175 in the plan, the magnet marker 0.049 above the ring pivot). Consequences:

| Effect on the pair-held ring | Size (h 2-5 cm) |
|---|---|
| Lean from one 11 g funnel on plate 9 alone (centre of mass 2.8 mm toward 9) | 3-9 deg, plate 9 low |
| Lean from funnel v4 (83 g) on plate 9 alone | 22-48 deg: never fly v4 against the supervisor's 11 g funnel |
| Two equal funnels on 3 and 9 | cancel |
| Vertical stiffness at plate 3 or 9 (M g h / r^2) | 3-8 N/m: a 1 N push moves the plate ~15-30 cm, the ring rolls freely |
| Largest restoring moment (M g h) | 0.15-0.42 N m |

So the first return, onto the pair, meets a ring that rolls away from a contact push and toward a magnet's pull, and
that a returner's downwash over plate 9 (QUAD4 hovers ~1 m above it with ~5.8 N of thrust) can roll by many degrees: the
twin has no downwash and cannot show this. In the twin the contact push was small: QUAD4's re-weld onto the pair peaked
at 5.6-9.9 deg (R3743, R2829) and 2.8 deg with the fold-in cap (R3748). The rig already hangs off level with four
carrying: 3.7-5.4 deg in 6 of 7 flights on 11 Nov, plate 9 low by 14-19 mm (FINDING 2026-11-11 15:56, cause open). On the
pair, a lean like that cannot be corrected. Hence the pair-hang check in the setup (section 5).

Two physical points that favour "together":
- **Detach together (square to pair):** the two leavers are symmetric about the 0-6 line, so their unload and release
  put no net roll on the ring. In succession the first release makes a T: the twin's soft square-to-T release peaks at
  21.6-29.6 deg (R3677, R3708-R3714, R2821/R2822), and the rig's T leaned 21-28 deg (sq8_2, sideways integral on).
- **Return together:** two returners over plates 3 and 9 push and blow on opposite sides of the 0-6 line, so their
  loads roughly cancel. Against it: their bodies end 0.45 m apart, and his planner does not see the other returner (an
  obstacle model change in his planner: Te-Jen's call, 13 Nov rule).

### 1.4 What our planner types, and what changes in the scripts

From `results/rig/2026-11-13/scripts/rig_nrtf.sh` (the hover detach and re-attach, today's values):

| Line in rig_nrtf.sh | 4 -> 2 -> 3 (rung B, today's attach graph) | 4 -> 2 -> 4 (rung C, the new stack) |
|---|---|---|
| `attach_azimuths_deg:=90,330,210` | `0,180,90` (drone_0 = QUAD1 on 0, drone_1 = QUAD2 on 6, drone_2 = QUAD3 on 3) | `0,180` for the two carriers, the newcomers' plates 3 and 9 typed per newcomer |
| `params_file:=attach_m1_1m_m2a_drop55_det15.yaml` | new `configs/rig/attach_m1_1m_m2a_drop55_det15_pair.yaml` = that file + `drop_gap_deg: 200.0`, `min_survivors: 2`, `int_k_xy: 0.0` (as `chain_pair_attach_1m.yaml`, Wesley 27 Oct, which lacks the M2A values and the 15 cm check) | the same |
| `drop_tilt_deg` | 55 (the attach lines' value; the chain files' 45 would cut the T's real 21-28 deg lean close) | 55 |
| his `carrier_azimuths_deg:=[90.0,330.0,210.0]`, `plate:=9` | `[0.0,180.0,90.0]`, `plate:=9` | A: `plate:=9`; B: `plate:=3`, carriers `[0.0,180.0]` plus QUAD4 once it is on |
| `X=0.0; Y=0.2293; W=0.0491; L=1.034; K=0.025` | QUAD4's, unchanged (the funnel stays on 9) | plus QUAD3's own W, X (~0), Y (~-0.229), L, measured in the plate-3 funnel |
| `mocap_magnet_tip_body_id:=24` | 24 | 24 for A, 23 for B (a second tip body: new launch argument) |
| `magnet_operator_topic:=/drone_3/magnet_operator` | as today | one per returner |
| Motive set | `8,11,12,13,14,24` (6 bodies) | `8,11,12,13,14,23,24` (7 of the 8 allowed at 240 Hz) |
| DETACH | console DETACH QUAD4, then RViz DETACH drone 2 | one press for both (new console button), or the same two presses |
| Unchanged | ring at 1.0 m (`target_z:=1.0`), `throttle_max:=0.95`, `detach_unload_s:=3.0`, his bands 5/5 cm (BAND=cone), `contact_dz:=-0.03`, the low route, `mission_mode:=floor_join`, the M2A solvers (`~/mdc_rig_generated_m2a_0p86` holds n = 2, 3 and 4; every attach launch already loads n = 2, `prebuild_sizes`) | |

The ring stays at 1.0 m (nrtf's height): the twin's plate contact fails at 1.2 m (R4243).

## 2. Hardware list

| # | Item | What to do | Why |
|---|---|---|---|
| 1 | Second funnel on plate 3 | One of the supervisor's 105 mm funnels (11 g). One already sits on the small object (FACT 2026-11-11 16:27); the object is not flown in this test, so move it to plate 3, or ask the supervisor for a third | QUAD3's catch needs the same guide as QUAD4's (11 Nov: the cone bands 5/5 cm caught on it) |
| 2 | Equal funnel masses on 3 and 9 | Two of the supervisor's, or two v4 (83 g), never one of each | One-sided funnel mass leans the pair 3-9 deg (11 g) or 22-48 deg (83 g) about the 0-6 line (section 1.3) |
| 3 | QUAD3's magnet catches on contact like QUAD4's | All four are the same 20 x 20 mm electromagnet on its holder, switched on AUX4 through each quad's own radio; they swap between quads (29 Oct). Check on the bench: QUAD3's holder is the same 64 mm disc the funnel's bore centres, its wire fitting clears the funnel's base (v3's 72 mm base did not clear QUAD4's, 6 Nov), its lead is tight (QUAD4's loose lead lost nrtfc_3 and nrtfc_4 on 11 Nov), and it catches the plate with the coil ON from 1-2 cm | the rig magnet catches only on contact (10 Oct) |
| 4 | Magnet hold, all four | Luggage-scale pull of each magnet off an M2A plate, straight up; the two strongest on the pair (0/6). Each pair magnet should hold well over the 4.9 N static pull | pair magnets let go at 6.4-6.5 N in the 21-22 Oct lifts; QUAD1's magnet was the weak one on 29 Oct |
| 5 | Mocap marker on QUAD3's magnet | Already there: magnet markers 21-24 sit on QUAD1-4's magnets (RIG-1028-payload; 21 swaps with its magnet). Enable body 23 in Motive, stream `8,11,12,13,14,23,24`, nothing else; stream check `python3 tools/mocap_stream_check.py --expect 8,11,12,13,14,23,24` must say OK, Motive not recording. Body 23's marker-to-face offset must be measured in the plate-3 funnel (the "QUAD4 record" steps, for QUAD3) | his planner and our contact proof read the magnet tip; at most 8 bodies at 240 Hz (30 Oct) |
| 6 | Radios | Nothing new: the four ELRS transmitters (`/dev/QUAD1-4`) stay on our stack. His second instance's MPC goes through our mux to QUAD3's radio as his first does to QUAD4's. Never start his IRL radio launch (one stack per radio) | 6 Nov crash rule |
| 7 | Packs | Fresh, 24.0 V or more at rest, on all four every flight; a fresh pair set (QUAD1, QUAD2) every flight. QUAD2's pack sagged to 18.9 V under load on 13 Nov (fm9c_2): balance-charge it and check its cells, or retire it. ~3-4 min in the air per flight; QUAD3 hovers parked ~2 min in rung B | the 28 Oct pair's sink and swing followed the pack voltage (f2b sank 166 mm on 21.6-21.8 V) |
| 8 | Re-hang | QUAD1's magnet to plate 0, QUAD2's to 6, QUAD3's to 3 (in the funnel), QUAD4's stays on 9; quads laid out on their plate lines | the square |
| 9 | Current draw (the supervisor's extra metric) | `elrs_interface` already decodes the CRSF battery frame's current (`Battery: V A mAh`, printed at most once a second) but the Telemetry message carries only voltage and mAh. The flight log's lines, or the mAh slope, give current per quad with 4, 3 and 2 carrying | 12 Nov list: current per drone with 4 and 3 carrying |

## 3. What the rig showed for a pair (and the T)

### 3.1 Pairs on the rig: all on the 0.86 kg M2A ring with 0.55 m links

| Day | Runs | Quads, plates | Filter | Outcome |
|---|---|---|---|---|
| 21 Oct | RIG-1021-c2q, -c2q_noxy_3, -c2q_noxy_4, -c2q_p11p5_2 | QUAD2/QUAD3, 0/6 then 11/5 | ema | every lift rocked (see-saw and roll about the pair's line up to +-11 deg); QUAD2's magnet let go in 3 of 4 (one at a 6.5 N pull, one ending in a 58 deg tilt drop), the fourth ended by the operator; QUAD2's magnet let go 6 times that day, all in lifts |
| 22 Oct | RIG-1022-h2, -h2t9p, -h2m, -h2rts | QUAD3/QUAD4, 0/6 then 11/5 | ema | the 1.3-1.6 Hz swing grew in every lift; h2: QUAD3's magnet let go at 6.4 N, tilt 62 deg, drop and land. Not thrust (throttle never above 0.8), not the plates, not the measured-pull feed-forward |
| 22 Oct | RIG-1022-h2slope, -f2slope | QUAD3/QUAD4, 11/5 | slope | the swing gone (first calm pair); f2slope the first full rig figure-eight with a pair: tilt peak 14.3, mean on the path 7.3 deg, "the ring hangs ~7 deg tilted about the pair's line" |
| 28 Oct | RIG-1028-hS1, -hS2, -f2a, -f2b, -f2c, -ls2a, -ls2b_4, -ls2b_5, -ls2c | QUAD2/QUAD3, 11/5 | slope | 9 of 9 held the ring. Hover: tilt mean 2.0-2.1, peak 4.4-5.4 deg. Paths: tilt mean 3.3-9.8, peak 14.0-23.5. Throttle peak 0.57-0.69 (cap 0.9). Sinks: f2b 166 mm (lowest packs), ls2c 57 cm at the line end (the 14.5 m/s^2 pull cap clipped the feed-forward) |

### 3.2 The chain lines that were to reach a pair

| Run | Ring, links | What happened | Problem |
|---|---|---|---|
| RIG-1022-t3p_2 | M2A, 0.55 m, T on 11/2/5 | all three magnet markers armed; the release command never reached the radio (T4's ROS environment) | void |
| RIG-1022-t3p_3 | as t3p_2 | the release went to a pair quad (the sheet assumed QUAD2 on plate 2; the matcher put it on 11): 270 deg gap, drop and land, tilt 49.8 deg | **a DETACH by a fixed drone id can hit the wrong quad: read the slot line first** |
| RIG-1029-ch1 | 1.15 kg, 1 m, square 0/3/6/9 | mid-lift (ring 0.66 m) plate 0's magnet (QUAD1) lost its pull at ~4 N: drop and land, tilt 48.5 | "plate 0 released mid-lift" (also h4) |
| RIG-1029-ch1b | as ch1, two 2 Hz stages | the first release ran (QUAD4 unloaded, detached); 0.4 s later plate 0 let go at ~5.2 N: drop and land | the plate-0 magnet again (3 of 3) |
| RIG-1029-ch1b_2 | magnets of QUAD1 and QUAD4 swapped | 4 s after QUAD4's release the T sank 0.86 -> 0.70 m at the 0.9 cap; the trigger was the swapped marker (most likely QUAD4's own released magnet): inconclusive | the T out of thrust on the 1.15 kg ring |
| RIG-1029-sq1 | 1.15 kg, T 0/3/6 + QUAD4's attach | plates 0 and 6 at 0.82-0.90 throttle (cap 0.9), the ring reached 0.90 m, tilt 33.6 deg: drop | the T (a pair plus a rider) cannot carry the 1.15 kg ring on 1 m cables |

None reached a pair. On the 0.86 kg ring the pair's throttle is ~0.68-0.72 (section 1.3), well under the 0.95 cap.

### 3.3 The T on the M2A ring with 1 m cables

RIG-1105-sq8_2 (5 Nov, square 11/2/5 + QUAD4 on 8, sideways integral on): DETACH, release, his re-attach caught on try 1
(17 mm, +0.4 mm), folded in 14.4 s after the release: the first real detach and re-attach. While QUAD4 was away **the T
leaned 21-28 deg** (twin ~8.5); the 55 deg drop is what let it finish. In the twin the T's lean is the sideways integral:
median 8.5-14.3 deg with it on (R3706, R3708, R3712-R3715), 1.9-2.0 with `int_k_xy 0` (R3707, R3709, R3740). Hence
`int_k_xy 0` on these lines.

### 3.4 The twin, for the same steps (M2A ring, 1 m links)

| Run | Steps | Read |
|---|---|---|
| R2821, R2822 | square -> T (soft) -> pair, our planner, hover | T release peak 29.6 / 29.0 deg, the rider's release 2.2 deg, the pair 0.5-0.6 deg |
| R2829 (27 Oct), R3743 (5 Nov, 4 Nov links) | `sq_chain_soft_rejoin_hover_L100(_rigargs)`: square, DETACH QUAD4, the plate-3 carrier detached 7 s later, the pair alone, his re-attach of QUAD4 on 9 -> T 0/6/9 | re-welded on try 1 (R2829 37.8 s after his TAKEOFF; R3743 17 s after the resize to the pair); tilt at the re-weld 0.1 deg, peak in the next 10 s 9.9 / 5.6 deg; T and pair 0.6-1.5 deg (int_k_xy 0) |
| R3748 | as R3743, the fold-in cap 'level' | peak 2.8 deg (the cap stays off by decision, 5 Nov) |
| X2 R3293-R3298, R3586-R3591, R3986-R3991 | square -> T -> pair -> T -> square on the figure-eight, our own re-attach approach | every run flew all five stages and landed; the pair's lean 0.74-0.85 deg |

## 4. The twin

### 4.1 What exists

| Piece | File | Use here |
|---|---|---|
| The square, our planner only, our own re-attach | `simulation_assets/four_rigid_ground_rig_L100.sdf`, `configs/experiments/x2_full_fig8_*.yaml`, `configs/twin/x2_chain_tip80.yaml` | the whole loop with our approach (twin and thesis only), no his planner |
| The square for his planner: carriers 0/3/6 (drone_0/1/2), QUAD4 welded on 9 | `simulation_assets/sq_rigid_ground_rig_attach_welded_pickup_L100.sdf` (`tools/make_attach_layout_worlds_L100.py`, layout 'sq') | rung B as is: `configs/experiments/sq_chain_soft_rejoin_hover_L100_rigargs.yaml` (R3743) and `sq_nr_hover_L100_1104_rigargs.yaml` (R3708) are the templates |
| The pair + QUAD4 | `pair_rigid_ground_rig_attach_welded_pickup_L100.sdf` | unusable today: R2828 void, "graph_attach.ATTACH_DRONE_ID = 3: the newcomer must follow three carriers" |
| The rig-twin args of today's attach line | `configs/experiments/rig_nrtf_twin.yaml` | his launch line, bands, events |

The newcomer is hard-wired as one drone in the twin and on the rig: `graph_attach.py` (`ATTACH_DRONE_ID = 3`, the rig
refuses `num_drones` other than 3 with the approach chain), `make_attach_worlds.py` (`ID = 3`, one DetachableJoint on
`/payload/attach|detach`), `partner_mission.launch.py` (`NS = 'tejen'`, `DRONE = 3`, global topics `/join_planner/*`,
`/magnet/contact`, `/magnet/object_attached`, `/partner/release`, `/magnet_tip_pose`), and our planner's partner wiring
(`dissipative_node.py`: `partner_contact` subscribes one newcomer `d = self._n_carry0`, one `attach_x/y_offset`). Our
announced detach is one drone at a time (`_start_unload`: a second DETACH during an unload is refused).

### 4.2 Files to create

For rung B and the probes (no new code):
- `configs/rig/attach_m1_1m_m2a_drop55_det15_pair.yaml` (rig and twin; section 1.4).
- `configs/experiments/rig_424b_seq_twin.yaml` (from `sq_chain_soft_rejoin_hover_L100_rigargs.yaml` + `rig_nrtf_twin.yaml`'s
  current his args), `rig_424a_hold_twin.yaml` and `rig_424a_rel2_twin.yaml` (from `x2_full_fig8_soft.yaml` in hover,
  with the pair params instead of `x2_chain_tip80.yaml`).
- Optional, to mirror the rig's ids exactly: one line in `tools/make_attach_layout_worlds_L100.py`
  (`'sq3': ('0,180,270', '0/6/3')`) -> `simulation_assets/sq3_rigid_ground_rig_attach_welded_pickup_L100.sdf` (drone_2 on
  plate 3, as QUAD3 on the rig). The existing 'sq' world has drone_1 there, which the twin does not mind.

After the two-drone DETACH is built: `configs/experiments/rig_424b_det2_twin.yaml` (the runner's DETACH taking "3,2", or
two events at one time if the code accepts the second during the first's unload).

After the two-newcomer stack is built:
- `tools/make_attach_worlds.py`: `newcomer_model(start, id, plate)` (today `ID = 3` and `WELD_Y` fixed), per-id
  DetachableJoint topics (e.g. `/payload/attach_2|detach_2`) and pose publisher; no pickup weld on drone 2.
- `tools/make_attach_layout_worlds_L100.py`: layout 'pair2' -> `simulation_assets/pair2_rigid_ground_rig_attach_welded_L100.sdf`
  (lift_system with carriers on 0/6 = drone_0/1; `x3_drone2` welded on plate 3 at (0, -0.225); `x3_drone3` welded on
  plate 9) and `models/x3_rig_drone2_magnet_welded.sdf`.
- `configs/experiments/rig_424_seq_twin.yaml`, `rig_424_sim_twin.yaml`, `rig_424_rev_twin.yaml`, `rig_424_land_twin.yaml`
  (fleet `num_drones 2`, `n_total 4`; two LAUNCH events of his mission, one per instance).
- Runner (`tools/experiment/config.py`, `runner_node.py`): WAIT_REWELD per drone (or WAIT_LOG on the resize to n = 4),
  HIS_LAND and FORCE_DROP per instance.
- A scorer for this test (tools/): tilt per stage (square, pair, T, square), each catch's descents and misses, the least
  distance between the two returners (bodies and cables), each returner to the pair's cables, the pair's peak cable
  pull against the rig's 6.4-6.5 N release level, the pair's throttle.

### 4.3 Test matrix (exploratory: one run per variant, judged after the run)

| ID | Config | World | What it flies | Needs | Runs | Sim s | Wall each (laptop / lab slot) |
|---|---|---|---|---|---|---|---|
| T0 | rig_424b_seq_twin | sq (or sq3) | **rung B as the rig flies it:** ring 1.0 m, DETACH QUAD4 (3 s unload), 7 s later DETACH the plate-3 carrier -> pair; his re-attach of QUAD4 on 9 with today's nrtf args except `contact_dz` left at his default, as R3743 (see below); 10 s on the T; LAND | nothing new | 1 | ~110 | ~12 min |
| T1 | rig_424a_hold_twin | four_rigid_ground_rig_L100 (our planner only, dissipative mode, the pair params) | the pair alone: DETACH the plate-9 quad (3 s unload), 7 s later the plate-3 quad; 30 s on the pair; LAND (the freed quads land alone) | nothing new | 1 | ~120 | ~10 min |
| T2 | rig_424a_rel2_twin | as T1 | twin-only probe: both magnets released unannounced at once (two RELEASE events at one time): square -> pair with no T; the pair's peak pull; whether the planner takes two losses in one tick | nothing new | 1 | ~90 | ~8 min |
| T3 | rig_424b_det2_twin | as T0 | T0 with one DETACH of both (together) | two-drone DETACH | 1 | ~110 | ~12 min |
| T4 | rig_424_seq_twin | pair2 | **rung C:** DETACH both; his A re-attaches QUAD4 on 9, then our planner releases QUAD3 to his B, which re-attaches on 3; 10 s on four; LAND | two-newcomer stack | 1 | ~200 | ~25-30 min |
| T5 | rig_424_sim_twin | pair2 | both returns at once | as T4 | 1 | ~170 | ~25 min |
| T6 | rig_424_rev_twin | pair2 | succession the other way: QUAD3 first | as T4 | 1 | ~200 | ~25-30 min |
| T7 | rig_424_land_twin | pair2 | our LAND during B's descent with A on: B taken back, everyone lands | as T4 | 1 | ~150 | ~20 min |

Neighbours, per the working rules: T0 is T4 with one fewer drone, T6 the other order, every run a floor start. No
trajectory: the test is a hover limit test.

**The twin's contact artifact.** Since 11 Nov the twin has not re-attached on the nrtf-type lines (nrtf, nrtf BAND=cone,
nro, nr1f: R4233-R4237; at 1.2 m R4243): with `contact_dz -0.03` the twin's rigid rod pushes the centred tip into the
plate, outside the twin's contact window (FACT 2026-11-11 06:38). R3743, which caught onto the pair, typed no contact_dz.
So the twin rows fly his default contact_dz and the rig flies -0.03, as today; a twin row with -0.03 would only show the
artifact.

**Time.** Measured, not the 4-6 min guess: the rig-twin attach runs of 10-11 Nov flew at RTF 0.15-0.19 on the laptop,
90-160 s of sim in 9-16 min of wall time, and 41-43 min when the re-attach did not complete (~460 s, R4233-R4237).
Two of his stacks will be slower (RTF ~0.12-0.15 assumed). Laptop, one at a time, idle for the first ~3 min of each:
T0-T3 ~50 min, T4-T7 ~1.5-2 h. Lab PC (up to 4 slots when nobody is on it): T0-T3 in one wave ~15-20 min, T4-T7 in one
wave ~30-35 min (~60 min if a re-attach runs to its timeout). Before T4: pytest, `tools/gate.sh --quick`, then the
desk check of the rig script (`results/rig/2026-11-11/desk/evalscript.sh`).

**What the twin cannot show here:** downwash (the pair-held ring's roll under a returner), a magnet pulling the plate
up or letting go under load (its welds are rigid), the funnel, the slack metal cable (its links are hinged rods). It
does show the contact push on a freely rolling pair, the stage tilts, the clearances and the order of events.

## 5. Draft sheet rows (18 / 19 Nov)

Fits after the minimum items (the re-attach count is still 2, nrtfc_2 and nrtfc_5; the full mission flew once,
fm9_5). On the 18th: the triangle lines first, then, with ~45 min left, the setup below and row 1. On the 19th: rows 1-2
as far as built and twin-checked. Rung C (row 2) flies only after T4 and T7 have run and the desk check is OK.

**Setup (once, ~30 min, props off unless said):**
1. Re-hang to the square: QUAD1's magnet on plate 0, QUAD2's on 6, QUAD3's on 3 (in the second funnel), QUAD4's on 9 (in
   its funnel). Quads laid out on their plate lines ~1.25 m out, the ring at the cage centre, plate 0 toward +x.
2. Magnet hold: luggage-scale pull of each magnet off a plate; the two strongest on 0/6 (swap assemblies if QUAD1's is
   the weakest; type the returner's tip body).
3. Pair-hang check: QUAD1's and QUAD2's magnets ON on plates 0/6, lift the ring ~20 cm by their two cables and read body
   8's roll about the 0-6 line in RViz or Motive. Over ~3 deg: tape a small trim mass on the high side, on the 3-9 line.
4. Rung C only: QUAD3's record in the plate-3 funnel (T1 `tools/t1_four_markers.sh detach:=true`, T2 `python3
   tools/payload_geometry.py`): W, X, Y; its taut cable length L.
5. Motive: only `8,11,12,13,14,24` (row 1) or `8,11,12,13,14,23,24` (row 2); `python3 tools/mocap_stream_check.py
   --expect <set>` OK; Motive NOT recording.
6. Fresh packs (24.0 V or more at rest) on all four; QUAD2's sagging pack out.
7. Console `python3 tools/console.py --small-angle 90 --slot-gate`. At launch, read the slot line: d0 on plate 0, d1 on
   6, d2 on 3, d3 on 9. Any SLOT OFFSET: stop and re-place (13 Nov: two launches stopped for it).

| # | Run | What it does | Layout | Buttons, in order (console) |
|---|---|---|---|---|
| 1 | `bash ~/rig_424b.sh` | **4 -> 2 -> 3 in hover:** the four lift the ring to 1.0 m; DETACH QUAD4; DETACH QUAD3 -> the pair QUAD1/QUAD2 on 0/6 carries alone (the rig's first pair on 1 m cables); his re-attach of QUAD4 on 9 onto the pair -> the T 0/6/9; QUAD3 parked ~1.2 m out on plate 3's side until LAND | the square, both funnels, QUAD3 and QUAD4 welded on 3 and 9 | His ARM, ARM, TAKEOFF, his TAKEOFF. Steady ~10 s: DETACH QUAD4. **~7 s later, once the T has formed: RViz DETACH, drone 2 (QUAD3)** -> the pair. QUAD4: CONTACT, `ATTACH: proven`, `FLEET RESIZED to n=3`. ~10 s on the T: LAND |
| 1b | `TOGETHER=1 bash ~/rig_424b.sh` | as 1 with one press for both detaches (no T on the way down) | as 1 | As 1, one press: DETACH QUAD3+4 |
| 2 | `bash ~/rig_424.sh` | **4 -> 2 -> 4 in hover:** both detach; his A re-attaches QUAD4 on 9 (-> T), then our planner releases QUAD3 to his B, which re-attaches on 3 (-> the square) | as 1, body 23 streamed | His ARM A, his ARM B, ARM, TAKEOFF, his TAKEOFF A, his TAKEOFF B. Steady ~10 s: DETACH QUAD3+4 (or the two presses of row 1). A: `FLEET RESIZED to n=3`; B: `FLEET RESIZED to n=4`. ~10 s on four: LAND |
| 2b | `SIM=1 bash ~/rig_424.sh` | as 2, both returns at once | as 2 | As 2; both returners start at the release |

Only rows 1 and 1b's first press exist on today's code; 1b and 2 need the builds in `ours.md` / `tejen_stack.md`, the
twin runs T3 / T4-T7 and a desk check. Row 2b only after row 2 has caught both at least once and T5 kept the returners
apart.

**Stop rules** (with the sheet's usual ones and the LAND defaults; the automatic drop at 55 deg, the position limit
at 1.8 m):
- The ring past ~40 deg: LAND.
- The pair: LAND if it leans past ~25 deg about the 0-6 line, if a swing grows at ~1-1.5 Hz (as 21-22 Oct), if a pair
  quad sits at the 0.95 throttle cap for more than ~2 s, or if the ring sinks more than ~10 cm (28 Oct f2b, ls2c).
- QUAD4 hovering or descending over plate 9 of the pair-held ring: if the ring rolls past ~20 deg, GIVE UP (his give-up
  takes QUAD4 back, the pair hangs on), then LAND.
- A returner within ~0.3 m of a pair quad off its plate line, or (row 2b) the two returners closer than ~0.4 m off their
  final descents: LAND.
- No catch ~120 s after a returner's first descent: his give-up takes it back.
- DETACH only by the slot line's ids (RIG-1022-t3p_3 released the wrong quad).

**Reading a flight (notes, not gates):** each stage on its own: the 4 -> 2 transition (peak tilt, time to settle, the
pair's peak pull and throttle), the pair (lean about its line, swing), each return (descents, misses, time from the
release, the tilt at the catch and in the 10 s after), the last fleet (lean), the whole time from DETACH to the last
fold-in, and current per quad with 4, 3 and 2 carrying (section 2 item 9).

## 6. For Wesley (recommended first)

1. **Layout:** the square, the pair QUAD1/QUAD2 on 0/6, QUAD3 on 3 and QUAD4 on 9. The alternative (no re-hang, the
   pair 3/9) makes both returners quads his planner has never flown onto plates it has never targeted.
2. **Rung B first** (4 -> 2 -> 3, today's graph, detaches in succession, twin R3743): it is the rig's first pair on 1 m
   cables, and it tells whether the pair-held ring holds still enough for a catch before rung C is worth its build.
3. **`int_k_xy 0`, `drop_gap_deg 200`, `min_survivors 2` in a new params file** for these lines (as `chain_pair_attach_1m.yaml`,
   which you approved on 27 Oct), the drop at 55 deg.
4. **Push-back on "together" for the returns:** physically good (the loads cancel about the pair's line) but the two
   bodies end 0.45 m apart and his planner does not see the other returner. Fly the returns in succession first; together
   only after a caught succession and the twin's T5.
5. **Ask the supervisor** for a second (or third) 105 mm funnel if the object lines fly on the same day.
6. **QUAD1's magnet:** replaced since 29 Oct? If not, the bench pull decides whether it stays on the pair.
