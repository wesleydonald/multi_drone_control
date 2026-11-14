# Making the real-world magnetic re-attach catch more often: ideas for the four sessions left (6 Nov 2026)

Desk analysis for the session plan's last item (docs/rig/plan_sessions_2026-11-11_to_19.md, "ideas to help the attach").
Exploratory: read from the 4-5 Nov descents already scored (results/analysis/rig_joins_2026-11-04_05/: joins.csv,
funnel_catch.csv, summary.md, funnel_catch.md), Te-Jen's planner in our fork, cad/funnel/funnel.py and the papers in
docs/papers/. No flight, no bag read, no code or config changed. Numbers were judged after the flights. No bars.

Evidence codes used below: **D** measured on the rig (4-5 Nov, exploratory), **A** arithmetic or geometry on measured
or drawn numbers, **P** a paper, **G** a guess (not measured).

## Verdict

The bare plate cannot be made reliable by settings in four sessions. QUAD4's magnet lands with a random scatter of
about 5.5 cm (radial) by 3.6 cm (sideways) rms, with no bias, and its position re-draws within about a second. The
plate only catches within about 2 cm (it sits 3 mm down in a 57 mm hole). That is about 5-9 % per descent, which fits
5 Nov's 0 of 19. Only a capture element larger than the scatter changes this. **Fit and fly the funnel (v3 first, v2 as
the spare) as the only change, after a 30-45 min bench check.** A flush steel oval is the fallback. Software gates and
cable changes come after, and only if the funnel's mouth is missed often.

## The numbers that drive the ranking

| # | What | Number | Source | What it means |
|---|---|---|---|---|
| N1 | Tip offset at plate height, 36 plate descents | radial rms 5.5 cm, sideways rms 3.6 cm, mean radial -0.2 cm | D, joins.csv | A random scatter, not a bias: no fixed offset or learned trim can remove it |
| N2 | Split into QUAD4's body (vs the plate) and the swing (tip minus body) | body 3.3 radial / 3.1 sideways; swing 3.4 / 2.1; corr(body, swing) radial +0.38 over all 36 (+0.47 in the misses), sideways -0.11 | D, joins.csv | Each is about half; radially they move together (in phase), so they add |
| N3 | His reference vs the plate | rms 1.5 radial / 1.0 sideways; corr with the tip +0.06 | D, joins.csv | His planner aims right; his measured-tip trim neither helps nor hurts measurably |
| N4 | The tip ~1 s before plate height (at the funnel mouth planes, 58-80 mm up; 0.8-1.15 s before) vs at plate height, 26 descents | radial corr 0.21-0.40, rms change 5.5-6.4 cm; radial rms 4.8-5.1 cm up there vs 5.2 cm at the plate; sideways 3.4-3.9 vs 3.8 | D, funnel_catch.csv + joins.csv | The landing point re-draws within ~1 s and the scatter is the same 6-8 cm up as at the plate. A gate more than ~0.3 s before contact cannot pick it; a slower or lower descent does not aim it |
| N5 | The same QUAD4 over the pickup object (floor, ~1.5 m from the ring), hover, 10 descents | body 3.0, swing 2.3, tip 4.4 cm rms per axis (plate, hover: body 3.2, swing 2.8) | D, joins.csv | The scatter is QUAD4 on its 1 m cable, not the ring or the carriers' wash |
| N6 | Where catches happened | on the plate 0.3-1.9 cm; 6 contacts declared at 2.4-5.9 cm (4 Nov, 60 mm radius) all lost | D, summary.md | The bare plate catches within ~2 cm, not its 2.75 cm radius |
| N7 | Plate 1 vs plate 9 (different world directions) | radial rms 5.2 / 5.6, sideways 3.1 / 3.8 | D, joins.csv | The radial excess turns with the plate: not a mocap or controller axis |
| N8 | First descent of a flight vs later ones; consecutive radial misses | median 6.2 vs 6.6 cm; same side 7, opposite side 8 | D, joins.csv | A retry is no worse; no sign that lifting off the edge kicks the next descent |
| N9 | 5 Nov triangle, descents within r of the centre | 1.9 cm: 0/19; 4 cm: 4/19; 8 cm: 12/19 | D, joins.csv | 5 Nov was the worse day |
| N10 | Funnel mouth, geometry only, the 24 triangle plate-9 descents | v2: 20 into the mouth, 11 fully inside; v3: 22 / 15 | D, funnel_catch.md | The funnel would meet most descents |
| A1 | The bare plate's recess: a 20 x 20 mm magnet (corner 14.1 mm) into the 57 mm skin hole, steel 3 mm down | the full face reaches the steel only within 14-18 mm of the centre | A, funnel.py | Matches N6; the magnet needs contact (no pull-in gap) |
| A2 | Catch chance per descent from N1's spread (Gaussian) | 1.4 cm: 5 %; 1.9 cm: 9 %; 2.75 cm: 18 % | A | 5 Nov's 0/19 fits ~5 % |
| A3 | Per flight, ~9 descents in the 120 s give-up (nr9m, 5 Nov) | at least one catch: 37 % at 5 % per descent, 74 % at 14 %, ~100 % at 36 % or more | A | On the bare plate, 5 static attaches would take more than one session |
| A4 | What if QUAD4 were better (recomputed from N2's parts) | within 1.9 cm: as flown 4/36; no swing at all 3/36; half the swing 5/36; half of both 9/36 | A | Even a twice-better QUAD4 leaves the bare plate near 1 in 4 |

## Ranked ideas

| Rank | Idea | Kind | Why it should help (data) | Evidence | Effort, who | Risk | Ready |
|---|---|---|---|---|---|---|---|
| 1 | The funnel on plate 9, v3 first, v2 spare | hardware | Catches by mouth size, not aim (N4, N10) | D (geometry), P | printing now (Wesley); fit ~1 h | medium: never flown | Wed 11 |
| 2 | Bench check of the funnel before its first flight | procedure | Its open risks are cheap to see by hand | A, G | 30-45 min, Wesley at the rig; list below | low | Wed 11, before rel |
| 3 | Fly the funnel as the only change; two typed contingencies | typed | The gates do not aim (N4); one variable | D | sheet lines (lab chat) | low | Wed 11 |
| 4 | Fallback: a flush steel oval ~200 x 140 mm | hardware + typed radii | Catch area by size (24/36 by geometry) | D (geometry), G (force) | workshop 1-2 h, force test, typed K, twin check | medium-high | Wed 18 |
| 5 | Swing gate in MATCH: the tip's speed over the plate, not the body's | code (his fork) | The swing is half the scatter (N2) | D (part), G | ~1 day: code, tests, twin, desk check | medium | Wed 18, only if the mouth is missed often |
| 6 | "Pounce": hold the face 2-3 cm up, drop fast on predicted alignment | code | The only software answer to N4 on a bare plate | G | 1-2 days | high | after hand-in |
| 7 | Attach from a lower hover (approach 0.15-0.20 m) | typed | No aiming gain (N4); saves 1.2-1.9 s per descent | D | typed | low | not recommended now |
| 8 | Descend slower near contact | code | No aiming gain (N4); may soften the cone entry | G | his fork, two-rate descent | low-medium | Wed 18, only if bounce-outs are seen |
| 9 | His measured-tip trim off or slower | typed | Nothing to learn (N1 mean -0.2 cm, N3 corr 0.06) | D | typed | low | no change |
| 10 | Shorter or stiffer link, swing damper, heavier holder | hardware + geometry | Halving the swing adds ~1 catch in 36 (A4) | A | geometry, his MPC length, twin, planner | high | not in these sessions |
| 11 | Carrier layout (the square) and their downwash | layout | The square's catch fits chance; the wakes land outside the ring (N5) | D, A, G | none | none | keep the triangle |
| 12 | QUAD4's own position hold (his MPC) | his code | The body is the largest single term (N2) | D, G | Te-Jen | high | after hand-in; offline check now |

None of these touches the flight controller (no Betaflight change), QUAD4's route (it never goes over the carriers),
or a gate threshold or the attach geometry without Wesley's word.

## 1. The funnel on plate 9

**What.** cad/funnel/funnel.py, bolted through three of plate 9's four M4 holes (the zip-tie hole clear), a 72 mm bore
over the plate for the 64 mm disc, an oval cone above it, long axis radial (where the misses are, N1).

| | v2 | v3 |
|---|---|---|
| mouth (full, radial x sideways) | 174 x 144 mm | 214 x 164 mm |
| height above the ring top | 75 mm | 97 mm |
| wall angle, radial / sideways | 45 / 55 deg | 45 / 57 deg |
| wall across | 1.6 mm, four ribs | 1.2 mm, two radial ribs |
| mass (PETG) | 93 g | 97 g |
| tip height above the weld point at the mouth plane | 58 mm | 80 mm |
| plate-9 triangle descents into the mouth / disc fully inside (N10) | 20 / 11 of 24 | 22 / 15 of 24 |
| inward rim edge from the ring centre | ~138 mm | ~118 mm |

**Why.** N4 says no gate or descent setting can choose where the magnet lands. A capture element bigger than the
scatter is what every working docking system in the papers uses. TRADY's magnetic dock (Sugihara 2023) uses a drogue of
5 cm radius "to absorb errors within 2.5 cm", with a 35 deg inner gradient, and docked 13 of 15 times. Shankar 2020
grasps with a ~16 cm gripper opening and still needs 3-4 approaches per grasp indoors. Gawel 2017 picks 150-165 mm
ferrous plates with a compliant magnetic gripper. Antal 2026 catches with a hook whose capture region is larger than its
error. The v3 mouth half-axes (107 x 82 mm) are about 1.9-2.3 times N1's rms. The funnel also re-centres: it is the
"catch then re-centre" idea done mechanically, so the weld point, our contact radius (25 mm) and the planner's geometry
stay as they are.

**Fit (A).**
- *Steel under the bore: yes.* The bore lets the disc sit at most 4 mm off its axis. Add ~1.5 mm for the M4 holes (4.5
  mm), the print and the plate in its hole. The magnet's corner is then at most ~19.6 mm from the plate centre. That is
  inside the steel (27.5 mm radius) and the skin hole (28.5 mm), so the whole face lands flat on the steel, ~8 mm clear.
- *The 4 mm play is right.* Up to ~8 mm would still put the whole face on steel. Under ~2 mm risks binding, because
  printed holes come out 0.2-0.4 mm small (G). Check by hand that the disc drops through the bore freely.
- *Cables clear the walls.* A folded carrier cable runs at 60 deg elevation (30 deg from vertical). The radial wall is
  45 deg, so there are 15 deg to spare on a level ring. Triangle tilts at the readings were 0.2-8 deg (D). A ring tilted
  over ~15 deg toward the cable would put it on the rim. The T leaned 21-29 deg (sq8_2), so the funnel and a T do not
  mix.
- *Release.* A pull steeper than the wall lifts the disc off it. QUAD4's cable at 30 deg from vertical takes the disc
  out without touching (rel's unannounced release included). Only a sideways drag at funnel height rubs the wall.

**Risks.**
- *Motion capture (G, the largest unknown).* Seated, the disc sits 17-20 mm above the ring top, inside a cone 75-97 mm
  tall. Body 24's markers are visible only to cameras looking down into the cone. If body 24 drops out, his tip gates
  stall and our co-motion proof goes stale. A glossy PETG cone may also reflect the cameras' strobes near the ring's
  own markers. Both are bench checks (section 2). Prefer matte black tape over Motive masking, which would also hide
  the ring's markers.
- *A miss outside the rim (G).* The disc can hang beside the rim. On the lift the cable can catch under the overhang or
  on a rib. Check by hand. In flight, his "sunk below the plate" rule cannot fire inside the cone. Outside it, it fires
  as it does now.
- *Mass and wash (A, G).* 93-97 g at plate 9 rides on carriers 7 and 11 in the triangle (Wesley 5 Nov: no ring-mass
  change). Watch the first lift's tilt. Under QUAD4 the cone is also a cup in its wash: ~0.1-0.2 N more down on plate 9
  at an assumed 2-3 m/s wash (G).
- *The drop into the net (fm9, fm9h; A, G).* The inward rim reaches ~118 mm (v3) or ~138 mm (v2) from the ring
  centre, about 75-97 mm up. QUAD4 arrives along plate 9's line and drops as the magnet enters 0.12 m of the centre, so
  the release is on plate 9's side. The 5 Nov drops hit 6.3-13.3 cm from the centre (summary.md). A 70 mm object
  centred beyond ~8 cm (v3) or ~10 cm (v2) toward plate 9 would clip the cone. Before the fm9 lines, read the impact
  direction from the 5 Nov bags. If the drops lean toward plate 9, type a smaller `drop_on_arrival_xy` (Wesley's call)
  or fly v2.
- *Fragility (G).* v3's 1.2 mm wall is the weaker one if QUAD4 drags the disc hard.
- *The paper.* An attach with the funnel is a different setup from 4-5 Nov's bare plate. Report both: 1 re-attach
  caught in 43 bare-plate descents, then the funnel's count.

**Which one.** v3 if body 24 tracks inside it and the drop check allows it: 2 more of 24 into the mouth, 4 more fully
inside. Otherwise v2: shorter, sturdier, and further from the net.

## 2. Bench check before the funnel's first flight (Wed 11, in the setup time, before rel)

Fit the funnel before the day's first flight, so every Wednesday flight (rel, nrtf) shares one setup. What to look
for, judged on the spot (no frozen bars):

| # | Check | How | What to look for |
|---|---|---|---|
| B0 | The bare plate's real catch radius (before fitting) | coil ON, lower the magnet by hand onto plate 9 at 1, 1.5, 2, 2.5, 3 cm off centre (radial and sideways), lift ~3 cm | Where it stops holding. A1 predicts 1.4-1.9 cm. This number goes to the thesis and to idea 4 |
| B1 | Body 24 inside the cone | disc seated, then moved by hand round the cone, Motive running as flown (bodies 6, 8, 11-14, 24) | Body 24 tracked throughout, no frozen or dropped frames. If not: v2, or a taller marker post (that redefines body 24, so W, X, Y and L are re-measured) |
| B2 | Reflections | Motive 2D views, funnel on, cameras as flown | No new blobs near plate 9 or the ring's markers. If any: matte black tape on the cone |
| B3 | W, X, Y with the magnet seated | the 4 Nov evening method | The new values go in the sheet (the plan already asks for W) |
| B4 | Hand drops | hold the cable ~1 m up, coil ON, lower at ~0.08 m/s at offsets out to the mouth (radial up to +-9 cm, sideways up to +-7 cm) with a small swing; ~10 drops, 3-4 on the rim | It slides in and seats; whether drops onto the rim bounce out |
| B5 | Release | coil OFF; pull up at ~30 deg outward (folded cable), straight up, then at 45-60 deg | Clean exit at 30 deg and vertical; note where it drags at 45-60 deg |
| B6 | Miss and lift | let the holder hang just outside the rim (out, in, sideways), lift straight up and at 30 deg | No catch under the overhang or on a rib |
| B7 | Cable over the rim | ring tilted ~10 deg by hand, a carrier-like cable at 30 deg from vertical over plate 9 | The cable does not touch the rim |

## 3. Fly the funnel as the only change; the settings stay as flown (his match, dwell and descent)

nrtf = nr9m plus the funnel. Each of his settings, and why it stays:

| Setting (typed on 5 Nov) | Value | With the funnel | Why |
|---|---|---|---|
| `match_xy` / `match_speed` / `match_dwell` | 0.08 m / 0.20 m/s (body) / 0.5 s | keep | N4: the start gate cannot pick the landing point 4 s later. Tightening it only costs attempts (Wesley's concern). match_dwell was decided 5 Nov |
| `approach_offset_z` | 0.30 m | keep | The disc starts 22 cm (v3) or 24 cm (v2) above the mouth |
| `contact_dz` | -0.03 m | keep | The body aims 3 cm under the seat, so the cable goes slack and the disc slides down the wall under its own weight (PETG friction well under the 45 deg wall's 1.0, G) |
| `descent_hold` / `descent_abort_xy` / `descent_debounce` | on / 0.10 m / 1.0 s | keep | 0.10 m is about v3's radial half-axis: past it the descent holds until the tip is back over the mouth |
| `contact_give_up_s`, his quiet miss | 120 s; 3.5 cm, 4 s, 5 mm | keep | A disc resting on the wall more than 3.5 cm off is lifted and retried |
| our contact radius K, co-motion radii | 0.025 m; 0.06 m | keep | The bore puts the magnet within ~5.5 mm |

Two lines to have typed and desk-checked in advance, used only if the first funnel flights show the need:
- the disc stops on the wall with the cable taut (the tip parked 1-5 cm above the seat and over 1 cm off centre while
  the body is at its bottom): `contact_dz:=-0.05`.
- v2 fitted and the misses fall sideways off its 72 mm half-axis: swap to v3 (re-measure W, X, Y).

## 4. Fallback: a flush steel oval (if the funnel cannot fly)

**What.** 0.8-1.0 mm mild steel, an oval ~200 x 140 mm (radial long), centred on plate 9, bolted on top of the ring's
skin through the same three M4 holes. It lies flush, so the recess goes away.

**Why.** It catches wherever the whole face lands on steel: about the oval's half-axes minus the magnet's corner (14 mm),
so ~8.6 x 5.6 cm. By the data that is 24 of 36 descents, 10 of 19 for the 5 Nov triangle. A 160 x 120 mm oval (6.6 x
4.6 cm) gives 17/36 (6/19).

**Costs.**
- Mass: ~0.14-0.16 kg at 0.8-0.9 mm (0.5 mm: ~0.09 kg).
- Thin sheet may saturate the magnet (G). Hang a known weight or use a spring scale on the bench. QUAD4's share of the
  ring is ~2.1 N static with four carriers.
- The catch lands up to ~8 cm off the typed weld point. Our contact radius and the co-motion loss and separation radii
  must grow to about 0.09 / 0.11 m. They are typed, but they are gate thresholds, so Wesley's word. The 30 Oct note says
  K over 0.040 needs comotion_lost and comotion_sep_xy at K + 0.021.
- The fold-in then plans QUAD4 at the typed point while its cable sits up to 8 cm away. Check it in the twin with an
  off-centre weld before the rig.
- Worse than the funnel on every count except height and occlusion. A spare to cut now only if steel sheet is at hand.

## 5. Swing gate in MATCH (code, only if needed)

His MATCH gate checks the tip's position but the BODY's speed. Te-Jen's own review of 3 Nov says so
(`tracking_errors`: tip position, body-minus-target velocity), and our fork shows it. A switch (default off) would gate
on the tip's speed over the plate instead, e.g. under ~0.05 m/s with the tip within 0.08 m for 0.5 s: a moment of low
pendulum energy. His code already computes the plate-frame tip speed for the quiet-miss rule. The expected gain is
small on the bare plate (A4: removing the swing entirely gives 3/36 within 1.9 cm, because the body's ~3 cm remains).
It would only push more descents into a funnel mouth that already takes 83-92 %. Before any code, read from the 5 Nov
bags how often the gate would open and whether a low-swing start stays low through the 4 s descent.

## 6-12, briefly

- **6. Pounce (G).** Hold the face 2-3 cm over the steel, predict the tip ~0.3 s ahead, descend at ~0.3 m/s when the
  prediction is within ~1 cm. The tip passes within 1.5 cm only ~5 % of the time (from N1's spread), at ~0.08 m/s, and
  the command delay is 45-65 ms. Untested, and a fast move at the ring. Future work, for the thesis' discussion.
- **7. Lower hover (D).** N4 shows no aiming gain. 0.30 -> 0.20 m saves ~1.25 s per descent at 0.08 m/s, 0.15 m saves
  ~1.9 s. With v3 keep at least 0.20 m (the rim is 8 cm above the weld point). Not worth a second change on Wed 11.
- **8. Slower near contact (G).** His contact descent has one rate (`descend_rate` 0.08 m/s, not typed on the rig
  path). His tiered ring descent has a slow LOW band but refuses contact_attach. N4 says slowing does not aim. With the
  funnel, a slower entry might stop rim bounce-outs. Build it only if B4 or the flights show them.
- **9. Trim (D).** The scatter has no mean to learn (N1) and his reference does not track the misses (N3). Leave it as
  flown. Te-Jen has the question (notes 5 Nov, question 1).
- **10. Link and holder (A).** The pendulum period is 2.04 s at 1.03 m and 1.42 s at 0.5 m. For slow body wander the
  swing scales about with length, so a 0.5 m cable roughly halves it. A4: within 1.9 cm 5/36 instead of 4/36. A rod
  hinged at both ends swings like the cable. A rod fixed to the airframe turns each degree of QUAD4's tilt into ~1.7 cm
  at the tip. A heavier holder changes neither the period nor a swing that follows the body (N2); it only cuts
  air-driven deflection. All of these change the attach geometry (Wesley's word), W and L, his MPC's cable length, the
  twin and the thesis' Setup 3. A 0.5 m cable also brings QUAD4 down level with the carriers' bodies.
- **11. Layout and downwash (D, A, G).** The square had one descent at plate height. At ~10-14 % per descent a catch
  in one is unremarkable (Wesley: "looked lucky"). At 60 deg elevation each carrier's body sits ~0.5 m outside its plate:
  ~0.64 m (triangle) or ~0.76 m (square) sideways of QUAD4 over the plate, and ~0.46 m below it at the approach. Their
  wakes come down outside the ring, not on the bob (the wake spread is G). N5 agrees: the same scatter far from the
  ring. The wash the bob does sit in is QUAD4's own, directly above it. N4 also shows the radial excess already 6-8 cm
  above the plate, so it is not made by the ring's surface. Keep the triangle (the plan).
- **12. QUAD4's hold (D, G).** QUAD4's body is a median 4.1 cm off his reference, the largest single term, and in phase
  with the swing (corr +0.44). An offline look at the spectrum of the body and the swing in MATCH from the bags would
  separate the two causes. A narrow peak near 0.4-0.5 Hz would be a controller-pendulum mode (his MPC's swing
  weighting; a question for Te-Jen). A broad spectrum would be air (the cage's recirculation, G). No flight change before
  hand-in.

## What the data support, and what is a guess

| Supported by the 4-5 Nov data or by arithmetic on it | Guesses, to check or leave |
|---|---|
| The scatter's size, its zero mean and its radial excess (N1, N7) | Why the swing is radial (a guess: QUAD4 arrives along plate 9's line and the stop excites it) |
| Body and swing each about half, in phase radially (N2) | What moves the body at ~0.5 Hz (his MPC with the pendulum, or the cage's air) |
| The landing point re-draws within ~1 s; the same scatter 6-8 cm up (N4) | Whether a low-swing start stays low through the descent (idea 5) |
| The ring and the carriers add little (N5); the square's catch fits chance | The carriers' wake spread |
| The bare plate catches within ~2 cm (N6, A1) | Body 24's visibility inside the cone; IR reflections from PETG |
| The funnel's mouth meets 20-22 of 24 descents (N10, geometry only) | The disc sliding to the seat, rim bounce-outs, snags on a miss (bench B4-B6) |
| Retries are no worse than first tries (N8) | The magnet's hold on thin sheet (idea 4) |
| The steel lies under the bore with ~8 mm to spare (A) | The cone's extra wash load; where the net drops land relative to plate 9 |

## Decisions for Wesley

1. Which funnel goes on for Wed 11: v3 (recommended, if B1 and the drop check pass) or v2.
2. Fit it before the day's first flight and keep it on plate 9 for every flight from Wed 11 (rel, nrtf, nro, fm9), so
   the paper's flights share one setup. Run the bench check (B0-B7, 30-45 min) in the setup time.
3. Cut a spare steel oval now (idea 4), or not.
4. Idea 5 and idea 8: decide after Wed 11's funnel flights, from how the descents met the mouth.

## Sources

- results/analysis/rig_joins_2026-11-04_05/ (summary.md, funnel_catch.md, joins.csv, funnel_catch.csv). N2-N9 and A4
  were recomputed from the CSVs for this note (scratch scripts, not kept).
- cad/funnel/funnel.py (P, VARIANTS, mouth()); docs/decisions.md 4-6 Nov; docs/rig/notes_for_tejen_2026-11-05.md;
  docs/rig/thu_2026-11-05.md (the typed lines); docs/rig/plan_sessions_2026-11-11_to_19.md.
- src/tejen_mission/tejen_mission/online_join_planner.py: `tracking_errors` (tip position, body velocity), MATCH and
  DESCEND_TO_ATTACHMENT guards, `contact_descent_hold`, the bottom hold, the rate profile (`descend_rate` 0.08);
  measured_tip_alignment.py (the trim: gain 0.8/s, filter 0.5 s, 0.025 m/s, 0.08 m cap, learning only in MATCH);
  launch/partner_mission.launch.py (what the rig types).
- Te-Jen's attach review of 3 Nov (~/Downloads/wesley-attach-review-20261103.zip, FINDINGS.md).
- Papers (docs/papers/): sugihara2023 (TRADY, drogue 5 cm radius for 2.5 cm errors, 35 deg, 13/15);
  shankar2020 (~16 cm gripper, abort-and-retry, 3-4 approaches per grasp indoors); gawel2017 (150-165 mm ferrous
  targets, compliant magnetic gripper, staged approach, 22/23); antal2026 (hook capture region, pendulum-model MPC on
  the hook, 27/27); miyazaki2018 (the wash is strongest straight under a rotor; a swinging cable-hung bar was hard to
  dock); zhang2026 (distributed iron keepers tolerate 27-34 mm misalignment).
