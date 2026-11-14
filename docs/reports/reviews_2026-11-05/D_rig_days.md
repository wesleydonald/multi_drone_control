# Review D: are the rig days Wed 11 / Thu 12 Nov ready? (read-only, 5 Nov 22:45-23:05)

**Sources read:** CLAUDE.md, both sheets, all scripts in `results/rig/2026-11-1{1,2}/scripts/`, the desk check
(`check_all.sh`, `evalscript.sh`, `check_2026-11-05_2150.txt` and the newer `_2256.txt`), the params files in
`configs/rig/`, `src/bringup/config/real.yaml`, feed F039-F052, decisions.md 3-5 Nov, registry RIG-1104-* / RIG-1105-*,
the joins summary and funnel_catch.md, notes for Te-Jen, `tools/rig_flight.sh`, `tools/mocap_stream_check.py`,
`tools/payload_geometry.py`, `tools/t1_four_markers.sh`, the funnel CAD parameters, reviews A and C in this folder.

**The sheets changed during this review** (both at 22:56-22:57; desk check re-run at 22:56, 16 OK on bf52a8a, 0 src files
dirty). C's sheet findings are now fixed: Wednesday's `rig_nrt.sh` (match_dwell 1.0) is deleted and no line names nrt;
Thursday's k1z table marks the 0.28 / 2.8 row; `--slot-gate` is back on the console line. Everything below was re-checked
against the 22:57 versions. Line numbers are the 22:57 files.

**Verdict:** every script matches its sheet and the latest decisions. The days would mostly go as written. What is
missing is in the order (the claim's re-flies and the release are placed where they will not be reached), in where
the landing checks sit, and in a few pre-flight checks that 4-5 Nov earned but no sheet line covers. Apart from A's
finding, nothing found here can break a quad by itself.

## Ranked by the cost on the day

| # | What | Kind | Cost if it bites | Cheap fix |
|---|---|---|---|---|
| 0 | QUAD4's tracker has no drop rule: a fast upset on the 55 deg lines can become a fleet EMERGENCY STOP in the air (review A) | cited, not re-derived | quads disarmed at ~2 m | A's fix (Wesley's word); the sheets have no line for flying without it |
| 1 | The first rig LAND under the three new LAND defaults is the claim's first flight, and the sheet's "watch the first landings / DISARM if a carrier presses or tips" sits in section 4b, after nrs and h4 | CONFIRMED (placement) + RISK | a tipped quad, an N2 miss on the claim | move 4b to "Before the first flight"; optionally fly one h4 before nrs (same square, same weld) |
| 2 | No runtime bring-up of the rig graphs on bf52a8a: the desk check is an offline launch walk only; the last fake-mocap bring-ups were on 4 Nov's tree | CONFIRMED (sheet says so) + RISK | the claim's first slot lost to a runtime fault in a rig-only path | bring nrs and k1z up on fake mocap once on the final tree before Wed |
| 3 | Thursday puts the claim's missing nrs flights after k1z, nrtf and nro, and its section 4 says "the square last", after the triangle lines and rel | CONFIRMED | the claim ends with 1-2 flights | if nrs < 3 on Wed, start Thursday on the square with nrs |
| 4 | rel (X1's real-world panel) is last on Wednesday with ~0 slack, then last again on Thursday; the 3 Nov rule put the release before the baselines | CONFIRMED (order) + RISK (time) | a missing figure panel for the headline | fly rel right after the re-hang (same triangle), then h3/f3/ls3 |
| 5 | The funnel: its fitting and W check are not in Thursday's time; no step takes it off before Wednesday's missing baselines and rel on the same plate 9; the release out of the funnel was never flown or simulated | CONFIRMED (sheet) + RISK | a lost flight; a baseline/rel row with an extra variable | budget ~15 min; "funnel off before line 4"; a hand pull-out check |
| 6 | No check that body 24 tracks QUAD4's magnet; nr9's start weld passed while 24 sat on QUAD2's magnet | CONFIRMED | a lost flight (as nr9) | a 30 s Motive check before each attach line |
| 7 | Wednesday's attach set expects body 6, which nrs does not use (sq8_2 flew without it); with no object in view the stream check says NOT OK | CONFIRMED | minutes of confusion | Wed attach set `8,11,12,13,14,24` |
| 8 | k1z (ring 1.2 m, drop_height 0.28) has never flown in the twin or on the rig; two changes against k1d | CONFIRMED (no registry row) + RISK | k1z x2 lost; k1.sh fallback exists | a twin run of k1z, or accept with the fallback |
| 9 | nro parks QUAD4's release 10 cm inside the fleet LAND limit (orbit centred 0.3 m toward plate 9) | CONFIRMED (code + nr9o_2) + RISK | one exploratory flight | press DETACH with the ring on the start side of its circle |
| 10 | Packs: ~17 flights on Wednesday, no rota or per-flight voltage step; 5 Nov had a battery LAND and 23.7 V packs | RISK | lost flights; weak-pack LAND | a pack line in "Before every flight" |
| 11 | Paperwork: no experiment card for the Nr claim; a "2 of 3" pass count against "no frozen bars"; Thursday's call 4 and l.122 still say the square's orbit twin run is not done (R3771 passed) | CONFIRMED | none on the day; the thesis/reviewer | a card or a waiver row; tidy the text |

## 1. Every flight line: script against sheet and decisions

Read on both sides (script and sheet) and against the 22:56 desk read-back. "OK" means CONFIRMED consistent.

| Line | Layout / plates (typed cw -> inside ccw) | Ring, cable, W/X/Y/L/K | Drop / loss / LAND / limit | Other | Verdict |
|---|---|---|---|---|---|
| nrs (Wed 1) | 330,60,150 + `reattach_plate:=8`, his `plate:=8`, carriers `[330,60,150]` (rig_nrs.sh:17,25,29-30) = square 11/2/5 + 8 (wed:82) | `attach_m1_1m_m2a_drop55_det15.yaml`: 0.86 kg, r 0.225; W 0.0491 L 1.034 K 0.025 X 0 Y 0.2293 (rig_nrs.sh:11, wed:60); his cable 1.0 | drop 55, loss 0.15, LAND from real.yaml:30-37 (no params file overrides), pos_limit 1.75 (desk) | takeoff gate /partner/release, match_dwell 0.5, contact_dz -0.03, QUAD4 2.20/2.38 m; diff against 5 Nov's rig_sq8.sh: only `load_traj:=hover` (the default) and the det15 file | OK. The tree differs from sq8_2's (see 3.4) |
| h4 / ls4 / f4 (Wed 2) | 330,60,150,240 -> 30,300,210,120 | ring_m2a_det15 / traj_hold_m2a_det15 (5 s hold), 0.75 m paths (real.yaml:41-42), 0.2 m/s typed | default drop 30, loss 0.15 | T1 q4 bodies = wed:65 | OK |
| h3 / f3 / ls3 (Wed 4) | 90,330,210 -> 270,30,150 (drone 0 on 3, wed:120) | as above | as above | T1 q3 bodies = wed:66 | OK |
| rel (Wed 5) | 90,330,210,270 (QUAD4 on 9) | m1_release_tip_m2a_det15: tip 0.04, loss 0.15 | default 30 | T4 topic `/drone_3/magnet` is real_io's coil topic (real_io_launch.py:182) | OK. C notes the rig tip check has no decisions.md line (C item 4) |
| nrs0 (optional) | as nrs | `..._noxy.yaml` int_k_xy 0 | as nrs | | OK (Wesley's word) |
| nrtf (Wed if time = Thu 2; the two copies identical) | 90,330,210, his plate 9 | as nrs | drop 55, loss 0.15 | match_dwell 0.5 (decisions.md:826); diff against 5 Nov's rig_nr9m.sh: only the run name | OK. "One variable against nr9m" holds for the arguments; the code tree differs (3.4) |
| k1z (Thu 1) | 90,330,210, his plate 9 | target_z 1.2, drop_height 0.28, max_body_z 2.8 (rig_k1z.sh:17,31); small object PC 0.038 PT 0.022 PS 0.07 | default 30 (as k1d), loss 0.15 | desk 22:56: QUAD4 2.56 / 2.56 / 2.76 m (within the pickup's 2.8 m, decisions.md:774) | OK (decisions.md:812). Thu table fixed at 22:56 (thu:19) |
| k1 (fallback) | as k1z at 1.0 m, drop_height 0.48 | | | | OK |
| nro (Thu 3) | triangle, orbit r 0.3 at 0.125, `contact_rig_orbit:=true` | orbit_det15: attach_pause, 5 s hold | drop 55, loss 0.15 | MD=0.5 (rig_nro.sh:14); the header comment still says "1.0 only if nrt caught" (rig_nro.sh:4): harmless, stale | OK |
| nros (Thu 3 alt.) | square, plate 8 | as nro | as nro | | OK; its sheet text is stale (finding 11) |
| h3rr (Thu 5) | triangle | `rod_len_reread: true` | | card bars frozen 31 Oct stay as written (CLAUDE.md) | OK |

Decisions checked: match_dwell 0.5 everywhere (no 1.0 script left, 22:56); only the small object (no k1ob script); the
pickup at 1.2 m / 0.28 / 2.8; the square for the claim; the 15 cm loss check on every line; the rig LAND defaults in
real.yaml:30-37 and no params file turns `land_descend_to_stall` off; `pos_limit_m` 1.75 on every tracker and the
fleet manager (desk); the takeoff gate on every floor-join line (desk). "No frozen bars": the sheets call N1-N3 / K1-K3
notes, but wed:99 and wed:103-104 still fix a pass count ("in at least 2 of the 3", "fewer than 2 of 3"); decisions.md
:822 says nothing is frozen before the flights. Low cost: say it is a reading note, or drop it.

## 2. The lessons of 4-5 Nov in the checklists

| Lesson | Where | Status |
|---|---|---|
| Never record in Motive | wed:63, thu:60 | Covered. The stream check cannot see a recording (decisions.md:798: "mocap_stream_check said OK"); the only symptom is a ~1 Hz lift bounce. The stop rule "a bounce that grows" (wed:156) could name the cause: "LAND, then check Motive is not recording". |
| <= 8 bodies at 240 Hz; stream check before the first flight | wed:64-66, per section (wed:108, 123, 144), thu:60 | Covered. Wed's attach set includes body 6, not used by nrs: see finding 7. |
| QUAD4 started on its plate (sq8, nr1r) | wed:86-87 (nrs), wed:72-73 (slot line every launch), console `--slot-gate` (wed:67) | Covered for nrs; nrtf/nro inherit it only through "Steps as Wednesday's nrs" (thu:104). |
| Body 6 centred, streamed only when needed | thu:63-71 | Covered on Thursday. Wednesday streams it when not needed (finding 7). |
| Body 24 not tracking another quad's magnet (nr9) | none | **Missing** (finding 6). wed:69 ("don't redefine 24") is a different lesson. |
| Fresh packs, 24.0 V at rest | wed:4, thu:4 | Stated once per sheet; no when/how (finding 10). |
| His TAKEOFF before our release (held) | wed:76-78 | Covered. |
| The 15 cm loss check | wed:19-20; every det15 file | Covered. |
| The T leaning 21-31 deg; the manual LAND at ~40 | wed:90-92, wed:156 | Covered for nrs. nros mentions the lean (thu:123) but the stop-rule exception (wed:156 "except nrs's T") is not extended to it. |
| The false touchdown that `land_rearm_touchdown` fixes | wed:133-141 (4b) | Explained, but placed after the first two lines' landings (finding 1). |
| Never LAND with the object held over the ring (k1m_3); operator OFF passes the gate | thu:84-85, thu:136-137 | Covered. |
| Only the small object (k1ob_2c) | thu:23-24 | Covered. |

## 3. Order, time, fallbacks

**3.1 Wednesday's time has no slack.** The table sums to 177 min (+7) from 8:00 (wed:40-45), i.e. 10:57-11:04. On
4-5 Nov an armed attach-day flight took ~11 min (5 Nov: 23 launches, 15 armed, 08:18-11:00; 4 Nov: 15 launches, 13
armed, 08:36-11:04; `launches.csv`), and one launch in three on 5 Nov never armed. The Motive set changes four times
(attach -> four-carrier -> three-carrier -> four-carrier), each with a stream check, and these are not in the minutes.
Whatever slips is rel (line 5).

**3.2 rel is last twice (finding 4).** rel is "X1's real-world partner" (wed:143, a figure panel; handin_plan "The
real-world release at 1 m"). On Wednesday it is after all twelve baselines; on Thursday it is the last of "Wednesday's
missing lines" (thu:50, thu:126). The 3 Nov ordering rule said "a missing release flies before the baselines"
(decisions.md:763); no later line reverses it. rel flies on the same triangle as h3/f3/ls3, with QUAD4 caught on 9:
flying it right after the re-hang costs nothing extra (the Motive set switches the same number of times).

**3.3 The claim's re-flies on Thursday (finding 3).** If Wednesday's nrs is short, Thursday has it at line 4 at
~9:25 (thu:50), after k1z, nrtf and nro, which are below the cut line (thu:3-4). Section 4 then says "Group by layout:
the triangle lines first (no re-hang), the square last" (thu:127), which puts nrs after h3/f3/ls3 and rel. thu:50 says
"in its priority: nrs ... the baselines, rel". The two disagree, and both put the claim after non-claim lines. Thursday
starts on the triangle (thu:46), so starting with nrs means setting up on the square and re-hanging once (the same
single re-hang thu:50 already counts). Also: the hand-in plan allows results on 18-19 Nov "only if clean"
(handin_plan.md:42, :139); thu:52 says "videos and the demo only". Which one holds decides whether a short claim can
be completed later.

**3.4 One tree.** All three nrs must name one SHA (wed:29-30, wed:105; CLAUDE.md). bf52a8a is committed and clean (desk
22:56). If A's fix goes in, it must be committed and the desk check re-run before Wednesday (wed:181 says so). If it
lands between Wednesday and Thursday, the claim spans two trees. nrtf's "one variable against nr9m" (thu:34) and nrs's
"as sq8_2" also carry the tree change: the takeoff hold (nr9m pressed his TAKEOFF 6 s before the release,
registry:3526), QUAD4's mux-gated checks and the LAND defaults. For exploratory lines this is fine. For nrs, the text
should say the claim flies a later tree than sq8_2.

**3.5 Re-hangs and the funnel.** Wednesday's square -> triangle re-hang is budgeted (wed:43). Thursday's funnel is not:
line 2 has 22 min for 1-2 flights (thu:48). The fitting (three M4 bolts from below into captured nuts,
cad/funnel/funnel.py:1-13), the seat and the W check (thu:100-103) are about 10-15 min. Nothing takes the funnel off
plate 9 before line 4. Wednesday's missing h3/f3/ls3 and rel would then fly with 27-40 g extra at plate 9 (SESSION_STATE:35,
:84), and rel would start with QUAD4's magnet in the funnel. Both are a second variable against Wednesday's flights
and the twin.

**3.6 Fallbacks.** Wednesday: a failed line moves on (wed:4); nrs after two misses becomes attach attempts (wed:48-50,
now naming nrtf only). Thursday: no funnel -> skip nrtf (thu:98); k1.sh is k1z's fallback (thu:77); nro only if nrtf
caught, else nros (thu:31-33). Thursday's call 4 still reads "the square's twin run is DONE: R3771" and also "(not yet
flown in simulation; the lab chat can queue it before Thursday)" (thu:31-33), and thu:122 says "only after its twin
run". R3771 passed and is marked "Thursday's nros ... is cleared by the twin" (registry:3599). On the day the condition
is just the re-hang.

**3.7 Anything a claim depends on after the cut line?** Yes, if Wednesday slips: (a) the missing nrs flights (the
only real-world claim) land on Thursday, after three below-the-cut lines; (b) the baselines (the real-world 3/4-carrier
rows and the "real-world figure-eight at 1 m" figure) move to Thursday's line 4 under Wednesday's fallback
(wed:48-50); (c) rel (X1's panel) is Thursday's last line; (d) R1 of the cable re-read needs Wednesday's h3 flights.
Nothing in the planned (no-slip) order is after the cut.

## 4. What could go wrong that no sheet line covers (ranked by cost)

1. **(Cited: review A, finding 1.)** QUAD4's tracker on the 55 deg lines. Neither sheet says what flies if the fix is
   not approved or built by Wednesday. The operator's ~40 deg LAND (wed:91) does not help against a 0.2 s upset
   (k1ob_2: 7 -> 54 deg in 0.2 s, registry:3524).

2. **The first rig LAND under the new defaults is the claim's.** `land_descend_to_stall`, `land_rearm_touchdown` and
   `carriers_down_alone` became rig defaults on 5 Nov (decisions.md:814, :820) and have never landed a real ring.
   Wesley chose "automatically tested when I do the first run" (decisions.md:814). Said once: decisions.md:814 itself
   notes the waiting LAND "has not flown under it". The card asked for a plain hover LAND first
   (docs/experiments/2026-11-05_carriers_down_alone.md:196-199: "Prerequisite. B's own rig flights ... on the first plain
   hover LAND"). The sheet's first flight is nrs. Its instructions for watching the landings (wed:139-141: the touchdown
   line, a carrier over +0.15 m = not down, "If a carrier presses or tips, DISARM") sit in section 4b, after nrs (x3)
   and h4/ls4/f4 (x6): nine landings before the operator reaches them (CONFIRMED). The cheapest alternative is one h4
   first. It flies the same square with QUAD4 welded on 8 (wed:108), is one of the six planned baseline flights, and
   costs one Motive tick (body 21 vs 6) and a T1. It also shows a Motive-recording bounce before the claim.
   (Risk: medium; cost: a tipped carrier or an N2 miss on the claim.)

3. **Runtime paths not brought up on the final tree.** wed:176-177: "Not exercised: anything the nodes check once
   running (his planner's TAKEOFF pre-checks, a solver build); the 4 Nov desk checks brought each line up on fake mocap
   for that." Since 4 Nov the rig graphs gained the takeoff gate (his planner), the mux subscription on QUAD4's tracker,
   the three LAND defaults and the operator topic. Review A read the code and the twin flew the logic (R3712-R3715,
   R3755-R3764), but not the rig launch with real_io, the mux and the magnet gate. A fault there shows up at 8:20 on
   Wednesday. One fake-mocap bring-up of rig_nrs.sh and rig_k1z.sh on the final tree (`tools/fake_mocap.py`, as on
   4 Nov), after the claim batch's freeze lifts, closes it. (Risk: low-medium; cost: the first slot.)

4. **Body 24 on another magnet (nr9, RIG-1104-nr9, registry:3402).** In nr9 the launch logged "drone 3 welded at start:
   plate at 267 deg cw (plate 9)" (results/rig/2026-11-04/nr9.log:77) while body 24 tracked QUAD2's magnet; the flight
   was lost. So the sheet's start checks (wed:86-87, wed:72) do not catch it (CONFIRMED). Body 24 was redefined after
   nr9 (decisions.md:776) and 5 Nov had no repeat, so the chance is lower now. Cheap check before each attach line:
   in Motive, lift QUAD4's magnet a few cm by hand and see body 24 move with it, not a carrier's. Possible cause (not
   verified): with 21-23 not streamed on attach lines, if they are disabled rather than unticked in Motive, their
   markers are free for body 24 to claim.

5. **The funnel, never flown.** (a) The release: at DETACH the coil goes OFF after the 3 s unload, then QUAD4 steps
   out and up in one move toward a point ~1.25 m out (dissipative_node.py:1820-1826). The 64 mm disc sits in a bore
   whose wall rises ~9 mm above it (bore_top 26 mm, disc seated at 17 mm; funnel.py P and funnel_catch.md) under a
   32-40 deg cone. It should ride out, but a snag would tug plate 9 outward on a three-carrier ring. (b) A disc jammed
   in the bore could lift the plate in his proof lift, and the co-motion proof (plate moves with the tip, normal
   within +0.04) would pass on a wedge instead of a magnet. This is the sheet's falsifier (wed:103) and is unlikely.
   (c) QUAD4 laid out 1.2 m away puts its cable over an 86 mm (v1) / 56 mm (v2) lip at the start. One hand check before
   nrtf covers all three: coil OFF, pull the magnet out at the step-out's angle and lay the cable as for the start.
   (Risk: low; cost: a flight.)

6. **k1z is a first.** The ring at 1.2 m has never flown on the rig, and no twin run of k1z exists (no registry row
   with drop_height 0.28 or target_z 1.2). Over the drop point QUAD4 at 2.56 m is ~0.30 m above the carriers'
   rotors (~2.24 m), against ~0.50 m in k1d. That is the attach's usual clearance, but now with the object hanging
   lower through the carriers' cables. The fallback k1.sh is ready (thu:77). A twin run of k1z (laptop, rig stack
   down, after the bench) would turn this into a known. (Risk: low-medium; cost: two K flights.)

7. **nro's release by the position limit.** The orbit's centre is the hover point plus the radius toward +y
   (dissipative_node.py:1875, :2746), i.e. toward plate 9. The release target is clipped 0.10 m inside the 1.75 m
   limit (utility_objects/position_limit.py:15). nr9o_2 released QUAD4 at (0.19, 1.65) (registry:3520), right on that
   clip, 10 cm from a fleet LAND, with QUAD4 0.2-6.2 cm off his reference in the descents (summary.md). Press DETACH
   when the ring is on its start side of the circle. (Risk: low; cost: one exploratory flight.)

8. **Packs.** ~17 flights on Wednesday with four quads, no rota, and no "check the pack before each launch" line.
   On 5 Nov fm9_2 flew 23.7 V packs (registry RIG-1105-fm9_2) and nr9o_2 ended in a battery LAND (registry:3520). The
   twin's weak-pack LANDs tipped a quad in all three runs (decisions.md:797). (Risk: medium for lost flights.)

9. **Hardware after 5 Nov.** The ring dropped four times on 5 Nov (fm9_2, fm9s, sq8_2, k1ob_2), two magnets were
   pulled off at ~16 N (k1ob_2c), and QUAD4 fell ~2.5 m (k1ob_2). No setup line inspects the magnet faces, cable ends,
   the net, the props or the markers on bodies 8 and 24. QUAD4 flew nr9m after the fall with its weld within a few mm,
   so this is a judgement, not a finding. (Risk: low.)

## 5. Wesley's open choices the sheets still need

1. Funnel v1 or v2 (Fri 6; the lab chat recommends v1 on geometry, funnel_catch.md). Also whether it goes on plate 8
   for the claim (wed:21-22; recommendation there: no).
2. Review A's QUAD4 drop-rule fix: yes/no. If yes: build after the claim batch's last chunk starts, commit, re-run the
   desk check before Wednesday. If no: what the 55 deg lines fly (no sheet line).
3. The baselines' loss check: 15 cm as scripted, or 6 cm (wed:19-20 asks; the non-det15 files exist).
4. Order (this review): rel before h3/f3/ls3 on Wednesday; on Thursday nrs first on the square if Wednesday left it
   short; one h4 before nrs as the first landing of the new defaults (or keep "tested on the first run",
   decisions.md:814).
5. 18-19 Nov: may a missing nrs flight fly there "if clean" (handin_plan.md:42) or not (thu:52)?
6. The Nr claim's card: write one (CLAUDE.md: a claim needs a card and a named baseline; none in docs/experiments/), or
   note the waiver in the registry rows.
7. Optional lines: nrs0 (wed:31-32), h3 + h3rr (thu:38), nros if nrtf misses (thu:31-33, its twin run is done).
8. From review C, still open: a decisions.md line for the rig's tip check on rel (C item 4).

## Not checked
No script, dry run, ROS launch or test was run (the laptop's bench from 00:30). Pose-age margins on the 15 cm check
were estimated from sq8_2's numbers only (0.6 s x 0.18 m/s = 11 cm; it holds unless a lag reaches ~0.8 s in a LAND).
