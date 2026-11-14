# Card: carriers down alone while the LAND waits for QUAD4

Written Thu 5 Nov 2026, ~14:40, before any run (an agent for the lab chat).
- **Rev 2 (~14:56)** after the critic ("build with changes"; its points and the response are at the end).
- **Rev 3 (~15:22)** for Wesley's combination (~15:15): `land_descend_to_stall` is the rig default now (real.yaml;
  the twin stays off), plus this switch with a 15 cm disarm band for the carriers. The rev 3 section at the end says
  what changed.

Exploratory: one run per arm and scenario (Wesley 30 Oct). Simulation bars are written now and judged after the runs
(Wesley 2 Nov). Rig bars are frozen before the first rig flight. Parent:
`docs/experiments/2026-10-17_land_ref_below_rest.md` (rev 4, its critic point 6, and the "Response").

## 1. Question

Wesley, 5 Nov: "when everything lands the quads attached to the ring stay floating around after being very close to
the ground and sometimes move towards each other/the ring, and they only disarm when quad4 is down as well."

Can each carrier be idled and disarmed on its own once the ring is down and it has stopped, within 15 cm of its rest,
instead of waiting for QUAD4 to land?

## 2. What the rig shows today (5 Nov; rev 4's offline section)

| | carriers airborne below 0.25 m, to LANDED | carriers' xy drift, to LANDED |
|---|---|---|
| flights that waited for a taken-back QUAD4 (k1d, nr9m, nr1r_3, nr9o_2, k1o_2, fm9y_2) | 6.5-8.6 s | 17-102 cm (k1d 47-66) |
| flights without the wait (fm9_4, fm9y; fm9_2, fm9s, k1ob_2 ended in a ring drop) | 0.3-2.6 s | 0.8-14 cm |

In all six waits the carriers were **in the air** (the critic's Q2). Their LAND reference stopped at z 0, and on the
rig they rode 0.15-0.23 m over it, 10-17 cm over their rests of 4.4-6.7 cm. The stall test then fired with them
airborne, and LANDED waited up to `LAND_DEPARTED_WAIT_S` (15 s) for QUAD4. Below `LAND_FOLLOW_XY_Z` (0.25 m) the xy
reference follows each drone, so nothing corrected the drift (deliberate: R0590).

`land_descend_to_stall` (B, the rig default since ~15:15) keeps the reference going down until the stall. Two things
are left during the wait:
- **A carrier that stops in the air.** Ground effect with the tracker's weak position stiffness can hold the highest
  carrier a few cm up. The stall then fires, the reference freezes 0.1-0.5 m below the floor, and the carrier hovers or
  presses until LANDED, up to 15 s (B's open risk, rev 4 critic point 6).
- **Carriers on the floor.** Within 3 cm of their rest they latch armed idle at zero throttle, with Betaflight's airmode
  running (card 2026-10-09_departed_disarm_alone).

In the twin (R3713, k1 on the orbit, the reference stopped at z 0, the unwind), each carrier latched armed idle before
the planner's touchdown line, and LANDED came 17.7 s after LAND.

**What this card can claim.** Under B, carriers that have stopped within 15 cm of their rest are idled and disarmed
during the wait, instead of hovering or idling armed for up to 15 s. The drop when the idle cuts the throttle is at
most 15 cm. That is inside the 9-16 cm Wesley accepted for the carriers at LANDED (decisions.md, 10 Oct, "Landing
drop"). It is not "the drift fixed": the drift is B's.

## 3. The one variable

`carriers_down_alone` (bool, default off). The launch passes it only when set (`common.yaml` '',
`real_mode.PLANNER_OPTIONAL` and `TRACKER_LAND`). It is read by:
- the planner;
- every tracker, QUAD4's too;
- the fleet manager, which listens to the carriers' own topic and needs no switch.

The carriers' band is `carriers_down_band_m`: the trackers' default is 0.15 m, typed only to change it; the launch and
the tracker refuse values outside 0.03-0.20 m.

**Planner.** It acts each tick of the LAND's wait for a departed drone (touchdown declared, `_departed_down()` still
False). It sends `/drone_d/carrier_released` True (latched; its own topic, not `/drone_d/departed`) to each carrier on
the load, once per LAND, when all of these hold:
- the touchdown came by the stall (a max-drop exit releases nobody);
- the ring is down (`_load_down()`);
- the ring and every carrier have been at most 0.05 m/s (`CARRIERS_STILL_MPS`), on poses fresher than 0.2 s, for
  0.5 s in a row (`CARRIERS_STILL_S`; any motion restarts the count);
- no take-back can still pull the ring (`_take_back_busy`): none in its `hold`, `guard` or `guard_follow` stage, and
  none whose release guard failed, until that take-back's own reference ends. While held, the planner says so once.

At LAND it creates the publishers and sends False, which clears a last flight's latched True.

**Tracker** (`_carrier_down_tick`, only with its switch on and the release received after this ARM). The carrier is
idled (if not already) and asks once to be disarmed alone when all of these hold for 0.2 s (`GROUND_IDLE_HOLD_S`):
- its reference at the floor;
- within -0.03..+`carriers_down_band_m` (0.15) m of its own resting height;
- its whole speed at most 0.05 m/s.

This works whether the carrier had already latched the ring-down idle (within 3 cm) or not (3-15 cm up, or in the air
in ground effect).

**Fleet manager.** `/drone_d/carrier_released` counts like `/drone_d/departed` for the disarm alone (only a True
received while flying; cleared at ARM). The carrier is disarmed alone by service and by radio, as an expected disarm.
Its line reads "Drone N is down at its rest on the landed ring (still on its plate): disarmed alone". A departed drone
reads "off the ring", exactly as before.

**Unchanged:**
- QUAD4's take-back and its own lone landing: the departed band (-3..+10 cm) and the committed code, tested tick by
  tick against a frozen copy, with this switch on and off;
- LANDED;
- anything where the LAND does not wait.

**Off.** Today's behaviour exactly:
- the planner's tick returns at once and nothing is published;
- the trackers do not subscribe;
- the manager's new subscription has no publisher.

**Needs `land_descend_to_stall`** (refused without it, by the launch and by the planner):
- On the rig that is now the default, so typing `carriers_down_alone:=true` alone is enough there. The twin's configs
  type both.
- Without it the stall came with the rig's carriers 0.15-0.23 m up (k1d). That is outside or at the edge of the band:
  some would hover as today, the others would be cut from up to 15 cm with the reference still at z 0.

**Code:**
- `planner_node.py`: `_carriers_down_tick`, `_take_back_busy`, `_carrier_release_pub`;
- `dissipative_node.py`: `_take_back_busy`;
- `tracker_node.py`: `carriers_down_alone`, `carriers_down_band_m`, `_carrier_released_cb`, `_carrier_down_tick`;
  the departed path is the committed code;
- `fleet_manager_node.py`: `_carrier_released_callback`, the line;
- `bringup`: `real_mode.py`, `graph_attach.py`, `common.yaml`.

**Tests** (all in `tools/gate.sh`):
- `src/mpc_planner/test/test_carriers_down_alone.py` (10)
- `src/dissipative_planner/test/test_carriers_take_back_busy.py` (5)
- `src/tracker/test/test_carrier_down_alone.py` (19):
  - +12 cm is idled and disarmed, latched or not;
  - +18 cm stays armed;
  - the band's edges, motion, the reference up, a typed band;
  - the departed band is the committed code with the switch on and off, QUAD4 at +12 cm staying armed.
- `src/fleet_manager/test/test_down_alone.py` (4 new)
- `src/bringup/test/test_real_mode_launches.py`: the switch reaches the planner and every tracker; the rig with the
  switch alone; the band typed and bounded.

## 4. Hypothesis

Where the LAND waits for QUAD4, D disarms each carrier within ~2 s of the planner's touchdown line: 0.5 s still, then
the trackers' 0.2 s hold. In the stronger-carrier pair this includes a carrier that stops off its rest, up to 15 cm. B
leaves them armed (idle or pressing) until LANDED. Where nothing waits, D's flight lines are B's.

## 5. Arms and scenarios (twin; lab PC; today's tree, worlds and links; M2A ring, 1 m links)

Both arms fly B on the rig LAND path: `land_descend_to_stall` on, `land_unwind` false, `land_rearm_touchdown` off, as
the rig flies. D adds `carriers_down_alone` with the default 15 cm band. The A arms of rev 2 (everything off) are
dropped: that is no longer the rig.

| scenario | base | B (`land_descend_to_stall`) | D: B + `carriers_down_alone` |
|---|---|---|---|
| S2 the k1d case on hover: QUAD4 alone picks up the object, drops it into the ring, LAND, QUAD4 taken back and landed alone | `k1_m2a_orbit_drop` (hover) | `cda_b_k1` | `cda_d_k1` |
| S2s the same with drone 1 stronger than its map by 0.03 (sim offset 0.155 against 0.185, as rev 4's S4): the band | `cda_b_k1` | `cda_b_k1s` | `cda_d_k1s` |
| S1 QUAD4 folded in: released on 9, his re-attach, LAND 10 s after the re-weld; nothing waits, so the switch must do nothing | `lr4a_nr9_u0` | `cda_b_nr9` | `cda_d_nr9` |
| S3 k1 on the orbit (r 0.3 m at 0.125 m/s, LAND during the orbit; the k1o case) | `k1_m2a_orbit_drop` | `cda_b_k1o` | `cda_d_k1o` |
| S4 LAND during his re-attach (LAND 24 s after the DETACH, ~7 s before the twin's re-weld): QUAD4 taken back (nr9m_4/5, nr1r_3, fm9y_2) | `lr4a_nr9_u0` | `cda_b_nr9tb` | `cda_d_nr9tb` |

- All 10 configs pass `tools/run_experiment.py --check`. A launch-graph read shows the switch reaching the planner and
  all four trackers in D, and none in B.
- If S4's LAND comes after a re-weld (the twin's timing moved), S4 becomes S1: re-time it, do not score it.
- **S2s and the band.** The twin's normal offsets (0.181-0.187) stall every carrier at its rest, so only S2s may put one
  off it. Under B the strong drone landed at its rest in rev 4 (R3726), but its max-drop clock ended 0.03 m short. S2s
  may show the max-drop exit instead: D then releases nobody (H1 not scored, recorded).

**Scored per carrier** (rest = its height at TAKEOFF, from the planner's line):
- the time from the planner's touchdown line to its disarm ("disarmed alone" or the fleet's LANDED);
- at that disarm: its height above rest, speed and tilt; its fall and contact speed if it was off its rest;
- its xy and tilt from its disarm to the end of the log;
- the ring's xy from the release line to LANDED;
- the touchdown exit (stall or max drop);
- any "held armed while ... take-back" line.

**Scored for QUAD4:** its take-back stages, its "is down" line (its departed band, unchanged), its contact speed; LAND
to LANDED; faults.

## 6. Bars (simulation: written before the runs, judged after them; D against the same scenario's B)

| # | bar | pass | fail |
|---|---|---|---|
| H1 | the carriers stop waiting | S2, S2s, S3, S4 with a stall exit: every carrier on the ring disarmed alone within 3.0 s of the touchdown line, or of the end of a take-back hold if one was logged, with QUAD4 not yet down. A max-drop exit is B's exit: not scored, recorded | any carrier within its band armed past that while QUAD4 is still landing |
| H2 | only within its band, only when nothing moves | at each carrier's disarm: within -3..+15 cm of its rest, speed at most 0.05 m/s, tilt at most 10 deg; the release line only after 0.5 s of the ring and every carrier still, and never during a take-back's hold, guard or guard_follow | any disarm outside that band, or a release with a carrier or the ring moving, or during those stages |
| H3 | nothing where nothing waits | S1: D has no "carriers_down_alone" line and no carrier disarmed alone; LANDED - ring touchdown within 1.0 s of B; each drone's height at LANDED within 1 cm of B's | any |
| H4 | QUAD4 and LANDED unchanged | S2-S4: the take-back as B's (lands alone, its "is down" line); LAND to LANDED within 2.0 s of B's; no "not down after 15 s" that B lacks | outside |
| H5 | no new faults, no tips, nothing dragged | no FLEET ABORT, EMERGENCY STOP, ENVELOPE FAULT, DROP AND LAND, UNANNOUNCED DETACH or POSITION LIMIT that B lacks; every carrier's tilt at most 30 deg and xy within 3 cm from its disarm to the end; the ring's xy within 2 cm from the release to LANDED | any |

**Falsifier (twin):** H1 fails in any waiting scenario with a stall exit, H2 or H5 fails anywhere, or H3 fails (the
switch acted where nothing waits).

**What it cannot say:**
- The twin's ground idle cuts the throttle at once, and the twin has no ground effect. S2s is the only arm that may put
  a carrier off its rest; on the rig the band is the test.
- The rig's slack cables and the twin's rigid rods differ near the floor (rev 4: the rod props a carrier on the rig
  LAND path). A carrier propped above +15 cm stays armed in both.
- `land_rearm_touchdown` off in the twin's rig path had false touchdowns in 2 of 4 (17 Oct, 0.55 m rods). A false
  touchdown with the ring down and a carrier still within 15 cm of its rest would disarm it there: a drop of at most
  15 cm, the band's.
- Whether disarming sooner matters on the rig (§8, F-rig).

## 7. Repeats and cost

Exploratory, one run per arm and scenario: 10 runs. In order:
1. `cda_b_k1`, `cda_d_k1`
2. `cda_b_k1s`, `cda_d_k1s`
3. `cda_b_nr9`, `cda_d_nr9`
4. `cda_b_k1o`, `cda_d_k1o`
5. `cda_b_nr9tb`, `cda_d_nr9tb`

No repeats unless this becomes a claim (then 2 per arm on one tree). If they are clean, Wesley's plan is to make
`carriers_down_alone` the rig default alongside `land_descend_to_stall` (not set now).

## 8. Rig: the first flight (after the twin, the reviewer and Wesley's word; bars frozen before the flight)

**Prerequisite.** B's own rig flights, now the default: rev 4's R1-R7 on the first plain hover LAND, with every
carrier's ground-idle line. That flight has no wait, so it is no comparison for this switch (critic CARD 4).

**The comparison this switch needs.** A rig flight with a wait (k1d or a LAND during the re-attach) under B, then the
same with `carriers_down_alone:=true` (on the rig, B is already on).

**F-rig, the falsifier of the switch's value** (the critic's Q2). Suppose that in B's flight with a wait every carrier
reaches its rest (within 3 cm) and stays put through the whole wait: xy within 5 cm, tilt at most 30 deg, no motor
twitch in the blackbox where read. Then D adds nothing on the rig, and the rule stands: record it and delete the code.
If a carrier stops off its rest (3-15 cm) or moves, D has its job.

| # | bar (the D flight) | pass |
|---|---|---|
| RD1 | carriers disarmed while QUAD4 lands | with a stall exit: every carrier within its band "disarmed alone" within 3.0 s of the planner's touchdown line (or of the end of a logged take-back hold), with QUAD4 still airborne. A max-drop exit is B's exit: not scored, recorded |
| RD2 | within the band | at each carrier's disarm: within -3..+15 cm of its height at TAKEOFF, speed at most 0.05 m/s (mocap), tilt at most 10 deg |
| RD3 | the drop and after | a carrier disarmed off its rest falls at most 15 cm and does not bounce over 30 deg; every carrier's tilt at most 30 deg and xy within 5 cm from its disarm to the end of the log, QUAD4's landing included |
| RD4 | the ring stays | the ring's xy within 3 cm from the release line to LANDED |
| RD5 | QUAD4 and the fleet as before | QUAD4 taken back and landed alone; LANDED; no EMERGENCY STOP, DROP AND LAND or fleet abort |

The claim of a pass is "carriers disarmed during the wait within 15 cm of their rest; nothing moved". The drift below
0.25 m is B's and is scored under B's card.

## 9. Risks (rev 3)

1. **The stall test watches only the highest drone.** The 0.5 s stillness hold over every carrier and the ring guards
   against "another carrier still descending".
2. **The 15 cm band now covers a carrier that stopped in the air** (ground effect). The release cuts its throttle and it
   falls up to 15 cm with its cable to a plate on the floor. That is a slack cable, and the drop Wesley accepted at
   LANDED. A carrier above 15 cm stays armed on B's frozen reference until LANDED (B's open risk, unchanged).
3. **A tethered carrier is disarmed on a resting ring.** Take-backs that could pull the ring hold the release (hold,
   guard, guard_follow, a failed guard). A ring drop after the release is not guarded. On the floor that is the same as
   armed idle at zero throttle, minus airmode.
4. **QUAD4 folded in counts as a carrier.** With its mux ours it is released and disarmed alone, within the carriers'
   band. A drone whose mux forwards the partner is refused by the manager.
5. **Max-drop exit: no release** (as today). B reached the max drop once on 1 m links (R3729).
6. **Latched topics.** The planner's `/drone_d/carrier_released` is latched. The LAND first sends False, and the
   trackers and the manager ignore a True from before their flight.

## Results (5 Nov; judged after the runs, Wesley 2 Nov)

**Runs.** R3755-R3764 on the lab PC (4 slots x 6 cpus, lab_1105_cda, synced 15:55, tree with rev 3). Every run exited
0 with a runner PASS. Scored with `tools/cda_metrics.py` (committed with this card), all times on the runner's sim
clock:
- the touchdown from the planner log (lift_progress freezes there);
- each disarm from run.csv's armed flags;
- the release line's wall stamp, mapped through the LAND, touchdown, disarm and LANDED anchors (~0.1 s);
- rest = each drone's median z before ARM.

| scenario | B / D | exit, carriers over rest at the touchdown | touchdown -> carriers disarmed (s) | D: release after touchdown (s); all still 0.5 s from (s) | D: at the disarm alone, dz (cm) / speed (m/s) / tilt (deg) | LAND -> LANDED (s), B / D | after each disarm: tilt max (deg) / xy max (cm), B / D |
|---|---|---|---|---|---|---|---|
| S2 k1d on hover | R3755 / R3756 | stall, +0.00 x3 / same | B 7.86 (LANDED) / D 0.64-0.66 alone | 0.43; 0.51 | +0.0..+0.2 / 0.000 / 0.7-5.9 | 19.06 / 18.74 | 4.8 / 0.0; 5.9 / 0.0 |
| S2s drone 1 stronger by 0.03 | R3757 / R3758 | **stall in the air, +0.77..+0.84 m (both arms)** | **B 12.82 / D 12.52: the fleet's LANDED, in the air** | 1.55; 1.56 (released at +0.77..+0.84 m; every tracker refused: outside its 15 cm band) | none alone | 18.82 / 18.72 | **179.4 / 51.3; 177.4 / 48.3 (all three fall ~0.8 m and flip)** |
| S1 QUAD4 folded in (nothing waits) | R3759 / R3760 | stall, +0.00..+0.03 / same | fleet at LANDED, both (no wait) | no release line | none alone | ring down -> LANDED 6.09 / 6.14 | 7.8 / 0.0; 10.6 / 0.0 |
| S3 k1o on the orbit | R3761 / R3762 | stall, +0.00 x3 / same | B 6.73 (LANDED) / D 0.65 alone | 0.44; 0.51 | +0.0..+0.2 / 0.000 / 0.6-4.2 | 18.04 / 17.86 | 4.9 / 0.0; 4.2 / 0.0 |
| S4 LAND during his re-attach (no re-weld before LAND: valid) | R3763 / R3764 | stall, +0.00 x3 / same | B 5.80 (LANDED) / D 0.60 alone | 0.41; 0.52 | +0.0..+0.2 / 0.000 / 0.6-4.6 | 16.90 / 16.70 | 5.9 / 0.0; 4.6 / 0.0 |

The release at 0.41-0.44 s against "still from 0.51-0.52 s": the carriers were already still at the touchdown (the
stall is 1 s of no descent). The planner counts its 5 still ticks from there on its own mocap. The 0.1 s gap is the
two clocks' and run.csv's 30 Hz.

| bar | result (judged after the runs) |
|---|---|
| H1 | **holds** in S2, S3, S4: every carrier disarmed alone 0.60-0.66 s after the touchdown, QUAD4 still landing (B: 5.80-7.86 s later, at LANDED). S2s: not scored; the touchdown came in the air with no carrier within its band (the card's "stall exit" did not foresee a stall in the air) |
| H2 | **holds**: every disarm alone within +0.0..+0.2 cm of rest, 0.000 m/s, tilt at most 5.9 deg; each release only after everything was still. S2s: the planner released carriers hovering +0.77..+0.84 m over their rests (it has no height test), and all three trackers held them armed (the band): no disarm outside the band |
| H3 | **holds** (S1): no release line, no disarm alone; ring down -> LANDED 6.09 / 6.14 s; heights at LANDED within 0.7 cm |
| H4 | **holds**: QUAD4 taken back and down in every take-back run; LAND -> LANDED within 0.1-0.3 s of B; no 15 s timeout |
| H5 | holds in S1-S4; **fails as written on S2s**: all three carriers flip (175-179 deg) and move 42-51 cm after the fleet's disarm at LANDED, in both arms alike (B 179.4 deg / 51.3 cm). No new fault anywhere; the POSITION LIMIT lines in S3 (QUAD4 at y +1.75 m in its take-back, "already landing") are in both arms |

**Verdict.** As written, the falsifier fires: H5 on S2s. That failure is not the switch's.
- **S2s.** In both arms the touchdown fired in the air, 1.0-1.2 s after the ring touched down. Every carrier was
  +0.77..+0.85 m over its rest, the reference froze 0.26-0.30 m below the ring-down anchors, and LANDED disarmed them
  there 12.5-12.8 s later. D released them, every tracker refused (the band), and the run ended as B's did.
- **The switch itself:**
  - where the LAND waited (S2, S3, S4), it disarmed every carrier 0.60-0.66 s after the touchdown, at its rest and
    still, instead of 5.8-7.9 s later at LANDED;
  - where nothing waited (S1) it did nothing;
  - it never disarmed a carrier in the air;
  - QUAD4's take-back and LANDED were unchanged.

**Rig findings first (from the twin, not yet seen on the rig).** S2s flies the rig's LAND settings: B as the rig
default, `land_rearm_touchdown` off, the rig LAND path. With one carrier 0.03 stronger than its map, the touchdown
fired in the air and LANDED cut three carriers at ~0.8 m.
- **The holding-up is partly the twin's.** Its rigid rods, with the xy held at the ring-down anchors above 0.25 m,
  stop a drone descending without moving out (rev 4's prop). The rig's cables would not hold it.
- **The trigger is the rig's.** With the re-arm off, the stall test has been armed since LAND. One second without
  descent right after the ring touches down then reads as a touchdown.
- Rev 4's same stronger carrier with the re-arm on (R3726, B; a hover LAND without QUAD4) landed every drone at its
  rest.

**Proposal:**
1. **The switch.** `carriers_down_alone` can become the rig default alongside `land_descend_to_stall`, as Wesley
   plans: it did its job in every waiting scenario and nothing else. It neither causes nor fixes S2s.
2. **Before B flies a wait on the rig, Wesley's word on one of:**
   - (a) `land_rearm_touchdown` on the rig. It is still waiting for its critic look at the wall-clock pose check.
   - (b) A touchdown guard: no stall counts while any carrier is more than ~0.25 m over its rest. This is a new
     variable, behind a switch.

   Twin check of (a), prepared and `--check`ed, not launched: `cda_b_k1s_rearm`, `cda_d_k1s_rearm` (S2s with the
   re-arm on).
3. **Optional, defence in depth for the switch.** The planner could release only carriers inside the band by its own
   rest records. Today the trackers alone refuse; S2s shows they do.
4. **On the rig, watch the planner's "touchdown by stall" line.** It logs each carrier's height over its rest. A
   touchdown with any carrier over +0.15 m is the S2s case.

## Critic

Run by the main chat, 5 Nov ~14:45. Verdict: **build with changes** (rewrite the card). As the main chat relayed it:

- CODE 1 (moderate): stillness is checked for one tick, then never again (planner_node.py ~1382-1393), and after the
  release the tracker's band tests only z and vz (tracker_node.py ~1430-1431). The one thing that pulls the ring after
  the release is the rig release guard: guard_follow steps QUAD4 out by GUARD_OUT_M as soon as _load_down()
  (dissipative_node.py ~2951-2961), the same window D releases the carriers. Hold the stillness ~0.5 s, and release
  only when no take-back is in its hold / guard / guard_follow stage.
- CODE 5 (low): the fleet manager says "is down at its rest off the ring: disarmed alone" for a carrier still on its
  plate (fleet_manager_node.py ~332): on a rig day that reads as a magnet that let go. Fix the text for a tethered
  carrier (a one-line manager change; update the card's "no fleet manager change").
- TEST 6 (low): a tracker test with the switch off that sends a late /departed flag from the detach mode (off ==
  today, shown, not argued).
- TEST 8: add src/mpc_planner/test/test_carriers_down_alone.py (and the tracker/manager tests if not already) to
  tools/gate.sh's list.
- CARD 2 (high for the evidence): no scenario can fail H2: every cda config flies thrust offsets 0.181-0.187, so
  carriers stall at rest; the band D relies on (+3..+10 cm) is untested. Add a D/B pair on k1 with one carrier 0.03
  stronger than its map (as rev 4's S4: A stopped it 26.6 cm up and it tipped).
- CARD 3 (moderate): a max-drop exit releases nobody (on 1 m links the backstop ~12.4 s vs the stall 11.0-11.9 s;
  R3729 hit it): RD1 must say a max-drop exit is B's exit, not scored.
- CARD 4 (moderate): the rig flight has no comparison (B's first rig flight is a plain hover with no wait), so the card
  claims "disarmed sooner", not "drift fixed".
- Q2 (framing): in all six 5 Nov wait flights the carriers were in the air; B fixes that; what D adds (armed idle ->
  disarmed, for up to 15 s) has never been seen to matter. A rig wait under B with the idle carriers staying put
  (< 5 cm, no tip) would show D adds nothing: put that falsifier in the card.

## Response (5 Nov ~14:56)

- **CODE 1.** Done.
  - The release needs the ring and every carrier still for 0.5 s in a row (`CARRIERS_STILL_S`); any motion restarts
    the count.
  - `_take_back_busy` (detach mode) holds it while any take-back is in hold, guard or guard_follow. Beyond the
    critic's list, it also holds after a failed guard until that take-back's reference ends: the tip may still be on
    plate 9 during the GUARD_OUT_M step, which starts at `_load_down()`.
  - The tracker's late path tests the drone's whole speed, not only vz.
  - Tests: `test_any_motion_restarts_the_hold`, `test_held_while_a_take_back_can_pull_the_ring`,
    `test_carriers_take_back_busy.py`, `test_not_outside_the_band_or_moving` (sliding).
- **CODE 5.** Done, two lines in the manager. During a LAND: "is down at its rest (on the landed ring, or off it):
  disarmed alone". Outside a LAND it is unchanged. The manager cannot tell a carrier from QUAD4 (a Bool flag), so the
  LAND wording covers both. Tests in `test_down_alone.py`.
- **TEST 6.** Done.
  - The trackers now carry the switch (passed only when set, through `tracker_drop_options` / `tracker_land_options`;
    QUAD4's tracker too), and the late path runs only with it on.
  - `test_off_a_late_flag_from_the_detach_mode_is_today`: four cases, flags at four ticks, compared tick by tick with a
    frozen copy of the committed `_departed_cb` and `_ground_idle_update`.
  - `test_on_the_same_late_flag_asks` shows that the comparison can fail.
  - The launch test checks every tracker gets the switch when set and none when unset.
- **TEST 8.** Done: `test_carriers_down_alone.py` added to `tools/gate.sh`. The tracker and dissipative test folders
  and `test_down_alone.py` were already in it.
- **CARD 2.** Done: S2s, `cda_b_k1s` / `cda_d_k1s` (drone 1 at sim offset 0.155). It may end in the max-drop exit under
  B (R3726 ended 0.03 m short); that is recorded under H1, not scored.
- **CARD 3.** Done: H1 and RD1 say a max-drop exit is B's exit, not scored, recorded.
- **CARD 4 and Q2.** Done:
  - §2 states that every 5 Nov wait had the carriers in the air (B's fix).
  - The claim is "disarmed sooner".
  - §8 asks for a B rig flight with a wait as the comparison and adds F-rig: if the idle carriers stay put there, D adds
    nothing and the code goes.

## Rev 3 (Wesley 5 Nov ~15:15; written ~15:22)

Wesley wants the combination: `land_descend_to_stall`, now the rig default (another agent set it in real.yaml and
deleted the slow descent), plus this switch with a 15 cm disarm height for the carriers. Changed:

- **The carriers' own band.**
  - `carriers_down_band_m` is a tracker parameter: 0.15 m by default, used only for carriers released by this switch,
    and refused outside 0.03-0.20 m.
  - The planner's release now goes on its own latched topic, `/drone_d/carrier_released`, not `/drone_d/departed`. The
    tracker therefore knows it is a carrier and applies its band, -0.03..+0.15 m, with the whole speed, for the
    existing 0.2 s dwell.
  - QUAD4's departed band for its own lone landing is untouched: the tracker's departed path is the committed code
    again (rev 2's `_late_down_alone_tick` / `_departed_after_idle` are gone).
  - The planner-side conditions are kept: the stall exit, the ring down, everything at most 0.05 m/s for 0.5 s, no
    take-back in hold, guard or guard_follow.
- **The manager.** It counts the carrier topic for the disarm alone and says "on the landed ring (still on its plate)".
  A departed drone reads "off the ring" exactly as before, so rev 2's LAND-dependent wording, and risk 7's off-case
  text change, are gone.
- **Configs.** Both arms fly B (rev 2's A arms, everything off, are deleted: no longer the rig). D's band is the
  default, not typed. The stronger-carrier pair stays: it is the arm that can test the band.
- **Tests.**
  - +12 cm idled and disarmed, latched or not; +18 cm stays armed; the edges.
  - QUAD4's band unchanged: the departed path compared tick by tick with the committed code, with the switch on and
    off; QUAD4 at +12 cm stays armed.
  - The manager's lines; the launch with the rig default and the band.
- **The launch test.** After the slow descent's deletion it now says what is meant: the rig's attach launch with
  `carriers_down_alone:=true` alone is valid (B by default). It is refused only with `land_descend_to_stall` typed
  false, and in the twin unless typed.

## Decision (Wesley 5 Nov ~16:25)

**`carriers_down_alone` is the rig default** alongside `land_descend_to_stall`, with the trackers' 15 cm band
(`carriers_down_band_m` 0.15 unless set). Set in `src/bringup/config/real.yaml`: every rig planner and every tracker
of the detach and attach graphs gets it (QUAD4's too); the carry has no departed drone, so its planner gets the switch
and does nothing with it. The twin keeps it off. A params_file can turn it off. Turning `land_descend_to_stall` off on
the rig now needs `carriers_down_alone: false` too: the launch refuses the switch without it and says so.

Basis: H1-H4 hold (R3755-R3764). Where the LAND waited for QUAD4, every carrier was disarmed alone 0.60-0.66 s after
the touchdown, within 0.2 cm of rest, instead of 5.8-7.9 s later at LANDED. Where nothing waited, it did nothing.

**With it, `land_rearm_touchdown` becomes the rig default too** (proposal 2(a) above; card
2026-10-17_touchdown_height_gate, its decision section). The S2s false touchdown in the air was the rig's LAND
settings with the re-arm off. Twin check on the rig settings with the re-arm on: `cda_b_k1s_rearm`, `cda_d_k1s_rearm`,
queued. Section 8's rig flight and F-rig stand as written.
